"""Restart contract for the Tianyi 2.0 head camera.

The bug these tests pin down: ``CameraPlugin.stop()`` cleared ``_running`` and
let the encode thread die, but ``dispatch("start")`` only reported a state and
relied on main.py's lazy start to re-arm the plugin — and that path fires once
per process. A stopped camera therefore stayed dark for the rest of the
container's life while ``info`` kept answering with a plausible-looking state,
which makes any downstream consumer (OCR, VLA, the dashboard card) look broken
instead.

The restart is deliberately asymmetric, so the tests check both halves:
the domain-42 publisher must be reused (it is a BridgedPublisher owning a Unix
socket to the socket_bridge process, and a second one for the same topic is the
duplicate connection its connection lock exists to prevent), while the
domain-0 subscription and the encode thread must be rebuilt.
"""

from __future__ import annotations

import importlib.util
import sys
import threading
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"


# ── stubs for the ROS/vision stack device.py imports ──────────────────────────


class _StubMessage:
    """Stand-in for any generated message type: attributes set freely."""

    def __init__(self, **fields):
        for key, value in fields.items():
            setattr(self, key, value)


class _StubQoSProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _StubEnum:
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"
    KEEP_LAST = "keep_last"
    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeSubscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.destroyed = False


class FakeNode:
    """Records endpoint churn — the thing the restart must not produce."""

    def __init__(self, name, context=None):
        self.name = name
        self.publishers_created = []
        self.subscriptions_created = []

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(topic)
        self.publishers_created.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = FakeSubscription(topic, callback)
        self.subscriptions_created.append(sub)
        return sub

    def destroy_subscription(self, sub):
        sub.destroyed = True

    def create_timer(self, period, callback):
        return types.SimpleNamespace(period=period, callback=callback)


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class FakeRos2:
    def __init__(self):
        self.ctx_tianyi = object()
        self.ctx_core = object()
        self.executor_tianyi = FakeExecutor()
        self.executor_core = FakeExecutor()


class FakeNumpy:
    @staticmethod
    def frombuffer(data, dtype=None):
        return types.SimpleNamespace(reshape=lambda *shape: ("image", bytes(data)))

    uint8 = "uint8"


class FakeCv2:
    IMWRITE_JPEG_QUALITY = 1
    COLOR_RGB2BGR = 4

    def __init__(self):
        self.encoded = 0

    def cvtColor(self, img, code):
        return img

    def imencode(self, ext, img, params):
        self.encoded += 1
        return True, bytearray(b"jpeg-" + img[1])


def _install_stub(name, module):
    if name not in sys.modules:
        sys.modules[name] = module
    return sys.modules[name]


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = _StubQoSProfile
    qos_mod.ReliabilityPolicy = _StubEnum
    qos_mod.HistoryPolicy = _StubEnum
    qos_mod.DurabilityPolicy = _StubEnum
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    _install_stub("rclpy", rclpy)
    _install_stub("rclpy.node", node_mod)
    _install_stub("rclpy.qos", qos_mod)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray", "UInt8MultiArray"):
        setattr(std_msgs_msg, name, type(name, (_StubMessage,), {}))
    std_msgs.msg = std_msgs_msg
    _install_stub("std_msgs", std_msgs)
    _install_stub("std_msgs.msg", std_msgs_msg)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    for name in ("Image", "CompressedImage", "Imu", "JointState", "PointCloud2",
                 "CameraInfo"):
        setattr(sensor_msgs_msg, name, type(name, (_StubMessage,), {}))
    sensor_msgs.msg = sensor_msgs_msg
    _install_stub("sensor_msgs", sensor_msgs)
    _install_stub("sensor_msgs.msg", sensor_msgs_msg)


_install_ros_stubs()
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


device = load("tianyi2_device", DRIVER / "device.py")


def _raw_frame(width=4, height=2, encoding="bgr8"):
    return _StubMessage(width=width, height=height, encoding=encoding,
                        data=b"\x01" * (width * height * 3))


class TianyiCameraLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.ros2 = FakeRos2()
        self.cv2 = FakeCv2()
        # The host Orbbec service is reached through nsenter; nothing about the
        # restart contract depends on it, so record the calls instead.
        self.service_calls = []
        self._real_ensure = device.CameraPlugin._ensure_orbbec_service
        device.CameraPlugin._ensure_orbbec_service = staticmethod(
            lambda: self.service_calls.append(True))
        self.plugin = device.CameraPlugin({}, "host", self.ros2)
        self.plugin._np = FakeNumpy()
        self.plugin._cv2 = self.cv2
        # start() re-imports these; keep it away from the real numpy/cv2.
        self._stub_vision_imports()
        self.addCleanup(self._cleanup)

    def _stub_vision_imports(self):
        _install_stub("numpy", types.ModuleType("numpy"))
        sys.modules["numpy"].frombuffer = FakeNumpy.frombuffer
        sys.modules["numpy"].uint8 = FakeNumpy.uint8
        cv2_mod = _install_stub("cv2", types.ModuleType("cv2"))
        cv2_mod.IMWRITE_JPEG_QUALITY = FakeCv2.IMWRITE_JPEG_QUALITY
        cv2_mod.COLOR_RGB2BGR = FakeCv2.COLOR_RGB2BGR
        cv2_mod.cvtColor = self.cv2.cvtColor
        cv2_mod.imencode = self.cv2.imencode

    def _cleanup(self):
        device.CameraPlugin._ensure_orbbec_service = self._real_ensure
        self.plugin.stop()
        worker = self.plugin._encode_thread
        if worker is not None:
            # Join here, or a leaked worker shows up in the next test's
            # threading.enumerate() and reads as a stacked thread.
            worker.join(timeout=2.0)

    # ── helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _live_encode_loops():
        """Every live thread running _encode_loop, whatever plugin owns it."""
        return {t for t in threading.enumerate()
                if t.is_alive()
                and getattr(getattr(t, "_target", None), "__name__", "") == "_encode_loop"}

    @property
    def live_subscription(self):
        return self.plugin._subscription

    def _feed(self, frame=None):
        """Deliver one raw frame the way the domain-0 executor would."""
        subscription = self.live_subscription
        self.assertIsNotNone(subscription, "no live subscription to feed")
        subscription.callback(frame or _raw_frame())

    def _wait_published(self, publisher, count, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(publisher.published) >= count:
                return True
            time.sleep(0.005)
        return False

    def _encode_threads_alive(self):
        return [t for t in (self.plugin._encode_thread,) if t is not None and t.is_alive()]

    # ── tests ────────────────────────────────────────────────────────────

    def test_first_start_publishes_encoded_frames(self):
        self.assertEqual({"state": "running"}, self.plugin.dispatch("start", {}))
        publisher = self.plugin._pub
        self._feed()
        self.assertTrue(self._wait_published(publisher, 1),
                        "first start did not publish an encoded frame")
        self.assertEqual("jpeg", publisher.published[0].format)

    def test_restart_after_stop_publishes_again(self):
        """The reported failure: OCR restarts fine, the camera never comes back."""
        self.plugin.dispatch("start", {})
        publisher = self.plugin._pub
        self._feed()
        self.assertTrue(self._wait_published(publisher, 1))

        self.plugin.dispatch("stop", {})
        self.assertEqual(
            {"state": "idle", "topic_out": [{"topic": "/host/camera/head",
                                             "format": "image/jpeg"}]},
            self.plugin.dispatch("info", {}))

        # dispatch("start") alone must re-arm: main.py's lazy start is a
        # once-per-process path and will not run a second time.
        self.assertEqual({"state": "running"}, self.plugin.dispatch("start", {}))
        self.assertEqual("running", self.plugin.dispatch("info", {})["state"])
        before = len(publisher.published)
        self._feed()
        self.assertTrue(self._wait_published(publisher, before + 1),
                        "camera did not publish again after stop→start")

    def test_restart_reuses_publisher_and_rebuilds_subscription(self):
        self.plugin.dispatch("start", {})
        first_sub = self.live_subscription
        self.plugin.dispatch("stop", {})
        self.assertTrue(first_sub.destroyed,
                        "stop must release the domain-0 raw-image subscription")
        self.plugin.dispatch("start", {})

        # One publisher for the process: a second BridgedPublisher on this topic
        # would open a duplicate socket_bridge connection.
        self.assertEqual(1, len(self.plugin._pub_node.publishers_created))
        self.assertEqual(2, len(self.plugin._sub_node.subscriptions_created))
        self.assertIsNot(first_sub, self.live_subscription)
        self.assertFalse(self.live_subscription.destroyed)

    def test_stopped_camera_drops_frames_and_ends_its_worker(self):
        self.plugin.dispatch("start", {})
        publisher = self.plugin._pub
        worker = self.plugin._encode_thread
        subscription = self.live_subscription
        self._feed()
        self.assertTrue(self._wait_published(publisher, 1))

        self.plugin.dispatch("stop", {})
        worker.join(timeout=2.0)
        self.assertFalse(worker.is_alive(), "encode thread outlived stop")
        # A late frame from the executor must not resurrect the stream.
        subscription.callback(_raw_frame())
        self.assertIsNone(self.plugin._latest_frame)
        self.assertEqual(1, len(publisher.published))

    def test_repeated_start_does_not_stack_workers_or_endpoints(self):
        before = self._live_encode_loops()
        for _ in range(3):
            self.plugin.dispatch("start", {})
        self.assertEqual(1, len(self.plugin._pub_node.publishers_created))
        self.assertEqual(1, len(self.plugin._sub_node.subscriptions_created))
        self.assertEqual(1, len(self._encode_threads_alive()))
        # Counted as a delta: a stacked worker is unreachable from
        # _encode_thread, so only the live-thread set can catch it.
        self.assertEqual(1, len(self._live_encode_loops() - before),
                         "a repeated start spawned a second encode thread")

    def test_restart_rechecks_the_host_orbbec_service(self):
        self.plugin.dispatch("start", {})
        self.plugin.dispatch("stop", {})
        self.plugin.dispatch("start", {})
        self.assertEqual(2, len(self.service_calls),
                         "restart must re-verify the host camera service, not "
                         "assume the first start's check still holds")

    def test_stale_frame_is_not_published_after_restart(self):
        """A frame captured before the stop is minutes old by the next start."""
        self.plugin.dispatch("start", {})
        publisher = self.plugin._pub
        self.plugin._running = False          # freeze the worker mid-stream
        self.plugin._encode_thread.join(timeout=2.0)
        with self.plugin._frame_lock:
            self.plugin._latest_frame = _raw_frame()
        self.plugin.dispatch("stop", {})
        self.plugin.dispatch("start", {})
        time.sleep(0.1)
        self.assertEqual([], publisher.published,
                         "a pre-stop frame was replayed after the restart")

    def test_start_reports_error_when_the_vision_stack_is_missing(self):
        # `from sensor_msgs.msg import ... CompressedImage` is the fragile part
        # of start()'s import block on a robot with a partial ROS install; drop
        # the name so the real ImportError path runs.
        msgs = sys.modules["sensor_msgs.msg"]
        compressed = msgs.CompressedImage
        del msgs.CompressedImage
        try:
            result = self.plugin.dispatch("start", {})
        finally:
            msgs.CompressedImage = compressed
        self.assertEqual("error", result["state"],
                         "a failed start must not report a healthy state")
        self.assertFalse(self.plugin._running)


if __name__ == "__main__":
    unittest.main()
