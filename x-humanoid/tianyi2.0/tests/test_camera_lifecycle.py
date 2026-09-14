"""Restart contract for Tianyi's head camera card.

The bug: ``CameraPlugin.stop()`` cleared ``_running`` and let the encode thread
die, but ``dispatch("start")`` only reported a state string and relied on
main.py's lazy start to re-arm the plugin — and that path fires once per
process, since a started plugin never leaves ``_started_plugins``. A stopped
camera therefore stayed dark for the rest of the container's life while ``info``
kept answering with a plausible-looking state, which makes any downstream
consumer (OCR, the dashboard card) look broken instead.

The restart rebuilds nothing but the encode thread; both ROS endpoints are
created once and kept. A second publisher on the topic would be a duplicate
socket_bridge connection. A recreated subscription measured worse than the
bandwidth it saves: on Tianyi, three quick stop→start cycles left it present in
the graph with the executor healthy (other domain-0 sensors kept publishing) and
the callback never firing again — the same silent-dead-stream this fixes.
"""

from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path

import pytest


# ── harness ───────────────────────────────────────────────────────────────────


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Subscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.destroyed = False


class _Node:
    """Records endpoint churn — what the restart must not produce."""

    def __init__(self, *args, **kwargs):
        self.publishers_created = []
        self.subscriptions_created = []

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher(topic)
        self.publishers_created.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _Subscription(topic, callback)
        self.subscriptions_created.append(sub)
        return sub

    def destroy_subscription(self, sub):
        sub.destroyed = True


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class _Ros2:
    def __init__(self):
        self.ctx_tianyi = object()
        self.ctx_core = object()
        self.executor_tianyi = _Executor()
        self.executor_core = _Executor()


def _install_vision_stubs():
    """numpy/cv2 as start() and _encode_loop import them, without the real ones."""
    numpy = types.ModuleType("numpy")
    numpy.uint8 = "uint8"
    numpy.frombuffer = lambda data, dtype=None: types.SimpleNamespace(
        reshape=lambda *shape: ("image", bytes(data)))

    cv2 = types.ModuleType("cv2")
    cv2.IMWRITE_JPEG_QUALITY = 1
    cv2.COLOR_RGB2BGR = 4
    cv2.cvtColor = lambda img, code: img
    cv2.imencode = lambda ext, img, params: (True, bytearray(b"jpeg-" + img[1]))

    sys.modules.update({"numpy": numpy, "cv2": cv2})


def _load_device_module():
    """Load device.py without requiring ROS 2 in the unit-test environment."""
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, **kwargs):
            pass

    class Enum:
        BEST_EFFORT = RELIABLE = KEEP_LAST = VOLATILE = 0

    rclpy.node.Node = _Node
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = Enum
    rclpy.qos.HistoryPolicy = Enum
    rclpy.qos.DurabilityPolicy = Enum
    sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy.node, "rclpy.qos": rclpy.qos})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray"):
        setattr(std_msgs.msg, name, type(name, (), {}))
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg})

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    for name in ("Image", "CompressedImage"):
        setattr(sensor_msgs.msg, name, type(name, (), {}))
    sys.modules.update({"sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs.msg})

    _install_vision_stubs()

    module = types.ModuleType("tianyi_device_camera_test")
    source = (Path(__file__).parents[1] / "device.py").read_text(encoding="utf-8")
    exec(compile(source, "device.py", "exec"), module.__dict__)
    return module


device = _load_device_module()


def _raw_frame(width=4, height=2, encoding="bgr8"):
    frame = types.SimpleNamespace()
    frame.width, frame.height, frame.encoding = width, height, encoding
    frame.data = b"\x01" * (width * height * 3)
    return frame


class _Camera:
    """A CameraPlugin plus the poking a lifecycle test needs."""

    def __init__(self):
        self.service_checks = []
        # The host Orbbec service is reached through nsenter; nothing in the
        # restart contract depends on it, so count the calls instead.
        self._real_ensure = device.CameraPlugin._ensure_orbbec_service
        device.CameraPlugin._ensure_orbbec_service = staticmethod(
            lambda: self.service_checks.append(True))
        self.plugin = device.CameraPlugin({}, "host", _Ros2())

    def close(self):
        device.CameraPlugin._ensure_orbbec_service = self._real_ensure
        self.plugin.stop()
        worker = self.plugin._encode_thread
        if worker is not None:
            # Join, or a leaked worker shows up in the next test's
            # threading.enumerate() and reads as a stacked thread.
            worker.join(timeout=2.0)

    def __getattr__(self, name):
        return getattr(self.plugin, name)

    def feed(self, frame=None):
        """Deliver one raw frame the way the domain-0 executor would."""
        assert self.plugin._subscription is not None, "no subscription to feed"
        self.plugin._subscription.callback(frame or _raw_frame())

    def wait_published(self, count, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.plugin._pub.published) >= count:
                return True
            time.sleep(0.005)
        return False


def _live_encode_loops():
    """Every live thread running _encode_loop, whatever plugin owns it."""
    return {t for t in threading.enumerate()
            if t.is_alive()
            and getattr(getattr(t, "_target", None), "__name__", "") == "_encode_loop"}


@pytest.fixture
def camera():
    handle = _Camera()
    try:
        yield handle
    finally:
        handle.close()


# ── tests ─────────────────────────────────────────────────────────────────────


def test_first_start_publishes_encoded_frames(camera):
    assert camera.plugin.dispatch("start", {}) == {"state": "running"}
    camera.feed()
    assert camera.wait_published(1), "first start did not publish an encoded frame"
    assert camera._pub.published[0].format == "jpeg"


def test_restart_after_stop_publishes_again(camera):
    """The reported failure: OCR restarts fine, the camera never comes back."""
    camera.plugin.dispatch("start", {})
    camera.feed()
    assert camera.wait_published(1)

    camera.plugin.dispatch("stop", {})
    assert camera.plugin.dispatch("info", {}) == {
        "state": "idle",
        "topic_out": [{"topic": "/host/camera/head", "format": "image/jpeg"}],
    }

    # dispatch("start") alone must re-arm: main.py's lazy start is a
    # once-per-process path and will not run a second time.
    assert camera.plugin.dispatch("start", {}) == {"state": "running"}
    assert camera.plugin.dispatch("info", {})["state"] == "running"
    before = len(camera._pub.published)
    camera.feed()
    assert camera.wait_published(before + 1), \
        "camera did not publish again after stop→start"


def test_restart_reuses_both_ros_endpoints(camera):
    camera.plugin.dispatch("start", {})
    first_sub = camera.plugin._subscription
    camera.plugin.dispatch("stop", {})
    assert not first_sub.destroyed, \
        "stop must not destroy a subscription a live executor is spinning"
    camera.plugin.dispatch("start", {})

    assert len(camera.plugin._pub_node.publishers_created) == 1
    assert len(camera.plugin._sub_node.subscriptions_created) == 1
    assert camera.plugin._subscription is first_sub


def test_many_quick_cycles_keep_publishing(camera):
    """The shape that wedged the stream on hardware: no pauses, no rebuilds."""
    camera.plugin.dispatch("start", {})
    for _ in range(5):
        camera.plugin.dispatch("stop", {})
        camera.plugin.dispatch("start", {})
    assert len(camera.plugin._sub_node.subscriptions_created) == 1
    assert len(camera.plugin._pub_node.publishers_created) == 1
    before = len(camera._pub.published)
    camera.feed()
    assert camera.wait_published(before + 1), \
        "stream did not survive repeated quick stop→start cycles"


def test_stopped_camera_drops_frames_and_ends_its_worker(camera):
    camera.plugin.dispatch("start", {})
    worker = camera.plugin._encode_thread
    camera.feed()
    assert camera.wait_published(1)

    camera.plugin.dispatch("stop", {})
    worker.join(timeout=2.0)
    assert not worker.is_alive(), "encode thread outlived stop"
    # The subscription stays live, so frames keep arriving; the callback is what
    # has to drop them. Nothing may reach the topic.
    camera.feed()
    assert camera.plugin._latest_frame is None
    assert len(camera._pub.published) == 1


def test_repeated_start_does_not_stack_workers_or_endpoints(camera):
    before = _live_encode_loops()
    for _ in range(3):
        camera.plugin.dispatch("start", {})
    assert len(camera.plugin._pub_node.publishers_created) == 1
    assert len(camera.plugin._sub_node.subscriptions_created) == 1
    # Counted as a delta: a stacked worker is unreachable from _encode_thread,
    # so only the live-thread set can catch it.
    assert len(_live_encode_loops() - before) == 1, \
        "a repeated start spawned a second encode thread"


def test_restart_rechecks_the_host_orbbec_service(camera):
    camera.plugin.dispatch("start", {})
    camera.plugin.dispatch("stop", {})
    camera.plugin.dispatch("start", {})
    assert len(camera.service_checks) == 2, \
        "restart must re-verify the host camera service, not assume the first " \
        "start's check still holds"


def test_stale_frame_is_not_published_after_restart(camera):
    """A frame captured before the stop is minutes old by the next start."""
    camera.plugin.dispatch("start", {})
    camera.plugin._running = False          # freeze the worker mid-stream
    camera.plugin._encode_thread.join(timeout=2.0)
    with camera.plugin._frame_lock:
        camera.plugin._latest_frame = _raw_frame()
    camera.plugin.dispatch("stop", {})
    camera.plugin.dispatch("start", {})
    time.sleep(0.1)
    assert camera._pub.published == [], \
        "a pre-stop frame was replayed after the restart"


def test_start_reports_error_when_the_vision_stack_is_missing(camera):
    # `from sensor_msgs.msg import ... CompressedImage` is the fragile part of
    # start()'s import block on a robot with a partial ROS install.
    msgs = sys.modules["sensor_msgs.msg"]
    compressed = msgs.CompressedImage
    del msgs.CompressedImage
    try:
        result = camera.plugin.dispatch("start", {})
    finally:
        msgs.CompressedImage = compressed
    assert result["state"] == "error", \
        "a failed start must not report a healthy state"
    assert not camera.plugin._running
