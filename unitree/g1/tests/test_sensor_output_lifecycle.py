"""Run real publication/lifecycle paths with local publishers, never hardware."""

import ast
from array import array
import json
import multiprocessing
from pathlib import Path
import queue
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from sensor_output import OutputGate
from camera_frame import (
    RGB_SCHEMA, DEPTH_SCHEMA, encode_envelope, compress_depth_payload,
    build_depth_image_metadata, decode_envelope,
)

G1 = Path(__file__).resolve().parents[1]


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")


def gate_worker(gate, connection):
    publisher = Publisher()
    while True:
        command = connection.recv()
        if command == "quit":
            return
        if command == "token":
            connection.send(gate.token())
        else:
            connection.send(gate.publish(command, publisher, "sample"))


def device_class(name, **extra):
    tree = ast.parse((G1 / "device.py").read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == name)
    scope = dict(globals(), Node=object, _Node=object,
                 RS_COLOR_W=1920, RS_COLOR_H=1080, RS_COLOR_FPS=15,
                 RS_DEPTH_W=640, RS_DEPTH_H=480, RS_DEPTH_FPS=15)
    scope.update(extra)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(G1 / "device.py"), "exec"), scope)
    return scope[name]


class OutputGateTests(unittest.TestCase):
    def test_shared_spawn_gate_and_old_epoch_rejected(self):
        ctx = multiprocessing.get_context("spawn")
        parent, child = ctx.Pipe()
        gate = OutputGate()
        process = ctx.Process(target=gate_worker, args=(gate, child))
        process.start()
        child.close()
        def request(value):
            parent.send(value)
            self.assertTrue(parent.poll(5))
            return parent.recv()
        try:
            token = request("token")
            self.assertTrue(request(token))
            self.assertTrue(gate.status()["ready"])
            gate.set_enabled(False)
            self.assertFalse(request(token))
            gate.set_enabled(True)
            self.assertFalse(gate.status()["ready"])
            self.assertFalse(request(token))
            self.assertTrue(request(request("token")))
        finally:
            parent.send("quit")
            process.join(5)
            if process.is_alive():
                process.terminate()
                process.join(2)
            parent.close()
        self.assertEqual(process.exitcode, 0)

    def test_stop_waits_for_publish_and_then_no_more_publish(self):
        gate = OutputGate()
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        publisher = SimpleNamespace(publish=lambda msg: (entered.set(), release.wait(1)))
        token = gate.token()
        publishing = threading.Thread(target=gate.publish, args=(token, publisher, "frame"))
        publishing.start()
        self.assertTrue(entered.wait(1))
        stopping = threading.Thread(target=lambda: (gate.set_enabled(False), stopped.set()))
        stopping.start()
        self.assertFalse(stopped.wait(.03))
        release.set()
        publishing.join(1)
        stopping.join(1)
        self.assertTrue(stopped.is_set())
        self.assertFalse(gate.publish(token, Publisher(), "late"))

    def test_timeout_is_not_idle_success(self):
        gate = OutputGate()
        gate._lock = mock.Mock()
        gate._lock.acquire.return_value = False
        with self.assertRaisesRegex(TimeoutError, "transition_timeout"):
            gate.set_enabled(False)
        self.assertEqual(gate._values[0], 1)


class CameraLifecycleTests(unittest.TestCase):
    def plugin(self, config=None):
        cache = mock.Mock()
        cls = device_class("RealSensePlugin", _CameraFrameNode=lambda topic: cache)
        plugin = cls(config or {}, "ubuntu", mock.Mock())
        plugin._proc = mock.Mock()
        plugin._proc.is_alive.return_value = True
        plugin._status = {"state": "running"}
        return plugin

    def test_tools_preserve_legacy_first_and_frame_overrides(self):
        plugin = self.plugin({"rgb_frame_topic": "/custom/rgb"})
        tools = {t["name"]: t for t in plugin.get_tools()}
        self.assertEqual(set(tools), {"camera_rgb", "camera_depth", "camera_distance"})
        self.assertEqual(tools["camera_rgb"]["topic_out"][0], {
            "topic": "/ubuntu/camera/rgb", "format": "image/jpeg"})
        frame = tools["camera_rgb"]["topic_out"][1]
        self.assertEqual((frame["port"], frame["topic"], frame["schema"]),
                         ("rgb_frame", "/custom/rgb", RGB_SCHEMA))
        for tool in self.plugin({"frame_enabled": False}).get_tools():
            self.assertEqual(len(tool["topic_out"]), 1)

    def test_cross_stop_and_capture_demand(self):
        plugin = self.plugin()
        for name in ("camera_rgb", "camera_depth"):
            gate = plugin._gates[name]
            old = gate.token()
            gate.publish(old, Publisher(), "frame")
            result = plugin.dispatch("stop", {"_tool_name": name})
            self.assertEqual(result["state"], "idle")
            self.assertEqual(plugin.dispatch("stop", {"_tool_name": name})["state"], "idle")
            self.assertFalse(gate.publish(old, Publisher(), "old"))
            plugin._proc.terminate.assert_not_called()
            self.assertIsNotNone(plugin._gates["camera_distance"].token())
        cache = plugin._frame_node
        def fresh_frame(*args):
            self.assertIsNotNone(plugin._gates["capture"].token())
            self.assertIsNone(plugin._gates["camera_rgb"].token())
            return {"data": b"jpeg"}, 1
        cache.wait_for_frame.side_effect = fresh_frame
        self.assertEqual(plugin.wait_for_color_frame()[1], 1)
        self.assertIsNone(plugin._gates["capture"].token())
        result = plugin.dispatch("start", {"_tool_name": "camera_rgb"})
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], "not_ready")
        self.assertIsNone(plugin._gates["camera_depth"].token())

    def test_stuck_publish_reports_error_without_killing_camera(self):
        plugin = self.plugin()
        with mock.patch.object(plugin._gates["camera_rgb"], "set_enabled",
                               side_effect=TimeoutError("blocked")):
            result = plugin.dispatch("stop", {"_tool_name": "camera_rgb"})
        self.assertEqual(result["state"], "error")
        plugin._proc.terminate.assert_not_called()

    def test_camera_restart_replaces_dead_worker_gates_and_old_health(self):
        plugin = self.plugin()
        old_gate = plugin._gates["camera_rgb"]
        old_gate.publish(old_gate.token(), Publisher(), "old")
        plugin._proc.is_alive.return_value = False
        proc = mock.Mock(pid=42)
        proc.is_alive.return_value = True
        ctx = multiprocessing.get_context("spawn")
        with mock.patch.object(ctx, "Process", return_value=proc), \
             mock.patch.dict(plugin._start_capture_process.__globals__, {"run_realsense_process": lambda: None}):
            plugin.start()
        self.assertIsNot(plugin._gates["camera_rgb"], old_gate)
        self.assertFalse(plugin._gates["camera_rgb"].status()["ready"])
        plugin._status_q.close()

    def test_concurrent_start_and_stop_are_serialized(self):
        plugin = self.plugin()
        responses = []
        def transition(action):
            responses.append(plugin.dispatch(action, {"_tool_name": "camera_rgb"}))
        threads = [threading.Thread(target=transition, args=(action,))
                   for action in ("stop", "start") * 5]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(2)
            self.assertFalse(thread.is_alive())
        self.assertEqual(len(responses), 10)
        self.assertTrue(all(r["state"] in {"idle", "not_ready"} for r in responses))
        plugin._proc.terminate.assert_not_called()

    def worker(self):
        # Execute the actual nested camera worker, replacing only SDK/ROS endpoints.
        import numpy as np
        gates = {n: OutputGate() for n in ("camera_rgb", "camera_depth", "camera_distance", "capture")}
        cls = device_class(
            "_RealSenseNode", gates=gates, np=np, Image=Message, CompressedImage=Message,
            UInt8MultiArray=Message, RS_JPEG_QUALITY=80,
            cv2=SimpleNamespace(IMWRITE_JPEG_QUALITY=1,
                                imencode=lambda *a: (True, np.frombuffer(b"\xff\xd8test\xff\xd9", dtype=np.uint8))),
            build_frame_metadata=lambda **kw: {"schema": kw["schema"], "image": kw["image"]},
        )
        node = cls.__new__(cls)
        node._worker_stop = threading.Event()
        node._diagnostics = {"rgb_frame_published": 0, "depth_frame_published": 0}
        node._record_error = mock.Mock(side_effect=lambda *args: self.fail(str(args)))
        for name in ("_color_pub", "_depth_pub", "_capture_pub", "_rgb_frame_pub", "_depth_frame_pub"):
            setattr(node, name, Publisher())
        return node, gates, np

    @staticmethod
    def one_frame_queue(node, item):
        q = queue.Queue()
        q.put(item)
        def get(**kwargs):
            try:
                return q.get_nowait()
            except queue.Empty:
                node._worker_stop.set()
                raise
        return SimpleNamespace(get=get)

    def test_real_color_loop_keeps_capture_when_rgb_stopped(self):
        node, gates, np = self.worker()
        old = gates["camera_rgb"].token()
        gates["camera_rgb"].set_enabled(False)
        node._color_q = self.one_frame_queue(node, (
            np.zeros((1, 1, 3)), None, None, 0, {}, 1, old, gates["capture"].token()))
        node._color_loop()
        self.assertEqual(len(node._capture_pub.messages), 1)
        self.assertFalse(node._color_pub.messages)
        self.assertFalse(node._rgb_frame_pub.messages)

    def test_real_color_loop_keeps_legacy_jpeg_and_passive_cache(self):
        node, gates, np = self.worker()
        gates["capture"].set_enabled(False)
        node._color_q = self.one_frame_queue(node, (
            np.zeros((1, 1, 3)), None, None, 0, {}, 1, gates["camera_rgb"].token(), None))
        node._color_loop()
        self.assertEqual(node._color_pub.messages[0].format, "jpeg")
        self.assertEqual(len(node._capture_pub.messages), 1)
        metadata, payload = decode_envelope(bytes(node._rgb_frame_pub.messages[0].data))
        self.assertEqual(payload, node._color_pub.messages[0].data)
        self.assertEqual(metadata["schema"], RGB_SCHEMA)

    def test_real_depth_loop_publishes_legacy_and_decodable_envelope(self):
        node, gates, np = self.worker()
        node._depth_q = self.one_frame_queue(node, (
            np.array([[1000]], dtype="<u2"), None, None, 0,
            {"depth_scale_m": .001}, 1, gates["camera_depth"].token()))
        node._depth_loop()
        self.assertEqual(node._depth_pub.messages[0].data, b"\xe8\x03")
        metadata, data = decode_envelope(bytes(node._depth_frame_pub.messages[0].data))
        self.assertEqual(metadata["schema"], DEPTH_SCHEMA)
        import zlib
        self.assertEqual(zlib.decompress(data), b"\xe8\x03")
        self.assertTrue(gates["camera_depth"].status()["ready"])

    def test_real_worker_drops_pre_stop_queue_after_restart(self):
        node, gates, np = self.worker()
        token = gates["camera_depth"].token()
        gates["camera_depth"].set_enabled(False)
        gates["camera_depth"].set_enabled(True)
        node._depth_q = self.one_frame_queue(node, (
            np.array([[1000]], dtype="<u2"), None, None, 0, {}, 1, token))
        node._depth_loop()
        self.assertFalse(node._depth_pub.messages)
        self.assertFalse(node._depth_frame_pub.messages)

    def test_capture_pauses_only_with_no_demands_and_resumes(self):
        node, gates, _ = self.worker()
        node._shutting_down = False
        node._pipeline = object()
        node._last_frame_monotonic = time.monotonic()
        node._stale_frame_timeout = 2.5
        node.stop_capture = mock.Mock()
        node.start_capture = mock.Mock()
        for g in gates.values():
            g.set_enabled(False)
        node._ensure_capture()
        node.stop_capture.assert_called_once_with(reconnecting=True)
        gates["capture"].set_enabled(True)
        node._pipeline = None
        node._ensure_capture()
        node.start_capture.assert_called_once()

    def test_passive_cache_info_does_not_clear_or_start_hardware(self):
        plugin = self.plugin()
        plugin._frame_node.wait_for_frame.return_value = ({"data": b"fresh"}, 3)
        self.assertEqual(plugin.wait_for_color_frame(timeout_s=0)[1], 3)
        plugin._frame_node.clear.assert_not_called()
        self.assertIsNone(plugin._gates["capture"].token())


class LidarLifecycleTests(unittest.TestCase):
    def test_real_imu_worker_keeps_publishing_with_cloud_disabled(self):
        from test_navigation_sensor_card import NavigationSensorCardContractTest
        module = NavigationSensorCardContractTest.load_bridge_module()
        class ImuMessage(Message):
            def __init__(self):
                super().__init__()
                self.orientation = SimpleNamespace(x=0., y=0., z=0., w=0.)
                self.angular_velocity = SimpleNamespace(x=1., y=2., z=3.)
                self.linear_acceleration = SimpleNamespace(x=0., y=0., z=9.8)
                self.orientation_covariance = [0.] * 9
                self.angular_velocity_covariance = [0.] * 9
                self.linear_acceleration_covariance = [0.] * 9
        node = module._NavigationSensorNode.__new__(module._NavigationSensorNode)
        node._gates = {"cloud": OutputGate(False), "imu": OutputGate()}
        node._stop = node._worker_stop = threading.Event()
        node._imu_queue = CameraLifecycleTests.one_frame_queue(node, (
            node._gates["imu"].token(), 10, ImuMessage()))
        node._sensor_rotation = module.validated_rotation_matrix(None)
        node._imu_frame = "livox_frame"
        node._set_stamp = lambda *args: None
        node._imu_pub = Publisher()
        node._counters = {"imu_published": 0}
        with mock.patch.object(module, "Imu", ImuMessage):
            node._imu_loop()
        self.assertEqual(node._counters["imu_published"], 1)
        self.assertEqual(node._imu_pub.messages[0].orientation_covariance[0], -1)
        self.assertTrue(node._gates["imu"].status()["ready"])

    def test_legacy_conversion_keeps_internal_safety_when_public_stopped(self):
        import types
        cls = device_class("_LidarNode", gravity_align_inplace=lambda data, *args: data)
        node = cls.__new__(cls)
        node.output_gate = OutputGate()
        token = node.output_gate.token()
        node.output_gate.set_enabled(False)
        node._worker_stop = node._closing = threading.Event()
        node._cloud_queue = CameraLifecycleTests.one_frame_queue(node, (12, 1, b"\0" * 12, 0, 0, token))
        node._safety_pub, node._cloud_pub = Publisher(), Publisher()
        node._worker_count = node._worker_total_ms = 0
        msgs = types.ModuleType("std_msgs.msg")
        msgs.UInt8MultiArray = Message
        with mock.patch.dict(sys.modules, {"std_msgs.msg": msgs}):
            node._process_loop()
        self.assertFalse(node._cloud_pub.messages)
        self.assertEqual(bytes(node._safety_pub.messages[0].data),
                         struct.pack("<II", 12, 1) + b"\0" * 12)
        source = (G1 / "safety_harness.py").read_text()
        self.assertIn('f"/{namespace}/lidar/cloud_internal", on_cloud', source)

    def test_lidar_tool_stop_controls_its_two_outputs_not_imu(self):
        from test_navigation_sensor_card import NavigationSensorCardContractTest
        module = NavigationSensorCardContractTest.load_bridge_module()
        nav = module.NavigationSensorPlugin.__new__(module.NavigationSensorPlugin)
        nav._lifecycle_lock = threading.RLock()
        nav._requested_outputs = {s: True for s in ("cloud", "imu")}
        nav._gates = {s: OutputGate() for s in nav._requested_outputs}
        nav._proc = mock.Mock(pid=123)
        nav._proc.is_alive.return_value = True
        nav._status_node = SimpleNamespace(
            cloud_topic="/ubuntu/navigation/lidar", lidar_frame="livox_frame",
            status=lambda running: {"ready": False, "blockers": ["imu_stale"]})
        cls = device_class("LidarPlugin")
        plugin = cls.__new__(cls)
        plugin._cloud_topic = "/ubuntu/lidar/cloud"
        plugin._node = SimpleNamespace(output_gate=OutputGate())
        plugin._navigation = nav
        plugin._lifecycle_lock = threading.RLock()
        tools = plugin.get_tools()
        self.assertEqual(tools[0]["name"], "lidar_cloud")
        self.assertEqual(tools[0]["topic_out"][0], {
            "topic": "/ubuntu/lidar/cloud", "format": "sensor/pointcloud"})
        result = plugin.dispatch("stop", {})
        self.assertEqual(result["state"], "idle")
        self.assertIsNone(plugin._node.output_gate.token())
        self.assertIsNone(nav._gates["cloud"].token())
        self.assertIsNotNone(nav._gates["imu"].token())
        nav._proc.terminate.assert_not_called()
        plugin.dispatch("start", {})
        nav._gates["cloud"].publish(nav._gates["cloud"].token(), Publisher(), "cloud")
        # Stale sibling IMU must not poison a fresh cloud output.
        self.assertTrue(nav.output_status("cloud")["ready"])

    def test_native_cloud_loop_rejects_queued_pre_stop_frame(self):
        from test_navigation_sensor_card import NavigationSensorCardContractTest
        module = NavigationSensorCardContractTest.load_bridge_module()
        node = module._NavigationSensorNode.__new__(module._NavigationSensorNode)
        node._gates = {s: OutputGate() for s in ("cloud", "imu")}
        token = node._gates["cloud"].token()
        node._gates["cloud"].set_enabled(False)
        node._gates["cloud"].set_enabled(True)
        node._stop = node._worker_stop = threading.Event()
        node._cloud_queue = CameraLifecycleTests.one_frame_queue(node, (
            token, 10, 1, 1, [], False, 12, 12, b"\0" * 12, True))
        node._sensor_rotation = None
        node._lidar_frame = "livox_frame"
        node._set_stamp = lambda *args: None
        node._cloud_pub = Publisher()
        node._counters = {"cloud_published": 0, "cloud_dropped": 0}
        with mock.patch.object(module, "unitree_mid360_to_navigation_cloud", return_value=b"\0" * 32), \
             mock.patch.object(module, "PointCloud2", Message), \
             mock.patch.object(module, "PointField", lambda **kwargs: kwargs):
            node._cloud_loop()
        self.assertFalse(node._cloud_pub.messages)
        self.assertEqual(node._counters["cloud_published"], 0)


if __name__ == "__main__":
    unittest.main()
