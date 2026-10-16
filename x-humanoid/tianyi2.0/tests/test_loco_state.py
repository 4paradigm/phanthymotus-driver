"""Tianyi's read-only dual-output state contract, testable without ROS or a robot."""

from __future__ import annotations

import ast
from concurrent.futures import ThreadPoolExecutor
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parents[1]))
from common.odom import axis, is_fresh, parse_interface
from common import lifecycle


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


class Node:
    def __init__(self, *args, **kwargs):
        self.publishers = []

    def create_publisher(self, message_type, topic, qos):
        publisher = Publisher()
        self.publishers.append(publisher)
        return publisher


def load_state():
    stubs = {name: types.ModuleType(name) for name in
             ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg")}
    stubs["rclpy.node"].Node = Node
    stubs["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    enum = types.SimpleNamespace(BEST_EFFORT=1, KEEP_LAST=1, VOLATILE=1)
    for name in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(stubs["rclpy.qos"], name, enum)
    stubs["std_msgs.msg"].String = types.SimpleNamespace
    spec = importlib.util.spec_from_file_location("tianyi_loco_test", ROOT / "loco_state.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


STATE = load_state()
POSE = {"x": 1.2, "y": -2, "z": 0.1, "roll": 0, "pitch": 0, "yaw": 1.57}
SPEED = {"vx": 0.3, "vy": 0, "omega": -0.4}
IMU = {"availibilityBitMap": 7, "timestamp": 1234,
       "quaternion": {"w": 1, "x": 0, "y": 0, "z": 0},
       "acc": {"x": 0, "y": 0, "z": 9.8},
       "gyro": {"x": 0.01, "y": 0.02, "z": -0.4}}


def snapshot(speed=None, imu=None):
    return STATE.locomotion_snapshot(
        {"pose": copy.deepcopy(POSE), "speed": copy.deepcopy(SPEED if speed is None else speed),
         "imu": copy.deepcopy(IMU if imu is None else imu)},
        {"pose": 1_792_052_530_000, "speed": 1_792_052_530_010, "imu": 1_792_052_530_050})


class Client:
    def __init__(self, barrier=None):
        self.calls = []
        self.barrier = barrier
        self.failed = False

    def _read(self, name, value, timeout):
        self.calls.append((name, timeout))
        if self.barrier:
            self.barrier.wait(timeout=1)
        return {"error": "offline"} if self.failed else copy.deepcopy(value)

    def get_odometry_pose(self, *, timeout):
        return self._read("odopose", POSE, timeout)

    def get_speed(self, *, timeout):
        return self._read("speed", SPEED, timeout)

    def get_imu(self, *, timeout):
        return self._read("imu", IMU, timeout)


def plugin(client=None):
    executor = types.SimpleNamespace(add_node=lambda node: None)
    ros = types.SimpleNamespace(ctx_core=object(), executor_core=executor)
    state = STATE.LocoStatePlugin({"request_timeout": 0.05}, "ubuntu", ros, "http://chassis")
    state._client = client or Client()
    return state


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    if not predicate():
        raise AssertionError("state worker did not produce the expected result")


class StateContractTests(unittest.TestCase):
    def test_measured_values_and_unavailable_r1_fields(self):
        state = snapshot()
        self.assertTrue(state["available"])
        self.assertEqual([1.2, -2, 0.1], state["position"])
        self.assertEqual([0.3, 0, None], state["velocity"])
        self.assertEqual(-0.4, state["yaw_speed"])
        self.assertEqual("moving", state["motion_status"])
        self.assertIsNone(state["mode"])
        self.assertIsNone(state["body_height"])
        self.assertEqual({}, state["errors"])

    def test_standard_sample_uses_body_twist_and_speed_arrival_time(self):
        sample = STATE.odometry_sample(snapshot())
        self.assertEqual("motus.odom/1", sample["schema"])
        self.assertEqual("body", sample["frame"])
        self.assertEqual([0.3, 0, None, None, None, -0.4], sample["twist"])
        self.assertEqual({"linear": "m/s", "angular": "rad/s", "length": "m"}, sample["units"])
        self.assertEqual(1_792_052_530_010, sample["stamp_ms"])
        self.assertEqual("received", sample["vendor"]["stamp_source"])
        self.assertIsNone(sample["pose"])
        self.assertNotIn("contact", sample)
        self.assertEqual([1.2, -2, 0.1], sample["vendor"]["position"])
        self.assertTrue(is_fresh(sample, 1_792_052_530_050, 100))
        self.assertFalse(is_fresh(sample, 1_792_052_531_050, 100))

    def test_partial_axes_keep_measured_zero_and_reject_non_finite_or_boolean(self):
        for bad in (None, True, float("nan"), float("inf"), "0.1"):
            with self.subTest(value=bad):
                sample = STATE.odometry_sample(snapshot(speed={"vx": 0, "vy": bad, "omega": 0.2}))
                self.assertEqual(0, axis(sample, "vx"))
                self.assertIsNone(axis(sample, "vy"))
                self.assertEqual(0.2, axis(sample, "wz"))
                json.dumps(sample, allow_nan=False)

    def test_timeouts_are_unknown_motion_instead_of_stationary(self):
        state = snapshot(speed={"error": "timed out"})
        self.assertFalse(state["available"])
        self.assertEqual("unknown", state["motion_status"])
        self.assertEqual("timed out", state["errors"]["speed"])
        sample = STATE.odometry_sample(state)
        self.assertEqual([None] * 6, sample["twist"])
        self.assertEqual(0, sample["vendor"]["samples"])

    def test_missing_pose_does_not_erase_valid_motion(self):
        state = STATE.locomotion_snapshot({"pose": {}, "speed": SPEED}, {"speed": 12345})
        self.assertIsNone(state["position"])
        self.assertFalse(state["available"])
        self.assertEqual(0.3, axis(STATE.odometry_sample(state), "vx"))

    def test_imu_bitmap_excludes_fields_even_when_vendor_includes_zeroes(self):
        imu = copy.deepcopy(IMU)
        imu["availibilityBitMap"] = 4
        state = snapshot(imu=imu)["imu"]
        self.assertTrue(state["available"])
        self.assertNotIn("quaternion", state)
        self.assertNotIn("accelerometer", state)
        self.assertEqual([0.01, 0.02, -0.4], state["gyroscope"])
        self.assertEqual(1234, state["uptime_ms"])

    def test_imu_failure_does_not_discard_valid_pose_and_velocity(self):
        state = snapshot(imu={"availibilityBitMap": 0, "gyro": IMU["gyro"]})
        self.assertTrue(state["available"])
        self.assertFalse(state["imu"]["available"])
        self.assertNotIn("gyroscope", state["imu"])

    def test_imu_euler_order_is_not_silently_relabelled(self):
        state = snapshot(imu={"availibilityBitMap": 512,
                              "euler_angle": {"x": 1, "y": 2, "z": 3}})
        self.assertEqual([1, 2, 3], state["imu"]["euler_angle"])
        self.assertNotIn("rpy", state["imu"])


class PluginTests(unittest.TestCase):
    def test_both_ports_and_declaration_before_first_sample(self):
        state = plugin()
        tool = state.get_tool()
        self.assertEqual({"type": "object", "properties": {}}, tool["inputSchema"])
        self.assertEqual([{"topic": "/ubuntu/loco/state", "format": "data/json"},
                          {"topic": "/ubuntu/state/odom", "format": "state/odom"}], tool["topic_out"])
        info = state.dispatch("info", {})
        interface = parse_interface(info["odom_interface"])
        self.assertEqual(("vx", "vy", "wz"), interface.provides)
        self.assertEqual("none", interface.pose_drift)
        self.assertTrue(interface.usable_for_control)
        self.assertFalse(info["available"])
        self.assertEqual({"state": "idle"}, state.dispatch("stop", {}))
        self.assertIsNone(state.dispatch("unsupported", {}))

    def test_concurrent_read_only_requests(self):
        client = Client(threading.Barrier(3))
        state = plugin(client)
        with ThreadPoolExecutor(max_workers=3) as pool:
            result = state._poll_once(pool)
        self.assertTrue(result["available"])
        self.assertEqual({("odopose", 0.05), ("speed", 0.05), ("imu", 0.05)}, set(client.calls))

    def test_publish_failure_recovery_and_repeated_start(self):
        client = Client()
        state = plugin(client)
        try:
            state.start()
            worker = state._thread
            state.start()
            self.assertIs(worker, state._thread)
            wait_for(lambda: len(state._odom_pub.messages) >= 1)
            raw, odom = state._pub.messages[0], state._odom_pub.messages[0]
            self.assertEqual(raw, {key: value for key, value in odom["vendor"].items()
                                   if key in raw})
            client.failed = True
            wait_for(lambda: state._odom_pub.messages[-1]["twist"] == [None] * 6)
            self.assertEqual("error", state.dispatch("info", {})["state"])
            self.assertIsNone(state._pub.messages[-1]["position"])
            client.failed = False
            wait_for(lambda: state._odom_pub.messages[-1]["twist"][0] == 0.3)
            count = len(state._odom_pub.messages)
            self.assertEqual({"state": "idle"}, state.dispatch("stop", {}))
            self.assertTrue(worker.is_alive())
            wait_for(lambda: len(state._odom_pub.messages) > count)
        finally:
            state.stop()
        self.assertFalse(state._thread.is_alive())
        self.assertEqual("idle", state.dispatch("info", {})["state"])
        self.assertEqual(2, len(state._node.publishers))

    def test_shutdown_restart_reuses_publishers(self):
        state = plugin()
        try:
            state.start()
            wait_for(lambda: len(state._odom_pub.messages) >= 1)
            state.stop()
            count = len(state._odom_pub.messages)
            state.dispatch("start", {})
            wait_for(lambda: len(state._odom_pub.messages) > count)
            self.assertEqual(2, len(state._node.publishers))
        finally:
            state.stop()

    def test_bundle_discovers_and_auto_starts_card(self):
        source = ast.parse((ROOT / "main.py").read_text())
        node = next(item for item in source.body if isinstance(item, ast.ClassDef)
                    and item.name == "TianyiDeviceBundle")
        namespace = {"DualDomainROS2": object, "_lifecycle": lifecycle, "os": os}
        exec(compile(ast.Module(body=[node], type_ignores=[]), "main.py", "exec"), namespace)
        ros = types.SimpleNamespace(ctx_core=object(), executor_core=types.SimpleNamespace(add_node=lambda _: None))
        with patch.dict(sys.modules, {"loco_state": STATE}), patch.dict(
                os.environ, {"SLAMTEC_URL": "http://chassis-override"}):
            bundle = namespace["TianyiDeviceBundle"](
                {"slamtec": {"base_url": "http://configured-chassis"},
                 "plugins": {"loco_state": {"enabled": True}}}, "robot", ros, object())
        try:
            self.assertEqual("http://chassis-override", bundle._plugins[0]._client._base)
            bundle._plugins[0]._client = Client()
            self.assertEqual(["loco_state"], [tool["name"] for tool in bundle.get_all_tools()])
            bundle.start_all()
            wait_for(lambda: bundle._plugins[0]._odom_pub.messages)
            self.assertEqual(2, len(bundle.dispatch("loco_state", {"action": "info"})["topic_out"]))
            self.assertEqual({"state": "idle"}, bundle.dispatch("loco_state", {"action": "stop"}))
            self.assertEqual(
                {"state": "error", "error": "Unknown action: unsupported (tool=loco_state)"},
                bundle.dispatch("loco_state", {"action": "unsupported"}))
        finally:
            bundle.stop_all()

    def test_loco_topic_uses_socket_bridge(self):
        source = ast.parse((ROOT / "bridged_publisher.py").read_text())
        function = next(item for item in source.body if isinstance(item, ast.FunctionDef)
                        and item.name == "should_use_bridge")
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), "bridge", "exec"), namespace)
        self.assertTrue(namespace["should_use_bridge"]("/robot/loco/state"))
        self.assertTrue(namespace["should_use_bridge"]("/robot/state/odom"))
        self.assertFalse(namespace["should_use_bridge"]("/arm/command"))


class ClientTests(unittest.TestCase):
    def test_api_paths_methods_and_request_deadlines(self):
        client = STATE._LocoStateClient("http://chassis/")
        with patch.object(STATE.urllib.request, "urlopen") as urlopen:
            urlopen.return_value.__enter__.return_value.read.return_value = b'{"vx":0}'
            for method, path in (
                (client.get_odometry_pose, "/api/core/slam/v1/localization/odopose"),
                (client.get_speed, "/api/core/motion/v1/speed"),
                (client.get_imu, "/api/core/slam/v1/imu"),
            ):
                self.assertEqual({"vx": 0}, method(timeout=0.2))
                request = urlopen.call_args.args[0]
                self.assertEqual("http://chassis" + path, request.full_url)
                self.assertEqual("GET", request.get_method())
                self.assertIsNone(request.data)
                self.assertEqual(0.2, urlopen.call_args.kwargs["timeout"])

    def test_bad_http_responses_become_current_errors(self):
        client = STATE._LocoStateClient("http://chassis")
        with patch.object(STATE.urllib.request, "urlopen") as urlopen:
            for body in (b"", b"broken-json", b"[]", b"false"):
                with self.subTest(body=body):
                    urlopen.return_value.__enter__.return_value.read.return_value = body
                    self.assertIn("error", client.get_speed(timeout=0.2))
            urlopen.side_effect = TimeoutError("timed out")
            self.assertEqual({"error": "timed out"}, client.get_speed(timeout=0.2))

    def test_existing_navigation_client_still_has_its_original_contract(self):
        spec = importlib.util.spec_from_file_location("tianyi_nav_test", ROOT / "nav_client.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        client = module.SlamtecClient("http://chassis")
        self.assertFalse(hasattr(client, "get_odometry_pose"))
        self.assertFalse(hasattr(client, "get_imu"))
        with patch.object(module.urllib.request, "urlopen") as urlopen:
            urlopen.return_value.__enter__.return_value.read.return_value = b'{"vx":0}'
            self.assertEqual({"vx": 0}, client.get_speed())
            self.assertEqual(5, urlopen.call_args.kwargs["timeout"])


if __name__ == "__main__":
    unittest.main()
