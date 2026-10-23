"""Tianyi's read-only dual-output state contract, testable without ROS or a robot."""

from __future__ import annotations

import ast
from concurrent.futures import ThreadPoolExecutor
import copy
import importlib.util
import json
import math
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
        self.context = kwargs.get("context")
        self.publishers = []

    def create_publisher(self, message_type, topic, qos):
        publisher = Publisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_node(self):
        return None


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
         "imu": copy.deepcopy(IMU if imu is None else imu), "length": {"length": 123.4}},
        {"pose": 1_792_052_530_000, "speed": 1_792_052_530_010,
         "imu": 1_792_052_530_050, "length": 1_792_052_530_060})


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

    def get_total_odometry(self, *, timeout):
        return self._read("length", {"length": 123.4}, timeout)


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
        self.assertEqual(0.3, state["v"])
        self.assertEqual(-0.4, state["w"])
        self.assertEqual(123.4, state["length"])
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
        self.assertEqual({"x": 1.2, "y": -2, "z": 0.1}, sample["pose"]["position"])
        self.assertEqual("odom", sample["pose"]["frame_id"])
        self.assertEqual("base_link", sample["pose"]["child_frame_id"])
        self.assertEqual(1_792_052_530_000, sample["pose"]["stamp_ms"])
        self.assertAlmostEqual(math.sin(1.57 / 2), sample["pose"]["orientation"]["z"])
        self.assertAlmostEqual(math.cos(1.57 / 2), sample["pose"]["orientation"]["w"])
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
        self.assertIsNone(STATE.odometry_sample(state)["pose"])
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

    def test_pose_remains_available_when_speed_request_fails(self):
        sample = STATE.odometry_sample(snapshot(speed={"error": "offline"}))
        self.assertIsNotNone(sample["pose"])
        self.assertEqual([None] * 6, sample["twist"])

    def test_quaternion_is_normalized_and_reconstructs_reported_rpy(self):
        for roll, pitch, yaw in ((0, 0, 0), (0, 0, math.pi / 2), (0.2, -0.3, 1.2)):
            state = snapshot()
            state["rpy"] = [roll, pitch, yaw]
            q = STATE.odometry_sample(state)["pose"]["orientation"]
            x, y, z, w = (q[axis] for axis in "xyzw")
            self.assertAlmostEqual(1, sum(value * value for value in q.values()))
            self.assertAlmostEqual(roll, math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)))
            self.assertAlmostEqual(pitch, math.asin(2 * (w*y - z*x)))
            self.assertAlmostEqual(yaw, math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))

    def test_invalid_pose_stays_unknown_instead_of_zero(self):
        for key in POSE:
            for invalid in (None, True, float("nan"), float("inf")):
                with self.subTest(key=key, value=invalid):
                    pose = dict(POSE, **{key: invalid})
                    state = STATE.locomotion_snapshot({"pose": pose, "speed": SPEED}, {})
                    self.assertIsNone(STATE.odometry_sample(state)["pose"])
                    json.dumps(state, allow_nan=False)


class DisplacementTests(unittest.TestCase):
    def test_relative_translation_is_not_total_mileage(self):
        tracker = STATE._DisplacementTracker()
        first = tracker.update(snapshot())
        self.assertEqual(0, first["displacement"]["distance"])
        second = snapshot()
        second["position"] = [4.2, 2, 0.1]
        second["length"] = 130
        second = tracker.update(second)
        self.assertEqual({"x": 3, "y": 4, "z": 0}, second["displacement"]["translation"])
        self.assertEqual(5, second["displacement"]["distance"])
        self.assertEqual(130, second["length"])
        returned = tracker.update(snapshot())
        self.assertEqual(0, returned["displacement"]["distance"])

    def test_missing_pose_does_not_set_or_replace_origin(self):
        tracker = STATE._DisplacementTracker()
        invalid = STATE.locomotion_snapshot({"pose": {"error": "offline"}, "speed": SPEED}, {})
        self.assertIsNone(tracker.update(invalid)["displacement"])
        self.assertIsNone(tracker._origin)
        tracker.update(snapshot())
        origin = copy.deepcopy(tracker._origin)
        self.assertIsNone(tracker.update(copy.deepcopy(invalid))["displacement"])
        self.assertEqual(origin, tracker._origin)
        recovered = snapshot()
        recovered["position"][0] += 1
        self.assertAlmostEqual(1, tracker.update(recovered)["displacement"]["distance"])

    def test_heading_wrap_and_pose_observation_timestamp(self):
        tracker = STATE._DisplacementTracker()
        first = snapshot()
        first["rpy"][2] = math.pi - 0.1
        tracker.update(first)
        current = snapshot()
        current["rpy"][2] = -math.pi + 0.1
        displacement = tracker.update(current)["displacement"]
        self.assertAlmostEqual(0.2, displacement["yaw"])
        self.assertEqual(current["sample_timestamps_ms"]["pose"], displacement["stamp_ms"])
        self.assertEqual("odom", displacement["frame_id"])


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
        self.assertEqual("unbounded", interface.pose_drift)
        self.assertTrue(interface.usable_for_control)
        self.assertFalse(info["available"])
        self.assertEqual({"state": "idle"}, state.dispatch("stop", {}))
        self.assertIsNone(state.dispatch("unsupported", {}))

    def test_concurrent_read_only_requests(self):
        client = Client(threading.Barrier(4))
        state = plugin(client)
        with ThreadPoolExecutor(max_workers=4) as pool:
            result = state._poll_once(pool)
        self.assertTrue(result["available"])
        self.assertEqual({("odopose", 0.05), ("speed", 0.05), ("imu", 0.05),
                          ("length", 0.05)}, set(client.calls))

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
            self.assertEqual(raw["position"], list(odom["pose"]["position"].values()))
            self.assertEqual(0, raw["displacement"]["distance"])
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

    def test_both_outputs_are_bridged_when_core_publishers_are_created(self):
        source = ast.parse((ROOT / "bridged_publisher.py").read_text())
        predicate = next(item for item in source.body if isinstance(item, ast.FunctionDef)
                         and item.name == "should_use_bridge")
        scope = {}
        exec(compile(ast.Module(body=[predicate], type_ignores=[]), "bridge", "exec"), scope)
        bridged = []
        transport = types.ModuleType("bridged_publisher")

        def create_publisher(node, message_type, topic, qos):
            bridged.append((node.context, topic))
            return Publisher()

        transport.create_bridged_publisher = create_publisher
        transport.create_bridged_subscription = lambda *args: None
        transport.should_use_bridge = scope["should_use_bridge"]
        transport.should_bridge_subscription = lambda topic: True
        node_module = types.ModuleType("rclpy.node")
        node_module.Node = Node
        qos_module = types.ModuleType("rclpy.qos")
        qos_module.QoSProfile = dict
        spec = importlib.util.spec_from_file_location("tianyi_bridge_integration_test",
                                                     ROOT / "bridge_integration.py")
        integration = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"rclpy.node": node_module, "rclpy.qos": qos_module,
                                      "bridged_publisher": transport}):
            spec.loader.exec_module(integration)
            core = object()
            ros = types.SimpleNamespace(ctx_core=core, executor_core=types.SimpleNamespace(add_node=lambda _: None))
            integration.enable(core)
            try:
                STATE.LocoStatePlugin({}, "robot", ros, "http://chassis")
                self.assertEqual([(core, "/robot/loco/state"), (core, "/robot/state/odom")], bridged)
                Node(context=object()).create_publisher(object, "/robot/state/odom", {})
                self.assertEqual(2, len(bridged))
            finally:
                integration.disable()


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

    def test_total_odometry_uses_the_documented_scalar_endpoint(self):
        client = STATE._LocoStateClient("http://chassis")
        with patch.object(STATE.urllib.request, "urlopen") as urlopen:
            response = urlopen.return_value.__enter__.return_value
            response.read.return_value = b"123.4"
            self.assertEqual({"length": 123.4}, client.get_total_odometry(timeout=0.2))
            self.assertEqual("http://chassis/api/core/statistics/v1/odometry",
                             urlopen.call_args.args[0].full_url)
            self.assertEqual("GET", urlopen.call_args.args[0].get_method())
            for body in (b"null", b"false", b"-1", b"NaN", b"[]", b"{}"):
                response.read.return_value = body
                self.assertIn("error", client.get_total_odometry(timeout=0.2))

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
