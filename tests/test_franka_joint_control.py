"""Franka joint-control contracts with a deterministic, non-hardware backend."""

from __future__ import annotations

import copy
import importlib.util
import math
import sys
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from common.vendor_runtime import DriverBundle

DRIVER = Path(__file__).resolve().parents[1] / "franka" / "fr3"


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


hardware = load_module("franka_control_test_hardware", "hardware.py")
discovery = load_module("franka_control_test_discovery", "discovery.py")
with mock.patch.dict(sys.modules, {"hardware": hardware, "discovery": discovery}):
    device = load_module("franka_control_test_device", "device.py")


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Joint-control condition did not become true before its deadline")


class FakeSession:
    """A shared physical arm: one active motion, explicit terminal events."""

    INITIAL_Q = [0.0, -0.4, 0.0, -1.5, 0.0, 1.5, 0.0]

    def __init__(self, address, **options):
        self.address = address
        self.closed = 0
        self.close_deadline = 0.5
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._motions = {}
        self._events = {}
        self._active = None
        self._latest = None
        self._owners = set()
        self.moves = []
        self.stops = []
        self.released = []
        self.stop_error = None
        self.complete_stop_immediately = True
        self.move_error = None
        self.move_entered = threading.Event()
        self.move_gate = None
        self.value = {
            "robot_ip": address, "state": "connected", "connected": True,
            "control_ready": True, "data_age_ms": 10, "error": None,
            "q": list(self.INITIAL_Q), "dq": [0.0] * 7, "tau_J": [0.0] * 7,
            "robot_mode": 1, "timestamp_ms": 1234567890,
        }

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self.value)

    @property
    def stopping(self):
        return self._stop.is_set()

    @property
    def stopped(self):
        return self.closed > 0

    def register_owner(self, owner):
        self._owners.add(owner)

    def begin_close(self):
        with self._lock:
            if self._active is not None:
                self.finish(self._active, "cancelled", reason="session_closed")

    def move(self, owner, target_rad, speed_rad_s, tolerance_rad=math.radians(0.5), timeout_s=120):
        self.move_entered.set()
        if self.move_gate is not None and not self.move_gate.wait(2):
            raise RuntimeError("fake submission was not released")
        with self._lock:
            if self.move_error is not None:
                raise self.move_error
            if owner not in self._owners:
                raise RuntimeError("Control card does not own this session")
            if self._active is not None:
                raise RuntimeError("another motion is active")
            action_id = f"fr3_fake_{self.address.replace('.', '_')}_{len(self.moves) + 1}"
            record = {
                "action_id": action_id, "owner": owner, "status": "running",
                "target_rad": list(target_rad), "speed_rad_s": speed_rad_s,
                "tolerance_rad": tolerance_rad, "timeout_s": timeout_s,
            }
            self.moves.append(copy.deepcopy(record))
            self._motions[action_id] = {
                "command_id": action_id, "owner": owner, "status": "running",
                "target": list(target_rad), "error": None,
            }
            self._events[action_id] = threading.Event()
            self._active = self._latest = action_id
            return action_id

    def wait(self, action_id, timeout=None):
        with self._lock:
            event = self._events[action_id]
        if not event.wait(timeout):
            raise TimeoutError("fake motion wait expired")
        return self.motion(action_id)

    def motion(self, command_id=None):
        with self._lock:
            key = command_id or self._latest
            if key is None:
                return {"status": "idle"}
            return copy.deepcopy({key: value for key, value in self._motions[key].items()
                                  if key != "owner"})

    def finish(self, action_id, status="succeeded", **result):
        with self._lock:
            record = self._motions[action_id]
            if record["status"] not in ("running", "stopping"):
                return
            record.update(status=status, **result)
            if status == "succeeded":
                self.value["q"] = list(record["target"])
            if self._active == action_id:
                self._active = None
            self._events[action_id].set()

    def stop_motion(self, owner=None, command_id=None):
        with self._lock:
            self.stops.append((owner, command_id))
            if self.stop_error is not None:
                raise self.stop_error
            selected = command_id or self._active
            if selected is None:
                return {"status": "idle"}
            record = self._motions[selected]
            if owner is not None and record["owner"] != owner:
                raise RuntimeError("This motion belongs to another control card")
            action_id = record["command_id"]
            if record["status"] in ("running", "stopping"):
                if self.complete_stop_immediately:
                    self.finish(action_id, "cancelled", reason="stop_requested")
                else:
                    record["status"] = "stopping"
            return self.motion(action_id)

    def release_owner(self, owner):
        self.released.append(owner)
        self._owners.discard(owner)
        with self._lock:
            if self._active is not None and self._motions[self._active]["owner"] == owner:
                return self.stop_motion(owner=owner)

    def close(self):
        self._stop.set()
        # Cleanup is deliberately independent of an injected stop failure.
        with self._lock:
            if self._active is not None:
                self.finish(self._active, "cancelled", reason="session_closed")
            self.closed += 1


class JointControlTests(unittest.TestCase):
    def setUp(self):
        self.module_patch = mock.patch.dict(sys.modules, {"hardware": hardware, "discovery": discovery})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.sessions = []
        self.publishers = mock.Mock(side_effect=AssertionError("Motion cards must not publish"))
        self.completions = []

        def session_factory(address, **options):
            session = FakeSession(address, **options)
            self.sessions.append(session)
            return session

        self.session_patch = mock.patch.object(hardware, "RobotSession", session_factory)
        self.session_patch.start()
        self.addCleanup(self.session_patch.stop)
        self.manager = hardware.SessionManager()
        self.addCleanup(self.manager.close)
        self.discovery = mock.Mock()
        self.discovery.snapshot.return_value = {"state": "ready", "robots": []}
        options = {
            "namespace": "test_franka", "manager": self.manager,
            "publisher_factory": self.publishers, "discovery": self.discovery,
        }
        self.sensor = device.FrankaPlugin({}, **options)
        self.controller = device.JointControlPlugin(
            {}, **options, acp_callback=lambda *args: self.completions.append(args),
        )
        self.addCleanup(self.controller.stop)
        self.addCleanup(self.sensor.stop)
        self.bundle = DriverBundle([self.sensor, self.controller])

    def call(self, action="info", instance_id="control-a", tool="joint_control", **args):
        return self.bundle.dispatch(tool, {"action": action, "instance_id": instance_id, **args})

    def start(self, instance_id="control-a", ip="172.16.0.3", **args):
        self.call("start", instance_id, robot_ip=ip, **args)
        return next(session for session in self.sessions if session.address == ip)

    def started_move(self, **args):
        session = self.start()
        result = self.call("set", joint1_deg=1, **args)
        self.assertEqual("running", result["state"])
        self.assertIn("action_id", result)
        return session, result["action_id"]

    def completed(self, action_id):
        wait_for(lambda: any(record[0] == action_id for record in self.completions))
        matching = [record for record in self.completions if record[0] == action_id]
        self.assertEqual(1, len(matching))
        return matching[0]

    def test_one_control_card_preserves_sensor_and_model_contracts(self):
        definitions = self.bundle.get_all_tools()
        self.assertEqual(3, len(definitions))
        tools = {item["name"]: item for item in definitions}
        self.assertEqual({"joint_states", "model", "joint_control"}, set(tools))
        spec = tools["joint_control"]
        self.assertEqual("actuator", spec["type"])
        self.assertTrue(spec["multiInstance"])
        schema = spec["inputSchema"]
        self.assertEqual({"set", "jog"}, set(schema["x-completion"]["actions"]))
        self.assertEqual({"action": "stopmotion"}, schema["x-hooks"]["on_interrupt_motion"])
        self.assertEqual("arm", schema["x-resource"])
        self.assertIs(True, schema["x-is-dangerous"])
        actions = {"set", "jog", "stopmotion", "info"}
        self.assertEqual(actions, set(schema["x-action-params"]))
        self.assertEqual(actions, set(schema["properties"]["action"]["enum"]))
        self.assertEqual(1, schema["properties"]["action"]["enum"].count("stopmotion"))
        self.assertNotIn("stop", schema["properties"]["action"]["enum"])
        self.assertNotIn("action_id", schema["properties"])
        self.assertEqual(["target_robot_ip"], schema["x-action-params"]["stopmotion"]["params"])
        self.assertFalse(spec.get("topic_out"))
        self.assertNotIn("robot_ip", spec["configSchema"].get("required", []))
        self.assertIn("robot_ip", tools["joint_states"]["configSchema"]["required"])
        self.assertEqual("string", schema["properties"]["output_topic"]["type"])
        config = spec["configSchema"]["properties"]
        self.assertEqual("instance", config["robot_ip"]["scope"])
        self.assertTrue(config["robot_ip"]["x-sensitive"])
        self.assertTrue(config["manual_robot_ip"]["x-sensitive"])
        self.assertEqual(5, config["speed_deg_s"]["default"])
        self.assertEqual(0.1, schema["properties"]["speed_deg_s"]["minimum"])
        self.assertEqual(10, schema["properties"]["speed_deg_s"]["maximum"])
        self.assertNotIn("minimum", schema["properties"]["step_deg"])
        self.assertNotIn("maximum", schema["properties"]["step_deg"])
        self.assertEqual(1, schema["properties"]["step_deg"]["default"])
        self.assertEqual("sensor", tools["joint_states"]["type"])
        self.assertEqual("resource", tools["model"]["type"])

    def test_schema_limits_leave_the_backend_margin_inside_all_seven_urdf_joints(self):
        schema = next(item for item in self.controller.get_tools() if item["name"] == "joint_control")["inputSchema"]
        root = ET.parse(DRIVER / "resource/fr3.urdf").getroot()
        for joint in root.findall("joint"):
            if joint.attrib.get("type") == "fixed":
                continue
            index = joint.attrib["name"].removeprefix("fr3_joint")
            limit = joint.find("limit")
            prop = schema["properties"][f"joint{index}_deg"]
            self.assertAlmostEqual(math.degrees(float(limit.attrib["lower"]) + 0.01), prop["minimum"], places=4)
            self.assertAlmostEqual(math.degrees(float(limit.attrib["upper"]) - 0.01), prop["maximum"], places=4)

    def test_canvas_topic_routes_each_control_card_to_its_own_ip(self):
        first = self.start("control-a", "172.16.0.2")
        second = self.start("control-b", "172.16.0.3")
        for key, session, degrees in (("control-a", first, 1), ("control-b", second, 2)):
            topic = self.controller._topic(key)[0]["topic"]
            result = self.bundle.dispatch("joint_control", {
                "action": "jog", "output_topic": topic, "joint": 1, "step_deg": degrees,
            })
            self.assertEqual(("joint_control", key), session.moves[-1]["owner"])
            self.assertAlmostEqual(math.radians(degrees), session.moves[-1]["target_rad"][0])
            info = self.bundle.dispatch("joint_control", {"action": "info", "output_topic": topic})
            self.assertEqual(session.address, info["robot_ip"])
            session.finish(result["action_id"])
            self.completed(result["action_id"])
        self.assertEqual(1, len(first.moves))
        self.assertEqual(1, len(second.moves))

    def test_canvas_stop_only_stops_the_card_bound_to_its_output_port(self):
        first = self.start("control-a", "172.16.0.2")
        second = self.start("control-b", "172.16.0.3")
        a = self.call("set", "control-a", joint1_deg=1)
        b = self.call("set", "control-b", joint1_deg=1)
        topic = self.controller._topic("control-b")[0]["topic"]
        result = self.bundle.dispatch("joint_control", {"action": "stopmotion", "output_topic": topic})
        self.assertEqual("ready", result["state"])
        self.assertEqual("cancelled", self.completed(b["action_id"])[1])
        self.assertEqual([], first.stops)
        self.assertEqual("running", first.motion()["status"])
        self.assertEqual([(None, None)], second.stops)
        first.finish(a["action_id"])
        self.completed(a["action_id"])

    def test_missing_unknown_or_conflicting_card_binding_never_selects_an_arm(self):
        first = self.start("control-a", "172.16.0.2")
        second = self.start("control-b", "172.16.0.3")
        topic = self.controller._topic("control-b")[0]["topic"]
        bindings = [{}, {"output_topic": ""}, {"output_topic": 42},
                    {"output_topic": topic + "/stale"},
                    {"instance_id": "control-a", "output_topic": topic}]
        for action in ("set", "jog", "stopmotion", "Stop"):
            for binding in bindings:
                if action == "stopmotion" and not binding:
                    continue  # Driver-wide interrupt hook explicitly stops all arms.
                with self.subTest(action=action, binding=binding), self.assertRaises(ValueError):
                    self.bundle.dispatch("joint_control", {"action": action, **binding,
                        "joint1_deg": 1, "joint": 1, "step_deg": 1})
        for session in (first, second):
            self.assertEqual([], session.moves)
            self.assertEqual([], session.stops)

    def test_topic_binding_does_not_bypass_start_or_required_robot_ip(self):
        with self.assertRaisesRegex(ValueError, "robot_ip is required"):
            self.call("start")
        self.assertEqual([], self.sessions)
        self.call("config", robot_ip="172.16.0.3")
        topic = self.controller._topic("control-a")[0]["topic"]
        with self.assertRaisesRegex(RuntimeError, "Start this joint_control card"):
            self.bundle.dispatch("joint_control", {"action": "set", "output_topic": topic, "joint1_deg": 1})
        self.assertEqual([], self.sessions)

    def test_rejected_configuration_blocks_topic_motion_using_old_ip(self):
        session = self.start()
        topic = self.controller._topic("control-a")[0]["topic"]
        for ip in ("172.16.0.2", "invalid-ip"):
            with self.subTest(ip=ip):
                with self.assertRaises(ValueError):
                    self.call("config", robot_ip=ip)
                self.assertFalse(self.call()["control_ready"])
                with self.assertRaisesRegex(RuntimeError, "rejected configuration"):
                    self.bundle.dispatch("joint_control", {
                        "action": "set", "output_topic": topic, "joint1_deg": 1})
                self.assertEqual([], session.moves)
                self.call("config", robot_ip="172.16.0.3")
                self.assertTrue(self.call()["control_ready"])
        self.assertEqual(1, len(self.sessions))

    def test_set_converts_units_and_omitted_joints_keep_actual_positions(self):
        session = self.start(speed_deg_s=2)
        original = session.snapshot()["q"]
        result = self.call("set", joint1_deg=10, joint3_deg=-20)
        command = session.moves[0]
        expected = list(original)
        expected[0], expected[2] = math.radians(10), math.radians(-20)
        self.assertEqual(expected, command["target_rad"])
        self.assertAlmostEqual(math.radians(2), command["speed_rad_s"])
        self.assertEqual(("joint_control", "control-a"), command["owner"])
        self.assertAlmostEqual(math.radians(0.5), command["tolerance_rad"])
        self.assertEqual(120, command["timeout_s"])
        self.assertEqual(result["action_id"], command["action_id"])

    def test_long_slow_motion_is_rejected_before_backend_submission(self):
        session = self.start()
        with self.assertRaisesRegex(ValueError, "120"):
            self.call("set", joint1_deg=100, speed_deg_s=0.1)
        self.assertEqual(session.moves, [])

    def test_canvas_jog_accepts_large_signed_steps_for_each_joint(self):
        session = self.start()
        topic = self.controller._topic("control-a")[0]["topic"]
        for joint in range(1, 8):
            for step in (15, -15):
                with self.subTest(joint=joint, step=step):
                    expected = session.snapshot()["q"]
                    expected[joint - 1] += math.radians(step)
                    result = self.bundle.dispatch("joint_control", {
                        "action": "jog", "output_topic": topic,
                        "joint": str(joint), "step_deg": step,
                    })
                    self.assertEqual(expected, session.moves[-1]["target_rad"])
                    session.finish(result["action_id"])
                    self.completed(result["action_id"])
        self.assertEqual(14, len(session.moves))

    def test_invalid_requests_never_reach_backend(self):
        session = self.start()
        invalid = [
            ("set", {}), ("set", {"joint1_deg": True}),
            ("set", {"joint1_deg": float("nan")}), ("set", {"joint1_deg": float("inf")}),
            ("set", {"joint4_deg": 0}), ("set", {"joint6_deg": 0}),
            ("jog", {"joint": 0, "step_deg": 1}), ("jog", {"joint": 8, "step_deg": 1}),
            ("jog", {"joint": True, "step_deg": 1}), ("jog", {"joint": 1.5, "step_deg": 1}),
            ("jog", {"joint": 1, "step_deg": 0}), ("jog", {"joint": 1, "step_deg": True}),
            ("jog", {"joint": 1, "step_deg": float("nan")}),
        ]
        invalid += [("set", {"joint1_deg": 1, "speed_deg_s": speed})
                    for speed in (0, 0.09, 10.01, True, float("nan"), float("inf"))]
        for action, args in invalid:
            with self.subTest(action=action, args=args), self.assertRaises((ValueError, TypeError)):
                self.call(action, **args)
        self.assertEqual([], session.moves)

    def test_jog_rejects_target_crossing_joint_limit(self):
        session = self.start()
        session.value["q"][0] = 2.9007 - math.radians(0.5)
        with self.assertRaises(ValueError):
            self.call("jog", joint=1, step_deg=1)
        self.assertEqual([], session.moves)

    def test_motion_requires_fresh_state_and_backend_control_readiness(self):
        session = self.start()
        original = session.snapshot()
        variants = [
            {"connected": False, "state": "stale", "data_age_ms": 3000},
            {"control_ready": False},
            {"q": [0] * 6},
            {"q": [float("nan")] + FakeSession.INITIAL_Q[1:]},
        ]
        for change in variants:
            session.value = {**copy.deepcopy(original), **change}
            with self.subTest(change=change), self.assertRaises((ValueError, RuntimeError, PermissionError)):
                self.call("set", joint1_deg=1)
        self.assertEqual([], session.moves)

    def test_completion_is_asynchronous_and_info_exposes_actual_and_target(self):
        session, action_id = self.started_move()
        self.assertEqual([], self.completions)
        info = self.call()
        self.assertEqual(FakeSession.INITIAL_Q, info["actual_rad"])
        self.assertAlmostEqual(1, info["target_deg"][0])
        self.assertAlmostEqual(math.radians(1), info["target_rad"][0])
        self.assertEqual(action_id, info["motion"].get("action_id", info["motion"].get("command_id")))
        session.finish(action_id)
        self.assertEqual("completed", self.completed(action_id)[1])
        info = self.call()
        self.assertAlmostEqual(1, info["actual_deg"][0])
        self.assertIsNotNone(info["last_result"])

    def test_backend_failure_and_rejection_are_not_reported_completed(self):
        session = self.start()
        for status in ("failed", "rejected"):
            with self.subTest(status=status):
                result = self.call("set", joint1_deg=1)
                session.finish(result["action_id"], status, error="backend declined motion")
                self.assertEqual("error", self.completed(result["action_id"])[1])

    def test_global_interrupt_hooks_stop_both_selected_robot_ips(self):
        first = self.start()
        second = self.start("control-b", "172.16.0.2")
        self.call("config", "not-started", robot_ip="172.16.0.9")
        spec = next(tool for tool in self.controller.get_tools() if tool["name"] == "joint_control")
        hooks = spec["inputSchema"]["x-hooks"]
        for name in ("on_interrupt_motion", "on_interrupt_all"):
            with self.subTest(hook=name):
                first_id = self.call("set", joint1_deg=1)["action_id"]
                second_id = self.call("set", "control-b", joint1_deg=2)["action_id"]
                # Core forwards exactly the hook arguments, without instance_id.
                response = self.bundle.dispatch("joint_control", dict(hooks[name]))
                self.assertEqual("stop_requested", response["state"])
                self.assertEqual({"172.16.0.3", "172.16.0.2"},
                                 {record["robot_ip"] for record in response["robots"]})
                self.assertEqual("cancelled", self.completed(first_id)[1])
                self.assertEqual("cancelled", self.completed(second_id)[1])
                self.assertEqual("cancelled", first.motion()["status"])
                self.assertEqual("cancelled", second.motion()["status"])
        self.assertEqual(2, len(self.sessions))
        self.assertEqual(0, first.closed)
        self.assertEqual(0, second.closed)

    def test_global_interrupt_failure_does_not_skip_another_robot(self):
        first = self.start()
        second = self.start("control-b", "172.16.0.2")
        first_id = self.call("set", joint1_deg=1)["action_id"]
        second_id = self.call("set", "control-b", joint1_deg=2)["action_id"]
        first.stop_error = RuntimeError("First robot stop unavailable")
        try:
            response = self.bundle.dispatch("joint_control", {"action": "stop"})
            self.assertEqual("error", response["state"])
            self.assertIn("First robot stop unavailable", response["error"])
            self.assertEqual([(None, None)], first.stops)
            self.assertEqual([(None, None)], second.stops)
            self.assertEqual("running", first.motion(first_id)["status"])
            self.assertEqual("cancelled", self.completed(second_id)[1])
            self.assertFalse(any(record[0] == first_id for record in self.completions))
        finally:
            first.stop_error = None

    def test_stop_timeout_keeps_card_and_feedback_available(self):
        session, action_id = self.started_move()
        session.complete_stop_immediately = False
        original_wait = session.wait

        def unconfirmed(command_id, timeout=None):
            if timeout == 3.5:
                raise TimeoutError("No physical stop confirmation")
            return original_wait(command_id, timeout)

        session.wait = unconfirmed
        try:
            with self.assertRaises(RuntimeError):
                self.call("stop")
            self.assertEqual("stopping", session.motion()["status"])
            self.assertEqual(0, session.closed)
            self.assertEqual([], self.completions)
            self.assertEqual("stopping", self.call()["motion"]["status"])
            self.publishers.assert_not_called()
        finally:
            session.wait = original_wait
            session.finish(action_id, "cancelled")

    def test_public_stop_waits_for_backend_confirmation_before_returning_ready(self):
        session, action_id = self.started_move()
        session.complete_stop_immediately = False
        result, errors = {}, []
        returned = threading.Event()

        def stop():
            try:
                result.update(self.call("stopmotion"))
            except Exception as exc:
                errors.append(exc)
            finally:
                returned.set()

        worker = threading.Thread(target=stop)
        worker.start()
        try:
            wait_for(lambda: session.motion()["status"] == "stopping")
            self.assertFalse(returned.is_set())
            self.assertEqual(0, session.closed)
            self.assertEqual([], self.completions)
        finally:
            session.finish(action_id, "cancelled")
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual([], errors)
        self.assertEqual("ready", result["state"])
        self.assertEqual(action_id, result["motion"]["command_id"])
        self.assertEqual("cancelled", self.completed(action_id)[1])

    def test_acp_delivery_failure_keeps_successful_physical_result_visible(self):
        attempts = []

        def failed_callback(*args):
            attempts.append(args)
            raise OSError("Core unavailable")

        self.controller._acp_callback = failed_callback
        session, action_id = self.started_move()
        session.finish(action_id)
        wait_for(lambda: (self.call()["acp"] or {}).get("state") == "failed")
        info = self.call()
        self.assertEqual("succeeded", info["motion"]["status"])
        self.assertEqual("completed", info["last_result"]["status"])
        self.assertEqual(action_id, info["acp"]["action_id"])
        self.assertTrue(attempts)
        self.assertLessEqual(len(attempts), 3)
        self.assertTrue(all(args[0] == action_id and args[1] == "completed" for args in attempts))

    def test_late_acp_ack_cannot_replace_a_newer_actions_delivery_status(self):
        first_callback = threading.Event()
        release_first = threading.Event()
        first_done = threading.Event()
        observed = []

        def callback(action_id, status, result):
            observed.append(action_id)
            if len(observed) == 1:
                first_callback.set()
                if not release_first.wait(2):
                    raise TimeoutError("Test did not release first ACP callback")
                first_done.set()

        self.controller._acp_callback = callback
        session, first_id = self.started_move()
        session.finish(first_id)
        try:
            self.assertTrue(first_callback.wait(1))
            second = self.call("set", joint1_deg=2)
            second_id = second["action_id"]
            session.finish(second_id)
            wait_for(lambda: (self.call()["acp"] or {}).get("action_id") == second_id
                     and self.call()["acp"]["state"] == "accepted")
        finally:
            release_first.set()
            self.assertTrue(first_done.wait(1))
        # Synchronize with the completed worker after its callback, so the
        # assertion covers the status write following that callback too.
        prefix = "franka-acp-control-a"
        for worker in threading.enumerate():
            if worker.name == prefix:
                worker.join(1)
        self.assertEqual(second_id, self.call()["last_result"]["action_id"])
        self.assertEqual(second_id, self.call()["acp"]["action_id"])

    def test_stop_racing_submission_leaves_no_running_motion(self):
        session = self.start()
        session.move_gate = threading.Event()
        errors = []
        results = {}

        def invoke(key, action, **args):
            try:
                results[key] = self.call(action, **args)
            except Exception as exc:
                errors.append(exc)

        mover = threading.Thread(target=invoke, args=("move", "set"), kwargs={"joint1_deg": 1})
        stopper = threading.Thread(target=invoke, args=("stop", "stop"))
        mover.start()
        try:
            self.assertTrue(session.move_entered.wait(1))
            stopper.start()
        finally:
            session.move_gate.set()
            mover.join(2)
            if stopper.ident is not None:
                stopper.join(2)
        self.assertFalse(mover.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertEqual([], errors)
        self.assertEqual("idle", results["stop"]["state"])
        self.assertEqual("cancelled", session.motion()["status"])
        self.assertEqual("cancelled", self.completed(results["move"]["action_id"])[1])


if __name__ == "__main__":
    unittest.main()
