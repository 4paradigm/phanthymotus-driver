"""Gripper card routing, units and ACP, without opening any hardware connection."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock

from common.vendor_runtime import DriverBundle

spec = importlib.util.spec_from_file_location(
    "franka_gripper_card_fixtures", Path(__file__).with_name("test_franka_joint_control.py"))
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)
with mock.patch.dict(sys.modules, {"device": fixtures.device, "hardware": fixtures.hardware}):
    gripper = fixtures.load_module("franka_gripper_card", "gripper.py")


class FakeGripper(fixtures.FakeSession):
    def __init__(self, address, **kwargs):
        super().__init__(address, **kwargs)
        self.commands, self.physical_stops = [], []
        self.value.update(width=0.04, max_width=0.0801, is_grasped=False, temperature=28)

    def command(self, owner, action, **options):
        # Reuse the deterministic ownership/busy/terminal fixture.
        action_id = self.move(owner, [0] * 7, speed_rad_s=1)
        self.commands.append({"owner": owner, "action": action, **options})
        self._motions[action_id]["target"] = {"action": action, "width_m": options.get("width_m")}
        return action_id

    def finish(self, action_id, status="succeeded", **result):
        # The joint fixture updates q as a list, but card assertions only use Hand fields.
        super().finish(action_id, status, **result)

    def stop_gripper(self, owner):
        if owner not in self._owners:
            raise RuntimeError("Unknown owner")
        self.physical_stops.append(owner)
        if self._active:
            return self.stop_motion()
        if self.stop_error:
            raise self.stop_error
        return {"status": "succeeded"}


class GripperCardTests(unittest.TestCase):
    def setUp(self):
        self.sessions, self.completions = [], []
        self.targets = {}
        self.publishers = mock.Mock(return_value=mock.Mock())

        def session_factory(address, **options):
            session = FakeGripper(address, **options)
            self.sessions.append(session)
            return session

        self.manager = fixtures.hardware.SessionManager(session_factory=session_factory)
        discovery = mock.Mock()
        discovery.snapshot.return_value = {"state": "ready", "robots": []}
        self.plugin = gripper.GripperPlugin({}, manager=self.manager, discovery=discovery,
            publisher_factory=self.publishers, acp_callback=lambda *args: self.completions.append(args), activity_callback=mock.Mock())
        self.plugin.READY_TIMEOUT = .1
        self.bundle = DriverBundle([self.plugin])
        self.addCleanup(self.plugin.stop)

    def call(self, action="info", instance_id="hand-a", **args):
        if action in ("set_position", "grasp"):
            args.setdefault("speed_mm_s", 20)
        if action == "grasp":
            args.setdefault("force_n", 35)
        if instance_id in self.targets and not self.plugin._arm_ips:
            args.setdefault("robot_ip", self.targets[instance_id])
        return self.bundle.dispatch("gripper", {"action": action, **args})

    def start(self, instance_id="hand-a", ip="172.16.0.2", **args):
        self.targets[instance_id] = ip
        self.plugin.dispatch("start", {"_tool_name": "gripper"})
        self.plugin._dispatch("start", {"_tool_name": "gripper", "instance_id": "robot_" + ip.encode("ascii").hex(), "robot_ip": ip})
        return next(s for s in self.sessions if s.address == ip)

    def complete(self, session, result, status="succeeded"):
        session.finish(result["action_id"], status)
        fixtures.wait_for(lambda: any(c[0] == result["action_id"] for c in self.completions))

    def test_arm_selection_routes_without_a_topic_or_card_id(self):
        self.plugin._arm_ips = {"left": "172.16.0.3", "right": "172.16.0.2"}
        spec, = self.bundle.get_all_tools()
        self.assertNotIn("configSchema", spec)
        self.assertNotIn("target_robot_ip", spec["inputSchema"]["properties"])
        self.assertEqual("ready", self.call("start", speed_mm_s=15)["state"])
        self.call("start", "hand-b", speed_mm_s=15)
        self.assertEqual([], self.sessions)
        left = self.call("set_position", instance_id="", arm="left", width_mm=20)
        right = self.call("set_position", instance_id="", arm="right", width_mm=20)
        self.assertEqual(["172.16.0.3", "172.16.0.2"], [session.address for session in self.sessions])
        self.assertEqual(20, left["speed_mm_s"])
        self.complete(self.sessions[0], left)
        self.complete(self.sessions[1], right)

    def test_missing_gripper_speed_is_rejected_without_connecting(self):
        with self.assertRaisesRegex(ValueError, "speed_mm_s"):
            self.bundle.dispatch("gripper", {"action": "set_position", "robot_ip": "172.16.0.2", "width_mm": 20})
        self.assertEqual([], self.sessions)
        self.assertNotIn("default", self.plugin.get_tools()[0]["inputSchema"]["properties"]["speed_mm_s"])

    def test_grasp_requires_force_and_converts_absolute_bounds(self):
        with self.assertRaisesRegex(ValueError, "force_n"):
            self.bundle.dispatch("gripper", {"action": "grasp", "robot_ip": "172.16.0.2",
                "width_mm": 20, "speed_mm_s": 10})
        self.assertEqual([], self.sessions)
        with self.assertRaisesRegex(ValueError, "width_mm, speed_mm_s, force_n"):
            self.bundle.dispatch("gripper", {"action": "grasp", "robot_ip": "172.16.0.2"})
        session = self.start()
        result = self.call("grasp", width_mm=20, speed_mm_s=10, force_n=35,
                           min_width_mm=18, max_width_mm=22)
        command = session.commands[-1]
        self.assertEqual(.002, command["epsilon_inner_m"])
        self.assertEqual(.002, command["epsilon_outer_m"])
        self.complete(session, result)
        for low, high in ((22, 18), (21, 22), (18, 19), (20, 20)):
            with self.subTest(low=low, high=high), self.assertRaisesRegex(ValueError, "Opening bounds"):
                self.call("grasp", width_mm=20, speed_mm_s=10, force_n=35,
                          min_width_mm=low, max_width_mm=high)
        self.assertNotIn("default", self.plugin.get_tools()[0]["inputSchema"]["properties"]["force_n"])

    def test_contract_and_start_are_read_only(self):
        definition, = self.bundle.get_all_tools()
        self.assertEqual("actuator", definition["type"])
        self.assertFalse(definition["multiInstance"])
        self.assertEqual(["set_position", "grasp", "homing", "stopmotion", "info"],
                         definition["inputSchema"]["properties"]["action"]["enum"])
        self.assertEqual(["set_position", "grasp", "homing"],
                         definition["inputSchema"]["x-completion"]["actions"])
        self.assertEqual("arm", definition["inputSchema"]["x-resource"])
        self.assertNotIn("configSchema", definition)
        session = self.start()
        self.assertEqual([], session.commands)
        info = self.call()
        self.assertEqual("ready", info["state"])
        self.assertEqual(40, info["width_mm"])
        self.assertAlmostEqual(80.1, info["max_width_mm"])
        self.assertEqual(28, info["temperature_c"])
        self.assertTrue(info["homed"])

    def test_set_position_converts_units_and_waits_for_physical_completion(self):
        session = self.start()
        result = self.call("set_position", width_mm=25, speed_mm_s=12)
        command, = session.commands
        self.assertEqual("set_position", command["action"])
        self.assertEqual(0.025, command["width_m"])
        self.assertEqual(0.012, command["speed_m_s"])
        self.assertEqual([], self.completions)
        self.assertEqual(25, self.call()["target_width_mm"])
        self.complete(session, result)
        self.assertEqual("completed", self.completions[0][1])
        self.assertEqual("ready", self.call()["state"])

    def test_grasp_sdk_failure_is_reported(self):
        session = self.start()
        result = self.call("grasp", width_mm=20, force_n=35, min_width_mm=18, max_width_mm=24)
        self.complete(session, result, "failed")
        self.assertEqual("error", self.completions[0][1])

    def test_homing_calls_sdk_without_width(self):
        session = self.start()
        session.value["max_width"] = 0
        result = self.call("homing")
        self.assertEqual("homing", session.commands[0]["action"])
        self.assertNotIn("width_m", session.commands[0])
        self.complete(session, result)

    def test_stopmotion_releases_hold_without_stopping_other_ip_or_card(self):
        first = self.start()
        second = self.start("hand-b", "172.16.0.3")
        result = self.call("grasp", width_mm=20)
        self.complete(first, result)
        first.value["is_grasped"] = True
        self.assertEqual("ready", self.call("stopmotion")["state"])
        self.assertEqual(1, len(first.physical_stops))
        self.assertEqual([], second.physical_stops)
        self.assertFalse(self.plugin.PUBLISH_STATE)

    def test_stop_failure_is_not_reported_as_stopped(self):
        session = self.start()
        session.stop_error = RuntimeError("SDK stop failed")
        with self.assertRaisesRegex(RuntimeError, "SDK stop failed"):
            self.call("stopmotion")
        self.assertFalse(self.plugin.PUBLISH_STATE)

    def test_invalid_input_and_rejected_ip_never_submit(self):
        session = self.start()
        for args in ({}, {"width_mm": True}, {"width_mm": float("nan")},
                     {"width_mm": 81}, {"width_mm": -1}, {"width_mm": 20, "speed_mm_s": 0},
                     {"width_mm": 20, "force_n": 20}, {"width_mm": 20, "min_width_mm": -1}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.call("grasp", **args)
        with self.assertRaises(ValueError):
            self.call("homing", robot_ip="invalid-ip")
        self.assertEqual([], session.commands)


if __name__ == "__main__":
    unittest.main()
