"""Gripper card routing, units and ACP, without opening any hardware connection."""

import importlib.util
from pathlib import Path
import sys
import unittest
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
        self.sessions, self.publishers, self.completions = [], [], []

        def session_factory(address, **options):
            session = FakeGripper(address, **options)
            self.sessions.append(session)
            return session

        def publisher_factory(topic, hz, callback):
            publisher = fixtures.FakePublisher(topic, hz, callback)
            self.publishers.append(publisher)
            return publisher

        self.manager = fixtures.hardware.SessionManager(session_factory=session_factory)
        discovery = mock.Mock()
        discovery.snapshot.return_value = {"state": "ready", "robots": []}
        self.plugin = gripper.GripperPlugin({}, manager=self.manager, discovery=discovery,
            publisher_factory=publisher_factory, acp_callback=lambda *args: self.completions.append(args))
        self.bundle = DriverBundle([self.plugin])
        self.addCleanup(self.plugin.stop)

    def call(self, action="info", instance_id="hand-a", **args):
        return self.bundle.dispatch("gripper", {"action": action, "instance_id": instance_id, **args})

    def start(self, instance_id="hand-a", ip="172.16.0.2", **args):
        self.call("start", instance_id, robot_ip=ip, **args)
        return next(s for s in self.sessions if s.address == ip)

    def complete(self, session, result, status="succeeded"):
        session.finish(result["action_id"], status)
        fixtures.wait_for(lambda: any(c[0] == result["action_id"] for c in self.completions))

    def test_contract_and_start_are_read_only(self):
        definition, = self.bundle.get_all_tools()
        self.assertEqual("actuator", definition["type"])
        self.assertTrue(definition["multiInstance"])
        self.assertEqual(["set_position", "grasp", "homing", "stopmotion", "info"],
                         definition["inputSchema"]["properties"]["action"]["enum"])
        self.assertEqual(["set_position", "grasp", "homing"],
                         definition["inputSchema"]["x-completion"]["actions"])
        self.assertEqual("arm", definition["inputSchema"]["x-resource"])
        self.assertNotIn("required", definition["configSchema"])
        session = self.start()
        self.assertEqual([], session.commands)
        info = self.call()
        self.assertEqual("ready", info["state"])
        self.assertEqual(40, info["width_mm"])
        self.assertAlmostEqual(80.1, info["max_width_mm"])
        self.assertEqual(28, info["temperature_c"])
        self.assertTrue(info["homed"])

    def test_set_position_converts_units_and_waits_for_physical_completion(self):
        session = self.start(speed_mm_s=12)
        result = self.call("set_position", width_mm=25)
        command, = session.commands
        self.assertEqual("set_position", command["action"])
        self.assertEqual(0.025, command["width_m"])
        self.assertEqual(0.012, command["speed_m_s"])
        self.assertEqual([], self.completions)
        self.assertEqual(25, self.call()["target_width_mm"])
        self.complete(session, result)
        self.assertEqual("completed", self.completions[0][1])
        self.assertEqual("ready", self.call()["state"])

    def test_grasp_defaults_and_sdk_failure_are_reported(self):
        session = self.start(force_n=35)
        result = self.call("grasp", width_mm=20, epsilon_inner_mm=2, epsilon_outer_mm=4)
        self.assertEqual(35, session.commands[0]["force_n"])
        self.assertEqual(0.002, session.commands[0]["epsilon_inner_m"])
        self.assertEqual(0.004, session.commands[0]["epsilon_outer_m"])
        self.complete(session, result, "failed")
        self.assertEqual("error", self.completions[0][1])

    def test_homing_is_explicit_and_has_no_width(self):
        session = self.start()
        session.value["max_width"] = 0
        result = self.call("homing")
        self.assertEqual("homing", session.commands[0]["action"])
        self.assertNotIn("width_m", session.commands[0])
        self.complete(session, result)

    def test_each_ip_is_independent_and_output_topic_selects_exact_card(self):
        first = self.start()
        second = self.start("hand-b", "172.16.0.3")
        topic = self.call(instance_id="hand-b")["topic_out"][0]["topic"]
        result = self.call("set_position", instance_id="", output_topic=topic, width_mm=10)
        self.assertEqual([], first.commands)
        self.assertEqual(1, len(second.commands))
        self.complete(second, result)
        for args in ({}, {"output_topic": "/unknown"},
                     {"instance_id": "hand-a", "output_topic": topic}):
            with self.assertRaises(ValueError):
                self.call("grasp", **{"instance_id": "", **args}, width_mm=10)
        self.assertEqual([], first.commands)

    def test_stopmotion_releases_hold_without_stopping_other_ip_or_card(self):
        first = self.start()
        second = self.start("hand-b", "172.16.0.3")
        result = self.call("grasp", width_mm=20)
        self.complete(first, result)
        first.value["is_grasped"] = True
        self.assertEqual("ready", self.call("stopmotion")["state"])
        self.assertEqual(1, len(first.physical_stops))
        self.assertEqual([], second.physical_stops)
        self.assertEqual(0, self.publishers[0].closed)

    def test_stop_failure_is_not_reported_as_stopped(self):
        session = self.start()
        session.stop_error = RuntimeError("SDK stop failed")
        with self.assertRaisesRegex(RuntimeError, "SDK stop failed"):
            self.call("stopmotion")
        self.assertEqual(0, self.publishers[0].closed)

    def test_invalid_input_and_rejected_ip_never_submit(self):
        session = self.start()
        for args in ({}, {"width_mm": True}, {"width_mm": float("nan")},
                     {"width_mm": 81}, {"width_mm": -1}, {"width_mm": 20, "speed_mm_s": 0},
                     {"width_mm": 20, "force_n": 20}, {"width_mm": 20, "epsilon_outer_mm": -1}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.call("grasp", **args)
        with self.assertRaises(ValueError):
            self.call("config", robot_ip="172.16.0.3")
        with self.assertRaisesRegex(RuntimeError, "rejected configuration"):
            self.call("homing")
        self.assertEqual([], session.commands)


if __name__ == "__main__":
    unittest.main()
