"""Control-card acceptance: only simulated CAN commands are recorded."""
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from agilex.piper.arm_control import ArmControlPlugin
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
from test_piper_ten_cards import Driver


class ArmControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        with patch.dict("os.environ", {"PIPER_MOTION_ENABLED": "0", "PIPER_CAMERA_ENABLED": "0"}):
            self.plugin = ArmControlPlugin({"data_dir": self.tmp.name, "async_actions": False,
                                            "name": "PiPER A", "can_interface": "can0"}, driver_factory=Driver)
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])
        self.tick = 0.

    def call(self, action, **args):
        return self.bundle.dispatch("arm_control", {"action": action, **args})

    def sdk(self):
        self.plugin.motion_enabled = True
        return self.plugin.connected().arm

    def fast(self, operation):
        def clock():
            self.tick += .05
            return self.tick
        with patch("agilex.piper.arm.time.sleep"), patch("agilex.piper.arm.time.monotonic", side_effect=clock):
            return operation()

    def execute(self, action, plan, **args):
        return self.fast(lambda: self.call(action, **args, execute=True,
            preview_id=plan["preview_id"], workspace_clear=True, take_control=True))

    def test_registration_and_lifecycle_are_passive(self):
        plugin = build_plugins({"data_dir": self.tmp.name}, "arm-a", None)[0]
        self.addCleanup(plugin.stop)
        self.assertIsInstance(plugin, ArmControlPlugin)
        self.assertEqual({tool["name"] for tool in plugin.get_tools()}, {"arm_control", "piper_photo"})
        for action in ("start", "info", "stop", "start"):
            self.call(action)
            self.assertIsNone(self.plugin.driver)
        snapshot = self.plugin.read_monitor_snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertIsNone(snapshot["joints_deg"])
        self.assertIn("connection", snapshot["reasons"])
        for old in ("piper_status", "piper_joint", "piper_enable", "piper_stop", "piper_trajectory"):
            self.assertIsNone(self.bundle.dispatch(old, {"action": "start"}))

    def test_permissions_and_status_do_not_move_or_enable(self):
        self.call("read")
        sdk = self.plugin.driver.arm
        self.call("allow_motion")
        self.assertTrue(self.plugin.motion_enabled)
        self.assertEqual(sdk.calls, [])
        self.call("block_motion")
        self.assertFalse(self.plugin.motion_enabled)
        self.assertEqual(sdk.calls, [])

    def test_mode_switch_does_not_enable_and_same_mode_is_noop(self):
        sdk = self.sdk()
        sdk.mode, sdk.enabled = "STANDBY(0x0)", (False,) * 6
        args = {"target_mode": "can", "other_controllers_stopped": True}
        plan = self.call("switch_mode", **args)
        self.assertEqual(sdk.calls, [])
        self.execute("switch_mode", plan, **args)
        self.assertEqual(sdk.enabled, (False,) * 6)
        self.assertTrue(all(call[0] == "mode" for call in sdk.calls))
        count = len(sdk.calls)
        self.call("switch_mode", **args, execute=True)
        self.assertEqual(len(sdk.calls), count)

    def test_enable_and_disable_keep_arm_and_gripper_separate(self):
        sdk = self.sdk()
        sdk.enabled = (False,) * 6
        original_width = sdk.width
        plan = self.call("enable", object="arm")
        self.execute("enable", plan, object="arm")
        self.assertTrue(all(sdk.enabled))
        self.assertFalse(sdk.gripper_enabled)
        plan = self.call("enable", object="gripper")
        self.execute("enable", plan, object="gripper")
        self.assertTrue(sdk.gripper_enabled)
        self.assertEqual(sdk.width, original_width)
        self.call("disable", object="gripper", supported=True)
        self.assertTrue(all(sdk.enabled))
        self.assertFalse(sdk.gripper_enabled)
        self.call("disable", object="arm", supported=True)
        self.assertFalse(any(sdk.enabled))

    def test_jog_all_axes_steps_and_directions(self):
        sdk = self.sdk()
        original = sdk.raw
        for joint in range(1, 7):
            for step in (.5, 1, 2):
                for action, sign in (("jog_positive", 1), ("jog_negative", -1)):
                    with self.subTest(joint=joint, step=step, action=action):
                        sdk.raw = original
                        args = {"joint": joint, "step_deg": step}
                        before = len(sdk.calls)
                        plan = self.call(action, **args)
                        self.assertEqual(len(sdk.calls), before)
                        self.execute(action, plan, **args)
                        expected = list(original)
                        expected[joint - 1] += round(sign * step * 1000)
                        self.assertEqual(sdk.raw, tuple(expected))

    def test_jog_execution_uses_preview_absolute_target_after_small_drift(self):
        sdk = self.sdk()
        plan = self.call("jog_positive", joint=1)
        self.assertEqual(plan["target_deg"][0], 1.5)
        sdk.raw = (1200, *sdk.raw[1:])
        self.execute("jog_positive", plan, joint=1)
        self.assertEqual(sdk.raw[0], 1500)

    def test_cancel_expired_reused_changed_previews_never_send_again(self):
        sdk = self.sdk()
        for invalidation in ("cancel", "expire", "pose", "mode", "step", "direction"):
            sdk.raw = (1000, 10000, -10000, 0, 20000, 0)
            sdk.mode = "CAN_CTRL(0x1)"
            args = {"joint": 1, "step_deg": .5}
            plan = self.call("jog_positive", **args)
            action = "jog_positive"
            if invalidation == "cancel":
                self.call("cancel_preview")
            elif invalidation == "expire":
                self.plugin.previews.entries[plan["preview_id"]].created -= 61
            elif invalidation == "pose":
                sdk.raw = (1600, *sdk.raw[1:])
            elif invalidation == "mode":
                sdk.mode = "STANDBY(0x0)"
            elif invalidation == "step":
                args["step_deg"] = 1
            else:
                action = "jog_negative"
            before = len(sdk.calls)
            with self.subTest(reason=invalidation), self.assertRaises(RuntimeError):
                self.call(action, **args, execute=True, preview_id=plan["preview_id"],
                          workspace_clear=True, take_control=True)
            self.assertEqual(len(sdk.calls), before)
        sdk.mode = "CAN_CTRL(0x1)"
        plan = self.call("jog_positive", joint=1)
        self.execute("jog_positive", plan, joint=1)
        before = len(sdk.calls)
        with self.assertRaises(RuntimeError):
            self.execute("jog_positive", plan, joint=1)
        self.assertEqual(len(sdk.calls), before)

    def test_limits_speed_staleness_fault_and_partial_enable_rejected(self):
        sdk = self.sdk()
        for args in ({"step_deg": 1.5}, {"speed_deg_s": 0}, {"speed_pct": 11},
                     {"joint": "1"}, {"execute": "true"}, {"step_deg": float("nan")}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.call("jog_positive", **args)
        sdk.raw = (150000, *sdk.raw[1:])
        with self.assertRaises(ValueError):
            self.call("jog_positive", joint=1)
        sdk.raw = (1000, *sdk.raw[1:])
        sdk.frozen = True
        with self.assertRaisesRegex(RuntimeError, "反馈"):
            self.call("jog_positive")
        sdk.frozen = False
        for attr, bad, good in (("error", 1, 0), ("status", "FAULT(0x1)", "NORMAL(0x0)"),
                                ("enabled", (True, False, True, True, True, True), (True,) * 6)):
            setattr(sdk, attr, bad)
            with self.subTest(attr=attr), self.assertRaises(RuntimeError):
                self.call("jog_positive")
            setattr(sdk, attr, good)
        self.assertEqual(sdk.calls, [])

    def test_absolute_joint_and_gripper_targets_use_existing_logic(self):
        sdk = self.sdk()
        plan = self.call("move_joint", joint=3, target_deg=-12)
        self.execute("move_joint", plan, joint=3, target_deg=-12)
        self.assertEqual(sdk.raw[2], -12000)
        with self.assertRaisesRegex(RuntimeError, "夹爪"):
            self.call("gripper_open")
        sdk.gripper_enabled = True
        for action, args, width in (("gripper_open", {}, 70), ("gripper_close", {}, 0),
                                    ("gripper_set_width", {"width_mm": 12}, 12)):
            plan = self.call(action, **args)
            self.execute(action, plan, **args)
            self.assertEqual(sdk.width, width)

    def test_stop_scope_bypasses_gate_and_permission_block_cancels(self):
        sdk = self.sdk()
        self.plugin.worker_context.operation_object = "gripper"
        with self.plugin.operation("arm_control", "gripper_open"):
            self.assertFalse(self.call("halt", target="arm")["operation_stopping"])
            self.assertTrue(self.call("halt", target="gripper")["operation_stopping"])
            self.assertTrue(self.plugin.driver.cancelled.is_set())
        self.plugin.worker_context.operation_object = "arm"
        with self.plugin.operation("arm_control", "jog_positive"):
            self.assertTrue(self.call("block_motion")["operation_stopping"])
            self.assertFalse(self.plugin.motion_enabled)
        self.assertFalse(any(call[0] == "disable" for call in sdk.calls))

    def test_async_repeated_click_rejects_and_reports_public_control_identity(self):
        self.sdk()
        self.plugin.config["async_actions"] = True
        started, release, completed = threading.Event(), threading.Event(), threading.Event()
        reports = []
        def motion(*args):
            started.set()
            release.wait(2)
            if self.plugin.cancel.is_set():
                raise RuntimeError("cancelled")
            return self.plugin.driver.get_state()
        def notify(*args):
            reports.append(args)
            completed.set()
        plan = self.call("jog_positive")
        with patch.object(self.plugin.driver, "_move_joint_target", side_effect=motion), patch.object(self.plugin, "acp_complete", side_effect=notify):
            try:
                result = self.call("jog_positive", execute=True, preview_id=plan["preview_id"],
                                   workspace_clear=True, take_control=True)
                self.assertIn("action_id", result)
                self.assertTrue(started.wait(1))
                with self.assertRaisesRegex(RuntimeError, "正在执行"):
                    self.call("jog_positive", execute=True, preview_id=plan["preview_id"],
                              workspace_clear=True, take_control=True)
                self.assertTrue(self.plugin.read_monitor_snapshot()["busy"])
                self.assertTrue(self.call("halt", target="arm")["operation_stopping"])
                release.set()
                self.assertTrue(completed.wait(1))
                self.assertEqual(reports[0][-1], "arm_control")
                self.assertEqual(self.plugin.last_actions["arm_control"]["status"], "cancelled")
                self.assertEqual(self.call("info")["last_action"]["action_id"], result["action_id"])
                self.assertFalse(self.plugin.gate.locked())
            finally:
                release.set()

    def test_snapshot_is_passive_detached_and_does_not_wait_connection_lock(self):
        sdk = self.sdk()
        self.plugin.config["joint_limits_deg"] = [(-100, 100), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120)]
        self.plugin.driver.joint_limits_deg = self.plugin.config["joint_limits_deg"]
        before = list(sdk.calls)
        snapshot = self.plugin.read_monitor_snapshot()
        self.assertTrue(snapshot["fresh"])
        self.assertEqual(snapshot["joint_limits_deg"][0], [-100, 100])
        snapshot["joints_deg"][0] = 123
        self.assertEqual(self.plugin.read_monitor_snapshot()["joints_deg"][0], 1)
        locked, release = threading.Event(), threading.Event()
        def hold_connection():
            with self.plugin.connection_lock:
                locked.set()
                release.wait(2)
        thread = threading.Thread(target=hold_connection)
        thread.start()
        self.assertTrue(locked.wait(1))
        try:
            started = time.monotonic()
            unavailable = self.plugin.read_monitor_snapshot()
            self.assertLess(time.monotonic() - started, .1)
            self.assertIsNone(unavailable["joints_deg"])
            self.assertIn("connection", unavailable["reasons"])
        finally:
            release.set()
            thread.join(1)
        sdk.frozen = True
        snapshot = self.plugin.read_monitor_snapshot()
        self.assertFalse(snapshot["fresh"])
        self.assertIsNone(snapshot["joints_deg"])
        self.assertIsNone(snapshot["gripper"])
        self.assertIsNone(snapshot["faults"])
        self.assertEqual(sdk.calls, before)

    def test_two_instances_keep_data_and_commands_isolated(self):
        first = self.sdk()
        other = ArmControlPlugin({"name": "PiPER B", "can_interface": "can1", "data_dir": self.tmp.name + "/b",
                                  "async_actions": False}, driver_factory=Driver)
        self.addCleanup(other.stop)
        second = other.connected().arm
        second.raw = (2000, *second.raw[1:])
        plan = self.call("jog_positive")
        self.execute("jog_positive", plan)
        self.assertEqual(second.calls, [])
        self.assertEqual(second.raw[0], 2000)
        self.assertEqual(other.read_monitor_snapshot()["device_name"], "PiPER B")
        self.assertEqual(other.read_monitor_snapshot()["can_interface"], "can1")
        self.assertEqual(self.plugin.read_monitor_snapshot()["joints_deg"][0], 1.5)


if __name__ == "__main__":
    unittest.main()
