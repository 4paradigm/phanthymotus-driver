"""Safety acceptance with a simulated SDK; commands are recorded, never sent."""
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agilex.piper.arm import ArmState, PiperDriver
from agilex.piper.control import Previews, mode_plan
from agilex.piper.device import PiperPlugin
from common.vendor_runtime import DriverBundle, make_handler


class SDK:
    def __init__(self):
        self.raw = (1000, 10000, -10000, 0, 20000, 0)
        self.enabled = (True,) * 6
        self.mode = "CAN_CTRL(0x1)"
        self.teach = "DISABLED(0x0)"
        self.status = "NORMAL(0x0)"
        self.error = 0
        self.calls = []
        self.width = 20.
        self.gripper_enabled = False
        self.gripper_error = 0
        self.follow = True
        self.frozen = False

    def GetSDKJointLimitParam(self, name):
        return tuple(map(math.radians, [(-150,150),(0,180),(-170,0),(-100,100),(-70,70),(-120,120)][int(name[1:])-1]))

    def GetSDKGripperRangeParam(self):
        return (0., .07)

    def MotionCtrl_2(self, mode, *args):
        self.calls.append(("mode", mode, *args))
        self.mode = "CAN_CTRL(0x1)" if mode == 1 else "STANDBY(0x0)"

    def MotionCtrl_1(self, **args):
        self.calls.append(("teach", args))
        code = args["grag_teach_ctrl"]
        if code == 1:
            self.mode, self.teach = "TEACHING_MODE(0x2)", "START_RECORDING(0x1)"
        elif code == 2:
            self.mode, self.teach = "CAN_CTRL(0x1)", "STOP_RECORDING(0x2)"
        elif code == 4:
            self.teach = "PAUSE_EXECUTION(0x4)"

    def JointCtrl(self, *target):
        self.calls.append(("joints", target))
        if self.follow:
            self.raw = tuple(target)

    def GripperCtrl(self, width, effort, enabled, zero):
        self.calls.append(("gripper", width, effort, enabled, zero))
        self.gripper_enabled = bool(enabled)
        if self.follow:
            self.width = width/1000

    def GetArmGripperMsgs(self):
        return SimpleNamespace(time_stamp=0 if self.frozen else time.time(), gripper_state=SimpleNamespace(
            grippers_angle=round(self.width*1000), grippers_effort=500,
            status_code=self.gripper_error, foc_status=SimpleNamespace(driver_enable_status=self.gripper_enabled)))

    def SearchPiperFirmwareVersion(self):
        self.calls.append(("firmware_query",))

    def GetPiperFirmwareVersion(self):
        return "S-V1.9-0"

    def DisconnectPort(self, **kwargs):
        pass


class Driver(PiperDriver):
    def __init__(self, can_name="can0"):
        super().__init__(can_name)
        self.arm = SDK()

    def connect(self):
        return self

    def _raw_state(self):
        a = self.arm
        state = ArmState(tuple(v/1000 for v in a.raw), a.enabled, a.mode, a.status, a.teach,
                         a.error, 0 if a.frozen else time.time())
        return a.raw, state, 0 if a.status.startswith("NORMAL") else 11

    def stationary(self):
        return True


class ControlsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.plugin = PiperPlugin({"data_dir": self.tmp.name, "async_actions": False}, driver_factory=Driver)
        self.plugin.motion_enabled = True
        self.bundle = DriverBundle([self.plugin])
        self.driver = self.plugin.connected()
        self.sdk = self.driver.arm
        self.addCleanup(self.plugin.stop)
        self.tick = 0.

    def clock(self):
        self.tick += .05
        return self.tick

    def call(self, card, action, **args):
        return self.bundle.dispatch("piper_"+card, {"action": action, **args})

    def run_fast(self, func):
        with patch("agilex.piper.arm.time.sleep"), patch("agilex.piper.arm.time.monotonic", side_effect=self.clock):
            return func()

    def test_manual_permission_never_sends_commands_and_invalidates_preview(self):
        self.plugin.motion_enabled = False
        self.sdk.enabled, self.sdk.mode = (False,)*6, "STANDBY(0x0)"
        preview = self.call("arm", "prepare")
        result = self.call("arm", "allow_motion")
        self.assertTrue(result["motion_enabled"])
        self.assertFalse(result["executed"])
        self.assertNotIn(preview["preview_id"], self.plugin.previews.entries)
        self.assertEqual(self.sdk.calls, [])
        self.assertFalse(self.call("arm", "block_motion")["motor_torque_released"])
        self.assertFalse(self.plugin.motion_enabled)
        self.assertEqual(self.sdk.calls, [])

    def test_manual_permission_rejects_bad_feedback_busy_or_mac(self):
        self.plugin.motion_enabled = False
        for fields in (dict(error=1), dict(frozen=True), dict(enabled=(True,False,True,True,True,True)),
                       dict(mode="TEACHING_MODE(0x2)"), dict(teach="EXECUTE_TRAJECTORY(0x3)")):
            original = dict(vars(self.sdk))
            with self.subTest(fields=fields):
                self.sdk.__dict__.update(fields)
                with self.assertRaises(RuntimeError):
                    self.call("arm", "allow_motion")
                self.assertFalse(self.plugin.motion_enabled)
            self.sdk.__dict__.update(original)
        with self.plugin.operation("piper_photo", "capture"), self.assertRaises(RuntimeError):
            self.call("arm", "allow_motion")
        self.plugin.config["can_transport"] = "gs_usb"
        with self.assertRaises(RuntimeError):
            self.call("arm", "allow_motion")
        self.assertEqual(self.sdk.calls, [])

    def test_manual_permission_resets_on_stop_reconnect_and_new_instance(self):
        for reset in (lambda: self.call("arm", "stop"),
                      lambda: self.call("status", "reconnect"), self.plugin.request_stop):
            self.call("arm", "start")
            self.call("arm", "allow_motion")
            reset()
            self.assertFalse(self.plugin.motion_enabled)
            self.call("arm", "start")
            self.assertFalse(self.plugin.motion_enabled)
        with patch.dict("os.environ", {"PIPER_MOTION_ENABLED": "0"}):
            fresh = PiperPlugin({"data_dir": self.tmp.name}, driver_factory=Driver)
            fresh.start()
            self.assertFalse(fresh.motion_enabled)
            self.assertIsNone(fresh.driver)
        self.assertEqual(self.sdk.calls, [])

    def test_closing_permission_cancels_motion_preserves_torque_and_photo(self):
        self.call("arm", "allow_motion")
        with self.plugin.operation("piper_gripper", "set_width"):
            result = self.call("arm", "block_motion")
            self.assertTrue(result["operation_stopping"])
            self.assertTrue(self.driver.cancelled.is_set())
            self.assertTrue(self.plugin.cancel.is_set())
            self.assertTrue(all(self.sdk.enabled))
        self.call("arm", "allow_motion")
        with self.plugin.operation("piper_photo", "import_photo"):
            self.assertFalse(self.call("arm", "block_motion")["operation_stopping"])
            self.assertFalse(self.plugin.cancel.is_set())
        self.assertEqual(self.sdk.calls, [])

    def test_permission_closed_between_validation_and_reservation_blocks_command(self):
        original = self.plugin.operation
        def revoke_then_reserve(*args, **kwargs):
            self.call("arm", "block_motion")
            return original(*args, **kwargs)
        with patch.object(self.plugin, "operation", side_effect=revoke_then_reserve), self.assertRaises(RuntimeError):
            self.call("arm", "hold",workspace_clear=True,take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_manual_permission_actions_are_canvas_only(self):
        schema = next(t["inputSchema"] for t in self.plugin.get_tools() if t["name"] == "piper_arm")
        for action in ("allow_motion", "block_motion"):
            self.assertIn(action, schema["properties"]["action"]["enum"])
            self.assertNotIn(action, schema["x-action-params"])
            for card in ("mode", "gripper", "photo"):
                with self.assertRaises(ValueError):
                    self.call(card, action)

    def test_invalid_motion_conditions_emit_zero_control_commands(self):
        cases = [dict(error=1), dict(enabled=(True,False,True,True,True,True)),
                 dict(mode="STANDBY(0x0)"), dict(mode="WIFI_CONTROL_MODE(0x4)"),
                 dict(teach="EXECUTE_TRAJECTORY(0x3)")]
        for fields in cases:
            original = dict(vars(self.sdk))
            for key,value in fields.items():
                setattr(self.sdk,key,value)
            with self.subTest(fields=fields), self.assertRaises(RuntimeError):
                self.call("arm", "move_joint", joint=1, target_deg=2)
            self.sdk.__dict__.update(original)
        self.assertEqual(self.sdk.calls, [])

    def test_bad_parameters_and_limits_emit_zero_commands(self):
        for args in ({"speed_deg_s": .4}, {"speed_deg_s": 6}, {"speed_pct": 11},
                     {"target_deg": float("nan")}, {"joint": True}, {"joint": "1"},
                     {"joint": 1.5}, {"joint": 0}, {"joint": 7}, {"execute": "false"},
                     {"target_deg": 14}, {"joint": 2, "target_deg": -1}):
            with self.subTest(args=args), self.assertRaises((RuntimeError,ValueError)):
                self.call("arm", "move_joint", **{"joint":1,"target_deg":2, **args})
        self.assertEqual(self.sdk.calls, [])

    def test_canvas_joint_numeric_input_previews_every_axis_without_commands(self):
        schema = next(t["inputSchema"] for t in self.plugin.get_tools() if t["name"] == "piper_arm")
        field = schema["properties"]["joint"]
        # Agent Core renders enums as string selects; this must be a numeric input.
        self.assertNotIn("enum", field)
        self.assertEqual(field["type"], "integer")
        self.assertEqual((field["minimum"], field["maximum"]), (1, 6))
        for joint in range(1, 7):
            with self.subTest(joint=joint):
                result = self.call("arm", "move_joint", joint=joint,
                                   target_deg=self.sdk.raw[joint-1]/1000+.5, execute=False)
                self.assertFalse(result["executed"])
                self.assertIn("preview_id", result)
        self.assertEqual(self.sdk.calls, [])

    def test_two_joint_move_is_previewed_bounded_and_stable(self):
        args = dict(j1_deg=2,j2_deg=11,j3_deg=-10,j4_deg=0,j5_deg=20,j6_deg=0)
        preview = self.call("arm", "move_joints", **args)
        self.assertEqual(self.sdk.calls, [])
        result = self.run_fast(lambda: self.call("arm", "move_joints", **args, execute=True,
                               preview_id=preview["preview_id"],workspace_clear=True,take_control=True))
        self.assertTrue(result["executed"])
        targets = [c[1] for c in self.sdk.calls if c[0] == "joints"]
        self.assertEqual(targets[-1], (2000,11000,-10000,0,20000,0))
        self.assertTrue(all(abs(b[0]-a[0]) <= 101 for a,b in zip(targets,targets[1:])))
        self.assertTrue(all(c[3] <= 10 for c in self.sdk.calls if c[0] == "mode"))
        count = len(self.sdk.calls)
        with self.assertRaises(RuntimeError):
            self.call("arm", "move_joints", **args, execute=True, workspace_clear=True,take_control=True)
        self.assertEqual(len(self.sdk.calls),count)

    def test_expired_drifted_changed_or_wrong_preview_emits_zero_commands(self):
        for change in ("age", "pose", "mode", "enable", "parameters"):
            self.sdk.mode, self.sdk.enabled, self.sdk.raw = "CAN_CTRL(0x1)", (True,)*6, (1000,10000,-10000,0,20000,0)
            preview = self.call("arm", "move_joint", joint=1,target_deg=2)
            if change == "age":
                self.plugin.previews.entries[preview["preview_id"]].created -= 61
            elif change == "pose":
                self.sdk.raw = (1600,*self.sdk.raw[1:])
            elif change == "mode":
                self.sdk.mode = "TEACHING_MODE(0x2)"
            elif change == "enable":
                self.sdk.enabled = (False,)*6
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                self.call("arm", "move_joint", joint=1,target_deg=3 if change=="parameters" else 2,
                          execute=True,preview_id=preview["preview_id"],workspace_clear=True,take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_mode_external_change_invalidates_even_if_changed_back(self):
        preview = self.call("arm", "move_joint", joint=1,target_deg=2)
        self.sdk.mode = "TEACHING_MODE(0x2)"
        self.call("status", "read")
        self.sdk.mode = "CAN_CTRL(0x1)"
        with self.assertRaises(RuntimeError):
            self.call("arm", "move_joint", joint=1,target_deg=2,execute=True,
                      preview_id=preview["preview_id"],workspace_clear=True,take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_mode_switch_does_not_enable_or_move(self):
        self.sdk.mode, self.sdk.enabled = "STANDBY(0x0)", (False,)*6
        preview = self.call("mode", "preview",target_mode="can",other_controllers_stopped=True)
        self.assertTrue(preview["allowed"])
        self.assertEqual(self.sdk.calls, [])
        result = self.call("mode", "switch",target_mode="can",other_controllers_stopped=True,preview_id=preview["preview_id"])
        self.assertTrue(result["confirmed"])
        self.assertEqual(self.sdk.calls, [("mode",1,1,5,0)])
        self.assertFalse(any(self.sdk.enabled))

    def test_standby_requires_disabled_and_can_requires_controller_confirmation(self):
        blocked = self.call("mode", "preview",target_mode="standby")
        self.assertFalse(blocked["allowed"])
        self.assertEqual(blocked["reminder"], "请先支撑机械臂并解除使能。")
        self.sdk.mode,self.sdk.enabled = "STANDBY(0x0)",(False,)*6
        self.assertFalse(self.call("mode","preview",target_mode="can")["allowed"])
        with self.assertRaises(RuntimeError):
            self.call("mode","switch",target_mode="can")
        self.assertEqual(self.sdk.calls, [])

    def test_unknown_firmware_gives_button_guidance_without_commands(self):
        args = dict(target_mode="teaching",workspace_clear=True,arm_supported=True)
        preview = self.call("mode", "preview", **args)
        self.assertTrue(preview["button_guidance"])
        self.assertFalse(preview["allowed"])
        self.assertFalse(self.call("mode", "switch", **args)["executed"])
        self.assertEqual(self.sdk.calls, [])

    def test_firmware_must_be_queried_and_explicitly_commissioned(self):
        self.plugin.config["mode"] = {"verified_teaching_firmwares": ["S-V1.9-0"]}
        self.assertFalse(self.plugin.verified_teaching())
        self.call("mode", "read",query_firmware=True)
        self.assertTrue(self.plugin.verified_teaching())
        self.sdk.calls.clear()
        args = dict(target_mode="teaching",workspace_clear=True,arm_supported=True)
        preview = self.call("mode", "preview", **args)
        result = self.call("mode", "switch", **args,preview_id=preview["preview_id"])
        self.assertTrue(result["confirmed"])
        self.assertEqual(self.sdk.calls, [("teach",{"grag_teach_ctrl":1})])

    def test_stop_teaching_does_not_need_parking_pose_or_claim_can(self):
        self.sdk.mode,self.sdk.teach,self.sdk.status = "TEACHING_MODE(0x2)","START_RECORDING(0x1)","TEACHING_RECORD(0xb)"
        result = self.call("mode","stop_teaching",arm_supported=True)
        self.assertTrue(result["button_guidance"])
        self.assertEqual(self.sdk.calls, [])
        self.assertTrue(self.sdk.mode.startswith("TEACHING"))

    def test_mode_confirmation_timeout_never_retries_or_uses_cached_feedback(self):
        state = self.driver.get_state()
        self.sdk.enabled = (False,)*6
        state = replace(state,enabled=(False,)*6)
        self.sdk.frozen = True
        result = self.run_fast(lambda: self.driver.switch_mode("standby",state))
        self.assertFalse(result["confirmed"])
        self.assertEqual(self.sdk.calls, [("mode",0,1,5,0)])

    def test_gripper_limits_units_torque_and_feedback(self):
        self.driver.gripper_limits_mm = (5,40)
        preview = self.call("gripper","open")
        self.assertEqual(preview["target_mm"],40)
        self.assertEqual(self.sdk.calls, [])
        result = self.run_fast(lambda: self.call("gripper","open",execute=True,
                               preview_id=preview["preview_id"],workspace_clear=True,take_control=True))
        self.assertEqual(result["gripper"]["width_mm"],40)
        self.assertTrue(all(c == ("gripper",40000,500,1,0) for c in self.sdk.calls))

    def test_bad_gripper_requests_emit_zero_control_commands(self):
        self.driver.gripper_limits_mm = (5,40)
        for args in (dict(width_mm=41),dict(width_mm=True),dict(width_mm=20,torque_nm=1.1)):
            with self.subTest(args=args),self.assertRaises(ValueError):
                self.call("gripper","set_width",**args)
        self.sdk.gripper_error = 1
        with self.assertRaises(RuntimeError):
            self.call("gripper","open")
        self.assertEqual(self.sdk.calls, [])

    def test_gripper_feedback_drift_invalidates_preview(self):
        preview = self.call("gripper","close")
        self.sdk.width = 22
        with self.assertRaises(RuntimeError):
            self.call("gripper","close",execute=True,preview_id=preview["preview_id"],workspace_clear=True,take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_motion_permission_off_blocks_mode_joint_and_gripper_execution(self):
        self.plugin.motion_enabled = False
        for card,action,args in (("arm","move_joint",dict(joint=1,target_deg=2)),("gripper","open",{})):
            self.call(card,action,**args)
            with self.assertRaises(RuntimeError):
                self.call(card,action,**args,execute=True,workspace_clear=True,take_control=True)
        self.sdk.mode,self.sdk.enabled="STANDBY(0x0)",(False,)*6
        self.call("mode","preview",target_mode="can",other_controllers_stopped=True)
        with self.assertRaises(RuntimeError):
            self.call("mode","switch",target_mode="can",other_controllers_stopped=True)
        self.assertEqual(self.sdk.calls, [])

    def test_motion_permission_off_also_blocks_supported_disable(self):
        self.plugin.motion_enabled = False
        for card, args in (("arm", {"arm_supported": True}),
                           ("gripper", {"object_supported": True})):
            with self.subTest(card=card), self.assertRaisesRegex(RuntimeError, "运动许可关闭"):
                self.call(card, "disable", **args)
        self.assertEqual(self.sdk.calls, [])

    def test_verified_playback_stop_keeps_mode_and_recording_uses_buttons(self):
        self.sdk.mode = "TEACHING_MODE(0x2)"
        self.sdk.teach = "EXECUTE_TRAJECTORY(0x3)"
        result = self.run_fast(lambda: self.driver.stop_teaching(True))
        self.assertTrue(result["confirmed"])
        self.assertEqual(self.sdk.mode, "TEACHING_MODE(0x2)")
        self.assertEqual(self.sdk.calls, [("teach", {"grag_teach_ctrl": 4})])
        self.sdk.calls.clear()
        self.sdk.teach = "START_RECORDING(0x1)"
        self.assertTrue(self.driver.stop_teaching(True)["button_guidance"])
        self.assertEqual(self.sdk.calls, [])

    def test_independent_stops_and_status_during_action(self):
        with self.plugin.operation("piper_arm","move_joints"):
            self.assertEqual(self.call("status","read")["error_code"],0)
            self.call("photo","stop")
            self.call("gripper","stop")
            self.call("mode","stop")
            self.call("status","stop")
            self.assertFalse(self.driver.cancelled.is_set())
            with self.assertRaises(RuntimeError):
                self.call("mode","preview",target_mode="standby")
            self.assertTrue(self.call("arm","stop")["operation_stopping"])
            self.assertTrue(self.driver.cancelled.is_set())
        self.assertEqual(self.sdk.calls, [])

    def test_cancel_and_external_mode_loss_discard_target_without_takeover(self):
        for kind in ("cancel", "mode"):
            self.sdk.raw = (1000,10000,-10000,0,20000,0)
            self.sdk.mode = "CAN_CTRL(0x1)"
            self.driver.cancelled.clear()
            self.sdk.calls.clear()
            def interrupt(_):
                if kind == "cancel":
                    self.driver.cancel_motion()
                else:
                    self.sdk.mode = "TEACHING_MODE(0x2)"
            with patch("agilex.piper.arm.time.sleep",side_effect=interrupt), patch("agilex.piper.arm.time.monotonic",side_effect=self.clock):
                with self.assertRaises(RuntimeError):
                    self.driver.move_joints((2,11,-10,0,20,0))
            self.assertLess(self.sdk.raw[0],2000)
            self.assertTrue(all(self.sdk.enabled))
            if kind == "mode":
                self.assertEqual(sum(c[0]=="mode" for c in self.sdk.calls),1)

    def test_unresponsive_joints_and_gripper_timeout(self):
        self.sdk.follow = False
        with self.assertRaisesRegex(RuntimeError,"超时"):
            self.run_fast(lambda: self.driver.move_joints((3,10,-10,0,20,0)))
        self.sdk.calls.clear()
        with self.assertRaisesRegex(RuntimeError,"未确认到位"):
            self.run_fast(lambda: self.driver.move_gripper(30))
        self.assertEqual(self.sdk.calls[-1],("gripper",20000,500,1,0))

    def test_reconnect_discards_preview_firmware_and_never_replays(self):
        self.call("arm","move_joint",joint=1,target_deg=2)
        self.plugin.firmware_version="S-V1.9-0"
        result = self.call("status","reconnect")
        self.assertEqual(result["error_code"],0)
        self.assertIsNone(result["firmware"])
        self.assertFalse(self.plugin.previews.entries)
        self.assertEqual(self.sdk.calls, [])
        self.assertEqual(self.plugin.driver.arm.calls, [])

    def test_second_driver_cannot_own_same_deployment_bus(self):
        other = PiperPlugin({"data_dir":self.tmp.name},driver_factory=Driver)
        self.addCleanup(other.stop)
        with self.assertRaises(BlockingIOError):
            other.connected()
        self.assertIsNone(other.driver)

    def test_async_completion_and_cancel_release_gate_before_callback(self):
        self.plugin.config["async_actions"] = True
        started, release, completed = threading.Event(),threading.Event(),threading.Event()
        reports = []
        def capture(args):
            started.set()
            release.wait(2)
            if self.plugin.cancel.is_set():
                raise RuntimeError("Capture cancelled")
            return {"output":"test.jpg"}
        def notify(*args):
            reports.append(args)
            self.assertFalse(self.plugin.gate.locked())
            completed.set()
        with patch.object(self.plugin,"capture",side_effect=capture),patch.object(self.plugin,"acp_complete",side_effect=notify):
            result = self.call("photo","capture")
            self.assertIn("action_id",result)
            self.assertTrue(started.wait(1))
            with self.assertRaises(RuntimeError):
                self.call("photo","capture")
            self.call("arm","stop")
            self.assertFalse(self.plugin.cancel.is_set())
            self.call("photo","stop")
            release.set()
            self.assertTrue(completed.wait(2))
        self.assertEqual(reports[0][1],"cancelled")
        self.assertEqual(self.call("photo","info")["last_action"]["status"],"cancelled")

    def test_schema_has_completion_limits_and_canvas_boolean_choices(self):
        schemas = {t["name"]:t["inputSchema"] for t in self.plugin.get_tools()}
        self.assertEqual(schemas["piper_arm"]["properties"]["speed_deg_s"]["default"],1)
        self.assertEqual(schemas["piper_mode"]["properties"]["arm_supported"]["enum"],[False,True])
        for card in ("arm","photo","gripper"):
            self.assertIn("x-completion",schemas["piper_"+card])

    def test_request_log_is_escaped_and_capped(self):
        handler = object.__new__(make_handler(lambda:self.bundle,"piper","piper"))
        handler.address_string=lambda:"127.0.0.1"
        with patch("builtins.print") as output:
            handler.log_message("%s", "GET /\x1b[31m\n" + "x"*500)
        message = output.call_args[0][0]
        self.assertNotIn("\x1b",message)
        self.assertNotIn("\n",message)
        self.assertIn("\\x1b",message)
        self.assertLessEqual(len(message),220)


if __name__ == "__main__":
    unittest.main()
