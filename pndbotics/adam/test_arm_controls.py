"""ROS-free validation tests for the Adam human-facing arm interface."""

from __future__ import annotations

import math
import sys
import time
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

import device
from device import (ADAM_PRO_JOINTS, ARM_ACTIONS, ARM_JOINT_CONTROLS,
                    ARM_POSES, ArmControlPlugin, ArmGesturePlugin, HandPlugin,
                    HandGesturePlugin, _arm_target_radians)


class _FakePublisher:
    def __init__(self):
        self.commands = []

    def Write(self, command, **kwargs):
        self.commands.append(command)


def _fake_lowcmd(dof):
    return types.SimpleNamespace(
        mode_pr=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0,
                                         kp=0.0, kd=0.0, ki=0.0)
                   for _ in range(dof)],
    )


def _prime_arm_plugin(publisher):
    plugin = ArmControlPlugin({}, "", None, dds_lowcmd_pub=publisher)
    plugin._hold_q = [index / 100.0 for index in range(31)]
    plugin._current_q = plugin._hold_q.copy()
    plugin._seg_current = plugin._hold_q.copy()
    plugin._seg_start = {}
    plugin._seg_span = plugin._DEFAULT_TRANSITION_SECONDS
    plugin._seg_started_at = time.monotonic()
    plugin._state_ready.set()
    return plugin


class ArmControlTests(unittest.TestCase):
    def test_control_ids_are_human_facing_and_cover_each_upper_body_joint(self):
        self.assertIn("left_shoulder_pitch", ARM_JOINT_CONTROLS)
        self.assertIn("right_wrist_roll", ARM_JOINT_CONTROLS)
        self.assertNotIn("shoulderPitch_Left", ARM_JOINT_CONTROLS)
        self.assertIn("neutral", ARM_POSES)

    def test_arm_lowcmd_rejects_non_pro_layouts_explicitly(self):
        with self.assertRaisesRegex(ValueError, "only Adam Pro"):
            ArmControlPlugin({}, "", None, variant="sp")

    def test_adam_pro_layout_matches_the_verified_motor_order(self):
        expected = {
            "waistRoll": 12, "waistPitch": 13, "waistYaw": 14,
            "neckYaw": 15, "neckPitch": 16,
            "shoulderPitch_Left": 17, "shoulderRoll_Left": 18,
            "shoulderYaw_Left": 19, "elbow_Left": 20,
            "wristYaw_Left": 21, "wristPitch_Left": 22,
            "wristRoll_Left": 23,
            "shoulderPitch_Right": 24, "shoulderRoll_Right": 25,
            "shoulderYaw_Right": 26, "elbow_Right": 27,
            "wristYaw_Right": 28, "wristPitch_Right": 29,
            "wristRoll_Right": 30,
        }
        for joint, index in expected.items():
            self.assertEqual(index, ADAM_PRO_JOINTS.index(joint), joint)

    def test_each_joint_has_a_distinct_action_and_angle_field(self):
        self.assertEqual("left_elbow", ARM_ACTIONS["set_left_elbow"])
        self.assertEqual("right_wrist_roll", ARM_ACTIONS["set_right_wrist_roll"])
        self.assertEqual(len(ARM_JOINT_CONTROLS), len(ARM_ACTIONS))

    def test_degrees_convert_to_the_ros_joint_target(self):
        name, target = _arm_target_radians("left_shoulder_pitch", -90)
        self.assertEqual("shoulderPitch_Left", name)
        self.assertAlmostEqual(-math.pi / 2, target)

    def test_each_joint_rejects_its_own_limit_violation(self):
        with self.assertRaisesRegex(ValueError, "left_shoulder_roll"):
            _arm_target_radians("left_shoulder_roll", -40)
        with self.assertRaisesRegex(ValueError, "right_shoulder_roll"):
            _arm_target_radians("right_shoulder_roll", 40)

    def test_unknown_or_nonfinite_inputs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "advertised"):
            _arm_target_radians("dof_pos/shoulderPitch_Left", 0)
        with self.assertRaisesRegex(ValueError, "finite"):
            _arm_target_radians("left_elbow", float("nan"))

    def test_shoulder_command_writes_arm_slots_without_targeting_neck(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        plugin._hold_q = [0.0] * 31
        plugin._current_q = [0.0] * 31
        plugin._seg_current = [0.0] * 31
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin.start()
            result = plugin.dispatch("set_shoulder", {
                "side": "left", "pitch_deg": -30, "roll_deg": 20,
                "yaw_deg": 10,
            })
            self.assertTrue(result["success"], result)
            plugin._stop_event.set()
            plugin._thread.join(1.0)
            plugin._thread = None
            plugin._seg_started_at = time.monotonic() - plugin._seg_span
            plugin._write_command(0.02)
        finally:
            plugin._stop_event.set()
            if plugin._thread is not None:
                plugin._thread.join(1.0)
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        command = publisher.commands[-1]
        self.assertEqual(0.0, command.motor_cmd[15].q)
        self.assertEqual(0.0, command.motor_cmd[16].q)
        self.assertAlmostEqual(math.radians(-30), command.motor_cmd[17].q)
        self.assertAlmostEqual(math.radians(20), command.motor_cmd[18].q)
        self.assertAlmostEqual(math.radians(10), command.motor_cmd[19].q)

    def test_lowcmd_holds_non_arm_joints_and_uses_official_arm_pd(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        plugin._active = True
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        plugin._target_q[elbow] = -0.5
        plugin._seg_start = {i: plugin._seg_current[i] for i in plugin._target_q}
        plugin._seg_started_at = time.monotonic() - 0.2

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        command = publisher.commands[-1]
        hip = ADAM_PRO_JOINTS.index("hipPitch_Left")
        self.assertEqual(command.motor_cmd[hip].q, plugin._hold_q[hip])
        self.assertEqual(command.motor_cmd[hip].kp, 400.0)
        self.assertEqual(command.motor_cmd[hip].kd, 6.1)
        self.assertLess(command.motor_cmd[elbow].q, plugin._hold_q[elbow])
        self.assertEqual(command.motor_cmd[elbow].kp, 100.0)
        self.assertEqual(command.motor_cmd[elbow].kd, 2.0)

    def test_release_writes_zero_arm_gain_before_deactivating(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        plugin._hold_q = [0.0] * 31
        plugin._current_q = [0.0] * 31
        plugin._seg_current = [0.0] * 31
        plugin._active = True
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        plugin._target_q[elbow] = -0.5
        plugin._release_started_at = time.monotonic() - 2.0

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        self.assertEqual(publisher.commands[-1].motor_cmd[elbow].kp, 0.0)
        self.assertFalse(plugin._active)

    def test_stop_waits_for_release_and_includes_waist(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        plugin._hold_q = [0.0] * 31
        plugin._current_q = [0.0] * 31
        plugin._seg_current = [0.0] * 31
        plugin._active = True
        plugin._streaming = True
        waist = ADAM_PRO_JOINTS.index("waistYaw")
        neck = ADAM_PRO_JOINTS.index("neckYaw")
        plugin._target_q[waist] = 0.2
        plugin._target_q[neck] = 0.1
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._release_started_at = time.monotonic() - 2.0
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory
        self.assertEqual(0.0, publisher.commands[-1].motor_cmd[waist].kp)
        self.assertEqual(0.0, publisher.commands[-1].motor_cmd[neck].kp)

    def test_segment_starts_from_current_output_and_ends_at_target(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        hold = plugin._hold_q[shoulder]
        plugin._set_targets({"shoulderPitch_Left": -1.0})
        self.assertEqual(plugin._seg_current[shoulder], hold)
        self.assertEqual(plugin._seg_start[shoulder], hold)
        self.assertEqual(plugin._target_q[shoulder], -1.0)

        # Mid-motion the command is eased between start and target.
        plugin._seg_started_at = time.monotonic() - plugin._seg_span / 2
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory
        mid = publisher.commands[-1].motor_cmd[shoulder].q
        self.assertTrue(hold > mid > -1.0)

        # Retarget to a new pose: the new segment starts from current output,
        # not from the original hold or previous target.
        plugin._set_targets({"shoulderPitch_Left": 0.5})
        self.assertAlmostEqual(plugin._seg_start[shoulder], mid)
        self.assertEqual(plugin._target_q[shoulder], 0.5)

        # At the end of the segment the output equals the newest target.
        plugin._seg_started_at = time.monotonic() - plugin._seg_span
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory
        self.assertAlmostEqual(publisher.commands[-1].motor_cmd[shoulder].q, 0.5)

    def test_easing_endpoints_have_zero_slope(self):
        ease = ArmControlPlugin._ease
        self.assertEqual(ease(0.0), 0.0)
        self.assertEqual(ease(1.0), 1.0)
        # Derivative of 10u^3-15u^4+6u^5 is 30u^2-60u^3+30u^4 = 30u^2(u-1)^2,
        # zero at both endpoints.
        self.assertAlmostEqual(ease(0.01), 0.0, places=2)
        self.assertAlmostEqual(ease(0.99), 1.0, places=2)

    def test_segment_span_keeps_the_easing_peak_within_the_velocity_limit(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        ease = ArmControlPlugin._ease
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        hold = plugin._hold_q[shoulder]
        samples = 400
        for distance in (0.2, 0.5, 1.0, 2.0):
            plugin._set_targets({"shoulderPitch_Left": hold - distance})
            span = plugin._seg_span
            peak = 0.0
            previous = ease(0.0)
            for step in range(1, samples + 1):
                current = ease(step / samples)
                peak = max(peak, (current - previous) * distance * samples / span)
                previous = current
            # The ease peaks at 1.875 / span, so sampling the profile is the
            # honest check: duration / distance alone lets a 1 rad move reach
            # 0.94 rad/s against a 0.5 rad/s limit.
            self.assertLessEqual(peak, plugin._MAX_VELOCITY_RAD_S + 1e-6,
                                 f"{distance} rad moved at {peak} rad/s")
            if distance >= 1.0:
                # Past the smoothing floor these spans come from the velocity
                # budget, so they must not be wastefully long either.
                self.assertGreater(peak, plugin._MAX_VELOCITY_RAD_S * 0.99)

    def test_small_targets_use_the_configured_minimum_span(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        plugin._set_targets({"shoulderPitch_Left": plugin._hold_q[shoulder] - 0.05})
        # The velocity budget only ever lengthens a segment, so a nudge is
        # smoothed over the configured transition rather than made quicker.
        self.assertAlmostEqual(plugin._DEFAULT_TRANSITION_SECONDS,
                               plugin._seg_span)


class GestureLifecycleTests(unittest.TestCase):
    """The canvas sends start/stop/info to every card on the project.

    A card whose dispatch returns None for one of them is reported to Agent
    Core as an unknown action, and a strict project start then rolls the whole
    project back — so a delegated gesture card has to forward the verb rather
    than fall off the end of dispatch.
    """

    class _Control:
        def __init__(self):
            self.actions = []

        def dispatch(self, action, args):
            self.actions.append(action)
            return {"state": "ready", "delegated": action}

    def test_gesture_cards_forward_the_lifecycle_verbs(self):
        control = self._Control()
        for card in (ArmGesturePlugin(control), HandGesturePlugin(control)):
            for action in ("start", "info", "stop"):
                result = card.dispatch(action, {})
                self.assertIsNotNone(result, f"{type(card).__name__}.{action}")
                self.assertIn("state", result, f"{type(card).__name__}.{action}")
        self.assertEqual(["start", "info", "stop"] * 2, control.actions)

    def test_unknown_gesture_still_declines(self):
        control = self._Control()
        self.assertIsNone(ArmGesturePlugin(control).dispatch("fly", {}))
        self.assertIsNone(HandGesturePlugin(control).dispatch("fly", {}))
        self.assertEqual([], control.actions)

    def test_arm_controller_answers_the_lifecycle_itself(self):
        publisher = _FakePublisher()
        control = _prime_arm_plugin(publisher)
        self.assertEqual({"state": "ready"}, control.dispatch("start", {}))
        self.assertIn("state", control.dispatch("info", {}))
        self.assertEqual("idle", control.dispatch("stop", {})["state"])

    def test_start_after_stop_restarts_the_lowcmd_writer(self):
        # A canvas stop tears the writer thread down (the stop event is set and
        # the thread cleared).  A subsequent start used to answer {"state":
        # "ready"} without bringing the thread back, so every later arm target
        # failed DDS_WRITE_FAILED until the container restarted.  The start
        # verb must be idempotent and revive the worker.
        publisher = _FakePublisher()
        control = _prime_arm_plugin(publisher)
        self.assertEqual({"state": "ready"}, control.dispatch("start", {}))
        first_thread = control._thread
        self.assertIsNotNone(first_thread)
        self.assertTrue(first_thread.is_alive())
        # A real gesture leaves the controller active, which is what makes a
        # subsequent stop actually tear the writer down.
        control._active = True
        control.dispatch("stop", {})
        self.assertIsNone(control._thread)
        # Second start must spawn a fresh, live thread.
        self.assertEqual({"state": "ready"}, control.dispatch("start", {}))
        self.assertIsNotNone(control._thread)
        self.assertTrue(control._thread.is_alive())
        self.assertIsNot(control._thread, first_thread)
        control.stop()


class HandSmoothTests(unittest.TestCase):
    def test_hand_ramps_toward_target_and_converges(self):
        old_flag = getattr(device, "HAS_PND_SDK", False)
        old_factory = getattr(device, "pnd_adam_msg_dds__HandCmd_", None)
        device.HAS_PND_SDK = True
        device.pnd_adam_msg_dds__HandCmd_ = lambda: types.SimpleNamespace(
            position=[0]*12)
        pub = _FakePublisher()
        try:
            plugin = HandPlugin({"transition_seconds": 0.1, "control_rate_hz": 100},
                                "", None, dds_hand_pub=pub)
            plugin._open_positions = [500] * 12
            target = [0] * 12
            result = plugin._activate(target, "close")
            self.assertEqual(result["state"], "active")
            # First activation seeds command from open positions.
            self.assertEqual(plugin._command_positions, [500] * 12)
            stop = plugin._control_stop_event

            # Let the control loop run for a few periods; it should move partway.
            time.sleep(0.05)
            stop.set()
            plugin._control_thread.join(1.0)
        finally:
            device.HAS_PND_SDK = old_flag
            if old_factory is None:
                del device.pnd_adam_msg_dds__HandCmd_
            else:
                device.pnd_adam_msg_dds__HandCmd_ = old_factory
        sent = [list(cmd.position) for cmd in pub.commands]
        self.assertGreater(len(sent), 1)
        # Consecutive writes monotonically approach the target (no snap).
        first, last = sent[0], sent[-1]
        for cmd in sent:
            self.assertTrue(all(first[i] >= cmd[i] >= last[i] for i in range(12)))
        # After enough time it should converge to the target.
        self.assertTrue(all(abs(cmd - 0) <= 60 for cmd in last))


if __name__ == "__main__":
    unittest.main()
