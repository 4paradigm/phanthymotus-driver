"""ROS-free validation tests for the Adam human-facing arm interface."""

from __future__ import annotations

import math
import sys
import threading
import time
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

import device
import main as adam_main
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


class SampleAdapterTests(unittest.TestCase):
    def test_latest_sample_reader_owns_polling_channel_lifecycle(self):
        class _Channel:
            def __init__(self):
                self.init_count = 0
                self.close_count = 0
                self.read_timeouts = []

            def Init(self):
                self.init_count += 1

            def Read(self, timeout=None):
                self.read_timeouts.append(timeout)
                return types.SimpleNamespace(motor_state=[1, 2, 3])

            def Close(self):
                self.close_count += 1

        channel = _Channel()
        reader = adam_main._LatestSampleReader(channel)
        reader.Init()
        sample = reader.Read(timeout=0.2)
        reader.Close()

        self.assertEqual(1, channel.init_count)
        self.assertEqual([0.2], channel.read_timeouts)
        self.assertEqual([1, 2, 3], sample.motor_state)
        self.assertEqual(1, reader.diagnostics()["received"])
        self.assertEqual(3, reader.diagnostics()["last_motor_count"])
        self.assertEqual(1, channel.close_count)

    def test_latest_sample_reader_poll_timeout_does_not_count_as_sample(self):
        class _Channel:
            def Init(self):
                pass

            def Read(self, timeout=None):
                return None

            def Close(self):
                pass

        reader = adam_main._LatestSampleReader(_Channel())
        reader.Init()

        self.assertIsNone(reader.Read(timeout=0.01))
        self.assertEqual(0, reader.diagnostics()["received"])
        self.assertIsNone(reader.diagnostics()["last_motor_count"])

    def test_latest_sample_reader_delivers_only_the_newest_pending_sample(self):
        reader = adam_main._LatestSampleReader()
        reader.put("old")
        reader.put("new")

        self.assertEqual("new", reader.Read(timeout=0.01))
        self.assertIsNone(reader.Read(timeout=0.01))

    def test_latest_sample_reader_unblocks_when_callback_delivers_sample(self):
        reader = adam_main._LatestSampleReader()
        delivered = []
        waiting = threading.Thread(
            target=lambda: delivered.append(reader.Read(timeout=1.0)))
        waiting.start()
        reader.put("sample")
        waiting.join(1.0)

        self.assertFalse(waiting.is_alive())
        self.assertEqual(["sample"], delivered)

    def test_latest_sample_reader_reports_receive_diagnostics(self):
        reader = adam_main._LatestSampleReader()
        reader.put(types.SimpleNamespace(motor_state=[1, 2, 3]))

        diagnostics = reader.diagnostics()

        self.assertEqual(1, diagnostics["received"])
        self.assertEqual(3, diagnostics["last_motor_count"])
        self.assertIsNotNone(diagnostics["last_sample_age_s"])
        self.assertTrue(diagnostics["has_sample"])
        self.assertFalse(diagnostics["closed"])

    def test_latest_sample_reader_close_unblocks_waiting_readers(self):
        reader = adam_main._LatestSampleReader()
        delivered = []
        waiting = threading.Thread(
            target=lambda: delivered.append(reader.Read(timeout=10.0)))
        waiting.start()
        reader.Close()
        waiting.join(1.0)

        self.assertFalse(waiting.is_alive())
        self.assertEqual([None], delivered)
        self.assertTrue(reader.diagnostics()["closed"])

    def test_json_native_materializes_nested_iterable_containers(self):
        class _Repeated:
            def __iter__(self):
                return iter(("STOP", "STAND_WALK"))

        result = adam_main._json_native({
            "states": _Repeated(),
            "nested": (bytearray((1, 2)), {3: True}),
        })

        self.assertEqual({
            "states": ["STOP", "STAND_WALK"],
            "nested": [[1, 2], {"3": True}],
        }, result)

    def test_json_native_rejects_unknown_non_iterable_objects(self):
        with self.assertRaisesRegex(TypeError, "Unsupported MCP result type"):
            adam_main._json_native(object())


class ArmControlTests(unittest.TestCase):
    def test_complete_callback_state_captures_the_startup_pose(self):
        reader = adam_main._LatestSampleReader()
        plugin = ArmControlPlugin({}, "", None, dds_arm_lowstate_sub=reader)
        expected = [index / 100.0 for index in range(31)]
        reader.put(types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=value) for value in expected
        ]))

        plugin._read_initial_state()

        self.assertTrue(plugin._state_ready.is_set())
        self.assertEqual(expected, plugin._hold_q)
        self.assertEqual(expected, plugin._current_q)

    def test_incomplete_callback_state_reports_reader_diagnostics(self):
        reader = adam_main._LatestSampleReader()
        plugin = ArmControlPlugin({}, "", None, dds_arm_lowstate_sub=reader,
                                  dds_lowcmd_pub=_FakePublisher())
        reader.put(types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=0.0) for _ in range(30)
        ]))
        plugin._stop_event.set()

        plugin._read_initial_state()
        error = plugin._ready_error()

        self.assertFalse(plugin._state_ready.is_set())
        self.assertIsNone(plugin._hold_q)
        self.assertEqual("LOWSTATE_UNAVAILABLE", error["code"])
        self.assertIn("30 motors", error["message"])
        self.assertIn("received=1", error["message"])

    def test_no_callback_state_reports_zero_samples(self):
        reader = adam_main._LatestSampleReader()
        plugin = ArmControlPlugin({}, "", None, dds_arm_lowstate_sub=reader,
                                  dds_lowcmd_pub=_FakePublisher())
        plugin._stop_event.set()

        plugin._read_initial_state()
        error = plugin._ready_error()

        self.assertEqual("LOWSTATE_UNAVAILABLE", error["code"])
        self.assertIn("no rt/lowstate samples received", error["message"])
        self.assertIn("received=0", error["message"])

    def test_nonfinite_callback_state_does_not_mark_controller_ready(self):
        reader = adam_main._LatestSampleReader()
        plugin = ArmControlPlugin({}, "", None, dds_arm_lowstate_sub=reader)
        values = [0.0] * 31
        values[20] = float("nan")
        reader.put(types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=value) for value in values
        ]))
        plugin._stop_event.set()

        plugin._read_initial_state()

        self.assertFalse(plugin._state_ready.is_set())
        self.assertIsNone(plugin._hold_q)

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
            "waistYaw": 12, "waistRoll": 13, "waistPitch": 14,
            "shoulderPitch_Left": 15, "shoulderRoll_Left": 16,
            "shoulderYaw_Left": 17, "elbow_Left": 18,
            "wristRoll_Left": 19, "wristPitch_Left": 20,
            "wristYaw_Left": 21,
            "shoulderPitch_Right": 22, "shoulderRoll_Right": 23,
            "shoulderYaw_Right": 24, "elbow_Right": 25,
            "wristRoll_Right": 26, "wristPitch_Right": 27,
            "wristYaw_Right": 28,
            "neckYaw": 29, "neckPitch": 30,
        }
        for joint, index in expected.items():
            self.assertEqual(index, ADAM_PRO_JOINTS.index(joint), joint)

    def test_upper_body_pd_matches_vendor_low_level_profile(self):
        expected = {
            "shoulderPitch_Left": (18.0, 0.9),
            "shoulderRoll_Left": (9.0, 0.9),
            "shoulderYaw_Left": (9.0, 0.9),
            "elbow_Left": (9.0, 0.9),
            "wristRoll_Left": (9.0, 0.9),
            "wristPitch_Left": (9.0, 0.9),
            "wristYaw_Left": (9.0, 0.9),
            "shoulderPitch_Right": (18.0, 0.9),
            "shoulderRoll_Right": (9.0, 0.9),
            "shoulderYaw_Right": (9.0, 0.9),
            "elbow_Right": (9.0, 0.9),
            "wristRoll_Right": (9.0, 0.9),
            "wristPitch_Right": (9.0, 0.9),
            "wristYaw_Right": (9.0, 0.9),
        }
        for joint, gains in expected.items():
            self.assertEqual(gains, ArmControlPlugin._pd_for_joint(joint), joint)

    def test_each_joint_has_a_distinct_action_and_angle_field(self):
        self.assertEqual("left_elbow", ARM_ACTIONS["set_left_elbow"])
        self.assertEqual("right_wrist_roll", ARM_ACTIONS["set_right_wrist_roll"])
        self.assertEqual(len(ARM_JOINT_CONTROLS), len(ARM_ACTIONS))

    def test_reset_is_advertised_and_restores_the_startup_arm_pose(self):
        plugin = _prime_arm_plugin(_FakePublisher())
        tool = plugin.get_tool()
        self.assertIn("reset", tool["inputSchema"]["properties"]["action"]["enum"])
        self.assertEqual(
            ["duration_s"],
            tool["inputSchema"]["x-action-params"]["reset"]["params"],
        )

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin.start()
            plugin._target_q = {
                ADAM_PRO_JOINTS.index("shoulderPitch_Left"): math.radians(-90),
            }
            result = plugin.dispatch("reset", {"duration_s": 3.0})
        finally:
            plugin._stop_event.set()
            if plugin._thread is not None:
                plugin._thread.join(1.0)
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        self.assertTrue(result["success"], result)
        self.assertEqual(len(ARM_JOINT_CONTROLS), result["joints_set"])
        self.assertEqual(3.0, result["duration_s"])
        for _, joint, _, _ in ARM_JOINT_CONTROLS.values():
            index = ADAM_PRO_JOINTS.index(joint)
            self.assertAlmostEqual(plugin._hold_q[index], plugin._target_q[index])

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
        self.assertAlmostEqual(math.radians(-30), command.motor_cmd[15].q)
        self.assertAlmostEqual(math.radians(20), command.motor_cmd[16].q)
        self.assertAlmostEqual(math.radians(10), command.motor_cmd[17].q)

    def test_upper_body_targets_write_only_verified_hardware_slots(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        groups = {
            range(12, 15): 0.12,
            range(15, 22): 0.15,
            range(22, 29): 0.22,
            range(29, 31): 0.29,
        }
        for slots, target in groups.items():
            plugin._target_q.update({slot: target for slot in slots})
        plugin._seg_start = {
            slot: plugin._seg_current[slot] for slot in plugin._target_q
        }
        plugin._seg_started_at = time.monotonic() - plugin._seg_span
        plugin._active = True

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
        for slot in range(12):
            self.assertEqual(plugin._hold_q[slot], command.motor_cmd[slot].q)
        for slots, target in groups.items():
            for slot in slots:
                self.assertAlmostEqual(target, command.motor_cmd[slot].q)
                expected_kp, expected_kd = plugin._pd_for_joint(
                    ADAM_PRO_JOINTS[slot])
                self.assertEqual(expected_kp, command.motor_cmd[slot].kp)
                self.assertEqual(expected_kd, command.motor_cmd[slot].kd)

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
        self.assertEqual(command.motor_cmd[hip].kp, 305.0)
        self.assertEqual(command.motor_cmd[hip].kd, 6.1)
        self.assertLess(command.motor_cmd[elbow].q, plugin._hold_q[elbow])
        self.assertEqual(command.motor_cmd[elbow].kp, 9.0)
        self.assertEqual(command.motor_cmd[elbow].kd, 0.9)

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

    def test_release_clears_segment_state_before_the_next_writer_tick(self):
        publisher = _FakePublisher()
        plugin = _prime_arm_plugin(publisher)
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        plugin._active = True
        plugin._streaming = True
        plugin._target_q[elbow] = -0.5
        plugin._seg_start[elbow] = plugin._seg_current[elbow]
        plugin._release_started_at = time.monotonic() - 2.0

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        self.assertEqual(2, len(publisher.commands))
        self.assertEqual({}, plugin._target_q)
        self.assertEqual({}, plugin._seg_start)

    def test_completed_release_does_not_clear_a_newer_command(self):
        plugin = _prime_arm_plugin(None)
        old_elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        new_elbow = ADAM_PRO_JOINTS.index("elbow_Right")
        plugin._active = True
        plugin._streaming = True
        plugin._target_q[old_elbow] = -0.5
        plugin._seg_start[old_elbow] = plugin._seg_current[old_elbow]
        plugin._release_started_at = time.monotonic() - 2.0

        class _RetargetingPublisher(_FakePublisher):
            def Write(self, command, **kwargs):
                super().Write(command, **kwargs)
                with plugin._lock:
                    plugin._command_generation += 1
                    plugin._target_q = {new_elbow: 0.25}
                    plugin._seg_start = {
                        new_elbow: plugin._seg_current[new_elbow],
                    }
                    plugin._release_started_at = None
                    plugin._active = True

        publisher = _RetargetingPublisher()
        plugin._publisher = publisher
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        self.assertTrue(plugin._active)
        self.assertEqual({new_elbow: 0.25}, plugin._target_q)
        self.assertIn(new_elbow, plugin._seg_start)

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
