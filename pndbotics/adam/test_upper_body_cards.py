"""ROS-free contract tests for the Adam upper-body cards.

Covers the behaviour the canvas and Agent Core rely on for ``arm_control``,
``arm_gesture``, ``hand`` and ``hand_gesture``: gesture shapes that are
physically distinguishable, arm poses that stay inside the vendor limit table,
the default arm for symmetric versus one-armed gestures, the bulk actions added
for multi-joint and multi-channel moves, and the completion contract for the
asynchronous wave.
"""

from __future__ import annotations

import math
import threading
import sys
import time
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

import device
from device import (ADAM_PRO_JOINTS, ARM_JOINT_CONTROLS, ARM_POSES,
                    HAND_CHANNEL_NAMES, HAND_DEFAULT_CLOSED,
                    HAND_DEFAULT_OPEN, HAND_DEFAULT_THUMB_CLOSE,
                    HEAD_JOINT_CONTROLS, WAIST_JOINT_CONTROLS,
                    ArmControlPlugin, ArmGesturePlugin, HandGesturePlugin,
                    HandPlugin, HeadControlPlugin, WaistControlPlugin,
                    AdamDeviceBundle)
from test_arm_controls import _FakePublisher, _fake_lowcmd, _prime_arm_plugin


class RunningArmMixin:
    """Run the card's real 50Hz worker against a fake rt/lowcmd writer.

    ``ArmControlPlugin._set_targets`` confirms a move by waiting for the worker
    to write the new target, so a card under test has to have its worker
    actually running; stubbing the write confirmation instead would leave the
    path that reports ``DDS_WRITE_FAILED`` untested.
    """

    def setUp(self):
        self._original_lowcmd = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        self._running = []
        super().setUp()

    def tearDown(self):
        super().tearDown()
        for plugin in self._running:
            plugin._stop_event.set()
            thread = plugin._thread
            if thread is not None:
                thread.join(1.0)
        if self._original_lowcmd is None:
            del device.pnd_adam_msg_dds__LowCmd_
        else:
            device.pnd_adam_msg_dds__LowCmd_ = self._original_lowcmd

    def arm_plugin(self):
        plugin = _prime_arm_plugin(_FakePublisher())
        plugin.start()
        self._running.append(plugin)
        return plugin


def _bare_hand_plugin():
    """A HandPlugin with only the shape math wired up (no DDS, no worker)."""
    plugin = HandPlugin.__new__(HandPlugin)
    plugin._max_val = 1000
    plugin._open_positions = list(HAND_DEFAULT_OPEN)
    plugin._close_positions = list(HAND_DEFAULT_CLOSED)
    plugin._thumb_close_positions = list(HAND_DEFAULT_THUMB_CLOSE)
    plugin._thumb_close_min_flex_position = 100
    return plugin


def _dispatchable_hand_plugin():
    """A HandPlugin whose activation is recorded instead of written to DDS."""
    plugin = _bare_hand_plugin()
    plugin.activated = []
    plugin._base_positions = lambda: list(HAND_DEFAULT_OPEN)

    def _activate(positions, action):
        plugin.activated.append((action, list(positions)))
        return {"state": "active", "action": action, "target": list(positions)}

    plugin._activate = _activate
    return plugin


class GestureShapeTests(unittest.TestCase):
    def test_every_hand_gesture_is_a_valid_six_channel_shape(self):
        control = _bare_hand_plugin()
        gestures = HandGesturePlugin(control)
        for name in HandGesturePlugin._GESTURES:
            shape = gestures._shape_for(name, "right")
            self.assertEqual(len(shape), 6, name)
            for value in shape:
                self.assertGreaterEqual(value, 0, name)
                self.assertLessEqual(value, control._max_val, name)
            self.assertEqual(shape, gestures._shape_for(name, "left"),
                             f"{name} must not depend on which hand plays it")

    def test_gestures_are_mutually_distinguishable(self):
        """Regression: `thumbs_up` used to reuse the fist vector verbatim."""
        control = _bare_hand_plugin()
        gestures = HandGesturePlugin(control)
        shapes = {name: tuple(gestures._shape_for(name, "right"))
                  for name in HandGesturePlugin._GESTURES}
        self.assertNotEqual(shapes["thumbs_up"], shapes["fist"])
        self.assertNotEqual(shapes["rock"], shapes["fist"])
        self.assertNotEqual(shapes["handshake"], shapes["fist"])
        self.assertEqual(len(set(shapes.values())), len(shapes),
                         f"two gestures share one shape: {shapes}")
        # A thumbs up extends the thumb instead of tucking it across the palm.
        thumb_flex = HAND_CHANNEL_NAMES.index("thumb_flex")
        thumb_rotate = HAND_CHANNEL_NAMES.index("thumb_rotate")
        self.assertEqual(shapes["thumbs_up"][thumb_flex], control._max_val)
        self.assertEqual(shapes["thumbs_up"][thumb_rotate], 0)
        self.assertLess(shapes["fist"][thumb_flex], shapes["thumbs_up"][thumb_flex])

    def test_labels_and_schema_cover_every_gesture(self):
        tool = HandGesturePlugin(_bare_hand_plugin()).get_tool()
        schema = tool["inputSchema"]
        for name in HandGesturePlugin._GESTURES:
            self.assertIn(name, schema["properties"]["action"]["enum"])
            self.assertIn(name, HandGesturePlugin._LABELS)
            self.assertIn(name, schema["x-action-params"])
        # stop/info are answered by dispatch and must be advertised too.
        for verb in ("stop", "info"):
            self.assertIn(verb, schema["properties"]["action"]["enum"])
        self.assertEqual(["action"], schema["required"])
        self.assertNotIn("side", schema["required"])

    def test_open_and_fist_follow_the_configured_profiles(self):
        """`open_palm`/`fist` must track config, not a hard-coded copy."""
        control = _bare_hand_plugin()
        control._open_positions = [700] * 12
        control._close_positions = [50] * 12
        gestures = HandGesturePlugin(control)
        self.assertEqual(gestures._shape_for("open_palm", "left"), [700] * 6)
        self.assertEqual(gestures._shape_for("fist", "right")[0:4], [50] * 4)


class ArmPoseLimitTests(unittest.TestCase):
    def test_advertised_poses_stay_inside_the_vendor_limits(self):
        mirrored = 0
        for pose, (_, values) in ARM_POSES.items():
            for control, degrees in values.items():
                self.assertIn(control, ARM_JOINT_CONTROLS,
                              f"{pose} uses unknown control {control}")
                _, _, minimum, maximum = ARM_JOINT_CONTROLS[control]
                self.assertGreaterEqual(degrees, minimum, f"{pose}.{control}")
                self.assertLessEqual(degrees, maximum, f"{pose}.{control}")
            # A one-armed pose is mirrored onto the other arm at dispatch time,
            # so the mirrored angles have to respect the mirrored limits too.
            for control, degrees in ArmControlPlugin.mirror_targets(values).items():
                _, _, minimum, maximum = ARM_JOINT_CONTROLS[control]
                self.assertGreaterEqual(degrees, minimum, f"mirror({pose}).{control}")
                self.assertLessEqual(degrees, maximum, f"mirror({pose}).{control}")
                mirrored += 1
        self.assertGreater(mirrored, 0)

    def test_mirroring_inverts_roll_and_yaw_only(self):
        mirrored = ArmControlPlugin.mirror_targets({
            "right_shoulder_pitch": -105.0, "right_shoulder_roll": -22.0,
            "right_shoulder_yaw": 25.0, "right_wrist_roll": -10.0,
            "waist_yaw": 5.0,
        })
        self.assertEqual(mirrored["left_shoulder_pitch"], -105.0)
        self.assertEqual(mirrored["left_shoulder_roll"], 22.0)
        self.assertEqual(mirrored["left_shoulder_yaw"], -25.0)
        self.assertEqual(mirrored["left_wrist_roll"], 10.0)
        # A control without a left_/right_ prefix passes through untouched
        # (the waist now lives on its own card, but the mirror helper still
        # must not touch non-limbed names).
        self.assertEqual(mirrored["waist_yaw"], 5.0)


class ArmGestureRoutingTests(RunningArmMixin, unittest.TestCase):
    def _gesture(self):
        return ArmGesturePlugin(self.arm_plugin())

    @staticmethod
    def _targeted_joints(control):
        return {ADAM_PRO_JOINTS[index] for index in control._target_q}

    def test_symmetric_gestures_default_to_both_arms(self):
        for name in ("welcome", "raise", "reset"):
            gestures = self._gesture()
            control = gestures._control
            result = gestures.dispatch(name, {})
            self.assertTrue(result["success"], f"{name}: {result}")
            self.assertEqual("both", result["side"], name)
            joints = self._targeted_joints(control)
            self.assertIn("elbow_Left", joints, f"{name} drove only one arm")
            self.assertIn("elbow_Right", joints, f"{name} drove only one arm")

    def test_one_armed_gestures_default_to_the_right_arm(self):
        for name in ("salute", "high_five"):
            gestures = self._gesture()
            control = gestures._control
            result = gestures.dispatch(name, {})
            self.assertTrue(result["success"], f"{name}: {result}")
            self.assertEqual("right", result["side"], name)
            joints = self._targeted_joints(control)
            self.assertIn("elbow_Right", joints, name)
            self.assertNotIn("elbow_Left", joints, name)
            gestures._cancel_sequence()

    def test_selecting_the_left_arm_uses_the_mirrored_joint_set(self):
        for name in ("salute", "high_five"):
            right = self._gesture()
            self.assertTrue(right.dispatch(name, {"side": "right"})["success"])
            left = self._gesture()
            self.assertTrue(left.dispatch(name, {"side": "left"})["success"])
            self.assertNotEqual(self._targeted_joints(right._control),
                                self._targeted_joints(left._control), name)
            self.assertEqual(
                {joint.replace("Right", "Left") for joint
                 in self._targeted_joints(right._control)},
                self._targeted_joints(left._control), name)
            right._cancel_sequence()
            left._cancel_sequence()

    def test_one_armed_gesture_rejects_both(self):
        result = self._gesture().dispatch("salute", {"side": "both"})
        self.assertFalse(result["success"])
        self.assertEqual("INVALID_ARGUMENT", result["code"])
        self.assertIn("one-armed", result["message"])

    def test_salute_and_high_five_are_different_poses(self):
        """Regression: these used to resolve to one shared pose."""
        poses = {name: pose for name, (pose, _, _)
                 in ArmGesturePlugin._GESTURES.items()}
        self.assertEqual(len(set(poses.values())), len(poses), poses)

    def test_semantic_gestures_play_at_a_slower_velocity_ceiling(self):
        """Semantic gestures budget against 0.3 rad/s, not the raw card's 0.5.

        A salute or welcome is a performance, so the same joint travel must
        span a longer, calmer duration than a bare ``arm_control`` target.
        """
        plugin = self.arm_plugin()
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        hold = plugin._hold_q[shoulder]
        distance = 1.0  # rad, well past the 2 s smoothing floor
        plugin._set_targets({"shoulderPitch_Left": hold - distance})
        default_span = plugin._seg_span
        plugin._set_targets(
            {"shoulderPitch_Left": hold - distance},
            velocity_limit=ArmGesturePlugin._GESTURE_VELOCITY_RAD_S)
        slow_span = plugin._seg_span
        self.assertGreater(slow_span, default_span)
        # Reproduce the exact travel from the segment snapshot so the worker
        # cannot perturb the comparison.
        max_distance = max(
            abs(plugin._target_q[index] - plugin._seg_start[index])
            for index in plugin._target_q)
        expected = (ArmControlPlugin._EASE_PEAK_RATE * max_distance
                    / ArmGesturePlugin._GESTURE_VELOCITY_RAD_S)
        self.assertAlmostEqual(slow_span, expected, places=6)

    def test_a_caller_duration_still_clamps_below_the_gesture_ceiling(self):
        """duration_s may only slow a gesture; it cannot beat the 0.3 ceiling."""
        gestures = self._gesture()
        control = gestures._control
        result = gestures.dispatch("salute", {"duration_s": 0.1})
        self.assertTrue(result["success"], result)
        span = control._seg_span
        self.assertGreaterEqual(span, 0.1)
        # Reproduce the exact travel the controller budgeted: the segment start
        # is a snapshot, so the worker cannot perturb this measurement.
        max_distance = max(
            abs(control._target_q[index] - control._seg_start[index])
            for index in control._target_q)
        expected = (ArmControlPlugin._EASE_PEAK_RATE * max_distance
                    / ArmGesturePlugin._GESTURE_VELOCITY_RAD_S)
        self.assertAlmostEqual(span, expected, places=6)

    def test_reset_returns_the_selected_arm_to_its_zero_target(self):
        gestures = self._gesture()
        control = gestures._control
        result = gestures.dispatch("reset", {"side": "left"})
        self.assertTrue(result["success"], result)
        self.assertEqual(control._target_q[ADAM_PRO_JOINTS.index("elbow_Left")],
                         0.0)
        self.assertNotIn(ADAM_PRO_JOINTS.index("elbow_Right"), control._target_q)

    def test_no_gesture_ever_selects_an_empty_joint_set(self):
        # `wave` is skipped here: it plays in the background, and a worker left
        # running would report into whatever test is executing when it ends
        # (ArmWaveTests covers it with the sequence stubbed short).
        for name in ArmGesturePlugin._GESTURES:
            if name in ("wave", "handshake"):
                continue
            for side in ("left", "right", "both"):
                gestures = self._gesture()
                result = gestures.dispatch(name, {"side": side})
                self.assertIsNotNone(result, f"{name}/{side}")
                if result.get("success"):
                    self.assertTrue(gestures._control._target_q, f"{name}/{side}")


class ArmBulkActionTests(RunningArmMixin, unittest.TestCase):
    def test_set_joints_accepts_several_joints_in_one_segment(self):
        plugin = self.arm_plugin()
        requested = {"left_elbow": -60, "right_elbow": -60,
                     "left_shoulder_pitch": -30}
        result = plugin.dispatch("set_joints", {"joints": requested})
        self.assertTrue(result["success"], result)
        self.assertEqual(3, result["joints_set"])
        self.assertEqual(set(requested), set(result["joints_deg"]))
        for name in ("elbow_Left", "elbow_Right", "shoulderPitch_Left"):
            self.assertIn(ADAM_PRO_JOINTS.index(name), plugin._target_q)
        self.assertAlmostEqual(
            plugin._target_q[ADAM_PRO_JOINTS.index("elbow_Left")],
            math.radians(-60))
        # One call opens one segment, so all three joints ease together.
        self.assertEqual(3, len(plugin._seg_start))

    def test_set_joints_rejects_empty_unknown_and_out_of_range_input(self):
        plugin = self.arm_plugin()
        for payload in ({}, None, [], {"left_elbow": 999}, {"nope": 0}):
            result = plugin.dispatch("set_joints", {"joints": payload})
            self.assertFalse(result["success"], payload)
            self.assertEqual("INVALID_ARGUMENT", result["code"], payload)

    def test_set_targets_refuses_an_empty_selection(self):
        plugin = self.arm_plugin()
        error = plugin._set_targets({})
        self.assertFalse(error["success"])
        self.assertFalse(plugin._active)

    def test_get_state_reports_degrees_and_settling(self):
        plugin = self.arm_plugin()
        plugin.dispatch("set_joints", {"joints": {"left_elbow": -45}})
        state = plugin.dispatch("get_state", {})
        elbow = state["joints"]["left_elbow"]
        self.assertEqual(elbow["target_deg"], -45.0)
        self.assertEqual(elbow["limits_deg"],
                         {"minimum": ARM_JOINT_CONTROLS["left_elbow"][2],
                          "maximum": ARM_JOINT_CONTROLS["left_elbow"][3]})
        self.assertFalse(state["settled"])
        self.assertEqual(1, state["tracking_joint_count"])
        # Joints that were never commanded report a null target.
        self.assertIsNone(state["joints"]["right_elbow"]["target_deg"])
        self.assertIn("sensor cards", state["angle_source"])

        with plugin._lock:
            plugin._seg_current[ADAM_PRO_JOINTS.index("elbow_Left")] = \
                math.radians(-45)
        state = plugin.dispatch("get_state", {})
        self.assertTrue(state["settled"])
        self.assertEqual(0.0, state["joints"]["left_elbow"]["error_deg"])

    def test_duration_s_can_slow_but_never_speed_up_a_move(self):
        ease = ArmControlPlugin._ease
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        samples = 400
        # 0.1 is the fastest a caller may ask for; the driver still clamps the
        # 1.2 rad move up to the velocity-limited span.
        for requested, distance in ((0.1, 1.2), (0.1, 0.05), (12.0, 0.05)):
            plugin = self.arm_plugin()
            hold = plugin._hold_q[shoulder]
            result = plugin.dispatch("set_joints", {
                "joints": {"left_shoulder_pitch": math.degrees(hold - distance)},
                "duration_s": requested,
            })
            self.assertTrue(result["success"],
                            f"requested={requested} distance={distance}: {result}")
            span = plugin._seg_span
            self.assertGreaterEqual(span, requested)
            # Sample the eased profile the way the driver writes it: the peak
            # must never exceed the configured velocity limit.
            peak = 0.0
            previous = ease(0.0)
            for step in range(1, samples + 1):
                current = ease(step / samples)
                peak = max(peak, (current - previous) * distance * samples / span)
                previous = current
            self.assertLessEqual(peak, plugin._MAX_VELOCITY_RAD_S + 1e-6,
                                 f"requested {requested}s for {distance} rad "
                                 f"peaked at {peak} rad/s")

    def test_duration_s_lets_a_small_move_beat_the_smoothing_floor(self):
        plugin = self.arm_plugin()
        shoulder = ADAM_PRO_JOINTS.index("shoulderPitch_Left")
        hold = plugin._hold_q[shoulder]
        result = plugin.dispatch("set_joints", {
            "joints": {"left_shoulder_pitch": math.degrees(hold - 0.02)},
            "duration_s": 0.15,
        })
        self.assertTrue(result["success"], result)
        self.assertLess(plugin._seg_span, plugin._DEFAULT_TRANSITION_SECONDS)
        self.assertGreaterEqual(plugin._seg_span, 0.1)

    def test_without_duration_s_the_configured_transition_still_applies(self):
        plugin = self.arm_plugin()
        result = plugin.dispatch("set_left_shoulder_pitch",
                                 {"left_shoulder_pitch_deg": -20})
        self.assertTrue(result["success"], result)
        self.assertIsNone(result["duration_s"])
        self.assertAlmostEqual(plugin._DEFAULT_TRANSITION_SECONDS, plugin._seg_span)

    def test_duration_s_rejects_bad_values(self):
        plugin = self.arm_plugin()
        for bad in (0, -1, 61, float("nan"), "soon", True):
            result = plugin.dispatch("set_joints", {
                "joints": {"left_elbow": -30}, "duration_s": bad,
            })
            self.assertFalse(result["success"], bad)
            self.assertEqual("INVALID_ARGUMENT", result["code"], bad)

    def test_tool_schema_only_advertises_body_part_actions(self):
        schema = self.arm_plugin().get_tool()["inputSchema"]
        self.assertEqual(["set_shoulder", "set_elbow", "set_wrist"],
                         schema["properties"]["action"]["enum"])
        self.assertEqual(set(ArmControlPlugin._GROUP_JOINTS),
                         set(schema["x-action-params"]))
        self.assertNotIn("joints", schema["properties"])
        self.assertNotIn("pose", schema["properties"])

    def test_compound_verbs_are_advertised_with_their_degree_fields(self):
        plugin = self.arm_plugin()
        schema = plugin.get_tool()["inputSchema"]
        properties = schema["properties"]
        actions = properties["action"]["enum"]
        for verb in ArmControlPlugin._GROUP_JOINTS:
            self.assertIn(verb, actions)
            self.assertIn(verb, schema["x-action-params"])
        # Every advertised field must exist as a property, otherwise a strict
        # canvas client rejects the payload before it reaches the driver.
        for verb, params in schema["x-action-params"].items():
            if verb not in ArmControlPlugin._GROUP_JOINTS:
                continue
            for field in params["params"]:
                self.assertIn(field, properties, (verb, field))
        self.assertEqual(["left", "right"], properties["side"]["enum"])
        self.assertEqual(
            set(ArmControlPlugin._GROUP_JOINTS),
            {verb for verb in schema["x-action-params"]
             if verb in ArmControlPlugin._GROUP_JOINTS})

    def test_compound_angle_descriptions_show_both_side_limits(self):
        properties = self.arm_plugin().get_tool()["inputSchema"]["properties"]
        expected = {
            "pitch_deg": ("左臂范围 [-207, 117] 度", "右臂范围 [-207, 117] 度"),
            "roll_deg": ("左臂范围 [-36, 160] 度", "右臂范围 [-160, 36] 度"),
            "yaw_deg": ("左臂范围 [-148, 148] 度", "右臂范围 [-148, 148] 度"),
            "bend_deg": ("左臂范围 [-143, 12] 度", "右臂范围 [-143, 12] 度"),
            "wrist_yaw_deg": ("左臂范围 [-153, 153] 度", "右臂范围 [-153, 153] 度"),
            "wrist_pitch_deg": ("左臂范围 [-55, 55] 度", "右臂范围 [-55, 55] 度"),
            "wrist_roll_deg": ("左臂范围 [-55, 55] 度", "右臂范围 [-55, 55] 度"),
        }
        for field, limits in expected.items():
            description = properties[field]["description"]
            for limit in limits:
                self.assertIn(limit, description, field)

    def test_compound_verbs_reach_several_joints_in_one_segment(self):
        plugin = self.arm_plugin()
        result = plugin.dispatch("set_shoulder", {
            "side": "left", "pitch_deg": -30, "roll_deg": 20, "yaw_deg": 10})
        self.assertTrue(result["success"], result)
        self.assertEqual("left", result["side"])
        self.assertEqual(3, result["joints_set"])
        # The compound verbs share the same smoothing segment machinery as
        # set_joints: every selected joint gets an eased start and the target
        # register is updated under the worker lock.
        self.assertEqual(3, len(plugin._seg_start))
        for name in ("shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left"):
            self.assertIn(ADAM_PRO_JOINTS.index(name), plugin._target_q)
        self.assertAlmostEqual(
            plugin._target_q[ADAM_PRO_JOINTS.index("shoulderPitch_Left")],
            math.radians(-30))
        # The worker interpolates straight away, so only final targets are
        # asserted, matching what test_set_joints_accepts_several_joints_in_one_segment
        # pins for the bulk action.

    def test_compound_verbs_default_to_the_right_arm_and_allow_partial_fields(self):
        plugin = self.arm_plugin()
        result = plugin.dispatch("set_elbow", {"bend_deg": -45})
        self.assertTrue(result["success"], result)
        self.assertEqual("right", result["side"])
        index = ADAM_PRO_JOINTS.index("elbow_Right")
        self.assertAlmostEqual(plugin._target_q[index], math.radians(-45))

    def test_compound_verbs_reject_bad_input(self):
        plugin = self.arm_plugin()
        cases = (
            ("set_elbow", {"side": "both", "bend_deg": 0}, "left or right"),
            ("set_shoulder", {"side": "left"}, "at least one"),
            ("set_wrist", {}, "at least one"),
            # The roll axis is mirrored, so a value legal on one side is out of
            # range on the other; the dispatcher must enforce the chosen side.
            ("set_shoulder", {"side": "left", "roll_deg": -40}, "left_shoulder_roll"),
            ("set_shoulder", {"side": "right", "roll_deg": 40}, "right_shoulder_roll"),
        )
        for action, payload, fragment in cases:
            result = plugin.dispatch(action, payload)
            self.assertFalse(result["success"], (action, payload))
            self.assertEqual("INVALID_ARGUMENT", result["code"], (action, payload))
            self.assertIn(fragment, result["message"], (action, payload))


class ArmGestureHandBindingTests(RunningArmMixin, unittest.TestCase):
    """Bound gestures drive the arm and the hand in one call."""

    def _pair(self):
        control = self.arm_plugin()
        hand = HandGesturePlugin(_dispatchable_hand_plugin())
        return control, hand, ArmGesturePlugin(control, hand=hand)

    def test_a_bound_gesture_applies_its_hand_shape(self):
        _, hand, gestures = self._pair()
        result = gestures.dispatch("salute", {"side": "right"})
        self.assertTrue(result["success"], result)
        self.assertEqual("flat_hand", result["hand_gesture"])
        # HandGesturePlugin.dispatch(action="flat_hand", {"side": "right"})
        # lands exactly one recorded activation driven from the bound shape.
        self.assertEqual(1, len(hand._control.activated))
        action, positions = hand._control.activated[0]
        self.assertEqual("flat_hand", action)
        expected = HandGesturePlugin(hand._control)._shape_for("flat_hand", "right")
        self.assertEqual(expected, positions[6:12])

    def test_unbound_gestures_leave_the_hand_alone(self):
        _, hand, gestures = self._pair()
        result = gestures.dispatch("welcome", {"side": "both"})
        self.assertTrue(result["success"], result)
        self.assertNotIn("hand_gesture", result)
        self.assertEqual([], hand._control.activated)

    def test_the_pairing_is_optional(self):
        control = self.arm_plugin()
        gestures = ArmGesturePlugin(control)
        result = gestures.dispatch("salute", {"side": "right"})
        self.assertTrue(result["success"], result)
        self.assertNotIn("hand_gesture", result)

    def test_a_hand_failure_is_reported_after_the_arm_accepts(self):
        for failure in (
            {"state": "error", "error": "DDS_UNAVAILABLE",
             "message": "rt/handcmd down"},
            {"success": False, "code": "DDS_UNAVAILABLE",
             "message": "rt/handcmd down"},
        ):
            _, hand, gestures = self._pair()
            hand._control._activate = lambda _positions, _action: failure.copy()
            result = gestures.dispatch("salute", {"side": "right"})
            self.assertFalse(result["success"], result)
            self.assertEqual("DDS_UNAVAILABLE", result["code"])
            self.assertTrue(result["arm_accepted"])
            self.assertIn("hand shape failed", result["message"])

    def test_failed_hand_does_not_start_a_wave_sequence(self):
        _, hand, gestures = self._pair()
        hand._control._activate = lambda _positions, _action: {
            "success": False, "code": "DDS_UNAVAILABLE"}
        result = gestures.dispatch("wave", {"side": "right"})
        self.assertFalse(result["success"], result)
        self.assertTrue(result["arm_accepted"])
        self.assertIsNone(gestures._sequence_id)

    def test_every_bound_shape_is_a_real_hand_gesture(self):
        for gesture, shape in ArmGesturePlugin._GESTURE_HAND_SHAPES.items():
            self.assertIn(shape, HandGesturePlugin._GESTURES, gesture)
            self.assertIn(gesture, ArmGesturePlugin._GESTURES, gesture)


class ArmWaveTests(RunningArmMixin, unittest.TestCase):
    """`wave` returns before it finishes, so it owes Agent Core a completion."""

    def setUp(self):
        self.calls = []
        self._original_notify = device._notify_action_completion
        self._original_sequence = ArmGesturePlugin._WAVE_SEQUENCE
        self._original_lower = ArmGesturePlugin._WAVE_LOWER_SECONDS
        device._notify_action_completion = (
            lambda action_id, status, result, tool:
            self.calls.append((action_id, status, result, tool)))
        # Keep the test quick without touching the production rhythm.
        ArmGesturePlugin._WAVE_SEQUENCE = (("wave_out", 0.01), ("wave_in", 0.01))
        ArmGesturePlugin._WAVE_LOWER_SECONDS = 0.01
        super().setUp()

    def tearDown(self):
        super().tearDown()
        device._notify_action_completion = self._original_notify
        ArmGesturePlugin._WAVE_SEQUENCE = self._original_sequence
        ArmGesturePlugin._WAVE_LOWER_SECONDS = self._original_lower

    def _wait(self):
        deadline = time.monotonic() + 5.0
        while not self.calls and time.monotonic() < deadline:
            time.sleep(0.01)

    def test_wave_declares_completion(self):
        plugin = self.arm_plugin()
        schema = ArmGesturePlugin(plugin).get_tool()["inputSchema"]
        self.assertIn("wave", schema["x-completion"]["actions"])
        self.assertGreater(schema["x-completion"]["timeout"], 0)

    def test_wave_reports_completion_after_lowering_the_arm(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 0.01
        result = ArmGesturePlugin(control).dispatch("wave", {})
        self.assertTrue(result["success"], result)
        self.assertIn("action_id", result)
        self.assertTrue(result["auto_lower"])
        self.assertEqual("right", result["side"])
        self.assertGreaterEqual(result["sequence_segments"], 2)

        self._wait()
        self.assertEqual(1, len(self.calls),
                         "wave must report exactly one completion")
        action_id, status, payload, tool = self.calls[0]
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertEqual("arm_gesture", tool)
        self.assertEqual("wave", payload["gesture"])
        # The arm is lowered again, so the final target is the neutral one.
        self.assertEqual(0.0, control._target_q[ADAM_PRO_JOINTS.index("elbow_Right")])

    def test_stop_cancels_a_running_wave(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 5.0
        gestures = ArmGesturePlugin(control)
        gestures._WAVE_SEQUENCE = (("wave_out", 5.0),)
        result = gestures.dispatch("wave", {})
        self.assertTrue(result["success"], result)
        time.sleep(0.05)
        gestures.dispatch("stop", {})
        self._wait()
        self.assertEqual(1, len(self.calls))
        self.assertEqual("cancelled", self.calls[0][1])

    def test_a_new_wave_supersedes_the_running_one(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 0.01
        gestures = ArmGesturePlugin(control)
        gestures._WAVE_SEQUENCE = (("wave_out", 0.3), ("wave_in", 0.3))
        first = gestures.dispatch("wave", {})
        time.sleep(0.05)
        second = gestures.dispatch("wave", {})
        self.assertNotEqual(first["action_id"], second["action_id"])
        deadline = time.monotonic() + 5.0
        while len(self.calls) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        # Each wave owns its own action id and reports exactly once: the
        # superseded one finishes as cancelled, the live one as completed.
        statuses = {call[0]: call[1] for call in self.calls}
        self.assertEqual("cancelled", statuses.get(first["action_id"]))
        self.assertEqual("completed", statuses.get(second["action_id"]))

    def test_handshake_reports_completion_after_elbow_oscillation(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 0.01
        gestures = ArmGesturePlugin(control)
        gestures._HANDSHAKE_ELBOW_SEQUENCE = (-72.0, -88.0, -80.0)
        result = gestures.dispatch("handshake", {"side": "left"})
        self.assertTrue(result["success"], result)
        self.assertEqual(3, result["sequence_segments"])
        deadline = time.monotonic() + 5.0
        while not self.calls and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual("completed", self.calls[0][1])
        self.assertEqual("handshake", self.calls[0][2]["gesture"])
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        self.assertAlmostEqual(math.radians(-80), control._target_q[elbow])

    def test_superseded_wave_cannot_write_after_new_pose(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 0.01
        gestures = ArmGesturePlugin(control)
        gestures._WAVE_SEQUENCE = (("wave_out", 0.01),)
        entered = threading.Event()
        release = threading.Event()
        original_targets = gestures._targets_for

        def delayed_targets(pose, side):
            if pose == "wave_out":
                entered.set()
                self.assertTrue(release.wait(2.0))
            return original_targets(pose, side)

        gestures._targets_for = delayed_targets
        wave = gestures.dispatch("wave", {})
        self.assertTrue(entered.wait(2.0))
        pose_result = []
        pose_thread = threading.Thread(
            target=lambda: pose_result.append(
                gestures.dispatch("salute", {"side": "right"})))
        pose_thread.start()
        release.set()
        pose_thread.join(2.0)
        self.assertFalse(pose_thread.is_alive())
        self.assertTrue(pose_result[0]["success"], pose_result)
        deadline = time.monotonic() + 2.0
        while not self.calls and time.monotonic() < deadline:
            time.sleep(0.01)
        statuses = {call[0]: call[1] for call in self.calls}
        self.assertEqual("cancelled", statuses.get(wave["action_id"]))
        elbow = ADAM_PRO_JOINTS.index("elbow_Right")
        self.assertAlmostEqual(
            math.radians(ARM_POSES["salute"][1]["right_elbow"]),
            control._target_q[elbow])

    def test_a_pose_cancels_a_running_wave_before_setting_its_target(self):
        control = self.arm_plugin()
        control._active_segment_span = lambda: 5.0
        gestures = ArmGesturePlugin(control)
        wave = gestures.dispatch("wave", {})
        time.sleep(0.05)
        salute = gestures.dispatch("salute", {"side": "right"})
        self.assertTrue(salute["success"], salute)
        deadline = time.monotonic() + 5.0
        while not self.calls and time.monotonic() < deadline:
            time.sleep(0.01)
        statuses = {call[0]: call[1] for call in self.calls}
        self.assertEqual("cancelled", statuses.get(wave["action_id"]))
        elbow = ADAM_PRO_JOINTS.index("elbow_Right")
        self.assertAlmostEqual(
            math.radians(ARM_POSES["salute"][1]["right_elbow"]),
            control._target_q[elbow])


class HandBulkActionTests(unittest.TestCase):
    def test_grip_interpolates_between_the_open_and_close_shapes(self):
        plugin = _dispatchable_hand_plugin()

        opened = plugin.dispatch("grip", {"side": "left", "grip_percent": 0})
        self.assertEqual("active", opened["state"])
        self.assertEqual(HAND_DEFAULT_OPEN[0:6], plugin.activated[-1][1][0:6])

        closed = plugin.dispatch("grip", {"side": "right", "grip_percent": 100})
        self.assertEqual("active", closed["state"])
        self.assertEqual(plugin._close_target()[6:12],
                         plugin.activated[-1][1][6:12])

        half = plugin.dispatch("grip", {"side": "both", "grip_percent": 50})
        self.assertEqual("active", half["state"])
        target = plugin.activated[-1][1]
        self.assertEqual(target[0:6], target[6:12])
        self.assertTrue(all(0 <= value <= 1000 for value in target))

    def test_grip_rejects_out_of_range_and_non_numeric_input(self):
        plugin = _dispatchable_hand_plugin()
        for bad in (-1, 101, "half", None, True, float("inf")):
            result = plugin.dispatch("grip",
                                     {"side": "left", "grip_percent": bad})
            self.assertEqual("error", result["state"], bad)
            self.assertEqual("INVALID_ARGUMENT", result["error"], bad)
        for side in ("middle", None):
            result = plugin.dispatch("grip",
                                     {"side": side, "grip_percent": 50})
            self.assertEqual("error", result["state"], side)

    def test_set_positions_requires_one_vector_per_selected_hand(self):
        plugin = _dispatchable_hand_plugin()
        six = [1000, 1000, 1000, 1000, 1000, 0]
        twelve = six + six
        for side, payload, state in (
            ("left", six, "active"),
            ("right", six, "active"),
            ("both", twelve, "active"),
            ("both", six, "error"),
            ("left", twelve, "error"),
            ("middle", six, "error"),
        ):
            result = plugin.dispatch("set_positions",
                                     {"side": side, "positions": payload})
            self.assertEqual(state, result["state"], f"{side}/{len(payload)}")
        target = plugin.activated[-1][1]
        self.assertEqual(six, target[0:6])
        self.assertEqual(six, target[6:12])

    def test_open_and_close_accept_both_hands(self):
        plugin = _dispatchable_hand_plugin()
        for action in ("open", "close"):
            result = plugin.dispatch(action, {"side": "both"})
            self.assertEqual("active", result["state"], action)
            self.assertEqual("both", result["side"], action)
            target = plugin.activated[-1][1]
            self.assertEqual(target[0:6], target[6:12], action)

    def test_hand_schema_advertises_the_bulk_actions(self):
        plugin = _bare_hand_plugin()
        schema = plugin.get_tool()["inputSchema"]
        for action in ("grip", "set_positions"):
            self.assertIn(action, schema["properties"]["action"]["enum"])
            self.assertIn(action, schema["x-action-params"])
        self.assertIn("positions", schema["properties"])
        self.assertIn("grip_percent", schema["properties"])
        self.assertIn("both", schema["properties"]["side"]["enum"])

    def test_hand_gesture_both_applies_the_same_shape_twice(self):
        control = _dispatchable_hand_plugin()
        gestures = HandGesturePlugin(control)
        result = gestures.dispatch("point", {"side": "both"})
        self.assertEqual("active", result["state"])
        target = control.activated[-1][1]
        self.assertEqual(target[0:6], target[6:12])
        self.assertEqual(target[0:6], gestures._shape_for("point", "right"))


class WaistHeadControlTests(RunningArmMixin, unittest.TestCase):
    def test_waist_and_head_are_split_out_of_arm_control(self):
        # The waist joints left arm_control for a dedicated card; the neck was
        # never there.  Both now have their own controls and actions.
        self.assertNotIn("waist_roll", ARM_JOINT_CONTROLS)
        self.assertNotIn("waist_pitch", ARM_JOINT_CONTROLS)
        self.assertEqual({"roll", "pitch", "yaw"}, set(WAIST_JOINT_CONTROLS))
        self.assertEqual({"yaw", "pitch"}, set(HEAD_JOINT_CONTROLS))

    def test_waist_schema_only_advertises_angles_and_reset(self):
        tool = WaistControlPlugin(self.arm_plugin()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual("waist_control", tool["name"])
        self.assertEqual(["set_angles", "reset"],
                         schema["properties"]["action"]["enum"])
        self.assertEqual(["roll_deg", "pitch_deg", "yaw_deg", "duration_s"],
                         schema["x-action-params"]["set_angles"]["params"])

    def test_head_schema_only_advertises_angles_and_reset(self):
        tool = HeadControlPlugin(self.arm_plugin()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual("head_control", tool["name"])
        self.assertEqual(["set_angles", "reset"],
                         schema["properties"]["action"]["enum"])
        self.assertEqual(["yaw_deg", "pitch_deg", "duration_s"],
                         schema["x-action-params"]["set_angles"]["params"])

    def test_waist_and_head_angle_descriptions_show_exact_limits(self):
        cases = (
            (WaistControlPlugin(self.arm_plugin()), WAIST_JOINT_CONTROLS),
            (HeadControlPlugin(self.arm_plugin()), HEAD_JOINT_CONTROLS),
        )
        for plugin, controls in cases:
            properties = plugin.get_tool()["inputSchema"]["properties"]
            for control, (_, _, minimum, maximum) in controls.items():
                description = properties[f"{control}_deg"]["description"]
                self.assertIn(
                    f"范围 [{minimum:g}, {maximum:g}] 度", description,
                    (plugin.PREFIX, control))

    def test_head_set_angles_targets_multiple_neck_joints_together(self):
        control = self.arm_plugin()
        result = HeadControlPlugin(control).dispatch(
            "set_angles", {"yaw_deg": -30, "pitch_deg": 10})
        self.assertTrue(result["success"], result)
        self.assertEqual(2, result["joints_set"])
        self.assertEqual(2, len(control._seg_start))
        for joint in ("neckYaw", "neckPitch"):
            self.assertIn(ADAM_PRO_JOINTS.index(joint), control._target_q)

    def test_waist_set_angles_accepts_a_partial_selection(self):
        control = self.arm_plugin()
        result = WaistControlPlugin(control).dispatch(
            "set_angles", {"pitch_deg": 20})
        self.assertTrue(result["success"], result)
        self.assertEqual(1, result["joints_set"])
        self.assertIn(ADAM_PRO_JOINTS.index("waistPitch"), control._target_q)

    def test_angle_actions_reject_empty_or_out_of_range_input(self):
        cases = (
            WaistControlPlugin(self.arm_plugin()).dispatch("set_angles", {}),
            HeadControlPlugin(self.arm_plugin()).dispatch(
                "set_angles", {"pitch_deg": 61}),
        )
        for result in cases:
            self.assertFalse(result["success"])
            self.assertEqual("INVALID_ARGUMENT", result["code"])

    def test_reset_returns_waist_to_the_hold_position(self):
        control = self.arm_plugin()
        waist = WaistControlPlugin(control)
        waist.dispatch("set_angles", {"roll_deg": 10})
        result = waist.dispatch("reset", {})
        self.assertTrue(result["success"], result)
        for _, joint, _, _ in WAIST_JOINT_CONTROLS.values():
            index = ADAM_PRO_JOINTS.index(joint)
            self.assertAlmostEqual(control._target_q[index],
                                   control._hold_q[index], places=6)

    def test_duration_s_passes_through_to_the_segment(self):
        control = self.arm_plugin()
        result = HeadControlPlugin(control).dispatch(
            "set_angles", {"yaw_deg": -30, "duration_s": 2.0})
        self.assertTrue(result["success"], result)
        self.assertEqual(2.0, result["duration_s"])
        self.assertGreaterEqual(control._seg_span, 2.0)

    def test_start_and_info_delegate_to_the_controller(self):
        control = self.arm_plugin()
        waist = WaistControlPlugin(control)
        self.assertEqual({"state": "ready"}, waist.dispatch("start", {}))
        self.assertIn("state", waist.dispatch("info", {}))


class RemovedAxisGestureTests(unittest.TestCase):
    def test_config_and_marketplace_no_longer_list_axis_gesture_cards(self):
        base = __file__.rsplit("/", 1)[0]
        for filename in ("config.yaml", "driver.yaml"):
            with open(f"{base}/{filename}", encoding="utf-8") as stream:
                content = stream.read()
            self.assertNotIn("waist_gesture", content, filename)
            self.assertNotIn("head_gesture", content, filename)

    def test_bundle_does_not_register_axis_gesture_cards(self):
        disabled = {name: {"enabled": False} for name in (
            "state", "estop", "loco", "motion", "tracking_motion",
            "camera", "vision_capture", "hand", "hand_gesture",
            "hand_state", "model",
        )}
        config = {
            "variant": "pro",
            "plugins": {
                **disabled,
                "arm": {"enabled": True},
                "arm_gesture": {"enabled": False},
                "waist": {"enabled": True},
                "head": {"enabled": True},
            },
        }
        bundle = AdamDeviceBundle(
            config, "", None, None, dds_lowcmd_pub=_FakePublisher(),
            ros2_enabled=False)
        self.assertEqual(
            {"arm_control", "waist_control", "head_control"},
            set(bundle._tool_map))
        self.assertNotIn("waist_gesture", bundle._tool_map)
        self.assertNotIn("head_gesture", bundle._tool_map)


if __name__ == "__main__":
    unittest.main()
