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
                    HandPlugin, HeadControlPlugin, WaistControlPlugin)
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
        for name in ("salute", "high_five", "handshake"):
            gestures = self._gesture()
            control = gestures._control
            result = gestures.dispatch(name, {})
            self.assertTrue(result["success"], f"{name}: {result}")
            self.assertEqual("right", result["side"], name)
            joints = self._targeted_joints(control)
            self.assertIn("elbow_Right", joints, name)
            self.assertNotIn("elbow_Left", joints, name)

    def test_selecting_the_left_arm_uses_the_mirrored_joint_set(self):
        for name in ("salute", "high_five", "handshake", "wave"):
            if name == "wave":
                continue  # covered by ArmWaveTests, which stubs the sequence
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
            if name == "wave":
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

    def test_tool_schema_advertises_the_new_actions(self):
        plugin = self.arm_plugin()
        schema = plugin.get_tool()["inputSchema"]
        actions = schema["properties"]["action"]["enum"]
        self.assertIn("set_joints", actions)
        self.assertIn("get_state", actions)
        self.assertIn("duration_s", schema["properties"])
        self.assertIn("joints", schema["properties"])
        self.assertIn("set_joints", schema["x-action-params"])
        advertised = schema["properties"]["pose"]["enum"]
        for pose, _, _ in ArmGesturePlugin._GESTURES.values():
            self.assertIn(pose, advertised)


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

    def test_waist_schema_advertises_its_actions(self):
        tool = WaistControlPlugin(self.arm_plugin()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual("waist_control", tool["name"])
        for action in ("set_roll", "set_pitch", "set_yaw", "reset", "stop", "info"):
            self.assertIn(action, schema["properties"]["action"]["enum"])
        self.assertEqual(["pitch_deg", "duration_s"],
                         schema["x-action-params"]["set_pitch"]["params"])

    def test_head_schema_advertises_its_actions(self):
        tool = HeadControlPlugin(self.arm_plugin()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual("head_control", tool["name"])
        for action in ("set_yaw", "set_pitch", "reset", "stop", "info"):
            self.assertIn(action, schema["properties"]["action"]["enum"])

    def test_head_set_yaw_targets_the_neck_joint(self):
        control = self.arm_plugin()
        head = HeadControlPlugin(control)
        result = head.dispatch("set_yaw", {"yaw_deg": -30})
        self.assertTrue(result["success"], result)
        self.assertIn(ADAM_PRO_JOINTS.index("neckYaw"), control._target_q)

    def test_waist_set_pitch_targets_the_waist_joint(self):
        control = self.arm_plugin()
        waist = WaistControlPlugin(control)
        result = waist.dispatch("set_pitch", {"pitch_deg": 20})
        self.assertTrue(result["success"], result)
        self.assertIn(ADAM_PRO_JOINTS.index("waistPitch"), control._target_q)

    def test_reset_returns_waist_to_the_hold_position(self):
        control = self.arm_plugin()
        waist = WaistControlPlugin(control)
        waist.dispatch("set_roll", {"roll_deg": 10})
        result = waist.dispatch("reset", {})
        self.assertTrue(result["success"], result)
        hold = control._hold_q
        for _, joint, _, _ in WAIST_JOINT_CONTROLS.values():
            index = ADAM_PRO_JOINTS.index(joint)
            self.assertAlmostEqual(control._target_q[index], hold[index], places=6)

    def test_duration_s_passes_through_to_the_segment(self):
        control = self.arm_plugin()
        head = HeadControlPlugin(control)
        result = head.dispatch("set_yaw", {"yaw_deg": -30, "duration_s": 2.0})
        self.assertTrue(result["success"], result)
        self.assertEqual(2.0, result["duration_s"])
        self.assertGreaterEqual(control._seg_span, 2.0)

    def test_head_rejects_out_of_range_angles(self):
        head = HeadControlPlugin(self.arm_plugin())
        result = head.dispatch("set_pitch", {"pitch_deg": 61})
        self.assertFalse(result["success"])
        self.assertEqual("INVALID_ARGUMENT", result["code"])

    def test_waist_rejects_an_unknown_action(self):
        waist = WaistControlPlugin(self.arm_plugin())
        self.assertIsNone(waist.dispatch("bogus", {}))

    def test_start_and_info_delegate_to_the_controller(self):
        control = self.arm_plugin()
        waist = WaistControlPlugin(control)
        self.assertEqual({"state": "ready"}, waist.dispatch("start", {}))
        self.assertIn("state", waist.dispatch("info", {}))


if __name__ == "__main__":
    unittest.main()
