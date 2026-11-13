import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_check import check_pose
from pose_session import PoseSession
from pose_mcp import PoseCard, tool_definition


def arms(up=True):
    return {f"{s}_{j}": (x, y, 1) for s, x in (("left", .4), ("right", .6))
            for j, y in (("shoulder", .5), ("elbow", .3 if up else .65),
                         ("wrist", .15 if up else .8))}


def legs(down=False):
    points = {"__image_size__": {"width": 640, "height": 480}}
    for side, x in (("left", .3), ("right", .6)):
        points.update({f"{side}_hip": (x, .55 if down else .4, 1),
                       f"{side}_knee": (x+.15 if down else x, .7 if down else .65, 1),
                       f"{side}_ankle": (x, .9, 1)})
    return points


def projected_squat():
    # A crouch viewed from above can make the 2D knee angle nearly zero even
    # though both hips descend and the feet remain in place.
    points = legs()
    for side, x in (("left", .3), ("right", .6)):
        points[f"{side}_hip"] = (x, .75, 1)
        points[f"{side}_knee"] = (x, .7, 1)
    return points


def straight_projected_squat():
    # The inferred knee can remain on the hip-to-ankle line while both legs
    # visibly contract and the hips descend.
    points = legs()
    for side, x in (("left", .3), ("right", .6)):
        points[f"{side}_hip"] = (x, .75, 1)
        points[f"{side}_knee"] = (x, .8, 1)
    return points


def height_pose(crouched=False, false_knee=False):
    points = legs()
    for side, x in (("left", .3), ("right", .6)):
        points[f"{side}_shoulder"] = (x, .45 if crouched else .15, 1)
        if crouched:
            points[f"{side}_hip"] = (x, .75, 1)
            points[f"{side}_knee"] = (x, .5, 1)
        elif false_knee:
            points[f"{side}_knee"] = (x, .3, 1)
    return points


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.frame = 0

    def clock(self):
        return self.now

    def feed(self, session, points, n=1):
        for _ in range(n):
            self.now += .25
            self.frame += 1
            result = session.update(points, self.frame)
        return result

    def test_hold_needs_continuous_fresh_frames(self):
        s = PoseSession("hands_up", hold_seconds=1, clock=self.clock)
        self.assertEqual(self.feed(s, arms(), 4)["status"], "almost")
        self.assertEqual(self.feed(s, arms())["state"], "completed")

    def test_repeated_frame_cannot_complete_hold(self):
        s = PoseSession("hands_up", hold_seconds=1, clock=self.clock)
        s.update(arms(), 1)
        self.now = 2
        result = s.update(arms(), 1)
        self.assertEqual(result["state"], "running")
        self.assertEqual(result["status"], "retry")
        self.assertEqual(result["progress"]["hold_seconds"], 0)

    def test_missing_points_reset_hold(self):
        s = PoseSession("hands_up", hold_seconds=1, clock=self.clock)
        self.feed(s, arms(), 4)
        self.feed(s, {})
        self.assertEqual(self.feed(s, arms(), 2)["state"], "running")

    def test_switching_raised_side_resets_hold(self):
        s = PoseSession("one_hand_up", hold_seconds=1, clock=self.clock)
        left = arms(False)
        left.update({k:v for k,v in arms().items() if k.startswith("left")})
        right = arms(False)
        right.update({k:v for k,v in arms().items() if k.startswith("right")})
        self.feed(s, left, 4)
        result = self.feed(s, right, 2)
        self.assertEqual(result["state"], "running")
        self.assertEqual(result["progress"]["hold_seconds"], .25)

    def test_timeout_without_any_frames(self):
        s = PoseSession("hands_up", hold_seconds=1, timeout_seconds=2, clock=self.clock)
        self.now = 3
        self.assertEqual(s.status()["state"], "timed_out")

    def test_squat_clock_starts_after_standing_calibration(self):
        s = PoseSession("squat", timeout_seconds=2, clock=self.clock)
        self.now = 3
        self.assertEqual(s.status()["state"], "running")
        self.assertFalse(s.status()["progress"]["calibrated"])
        self.assertEqual(s.status()["progress"]["elapsed_seconds"], 0.0)
        self.feed(s, legs(), 3)
        status = s.status()
        self.assertTrue(status["progress"]["calibrated"])
        self.assertLess(status["progress"]["elapsed_seconds"], 0.1)
        self.now += 2.1
        self.assertEqual(s.status()["state"], "timed_out")

    def test_squat_requires_standing_down_and_return(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(True), 4)
        self.assertEqual(s.count, 0)
        self.feed(s, legs(), 3)
        self.feed(s, legs(True), 3)
        self.assertEqual(s.count, 0)
        result = self.feed(s, legs(), 3)
        self.assertEqual(result["state"], "completed")
        self.assertEqual(result["progress"]["repetitions"], 1)
        self.assertEqual(result["progress"]["mode"], "repetitions")
        self.assertEqual(result["progress"]["target_seconds"], 0.0)
        self.assertEqual(result["feedback"], "深蹲完成，共 1 次")
        self.assertIsNotNone(result["progress"]["last_round_seconds"])

    def test_squat_counts_near_standing_return(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, legs(True), 3)
        # Return with a small residual bend and a hip height just outside the
        # old strict threshold, but inside the practical return tolerance.
        almost_standing = legs()
        for side, x in (("left", .3), ("right", .6)):
            almost_standing[f"{side}_hip"] = (x, .47, 1)
            almost_standing[f"{side}_knee"] = (x, .66, 1)
        result = self.feed(s, almost_standing, 3)
        self.assertEqual(result["progress"]["repetitions"], 1)

    def test_projected_leg_motion_counts_when_knee_angles_collapse(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        crouched = projected_squat()
        raw = check_pose(crouched, "squat")
        self.assertEqual(raw["phase"], "transition")
        self.assertLess(max(raw["knee_angles"]), 65)
        self.assertEqual(len(raw["shin_lengths"]), 2)
        self.assertEqual(len(raw["leg_lengths"]), 2)
        observed = self.feed(s, crouched, 2)
        self.assertEqual(observed["progress"]["phase"], "returning")
        self.assertEqual(observed["squat_evidence"], "projected_leg")
        result = self.feed(s, legs(), 3)
        self.assertEqual(result["progress"]["repetitions"], 1)

    def test_projected_leg_motion_counts_when_knees_look_straight(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        crouched = straight_projected_squat()
        self.assertEqual(check_pose(crouched, "squat")["phase"], "standing")
        observed = self.feed(s, crouched, 2)
        self.assertEqual(observed["progress"]["phase"], "returning")
        self.assertEqual(observed["squat_evidence"], "projected_leg")
        self.assertEqual(self.feed(s, legs(), 3)["progress"]["repetitions"], 1)

    def test_one_missing_frame_can_bridge_two_projected_down_observations(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, straight_projected_squat())
        self.now += .1
        self.frame += 1
        s.update({}, self.frame)
        self.now += .1
        self.frame += 1
        result = s.update(straight_projected_squat(), self.frame)
        self.assertEqual(result["progress"]["phase"], "returning")

    def test_single_projected_down_frame_does_not_count(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, straight_projected_squat())
        result = self.feed(s, legs(), 4)
        self.assertEqual(result["progress"]["repetitions"], 0)

    def test_height_motion_counts_when_knee_geometry_is_wrong(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, height_pose(), 3)
        crouched = height_pose(crouched=True)
        raw = check_pose(crouched, "squat")
        self.assertFalse(raw["matched"])
        self.assertGreater(raw["leg_length"], check_pose(height_pose(), "squat")["leg_length"])
        observed = self.feed(s, crouched, 2)
        self.assertEqual(observed["progress"]["phase"], "returning")
        self.assertEqual(observed["squat_evidence"], "body_height")
        # The standing knee angle can be wrong too; recovered body height and
        # hip position provide a second way to confirm the return.
        result = self.feed(s, height_pose(false_knee=True), 3)
        self.assertEqual(result["progress"]["repetitions"], 1)

    def test_height_motion_survives_occluded_knees_after_calibration(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, height_pose(), 3)
        crouched = height_pose(crouched=True)
        crouched.pop("left_knee")
        crouched.pop("right_knee")
        self.assertFalse(check_pose(crouched, "squat")["detected"])
        observed = self.feed(s, crouched, 2)
        self.assertEqual(observed["progress"]["phase"], "returning")
        self.assertEqual(observed["squat_evidence"], "body_height")
        self.assertEqual(self.feed(s, height_pose(), 3)["progress"]["repetitions"], 1)

    def test_static_squat_check_calibrates_and_detects_height_change(self):
        card = PoseCard(clock=self.clock)
        card.ingest(height_pose())
        first = card.dispatch("check", {"pose": "squat"})
        self.assertFalse(first["calibrated"])
        self.assertFalse(first["matched"])
        for _ in range(3):
            self.now += .2
            card.ingest(height_pose())
        self.assertTrue(card.selected_result()["calibrated"])
        self.assertTrue(card.dispatch("info", {})["check_calibrated"])
        for _ in range(3):
            self.now += .1
            card.ingest(height_pose(crouched=True))
        detected = card.selected_result()
        self.assertTrue(detected["matched"])
        self.assertEqual(detected["squat_evidence"], "body_height")
        self.assertEqual(detected["phase"], "down")
        self.assertEqual([e["event"] for e in card.drain_events()], ["pose_detected"])
        for _ in range(3):
            self.now += .1
            card.ingest(height_pose())
        self.assertEqual([e["event"] for e in card.drain_events()], ["pose_lost"])

    def test_static_squat_check_uses_height_when_knees_are_occluded(self):
        card = PoseCard(clock=self.clock)
        card.ingest(height_pose())
        card.dispatch("check", {"pose": "squat"})
        for _ in range(3):
            self.now += .2
            card.ingest(height_pose())
        crouched = height_pose(crouched=True)
        crouched.pop("left_knee")
        crouched.pop("right_knee")
        for _ in range(3):
            self.now += .1
            card.ingest(crouched)
        result = card.selected_result()
        self.assertTrue(result["matched"])
        self.assertEqual(result["squat_evidence"], "body_height")
        self.assertEqual(result["raw_phase"], "height_only")

    def test_height_check_does_not_match_bending_or_camera_shift(self):
        card = PoseCard(clock=self.clock)
        card.ingest(height_pose())
        card.dispatch("check", {"pose": "squat"})
        for _ in range(3):
            self.now += .2
            card.ingest(height_pose())
        bent = height_pose()
        for side, x in (("left", .3), ("right", .6)):
            bent[f"{side}_shoulder"] = (x, .45, 1)
        for _ in range(3):
            self.now += .1
            card.ingest(bent)
        self.assertFalse(card.selected_result()["matched"])
        shifted_feet = height_pose(crouched=True)
        for side, x in (("left", .3), ("right", .6)):
            shifted_feet[f"{side}_ankle"] = (x, .65, 1)
        for _ in range(3):
            self.now += .1
            card.ingest(shifted_feet)
        self.assertFalse(card.selected_result()["matched"])

    def test_one_collapsed_leg_does_not_count_as_squat(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        points = legs()
        points["left_hip"], points["left_knee"] = (.3, .75, 1), (.3, .72, 1)
        points["right_hip"], points["right_knee"] = (.6, .65, 1), (.6, .55, 1)
        raw = check_pose(points, "squat")
        self.assertLess(max(raw["knee_angles"]), 65)
        self.assertLess(raw["leg_length"], .7 * check_pose(legs(), "squat")["leg_length"])
        self.assertGreater(raw["leg_lengths"][1], .78 * .5)
        self.feed(s, points, 3)
        result = self.feed(s, legs(), 3)
        self.assertEqual(result["progress"]["repetitions"], 0)

    def test_prolonged_landmark_loss_does_not_finish_half_squat(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, legs(True), 3)
        self.feed(s, {}, 8)
        self.feed(s, legs(), 4)
        self.assertEqual(s.count, 0)

    def test_brief_landmark_loss_preserves_confirmed_down_phase(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, legs(True), 2)
        self.assertEqual(s.stage, "returning")
        self.feed(s, {}, 2)
        self.assertEqual(s.stage, "returning")
        result = self.feed(s, legs(), 3)
        self.assertEqual(result["progress"]["repetitions"], 1)

    def test_squat_grace_depends_on_elapsed_time_not_camera_fps(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        # At 15 fps, 12 missing frames last less than the 1.5 s grace.
        for _ in range(12):
            self.now += 1 / 15
            self.frame += 1
            result = s.update({}, self.frame)
        self.assertTrue(result["progress"]["calibrated"])
        self.assertEqual(result["progress"]["phase"], "ready")

    def test_isolated_ankle_drift_does_not_discard_calibration(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        noisy = legs()
        for side, x in (("left", .3), ("right", .6)):
            noisy[f"{side}_ankle"] = (x, .8, 1)
        result = self.feed(s, noisy, 6)
        self.assertTrue(result["progress"]["calibrated"])
        self.assertEqual(result["progress"]["phase"], "ready")

    def test_whole_body_shift_recalibrates_without_restarting_timeout(self):
        s = PoseSession("squat", timeout_seconds=3, clock=self.clock)
        self.feed(s, legs(), 3)
        shifted = legs()
        for side in ("left", "right"):
            for joint in ("hip", "knee", "ankle"):
                x, y, visibility = shifted[f"{side}_{joint}"]
                shifted[f"{side}_{joint}"] = (x, y - .1, visibility)
        result = self.feed(s, shifted, 4)
        self.assertFalse(result["progress"]["calibrated"])
        self.assertEqual(result["progress"]["phase"], "standing_required")
        self.assertGreater(result["progress"]["elapsed_seconds"], 0)
        self.assertIsNotNone(result["progress"]["remaining_seconds"])
        self.now = 4
        self.assertEqual(s.status()["state"], "timed_out")

    def test_prolonged_keypoint_loss_keeps_squat_timeout(self):
        s = PoseSession("squat", timeout_seconds=3, clock=self.clock)
        self.feed(s, legs(), 3)
        result = self.feed(s, {}, 8)
        self.assertFalse(result["progress"]["calibrated"])
        self.assertGreater(result["progress"]["elapsed_seconds"], 0)
        self.assertIsNotNone(result["progress"]["remaining_seconds"])
        self.now = 4
        self.assertEqual(s.status()["state"], "timed_out")

    def test_calibrated_squat_survives_brief_landmark_loss(self):
        s = PoseSession("squat", clock=self.clock)
        self.feed(s, legs(), 3)
        self.assertTrue(s.status()["progress"]["calibrated"])
        self.feed(s, {}, 4)
        recovering = s.status()
        self.assertEqual(recovering["state"], "running")
        self.assertTrue(recovering["progress"]["calibrated"])
        self.assertEqual(recovering["progress"]["phase"], "ready")
        self.feed(s, legs(True), 3)
        result = self.feed(s, legs(), 3)
        self.assertEqual(result["progress"]["repetitions"], 1)

    def test_stationary_pose_does_not_increment_repetitions(self):
        s = PoseSession("squat", repetitions=2, clock=self.clock)
        self.feed(s, legs(), 3)
        self.feed(s, legs(True), 3)
        self.feed(s, legs(), 12)
        self.assertEqual(s.count, 1)
        self.feed(s, legs(True), 3)
        self.assertEqual(self.feed(s, legs(), 3)["state"], "completed")

    def test_upper_pose_uses_repetitions_and_requires_release(self):
        s = PoseSession("hands_up", hold_seconds=0, repetitions=2, clock=self.clock)
        first = self.feed(s, arms())
        self.assertEqual(first["state"], "running")
        self.assertEqual(first["progress"]["repetitions"], 1)
        self.assertEqual(first["progress"]["phase"], "release_required")
        # Holding the same raised pose cannot become a second repetition.
        self.feed(s, arms(), 4)
        self.assertEqual(s.count, 1)
        self.feed(s, arms(False), 2)
        self.assertEqual(s.status()["progress"]["phase"], "holding")
        second = self.feed(s, arms())
        self.assertEqual(second["state"], "completed")
        self.assertEqual(second["progress"]["repetitions"], 2)

    def test_cancel_and_invalid_arguments(self):
        s = PoseSession("arms_open", clock=self.clock)
        s.cancel()
        self.assertEqual(self.feed(s, arms())["state"], "cancelled")
        for kw in ({"hold_seconds": float("nan")}, {"repetitions": 1.5}, {"timeout_seconds": -1}):
            with self.assertRaises(ValueError):
                PoseSession("hands_up", **kw)

    def test_squat_requires_dimensions_and_reliable_legs(self):
        p = legs(True)
        self.assertEqual(check_pose(p, "squat")["phase"], "down")
        p.pop("__image_size__")
        self.assertEqual(check_pose(p, "squat")["error"], "image_size_required")
        p["left_ankle"] = (.3, 1.2, 1)
        self.assertFalse(check_pose(p, "squat")["detected"])

    def test_squat_angles_invariant_under_resize(self):
        p = legs(True)
        first = check_pose(p, "squat")["knee_angles"]
        p["__image_size__"] = {"width": 1280, "height": 960}
        self.assertEqual(first, check_pose(p, "squat")["knee_angles"])

    def test_mcp_session_lifecycle_and_stale_input(self):
        card = PoseCard(clock=self.clock)
        self.assertIn("error", card.dispatch("begin", {"pose":"hands_up"}))
        card.ingest(arms())
        result = card.dispatch("begin", {"pose":"hands_up", "hold_seconds": .5})
        sid = result["session_id"]
        self.assertEqual(result["reason"], "waiting_for_frame")
        self.assertEqual(card.dispatch("status", {})["error"], "session_id_mismatch")
        self.assertEqual(card.dispatch("begin", {"pose":"squat"})["error"], "session_busy")
        for _ in range(5):
            self.now += .25
            card.ingest(arms())
        self.assertEqual(card.dispatch("status", {"session_id":sid})["state"], "completed")
        self.assertEqual(card.dispatch("status", {"session_id":sid})["progress"]["repetitions"], 1)
        self.now += 1
        self.assertIn("error", card.dispatch("check", {"pose":"hands_up"}))
        card.dispatch("stop", {})
        info = card.dispatch("info", {})
        self.assertEqual(info["state"], "idle")
        self.assertEqual(info["topic_out"], tool_definition()["topic_out"])
        json.dumps(tool_definition())

    def test_mcp_publishes_squat_rep_event_for_tts(self):
        card = PoseCard(clock=self.clock)
        card.ingest(legs())
        begin = card.dispatch("begin", {"pose": "squat", "repetitions": 2})
        self.assertEqual(begin["state"], "running")
        for points, count in ((legs(), 3), (legs(True), 3), (legs(), 3)):
            for _ in range(count):
                self.now += .25
                result = card.ingest(points)
        self.assertEqual(result["progress"]["repetitions"], 1)
        events = card.drain_events()
        rep_events = [event for event in events if event["event"] == "rep_completed"]
        self.assertEqual(len(rep_events), 1)
        self.assertIn("第 1 次深蹲完成，用时", rep_events[0]["narration"])
        self.assertEqual(events[0]["event"], "calibration_completed")
        self.assertEqual(card.drain_events(), [])

    def test_squat_begin_announces_calibration_once(self):
        card = PoseCard(clock=self.clock)
        card.ingest(legs())
        begin = card.dispatch("begin", {"pose": "squat", "repetitions": 3})
        self.assertEqual(begin["events"][0]["event"], "task_started")
        self.assertIn("站直", begin["narration"])

    def test_persistent_calibration_problem_does_not_repeat_event_by_timer(self):
        card = PoseCard(clock=self.clock)
        card.ingest(legs())
        card.dispatch("begin", {"pose": "squat", "repetitions": 3})
        for _ in range(8):
            self.now += 1
            card.ingest({})
        corrections = [event for event in card.drain_events()
                       if event["event"] == "correction"]
        self.assertEqual(len(corrections), 1)

    def test_squat_emits_midpoint_event_once(self):
        card = PoseCard(clock=self.clock)
        card.ingest(legs())
        card.dispatch("begin", {"pose": "squat", "repetitions": 3})
        for points in (legs(), legs(True), legs()):
            for _ in range(3):
                self.now += .25
                card.ingest(points)
        events = card.drain_events()
        midpoint = [event for event in events if event["event"] == "progress_milestone"]
        self.assertEqual(len(midpoint), 0)
        # Complete a second repetition; 2/3 is the midpoint milestone.
        for points in (legs(True), legs()):
            for _ in range(3):
                self.now += .25
                card.ingest(points)
        events = card.drain_events()
        midpoint = [event for event in events if event["event"] == "progress_milestone"]
        self.assertEqual(len(midpoint), 1)
        self.assertEqual(midpoint[0]["count"], 2)

    def test_selected_pose_requires_three_of_five_frames(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        card.dispatch("begin", {"pose": "hands_up", "hold_seconds": 3})
        for expected_stable, expected_matches in ((False, 1), (False, 2), (True, 3)):
            self.now += .1
            card.ingest(arms())
            result = card.selected_result()
            self.assertEqual(result["stable"], expected_stable)
            self.assertEqual(result["stable_matches"], expected_matches)
        self.assertEqual(card.selected_observation()["pose"], "hands_up")

    def test_check_mode_emits_detection_transitions_without_spam(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        first = card.dispatch("check", {"pose": "hands_up"})
        self.assertNotIn("events", first)
        for _ in range(2):
            self.now += .1
            card.ingest(arms())
        events = card.drain_events()
        self.assertEqual([event["event"] for event in events], ["pose_detected"])
        for _ in range(4):
            self.now += .1
            card.ingest(arms())
        self.assertEqual(card.drain_events(), [])
        for _ in range(3):
            self.now += .1
            card.ingest(arms(False))
        events = card.drain_events()
        self.assertEqual([event["event"] for event in events], ["pose_lost"])

    def test_cancel_exits_continuous_check_without_a_session(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        card.dispatch("check", {"pose": "hands_up"})
        result = card.dispatch("cancel", {})
        self.assertEqual(result, {"state": "idle", "event": "check_cancelled", "pose": "hands_up"})
        self.assertIsNone(card.selected_observation()["pose"])

    def test_upper_pose_counts_after_stability_without_hold_delay(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        begin = card.dispatch("begin", {"pose": "arms_open"})
        self.assertEqual(begin["state"], "running")
        open_arms = {
            "left_shoulder": (.4, .5, 1), "left_elbow": (.25, .5, 1), "left_wrist": (.1, .5, 1),
            "right_shoulder": (.6, .5, 1), "right_elbow": (.75, .5, 1), "right_wrist": (.9, .5, 1),
        }
        for _ in range(3):
            self.now += .1
            card.ingest(open_arms)
        status = card.dispatch("status", {"session_id": begin["session_id"]})
        self.assertEqual(status["state"], "completed")
        self.assertEqual(status["progress"]["repetitions"], 1)

    def test_upper_card_counts_multiple_repetitions_after_release(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        begin = card.dispatch("begin", {"pose": "hands_up", "repetitions": 2})
        self.assertEqual([event["event"] for event in begin["events"]], ["task_started"])
        for _ in range(3):
            self.now += .1
            card.ingest(arms())
        first = card.dispatch("status", {"session_id": begin["session_id"]})
        self.assertEqual(first["progress"]["repetitions"], 1)
        self.assertEqual(first["state"], "running")
        self.assertEqual([event["event"] for event in first["events"]],
                         ["rep_completed", "progress_milestone"])
        self.assertEqual(first["events"][0]["narration"], "第 1 次双手举高完成，用时 0.0 秒")
        for _ in range(4):
            self.now += .1
            card.ingest(arms(False))
        for _ in range(3):
            self.now += .1
            card.ingest(arms())
        second = card.dispatch("status", {"session_id": begin["session_id"]})
        self.assertEqual(second["state"], "completed")
        self.assertEqual(second["progress"]["repetitions"], 2)

    def test_timeout_is_reported_as_one_shot_event(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        begin = card.dispatch("begin", {"pose": "hands_up", "hold_seconds": .5, "timeout_seconds": 2})
        self.now = 3
        result = card.dispatch("status", {"session_id": begin["session_id"]})
        self.assertEqual(result["state"], "timed_out")
        self.assertEqual(result["events"][0]["event"], "session_timed_out")
        self.assertIn("超时", result["narration"])
        self.assertNotIn("events", card.dispatch("status", {"session_id": begin["session_id"]}))

    def test_cancel_running_session_does_not_need_session_id(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        begin = card.dispatch("begin", {"pose": "hands_up", "hold_seconds": 3})
        result = card.dispatch("cancel", {})
        self.assertEqual(result["state"], "cancelled")
        self.assertEqual(result["session_id"], begin["session_id"])
        self.assertEqual(result["events"][0]["event"], "session_cancelled")

    def test_cancel_clears_timed_out_session(self):
        card = PoseCard(clock=self.clock)
        card.ingest(arms())
        begin = card.dispatch("begin", {"pose": "hands_up", "hold_seconds": 1,
                                          "timeout_seconds": 2})
        self.now = 3
        timed_out = card.dispatch("status", {"session_id": begin["session_id"]})
        self.assertEqual(timed_out["state"], "timed_out")
        cancelled = card.dispatch("cancel", {})
        self.assertEqual(cancelled["state"], "cancelled")
        self.assertEqual(card.dispatch("info", {})["target_pose"], None)
        card.ingest(arms())
        self.assertNotIn("error", card.dispatch("begin", {"pose": "hands_up"}))


if __name__ == "__main__":
    unittest.main()
