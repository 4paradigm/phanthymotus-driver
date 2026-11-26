"""Exercise the public read-only grasp tool with controlled feedback timestamps."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from agilex.piper import grasp_check
from common.vendor_runtime import DriverBundle


class GraspCheckTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.width = 20.0
        self.effort = -0.25
        self.enabled = True
        self.error_code = 0
        self.busy = False
        self.task_id = "finished"
        self.missing = False
        self.freeze_stamp = False
        self.sample = self.snapshot
        runtime = SimpleNamespace(read_monitor_snapshot=lambda: self.sample())
        self.bundle = DriverBundle([grasp_check.PiperGraspCheck(runtime)])
        self.settings = {
            "min_width_mm": 19.0,
            "max_width_mm": 21.0,
            "empty_width_mm": 0.2,
            "empty_tolerance_mm": 0.3,
            "holding_effort_threshold_nm": -0.15,
        }
        for name, callback in (("monotonic", lambda: self.now), ("time", lambda: self.now),
                               ("sleep", self.advance)):
            patcher = patch.object(grasp_check, name, callback)
            patcher.start()
            self.addCleanup(patcher.stop)

    def advance(self, seconds):
        self.now += seconds

    def snapshot(self):
        return {
            "gripper": None if self.missing else {
                "width_mm": self.width, "effort_nm": self.effort, "enabled": self.enabled,
                "error_code": self.error_code, "feedback_time": 100.0 if self.freeze_stamp else self.now,
            },
            "busy": self.busy,
            "task": {"task_id": self.task_id},
            "reasons": {"gripper": "Stale gripper feedback."} if self.missing else {},
        }

    def call(self, action, **args):
        return self.bundle.dispatch("piper_grasp_check", {"action": action, **args})

    def check(self):
        return self.call("check_grasp", **self.settings)

    def test_read_and_lifecycle_return_passive_feedback(self):
        self.assertEqual(self.call("start"), {"state": "ready"})
        result = self.call("read_feedback")
        self.assertEqual(result["state"], "ok")
        self.assertEqual((result["gripper"]["width_mm"], result["gripper"]["effort_nm"]), (20, -0.25))
        self.assertEqual(result["feedback_age_ms"], 0)
        self.assertEqual(self.call("stop"), {"state": "idle"})
        self.missing = True
        self.assertEqual(self.call("info")["reason"], "Stale gripper feedback.")
        with self.assertRaises(ValueError):
            self.call("close")

    def test_calibrated_hold_and_empty_result(self):
        result = self.check()
        self.assertEqual((result["state"], result["is_grasped"]), ("grasped", True))
        self.assertGreaterEqual(result["sample_count"], 3)
        self.assertGreaterEqual(result["sample_span_ms"], 120)

        self.width = 0.2
        self.effort = 0.0
        result = self.check()
        self.assertEqual((result["state"], result["is_grasped"]), ("empty", False))

    def test_observed_opening_can_exceed_the_motion_limit(self):
        self.width = 83.2
        self.settings.update(min_width_mm=82.0, max_width_mm=84.0)
        self.assertEqual(self.check()["state"], "grasped")

    def test_partial_or_conflicting_evidence_stays_indeterminate(self):
        cases = (
            ({"effort": 0.25}, "holding_effort_not_confirmed"),
            ({"effort": -0.1}, "holding_effort_not_confirmed"),
            ({"width": 23}, "outside_expected_opening"),
            ({"freeze_stamp": True}, "insufficient_distinct_feedback"),
            ({"enabled": False}, "gripper_disabled"),
            ({"error_code": 4}, "gripper_fault"),
            ({"busy": True}, "arm_operation_in_progress"),
            ({"missing": True}, "feedback_unavailable"),
        )
        for changes, reason in cases:
            with self.subTest(reason=reason, changes=changes), patch.multiple(self, **changes):
                result = self.check()
                self.assertEqual(result["state"], "indeterminate")
                self.assertIsNone(result["is_grasped"])
                self.assertEqual(result["reason"], reason)

    def test_motion_and_new_task_during_window_do_not_count_as_grasp(self):
        def moving():
            self.width += 0.2
            return self.snapshot()

        self.sample = moving
        self.assertEqual(self.check()["reason"], "opening_unstable")

        start = self.now
        self.width = 20.0

        def interrupted():
            if self.now - start > 0.04:
                self.task_id = "new"
            return self.snapshot()

        self.sample = interrupted
        self.assertEqual(self.check()["reason"], "arm_operation_in_progress")

    def test_invalid_or_overlapping_calibration(self):
        with self.assertRaises(ValueError):
            self.call("check_grasp", **{**self.settings, "holding_effort_threshold_nm": 0})
        with self.assertRaises(ValueError):
            self.call("check_grasp", **{**self.settings, "max_width_mm": 18})
        result = self.call("check_grasp", **{**self.settings, "min_width_mm": 0.4})
        self.assertEqual(result["reason"], "expected_opening_overlaps_empty_band")
        self.assertIsNone(result["is_grasped"])
