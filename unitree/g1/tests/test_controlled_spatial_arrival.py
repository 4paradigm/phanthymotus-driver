"""
Tests for the G1 controlled_spatial fallback arrival decision.

The isolated-process path builds the plugin without SmartMotion, so arrival has
to come from the pose on rt/slam_info. The bug this pins: the fallback judged the
stall timer before arrival, so a robot standing exactly on the tag (measured
0.01 m) was reported to the agent as ``stall_timeout`` — the agent then never
learned it had arrived and could not narrate or continue the tour.

Run:  python3 -m unittest discover -s unitree/g1/tests -t .
"""

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_module():
    spec = importlib.util.spec_from_file_location(
        "g1_controlled_spatial", ROOT / "controlled_spatial.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cs = load_module()

TARGET = {"x": -6.304803199746908, "y": -3.611632396991577}


class PoseDistanceTest(unittest.TestCase):
    def test_distance_is_planar(self):
        self.assertAlmostEqual(cs._pose_distance({"x": 0, "y": 0}, {"x": 3, "y": 4}), 5.0)

    def test_missing_inputs_return_none(self):
        self.assertIsNone(cs._pose_distance(None, TARGET))
        self.assertIsNone(cs._pose_distance({"x": 1, "y": 2}, None))
        self.assertIsNone(cs._pose_distance({"x": "?"}, TARGET))


class NavOutcomeTest(unittest.TestCase):
    def test_on_target_is_arrival_even_when_not_moving(self):
        """The regression: stationary on the tag must not read as a stall."""
        pose = {"x": TARGET["x"] + 0.01, "y": TARGET["y"]}
        state, dist, _, _ = cs._nav_outcome(
            pose, TARGET, pose, last_move_time=0.0, started_at=0.0, now=200.0)
        self.assertEqual(state, "arrived")
        self.assertLess(dist, cs.NAV_ARRIVE_RADIUS_M)

    def test_arrival_wins_over_stall_timer(self):
        """Stall timer long expired, but the robot is on the target."""
        pose = {"x": TARGET["x"], "y": TARGET["y"]}
        state, _, _, _ = cs._nav_outcome(
            pose, TARGET, pose, last_move_time=0.0, started_at=0.0,
            now=cs.NAV_HARD_TIMEOUT_S + 1)
        self.assertEqual(state, "arrived")

    def test_just_outside_radius_still_arrives_once_stalled(self):
        """Stopping 0.5 m short is treated as arrival rather than a failed leg."""
        pose = {"x": TARGET["x"] + 0.5, "y": TARGET["y"]}
        state, dist, _, _ = cs._nav_outcome(
            pose, TARGET, pose, last_move_time=0.0, started_at=0.0, now=95.0)
        self.assertEqual(state, "arrived")
        self.assertGreater(dist, cs.NAV_ARRIVE_RADIUS_M)

    def test_slow_cumulative_movement_resets_stall(self):
        """Creeping toward the tag must not be judged stalled."""
        ref = {"x": TARGET["x"] + 1.0, "y": TARGET["y"]}
        pose = {"x": TARGET["x"] + 0.9, "y": TARGET["y"]}
        state, _, new_ref, new_last = cs._nav_outcome(
            pose, TARGET, ref, last_move_time=0.0, started_at=0.0, now=120.0)
        self.assertEqual(state, "continue")
        self.assertEqual(new_ref, pose)
        self.assertEqual(new_last, 120.0)

    def test_stationary_far_away_stalls(self):
        pose = {"x": 10.0, "y": 10.0}
        state, _, _, _ = cs._nav_outcome(
            pose, TARGET, pose, last_move_time=0.0, started_at=0.0, now=95.0)
        self.assertEqual(state, "stall")

    def test_hard_timeout_still_reported(self):
        pose = {"x": TARGET["x"] + 5.0, "y": TARGET["y"]}
        ref = {"x": TARGET["x"] + 4.0, "y": TARGET["y"]}
        state, _, _, _ = cs._nav_outcome(
            pose, TARGET, ref, last_move_time=180.0, started_at=0.0,
            now=cs.NAV_HARD_TIMEOUT_S + 1)
        self.assertEqual(state, "timeout")

    def test_without_target_pose_never_reports_arrival(self):
        """Older callers passed no target: behaviour must stay stall/timeout only."""
        pose = {"x": TARGET["x"], "y": TARGET["y"]}
        state, dist, _, _ = cs._nav_outcome(
            pose, None, pose, last_move_time=0.0, started_at=0.0, now=95.0)
        self.assertEqual(state, "stall")
        self.assertIsNone(dist)


if __name__ == "__main__":
    unittest.main()
