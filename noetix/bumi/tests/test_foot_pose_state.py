"""No SDK, ROS, network, or actuator is needed for these geometry checks."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from foot_pose_state import LEG_IDS, describe_feet, main, source_freshness


def snapshot():
    return {"joint_states": [{"motor_id": i, "position": 0.0} for i in sorted(LEG_IDS)]}


class GeometryTests(unittest.TestCase):
    def test_symmetric_zero_angles_reuse_vendor_chain_in_body_frame(self):
        result = describe_feet(snapshot())
        self.assertEqual(result["frame"], "base_link")
        left, right = [result["feet"][side]["sole_center_m"] for side in ("left", "right")]
        self.assertAlmostEqual(left[0], 0.03013641)
        self.assertAlmostEqual(left[1], 0.0995)
        self.assertAlmostEqual(left[2], -0.48022)
        self.assertAlmostEqual(left[0], right[0])
        self.assertAlmostEqual(left[1], -right[1])
        self.assertAlmostEqual(result["separation"]["lateral_width_m"], 0.199)
        self.assertTrue(result["fore_aft_aligned"])
        self.assertFalse(result["accuracy_validated"])
        self.assertFalse(result["floor_contact_verified"])
        self.assertFalse(result["command_sent"])
        self.assertFalse(result["pose_confirmed"])

    def test_one_leg_pitch_changes_fore_aft_gap_without_a_motion_command(self):
        data = snapshot()
        data["joint_states"][0]["position"] = 0.25
        result = describe_feet(data)
        self.assertGreater(result["separation"]["fore_aft_offset_m"], 0.04)
        self.assertLess(result["separation"]["left_minus_right_xyz_m"][0], 0)
        self.assertFalse(result["fore_aft_aligned"])
        self.assertFalse(result["command_sent"])

    def test_joint_ids_determine_order_including_unneeded_arm_joints(self):
        data = snapshot()
        data["joint_states"].reverse()
        data["joint_states"].append({"motor_id": 0, "position": 1.0})
        self.assertEqual(describe_feet(data)["feet"], describe_feet(snapshot())["feet"])

    def test_missing_duplicate_boolean_and_nonfinite_joint_data_are_rejected(self):
        cases = [{}, {"joint_states": []}, {"joint_states": [None]}]
        for value in [None, True, "0", math.nan, math.inf]:
            data = snapshot()
            data["joint_states"][0]["position"] = value
            cases.append(data)
        duplicate = snapshot()
        duplicate["joint_states"].append(dict(duplicate["joint_states"][0]))
        cases.append(duplicate)
        boolean_id = snapshot()
        boolean_id["joint_states"][0]["motor_id"] = True
        cases.append(boolean_id)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(ValueError):
                describe_feet(data)

    def test_receipt_clock_is_explicit_not_inferred_from_successful_poll(self):
        self.assertFalse(source_freshness({"fresh": True}, now=10)["source_recent"])
        for stamp in (None, True, math.nan, math.inf, 11, 9.49):
            self.assertFalse(source_freshness({"source_received_monotonic": stamp,
                                             "source_freshness_verified": True}, now=10)["source_recent"])
        self.assertTrue(source_freshness({"source_received_monotonic": 9.6,
                                         "source_freshness_verified": True}, now=10)["source_recent"])


class OfflineTests(unittest.TestCase):
    def test_demo_is_labelled_synthetic_and_never_confirms_a_physical_pose(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(main(["--demo"]), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result["input_kind"], "synthetic_demo")
        self.assertTrue(result["offline"])
        self.assertFalse(result["pose_confirmed"])
        self.assertFalse(result["command_sent"])

    def test_saved_json_cannot_impersonate_live_receive_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.json"
            data = snapshot()
            data.update(source_received_monotonic=0, source_freshness_verified=True)
            path.write_text(json.dumps(data), encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["--sample", str(path)]), 0)
            result = json.loads(output.getvalue())
            self.assertFalse(result["source_freshness_verified"])
            self.assertFalse(result["command_sent"])

    def test_invalid_snapshot_reports_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.json"
            path.write_text("{}", encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stderr(output):
                self.assertEqual(main(["--sample", str(path)]), 1)
            self.assertFalse(json.loads(output.getvalue())["command_sent"])


if __name__ == "__main__":
    unittest.main()
