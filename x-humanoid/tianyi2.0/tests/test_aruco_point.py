"""Hardware-free geometry and actuator-boundary checks for ArUco pointing."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


SOURCE = Path(__file__).resolve().parents[1] / "aruco_point.py"
SPEC = importlib.util.spec_from_file_location("tianyi_aruco_point", SOURCE)
pointing = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pointing)


class FakeArmPlugin:
    @staticmethod
    def _pose_violations(poses):
        return [{"joint": "shoulder_roll"}] if any(
            abs(pose[1]) > 90 for pose in poses.values()) else []


CALIBRATION = {
    "camera_fx_px": 500, "camera_cx_px": 320,
    "side": "right", "joint": "shoulder_roll",
    "reference_pose": "[0, 0, 0, 0, 0, 0, 0]",
    "reference_bearing_deg": 0,
    "joint_deg_per_bearing_deg": 1,
}
IMAGE = {"image_width": 640, "detected_markers": [
    {"id": 7, "center_normalized": [0.5, 0.5]},
    {"id": 9, "center_normalized": [0.625, 0.5]},
]}


class ArucoPointTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.dict(sys.modules, {"device": SimpleNamespace(ArmPlugin=FakeArmPlugin)})
        patch.start()
        self.addCleanup(patch.stop)

    def test_bearing_and_target_follow_selected_id(self):
        calibration = pointing.parse_calibration(CALIBRATION)
        center = pointing.plan_point(IMAGE, 7, calibration)
        right = pointing.plan_point(IMAGE, 9, calibration)
        self.assertEqual(center["target_joint_deg"], 0)
        self.assertAlmostEqual(right["bearing_deg"], math.degrees(math.atan(80 / 500)), places=2)
        self.assertGreater(right["target_joint_deg"], center["target_joint_deg"])
        with self.assertRaisesRegex(ValueError, "未唯一识别"):
            pointing.plan_point(IMAGE, 4, calibration)

    def test_missing_or_bad_calibration_cannot_generate_joint_target(self):
        with self.assertRaisesRegex(ValueError, "缺少真机标定"):
            pointing.parse_calibration({})
        with self.assertRaisesRegex(ValueError, "参考姿势|限位|标定值"):
            pointing.parse_calibration({**CALIBRATION, "reference_pose": "[0, 100, 0, 0, 0, 0, 0]"})

    def test_preview_is_read_only_and_point_requires_confirmation(self):
        zones = SimpleNamespace(dispatch=lambda action, args: {
            "state": "running", "latest": IMAGE, "latest_frame_age_s": 0.1})
        arm = SimpleNamespace(dispatch=mock.Mock())
        hand = SimpleNamespace(dispatch=mock.Mock())
        card = pointing.ArucoPointPlugin(CALIBRATION, zones, arm, hand)
        preview = card.dispatch("preview", {"marker_id": 9})
        self.assertEqual(preview["state"], "preview")
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            card.dispatch("point", {"marker_id": 9})
        arm.dispatch.assert_not_called()
        hand.dispatch.assert_not_called()

    def test_stale_image_blocks_motion_before_arm_access(self):
        zones = SimpleNamespace(dispatch=lambda action, args: {
            "state": "running", "latest": IMAGE, "latest_frame_age_s": 2.0})
        card = pointing.ArucoPointPlugin(CALIBRATION, zones, None, None)
        with self.assertRaisesRegex(ValueError, "过期"):
            card.dispatch("point", {"marker_id": 9, "confirm_motion": True})

    def test_point_uses_feedback_and_commands_arm_then_hand(self):
        zones = SimpleNamespace(dispatch=lambda action, args: {
            "state": "running", "latest": IMAGE, "latest_frame_age_s": 0.1})
        feedback = SimpleNamespace(
            preflight=lambda publisher, ids: None,
            snapshot=lambda ids: (1, {motor_id: 0.0 for motor_id in ids}),
            wait_for_motion=lambda targets, seq, baseline: {"state": "completed"})
        arm = SimpleNamespace(_feedback=feedback, _pos_publisher=object(),
                              _send_pos=mock.Mock(return_value={"state": "moving"}),
                              _target_positions=lambda poses: {})
        hand = SimpleNamespace(dispatch=mock.Mock(return_value={"state": "moving"}))
        card = pointing.ArucoPointPlugin(CALIBRATION, zones, arm, hand)
        result = card.dispatch("point", {"marker_id": 9, "confirm_motion": True})
        self.assertEqual(result["state"], "commanded")
        self.assertFalse(result["base_motion"])
        arm._send_pos.assert_called_once()
        hand.dispatch.assert_called_once_with("point", {"side": "right"})

    def test_large_joint_step_is_rejected_without_publishing(self):
        far_marker = {"image_width": 640, "detected_markers": [
            {"id": 9, "center_normalized": [0.75, 0.5]}]}
        zones = SimpleNamespace(dispatch=lambda action, args: {
            "state": "running", "latest": far_marker, "latest_frame_age_s": 0.1})
        feedback = SimpleNamespace(
            preflight=lambda publisher, ids: None,
            snapshot=lambda ids: (1, {motor_id: 0.0 for motor_id in ids}))
        arm = SimpleNamespace(_feedback=feedback, _pos_publisher=object(),
                              _send_pos=mock.Mock())
        hand = SimpleNamespace(dispatch=mock.Mock())
        card = pointing.ArucoPointPlugin(CALIBRATION, zones, arm, hand)
        with self.assertRaisesRegex(ValueError, "超过 12°"):
            card.dispatch("point", {"marker_id": 9, "confirm_motion": True})
        arm._send_pos.assert_not_called()
        hand.dispatch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
