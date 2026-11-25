"""ROS-free tests for the R1 pose_check geometry rules."""

import sys
import unittest
import time
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pose_check import check_pose
from pose_plugin import PoseCheckPlugin


class PoseCheckTests(unittest.TestCase):
    def setUp(self):
        self.shoulders = {
            "left_shoulder": (0.4, 0.5, 1.0),
            "right_shoulder": (0.6, 0.5, 1.0),
            "left_elbow": (.4, .35, 1),
            "right_elbow": (.6, .35, 1),
        }

    def test_hands_up_success(self):
        points = {**self.shoulders,
                  "left_wrist": (0.4, 0.2, 1.0),
                  "right_wrist": (0.6, 0.2, 1.0)}
        result = check_pose(points, "hands_up")
        self.assertTrue(result["detected"])
        self.assertEqual(result["feedback"], "已完成")

    def test_hands_up_left_too_low(self):
        points = {**self.shoulders,
                  "left_wrist": (0.4, 0.6, 1.0),
                  "right_wrist": (0.6, 0.2, 1.0)}
        result = check_pose(points, "hands_up")
        self.assertEqual(result["feedback"], "左手再抬高，让手腕高于肘部")

    def test_arms_open_success(self):
        points = {**self.shoulders,
                  "left_elbow": (.27, .5), "right_elbow": (.73, .5),
                  "left_wrist": (0.15, 0.5, 1.0),
                  "right_wrist": (0.85, 0.5, 1.0)}
        result = check_pose(points, "arms_open")
        self.assertTrue(result["detected"])
        self.assertEqual(result["feedback"], "已完成")

    def test_one_hand_up_success(self):
        points = {**self.shoulders,
                  "left_wrist": (0.4, 0.2, 1.0),
                  "right_elbow": (.6, .55),
                  "right_wrist": (0.6, 0.6, 1.0)}
        result = check_pose(points, "one_hand_up")
        self.assertTrue(result["detected"])
        self.assertEqual(result["raised_side"], "left")

    def test_no_person(self):
        result = check_pose({}, "hands_up")
        self.assertFalse(result["detected"])
        self.assertEqual(result["reason"], "no_person")

    def test_only_forearms_raised_do_not_match(self):
        points = {**self.shoulders, "left_elbow": (.4, .7),
                  "right_elbow": (.6, .7), "left_wrist": (.4, .4),
                  "right_wrist": (.6, .4)}
        for pose in ("hands_up", "one_hand_up"):
            with self.subTest(pose=pose):
                result = check_pose(points, pose)
                self.assertFalse(result["matched"])
                self.assertIn("大臂", result["feedback"])

    def test_wrists_on_shoulder_line_but_elbows_down_do_not_match(self):
        points = {**self.shoulders, "left_elbow": (.27, .7),
                  "right_elbow": (.73, .7), "left_wrist": (.15, .5),
                  "right_wrist": (.85, .5)}
        self.assertFalse(check_pose(points, "arms_open")["matched"])

    def test_folded_forearms_do_not_match_open(self):
        points = {**self.shoulders, "left_elbow": (.2, .5),
                  "right_elbow": (.8, .5), "left_wrist": (.3, .5),
                  "right_wrist": (.7, .5)}
        result = check_pose(points, "arms_open")
        self.assertFalse(result["matched"])
        self.assertIn("伸直", result["feedback"])

    def test_missing_or_hidden_elbow_rejected(self):
        points = {**self.shoulders, "left_wrist": (.4, .2), "right_wrist": (.6, .2)}
        points.pop("left_elbow")
        self.assertIn("left_elbow", check_pose(points, "hands_up")["missing"])
        points["left_elbow"] = (.4, .35, .2)
        result = check_pose(points, "hands_up")
        self.assertFalse(result["matched"])
        self.assertIn("left_elbow", result["low_visibility"])

    def test_lowering_arms_exits_match(self):
        up = {**self.shoulders, "left_wrist": (.4, .2), "right_wrist": (.6, .2)}
        down = {**self.shoulders, "left_elbow": (.4, .65),
                "right_elbow": (.6, .65), "left_wrist": (.4, .8),
                "right_wrist": (.6, .8)}
        self.assertTrue(check_pose(up, "hands_up")["matched"])
        self.assertFalse(check_pose(down, "hands_up")["matched"])

    def test_other_arm_must_be_lowered_for_one_hand(self):
        points = {**self.shoulders, "left_wrist": (.4, .2),
                  "right_elbow": (.75, .5), "right_wrist": (.9, .5)}
        self.assertFalse(check_pose(points, "one_hand_up")["matched"])

    def test_unsupported_pose(self):
        result = check_pose({}, "jumping_jack")
        self.assertEqual(result["error"], "unsupported_pose")

    def test_anatomical_left_on_image_right(self):
        points = {"left_shoulder": (.6, .5), "right_shoulder": (.4, .5),
                  "left_elbow": (.73, .5), "right_elbow": (.27, .5),
                  "left_wrist": (.85, .5), "right_wrist": (.15, .5)}
        self.assertTrue(check_pose(points, "arms_open")["matched"])

    def test_two_hands_do_not_match_one_hand(self):
        points = {**self.shoulders, "left_wrist": (.4, .2), "right_wrist": (.6, .2)}
        result = check_pose(points, "one_hand_up")
        self.assertTrue(result["detected"])
        self.assertFalse(result["matched"])
        self.assertEqual(result["score"], 0)

    def test_nonfinite_point_does_not_match(self):
        points = {**self.shoulders, "left_wrist": (.4, float("nan")),
                  "right_wrist": (.6, .2)}
        self.assertFalse(check_pose(points, "hands_up")["matched"])

    def test_plugin_jpeg_path_and_stale_frame(self):
        plugin = PoseCheckPlugin({}, "test", None)
        tool = plugin.get_tool()
        self.assertEqual(tool["topic_in"][0]["topic"], "/test/camera/main")
        self.assertEqual(tool["inputSchema"]["required"], ["action"])
        self.assertEqual(tool["inputSchema"]["x-action-params"], {
            "start": {"params": [], "description": "启用姿态检查"},
            "stop": {"params": [], "description": "停止姿态检查"},
            "info": {"params": [], "description": "查询相机输入和模型状态"},
            "check": {"params": ["pose"], "description": "检查当前相机画面的姿态"},
        })
        info = plugin.dispatch("info", {})
        self.assertEqual(info["topic_in"], [{"topic": "/test/camera/main",
                                              "format": "image/jpeg"}])
        self.assertEqual(plugin.dispatch("check", {"pose": "hands_up"})["error"],
                         "camera_no_data")
        self.assertEqual(plugin.dispatch("check", {
            "pose": "hands_up",
            "keypoints": {**self.shoulders, "left_wrist": (.4, .2),
                          "right_wrist": (.6, .2)},
        })["error"], "camera_no_data")
        plugin._on_frame(SimpleNamespace(data=b"jpeg"))
        received = []
        points = {**self.shoulders, "left_wrist": (.4, .2), "right_wrist": (.6, .2)}
        plugin._estimator = SimpleNamespace(estimate=lambda jpeg: received.append(jpeg) or points)
        self.assertTrue(plugin.dispatch("check", {"pose": "hands_up"})["matched"])
        self.assertEqual(received, [b"jpeg"])
        plugin._latest_frame_at = time.monotonic() - 3
        self.assertEqual(plugin.dispatch("check", {"pose": "hands_up"})["error"],
                         "camera_stale")

    def test_no_person_and_model_failure_are_distinct(self):
        plugin = PoseCheckPlugin({"input_topic": "/bumi/camera/color"}, "test", None)
        plugin._on_frame(SimpleNamespace(data=b"jpeg"))
        self.assertEqual(plugin.dispatch("check", {"pose": "hands_up"})["error"],
                         "model_unavailable")
        plugin._estimator = SimpleNamespace(estimate=lambda jpeg: {})
        self.assertEqual(plugin.dispatch("check", {"pose": "hands_up"})["reason"], "no_person")


if __name__ == "__main__":
    unittest.main()
