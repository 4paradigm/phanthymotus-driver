"""测量数学、时间戳配对和异常输入的离线测试；不依赖真机 ROS 2。"""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import visual_ee_repeatability as module
from visual_ee_repeatability import (
    LEFT_ARM_JOINTS, Plugin, _angle_deg, aggregate_frames, relative_pose,
    repeatability_report,
)


class _Client:
    """模拟没有关节反馈的开发机。"""

    def snapshot(self):
        return {"fresh": False, "age_ms": None}


class _LiveClient:
    """模拟左臂静止且反馈完整的真机状态。"""

    def snapshot(self):
        return {"fresh": True, "age_ms": 0, "message_timestamp_ms": None,
                "joints": {j: 0.0 for j in LEFT_ARM_JOINTS},
                "velocities": {j: 0.0 for j in LEFT_ARM_JOINTS}}


def _tf(x):
    """生成相机坐标系下位于 X 轴的测试 TF。"""
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


class VisualRepeatabilityTests(unittest.TestCase):
    def test_canvas_start_stop_are_read_only_lifecycle_actions(self):
        """画布启动和停止需返回传感器状态，且不改变正在运行的测量节点。"""
        plugin = Plugin({"tag_size_confirmed": False}, "test", None, _Client())
        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertIn("start", actions)
        self.assertIn("stop", actions)
        self.assertEqual(plugin.dispatch("start", {}), {"state": "running"})
        self.assertEqual(plugin.dispatch("stop", {}), {"state": "idle"})

    def test_relative_pose_cancels_camera_translation_and_rotation(self):
        """相机整体平移与旋转不应改变两 Tag 的相对位姿。"""
        # 相机绕 Z 轴旋转 90 度后，参考系下的手部位移仍应沿 X 轴。
        quarter_turn = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        reference = ((2.0, 3.0, 0.5), quarter_turn)
        hand = ((2.0, 3.2, 0.5), quarter_turn)
        position, rotation = relative_pose(reference, hand)
        self.assertAlmostEqual(position[0], 0.2)
        self.assertAlmostEqual(position[1], 0.0)
        self.assertAlmostEqual(_angle_deg(rotation, (0, 0, 0, 1)), 0.0)

    def test_aggregate_rejects_one_far_visual_outlier(self):
        """单帧明显偏离时应剔除，不能污染一次到位样本。"""
        frames = [((0.1 + i * 0.0001, 0.2, 0.3), (0, 0, 0, 1)) for i in range(20)]
        frames.append(((0.2, 0.2, 0.3), (0, 0, 0, 1)))
        pose, accepted, rejected = aggregate_frames(frames)
        self.assertEqual((accepted, rejected), (20, 1))
        self.assertLess(abs(pose[0][0] - 0.101), 0.001)

    def test_report_uses_arrival_samples_and_quaternion_sign_invariance(self):
        """两次到位相差 2 mm 时 RMS 为 1 mm；q 与 -q 视为同一旋转。"""
        samples = [((0, 0, 0), (0, 0, 0, 1)),
                   ((0.002, 0, 0), (0, 0, 0, -1))]
        report = repeatability_report(samples)
        self.assertAlmostEqual(report["translation"]["rms_mm"], 1.0)
        self.assertAlmostEqual(report["rotation"]["max_deg"], 0.0)

    def test_session_refuses_unmeasured_tag_size(self):
        """尚未实测 Tag 黑框边长时，不允许开始正式测量。"""
        plugin = Plugin({"tag_size_confirmed": False}, "test", None, _Client())
        result = plugin.dispatch("start_session", {"side": "left"})
        self.assertEqual(result["code"], "TAG_SIZE_UNCONFIRMED")

    def test_detection_and_tf_are_joined_at_exact_image_stamp(self):
        """两个 Tag 的 TF 查询必须使用检测图像的原始时间戳。"""
        plugin = Plugin({"tag_size_confirmed": True}, "test", None, _LiveClient())
        stamp = SimpleNamespace(sec=123, nanosec=456)
        msg = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id="camera_color_optical_frame"),
            detections=[SimpleNamespace(id=i, hamming=0, decision_margin=80.0)
                        for i in (0, 1)])
        calls = []

        class Buffer:
            def lookup_transform(self, target, source, when):
                calls.append((target, source, when.nanoseconds))
                return _tf(0.2 if source == "q5_reference_tag" else 0.3)

        class Time:
            def __init__(self, nanoseconds):
                self.nanoseconds = nanoseconds

        plugin._tf_buffer = Buffer()
        plugin._on_detections(msg)
        with patch.object(module, "Time", Time, create=True):
            plugin._process_pending()
        self.assertEqual(len(plugin._static_poses), 1)
        self.assertAlmostEqual(plugin._static_poses[0][1][0][0], 0.1)
        self.assertEqual(calls, [
            ("camera_color_optical_frame", "q5_reference_tag", 123000000456),
            ("camera_color_optical_frame", "q5_left_hand_tag", 123000000456),
        ])

    def test_missing_joint_age_does_not_break_departure_observation(self):
        """关节状态声称新鲜但没有年龄时，应安全跳过离开姿态判定。"""
        class IncompleteClient:
            def snapshot(self):
                return {"fresh": True, "age_ms": None, "joints": {}}

        plugin = Plugin({"tag_size_confirmed": True}, "test", None, IncompleteClient())
        plugin._previous_sample_joint_positions = {j: 0.0 for j in LEFT_ARM_JOINTS}
        plugin._observe_departure()
        self.assertFalse(plugin._departure_seen)


if __name__ == "__main__":
    unittest.main()
