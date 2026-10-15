"""Hardware-free checks for the RealMan RM75 ArUco marker center detection card."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

REALMAN_DIR = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v"
SOURCE = REALMAN_DIR / "aruco_detector.py"
if str(REALMAN_DIR) not in sys.path:
    sys.path.insert(0, str(REALMAN_DIR))


def _load_aruco_module():
    spec = importlib.util.spec_from_file_location("rm75_aruco_detector", SOURCE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ArucoDetectorCardTests(unittest.TestCase):
    def test_driver_registers_card_and_packages_aruco_runtime(self):
        driver_yaml = (REALMAN_DIR / "driver.yaml").read_text()
        self.assertIn("name: aruco_detector,   type: processor", driver_yaml)

        config_yaml = (REALMAN_DIR / "config.yaml").read_text()
        self.assertIn("aruco_detector:\n  enabled: true", config_yaml)

        dockerfile = (REALMAN_DIR / "Dockerfile").read_text()
        self.assertIn("opencv-contrib-python-headless==4.11.0.86", dockerfile)
        self.assertIn("aruco_detector.py", dockerfile)
        self.assertIn("aruco_zones.py", dockerfile)
        self.assertIn("aruco_canvas.py", dockerfile)

    def test_extract_markers_from_corners_computes_geometric_centers(self):
        mod = _load_aruco_module()

        # 模拟两个标记码：ID 5 与 ID 2
        # ID 5: 矩形角点 (100, 100), (200, 100), (200, 200), (100, 200)，中心应为 (150, 150)
        # ID 2: 矩形角点 (300, 200), (400, 200), (400, 300), (300, 300)，中心应为 (350, 250)
        corners = [
            [[[100.0, 100.0], [200.0, 100.0], [200.0, 200.0], [100.0, 200.0]]],
            [[[300.0, 200.0], [400.0, 200.0], [400.0, 300.0], [300.0, 300.0]]],
        ]
        ids = [[5], [2]]

        markers = mod.extract_markers_from_corners(corners, ids, image_width=640, image_height=480)

        # 结果应按 ID 升序排序：ID 2 在前，ID 5 在后
        self.assertEqual(len(markers), 2)
        self.assertEqual(markers[0]["id"], 2)
        self.assertEqual(markers[0]["center_px"], [350.0, 250.0])
        self.assertEqual(markers[0]["position"], [0.0938, 0.0417])
        self.assertEqual(markers[0]["center"], markers[0]["position"])
        self.assertEqual(markers[0]["center_normalized"], [round(350.0 / 640, 4), round(250.0 / 480, 4)])
        self.assertEqual(len(markers[0]["corners_px"]), 4)

        self.assertEqual(markers[1]["id"], 5)
        self.assertEqual(markers[1]["center_px"], [150.0, 150.0])
        self.assertEqual(markers[1]["position"], [-0.5312, -0.375])
        self.assertEqual(markers[1]["center"], markers[1]["position"])
        self.assertEqual(markers[1]["center_normalized"], [round(150.0 / 640, 4), round(150.0 / 480, 4)])

    def test_extract_markers_from_empty_returns_empty_list(self):
        mod = _load_aruco_module()
        self.assertEqual(mod.extract_markers_from_corners([], None, 640, 480), [])
        self.assertEqual(mod.extract_markers_from_corners([], [], 640, 480), [])

    def test_detector_manifest_and_tool_contract(self):
        def module(name, **attributes):
            value = types.ModuleType(name)
            value.__dict__.update(attributes)
            return value

        fake_modules = {
            "rclpy": module("rclpy"),
            "rclpy.node": module("rclpy.node", Node=object),
            "rclpy.qos": module("rclpy.qos", qos_profile_sensor_data=object()),
            "sensor_msgs": module("sensor_msgs"),
            "sensor_msgs.msg": module("sensor_msgs.msg", CompressedImage=object),
            "std_msgs": module("std_msgs"),
            "std_msgs.msg": module("std_msgs.msg", String=object),
        }
        with mock.patch.dict(sys.modules, fake_modules):
            mod = _load_aruco_module()
            executor = type("Executor", (), {"context": object()})()
            plugin = mod.ArucoDetectorPlugin({}, "rm75", executor)

            # 校验单工具清单与多工具列表
            manifest = plugin.get_tool()
            self.assertEqual(manifest["name"], "aruco_detector")
            self.assertEqual(manifest["type"], "processor")
            self.assertEqual(manifest["topic_in"][0]["format"], "image/jpeg")
            self.assertEqual([out["format"] for out in manifest["topic_out"]],
                             ["image/jpeg", "data/json"])

            actions = manifest["inputSchema"]["properties"]["action"]["enum"]
            self.assertIn("detect", actions)
            self.assertIn("get_marker_center", actions)
            self.assertIn("info", actions)
            self.assertIn("start", actions)
            self.assertIn("stop", actions)

            # 校验无分拣区配置
            self.assertNotIn("waiting_rect", manifest["inputSchema"]["properties"])
            self.assertNotIn("sorting_1_rect", manifest["inputSchema"]["properties"])

            # 校验兼容别名工具定义
            all_tools = plugin.get_tools()
            tool_names = [t["name"] for t in all_tools]
            self.assertIn("aruco_detector", tool_names)
            self.assertIn("aruco_zones", tool_names)

    def test_callable_detect_action_with_and_without_marker_id(self):
        mod = _load_aruco_module()
        executor = type("Executor", (), {"context": object()})()
        plugin = mod.ArucoDetectorPlugin({}, "rm75", executor)

        # 模拟最新检测到的帧数据（包含 ID 3 和 ID 7）
        simulated_result = {
            "detected_markers": [
                {
                    "id": 3,
                    "position": [-0.3, -0.1],
                    "compensated_position": [-0.25, -0.2],
                    "center": [-0.3, -0.1],
                    "center_normalized": [0.35, 0.45],
                    "center_px": [224.0, 216.0],
                    "compensated_center_px": [240.0, 192.0],
                    "corners_px": [[210, 200], [238, 200], [238, 232], [210, 232]],
                    "corners_normalized": [[0.328, 0.417], [0.372, 0.417], [0.372, 0.483], [0.328, 0.483]],
                },
                {
                    "id": 7,
                    "position": [0.5, 0.6],
                    "compensated_position": [0.55, 0.5],
                    "center": [0.5, 0.6],
                    "center_normalized": [0.75, 0.80],
                    "center_px": [480.0, 384.0],
                    "compensated_center_px": [496.0, 360.0],
                    "corners_px": [[460, 360], [500, 360], [500, 408], [460, 408]],
                    "corners_norm": [[0.719, 0.75], [0.781, 0.75], [0.781, 0.85], [0.719, 0.85]],
                },
            ],
            "detected_marker_ids": [3, 7],
            "marker_count": 2,
            "image_width": 640,
            "image_height": 480,
        }
        plugin._latest_result = simulated_result
        plugin._last_frame_at = 1000.0

        with mock.patch("time.monotonic", return_value=1000.5):
            # 1. 调用 detect（不传 marker_id）：返回所有检测到的码及中心
            res_all = plugin.dispatch("detect", {})
            self.assertTrue(res_all["success"])
            self.assertEqual(res_all["detected_marker_ids"], [3, 7])
            self.assertEqual(len(res_all["detected_markers"]), 2)
            self.assertEqual(res_all["detected_markers"][0]["id"], 3)
            self.assertEqual(res_all["detected_markers"][0]["center_normalized"], [0.35, 0.45])

            # 2. 调用 detect（传入存在的 marker_id=3）：直接返回目标码中心点
            res_target = plugin.dispatch("detect", {"marker_id": 3})
            self.assertTrue(res_target["success"])
            self.assertTrue(res_target["found"])
            self.assertEqual(res_target["marker_id"], 3)
            self.assertEqual(res_target["position"], [-0.3, -0.1])
            self.assertEqual(res_target["center"], [-0.3, -0.1])
            self.assertEqual(res_target["center_normalized"], [0.35, 0.45])
            self.assertEqual(res_target["center_px"], [224.0, 216.0])

            # 3. 调用 get_marker_center 别名动作
            res_alias = plugin.dispatch("get_marker_center", {"marker_id": 7})
            self.assertTrue(res_alias["success"])
            self.assertTrue(res_alias["found"])
            self.assertEqual(res_alias["marker_id"], 7)
            self.assertEqual(res_alias["center_px"], [480.0, 384.0])

            # 4. 调用 detect（传入不存在的 marker_id=99）：found 为 False，center 为 None
            res_missing = plugin.dispatch("detect", {"marker_id": 99})
            self.assertTrue(res_missing["success"])
            self.assertFalse(res_missing["found"])
            self.assertIsNone(res_missing["center"])
            self.assertIsNone(res_missing["center_px"])

            # 5. 调用 info：查看状态
            info = plugin.dispatch("info", {})
            self.assertEqual(info["card_name"], "aruco_detector")
            self.assertEqual(info["detected_marker_ids"], [3, 7])

            # 6. 调用 config：更新配置（界面保存/启动智能控制时触发）
            cfg_res = plugin.dispatch("config", {"input_topic": "/ext_camera/default/rgb", "dictionary": "DICT_5X5_50"})
            self.assertTrue(cfg_res.get("ok"))
            self.assertEqual(cfg_res["input_topic"], "/ext_camera/default/rgb")
            self.assertEqual(cfg_res["dictionary"], "DICT_5X5_50")

            # 7. 调用 stop
            stopped = plugin.dispatch("stop", {})
            self.assertEqual(stopped["state"], "idle")

    def test_compatibility_wrappers_reexport_detector(self):
        # 校验 aruco_zones.py 与 aruco_canvas.py 均正确重导出
        import aruco_zones
        import aruco_canvas

        self.assertIs(aruco_zones.ArucoZonesPlugin, aruco_zones.ArucoDetectorPlugin)
        self.assertIs(aruco_canvas.ArucoZonesPlugin, aruco_canvas.ArucoDetectorPlugin)
        self.assertTrue(callable(aruco_zones.extract_markers_from_corners))
        self.assertTrue(callable(aruco_canvas.extract_markers_from_corners))


if __name__ == "__main__":
    unittest.main()
