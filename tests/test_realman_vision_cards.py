"""Hardware-free unit tests for RealMan RM75 color_detector and object_segmentor cards."""

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

REALMAN_DIR = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v"
if str(REALMAN_DIR) not in sys.path:
    sys.path.insert(0, str(REALMAN_DIR))


def _load_module(name: str, file_name: str):
    path = REALMAN_DIR / file_name
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class RealmanVisionCardsTests(unittest.TestCase):
    def test_configuration_and_dockerfile_registration(self):
        driver_yaml = (REALMAN_DIR / "driver.yaml").read_text()
        self.assertIn("name: color_detector,   type: processor", driver_yaml)
        self.assertIn("name: object_segmentor, type: processor", driver_yaml)

        config_yaml = (REALMAN_DIR / "config.yaml").read_text()
        self.assertIn("color_detector:\n  enabled: true", config_yaml)
        self.assertIn("object_segmentor:\n  enabled: true", config_yaml)

        dockerfile = (REALMAN_DIR / "Dockerfile").read_text()
        self.assertIn("color_detector.py", dockerfile)
        self.assertIn("object_segmentor.py", dockerfile)

    def test_color_detector_manifest_and_config(self):
        mod = _load_module("rm75_color_detector", "color_detector.py")
        plugin = mod.ColorDetectorPlugin({}, "nvidia_desktop", None)
        tool = plugin.get_tool()

        self.assertEqual(tool["name"], "color_detector")
        self.assertEqual(tool["type"], "processor")
        self.assertIn("detect", tool["inputSchema"]["x-action-params"])
        self.assertIn("classify", tool["inputSchema"]["x-action-params"])
        self.assertIn("config", tool["inputSchema"]["x-action-params"])

        cfg_res = plugin.dispatch("config", {"input_topic": "/nvidia_desktop/ext_camera/card_test/rgb"})
        self.assertTrue(cfg_res.get("ok"))
        self.assertEqual(cfg_res.get("status"), "ready")
        self.assertTrue(cfg_res.get("ready"))
        self.assertEqual(cfg_res.get("input_topic"), "/nvidia_desktop/ext_camera/card_test/rgb")

        info_res = plugin.dispatch("info", {})
        self.assertEqual(info_res.get("input_topic"), "/nvidia_desktop/ext_camera/card_test/rgb")

    def test_object_segmentor_manifest_and_config(self):
        mod = _load_module("rm75_object_segmentor", "object_segmentor.py")
        plugin = mod.ObjectSegmentorPlugin({}, "nvidia_desktop", None)
        tool = plugin.get_tool()

        self.assertEqual(tool["name"], "object_segmentor")
        self.assertEqual(tool["type"], "processor")
        self.assertIn("segment", tool["inputSchema"]["x-action-params"])
        self.assertIn("measure_size", tool["inputSchema"]["x-action-params"])
        self.assertIn("config", tool["inputSchema"]["x-action-params"])

        cfg_res = plugin.dispatch("config", {"input_topic": "/nvidia_desktop/ext_camera/card_test/rgb"})
        self.assertTrue(cfg_res.get("ok"))
        self.assertEqual(cfg_res.get("status"), "ready")
        self.assertTrue(cfg_res.get("ready"))
        self.assertEqual(cfg_res.get("input_topic"), "/nvidia_desktop/ext_camera/card_test/rgb")

        info_res = plugin.dispatch("info", {})
        self.assertEqual(info_res.get("input_topic"), "/nvidia_desktop/ext_camera/card_test/rgb")

    def test_color_detector_synthetic_detection(self):
        try:
            import numpy as np
            import cv2
        except ImportError:
            self.skipTest("numpy or cv2 not available in local test environment")

        mod = _load_module("rm75_color_detector", "color_detector.py")

        # 640x480 白色背景
        img = np.full((480, 640, 3), 255, dtype=np.uint8)
        # 绘制红色矩形 (100, 100, 80, 80) -> BGR: (0, 0, 255)
        img[100:180, 100:180] = [0, 0, 255]
        # 绘制黄色矩形 (300, 200, 100, 60) -> BGR: (0, 255, 255)
        img[200:260, 300:400] = [0, 255, 255]
        # 绘制黑色矩形 (450, 100, 60, 60) -> BGR: (0, 0, 0)
        img[100:160, 450:510] = [0, 0, 0]

        detected = mod.detect_color_objects(img, target_color="all", min_area=200)
        detected_colors = {d["color"] for d in detected}
        self.assertIn("red", detected_colors)
        self.assertIn("yellow", detected_colors)
        self.assertIn("black", detected_colors)

        # 仅筛选黄色
        yellow_only = mod.detect_color_objects(img, target_color="yellow", min_area=200)
        self.assertEqual(len(yellow_only), 1)
        self.assertEqual(yellow_only[0]["color"], "yellow")
        self.assertAlmostEqual(yellow_only[0]["center_px"][0], 350.0, delta=2.0)
        self.assertAlmostEqual(yellow_only[0]["center_px"][1], 230.0, delta=2.0)

    def test_object_segmentor_synthetic_segmentation(self):
        try:
            import numpy as np
            import cv2
        except ImportError:
            self.skipTest("numpy or cv2 not available in local test environment")

        mod = _load_module("rm75_object_segmentor", "object_segmentor.py")

        # 640x480 亮灰色背景 (200, 200, 200)
        img = np.full((480, 640, 3), 200, dtype=np.uint8)
        # 绘制黑色方块 (100, 100, 120, 120)
        img[100:220, 100:220] = [20, 20, 20]
        # 绘制红色小块 (300, 300, 40, 40)
        img[300:340, 300:340] = [0, 0, 240]

        objects = mod.segment_objects(img, min_area=400)
        self.assertGreaterEqual(len(objects), 1)
        largest = objects[0]
        self.assertIn(largest["size_level"], ("small", "medium", "large"))
        self.assertIn("width_px", largest)
        self.assertIn("height_px", largest)
        self.assertIn("edge_points_px", largest)
        self.assertGreater(len(largest["edge_points_px"]), 2)

    def test_build_plugins_instantiates_new_cards(self):
        fake_client = mock.MagicMock()
        fake_ros2 = mock.MagicMock()
        fake_ros2.executor_core = None

        config = {
            "mcp_port": 15718,
            "ext_camera": {"enabled": False},
            "aruco_detector": {"enabled": True},
            "color_detector": {"enabled": True},
            "object_segmentor": {"enabled": True},
            "vision_capture": {"enabled": False},
        }

        with mock.patch.dict("sys.modules", {"numpy": mock.MagicMock(), "cv2": mock.MagicMock()}):
            from device import build_plugins
            with mock.patch("device.RM75SDKClient") as mock_client_cls:
                mock_client_cls.return_value = fake_client
                fake_client.shared_client.return_value = fake_client
                fake_client.exclusive_client.return_value = fake_client

                plugins = build_plugins(config, namespace="nvidia_desktop", ros2=fake_ros2)
                plugin_names = [getattr(p, "PREFIX", getattr(p, "name", type(p).__name__)) for p in plugins]

                self.assertIn("color_detector", plugin_names)
                self.assertIn("object_segmentor", plugin_names)
                self.assertIn("aruco_detector", plugin_names)


if __name__ == "__main__":
    unittest.main()
