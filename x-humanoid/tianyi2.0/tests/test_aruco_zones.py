"""Hardware-free checks for the Tianyi ArUco canvas card."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "aruco_zones.py"
SPEC = importlib.util.spec_from_file_location("tianyi_aruco_zones", MODULE_PATH)
aruco = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(aruco)


class ArucoZonesTests(unittest.TestCase):
    def test_marker_ids_anchor_sized_zones_as_markers_move(self):
        config, sizes = aruco.validate_config({
            "waiting_marker_id": "7", "waiting_size": "0.2,0.2",
            "sorting_1_marker_id": "2", "sorting_1_size": "0.2,0.2",
            "sorting_2_marker_id": "9", "sorting_2_size": "0.2,0.2",
        })
        markers = [
            {"id": 7, "center_px": [85, 50]},
            {"id": 2, "center_px": [50, 50]},
            {"id": 9, "center_px": [15, 50]},
        ]
        result = aruco.assign_zones(markers, sizes, 100, 100, config)
        self.assertTrue(result["ready"])
        self.assertFalse(result["motion_enabled"])
        self.assertEqual(result["zones"]["sorting_1"]["marker_id"], 2)
        for actual, expected in zip(result["zones"]["waiting"]["rect_normalized"],
                                    [0.75, 0.4, 0.95, 0.6]):
            self.assertAlmostEqual(actual, expected)
        markers[0]["center_px"] = [70, 50]
        moved = aruco.assign_zones(markers, sizes, 100, 100, config)
        for actual, expected in zip(moved["zones"]["waiting"]["rect_normalized"],
                                    [0.6, 0.4, 0.8, 0.6]):
            self.assertAlmostEqual(actual, expected)
        self.assertFalse(aruco.assign_zones(markers[:2], sizes, 100, 100, config)["ready"])

    def test_invalid_sizes_and_duplicate_ids_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "必须不同"):
            aruco.validate_config({"waiting_marker_id": "2", "sorting_1_marker_id": "2"})
        with self.assertRaisesRegex(ValueError, "宽、高"):
            aruco.parse_size("0,0.2")
        with self.assertRaisesRegex(ValueError, "须小于"):
            aruco.validate_config({"waiting_marker_id": "50"})

    def test_overlapping_or_clipped_zones_are_not_ready(self):
        config, sizes = aruco.validate_config({
            "waiting_marker_id": "1", "waiting_size": "0.3,0.3",
            "sorting_1_marker_id": "2", "sorting_1_size": "0.3,0.3",
            "sorting_2_marker_id": "3", "sorting_2_size": "0.3,0.3",
        })
        markers = [{"id": 1, "center_px": [20, 50]},
                   {"id": 2, "center_px": [30, 50]},
                   {"id": 3, "center_px": [80, 50]}]
        result = aruco.assign_zones(markers, sizes, 100, 100, config)
        self.assertTrue(result["zones_overlap"])
        self.assertFalse(result["ready"])
        markers[0]["center_px"] = [5, 50]
        markers[1]["center_px"] = [50, 50]
        result = aruco.assign_zones(markers, sizes, 100, 100, config)
        self.assertTrue(result["zones"]["waiting"]["clipped"])
        self.assertFalse(result["ready"])

    def test_tool_contract_and_runtime_metadata_filter(self):
        class DummyROS:
            pass

        plugin = aruco.ArucoZonesPlugin({"enabled": True}, "tianyi", DummyROS())
        tool = plugin.get_tool()
        self.assertEqual(tool["name"], "aruco_zones")
        self.assertEqual(tool["type"], "processor")
        self.assertIn("7", tool["inputSchema"]["properties"]["waiting_marker_id"]["enum"])
        self.assertIn("waiting_size", tool["configSchema"]["properties"])
        self.assertEqual(tool["topic_in"][0]["format"], "image/jpeg")
        self.assertEqual(tool["topic_out"][0]["topic"], "/tianyi/aruco_zones/overlay")
        self.assertEqual(tool["topic_out"][1]["format"], "data/json")
        result = plugin.dispatch("config", {"sorting_1_color": "green",
                                            "_tool_name": "aruco_zones"})
        self.assertEqual(result["config"]["sorting_1_color"], "green")

    def test_process_image_honors_stride_and_rgb_encoding(self):
        import numpy as np

        captured = []

        class Detector:
            def __init__(self, dictionary):
                self.dictionary = dictionary

            def detectMarkers(self, gray):
                return [], None, []

        def cvt_color(image, conversion):
            if conversion == 1:
                return image[:, :, ::-1].copy()
            return np.zeros(image.shape[:2], dtype=np.uint8)

        def encode(extension, frame, options):
            captured.append(frame.copy())
            return True, np.array([1, 2, 3], dtype=np.uint8)

        fake_cv2 = SimpleNamespace(
            COLOR_RGB2BGR=1,
            COLOR_BGR2GRAY=2,
            IMWRITE_JPEG_QUALITY=3,
            cvtColor=cvt_color,
            imencode=encode,
            aruco=SimpleNamespace(
                DICT_4X4_50=4,
                getPredefinedDictionary=lambda marker_id: marker_id,
                ArucoDetector=Detector,
            ),
        )
        config, rects = aruco.validate_config({})
        rgb = SimpleNamespace(width=2, height=1, step=8, encoding="rgb8",
                              data=bytes([10, 20, 30, 40, 50, 60, 99, 99]))
        bgr = SimpleNamespace(width=2, height=1, step=8, encoding="bgr8",
                              data=bytes([30, 20, 10, 60, 50, 40, 99, 99]))
        with mock.patch.dict(sys.modules, {"cv2": fake_cv2}):
            first, jpeg = aruco.process_image(rgb, config, rects)
            second, _ = aruco.process_image(bgr, config, rects)

        self.assertEqual(jpeg, b"\x01\x02\x03")
        self.assertEqual(first["image_width"], 2)
        self.assertEqual(first["image_height"], 1)
        self.assertFalse(first["ready"])
        self.assertFalse(second["motion_enabled"])
        self.assertEqual(captured[0].tolist(), [[[30, 20, 10], [60, 50, 40]]])
        self.assertEqual(captured[1].tolist(), captured[0].tolist())

    def test_process_image_rejects_incomplete_buffer(self):
        with mock.patch.dict(sys.modules, {"cv2": SimpleNamespace()}):
            msg = SimpleNamespace(width=2, height=1, step=8, encoding="rgb8",
                                  data=bytes([10, 20, 30]))
            with self.assertRaisesRegex(ValueError, "数据不完整"):
                aruco.process_image(msg, *aruco.validate_config({}))


if __name__ == "__main__":
    unittest.main()
