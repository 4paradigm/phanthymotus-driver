"""Hardware-free checks for the Tianyi ArUco canvas card."""

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "aruco_zones.py"
SPEC = importlib.util.spec_from_file_location("tianyi_aruco_zones", MODULE_PATH)
aruco = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(aruco)


class ArucoZonesTests(unittest.TestCase):
    def test_three_distinct_markers_make_zones_ready(self):
        config, rects = aruco.validate_config({
            "waiting_rect": "0,0,0.3,1",
            "sorting_1_rect": "0.35,0,0.65,1",
            "sorting_2_rect": "0.7,0,1,1",
        })
        markers = [
            {"id": 1, "center_px": [15, 50]},
            {"id": 2, "center_px": [50, 50]},
            {"id": 3, "center_px": [85, 50]},
        ]
        result = aruco.assign_zones(markers, rects, 100, 100, config)
        self.assertTrue(result["ready"])
        self.assertFalse(result["motion_enabled"])
        self.assertEqual(result["zones"]["sorting_1"]["marker_id"], 2)
        self.assertFalse(aruco.assign_zones(markers[:2], rects, 100, 100, config)["ready"])
        markers[2]["id"] = 2
        self.assertFalse(aruco.assign_zones(markers, rects, 100, 100, config)["ready"])

    def test_overlapping_regions_and_invalid_rectangles_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "重叠"):
            aruco.validate_config({"waiting_rect": "0,0,0.6,1",
                                   "sorting_1_rect": "0.5,0,1,1"})
        with self.assertRaisesRegex(ValueError, "区域须满足"):
            aruco.parse_rect("0.8,0,0.2,1")

    def test_tool_contract_and_runtime_metadata_filter(self):
        class DummyROS:
            pass

        plugin = aruco.ArucoZonesPlugin({"enabled": True}, "tianyi", DummyROS())
        tool = plugin.get_tool()
        self.assertEqual(tool["name"], "aruco_zones")
        self.assertEqual(tool["type"], "processor")
        self.assertEqual(tool["topic_in"][0]["format"], "image/jpeg")
        self.assertEqual(tool["topic_out"][0]["topic"], "/tianyi/aruco_zones/overlay")
        result = plugin.dispatch("config", {"sorting_1_color": "green",
                                            "_tool_name": "aruco_zones"})
        self.assertEqual(result["config"]["sorting_1_color"], "green")


if __name__ == "__main__":
    unittest.main()
