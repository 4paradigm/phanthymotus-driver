"""Hardware-free checks for the ArUco canvas card's region contract."""

import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


SOURCE = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v/aruco_canvas.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
# Geometry and validation are pure Python; do not require ROS/OpenCV on CI hosts.
selected = [node for node in tree.body if isinstance(node, (ast.Assign, ast.FunctionDef))]
namespace = {}
exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"), namespace)


class ArucoCanvasRegionTests(unittest.TestCase):
    def test_driver_registers_card_and_packages_aruco_runtime(self):
        driver = SOURCE.parent
        self.assertIn("name: aruco_zones,      type: processor",
                      (driver / "driver.yaml").read_text())
        self.assertIn("aruco_zones:\n  enabled: true", (driver / "config.yaml").read_text())
        dockerfile = (driver / "Dockerfile").read_text()
        self.assertIn("opencv-contrib-python-headless==4.11.0.86", dockerfile)
        self.assertIn("aruco_canvas.py", dockerfile)

    def test_canvas_manifest_and_connection_contract(self):
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
            spec = importlib.util.spec_from_file_location("realman_aruco_canvas_test", SOURCE)
            card_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(card_module)
            executor = type("Executor", (), {"context": object()})()
            plugin = card_module.ArucoZonesPlugin({}, "rm75", executor)
            manifest = plugin.get_tool()
            self.assertEqual(manifest["type"], "processor")
            self.assertEqual(manifest["topic_in"][0]["format"], "image/jpeg")
            self.assertEqual([out["format"] for out in manifest["topic_out"]],
                             ["image/jpeg", "data/json"])
            self.assertIn("waiting_rect", manifest["configSchema"]["properties"])
            with self.assertRaises(ValueError):
                plugin.dispatch("start", {})
            self.assertEqual(plugin.dispatch("info", {})["state"], "idle")

    def test_three_unordered_markers_bind_by_center(self):
        config, rects = namespace["validate_config"]({
            "waiting_rect": "0,0,0.3,1", "sorting_1_rect": "0.35,0,0.65,1",
            "sorting_2_rect": "0.7,0,1,1", "sorting_1_color": "red",
            "sorting_2_color": "blue"})
        markers = [{"id": 9, "center_px": [80, 50]},
                   {"id": 2, "center_px": [20, 50]},
                   {"id": 7, "center_px": [50, 50]}]
        result = namespace["assign_zones"](markers, rects, 100, 100, config)
        self.assertTrue(result["ready"])
        self.assertEqual([result["zones"][role]["marker_id"] for role in namespace["ROLES"]],
                         [2, 7, 9])
        self.assertFalse(result["motion_enabled"])

    def test_overlap_and_invalid_coordinates_rejected(self):
        with self.assertRaises(ValueError):
            namespace["validate_config"]({"waiting_rect": "0,0,0.5,0.5",
                                          "sorting_1_rect": "0.4,0.4,0.8,0.8"})
        with self.assertRaises(ValueError):
            namespace["parse_rect"]("-0.1,0,0.5,1")

    def test_missing_or_multiple_markers_not_ready(self):
        config, rects = namespace["validate_config"]({
            "waiting_rect": "0,0,0.3,1", "sorting_1_rect": "0.35,0,0.65,1",
            "sorting_2_rect": "0.7,0,1,1"})
        markers = [{"id": 2, "center_px": [20, 50]},
                   {"id": 7, "center_px": [50, 50]},
                   {"id": 8, "center_px": [55, 50]}]
        result = namespace["assign_zones"](markers, rects, 100, 100, config)
        self.assertFalse(result["ready"])
        self.assertIsNone(result["zones"]["sorting_1"]["marker_id"])
        self.assertIsNone(result["zones"]["sorting_2"]["marker_id"])


if __name__ == "__main__":
    unittest.main()
