"""Public four-card S10 monitoring contract; no robot or ROS dependency."""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "deep_robotics/s10_pro"
sys.path.insert(0, str(SOURCE))
spec = importlib.util.spec_from_file_location("s10_four_monitors", SOURCE / "device.py")
device = importlib.util.module_from_spec(spec)
assert spec.loader
spec.loader.exec_module(device)


class Bridge:
    def acquire(self, *_args):
        pass

    def release(self, *_args):
        pass

    def snapshot(self, _source):
        return {}

    def status(self):
        return {"reader_running": False, "last_error": None}


class FourMonitorContract(unittest.TestCase):
    def setUp(self):
        self.plugins = device.build_plugins(
            {}, "s10_pro", Bridge(), lambda *_args: None, public_only=True
        )

    def tearDown(self):
        for plugin in self.plugins:
            plugin.stop()

    def test_public_catalog_contains_only_four_monitors(self):
        self.assertEqual(
            ["battery", "joints", "lidar_cloud", "imu"],
            [plugin.name for plugin in self.plugins],
        )
        tools = [tool for plugin in self.plugins for tool in plugin.get_tools()]
        self.assertEqual(
            {"battery", "joints", "model", "lidar_cloud", "imu"},
            {tool["name"] for tool in tools},
        )
        self.assertEqual(
            ["data/json", "sensor/skeleton"],
            [port["format"] for port in self.plugins[1].get_tool()["topic_out"]],
        )
        self.assertEqual(
            "sensor/pointcloud", self.plugins[2].get_tool()["topic_out"][0]["format"]
        )

    def test_empty_schema_calls_start_each_public_monitor_through_bundle(self):
        from common.vendor_runtime import DriverBundle

        class RecordingBridge(Bridge):
            def __init__(self):
                self.acquired = []

            def acquire(self, name, *_args):
                self.acquired.append(name)

        bridge = RecordingBridge()
        plugins = device.build_plugins(
            {}, "s10_pro", bridge, lambda *_args: None, public_only=True
        )
        bundle = DriverBundle(plugins)
        try:
            for name in ("battery", "joints", "lidar_cloud", "imu"):
                with self.subTest(name=name):
                    definition = next(t for t in bundle.get_all_tools() if t["name"] == name)
                    self.assertEqual({}, definition["inputSchema"]["properties"])
                    result = bundle.dispatch(name, {})
                    self.assertEqual("running", result["state"])
                    self.assertIn(name, bridge.acquired)
                    bundle.dispatch(name, {})
                    self.assertEqual(1, bridge.acquired.count(name))
                    self.assertEqual("idle", bundle.dispatch(name, {"action": "stop"})["state"])
        finally:
            bundle.stop_all()

    def test_reader_catalog_is_limited_to_monitor_subscriptions(self):
        from reader import DEFAULT_SOURCES
        self.assertEqual({"battery", "joints", "imu", "lidar"}, set(DEFAULT_SOURCES))

    def test_skeleton_marks_the_fixed_mechanical_front_without_changing_dof(self):
        model = self.plugins[1].dispatch("model", {"_tool_name": "model"})
        root = ET.fromstring(model["urdf"])
        movable = [joint for joint in root.findall("joint")
                   if joint.attrib["type"] != "fixed"]
        self.assertEqual(16, len(movable))
        fixed = {joint.attrib["name"]: joint for joint in root.findall("joint")
                 if joint.attrib["type"] == "fixed"}
        self.assertEqual(
            {"head_direction_tip_fixed", "head_direction_left_fixed",
             "head_direction_right_fixed", "head_direction_tip_cap_fixed"},
            set(fixed),
        )
        self.assertEqual(
            "0.34 0 0.06",
            fixed["head_direction_tip_fixed"].find("origin").attrib["xyz"],
        )

        moving = {joint.attrib["name"]: joint for joint in movable}
        self.assertEqual("0 0.35 0", moving["fl_hipy_joint"].find("origin").attrib["rpy"])
        self.assertEqual("0 -0.65 0", moving["fl_knee_joint"].find("origin").attrib["rpy"])
        self.assertEqual("0 -0.35 0", moving["hl_hipy_joint"].find("origin").attrib["rpy"])
        self.assertEqual("0 0.65 0", moving["hl_knee_joint"].find("origin").attrib["rpy"])

        from s10_skeleton import (
            JOINT_DIRECTIONS,
            JOINT_OFFSETS_DEG,
            MODEL_REFERENCE_ANGLES,
            model_angle,
        )

        for index, reference in enumerate(MODEL_REFERENCE_ANGLES):
            # Hardware zero values are not the model zero.  A sample that maps
            # to the nominal pose must publish a zero delta to the renderer.
            raw = (reference - math.radians(JOINT_OFFSETS_DEG[index])) / JOINT_DIRECTIONS[index]
            self.assertAlmostEqual(0.0, model_angle(index, raw))

    def test_readable_battery_and_imu_payloads(self):
        battery = device._monitor_output("battery", {"data": [{
            "battery_level": 72, "current": -1.25, "voltage": 51.4,
            "battery_temperature": [31.0, 32.0], "cycles": 18,
        }]})
        self.assertEqual(72, battery["soc"])
        self.assertEqual(-1.25, battery["current"])
        self.assertEqual([51.4], battery["voltage"])
        self.assertEqual([], battery["cell_vol"])

        imu = device._monitor_output("imu", {
            "header": {"stamp": {"sec": 1793343577, "nanosec": 638000000}},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.1, "y": 0.2, "z": 0.3},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.8},
        })
        self.assertEqual([1.0, 0.0, 0.0, 0.0], imu["quaternion"])
        self.assertEqual([0.0, 0.0, 0.0], imu["rpy"])
        self.assertEqual({"sec": 1793343577, "nanosec": 638000000}, imu["header"]["stamp"])

    def test_imu_timestamp_preserves_ros_time_and_does_not_fabricate_missing_time(self):
        for stamp in ({"sec": 0, "nanosec": 0}, {"sec": -1, "nanosec": 999999999}):
            self.assertEqual(stamp, device._monitor_output("imu", {"header": {"stamp": stamp}})["header"]["stamp"])
        for source in ({}, {"header": None}, {"header": {"stamp": {"sec": 1}}},
                       {"header": {"stamp": {"sec": True, "nanosec": 0}}},
                       {"header": {"stamp": {"sec": 1, "nanosec": 1000000000}}}):
            self.assertIsNone(device._monitor_output("imu", source)["header"]["stamp"])


if __name__ == "__main__":
    unittest.main()
