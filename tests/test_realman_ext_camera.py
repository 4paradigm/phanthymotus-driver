"""RealMan RealSense discovery and ext_camera card contract regressions."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


SOURCE = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v/camera.py"
spec = importlib.util.spec_from_file_location("realman_ext_camera_test", SOURCE)
ext = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ext)


class FakeDevice:
    def __init__(self, serial="1234", name="Intel RealSense D435", usb="3.2"):
        self.values = {"serial": serial, "name": name, "usb": usb}

    def supports(self, field):
        return field in self.values and self.values[field] is not None

    def get_info(self, field):
        return self.values[field]


def sdk_with(devices=None, error=None):
    module = types.ModuleType("pyrealsense2")
    module.camera_info = types.SimpleNamespace(
        serial_number="serial", name="name", usb_type_descriptor="usb")

    def context():
        if error:
            raise error
        return types.SimpleNamespace(query_devices=lambda: list(devices or []))

    module.context = context
    return module


class CameraDiscoveryTests(unittest.TestCase):
    def enumerate(self, devices=None, error=None):
        with mock.patch.dict(sys.modules, {
            "pyrealsense2": sdk_with(devices, error),
        }):
            return ext._enumerate_ext_cameras()

    def test_real_sense_is_exposed_by_stable_serial_not_video_number(self):
        self.assertEqual(self.enumerate([FakeDevice()]), [{
            "path": "realsense://1234",
            "name": "Intel RealSense D435",
            "serial_number": "1234",
            "usb_type": "3.2",
            "realsense": True,
            "channels": ["rgb", "depth", "infrared"],
        }])

    def test_no_camera_and_enumeration_failure_are_nonfatal(self):
        self.assertEqual(self.enumerate([]), [])
        self.assertEqual(self.enumerate(error=RuntimeError("USB unavailable")), [])

    def test_devices_without_serial_are_ignored_and_duplicates_are_removed(self):
        result = self.enumerate([
            FakeDevice(serial=None), FakeDevice(serial="1234"),
            FakeDevice(serial="1234", name="duplicate"), FakeDevice(serial="5678"),
        ])
        self.assertEqual([item["serial_number"] for item in result], ["1234", "5678"])


class FakeSession:
    def __init__(self, namespace, serial_number):
        self.namespace = namespace
        self.serial_number = serial_number
        self.routes = {}
        self.starts = 0

    def start(self, instance_id, channel):
        self.starts += 1
        self.routes[instance_id] = channel
        return self.info(instance_id, channel)

    def stop(self, instance_id):
        self.routes.pop(instance_id, None)

    def info(self, instance_id, channel):
        return {
            "state": "running" if instance_id in self.routes else "idle",
            "channel": channel,
        }


class CameraChannelTests(unittest.TestCase):
    def setUp(self):
        self.devices = [
            {
                "path": "realsense://1234",
                "name": "Intel RealSense D435",
                "serial_number": "1234",
                "usb_type": "3.2",
                "realsense": True,
                "channels": ["rgb", "depth", "infrared"],
            }
        ]
        enumeration = mock.patch.object(
            ext, "_enumerate_ext_cameras", return_value=self.devices)
        enumeration.start()
        self.addCleanup(enumeration.stop)
        sdk = mock.patch.dict(sys.modules, {
            "realsense": types.SimpleNamespace(RealSenseSession=FakeSession),
        })
        sdk.start()
        self.addCleanup(sdk.stop)
        self.plugin = ext.ExtCameraPlugin({}, "robot_a", None)

    def test_one_pure_sensor_with_serial_and_channel_configuration(self):
        tool = self.plugin.get_tools()[0]
        self.assertEqual(tool["name"], "ext_camera")
        self.assertTrue(tool["multiInstance"])
        properties = tool["configSchema"]["properties"]
        self.assertEqual(
            properties["device_path"]["oneOf"],
            [{"const": "realsense://1234", "title": "Intel RealSense D435 (1234)"}],
        )
        self.assertEqual(
            properties["channel"]["enum"], ["rgb", "depth", "infrared"])
        self.assertNotIn("resolution", properties)
        self.assertNotIn("pixel_format", properties)

    def test_rgb_depth_and_infrared_share_one_physical_owner(self):
        channels = {"card-a": "rgb", "card-b": "depth", "card-c": "infrared"}
        for instance_id, channel in channels.items():
            result = self.plugin.dispatch(
                "start", {"instance_id": instance_id, "channel": channel})
            self.assertEqual(result["state"], "running")
        self.assertEqual(len(self.plugin._sessions), 1)
        self.assertEqual(self.plugin._sessions["1234"].routes, channels)
        self.plugin.dispatch("stop", {"instance_id": "card-b"})
        self.assertEqual(
            self.plugin._sessions["1234"].routes,
            {"card-a": "rgb", "card-c": "infrared"},
        )

    def test_multiple_rgb_cards_share_the_same_capture(self):
        self.plugin.dispatch("start", {"instance_id": "rgb-a", "channel": "rgb"})
        self.plugin.dispatch("start", {"instance_id": "rgb-b", "channel": "rgb"})
        self.assertEqual(
            self.plugin._sessions["1234"].routes,
            {"rgb-a": "rgb", "rgb-b": "rgb"},
        )

    def test_running_card_switches_modality_and_topic_without_new_session(self):
        args = {"instance_id": "card-a"}
        self.plugin.dispatch("start", args)
        session = self.plugin._sessions["1234"]
        for channel in ("depth", "infrared", "rgb"):
            result = self.plugin.dispatch("config", {**args, "channel": channel})
            expected_format = (
                "image/depth-zlib" if channel == "depth" else "image/jpeg")
            self.assertEqual(result["state"], "running")
            self.assertEqual(result["topic_out"], [{
                "topic": f"/robot_a/ext_camera/card_a/{channel}",
                "format": expected_format,
            }])
            self.assertEqual(session.routes, {"card-a": channel})
        self.assertIs(session, self.plugin._sessions["1234"])

    def test_idle_config_infers_topic_without_starting(self):
        result = self.plugin.dispatch(
            "config", {"instance_id": "card-a", "channel": "depth"})
        self.assertEqual(result["state"], "idle")
        self.assertEqual(result["topic_out"][0]["format"], "image/depth-zlib")
        self.assertEqual(self.plugin._nodes, {})

    def test_saved_channel_configs_are_isolated(self):
        channels = {"card-a": "rgb", "card-b": "depth", "card-c": "infrared"}
        for instance_id, channel in channels.items():
            self.plugin.dispatch(
                "config", {"instance_id": instance_id, "channel": channel})
        for instance_id, channel in channels.items():
            result = self.plugin.dispatch("start", {"instance_id": instance_id})
            self.assertEqual(result["channel"], channel)
        self.assertEqual(self.plugin._sessions["1234"].routes, channels)

    def test_legacy_video_path_migrates_for_one_connected_camera(self):
        result = self.plugin.dispatch("config", {
            "instance_id": "card-a", "device_path": "/dev/video4"})
        self.assertEqual(result["device_path"], "realsense://1234")

    def test_selected_serial_chooses_the_intended_camera(self):
        second = {**self.devices[0], "path": "realsense://5678", "serial_number": "5678"}
        self.devices.append(second)
        self.plugin.dispatch("start", {
            "instance_id": "card-a", "device_path": "realsense://5678"})
        self.assertEqual(list(self.plugin._sessions), ["5678"])

    def test_repeated_config_does_not_restart_and_topic_ids_do_not_collide(self):
        args = {"instance_id": "card-a"}
        self.plugin.dispatch("start", args)
        node = self.plugin._nodes["card-a"]
        session = self.plugin._sessions["1234"]
        self.plugin.dispatch("config", {**args, "channel": "rgb"})
        self.assertIs(node, self.plugin._nodes["card-a"])
        self.assertEqual(session.starts, 1)
        with self.assertRaisesRegex(ValueError, "collides"):
            self.plugin.dispatch("start", {"instance_id": "card_a"})

    def test_start_preserves_readiness_and_known_capture_errors(self):
        with mock.patch.object(
            FakeSession, "info", return_value={"state": "starting", "fresh": False}
        ):
            result = self.plugin.dispatch("start", {
                "instance_id": "card-a", "channel": "rgb"})
            self.assertEqual(result["state"], "running")
            self.assertEqual(result["readiness"], "starting")
            self.assertFalse(result["fresh"])

        self.plugin.dispatch("stop", {"instance_id": "card-a"})
        with mock.patch.object(FakeSession, "info", return_value={
            "state": "error", "fresh": False, "error": "Device disconnected",
        }):
            result = self.plugin.dispatch("start", {
                "instance_id": "card-a", "channel": "depth"})
            self.assertEqual(result["state"], "error")
            self.assertEqual(result["error"], "Device disconnected")


class CameraAbsentTests(unittest.TestCase):
    def test_plugin_loads_without_camera_and_start_reports_unavailable(self):
        with mock.patch.object(ext, "_enumerate_ext_cameras", return_value=[]):
            plugin = ext.ExtCameraPlugin({}, "robot_a", None)
            options = plugin.get_tools()[0]["configSchema"]["properties"]["device_path"]["oneOf"]
            self.assertEqual(options, [{"const": "", "title": "无可用 RealSense 设备"}])
            with self.assertRaisesRegex(ValueError, "unavailable"):
                plugin.dispatch("start", {"instance_id": "card-a"})


if __name__ == "__main__":
    unittest.main()
