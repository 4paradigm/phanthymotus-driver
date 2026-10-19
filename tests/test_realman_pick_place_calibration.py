"""Read-only profile selection against the RealSense SDK data contract."""

import hashlib
import sys
import types
import unittest
from unittest import mock
import zlib

import test_realman_pick_place as fixtures
from pick_place.calibration import read_calibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.serial = "camera-a"
        self.publisher = "robot_realsense_rgbd_" + hashlib.sha256(self.serial.encode()).hexdigest()[:12]
        self.intr = types.SimpleNamespace(width=80, height=60, fx=80., fy=60., ppx=40., ppy=30.,
                                          model="distortion.none", coeffs=[0.] * 5)
        self.color = self.profile("color", "bgr8", self.intr)
        self.depth = self.profile("depth", "z16", types.SimpleNamespace(
            **{**vars(self.intr), "width": 40, "height": 30}))
        self.profiles = [self.color, self.depth]
        self.sensor = mock.Mock()
        self.sensor.get_stream_profiles.side_effect = lambda: self.profiles
        self.device = mock.Mock()
        self.device.get_info.return_value = self.serial
        self.device.query_sensors.return_value = [self.sensor]
        self.context = mock.Mock()
        self.context.query_devices.return_value = [self.device]
        self.sdk = types.SimpleNamespace(context=mock.Mock(return_value=self.context),
                                        stream=types.SimpleNamespace(color="color", depth="depth"),
                                        format=types.SimpleNamespace(bgr8="bgr8", z16="z16"),
                                        camera_info=types.SimpleNamespace(serial_number="serial"),
                                        pipeline=mock.Mock())
        self.enterContext(mock.patch.dict(sys.modules, {"pyrealsense2": self.sdk}))
        self.payload = zlib.compress(b"\x90\x01" * 40 * 30)

    @staticmethod
    def profile(kind, fmt, intr):
        profile = mock.Mock()
        profile.is_video_stream_profile.return_value = True
        profile.as_video_stream_profile.return_value = profile
        profile.stream_type.return_value = kind
        profile.format.return_value = fmt
        profile.width.return_value, profile.height.return_value = intr.width, intr.height
        profile.get_intrinsics.return_value = intr
        profile.get_extrinsics_to.return_value = types.SimpleNamespace(
            rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[.02, 0., 0.])
        return profile

    def read(self, **kwargs):
        args = dict(publisher=self.publisher, rgb_shape=(60, 80), depth_payload=self.payload, session_id="local")
        return read_calibration(**{**args, **kwargs})

    def test_queries_matching_device_without_opening_or_configuring_it(self):
        other = mock.Mock()
        other.get_info.return_value = "unrelated-camera"
        self.context.query_devices.return_value = [other, self.device]
        result = self.read()
        self.assertEqual(result["serial_number"], self.serial)
        self.assertEqual(result["rgb_intrinsics"]["width"], 80)
        self.assertEqual(result["depth_intrinsics"]["width"], 40)
        self.assertEqual(result["depth_scale_m"], .001)
        self.assertEqual(result["depth_to_color"]["translation"], [.02, 0., 0.])
        self.depth.get_extrinsics_to.assert_called_once_with(self.color)
        self.sdk.pipeline.assert_not_called()
        for method in (self.sensor.open, self.sensor.start, self.sensor.set_option,
                       self.device.first_depth_sensor, other.query_sensors):
            method.assert_not_called()

    def test_frame_rate_variants_with_identical_calibration_are_not_ambiguous(self):
        self.profiles.append(self.profile("color", "bgr8", self.intr))
        self.assertEqual(self.read()["rgb_intrinsics"]["fx"], 80.)

    def test_missing_wrong_or_ambiguous_local_device_is_rejected(self):
        for devices in ([], [self.device, self.device]):
            with self.subTest(devices=len(devices)), self.assertRaisesRegex(ValueError, "one locally"):
                self.context.query_devices.return_value = devices
                self.read()
        self.context.query_devices.return_value = [self.device]
        with self.assertRaisesRegex(ValueError, "one locally"):
            self.read(publisher="different-camera-node")

    def test_resolution_and_ambiguous_calibration_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "dimensions"):
            self.read(rgb_shape=(480, 640))
        different = types.SimpleNamespace(**{**vars(self.intr), "fx": 81.})
        self.profiles.append(self.profile("color", "bgr8", different))
        with self.assertRaisesRegex(ValueError, "dimensions"):
            self.read()

    def test_depth_payload_and_sdk_failure_do_not_start_acquisition(self):
        for payload in (self.payload[:-1], self.payload + b"extra", zlib.compress(b"short")):
            with self.subTest(size=len(payload)), self.assertRaises(ValueError):
                self.read(depth_payload=payload)
        self.color.get_intrinsics.side_effect = RuntimeError("unplugged")
        with self.assertRaisesRegex(RuntimeError, "unplugged"):
            self.read()
        self.sdk.pipeline.assert_not_called()
        self.sensor.open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
