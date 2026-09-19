"""Calibration sideband preserves ext_camera image payloads and source identity."""

import time
import types
import unittest
from unittest import mock

import test_realman_realsense as fixtures

rs = fixtures.rs


class MetadataTests(unittest.TestCase):
    def frame(self, width, height, stamp):
        intr = types.SimpleNamespace(
            width=width,
            height=height,
            fx=500.0,
            fy=501.0,
            ppx=width / 2,
            ppy=height / 2,
            model="distortion.none",
            coeffs=[0] * 5,
        )
        extr = types.SimpleNamespace(rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1], translation=[0.02, 0, 0])
        profile = types.SimpleNamespace(get_intrinsics=lambda: intr, get_extrinsics_to=lambda other: extr)
        return types.SimpleNamespace(
            profile=types.SimpleNamespace(as_video_stream_profile=lambda: profile),
            get_timestamp=lambda: stamp * 1000,
            get_frame_timestamp_domain=lambda: "global",
        )

    def test_frame_timestamp_is_acquisition_time_not_publication_time(self):
        sdk = types.SimpleNamespace(
            timestamp_domain=types.SimpleNamespace(system_time="system", global_time="global")
        )
        with mock.patch.object(rs.time, "time", return_value=1000):
            self.assertEqual(rs.frame_time_ns(self.frame(1280, 720, 999.9), sdk), 999900000000)
            for stamp in (998, 1001, float("nan")):
                with self.assertRaises(ValueError):
                    rs.frame_time_ns(self.frame(1280, 720, stamp), sdk)
            frame = self.frame(1280, 720, 1000)
            frame.get_frame_timestamp_domain = lambda: "hardware"
            with self.assertRaises(ValueError):
                rs.frame_time_ns(frame, sdk)

    def test_metadata_preserves_native_profiles_extrinsics_and_wire_depth_units(self):
        frames = {"rgb": self.frame(1280, 720, 1000), "depth": self.frame(640, 480, 1000.001)}
        result = rs.rgbd_metadata(
            frames, {"rgb": 1000000000000, "depth": 1000001000000}, "serial", "session", ["/cam/rgb"]
        )
        self.assertEqual(result["rgb_intrinsics"]["width"], 1280)
        self.assertEqual(result["depth_intrinsics"]["width"], 640)
        self.assertEqual(result["depth_scale_m"], 0.001)
        self.assertEqual(result["depth_aligned_to"], "depth")
        self.assertEqual(result["depth_to_color"]["translation"], [0.02, 0, 0])
        self.assertEqual(result["rgb_topics"], ["/cam/rgb"])
        self.assertEqual(result["session_id"], "session")
        self.assertEqual(result["depth_stamp_ns"] - result["rgb_stamp_ns"], 1000000)


if __name__ == "__main__":
    unittest.main()
