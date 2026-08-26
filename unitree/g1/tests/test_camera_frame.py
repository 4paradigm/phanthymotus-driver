import math
from pathlib import Path
import struct
import unittest
import zlib

import yaml

from camera_frame import (
    DEPTH_COMPRESSION,
    DEPTH_COMPRESSION_LEVEL,
    DEPTH_SCHEMA,
    ENVELOPE_FORMAT,
    ENVELOPE_MAGIC,
    RGB_SCHEMA,
    CameraFrameTiming,
    RealSenseClockNormalizer,
    build_calibrations,
    build_depth_image_metadata,
    build_frame_metadata,
    build_intrinsics,
    compress_depth_payload,
    decode_envelope,
    encode_envelope,
    load_lidar_camera_calibration,
    realsense_extrinsics_transform,
)


G1_DIR = Path(__file__).resolve().parents[1]
FACTORY_CALIBRATION = G1_DIR / "calibration" / "g1_factory_nominal_lidar_camera.yaml"


def make_intrinsics(width=640, height=480, fx=600.0):
    return build_intrinsics(
        width=width,
        height=height,
        fx=fx,
        fy=601.0,
        cx=width / 2,
        cy=height / 2,
        coefficients=[0.1, -0.2, 0.0, 0.0, 0.01],
        realsense_model="distortion.brown_conrady",
    )


class CameraEnvelopeTest(unittest.TestCase):
    def test_public_contract_uses_frame_v1_names(self):
        self.assertEqual(ENVELOPE_MAGIC, b"PSE1")
        self.assertEqual(
            ENVELOPE_FORMAT, "application/vnd.phanthy.sensor-envelope.v1"
        )
        self.assertEqual(RGB_SCHEMA, "phanthy.sensor.camera_rgb_frame.v1")
        self.assertEqual(DEPTH_SCHEMA, "phanthy.sensor.camera_depth_frame.v1")

    def test_binary_envelope_round_trip(self):
        metadata = {"schema": RGB_SCHEMA, "header": {"stamp_ns": None}}
        payload = b"\xff\xd8jpeg\xff\xd9"

        decoded_metadata, decoded_payload = decode_envelope(
            encode_envelope(metadata, payload)
        )

        self.assertEqual(decoded_metadata, metadata)
        self.assertEqual(decoded_payload, payload)

    def test_binary_envelope_rejects_truncation(self):
        envelope = encode_envelope({"schema": DEPTH_SCHEMA}, b"depth")
        with self.assertRaisesRegex(ValueError, "length mismatch"):
            decode_envelope(envelope[:-1])

    def test_depth_payload_compression_is_lossless(self):
        raw = b"\x00\x00\x10\x00" * 1024
        compressed = compress_depth_payload(raw)

        self.assertEqual(DEPTH_COMPRESSION, "zlib")
        self.assertEqual(DEPTH_COMPRESSION_LEVEL, 1)
        self.assertLess(len(compressed), len(raw))
        self.assertEqual(zlib.decompress(compressed), raw)

    def test_depth_envelope_declares_raw_units_and_meter_scale(self):
        raw = struct.pack("<H", 1000)
        payload = compress_depth_payload(raw)
        image = build_depth_image_metadata(
            width=1,
            height=1,
            uncompressed_size=len(raw),
            payload_size=len(payload),
            depth_scale_m=0.001,
        )
        metadata, decoded_payload = decode_envelope(
            encode_envelope({"schema": DEPTH_SCHEMA, "image": image}, payload)
        )

        raw_value = struct.unpack("<H", zlib.decompress(decoded_payload))[0]
        self.assertEqual(metadata["image"]["unit"], "realsense_depth_unit")
        self.assertEqual(
            metadata["image"]["depth_scale_semantics"],
            "meters_per_realsense_depth_unit",
        )
        self.assertEqual(raw_value * metadata["image"]["depth_scale_m"], 1.0)


class CameraTimingTest(unittest.TestCase):
    def test_system_time_is_preserved(self):
        normalizer = RealSenseClockNormalizer(warmup_samples=2, window_samples=4)
        source_ns = 1_780_000_000_123_000_000
        timing = normalizer.normalize(
            source_timestamp_ms=source_ns / 1_000_000,
            source_domain="timestamp_domain.system_time",
            driver_receive_stamp_ns=source_ns + 2_000_000,
            stream="rgb",
        )
        # RealSense exposes timestamps as floating-point milliseconds, so the
        # round trip can lose sub-microsecond precision at Unix epoch scale.
        self.assertLessEqual(abs(timing.source_stamp_ns - source_ns), 1_000)
        self.assertEqual(timing.clock_domain, "ros_system_time")
        self.assertEqual(timing.normalization_status, "source_system_time")

    def test_direct_clock_domain_matching_is_exact(self):
        for domain in ("not_system_time", "hardware_global_time"):
            with self.subTest(domain=domain):
                timing = RealSenseClockNormalizer(
                    warmup_samples=2, window_samples=4
                ).normalize(
                    source_timestamp_ms=1000.0,
                    source_domain=domain,
                    driver_receive_stamp_ns=20_001_000_000,
                    stream="rgb",
                )
                self.assertIsNone(timing.source_stamp_ns)
                self.assertEqual(timing.clock_domain, "unavailable")
                self.assertEqual(timing.normalization_status, "source_clock_warmup")

    def test_hardware_clock_warms_up_without_faking_receive_time(self):
        normalizer = RealSenseClockNormalizer(warmup_samples=2, window_samples=4)
        first = normalizer.normalize(
            source_timestamp_ms=1000.0,
            source_domain="hardware_clock",
            driver_receive_stamp_ns=20_001_000_000,
            stream="rgb",
        )
        second = normalizer.normalize(
            source_timestamp_ms=1010.0,
            source_domain="hardware_clock",
            driver_receive_stamp_ns=20_011_000_000,
            stream="rgb",
        )
        self.assertIsNone(first.source_stamp_ns)
        self.assertEqual(first.clock_domain, "unavailable")
        self.assertEqual(second.source_stamp_ns, 20_011_000_000)

    def test_out_of_order_source_stamp_is_explicitly_unavailable(self):
        normalizer = RealSenseClockNormalizer(warmup_samples=1, window_samples=2)
        normalizer.normalize(
            source_timestamp_ms=2000.0,
            source_domain="hardware_clock",
            driver_receive_stamp_ns=30_000_000_000,
            stream="depth",
        )
        timing = normalizer.normalize(
            source_timestamp_ms=1999.0,
            source_domain="hardware_clock",
            driver_receive_stamp_ns=30_001_000_000,
            stream="depth",
        )
        self.assertIsNone(timing.source_stamp_ns)
        self.assertTrue(timing.out_of_order)
        self.assertEqual(timing.normalization_status, "source_stamp_out_of_order")


class CameraCalibrationTest(unittest.TestCase):
    def test_intrinsics_restore_camera_matrix(self):
        value = make_intrinsics()
        self.assertEqual(value["distortion_model"], "plumb_bob")
        self.assertEqual(value["k"], [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0])
        self.assertEqual(value["r"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(value["p"][0:3], [600.0, 0.0, 320.0])

    def test_realsense_column_major_rotation_is_converted(self):
        transform = realsense_extrinsics_transform(
            source_frame="depth",
            target_frame="rgb",
            rotation_column_major=[1, 4, 7, 2, 5, 8, 3, 6, 9],
            translation_m=[0.1, 0.2, 0.3],
        )
        self.assertEqual(
            transform["rotation_matrix_row_major"],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        )
        self.assertEqual(transform["matrix_row_major"][3:12:4], [0.1, 0.2, 0.3])

    def test_factory_nominal_extrinsic_is_rigid_and_versioned(self):
        calibration, error = load_lidar_camera_calibration(str(FACTORY_CALIBRATION))
        self.assertIsNone(error)
        self.assertEqual(calibration["status"], "factory_nominal")
        transform = calibration["transform"]
        self.assertEqual(transform["source_frame"], "livox_frame")
        self.assertEqual(transform["target_frame"], "camera_color_optical_frame")
        rotation = transform["rotation_matrix_row_major"]
        determinant = (
            rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
            - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
            + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6])
        )
        self.assertTrue(math.isclose(determinant, 1.0, abs_tol=1e-9))

    def test_missing_extrinsic_is_not_replaced_by_identity(self):
        calibration, error = load_lidar_camera_calibration(None)
        self.assertEqual(error, "configuration_missing")
        self.assertEqual(calibration["status"], "unavailable")
        self.assertNotIn("matrix_row_major", calibration["transform"])

    def test_camera_reconnect_profile_change_rebuilds_calibration_id(self):
        lidar, _ = load_lidar_camera_calibration(str(FACTORY_CALIBRATION))
        identity = realsense_extrinsics_transform(
            source_frame="depth",
            target_frame="rgb",
            rotation_column_major=[1, 0, 0, 0, 1, 0, 0, 0, 1],
            translation_m=[0, 0, 0],
        )
        rgb_a, depth_a = build_calibrations(
            serial="camera-1",
            rgb_intrinsics=make_intrinsics(1920, 1080, 1000),
            depth_intrinsics=make_intrinsics(),
            depth_to_rgb=identity,
            lidar_to_rgb=lidar,
            depth_scale_m=0.001,
        )
        rgb_b, _ = build_calibrations(
            serial="camera-1",
            rgb_intrinsics=make_intrinsics(1920, 1080, 1001),
            depth_intrinsics=make_intrinsics(),
            depth_to_rgb=identity,
            lidar_to_rgb=lidar,
            depth_scale_m=0.001,
        )
        self.assertEqual(rgb_a["calibration_id"], depth_a["calibration_id"])
        self.assertNotEqual(rgb_a["calibration_id"], rgb_b["calibration_id"])

    def test_unavailable_timestamp_remains_null_in_metadata(self):
        timing = CameraFrameTiming(
            source_stamp_ns=None,
            source_stamp_raw_ns=None,
            source_clock_domain="unknown",
            driver_receive_stamp_ns=1_780_000_000_000_000_000,
            clock_domain="unavailable",
            normalization_status="source_stamp_invalid",
            offset_ns=None,
            out_of_order=False,
        )
        metadata = build_frame_metadata(
            schema=RGB_SCHEMA,
            frame_id="camera_color_optical_frame",
            timing=timing,
            driver_receive_monotonic_ns=123,
            sequence=7,
            image={"encoding": "jpeg", "width": 1, "height": 1},
            calibration={"calibration_id": "test"},
        )
        self.assertIsNone(metadata["header"]["stamp_ns"])
        self.assertFalse(metadata["timing"]["available"])


class DriverCameraContractTest(unittest.TestCase):
    def test_driver_registers_frame_topics_without_replacing_legacy(self):
        source = (G1_DIR / "device.py").read_text(encoding="utf-8")
        self.assertIn('self._color_topic = f"/{namespace}/camera/rgb"', source)
        self.assertIn('self._depth_topic = f"/{namespace}/camera/depth"', source)
        self.assertIn('"name": "camera_rgb_frame"', source)
        self.assertIn('"name": "camera_depth_frame"', source)
        self.assertNotIn('"name": "camera_rgb_v2"', source)
        self.assertNotIn('"name": "camera_depth_v2"', source)
        self.assertIn('"ros_type": "std_msgs/msg/UInt8MultiArray"', source)
        self.assertIn('depth=1', source)

    def test_reconnect_resets_clock_sequence_and_reloads_active_profile(self):
        source = (G1_DIR / "device.py").read_text(encoding="utf-8")
        start_capture = source[source.index("        def start_capture(self):"):]
        self.assertIn("self._configure_profile(profile)", start_capture)
        self.assertIn(
            "self._camera_clock_normalizer = self._new_clock_normalizer()",
            start_capture,
        )
        self.assertNotIn("self._clock = self._new_clock_normalizer()", source)
        self.assertIn('self._sequence = {"rgb": 0, "depth": 0}', start_capture)
        self.assertIn("self.stop_capture(reconnecting=True)", start_capture)

    def test_camera_worker_installs_logsafe_compresses_depth_and_samples_errors(self):
        source = (G1_DIR / "device.py").read_text(encoding="utf-8")
        worker = source[source.index("def run_realsense_process("):]
        self.assertLess(
            worker.index("logsafe.install(check_fd=False)"),
            worker.index("from array import array"),
        )
        self.assertIn("payload = compress_depth_payload(raw_payload)", worker)
        self.assertIn("image=build_depth_image_metadata(", worker)
        self.assertIn("uncompressed_size=len(raw_payload)", worker)
        self.assertIn("if count == 1 or count % 100 == 0:", worker)
        self.assertIn('"frameset_errors": 0', worker)

    def test_container_and_config_include_frame_runtime_files(self):
        config = yaml.safe_load((G1_DIR / "config.yaml").read_text(encoding="utf-8"))
        camera = config["plugins"]["camera"]
        self.assertEqual(camera["rgb_frame_topic"], "/ubuntu/camera/rgb_frame")
        self.assertEqual(camera["depth_frame_topic"], "/ubuntu/camera/depth_frame")
        dockerfile = (G1_DIR / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("COPY camera_frame.py /work/camera_frame.py", dockerfile)
        self.assertIn("COPY calibration/ /work/calibration/", dockerfile)


if __name__ == "__main__":
    unittest.main()
