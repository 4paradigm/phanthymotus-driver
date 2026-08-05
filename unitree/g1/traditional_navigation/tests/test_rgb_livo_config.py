import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


NAV_DIR = Path(__file__).resolve().parents[1]
RENDERER = NAV_DIR / "render-g1-livo-config.py"
DERIVER = NAV_DIR / "derive-g1-rgb-preview-calibration.py"
TIME_PROBE = NAV_DIR / "probe-g1-rgb-time-offset.py"


def callback_time_probe(boot_id: str = "test-boot-id") -> dict:
    data = {
        "schema_version": 1,
        "probe_type": "g1_realsense_callback_latency",
        "captured_at_epoch_ns": 1,
        "boot_id": boot_id,
        "d435i": {
            "serial": "346522072810",
            "color": {
                "width": 1920,
                "height": 1080,
                "fps": 15,
                "format": "bgr8",
            },
        },
        "measurement": {
            "method": "realsense_global_time_to_host_delivery_proxy",
            "timestamp_domain": "global_time",
            "sample_count": 120,
            "callback_latency_ms": {
                "min": 28.0,
                "median": 32.5,
                "p95": 35.0,
                "max": 38.0,
            },
            "recommended_img_time_offset_s": -0.0325,
            "p95_abs_residual_ms": 4.5,
            "exposure_us": {"median": 8000.0, "p95": 9000.0},
        },
    }
    encoded = json.dumps(
        data,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return {
        **data,
        "probe_id": "sha256:" + hashlib.sha256(encoded).hexdigest(),
    }


def calibration_snapshot() -> dict:
    data = {
        "schema_version": 1,
        "device_id": "sh-g1",
        "d435i": {
            "identity": {
                "manufacturer": "Intel RealSense",
                "model": "Intel RealSense D435I",
                "serial": "346522072810",
            },
            "profiles": {
                "color": {
                    "width": 1920,
                    "height": 1080,
                    "fps": 15,
                    "format": "bgr8",
                    "intrinsics": {
                        "fx": 1368.0,
                        "fy": 1372.0,
                        "ppx": 981.0,
                        "ppy": 552.0,
                        "distortion_model": "inverse_brown_conrady",
                        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                    },
                }
            },
        },
        "ground_truth": {
            "transforms": {
                "lidar_to_camera": {
                    "status": "calibrated",
                    "rotation_row_major": [
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    ],
                    "translation_m": [0.1, 0.0, 0.0],
                }
            },
            "time_alignment": {
                "d435i_color_to_mid360": {
                    "status": "verified",
                    "timestamp_source": "realsense_hardware_clock",
                    "img_time_offset_s": 0.012,
                    "p95_abs_skew_ms": 14.0,
                }
            },
        },
    }
    encoded = json.dumps(
        data,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return {
        "schema_version": 1,
        "calibration_id": "sha256:" + hashlib.sha256(encoded).hexdigest(),
        "captured_at_epoch_ns": 1,
        "data": data,
        "provenance": {},
    }


class RenderG1LivoConfigTests(unittest.TestCase):
    def run_renderer(self, snapshot: dict, output: Path) -> subprocess.CompletedProcess[str]:
        calibration = output.with_suffix(".json")
        calibration.write_text(json.dumps(snapshot), encoding="utf-8")
        return subprocess.run(
            [sys.executable, str(RENDERER), str(calibration), str(output)],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_deriver_rejects_empty_probe_without_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            root_path = Path(root)
            probe = root_path / "empty-probe.json"
            output = root_path / "preview.json"
            probe.write_text("", encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(DERIVER), str(probe), str(output)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIn("invalid live probe JSON", result.stderr)
            self.assertIn("bytes=0", result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertFalse(output.exists())

    def test_callback_time_probe_cli_validates_hash_and_jitter(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            probe_path = Path(root) / "time-probe.json"
            probe_path.write_text(json.dumps(callback_time_probe()), encoding="utf-8")
            valid = subprocess.run(
                [sys.executable, str(TIME_PROBE), "--validate", str(probe_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(valid.returncode, 0, valid.stdout + valid.stderr)
            self.assertIn("offset_s=-0.032500000", valid.stdout)
            self.assertIn("p95_residual_ms=4.500", valid.stdout)

            tampered = callback_time_probe()
            tampered["measurement"]["p95_abs_residual_ms"] = 21.0
            probe_path.write_text(json.dumps(tampered), encoding="utf-8")
            rejected = subprocess.run(
                [sys.executable, str(TIME_PROBE), "--validate", str(probe_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(rejected.returncode, 1)
            self.assertIn("canonical SHA256", rejected.stderr)

            unstable = callback_time_probe()
            unstable["measurement"]["p95_abs_residual_ms"] = 21.0
            payload = {key: value for key, value in unstable.items() if key != "probe_id"}
            encoded = json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
            unstable["probe_id"] = "sha256:" + hashlib.sha256(encoded).hexdigest()
            probe_path.write_text(json.dumps(unstable), encoding="utf-8")
            jitter_rejected = subprocess.run(
                [sys.executable, str(TIME_PROBE), "--validate", str(probe_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(jitter_rejected.returncode, 1)
            self.assertIn("residual p95 exceeds 20 ms", jitter_rejected.stderr)

    def test_renders_only_complete_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "g1_livo.yaml"
            result = self.run_renderer(calibration_snapshot(), output)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            rendered = output.read_text(encoding="utf-8")
            self.assertIn("img_en: 1", rendered)
            self.assertIn("enable_image_processing: true", rendered)
            self.assertIn("width: 1920", rendered)
            self.assertIn(
                "Rcl: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
                rendered,
            )
            self.assertIn("Pcl: [0.1, 0.0, 0.0]", rendered)
            self.assertIn("img_time_offset: 0.012", rendered)
            self.assertIn("d0: 0.0", rendered)
            self.assertIn("filter_size_pcd: 0.02", rendered)

    def test_rejects_unknown_lidar_camera_extrinsic(self) -> None:
        snapshot = calibration_snapshot()
        transform = snapshot["data"]["ground_truth"]["transforms"]["lidar_to_camera"]
        transform["status"] = "unknown_requires_target_calibration"
        transform["rotation_row_major"] = None
        transform["translation_m"] = None
        encoded = json.dumps(
            snapshot["data"],
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        snapshot["calibration_id"] = "sha256:" + hashlib.sha256(encoded).hexdigest()
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "g1_livo.yaml"
            result = self.run_renderer(snapshot, output)
            self.assertEqual(result.returncode, 1)
            self.assertIn("LiDAR→D435i 外参状态", result.stdout)
            self.assertFalse(output.exists())

    def test_rejects_runtime_profile_mismatch(self) -> None:
        snapshot = calibration_snapshot()
        snapshot["data"]["d435i"]["profiles"]["color"].update(
            {"width": 1280, "height": 720, "fps": 30}
        )
        encoded = json.dumps(
            snapshot["data"],
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        snapshot["calibration_id"] = "sha256:" + hashlib.sha256(encoded).hexdigest()
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "g1_livo.yaml"
            result = self.run_renderer(snapshot, output)
            self.assertEqual(result.returncode, 1)
            self.assertIn("与运行 profile 1920x1080@15 不一致", result.stdout)
            self.assertFalse(output.exists())

    def test_rejects_tampered_calibration_id(self) -> None:
        snapshot = calibration_snapshot()
        snapshot["calibration_id"] = "sha256:" + "0" * 64
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "g1_livo.yaml"
            result = self.run_renderer(snapshot, output)
            self.assertEqual(result.returncode, 1)
            self.assertIn("calibration_id", result.stdout)
            self.assertFalse(output.exists())

    def test_nominal_mode4_preview_requires_explicit_flag(self) -> None:
        probe = {
            "schema_version": 1,
            "captured_at_epoch_ns": 1,
            "boot_id": "test-boot-id",
            "mode_machine": 4,
            "mode_pr": 0,
            "network_interface": "eth0",
            "d435i": {
                "serial": "346522072810",
                "model": "Intel RealSense D435I",
                "color": {
                    "width": 1920,
                    "height": 1080,
                    "fps": 15,
                    "format": "bgr8",
                    "intrinsics": {
                        "fx": 1368.246826171875,
                        "fy": 1372.2265625,
                        "ppx": 981.5875854492188,
                        "ppy": 552.4080810546875,
                        "distortion_model": "inverse_brown_conrady",
                        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                    },
                },
                "depth_to_color_optical": {
                    "rotation_column_major": [
                        0.9999568462371826,
                        -0.008939534425735474,
                        0.002532962244004011,
                        0.008926372043788433,
                        0.9999468326568604,
                        0.005160734057426453,
                        -0.002578962128609419,
                        -0.005137901287525892,
                        0.9999834895133972,
                    ],
                    "translation_m": [
                        0.015179511159658432,
                        0.0015000001294538379,
                        0.001500000013038516,
                    ],
                },
                "global_time": [
                    {"sensor": "Stereo Module", "enabled": True},
                    {"sensor": "RGB Camera", "enabled": True},
                    {"sensor": "Motion Module", "enabled": True},
                ],
            },
        }
        with tempfile.TemporaryDirectory() as root:
            root_path = Path(root)
            probe_path = root_path / "probe.json"
            snapshot_path = root_path / "preview.json"
            output = root_path / "g1_livo.preview.yaml"
            probe_path.write_text(json.dumps(probe), encoding="utf-8")
            derived = subprocess.run(
                [sys.executable, str(DERIVER), str(probe_path), str(snapshot_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(derived.returncode, 0, derived.stdout + derived.stderr)
            snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
            transform = snapshot["data"]["ground_truth"]["transforms"][
                "lidar_to_camera"
            ]
            self.assertEqual(snapshot["data"]["unitree"]["model"], "g1_23dof_rev_1_0")
            self.assertEqual(transform["status"], "nominal_public_urdf")
            expected_rotation = [
                -0.008109593435824423,
                -0.9999568462371825,
                -0.004534937467106068,
                -0.7066513125489848,
                0.008939534425735472,
                -0.7075054689844624,
                0.7075154854731978,
                -0.0025329622440039496,
                -0.7066933212441079,
            ]
            expected_translation = [
                0.033162348481982296,
                0.044845789024666904,
                -0.03583561107829775,
            ]
            for actual, expected in zip(
                transform["rotation_row_major"], expected_rotation
            ):
                self.assertAlmostEqual(actual, expected, places=9)
            for actual, expected in zip(transform["translation_m"], expected_translation):
                self.assertAlmostEqual(actual, expected, places=9)

            denied = subprocess.run(
                [sys.executable, str(RENDERER), str(snapshot_path), str(output)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(denied.returncode, 1)
            self.assertFalse(output.exists())

            allowed = subprocess.run(
                [
                    sys.executable,
                    str(RENDERER),
                    str(snapshot_path),
                    str(output),
                    "--allow-nominal-preview",
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(allowed.returncode, 0, allowed.stdout + allowed.stderr)
            rendered = output.read_text(encoding="utf-8")
            self.assertIn("# evidence_tier: nominal_public_urdf", rendered)
            self.assertIn("# preview_only: true", rendered)
            self.assertIn("p95_skew_ms: unverified", rendered)
            self.assertIn("img_time_offset: 0.0", rendered)
            self.assertIn("Rcl: [-0.00810959343582", rendered)

            time_probe_path = root_path / "time-probe.json"
            timed_snapshot_path = root_path / "timed-preview.json"
            timed_output = root_path / "g1_livo.timed-preview.yaml"
            time_probe_path.write_text(
                json.dumps(callback_time_probe()), encoding="utf-8"
            )
            timed_derived = subprocess.run(
                [
                    sys.executable,
                    str(DERIVER),
                    str(probe_path),
                    str(timed_snapshot_path),
                    "--time-offset-probe",
                    str(time_probe_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                timed_derived.returncode,
                0,
                timed_derived.stdout + timed_derived.stderr,
            )
            self.assertIn("img_time_offset_s=-0.032500000", timed_derived.stdout)
            timed_snapshot = json.loads(timed_snapshot_path.read_text(encoding="utf-8"))
            timed_alignment = timed_snapshot["data"]["ground_truth"][
                "time_alignment"
            ]["d435i_color_to_mid360"]
            self.assertEqual(timed_alignment["img_time_offset_s"], -0.0325)
            self.assertIsNone(timed_alignment["p95_abs_skew_ms"])
            self.assertEqual(
                timed_alignment["preview_time_correction"]["p95_abs_residual_ms"],
                4.5,
            )
            timed_render = subprocess.run(
                [
                    sys.executable,
                    str(RENDERER),
                    str(timed_snapshot_path),
                    str(timed_output),
                    "--allow-nominal-preview",
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                timed_render.returncode,
                0,
                timed_render.stdout + timed_render.stderr,
            )
            self.assertIn(
                "img_time_offset: -0.0325",
                timed_output.read_text(encoding="utf-8"),
            )

    def test_nominal_preview_rejects_stale_time_probe_boot(self) -> None:
        live_probe = {
            "schema_version": 1,
            "captured_at_epoch_ns": 1,
            "boot_id": "current-boot",
            "mode_machine": 4,
            "d435i": {
                "serial": "346522072810",
                "model": "Intel RealSense D435I",
                "color": {"width": 1920, "height": 1080, "fps": 15},
                "depth_to_color_optical": {
                    "rotation_column_major": [
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    ],
                    "translation_m": [0.01, 0.0, 0.0],
                },
                "global_time": [{"sensor": "RGB Camera", "enabled": True}],
            },
        }
        with tempfile.TemporaryDirectory() as root:
            root_path = Path(root)
            live_path = root_path / "live.json"
            time_path = root_path / "time.json"
            output = root_path / "preview.json"
            live_path.write_text(json.dumps(live_probe), encoding="utf-8")
            time_path.write_text(json.dumps(callback_time_probe()), encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(DERIVER),
                    str(live_path),
                    str(output),
                    "--time-offset-probe",
                    str(time_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIn("boot_id", result.stderr)
            self.assertFalse(output.exists())

    def test_nominal_preview_rejects_unmatched_machine_revision(self) -> None:
        probe = {
            "mode_machine": 2,
            "d435i": {
                "depth_to_color_optical": {
                    "rotation_column_major": [1.0] * 9,
                    "translation_m": [0.0, 0.0, 0.0],
                }
            },
        }
        with tempfile.TemporaryDirectory() as root:
            probe_path = Path(root) / "probe.json"
            output = Path(root) / "preview.json"
            probe_path.write_text(json.dumps(probe), encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(DERIVER), str(probe_path), str(output)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("unsupported mode_machine", result.stderr)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
