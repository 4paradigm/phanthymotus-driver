#!/usr/bin/env python3
"""Derive a static-preview LiDAR-to-color transform from public G1 mounts.

This deliberately emits ``nominal_public_urdf`` evidence.  It is suitable for
an isolated, stationary RViz alignment check, but it is not a measured sensor
calibration and must not pass the production RGB-LIVO gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import numpy as np


UNITREE_G1_DESCRIPTION = (
    "https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description"
)
REALSENSE_D435_DESCRIPTION = (
    "https://github.com/realsenseai/realsense-ros/blob/ros2-development/"
    "realsense2_description/urdf/_d435.urdf.xacro"
)

# These three official descriptions carry the same sensor mounts.  Keep the
# allow-list explicit so a new or deprecated machine revision cannot silently
# inherit the Shanghai candidate.
PUBLIC_MOUNTS = {
    4: "g1_23dof_rev_1_0",
    5: "g1_29dof_rev_1_0",
    11: "g1_29dof_mode_11",
}
D435_XYZ = (0.0576235, 0.01753, 0.42987)
D435_RPY = (0.0, 0.8307767239493009, 0.0)
MID360_XYZ = (0.0002835, 0.00003, 0.428434)
MID360_RPY = (math.pi, 0.05112069379091391, 0.0)
BRIDGE_RPY = (math.pi, 0.0, 0.0)
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ]
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ]
    )
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rz @ ry @ rx


def transform(rotation: np.ndarray, translation) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def derive_lidar_to_color(probe: dict[str, Any]) -> tuple[list[float], list[float], str]:
    mode_machine = int(probe["mode_machine"])
    if mode_machine not in PUBLIC_MOUNTS:
        raise ValueError(f"unsupported mode_machine for nominal preview: {mode_machine}")

    d435i = probe["d435i"]
    internal = d435i["depth_to_color_optical"]
    rotation_values = internal["rotation_column_major"]
    translation_values = internal["translation_m"]
    if len(rotation_values) != 9 or len(translation_values) != 3:
        raise ValueError("invalid RealSense depth-to-color extrinsic dimensions")

    # get_extrinsics_to(depth, color) maps depth-optical coordinates to
    # color-optical coordinates.  Its flat rotation array is column-major.
    color_from_depth = transform(
        np.asarray(rotation_values, dtype=float).reshape((3, 3), order="F"),
        translation_values,
    )
    d435_from_depth_optical = transform(rpy_matrix(OPTICAL_RPY), (0.0, 0.0, 0.0))
    d435_from_color_optical = d435_from_depth_optical @ np.linalg.inv(
        color_from_depth
    )

    torso_from_d435 = transform(rpy_matrix(D435_RPY), D435_XYZ)
    torso_from_mid360_raw = transform(rpy_matrix(MID360_RPY), MID360_XYZ)

    # The navigation bridge publishes corrected = Rx(pi) * raw.  Convert the
    # public raw MID360 link into the exact corrected livox_frame consumed by
    # FAST-LIVO2, avoiding a second upside-down rotation.
    raw_from_corrected = transform(rpy_matrix(BRIDGE_RPY).T, (0.0, 0.0, 0.0))
    torso_from_corrected_lidar = torso_from_mid360_raw @ raw_from_corrected
    torso_from_color_optical = torso_from_d435 @ d435_from_color_optical
    color_from_corrected_lidar = (
        np.linalg.inv(torso_from_color_optical) @ torso_from_corrected_lidar
    )

    rotation = color_from_corrected_lidar[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=5e-6):
        raise ValueError("derived LiDAR-to-color rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=5e-6):
        raise ValueError("derived LiDAR-to-color rotation determinant is not +1")

    return (
        rotation.reshape(9).tolist(),
        color_from_corrected_lidar[:3, 3].tolist(),
        PUBLIC_MOUNTS[mode_machine],
    )


def canonical_id(value: dict[str, Any]) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def validate_time_probe(
    time_probe: dict[str, Any], live_probe: dict[str, Any]
) -> dict[str, Any]:
    if time_probe.get("schema_version") != 1 or time_probe.get("probe_type") != (
        "g1_realsense_callback_latency"
    ):
        raise ValueError("unsupported RGB time-offset probe schema/type")
    payload = {key: value for key, value in time_probe.items() if key != "probe_id"}
    if time_probe.get("probe_id") != canonical_id(payload):
        raise ValueError("RGB time-offset probe_id does not match canonical SHA256")
    if str(time_probe.get("boot_id", "")) != str(live_probe.get("boot_id", "")):
        raise ValueError("RGB time-offset probe boot_id does not match the live G1 boot")

    timed_d435i = time_probe.get("d435i")
    measurement = time_probe.get("measurement")
    if not isinstance(timed_d435i, dict) or not isinstance(measurement, dict):
        raise ValueError("RGB time-offset probe is incomplete")
    live_d435i = live_probe["d435i"]
    if str(timed_d435i.get("serial", "")) != str(live_d435i["serial"]):
        raise ValueError("RGB time-offset probe D435I serial does not match live probe")
    timed_color = timed_d435i.get("color")
    if not isinstance(timed_color, dict):
        raise ValueError("RGB time-offset probe color profile is missing")
    live_color = live_d435i["color"]
    timed_profile = tuple(int(timed_color.get(key, 0)) for key in ("width", "height", "fps"))
    live_profile = tuple(int(live_color[key]) for key in ("width", "height", "fps"))
    if timed_profile != live_profile:
        raise ValueError(
            f"RGB time-offset probe profile {timed_profile} does not match live {live_profile}"
        )
    if measurement.get("timestamp_domain") != "global_time":
        raise ValueError("RGB time-offset probe must use RealSense global_time")
    if int(measurement.get("sample_count", 0)) < 30:
        raise ValueError("RGB time-offset probe requires at least 30 samples")

    offset_s = float(measurement.get("recommended_img_time_offset_s"))
    residual_ms = float(measurement.get("p95_abs_residual_ms"))
    if not math.isfinite(offset_s) or abs(offset_s) > 0.5:
        raise ValueError("RGB time-offset recommendation is outside +/-0.5 s")
    if not math.isfinite(residual_ms) or not 0.0 <= residual_ms <= 20.0:
        raise ValueError("RGB callback latency residual p95 exceeds 20 ms")
    latency = measurement.get("callback_latency_ms")
    if not isinstance(latency, dict) or not math.isfinite(float(latency.get("median"))):
        raise ValueError("RGB callback latency median is missing")
    return {
        "probe_id": time_probe["probe_id"],
        "method": str(measurement.get("method", "")),
        "offset_s": offset_s,
        "p95_residual_ms": residual_ms,
        "callback_latency_median_ms": float(latency["median"]),
        "sample_count": int(measurement["sample_count"]),
    }


def build_snapshot(
    probe: dict[str, Any], time_probe: dict[str, Any] | None = None
) -> dict[str, Any]:
    rotation, translation, model = derive_lidar_to_color(probe)
    d435i = probe["d435i"]
    color = d435i["color"]
    global_time = d435i.get("global_time", [])
    if not global_time or not all(item.get("enabled") for item in global_time):
        raise ValueError("RealSense global_time_enabled is not active on every sensor")

    if time_probe is None:
        time_alignment = {
            "status": "preview_only",
            "timestamp_source": "callback_arrival_preview",
            "img_time_offset_s": 0.0,
            "p95_abs_skew_ms": None,
        }
        time_warning = "unverified camera-LiDAR time alignment"
    else:
        correction = validate_time_probe(time_probe, probe)
        time_alignment = {
            "status": "preview_only",
            "timestamp_source": "callback_arrival_preview",
            "img_time_offset_s": correction["offset_s"],
            # This probe bounds callback-latency jitter, not true camera-to-
            # LiDAR skew.  Keep the production skew field unverified.
            "p95_abs_skew_ms": None,
            "preview_time_correction": {
                "method": correction["method"],
                "time_probe_id": correction["probe_id"],
                "sample_count": correction["sample_count"],
                "callback_latency_median_ms": correction[
                    "callback_latency_median_ms"
                ],
                "p95_abs_residual_ms": correction["p95_residual_ms"],
            },
        }
        time_warning = (
            "callback latency corrected from RealSense GLOBAL_TIME, but "
            "camera-LiDAR skew is not independently verified"
        )

    data = {
        "schema_version": 1,
        "device_id": "g1-live-probe",
        "unitree": {
            "mode_machine": int(probe["mode_machine"]),
            "model": model,
        },
        "d435i": {
            "identity": {
                "manufacturer": "Intel RealSense",
                "model": d435i["model"],
                "serial": d435i["serial"],
            },
            "profiles": {"color": color},
            "factory_extrinsics": {
                "depth_to_color_optical": d435i["depth_to_color_optical"]
            },
        },
        "ground_truth": {
            "transforms": {
                "lidar_to_camera": {
                    "status": "nominal_public_urdf",
                    "source_frame": "livox_frame",
                    "target_frame": "camera_color_optical_frame",
                    "rotation_row_major": rotation,
                    "translation_m": translation,
                }
            },
            "time_alignment": {
                "d435i_color_to_mid360": time_alignment
            },
        },
    }
    return {
        "schema_version": 1,
        "calibration_id": canonical_id(data),
        "captured_at_epoch_ns": int(probe["captured_at_epoch_ns"]),
        "data": data,
        "provenance": {
            "evidence_tier": "nominal_public_urdf",
            "unitree_description": UNITREE_G1_DESCRIPTION,
            "realsense_description": REALSENSE_D435_DESCRIPTION,
            "warning": (
                "STATIC RGB PREVIEW ONLY; not a measured robot-specific extrinsic "
                f"and {time_warning}"
            ),
        },
    }


def write_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, prefix=path.name + ".", delete=False
    ) as handle:
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("probe", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--time-offset-probe", type=Path)
    args = parser.parse_args()

    try:
        with args.probe.open(encoding="utf-8") as handle:
            probe = json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        byte_count = args.probe.stat().st_size if args.probe.exists() else -1
        print(
            f"invalid live probe JSON: path={args.probe} bytes={byte_count} error={error}",
            file=sys.stderr,
        )
        return 1
    if not isinstance(probe, dict):
        print("invalid live probe JSON: root must be an object", file=sys.stderr)
        return 1
    time_probe = None
    if args.time_offset_probe is not None:
        try:
            with args.time_offset_probe.open(encoding="utf-8") as handle:
                time_probe = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            print(f"invalid RGB time-offset probe: {error}", file=sys.stderr)
            return 1
        if not isinstance(time_probe, dict):
            print("invalid RGB time-offset probe: root must be an object", file=sys.stderr)
            return 1
    try:
        snapshot = build_snapshot(probe, time_probe)
    except (KeyError, TypeError, ValueError) as error:
        print(f"invalid RGB preview calibration input: {error}", file=sys.stderr)
        return 1
    write_atomic(
        args.output,
        json.dumps(snapshot, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )
    transform_data = snapshot["data"]["ground_truth"]["transforms"][
        "lidar_to_camera"
    ]
    print(
        "rgb_livo_preview_calibration=nominal "
        f"mode_machine={snapshot['data']['unitree']['mode_machine']} "
        f"model={snapshot['data']['unitree']['model']} "
        f"serial={snapshot['data']['d435i']['identity']['serial']} "
        f"translation_m={transform_data['translation_m']}"
    )
    alignment = snapshot["data"]["ground_truth"]["time_alignment"][
        "d435i_color_to_mid360"
    ]
    correction = alignment.get("preview_time_correction")
    if isinstance(correction, dict):
        print(
            "rgb_time_alignment=callback_latency_corrected_preview "
            f"img_time_offset_s={alignment['img_time_offset_s']:.9f} "
            f"p95_residual_ms={correction['p95_abs_residual_ms']:.3f}"
        )
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
