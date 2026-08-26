"""Self-describing RealSense RGB/depth frame and calibration helpers.

This module has no ROS or RealSense imports so the public wire contract,
timestamp policy, and calibration validation can be tested off-robot.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
import threading
from typing import Any
import zlib

import yaml

from navigation_time import ClockOffsetEstimator


ENVELOPE_MAGIC = b"PSE1"
ENVELOPE_HEADER = struct.Struct("<4sII")
ENVELOPE_FORMAT = "application/vnd.phanthy.sensor-envelope.v1"
RGB_SCHEMA = "phanthy.sensor.camera_rgb_frame.v1"
DEPTH_SCHEMA = "phanthy.sensor.camera_depth_frame.v1"
DEPTH_COMPRESSION = "zlib"
DEPTH_COMPRESSION_LEVEL = 1

_DIRECT_CLOCK_DOMAINS = ("system_time", "global_time")
_MAX_DIRECT_CLOCK_ERROR_NS = 24 * 60 * 60 * 1_000_000_000


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def encode_envelope(metadata: dict, payload: bytes | bytearray | memoryview) -> bytes:
    metadata_bytes = canonical_json_bytes(metadata)
    payload_bytes = bytes(payload)
    return (
        ENVELOPE_HEADER.pack(
            ENVELOPE_MAGIC,
            len(metadata_bytes),
            len(payload_bytes),
        )
        + metadata_bytes
        + payload_bytes
    )


def decode_envelope(data: bytes | bytearray | memoryview) -> tuple[dict, bytes]:
    raw = bytes(data)
    if len(raw) < ENVELOPE_HEADER.size:
        raise ValueError("camera envelope is shorter than its fixed header")
    magic, metadata_size, payload_size = ENVELOPE_HEADER.unpack_from(raw)
    if magic != ENVELOPE_MAGIC:
        raise ValueError(f"unexpected camera envelope magic: {magic!r}")
    expected_size = ENVELOPE_HEADER.size + metadata_size + payload_size
    if len(raw) != expected_size:
        raise ValueError(
            f"camera envelope length mismatch: expected {expected_size}, got {len(raw)}"
        )
    metadata_end = ENVELOPE_HEADER.size + metadata_size
    metadata = json.loads(raw[ENVELOPE_HEADER.size:metadata_end].decode("utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("camera envelope metadata must be a JSON object")
    return metadata, raw[metadata_end:]


def compress_depth_payload(payload: bytes | bytearray | memoryview) -> bytes:
    return zlib.compress(bytes(payload), level=DEPTH_COMPRESSION_LEVEL)


def build_depth_image_metadata(
    *,
    width: int,
    height: int,
    uncompressed_size: int,
    payload_size: int,
    depth_scale_m: float,
) -> dict:
    scale = float(depth_scale_m)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("depth_scale_m must be positive and finite")
    return {
        "encoding": "z16_le",
        "width": int(width),
        "height": int(height),
        "step_bytes": int(width) * 2,
        "compression": {
            "codec": DEPTH_COMPRESSION,
            "level": DEPTH_COMPRESSION_LEVEL,
        },
        "uncompressed_size": int(uncompressed_size),
        "payload_size": int(payload_size),
        "unit": "realsense_depth_unit",
        "depth_scale_m": scale,
        "depth_scale_semantics": "meters_per_realsense_depth_unit",
        "aligned_to_rgb": False,
    }


@dataclass(frozen=True)
class CameraFrameTiming:
    source_stamp_ns: int | None
    source_stamp_raw_ns: int | None
    source_clock_domain: str
    driver_receive_stamp_ns: int
    clock_domain: str
    normalization_status: str
    offset_ns: int | None
    out_of_order: bool

    @property
    def available(self) -> bool:
        return self.source_stamp_ns is not None

    def to_dict(self) -> dict:
        result = asdict(self)
        result["available"] = self.available
        return result


class RealSenseClockNormalizer:
    """Map RealSense source timestamps into ROS system time without fabrication.

    Hardware-clock timestamps are normalized with the same minimum-arrival
    offset model used by navigation sensors. Invalid, warming-up, reset, and
    out-of-order observations remain explicitly unavailable; receive time is
    never substituted for the acquisition timestamp.
    """

    def __init__(
        self,
        *,
        warmup_samples: int = 8,
        window_samples: int = 300,
        reset_threshold_ns: int = 1_000_000_000,
        reset_confirm_samples: int = 5,
    ) -> None:
        self._hardware_clock = ClockOffsetEstimator(
            warmup_samples=warmup_samples,
            window_samples=window_samples,
            reset_threshold_ns=reset_threshold_ns,
            reset_confirm_samples=reset_confirm_samples,
        )
        self._last_source_by_stream: dict[str, int] = {}
        self._lock = threading.Lock()

    def normalize(
        self,
        *,
        source_timestamp_ms: float | int | None,
        source_domain: str,
        driver_receive_stamp_ns: int,
        stream: str,
    ) -> CameraFrameTiming:
        receive_ns = int(driver_receive_stamp_ns)
        if receive_ns <= 0:
            raise ValueError("driver_receive_stamp_ns must be positive")
        domain = _normalize_clock_domain(source_domain)
        source_raw_ns = _source_ms_to_ns(source_timestamp_ms)
        if source_raw_ns is None:
            return self._unavailable(
                domain=domain,
                receive_ns=receive_ns,
                raw_ns=None,
                status="source_stamp_invalid",
            )

        with self._lock:
            previous = self._last_source_by_stream.get(stream)
            if previous is not None and source_raw_ns <= previous:
                return self._unavailable(
                    domain=domain,
                    receive_ns=receive_ns,
                    raw_ns=source_raw_ns,
                    status="source_stamp_out_of_order",
                    out_of_order=True,
                )
            self._last_source_by_stream[stream] = source_raw_ns

        if any(name in domain for name in _DIRECT_CLOCK_DOMAINS):
            if abs(receive_ns - source_raw_ns) > _MAX_DIRECT_CLOCK_ERROR_NS:
                return self._unavailable(
                    domain=domain,
                    receive_ns=receive_ns,
                    raw_ns=source_raw_ns,
                    status="source_clock_out_of_range",
                )
            return CameraFrameTiming(
                source_stamp_ns=source_raw_ns,
                source_stamp_raw_ns=source_raw_ns,
                source_clock_domain=domain,
                driver_receive_stamp_ns=receive_ns,
                clock_domain="ros_system_time",
                normalization_status="source_system_time",
                offset_ns=0,
                out_of_order=False,
            )

        normalized_ns = self._hardware_clock.correct_observation(
            source_raw_ns,
            receive_ns,
        )
        snapshot = self._hardware_clock.snapshot()
        if normalized_ns is None:
            if snapshot.pending_reset_samples:
                status = "source_clock_reset_pending"
            elif not snapshot.ready:
                status = "source_clock_warmup"
            else:
                status = "source_clock_observation_rejected"
            return self._unavailable(
                domain=domain,
                receive_ns=receive_ns,
                raw_ns=source_raw_ns,
                status=status,
                offset_ns=snapshot.offset_ns,
            )
        return CameraFrameTiming(
            source_stamp_ns=int(normalized_ns),
            source_stamp_raw_ns=source_raw_ns,
            source_clock_domain=domain,
            driver_receive_stamp_ns=receive_ns,
            clock_domain="ros_system_time",
            normalization_status="hardware_clock_normalized",
            offset_ns=snapshot.offset_ns,
            out_of_order=False,
        )

    @staticmethod
    def _unavailable(
        *,
        domain: str,
        receive_ns: int,
        raw_ns: int | None,
        status: str,
        offset_ns: int | None = None,
        out_of_order: bool = False,
    ) -> CameraFrameTiming:
        return CameraFrameTiming(
            source_stamp_ns=None,
            source_stamp_raw_ns=raw_ns,
            source_clock_domain=domain,
            driver_receive_stamp_ns=receive_ns,
            clock_domain="unavailable",
            normalization_status=status,
            offset_ns=offset_ns,
            out_of_order=out_of_order,
        )


def _source_ms_to_ns(value: float | int | None) -> int | None:
    try:
        source_ms = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(source_ms) or source_ms <= 0:
        return None
    return int(round(source_ms * 1_000_000))


def _normalize_clock_domain(value: str) -> str:
    domain = str(value or "unknown").strip().lower()
    return domain.removeprefix("timestamp_domain.") or "unknown"


def build_intrinsics(
    *,
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    coefficients: list[float] | tuple[float, ...],
    realsense_model: str,
) -> dict:
    values = [float(fx), float(fy), float(cx), float(cy)]
    if int(width) <= 0 or int(height) <= 0 or not all(
        math.isfinite(value) and value > 0 for value in values
    ):
        raise ValueError("camera intrinsics require positive finite dimensions/focal data")
    model = str(realsense_model).lower().removeprefix("distortion.")
    distortion_model = (
        "plumb_bob"
        if model in {"brown_conrady", "modified_brown_conrady"}
        else f"realsense_{model}"
    )
    d = [float(value) for value in coefficients]
    if not all(math.isfinite(value) for value in d):
        raise ValueError("camera distortion coefficients must be finite")
    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": distortion_model,
        "realsense_distortion_model": model,
        "k": [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0],
        "d": d,
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [
            float(fx), 0.0, float(cx), 0.0,
            0.0, float(fy), float(cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
        "r_source": "identity_unrectified_stream",
        "p_source": "derived_from_k_zero_baseline",
    }


def realsense_extrinsics_transform(
    *,
    source_frame: str,
    target_frame: str,
    rotation_column_major: list[float] | tuple[float, ...],
    translation_m: list[float] | tuple[float, ...],
) -> dict:
    rotation = [float(value) for value in rotation_column_major]
    translation = [float(value) for value in translation_m]
    if len(rotation) != 9 or len(translation) != 3:
        raise ValueError("RealSense extrinsics require 9 rotation and 3 translation values")
    if not all(math.isfinite(value) for value in rotation + translation):
        raise ValueError("RealSense extrinsics must be finite")
    row_major = [rotation[row + 3 * column] for row in range(3) for column in range(3)]
    matrix = [
        row_major[0], row_major[1], row_major[2], translation[0],
        row_major[3], row_major[4], row_major[5], translation[1],
        row_major[6], row_major[7], row_major[8], translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]
    return {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "convention": "target_from_source",
        "translation_m": translation,
        "rotation_matrix_row_major": row_major,
        "matrix_row_major": matrix,
        "source": "realsense_factory_calibration",
    }


def load_lidar_camera_calibration(path_value: str | None) -> tuple[dict, str | None]:
    source_frame = "livox_frame"
    target_frame = "camera_color_optical_frame"
    if not path_value:
        return _unavailable_extrinsic(source_frame, target_frame, "configuration_missing"), "configuration_missing"
    path = Path(path_value)
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("calibration root must be an object")
        if value.get("schema") != "phanthy.calibration.lidar_camera.v1":
            raise ValueError("unsupported lidar-camera calibration schema")
        transform = value.get("transform")
        if not isinstance(transform, dict):
            raise ValueError("calibration transform must be an object")
        _validate_transform(transform)
        status = value.get("calibration_status")
        if status not in {"factory_nominal", "validated_on_device"}:
            raise ValueError("calibration_status must be factory_nominal or validated_on_device")
        return {
            "status": status,
            "calibration_version": str(value.get("calibration_version", "")),
            "transform": transform,
            "provenance": value.get("provenance", {}),
            "assumptions": value.get("assumptions", []),
        }, None
    except Exception as exc:
        reason = f"invalid_configuration:{exc}"
        return _unavailable_extrinsic(source_frame, target_frame, reason), reason


def _validate_transform(transform: dict) -> None:
    source_frame = transform.get("source_frame")
    target_frame = transform.get("target_frame")
    if not isinstance(source_frame, str) or not source_frame:
        raise ValueError("transform source_frame is required")
    if not isinstance(target_frame, str) or not target_frame:
        raise ValueError("transform target_frame is required")
    if transform.get("convention") != "target_from_source":
        raise ValueError("transform convention must be target_from_source")
    matrix = [float(value) for value in transform.get("matrix_row_major", [])]
    rotation = [float(value) for value in transform.get("rotation_matrix_row_major", [])]
    translation = [float(value) for value in transform.get("translation_m", [])]
    if len(matrix) != 16 or len(rotation) != 9 or len(translation) != 3:
        raise ValueError("transform requires 4x4 matrix, 3x3 rotation, and xyz translation")
    if not all(math.isfinite(value) for value in matrix + rotation + translation):
        raise ValueError("transform values must be finite")
    if any(abs(a - b) > 1e-9 for a, b in zip(matrix[12:], [0.0, 0.0, 0.0, 1.0])):
        raise ValueError("transform homogeneous bottom row is invalid")
    matrix_rotation = [matrix[index] for index in (0, 1, 2, 4, 5, 6, 8, 9, 10)]
    if any(abs(a - b) > 1e-9 for a, b in zip(matrix_rotation, rotation)):
        raise ValueError("transform rotation fields disagree")
    if any(abs(a - b) > 1e-9 for a, b in zip([matrix[3], matrix[7], matrix[11]], translation)):
        raise ValueError("transform translation fields disagree")
    rows = [rotation[0:3], rotation[3:6], rotation[6:9]]
    for row_index, row in enumerate(rows):
        for other_index, other in enumerate(rows):
            dot = sum(a * b for a, b in zip(row, other))
            expected = 1.0 if row_index == other_index else 0.0
            if abs(dot - expected) > 1e-6:
                raise ValueError("transform rotation is not orthonormal")
    determinant = (
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6])
    )
    if abs(determinant - 1.0) > 1e-6:
        raise ValueError("transform rotation determinant must be +1")


def _unavailable_extrinsic(source_frame: str, target_frame: str, reason: str) -> dict:
    return {
        "status": "unavailable",
        "reason": reason,
        "transform": {
            "source_frame": source_frame,
            "target_frame": target_frame,
            "convention": "target_from_source",
        },
    }


def build_calibrations(
    *,
    serial: str,
    rgb_intrinsics: dict,
    depth_intrinsics: dict,
    depth_to_rgb: dict,
    lidar_to_rgb: dict,
    depth_scale_m: float,
) -> tuple[dict, dict]:
    scale = float(depth_scale_m)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("depth_scale_m must be positive and finite")
    seed = {
        "camera_serial": str(serial),
        "rgb": rgb_intrinsics,
        "depth": depth_intrinsics,
        "depth_to_rgb": depth_to_rgb,
        "lidar_to_rgb": lidar_to_rgb,
        "depth_scale_m": scale,
    }
    calibration_id = "sha256:" + hashlib.sha256(canonical_json_bytes(seed)).hexdigest()
    rgb = {
        "calibration_id": calibration_id,
        "camera_serial": str(serial),
        **rgb_intrinsics,
        "lidar_to_camera": lidar_to_rgb,
        "intrinsics_source": "realsense_active_profile",
    }
    depth = {
        "calibration_id": calibration_id,
        "camera_serial": str(serial),
        **depth_intrinsics,
        "depth_scale_m": scale,
        "aligned_to_rgb": False,
        "depth_to_rgb": depth_to_rgb,
        "rgb_intrinsics": rgb_intrinsics,
        "lidar_to_camera": lidar_to_rgb,
        "intrinsics_source": "realsense_active_profile",
    }
    return rgb, depth


def build_frame_metadata(
    *,
    schema: str,
    frame_id: str,
    timing: CameraFrameTiming,
    driver_receive_monotonic_ns: int,
    sequence: int,
    image: dict,
    calibration: dict,
) -> dict:
    if schema not in {RGB_SCHEMA, DEPTH_SCHEMA}:
        raise ValueError(f"unsupported camera schema: {schema}")
    return {
        "schema": schema,
        "header": {
            "stamp_ns": timing.source_stamp_ns,
            "frame_id": frame_id,
        },
        "timing": {
            **timing.to_dict(),
            "driver_receive_monotonic_ns": int(driver_receive_monotonic_ns),
        },
        "sequence": int(sequence),
        "image": image,
        "calibration": calibration,
    }
