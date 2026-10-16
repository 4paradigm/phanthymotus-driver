"""PSE1 RGB frame envelope for the X2 front camera.

The wire format intentionally matches the self-describing camera-frame contract
introduced by the G1 driver.  This module contains no ROS imports so it can be
tested without robot middleware.
"""

from __future__ import annotations

import hashlib
import json
import struct
import time
from typing import Any


ENVELOPE_MAGIC = b"PSE1"
ENVELOPE_HEADER = struct.Struct("<4sII")
ENVELOPE_FORMAT = "application/vnd.phanthy.sensor-envelope.v1"
RGB_SCHEMA = "phanthy.sensor.camera_rgb_frame.v1"

# Official X2 URDF, x2_{fist,hand}.urdf. This is the chain from the root
# `pelvis` to `rgb_head_center_link` when waist/head joints are all zero.
# The live X2 RGB topic is rgb_head_front_center, which maps to this URDF link.
NOMINAL_PELVIS_FROM_RGB_HEAD_CENTER = (
    (-0.003176840070, -0.000000150255, 0.999994953831, 0.076747710165),
    (-0.999994953831, 0.000000000477, 0.003176840070, -0.000217129491),
    (0.000000000000, -1.000000000000, -0.000000150256, 0.603367713833),
    (0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000),
)


def _canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")


def encode_envelope(metadata: dict, payload: bytes | bytearray | memoryview) -> bytes:
    metadata_bytes = _canonical_json(metadata)
    image_bytes = bytes(payload)
    return ENVELOPE_HEADER.pack(ENVELOPE_MAGIC, len(metadata_bytes), len(image_bytes)) + metadata_bytes + image_bytes


def decode_envelope(data: bytes | bytearray | memoryview) -> tuple[dict, bytes]:
    raw = bytes(data)
    if len(raw) < ENVELOPE_HEADER.size:
        raise ValueError("camera envelope is shorter than its fixed header")
    magic, metadata_size, payload_size = ENVELOPE_HEADER.unpack_from(raw)
    if magic != ENVELOPE_MAGIC:
        raise ValueError(f"unexpected camera envelope magic: {magic!r}")
    metadata_end = ENVELOPE_HEADER.size + metadata_size
    if len(raw) != metadata_end + payload_size:
        raise ValueError("camera envelope length mismatch")
    metadata = json.loads(raw[ENVELOPE_HEADER.size:metadata_end].decode("utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("camera envelope metadata must be an object")
    return metadata, raw[metadata_end:]


def stamp_ns(header: Any) -> int | None:
    stamp = getattr(header, "stamp", None)
    try:
        seconds = int(getattr(stamp, "sec"))
        nanoseconds = int(getattr(stamp, "nanosec"))
    except (AttributeError, TypeError, ValueError):
        return None
    value = seconds * 1_000_000_000 + nanoseconds
    return value if value > 0 else None


def calibration_from_camera_info(camera_info: Any, frame_id: str) -> dict:
    """Build RGB calibration from X2 CameraInfo and the official nominal URDF chain."""
    k = [float(value) for value in getattr(camera_info, "k", [])]
    r = [float(value) for value in getattr(camera_info, "r", [])]
    p = [float(value) for value in getattr(camera_info, "p", [])]
    d = [float(value) for value in getattr(camera_info, "d", [])]
    width, height = int(getattr(camera_info, "width", 0)), int(getattr(camera_info, "height", 0))
    if width <= 0 or height <= 0 or len(k) != 9 or len(r) != 9 or len(p) != 12:
        raise ValueError("CameraInfo is incomplete")
    intrinsics = {
        "width": width,
        "height": height,
        "distortion_model": str(getattr(camera_info, "distortion_model", "")),
        "d": d,
        "k": k,
        "r": r,
        "p": p,
    }
    extrinsic = {
        "status": "nominal_zero_joint_pose",
        "source": "AimDK X2 URDF x2_{fist,hand}.urdf",
        "source_frame": "rgb_head_center_link",
        "target_frame": "pelvis",
        "convention": "target_from_source",
        "matrix_4x4_row_major": [value for row in NOMINAL_PELVIS_FROM_RGB_HEAD_CENTER for value in row],
        "joint_positions_rad": {
            "waist_yaw_joint": 0.0,
            "waist_pitch_joint": 0.0,
            "waist_roll_joint": 0.0,
            "head_yaw_joint": 0.0,
            "head_pitch_joint": 0.0,
        },
        "dynamic_joint_chain": [
            "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
            "head_yaw_joint", "head_pitch_joint",
        ],
        "validity": "nominal-only; recompute from live joint states before spatial fusion when waist or head moves",
    }
    calibration_id = "sha256:" + hashlib.sha256(_canonical_json({
        "frame_id": frame_id, **intrinsics, "base_to_camera": extrinsic,
    })).hexdigest()
    return {
        "calibration_id": calibration_id,
        "camera_serial": "unavailable",
        **intrinsics,
        "intrinsics_source": "x2_ros_camera_info",
        "base_to_camera": extrinsic,
    }


def build_rgb_metadata(message: Any, calibration: dict, sequence: int) -> dict:
    header = getattr(message, "header", None)
    frame_id = str(getattr(header, "frame_id", "")) or "rgb_head_center_link"
    source_ns = stamp_ns(header)
    receive_ns = time.time_ns()
    image = bytes(getattr(message, "data", b""))
    return {
        "schema": RGB_SCHEMA,
        "header": {"stamp_ns": source_ns, "frame_id": frame_id},
        "timing": {
            "source_stamp_ns": source_ns,
            "source_stamp_raw_ns": source_ns,
            "source_clock_domain": "ros_system_time",
            "driver_receive_stamp_ns": receive_ns,
            "clock_domain": "ros_system_time" if source_ns is not None else "unavailable",
            "normalization_status": "source_system_time" if source_ns is not None else "source_stamp_invalid",
            "offset_ns": 0 if source_ns is not None else None,
            "out_of_order": False,
            "available": source_ns is not None,
        },
        "sequence": int(sequence),
        "image": {
            "encoding": "jpeg",
            "width": int(calibration["width"]),
            "height": int(calibration["height"]),
            "payload_size": len(image),
        },
        "calibration": calibration,
    }
