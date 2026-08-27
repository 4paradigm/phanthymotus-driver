"""MID360 PointCloud2 conversion for the generic navigation sensor contract."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# sensor_msgs/msg/PointField datatype constants
UINT8 = 2
UINT16 = 4
FLOAT32 = 7
FLOAT64 = 8


@dataclass(frozen=True)
class PointFieldSpec:
    name: str
    offset: int
    datatype: int
    count: int = 1


# Stable navigation layout: PCL_ADD_POINT4D, intensity, tag, line, alignment
# padding and an absolute per-point timestamp in the normalized ROS clock.
NAVIGATION_FIELDS = (
    PointFieldSpec("x", 0, FLOAT32),
    PointFieldSpec("y", 4, FLOAT32),
    PointFieldSpec("z", 8, FLOAT32),
    PointFieldSpec("intensity", 16, FLOAT32),
    PointFieldSpec("tag", 20, UINT8),
    PointFieldSpec("line", 21, UINT8),
    PointFieldSpec("timestamp", 24, FLOAT64),
)
NAVIGATION_POINT_STEP = 32
_NAVIGATION_DTYPE = np.dtype(
    {
        "names": [field.name for field in NAVIGATION_FIELDS],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [field.offset for field in NAVIGATION_FIELDS],
        "itemsize": NAVIGATION_POINT_STEP,
    }
)

_EXPECTED_INPUT_TYPES = {
    "x": FLOAT32,
    "y": FLOAT32,
    "z": FLOAT32,
    "intensity": FLOAT32,
    "ring": UINT16,
    "time": FLOAT32,
}
_INPUT_DTYPES = {
    "x": "<f4",
    "y": "<f4",
    "z": "<f4",
    "intensity": "<f4",
    "ring": "<u2",
    "time": "<f4",
}


def validated_rotation_matrix(values: object | None) -> np.ndarray:
    """Return a proper 3x3 rotation matrix from a row-major config value."""
    if values is None:
        return np.eye(3, dtype=np.float64)

    rotation = np.asarray(values, dtype=np.float64)
    if rotation.size != 9:
        raise ValueError("sensor_rotation_matrix must contain 9 values")
    rotation = rotation.reshape(3, 3)
    if not np.isfinite(rotation).all():
        raise ValueError("sensor_rotation_matrix contains non-finite values")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
        raise ValueError("sensor_rotation_matrix must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError("sensor_rotation_matrix must be a proper rotation")
    return rotation


def rotate_vector3(values: object, rotation: np.ndarray) -> tuple[float, float, float]:
    """Rotate one xyz vector from the raw MID360 frame into navigation frame."""
    vector = np.asarray(values, dtype=np.float64)
    if vector.size != 3:
        raise ValueError("vector must contain 3 values")
    rotated = rotation @ vector.reshape(3)
    return tuple(float(value) for value in rotated)


def rotate_covariance9(values: object, rotation: np.ndarray) -> list[float]:
    """Rotate a row-major 3x3 covariance into the navigation frame."""
    covariance = np.asarray(values, dtype=np.float64)
    if covariance.size != 9:
        raise ValueError("covariance must contain 9 values")
    rotated = rotation @ covariance.reshape(3, 3) @ rotation.T
    return [float(value) for value in rotated.reshape(9)]


def rotate_orientation_xyzw(
    values: object, rotation: np.ndarray
) -> tuple[float, float, float, float]:
    """Express a raw-frame orientation quaternion in the corrected frame.

    ``rotation`` maps raw sensor vectors into the corrected navigation frame.
    An IMU orientation describes the raw sensor frame in the world, so the
    corrected-frame orientation is ``R_world_raw * rotation.T``.
    """
    quaternion = np.asarray(values, dtype=np.float64)
    if quaternion.size != 4:
        raise ValueError("quaternion must contain 4 values")
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must be finite and non-zero")
    x, y, z, w = quaternion / norm
    raw_to_world = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    corrected_to_world = raw_to_world @ rotation.T

    trace = float(np.trace(corrected_to_world))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * scale
        qx = (corrected_to_world[2, 1] - corrected_to_world[1, 2]) / scale
        qy = (corrected_to_world[0, 2] - corrected_to_world[2, 0]) / scale
        qz = (corrected_to_world[1, 0] - corrected_to_world[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(corrected_to_world)))
        if index == 0:
            scale = 2.0 * np.sqrt(
                1.0 + corrected_to_world[0, 0]
                - corrected_to_world[1, 1]
                - corrected_to_world[2, 2]
            )
            qw = (corrected_to_world[2, 1] - corrected_to_world[1, 2]) / scale
            qx = 0.25 * scale
            qy = (corrected_to_world[0, 1] + corrected_to_world[1, 0]) / scale
            qz = (corrected_to_world[0, 2] + corrected_to_world[2, 0]) / scale
        elif index == 1:
            scale = 2.0 * np.sqrt(
                1.0 + corrected_to_world[1, 1]
                - corrected_to_world[0, 0]
                - corrected_to_world[2, 2]
            )
            qw = (corrected_to_world[0, 2] - corrected_to_world[2, 0]) / scale
            qx = (corrected_to_world[0, 1] + corrected_to_world[1, 0]) / scale
            qy = 0.25 * scale
            qz = (corrected_to_world[1, 2] + corrected_to_world[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(
                1.0 + corrected_to_world[2, 2]
                - corrected_to_world[0, 0]
                - corrected_to_world[1, 1]
            )
            qw = (corrected_to_world[1, 0] - corrected_to_world[0, 1]) / scale
            qx = (corrected_to_world[0, 2] + corrected_to_world[2, 0]) / scale
            qy = (corrected_to_world[1, 2] + corrected_to_world[2, 1]) / scale
            qz = 0.25 * scale

    corrected = np.array([qx, qy, qz, qw], dtype=np.float64)
    corrected /= np.linalg.norm(corrected)
    return tuple(float(value) for value in corrected)


def unitree_mid360_to_navigation_cloud(
    *,
    data: bytes,
    height: int,
    width: int,
    point_step: int,
    row_step: int,
    fields: list[tuple[str, int, int, int]],
    header_stamp_ns: int,
    rotation_matrix: np.ndarray | None = None,
) -> bytes:
    """Convert Unitree's packed MID360 points to the navigation PCL schema.

    Unitree provides ``time`` as a per-point offset in nanoseconds.  The output
    preserves it as an absolute nanosecond ``timestamp`` in the same normalized
    clock domain as the ROS header and navigation IMU.
    """
    height = int(height)
    width = int(width)
    point_step = int(point_step)
    row_step = int(row_step)
    header_stamp_ns = int(header_stamp_ns)
    if min(height, width, point_step, row_step, header_stamp_ns) < 1:
        raise ValueError(
            "height, width, point_step, row_step and header_stamp_ns must be positive"
        )
    expected_row_step = width * point_step
    if row_step != expected_row_step:
        raise ValueError(
            "MID360 PointCloud2 rows must be tightly packed: "
            f"row_step={row_step}, expected={expected_row_step}"
        )
    expected_size = height * row_step
    if len(data) != expected_size:
        raise ValueError(
            f"point cloud data size must equal row_step * height: "
            f"expected={expected_size}, actual={len(data)}"
        )
    point_count = height * width

    field_map = {name: (int(offset), int(datatype), int(count)) for name, offset, datatype, count in fields}
    for name, expected_type in _EXPECTED_INPUT_TYPES.items():
        if name not in field_map:
            raise ValueError(f"missing MID360 field: {name}")
        offset, datatype, count = field_map[name]
        if datatype != expected_type or count != 1:
            raise ValueError(
                f"unexpected MID360 field {name}: datatype={datatype}, count={count}"
            )
        if offset < 0 or offset + np.dtype(_INPUT_DTYPES[name]).itemsize > point_step:
            raise ValueError(f"MID360 field {name} exceeds point_step")

    def view(name: str) -> np.ndarray:
        offset = field_map[name][0]
        return np.ndarray(
            shape=(point_count,),
            dtype=_INPUT_DTYPES[name],
            buffer=data,
            offset=offset,
            strides=(point_step,),
        )

    ring = view("ring")
    if int(ring.max(initial=0)) > 255:
        raise ValueError("MID360 ring value exceeds uint8 line range")

    relative_time_ns = view("time")
    if not np.isfinite(relative_time_ns).all() or float(relative_time_ns.min()) < 0:
        raise ValueError("MID360 time contains negative or non-finite values")

    rotation = validated_rotation_matrix(rotation_matrix)
    xyz = np.column_stack((view("x"), view("y"), view("z"))).astype(
        np.float32, copy=False
    )
    if not np.array_equal(rotation, np.eye(3)):
        xyz = xyz @ rotation.astype(np.float32).T

    converted = np.zeros(point_count, dtype=_NAVIGATION_DTYPE)
    converted["x"] = xyz[:, 0]
    converted["y"] = xyz[:, 1]
    converted["z"] = xyz[:, 2]
    converted["intensity"] = view("intensity")
    converted["tag"] = 0x10
    converted["line"] = ring.astype(np.uint8, copy=False)
    converted["timestamp"] = np.float64(header_stamp_ns) + relative_time_ns.astype(
        np.float64, copy=False
    )
    return converted.tobytes(order="C")
