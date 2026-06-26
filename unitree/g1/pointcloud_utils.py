"""Point cloud utilities for gravity alignment using IMU data."""

import numpy as np


def gravity_align_inplace(data: bytes, point_step: int, total_points: int,
                          roll: float, pitch: float) -> bytes:
    """Rotate point cloud xyz by -roll/-pitch to align with gravity.

    Assumes xyz are the first 3 float32 fields (offsets 0, 4, 8) in each point.

    Returns original data if rotation is negligible.
    """
    if total_points == 0 or (abs(roll) < 0.001 and abs(pitch) < 0.001):
        return data

    # Build rotation matrix: Rx(-roll) * Ry(-pitch)
    cr, sr = np.cos(-roll), np.sin(-roll)
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    R = np.array([
        [cp,       0,    sp],
        [sr * sp,  cr,  -sr * cp],
        [-cr * sp, sr,   cr * cp],
    ], dtype=np.float32)

    # Use numpy stride tricks for efficient xyz access without full reshape
    buf = bytearray(data)
    raw = np.frombuffer(buf, dtype=np.uint8)

    # Extract xyz: 3 float32 at offset 0 in each point_step-sized block
    # Use as_strided to create a (total_points, 3) float32 view
    xyz_bytes = np.lib.stride_tricks.as_strided(
        np.frombuffer(buf, dtype=np.float32),
        shape=(total_points, 3),
        strides=(point_step, 4),
    ).copy()

    # Apply rotation
    xyz_bytes = xyz_bytes @ R.T

    # Write back
    result = np.frombuffer(buf, dtype=np.float32)
    out_view = np.lib.stride_tricks.as_strided(
        result,
        shape=(total_points, 3),
        strides=(point_step, 4),
    )
    out_view[:] = xyz_bytes

    return bytes(buf)
