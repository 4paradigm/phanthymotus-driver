"""Point cloud utilities for gravity alignment using IMU data."""

import numpy as np


def gravity_align_inplace(data: bytes | bytearray, point_step: int, total_points: int,
                          roll: float, pitch: float) -> bytearray:
    """Rotate point cloud xyz by -roll/-pitch to align with gravity.

    Assumes xyz are the first 3 float32 fields (offsets 0, 4, 8) in each point.

    Returns bytearray (avoids extra copy vs bytes).
    Returns original data (as bytearray) if rotation is negligible.
    """
    if total_points == 0 or (abs(roll) < 0.001 and abs(pitch) < 0.001):
        return data if isinstance(data, bytearray) else bytearray(data)

    # Build rotation matrix: Rx(-roll) * Ry(-pitch)
    cr, sr = np.cos(-roll), np.sin(-roll)
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    R = np.array([
        [cp,       0,    sp],
        [sr * sp,  cr,  -sr * cp],
        [-cr * sp, sr,   cr * cp],
    ], dtype=np.float32)

    # Extract xyz (3 x float32 = 12 bytes at start of each point)
    buf = data if isinstance(data, bytearray) else bytearray(data)
    # View as uint8, reshape to (total_points, point_step), slice first 12 bytes
    raw = np.frombuffer(buf, dtype=np.uint8).reshape(total_points, point_step)
    xyz = raw[:, :12].copy().view(np.float32).reshape(total_points, 3)

    # Apply rotation
    xyz = xyz @ R.T

    # Write back
    raw[:, :12] = xyz.view(np.uint8).reshape(total_points, 12)

    return buf
