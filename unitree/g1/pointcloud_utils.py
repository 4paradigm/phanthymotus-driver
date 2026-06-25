"""Point cloud utilities for gravity alignment using IMU data."""

import numpy as np


def gravity_align_inplace(data: bytes, point_step: int, total_points: int,
                          roll: float, pitch: float) -> bytes:
    """Rotate point cloud xyz by -roll/-pitch to align with gravity.

    Assumes xyz are the first 3 float32 fields (offsets 0, 4, 8) in each point.

    Args:
        data: Raw point cloud bytes (total_points * point_step bytes).
        point_step: Bytes per point.
        total_points: Number of points.
        roll: Current roll angle in radians (from IMU).
        pitch: Current pitch angle in radians (from IMU).

    Returns:
        Modified bytes with rotated xyz. Returns original data if rotation is negligible.
    """
    if abs(roll) < 0.001 and abs(pitch) < 0.001:
        return data

    buf = np.frombuffer(data, dtype=np.uint8).copy().reshape(total_points, point_step)
    xyz = np.zeros((total_points, 3), dtype=np.float32)
    xyz[:, 0] = buf[:, 0:4].view('<f4').flatten()
    xyz[:, 1] = buf[:, 4:8].view('<f4').flatten()
    xyz[:, 2] = buf[:, 8:12].view('<f4').flatten()

    # Rotation matrix: Rx(-roll) * Ry(-pitch)
    cr, sr = np.cos(-roll), np.sin(-roll)
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    R = np.array([
        [cp,       0,    sp],
        [sr * sp,  cr,  -sr * cp],
        [-cr * sp, sr,   cr * cp],
    ], dtype=np.float32)

    xyz = xyz @ R.T

    buf[:, 0:4] = xyz[:, 0:1].view(np.uint8)
    buf[:, 4:8] = xyz[:, 1:2].view(np.uint8)
    buf[:, 8:12] = xyz[:, 2:3].view(np.uint8)
    return buf.tobytes()
