"""Point cloud utilities for gravity alignment using IMU data."""

import math
import numpy as np

# Livox Mid-360 mounting on G1: inverted (180° around x-axis), pitch -2.3°
_MOUNT_PITCH_RAD = math.radians(-2.3)

# Pre-compute mounting rotation: Ry(mount_pitch) * Rx(180°)
# Rx(180): [[1,0,0],[0,-1,0],[0,0,-1]]
# Ry(p):   [[cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]]
_cp = math.cos(_MOUNT_PITCH_RAD)
_sp = math.sin(_MOUNT_PITCH_RAD)
MOUNT_ROTATION = np.array([
    [_cp,  0,   -_sp],
    [0,   -1,    0  ],
    [-_sp, 0,   -_cp],
], dtype=np.float32)


def gravity_align_inplace(data: bytes, point_step: int, total_points: int,
                          roll: float, pitch: float,
                          mount_rotation: np.ndarray = MOUNT_ROTATION) -> bytes:
    """Transform point cloud from lidar frame to gravity-aligned robot frame.

    Steps:
      1. Apply mounting rotation (inverted install + pitch offset)
      2. Apply IMU gravity compensation (Rx(-roll) * Ry(-pitch))

    Assumes xyz are the first 3 float32 fields (offsets 0, 4, 8) in each point.

    Args:
        data: Raw point cloud bytes (total_points * point_step bytes).
        point_step: Bytes per point.
        total_points: Number of points.
        roll: Current body roll angle in radians (from IMU).
        pitch: Current body pitch angle in radians (from IMU).
        mount_rotation: 3x3 rotation from lidar frame to robot body frame.

    Returns:
        Modified bytes with rotated xyz.
    """
    if total_points == 0:
        return data

    buf = np.frombuffer(data, dtype=np.uint8).copy().reshape(total_points, point_step)
    xyz = np.zeros((total_points, 3), dtype=np.float32)
    xyz[:, 0] = buf[:, 0:4].view('<f4').flatten()
    xyz[:, 1] = buf[:, 4:8].view('<f4').flatten()
    xyz[:, 2] = buf[:, 8:12].view('<f4').flatten()

    # Step 1: lidar frame → robot body frame (mounting transform)
    xyz = xyz @ mount_rotation.T

    # Step 2: robot body frame → gravity-aligned frame (IMU compensation)
    if abs(roll) > 0.001 or abs(pitch) > 0.001:
        cr, sr = np.cos(-roll), np.sin(-roll)
        cp, sp = np.cos(-pitch), np.sin(-pitch)
        R_gravity = np.array([
            [cp,       0,    sp],
            [sr * sp,  cr,  -sr * cp],
            [-cr * sp, sr,   cr * cp],
        ], dtype=np.float32)
        xyz = xyz @ R_gravity.T

    buf[:, 0:4] = xyz[:, 0:1].view(np.uint8)
    buf[:, 4:8] = xyz[:, 1:2].view(np.uint8)
    buf[:, 8:12] = xyz[:, 2:3].view(np.uint8)
    return buf.tobytes()
