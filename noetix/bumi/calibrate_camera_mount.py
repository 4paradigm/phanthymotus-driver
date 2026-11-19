"""Fit the Bumi RGB optical centre to a URDF link using corresponding 3-D points in metres."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from vision_tf import frame_name, vector


def fit_mount(pairs, parent, max_error_m):
    frame_name(parent)
    if not isinstance(pairs, list) or len(pairs) < 4:
        raise ValueError("Supply at least four corresponding, well-separated measured points")
    if not math.isfinite(max_error_m) or max_error_m <= 0:
        raise ValueError("max-error-m must be positive")
    camera = np.array([vector(p["optical_m"], "xyz") for p in pairs])
    link = np.array([vector(p["link_m"], "xyz") for p in pairs])
    a, b = camera - camera.mean(axis=0), link - link.mean(axis=0)
    for centered in (a, b):
        spread = np.linalg.svd(centered, compute_uv=False)
        if spread[1] <= max(1e-6, spread[0] * 1e-3):
            raise ValueError("Point distribution is collinear or too small for calibration")
    u, _, vt = np.linalg.svd(a.T @ b)
    correction = np.diag([1, 1, np.linalg.det(vt.T @ u.T)])
    optical_to_link = vt.T @ correction @ u.T
    translation = link.mean(axis=0) - optical_to_link @ camera.mean(axis=0)
    residual = np.linalg.norm(camera @ optical_to_link.T + translation - link, axis=1)
    if residual.max() > max_error_m:
        raise ValueError(f"Largest residual {residual.max():.6f} m exceeds {max_error_m:g} m")
    # Columns of R_mount_optical are optical right, down, forward in FLU axes.
    mount_to_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    rot = optical_to_link @ mount_to_optical.T
    horizontal = math.hypot(rot[0, 0], rot[1, 0])
    pitch = math.atan2(-rot[2, 0], horizontal)
    if horizontal > 1e-8:
        roll, yaw = math.atan2(rot[2, 1], rot[2, 2]), math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll, yaw = math.atan2(-rot[1, 2], rot[1, 1]), 0.0
    return {"camera_mount": {"parent": parent, "xyz_m": translation.tolist(), "rpy_rad": [roll, pitch, yaw]},
            "points": len(pairs), "rms_m": float(np.sqrt(np.mean(residual**2))), "max_m": float(residual.max())}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("--parent", required=True)
    parser.add_argument("--max-error-m", type=float, required=True)
    args = parser.parse_args()
    result = fit_mount(json.loads(args.pairs.read_text()), args.parent, args.max_error_m)
    print(json.dumps(result, indent=2, allow_nan=False))
