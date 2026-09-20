"""Fixed-photo normalized positions and millimetre targets for the installed camera."""

import json
import math
from pathlib import Path
import zlib

import numpy as np

from .motion import rotation, vector


def positions(args, names=("start_point_x", "start_point_y", "target_point_x", "target_point_y")):
    result = []
    for name in names:
        value = args.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not -1 <= value <= 1):
            raise ValueError(f"{name} must be a finite normalized image coordinate in [-1, 1]")
        result.append(float(value))
    return [result[index:index + 2] for index in range(0, len(result), 2)]


def rotation_degrees(args):
    value = args.get("rotation_deg", 0)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not -180 <= value <= 180):
        raise ValueError("rotation_deg must be a finite angle in [-180, 180] degrees")
    return float(value)


def displacement(args, rotation_deg=0):
    result = []
    for name in ("delta_x", "delta_y"):
        value = args.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite displacement in millimetres")
        result.append(float(value))
    if not any(result) and not rotation_deg:
        raise ValueError("At least one of delta_x, delta_y and rotation_deg must be nonzero")
    return result


def camera_point(depth, metadata, pixel):
    x, y = pixel
    height, width = depth.shape
    if not 0 <= x < width or not 0 <= y < height:
        raise ValueError(f"Pixel {pixel} is outside the {width} x {height} photograph")
    scale = float(metadata["depth_scale_m"])
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid photograph depth scale; run observe again")
    z = int(depth[y, x]) * scale
    if z <= 0:
        raise ValueError(f"Pixel {pixel} has no valid depth; choose a valid surface pixel")
    intrinsics = metadata["intrinsics"]
    fx, fy, cx, cy = vector([intrinsics[k] for k in ("fx", "fy", "ppx", "ppy")], 4, "intrinsics")
    coefficients = vector(intrinsics["coeffs"], 5, "distortion coefficients")
    model = str(intrinsics["model"]).lower().rsplit(".", 1)[-1]
    if fx <= 0 or fy <= 0 or model not in ("none", "brown_conrady", "inverse_brown_conrady", "modified_brown_conrady"):
        raise ValueError("Unsupported photograph intrinsics")
    if any(coefficients):
        if model != "brown_conrady":
            raise ValueError("Unsupported nonzero photograph distortion")
        import cv2

        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
        ray = cv2.undistortPoints(np.asarray(pixel, dtype=float).reshape(1, 1, 2),
                                  matrix, np.asarray(coefficients))[0, 0]
    else:
        ray = (np.asarray(pixel) - [cx, cy]) / [fx, fy]
    return np.asarray(vector([ray[0] * z, ray[1] * z, z], 3, "camera point"))


def load_targets(observation, config, endpoint, selected, displacement_mm=None):
    path = Path(observation["metadata_path"])
    metadata = json.loads(path.read_text())
    if (metadata["observation_id"] != observation["observation_id"]
            or metadata["arm_endpoint"] != endpoint or metadata["config"] != config):
        raise ValueError("Photograph context changed; run observe again")
    if metadata.get("depth_aligned_to") != "color" or metadata.get("depth_encoding") != "zlib/uint16-le":
        raise ValueError("Photograph lacks aligned raw depth; run observe again")
    height, width = metadata["height"], metadata["width"]
    if type(height) is not int or type(width) is not int or min(height, width) <= 0:
        raise ValueError("Invalid photograph dimensions")
    # Image center is (0, 0), +X right and +Y down. Include the outer edges at +1.
    pixels = [[min(round((x + 1) * width / 2), width - 1),
               min(round((y + 1) * height / 2), height - 1)] for x, y in selected]
    depth = np.frombuffer(zlib.decompress(path.with_name("depth.zlib").read_bytes()), dtype="<u2").reshape(height, width)
    pose = vector(metadata["pose"], 6, "photograph pose")
    work = vector(metadata["frames"]["work"]["pose"], 6, "photograph work frame")
    anchor = rotation(work) @ np.asarray(pose[:3]) + work[:3]
    compensation = np.array([config["x_compensation_mm"], config["y_compensation_mm"]]) / 1000
    # This installation uses camera right -> base -X and camera down -> base +Y.
    # Pixel targets share the photograph anchor and installation compensation.
    points = [camera_point(depth, metadata, pixel) for pixel in pixels]
    targets = [anchor[:2] + point[:2] * [-1, 1] + compensation for point in points]
    if displacement_mm is not None:
        # Millimetres along image right/down map directly to base -X/+Y.
        # Start at the compensated pick point; do not apply compensation twice.
        targets.append(targets[0] + np.asarray(displacement_mm) * [-1, 1] / 1000)
    return metadata, pixels, targets
