"""Honest AS2W front-camera metadata; no borrowed R1/Go2 lens calibration.

The videohub transport was verified, not the optics. Dimensions and geometry
are null by default. A verified declaration can be supplied under the camera
card's ``camera_info`` configuration using the motus.camera/1 field names.
Merely publishing JPEG does not establish a pinhole model or a field of view.
"""
from __future__ import annotations

import math

from common.camera_info import CameraInfoError, build


CAMERA_ID = "unitree/as2w/camera_front"


def jpeg_dimensions(frame):
    """Read JPEG SOF dimensions without decoding pixels or loading an image lib.

    Skip length-delimited metadata; never scan entropy-coded image data. A
    truncated/unsupported header gives None, not an assumed videohub resolution.
    """
    try:
        data = memoryview(frame).cast("B")
    except (TypeError, ValueError):
        return None
    if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    offset = 2
    sof = (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
           0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF)
    while offset < len(data):
        if data[offset] != 0xFF:
            return None
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            return None
        marker = data[offset]
        offset += 1
        if marker in (0xD9, 0xDA, 0x00):  # end, start of scan, or invalid stuffing
            return None
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            continue
        if offset + 2 > len(data):
            return None
        length = (data[offset] << 8) | data[offset + 1]
        if length < 2 or offset + length > len(data):
            return None
        if marker in sof:
            if length < 8:
                return None
            height = (data[offset + 3] << 8) | data[offset + 4]
            width = (data[offset + 5] << 8) | data[offset + 6]
            components = data[offset + 7]
            if width and height and components and length >= 8 + 3 * components:
                return width, height
            return None
        offset += length
    return None


def declare(topic, config=None, observed_dimensions=None):
    config = {} if config is None else config
    if not isinstance(config, dict):
        raise CameraInfoError("AS2W camera.camera_info must be an object")
    config = dict(config)
    dimensions_source = "configured" if (
        config.get("width") is not None or config.get("height") is not None
    ) else "unknown"
    if dimensions_source == "unknown" and observed_dimensions is not None:
        if (isinstance(observed_dimensions, (list, tuple)) and len(observed_dimensions) == 2
                and all(isinstance(v, int) and not isinstance(v, bool) and v > 0
                        for v in observed_dimensions)):
            config["width"], config["height"] = observed_dimensions
            dimensions_source = "jpeg-header"
    # The common format validates ranges and provenance. Reject NaN/Infinity
    # here too: JSON extensions are not usable camera geometry downstream.
    for name in ("width", "height", "half_fov_rad", "half_fov_v_rad"):
        value = config.get(name)
        if value is not None and (isinstance(value, bool)
                                  or not isinstance(value, (int, float))
                                  or not math.isfinite(value)):
            raise CameraInfoError(f"camera_info.{name} must be finite or null")
    for name in ("width", "height"):
        value = config.get(name)
        if value is not None and int(value) != value:
            raise CameraInfoError(f"camera_info.{name} must be an integer or null")
    for name in ("K", "D"):
        value = config.get(name)
        if value is not None:
            if (not isinstance(value, (list, tuple))
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in value)):
                raise CameraInfoError(f"camera_info.{name} must contain finite numbers")
    if config.get("K") is not None:
        K = config["K"]
        if len(K) != 9 or K[0] <= 0 or K[4] <= 0:
            raise CameraInfoError("camera_info.K requires 9 entries and positive fx/fy")
        if config.get("width") is None or config.get("height") is None:
            raise CameraInfoError("camera_info.K requires its image width and height")
    return [build(
        topic=topic, id=CAMERA_ID, format="image/jpeg",
        width=config.get("width"), height=config.get("height"),
        distortion_model=config.get("distortion_model", "unknown"),
        K=config.get("K"), D=config.get("D"),
        half_fov_rad=config.get("half_fov_rad"),
        half_fov_v_rad=config.get("half_fov_v_rad"),
        source=config.get("source", "unknown"),
        measured_on=config.get("measured_on", ""),
        pipeline=[CAMERA_ID],
        vendor={"transport": "videohub",
                "geometry_verification": "operator-configured" if (
                    config.get("K") is not None or config.get("half_fov_rad") is not None
                ) else "unverified",
                "dimensions_source": dimensions_source})]
