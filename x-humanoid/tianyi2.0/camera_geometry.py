"""Translate a validated ROS color calibration into output-port geometry.

This module does not determine whether a depth image is aligned to color. A
depth caller must establish that independently before borrowing this geometry.
"""
from copy import deepcopy
import math
from typing import Optional

from common.camera_info import build


COLOR_CAMERA_ID = "x-humanoid/tianyi2.0/head/color"
COLOR_CAMERA_INFO_TOPIC = "/ob_camera_head/color/camera_info"
_MODELS = {"plumb_bob": "pinhole", "rational_polynomial": "pinhole",
           "equidistant": "equidistant"}


def _size(value):
    try:
        width, height = value
    except (TypeError, ValueError):
        return None
    if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0
           for v in (width, height)):
        return None
    return width, height


def _finite(values, length=None):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return None
    if (length is not None and len(result) != length
            or not all(math.isfinite(value) for value in result)):
        return None
    return result


def camera_info_snapshot(msg) -> Optional[dict]:
    """Copy a usable, full-frame ROS CameraInfo; malformed input stays unknown.

    Binned or partial-ROI inputs require their own transform and are refused
    here rather than accidentally applying a full-frame K to a different grid.
    """
    try:
        size = _size((msg.width, msg.height))
        frame_id = str(msg.header.frame_id).strip()
        ros_model = str(msg.distortion_model)
        K, D = _finite(msg.k, 9), _finite(msg.d)
        if (size is None or not frame_id or ros_model not in _MODELS
                or K is None or D is None or K[0] <= 0 or K[4] <= 0):
            return None
        if msg.binning_x not in (0, 1) or msg.binning_y not in (0, 1):
            return None
        roi = msg.roi
        if (roi.x_offset != 0 or roi.y_offset != 0
                or (roi.width, roi.height) not in ((0, 0), size)):
            return None
        stamp = getattr(msg.header, "stamp", None)
        stamp_ms = None
        if stamp is not None:
            seconds, nanoseconds = stamp.sec, stamp.nanosec
            if (isinstance(seconds, int) and isinstance(nanoseconds, int)
                    and seconds >= 0 and 0 <= nanoseconds < 1_000_000_000):
                stamp_ms = seconds * 1000 + nanoseconds // 1_000_000
        return {"width": size[0], "height": size[1], "frame_id": frame_id,
                "K": K, "D": D, "ros_distortion_model": ros_model,
                "distortion_model": _MODELS[ros_model], "stamp_ms": stamp_ms}
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def color_camera_declaration(snapshot, *, topic, format, source_size,
                             output_size, frame_id, pipeline,
                             vendor=None) -> Optional[dict]:
    """Declare a full-frame resize only when calibration and source agree.

    X and Y scaling are independent: 1280x720 -> 640x480 preserves the full
    view but produces non-square pixels. Distortion remains the source lens's
    distortion; resizing is not undistortion. K uses the ROS/OpenCV convention
    of scaling focal lengths and principal points with image coordinates.
    """
    if not isinstance(snapshot, dict):
        return None
    source_size, output_size = _size(source_size), _size(output_size)
    if source_size is None or output_size is None:
        return None
    try:
        if (source_size != (snapshot["width"], snapshot["height"])
                or not frame_id or frame_id != snapshot["frame_id"]):
            return None
        source_K, source_D = _finite(snapshot["K"], 9), _finite(snapshot["D"])
        if (source_K is None or source_D is None
                or source_K[0] <= 0 or source_K[4] <= 0
                or snapshot["ros_distortion_model"] not in _MODELS):
            return None
        sx = output_size[0] / source_size[0]
        sy = output_size[1] / source_size[1]
        K = [value * (sx if index < 3 else sy if index < 6 else 1.0)
             for index, value in enumerate(source_K)]
        provenance = deepcopy(vendor or {})
        provenance.update({
            "camera_info_topic": COLOR_CAMERA_INFO_TOPIC,
            "calibration_frame_id": snapshot["frame_id"],
            "calibration_size": list(source_size),
            "calibration_K": list(source_K),
            "calibration_D": list(source_D),
            "calibration_stamp_ms": snapshot.get("stamp_ms"),
            "ros_distortion_model": snapshot["ros_distortion_model"],
            "resize_scale_xy": [sx, sy],
        })
        return build(
            topic=topic, id=COLOR_CAMERA_ID, format=format,
            width=output_size[0], height=output_size[1], K=K, D=source_D,
            distortion_model=_MODELS[snapshot["ros_distortion_model"]],
            half_fov_rad=math.atan(output_size[0] / (2.0 * K[0])),
            half_fov_v_rad=math.atan(output_size[1] / (2.0 * K[4])),
            source="derived-from-K", pipeline=pipeline, vendor=provenance)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
