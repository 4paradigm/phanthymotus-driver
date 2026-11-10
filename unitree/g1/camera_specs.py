"""G1 RealSense camera declarations from the *active* stream profiles.

The G1 publishes unaligned 640x480 Z16 depth and separate 1920x1080 RGB.
Their intrinsic matrices are not interchangeable. Never use guessed FOV,
reuse the colour K for depth, or claim hardware capture-time synchronization
from a ROS callback receipt stamp.

ROS/RealSense-free helpers support offline validation.
"""
from __future__ import annotations

import math

from common.camera_info import build

PROFILES = {
    "camera_depth": ("depth", "unitree/g1/realsense/depth", "image/depth-zlib",
                     640, 480, "camera_depth_optical_frame"),
    "camera_rgb": ("color", "unitree/g1/realsense/color", "image/jpeg",
                   1920, 1080, "camera_color_optical_frame"),
}


def active_stream_profiles(pipeline_profile, rs):
    """Read both *negotiated* RealSense profiles, not the requested config."""
    profiles = {}
    for name, stream in (("depth", rs.stream.depth), ("color", rs.stream.color)):
        try:
            intr = pipeline_profile.get_stream(stream).as_video_stream_profile().get_intrinsics()
            profiles[name] = {
                "width": intr.width, "height": intr.height,
                "fx": intr.fx, "fy": intr.fy,
                "ppx": intr.ppx, "ppy": intr.ppy,
                "model": str(intr.model).rsplit(".", 1)[-1],
                "coeffs": list(intr.coeffs),
            }
        except (AttributeError, RuntimeError, ValueError):
            # A missing RGB profile must not hide a valid depth calibration.
            continue
    return profiles


def _validated_profile(raw, *, width, height):
    """Return a bounded independent copy or None; invalid optics stay unknown."""
    if not isinstance(raw, dict):
        return None
    w, h = raw.get("width"), raw.get("height")
    if type(w) is not int or type(h) is not int or (w, h) != (width, height):
        return None
    keys = ("fx", "fy", "ppx", "ppy")
    values = [raw.get(k) for k in keys]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return None
    fx, fy, ppx, ppy = map(float, values)
    if fx <= 0 or fy <= 0 or not 0 <= ppx < w or not 0 <= ppy < h:
        return None
    coeffs = raw.get("coeffs")
    if not isinstance(coeffs, (list, tuple)) or len(coeffs) != 5:
        return None
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in coeffs):
        return None
    coeffs = [float(v) for v in coeffs]
    model = raw.get("model")
    if not isinstance(model, str) or not model.strip() or len(model) > 64:
        return None
    return dict(width=w, height=h, fx=fx, fy=fy, ppx=ppx, ppy=ppy,
                coeffs=coeffs, model=model.strip())


def validated_profiles(raw):
    """Keep each stream independent: a bad RGB profile cannot poison depth."""
    if not isinstance(raw, dict):
        return {}
    result = {}
    for tool, (stream, _ident, _fmt, w, h, _frame) in PROFILES.items():
        entry = _validated_profile(raw.get(stream), width=w, height=h)
        if entry:
            result[stream] = entry
    return result


def declare(tool_name, topic, fmt, profiles=None):
    """Return motus.camera/1 from one truth source, for get_tools()/info().

    Unavailable calibration advertises no K, not a guessed value.
    Nonzero distortion is reported rather than silently treated as rectified.
    """
    spec = PROFILES.get(tool_name)
    if not spec:
        return []
    stream, ident, expected_fmt, width, height, frame = spec
    if fmt != expected_fmt:
        return []
    profile = _validated_profile((profiles or {}).get(stream),
                                 width=width, height=height)
    vendor = {
        "source_stream": stream,
        "source_frame_id": frame,
        "timestamp_semantics": "ros_callback_receipt_unverified",
        "timestamp_clock_relation_verified": False,
    }
    if stream == "depth":
        vendor.update(depth_unit="mm", depth_source="measured",
                      depth_aligned_to="none")
    values = dict(width=width, height=height, distortion_model="unknown",
                  D=None, K=None, source="unknown", measured_on="")
    if profile:
        coeffs = profile["coeffs"]
        vendor["realsense_distortion_model"] = profile["model"]
        vendor["needs_rectification"] = any(abs(x) >= 1e-9 for x in coeffs)
        values.update(
            distortion_model=("pinhole" if not vendor["needs_rectification"]
                              else "unknown"),
            D=coeffs,
            K=[profile["fx"], 0., profile["ppx"],
               0., profile["fy"], profile["ppy"],
               0., 0., 1.],
            source="derived-from-K",
            measured_on="RealSense negotiated stream profile",
        )
    return [build(topic=topic, id=ident, format=fmt,
                  pipeline=["unitree/g1/realsense/" + stream],
                  vendor=vendor, **values)]
