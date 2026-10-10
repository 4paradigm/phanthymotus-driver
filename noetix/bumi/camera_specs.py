"""Bumi's two camera ports, as data — `motus.camera/1`.

Separate from `device.py` for one reason: that module imports `rclpy`, which
does not exist on a laptop, so anything living in it can only be tested on a
robot. These numbers are exactly the kind that must **not** reach a robot
unchecked — a field of view wrong by a factor of 1.6 is what made r1_sz refuse
doorways while its depth map reported clear ahead. A skipped test is the silent
failure this format was written against, one level up.

Same shape as `loco_servo.build_descriptor()`: pure functions over constants,
loadable and assertable on their own.

── why the datasheet is the fallback and not the answer ─────────────────────

Bumi's camera is an Intel RealSense D435i, which knows its own intrinsics and
will hand them over on request. `SPECS` below is only what to say before the
pipeline has started — and it is *not* simply the manual's numbers for the
colour port, because the manual's numbers do not describe what this driver
publishes.

The D435i's RGB sensor is 16:9 and `_camera_subprocess` opens it at 640x480,
which is 4:3. That mode keeps the vertical field and **crops horizontally**, so
the real horizontal angle is narrower than the 69.4 deg on the datasheet. Using
the datasheet value would understate nothing and overstate the width of the
world by about a third — an authoritative-looking wrong number, which is the
precise failure `common/camera_info.py` exists to prevent. So the colour
fallback is derived from the datasheet by the aspect ratio the driver actually
opens, and says `source: "manual"` rather than `"vendor-spec"`, because it is an
arithmetic step past anything Intel published.

The depth port needs no such correction: the D435i's depth stream is natively
4:3 and 640x480 is its own native mode, so 87 deg is the angle for the picture
we publish.
"""
from __future__ import annotations

import math

# Intel's published fields of view for the D435i, full angle, in degrees.
# Depth 87 x 58, RGB 69.4 x 42.5.
_DEPTH_FULL_H_DEG = 87.0
_DEPTH_FULL_V_DEG = 58.0
_RGB_FULL_H_DEG = 69.4
_RGB_FULL_V_DEG = 42.5

# What `_camera_subprocess` opens. Both streams, same resolution — but only one
# of them is native to its sensor's aspect ratio.
_WIDTH = 640
_HEIGHT = 480


def _half(full_deg: float) -> float:
    return math.radians(full_deg) / 2.0


def _cropped_half_h(full_h_deg: float, full_v_deg: float) -> float:
    """The horizontal half angle left after a 16:9 sensor is opened at 4:3.

    The mode preserves the vertical field, so the *sensor* half-width in
    normalised image plane units shrinks by the ratio of the two aspect ratios.
    Done in tangent space because that is where a pinhole projection is linear —
    halving an angle is not the same as halving the width it subtends, and this
    is a wide enough lens for the difference to matter.
    """
    tan_v = math.tan(_half(full_v_deg))
    tan_h_native = math.tan(_half(full_h_deg))
    # The native sensor's aspect, recovered from its own two angles rather than
    # assumed to be exactly 16:9 — Intel's published pair is not exactly that.
    native_aspect = tan_h_native / tan_v
    target_aspect = _WIDTH / _HEIGHT
    return math.atan(tan_v * min(native_aspect, target_aspect))


# **Fallbacks only.** `declare()` prefers the intrinsics the RealSense reports at
# runtime; these are what the card can answer before the camera has started, so
# that a consumer can refuse or degrade at `start` rather than discovering the
# problem one frame at a time.
#
# `distortion_model` stays `unknown` in both. The angles here come from a
# datasheet and an aspect-ratio argument, neither of which says anything about
# how either lens behaves towards the edges of the frame; declaring `pinhole` on
# that evidence would be a guess wearing a different field name.
SPECS = {
    "camera": {
        "id": "noetix/bumi/camera_color",
        "width": _WIDTH, "height": _HEIGHT,
        "half_fov_rad": _cropped_half_h(_RGB_FULL_H_DEG, _RGB_FULL_V_DEG),
        "half_fov_v_rad": _half(_RGB_FULL_V_DEG),
        "source": "manual",
        "vendor": {"note": "Intel D435i 手册标称 RGB 69.4x42.5 度，但那是 16:9 传感器的"
                           "全幅；驱动开的是 640x480（4:3），保垂直裁水平，所以水平角比"
                           "手册窄。这里是按宽高比换算出来的，不是量出来的 —— 相机一起来"
                           "就会被运行时内参顶掉。"},
    },
    "depth": {
        "id": "noetix/bumi/camera_depth",
        "width": _WIDTH, "height": _HEIGHT,
        "half_fov_rad": _half(_DEPTH_FULL_H_DEG),
        "half_fov_v_rad": _half(_DEPTH_FULL_V_DEG),
        "source": "vendor-spec",
        "vendor": {"note": "Intel D435i 手册标称深度 87x58 度。深度流原生就是 4:3，"
                           "640x480 是它的原生模式，所以这个角对应的就是我们发出去的"
                           "那张图。"},
    },
}

# `_camera_subprocess` prints one line with this prefix once the pipeline is up.
# The parent's existing stdout-forwarding thread recognises it. A marker rather
# than a new pipe or a file: the channel already exists, and anything written to
# it also stays in `docker logs`, which is where somebody debugging a wrong
# field of view will look first.
INTRINSICS_MARKER = "[camera_subprocess] INTRINSICS "

# RealSense stream name → the tool that publishes it.
_STREAM_OF_TOOL = {"camera": "color", "depth": "depth"}


def parse_intrinsics_line(line: str) -> dict:
    """The intrinsics out of a forwarded log line, or `{}`.

    Returns `{}` for anything unparseable rather than raising: this runs on the
    thread that forwards the camera's stdout, and a malformed line must not take
    that thread down and with it every subsequent camera log.
    """
    import json

    if not isinstance(line, str):
        return {}
    index = line.find(INTRINSICS_MARKER)
    if index < 0:
        return {}
    try:
        payload = json.loads(line[index + len(INTRINSICS_MARKER):])
    except Exception:                                         # noqa: BLE001
        return {}
    return payload if isinstance(payload, dict) else {}


def _from_intrinsics(spec: dict, intrinsics: dict) -> dict:
    """A `build()` keyword set from one RealSense `rs.intrinsics`.

    `K` rather than an angle, because `resolve_half_fov` prefers it and should:
    a calibration matrix is solved from many observations, an angle beside it is
    usually a tape measure and some trigonometry. The angle is not computed here
    at all — deriving it and shipping both would create two numbers that can
    disagree, which is the thing `resolve_half_fov`'s precedence rule exists to
    settle rather than something to reproduce on the producing side.
    """
    width = intrinsics.get("width")
    height = intrinsics.get("height")
    fx = intrinsics.get("fx")
    fy = intrinsics.get("fy")
    ppx = intrinsics.get("ppx")
    ppy = intrinsics.get("ppy")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               for v in (width, height, fx, fy, ppx, ppy)):
        return {}
    if not (width > 0 and height > 0 and fx > 0 and fy > 0):
        return {}

    coeffs = intrinsics.get("coeffs") or []
    try:
        coeffs = [float(c) for c in coeffs]
    except (TypeError, ValueError):
        coeffs = []
    model_name = str(intrinsics.get("model") or "")
    # RealSense's own model names (`brown_conrady`, `inverse_brown_conrady`,
    # `modified_brown_conrady`, `none`, ...) are not the enum this format
    # accepts, and mapping them by guesswork would put a claim in a field that
    # consumers act on. All-zero coefficients mean the stream is already
    # rectified, which *is* the pinhole case and is the one worth stating;
    # anything else stays `unknown` with the vendor's own name recorded beside
    # it, so the next person can decide with the name in front of them.
    rectified = bool(coeffs) and all(abs(c) < 1e-9 for c in coeffs)
    if rectified or model_name in ("none", ""):
        model = "pinhole"
    else:
        model = "unknown"

    vendor = dict(spec.get("vendor") or {})
    vendor["realsense_distortion_model"] = model_name or "unspecified"
    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": model,
        "D": coeffs or None,
        "K": [float(fx), 0.0, float(ppx),
              0.0, float(fy), float(ppy),
              0.0, 0.0, 1.0],
        "half_fov_rad": None,
        "half_fov_v_rad": None,
        "source": "derived-from-K",
        "measured_on": "RealSense D435i, 运行时内参",
        "vendor": vendor,
    }


def declare(tool_name: str, topic: str, fmt: str, intrinsics: dict = None) -> list:
    """This port's optics, for the camera tool's `info()`.

    A declaration, not runtime state — answerable whether or not the camera is
    streaming, which is what lets a consumer refuse or degrade at start instead
    of discovering the problem one frame at a time. `intrinsics` is the payload
    `parse_intrinsics_line` recovered, keyed by RealSense stream name; when the
    stream this port publishes is in there, it wins over the constants above.
    """
    from common.camera_info import build

    spec = SPECS.get(tool_name)
    if not spec:
        return []

    fields = {
        "width": spec.get("width"),
        "height": spec.get("height"),
        "half_fov_rad": spec.get("half_fov_rad"),
        "half_fov_v_rad": spec.get("half_fov_v_rad"),
        "source": spec.get("source", "unknown"),
        "measured_on": spec.get("measured_on", ""),
        "vendor": spec.get("vendor"),
    }
    stream = (intrinsics or {}).get(_STREAM_OF_TOOL.get(tool_name, ""))
    if isinstance(stream, dict):
        runtime = _from_intrinsics(spec, stream)
        if runtime:
            fields.update(runtime)

    return [build(topic=topic, format=fmt, id=spec["id"],
                  pipeline=[spec["id"]], **fields)]
