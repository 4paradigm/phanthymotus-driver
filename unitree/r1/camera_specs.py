"""R1's four lenses, as data.

Separate from `device.py` for one reason: that module imports `rclpy`, which does
not exist on a laptop, so anything living in it can only be tested on a robot.
These numbers are exactly the kind that must **not** reach a robot unchecked —
the field of view being wrong by a factor of 1.6 is what made the robot refuse
doorways while its depth map reported clear ahead. A skipped test is the silent
failure this format was written against, one level up.

Same shape as `loco_servo.build_descriptor()`: a pure function over constants,
loadable and assertable on its own.
"""
from __future__ import annotations

# **Only `main` has been measured.** The other three report `None` and say
# `unknown`, which is the point of the format: a consumer has to be able to tell
# "nobody told me" from "I was told", so it can fall back conservatively *and
# report that it did*. Filling in a plausible number for the stereo pair because
# it would look tidier is the failure `common/camera_info.py` exists to prevent.
#
# `distortion_model` stays `unknown` even for `main`. The half angle below was
# fitted with a pinhole model against a plane near the centre of the frame, which
# says nothing about how the lens behaves towards the edges — and this is an
# ultra-wide, so it probably does not stay pinhole out there. Declaring `fisheye`
# on that evidence would be the same guess wearing a different field name.
SPECS = {
    "camera_main": {
        "id": "unitree/r1/camera_main",
        "width": 1280, "height": 720,
        "half_fov_rad": 0.888,
        "source": "measured",
        "measured_on": "r1_sz, 2026-09-23",
        "vendor": {"note": "约 102 度全视场（超广角）。用 1.8 m 处一块 1.2 m 宽的"
                           "平板拟合，见 phanthymotus/actucore/tools/measure_fov.py。"
                           "**换镜头必须重量** —— 这个数错了不会报错，只会让机器人"
                           "过不了门"},
    },
    "camera_left":  {"id": "unitree/r1/camera_left",  "width": 544, "height": 448},
    "camera_right": {"id": "unitree/r1/camera_right", "width": 544, "height": 448},
    "camera_depth": {"id": "unitree/r1/camera_depth", "width": 544, "height": 448},
}


def declare(tool_name: str, topic: str, fmt: str) -> list:
    """This port's optics, for the camera tool's `info()`.

    A declaration, not runtime state — answerable whether or not the camera is
    streaming, which is what lets a consumer refuse or degrade at start instead
    of discovering the problem one frame at a time.
    """
    from common.camera_info import build

    spec = SPECS.get(tool_name)
    if not spec:
        return []
    return [build(topic=topic, format=fmt,
                  id=spec["id"],
                  width=spec.get("width"), height=spec.get("height"),
                  half_fov_rad=spec.get("half_fov_rad"),
                  source=spec.get("source", "unknown"),
                  measured_on=spec.get("measured_on", ""),
                  pipeline=[spec["id"]],
                  vendor=spec.get("vendor"))]
