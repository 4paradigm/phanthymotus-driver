"""AS2-W front-camera declaration for the ``motus.camera/1`` contract.

The AS2-W videohub stream was verified on hardware as 1920x1080 JPEG frames. The
horizontal field of view is deliberately not copied from another Unitree
model: it must be measured on this lens, then supplied through ``config.yaml``.
Keeping the declaration in this rclpy-free module makes the geometry testable
off the robot before navigation is allowed to trust it.
"""
from __future__ import annotations


CAMERA_ID = "unitree/as2w/camera_front"
DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080


def declare(topic: str, fmt: str, config: dict | None = None) -> list:
    """Return this output port's validated ``motus.camera/1`` declaration."""
    from common.camera_info import build

    config = dict(config or {})
    width = config.get("width", DEFAULT_WIDTH)
    height = config.get("height", DEFAULT_HEIGHT)
    half_fov = config.get("half_fov_rad")
    half_fov_v = config.get("half_fov_v_rad")
    K = config.get("K")
    geometry_known = half_fov is not None or K is not None
    source = config.get("source") or ("manual" if geometry_known else "unknown")

    return [build(
        topic=topic,
        format=fmt,
        id=CAMERA_ID,
        width=width,
        height=height,
        distortion_model=config.get("distortion_model", "unknown"),
        D=config.get("D"),
        K=K,
        half_fov_rad=half_fov,
        half_fov_v_rad=half_fov_v,
        source=source,
        measured_on=config.get("measured_on", ""),
        pipeline=[CAMERA_ID],
        vendor={
            "transport": "unitree-videohub",
            "note": ("width/height describe the JPEG stream; measure this "
                     "physical lens before setting half_fov_rad"),
        },
    )]
