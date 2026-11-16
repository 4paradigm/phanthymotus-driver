#!/usr/bin/env python3
"""RealSense device discovery and capture adapter for the FR3 driver.

Discovery and capture use pyrealsense2 with the Linux V4L2 backend. Deployment
exposes dynamically enumerated video/USB nodes; cards select stable serials.
RGB, depth and infrared instances share one SDK session per camera.
"""

import logging

log = logging.getLogger(__name__)

CHANNELS = ("rgb", "depth", "infrared")
TOPIC_FORMATS = {
    "rgb": "image/jpeg",
    "depth": "image/depth-zlib",
    "infrared": "image/jpeg",
}

def _device_info(device, field, default=""):
    try:
        if device.supports(field):
            return device.get_info(field)
    except Exception:
        pass
    return default


def _enumerate_ext_cameras() -> list[dict]:
    """Enumerate RealSense devices by stable serial number via the SDK."""
    try:
        import pyrealsense2 as rs
    except (ImportError, OSError) as exc:
        log.warning("[ext_camera] pyrealsense2 unavailable: %s", exc)
        return []

    try:
        devices = rs.context().query_devices()
    except Exception as exc:
        # A missing or disconnected camera must not stop the arm Driver.
        log.warning("[ext_camera] RealSense enumeration failed: %s", exc)
        return []

    result = []
    seen = set()
    for device in devices:
        serial = str(_device_info(device, rs.camera_info.serial_number)).strip()
        if not serial or serial in seen:
            continue
        seen.add(serial)
        name = str(_device_info(device, rs.camera_info.name, "Intel RealSense")).strip()
        usb_type = str(_device_info(
            device, rs.camera_info.usb_type_descriptor, "unknown")).strip()
        result.append({
            "path": f"realsense://{serial}",
            "name": name,
            "serial_number": serial,
            "usb_type": usb_type,
            "realsense": True,
            "channels": list(CHANNELS),
        })
    return result


class _RealSenseCameraNode:
    def __init__(self, session, instance_id, channel):
        self.session = session
        self.instance_id = instance_id
        self.channel = channel

    def start(self):
        return self.session.start(self.instance_id, self.channel)

    def stop(self):
        self.session.stop(self.instance_id)
        return self._status_dict()

    def _status_dict(self):
        return self.session.info(self.instance_id, self.channel)
