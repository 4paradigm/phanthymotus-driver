#!/usr/bin/env python3
"""RealSense camera card for the RealMan RM75 upper-computer Driver.

Discovery and capture use pyrealsense2 through /dev/bus/usb.  No V4L2 node is
required, so Linux /dev/video numbering cannot prevent the arm Driver from
starting.  RGB, depth and infrared instances share one SDK session per camera.
"""

import logging
import re
import threading

log = logging.getLogger(__name__)

CHANNELS = ("rgb", "depth", "infrared")
TOPIC_FORMATS = {
    "rgb": "image/jpeg",
    "depth": "image/depth-zlib",
    "infrared": "image/jpeg",
}

TOOLS_EXT_CAMERA = [
    {
        "name": "ext_camera",
        "type": "sensor",
        "multiInstance": True,
        "description": "Upper-computer RealSense RGB, depth, or infrared camera",
        "inputSchema": {"type": "object", "properties": {}},
        "configSchema": {"type": "object", "properties": {}},
        "topic_in": [],
        "topic_out": [{"format": "image/jpeg", "desc": "RealSense camera stream"}],
    }
]


def _device_info(device, field, default=""):
    try:
        if device.supports(field):
            return device.get_info(field)
    except Exception:
        pass
    return default


def _enumerate_ext_cameras() -> list[dict]:
    """Enumerate RealSense devices by stable serial number via the RSUSB API."""
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


class ExtCameraPlugin:
    PREFIX = "ext_camera"

    def __init__(self, plugin_cfg: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        self._lock = threading.RLock()
        self._nodes = {}
        self._instance_configs = {}
        self._sessions = {}
        self._available_devices = _enumerate_ext_cameras()

    def get_tools(self) -> list:
        devices = [
            {
                "const": device["path"],
                "title": f"{device['name']} ({device['serial_number']})",
            }
            for device in self._available_devices
        ]
        tool = dict(TOOLS_EXT_CAMERA[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_path": {
                    "type": "string",
                    "description": "RealSense camera selected by serial number",
                    "scope": "instance",
                    "oneOf": devices or [{"const": "", "title": "无可用 RealSense 设备"}],
                },
                "channel": {
                    "type": "string",
                    "title": "channel",
                    "scope": "instance",
                    "enum": list(CHANNELS),
                    "default": "rgb",
                    "description": "rgb 彩色；depth 深度；infrared 左近红外（非热成像）",
                },
            },
        }
        return [tool]

    def start(self):
        pass

    def stop(self):
        with self._lock:
            for node in list(self._nodes.values()):
                node.stop()
            self._nodes.clear()

    def _topic(self, instance_id, channel):
        topic = (
            f"/{self._namespace}/ext_camera/"
            f"{instance_id.replace('-', '_')}/{channel}"
            if instance_id else ""
        )
        return [{"topic": topic, "format": TOPIC_FORMATS[channel]}]

    def _info(self, instance_id):
        cfg = self._instance_configs.get(instance_id, {})
        channel = cfg.get("channel", "rgb")
        info = (
            self._nodes[instance_id]._status_dict()
            if instance_id in self._nodes else {"state": "idle"}
        )
        return {
            **info,
            "channel": channel,
            "device_path": cfg.get("device_path", ""),
            "topic_in": [],
            "topic_out": self._topic(instance_id, channel),
            "available_devices": self._available_devices,
            "active_instances": list(self._nodes),
        }

    def _validate(self, cfg):
        cfg = dict(cfg)
        channel = cfg.setdefault("channel", "rgb")
        if channel not in CHANNELS:
            raise ValueError("channel must be rgb, depth or infrared")

        self._available_devices = _enumerate_ext_cameras()
        selected = cfg.get("device_path", "")
        # Existing Canvas projects may still contain the former V4L2 path.
        # Migrate it only when exactly one RealSense is connected; selecting
        # silently among multiple physical cameras would be unsafe.
        if selected.startswith("/dev/video") and len(self._available_devices) == 1:
            selected = self._available_devices[0]["path"]
        if not selected and self._available_devices:
            selected = self._available_devices[0]["path"]
        device = next(
            (item for item in self._available_devices if item["path"] == selected),
            None,
        )
        if device is None:
            raise ValueError("Selected RealSense camera is unavailable")
        cfg["device_path"] = selected
        return cfg, device

    def _make_node(self, instance_id, cfg, device):
        from realsense import RealSenseSession

        serial = device["serial_number"]
        if serial not in self._sessions:
            self._sessions[serial] = RealSenseSession(self._namespace, serial)
        return _RealSenseCameraNode(
            self._sessions[serial], instance_id, cfg["channel"])

    def dispatch(self, action, args):
        instance_id = args.get("instance_id", "")
        with self._lock:
            if action == "info":
                return self._info(instance_id)
            if action == "stop":
                if instance_id:
                    node = self._nodes.pop(instance_id, None)
                    if node is not None:
                        node.stop()
                else:
                    self.stop()
                return self._info(instance_id)
            if action not in ("config", "start"):
                return None
            if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]{0,127}", instance_id):
                raise ValueError("A valid instance_id is required")
            if any(
                key != instance_id
                and key.replace("-", "_") == instance_id.replace("-", "_")
                for key in self._instance_configs
            ):
                raise ValueError("instance_id collides with an existing ROS topic")

            supplied = {
                key: value for key, value in args.items()
                if key in ("channel", "device_path")
            }
            previous = self._instance_configs.get(instance_id, {})
            cfg, device = self._validate({**previous, **supplied})
            changed = cfg != previous
            node = self._nodes.get(instance_id)
            if node is not None and changed:
                replacement = self._make_node(instance_id, cfg, device)
                node.stop()
                self._nodes[instance_id] = replacement
                self._instance_configs[instance_id] = cfg
                replacement.start()
            else:
                self._instance_configs[instance_id] = cfg
                if action == "start":
                    if node is None:
                        node = self._make_node(instance_id, cfg, device)
                        self._nodes[instance_id] = node
                    node.start()

            result = self._info(instance_id)
            if action == "start" and result["state"] in ("starting", "running"):
                # Activation and first-frame readiness are intentionally
                # separate so Canvas can start without blocking on USB.
                result["readiness"] = result["state"]
                result["state"] = "running"
            return result
