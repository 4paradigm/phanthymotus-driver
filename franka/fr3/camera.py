"""Franka camera cards, reusing the existing RealSense capture backend."""

import threading
import re

from common.camera_info import build as camera_info
from franka.fr3.camera_contract import stream_semantics
from franka.fr3 import camera_backend as shared_camera


class ExtCameraPlugin:
    """Explicit per-card device binding; one shared session per USB serial."""

    PREFIX = "ext_camera"

    def __init__(self, plugin_cfg, namespace="franka", executor=None):
        self._namespace = namespace.strip("/") or "franka"
        self._lock = threading.RLock()
        self._nodes = {}
        self._instance_configs = {}
        self._sessions = {}
        self._available_devices = shared_camera._enumerate_ext_cameras()
        self._closed = False
        self._generations = {}

    def get_tools(self):
        self._available_devices = shared_camera._enumerate_ext_cameras()
        options = {device["path"]: f"{device['name']} ({device['serial_number']})"
                   for device in self._available_devices}
        selection = {
            "type": "string", "title": "相机", "scope": "instance",
            "x-sensitive": True,
            "description": "按相机序列号选择；同一相机可添加多张卡分别显示不同通道。",
        }
        with self._lock:
            for cfg in self._instance_configs.values():
                path = cfg["device_path"]
                options.setdefault(path, f"已配置 · {path.removeprefix('realsense://')}（暂未发现）")
        selection["oneOf"] = [
            {"const": path, "title": title} for path, title in sorted(options.items())
        ] or [{"const": "", "title": "无可用 RealSense 相机"}]
        channel = {
            "type": "string", "title": "通道", "scope": "instance", "default": "rgb",
            "enum": list(shared_camera.CHANNELS),
            "description": "深度单位为毫米（0 无效，未对齐 RGB）；近红外为灰度图。运行中保存配置会切换当前实例；关闭预览不会停止采集。",
            "oneOf": [
                {"const": "rgb", "title": "彩色（RGB）"},
                {"const": "depth", "title": "深度（距离）"},
                {"const": "infrared", "title": "近红外（黑白，非热成像）"},
            ],
        }
        return [{
            "name": self.PREFIX, "type": "sensor", "multiInstance": True,
            "description": "外接 RealSense 相机：每张卡独立选择相机及彩色、深度或近红外通道。",
            "inputSchema": {"type": "object", "properties": {}},
            "configSchema": {
                "type": "object", "required": ["device_path"],
                "properties": {"device_path": selection, "channel": channel},
            },
            "topic_in": [],
            "topic_out": [{"desc": "当前通道；话题及格式由实例 info 返回"}],
        }]

    def start(self):
        pass

    def _validate(self, cfg):
        channel = cfg.get("channel", "rgb")
        if not isinstance(channel, str) or channel not in shared_camera.CHANNELS:
            raise ValueError("channel must be rgb, depth or infrared")
        path = cfg.get("device_path", "")
        if not isinstance(path, str) or not re.fullmatch(
                r"realsense://[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", path):
            raise ValueError("Select a camera in this card's configuration (device_path is required)")
        devices = shared_camera._enumerate_ext_cameras()
        self._available_devices = devices
        # Preserve an offline selection. The backend reconnects only this serial;
        # neither USB enumeration order nor another connected camera can replace it.
        device = next((item for item in devices if item["path"] == path), {
            "path": path, "serial_number": path.removeprefix("realsense://"),
        })
        return {"device_path": path, "channel": channel}, device

    def _make_node(self, instance_id, cfg, device):
        # Lazy import keeps tools/list and arm cards usable without a camera SDK.
        from franka.fr3.realsense import RealSenseSession

        serial = device["serial_number"]
        if serial not in self._sessions:
            self._sessions[serial] = RealSenseSession(self._namespace, serial)
        return shared_camera._RealSenseCameraNode(
            self._sessions[serial], instance_id, cfg["channel"])

    def _topic(self, instance_id, channel):
        topic = (
            f"/{self._namespace}/ext_camera/"
            f"{instance_id.replace('-', '_')}/{channel}"
            if instance_id else ""
        )
        return [{"topic": topic, "format": shared_camera.TOPIC_FORMATS[channel]}]

    def _info(self, instance_id):
        cfg = self._instance_configs.get(instance_id, {})
        channel = cfg.get("channel", "rgb")
        info = (
            self._nodes[instance_id]._status_dict()
            if instance_id in self._nodes else {"state": "idle"}
        )
        result = {
            **info,
            "channel": channel,
            "device_path": cfg.get("device_path", ""),
            "topic_in": [],
            "topic_out": self._topic(instance_id, channel),
            "available_devices": self._available_devices,
            "active_instances": list(self._nodes),
        }
        path = result["device_path"]
        result.update(stream_semantics(self._namespace, path.removeprefix("realsense://"), result["channel"]))
        output = result["topic_out"][0]
        # The reused backend reports frame dimensions but no calibration. Unknown
        # optics stay unknown; do not infer a FOV from the camera's product name.
        result["camera_info"] = [camera_info(
            topic=output["topic"], id=f"intel/realsense/{result['channel']}",
            format=output["format"],
            width=result.get("width") if result.get("fps") else None,
            height=result.get("height") if result.get("fps") else None,
            source="unknown", pipeline=("franka/fr3/ext_camera",),
        )]
        return result

    def dispatch(self, action, args):
        if args.get("_tool_name", self.PREFIX) != self.PREFIX:
            return None
        if action not in ("config", "start", "stop", "info"):
            return None
        instance_id = args.get("instance_id", "")
        if not isinstance(instance_id, str) or not re.fullmatch(
                r"[A-Za-z_][A-Za-z0-9_-]{0,127}", instance_id):
            raise ValueError("A valid instance_id is required")
        with self._lock:
            if action == "stop":
                self._generations[instance_id] = self._generations.get(instance_id, 0) + 1
                node = self._nodes.get(instance_id)
                if node is not None:
                    node.stop()
                    del self._nodes[instance_id]
                return self._info(instance_id)
            if action == "info":
                return self._info(instance_id)
            if self._closed:
                raise RuntimeError("Driver is shutting down")
            if any(key != instance_id and key.replace("-", "_") == instance_id.replace("-", "_")
                   for key in self._generations):
                raise ValueError("instance_id collides with an existing ROS topic")
            # Reserve an ordering token before slow discovery: the latest
            # request wins even when an older enumeration finishes last.
            generation = self._generations.get(instance_id, 0) + 1
            self._generations[instance_id] = generation
            previous = dict(self._instance_configs.get(instance_id, {}))
        supplied = {key: args[key] for key in ("device_path", "channel") if key in args}
        # Discovery is outside the lifecycle lock so stop can cancel a pending start.
        cfg, device = self._validate({**previous, **supplied})
        with self._lock:
            if self._closed or self._generations[instance_id] != generation:
                return self._info(instance_id)
            node = self._nodes.get(instance_id)
            restart = node is not None and cfg != self._instance_configs[instance_id]
            if restart:
                # Replace only this card's route; other cards retain ownership of
                # their shared sessions. Validation completed before stopping it.
                if cfg["device_path"] != self._instance_configs[instance_id]["device_path"]:
                    node.stop()
                # Same camera: session.start updates its route in place. Do not
                # close/reopen USB capture merely to change the displayed channel.
                del self._nodes[instance_id]
                node = None
            if cfg != self._instance_configs.get(instance_id):
                # A newer saved selection also cancels operations which began
                # against an older snapshot while discovery was in progress.
                self._generations[instance_id] += 1
            self._instance_configs[instance_id] = cfg
            if action == "start" or restart:
                if node is None:
                    node = self._make_node(instance_id, cfg, device)
                    self._nodes[instance_id] = node
                try:
                    node.start()
                except Exception:
                    node.stop()
                    del self._nodes[instance_id]
                    raise
            result = self._info(instance_id)
            if action == "start" and result["state"] in ("starting", "running"):
                result["readiness"] = result["state"]
                result["state"] = "running"
            return result

    def stop(self):
        with self._lock:
            self._closed = True
            errors = []
            for instance_id, node in list(self._nodes.items()):
                try:
                    node.stop()
                    del self._nodes[instance_id]
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise errors[0]
