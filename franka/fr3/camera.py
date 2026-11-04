"""Franka camera cards, reusing the existing RealSense capture backend."""

import copy
import re

from common.camera_info import build as camera_info
from franka.fr3.camera_contract import stream_semantics
from franka.fr3 import camera_backend as shared_camera


class ExtCameraPlugin(shared_camera.ExtCameraPlugin):
    """Explicit per-card device binding; one shared session per USB serial."""

    def __init__(self, plugin_cfg, namespace="franka", executor=None):
        super().__init__(plugin_cfg, namespace.strip("/") or "franka", executor)
        self._closed = False
        self._generations = {}
        self._device_names = {}

    def get_tools(self):
        definition = copy.deepcopy(super().get_tools()[0])
        definition["description"] = "外接 RealSense 相机：每张卡独立选择相机及彩色、深度或近红外通道。"
        # One dynamic port per instance; info supplies its selected wire format.
        definition["topic_out"] = [{"desc": "当前通道；话题及格式由实例 info 返回"}]
        schema = definition["configSchema"]
        selection = schema["properties"]["device_path"]
        selection.update(title="相机", **{
            "x-sensitive": True,
            "description": "按相机序列号选择；同一相机可添加多张卡分别显示不同通道。",
        })
        options = {row["const"]: row["title"] for row in selection["oneOf"] if row["const"]}
        with self._lock:
            for cfg in self._instance_configs.values():
                path = cfg["device_path"]
                options.setdefault(path, f"已配置 · {path.removeprefix('realsense://')}（暂未发现）")
        selection["oneOf"] = [
            {"const": path, "title": title} for path, title in sorted(options.items())
        ] or [{"const": "", "title": "无可用 RealSense 相机"}]
        schema["properties"]["channel"].update(
            title="通道", description="rgb 彩色；depth 毫米深度（0 无效，未对齐 RGB）；infrared 左近红外灰度（非温度）。运行中保存配置会自动切换当前实例；关闭预览不会停止采集。")
        schema["properties"]["channel"]["oneOf"] = [
            {"const": "rgb", "title": "彩色（RGB）"},
            {"const": "depth", "title": "深度（距离）"},
            {"const": "infrared", "title": "近红外（黑白，非热成像）"},
        ]
        schema["required"] = ["device_path"]
        return [definition]

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
        # Lazy import keeps tools/list and arm/master cards usable without a camera SDK.
        from franka.fr3.realsense import RealSenseSession

        serial = device["serial_number"]
        if serial not in self._sessions:
            self._sessions[serial] = RealSenseSession(self._namespace, serial)
        return shared_camera._RealSenseCameraNode(
            self._sessions[serial], instance_id, cfg["channel"])

    def _info(self, instance_id):
        result = super()._info(instance_id)
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
            if device.get("name"):
                self._device_names[cfg["device_path"]] = device["name"]
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
