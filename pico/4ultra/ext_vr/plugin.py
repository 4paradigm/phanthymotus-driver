"""Independent PICO sensor card: ordinary MCP lifecycle and one local DDS input topic."""

from __future__ import annotations
import asyncio
import json
import os
import secrets
import threading
from pathlib import Path
from urllib.parse import urlsplit
from common.teleop_contract import canonical_instance
from .runtime import DeviceRuntime


class ExtVrPlugin:
    PREFIX = "teleop_device"

    def __init__(self, config, namespace, ros2=None, *, transport_factory=None):
        self.config, self.namespace, self.ros2 = dict(config), namespace, ros2
        self.transport_factory = transport_factory
        self.instances = {}
        self._lock = threading.RLock()
        self._instance_lock = asyncio.Lock()
        self._lifecycle_lock = asyncio.Lock()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.loop.run_forever, name="pico-device", daemon=True
        )
        self.thread.start()

    def start(self):
        pass

    def stop(self):
        self.close()

    def get_tools(self):
        return [self.get_tool()]

    def get_tool(self):
        origin = (
            self.config.get("public_wss_url", "")
            .replace("wss://", "https://")
            .split("/ws/")[0]
        )
        return {
            "name": self.PREFIX,
            "type": "sensor",
            "description": (
                "PICO 设备安装步骤：1. 部署 PICO 和机器人遥操 Driver；"
                "2. 首次设置四位管理 PIN 并保存配置；下载 App 无需 PIN；"
                "3. 复制网址下载 App 并在网页配对，连接 teleop_control 后开启项目；佩戴头显，松开双握把就绪后按住双握把遥操。"
                + "下载与配对网址：" + origin + "/onboarding。"
                +
                "本卡只采集输入，不直接执行机器人动作。"
            ),
            "multiInstance": False,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "config", "start", "stop"],
                    },
                    "instance_id": {"type": "string"},
                },
                "required": ["action"],
            },
            "configSchema": {
                "type": "object",
                "description": "App 下载与配对网址（复制到浏览器）：" + origin + "/onboarding。地址由 Driver 自动生成。配对不会启动机器人。",
                "properties": {
                    "management_pin": {
                        "type": "string", "title": "管理 PIN（四位数字）",
                        "description": "首次自行设置，之后固定保存；留空保留已有 PIN。仅用于网页配对管理，下载和已配对设备重连无需输入。",
                        "format": "password", "x-sensitive": True,
                        "pattern": "^([0-9]{4})?$", "maxLength": 4,
                        # Ordinary Canvas gear renders instance-scoped fields.
                        # Storage remains global to this single-device Driver.
                        "scope": "instance",
                    }
                },
            },
            "topic_out": [
                {
                    "port_id": "command",
                    "format": "data/teleop-cmd",
                    "schema": "motus.teleop.command/1",
                }
            ],
        }

    def dispatch(self, action, args):
        if action not in ("info", "config", "start", "stop"):
            return None
        return asyncio.run_coroutine_threadsafe(
            self.call(action, args), self.loop
        ).result(timeout=15)

    def _installation_url(self):
        return self.config.get("public_wss_url", "").replace("wss://", "https://").split("/ws/")[0] + "/onboarding"

    def _defaults(self):
        return {
            "display_name": self.config.get("display_name", "PICO 4 Ultra"),
            "driver_installed": True,
        }

    def _read_config(self, instance_id):
        path = Path(self.config["state_dir"]) / (instance_id + ".config.json")
        value = json.loads(path.read_text()) if path.exists() else self._defaults()
        value.pop("input_filter_ms", None)  # Ignore obsolete saved input filtering.
        if set(value) != {"display_name", "driver_installed"}:
            raise ValueError("device_config_invalid")
        self._validate_config(value)
        return value

    @staticmethod
    def _validate_config(value):
        if (
            not isinstance(value["display_name"], str)
            or not 1 <= len(value["display_name"]) <= 64
        ):
            raise ValueError("invalid_display_name")
        if type(value["driver_installed"]) is not bool:
            raise ValueError("invalid_installation_confirmation")

    async def _instance(self, instance_id):
        async with self._instance_lock:
            if instance_id in self.instances:
                return self.instances[instance_id]
            if self.instances:
                raise ValueError("headset_instance_already_owned")
            from .capture import CaptureManager
            from .capture_server import CaptureWssServer, capture_certificate_base64
            from .protocol import TicketCodec, TicketVerifier
            from .rtc import RtcManager

            state = Path(self.config["state_dir"])
            state.mkdir(parents=True, exist_ok=True, mode=0o700)
            config = self._read_config(instance_id)
            runtime = DeviceRuntime(instance_id)
            codec = TicketCodec(secrets.token_bytes(32))
            rtc = RtcManager(runtime, TicketVerifier(codec))
            manager = CaptureManager(
                runtime,
                rtc,
                codec,
                state_file=state / (instance_id + ".json"),
                public_wss_url=self.config["public_wss_url"],
                ca_certificate_base64=capture_certificate_base64(self.config),
            )
            server = CaptureWssServer(manager, self.config)
            item = {
                "runtime": runtime,
                "manager": manager,
                "rtc": rtc,
                "server": server,
                "config": config,
                "config_file": state / (instance_id + ".config.json"),
                "transport": None,
                "task": None,
            }

            await server.start()
            self.instances[instance_id] = item
            return item

    async def _publish(self, item):
        while item["runtime"].running:
            latest = item["runtime"].take_latest()
            if latest is not None:
                try:
                    # The transport owns an isolated, bounded DDS writer, so
                    # reliable publication never blocks this capture loop.
                    item["transport"].publish(latest)
                except Exception:
                    item["runtime"].record_protocol_error("input_publish_failed")
            await asyncio.sleep(0.005)

    async def call(self, action, args):
        if action == "info":
            return await self._call(action, args)
        # Ordinary Core immediately pushes gear saves, then reapplies them on
        # start. Serialize independent config/lifecycle requests, not input reception.
        async with self._lifecycle_lock:
            return await self._call(action, args)

    async def _call(self, action, args):
        instance = args.get("instance_id", "default")
        canonical_instance(instance)
        if len(instance) > 64 or "/" in instance:
            raise ValueError("invalid_instance_id")
        # One physical input source, regardless of the Canvas card's identity.
        # Retain the old per-card pairing file when upgrading to single-instance.
        if self.instances:
            instance = next(iter(self.instances))
        else:
            state = Path(self.config["state_dir"])
            saved = {p.name.removesuffix(".config.json") for p in state.glob("*.config.json")}
            # Older start-without-config flows still persisted headset credentials.
            for path in state.glob("*.json"):
                if path.name.endswith(".config.json"):
                    continue
                value = json.loads(path.read_text())
                if isinstance(value, dict) and set(value) == {"schema_version", "capture"}:
                    saved.add(path.stem)
            if len(saved) > 1:
                raise ValueError("multiple_saved_device_instances")
            if saved:
                instance = next(iter(saved))
                canonical_instance(instance)
        command = "/teleop/command"
        ports = {
            "topic_out": [
                {
                    "port_id": "command",
                    "topic": command,
                    "format": "data/teleop-cmd",
                    "schema": "motus.teleop.command/1",
                }
            ],
        }
        if action == "info" and instance not in self.instances:
            return {
                "state": "idle",
                "instance_id": instance,
                "configured": False,
                "config": self._read_config(instance),
                "actuation_enabled": False,
                "installation_url": self._installation_url(),
                **ports,
            }
        item = await self._instance(instance)
        runtime = item["runtime"]
        if action == "info":
            return {
                "instance_id": instance,
                **runtime.status(),
                "config": dict(item["config"]),
                "configured": True,
                "pairing_password_required": False,
                "management_pin_configured": item["server"].management_pin.configured,
                "installation": item["server"].installation_info(),
                "capture": await item["manager"].status(),
                **ports,
            }
        if action == "config":
            values = args.get(
                "config",
                {
                    k: v
                    for k, v in args.items()
                    if k not in ("action", "instance_id", "_tool_name")
                },
            )
            if not isinstance(values, dict) or set(values) - {
                "display_name",
                "driver_installed",
                "input_filter_ms",
                "pairing_admin_password",
                "installation_url",
                "usage_guide",
                "management_pin",
            }:
                raise ValueError("unsupported_device_config")
            if runtime.running:
                raise ValueError("collection_running")
            # Ignore obsolete UI URL values; the endpoint is always driver-owned.
            # Accept old saved field names for migration, but use driver presets.
            candidate = self._defaults()
            self._validate_config(candidate)
            pin = values.get("management_pin", "")
            if pin != "":
                changed = await asyncio.to_thread(item["server"].management_pin.configure, pin)
                if changed:
                    enrollment = item["server"].enrollment
                    enrollment.revoke_invitation()
                    enrollment.pending = None
                    enrollment.deadline = 0
            # Legacy saved passwords are ignored and never persisted or used.
            temporary = item["config_file"].with_suffix(".tmp")
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as stream:
                json.dump(candidate, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, item["config_file"])
            confirmed = self._read_config(instance)
            if confirmed != candidate:
                raise ValueError("configuration_readback_mismatch")
            item["config"] = confirmed
            return {
                "state": "configured",
                "confirmed": True,
                "config": dict(confirmed),
                "pairing_password_required": False,
                "management_pin_configured": item["server"].management_pin.configured,
            }
        if action == "start":
            if not runtime.running:
                if self.transport_factory:
                    item["transport"] = self.transport_factory(
                        self.namespace, instance, None
                    )
                else:
                    from .transport import RosTransport

                    item["transport"] = RosTransport(
                        self.ros2,
                        self.namespace,
                        instance,
                        None,
                    )
                runtime.start()
                item["task"] = asyncio.create_task(self._publish(item))
                await item["manager"].issue_assignment_if_connected()
            return {**runtime.status(), **ports}
        if action == "stop":
            runtime.stop()
            await item["manager"].revoke_assignment("collection_stopped")
            await item["rtc"].close_all()
            if item["task"]:
                item["task"].cancel()
                await asyncio.gather(item["task"], return_exceptions=True)
                item["task"] = None
            if item["transport"]:
                item["transport"].close()
                item["transport"] = None
            return runtime.status()
        return None

    def close(self):
        if self.loop.is_closed():
            return

        async def cleanup():
            for instance, item in list(self.instances.items()):
                await self.call("stop", {"instance_id": instance})
                await item["server"].close()

        asyncio.run_coroutine_threadsafe(cleanup(), self.loop).result(timeout=15)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)
        if not self.thread.is_alive():
            self.loop.close()
