"""Independent PICO sensor card: ordinary MCP lifecycle and two local DDS topics."""

from __future__ import annotations
import asyncio
import copy
import json
import os
import secrets
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit
from common.teleop_contract import topics, validate_feedback, canonical_instance
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
                "2. 保存配置以启动设备接入；无需配对密码；"
                "3. 复制网址下载 App 并在网页配对，连接 teleop_control 后开启项目；佩戴头显，松开双握把就绪后按住双握把遥操。"
                + "下载与配对网址：" + origin + "/onboarding。"
                +
                "本卡只采集输入，不直接执行机器人动作。"
            ),
            "multiInstance": True,
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
                "properties": {"usage_guide": {"type": "string", "title": "使用说明（固定，无需配置）", "enum": ["无需配置"], "default": "无需配置", "scope": "instance"}},
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
            "display_name": self.config.get("display_name", "PICO"),
            "driver_installed": True,
            "input_filter_ms": 0,
        }

    def _read_config(self, instance_id):
        path = Path(self.config["state_dir"]) / (instance_id + ".config.json")
        value = json.loads(path.read_text()) if path.exists() else self._defaults()
        if set(value) != {"display_name", "driver_installed", "input_filter_ms"}:
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
        if (
            type(value["input_filter_ms"]) not in (int, float)
            or not 0 <= value["input_filter_ms"] <= 200
        ):
            raise ValueError("invalid_input_filter")

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
            runtime = DeviceRuntime(
                instance_id, filter_time_ms=config["input_filter_ms"]
            )
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
                "feedback": None,
                "feedback_sequence": -1,
                "server_epoch": None,
                "retired_epochs": set(),
                "feedback_pending": None,
                "feedback_scheduled": False,
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
        # start. Serialize independent config/lifecycle requests, not input/feedback.
        async with self._lifecycle_lock:
            return await self._call(action, args)

    async def _call(self, action, args):
        instance = args.get("instance_id", "default")
        canonical_instance(instance)
        if len(instance) > 64 or "/" in instance:
            raise ValueError("invalid_instance_id")
        command, feedback = "/teleop/command", "/teleop/state"
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
            }:
                raise ValueError("unsupported_device_config")
            if runtime.running:
                raise ValueError("collection_running")
            # Ignore obsolete UI URL values; the endpoint is always driver-owned.
            # Accept old saved field names for migration, but use driver presets.
            candidate = self._defaults()
            self._validate_config(candidate)
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
            runtime.filter_time_ms = confirmed["input_filter_ms"]
            return {
                "state": "configured",
                "confirmed": True,
                "config": dict(confirmed),
                "pairing_password_required": False,
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
