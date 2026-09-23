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
                "2. 在本页设置配对管理密码，将已安装选为是并保存；"
                "3. 复制下方网址下载 App 并在网页配对，连接 teleop_control 后开启项目。"
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
                "description": "先安装 PICO 和天轶 Driver，再下载头显 App。配对连接不会启动机器人。",
                "properties": {
                    "driver_installed": {
                        "type": "boolean",
                        "title": "是否已经安装遥操驱动",
                        "default": False,
                        "scope": "instance",
                        "description": "1. 部署 PICO 和天轶 Driver；2. 从下面地址下载并安装 App；3. 设置管理密码并打开配对页；完成后手动选是。",
                    },
                    "installation_url": {
                        "type": "string",
                        "title": "App 下载与配对网址（复制后在浏览器打开）",
                        "default": origin + "/onboarding",
                        "readOnly": True,
                        "scope": "instance",
                    },
                    "display_name": {
                        "type": "string",
                        "title": "设备名称",
                        "default": "PICO",
                        "scope": "instance",
                    },
                    "pairing_admin_password": {
                        "type": "string",
                        "title": "配对管理密码",
                        "format": "password",
                        "x-sensitive": True,
                        "scope": "instance",
                        "description": "首次设置至少12字符。仅用于设备配对管理页，留空保持原密码。",
                    },
                    "input_filter_ms": {
                        "type": "number",
                        "title": "输入滤波时间常数（ms，0为关闭）",
                        "default": 0,
                        "minimum": 0,
                        "maximum": 200,
                        "scope": "instance",
                    },
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

    def _defaults(self):
        return {
            "display_name": self.config.get("display_name", "PICO"),
            "driver_installed": False,
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
            from .management import OperatorCommands

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

            def publish(value):
                if item["transport"] is None:
                    raise ValueError("device_collection_stopped")
                item["transport"].publish(value)

            operators = OperatorCommands(manager, runtime, publish)
            item["operators"] = operators
            manager.operator_commands = operators
            manager.visualization_provider = lambda: self.display(item)
            await server.start()
            self.instances[instance_id] = item
            return item

    def display(self, item):
        # The legacy PICO WSS envelope remains internal. No robot geometry is
        # forwarded by default; the camera/passthrough stays independent.
        with self._lock:
            operator = copy.deepcopy(item["operators"].status)
        if (
            time.monotonic_ns() - operator.get("observed_monotonic_ns", 0)
            > 1_000_000_000
        ):
            operator.update(
                state="unavailable", armed=False, error="teleop_feedback_unavailable"
            )
        return {
            "schema": "motus.motion.feedback/1",
            "available": False,
            "reason": "model_display_disabled",
            "operator": {
                k: v
                for k, v in operator.items()
                if k in ("state", "armed", "started", "mode", "error")
            },
        }

    def feedback(self, item, value):
        try:
            result = validate_feedback(
                value,
                instance_id=item["runtime"].instance_id,
                clock_id=item["runtime"].clock_id,
                now_ns=item["runtime"].clock_ns(),
            )
        except (TypeError, ValueError):
            return
        with self._lock:
            item["feedback_pending"] = result
            if not item["feedback_scheduled"]:
                item["feedback_scheduled"] = True
                self.loop.call_soon_threadsafe(self._drain_feedback, item)

    def _drain_feedback(self, item):
        with self._lock:
            value, item["feedback_pending"] = item["feedback_pending"], None
            item["feedback_scheduled"] = False
        if value is not None:
            self._accept_feedback(item, value)

    def _accept_feedback(self, item, value):
        if not item["runtime"].running:
            return
        current_input = value["connection_epoch"] in (None, item["runtime"].generation)
        epoch = value["server_epoch"]
        if epoch in item["retired_epochs"]:
            return
        if (
            item["server_epoch"] == epoch
            and value["sequence"] <= item["feedback_sequence"]
        ):
            return
        if item["server_epoch"] and item["server_epoch"] != epoch:
            if len(item["retired_epochs"]) >= 32:
                item["runtime"].record_protocol_error("control_restart_limit")
                return
            item["retired_epochs"].add(item["server_epoch"])
        item["server_epoch"], item["feedback_sequence"] = epoch, value["sequence"]
        with self._lock:
            if current_input:
                item["feedback"] = value
            # A stop after RTC loss is attributed to its own command identity;
            # the controller may still report the last pose's older epoch.
            item["operators"].feedback(value, update_status=current_input)

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
        # start. Those independent HTTP requests can overlap while password
        # hashing runs in a worker; serialize mutations, not input/feedback.
        async with self._lifecycle_lock:
            return await self._call(action, args)

    async def _call(self, action, args):
        instance = args.get("instance_id", "default")
        canonical_instance(instance)
        if len(instance) > 64 or "/" in instance:
            raise ValueError("invalid_instance_id")
        command, feedback = topics(self.namespace, instance)
        ports = {
            "topic_out": [
                {
                    "port_id": "command",
                    "topic": command,
                    "format": "data/teleop-cmd",
                    "schema": "motus.teleop.command/1",
                }
            ],
            "feedback_topic": feedback,
            "feedback_format": "data/teleop-state",
        }
        if action == "info" and instance not in self.instances:
            return {
                "state": "idle",
                "instance_id": instance,
                "configured": False,
                "config": self._read_config(instance),
                "actuation_enabled": False,
                "installation_url": self.get_tool()["configSchema"]["properties"][
                    "installation_url"
                ]["default"],
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
                "pairing_password_set": item["server"].admin.configured,
                "installation": item["server"].installation_info(),
                "capture": await item["manager"].status(),
                "operator": dict(item["operators"].status),
                "feedback": copy.deepcopy(item["feedback"]),
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
            }:
                raise ValueError("unsupported_device_config")
            if runtime.running:
                raise ValueError("collection_running")
            if (
                values.get(
                    "installation_url",
                    self.get_tool()["configSchema"]["properties"]["installation_url"][
                        "default"
                    ],
                )
                != self.get_tool()["configSchema"]["properties"]["installation_url"][
                    "default"
                ]
            ):
                raise ValueError("installation_url_is_driver_owned")
            candidate = {
                **item["config"],
                **{k: v for k, v in values.items() if k in item["config"]},
            }
            self._validate_config(candidate)
            password = values.get("pairing_admin_password", "")
            if password:
                await asyncio.to_thread(item["server"].admin.set_password, password)
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
                "pairing_password_set": item["server"].admin.configured,
            }
        if action == "start":
            if not item["config"]["driver_installed"]:
                raise ValueError("confirm_driver_installation_in_settings")
            if not runtime.running:
                if self.transport_factory:
                    item["transport"] = self.transport_factory(
                        self.namespace, instance, lambda v: self.feedback(item, v)
                    )
                else:
                    from .transport import RosTransport

                    item["transport"] = RosTransport(
                        self.ros2,
                        self.namespace,
                        instance,
                        lambda v: self.feedback(item, v),
                    )
                runtime.start()
                item["operators"].bind(True)
                item["task"] = asyncio.create_task(self._publish(item))
                await item["manager"].issue_assignment_if_connected()
            return {**runtime.status(), **ports}
        if action == "stop":
            item["operators"].bind(False)
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
                await item["operators"].close()
                await item["server"].close()

        asyncio.run_coroutine_threadsafe(cleanup(), self.loop).result(timeout=15)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)
        if not self.thread.is_alive():
            self.loop.close()
