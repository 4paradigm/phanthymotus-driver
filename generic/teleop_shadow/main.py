"""Deployable MCP + WebRTC server for the generic teleop shadow Driver."""

from __future__ import annotations

import asyncio
import base64
import binascii
import fcntl
import ipaddress
import json
import logging
import os
import re
import secrets
import ssl
import stat
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import aiohttp
import yaml
from aiohttp import web
from cryptography import x509
from capture import (
    MAX_CAPTURE_MESSAGE_BYTES,
    CaptureConnection,
    CaptureError,
    CaptureManager,
)
from protocol import (
    CAPABILITY_DIGEST,
    DISPATCH_CONTRACT,
    PROTOCOL,
    ProtocolError,
    TicketCodec,
    TicketVerifier,
)
from rtc import RtcManager
from runtime import ShadowRuntime

LOGGER = logging.getLogger("teleop-shadow")
DEFAULT_DRIVER_ID = "teleop-shadow-driver"
DEFAULT_DRIVER_NAME = "Generic Teleop Shadow Diagnostics"
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.yaml")
_INSTANCE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
MAX_CAPTURE_CA_PEM_BYTES = 32 * 1024
MAX_CAPTURE_CA_BASE64_CHARS = 43_692
REGISTRATION_COORDINATION_SCHEMA_VERSION = 1
MAX_REGISTRATION_MARKER_BYTES = 1024


class RegistrationTlsError(RuntimeError):
    """Agent Core registration cannot establish the required pinned TLS trust."""


class RegistrationCoordinationError(RuntimeError):
    """Shared stock-Core registration ordering cannot be guaranteed."""


class CaptureTlsError(RuntimeError):
    """The network Capture listener cannot start with authenticated TLS."""


def _is_loopback_bind(host: object) -> bool:
    if not isinstance(host, str):
        return False
    try:
        address = ipaddress.ip_address(host)
        if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped:
            return address.ipv4_mapped.is_loopback
        return address.is_loopback
    except ValueError:
        return host.lower() == "localhost"


def tool_definitions(
    *,
    driver_id: str = DEFAULT_DRIVER_ID,
    driver_name: str = DEFAULT_DRIVER_NAME,
    robot_id: str | None = None,
    signaling_enabled: bool = True,
) -> list[dict]:
    actions = {
        "start": {"params": [], "description": "启动 Driver 本地独占 Shadow 会话"},
        "stop": {"params": [], "description": "停止卡片并安全释放会话"},
        "info": {"params": [], "description": "返回卡片状态和数据主题信息"},
        "pair_headset": {"params": [], "description": "生成一次性头显配对码"},
        "revoke_headset": {"params": [], "description": "撤销已配对头显及其持久凭据"},
        "pause": {"params": [], "description": "暂停并进入安全保持状态"},
        "release": {"params": [], "description": "释放当前本地会话"},
        "emergency_stop": {"params": [], "description": "立即撤销会话并触发最终安全停止"},
        "status": {"params": [], "description": "读取会话、Capture、RTC 与安全状态"},
    }
    signaling = ({
        "signaling": {
            "protocol": "motus.teleop.capture.v1",
            "path": "/ws/teleop-capture",
            "access": "paired-capture-credential-only",
        },
    } if signaling_enabled else {})
    return [
        {
            "name": "teleop_session",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "无需修改 Core 的 Quest/PICO WebRTC Shadow 遥操作卡；Driver 本地管理配对、"
                "独占会话、租约和一次性 RTC 授权，并仅记录 would-apply/would-stop。"
            ),
            "annotations": {"destructiveHint": False, "idempotentHint": False},
            "inputSchema": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "action": {"type": "string", "enum": list(actions)},
                    "instance_id": {
                        "type": "string",
                        "description": "由 PhanthyMotus Core 自动注入的卡片实例 ID",
                    },
                },
                "required": ["action"],
                "x-action-params": actions,
            },
            "x-teleop": {
                "protocol": PROTOCOL,
                "driver_id": driver_id,
                "driver_name": driver_name,
                "robot_id": robot_id,
                "mode": "shadow",
                "actuation_enabled": False,
                "capability_digest": CAPABILITY_DIGEST,
                "dispatch_contract": DISPATCH_CONTRACT,
                **signaling,
            },
        },
        {
            "name": "teleop_state",
            "type": "resource",
            "multiInstance": False,
            "readOnly": True,
            "description": "Shadow 会话、Capture 租约、Pose、RTC 和拒绝计数的只读快照",
            "annotations": {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True},
            "inputSchema": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "status"],
                    },
                    "instance_id": {"type": "string"},
                },
                "x-action-params": {
                    action: {"params": []}
                    for action in ("start", "stop", "info", "status")
                },
            },
            "x-teleop": {
                "protocol": PROTOCOL,
                "driver_id": driver_id,
                "driver_name": driver_name,
                "robot_id": robot_id,
                "mode": "shadow",
                "actuation_enabled": False,
                "capability_digest": CAPABILITY_DIGEST,
                "dispatch_contract": DISPATCH_CONTRACT,
                **signaling,
            },
        },
    ]


def load_config(path: str | Path | None = None) -> dict:
    config_path = Path(path or os.environ.get("CONFIG_PATH", DEFAULT_CONFIG_PATH))
    with config_path.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    config["driver_id"] = os.environ.get(
        "MOTUS_DRIVER_ID", config.get("driver_id", DEFAULT_DRIVER_ID)
    )
    config["driver_name"] = os.environ.get(
        "MOTUS_DRIVER_NAME", config.get("driver_name", DEFAULT_DRIVER_NAME)
    )
    if "MOTUS_ROBOT_ID" in os.environ:
        config["robot_id"] = os.environ["MOTUS_ROBOT_ID"] or None
    config["bind_host"] = os.environ.get("MOTUS_BIND_HOST", config.get("bind_host", "127.0.0.1"))
    config["mcp_port"] = int(os.environ.get("MOTUS_MCP_PORT", config.get("mcp_port", 15711)))
    registration = config.setdefault("registration", {})
    registration["agent_core_url"] = os.environ.get(
        "AGENT_CORE_URL", registration.get("agent_core_url", "https://localhost:15678")
    )
    registration["advertise_url"] = os.environ.get(
        "MOTUS_MCP_URL", registration.get("advertise_url", f"http://localhost:{config['mcp_port']}/mcp")
    )
    # Production always verifies the mounted Core certificate.  Disabling it
    # requires the explicit, conspicuous local-development environment value.
    registration["verify_tls"] = True
    if "MOTUS_AGENT_CORE_VERIFY_TLS" in os.environ:
        verify_value = os.environ["MOTUS_AGENT_CORE_VERIFY_TLS"].strip()
        if verify_value not in ("0", "1"):
            raise ValueError("MOTUS_AGENT_CORE_VERIFY_TLS must be 1, or 0 for explicit local development")
        registration["verify_tls"] = verify_value == "1"
    registration["ca_file"] = os.environ.get(
        "MOTUS_AGENT_CORE_CA_FILE",
        registration.get("ca_file", "/etc/motus-core-ca/core-ca.pem"),
    )
    coordination_file = os.environ.get(
        "MOTUS_REGISTRATION_COORDINATION_FILE",
        registration.get("coordination_file"),
    )
    if coordination_file in (None, ""):
        registration["coordination_file"] = None
    else:
        coordination_path = Path(str(coordination_file))
        if (
            not coordination_path.is_absolute()
            or str(coordination_path) != str(coordination_file)
            or ".." in coordination_path.parts
            or len(str(coordination_path)) > 1024
        ):
            raise ValueError(
                "registration coordination_file must be a normalized absolute path"
            )
        registration["coordination_file"] = str(coordination_path)
    capture = config.setdefault("capture", {})
    capture["bind_host"] = os.environ.get(
        "MOTUS_CAPTURE_BIND_HOST", capture.get("bind_host", "0.0.0.0")
    )
    capture["port"] = int(os.environ.get(
        "MOTUS_CAPTURE_PORT", capture.get("port", 15712)
    ))
    capture["public_wss_url"] = os.environ.get(
        "MOTUS_CAPTURE_WSS_URL",
        capture.get("public_wss_url"),
    )
    capture["tls_cert_file"] = os.environ.get(
        "MOTUS_CAPTURE_TLS_CERT_FILE",
        capture.get("tls_cert_file", "/etc/motus-capture-tls/cert.pem"),
    )
    capture["tls_key_file"] = os.environ.get(
        "MOTUS_CAPTURE_TLS_KEY_FILE",
        capture.get("tls_key_file", "/etc/motus-capture-tls/key.pem"),
    )
    capture["state_file"] = os.environ.get(
        "MOTUS_CAPTURE_STATE_FILE",
        capture.get("state_file", "/var/lib/motus-teleop-shadow/capture.json"),
    )
    return config


def _validated_instance_id(
    value: Any,
    name: str,
    *,
    optional: bool = False,
    max_length: int = 128,
) -> str | None:
    if optional and value is None:
        return None
    if (
        not isinstance(value, str)
        or len(value) > max_length
        or not _INSTANCE_ID_RE.fullmatch(value)
    ):
        suffix = " or null" if optional else ""
        raise ValueError(
            f"{name} must be a stable 1-{max_length} character instance identifier{suffix}"
        )
    return value


def build_registration_ssl_context(registration: dict) -> ssl.SSLContext | bool:
    """Build an Agent Core client context pinned to the deployed Core certificate."""

    if not bool(registration.get("verify_tls", True)):
        return False

    ca_file = Path(str(registration.get("ca_file", "/etc/motus-core-ca/core-ca.pem")))
    if not ca_file.is_file():
        raise RegistrationTlsError(f"Agent Core pinned CA file is missing: {ca_file}")
    try:
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH, cafile=str(ca_file))
    except (OSError, ssl.SSLError) as exc:
        raise RegistrationTlsError(f"Agent Core pinned CA file is invalid: {ca_file}: {exc}") from exc

    # The current self-signed Core certificate is issued to `phanthy-motus`, while
    # host-network registration intentionally targets https://localhost:15678.
    # Chain verification remains mandatory; only the DNS-name check is disabled.
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    return context


def _validated_capture_wss_url(
    value: Any,
    *,
    expected_port: int | None = None,
) -> str:
    if not isinstance(value, str) or len(value) > 2048:
        raise CaptureTlsError("Capture public URL must be a bounded wss:// URL")
    parsed = urlsplit(value)
    try:
        url_port = parsed.port
    except ValueError as exc:
        raise CaptureTlsError("Capture public URL contains an invalid port") from exc
    if (
        parsed.scheme != "wss"
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path != "/ws/teleop-capture"
        or (expected_port is not None and url_port != expected_port)
    ):
        raise CaptureTlsError(
            "Capture public URL must be wss://<host>:<port>/ws/teleop-capture"
        )
    return value


def _capture_certificate_base64(capture: dict) -> str:
    certificate_path = Path(str(capture.get(
        "tls_cert_file", "/etc/motus-capture-tls/cert.pem"
    )))
    if not certificate_path.is_file() or certificate_path.is_symlink():
        raise CaptureTlsError(f"Capture TLS certificate is missing: {certificate_path}")
    certificate = certificate_path.read_bytes()
    if (
        not certificate
        or len(certificate) > MAX_CAPTURE_CA_PEM_BYTES
        or b"-----BEGIN CERTIFICATE-----" not in certificate
        or b"PRIVATE KEY" in certificate
    ):
        raise CaptureTlsError("Capture TLS certificate must contain only a bounded public PEM chain")
    return base64.b64encode(certificate).decode("ascii")


def _validated_capture_ca_base64(value: object) -> str:
    """Match the native client's exact public-CA bootstrap limits."""

    if (
        not isinstance(value, str)
        or not value
        or len(value) > MAX_CAPTURE_CA_BASE64_CHARS
    ):
        raise ValueError(
            "capture.ca_certificate_base64 must be at most 43692 base64 characters"
        )
    try:
        decoded = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("capture.ca_certificate_base64 must be valid base64") from exc
    if not decoded or len(decoded) > MAX_CAPTURE_CA_PEM_BYTES:
        raise ValueError(
            "capture.ca_certificate_base64 must decode to 1-32768 bytes"
        )
    if base64.b64encode(decoded).decode("ascii") != value:
        raise ValueError("capture.ca_certificate_base64 must use canonical base64")
    return value


def build_capture_ssl_context(capture: dict) -> ssl.SSLContext:
    """Load the dedicated network Capture listener's server certificate."""

    public_wss_url = _validated_capture_wss_url(
        capture.get("public_wss_url"),
        expected_port=int(capture.get("port", 15712)),
    )
    # Validate the exact public bootstrap material before loading its private
    # counterpart.  Pairing returns only this base64-encoded public chain.
    _capture_certificate_base64(capture)
    certificate_path = Path(str(capture.get(
        "tls_cert_file", "/etc/motus-capture-tls/cert.pem"
    )))
    key_path = Path(str(capture.get(
        "tls_key_file", "/etc/motus-capture-tls/key.pem"
    )))
    if not key_path.is_file() or key_path.is_symlink():
        raise CaptureTlsError(f"Capture TLS private key is missing: {key_path}")
    try:
        certificate = x509.load_pem_x509_certificate(certificate_path.read_bytes())
        alternatives = certificate.extensions.get_extension_for_class(
            x509.SubjectAlternativeName
        ).value
    except (OSError, ValueError, x509.ExtensionNotFound) as exc:
        raise CaptureTlsError("Capture TLS certificate must contain a valid SAN extension") from exc
    hostname = urlsplit(public_wss_url).hostname
    assert hostname is not None
    try:
        requested_ip = ipaddress.ip_address(hostname)
    except ValueError:
        requested_ip = None
    if requested_ip is not None:
        matches_host = requested_ip in alternatives.get_values_for_type(x509.IPAddress)
    else:
        normalized_host = hostname.rstrip(".").lower()
        matches_host = any(
            normalized_host == candidate.rstrip(".").lower()
            or (
                candidate.startswith("*.")
                and normalized_host.endswith(candidate[1:].lower())
                and normalized_host.count(".") == candidate.count(".")
            )
            for candidate in alternatives.get_values_for_type(x509.DNSName)
        )
    if not matches_host:
        raise CaptureTlsError(
            "Capture TLS certificate SAN does not match MOTUS_CAPTURE_WSS_URL host"
        )
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    try:
        context.load_cert_chain(str(certificate_path), str(key_path))
    except (OSError, ssl.SSLError) as exc:
        raise CaptureTlsError(f"Capture TLS certificate/key pair is invalid: {exc}") from exc
    return context


class ShadowDriverService:
    def __init__(self, config: dict):
        self.driver_id = str(_validated_instance_id(
            config.get("driver_id", DEFAULT_DRIVER_ID), "driver_id", max_length=64
        ))
        driver_name = config.get("driver_name", DEFAULT_DRIVER_NAME)
        if not isinstance(driver_name, str) or not driver_name.strip() or len(driver_name) > 200:
            raise ValueError("driver_name must be a non-empty string of at most 200 characters")
        self.driver_name = driver_name.strip()
        self.robot_id = _validated_instance_id(
            config.get("robot_id"),
            "robot_id",
            optional=True,
            # Agent Core uses the same 64-character authority-domain boundary.
            max_length=64,
        )
        registration = config.get("registration", {})
        registration_enabled = bool(registration.get("enabled", True))
        teleop = config.get("teleop", {})
        # This secret protects only the in-process, one-use handoff from the
        # authenticated Capture socket to the RTC verifier.  It is deliberately
        # regenerated per process and is never configurable, persisted, or
        # shared with Core.
        ticket_codec = TicketCodec(secrets.token_bytes(32))
        self.runtime = ShadowRuntime(
            lease_timeout_ms=int(teleop.get("lease_timeout_ms", 1000)),
            pose_timeout_ms=int(teleop.get("pose_timeout_ms", 200)),
            watchdog_interval_ms=int(teleop.get("watchdog_interval_ms", 25)),
            driver_id=self.driver_id,
            driver_name=self.driver_name,
            robot_id=self.robot_id,
            dispatch_io_timeout_ms=int(teleop.get("dispatch_io_timeout_ms", 100)),
            dispatch_ack_timeout_ms=int(teleop.get("dispatch_ack_timeout_ms", 200)),
        )
        verifier = TicketVerifier(
            ticket_codec,
            max_ttl_seconds=int(teleop.get("ticket_ttl_max_seconds", 30)),
            max_replay_entries=int(teleop.get("ticket_replay_cache_entries", 4096)),
        )
        self.rtc = RtcManager(self.runtime, verifier)
        capture_config = config.get("capture", {})
        public_wss_url = capture_config.get("public_wss_url")
        if public_wss_url is not None:
            public_wss_url = _validated_capture_wss_url(
                public_wss_url,
                expected_port=int(capture_config.get("port", 15712)),
            )
        ca_certificate_base64 = capture_config.get("ca_certificate_base64")
        if ca_certificate_base64 is None and capture_config.get("tls_cert_file"):
            certificate_path = Path(str(capture_config["tls_cert_file"]))
            if certificate_path.is_file() and not certificate_path.is_symlink():
                ca_certificate_base64 = _capture_certificate_base64(capture_config)
        if ca_certificate_base64 is not None:
            ca_certificate_base64 = _validated_capture_ca_base64(
                ca_certificate_base64
            )
        self.capture = CaptureManager(
            self.runtime,
            self.rtc,
            ticket_codec,
            pairing_ttl_seconds=int(capture_config.get("pairing_ttl_seconds", 60)),
            ticket_ttl_seconds=int(teleop.get("ticket_ttl_seconds", 20)),
            presence_interval_ms=int(capture_config.get("presence_interval_ms", 1000)),
            presence_timeout_ms=int(capture_config.get("presence_timeout_ms", 5000)),
            state_file=capture_config.get("state_file") or None,
            public_wss_url=public_wss_url,
            ca_certificate_base64=ca_certificate_base64,
        )
        self.config = config
        self._instance_id: str | None = None
        if bool(config.get("mcp_allow_non_loopback", False)):
            raise ValueError("MCP is permanently loopback-only for stock Core compatibility")
        self.registration_task: asyncio.Task | None = None
        self.registration_status = {
            "state": "starting" if registration_enabled else "disabled",
            "attempts": 0,
            "successes": 0,
            "last_http_status": None,
            "last_error": None,
            "coordination": (
                "shared_file_lock"
                if registration.get("coordination_file")
                else "single_instance_fast_path"
            ),
            "tls_verification": (
                "pinned_certificate"
                if bool(registration.get("verify_tls", True))
                else "disabled_local_development"
            ),
        }

    async def dispatch_tool(self, name: str, arguments: Any) -> dict:
        if not isinstance(arguments, dict):
            raise ProtocolError("invalid_arguments", "tool arguments must be an object")
        args = dict(arguments)
        if name == "teleop_state":
            instance_id = args.pop("instance_id", None)
            action = args.pop("action", "status")
            if args or action not in {"start", "stop", "info", "status"}:
                raise ProtocolError("invalid_arguments", "teleop_state accepts only lifecycle/status actions")
            result = self.runtime.status()
            result["capture_control"] = await self.capture.status()
            result["instance_id"] = instance_id
            result["topic_out"] = []
            return result
        if name != "teleop_session":
            raise ProtocolError("unknown_tool", f"unknown tool: {name}")
        action = args.pop("action", None)
        if not isinstance(action, str):
            raise ProtocolError("missing_action", "teleop_session requires an action")
        instance_id = args.pop("instance_id", None)
        if instance_id is not None:
            try:
                instance_id = _validated_instance_id(instance_id, "instance_id")
            except ValueError as exc:
                raise ProtocolError("invalid_instance_id", str(exc)) from exc
        if args:
            raise ProtocolError("invalid_arguments", f"{action} accepts no additional arguments")
        if action == "start":
            if (
                self._instance_id is not None
                and instance_id is not None
                and self._instance_id != instance_id
                and self.runtime.status()["authority_valid"]
            ):
                raise ProtocolError("instance_conflict", "another card instance owns the session")
            result = await asyncio.to_thread(self.runtime.prepare_local_session)
            self._instance_id = instance_id or self._instance_id
            await self.capture.issue_assignment_if_connected()
            result["lifecycle_state"] = (
                "ready" if result["dispatch"]["ready"] else "fault"
            )
            result["instance_id"] = self._instance_id
            result["capture_control"] = await self.capture.status()
            return result
        if action == "stop":
            result = await asyncio.to_thread(
                self.runtime.release_local,
                reason="lifecycle_stop",
            )
            await self.rtc.close_all()
            await self.capture.revoke_assignment("lifecycle_stop")
            self._instance_id = None
            return result
        if action == "info":
            result = self.runtime.status()
            result["instance_id"] = self._instance_id or instance_id
            result["topic_out"] = []
            result["capture_control"] = await self.capture.status()
            return result
        if action == "pair_headset":
            if not self.runtime.status()["authority_valid"]:
                raise ProtocolError("session_inactive", "start the teleop_session card before pairing")
            result = await self.capture.create_pairing()
            if not result["wss_url"] or not result["ca_certificate_base64"]:
                raise ProtocolError(
                    "capture_tls_unavailable",
                    "Capture WSS URL or public CA bootstrap is not configured",
                )
            result["state"] = "pairing_ready"
            return result
        if action == "revoke_headset":
            result = await asyncio.to_thread(
                self.runtime.release_local,
                reason="capture_revoked",
            )
            await self.rtc.close_all()
            try:
                revoked = await self.capture.revoke_headset()
            except CaptureError as exc:
                raise ProtocolError(exc.code, str(exc)) from exc
            result["headset"] = revoked
            return result
        if action == "pause":
            result = await asyncio.to_thread(self.runtime.pause_local)
            await self.rtc.close_all()
            await self.capture.revoke_assignment("operator_pause")
            return result
        if action == "release":
            result = await asyncio.to_thread(self.runtime.release_local)
            await self.rtc.close_all()
            await self.capture.revoke_assignment("operator_release")
            return result
        if action == "emergency_stop":
            result = await asyncio.to_thread(
                self.runtime.release_local,
                reason="emergency_stop",
            )
            await self.rtc.close_all()
            await self.capture.revoke_assignment("emergency_stop")
            return result
        if action == "status":
            result = self.runtime.status()
            result["instance_id"] = self._instance_id or instance_id
            result["capture_control"] = await self.capture.status()
            return result
        raise ProtocolError("unknown_action", f"unknown teleop_session action: {action}")

    async def close(self) -> None:
        if self.registration_task:
            self.registration_task.cancel()
            try:
                await self.registration_task
            except asyncio.CancelledError:
                pass
        await self.rtc.close_all()
        await asyncio.to_thread(self.runtime.close)


SERVICE_KEY = web.AppKey("teleop_shadow_service", ShadowDriverService)


def _rpc_success(request_id: Any, result: Any) -> web.Response:
    return web.json_response({"jsonrpc": "2.0", "id": request_id, "result": result})


def _rpc_error(request_id: Any, code: int, message: str, *, data: dict | None = None) -> web.Response:
    error = {"code": code, "message": message}
    if data:
        error["data"] = data
    return web.json_response({"jsonrpc": "2.0", "id": request_id, "error": error})


async def mcp_handler(request: web.Request) -> web.Response:
    service = request.app[SERVICE_KEY]
    if not _is_loopback_bind(request.remote):
        return web.json_response({"error": "mcp_loopback_only"}, status=403)
    try:
        rpc = await request.json()
    except (json.JSONDecodeError, ValueError, RecursionError):
        return _rpc_error(None, -32700, "Parse error")
    if not isinstance(rpc, dict) or rpc.get("jsonrpc") != "2.0":
        return _rpc_error(rpc.get("id") if isinstance(rpc, dict) else None, -32600, "Invalid Request")
    request_id = rpc.get("id")
    if request_id is None:
        return web.Response(status=202)
    method = rpc.get("method")
    params = rpc.get("params") or {}
    try:
        if method == "initialize":
            return _rpc_success(request_id, {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "teleop-shadow-driver", "version": "2.0.0"},
            })
        if method == "tools/list":
            return _rpc_success(request_id, {"tools": tool_definitions(
                driver_id=service.driver_id,
                driver_name=service.driver_name,
                robot_id=service.robot_id,
                signaling_enabled=service.rtc.enabled,
            )})
        if method == "tools/call":
            if not isinstance(params, dict):
                raise ProtocolError("invalid_params", "params must be an object")
            result = await service.dispatch_tool(params.get("name", ""), params.get("arguments") or {})
            return _rpc_success(request_id, {
                "content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False, allow_nan=False)}]
            })
        return _rpc_error(request_id, -32601, f"Method not found: {method}")
    except ProtocolError as exc:
        if exc.code in ("unknown_tool", "unknown_action"):
            return _rpc_error(request_id, -32601, str(exc), data={"code": exc.code})
        return _rpc_error(request_id, -32602, str(exc), data={"code": exc.code})
    except Exception:
        LOGGER.exception("unhandled MCP request failure")
        return _rpc_error(request_id, -32603, "Internal error")


async def offer_handler(request: web.Request) -> web.Response:
    return web.json_response(
        {
            "error": {
                "code": "capture_control_required",
                "message": "RTC offers must use an authenticated /ws/teleop-capture connection",
            }
        },
        status=401,
    )


class _DuplicateJsonField(ValueError):
    pass


def _capture_json(raw: str) -> dict:
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > MAX_CAPTURE_MESSAGE_BYTES:
        raise CaptureError("capture_message_invalid")

    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise _DuplicateJsonField(key)
            result[key] = value
        return result

    try:
        value = json.loads(
            raw,
            object_pairs_hook=reject_duplicates,
            parse_constant=lambda _value: (_ for _ in ()).throw(ValueError()),
        )
    except (
        UnicodeEncodeError,
        json.JSONDecodeError,
        _DuplicateJsonField,
        TypeError,
        ValueError,
        RecursionError,
    ) as exc:
        raise CaptureError("capture_message_invalid") from exc
    if not isinstance(value, dict):
        raise CaptureError("capture_message_invalid")
    return value


async def _capture_ws_text(
    websocket: web.WebSocketResponse,
    *,
    timeout: float | None = None,
) -> str:
    try:
        message = await websocket.receive(timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise CaptureError("capture_auth_timeout", status=408) from exc
    if message.type != web.WSMsgType.TEXT or not isinstance(message.data, str):
        if message.type in {web.WSMsgType.CLOSE, web.WSMsgType.CLOSING, web.WSMsgType.CLOSED}:
            raise EOFError
        raise CaptureError("capture_message_invalid")
    return message.data


async def capture_websocket_handler(request: web.Request) -> web.StreamResponse:
    service = request.app[SERVICE_KEY]
    websocket = web.WebSocketResponse(max_msg_size=MAX_CAPTURE_MESSAGE_BYTES)
    await websocket.prepare(request)
    if request.query_string:
        await websocket.send_json({"type": "error", "code": "capture_query_forbidden"})
        await websocket.close(code=4400, message=b"capture_query_forbidden")
        return websocket

    connection: CaptureConnection | None = None
    receive_task: asyncio.Task | None = None
    event_task: asyncio.Task | None = None
    try:
        first = _capture_json(await _capture_ws_text(websocket, timeout=5.0))
        connection, acknowledgement, assignment = await service.capture.connect(first)
        await websocket.send_json(acknowledgement)
        if assignment is not None:
            await websocket.send_json(assignment)

        receive_task = asyncio.create_task(_capture_ws_text(websocket))
        event_task = asyncio.create_task(connection.events.get())
        while True:
            done, _ = await asyncio.wait(
                {receive_task, event_task},
                timeout=0.25,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if not done:
                if service.capture.presence_expired(connection):
                    await service.capture.disconnect(connection)
                    connection = None
                    await websocket.send_json({
                        "type": "error",
                        "code": "capture_presence_timeout",
                    })
                    await websocket.close(code=4408, message=b"capture_presence_timeout")
                    break
                continue
            if event_task in done:
                event = event_task.result()
                await websocket.send_json(event)
                if event.get("type") in {"capture_revoked", "capture_stale"}:
                    await websocket.close(code=4403, message=b"capture_revoked")
                    break
                event_task = asyncio.create_task(connection.events.get())
            if receive_task in done:
                incoming = _capture_json(receive_task.result())
                message_type = incoming.get("type")
                if message_type == "presence":
                    response = await service.capture.presence(connection, incoming)
                elif message_type == "signaling_offer":
                    response = await service.capture.signaling_offer(connection, incoming)
                else:
                    raise CaptureError("capture_message_invalid")
                await websocket.send_json(response)
                receive_task = asyncio.create_task(_capture_ws_text(websocket))
    except EOFError:
        pass
    except asyncio.CancelledError:
        raise
    except CaptureError as exc:
        if connection is not None:
            # A protocol/signaling failure is itself a Capture control-plane
            # loss.  HOLD before awaiting the error response or close frame.
            await service.capture.disconnect(connection)
            connection = None
        if not websocket.closed:
            await websocket.send_json({"type": "error", "code": exc.code})
            await websocket.close(
                code=4403 if exc.status in {401, 403} else 4400,
                message=exc.code.encode("ascii", errors="ignore")[:120],
            )
    finally:
        tasks = [
            task
            for task in (receive_task, event_task)
            if task is not None
        ]
        for task in tasks:
            if task is not None:
                task.cancel()
        if tasks:
            # Consume both pending and already-completed task outcomes.  A
            # terminal Capture event and a peer disconnect can become ready in
            # the same loop turn; leaving the completed receive task unread
            # would otherwise emit ``Task exception was never retrieved``.
            await asyncio.gather(*tasks, return_exceptions=True)
        if connection is not None:
            await service.capture.disconnect(connection)
    return websocket


async def health_handler(request: web.Request) -> web.Response:
    service = request.app[SERVICE_KEY]
    state = service.runtime.status()
    dispatch_ready = bool(state["dispatch"]["ready"])
    ready = dispatch_ready and service.rtc.enabled
    return web.json_response({
        "state": "running" if ready else "diagnostic" if dispatch_ready else "fault",
        "ready": ready,
        "driver": service.driver_id,
        "driver_id": service.driver_id,
        "driver_name": service.driver_name,
        "robot_id": service.robot_id,
        "mode": "shadow",
        "actuation_enabled": False,
        "boot_id": state["boot_id"],
        "dispatch": {
            "kind": state["dispatch"]["kind"],
            "state": state["dispatch"]["state"],
            "stop_acknowledged": state["dispatch"]["stop_acknowledged"],
            "fault_code": state["dispatch"]["fault_code"],
        },
        "rtc_enabled": service.rtc.enabled,
        "mcp_access": "loopback-only",
        "capture_control": await service.capture.status(),
        "registration": dict(service.registration_status),
    })


def registration_payload(service: ShadowDriverService, registration: dict | None = None) -> dict:
    registration = registration or service.config.get("registration", {})
    return {
        "name": service.driver_name,
        "url": registration.get("advertise_url", "http://localhost:15711/mcp"),
        "transport": "http",
        "category": "driver",
        "render_hint": "teleop",
    }


def registration_headers(service: ShadowDriverService) -> dict[str, str]:
    # Stock Core's registration and Driver calls do not use a Core-issued
    # Bearer credential.  The MCP route is protected by its loopback boundary.
    return {"Content-Type": "application/json"}


class RegistrationCoordinator:
    """Serialize stock-Core POSTs across Driver containers with a durable barrier."""

    def __init__(
        self,
        marker_file: str | Path,
        *,
        wall_clock=time.time,
        barrier_sleep=asyncio.sleep,
        lock_poll_sleep=asyncio.sleep,
        lock_poll_seconds: float = 0.02,
    ):
        marker = Path(marker_file)
        if (
            not marker.is_absolute()
            or marker.parent == Path(marker.anchor)
            or ".." in marker.parts
            or len(str(marker)) > 1024
        ):
            raise RegistrationCoordinationError(
                "registration coordination marker must be a bounded absolute file path"
            )
        self.marker_file = marker
        self.lock_file = marker.with_name(f".{marker.name}.lock")
        self._wall_clock = wall_clock
        self._barrier_sleep = barrier_sleep
        self._lock_poll_sleep = lock_poll_sleep
        self._lock_poll_seconds = max(0.001, float(lock_poll_seconds))
        self._descriptor: int | None = None
        self._barrier_second: int | None = None
        self._validate_real_parent()

    def _validate_real_parent(self) -> None:
        parent = self.marker_file.parent
        current = Path(parent.anchor)
        try:
            for part in parent.parts[1:]:
                current /= part
                metadata = current.lstat()
                if stat.S_ISLNK(metadata.st_mode):
                    raise RegistrationCoordinationError(
                        "registration coordination directory must not use symlink ancestors"
                    )
            if parent.resolve(strict=True) != parent or not parent.is_dir():
                raise RegistrationCoordinationError(
                    "registration coordination directory must be an existing canonical directory"
                )
        except RegistrationCoordinationError:
            raise
        except (OSError, RuntimeError) as exc:
            raise RegistrationCoordinationError(
                "registration coordination directory must be an existing canonical directory"
            ) from exc

    @staticmethod
    def _directory_fsync(directory: Path) -> None:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        flags |= getattr(os, "O_CLOEXEC", 0)
        descriptor = os.open(directory, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @staticmethod
    def _validate_private_regular_file(descriptor: int, field: str) -> os.stat_result:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or stat.S_IMODE(metadata.st_mode) != 0o600
            or metadata.st_nlink != 1
        ):
            raise RegistrationCoordinationError(
                f"{field} must be a private 0600 regular file with one link"
            )
        return metadata

    def _open_lock_file(self) -> int:
        flags = os.O_RDWR | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        created = False
        try:
            descriptor = os.open(self.lock_file, flags | os.O_CREAT | os.O_EXCL, 0o600)
            created = True
        except FileExistsError:
            try:
                descriptor = os.open(self.lock_file, flags)
            except OSError as exc:
                raise RegistrationCoordinationError(
                    "registration coordination lock is not a safe regular file"
                ) from exc
        except OSError as exc:
            raise RegistrationCoordinationError(
                "registration coordination lock could not be created"
            ) from exc
        try:
            if created:
                os.fchmod(descriptor, 0o600)
            self._validate_private_regular_file(
                descriptor,
                "registration coordination lock",
            )
            if created:
                self._directory_fsync(self.lock_file.parent)
            return descriptor
        except BaseException:
            os.close(descriptor)
            raise

    async def __aenter__(self) -> RegistrationCoordinator:
        if self._descriptor is not None:
            raise RegistrationCoordinationError(
                "registration coordinator cannot be entered twice"
            )
        descriptor = self._open_lock_file()
        try:
            while True:
                try:
                    fcntl.flock(
                        descriptor,
                        fcntl.LOCK_EX | fcntl.LOCK_NB,
                    )
                    break
                except BlockingIOError:
                    await self._lock_poll_sleep(self._lock_poll_seconds)
            self._descriptor = descriptor
            descriptor = -1
            return self
        finally:
            if descriptor >= 0:
                os.close(descriptor)

    async def __aexit__(self, _exc_type, _exc, _traceback) -> None:
        descriptor = self._descriptor
        self._descriptor = None
        self._barrier_second = None
        if descriptor is None:
            return
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def _read_barrier_sync(self) -> int | None:
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        flags |= getattr(os, "O_NONBLOCK", 0)
        try:
            descriptor = os.open(self.marker_file, flags)
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise RegistrationCoordinationError(
                "registration coordination marker is not a safe regular file"
            ) from exc
        try:
            metadata = self._validate_private_regular_file(
                descriptor,
                "registration coordination marker",
            )
            if not 1 <= metadata.st_size <= MAX_REGISTRATION_MARKER_BYTES:
                raise RegistrationCoordinationError(
                    "registration coordination marker has an invalid size"
                )
            encoded = os.read(descriptor, MAX_REGISTRATION_MARKER_BYTES + 1)
        finally:
            os.close(descriptor)
        try:
            value = json.loads(encoded.decode("ascii"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise RegistrationCoordinationError(
                "registration coordination marker is invalid"
            ) from exc
        if (
            type(value) is not dict
            or set(value) != {
                "schema_version",
                "last_attempt_barrier_unix_second",
            }
            or value["schema_version"] != REGISTRATION_COORDINATION_SCHEMA_VERSION
            or type(value["last_attempt_barrier_unix_second"]) is not int
            or not 0 <= value["last_attempt_barrier_unix_second"] <= (2**63 - 1)
        ):
            raise RegistrationCoordinationError(
                "registration coordination marker schema is invalid"
            )
        return value["last_attempt_barrier_unix_second"]

    def _write_barrier_sync(self, second: int) -> None:
        if self.marker_file.exists() or self.marker_file.is_symlink():
            try:
                metadata = self.marker_file.lstat()
            except OSError as exc:
                raise RegistrationCoordinationError(
                    "registration coordination marker could not be inspected"
                ) from exc
            if (
                not stat.S_ISREG(metadata.st_mode)
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or metadata.st_nlink != 1
            ):
                raise RegistrationCoordinationError(
                    "registration coordination marker must remain a private 0600 regular file"
                )
        encoded = json.dumps(
            {
                "schema_version": REGISTRATION_COORDINATION_SCHEMA_VERSION,
                "last_attempt_barrier_unix_second": second,
            },
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
        temporary = self.marker_file.with_name(
            f".{self.marker_file.name}.{secrets.token_hex(8)}.tmp"
        )
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        descriptor = -1
        try:
            descriptor = os.open(temporary, flags, 0o600)
            with os.fdopen(descriptor, "wb") as handle:
                descriptor = -1
                os.fchmod(handle.fileno(), 0o600)
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.marker_file)
            self._directory_fsync(self.marker_file.parent)
        except RegistrationCoordinationError:
            raise
        except OSError as exc:
            raise RegistrationCoordinationError(
                "registration coordination marker could not be persisted"
            ) from exc
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    async def wait_for_turn(self) -> None:
        if self._descriptor is None:
            raise RegistrationCoordinationError(
                "registration coordination lock is not held"
            )
        barrier = await asyncio.to_thread(self._read_barrier_sync)
        self._barrier_second = barrier
        while barrier is not None and int(self._wall_clock()) <= barrier:
            remaining = (barrier + 1) - float(self._wall_clock())
            await self._barrier_sleep(max(0.001, min(remaining, 0.25)))

    async def _persist_barrier(self, second: int) -> None:
        barrier = max(second, self._barrier_second or 0)
        write_task = asyncio.create_task(
            asyncio.to_thread(self._write_barrier_sync, barrier)
        )
        try:
            await asyncio.shield(write_task)
        except asyncio.CancelledError:
            # The flock must not be released until the durable barrier exists.
            await write_task
            raise
        self._barrier_second = barrier

    async def reserve_attempt_second(self) -> None:
        """Persist a conservative same-second barrier before starting POST."""

        if self._descriptor is None:
            raise RegistrationCoordinationError(
                "registration coordination lock is not held"
            )
        await self._persist_barrier(int(self._wall_clock()))

    async def record_attempt_finished(self) -> None:
        """Advance the barrier at the actual response/error/cancel boundary."""

        if self._descriptor is None:
            raise RegistrationCoordinationError(
                "registration coordination lock is not held"
            )
        await self._persist_barrier(int(self._wall_clock()))


async def _post_registration_attempt(
    session: aiohttp.ClientSession,
    endpoint: str,
    *,
    payload: dict,
    headers: dict[str, str],
    ssl_context: ssl.SSLContext | bool,
    coordinator: RegistrationCoordinator | None,
) -> int:
    async def post_and_consume() -> int:
        async with session.post(
            endpoint,
            json=payload,
            headers=headers,
            ssl=ssl_context,
        ) as response:
            await response.read()
            return response.status

    if coordinator is None:
        return await post_and_consume()

    async with coordinator:
        await coordinator.wait_for_turn()
        await coordinator.reserve_attempt_second()
        post_started = False
        try:
            post_started = True
            return await post_and_consume()
        finally:
            if post_started:
                await coordinator.record_attempt_finished()


async def _registration_loop(service: ShadowDriverService) -> None:
    registration = service.config.get("registration", {})
    interval = max(5.0, float(registration.get("interval_seconds", 30)))
    endpoint = registration.get("agent_core_url", "https://localhost:15678").rstrip("/") + "/api/mcp"
    payload = registration_payload(service, registration)
    headers = registration_headers(service)
    timeout = aiohttp.ClientTimeout(total=5)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        while True:
            if not service.rtc.enabled:
                service.registration_status["state"] = "rtc_unavailable"
                service.registration_status["last_error"] = (
                    "ticket verification is not ready; registration inhibited"
                )
                await asyncio.sleep(interval)
                continue
            if not service.runtime.status()["dispatch"]["ready"]:
                service.registration_status["state"] = "dispatch_fault"
                service.registration_status["last_error"] = (
                    "final dispatch is not ready; registration inhibited"
                )
                await asyncio.sleep(interval)
                continue
            service.registration_status["attempts"] += 1
            try:
                ssl_context = build_registration_ssl_context(registration)
                coordination_file = registration.get("coordination_file")
                coordinator = (
                    RegistrationCoordinator(coordination_file)
                    if coordination_file
                    else None
                )
                status = await _post_registration_attempt(
                    session,
                    endpoint,
                    payload=payload,
                    headers=headers,
                    ssl_context=ssl_context,
                    coordinator=coordinator,
                )
                service.registration_status["last_http_status"] = status
                if status >= 400:
                    service.registration_status["state"] = "http_error"
                    service.registration_status["last_error"] = f"Agent Core returned HTTP {status}"
                    LOGGER.warning("registration failed with HTTP %s", status)
                else:
                    service.registration_status["state"] = "registered"
                    service.registration_status["successes"] += 1
                    service.registration_status["last_error"] = None
            except asyncio.CancelledError:
                raise
            except RegistrationCoordinationError as exc:
                service.registration_status["state"] = "coordination_error"
                service.registration_status["last_error"] = str(exc)
                LOGGER.warning(
                    "registration coordination failed closed; will retry: %s",
                    exc,
                )
            except RegistrationTlsError as exc:
                service.registration_status["state"] = "tls_error"
                service.registration_status["last_error"] = str(exc)
                LOGGER.warning("registration TLS setup failed; will retry: %s", exc)
            except (aiohttp.ClientError, OSError) as exc:
                service.registration_status["state"] = "connection_error"
                service.registration_status["last_error"] = f"{type(exc).__name__}: {exc}"
                LOGGER.warning("registration failed: %s", exc)
            await asyncio.sleep(interval)


def create_app(
    config: dict | None = None,
    *,
    allow_insecure_test_transport: bool = False,
) -> web.Application:
    """Create a loopback-only combined app for deterministic local tests.

    Production uses :func:`_serve` so the stock-Core MCP and TLS Capture
    listeners can never share a network socket.
    """

    if not allow_insecure_test_transport:
        raise CaptureTlsError(
            "the combined HTTP Capture app is test-only; production must use dedicated WSS"
        )
    effective = config or load_config()
    service = ShadowDriverService(effective)
    app = web.Application(client_max_size=2 * 1024 * 1024)
    app[SERVICE_KEY] = service
    app.router.add_post("/mcp", mcp_handler)
    app.router.add_post("/offer", offer_handler)
    app.router.add_get("/ws/teleop-capture", capture_websocket_handler)
    app.router.add_get("/health", health_handler)

    async def startup(_app: web.Application) -> None:
        if (
            effective.get("registration", {}).get("enabled", True)
            and service.runtime.status()["dispatch"]["ready"]
        ):
            service.registration_task = asyncio.create_task(
                _registration_loop(service), name="teleop-shadow-registration"
            )
        elif effective.get("registration", {}).get("enabled", True):
            service.registration_status["state"] = "dispatch_fault"
            service.registration_status["last_error"] = (
                "startup safe-stop was not acknowledged; registration inhibited"
            )

    async def cleanup(_app: web.Application) -> None:
        await service.close()

    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    return app


def create_mcp_app(service: ShadowDriverService) -> web.Application:
    app = web.Application(client_max_size=2 * 1024 * 1024)
    app[SERVICE_KEY] = service
    app.router.add_post("/mcp", mcp_handler)
    app.router.add_post("/offer", offer_handler)
    app.router.add_get("/health", health_handler)
    return app


def create_capture_app(service: ShadowDriverService) -> web.Application:
    app = web.Application(client_max_size=MAX_CAPTURE_MESSAGE_BYTES)
    app[SERVICE_KEY] = service
    app.router.add_get("/ws/teleop-capture", capture_websocket_handler)
    return app


async def _serve(config: dict) -> None:
    mcp_host = config.get("bind_host", "127.0.0.1")
    if not _is_loopback_bind(mcp_host):
        raise RuntimeError("MCP listener must bind a loopback address")
    capture_config = config.get("capture", {})
    capture_host = capture_config.get("bind_host", "0.0.0.0")
    capture_port = int(capture_config.get("port", 15712))
    mcp_port = int(config["mcp_port"])
    if capture_port == mcp_port:
        raise RuntimeError("Capture WSS and MCP HTTP must use different ports")
    capture_ssl = build_capture_ssl_context(capture_config)
    service = ShadowDriverService(config)
    mcp_runner = web.AppRunner(create_mcp_app(service))
    capture_runner = web.AppRunner(create_capture_app(service))
    try:
        await mcp_runner.setup()
        await capture_runner.setup()
        await web.TCPSite(mcp_runner, mcp_host, mcp_port).start()
        await web.TCPSite(
            capture_runner,
            capture_host,
            capture_port,
            ssl_context=capture_ssl,
        ).start()
        if (
            config.get("registration", {}).get("enabled", True)
            and service.runtime.status()["dispatch"]["ready"]
        ):
            service.registration_task = asyncio.create_task(
                _registration_loop(service),
                name="teleop-shadow-registration",
            )
        await asyncio.Event().wait()
    finally:
        await capture_runner.cleanup()
        await mcp_runner.cleanup()
        await service.close()


def main() -> None:
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
    config = load_config()
    try:
        asyncio.run(_serve(config))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
