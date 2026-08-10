"""Deployable MCP + WebRTC server for the generic teleop shadow Driver."""

from __future__ import annotations

import asyncio
import ipaddress
import json
import logging
import os
import re
import ssl
from pathlib import Path
from typing import Any

import aiohttp
import yaml
from aiohttp import web
from protocol import (
    CAPABILITY_DIGEST,
    DISPATCH_CONTRACT,
    PROTOCOL,
    SIGNALING_PROTOCOL,
    ProtocolError,
    TicketCodec,
    TicketVerifier,
)
from rtc import RtcManager, RtcRequestError
from runtime import ShadowRuntime

LOGGER = logging.getLogger("teleop-shadow")
DEFAULT_DRIVER_ID = "teleop-shadow-driver"
DEFAULT_DRIVER_NAME = "Generic Teleop Shadow Diagnostics"
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.yaml")
_INSTANCE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_DRIVER_BEARER_RE = re.compile(r"^[A-Za-z0-9._~+/=-]{24,4096}$")


class RegistrationTlsError(RuntimeError):
    """Agent Core registration cannot establish the required pinned TLS trust."""


def _is_loopback_bind(host: object) -> bool:
    if not isinstance(host, str):
        return False
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host.lower() == "localhost"


def _identity_properties() -> dict:
    return {
        "boot_id": {"type": "string", "format": "uuid"},
        "session_id": {"type": "string", "format": "uuid"},
        "epoch": {"type": "integer", "minimum": 1},
        "fence": {"type": "string", "minLength": 24},
    }


def tool_definitions(
    *,
    driver_id: str = DEFAULT_DRIVER_ID,
    driver_name: str = DEFAULT_DRIVER_NAME,
    robot_id: str | None = None,
    signaling_enabled: bool = True,
) -> list[dict]:
    identity = _identity_properties()
    actions = {
        "start": {"params": [], "description": "Passive lifecycle readiness check; never prepares a session"},
        "stop": {"params": [], "description": "Stop lifecycle and safely release the Shadow session"},
        "prepare_shadow": {
            "params": ["session_id", "epoch", "fence"],
            "description": "Install a new Core-issued epoch/fence for a Shadow session",
        },
        "heartbeat": {"params": list(identity), "description": "Renew the lease from authenticated Agent Core MCP only"},
        "pause": {"params": list(identity), "description": "Pause diagnostics and enter a non-resuming state"},
        "release": {"params": list(identity), "description": "Release the current fenced session"},
        "soft_stop": {"params": list(identity), "description": "Enter HOLD; no hardware action exists"},
        "status": {"params": [], "description": "Read the current session and transport diagnostics"},
        "submit_shadow_frame": {
            "params": ["frame"],
            "description": "Submit one strict Frame v1 for diagnostics/replay; not a production high-rate path",
        },
    }
    signaling = ({
        "signaling": {
            "protocol": SIGNALING_PROTOCOL,
            "path": "/offer",
            "access": "authenticated-core-proxy-only",
        },
    } if signaling_enabled else {})
    return [
        {
            "name": "teleop_session",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Robot-free Quest/WebRTC Shadow diagnostics with a bounded would-apply/would-stop "
                "final-dispatch trace. It can never actuate hardware; only authenticated MCP "
                "heartbeat renews the lease."
            ),
            "annotations": {"destructiveHint": False, "idempotentHint": False},
            "inputSchema": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "action": {"type": "string", "enum": list(actions)},
                    **identity,
                    "frame": {"type": "object", "description": "Strict Teleop Frame v1 object"},
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
            "description": "Callable read-only snapshot of Shadow session, lease, Pose, RTC and rejection counters",
            "annotations": {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True},
            "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
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
        registration.get("ca_file", "/etc/motus-core-certs/cert.pem"),
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

    ca_file = Path(str(registration.get("ca_file", "/etc/motus-core-certs/cert.pem")))
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
        raw_driver_token = os.environ.get("MOTUS_DRIVER_TOKEN")
        if raw_driver_token and not _DRIVER_BEARER_RE.fullmatch(raw_driver_token):
            raise ValueError(
                "MOTUS_DRIVER_TOKEN must use 24–4096 restricted ASCII Bearer "
                "characters (allowed: A-Z a-z 0-9 . _ ~ + / = -)"
            )
        self.driver_token = raw_driver_token or None
        if self.driver_token is None and (
            registration_enabled
            or not _is_loopback_bind(config.get("bind_host", "127.0.0.1"))
        ):
            raise ValueError(
                "MOTUS_DRIVER_TOKEN is required for registration or non-loopback binding"
            )
        teleop = config.get("teleop", {})
        ticket_secret = os.environ.get("MOTUS_TELEOP_TICKET_SECRET")
        if ticket_secret is None:
            if registration_enabled or not _is_loopback_bind(
                config.get("bind_host", "127.0.0.1")
            ):
                raise ValueError(
                    "MOTUS_TELEOP_TICKET_SECRET is required for registration "
                    "or non-loopback binding"
                )
            ticket_codec = None
        else:
            # A configured empty or malformed value is an operator error, not
            # a request to silently downgrade signaling readiness.
            ticket_codec = TicketCodec(ticket_secret)
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
        if ticket_codec is None:
            verifier = None
            LOGGER.warning("RTC disabled: MOTUS_TELEOP_TICKET_SECRET is not configured")
        else:
            verifier = TicketVerifier(
                ticket_codec,
                max_ttl_seconds=int(teleop.get("ticket_ttl_max_seconds", 30)),
                max_replay_entries=int(teleop.get("ticket_replay_cache_entries", 4096)),
            )
        self.rtc = RtcManager(self.runtime, verifier)
        self.config = config
        self.registration_task: asyncio.Task | None = None
        self.registration_status = {
            "state": "starting" if registration_enabled else "disabled",
            "attempts": 0,
            "successes": 0,
            "last_http_status": None,
            "last_error": None,
            "tls_verification": (
                "pinned_certificate"
                if bool(registration.get("verify_tls", True))
                else "disabled_local_development"
            ),
        }

    def authorized(self, request: web.Request) -> bool:
        if not self.driver_token:
            return True
        header = request.headers.get("Authorization", "")
        if not header.startswith("Bearer "):
            return False
        import hmac

        return hmac.compare_digest(header[7:], self.driver_token)

    async def dispatch_tool(self, name: str, arguments: Any) -> dict:
        if not isinstance(arguments, dict):
            raise ProtocolError("invalid_arguments", "tool arguments must be an object")
        args = dict(arguments)
        if name == "teleop_state":
            if args:
                raise ProtocolError("invalid_arguments", "teleop_state accepts no arguments")
            return self.runtime.status()
        if name != "teleop_session":
            raise ProtocolError("unknown_tool", f"unknown tool: {name}")
        action = args.pop("action", None)
        if not isinstance(action, str):
            raise ProtocolError("missing_action", "teleop_session requires an action")
        if action == "start":
            if args:
                raise ProtocolError("invalid_arguments", "start accepts no arguments")
            result = self.runtime.status()
            result["lifecycle_state"] = (
                "ready" if result["dispatch"]["ready"] else "fault"
            )
            return result
        if action == "stop":
            if args:
                raise ProtocolError("invalid_arguments", "stop accepts no arguments")
            result = await asyncio.to_thread(self.runtime.release, lifecycle=True)
            await self.rtc.close_all()
            return result
        if action == "prepare_shadow":
            if set(args) != {"session_id", "epoch", "fence"}:
                raise ProtocolError(
                    "invalid_arguments", "prepare_shadow requires session_id, epoch and fence"
                )
            result = await asyncio.to_thread(self.runtime.prepare_shadow, args)
            # Stale/invalid prepare requests are rejected before touching the
            # active RTC peer.  Successful prepare advances the generation, so
            # callbacks from the old peer are diagnostics-only while it closes.
            await self.rtc.close_all()
            return result
        if action == "heartbeat":
            self._require_only_identity(args)
            return self.runtime.heartbeat(args)
        if action == "pause":
            self._require_only_identity(args)
            result = await asyncio.to_thread(self.runtime.pause, args)
            await self.rtc.close_all()
            return result
        if action == "release":
            self._require_only_identity(args)
            result = await asyncio.to_thread(self.runtime.release, args)
            await self.rtc.close_all()
            return result
        if action == "soft_stop":
            self._require_only_identity(args)
            result = await asyncio.to_thread(self.runtime.soft_stop, args)
            await self.rtc.close_all()
            return result
        if action == "status":
            if args:
                raise ProtocolError("invalid_arguments", "status accepts no arguments")
            return self.runtime.status()
        if action == "submit_shadow_frame":
            if set(args) != {"frame"}:
                raise ProtocolError("invalid_arguments", "submit_shadow_frame requires only frame")
            return self.runtime.submit_shadow_frame(args["frame"], source="mcp_diagnostic")
        raise ProtocolError("unknown_action", f"unknown teleop_session action: {action}")

    @staticmethod
    def _require_only_identity(args: dict) -> None:
        expected = {"boot_id", "session_id", "epoch", "fence"}
        if set(args) != expected:
            raise ProtocolError("invalid_arguments", f"action requires exactly {sorted(expected)}")

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
    if not service.authorized(request):
        return web.json_response({"error": "unauthorized"}, status=401)
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
                "serverInfo": {"name": "teleop-shadow-driver", "version": "1.2.0"},
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
    service = request.app[SERVICE_KEY]
    if not service.authorized(request):
        return web.json_response({"error": "unauthorized"}, status=401)
    try:
        payload = await request.json()
        return web.json_response(await service.rtc.accept_offer(payload))
    except (json.JSONDecodeError, ValueError, RecursionError):
        return web.json_response({"error": {"code": "invalid_json", "message": "body must be JSON"}}, status=400)
    except RtcRequestError as exc:
        return web.json_response({"error": {"code": exc.code, "message": str(exc)}}, status=exc.status)


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
        "mcp_auth_enabled": bool(service.driver_token),
        "registration": dict(service.registration_status),
    })


def registration_payload(service: ShadowDriverService, registration: dict | None = None) -> dict:
    registration = registration or service.config.get("registration", {})
    payload = {
        "id": service.driver_id,
        "name": service.driver_name,
        "url": registration.get("advertise_url", "http://localhost:15711/mcp"),
        "transport": "http",
        "category": "driver",
    }
    if service.robot_id is not None:
        payload["robot_id"] = service.robot_id
    return payload


def registration_headers(service: ShadowDriverService) -> dict[str, str]:
    headers = {"Content-Type": "application/json"}
    if service.driver_token:
        headers["Authorization"] = f"Bearer {service.driver_token}"
    return headers


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
                async with session.post(endpoint, json=payload, headers=headers, ssl=ssl_context) as response:
                    service.registration_status["last_http_status"] = response.status
                    if response.status >= 400:
                        service.registration_status["state"] = "http_error"
                        service.registration_status["last_error"] = f"Agent Core returned HTTP {response.status}"
                        LOGGER.warning("registration failed with HTTP %s", response.status)
                    else:
                        service.registration_status["state"] = "registered"
                        service.registration_status["successes"] += 1
                        service.registration_status["last_error"] = None
            except asyncio.CancelledError:
                raise
            except RegistrationTlsError as exc:
                service.registration_status["state"] = "tls_error"
                service.registration_status["last_error"] = str(exc)
                LOGGER.warning("registration TLS setup failed; will retry: %s", exc)
            except (aiohttp.ClientError, OSError) as exc:
                service.registration_status["state"] = "connection_error"
                service.registration_status["last_error"] = f"{type(exc).__name__}: {exc}"
                LOGGER.warning("registration failed: %s", exc)
            await asyncio.sleep(interval)


def create_app(config: dict | None = None) -> web.Application:
    effective = config or load_config()
    service = ShadowDriverService(effective)
    app = web.Application(client_max_size=2 * 1024 * 1024)
    app[SERVICE_KEY] = service
    app.router.add_post("/mcp", mcp_handler)
    app.router.add_post("/offer", offer_handler)
    app.router.add_get("/health", health_handler)

    async def startup(_app: web.Application) -> None:
        if (
            effective.get("registration", {}).get("enabled", True)
            and service.runtime.status()["dispatch"]["ready"]
            and service.rtc.enabled
        ):
            service.registration_task = asyncio.create_task(
                _registration_loop(service), name="teleop-shadow-registration"
            )
        elif effective.get("registration", {}).get("enabled", True):
            if not service.rtc.enabled:
                service.registration_status["state"] = "rtc_unavailable"
                service.registration_status["last_error"] = (
                    "ticket verification is not ready; registration inhibited"
                )
            else:
                service.registration_status["state"] = "dispatch_fault"
                service.registration_status["last_error"] = (
                    "startup safe-stop was not acknowledged; registration inhibited"
                )

    async def cleanup(_app: web.Application) -> None:
        await service.close()

    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    return app


def main() -> None:
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
    config = load_config()
    web.run_app(create_app(config), host=config["bind_host"], port=int(config["mcp_port"]))


if __name__ == "__main__":
    main()
