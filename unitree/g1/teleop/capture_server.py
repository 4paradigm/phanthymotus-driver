"""Dedicated TLS WebSocket listener for G1 Capture control."""

from __future__ import annotations

import asyncio
import base64
import ipaddress
import json
import ssl
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from aiohttp import web
from cryptography import x509

from .capture import (
    MAX_CAPTURE_CA_PEM_BYTES,
    MAX_CAPTURE_MESSAGE_BYTES,
    CaptureConnection,
    CaptureError,
    CaptureManager,
)


class CaptureTlsError(RuntimeError):
    """Capture listener cannot establish its authenticated TLS boundary."""


def validated_capture_wss_url(value: Any, *, expected_port: int | None = None) -> str:
    if not isinstance(value, str) or not value or len(value) > 2048:
        raise CaptureTlsError("Capture public URL must be a bounded wss:// URL")
    parsed = urlsplit(value)
    try:
        parsed_port = parsed.port
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
        or parsed_port is None
        or (expected_port is not None and parsed_port != expected_port)
    ):
        raise CaptureTlsError(
            "Capture public URL must be wss://<host>:<port>/ws/teleop-capture"
        )
    return value


def capture_certificate_base64(config: dict) -> str:
    certificate_path = Path(str(config.get(
        "tls_cert_file",
        "/etc/motus-g1-capture-tls/cert.pem",
    )))
    if not certificate_path.is_file() or certificate_path.is_symlink():
        raise CaptureTlsError(f"Capture TLS certificate is missing: {certificate_path}")
    try:
        with certificate_path.open("rb") as certificate_file:
            certificate = certificate_file.read(MAX_CAPTURE_CA_PEM_BYTES + 1)
    except OSError as exc:
        raise CaptureTlsError(
            f"Capture TLS certificate is unreadable: {certificate_path}"
        ) from exc
    if (
        not certificate
        or len(certificate) > MAX_CAPTURE_CA_PEM_BYTES
        or b"-----BEGIN CERTIFICATE-----" not in certificate
        or b"PRIVATE KEY" in certificate
    ):
        raise CaptureTlsError(
            "Capture TLS certificate must contain only a bounded public PEM chain"
        )
    return base64.b64encode(certificate).decode("ascii")


def build_capture_ssl_context(config: dict) -> ssl.SSLContext:
    """Validate SAN and load the independent Capture certificate/key pair."""

    port = int(config.get("port", 15702))
    public_wss_url = validated_capture_wss_url(
        config.get("public_wss_url"),
        expected_port=port,
    )
    capture_certificate_base64(config)
    certificate_path = Path(str(config.get(
        "tls_cert_file",
        "/etc/motus-g1-capture-tls/cert.pem",
    )))
    key_path = Path(str(config.get(
        "tls_key_file",
        "/etc/motus-g1-capture-tls/key.pem",
    )))
    forbidden_core_roots = (
        Path("/etc/motus-core-certs"),
        Path("/opt/phanthy-motus/data/certs"),
    )
    for candidate in (certificate_path, key_path):
        for core_key_root in forbidden_core_roots:
            try:
                candidate.resolve(strict=False).relative_to(core_key_root)
            except ValueError:
                continue
            raise CaptureTlsError(
                "Capture TLS material must not reuse the Agent Core certificate directory"
            )
    if not key_path.is_file() or key_path.is_symlink():
        raise CaptureTlsError(f"Capture TLS private key is missing: {key_path}")
    if certificate_path.resolve() == key_path.resolve():
        raise CaptureTlsError("Capture TLS certificate and private key must be separate files")
    try:
        certificate = x509.load_pem_x509_certificate(certificate_path.read_bytes())
        alternatives = certificate.extensions.get_extension_for_class(
            x509.SubjectAlternativeName
        ).value
    except (OSError, ValueError, x509.ExtensionNotFound) as exc:
        raise CaptureTlsError(
            "Capture TLS certificate must contain a valid SAN extension"
        ) from exc
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
            "Capture TLS certificate SAN does not match capture.public_wss_url host"
        )
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    try:
        context.load_cert_chain(str(certificate_path), str(key_path))
    except (OSError, ssl.SSLError) as exc:
        raise CaptureTlsError(
            "Capture TLS certificate/key pair is invalid"
        ) from exc
    return context


class _DuplicateJsonField(ValueError):
    pass


def capture_json(raw: str) -> dict:
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


async def _ws_text(
    websocket: web.WebSocketResponse,
    *,
    timeout: float | None = None,
) -> str:
    try:
        message = await websocket.receive(timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise CaptureError("capture_auth_timeout", status=408) from exc
    if message.type != web.WSMsgType.TEXT or not isinstance(message.data, str):
        if message.type in {
            web.WSMsgType.CLOSE,
            web.WSMsgType.CLOSING,
            web.WSMsgType.CLOSED,
        }:
            raise EOFError
        raise CaptureError("capture_message_invalid")
    return message.data


CAPTURE_KEY = web.AppKey("g1_capture_manager", CaptureManager)


async def capture_websocket_handler(request: web.Request) -> web.StreamResponse:
    manager = request.app[CAPTURE_KEY]
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
        first = capture_json(await _ws_text(websocket, timeout=5.0))
        connection, acknowledgement = await manager.connect(first)
        await websocket.send_json(acknowledgement)
        receive_task = asyncio.create_task(_ws_text(websocket))
        event_task = asyncio.create_task(connection.events.get())
        while True:
            done, _ = await asyncio.wait(
                {receive_task, event_task},
                timeout=0.25,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if not done:
                if manager.presence_expired(connection):
                    manager.disconnect_immediate(connection)
                    await websocket.send_json({
                        "type": "error",
                        "code": "capture_presence_timeout",
                    })
                    await websocket.close(
                        code=4408,
                        message=b"capture_presence_timeout",
                    )
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
                incoming = capture_json(receive_task.result())
                message_type = incoming.get("type")
                if message_type == "presence":
                    response = await manager.presence(connection, incoming)
                elif message_type == "signaling_offer":
                    response = await manager.signaling_offer(connection, incoming)
                else:
                    raise CaptureError("capture_message_invalid")
                await websocket.send_json(response)
                receive_task = asyncio.create_task(_ws_text(websocket))
    except EOFError:
        pass
    except asyncio.CancelledError:
        raise
    except CaptureError as exc:
        if connection is not None:
            manager.disconnect_immediate(connection)
        if not websocket.closed:
            await websocket.send_json({"type": "error", "code": exc.code})
            await websocket.close(
                code=4403 if exc.status in {401, 403} else 4400,
                message=exc.code.encode("ascii", errors="ignore")[:120],
            )
    finally:
        if connection is not None:
            manager.disconnect_immediate(connection)
        tasks = [
            task
            for task in (receive_task, event_task)
            if task is not None
        ]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        if connection is not None:
            await manager.disconnect(connection)
    return websocket


def create_capture_app(manager: CaptureManager) -> web.Application:
    app = web.Application(client_max_size=MAX_CAPTURE_MESSAGE_BYTES)
    app[CAPTURE_KEY] = manager
    app.router.add_get("/ws/teleop-capture", capture_websocket_handler)
    return app


class CaptureWssServer:
    """Lifecycle wrapper started on the teleop service's asyncio loop."""

    def __init__(self, manager: CaptureManager, config: dict):
        self._manager = manager
        self._config = dict(config)
        self._ssl_context = build_capture_ssl_context(self._config)
        self._runner: web.AppRunner | None = None
        self._site: web.TCPSite | None = None

    async def start(self) -> None:
        if self._runner is not None:
            return
        runner = web.AppRunner(create_capture_app(self._manager))
        await runner.setup()
        try:
            site = web.TCPSite(
                runner,
                str(self._config.get("bind_host", "0.0.0.0")),
                int(self._config.get("port", 15702)),
                ssl_context=self._ssl_context,
            )
            await site.start()
        except Exception:
            await runner.cleanup()
            raise
        self._runner = runner
        self._site = site

    async def close(self) -> None:
        runner = self._runner
        self._runner = None
        self._site = None
        if runner is not None:
            await runner.cleanup()


__all__ = [
    "CAPTURE_KEY",
    "CaptureTlsError",
    "CaptureWssServer",
    "build_capture_ssl_context",
    "capture_certificate_base64",
    "capture_json",
    "capture_websocket_handler",
    "create_capture_app",
    "validated_capture_wss_url",
]
