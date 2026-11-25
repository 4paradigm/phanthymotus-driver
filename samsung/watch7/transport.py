"""Watch7-only authenticated HTTP ingress (no MCP endpoint exposed to Wi-Fi).

Unsupported control paths return empty queues or explicit denials. The companion
App can continue to upload telemetry without thinking there are pending actions.
"""
from __future__ import annotations

import hmac
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY_BYTES = 8191


def make_device_handler(device, token: str):
    if not isinstance(token, str) or len(token) < 16:
        raise ValueError("WATCH7_DEVICE_TOKEN must be at least 16 characters")

    class Handler(BaseHTTPRequestHandler):
        server_version = "watch7-driver/0.1"
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):
            # Never log URLs with tokens or health samples.
            return

        def _reply(self, obj: dict, status: int = 200) -> None:
            raw = json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _auth(self) -> bool:
            received = self.headers.get("X-Wearable-Token", "")
            if not hmac.compare_digest(received, token):
                self._reply({"ok": False, "reason": "unauthorized"}, 401)
                return False
            return True

        def do_GET(self):
            if self.path == "/health":
                return self._reply({"ok": True, "driver": "samsung-watch7"})
            if self.path not in ("/v1/commands", "/v2/commands", "/v2/task-prompts"):
                return self._reply({"ok": False, "reason": "not_found"}, 404)
            if not self._auth():
                return
            if self.path == "/v2/task-prompts":
                return self._reply({"ok": True, "prompts": []})
            return self._reply({
                "ok": True, "commands": [], "server_now_ms": int(time.time() * 1000),
            })

        def do_POST(self):
            if self.path not in (
                "/v1/telemetry", "/v2/telemetry", "/v1/ack",
                "/v2/ack", "/v2/task-response", "/v2/haptic-authorize",
            ):
                return self._reply({"ok": False, "reason": "not_found"}, 404)
            if not self._auth():
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size <= 0 or size > MAX_BODY_BYTES:
                    return self._reply({"ok": False, "reason": "invalid_body_length"}, 413)
                payload = json.loads(self.rfile.read(size))
            except (ValueError, TypeError, UnicodeError):
                return self._reply({"ok": False, "reason": "bad_input"}, 400)
            if self.path == "/v1/telemetry":
                result = device.ingest_v1(payload)
            elif self.path == "/v2/telemetry":
                result = device.ingest_v2(payload)
            elif self.path in ("/v1/ack", "/v2/ack"):
                result = {"ok": False, "reason": "not_pending"}
            elif self.path == "/v2/task-response":
                result = {"ok": False, "reason": "unknown_task"}
            else:
                result = {"ok": True, "allowed": False, "reason": "commands_disabled"}
            return self._reply(result, 200 if result.get("ok") else 422)

    return Handler


def make_device_server(host: str, port: int, device, token: str) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((host, port), make_device_handler(device, token))
