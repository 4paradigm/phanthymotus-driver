"""Direction RPC bounds, image association and cancellation; no robot/model."""
import importlib
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import pytest


@pytest.fixture
def mod(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "noetix" / "bumi"))
    return importlib.import_module("planar_orientation")


def page(word):
    image = np.full((300, 220, 3), 230, np.uint8)
    cv2.putText(image, word, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    return image


def wait_for(condition):
    end = time.monotonic() + 2
    while not condition():
        assert time.monotonic() < end
        time.sleep(0.01)


def test_direction_is_async_and_old_page_reply_is_discarded(mod, monkeypatch):
    client = mod.OCROrientation(interval=0.1)
    entered, release = threading.Event(), threading.Event()
    calls = []
    def rpc(data):
        calls.append(data)
        entered.set()
        assert release.wait(2)
        return {"status": "ready", "rotation_cw_degrees": 180, "support": 1.0}
    monkeypatch.setattr(client, "_request", rpc)
    client.start()
    try:
        assert client.consider(page("HELLO"), 1)["status"] == "unknown"
        assert entered.wait(1)
        client.invalidate()
        release.set()
        time.sleep(0.05)
        assert client._result["status"] == "unknown"
        client.consider(page("NEXT"), 2)
        wait_for(lambda: client._result["status"] == "ready")
        result = client.consider(page("NEXT"), 3)
        assert result["rotation_cw_degrees"] == 180
        assert result["based_on_frame_sequence"] == 2
        with client._lock:
            client._result["received_monotonic_s"] -= 5
        assert client.consider(page("NEXT"), 4)["status"] == "unknown"
    finally:
        release.set()
        client.stop()
    assert client._thread is None and client._pending is None


def test_missing_ocr_is_rate_limited_and_never_blocks_geometry(mod, monkeypatch):
    client = mod.OCROrientation()
    calls = []
    monkeypatch.setattr(client, "_request", lambda data:
                        calls.append(data) or client._unknown("unsupported_ocr_api"))
    client.start()
    try:
        image = page("HELLO")
        client.consider(image, 1)
        wait_for(lambda: client._result.get("reason") == "unsupported_ocr_api")
        for n in range(2, 30):
            result = client.consider(image, n)
        assert len(calls) == 1 and result["rotation_cw_degrees"] is None
    finally:
        client.stop()


def test_redirect_is_rejected(mod):
    with pytest.raises(mod.urllib.error.URLError):
        mod._NoRedirect().redirect_request(None, None, 302, "Found", {}, "http://example.com")


@pytest.mark.parametrize("payload,expected", [
    ({"ok": True, "schema": "motus.document-orientation/1", "status": "ready",
      "rotation_cw_degrees": 270, "support": 1.0}, "ready"),
    ({"ok": True, "schema": "motus.document-orientation/1", "status": "unknown",
      "reason": "no_text"}, "unknown"),
    ({"ok": True, "schema": "motus.document-orientation/1", "status": "ready",
      "rotation_cw_degrees": True, "support": 1.0}, "unknown"),
    ({"ok": True, "schema": "motus.document-orientation/1", "status": "ready",
      "rotation_cw_degrees": 90, "support": float("nan")}, "unknown"),
    ({"ok": False, "reason": "adapter_loading"}, "unknown"),
])
def test_loopback_rpc_contract_without_files_models_or_proxy(mod, monkeypatch, payload, expected):
    seen = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            assert self.path == "/mcp" and body["params"]["name"] == "ocr"
            args = body["params"]["arguments"]
            assert args["action"] == "classify_orientation"
            seen.append(base64.b64decode(args["image_base64"], validate=True))
            reply = json.dumps({"jsonrpc": "2.0", "id": 1, "result": {
                "content": [{"type": "text", "text": json.dumps(payload)}]}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:1")
    try:
        client = mod.OCROrientation(port=server.server_port)
        result = client._request(b"\xff\xd8test-bounded-JPEG")
        assert result["status"] == expected and len(seen) == 1
        if expected == "ready":
            assert result["rotation_cw_degrees"] == 270
        else:
            assert result["rotation_cw_degrees"] is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
