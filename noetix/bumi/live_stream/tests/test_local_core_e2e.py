"""Opt-in Core Canvas -> Bumi Card -> real FFmpeg/librtmp -> local RTMP test.

All frames and credentials are synthetic. No robot, public endpoint, or secret.
"""
import asyncio
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from live_stream import LiveStreamPlugin
from live_stream.transport import IsolatedRTMPPublisher
from live_stream.worker import AudioBuffer, LiveStreamWorker

pytestmark = pytest.mark.skipif(
    os.getenv("BUMI_LOCAL_RTMP_TEST") != "1" or not os.getenv("PHANTHY_CORE_SRC"),
    reason="requires isolated local-RTMP test with Core source mounted",
)


def _free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class _Store:
    def __init__(self):
        self.url = None

    def read(self):
        return self.url

    def save(self, value):
        self.url = value


class _Camera:
    def __init__(self, frame):
        self.frame = frame
        self.sequence = 0

    def is_running(self):
        return True

    def wait_for_color_frame(self, **_):
        self.sequence += 1
        return {"data": self.frame}, self.sequence


def _jpeg_frame():
    return subprocess.check_output([
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "lavfi", "-i", "testsrc=size=160x120:rate=15",
        "-frames:v", "1", "-c:v", "mjpeg", "-f", "image2pipe", "pipe:1",
    ], timeout=12)


def _audio_source(buffer, stop):
    # 40ms 16kHz mono PCM per message, just like the Bumi microphone topic.
    pcm = struct.pack("<640h", *(int(5000 * math.sin(i * 2 * math.pi * 500 / 16000))
                                  for i in range(640)))
    while not stop.wait(0.04):
        buffer.push(pcm, "audio/pcm-16k")


def test_canvas_save_to_mock_bumi_and_real_local_rtmp(monkeypatch):
    sys.path.insert(0, os.environ["PHANTHY_CORE_SRC"])
    import config
    from api import canvas, mcp_manage
    from api.mcp_manage import MCPCallRequest

    port = _free_port()
    local_url = f"rtmp://127.0.0.1:{port}/live/test"
    receiver = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "warning", "-listen", "1",
        "-i", local_url, "-c", "copy", "-f", "null", "-",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    buffer = AudioBuffer()
    audio_stop = threading.Event()
    audio_thread = threading.Thread(target=_audio_source, args=(buffer, audio_stop), daemon=True)
    audio_thread.start()

    worker = LiveStreamWorker(_Camera(_jpeg_frame()), buffer, fps=15,
                              bitrate_kbps=300,
                              publisher_factory=lambda _: IsolatedRTMPPublisher(local_url))
    plugin = LiveStreamPlugin({}, "mock", None, None, store=_Store(),
                              audio=buffer, worker=worker)

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            if body["method"] == "initialize":
                result = {}
            elif body["method"] == "tools/call":
                args = body["params"].get("arguments") or {}
                answer = plugin.dispatch(args.get("action", ""), args)
                result = {"content": [{"type": "text", "text": json.dumps(answer)}]}
            else:
                result = {}
            payload = json.dumps({"jsonrpc": "2.0", "id": body.get("id"), "result": result}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *_):
            pass

    api = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    api_thread = threading.Thread(target=api.serve_forever, daemon=True)
    api_thread.start()

    mcp_id, card_id = "mcp-local-test", "card-test"
    cfg = {
        "services": {"mcp": [{
            "id": mcp_id, "url": f"http://127.0.0.1:{api.server_port}/mcp",
            "transport": "http", "tools": [plugin.get_tool()],
        }]},
        "canvas_layout": {"cards": [
            {"id": card_id, "mcpId": mcp_id, "toolName": "live_stream"}]},
    }
    monkeypatch.setattr(config, "main", cfg)
    monkeypatch.setattr(canvas, "_broadcast", lambda *_: None)
    demo_url = ("rtmp://live-push.xhscdn.com/live/123456789"
                "?txSecret=synthetic&txTime=ffffffff")

    try:
        time.sleep(0.5)
        saved = asyncio.run(canvas.save_instance_config(
            mcp_id, "live_stream", card_id, {"rtmp_url": demo_url}))
        assert saved["code"] == 200 and saved["applied"] is True
        assert saved["ready_to_go_live"] is True
        assert cfg[f"tool_config:{mcp_id}:live_stream:{card_id}"]["rtmp_url"] == demo_url

        # Actual Core HTTP MCP dispatch, including instance-config replay.
        result = asyncio.run(mcp_manage.mcp_call_tool(
            mcp_id, MCPCallRequest(tool="live_stream",
                arguments={"action": "go_live", "instance_id": card_id}), timeout_s=10))
        assert result["code"] == 200, result
        assert json.loads(result["data"][0]["text"])["state"] == "starting"

        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            status = plugin.dispatch("info", {})
            if status["state"] == "streaming" and status["published_records"] > 5:
                break
            if status["state"] == "error":
                pytest.fail(f"Stream failed: {status['error_code']}")
            time.sleep(0.12)
        else:
            pytest.fail(f"No local stream media: {plugin.dispatch('info', {})}")

        stopped = asyncio.run(mcp_manage.mcp_call_tool(
            mcp_id, MCPCallRequest(tool="live_stream",
                arguments={"action": "end_live", "instance_id": card_id}), timeout_s=10))
        assert stopped["code"] == 200, stopped

        # Same URL cannot silently rearm; rejection must be surfaced as failure.
        same = asyncio.run(canvas.save_instance_config(
            mcp_id, "live_stream", card_id, {"rtmp_url": demo_url}))
        assert same["code"] == 200 and same["ready_to_go_live"] is False
        denied = asyncio.run(mcp_manage.mcp_call_tool(
            mcp_id, MCPCallRequest(tool="live_stream",
                arguments={"action": "go_live", "instance_id": card_id}), timeout_s=10))
        assert denied["code"] == 400
        assert "APPROVAL_REQUIRED" in denied["message"]

        invalid = asyncio.run(canvas.save_instance_config(
            mcp_id, "live_stream", card_id, {"rtmp_url": "not-rtmp"}))
        assert invalid.status_code == 422
        assert cfg[f"tool_config:{mcp_id}:live_stream:{card_id}"]["rtmp_url"] == demo_url
    finally:
        audio_stop.set()
        audio_thread.join(timeout=2)
        worker.stop()
        api.shutdown()
        api.server_close()
        receiver.terminate()
        try:
            receiver.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            receiver.kill()
            receiver.communicate(timeout=3)
