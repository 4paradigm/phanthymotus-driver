"""Exercise Tianyi's real MCP HTTP handler without importing ROS or the SDK."""
import ast
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest


@pytest.fixture
def call_tool():
    source = (Path(__file__).resolve().parents[1]
              / "x-humanoid/tianyi2.0/main.py").read_text()
    factory = next(node for node in ast.parse(source).body
                   if isinstance(node, ast.FunctionDef) and node.name == "make_handler")
    bundle = SimpleNamespace(dispatch=lambda name, args: bundle.reply)
    scope = {"BaseHTTPRequestHandler": BaseHTTPRequestHandler,
             "json": json, "_bundle": bundle}
    exec(compile(ast.Module(body=[factory], type_ignores=[]), "main.py", "exec"), scope)
    server = ThreadingHTTPServer(("127.0.0.1", 0), scope["make_handler"]())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def call(reply):
        bundle.reply = reply
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        try:
            conn.request("POST", "/mcp", json.dumps({
                "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                "params": {"name": "camera_wrist_left_rgb", "arguments": {"action": "start"}},
            }), {"Content-Type": "application/json"})
            response = conn.getresponse()
            assert response.status == 200
            return json.loads(response.read())["result"]
        finally:
            conn.close()

    yield call
    server.shutdown()
    server.server_close()
    thread.join(timeout=5)


@pytest.mark.parametrize("reply,is_error", [
    ({"state": "loading", "error": None}, False),
    ({"state": "starting", "error": None}, False),
    ({"state": "running", "error": None}, False),
    ({"state": "idle", "error": ""}, False),
    ({"state": "running"}, False),
    ({"state": "error", "error": None}, True),
    ({"error": "D405 disconnected"}, True),
    ({"state": "running", "error": "Capture failed"}, True),
])
def test_mcp_error_flag_requires_an_actual_error(call_tool, reply, is_error):
    result = call_tool(reply)
    assert bool(result.get("isError")) is is_error
    assert json.loads(result["content"][0]["text"]) == reply
