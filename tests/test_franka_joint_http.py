"""Real MCP HTTP envelope, with fake robot sessions and no robot/network I/O."""
import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
import test_franka_joint_control as fixtures
from common.vendor_runtime import make_handler


@pytest.fixture
def rpc():
    fixture = fixtures.JointControlTests(methodName="runTest")
    fixture.setUp()
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
        lambda: fixture.bundle, "franka-fr3", "franka-fr3-driver"))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()

    def request(method, params=None):
        body = json.dumps(dict(jsonrpc="2.0", id=1, method=method,
                               params=params or {})).encode()
        req = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                                     data=body, headers={"Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req, timeout=5))
    try:
        yield request, fixture
    finally:
        server.shutdown()
        server.server_close()
        worker.join(2)
        fixture.doCleanups()


def invoke(rpc, action, **args):
    response = rpc("tools/call", {"name": "joint_control", "arguments": {
        "action": action, "instance_id": "http_card", **args}})
    assert "error" not in response, response
    result = response["result"]
    assert not result.get("isError"), result
    return json.loads(result["content"][0]["text"])


def test_mcp_schema_lifecycle_and_confirmed_stop(rpc):
    request, fixture = rpc
    assert request("initialize")["result"]["serverInfo"]["name"] == "franka-fr3"
    tool = next(t for t in request("tools/list")["result"]["tools"]
                if t["name"] == "joint_control")
    assert tool["type"] == "actuator"
    assert "robot_ip" not in tool["configSchema"].get("required", [])
    assert not fixture.sessions
    invoke(request, "config", robot_ip="172.16.0.3")
    assert not fixture.sessions
    invoke(request, "start")
    result = invoke(request, "jog", joint=1, step_deg=1)
    assert result["action_id"]
    assert invoke(request, "stopmotion")["state"] == "ready"
    assert invoke(request, "stop")["state"] == "idle"


def test_invalid_motion_is_mcp_error_without_submission(rpc):
    request, fixture = rpc
    invoke(request, "config", robot_ip="172.16.0.3")
    invoke(request, "start")
    response = request("tools/call", {"name": "joint_control", "arguments": {
        "action": "set", "instance_id": "http_card", "joint1_deg": 100,
        "speed_deg_s": 0.1}})
    assert "error" in response or response.get("result", {}).get("isError"), response
    assert fixture.sessions[0].moves == []
