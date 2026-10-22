"""Real HTTP/MCP transport with five cards, without a ROS or robot dependency."""
import importlib.util
import json
import sys
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))
from common.vendor_runtime import DriverBundle, make_handler
from deep_robotics.s10_pro.device import build_plugins


class NoRobot:
    def __init__(self):
        self.active = {}

    def acquire(self, owner, sources, options):
        self.active[owner] = sources

    def release(self, owner):
        self.active.pop(owner, None)

    def snapshot(self, source):
        return {}

    def status(self):
        return {"reader_running": False}


@pytest.fixture
def mcp():
    bridge, published = NoRobot(), []
    bundle = DriverBundle(build_plugins({}, "contract", bridge, lambda *args: published.append(args)))
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "s10-test", "s10-test"))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def call(method, params=None):
        request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
            json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)
    yield call, bridge, published
    bundle.stop_all()
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_http_tools_are_only_five_idle_sensors_with_fixed_ports(mcp):
    call, bridge, published = mcp
    assert call("initialize")["result"]["serverInfo"]["name"] == "s10-test"
    tools = call("tools/list")["result"]["tools"]
    assert {t["name"] for t in tools} == {"state", "joints", "imu", "odometry", "lidar"}
    assert all(t["type"] == "sensor" for t in tools)
    assert not bridge.active and not published
    for tool in tools:
        rpc = call("tools/call", {"name": tool["name"], "arguments": {"action": "info"}})
        content = rpc["result"]["content"]
        assert len(content) == 1 and content[0]["type"] == "text"
        info = json.loads(content[0]["text"])
        assert isinstance(info, dict) and info["state"] == "idle"
        assert info["topic_out"] == tool["topic_out"]
        assert not info["connected"]


def test_canvas_style_start_config_stop_and_errors(mcp):
    call, bridge, published = mcp
    def action(name, action, **options):
        response = call("tools/call", {"name": name, "arguments": {"action": action, **options}})
        if "error" in response:
            return response
        return json.loads(response["result"]["content"][0]["text"])
    assert action("imu", "config", refresh_hz=2)["settings"]["refresh_hz"] == 2
    assert action("imu", "start", instance_id="card-123")["state"] == "running"
    assert action("joints", "start")["state"] == "running"
    assert action("imu", "stop")["state"] == "idle"
    assert set(bridge.active) == {"joints"}
    assert action("joints", "stop")["state"] == "idle"
    assert not bridge.active
    assert "error" in action("motion", "stand")
    assert "error" in action("imu", "config", refresh_hz=float("inf"))


def test_profile_validation_rejects_unconfined_transport(tmp_path):
    # main has lazy ROS imports, so this check does not create participants.
    path = ROOT / "deep_robotics/s10_pro/main.py"
    spec = importlib.util.spec_from_file_location("s10_main_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profile = tmp_path / "dds.xml"
    profile.write_text('''<dds><profiles><transport_descriptors><transport_descriptor>
      <transport_id>local</transport_id><type>UDPv4</type>
      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
      </transport_descriptor></transport_descriptors>
      <participant is_default_profile="true"><rtps><userTransports>
      <transport_id>local</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports>
      </rtps></participant></profiles></dds>''')
    module.validate_platform_profile(profile)
    profile.write_text(profile.read_text().replace("127.0.0.1", "0.0.0.0"))
    with pytest.raises(ValueError):
        module.validate_platform_profile(profile)
