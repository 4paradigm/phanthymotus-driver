"""Exercise configured Go1 power cards through the real HTTP MCP handler."""

import json
import os
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request, build_opener, ProxyHandler

GO1 = Path(__file__).resolve().parents[1] / "unitree" / "go1"


def _check_http_mcp():
    # Isolate the driver's process-wide logging setup from pytest's streams.
    sys.path.insert(0, str(GO1))
    import main
    import yaml

    class StubClient:
        def snapshot(self):
            return {"fresh": False}

    with (GO1 / "config.yaml").open() as stream:
        config = yaml.safe_load(stream)
    names = {"battery_power", "joint_power"}
    config["plugins"] = {name: config["plugins"][name] for name in names}
    assert all(card["enabled"] for card in config["plugins"].values())
    bundle = main.Go1Bundle(config, "power_test", None, StubClient())
    main._bundle = bundle
    # An ephemeral loopback port keeps this test independent of running drivers.
    server = ThreadingHTTPServer(("127.0.0.1", 0), main.make_handler())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    opener = build_opener(ProxyHandler({}))

    def rpc(method, params=None):
        request = Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1,
                             "method": method, "params": params or {}}).encode(),
            headers={"Content-Type": "application/json"},
        )
        with opener.open(request, timeout=3) as response:
            payload = json.load(response)
        assert payload["jsonrpc"] == "2.0" and payload["id"] == 1
        assert "error" not in payload, payload
        return payload["result"]

    def call(name, action):
        content = rpc("tools/call", {"name": name, "arguments": {"action": action}})["content"]
        assert len(content) == 1 and content[0]["type"] == "text"
        result = json.loads(content[0]["text"])
        assert isinstance(result, dict) and "content" not in result
        return result

    try:
        thread.start()
        bundle.start_all()
        assert rpc("initialize")["capabilities"] == {"tools": {}}
        tools = rpc("tools/list")["tools"]
        assert {tool["name"] for tool in tools} == names
        assert all(tool["type"] == "sensor" for tool in tools)
        for name in names:
            assert call(name, "start") == {"state": "running"}
            data = call(name, "info")["data"]
            assert {"timestamp_ms", "control_level", "fresh"} <= data.keys()
            assert data["fresh"] is False and data["available"] is False
            if name == "battery_power":
                assert data["discharged_since_start_wh"] == 0
                assert data["remaining_runtime_minutes"] is None
            else:
                assert data["electrical_power_available"] is False
            assert call(name, "stop") == {"state": "idle"}
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        bundle.stop_all()
    assert not thread.is_alive()
    assert all(not card._thread.is_alive() for card in bundle._plugins
               if getattr(card, "_thread", None) is not None)


def test_configured_power_cards_through_http_mcp():
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve())],
        capture_output=True, text=True, timeout=15,
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    assert result.returncode == 0, result.stdout + result.stderr


if __name__ == "__main__":
    _check_http_mcp()
