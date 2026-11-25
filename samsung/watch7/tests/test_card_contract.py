"""Company MCP bundle semantics independent of ROS/robot hardware."""
import json

from common.vendor_runtime import DriverBundle, make_handler
from device import DeviceStore
from sensors import build_plugins


def test_two_sensor_cards_and_lifecycle():
    published = {}
    def publisher(topic):
        published[topic] = []
        return published[topic].append

    store = DeviceStore()
    plugins = build_plugins(store, "watch_demo", publisher, {
        "plugins": {"heart_rate": {"enabled": True}, "device_state": {"enabled": True}},
    })
    assert {x.name for x in plugins} == {"heart_rate", "device_state"}
    bundle = DriverBundle(plugins)
    for tool in bundle.get_all_tools():
        assert tool["type"] == "sensor"
        assert tool["topic_out"][0]["format"] == "data/json"
        assert set(tool["inputSchema"]["properties"]["action"]["enum"]) == {"start", "stop", "info"}
    assert bundle.dispatch("heart_rate", {"action": "start"}) == {"state": "running"}
    assert bundle.dispatch("heart_rate", {"action": "stop"}) == {"state": "idle"}
    assert bundle.dispatch("heart_rate", {"action": "unknown"}) is None
    assert bundle.dispatch("heart_rate", {"action": "info"})["snapshot"]["fresh"] is False
    assert not any(published.values())


def test_mcp_http_list_and_call():
    import urllib.request
    import threading
    from http.server import ThreadingHTTPServer

    plugins = build_plugins(
        DeviceStore(), "watch_demo", lambda _: (lambda data: None),
        {"plugins": {"heart_rate": {"enabled": True}, "device_state": {"enabled": True}}},
    )
    bundle = DriverBundle(plugins)
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0),
        make_handler(lambda: bundle, "Samsung Watch7", "samsung-watch7-driver"),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def call(method, params=None):
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            json.dumps({"jsonrpc": "2.0", "id": 42, "method": method,
                        "params": params or {}}).encode(),
            {"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)
    try:
        assert call("initialize")["result"]["serverInfo"]["name"] == "Samsung Watch7"
        assert [t["name"] for t in call("tools/list")["result"]["tools"]] == [
            "heart_rate", "device_state"]
        call_result = call("tools/call", {
            "name": "heart_rate", "arguments": {"action": "start"}
        })
        value = json.loads(call_result["result"]["content"][0]["text"])
        assert value["state"] == "running"
        assert call("tools/call", {
            "name": "device_state", "arguments": {"action": "info"}
        })["result"]["content"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
