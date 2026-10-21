"""Exercise the bundle's actual HTTP MCP wrapper without robot hardware."""
import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

import main as driver


def run():
    driver._bundle = driver.Q5Bundle({"plugins": {"wooden_man": {"enabled": True}}},
                                    "smoke", None, None)
    driver._bundle.start_all()
    server = ThreadingHTTPServer(("127.0.0.1", 0), driver.make_handler())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def call(method, params):
        req = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                                     data=json.dumps({"jsonrpc": "2.0", "id": 1,
                                                      "method": method, "params": params}).encode(),
                                     headers={"Content-Type": "application/json"})
        with opener.open(req, timeout=10) as response:
            body = json.load(response)
        assert "error" not in body, body
        return body["result"]

    try:
        tools = call("tools/list", {})["tools"]
        assert len(tools) == 1 and tools[0]["name"] == "wooden_man", tools
        assert tools[0]["type"] == "processor"
        result = call("tools/call", {"name": "wooden_man", "arguments": {"action": "info"}})
        payload = json.loads(result["content"][0]["text"])
        assert isinstance(payload, dict) and payload["state"] == "idle", payload
        assert payload["topic_out"][0]["format"] == "text/plain", payload
        result = call("tools/call", {"name": "wooden_man", "arguments": {"action": "start"}})
        assert json.loads(result["content"][0]["text"])["state"] == "error"
        print("PASS: actual bundle registration, HTTP tools/list, plain-dict MCP wrapping and missing-input error")
    finally:
        server.shutdown()
        server.server_close()
        driver._bundle.stop_all()


if __name__ == "__main__":
    run()
