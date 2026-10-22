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

    def call(method, params, expect_error=False):
        req = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                                     data=json.dumps({"jsonrpc": "2.0", "id": 1,
                                                      "method": method, "params": params}).encode(),
                                     headers={"Content-Type": "application/json"})
        with opener.open(req, timeout=10) as response:
            body = json.load(response)
        if expect_error:
            assert "error" in body, body
            return body["error"]
        assert "error" not in body, body
        return body["result"]

    try:
        tools = call("tools/list", {})["tools"]
        assert {t["name"] for t in tools} == {"wooden_man"}, tools
        assert next(t for t in tools if t["name"] == "wooden_man")["type"] == "processor"
        result = call("tools/call", {"name": "wooden_man", "arguments": {"action": "info"}})
        payload = json.loads(result["content"][0]["text"])
        assert isinstance(payload, dict) and payload["state"] == "idle", payload
        assert payload["topic_out"][0]["format"] == "data/json", payload
        # Reproduce Core's auto-config call before a project starts. This used
        # to fall through dispatch(None) and become "Unknown tool: wooden_man".
        result = call("tools/call", {"name": "wooden_man", "arguments": {
            "action": "config", "instance_id": "card-smoke", "cue_seconds": 4,
            "asr_topic": "", "pose_topic": ""}})
        payload = json.loads(result["content"][0]["text"])
        assert payload["status"] == "configured" and payload["state"] == "idle", payload
        result = call("tools/call", {"name": "wooden_man", "arguments": {"action": "info"}})
        assert json.loads(result["content"][0]["text"])["config"]["cue_seconds"] == 4
        error = call("tools/call", {"name": "wooden_man", "arguments": {
            "action": "config", "confidence": -1}}, expect_error=True)
        assert "confidence" in error["message"], error
        result = call("tools/call", {"name": "wooden_man", "arguments": {"action": "start"}})
        assert json.loads(result["content"][0]["text"])["state"] == "error"
        print("PASS: actual HTTP config -> info, persistence, invalid-config RPC error and missing-input error")
    finally:
        server.shutdown()
        server.server_close()
        driver._bundle.stop_all()


if __name__ == "__main__":
    run()
