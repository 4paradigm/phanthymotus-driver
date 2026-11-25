"""Read registration and battery freshness from the driver's network namespace."""
import json
import os
import ssl
import urllib.request


def request(url, data=None, context=None):
    req = urllib.request.Request(
        url, data=json.dumps(data).encode() if data is not None else None,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=3, context=context) as response:
        return json.load(response)


def main():
    endpoint = "http://localhost:15748/mcp"
    result = {"registered": False, "battery": {"state": "unknown"}}
    errors = []
    try:
        core = os.environ["AGENT_CORE_URL"].rstrip("/")
        registrations = request(core + "/api/mcp", context=ssl._create_unverified_context())
        result["registered"] = any(item.get("url") == endpoint for item in registrations["data"])
    except (OSError, ValueError, KeyError) as exc:
        errors.append(f"Core: {exc}")
    try:
        reply = request(endpoint, {
            "jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": "battery", "arguments": {"action": "info"}},
        })
        payload = json.loads(reply["result"]["content"][0]["text"])
        result["battery"] = payload["data"]["battery"]
    except (OSError, ValueError, KeyError) as exc:
        errors.append(f"Driver: {exc}")
    if errors:
        result["errors"] = errors
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
