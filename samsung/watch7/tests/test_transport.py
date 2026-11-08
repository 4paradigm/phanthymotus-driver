"""Watch App route compatibility and fail-closed security behaviour."""
import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from device import DeviceStore
from transport import make_device_server

TOKEN = "test-" + "x" * 24


def make_request(url, path, *, token=None, payload=None):
    headers = {"Accept": "application/json"}
    if token is not None:
        headers["X-Wearable-Token"] = token
    if payload is not None:
        headers["Content-Type"] = "application/json"
    request = Request(
        url + path, headers=headers,
        data=json.dumps(payload).encode() if payload is not None else None,
    )
    try:
        with urlopen(request, timeout=3) as response:
            return response.status, json.load(response)
    except HTTPError as exc:
        return exc.code, json.load(exc)


def test_watch_http_auth_and_disabled_commands():
    store = DeviceStore()
    server = make_device_server("127.0.0.1", 0, store, TOKEN)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    url = f"http://127.0.0.1:{server.server_port}"
    try:
        assert make_request(url, "/health")[0] == 200
        assert make_request(url, "/v1/commands")[0] == 401
        assert make_request(url, "/v1/commands", token="wrong")[0] == 401
        status, body = make_request(url, "/v1/commands", token=TOKEN)
        assert status == 200 and body["commands"] == []
        status, body = make_request(url, "/v2/commands", token=TOKEN)
        assert status == 200 and body["commands"] == []
        status, body = make_request(url, "/v2/task-prompts", token=TOKEN)
        assert status == 200 and body["prompts"] == []
        status, body = make_request(url, "/v2/haptic-authorize",
                                    token=TOKEN, payload={"request_id": "fake"})
        assert body["allowed"] is False
        assert make_request(url, "/mcp", token=TOKEN, payload={})[0] == 404
        assert make_request(url, "/v1/ack", token=TOKEN,
                            payload={"request_id": "old"})[1]["reason"] == "not_pending"
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


def test_http_ingest_v1_then_duplicate_rejected():
    import time
    store = DeviceStore()
    server = make_device_server("127.0.0.1", 0, store, TOKEN)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    frame = {
        "protocol_version": 1, "device_id": "watch7", "source": "watch7_real",
        "session_id": "abc", "seq": 1,
        "captured_at_ms": int(time.time() * 1000),
        "bpm": 85, "motion_score": None,
    }
    try:
        assert make_request(url, "/v1/telemetry", token=TOKEN, payload=frame)[0] == 200
        assert store.heart_snapshot()["heart_rate_bpm"] == 85
        status, result = make_request(url, "/v1/telemetry", token=TOKEN, payload=frame)
        assert status == 422 and result["reason"] == "duplicate_or_out_of_order"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
