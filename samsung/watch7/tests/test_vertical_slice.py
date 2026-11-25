"""Real Watch-compatible HTTP -> DeviceStore -> Sensor Card -> ROS2 JSON payload."""
import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from device import DeviceStore
from sensors import build_plugins
from transport import make_device_server


def test_watch_upload_produces_real_card_data_and_off_body_stops_heart():
    received = {}
    publishers = {}
    token = "test-" + "z" * 24

    def publisher(topic):
        publishers[topic] = []
        return publishers[topic].append

    device = DeviceStore(
        on_heart=lambda event: received["heart_rate"].publish(event),
        on_device=lambda event: received["device_state"].publish(event),
    )
    cards = build_plugins(
        device, "robot_1", publisher, {"plugins": {
            "heart_rate": {"enabled": True},
            "device_state": {"enabled": True},
        }}
    )
    received.update({card.name: card for card in cards})
    for card in cards:
        card.start()

    server = make_device_server("127.0.0.1", 0, device, token)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    now = int(time.time() * 1000)

    def send(payload, version=2):
        req = Request(
            url + f"/v{version}/telemetry",
            data=json.dumps(payload).encode(),
            headers={"X-Wearable-Token": token, "Content-Type": "application/json"},
        )
        try:
            with urlopen(req, timeout=3) as reply:
                return json.load(reply)
        except HTTPError as reply:
            return json.load(reply)

    try:
        base = {
            "protocol_version": 2, "device_id": "watch7", "source": "watch7_real",
            "session_id": "stream1", "captured_at_ms": now, "wear_state": "on_body",
        }
        state = {**base, "kind": "device_state", "seq": 0,
                 "online": True, "collecting": True, "battery_pct": 70}
        assert send(state)["ok"]
        summary = {**base, "kind": "summary", "seq": 0, "metrics": {
            "heart_rate_bpm": {"value": 81, "unit": "bpm",
                               "quality": "valid", "provider": "health_services",
                               "age_ms": 0},
        }}
        assert send(summary)["ok"]
        htopic = "/robot_1/watch7/heart_rate"
        dtopic = "/robot_1/watch7/device_state"
        assert json.loads(publishers[htopic][-1])["heart_rate_bpm"] == 81
        assert json.loads(publishers[dtopic][-1])["battery_pct"] == 70
        assert device.heart_snapshot()["fresh"]

        # The same session later reports off-body; cached BPM cannot be read.
        state2 = {**state, "seq": 1, "wear_state": "off_body"}
        assert send(state2)["ok"]
        assert not device.heart_snapshot()["fresh"]

        # Stop is synchronous: no further publishing after the card is stopped.
        received["heart_rate"].stop()
        before = len(publishers[htopic])
        v1 = {
            "protocol_version": 1, "device_id": "watch7", "source": "watch7_real",
            "session_id": "stream2", "seq": 0, "captured_at_ms": now,
            "bpm": 82, "motion_score": None,
        }
        # New session is blocked while a fresh different-session heartbeat exists.
        # After this test, it must never produce an extra ROS2 message.
        assert not send(v1, version=1)["ok"]
        assert len(publishers[htopic]) == before
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


def test_missing_deployment_token_fails_before_ros(monkeypatch):
    from main import run
    monkeypatch.delenv("WATCH7_DEVICE_TOKEN", raising=False)
    try:
        run()
    except RuntimeError as exc:
        assert "WATCH7_DEVICE_TOKEN" in str(exc)
    else:
        raise AssertionError("Driver must refuse to start without an ingress credential")
