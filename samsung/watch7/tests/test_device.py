"""Unit tests for real data, quality, session, TTL and replay handling."""
from device import DeviceStore


class Clock:
    def __init__(self):
        self.wall = 1_800_000_000_000
        self.mono = 10000

    def advance(self, ms):
        self.wall += ms
        self.mono += ms


def v1(clock, seq=1, session="a", source="watch7_real"):
    return {
        "protocol_version": 1, "device_id": "watch7", "source": source,
        "session_id": session, "seq": seq, "captured_at_ms": clock.wall,
        "bpm": 79.0, "motion_score": 0.2,
    }


def v2_device(clock, seq=1, wear="on_body", session="a", online=True):
    return {
        "protocol_version": 2, "device_id": "watch7",
        "source": "watch7_real", "kind": "device_state",
        "session_id": session, "seq": seq,
        "captured_at_ms": clock.wall, "wear_state": wear,
        "online": online, "collecting": online,
        "battery_pct": 68,
    }


def v2_summary(clock, seq=1, session="a", wear="on_body", quality="valid", age=200):
    return {
        "protocol_version": 2, "device_id": "watch7",
        "source": "watch7_real", "kind": "summary",
        "session_id": session, "seq": seq, "captured_at_ms": clock.wall,
        "wear_state": wear, "metrics": {
            "heart_rate_bpm": {
                "value": 82 if quality == "valid" else None,
                "unit": "bpm", "quality": quality,
                "provider": "health_services", "age_ms": age,
            },
        },
    }


def test_real_v1_fresh_then_stale():
    clock = Clock()
    events = []
    store = DeviceStore(
        wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono,
        on_heart=events.append,
    )
    assert store.heart_snapshot()["heart_rate_bpm"] is None
    assert store.ingest_v1(v1(clock))["ok"]
    assert store.heart_snapshot()["heart_rate_bpm"] == 79
    assert events[0]["source"] == "watch7_real"
    clock.advance(15_001)
    assert not store.heart_snapshot()["fresh"]
    assert store.heart_snapshot()["heart_rate_bpm"] is None


def test_simulation_rejected_and_never_counts_as_live():
    clock = Clock()
    store = DeviceStore(wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono)
    assert store.ingest_v1(v1(clock, source="simulated"))["reason"] == "simulated_disabled"
    assert not store.device_snapshot()["online"]
    frame = v2_summary(clock)
    frame["source"] = "simulated"
    assert store.ingest_v2(frame)["reason"] == "simulated_disabled"


def test_replay_out_of_order_and_invalid_frame():
    clock = Clock()
    store = DeviceStore(wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono)
    assert store.ingest_v1(v1(clock, seq=4))["ok"]
    assert store.ingest_v1(v1(clock, seq=4))["reason"] == "duplicate_or_out_of_order"
    assert store.ingest_v1(v1(clock, seq=3))["reason"] == "duplicate_or_out_of_order"
    assert store.ingest_v1({})["reason"] == "bad_input"


def test_v2_device_and_heart_flow():
    clock = Clock()
    hearts, devices = [], []
    store = DeviceStore(
        wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono,
        on_heart=hearts.append, on_device=devices.append,
    )
    assert store.ingest_v2(v2_device(clock))["ok"]
    assert store.device_snapshot()["battery_pct"] == 68
    assert store.ingest_v2(v2_summary(clock))["ok"]
    assert store.heart_snapshot()["protocol_version"] == 2
    assert store.heart_snapshot()["heart_rate_bpm"] == 82
    assert hearts[0]["unit"] == "bpm"
    assert len(devices) == 1


def test_off_body_revokes_fresh_heart_immediately():
    clock = Clock()
    store = DeviceStore(wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono)
    assert store.ingest_v1(v1(clock))["ok"]
    assert store.ingest_v2(v2_device(clock, wear="off_body"))["ok"]
    assert not store.heart_snapshot()["available"]
    assert store.ingest_v1(v1(clock, seq=2))["reason"] == "off_body_or_wrong_session"


def test_new_device_session_cannot_reuse_old_heart():
    clock = Clock()
    store = DeviceStore(wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono)
    assert store.ingest_v1(v1(clock, session="old"))["ok"]
    assert store.ingest_v2(v2_device(clock, session="new"))["ok"]
    assert not store.heart_snapshot()["available"]
    assert store.ingest_v1(v1(clock, session="new"))["ok"]


def test_v2_bad_quality_not_published_and_no_raw_signal():
    clock = Clock()
    events = []
    store = DeviceStore(
        wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono,
        on_heart=events.append,
    )
    assert store.ingest_v2(v2_summary(clock, quality="unavailable"))["ok"]
    assert not store.heart_snapshot()["available"]
    assert events == []
    frame = v2_summary(clock, seq=2)
    frame["waveform"] = [1, 2]
    assert store.ingest_v2(frame)["reason"] == "bad_input"


def test_v2_heart_metric_age_and_device_staleness():
    clock = Clock()
    store = DeviceStore(wall_ms=lambda: clock.wall, monotonic_ms=lambda: clock.mono)
    assert store.ingest_v2(v2_summary(clock, age=16_000))["ok"]
    assert not store.heart_snapshot()["available"]
    assert store.ingest_v2(v2_device(clock))["ok"]
    clock.advance(15_001)
    assert not store.device_snapshot()["available"]
