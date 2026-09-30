"""Go1 power cards: units, source freshness, outage handling and registration."""

import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GO1 = ROOT / "unitree" / "go1"
sys.path.insert(0, str(GO1))

import sensors as power  # noqa: E402


class StubClient:
    def snapshot(self):
        return {"fresh": False}


def battery_snap(stamp, status=1, current_ma=10000, cell_mv=3600, soc=80):
    return {"fresh": True, "sample_monotonic_s": stamp,
            "battery": {"status_code": status, "current_ma": current_ma,
                        "cell_voltage_mv": [cell_mv] * 10, "soc_percent": soc}}


def test_battery_power_and_energy_skip_duplicate_and_outage():
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    first = card._sample(battery_snap(100.0), now=100.1)
    assert first["power_w"] == 360.0
    assert first["voltage_v"] == 36.0
    assert first["discharged_since_start_wh"] == 0
    second = card._sample(battery_snap(101.0), now=101.1)
    assert second["discharged_since_start_wh"] == 0.1
    assert card._sample(battery_snap(101.0), now=101.2)["discharged_since_start_wh"] == 0.1
    assert not card._sample(battery_snap(101.0), now=105.0)["available"]
    assert card._sample(battery_snap(106.0), now=106.1)["discharged_since_start_wh"] == 0.1
    assert card._sample(battery_snap(107.0), now=107.1)["discharged_since_start_wh"] == 0.2


def test_charge_is_separate_and_invalid_cells_do_not_accumulate():
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    card._sample(battery_snap(100.0), now=100.1)
    transition = card._sample(battery_snap(101.0, status=2), now=101.1)
    assert transition["charged_since_start_wh"] == 0
    charged = card._sample(battery_snap(102.0, status=2), now=102.1)
    assert charged["signed_power_w"] == -360.0
    assert charged["charged_since_start_wh"] == 0.1
    assert charged["discharged_since_start_wh"] == 0
    assert card._sample(battery_snap(103.0, cell_mv=0), now=103.1)["reason"] == "invalid_bms_values"
    assert card._sample(battery_snap(104.0, status=2), now=104.1)["charged_since_start_wh"] == 0.1


def test_runtime_requires_observed_soc_drop_and_resets_after_gap_or_charge():
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    for second in range(61):
        soc = 80 - second // 20
        result = card._sample(battery_snap(100.0 + second, soc=soc), now=100.1 + second)
    assert result["remaining_runtime_minutes"] == 25.7
    assert result["runtime_estimate_method"] == "observed_soc_decline"
    assert card._sample(battery_snap(161.0, status=2, soc=77), now=161.1)["remaining_runtime_minutes"] is None
    assert card._sample(battery_snap(162.0, soc=77), now=162.1)["runtime_estimate_reason"] == "insufficient_soc_history"
    assert card._sample(battery_snap(170.0, soc=76), now=170.1)["remaining_runtime_minutes"] is None


def test_runtime_does_not_invent_time_without_soc_or_after_soc_rebound():
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    for second in range(61):
        result = card._sample(battery_snap(100.0 + second, soc=80), now=100.1 + second)
    assert result["remaining_runtime_minutes"] is None
    result = card._sample(battery_snap(161.0, soc=None), now=161.1)
    assert result["runtime_estimate_reason"] == "soc_unavailable"
    card._sample(battery_snap(162.0, soc=77), now=162.1)
    result = card._sample(battery_snap(163.0, soc=78), now=163.1)
    assert result["remaining_runtime_minutes"] is None
    assert result["runtime_estimate_reason"] == "insufficient_soc_history"


def test_cached_battery_data_expires_and_has_required_headers(monkeypatch):
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    card._sample(battery_snap(100.0), now=100.1)
    monkeypatch.setattr(power.time, "monotonic", lambda: 104.0)
    data = card.dispatch("info", {})["data"]
    assert data["fresh"] is False
    assert data["available"] is False
    assert {"timestamp_ms", "control_level", "fresh"} <= data.keys()


def test_concurrent_starts_use_one_sampler_and_stop_cancels_it():
    card = power.BatteryPowerPlugin({}, "test", None, StubClient())
    barrier = threading.Barrier(6)
    workers = [threading.Thread(target=lambda: (barrier.wait(), card.start()))
               for _ in range(6)]
    try:
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(timeout=3)
            assert not worker.is_alive()
        sampler = card._thread
        assert sampler.is_alive()
        card.start()
        assert card._thread is sampler
        assert card.dispatch("stop", {}) == {"state": "idle"}
        assert not sampler.is_alive()
        assert card.dispatch("unknown", {}) is None
    finally:
        card.stop()


def test_dispatch_stop_halts_sampling_and_start_resumes_without_pause_energy(monkeypatch):
    sampled = threading.Event()
    now = [101.1]
    monkeypatch.setattr(power.time, "monotonic", lambda: now[0])

    class CountingClient:
        calls = 0
        stamp = 101.0

        def snapshot(self):
            self.calls += 1
            return battery_snap(self.stamp)

    client = CountingClient()
    card = power.BatteryPowerPlugin({}, "test", None, client)
    sample = card._sample

    def record_sample(snap):
        result = sample(snap)
        sampled.set()
        return result

    monkeypatch.setattr(card, "_sample", record_sample)
    sample(battery_snap(100.0), now=100.1)
    sample(battery_snap(101.0), now=101.1)
    try:
        assert card.dispatch("start", {}) == {"state": "running"}
        assert sampled.wait(timeout=2)
        first_thread = card._thread
        assert card.dispatch("stop", {}) == {"state": "idle"}
        assert not first_thread.is_alive()
        reads_at_stop = client.calls
        sampled.clear()
        assert not sampled.wait(timeout=0.05)
        assert client.calls == reads_at_stop
        stopped = card.dispatch("info", {})
        assert stopped["state"] == "idle"
        assert stopped["data"]["reason"] == "sampling_stopped"
        assert stopped["data"]["available"] is False
        assert stopped["data"]["discharged_since_start_wh"] == 0.1
        assert card._last is None and card._runtime_start is None

        # A short pause must also be skipped, even within the normal gap limit.
        client.stamp = 102.0
        now[0] = 102.1
        assert card.dispatch("start", {}) == {"state": "running"}
        assert sampled.wait(timeout=2)
        assert card._thread is not first_thread and card._thread.is_alive()
        resumed = card.dispatch("info", {})
        assert resumed["state"] == "running"
        assert resumed["data"]["discharged_since_start_wh"] == 0.1
        assert resumed["data"]["runtime_estimate_reason"] == "insufficient_soc_history"
        next_sample = sample(battery_snap(103.0), now=103.1)
        assert next_sample["discharged_since_start_wh"] == 0.2
    finally:
        card.stop()


def test_joint_power_is_mechanical_and_rejects_stale_or_incomplete_data():
    joints = [{"tau": 2.0, "dq": 3.0} for _ in range(12)]
    joints[1] = {"tau": -2.0, "dq": 3.0}
    snap = {"fresh": True, "sample_monotonic_s": 100.0, "joints": joints}
    result = power._build_joint_power(snap, now=100.1)
    assert result["available"]
    assert result["joints"][0]["mechanical_power_w"] == 6.0
    assert result["joints"][1]["mechanical_power_w"] == -6.0
    assert result["positive_mechanical_power_w"] == 66.0
    assert result["negative_mechanical_power_w"] == -6.0
    assert result["electrical_power_available"] is False
    assert power._build_joint_power(snap, now=103.0)["reason"] == "stale_or_missing_sample"
    snap["joints"] = joints[:11]
    assert power._build_joint_power(snap, now=100.1)["reason"] == "incomplete_joint_state"


def test_cards_are_configured_and_copied_into_image():
    import yaml

    config = yaml.safe_load((GO1 / "config.yaml").read_text())
    manifest = yaml.safe_load((GO1 / "driver.yaml").read_text())
    enabled = {name for name, settings in config["plugins"].items()
               if settings.get("enabled", False)}
    cards = manifest["cards"]
    assert len({card["name"] for card in cards}) == len(cards)
    assert {card["name"] for card in cards} == enabled
    for name in ("battery_power", "joint_power"):
        assert name in enabled
        assert next(card["type"] for card in cards if card["name"] == name) == "sensor"
    assert "COPY sensors.py" in (GO1 / "Dockerfile").read_text()
