"""Go1 power cards: units, source freshness, outage handling and registration."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GO1 = ROOT / "unitree" / "go1"
sys.path.insert(0, str(GO1))

import power  # noqa: E402


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
    config = (GO1 / "config.yaml").read_text()
    manifest = (GO1 / "driver.yaml").read_text()
    for name in ("battery_power", "joint_power"):
        assert f"  {name}:" in config
        assert f"name: {name}," in manifest
    assert "COPY power.py" in (GO1 / "Dockerfile").read_text()
