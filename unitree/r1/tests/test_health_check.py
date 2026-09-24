"""R1 health card verdicts do not require ROS or a connected robot."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from health_check import HealthCheckPlugin, evaluate_health


NOW = 100.0
WALL = 1_700_000_000.0


def samples():
    data = {
        "battery": {"soc": 65, "temperature": [31]},
        "imu": {"rpy": [0, 0, 0], "temperature": 37},
        "joints": {"joints": [{"name": "hip", "temp": [40, 42]}]},
        "mainboard": {"temperature": [44], "fan_state": [1]},
    }
    return {name: {"data": value, "received_monotonic": NOW - 0.01,
                   "received_at": WALL - 0.01} for name, value in data.items()}


def check(value, config=None):
    return evaluate_health(value, config or {"battery_min_soc": 20},
                           now_monotonic=NOW, now_wall=WALL)


def test_normal_report_preserves_readings_and_discloses_unassessed_temperature():
    report = check(samples())
    assert report["status"] == "正常"
    assert set(report["temperature_unassessed"]) == set(samples())
    assert report["checks"]["battery"]["data"]["soc"] == 65
    assert report["checks"]["joints"]["temperature"]["maximum"] == 42
    assert report["checks"]["battery"]["received_at"]
    assert "不是运动安全许可" in report["summary"]


def test_low_battery_reports_the_reading_and_threshold():
    value = samples()
    value["battery"]["data"]["soc"] = 19
    report = check(value)
    assert report["status"] == "异常"
    assert report["checks"]["battery"]["status"] == "异常"
    assert "19%" in report["checks"]["battery"]["reasons"][0]


def test_missing_and_stale_sources_never_report_normal():
    value = samples()
    del value["mainboard"]
    assert check(value)["status"] == "数据不足"
    value = samples()
    value["battery"]["received_monotonic"] = NOW - 3.01
    report = check(value)
    assert report["status"] == "数据不足"
    assert report["checks"]["battery"]["status"] == "数据不足"


def test_abnormal_overrides_missing_data():
    value = samples()
    value["battery"]["data"]["soc"] = 5
    del value["mainboard"]
    assert check(value)["status"] == "异常"


def test_configured_temperature_limit_flags_overheat():
    report = check(samples(), {"battery_min_soc": 20,
                               "temperature_limits": {"joints": 41}})
    assert report["status"] == "异常"
    assert report["checks"]["joints"]["temperature"]["status"] == "异常"
    assert "42" in report["checks"]["joints"]["reasons"][0]


def test_configured_temperature_without_reading_is_incomplete():
    value = samples()
    value["mainboard"]["data"]["temperature"] = []
    report = check(value, {"temperature_limits": {"mainboard": 70}})
    assert report["status"] == "数据不足"
    assert report["checks"]["mainboard"]["temperature"]["status"] == "数据不足"


def test_plugin_is_read_only_and_accepts_canvas_lifecycle():
    import time

    class State:
        def health_snapshot(self):
            result = samples()
            for item in result.values():
                item["received_monotonic"] = time.monotonic()
            return result

    plugin = HealthCheckPlugin({"battery_min_soc": 20}, State())
    assert plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"] == ["check"]
    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert plugin.dispatch("check", {})["status"] == "正常"
    assert plugin.dispatch("stop", {}) == {"state": "idle"}


def test_invalid_thresholds_fail_at_startup():
    import pytest

    with pytest.raises(ValueError):
        HealthCheckPlugin({"battery_min_soc": -1})
    with pytest.raises(ValueError):
        HealthCheckPlugin({"temperature_limits": {"joints": float("nan")}})
