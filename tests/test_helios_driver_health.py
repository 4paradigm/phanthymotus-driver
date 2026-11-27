"""Regression checks for the on-demand Helios health card."""

import json
from types import SimpleNamespace

import pytest

from common.vendor_runtime import DriverBundle
from rokae.helios import driver_health as health


def test_stream_reports_receive_rate_and_source_progress():
    sample = health.StreamSample(
        arrivals=[8.5, 9.5],
        stamps={"cloud": [1_700_000_000_000_000_000, 1_700_000_000_600_000_000]},
    )

    result = health.stream_health("/test/lidar/points", sample, publishers=1, now=10)

    assert result["state"] == "running"
    assert result["received_frames"] == 2
    assert result["receive_age_ms"] == 500.0
    assert result["observed_hz"] == 1.0
    assert result["source_timestamps"]["cloud"] == {
        "state": "running",
        "stamp_ns": "1700000000600000000",
        "advance_ms": 600.0,
        "receive_span_ms": 1_000.0,
        "lag_growth_ms": 400.0,
    }


@pytest.mark.parametrize(
    "publishers,arrivals,stamps,expected",
    [
        (0, [], {}, "unavailable"),
        (1, [], {}, "waiting"),
        (1, [9.9], {"cloud": [1_000_000]}, "waiting"),
        (1, [6.0, 7.0], {"cloud": [1_000_000, 2_000_000]}, "stale"),
        (1, [9.0, 9.9], {"cloud": [1_000_000, 1_000_000]}, "stale"),
        (1, [6.5, 7.0, 9.9], {"cloud": [1_000_000, 2_000_000, 2_000_000]}, "stale"),
        (1, [9.0, 9.9], {"cloud": [1_000_000, None]}, "unavailable"),
    ],
)
def test_stream_distinguishes_waiting_stalled_and_missing(publishers, arrivals, stamps, expected):
    sample = health.StreamSample(arrivals=arrivals, stamps=stamps)

    result = health.stream_health("/test/lidar/points", sample, publishers=publishers, now=10)

    assert result["state"] == expected


def test_odometry_distinguishes_source_stamps_from_publication_stamp(monkeypatch):
    clock = iter([9.0, 9.5])
    monkeypatch.setattr(health, "time", SimpleNamespace(monotonic=lambda: next(clock)))
    sample = health.StreamSample()
    health.record_odometry(sample, '{"schema":"other","stamp_ms":1500}')
    health.record_odometry(sample, "invalid JSON")
    for stamp, pose in [(1500, "1000000000"), (2000, "1500000000")]:
        health.record_odometry(sample, json.dumps({
            "schema": "motus.odom/1",
            "stamp_ms": stamp,
            "vendor": {"pose_source_stamp_ns": pose, "source_stamp_ms": 1000},
        }))

    result = health.stream_health("/test/state/odom", sample, publishers=1, now=10)

    assert result["received_frames"] == 2
    assert result["state"] == "degraded"
    assert result["source_timestamps"]["pose"]["state"] == "running"
    assert result["source_timestamps"]["velocity"]["state"] == "stale"
    assert result["source_timestamps"]["velocity"]["stamp_ns"] == "1000000000"


def test_link_status_from_sysfs(tmp_path):
    interface = tmp_path / "eth0"
    interface.mkdir()
    (interface / "carrier").write_text("1\n")
    (interface / "speed").write_text("100\n")
    (interface / "carrier_down_count").write_text("17\n")

    assert health.read_link("eth0", tmp_path) == {
        "state": "running",
        "interface": "eth0",
        "carrier": True,
        "speed_mbps": 100,
        "carrier_down_count": 17,
    }
    (interface / "carrier").write_text("0\n")
    (interface / "speed").write_text("-1\n")
    offline = health.read_link("eth0", tmp_path)
    assert (offline["state"], offline["carrier"], offline["speed_mbps"]) == ("offline", False, None)
    missing = health.read_link("eth1", tmp_path)
    assert (missing["state"], missing["carrier"]) == ("unavailable", None)
    with pytest.raises(ValueError):
        health.read_link("..", tmp_path)


def test_status_card_reports_driver_input_separately_from_ros_output(monkeypatch):
    plugin = health.HeliosDriverHealth(
        {"health_base_port": 0, "health_lidar_port": 15801, "health_odometry_port": 15802},
        "test",
        None,
    )
    bundle = DriverBundle([plugin])
    definition, = bundle.get_all_tools()
    assert definition["name"] == "driver_health"
    assert definition["type"] == "actuator"
    assert definition["inputSchema"]["properties"]["action"]["enum"] == ["status"]
    assert "topic_out" not in definition

    monkeypatch.setattr(plugin, "_sample_streams", lambda: {
        "left_front": {"state": "running"},
        "right_rear": {"state": "stale"},
        "odometry": {"state": "running"},
    })
    monkeypatch.setattr(health, "read_link", lambda _: {"state": "running"})
    expected_probes = {
        (0, "base_drive", "info"): {"error": "未配置 MCP 端口"},
        (15801, "lidar_cloud", "status"): {"sources": {
            "left_front": {"available": True}, "right_rear": {"available": True},
        }},
        (15802, "loco_state", "status"): {"error": "里程计查询不可用"},
    }
    monkeypatch.setattr(health, "_read_tool", lambda *args: expected_probes[args])

    result = bundle.dispatch("driver_health", {"action": "status"})

    assert result["state"] == "degraded"
    assert result["health_summary"] == "3/5 项正常"
    assert result["issues"] == ["chassis unavailable", "right_rear stale"]
    assert result["chassis"]["state"] == "unavailable"
    assert result["streams"]["right_rear"]["driver_input"] == {"available": True}
    assert result["streams"]["odometry"]["driver_input"] == "里程计查询不可用"


def test_canvas_lifecycle_does_not_sample(monkeypatch):
    plugin = health.HeliosDriverHealth({}, "test", None)
    monkeypatch.setattr(plugin, "_sample_streams", lambda: pytest.fail("Only status should sample"))
    bundle = DriverBundle([plugin])

    start = bundle.dispatch("driver_health", {"action": "start", "instance_id": "canvas-card"})
    stop = bundle.dispatch("driver_health", {"action": "stop", "instance_id": "canvas-card"})

    assert start == {"state": "ready"}
    assert stop == {"state": "idle"}


def test_chassis_fault_and_unreported_state_have_distinct_status():
    ready = {"sdk_available": True, "worker_running": True}
    assert health.chassis_health(ready)["state"] == "waiting"
    assert health.chassis_health({**ready, "chassis_exception_level": 0})["state"] == "running"
    assert health.chassis_health({**ready, "chassis_exception_level": 1})["state"] == "degraded"
    assert health.chassis_health({**ready, "connect_error": "timeout"})["state"] == "error"
