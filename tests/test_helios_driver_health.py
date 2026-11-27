"""Regression checks for the on-demand Helios health card."""

import json
import ssl
from io import BytesIO
from queue import Queue
from threading import Event
from types import SimpleNamespace
from urllib import request as urllib_request

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
    schema = definition["inputSchema"]
    assert schema["properties"]["action"]["enum"] == ["status", "latest"]
    assert schema["x-completion"] == {"actions": ["status"], "timeout": 20}
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

    result = plugin._collect_health()

    assert result["state"] == "degraded"
    assert result["health_summary"] == "3/5 项正常"
    assert result["issues"] == ["chassis unavailable", "right_rear stale"]
    assert result["recommendation"] == "建议先等待片刻后重新查询。若异常持续，请重启机器人后再查询。"
    assert result["chassis"]["state"] == "unavailable"
    assert result["streams"]["right_rear"]["driver_input"] == {"available": True}
    assert result["streams"]["odometry"]["driver_input"] == "里程计查询不可用"


def test_healthy_status_has_no_recommendation(monkeypatch):
    plugin = health.HeliosDriverHealth({}, "test", None)
    monkeypatch.setattr(plugin, "_sample_streams", lambda: {
        name: {"state": "running"} for name in ("left_front", "right_rear", "odometry")
    })
    monkeypatch.setattr(health, "chassis_health", lambda _: {"state": "running"})
    monkeypatch.setattr(health, "_read_tool", lambda *args: {})
    monkeypatch.setattr(health, "read_link", lambda _: {"state": "running"})

    result = plugin._collect_health()

    assert result["state"] == "running"
    assert result["issues"] == []
    assert "recommendation" not in result


def test_canvas_lifecycle_does_not_sample(monkeypatch):
    plugin = health.HeliosDriverHealth({}, "test", None)
    monkeypatch.setattr(plugin, "_sample_streams", lambda: pytest.fail("Only status should sample"))
    bundle = DriverBundle([plugin])

    start = bundle.dispatch("driver_health", {"action": "start", "instance_id": "canvas-card"})
    stop = bundle.dispatch("driver_health", {"action": "stop", "instance_id": "canvas-card"})

    assert start == {"state": "ready"}
    assert stop == {"state": "idle"}


def test_status_completes_in_background_and_exposes_latest_snapshot(monkeypatch):
    plugin = health.HeliosDriverHealth({}, "test", None)
    bundle = DriverBundle([plugin])
    started = Event()
    release = Event()
    completions = Queue()
    snapshot = {"state": "running", "health_summary": "5/5 项正常"}

    def collect_health():
        started.set()
        assert release.wait(timeout=3)
        return dict(snapshot)

    monkeypatch.setattr(plugin, "_collect_health", collect_health)
    monkeypatch.setattr(plugin, "_post_completion", lambda *args: completions.put(args))

    try:
        assert bundle.dispatch("driver_health", {"action": "latest"})["state"] in {"idle", "waiting"}
        pending = bundle.dispatch("driver_health", {"action": "status"})
        assert pending["state"] == "sampling"
        assert pending["action_id"]
        assert started.wait(timeout=3)
        assert bundle.dispatch("driver_health", {"action": "latest"})["state"] == "sampling"
        busy = bundle.dispatch("driver_health", {"action": "status"})
        assert busy["state"] == "busy"
        assert "action_id" not in busy
        assert completions.empty()
    finally:
        release.set()

    action_id, status, result = completions.get(timeout=3)
    assert action_id == pending["action_id"]
    assert status == "completed"
    assert {key: result[key] for key in snapshot} == snapshot
    latest = bundle.dispatch("driver_health", {"action": "latest"})
    assert latest["action_id"] == action_id
    assert {key: latest[key] for key in snapshot} == snapshot


def test_failed_worker_posts_terminal_error_and_allows_new_sample(monkeypatch):
    plugin = health.HeliosDriverHealth({}, "test", None)
    completions = Queue()

    def collect_health():
        raise RuntimeError("test failure")

    monkeypatch.setattr(plugin, "_collect_health", collect_health)
    monkeypatch.setattr(plugin, "_post_completion", lambda *args: completions.put(args))

    pending = plugin.dispatch("driver_health", {})
    action_id, status, result = completions.get(timeout=3)
    assert action_id == pending["action_id"]
    assert status == "error"
    assert result["state"] == "error"
    assert plugin.dispatch("latest", {})["state"] == "error"

    monkeypatch.setattr(plugin, "_collect_health", lambda: {"state": "running"})
    retry = plugin.dispatch("status", {})
    new_id, status, _ = completions.get(timeout=3)
    assert new_id == retry["action_id"]
    assert new_id != action_id
    assert status == "completed"


@pytest.fixture
def acp_transport(monkeypatch):
    transport = SimpleNamespace(
        handlers=None, request=None, timeout=None, cafile=None,
        acknowledgement=b'{"ok": true, "action_id": "health-1"}',
    )

    def create_context(*, cafile=None):
        transport.cafile = cafile
        return ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)

    def open_completion(request, *, timeout):
        transport.request = request
        transport.timeout = timeout
        return BytesIO(transport.acknowledgement)

    def build_opener(*handlers):
        transport.handlers = handlers
        return SimpleNamespace(open=open_completion)

    monkeypatch.delenv("AGENT_CORE_CA_CERT", raising=False)
    monkeypatch.setattr(health.ssl, "create_default_context", create_context)
    monkeypatch.setattr(health.urllib.request, "build_opener", build_opener)
    return transport


@pytest.mark.parametrize("endpoint,ca_file", [
    ("https://localhost:15678", None),
    ("https://127.0.0.1:15678", None),
    ("https://[::1]:15678", None),
    ("https://core.example.test:15678", "core-test.pem"),
])
def test_acp_completion_uses_hostname_verification_for_remote_core(
    monkeypatch, capsys, acp_transport, endpoint, ca_file,
):
    monkeypatch.setenv("AGENT_CORE_URL", endpoint)
    if ca_file:
        monkeypatch.setenv("AGENT_CORE_CA_CERT", ca_file)
    plugin = health.HeliosDriverHealth({}, "test", None)
    snapshot = {"state": "running"}

    plugin._post_completion("health-1", "completed", snapshot)

    tls_handler, = [handler for handler in acp_transport.handlers if isinstance(handler, urllib_request.HTTPSHandler)]
    assert acp_transport.cafile == ca_file
    assert tls_handler._context.check_hostname is bool(ca_file)
    assert tls_handler._context.verify_mode == (ssl.CERT_REQUIRED if ca_file else ssl.CERT_NONE)
    assert acp_transport.request.full_url == f"{endpoint}/api/acp/complete"
    assert acp_transport.request.get_method() == "POST"
    assert acp_transport.timeout == 3
    payload = json.loads(acp_transport.request.data)
    assert payload["tool"] == "driver_health"
    assert (payload["action_id"], payload["status"], payload["result"]) == ("health-1", "completed", snapshot)
    assert not capsys.readouterr().err


@pytest.mark.parametrize("endpoint,ca_file", [
    ("https://core.example.test:15678", None),
    ("http://core.example.test:15678", "core-test.pem"),
])
def test_acp_completion_blocks_untrusted_remote_endpoints(monkeypatch, capsys, acp_transport, endpoint, ca_file):
    monkeypatch.setenv("AGENT_CORE_URL", endpoint)
    if ca_file:
        monkeypatch.setenv("AGENT_CORE_CA_CERT", ca_file)

    health.HeliosDriverHealth({}, "test", None)._post_completion("health-1", "completed", {})

    assert acp_transport.handlers is None
    assert "ACP callback failed for health-1" in capsys.readouterr().err


@pytest.mark.parametrize("acknowledgement", [
    b"malformed JSON",
    b"[]",
    b'{"ok": false, "action_id": "health-1"}',
    b'{"ok": 1, "action_id": "health-1"}',
    b'{"ok": true, "action_id": "other"}',
])
def test_acp_completion_logs_invalid_acknowledgements(monkeypatch, capsys, acp_transport, acknowledgement):
    monkeypatch.setenv("AGENT_CORE_URL", "https://localhost:15678")
    acp_transport.acknowledgement = acknowledgement

    health.HeliosDriverHealth({}, "test", None)._post_completion("health-1", "completed", {})

    assert acp_transport.request is not None
    assert "ACP callback failed for health-1" in capsys.readouterr().err


@pytest.mark.parametrize("odometry_port", [None, "15749"])
def test_odometry_probe_is_disabled_by_default_and_can_be_enabled(monkeypatch, odometry_port):
    monkeypatch.delenv("HELIOS_HEALTH_ODOMETRY_PORT", raising=False)
    if odometry_port is not None:
        monkeypatch.setenv("HELIOS_HEALTH_ODOMETRY_PORT", odometry_port)
    plugin = health.HeliosDriverHealth({}, "test", None)
    probes = []

    def read_tool(port, name, action):
        probes.append((port, name, action))
        return {"data": {"pose_state": "online", "speed_state": "online"}}

    monkeypatch.setattr(plugin, "_sample_streams", lambda: {
        key: {"state": "running"} for key in ("left_front", "right_rear", "odometry")
    })
    monkeypatch.setattr(health, "_read_tool", read_tool)
    monkeypatch.setattr(health, "read_link", lambda _: {"state": "running"})

    result = plugin._collect_health()["streams"]["odometry"]
    assert result["state"] == "running"
    odometry_probes = [probe for probe in probes if probe[1] == "loco_state"]
    if odometry_port is None:
        assert not odometry_probes
        assert result["driver_input"] == {"state": "disabled"}
    else:
        assert odometry_probes == [(int(odometry_port), "loco_state", "status")]
        assert result["driver_input"]["pose_state"] == "online"


def test_chassis_fault_and_unreported_state_have_distinct_status():
    ready = {"sdk_available": True, "worker_running": True}
    assert health.chassis_health(ready)["state"] == "waiting"
    assert health.chassis_health({**ready, "chassis_exception_level": 0})["state"] == "running"
    assert health.chassis_health({**ready, "chassis_exception_level": 1})["state"] == "degraded"
    assert health.chassis_health({**ready, "connect_error": "timeout"})["state"] == "error"
