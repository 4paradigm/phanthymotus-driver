"""Personal fixed-point patrol integration, using synthetic telemetry/photos."""
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import sys
import threading

import pytest


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix/bumi"
sys.path.insert(0, str(BUMI))
from patrol_card import PatrolPlugin, PatrolTelemetry


class Clock:
    def __init__(self):
        self.now = 100.0
    def __call__(self):
        return self.now


class Capture:
    def __init__(self):
        self.available = True
        self.age = 0.1
        self.photos = 0
        self.timestamp = lambda: datetime.now(timezone.utc).isoformat()
    def dispatch(self, action, args):
        if action == "info":
            return {"ok": self.available, "latest_frame_age_s": self.age}
        assert action == "capture_photo"
        self.photos += 1
        return {"ok": True, "file_path": f"synthetic:photo-{self.photos}",
                "frame_captured_at": self.timestamp(), "timestamp_source": "synthetic"}


class Inputs:
    def __init__(self, *args):
        self.closed = False
    def close(self):
        self.closed = True


def feed(telemetry, *, soc=80, alarm=0, mode=2, raw_error=0, velocity=0):
    telemetry.receive("battery", {"soc": soc, "alarm": alarm})
    telemetry.receive("motion", {"fresh": True, "workmode": {"code": mode},
                                 "joint_states": [
                                     {"motor_id": i, "velocity": velocity, "error": raw_error}
                                     for i in range(21)]})


@pytest.fixture
def setup_card(tmp_path):
    clock = Clock()
    telemetry = PatrolTelemetry(clock=clock)
    capture = Capture()
    card = PatrolPlugin({"report_dir": str(tmp_path / "reports")}, "test", None, capture,
                        telemetry=telemetry, input_factory=Inputs)
    feed(telemetry)
    return card, telemetry, capture, clock


def begin(card, run_id="round-1"):
    return card.dispatch("begin", {"run_id": run_id})


def inspect(card):
    return card.dispatch("inspect", {"point_id": "fixed-passage"})


def record(card, inspection, decision="normal", **overrides):
    return card.dispatch("record", {"inspection_id": inspection["inspection_id"],
                                    "check_id": "clear-walkway", "decision": decision,
                                    "assessment": "合成观测，通道连续可见且畅通。", **overrides})


@pytest.mark.parametrize("soc,alarm,mode,error,velocity,expected,reason", [
    (80, 0, 2, 0, 0, "ready", None),
    (19, 0, 2, 0, 0, "blocked", "low_battery"),
    (20, 0, 2, 0, 0, "ready", None),
    (80, 1, 2, 0, 0, "blocked", "battery_alarm"),
    (80, 0, 26, 0, 0, "blocked", "workmode_not_walking"),
    (80, 0, 2, 1, 0, "needs_review", "joint_raw_error_0_1"),
    (80, 0, 2, 0, 0.15, "blocked", "robot_is_moving"),
    (101, 0, 2, 0, 0, "needs_review", "telemetry_fields_invalid"),
])
def test_readiness_is_inside_patrol(setup_card, soc, alarm, mode, error, velocity, expected, reason):
    card, telemetry, capture, clock = setup_card
    feed(telemetry, soc=soc, alarm=alarm, mode=mode, raw_error=error, velocity=velocity)
    result = begin(card)
    assert result["state"] == ("active" if expected == "ready" else expected)
    if reason:
        assert reason in result["reasons"]
        assert card.dispatch("info", {})["run_id"] is None
    assert capture.photos == 0


@pytest.mark.parametrize("bad", ["{", [], None, {"soc": float("nan")}, {"soc": True, "alarm": 0}])
def test_invalid_update_does_not_reuse_previous_good_battery(setup_card, bad):
    card, telemetry, capture, clock = setup_card
    telemetry.receive("battery", bad)
    assert begin(card)["state"] == "needs_review"


def test_missing_stale_and_recovered_telemetry(setup_card):
    card, telemetry, capture, clock = setup_card
    clock.now += 3.01
    result = begin(card)
    assert result["state"] == "needs_review"
    assert "battery_missing_invalid_or_stale" in result["reasons"]
    feed(telemetry)
    assert begin(card)["state"] == "active"
    card.stop()
    assert card.dispatch("preflight", {})["state"] == "needs_review"


@pytest.mark.parametrize("age", [None, -1, 3.01, float("nan")])
def test_bad_camera_prevents_patrol_start(setup_card, age):
    card, telemetry, capture, clock = setup_card
    capture.age = age
    assert begin(card)["state"] == "needs_review"
    assert capture.photos == 0


def test_full_normal_patrol_captures_assesses_and_writes_report_once(setup_card):
    card, telemetry, capture, clock = setup_card
    card.start()
    assert begin(card)["state"] == "active"
    assert begin(card, "another")["state"] == "busy"
    observation = inspect(card)
    assert observation["state"] == "awaiting_assessment"
    assert capture.photos == 1
    result = record(card, observation)
    assert result["status"] == "normal"
    assert result["agent_decision"] == "normal"
    assert result["sent"] is False
    summary = card.dispatch("finish", {})
    assert summary["all_normal"] is True
    path = Path(summary["report_path"])
    assert json.loads(path.read_text(encoding="utf-8"))["records"][0]["status"] == "normal"
    assert card.dispatch("finish", {}) == summary
    assert list(path.parent.glob("*.json")) == [path]
    assert begin(card, "round-2")["state"] == "active"
    assert card.dispatch("finish", {})["state"] == "incomplete"


def test_unknown_point_is_rejected_before_capture(setup_card):
    card, telemetry, capture, clock = setup_card
    begin(card)
    with pytest.raises(ValueError):
        card.dispatch("inspect", {"point_id": "unknown"})
    assert capture.photos == 0


@pytest.mark.parametrize("timestamp", [None, "2026-11-26T12:00:00", "bad", "2000-01-01T00:00:00+00:00"])
def test_unknown_or_stale_photo_time_never_becomes_normal(setup_card, timestamp):
    card, telemetry, capture, clock = setup_card
    capture.timestamp = lambda: timestamp
    begin(card)
    result = record(card, inspect(card))
    assert result["status"] == "needs_review"
    assert result["agent_decision"] == "normal"
    assert card.dispatch("finish", {})["all_normal"] is False


def test_state_changes_after_capture_force_review(setup_card):
    card, telemetry, capture, clock = setup_card
    begin(card)
    observation = inspect(card)
    feed(telemetry, soc=1)
    result = record(card, observation)
    assert result["status"] == "needs_review"
    assert result["review_reason"] == "patrol_readiness_changed"


def test_client_cannot_replace_server_owned_capture_reference(setup_card):
    card, telemetry, capture, clock = setup_card
    begin(card)
    result = record(card, inspect(card), "deviation", evidence={"reference": "forged"})
    assert result["evidence"]["reference"] == "synthetic:photo-1"
    assert result["status"] == "deviation_report"
    assert card.dispatch("finish", {})["all_normal"] is False


def test_failed_capture_never_completes_checks(setup_card):
    card, telemetry, capture, clock = setup_card
    begin(card)
    original = capture.dispatch
    def broken(action, args):
        if action == "capture_photo":
            raise RuntimeError("camera disconnected")
        return original(action, args)
    capture.dispatch = broken
    assert inspect(card)["state"] == "needs_review"
    assert card.dispatch("finish", {})["state"] == "incomplete"


def test_stop_discards_late_capture_and_releases_only_own_inputs(setup_card):
    card, telemetry, capture, clock = setup_card
    card.start()
    inputs = card._inputs
    begin(card)
    entered, release = threading.Event(), threading.Event()
    original = capture.dispatch
    def delayed(action, args):
        if action == "capture_photo":
            entered.set()
            assert release.wait(2)
        return original(action, args)
    capture.dispatch = delayed
    results = []
    thread = threading.Thread(target=lambda: results.append(inspect(card)))
    thread.start()
    assert entered.wait(2)
    card.stop()
    release.set()
    thread.join(2)
    assert not thread.is_alive()
    assert inputs.closed
    assert results[0]["state"] == "cancelled"
    assert card.dispatch("info", {})["records"] == []


def test_report_save_failure_cleans_temporary_file_and_can_retry(setup_card, monkeypatch):
    import patrol_card
    card, telemetry, capture, clock = setup_card
    begin(card)
    record(card, inspect(card))
    replace = patrol_card.os.replace
    def fail(*args):
        raise OSError("simulated disk failure")
    monkeypatch.setattr(patrol_card.os, "replace", fail)
    with pytest.raises(OSError):
        card.dispatch("finish", {})
    assert list(card._report_dir.iterdir()) == []
    monkeypatch.setattr(patrol_card.os, "replace", replace)
    assert card.dispatch("finish", {})["state"] == "completed"


def test_only_one_patrol_card_and_internal_status_actions_are_declared(setup_card):
    card, telemetry, capture, clock = setup_card
    spec = card.get_tool()
    assert spec["name"] == "patrol"
    assert "preflight" in spec["inputSchema"]["x-action-params"]
    assert "inspect" in spec["inputSchema"]["x-action-params"]
    assert "evidence" not in spec["inputSchema"]["properties"]
    with pytest.raises(ValueError):
        card.dispatch("begin", {"run_id": "../escape"})


def test_real_bundle_registration_and_http_patrol_workflow(setup_card, monkeypatch):
    from http.server import ThreadingHTTPServer
    from types import SimpleNamespace
    from urllib.request import Request, urlopen
    import patrol_card
    card, telemetry, capture, clock = setup_card
    fake_ros = SimpleNamespace(executors=SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rclpy", fake_ros)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_ros.executors)
    factory_calls = []
    def factory(config, namespace, executor, capture_plugin):
        factory_calls.append((config, capture_plugin))
        return card
    monkeypatch.setattr(patrol_card, "PatrolPlugin", factory)
    spec = importlib.util.spec_from_file_location("patrol_http_bundle_test", BUMI / "main.py")
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    main._bundle = main.BumiDeviceBundle({"plugins": {"patrol": {"enabled": True}}}, "test", None, None, None)
    assert len(factory_calls) == 1
    main._bundle.start_all()
    server = ThreadingHTTPServer(("127.0.0.1", 0), main.make_handler())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        def rpc(method, params):
            request = Request(f"http://127.0.0.1:{server.server_port}/mcp",
                              data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                                               "params": params}).encode(),
                              headers={"Content-Type": "application/json"})
            with urlopen(request, timeout=3) as response:
                return json.load(response)
        assert [tool["name"] for tool in rpc("tools/list", {})["result"]["tools"]] == ["patrol"]
        def call(arguments):
            response = rpc("tools/call", {"name": "patrol", "arguments": arguments})
            return json.loads(response["result"]["content"][0]["text"])
        assert call({"action": "begin", "run_id": "http-round"})["state"] == "active"
        observation = call({"action": "inspect", "point_id": "fixed-passage"})
        result = call({"action": "record", "inspection_id": observation["inspection_id"],
                       "check_id": "clear-walkway", "decision": "normal", "assessment": "合成 HTTP 巡检通过。"})
        assert result["status"] == "normal"
        assert call({"action": "finish"})["all_normal"] is True
        assert Path(call({"action": "finish"})["report_path"]).is_file()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)
        main._bundle.stop_all()


def test_patrol_sources_and_default_baseline_are_packaged():
    spec = importlib.util.spec_from_file_location("patrol_packaging_test", ROOT / "tests/test_dockerfile_copies.py")
    packaging = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(packaging)
    packaging.test_every_copied_path_exists(BUMI / "Dockerfile")
    docker = (BUMI / "Dockerfile").read_text(encoding="utf-8")
    assert "COPY patrol_baseline.py patrol_card.py /work/" in docker
    assert "patrol-baseline-bumi-fixed.json" in docker
    assert "report_dir: /opt/phanthy-motus/data/patrol" in (BUMI / "config.yaml").read_text(encoding="utf-8")
    assert "/opt/phanthy-motus/data:/opt/phanthy-motus/data" in (BUMI / "deploy/service.yml").read_text()


@pytest.mark.parametrize(("source_stamp", "source_seconds"), [
    ({"sec": 1795676400, "nanosec": 123000000}, 1795676400.123),
    (None, None),
    ({"sec": 0, "nanosec": 0}, None),
    ({"sec": 1795676400, "nanosec": 1_000_000_000}, None),
])
def test_real_photo_method_preserves_source_header_and_saved_bytes(tmp_path, source_stamp, source_seconds):
    import ast
    import time
    from types import SimpleNamespace
    # Exercise the production method while keeping ROS/SDK outside this test.
    tree = ast.parse((BUMI / "device.py").read_text(encoding="utf-8"))
    capture_class = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                         and node.name == "VisionCapturePlugin")
    capture_method = next(node for node in capture_class.body if isinstance(node, ast.FunctionDef)
                          and node.name == "_capture_photo")
    scope = {"datetime": datetime, "timezone": timezone, "time": time}
    exec(compile(ast.Module(body=[capture_method], type_ignores=[]), "actual_photo_method", "exec"), scope)
    frame = {"data": b"synthetic-jpeg-payload", "timestamp_ms": 1795676520123,
             "ros_timestamp": source_stamp}
    capture = SimpleNamespace(_output_dir=tmp_path, _frame=lambda: (frame, 1))
    result = scope["_capture_photo"](capture, {})
    assert result["ok"] is True
    assert Path(result["file_path"]).read_bytes() == frame["data"]
    if source_seconds is None:
        assert result["frame_captured_at"] is None
        assert result["timestamp_source"] == "unknown"
    else:
        assert datetime.fromisoformat(result["frame_captured_at"]).timestamp() == pytest.approx(
            source_seconds, rel=0, abs=1e-6)
        assert result["timestamp_source"] == "camera_host_publish"
