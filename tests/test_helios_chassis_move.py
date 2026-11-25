"""Guarded chassis movement: limits, single active move, completion, stop and the SDK worker."""

import json
import math
import sys
import textwrap
import threading
import time

import pytest

from common.vendor_runtime import DriverBundle
from rokae.helios.chassis_move import HeliosChassisMove, SdkWorkerLink
from rokae.helios.chassis_sdk_worker import OdomTracker, SpeedSession, Worker


# ── fake vendor nav module ──────────────────────────────────────────────
class FakeNav:
    class Vector3:
        x = y = z = 0.0

    class Twist:
        linear = angular = None

    def __init__(self, acquire=True, send_ok=True):
        self.acquire = acquire
        self.send_ok = send_ok
        self.calls = []
        self.sent = []

    def set_third_speed_ctrl(self):
        self.calls.append("acquire")
        return self.acquire

    def close_third_speed_ctrl(self):
        self.calls.append("release")
        return True

    def send_third_expected_speed(self, twist):
        self.sent.append((twist.linear.x, twist.linear.y, twist.angular.z))
        return self.send_ok


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def session(nav, clock=None):
    clock = clock or FakeClock()
    return SpeedSession(nav, 50, clock=clock, sleep=clock.sleep), clock


def test_session_streams_until_deadline_then_zeroes_and_releases():
    nav = FakeNav()
    speed, _ = session(nav)
    status, _ = speed.run(0.1, 0.0, 0.2, 1.0, threading.Event())
    assert status == "completed"
    moving = [s for s in nav.sent if s != (0.0, 0.0, 0.0)]
    assert 45 <= len(moving) <= 51 and moving[0] == (0.1, 0.0, 0.2)
    assert nav.sent[-5:] == [(0.0, 0.0, 0.0)] * 5
    assert nav.calls == ["acquire", "release"]


def test_session_rate_must_exceed_vendor_minimum():
    with pytest.raises(ValueError):
        SpeedSession(FakeNav(), 20)


def test_session_refuses_without_speed_control_and_never_sends():
    nav = FakeNav(acquire=False)
    speed, _ = session(nav)
    status, message = speed.run(0.1, 0, 0, 1.0, threading.Event())
    assert status == "error" and "set_third_speed_ctrl" in message and nav.sent == []
    assert nav.calls == ["acquire"] * 3 + ["release"]


def test_duration_starts_when_odometry_shows_motion():
    # Steerable wheels turn first after a direction change; time the move from real motion.
    nav = FakeNav()
    speed, clock = session(nav)
    status, _ = speed.run(0.05, 0, 0, 1.0, threading.Event(), moving=lambda: clock.now >= 0.8)
    moving_sends = [x for x in nav.sent if x != (0.0, 0.0, 0.0)]
    assert status == "completed" and speed.start_delay_s == 0.8
    assert 88 <= len(moving_sends) <= 92  # 0.8 s turning + 1.0 s driving at 50 Hz


def test_steering_wait_is_capped():
    nav = FakeNav()
    speed, clock = session(nav)
    speed.run(0.05, 0, 0, 1.0, threading.Event(), moving=lambda: False)
    assert 5.0 <= speed.start_delay_s <= 5.05 and clock.now < 6.5  # never waits longer than START_GRACE_S


def test_stop_during_steering_wait_cancels():
    nav = FakeNav()
    speed, clock = session(nav)
    stop = threading.Event()
    status, _ = speed.run(0.05, 0, 0, 1.0, stop, moving=lambda: stop.set() or False)
    assert status == "cancelled" and nav.calls[-1] == "release"


def test_moving_check_needs_real_displacement_not_speed_spikes():
    events = []
    worker = Worker(FakeNav(), events.append, 50, odom=OdomTracker(events.append))
    check = worker._moving_check(0.05, 0, 0)
    assert check() is True  # no odometry yet: do not extend the move

    def odom(x, y=0.0, th=0.0, vx=0.0):
        worker._odom.on_odom(type("O", (), {"x": x, "y": y, "theta": th, "vx": vx, "vy": 0.0, "w": 0.0})())
    check = worker._moving_check(0.05, 0, 0)
    odom(0.131)
    assert check() is False  # origin recorded
    odom(0.129, vx=0.018)  # wheel-turning spike seen on the robot: fast but going nowhere
    assert check() is False
    odom(0.136)
    assert check() is False  # 5 mm forward: still turning/creeping
    odom(0.142)
    assert check() is True  # 11 mm along the command

    turn = worker._moving_check(0, 0, -0.17)
    odom(0.142, th=0.0)
    assert turn() is False
    odom(0.142, th=-0.03)  # turned 1.7 degrees clockwise
    assert turn() is True


def test_session_retries_acquire_rpc_timeouts_then_moves():
    # Seen on the robot after a restart: the first call times out while the chassis wakes.
    nav = FakeNav()
    failures = [RuntimeError("RPC call timeout: set_third_speed_ctrl")] * 2

    def acquire():
        nav.calls.append("acquire")
        if failures:
            raise failures.pop()
        return True
    nav.set_third_speed_ctrl = acquire
    acquired = []
    speed, _ = session(nav)
    status, _ = speed.run(0.1, 0, 0, 1.0, threading.Event(), on_acquired=lambda: acquired.append(1))
    assert status == "completed" and acquired == [1] and nav.calls[:3] == ["acquire"] * 3


def test_session_reports_acquire_timeouts_instead_of_crashing():
    nav = FakeNav()

    def acquire():
        raise RuntimeError("RPC call timeout: set_third_speed_ctrl")
    nav.set_third_speed_ctrl = acquire
    speed, _ = session(nav)
    status, message = speed.run(0.1, 0, 0, 1.0, threading.Event(), on_acquired=lambda: pytest.fail("not acquired"))
    assert status == "error" and "RPC call timeout" in message and "waking" in message
    assert nav.sent == [] and nav.calls == ["release"]


def test_worker_always_reports_move_done_even_if_the_session_raises():
    events = []
    worker = Worker(FakeNav(), events.append, 50)
    worker._session.run = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom"))
    worker._run("m1", threading.Event(), 0.1, 0, 0, 1.0)
    assert events[-1]["event"] == "move_done" and events[-1]["status"] == "error" and "boom" in events[-1]["message"]


def test_session_stops_on_cancel_block_send_failure_and_exception():
    nav = FakeNav()
    speed, _ = session(nav)
    stop = threading.Event()
    stop.set()
    assert speed.run(0.1, 0, 0, 5.0, stop)[0] == "cancelled"
    assert nav.sent == [] and nav.calls == ["release"]  # stopped before acquiring: nothing sent

    nav = FakeNav()
    speed, _ = session(nav)
    stop = threading.Event()
    speed.run(0.1, 0, 0, 5.0, stop, on_acquired=stop.set)  # cancelled after acquiring
    assert nav.sent == [(0.0, 0.0, 0.0)] * 5 and nav.calls == ["acquire", "release"]

    nav = FakeNav()
    speed, _ = session(nav)
    status, message = speed.run(0.1, 0, 0, 5.0, threading.Event(), blocked=lambda: "estop")
    assert (status, message) == ("error", "estop")

    nav = FakeNav(send_ok=False)
    speed, _ = session(nav)
    status, message = speed.run(0.1, 0, 0, 5.0, threading.Event())
    assert status == "error" and "repeatedly" in message and nav.calls[-1] == "release"

    nav = FakeNav()
    nav.send_third_expected_speed = lambda twist: (_ for _ in ()).throw(OSError("rpc down"))
    speed, _ = session(nav)
    status, message = speed.run(0.1, 0, 0, 5.0, threading.Event())
    assert status == "error" and "rpc down" in message and nav.calls[-1] == "release"


def test_worker_blocks_on_chassis_exception_level():
    events = []
    worker = Worker(FakeNav(), events.append, 50)
    item = type("E", (), {"exception_moudle": 720, "exception_code": 5})()
    worker._on_exception(type("Info", (), {"robot_exception": 2, "exceptions": [item]})())
    assert worker._blocked() == "chassis exception level 2"
    assert events[-1] == {"event": "exception", "level": 2, "exceptions": [{"module": 720, "code": 5}]}


# ── card logic with a fake link ─────────────────────────────────────────
class FakeLink:
    def __init__(self):
        self.sent = []
        self.exception_level = 0
        self.exception_supported = True
        self.exception_seen = True
        self.started = 0
        self.killed = 0
        self.closed = 0
        self.available = (True, "/opt/vmr-sdk/lib")

    def sdk_status(self):
        return self.available

    def ensure_started(self):
        if not self.available[0]:
            raise RuntimeError(self.available[1])
        self.started += 1

    def running(self):
        return self.started > 0

    def wait_exception_state(self, timeout_s=None):
        return self.exception_seen

    def send(self, message):
        self.sent.append(message)

    def kill(self):
        self.killed += 1

    def close(self):
        self.closed += 1


CONFIG = {"chassis_max_linear_m_s": 0.2, "chassis_max_angular_deg_s": 15, "chassis_motion_enabled": True}


def card(config=CONFIG, clock=time.monotonic):
    notes = []
    link = FakeLink()
    move = HeliosChassisMove(dict(config), link=link,
                             notify=lambda *args: notes.append(args), clock=clock)
    return move, link, notes


def test_tool_contract_matches_guarded_base_drive():
    move, _, _ = card()
    [definition] = move.get_tools()
    assert definition["name"] == "base_drive" and definition["type"] == "actuator"
    schema = definition["inputSchema"]
    assert schema["x-resource"] == "base"
    assert schema["x-completion"]["actions"] == ["move"]
    assert schema["x-action-params"]["move"]["params"] == ["vx", "vyaw", "duration_s"]
    assert schema["properties"]["vx"]["maximum"] == 0.2
    assert schema["properties"]["vyaw"]["maximum"] == 15
    assert "vy" not in schema["properties"]
    lateral, _, _ = card({**CONFIG, "chassis_allow_lateral": True})
    assert "vy" in lateral.get_tools()[0]["inputSchema"]["properties"]


@pytest.mark.parametrize("args, match", [
    ({"vx": 0.1}, "duration_s"),
    ({"vx": 0.1, "duration_s": 0}, "duration_s"),
    ({"vx": 0.1, "duration_s": 11}, "duration_s"),
    ({"vx": 0.3, "duration_s": 1}, "exceeds"),
    ({"vyaw": -20, "duration_s": 1}, "exceeds"),
    ({"vx": float("nan"), "duration_s": 1}, "finite"),
    ({"vx": True, "duration_s": 1}, "finite"),
    ({"vy": 0.1, "duration_s": 1}, "lateral"),
    ({"duration_s": 1}, "non-zero"),
])
def test_move_rejects_unsafe_or_ambiguous_commands(args, match):
    move, link, _ = card()
    with pytest.raises(ValueError, match=match):
        move.dispatch("move", args)
    assert link.sent == [] and link.started == 0


def test_hard_limit_cannot_be_raised_by_config():
    with pytest.raises(ValueError):
        card({"chassis_max_linear_m_s": 1.0})
    with pytest.raises(ValueError):
        card({"chassis_max_angular_deg_s": 90})


def test_move_sends_radians_and_completes_via_acp():
    move, link, notes = card()
    result = move.dispatch("move", {"vx": 0.1, "vyaw": 10, "duration_s": 2})
    assert result["state"] == "running"
    [sent] = link.sent
    assert sent["op"] == "move" and sent["id"] == result["action_id"]
    assert sent["wz"] == pytest.approx(math.radians(10)) and sent["duration_s"] == 2
    assert move.dispatch("move", {"vx": 0.1, "duration_s": 1})["error"].startswith("a timed base move")
    move._on_move_done(result["action_id"], "completed", "")
    assert notes == [(result["action_id"], "completed", {})]
    assert move.dispatch("info", {})["state"] == "ready"


def test_move_refused_when_disabled_sdk_missing_or_chassis_in_exception(monkeypatch):
    move, link, _ = card({**CONFIG, "chassis_motion_enabled": False})
    with pytest.raises(RuntimeError, match="disabled"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    monkeypatch.setenv("HELIOS_CHASSIS_MOTION_ENABLED", "0")
    move, link, _ = card()
    with pytest.raises(RuntimeError, match="disabled"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    monkeypatch.delenv("HELIOS_CHASSIS_MOTION_ENABLED")

    move, link, _ = card()
    link.available = (False, "vendor chassis SDK not mounted at /opt/vmr-sdk/lib")
    with pytest.raises(RuntimeError, match="not mounted"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    info = move.dispatch("info", {})
    assert info["sdk_available"] is False and "not mounted" in info["sdk_detail"]

    move, link, _ = card()
    link.exception_level = 1
    with pytest.raises(RuntimeError, match="exception level 1"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    assert link.sent == []


def test_motion_is_off_unless_the_deployment_opts_in(monkeypatch):
    monkeypatch.delenv("HELIOS_CHASSIS_MOTION_ENABLED", raising=False)
    move, link, _ = card({"chassis_max_linear_m_s": 0.2, "chassis_max_angular_deg_s": 15})
    with pytest.raises(RuntimeError, match="disabled"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    assert move.dispatch("info", {})["motion_enabled"] is False and link.sent == []
    monkeypatch.setenv("HELIOS_CHASSIS_MOTION_ENABLED", "1")
    move, link, _ = card({"chassis_max_linear_m_s": 0.2, "chassis_max_angular_deg_s": 15})
    assert move.dispatch("move", {"vx": 0.1, "duration_s": 1})["state"] == "running"


def test_move_fails_closed_while_exception_state_is_unknown():
    move, link, _ = card()
    link.exception_level, link.exception_seen = None, False
    with pytest.raises(RuntimeError, match="not received"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    link.exception_supported = False
    with pytest.raises(RuntimeError, match="no exception device"):
        move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    assert link.sent == []


def test_worker_blocks_until_first_exception_report():
    class Module:
        def exception_state(self, cb):
            self.cb = cb
    module = Module()
    worker = Worker(FakeNav(), lambda e: None, 50, exception_module=module)
    assert worker._blocked() == "chassis exception state not received yet"
    module.cb(type("Info", (), {"robot_exception": 0, "exceptions": []})())
    assert worker._blocked() is None
    assert Worker(FakeNav(), lambda e: None, 50)._blocked() is None  # no device: parent refuses instead


def test_stop_cancels_active_move_and_worker_exit_finishes_it():
    move, link, notes = card()
    first = move.dispatch("move", {"vx": 0.1, "duration_s": 5})
    assert move.dispatch("stop", {})["cancelled_move"] == first["action_id"]
    assert link.sent[-1] == {"op": "stop"}
    move._on_move_done(first["action_id"], "cancelled", "")
    assert notes[-1][:2] == (first["action_id"], "cancelled")

    second = move.dispatch("move", {"vx": -0.1, "duration_s": 5})
    move._on_move_done(None, "error", "chassis SDK worker exited")
    assert notes[-1][:2] == (second["action_id"], "error")


def test_watchdog_allows_motor_wake_up_until_speed_control_is_acquired():
    clock = FakeClock()
    move, link, notes = card(clock=clock)
    result = move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    clock.now = 11.0  # still acquiring (chassis waking): not cut off
    time.sleep(0.5)
    assert link.killed == 0 and not notes
    move._on_move_acquired(result["action_id"])  # acquired at t=11: 5 s start + 1 s move + 3 s margin
    clock.now = 19.5
    time.sleep(0.5)
    assert link.killed == 0 and not notes
    clock.now = 20.5
    for _ in range(50):
        if notes:
            break
        time.sleep(0.05)
    assert link.killed == 1 and notes[-1][:2] == (result["action_id"], "error")


def test_watchdog_kills_worker_when_completion_never_arrives():
    clock = FakeClock()
    move, link, notes = card(clock=clock)
    result = move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    clock.now = 30.0
    for _ in range(50):
        if notes:
            break
        time.sleep(0.05)
    assert link.killed == 1 and notes[-1][:2] == (result["action_id"], "error")


def test_bundle_routes_base_drive_alongside_other_cards():
    move, _, _ = card()
    bundle = DriverBundle([move])
    assert bundle.dispatch("base_drive", {"action": "info"})["limits"]["max_linear_m_s"] == 0.2


# ── end to end through a real worker process and a fake SDK ─────────────
FAKE_SDK = {
    "py_client_common.py": """
        class ClientConfig:
            pass
        def init_client(conf):
            return conf.ip == "127.0.0.1"
        def close_client():
            pass
    """,
    "odometry.py": """
        class _O:
            x = 1.0; y = 2.0; theta = 0.0; vx = 0.1; vy = 0.0; w = 0.0
        def has_odom_dev():
            return True
        def odom_state(cb):
            import threading, time
            def run():
                o = _O()
                while True:
                    o.x += 0.002
                    cb(o)
                    time.sleep(0.02)
            threading.Thread(target=run, daemon=True).start()
    """,
    "exception.py": """
        class _Info:
            robot_exception = 0
            exceptions = []
        def has_exception_dev():
            return True
        def exception_state(cb):
            import threading
            threading.Timer(0.05, cb, args=(_Info(),)).start()
    """,
    "nav.py": """
        import json, os
        LOG = os.environ["FAKE_NAV_LOG"]
        class Vector3:
            pass
        class Twist:
            pass
        def _log(*item):
            with open(LOG, "a") as f:
                f.write(json.dumps(item) + "\\n")
        def has_nav_dev():
            return True
        def set_third_speed_ctrl():
            _log("acquire"); return True
        def close_third_speed_ctrl():
            _log("release"); return True
        def send_third_expected_speed(t):
            _log("speed", t.linear.x, t.angular.z); return True
    """,
}


def fake_sdk(tmp_path):
    lib = tmp_path / "sdk" / "lib"
    lib.mkdir(parents=True)
    for name, body in FAKE_SDK.items():
        (lib / name).write_text(textwrap.dedent(body))
    return tmp_path / "sdk"


def read_log(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def test_worker_process_moves_completes_and_stops_on_parent_exit(tmp_path, monkeypatch):
    sdk = fake_sdk(tmp_path)
    log = tmp_path / "nav.log"
    monkeypatch.setenv("FAKE_NAV_LOG", str(log))
    done = []
    link = SdkWorkerLink({"chassis_sdk_dir": str(sdk), "chassis_sdk_host": "127.0.0.1",
                          "chassis_sdk_port": 9003, "chassis_sdk_python": sys.executable},
                         lambda *event: done.append(event))
    link.ensure_started()
    assert link.running() and link.exception_supported
    assert link.wait_exception_state(2) and link.exception_level == 0
    link.send({"op": "move", "id": "m1", "vx": 0.1, "vy": 0.0, "wz": 0.1, "duration_s": 0.3})
    for _ in range(100):
        if done:
            break
        time.sleep(0.02)
    assert done[0][:2] == ("m1", "completed")
    odometry = done[0][3]
    assert odometry and odometry["forward_m"] > 0 and abs(odometry["left_m"]) < 1e-6
    assert link.odometry and link.odometry["y"] == 2.0
    entries = read_log(log)
    assert entries[0] == ["acquire"] and entries[-1] == ["release"]
    assert ["speed", 0.1, 0.1] in entries and entries[-2] == ["speed", 0.0, 0.0]

    # A long move is cut off when the Driver side goes away.
    link.send({"op": "move", "id": "m2", "vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 9})
    time.sleep(0.2)
    link.close()
    entries = read_log(log)
    assert entries[-1] == ["release"] and entries[-2] == ["speed", 0.0, 0.0]


def test_worker_reports_fatal_when_chassis_unreachable(tmp_path, monkeypatch):
    sdk = fake_sdk(tmp_path)
    monkeypatch.setenv("FAKE_NAV_LOG", str(tmp_path / "nav.log"))
    link = SdkWorkerLink({"chassis_sdk_dir": str(sdk), "chassis_sdk_host": "10.0.0.1",
                          "chassis_sdk_port": 9003, "chassis_sdk_python": sys.executable},
                         lambda *event: None)
    with pytest.raises(RuntimeError, match="cannot connect"):
        link.ensure_started(timeout_s=5)


def test_missing_sdk_is_reported_without_starting_a_process(tmp_path):
    link = SdkWorkerLink({"chassis_sdk_dir": str(tmp_path / "absent")}, lambda *e: None,
                         popen=lambda *a, **k: pytest.fail("must not spawn"))
    available, detail = link.sdk_status()
    assert not available and "not mounted" in detail
    with pytest.raises(RuntimeError, match="not mounted"):
        link.ensure_started()


def test_odometry_displacement_is_in_start_body_frame():
    start = {"x": 1.0, "y": 1.0, "theta": math.pi / 2}
    end = {"x": 1.0, "y": 1.5, "theta": math.pi / 2 + math.radians(10)}
    moved = OdomTracker.displacement(start, end)
    assert moved["forward_m"] == pytest.approx(0.5) and moved["left_m"] == pytest.approx(0.0, abs=1e-9)
    assert moved["distance_m"] == pytest.approx(0.5) and moved["yaw_deg"] == pytest.approx(10.0)
    assert OdomTracker.displacement(None, end) is None


def test_odom_tracker_throttles_and_expires():
    events, clock = [], FakeClock()
    tracker = OdomTracker(events.append, clock=clock, period_s=0.5)
    sample = type("O", (), {"x": 0.1, "y": 0.0, "theta": 0.0, "vx": 0.05, "vy": 0.0, "w": float("nan")})()
    for _ in range(10):
        tracker.on_odom(sample)
        clock.now += 0.1
    assert 2 <= len(events) <= 3 and "w" not in events[0]
    assert tracker.snapshot()["vx"] == 0.05
    clock.now += 2.0
    assert tracker.snapshot() is None


def test_move_result_and_info_carry_odometry():
    move, link, notes = card()
    link.odometry = {"x": 0.5, "y": 0.0, "theta": 0.0, "vx": 0.0, "vy": 0.0, "w": 0.0,
                     "received_monotonic": time.monotonic()}
    assert move.dispatch("info", {})["odometry"]["x"] == 0.5
    result = move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    move._on_move_done(result["action_id"], "completed", "", {"forward_m": 0.1, "yaw_deg": 0.0})
    assert notes[-1][2]["odometry"]["forward_m"] == 0.1


def test_missing_interpreter_is_reported(tmp_path):
    lib = tmp_path / "sdk" / "lib"
    lib.mkdir(parents=True)
    link = SdkWorkerLink({"chassis_sdk_dir": str(tmp_path / "sdk"), "chassis_sdk_host": "x",
                          "chassis_sdk_port": 1, "chassis_sdk_python": "/opt/absent/python3"},
                         lambda *e: None, popen=lambda *a, **k: pytest.fail("must not spawn"))
    with pytest.raises(RuntimeError, match="interpreter not found"):
        link.ensure_started()


def test_info_connects_read_only_and_reports_errors():
    move, link, _ = card()
    move.dispatch("info", {})
    assert link.started == 1 and link.sent == []  # connected, no speed command
    move, link, _ = card()
    link.ensure_started = lambda: (_ for _ in ()).throw(RuntimeError("cannot connect to chassis RPC"))
    assert "cannot connect" in move.dispatch("info", {})["connect_error"]


def test_stop_move_is_a_public_canvas_action_and_stops_like_stop():
    move, link, notes = card()
    schema = move.get_tools()[0]["inputSchema"]
    assert "stop_move" in schema["properties"]["action"]["enum"]
    assert "stop_move" in schema["x-action-params"]
    active = move.dispatch("move", {"vx": 0.1, "duration_s": 5})
    result = move.dispatch("stop_move", {})
    assert result["state"] == "idle" and result["cancelled_move"] == active["action_id"]
    assert link.sent[-1] == {"op": "stop"}


def test_lateral_field_is_listed_right_after_vx():
    move, _, _ = card({**CONFIG, "chassis_allow_lateral": True})
    schema = move.get_tools()[0]["inputSchema"]
    assert list(schema["properties"])[:5] == ["action", "vx", "vy", "vyaw", "duration_s"]
    assert schema["x-action-params"]["move"]["params"] == ["vx", "vy", "vyaw", "duration_s"]


def test_move_diagnostics_reach_the_result_and_info():
    nav = FakeNav()
    speed, clock = session(nav)
    speed.run(0.05, 0, 0, 1.0, threading.Event(), moving=lambda: clock.now >= 0.4)
    assert speed.stats["start_reason"] == "moving" and speed.stats["acquire_s"] == 0
    assert speed.stats["commands_sent"] >= 70 and 1.3 <= speed.stats["streamed_s"] <= 1.45
    speed, clock = session(FakeNav())
    speed.run(0.05, 0, 0, 1.0, threading.Event(), moving=lambda: False)
    assert speed.stats["start_reason"] == "grace"

    move, link, notes = card()
    result = move.dispatch("move", {"vx": 0.1, "duration_s": 1})
    move._on_move_done(result["action_id"], "completed", "", {"forward_m": 0.06}, 0.8,
                       {"start_reason": "distance", "commands_sent": 90})
    last = move.dispatch("info", {})["last_move"]
    assert last["status"] == "completed" and last["start_delay_s"] == 0.8
    assert last["diagnostics"]["start_reason"] == "distance" and last["odometry"]["forward_m"] == 0.06


def test_exception_history_keeps_only_changes():
    import io
    lines = [json.dumps({"event": "exception", "level": lvl, "exceptions": exc}) + "\n" for lvl, exc in
             [(0, [{"module": 2, "code": 7}]), (1, [{"module": 9, "code": 9}])] + [(0, [{"module": 116, "code": 1}])] * 20]
    link = SdkWorkerLink({}, lambda *a: None)
    link._read_events(type("P", (), {"stdout": io.StringIO("".join(lines))})())
    assert [h["level"] for h in link.exception_history] == [0, 1, 0]


def _info(level, codes):
    items = [type("E", (), {"exception_moudle": m, "exception_code": c})() for m, c in codes]
    return type("Info", (), {"robot_exception": level, "exceptions": items})()


def test_wake_brake_is_tolerated_only_before_motion_and_only_briefly():
    class Module:
        def exception_state(self, cb):
            self.cb = cb
    clock = FakeClock()
    module = Module()
    worker = Worker(FakeNav(), lambda e: None, 50, exception_module=module, clock=clock)
    worker._session.stats = {"start_reason": None}  # still waiting for the base to move
    module.cb(_info(1, [(116, 1), (2, 2)]))  # brake while the motors wake (recorded on the robot)
    assert worker._blocked() is None
    clock.now = 1.9
    assert worker._blocked() is None
    clock.now = 2.1
    assert worker._blocked() == "chassis exception level 1"  # brake held too long
    module.cb(_info(0, [(116, 1)]))
    assert worker._blocked() is None

    module.cb(_info(1, [(116, 1), (311, 1)]))  # localisation timeout: never tolerated
    assert worker._blocked() == "chassis exception level 1"
    module.cb(_info(2, [(2, 2)]))  # emergency stop level: never tolerated
    assert worker._blocked() == "chassis exception level 2"
    worker._session.stats = {"start_reason": "distance"}  # already moving: any level stops
    module.cb(_info(1, [(116, 1), (2, 2)]))
    assert worker._blocked() == "chassis exception level 1"
