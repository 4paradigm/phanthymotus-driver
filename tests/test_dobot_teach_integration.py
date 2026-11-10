"""Real local TCP/HTTP integration; never connects to a robot or Jetson."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ssl
import subprocess
from urllib.request import Request, urlopen

import pytest
from dobot_sdk.core.connection import DobotConnection

from common.vendor_runtime import DriverBundle, make_handler
from dobot.cra.control import DashboardConnection, RobotController
from dobot.cra.device import DobotSensors
from dobot.cra.feedback import FeedbackClient
from dobot_teach_simulator import RobotSimulator
from test_dobot_teach import recorded_store, recording


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("condition not reached")


@pytest.fixture
def rig(tmp_path, monkeypatch):
    store = recorded_store(tmp_path)
    robot = RobotSimulator(store.controller_root)
    monkeypatch.setenv("DOBOT_CONTROLLER_IP", "127.0.0.1")
    monkeypatch.setenv("DOBOT_FEEDBACK_PORT", "30004")
    monkeypatch.setenv("DOBOT_MODEL_NAME", "CR3A")
    config = {"controller_ip": "127.0.0.1", "feedback_port": 30004, "model_name": "CR3A",
              "receive_timeout_s": .2, "stale_timeout_s": .2, "connect_timeout_s": .2,
              "joint_move": {"enable_timeout_s": 1, "receive_timeout_s": .3, "motion_timeout_s": 2},
              "teach": {"drag_exit_timeout_s": 1}}

    def feedback_connection(address, port):
        connection = DobotConnection("127.0.0.1", 30004)
        connection.port = robot.feedback_port
        return connection

    def dashboard_connection(address, port):
        connection = DashboardConnection("127.0.0.1", 29999)
        connection.port = robot.dashboard_port
        return connection

    client = FeedbackClient(config, connection_factory=feedback_connection)
    control = RobotController(client, config, connection_factory=dashboard_connection)
    plugin = DobotSensors(config, client=client, controller=control)
    plugin.teach.store = store
    plugin.teach.poll_interval = plugin.completion.poll_interval = .005
    events = []
    plugin.teach._notify = plugin.completion._notify = lambda *args: events.append(args)
    client.start()
    wait_until(lambda: client.snapshot()["connected"])
    yield plugin, robot, store, events
    plugin.stop()
    robot.close()


def dispatch(plugin, action, **args):
    return plugin.dispatch(action, {"_tool_name": "teach", "confirm_move": True, **args})


def terminal(events, result, status="completed", timeout=3):
    identifier = result["action_id"]
    wait_until(lambda: any(event[0] == identifier for event in events), timeout)
    event = next(event for event in events if event[0] == identifier)
    assert event[1] == status, event
    return event[2]


def manual_drag(robot, active=True, entry_packets=0):
    """Simulate the operator pressing the drag button on the arm. On the field
    CR3A, entering drag reports ~3 packets of mode 0 (enabled, drag_status=1)."""
    with robot.lock:
        robot._advance()
        robot.mode = 6 if active else 5
        robot.entry_packets = entry_packets if active else 0
        if not active:
            # Field CR3A: one mode-5 packet still reports RunningStatus=1 after drag.
            robot.hand_velocity, robot.running_packets = 0, 1


def release_later(robot, delay=.15):
    """The operator presses the drag button again after stop_record was sent."""
    timer = threading.Timer(delay, manual_drag, args=(robot, False))
    timer.start()
    return timer


def perform_recording(rig):
    plugin, robot, store, events = rig
    event = terminal(events, dispatch(plugin, "start_record", name="本地端到端测试"))
    assert event["state"] == "recording" and event["drag"] == "waiting_for_first_manual_drag"
    identifier = event["trajectory_id"]
    time.sleep(.05)
    manual_drag(robot, entry_packets=3)
    with robot.lock:
        robot.hand_velocity = 30
    time.sleep(.25)
    stop = dispatch(plugin, "stop_record")
    release_later(robot).join()
    result = terminal(events, stop)
    assert result["state"] == "recorded" and result["native_state"] == "exported"
    assert result["point_count"] >= 3
    return identifier


def test_record_go_to_start_native_play_and_completion_over_real_local_tcp(rig):
    plugin, robot, store, events = rig
    identifier = perform_recording(rig)
    commands = list(robot.commands)
    assert commands[0] == "ClearError()" and "StartDrag()" not in commands
    # stop_record during a button drag never sends StopDrag; the operator exits.
    assert "StopDrag()" not in commands
    blocked = terminal(events, dispatch(plugin, "play", trajectory_id=identifier), status="error")
    assert "go_to_start" in blocked["reason"]
    assert not any(command.startswith("StartPath") for command in robot.commands)
    terminal(events, dispatch(plugin, "go_to_start", trajectory_id=identifier))
    result = terminal(events, dispatch(plugin, "play", trajectory_id=identifier))
    assert result["trajectory_id"] == identifier and result["reason"] == "target_reached"
    assert sum(command.startswith("StartPath") for command in robot.commands) == 1
    assert robot.mode == 5 and plugin.control.ownership.owner is None
    assert any("sample=50" in command and "freq=1.0" in command for command in robot.commands)


@pytest.mark.parametrize("path_id", [False, True])
def test_pause_zero_flag_resume_keeps_queue_and_skin_configuration(rig, path_id):
    plugin, robot, store, events = rig
    robot.return_path_id = path_id
    record, _ = recording(store, [0, 1, 2, 3, 4, 5, 6, 7])
    identifier = record["trajectory_id"]
    played = dispatch(plugin, "play", trajectory_id=identifier, speed_multiplier=.25, detect_collision=True)
    wait_until(lambda: robot.motion is not None and robot.mode == 7)
    terminal(events, dispatch(plugin, "pause"))
    terminal(events, played, status="cancelled")
    assert plugin.client.snapshot()["data"]["pause_cmd_flag"] == 0
    assert plugin.control.ownership.owner == "teach"
    previous = len(robot.commands)
    result = terminal(events, dispatch(plugin, "resume"))
    assert robot.commands[previous:] == ["Continue()"]
    assert robot.skin == 1 and robot.skin_setting == [1, 1, 1, 1]
    assert result["trajectory_id"] == identifier and plugin.control.ownership.owner is None


def test_two_cards_cannot_start_competing_motion_and_stop_routes_to_owner(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [0, 1, 2, 3, 4, 5, 6, 7])
    played = dispatch(plugin, "play", trajectory_id=record["trajectory_id"], speed_multiplier=.25)
    wait_until(lambda: robot.motion is not None)
    count = len(robot.commands)
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 10, "confirm_move": True})
    assert len(robot.commands) == count
    stopped = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
    terminal(events, stopped)
    terminal(events, played, status="cancelled")
    assert robot.mode == 5 and plugin.control.ownership.owner is None
    with pytest.raises(RuntimeError, match="no teaching motion"):
        dispatch(plugin, "resume")


def test_recording_owns_arm_after_start_transition_completes(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="record"))
    assert plugin.teach._active is None
    assert plugin.control.ownership.owner == "teach"
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 5, "confirm_move": True})
    with pytest.raises(RuntimeError, match="active arm operation"):
        plugin.control.reconfigure("127.0.0.2", 30004)
    stopped = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
    result = terminal(events, stopped)
    assert result["state"] == "incomplete" and robot.mode == 5
    assert plugin.control.ownership.owner is None


def test_missing_native_profile_never_sends_preparation_or_motion(rig):
    plugin, robot, store, events = rig
    store.profile_path = ""
    record, _ = recording(store)
    result = terminal(events, dispatch(plugin, "play", trajectory_id=record["trajectory_id"]), status="error")
    assert "format_required" in result["reason"]
    assert robot.commands == []


def test_start_record_while_already_dragging_is_rejected_without_commands(rig):
    plugin, robot, store, events = rig
    manual_drag(robot)
    wait_until(lambda: plugin.client.snapshot()["data"]["drag_status"])
    result = terminal(events, dispatch(plugin, "start_record", name="already dragging"), status="error")
    assert "stationary" in result["reason"]
    assert robot.commands == []
    assert store.list_records()[0]["state"] == "incomplete"
    assert plugin.control.ownership.owner is None


def move_j_rejected(plugin):
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 5, "confirm_move": True})


def drag_cycle(robot, drag_s=.2, pause_s=.2):
    manual_drag(robot, entry_packets=3)
    with robot.lock:
        robot.hand_velocity = 30
    time.sleep(drag_s)
    manual_drag(robot, False)
    time.sleep(pause_s)


def test_multiple_drags_continue_one_recording_until_stop_record_in_pause(rig):
    plugin, robot, store, events = rig
    started = dispatch(plugin, "start_record", name="multi drag")
    identifier = terminal(events, started)["trajectory_id"]
    time.sleep(.1)
    for _ in range(2):
        drag_cycle(robot)
        assert not plugin.teach.recorder.finished.is_set()
        assert plugin.control.ownership.owner == "teach"
        move_j_rejected(plugin)
    # Entering or leaving drag never produces another ACP completion.
    assert len(events) == 1
    result = terminal(events, dispatch(plugin, "stop_record"))
    assert result["state"] == "recorded" and result["end_reason"] == "stop_record"
    assert result["native_state"] == "exported" and result["duration_s"] >= .5
    assert store.load(identifier)["drag_entries"] == 2
    assert "StartDrag()" not in robot.commands and "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None
    points = store.samples(identifier)["points"]
    assert [p["time_ms"] for p in points] == list(range(0, len(points) * 50, 50))
    # Idle pauses stay on the axis as runs of unchanged angles, not removed.
    angles = [p["joints_deg"][0] for p in points]
    assert sum(abs(a - b) < 1e-9 for a, b in zip(angles, angles[1:])) >= 4
    assert angles[-1] > angles[0]


def test_stop_record_waits_for_mode5_coasting_to_reach_rest(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="coasting"))
    drag_cycle(robot, pause_s=0)
    robot.joint_speed = [5, 0, 0, 0, 0, 0]
    stop = dispatch(plugin, "stop_record")
    time.sleep(.3)
    assert not any(event[0] == stop["action_id"] for event in events)
    robot.joint_speed = None
    result = terminal(events, stop)
    assert result["state"] == "recorded" and "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None


def test_unconfirmed_exit_is_not_success_and_keeps_the_arm_until_stop_motion(rig):
    plugin, robot, store, events = rig
    identifier = terminal(events, dispatch(plugin, "start_record", name="never settles"))["trajectory_id"]
    drag_cycle(robot, pause_s=0)
    robot.joint_speed = [5, 0, 0, 0, 0, 0]
    result = terminal(events, dispatch(plugin, "stop_record"), status="error", timeout=5)
    assert "could not confirm exit from drag" in result["reason"]
    record = store.load(identifier)
    assert record["state"] == "incomplete" and "could not confirm" in record["reason"]
    assert plugin.control.ownership.owner == "teach"
    move_j_rejected(plugin)
    robot.joint_speed = None
    stopped = terminal(events, plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True}))
    assert stopped["state"] == "incomplete"
    assert plugin.control.ownership.owner is None


@pytest.mark.parametrize("interrupt", ["stop_motion", "driver_stop"])
def test_interruption_during_exit_confirmation_is_never_saved_as_recorded(rig, interrupt):
    plugin, robot, store, events = rig
    identifier = terminal(events, dispatch(plugin, "start_record", name="late interrupt"))["trajectory_id"]
    drag_cycle(robot, pause_s=0)
    robot.joint_speed = [5, 0, 0, 0, 0, 0]  # keeps the exit confirmation pending
    stop = dispatch(plugin, "stop_record")
    wait_until(lambda: plugin.teach.recorder.stop_event.is_set())
    time.sleep(.1)
    assert not plugin.teach.recorder.finished.is_set()
    settle = threading.Timer(.3, lambda: setattr(robot, "joint_speed", None))
    settle.start()
    try:
        if interrupt == "stop_motion":
            stopped = terminal(events, plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True}))
            assert stopped["state"] == "incomplete"
            expected = "recording cancelled"
        else:
            plugin.dispatch("stop", {"_tool_name": "teach"})
            expected = "driver stopped during recording"
    finally:
        settle.join()
    wait_until(lambda: any(event[0] == stop["action_id"] for event in events))
    assert next(event for event in events if event[0] == stop["action_id"])[1] != "completed"
    record = store.load(identifier)
    assert record["state"] == "incomplete" and record["end_reason"] == "interrupted"
    assert expected in record["reason"]
    assert plugin.control.ownership.owner is None


def test_feedback_loss_during_exit_confirmation_is_not_reported_as_success(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="lost confirmation"))
    manual_drag(robot)
    time.sleep(.1)
    robot.feedback_enabled = False
    result = terminal(events, dispatch(plugin, "stop_record"), status="error", timeout=5)
    assert "incomplete" in result["reason"]
    assert "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner == "teach"
    robot.feedback_enabled = True
    manual_drag(robot, False)
    plugin.teach.drag_exit_timeout = 4  # covers the 1 s feedback reconnect delay
    terminal(events, dispatch(plugin, "stop_motion"), timeout=8)
    assert plugin.control.ownership.owner is None


@pytest.mark.parametrize("mode", [7, 10])
def test_non_idle_mode_during_pause_ends_incomplete_without_recovery_commands(rig, mode):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="bad pause"))
    drag_cycle(robot, pause_s=.1)
    count = len(robot.commands)
    with robot.lock:
        robot.mode = mode
        robot.setting_until = time.monotonic() + .3  # hold simulated mode 7 briefly
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "incomplete" and f"controller mode {mode}" in record["reason"]
    assert record["end_reason"] == "interrupted" and record["point_count"] >= 3
    assert not any(c in robot.commands[count:] for c in ("ClearError()", "EnableRobot()", "Continue()"))
    if mode == 7:
        # The simulated motion returns to rest in mode 5, so the exit is confirmed.
        assert plugin.control.ownership.owner is None
        return
    # A paused queue never confirms rest in mode 5, so the arm stays reserved.
    assert plugin.control.ownership.owner == "teach"
    with robot.lock:
        robot.mode = 5
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.control.ownership.owner is None


def test_recording_limit_waits_for_button_exit_and_saves_as_recorded(rig):
    plugin, robot, store, events = rig
    plugin.teach.max_duration = .5
    terminal(events, dispatch(plugin, "start_record", name="limit"))
    manual_drag(robot, entry_packets=3)
    with robot.lock:
        robot.hand_velocity = 30
    wait_until(lambda: plugin.teach.recorder.limit_reached)
    time.sleep(.1)
    assert not plugin.teach.recorder.finished.is_set()  # still waiting for the operator
    manual_drag(robot, False)
    wait_until(lambda: plugin.teach.recorder.finished.is_set())
    record = plugin.teach.recorder.record
    assert record["state"] == "recorded" and record["end_reason"] == "limit_reached"
    assert "StopDrag()" not in robot.commands and robot.mode == 5
    assert plugin.control.ownership.owner is None
    assert terminal(events, dispatch(plugin, "stop_record"))["end_reason"] == "limit_reached"


def test_drag_not_exited_by_operator_never_sends_stopdrag_and_keeps_the_arm(rig):
    plugin, robot, store, events = rig
    identifier = terminal(events, dispatch(plugin, "start_record", name="button still held"))["trajectory_id"]
    manual_drag(robot, entry_packets=3)
    time.sleep(.15)
    result = terminal(events, dispatch(plugin, "stop_record"), status="error", timeout=5)
    assert "press the drag button" in result["reason"]
    assert store.load(identifier)["state"] == "incomplete"
    assert "StopDrag()" not in robot.commands and robot.mode == 6
    assert plugin.control.ownership.owner == "teach"
    move_j_rejected(plugin)
    # stop_motion also waits for the button, then releases the arm.
    stop = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
    release_later(robot).join()
    terminal(events, stop, timeout=5)
    assert plugin.control.ownership.owner is None and "StopDrag()" not in robot.commands


def test_mode6_without_drag_status_is_reported_as_inconsistent(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="inconsistent"))
    drag_cycle(robot, pause_s=.05)
    with robot.lock:
        robot.mode, robot.drag_status = 6, False
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "incomplete" and "drag state inconsistent" in record["reason"]
    assert "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner == "teach"
    with robot.lock:
        robot.mode, robot.drag_status = 5, None
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.control.ownership.owner is None


def test_long_mode0_is_not_accepted_as_drag_entry(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="mode 0 too long"))
    manual_drag(robot, entry_packets=40)  # ~320 ms of mode 0
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "incomplete" and "mode 0 lasting over 100 ms" in record["reason"]
    manual_drag(robot, False)
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.control.ownership.owner is None


def test_waiting_limit_without_drag_is_incomplete(rig):
    plugin, robot, store, events = rig
    plugin.teach.max_duration = .3
    terminal(events, dispatch(plugin, "start_record", name="no drag"))
    wait_until(lambda: plugin.teach.recorder.finished.is_set())
    record = plugin.teach.recorder.record
    assert record["state"] == "incomplete" and "no manual drag within" in record["reason"]
    assert "StopDrag()" not in robot.commands and plugin.control.ownership.owner is None


def test_persistence_failure_reports_error(rig, monkeypatch):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="disk full"))
    drag_cycle(robot)

    def full(record):
        raise OSError("disk full")
    monkeypatch.setattr(store, "save", full)
    result = terminal(events, dispatch(plugin, "stop_record"), status="error")
    assert "persistence failed" in result["reason"]
    assert plugin.teach.recorder.record["state"] == "incomplete"


def test_native_export_failure_keeps_local_recording_recorded(rig, monkeypatch):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="share offline"))
    drag_cycle(robot)

    def offline(identifier):
        raise RuntimeError("controller share offline")
    monkeypatch.setattr(store, "export", offline)
    result = terminal(events, dispatch(plugin, "stop_record"))
    assert result["state"] == "recorded" and result["reason"] is None
    assert result["native_state"] == "export_failed" and "offline" in result["native_reason"]
    assert plugin.control.ownership.owner is None


def test_stop_record_before_manual_drag_reports_no_drag(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="never dragged"))
    time.sleep(.1)
    result = terminal(events, dispatch(plugin, "stop_record"), status="error")
    assert "no manual drag" in result["reason"]
    assert "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None


def test_feedback_loss_during_recording_saves_incomplete_after_button_exit(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="drop"))
    plugin.teach.drag_exit_timeout = 4  # covers the 1 s feedback reconnect delay
    manual_drag(robot)
    time.sleep(.1)
    robot.feedback_enabled = False
    time.sleep(.4)  # recorder ends on missing feedback, then waits for the button exit
    robot.feedback_enabled = True
    manual_drag(robot, False)
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 8)
    assert plugin.teach.recorder.record["state"] == "incomplete"
    assert "feedback" in plugin.teach.recorder.record["reason"]
    assert robot.mode == 5 and "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None


def test_profile_mismatch_stops_before_any_motion(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store)
    original = plugin.control.get_start_pose
    plugin.control.get_start_pose = lambda name: [100, 0, 0, 0, 0, 0]
    result = terminal(events, dispatch(plugin, "go_to_start", trajectory_id=record["trajectory_id"]), status="error")
    assert "does not match" in result["reason"]
    assert not robot.commands
    plugin.control.get_start_pose = original


def test_mcp_http_card_discovery_recording_and_results(rig):
    plugin, robot, store, events = rig
    bundle = DriverBundle([plugin])
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "dobot-cra", "dobot-cra-driver"))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def rpc(method, params):
        request = Request(f"http://127.0.0.1:{server.server_port}/mcp",
                          data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode(),
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=2) as response:
            return json.load(response)

    def call(action, **args):
        reply = rpc("tools/call", {"name": "teach", "arguments": {"action": action, **args}})
        assert "error" not in reply, reply
        return json.loads(reply["result"]["content"][0]["text"])

    try:
        tools = rpc("tools/list", {})["result"]["tools"]
        assert {tool["name"] for tool in tools} == {"joints", "joint_move", "teach", "model"}
        terminal(events, call("start_record", name="HTTP recording", confirm_move=True))
        manual_drag(robot)
        time.sleep(.15)
        manual_drag(robot, False)
        time.sleep(.05)
        result = terminal(events, call("stop_record", confirm_move=True))
        assert result["state"] == "recorded"
        listed = call("list_records")
        assert listed["records"][0]["trajectory_id"] == result["trajectory_id"]
        assert call("info")["name"] == "teach"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1)


def test_verified_https_acp_retry_keeps_action_id_tool_and_does_not_repeat_robot_command(rig, tmp_path, monkeypatch):
    plugin, robot, store, events = rig
    cert, key, openssl_config = [tmp_path / name for name in ("cert.pem", "key.pem", "openssl.cnf")]
    openssl_config.write_text("[req]\ndistinguished_name=dn\nx509_extensions=ext\nprompt=no\n"
                              "[dn]\nCN=localhost\n[ext]\nsubjectAltName=DNS:localhost\n"
                              "basicConstraints=critical,CA:TRUE\n")
    subprocess.run(["/usr/bin/openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                    "-keyout", str(key), "-out", str(cert), "-config", str(openssl_config)],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=10)
    received = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            assert self.path == "/api/acp/complete"
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            received.append(payload)
            # First ACK is deliberately invalid; only delivery may be retried.
            reply = json.dumps({"ok": len(received) > 1, "action_id": payload["action_id"]}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    tls.load_cert_chain(cert, key)
    server.socket = tls.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    monkeypatch.setenv("AGENT_CORE_URL", f"https://localhost:{server.server_port}")
    monkeypatch.setenv("AGENT_CORE_CA_CERT", str(cert))
    plugin.teach._notify = None
    try:
        start = dispatch(plugin, "start_record", name="real TLS callback")
        wait_until(lambda: plugin.teach.last_completion and plugin.teach.last_completion["callback"] == "accepted")
        assert [item["action_id"] for item in received] == [start["action_id"]] * 2
        assert all(item["tool"] == "teach" and item["status"] == "completed" for item in received)
        stop = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
        wait_until(lambda: any(item["action_id"] == stop["action_id"] for item in received))
        stopped = next(item for item in received if item["action_id"] == stop["action_id"])
        assert stopped["tool"] == "joint_move" and stopped["status"] == "completed"
        assert robot.commands.count("StartDrag()") == 0
        assert robot.commands.count("StopDrag()") == 0
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1)


def test_disabled_recording_prepares_enable_without_changing_skin(rig):
    plugin, robot, store, events = rig
    with robot.lock:
        robot.mode, robot.enabled, robot.skin = 4, False, 1
    wait_until(lambda: plugin.client.snapshot()["data"]["robot_mode"] == 4)
    terminal(events, dispatch(plugin, "start_record", name="disabled"))
    assert robot.commands == ["ClearError()", "EnableRobot()"]
    assert robot.skin == 1
    manual_drag(robot)
    time.sleep(.1)
    manual_drag(robot, False)
    time.sleep(.05)
    terminal(events, dispatch(plugin, "stop_record"))


def test_protection_during_playback_stops_once_and_reports_error(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [0, 1, 2, 3, 4, 5])
    play = dispatch(plugin, "play", trajectory_id=record["trajectory_id"], speed_multiplier=.25)
    wait_until(lambda: robot.motion is not None)
    robot.collision = True
    result = terminal(events, play, status="error")
    assert "protection" in result["reason"]
    assert robot.commands.count("Stop()") == 1
    assert robot.commands.count("Continue()") == 0


def test_resume_rejected_by_protection_does_not_clear_or_lose_paused_context(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [0, 1, 2, 3, 4, 5])
    play = dispatch(plugin, "play", trajectory_id=record["trajectory_id"], speed_multiplier=.25)
    wait_until(lambda: robot.motion is not None)
    terminal(events, dispatch(plugin, "pause"))
    terminal(events, play, status="cancelled")
    robot.collision = True
    wait_until(lambda: plugin.client.snapshot()["data"]["collision_state"])
    count = len(robot.commands)
    terminal(events, dispatch(plugin, "resume"), status="error")
    assert len(robot.commands) == count
    assert plugin.control.ownership.owner == "teach" and plugin.teach._resume_context is not None
    robot.collision = False
    wait_until(lambda: not plugin.client.snapshot()["data"]["collision_state"])
    terminal(events, dispatch(plugin, "stop_motion"))


def test_paused_recording_request_never_exits_someone_elses_drag_or_clears_queue(rig):
    plugin, robot, store, events = rig
    robot.mode = 10
    wait_until(lambda: plugin.client.snapshot()["data"]["robot_mode"] == 10)
    result = terminal(events, dispatch(plugin, "start_record", name="bad state"), status="error")
    assert "paused" in result["reason"]
    assert robot.commands == []


def test_card_stop_cancels_recording_and_exits_drag(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="stop lifecycle"))
    time.sleep(.1)
    result = plugin.dispatch("stop", {"_tool_name": "teach"})
    assert result["state"] == "idle"
    assert plugin.teach.recorder.finished.is_set()
    assert plugin.teach.recorder.record["state"] == "incomplete"
    assert robot.mode == 5 and plugin.control.ownership.owner is None


def test_short_resume_uses_original_queue_id_when_mode7_is_not_observed(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [0, 1, 2, 3, 4, 5])
    play = dispatch(plugin, "play", trajectory_id=record["trajectory_id"], speed_multiplier=.25)
    wait_until(lambda: robot.motion is not None)
    terminal(events, dispatch(plugin, "pause"))
    terminal(events, play, status="cancelled")
    with robot.lock:
        robot.motion["duration"] = .001
    count = len(robot.commands)
    terminal(events, dispatch(plugin, "resume"))
    assert robot.commands[count:] == ["Continue()"]


def test_stop_during_preparation_never_submits_late_startpath(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store)
    original = robot.handle
    setting_seen = threading.Event()

    def delayed_setting(command):
        reply = original(command)
        if command.startswith("EnableSafeSkin"):
            with robot.lock:
                robot.setting_until = time.monotonic() + .5
            setting_seen.set()
        return reply

    robot.handle = delayed_setting
    play = dispatch(plugin, "play", trajectory_id=record["trajectory_id"])
    assert setting_seen.wait(2)
    terminal(events, dispatch(plugin, "stop_motion"))
    terminal(events, play, status="cancelled")
    assert not any(command.startswith("StartPath") for command in robot.commands)
    assert robot.commands.count("Stop()") == 1


def test_stop_with_missing_feedback_is_not_reported_complete_or_replayed(rig):
    plugin, robot, store, events = rig
    plugin.teach.timeout = .2
    robot.feedback_enabled = False
    wait_until(lambda: plugin.client.snapshot()["stale"])
    result = terminal(events, dispatch(plugin, "stop_motion"), status="error")
    assert "confirm" in result["reason"]
    assert robot.commands == ["Stop()"]
    assert plugin.control.ownership.owner == "teach"
    robot.feedback_enabled = True
    wait_until(lambda: plugin.client.snapshot()["connected"])
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.control.ownership.owner is None


def test_joint_move_holds_the_same_arm_and_teach_stop_reaches_it(rig):
    plugin, robot, store, events = rig
    moved = plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 10, "confirm_move": True})
    wait_until(lambda: robot.motion is not None)
    with pytest.raises(RuntimeError, match="occupied"):
        dispatch(plugin, "start_record", name="must not interrupt joint_move")
    stopped = dispatch(plugin, "stop_motion")
    terminal(events, stopped)
    terminal(events, moved, status="cancelled")
    assert "StartDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None


def test_production_mount_guard_rejects_a_local_folder_before_any_dashboard_command(rig, monkeypatch):
    plugin, robot, store, events = rig
    record, _ = recording(store)
    store.require_network_mount = True
    monkeypatch.setattr("dobot.cra.native_mounts._mountinfo_text",
                        lambda: "1 0 0:1 / / rw - ext4 /dev/root rw\n")
    result = terminal(events, dispatch(plugin, "play", trajectory_id=record["trajectory_id"]), status="error")
    assert "CIFS/NFS" in result["reason"]
    assert not robot.commands
    assert not list(store.controller_root.iterdir())
