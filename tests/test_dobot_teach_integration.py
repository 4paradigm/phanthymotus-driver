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
              "joint_move": {"enable_timeout_s": 1, "receive_timeout_s": .3, "motion_timeout_s": 2}}

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


def perform_recording(rig):
    plugin, robot, store, events = rig
    event = terminal(events, dispatch(plugin, "start_record", name="本地端到端测试"))
    assert event["state"] == "recording"
    identifier = event["trajectory_id"]
    with robot.lock:
        robot.hand_velocity = 30
    time.sleep(.25)
    result = terminal(events, dispatch(plugin, "stop_record"))
    assert result["state"] == "recorded" and result["native_state"] == "exported"
    assert result["point_count"] >= 3
    return identifier


def test_record_go_to_start_native_play_and_completion_over_real_local_tcp(rig):
    plugin, robot, store, events = rig
    identifier = perform_recording(rig)
    commands = list(robot.commands)
    assert commands[:2] == ["ClearError()", "StartDrag()"]
    assert commands[-1] == "StopDrag()"
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


def test_failed_start_drag_does_not_claim_recording_started(rig):
    plugin, robot, store, events = rig
    robot.failure_command = "StartDrag"
    result = terminal(events, dispatch(plugin, "start_record", name="failed"), status="error")
    assert "StartDrag rejected" in result["reason"]
    assert store.list_records()[0]["state"] == "incomplete"
    assert plugin.control.ownership.owner is None


def test_feedback_loss_during_recording_saves_incomplete_and_exits_drag(rig):
    plugin, robot, store, events = rig
    terminal(events, dispatch(plugin, "start_record", name="drop"))
    time.sleep(.1)
    robot.feedback_enabled = False
    wait_until(lambda: "StopDrag()" in robot.commands)
    robot.feedback_enabled = True
    wait_until(lambda: plugin.teach.recorder.finished.is_set())
    assert plugin.teach.recorder.record["state"] == "incomplete"
    assert "feedback" in plugin.teach.recorder.record["reason"]
    assert robot.mode == 5


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
        time.sleep(.15)
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
        assert robot.commands.count("StartDrag()") == 1
        assert robot.commands.count("StopDrag()") == 1
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
    assert robot.commands == ["ClearError()", "EnableRobot()", "StartDrag()"]
    assert robot.skin == 1
    time.sleep(.1)
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
