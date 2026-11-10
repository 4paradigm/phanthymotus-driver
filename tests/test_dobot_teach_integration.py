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
    robot = RobotSimulator()
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
    assert result["state"] == "recorded" and "native_state" not in result
    assert result["point_count"] >= 3
    return identifier


def servo_targets(commands):
    """ServoJ joint targets in send order."""
    return [[float(v) for v in c[len("ServoJ("):-1].split(",")[:6]] for c in commands if c.startswith("ServoJ(")]


def nothing_after_stop(commands):
    stop = commands.index("Stop()")
    return not any(c.startswith(("ServoJ", "MovJ", "Continue")) for c in commands[stop + 1:])


def start_play(plugin, robot, angles=(0, 1, 2, 3, 4, 5, 6, 7), **args):
    record, _ = recording(plugin.teach.store, list(angles))
    played = dispatch(plugin, "play", trajectory_id=record["trajectory_id"], speed_multiplier=.25, **args)
    wait_until(lambda: len(servo_targets(robot.commands)) >= 3)
    return record["trajectory_id"], played


def test_record_go_to_start_servo_play_and_completion_over_real_local_tcp(rig):
    plugin, robot, store, events = rig
    identifier = perform_recording(rig)
    commands = list(robot.commands)
    assert commands[0] == "ClearError()" and "StartDrag()" not in commands
    # stop_record during a button drag never sends StopDrag; the operator exits.
    assert "StopDrag()" not in commands
    count = len(robot.commands)
    blocked = terminal(events, dispatch(plugin, "play", trajectory_id=identifier), status="error")
    assert "go_to_start" in blocked["reason"] and len(robot.commands) == count
    terminal(events, dispatch(plugin, "go_to_start", trajectory_id=identifier))
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "play", trajectory_id=identifier), timeout=5)
    assert result["trajectory_id"] == identifier and result["reason"] == "target_reached"
    played = robot.commands[count:]
    assert not any(c.startswith(("StartPath", "GetStartPose", "MovJ")) for c in played)
    points = [p["joints_deg"] for p in store.samples(identifier)["points"]]
    targets = servo_targets(played)
    # 50 Hz targets along the 50 ms recording at 1x: about 2.5 targets per sample.
    assert len(targets) >= 2 * (len(points) - 1)
    assert all("t=0.020" in c and "aheadtime=50.0" in c and "gain=500.0" in c
               for c in played if c.startswith("ServoJ"))
    assert targets[-1] == pytest.approx(points[-1], abs=1e-5)
    assert all(b[0] >= a[0] - 1e-6 for a, b in zip(targets, targets[1:]))
    assert robot.mode == 5 and plugin.control.ownership.owner is None


@pytest.mark.parametrize("card", ["teach", "joint_move"])
def test_pause_decelerates_along_the_path_and_resume_streams_to_the_end(rig, card):
    plugin, robot, store, events = rig
    identifier, played = start_play(plugin, robot, detect_collision=True)
    paused = terminal(events, plugin.dispatch("pause", {"_tool_name": card, "confirm_move": True}), timeout=5)
    stream = terminal(events, played, status="cancelled")
    assert stream["reason"] == "paused" and 0 < stream["position_s"] < .35
    assert paused["reason"] == "paused" and paused["trajectory_id"] == identifier
    assert "Pause()" not in robot.commands and robot.mode == 5
    assert plugin.control.ownership.owner == "teach"
    targets = servo_targets(robot.commands)
    steps = [b[0] - a[0] for a, b in zip(targets, targets[1:])]
    # The 0.5 s ramp: the per-target step shrinks to zero rather than stopping abruptly.
    assert steps[-1] < max(steps) / 4
    with pytest.raises(RuntimeError, match="resume or stop"):
        dispatch(plugin, "play", trajectory_id=identifier)
    count = len(robot.commands)
    result = terminal(events, plugin.dispatch("resume", {"_tool_name": card, "confirm_move": True}), timeout=5)
    resumed = robot.commands[count:]
    assert resumed and all(c.startswith("ServoJ") for c in resumed)
    assert robot.skin == 1 and robot.skin_setting == [1, 1, 1, 1]
    assert servo_targets(resumed)[-1][0] == pytest.approx(7)
    assert result["trajectory_id"] == identifier and plugin.control.ownership.owner is None
    with pytest.raises(RuntimeError, match="no teaching motion"):
        dispatch(plugin, "resume")


def test_two_cards_cannot_start_competing_motion_and_stop_routes_to_owner(rig):
    plugin, robot, store, events = rig
    _, played = start_play(plugin, robot)
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 10, "confirm_move": True})
    assert not any(c.startswith("MovJ") for c in robot.commands)
    stopped = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
    terminal(events, stopped)
    terminal(events, played, status="cancelled")
    assert robot.commands.count("Stop()") == 1 and nothing_after_stop(robot.commands)
    assert robot.mode == 5 and plugin.control.ownership.owner is None
    with pytest.raises(RuntimeError, match="no teaching motion"):
        dispatch(plugin, "resume")


def test_stop_during_pause_ramp_cancels_both_and_sends_no_late_target(rig):
    plugin, robot, store, events = rig
    _, played = start_play(plugin, robot)
    pause = dispatch(plugin, "pause")
    stopped = dispatch(plugin, "stop_motion")
    terminal(events, stopped)
    terminal(events, played, status="cancelled")
    terminal(events, pause, status="cancelled")
    assert robot.commands.count("Stop()") == 1 and nothing_after_stop(robot.commands)
    assert plugin.teach._resume_context is None and plugin.control.ownership.owner is None


def test_resume_after_the_arm_moved_is_rejected_without_commands(rig):
    plugin, robot, store, events = rig
    _, played = start_play(plugin, robot)
    terminal(events, dispatch(plugin, "pause"), timeout=5)
    terminal(events, played, status="cancelled")
    with robot.lock:
        robot.joints[0] += 2
    time.sleep(.05)
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "resume"), status="error")
    assert "moved away" in result["reason"] and len(robot.commands) == count
    assert plugin.control.ownership.owner == "teach" and plugin.teach._resume_context is not None
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.control.ownership.owner is None


def test_arm_not_following_the_stream_stops_once(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [0, 1, 2, 3, 4, 5, 6, 7])
    robot.servo_gap = 5
    result = terminal(events, dispatch(plugin, "play", trajectory_id=record["trajectory_id"]), status="error")
    assert "streamed path" in result["reason"]
    assert robot.commands.count("Stop()") == 1 and nothing_after_stop(robot.commands)
    assert plugin.control.ownership.owner is None


def test_driver_stop_during_stream_stops_the_arm(rig):
    plugin, robot, store, events = rig
    _, played = start_play(plugin, robot)
    plugin.dispatch("stop", {"_tool_name": "teach"})
    terminal(events, played, status="cancelled")
    assert robot.commands.count("Stop()") == 1 and nothing_after_stop(robot.commands)
    assert plugin.control.ownership.owner is None


def test_go_to_start_pause_and_resume_use_the_controller_queue(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    original = robot.handle

    def slow_movj(command):
        reply = original(command)
        if command.startswith("MovJ"):
            with robot.lock:
                robot.motion["duration"] = 1
        return reply

    robot.handle = slow_movj
    moving = dispatch(plugin, "go_to_start", trajectory_id=record["trajectory_id"])
    wait_until(lambda: robot.motion is not None)
    terminal(events, dispatch(plugin, "pause"))
    terminal(events, moving, status="cancelled")
    assert robot.commands[-1] == "Pause()" and robot.mode == 10
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "resume"), timeout=5)
    assert robot.commands[count:] == ["Continue()"]
    assert result["trajectory_id"] == record["trajectory_id"] and robot.joints[0] == pytest.approx(20)
    assert plugin.control.ownership.owner is None


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
    assert result["duration_s"] >= .5
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
    _, play = start_play(plugin, robot)
    robot.collision = True
    result = terminal(events, play, status="error")
    assert "collision" in result["reason"]
    assert robot.commands.count("Stop()") == 1 and nothing_after_stop(robot.commands)
    assert robot.commands.count("Continue()") == 0


def test_resume_rejected_by_protection_does_not_clear_or_lose_paused_context(rig):
    plugin, robot, store, events = rig
    _, play = start_play(plugin, robot)
    terminal(events, dispatch(plugin, "pause"), timeout=5)
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


def test_stop_motion_during_recorder_handoff_stops_the_installed_recorder(rig, monkeypatch):
    from dobot.cra import teaching

    plugin, robot, store, events = rig
    starting, release_start = threading.Event(), threading.Event()
    original_start = teaching.TrajectoryRecorder.start

    def held_start(recorder):
        starting.set()
        assert release_start.wait(2)
        return original_start(recorder)

    monkeypatch.setattr(teaching.TrajectoryRecorder, "start", held_start)
    starting_action = dispatch(plugin, "start_record", name="handoff race")
    assert starting.wait(2)
    stopped = []
    stop_thread = threading.Thread(target=lambda: stopped.append(dispatch(plugin, "stop_motion")))
    stop_thread.start()
    time.sleep(.02)
    assert stop_thread.is_alive()
    release_start.set()
    stop_thread.join(2)
    assert not stop_thread.is_alive() and stopped
    terminal(events, starting_action)
    terminal(events, stopped[0])
    assert plugin.teach.recorder is not None and plugin.teach.recorder.finished.is_set()
    assert plugin.control.ownership.owner is None


def test_stop_during_preparation_never_submits_late_servoj(rig):
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
    assert not any(command.startswith("ServoJ") for command in robot.commands)
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
