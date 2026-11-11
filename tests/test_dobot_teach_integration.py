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


def hand_moves(robot, velocity=30.0):
    """The operator pushes joint 1 while the arm is in drag."""
    with robot.lock:
        robot._advance()
        robot.hand_velocity = velocity


def operator_exits_drag(robot):
    """The operator presses the drag button on the arm (the driver sent no StopDrag)."""
    with robot.lock:
        robot._advance()
        robot.mode, robot.hand_velocity, robot.running_packets = 5, 0.0, 1


def start_recording(plugin, events, name, timeout=3):
    return terminal(events, dispatch(plugin, "start_trajectory", name=name), timeout=timeout)


def recording_end_events(events):
    """Unsolicited completions sent when a recording ends without a stop action."""
    return [event for event in events if event[0].startswith("dobot_cra_teach_recording_")]


def recording_end(events, end_reason, status="completed"):
    wait_until(lambda: recording_end_events(events), 5)
    time.sleep(.1)
    (identifier, reported, result), = recording_end_events(events)
    # The platform keeps only the status of a success: the reason is also readable in the id.
    assert identifier.startswith(f"dobot_cra_teach_recording_{end_reason}_")
    assert reported == status and result["end_reason"] == end_reason
    return result


def perform_recording(rig):
    plugin, robot, store, events = rig
    robot.entry_packets = 3  # field CR3A reports the first drag packets as mode 0
    event = start_recording(plugin, events, "本地端到端测试")
    assert event["state"] == "recording" and event["drag"] == "entered" and event["name"] == "本地端到端测试"
    assert robot.mode == 6
    hand_moves(robot)
    time.sleep(.25)
    result = terminal(events, dispatch(plugin, "stop_trajectory"))
    assert result["state"] == "recorded" and result["end_reason"] == "stop_trajectory"
    assert result["point_count"] >= 3 and "recorded_at" in result and "trajectory_id" not in result
    # The stop action's own completion is the only notification: no second one for the end.
    assert recording_end_events(events) == []
    return result["name"]


def servo_targets(commands):
    """ServoJ joint targets in send order."""
    return [[float(v) for v in c[len("ServoJ("):-1].split(",")[:6]] for c in commands if c.startswith("ServoJ(")]


def nothing_after_stop(commands):
    stop = commands.index("Stop()")
    return not any(c.startswith(("ServoJ", "MovJ", "Continue")) for c in commands[stop + 1:])


def start_play(plugin, robot, angles=(0, 1, 2, 3, 4, 5, 6, 7), **args):
    record, _ = recording(plugin.teach.store, list(angles))
    played = dispatch(plugin, "play", name=record["name"], speed="0.25x", **args)
    wait_until(lambda: len(servo_targets(robot.commands)) >= 3)
    return record["name"], played


def test_record_with_drag_commands_then_play_goes_to_start_and_streams_over_real_local_tcp(rig):
    plugin, robot, store, events = rig
    name = perform_recording(rig)
    commands = list(robot.commands)
    assert commands[0] == "ClearError()"
    # The driver enters and leaves drag itself, once each, in that order.
    assert commands.count("StartDrag()") == 1 and commands.count("StopDrag()") == 1
    assert commands.index("StartDrag()") < commands.index("StopDrag()") and robot.mode == 5
    assert plugin.control.ownership.owner is None
    count = len(robot.commands)
    # play first reaches the start (the arm is at the end of the take) and verifies arrival.
    result = terminal(events, dispatch(plugin, "play", name=name), timeout=5)
    assert result["name"] == name and result["reason"] == "target_reached" and result["moved_to_start"] is True
    played = robot.commands[count:]
    first_servo = next(i for i, c in enumerate(played) if c.startswith("ServoJ"))
    assert sum(c.startswith("MovJ") for c in played) == 1
    assert next(i for i, c in enumerate(played) if c.startswith("MovJ")) < first_servo
    points = [p["joints_deg"] for p in store.samples(name)["points"]]
    targets = servo_targets(played)
    # The stream begins one 20 ms step after the verified start pose.
    assert targets[0][0] == pytest.approx(points[0][0], abs=1.5)
    # 50 Hz targets along the 50 ms recording at 1x: about 2.5 targets per sample.
    assert len(targets) >= 2 * (len(points) - 1)
    assert all("t=0.020" in c and "aheadtime=50.0" in c and "gain=500.0" in c
               for c in played if c.startswith("ServoJ"))
    assert targets[-1] == pytest.approx(points[-1], abs=1e-5)
    assert all(b[0] >= a[0] - 1e-6 for a, b in zip(targets, targets[1:]))
    assert robot.mode == 5 and plugin.control.ownership.owner is None
    # The safe skin is on by default for playback (recording turned it off).
    assert robot.skin == 1 and robot.skin_setting == [1, 1, 1, 1]
    # Already at the end, not the start: return_to_start brings the arm back after the stream.
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "play", name=name, return_to_start=True), timeout=8)
    played = robot.commands[count:]
    last_servo = max(i for i, c in enumerate(played) if c.startswith("ServoJ"))
    assert result["moved_to_start"] is True and result["returned_to_start"] is True
    assert sum(c.startswith("MovJ") for c in played) == 2
    assert max(i for i, c in enumerate(played) if c.startswith("MovJ")) > last_servo
    assert robot.joints[0] == pytest.approx(points[0][0]) and result["elapsed_s"] > 0
    # At the start, play skips the MovJ and stays at the end of the take.
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "play", name=name), timeout=5)
    assert result["moved_to_start"] is False and result["name"] == name
    assert "returned_to_start" not in result
    assert not any(c.startswith(("StartPath", "GetStartPose", "MovJ")) for c in robot.commands[count:])
    assert robot.mode == 5 and plugin.control.ownership.owner is None


@pytest.mark.parametrize("card", ["teach", "joint_move"])
def test_pause_decelerates_along_the_path_and_resume_streams_to_the_end(rig, card):
    plugin, robot, store, events = rig
    identifier, played = start_play(plugin, robot, detect_collision=True)
    paused = terminal(events, plugin.dispatch("pause", {"_tool_name": card, "confirm_move": True}), timeout=5)
    stream = terminal(events, played, status="cancelled")
    assert stream["reason"] == "paused" and 0 < stream["position_s"] < .35
    assert paused["reason"] == "paused" and paused["name"] == identifier
    assert "Pause()" not in robot.commands and robot.mode == 5
    assert plugin.control.ownership.owner == "teach"
    targets = servo_targets(robot.commands)
    steps = [b[0] - a[0] for a, b in zip(targets, targets[1:])]
    # The 0.5 s ramp: the per-target step shrinks to zero rather than stopping abruptly.
    assert steps[-1] < max(steps) / 4
    with pytest.raises(RuntimeError, match="resume or stop"):
        dispatch(plugin, "play", name=identifier)
    count = len(robot.commands)
    result = terminal(events, plugin.dispatch("resume", {"_tool_name": card, "confirm_move": True}), timeout=5)
    resumed = robot.commands[count:]
    assert resumed and all(c.startswith("ServoJ") for c in resumed)
    assert robot.skin == 1 and robot.skin_setting == [1, 1, 1, 1]
    assert servo_targets(resumed)[-1][0] == pytest.approx(7)
    assert result["name"] == identifier and plugin.control.ownership.owner is None
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
    result = terminal(events, dispatch(plugin, "play", name=record["name"]), status="error")
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


def slow_movj(robot):
    """MovJ takes 1 s so that a pause can land in the middle of it."""
    original = robot.handle

    def handle(command):
        reply = original(command)
        if command.startswith("MovJ"):
            with robot.lock:
                robot.motion["duration"] = 1
        return reply

    robot.handle = handle


def at_joint1(robot, plugin, angle):
    """Put the simulated arm at `angle` on joint 1 and wait until feedback shows it."""
    with robot.lock:
        robot.joints[0] = angle
    wait_until(lambda: plugin.client.snapshot()["data"]["joint_state"]["q_actual"][0] == pytest.approx(angle))


def test_return_to_start_pause_and_resume_use_the_controller_queue(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    at_joint1(robot, plugin, 20)
    slow_movj(robot)
    play = dispatch(plugin, "play", name=record["name"], speed="2x", return_to_start=True)
    wait_until(lambda: any(c.startswith("MovJ") for c in robot.commands), 5)
    terminal(events, dispatch(plugin, "pause"))
    terminal(events, play, status="cancelled")
    assert robot.commands[-1] == "Pause()" and robot.mode == 10
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "resume"), timeout=5)
    assert robot.commands[count:] == ["Continue()"]
    assert result["name"] == record["name"] and robot.joints[0] == pytest.approx(20)
    assert plugin.control.ownership.owner is None


def test_pause_at_start_holds_the_arm_until_resume_starts_the_stream(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    held = terminal(events, dispatch(plugin, "play", name=record["name"], pause_at_start=True),
                    status="cancelled", timeout=5)
    assert held["reason"] == "paused_at_start" and held["moved_to_start"] is True
    assert not any(c.startswith("ServoJ") for c in robot.commands)
    assert robot.joints[0] == pytest.approx(20) and robot.skin == 1
    assert plugin.control.ownership.owner == "teach" and plugin.teach._resume_context is not None
    with pytest.raises(RuntimeError, match="resume or stop"):
        dispatch(plugin, "play", name=record["name"])
    count = len(robot.commands)
    result = terminal(events, dispatch(plugin, "resume"), timeout=5)
    streamed = robot.commands[count:]
    assert streamed and all(c.startswith("ServoJ") for c in streamed)
    assert result["name"] == record["name"] and robot.joints[0] == pytest.approx(22)
    assert plugin.control.ownership.owner is None


def test_pause_at_start_without_moving_and_stop_motion_releases_the_hold(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    at_joint1(robot, plugin, 20)
    held = terminal(events, dispatch(plugin, "play", name=record["name"], pause_at_start=True),
                    status="cancelled", timeout=5)
    assert held["moved_to_start"] is False and not any(c.startswith("MovJ") for c in robot.commands)
    terminal(events, dispatch(plugin, "stop_motion"))
    assert plugin.teach._resume_context is None and plugin.control.ownership.owner is None
    assert not any(c.startswith("ServoJ") for c in robot.commands)


def test_play_rejects_unknown_tiers_and_non_boolean_options_before_any_command(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    for bad in ({"speed": 2}, {"speed": "3x"}, {"speed": "100%"}, {"approach_speed": "turbo"},
                {"pause_at_start": "yes"}, {"return_to_start": 1}, {"detect_collision": []}):
        with pytest.raises(ValueError):
            dispatch(plugin, "play", name=record["name"], **bad)
    assert robot.commands == [] and plugin.control.ownership.owner is None


def test_boolean_options_accept_true_false_text_and_null(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    held = terminal(events, dispatch(plugin, "play", name=record["name"], pause_at_start="true",
                                     return_to_start=None, detect_collision="false"),
                    status="cancelled", timeout=5)
    assert held["reason"] == "paused_at_start" and robot.skin == 0
    terminal(events, dispatch(plugin, "stop_motion"))


def test_play_reaches_and_verifies_the_start_before_streaming(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    result = terminal(events, dispatch(plugin, "play", name=record["name"], approach_speed="fast",
                                       speed="2x"), timeout=5)
    assert result["moved_to_start"] is True and result["name"] == record["name"]
    movj = next(i for i, c in enumerate(robot.commands) if c.startswith("MovJ"))
    first_servo = next(i for i, c in enumerate(robot.commands) if c.startswith("ServoJ"))
    assert movj < first_servo
    assert "joint={20" in robot.commands[movj] and "a=50,v=50" in robot.commands[movj]
    assert "EnableSafeSkin(1)" in robot.commands and "EnableSafeSkin(0)" not in robot.commands
    # The MovJ preparation already set the skin; it is not set again after arrival.
    assert robot.commands.count("EnableSafeSkin(1)") == 1
    assert sum(c.startswith("SetSafeSkin") for c in robot.commands) == 4
    # Streaming only began from the verified start: the first target is the recorded start.
    assert servo_targets(robot.commands)[0][0] == pytest.approx(20, abs=1.5)
    assert robot.joints[0] == pytest.approx(22) and robot.mode == 5
    assert plugin.control.ownership.owner is None


def test_play_can_turn_the_safe_skin_off(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    with robot.lock:
        robot.skin = 1
    terminal(events, dispatch(plugin, "play", name=record["name"], detect_collision=False), timeout=5)
    assert robot.skin == 0 and "EnableSafeSkin(1)" not in robot.commands


def test_play_cancelled_while_moving_to_start_never_streams(rig):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    slow_movj(robot)
    play = dispatch(plugin, "play", name=record["name"])
    wait_until(lambda: robot.motion is not None)
    terminal(events, dispatch(plugin, "stop_motion"))
    terminal(events, play, status="cancelled")
    assert not any(c.startswith("ServoJ") for c in robot.commands)
    assert robot.commands.count("Stop()") == 1 and plugin.control.ownership.owner is None


@pytest.mark.parametrize("card", ["teach", "joint_move"])
def test_pause_and_resume_while_play_is_moving_to_start_continue_into_the_stream(rig, card):
    plugin, robot, store, events = rig
    record, _ = recording(store, [20, 21, 22])
    slow_movj(robot)
    play = dispatch(plugin, "play", name=record["name"], speed="2x")
    wait_until(lambda: robot.motion is not None)
    terminal(events, plugin.dispatch("pause", {"_tool_name": card, "confirm_move": True}))
    terminal(events, play, status="cancelled")
    assert robot.commands[-1] == "Pause()" and robot.mode == 10
    assert not any(c.startswith("ServoJ") for c in robot.commands)
    assert plugin.control.ownership.owner == "teach"
    with pytest.raises(RuntimeError, match="resume or stop"):
        dispatch(plugin, "play", name=record["name"])
    count = len(robot.commands)
    result = terminal(events, plugin.dispatch("resume", {"_tool_name": card, "confirm_move": True}), timeout=6)
    resumed = robot.commands[count:]
    assert resumed[0] == "Continue()"
    # After the controller queue finishes at the start, the same resume continues with ServoJ.
    assert any(c.startswith("ServoJ") for c in resumed)
    assert resumed.index("Continue()") < next(i for i, c in enumerate(resumed) if c.startswith("ServoJ"))
    assert result["moved_to_start"] is True and result["name"] == record["name"]
    assert robot.joints[0] == pytest.approx(22) and plugin.control.ownership.owner is None


def test_recording_owns_arm_after_start_transition_completes(rig):
    plugin, robot, store, events = rig
    started = start_recording(plugin, events, "record")
    assert started["drag"] == "entered" and robot.mode == 6
    assert plugin.teach._active is None
    assert plugin.control.ownership.owner == "teach"
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 5, "confirm_move": True})
    with pytest.raises(RuntimeError, match="active arm operation"):
        plugin.control.reconfigure("127.0.0.2", 30004)
    stopped = plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True})
    result = terminal(events, stopped)
    assert result["state"] == "incomplete" and robot.mode == 5
    assert robot.commands.count("StopDrag()") == 1
    assert plugin.control.ownership.owner is None


def test_start_trajectory_with_an_existing_name_fails_before_any_command(rig):
    plugin, robot, store, events = rig
    recording(store, name="已有轨迹")
    for name in ("已有轨迹", "  已有轨迹  "):
        with pytest.raises(ValueError, match="already exists"):
            dispatch(plugin, "start_trajectory", name=name)
    assert robot.commands == [] and plugin.control.ownership.owner is None
    assert [item["name"] for item in store.list_trajectories()] == ["已有轨迹"]


def test_start_trajectory_while_the_arm_is_already_dragging_is_rejected_without_commands(rig):
    plugin, robot, store, events = rig
    robot.mode = 6
    wait_until(lambda: plugin.client.snapshot()["data"]["drag_status"])
    result = terminal(events, dispatch(plugin, "start_trajectory", name="already dragging"), status="error")
    assert "stationary" in result["reason"]
    assert robot.commands == []
    assert store.list_trajectories() == []  # nothing captured: the name is not left occupied
    assert plugin.control.ownership.owner is None


def move_j_rejected(plugin):
    with pytest.raises(RuntimeError, match="occupied"):
        plugin.dispatch("move_j", {"_tool_name": "joint_move", "joint1_deg": 5, "confirm_move": True})


def drag_for(plugin, robot, events, name, drag_s=.2):
    start_recording(plugin, events, name)
    hand_moves(robot)
    time.sleep(drag_s)


def test_operator_leaving_drag_stops_the_recording_and_marks_it_usable(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "button exit")
    operator_exits_drag(robot)
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "recorded" and record["end_reason"] == "drag_exited"
    assert "left drag" in record["note"] and "usable" in record["note"] and "reason" in record
    assert record["reason"] is None and "recorded_at" in record
    # The operator already left drag: the driver sends no StopDrag and releases the arm.
    assert robot.commands.count("StopDrag()") == 0 and robot.mode == 5
    assert plugin.control.ownership.owner is None
    # The operator's button press has no pending action: the agent is told by a separate completion.
    reported = recording_end(events, "drag_exited")
    assert reported["name"] == "button exit" and reported["state"] == "recorded" and "usable" in reported["note"]
    result = terminal(events, dispatch(plugin, "stop_trajectory"))
    assert result["state"] == "recorded" and result["end_reason"] == "drag_exited" and "note" in result
    assert len(recording_end_events(events)) == 1
    assert robot.commands.count("StopDrag()") == 0
    listed = plugin.dispatch("list_trajectories", {"_tool_name": "teach"})["trajectories"]
    assert listed[0]["name"] == "button exit" and listed[0]["end_reason"] == "drag_exited"
    # A take ended by the operator plays like any other.
    terminal(events, dispatch(plugin, "play", name="button exit"), timeout=5)
    assert plugin.control.ownership.owner is None


def test_stop_trajectory_waits_for_mode5_coasting_to_reach_rest(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "coasting")
    robot.joint_speed = [5, 0, 0, 0, 0, 0]
    stop = dispatch(plugin, "stop_trajectory")
    time.sleep(.3)
    assert not any(event[0] == stop["action_id"] for event in events)
    robot.joint_speed = None
    result = terminal(events, stop)
    assert result["state"] == "recorded" and robot.commands.count("StopDrag()") == 1
    assert plugin.control.ownership.owner is None


def test_unconfirmed_exit_is_not_success_and_keeps_the_arm_until_stop_motion(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "never settles")
    robot.joint_speed = [5, 0, 0, 0, 0, 0]
    result = terminal(events, dispatch(plugin, "stop_trajectory"), status="error", timeout=5)
    assert "could not confirm exit from drag" in result["reason"]
    record = store.load("never settles")
    assert record["state"] == "incomplete" and "could not confirm" in record["reason"]
    assert plugin.control.ownership.owner == "teach"
    move_j_rejected(plugin)
    robot.joint_speed = None
    stopped = terminal(events, plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True}))
    assert stopped["state"] == "incomplete"
    assert plugin.control.ownership.owner is None


def test_rejected_stopdrag_keeps_the_arm_and_is_retried_by_stop_motion(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "stopdrag rejected")
    robot.failure_command = "StopDrag"
    result = terminal(events, dispatch(plugin, "stop_trajectory"), status="error", timeout=5)
    assert "could not confirm exit from drag" in result["reason"] and "StopDrag" in result["reason"]
    assert store.load("stopdrag rejected")["state"] == "incomplete"
    assert robot.mode == 6 and plugin.control.ownership.owner == "teach"
    robot.failure_command = None
    terminal(events, plugin.dispatch("stop_motion", {"_tool_name": "joint_move", "confirm_move": True}))
    assert robot.mode == 5 and robot.commands.count("StopDrag()") == 2
    assert plugin.control.ownership.owner is None


@pytest.mark.parametrize("interrupt", ["stop_motion", "driver_stop"])
def test_interruption_during_exit_confirmation_is_never_saved_as_recorded(rig, interrupt):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "late interrupt")
    robot.joint_speed = [5, 0, 0, 0, 0, 0]  # keeps the exit confirmation pending
    stop = dispatch(plugin, "stop_trajectory")
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
    record = store.load("late interrupt")
    assert record["state"] == "incomplete" and record["end_reason"] == "interrupted"
    assert expected in record["reason"]
    assert plugin.control.ownership.owner is None


def test_no_stopdrag_without_fresh_feedback_and_it_is_sent_once_feedback_returns(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "lost confirmation", drag_s=.1)
    robot.feedback_enabled = False
    time.sleep(.6)  # feedback is stale: the recorder has ended and cannot see the drag state
    result = terminal(events, dispatch(plugin, "stop_trajectory"), status="error", timeout=6)
    assert "incomplete" in result["reason"]
    assert robot.commands.count("StopDrag()") == 0 and robot.mode == 6
    assert plugin.control.ownership.owner == "teach"
    robot.feedback_enabled = True
    plugin.teach.drag_exit_timeout = 4  # covers the 1 s feedback reconnect delay
    terminal(events, dispatch(plugin, "stop_motion"), timeout=8)
    assert robot.commands.count("StopDrag()") == 1 and robot.mode == 5
    assert plugin.control.ownership.owner is None


@pytest.mark.parametrize("mode", [7, 10])
def test_non_idle_mode_during_recording_ends_incomplete_without_recovery_commands(rig, mode):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "bad mode")
    count = len(robot.commands)
    with robot.lock:
        robot.mode = mode
        robot.setting_until = time.monotonic() + .3  # hold simulated mode 7 briefly
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "incomplete" and f"controller mode {mode}" in record["reason"]
    assert record["end_reason"] == "interrupted" and record["point_count"] >= 3
    failed = recording_end(events, "interrupted", status="error")
    assert f"controller mode {mode}" in failed["reason"] and failed["state"] == "incomplete"
    assert not any(c in robot.commands[count:] for c in ("ClearError()", "EnableRobot()", "Continue()", "StopDrag()"))
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


def test_recording_limit_sends_stopdrag_and_saves_as_recorded(rig):
    plugin, robot, store, events = rig
    plugin.teach.max_duration = .5
    start_recording(plugin, events, "limit")
    hand_moves(robot)
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 5)
    record = plugin.teach.recorder.record
    assert record["state"] == "recorded" and record["end_reason"] == "limit_reached"
    assert recording_end(events, "limit_reached")["point_count"] >= 3
    assert robot.commands.count("StopDrag()") == 1 and robot.mode == 5
    assert plugin.control.ownership.owner is None
    assert terminal(events, dispatch(plugin, "stop_trajectory"))["end_reason"] == "limit_reached"
    assert robot.commands.count("StopDrag()") == 1


def test_mode6_without_drag_status_is_reported_as_inconsistent_and_gets_no_stopdrag(rig):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "inconsistent")
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
    assert "StopDrag()" not in robot.commands and plugin.control.ownership.owner is None


def test_long_mode0_after_start_is_a_drag_not_an_error(rig):
    plugin, robot, store, events = rig
    robot.entry_packets = 40
    started = start_recording(plugin, events, "mode 0 accepted")
    time.sleep(.2)
    record = plugin.teach.recorder.record
    assert record["state"] == "recording" and not plugin.teach.recorder.finished.is_set()
    result = terminal(events, dispatch(plugin, "stop_trajectory"))
    assert result["state"] == "recorded" and result["point_count"] > 1
    assert started["state"] == "recording"


def test_arm_that_never_enters_drag_discards_the_take_and_frees_the_name(rig):
    plugin, robot, store, events = rig
    plugin.teach.drag_start_timeout = .3
    original = robot.handle

    def ignore_startdrag(command):
        reply = original(command)
        if command == "StartDrag()":
            with robot.lock:
                robot.mode = 5
        return reply

    robot.handle = ignore_startdrag
    result = terminal(events, dispatch(plugin, "start_trajectory", name="no drag"), status="error", timeout=5)
    assert "did not enter drag" in result["reason"]
    assert store.list_trajectories() == [] and not store.exists("no drag")
    assert robot.commands.count("StartDrag()") == 1 and "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None
    robot.handle = original
    assert start_recording(plugin, events, "no drag")["drag"] == "entered"  # the name is usable again
    terminal(events, dispatch(plugin, "stop_motion"))


def test_rejected_startdrag_discards_the_take_without_leave_drag_commands(rig):
    plugin, robot, store, events = rig
    robot.failure_command = "StartDrag"
    result = terminal(events, dispatch(plugin, "start_trajectory", name="rejected"), status="error")
    assert "StartDrag" in result["reason"]
    assert store.list_trajectories() == [] and "StopDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None


def test_persistence_failure_reports_error(rig, monkeypatch):
    plugin, robot, store, events = rig
    drag_for(plugin, robot, events, "disk full")

    def full(record):
        raise OSError("disk full")
    monkeypatch.setattr(store, "save", full)
    result = terminal(events, dispatch(plugin, "stop_trajectory"), status="error")
    assert "persistence failed" in result["reason"]
    assert plugin.teach.recorder.record["state"] == "incomplete"


def test_feedback_loss_during_recording_saves_incomplete_and_still_leaves_drag(rig):
    plugin, robot, store, events = rig
    plugin.teach.drag_exit_timeout = 4  # covers the 1 s feedback reconnect delay
    drag_for(plugin, robot, events, "drop", drag_s=.1)
    robot.feedback_enabled = False
    time.sleep(.4)  # the recorder ends on missing feedback and waits for fresh packets
    robot.feedback_enabled = True
    wait_until(lambda: plugin.teach.recorder.finished.is_set(), 8)
    assert plugin.teach.recorder.record["state"] == "incomplete"
    assert "feedback" in plugin.teach.recorder.record["reason"]
    assert robot.mode == 5 and robot.commands.count("StopDrag()") == 1
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
        terminal(events, call("start_trajectory", name="HTTP recording", confirm_move=True))
        hand_moves(robot)
        time.sleep(.2)
        result = terminal(events, call("stop_trajectory", confirm_move=True))
        assert result["state"] == "recorded" and result["name"] == "HTTP recording"
        listed = call("list_trajectories")
        assert listed["trajectories"][0]["name"] == "HTTP recording" and "recorded_at" in listed["trajectories"][0]
        assert "trajectory_id" not in listed["trajectories"][0]
        assert call("delete_trajectory", name="HTTP recording", confirm_move=True) == {
            "name": "HTTP recording", "state": "deleted"}
        assert call("list_trajectories") == {"trajectories": []}
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
        start = dispatch(plugin, "start_trajectory", name="real TLS callback")
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


def test_disabled_recording_prepares_enable_and_turns_the_safe_skin_off(rig):
    plugin, robot, store, events = rig
    with robot.lock:
        robot.mode, robot.enabled, robot.skin = 4, False, 1
    wait_until(lambda: plugin.client.snapshot()["data"]["robot_mode"] == 4)
    start_recording(plugin, events, "disabled")
    # Dragging would trigger the skin, so it is off before StartDrag.
    assert robot.commands == ["ClearError()", "EnableRobot()", "EnableSafeSkin(0)", "StartDrag()"]
    assert robot.skin == 0
    hand_moves(robot)
    time.sleep(.15)
    terminal(events, dispatch(plugin, "stop_trajectory"))


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
    result = terminal(events, dispatch(plugin, "start_trajectory", name="bad state"), status="error")
    assert "paused" in result["reason"]
    assert robot.commands == [] and store.list_trajectories() == []


def test_card_stop_cancels_recording_and_exits_drag(rig):
    plugin, robot, store, events = rig
    start_recording(plugin, events, "stop lifecycle")
    time.sleep(.1)
    result = plugin.dispatch("stop", {"_tool_name": "teach"})
    assert result["state"] == "idle"
    assert plugin.teach.recorder.finished.is_set()
    assert plugin.teach.recorder.record["state"] == "incomplete"
    assert robot.mode == 5 and robot.commands.count("StopDrag()") == 1
    assert plugin.control.ownership.owner is None


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
    starting_action = dispatch(plugin, "start_trajectory", name="handoff race")
    assert starting.wait(2)
    stopped = []
    stop_thread = threading.Thread(target=lambda: stopped.append(dispatch(plugin, "stop_motion")))
    stop_thread.start()
    time.sleep(.02)
    assert stop_thread.is_alive()
    release_start.set()
    stop_thread.join(2)
    assert not stop_thread.is_alive() and stopped
    # Whether the start finished before the stop arrived is a race; either way nothing is left behind.
    wait_until(lambda: any(event[0] == starting_action["action_id"] for event in events))
    assert next(e for e in events if e[0] == starting_action["action_id"])[1] in ("completed", "cancelled")
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
    play = dispatch(plugin, "play", name=record["name"])
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
        dispatch(plugin, "start_trajectory", name="must not interrupt joint_move")
    stopped = dispatch(plugin, "stop_motion")
    terminal(events, stopped)
    terminal(events, moved, status="cancelled")
    assert "StartDrag()" not in robot.commands
    assert plugin.control.ownership.owner is None
