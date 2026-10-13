"""Paired control tests use a simulated SDK; they never open SocketCAN."""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest

from agilex.piper.device import build_plugins, PiperPlugin
from agilex.piper.dual import DualPiperPlugin
from agilex.piper.motion.kinematics import Calibration, Chain, MODEL_SHA256, RelativeMapping, PiperIK, transform, pose_values, rotation_error
from agilex.piper.paired_motion import numerical_worker
from agilex.piper.trajectory import Recorder, read_recording
from common.vendor_runtime import DriverBundle
from test_piper_ten_cards import Driver
from test_teleop_contract import input_frame


def calibration_document():
    identity = {"position": [0., 0., 0.], "orientation": [0., 0., 0., 1.]}
    return {"schema": "motus.piper.calibration/1", "calibration_id": "SIMULATED-TEST-ONLY",
            "hardware_verified": True, "model_sha256": MODEL_SHA256,
            "arms": {side: {"can_interface": "can0" if side == "left" else "can1",
                            "flange_to_tcp": copy.deepcopy(identity), "controller_to_tcp": copy.deepcopy(identity),
                            "tracking_to_base": copy.deepcopy(identity), "base_to_world": copy.deepcopy(identity),
                            "workspace_m": [[-1, -1, -1], [1, 1, 1]]} for side in ("left", "right")}}


class FastWorker:
    """A deterministic target generator for lifecycle tests, not an IK substitute claim."""
    def __init__(self, path):
        self.profile_sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.closed = False

    def self_test(self, q):
        pass

    def call(self, op, payload, **kwargs):
        q = list(payload["q"])
        q[0] += math.radians(.5)
        q[6] += math.radians(.5)
        return {"q": q}

    def interrupt(self):
        self.closed = True

    close = interrupt


class FakeBus:
    def __init__(self, ros2, motion):
        self.running = True

    def status(self):
        return {"running": self.running}

    def close(self):
        self.running = False


@pytest.fixture
def pair(tmp_path, monkeypatch):
    monkeypatch.setenv("PIPER_MOTION_ENABLED", "0")
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(calibration_document()))
    plugin = DualPiperPlugin({"data_dir": str(tmp_path / "data"), "async_actions": False,
                              "arms": {"left": {"can_interface": "can0"}, "right": {"can_interface": "can1"}},
                              "calibration_path": str(path)}, ros2=object(), driver_factory=Driver,
                             bus_factory=FakeBus, worker_factory=FastWorker, clock_id="boot-test")
    plugin.start()
    yield plugin
    plugin.stop()


def ready(pair):
    for arm in pair.arms.values():
        sdk = arm.connected().arm
        sdk.gripper_enabled = True
        arm.motion_enabled = arm.manual_motion_permission = True


def call(pair, card, action, **args):
    return DriverBundle([pair]).dispatch("piper_" + card, {"action": action, **args})


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), "condition timed out"


def frame(sequence, held=False):
    value = input_frame()
    value["received_monotonic_ns"] = time.monotonic_ns()
    value["source_monotonic_ns"] = value["received_monotonic_ns"]
    value["sequence"] = sequence
    value["left"]["grip"] = value["right"]["grip"] = 1. if held else 0.
    return value


def begin(pair):
    ready(pair)
    preview = call(pair, "teleop", "begin", execute=False)
    result = call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"],
                  workspace_clear=True, take_control=True)
    wait_for(lambda: pair.motion.reason == "waiting_for_input")
    assert pair.motion.receive(frame(1))
    wait_for(lambda: pair.motion.reason == "mapped_waiting_for_both_grips")
    return result


def feed(pair, duration=.3, held=True, first=2):
    end = time.monotonic() + duration
    sequence = first
    while time.monotonic() < end:
        pair.motion.receive(frame(sequence, held))
        sequence += 1
        time.sleep(.02)
    return sequence


def test_single_arm_factory_preserves_deployed_control_card(tmp_path):
    plugins = build_plugins({"data_dir": str(tmp_path)}, "piper", None)
    assert len(plugins) == 3 and isinstance(plugins[0], PiperPlugin)
    assert {t["name"] for t in plugins[0].get_tools()} == {"arm_control", "piper_photo"}


@pytest.mark.parametrize("arms", [{"left": {"can_interface": "can0"}},
    {"left": None, "right": {"can_interface": "can1"}},
    {"left": {"can_interface": "can0", "can_transport": "gs_usb"}, "right": {"can_interface": "can1"}},
    {"left": {"can_interface": "can0"}, "right": {"can_interface": "can0"}},
    {"left": {"can_interface": "../can0"}, "right": {"can_interface": "can1"}}])
def test_bad_pair_binding_rejected_before_connection(arms):
    with patch.object(Driver, "connect") as connect:
        with pytest.raises(ValueError):
            DualPiperPlugin({"arms": arms}, driver_factory=Driver)
        connect.assert_not_called()


def test_passive_start_and_explicit_side_and_no_double_sdk(pair):
    assert all(a.driver is None for a in pair.arms.values())
    assert len(pair.get_tools()) == 12
    call(pair, "teleop", "start", input_topic="/teleop/command")
    assert all(a.driver is None for a in pair.arms.values())
    with pytest.raises(ValueError, match="select_left"):
        call(pair, "status", "read")
    states = call(pair, "status", "read", arm_side="all")
    assert set(states["arms"]) == {"left", "right"}
    left = pair.arms["left"].driver
    call(pair, "status", "read", arm_side="left")
    assert pair.arms["left"].driver is left
    assert left is not pair.arms["right"].driver
    assert not any(a.driver.arm.calls for a in pair.arms.values())


def test_side_selection_cannot_change_other_arm(pair):
    ready(pair)
    call(pair, "permission", "block", arm_side="left")
    assert not pair.arms["left"].motion_enabled
    assert pair.arms["right"].motion_enabled
    with pytest.raises(ValueError, match="one_arm"):
        call(pair, "enable", "enable", arm_side="all")


@pytest.mark.parametrize("key,value", [("hardware_verified", False), ("model_sha256", "bad")])
def test_calibration_gate_keeps_can_silent(pair, key, value):
    ready(pair)
    path = Path(pair.motion.calibration_path)
    document = json.loads(path.read_text())
    document[key] = value
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError):
        call(pair, "teleop", "begin")
    assert not any(a.driver.arm.calls for a in pair.arms.values())


def test_permission_and_gripper_are_required_before_preview(pair):
    with pytest.raises(RuntimeError, match="许可关闭"):
        call(pair, "teleop", "begin")
    ready(pair)
    pair.arms["right"].driver.arm.gripper_enabled = False
    with pytest.raises(RuntimeError, match="gripper_not_ready"):
        call(pair, "teleop", "begin")
    assert not any(a.driver.arm.calls for a in pair.arms.values())


def test_preview_is_one_use_and_gates_release_on_failed_reservation(pair):
    ready(pair)
    preview = call(pair, "teleop", "begin")
    right = pair.arms["right"]
    right.gate.acquire()
    try:
        with pytest.raises(RuntimeError):
            call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
        assert not pair.arms["left"].gate.locked()
    finally:
        right.gate.release()
    call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    pair.motion.request_stop()
    wait_for(lambda: not pair.motion.thread.is_alive())
    with pytest.raises(RuntimeError, match="预览"):
        call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)


def test_manual_preview_invalidates_pair_preview(pair):
    ready(pair)
    preview = call(pair, "teleop", "begin")
    call(pair, "joint", "move", arm_side="left", joint=1, target_deg=1.)
    with pytest.raises(RuntimeError, match="预览"):
        call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    assert not any(a.gate.locked() for a in pair.arms.values())


def test_loose_mapping_double_grip_rate_limit_hold_and_stop(pair):
    begin(pair)
    assert not any(a.driver.arm.calls for a in pair.arms.values())
    for arm in pair.arms.values():
        with pytest.raises(RuntimeError):
            with arm.operation("piper_joint", "move"):
                pass
    sequence = feed(pair)
    assert pair.motion.output
    for arm in pair.arms.values():
        targets = [c[1] for c in arm.driver.arm.calls if c[0] == "joints"]
        assert len(targets) >= 2
        assert all(abs(b[0]-a[0]) <= 51 for a, b in zip(targets, targets[1:]))
        assert not any(c[0] in {"enable", "disable", "teach"} for c in arm.driver.arm.calls)
    sequence = feed(pair, .15, held=False, first=sequence)
    held_q = list(pair.motion.last_q)
    feed(pair, .15, held=False, first=sequence)
    assert pair.motion.last_q == held_q
    result = call(pair, "stop", "halt", arm_side="right", target="gripper")
    assert result["paired_motion"]["operation_stopping"]
    wait_for(lambda: not pair.motion.thread.is_alive())
    assert all(not a.gate.locked() for a in pair.arms.values())
    assert all(a.driver.arm.gripper_enabled and all(a.driver.arm.enabled) for a in pair.arms.values())
    assert set(pair.motion.holds) == {"left", "right"}


@pytest.mark.parametrize("mutation", ["connection", "tracking", "stale", "permission", "feedback"])
def test_fault_on_one_side_stops_pair_and_never_auto_resumes(pair, mutation):
    begin(pair)
    sequence = feed(pair, .15)
    if mutation == "connection":
        value = frame(sequence, True); value["connection_epoch"] += 1
        assert not pair.motion.receive(value)
    elif mutation == "tracking":
        value = frame(sequence, True); value["right"].update(tracked=False, position=None, orientation_xyzw=None)
        assert not pair.motion.receive(value)
    elif mutation == "permission":
        call(pair, "permission", "block", arm_side="left")
    elif mutation == "feedback":
        pair.arms["right"].driver.arm.frozen = True
    wait_for(lambda: not pair.motion.thread.is_alive())
    assert pair.motion.mode == "idle"
    assert not pair.motion.receive(frame(sequence+1, True))
    assert all(not a.gate.locked() for a in pair.arms.values())
    if mutation == "feedback":
        assert not pair.motion.holds["right"]["arm_hold_sent"]


def test_source_sequence_and_clock_validation(pair):
    begin(pair)
    assert not pair.motion.receive(frame(1))
    value = frame(2); value["clock_id"] = "another-host"
    assert not pair.motion.receive(value)
    value = frame(3); value["received_monotonic_ns"] -= 1_000_000_000
    assert not pair.motion.receive(value)
    assert pair.motion.sequence == 1


def test_second_can_send_failure_stops_and_reports_both_holds(pair):
    begin(pair)
    right = pair.arms["right"].driver
    with patch.object(right, "stream_target", side_effect=OSError("CAN unavailable")):
        pair.motion.receive(frame(2, True))
        wait_for(lambda: not pair.motion.thread.is_alive())
    assert pair.motion.last_action["status"] == "error"
    assert "CAN unavailable" in pair.motion.reason
    assert set(pair.motion.holds) == {"left", "right"}


def test_record_teleop_then_replay_and_refuse_wrong_start(pair):
    begin(pair)
    call(pair, "motion", "record_start")
    feed(pair, .4)
    result = call(pair, "motion", "record_stop")
    assert result["complete"], result
    filename = result["filename"]
    with pytest.raises(RuntimeError, match="busy"):
        call(pair, "motion", "play", filename=filename)
    pair.motion.request_stop()
    wait_for(lambda: not pair.motion.thread.is_alive())
    inspected = call(pair, "motion", "inspect", filename=filename)
    assert inspected["frames"] >= 3 and not inspected["hardware_checked"]
    with pytest.raises(ValueError, match="start_mismatch"):
        call(pair, "motion", "play", filename=filename)
    recording = read_recording(pair.motion.directory, filename, pair.motion.binding)
    # Move only the simulated feedback to the recorded start; no hardware command.
    first = recording["frames"][0]
    for i, arm in enumerate(pair.arms.values()):
        arm.driver.arm.raw = tuple(round(math.degrees(v)*1000) for v in first["target_q"][i*6:(i+1)*6])
        arm.driver.arm.width = first["target_width_mm"][i]
    preview = call(pair, "motion", "play", filename=filename)
    with patch.object(pair.arms["left"], "acp_complete") as complete:
        call(pair, "motion", "play", filename=filename, execute=True, preview_id=preview["preview_id"],
             workspace_clear=True, take_control=True)
        wait_for(lambda: not pair.motion.thread.is_alive())
        assert pair.motion.last_action["status"] == "completed", pair.motion.last_action
        assert complete.call_args.args[1] == "completed"


def test_real_numpy_fk_jacobian_and_ik_round_trip(pair):
    solver = PiperIK(pair.motion.calibration_path)
    seed = np.radians([0., 45., -90., 0., 30., 0.])
    goal = seed + np.radians([2., -1., 1., .5, .5, -.5])
    chain = solver.chain
    tcp = np.eye(4)
    pose, jac = chain.fk(seed, tcp, jacobian=True)
    for i in range(6):
        shifted = seed.copy(); shifted[i] += 1e-6
        other = chain.fk(shifted, tcp)
        numeric = np.r_[(other[:3, 3]-pose[:3, 3])/1e-6, rotation_error(other[:3, :3], pose[:3, :3])/1e-6]
        assert np.max(abs(numeric-jac[:, i])) < 1e-5
    target = chain.fk(goal, tcp)
    solved = solver.solve([target, target], list(seed)*2, None, time.monotonic()+2)
    for reached in solver.palms(solved):
        assert np.linalg.norm(reached[:3, 3]-target[:3, 3]) <= .002
        assert np.linalg.norm(rotation_error(reached[:3, :3], target[:3, :3])) <= .02


def test_real_isolated_worker_load_solve_and_close(pair):
    q = np.radians([0., 45., -90., 0., 30., 0.]).tolist()*2
    solver = PiperIK(pair.motion.calibration_path)
    values = sum([pose_values(t) for t in solver.palms(q)], [])
    worker = numerical_worker(pair.motion.calibration_path)
    try:
        assert worker.self_test(q)
        reply = worker.call("solve", {"values": values, "q": q, "commanded": q, "measured_ns": time.monotonic_ns()},
                            identity={"boot_id": "test", "session_id": "test", "mapping_epoch": 0, "generation": 1},
                            deadline=time.monotonic()+2)
        assert np.max(abs(np.asarray(reply["q"])-q)) < 1e-10
        process = worker._process
    finally:
        worker.close()
    assert process.poll() is not None


def test_mapping_keeps_initial_tcp_and_gripper_and_applies_relative_delta(pair):
    calibration = Calibration(pair.motion.calibration_path)
    q = np.radians([0., 45., -90., 0., 30., 0.]).tolist()*2
    initial = frame(1)
    mapping = RelativeMapping(calibration, q, initial, {"left": 20., "right": 30.})
    values, widths = mapping.targets(initial, {"left": (0, 70), "right": (0, 70)})
    expected = sum([pose_values(Chain().fk(q[i*6:(i+1)*6], np.eye(4))) for i in range(2)], [])
    assert np.allclose(values, expected)
    assert widths == {"left": 20., "right": 30.}
    moved = copy.deepcopy(initial)
    moved["left"]["position"] = [0.01, 0., 1.]
    moved["left"]["trigger"] = .1
    values2, widths2 = mapping.targets(moved, {"left": (0, 70), "right": (0, 70)})
    assert abs(values2[0]-values[0]-.01) < 1e-9
    assert widths2 == {"left": 13., "right": 30.}


def test_stop_during_worker_start_sends_no_commands_and_releases_gates(pair):
    entered, release = threading.Event(), threading.Event()
    def factory(path):
        entered.set()
        assert release.wait(2)
        return FastWorker(path)
    pair.motion.worker_factory = factory
    ready(pair)
    preview = call(pair, "teleop", "begin")
    call(pair, "teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    assert entered.wait(1)
    try:
        assert call(pair, "teleop", "stop")["operation_stopping"]
    finally:
        release.set()
    wait_for(lambda: not pair.motion.thread.is_alive())
    assert not any(a.driver.arm.calls for a in pair.arms.values())
    assert all(not a.gate.locked() for a in pair.arms.values())


def test_late_solution_cannot_move_after_stop(pair):
    begin(pair)
    entered, release = threading.Event(), threading.Event()
    worker = pair.motion.worker
    original = worker.call
    def slow(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(*args, **kwargs)
    with patch.object(worker, "call", side_effect=slow):
        pair.motion.receive(frame(2, True))
        assert entered.wait(1)
        pair.motion.request_stop()
        release.set()
        wait_for(lambda: not pair.motion.thread.is_alive())
    assert not any(a.driver.arm.calls for a in pair.arms.values())


def test_stop_between_left_and_right_writes_prevents_right_target(pair):
    begin(pair)
    left, right = [a.driver for a in pair.arms.values()]
    original = left.stream_target
    def left_and_cancel(*args):
        result = original(*args)
        pair.motion.request_stop()
        return result
    with patch.object(left, "stream_target", side_effect=left_and_cancel), patch.object(right, "stream_target", wraps=right.stream_target) as right_send:
        pair.motion.receive(frame(2, True))
        wait_for(lambda: not pair.motion.thread.is_alive())
        right_send.assert_not_called()
    assert set(pair.motion.holds) == {"left", "right"}


def test_disk_failure_stops_recording_but_keeps_operator_control(pair):
    begin(pair)
    call(pair, "motion", "record_start")
    pair.motion.recorder.fail("disk full")
    feed(pair, .2)
    assert pair.motion.output and pair.motion.mode == "teleop"
    result = call(pair, "motion", "record_stop")
    assert not result["complete"] and result["error"] == "disk full"


def test_status_reports_other_arm_when_one_feedback_fails(pair):
    ready(pair)
    with patch.object(pair.arms["right"], "status", side_effect=RuntimeError("lost CAN")):
        state = call(pair, "status", "read", arm_side="all")
    assert state["arms"]["left"]["feedback_fresh"]
    assert state["arms"]["right"]["error"] == "lost CAN"


def test_ik_timeout_and_unreachable_target_are_rejected(pair):
    solver = PiperIK(pair.motion.calibration_path)
    q = np.radians([0., 45., -90., 0., 30., 0.]).tolist()*2
    targets = solver.palms(q)
    targets[0][:3, 3] = [.9, .9, .9]
    with pytest.raises(ValueError, match="expired"):
        solver.solve(targets, q, q, time.monotonic()-1)
    with pytest.raises(ValueError, match="no_solution"):
        solver.solve(targets, q, q, time.monotonic()+2)


def test_monitor_card_stop_does_not_cancel_paired_motion(pair):
    begin(pair)
    call(pair, "status", "stop")
    call(pair, "photo", "stop")
    assert not pair.motion.cancel.is_set()
    assert call(pair, "motion", "info")["topic_in"] == []
    assert call(pair, "teleop", "info")["topic_in"][0]["topic"] == "/teleop/command"


def test_bus_restart_waits_for_previous_bus_close(pair):
    call(pair, "teleop", "start")
    old_bus = pair.bus
    closing, release = threading.Event(), threading.Event()
    def slow_close():
        closing.set()
        assert release.wait(2)
        old_bus.running = False
    with patch.object(old_bus, "close", side_effect=slow_close):
        stop = threading.Thread(target=lambda: call(pair, "teleop", "stop"))
        stop.start()
        assert closing.wait(1)
        start = threading.Thread(target=lambda: call(pair, "teleop", "start"))
        start.start()
        try:
            start.join(.05)
            assert start.is_alive()
        finally:
            release.set()
            stop.join(2)
            start.join(2)
    assert not stop.is_alive() and not start.is_alive()
    assert pair.bus is not old_bus and pair.bus.running
