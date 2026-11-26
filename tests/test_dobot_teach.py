"""Offline recording integrity, playback samples and card contracts."""

import copy
from datetime import datetime
import hashlib
import struct
import threading
import time

import pytest

from dobot.cra.device import DobotSensors
from dobot.cra.feedback import FeedbackClient, FeedbackSubscription
from dobot.cra import trajectory
from dobot.cra.teaching import ServoPlayback
from dobot.cra.trajectory import TrajectoryRecorder, TrajectoryStore, at_rest_after_drag, atomic_json
from test_dobot_joint_move import Feedback, set_mode
from test_dobot_cra_sensors import FakeConnection, packet, wait_until


def recorded_store(tmp_path):
    return TrajectoryStore(tmp_path / "records")


def recording(store, angles=None, name="测试轨迹"):
    record = store.create(name, "127.0.0.1")
    points = [{"time_ms": i * 50, "joints_deg": [float(q), 0, 0, 0, 0, 0]}
              for i, q in enumerate(angles or [0, 1, 2, 3])]
    data = {"version": 1, "sample_ms": 50,
            "frames": {"user": 0, "tool": 0, "user_value": [0.0] * 6, "tool_value": [0.0] * 6},
            "points": points}
    path = store._directory(record["name"]) / "samples.json"
    atomic_json(path, data)
    record.update(state="recorded", point_count=len(points), duration_s=points[-1]["time_ms"] / 1000,
                  samples_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    store.save(record)
    return record, data


def recorder(tmp_path):
    store = recorded_store(tmp_path)
    record = store.create("recording", "127.0.0.1")
    client = FeedbackClient({})
    subscription = client.subscribe(2048)
    finished = []
    recorder = TrajectoryRecorder(store, record, subscription, lambda: finished.append(True))
    state = Feedback()
    set_mode(state, 6)
    state.sample["data"]["timestamp"] = 1000
    return recorder, state.sample["data"], finished


def test_resampling_uses_controller_timestamps_not_display_or_arrival_time(tmp_path):
    rec, state, _ = recorder(tmp_path)
    for t in range(0, 105, 8):
        state["timestamp"] = 1000 + t
        state["joint_state"]["q_actual"] = [t / 10, 0, 0, 0, 0, 0]
        rec._consume({"data": copy.deepcopy(state), "received_at": 123})
    assert [p["time_ms"] for p in rec._points] == [0, 50, 100]
    assert [p["joints_deg"][0] for p in rec._points] == [0, 5, 10]
    rec.subscription.close()


@pytest.mark.parametrize("fault", ["gap", "duplicate", "backward", "disabled", "alarm", "collision", "skin", "model", "frame", "nan", "limit"])
def test_recording_rejects_discontinuities_and_protection_changes(tmp_path, fault):
    rec, state, _ = recorder(tmp_path)
    rec._consume({"data": copy.deepcopy(state)})
    state["timestamp"] += 8
    if fault == "gap": state["timestamp"] += 100
    if fault == "duplicate": state["timestamp"] = 1000
    if fault == "backward": state["timestamp"] = 999
    if fault == "disabled": state["enable_status"] = False
    if fault == "alarm": state["error_status"] = True
    if fault == "collision": state["collision_state"] = True
    if fault == "skin": state["j4_approach_state"] = True
    if fault == "model": state["robot_type"] = 114
    if fault == "frame": state["tool_value"][0] = 10
    if fault == "nan": state["joint_state"]["q_actual"][0] = float("nan")
    if fault == "limit": state["joint_state"]["q_actual"][2] = 156
    with pytest.raises((ValueError, RuntimeError)):
        rec._consume({"data": state})
    rec.subscription.close()


def feed(rec, state, mode, packets, *, step_ms=8, velocity=0.0):
    """Advance the controller clock; the hand moves joint1 at `velocity` deg/s."""
    results = []
    for _ in range(packets):
        state["timestamp"] += step_ms
        state["robot_mode"], state["drag_status"] = mode, mode == 6
        state["joint_state"]["q_actual"][0] += velocity * step_ms / 1000
        results.append(rec._consume({"data": copy.deepcopy(state)}))
    return results


def waiting(tmp_path):
    rec, state, finished = recorder(tmp_path)
    state["robot_mode"], state["drag_status"] = 5, False
    return rec, state, finished


def test_waiting_for_first_drag_produces_no_points(tmp_path):
    rec, state, _ = waiting(tmp_path)
    assert feed(rec, state, 5, 50) == [None] * 50
    assert rec._points == [] and not rec.drag_started.is_set() and rec._origin is None
    rec.subscription.close()


def test_leaving_drag_ends_the_take_but_pauses_before_it_are_kept_on_one_time_axis(tmp_path):
    rec, state, _ = waiting(tmp_path)
    feed(rec, state, 5, 10)
    first = feed(rec, state, 6, 25, velocity=100)         # 200 ms drag
    held = state["joint_state"]["q_actual"][0]
    assert rec.drag_started.is_set() and not rec.drag_exited
    feed(rec, state, 5, 3)                                  # 16 ms of mode 5 is not yet an exit
    assert not rec.drag_exited
    feed(rec, state, 6, 1)                                  # back in drag within the window
    feed(rec, state, 5, 2)
    assert not rec.drag_exited
    pause = feed(rec, state, 5, 50)                         # mode 5 for >= 24 ms: the arm left drag
    assert rec.drag_exited and not rec.limit_reached
    assert first[0]["time_ms"] == 0 and all(item is not None for item in pause)
    times = [point["time_ms"] for point in rec._points]
    assert times == list(range(0, len(times) * 50, 50))
    # The idle tail stays on the time axis at the held angle.
    assert abs(rec._points[-1]["joints_deg"][0] - held) < 1.0
    rec.subscription.close()


def test_first_drag_packet_marks_the_drag_as_started(tmp_path):
    rec, state, _ = waiting(tmp_path)
    feed(rec, state, 5, 5)
    assert not rec.drag_started.is_set() and not rec.drag_exited
    feed(rec, state, 0, 1)                                  # field: entry is reported as mode 0
    assert rec.drag_started.is_set()
    rec.subscription.close()


def test_mode5_coasting_after_drag_is_still_captured(tmp_path):
    rec, state, _ = recorder(tmp_path)
    feed(rec, state, 6, 10, velocity=50)
    state["joint_state"]["qd_actual"] = [20, 0, 0, 0, 0, 0]
    coasting = feed(rec, state, 5, 10, velocity=20)
    assert all(item["robot_mode"] == 5 for item in coasting)
    assert coasting[-1]["joints_deg"][0] > coasting[0]["joints_deg"][0]
    assert not at_rest_after_drag(state)
    state["joint_state"]["qd_actual"] = [0.1, -0.1, 0, 0, 0, 0]
    assert at_rest_after_drag(state)
    rec.subscription.close()


@pytest.mark.parametrize("field", ["running_status", "jog_status", "run_queued_cmd", "pause_cmd_flag"])
@pytest.mark.parametrize("phase", ["waiting", "pause"])
def test_mode5_with_activity_is_not_an_idle_pause(tmp_path, field, phase):
    rec, state, _ = waiting(tmp_path)
    if phase == "pause":
        feed(rec, state, 6, 10)
        feed(rec, state, 5, 15)  # past the 100 ms post-drag running blip window
    state[field] = 1
    with pytest.raises(RuntimeError, match="interrupted"):
        feed(rec, state, 5, 1)
    rec.subscription.close()


@pytest.mark.parametrize("mode", [7, 8, 9, 10, 11])
@pytest.mark.parametrize("phase", ["waiting", "drag", "pause"])
def test_other_modes_with_clean_flags_end_recording(tmp_path, mode, phase):
    rec, state, _ = waiting(tmp_path)
    if phase != "waiting":
        feed(rec, state, 6, 10)
    if phase == "pause":
        feed(rec, state, 5, 5)
    with pytest.raises(RuntimeError, match=f"controller mode {mode}"):
        feed(rec, state, mode, 1)
    rec.subscription.close()


@pytest.mark.parametrize("fault", ["brake", "skin", "alarm", "disabled"])
def test_protection_is_checked_before_mode5_is_accepted(tmp_path, fault):
    rec, state, _ = recorder(tmp_path)
    feed(rec, state, 6, 10)
    if fault == "brake": state["brake_released"][3] = True
    if fault == "skin": state["arm_approach_state"] = True
    if fault == "alarm": state["error_status"] = True
    if fault == "disabled": state["enable_status"] = False
    with pytest.raises(RuntimeError, match="recording interrupted: "):
        feed(rec, state, 5, 1)
    rec.subscription.close()


def replay(rec, state, rows, step_ms=8):
    """rows: (packets, mode, drag_status, running_status, joint1 velocity deg/s)."""
    results = []
    for packets, mode, drag, running, velocity in rows:
        for _ in range(packets):
            state["timestamp"] += step_ms
            state["robot_mode"], state["drag_status"], state["running_status"] = mode, drag, running
            state["joint_state"]["q_actual"][0] += velocity * step_ms / 1000
            results.append(rec._consume({"data": copy.deepcopy(state)}))
    return results


# CR3A drag-button capture, 2026-11-10 11:45 (30004, 8 ms): entering drag reports
# mode 0 with enable=1/drag=1 for ~24 ms; mode 6 has running=1 while the hand moves;
# leaving drag reports one mode-5 packet with running=1 before mode 5 is clean.
FIELD_DRAG = [(10, 5, 0, 0, 0), (3, 0, 1, 0, 0), (30, 6, 1, 1, 6), (1, 5, 0, 1, 0), (20, 5, 0, 0, 0)]


def test_field_drag_button_sequence_replays_without_interruption(tmp_path):
    rec, state, _ = waiting(tmp_path)
    results = replay(rec, state, FIELD_DRAG * 2)
    assert results[:10] == [None] * 10
    # Mode 0 already carries real angles and starts the time axis.
    assert results[10]["robot_mode"] == 0 and results[10]["time_ms"] == 0
    assert rec.drag_exited and all(item is not None for item in results[10:])
    times = [point["time_ms"] for point in rec._points]
    assert times == list(range(0, len(times) * 50, 50))
    rec.subscription.close()


@pytest.mark.parametrize("rows", [
    [(1, 0, 0, 0, 0)],
    [(14, 0, 1, 0, 0)],
    [(3, 0, 1, 0, 0), (1, 5, 0, 0, 0)],
])
def test_mode0_is_not_an_error_or_drag_transition_gate(tmp_path, rows):
    rec, state, _ = waiting(tmp_path)
    results = replay(rec, state, rows)
    assert all(item is not None for item in results)
    rec.subscription.close()


def test_mode0_does_not_whitelist_other_unsupported_modes(tmp_path):
    rec, state, _ = waiting(tmp_path)
    with pytest.raises(RuntimeError, match="controller mode 7"):
        replay(rec, state, [(3, 0, 1, 0, 0), (1, 7, 0, 0, 0)])
    rec.subscription.close()


def test_mode0_without_enable_is_a_protection_fault(tmp_path):
    rec, state, _ = waiting(tmp_path)
    state["enable_status"] = False
    with pytest.raises(RuntimeError, match="robot disabled"):
        replay(rec, state, [(1, 0, 1, 0, 0)])
    rec.subscription.close()


@pytest.mark.parametrize("rows,flag", [
    ([(13, 5, 0, 1, 0)], None),                       # running > 100 ms after drag
    ([(1, 5, 0, 1, 0)], "run_queued_cmd"),            # running plus a queue is a motion
    ([(1, 5, 0, 1, 0)], "jog_status"),
])
def test_running_after_drag_is_only_tolerated_as_a_short_blip(tmp_path, rows, flag):
    rec, state, _ = waiting(tmp_path)
    replay(rec, state, [(3, 0, 1, 0, 0), (10, 6, 1, 0, 5)])
    if flag:
        state[flag] = 1
    with pytest.raises(RuntimeError, match="recording interrupted: "):
        replay(rec, state, rows)
    rec.subscription.close()


def test_running_in_mode5_before_any_drag_is_not_a_blip(tmp_path):
    rec, state, _ = waiting(tmp_path)
    with pytest.raises(RuntimeError, match="robot is running"):
        replay(rec, state, [(1, 5, 0, 1, 0)])
    rec.subscription.close()


def test_mode6_without_drag_status_is_an_inconsistent_controller_state(tmp_path):
    rec, state, _ = waiting(tmp_path)
    replay(rec, state, [(3, 0, 1, 0, 0), (10, 6, 1, 0, 5)])
    with pytest.raises(RuntimeError, match="drag state inconsistent"):
        replay(rec, state, [(1, 6, 0, 0, 0)])
    rec.subscription.close()


def test_recording_limit_includes_pauses_and_is_not_reset_by_reentering_drag(tmp_path):
    rec, state, _ = waiting(tmp_path)
    rec.max_duration_s = .5
    feed(rec, state, 5, 20)                                # waiting does not count
    feed(rec, state, 6, 12)
    feed(rec, state, 5, 37)                                # ~392 ms elapsed incl. pause
    assert not rec.limit_reached
    feed(rec, state, 6, 20)                                # re-entering drag keeps the origin
    assert rec.limit_reached and rec._previous["time_ms"] >= 500
    rec.subscription.close()


def test_point_count_never_exceeds_max_points(tmp_path, monkeypatch):
    monkeypatch.setattr(trajectory, "MAX_POINTS", 3)
    rec, state, _ = recorder(tmp_path)
    feed(rec, state, 6, 40)
    assert len(rec._points) == 3 and rec.limit_reached
    rec.subscription.close()


def test_subscription_overflow_is_explicit_and_does_not_block_receiver():
    client = FeedbackClient({})
    subscription = client.subscribe(1)
    subscription.put({"packet": 1})
    subscription.put({"packet": 2})
    with pytest.raises(RuntimeError, match="overflow"):
        subscription.read()
    subscription.close()
    assert not client._subscriptions


def test_recording_tap_keeps_each_packet_when_feedback_arrives_in_one_burst():
    packets = []
    for index in range(3):
        value = bytearray(packet(mode=6, angle=index))
        struct.pack_into("<Q", value, 32, 1000 + index * 8)
        packets.append(bytes(value))
    connection = FakeConnection([b"".join(packets)])
    client = FeedbackClient({"controller_ip": "127.0.0.1"}, connection_factory=lambda *args: connection)
    subscription = client.subscribe()
    try:
        client.start()
        wait_until(lambda: client.snapshot()["packets_received"] == 3)
        captured = [subscription.read() for _ in range(3)]
        assert [item["data"]["timestamp"] for item in captured] == [1000, 1008, 1016]
        assert [item["data"]["joint_state"]["q_actual"][0] for item in captured] == [0, 1, 2]
        assert client.snapshot()["data"]["joint_state"]["q_actual"][0] == 2
    finally:
        subscription.close()
        client.stop()


def test_failed_recording_leaves_partial_data_and_confirms_cleanup(tmp_path):
    rec, state, finished = recorder(tmp_path)
    rec.subscription.put({"data": state, "received_at": time.monotonic()})
    rec.start()
    assert rec.finished.wait(1)
    assert rec.record["state"] == "incomplete"
    assert "feedback missing" in rec.record["reason"]
    assert finished == [True]
    assert (rec.store._directory(rec.record["name"]) / "raw.jsonl").exists()


def test_restart_marks_unfinished_recording_incomplete(tmp_path):
    store = recorded_store(tmp_path)
    record = store.create("unfinished", "127.0.0.1")
    recovered = TrajectoryStore(store.root)
    assert recovered.load(record["name"])["state"] == "incomplete"
    assert "restarted" in recovered.load(record["name"])["reason"]


def test_playback_returns_verified_samples_and_delete_removes_local_files(tmp_path):
    store = recorded_store(tmp_path)
    record, data = recording(store)
    directory = store._directory(record["name"])
    (directory / "native.csv").write_text("left by an earlier version")
    loaded, samples = store.playback(record["name"], "127.0.0.1")
    assert loaded["name"] == record["name"] and samples["points"] == data["points"]
    assert "native_state" not in store.summary(loaded)
    assert store.delete(record["name"]) == {"name": record["name"], "state": "deleted"}
    assert not directory.exists() and store.list_trajectories() == []


def test_names_are_the_only_identity_and_a_duplicate_is_rejected(tmp_path):
    store = recorded_store(tmp_path)
    created = store.create("同名轨迹", "127.0.0.1")
    assert "trajectory_id" not in created and created["name"] == "同名轨迹"
    store.create("e\u0301", "127.0.0.1")
    with pytest.raises(ValueError, match="already exists"):
        store.create("\u00e9", "127.0.0.1")  # canonically equivalent
    with pytest.raises(ValueError, match="already exists"):
        store.create("  同名轨迹  ", "127.0.0.1")
    with pytest.raises(ValueError, match="already exists"):
        store.check_new("同名轨迹")
    store.check_new("另一个名字")


def test_recording_time_is_stored_and_listed(tmp_path):
    store = recorded_store(tmp_path)
    record = store.create("timed", "127.0.0.1")
    assert datetime.fromisoformat(record["recorded_at"]).tzinfo is not None
    assert abs(record["created_at_ms"] / 1000 - time.time()) < 5
    assert store.list_trajectories()[0]["recorded_at"] == record["recorded_at"]
    assert store.summary(record)["recorded_at"] == record["recorded_at"]


def test_trajectories_are_listed_newest_first(tmp_path):
    store = recorded_store(tmp_path)
    for index, name in enumerate(("old", "new")):
        record = store.create(name, "127.0.0.1")
        record["recorded_at"] = f"2026-11-1{index}T10:00:00+08:00"
        store.save(record)
    assert [item["name"] for item in store.list_trajectories()] == ["new", "old"]


def test_wrong_controller_is_rejected(tmp_path):
    store = recorded_store(tmp_path)
    record, _ = recording(store)
    with pytest.raises(RuntimeError, match="different controller"):
        store.playback(record["name"], "127.0.0.2")


def test_incomplete_recording_cannot_be_played(tmp_path):
    store = recorded_store(tmp_path)
    record = store.create("unfinished", "127.0.0.1")
    with pytest.raises(RuntimeError, match="not complete"):
        store.playback(record["name"], "127.0.0.1")


def test_servo_playback_interpolates_along_the_recording():
    playback = ServoPlayback("a" * 32, [[0.0] * 6, [1.0] + [0.0] * 5, [3.0] + [0.0] * 5], 1)
    assert playback.duration_ms == 100
    assert playback.at(0)[0] == 0 and playback.at(25)[0] == pytest.approx(0.5)
    assert playback.at(75)[0] == pytest.approx(2.0) and playback.at(100)[0] == 3.0


def test_tampered_recording_is_rejected(tmp_path):
    store = recorded_store(tmp_path)
    record, _ = recording(store)
    (store._directory(record["name"]) / "samples.json").write_text("{}")
    with pytest.raises(RuntimeError, match="checksum"):
        store.samples(record["name"])


@pytest.mark.parametrize("bad", [None, 1, "", "   ", "a\x00b", "line\nbreak", "A" * 81])
def test_invalid_names_are_rejected(tmp_path, bad):
    with pytest.raises(ValueError): TrajectoryStore(tmp_path)._directory(bad)


def test_any_printable_name_maps_to_a_safe_directory(tmp_path):
    store = TrajectoryStore(tmp_path)
    for name in ("../x", "x);Stop();", "路径/轨迹", "a" * 80):
        directory = store._directory(name)
        assert directory.parent == store.root and len(directory.name) == 32


def test_card_contract_keeps_sensor_and_confirmation_conventions():
    plugin = DobotSensors({})
    definitions = {definition["name"]: definition for definition in plugin.get_tools()}
    teach = definitions["teach"]
    schema = teach["inputSchema"]
    assert teach["type"] == "actuator" and schema["x-resource"] == "arm"
    assert "topic_out" not in teach
    assert "get_status" not in schema["properties"]["action"]["enum"]
    assert "detect_collision" not in schema["x-action-params"]["resume"]["params"]
    assert set(schema["properties"]["confirm_move"]) == {"type", "description"}
    assert schema["x-completion"]["timeout"] > 2000
    assert plugin.dispatch("info", {"_tool_name": "teach"}) == teach
    actions = schema["properties"]["action"]["enum"]
    assert {"start_trajectory", "stop_trajectory", "list_trajectories", "delete_trajectory", "play"} <= set(actions)
    assert "go_to_start" not in actions
    play = schema["x-action-params"]["play"]["params"]
    assert play == ["name", "speed", "approach_speed", "detect_collision", "pause_at_start", "return_to_start",
                    "confirm_move"]
    properties = schema["properties"]
    assert properties["speed"]["enum"] == ["0.25x", "0.5x", "1x", "1.5x", "2x"] and properties["speed"]["default"] == "1x"
    assert properties["approach_speed"]["enum"] == ["slow", "normal", "fast"]
    assert properties["approach_speed"]["default"] == "slow"
    assert properties["detect_collision"]["default"] == "true"
    assert all(properties[k]["type"] == "string" and properties[k]["enum"] == ["true", "false"]
               for k in ("detect_collision", "pause_at_start", "return_to_start"))
    assert properties["pause_at_start"]["default"] == "false" and properties["return_to_start"]["default"] == "false"
    assert not {"velocity", "acceleration", "speed_multiplier"} & set(properties)
    assert not any("record" in action for action in actions)
    assert "trajectory_id" not in schema["properties"]
    assert "name" in schema["properties"]
    assert "action" not in definitions["joints"]["inputSchema"]["properties"] or "start_trajectory" not in str(definitions["joints"])


@pytest.mark.parametrize("bad", [None, 1, "true", False])
def test_teaching_requires_literal_true(bad):
    plugin = DobotSensors({})
    with pytest.raises(ValueError, match="confirm_move"):
        plugin.teach.start("start_trajectory", {"name": "a", "confirm_move": bad})


def test_interruption_is_irrevocable_and_sealed_at_final_save(tmp_path):
    rec, state, _ = recorder(tmp_path)
    assert rec.stop("driver stopped during recording")
    rec.stop(None)
    assert rec.reason == "driver stopped during recording"
    assert rec._seal(None) == "driver stopped during recording"
    assert rec._seal("cleanup; driver stopped during recording").count("driver stopped") == 1
    assert rec.stop("late") is False and rec.reason == "driver stopped during recording"
    rec.subscription.close()
