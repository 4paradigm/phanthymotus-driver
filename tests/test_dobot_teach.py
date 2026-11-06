"""Offline recording integrity, native-format boundary and card contracts."""

import copy
import hashlib
import json
import struct
import threading
import time

import pytest

from dobot.cra.control import parse_start_pose, trajectory_filename
from dobot.cra.device import DobotSensors
from dobot.cra.feedback import FeedbackClient, FeedbackSubscription
from dobot.cra.trajectory import NativeCsvProfile, TrajectoryRecorder, TrajectoryStore, atomic_json
from test_dobot_joint_move import Feedback, set_mode
from test_dobot_cra_sensors import FakeConnection, packet, wait_until


def recorded_store(tmp_path, native=True, shared=True):
    root = tmp_path / "records"
    controller = tmp_path / "controller"
    if shared:
        controller.mkdir(exist_ok=True)
    profile = tmp_path / "simulator-profile.json"
    profile.write_text(json.dumps({
        "version": 1, "format_id": "SIMULATOR_ONLY_NOT_A_DOBOT_FORMAT",
        "header": [["SIMULATOR_ONLY_NOT_A_DOBOT_FORMAT"]],
        "columns": [f"joint{i}_deg" for i in range(1, 7)],
    }))
    store = TrajectoryStore(root, str(controller) if shared else "", str(profile) if native else "")
    return store


def recording(store, angles=None, name="测试轨迹"):
    record = store.create(name, "127.0.0.1")
    points = [{"time_ms": i * 50, "joints_deg": [float(q), 0, 0, 0, 0, 0]}
              for i, q in enumerate(angles or [0, 1, 2, 3])]
    data = {"version": 1, "sample_ms": 50,
            "frames": {"user": 0, "tool": 0, "user_value": [0.0] * 6, "tool_value": [0.0] * 6},
            "points": points}
    path = store._directory(record["trajectory_id"]) / "samples.json"
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


@pytest.mark.parametrize("fault", ["gap", "duplicate", "backward", "mode", "disabled", "alarm", "collision", "skin", "model", "frame", "nan", "limit"])
def test_recording_rejects_discontinuities_and_protection_changes(tmp_path, fault):
    rec, state, _ = recorder(tmp_path)
    rec._consume({"data": copy.deepcopy(state)})
    state["timestamp"] += 8
    if fault == "gap": state["timestamp"] += 100
    if fault == "duplicate": state["timestamp"] = 1000
    if fault == "backward": state["timestamp"] = 999
    if fault == "mode": state["robot_mode"] = 5
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
    assert (rec.store._directory(rec.record["trajectory_id"]) / "raw.jsonl").exists()


def test_restart_marks_unfinished_recording_incomplete(tmp_path):
    store = recorded_store(tmp_path)
    record = store.create("unfinished", "127.0.0.1")
    recovered = TrajectoryStore(store.root)
    assert recovered.load(record["trajectory_id"])["state"] == "incomplete"
    assert "restarted" in recovered.load(record["trajectory_id"])["reason"]


@pytest.mark.parametrize("native,shared,state", [(False, True, "format_required"), (True, False, "transfer_required")])
def test_missing_native_contract_blocks_playback_but_keeps_recording(tmp_path, native, shared, state):
    store = recorded_store(tmp_path, native, shared)
    record, _ = recording(store)
    assert store.export(record["trajectory_id"])["native_state"] == state
    with pytest.raises(RuntimeError, match=state):
        store.playback(record["trajectory_id"], "127.0.0.1")
    assert store.load(record["trajectory_id"])["state"] == "recorded"


def test_native_bridge_readback_collision_and_local_delete(tmp_path):
    store = recorded_store(tmp_path)
    record, data = recording(store)
    result = store.export(record["trajectory_id"])
    remote = store.controller_root / result["controller_file"]
    assert result["native_state"] == "exported"
    assert hashlib.sha256(remote.read_bytes()).hexdigest() == result["native_sha256"]
    store.playback(record["trajectory_id"], "127.0.0.1")
    remote.write_text("different file")
    with pytest.raises(RuntimeError, match="overwrite"):
        store.export(record["trajectory_id"])
    store.delete(record["trajectory_id"])
    assert remote.exists()
    assert store.list_records() == []


def test_wrong_controller_does_not_upload_anything(tmp_path):
    store = recorded_store(tmp_path)
    record, _ = recording(store)
    with pytest.raises(RuntimeError, match="different controller"):
        store.playback(record["trajectory_id"], "127.0.0.2")
    assert not list(store.controller_root.iterdir())


def test_tampered_recording_is_rejected(tmp_path):
    store = recorded_store(tmp_path)
    record, _ = recording(store)
    (store._directory(record["trajectory_id"]) / "samples.json").write_text("{}")
    with pytest.raises(RuntimeError, match="checksum"):
        store.samples(record["trajectory_id"])


@pytest.mark.parametrize("bad", ["../x", "../x.csv", "x.csv);Stop();", "x\n.csv", "x.csv/", "", None])
def test_paths_and_protocol_injection_are_rejected(tmp_path, bad):
    with pytest.raises(ValueError): trajectory_filename(bad)
    with pytest.raises(ValueError): TrajectoryStore(tmp_path)._directory(bad)


@pytest.mark.parametrize("reply", [
    '0,{1,{1,2,3,4,5,6}},GetStartPose("a.csv",1);',
    '0,{0,{1,2,3,4,5,6},0,0,{1,2,3,4,5,6}},GetStartPose("a.csv",1);',
])
def test_nested_start_pose_reply_is_parsed(reply):
    assert parse_start_pose(reply) == [1, 2, 3, 4, 5, 6]


@pytest.mark.parametrize("reply", [
    '0,{2,{1,2,3,4,5,6}},GetStartPose("a.csv",1);',
    '0,{1,{1,2,156,4,5,6}},GetStartPose("a.csv",1);',
    '0,{1,{1,2}},GetStartPose("a.csv",1);',
    '-1,{},GetStartPose("a.csv",1);', '0,{},MovJ();',
])
def test_unusable_start_pose_is_rejected(reply):
    with pytest.raises((ValueError, RuntimeError)):
        parse_start_pose(reply)


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
    assert "action" not in definitions["joints"]["inputSchema"]["properties"] or "start_record" not in str(definitions["joints"])


@pytest.mark.parametrize("bad", [None, 1, "true", False])
def test_teaching_requires_literal_true(bad):
    plugin = DobotSensors({})
    with pytest.raises(ValueError, match="confirm_move"):
        plugin.teach.start("start_record", {"name": "a", "confirm_move": bad})
