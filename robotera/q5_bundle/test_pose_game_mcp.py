from __future__ import annotations

import json
import http.client
import queue
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import yaml

import main as q5_main
import pose_game


TRUSTED_COMPLETION = {
    "result": "completed",
    "completion_level": "alsa_device_consumption",
    "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
    "playback_finished_at_ns": 123,
    "error_code": None,
    "error_message": None,
    "underrun_count": 0,
}


class FakeClock:
    def __init__(self, now_ns=0):
        self.now_ns = now_ns
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.now_ns

    def set(self, now_ns):
        with self.lock:
            self.now_ns = now_ns


class FakeClient:
    def __init__(self, clock):
        self.clock = clock
        self.items = queue.Queue()
        self.configured = []
        self.transport_drops = 0

    def configure_pose(self, topic):
        self.configured.append(topic)

    def pop_pose(self):
        try:
            return self.items.get_nowait()
        except queue.Empty:
            return None

    def pose_stats(self):
        return {"bridge_transport_drops": self.transport_drops}

    def request_pose_barrier(self, token):
        self.items.put({
            "kind": "barrier",
            "token": token,
            "bridge_received_monotonic_ns": self.clock(),
            "bridge_transport_drops": self.transport_drops,
        })

    def put_pose(self, person_id, bridge_ns):
        self.items.put({
            "kind": "pose",
            "raw_json": json.dumps({
                "timestamp": -bridge_ns,
                "persons": [{"id": person_id, "posture": "standing"}],
            }),
            "bridge_received_monotonic_ns": bridge_ns,
        })


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return False


def config(max_rounds=20):
    return {"plugins": {"pose_game": {
        "enabled": True,
        "topic": pose_game.POSE_TOPIC,
        "max_frames": 600,
        "max_rounds": max_rounds,
        "max_frames_per_round": 600,
        "max_raw_bytes": 1_048_576,
        "barrier_timeout_seconds": 0.5,
    }}}


def make_bundle(monkeypatch, clock, max_rounds=20):
    client = FakeClient(clock)
    monkeypatch.setattr(
        pose_game, "make_plugin",
        lambda plugin_config, namespace, executor, shared_client:
            pose_game.Plugin(plugin_config, namespace, executor, shared_client,
                             clock_ns=clock))
    bundle = q5_main.Q5Bundle(
        config(max_rounds), "nvidia_desktop", None, client)
    bundle.start_all()
    return bundle, client


def json_rpc(port, method, params=None):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
    body = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": method,
        "params": params or {},
    })
    connection.request("POST", "/mcp", body, {
        "Content-Type": "application/json",
    })
    response = connection.getresponse()
    payload = json.loads(response.read())
    connection.close()
    assert response.status == 200
    return payload


def rpc_tool_call(port, arguments):
    payload = json_rpc(port, "tools/call", {
        "name": "pose_game", "arguments": arguments,
    })
    return json.loads(payload["result"]["content"][0]["text"])


def test_q5bundle_dynamically_registers_one_pose_game_mcp_card():
    package_dir = Path(__file__).parent
    deployed_config = yaml.safe_load(
        (package_dir / "config.yaml").read_text(encoding="utf-8"))
    pose_config = deployed_config["plugins"]["pose_game"]
    assert pose_config["enabled"] is True
    assert "COPY pose_game.py /work/pose_game.py" in (
        package_dir / "Dockerfile").read_text(encoding="utf-8")
    bundle = q5_main.Q5Bundle(
        {"plugins": {"pose_game": pose_config}},
        "nvidia_desktop", None, FakeClient(FakeClock()))

    tools = bundle.get_all_tools()
    assert len(tools) == 1
    tool = tools[0]
    assert tool["name"] == "pose_game"
    assert tool["type"] == "sensor"
    assert tool["description"] == (
        "Bounded Pose capture windows for offline action scoring")
    schema = tool["inputSchema"]
    assert schema["properties"]["action"]["enum"] == [
        "start_round", "finish_round", "export_round", "info"]
    assert schema["properties"]["outcome"]["enum"] == [
        "COMPLETED", "PLAYBACK_FAILED", "TIMEOUT", "MANUAL_TERMINATED",
        "SUBMIT_FAILED", "UNTRUSTED_COMPLETION"]
    assert schema["additionalProperties"] is False
    assert schema["x-action-params"] == {
        "start_round": {"params": ["round_id"],
                        "description": "Start capture before submitting TTS."},
        "finish_round": {
            "params": ["round_id", "outcome", "playback_result",
                       "completion_observed_monotonic_ns"],
            "description": "Freeze after a terminal outcome; playback evidence fields apply only to COMPLETED."},
        "export_round": {"params": ["round_id"],
                         "description": "Export one retained frozen round."},
        "info": {"params": [],
                 "description": "Read active round, capacities, and counters."},
    }


def test_json_rpc_tools_list_and_info_call_expose_pose_game_card():
    previous_bundle = q5_main._bundle
    q5_main._bundle = q5_main.Q5Bundle(
        config(), "nvidia_desktop", None, FakeClient(FakeClock()))
    server = ThreadingHTTPServer(("127.0.0.1", 0), q5_main.make_handler())
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        listed = json_rpc(server.server_port, "tools/list")
        assert [tool["name"] for tool in listed["result"]["tools"]] == [
            "pose_game"]

        called = json_rpc(server.server_port, "tools/call", {
            "name": "pose_game", "arguments": {"action": "info"},
        })
        content = called["result"]["content"]
        assert len(content) == 1
        assert json.loads(content[0]["text"])["active_round"] is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1)
        q5_main._bundle = previous_bundle


def test_mcp_card_full_round_keeps_sampling_during_mock_playback_and_post_window(
        monkeypatch):
    clock = FakeClock(0)
    bundle, client = make_bundle(monkeypatch, clock)
    previous_bundle = q5_main._bundle
    q5_main._bundle = bundle
    server = ThreadingHTTPServer(("127.0.0.1", 0), q5_main.make_handler())
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start()
    playback_blocked = threading.Event()
    playback_completed = threading.Event()
    try:
        started = rpc_tool_call(server.server_port, {
            "action": "start_round", "round_id": "round-1"})
        assert started["state"] == "RECORDING"

        waiter = threading.Thread(target=lambda: (
            playback_blocked.set(), playback_completed.wait(1)))
        waiter.start()
        assert playback_blocked.wait(1)
        client.put_pose("speaking-1", 100)
        client.put_pose("speaking-2", 200)
        assert wait_until(lambda: rpc_tool_call(
            server.server_port, {"action": "info"})["buffered_frames"] == 2)

        clock.set(1_000_000_000)
        playback_completed.set()
        waiter.join(1)
        client.put_pose("post-1", 1_500_000_000)
        client.put_pose("post-boundary", 4_000_000_000)
        assert wait_until(lambda: rpc_tool_call(
            server.server_port, {"action": "info"})["buffered_frames"] == 4)
        clock.set(4_000_000_000)

        frozen = rpc_tool_call(server.server_port, {
            "action": "finish_round",
            "round_id": "round-1",
            "outcome": "COMPLETED",
            "playback_result": dict(TRUSTED_COMPLETION),
            "completion_observed_monotonic_ns": 1_000_000_000,
        })
        exported = rpc_tool_call(server.server_port, {
            "action": "export_round", "round_id": "round-1"})

        assert frozen["outcome"] == "COMPLETED"
        assert frozen["ended_monotonic_ns"] == 4_000_000_000
        assert exported == frozen
        assert [frame["persons"][0]["id"] for frame in exported["frames"]] == [
            "speaking-1", "speaking-2", "post-1", "post-boundary"]

        missing = json_rpc(server.server_port, "tools/call", {
            "name": "pose_game",
            "arguments": {"action": "export_round", "round_id": "missing"},
        })
        assert missing["error"]["code"] == -32603
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(1)
        bundle.stop_all()
        q5_main._bundle = previous_bundle


def test_mcp_card_ten_completed_rounds_remain_isolated(monkeypatch):
    clock = FakeClock(1_000)
    bundle, client = make_bundle(monkeypatch, clock, max_rounds=10)

    for index in range(10):
        round_id = f"round-{index}"
        start_ns = clock()
        bundle.dispatch("pose_game", {
            "action": "start_round", "round_id": round_id})
        client.put_pose(f"{round_id}-speaking", start_ns + 1)
        completion_ns = start_ns + 10
        client.put_pose(f"{round_id}-post", completion_ns + 3_000_000_000)
        assert wait_until(lambda: bundle.dispatch(
            "pose_game", {"action": "info"})["buffered_frames"] >= 2 * (index + 1))
        clock.set(completion_ns + 3_000_000_000)
        bundle.dispatch("pose_game", {
            "action": "finish_round",
            "round_id": round_id,
            "outcome": "COMPLETED",
            "playback_result": dict(TRUSTED_COMPLETION),
            "completion_observed_monotonic_ns": completion_ns,
        })
        clock.set(clock() + 100)

    for index in range(10):
        round_id = f"round-{index}"
        exported = bundle.dispatch("pose_game", {
            "action": "export_round", "round_id": round_id})
        assert [frame["persons"][0]["id"] for frame in exported["frames"]] == [
            f"{round_id}-speaking", f"{round_id}-post"]
    bundle.stop_all()
