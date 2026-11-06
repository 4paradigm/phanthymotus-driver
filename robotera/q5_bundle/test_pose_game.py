from __future__ import annotations

import json
import queue
import threading
import time

import pytest

import pose_game


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
            "kind": "barrier", "token": token,
            "bridge_received_monotonic_ns": self.clock(),
            "bridge_transport_drops": self.transport_drops,
        })

    def put_pose(self, person_id, bridge_ns):
        self.items.put({
            "kind": "pose",
            "raw_json": json.dumps({
                "timestamp": -bridge_ns,
                "persons": [{"id": person_id, "future_keypoint": [1, 2]}],
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


def make_plugin(clock, **config):
    client = FakeClient(clock)
    defaults = {
        "topic": pose_game.POSE_TOPIC,
        "max_frames": 20,
        "max_rounds": 10,
        "max_frames_per_round": 20,
        "barrier_timeout_seconds": 0.5,
    }
    defaults.update(config)
    return pose_game.Plugin(defaults, "nvidia_desktop", None, client,
                            clock_ns=clock), client


def test_pump_lifecycle_uses_fixed_topic_and_ingests_independently():
    clock = FakeClock(100)
    plugin, client = make_plugin(clock)

    plugin.start()
    plugin.dispatch("start_round", {"round_id": "r1"})
    blocked = threading.Event()
    release = threading.Event()

    def blocked_tts():
        blocked.set()
        release.wait(1)

    waiter = threading.Thread(target=blocked_tts)
    waiter.start()
    assert blocked.wait(1)
    client.put_pose("during-blocked-tts", 101)
    assert wait_until(lambda: plugin.dispatch("info", {})["buffered_frames"] == 1)

    release.set()
    waiter.join(1)
    plugin.stop()
    plugin.stop()
    assert client.configured == [pose_game.POSE_TOPIC, ""]
    assert plugin._pump_thread is None


def test_finish_barrier_drains_queued_pose_before_freeze_and_isolates_next_round():
    clock = FakeClock(1_000)
    plugin, client = make_plugin(clock)
    plugin.start()
    plugin.dispatch("start_round", {"round_id": "round-1"})

    client.put_pose("queued-1", 1_001)
    client.put_pose("queued-2", 1_002)
    clock.set(1_002)
    frozen = plugin.dispatch("finish_round", {
        "round_id": "round-1", "outcome": "PLAYBACK_FAILED",
        "playback_result": {"result": "failed", "error_code": "MOCK"},
    })
    assert [frame["persons"][0]["id"] for frame in frozen["frames"]] == [
        "queued-1", "queued-2"]

    clock.set(2_000)
    plugin.dispatch("start_round", {"round_id": "round-2"})
    client.put_pose("round-2-only", 2_001)
    clock.set(2_001)
    second = plugin.dispatch("finish_round", {
        "round_id": "round-2", "outcome": "TIMEOUT",
        "playback_result": {"result": "timeout"},
    })
    assert [frame["persons"][0]["id"] for frame in second["frames"]] == [
        "round-2-only"]
    assert [frame["persons"][0]["id"] for frame in
            plugin.dispatch("export_round", {"round_id": "round-1"})["frames"]] == [
        "queued-1", "queued-2"]
    plugin.stop()


def test_freeze_boundary_is_bridge_barrier_time_not_later_driver_wakeup():
    clock = FakeClock(10)
    plugin, client = make_plugin(clock)
    original_barrier = client.request_pose_barrier

    def barrier_then_newer_pose(token):
        original_barrier(token)
        client.put_pose("after-barrier", 101)
        clock.set(110)

    client.request_pose_barrier = barrier_then_newer_pose
    plugin.start()
    plugin.dispatch("start_round", {"round_id": "cut-at-barrier"})
    client.put_pose("before-barrier", 99)
    clock.set(100)

    frozen = plugin.dispatch("finish_round", {
        "round_id": "cut-at-barrier", "outcome": "PLAYBACK_FAILED",
        "playback_result": {"result": "failed"},
    })

    assert frozen["ended_monotonic_ns"] == 100
    assert [frame["persons"][0]["id"] for frame in frozen["frames"]] == [
        "before-barrier"]
    plugin.stop()


def test_finish_barrier_timeout_does_not_fake_freeze_or_lose_active_round():
    clock = FakeClock(100)
    plugin, client = make_plugin(clock, barrier_timeout_seconds=0.01)
    client.request_pose_barrier = lambda _token: None
    plugin.start()
    plugin.dispatch("start_round", {"round_id": "still-active"})

    with pytest.raises(TimeoutError, match="Pose barrier"):
        plugin.dispatch("finish_round", {
            "round_id": "still-active", "outcome": "TIMEOUT",
        })

    assert plugin.dispatch("info", {})["active_round"] == "still-active"
    plugin.stop()


def test_info_keeps_bridge_drops_distinct_from_recorder_evictions():
    clock = FakeClock(0)
    plugin, client = make_plugin(clock, max_frames=1)
    client.transport_drops = 7
    plugin.start()
    client.put_pose("one", 1)
    client.put_pose("two", 2)
    assert wait_until(lambda: plugin.dispatch("info", {})["recorder_cache_evictions"] == 1)

    info = plugin.dispatch("info", {})
    assert info["bridge_transport_drops"] == 7
    assert info["recorder_cache_evictions"] == 1
    plugin.stop()


def test_bridge_drops_during_round_mark_frozen_snapshot_incomplete():
    clock = FakeClock(100)
    plugin, client = make_plugin(clock)
    client.transport_drops = 2
    plugin.start()
    plugin.dispatch("start_round", {"round_id": "drops"})
    client.transport_drops = 3

    snapshot = plugin.dispatch("finish_round", {
        "round_id": "drops", "outcome": "PLAYBACK_FAILED",
    })

    assert snapshot["complete"] is False
    assert snapshot["bridge_transport_drops"] == 1
    assert snapshot["incomplete_reasons"] == ["bridge_transport_drop"]
    plugin.stop()


def test_completed_finish_requires_exact_trusted_evidence_and_three_seconds():
    clock = FakeClock(4_000_000_000)
    plugin, _client = make_plugin(clock)
    plugin.start()
    plugin.dispatch("start_round", {"round_id": "untrusted"})

    with pytest.raises(ValueError, match="trusted playback evidence"):
        plugin.dispatch("finish_round", {
            "round_id": "untrusted", "outcome": "COMPLETED",
            "completion_observed_monotonic_ns": 1_000_000_000,
            "playback_result": {"result": "completed"},
        })
    assert plugin.dispatch("info", {})["active_round"] == "untrusted"
    plugin.stop()
