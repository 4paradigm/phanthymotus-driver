from __future__ import annotations

import queue
import threading
import time
from pathlib import Path

import q5_media_bridge


class FakeCounter:
    def __init__(self):
        self.value = 0

    def get_lock(self):
        return self

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


class Message:
    data = '{"timestamp": 7, "persons": [{"id": "p1"}]}'


def test_pose_message_retains_raw_json_and_bridge_monotonic_timestamp():
    item = q5_media_bridge.pose_message_item(Message(), lambda: 123_456)

    assert item == {
        "kind": "pose",
        "raw_json": Message.data,
        "bridge_received_monotonic_ns": 123_456,
    }


def test_pose_lane_is_bounded_nonblocking_and_counts_transport_drops():
    pose_q = queue.Queue(maxsize=2)
    drops = FakeCounter()

    assert q5_media_bridge._offer_pose(pose_q, {"sequence": 1}, drops)
    assert q5_media_bridge._offer_pose(pose_q, {"sequence": 2}, drops)
    assert q5_media_bridge._offer_pose(pose_q, {"sequence": 3}, drops)

    assert pose_q.qsize() == 2
    assert [pose_q.get_nowait()["sequence"] for _ in range(2)] == [2, 3]
    assert drops.value == 1


def test_pose_barrier_uses_same_ordered_lane_as_prior_pose_messages():
    pose_q = queue.Queue(maxsize=4)
    drops = FakeCounter()
    q5_media_bridge._offer_pose(pose_q, {"kind": "pose", "sequence": 1}, drops)
    q5_media_bridge._offer_pose(pose_q, {"kind": "pose", "sequence": 2}, drops)
    q5_media_bridge._offer_pose(
        pose_q, q5_media_bridge.pose_barrier_item("freeze-1", lambda: 900), drops)

    assert [pose_q.get_nowait()["kind"] for _ in range(3)] == [
        "pose", "pose", "barrier"]
    assert drops.value == 0


def test_full_pose_lane_barrier_waits_for_space_without_evicting_pose():
    pose_q = queue.Queue(maxsize=2)
    drops = FakeCounter()
    pose_q.put_nowait({"kind": "pose", "sequence": 1})
    pose_q.put_nowait({"kind": "pose", "sequence": 2})
    removed = []

    def consume_one():
        time.sleep(0.01)
        removed.append(pose_q.get_nowait())

    consumer = threading.Thread(target=consume_one)
    consumer.start()
    assert q5_media_bridge._offer_pose_barrier(
        pose_q, q5_media_bridge.pose_barrier_item("freeze", lambda: 3),
        drops, timeout_seconds=0.2)
    consumer.join(1)

    assert removed == [{"kind": "pose", "sequence": 1}]
    assert [pose_q.get_nowait() for _ in range(2)] == [
        {"kind": "pose", "sequence": 2},
        {"kind": "barrier", "token": "freeze",
         "bridge_received_monotonic_ns": 3},
    ]
    assert drops.value == 0


def test_bridge_parent_exposes_pose_commands_stats_and_nonblocking_pop():
    worker = q5_media_bridge.BridgeWorker()
    worker.configure_pose("/nvidia_desktop/camera/rgb/poses")
    worker.request_pose_barrier("freeze-2")

    assert worker._cmd_q.get(timeout=1) == {
        "kind": "pose_config",
        "topic": "/nvidia_desktop/camera/rgb/poses",
    }
    assert worker._cmd_q.get(timeout=1) == {
        "kind": "pose_barrier",
        "token": "freeze-2",
    }
    assert worker.pop_pose() is None
    assert worker.pose_stats() == {"bridge_transport_drops": 0}


def test_main_binds_pose_lane_to_existing_client_lifecycle():
    source = Path(__file__).with_name("main.py").read_text()

    assert "client.configure_pose = bridge.configure_pose" in source
    assert "client.pop_pose = bridge.pop_pose" in source
    assert "client.pose_stats = bridge.pose_stats" in source
    assert "client.request_pose_barrier = bridge.request_pose_barrier" in source
