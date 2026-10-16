"""Bumi squat counter: complete cycles, track isolation and MCP event output."""

import importlib.util
import json
import sys
import types
from pathlib import Path
from unittest import mock

DRIVER = Path(__file__).resolve().parents[1] / "noetix" / "bumi"
SPEC = importlib.util.spec_from_file_location("bumi_squat_count", DRIVER / "squat_count.py")
count_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(count_module)
SquatCounter = count_module.SquatCounter


def person(track=1, posture="standing", confidence=0.9):
    return {"id": track, "posture": posture, "posture_confidence": confidence}


def feed(counter, frames, start=0.0, dt=0.1):
    return [event for i, frame in enumerate(frames)
            if (event := counter.update(frame, start + i * dt)) is not None]


def cycle(track=1):
    return ([[person(track, "standing")]] * 4
            + [[person(track, "crouching")]] * 4
            + [[person(track, "standing")]] * 4)


def test_complete_cycles_are_counted_once_each():
    counter = SquatCounter()
    events = feed(counter, cycle() + cycle(), dt=0.1)
    assert [e["count"] for e in events] == [1, 2]
    assert all(e["priority"] == 1 and e["track_id"] == 1 for e in events)


def test_short_crouch_and_low_confidence_do_not_count():
    counter = SquatCounter()
    frames = ([[person()]] * 4 + [[person(posture="crouching")]]
              + [[person()]] * 4 + [[person(posture="crouching", confidence=0.1)]] * 4
              + [[person()]] * 4)
    assert feed(counter, frames) == []


def test_other_person_and_occlusion_cannot_complete_repetition():
    counter = SquatCounter(lost_s=0.5)
    frames = ([[person(1)]] * 4 + [[person(1, "crouching")]] * 4
              + [[person(2)]] * 4 + [[]] * 7 + [[person(1)]] * 4)
    assert feed(counter, frames) == []
    assert counter.count == 0


def test_sitting_or_long_unknown_period_cancels_partial_repetition():
    counter = SquatCounter(lost_s=0.5)
    frames = ([[person()]] * 4 + [[person(posture="crouching")]] * 4
              + [[person(posture="sitting")]] * 3 + [[person()]] * 4)
    assert feed(counter, frames) == []
    counter.reset()
    frames = ([[person()]] * 4 + [[person(posture="crouching")]] * 4
              + [[person(posture=None)]] * 7 + [[person()]] * 4)
    assert feed(counter, frames) == []


def test_reset_starts_new_set_and_requires_initial_standing():
    counter = SquatCounter()
    assert len(feed(counter, cycle())) == 1
    counter.reset()
    assert feed(counter, [[person(posture="crouching")]] * 4) == []
    assert len(feed(counter, cycle(), start=1.0)) == 1
    assert counter.count == 1


def load_card(monkeypatch):
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = object
    msgs = types.ModuleType("std_msgs.msg")
    msgs.String = type("String", (), {})
    for name, module in (("rclpy", rclpy), ("rclpy.node", node_mod),
                         ("rclpy.qos", qos), ("std_msgs", types.ModuleType("std_msgs")),
                         ("std_msgs.msg", msgs), ("squat_count", count_module)):
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("bumi_squat_card", DRIVER / "squat_counter.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_card_subscribes_to_pose_and_publishes_only_completed_events(monkeypatch):
    module = load_card(monkeypatch)
    node = mock.Mock()
    node.create_publisher.return_value = mock.Mock()
    with mock.patch.object(module, "Node", return_value=node):
        card = module.SquatCounterPlugin({}, "bumi", mock.Mock())
    tool = card.get_tool()
    assert tool["type"] == "processor"
    assert tool["topic_in"][0]["format"] == "data/json"
    assert tool["topic_out"][0]["topic"] == "/bumi/squat/count"

    assert card.dispatch("start", {})["state"] == "error"
    assert card.dispatch("start", {"input_topic": "/bumi/camera/color/poses"})["state"] == "running"
    callback = node.create_subscription.call_args.args[2]
    with mock.patch.object(module.time, "monotonic", return_value=0.0):
        callback(types.SimpleNamespace(data=json.dumps({"persons": person()})))
    # Reject malformed per-frame payloads without announcing a repetition.
    assert node.create_publisher.return_value.publish.call_count == 0
    card.dispatch("reset", {})
    with mock.patch.object(module.time, "monotonic", side_effect=[i * 0.1 for i in range(12)]):
        for frame in cycle():
            callback(types.SimpleNamespace(data=json.dumps({"persons": frame})))
    published = node.create_publisher.return_value.publish.call_args.args[0]
    assert json.loads(published.data)["count"] == 1
    assert node.create_publisher.return_value.publish.call_count == 1
    card.dispatch("stop", {})
    assert card.dispatch("info", {})["count"] == 0
