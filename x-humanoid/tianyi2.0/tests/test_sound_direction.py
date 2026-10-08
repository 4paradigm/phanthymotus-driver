"""The wake-word angle must reach Agent Core without becoming a motion command."""

import json
import sys
import threading
import time
import types
from pathlib import Path


class _Node:
    def __init__(self, *args, **kwargs):
        self.subscriptions = []
        self.publishers = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback))
        return self.subscriptions[-1]

    def create_publisher(self, msg_type, topic, qos):
        publisher = types.SimpleNamespace(topic=topic, messages=[])
        publisher.publish = publisher.messages.append
        self.publishers.append(publisher)
        return publisher


def _plugin(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")
    rclpy.node.Node = _Node

    class _Qos:
        def __init__(self, **kwargs):
            pass

    class _Policy:
        BEST_EFFORT = RELIABLE = KEEP_LAST = VOLATILE = 0

    rclpy.qos.QoSProfile = _Qos
    rclpy.qos.ReliabilityPolicy = _Policy
    rclpy.qos.HistoryPolicy = _Policy
    rclpy.qos.DurabilityPolicy = _Policy
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg.String = type("String", (), {})
    std_msgs.msg.Bool = type("Bool", (), {})
    std_msgs.msg.UInt32MultiArray = type("UInt32MultiArray", (), {})
    lyre_msgs = types.ModuleType("lyre_msgs")
    lyre_msgs.msg = types.ModuleType("lyre_msgs.msg")
    lyre_msgs.msg.AsrKeyword = type("AsrKeyword", (), {})
    for name, module in (("rclpy", rclpy), ("rclpy.node", rclpy.node),
                         ("rclpy.qos", rclpy.qos), ("std_msgs", std_msgs),
                         ("std_msgs.msg", std_msgs.msg), ("lyre_msgs", lyre_msgs),
                         ("lyre_msgs.msg", lyre_msgs.msg)):
        monkeypatch.setitem(sys.modules, name, module)
    source = (Path(__file__).parents[1] / "device.py").read_text(encoding="utf-8")
    module = types.ModuleType("tianyi_device_sound_direction_test")
    exec(compile(source, "device.py", "exec"), module.__dict__)
    ros2 = types.SimpleNamespace(ctx_tianyi=object(), ctx_core=object(),
                                 executor_tianyi=types.SimpleNamespace(add_node=lambda node: None),
                                 executor_core=types.SimpleNamespace(add_node=lambda node: None))
    return module.SoundDirectionPlugin({}, "robot", ros2)


def test_wake_angle_is_published_and_queryable(monkeypatch):
    plugin = _plugin(monkeypatch)
    tool = plugin.get_tool()
    assert tool["name"] == "sound_direction"
    assert tool["default_action"] == "info"
    assert plugin.dispatch("info", {})["state"] == "idle"
    plugin.start()
    plugin.start()
    assert plugin.dispatch("info", {})["state"] == "no_event"
    assert len(plugin._sub_node.subscriptions) == 1
    msg_type, topic, callback = plugin._sub_node.subscriptions[0]
    assert msg_type.__name__ == "AsrKeyword"
    assert topic == "/audio_asr/keyword"
    callback(types.SimpleNamespace(keyword="小范小范", angle=42))
    payload = json.loads(plugin._pub.messages[-1].data)
    assert payload["angle"] == 42
    assert payload["keyword"] == "小范小范"
    assert isinstance(payload["timestamp_ms"], int)
    result = plugin.dispatch("info", {})
    assert result["state"] == "fresh"
    assert result["angle"] == 42
    assert result["age_ms"] >= 0


def test_dispatch_start_stop_report_lifecycle_state(monkeypatch):
    plugin = _plugin(monkeypatch)
    assert plugin.dispatch("start", {})["state"] == "running"
    assert plugin.dispatch("info", {})["state"] == "no_event"
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["state"] == "running"


def test_old_or_stopped_direction_is_not_current(monkeypatch):
    plugin = _plugin(monkeypatch)
    plugin.start()
    callback = plugin._sub_node.subscriptions[0][2]
    callback(types.SimpleNamespace(keyword="小范小范", angle=-25))
    plugin._last_seen_monotonic = time.monotonic() - 11
    assert plugin.dispatch("info", {})["state"] == "stale"
    assert "angle" not in plugin.dispatch("info", {})
    count = len(plugin._pub.messages)
    plugin.stop()
    callback(types.SimpleNamespace(keyword="小范小范", angle=90))
    assert len(plugin._pub.messages) == count
    plugin.start()
    assert plugin.dispatch("info", {})["state"] == "no_event"
    assert len(plugin._sub_node.subscriptions) == 1


def test_stop_during_wake_callback_cannot_restore_old_direction(monkeypatch):
    plugin = _plugin(monkeypatch)
    plugin.start()
    callback = plugin._sub_node.subscriptions[0][2]
    entered = threading.Event()
    release = threading.Event()
    stopped = threading.Event()
    real_time = time.time

    def paused_time():
        entered.set()
        assert release.wait(2)
        return real_time()

    monkeypatch.setitem(plugin._on_keyword.__globals__, "time",
                        types.SimpleNamespace(time=paused_time, monotonic=time.monotonic))
    reader = threading.Thread(target=callback,
                              args=(types.SimpleNamespace(keyword="小范小范", angle=15),))
    reader.start()
    assert entered.wait(2)
    stopper = threading.Thread(target=lambda: (plugin.stop(), stopped.set()))
    stopper.start()
    stopped.wait(0.2)
    release.set()
    reader.join(2)
    stopper.join(2)
    assert not reader.is_alive() and not stopper.is_alive()
    plugin.start()
    assert plugin.dispatch("info", {})["state"] == "no_event"
