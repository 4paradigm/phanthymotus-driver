"""ASR text and wake-word direction share one card without changing text output."""

import json
import sys
import threading
import time
import types
from pathlib import Path


class _Node:
    def __init__(self, *args, **kwargs):
        self.context = kwargs.get("context")
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

    def destroy_node(self):
        pass


def _plugin(monkeypatch, before_create=None):
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
    lyre_msgs.msg.AsrIat = type("AsrIat", (), {})
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
    if before_create:
        before_create(ros2, rclpy)
    return module.AsrPlugin({}, "robot", ros2)


def test_wake_angle_is_published_and_queryable(monkeypatch):
    plugin = _plugin(monkeypatch)
    tool = plugin.get_tool()
    assert tool["name"] == "asr"
    assert [item["topic"] for item in tool["topic_out"]] == [
        "/robot/asr/text", "/robot/asr/sound_direction"]
    assert plugin.dispatch("info", {})["state"] == "idle"
    plugin.start()
    plugin.start()
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "no_event"
    assert len(plugin._sub_node.subscriptions) == 2
    msg_type, topic, callback = next(sub for sub in plugin._sub_node.subscriptions
                                      if sub[1] == "/audio_asr/keyword")
    assert msg_type.__name__ == "AsrKeyword"
    assert topic == "/audio_asr/keyword"
    callback(types.SimpleNamespace(keyword="小范小范", angle=42))
    payload = json.loads(plugin._direction_pub.messages[-1].data)
    assert payload["angle"] == 42
    assert payload["keyword"] == "小范小范"
    assert isinstance(payload["timestamp_ms"], int)
    text_callback = next(sub[2] for sub in plugin._sub_node.subscriptions
                         if sub[1] == "/audio_asr/iat")
    plugin.dispatch("config", {"kws_enabled": False})
    text_callback(types.SimpleNamespace(id="speech-1", text="测试语音"))
    assert json.loads(plugin._pub.messages[-1].data) == {
        "id": "speech-1", "text": "测试语音"}
    result = plugin.dispatch("info", {})
    assert result["state"] == "running"
    assert result["sound_direction"]["state"] == "fresh"
    assert result["sound_direction"]["angle"] == 42
    assert result["sound_direction"]["age_ms"] >= 0


def test_sound_direction_publishes_through_core_bridge(monkeypatch):
    bridged_messages = {}
    bridge = None

    def enable_bridge(ros2, rclpy):
        nonlocal bridge
        rclpy.publisher = types.ModuleType("rclpy.publisher")
        rclpy.serialization = types.ModuleType("rclpy.serialization")
        rclpy.publisher.Publisher = type("Publisher", (), {})
        rclpy.serialization.deserialize_message = lambda *args: None
        rclpy.serialization.serialize_message = lambda *args: b""
        monkeypatch.setitem(sys.modules, "rclpy.publisher", rclpy.publisher)
        monkeypatch.setitem(sys.modules, "rclpy.serialization", rclpy.serialization)
        root = Path(__file__).parents[1]
        publisher_module = types.ModuleType("bridged_publisher")
        source = (root / "bridged_publisher.py").read_text(encoding="utf-8")
        exec(compile(source, "bridged_publisher.py", "exec"), publisher_module.__dict__)
        def create_bridged_publisher(node, msg_type, topic, qos):
            messages = bridged_messages.setdefault(topic, [])
            return types.SimpleNamespace(publish=messages.append)

        publisher_module.create_bridged_publisher = create_bridged_publisher
        monkeypatch.setitem(sys.modules, "bridged_publisher", publisher_module)
        bridge = types.ModuleType("bridge_integration")
        source = (root / "bridge_integration.py").read_text(encoding="utf-8")
        exec(compile(source, "bridge_integration.py", "exec"), bridge.__dict__)
        bridge.enable(ros2.ctx_core)

    try:
        plugin = _plugin(monkeypatch, before_create=enable_bridge)
        assert plugin._pub_node.context is not plugin._sub_node.context
        assert plugin._pub_node.publishers == []
        plugin.start()
        callback = next(sub[2] for sub in plugin._sub_node.subscriptions
                        if sub[1] == "/audio_asr/keyword")
        callback(
            types.SimpleNamespace(keyword="小范小范", angle=42))
        assert json.loads(bridged_messages["/robot/asr/sound_direction"][0].data)["angle"] == 42
    finally:
        if bridge:
            bridge.disable()


def test_dispatch_start_stop_report_lifecycle_state(monkeypatch):
    plugin = _plugin(monkeypatch)
    assert plugin.dispatch("start", {})["state"] == "running"
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "no_event"
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["state"] == "running"
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "no_event"
    assert plugin.dispatch("unsupported", {}) is None


def test_text_asr_still_works_without_keyword_message_type(monkeypatch):
    plugin = _plugin(monkeypatch)
    del sys.modules["lyre_msgs.msg"].AsrKeyword
    plugin.start()
    assert [sub[1] for sub in plugin._sub_node.subscriptions] == ["/audio_asr/iat"]
    plugin.dispatch("config", {"kws_enabled": False})
    plugin._sub_node.subscriptions[0][2](
        types.SimpleNamespace(id="speech-2", text="测试语音"))
    assert json.loads(plugin._pub.messages[-1].data) == {
        "id": "speech-2", "text": "测试语音"}


def test_old_or_stopped_direction_is_not_current(monkeypatch):
    plugin = _plugin(monkeypatch)
    plugin.start()
    callback = next(sub[2] for sub in plugin._sub_node.subscriptions
                    if sub[1] == "/audio_asr/keyword")
    callback(types.SimpleNamespace(keyword="小范小范", angle=-25))
    plugin._last_direction_monotonic = time.monotonic() - 11
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "stale"
    assert "angle" not in plugin.dispatch("info", {})["sound_direction"]
    count = len(plugin._direction_pub.messages)
    plugin.stop()
    callback(types.SimpleNamespace(keyword="小范小范", angle=90))
    assert len(plugin._direction_pub.messages) == count
    plugin.start()
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "no_event"
    assert len(plugin._sub_node.subscriptions) == 2


def test_stop_during_wake_callback_cannot_restore_old_direction(monkeypatch):
    plugin = _plugin(monkeypatch)
    plugin.start()
    callback = next(sub[2] for sub in plugin._sub_node.subscriptions
                    if sub[1] == "/audio_asr/keyword")
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
    assert plugin.dispatch("info", {})["sound_direction"]["state"] == "no_event"
