"""Timer card: scheduling core, plugin contract and bundle wiring."""

import importlib.util
import json
import sys
import time
import types
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).resolve().parent
BUNDLE = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(BUNDLE))

import timer_card


class FakeNode:
    def __init__(self, topic):
        self.topic = topic
        self.payloads = []

    def publish(self, payload):
        self.payloads.append(payload)


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


def _plugin(monkeypatch, interval=0.05):
    monkeypatch.setattr(timer_card, "MIN_INTERVAL_S", 0.01)
    node = FakeNode("/robot/timer/events")
    plugin = timer_card.TimerPlugin(
        {"interval_s": interval}, "robot", FakeExecutor(),
        node_factory=lambda topic: node,
    )
    return plugin, node


def test_core_ticks_then_stops(monkeypatch):
    monkeypatch.setattr(timer_card, "MIN_INTERVAL_S", 0.01)
    seen = []
    core = timer_card.TimerCore(seen.append)
    core.start(0.05, label="sedentary")
    time.sleep(0.24)
    stopped = core.stop()
    count = len(seen)
    assert count >= 2
    assert stopped["state"] == "idle"
    assert all(p["priority"] == 1 for p in seen)
    assert all(p["event"] == "timer_tick" for p in seen)
    assert all(p["label"] == "sedentary" for p in seen)
    assert [p["tick"] for p in seen] == list(range(1, count + 1))
    time.sleep(0.12)
    assert len(seen) == count


def test_core_max_ticks_finishes(monkeypatch):
    monkeypatch.setattr(timer_card, "MIN_INTERVAL_S", 0.01)
    seen = []
    core = timer_card.TimerCore(seen.append)
    core.start(0.02, label="once", max_ticks=2)
    deadline = time.monotonic() + 1.0
    while len(seen) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.1)
    assert len(seen) == 2
    assert core.snapshot()["state"] == "finished"


def test_start_arms_but_does_not_tick(monkeypatch):
    plugin, node = _plugin(monkeypatch)
    started = plugin.dispatch("start", {})
    assert started["state"] == "running"
    assert started["ticking"] is False
    time.sleep(0.12)
    assert node.payloads == []

    begun = plugin.dispatch("begin", {"interval_s": 0.05, "label": "sedentary"})
    assert begun["state"] == "running"
    assert begun["ticking"] is True
    time.sleep(0.12)
    assert len(node.payloads) >= 2
    assert node.payloads[0]["priority"] == 1
    assert node.payloads[0]["label"] == "sedentary"

    plugin.dispatch("stop", {})
    count = len(node.payloads)
    time.sleep(0.12)
    assert len(node.payloads) == count


def test_begin_is_idempotent_for_same_parameters(monkeypatch):
    plugin, node = _plugin(monkeypatch)
    first = plugin.dispatch("begin", {"interval_s": 0.05, "label": "sedentary"})
    timer_id = first["timer_id"]
    assert first["ticking"] is True
    second = plugin.dispatch("begin", {"interval_s": 0.05, "label": "sedentary"})
    assert second["timer_id"] == timer_id
    plugin.dispatch("stop", {})


def test_armed_lifecycle(monkeypatch):
    plugin, _ = _plugin(monkeypatch)
    assert plugin.dispatch("info", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["state"] == "running"
    info = plugin.dispatch("info", {})
    assert info["state"] == "running"
    assert info["armed"] is True
    assert info["ticking"] is False
    halted = plugin.dispatch("halt", {})
    assert halted["state"] == "running"
    assert halted["ticking"] is False
    assert plugin.dispatch("stop", {})["state"] == "idle"
    info = plugin.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["armed"] is False


def test_invalid_interval_is_rejected(monkeypatch):
    plugin, _ = _plugin(monkeypatch)
    result = plugin.dispatch("begin", {"interval_s": 0, "label": "bad"})
    assert result["state"] == "error"
    assert "interval_s" in result["error"]


@pytest.mark.parametrize("value", [1.9, 0.5, True, False, "2", [1], float("nan"), -1, timer_card.MAX_TICKS + 1])
def test_max_ticks_rejects_non_integers_without_starting(value):
    # No sleep: rejection must happen before any thread exists.
    node = FakeNode("/robot/timer/events")
    plugin = timer_card.TimerPlugin({}, "robot", FakeExecutor(),
                                    node_factory=lambda topic: node)
    result = plugin.dispatch("begin", {"interval_s": 60, "max_ticks": value})
    assert result["state"] == "error"
    assert "max_ticks" in result["error"]
    assert plugin.dispatch("info", {})["ticking"] is False
    assert node.payloads == []


def test_max_ticks_rejection_beats_idempotence(monkeypatch):
    plugin, node = _plugin(monkeypatch)
    plugin.dispatch("begin", {"interval_s": 60, "label": "x", "max_ticks": 1})
    result = plugin.dispatch("begin", {"interval_s": 60, "label": "x", "max_ticks": 1.9})
    assert result["state"] == "error"
    plugin.dispatch("stop", {})


@pytest.mark.parametrize("value", [2, 2.0, 0, None])
def test_max_ticks_accepts_integers(value):
    assert timer_card.parse_max_ticks(value) == (int(value) if value is not None else 0)


@pytest.mark.parametrize("value", [True, "abc", float("inf"), 0, 3601])
def test_interval_rejects_bad_values(value):
    with pytest.raises(ValueError):
        timer_card.parse_interval(value)


def test_config_is_accepted_and_sets_default_interval(monkeypatch):
    # Agent Core sends `config` on save, before start and on reconnect; an
    # unhandled action surfaced as `Unknown tool: timer`.
    plugin, node = _plugin(monkeypatch)
    result = plugin.dispatch("config", {"interval_s": 0.05, "input_topic": "ignored"})
    assert result["state"] == "configured"
    assert result["default_interval_s"] == 0.05
    assert plugin.dispatch("config", {})["state"] == "configured"
    assert plugin.get_tool()["inputSchema"]["properties"]["interval_s"]["default"] == 0.05
    assert plugin.dispatch("info", {})["ticking"] is False
    assert node.payloads == []
    begun = plugin.dispatch("begin", {"label": "cfg"})
    assert begun["interval_s"] == 0.05
    plugin.dispatch("stop", {})


def test_config_rejects_bad_interval():
    node = FakeNode("/robot/timer/events")
    plugin = timer_card.TimerPlugin({}, "robot", FakeExecutor(),
                                    node_factory=lambda topic: node)
    result = plugin.dispatch("config", {"interval_s": True})
    assert result["state"] == "error"
    assert plugin.dispatch("config", {"interval_s": 60})["state"] == "configured"


def test_bundle_answers_config_for_timer(monkeypatch):
    from test_mic_card import _load_device
    import importlib
    _load_device(monkeypatch)
    reloaded = importlib.reload(timer_card)
    monkeypatch.setitem(sys.modules, "timer_card", reloaded)
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    spec = importlib.util.spec_from_file_location("bumi_timer_cfg_test", BUNDLE / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bundle = module.BumiDeviceBundle({"plugins": {"timer": {"enabled": True}}}, "robot",
                                     FakeExecutor(), None, object())
    # main.py turns a None dispatch result into JSON-RPC -32601 "Unknown tool".
    assert bundle.dispatch("timer", {"action": "config", "interval_s": 30}) is not None


def test_tool_schema_and_topic():
    node = FakeNode("/robot/timer/events")
    plugin = timer_card.TimerPlugin({}, "robot", FakeExecutor(),
                                    node_factory=lambda topic: node)
    tool = plugin.get_tool()
    assert tool["name"] == "timer"
    assert tool["type"] == "sensor"
    assert set(tool["inputSchema"]["properties"]["action"]["enum"]) == {
        "start", "begin", "halt", "stop", "info"}
    assert tool["topic_out"] == [{"topic": "/robot/timer/events", "format": "data/json"}]
    assert tool["inputSchema"]["x-action-params"]["begin"]["params"] == [
        "interval_s", "label", "max_ticks"]


def test_bundle_registers_timer(monkeypatch):
    from test_mic_card import _load_device

    _load_device(monkeypatch)
    # rclpy was absent when this test module imported timer_card on a laptop;
    # reload it now that the stubs exist so _TimerNode subclasses the stub Node.
    import importlib
    reloaded = importlib.reload(timer_card)
    monkeypatch.setitem(sys.modules, "timer_card", reloaded)
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    path = BUNDLE / "main.py"
    spec = importlib.util.spec_from_file_location("bumi_timer_bundle_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bundle = module.BumiDeviceBundle(
        {"plugins": {"timer": {"enabled": True}}}, "robot",
        FakeExecutor(), None, object())
    assert [tool["name"] for tool in bundle.get_all_tools()] == ["timer"]
    assert bundle.dispatch("timer", {"action": "start"})["state"] == "running"
    assert bundle.dispatch("timer", {"action": "stop"})["state"] == "idle"


def test_manifest_and_config_agree():
    manifest = yaml.safe_load((BUNDLE / "driver.yaml").read_text())
    cards = {card["name"]: card["type"] for card in manifest["cards"]}
    assert cards["timer"] == "sensor"
    config = yaml.safe_load((BUNDLE / "config.yaml").read_text())
    assert config["plugins"]["timer"]["enabled"] is True
