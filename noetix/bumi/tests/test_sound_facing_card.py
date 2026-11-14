"""Card lifecycle, schema and packaging, using an isolated fake ROS executor."""

import json
import sys
import types
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sound_facing import FacingConfig
from sound_facing_card import SoundFacingPlugin
from test_sound_facing import Motion
from sound_authorization_fixtures import approved_authority


class Node:
    created = []

    def __init__(self, name):
        self.subscriptions, self.timers, self.messages = [], [], []
        self.destroyed = False
        self.created.append(self)

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append((topic, callback))

    def create_publisher(self, kind, topic, qos):
        return types.SimpleNamespace(publish=self.messages.append)

    def create_timer(self, interval, callback):
        self.timers.append(callback)

    def destroy_node(self):
        self.destroyed = True
        self.subscriptions.clear()
        self.timers.clear()


@pytest.fixture
def ros(monkeypatch):
    Node.created = []
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg")}
    modules["rclpy.node"].Node = Node
    modules["rclpy.qos"].qos_profile_sensor_data = object()
    modules["std_msgs.msg"].String = lambda **kwargs: types.SimpleNamespace(**kwargs)
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    nodes = []
    return types.SimpleNamespace(add_node=nodes.append, remove_node=nodes.remove, nodes=nodes)


def test_start_registration_restart_and_stop_release_all_resources(ros):
    motion = Motion()
    card = SoundFacingPlugin({}, "sim", ros, motion)
    for _ in range(3):
        assert card.start() and card.start()
        assert len(ros.nodes) == 1
        assert card._engine.snapshot()["enabled"] is False
        card._publish()
        assert json.loads(card._node.messages[-1].data)["enabled"] is False
        assert not motion.calls
        node, worker = card._node, card._worker
        card.stop()
        assert not ros.nodes and node.destroyed and not worker.is_alive()
        assert not card.stop()["enabled"]


@pytest.mark.parametrize("args", [{"owner": "x"}, {"session_id": "stale"}, {"session_timeout_s": 60}])
def test_enable_rejects_parameters_not_declared_for_action(ros, args):
    card = SoundFacingPlugin({}, "sim", ros, Motion())
    assert card.dispatch("enable_follow", args)["error"] == "unknown_parameters"
    assert card._engine.session_id is None


def test_malformed_messages_and_source_binding(ros):
    card = SoundFacingPlugin({}, "sim", ros, Motion())
    assert card.dispatch("start", {"input_topic": "/other/sound_direction"})["state"] == "error"
    card.start()
    for text in ("{", "null", "[]", "true", "x" * 4097):
        card._on_direction(types.SimpleNamespace(data=text))
    assert card._engine.state == "idle"
    card.stop()


def test_failed_subscription_start_does_not_leak_node(ros, monkeypatch):
    monkeypatch.setattr(Node, "create_subscription", lambda *args: (_ for _ in ()).throw(RuntimeError()))
    card = SoundFacingPlugin({}, "sim", ros, Motion())
    # remove_node may legitimately be called before add_node on failure.
    ros.remove_node = lambda node: ros.nodes.remove(node) if node in ros.nodes else None
    with pytest.raises(RuntimeError):
        card.start()
    assert Node.created[-1].destroyed and not ros.nodes
    assert not card._engine.running


def test_config_manifest_docker_and_schema_agree():
    bumi = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((bumi / "config.yaml").read_text())["plugins"]["bumi_sound_facing"]
    assert config.pop("enabled") is False
    assert config == asdict(FacingConfig())
    card = SoundFacingPlugin({}, "sim", None, None)
    tool = card.get_tool()
    assert tool["name"] == "bumi_sound_facing"
    for name, value in config.items():
        assert tool["configSchema"]["properties"][name]["default"] == value
    manifest = yaml.safe_load((bumi / "driver.yaml").read_text())
    assert {"name": tool["name"], "type": tool["type"]} in manifest["cards"]
    docker = (bumi / "Dockerfile").read_text()
    assert "COPY sound_facing.py sound_facing_card.py sound_facing_authority.py /work/" in docker
    assert "from sound_facing_card import SoundFacingPlugin" in docker
    assert tool["topic_in"][0]["format"] == "sensor/sound-direction"
    assert "x-completion" not in tool["inputSchema"]
    assert set(tool["inputSchema"]["x-action-params"]) == set(tool["inputSchema"]["properties"]["action"]["enum"])


def test_config_does_not_accept_persisted_auto_enable():
    with pytest.raises(ValueError):
        SoundFacingPlugin({"enabled": True, "auto_enable": True}, "sim", None, None)


def test_actual_core_trace_and_single_card_instance_metadata(ros):
    source = types.SimpleNamespace(dispatch=lambda *args: {
        "state": "running", "calibrated_directions": ["front", "right"]})
    card = SoundFacingPlugin({}, "sim", ros, Motion(), source_plugin=source,
                             authority=approved_authority())
    card.start()
    result = card.dispatch("enable_follow", {"_trace_id": "trace", "instance_id": "canvas-card"})
    assert result["enabled"] and result["session_id"]
    assert not card.dispatch("disable_follow", {"instance_id": "canvas-card"})["enabled"]
    card.stop()


def test_stop_cancels_only_the_injected_owned_motion(ros):
    motion = Motion()
    source = types.SimpleNamespace(dispatch=lambda *args: {"state": "running", "calibrated_directions": ["front", "right"]})
    card = SoundFacingPlugin({}, "sim", ros, motion, source_plugin=source,
                             authority=approved_authority())
    card.start()
    card._engine.enable()
    assert motion.owner is card._engine.owner
    assert card.stop()["state"] == "idle"
    assert motion.owner is None


def test_calibration_or_worker_loss_revokes_before_motion(ros):
    source = types.SimpleNamespace(dispatch=lambda *args: {
        "state": "running", "calibrated_directions": ["front", "right"]})
    motion = Motion()
    card = SoundFacingPlugin({}, "sim", ros, motion, source_plugin=source,
                             authority=approved_authority())
    card._engine.start()
    assert card._engine.enable()["enabled"] is True
    source.dispatch = lambda *args: {"state": "unavailable", "calibrated_directions": []}
    card._engine.tick()
    assert card._engine.reason == "sound_direction_unavailable"
    assert card._engine.session_id is None and motion.owner is None and not motion.calls


def test_missing_direction_plugin_refuses_enable(ros):
    card = SoundFacingPlugin({}, "sim", ros, Motion(), authority=approved_authority())
    card._engine.start()
    assert card._engine.enable()["reason"] == "sound_direction_unavailable"
