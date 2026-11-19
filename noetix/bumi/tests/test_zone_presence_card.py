"""Adapter contracts and ROS resource ownership without hardware dependencies."""
import importlib
import json
from pathlib import Path
import sys
import types

import pytest


@pytest.fixture
def runtime(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    nodes = []
    failure = [None]

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(json.loads(message.data))

    class Node:
        def __init__(self, name):
            self.name = name
            self.publishers = []
            self.destroyed = False
            nodes.append(self)

        def create_publisher(self, *args):
            if failure[0] == "publisher":
                raise RuntimeError("publisher failed")
            publisher = Publisher()
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, message_type, topic, callback, qos):
            if failure[0] == "subscription":
                raise RuntimeError("subscription failed")
            self.receive = callback

        def create_timer(self, interval, callback):
            if failure[0] == "timer":
                raise RuntimeError("timer failed")
            self.tick = callback

        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self):
            self.nodes = []

        def add_node(self, node):
            self.nodes.append(node)
            if failure[0] == "executor":
                raise RuntimeError("executor failed")

        def remove_node(self, node):
            if node in self.nodes:
                self.nodes.remove(node)

    modules = {
        "rclpy": {}, "rclpy.node": {"Node": Node},
        "rclpy.qos": {"QoSProfile": lambda **kw: kw,
                      "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT=1)},
        "std_msgs": {}, "std_msgs.msg": {"String": types.SimpleNamespace},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    card = importlib.import_module('zone_presence_card')
    executor = Executor()
    plugin = card.ZonePresencePlugin({}, 'test_robot', executor)
    return plugin, executor, nodes, failure


def test_schema_covers_every_action_and_prefix(runtime):
    plugin, _, _, _ = runtime
    schema = plugin.get_tool()['inputSchema']
    assert plugin.PREFIX == 'zone_presence'
    assert set(schema['x-action-params']) == set(schema['properties']['action']['enum'])
    for action, description in schema['x-action-params'].items():
        assert description['params'] == (['input_topic'] if action == 'start' else [])


def test_info_resolves_ports_before_start_during_run_and_after_stop(runtime):
    plugin, executor, nodes, _ = runtime
    expected = plugin.get_tool()['topic_out']
    before = plugin.dispatch('info', {})
    assert before['topic_out'] == expected and not nodes
    assert all(p['msg_type'] == 'std_msgs/msg/String' for p in expected)
    assert plugin.dispatch('start', {'input_topic': '/camera/objects'})['topic_out'] == expected
    info = plugin.dispatch('info', {})
    assert info['topic_out'] == expected
    assert info['topic_in'][0]['topic'] == '/camera/objects'
    info['topic_out'].clear()
    assert plugin.dispatch('info', {})['topic_out'] == expected
    plugin.dispatch('stop', {})
    assert nodes[0].destroyed and not executor.nodes
    stopped = plugin.dispatch('info', {})
    assert stopped['topic_out'] == expected
    assert stopped['input_topic'] is None and not stopped['running']


def test_lifecycle_is_idempotent_and_can_restart(runtime):
    plugin, executor, nodes, _ = runtime
    plugin.start()
    assert not nodes
    assert plugin.dispatch('start', {})['state'] == 'error'
    for _ in range(2):
        plugin.dispatch('start', {'input_topic': '/vop/objects'})
    assert len(nodes) == len(executor.nodes) == 1
    assert plugin.dispatch('start', {'input_topic': '/other'})['state'] == 'error'
    plugin.stop()
    plugin.stop()
    plugin.dispatch('start', {'input_topic': '/other'})
    assert len(nodes) == 2 and len(executor.nodes) == 1
    plugin.stop()


@pytest.mark.parametrize('stage', ['publisher', 'subscription', 'timer', 'executor'])
def test_partial_start_failure_releases_resources_and_allows_retry(runtime, stage):
    plugin, executor, nodes, failure = runtime
    failure[0] = stage
    with pytest.raises(RuntimeError):
        plugin.dispatch('start', {'input_topic': '/vop/objects'})
    assert nodes[-1].destroyed and not executor.nodes
    assert plugin.node is None and plugin.event_pub is None
    failure[0] = None
    assert plugin.dispatch('start', {'input_topic': '/vop/objects'})['state'] == 'running'
    plugin.stop()


def test_callback_timer_and_configuration(runtime):
    plugin, _, nodes, _ = runtime
    assert plugin.dispatch('config', {'enter_s': .1, 'leave_s': .1})['state'] == 'configured'
    plugin.dispatch('start', {'input_topic': '/vop/objects'})
    assert plugin.dispatch('config', {'confidence': .8})['state'] == 'error'
    node = nodes[0]
    node.receive(types.SimpleNamespace(data='invalid json'))
    node.tick()
    assert node.publishers[0].messages[-1]['state'] == 'unknown'
    assert plugin.dispatch('info', {})['running']
    plugin.stop()
    assert plugin.dispatch('config', {'region': [0, 0, 0, 0]})['state'] == 'error'
    assert plugin.dispatch('config', {'confidence': .8})['state'] == 'configured'
