"""Offline processor tests: no ROS installation, SDK, or robot."""
import ast
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('joint_stats_test', ROOT / 'joint_activity_stats.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
NAME = m.JOINT_NAMES[0]


def packet(q, name=NAME):
    return {'joints': [{'name': name, 'q': q}]}


def test_window_expiry_and_missing():
    now = [0.0]
    w = m.JointWindow(clock=lambda: now[0])
    w.observe(packet(-1))
    now[0] = 2
    w.observe(packet(2))
    j = w.snapshot()['joints'][0]
    assert (j['min'], j['max'], j['range'], j['count']) == (-1, 2, 3, 2)
    assert w.snapshot()['joints'][1]['status'] == 'no_valid_sample'
    now[0] = 10
    assert w.snapshot()['joints'][0]['count'] == 1
    now[0] = 12
    j = w.snapshot()['joints'][0]
    assert j['status'] == 'expired' and j['min'] is None and j['count'] == 0


@pytest.mark.parametrize('q', [float('nan'), float('inf'), -float('inf'), None, True, '1'])
def test_invalid_values(q):
    w = m.JointWindow()
    w.observe(packet(q))
    assert w.snapshot()['joints'][0]['count'] == 0
    assert w.invalid_entries == 1
    json.dumps(w.snapshot(), allow_nan=False)


def test_duplicate_policy_and_bad_messages():
    w = m.JointWindow()
    w.observe(packet(1)); w.observe(packet(1))
    assert w.snapshot()['joints'][0]['count'] == 2
    w.observe({'joints': [packet(2)['joints'][0], packet(3)['joints'][0]]})
    assert w.duplicate_entries == 1 and w.snapshot()['joints'][0]['count'] == 2
    for bad in [None, [], {}, {'joints': None}]:
        w.observe(bad)
    assert w.invalid_messages == 4
    w.observe({'joints': [None, {'name': [], 'q': 1}, {'name': 'unknown', 'q': 1}]})
    assert w.invalid_entries == 3


def test_all_joint_mapping_matches_producer():
    tree = ast.parse((ROOT / 'device.py').read_text(encoding='utf-8'))
    names = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == '_BUMI_JOINT_NAMES' for t in n.targets))
    assert list(m.JOINT_NAMES) == names and len(names) == 21
    w = m.JointWindow()
    w.observe({'joints': [{'name': n, 'q': i/10} for i, n in enumerate(names)]})
    assert w.snapshot()['status'] == 'valid'


@pytest.fixture
def runtime(monkeypatch):
    class Node:
        fail = False
        def __init__(self, name):
            self.destroyed = False
            self.messages = []
        def create_publisher(self, *args):
            return types.SimpleNamespace(publish=lambda msg: self.messages.append(json.loads(msg.data)))
        def create_subscription(self, *args):
            self.receive = args[2]
            if self.fail:
                raise RuntimeError('subscription failure')
        def create_timer(self, period, callback):
            self.tick = callback
        def destroy_node(self):
            self.destroyed = True
    class Executor:
        def __init__(self): self.nodes = []
        def add_node(self, node): self.nodes.append(node)
        def remove_node(self, node):
            if node in self.nodes: self.nodes.remove(node)
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **kw: kw
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    modules = {'rclpy': types.ModuleType('rclpy'), 'rclpy.qos': qos,
               'rclpy.node': types.SimpleNamespace(Node=Node),
               'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': types.SimpleNamespace(String=type('String', (), {}))}
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    executor = Executor()
    return m.JointActivityStatsPlugin({}, 'robot', executor), executor, Node


def test_lifecycle_binding_cleanup_and_late_callbacks(runtime):
    p, executor, _ = runtime
    assert p.start() and executor.nodes == []
    assert p.dispatch('start', {})['state'] == 'error'
    assert p.dispatch('start', {'input_topics': ['/custom']})['state'] == 'running'
    old = p.node
    old.receive(types.SimpleNamespace(data=json.dumps(packet(1))))
    old.tick()
    assert old.messages[-1]['joints'][0]['count'] == 1
    assert p.dispatch('start', {'input_topic': '/other'})['state'] == 'error'
    p.stop(); p.stop()
    assert old.destroyed and not executor.nodes
    old.receive(types.SimpleNamespace(data=json.dumps(packet(2))))
    old.tick()
    assert p.window.received == 0 and len(old.messages) == 1
    p.dispatch('start', {'input_topic': '/new'})
    assert p.node is not old
    p.stop()


def test_failed_start_cleanup(runtime):
    p, executor, Node = runtime
    Node.fail = True
    assert p.dispatch('start', {'input_topic': '/x'})['state'] == 'error'
    assert not executor.nodes and p.node is None


def test_concurrent_receive_info_and_stop(runtime):
    p, executor, _ = runtime
    p.dispatch('start', {'input_topic': '/x'})
    node = p.node
    errors = []
    def run():
        try:
            for _ in range(100):
                node.receive(types.SimpleNamespace(data=json.dumps(packet(1))))
                node.tick()
                p.dispatch('info', {})
        except Exception as exc: errors.append(exc)
    threads = [threading.Thread(target=run) for _ in range(3)]
    for t in threads: t.start()
    p.stop()
    for t in threads: t.join(5); assert not t.is_alive()
    assert not errors and not executor.nodes and node.destroyed


def test_manifest_and_packaging():
    manifest = yaml.safe_load((ROOT / 'driver.yaml').read_text(encoding='utf-8'))
    names = [c['name'] for c in manifest['cards']]
    assert len(names) == len(set(names))
    assert next(c for c in manifest['cards'] if c['name'] == m.JointActivityStatsPlugin.PREFIX)['type'] == 'processor'
    assert 'COPY joint_activity_stats.py /work/joint_activity_stats.py' in (ROOT / 'Dockerfile').read_text(encoding='utf-8')


@pytest.mark.parametrize('value', [0, True, float('nan'), 301])
def test_config_validation(value):
    with pytest.raises(ValueError): m.JointWindow(value)


def test_bundle_registration_without_controllers(runtime, monkeypatch):
    p, executor, _ = runtime
    monkeypatch.setitem(sys.modules, 'rclpy.executors', types.ModuleType('rclpy.executors'))
    sys.modules['rclpy'].executors = sys.modules['rclpy.executors']
    monkeypatch.setitem(sys.modules, 'joint_activity_stats', m)
    spec = importlib.util.spec_from_file_location('stats_bundle_test', ROOT / 'main.py')
    bundle_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle_module)
    bundle = bundle_module.BumiDeviceBundle({'plugins': {'joint_activity_stats': {'enabled': True}}}, 'robot', executor, None, None)
    tools = bundle.get_all_tools()
    assert len(tools) == 1 and tools[0]['name'] == p.PREFIX
    assert tools[0]['topic_in'] == [{'format': 'sensor/skeleton'}]
    assert tools[0]['topic_out'][0]['format'] == 'data/json'
    bundle.start_all()
    assert not executor.nodes
    assert bundle.dispatch(p.PREFIX, {'action': 'start', 'input_topic': '/robot/state/joints'})['state'] == 'running'
    bundle.stop_all()
    assert not executor.nodes


def test_concurrent_start_stop(runtime):
    p, executor, _ = runtime
    errors = []
    def run():
        try:
            for _ in range(50):
                p.dispatch('start', {'input_topic': '/x'})
                p.stop()
        except Exception as exc: errors.append(exc)
    threads = [threading.Thread(target=run) for _ in range(4)]
    for t in threads: t.start()
    for t in threads: t.join(5); assert not t.is_alive()
    p.stop()
    assert not errors and not executor.nodes


def test_capacity_and_numeric_overflow_are_explicit():
    w = m.JointWindow(clock=lambda: 0)
    for i in range(30001): w.observe(packet(i))
    report = w.snapshot()
    assert report['capacity_dropped_samples'] == 1 and not report['window_complete']
    assert report['joints'][0]['count'] == 30000
    w = m.JointWindow()
    w.observe(packet(-1e308)); w.observe(packet(1e308))
    assert w.snapshot()['joints'][0]['range'] is None
    assert not w.snapshot()['joints'][0]['range_valid']
    json.dumps(w.snapshot(), allow_nan=False)


def test_core_dual_argument_multi_input_is_rejected(runtime):
    p, executor, _ = runtime
    for args in [dict(input_topic='/a', input_topics=['/a', '/b']),
                 dict(input_topic='/a', input_topics=['/b'])]:
        assert p.dispatch('start', args)['state'] == 'error'
    assert not executor.nodes
    assert p.dispatch('start', dict(input_topic='/a', input_topics=['/a']))['state'] == 'running'
    assert p.dispatch('info', {})['topic_in'] == [{'topic': '/a', 'format': 'sensor/skeleton'}]
    p.stop()


def test_timer_expires_samples_without_more_messages(runtime):
    p, executor, _ = runtime
    now = [0.0]
    p.clock = lambda: now[0]
    p.window = m.JointWindow(clock=p.clock)
    p.dispatch('start', {'input_topic': '/x'})
    p.node.receive(types.SimpleNamespace(data=json.dumps(packet(1))))
    now[0] = 10
    p.node.tick()
    report = p.node.messages[-1]
    assert report['status'] == 'no_valid_samples'
    assert report['joints'][0]['status'] == 'expired'
    assert report['joints'][0]['count'] == 0
    p.stop()


def test_nonvalid_range_cannot_report_valid():
    w = m.JointWindow()
    w.observe({'joints': [{'name': n, 'q': -1e308} for n in m.JOINT_NAMES]})
    w.observe(packet(1e308))
    assert w.snapshot()['joints'][0]['status'] == 'invalid_range'
    assert w.snapshot()['status'] != 'valid'
    w.capacity_drops = 1
    assert w.snapshot()['joints'][1]['status'] == 'incomplete'
    assert w.snapshot()['status'] != 'valid'


def test_executor_refusal_cleans_node(runtime, monkeypatch):
    p, executor, Node = runtime
    nodes = []
    p.node_factory = lambda name: nodes.append(Node(name)) or nodes[-1]
    monkeypatch.setattr(executor, 'add_node', lambda node: False)
    assert p.dispatch('start', {'input_topic': '/x'})['state'] == 'error'
    assert nodes[0].destroyed and p.node is None
