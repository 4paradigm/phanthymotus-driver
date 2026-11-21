"""Independent leg sensor behavior at the ROS and SDK boundaries."""
import ast
import importlib
import json
import os
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def sensor(monkeypatch):
    class Message:
        def __init__(self, data=''):
            self.data = data

    class Node:
        def __init__(self, name):
            self.name, self.now_ns = name, 1_000_000_000
            self.published = {}

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=self.now_ns))

        def create_publisher(self, kind, topic, qos):
            self.published[topic] = []
            return NS(publish=self.published[topic].append)

        def create_timer(self, period, callback):
            self.tick = callback
            return NS()

    # Deliberately no subscriptions or camera/native-estimator dependencies.
    ros = ModuleType('rclpy')
    nodes = ModuleType('rclpy.node')
    nodes.Node = Node
    qos = ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = object()
    messages = ModuleType('std_msgs.msg')
    messages.String = Message
    for name, value in [('rclpy', ros), ('rclpy.node', nodes),
                        ('rclpy.qos', qos), ('std_msgs.msg', messages)]:
        monkeypatch.setitem(sys.modules, name, value)
    return lambda: (importlib.import_module('noetix.bumi.leg_odometry_sensor'),
                    NS(add_node=lambda node: None))


class Controller:
    def __init__(self):
        self.quaternion = [0., 0., 0., 1.]
        self.error = None
        self.sequence = 0
        self.frozen = False

    def get_imu_data(self):
        if self.error:
            raise self.error
        if not self.frozen:
            self.sequence += 1
        # A stationary body still has newly sampled IMU noise. This varies no
        # kinematic input, but distinguishes an SDK update from a cached getter.
        return NS(ori=self.quaternion, angular_vel=[0., 0., .1],
                  linear_acc=[0., 0., 9.81 + self.sequence * 1e-7])

    def get_joint_state(self):
        return [NS(pos=0.) for _ in range(21)]


def test_source_runs_without_visual_inputs_and_keeps_read_timestamp(sensor, monkeypatch):
    module, executor = sensor()
    plugin = module.LegOdometryPlugin({}, 'robot', executor, Controller())
    plugin.active = True
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    plugin._sample_once()
    clock[0] = 1.025
    plugin.node.now_ns = 1_025_000_123
    plugin._sample_once()
    plugin._publish()
    sample = json.loads(plugin.node.published[plugin.topic][-1].data)
    assert sample['schema'] == 'motus.odom/1'
    assert sample['pose']['position'] == pytest.approx([0., 0., 0.])
    assert sample['twist'] == pytest.approx([0., 0., 0., 0., 0., .1])
    assert sample['stamp_ms'] == 1025
    plugin.node.now_ns = 1_125_000_000
    plugin._publish()
    repeated = json.loads(plugin.node.published[plugin.topic][-1].data)
    assert repeated['stamp_ms'] == sample['stamp_ms']
    assert repeated['pose'] == sample['pose']
    plugin.node.now_ns = 1_300_000_000
    plugin._publish()
    stale = json.loads(plugin.node.published[plugin.topic][-1].data)
    assert stale['pose'] is None
    assert stale['twist'] == [None] * 6
    info = plugin.dispatch('info', {})
    assert 'odom_interface' not in info
    assert {'topic': plugin.topic, 'format': 'state/odom'} in info['topic_out']
    assert any(p['topic'] == plugin.topic and p['format'] == 'state/odom'
               for p in info['input_metadata'])


def test_sdk_failure_invalidates_pose_instead_of_refreshing_it(sensor, monkeypatch):
    module, executor = sensor()
    controller = Controller()
    plugin = module.LegOdometryPlugin({}, 'robot', executor, controller)
    plugin.active = True
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    plugin._sample_once()
    clock[0] = 1.025
    plugin.node.now_ns = 1_025_000_000
    plugin._sample_once()
    assert plugin.info()['estimate']['pose'] is not None
    controller.error = RuntimeError('SDK read failed')
    plugin.node.now_ns = 1_050_000_000
    plugin._sample_once()
    sample = plugin.info()['estimate']
    assert sample['pose'] is None
    assert sample['twist'] == [None] * 6
    assert 'SDK read failed' in plugin.info()['last_error']


def test_cached_sdk_values_expire_and_recovery_rebuilds_the_leg_baseline(sensor, monkeypatch):
    module, executor = sensor()
    controller = Controller()
    plugin = module.LegOdometryPlugin({}, 'robot', executor, controller)
    plugin.active = True
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    plugin._sample_once()
    clock[0], plugin.node.now_ns = 1.025, 1_025_000_000
    plugin._sample_once()
    first = plugin.info()['estimate']
    controller.frozen = True
    for tick in range(2, 15):
        clock[0], plugin.node.now_ns = 1. + tick * .025, 1_000_000_000 + tick * 25_000_000
        plugin._sample_once()
    stale = plugin.info()['estimate']
    assert stale['pose'] is None
    assert stale['twist'] == [None] * 6
    assert stale['stamp_ms'] == first['stamp_ms']
    controller.frozen = False
    clock[0], plugin.node.now_ns = 1.375, 1_375_000_000
    plugin._sample_once()
    assert plugin.info()['estimate']['pose'] is None
    clock[0], plugin.node.now_ns = 1.4, 1_400_000_000
    plugin._sample_once()
    recovered = plugin.info()['estimate']
    assert recovered['pose'] is not None
    assert recovered['pose']['frame_id'] != first['pose']['frame_id']


def test_stop_does_not_allow_a_blocked_worker_to_revive(sensor, monkeypatch):
    module, executor = sensor()
    entered, release = threading.Event(), threading.Event()

    class BlockingController(Controller):
        def get_imu_data(self):
            entered.set()
            release.wait(2.)
            return super().get_imu_data()

    monkeypatch.setattr(module, 'STOP_TIMEOUT_S', .01)
    plugin = module.LegOdometryPlugin({}, 'robot', executor, BlockingController())
    assert plugin.start() is True
    assert entered.wait(1.)
    worker = plugin.thread
    try:
        plugin.stop()
        assert plugin.active is False
        assert plugin.start() is False
        count = len(plugin.node.published[plugin.topic])
        release.set()
        worker.join(1.)
        assert not worker.is_alive()
        assert len(plugin.node.published[plugin.topic]) == count
        assert plugin.info()['estimate']['pose'] is None
        assert plugin.start() is True
    finally:
        release.set()
        plugin.stop()


def test_bundle_loads_leg_sensor_with_no_camera_or_visual_card(sensor, monkeypatch):
    module, executor = sensor()
    monkeypatch.setitem(sys.modules, 'leg_odometry_sensor', module)
    tree = ast.parse((ROOT / 'noetix/bumi/main.py').read_text())
    bundle_class = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef) and node.name == 'BumiDeviceBundle')
    scope = {'os': os}
    exec(compile(ast.Module(body=[bundle_class], type_ignores=[]), 'main.py', 'exec'), scope)
    bundle = scope['BumiDeviceBundle'](
        {'plugins': {'leg_odometry': {'enabled': True}}}, 'robot', executor, Controller(), None)
    assert [tool['name'] for tool in bundle.get_all_tools()] == ['leg_odometry']
    assert bundle.dispatch('leg_odometry', {'action': 'info'})['state'] == 'idle'
