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


@pytest.fixture
def stream(sensor, monkeypatch):
    module, executor = sensor()
    controller = Controller()
    plugin = module.LegOdometryPlugin({}, 'robot', executor, controller)
    plugin.active = True
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    def read(ms, monotonic_s=None):
        plugin.node.now_ns = round(ms * 1_000_000)
        clock[0] = ms / 1000 if monotonic_s is None else monotonic_s
        plugin._sample_once()

    def publish():
        plugin.node.tick()
        return json.loads(plugin.node.published[plugin.topic][-1].data)

    return NS(plugin=plugin, controller=controller, read=read, publish=publish)


def test_sdk_recovery_requires_a_new_read_even_after_a_fault(stream):
    stream.read(1000)
    stream.read(1025)
    assert stream.publish()['pose'] is not None
    stream.controller.error = RuntimeError('sample unavailable')
    stream.read(1050)
    assert stream.publish()['twist'] == [None] * 6
    stream.controller.error = None
    stream.controller.frozen = True
    stream.read(1075)
    cached = stream.publish()
    assert cached['stamp_ms'] == 1025
    assert cached['twist'] == [None] * 6
    assert not cached.get('reset')
    stream.controller.frozen = False
    stream.read(1100)
    measured = stream.publish()
    assert measured['stamp_ms'] == 1100
    assert measured['pose'] is None
    assert measured['twist'] == [None, None, None, 0., 0., .1]
    assert measured.get('reset') is True


def test_gap_reset_reaches_first_measured_publication_when_timer_skips_baseline(stream):
    stream.read(1000)
    stream.read(1025)
    first = stream.publish()
    assert first.get('reset') is True
    stream.read(1200)
    stream.plugin.node.now_ns += 250_000_001
    stale = stream.publish()
    assert stale['twist'] == [None] * 6
    assert not stale.get('reset')
    stream.read(1225)
    for _ in range(2):
        assert stream.plugin.info()['estimate']['pose'] is not None
    measured = stream.publish()
    assert measured['pose']['frame_id'] != first['pose']['frame_id']
    assert measured.get('reset') is True
    assert stream.publish() == measured
    stream.read(1250)
    assert not stream.publish().get('reset')


def test_rollback_reset_waits_for_a_strictly_newer_wire_measurement(stream):
    stream.read(1000)
    stream.read(1025)
    assert stream.publish()['stamp_ms'] == 1025
    stream.read(1075)  # Observed, but never published.
    for ms in (1015, 1025, 1025.5):
        stream.read(ms)
        withheld = stream.publish()
        assert withheld['pose'] is None and withheld['twist'] == [None] * 6
        assert not withheld.get('reset')
    stream.read(1050)
    recovered = stream.publish()
    assert recovered['pose'] is not None
    assert recovered['stamp_ms'] == 1050
    assert recovered.get('reset') is True
    assert stream.publish() == recovered


def test_foot_rebase_keeps_uninterrupted_gyro_on_wire(stream, monkeypatch):
    from noetix.bumi.leg_odometry import JOINT_INDEX

    joints = [0.] * 21
    monkeypatch.setattr(stream.controller, 'get_joint_state', lambda: joints)
    stream.read(1000)
    stream.read(1025)
    first = stream.publish()
    for side in ('l', 'r'):
        joints[JOINT_INDEX[f'{side}_leg_pitch_joint']] = .6
    for ms in (1050, 1075, 1100, 1125, 1150):
        stream.read(ms)
        partial = stream.publish()
        assert partial['twist'] == [None, None, None, 0., 0., .1]
        assert partial['pose'] is None
        assert not partial.get('reset')
    stream.read(1175)
    after = stream.publish()
    assert after['pose']['frame_id'] != first['pose']['frame_id']
    assert not after.get('reset')


def test_explicit_restart_accepts_the_first_read_and_marks_a_new_epoch(stream, monkeypatch):
    stream.read(1000)
    stream.read(1025)
    assert stream.publish()['pose'] is not None
    stream.plugin.stop()
    stream.controller.frozen = True
    # Drive acquisition by hand so timer/worker ordering is deterministic.
    monkeypatch.setattr(stream.plugin, '_sample_loop', lambda event: None)
    assert stream.plugin.start()
    try:
        stream.read(1050)
        resumed = stream.publish()
        assert resumed['stamp_ms'] == 1050
        assert resumed['twist'] == [None, None, None, 0., 0., .1]
        assert resumed.get('reset') is True
    finally:
        stream.plugin.stop()


@pytest.mark.parametrize(('stamp_ms', 'monotonic_s', 'reset'), [
    (1125, 1.125, False),                 # Exactly 100 ms on both clocks
    (1125.000001, 1.125, True),           # One nanosecond over on ROS time
    (1125, 1.125000001, True),            # One nanosecond over on monotonic time
    (1050, 1.025, True),                  # Monotonic time stopped
])
def test_acquisition_reset_uses_the_bound_on_either_clock(stream, stamp_ms, monotonic_s, reset):
    stream.read(1000)
    stream.read(1025)
    stream.publish()
    stream.read(stamp_ms, monotonic_s)
    sample = stream.publish()
    assert sample['twist'][3:] == [0., 0., .1]
    assert (sample.get('reset') is True) is reset


def test_failed_odometry_publish_leaves_epoch_pending(stream, monkeypatch):
    stream.read(1000)
    stream.publish()
    stream.read(1200)

    def fail(message):
        raise RuntimeError('publish failed')

    with monkeypatch.context() as patch:
        patch.setattr(stream.plugin.pub, 'publish', fail)
        with pytest.raises(RuntimeError, match='publish failed'):
            stream.publish()
    stream.read(1225)
    recovered = stream.publish()
    assert recovered['pose'] is not None
    assert recovered.get('reset') is True
