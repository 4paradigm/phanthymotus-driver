"""S10 DDS high-level messages and opt-in boundary, with injected fake ROS.

No ROS participant, socket, robot SDK, publisher or real robot is used here.
"""
from __future__ import annotations

import copy
import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from deep_robotics.s10_pro.motion_dds import (
    AuthorityUnavailable, COMMAND_SPECS, DDSAdapterClosed, DDSCommandAdapter,
    EXPECTED_FIELDS, MessageCodec, MotionDDSDisabled, NavigationModeRequired,
    validate_authority,
)


class Time:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'sec': 'int32', 'nanosec': 'uint32'}

    def __init__(self):
        self.sec = self.nanosec = 0


class MetaType:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'frame_id': 'uint64', 'stamp': 'builtin_interfaces/Time'}

    def __init__(self):
        self.frame_id = 0
        self.stamp = Time()


class StateValue:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'state': 'int32'}

    def __init__(self):
        self.state = 0


class NavValue:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'x_vel': 'float', 'y_vel': 'float', 'yaw_vel': 'float'}

    def __init__(self):
        self.x_vel = self.y_vel = self.yaw_vel = 0.


class MotionState:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'header': 'drdds/MetaType', 'data': 'drdds/MotionStateValue'}

    def __init__(self):
        self.header = MetaType()
        self.data = StateValue()


class NavCmd:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/NavCmdValue'}

    def __init__(self):
        self.header = MetaType()
        self.data = NavValue()


CLASSES = {
    'drdds/msg/MotionState': MotionState, 'drdds/msg/NavCmd': NavCmd,
    'drdds/msg/MetaType': MetaType, 'drdds/msg/MotionStateValue': StateValue,
    'drdds/msg/NavCmdValue': NavValue, 'builtin_interfaces/msg/Time': Time,
}


class Publisher:
    def __init__(self):
        self.frames = []

    def publish(self, message):
        self.frames.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return 2


class Node:
    def __init__(self):
        self.created = []
        self.destroyed = []
        self.publishers = []
        self.on_create = None
        self.graph_calls = []

    def create_publisher(self, cls, topic, qos):
        publisher = Publisher()
        self.created.append((cls, topic, qos))
        self.publishers.append(publisher)
        if self.on_create:
            self.on_create()
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed.append(publisher)

    def get_publishers_info_by_topic(self, topic):
        self.graph_calls.append(('publishers', topic))
        return [SimpleNamespace(topic_type=next(v['type'] for v in COMMAND_SPECS.values() if v['topic'] == topic))]

    def get_subscriptions_info_by_topic(self, topic):
        self.graph_calls.append(('subscribers', topic))
        return [SimpleNamespace(topic_type=next(v['type'] for v in COMMAND_SPECS.values() if v['topic'] == topic))]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


@pytest.fixture
def setup():
    clock, node, loaded = Clock(), Node(), []
    def get_type(name):
        loaded.append(name)
        return CLASSES[name]
    codec = MessageCodec(get_type, clock_ns=lambda: 1_792_000_000_000_123_456)
    adapter = DDSCommandAdapter(node, execution_enabled=True, codec=codec, clock=clock)
    yield clock, node, loaded, codec, adapter
    adapter.close()


def permit(clock, **overrides):
    return {'owned': True, 'fresh': True, 'request_id': 'offline-only',
        'received_monotonic': clock.now, 'expires_monotonic': clock.now + .4,
        'age_s': 0., 'navigation_mode_confirmed': True, **overrides}


def test_import_and_construction_do_not_import_ros_or_load_types(monkeypatch):
    # Use a new import to detect module-level ROS imports even when tests coexist
    # with other modules in the process.
    import builtins
    original = builtins.__import__
    def guard(name, *args, **kwargs):
        if name.startswith(('rclpy', 'drdds', 'rosidl_runtime_py')):
            raise AssertionError('ROS import during construction')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guard)
    path = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro/motion_dds.py'
    spec = importlib.util.spec_from_file_location('s10_lazy_codec_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    codec = module.MessageCodec()
    node = Node()
    adapter = module.DDSCommandAdapter(node, codec=codec)
    assert not node.created and not node.graph_calls
    adapter.close()
    assert not node.destroyed


def test_schema_and_stamp_match_actual_vendored_ros_definitions(setup):
    _, _, loaded, codec, _ = setup
    state = codec.build_state(1)
    velocity = codec.build_velocity(.1, -.2, .3)
    assert isinstance(state, MotionState) and state.data.state == 1
    assert state.header.frame_id == 1 and velocity.header.frame_id == 2
    assert state.header.stamp.sec == 1_792_000_000
    assert state.header.stamp.nanosec == 123_456
    assert (velocity.data.x_vel, velocity.data.y_vel, velocity.data.yaw_vel) == (.1, -.2, .3)
    assert set(loaded) == set(EXPECTED_FIELDS)


@pytest.mark.parametrize('state', [0, 2, 3, 5, -2, 0x1001, 18, True, 1., '1', None])
def test_only_initial_requested_motion_states_are_encoded(setup, state):
    _, node, loaded, codec, _ = setup
    with pytest.raises(ValueError):
        codec.build_state(state)
    assert not node.created and not loaded


@pytest.mark.parametrize('state', [1, 4, 17])
def test_documented_state_value_is_not_reinterpreted(setup, state):
    assert setup[3].build_state(state).data.state == state


def test_explicit_header_zero_and_upper_frame_id_bound_are_preserved(setup):
    codec = setup[3]
    msg = codec.build_state(4, frame_id=0, stamp_ns=0)
    assert msg.header.frame_id == msg.header.stamp.sec == msg.header.stamp.nanosec == 0
    msg = codec.build_state(17, frame_id=2**64 - 1, stamp_ns=(2**31 - 1) * 10**9 + 999_999_999)
    assert msg.header.frame_id == 2**64 - 1
    assert msg.header.stamp.sec == 2**31 - 1 and msg.header.stamp.nanosec == 999_999_999


@pytest.mark.parametrize('field,value', [('frame_id', -1), ('frame_id', True), ('frame_id', 2**64),
    ('frame_id', 1.), ('stamp_ns', -1), ('stamp_ns', True), ('stamp_ns', 2**31 * 10**9), ('stamp_ns', float('nan'))])
def test_invalid_meta_values_are_not_silently_truncated(setup, field, value):
    with pytest.raises(ValueError):
        setup[3].build_state(1, **{field: value})


@pytest.mark.parametrize('key,limit', [('vx', 1.67), ('vy', .4), ('yaw_rate', 1.)])
def test_velocity_has_documented_physical_hard_bounds(setup, key, limit):
    codec = setup[3]
    assert codec.build_velocity(**{key: limit})
    assert codec.build_velocity(**{key: -limit})
    for value in [limit + .01, -limit - .01, True, float('nan'), float('inf'), '0']:
        with pytest.raises(ValueError):
            codec.build_velocity(**{key: value})


@pytest.mark.parametrize('name,changed', [
    ('drdds/msg/MetaType', {'frame_id': 'uint64', 'timestamp': 'builtin_interfaces/Time'}),
    ('drdds/msg/MetaType', {'frame_id': 'uint32', 'stamp': 'builtin_interfaces/Time'}),
    ('builtin_interfaces/msg/Time', {'sec': 'uint32', 'nanosec': 'uint32'}),
    ('drdds/msg/NavCmdValue', {'x_vel': 'double', 'y_vel': 'float', 'yaw_vel': 'float'}),
    ('drdds/msg/MotionStateValue', {'state': 'int32', 'undocumented': 'bool'}),
])
def test_nested_schema_mismatch_fails_before_message_construction(name, changed):
    class Wrong:
        @classmethod
        def get_fields_and_field_types(cls):
            return changed
        def __init__(self):
            raise AssertionError('mismatched type must not be constructed')
    codec = MessageCodec(lambda requested: Wrong if requested == name else CLASSES[requested])
    with pytest.raises(ValueError, match='schema mismatch'):
        if name == 'drdds/msg/NavCmdValue':
            codec.build_velocity()
        else:
            codec.build_state(1)


def test_default_disabled_adapter_cannot_load_types_or_create_publisher(setup):
    clock, node, loaded, codec, _ = setup
    adapter = DDSCommandAdapter(node, codec=codec, clock=clock)
    with pytest.raises(MotionDDSDisabled):
        adapter.publish_state(1, authority=permit(clock))
    with pytest.raises(MotionDDSDisabled):
        adapter.publish_zero_velocity(authority=permit(clock))
    assert not loaded and not node.created


@pytest.mark.parametrize('override', [
    {'owned': False}, {'owned': 1}, {'fresh': False}, {'fresh': 1}, {'request_id': ''},
    {'request_id': 'x' * 129}, {'received_monotonic': 101}, {'received_monotonic': 99},
    {'expires_monotonic': 100}, {'expires_monotonic': 101}, {'age_s': .6}, {'age_s': True},
    {'received_monotonic': float('nan')}, {'expires_monotonic': float('inf')},
    {'received_monotonic': 99.6, 'age_s': 0.},
])
def test_each_send_requires_fresh_current_parent_authority(setup, override):
    clock, node, loaded, _, adapter = setup
    with pytest.raises(AuthorityUnavailable):
        adapter.publish_state(1, authority=permit(clock, **override))
    assert not loaded and not node.created


def test_topic_discovery_is_read_only_and_does_not_prove_authority(setup):
    _, node, loaded, _, adapter = setup
    graph = adapter.graph_snapshot()
    assert graph['state']['publisher_count'] == graph['state']['subscriber_count'] == 1
    assert graph['state']['authority_proven'] is False
    assert {v['topic'] for v in graph.values()} == {'/MOTION_STATE', '/NAV_CMD'}
    assert len(node.graph_calls) == 4 and not node.created and not loaded


def test_discovery_graph_is_bounded(setup):
    _, node, loaded, _, adapter = setup
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(topic_type='drdds/msg/MotionState')] * 129
    with pytest.raises(ValueError, match='bound'):
        adapter.graph_snapshot()
    assert not node.created and not loaded


@pytest.mark.parametrize('method,kwargs', [('publish_velocity', {'vx': .1}), ('publish_zero_velocity', {})])
def test_even_zero_nav_command_needs_confirmed_navigation_mode(setup, method, kwargs):
    clock, node, loaded, _, adapter = setup
    with pytest.raises(NavigationModeRequired):
        getattr(adapter, method)(authority=permit(clock, navigation_mode_confirmed=False), **kwargs)
    assert not loaded and not node.created


def test_success_is_local_sent_not_acceptance_or_physical_completion(setup):
    clock, node, _, _, adapter = setup
    first = adapter.publish_state(1, authority=permit(clock))
    second = adapter.publish_state(17, authority=permit(clock))
    assert len(node.created) == 1 and len(node.publishers[0].frames) == 2
    assert [m.data.state for m in node.publishers[0].frames] == [1, 17]
    assert first['sent'] and first['accepted'] is None
    assert not first['confirmed'] and not first['completed']
    assert first['topic'] == '/MOTION_STATE' and second['frame_id'] > first['frame_id']
    # There is no hidden publish of 17 while issuing stand=1.
    assert node.publishers[0].frames[0].data.state == 1


def test_velocity_and_zero_use_only_navcmd_and_same_publisher(setup):
    clock, node, _, _, adapter = setup
    adapter.publish_velocity(.1, -.05, .2, authority=permit(clock))
    receipt = adapter.publish_zero_velocity(authority=permit(clock))
    assert [topic for _, topic, _ in node.created] == ['/NAV_CMD']
    assert (node.publishers[0].frames[1].data.x_vel, node.publishers[0].frames[1].data.y_vel,
            node.publishers[0].frames[1].data.yaw_vel) == (0., 0., 0.)
    assert receipt['accepted'] is None and not receipt['completed']


def test_cancel_before_build_does_not_create_publisher(setup):
    clock, node, loaded, _, adapter = setup
    cancel = threading.Event(); cancel.set()
    with pytest.raises(InterruptedError):
        adapter.publish_state(1, authority=permit(clock), cancel_event=cancel)
    assert not loaded and not node.created


def test_slow_type_import_cannot_use_an_expired_permit(setup):
    clock, node, _, _, _ = setup
    def slow_get(name):
        clock.now += .2
        return CLASSES[name]
    adapter = DDSCommandAdapter(node, execution_enabled=True, codec=MessageCodec(slow_get), clock=clock)
    with pytest.raises(AuthorityUnavailable):
        adapter.publish_state(1, authority=permit(clock))
    assert not node.created


def test_cancel_during_publisher_creation_prevents_command_publish(setup):
    clock, node, _, _, adapter = setup
    cancel = threading.Event()
    node.on_create = cancel.set
    with pytest.raises(InterruptedError):
        adapter.publish_state(1, authority=permit(clock), cancel_event=cancel)
    assert len(node.created) == 1 and node.publishers[0].frames == []


def test_permit_expiry_after_publisher_creation_prevents_publish(setup):
    clock, node, _, _, adapter = setup
    node.on_create = lambda: setattr(clock, 'now', clock.now + .5)
    with pytest.raises(AuthorityUnavailable):
        adapter.publish_state(1, authority=permit(clock))
    assert node.publishers[0].frames == []


def test_lost_authority_cannot_reuse_an_existing_publisher(setup):
    clock, node, _, _, adapter = setup
    adapter.publish_state(1, authority=permit(clock))
    with pytest.raises(AuthorityUnavailable):
        adapter.publish_state(4, authority=permit(clock, owned=False))
    assert len(node.publishers[0].frames) == 1


def test_close_destroys_only_own_publishers_and_never_sends_stop(setup):
    clock, node, _, _, adapter = setup
    adapter.publish_state(1, authority=permit(clock))
    adapter.publish_zero_velocity(authority=permit(clock))
    before = [len(p.frames) for p in node.publishers]
    adapter.close(); adapter.close()
    assert node.destroyed == node.publishers
    assert [len(p.frames) for p in node.publishers] == before
    with pytest.raises(DDSAdapterClosed):
        adapter.publish_state(4, authority=permit(clock))
    with pytest.raises(DDSAdapterClosed):
        adapter.graph_snapshot()


def test_optional_installed_ros_generated_messages_without_participant():
    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        pytest.skip('ROS-generated classes are checked in the ARM64/Jazzy image')
    # Loading/constructing message classes creates no ROS participant, node,
    # publisher or subscriber. This catches aliases/setters our fake cannot.
    codec = MessageCodec(get_message, clock_ns=lambda: 1_792_000_000_000_123_456)
    state = codec.build_state(1)
    nav = codec.build_velocity(.125, -.0625, .25)
    assert state.data.state == 1
    assert state.header.stamp.sec == 1_792_000_000
    assert nav.data.x_vel == .125 and nav.data.y_vel == -.0625 and nav.data.yaw_vel == .25
