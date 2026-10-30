"""Offline authorization and transport bounds for fixed-distance NAV_CMD."""
import threading
import time

import pytest

from deep_robotics.s10_pro.motion_dds import (
    AuthorityUnavailable, DDSCommandAdapter, MessageCodec, validate_operator_permit)


class TimeValue:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'sec': 'int32', 'nanosec': 'uint32'}

    def __init__(self):
        self.sec = self.nanosec = 0


class Meta:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'frame_id': 'uint64', 'stamp': 'builtin_interfaces/msg/Time'}

    def __init__(self):
        self.frame_id = 0
        self.stamp = TimeValue()


class MotionStateValue:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'state': 'int32'}

    def __init__(self):
        self.state = 0


class NavCmdValue:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'x_vel': 'float32', 'y_vel': 'float32', 'yaw_vel': 'float32'}

    def __init__(self):
        self.x_vel = self.y_vel = self.yaw_vel = 0.


class MotionState:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/MotionStateValue'}

    def __init__(self):
        self.header, self.data = Meta(), MotionStateValue()


class NavCmd:
    @classmethod
    def get_fields_and_field_types(cls):
        return {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/NavCmdValue'}

    def __init__(self):
        self.header, self.data = Meta(), NavCmdValue()


CLASSES = {
    'builtin_interfaces/msg/Time': TimeValue,
    'drdds/msg/MetaType': Meta,
    'drdds/msg/MotionStateValue': MotionStateValue,
    'drdds/msg/NavCmdValue': NavCmdValue,
    'drdds/msg/MotionState': MotionState,
    'drdds/msg/NavCmd': NavCmd,
}


class Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 2

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, cls, topic, qos):
        value = Publisher()
        self.publishers[topic] = value
        return value

    def get_publishers_info_by_topic(self, topic):
        return []

    def get_subscriptions_info_by_topic(self, topic):
        return [object(), object()]

    def destroy_publisher(self, publisher):
        pass


def permit(action, request_id='r1'):
    now = time.monotonic()
    return {'schema': 's10_pro.operator_permit/1', 'session_id': 'session',
        'robot_session': 'robot', 'request_id': request_id, 'action': action,
        'issued_monotonic': now - 1., 'expires_monotonic': now + 30.,
        'action_deadline_monotonic': now + 8.}


@pytest.fixture
def adapter():
    node = Node()
    codec = MessageCodec(lambda name: CLASSES[name], clock_ns=lambda: 1_000_000_000)
    codec.prepare(supervised_posture=True)
    value = DDSCommandAdapter(node, execution_enabled=True, supervised_posture=True, codec=codec)
    value.prepare_state_channel()
    return value, node


def test_distance_permit_may_repeat_same_bounded_request_and_zero_stop(adapter):
    value, node = adapter
    scope = permit('forward_30cm')
    for speed in (.12, .06, 0., 0.):
        assert value.publish_velocity(speed, 0., 0., authority=scope,
                                      cancel_event=threading.Event())['sent']
    assert [message.data.x_vel for message in node.publishers['/NAV_CMD'].messages] == [.12, .06, 0., 0.]


@pytest.mark.parametrize('action,vx', [
    ('forward_30cm', -.1), ('backward_30cm', .1), ('forward_30cm', 1.0001),
    ('stand', .1), ('move_mode', 0.)])
def test_distance_scope_rejects_wrong_direction_speed_or_posture_action(adapter, action, vx):
    with pytest.raises(AuthorityUnavailable):
        adapter[0].publish_velocity(vx, 0., 0., authority=permit(action),
                                    cancel_event=threading.Event())


def test_distance_scope_cannot_publish_state(adapter):
    with pytest.raises(AuthorityUnavailable):
        adapter[0].publish_state(17, authority=permit('forward_30cm'),
                                 cancel_event=threading.Event())


def test_permit_validation_accepts_only_named_finite_distance_actions():
    assert validate_operator_permit(permit('backward_30cm'), now=time.monotonic())['action'] == 'backward_30cm'
    with pytest.raises(AuthorityUnavailable):
        validate_operator_permit(permit('velocity'), now=time.monotonic())
