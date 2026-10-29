"""Bundle-facing visual odometry plugin: lifecycle, tools and dispatch.

Only ROS transport and the estimator process boundary are replaced. There is no
IMU/EKF in this card, so nothing here may fake a predicted pose: a stopped or
lost segment must publish no pose at all.
"""
import json
import sys
from types import ModuleType
from types import SimpleNamespace as NS

import pytest

from noetix.bumi import visual_odometry as module
from test_contract import calibration


@pytest.fixture
def executor(monkeypatch):
    class String:
        def __init__(self, data=''):
            self.data = data

    class Node:
        def __init__(self, *args):
            self.messages = []
            self.time = 10**10
            self.removed = []
            self.destroyed = False

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=self.time))

        def create_publisher(self, cls, topic, qos):
            return NS(publish=self.messages.append)

        def create_timer(self, *args):
            pass

        def destroy_subscription(self, sub):
            self.removed.append(sub)

        def destroy_publisher(self, pub):
            self.removed.append(pub)

        def destroy_node(self):
            self.destroyed = True

    for name, attrs in (('rclpy.node', {'Node': Node}),
                        ('rclpy.qos', {'qos_profile_sensor_data': object()}),
                        ('std_msgs.msg', {'String': String})):
        fake = ModuleType(name)
        fake.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, fake)

    class Executor:
        def __init__(self):
            self.nodes = []

        def add_node(self, node):
            self.nodes.append(node)

    return Executor()


@pytest.fixture
def plugin(executor, monkeypatch):
    # Starting the card would subscribe to the camera topics and launch the
    # estimator; both are exercised elsewhere, so record the binding instead.
    monkeypatch.setattr(module.VisualOdometryNode, 'bind_inputs',
                        lambda self, namespace: setattr(self, 'bound', namespace))
    instance = module.VisualOdometryPlugin({}, 'robot', executor)
    return instance


def last(plugin):
    return json.loads(plugin._node.node.messages[-1].data)


def test_card_is_a_read_only_sensor_on_its_own_topic(plugin):
    tools = plugin.get_tools()
    assert len(tools) == 1
    tool = tools[0]
    assert tool['name'] == 'rgbd_visual_odometry'
    assert tool['type'] == 'sensor' and tool['multiInstance'] is False
    assert tool['topic_out'] == [{'topic': '/bumi_visual_odometry/state/odom',
                                  'format': 'state/odom'}]
    assert set(tool['inputSchema']['properties']['action']['enum']) == {
        'start', 'stop', 'info', 'reset', 'config'}
    assert set(tool['configSchema']['properties']) == {'calibration'}
    assert 'no IMU/EKF/fusion' in tool['description']


def test_start_binds_the_bundle_namespace_and_waits_for_inputs(plugin, executor):
    result = plugin.dispatch('start', {})
    assert result['state'] == 'running'
    assert plugin._node.bound == 'robot'
    assert plugin._node.active is True
    assert executor.nodes == [plugin._node.node]
    message = last(plugin)
    assert message['pose'] is None
    assert message['twist'] == [None]*6
    assert message['vendor']['status'] == 'initializing'
    assert message['vendor']['pose_reference'] == 'rgb_optical'


def test_stopping_the_card_reports_stopped_and_never_a_pose(plugin):
    plugin.dispatch('start', {})
    plugin.dispatch('stop', {})
    message = last(plugin)
    assert plugin._node.active is False
    assert message['pose'] is None
    assert message['vendor']['status'] == 'initializing'
    assert message['vendor']['reason'] == 'visual odometry stopped'


def test_reset_moves_to_a_new_segment_and_reports_the_old_one_lost(plugin):
    plugin.dispatch('start', {})
    segment = plugin._node.segment
    plugin.dispatch('reset', {})
    assert plugin._node.segment != segment
    samples = [json.loads(m.data) for m in plugin._node.node.messages]
    assert any(s['vendor']['segment_id'] == segment and s['vendor']['status'] == 'lost'
               for s in samples)


def test_measured_mount_only_changes_the_pose_reference(plugin):
    result = plugin.dispatch('config', {'calibration': calibration()})
    assert result['pose_reference'] == 'body'
    assert result['calibration_id'] == 'fixture'
    assert last(plugin)['vendor']['pose_reference'] == 'body'


def test_malformed_mount_is_refused_without_interrupting_the_card(plugin):
    plugin.dispatch('start', {})
    with pytest.raises(ValueError):
        plugin.dispatch('config', {'calibration': {'calibration_id': 'x'}})
    assert plugin._node.active is True


def test_info_is_the_default_action(plugin):
    result = plugin.dispatch('rgbd_visual_odometry', {})
    assert result['state'] == 'idle'
    assert result['namespace'] == 'robot'
    assert result['odom_interface']['provides'] == []
