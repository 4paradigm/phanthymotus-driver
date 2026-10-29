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

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class Node:
        def __init__(self, *args):
            self.messages = []
            self.topics = {}
            self.timers = []
            self.time = 10**10
            self.removed = []
            self.destroyed = False

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=self.time))

        def create_publisher(self, cls, topic, qos):
            messages = self.messages if topic == module.OUTPUT_TOPIC else []
            self.topics[topic] = messages
            return NS(publish=messages.append)

        def create_timer(self, *args):
            self.timers.append(args)

        def destroy_subscription(self, sub):
            self.removed.append(sub)

        def destroy_publisher(self, pub):
            self.removed.append(pub)

        def destroy_node(self):
            self.destroyed = True

    for name, attrs in (('rclpy.node', {'Node': Node}),
                        ('rclpy.qos', {'qos_profile_sensor_data': object()}),
                        ('std_msgs.msg', {'String': String, 'Bool': Bool})):
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
    assert tool['topic_out'] == [{'topic': '/bumi_visual_odometry/status',
                                  'format': 'data/json'}]
    assert set(tool['inputSchema']['properties']['action']['enum']) == {
        'start', 'stop', 'info'}
    assert 'x-action-params' not in tool['inputSchema']
    assert set(tool['configSchema']['properties']) == {'calibration'}
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
    assert plugin.dispatch('info', {})['state'] == 'idle'


def test_unknown_action_is_rejected_instead_of_reported_as_success(plugin):
    assert plugin.dispatch('not-a-card-action', {}) is None
    assert plugin.dispatch('', {}) is None


def tracking(plugin):
    node = plugin._node
    node.active = True
    now = node.now()
    node.health = module.Health(node.segment, None, now-100_000_000)
    for offset in (100_000_000, 50_000_000, 0):
        for kind in node.health.kinds:
            node.health.observe(kind, now-offset, now-offset, True)
    node.estimate = dict(stamp_ns=now, pose=dict(
        position=[1., 2., 3.], quaternion=[0., 0., 0., 1.],
        frame_id=node.segment+'_odom', child_frame_id=node.base_frame(),
        covariance=[0.]*36))
    node.health.details['visual_diagnostic'] = dict(inliers=20, matches=30, lost=False)
    return node


def summary(node):
    return json.loads(node.node.topics['/bumi_visual_odometry/status'][-1].data)


def test_summary_is_small_and_measures_segment_displacement_without_changing_odom(plugin):
    node = tracking(plugin)
    node.publish()
    assert summary(node) == dict(status='tracking', displacement_m=0.,
                                position_m=[1., 2., 3.], pose_reference='rgb_optical',
                                age_ms=0., reason=None)
    node.estimate['pose']['position'] = [1., 2., 5.123456]
    node.last_summary_ns = None
    node.publish()
    assert summary(node)['displacement_m'] == 2.123
    assert summary(node)['position_m'] == [1., 2., 5.123]
    assert last(plugin)['pose']['position'] == [1., 2., 5.123456]
    assert last(plugin)['pose']['covariance'] == [0.]*36
    assert last(plugin)['vendor']['visual_diagnostic']['inliers'] == 20


def test_summary_throttle_keeps_the_twenty_hz_odom_and_immediate_lost_transition(
        plugin, monkeypatch):
    node = tracking(plugin)
    clock = [0]
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock[0])
    for i in range(20):
        clock[0] = i*50_000_000
        node.publish()
    assert len(node.node.messages) == 20
    assert len(node.node.topics['/bumi_visual_odometry/status']) == 5
    assert node.node.timers[0][0] == .05
    # Lose freshness only 150 ms after the last summary: do not wait for 5 Hz.
    clock[0] = 950_000_000
    node.node.time += 500_000_001
    node.tick()
    assert len(node.node.topics['/bumi_visual_odometry/status']) == 6
    assert summary(node)['status'] == 'lost'
    assert summary(node)['position_m'] is None
    assert summary(node)['displacement_m'] is None
    assert summary(node)['age_ms'] == 500.
    assert last(plugin)['pose'] is None


def test_stop_and_reset_do_not_display_a_previous_segment_pose(plugin):
    node = tracking(plugin)
    node.publish()
    plugin.dispatch('stop', {})
    assert summary(node)['status'] == 'initializing'
    assert summary(node)['position_m'] is None
    assert summary(node)['displacement_m'] is None
    plugin.dispatch('start', {})
    tracking(plugin)
    node.publish()
    assert summary(node)['displacement_m'] == 0.
    assert summary(node)['reason'] is None


def test_info_keeps_full_sample_and_exposes_it_in_canvas_details(plugin):
    node = tracking(plugin)
    result = plugin.dispatch('info', {})
    assert result['topic_out'][0]['format'] == 'data/json'
    assert result['odom_topic'] == module.OUTPUT_TOPIC
    assert result['sample'] == last(plugin)
    assert result['quality']['visual_diagnostic']['inliers'] == 20
    assert '"covariance"' in result['description']
    assert '"visual_diagnostic"' in result['description']


# ── recoverable faults: one bad sample must not kill the segment ─────────────

def started_node(plugin):
    plugin.dispatch('start', {})
    return plugin._node


def pose_at(node, position):
    return dict(stamp_ns=node.now(),
                pose=dict(position=list(position), quaternion=[0, 0, 0, 1],
                          frame_id=node.segment+'_odom',
                          child_frame_id=node.base_frame(),
                          covariance=[0.]*36))


def feed(node, position, step_ns=30_000_000):
    node.node.time += step_ns
    data = pose_at(node, position)
    t = data['stamp_ns']
    node.visual_pending[t] = data
    node.diagnostic_pending[t] = object()
    node._release_visual(t)
    return data


def test_a_single_pose_glitch_is_dropped_not_latched(plugin):
    node = started_node(plugin)
    feed(node, [0., 0., 0.])
    accepted = node.estimate
    feed(node, [.5, 0., 0.])           # one impossible registration
    assert node.health.reason is None
    assert node.estimate is accepted
    assert node.jump_count == 1
    feed(node, [.002, 0., 0.])         # back on the accepted trajectory
    assert node.health.reason is None
    assert node.estimate['pose']['position'] == [.002, 0., 0.]
    assert node.jump_count == 0


def test_persistent_pose_reset_requests_a_new_segment(plugin):
    node = started_node(plugin)
    old_segment = node.segment
    feed(node, [0., 0., 0.])
    feed(node, [.5, 0., 0.])
    feed(node, [.52, 0., 0.])          # consistent with the new baseline
    assert node.restart_requested == 'visual pose reset'
    assert node.segment == old_segment
    node.tick()
    assert node.restarts == 1
    assert node.segment != old_segment
    assert node.restart_requested is None
    assert node.estimate is None


def test_stale_camera_does_not_spin_restarts_but_recovers(plugin, monkeypatch):
    node = started_node(plugin)
    clock = [1000.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node.health.samples['camera'] = node.now()
    node.health.samples['visual'] = node.now()
    node.health.fault('input stale')
    node.tick()                         # starts grace, no restart yet
    assert node.restarts == 0
    clock[0] += 1.0
    node.tick()
    assert node.restarts == 1
    assert node.segment is not None


def test_jump_grace_restarts_a_segment_without_a_second_sample(plugin, monkeypatch):
    node = started_node(plugin)
    clock = [1000.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    feed(node, [0., 0., 0.])
    feed(node, [.5, 0., 0.])
    clock[0] += .7
    node.tick()
    assert node.restarts == 1


def test_start_and_stop_toggle_the_shared_raw_branch(plugin):
    plugin.dispatch('start', {})
    raw_topic = plugin._node.raw_control_topic
    assert raw_topic == '/robot/odometry_input/raw_enabled'
    assert [m.data for m in plugin._node.node.topics[raw_topic]] == [True]
    plugin.dispatch('stop', {})
    assert [m.data for m in plugin._node.node.topics[raw_topic]] == [True, False]
