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

    class QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.__dict__.update(kwargs)

    class Policies:
        RELIABLE = 'reliable'
        BEST_EFFORT = 'best_effort'
        TRANSIENT_LOCAL = 'transient_local'
        VOLATILE = 'volatile'
        KEEP_LAST = 'keep_last'

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
            messages = self.messages if topic.endswith(module.OUTPUT_TOPIC) else []
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
                        ('rclpy.qos', {'qos_profile_sensor_data': object(),
                                       'QoSProfile': QoSProfile,
                                       'ReliabilityPolicy': Policies,
                                       'DurabilityPolicy': Policies,
                                       'HistoryPolicy': Policies}),
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
    # Both pre-existing outputs must stay declared: the concise status summary
    # first, then the full odometry sample. Saved canvas layouts reference these
    # ports by index, so dropping one is not an additive change.
    assert tool['topic_out'] == [
        {'topic': '/robot/bumi_visual_odometry/status', 'format': 'data/json'},
        {'topic': '/robot/bumi_visual_odometry/state/odom', 'format': 'state/odom'}]
    assert set(tool['inputSchema']['properties']['action']['enum']) == {
        'start', 'stop', 'info'}
    assert 'x-action-params' not in tool['inputSchema']
    assert 'configSchema' not in tool
    assert 'no IMU/EKF/fusion' in tool['description']
    assert '/robot/bumi_visual_odometry/status' in tool['description']


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
    assert result['odom_interface']['slam_pose'] == {'available': True, 'profile': 'motus_slam/1'}
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
    return json.loads(node.node.messages[-1].data)['vendor']


def test_odom_carries_segment_displacement_with_a_status_companion(plugin):
    node = tracking(plugin)
    node.publish()
    assert summary(node)['status'] == 'tracking'
    assert summary(node)['displacement_m'] == 0.
    status = json.loads(node.node.topics['/robot/bumi_visual_odometry/status'][-1].data)
    assert status['status'] == 'tracking'
    assert status['displacement_m'] == 0.
    node.estimate['pose']['position'] = [1., 2., 5.123456]
    node.publish()
    assert summary(node)['displacement_m'] == pytest.approx(2.123456)
    assert last(plugin)['pose']['position'] == [1., 2., 5.123456]
    assert last(plugin)['pose']['covariance'] == [0.]*36
    assert last(plugin)['vendor']['visual_diagnostic']['inliers'] == 20


def test_turn_angle_is_relative_to_the_first_imu_sample(plugin, monkeypatch):
    import math
    class Imu:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
            self.orientation = NS(x=0., y=0., z=0., w=1.)
            self.orientation_covariance = [0.]*9
            self.angular_velocity = NS(x=0., y=0., z=0.)
            self.linear_acceleration = NS(x=0., y=0., z=0.)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Imu=Imu))
    node = tracking(plugin)
    node.health.imu_required = True
    node.health.kinds = node.health.kinds + ('imu',)
    node.health.details['imu_assistance'] = {}
    node.body_imu_pub = NS(publish=lambda msg: None)
    node._publish_body_imu([0., 0., 0., 1.], [0., 0., 0.], [0., 0., 0.],
                           node.now(), 'read_completion')
    node.publish()
    assert summary(node)['turn_angle_deg'] == 0.
    node.node.time += 100_000_000
    node._publish_body_imu([0., 0., math.sin(math.pi/4), math.cos(math.pi/4)],
                           [0., 0., 0.], [0., 0., 0.],
                           node.now(), 'read_completion')
    node.publish()
    assert summary(node)['turn_angle_deg'] == pytest.approx(90.)


def test_both_outputs_keep_twenty_hz_odom_and_immediate_lost_transition(
        plugin, monkeypatch):
    node = tracking(plugin)
    clock = [0]
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock[0])
    for i in range(20):
        clock[0] = i*50_000_000
        node.publish()
    assert len(node.node.messages) == 20
    statuses = node.node.topics['/robot/bumi_visual_odometry/status']
    # 20 odometry samples over 1 s, but the same tracking state is summarised
    # at the legacy 200 ms cadence: t=0, 200, 400, 600, 800 ms.
    assert len(statuses) == 5
    assert node.node.timers[0][0] == .05
    # Lose freshness after the one-second camera watchdog: do not wait for 5 Hz.
    clock[0] = 1_450_000_000
    node.node.time += 1_000_000_001
    node.tick()
    assert summary(node)['status'] == 'lost'
    assert summary(node)['displacement_m'] is None
    assert last(plugin)['pose'] is None
    lost = json.loads(statuses[-1].data)
    assert lost['status'] == 'lost'
    assert lost['position_m'] is None
    assert lost['reason']


def test_stop_and_reset_do_not_display_a_previous_segment_pose(plugin):
    node = tracking(plugin)
    node.publish()
    plugin.dispatch('stop', {})
    assert summary(node)['status'] == 'initializing'
    assert summary(node)['motus_slam']['status'] == 'initializing'
    assert summary(node)['displacement_m'] is None
    plugin.dispatch('start', {})
    tracking(plugin)
    node.publish()
    assert summary(node)['displacement_m'] == 0.
    assert summary(node)['reason'] is None


def test_info_keeps_full_sample_and_exposes_it_in_canvas_details(plugin):
    node = tracking(plugin)
    result = plugin.dispatch('info', {})
    assert [port['format'] for port in result['topic_out']] == ['data/json', 'state/odom']
    assert result['odom_topic'] == '/robot'+module.OUTPUT_TOPIC
    assert result['sample'] == last(plugin)
    assert result['quality']['visual_diagnostic']['inliers'] == 20
    assert '"covariance"' in result['description']
    assert '"visual_diagnostic"' in result['description']


def test_card_declares_and_publishes_both_status_and_odometry_ports(plugin, monkeypatch):
    """Regression for the non-additive removal: both pre-existing ports stay.

    The card contract must keep declaring the ``data/json`` status summary next
    to ``state/odom`` and must publish both across the lifecycle, with the
    legacy summary payload and coherent SLAM/status values. Canvas layouts and
    ROS subscribers already depend on the status port, so removing it cannot be
    called additive.
    """
    node = plugin._node
    status_topic = '/robot/bumi_visual_odometry/status'
    odom_topic = '/robot/bumi_visual_odometry/state/odom'

    # 1) Declared: get_tools and info() agree, status port first.
    for ports in (plugin.get_tools()[0]['topic_out'],
                  plugin.dispatch('info', {})['topic_out']):
        assert [(p['topic'], p['format']) for p in ports] == [
            (status_topic, 'data/json'), (odom_topic, 'state/odom')]
    statuses = node.node.topics[status_topic]
    assert json.loads(statuses[-1].data)['status'] == 'initializing'

    # 2) Published while tracking, with the unchanged legacy summary shape.
    clock = [10**12]
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock[0])
    tracking(plugin)
    sample = node.publish()
    summary_msg = json.loads(statuses[-1].data)
    assert set(summary_msg) == {'status', 'turn_angle_deg', 'displacement_m',
                                'position_m', 'pose_reference', 'age_ms', 'reason'}
    assert summary_msg['status'] == sample['vendor']['status'] == \
        sample['vendor']['motus_slam']['status'] == 'tracking'
    assert summary_msg['pose_reference'] == sample['vendor']['pose_reference']
    assert summary_msg['position_m'] == [1., 2., 3.]
    assert summary_msg['displacement_m'] == 0.

    # 3) A state transition publishes immediately; steady state is throttled.
    clock[0] += 50_000_000
    node.publish()
    assert json.loads(statuses[-1].data)['status'] == 'tracking'
    node.health.fault('camera stopped')
    node.publish()
    lost = json.loads(statuses[-1].data)
    assert lost['status'] == 'lost'
    assert last(plugin)['vendor']['motus_slam']['status'] == 'lost'

    # 4) Stopping reports initializing in both the summary and SLAM metadata.
    node.health = module.Health(node.segment, None, node.now())
    plugin.dispatch('stop', {})
    stopped = json.loads(statuses[-1].data)
    assert stopped['status'] == 'initializing'
    assert last(plugin)['vendor']['motus_slam']['status'] == 'initializing'


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
    # Latched stopped state is published at construction, then true on start.
    assert [m.data for m in plugin._node.node.topics[raw_topic]] == [False, True]
    assert plugin._node.raw_control_qos.kwargs['durability'] == 'transient_local'
    plugin.dispatch('stop', {})
    assert [m.data for m in plugin._node.node.topics[raw_topic]] == [False, True, False]


def test_lost_diagnostic_clears_the_pose_before_the_restart(plugin):
    node = started_node(plugin)
    node.diagnostic_started = True
    for position in ([0., 0., 0.], [.001, 0., 0.], [.002, 0., 0.]):
        feed(node, position)
    node.health.samples['camera'] = node.now()
    node.health.counts['camera'] = 3
    assert node.publish()['vendor']['status'] == 'tracking'

    t = node.now()
    node.diagnostic(NS(header=NS(stamp=NS(sec=t//10**9, nanosec=t % 10**9)),
                       lost=True, inliers=0, matches=0))
    sample = node.publish()
    assert sample['vendor']['status'] == 'lost'
    assert sample['pose'] is None
    assert 'tracking lost' in sample['vendor']['reason']


def test_camera_epoch_change_requests_a_restart(plugin):
    node = started_node(plugin)
    node.epochs['camera'] = 'old-epoch'
    node.metadata(NS(data=json.dumps({'epoch': 'new-epoch'})))
    assert node.restart_requested == 'camera input restarted'


def test_body_imu_requires_a_fresh_sample_timestamp(plugin, monkeypatch):
    class Imu:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
            self.orientation = NS(x=0., y=0., z=0., w=1.)
            self.orientation_covariance = [0.]*9
            self.angular_velocity = NS(x=0., y=0., z=0.)
            self.linear_acceleration = NS(x=0., y=0., z=0.)

    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Imu=Imu))
    node = plugin._node
    node.imu_assisted, node.imu_source = True, 'state'
    node.health = module.Health('seg', None, 0, imu_required=True)
    node.health.details['imu_assistance'] = {}
    published = []
    node.body_imu_pub = NS(publish=published.append)
    now = node.now()

    def message(stamp):
        return NS(data=json.dumps({'stamp_ns': stamp, 'quaternion': [0., 0., 0., 1.],
                                   'angular_vel': [0., 0., 0.],
                                   'linear_acc': [0., 0., 9.8]}))

    node.body_imu(message(now))
    assert len(published) == 1
    assert published[0].header.frame_id == node.base_frame()
    assert node.health.samples['imu'] == now

    with pytest.raises(ValueError, match='body IMU input stale'):
        node.body_imu(message(now-600_000_000))


def test_camera_imu_mode_is_rejected_instead_of_breaking_the_camera(executor):
    with pytest.raises(ValueError, match='camera IMU is not packaged'):
        module.VisualOdometryPlugin({'imu_source': 'camera'}, 'robot', executor)


def test_controller_imu_reader_publishes_changed_samples_only(plugin, monkeypatch):
    import time

    class Imu:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
            self.orientation = NS(x=0., y=0., z=0., w=1.)
            self.orientation_covariance = [0.]*9
            self.angular_velocity = NS(x=0., y=0., z=0.)
            self.linear_acceleration = NS(x=0., y=0., z=0.)

    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Imu=Imu))
    node = plugin._node
    node.active = True
    node.imu_assisted, node.imu_source = True, 'controller'
    node.health = module.Health('seg', None, 0, imu_required=True)
    node.health.details['imu_assistance'] = {}
    published = []
    node.body_imu_pub = NS(publish=published.append)
    reading = [0.0]

    class Controller:
        def get_imu_data(self):
            return NS(ori=[0.0, 0.0, 0.0, 1.0],
                      angular_vel=[reading[0], 0.0, 0.0],
                      linear_acc=[0.0, 0.0, 9.8])

    node.high_ctrl = Controller()
    node._start_controller_imu()
    time.sleep(.08)
    first = len(published)
    assert first >= 1
    time.sleep(.08)
    assert len(published) == first  # unchanged samples must not refresh freshness
    reading[0] = .1
    time.sleep(.08)
    assert len(published) > first
    node._stop_controller_imu()


def test_output_topics_follow_the_bundle_namespace(executor):
    plugin = module.VisualOdometryPlugin({}, 'alpha', executor)
    tool = plugin.get_tools()[0]
    assert plugin._node.output == '/alpha'+module.OUTPUT_TOPIC
    assert plugin._node.status_topic == '/alpha'+module.STATUS_TOPIC
    assert tool['topic_out'] == [
        {'topic': '/alpha'+module.STATUS_TOPIC, 'format': 'data/json'},
        {'topic': '/alpha'+module.OUTPUT_TOPIC, 'format': 'state/odom'}]
