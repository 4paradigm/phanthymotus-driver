"""Run real lifecycle code; replace only the ROS transport and process boundary.

There is no EKF in this runtime, so nothing here may fake a predicted pose: if
the camera stops, the output has to say lost rather than keep moving.
"""
import json
import sys
from types import ModuleType
from types import SimpleNamespace as NS

import pytest

from test_contract import calibration
from noetix.bumi_visual_odometry.runtime import VisualRuntime
from noetix.bumi_visual_odometry.service import SensorService


@pytest.fixture
def runtime(monkeypatch):
    class String:
        def __init__(self, data=''):
            self.data = data

    class Node:
        def __init__(self, *args):
            self.messages = []
            self.time = 10**10
            self.removed = []
            self.topics = []

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
            pass

        def get_topic_names_and_types(self):
            return self.topics

    for name, attrs in (('rclpy.node', {'Node': Node}),
                        ('rclpy.qos', {'qos_profile_sensor_data': object()}),
                        ('std_msgs.msg', {'String': String})):
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    visual = VisualRuntime()
    yield visual
    visual.close()


def last(runtime):
    return json.loads(runtime.node.messages[-1].data)


def diagnostics(*, lost=False, inliers=20, matches=30, sec=10, nanosec=0):
    return NS(lost=lost, inliers=inliers, matches=matches,
              header=NS(stamp=NS(sec=sec, nanosec=nanosec)))


def test_missing_calibration_still_runs_in_the_optical_frame(runtime):
    runtime.start()
    for _ in range(10):
        runtime.tick()
    message = last(runtime)
    assert message['vendor']['status'] == 'initializing'
    assert message['pose'] is None
    assert message['twist'] == [None]*6
    assert message['vendor']['pose_reference'] == 'rgb_optical'
    assert runtime.info()['odom_interface']['provides'] == []
    assert not runtime.children.children


def test_calibration_selects_the_body_reference(runtime):
    runtime.config['calibration'] = calibration()
    runtime.start()
    for _ in range(3):
        runtime.tick()
    assert runtime.info()['pose_reference'] == 'body'
    assert last(runtime)['vendor']['pose_reference'] == 'body'
    assert last(runtime)['vendor']['calibration_id'] == 'fixture'


def test_malformed_calibration_is_refused_not_assumed_zero(runtime, tmp_path):
    runtime.config['calibration'] = calibration()
    runtime.start()
    service = SensorService(runtime, tmp_path/'config.json')
    with pytest.raises(ValueError):
        service.dispatch({'action': 'config', 'calibration': {'calibration_id': 'x'}})
    assert runtime.active and runtime.calibration.id == 'fixture'


def test_reset_uses_a_new_segment_and_reports_the_old_one_lost(runtime):
    runtime.start()
    segment = runtime.segment
    runtime.reset()
    assert runtime.segment != segment
    samples = [json.loads(m.data) for m in runtime.node.messages]
    assert any(s['vendor']['segment_id'] == segment and s['vendor']['status'] == 'lost'
               for s in samples)


def test_stopped_sensor_never_reports_a_pose(runtime):
    runtime.start()
    runtime.stop()
    message = last(runtime)
    assert message['pose'] is None
    assert message['vendor']['status'] == 'initializing'
    assert message['vendor']['reason'] == 'sensor stopped'


def test_startup_lost_diagnostic_cannot_brick_initialization(runtime):
    runtime.start()
    runtime.diagnostic(diagnostics(lost=True, inliers=0, matches=0))
    assert runtime.health.reason is None
    runtime.diagnostic(diagnostics())
    assert runtime.diagnostic_started is True
    runtime.diagnostic(diagnostics(lost=True))
    assert 'lost' in runtime.health.reason


def test_visual_pose_is_released_only_with_its_matching_diagnostic(runtime, monkeypatch):
    from noetix.bumi_visual_odometry import runtime as module
    runtime.start()
    pose = dict(position=[0., 0., 0.], quaternion=[0., 0., 0., 1.],
                frame_id='odom', child_frame_id='opt', covariance=[0.]*36)
    monkeypatch.setattr(module, 'estimate',
                        lambda msg, segment, frame_id: dict(stamp_ns=10**10, pose=dict(pose)))
    visual = NS(header=NS(stamp=NS(sec=10, nanosec=0)))
    runtime.odometry(visual)
    assert runtime.estimate is None
    runtime.diagnostic(diagnostics(sec=10, nanosec=0))
    assert runtime.estimate is not None
    assert runtime.estimate['pose']['position'] == [0., 0., 0.]


def test_visual_pose_jump_faults_instead_of_teleporting(runtime, monkeypatch):
    from noetix.bumi_visual_odometry import runtime as module
    runtime.start()
    poses = [dict(position=[0., 0., 0.], quaternion=[0., 0., 0., 1.],
                  frame_id='odom', child_frame_id='opt', covariance=[0.]*36),
             dict(position=[5., 0., 0.], quaternion=[0., 0., 0., 1.],
                  frame_id='odom', child_frame_id='opt', covariance=[0.]*36)]
    values = iter([10**10, 10**10+10**8])
    monkeypatch.setattr(module, 'estimate',
                        lambda msg, segment, frame_id: dict(stamp_ns=next(values),
                                                            pose=dict(poses.pop(0))))
    runtime.odometry(NS(header=NS(stamp=NS(sec=10, nanosec=0))))
    runtime.diagnostic(diagnostics(sec=10, nanosec=0))
    runtime.node.time = 10**10+10**8
    runtime.odometry(NS(header=NS(stamp=NS(sec=10, nanosec=10**8))))
    with pytest.raises(ValueError):
        runtime.diagnostic(diagnostics(sec=10, nanosec=10**8))


def test_reapplying_the_same_configuration_does_not_reset_an_active_segment(runtime, tmp_path):
    runtime.start()
    service = SensorService(runtime, tmp_path/'config.json')
    service.dispatch({'action': 'config', 'source_namespace': 'robot'})
    segment = runtime.segment
    service.dispatch({'action': 'config', 'source_namespace': 'robot'})
    assert runtime.segment == segment and runtime.active
    assert json.loads((tmp_path/'config.json').read_text())['source_namespace'] == 'robot'


def test_invalid_namespace_is_refused_before_interrupting(runtime, tmp_path):
    runtime.start()
    service = SensorService(runtime, tmp_path/'config.json')
    with pytest.raises(ValueError):
        service.dispatch({'action': 'config', 'source_namespace': 'not a namespace'})
    assert runtime.active


def test_discovery_uses_the_only_camera_input_or_the_configured_namespace(runtime):
    runtime.node.topics = [('/robot/odometry_input/camera_meta', ['std_msgs/msg/String'])]
    assert runtime.discover() == 'robot'
    runtime.node.topics.append(('/other/odometry_input/camera_meta', ['std_msgs/msg/String']))
    assert runtime.discover() is None
    runtime.config['source_namespace'] = 'robot'
    assert runtime.discover() == 'robot'
    runtime.config['source_namespace'] = 'missing'
    assert runtime.discover() is None


def test_mount_transform_is_identity_without_measurement(runtime):
    from noetix.bumi_visual_odometry.runtime import mount_transform
    from common.visual_odometry import CameraCalibration
    assert runtime.base_frame() == runtime.segment+'_base'
    assert mount_transform(None) == ([0., 0., 0.], [0., 0., 0., 1.])
    mount = calibration()['camera_to_body']
    translation, quaternion = mount_transform(CameraCalibration(calibration()))
    assert translation == mount['translation']
    assert quaternion == mount['quaternion']
