"""Offline Orbbec camera motion contract; no native ROS or camera access."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

PATH = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0/rgbd_camera.py'
SPEC = importlib.util.spec_from_file_location('tianyi_rgbd_imu', PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)
NOW = 10_000_000_000


def motion(stamp=NOW, frame='gyro_optical', xyz=(1., 2., 3.)):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9)),
              angular_velocity=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
              linear_acceleration=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
              orientation_covariance=[-1.] + [0.] * 8)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


class Node:
    def __init__(self, *args, **kwargs):
        self.subscriptions = {}
        self.publishers = {}
        self.now_ns = NOW
    def create_publisher(self, msg_type, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]
    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback
    def create_client(self, *args): return NS(service_is_ready=lambda: False)
    def create_timer(self, *args): pass
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=self.now_ns))


class Buffer:
    def __init__(self):
        self.calls = []
        s = math.sqrt(.5)
        self.quaternions = {'gyro_optical': (0., 0., s, s), 'accel_frame': (s, 0., 0., s)}
        self.translations = {}
    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp.nanoseconds))
        if source not in self.quaternions: raise LookupError('unknown source frame')
        x, y, z, w = self.quaternions[source]
        transform = NS(rotation=NS(x=x, y=y, z=z, w=w))
        if source in self.translations:
            transform.translation = self.translations[source]
        return NS(transform=transform)


@pytest.fixture
def source(monkeypatch, request):
    monkeypatch.setitem(sys.modules, 'rclpy.node', NS(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=NS(from_msg=lambda s: NS(nanoseconds=s.sec*10**9+s.nanosec))))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=object, CameraInfo=object, CompressedImage=object, Imu=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=object))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=lambda b, n: NS(buffer=b, node=n)))
    ros = NS(ctx_tianyi=object(), ctx_core=object(), executor_tianyi=NS(add_node=lambda n: None),
             executor_core=NS(add_node=lambda n: None))
    plugin = m.RgbdCameraPlugin(getattr(request, 'param', {}), 'test', ros)
    plugin._work = lambda: None
    plugin.start()
    plugin.registered = True
    plugin.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    plugin.count = 1
    yield plugin
    plugin.stop()


def feed(source, gyro=None, accel=None):
    source._imu('accel', accel or motion(NOW-5_000_000, 'accel_frame', (0., 0., 9.81)))
    source._imu('gyro', gyro or motion())


def packets(source):
    publisher = source.pub_node.publishers.get('/test/camera/rgbd/imu')
    return [] if publisher is None else [json.loads(msg.data) for msg in publisher.messages]


def test_third_port_emits_capture_clock_raw_vectors_and_separate_90_degree_rotations(source):
    feed(source)
    assert source.get_tool()['topic_out'][2] == {'topic': '/test/camera/rgbd/imu', 'format': 'sensor/imu'}
    packet, = packets(source)
    assert packet['schema'] == 'motus.imu/1'
    assert packet['source_id'] and packet['epoch']
    assert packet['stamp_ns'] == NOW
    assert packet['accel_stamp_ns'] == NOW-5_000_000
    assert packet['stamp_source'] == 'measurement'
    assert packet['clock_id'] == source._declarations()[0]['vendor']['clock_id']
    assert packet['clock_id'] == source._declarations()[1]['vendor']['clock_id']
    assert packet['camera_frame_id'] == 'color_optical'
    assert (packet['gyro_frame_id'], packet['accel_frame_id']) == ('gyro_optical', 'accel_frame')
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert packet['linear_acceleration'] == [0., 0., 9.81]
    np.testing.assert_allclose(packet['camera_from_gyro'], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-12)
    np.testing.assert_allclose(packet['camera_from_accel'], [[1,0,0],[0,0,-1],[0,1,0]], atol=1e-12)
    assert packet['rotation_source'] == 'tf'
    assert packet['units'] == {'angular_velocity': 'rad/s', 'linear_acceleration': 'm/s^2'}
    assert set(source.imu_buffer.calls) == {('color_optical', 'gyro_optical', NOW), ('color_optical', 'accel_frame', NOW-5_000_000)}
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_reuses_vendor_streams_and_existing_subnode_for_tf_without_new_camera_owner(source):
    assert '/ob_camera_head/gyro/sample' in source.sub_node.subscriptions
    assert '/ob_camera_head/accel/sample' in source.sub_node.subscriptions
    assert source.imu_listener.node is source.sub_node


@pytest.mark.parametrize('frame', ['', 'unknown_body_frame'])
def test_missing_motion_frame_or_tf_withholds_only_imu(source, frame):
    feed(source, gyro=motion(frame=frame))
    assert packets(source) == []
    result = source.dispatch('info', {})
    assert result['state'] == 'running' and result['registered_source']
    assert result['camera_info'] and 'error' not in result
    assert result['imu_status']['state'] == 'unavailable'
    assert result['imu_status']['reason']


def test_tf_failure_recovers_without_restarting_rgbd(source):
    feed(source, gyro=motion(frame='unknown'))
    feed(source, gyro=motion(NOW+1_000_000))
    assert len(packets(source)) == 1
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_missing_accel_capture_does_not_block_later_matching_pairs(source):
    source._imu('accel', motion(NOW-50_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW-25_000_000))
    assert packets(source) == []
    # The older accel remains nearest to the unmatched gyro even after a
    # newer accel arrives; that hole must not pin the entire gyro queue.
    source._imu('accel', motion(NOW+5_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW+5_000_000))
    assert [p['stamp_ns'] for p in packets(source)] == [NOW+5_000_000]
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_waits_for_matching_accel_when_only_older_samples_exist(source):
    source._imu('accel', motion(NOW-50_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW-25_000_000))
    assert packets(source) == []
    source._imu('accel', motion(NOW-24_000_000, 'accel_frame'))
    assert [p['stamp_ns'] for p in packets(source)] == [NOW-25_000_000]


def test_matching_discards_accel_history_older_than_the_nearest_preceding_sample(source):
    for offset in range(-400, 1):
        source._imu('accel', motion(NOW+offset*1_000_000, 'accel_frame'))
    source._imu('gyro', motion())
    assert len(packets(source)) == 1
    assert len(source.imu_samples['accel']) == 1


@pytest.mark.parametrize('gyro_stamp,accel_stamp', [
    (NOW, NOW-20_000_001), (NOW-500_000_001, NOW-500_000_001),
    (NOW+50_000_001, NOW+50_000_001), (0, 0),
])
def test_skew_stale_future_or_zero_capture_stamp_is_not_restamped(source, gyro_stamp, accel_stamp):
    feed(source, motion(gyro_stamp), motion(accel_stamp, 'accel_frame'))
    assert packets(source) == []
    assert source.dispatch('info', {})['imu_status']['state'] == 'unavailable'


def test_no_source_global_clock_confirmation_means_no_imu_even_if_tf_exists(source):
    source.registered = False
    feed(source)
    assert packets(source) == []
    assert 'clock' in source.dispatch('info', {})['imu_status']['reason'].lower()


def test_bad_raw_vector_or_nonunit_tf_is_not_published(source):
    feed(source, gyro=motion(xyz=(float('nan'), 0., 0.)))
    assert packets(source) == []
    source.imu_buffer.quaternions['gyro_optical'] = (0., 0., 0., 2.)
    feed(source, gyro=motion(NOW+1))
    assert packets(source) == []
    assert source.running and source.error is None


def test_accel_arriving_after_gyro_can_complete_one_pair_without_duplicate_replay(source):
    source._imu('gyro', motion())
    source._imu('accel', motion(NOW, 'accel_frame'))
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 1


def test_status_becomes_unavailable_when_source_stops_producing(source):
    feed(source)
    source.sub_node.now_ns += 500_000_001
    result = source.dispatch('info', {})
    assert result['imu_status']['state'] == 'unavailable'
    assert 'stale' in result['imu_status']['reason']
    assert result['state'] == 'running' and 'error' not in result


def test_restart_changes_epoch_and_does_not_reuse_old_acceleration(source):
    feed(source)
    epoch = packets(source)[0]['epoch']
    source.stop()
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source._imu('gyro', motion(NOW+1))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 2
    assert packets(source)[1]['epoch'] != epoch


def test_disabled_source_keeps_rgbd_ready_with_declared_optional_port(source):
    source.stop()
    source.imu_enabled = False
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    feed(source)
    assert packets(source) == []
    result = source.dispatch('info', {})
    assert result['camera_info'] and result['state'] == 'running'
    assert result['imu_status']['state'] == 'disabled'
    assert 'error' not in result


@pytest.mark.parametrize('source', [{'imu_enabled': False}], indirect=True)
def test_disabled_config_never_attaches_motion_or_tf_subscriptions(source):
    assert '/ob_camera_head/gyro/sample' not in source.sub_node.subscriptions
    assert '/ob_camera_head/accel/sample' not in source.sub_node.subscriptions
    assert not hasattr(source, 'imu_listener')
    assert source.dispatch('info', {})['imu_status']['state'] == 'disabled'
    assert source.dispatch('info', {})['camera_info']


def test_missing_tf2_dependency_withholds_motion_without_failing_rgbd_start(source, monkeypatch):
    source.stop()
    source.imu_attached = False
    monkeypatch.setitem(sys.modules, 'tf2_ros', None)
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    result = source.dispatch('info', {})
    assert result['state'] == 'running' and result['camera_info']
    assert 'error' not in result
    assert result['imu_status']['state'] == 'unavailable'
    assert 'tf2' in result['imu_status']['reason']


def test_source_capture_interruption_changes_epoch_on_resume(source):
    feed(source)
    epoch = packets(source)[0]['epoch']
    source.sub_node.now_ns += 600_000_000
    feed(source, motion(NOW+600_000_000), motion(NOW+600_000_000, 'accel_frame'))
    assert len(packets(source)) == 2
    assert packets(source)[1]['epoch'] != epoch


def test_long_interruption_cannot_be_hidden_by_reusing_stale_acceleration(source):
    feed(source)
    source.sub_node.now_ns += 600_000_000
    source._imu('gyro', motion(NOW+600_000_000))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+600_000_000, 'accel_frame'))
    assert len(packets(source)) == 2


def test_out_of_order_capture_is_not_published_twice(source):
    feed(source)
    source._imu('gyro', motion(NOW-1))
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 1



def test_source_time_tf_translation_and_rotation_use_one_lookup_per_sensor(source):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.012, y=-.034, z=.056),
                                      'accel_frame': NS(x=-.078, y=.09, z=-.011)}
    feed(source)
    packet, = packets(source)
    assert packet['camera_from_gyro_translation'] == [.012, -.034, .056]
    assert packet['camera_from_accel_translation'] == [-.078, .09, -.011]
    assert packet['translation_source'] == 'tf'
    np.testing.assert_allclose(packet['camera_from_gyro'], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-12)
    assert source.imu_buffer.calls == [('color_optical', 'gyro_optical', NOW),
                                       ('color_optical', 'accel_frame', NOW-5_000_000)]
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'available'


@pytest.mark.parametrize('translation', [None, NS(x=float('nan'), y=0., z=0.),
                                         NS(x=0., y=float('inf'), z=0.), NS(x=0., y=0.)])
def test_missing_or_invalid_tf_translation_preserves_attitude_only(source, translation):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.01, y=.02, z=.03)}
    if translation is not None:
        source.imu_buffer.translations['accel_frame'] = translation
    feed(source)
    packet, = packets(source)
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert 'camera_from_gyro_translation' not in packet
    assert 'camera_from_accel_translation' not in packet
    assert 'translation_source' not in packet
    status = source.dispatch('info', {})['imu_status']
    assert status['state'] == 'available'
    assert status['fusion']['state'] == 'unavailable'
    assert 'translation' in status['fusion']['reason']


def test_restart_does_not_reuse_old_tf_translation_or_acceleration(source):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.01, y=.02, z=.03),
                                      'accel_frame': NS(x=.04, y=.05, z=.06)}
    feed(source)
    first, = packets(source)
    source.stop()
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source.imu_buffer.translations.clear()
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'unavailable'
    source._imu('gyro', motion(NOW+1))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    resumed = packets(source)[1]
    assert resumed['epoch'] != first['epoch']
    assert 'camera_from_accel_translation' not in resumed
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'unavailable'
