"""Offline conversion of the documented G1 mapping Odometry source."""
import importlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def message(sec=27, nanosec=123456789, frame='source_map', child='source_imu'):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame),
              child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=1.25, y=-2., z=.3),
                              orientation=NS(w=math.sqrt(.5), x=0., y=0., z=math.sqrt(.5))),
                      covariance=[0.]*36))


def adapter():
    path = Path(__file__).resolve().parents[1] / 'unitree/g1/slam_odometry.py'
    assert path.exists(), 'G1 mapping odometry adapter is required'
    return importlib.import_module('unitree.g1.slam_odometry').MappingOdometry()


def test_named_quaternion_fields_metric_position_and_exact_source_stamp():
    a = adapter()
    msg = message()
    result = a.convert(msg)
    assert result['schema'] == 'motus.odom/1'
    assert result['units']['length'] == 'm'
    assert result['stamp_ms'] == 27123
    assert result['pose'] == dict(frame_id='source_map', child_frame_id='source_imu',
                                  position=[1.25, -2., .3], quaternion=[0., 0., math.sqrt(.5), math.sqrt(.5)])
    q = result['vendor']['motus_slam']
    assert q['source_stamp_ns'] == 27123456789
    assert q['stamp_source'] is None and q['status'] == 'unavailable'
    assert q['clock_id'] is None and q['clock_epoch'] is None and q['segment_id'] is None
    assert set(q['missing_facts']) >= {'measurement_stamp', 'tracking_quality', 'segment_identity', 'sensor_extrinsic'}
    assert result['vendor']['source_covariance'] == [0.]*36
    msg.pose.pose.position.x = 99
    msg.pose.covariance[0] = 99
    assert result['pose']['position'][0] == 1.25
    assert result['vendor']['source_covariance'][0] == 0.
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('path,value', [
    ('header.stamp.sec', -1), ('header.stamp.sec', True), ('header.stamp.nanosec', 1000000000),
    ('header.stamp.nanosec', -1), ('header.stamp.nanosec', 1.5),
    ('header.frame_id', ''), ('child_frame_id', ' '), ('child_frame_id', None),
    ('pose.pose.position.x', float('nan')), ('pose.pose.position.y', float('inf')),
    ('pose.pose.position.z', True), ('pose.pose.orientation.w', 0.),
    ('pose.pose.orientation.x', '0'), ('pose.covariance', [0.]*35),
    ('pose.covariance', [float('nan')]*36),
])
def test_invalid_fields_emit_a_rejection_without_an_old_pose(path, value):
    a = adapter()
    a.convert(message())
    msg = message(sec=28)
    obj = msg
    keys = path.split('.')
    for key in keys[:-1]:
        obj = getattr(obj, key)
    setattr(obj, keys[-1], value)
    result = a.convert(msg)
    assert result['pose'] is None
    assert result['vendor']['motus_slam']['status'] == 'unavailable'
    assert result['vendor']['motus_slam']['rejection']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('sec,nsec', [(0, 0), (0, 999999)])
def test_zero_or_sub_millisecond_stamp_cannot_become_receipt_time(sec, nsec):
    result = adapter().convert(message(sec, nsec))
    assert result['stamp_ms'] is None and result['pose'] is None


def test_missing_pose_field_is_not_defaulted():
    msg = message()
    del msg.pose.pose.orientation.w
    assert adapter().convert(msg)['pose'] is None


def test_repeated_backward_time_and_frame_reset_do_not_generate_epochs_or_segments():
    a = adapter()
    packets = [a.convert(message(s, frame=f)) for s, f in
               [(27, 'map'), (27, 'map'), (26, 'map'), (28, 'new_map')]]
    assert [p['vendor']['motus_slam']['rejection'] for p in packets][1:] == [
        'duplicate_source_stamp', 'out_of_order_source_stamp', 'source_frames_changed']
    assert all(p['vendor']['motus_slam']['segment_id'] is None for p in packets)
    assert all(p['vendor']['motus_slam']['clock_epoch'] is None for p in packets)
    assert all(p['pose'] is None for p in packets[1:])
    assert a.source_generation != adapter().source_generation


def test_declaration_retains_source_frames_but_does_not_claim_slam_readiness():
    a = adapter()
    assert a.declaration('/g1/slam/mapping/odom')['frame_id'] is None
    a.convert(message())
    d = a.declaration('/g1/slam/mapping/odom')
    assert d['format'] == 'state/odom' and d['kind'] == 'pose'
    assert d['source_topic'] == 'rt/unitree/slam_mapping/odom'
    assert d['frame_id'] == 'source_map' and d['child_frame_id'] == 'source_imu'
    assert d['length_unit'] == 'm'
    assert d['clock_id'] is None and d['stamp_source'] is None and d['extrinsic'] is None
    assert d['clock']['verified'] is False


def test_packaged_module_is_available_in_flat_g1_container():
    dockerfile = Path(__file__).resolve().parents[1] / 'unitree/g1/Dockerfile'
    assert 'COPY slam_odometry.py /work/slam_odometry.py' in dockerfile.read_text()


def test_base_odometry_envelope_does_not_invent_motion_axes():
    for result in (adapter().convert(message()), adapter().convert(NS())):
        assert result['frame'] == 'world'
        assert result['twist'] == [None]*6
        assert result['units'] == {'linear': 'm/s', 'angular': 'rad/s', 'length': 'm'}


def test_profile_millisecond_collision_is_rejected_without_rounding_time_forward():
    a = adapter()
    first = a.convert(message(nanosec=123000001))
    second = a.convert(message(nanosec=123000002))
    assert second['stamp_ms'] == first['stamp_ms']
    assert second['pose'] is None
    assert second['vendor']['motus_slam']['source_stamp_ns'] == 27123000002
    assert second['vendor']['motus_slam']['rejection'] == 'duplicate_profile_stamp'


def test_unproven_quality_and_coordinate_reset_never_promote_to_tracking():
    a = adapter()
    first = a.convert(message())
    reset = message(sec=28)
    reset.pose.pose.position.x = 10000.
    reset.errorCode = 0
    reset.tracking = True
    reset.segment_id = 'untrusted-extra-field'
    second = a.convert(reset)
    for sample in (first, second, adapter().convert(message(sec=1))):
        q = sample['vendor']['motus_slam']
        assert q['status'] == 'unavailable' and q['segment_id'] is None
        assert q['clock_epoch'] is None and q['stamp_source'] is None
