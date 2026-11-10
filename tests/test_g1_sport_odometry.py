"""ROS-free G1 sport-mode odometry candidate / safety-contract coverage."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from common.odom import parse_interface
from common.spatial_slam import SpatialSlamError, validate_slam_capability, validate_tracking_pose
from unitree.g1.sport_odometry import SportOdometry, SOURCE_TOPIC


def packet(sec=27, nsec=123456789, *, error_code=0):
    return NS(
        stamp=NS(sec=sec, nanosec=nsec),
        position=[1.2, -2.3, 0.4],
        velocity=[0.1, 0.2, 0.3],
        yaw_speed=0.25,
        error_code=error_code,
        imu_state=NS(quaternion=[math.sqrt(.5), 0., 0., math.sqrt(.5)],
                     gyroscope=[0.1, 0.2, 0.3]),
    )


def test_source_is_regular_sport_odom_not_mapping_slam():
    raw = packet()
    converted = SportOdometry().convert(raw)
    assert converted['schema'] == 'motus.odom/1'
    assert converted['vendor']['source_topic'] == SOURCE_TOPIC == 'rt/odommodestate'
    assert converted['vendor']['source_type'] == 'unitree_go.msg.dds_.SportModeState_'
    assert converted['stamp_ms'] == 27123
    assert converted['vendor']['motus_slam']['source_stamp_ns'] == 27123456789
    assert converted['vendor']['raw_position'] == [1.2, -2.3, .4]
    assert converted['vendor']['raw_imu_quaternion_xyzw'] == [0., 0., math.sqrt(.5), math.sqrt(.5)]
    assert converted['vendor']['raw_velocity'] == [.1, .2, .3]
    assert converted['vendor']['raw_yaw_speed'] == .25
    assert converted['frame'] == 'world'
    assert converted['twist'] == [None] * 6
    assert converted['pose'] is None  # Source message contains no frame IDs.
    assert converted['vendor']['motus_slam']['status'] == 'unavailable'
    assert converted['vendor']['motus_slam']['clock_id'] is None
    assert converted['vendor']['motus_slam']['segment_id'] is None
    assert converted['vendor']['motus_slam']['stamp_source'] is None
    raw.position[0] = 9.
    raw.imu_state.quaternion[0] = 9.
    assert converted['vendor']['raw_position'][0] == 1.2
    assert converted['vendor']['raw_imu_quaternion_xyzw'][-1] == math.sqrt(.5)
    json.dumps(converted, allow_nan=False)


def test_explicit_frames_produce_unverified_pose_not_verified_tracking():
    converter = SportOdometry(frame_id='odom', child_frame_id='imu_in_torso')
    msg = converter.convert(packet())
    assert msg['pose'] == {
        'frame_id': 'odom', 'child_frame_id': 'imu_in_torso',
        'position': [1.2, -2.3, 0.4],
        'quaternion': [0., 0., math.sqrt(.5), math.sqrt(.5)],
    }
    with pytest.raises(SpatialSlamError):
        validate_tracking_pose(msg)
    declaration = converter.declaration('/alpha/slam/odom')
    assert declaration['source_topic'] == 'rt/odommodestate'
    assert declaration['frame_id'] == 'odom' and declaration['child_frame_id'] == 'imu_in_torso'
    assert declaration['clock']['verified'] is False
    assert declaration['stamp_source'] is None and declaration['clock_id'] is None
    assert declaration['available'] is False
    with pytest.raises(SpatialSlamError, match='availability'):
        validate_slam_capability(converter.interface())
    parse_interface(converter.interface())


@pytest.mark.parametrize('kwargs', [
    {'frame_id': 'odom'}, {'child_frame_id': 'base'},
    {'frame_id': ' ', 'child_frame_id': 'base'},
    {'frame_id': 'odom', 'child_frame_id': ''},
    {'frame_id': 1, 'child_frame_id': 'base'},
])
def test_incomplete_or_invalid_frame_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        SportOdometry(**kwargs)


@pytest.mark.parametrize('mutation', [
    ('stamp.sec', -1), ('stamp.sec', True),
    ('stamp.nanosec', 1_000_000_000),
    ('stamp.nanosec', -1),
    ('stamp.nanosec', 0.5),
    ('position', [1., 2.]),
    ('position', [0., float('nan'), 0.]),
    ('position', [0., 0., True]),
    ('velocity', [0., float('inf'), 0.]),
    ('imu_state.quaternion', [0., 0., 0., 0.]),
    ('imu_state.quaternion', [1., 0., 0.]),
    ('error_code', -1),
    ('error_code', True),
    ('yaw_speed', float('nan')),
])
def test_invalid_input_never_leaks_nonfinite_or_stale_pose(mutation):
    adapter = SportOdometry(frame_id='odom', child_frame_id='body')
    good = adapter.convert(packet(sec=26))
    msg = packet(sec=27)
    field, value = mutation
    obj = msg
    parts = field.split('.')
    for item in parts[:-1]:
        obj = getattr(obj, item)
    setattr(obj, parts[-1], value)
    got = adapter.convert(msg)
    assert good['pose'] is not None
    assert got['pose'] is None
    assert got['vendor']['motus_slam']['status'] == 'unavailable'
    assert got['vendor']['motus_slam']['rejection']
    json.dumps(got, allow_nan=False)


@pytest.mark.parametrize('sec,nsec', [(0,0), (0, 999999)])
def test_zero_stamp_is_invalid_and_not_replaced_with_receipt(sec,nsec):
    x = SportOdometry().convert(packet(sec, nsec))
    assert x['pose'] is None and x['stamp_ms'] is None
    assert x['vendor']['motus_slam']['source_stamp_ns'] is None


def test_bad_error_code_does_not_become_tracking():
    p = SportOdometry(frame_id='odom', child_frame_id='base').convert(packet(error_code=13))
    assert p['vendor']['source_error_code'] == 13
    assert p['pose'] is None
    assert p['vendor']['motus_slam']['rejection'] == 'nonzero_source_error_code'


def test_duplicates_millisecond_collisions_and_rollbacks():
    converter = SportOdometry(frame_id='odom', child_frame_id='body')
    first = converter.convert(packet())
    duplicate = converter.convert(packet())
    collision = converter.convert(packet(nsec=123456790))
    rollback = converter.convert(packet(sec=26))
    future = converter.convert(packet(sec=29))
    assert first['pose'] is not None
    assert [p['vendor']['motus_slam']['rejection'] for p in
            (duplicate,collision,rollback,future)] == [
                'duplicate_source_stamp', 'duplicate_profile_stamp',
                'out_of_order_source_stamp',
                'unverified_continuity_after_discontinuity',
            ]
    assert all(p['pose'] is None for p in (duplicate,collision,rollback,future))
    observation = converter.observation_status()
    assert observation['suspected_resets'] == 1
    assert observation['continuity_quarantined'] is True
    assert observation['accepted_samples'] == 1
    assert observation['rejected_samples'] == 4
    assert observation['clock_epoch'] is None and observation['segment_id'] is None


def test_receipt_staleness_is_diagnostic_not_measurement_time():
    adapter = SportOdometry()
    assert adapter.observation_status()['state'] == 'waiting'
    p = adapter.convert(packet())
    t = p['vendor']['motus_slam']['observation']['received_monotonic_ns']
    assert adapter.observation_status(now_monotonic_ns=t)['receipt_stale'] is False
    late = adapter.observation_status(now_monotonic_ns=t+3_000_000_000)
    assert late['state'] == 'stale_observation' and late['receipt_stale'] is True
    assert p['stamp_ms'] == 27123
    assert p['vendor']['motus_slam']['stamp_source'] is None


def test_dockerfile_copies_sport_source():
    docker = Path(__file__).resolve().parents[1] / 'unitree/g1/Dockerfile'
    assert 'COPY sport_odometry.py /work/sport_odometry.py' in docker.read_text()
