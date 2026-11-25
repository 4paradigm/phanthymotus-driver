"""ROS-free checks for G1 read-only SLAM evidence capture."""
import json
from types import SimpleNamespace as NS

import pytest

from unitree.g1.slam_capture_probe import main, source_header, to_record


def timestamp(sec=7, nanosec=89, frame='source_scan'):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def pose():
    return NS(
        header=timestamp(frame='mapping_map'),
        child_frame_id='mapping_imu',
        pose=NS(pose=NS(
            position=NS(x=1.25, y=-2.5, z=.4),
            orientation=NS(x=0., y=0., z=0., w=1.)),
            covariance=[0.]*36),
    )


def cloud():
    return NS(
        header=timestamp(), width=1, height=1, point_step=16,
        row_step=16, is_bigendian=False, is_dense=False,
        data=bytearray(b'1234567890abcdef'),
        fields=[NS(name='x', offset=0, datatype=7, count=1),
                NS(name='y', offset=4, datatype=7, count=1),
                NS(name='z', offset=8, datatype=7, count=1)],
    )


def test_odom_evidence_preserves_fields_and_does_not_assert_tracking():
    msg = pose()
    output = to_record('odom', msg, received_ns=123, received_monotonic_ns=456)
    assert output['source_header']['stamp'] == {'sec': 7, 'nanosec': 89}
    assert output['source_header']['frame_id'] == 'mapping_map'
    assert output['child_frame_id'] == 'mapping_imu'
    assert output['position_xyz'] == [1.25, -2.5, .4]
    assert output['quaternion_xyzw'] == [0., 0., 0., 1.]
    assert output['measurement_clock_verified'] is False
    assert output['tracking_verified'] is False
    msg.pose.pose.position.x = 9.
    assert output['position_xyz'][0] == 1.25
    json.dumps(output, allow_nan=False)


def test_cloud_records_exact_header_and_layout_without_raw_payload():
    msg = cloud()
    result = to_record('cloud', msg, received_ns=123, received_monotonic_ns=456)
    assert result['source_topic'] == 'rt/utlidar/cloud_livox_mid360'
    assert result['data_length_bytes'] == 16
    assert result['source_header']['stamp'] == {'sec': 7, 'nanosec': 89}
    assert [f['name'] for f in result['fields']] == ['x', 'y', 'z']
    assert 'data' not in result
    assert 'data_length_bytes' in result
    json.dumps(result, allow_nan=False)


def test_nonmeasurement_timestamps_are_never_replaced_with_receipt_time():
    msg = cloud()
    msg.header.stamp.sec = -1
    result = to_record('cloud', msg, received_ns=10_000_000, received_monotonic_ns=1000)
    assert result['source_header']['stamp']['sec'] == -1
    assert result['received_ns'] == 10_000_000
    assert result['measurement_clock_verified'] is False


def test_status_payload_capped_and_raw_kind_preserved():
    packet = to_record('slam_info', NS(data='a'*10000),
                       received_ns=10, received_monotonic_ns=20, max_status_chars=32)
    assert packet['status_payload'] == 'a'*32
    assert packet['status_payload_truncated'] is True
    assert packet['source_topic'] == 'rt/slam_info'
    assert to_record('slam_key_info', NS(data_='ok'),
                     received_ns=1, received_monotonic_ns=2)['status_payload'] == 'ok'


def test_invalid_kind_fails_without_subscribing():
    with pytest.raises(ValueError, match='unknown capture'):
        to_record('move', NS(), received_ns=1, received_monotonic_ns=2)


@pytest.mark.parametrize('args', [
    ['--duration', '-1'], ['--duration', 'nan'],
    ['--duration', '121'], ['--max-records', '0'],
    ['--max-records', '20001'],
])
def test_cli_rejects_invalid_resource_limits_without_sdk_import(args):
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


def test_regular_sport_odom_is_captured_without_native_slam_or_lidar():
    msg = NS(
        stamp=NS(sec=44, nanosec=12345678),
        position=[1., 2., 3.], velocity=[.1, .2, .3],
        yaw_speed=.25, error_code=0,
        imu_state=NS(quaternion=[1., 0., 0., 0.],
                     gyroscope=[.1, .2, .3]),
    )
    record = to_record('sport_odom', msg,
                       received_ns=123, received_monotonic_ns=456)
    assert record['source_topic'] == 'rt/odommodestate'
    assert record['source_header'] == {'stamp': {'sec': 44, 'nanosec': 12345678},
                                       'frame_id': None}
    assert record['position_xyz'] == [1., 2., 3.]
    assert record['quaternion_wxyz'] == [1., 0., 0., 0.]
    assert record['position_reference'] == 'unknown'
    assert record['imu_orientation_reference'] == 'unknown'
    assert record['measurement_clock_verified'] is False
    assert record['tracking_verified'] is False
    msg.position[0] = 99.
    assert record['position_xyz'][0] == 1.
    json.dumps(record, allow_nan=False)
