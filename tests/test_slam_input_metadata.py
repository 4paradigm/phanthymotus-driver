"""Raw ROS header evidence never implies a verified acquisition clock."""
from copy import deepcopy
import hashlib
from types import SimpleNamespace as NS

import pytest


def cloud(sec=12, nanosec=345):
    return NS(header=NS(frame_id='native_frame', stamp=NS(sec=sec, nanosec=nanosec)),
              data=bytearray(b'original cloud payload'))


def test_native_header_and_payload_keep_exact_source_time_and_unknown_clock():
    from common.slam_input import native_frame_metadata
    msg = cloud()
    packet = native_frame_metadata(msg, source_generation='driver-process-1')
    assert packet['schema'] == 'motus.slam_frame/1'
    assert packet['stamp_ns'] == 12_000_000_345
    assert packet['frame_id'] == 'native_frame'
    assert packet['source_stamp'] == 12_000_000_345
    assert packet['source_unit'] == 'ns'
    assert packet['source_header'] == dict(frame_id='native_frame', stamp=dict(sec=12, nanosec=345))
    assert packet['data_sha256'] == hashlib.sha256(msg.data).hexdigest()
    assert packet['source_generation'] == 'driver-process-1'
    assert packet['clock_id'] is None and packet['clock_epoch'] is None
    assert packet['source_clock'] is None and packet['stamp_source'] is None
    assert packet['mapping'] == dict(method=None, version=None, offset_ns=None,
                                     verified=False, uncertainty_ns=None)
    assert packet['received_ns'] > 0 and packet['received_monotonic_ns'] > 0


@pytest.mark.parametrize(('sec', 'nanosec'), [(0, 0), (-1, 0), (False, 1), (2, -1), (2, 10**9), (None, 1)])
def test_invalid_native_stamp_does_not_fall_back_to_receipt(sec, nanosec):
    from common.slam_input import native_frame_metadata
    packet = native_frame_metadata(cloud(sec, nanosec), source_generation='driver-process-1')
    assert packet['stamp_ns'] is None and packet['source_stamp'] is None
    assert packet['source_header']['stamp'] == dict(sec=sec, nanosec=nanosec)
    assert packet['clock_epoch'] is None
    assert packet['received_ns'] > 0


def test_absent_header_is_unknown_and_data_stays_immutable():
    from common.slam_input import native_frame_metadata
    msg = NS(data=bytearray(b'payload'))
    packet = native_frame_metadata(msg, source_generation='driver-process-1')
    assert packet['stamp_ns'] is None and packet['frame_id'] is None
    assert msg.data == bytearray(b'payload')
    assert packet['source_header'] == dict(frame_id=None, stamp=dict(sec=None, nanosec=None))


def test_source_mutation_does_not_rewrite_a_captured_evidence_packet():
    from common.slam_input import native_frame_metadata
    msg = cloud()
    packet = native_frame_metadata(msg, source_generation='driver-process-1')
    original = deepcopy(packet)
    msg.header.stamp.sec = 999
    msg.header.frame_id = 'changed_frame'
    msg.data[:] = b'mutated'
    assert packet == original
