import math
import socket
import struct
import time
from unittest.mock import patch

import pytest

from agilex.piper.follow_analysis import FEEDBACK_IDS, REFERENCE_IDS, FollowPair, JointFrames
from agilex.piper.follow_consistency import (
    CAN_FRAME, CAN_MASK, KERNEL_TIMESPEC, NAME, SO_TIMESTAMPNS, FollowConsistency, receive_sample,
)
from agilex.piper.follow_dashboard import HEIGHT, WIDTH, render_dashboard
from common.vendor_runtime import DriverBundle


def send(pair, ids, stamp, first_joint=0.0):
    values = (int(first_joint * 1000), 0, 0, 0, 0, 0)
    for index, frame_id in enumerate(ids):
        pair.accept(frame_id, struct.pack('>ii', *values[2 * index:2 * index + 2]), stamp + index / 10000)


def test_triplets_require_valid_sequence_and_window():
    decoder = JointFrames(FEEDBACK_IDS)
    part = struct.pack('>ii', -1250, 2200)
    for frame_id, stamp in zip((0x2A5, 0x2A7, 0x2A6), (1, 1.001, 1.002)):
        assert decoder.accept(frame_id, part, stamp) is None
    decoder.accept(0x2A5, part, 2)
    decoder.accept(0x2A6, part, 2.001)
    assert decoder.accept(0x2A7, part, 2.020) is None
    decoder.accept(0x2A5, part, 3)
    decoder.accept(0x2A6, part, 3.001)
    assert decoder.accept(0x2A7, part, 3.002).angles == (-1.25, 2.2) * 3


def test_feedback_without_a_reference_is_not_a_verdict():
    pair = FollowPair(3, 0.25, 0.35, 0.08)
    send(pair, FEEDBACK_IDS, 1, -2)
    assert pair.snapshot(1.1)['state'] == 'reference_unavailable'
    assert pair.snapshot(1.1)['max_error_deg'] is None
    assert pair.snapshot(1.7)['state'] == 'feedback_unavailable'


def test_deviation_duration_requires_contiguous_aligned_samples():
    pair = FollowPair(3, 0.25, 0.35, 0.08)
    for step in range(8):
        stamp = 1 + step * 0.05
        send(pair, REFERENCE_IDS, stamp)
        send(pair, FEEDBACK_IDS, stamp, -7)
        result = pair.snapshot(stamp + 0.001)
        if step == 1:
            assert result['state'] == 'transient_deviation'
    assert result['state'] == 'sustained_deviation'
    assert result['worst_joint'] == 1
    assert result['joints'][0]['error_deg'] == -7
    send(pair, REFERENCE_IDS, 1.6)
    assert pair.snapshot(1.61)['state'] == 'unaligned'
    send(pair, FEEDBACK_IDS, 1.6, -7)
    assert pair.snapshot(1.61)['state'] == 'transient_deviation'
    send(pair, REFERENCE_IDS, 1.65)
    send(pair, FEEDBACK_IDS, 1.65)
    assert pair.snapshot(1.66)['state'] == 'following'


def test_delay_is_estimated_only_with_distinct_motion():
    pair = FollowPair(3, 0.25, 0.35, 0.08)
    for step in range(161):
        t = step / 100
        send(pair, REFERENCE_IDS, t, 15 * math.sin(t * 7))
        send(pair, FEEDBACK_IDS, t, 15 * math.sin((t - 0.08) * 7))
    assert pair.snapshot(1.601, include_lag=True)['estimated_lag_ms'] == 80
    for step in range(161, 300):
        send(pair, REFERENCE_IDS, step / 100, 4)
        send(pair, FEEDBACK_IDS, step / 100, 4)
    assert pair.snapshot(2.991, include_lag=True)['estimated_lag_ms'] is None


def test_socket_receiver_rejects_host_originated_and_untimestamped_frames():
    class Receiver:
        def recvmsg(self, *_):
            return packet, ancillary, flags, None

    assert CAN_MASK == 0x80000000 | 0x40000000 | 0x7FF
    packet = CAN_FRAME.pack(REFERENCE_IDS[0], 8, b'\x00' * 8)
    seconds, nanos = divmod(time.time_ns(), 1_000_000_000)
    ancillary = [(socket.SOL_SOCKET, SO_TIMESTAMPNS, KERNEL_TIMESPEC.pack(seconds, nanos))]
    flags = 4
    with (patch.object(socket, 'CMSG_SPACE', lambda size: size, create=True),
          patch.object(socket, 'MSG_DONTROUTE', 4, create=True),
          patch.object(socket, 'MSG_TRUNC', 32, create=True),
          patch.object(socket, 'MSG_CTRUNC', 8, create=True)):
        assert receive_sample(Receiver()) is None
        flags = 0
        assert receive_sample(Receiver())[0] == REFERENCE_IDS[0]
        ancillary = []
        assert receive_sample(Receiver()) is None


def test_mcp_tool_has_read_only_actions_and_explicit_bus_mapping():
    with pytest.raises(ValueError, match='distinct'):
        FollowConsistency({'pairs': {'left': 'can0', 'right': 'can0'}}, 'test', None)
    plugin = FollowConsistency({'pairs': {'left': 'can0', 'right': 'can1'}}, 'test', None)
    bundle = DriverBundle([plugin])
    definition, = bundle.get_all_tools()
    assert definition['topic_out'] == [{'topic': f'/test/{NAME}', 'format': 'data/json'},
                                       {'topic': f'/test/{NAME}/tracking_dashboard', 'format': 'image/jpeg'}]
    assert definition['title'] == 'ALOHA Leader-Follower Tracking'
    assert definition['type'] == 'sensor'
    assert bundle.dispatch(NAME, {'action': 'info'})['state'] == 'idle'
    with pytest.raises(ValueError, match='only'):
        bundle.dispatch(NAME, {'action': 'move'})


def test_dashboard_renders_valid_and_unavailable_pairs():
    import cv2
    import numpy as np

    pair = FollowPair(3, 0.25, 0.35, 0.08)
    send(pair, REFERENCE_IDS, 1)
    send(pair, FEEDBACK_IDS, 1, -7)
    active = dict(pair.snapshot(1.01), interface='can0')
    unavailable = dict(pair.snapshot(2), interface='can1')
    single = render_dashboard({'left': unavailable}, 3, 250, 1)
    mixed = render_dashboard({'left': active, 'right': unavailable}, 3, 250, 1)
    for jpeg in (single, mixed):
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert image.shape == (HEIGHT, WIDTH, 3)
    assert single != mixed
