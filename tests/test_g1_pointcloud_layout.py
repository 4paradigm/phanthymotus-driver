"""Offline PointCloud2 layout conversion, without DDS or ROS."""
import struct
from types import SimpleNamespace as NS

import pytest

from unitree.g1 import pointcloud_utils


def message():
    fields = [NS(name='xyz', offset=0, datatype=7, count=3),
              NS(name='ring', offset=12, datatype=4, count=1),
              NS(name='tag', offset=14, datatype=2, count=1)]
    point = struct.pack('>fffHB', 1., -2., 3., 54321, 17) + b'X'
    return NS(width=1, height=2, point_step=16, row_step=20,
              is_bigendian=True, fields=fields, data=(point+b'pad!')*2)


def convert(msg):
    fn = getattr(pointcloud_utils, 'pack_slam_cloud', None)
    assert callable(fn), 'G1 needs a lossless SLAM layout converter'
    return fn(msg)


def test_multiline_endian_conversion_preserves_every_field_and_point_padding():
    msg = message()
    original = msg.data
    data = convert(msg)
    assert data == (struct.pack('<fffHB', 1., -2., 3., 54321, 17)+b'X')*2
    assert msg.data == original and msg.is_bigendian and msg.row_step == 20


@pytest.mark.parametrize('change', [
    {'data': b'short'}, {'data': bytes(41)}, {'row_step': 15},
    {'point_step': 0}, {'height': -1}, {'width': True},
    {'fields': [NS(name='bad', offset=15, datatype=7, count=1)]},
    {'fields': [NS(name='bad', offset=0, datatype=99, count=1)]},
    {'fields': [NS(name='bad', offset=0, datatype=7, count=0)]},
    {'fields': [NS(name='a', offset=0, datatype=7, count=1),
                NS(name='b', offset=2, datatype=7, count=1)]},
])
def test_rejects_invalid_or_ambiguous_layout(change):
    msg = message()
    msg.__dict__.update(change)
    with pytest.raises(ValueError):
        convert(msg)


def test_little_endian_preserves_nan_bits_and_all_point_bytes():
    msg = message()
    msg.is_bigendian = False
    msg.data = bytes.fromhex('0100c07f') + msg.data[4:]
    assert convert(msg) == msg.data[:16] + msg.data[20:36]


@pytest.mark.parametrize('datatype,size', [(1,1),(2,1),(3,2),(4,2),(5,4),(6,4),(7,4),(8,8)])
def test_all_ros_field_types_and_counts(datatype, size):
    msg = NS(width=1, height=1, point_step=2*size, row_step=2*size,
             is_bigendian=True, fields=[NS(name='value', offset=0, datatype=datatype, count=2)],
             data=bytes(range(2*size)))
    assert convert(msg) == bytes(range(size))[::-1] + bytes(range(size,2*size))[::-1]
