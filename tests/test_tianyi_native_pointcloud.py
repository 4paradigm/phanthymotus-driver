"""ROS-free Tianyi PointCloud2 source layout and trust-boundary tests."""
import hashlib
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'x-humanoid' / 'tianyi2.0' / 'slam_pointcloud.py'
SPEC = importlib.util.spec_from_file_location('tianyi_native_slam_cloud', MODULE_PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def cloud(*, stamp=5, frame='orbbec_optical', row_padding=0):
    xyz = struct.pack('<fff', 1., 2., 3.)
    fields = [NS(name=n, offset=i * 4, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z'))]
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=17)),
              fields=fields, width=1, height=1, point_step=12,
              row_step=12 + row_padding, data=xyz + b'\0' * row_padding,
              is_bigendian=False)


def test_original_byte_layout_header_stamp_and_digest_are_preserved():
    msg = cloud(row_padding=8)
    before = bytes(msg.data)
    evidence = module.native_cloud_evidence(msg)
    assert msg.data == before
    assert evidence['frame_id'] == 'orbbec_optical'
    assert evidence['stamp_ns'] == 5_000_000_017
    assert evidence['byte_length'] == 20
    assert evidence['data_sha256'] == hashlib.sha256(before).hexdigest()
    sidecar = module.frame_declaration(evidence, 'generation-1')
    assert sidecar['stamp_ns'] == evidence['stamp_ns']
    assert sidecar['data_sha256'] == evidence['data_sha256']
    assert sidecar['mapping']['verified'] is False


def test_static_declaration_explicitly_cannot_admit_native_cloud_for_slam():
    claim = module.input_declaration('/tianyi/native', '/tianyi/native/metadata',
                                     {'frame_id': 'head_camera_optical'})
    assert claim['format'] == 'sensor/pointcloud2'
    assert claim['frame_id'] == 'head_camera_optical'
    assert claim['stamp_source'] == 'unknown'
    assert claim['clock']['verified'] is False
    assert claim['clock']['uncertainty_ns'] is None
    assert claim['timing_model'] == 'unknown'
    assert 'extrinsic' not in claim
    # "Valid PointCloud2" is not "verified acquisition time".
    assert module.input_declaration('/tianyi/native', '/metadata')['frame_id'] is None


@pytest.mark.parametrize(('field', 'value'), [
    ('point_step', 8), ('row_step', 10), ('width', 2),
    ('data', b'abcd'), ('fields', []),
])
def test_invalid_pointcloud_layout_is_rejected(field, value):
    msg = cloud()
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        module.native_cloud_evidence(msg)


def test_no_zero_timestamp_missing_frame_or_oversize_payload():
    with pytest.raises(ValueError, match='time or empty frame'):
        zero = cloud(stamp=0)
        zero.header.stamp.nanosec = 0
        module.native_cloud_evidence(zero)
    with pytest.raises(ValueError, match='time or empty frame'):
        module.native_cloud_evidence(cloud(frame=''))
    with pytest.raises(ValueError, match='bounded transport'):
        module.native_cloud_evidence(cloud(), max_bytes=10)
    with pytest.raises(ValueError, match='XYZ'):
        msg = cloud()
        msg.fields[0].datatype = 2
        module.native_cloud_evidence(msg)
