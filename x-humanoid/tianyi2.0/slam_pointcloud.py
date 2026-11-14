"""ROS-free Tianyi Orbbec PointCloud2 preservation and diagnostic provenance.

The raw PointCloud2 and its SHA256 describe the exact source bytes. This file
does NOT claim sensor measurement-time semantics, inter-device clock relation,
or a calibrated static mount. Head joints may require a dynamic transform.
"""
from __future__ import annotations

import hashlib


def native_cloud_evidence(msg, *, max_bytes=12_000_000):
    """Validate layout and return facts from a raw Orbbec PointCloud2.

    A valid layout does not certify the ROS header as the acquisition time.
    The caller must not convert/rotate/level/pad this message before publishing.
    """
    fields = getattr(msg, 'fields', None)
    if not isinstance(fields, (list, tuple)):
        raise ValueError('native cloud has no PointFields')
    width, height = int(msg.width), int(msg.height)
    step, row_step = int(msg.point_step), int(msg.row_step)
    if width <= 0 or height <= 0 or step < 12 or row_step < width * step:
        raise ValueError('native cloud has invalid row/point layout')
    required = row_step * height
    if required <= 0 or required > max_bytes:
        raise ValueError('native cloud exceeds bounded transport byte budget')
    payload = bytes(msg.data)
    if len(payload) != required:
        raise ValueError('native cloud data length does not match row_step * height')
    xyz = {f.name: f for f in fields if getattr(f, 'name', None) in ('x', 'y', 'z')}
    if set(xyz) != {'x', 'y', 'z'}:
        raise ValueError('native cloud missing XYZ fields')
    # PointField.FLOAT32 is datatype 7 on ROS 2 (not a guess from offsets).
    if any(int(f.datatype) != 7 or int(f.count) != 1 or
           int(f.offset) < 0 or int(f.offset) + 4 > step for f in xyz.values()):
        raise ValueError('native cloud XYZ must be float32 fields within point_step')
    stamp = msg.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    frame = str(msg.header.frame_id or '')
    if stamp_ns <= 0 or not frame.strip():
        raise ValueError('native cloud has zero ROS header time or empty frame_id')
    return {
        'frame_id': frame,
        'stamp_ns': stamp_ns,
        'width': width,
        'height': height,
        'point_step': step,
        'row_step': row_step,
        'is_bigendian': bool(msg.is_bigendian),
        'byte_length': required,
        'data_sha256': hashlib.sha256(payload).hexdigest(),
    }


def input_declaration(topic, metadata_topic, snapshot=None):
    """Unverified source declaration: strictly NOT admissible for SLAM."""
    snapshot = snapshot or {}
    return {
        'schema': 'motus.slam_input/1',
        'topic': topic,
        'format': 'sensor/pointcloud2',
        'kind': 'cloud',
        'source_id': 'x-humanoid/tianyi2/orbbec_head_raw',
        'reference_point': 'raw_orbbec_pointcloud_origin',
        'frame_id': snapshot.get('frame_id'),
        'clock_id': 'unknown',
        'stamp_source': 'unknown',
        'length_unit': 'm',
        'cloud_semantics': 'depth_frame',
        'observation_model': 'pinhole',
        'timing_model': 'unknown',
        'frame_metadata_topic': metadata_topic,
        'clock': {'mapping': 'unverified_ros_header_passthrough',
                  'verified': False, 'uncertainty_ns': None},
        # Not a static head mount: verified joint-aware TF is still missing.
    }


def frame_declaration(evidence, generation):
    """Hash-correlated diagnostics for the raw payload; NO clock validation."""
    return {
        'schema': 'motus.slam_frame/1',
        'stamp_ns': evidence['stamp_ns'],
        'frame_id': evidence['frame_id'],
        'clock_id': 'unknown',
        'clock_epoch': 'unknown',
        'source_generation': generation,
        'source_clock': 'unknown',
        'source_stamp': evidence['stamp_ns'],
        'source_unit': 'ns',
        'mapping': {'method': 'unverified_ros_header_passthrough',
                    'version': 'diagnostics/1', 'offset_ns': 0,
                    'verified': False, 'uncertainty_ns': None},
        'data_sha256': evidence['data_sha256'],
    }
