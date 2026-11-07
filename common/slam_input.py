"""ROS-free declarations of producer facts for SLAM admission."""
import hashlib
import time


def input_declaration(topic, format, kind, source_id, **facts):
    """Unknown evidence stays explicit rather than being inferred by a consumer."""
    return {
        'schema': 'motus.slam_input/1', 'topic': topic, 'format': format,
        'kind': kind, 'source_id': source_id, 'stamp_source': None,
        'clock_id': None, 'length_unit': None, 'frame_id': None,
        'reference_point': None, 'observation_model': None,
        'cloud_semantics': None, 'timing_model': None,
        'frame_metadata_topic': None,
        'clock': {'mapping': None, 'verified': False, 'uncertainty_ns': None},
        'extrinsic': None, **facts,
    }


def native_frame_metadata(msg, *, source_generation):
    """Keep native source evidence without asserting acquisition semantics.

    A producer process generation identifies this adapter, not the robot's
    clock epoch. These packets remain inadmissible until the missing clock
    relation and measurement facts are supplied by a verified source.
    """
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec, nanosec = getattr(stamp, 'sec', None), getattr(stamp, 'nanosec', None)
    frame = getattr(header, 'frame_id', None)
    valid = (type(sec) is int and sec >= 0 and type(nanosec) is int and
             0 <= nanosec < 1_000_000_000 and sec * 1_000_000_000 + nanosec > 0)
    source_ns = sec * 1_000_000_000 + nanosec if valid else None
    return {
        'schema': 'motus.slam_frame/1', 'stamp_ns': source_ns, 'frame_id': frame,
        'source_stamp': source_ns, 'source_unit': 'ns', 'stamp_source': None,
        'source_header': {'frame_id': frame, 'stamp': {'sec': sec, 'nanosec': nanosec}},
        'source_generation': source_generation, 'clock_epoch': None,
        'clock_id': None, 'source_clock': None,
        'mapping': {'method': None, 'version': None, 'offset_ns': None,
                    'verified': False, 'uncertainty_ns': None},
        'data_sha256': hashlib.sha256(bytes(msg.data)).hexdigest(),
        'received_ns': time.time_ns(), 'received_monotonic_ns': time.monotonic_ns(),
    }
