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



def validate_extrinsic_candidate(candidate):
    """Check an operator-supplied rigid transform without asserting calibration.

    This is diagnostic evidence only. The SLAM input declaration's 'extrinsic'
    remains unknown until physical calibration and reference-frame verification
    are provided by a trusted producer. A candidate cannot satisfy admission.
    """
    import math
    if not isinstance(candidate, dict):
        raise ValueError('slam_extrinsic_candidate must be an object')
    out = {}
    for key in ('parent_frame', 'child_frame', 'source', 'version'):
        value = candidate.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError('slam_extrinsic_candidate.' + key + ' is required')
        out[key] = value.strip()
    if candidate.get('verified') is True or candidate.get('calibrated') is True:
        raise ValueError('candidate cannot assert verified or calibrated without evidence')
    matrix = candidate.get('matrix')
    if (not isinstance(matrix, (list, tuple)) or len(matrix) != 4 or
            any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in matrix)):
        raise ValueError('slam_extrinsic_candidate.matrix must be 4x4')
    rows = []
    for row in matrix:
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in row):
            raise ValueError('slam_extrinsic_candidate.matrix must be finite')
        rows.append([float(v) for v in row])
    if any(abs(rows[3][j] - (1.0 if j == 3 else 0.0)) > 1e-6 for j in range(4)):
        raise ValueError('slam_extrinsic_candidate.matrix must be homogeneous')
    columns = [[rows[r][c] for r in range(3)] for c in range(3)]
    for i in range(3):
        for j in range(3):
            dot = sum(columns[i][k] * columns[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-4:
                raise ValueError('slam_extrinsic_candidate.matrix rotation is not orthonormal')
    a, b, c = columns
    det = (a[0] * (b[1]*c[2] - b[2]*c[1]) -
           b[0] * (a[1]*c[2] - a[2]*c[1]) +
           c[0] * (a[1]*b[2] - a[2]*b[1]))
    if abs(det - 1.0) > 1e-4:
        raise ValueError('slam_extrinsic_candidate.matrix rotation is reflected')
    out['matrix'] = rows
    out['calibrated'] = False
    out['verified'] = False
    return out

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
    payload = bytes(msg.data)
    return {
        'schema': 'motus.slam_frame/1', 'stamp_ns': source_ns, 'frame_id': frame,
        'source_stamp': source_ns, 'source_unit': 'ns', 'stamp_source': None,
        'source_header': {'frame_id': frame, 'stamp': {'sec': sec, 'nanosec': nanosec}},
        'source_generation': source_generation, 'clock_epoch': None,
        'clock_id': None, 'source_clock': None,
        'mapping': {'method': None, 'version': None, 'offset_ns': None,
                    'verified': False, 'uncertainty_ns': None},
        'data_sha256': hashlib.sha256(payload).hexdigest(),
        'source_evidence': {'raw_header_present': header is not None,
                            'source_stamp_valid': valid,
                            'source_frame_valid': isinstance(frame, str) and bool(frame.strip()),
                            'payload_size_bytes': len(payload),
                            'measurement_stamp_verified': False,
                            'clock_relation_verified': False},
        'received_ns': time.time_ns(), 'received_monotonic_ns': time.monotonic_ns(),
    }
