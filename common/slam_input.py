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
    # Apply the *same* stricter rigid-mount contract as Perception. Passing
    # this structural check is NOT physical calibration or SLAM admission.
    from common.spatial_slam import SpatialSlamError, validate_mount
    try:
        validate_mount({
            'parent_frame': out['parent_frame'],
            'child_frame': out['child_frame'],
            'source': out['source'], 'version': out['version'],
            'matrix': out['matrix'], 'calibrated': False,
        })
    except SpatialSlamError as error:
        raise ValueError('slam_extrinsic_candidate.' + str(error)) from error
    out['calibrated'] = False
    out['verified'] = False
    return out

def calibrated_urdf_mount(urdf_path, parent_frame, child_frame, *,
                          urdf_parent_frame=None, urdf_child_frame=None):
    """Return a user-verified G1 URDF fixed-chain mount (T_parent_child).

    The physical installation represented by g1_model.urdf was confirmed
    by the operator. Only named links joined by fixed URDF joints qualify;
    never replace an unrecognized native odometry/lidar frame with a guess.
    Optional URDF link aliases represent explicit operator frame matching.
    No claim about clock synchronization or odometry tracking is made.
    """
    from collections import deque
    from pathlib import Path
    from xml.etree import ElementTree
    import math

    def text_frame(value, key):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(key + ' must be a named frame')
        return value.strip()

    actual_parent = text_frame(parent_frame, 'parent_frame')
    actual_child = text_frame(child_frame, 'child_frame')
    model_parent = text_frame(urdf_parent_frame or actual_parent, 'urdf_parent_frame')
    model_child = text_frame(urdf_child_frame or actual_child, 'urdf_child_frame')
    if actual_parent == actual_child and model_parent != model_child:
        raise ValueError('identical native frames cannot be assigned different URDF links')
    if actual_parent != actual_child and model_parent == model_child:
        raise ValueError('distinct native frames cannot be assigned one URDF link')

    xml_bytes = Path(urdf_path).read_bytes()
    root = ElementTree.fromstring(xml_bytes)
    if root.tag != 'robot':
        raise ValueError('expected a URDF robot root')
    links = {link.get('name') for link in root.findall('link')}
    if model_parent not in links or model_child not in links:
        raise ValueError('URDF does not contain both native frame links; explicit alias is required')

    def identity():
        return [[1., 0., 0., 0.], [0., 1., 0., 0.],
                [0., 0., 1., 0.], [0., 0., 0., 1.]]

    def multiply(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(4))
                 for j in range(4)] for i in range(4)]

    def inverse(t):
        result = identity()
        for i in range(3):
            for j in range(3):
                result[i][j] = t[j][i]
            result[i][3] = -sum(result[i][j] * t[j][3] for j in range(3))
        return result

    def origin_matrix(node):
        origin = node.find('origin')
        xyz = ((origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
               .split())
        rpy = ((origin.get('rpy', '0 0 0') if origin is not None else '0 0 0')
               .split())
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError('URDF joint origin must have three xyz and rpy values')
        values = [float(x) for x in xyz + rpy]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('URDF joint origin contains nonfinite coordinates')
        x, y, z, roll, pitch, yaw = values
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return [
            [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, x],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, y],
            [-sp, cp*sr, cp*cr, z],
            [0., 0., 0., 1.],
        ]

    fixed = {}
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent = joint.find('parent')
        child = joint.find('child')
        a = parent.get('link') if parent is not None else None
        b = child.get('link') if child is not None else None
        if a not in links or b not in links or a == b:
            raise ValueError('invalid URDF fixed joint link')
        t = origin_matrix(joint)
        fixed.setdefault(a, []).append((b, t))
        fixed.setdefault(b, []).append((a, inverse(t)))

    queue = deque([(model_parent, identity())])
    visited = {model_parent}
    found = None
    while queue:
        frame, t = queue.popleft()
        if frame == model_child:
            found = t
            break
        for name, step in fixed.get(frame, ()):
            if name not in visited:
                visited.add(name)
                queue.append((name, multiply(t, step)))
    if found is None:
        raise ValueError('URDF links are not joined by a fixed chain')

    mount = {
        'parent_frame': actual_parent, 'child_frame': actual_child,
        'source': 'g1_model.urdf:operator_confirmed',
        'version': 'sha256:' + hashlib.sha256(xml_bytes).hexdigest(),
        'mode': 'static', 'matrix': found, 'calibrated': True,
        'calibration_evidence': 'operator_confirmed_prior_g1_test',
        'urdf_frames': {'parent': model_parent, 'child': model_child},
    }
    # Use the normative spatial contract, identical to PR #371's validator.
    from common.spatial_slam import validate_mount
    validate_mount(mount, parent_frame=actual_parent,
                   child_frame=actual_child, require_calibrated=True)
    return mount


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
