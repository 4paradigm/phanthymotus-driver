"""Bounded, ROS-free projection of original cloud XYZ into optical-Z depth.

The zlib payload is always 640x480, little-endian uint16 millimetres, with
zero marking unobserved pixels. The projection step never interpolates.
The bounded hole-filled output is explicitly marked as estimated;
no RGB synchronization or motion compensation is assumed.
"""
from collections.abc import Mapping
import json
import math
import struct
import zlib

from deep_robotics.s10_pro import cpu_kernels


WIDTH = 640
HEIGHT = 480
MAX_SOURCE_BYTES = 64 * 1024 * 1024
MAX_SOURCE_POINTS = 2_000_000
_TRANSFORM_TOLERANCE = 1e-4


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f'{label} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{label} must be a finite number')
    return result


def _integer(value, label, minimum=0, maximum=MAX_SOURCE_BYTES):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f'{label} must be an integer in [{minimum}, {maximum}]')
    return value


def validate_transform(value):
    """Validate a right-handed rigid camera_optical_from_cloud 4x4 matrix."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (ValueError, RecursionError) as exc:
            raise ValueError('camera_optical_from_cloud must be a JSON 4x4 matrix') from exc
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in value)):
        raise ValueError('camera_optical_from_cloud must be an explicit 4x4 matrix')
    matrix = [[_number(entry, 'transform entry') for entry in row] for row in value]
    tol = _TRANSFORM_TOLERANCE
    if any(abs(actual - expected) > tol for actual, expected in zip(matrix[3], (0, 0, 0, 1))):
        raise ValueError('transform bottom row must be [0, 0, 0, 1]')
    r = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(r[i][k] * r[j][k] for k in range(3))
            if not math.isfinite(dot) or abs(dot - (1 if i == j else 0)) > tol:
                raise ValueError('transform rotation must be orthonormal')
    determinant = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                   - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                   + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    if abs(determinant - 1) > tol:
        raise ValueError('transform rotation must be right-handed (determinant +1)')
    return matrix


def _intrinsics(value):
    if not isinstance(value, (list, tuple)) or len(value) != 9:
        raise ValueError('K must contain nine finite pinhole entries')
    k = [_number(entry, 'K entry') for entry in value]
    if k[0] <= 0 or k[4] <= 0 or any(k[i] != 0 for i in (1, 3, 6, 7)) or k[8] != 1:
        raise ValueError('K must be a pinhole matrix with positive focal lengths')
    return k


def depth_intrinsics(calibration):
    """Scale calibration K to depth size, preserving pixel-centre convention."""
    if not isinstance(calibration, Mapping):
        raise ValueError('calibration must be an object with width, height and K')
    width = _integer(calibration.get('width'), 'calibration width', 1)
    height = _integer(calibration.get('height'), 'calibration height', 1)
    k = _intrinsics(calibration.get('K'))
    if not 0 <= k[2] < width or not 0 <= k[5] < height:
        raise ValueError('calibration principal point must be within its image')
    sx, sy = WIDTH / width, HEIGHT / height
    return _intrinsics([k[0] * sx, 0, (k[2] + .5) * sx - .5,
                        0, k[4] * sy, (k[5] + .5) * sy - .5, 0, 0, 1])


def validate_cloud_layout(cloud):
    """Validate bounded metadata/buffer layout without reading or copying points."""
    if not isinstance(cloud, Mapping):
        raise ValueError('cloud must be an object')
    width = _integer(cloud.get('width'), 'cloud width', maximum=MAX_SOURCE_POINTS)
    height = _integer(cloud.get('height'), 'cloud height', maximum=MAX_SOURCE_POINTS)
    total = width * height
    if total > MAX_SOURCE_POINTS:
        raise ValueError('cloud exceeds the source point limit')
    point_step = _integer(cloud.get('point_step'), 'point_step', 1)
    row_step = _integer(cloud.get('row_step'), 'row_step')
    required = row_step * height
    if row_step < width * point_step or required > MAX_SOURCE_BYTES:
        raise ValueError('cloud row layout is invalid or exceeds the source byte limit')
    data = cloud.get('data')
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ValueError('cloud data must be bytes')
    try:
        data = memoryview(data).cast('B')
    except (TypeError, ValueError) as exc:
        raise ValueError('cloud data must be a contiguous byte buffer') from exc
    if not required <= len(data) <= MAX_SOURCE_BYTES:
        raise ValueError('cloud data is truncated or exceeds the source byte limit')
    bigendian = cloud.get('is_bigendian')
    if not isinstance(bigendian, bool):
        raise ValueError('is_bigendian must be a boolean')
    fields = cloud.get('fields')
    if not isinstance(fields, (list, tuple)) or len(fields) > 256:
        raise ValueError('cloud fields must be a bounded list')
    xyz = {}
    for field in fields:
        if isinstance(field, Mapping):
            name, offset, datatype, count = (field.get(key) for key in ('name', 'offset', 'datatype', 'count'))
        elif isinstance(field, (list, tuple)) and len(field) == 4:
            name, offset, datatype, count = field
        else:
            raise ValueError('each cloud field must contain name, offset, datatype, count')
        if not isinstance(name, str):
            raise ValueError('cloud field name must be a string')
        if name not in ('x', 'y', 'z'):
            continue
        if name in xyz:
            raise ValueError(f'duplicate cloud {name} field')
        offset = _integer(offset, f'{name} offset')
        datatype = _integer(datatype, f'{name} datatype', 7, 8)
        _integer(count, f'{name} count', 1, 1)
        scalar = struct.Struct(('>' if bigendian else '<') + ('f' if datatype == 7 else 'd'))
        if offset + scalar.size > point_step:
            raise ValueError(f'cloud {name} field extends beyond point_step')
        xyz[name] = (offset, scalar)
    if set(xyz) != {'x', 'y', 'z'}:
        raise ValueError('cloud requires FLOAT32/FLOAT64 count-one x, y and z fields')
    return width, total, point_step, row_step, data, [xyz[name] for name in ('x', 'y', 'z')]


def project_depth(cloud, K, camera_optical_from_cloud, *, min_depth_m=0.1,
                  max_depth_m=50.0, max_points=200000):
    raw, stats = project_depth_mm(cloud, K, camera_optical_from_cloud, min_depth_m=min_depth_m,
                                  max_depth_m=max_depth_m, max_points=max_points)
    return zlib.compress(raw, level=1), stats


def project_depth_mm(cloud, K, camera_optical_from_cloud, *, min_depth_m=0.1,
                     max_depth_m=50.0, max_points=200000):
    """Project a bounded uniformly sampled cloud; return uint16 LE bytes and counts.

    Source indices include both endpoints when sampling two or more points.
    Z-buffer comparisons use unquantized optical Z, not radial distance.
    Positive depths quantize to nearest millimetre, clamped to 1..65535.
    """
    import numpy as np

    minimum = _number(min_depth_m, 'min_depth_m')
    maximum = _number(max_depth_m, 'max_depth_m')
    if minimum < 0 or maximum <= minimum:
        raise ValueError('depth range requires 0 <= min_depth_m < max_depth_m')
    max_points = _integer(max_points, 'max_points', 1, MAX_SOURCE_POINTS)
    k = _intrinsics(K)
    matrix = validate_transform(camera_optical_from_cloud)
    width, total, point_step, row_step, data, fields = validate_cloud_layout(cloud)
    sampled = min(total, max_points)
    # Allocation happens only after all input/resource validation succeeds.
    zbuffer = np.full(WIDTH * HEIGHT, np.inf, dtype=np.float64)
    if sampled:
        indices = (np.arange(sampled, dtype=np.int64) * (total - 1) // (sampled - 1)
                   if sampled > 1 else np.zeros(1, dtype=np.int64))
        rows, columns = divmod(indices, width)
        xyz = [np.ndarray((cloud['height'], width), dtype=np.dtype(scalar.format),
                          buffer=data, offset=offset, strides=(row_step, point_step)
                          )[rows, columns].astype(np.float64)
               for offset, scalar in fields]
        x, y, z = xyz
        # Keep float64 and the scalar operation order for optical-Z and ties.
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            xc, yc, zc = [r[0] * x + r[1] * y + r[2] * z + r[3] for r in matrix[:3]]
            valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                     & np.isfinite(xc) & np.isfinite(yc) & np.isfinite(zc)
                     & (zc > 0) & (zc >= minimum) & (zc <= maximum))
            u, v = k[0] * (xc[valid] / zc[valid]) + k[2], k[4] * (yc[valid] / zc[valid]) + k[5]
            visible = (np.isfinite(u) & np.isfinite(v) & (u >= -.5) & (u < WIDTH - .5)
                       & (v >= -.5) & (v < HEIGHT - .5))
            pixels = (np.floor(v[visible] + .5).astype(np.int64) * WIDTH
                      + np.floor(u[visible] + .5).astype(np.int64))
            np.minimum.at(zbuffer, pixels, zc[valid][visible])
    observed = np.isfinite(zbuffer)
    valid_pixels = int(np.count_nonzero(observed))
    output = np.where(observed, np.clip(np.floor(np.minimum(zbuffer, 65.535) * 1000 + .5),
                                       1, 65535), 0).astype('<u2')
    return output.tobytes(), {
        'source_points': total, 'sample_points': sampled, 'valid_pixels': valid_pixels,
    }

# Synthetic depth is not a measurement or evidence of observed free space.
HOLE_FILL_MAX_GAP_PX = 16
HOLE_FILL_MAX_JUMP_MM = 150
HOLE_FILL_RELATIVE_JUMP = 0.06


def fill_depth_holes(packet):
    filled, stats = fill_depth_holes_mm(zlib.decompress(packet))
    return zlib.compress(filled, level=1), stats


def fill_depth_holes_mm(raw):
    """Fill bounded 2-D patches; preserve originals and reject depth boundaries.

    Two fixed paths (rows then columns, columns then rows) interpolate between
    observations or segments bracketed by observations. Every stage carries
    the depth range of ALL its original supports. No iterative growth,
    extrapolation, temporal reuse or RGB alignment is assumed.
    """
    import numpy as np

    if len(raw) != WIDTH * HEIGHT * 2:
        raise ValueError("depth packet is not 640x480 uint16")
    if cpu_kernels.AVAILABLE:
        return cpu_kernels.fill_holes(raw, HOLE_FILL_MAX_GAP_PX,
                                      HOLE_FILL_MAX_JUMP_MM, HOLE_FILL_RELATIVE_JUMP)
    source = np.frombuffer(raw, dtype='<u2').reshape(HEIGHT, WIDTH).astype(np.int32)

    def bridge(state, axis):
        depth, low, high, rejected = (np.swapaxes(a, axis, 1) for a in state)
        size = depth.shape[1]
        index = np.arange(size, dtype=np.int32)[None, :]
        known = depth != 0
        left = np.maximum.accumulate(np.where(known, index, -1), axis=1)
        right = np.minimum.accumulate(np.where(known, index, size)[:, ::-1], axis=1)[:, ::-1]
        bracketed = (~known & (left >= 0) & (right < size)
                     & (right - left - 1 <= HOLE_FILL_MAX_GAP_PX))
        rows, columns = np.nonzero(bracketed & ~rejected)
        if not len(rows):
            return state
        left, right = left[rows, columns], right[rows, columns]
        first, last = depth[rows, left], depth[rows, right]
        minimum = np.minimum(low[rows, left], low[rows, right])
        maximum = np.maximum(high[rows, left], high[rows, right])
        consistent = ((maximum - minimum <= HOLE_FILL_MAX_JUMP_MM)
                      & (maximum - minimum <= HOLE_FILL_RELATIVE_JUMP * minimum))
        # int32 also covers 65535-mm inputs and the full 640-pixel row span.
        span = np.maximum(1, right - left)
        estimate = (first * (right - columns) + last * (columns - left) + span // 2) // span
        result = tuple(a.copy() for a in (depth, low, high, rejected))
        valid_rows, valid_columns = rows[consistent], columns[consistent]
        for target, values in zip(result[:3], (estimate, minimum, maximum)):
            target[valid_rows, valid_columns] = values[consistent]
        result[3][rows[~consistent], columns[~consistent]] = True
        return tuple(np.swapaxes(a, axis, 1) for a in result)

    original = (source, source, source, np.zeros(source.shape, dtype=bool))
    horizontal = bridge(bridge(original, 1), 0)
    vertical = bridge(bridge(original, 0), 1)
    a, b = horizontal[0], vertical[0]
    both = (a != 0) & (b != 0)
    disagreement = both & (np.abs(a - b) > np.minimum(60, 0.02 * np.minimum(a, b)))
    minimum = np.minimum(horizontal[1], vertical[1])
    maximum = np.maximum(horizontal[2], vertical[2])
    support_conflict = both & ((maximum - minimum > HOLE_FILL_MAX_JUMP_MM)
                              | (maximum - minimum > HOLE_FILL_RELATIVE_JUMP * minimum))
    # A path which crosses an observed depth boundary vetoes the other path;
    # otherwise orthogonal filling could paint across a thin foreground return.
    candidate = (source == 0) & ((a != 0) | (b != 0))
    conflict = candidate & (horizontal[3] | vertical[3] | disagreement | support_conflict)
    filled = np.where(both, (a + b + 1) // 2, np.maximum(a, b))
    filled[conflict] = 0
    filled[source != 0] = source[source != 0]
    return filled.astype('<u2').tobytes(), {
        'filled_pixels': int(np.count_nonzero((source == 0) & (filled != 0))),
        'fill_conflicts': int(np.count_nonzero(conflict)),
    }
