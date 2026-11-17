"""Bounded, ROS-free projection of original cloud XYZ into optical-Z depth.

The zlib payload is always 640x480, little-endian uint16 millimetres, with
zero marking unobserved pixels. The projection step never interpolates.
The bounded hole-filled output is explicitly marked as estimated;
no RGB synchronization or motion compensation is assumed.
"""
from array import array
from collections.abc import Mapping
import json
import math
import struct
import sys
import zlib


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
    """Project a bounded uniformly sampled cloud; return zlib bytes and counts.

    Source indices include both endpoints when sampling two or more points.
    Z-buffer comparisons use unquantized optical Z, not radial distance.
    Positive depths quantize to nearest millimetre, clamped to 1..65535.
    """
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
    zbuffer = array('d', [0.0]) * (WIDTH * HEIGHT)
    valid_pixels = 0
    r0, r1, r2 = matrix[:3]
    for n in range(sampled):
        index = n * (total - 1) // (sampled - 1) if sampled > 1 else 0
        row, column = divmod(index, width)
        offset = row * row_step + column * point_step
        x, y, z = (scalar.unpack_from(data, offset + field_offset)[0] for field_offset, scalar in fields)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        xc = r0[0] * x + r0[1] * y + r0[2] * z + r0[3]
        yc = r1[0] * x + r1[1] * y + r1[2] * z + r1[3]
        zc = r2[0] * x + r2[1] * y + r2[2] * z + r2[3]
        if not (math.isfinite(xc) and math.isfinite(yc) and math.isfinite(zc)):
            continue
        if zc <= 0 or zc < minimum or zc > maximum:
            continue
        u, v = k[0] * (xc / zc) + k[2], k[4] * (yc / zc) + k[5]
        if not (math.isfinite(u) and math.isfinite(v)):
            continue
        if not (-.5 <= u < WIDTH - .5 and -.5 <= v < HEIGHT - .5):
            continue
        pixel = math.floor(v + .5) * WIDTH + math.floor(u + .5)
        previous = zbuffer[pixel]
        if previous == 0:
            valid_pixels += 1
            zbuffer[pixel] = zc
        elif zc < previous:
            zbuffer[pixel] = zc
    output = array('H', (0 if z == 0 else min(65535, max(1, math.floor(min(z, 65.535) * 1000 + .5)))
                         for z in zbuffer))
    if sys.byteorder != 'little':
        output.byteswap()
    return zlib.compress(output.tobytes(), level=1), {
        'source_points': total, 'sample_points': sampled, 'valid_pixels': valid_pixels,
    }

# Only short, locally supported holes are filled. Synthetic depth is not a
# sensor measurement and must never count as measured obstacle coverage.
HOLE_FILL_MAX_GAP_PX = 8
HOLE_FILL_MAX_JUMP_MM = 150
HOLE_FILL_RELATIVE_JUMP = 0.06


def fill_depth_holes(packet):
    """Return (filled image/depth-zlib, stats); never alter an observed pixel.

    Horizontal and vertical 1-D gaps must be bounded by ORIGINAL observations.
    Bridging depth discontinuities or extrapolating from a single return can
    manufacture free space or erase thin obstacles, so neither is permitted.
    Conflicting horizontal/vertical predictions are invalidated, not averaged.
    """
    raw = zlib.decompress(packet)
    if len(raw) != WIDTH * HEIGHT * 2:
        raise ValueError("depth packet is not 640x480 uint16")
    source = array('H')
    source.frombytes(raw)
    if sys.byteorder != 'little':
        source.byteswap()
    filled = array('H', source)
    # 0: original or unknown; 1: synthetic; 2: rejected synthetic conflict
    flags = bytearray(WIDTH * HEIGHT)
    conflicts = 0

    def bridge(start, end, stride):
        nonlocal conflicts
        gap = (end - start) // stride - 1
        if gap <= 0 or gap > HOLE_FILL_MAX_GAP_PX:
            return
        first, last = source[start], source[end]
        if abs(first - last) > min(
                HOLE_FILL_MAX_JUMP_MM,
                HOLE_FILL_RELATIVE_JUMP * min(first, last)):
            return
        for step in range(1, gap + 1):
            index = start + stride * step
            estimate = (first * (gap + 1 - step) + last * step
                        + (gap + 1) // 2) // (gap + 1)
            if flags[index] == 0:
                filled[index] = estimate
                flags[index] = 1
            elif flags[index] == 1:
                if abs(filled[index] - estimate) > min(
                        60, 0.02 * min(filled[index], estimate)):
                    filled[index] = 0
                    flags[index] = 2
                    conflicts += 1
                else:
                    filled[index] = (filled[index] + estimate + 1) // 2

    # Source pixels, not pixels filled in the preceding pass, supply every
    # anchor. The interpolation can never recursively grow into unseen space.
    for row in range(HEIGHT):
        previous = None
        for col in range(WIDTH):
            index = row * WIDTH + col
            if source[index]:
                if previous is not None:
                    bridge(previous, index, 1)
                previous = index
    for col in range(WIDTH):
        previous = None
        for row in range(HEIGHT):
            index = row * WIDTH + col
            if source[index]:
                if previous is not None:
                    bridge(previous, index, WIDTH)
                previous = index

    synthetic = flags.count(1)
    if sys.byteorder != 'little':
        filled.byteswap()
    return zlib.compress(filled.tobytes(), level=1), {
        'filled_pixels': synthetic,
        'fill_conflicts': conflicts,
    }
