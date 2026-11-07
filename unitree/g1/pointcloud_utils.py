"""Point cloud utilities for gravity alignment using IMU data."""

import numpy as np


def gravity_align_inplace(data: bytes | bytearray, point_step: int, total_points: int,
                          roll: float, pitch: float) -> bytearray:
    """Rotate point cloud xyz by -roll/-pitch to align with gravity.

    Assumes xyz are the first 3 float32 fields (offsets 0, 4, 8) in each point.

    Returns bytearray (avoids extra copy vs bytes).
    Returns original data (as bytearray) if rotation is negligible.
    """
    if total_points == 0 or (abs(roll) < 0.001 and abs(pitch) < 0.001):
        return data if isinstance(data, bytearray) else bytearray(data)

    # Build rotation matrix: Rx(-roll) * Ry(-pitch)
    cr, sr = np.cos(-roll), np.sin(-roll)
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    R = np.array([
        [cp,       0,    sp],
        [sr * sp,  cr,  -sr * cp],
        [-cr * sp, sr,   cr * cp],
    ], dtype=np.float32)

    # Extract xyz (3 x float32 = 12 bytes at start of each point)
    buf = data if isinstance(data, bytearray) else bytearray(data)
    # View as uint8, reshape to (total_points, point_step), slice first 12 bytes
    raw = np.frombuffer(buf, dtype=np.uint8).reshape(total_points, point_step)
    xyz = raw[:, :12].copy().view(np.float32).reshape(total_points, 3)

    # Apply rotation
    xyz = xyz @ R.T

    # Write back
    raw[:, :12] = xyz.view(np.uint8).reshape(total_points, 12)

    return buf


def pack_slam_cloud(msg) -> bytes:
    """Pack rows and endian-swap declared scalars without changing their bits.

    Only row padding is removed. Point padding and all field offsets survive.
    Reject overlapping fields: their endian conversion would be ambiguous.
    The caller retains the source header and field descriptors unchanged.
    """
    width, height, step, row = msg.width, msg.height, msg.point_step, msg.row_step
    if (any(type(v) is not int for v in (width, height, step, row)) or
            width < 0 or height < 0 or step <= 0 or row < width * step or
            len(msg.data) != row * height):
        raise ValueError('invalid PointCloud2 dimensions or data length')
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    spans = []
    for field in msg.fields:
        size = sizes.get(field.datatype)
        if (size is None or type(field.offset) is not int or type(field.count) is not int or
                field.offset < 0 or field.count <= 0 or field.offset + size * field.count > step):
            raise ValueError('invalid PointCloud2 field')
        end = field.offset + size * field.count
        if any(field.offset < stop and start < end for start, stop, _ in spans):
            raise ValueError('overlapping PointCloud2 fields')
        spans.append((field.offset, end, size))
    source = bytes(msg.data)
    packed = bytearray(b''.join(source[r * row:r * row + width * step] for r in range(height)))
    if msg.is_bigendian:
        # Strided views swap each scalar, including count > 1, without float conversion.
        for start, stop, size in spans:
            if size > 1:
                for offset in range(start, stop, size):
                    values = np.ndarray((width * height,), dtype=f'>u{size}',
                                        buffer=packed, offset=offset, strides=(step,))
                    values.byteswap(inplace=True)
    return bytes(packed)
