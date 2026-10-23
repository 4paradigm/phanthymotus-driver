"""Read PointCloud2 layouts and encode the existing canvas point-cloud format.

This module has no ROS dependency. The canvas wire format is little-endian
``uint32 point_step, uint32 count`` followed by packed float32 XYZI records.
Source coordinates are preserved: this conversion does not claim a robot frame
or apply another model's sensor mounting transform.
"""

from __future__ import annotations

import math
import struct
from collections.abc import Mapping
from typing import Any


_FIELD_FORMATS = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}
_FLOAT32_MAX = 3.4028234663852886e38
_OUTPUT_POINT_STEP = 16
_RECORD = struct.Struct("<ffff")
_MISSING = object()


def _get(value: Any, name: str, default: Any = _MISSING) -> Any:
    if isinstance(value, Mapping):
        result = value.get(name, default)
    else:
        result = getattr(value, name, default)
    if result is _MISSING:
        raise ValueError(f"PointCloud2 is missing {name}")
    return result


def _integer(value: Any, name: str, minimum: int = 0) -> int:
    # Do not silently truncate a malformed floating-point size or accept bools.
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _stamp(header: Any) -> dict[str, int] | None:
    stamp = _get(header, "stamp", None)
    if stamp is None:
        return None
    result = {}
    for name in ("sec", "nanosec"):
        value = _get(stamp, name, None)
        if value is not None:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"header.stamp.{name} must be an integer")
            result[name] = value
    return result


def _data_view(data: Any) -> memoryview:
    if isinstance(data, (int, bool, float, str)) or data is None:
        # bytes(integer) allocates that many zero bytes; a malformed replay
        # record must not turn its numeric data field into a memory allocation.
        raise ValueError("PointCloud2 data must contain uint8 bytes")
    try:
        view = memoryview(data)
    except TypeError:
        # ROS-free replay records may carry a JSON list of uint8 values.
        # Do not invoke arbitrary __bytes__/__index__ methods or consume an
        # unbounded iterable while normalizing a malformed record.
        if not isinstance(data, (list, tuple)):
            raise ValueError("PointCloud2 data must contain uint8 bytes")
        if any(isinstance(value, bool) or not isinstance(value, int)
               or not 0 <= value <= 255 for value in data):
            raise ValueError("PointCloud2 data must contain uint8 bytes")
        try:
            view = memoryview(bytes(data))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("PointCloud2 data must contain uint8 bytes") from exc
    if not view.c_contiguous:
        view = memoryview(view.tobytes())
    try:
        return view.cast("B")
    except (TypeError, ValueError) as exc:
        raise ValueError("PointCloud2 data is not a contiguous byte buffer") from exc


def _selected_fields(fields: Any, point_step: int, endian: str):
    try:
        items = iter(fields)
    except TypeError as exc:
        raise ValueError("PointCloud2 fields must be iterable") from exc
    selected = {}
    for field in items:
        name = _get(field, "name", None)
        if name not in ("x", "y", "z", "intensity"):
            continue
        if name in selected:
            raise ValueError(f"ambiguous duplicate PointCloud2 field {name}")
        offset = _integer(_get(field, "offset"), f"{name}.offset")
        datatype = _integer(_get(field, "datatype"), f"{name}.datatype", 1)
        count = _integer(_get(field, "count", 1), f"{name}.count", 1)
        if datatype not in _FIELD_FORMATS:
            raise ValueError(f"unsupported PointCloud2 datatype {datatype} for {name}")
        unpacker = struct.Struct(endian + _FIELD_FORMATS[datatype])
        if offset + unpacker.size * count > point_step:
            raise ValueError(f"PointCloud2 field {name} extends beyond point_step")
        selected[name] = (offset, unpacker)
    missing = {"x", "y", "z"} - selected.keys()
    if missing:
        raise ValueError("PointCloud2 is missing coordinate fields: " + ", ".join(sorted(missing)))
    return selected


def convert_pointcloud(message: Any, max_points: int = 6000) -> tuple[bytes, dict]:
    """Convert a ROS PointCloud2 object or mapping without importing ROS.

    All source rows and fields are bounds-checked before reading. Sampling is
    uniform over valid points, includes both endpoints when at least two points
    are requested, and uses two passes to keep additional memory bounded by the
    output budget. Nonfinite/unrepresentable XYZ points are omitted; unavailable
    or nonfinite intensity becomes zero. A malformed layout raises ValueError.
    """
    max_points = _integer(max_points, "max_points", 1)
    width = _integer(_get(message, "width"), "width")
    height = _integer(_get(message, "height"), "height")
    point_step = _integer(_get(message, "point_step"), "point_step")
    row_step = _integer(_get(message, "row_step"), "row_step")
    is_bigendian = _get(message, "is_bigendian", False)
    if not isinstance(is_bigendian, bool):
        raise ValueError("is_bigendian must be a boolean")
    header = _get(message, "header", {})
    frame_id = _get(header, "frame_id", "")
    if not isinstance(frame_id, str):
        raise ValueError("header.frame_id must be a string")
    point_count = width * height
    metadata = {
        "point_count": point_count,
        "output_count": 0,
        "valid_count": 0,
        "invalid_count": 0,
        "frame_id": frame_id,
        "stamp": _stamp(header),
        "is_bigendian": is_bigendian,
        "source_point_step": point_step,
        "source_row_step": row_step,
        "width": width,
        "height": height,
        "output_point_step": _OUTPUT_POINT_STEP,
    }
    data = _data_view(_get(message, "data"))
    if point_count == 0:
        return struct.pack("<II", _OUTPUT_POINT_STEP, 0), metadata
    if point_step == 0:
        raise ValueError("nonempty PointCloud2 requires positive point_step")
    if row_step < width * point_step:
        raise ValueError("PointCloud2 row_step is shorter than its packed row")
    # PointCloud2 specifies data length as row_step * height, including padding
    # after the last row. Reject truncation instead of displaying partial data.
    if len(data) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step * height")
    fields = _selected_fields(_get(message, "fields"), point_step, ">" if is_bigendian else "<")
    axes = [fields[name] for name in ("x", "y", "z")]
    intensity_field = fields.get("intensity")

    def valid_points():
        for row in range(height):
            row_offset = row * row_step
            for column in range(width):
                offset = row_offset + column * point_step
                xyz = tuple(float(unpacker.unpack_from(data, offset + field_offset)[0])
                            for field_offset, unpacker in axes)
                if all(math.isfinite(value) and abs(value) <= _FLOAT32_MAX for value in xyz):
                    yield offset, xyz

    valid_count = sum(1 for _ in valid_points())
    output_count = min(valid_count, max_points)
    metadata.update(valid_count=valid_count, output_count=output_count,
                    invalid_count=point_count - valid_count)
    output = bytearray(8 + _OUTPUT_POINT_STEP * output_count)
    struct.pack_into("<II", output, 0, _OUTPUT_POINT_STEP, output_count)
    if not output_count:
        return bytes(output), metadata

    # Explicit endpoint-spanning indices avoid the first-N bias of a slice and
    # remain deterministic when invalid samples are interspersed in the cloud.
    def selected_index(index):
        if output_count == 1:
            return (valid_count - 1) // 2
        return index * (valid_count - 1) // (output_count - 1)

    output_index = 0
    target_index = selected_index(0)
    for valid_index, (offset, xyz) in enumerate(valid_points()):
        if valid_index != target_index:
            continue
        intensity = 0.0
        if intensity_field is not None:
            field_offset, unpacker = intensity_field
            value = float(unpacker.unpack_from(data, offset + field_offset)[0])
            if math.isfinite(value) and abs(value) <= _FLOAT32_MAX:
                intensity = value
        _RECORD.pack_into(output, 8 + output_index * _OUTPUT_POINT_STEP, *xyz, intensity)
        output_index += 1
        if output_index == output_count:
            break
        target_index = selected_index(output_index)
    return bytes(output), metadata
