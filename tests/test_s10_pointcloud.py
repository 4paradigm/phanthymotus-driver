"""PointCloud2 layout and canvas wire contracts; no ROS installation required."""

from __future__ import annotations

import array
import copy
import json
import math
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from deep_robotics.s10_pro.pointcloud import convert_pointcloud  # noqa: E402


def field(name, offset, datatype=7, count=1):
    return {"name": name, "offset": offset, "datatype": datatype, "count": count}


def cloud(points, *, bigendian=False):
    order = ">" if bigendian else "<"
    return {"width": len(points), "height": 1, "point_step": 16,
            "row_step": len(points) * 16, "is_bigendian": bigendian,
            "fields": [field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)],
            "data": b"".join(struct.pack(order + "ffff", *point) for point in points),
            "header": {"frame_id": "lidar_sensor", "stamp": {"sec": 123, "nanosec": 456}}}


def decode(buffer):
    # Independently follow the existing browser renderer's little-endian layout.
    step, count = struct.unpack_from("<II", buffer)
    assert step == 16
    assert len(buffer) == 8 + step * count
    return [struct.unpack_from("<ffff", buffer, 8 + index * step) for index in range(count)]


@pytest.mark.parametrize("bigendian", [False, True])
def test_decodes_endian_and_preserves_coordinates_and_metadata(bigendian):
    message = cloud([(1.25, -2.5, 3.75, 42.0), (-4.5, 5.25, -6.0, 7.0)], bigendian=bigendian)
    original = copy.deepcopy(message)
    buffer, meta = convert_pointcloud(message)
    assert decode(buffer) == [(1.25, -2.5, 3.75, 42.0), (-4.5, 5.25, -6.0, 7.0)]
    assert meta["point_count"] == meta["output_count"] == meta["valid_count"] == 2
    assert meta["is_bigendian"] is bigendian
    assert meta["frame_id"] == "lidar_sensor"
    assert meta["stamp"] == {"sec": 123, "nanosec": 456}
    assert json.loads(json.dumps(meta, allow_nan=False)) == meta
    assert message == original


def test_organized_rows_with_padding_and_nonstandard_field_offsets():
    # x=int16 at 2, y=float64 at 4, z=int32 at 12, intensity=uint16 at 16.
    records = []
    for index in range(4):
        record = bytearray(20)
        struct.pack_into(">hdiH", record, 2, index + 1, index + 0.5, -index, index + 100)
        records.append(bytes(record))
    message = {"width": 2, "height": 2, "point_step": 20, "row_step": 48,
               "is_bigendian": True,
               "fields": [field("z", 12, 5), field("intensity", 16, 4),
                          field("y", 4, 8), field("x", 2, 3)],
               "data": records[0] + records[1] + b"padding!" + records[2] + records[3] + b"padding!"}
    output, meta = convert_pointcloud(message)
    assert decode(output) == [(i + 1.0, i + 0.5, float(-i), i + 100.0) for i in range(4)]
    assert meta["source_point_step"] == 20
    assert meta["source_row_step"] == 48
    assert meta["frame_id"] == ""
    assert meta["stamp"] is None


@pytest.mark.parametrize("datatype,fmt,value", [(1, "b", -8), (2, "B", 250),
    (3, "h", -1234), (4, "H", 4567), (5, "i", -123456), (6, "I", 123456),
    (7, "f", 1.25), (8, "d", -4.5)])
def test_all_pointfield_datatypes_and_vector_fields(datatype, fmt, value):
    size = struct.calcsize(fmt)
    message = {"width": 1, "height": 1, "point_step": 3 * 2 * size, "row_step": 3 * 2 * size,
               "is_bigendian": False,
               "fields": [field(name, i * 2 * size, datatype, 2) for i, name in enumerate("xyz")],
               "data": struct.pack("<" + fmt * 6, value, value, value, value, value, value)}
    output, _ = convert_pointcloud(message)
    assert decode(output) == [(float(value), float(value), float(value), 0.0)]


@pytest.mark.parametrize("data_type", [bytes, bytearray, memoryview, list, tuple,
                                      lambda b: array.array("B", b)])
def test_ros_like_objects_and_byte_container_types(data_type):
    message = cloud([(1.0, 2.0, 3.0, 4.0)])
    message["data"] = data_type(message["data"])
    message["fields"] = [SimpleNamespace(**f) for f in message["fields"]]
    message["header"] = SimpleNamespace(frame_id="raw", stamp=SimpleNamespace(sec=7, nanosec=8))
    output, meta = convert_pointcloud(SimpleNamespace(**message))
    assert decode(output) == [(1.0, 2.0, 3.0, 4.0)]
    assert meta["frame_id"] == "raw" and meta["stamp"] == {"sec": 7, "nanosec": 8}


def test_uniform_sampling_over_valid_points_includes_cloud_endpoints():
    points = [(float(i), float(i), float(i), float(i)) for i in range(20)]
    points[2] = (math.nan, 0.0, 0.0, 0.0)
    points[15] = (0.0, math.inf, 0.0, 0.0)
    output, meta = convert_pointcloud(cloud(points), max_points=4)
    assert [p[0] for p in decode(output)] == [0.0, 6.0, 12.0, 19.0]
    assert meta["point_count"] == 20
    assert meta["valid_count"] == 18
    assert meta["invalid_count"] == 2
    assert meta["output_count"] == 4


def test_one_point_budget_picks_middle_instead_of_first():
    output, _ = convert_pointcloud(cloud([(float(i), 0.0, 0.0, 1.0) for i in range(9)]), 1)
    assert decode(output) == [(4.0, 0.0, 0.0, 1.0)]


def test_nonfinite_intensity_becomes_zero_but_nonfinite_xyz_are_removed():
    output, meta = convert_pointcloud(cloud([(1.0, 2.0, 3.0, math.nan),
        (2.0, 3.0, 4.0, math.inf), (math.nan, 0.0, 0.0, 1.0),
        (0.0, -math.inf, 0.0, 1.0), (0.0, 0.0, math.inf, 1.0)]))
    assert decode(output) == [(1.0, 2.0, 3.0, 0.0), (2.0, 3.0, 4.0, 0.0)]
    assert meta["invalid_count"] == 3


def test_double_values_outside_float32_range_are_handled_without_overflow():
    message = {"width": 2, "height": 1, "point_step": 32, "row_step": 64,
               "is_bigendian": False,
               "fields": [field(name, i * 8, 8) for i, name in enumerate(("x", "y", "z", "intensity"))],
               "data": struct.pack("<8d", 1e300, 0, 0, 1, 1, 2, 3, 1e300)}
    output, meta = convert_pointcloud(message)
    assert decode(output) == [(1.0, 2.0, 3.0, 0.0)]
    assert meta["invalid_count"] == 1


def test_no_valid_points_and_completely_empty_frame_are_explicit():
    output, meta = convert_pointcloud(cloud([(math.nan, 0.0, 0.0, 0.0)]))
    assert decode(output) == [] and meta["point_count"] == 1 and meta["output_count"] == 0
    empty = {"width": 0, "height": 0, "point_step": 0, "row_step": 0, "data": []}
    output, meta = convert_pointcloud(empty)
    assert output == struct.pack("<II", 16, 0)
    assert meta["point_count"] == meta["invalid_count"] == 0


@pytest.mark.parametrize("budget", [0, -1, True, 2.5, "6000", None])
def test_invalid_point_budgets_fail_explicitly(budget):
    with pytest.raises(ValueError, match="max_points"):
        convert_pointcloud(cloud([(1.0, 2.0, 3.0, 4.0)]), budget)


@pytest.mark.parametrize("data", [10**100, True, False, 16.0, "0123456789", None,
                                  object(), iter([0] * 16), [True] * 16, [1.5] * 16])
def test_data_rejects_scalars_and_non_byte_iterables_without_allocating(data):
    message = cloud([(1.0, 2.0, 3.0, 4.0)])
    message["data"] = data
    with pytest.raises(ValueError, match="uint8 bytes"):
        convert_pointcloud(message)


def test_data_does_not_invoke_arbitrary_bytes_or_index_protocol():
    class AllocationTrap:
        def __bytes__(self):
            pytest.fail("must not invoke __bytes__ on non-byte data")

        def __index__(self):
            pytest.fail("must not invoke __index__ on non-byte data")

    message = cloud([(1.0, 2.0, 3.0, 4.0)])
    message["data"] = AllocationTrap()
    with pytest.raises(ValueError, match="uint8 bytes"):
        convert_pointcloud(message)


@pytest.mark.parametrize("mutate,match", [
    (lambda m: m["fields"].pop(0), "missing coordinate fields"),
    (lambda m: m["fields"].append(field("x", 0)), "duplicate"),
    (lambda m: m["fields"][0].update(datatype=99), "unsupported"),
    (lambda m: m["fields"][0].update(offset=15), "beyond point_step"),
    (lambda m: m["fields"][0].update(count=9), "beyond point_step"),
    (lambda m: m["fields"][0].update(count=0), "count"),
    (lambda m: m["fields"][0].update(offset=-1), "offset"),
    (lambda m: m.update(row_step=15), "row_step"),
    (lambda m: m.update(data=m["data"][:-1]), "shorter"),
    (lambda m: m.update(width=1.5), "width"),
    (lambda m: m.update(height=-1), "height"),
    (lambda m: m.update(point_step=0), "point_step"),
    (lambda m: m.update(is_bigendian="false"), "is_bigendian"),
    (lambda m: m.update(data=[256]), "uint8"),
])
def test_malformed_layout_is_rejected_instead_of_misrendered(mutate, match):
    message = cloud([(1.0, 2.0, 3.0, 4.0)])
    mutate(message)
    with pytest.raises(ValueError, match=match):
        convert_pointcloud(message)
