"""Observable depth wire geometry; all fixtures have hand-derived pixels/depths."""
import importlib
import json
import math
import struct
import zlib

import pytest


IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
K = [100, 0, 320, 0, 100, 240, 0, 0, 1]


@pytest.fixture
def projection():
    return importlib.import_module('deep_robotics.s10_pro.depth_projection')


def cloud(points, *, bigendian=False, float64=False, width=None, padding=0):
    fmt = ('>' if bigendian else '<') + ('ddd' if float64 else 'fff')
    step = struct.calcsize(fmt)
    width = len(points) if width is None else width
    height = len(points) // width if width else 1
    data = b''.join(b''.join(struct.pack(fmt, *p) for p in points[r * width:(r + 1) * width])
                    + b'\xff' * padding for r in range(height))
    scalar = 8 if float64 else 4
    return {'fields': [(name, offset * scalar, 8 if float64 else 7, 1)
                       for offset, name in enumerate('xyz')],
            'is_bigendian': bigendian, 'width': width, 'height': height,
            'point_step': step, 'row_step': width * step + padding, 'data': data}


def decode(result):
    packed, stats = result
    raw = zlib.decompress(packed)
    assert len(raw) == 640 * 480 * 2
    return struct.unpack('<307200H', raw), stats


def test_known_points_keep_optical_z_in_millimetres_and_wire_shape(projection):
    depths, stats = decode(projection.project_depth(cloud([(0, 0, 2), (1, 0, 2)]), K, IDENTITY))
    assert depths[240 * 640 + 320] == 2000
    assert depths[240 * 640 + 370] == 2000
    assert sum(value != 0 for value in depths) == 2
    assert stats == {'source_points': 2, 'sample_points': 2, 'valid_pixels': 2}


def test_transforms_original_xyz_without_display_axis_flip(projection):
    # Rotate cloud +X into optical +Z and cloud +Z into optical -X, translate X.
    transform = [[0, 0, -1, 1], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
    depths, _ = decode(projection.project_depth(cloud([(2, 1, 1)]), K, transform))
    assert depths[290 * 640 + 320] == 2000
    assert sum(value != 0 for value in depths) == 1


def test_same_pixel_uses_nearest_z_even_when_radial_range_is_larger(projection):
    # Adjacent continuous coordinates round to the same pixel, but the lower-Z
    # point has the longer radial range. A radial z-buffer chooses incorrectly.
    points = [(2.0049, 0, 1), (1.997, 0, 1.001)]
    depths, stats = decode(projection.project_depth(cloud(points, float64=True), K, IDENTITY))
    assert depths[240 * 640 + 520] == 1000
    assert stats['valid_pixels'] == 1


def test_rejects_invalid_behind_out_of_range_and_off_image_points(projection):
    points = [(math.nan, 0, 1), (0, math.inf, 1), (0, 0, -2), (0, 0, 0),
              (0, 0, .09), (0, 0, 51), (100, 0, 1), (0, -100, 1), (0, 0, 2)]
    depths, stats = decode(projection.project_depth(cloud(points), K, IDENTITY))
    assert depths[240 * 640 + 320] == 2000
    assert sum(value != 0 for value in depths) == 1
    assert stats['sample_points'] == 9


def test_big_endian_float64_and_organized_row_padding(projection):
    points = [(0, 0, 1), (1, 0, 2), (0, 1, 4), (-1, 0, 2)]
    depths, _ = decode(projection.project_depth(
        cloud(points, bigendian=True, float64=True, width=2, padding=11), K, IDENTITY))
    assert [depths[v * 640 + u] for u, v in [(320, 240), (370, 240), (320, 265), (270, 240)]] == [1000, 2000, 4000, 2000]


def test_rounds_pixels_and_mm_to_nearest_and_reserves_zero(projection):
    depths, _ = decode(projection.project_depth(
        cloud([(.005, .005, 1), (0, 0, .0001), (-.01, 0, 1.2345)], float64=True),
        K, IDENTITY, min_depth_m=0, max_depth_m=70))
    assert depths[241 * 640 + 321] == 1000
    assert depths[240 * 640 + 320] == 1
    assert depths[240 * 640 + 319] == 1235


def test_depth_larger_than_uint16_is_clipped_without_wrapping(projection):
    depths, _ = decode(projection.project_depth(cloud([(0, 0, 70)]), K, IDENTITY, max_depth_m=100))
    assert depths[240 * 640 + 320] == 65535


def test_empty_cloud_produces_valid_all_invalid_wire_image(projection):
    depths, stats = decode(projection.project_depth(cloud([]), K, IDENTITY))
    assert not any(depths)
    assert stats == {'source_points': 0, 'sample_points': 0, 'valid_pixels': 0}


def test_uniform_sampling_is_bounded_and_spans_source(projection):
    depths, stats = decode(projection.project_depth(
        cloud([(i / 10, 0, 1) for i in range(9)]), K, IDENTITY, max_points=3))
    assert stats == {'source_points': 9, 'sample_points': 3, 'valid_pixels': 3}
    occupied = [i % 640 for i, value in enumerate(depths) if value]
    assert occupied == [320, 360, 400]


def test_accepts_mapping_field_descriptors_and_extra_fields(projection):
    msg = cloud([(0, 0, 2)])
    msg['fields'] = [dict(zip(('name', 'offset', 'datatype', 'count'), field)) for field in msg['fields']]
    msg['fields'].append({'name': 'intensity', 'offset': 0, 'datatype': 7, 'count': 1})
    depths, _ = decode(projection.project_depth(msg, K, IDENTITY))
    assert depths[240 * 640 + 320] == 2000


@pytest.mark.parametrize('updates', [
    {'width': -1}, {'width': True}, {'height': 1.0}, {'point_step': 0},
    {'row_step': 11}, {'data': b'\x00' * 11}, {'data': 'not bytes'},
    {'is_bigendian': 1}, {'width': 2_000_001, 'row_step': 24_000_012},
    {'height': 6_000_000, 'row_step': 12},
    {'data': b'\0' * (64 * 1024 * 1024 + 1)},
    {'fields': [('x', 0, 7, 1), ('y', 4, 7, 1)]},
    {'fields': [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 2)]},
    {'fields': [('x', 0, 2, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]},
    {'fields': [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 10, 7, 1)]},
    {'fields': [('x', 0, 7, 1), ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]},
])
def test_rejects_malformed_or_unbounded_cloud_before_projection(projection, updates):
    msg = dict(cloud([(0, 0, 1)]), **updates)
    with pytest.raises(ValueError):
        projection.project_depth(msg, K, IDENTITY)


@pytest.mark.parametrize('kwargs', [
    {'min_depth_m': -1}, {'max_depth_m': math.inf}, {'max_depth_m': .05},
    {'min_depth_m': True}, {'max_points': 0}, {'max_points': True},
    {'max_points': 2_000_001}, {'max_points': 1.5},
])
def test_rejects_invalid_projection_limits(projection, kwargs):
    with pytest.raises(ValueError):
        projection.project_depth(cloud([]), K, IDENTITY, **kwargs)


def test_transform_accepts_json_and_returns_a_numeric_copy(projection):
    value = projection.validate_transform(json.dumps(IDENTITY))
    assert value == IDENTITY
    value[0][0] = 4
    assert IDENTITY[0][0] == 1


@pytest.mark.parametrize('matrix', [
    None, 'bad json', [], [[1] * 4] * 4,
    [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[1, 0, 0, math.nan], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[True, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, .01, 1]],
])
def test_rejects_missing_reflected_nonfinite_or_nonrigid_transform(projection, matrix):
    with pytest.raises(ValueError):
        projection.validate_transform(matrix)
    with pytest.raises(ValueError):
        projection.project_depth(cloud([]), K, matrix)


def test_scales_intrinsics_with_pixel_centres(projection):
    cal = {'width': 640, 'height': 360, 'K': [300, 0, 320, 0, 315, 180, 0, 0, 1]}
    assert projection.depth_intrinsics(cal) == pytest.approx(
        [300, 0, 320, 0, 420, 240 + 1 / 6, 0, 0, 1])
    assert cal['K'] == [300, 0, 320, 0, 315, 180, 0, 0, 1]


@pytest.mark.parametrize('updates', [
    {'width': 0}, {'height': True}, {'width': 640.0}, {'K': [1]},
    {'K': [0, 0, 320, 0, 100, 240, 0, 0, 1]},
    {'K': [100, 1, 320, 0, 100, 240, 0, 0, 1]},
    {'K': [100, 0, 320, 0, math.inf, 240, 0, 0, 1]},
    {'K': [100, 0, 320, 0, 100, 240, 0, 1, 1]},
    {'K': [True, 0, 320, 0, 100, 240, 0, 0, 1]},
])
def test_rejects_invalid_calibration(projection, updates):
    with pytest.raises(ValueError):
        projection.depth_intrinsics(dict(width=640, height=480, K=K, **{}) | updates)


def test_rejects_invalid_output_intrinsics(projection):
    with pytest.raises(ValueError):
        projection.project_depth(cloud([]), [1], IDENTITY)


def test_large_finite_depth_saturates_uint16_without_numeric_overflow(projection):
    depths, _ = decode(projection.project_depth(
        cloud([(0, 0, 1e308)], float64=True), K, IDENTITY, max_depth_m=1e308))
    assert depths[240 * 640 + 320] == 65535


def test_mixed_float_fields_use_declared_offsets_not_xyz_order(projection):
    msg = {'width': 1, 'height': 1, 'point_step': 24, 'row_step': 24,
           'is_bigendian': False,
           'fields': [('z', 0, 8, 1), ('intensity', 8, 7, 1), ('y', 12, 7, 1), ('x', 16, 8, 1)],
           'data': struct.pack('<dffd', 2, 999, 1, -1)}
    depths, _ = decode(projection.project_depth(msg, K, IDENTITY))
    assert depths[290 * 640 + 270] == 2000


def test_pixel_boundary_rounding_handles_half_pixel_edges(projection):
    depths, _ = decode(projection.project_depth(
        cloud([(-3.205, 0, 1), (3.195, 0, 1), (3.194, 0, 1)], float64=True), K, IDENTITY))
    assert depths[240 * 640] == 1000
    assert depths[240 * 640 + 639] == 1000
    assert sum(value != 0 for value in depths) == 2

def _hole_packet(points):
    from array import array
    data = array('H', [0]) * (640 * 480)
    for x, y, millimetres in points:
        data[y * 640 + x] = millimetres
    return zlib.compress(data.tobytes())


def _hole_map(packet):
    return struct.unpack('<307200H', zlib.decompress(packet))


def test_hole_fill_short_bracketed_surface_interpolates_without_changing_measurements(projection):
    packet = _hole_packet([(318, 240, 2000), (323, 240, 2100),
                           (320, 238, 2050), (320, 242, 2050)])
    filled, stats = projection.fill_depth_holes(packet)
    raw, result = _hole_map(packet), _hole_map(filled)
    assert result[240 * 640 + 318] == raw[240 * 640 + 318] == 2000
    assert result[240 * 640 + 323] == raw[240 * 640 + 323] == 2100
    assert result[240 * 640 + 319] > 0
    assert result[239 * 640 + 320] > 0
    assert raw[240 * 640 + 319] == 0
    assert result[240 * 640 + 310] == 0
    assert stats["filled_pixels"] >= 5


def test_hole_fill_preserves_occlusions_large_gaps_and_unseen_regions(projection):
    packet = _hole_packet([(310, 240, 1000), (330, 240, 1000),
                           (300, 250, 1000), (303, 250, 2000),
                           (320, 260, 2000)])
    filled, stats = projection.fill_depth_holes(packet)
    result = _hole_map(filled)
    assert result[240 * 640 + 320] == 0   # Unobserved gap is too wide
    assert result[250 * 640 + 301] == 0   # Sharp depth discontinuity
    assert result[260 * 640 + 321] == 0   # No extrapolation
    assert stats["filled_pixels"] == 0


def test_hole_fill_rejects_disagreeing_orthogonal_estimates(projection):
    packet = _hole_packet([(319, 240, 1000), (321, 240, 1000),
                           (320, 239, 1050), (320, 241, 1050)])
    filled, stats = projection.fill_depth_holes(packet)
    result = _hole_map(filled)
    assert result[240 * 640 + 320] == 0
    assert stats == {"filled_pixels": 0, "fill_conflicts": 1}
    assert result[240 * 640 + 319] == 1000


def test_hole_fill_empty_and_invalid_packets(projection):
    packet = _hole_packet([])
    output, stats = projection.fill_depth_holes(packet)
    assert output == packet or _hole_map(output) == _hole_map(packet)
    assert stats == {"filled_pixels": 0, "fill_conflicts": 0}
    with pytest.raises((zlib.error, ValueError)):
        projection.fill_depth_holes(zlib.compress(b"too short"))


def test_hole_fill_covers_surface_interior_without_growing_outside_support(projection):
    # Four measured corners enclose a 13x13 constant-depth patch. Independent
    # scanline interpolation only draws its perimeter, leaving a black interior.
    packet = _hole_packet([(310, 234, 2000), (322, 234, 2000),
                           (310, 246, 2000), (322, 246, 2000)])
    result, stats = decode(projection.fill_depth_holes(packet))
    assert all(result[y * 640 + x] == 2000
               for y in range(234, 247) for x in range(310, 323))
    assert stats == {"filled_pixels": 165, "fill_conflicts": 0}
    assert sum(value != 0 for value in result) == 169


def test_hole_fill_interpolates_sloped_surface_in_two_dimensions(projection):
    packet = _hole_packet([(310, 234, 2000), (322, 234, 2048),
                           (310, 246, 2048), (322, 246, 2096)])
    result, _ = decode(projection.fill_depth_holes(packet))
    assert result[240 * 640 + 316] == 2048
    assert result[237 * 640 + 313] == 2024
    assert result[246 * 640 + 322] == 2096


@pytest.mark.parametrize("corners", [
    (2000, 2000, 3000, 3000),  # A depth boundary must not become a surface.
    (2000, 2100, 2100, 2200),  # Each edge passes; all four originals disagree.
    (60000, 60000, 61000, 61000),  # uint16 arithmetic must not wrap.
])
def test_hole_fill_checks_all_original_support_depths(projection, corners):
    points = [(310, 234, corners[0]), (322, 234, corners[1]),
              (310, 246, corners[2]), (322, 246, corners[3])]
    packet = _hole_packet(points)
    result, _ = decode(projection.fill_depth_holes(packet))
    assert result[240 * 640 + 316] == 0
    for x, y, value in points:
        assert result[y * 640 + x] == value


def test_hole_fill_retains_thin_measured_foreground_inside_surface(projection):
    points = [(310, 234, 2000), (322, 234, 2000),
              (310, 246, 2000), (322, 246, 2000), (316, 240, 400)]
    result, _ = decode(projection.fill_depth_holes(_hole_packet(points)))
    assert result[240 * 640 + 316] == 400
    assert result[239 * 640 + 316] == 0
    assert result[240 * 640 + 315] == 0


def test_hole_fill_does_not_recurse_across_wide_surface_gap(projection):
    packet = _hole_packet([(310, 230, 2000), (322, 230, 2000),
                           (310, 250, 2000), (322, 250, 2000)])
    result, _ = decode(projection.fill_depth_holes(packet))
    assert result[240 * 640 + 316] == 0
    assert result[229 * 640 + 316] == 0


def test_hole_fill_checks_union_of_both_paths_original_supports(projection):
    # Predictions nearly agree, but their combined original supports span
    # 230mm: checking only the two predicted values would hide that boundary.
    packet = _hole_packet([(304, 240, 2000), (321, 240, 2110),
                           (320, 239, 2110), (320, 256, 2230)])
    result, _ = decode(projection.fill_depth_holes(packet))
    assert result[240 * 640 + 320] == 0
