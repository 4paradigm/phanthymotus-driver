"""Metric completion calibrated only against original LiDAR observations."""
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro.depth_fusion import fuse_depth


def packet(values):
    return zlib.compress(np.asarray(values, dtype='<u2').tobytes())


def scene():
    y, x = np.indices((480, 640))
    mono = (1000 + 2 * x + y).astype(np.uint16)
    raw = np.zeros_like(mono)
    raw[40:440:8, 40:600:8] = mono[40:440:8, 40:600:8] * 2
    return raw, mono


def test_fusion_completes_a_metric_surface_and_preserves_original_and_local_fill():
    raw, mono = scene()
    local = raw.copy()
    local[101, 101] = 2600
    output, stats = fuse_depth(packet(raw), packet(local), packet(mono))
    result = np.frombuffer(zlib.decompress(output), dtype='<u2').reshape(raw.shape)
    assert np.array_equal(result[raw != 0], raw[raw != 0])
    assert result[101, 101] == 2600
    assert result[200, 201] == 3204
    assert np.all(result > 0)
    assert stats['output_coverage'] == 1.0
    assert stats['monocular_pixels'] == int(np.count_nonzero(local == 0))


def test_interpolated_values_cannot_change_the_calibration_fit():
    raw, mono = scene()
    local = np.full_like(raw, 19000)
    local[raw != 0] = raw[raw != 0]
    _, first = fuse_depth(packet(raw), packet(raw), packet(mono))
    _, second = fuse_depth(packet(raw), packet(local), packet(mono))
    assert first['calibration_a'] == second['calibration_a']
    assert first['calibration_b'] == second['calibration_b']
    assert first['calibration_median_relative_error'] == second['calibration_median_relative_error']


@pytest.mark.parametrize('exponent,offset', [(1.25, -.3), (2., .1)])
def test_robust_log_fit_recovers_nonunit_scale(exponent, offset):
    raw, mono = scene()
    measured = np.rint(np.exp(exponent * np.log(mono.astype(float) / 1000) + offset) * 1000)
    raw[raw != 0] = measured[raw != 0].astype(np.uint16)
    _, stats = fuse_depth(packet(raw), packet(raw), packet(mono))
    assert stats['calibration_a'] == pytest.approx(exponent, abs=.002)
    assert stats['calibration_b'] == pytest.approx(offset, abs=.002)


def test_missing_lidar_references_cannot_be_replaced_by_a_guessed_scale():
    raw, mono = scene()
    with pytest.raises(ValueError, match='reference'):
        fuse_depth(packet(np.zeros_like(raw)), packet(raw), packet(mono))


def test_bad_correspondences_are_rejected_before_overwriting_with_measured_pixels():
    raw, mono = scene()
    y, x = np.indices(raw.shape)
    raw[raw != 0] = np.where((x[raw != 0] // 8 + y[raw != 0] // 8) % 2, 1000, 9000)
    with pytest.raises(ValueError, match='calibration'):
        fuse_depth(packet(raw), packet(raw), packet(mono))


def test_fusion_keeps_invalid_model_pixels_unknown_without_lidar_support():
    raw, mono = scene()
    mono[0, 0] = 0
    output, _ = fuse_depth(packet(raw), packet(raw), packet(mono))
    assert np.frombuffer(zlib.decompress(output), dtype='<u2')[0] == 0


def test_fusion_rejects_mismatched_local_projection():
    raw, mono = scene()
    local = raw.copy()
    local[40, 40] += 10
    with pytest.raises(ValueError, match='original'):
        fuse_depth(packet(raw), packet(local), packet(mono))


def test_fusion_refuses_wrong_packet_dimensions():
    raw, mono = scene()
    with pytest.raises(ValueError, match='640x480'):
        fuse_depth(zlib.compress(b'bad'), packet(raw), packet(mono))


def test_fusion_does_not_generate_depth_below_the_configured_minimum():
    raw, mono = scene()
    mono[0, 0] = 20
    output, _ = fuse_depth(packet(raw), packet(raw), packet(mono))
    assert np.frombuffer(zlib.decompress(output), dtype='<u2')[0] == 0


def test_near_original_measurements_remain_usable_calibration_references():
    raw, mono = scene()
    mono[:] = 150
    raw[raw != 0] = 300
    output, stats = fuse_depth(packet(raw), packet(raw), packet(mono))
    assert np.all(np.frombuffer(zlib.decompress(output), dtype='<u2') == 300)
    assert stats['calibration_reference_pixels'] == int(np.count_nonzero(raw))
