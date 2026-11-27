"""Native CPU kernels preserve the portable depth algorithms and boundaries."""
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro import cpu_kernels, depth_fusion, depth_projection

pytestmark = pytest.mark.skipif(not cpu_kernels.AVAILABLE, reason='native image build required')


@pytest.mark.parametrize('seed', [11, 47, 123])
def test_native_fill_matches_portable_support_and_occlusion_rules(monkeypatch, seed):
    random = np.random.default_rng(seed)
    source = np.where(random.random((480, 640)) < .06,
                      random.integers(2000, 2101, size=(480, 640)), 0).astype('<u2')
    source[200:250, 300] = 500
    packet = zlib.compress(source.tobytes())
    actual = depth_projection.fill_depth_holes(packet)
    monkeypatch.setattr(cpu_kernels, 'AVAILABLE', False)
    expected = depth_projection.fill_depth_holes(packet)
    assert actual == expected


def test_native_window_keeps_zero_padding_and_does_not_mix_opposite_edges():
    values = np.ones((480, 640))
    result = cpu_kernels.window_sum(values)
    assert result[0, 0] == result[-1, -1] == 81
    assert result[240, 320] == 289
    values.fill(0)
    values[0, 0] = 3
    result = cpu_kernels.window_sum(values)
    assert np.all(result[:9, :9] == 3)
    assert result[9, 9] == result[-1, -1] == 0


def test_native_window_preserves_signed_residual_sums(monkeypatch):
    values = np.random.default_rng(19).normal(size=(480, 640))
    result = cpu_kernels.window_sum(values)
    monkeypatch.setattr(cpu_kernels, 'AVAILABLE', False)
    np.testing.assert_allclose(result, depth_fusion._window_sum(values), rtol=0, atol=1e-10)
    for y, x in [(0, 0), (8, 9), (240, 320), (479, 639)]:
        expected = values[max(0, y-8):y+9, max(0, x-8):x+9].sum()
        assert result[y, x] == pytest.approx(expected, abs=1e-10)


def test_native_wrapper_rejects_wrong_buffer_dimensions():
    with pytest.raises(ValueError, match='640x480'):
        cpu_kernels.fill_holes(b'bad', 16, 150, .06)
    with pytest.raises(ValueError, match='640x480'):
        cpu_kernels.window_sum(np.ones((2, 2)))
