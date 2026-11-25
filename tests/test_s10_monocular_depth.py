"""Driver-native inference geometry, without a perception service or ROS node."""
import io
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro.monocular_depth import MonocularDepth


def test_missing_model_has_an_explicit_error(tmp_path):
    with pytest.raises(ValueError, match='model.*missing'):
        MonocularDepth(tmp_path / 'absent.onnx')


def test_wrong_model_checksum_is_rejected_before_runtime_loading(tmp_path):
    path = tmp_path / 'model.onnx'
    path.write_bytes(b'wrong model')
    with pytest.raises(ValueError, match='checksum'):
        MonocularDepth(path)


def test_inference_removes_letterbox_padding_and_normalizes_rgb():
    Image = pytest.importorskip('PIL.Image')
    from types import SimpleNamespace
    seen = []

    def run(names, inputs):
        blob = inputs['images']
        seen.append(blob)
        # Padding is deliberately a very different depth from the scene.
        depth = np.full((1, 640, 640), 40, dtype=np.float32)
        depth[:, 140:500] = 2
        return [depth]

    model = object.__new__(MonocularDepth)
    model._session = SimpleNamespace(run=run)
    model._input_name = 'images'
    model._input_h = model._input_w = 640
    model._input_dtype = np.float32
    image = Image.new('RGB', (640, 360), (255, 0, 0))
    data = io.BytesIO()
    image.save(data, format='PNG')
    output = model.predict(data.getvalue())
    values = np.frombuffer(zlib.decompress(output), dtype='<u2')
    assert values.size == 640 * 480
    assert np.all(values == 2000)
    assert np.array_equal(seen[0][0, :, 200, 200], [1, 0, 0])
    assert np.allclose(seen[0][0, :, 0, 0], 114 / 255)


def test_inference_rejects_a_different_rgb_geometry():
    Image = pytest.importorskip('PIL.Image')
    model = object.__new__(MonocularDepth)
    data = io.BytesIO()
    Image.new('RGB', (320, 240)).save(data, format='PNG')
    with pytest.raises(ValueError, match='640x360'):
        model.predict(data.getvalue())


def test_metric_quantization_matches_expanded_grid_at_range_and_rounding_edges():
    Image = pytest.importorskip('PIL.Image')
    from types import SimpleNamespace

    model = object.__new__(MonocularDepth)
    model._input_h, model._input_w = 192, 320
    model._input_name, model._input_dtype = 'images', np.float32
    samples = np.array([np.nan, np.inf, -.5, .0009, .001, 1.0005, 3.0005,
                        65.535, 65.536], dtype=np.float32)
    depth = np.resize(samples, (192, 320))
    model._session = SimpleNamespace(run=lambda *_: [depth])
    jpeg = io.BytesIO()
    Image.new('RGB', (640, 360)).save(jpeg, format='JPEG')

    expanded = np.asarray(Image.fromarray(depth[6:186]).resize((640, 480), Image.Resampling.NEAREST))
    valid = np.isfinite(expanded) & (expanded >= .001) & (expanded <= 65.535)
    expected = np.rint(np.where(valid, expanded * 1000, 0)).astype('<u2')
    assert model.predict_mm(jpeg.getvalue()) == expected.tobytes()
