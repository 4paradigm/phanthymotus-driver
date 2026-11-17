"""Build-time retargeting keeps checked weights and a consistent CPU graph."""
import hashlib
import io
import sys
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro import monocular_depth as model


def tiny_model(height, width, shapes):
    onnx = pytest.importorskip('onnx')
    names = ['/model.10/m/m.0/attn/Constant_output_0',
             '/model.10/m/m.0/attn/Constant_3_output_0',
             '/model.23/Concat_output_0', '/model.23/Concat_1_output_0']
    graph = onnx.helper.make_graph([
        onnx.helper.make_node('ReduceMean', ['images'], ['depth'], axes=[1], keepdims=1)],
        'fixture', [onnx.helper.make_tensor_value_info('images', onnx.TensorProto.FLOAT,
                                                    [1, 3, height, width])],
        [onnx.helper.make_tensor_value_info('depth', onnx.TensorProto.FLOAT,
                                           [1, 1, height, width])],
        initializer=[onnx.numpy_helper.from_array(np.array(value, dtype=np.int64), name)
                     for name, value in zip(names, shapes)])
    graph.initializer.append(onnx.numpy_helper.from_array(np.array([1., 2., 3.], np.float32),
                                                        'unchanged_weights'))
    result = onnx.helper.make_model(graph, opset_imports=[onnx.helper.make_opsetid('', 13)])
    result.ir_version = 10
    result.metadata_props.add(key='imgsz', value=str([height, width]))
    return result.SerializeToString()


def test_cpu_model_retargets_spatial_dimensions_and_runs_without_build_tools(tmp_path, monkeypatch):
    original = tiny_model(768, 768, [[1, 2, 128, 576], [1, 128, 24, 24],
                                    [1, 256, 48, 48], [1, 256, 96, 96]])
    expected = tiny_model(192, 320, [[1, 2, 128, 60], [1, 128, 6, 10],
                                    [1, 256, 12, 20], [1, 256, 24, 40]])
    path = tmp_path / 'model.onnx'
    path.write_bytes(original)
    monkeypatch.setattr(model, 'MODEL_PATH', path)
    monkeypatch.setattr(model, 'MODEL_SHA256', hashlib.sha256(original).hexdigest())
    monkeypatch.setattr(model, 'CPU_MODEL_SHA256', hashlib.sha256(expected).hexdigest(), raising=False)
    model.prepare_cpu_model()
    assert path.read_bytes() == expected
    pytest.importorskip('onnxruntime')
    Image = pytest.importorskip('PIL.Image')
    monkeypatch.setitem(sys.modules, 'onnx', None)
    runtime = model.MonocularDepth(path)
    image = io.BytesIO()
    Image.new('RGB', (640, 360), (255, 0, 0)).save(image, format='PNG')
    output = np.frombuffer(zlib.decompress(runtime.predict(image.getvalue())), dtype='<u2')
    assert output.size == 640 * 480
    assert np.all(output == 333)


def test_cpu_preparation_rejects_a_changed_source_without_replacing_it(tmp_path, monkeypatch):
    path = tmp_path / 'model.onnx'
    path.write_bytes(b'wrong source')
    monkeypatch.setattr(model, 'MODEL_PATH', path)
    with pytest.raises(ValueError, match='checksum'):
        model.prepare_cpu_model()
    assert path.read_bytes() == b'wrong source'
