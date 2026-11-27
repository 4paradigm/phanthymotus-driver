"""Standalone ONNX monocular depth for the S10 front camera."""
import hashlib
import io
from pathlib import Path
import zlib

import numpy as np

from deep_robotics.s10_pro.model_artifact import MODEL_SHA256


CPU_MODEL_SHA256 = '22be352c818c7f87f63d435557714b02217873a30bd7d30487a57ce9bde0b347'
MODEL_PATH = Path(__file__).with_name('resource') / 'models' / 'yolo26n-depth.onnx'


def prepare_cpu_model(source_path=None):
    """Retarget the pinned export's spatial shapes; ONNX is build-only."""
    source = (Path(source_path) if source_path is not None else MODEL_PATH).read_bytes()
    if hashlib.sha256(source).hexdigest() != MODEL_SHA256:
        raise ValueError('monocular model source checksum mismatch')
    import onnx

    model = onnx.load_from_string(source)
    shapes = {
        '/model.10/m/m.0/attn/Constant_output_0': [1, 2, 128, 60],
        '/model.10/m/m.0/attn/Constant_3_output_0': [1, 128, 6, 10],
        '/model.23/Concat_output_0': [1, 256, 12, 20],
        '/model.23/Concat_1_output_0': [1, 256, 24, 40],
    }
    for tensor in model.graph.initializer:
        if tensor.name in shapes:
            tensor.CopyFrom(onnx.numpy_helper.from_array(
                np.array(shapes[tensor.name], dtype=np.int64), tensor.name))
    for tensor in list(model.graph.input) + list(model.graph.output):
        tensor.type.tensor_type.shape.dim[-2].dim_value = 192
        tensor.type.tensor_type.shape.dim[-1].dim_value = 320
    del model.graph.value_info[:]
    for entry in model.metadata_props:
        if entry.key == 'imgsz':
            entry.value = '[192, 320]'
    prepared = model.SerializeToString()
    onnx.checker.check_model(prepared)
    if hashlib.sha256(prepared).hexdigest() != CPU_MODEL_SHA256:
        raise ValueError('CPU model preparation checksum mismatch')
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    MODEL_PATH.write_bytes(prepared)
    print('[s10 model] checked CPU model prepared at 320x192', flush=True)


class MonocularDepth:
    def __init__(self, path=None):
        path = Path(path or MODEL_PATH)
        if not path.is_file():
            raise ValueError('monocular depth model is missing from the driver image')
        if path.stat().st_size > 64 * 1024 * 1024 or hashlib.sha256(path.read_bytes()).hexdigest() != CPU_MODEL_SHA256:
            raise ValueError('monocular depth model checksum mismatch')
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        options.add_session_config_entry('session.inter_op.allow_spinning', '0')
        self._session = ort.InferenceSession(str(path), sess_options=options,
                                            providers=['CPUExecutionProvider'])
        descriptor = self._session.get_inputs()[0]
        if (len(descriptor.shape) != 4 or descriptor.shape[:2] != [1, 3]
                or any(type(n) is not int or not 32 <= n <= 1024 for n in descriptor.shape[2:])):
            raise ValueError('unsupported monocular model image geometry')
        self._input_h, self._input_w = descriptor.shape[2:]
        self._input_name = descriptor.name
        if descriptor.type not in ('tensor(float)', 'tensor(float16)'):
            raise ValueError('unsupported monocular model input type')
        self._input_dtype = np.float16 if descriptor.type == 'tensor(float16)' else np.float32

    def predict(self, jpeg):
        return zlib.compress(self.predict_mm(jpeg), level=1)

    def predict_mm(self, jpeg):
        from PIL import Image

        if not 0 < len(jpeg) <= 2 * 1024 * 1024:
            raise ValueError('invalid front-camera JPEG size')
        with Image.open(io.BytesIO(jpeg)) as image:
            if image.size != (640, 360):
                raise ValueError('front-camera image must be rectified 640x360')
            scale = min(self._input_w / 640, self._input_h / 360)
            width, height = round(640 * scale), round(360 * scale)
            x, y = (self._input_w - width) // 2, (self._input_h - height) // 2
            canvas = Image.new('RGB', (self._input_w, self._input_h), (114, 114, 114))
            canvas.paste(image.convert('RGB').resize((width, height), Image.Resampling.BILINEAR), (x, y))
        blob = np.asarray(canvas, dtype=np.float32).transpose(2, 0, 1)[None] / 255
        outputs = self._session.run(None, {self._input_name: np.ascontiguousarray(blob, dtype=self._input_dtype)})
        depth = np.squeeze(np.asarray(outputs[0], dtype=np.float32))
        if depth.shape != (self._input_h, self._input_w):
            raise ValueError('unexpected monocular depth output geometry')
        # Nearest-neighbour resize copies samples. Quantize each model sample
        # once before expanding it onto the 640x480 output grid.
        depth = depth[y:y + height, x:x + width]
        valid = np.isfinite(depth) & (depth >= .001) & (depth <= 65.535)
        packet = np.rint(np.where(valid, depth * 1000, 0)).astype('<u2')
        return Image.fromarray(packet).resize((640, 480), Image.Resampling.NEAREST).tobytes()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--source-model', required=True)
    prepare_cpu_model(parser.parse_args().source_model)
