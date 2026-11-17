"""Standalone ONNX monocular depth for the S10 front camera."""
import hashlib
import http.client
import io
from pathlib import Path
import ssl
import threading
import time
import urllib.error
import urllib.request
import zlib

import numpy as np


MODEL_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n-depth.onnx'
MODEL_SHA256 = '6583ec12521a7e75eacb209db63393158c7e0e772cda9fdad8c8c0fe74af2a1e'
MODEL_PATH = Path(__file__).with_name('resource') / 'models' / 'yolo26n-depth.onnx'
DOWNLOAD_HEARTBEAT_SECONDS = 30
DOWNLOAD_RETRY_DELAY_SECONDS = 2


def download_model(url=None):
    """Bake a checked artifact into the driver image; never download on start."""
    stopped = threading.Event()
    started = time.monotonic()

    def heartbeat():
        while not stopped.wait(DOWNLOAD_HEARTBEAT_SECONDS):
            elapsed = int(time.monotonic() - started)
            print(f'[s10 model] waiting for download/verification ({elapsed}s elapsed)', flush=True)

    print('[s10 model] downloading yolo26n-depth.onnx; SHA-256 verification required', flush=True)
    worker = threading.Thread(target=heartbeat, name='s10-model-download-heartbeat', daemon=True)
    worker.start()
    try:
        for attempt in range(3):
            try:
                with urllib.request.urlopen(MODEL_URL if url is None else url, timeout=120) as response:
                    payload = response.read(64 * 1024 * 1024 + 1)
                break
            except (urllib.error.URLError, ConnectionError, TimeoutError, http.client.IncompleteRead) as error:
                if isinstance(error, urllib.error.HTTPError) and error.code not in (408, 429, 500, 502, 503, 504):
                    raise
                if isinstance(error, urllib.error.URLError) and isinstance(error.reason, ssl.SSLCertVerificationError):
                    raise
                if attempt == 2:
                    raise
                delay = DOWNLOAD_RETRY_DELAY_SECONDS * 2 ** attempt
                print(f'[s10 model] attempt {attempt + 1}/3 failed ({type(error).__name__}); '
                      f'retrying in {delay:g}s', flush=True)
                time.sleep(delay)
        if len(payload) > 64 * 1024 * 1024 or hashlib.sha256(payload).hexdigest() != MODEL_SHA256:
            raise ValueError('monocular model download checksum mismatch')
        MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        MODEL_PATH.write_bytes(payload)
    finally:
        stopped.set()
        worker.join()
    print('[s10 model] download verified and saved', flush=True)


class MonocularDepth:
    def __init__(self, path=None):
        path = Path(path or MODEL_PATH)
        if not path.is_file():
            raise ValueError('monocular depth model is missing from the driver image')
        if path.stat().st_size > 64 * 1024 * 1024 or hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
            raise ValueError('monocular depth model checksum mismatch')
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
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
        depth = np.asarray(Image.fromarray(depth[y:y + height, x:x + width]).resize(
            (640, 480), Image.Resampling.NEAREST))
        valid = np.isfinite(depth) & (depth >= .001) & (depth <= 65.535)
        packet = np.rint(np.where(valid, depth * 1000, 0)).astype('<u2')
        return zlib.compress(packet.tobytes(), level=1)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--model-url', default=MODEL_URL)
    download_model(parser.parse_args().model_url)
