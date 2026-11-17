"""Driver-native inference geometry, without a perception service or ROS node."""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro.monocular_depth import MonocularDepth


@pytest.mark.parametrize('status,payload,hold_headers', [
    (200, b'model', True), (200, b'model', False),
    (200, b'corrupt', False), (503, b'', True),
])
def test_model_download_heartbeats_while_blocked_and_preserves_result(
        tmp_path, status, payload, hold_headers):
    release = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if hold_headers:
                release.wait(30)
            self.send_response(status)
            self.end_headers()
            if not hold_headers:
                release.wait(30)
            self.wfile.write(payload)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    model_path = tmp_path / 'model.onnx'
    script = f'''
import sys, time, traceback
from pathlib import Path
from deep_robotics.s10_pro import monocular_depth as model
model.MODEL_URL = 'http://127.0.0.1:{server.server_port}/model'
model.MODEL_PATH = Path({str(model_path)!r})
model.MODEL_SHA256 = {hashlib.sha256(b'model').hexdigest()!r}
model.DOWNLOAD_HEARTBEAT_SECONDS = .03
try:
    model.download_model()
except Exception:
    traceback.print_exc()
    result = 1
else:
    result = 0
print('DOWNLOAD_FINISHED', flush=True)
time.sleep(.12)
sys.exit(result)
'''
    # No -u: heartbeat output must reach a piped build log without buffering.
    process = subprocess.Popen([sys.executable, '-c', script], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, cwd=Path(__file__).resolve().parents[1])
    output = b''
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and output.count(b'waiting') < 2:
            if select.select([process.stdout], [], [], .05)[0]:
                output += process.stdout.read1(4096)
        assert output.count(b'waiting') >= 2, output.decode()
        assert process.poll() is None
        assert not model_path.exists()
        release.set()
        output += process.communicate(timeout=3)[0]
        before, after = output.split(b'DOWNLOAD_FINISHED', 1)
        assert b'waiting' not in after
        if payload == b'model':
            assert process.returncode == 0
            assert model_path.read_bytes() == b'model'
        else:
            assert process.returncode != 0
            assert not model_path.exists()
            assert (b'checksum mismatch' if status == 200 else b'HTTP Error 503') in before
    finally:
        release.set()
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=3)
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


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
