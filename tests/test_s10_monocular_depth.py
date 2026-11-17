"""Driver-native inference geometry, without a perception service or ROS node."""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
from pathlib import Path
import select
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import zlib

import numpy as np
import pytest

from deep_robotics.s10_pro.monocular_depth import MonocularDepth


@pytest.mark.parametrize('status,payload,hold_headers,disconnects', [
    (200, b'model', True, 0), (200, b'model', False, 0),
    (200, b'corrupt', False, 0), (503, b'', True, 0),
    (200, b'model', True, 2), (200, b'model', True, 3),
])
def test_model_download_heartbeats_while_blocked_and_preserves_result(
        tmp_path, status, payload, hold_headers, disconnects):
    release = threading.Event()
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/model':
                self.send_error(404)
                return
            if hold_headers:
                release.wait(30)
            requests.append(self.path)
            if len(requests) <= disconnects:
                self.close_connection = True
                return
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
model.MODEL_URL = 'http://127.0.0.1:{server.server_port}/wrong-default'
model.MODEL_PATH = Path({str(model_path)!r})
model.MODEL_SHA256 = {hashlib.sha256(b'model').hexdigest()!r}
model.DOWNLOAD_HEARTBEAT_SECONDS = .03
model.DOWNLOAD_RETRY_DELAY_SECONDS = .01
try:
    model.download_model(url='http://127.0.0.1:{server.server_port}/model')
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
        if payload == b'model' and disconnects < 3:
            assert process.returncode == 0
            assert model_path.read_bytes() == b'model'
        else:
            assert process.returncode != 0
            assert not model_path.exists()
            if disconnects:
                assert b'RemoteDisconnected' in before
            else:
                assert (b'checksum mismatch' if status == 200 else b'HTTP Error 503') in before
        assert len(requests) == (3 if disconnects or status == 503 else 1)
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


def test_model_download_cli_uses_the_requested_artifact_url():
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            requests.append(self.path)
            self.send_error(404 if self.path == '/requested-model' else 410)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        result = subprocess.run([sys.executable, '-m', 'deep_robotics.s10_pro.monocular_depth',
            '--model-url', f'http://127.0.0.1:{server.server_port}/requested-model'],
            capture_output=True, timeout=10, cwd=Path(__file__).resolve().parents[1])
        assert result.returncode != 0
        assert b'HTTP Error 404' in result.stderr
        assert b'download verified and saved' not in result.stdout
        assert requests == ['/requested-model']
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


def test_certificate_verification_failure_is_not_retried(monkeypatch, tmp_path):
    from deep_robotics.s10_pro import monocular_depth as model

    failure = urllib.error.URLError(ssl.SSLCertVerificationError('untrusted certificate'))
    attempts = []

    def urlopen(*args, **kwargs):
        attempts.append(args)
        raise failure

    monkeypatch.setattr(model.urllib.request, 'urlopen', urlopen)
    monkeypatch.setattr(model, 'MODEL_PATH', tmp_path / 'model.onnx')
    with pytest.raises(urllib.error.URLError) as error:
        model.download_model()
    assert error.value is failure
    assert len(attempts) == 1
    assert not model.MODEL_PATH.exists()


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
