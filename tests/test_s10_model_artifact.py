"""Bounded model download over real HTTP sockets and child processes."""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import select
import ssl
import subprocess
import sys
import threading
import time
import urllib.error

import pytest


@pytest.mark.parametrize('status,payload,hold_headers,disconnects,partial_body', [
    (200, b'model', True, 0, False), (200, b'model', False, 0, False),
    (200, b'corrupt', False, 0, False), (503, b'', True, 0, False),
    (200, b'model', True, 2, False), (200, b'model', True, 3, False),
    (200, b'model', True, 2, True),
])
def test_model_download_heartbeats_while_blocked_and_preserves_result(
        tmp_path, status, payload, hold_headers, disconnects, partial_body):
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
            interrupted = len(requests) <= disconnects
            if interrupted and not partial_body:
                self.close_connection = True
                return
            self.send_response(status)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            if not hold_headers:
                release.wait(30)
            self.wfile.write(payload[:2] if interrupted else payload)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    model_path = tmp_path / 'model.onnx'
    script = f'''
import sys, time, traceback
from pathlib import Path
from deep_robotics.s10_pro import model_artifact as model
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


def test_absolute_deadline_kills_and_reaps_a_slow_transfer(monkeypatch, tmp_path):
    from deep_robotics.s10_pro import model_artifact as model

    stop = threading.Event()
    requests, children = [], []
    payload = b'x' * 200

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            requests.append(self.path)
            self.send_response(200)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                for byte in payload:
                    self.wfile.write(bytes([byte]))
                    self.wfile.flush()
                    if stop.wait(.02):
                        break
            except ConnectionError:
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    real_popen = subprocess.Popen

    def spawn(*args, **kwargs):
        child = real_popen(*args, **kwargs)
        children.append(child)
        return child

    monkeypatch.setattr(subprocess, 'Popen', spawn)
    monkeypatch.setattr(model, 'DOWNLOAD_ATTEMPT_TIMEOUT_SECONDS', 1., raising=False)
    monkeypatch.setattr(model, 'DOWNLOAD_RETRY_DELAY_SECONDS', .01)
    monkeypatch.setattr(model, 'MODEL_PATH', tmp_path / 'model.onnx')
    monkeypatch.setattr(model, 'MODEL_SHA256', hashlib.sha256(payload).hexdigest())
    started = time.monotonic()
    try:
        with pytest.raises(TimeoutError, match='deadline'):
            model.download_model(f'http://127.0.0.1:{server.server_port}/slow')
        assert time.monotonic() - started < 6
        assert len(requests) == len(children) == 3
        assert all(child.poll() is not None for child in children)
        assert not model.MODEL_PATH.exists()
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


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
        result = subprocess.run([sys.executable, '-m', 'deep_robotics.s10_pro.model_artifact',
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


def test_certificate_verification_failure_is_permanent(monkeypatch, capsys):
    from deep_robotics.s10_pro import model_artifact as model

    failure = urllib.error.URLError(ssl.SSLCertVerificationError('untrusted certificate'))
    def urlopen(*args, **kwargs):
        raise failure
    monkeypatch.setattr(model.urllib.request, 'urlopen', urlopen)
    assert model._fetch_worker('https://invalid.test/model') == 4
    assert 'SSLCertVerificationError' in capsys.readouterr().err


def test_official_asset_protocol_follows_redirect_and_requests_binary(monkeypatch, tmp_path):
    from deep_robotics.s10_pro import model_artifact as model
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            requests.append((self.path, self.headers.get('Accept')))
            if self.path == '/asset':
                self.send_response(302)
                self.send_header('Location', '/binary')
                self.end_headers()
            else:
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'model')

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    monkeypatch.setattr(model, 'MODEL_SHA256', hashlib.sha256(b'model').hexdigest())
    path = tmp_path / 'source.onnx'
    try:
        model.download_model(f'http://127.0.0.1:{server.server_port}/asset', path)
        assert path.read_bytes() == b'model'
        assert requests == [('/asset', 'application/octet-stream'),
                            ('/binary', 'application/octet-stream')]
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


def test_artifact_stage_has_no_site_package_dependency():
    from deep_robotics.s10_pro import model_artifact as model
    result = subprocess.run([sys.executable, '-S', model.__file__, '--help'],
                            capture_output=True, timeout=5)
    assert result.returncode == 0
    assert b'--output' in result.stdout


