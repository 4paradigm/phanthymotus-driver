"""Build-only, bounded download of the pinned official depth export."""
import hashlib
import http.client
from pathlib import Path
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

MODEL_URL = 'https://api.github.com/repos/ultralytics/assets/releases/assets/471363270'
MODEL_SHA256 = '6583ec12521a7e75eacb209db63393158c7e0e772cda9fdad8c8c0fe74af2a1e'
MODEL_PATH = Path(__file__).with_name('source-model.onnx')
MAX_BYTES = 64 * 1024 * 1024
DOWNLOAD_HEARTBEAT_SECONDS = 30
DOWNLOAD_RETRY_DELAY_SECONDS = 2
DOWNLOAD_ATTEMPT_TIMEOUT_SECONDS = 180


def _fetch_worker(url):
    """Only this killable child may block in DNS, TLS, redirects or reads."""
    try:
        request = urllib.request.Request(url, headers={
            'Accept': 'application/octet-stream', 'User-Agent': 's10-pro-driver-build'})
        with urllib.request.urlopen(request, timeout=30) as response:
            expected = response.headers.get('Content-Length')
            payload = response.read(MAX_BYTES + 1)
        if len(payload) > MAX_BYTES:
            print('model download exceeds size limit', file=sys.stderr)
            return 4
        if expected is not None and len(payload) < int(expected):
            raise http.client.IncompleteRead(payload, int(expected) - len(payload))
        view = memoryview(payload)
        while view:
            count = sys.stdout.buffer.write(view)
            if not count:
                raise BrokenPipeError('model output closed')
            view = view[count:]
        return 0
    except urllib.error.HTTPError as error:
        print(f'HTTP Error {error.code}', file=sys.stderr)
        return 3 if error.code in (408, 429, 500, 502, 503, 504) else 4
    except urllib.error.URLError as error:
        print(type(error.reason).__name__, file=sys.stderr)
        return 4 if isinstance(error.reason, ssl.SSLCertVerificationError) else 3
    except Exception as error:
        print(type(error).__name__, file=sys.stderr)
        return 3 if isinstance(error, (ConnectionError, TimeoutError, http.client.IncompleteRead)) else 4


def download_model(url=None, output_path=None):
    stopped = threading.Event()
    started = time.monotonic()

    def heartbeat():
        while not stopped.wait(DOWNLOAD_HEARTBEAT_SECONDS):
            print(f'[s10 model] waiting for download/verification '
                  f'({int(time.monotonic() - started)}s elapsed)', flush=True)

    print('[s10 model] downloading yolo26n-depth.onnx; SHA-256 verification required', flush=True)
    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    try:
        for attempt in range(3):
            try:
                result = subprocess.run([sys.executable, '-S', str(Path(__file__).resolve()),
                    '--fetch-worker', MODEL_URL if url is None else url],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    timeout=DOWNLOAD_ATTEMPT_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                reason = f'absolute download deadline of {DOWNLOAD_ATTEMPT_TIMEOUT_SECONDS:g}s exceeded'
                if attempt == 2:
                    raise TimeoutError(reason) from None
            else:
                if result.returncode == 0:
                    payload = result.stdout
                    break
                reason = result.stderr.decode(errors='replace').strip()[:256] or f'worker exit {result.returncode}'
                if result.returncode != 3 or attempt == 2:
                    raise RuntimeError('model download failed: ' + reason)
            delay = DOWNLOAD_RETRY_DELAY_SECONDS * 2 ** attempt
            print(f'[s10 model] attempt {attempt + 1}/3 failed ({reason}); '
                  f'retrying in {delay:g}s', flush=True)
            time.sleep(delay)
        if len(payload) > MAX_BYTES or hashlib.sha256(payload).hexdigest() != MODEL_SHA256:
            raise ValueError('monocular model download checksum mismatch')
        path = Path(output_path) if output_path is not None else MODEL_PATH
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    finally:
        stopped.set()
        worker.join()
    print('[s10 model] download verified and saved', flush=True)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--model-url', default=MODEL_URL)
    parser.add_argument('--output', default=str(MODEL_PATH))
    parser.add_argument('--fetch-worker', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.fetch_worker is not None:
        sys.exit(_fetch_worker(args.fetch_worker))
    download_model(args.model_url, args.output)
