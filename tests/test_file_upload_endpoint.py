"""`POST /file/upload` — how a file reaches a driver's container at all.

Driven over a **real socket** against a real `ThreadingHTTPServer`, because the
things that broke in the equivalent agent-core work were all at that boundary:
the body being consumed by the bundle's own read before the upload branch saw
it, a `/mcp` path check 404-ing the upload first, and three different names for
"send a response". A mock of the handler would have passed all three.

Run: python3 -m pytest tests/test_file_upload_endpoint.py -q
"""
from __future__ import annotations

import io
import json
import pathlib
import re
import socket
import sys
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common import file_intake  # noqa: E402

TOKEN = 'upload-token'


def multipart(fields: dict, filename: str, payload: bytes) -> tuple[bytes, str]:
    boundary = '----motustest'
    parts = []
    for key, value in fields.items():
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{key}"\r\n\r\n'
                     f'{value}\r\n'.encode())
    parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="file"; '
                 f'filename="{filename}"\r\nContent-Type: application/octet-stream\r\n\r\n'
                 .encode() + payload + b'\r\n')
    parts.append(f'--{boundary}--\r\n'.encode())
    return b''.join(parts), f'multipart/form-data; boundary={boundary}'


@pytest.fixture
def server(tmp_path, monkeypatch):
    """A handler shaped like a bundle's: upload glue, then its own JSON-RPC."""
    monkeypatch.setenv('ACCESS_TOKEN', TOKEN)
    monkeypatch.setattr(file_intake, 'DEFAULT_BASE_DIR', str(tmp_path / 'uploads'))

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):      # keep pytest output readable
            pass

        def _send(self, status, body):  # one bundle's spelling of this
            raw = body.encode()
            self.send_response(status)
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_POST(self):
            if file_intake.serve_upload(self):
                return
            if self.path != '/mcp':
                self._send(404, json.dumps({'error': 'Not found'}))
                return
            length = int(self.headers.get('Content-Length', 0))
            rpc = json.loads(self.rfile.read(length))
            self._send(200, json.dumps({'jsonrpc': '2.0', 'id': rpc.get('id'),
                                        'result': {'ok': True}}))

    httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{httpd.server_address[1]}', tmp_path / 'uploads'
    httpd.shutdown()
    httpd.server_close()


def post(url, body, content_type, token=TOKEN):
    request = urllib.request.Request(url, data=body, method='POST')
    request.add_header('Content-Type', content_type)
    if token is not None:
        request.add_header('X-Access-Token', token)
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_a_file_lands_in_this_container_and_the_reply_names_its_own_path(server):
    base, uploads = server
    body, ctype = multipart({}, 'trajectory.jsonl', b'{"t":0}\n')
    status, payload = post(f'{base}/file/upload', body, ctype)
    assert status == 200 and payload['ok'] is True
    stored = pathlib.Path(payload['path'])
    # Absolute, inside this process's own filesystem. That is the whole point:
    # the caller passes it back verbatim as a tool argument.
    assert stored.is_absolute() and stored.is_file()
    assert stored.read_bytes() == b'{"t":0}\n'
    assert str(uploads) in payload['path'] and payload['bytes'] == 8


def test_the_bundles_own_json_rpc_still_works(server):
    """The glue returns False for anything else, so /mcp falls through."""
    base, _ = server
    status, payload = post(f'{base}/mcp', json.dumps({'id': 7, 'method': 'tools/list'}).encode(),
                           'application/json')
    assert status == 200 and payload['id'] == 7


def test_an_unknown_path_still_404s(server):
    base, _ = server
    status, _ = post(f'{base}/nope', b'{}', 'application/json')
    assert status == 404


def test_upload_is_not_404d_by_the_mcp_path_check(server):
    """Five of the fifteen bundles check the path first. The glue must precede
    it, or `/file/upload` is refused before it is ever recognised."""
    base, _ = server
    body, ctype = multipart({}, 'a.bin', b'x')
    status, _ = post(f'{base}/file/upload', body, ctype)
    assert status == 200


@pytest.mark.parametrize('token', [None, '', 'wrong'])
def test_a_bad_token_is_refused(server, token):
    base, uploads = server
    body, ctype = multipart({}, 'a.bin', b'x')
    status, payload = post(f'{base}/file/upload', body, ctype, token=token)
    assert status == 401 and payload['ok'] is False
    assert not list(uploads.rglob('*.bin')), '被拒的上传不该留下文件'


def test_a_declared_length_over_the_limit_is_refused_before_reading(server, monkeypatch):
    """Rejected on Content-Length, before a byte enters memory: a driver shares
    a 7.4 GB robot with everything else on it."""
    base, _ = server
    body, ctype = multipart({}, 'big.bin', b'x' * 1024)
    request = urllib.request.Request(f'{base}/file/upload', data=body, method='POST')
    request.add_header('Content-Type', ctype)
    request.add_header('X-Access-Token', TOKEN)
    monkeypatch.setattr(file_intake, 'DEFAULT_MAX_BYTES', 8)
    # serve_upload reads its default at call time, so patching the module is
    # enough -- no need to restart the server.
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            status = response.status
    except urllib.error.HTTPError as error:
        status = error.code
    assert status == 413


@pytest.mark.parametrize('name', [
    '../escape.bin', 'a/../../escape.bin', '/abs.bin', '..\\escape.bin',
    'C:\\x.bin', '\x00null.bin',
])
def test_a_filename_that_escapes_the_upload_directory_is_neutralised(server, name):
    """The uploader controls this string. It must not decide where the file goes."""
    base, uploads = server
    body, ctype = multipart({}, name, b'x')
    status, payload = post(f'{base}/file/upload', body, ctype)
    assert status == 200, payload
    stored = pathlib.Path(payload['path']).resolve()
    assert uploads.resolve() in stored.parents, f'{name} 落到了 {stored}'


@pytest.mark.parametrize('subdir', ['../escape', '/abs', 'a/../../..', '..\\win'])
def test_a_subdir_that_escapes_is_refused_or_neutralised(server, subdir):
    base, uploads = server
    body, ctype = multipart({'subdir': subdir}, 'a.bin', b'x')
    status, payload = post(f'{base}/file/upload', body, ctype)
    if status == 200:
        stored = pathlib.Path(payload['path']).resolve()
        assert uploads.resolve() in stored.parents, f'{subdir} 落到了 {stored}'
    else:
        assert payload['ok'] is False


def test_concurrent_uploads_do_not_collide(server):
    """Each bundle runs a ThreadingHTTPServer, so this is the real case."""
    base, _ = server
    results, lock = [], threading.Lock()

    def upload(index):
        body, ctype = multipart({}, f'f{index}.bin', f'payload-{index}'.encode())
        status, payload = post(f'{base}/file/upload', body, ctype)
        with lock:
            results.append((index, status, payload))

    threads = [threading.Thread(target=upload, args=(i,)) for i in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=20)
    assert len(results) == 8
    for index, status, payload in results:
        assert status == 200, payload
        assert pathlib.Path(payload['path']).read_bytes() == f'payload-{index}'.encode()
    assert len({p['path'] for _, _, p in results}) == 8, '并发上传互相覆盖了'


# ── every bundle is wired, and wired correctly ───────────────────────────────

BUNDLES = sorted(p for p in ROOT.glob('*/*/main.py')
                 if 'def do_POST' in p.read_text(encoding='utf-8'))


def test_every_http_bundle_is_wired():
    assert len(BUNDLES) == 15, f'找到 {len(BUNDLES)} 份有 do_POST 的 main.py'
    missing = [str(p.relative_to(ROOT)) for p in BUNDLES
               if 'file_intake.serve_upload(self)' not in p.read_text(encoding='utf-8')]
    assert not missing, f'未接入 /file/upload：{missing}'


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p: str(p.parent.relative_to(ROOT)))
def test_the_glue_precedes_the_body_read_and_any_path_check(bundle):
    """Order is the whole correctness condition.

    After `rfile.read(length)` the stream is consumed and the upload gets an
    empty body; after a path check the upload is 404'd before it is recognised.
    Both failures are silent.

    Judged on the AST, not the text: the glue's own comment quotes both of those
    expressions, and a regex over the source matches them inside the comment --
    which is how the first version of this test failed on all 15 bundles while
    the code was correct.
    """
    import ast

    handler = _handler_ast(bundle)
    glue_line = min(
        node.lineno for node in ast.walk(handler)
        if isinstance(node, ast.Attribute) and node.attr == 'serve_upload')

    reads = [node.lineno for node in ast.walk(handler)
             if isinstance(node, ast.Attribute) and node.attr == 'read'
             and isinstance(node.value, ast.Attribute) and node.value.attr == 'rfile']
    if reads:
        assert glue_line < min(reads), '读取请求体出现在 serve_upload 之前'

    # Any comparison involving `self.path`, however the bundle spells it.
    path_checks = []
    for node in ast.walk(handler):
        if not isinstance(node, ast.Compare):
            continue
        if any(isinstance(sub, ast.Attribute) and sub.attr == 'path'
               for sub in ast.walk(node)):
            path_checks.append(node.lineno)
    if path_checks:
        assert glue_line < min(path_checks), '路径检查出现在 serve_upload 之前'


def _handler_ast(path):
    import ast
    tree = ast.parse(path.read_text(encoding='utf-8'))
    handler = next((node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == 'do_POST'), None)
    assert handler is not None, path
    return handler


@pytest.mark.parametrize('bundle', BUNDLES, ids=lambda p: str(p.parent.relative_to(ROOT)))
def test_the_glue_is_self_contained(bundle):
    """The glue must bind its own name, not reach for a module global.

    Several tests in this suite lift a bundle's `make_handler` out of `main.py`
    by AST and exec only the selected definitions. A module global referenced
    from inside `do_POST` is undefined in those namespaces, and the symptom is a
    `NameError` on **every** POST -- ordinary JSON-RPC included, not just
    uploads. That is how the first version of this change broke four tests in
    two other files while its own passed.
    """
    import ast

    handler = _handler_ast(bundle)
    imported = {alias.asname or alias.name.split('.')[0]
                for node in ast.walk(handler)
                if isinstance(node, (ast.Import, ast.ImportFrom))
                for alias in node.names}
    used = {node.value.id for node in ast.walk(handler)
            if isinstance(node, ast.Attribute) and node.attr == 'serve_upload'
            and isinstance(node.value, ast.Name)}
    assert used, f'{bundle} 未接入 /file/upload'
    assert used <= imported, f'{bundle}: {used - imported} 不是 do_POST 内部导入的'
