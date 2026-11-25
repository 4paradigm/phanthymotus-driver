"""Build-time download helper used by rokae/helios/Dockerfile: failover and SHA256 check."""

import hashlib
import http.server
import threading
from functools import partial

import pytest

from rokae.helios.build_fetch import main


@pytest.fixture
def server(tmp_path):
    (tmp_path / "good.bin").write_bytes(b"x" * 3_000_000)
    (tmp_path / "bad.bin").write_bytes(b"y" * 10)
    handler = partial(http.server.SimpleHTTPRequestHandler, directory=str(tmp_path))
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()


def test_falls_back_to_next_url_and_checks_sha(server, tmp_path, capsys):
    sha = hashlib.sha256(b"x" * 3_000_000).hexdigest()
    dest = tmp_path / "out.bin"
    assert main(str(dest), sha, f"{server}/missing.bin", f"{server}/bad.bin", f"{server}/good.bin") == 0
    assert dest.read_bytes() == b"x" * 3_000_000
    log = capsys.readouterr().out
    assert "failed: HTTPError" in log and "sha256 mismatch" in log and "sha256 ok" in log
    assert "MiB" in log  # progress keeps the build log alive


def test_fails_when_no_source_matches(server, tmp_path):
    assert main(str(tmp_path / "out.bin"), "0" * 64, f"{server}/bad.bin", "") == 1
