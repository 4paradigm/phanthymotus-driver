"""Fail-closed authorization and restart-safe replay protection for Bumi."""
import hashlib
import hmac
import importlib
import os
import stat
import subprocess
import sys
import time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

import pytest

from live_stream import approval

pytestmark = pytest.mark.skipif(os.name == "nt", reason="Linux protected-file contract")


@pytest.fixture
def private_approval_dir(tmp_path, monkeypatch):
    directory = tmp_path / "private"
    directory.mkdir(mode=0o700)
    directory.chmod(0o700)
    (directory / ".operator-approval-key").write_bytes(b"k" * 32)
    (directory / ".operator-approval-key").chmod(0o600)
    monkeypatch.setattr(approval, "_KEY_DIR", directory)
    return directory


def _signed(action, url, *, nonce="a" * 32, ttl=10):
    expires = str(int(time.time()) + ttl)
    message = (action + "\0" + url + "\0" + expires + "\0" + nonce).encode()
    proof = expires + "." + nonce + "." + hmac.new(
        b"k" * 32, message, hashlib.sha256
    ).hexdigest()
    return proof


def test_signed_approval_single_use_and_not_reusable(private_approval_dir):
    url = "rtmp://example.invalid/live/canary-placeholder"
    proof = _signed("config", url)
    assert not approval.verify_live_stream_action("config", url, None)
    assert not approval.verify_live_stream_action("go_live", url, proof)
    assert not approval.verify_live_stream_action("config", url + "x", proof)
    assert approval.verify_live_stream_action("config", url, proof)
    assert not approval.verify_live_stream_action("config", url, proof)
    assert not approval.verify_live_stream_action(
        "config", url, "1." + "a" * 32 + "." + "0" * 64
    )
    markers = list(private_approval_dir.glob(".operator-approval-used-*"))
    assert len(markers) == 1
    assert stat.S_IMODE(markers[0].stat().st_mode) == 0o600
    assert proof not in markers[0].name
    assert "canary-placeholder" not in markers[0].name

    (private_approval_dir / ".operator-approval-key").chmod(0o644)
    assert not approval.verify_live_stream_action(
        "config", url, _signed("config", url, nonce="b" * 32)
    )


def test_used_proof_is_rejected_after_driver_module_restart(private_approval_dir, monkeypatch):
    url = "rtmp://example.invalid/live/canary-placeholder"
    proof = _signed("go_live", url)
    assert approval.verify_live_stream_action("go_live", url, proof)
    # Re-import the verifier like a new Driver worker (no in-memory _USED state).
    importlib.reload(approval)
    monkeypatch.setattr(approval, "_KEY_DIR", private_approval_dir)
    assert not approval.verify_live_stream_action("go_live", url, proof)
    assert approval.verify_live_stream_action(
        "go_live", url, _signed("go_live", url, nonce="c" * 32)
    )


def test_used_proof_rejected_in_separate_driver_process(private_approval_dir):
    url = "rtmp://example.invalid/live/canary-placeholder"
    proof = _signed("config", url, nonce="f" * 32)
    assert approval.verify_live_stream_action("config", url, proof)
    script = (
        "import sys; from pathlib import Path; "
        "sys.path.insert(0, sys.argv[1]); "
        "from live_stream import approval; "
        "approval._KEY_DIR = Path(sys.argv[2]); "
        "proof = sys.stdin.read(); "
        "sys.exit(1 if approval.verify_live_stream_action('config', sys.argv[3], proof) else 0)"
    )
    project_module_root = Path(__file__).resolve().parents[2]
    completed = subprocess.run(
        [sys.executable, "-c", script, str(project_module_root),
         str(private_approval_dir), url],
        input=proof, text=True, capture_output=True, timeout=8,
    )
    assert completed.returncode == 0, completed.stderr


def test_atomic_exclusive_creation_rejects_concurrent_replay(private_approval_dir):
    url = "rtmp://example.invalid/live/canary-placeholder"
    proof = _signed("config", url)
    with ThreadPoolExecutor(max_workers=8) as executor:
        results = list(executor.map(
            lambda _: approval.verify_live_stream_action("config", url, proof),
            range(12),
        ))
    assert sum(results) == 1


def test_expired_records_pruned_without_touching_other_data(private_approval_dir):
    url = "rtmp://example.invalid/live/canary-placeholder"
    previous = (private_approval_dir /
                f".operator-approval-used-{int(time.time()) - 60}-{'d' * 64}")
    previous.write_bytes(b"")
    previous.chmod(0o600)
    unrelated = private_approval_dir / "keep-me"
    unrelated.write_text("unrelated", encoding="utf-8")
    assert approval.verify_live_stream_action(
        "config", url, _signed("config", url, nonce="e" * 32)
    )
    assert not previous.exists()
    assert unrelated.read_text(encoding="utf-8") == "unrelated"


def test_marker_creation_failure_denies_approval(private_approval_dir, monkeypatch):
    url = "rtmp://example.invalid/live/canary-placeholder"
    proof = _signed("config", url)
    real_open = approval.os.open

    def deny_new_marker(path, flags, *args, **kwargs):
        if isinstance(path, str) and path.startswith(".operator-approval-used-"):
            raise PermissionError("simulated read-only data directory")
        return real_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(approval.os, "open", deny_new_marker)
    assert not approval.verify_live_stream_action("config", url, proof)
