"""Fail-closed authorization at the raw Bumi MCP boundary."""
import hashlib
import hmac
import os
import time

import pytest

from live_stream import approval

pytestmark = pytest.mark.skipif(os.name == "nt", reason="Linux protected-file contract")


def test_signed_approval_single_use_and_not_reusable(tmp_path, monkeypatch):
    directory = tmp_path / "private"
    directory.mkdir(mode=0o700)
    directory.chmod(0o700)
    (directory / ".operator-approval-key").write_bytes(b"k" * 32)
    (directory / ".operator-approval-key").chmod(0o600)
    monkeypatch.setattr(approval, "_KEY_DIR", directory)
    approval._USED.clear()

    url = "rtmp://live-push-reg-tx.xhscdn.com/live/qa?txSecret=fake&txTime=ffffffff"
    expiry = str(int(time.time()) + 10)
    nonce = "a" * 32
    signed = ("config\0" + url + "\0" + expiry + "\0" + nonce).encode()
    proof = expiry + "." + nonce + "." + hmac.new(
        b"k" * 32, signed, hashlib.sha256
    ).hexdigest()

    assert not approval.verify_live_stream_action("config", url, None)
    assert not approval.verify_live_stream_action("go_live", url, proof)
    assert not approval.verify_live_stream_action("config", url + "x", proof)
    assert approval.verify_live_stream_action("config", url, proof)
    assert not approval.verify_live_stream_action("config", url, proof)  # replay
    assert not approval.verify_live_stream_action(
        "config", url, "1." + nonce + "." + "0" * 64
    )
    (directory / ".operator-approval-key").chmod(0o644)
    expiry_2 = str(int(time.time()) + 9)
    nonce_2 = "b" * 32
    signed_2 = ("config\0" + url + "\0" + expiry_2 + "\0" + nonce_2).encode()
    second = expiry_2 + "." + nonce_2 + "." + hmac.new(
        b"k" * 32, signed_2, hashlib.sha256
    ).hexdigest()
    assert not approval.verify_live_stream_action("config", url, second)
