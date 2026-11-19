"""Actual offline signing CLI under a synthetic PTY, no human presence claim."""

import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sound_facing_authority import OperatorAuthority


def terminal(command, exchanges):
    child, fd = pty.fork()
    if child == 0:
        os.execv(sys.executable, [sys.executable, "-B", *command])
    output = b""
    deadline = time.monotonic() + 15
    try:
        for prompt, response in exchanges:
            while prompt.encode() not in output:
                assert time.monotonic() < deadline, output.decode(errors="replace")
                if select.select([fd], [], [], .1)[0]:
                    try:
                        output += os.read(fd, 8192)
                    except OSError as exc:
                        raise AssertionError(output.decode(errors="replace")) from exc
            os.write(fd, response.encode() + b"\n")
        while time.monotonic() < deadline:
            if select.select([fd], [], [], .1)[0]:
                try:
                    block = os.read(fd, 8192)
                except OSError as exc:
                    if exc.errno != errno.EIO:raise
                    break
                if not block:break
                output += block
        else:
            raise AssertionError("operator CLI timed out")
        _, status = os.waitpid(child, 0)
        child = None
        assert os.waitstatus_to_exitcode(status) == 0, output.decode(errors="replace")
        return output.decode()
    finally:
        os.close(fd)
        if child is not None:
            os.kill(child, signal.SIGKILL)
            os.waitpid(child, 0)


def test_actual_operator_cli_creates_encrypted_key_and_verifiable_grant(tmp_path):
    signer = str(Path(__file__).resolve().parents[1] / "tools/sign_sound_facing_authorization.py")
    private = tmp_path / "operator.pem"
    terminal([signer, "--private-key", str(private), "--generate"], [
        ("Encrypt the new operator key with a passphrase:", "synthetic-test-passphrase"),
        ("Repeat passphrase:", "synthetic-test-passphrase")])
    assert private.stat().st_mode & 0o777 == 0o600
    assert b"ENCRYPTED PRIVATE KEY" in private.read_bytes()
    authority = OperatorAuthority("synthetic-operator-cli", private.with_suffix(".pub.pem"))
    request = tmp_path / "request.json"
    request.write_text(json.dumps(authority.snapshot()["request"]))
    output = terminal([signer, "--private-key", str(private), "--request", str(request)], [
        ("Type APPROVE:", "APPROVE"), ("Operator key passphrase:", "synthetic-test-passphrase")])
    proof = next(line for line in output.splitlines() if line.startswith('{"payload":'))
    assert authority.authorize(proof)["state"] == "approved"
    session = authority.claim()
    assert authority.deadline(session) > time.monotonic()
