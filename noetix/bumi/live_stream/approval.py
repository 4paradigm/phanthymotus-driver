"""Verify Core's short-lived live-stream approvals without allowing restart replay."""
from __future__ import annotations

import hashlib
import hmac
import os
import stat
import time
from pathlib import Path

_KEY_DIR = Path(os.environ.get(
    "PHANTHY_LIVE_STREAM_AUTH_DIR", "/opt/phanthy-motus/data/live_stream"
))
_KEY_NAME = ".operator-approval-key"
_USED_PREFIX = ".operator-approval-used-"


def _consume_proof(directory: int, proof: str, expiry_text: str) -> bool:
    """Atomically consume a proof on the shared, protected persistent volume.

    O_EXCL prevents races between Driver processes; fsync makes the refusal
    survive a restart. File names contain only expiry and a hash, never RTMP
    credentials or the bearer proof itself.
    """
    digest = hashlib.sha256(proof.encode("utf-8")).hexdigest()
    marker = f"{_USED_PREFIX}{expiry_text}-{digest}"
    try:
        fd = os.open(
            marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o600, dir_fd=directory,
        )
    except FileExistsError:
        return False
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
    os.fsync(directory)
    return True


def _prune_expired(directory: int, now: int) -> None:
    """Best-effort bounded-lifetime cleanup; never touch unrelated files."""
    try:
        names = os.listdir(directory)
    except OSError:
        return
    for name in names:
        if not name.startswith(_USED_PREFIX):
            continue
        parts = name[len(_USED_PREFIX):].split("-", 1)
        if (len(parts) != 2 or not parts[0].isascii() or not parts[0].isdecimal()
                or len(parts[1]) != 64
                or any(ch not in "0123456789abcdef" for ch in parts[1])):
            continue
        if int(parts[0]) >= now:
            continue
        try:
            entry = os.stat(name, dir_fd=directory, follow_symlinks=False)
            if (stat.S_ISREG(entry.st_mode) and entry.st_uid == os.geteuid()
                    and not entry.st_mode & 0o077):
                os.unlink(name, dir_fd=directory)
        except OSError:
            pass  # Expired markers are harmless; never fail an approved action.


def verify_live_stream_action(action: str, url: str, proof: object) -> bool:
    """Fail closed. Raw MCP callers cannot configure or publicly broadcast."""
    if action not in ("config", "go_live") or not isinstance(url, str):
        return False
    if not isinstance(proof, str) or len(proof) > 150:
        return False
    try:
        expiry_text, nonce, tag = proof.split(".")
        expiry = int(expiry_text)
        now = int(time.time())
        if (not expiry_text.isdecimal() or len(nonce) != 32 or len(tag) != 64
                or expiry < now or expiry > now + 25):
            return False
        int(nonce, 16)
        int(tag, 16)
    except (ValueError, TypeError):
        return False

    try:
        directory = os.open(_KEY_DIR, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            directory_stat = os.fstat(directory)
            if (not stat.S_ISDIR(directory_stat.st_mode)
                    or directory_stat.st_uid != os.geteuid()
                    or directory_stat.st_mode & 0o077):
                return False
            fd = os.open(_KEY_NAME, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=directory)
            try:
                entry = os.fstat(fd)
                if (not stat.S_ISREG(entry.st_mode)
                        or entry.st_uid != os.geteuid()
                        or entry.st_mode & 0o077):
                    return False
                key = os.read(fd, 33)
            finally:
                os.close(fd)
            if len(key) != 32:
                return False
            expected = hmac.new(
                key, (action + "\x00" + url + "\x00" + expiry_text + "\x00" + nonce).encode("utf-8"),
                hashlib.sha256,
            ).hexdigest()
            if not hmac.compare_digest(expected, tag):
                return False
            if not _consume_proof(directory, proof, expiry_text):
                return False
            _prune_expired(directory, now)
            return True
        finally:
            os.close(directory)
    except (OSError, ValueError, UnicodeError):
        return False
