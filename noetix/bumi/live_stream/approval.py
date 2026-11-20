"""Read-only verification of explicit Core approval for a public broadcast."""
from __future__ import annotations

import hashlib
import hmac
import os
import stat
import time
import threading
from pathlib import Path

_KEY_DIR = Path(os.environ.get(
    "PHANTHY_LIVE_STREAM_AUTH_DIR", "/opt/phanthy-motus/data/live_stream"
))
_KEY_NAME = ".operator-approval-key"
_USED = {}
_USED_LOCK = threading.Lock()


def verify_live_stream_action(action: str, url: str, proof: object) -> bool:
    """Fail closed. Unauthenticated raw tools/call cannot configure or go live."""
    if action not in ("config", "go_live") or not isinstance(url, str):
        return False
    if not isinstance(proof, str) or len(proof) > 150:
        return False
    try:
        expiry_text, nonce, tag = proof.split('.')
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
        finally:
            os.close(directory)
        if len(key) != 32:
            return False
        expected = hmac.new(
            key, (action + "\x00" + url + "\x00" + expiry_text + "\x00" + nonce).encode("utf-8"),
            hashlib.sha256,
        ).hexdigest()
        if not hmac.compare_digest(expected, tag):
            return False
        # Single-use approval tokens, scoped to this Driver process.
        with _USED_LOCK:
            if len(_USED) > 512:
                for seen, expires in list(_USED.items()):
                    if expires < now:
                        _USED.pop(seen, None)
            if proof in _USED:
                return False
            _USED[proof] = expiry
        return True
    except (OSError, ValueError, UnicodeError):
        return False
