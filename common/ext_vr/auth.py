"""Driver-owned password/session protection for browser pairing management."""

from __future__ import annotations
import hashlib
import hmac
import json
import os
import secrets
import time
from pathlib import Path


class PairingAdmin:
    COOKIE = "motus_pico_admin"

    def __init__(self, state_dir, origin, *, clock=time.monotonic):
        self.path = Path(state_dir) / "pairing-admin.json"
        self.origin, self.clock = origin, clock
        self.sessions = {}

    @property
    def configured(self):
        return self.path.is_file() and not self.path.is_symlink()

    def set_password(self, password):
        if not isinstance(password, str) or not 12 <= len(password) <= 256:
            raise ValueError("pairing_password_requires_12_to_256_characters")
        salt = secrets.token_bytes(24)
        digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, 300000)
        data = {"salt": salt.hex(), "digest": digest.hex(), "iterations": 300000}
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        temporary = self.path.with_suffix(".tmp")
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump(data, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path)
        self.sessions.clear()

    def login(self, password, origin):
        if origin != self.origin or not self.configured:
            raise PermissionError("pairing_management_unavailable")
        if not isinstance(password, str) or len(password) > 256:
            raise PermissionError("pairing_authentication_failed")
        data = json.loads(self.path.read_text())
        actual = hashlib.pbkdf2_hmac(
            "sha256", password.encode(), bytes.fromhex(data["salt"]), data["iterations"]
        )
        if not hmac.compare_digest(actual, bytes.fromhex(data["digest"])):
            raise PermissionError("pairing_authentication_failed")
        now = self.clock()
        self.sessions = {k: v for k, v in self.sessions.items() if v["expires"] > now}
        if len(self.sessions) >= 16:
            self.sessions.pop(next(iter(self.sessions)))
        token, csrf = secrets.token_urlsafe(32), secrets.token_urlsafe(32)
        self.sessions[token] = {"csrf": csrf, "expires": now + 900}
        return token, csrf

    def authorize(self, token, csrf, origin):
        value = self.sessions.get(token or "")
        if (
            origin != self.origin
            or not value
            or value["expires"] <= self.clock()
            or not isinstance(csrf, str)
            or not hmac.compare_digest(value["csrf"], csrf)
        ):
            raise PermissionError("pairing_authentication_required")
