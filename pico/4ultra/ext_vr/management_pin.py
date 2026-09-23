"""Local MCP provisions a PIN; HTTPS management uses short-lived sessions."""
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import stat
import tempfile
import threading
import time

from .capture import CaptureError


class ManagementPin:
    COOKIE = "__Host-motus-management"
    SESSION_SECONDS = 900
    MAX_FAILURES = 5
    LOCK_SECONDS = 300

    def __init__(self, path, clock=time.monotonic):
        self.path, self.clock = Path(path), clock
        self._lock = threading.RLock()
        self._record = self._read_record()
        if self._record is not None:
            if (not isinstance(self._record, dict)
                    or set(self._record) != {"salt", "digest"}
                    or not all(isinstance(v, str) for v in self._record.values())
                    or len(bytes.fromhex(self._record["salt"])) != 16
                    or len(bytes.fromhex(self._record["digest"])) != 32):
                raise ValueError("management_pin_state_invalid")
        self._sessions = {}
        self._failures = 0
        self._blocked_until = 0

    def _check_directory(self):
        metadata = self.path.parent.lstat()
        if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o700:
            raise ValueError("management_pin_directory_must_be_private_0700")

    def _read_record(self):
        if self.path.parent.exists() or self.path.parent.is_symlink():
            self._check_directory()
        try:
            metadata = self.path.lstat()
        except FileNotFoundError:
            return None
        if not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o600:
            raise ValueError("management_pin_file_must_be_regular_0600")
        # Check the opened object too; never follow a replacement symlink or
        # block on a FIFO inserted between lstat and open.
        fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "r") as stream:
            opened = os.fstat(stream.fileno())
            if not stat.S_ISREG(opened.st_mode) or stat.S_IMODE(opened.st_mode) != 0o600:
                raise ValueError("management_pin_file_must_be_regular_0600")
            if opened.st_size > 1024:
                raise ValueError("management_pin_state_invalid")
            return json.load(stream)

    @property
    def configured(self):
        return self._record is not None

    @staticmethod
    def validate(pin):
        if not isinstance(pin, str) or len(pin) != 4 or any(c not in "0123456789" for c in pin):
            raise ValueError("management_pin_must_be_four_digits")

    @staticmethod
    def _digest(pin, salt):
        return hashlib.pbkdf2_hmac("sha256", pin.encode(), bytes.fromhex(salt), 100_000).hex()

    def configure(self, pin):
        """Only called by the existing local MCP configuration entry."""
        self.validate(pin)
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            existing = self._read_record()
            if existing == self._record and self._record and hmac.compare_digest(
                    self._record["digest"], self._digest(pin, self._record["salt"])):
                return False
            salt = secrets.token_hex(16)
            record = {"salt": salt, "digest": self._digest(pin, salt)}
            fd, name = tempfile.mkstemp(prefix=".management-pin-", suffix=".tmp", dir=self.path.parent)
            temporary = Path(name)
            try:
                with os.fdopen(fd, "w") as stream:
                    json.dump(record, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                temporary.unlink(missing_ok=True)
            self._record = record
            self._sessions.clear()
            self._failures = 0
            self._blocked_until = 0
            return True

    def login(self, pin):
        with self._lock:
            if not self.configured:
                raise CaptureError("management_pin_not_configured", status=503)
            now = self.clock()
            if now < self._blocked_until:
                raise CaptureError("management_rate_limited", status=429)
            if self._blocked_until:
                self._failures = 0
                self._blocked_until = 0
            valid = isinstance(pin, str) and len(pin) == 4 and all(c in "0123456789" for c in pin)
            if not valid or not hmac.compare_digest(
                    self._record["digest"], self._digest(pin, self._record["salt"])):
                self._failures += 1
                if self._failures >= self.MAX_FAILURES:
                    self._blocked_until = now + self.LOCK_SECONDS
                    raise CaptureError("management_rate_limited", status=429)
                raise CaptureError("management_pin_invalid", status=403)
            self._failures = 0
            self._sessions = {k: v for k, v in self._sessions.items() if v > now}
            if len(self._sessions) >= 8:
                del self._sessions[min(self._sessions, key=self._sessions.get)]
            token = secrets.token_urlsafe(32)
            self._sessions[hashlib.sha256(token.encode()).hexdigest()] = now + self.SESSION_SECONDS
            return token

    def authorize(self, token):
        with self._lock:
            if not self.configured:
                raise CaptureError("management_pin_not_configured", status=503)
            if not isinstance(token, str) or not 1 <= len(token) <= 128:
                raise CaptureError("management_pin_required", status=401)
            key = hashlib.sha256(token.encode()).hexdigest()
            if self._sessions.get(key, 0) <= self.clock():
                self._sessions.pop(key, None)
                raise CaptureError("management_session_expired", status=401)

    def logout(self, token):
        with self._lock:
            if isinstance(token, str):
                self._sessions.pop(hashlib.sha256(token.encode()).hexdigest(), None)
