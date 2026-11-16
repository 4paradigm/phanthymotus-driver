"""Huya-only URL validation and restricted local credential persistence."""
from __future__ import annotations

import os
import stat
import tempfile
from pathlib import Path
from urllib.parse import urlsplit

MAX_URL = 4096


def validate_huya_url(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("请填写虎牙完整 RTMP 地址")
    value = value.strip()
    if not value or len(value) > MAX_URL or any(ord(ch) < 32 for ch in value):
        raise ValueError("推流地址为空、过长或包含控制字符")
    try:
        parts = urlsplit(value)
        host = (parts.hostname or "").lower()
        allowed = host == "tx.direct.huya.com" or host.endswith(".direct.huya.com")
        if not (parts.scheme in ("rtmp", "rtmps") and allowed
                and parts.port in (None, 1935, 443)
                and not parts.username and not parts.password
                and parts.path.startswith("/huyalive/")
                and len(parts.path) > len("/huyalive/")
                and not parts.fragment):
            raise ValueError
    except ValueError:
        raise ValueError("仅支持虎牙官方远程推流完整地址") from None
    return value


class HuyaCredentialStore:
    """Never exposes the saved credential in card status or logs."""

    def __init__(self, path: str | Path):
        self.path = Path(path)

    def read(self) -> str | None:
        try:
            fd = os.open(self.path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
        except (OSError, ValueError):
            return None
        try:
            meta = os.fstat(fd)
            if (not stat.S_ISREG(meta.st_mode) or meta.st_size > MAX_URL + 16
                    or (os.name != "nt" and meta.st_mode & 0o077)):
                return None
            return validate_huya_url(os.read(fd, MAX_URL + 16).decode("utf-8"))
        except (OSError, UnicodeError, ValueError):
            return None
        finally:
            os.close(fd)

    def save(self, raw: object) -> None:
        url = validate_huya_url(raw)
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.path.is_symlink():
            raise OSError("credential path is a symlink")
        temp: str | None = None
        try:
            fd, temp = tempfile.mkstemp(prefix=".huya-", dir=self.path.parent)
            with os.fdopen(fd, "w", encoding="utf-8") as output:
                if hasattr(os, "fchmod"):
                    os.fchmod(output.fileno(), 0o600)
                output.write(url)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temp, self.path)
        finally:
            if temp and os.path.exists(temp):
                os.unlink(temp)
