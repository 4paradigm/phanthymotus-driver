"""Huya-only URL validation and protected, atomic local credential storage."""
from __future__ import annotations

import os
import secrets
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
    """Private atomic storage; no symlink traversal for the credential filename.

    Linux: hold a verified 0700 directory FD and perform file operations relative
    to that FD, so path replacement cannot redirect a privileged write.
    Windows is supported for local tests only, not as a production secret store.
    """

    def __init__(self, path: str | Path):
        self.path = Path(path)

    def _open_directory(self, *, create: bool) -> int:
        if create:
            self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            meta = os.fstat(fd)
            if (not stat.S_ISDIR(meta.st_mode) or meta.st_uid != os.geteuid()
                    or meta.st_mode & 0o077):
                raise OSError("credential directory must be owner-only (0700)")
        except BaseException:
            os.close(fd)
            raise
        return fd

    def read(self) -> str | None:
        if os.name == "nt":
            try:
                if self.path.is_symlink():
                    return None
                return validate_huya_url(self.path.read_text(encoding="utf-8"))
            except (OSError, ValueError, UnicodeError):
                return None
        try:
            directory = self._open_directory(create=False)
        except OSError:
            return None
        try:
            fd = os.open(self.path.name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=directory)
            try:
                meta = os.fstat(fd)
                if not stat.S_ISREG(meta.st_mode) or meta.st_size > MAX_URL + 16 or meta.st_mode & 0o077:
                    return None
                return validate_huya_url(os.read(fd, MAX_URL + 16).decode("utf-8"))
            finally:
                os.close(fd)
        except (OSError, UnicodeError, ValueError):
            return None
        finally:
            os.close(directory)

    def save(self, raw: object) -> None:
        url = validate_huya_url(raw)
        if os.name == "nt":
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.is_symlink():
                raise OSError("credential path is a symlink")
            temp: str | None = None
            try:
                fd, temp = tempfile.mkstemp(prefix=".huya-", dir=self.path.parent)
                with os.fdopen(fd, "w", encoding="utf-8") as output:
                    output.write(url)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(temp, self.path)
            finally:
                if temp and os.path.exists(temp):
                    os.unlink(temp)
            return

        directory = self._open_directory(create=True)
        name = ".huya-" + secrets.token_hex(12)
        try:
            try:
                entry = os.stat(self.path.name, dir_fd=directory, follow_symlinks=False)
                if stat.S_ISLNK(entry.st_mode) or not stat.S_ISREG(entry.st_mode):
                    raise OSError("credential target must be a regular file")
            except FileNotFoundError:
                pass
            fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                         0o600, dir_fd=directory)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as output:
                    output.write(url)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(name, self.path.name,
                           src_dir_fd=directory, dst_dir_fd=directory)
                os.fsync(directory)
            finally:
                try:
                    os.unlink(name, dir_fd=directory)
                except FileNotFoundError:
                    pass
        finally:
            os.close(directory)
