"""Native teach mount validation, also runnable on the Linux deployment host.

Standard library only: copying this preflight tool out of the driver image does
not require installing the robot SDK on the Jetson host.
"""

import json
import os
from pathlib import Path
import re
import stat
import tempfile


def readable_profile(path):
    path = Path(path)
    if not path.is_absolute():
        raise ValueError("native CSV profile requires an absolute path")
    try:
        metadata = path.stat()
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("native CSV profile must be a regular file")
        if not metadata.st_mode & 0o444 or not os.access(path, os.R_OK):
            raise ValueError("native CSV profile is not readable")
        if metadata.st_size > 65536:
            raise ValueError("native CSV profile exceeds 64 KiB")
        with path.open(encoding="utf-8") as file:
            data = json.load(file)
        if not isinstance(data, dict):
            raise ValueError("native CSV profile must contain a JSON object")
    except OSError as exc:
        raise ValueError(f"native CSV profile cannot be read: {path}: {exc.strerror}") from exc
    return path


def _mountinfo_text():
    try:
        return Path("/proc/self/mountinfo").read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError("native export requires Linux mount information to verify the controller share") from exc


def _unescape_mount(value):
    return re.sub(r"\\([0-7]{3})", lambda match: chr(int(match[1], 8)), value)


def network_mount(path):
    """Resolve the longest enclosing Linux mount, including bind mounts.

    ismount() alone cannot identify a bind mount or prove that a directory
    belongs to a network share rather than the host's local filesystem.
    """
    path = Path(path).resolve()
    candidates = []
    for line in _mountinfo_text().splitlines():
        parts = line.split(" - ", 1)
        if len(parts) != 2:
            continue
        left, right = parts[0].split(), parts[1].split()
        if len(left) < 6 or len(right) < 3:
            continue
        mountpoint = Path(_unescape_mount(left[4]))
        if path == mountpoint or mountpoint in path.parents:
            candidates.append((len(mountpoint.parts), right[0], set(left[5].split(",")), set(right[2].split(","))))
    if not candidates:
        raise RuntimeError("controller trajectory directory has no verifiable network mount")
    _, filesystem, options, super_options = max(candidates, key=lambda item: item[0])
    if filesystem not in ("cifs", "smb3", "nfs", "nfs4"):
        raise RuntimeError("controller trajectory directory must be on an existing CIFS/NFS share, not a local directory")
    if "ro" in options or "ro" in super_options or "rw" not in options:
        raise RuntimeError("controller trajectory share is mounted read-only; native export requires write access")


def writable_controller_directory(path, *, require_network_mount=True, probe=False):
    path = Path(path)
    if not path.is_absolute():
        raise ValueError("controller trajectory directory requires an absolute path")
    if not path.is_dir():
        raise ValueError("controller trajectory directory does not exist or is not a directory")
    if require_network_mount:
        network_mount(path)
    mode = path.stat().st_mode
    if not mode & 0o222 or not mode & 0o111 or not os.access(path, os.W_OK | os.X_OK):
        raise RuntimeError("controller trajectory directory is not writable/searchable")
    if probe:
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(prefix=".dobot-teach-preflight-", dir=path, delete=False) as file:
                temporary = Path(file.name)
                file.write(b"dobot-teach-write-check\n")
                file.flush()
                os.fsync(file.fileno())
            if temporary.read_bytes() != b"dobot-teach-write-check\n":
                raise RuntimeError("controller share write/readback check failed")
        except OSError as exc:
            raise RuntimeError(f"controller share write/readback check failed: {exc}") from exc
        finally:
            if temporary is not None:
                temporary.unlink()
    return path


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Check native teach host mounts without contacting robot command ports")
    parser.add_argument("--profile", default=os.environ.get("DOBOT_TEACH_PROFILE_HOST_FILE", ""))
    parser.add_argument("--controller-dir", default=os.environ.get("DOBOT_TEACH_CONTROLLER_HOST_DIR", ""))
    args = parser.parse_args()
    try:
        readable_profile(args.profile)
        writable_controller_directory(args.controller_dir, probe=True)
    except (ValueError, RuntimeError, OSError) as exc:
        parser.exit(1, f"Native teach preflight failed: {exc}\n")
    print("Native teach host profile and writable CIFS/NFS trajectory share verified; CSV/firmware compatibility still requires validation.")


if __name__ == "__main__":
    main()
