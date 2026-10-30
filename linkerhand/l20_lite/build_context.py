"""Stage the ARM64 driver build context without host/runtime/SDK dependencies."""

from __future__ import annotations

import argparse
import shutil
import tempfile
from pathlib import Path

DRIVER = Path(__file__).resolve().parent
FILES = (
    "Dockerfile",
    ".dockerignore",
    "requirements.txt",
    "main.py",
    "modbus_client.py",
    "modbus_device.py",
    "gesture_device.py",
    "glove_teleop_device.py",
    "glove_direct_reader.py",
    "glove_protocol.py",
    "glove_skeleton.py",
    "glove_thumb.py",
    "check_runtime.py",
    "config.yaml",
    "driver.yaml",
)


def ignore_private_files(_directory, names):
    return {
        name
        for name in names
        if name in (".git", "__pycache__")
        or name.endswith((".pyc", ".pyo"))
        or (
            name != ".env.example"
            and (name.startswith(".env") or name.endswith(".env") or ".env." in name)
        )
    }


def stage_context(destination: Path | None = None) -> Path:
    target = (destination or Path(tempfile.mkdtemp(prefix="l20-lite-build-"))).resolve()
    if (
        target in (Path("/"), Path.home(), DRIVER, DRIVER.parents[1])
        or DRIVER in target.parents
    ):
        raise ValueError("choose a separate empty build directory")
    if target.exists() and (not target.is_dir() or any(target.iterdir())):
        raise ValueError(
            f"build directory must be empty; existing files were preserved: {target}"
        )
    target.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        shutil.copy2(DRIVER / name, target / name)
    for source, name in (
        (DRIVER.parents[1] / "common", "common"),
        (DRIVER / "resource", "resource"),
        (DRIVER / "deploy", "deploy"),
    ):
        shutil.copytree(source, target / name, ignore=ignore_private_files)
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--context",
        type=Path,
        help="empty destination; default: a fresh temporary directory",
    )
    args = parser.parse_args()
    print(stage_context(args.context))


if __name__ == "__main__":
    main()
