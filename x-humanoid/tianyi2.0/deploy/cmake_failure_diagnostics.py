"""Read-only, bounded diagnostics for the Tianyi ROS message build failure.

Only named build metadata is reported: never dump the environment, CMakeCache,
compiler logs or registry configuration wholesale. The caller keeps colcon's
original exit code even if this diagnostic itself fails.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


KEYS = {
    "rcutils_DIR", "_lib", "CMAKE_BUILD_TYPE", "CMAKE_PREFIX_PATH",
    "CMAKE_LIBRARY_PATH", "CMAKE_SYSTEM_LIBRARY_PATH", "CMAKE_LIBRARY_ARCHITECTURE",
    "CMAKE_FIND_ROOT_PATH", "CMAKE_FIND_ROOT_PATH_MODE_LIBRARY",
    "CMAKE_FIND_LIBRARY_PREFIXES", "CMAKE_FIND_LIBRARY_SUFFIXES",
    "CMAKE_FIND_LIBRARY_CUSTOM_LIB_SUFFIX", "CMAKE_SYSROOT", "CMAKE_SIZEOF_VOID_P",
    "CMAKE_SYSTEM_NAME", "CMAKE_SYSTEM_PROCESSOR", "CMAKE_CROSSCOMPILING",
    "CMAKE_HOST_SYSTEM_NAME", "CMAKE_HOST_SYSTEM_PROCESSOR",
    "CMAKE_C_COMPILER", "CMAKE_C_COMPILER_ID", "CMAKE_C_COMPILER_VERSION",
    "CMAKE_C_COMPILER_TARGET", "CMAKE_C_COMPILER_ARCHITECTURE_ID",
    "CMAKE_C_SIZEOF_DATA_PTR", "CMAKE_C_LIBRARY_ARCHITECTURE",
    "CMAKE_CXX_COMPILER", "CMAKE_CXX_COMPILER_ID", "CMAKE_CXX_COMPILER_VERSION",
    "CMAKE_CXX_COMPILER_TARGET", "CMAKE_CXX_SIZEOF_DATA_PTR",
}


def emit(label, value):
    # JSON escaping prevents build metadata from injecting terminal controls.
    print("TIANYI_CMAKE_DIAGNOSTIC " + label + " " + json.dumps(value, ensure_ascii=True))


def read(path):
    try:
        with path.open(errors="replace") as stream:
            return stream.read(65536)
    except OSError as exc:
        emit("read_error", {"path": str(path), "error": type(exc).__name__})
        return ""


def diagnose(workspace, ros_prefix):
    emit("platform", {"machine": platform.machine(), "system": platform.system(),
                      "python_bits": 64 if sys.maxsize > 2**32 else 32})
    emit("CMAKE_PREFIX_PATH", os.environ.get("CMAKE_PREFIX_PATH", ""))
    try:
        version = subprocess.run(["cmake", "--version"], capture_output=True, text=True, timeout=5)
        emit("cmake", {"returncode": version.returncode, "version": version.stdout.splitlines()[:1]})
    except (OSError, subprocess.TimeoutExpired) as exc:
        emit("cmake", {"error": type(exc).__name__})

    paths = sorted(set([ros_prefix / "lib/librcutils.so", *ros_prefix.glob("lib/librcutils*")]))
    for path in paths[:16]:
        item = {"path": str(path), "exists": path.exists(), "symlink": path.is_symlink()}
        if path.is_symlink():
            item["link_target"] = os.readlink(path)
        if path.is_file():
            item["size"] = path.stat().st_size
            with path.open("rb") as stream:
                header = stream.read(20)
            if header[:4] == b"\x7fELF" and len(header) == 20:
                item["elf_class"] = header[4]
                item["elf_machine"] = int.from_bytes(header[18:20], "little" if header[5] == 1 else "big")
        emit("rcutils_library", item)

    export = ros_prefix / "share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake"
    text = read(export)
    start = text.find("find_library(")
    emit("rcutils_find_library", {"path": str(export),
                                 "call": text[start:text.find(")", start) + 1] if start >= 0 else "missing"})
    for package in ("bodyctrl_msgs", "lyre_msgs"):
        folder = workspace / "build" / package
        paths = [folder / "CMakeCache.txt"]
        for filename in ("CMakeSystem.cmake", "CMakeCCompiler.cmake", "CMakeCXXCompiler.cmake"):
            paths.extend(sorted((folder / "CMakeFiles").glob("*/" + filename))[:4])
        for path in paths:
            entries = []
            for line in read(path).splitlines():
                if path.name == "CMakeCache.txt":
                    if line.partition(":")[0] in KEYS and "=" in line:
                        entries.append(line[:2048])
                elif any(line.startswith("set(" + key + " ") for key in KEYS):
                    entries.append(line[:2048])
            emit("cmake_metadata", {"path": str(path), "entries": entries})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path("/tianyi_ws"))
    parser.add_argument("--ros-prefix", type=Path, default=Path("/opt/ros/humble"))
    args = parser.parse_args()
    diagnose(args.workspace, args.ros_prefix)
