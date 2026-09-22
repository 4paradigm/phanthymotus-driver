"""Exercise the real failure reporter and Docker shell without Docker or ROS."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"
REPORTER = DRIVER / "deploy/cmake_failure_diagnostics.py"


@pytest.mark.parametrize("library", ["present", "missing", "dangling"])
def test_reports_missing_library_architecture_and_only_selected_metadata(tmp_path, library):
    prefix = tmp_path / "ros"
    lib = prefix / "lib/librcutils.so"
    lib.parent.mkdir(parents=True)
    if library == "present":
        header = bytearray(20)
        header[:6] = b"\x7fELF\x02\x01"
        header[18:20] = (183).to_bytes(2, "little")
        lib.write_bytes(header)
    elif library == "dangling":
        lib.symlink_to("absent-rcutils.so")
    export = prefix / "share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake"
    export.parent.mkdir(parents=True)
    export.write_text('find_library(\n _lib NAMES "${_library}"\n'
                      ' PATHS "${rcutils_DIR}/../../../lib"\n'
                      ' NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH\n)\n')
    workspace = tmp_path / "workspace"
    build = workspace / "build/bodyctrl_msgs"
    build.mkdir(parents=True)
    (build / "CMakeCache.txt").write_text(
        "_lib:FILEPATH=_lib-NOTFOUND\nrcutils_DIR:PATH=/ros/share/rcutils/cmake\n"
        "CMAKE_C_COMPILER:FILEPATH=/usr/bin/cc\nPRIVATE_TOKEN:STRING=never-dump-me\n")
    cmake = build / "CMakeFiles/3.22.1/CMakeSystem.cmake"
    cmake.parent.mkdir(parents=True)
    cmake.write_text('set(CMAKE_SYSTEM_PROCESSOR "aarch64")\n'
                     'set(UNRELATED_SECRET "never-dump-me")\n')
    env = {**os.environ, "CMAKE_PREFIX_PATH": str(prefix), "PRIVATE_TOKEN": "never-dump-me"}
    result = subprocess.run([sys.executable, str(REPORTER), "--workspace", str(workspace),
                             "--ros-prefix", str(prefix)], env=env, text=True,
                            capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "never-dump-me" not in result.stdout + result.stderr
    assert "_lib-NOTFOUND" in result.stdout and "aarch64" in result.stdout
    assert "NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH" in result.stdout
    rows = [line.split(" ", 2) for line in result.stdout.splitlines()]
    item = next(json.loads(data) for _, kind, data in rows if kind == "rcutils_library")
    assert item["exists"] is (library == "present")
    assert item["symlink"] is (library == "dangling")
    if library == "present":
        assert item["elf_class"] == 2 and item["elf_machine"] == 183
    elif library == "dangling":
        assert item["link_target"] == "absent-rcutils.so"


@pytest.mark.parametrize("build_status,diagnostic_fails", [(0, False), (37, False), (37, True)])
def test_docker_compile_keeps_original_failure_status(tmp_path, build_status, diagnostic_fails):
    text = (DRIVER / "Dockerfile").read_text().replace("\\\n", "")
    command = next(line[4:] for line in text.splitlines() if line.startswith("RUN cd /tianyi_ws"))
    for prefix in ("/opt/ros/humble", "/ros_ws/install", "/tianyi_ws"):
        folder = tmp_path / prefix.lstrip("/")
        folder.mkdir(parents=True)
        (folder / "setup.sh").write_text(":\n")
        command = command.replace(prefix, shlex.quote(str(folder)))
    reporter = tmp_path / "reporter.py"
    reporter.write_text("print('DIAGNOSTIC_CALLED')\nraise SystemExit(86)\n" if diagnostic_fails
                        else REPORTER.read_text())
    command = command.replace("/tmp/tianyi-cmake-failure.py", shlex.quote(str(reporter)))
    bindir = tmp_path / "bin"
    bindir.mkdir()
    colcon = bindir / "colcon"
    colcon.write_text("#!/bin/sh\nexit " + str(build_status) + "\n")
    colcon.chmod(0o755)
    (bindir / "python3").symlink_to(sys.executable)
    env = {"PATH": str(bindir) + os.pathsep + os.defpath}
    result = subprocess.run(["/bin/sh", "-c", command], env=env, text=True,
                            capture_output=True, timeout=10)
    assert result.returncode == build_status, result.stderr
    if build_status == 0:
        assert result.stdout == ""  # Successful builds do not execute diagnostics.
    elif diagnostic_fails:
        assert "DIAGNOSTIC_CALLED" in result.stdout
    else:
        assert "TIANYI_CMAKE_DIAGNOSTIC" in result.stdout
