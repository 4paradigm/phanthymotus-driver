"""Run both real Tianyi build scripts with a Docker recorder, never a daemon.

audio_msgs already exists in ros-base. Docker RUN does not execute the base's
entrypoint, so the compile/import commands must explicitly source that overlay.
Check real context staging and execute the Dockerfile's shell commands in an
isolated substitute filesystem; ROS compilation remains an image-build check.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = Path("x-humanoid/tianyi2.0")


@pytest.fixture
def build_tree(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    shutil.copy2(ROOT / "build.sh", repo / "build.sh")
    for path in (DRIVER, Path("common")):
        shutil.copytree(ROOT / path, repo / path)
    bindir = tmp_path / "bin"
    bindir.mkdir()
    # Record the context at the exact moment the real build script calls Docker.
    # Reject any login/run/push so a test cannot accidentally exercise deployment.
    docker = bindir / "docker"
    docker.write_text(f"#!{sys.executable}\n" + '''
import hashlib
import json
import os
from pathlib import Path
import sys

args = sys.argv[1:]
if not (args[:1] == ["build"] or args[:2] == ["buildx", "build"]):
    raise SystemExit("unexpected Docker action: " + repr(args))
if "--push" in args:
    raise SystemExit("tests must never push")
context = Path(args[-1])
files = {str(p.relative_to(context)): hashlib.sha256(p.read_bytes()).hexdigest()
         for p in context.rglob("*") if p.is_file()}
Path(os.environ["BUILD_RECORD"]).write_text(json.dumps({
    "args": args, "context": str(context), "files": files,
    "dockerfile": (context / "Dockerfile").read_text(),
}))
# A missing explicit COPY is a hard build failure, unlike colcon's warning for
# a missing --packages-select name. This recorder covers directory COPY only;
# the repository-wide Dockerfile tests cover the remaining COPY operands.
for line in (context / "Dockerfile").read_text().splitlines():
    words = line.split()
    if words[:1] == ["COPY"] and len(words) == 3 and words[1].endswith("/"):
        if not (context / words[1]).is_dir():
            raise SystemExit("missing COPY source: " + words[1])
''')
    docker.chmod(0o755)
    uname = bindir / "uname"
    uname.write_text("#!/bin/sh\nprintf 'aarch64\\n'\n")
    uname.chmod(0o755)
    tempdir = tmp_path / "build-tmp"
    tempdir.mkdir()
    record = tmp_path / "record.json"
    # Do not inherit registry credentials, proxy settings or a developer .env.
    env = {
        "PATH": os.pathsep.join((str(bindir), str(Path(sys.executable).parent), os.defpath)),
        "HOME": str(tmp_path), "TMPDIR": str(tempdir),
        "BUILD_RECORD": str(record), "LC_ALL": "C",
    }
    return repo, env, record


def run_build(build_tree, entrypoint):
    repo, env, record = build_tree
    if entrypoint == "standard":
        args = ["bash", "build.sh", "--mirror", "none", str(DRIVER)]
    else:
        args = ["bash", str(DRIVER / "deploy/build_teleop.sh"), "local/tianyi:test"]
    result = subprocess.run(args, cwd=repo, env=env, text=True,
                            capture_output=True, timeout=30)
    return result, json.loads(record.read_text()) if record.exists() else None


@pytest.mark.parametrize("entrypoint", ["standard", "candidate"])
def test_build_stages_message_packages_used_by_dockerfile(build_tree, entrypoint):
    result, record = run_build(build_tree, entrypoint)
    assert result.returncode == 0, result.stdout + result.stderr
    assert record is not None
    assert not Path(record["context"]).exists(), "temporary context must be cleaned"
    assert "--platform" in record["args"] and "linux/arm64" in record["args"]
    assert "--push" not in record["args"]

    assert "common/logsafe.py" in record["files"]
    assert not any("audio_msgs/" in name for name in record["files"])

    # Inspect the actual Dockerfile COPY operands rather than assuming that a
    # package somewhere in the context will be available to colcon.
    packages = set()
    repo = build_tree[0]
    for line in record["dockerfile"].splitlines():
        words = line.split()
        if words[:1] != ["COPY"] or len(words) != 3:
            continue
        source, destination = words[1:]
        if not destination.startswith("/tianyi_ws/src/"):
            continue
        prefix = source.rstrip("/") + "/"
        for name in record["files"]:
            if name.startswith(prefix) and name.endswith("/package.xml"):
                original = repo / DRIVER / name
                packages.add(ET.parse(original).findtext("name"))
    assert {"bodyctrl_msgs", "lyre_msgs"} == packages
    assert 'from audio_msgs.msg import AudioChunk' in record["dockerfile"]
    selected = next(line.split('--packages-select ', 1)[1].split(';', 1)[0].split()
                    for line in record["dockerfile"].splitlines() if 'colcon build ' in line)
    assert selected == ['bodyctrl_msgs', 'lyre_msgs']


@pytest.mark.parametrize('phase', ['compile', 'import', 'runtime'])
@pytest.mark.parametrize('missing_overlay', [False, True])
def test_docker_shell_sources_base_overlay_and_fails_if_missing(tmp_path, phase, missing_overlay):
    dockerfile = (ROOT / DRIVER / 'Dockerfile').read_text().replace('\\\n', '')
    lines = dockerfile.splitlines()
    if phase == 'compile':
        command = next(line[4:] for line in lines if line.startswith('RUN cd /tianyi_ws'))
    elif phase == 'import':
        command = shlex.split(next(line[4:] for line in lines
                                  if line.startswith('RUN bash -c')))[2]
    else:
        command = json.loads(next(line[4:] for line in lines if line.startswith('CMD ')))[2]

    root = tmp_path / 'isolated root'
    for prefix, label in (('/opt/ros/humble', 'ros'), ('/ros_ws/install', 'audio'),
                          ('/tianyi_ws/install', 'tianyi')):
        folder = root / prefix.lstrip('/')
        folder.mkdir(parents=True)
        for extension in ('sh', 'bash'):
            if label != 'audio' or not missing_overlay:
                (folder / ('setup.' + extension)).write_text(
                    'export OVERLAY_TRACE="${OVERLAY_TRACE:+${OVERLAY_TRACE},}' + label + '"\n')
    for prefix in ('/opt/ros', '/ros_ws', '/tianyi_ws', '/work'):
        command = command.replace(prefix, shlex.quote(str(root / prefix.lstrip('/'))))

    bindir = tmp_path / 'bin'
    bindir.mkdir()
    for name in ('python3', 'colcon'):
        probe = bindir / name
        probe.write_text('#!/bin/sh\n'
                         '[ "$OVERLAY_TRACE" = "$EXPECTED_OVERLAYS" ] || exit 97\n'
                         'printf "overlay-ready\\n"\n')
        probe.chmod(0o755)
    env = {'PATH': str(bindir) + os.pathsep + os.defpath,
           'EXPECTED_OVERLAYS': 'ros,audio' if phase == 'compile' else 'ros,audio,tianyi'}
    result = subprocess.run(['bash', '-c', command], cwd=tmp_path, env=env,
                            capture_output=True, text=True, timeout=10)
    if missing_overlay:
        assert result.returncode != 0
        assert result.stdout == ''  # Never reaches compilation/import/startup.
        assert 'ros_ws/install/setup.' in result.stderr
    else:
        assert result.returncode == 0, result.stderr
        assert result.stdout == 'overlay-ready\n'
