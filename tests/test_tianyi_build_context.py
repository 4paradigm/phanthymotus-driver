"""Run both real Tianyi build scripts with a Docker recorder, never a daemon.

The standard CI build once omitted audio_msgs while the candidate helper staged
it. colcon only warned about the unknown selected package, so the error surfaced
later when main imported ext_devices. Check both generated contexts and the ROS
sources that the Dockerfile actually copies, including missing-source failures.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = Path("x-humanoid/tianyi2.0")
AUDIO = Path("robotera/q5_bundle/vendor/audio_msgs")


@pytest.fixture
def build_tree(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    shutil.copy2(ROOT / "build.sh", repo / "build.sh")
    for path in (DRIVER, Path("common"), AUDIO):
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

    for path in (ROOT / AUDIO).rglob("*"):
        if path.is_file():
            staged = str(Path("audio_msgs") / path.relative_to(ROOT / AUDIO))
            assert record["files"][staged] == hashlib.sha256(path.read_bytes()).hexdigest()

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
                original = repo / AUDIO / "package.xml" if name.startswith("audio_msgs/") else repo / DRIVER / name
                packages.add(ET.parse(original).findtext("name"))
    assert {"bodyctrl_msgs", "lyre_msgs", "audio_msgs"} <= packages
    assert 'from audio_msgs.msg import AudioChunk' in record["dockerfile"]


@pytest.mark.parametrize("entrypoint", ["standard", "candidate"])
def test_missing_audio_source_cannot_produce_successful_build(build_tree, entrypoint):
    repo, env, _ = build_tree
    shutil.rmtree(repo / AUDIO)  # Only the private pytest fixture copy.
    result, _ = run_build(build_tree, entrypoint)
    assert result.returncode != 0
    assert "audio_msgs" in result.stdout + result.stderr
    if entrypoint == "candidate":
        assert list(Path(env["TMPDIR"]).iterdir()) == []
