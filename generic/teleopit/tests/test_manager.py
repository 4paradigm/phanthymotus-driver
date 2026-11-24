"""Supervisor tests exercise cancellation and the actual child/event pipe seam."""

import base64
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import manager
from manager import SimulationManager, validate_options


def until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("timed out waiting for supervisor state")


@pytest.mark.parametrize("options", [
    {"source": "live"}, {"max_steps": True}, {"max_steps": -1}, {"max_steps": 180001},
    {"max_steps": 0.5}, {"human_height": float("nan")}, {"human_height": float("inf")},
    {"render": "true"}, {"upstream_root": "/tmp"}, {"worker_python": "sh"},
    {"policy_path": "/tmp/custom.onnx"}, {"bvh_path": "a\nb"},
])
def test_rejects_invalid_and_server_only_controls(options):
    with pytest.raises(ValueError):
        validate_options(options)


def test_no_install_or_worker_on_lifecycle_info():
    session = SimulationManager()
    assert session.info()["state"] == "idle"
    assert session.info()["hardware_output"] is False
    assert session.preview() is None
    assert session.preflight()["ready"] is False
    assert session._process is None
    session.close()
    with pytest.raises(ValueError, match="关闭"):
        session.run()


def test_failed_preflight_never_launches(monkeypatch):
    session = SimulationManager()
    monkeypatch.setattr(session, "_check", lambda _: {"ready": False, "errors": ["missing policy"]})
    monkeypatch.setattr(manager.subprocess, "Popen", lambda *a, **k: pytest.fail("must not spawn"))
    assert session.run()["state"] == "starting"
    until(lambda: session.info()["state"] == "error")
    assert "missing policy" in session.info()["error"]
    session.close()


def test_stop_cancels_blocked_preflight_and_fences_old_frames(monkeypatch):
    session = SimulationManager()
    entered, release = threading.Event(), threading.Event()

    def check(_):
        entered.set()
        assert release.wait(3)
        return {"ready": True, "paths": {}}

    monkeypatch.setattr(session, "_check", check)
    monkeypatch.setattr(manager.subprocess, "Popen", lambda *a, **k: pytest.fail("cancelled launch"))
    first = session.run()
    assert entered.wait(1)
    assert session.run()["accepted"] is False
    old_generation = session._generation
    assert session.stop()["state"] == "idle"
    release.set()
    session._receive(old_generation, {"event": "ready"})
    session._receive(old_generation, {"event": "frame", "snapshot": {"step": 8}})
    assert session.info()["snapshot"] == {}
    assert first["session_id"] == session.info()["session_id"]
    session.close()


CHILD = """
import base64,json,os,sys
fd=int(sys.argv[1]); out=os.fdopen(fd,'w',buffering=1)
def emit(value): out.write(json.dumps(value)+'\\n')
json.loads(sys.stdin.readline())
emit({'event':'ready'})
emit({'event':'frame','snapshot':{'step':1,'joint_positions':[0]*29},'preview_jpeg_b64':base64.b64encode(b'jpeg').decode()})
for line in sys.stdin:
    command=json.loads(line)['command']
    if command=='pause': emit({'event':'paused'})
    if command=='resume': emit({'event':'resumed'})
    if command=='stop': break
emit({'event':'complete','summary':{'steps':1}})
"""


def setup_process(monkeypatch, child=CHILD):
    session = SimulationManager()
    monkeypatch.setattr(session, "_check", lambda _: {"ready": True, "paths": {}})
    original = subprocess.Popen

    def spawn(argv, **kwargs):
        return original([sys.executable, "-u", "-c", child, argv[-1]], **kwargs)

    monkeypatch.setattr(manager.subprocess, "Popen", spawn)
    return session


def test_real_pipe_pause_resume_stop_and_restart(monkeypatch):
    session = setup_process(monkeypatch)
    try:
        first = session.run()
        until(lambda: session.info()["snapshot"].get("step") == 1)
        assert session.preview() == b"jpeg"
        snapshot = session.info()["snapshot"]
        snapshot["joint_positions"].clear()
        assert len(session.info()["snapshot"]["joint_positions"]) == 29
        assert session.pause()["state"] in {"pausing", "paused"}
        until(lambda: session.info()["state"] == "paused")
        session.resume()
        until(lambda: session.info()["state"] == "running")
        assert session.stop()["state"] == "idle"
        assert session.preview() is None
        second = session.run()
        assert second["session_id"] != first["session_id"]
        until(lambda: session.info()["state"] == "running")
    finally:
        session.close()


def test_unexpected_exit_is_error(monkeypatch):
    session = setup_process(monkeypatch, "import sys; sys.stdin.readline(); sys.exit(7)")
    try:
        session.run()
        until(lambda: session.info()["state"] == "error")
        assert "code=7" in session.info()["error"]
    finally:
        session.close()


def test_kills_child_that_ignores_stop(monkeypatch):
    session = setup_process(monkeypatch, "import sys,time; sys.stdin.readline(); time.sleep(30)")
    session.run()
    until(lambda: session._process is not None)
    process = session._process
    begin = time.monotonic()
    assert session.stop()["state"] == "idle"
    assert time.monotonic() - begin < 4
    assert process.poll() is not None
    session.close()


def test_complete_waits_for_child_exit(monkeypatch):
    child = """import json,os,sys,time
out=os.fdopen(int(sys.argv[1]),'w',buffering=1)
sys.stdin.readline()
out.write(json.dumps({'event':'complete','summary':{'steps':2}})+'\\n')
time.sleep(.25)
"""
    session = setup_process(monkeypatch, child)
    try:
        session.run()
        until(lambda: session.info()["state"] == "finishing")
        assert session.run()["accepted"] is False
        until(lambda: session.info()["state"] == "completed")
        assert session.info()["snapshot"]["summary"]["steps"] == 2
    finally:
        session.close()


def test_concurrent_stops_cannot_reopen_while_reaping(monkeypatch):
    session = SimulationManager()
    entered, release = threading.Event(), threading.Event()

    def terminate(_):
        entered.set()
        assert release.wait(3)

    monkeypatch.setattr(session, "_terminate", terminate)
    first = threading.Thread(target=session.stop)
    second = threading.Thread(target=session.stop)
    first.start()
    assert entered.wait(1)
    second.start()
    assert session.info()["state"] == "stopping"
    assert session.run()["accepted"] is False
    release.set()
    first.join(1)
    second.join(1)
    assert not first.is_alive() and not second.is_alive()
    assert session.info()["state"] == "idle"


def test_diagnostics_and_waiting_input_are_visible():
    session = SimulationManager()
    session._receive(0, {"event": "ready", "waiting_for_input": True, "source": "pico"})
    assert session.info()["backend"]["waiting_for_input"] is True
    session._receive(0, {"event": "diagnostic", "code": "preview_unavailable", "message": "missing GL"})
    assert session.info()["diagnostics"][0]["code"] == "preview_unavailable"
    session._receive(0, {"event": "frame", "snapshot": {"step": 1}})
    assert session.info()["backend"]["waiting_for_input"] is False
    session.close()


def test_preflight_rejects_missing_hydra_or_old_pico_bridge(monkeypatch, tmp_path):
    session = SimulationManager({"upstream_root": str(tmp_path)})
    monkeypatch.setattr(manager, "inspect_installation", lambda *a, **k: {"ready": True, "errors": [], "paths": {}})

    def probe(argv, **kwargs):
        assert "hydra" in argv[-1] and "qpsolvers" in argv[-1] and "scipy" in argv[-1]
        assert "pico-bridge" in argv[-1] and "0.2.1" in argv[-1]
        return subprocess.CompletedProcess(argv, 0, stdout=json.dumps({
            "missing": ["hydra"], "version_mismatches": ["pico-bridge: 0.2.0 != 0.2.1"],
            "origin": str(tmp_path / "teleopit" / "__init__.py"),
        }))

    monkeypatch.setattr(manager.subprocess, "run", probe)
    result = session.preflight({"source": "pico"})
    assert result["ready"] is False
    assert "hydra" in " ".join(result["errors"])
    assert "pico-bridge" in " ".join(result["errors"])
    session.close()
