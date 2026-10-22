"""Real worker IPC and compiled lease/ownership logic, without hardware."""

import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"
spec = importlib.util.spec_from_file_location("tianyi_sdk_backend_test", DRIVER / "slamtec_velocity.py")
backend_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(backend_module)


@pytest.fixture
def backend(tmp_path):
    workers = []
    def create(mode="ok"):
        script = tmp_path / (mode + "-worker")
        script.write_text(f'''#!{sys.executable}
import sys,time
mode={mode!r}
print("SDK diagnostic on stdout",flush=True)
print("MOTUS_SDK 0 OK 0 ready",flush=True)
for line in sys.stdin:
    seq,verb,*args=line.split()
    if mode == "timeout" and verb == "SET":
        time.sleep(30)
    if mode == "error" and verb == "SET":
        print("MOTUS_SDK "+seq+" ERROR "+b"localization quality too low".hex(),flush=True)
        sys.exit(1)
    if mode == "expired" and verb == "SET" and seq == "1":
        print("MOTUS_SDK "+seq+" EXPIRED "+b"command lease expired at worker: remaining_ms=-12".hex(),flush=True)
        continue
    if mode == "expired_stop" and verb == "STOP":
        print("MOTUS_SDK "+seq+" EXPIRED "+b"unexpected stop reply".hex(),flush=True)
        continue
    if mode == "wrong_sequence":
        seq=str(int(seq)+1)
    if mode == "dead":
        sys.exit(1)
    if mode == "partial":
        response="MOTUS_SDK "+seq+" OK 17 active\\n"
        sys.stdout.write(response[:7]);sys.stdout.flush();time.sleep(.01)
        sys.stdout.write(response[7:]);sys.stdout.flush()
    else:
        print("MOTUS_SDK "+seq+" OK "+("17 active" if verb == "SET" else "0 ready"),flush=True)
''')
        script.chmod(0o700)
        instance = backend_module.SlamtecVelocityBackend({"sdk_binary": str(script), "sdk_reply_timeout": 0.15})
        workers.append(instance)
        return instance
    yield create
    for worker in workers:
        try:
            worker.close()
        except RuntimeError:
            pass


def write(worker):
    worker.write([0.05, 0, 0, 0, 0, -0.1], int(time.time() * 1000) + 200)


def test_connect_is_readonly_and_stop_closes_own_session(backend):
    worker = backend()
    assert not worker.info()["connected"]
    worker.connect()
    assert worker.info()["connected"]
    assert worker.info()["action_id"] == 0
    write(worker)
    assert worker.info()["action_id"] == 17
    worker.stop()
    assert worker.info()["action_id"] == 0
    worker.close()
    assert not worker.info()["connected"]


@pytest.mark.parametrize("mode", ["partial", "ok"])
def test_buffered_protocol_handles_partial_replies_and_sdk_logs(backend, mode):
    worker = backend(mode)
    worker.connect()
    write(worker)
    assert worker.info()["action_id"] == 17


@pytest.mark.parametrize("mode,reason", [("error", "localization quality"),
                                        ("wrong_sequence", "sequence mismatch"),
                                        ("dead", "closed its output")])
def test_sdk_errors_are_visible_and_never_auto_reconnect(backend, mode, reason):
    worker = backend(mode)
    worker.connect()
    with pytest.raises(RuntimeError, match=reason):
        write(worker)
    assert worker.info()["state"] == "error"
    assert not worker.info()["connected"]
    with pytest.raises(RuntimeError, match="faulted"):
        worker.connect()
    with pytest.raises(RuntimeError, match="heartbeat expiry"):
        worker.stop()


def test_blocked_sdk_call_is_killed_without_a_command_queue(backend):
    worker = backend("timeout")
    worker.connect()
    started = time.monotonic()
    with pytest.raises(RuntimeError, match="timed out"):
        write(worker)
    assert time.monotonic() - started < 1
    assert worker._process.poll() is not None
    assert worker.info()["handover_wait_ms"] > 0
    # No new worker is launched by a subsequent command.
    pid = worker._process.pid
    with pytest.raises(RuntimeError):
        write(worker)
    assert worker._process.pid == pid


def test_expired_ack_preserves_connection_but_never_replays_old_velocity(backend):
    worker = backend("expired")
    worker.connect()
    pid = worker._process.pid
    with pytest.raises(backend_module.CommandLeaseExpired, match="remaining_ms=-12"):
        write(worker)
    info = worker.info()
    assert info["connected"] and info["state"] == "ready"
    assert info["error"] == "" and info["action_id"] == 0
    assert info["expired_commands"] == 1
    assert "remaining_ms=-12" in info["last_expired_error"]
    assert 0 < info["last_write"]["lease_remaining_ms"] <= 200
    assert info["last_write"]["round_trip_ms"] is not None
    assert not worker._armed and worker._sequence == 1
    worker.stop()
    assert worker._sequence == 2 and worker._process.pid == pid
    write(worker)  # Only this new call may start another velocity action.
    assert worker.info()["action_id"] == 17
    assert worker._sequence == 3 and worker._process.pid == pid


def test_expired_status_cannot_be_used_as_a_stop_acknowledgement(backend):
    worker = backend("expired_stop")
    worker.connect()
    write(worker)
    with pytest.raises(RuntimeError, match="unexpected EXPIRED"):
        worker.stop()
    assert not worker.info()["connected"]
    assert worker.info()["handover_wait_ms"] > 0


def test_dead_worker_requires_firmware_lease_to_expire_before_handover(backend, monkeypatch):
    worker = backend()
    worker.connect()
    write(worker)
    worker._process.kill(); worker._process.wait()
    with pytest.raises(RuntimeError, match="not connected"):
        worker.stop()
    with pytest.raises(RuntimeError, match="heartbeat expiry"):
        worker.stop()
    real_monotonic = time.monotonic
    monkeypatch.setattr(backend_module.time, "monotonic", lambda: real_monotonic() + 2)
    worker.stop()
    assert not worker._armed


@pytest.mark.parametrize("values,deadline", [
    ([0.1, 0, 0, 0, 0, float("nan")], "fresh"),
    ([0.1, 0.1, 0, 0, 0, 0], "fresh"),
    ([0.1, 0, 0, 0, 0, 0], "expired"),
])
def test_invalid_or_expired_values_do_not_reach_worker(backend, values, deadline):
    worker = backend()
    worker.connect()
    expiry = int(time.time() * 1000) + (200 if deadline == "fresh" else -10)
    with pytest.raises(ValueError):
        worker.write(values, expiry)
    assert worker.info()["action_id"] == 0
    assert worker._sequence == 0


@pytest.fixture(scope="module")
def session_binary(tmp_path_factory):
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    binary = tmp_path_factory.mktemp("native-session") / "session-test"
    subprocess.run([compiler, "-std=c++14", "-pthread", "-I", str(DRIVER / "native"),
                    str(ROOT / "tests/tianyi_velocity_session_test.cpp"), "-o", str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize("case", ["idle", "combined", "watchdog", "foreign", "expired",
                                 "slow_begin", "slow_set", "cancel_failure", "begin_failure", "preempted"])
def test_native_session_leases_and_ownership(session_binary, case):
    subprocess.run([str(session_binary), case], check=True, timeout=3)
