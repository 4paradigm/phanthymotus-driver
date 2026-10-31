"""Compile the real SDK worker against a fake adapter; no ROS/SDK/hardware."""

from pathlib import Path
import json
import select
import shutil
import subprocess
import time

import pytest

NATIVE = Path(__file__).resolve().parents[1] / "x-humanoid/tianyi2.0/native"

FAKE_ADAPTER = r'''
#pragma once
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
class SlamwareAdapter {
    std::string mode_;
    unsigned long id_ = 0, next_ = 0;
    void delay(const char* phase) {
        if (mode_ == phase)
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
public:
    SlamwareAdapter(const std::string& mode, int, int) : mode_(mode) {}
    bool busy() { delay("slow_busy"); return false; }
    void start_heartbeat() { delay("slow_heartbeat"); }
    void refresh_heartbeat() { delay("slow_refresh"); }
    void stop_heartbeat() {}
    void begin() { id_ = ++next_; delay("slow_begin"); }
    void verify_owned() {
        delay("slow_verify");
        if (mode_ == "ownership_failure") throw std::runtime_error("velocity action was replaced");
    }
    void set(double vx, double wz) { if (vx || wz) delay("slow_set"); }
    void cancel() {
        if (mode_ == "cancel_failure") throw std::runtime_error("cancel unconfirmed");
        id_ = 0;
    }
    unsigned long action_id() { return id_; }
};
'''


@pytest.fixture(scope="module")
def native_worker(tmp_path_factory):
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    directory = tmp_path_factory.mktemp("native-worker")
    for name in ("loco_sdk.cpp", "velocity_session.h"):
        shutil.copyfile(NATIVE / name, directory / name)
    (directory / "slamware_adapter.h").write_text(FAKE_ADAPTER)
    binary = directory / "worker-test"
    subprocess.run([compiler, "-std=c++14", "-pthread", str(directory / "loco_sdk.cpp"),
                    "-o", str(binary)], check=True, capture_output=True, text=True)
    return binary


@pytest.fixture(scope="module")
def native_session(tmp_path_factory):
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    binary = tmp_path_factory.mktemp("atomic-session") / "session-test"
    source = Path(__file__).with_name("tianyi_velocity_session_test.cpp")
    subprocess.run([compiler, "-std=c++14", "-pthread", "-I", str(NATIVE), str(source),
                    "-o", str(binary)], check=True, capture_output=True, text=True)
    return binary


class Reply(list):
    """Keep historical protocol assertions while exposing the optional metrics."""

    def __init__(self, fields, stats):
        super().__init__(fields)
        self.stats = stats


def read_reply(process):
    ready, _, _ = select.select([process.stdout], [], [], 2)
    assert ready, "worker did not acknowledge within two seconds"
    fields = process.stdout.readline().strip().split()
    assert len(fields) >= 4 and fields[0] == "MOTUS_SDK", fields
    stats = None
    if fields[-1].startswith("{"):
        stats = json.loads(fields.pop())
        assert isinstance(stats, dict)
    return Reply(fields, stats)


def request(process, sequence, verb):
    process.stdin.write(f"{sequence} {verb}\n")
    process.stdin.flush()
    fields = read_reply(process)
    assert fields[1] == str(sequence)
    return fields


def velocity(ttl_ms):
    return f"SET {int(time.time() * 1000) + ttl_ms} 0.05 0.1"


@pytest.fixture
def worker(native_worker):
    processes = []

    def create(mode="ok", max_wz="0.6"):
        process = subprocess.Popen([str(native_worker), mode, "1445", "1500", "0.3", max_wz],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True, bufsize=1)
        processes.append(process)
        ready = read_reply(process)
        assert ready[2:] == ["OK", "0", "ready"]
        assert ready.stats == {"capabilities": {"atomic_expiry_update": True, "staged_stop": True}}
        return process

    yield create
    for process in processes:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=2)
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close()


@pytest.mark.parametrize("yaw", [-1.0, 1.0])
@pytest.mark.parametrize("verb", ["SET", "UPDATE"])
def test_continuous_fresh_velocity_updates_keep_one_sdk_action(worker, yaw, verb):
    process = worker(max_wz="1.0")
    for sequence in range(1, 9):
        reply = request(process, sequence, f"{verb} {int(time.time() * 1000) + 200} 0.1 {yaw}")
        assert reply[2:] == ["OK", "1", "active"]
        assert reply.stats["set_attempts"] == sequence
        assert reply.stats["action_start_attempts"] == reply.stats["action_starts"] == 1
        assert reply.stats["stop_attempts"] == reply.stats["confirmed_stops"] == 0
        timing = reply.stats["last_set"]
        assert timing["verify_owned_ms"] >= 0 and timing["set_velocity_ms"] >= 0
        assert 0 < timing["remaining_after_set_ms"] <= timing["remaining_before_set_ms"] <= 200
        if sequence > 1:
            assert all(timing[stage] is None for stage in ("busy_ms", "start_heartbeat_ms", "begin_ms"))
    stopped = request(process, 9, "STOP")
    assert stopped[2:] == ["OK", "0", "ready"]
    assert stopped.stats["stop_attempts"] == stopped.stats["confirmed_stops"] == 1
    assert stopped.stats["stop_reason"] == "explicit_stop"
    idle = request(process, 10, "STOP")
    assert idle.stats["stop_attempts"] == idle.stats["confirmed_stops"] == 1


@pytest.mark.parametrize("verb", ["SET", "UPDATE"])
def test_expired_ingress_confirms_stop_and_waits_for_a_new_fresh_command(worker, verb):
    process = worker()
    pid = process.pid
    assert request(process, 1, velocity(250))[2:] == ["OK", "1", "active"]
    expired = request(process, 2, velocity(-10).replace("SET ", f"{verb} ", 1))
    assert expired[2] == "EXPIRED"
    assert "remaining_ms=-" in bytes.fromhex(expired[3]).decode()
    assert expired.stats["stop_attempts"] == expired.stats["confirmed_stops"] == 1
    assert expired.stats["stop_reason"] == "lease_expired"
    assert process.poll() is None and process.pid == pid
    assert request(process, 3, "STOP")[2:] == ["OK", "0", "ready"]
    # No stale request is replayed: only a new SET creates action 2.
    assert request(process, 4, velocity(250))[2:] == ["OK", "2", "active"]
    assert request(process, 5, "STOP")[2:] == ["OK", "0", "ready"]


def test_expired_idle_request_does_not_create_an_action(worker):
    process = worker()
    assert request(process, 1, velocity(-10))[2] == "EXPIRED"
    assert request(process, 2, "STOP")[2:] == ["OK", "0", "ready"]
    assert request(process, 3, velocity(250))[2:] == ["OK", "1", "active"]


def test_fresh_update_after_expiry_restarts_without_a_separate_stop_request(worker):
    process = worker()
    assert request(process, 1, velocity(80))[2:] == ["OK", "1", "active"]
    time.sleep(0.12)
    fresh = request(process, 2, velocity(250).replace("SET ", "UPDATE ", 1))
    assert fresh[2:] == ["OK", "2", "active"]
    assert fresh.stats["set_attempts"] == 2
    assert fresh.stats["action_starts"] == 2
    assert fresh.stats["confirmed_stops"] == 1
    assert fresh.stats["stop_reason"] == "lease_expired"


@pytest.mark.parametrize("case", ["atomic_fresh_update", "atomic_expiry_update", "atomic_stop_exhausts_new_lease",
                                 "atomic_stop_failure"])
def test_update_atomically_stops_old_expired_action_before_using_new_lease(native_session, case):
    # Directly exercise Session.update without an intervening timer tick or STOP.
    subprocess.run([str(native_session), case], check=True, timeout=3)


@pytest.mark.parametrize("mode,reason", [
    ("slow_busy", "before action creation"),
    ("slow_heartbeat", "before action creation"),
    ("slow_begin", "before velocity write"),
    ("slow_verify", "before velocity write"),
    ("slow_set", "during SDK write"),
])
def test_expiry_during_vendor_calls_confirms_stop_without_extending_lease(worker, mode, reason):
    process = worker(mode)
    expired = request(process, 1, velocity(60))
    assert expired[2] == "EXPIRED"
    assert reason in bytes.fromhex(expired[3]).decode()
    stage = {"slow_busy": "busy_ms", "slow_heartbeat": "start_heartbeat_ms",
             "slow_begin": "begin_ms", "slow_verify": "verify_owned_ms",
             "slow_set": "set_velocity_ms"}[mode]
    timing = expired.stats["last_set"]
    assert timing[stage] >= 100
    assert expired.stats["stop_attempts"] == expired.stats["confirmed_stops"] == 1
    assert expired.stats["stop_reason"] == "lease_expired"
    if mode == "slow_set":
        assert timing["remaining_before_set_ms"] > 0
        assert timing["remaining_after_set_ms"] < 0
    elif mode in ("slow_begin", "slow_verify"):
        assert timing["remaining_before_set_ms"] < 0
        assert timing["set_velocity_ms"] is None
    else:
        assert timing["begin_ms"] is None
    assert request(process, 2, "STOP")[2:] == ["OK", "0", "ready"]
    # The same slow SDK fits a longer, but still bounded, ORIGINAL input lease.
    assert request(process, 3, velocity(280))[2] == "OK"
    assert request(process, 4, "STOP")[2:] == ["OK", "0", "ready"]


def test_excessive_lease_remains_fatal_and_reports_its_limit(worker):
    process = worker()
    fields = request(process, 1, velocity(1000))
    assert fields[2] == "ERROR"
    reason = bytes.fromhex(fields[3]).decode()
    assert "excessive command lease" in reason and "limit_ms=300" in reason
    assert fields.stats["action_starts"] == fields.stats["stop_attempts"] == 0
    assert process.wait(timeout=2) == 1


def test_expiry_with_unconfirmed_stop_is_fatal_not_an_expired_ack(worker):
    process = worker("cancel_failure")
    assert request(process, 1, velocity(250))[2] == "OK"
    fields = request(process, 2, velocity(-10))
    assert fields[2] == "ERROR"
    assert bytes.fromhex(fields[3]).decode() == "cancel unconfirmed"
    assert fields.stats["stop_attempts"] >= 1 and fields.stats["confirmed_stops"] == 0
    assert fields.stats["stop_reason"] == "sdk_exception"
    assert process.wait(timeout=2) == 1


def test_native_lease_watchdog_stop_is_visible_on_next_ack(worker):
    process = worker()
    assert request(process, 1, velocity(80))[2] == "OK"
    time.sleep(0.14)
    stopped = request(process, 2, "STOP")
    assert stopped[2:] == ["OK", "0", "ready"]
    assert stopped.stats["stop_attempts"] == stopped.stats["confirmed_stops"] == 1
    assert stopped.stats["stop_reason"] == "lease_expired"


def test_heartbeat_refresh_latency_is_visible_without_changing_its_cadence(worker):
    process = worker("slow_refresh")
    assert request(process, 1, velocity(280))[2] == "OK"
    time.sleep(0.15)
    stopped = request(process, 2, "STOP")
    assert stopped[2:] == ["OK", "0", "ready"]
    assert stopped.stats["heartbeat_refreshes"] >= 1
    assert stopped.stats["last_heartbeat_ms"] >= 100
    assert stopped.stats["max_heartbeat_ms"] >= stopped.stats["last_heartbeat_ms"]
    assert stopped.stats["confirmed_stops"] == 1


def test_ownership_failure_stays_fatal_and_has_stage_diagnostics(worker):
    process = worker("ownership_failure")
    failed = request(process, 1, velocity(250))
    assert failed[2] == "ERROR"
    assert bytes.fromhex(failed[3]).decode() == "velocity action was replaced"
    assert failed.stats["last_set"]["verify_owned_ms"] >= 0
    assert failed.stats["last_set"]["set_velocity_ms"] is None
    assert failed.stats["action_starts"] == failed.stats["confirmed_stops"] == 1
    assert failed.stats["stop_reason"] == "sdk_exception"
    assert process.wait(timeout=2) == 1


@pytest.mark.parametrize("case", ["staged_resume", "staged_cancel", "staged_repeat_zero",
                                 "staged_stale_drop", "staged_slow_write",
                                 "staged_cancel_failure", "staged_explicit_stop",
                                 "staged_stale_write_keeps_cancel"])
def test_native_staged_velocity_session(native_session, case):
    subprocess.run([str(native_session), case], check=True, timeout=3)


def follow(motion_ms=300, cancel_ms=1000, vx=0.1, wz=-0.2):
    now = int(time.time()*1000)
    return f"FOLLOW {now+motion_ms} {now+cancel_ms} {vx} {wz}"


def test_follow_gap_soft_zeros_but_reuses_sdk_action_and_heartbeat(worker):
    process = worker()
    first = request(process,1,follow(80))
    assert first[2:] == ["OK","1","active"]
    time.sleep(0.15)
    held = request(process,2,"SOFT_STOP")
    assert held[2:] == ["OK","1","active"]
    assert held.stats["soft_stops"] == 1 and held.stats["heartbeat_refreshes"] >= 1
    assert held.stats["confirmed_stops"] == 0
    fresh = request(process,3,follow())
    assert fresh[2:] == ["OK","1","active"]
    assert fresh.stats["action_starts"] == 1 and fresh.stats["confirmed_stops"] == 0
    stopped = request(process,4,"STOP")
    assert stopped[2:] == ["OK","0","ready"]


def test_follow_silence_cancels_at_original_one_second_without_python_ticks(worker):
    process = worker()
    assert request(process,1,follow(100))[2:] == ["OK","1","active"]
    time.sleep(1.10)
    idle = request(process,2,"SOFT_STOP")
    assert idle[2:] == ["OK","0","ready"]
    assert idle.stats["soft_stops"] == 1 and idle.stats["confirmed_stops"] == 1
    assert idle.stats["stop_reason"] == "lease_expired"
    fresh = request(process,3,follow())
    assert fresh[2:] == ["OK","2","active"]


def test_follow_old_packets_neither_cancel_nor_renew_live_action(worker):
    process = worker()
    assert request(process,1,follow())[2:] == ["OK","1","active"]
    stale = request(process,2,follow(-10))
    assert stale[2] == "STALE"
    assert stale.stats["action_id"] == 1 and stale.stats["session_active"]
    assert stale.stats["soft_stops"] == stale.stats["confirmed_stops"] == 0
    # Stale packets do not move the accepted cancellation deadline forward.
    time.sleep(1.05)
    idle = request(process,3,"SOFT_STOP")
    assert idle[2:] == ["OK","0","ready"]
    assert idle.stats["confirmed_stops"] == 1


@pytest.mark.parametrize("motion,cancel", [(500,1000),(300,2000),(250,200)])
def test_follow_rejects_excessive_or_reversed_deadlines(worker,motion,cancel):
    process=worker()
    invalid=request(process,1,follow(motion,cancel))
    assert invalid[2] == "ERROR"
    assert invalid.stats["action_starts"] == 0
    assert process.wait(timeout=2) == 1
