"""Compile the real SDK worker against a fake adapter; no ROS/SDK/hardware."""

from pathlib import Path
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
    bool busy() { return false; }
    void start_heartbeat() { delay("slow_heartbeat"); }
    void refresh_heartbeat() {}
    void stop_heartbeat() {}
    void begin() { id_ = ++next_; delay("slow_begin"); }
    void verify_owned() {}
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


def read_reply(process):
    ready, _, _ = select.select([process.stdout], [], [], 2)
    assert ready, "worker did not acknowledge within two seconds"
    fields = process.stdout.readline().strip().split()
    assert len(fields) >= 4 and fields[0] == "MOTUS_SDK", fields
    return fields


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
        assert read_reply(process)[2:] == ["OK", "0", "ready"]
        return process

    yield create
    for process in processes:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=2)
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close()


@pytest.mark.parametrize("yaw", [-1.0, 1.0])
def test_continuous_fresh_velocity_updates_keep_one_sdk_action(worker, yaw):
    process = worker(max_wz="1.0")
    for sequence in range(1, 9):
        reply = request(process, sequence, f"SET {int(time.time() * 1000) + 200} 0.1 {yaw}")
        assert reply[2:] == ["OK", "1", "active"]
    assert request(process, 9, "STOP")[2:] == ["OK", "0", "ready"]


def test_expired_ingress_confirms_stop_and_waits_for_a_new_fresh_command(worker):
    process = worker()
    pid = process.pid
    assert request(process, 1, velocity(250))[2:] == ["OK", "1", "active"]
    expired = request(process, 2, velocity(-10))
    assert expired[2] == "EXPIRED"
    assert "remaining_ms=-" in bytes.fromhex(expired[3]).decode()
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


@pytest.mark.parametrize("mode,reason", [
    ("slow_heartbeat", "before action creation"),
    ("slow_begin", "before velocity write"),
    ("slow_set", "during SDK write"),
])
def test_expiry_during_vendor_calls_confirms_stop_without_extending_lease(worker, mode, reason):
    process = worker(mode)
    expired = request(process, 1, velocity(60))
    assert expired[2] == "EXPIRED"
    assert reason in bytes.fromhex(expired[3]).decode()
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
    assert process.wait(timeout=2) == 1


def test_expiry_with_unconfirmed_stop_is_fatal_not_an_expired_ack(worker):
    process = worker("cancel_failure")
    assert request(process, 1, velocity(250))[2] == "OK"
    fields = request(process, 2, velocity(-10))
    assert fields[2] == "ERROR"
    assert bytes.fromhex(fields[3]).decode() == "cancel unconfirmed"
    assert process.wait(timeout=2) == 1
