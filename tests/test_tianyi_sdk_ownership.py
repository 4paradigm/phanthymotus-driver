"""Exercise the real Slamware adapter against SDK-shaped in-memory handles."""

from pathlib import Path
import shutil
import subprocess

import pytest


NATIVE = Path(__file__).resolve().parents[1] / "x-humanoid/tianyi2.0/native"

SDK_STUB = r'''
#pragma once
#include <stdexcept>
#include <string>

namespace fake {
struct Handle {
    unsigned long id = 11;
    int status = 1;
    std::string reason;
    bool fail_status = false, fail_reason = false;
    int ids = 0, statuses = 0, reasons = 0, writes = 0, cancels = 0;
};
Handle owned, current;
int current_reads = 0, begin_calls = 0, selected_flag = -1;
}

namespace rpos {
namespace core {
enum ActionStatus {
    ActionStatusWaitingForStart, ActionStatusRunning, ActionStatusFinished,
    ActionStatusPaused, ActionStatusStopped, ActionStatusError
};
}
namespace features {
namespace system_resource { struct HeartBeatToken {}; }
namespace motion_planner {
enum VelocityControlFlag { MonitoredByLocalizationQuality = 0, NotMonitored };
}
}
namespace actions {
class MoveAction {
protected:
    fake::Handle* handle_ = nullptr;
public:
    MoveAction() = default;
    explicit MoveAction(fake::Handle* handle) : handle_(handle) {}
    operator bool() const { return handle_ && handle_->id; }
    unsigned long getActionId() { ++handle_->ids; return handle_->id; }
    core::ActionStatus getStatus() {
        ++handle_->statuses;
        if (handle_->fail_status) throw std::runtime_error("status read failed");
        return static_cast<core::ActionStatus>(handle_->status);
    }
    std::string getReason() {
        ++handle_->reasons;
        if (handle_->fail_reason) throw std::runtime_error("reason read failed");
        return handle_->reason;
    }
    void cancel() { ++handle_->cancels; }
};
class VelocityControlMoveAction : public MoveAction {
public:
    VelocityControlMoveAction() = default;
    explicit VelocityControlMoveAction(fake::Handle* handle) : MoveAction(handle) {}
    void setVelocity(float, float, float) { ++handle_->writes; }
};
}
namespace robot_platforms {
class SlamwareCorePlatform {
public:
    static SlamwareCorePlatform connect(const std::string&, int, int) { return {}; }
    actions::MoveAction getCurrentAction() {
        ++fake::current_reads;
        return actions::MoveAction(&fake::current);
    }
    actions::VelocityControlMoveAction velocityControl(
        features::motion_planner::VelocityControlFlag flag =
            features::motion_planner::MonitoredByLocalizationQuality) {
        ++fake::begin_calls;
        fake::selected_flag = flag;
        return actions::VelocityControlMoveAction(&fake::owned);
    }
    features::system_resource::HeartBeatToken startHeartBeat(int) { return {}; }
    void refreshHeartBeat(features::system_resource::HeartBeatToken) {}
    void stopHeartBeat(features::system_resource::HeartBeatToken) {}
};
}
}
'''

HARNESS = r'''
#include "slamware_adapter.h"
#include <iostream>

int main(int argc, char** argv) {
    std::string mode = argv[1];
    if (mode == "current_empty") fake::current.id = 0;
    if (mode == "mismatch" || mode == "diagnostics_fail") {
        fake::current.id = 27;
        fake::current.reason = "another task";
    }
    if (mode == "diagnostics_fail") {
        fake::owned.fail_status = true;
        fake::current.fail_reason = true;
    }
    if (mode == "not_running") {
        fake::owned.status = fake::current.status = 5;
        fake::owned.reason = fake::current.reason = "localization quality low";
    }
    if (mode == "waiting") fake::owned.status = fake::current.status = 0;
    if (mode == "finished") fake::owned.status = fake::current.status = 2;
    if (mode == "paused") fake::owned.status = fake::current.status = 3;
    if (mode == "stopped") fake::owned.status = fake::current.status = 4;
    SlamwareAdapter adapter("fake", 1445, 1500);
    try {
        if (mode != "owned_empty") adapter.begin();
        adapter.verify_owned();
        adapter.set(0.1, 0.2);
        std::cout << "message=ok\n";
    } catch (const std::exception& error) {
        std::cout << "message=" << error.what() << '\n';
    }
    // Cleanup must still target our held handle, including after a mismatch.
    if (mode != "owned_empty") adapter.cancel();
    std::cout << "current_reads=" << fake::current_reads << '\n'
              << "begin_calls=" << fake::begin_calls << '\n'
              << "selected_flag=" << fake::selected_flag << '\n'
              << "owned_ids=" << fake::owned.ids << '\n'
              << "current_ids=" << fake::current.ids << '\n'
              << "owned_statuses=" << fake::owned.statuses << '\n'
              << "current_statuses=" << fake::current.statuses << '\n'
              << "owned_reasons=" << fake::owned.reasons << '\n'
              << "current_reasons=" << fake::current.reasons << '\n'
              << "writes=" << fake::owned.writes << '\n'
              << "owned_cancels=" << fake::owned.cancels << '\n'
              << "current_cancels=" << fake::current.cancels << '\n';
}
'''


@pytest.fixture(scope="module")
def adapter_binary(tmp_path_factory):
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    directory = tmp_path_factory.mktemp("sdk-ownership")
    platform = directory / "rpos/robot_platforms/slamware_core_platform.h"
    platform.parent.mkdir(parents=True)
    platform.write_text(SDK_STUB)
    velocity = directory / "rpos/features/motion_planner/velocity_control_move_action.h"
    velocity.parent.mkdir(parents=True)
    velocity.write_text("#pragma once\n")
    harness = directory / "ownership.cpp"
    harness.write_text(HARNESS)
    binary = directory / "ownership"
    subprocess.run([compiler, "-std=c++14", "-I", str(directory), "-I", str(NATIVE),
                    str(harness), "-o", str(binary)], check=True, capture_output=True, text=True)
    return binary


def run_adapter(binary, mode):
    result = subprocess.run([str(binary), mode], check=True, capture_output=True, text=True)
    return dict(line.split("=", 1) for line in result.stdout.splitlines())


@pytest.mark.parametrize("mode", ["running", "waiting"])
def test_accepted_action_preserves_successful_sdk_calls(adapter_binary, mode):
    result = run_adapter(adapter_binary, mode)
    assert result["message"] == "ok"
    assert result["selected_flag"] == "0"  # Default localization monitor remains.
    assert result["current_reads"] == result["begin_calls"] == "1"
    assert result["owned_ids"] == result["current_ids"] == "1"
    assert result["current_statuses"] == "1"
    assert result["owned_statuses"] == result["owned_reasons"] == result["current_reasons"] == "0"
    assert result["writes"] == result["owned_cancels"] == "1"
    assert result["current_cancels"] == "0"


@pytest.mark.parametrize("mode, expected", [
    ("owned_empty", "owned action handle is empty; current={id=11,status=1,reason=}"),
    ("current_empty", "current action is empty; owned={id=11,status=1,reason=}"),
    ("mismatch", "velocity action was replaced; owned={id=11,status=1,reason=}; current={id=27,status=1,reason=another task}"),
    ("not_running", "velocity action is not running; owned={id=11,status=5,reason=localization quality low}"),
])
def test_ownership_failures_distinguish_missing_handles_and_replacement(adapter_binary, mode, expected):
    result = run_adapter(adapter_binary, mode)
    assert expected in result["message"]
    assert result["message"].endswith("explicit restart required")
    assert result["writes"] == "0"
    assert result["current_cancels"] == "0"
    assert result["owned_cancels"] == ("0" if mode == "owned_empty" else "1")
    assert result["begin_calls"] == ("0" if mode == "owned_empty" else "1")


@pytest.mark.parametrize("mode", ["finished", "paused", "stopped"])
def test_nonrunning_states_remain_rejected(adapter_binary, mode):
    result = run_adapter(adapter_binary, mode)
    assert "velocity action is not running;" in result["message"]
    assert result["writes"] == "0"
    assert result["owned_cancels"] == "1"
    assert result["current_cancels"] == "0"


def test_diagnostic_read_failure_preserves_primary_ownership_error(adapter_binary):
    result = run_adapter(adapter_binary, "diagnostics_fail")
    assert "velocity action was replaced;" in result["message"]
    assert "owned={id=11,status=unavailable,reason=}" in result["message"]
    assert "current={id=27,status=1,reason=unavailable}" in result["message"]
    assert result["writes"] == "0"
    assert result["current_cancels"] == "0"
