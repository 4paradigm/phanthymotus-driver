"""Exercise the actual C++ process against a fake SDK; never connects to a robot.

Run with python3 -m unittest discover -s franka/fr3/native -p 'test_session.py'.
The separate joint_motion executable checks trajectory math at high resolution.
"""
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import unittest


FAKE_SDK = r'''
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
namespace franka {
struct Duration { double seconds = 0.001; double toSec() const { return seconds; } };
enum class RobotMode { kOther, kIdle, kMove, kGuiding, kReflex, kUserStopped, kAutomaticErrorRecovery };
enum class RealtimeConfig { kIgnore, kEnforce };
enum class ControllerMode { kJointImpedance };
struct RobotState {
  std::array<double,7> q{0,-0.5,0,-1.5,0,1.5,0}, q_d=q, dq{}, tau_J{};
  RobotMode robot_mode=RobotMode::kIdle;
  bool current_errors=false;
};
struct JointPositions {
  std::array<double,7> q;
  bool motion_finished=false;
  explicit JointPositions(std::array<double,7> a):q(a){}
};
inline JointPositions MotionFinished(JointPositions q) { q.motion_finished=true; return q; }
struct ControlException:std::runtime_error { using std::runtime_error::runtime_error; };
inline bool hasRealtimeKernel() { return std::getenv("FR3_FAKE_RT") != nullptr; }
inline bool setCurrentThreadToHighestSchedulerPriority(std::string* error) {
  if (std::getenv("FR3_FAKE_NO_FIFO")) {
    if (error) *error="SCHED_FIFO permission denied";
    return false;
  }
  return true;
}
class Robot {
  RobotState state_;
  std::atomic<bool> stopped_{false};
 public:
  Robot(std::string, RealtimeConfig config) {
    if (config==RealtimeConfig::kEnforce && !hasRealtimeKernel())
      throw std::runtime_error("SDK enforces realtime kernel");
    if (config==RealtimeConfig::kEnforce && std::getenv("FR3_FAKE_NO_FIFO"))
      throw std::runtime_error("SDK enforces realtime scheduling");
    if (const char* expected=std::getenv("FR3_FAKE_EXPECT_CONFIG")) {
      const auto actual=config==RealtimeConfig::kEnforce ? "enforce" : "ignore";
      if (std::string(expected)!=actual)
        throw std::runtime_error("Unexpected SDK realtime configuration");
    }
  }
  template<typename Callback> void read(Callback cb) { while(cb(readOnce())) {} }
  RobotState readOnce() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); return state_;
  }
  void stop() {
    if(std::getenv("FR3_FAKE_STOP_FAIL")) throw std::runtime_error("Simulated stop failure");
    stopped_=true;
  }
  template<typename Callback> void control(Callback cb, ControllerMode, bool) {
    stopped_=false;
    state_.robot_mode=RobotMode::kMove;
    unsigned tick=0;
    try {
      for (;;) {
        if (stopped_) throw ControlException("Stopped by user");
        if (std::getenv("FR3_FAKE_COLLISION") && tick>30) {
          state_.current_errors=true; throw ControlException("Simulated collision");
        }
        auto command=cb(state_, Duration{tick++ == 0 ? 0.0 : 0.001});
        state_.q_d=command.q;
        if (!std::getenv("FR3_FAKE_NO_TRACKING")) state_.q=command.q;
        if(command.motion_finished) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (...) { state_.robot_mode=RobotMode::kIdle; throw; }
    state_.robot_mode=RobotMode::kIdle;
  }
};
}
'''


class NativeSessionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temporary = tempfile.TemporaryDirectory(prefix="fr3-native-test-")
        cls.directory = Path(cls.temporary.name)
        headers = cls.directory / "franka"
        headers.mkdir()
        (headers / "fake.h").write_text(FAKE_SDK)
        for name in ("control_tools.h", "exception.h", "robot.h", "robot_state.h"):
            (headers / name).write_text('#include "fake.h"\n')
        cls.binary = cls.directory / "franka-state-reader"
        subprocess.run([compiler, "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Wpedantic",
                        "-I", str(cls.directory), str(Path(__file__).with_name("read_state.cpp")),
                        "-o", str(cls.binary)], check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.processes = []
        self.lock_dir = tempfile.TemporaryDirectory(prefix="fr3-lock-test-")

    def tearDown(self):
        for process, _, _ in self.processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            for pipe in (process.stdin, process.stdout, process.stderr):
                if pipe and not pipe.closed:
                    pipe.close()
        self.lock_dir.cleanup()

    def start(self, realtime=True, **options):
        env = dict(os.environ, FRANKA_LOCK_DIR=self.lock_dir.name, **options)
        if realtime:
            env["FR3_FAKE_RT"] = "1"
        else:
            env.pop("FR3_FAKE_RT", None)
        env.setdefault("FR3_FAKE_EXPECT_CONFIG",
                       "enforce" if realtime and not env.get("FR3_FAKE_NO_FIFO") else "ignore")
        process = subprocess.Popen([str(self.binary), "127.0.0.2"], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1, env=env)
        received = queue.Queue()
        history = []

        def consume():
            for line in process.stdout:
                record = json.loads(line)
                history.append(record)
                received.put(record)

        thread = threading.Thread(target=consume, daemon=True)
        thread.start()
        item = process, received, history
        self.processes.append(item)
        return item

    def wait_for(self, item, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                message = item[1].get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if predicate(message):
                return message
        self.fail(f"Expected native event missing; observed {item[2][-8:]}")

    def send(self, item, line):
        item[0].stdin.write(line + "\n")
        item[0].stdin.flush()

    def move(self, item, identifier="test", target=0.01):
        self.send(item, f"MOVE {identifier} 0.08726646259971647 0.008726646259971648 120 "
                  f"{target} -0.5 0 -1.5 0 1.5 0")

    def terminal(self, item):
        return self.wait_for(item, lambda r: r.get("event") == "motion" and
                             r["status"] in ("succeeded", "failed", "cancelled", "rejected"))

    def test_non_rt_kernel_with_fifo_refuses_motion_but_keeps_state(self):
        item = self.start(realtime=False, FR3_FAKE_EXPECT_CONFIG="ignore")
        state = self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertFalse(state["rt_kernel"])
        self.assertTrue(state["sched_fifo"])
        self.assertFalse(state["control_ready"])
        self.assertIn("PREEMPT_RT", state["control_error"])
        self.move(item)
        self.assertEqual(self.terminal(item)["status"], "rejected")

    def test_standard_kernel_opt_in_runs_motion_using_ignore(self):
        item = self.start(realtime=False, FRANKA_REALTIME_POLICY="standard",
                          FR3_FAKE_EXPECT_CONFIG="ignore")
        state = self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertFalse(state["rt_kernel"])
        self.assertTrue(state["control_ready"])
        self.assertEqual(state["realtime_policy"], "standard")
        self.move(item)
        self.assertEqual(self.terminal(item)["status"], "succeeded")

    def test_standard_kernel_still_requires_fifo(self):
        item = self.start(realtime=False, FRANKA_REALTIME_POLICY="standard",
                          FR3_FAKE_NO_FIFO="1", FR3_FAKE_EXPECT_CONFIG="ignore")
        state = self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertFalse(state["control_ready"])
        self.move(item)
        self.assertEqual(self.terminal(item)["status"], "rejected")

    def test_unknown_policy_fails_before_connecting(self):
        result = subprocess.run([str(self.binary), "--check-realtime"],
                                env=dict(os.environ, FRANKA_REALTIME_POLICY="typo"),
                                capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("must be enforce or standard", result.stderr)

    def test_rt_kernel_with_fifo_runs_motion_using_enforce(self):
        item = self.start(realtime=True, FR3_FAKE_EXPECT_CONFIG="enforce")
        state = self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertTrue(state["rt_kernel"])
        self.assertTrue(state["control_ready"])
        self.assertIsNone(state["control_error"])
        self.move(item)
        self.assertEqual(self.terminal(item)["status"], "succeeded")

    def test_missing_fifo_refuses_motion_but_keeps_state(self):
        item = self.start(FR3_FAKE_NO_FIFO="1", FR3_FAKE_EXPECT_CONFIG="ignore")
        state = self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertFalse(state["control_ready"])
        self.assertFalse(state["sched_fifo"])
        self.assertIn("SCHED_FIFO", state["control_error"])
        self.move(item)
        self.assertEqual(self.terminal(item)["status"], "rejected")
        self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertIsNone(item[0].poll())

    def test_realtime_probe_requires_kernel_and_fifo(self):
        for realtime in (False, True):
            for scheduling in (False, True):
                with self.subTest(realtime=realtime, scheduling=scheduling):
                    env = dict(os.environ)
                    env.pop("FR3_FAKE_RT", None)
                    env.pop("FR3_FAKE_NO_FIFO", None)
                    if realtime:
                        env["FR3_FAKE_RT"] = "1"
                    if not scheduling:
                        env["FR3_FAKE_NO_FIFO"] = "1"
                    result = subprocess.run([str(self.binary), "--check-realtime"], env=env,
                                            capture_output=True, text=True, timeout=5)
                    state = json.loads(result.stdout)
                    self.assertEqual(result.returncode, 0 if realtime and scheduling else 1)
                    self.assertEqual(state["rt_kernel"], realtime)
                    self.assertEqual(state["sched_fifo"], scheduling)
                    self.assertEqual(state["control_ready"], realtime and scheduling)
                    if realtime and scheduling:
                        self.assertIsNone(state["control_error"])
                    if not realtime:
                        self.assertIn("PREEMPT_RT", state["control_error"])
                    if not scheduling:
                        self.assertIn("SCHED_FIFO", state["control_error"])

    def test_move_requires_measured_completion(self):
        item = self.start()
        self.move(item)
        result = self.terminal(item)
        self.assertEqual(result["status"], "succeeded")
        self.assertLess(result["max_error"], 1e-9)
        self.assertTrue(any(r.get("q_d") for r in item[2]))

    def test_tracking_failure_is_not_success(self):
        item = self.start(FR3_FAKE_NO_TRACKING="1")
        self.move(item, target=0.02)
        result = self.terminal(item)
        self.assertEqual(result["status"], "failed")
        self.assertIn("tolerance", result["error"])

    def test_stop_busy_and_no_automatic_resume(self):
        item = self.start()
        self.move(item, target=0.2)
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.move(item, identifier="other", target=0.1)
        self.assertEqual(self.terminal(item)["status"], "rejected")
        self.send(item, "STOP test")
        result = self.terminal(item)
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(result["command_id"], "test")
        self.wait_for(item, lambda r: r.get("type") == "state")
        self.assertIsNone(item[0].poll())

    def test_stop_failure_is_reported(self):
        item = self.start(FR3_FAKE_STOP_FAIL="1")
        self.move(item, target=0.2)
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "STOP test")
        self.assertEqual(self.terminal(item)["status"], "failed")
        item[0].wait(timeout=4)

    def test_eof_cancels_active_motion(self):
        item = self.start()
        self.move(item, target=0.2)
        self.wait_for(item, lambda r: r.get("status") == "running")
        item[0].stdin.close()
        self.assertEqual(self.terminal(item)["status"], "cancelled")
        self.assertEqual(item[0].wait(timeout=4), 0)

    def test_cancel_before_first_control_callback(self):
        item = self.start()
        self.send(item, "MOVE test 0.08726646259971647 0.008726646259971648 120 "
                       "0.2 -0.5 0 -1.5 0 1.5 0\nSTOP test")
        self.assertEqual(self.terminal(item)["status"], "cancelled")

    def test_quit_cancels_and_exits(self):
        item = self.start()
        self.move(item, target=0.2)
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "QUIT")
        self.assertEqual(self.terminal(item)["status"], "cancelled")
        self.assertEqual(item[0].wait(timeout=4), 0)

    def test_broken_input_channel_aborts_control(self):
        item = self.start()
        self.move(item, target=0.2)
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "X" * 4097)
        self.assertEqual(self.terminal(item)["status"], "failed")
        item[0].wait(timeout=4)

    def test_native_connection_lock(self):
        first = self.start()
        self.wait_for(first, lambda r: r.get("type") == "state")
        second = self.start()
        self.assertEqual(second[0].wait(timeout=4), 1)
        self.assertIn("Another Franka process", second[0].stderr.read())
        self.assertIsNone(first[0].poll())

    def test_collision_remains_failure(self):
        item = self.start(FR3_FAKE_COLLISION="1")
        self.move(item, target=0.1)
        result = self.terminal(item)
        self.assertEqual(result["status"], "failed")
        self.assertIn("errors", result["error"])


if __name__ == "__main__":
    unittest.main()
