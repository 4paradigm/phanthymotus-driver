"""Run the real native gripper process against a fake SDK, without hardware.

python3 -m unittest discover -s franka/fr3/native -p 'test_gripper.py'
"""
import fcntl
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
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
namespace franka {
struct GripperState {
  double width = .06, max_width = .0801068;
  bool is_grasped = false;
  uint16_t temperature = 27;
};
class Gripper {
  mutable std::mutex mutex_;
  mutable GripperState state_;
  mutable std::atomic<bool> stopped_{false}, reading_{false};
  mutable std::atomic<unsigned> reads_{0};
  static void log(const std::string& message) {
    if (const char* path = std::getenv("FR3_GRIPPER_LOG")) {
      std::ofstream out(path, std::ios::app); out << message << '\n';
    }
  }
  bool motion(const std::string& action, double width) const {
    if (std::getenv("FR3_GRIPPER_DELAY_START"))
      std::this_thread::sleep_for(std::chrono::milliseconds(130));
    stopped_ = false;  // Like an SDK start, an earlier stop cannot cancel this.
    log(action);
    if (std::getenv("FR3_GRIPPER_THROW")) throw std::runtime_error("Simulated command failure");
    double initial;
    { std::lock_guard<std::mutex> lock(mutex_); initial = state_.width; }
    int milliseconds = 80;
    if (const char* configured = std::getenv("FR3_GRIPPER_DURATION"))
      milliseconds = std::atoi(configured);
    for (int tick = 0; tick < milliseconds; tick += 5) {
      if (stopped_) { log("INTERRUPTED"); return false; }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::getenv("FR3_GRIPPER_NO_TRACKING"))
          state_.width = initial + (width - initial) * (tick + 5.0) / milliseconds;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (std::getenv("FR3_GRIPPER_FALSE")) return false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (action == "HOME") state_.max_width = .0801068;
      state_.is_grasped = action == "GRASP" && !std::getenv("FR3_GRIPPER_NO_GRASP");
    }
    log("DONE");
    return true;
  }
 public:
  explicit Gripper(const std::string&) {
    if (std::getenv("FR3_GRIPPER_UNHOMED")) state_.max_width = 0;
    if (std::getenv("FR3_GRIPPER_NARROW")) state_.max_width = .04;
  }
  GripperState readOnce() const {
    if (reading_.exchange(true)) throw std::runtime_error("Parallel readOnce");
    const auto count = ++reads_;
    if ((count == 1 && std::getenv("FR3_GRIPPER_INITIAL_STALL")) ||
        (count == 11 && std::getenv("FR3_GRIPPER_READ_STALL")))
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (count > 30 && std::getenv("FR3_GRIPPER_READ_FAIL")) {
      reading_ = false;
      throw std::runtime_error("Simulated read failure");
    }
    GripperState result;
    { std::lock_guard<std::mutex> lock(mutex_); result = state_; }
    reading_ = false;
    return result;
  }
  bool move(double width, double) const { return motion("MOVE", width); }
  bool grasp(double width, double, double, double, double) const { return motion("GRASP", width); }
  bool homing() const { return motion("HOME", .0801068); }
  bool stop() const {
    log("STOP");
    if (std::getenv("FR3_GRIPPER_STOP_THROW")) throw std::runtime_error("Simulated stop failure");
    if (std::getenv("FR3_GRIPPER_STOP_FALSE")) return false;
    stopped_ = true;
    { std::lock_guard<std::mutex> lock(mutex_); state_.is_grasped = false; }
    return true;
  }
};
}
'''


class NativeGripperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temporary = tempfile.TemporaryDirectory(prefix="fr3-gripper-native-test-")
        cls.directory = Path(cls.temporary.name)
        headers = cls.directory / "franka"
        headers.mkdir()
        (headers / "fake.h").write_text(FAKE_SDK)
        for name in ("gripper.h", "gripper_state.h"):
            (headers / name).write_text('#include "fake.h"\n')
        cls.binary = cls.directory / "franka-gripper"
        built = subprocess.run(
            [compiler, "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Wpedantic",
             "-I", str(cls.directory), str(Path(__file__).with_name("gripper.cpp")),
             "-o", str(cls.binary)], capture_output=True, text=True)
        if built.returncode:
            raise RuntimeError(built.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.processes = []
        self.lock_dir = tempfile.TemporaryDirectory(prefix="fr3-gripper-lock-test-")

    def tearDown(self):
        for process, _, _, _ in self.processes:
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

    def start(self, address=None, consume=True, **options):
        if address is None:
            address = f"127.0.0.{2 + len(self.processes)}"
        log = Path(self.lock_dir.name) / f"calls-{len(self.processes)}.txt"
        env = dict(os.environ, FRANKA_LOCK_DIR=self.lock_dir.name,
                   FR3_GRIPPER_LOG=str(log), **options)
        process = subprocess.Popen([str(self.binary), address], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1, env=env)
        received = queue.Queue()
        history = []

        def receive():
            for line in process.stdout:
                record = json.loads(line)
                history.append(record)
                received.put(record)

        if consume:
            threading.Thread(target=receive, daemon=True).start()
        item = process, received, history, log
        self.processes.append(item)
        return item

    def wait_for(self, item, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                message = item[1].get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if predicate(message):
                return message
        self.fail(f"Expected native event missing; observed {item[2][-8:]}")

    def ready(self, item):
        return self.wait_for(item, lambda r: r.get("type") == "state")

    def send(self, item, line):
        item[0].stdin.write(line + "\n")
        item[0].stdin.flush()

    def terminal(self, item, identifier="test"):
        return self.wait_for(item, lambda r: r.get("event") == "motion" and
                             r.get("command_id") == identifier and r["status"] in
                             ("succeeded", "failed", "cancelled", "rejected"))

    def calls(self, item):
        return item[3].read_text().splitlines() if item[3].exists() else []

    def run_motion(self, command="MOVE test .02 .03 30", **options):
        item = self.start(**options)
        self.ready(item)
        self.send(item, command)
        return item, self.terminal(item)

    def test_startup_and_idle_shutdown_do_not_move_home_or_stop(self):
        item = self.start()
        state = self.ready(item)
        self.assertTrue(state["control_ready"])
        self.assertIsNone(state["control_error"])
        self.assertGreater(state["max_width"], .08)
        self.assertEqual(state["temperature"], 27)
        self.send(item, "QUIT")
        self.assertEqual(item[0].wait(timeout=3), 0)
        self.assertEqual(self.calls(item), [])

    def test_move_reports_actual_width_after_sdk_completion(self):
        item, result = self.run_motion()
        self.assertEqual(result["status"], "succeeded")
        state = self.ready(item)
        self.assertAlmostEqual(state["width"], .03)

    def test_grasp_keeps_force_until_explicit_idle_stop(self):
        item, result = self.run_motion("GRASP test .02 .03 30 .005 .005 30")
        self.assertEqual(result["status"], "succeeded")
        self.assertTrue(self.ready(item)["is_grasped"])
        self.assertNotIn("STOP", self.calls(item))
        self.send(item, "STOP release")
        stop = self.wait_for(item, lambda r: r.get("event") == "stop")
        self.assertEqual((stop["command_id"], stop["status"]), ("release", "stopped"))
        state = self.wait_for(item, lambda r: r.get("type") == "state" and not r["is_grasped"])
        self.assertFalse(state["is_grasped"])
        self.assertIn("STOP", self.calls(item))

    def test_completed_grasp_is_not_stopped_by_normal_shutdown(self):
        item, result = self.run_motion("GRASP test .02 .03 30 .005 .005 30")
        self.assertEqual(result["status"], "succeeded")
        item[0].terminate()
        self.assertEqual(item[0].wait(timeout=3), 0)
        self.assertNotIn("STOP", self.calls(item))

    def test_unhomed_gripper_requires_explicit_home(self):
        item = self.start(FR3_GRIPPER_UNHOMED="1")
        state = self.ready(item)
        self.assertTrue(state["control_ready"])
        self.assertEqual(state["max_width"], 0)
        self.send(item, "MOVE before .02 .03 30")
        self.assertEqual(self.terminal(item, "before")["status"], "rejected")
        self.assertEqual(self.calls(item), [])
        self.send(item, "HOME home 30")
        self.assertEqual(self.terminal(item, "home")["status"], "succeeded")
        self.send(item, "MOVE after .02 .03 30")
        self.assertEqual(self.terminal(item, "after")["status"], "succeeded")

    def test_command_validation_and_calibrated_maximum(self):
        item = self.start(FR3_GRIPPER_NARROW="1")
        self.ready(item)
        invalid = ["MOVE {id} .02 .05 30", "MOVE {id} 0 .03 30", "MOVE {id} .101 .03 30",
                   "MOVE {id} .02 -.01 30", "MOVE {id} .02 .081 30", "MOVE {id} nan .03 30",
                   "MOVE {id} .02 inf 30", "MOVE {id} .02 .03 0", "MOVE {id} .02 .03 121",
                   "MOVE {id} .02 .03 30 trailing", "GRASP {id} .02 .03 29 .005 .005 30",
                   "GRASP {id} .02 .03 71 .005 .005 30", "GRASP {id} .02 .03 30 -.001 .005 30",
                   "GRASP {id} .02 .03 30 .005 .081 30", "HOME {id} 0", "HOME {id} nan"]
        for index, command in enumerate(invalid):
            identifier = f"bad{index}"
            with self.subTest(command=command):
                self.send(item, command.format(id=identifier))
                self.assertEqual(self.terminal(item, identifier)["status"], "rejected")
        self.assertEqual(self.calls(item), [])

    def test_sdk_false_and_exceptions_report_failure(self):
        for option in ("FR3_GRIPPER_FALSE", "FR3_GRIPPER_THROW"):
            with self.subTest(option=option):
                _, result = self.run_motion(**{option: "1"})
                self.assertEqual(result["status"], "failed")

    def test_success_requires_actual_final_state(self):
        for command, options in [
            ("MOVE test .02 .03 30", {"FR3_GRIPPER_NO_TRACKING": "1"}),
            ("GRASP test .02 .03 30 .005 .005 30", {"FR3_GRIPPER_NO_GRASP": "1"}),
        ]:
            with self.subTest(command=command):
                _, result = self.run_motion(command, **options)
                self.assertEqual(result["status"], "failed")
                self.assertIn("final state", result["error"])

    def test_stop_reaches_sdk_and_waits_for_worker(self):
        item = self.start(FR3_GRIPPER_DURATION="1000")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "STOP test")
        self.assertEqual(self.terminal(item)["status"], "cancelled")
        self.assertEqual(self.wait_for(item, lambda r: r.get("event") == "stop")["status"], "stopped")
        self.assertIn("STOP", self.calls(item))
        self.assertIn("INTERRUPTED", self.calls(item))

    def test_stop_before_sdk_start_cannot_leave_a_later_motion_running(self):
        item = self.start(FR3_GRIPPER_DELAY_START="1", FR3_GRIPPER_DURATION="600")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "STOP test")
        self.assertEqual(self.terminal(item)["status"], "cancelled")
        calls = self.calls(item)
        self.assertIn("INTERRUPTED", calls)
        self.assertGreater(calls.count("STOP"), 1)

    def test_busy_and_wrong_stop_do_not_interrupt_or_queue(self):
        item = self.start(FR3_GRIPPER_DURATION="400")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "MOVE second .02 .02 30")
        self.assertEqual(self.terminal(item, "second")["status"], "rejected")
        self.send(item, "STOP wrong")
        self.assertEqual(self.wait_for(item, lambda r: r.get("event") == "stop")["status"], "idle")
        self.assertEqual(self.terminal(item)["status"], "succeeded")
        self.assertEqual(self.calls(item).count("MOVE"), 1)
        self.assertNotIn("STOP", self.calls(item))

    def test_completed_command_id_cannot_be_replayed(self):
        item, result = self.run_motion()
        self.assertEqual(result["status"], "succeeded")
        self.send(item, "MOVE test .02 .02 30")
        self.assertEqual(self.terminal(item)["status"], "rejected")
        self.assertEqual(self.calls(item).count("MOVE"), 1)

    def test_stop_failure_is_not_cancelled(self):
        for option in ("FR3_GRIPPER_STOP_FALSE", "FR3_GRIPPER_STOP_THROW"):
            with self.subTest(option=option):
                item = self.start(FR3_GRIPPER_DURATION="200", **{option: "1"})
                self.ready(item)
                self.send(item, "MOVE test .02 .03 30")
                self.wait_for(item, lambda r: r.get("status") == "running")
                self.send(item, "STOP test")
                result = self.terminal(item)
                self.assertEqual(result["status"], "failed")
                self.assertIn("stop", result["error"])
                self.assertEqual(self.wait_for(item, lambda r: r.get("event") == "stop")["status"], "failed")

    def test_idle_stop_failure_is_reported(self):
        item = self.start(FR3_GRIPPER_STOP_FALSE="1")
        self.ready(item)
        self.send(item, "STOP idle")
        result = self.wait_for(item, lambda r: r.get("event") == "stop")
        self.assertEqual(result["status"], "failed")

    def test_timeout_stops_sdk_and_reports_failure(self):
        item, result = self.run_motion("MOVE test .02 .03 1", FR3_GRIPPER_DURATION="3000")
        self.assertEqual(result["status"], "failed")
        self.assertIn("timed out", result["error"])
        self.assertIn("INTERRUPTED", self.calls(item))

    def test_eof_signal_and_quit_stop_active_motion(self):
        for shutdown in ("eof", "signal", "quit"):
            with self.subTest(shutdown=shutdown):
                item = self.start(FR3_GRIPPER_DURATION="1500")
                self.ready(item)
                self.send(item, "MOVE test .02 .03 30")
                self.wait_for(item, lambda r: r.get("status") == "running")
                if shutdown == "eof":
                    item[0].stdin.close()
                elif shutdown == "signal":
                    item[0].terminate()
                else:
                    self.send(item, "QUIT")
                self.assertEqual(item[0].wait(timeout=3), 0)
                self.assertIn("STOP", self.calls(item))
                self.assertIn("INTERRUPTED", self.calls(item))

    def test_read_failure_stops_active_motion_and_exits(self):
        item = self.start(FR3_GRIPPER_READ_FAIL="1", FR3_GRIPPER_DURATION="1500")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        result = self.terminal(item)
        self.assertEqual(result["status"], "failed")
        self.assertIn("read failure", result["error"])
        self.assertEqual(item[0].wait(timeout=3), 1)
        self.assertIn("STOP", self.calls(item))

    def test_stale_read_is_not_published_as_ready_and_stops_motion(self):
        item = self.start(FR3_GRIPPER_READ_STALL="1", FR3_GRIPPER_DURATION="3000")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        result = self.terminal(item)
        self.assertEqual(result["status"], "failed")
        self.assertIn("Fresh gripper state", result["error"])
        self.assertEqual(item[0].wait(timeout=3), 1)
        self.assertIn("STOP", self.calls(item))
        states = [record for record in item[2] if record.get("type") == "state"]
        # No stale snapshot is re-emitted, even as control_ready=false: consumers
        # timestamp received samples, so doing that would hide its actual age.
        self.assertTrue(all(state["control_ready"] for state in states))

    def test_initial_read_timeout_never_announces_control_ready(self):
        item = self.start(FR3_GRIPPER_INITIAL_STALL="1")
        self.assertEqual(item[0].wait(timeout=3), 1)
        self.assertFalse(any(record.get("control_ready") for record in item[2]))
        self.assertEqual(self.calls(item), [])

    def test_closed_output_pipe_stops_active_motion(self):
        item = self.start(consume=False, FR3_GRIPPER_DURATION="1500")
        self.assertEqual(json.loads(item[0].stdout.readline())["type"], "state")
        self.send(item, "MOVE test .02 .03 30")
        while json.loads(item[0].stdout.readline()).get("status") != "running":
            pass
        item[0].stdout.close()
        self.assertEqual(item[0].wait(timeout=3), 1)
        self.assertIn("STOP", self.calls(item))

    def test_oversized_input_stops_active_motion(self):
        item = self.start(FR3_GRIPPER_DURATION="1500")
        self.ready(item)
        self.send(item, "MOVE test .02 .03 30")
        self.wait_for(item, lambda r: r.get("status") == "running")
        self.send(item, "x" * 5000)
        self.assertEqual(item[0].wait(timeout=3), 1)
        self.assertIn("STOP", self.calls(item))

    def test_lock_is_per_ip_canonical_and_separate_from_arm(self):
        arm_lock = Path(self.lock_dir.name) / "127.0.0.2.lock"
        with arm_lock.open("w") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            first = self.start()
            self.ready(first)
            second = self.start(address="::ffff:127.0.0.2")
            self.assertEqual(second[0].wait(timeout=3), 1)
            self.assertIn("Another Franka gripper", second[0].stderr.read())
            other = self.start(address="127.0.0.3")
            self.ready(other)


if __name__ == "__main__":
    unittest.main()
