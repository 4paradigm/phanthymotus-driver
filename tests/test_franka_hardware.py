"""The read-only bridge is supervised without hardware or ROS dependencies."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest

SPEC = importlib.util.spec_from_file_location(
    "franka_hardware_test", Path(__file__).resolve().parents[1] / "franka/fr3/hardware.py")
hardware = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hardware)

STREAM = '''
import json, sys, time
value = float(sys.argv[1].split('.')[-1])
while True:
    print(json.dumps(dict(q=[value]*7, dq=[0]*7, tau_J=[0]*7, robot_mode=1)), flush=True)
    time.sleep(0.01)
'''


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Reader condition did not become true before the deadline")


class ReaderFactory:
    def __init__(self, scripts=None):
        self.scripts = scripts or [STREAM]
        self.processes = []
        self.addresses = []
        self.lock = threading.Lock()

    def __call__(self, args, **kwargs):
        with self.lock:
            script = self.scripts[min(len(self.processes), len(self.scripts) - 1)]
            process = subprocess.Popen([sys.executable, "-u", "-c", script, args[1]], **kwargs)
            self.processes.append(process)
            self.addresses.append(args[1])
            return process


class SessionTests(unittest.TestCase):
    def manager(self, factory, **options):
        defaults = dict(connect_timeout=0.5, stale_after=0.3, retry_delay=0.1,
                        retry_max=0.2, stop_timeout=0.15)
        defaults.update(options)
        manager = hardware.SessionManager(process_factory=factory, **defaults)
        self.addCleanup(manager.close)
        return manager

    def test_ip_validation_and_equivalent_literals(self):
        self.assertEqual(hardware.canonical_ip(" 172.16.0.3 "), "172.16.0.3")
        self.assertEqual(hardware.canonical_ip("2001:0db8:0:0:0:0:0:1"), "2001:db8::1")
        self.assertEqual(hardware.canonical_ip("::ffff:172.16.0.3"), "172.16.0.3")
        for value in (None, "", "robot.local", "172.16.0.3:80", "172.16.0.3/24", "fe80::1%eth0"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                hardware.canonical_ip(value)

    def test_same_ip_shares_reader_and_last_owner_releases_it(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        first = manager.acquire("172.16.0.3", ("joint_states", "one"))
        other_card = manager.acquire("::ffff:172.16.0.3", ("joint_states", "two"))
        self.assertIs(first, other_card)
        self.assertIs(first, manager.acquire("172.16.0.3", ("joint_states", "one")))
        wait_for(lambda: first.snapshot()["connected"])
        self.assertEqual(len(factory.processes), 1)
        manager.release("172.16.0.3", ("joint_states", "one"))
        self.assertIsNone(factory.processes[0].poll())
        self.assertTrue(other_card.snapshot()["connected"])
        manager.release("172.16.0.3", ("joint_states", "two"))
        self.assertIsNotNone(factory.processes[0].poll())
        self.assertFalse(first.snapshot()["connected"])
        self.assertNotIn("q", first.snapshot())
        manager.release("172.16.0.3", ("joint_states", "two"))

    def test_different_robots_are_isolated_and_snapshots_are_copies(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        first = manager.acquire("172.16.0.3", ("joint_states", "one"))
        second = manager.acquire("172.16.0.4", ("joint_states", "two"))
        wait_for(lambda: first.snapshot()["connected"] and second.snapshot()["connected"])
        snapshot = first.snapshot()
        self.assertEqual(snapshot["q"], [3] * 7)
        self.assertEqual(second.snapshot()["q"], [4] * 7)
        snapshot["q"][0] = 123
        self.assertEqual(first.snapshot()["q"][0], 3)
        self.assertGreater(first.snapshot()["timestamp_ms"], 0)
        manager.release("172.16.0.3", ("joint_states", "one"))
        self.assertTrue(second.snapshot()["connected"])

    def test_stale_sample_disappears_and_reader_is_restarted(self):
        record = json.dumps(dict(q=[0] * 7, dq=[0] * 7, tau_J=[0] * 7, robot_mode=1))
        script = f"import time; print({record!r}, flush=True); time.sleep(20)"
        factory = ReaderFactory([script, STREAM])
        manager = self.manager(factory, retry_delay=0.3)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["connected"])
        first_timestamp = session.snapshot()["timestamp_ms"]
        wait_for(lambda: not session.snapshot()["connected"])
        stale = session.snapshot()
        self.assertNotIn("q", stale)
        self.assertGreaterEqual(stale["data_age_ms"], 300)
        wait_for(lambda: len(factory.processes) >= 2 and session.snapshot()["connected"])
        self.assertIsNotNone(factory.processes[0].poll())
        self.assertEqual(session.snapshot()["q"], [3] * 7)
        self.assertGreater(session.snapshot()["timestamp_ms"], first_timestamp)

    def test_bad_state_cannot_mark_robot_connected(self):
        good = dict(q=[0] * 7, dq=[0] * 7, tau_J=[0] * 7, robot_mode=1)
        records = [[], {**good, "q": [0] * 6}, {**good, "q": [float("nan")] * 7},
                   {**good, "dq": [True] * 7}, {**good, "tau_J": ["0"] * 7},
                   {**good, "robot_mode": "idle"}]
        for record in records:
            with self.subTest(record=record):
                encoded = json.dumps(record)
                factory = ReaderFactory([f"import time; print({encoded!r}, flush=True); time.sleep(20)"])
                manager = self.manager(factory, retry_delay=2)
                session = manager.acquire("172.16.0.3", ("joint_states", "one"))
                wait_for(lambda session=session: session.snapshot()["error"] is not None)
                snapshot = session.snapshot()
                self.assertFalse(snapshot["connected"])
                self.assertIsNone(snapshot["data_age_ms"])
                self.assertNotIn("q", snapshot)
                manager.close()

    def test_deeply_nested_json_is_rejected_and_supervisor_recovers(self):
        # Smaller than the byte limit, but deeper than Python's JSON parser can
        # decode. This must not silently end the supervision thread.
        nested = "[" * 1500 + "0" + "]" * 1500
        script = f"import time; print({nested!r}, flush=True); time.sleep(20)"
        factory = ReaderFactory([script, STREAM])
        manager = self.manager(factory)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: len(factory.processes) >= 2 and session.snapshot()["connected"])
        self.assertIsNotNone(factory.processes[0].poll())
        self.assertEqual(session.snapshot()["q"], [3] * 7)
        self.assertIsNone(session.snapshot()["error"])

    def test_missing_reader_reports_failure_then_recovers(self):
        factory = ReaderFactory()
        attempts = []

        def missing_once(args, **kwargs):
            attempts.append(args)
            if len(attempts) == 1:
                raise FileNotFoundError("Reader executable is unavailable")
            return factory(args, **kwargs)

        manager = self.manager(missing_once, retry_delay=0.3)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["error"] is not None)
        self.assertFalse(session.snapshot()["connected"])
        self.assertNotIn("q", session.snapshot())
        self.assertIn("unavailable", session.snapshot()["error"])
        wait_for(lambda: session.snapshot()["connected"])
        self.assertEqual(len(attempts), 2)
        self.assertEqual(len(factory.processes), 1)
        self.assertIsNone(session.snapshot()["error"])

    def test_connect_watchdog_kills_reader_ignoring_sigterm(self):
        factory = ReaderFactory(["import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(20)"])
        manager = self.manager(factory, connect_timeout=0.15, retry_delay=2)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["error"] is not None)
        self.assertIn("first robot state", session.snapshot()["error"])
        wait_for(lambda: factory.processes[0].poll() is not None)
        self.assertLess(factory.processes[0].returncode, 0)

    def test_close_kills_blocked_reader_and_is_idempotent(self):
        factory = ReaderFactory(["import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(20)"])
        manager = self.manager(factory)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: len(factory.processes) == 1)
        started = time.monotonic()
        manager.close()
        self.assertLess(time.monotonic() - started, 1.5)
        self.assertIsNotNone(factory.processes[0].poll())
        self.assertEqual(session.snapshot()["state"], "stopped")
        manager.close()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            manager.acquire("172.16.0.3", ("joint_states", "one"))

    def test_concurrent_manager_close_waits_until_reader_cleanup_finishes(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["connected"])
        entered, finish, second_finished = threading.Event(), threading.Event(), threading.Event()
        original_close = session.close
        failures = []

        def delayed_close():
            entered.set()
            finish.wait(2)
            original_close()

        def close_manager(done=None):
            try:
                manager.close()
            except Exception as exc:
                failures.append(exc)
            finally:
                if done is not None:
                    done.set()

        session.close = delayed_close
        first = threading.Thread(target=close_manager)
        second = threading.Thread(target=close_manager, args=(second_finished,))
        first.start()
        self.assertTrue(entered.wait(1))
        try:
            second.start()
            self.assertFalse(second_finished.wait(0.1), "close returned before reader cleanup completed")
        finally:
            finish.set()
            first.join(2)
            second.join(2)
            session.close = original_close
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(failures, [])
        self.assertIsNotNone(factory.processes[0].poll())

    def test_stderr_is_drained_and_error_memory_is_bounded(self):
        factory = ReaderFactory(["import sys; sys.stderr.write('x'*50000); sys.stderr.flush(); sys.exit(7)"])
        manager = self.manager(factory, retry_delay=2)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["error"] is not None)
        self.assertLessEqual(len(session.snapshot()["error"]), 4096)
        self.assertEqual(session.snapshot()["error"], "x" * 4096)

    def test_unbounded_stdout_is_rejected_without_waiting_for_newline(self):
        factory = ReaderFactory(["import sys,time; sys.stdout.write('x'*20000); sys.stdout.flush(); time.sleep(20)"])
        manager = self.manager(factory, retry_delay=2)
        session = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: session.snapshot()["error"] is not None)
        self.assertIn("8192", session.snapshot()["error"])
        self.assertNotIn("q", session.snapshot())

    def test_reacquire_waits_for_old_reader_without_blocking_another_robot(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        old = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: old.snapshot()["connected"])
        closing, finish = threading.Event(), threading.Event()
        original_close = old.close

        def delayed_close():
            closing.set()
            finish.wait(2)
            original_close()

        old.close = delayed_close
        release = threading.Thread(target=manager.release, args=("172.16.0.3", ("joint_states", "one")))
        release.start()
        self.assertTrue(closing.wait(1))
        received = []
        acquire = threading.Thread(target=lambda: received.append(manager.acquire("172.16.0.3", ("joint_states", "new"))))
        acquire.start()
        try:
            second = manager.acquire("172.16.0.4", ("joint_states", "two"))
            wait_for(lambda: second.snapshot()["connected"])
            self.assertEqual(received, [])
        finally:
            finish.set()
            release.join(2)
            acquire.join(2)
        self.assertEqual(len(received), 1)
        self.assertIsNot(received[0], old)
        self.assertIsNotNone(factory.processes[0].poll())
        wait_for(lambda: received[0].snapshot()["connected"])

    def test_failed_release_cannot_return_a_stopping_or_stopped_session(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        old = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: old.snapshot()["connected"])
        disposing, finish = threading.Event(), threading.Event()
        dispose = old._dispose
        original_close = old.close

        def delayed_dispose(process):
            disposing.set()
            finish.wait(2)
            dispose(process)

        def failed_close():
            old._stop.set()
            raise RuntimeError("Reader is still terminating")

        old._dispose = delayed_dispose
        old.close = failed_close
        try:
            with self.assertRaisesRegex(RuntimeError, "terminating"):
                manager.release("172.16.0.3", ("joint_states", "one"))
            self.assertTrue(disposing.wait(1))
            with self.assertRaisesRegex(RuntimeError, "still stopping"):
                manager.acquire("172.16.0.3", ("joint_states", "two"))
            self.assertEqual(len(factory.processes), 1)
        finally:
            old.close = original_close
            finish.set()
        wait_for(lambda: old.stopped)
        new = manager.acquire("172.16.0.3", ("joint_states", "two"))
        self.assertIsNot(new, old)
        wait_for(lambda: new.snapshot()["connected"])

    def test_shutdown_observes_concurrent_release_failure_and_can_retry(self):
        factory = ReaderFactory()
        manager = self.manager(factory)
        old = manager.acquire("172.16.0.3", ("joint_states", "one"))
        wait_for(lambda: old.snapshot()["connected"])
        closing, finish = threading.Event(), threading.Event()
        original_close = old.close
        calls = []

        def failed_close():
            calls.append(True)
            if len(calls) == 1:
                closing.set()
                finish.wait(2)
            raise RuntimeError("Simulated cleanup failure")

        def capture(action, errors):
            try:
                action()
            except RuntimeError as exc:
                errors.append(str(exc))

        old.close = failed_close
        release_errors, shutdown_errors = [], []
        release = threading.Thread(target=capture, args=(
            lambda: manager.release("172.16.0.3", ("joint_states", "one")), release_errors))
        shutdown = threading.Thread(target=capture, args=(manager.close, shutdown_errors))
        release.start()
        self.assertTrue(closing.wait(1))
        shutdown.start()
        try:
            wait_for(lambda: manager._closed)
        finally:
            finish.set()
            release.join(2)
            shutdown.join(2)
            old.close = original_close
        self.assertEqual(release_errors, ["Simulated cleanup failure"])
        self.assertEqual(shutdown_errors, ["Simulated cleanup failure"])
        manager.close()
        self.assertIsNotNone(factory.processes[0].poll())


if __name__ == "__main__":
    unittest.main()
