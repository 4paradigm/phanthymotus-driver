"""Exercise the real command pipe and session ownership without moving hardware."""

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest


SPEC = importlib.util.spec_from_file_location(
    "franka_control_session_test", Path(__file__).resolve().parents[1] / "franka/fr3/hardware.py")
hardware = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hardware)

TARGET = [0.1, -0.5, 0, -1.6, 0, 1.6, 0]
BACKEND = r'''
import json, os, select, sys, time
mode, log_path = sys.argv[1:]
sys.stdin = os.fdopen(sys.stdin.fileno(), "rb", buffering=0)
active = None
emitted = False
input_closed = False
started = stop_at = None
q = [0, -0.5, 0, -1.6, 0, 1.6, 0]
def emit(**record):
    print(json.dumps(record), flush=True)
def event(status, **fields):
    emit(event="motion", command_id=active, status=status, **fields)
while True:
    now = time.monotonic()
    if not (active and mode == "event_only") and not (emitted and mode == "stale"):
        emit(q=q, q_d=q, dq=[0]*7, tau_J=[0]*7, robot_mode=1,
             rt_kernel=mode == "rt_success", sched_fifo=mode != "no_fifo",
             control_ready=mode != "no_fifo",
             control_error="SCHED_FIFO permission denied" if mode == "no_fifo" else None)
        emitted = True
    if active and mode == "event_only":
        event("running")
    if active and mode in ("success", "rt_success") and now - started > 0.1:
        event("succeeded", max_error=0.001)
        active = None
    if active and stop_at is not None and now >= stop_at:
        event("cancelled")
        emit(event="stop", command_id=active, status="stopped")
        active = None
        stop_at = None
    ready, _, _ = select.select([] if input_closed else [sys.stdin], [], [], 0.01)
    if ready:
        line = sys.stdin.readline().decode()
        if not line:
            break
        with open(log_path, "a") as handle:
            handle.write(line)
        args = line.split()
        if args[0] == "MOVE":
            active, started = args[1], time.monotonic()
            q = [float(value) for value in args[5:]]
            if mode == "disconnect":
                sys.exit(3)
            if mode == "reject":
                event("rejected", error="Robot is not idle")
                active = None
            elif mode == "malformed":
                event("succeeded", max_error=float("nan"))
            else:
                if mode == "close_input":
                    sys.stdin.close()
                    input_closed = True
                event("accepted")
                event("running")
        elif args[0] == "STOP" and active:
            if mode == "stop_failed":
                emit(event="stop", command_id=active, status="failed", error="stop failed")
            elif mode == "stop_only":
                emit(event="stop", command_id=active, status="stopped")
            elif mode != "ignore_stop":
                stop_at = time.monotonic() + 0.1
'''


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Session condition did not become true")


class NativeFactory:
    def __init__(self, path, mode):
        self.path, self.mode = path, mode
        self.processes = []
        self.lock = threading.Lock()

    def __call__(self, args, **kwargs):
        with self.lock:
            process = subprocess.Popen(
                [sys.executable, "-u", "-c", BACKEND, self.mode, self.path], **kwargs)
            self.processes.append(process)
            return process

    def commands(self):
        return Path(self.path).read_text().splitlines()


class ControlSessionTests(unittest.TestCase):
    def session(self, mode="hold", **options):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = str(Path(directory.name) / "commands.log")
        Path(path).touch()
        factory = NativeFactory(path, mode)
        defaults = dict(connect_timeout=0.3, stale_after=0.15, retry_delay=0.1,
                        retry_max=0.2, stop_timeout=0.1, motion_stop_timeout=0.4)
        defaults.update(options)
        manager = hardware.SessionManager(process_factory=factory, **defaults)
        self.addCleanup(manager.close)
        owner = ("joint_control", "first")
        session = manager.acquire("172.16.0.3", owner)
        eventually(lambda: session.snapshot()["connected"])
        return manager, session, factory, owner

    def test_move_uses_one_native_process_and_waits_for_backend_completion(self):
        manager, session, factory, owner = self.session("success")
        self.assertIs(session, manager.acquire("172.16.0.3", ("joint_states", "reader")))
        command_id = session.move(owner, TARGET)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual(result["target"], TARGET)
        self.assertEqual(result["max_error"], 0.001)
        self.assertEqual(len(factory.processes), 1)
        fields = factory.commands()[0].split()
        self.assertEqual(fields[:2], ["MOVE", command_id])
        self.assertAlmostEqual(float(fields[2]), math.radians(5))
        self.assertAlmostEqual(float(fields[3]), math.radians(0.5))
        self.assertEqual(float(fields[4]), 120)
        self.assertEqual([float(value) for value in fields[5:]], TARGET)
        self.assertIn("q_d", session.snapshot())
        self.assertFalse(session.snapshot()["rt_kernel"])
        self.assertTrue(session.snapshot()["sched_fifo"])
        self.assertTrue(session.snapshot()["control_ready"])
        self.assertIsNone(session.snapshot()["control_error"])

    def test_rt_backend_still_completes_motion(self):
        _, session, _, owner = self.session("rt_success")
        self.assertTrue(session.snapshot()["rt_kernel"])
        command_id = session.move(owner, TARGET)
        self.assertEqual(session.wait(command_id, 2)["status"], "succeeded")

    def test_same_ip_concurrent_moves_are_rejected_without_queueing(self):
        manager, session, factory, owner = self.session()
        other = ("joint_control", "second")
        manager.acquire("172.16.0.3", other)
        command_id = session.move(owner, TARGET)
        with self.assertRaisesRegex(RuntimeError, "active motion"):
            session.move(other, TARGET)
        with self.assertRaisesRegex(RuntimeError, "active motion"):
            session.move(owner, TARGET)
        eventually(lambda: len(factory.commands()) == 1)
        self.assertEqual(session.motion(command_id)["command_id"], command_id)

    def test_simultaneous_submissions_have_exactly_one_winner(self):
        manager, session, factory, owner = self.session()
        other = ("joint_control", "second")
        manager.acquire("172.16.0.3", other)
        barrier = threading.Barrier(3)
        successes, failures = [], []

        def submit(selected_owner):
            barrier.wait()
            try:
                successes.append(session.move(selected_owner, TARGET))
            except RuntimeError as error:
                failures.append(str(error))

        workers = [threading.Thread(target=submit, args=(selected,)) for selected in (owner, other)]
        for worker in workers:
            worker.start()
        barrier.wait()
        for worker in workers:
            worker.join(1)
            self.assertFalse(worker.is_alive())
        self.assertEqual(len(successes), 1)
        self.assertEqual(len(failures), 1)
        self.assertIn("active motion", failures[0])
        eventually(lambda: len(factory.commands()) == 1)

    def test_different_robot_sessions_can_move_independently(self):
        manager, first, factory, owner = self.session("success")
        second = manager.acquire("172.16.0.4", owner)
        eventually(lambda: second.snapshot()["connected"])
        first_id = first.move(owner, TARGET)
        second_id = second.move(owner, TARGET)
        self.assertEqual(first.wait(first_id, 2)["status"], "succeeded")
        self.assertEqual(second.wait(second_id, 2)["status"], "succeeded")
        self.assertEqual(len(factory.processes), 2)

    def test_stop_is_sent_and_ownership_is_held_until_confirmed(self):
        _, session, factory, owner = self.session()
        command_id = session.move(owner, TARGET)
        result = session.stop_motion(owner, command_id)
        self.assertEqual(result["status"], "stopping")
        with self.assertRaises(TimeoutError):
            session.wait(command_id, 0.01)
        with self.assertRaisesRegex(RuntimeError, "active motion"):
            session.move(owner, TARGET)
        self.assertEqual(session.wait(command_id, 2)["status"], "cancelled")
        self.assertIn("STOP " + command_id, factory.commands())

    def test_only_owner_can_cancel_but_explicit_stop_is_robot_wide(self):
        _, session, _, owner = self.session()
        command_id = session.move(owner, TARGET)
        with self.assertRaisesRegex(RuntimeError, "another control card"):
            session.stop_motion(("joint_control", "other"), command_id)
        session.stop_motion()
        self.assertEqual(session.wait(command_id, 2)["status"], "cancelled")

    def test_releasing_controller_stops_motion_and_keeps_reader(self):
        manager, session, factory, owner = self.session()
        manager.acquire("172.16.0.3", ("joint_states", "reader"))
        command_id = session.move(owner, TARGET)
        manager.release("172.16.0.3", owner)
        self.assertEqual(session.motion(command_id)["status"], "cancelled")
        self.assertIn("STOP " + command_id, factory.commands())
        self.assertIsNone(factory.processes[0].poll())
        self.assertTrue(session.snapshot()["connected"])
        with self.assertRaisesRegex(RuntimeError, "does not own"):
            session.move(owner, TARGET)

    def test_releasing_state_card_does_not_stop_another_cards_motion(self):
        manager, session, factory, owner = self.session()
        reader = ("joint_states", "reader")
        manager.acquire("172.16.0.3", reader)
        command_id = session.move(owner, TARGET)
        manager.release("172.16.0.3", reader)
        eventually(lambda: session.motion(command_id)["status"] == "running")
        self.assertEqual(len(factory.commands()), 1)

    def test_last_release_and_shutdown_stop_before_terminating_native(self):
        for shutdown in (False, True):
            with self.subTest(shutdown=shutdown):
                manager, session, factory, owner = self.session()
                command_id = session.move(owner, TARGET)
                if shutdown:
                    manager.close()
                else:
                    manager.release("172.16.0.3", owner)
                self.assertEqual(session.motion(command_id)["status"], "cancelled")
                self.assertIn("STOP " + command_id, factory.commands())
                self.assertIsNotNone(factory.processes[0].poll())

    def test_disconnect_fails_action_and_never_replays_it(self):
        _, session, factory, owner = self.session("disconnect")
        command_id = session.move(owner, TARGET)
        self.assertEqual(session.wait(command_id, 2)["status"], "failed")
        eventually(lambda: len(factory.processes) >= 2 and session.snapshot()["connected"])
        self.assertEqual(len(factory.commands()), 1)
        self.assertIn("exited", session.motion(command_id)["error"])

    def test_backend_rejection_is_terminal_and_not_success(self):
        _, session, _, owner = self.session("reject")
        command_id = session.move(owner, TARGET)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "rejected")
        self.assertEqual(result["error"], "Robot is not idle")

    def test_missing_fifo_rejects_before_any_write(self):
        _, session, factory, owner = self.session("no_fifo")
        with self.assertRaisesRegex(RuntimeError, "SCHED_FIFO"):
            session.move(owner, TARGET)
        self.assertFalse(session.snapshot()["control_ready"])
        self.assertFalse(session.snapshot()["rt_kernel"])
        self.assertFalse(session.snapshot()["sched_fifo"])
        self.assertEqual(factory.commands(), [])

    def test_stale_state_rejects_before_any_write(self):
        _, stale_session, stale_factory, stale_owner = self.session("stale", retry_delay=1)
        eventually(lambda: not stale_session.snapshot()["connected"])
        with self.assertRaisesRegex(RuntimeError, "Fresh"):
            stale_session.move(stale_owner, TARGET)
        self.assertEqual(stale_factory.commands(), [])

    def test_invalid_target_and_parameters_never_reach_native(self):
        _, session, factory, owner = self.session()
        cases = [([0] * 6, {}), ([True] * 7, {}), ([float("nan")] * 7, {}),
                 (["0"] * 7, {}), ([0] * 7, {}),
                 (TARGET, {"speed_rad_s": True}), (TARGET, {"speed_rad_s": 1}),
                 (TARGET, {"tolerance_rad": 0}), (TARGET, {"tolerance_rad": float("inf")}),
                 (TARGET, {"timeout_s": 0}), (TARGET, {"timeout_s": 121})]
        for target, kwargs in cases:
            with self.subTest(target=target, kwargs=kwargs), self.assertRaises(ValueError):
                session.move(owner, target, **kwargs)
        self.assertEqual(factory.commands(), [])

    def test_motion_events_do_not_keep_stale_robot_state_alive(self):
        _, session, _, owner = self.session("event_only", retry_delay=0.3)
        command_id = session.move(owner, TARGET)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertIn("stale", result["error"])
        self.assertNotIn("q", session.snapshot())

    def test_malformed_completion_fails_connection(self):
        _, session, _, owner = self.session("malformed")
        command_id = session.move(owner, TARGET)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertIn("max_error", result["error"])

    def test_failed_stop_fails_action_and_restarts_connection(self):
        _, session, factory, owner = self.session("stop_failed")
        command_id = session.move(owner, TARGET)
        session.stop_motion(owner)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"], "stop failed")
        eventually(lambda: len(factory.processes) >= 2 and session.snapshot()["connected"])
        self.assertIsNotNone(factory.processes[0].poll())
        self.assertEqual(len(factory.commands()), 2)

    def test_unacknowledged_stop_is_bounded_and_forces_connection_cleanup(self):
        manager, session, factory, owner = self.session("ignore_stop", motion_stop_timeout=0.15)
        command_id = session.move(owner, TARGET)
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "not confirmed"):
            manager.close()
        self.assertLess(time.monotonic() - started, 1)
        self.assertEqual(session.motion(command_id)["status"], "failed")
        self.assertIn("not confirmed", session.snapshot()["error"])
        self.assertIsNotNone(factory.processes[0].poll())

    def test_explicit_stop_watchdog_fails_instead_of_holding_ownership_forever(self):
        _, session, factory, owner = self.session("ignore_stop", motion_stop_timeout=0.15)
        command_id = session.move(owner, TARGET)
        session.stop_motion(owner)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertIn("stop was not acknowledged", result["error"])
        eventually(lambda: len(factory.processes) >= 2 and session.snapshot()["connected"])
        self.assertIsNotNone(factory.processes[0].poll())

    def test_stop_ack_without_motion_terminal_never_counts_as_completion(self):
        _, session, _, owner = self.session("stop_only", motion_stop_timeout=0.15)
        command_id = session.move(owner, TARGET)
        session.stop_motion(owner)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["stop_status"], "stopped")

    def test_owner_release_reports_real_broken_command_pipe(self):
        manager, session, factory, owner = self.session("close_input")
        manager.acquire("172.16.0.3", ("joint_states", "reader"))
        command_id = session.move(owner, TARGET)
        eventually(lambda: session.motion(command_id)["status"] == "running")
        with self.assertRaisesRegex(RuntimeError, "not confirmed.*command channel"):
            manager.release("172.16.0.3", owner)
        self.assertEqual(session.motion(command_id)["status"], "failed")
        eventually(lambda: factory.processes[0].poll() is not None)

    def test_owner_release_reports_backend_stop_failure(self):
        manager, session, factory, owner = self.session("stop_failed")
        manager.acquire("172.16.0.3", ("joint_states", "reader"))
        command_id = session.move(owner, TARGET)
        with self.assertRaisesRegex(RuntimeError, "not confirmed.*stop failed"):
            manager.release("172.16.0.3", owner)
        self.assertEqual(session.motion(command_id)["status"], "failed")
        eventually(lambda: factory.processes[0].poll() is not None)

    def test_last_release_cleans_up_after_stop_failure_and_reports_it(self):
        manager, session, factory, owner = self.session("stop_failed")
        command_id = session.move(owner, TARGET)
        with self.assertRaisesRegex(RuntimeError, "not confirmed.*stop failed"):
            manager.release("172.16.0.3", owner)
        self.assertEqual(session.motion(command_id)["status"], "failed")
        self.assertIsNotNone(factory.processes[0].poll())

    def test_terminal_results_survive_later_moves_and_duplicate_events(self):
        _, session, _, owner = self.session("success")
        first = session.move(owner, TARGET)
        eventually(lambda: session.motion(first)["status"] == "succeeded")
        second = session.move(owner, TARGET)
        session._accept(json.dumps(dict(event="motion", command_id=first, status="failed")))
        self.assertEqual(session.wait(first, 0)["status"], "succeeded")
        self.assertEqual(session.wait(second, 2)["status"], "succeeded")
        snapshot = session.snapshot()
        snapshot["motion"]["target"][0] = 999
        self.assertEqual(session.motion(second)["target"], TARGET)


if __name__ == "__main__":
    unittest.main()
