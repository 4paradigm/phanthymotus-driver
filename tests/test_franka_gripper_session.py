"""Gripper session protocol/lifecycle checks with a real pipe and no hardware."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


DRIVER = Path(__file__).resolve().parents[1] / "franka/fr3"


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


hardware = load_module("franka_gripper_test_hardware", "hardware.py")
with mock.patch.dict(sys.modules, {"hardware": hardware}):
    gripper = load_module("franka_gripper_test_backend", "gripper_hardware.py")


STATE = dict(width=0.04, max_width=0.08, is_grasped=False, temperature=29,
             control_ready=True, control_error=None)
BACKEND = r'''
import json, os, select, sys, time
mode, log_path = sys.argv[1:]
sys.stdin = os.fdopen(sys.stdin.fileno(), "rb", buffering=0)
active = None
emitted = False
state = dict(width=.04, max_width=0 if mode == "unhomed" else (.075 if mode == "smallmax" else .08),
             is_grasped=False, temperature=29, control_ready=mode != "not_ready",
             control_error="Hand unavailable" if mode == "not_ready" else None)
def emit(**record):
    print(json.dumps(record), flush=True)
def motion(status, **fields):
    emit(event="motion", command_id=active, status=status, **fields)
while True:
    if not (emitted and mode == "stale") and not (active and mode == "event_only"):
        emit(**state)
        emitted = True
    if active and mode == "event_only":
        motion("running")
    if active and mode in ("success", "unhomed") and time.monotonic() - started > .04:
        if action == "HOME":
            state.update(max_width=.08, width=.08, is_grasped=False)
        else:
            state.update(width=target, is_grasped=action == "GRASP")
        emit(**state)
        motion("succeeded")
        active = None
    ready, _, _ = select.select([sys.stdin], [], [], .01)
    if not ready:
        continue
    line = sys.stdin.readline().decode()
    if not line:
        break
    with open(log_path, "a") as output:
        output.write(line)
    args = line.split()
    action, command_id = args[:2]
    if action == "STOP":
        if mode in ("late_stop_ack", "late_stop_failed", "stop_terminal_only"):
            motion("succeeded")
            active = None
            if mode != "stop_terminal_only":
                time.sleep(.08)
                emit(event="stop", command_id=command_id,
                     status="failed" if mode == "late_stop_failed" else "stopped",
                     error="SDK stop returned false" if mode == "late_stop_failed" else None)
        elif mode == "stop_failed":
            emit(event="stop", command_id=command_id, status="failed", error="SDK stop returned false")
        elif mode == "stop_idle":
            emit(event="stop", command_id=command_id, status="idle")
        elif mode != "ignore_stop":
            emit(event="stop", command_id=command_id, status="stopped")
            if active and mode != "stop_only":
                motion("cancelled")
                active = None
    else:
        active, started = command_id, time.monotonic()
        target = None if action == "HOME" else float(args[3])
        if mode == "disconnect":
            sys.exit(3)
        motion("accepted")
        motion("running")
        if mode == "failed":
            motion("failed", error="SDK action returned false")
            active = None
'''


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Gripper session condition did not become true")


class NativeFactory:
    def __init__(self, path, mode):
        self.path, self.mode = path, mode
        self.calls = []
        self.lock = threading.Lock()

    def __call__(self, args, **options):
        with self.lock:
            self.calls.append(args)
        return subprocess.Popen(
            [sys.executable, "-u", "-c", BACKEND, self.mode, self.path], **options)

    def commands(self):
        return [line.split() for line in Path(self.path).read_text().splitlines()]


class GripperSessionTests(unittest.TestCase):
    def session(self, mode="hold", **options):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = str(Path(directory.name) / "commands.log")
        Path(path).touch()
        factory = NativeFactory(path, mode)
        defaults = dict(connect_timeout=.3, stale_after=.15, retry_delay=.1,
                        retry_max=.2, stop_timeout=.1, motion_stop_timeout=.4)
        defaults.update(options)
        manager = hardware.SessionManager(
            session_factory=gripper.GripperSession, process_factory=factory, **defaults)
        self.addCleanup(manager.close)
        owner = ("gripper", "first")
        session = manager.acquire("172.16.0.3", owner)
        eventually(lambda: session.snapshot()["connected"])
        return manager, session, factory, owner

    def test_connect_read_and_close_never_send_homing_or_stop(self):
        manager, session, factory, _ = self.session()
        state = session.snapshot()
        for name, value in STATE.items():
            self.assertEqual(state[name], value)
        self.assertNotIn("q", state)
        self.assertTrue(factory.calls[0][0].endswith("franka-gripper"))
        self.assertEqual(factory.calls[0][1], "172.16.0.3")
        manager.close()
        self.assertEqual(factory.commands(), [])

    def test_move_uses_meters_and_returns_actual_state(self):
        _, session, factory, owner = self.session("success")
        command_id = session.command(owner, "set_position", width_m=.025, speed_m_s=.03,
                                     timeout_s=10)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual(result["target"], {"action": "set_position", "width_m": .025, "speed_m_s": .03, "speed_source": "argument"})
        self.assertEqual(factory.commands()[0][:2], ["MOVE", command_id])
        self.assertEqual(list(map(float, factory.commands()[0][2:])), [.03, .025, 10])
        self.assertEqual(session.snapshot()["width"], .025)

    def test_grasp_force_epsilon_and_completed_hold_survive_release(self):
        manager, session, factory, owner = self.session("success")
        command_id = session.command(owner, "grasp", width_m=.01, force_n=45,
                                     epsilon_inner_m=.002, epsilon_outer_m=.003)
        self.assertEqual(session.wait(command_id, 2)["status"], "succeeded")
        self.assertEqual(factory.commands()[0][:2], ["GRASP", command_id])
        self.assertEqual(list(map(float, factory.commands()[0][2:])),
                         [.02, .01, 45, .002, .003, 30])
        self.assertTrue(session.snapshot()["is_grasped"])
        manager.release("172.16.0.3", owner)
        self.assertEqual(len(factory.commands()), 1)

    def test_unhomed_rejects_width_commands_but_allows_explicit_homing(self):
        _, session, factory, owner = self.session("unhomed")
        for action in ("set_position", "grasp"):
            with self.assertRaisesRegex(RuntimeError, "homed"):
                session.command(owner, action, width_m=.02)
        self.assertEqual(factory.commands(), [])
        command_id = session.command(owner, "homing")
        self.assertEqual(session.wait(command_id, 2)["status"], "succeeded")
        self.assertEqual(factory.commands()[0], ["HOME", command_id, "30"])
        self.assertEqual(session.snapshot()["max_width"], .08)
        command_id = session.command(owner, "set_position", width_m=.02)
        self.assertEqual(session.wait(command_id, 2)["status"], "succeeded")

    def test_limits_reject_invalid_commands_without_sending(self):
        _, session, factory, owner = self.session()
        cases = [dict(width_m=value) for value in (None, True, -.001, .081, float("nan"),
                                                 float("inf"), "0.02")]
        cases += [dict(speed_m_s=value) for value in (0, -.01, .101, True, float("nan"))]
        cases += [dict(force_n=value) for value in (29, 71, False, float("inf"))]
        cases += [dict(epsilon_inner_m=value) for value in (-.001, .081, True)]
        cases += [dict(epsilon_outer_m=value) for value in (-.001, .081, float("nan"))]
        cases += [dict(timeout_s=value) for value in (0, 121, True)]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                session.command(owner, "grasp", **(dict(width_m=.02) | values))
        with self.assertRaisesRegex(ValueError, "Unknown gripper action"):
            session.command(owner, "unknown")
        self.assertEqual(factory.commands(), [])

    def test_width_checked_against_latest_calibrated_maximum(self):
        _, session, factory, owner = self.session("smallmax")
        with self.assertRaisesRegex(ValueError, "measured max_width"):
            session.command(owner, "set_position", width_m=.076)
        self.assertEqual(factory.commands(), [])
        command_id = session.command(owner, "set_position", width_m=.075)
        session.stop_gripper(owner)
        self.assertEqual(session.wait(command_id, 2)["status"], "cancelled")

    def test_same_ip_shared_different_ip_independent_and_motion_exclusive(self):
        manager, session, factory, owner = self.session()
        other = ("gripper", "second")
        self.assertIs(session, manager.acquire("::ffff:172.16.0.3", other))
        second = manager.acquire("172.16.0.2", ("gripper", "third"))
        self.assertIsNot(session, second)
        eventually(lambda: second.snapshot()["connected"])
        session.command(owner, "set_position", width_m=.02)
        with self.assertRaisesRegex(RuntimeError, "active motion"):
            session.command(other, "grasp", width_m=.03)
        with self.assertRaisesRegex(RuntimeError, "own this session"):
            session.command(("gripper", "unknown"), "homing")
        self.assertEqual(len(factory.calls), 2)

    def test_explicit_idle_stop_reaches_native_and_waits_for_ack(self):
        _, session, factory, owner = self.session()
        result = session.stop_gripper(owner)
        result = session.wait(result["command_id"], 2)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual(factory.commands(), [["STOP", result["command_id"]]])

    def test_registered_card_can_stop_another_cards_active_grasp(self):
        manager, session, factory, owner = self.session()
        other = ("gripper", "second")
        manager.acquire("172.16.0.3", other)
        command_id = session.command(owner, "grasp", width_m=.02)
        self.assertEqual(session.stop_gripper(other)["command_id"], command_id)
        self.assertEqual(session.wait(command_id, 2)["status"], "cancelled")
        self.assertEqual(factory.commands()[-1], ["STOP", command_id])

    def test_stop_ack_alone_does_not_finish_an_active_motion(self):
        _, session, _, owner = self.session("stop_only")
        command_id = session.command(owner, "grasp", width_m=.02)
        session.stop_gripper(owner)
        eventually(lambda: session.motion(command_id).get("stop_status") == "stopped")
        self.assertEqual(session.motion(command_id)["status"], "stopping")
        self.assertEqual(session.wait(command_id, 2)["status"], "failed")

    def test_idle_stop_timeout_and_false_result_are_failures(self):
        for mode in ("ignore_stop", "stop_failed", "stop_idle"):
            with self.subTest(mode=mode):
                _, session, _, owner = self.session(mode)
                command_id = session.stop_gripper(owner)["command_id"]
                self.assertEqual(session.wait(command_id, 2)["status"], "failed")

    def test_active_stop_waits_for_ack_even_when_motion_succeeded_first(self):
        for mode, expected in (("late_stop_ack", "succeeded"),
                               ("late_stop_failed", "failed"),
                               ("stop_terminal_only", "failed")):
            with self.subTest(mode=mode):
                _, session, _, owner = self.session(mode)
                command_id = session.command(owner, "grasp", width_m=.02)
                session.stop_gripper(owner)
                eventually(lambda: session._motions[command_id].get("_terminal_after_stop"))
                self.assertEqual(session.motion(command_id)["status"], "stopping")
                with self.assertRaises(TimeoutError):
                    session.wait(command_id, .01)
                self.assertEqual(session.wait(command_id, 2)["status"], expected)

    def test_not_ready_blocks_motion_but_explicit_stop_is_still_sent(self):
        _, session, factory, owner = self.session("not_ready")
        with self.assertRaisesRegex(RuntimeError, "Hand unavailable"):
            session.command(owner, "homing")
        command_id = session.stop_gripper(owner)["command_id"]
        self.assertEqual(session.wait(command_id, 2)["status"], "succeeded")
        self.assertEqual(factory.commands()[0][0], "STOP")

    def test_owner_release_stops_its_active_action_with_other_reader_left(self):
        manager, session, factory, owner = self.session()
        manager.acquire("172.16.0.3", ("gripper", "reader"))
        command_id = session.command(owner, "homing")
        manager.release("172.16.0.3", owner)
        self.assertEqual(session.motion(command_id)["status"], "cancelled")
        self.assertTrue(session.snapshot()["connected"])
        self.assertEqual(factory.commands()[-1], ["STOP", command_id])

    def test_sdk_false_result_is_failed(self):
        _, session, _, owner = self.session("failed")
        command_id = session.command(owner, "grasp", width_m=.02)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertIn("returned false", result["error"])

    def test_disconnect_fails_without_replaying_command(self):
        _, session, factory, owner = self.session("disconnect")
        command_id = session.command(owner, "homing")
        self.assertEqual(session.wait(command_id, 2)["status"], "failed")
        eventually(lambda: len(factory.calls) >= 2 and session.snapshot()["connected"])
        self.assertEqual(factory.commands(), [["HOME", command_id, "30"]])

    def test_events_do_not_keep_stale_state_fresh(self):
        _, session, _, owner = self.session("event_only", retry_delay=1)
        command_id = session.command(owner, "set_position", width_m=.02)
        result = session.wait(command_id, 2)
        self.assertEqual(result["status"], "failed")
        self.assertIn("stale", result["error"])
        self.assertNotIn("width", session.snapshot())

    def test_state_validation_and_unhomed_feedback(self):
        _, session, _, _ = self.session()
        bad_fields = dict(width=float("nan"), max_width=-.01, temperature=True,
                          is_grasped=1, control_ready="true", control_error=7)
        for field, value in bad_fields.items():
            with self.subTest(field=field), self.assertRaises(ValueError):
                session._accept(json.dumps(STATE | {field: value}))
        self.assertTrue(session._accept(json.dumps(STATE | {"width": .04, "max_width": 0})))
        self.assertTrue(session._accept(json.dumps(STATE | {"max_width": .080107})))


if __name__ == "__main__":
    unittest.main()
