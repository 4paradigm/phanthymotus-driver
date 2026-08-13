from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from teleop.adapter import G1ControllerPoseMapper
from teleop.dispatch import RecordingAdapter
from teleop.ik_diagnostic import G1TeleopIkDiagnostic
from teleop.protocol import ProtocolError
from teleop.runtime import G1TeleopRuntime
from tests.helpers import session


class DiagnosticLowState:
    def __init__(self):
        self.q = np.linspace(-0.2, 0.2, 10)
        self.dq = np.zeros(10)
        self.mode_machine = 4

    def read_arm_state(self):
        return {
            "joint_positions": self.q.copy(),
            "joint_velocities": self.dq.copy(),
            "mode_machine": self.mode_machine,
            "sample_monotonic": time.monotonic(),
        }


class DiagnosticIk:
    def __init__(self):
        self.solves = []
        self.resets = []

    def ready(self):
        return True

    def snapshot(self):
        return {"ready": True, "warmup_ms": 2.5, "history_depth": 0}

    def solve(self, left, right, q, dq):
        self.solves.append((left.copy(), right.copy(), q.copy(), dq.copy()))
        return q + 0.05, np.linspace(0.0, 0.9, 10)

    def reset(self, q):
        self.resets.append(np.asarray(q, dtype=float).copy())

    def current_targets(self, q):
        left = np.eye(4)
        right = np.eye(4)
        left[1, 3] = 0.25
        right[1, 3] = -0.25
        return left, right


def poses():
    identity = [0.0, 0.0, 0.0, 1.0]
    return {
        "head": {"position": [0.0, 1.6, 0.0], "orientation": identity},
        "left_controller": {
            "position": [-0.2, 1.2, -0.4],
            "orientation": identity,
        },
        "right_controller": {
            "position": [0.2, 1.2, -0.4],
            "orientation": identity,
        },
    }


def solve_arguments():
    return {"frame_json": json.dumps(poses(), separators=(",", ":"))}


class G1TeleopIkDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.low_state = DiagnosticLowState()
        self.ik = DiagnosticIk()
        self.runtime = {"state": "idle", "authority_valid": False}

        def run_guard(operation):
            if (
                self.runtime.get("authority_valid") is True
                or self.runtime.get("state") not in {"idle", "released"}
            ):
                raise ProtocolError(
                    "teleop_session_active",
                    "diagnostic is unavailable while a session is active",
                )
            return operation()

        self.diagnostic = G1TeleopIkDiagnostic(
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=self.ik,
            low_state_reader=self.low_state,
            runtime_status=lambda: dict(self.runtime),
            run_guard=run_guard,
            ik_access_lock=threading.RLock(),
        )

    def test_solve_uses_shared_ik_and_restores_seed_without_hardware_output(self):
        result = self.diagnostic.dispatch("solve", solve_arguments())

        self.assertEqual("solved", result["state"])
        self.assertFalse(result["diagnostic_hardware_output"])
        self.assertFalse(result["diagnostic_publisher_present"])
        self.assertFalse(result["diagnostic_output_active"])
        self.assertEqual(10, len(result["joint_positions_rad"]))
        self.assertEqual(10, len(result["feedforward_torques_nm"]))
        self.assertEqual(1, len(self.ik.solves))
        self.assertEqual(1, len(self.ik.resets))
        np.testing.assert_array_equal(self.low_state.q, self.ik.resets[-1])
        status = self.diagnostic.dispatch("status", {})
        self.assertEqual(1, status["counters"]["solves"])
        self.assertEqual(result, status["last_result"])

    def test_reset_and_solve_fail_closed_while_session_is_active(self):
        self.runtime = {"state": "prepared_live", "authority_valid": True}
        for action, arguments in (("reset", {}), ("solve", solve_arguments())):
            with self.subTest(action=action):
                with self.assertRaises(ProtocolError) as caught:
                    self.diagnostic.dispatch(action, arguments)
                self.assertEqual("teleop_session_active", caught.exception.code)
        self.assertEqual([], self.ik.solves)
        self.assertEqual([], self.ik.resets)
        self.assertEqual(2, self.diagnostic.status()["counters"]["failures"])

    def test_reset_uses_current_measured_posture_and_clears_last_result(self):
        self.diagnostic.dispatch("solve", solve_arguments())
        self.low_state.q = np.linspace(0.3, -0.3, 10)

        result = self.diagnostic.dispatch("reset", {})

        self.assertEqual("reset", result["state"])
        self.assertFalse(result["diagnostic_hardware_output"])
        self.assertIsNone(result["last_result"])
        np.testing.assert_array_equal(self.low_state.q, self.ik.resets[-1])

    def test_self_test_uses_measured_ee_targets_and_is_visible_but_zero_output(self):
        result = self.diagnostic.dispatch("self_test", {})

        self.assertEqual("self_tested", result["state"])
        self.assertFalse(result["diagnostic_hardware_output"])
        self.assertFalse(result["diagnostic_publisher_present"])
        self.assertFalse(result["diagnostic_output_active"])
        self.assertEqual(1, len(self.ik.solves))
        self.assertEqual(0.25, self.ik.solves[-1][0][1, 3])
        self.assertEqual(-0.25, self.ik.solves[-1][1][1, 3])
        status = self.diagnostic.status()
        self.assertEqual(1, status["counters"]["self_tests"])
        self.assertEqual(result, status["last_result"])

    def test_frame_json_is_strict_bounded_and_rejects_unknown_or_duplicate_fields(self):
        pose_json = json.dumps(poses()["head"], separators=(",", ":"))
        invalid = (
            None,
            "{" + f'"head":{pose_json},"head":{pose_json},' +
            f'"left_controller":{pose_json},"right_controller":{pose_json}' + "}",
            json.dumps({**poses(), "unknown": 1}),
            json.dumps({
                **poses(),
                "head": {**poses()["head"], "unknown": 1},
            }),
            json.dumps({
                **poses(),
                "head": {"position": [True, 1.6, 0.0], "orientation": [0, 0, 0, 1]},
            }),
            '{"head":{"position":[NaN,0,0],"orientation":[0,0,0,1]},'
            '"left_controller":{"position":[0,0,0],"orientation":[0,0,0,1]},'
            '"right_controller":{"position":[0,0,0],"orientation":[0,0,0,1]}}',
            " " * (16 * 1024 + 1),
        )
        for frame_json in invalid:
            with self.subTest(frame_json=repr(frame_json)[:80]):
                with self.assertRaises(ProtocolError) as caught:
                    self.diagnostic.dispatch("solve", {"frame_json": frame_json})
                self.assertIn(caught.exception.code, {"invalid_arguments", "invalid_frame_json"})
        self.assertEqual([], self.ik.solves)

    def test_frame_json_uses_the_realtime_quaternion_tolerance(self):
        for orientation in (
            [0.0, 0.0, 0.0, 0.5],
            [0.0, 0.0, 0.0, 0.0],
        ):
            frame = poses()
            frame["left_controller"]["orientation"] = orientation
            with self.subTest(orientation=orientation):
                with self.assertRaises(ProtocolError) as caught:
                    self.diagnostic.dispatch(
                        "solve",
                        {"frame_json": json.dumps(frame, separators=(",", ":"))},
                    )
                self.assertEqual("invalid_quaternion", caught.exception.code)
        self.assertEqual([], self.ik.solves)

    def test_runtime_guard_serializes_diagnostic_and_prepare_without_toctou(self):
        class GuardAdapter(RecordingAdapter):
            def __init__(adapter_self):
                super().__init__()
                adapter_self.prepare_safe_stop_entered = threading.Event()

            def safe_stop(adapter_self, request):
                if request.reason == "prepare_shadow":
                    adapter_self.prepare_safe_stop_entered.set()
                return super().safe_stop(request)

        class BlockingIk(DiagnosticIk):
            def __init__(ik_self):
                super().__init__()
                ik_self.solve_entered = threading.Event()
                ik_self.solve_release = threading.Event()

            def solve(ik_self, left, right, q, dq):
                ik_self.solve_entered.set()
                if not ik_self.solve_release.wait(2.0):
                    raise RuntimeError("test did not release diagnostic solve")
                return super().solve(left, right, q, dq)

        adapter = GuardAdapter()
        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=adapter,
            lease_timeout_ms=15_000,
            auto_watchdog=False,
        )
        ik = BlockingIk()
        diagnostic = G1TeleopIkDiagnostic(
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=ik,
            low_state_reader=DiagnosticLowState(),
            runtime_status=runtime.status,
            run_guard=runtime.run_ik_diagnostic,
            ik_access_lock=threading.RLock(),
        )
        diagnostic_result = []
        diagnostic_error = []
        prepare_result = []
        prepare_error = []
        prepare_started = threading.Barrier(2)

        def run_diagnostic():
            try:
                diagnostic_result.append(diagnostic.dispatch("solve", solve_arguments()))
            except BaseException as exc:  # surfaced by the assertions below
                diagnostic_error.append(exc)

        def run_prepare():
            prepare_started.wait()
            try:
                prepare_result.append(runtime.prepare("shadow", session()))
            except BaseException as exc:  # surfaced by the assertions below
                prepare_error.append(exc)

        diagnostic_thread = threading.Thread(target=run_diagnostic)
        prepare_thread = threading.Thread(target=run_prepare)
        try:
            diagnostic_thread.start()
            self.assertTrue(ik.solve_entered.wait(1.0))
            prepare_thread.start()
            prepare_started.wait()
            # The barrier proves prepare was released to call the runtime; it
            # still cannot reach final-dispatch while diagnostic owns the
            # transition guard.
            self.assertFalse(adapter.prepare_safe_stop_entered.wait(0.05))

            ik.solve_release.set()
            diagnostic_thread.join(1.0)
            self.assertFalse(diagnostic_thread.is_alive())
            self.assertTrue(adapter.prepare_safe_stop_entered.wait(1.0))
            prepare_thread.join(1.0)
            self.assertFalse(prepare_thread.is_alive())
            self.assertEqual([], diagnostic_error)
            self.assertEqual([], prepare_error)
            self.assertEqual("solved", diagnostic_result[0]["state"])
            self.assertEqual("prepared_shadow", prepare_result[0]["state"])

            with self.assertRaises(ProtocolError) as caught:
                diagnostic.dispatch("self_test", {})
            self.assertEqual("teleop_session_active", caught.exception.code)
        finally:
            ik.solve_release.set()
            diagnostic_thread.join(1.0)
            prepare_thread.join(1.0)
            runtime.close()

    def test_import_does_not_load_hardware_module(self):
        completed = subprocess.run(
            [
                sys.executable,
                "-c",
                (
                    "import json,sys; import teleop.ik_diagnostic; "
                    "print(json.dumps('teleop.hardware' in sys.modules))"
                ),
            ],
            cwd=Path(__file__).resolve().parents[1],
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertFalse(json.loads(completed.stdout))


if __name__ == "__main__":
    unittest.main()
