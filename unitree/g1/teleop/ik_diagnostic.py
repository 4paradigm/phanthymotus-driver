"""Zero-output MCP diagnostic facade for the shared G1_23 IK solver."""

from __future__ import annotations

import copy
import json
import math
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

import numpy as np

from .descriptor import PROFILE_ID
from .protocol import ProtocolError, validate_tracked_pose_v1

_ARM_JOINT_COUNT = 10
_REQUIRED_MODE_MACHINE = 4
_INACTIVE_STATES = frozenset({"idle", "released"})
_FRAME_JSON_MAX_BYTES = 16 * 1024
_FRAME_KEYS = frozenset({"head", "left_controller", "right_controller"})


class G1TeleopIkDiagnostic:
    """Expose mapping and IK without owning or calling an ``arm_sdk`` port.

    The real-time adapter and this diagnostic share one solver and one access
    lock. Diagnostic solves restore the measured seed before releasing that
    lock, so a later real-time frame never inherits diagnostic filter history.
    """

    def __init__(
        self,
        *,
        pose_mapper: object,
        ik_solver: object,
        low_state_reader: object,
        runtime_status: Callable[[], Mapping[str, object]],
        run_guard: Callable[[Callable[[], Any]], Any],
        ik_access_lock: threading.RLock,
        clock: Callable[[], float] = time.monotonic,
    ):
        if not callable(getattr(pose_mapper, "map_frame", None)):
            raise ValueError("IK diagnostic requires a pose mapper")
        if not callable(getattr(ik_solver, "solve", None)):
            raise ValueError("IK diagnostic requires a solver")
        if not callable(getattr(ik_solver, "reset", None)):
            raise ValueError("IK diagnostic requires a resettable solver")
        if not callable(getattr(ik_solver, "current_targets", None)):
            raise ValueError("IK diagnostic requires measured end-effector targets")
        if not callable(getattr(low_state_reader, "read_arm_state", None)):
            raise ValueError("IK diagnostic requires a LowState reader")
        if not callable(runtime_status):
            raise ValueError("IK diagnostic requires a runtime status probe")
        if not callable(run_guard):
            raise ValueError("IK diagnostic requires a runtime transition guard")
        self._pose_mapper = pose_mapper
        self._ik = ik_solver
        self._low_state = low_state_reader
        self._runtime_status = runtime_status
        self._run_guard = run_guard
        self._ik_access_lock = ik_access_lock
        self._clock = clock
        self._state_lock = threading.Lock()
        self._last_result: dict | None = None
        self._counters = {
            "solves": 0,
            "self_tests": 0,
            "resets": 0,
            "failures": 0,
        }

    def dispatch(self, action: str, arguments: Any) -> dict:
        if not isinstance(arguments, dict):
            raise ProtocolError("invalid_arguments", "teleop_ik arguments must be an object")
        args = dict(arguments)
        try:
            if action == "solve":
                return self._solve(args)
            if action == "self_test":
                self._require_exact(args, set(), action)
                return self._self_test()
            if action == "reset":
                self._require_exact(args, set(), action)
                return self._reset()
            if action == "status":
                self._require_exact(args, set(), action)
                return self.status()
            raise ProtocolError("unknown_action", f"unknown teleop_ik action: {action}")
        except ProtocolError:
            self._record_failure()
            raise
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
            self._record_failure()
            raise ProtocolError(
                "ik_diagnostic_failed",
                "G1 IK diagnostic could not complete safely",
            ) from exc

    def status(self) -> dict:
        runtime = dict(self._runtime_status())
        solver = self._solver_status()
        with self._state_lock:
            counters = dict(self._counters)
            last_result = copy.deepcopy(self._last_result)
        return {
            "state": "ready" if solver["ready"] else "unavailable",
            "profile_id": PROFILE_ID,
            "diagnostic_hardware_output": False,
            "diagnostic_publisher_present": False,
            "diagnostic_output_active": False,
            "actuation_enabled": bool(runtime.get("actuation_enabled")),
            "publisher_present": bool(runtime.get("publisher_present")),
            "output_active": bool(runtime.get("output_active")),
            "shares_realtime_ik": True,
            "session_state": runtime.get("state"),
            "session_active": bool(
                runtime.get("authority_valid")
                or runtime.get("state") not in _INACTIVE_STATES
            ),
            "solver": solver,
            "last_result": last_result,
            "counters": counters,
        }

    def _solve(self, args: dict) -> dict:
        self._require_exact(args, {"frame_json"}, "solve")
        frame = self._decode_frame_json(args["frame_json"])

        def target_provider(_state: dict) -> tuple[object, object]:
            return self._pose_mapper.map_frame(frame)

        return self._solve_guarded(target_provider, result_state="solved")

    def _self_test(self) -> dict:
        def target_provider(state: dict) -> tuple[object, object]:
            return self._ik.current_targets(state["joint_positions"])

        return self._solve_guarded(target_provider, result_state="self_tested")

    def _solve_guarded(
        self,
        target_provider: Callable[[dict], tuple[object, object]],
        *,
        result_state: str,
    ) -> dict:
        def operation() -> tuple[dict, np.ndarray, np.ndarray, float]:
            with self._ik_access_lock:
                state = self._read_state()
                started = self._clock()
                solve_error: BaseException | None = None
                target = torques = None
                try:
                    left, right = target_provider(state)
                    target, torques = self._ik.solve(
                        left,
                        right,
                        state["joint_positions"],
                        state["joint_velocities"],
                    )
                    target = self._vector(target, "IK joint target")
                    torques = self._vector(torques, "IK feedforward torque")
                except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
                    solve_error = exc
                try:
                    self._ik.reset(state["joint_positions"])
                except (TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
                    raise ProtocolError(
                        "ik_diagnostic_reset_failed",
                        "G1 IK diagnostic could not restore the measured seed",
                    ) from exc
                if solve_error is not None:
                    raise ProtocolError(
                        "ik_diagnostic_solve_failed",
                        "G1 IK diagnostic solve failed",
                    ) from solve_error
                elapsed_ms = round(
                    min(600_000.0, max(0.0, (self._clock() - started) * 1000.0)),
                    3,
                )
                return state, target, torques, elapsed_ms

        state, target, torques, elapsed_ms = self._run_guard(operation)

        result = {
            "state": result_state,
            "profile_id": PROFILE_ID,
            "diagnostic_hardware_output": False,
            "diagnostic_publisher_present": False,
            "diagnostic_output_active": False,
            "mode_machine": state["mode_machine"],
            "solve_ms": elapsed_ms,
            "joint_positions_rad": [round(float(value), 6) for value in target],
            "feedforward_torques_nm": [round(float(value), 6) for value in torques],
            "measured_joint_positions_rad": [
                round(float(value), 6) for value in state["joint_positions"]
            ],
        }
        with self._state_lock:
            counter = "self_tests" if result_state == "self_tested" else "solves"
            self._counters[counter] += 1
            self._last_result = copy.deepcopy(result)
        return result

    def _reset(self) -> dict:
        def operation() -> None:
            with self._ik_access_lock:
                state = self._read_state()
                self._ik.reset(state["joint_positions"])

        self._run_guard(operation)
        with self._state_lock:
            self._counters["resets"] += 1
            self._last_result = None
        result = self.status()
        result["state"] = "reset"
        return result

    @classmethod
    def _decode_frame_json(cls, value: object) -> dict:
        if not isinstance(value, str):
            raise ProtocolError(
                "invalid_arguments",
                "teleop_ik solve frame_json must be a string",
            )
        try:
            encoded = value.encode("utf-8")
        except UnicodeEncodeError as exc:
            raise ProtocolError(
                "invalid_frame_json",
                "teleop_ik frame_json must be valid UTF-8 text",
            ) from exc
        if len(encoded) > _FRAME_JSON_MAX_BYTES:
            raise ProtocolError(
                "invalid_frame_json",
                "teleop_ik frame_json exceeds 16 KiB",
            )

        def object_pairs(pairs):
            result = {}
            for key, item in pairs:
                if key in result:
                    raise ValueError(f"duplicate JSON field: {key}")
                result[key] = item
            return result

        def reject_constant(constant):
            raise ValueError(f"non-finite JSON number: {constant}")

        try:
            frame = json.loads(
                value,
                object_pairs_hook=object_pairs,
                parse_constant=reject_constant,
            )
        except (json.JSONDecodeError, TypeError, ValueError, RecursionError) as exc:
            raise ProtocolError(
                "invalid_frame_json",
                "teleop_ik frame_json must be strict JSON without duplicate fields or NaN",
            ) from exc
        if not isinstance(frame, dict) or set(frame) != _FRAME_KEYS:
            raise ProtocolError(
                "invalid_frame_json",
                "frame_json requires exactly head, left_controller and right_controller",
            )
        normalized = {}
        for name in sorted(_FRAME_KEYS):
            try:
                normalized[name] = validate_tracked_pose_v1(frame[name], name)
            except ProtocolError as exc:
                # Preserve the stable realtime validation code (notably
                # invalid_quaternion) so diagnostics and RTC never disagree.
                if exc.code == "invalid_quaternion":
                    raise
                raise ProtocolError(
                    "invalid_frame_json",
                    f"teleop_ik {name} does not match the realtime pose contract",
                ) from exc
        return normalized

    def _read_state(self) -> dict:
        raw = self._low_state.read_arm_state()
        if not isinstance(raw, Mapping):
            raise ValueError("LowState must be an object")
        q = self._vector(raw.get("joint_positions"), "LowState joint positions")
        dq = self._vector(raw.get("joint_velocities"), "LowState joint velocities")
        mode_machine = raw.get("mode_machine")
        sampled = raw.get("sample_monotonic")
        if isinstance(mode_machine, bool) or not isinstance(mode_machine, int):
            raise ValueError("LowState mode_machine is invalid")
        if mode_machine != _REQUIRED_MODE_MACHINE:
            raise RuntimeError("G1 is not in mode_machine=4")
        if (
            isinstance(sampled, bool)
            or not isinstance(sampled, (int, float))
            or not math.isfinite(float(sampled))
        ):
            raise ValueError("LowState sample time is invalid")
        age = self._clock() - float(sampled)
        if not math.isfinite(age) or age < 0.0 or age > 0.1:
            raise RuntimeError("LowState is stale")
        return {
            "joint_positions": q,
            "joint_velocities": dq,
            "mode_machine": mode_machine,
        }

    def _solver_status(self) -> dict:
        ready_probe = getattr(self._ik, "ready", None)
        try:
            ready = bool(ready_probe()) if callable(ready_probe) else True
        except (TypeError, ValueError, RuntimeError, ArithmeticError):
            ready = False
        snapshot_probe = getattr(self._ik, "snapshot", None)
        raw = {}
        if callable(snapshot_probe):
            try:
                candidate = snapshot_probe()
                if isinstance(candidate, Mapping):
                    raw = dict(candidate)
            except (TypeError, ValueError, RuntimeError, ArithmeticError):
                ready = False
        warmup_ms = raw.get("warmup_ms")
        if (
            isinstance(warmup_ms, bool)
            or not isinstance(warmup_ms, (int, float))
            or not math.isfinite(float(warmup_ms))
            or float(warmup_ms) < 0.0
        ):
            warmup_ms = None
        history_depth = raw.get("history_depth")
        if (
            isinstance(history_depth, bool)
            or not isinstance(history_depth, int)
            or not 0 <= history_depth <= 4
        ):
            history_depth = None
        return {
            "ready": ready,
            "model": "g1_body23.urdf",
            "implementation": "pinocchio-casadi-ipopt",
            "warmup_ms": None if warmup_ms is None else round(float(warmup_ms), 3),
            "history_depth": history_depth,
        }

    def _record_failure(self) -> None:
        with self._state_lock:
            self._counters["failures"] += 1

    @staticmethod
    def _vector(value: object, name: str) -> np.ndarray:
        result = np.asarray(value, dtype=float)
        if result.shape != (_ARM_JOINT_COUNT,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite ten-joint vector")
        return result

    @staticmethod
    def _require_exact(args: dict, expected: set[str], action: str) -> None:
        if set(args) != expected:
            raise ProtocolError(
                "invalid_arguments",
                f"teleop_ik {action} requires exactly {sorted(expected)}",
            )


__all__ = ["G1TeleopIkDiagnostic"]
