"""G1_23 dual-arm pose projection and final output adapter.

The module is robot-SDK free.  Production DDS objects and the Pinocchio IK
solver are injected by :mod:`teleop.factory`; tests use deterministic fakes.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from typing import Protocol

import numpy as np

from .descriptor import PROFILE_ID
from .dispatch import AdapterAck, MotionIntent, StopRequest

ARM_JOINT_COUNT = 10
REQUIRED_MODE_MACHINE = 4
_OPENXR_TO_ROBOT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=float,
)
_TO_UNITREE_LEFT_ARM = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0],
     [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    dtype=float,
)
_TO_UNITREE_RIGHT_ARM = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0],
     [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    dtype=float,
)


class IkSolver(Protocol):
    def solve(
        self,
        left_target: np.ndarray,
        right_target: np.ndarray,
        current_joint_positions: np.ndarray,
        current_joint_velocities: np.ndarray,
    ) -> tuple[Sequence[float], Sequence[float]]: ...

    def reset(self, current_joint_positions: Sequence[float]) -> None: ...


class LowStateReader(Protocol):
    def read_arm_state(self) -> Mapping[str, object]: ...


class ArmSdkPort(Protocol):
    publisher_count: int

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck: ...

    def apply_target(
        self,
        joint_positions: Sequence[float],
        feedforward_torques: Sequence[float],
        *,
        expires_monotonic: float,
        required_mode_machine: int,
        allow_arm: bool = False,
    ) -> AdapterAck: ...

    def safe_stop(self, *, reason: str, deadline_monotonic: float) -> AdapterAck: ...

    def snapshot(self) -> Mapping[str, object]: ...

    def close(self) -> AdapterAck | None: ...


class _LatencyWindow:
    def __init__(self, maximum: int = 256):
        self._samples: deque[float] = deque(maxlen=maximum)

    def observe_seconds(self, seconds: float) -> None:
        value = max(0.0, min(60_000.0, float(seconds) * 1000.0))
        self._samples.append(value)

    def snapshot(self) -> dict:
        if not self._samples:
            return {"last": None, "p50": None, "p95": None, "p99": None, "count": 0}
        ordered = sorted(self._samples)

        def percentile(fraction: float) -> float:
            index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
            return round(ordered[index], 3)

        return {
            "last": round(self._samples[-1], 3),
            "p50": percentile(0.50),
            "p95": percentile(0.95),
            "p99": percentile(0.99),
            "count": len(self._samples),
        }


class G1ControllerPoseMapper:
    """Map OpenXR head/controller poses into a pelvis-fixed G1 IK frame."""

    def __init__(
        self,
    ):
        # V1 is intentionally fixed to the Apache-2.0 upstream G1 controller
        # calibration.  Any future calibration change requires a new profile.
        self._waist_offset = np.array([0.15, 0.0, 0.45])

    def map_frame(self, frame: Mapping[str, object]) -> tuple[np.ndarray, np.ndarray]:
        head = self._pose_matrix(frame["head"])
        left = self._pose_matrix(frame["left_controller"])
        right = self._pose_matrix(frame["right_controller"])

        head_robot = self._change_basis(head)
        left_robot = self._change_basis(left) @ _TO_UNITREE_LEFT_ARM
        right_robot = self._change_basis(right) @ _TO_UNITREE_RIGHT_ARM
        yaw_inverse = self._head_yaw(head_robot).T
        return (
            self._head_relative(left_robot, head_robot, yaw_inverse),
            self._head_relative(right_robot, head_robot, yaw_inverse),
        )

    def _head_relative(
        self,
        pose: np.ndarray,
        head: np.ndarray,
        yaw_inverse: np.ndarray,
    ) -> np.ndarray:
        # V1 is locked to xr_teleoperate's tested arm_reference_mode=head_yaw:
        # express wrist rotation/translation in headset yaw, ignore pitch and
        # roll, then translate the origin from head to the fixed IK waist.
        result = np.eye(4)
        result[:3, :3] = yaw_inverse @ pose[:3, :3]
        relative = yaw_inverse @ (pose[:3, 3] - head[:3, 3])
        result[:3, 3] = self._waist_offset + relative
        if not np.all(np.isfinite(result)) or np.linalg.norm(relative) > 2.0:
            raise ValueError("controller pose is outside the bounded G1 workspace")
        return result

    @staticmethod
    def _head_yaw(head: np.ndarray) -> np.ndarray:
        x_axis = head[:3, 0].copy()
        x_axis[2] = 0.0
        norm = float(np.linalg.norm(x_axis))
        if not math.isfinite(norm) or norm <= 1e-6:
            return np.eye(3)
        x_axis /= norm
        z_axis = np.array([0.0, 0.0, 1.0])
        y_axis = np.cross(z_axis, x_axis)
        y_norm = float(np.linalg.norm(y_axis))
        if not math.isfinite(y_norm) or y_norm <= 1e-6:
            return np.eye(3)
        y_axis /= y_norm
        return np.column_stack((x_axis, y_axis, z_axis))

    @staticmethod
    def _change_basis(pose: np.ndarray) -> np.ndarray:
        result = np.eye(4)
        result[:3, :3] = _OPENXR_TO_ROBOT @ pose[:3, :3] @ _OPENXR_TO_ROBOT.T
        result[:3, 3] = _OPENXR_TO_ROBOT @ pose[:3, 3]
        return result

    @classmethod
    def _pose_matrix(cls, value: object) -> np.ndarray:
        if not isinstance(value, Mapping):
            raise ValueError("pose must be an object")
        position = np.asarray(value.get("position"), dtype=float)
        quaternion = np.asarray(value.get("orientation"), dtype=float)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError("pose position/quaternion dimensions are invalid")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
            raise ValueError("pose contains non-finite values")
        norm = float(np.linalg.norm(quaternion))
        if norm <= 0.0:
            raise ValueError("pose quaternion is invalid")
        x, y, z, w = quaternion / norm
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=float,
        )
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = position
        return result


class G1DualArmAdapter:
    """OutputAdapter that always runs mapping, IK and LowState observation.

    Shadow mode deliberately has no ArmSdkPort, so construction cannot create a
    publisher.  Live mode requires one injected port whose publisher_count is
    exactly one.
    """

    def __init__(
        self,
        *,
        mode: str,
        pose_mapper: G1ControllerPoseMapper,
        ik_solver: IkSolver,
        low_state_reader: LowStateReader,
        arm_sdk: ArmSdkPort | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        if mode not in ("shadow", "live"):
            raise ValueError("mode must be 'shadow' or 'live'")
        if mode == "shadow" and arm_sdk is not None:
            raise ValueError("Shadow mode forbids an arm SDK port")
        if mode == "live" and (
            arm_sdk is None or getattr(arm_sdk, "publisher_count", None) != 1
        ):
            raise ValueError("Live mode requires exactly one arm_sdk publisher")
        self.hardware_output = mode == "live"
        self._mode = mode
        self._mapper = pose_mapper
        self._ik = ik_solver
        self._low_state = low_state_reader
        self._arm_sdk = arm_sdk
        self._clock = clock
        self._lock = threading.Lock()
        self._latency = {
            "ik": _LatencyWindow(),
            "adapter_apply": _LatencyWindow(),
            "robot_follow": _LatencyWindow(),
        }
        self._closed = False
        self._last_command_at: float | None = None
        self._last_lowstate_at: float | None = None
        self._output = self._empty_output("safe")

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck:
        try:
            state = self._read_state()
            ready = getattr(self._ik, "ready", None)
            if ready is not None and (not callable(ready) or ready() is not True):
                return self._fault("ik_not_ready")
            if not callable(getattr(self._ik, "solve", None)):
                return self._fault("ik_not_ready")
            if state["mode_machine"] != REQUIRED_MODE_MACHINE:
                return self._fault("mode_machine_not_ai")
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
            return self._fault("startup_dependency_unavailable")
        with self._lock:
            measured = state["joint_positions"]
            self._last_lowstate_at = state["sample_monotonic"]
            self._output = self._make_output(
                "stopped",
                measured,
                measured,
                fault_reason=None,
            )
        reset = getattr(self._ik, "reset", None)
        if callable(reset):
            try:
                reset(measured)
            except (TypeError, ValueError, RuntimeError, ArithmeticError):
                return self._fault("ik_reset_failed")
        if self._arm_sdk is not None:
            ack = self._arm_sdk.startup_safe(deadline_monotonic)
            self._refresh_sdk_output()
            return ack
        if self._clock() >= deadline_monotonic:
            return self._fault("startup_safe_deadline_missed")
        return AdapterAck(True)

    def apply(self, intent: MotionIntent) -> AdapterAck:
        if self._clock() >= intent.expires_monotonic:
            return AdapterAck(False, "intent_expired_at_adapter")
        if intent.frame.get("deadman") is not True:
            return AdapterAck(False, "unsafe_deadman")
        tracking = intent.frame.get("tracking")
        if not isinstance(tracking, Mapping) or not all(value is True for value in tracking.values()):
            return AdapterAck(False, "unsafe_tracking")
        try:
            state = self._read_state()
            left_target, right_target = self._mapper.map_frame(intent.frame)
            ik_started = self._clock()
            raw_target, raw_torques = self._ik.solve(
                left_target,
                right_target,
                state["joint_positions"],
                state["joint_velocities"],
            )
            target = np.asarray(raw_target, dtype=float)
            torques = np.asarray(raw_torques, dtype=float)
            self._latency["ik"].observe_seconds(self._clock() - ik_started)
            if (
                target.shape != (ARM_JOINT_COUNT,)
                or torques.shape != (ARM_JOINT_COUNT,)
                or not np.all(np.isfinite(target))
                or not np.all(np.isfinite(torques))
            ):
                return self._fault("invalid_ik_result")
            refreshed_state = self._read_state()
            if self._clock() >= intent.expires_monotonic:
                return self._fault("intent_expired_after_ik")
            if self._mode == "live" and refreshed_state["mode_machine"] != REQUIRED_MODE_MACHINE:
                return self._fault("mode_machine_not_ai")
            apply_started = self._clock()
            if self._arm_sdk is not None:
                ack = self._arm_sdk.apply_target(
                    target.tolist(),
                    torques.tolist(),
                    expires_monotonic=intent.expires_monotonic,
                    required_mode_machine=REQUIRED_MODE_MACHINE,
                    allow_arm=intent.allow_reclutch,
                )
                if not ack.ok:
                    if ack.code in {"arm_sdk_reclutch_required", "arm_sdk_releasing"}:
                        self._refresh_sdk_output()
                        return ack
                    return self._fault(ack.code)
            self._latency["adapter_apply"].observe_seconds(self._clock() - apply_started)
            now = self._clock()
            with self._lock:
                if (
                    self.hardware_output
                    and self._last_command_at is not None
                    and state["sample_monotonic"] >= self._last_command_at
                ):
                    self._latency["robot_follow"].observe_seconds(
                        state["sample_monotonic"] - self._last_command_at
                    )
                self._last_command_at = now
                self._last_lowstate_at = refreshed_state["sample_monotonic"]
                self._output = self._make_output(
                    "published" if self.hardware_output else "would_apply",
                    target,
                    refreshed_state["joint_positions"],
                    fault_reason=None,
                )
            self._refresh_sdk_output()
            return AdapterAck(True)
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
            return self._fault("projection_or_ik_failed")

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        if self._arm_sdk is not None:
            ack = self._arm_sdk.safe_stop(
                reason=request.reason,
                deadline_monotonic=request.deadline_monotonic,
            )
            if not ack.ok:
                return self._fault(ack.code)
        elif self._clock() >= request.deadline_monotonic:
            return self._fault("adapter_stop_deadline_missed")
        try:
            state = self._read_state()
            measured = state["joint_positions"]
            reset = getattr(self._ik, "reset", None)
            if callable(reset):
                reset(measured)
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
            return self._fault("stop_dependency_unavailable")
        with self._lock:
            self._output = self._make_output(
                "stopped",
                measured,
                measured,
                fault_reason=None,
            )
        self._refresh_sdk_output()
        return AdapterAck(True)

    def snapshot(self) -> dict:
        self._refresh_sdk_output()
        with self._lock:
            output = copy.deepcopy(self._output)
            if self._last_command_at is not None:
                output["command_age_ms"] = round(
                    min(60_000.0, max(0.0, self._clock() - self._last_command_at) * 1000.0),
                    3,
                )
            value = {
                "kind": "hardware" if self.hardware_output else "recording",
                "hardware_output": self.hardware_output,
                "diagnostics": {
                    "latency_ms": {
                        name: window.snapshot() for name, window in self._latency.items()
                    }
                },
                "output": output,
            }
            if not self.hardware_output:
                operation = output.get("state")
                if operation == "would_apply":
                    current_kind = "would_apply"
                elif operation in {"stopped", "releasing"}:
                    current_kind = "would_stop"
                else:
                    current_kind = "safe"
                value.update({
                    "closed": self._closed,
                    "current": {"kind": current_kind},
                    "records": [],
                })
            return value

    def external_fault_code(self) -> str | None:
        probe = getattr(self._arm_sdk, "external_fault_code", None)
        return probe() if callable(probe) else None

    def external_release_signal(self) -> Mapping[str, object]:
        probe = getattr(self._arm_sdk, "external_release_signal", None)
        if callable(probe):
            return probe()
        return {"generation": 0, "reason": None, "acknowledged": True}

    def close(self) -> AdapterAck | None:
        with self._lock:
            if self._closed:
                return AdapterAck(True)
            self._closed = True
        ack = self._arm_sdk.close() if self._arm_sdk is not None else AdapterAck(True)
        close_reader = getattr(self._low_state, "close", None)
        if callable(close_reader):
            close_reader()
        return ack

    def _read_state(self) -> dict:
        raw = self._low_state.read_arm_state()
        q = np.asarray(raw.get("joint_positions"), dtype=float)
        dq = np.asarray(raw.get("joint_velocities"), dtype=float)
        mode_machine = raw.get("mode_machine")
        sampled = float(raw.get("sample_monotonic"))
        if q.shape != (ARM_JOINT_COUNT,) or dq.shape != (ARM_JOINT_COUNT,):
            raise ValueError("LowState arm vector shape is invalid")
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)) or not math.isfinite(sampled):
            raise ValueError("LowState contains non-finite values")
        if isinstance(mode_machine, bool) or not isinstance(mode_machine, int):
            raise ValueError("LowState mode_machine is invalid")
        age = self._clock() - sampled
        if age < 0.0 or age > 0.1:
            raise RuntimeError("LowState is stale")
        return {
            "joint_positions": q,
            "joint_velocities": dq,
            "mode_machine": mode_machine,
            "sample_monotonic": sampled,
        }

    def _fault(self, reason: str) -> AdapterAck:
        with self._lock:
            self._output["state"] = "fault"
            self._output["fault_reason"] = str(reason)[:128]
        return AdapterAck(False, str(reason)[:64])

    def _make_output(
        self,
        state: str,
        target: Sequence[float] | None,
        measured: Sequence[float] | None,
        *,
        fault_reason: str | None,
    ) -> dict:
        target_values = [] if target is None else [round(float(value), 6) for value in target]
        measured_values = (
            [] if measured is None else [round(float(value), 6) for value in measured]
        )
        error = None
        if target_values and measured_values:
            error = round(max(abs(a - b) for a, b in zip(target_values, measured_values)), 6)
        return {
            "profile_id": PROFILE_ID,
            "hardware_output": self.hardware_output,
            "state": state,
            "target_joint_positions_rad": target_values,
            "measured_joint_positions_rad": measured_values,
            "max_abs_error_rad": error,
            "arm_sdk_weight": 0.0 if self.hardware_output else None,
            "command_age_ms": None,
            "fault_reason": fault_reason,
        }

    def _empty_output(self, state: str) -> dict:
        return self._make_output(state, None, None, fault_reason=None)

    def _refresh_sdk_output(self) -> None:
        if self._arm_sdk is None:
            return
        sdk = self._arm_sdk.snapshot()
        weight = sdk.get("arm_sdk_weight")
        if isinstance(weight, (int, float)) and not isinstance(weight, bool) and math.isfinite(weight):
            with self._lock:
                self._output["arm_sdk_weight"] = round(min(1.0, max(0.0, float(weight))), 6)


__all__ = [
    "ARM_JOINT_COUNT",
    "G1ControllerPoseMapper",
    "G1DualArmAdapter",
    "REQUIRED_MODE_MACHINE",
]
