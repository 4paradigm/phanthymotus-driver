"""Production G1_23 LowState reader and sole ``rt/arm_sdk`` publisher.

The lifecycle is a focused Apache-2.0 port of the tested G1_23 safety changes
in Unitree ``xr_teleoperate``.  Construction never publishes.  One owner
thread performs every DDS write and a stop ACK is returned only after five
real, successful zero-weight frames.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence

import numpy as np

from .control_safety import ArmSdkGate, ArmSdkState
from .dispatch import AdapterAck

ARM_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
WRIST_INDICES = frozenset({19, 26})
WEAK_INDICES = frozenset({4, 10, 15, 16, 17, 18, 22, 23, 24, 25})
WEIGHT_INDEX = 29
MOTOR_COUNT = 35
ARM_SDK_TOPIC = "rt/arm_sdk"
LOWSTATE_TOPIC = "rt/lowstate"
REQUIRED_MODE_MACHINE = 4

_publisher_guard = threading.Lock()
_active_publishers = 0


class G1LowStateReader:
    """One callback-backed ``rt/lowstate`` reader shared by IK and publishing."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        ready_timeout_s: float = 2.0,
        subscriber=None,
    ):
        self._clock = clock
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._closed = False
        self._sample: dict | None = None
        self._invalid_samples = 0
        if subscriber is None:
            try:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            except (ImportError, OSError) as exc:
                raise RuntimeError("Unitree LowState DDS dependencies are unavailable") from exc
            subscriber = ChannelSubscriber(LOWSTATE_TOPIC, LowState_)
        self._subscriber = subscriber
        try:
            self._subscriber.Init(self._on_sample, queueLen=1)
        except TypeError:
            # Minimal injected fakes may not expose the optional queue length.
            self._subscriber.Init(self._on_sample)
        if not self._ready.wait(float(ready_timeout_s)):
            self.close()
            raise RuntimeError(
                f"no valid {LOWSTATE_TOPIC} sample within {float(ready_timeout_s):.3f}s"
            )

    def _on_sample(self, message) -> None:
        try:
            motors = message.motor_state
            if len(motors) < MOTOR_COUNT:
                raise ValueError("LowState motor vector is too short")
            q = np.asarray([motors[index].q for index in range(MOTOR_COUNT)], dtype=float)
            dq = np.asarray([motors[index].dq for index in range(MOTOR_COUNT)], dtype=float)
            mode_machine = message.mode_machine
            if (
                q.shape != (MOTOR_COUNT,)
                or dq.shape != (MOTOR_COUNT,)
                or not np.all(np.isfinite(q))
                or not np.all(np.isfinite(dq))
                or isinstance(mode_machine, bool)
                or not isinstance(mode_machine, int)
            ):
                raise ValueError("LowState sample is invalid")
            sample = {
                "joint_positions": q[list(ARM_INDICES)].copy(),
                "joint_velocities": dq[list(ARM_INDICES)].copy(),
                "all_joint_positions": q,
                "mode_machine": int(mode_machine),
                "sample_monotonic": self._clock(),
            }
        except (AttributeError, IndexError, TypeError, ValueError):
            with self._lock:
                self._invalid_samples += 1
            return
        with self._lock:
            if self._closed:
                return
            self._sample = sample
            self._ready.set()

    def read_arm_state(self) -> Mapping[str, object]:
        with self._lock:
            if self._closed:
                raise RuntimeError("LowState reader is closed")
            if self._sample is None:
                raise RuntimeError("LowState has not been received")
            return {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in self._sample.items()
            }

    def snapshot(self) -> dict:
        with self._lock:
            sampled = None if self._sample is None else self._sample["sample_monotonic"]
            return {
                "topic": LOWSTATE_TOPIC,
                "ready": self._sample is not None and not self._closed,
                "sample_age_ms": (
                    None
                    if sampled is None
                    else round(min(60_000.0, max(0.0, self._clock() - sampled) * 1000.0), 3)
                ),
                "invalid_samples": self._invalid_samples,
            }

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        close = getattr(self._subscriber, "Close", None)
        if callable(close):
            close()


class G1ArmSdkPort:
    """Exactly-one, 250 Hz G1_23 arm SDK publication boundary."""

    FINAL_ZERO_WEIGHT_FRAMES = 5
    publisher_count = 1

    def __init__(
        self,
        low_state_reader,
        *,
        control_hz: float = 250.0,
        ramp_seconds: float = 2.0,
        release_seconds: float = 0.05,
        velocity_limit_rad_s: float = 0.5,
        command_timeout_s: float = 0.2,
        clock: Callable[[], float] = time.monotonic,
        publisher=None,
        message_factory=None,
        crc=None,
    ):
        global _active_publishers
        values = {
            "control_hz": control_hz,
            "ramp_seconds": ramp_seconds,
            "release_seconds": release_seconds,
            "velocity_limit_rad_s": velocity_limit_rad_s,
            "command_timeout_s": command_timeout_s,
        }
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a finite number")
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
        if not 50.0 <= float(control_hz) <= 500.0:
            raise ValueError("control_hz must be in [50, 500]")
        if float(ramp_seconds) < 0.0 or not 0.0 <= float(release_seconds) <= 0.1:
            raise ValueError("ramp/release seconds are outside the bounded profile")
        if float(velocity_limit_rad_s) <= 0.0 or not 0.05 <= float(command_timeout_s) <= 0.5:
            raise ValueError("velocity limit/command timeout is outside the bounded profile")

        with _publisher_guard:
            if _active_publishers != 0:
                raise RuntimeError("a G1 rt/arm_sdk publisher already exists in this process")
            _active_publishers = 1
        self._guard_owned = True
        try:
            if publisher is None or message_factory is None or crc is None:
                try:
                    from unitree_sdk2py.core.channel import ChannelPublisher
                    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
                    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
                    from unitree_sdk2py.utils.crc import CRC
                except (ImportError, OSError) as exc:
                    raise RuntimeError("Unitree arm_sdk DDS dependencies are unavailable") from exc
                publisher = publisher or ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
                message_factory = message_factory or unitree_hg_msg_dds__LowCmd_
                crc = crc or CRC()
            self._publisher = publisher
            self._publisher.Init()
            self._message_factory = message_factory
            self._crc = crc
        except Exception:
            self._release_publisher_guard()
            raise

        self._low_state = low_state_reader
        self._clock = clock
        self._period = 1.0 / float(control_hz)
        self._ramp_seconds = float(ramp_seconds)
        self._release_seconds = float(release_seconds)
        self._velocity_limit = float(velocity_limit_rad_s)
        self._command_timeout = float(command_timeout_s)
        self._condition = threading.Condition(threading.RLock())
        self._publish_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._gate = ArmSdkGate()
        self._last_gate_state = ArmSdkState.DISARMED
        self._target_q = np.zeros(10)
        self._target_tau = np.zeros(10)
        self._target_generation = 0
        self._last_published_generation = 0
        self._target_expires = 0.0
        self._required_mode = REQUIRED_MODE_MACHINE
        self._last_target_update: float | None = None
        self._last_published_at: float | None = None
        self._writes = 0
        self._arm_sdk_weight = 0.0
        self._final_zero_remaining = 0
        self._external_release_generation = 0
        self._external_release_reason: str | None = None
        self._external_release_acknowledged = True
        self._fault_reason: str | None = None
        self._closed = False
        self._close_result: AdapterAck | None = None
        self._thread = threading.Thread(
            target=self._publisher_loop,
            name="g1-arm-sdk-publisher",
            daemon=True,
        )
        self._thread.start()

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck:
        try:
            state = self._validated_lowstate(REQUIRED_MODE_MACHINE)
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
            return AdapterAck(False, "lowstate_not_ready")
        with self._condition:
            if self._fault_reason is not None:
                return AdapterAck(False, "arm_sdk_fault")
            safe = (
                self._thread.is_alive()
                and self._gate.state is ArmSdkState.DISARMED
                and self._final_zero_remaining == 0
                and state["mode_machine"] == REQUIRED_MODE_MACHINE
                and self._clock() < float(deadline_monotonic)
            )
            return AdapterAck(safe, "ok" if safe else "startup_not_safe")

    def apply_target(
        self,
        joint_positions: Sequence[float],
        feedforward_torques: Sequence[float],
        *,
        expires_monotonic: float,
        required_mode_machine: int,
        allow_arm: bool = False,
    ) -> AdapterAck:
        try:
            target = self._vector(joint_positions, "joint_positions")
            torques = self._vector(feedforward_torques, "feedforward_torques")
            expiry = float(expires_monotonic)
            if not math.isfinite(expiry):
                raise ValueError("expiry must be finite")
            if required_mode_machine != REQUIRED_MODE_MACHINE:
                raise ValueError("unsupported mode_machine")
        except (TypeError, ValueError):
            return AdapterAck(False, "invalid_arm_target")

        with self._publish_lock:
            try:
                self._validated_lowstate(required_mode_machine)
            except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
                return AdapterAck(False, "lowstate_not_ready")
            with self._condition:
                now = self._clock()
                if now >= expiry:
                    return AdapterAck(False, "intent_expired_at_publisher")
                if self._closed:
                    return AdapterAck(False, "arm_sdk_closed")
                if self._fault_reason is not None:
                    return AdapterAck(False, "arm_sdk_fault")
                if self._final_zero_remaining:
                    return AdapterAck(False, "arm_sdk_releasing")
                state = self._gate.state
                if state is ArmSdkState.DISARMED:
                    if allow_arm is not True:
                        return AdapterAck(False, "arm_sdk_reclutch_required")
                    armed = self._gate.arm(now, self._ramp_seconds)
                    self._last_gate_state = armed.state
                    self._arm_sdk_weight = armed.weight
                elif state is ArmSdkState.RELEASING:
                    return AdapterAck(False, "arm_sdk_releasing")
                elif state is ArmSdkState.HARD_FAULT:
                    return AdapterAck(False, "arm_sdk_fault")
                self._target_q = target
                self._target_tau = torques
                self._target_generation += 1
                generation = self._target_generation
                self._target_expires = expiry
                self._required_mode = required_mode_machine
                self._last_target_update = now
                self._condition.notify_all()

        with self._condition:
            while self._last_published_generation < generation:
                if self._fault_reason is not None:
                    return AdapterAck(False, "arm_sdk_fault")
                remaining = expiry - self._clock()
                if remaining <= 0.0:
                    return AdapterAck(False, "publish_deadline_missed")
                self._condition.wait(min(remaining, self._period * 2.0))
            return AdapterAck(True)

    def safe_stop(self, *, reason: str, deadline_monotonic: float) -> AdapterAck:
        deadline = float(deadline_monotonic)
        with self._publish_lock:
            with self._condition:
                if self._fault_reason is not None:
                    return AdapterAck(False, "arm_sdk_fault")
                if self._closed:
                    return AdapterAck(False, "arm_sdk_closed")
                if (
                    self._gate.state is ArmSdkState.DISARMED
                    and self._final_zero_remaining == 0
                ):
                    return AdapterAck(True)
                now = self._clock()
                remaining = deadline - now
                final_budget = (self.FINAL_ZERO_WEIGHT_FRAMES + 1) * self._period
                if remaining <= final_budget:
                    return AdapterAck(False, "zero_weight_deadline_too_short")
                release_duration = min(
                    self._release_seconds,
                    max(0.0, remaining - final_budget),
                )
                if self._gate.state is not ArmSdkState.RELEASING:
                    previous = self._last_gate_state
                    released = self._gate.release(
                        now,
                        release_duration,
                        str(reason)[:128] or "safe_stop",
                    )
                    self._record_gate_sample_locked(previous, released)
                self._condition.notify_all()

        with self._condition:
            while not (
                self._gate.state is ArmSdkState.DISARMED
                and self._final_zero_remaining == 0
            ):
                if self._fault_reason is not None:
                    return AdapterAck(False, "arm_sdk_fault")
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    return AdapterAck(False, "zero_weight_ack_timeout")
                self._condition.wait(min(remaining, self._period * 2.0))
            return AdapterAck(True)

    def snapshot(self) -> Mapping[str, object]:
        with self._condition:
            return {
                "topic": ARM_SDK_TOPIC,
                "publisher_count": self.publisher_count,
                "control_state": self._last_gate_state.value,
                "arm_sdk_weight": float(self._arm_sdk_weight),
                "writes": self._writes,
                "final_zero_weight_frames_remaining": self._final_zero_remaining,
                "last_published_monotonic": self._last_published_at,
                "fault_reason": self._fault_reason,
                "external_release_generation": self._external_release_generation,
                "external_release_reason": self._external_release_reason,
                "external_release_acknowledged": self._external_release_acknowledged,
            }

    def external_fault_code(self) -> str | None:
        """Return a bounded asynchronous fault without performing I/O."""

        with self._condition:
            return None if self._fault_reason is None else "arm_sdk_async_fault"

    def external_release_signal(self) -> Mapping[str, object]:
        """Return the latest autonomous TTL release and its zero-frame ACK."""

        with self._condition:
            return {
                "generation": self._external_release_generation,
                "reason": self._external_release_reason,
                "acknowledged": self._external_release_acknowledged,
            }

    def close(self) -> AdapterAck:
        with self._condition:
            if self._close_result is not None:
                return self._close_result
        stop_ack = self.safe_stop(
            reason="arm_sdk_close",
            deadline_monotonic=self._clock() + max(0.25, self._release_seconds + 0.1),
        )
        with self._publish_lock:
            with self._condition:
                self._closed = True
                self._stop_event.set()
                self._condition.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=0.5)
        if self._thread.is_alive():
            result = AdapterAck(False, "publisher_close_timeout")
            with self._condition:
                self._close_result = result
            return result
        try:
            close = getattr(self._publisher, "Close", None)
            if callable(close):
                close()
        except Exception:  # noqa: BLE001 -- uncertain DDS close requires restart
            result = AdapterAck(False, "publisher_dds_close_failed")
            with self._condition:
                self._close_result = result
            return result
        self._release_publisher_guard()
        with self._condition:
            self._close_result = stop_ack
        return stop_ack

    def _publisher_loop(self) -> None:
        while not self._stop_event.is_set():
            started = self._clock()
            try:
                self._publish_once()
            except Exception as exc:  # noqa: BLE001 -- boundary must latch
                self._latch_fault(f"publisher_cycle:{type(exc).__name__}")
            elapsed = self._clock() - started
            self._stop_event.wait(max(0.0, self._period - elapsed))

    def _publish_once(self) -> None:
        with self._publish_lock:
            with self._condition:
                if self._closed or self._fault_reason is not None:
                    return
                now = self._clock()
                sample = self._sample_gate_locked(now)
                if (
                    sample.publish_allowed
                    and self._last_target_update is not None
                    and now - self._last_target_update > self._command_timeout
                ):
                    self._begin_autonomous_release_locked(
                        now,
                        self._release_seconds,
                        "command_timeout",
                    )
                    sample = self._sample_gate_locked(now)
                if sample.publish_allowed and now >= self._target_expires:
                    self._begin_autonomous_release_locked(now, 0.0, "intent_expired")
                    sample = self._sample_gate_locked(now)
                publish_final_zero = self._final_zero_remaining > 0
                if not sample.publish_allowed and not publish_final_zero:
                    return
                target = self._target_q.copy()
                torques = self._target_tau.copy()
                generation = self._target_generation
                required_mode = self._required_mode
                weight = 0.0 if publish_final_zero else sample.weight

            # Final mode/freshness/deadline checks and the DDS write are under
            # the same publish lock as stop/target transitions.
            state = self._validated_lowstate(required_mode)
            if not publish_final_zero and self._clock() >= self._target_expires:
                return
            current_q = np.asarray(state["joint_positions"], dtype=float)
            clipped = self._clip_arm_target(target, current_q)
            message = self._build_message(
                state,
                clipped,
                torques,
                weight,
                required_mode,
            )
            write_timeout = self._period
            if not publish_final_zero:
                write_timeout = max(
                    0.0,
                    min(self._period, self._target_expires - self._clock()),
                )
                if write_timeout <= 0.0:
                    return
            try:
                result = self._publisher.Write(message, timeout=write_timeout)
            except TypeError:
                result = self._publisher.Write(message)
            if result is False:
                raise RuntimeError("DDS Write returned false")

            with self._condition:
                self._writes += 1
                self._last_published_at = self._clock()
                if publish_final_zero:
                    self._final_zero_remaining -= 1
                    if (
                        self._final_zero_remaining == 0
                        and self._external_release_reason is not None
                    ):
                        self._external_release_acknowledged = True
                else:
                    self._last_published_generation = max(
                        self._last_published_generation,
                        generation,
                    )
                self._condition.notify_all()

    def _begin_autonomous_release_locked(
        self,
        now: float,
        duration: float,
        reason: str,
    ) -> None:
        if self._gate.state in (ArmSdkState.DISARMED, ArmSdkState.RELEASING):
            return
        previous = self._last_gate_state
        released = self._gate.release(now, duration, reason)
        self._external_release_generation += 1
        self._external_release_reason = reason
        self._external_release_acknowledged = False
        self._record_gate_sample_locked(previous, released)

    def _sample_gate_locked(self, now: float):
        previous = self._last_gate_state
        sample = self._gate.sample(now)
        return self._record_gate_sample_locked(previous, sample)

    def _record_gate_sample_locked(self, previous, sample):
        if (
            previous in (ArmSdkState.ARMING, ArmSdkState.ARMED, ArmSdkState.RELEASING)
            and sample.state is ArmSdkState.DISARMED
        ):
            self._final_zero_remaining = max(
                self._final_zero_remaining,
                self.FINAL_ZERO_WEIGHT_FRAMES,
            )
        self._last_gate_state = sample.state
        self._arm_sdk_weight = sample.weight
        return sample

    def _validated_lowstate(self, required_mode: int) -> dict:
        raw = dict(self._low_state.read_arm_state())
        q = np.asarray(raw.get("joint_positions"), dtype=float)
        dq = np.asarray(raw.get("joint_velocities"), dtype=float)
        if "all_joint_positions" not in raw:
            raise RuntimeError("LowState full motor vector is missing")
        all_q = np.asarray(raw["all_joint_positions"], dtype=float)
        sampled = float(raw.get("sample_monotonic"))
        mode = raw.get("mode_machine")
        if (
            q.shape != (10,)
            or dq.shape != (10,)
            or all_q.shape != (MOTOR_COUNT,)
            or not np.all(np.isfinite(q))
            or not np.all(np.isfinite(dq))
            or not np.all(np.isfinite(all_q))
            or not math.isfinite(sampled)
        ):
            raise RuntimeError("LowState is invalid")
        age = self._clock() - sampled
        if age < 0.0 or age > 0.1:
            raise RuntimeError("LowState is stale")
        if mode != required_mode:
            raise RuntimeError("mode_machine changed")
        raw.update({
            "joint_positions": q,
            "joint_velocities": dq,
            "all_joint_positions": all_q,
            "sample_monotonic": sampled,
            "mode_machine": mode,
        })
        return raw

    def _build_message(self, state, arm_q, arm_tau, weight, required_mode):
        message = self._message_factory()
        all_q = state["all_joint_positions"]
        for index in range(MOTOR_COUNT):
            command = message.motor_cmd[index]
            command.mode = 1
            command.q = float(all_q[index])
            command.dq = 0.0
            command.tau = 0.0
            if index in WRIST_INDICES:
                command.kp, command.kd = 40.0, 1.5
            elif index in WEAK_INDICES:
                command.kp, command.kd = 80.0, 3.0
            else:
                command.kp, command.kd = 300.0, 3.0
        for vector_index, motor_index in enumerate(ARM_INDICES):
            command = message.motor_cmd[motor_index]
            command.q = float(arm_q[vector_index])
            command.tau = float(arm_tau[vector_index])
        message.mode_pr = 0
        message.mode_machine = required_mode
        message.motor_cmd[WEIGHT_INDEX].q = float(min(1.0, max(0.0, weight)))
        message.crc = self._crc.Crc(message)
        return message

    def _clip_arm_target(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        """Keep the ten-joint trajectory direction while limiting speed."""

        delta = target - current
        motion_scale = float(np.max(np.abs(delta))) / (
            self._velocity_limit * self._period
        )
        return current + delta / max(motion_scale, 1.0)

    def _latch_fault(self, reason: str) -> None:
        with self._publish_lock:
            with self._condition:
                if self._fault_reason is None:
                    self._fault_reason = str(reason)[:128]
                    self._gate.hard_fault(self._fault_reason)
                    self._last_gate_state = ArmSdkState.HARD_FAULT
                    self._arm_sdk_weight = 0.0
                self._condition.notify_all()

    @staticmethod
    def _vector(value, name: str) -> np.ndarray:
        result = np.asarray(value, dtype=float)
        if result.shape != (10,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite ten-joint vector")
        return result.copy()

    def _release_publisher_guard(self) -> None:
        global _active_publishers
        if not self._guard_owned:
            return
        with _publisher_guard:
            _active_publishers = max(0, _active_publishers - 1)
            self._guard_owned = False


__all__ = [
    "ARM_INDICES",
    "ARM_SDK_TOPIC",
    "G1ArmSdkPort",
    "G1LowStateReader",
    "LOWSTATE_TOPIC",
    "REQUIRED_MODE_MACHINE",
]
