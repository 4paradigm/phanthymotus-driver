"""Shared read-only observations; no SDK/ROS imports and no motor writes.

Production weighing uses coherent native DDS status with motor source timestamps.
accept_getter is retained for diagnostics/tests; getter receipt is NOT a source
heartbeat and cannot enable quick physical weighing. No SDK callbacks are installed.
"""
from __future__ import annotations

from collections import deque
import copy
import math
import threading
import time


class MassError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def finite(value, name, low=None, high=None):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (low is not None and value < low) or (high is not None and value > high)):
        raise MassError("BAD_PARAMETER", f"{name}: expected finite number in [{low}, {high}]")
    return float(value)


def vector(values, size, name):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise MassError("BAD_DATA", f"{name}: expected {size} values")
    return [finite(v, name) for v in values]


def quaternion_angle(a, b):
    a, b = vector(a, 4, "quaternion"), vector(b, 4, "quaternion")
    na, nb = math.sqrt(sum(x*x for x in a)), math.sqrt(sum(x*x for x in b))
    if not 0.9 <= na <= 1.1 or not 0.9 <= nb <= 1.1:
        raise MassError("BAD_DATA", "IMU quaternion is not normalized")
    return 2 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(a, b)) / na / nb)))


def fixed_pose_motor_ids(profile):
    # The quick model fixes arm geometry relative to the measured torso. A
    # standing robot's leg stance may change under load; leg velocity, body
    # orientation, modes, faults, temperatures and freshness are still checked.
    return (0, 1, 2, 3, 10, 11, 12, 13, 20) if profile.get("setup_mode") == "quick" else tuple(range(21))


def pose_span_violation(frames, profile):
    """Use the same geometry scope for window spans and fixed references."""
    low = {j: min(f["q"][j] for f in frames) for j in fixed_pose_motor_ids(profile)}
    high = {j: max(f["q"][j] for f in frames) for j in low}
    return _pose_bounds_violation(low, high, profile)


def _pose_bounds_violation(low, high, profile):
    spans = {j: high[j]-low[j] for j in low}
    limits = {j: min(profile.get("max_pose_span_rad", .03), profile["pose_tolerance_rad"])
              if j in profile["motor_ids"] else profile["pose_tolerance_rad"] for j in spans}
    j = max(spans, key=lambda i: spans[i]/limits[i])
    return (j, spans[j], limits[j]) if spans[j] > limits[j] else None


class SharedSampler:
    def __init__(self, maxlen=4096):
        self._condition = threading.Condition()
        self._frames = deque(maxlen=maxlen)
        self._serial = 0
        self._last_source = None
        self._last_timestamp = None
        self._errors = 0
        self._fatal_errors = 0
        self._error = None
        self._duplicates = 0

    def accept_getter(self, motors, imu, workmode):
        """Reuse the already initialized HighController, including raw floats."""
        self.push({"motor_ids": [int(m.motor_id) for m in motors],
                   "q": [float(m.pos) for m in motors],
                   "dq": [float(m.vel) for m in motors],
                   "tau": [float(m.tau) for m in motors],
                   "temperature": [float(m.temperature) for m in motors],
                   "errors": [int(m.error) for m in motors],
                   "quaternion_xyzw": [float(v) for v in imu.ori],
                   "gyro": [float(v) for v in imu.angular_vel],
                   "workmode": int(workmode), "source_seq": None,
                   "source_timestamp": None, "adapter": "shared_highcontroller_getter"})

    def record_error(self, exc, *, recoverable=False):
        with self._condition:
            self._errors += 1
            if not recoverable:
                self._fatal_errors += 1
            self._error = str(exc)
            self._condition.notify_all()

    def push(self, raw):
        frame = copy.deepcopy(raw)
        try:
            ids = frame["motor_ids"]
            if (len(ids) != 21 or any(type(i) is not int for i in ids)
                    or set(ids) != set(range(21))):
                raise MassError("MAPPING", "expected unique SDK motor IDs 0..20; verify mapping")
            # Indexing is by actual motor ID, never assumed array order.
            order = [ids.index(i) for i in range(21)]
            for name in ("q", "dq", "tau", "temperature", "errors"):
                values = vector(frame[name], 21, name)
                frame[name] = [values[i] for i in order]
            frame["motor_ids"] = list(range(21))
            frame["quaternion_xyzw"] = vector(frame["quaternion_xyzw"], 4, "quaternion")
            quaternion_angle(frame["quaternion_xyzw"], frame["quaternion_xyzw"])
            frame["gyro"] = vector(frame["gyro"], 3, "gyro")
            if type(frame["workmode"]) is not int:
                raise MassError("BAD_DATA", "workmode must be an integer")
            seq = frame.get("source_seq")
            if seq is not None and (type(seq) is not int or seq < 0):
                raise MassError("BAD_DATA", "source_seq must be a nonnegative integer")
            stamp = frame.get("source_timestamp")
            if stamp is not None:
                finite(stamp,"source_timestamp",0)
        except (KeyError, TypeError, MassError) as exc:
            self.record_error(exc)
            return False
        with self._condition:
            # Monotonic source IDs are optional, but known repeated/old IDs
            # cannot be given new host times and counted as new feedback.
            if ((seq is not None and self._last_source is not None and seq <= self._last_source)
                    or (stamp is not None and self._last_timestamp is not None and stamp <= self._last_timestamp)):
                self._duplicates += 1
                self._condition.notify_all()
                return False
            if seq is not None:
                self._last_source = seq
            if stamp is not None:
                self._last_timestamp = stamp
            self._serial += 1
            frame.update(serial=self._serial, host_monotonic_s=time.monotonic(),
                         host_time_s=time.time(), error_epoch=self._errors)
            self._frames.append(frame)
            self._condition.notify_all()
        return True

    def diagnostics(self):
        with self._condition:
            latest = copy.deepcopy(self._frames[-1]) if self._frames else None
            frames = list(self._frames)
            span = frames[-1]["host_monotonic_s"] - frames[0]["host_monotonic_s"] if len(frames)>1 else 0
            changes = sum(a["tau"] != b["tau"] for a,b in zip(frames, frames[1:]))
            return {"latest": latest, "host_age_s": time.monotonic()-latest["host_monotonic_s"] if latest else None,
                    "received_count": self._serial, "buffer_count": len(frames),
                    "host_observation_hz": (len(frames)-1)/span if span else None,
                    "torque_changed_observations": changes,
                    "source_duplicate_count": self._duplicates,
                    "read_error_count": self._errors, "last_read_error": self._error,
                    "recoverable_gap_count": self._errors-self._fatal_errors,
                    "freshness_status": "source_sequence" if latest and latest.get("source_seq") is not None else "source_timestamp" if latest and latest.get("source_timestamp") is not None else "host_receipt_only",
                    "effective_sample_count": None,
                    "note": "Getter receipt/change rate cannot prove source refresh rate or independent samples."}

    def read_since(self, cursor, error_epoch):
        """Read only new observations; a getter read never fabricates a sample."""
        with self._condition:
            if error_epoch != self._errors:
                raise MassError("DATA_ERROR", self._error or "state feedback interrupted")
            frames = [f for f in self._frames if f["serial"] > cursor]
            if frames and frames[0]["serial"] != cursor + 1:
                raise MassError("DATA_GAP", "observation buffer overrun")
        # Stored frames are never mutated. Take references under the lock,
        # then copy outside it so a large window cannot block DDS ingestion.
        return self._copy_snapshot(frames, error_epoch)

    def recent(self, duration, error_epoch):
        """Snapshot a real received interval; never synthesize getter samples."""
        with self._condition:
            if error_epoch != self._errors:
                raise MassError("DATA_ERROR", self._error or "state feedback interrupted")
            frames = list(self._frames)
            if not frames:
                raise MassError("STALE_SOURCE", "no received observations")
            cutoff = frames[-1]["host_monotonic_s"]-duration
            start = next((i for i, f in enumerate(frames) if f["host_monotonic_s"] >= cutoff), len(frames)-1)
            frames = frames[max(0, start-1):]
        return self._copy_snapshot(frames, error_epoch)

    def _copy_snapshot(self, frames, error_epoch):
        result = copy.deepcopy(frames)
        with self._condition:
            if error_epoch != self._errors:
                raise MassError("DATA_ERROR", self._error or "state feedback interrupted during snapshot")
        return result

    def collect(self, duration, profile, cancel, *, learn_pose_from=None):
        # Only quick empty preparation may learn its final pose. Weighing and
        # returning home always keep their previously captured target fixed.
        if learn_pose_from is not None:
            if profile.get("setup_mode") != "quick":
                raise MassError("BAD_PARAMETER", "pose learning is only for quick empty preparation")
            learn_pose_from = vector(learn_pose_from, 21, "home_q")
        with self._condition:
            cursor, epoch = self._serial, self._errors
            fatal_epoch = self._fatal_errors
        deadline = time.monotonic() + duration + profile["settle_timeout_s"]
        stable = []
        reason = "INSUFFICIENT_SAMPLES"
        reason_detail = "waiting for new observations"
        best_window_s = 0.0
        pose_low, pose_high = {}, {}
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise MassError("CANCELLED", "sampling cancelled; no motor command sent")
            with self._condition:
                if epoch != self._errors:
                    if learn_pose_from is None or fatal_epoch != self._fatal_errors:
                        raise MassError("DATA_ERROR", self._error or "shared state read failed")
                    # Empty preparation has not established a baseline yet.
                    # After a recoverable pipe gap, start a whole new window;
                    # never combine pre/post-gap observations or extend timeout.
                    cursor, epoch = self._serial, self._errors
                    stable = []
                    pose_low, pose_high = {}, {}
                    reason, reason_detail = "STALE_SOURCE", self._error
                incoming = [f for f in self._frames if f["serial"] > cursor]
                self._condition.wait(timeout=0.02 if not incoming else 0)
            incoming = copy.deepcopy(incoming)
            for frame in incoming:
                if frame["serial"] != cursor + 1:
                    stable = []
                    reason = "DATA_GAP"
                    reason_detail = f"observation serial gap: previous={cursor}, current={frame['serial']}"
                cursor = frame["serial"]
                try:
                    observed_profile = ({**profile, "pose_q": frame["q"]}
                                        if learn_pose_from is not None else profile)
                    check_frame(frame, observed_profile)
                    if stable and frame["host_monotonic_s"]-stable[-1]["host_monotonic_s"] > profile["max_receive_gap_s"]:
                        reason = "STALE_SOURCE"
                        reason_detail = (f"receive gap={frame['host_monotonic_s']-stable[-1]['host_monotonic_s']:.4f}, "
                                         f"limit={profile['max_receive_gap_s']:.4f} s")
                        stable = []
                    if learn_pose_from is not None:
                        elbow = profile["motor_ids"][-1]
                        if abs(frame["q"][elbow] - learn_pose_from[elbow]) < .2:
                            raise MassError("UNSTABLE", "elbow displacement below bend threshold: "
                                            f"motor_id={elbow}, actual={frame['q'][elbow]:.4f}, "
                                            f"home={learn_pose_from[elbow]:.4f}, "
                                            f"delta={abs(frame['q'][elbow]-learn_pose_from[elbow]):.4f}, min=0.2000 rad")
                        # Check measured angle movement even when motor dq is
                        # reported as zero. Never anchor to the first bent frame.
                        if stable:
                            for j in pose_low:
                                pose_low[j] = min(pose_low[j], frame["q"][j])
                                pose_high[j] = max(pose_high[j], frame["q"][j])
                            violation = _pose_bounds_violation(pose_low, pose_high, profile)
                            if violation is not None:
                                j, span, limit = violation
                                reason = "UNSTABLE"
                                reason_detail = (f"joint angle span too high: motor_id={j}, "
                                                 f"q_span={span:.4f}, limit={limit:.4f} rad")
                                stable = []
                        if not stable:
                            pose_low = {j: frame["q"][j] for j in fixed_pose_motor_ids(profile)}
                            pose_high = dict(pose_low)
                    stable.append(frame)
                    best_window_s = max(best_window_s, stable[-1]["host_monotonic_s"]-stable[0]["host_monotonic_s"])
                    # A stable joint alone does not show the hanging object is
                    # stationary: reject torque spread AND drift in the window.
                    if stable[-1]["host_monotonic_s"]-stable[0]["host_monotonic_s"] >= duration:
                        if len(stable) < profile["min_samples"]:
                            raise MassError("INSUFFICIENT_SAMPLES", "too few observations")
                        for j in profile["motor_ids"]:
                            values = [f["tau"][j] for f in stable]
                            if max(values)-min(values) > profile["max_torque_span"]:
                                raise MassError("UNSTABLE", "torque spread exceeds stable-window limit: "
                                                f"motor_id={j}, tau_span={max(values)-min(values):.6f}, "
                                                f"limit={profile['max_torque_span']:.6f}")
                        if time.monotonic()-stable[-1]["host_monotonic_s"] > profile["max_receive_gap_s"]:
                            raise MassError("STALE_SOURCE", "last observation too old")
                        with self._condition:
                            if epoch != self._errors:
                                # Re-enter the interruption handler before
                                # returning a baseline from a raced snapshot.
                                break
                        return stable
                except MassError as exc:
                    # Faults and mismatched conditions invalidate this attempt;
                    # ordinary settling may wait for a complete stable window.
                    if exc.code not in ("UNSTABLE", "INSUFFICIENT_SAMPLES"):
                        raise
                    reason = exc.code
                    reason_detail = str(exc)
                    stable = []
            if stable and time.monotonic()-stable[-1]["host_monotonic_s"] > profile["max_receive_gap_s"]:
                stable = []
                reason = "STALE_SOURCE"
                reason_detail = "feedback gap exceeds max_receive_gap_s"
        raise MassError(reason, "no complete continuous stable window before timeout; "
                        f"best_window_s={best_window_s:.3f}, required_s={duration:.3f}; "
                        f"last_rejection: {reason_detail}")


def check_frame(frame, p):
    # Check before UNSTABLE: live weighing tolerates motion while loading, but
    # must never turn an unverified moving frame into a provisional kg reading.
    # Quick sessions also reject the opt-in used by older quick profiles.
    if (frame.get("source_seq") is None and frame.get("source_timestamp") is None
            and (p.get("setup_mode") == "quick" or not p["accept_host_only_freshness"])):
        raise MassError("FRESHNESS_UNVERIFIED",
                        "无法验证机器人反馈是否更新：当前接口无源序号或时间戳，称重已拒绝；需接入可验证的新鲜反馈。")
    if frame["workmode"] != p["workmode"] or frame["workmode"] in (26,30):
        raise MassError("MODE_MISMATCH", "workmode/protection differs from profile")
    if p.get("setup_mode") == "quick":
        # Match the existing motion_state distinction: known fault codes block,
        # undocumented status (e.g. 1) remains visible in the prototype output.
        if set(frame["errors"]) & {2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14}:
            raise MassError("MOTOR_FAULT", "documented motor fault")
    elif any(frame["errors"]):
        raise MassError("MOTOR_FAULT", "nonzero motor error (including undocumented codes)")
    temp_motor = max(range(21), key=lambda i: frame["temperature"][i])
    if frame["temperature"][temp_motor] >= p["max_temperature_c"]:
        raise MassError("OVER_TEMPERATURE", "temperature exceeds configured limit: "
                        f"motor_id={temp_motor}, current={frame['temperature'][temp_motor]:.2f}, "
                        f"limit={p['max_temperature_c']:.2f} C")
    differences = [abs(a-b) for a,b in zip(frame["q"],p["pose_q"])]
    j = max(fixed_pose_motor_ids(p), key=lambda i: differences[i])
    if differences[j] > p["pose_tolerance_rad"]:
        raise MassError("POSE_MISMATCH", "arm/waist/body joint pose differs from profile: "
                        f"motor_id={j}, actual={frame['q'][j]:.4f}, target={p['pose_q'][j]:.4f}, "
                        f"delta={differences[j]:.4f} rad, limit={p['pose_tolerance_rad']:.4f} rad")
    if quaternion_angle(frame["quaternion_xyzw"], p["quaternion_xyzw"]) > p["body_tolerance_rad"]:
        raise MassError("BODY_MISMATCH", "body orientation differs from profile")
    j = max(range(21), key=lambda i: abs(frame["dq"][i]))
    axis = max(range(3), key=lambda i: abs(frame["gyro"][i]))
    violations = []
    if abs(frame["dq"][j]) > p["max_velocity_rad_s"]:
        violations.append(f"joint velocity: motor_id={j}, dq={frame['dq'][j]:.4f}, limit={p['max_velocity_rad_s']:.4f}")
    if abs(frame["gyro"][axis]) > p["max_gyro_rad_s"]:
        violations.append(f"body gyro: axis={axis}, gyro={frame['gyro'][axis]:.4f}, limit={p['max_gyro_rad_s']:.4f}")
    if violations:
        raise MassError("UNSTABLE", "; ".join(violations))
    if any(abs(frame["tau"][j]) >= p["torque_abs_limit"] for j in p["motor_ids"]):
        raise MassError("SATURATED", "torque outside engineer-confirmed measurement limits")
