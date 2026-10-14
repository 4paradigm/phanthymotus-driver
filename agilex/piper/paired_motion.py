"""One owner for both CAN streams, shared by VR teleoperation and trajectory replay."""
from contextlib import ExitStack
import copy
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

import numpy as np

from agilex.piper.control import ready_to_move
from agilex.piper.motion.kinematics import Calibration, Chain, RelativeMapping, SIDES, vector
from agilex.piper.runtime import confirm
from agilex.piper.trajectory import Recorder, list_recordings, read_recording
from common.motion.worker import NumericalWorker
from common.teleop_contract import MAX_INPUT_AGE_NS, canonical_instance, validate_input

PERIOD = .05


def numerical_worker(path):
    return NumericalWorker(path, driver_dir=Path(__file__).resolve().parents[2],
                           solver_module="agilex.piper.motion.kinematics", solver_class="PiperIK")


class PairedMotion:
    def __init__(self, arms, calibration_path, directory, *, worker_factory=numerical_worker, clock_id=None):
        self.arms, self.calibration_path = arms, calibration_path
        self.directory = Path(directory) / "recordings"
        self.worker_factory = worker_factory
        boot = Path("/proc/sys/kernel/random/boot_id")
        self.clock_id = clock_id or (boot.read_text().strip() if boot.exists() else "unavailable")
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = self.worker = self.recorder = None
        self.mode = "idle"
        self.reason = None
        self.latest = self.source = None
        self.sequence = -1
        self.session_id = None
        self.last_action = None
        self.holds = {}
        self.last_q = self.last_width = None
        self.last_send = None
        self.output = False
        self.deadline = None
        self.calibration = None
        self.binding = None
        self.feedback = None
        self.rejected = 0
        self.record_start_lock = threading.Lock()

    def configuration(self):
        if not self.calibration_path:
            raise ValueError("verified_piper_calibration_required")
        calibration = Calibration(self.calibration_path)
        for side in SIDES:
            if calibration.profile["arms"][side]["can_interface"] != self.arms[side].config["can_interface"]:
                raise ValueError("calibration_can_binding_mismatch")
        binding = {"model_sha256": calibration.profile["model_sha256"],
                   "calibration_sha256": calibration.sha256,
                   "calibration_id": calibration.profile["calibration_id"],
                   "arms": {side: self.arms[side].config["can_interface"] for side in SIDES},
                   "joint_names": [f"{side}_joint{i}" for side in SIDES for i in range(1, 7)]}
        return calibration, binding

    def read_pair(self, permission=True):
        states, grips, q, widths = {}, {}, [], []
        for side in SIDES:
            arm = self.arms[side]
            if permission:
                arm.require_motion_permission()
            state = arm.read_state(monitor=True)
            grip = arm.connected().get_gripper()
            ready_to_move(state)
            if not grip.enabled or grip.error_code:
                raise RuntimeError(side + "_gripper_not_ready")
            if any(not math.isfinite(t) or abs(time.time()-t) > .5 for t in (state.feedback_time, grip.feedback_time)):
                raise RuntimeError(side + "_feedback_stale")
            q.extend(math.radians(v) for v in vector(state.joints_deg, 6, "feedback"))
            if not math.isfinite(grip.width_mm):
                raise RuntimeError(side + "_invalid_gripper_feedback")
            widths.append(grip.width_mm)
            states[side], grips[side] = state, grip
        return {"states": states, "grips": grips, "q": q, "widths": widths,
                "feedback_time": [states[s].feedback_time for s in SIDES] + [grips[s].feedback_time for s in SIDES]}

    def _check_targets(self, q, widths, calibration, *, hardware=True):
        q = vector(q, 12, "target_q")
        widths = vector(widths, 2, "target_widths")
        chain = Chain()
        for i, side in enumerate(SIDES):
            joints = q[i*6:(i+1)*6]
            if np.any(joints < chain.lower) or np.any(joints > chain.upper):
                raise ValueError(side + "_model_joint_limit")
            calibration.check_workspace(side, chain.fk(joints, calibration.tcp[side]))
            if hardware:
                self.arms[side].connected().validate_stream_target(joints.tolist(), float(widths[i]))

    def _check_recording(self, recording, sample, calibration):
        # Read hardware limits once, rather than query CAN for every stored frame.
        chain = Chain()
        lower, upper, ranges = [], [], []
        for side in SIDES:
            driver = self.arms[side].connected()
            for i in range(6):
                lo, hi = driver.arm.GetSDKJointLimitParam(f"j{i+1}")
                configured = driver.joint_limits_deg
                if configured is not None:
                    lo, hi = max(lo, math.radians(configured[i][0])), min(hi, math.radians(configured[i][1]))
                lower.append(max(lo, chain.lower[i]))
                upper.append(min(hi, chain.upper[i]))
            ranges.append(driver.gripper_range())
        for frame in recording["frames"]:
            q, widths = frame["target_q"], frame["target_width_mm"]
            if any(not lo <= v <= hi for v, lo, hi in zip(q, lower, upper)):
                raise ValueError("recording_joint_limit")
            if any(not lo <= v <= hi for v, (lo, hi) in zip(widths, ranges)):
                raise ValueError("recording_gripper_limit")
            self._check_targets(q, widths, calibration, hardware=False)
        first = recording["frames"][0]
        if (any(abs(a-b) > math.radians(.05) for a, b in zip(sample["q"], first["target_q"])) or
                any(abs(a-b) > .5 for a, b in zip(sample["widths"], first["target_width_mm"]))):
            raise ValueError("replay_start_mismatch_no_automatic_return")

    def prepare(self, mode, args):
        """Preview or reserve both operation gates before launching any worker."""
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError("paired_motion_busy")
            if self.recorder is not None and self.recorder.thread.is_alive():
                raise RuntimeError("recording_still_stopping")
            calibration, binding = self.configuration()
            sample = self.read_pair()
            self._check_targets(sample["q"], sample["widths"], calibration)
            recording = None
            if mode == "replay":
                recording = read_recording(self.directory, args.get("filename"), binding)
                self._check_recording(recording, sample, calibration)
            key = mode + ":" + calibration.sha256 + ":" + (recording["sha256"] if recording else "")
            extras = {s: (sample["grips"][s].width_mm, sample["grips"][s].enabled,
                           sample["grips"][s].error_code) for s in SIDES}
            if args.get("execute", False) is not True:
                tokens = [self.arms[s].previews.put(key, sample["states"][s], extras[s])["preview_id"] for s in SIDES]
                return {"allowed": True, "executed": False, "preview_id": ":".join(tokens), "valid_for_seconds": 60,
                        "mode": mode, "binding": binding, "start_q": sample["q"], "start_width_mm": sample["widths"],
                        "duration_seconds": recording["duration_seconds"] if recording else None,
                        "message": "确认双臂、夹爪及左右绑定；启动不会切模式或使能。"}
            confirm(args, "workspace_clear")
            confirm(args, "take_control")
            token = args.get("preview_id")
            if not isinstance(token, str) or len(token.split(":")) != 2:
                raise ValueError("paired_preview_required")
            reservations = ExitStack()
            try:
                for side in SIDES:
                    reservations.enter_context(self.arms[side].operation("piper_teleop" if mode == "teleop" else "piper_motion", mode))
                # Recheck after reservation: a manual operation may have completed during preview.
                sample = self.read_pair()
                for side, part in zip(SIDES, token.split(":")):
                    grip = sample["grips"][side]
                    self.arms[side].previews.consume(part, key, sample["states"][side], (grip.width_mm, grip.enabled, grip.error_code))
                if recording:
                    self._check_recording(recording, sample, calibration)
                self.mode, self.reason = mode, "starting"
                self.calibration, self.binding = calibration, binding
                self.cancel = threading.Event()
                self.latest = self.source = None
                self.sequence = -1
                self.session_id = uuid4().hex
                self.holds, self.output, self.deadline = {}, False, None
                self.last_q, self.last_width = sample["q"], sample["widths"]
                self.last_send = None
                self.recorder = None
                self.last_action = {"action_id": self.session_id, "status": "running", "mode": mode}
                self.thread = threading.Thread(target=self._run, args=(reservations, recording),
                                               name="piper-paired-motion", daemon=True)
                self.thread.start()
            except BaseException:
                reservations.close()
                raise
            return {"state": "running", "action_id": self.session_id, "mode": mode,
                    "message": "会话已提交；请查看状态。启动未使能或切换模式。"}

    def receive(self, value):
        with self.lock:
            if self.mode != "teleop" or self.cancel.is_set():
                return False
            try:
                frame = validate_input(value, instance_id=value.get("instance_id"), clock_id=self.clock_id,
                                       now_ns=time.monotonic_ns())
                source = (canonical_instance(frame["instance_id"]), frame["device_id"], frame["connection_epoch"], frame["space_epoch"])
                if self.source is not None and source != self.source:
                    self.cancel.set()
                    self.reason = "input_identity_changed_restart_required"
                    return False
                if frame["sequence"] <= self.sequence:
                    raise ValueError("input_sequence_not_increasing")
                if not all(frame[s]["tracked"] for s in (*SIDES, "head_reference")):
                    self.cancel.set()
                    self.reason = "tracking_lost_restart_required"
                    return False
                self.source, self.sequence, self.latest = source, frame["sequence"], frame
                return True
            except (ValueError, TypeError, AttributeError, OverflowError):
                self.rejected += 1
                return False

    def request_stop(self, reason="operator_stop"):
        with self.lock:
            active = self.thread is not None and self.thread.is_alive()
            if active:
                self.reason = reason
                self.cancel.set()
            worker = self.worker
            for arm in self.arms.values():
                arm.previews.invalidate()
                if active:
                    arm.cancel.set()
        if worker is not None:
            worker.interrupt()
        if active:
            for arm in self.arms.values():
                if arm.driver is not None:
                    arm.driver.cancel_motion()
        return {"state": "stopping" if active else "idle", "operation_stopping": active,
                "motor_torque_released": False}

    def _watchdog(self, finished):
        while not finished.wait(.02):
            with self.lock:
                deadline, active = self.deadline, self.output
            if active and deadline is not None and time.monotonic() >= deadline:
                self.request_stop("stream_watchdog_expired")
                return

    def _run(self, reservations, recording):
        finished = threading.Event()
        watchdog = threading.Thread(target=self._watchdog, args=(finished,), name="piper-paired-watchdog", daemon=True)
        watchdog.start()
        error = None
        try:
            if recording is None:
                worker = self.worker_factory(self.calibration_path)
                with self.lock:
                    self.worker = worker
                if worker.profile_sha256 != self.calibration.sha256:
                    raise ValueError("calibration_changed_during_start")
                worker.self_test(self.read_pair()["q"])
                self._teleop(worker)
            else:
                self._replay(recording)
        except Exception as exc:
            error = str(exc)
        finally:
            finished.set()
            watchdog.join(.3)
            with self.lock:
                cancelled = self.cancel.is_set()
                reason = self.reason if cancelled else error
                self.cancel.set()
                worker, self.worker = self.worker, None
            holds = {}
            for side, arm in self.arms.items():
                if arm.driver is not None:
                    try:
                        arm.driver.cancel_motion()
                        if self.output:
                            holds[side] = arm.driver.stream_hold()
                    except Exception as exc:
                        holds[side] = {"arm_hold_sent": False, "gripper_hold_sent": False, "errors": [str(exc)]}
            if worker is not None:
                try:
                    worker.close()
                except Exception as exc:
                    error = error or str(exc)
                    reason = reason or error
            normal_stop = reason in (None, "operator_stop", "card_stopped", "shutdown")
            if self.recorder is not None and self.recorder.thread.is_alive():
                if error and not cancelled or not normal_stop:
                    self.recorder.fail(reason or error)
                self.recorder.stop()
            reservations.close()
            with self.lock:
                self.holds = holds
                state = "error" if not normal_stop or error and not cancelled else "cancelled" if cancelled else "completed"
                self.last_action = {"action_id": self.session_id, "status": state, "mode": self.mode,
                                    "result": {"reason": reason, "holds": copy.deepcopy(self.holds)}}
                self.mode, self.reason, self.output = "idle", reason, False
            if recording is not None:
                self.arms["left"].acp_complete(self.session_id, state, self.last_action["result"], "piper_motion")

    def _teleop(self, worker):
        mapping = None
        started = time.monotonic()
        last_sequence = -1
        while not self.cancel.is_set():
            tick = time.monotonic()
            sample = self.read_pair()
            with self.lock:
                frame = copy.deepcopy(self.latest)
            if frame is None:
                if tick - started > 5:
                    raise RuntimeError("input_timeout_restart_required")
                self.reason = "waiting_for_input"
                self.cancel.wait(PERIOD)
                continue
            expiry = frame["received_monotonic_ns"]/1e9 + MAX_INPUT_AGE_NS/1e9
            if tick >= expiry:
                raise RuntimeError("input_stale_restart_required")
            if mapping is None:
                if any(frame[s]["grip"] >= .5 for s in SIDES):
                    self.reason = "release_both_grips_to_map"
                    self.cancel.wait(PERIOD)
                    continue
                mapping = RelativeMapping(self.calibration, sample["q"], frame, dict(zip(SIDES, sample["widths"])))
                self.reason = "mapped_waiting_for_both_grips"
            held = all(frame[s]["grip"] >= .5 for s in SIDES)
            if held and frame["sequence"] != last_sequence:
                values, widths = mapping.targets(frame, {s: self.arms[s].driver.gripper_range() for s in SIDES})
                identity = {"boot_id": self.clock_id, "session_id": self.session_id, "mapping_epoch": 0,
                            "generation": self.session_id, "source_seq": frame["sequence"]}
                answer = worker.call("solve", {"values": values, "q": sample["q"], "commanded": self.last_q,
                                              "measured_ns": time.monotonic_ns()},
                                     identity=identity, deadline=min(expiry, time.monotonic()+.15))
                last_sequence = frame["sequence"]
                with self.lock:
                    current = self.latest
                    held = current is not None and all(current[s]["grip"] >= .5 for s in SIDES)
                if held:
                    self._send(answer["q"], [widths[s] for s in SIDES], expiry, sample)
                    self.reason = "following"
            elif self.output:
                self._send(self.last_q, self.last_width, expiry, sample)
                self.reason = "holding" if not held else "waiting_for_fresh_frame"
            # A slow solve never creates a catch-up burst on the CAN bus.
            self.cancel.wait(PERIOD)

    def _send(self, q, widths, expiry, sample):
        if self.cancel.is_set() or time.monotonic() >= expiry:
            raise RuntimeError("stream_cancelled_or_expired")
        q, widths = vector(q, 12, "target_q"), vector(widths, 2, "target_widths")
        now = time.monotonic()
        dt = PERIOD if self.last_send is None else min(PERIOD, max(0, now-self.last_send))
        previous_q, previous_width = np.asarray(self.last_q), np.asarray(self.last_width)
        target_q = previous_q + np.clip(q-previous_q, -math.radians(1)*dt, math.radians(1)*dt)
        target_width = previous_width + np.clip(widths-previous_width, -10*dt, 10*dt)
        if (np.max(np.abs(np.asarray(sample["q"])-previous_q)) > math.radians(5) or
                np.max(np.abs(np.asarray(sample["widths"])-previous_width)) > 5):
            raise RuntimeError("paired_tracking_error")
        self._check_targets(target_q, target_width, self.calibration)
        frame_ns = time.monotonic_ns()
        actual_q, actual_width, offsets = [], [], []
        # Both targets are checked before the first write. CAN itself is not atomic.
        for i, side in enumerate(SIDES):
            self.arms[side].require_motion_permission()
            if self.cancel.is_set() or time.monotonic() >= expiry:
                raise RuntimeError("stream_cancelled_or_expired")
            self.output = True  # Even a partially failed SDK call needs best-effort hold.
            self.deadline = expiry
            result = self.arms[side].driver.stream_target(target_q[i*6:(i+1)*6].tolist(), float(target_width[i]))
            offsets.append(time.monotonic_ns()-frame_ns)
            actual_q.extend(result["q"])
            actual_width.append(result["width_mm"])
        if offsets[-1] > 50_000_000:
            raise RuntimeError("pair_send_timeout")
        self.last_q, self.last_width, self.last_send = actual_q, actual_width, time.monotonic()
        frame = {"type": "frame", "t_ns": frame_ns, "target_q": actual_q, "target_width_mm": actual_width,
                 "feedback_q": sample["q"], "feedback_width_mm": sample["widths"],
                 "feedback_time": sample["feedback_time"], "send_ok": [True, True], "send_offset_ns": offsets}
        with self.lock:
            self.feedback = frame
            recorder = self.recorder
        if recorder is not None:
            recorder.append(frame)

    def _replay(self, recording):
        frames = recording["frames"]
        start = time.monotonic() + PERIOD
        origin = frames[0]["t_ns"]
        for frame in frames:
            scheduled = start + (frame["t_ns"]-origin)/1e9
            if self.cancel.wait(max(0, scheduled-time.monotonic())):
                return
            if time.monotonic()-scheduled > .1:
                raise RuntimeError("replay_late_no_catchup")
            self._send(frame["target_q"], frame["target_width_mm"], scheduled+.3, self.read_pair())
        deadline = time.monotonic() + 2
        while not self.cancel.is_set():
            sample = self.read_pair()
            if (max(abs(a-b) for a, b in zip(sample["q"], frames[-1]["target_q"])) <= math.radians(.5)
                    and max(abs(a-b) for a, b in zip(sample["widths"], frames[-1]["target_width_mm"])) <= .5):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("replay_final_feedback_timeout")
            self._send(frames[-1]["target_q"], frames[-1]["target_width_mm"], time.monotonic()+.15, sample)
            self.cancel.wait(PERIOD)

    def record_start(self):
        with self.record_start_lock:
            with self.lock:
                if self.mode != "teleop" or self.cancel.is_set():
                    raise RuntimeError("recording_requires_active_teleop_session")
                if self.recorder is not None and self.recorder.thread.is_alive():
                    raise RuntimeError("recording_already_active_or_stopping")
                binding, session = copy.deepcopy(self.binding), self.session_id
            recorder = Recorder(self.directory, binding)
            with self.lock:
                if self.session_id != session or self.cancel.is_set():
                    recorder.fail("teleop_stopped_during_record_start")
                    recorder.stop()
                    raise RuntimeError("teleop_stopped_during_record_start")
                self.recorder = recorder
                return recorder.status()

    def record_stop(self):
        with self.lock:
            recorder = self.recorder
        return recorder.stop() if recorder else {"recording": False, "complete": False}

    def inspect_recording(self, filename):
        _, binding = self.configuration()
        result = read_recording(self.directory, filename, binding)
        return {k: v for k, v in result.items() if k != "frames"} | {"frames": len(result["frames"]), "hardware_checked": False}

    def status(self):
        with self.lock:
            return {"state": self.mode, "reason": self.reason, "session_id": self.session_id,
                    "source_sequence": self.sequence, "source": self.source, "rejected_inputs": self.rejected,
                    "output": self.output, "last_action": copy.deepcopy(self.last_action),
                    "feedback": copy.deepcopy(self.feedback), "holds": copy.deepcopy(self.holds),
                    "recording": self.recorder.status() if self.recorder else None}

    def recordings(self):
        return list_recordings(self.directory)

    def close(self):
        self.request_stop("shutdown")
        if self.thread is not None:
            self.thread.join(3)
        self.record_stop()
