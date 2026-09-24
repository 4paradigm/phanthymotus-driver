"""T800 two-card session and command watchdog; no ROS imports.

The numerical worker never publishes. A separate 100 Hz owner validates the
session, input and robot feedback immediately before each command. Stop and
publication share one lock so a late solve cannot restart an ended session.
"""
from __future__ import annotations

import copy
import html
import math
from pathlib import Path
import secrets
import threading
import time

import numpy as np

from common.teleop_contract import (
    COMMAND_TOPIC, STATE_TOPIC, FEEDBACK_SCHEMA, canonical_instance, validate_input,
)
from control import T800_JOINT_NAMES, T800_JOINT_POSITION_LIMITS
from teleop_kinematics import DualArmModel, RelativeMapping, smooth_reference


INPUT_TTL_NS = 300_000_000
SOLVE_TTL_NS = 150_000_000


class TeleopControl:
    def __init__(self, config, feedback, publish, *, clock=time.monotonic_ns,
                 clock_id=None, model=None):
        self.mode = config.get("mode", "shadow")
        self.scale = config.get("position_scale", .5)
        if self.mode not in ("shadow", "live"):
            raise ValueError("invalid_teleop_mode")
        if type(self.scale) not in (int, float) or not math.isfinite(self.scale) or not 0 < self.scale <= 1:
            raise ValueError("invalid_position_scale")
        self.model = model or DualArmModel()
        self.robot_feedback, self.publish, self.clock = feedback, publish, clock
        boot = Path("/proc/sys/kernel/random/boot_id")
        self.clock_id = clock_id or (boot.read_text().strip() if boot.exists() else None)
        self.lock = threading.RLock()
        self.server_epoch = secrets.token_hex(16)
        self.instance_id = "teleop_control"
        self.running = self.sent = self.engaged = self.release_pending = False
        self.state, self.reason = "idle", None
        self.generation = self.mapping_epoch = self.monitor_sequence = 0
        self.started_ns = 0
        self.source = self.identity = self.frame = self.mapping = None
        self.sequence = self.received_ns = self.source_ns = -1
        self.q = self.body = None
        self.velocity = np.zeros(10)
        self.solution = None
        self.residual = None
        self.last_tick = None
        self.release_seen = False
        self.rejected = 0
        self.input_error = self.publish_error = None
        self._claimed_job = None

    def _robot(self):
        snapshot = self.robot_feedback()
        for key, limit in (("joints", .3), ("motion", .5), ("planner", .5)):
            data = snapshot.get(key, {})
            age = data.get("age_sec")
            if (type(age) not in (int, float) or not math.isfinite(age)
                    or not 0 <= age <= limit or data.get("stale", False)):
                raise ValueError(key + "_feedback_stale")
        if snapshot["motion"].get("current_motion_task") != "lower_body_balance":
            raise ValueError("requires_lower_body_balance")
        if snapshot["planner"].get("status") != 1:
            raise ValueError("joint_planner_not_idle")
        joints = snapshot["joints"].get("joints")
        if not isinstance(joints, list) or len(joints) != 25:
            raise ValueError("requires_complete_25_joint_feedback")
        q = []
        for i, joint in enumerate(joints):
            value = joint.get("q")
            if (joint.get("idx") != i or joint.get("name") != T800_JOINT_NAMES[i]
                    or type(value) not in (int, float) or not math.isfinite(value)
                    or not T800_JOINT_POSITION_LIMITS[i][0] <= value <= T800_JOINT_POSITION_LIMITS[i][1]):
                raise ValueError("invalid_joint_feedback")
            q.append(value)
        return np.asarray(q)

    def begin(self, args):
        with self.lock:
            if args.get("input_topic") != COMMAND_TOPIC:
                raise ValueError("connect_teleop_device_command_port")
            if not self.clock_id:
                raise ValueError("host_boot_clock_unavailable")
            if self.release_pending:
                raise ValueError("override_release_pending")
            if self.running:
                return self.info()
            q = self._robot()
            if np.any(q[13:23] < self.model.lower) or np.any(q[13:23] > self.model.upper):
                raise ValueError("initial_arm_pose_outside_soft_limits")
            self.instance_id = canonical_instance(args.get("instance_id", "teleop_control"))
            self.running = True
            self.engaged = False
            self.started_ns = self.clock()
            self.generation += 1
            self.source = self.identity = self.frame = self.mapping = None
            self.sequence = self.received_ns = self.source_ns = -1
            self.q, self.body = q[13:23].copy(), q[:13].copy()
            self.velocity[:] = 0
            self.solution = self.residual = self._claimed_job = None
            self.last_tick = None
            self.release_seen = False
            self.input_error = self.publish_error = None
            self.state, self.reason = "ready", "release_both_grips_to_calibrate"
            return self.info()

    def motion_active(self):
        with self.lock:
            return self.running or self.release_pending

    def _hold(self, reason, *, require_release=False):
        self.generation += 1
        self.solution = None
        self._claimed_job = None
        self.velocity[:] = 0
        self.state, self.reason = "hold", reason
        if require_release:
            self.release_seen = False

    def _release(self):
        if not self.sent and not self.release_pending:
            return
        self.release_pending = True
        try:
            self.publish(np.zeros(10), np.zeros(10), 0.)
        except Exception as exc:
            self.publish_error = str(exc)
            self.state, self.reason = "error", "override_release_pending"
        else:
            self.sent = self.release_pending = False
            self.publish_error = None
            if not self.running:
                self.state, self.reason = "idle", "project_stopped"

    def halt(self):
        with self.lock:
            self.running = False
            self.engaged = False
            self._hold("project_stopped", require_release=True)
            self.frame = self.mapping = None
            self.state = "idle"
            self._release()
            return self.info()

    def _fault(self, reason):
        self._hold(reason, require_release=True)
        self.state = "error"
        self._release()
        # Keep the session reserved until an explicit project stop. Recovery of
        # mode/feedback alone must not silently reactivate a failed controller.

    def receive(self, value):
        with self.lock:
            if not self.running or self.state == "error":
                return False
            try:
                source = self.source or canonical_instance(value.get("instance_id"))
                frame = validate_input(value, instance_id=source, clock_id=self.clock_id, now_ns=self.clock())
                if frame["received_monotonic_ns"] < self.started_ns:
                    raise ValueError("input_precedes_session")
                for name in ("head_reference", "left", "right"):
                    if frame[name]["tracked"] and max(abs(v) for v in frame[name]["position"]) > 100:
                        raise ValueError("input_position_out_of_range")
                identity = (frame["device_id"], frame["connection_epoch"], frame["space_epoch"])
                if self.identity:
                    if identity[0] != self.identity[0]:
                        raise ValueError("input_device_mismatch")
                    if identity[1] < self.identity[1] or identity[2] < self.identity[2]:
                        raise ValueError("old_input_epoch")
                    if identity == self.identity and (frame["sequence"] <= self.sequence
                            or frame["received_monotonic_ns"] <= self.received_ns
                            or frame["source_monotonic_ns"] <= self.source_ns):
                        raise ValueError("out_of_order_input")
                if self.identity and identity != self.identity:
                    self._hold("input_epoch_changed", require_release=True)
                    if identity[2] != self.identity[2]:
                        self.mapping = None
                self.source, self.identity = source, identity
                self.frame = frame
                self.sequence = frame["sequence"]
                self.received_ns = frame["received_monotonic_ns"]
                self.source_ns = frame["source_monotonic_ns"]
                self.input_error = None
                if not all(frame[s]["tracked"] for s in ("head_reference", "left", "right")):
                    self._hold("tracking_lost", require_release=True)
                elif not all(frame[s]["grip"] >= .7 for s in ("left", "right")):
                    self._hold("grip_released")
                    if all(frame[s]["grip"] <= .2 for s in ("left", "right")):
                        self.release_seen = True
                return True
            except (ValueError, TypeError, AttributeError) as exc:
                self.input_error = str(exc)
                self.rejected += 1
                return False

    def _enabled(self, now):
        return (self.frame is not None and 0 <= now-self.received_ns <= INPUT_TTL_NS
            and self.mapping is not None and self.release_seen
            and all(self.frame[s]["tracked"] for s in ("head_reference", "left", "right"))
            and all(self.frame[s]["grip"] >= .7 for s in ("left", "right")))

    def solve_once(self):
        with self.lock:
            now = self.clock()
            if not self.running or self.state == "error" or not self._enabled(now):
                return False
            key = (self.generation, self.sequence)
            if key == self._claimed_job:
                return False
            self._claimed_job = key
            targets = self.mapping.targets(self.frame)
            seed = self.q.copy()
            reference = self.mapping.reference
            received = self.received_ns
        # Deliberately outside the command/stop lock. The watchdog can hold or
        # release independently of numerical work, including a blocked solver.
        try:
            answer, residual = self.model.solve(targets, seed, reference=reference)
            answer = np.asarray(answer)
            if (answer.shape != (10,) or not np.isfinite(answer).all()
                    or np.any(answer < self.model.lower) or np.any(answer > self.model.upper)
                    or not np.isfinite(residual).all()):
                raise ValueError("invalid_ik_result")
        except Exception as exc:
            with self.lock:
                if key[0] == self.generation and self.running:
                    self._fault("ik_failed: " + str(exc))
            return False
        with self.lock:
            if (not self.running or key[0] != self.generation or not self._enabled(self.clock())
                    or not 0 <= self.clock()-received <= SOLVE_TTL_NS):
                return False
            self.solution = (answer.copy(), received)
            self.residual = list(residual)
            return True

    def tick(self):
        with self.lock:
            if self.release_pending:
                self._release()
            if not self.running or self.state == "error":
                return
            now = self.clock()
            elapsed = .01 if self.last_tick is None else (now-self.last_tick)/1e9
            self.last_tick = now
            try:
                q = self._robot()
                if not self.engaged:
                    if np.any(q[13:23] < self.model.lower) or np.any(q[13:23] > self.model.upper):
                        raise ValueError("initial_arm_pose_outside_soft_limits")
                    self.q = q[13:23].copy()
                if np.max(np.abs(q[:13]-self.body)) > .15:
                    raise ValueError("lower_body_or_torso_moved")
                if self.sent and np.max(np.abs(q[13:23]-self.q)) > .35:
                    raise ValueError("arm_tracking_error")
            except (ValueError, TypeError, KeyError) as exc:
                self._fault(str(exc))
                return
            if elapsed <= 0 or elapsed > .05:
                self._hold("command_loop_gap", require_release=True)
            if self.frame is None or not 0 <= now-self.received_ns <= INPUT_TTL_NS:
                self._hold("input_stale", require_release=True)
            elif self.mapping is None and self.release_seen and all(
                self.frame[s]["tracked"] for s in ("head_reference", "left", "right")
            ) and all(self.frame[s]["grip"] <= .2 for s in ("left", "right")):
                try:
                    # Before any output, calibrate against measured joints, not
                    # the driver's zero-filled legacy joint position cache.
                    if not self.engaged:
                        self.q = q[13:23].copy()
                    self.mapping = RelativeMapping(self.frame, self.model.poses(self.q), self.scale, self.q)
                    self.mapping_epoch += 1
                    self.state, self.reason = "ready", "press_both_grips"
                except ValueError as exc:
                    self.reason = str(exc)
            candidate, velocity = self.q, np.zeros(10)
            moving = self._enabled(now) and self.solution is not None
            if moving and not 0 <= now-self.solution[1] <= SOLVE_TTL_NS:
                self._hold("solver_stale", require_release=True)
                moving = False
            if moving:
                candidate, velocity = smooth_reference(self.q, self.velocity, self.solution[0],
                    min(elapsed, .02), self.model.lower, self.model.upper)
                self.state = "limited" if max(self.residual) > .015 else "following"
                self.reason = "local_workspace_boundary" if self.state == "limited" else None
            elif not self.engaged:
                return  # Starting the project/calibrating never sends a command.
            try:
                if self.mode == "live":
                    # Mark first: a publisher can send and then raise. A failed
                    # call must still attempt release, retaining ownership until
                    # a release publish succeeds.
                    self.sent = True
                    self.publish(candidate.copy(), velocity.copy(), 1.)
                self.q, self.velocity = candidate.copy(), velocity.copy()
                self.engaged = True
            except Exception as exc:
                self.publish_error = str(exc)
                self._fault("command_publish_failed")

    def info(self):
        with self.lock:
            return {"state": self.state, "reason": self.reason, "mode": self.mode,
                "position_scale": self.scale, "running": self.running,
                "override_release_pending": self.release_pending,
                "input_error": self.input_error, "publish_error": self.publish_error,
                "source_sequence": self.sequence, "rejected_inputs": self.rejected,
                "input_age_ms": None if self.frame is None else (self.clock()-self.received_ns)/1e6,
                "mapping_epoch": self.mapping_epoch, "position_error_m": self.residual,
                "reference_positions": None if self.q is None else self.q.tolist(),
                "reference_velocity_rad_s": self.velocity.tolist(),
                "capabilities": ["dual_arm", "position_priority_5dof"],
                "topic_in": [{"port_id": "command", "format": "data/teleop-cmd", "topic": COMMAND_TOPIC}],
                "topic_out": [{"port_id": "state", "format": "data/teleop-state", "topic": STATE_TOPIC}]}

    def feedback(self):
        with self.lock:
            self.monitor_sequence += 1
            label = {"idle": "未启动", "ready": "等待握把", "hold": "保持",
                     "limited": "可达边界", "following": "跟随", "error": "错误"}[self.state]
            mode = "Shadow（不驱动本体）" if self.mode == "shadow" else "本体执行"
            residual = "-" if self.residual is None else f"{max(self.residual):.3f} m"
            return {"schema": FEEDBACK_SCHEMA, "instance_id": self.source or "teleop_device",
                "text": html.escape(f"T800 遥操：{label} | {mode} | 输入帧：{self.sequence} | "
                    f"位置残差：{residual} | 原因：{self.reason or self.input_error or '无'}"),
                "control_instance_id": self.instance_id, "server_epoch": self.server_epoch,
                "clock_id": self.clock_id, "sequence": self.monitor_sequence,
                "emitted_monotonic_ns": self.clock(), "source_sequence": self.sequence,
                "connection_epoch": self.identity[1] if self.identity else 0,
                "space_epoch": self.identity[2] if self.identity else 0,
                "operator_session_id": str(self.started_ns) if self.running else None,
                "mapping_epoch": self.mapping_epoch, "state": self.state, "reason": self.reason,
                "capabilities": ["dual_arm", "position_priority_5dof"],
                "execution": copy.deepcopy(self.info()), "receipts": []}
