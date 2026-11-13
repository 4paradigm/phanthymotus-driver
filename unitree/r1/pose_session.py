"""Time-based practice checks. Caller serializes access and supplies fresh frames."""
from __future__ import annotations

import math
import time
from collections import Counter, deque
from uuid import uuid4

from pose_check import POSES, check_pose, compare_squat_height, squat_height_metrics


class PoseSession:
    # MediaPipe on the ARM device can occasionally take a little longer than
    # one frame. Do not throw away a calibrated squat baseline for one slow
    # inference cycle.
    MAX_GAP = 1.0
    SETTLE_SECONDS = 0.4
    PHASE_WINDOW = 5
    # Two consistent down observations within a short interval catch a moving
    # squat, whose lowest point is often visible for only 2-4 frames.
    DOWN_REQUIRED = 2
    DOWN_WINDOW_SECONDS = 0.45
    POSITION_SHIFT_SECONDS = 0.75
    INVALID_GRACE_SECONDS = 1.5
    RETURN_HIP_TOLERANCE = 0.16
    RETURN_KNEE_MIN_ANGLE = 150.0
    RELEASE_SECONDS = 0.25

    def __init__(self, pose, hold_seconds=3, repetitions=1, timeout_seconds=45, clock=time.monotonic):
        if pose not in POSES:
            raise ValueError("unsupported_pose")
        values = (hold_seconds, repetitions, timeout_seconds)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in values):
            raise ValueError("session parameters must be finite numbers")
        if not 0 <= hold_seconds <= 30 or not 1 <= repetitions <= 20 or int(repetitions) != repetitions:
            raise ValueError("hold_seconds: 0..30; repetitions: integer 1..20")
        if not 1 <= timeout_seconds <= 300 or (pose != "squat" and timeout_seconds <= hold_seconds):
            raise ValueError("timeout_seconds: 1..300 and longer than the hold")
        self.pose, self.hold_seconds = pose, hold_seconds
        self.target = int(repetitions)
        self.clock, self.started = clock, clock()
        # Squat timing starts only after a stable standing calibration.  The
        # upper-body poses can start their hold timeout immediately.
        self.calibrated_at = None if pose == "squat" else self.started
        self.workout_started_at = None if pose == "squat" else self.started
        self.deadline = (None if pose == "squat"
                         else self.started + timeout_seconds)
        self.timeout_seconds = timeout_seconds
        self.session_id = uuid4().hex
        self.state = "running"
        self.last_frame = None
        self.last_at = None
        self.run_since = None
        self.candidate = None
        self.stage = "standing_required" if pose == "squat" else "holding"
        self.baseline = None
        self.height_baseline = None
        self.count = 0
        self.held = 0.0
        self.rep_started_at = None
        self.last_round_seconds = None
        self.phase_history = deque(maxlen=self.PHASE_WINDOW)
        self.down_observations = deque(maxlen=self.PHASE_WINDOW)
        self.position_shift_since = None
        self.invalid_since = None
        self.last = {"detected": False, "matched": False, "status": "retry",
                     "feedback": "等待新画面", "reason": "waiting_for_frame"}

    def _reset_continuity(self):
        self.run_since = None
        self.candidate = None
        self.held = 0.0
        self.rep_started_at = None
        self.phase_history.clear()
        self.down_observations.clear()
        self.position_shift_since = None
        self.invalid_since = None
        if self.pose == "squat":
            self.stage, self.baseline = "standing_required", None
            self.height_baseline = None
            # Losing the current standing baseline must not restart the
            # workout clock or let a session avoid its timeout indefinitely.
            self.calibrated_at = None

    def _stable(self, label, now, seconds):
        if self.candidate != label:
            self.candidate, self.run_since = label, now
        return now - self.run_since >= seconds

    def update(self, keypoints, frame_id, *, stable_match=None):
        """Update one frame.

        ``stable_match`` comes from PoseCard's 3-of-5 gate for upper-body
        sessions. Direct callers can omit it and use this frame's geometry.
        """
        now = self.clock()
        self.status()
        if self.state != "running" or frame_id == self.last_frame:
            return self.status()
        if self.last_at is not None and now - self.last_at > self.MAX_GAP:
            self._reset_continuity()
        self.last_frame, self.last_at = frame_id, now
        result = check_pose(keypoints, self.pose)
        if (self.pose == "squat" and self.height_baseline is not None and
                not result.get("detected")):
            height_points = squat_height_metrics(keypoints)
            if height_points is not None:
                result = {**result, **height_points,
                          "detected": True, "matched": False,
                          "status": "almost", "phase": "height_only",
                          "knee_angles": [], "leg_length": self.baseline[2],
                          "leg_lengths": None, "shin_lengths": None,
                          "reason": "knee_unreliable_height_available"}
        self.last = dict(result)
        if not result.get("detected") or result.get("error"):
            if self.invalid_since is None:
                self.invalid_since = now
            # Low-visibility landmarks are common for a frame or two on the
            # robot camera. Preserve an established squat baseline and phase
            # during a short recovery window instead of falling back to
            # standing_required and emitting another "start" instruction.
            invalid_age = now - self.invalid_since
            if self.pose == "squat" and invalid_age <= self.INVALID_GRACE_SECONDS:
                if self.calibrated_at is not None:
                    # Pause the current phase during a short occlusion. A
                    # confirmed down phase may still finish after visibility
                    # returns, but missing frames never advance the timer.
                    self.candidate = None
                    self.position_shift_since = None
                    # Keep recent down evidence in the ready stage so one
                    # dropped frame does not split a moving squat in two.
                    # The 0.45 s window still bounds how far apart the two
                    # valid observations may be.
                    if self.stage != "ready":
                        self.down_observations.clear()
                self.last.update(
                    status="almost",
                    matched=False,
                    feedback="关键点暂时不稳定，请保持机位",
                    reason="transient_keypoint_loss",
                )
                return self.status()
            self._reset_continuity()
            self.last["status"] = "retry"
            return self.status()
        self.invalid_since = None
        if self.pose != "squat":
            matched = bool(result["matched"] if stable_match is None else stable_match)
            if self.stage == "release_required":
                self.held = 0.0
                if matched:
                    self.run_since, self.candidate = None, None
                    self.last.update(status="almost", feedback="请先放下动作，再做下一次")
                elif self._stable("released", now, self.RELEASE_SECONDS):
                    self.stage = "holding"
                    self.run_since, self.candidate = None, None
                    self.last.update(status="almost", feedback="已放下动作，可以做下一次")
                else:
                    self.last.update(status="almost", feedback="请放下动作，准备下一次")
            elif not matched:
                self.run_since, self.candidate, self.held = None, None, 0.0
                self.last.update(status="retry", feedback="请完成目标动作")
            else:
                # A one-hand hold must stay on the SAME anatomical side.
                label = result.get("raised_side", self.pose)
                finished = self._stable(label, now, self.hold_seconds)
                self.held = min(self.hold_seconds, now - self.run_since)
                self.last.update(status="almost", feedback="动作正确，请继续保持")
                if finished:
                    self.count += 1
                    self.last_round_seconds = round(max(0.0, now - self.run_since), 2)
                    self.held = 0.0
                    self.run_since, self.candidate = None, None
                    if self.count >= self.target:
                        self.state = "completed"
                    else:
                        self.stage = "release_required"
                        self.last.update(status="almost",
                                         feedback=f"已完成 {self.count} 次，请放下动作后继续")
        else:
            self._squat(result, now)
        if self.state == "completed":
            self.last.update(
                status="completed",
                feedback=(f"深蹲完成，共 {self.count} 次"
                          if self.pose == "squat" else "已完成"),
            )
        return self.status()

    def _squat(self, result, now):
        self.phase_history.append(result["phase"])
        # The phase itself remains responsive; the history is exposed for
        # diagnostics and transient keypoint loss is tolerated separately.
        phase = result["phase"]
        self.last.update(status="almost", feedback="请先站直，保持片刻")
        if self.stage == "standing_required":
            if phase == "standing":
                if self._stable("standing", now, self.SETTLE_SECONDS):
                    self.baseline = (result["hip_height"], result["ankle_height"],
                                     result["leg_length"], result.get("leg_lengths"),
                                     result.get("shin_lengths"), result.get("hip_heights"))
                    self.height_baseline = dict(result)
                    self.stage, self.candidate = "ready", None
                    self.down_observations.clear()
                    self.calibrated_at = now
                    if self.workout_started_at is None:
                        self.workout_started_at = now
                        self.deadline = now + self.timeout_seconds
                    self.last["feedback"] = "站直校准完成，开始深蹲"
            else:
                self.candidate = None
            return
        hip, ankle, length, baseline_legs, baseline_shins, baseline_hips = self.baseline
        height = compare_squat_height(result, self.height_baseline)
        self.last.update(height)
        # Ankle landmarks can drift while the person's hips stay in place.
        # Only a sustained, same-direction shift of both hips and ankles
        # indicates that the whole person/camera moved enough to recalibrate.
        hip_shift = result["hip_height"] - hip
        ankle_shift = result["ankle_height"] - ankle
        position_shifted = (abs(hip_shift) > 0.12 * length and
                            abs(ankle_shift) > 0.12 * length and
                            hip_shift * ankle_shift > 0)
        if self.stage == "ready" and phase == "standing" and position_shifted:
            if self.position_shift_since is None:
                self.position_shift_since = now
            elif now - self.position_shift_since >= self.POSITION_SHIFT_SECONDS:
                self._reset_continuity()
                self.last.update(status="retry", feedback="机位或站位变化，请重新站直")
                return
        else:
            self.position_shift_since = None
        if self.stage == "ready":
            self.last["feedback"] = "请缓慢浅蹲"
            hip_drop = result["hip_height"] - hip
            knee_angles = result.get("knee_angles", [])
            leg_lengths = result.get("leg_lengths")
            shin_lengths = result.get("shin_lengths")
            hip_heights = result.get("hip_heights")
            # A front-facing camera can project a crouched leg almost on top
            # of itself while the 2D knee angle remains near standing or
            # becomes implausibly small. Use coordinated leg contraction only
            # when *both* hips descend and the ankles remain near their
            # standing position. A lone bad knee/ankle landmark cannot count.
            bilateral_contraction = (
                isinstance(leg_lengths, (list, tuple)) and len(leg_lengths) == 2 and
                isinstance(baseline_legs, (list, tuple)) and len(baseline_legs) == 2 and
                all(base > 0 and current <= 0.78 * base
                    for current, base in zip(leg_lengths, baseline_legs))
            )
            # Older recorded results lack per-leg distances; their two
            # implausible knee angles give a conservative replay fallback.
            if leg_lengths is None and baseline_legs is None:
                bilateral_contraction = len(knee_angles) == 2 and max(knee_angles) < 65
            # A shin collapsing almost to zero usually means the ankle and
            # knee landmarks have merged; do not infer a rep from that frame.
            shin_reliable = (
                shin_lengths is None or baseline_shins is None or
                (len(shin_lengths) == 2 and len(baseline_shins) == 2 and
                 all(base > 0 and current >= 0.15 * base
                     for current, base in zip(shin_lengths, baseline_shins)))
            )
            both_hips_descend = (
                hip_heights is None or baseline_hips is None or
                (len(hip_heights) == 2 and len(baseline_hips) == 2 and
                 all(current - base >= 0.12 * length
                     for current, base in zip(hip_heights, baseline_hips)))
            )
            projected_down = (
                bilateral_contraction and shin_reliable and both_hips_descend and
                result["leg_length"] <= 0.7 * length and
                hip_drop >= max(0.06, 0.25 * length) and
                abs(ankle_shift) <= 0.35 * length
            )
            angle_down = phase == "down" and hip_drop >= 0.08 * length
            if angle_down or projected_down or height["height_matched"]:
                self.last["squat_evidence"] = (
                    "knee_angle" if angle_down else
                    "projected_leg" if projected_down else "body_height"
                )
                self.down_observations.append(now)
                while (self.down_observations and
                       now - self.down_observations[0] > self.DOWN_WINDOW_SECONDS):
                    self.down_observations.popleft()
                if len(self.down_observations) >= self.DOWN_REQUIRED:
                    self.stage, self.candidate = "returning", None
                    self.rep_started_at = now
                    self.down_observations.clear()
                    self.last["feedback"] = "已检测到下蹲，请站起来"
            else:
                while (self.down_observations and
                       now - self.down_observations[0] > self.DOWN_WINDOW_SECONDS):
                    self.down_observations.popleft()
        else:
            self.last["feedback"] = "请站起来，完成这一次"
            # A practical office squat may finish with a small knee bend. Once
            # the user has gone through the down phase, accept a near-standing
            # return with a slightly wider hip tolerance instead of requiring
            # an anatomically perfect 180-degree lockout.
            knee_angles = result.get("knee_angles", [])
            near_standing = (phase == "standing" or
                             (phase == "transition" and knee_angles and
                              min(knee_angles) >= self.RETURN_KNEE_MIN_ANGLE))
            if ((near_standing or height["height_recovered"]) and
                    abs(result["hip_height"] - hip) <= self.RETURN_HIP_TOLERANCE * length):
                if self._stable("returned", now, self.SETTLE_SECONDS):
                    self.count += 1
                    self.last_round_seconds = (
                        round(max(0.0, now - self.rep_started_at), 2)
                        if self.rep_started_at is not None else None
                    )
                    self.rep_started_at = None
                    self.stage, self.candidate = "ready", None
                    self.down_observations.clear()
                    self.last["feedback"] = f"已完成 {self.count} 次，请继续"
                    if self.count >= self.target:
                        self.state = "completed"
            else:
                self.candidate = None

    def cancel(self):
        if self.state != "cancelled":
            self.state = "cancelled"
            self.last.update(status="retry", matched=False, feedback="本次跟练已停止")
        return self.status()

    def status(self):
        now = self.clock()
        if self.state == "running":
            if self.deadline is not None and now >= self.deadline:
                self.state = "timed_out"
                self._reset_continuity()
                self.last.update(status="retry", matched=False, feedback="本次跟练超时，可重试或跳过")
            elif self.last_at is not None and now - self.last_at > self.MAX_GAP:
                self._reset_continuity()
                self.last.update(status="retry", matched=False, detected=False,
                                 feedback="等待新画面", reason="camera_stale")
        total_elapsed = max(0.0, now - self.started)
        # For squats this is deliberately zero until calibration finishes, so
        # the user sees the workout clock start at the first valid standing
        # baseline rather than while being told how to stand.
        elapsed = (max(0.0, now - self.workout_started_at)
                   if self.workout_started_at is not None else 0.0)
        phase_elapsed = (max(0.0, now - self.run_since)
                         if self.run_since is not None else 0.0)
        is_squat = self.pose == "squat"
        return {**self.last, "pose": self.pose, "session_id": self.session_id,
                "state": self.state, "progress": {
                    "mode": "repetitions",
                    "hold_seconds": 0.0 if is_squat else round(self.held, 2),
                    "target_seconds": 0.0 if is_squat else self.hold_seconds,
                    "elapsed_seconds": round(elapsed, 2),
                    "session_elapsed_seconds": round(total_elapsed, 2),
                    "calibrated": (self.calibrated_at is not None),
                    "phase_elapsed_seconds": round(phase_elapsed, 2),
                    "last_round_seconds": self.last_round_seconds,
                    "phase_votes": dict(Counter(self.phase_history)),
                    "repetitions": self.count, "target_repetitions": self.target,
                    "phase": self.stage,
                    "remaining_seconds": (round(max(0, self.deadline-now), 1)
                                          if self.deadline is not None else None)}}
