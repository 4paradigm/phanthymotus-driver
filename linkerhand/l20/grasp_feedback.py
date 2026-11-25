"""Position and tactile feedback for a bounded whole-hand grasp."""
from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median

from can_client import FINGER_NAMES


@dataclass(frozen=True)
class GraspSettings:
    output_limit: int
    speed: int
    contact_peak: int
    peak_ceiling: int
    approach_travel: int
    max_corrections: int

    @property
    def hard_stop_peak_raw(self):
        return min(255, math.ceil(1.5 * self.peak_ceiling))

    @property
    def travel_by_finger(self):
        extra = round(self.approach_travel * 0.20)
        return {f: min(255, self.approach_travel + extra) if f in ("ring", "pinky")
                else self.approach_travel for f in FINGER_NAMES}


# (minimum, maximum, default, description) in the device's raw units.
SETTING_FIELDS = {
    "output_limit": (1, 255, 40, "本次动作的电机输出限值，原始档位"),
    "speed": (1, 255, 35, "本次动作的电机速度，原始档位"),
    "contact_peak": (1, 254, 3, "确认接触的指尖阵列峰值"),
    "peak_ceiling": (2, 255, 18, "单指停止加紧的峰值；整手过压截停阈值为设定值的 1.5 倍向上取整，触觉满量程也会截停"),
    "approach_travel": (1, 255, 110, "各屈伸关节从准备位置起的最大收拢行程"),
    "max_corrections": (0, 10, 0, "自动保持时最多补夹几次，0 表示只监测"),
}

# The gentle Skill can interpolate within these task-specific upper bounds. Other
# callers retain the full parameter range of the generic prepare action.
GENTLE_MAX = dict(output_limit=85, speed=40, contact_peak=5, peak_ceiling=150,
                  approach_travel=190, max_corrections=3)
GENTLE_DEFAULT_TRAVEL = 150
SEARCH_WATCHDOG_S = 120
NO_PROGRESS_S = 5
CONTACT_WINDOW_FRAMES = 3
MAX_CONTACT_FRAME_GAP_S = 1.0


def parameter_limits(gentle):
    return {
        name: {"minimum": low, "maximum": GENTLE_MAX[name] if gentle else high}
        for name, (low, high, _, _) in SETTING_FIELDS.items()
    }


def settings_from(args):
    gentle = args.get("gentle_limits", False)
    if type(gentle) is not bool:
        raise ValueError("gentle_limits 必须是布尔值")
    values = {}
    for name, bounds in parameter_limits(gentle).items():
        low, high = bounds["minimum"], bounds["maximum"]
        default = SETTING_FIELDS[name][2]
        if gentle and name == 'approach_travel':
            default = GENTLE_DEFAULT_TRAVEL
        value = args.get(name, default)
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"{name} 必须是 {low} 到 {high} 之间的整数")
        values[name] = value
    if values["contact_peak"] >= values["peak_ceiling"]:
        raise ValueError("contact_peak 必须小于 peak_ceiling")
    return GraspSettings(**values)


@dataclass(frozen=True)
class Contact:
    peak: int
    total: int
    row: float
    col: float
    saturated: bool = False


def empty_baseline(matrices):
    return {
        finger: [median(values) for values in zip(
            *[[v for row in frame[finger] for v in row] for frame in matrices])]
        for finger in FINGER_NAMES
    }


def contact_patches(matrix, baseline):
    patches = {}
    for finger in FINGER_NAMES:
        raw = [v for row in matrix[finger] for v in row]
        values = [max(0, int(v - zero)) for v, zero in zip(raw, baseline[finger])]
        total = sum(values)
        row = sum((i // 6) * v for i, v in enumerate(values)) / total if total else 0.0
        col = sum((i % 6) * v for i, v in enumerate(values)) / total if total else 0.0
        patches[finger] = Contact(max(values), total, row, col, saturated=255 in raw)
    return patches


def _aligned_thumb_tip_travel(root_travel):
    """Right thumb raw-position curve fitted in the G20/box approach replay."""
    if root_travel <= 115:
        return round(28 * root_travel / 115)
    return min(60, 28 + round(32 * (root_travel - 115) / 35))


class FeedbackGrasp:
    """One decision per complete new tactile and position sample."""

    def __init__(self, profile, start_pose, close_limit, *, finger_tip_travel_percent=100, align_thumb=False):
        self.profile = profile
        self.target = list(start_pose)
        self.initial = list(start_pose)
        self.lower = list(start_pose)
        self.finger_tip_travel_percent = finger_tip_travel_percent
        self.tip_remainder = [0] * len(FINGER_NAMES)
        travel = profile.travel_by_finger
        for i, finger in enumerate(FINGER_NAMES):
            for j in (i, i + 15):
                available = max(0, min(travel[finger], start_pose[j] - close_limit[j]))
                if i > 0 and j == i + 15:
                    available = available * finger_tip_travel_percent // 100
                self.lower[j] = start_pose[j] - available
        self.align_thumb = align_thumb
        self.thumb_rotation_locked = False
        self.thumb_middle_contact_seen = False
        if align_thumb:
            tip_travel = _aligned_thumb_tip_travel(self.initial[0] - self.lower[0])
            self.lower[15] = max(self.lower[15], self.initial[15] - tip_travel)
        self.latched = set()
        self.travel_limited = set()
        self.anchors = {}
        self.seated = set()
        self.clear_frames = dict.fromkeys(FINGER_NAMES, 0)
        self.seek_contact = True
        self.search_stop = None
        self.search_start = None
        self.last_progress = None
        self.frontier = list(start_pose)
        self.at_limit_frames = dict.fromkeys(FINGER_NAMES, 0)
        self.peak_limited_fingers = ()
        self.saturated_fingers = ()
        self.relief_count = 0
        self.contact_history = []
        self.best_contact_windows = {}
        self.waiting_for_signal = False
        self.stable_fingers = set()
        self.slip_frames = {finger: 0 for finger in FINGER_NAMES}
        self.loss_frames = 0
        self.corrections = 0
        self.last_correction = -math.inf
        self.reanchor = False
        self.paused = ""
        self.overpressure_fingers = ()
        self.opposed = False
        self.last_evidence = ()

    def _align_thumb_tip(self, actual, step):
        # Measured root motion paces the tip even when the thumb root stalls.
        travel = max(0, self.initial[0] - max(self.target[0], actual[0] - step))
        if actual[15] - self.target[15] <= 5:
            tip = max(self.lower[15], self.initial[15] - _aligned_thumb_tip_travel(travel))
            self.target[15] -= min(step, max(0, self.target[15] - tip))
        if not self.thumb_rotation_locked:
            roll = min(255, self.initial[10] + round(8 * min(travel, 100) / 100))
            if self.target[10] - actual[10] <= 2:
                self.target[10] += min(1, max(0, roll - self.target[10]))

    def _close(self, fingers, actual, step, approach=False):
        previous = self.target.copy()
        for finger in fingers:
            i = FINGER_NAMES.index(finger)
            for j in (i, 15 + i):
                if approach and self.align_thumb and j == 15:
                    self._align_thumb_tip(actual, step)
                    continue
                # Never advance further when the measured joint lags the command.
                if actual[j] - self.target[j] <= 5:
                    advance = 1 if approach and finger in ('ring', 'pinky') else step
                    if approach and i > 0 and j == i + 15 and self.finger_tip_travel_percent < 100:
                        # Carry fractions across eligible samples, including the outer digits' one-unit steps.
                        advance, self.tip_remainder[i] = divmod(
                            self.tip_remainder[i] + advance * self.finger_tip_travel_percent, 100
                        )
                    self.target[j] = max(self.lower[j], self.target[j] - advance)
        return previous != self.target

    @property
    def remaining_fingers(self):
        return [f for f in FINGER_NAMES if f not in self.latched and f not in self.travel_limited]

    def _search_limit(self, actual, now):
        if self.search_start is None:
            self.search_start = self.last_progress = now
        for i, finger in enumerate(FINGER_NAMES):
            if finger in self.latched or finger in self.travel_limited:
                continue
            joints = (i, i + 15)
            for j in joints:
                if actual[j] <= self.frontier[j] - 2:
                    self.frontier[j] = actual[j]
                    self.last_progress = now
            at_limit = all(self.target[j] == self.lower[j] and actual[j] <= self.lower[j] + 2
                           for j in joints)
            self.at_limit_frames[finger] = self.at_limit_frames[finger] + 1 if at_limit else 0
            if self.at_limit_frames[finger] >= 3:
                self.travel_limited.add(finger)
        if not self.remaining_fingers:
            return "fingers_settled" if self.latched else "travel_limit"
        if now - self.last_progress >= NO_PROGRESS_S:
            return "no_progress"
        if now - self.search_start >= SEARCH_WATCHDOG_S:
            return "watchdog"
        return None

    def _seat(self, fingers, patches, actual):
        for f in fingers - self.seated:
            relief = int(patches[f].peak >= 0.65 * self.profile.peak_ceiling)
            self._cancel_inward_error(actual, [f], relief)
            self.relief_count += relief
        self.seated.update(fingers)

    def _cancel_inward_error(self, actual, fingers=FINGER_NAMES, relief=0):
        for finger in fingers:
            i = FINGER_NAMES.index(finger)
            for j in (i, i + 15):
                self.target[j] = min(self.initial[j], max(actual[j], self.target[j]) + relief)

    def _approach(self, contact, patches, actual, now):
        self._seat(contact | self.latched, patches, actual)
        for f in self.seated - self.latched:
            self.clear_frames[f] = 0 if f in contact else self.clear_frames[f] + 1
            if self.clear_frames[f] >= 2:
                self.seated.remove(f)
                self.clear_frames[f] = 0
        self.search_stop = self._search_limit(actual, now)
        if self.search_stop:
            self.seek_contact = False
            if self.search_stop == "fingers_settled" and not self.stable_fingers:
                self.waiting_for_signal = True
            self._cancel_inward_error(actual)
        else:
            moving = [f for f in self.remaining_fingers if f not in self.seated]
            self._close(moving, actual, 2, approach=True)

    def update(self, patches, actual, now, monitoring_hold):
        if self.paused:
            return False
        previous = self.target.copy()
        hard_stop = self.profile.hard_stop_peak_raw
        self.saturated_fingers = tuple(f for f, p in patches.items() if p.saturated)
        limited = tuple(f for f, p in patches.items() if p.saturated or p.peak >= self.profile.peak_ceiling)
        entering_limit = set(limited).difference(self.peak_limited_fingers)
        self.peak_limited_fingers = limited
        self.overpressure_fingers = tuple(f for f in limited if patches[f].saturated or patches[f].peak >= hard_stop)
        # A complete stop frame is still evidence. Observe it exactly once, then
        # take the protection branch without sending any further inward step.
        contact = self._observe_contact(patches, now, self.seek_contact and not monitoring_hold)
        if self.overpressure_fingers:
            self.contact_history.clear()
            self.stable_fingers.clear()
            self.opposed = False
            self._cancel_inward_error(actual)
            self._cancel_inward_error(actual, self.overpressure_fingers, relief=2)
            readings = "、".join(f"{f}={patches[f].peak}" for f in self.overpressure_fingers)
            reason = "阵列满量程" if self.saturated_fingers else "达到整手截停阈值"
            self.paused = (f"{reason}（{readings}，本轮设定 {self.profile.peak_ceiling}，"
                           f"整手截停值 {hard_stop}），请托住物品")
        else:
            # Cancel any pending inward command on just the newly limited digits,
            # including an in-flight slip correction. Other digits still wrap.
            self._cancel_inward_error(actual, entering_limit)
            self._update_contact(contact, patches, actual, now, monitoring_hold)
        if self.align_thumb and not self.thumb_rotation_locked and (
            "thumb" in contact or "thumb" in self.latched or self.overpressure_fingers or
            not self.seek_contact or self.paused or monitoring_hold
        ):
            self.thumb_rotation_locked = True
            self.target[10] = actual[10]  # Cancel rotational tracking error on the first contact.
        return self.target != previous

    def _observe_contact(self, patches, now, record):
        threshold = self.profile.contact_peak
        peaks = {f: p.peak for f, p in patches.items()}
        contact = {f for f, peak in peaks.items() if peak >= threshold}
        if self.contact_history and now - self.contact_history[-1][0] > MAX_CONTACT_FRAME_GAP_S:
            self.contact_history.clear()
        self.contact_history.append((now, peaks))
        self.contact_history = self.contact_history[-CONTACT_WINDOW_FRAMES:]
        self.stable_fingers = set()
        if len(self.contact_history) == CONTACT_WINDOW_FRAMES:
            for f in FINGER_NAMES:
                values = [frame[f] for _, frame in self.contact_history]
                if min(values) >= threshold:
                    self.stable_fingers.add(f)  # Live contact for slip monitoring.
                if record and sum(values) >= CONTACT_WINDOW_FRAMES * threshold:
                    self.latched.add(f)
                    mean = round(sum(values) / CONTACT_WINDOW_FRAMES, 2)
                    if f not in self.best_contact_windows or mean > self.best_contact_windows[f]["mean_peak_raw"]:
                        self.best_contact_windows[f] = {
                            "mean_peak_raw": mean, "frames": CONTACT_WINDOW_FRAMES,
                            "peak_samples_raw": values,
                            "span_ms": round(1000 * (now - self.contact_history[0][0])),
                        }
        if record and {"thumb", "middle"} <= self.stable_fingers:
            self.thumb_middle_contact_seen = True
        self.opposed = "thumb" in contact and len(contact) >= 2
        return contact

    def _update_contact(self, contact, patches, actual, now, monitoring_hold):
        if self.waiting_for_signal:
            # Keep the bounded pose until a fresh slip reference is available.
            if not (monitoring_hold and self.stable_fingers):
                return
            self.waiting_for_signal = False
            self.anchors.clear()
            self.loss_frames = 0
            self.reanchor = False
        lost_during_wrap = not monitoring_hold and self.best_contact_windows and not contact
        if (self.anchors or lost_during_wrap) and not self.stable_fingers:
            self.loss_frames += 1
            self.slip_frames = dict.fromkeys(FINGER_NAMES, 0)
            if self.loss_frames == 1:
                self._cancel_inward_error(actual)
            empty_pair = self.loss_frames >= 2 and all(max(frame.values()) < self.profile.contact_peak
                                                      for _, frame in self.contact_history[-2:])
            if self.loss_frames >= 3 or empty_pair:
                if self.best_contact_windows and self.search_stop in (None, "fingers_settled", "contact_faded"):
                    self.waiting_for_signal = True
                    if self.seek_contact:
                        self.seek_contact = False
                        self.search_stop = "contact_faded"
                else:
                    self.paused = "所有指尖都已失去持续接触，请托住物品并重新摆放"
            return
        if not self.anchors and contact:
            self.loss_frames = 0
        if self.stable_fingers:
            # Keep baselines on the same uninterrupted patch only.
            # Continuing the wrap can shift pressure on a stationary finger.
            # Use the last last wrapping sample as its baseline for slip monitoring.
            previous_refs = self.anchors if monitoring_hold and not self.loss_frames else {}
            self.anchors = {f: previous_refs.get(f, patches[f]) for f in self.stable_fingers}
            for f in self.slip_frames:
                if f not in self.stable_fingers:
                    self.slip_frames[f] = 0
            self.loss_frames = 0
        if not monitoring_hold:
            if self.seek_contact:
                self._approach(contact, patches, actual, now)
            return
        if not self.stable_fingers or self.profile.max_corrections == 0 or now - self.last_correction < 0.45:
            return
        if self.reanchor:
            if all(f in self.stable_fingers for f in self.last_evidence):
                self.anchors = {f: patches[f] for f in self.stable_fingers}
                self.reanchor = False
            elif now - self.last_correction > 1.5:
                self.paused = "补夹指的持续接触未恢复，请托住物品"
            return

        slipping = []
        for f, ref in self.anchors.items():
            p = patches[f]
            drift = math.hypot(p.row - ref.row, p.col - ref.col)
            suspect = (f not in self.peak_limited_fingers and f in contact
                       and p.total <= 0.80 * ref.total and drift >= 0.45)
            self.slip_frames[f] = self.slip_frames[f] + 1 if suspect else 0
            if self.slip_frames[f] >= 2:
                slipping.append(f)
        if not slipping:
            return
        self.last_evidence = tuple(slipping)
        if self.corrections >= self.profile.max_corrections:
            self.paused = "补夹次数已达上限，仍有滑动迹象；请托住物品"
            return
        if not self._close(slipping, actual, 1):
            self.paused = "关节已到行程或跟踪边界；请托住物品"
            return
        self.corrections += 1
        self.last_correction = now
        self.reanchor = True
        self.slip_frames = {finger: 0 for finger in FINGER_NAMES}
