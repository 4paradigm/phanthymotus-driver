"""Continuous hand-shape following for the LinkerHand L20 Lite.

The existing ``hand`` card is a call-shaped actuator: one MCP request, one ACP
completion, a halfway pause and a poll to the target. That is the right shape
for an LLM deciding to close a hand, and the wrong shape for a vision model
emitting a new hand shape several times a second.

This card is the stream-shaped path. It subscribes to ``control/hand_flex`` from the
ActuCore hand-vision card, maps five normalized finger curls onto the L20 Lite's
*own calibrated* open/fist poses, limits the step per tick, and writes raw
0..255 positions over the same Modbus connection. It is deliberately a driver
card: the calibration and the raw protocol values are specific to this hand.

Safety choices, all of which are checked before the first write:

* ``start`` (the canvas project-start path) subscribes and stays **disarmed**:
  no torque is applied and no sample is acted on. ``resume`` carries the
  explicit ``confirm_motion: true`` and is the only thing that energizes the
  hand. The canvas hides ``start``/``stop``/``info``/``config`` behind project
  buttons and never sends a ``confirm_motion`` argument on project start, so
  putting the gate on ``start`` made the card permanently unwirable. ``resume``
  is a real, visible button on the card, which is where a human confirmation
  belongs. A stream cannot ask once per frame, so this is the single gate.
* The bound side must be detected. If auto-detection finds zero or two hands,
  the card refuses rather than guessing.
* No data for ``watchdog_ms`` releases torque and requires a fresh ``start``.
  Re-energizing automatically after a silence would be a surprise motion.
* Every received sample carries the upstream observation timestamp. A sample
  older than ``max_obs_age_ms`` is dropped, not followed.
* Every position step is clamped to ``max_step`` raw units per command.
* ``stop``, ``pause`` and any write error release torque.

Values written here are protocol positions, not degrees and not newtons.
The card makes no claim about grip force or collision safety.
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor
from common.vendor_runtime import action_schema, tool
from modbus_client import JOINT_COUNT
from modbus_device import (L20LiteConnection, SPEED_RATIOS, _named,
                           confirm_motion_ok)

PREFIX = "hand_visual_follow"

# The shared control contract for the five normalized finger curls. The card
# consumes `values` = (thumb, index, middle, ring, pinky) in 0..1 and maps them
# onto raw 0..255 joint positions after the sink's admission checks. Every
# safety gate that is common to all motus.control/1 servo paths — contract,
# freshness, source arbitration, step clamp, hard limits, watchdog, abort —
# lives in common/control.ControlSink; this card keeps only the vendor mapping,
# motion gating and torque release.
FLEX_DESCRIPTOR = {
    "control_interface": "motus.control/1",
    "mode": "joint_position",
    "dof": 5,
    "joint_names": ["thumb", "index", "middle", "ring", "pinky"],
    "units": {"angle": "normalized"},
    "limits": {
        "lower": [0.0, 0.0, 0.0, 0.0, 0.0],
        "upper": [1.0, 1.0, 1.0, 1.0, 1.0],
        # Curvature changes slowly; a whole-range jump in one 10 Hz step is
        # never legitimate. The sink clamps rather than rejects it.
        "max_delta_per_step": [1.0, 1.0, 1.0, 1.0, 1.0],
    },
    "rate": {
        "max_hz": 20,
        "expected_hz": 10,
        "watchdog_ms": 5000,
        "max_obs_age_ms": 500,
    },
    "force_torque": None,
}
FLEX_SOURCE = f"mcp__driver__{PREFIX}"

# L20 Lite raw joint order (see modbus_device.JOINT_NAMES). Only the five
# flexion joints are driven; sway and thumb roll stay at their calibrated
# open-pose values so the visual stream never invents an uncalibrated motion.
FLEXION_JOINTS = {
    "thumb": 0,      # thumb_root
    "index": 2,      # index_root
    "middle": 3,     # middle_root
    "ring": 4,       # ring_root
    "pinky": 5,      # pinky_root
}
MOTION_ACTIONS = ("resume",)

# The thumb does not oppose the palm by flexing alone: L20 Lite needs its
# side-swing and roll motors too. These are the operator-verified values from
# the OK / pinch gestures (thumb tip touching a finger): root 132, sway 28,
# roll 255; and the full fist: root 93, sway 255, roll 255. For a fist-like
# curl we interpolate toward a palm-touching pose, NOT the plain fist pose
# (which leaves sway at 255 = thumb held out to the side).
# Minimum mean four-finger curl before the thumb starts to oppose. Above
# ~0.35 a single finger cannot reach it; a genuine fist (all four ~1) can.
THUMB_GRASP_ONSET = 0.35

THUMB_OPPOSE_SWAY = 28     # from OK gesture: thumb swung across the palm
THUMB_OPPOSE_ROOT = 132    # from OK gesture: thumb root bend
THUMB_OPPOSE_ROLL = 172    # from number-9 pose: thumb rolled across palm

# L20 Lite thumb joints have a different gear ratio from the four fingers.
# gesture_device already scales these three down so the thumb is not driven
# at finger speed; the follow card must do the same. Factors are taken from
# that card (29.11 / 63.11, 63.21 and 50.90) — not invented here.
THUMB_SPEED_SCALE = {0: 29.11 / 63.11, 1: 29.11 / 63.21, 9: 29.11 / 50.90}

# Highest speed the operator verified on this hardware (firm grasp in
# gesture_device). 255 is the wire limit, not a verified mechanical limit.
MAX_VERIFIED_SPEED = 110


def joint_speeds(base: int) -> list:
    """Per-joint speeds with the thumb gear-ratio protection applied."""
    speeds = [base] * JOINT_COUNT
    for index, factor in THUMB_SPEED_SCALE.items():
        speeds[index] = max(1, round(base * factor))
    return speeds


def _raw_pose(values, name: str) -> list:
    if not isinstance(values, (list, tuple)) or len(values) != JOINT_COUNT:
        raise ValueError(f"{name} must contain {JOINT_COUNT} position values")
    try:
        pose = [int(v) for v in values]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain integers 0..255") from error
    if any(v < 0 or v > 255 for v in pose):
        raise ValueError(f"{name} values must be in 0..255")
    return pose


# Landmark indices (same 21-point topology as the perception card).
_LM_THUMB_TIP = 4
_LM_FINGER_TIPS = (8, 12, 16, 20)   # index, middle, ring, pinky
_LM_PALM = (0, 5, 9, 13, 17)        # wrist + four MCPs


def adaptive_weight(value: float, lo: float | None, hi: float | None,
                    min_span: float = 0.02):
    """Map an observed weight onto 0..1 using the running min/max range.

    The real camera's geometry ratios do not match the mock, so a fixed
    gain/bias either stalls the thumb or over-drives it. Instead we learn the
    range this operator actually produces: the lowest observed weight becomes
    0 (open) and the highest becomes 1 (full opposition). This removes all
    guesswork and always uses the motor's full travel.

    Returns (normalized_weight, new_lo, new_hi).
    """
    w = max(0.0, min(1.0, float(value)))
    lo = w if lo is None else min(float(lo), w)
    hi = w if hi is None else max(float(hi), w)
    span = hi - lo
    if span < min_span:
        # No meaningful range observed yet: stay open, do not drive on noise.
        return 0.0, lo, hi
    return (w - lo) / span, lo, hi


def thumb_opposition_from_landmarks(landmarks, *,
                                    palm_open: float = 2.0,
                                    palm_fist: float = 0.6,
                                    mouth_open: float = 0.8,
                                    mouth_fist: float = 0.1,
                                    pinch_ratio: float = 1.3,
                                    pinch_fist: float = 0.3) -> float:
    """Continuous thumb bend from its two distal joints + the palm plane.

    Follows the hybrid joint-Cartesian idea (Meattini et al., LRA 2021) but
    here the thumb is measured against the PALM rather than a robot IK: the
    thumb has only two driven joints and the 21-point model gives us its IP(3)
    and TIP(4) plus the palm triangle (wrist 0, index MCP 5, pinky MCP 17).

    Three geometry cues, all normalized by palm width (wrist->middle-MCP):
      * TIP distance to palm centre -- falls ~2.06 (open) to ~0.57 (fist),
        the main continuous signal;
      * TIP perpendicular distance to the palm mouth line (index MCP -> pinky
        MCP) -- falls ~0.83 to ~0.09, confirms folding toward the palm;
      * nearest finger tip -- captures a pinch, where the thumb touches a
        finger earlier than the palm (Cartesian contact zone of the paper).

    All are linear-mapped open->0, fist->1; the strongest wins. This gives a
    *graded* bend across the whole range instead of only the two extremes, so
    the motor tracks intermediate thumb positions. Depends on the thumb's own
    geometry only -- the four fingers never contribute.
    """
    pts = landmarks
    try:
        if len(pts) != 21:
            raise ValueError("landmarks must have 21 points")
        p = [[float(v[0]), float(v[1])] for v in pts]
        scale = math.hypot(p[9][0] - p[0][0], p[9][1] - p[0][1])
        if scale < 1e-6:
            raise ValueError("degenerate palm scale")
        cx = sum(p[i][0] for i in _LM_PALM) / len(_LM_PALM)
        cy = sum(p[i][1] for i in _LM_PALM) / len(_LM_PALM)
        ip, tip = p[3], p[4]
        # Cue 1: thumb tip toward palm centre.
        to_palm = math.hypot(tip[0] - cx, tip[1] - cy) / scale
        # Cue 2: perpendicular distance from TIP to the palm-mouth line
        # (index MCP -> pinky MCP); the "plane" the thumb folds toward.
        ax, ay = p[5]
        bx, by = p[17]
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        if vv < 1e-12:
            raise ValueError("degenerate palm mouth")
        cross = abs(vx * (tip[1] - ay) - vy * (tip[0] - ax)) / math.sqrt(vv)
        mouth = cross / scale
        # Cue 3: nearest finger tip (pinch).
        nearest = min(math.hypot(tip[0] - p[i][0], tip[1] - p[i][1])
                      for i in _LM_FINGER_TIPS) / scale
    except (ValueError, TypeError, IndexError, KeyError, ZeroDivisionError):
        return 0.0

    def norm(value, open_v, fist_v):
        return max(0.0, min(1.0, (open_v - value) / max(open_v - fist_v, 1e-6)))

    palm_curl = norm(to_palm, palm_open, palm_fist)
    mouth_curl = norm(mouth, mouth_open, mouth_fist)
    pinch_curl = norm(nearest, pinch_ratio, pinch_fist)
    # Smooth graded signal: palm/mouth give continuous bend across the range;
    # pinch can dominate when the thumb reaches for a finger.
    base = 0.6 * palm_curl + 0.4 * mouth_curl
    return float(max(base, pinch_curl))


def target_from_flexions(flexions: dict, open_pose: list, fist_pose: list,
                         gamma: float = 1.0, gear_flexions: dict | None = None,
                         thumb_root_fist: int | None = None,
                         thumb_override: float | None = None,
                         thumb_travel: float = 1.0,
                         thumb_sway_travel: float | None = None,
                         thumb_roll_travel: float | None = None) -> list:
    """Map five 0..1 curls onto a full raw 10-joint target.

    Interpolating between the operator-calibrated poses keeps this card from
    inventing joint limits. Non-flexion joints are held at the calibrated open
    pose; the visual model has no signal for them.
    """
    if not isinstance(flexions, dict):
        raise ValueError("flexions must be an object")
    target = list(open_pose)
    # thumb_travel in 0..1 shrinks the three thumb joints' travel so the
    # thumb never reaches the pose that collides with the fingers/palm.
    # 1.0 = original full range; 0.0 = no motion. Only the thumb is affected.
    travel = max(0.0, min(1.0, float(thumb_travel)))
    # Sway/roll can be limited independently of root. If not set they follow
    # `travel`, so existing configs keep their behaviour.
    sway_travel = (travel if thumb_sway_travel is None
                   else max(0.0, min(1.0, float(thumb_sway_travel))))
    roll_travel = (travel if thumb_roll_travel is None
                   else max(0.0, min(1.0, float(thumb_roll_travel))))

    gear = gear_flexions if gear_flexions is not None else flexions
    # Thumb is driven ONLY by its OWN vision curl -- fully independent of
    # the four fingers. A four-finger grasp with the thumb held out (thumbs-up)
    # must NOT bend the thumb, so no finger signal may contribute here.
    # ``gear`` is the live (pre-deadband) value: thumb curl changes are small
    # and the per-finger deadband would otherwise freeze it after one step.
    if thumb_override is not None:
        # Landmark-based opposition (Meattini et al.) overrides the unreliable
        # thumb vision curl. Still only the thumb -- four fingers unaffected.
        thumb_weight = max(0.0, min(1.0, float(thumb_override))) ** gamma
    else:
        thumb_curl = float(gear.get("thumb", 0.0))
        thumb_weight = max(0.0, min(1.0, thumb_curl)) ** gamma

    for name, index in FLEXION_JOINTS.items():
        # The thumb's curl changes are small; the per-finger deadband freezes
        # them after the first step, so the thumb root uses the LIVE value.
        # The four fingers keep using the deadbanded value (they jump well).
        value = gear.get(name) if name == "thumb" else flexions.get(name)
        if value is None:
            raise ValueError(f"flexions missing {name}")
        if isinstance(value, bool):
            raise ValueError(f"flexions.{name} must be a number in 0..1")
        try:
            curl = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"flexions.{name} must be a number in 0..1") from error
        if not 0.0 <= curl <= 1.0:
            raise ValueError(f"flexions.{name} must be in 0..1, got {curl}")
        low, high = open_pose[index], fist_pose[index]
        # Only the thumb BEND motor gets an independent fist-end override.
        # Four fingers and the sway/roll gear keep the calibrated values.
        if name == "thumb" and index == 0:
            # Root fist endpoint, pulled back toward open by `travel`.
            fist_end = thumb_root_fist if thumb_root_fist is not None else high
            high = low + travel * (fist_end - low)
        # gamma < 1 pushes mid/low curls toward the fist end, so the
        # proximal (first) finger joint actually bends instead of hovering
        # halfway. Endpoints stay exact: 0 -> open, 1 -> fist.
        weight = thumb_weight if name == "thumb" else curl ** gamma
        target[index] = max(0, min(255, round(low + weight * (high - low))))

    # Thumb opposition: drive sway and roll together with the thumb curl so
    # it sweeps across the palm. Without this the thumb only flexes in place
    # (sway stuck at open=255) and can never touch the palm. Driven solely by
    # the thumb's own curl -> other fingers stay independent.
    # The thumb gear must use LIVE flexion, not the deadband-frozen copy:
    # thumb curl changes are small, so the per-finger deadband would freeze
    # it after the first step and the gear would move once then stop.
    sway_end = open_pose[1] + sway_travel * (THUMB_OPPOSE_SWAY - open_pose[1])
    roll_end = open_pose[9] + roll_travel * (THUMB_OPPOSE_ROLL - open_pose[9])
    target[1] = max(0, min(255, round(
        open_pose[1] + thumb_weight * (sway_end - open_pose[1]))))
    target[9] = max(0, min(255, round(
        open_pose[9] + thumb_weight * (roll_end - open_pose[9]))))
    return target


def normalized_grasp(flexions: dict, fist_reference: float,
                     synergy_weight: float) -> dict:
    """Map human grasp curls onto robot flexions (Li et al., TCSVT 2022).

    ``fist_reference`` is the vision-side flexion value at a verified full
    fist; dividing by it turns "human fully closed" into robot ``fist_pose``
    without an ever-growing magic gain. Four fingers share a power-grasp
    component (synergy); a residual keeps individual fingers responsive while
    suppressing jitter from a single noisy fingertip. Thumb is independent.
    """
    if not 0.05 <= fist_reference <= 1.0:
        raise ValueError("fist_reference must be in 0.05..1.0")
    if not 0.0 <= synergy_weight <= 1.0:
        raise ValueError("synergy_weight must be in 0..1")
    out = {name: min(1.0, max(0.0, float(flexions.get(name, 0.0)) / fist_reference))
           for name in FLEXION_JOINTS}
    # Each finger is driven ONLY by its own vision signal. No synergy, no
    # cross-finger averaging: pointing with one finger must not move the
    # others. Per-finger jitter is handled by the deadband in accept(), not
    # by blending fingers together.
    return out


def clamp_step(current: list, target: list, max_step: int) -> list:
    """Limit how far one joint may move in a single command."""
    return [c + max(-max_step, min(max_step, t - c))
            for c, t in zip(current, target)]


class VisualFollowPlugin:
    PREFIX = PREFIX

    def __init__(self, hand, config: dict, ros2=None):
        self.hand = hand              # shares discovery and the RS-485 bus lock
        self.config = config
        self._ros2 = ros2
        self._lock = threading.RLock()
        self._bindings: dict = {}
        self._connections: dict = {}
        self._streams: dict = {}      # instance_id -> stream state
        self._node = None
        self._subscribed_topic = ""
        self._last_error = ""
        self._rejects: list = []

        self.speed = self._setting(config.get("speed", MAX_VERIFIED_SPEED), "speed")
        if self.speed > MAX_VERIFIED_SPEED:
            raise ValueError(
                f"hand_visual_follow.speed {self.speed} exceeds the "
                f"operator-verified maximum {MAX_VERIFIED_SPEED}; higher "
                "values are not known safe for this thumb gearbox")
        # speed_ratio uses the official multiplier (1.0..3.0x) applied on
        # top of the operator-verified base speed, instead of pushing the
        # raw speed past what was validated on this hardware.
        self.speed_ratio = float(config.get("speed_ratio", 1.5))
        if self.speed_ratio not in (1.0, 1.5, 2.0, 2.5, 3.0):
            raise ValueError("speed_ratio must be 1.0/1.5/2.0/2.5/3.0")
        self.torque = self._setting(config.get("torque", 100), "torque")
        self.max_hz = float(config.get("max_hz", 10))
        if not 0 < self.max_hz <= 20:
            raise ValueError("hand_visual_follow.max_hz must be in (0, 20]")
        self.max_step = int(config.get("max_step", 8))
        if not 1 <= self.max_step <= 255:
            raise ValueError("hand_visual_follow.max_step must be in 1..255")
        # Independent fist-end for the thumb BEND motor (joint 0), so it can
        # be tuned on the real hardware without touching the shared fist_pose
        # or the sway/roll gear. None = use the calibrated fist_pose[0].
        # Stretch the thumb opposition weight's dynamic range: real camera
        # geometry often spans only e.g. 0.3..0.6, so the motor uses a small
        # slice of its travel. w' = clamp(w * gain + bias).
        self.thumb_adaptive = bool(config.get("thumb_adaptive", True))
        self.thumb_gain = float(config.get("thumb_gain", 1.8))
        self.thumb_bias = float(config.get("thumb_bias", 0.0))
        if self.thumb_gain <= 0 or not -1.0 <= self.thumb_bias <= 1.0:
            raise ValueError("thumb_gain>0 and thumb_bias in -1..1 required")
        self.thumb_from_landmarks = bool(config.get("thumb_from_landmarks", True))
        self.thumb_near_ratio = float(config.get("thumb_near_ratio", 1.2))
        self.thumb_far_ratio = float(config.get("thumb_far_ratio", 2.6))
        self.thumb_root_fist = config.get("thumb_root_fist", None)
        if self.thumb_root_fist is not None:
            self.thumb_root_fist = int(self.thumb_root_fist)
            if not 0 <= self.thumb_root_fist <= 255:
                raise ValueError("thumb_root_fist must be 0..255")
        self.thumb_gain = float(config.get("thumb_gain", 1.0))
        if not 1.0 <= self.thumb_gain <= 10.0:
            raise ValueError("hand_visual_follow.thumb_gain must be in 1..10")
        # Thumb travel limit: shrink the three thumb joints' range to avoid
        # colliding with the other fingers. 1.0 = full range, 0.6 = 60%.
        self.thumb_travel = float(config.get("thumb_travel", 0.6))
        if not 0.0 <= self.thumb_travel <= 1.0:
            raise ValueError("hand_visual_follow.thumb_travel must be in 0..1")
        # Independent sway/roll limits: smaller = less across-palm sweep / less
        # roll, so the thumb tip rests against the index tip (OK) instead of
        # folding on top of it. None -> follow thumb_travel.
        self.thumb_sway_travel = config.get("thumb_sway_travel", None)
        self.thumb_roll_travel = config.get("thumb_roll_travel", None)
        for key, value in (("thumb_sway_travel", self.thumb_sway_travel),
                           ("thumb_roll_travel", self.thumb_roll_travel)):
            if value is not None and not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"hand_visual_follow.{key} must be in 0..1")
        # Four finger-root joints (index/middle/ring/pinky) get their own gain:
        # whole-frame RTMPose under-reports their curl just as it does the
        # thumb, and without this the proximal joints barely bend.
        self.fist_reference = float(config.get("fist_reference", 0.6))
        if not 0.05 <= self.fist_reference <= 1.0:
            raise ValueError("hand_visual_follow.fist_reference must be in 0.05..1.0")
        self.synergy_weight = float(config.get("synergy_weight", 0.0))
        if not 0.0 <= self.synergy_weight <= 1.0:
            raise ValueError("hand_visual_follow.synergy_weight must be in 0..1")
        self.flexion_gain = float(config.get("flexion_gain", 1.0))
        if not 1.0 <= self.flexion_gain <= 10.0:
            raise ValueError("hand_visual_follow.flexion_gain must be in 1..10")
        self.flexion_gamma = float(config.get("flexion_gamma", 1.0))
        if not 0.05 <= self.flexion_gamma <= 2.0:
            raise ValueError("hand_visual_follow.flexion_gamma must be in 0.05..2.0")
        self.angle_deadband_deg = float(config.get("angle_deadband_deg", 10.0))
        if not 0.0 <= self.angle_deadband_deg <= 45.0:
            raise ValueError("hand_visual_follow.angle_deadband_deg must be in 0..45")
        # flexion is (pi - angle)/pi, so N degrees equals N/180 in flexion units.
        self.deadband = self.angle_deadband_deg / 180.0
        self.watchdog_ms = int(config.get("watchdog_ms", 600))
        if not 100 <= self.watchdog_ms <= 5000:
            raise ValueError("hand_visual_follow.watchdog_ms must be in 100..5000")
        self.max_obs_age_ms = int(config.get("max_obs_age_ms", 500))
        if not 50 <= self.max_obs_age_ms <= 5000:
            raise ValueError("hand_visual_follow.max_obs_age_ms must be in 50..5000")
        # The shared sink's timing must match this card's configured window;
        # a hard-coded descriptor would make the watchdog fire at the wrong
        # time (and silently ignore the operator's exhibition setting).
        self._descriptor = dict(FLEX_DESCRIPTOR)
        self._descriptor["rate"] = dict(FLEX_DESCRIPTOR["rate"],
                                        watchdog_ms=self.watchdog_ms,
                                        max_obs_age_ms=self.max_obs_age_ms)
        self.require_handedness = bool(config.get("require_handedness", True))

        self._calibration = {}
        for side in ("left", "right"):
            open_pose = config.get(f"{side}_open_pose")
            fist_pose = config.get(f"{side}_fist_pose")
            if open_pose is None and fist_pose is None:
                continue
            if open_pose is None or fist_pose is None:
                raise ValueError(f"{side} visual follow needs both open and fist poses")
            self._calibration[side] = (
                _raw_pose(open_pose, f"{side}_open_pose"),
                _raw_pose(fist_pose, f"{side}_fist_pose"),
            )
        if not self._calibration:
            raise ValueError("hand_visual_follow needs left_open_pose/fist_pose or right_open_pose/fist_pose")

    @staticmethod
    def _setting(value, name):
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ValueError(f"{name} must be an integer in 1..255")
        return value

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "start": (["input_topic"],
                          "订阅手部弯曲量并待命（不通电、不驱动）；项目启动时由画布自动调用"),
                "stop": ([], "停止跟随并立即释放力矩"),
                "pause": ([], "暂停跟随并释放力矩；保留绑定"),
                "resume": ([], "★上电并开始跟随；点击即启动，stop 用于紧急停止"),
                "info": ([], "只读查看绑定、检测到的手、最近目标与拒绝原因"),
                "config": (["bound_side"], "绑定到左手或右手；保存配置不会运动"),
            },
            {
                "confirm_motion": {"type": "boolean", "enum": [True, False],
                                   "description": "确认手及周围安全后选 true；只有 resume 需要"},
                "input_topic": {"type": "string",
                                "description": "ROS2 话题，需连接上游 hand_landmarks 的 control/hand_flex 输出"},
                "bound_side": {"type": "string", "enum": ["left", "right"]},
                "instance_id": {"type": "string"},
            },
        )
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"},
                             "on_interrupt_all": {"action": "stop"}}
        # The canvas renders start/stop/info/config as project buttons and
        # hides them from the ACTION dropdown. Put resume first in x-workbench
        # so the operator sees the one action that actually moves the hand.
        schema["x-workbench"] = {"entry": "resume"}
        # Show confirm_motion only when resume is selected, so the operator
        # can tick it before energizing (canvas renders enum as a dropdown;
        # its "true" value is delivered as a real boolean).
        schema["x-action-params"] = {
            "resume": {"params": ["confirm_motion"],
                       "description": "勾选 true 确认安全后上电开始跟随"},
            "start": {"params": ["input_topic"]},
            "config": {"params": ["bound_side"]},
        }
        definition = tool(
            PREFIX, "actuator",
            "手部视觉连续跟随：把 0..1 的五指弯曲量按已标定姿态映射到 L20 Lite 原始关节位置。",
            schema,
            topic_in=[{"format": "control/hand_flex",
                       "desc": "control/hand_flex 五指弯曲量指令流（motus.hand/1 payload）"}],
        )
        definition["title"] = "L20 Lite 视觉跟随"
        return definition

    def start(self):
        """Bundle lifecycle. A container restart must not energize the hand."""

    def stop(self):
        self._stop_streams()

    # ── connection and lifecycle ─────────────────────────────────────────────

    def _connection(self, instance_id: str, side: str, port: str):
        with self._lock:
            connection = self._connections.get(instance_id)
            if connection is not None and (connection.port != port or connection.side != side):
                connection.close()
                connection = None
            if connection is None:
                connection = L20LiteConnection(
                    {**self.hand.config, "side": side, "serial_port": port})
                self._connections[instance_id] = connection
            return connection

    def _side_for(self, instance_id: str):
        side = self._bindings.get(instance_id)
        found = self.hand._discover()
        if side is None:
            if len(found) != 1:
                raise RuntimeError(
                    "检测到 0 只或多只手；请在实例配置里选择 bound_side")
            side = next(iter(found))
        if side not in found:
            raise RuntimeError(f"绑定的 {side} 手未连接")
        if side not in self._calibration:
            raise RuntimeError(f"{side} 手没有视觉跟随所需的 open/fist 标定")
        port = found.get(side)
        if port is None:
            raise RuntimeError(f"{side} 手的串口未知")
        return side, port

    def _release(self, instance_id: str):
        with self.hand._bus_lock:
            connection = self._connections.get(instance_id)
            if connection is not None and connection._client is not None:
                try:
                    connection._client.set_torques([0] * JOINT_COUNT)
                except Exception:        # noqa: BLE001 -- unplug must not block stop
                    pass

    def _energize(self, instance_id: str):
        with self.hand._bus_lock:
            connection = self._connections.get(instance_id)
            if connection is not None and connection._client is not None:
                connection._client.set_speeds(
                    joint_speeds(min(255, round(self.speed * self.speed_ratio))))
                connection._client.set_torques([self.torque] * JOINT_COUNT)

    def _stop_streams(self):
        with self._lock:
            streams = list(self._streams)
            self._streams.clear()
        for instance_id in streams:
            self._release(instance_id)
        self._close_node()

    def _close_node(self):
        node, self._node = self._node, None
        self._subscribed_topic = ""
        if node is None:
            return
        try:
            if self._ros2 is not None and self._ros2.executor_core is not None:
                self._ros2.executor_core.remove_node(node)
        finally:
            node.destroy_node()

    def _observed_thumb_range(self):
        """Learned thumb weight range of the first stream, for diagnostics."""
        with self._lock:
            stream = next(iter(self._streams.values()), None)
        if stream is None:
            return None
        return [stream.get("thumb_w_lo"), stream.get("thumb_w_hi")]

    # ── dispatch ─────────────────────────────────────────────────────────────

    def _resolve_instance(self, instance_id: str) -> str:
        """Canvas custom-action calls omit instance_id; project start passes it.

        With exactly one stream there is no ambiguity, so use it. With two or
        more, never guess which hand to energize — require the caller to name
        one. This keeps the single-card canvas workflow working without
        weakening the multi-card safety boundary.
        """
        if instance_id and instance_id in self._streams:
            return instance_id
        with self._lock:
            if not self._streams:
                return instance_id
            if len(self._streams) == 1:
                return next(iter(self._streams))
        return instance_id

    def dispatch(self, action: str, args: dict):
        args = dict(args or {})
        raw_id = str(args.get("instance_id") or "")
        instance_id = self._resolve_instance(raw_id) if action in ("pause", "resume", "info") else raw_id
        if action == "config":
            side = args.get("bound_side")
            if not instance_id or side not in ("left", "right"):
                raise ValueError("instance_id and bound_side (left/right) are required")
            with self._lock:
                if instance_id in self._streams:
                    raise RuntimeError("cannot change binding while following")
                self._bindings[instance_id] = side
            return {"state": "configured", "instance_id": instance_id, "bound_side": side}

        if action == "info":
            found = self.hand._discover()
            with self._lock:
                stream = dict(self._streams.get(instance_id) or {})
                side = self._bindings.get(instance_id)
            return {
                "state": ("armed" if stream.get("paused") else "running") if stream else "idle",
                "bound_side": side,
                "detected_sides": list(found),
                "detected_count": len(found),
                "topic": self._subscribed_topic,
                # The canvas treats info() as authoritative after wiring; it
                # must see the bound input topic to confirm/preserve the link.
                "topic_in": ([{"format": "control/hand_flex",
                               "topic": self._subscribed_topic}]
                             if self._subscribed_topic else []),
                "control_stream": stream.get("sink").stats()
                if stream.get("sink") is not None else None,
                "calibrated_sides": list(self._calibration),
                "last_target": stream.get("target"),
                "last_flexions": stream.get("flexions"),
                "last_command_ms": stream.get("last_command_ms"),
                "commands": stream.get("commands", 0),
                "rejects": list(self._rejects[-10:]),
                "error": self._last_error,
            }

        if action == "stop":
            self._stop_streams()
            return {"state": "idle", "torque_released": True}

        if action == "pause":
            with self._lock:
                stream = self._streams.get(instance_id)
                if stream is None:
                    return {"state": "idle", "message": "card is not running"}
                stream["paused"] = True
            self._release(instance_id)
            return {"state": "paused", "torque_released": True}

        if action == "resume":
            # Energizing the hand is dangerous. Require an explicit
            # confirm_motion: true. The canvas now renders this field for the
            # resume action via `x-action-params` (see get_tool), so the
            # operator can tick it; a missing/false confirmation is refused.
            if not confirm_motion_ok(args):
                return {"state": "error",
                        "error": "resume requires confirm_motion: true"}
            with self._lock:
                stream = self._streams.get(instance_id)
                if stream is None:
                    return {"state": "idle", "message": "card is not running"}
                stream["paused"] = False
                stream["starved"] = False
                stream["last_data_monotonic"] = time.monotonic()
            self._energize(instance_id)
            return {"state": "running", "side": stream["side"]}

        if action == "start":
            # Canvas project-start calls this with no confirm_motion. Subscribe
            # and DISARM: the subscription is cheap and read-only, but the hand
            # stays torque-free until a human picks "resume" and confirms.
            topic = str(args.get("input_topic") or "")
            if not topic:
                return {"state": "error", "error": "input_topic is required; wire a control/hand_flex source"}
            side, port = self._side_for(instance_id)
            connection = self._connection(instance_id, side, port)
            with self.hand._bus_lock:
                client = connection.get()
                client.get_positions()       # fail closed on a dead bus before energizing
            with self._lock:
                self._streams[instance_id] = {
                    "side": side, "port": port, "target": None, "flexions": None,
                    "accepted_flexions": None,   # last pose that actually moved the hand
                    "thumb_w_lo": None, "thumb_w_hi": None,  # adaptive range
                    "starved": False,
                    "paused": True,          # start = armed, not moving
                    "last_data_monotonic": time.monotonic(),
                    "last_command_monotonic": 0.0, "last_command_ms": 0,
                    "commands": 0,
                    "sink_seq": 0,           # control message sequence per stream
                    "sink": None,            # shared-admission ControlSink (created below)
                }
                self._last_error = ""
                self._rejects.clear()
                # Shared admission chain for this stream. Apply only records the
                # admitted values; the actual vendor mapping (which needs
                # landmarks + per-side calibration) stays in accept().
                stream = self._streams[instance_id]
                stream["sink"] = ControlSink(
                    self._descriptor,
                    lambda values, gripper, iid=instance_id: self._record_admitted(iid, values),
                    on_watchdog=lambda iid=instance_id: self._sink_watchdog(iid),
                    on_abort=lambda iid=instance_id: self._sink_abort(iid),
                )
            try:
                self._subscribe(topic)
            except Exception as error:       # noqa: BLE001
                self._stop_streams()
                return {"state": "error", "error": f"subscribe failed: {error}"}
            return {"state": "armed", "input": topic, "side": side,
                    "message": "已订阅并待命，手未通电；确认安全后执行 resume 开始跟随",
                    "max_hz": self.max_hz, "watchdog_ms": self.watchdog_ms,
                    "max_step": self.max_step, "thumb_gain": self.thumb_gain,
                    "thumb_travel": self.thumb_travel,
                    "thumb_sway_travel": self.thumb_sway_travel,
                    "thumb_roll_travel": self.thumb_roll_travel,
                    "angle_deadband_deg": self.angle_deadband_deg,
                    "speed_ratio": self.speed_ratio,
                    "flexion_gain": self.flexion_gain,
                    "flexion_gamma": self.flexion_gamma,
                    "fist_reference": self.fist_reference,
                    "thumb_root_fist": self.thumb_root_fist,
                    "thumb_from_landmarks": self.thumb_from_landmarks,
                    "thumb_adaptive": self.thumb_adaptive,
                    "thumb_observed_range": self._observed_thumb_range(),
                    "thumb_gain": self.thumb_gain,
                    "thumb_bias": self.thumb_bias,
                    "synergy_weight": self.synergy_weight}
        return None

    # ── stream ───────────────────────────────────────────────────────────────

    def _subscribe(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from std_msgs.msg import String

        if self._ros2 is None or self._ros2.ctx_core is None:
            raise RuntimeError("driver has no core-domain ROS context")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        node = Node("l20_lite_visual_follow", context=self._ros2.ctx_core)
        node.create_subscription(String, topic, self._on_message, qos)
        # A watchdog needs its own heartbeat: silence produces no callbacks.
        # Tick the shared sinks often enough that a watchdog release is not
        # delayed by a large fraction of the timeout window (a 5 s exhibition
        # watchdog must not wait 2.5 s extra). The sink itself decides when
        # silence has actually lasted long enough.
        node.create_timer(0.5, self._on_watchdog)
        self._ros2.executor_core.add_node(node)
        self._node = node
        self._subscribed_topic = topic

    def _reject(self, reason: str):
        with self._lock:
            self._rejects.append(reason)
            if len(self._rejects) > 50:
                del self._rejects[:-50]

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as error:           # noqa: BLE001
            self._reject(f"undecodable payload: {error}")
            return
        self.accept(payload)

    def _record_admitted(self, instance_id: str, values, gripper=None):
        """ControlSink.apply callback: stash the admitted (clamped) curls.

        The vendor mapping needs per-side calibration and the raw landmark
        geometry, which the shared control contract does not carry. So the
        sink only decides *whether* this sample is admissible and clamps the
        five curls; `accept()` picks them up and does the card-specific work.
        """
        with self._lock:
            stream = self._streams.get(instance_id)
            if stream is not None:
                stream["admitted"] = tuple(float(v) for v in values)

    def _sink_watchdog(self, instance_id: str):
        """Shared watchdog fired: release torque and mark this stream starved."""
        self._release(instance_id)
        with self._lock:
            stream = self._streams.get(instance_id)
            if stream is not None:
                stream["starved"] = True
            self._last_error = (
                f"手部视觉数据超过 {self.watchdog_ms} ms 未更新，已释放力矩")
        self._reject("control watchdog")

    def _sink_abort(self, instance_id: str):
        """Shared abort (force-torque / escalation): stop everything."""
        self._sink_watchdog(instance_id)
        self._stop_streams()

    def _apply_flexions(self, instance_id: str, stream: dict, payload: dict,
                        values, now: float, now_ms: int) -> bool:
        """Card-specific work after the shared sink admitted this sample.

        `values` are the five admitted (clamped) curls from ControlSink. All
        vendor mapping, thumb opposition, deadband and the raw-position write
        stay here; the shared chain owns admission/timing/arbitration/limits.
        Returns True only when a position command was actually written.
        """
        # Use the values admitted (and possibly clamped) by the shared sink,
        # not the raw payload — the sink's hard-limit/step decisions must reach
        # the motors, otherwise its admission would be cosmetic.
        side = stream["side"]
        flexions = {name: float(v) for name, v in
                    zip(("thumb", "index", "middle", "ring", "pinky"), values)}
        for name in FLEXION_JOINTS:
            gain = self.thumb_gain if name == "thumb" else self.flexion_gain
            if gain != 1.0 and name in flexions:
                raw = flexions[name]
                try:
                    flexions[name] = min(1.0, max(0.0,
                        float(raw) * gain))
                except (TypeError, ValueError):
                    pass    # target_from_flexions reports the bad value
        flexions = normalized_grasp(
            flexions, self.fist_reference, self.synergy_weight)
        # Live (pre-deadband) values for the thumb gear only.
        live_for_gear = dict(flexions)
        # Deadband: ignore small finger jitter. Compare against the
        # last pose that ACTUALLY moved the hand, not the previous
        # frame — otherwise a slow continuous bend never accumulates
        # past the threshold and the hand would never move.
        # flexion = (pi-angle)/pi, so 10 deg == 10/180 flexion units.
        if self.deadband > 0:
            accepted = stream.get("accepted_flexions")
            if accepted is None:
                stream["accepted_flexions"] = dict(flexions)
            else:
                # Per-finger deadband: only a finger that moved past
                # its own threshold gets updated. Fingers still inside
                # the deadband keep their previous accepted value, so
                # their micro-jitter never reaches the motor. This is
                # what makes single-finger control precise.
                merged = dict(accepted)
                for name in FLEXION_JOINTS:
                    if name not in flexions:
                        return False
                    new_value = float(flexions[name])
                    old_value = float(accepted.get(name, new_value))
                    if abs(new_value - old_value) >= self.deadband:
                        merged[name] = new_value
                if merged == accepted:
                    self._reject("deadband")
                    return False
                flexions = merged
                stream["accepted_flexions"] = dict(merged)
        # Thumb opposition from in-hand geometry (landmarks), fully
        # independent of the four fingers. Opt-out via config.
        thumb_override = None
        if self.thumb_from_landmarks:
            raw_lm = payload.get("landmarks")
            if raw_lm:
                w = thumb_opposition_from_landmarks(raw_lm)
                if self.thumb_adaptive:
                    w, lo, hi = adaptive_weight(
                        w, stream.get("thumb_w_lo"),
                        stream.get("thumb_w_hi"))
                    stream["thumb_w_lo"] = lo
                    stream["thumb_w_hi"] = hi
                else:
                    w = max(0.0, min(1.0,
                        w * self.thumb_gain + self.thumb_bias))
                thumb_override = w
        try:
            target = target_from_flexions(
                flexions, *self._calibration[side],
                gamma=self.flexion_gamma,
                gear_flexions=live_for_gear,
                thumb_root_fist=self.thumb_root_fist,
                thumb_override=thumb_override,
                thumb_travel=self.thumb_travel,
                thumb_sway_travel=self.thumb_sway_travel,
                thumb_roll_travel=self.thumb_roll_travel)
        except ValueError as error:
            self._reject(str(error))
            return False

        try:
            with self.hand._bus_lock:
                connection = self._connections.get(instance_id)
                if connection is None:
                    self._reject("connection missing")
                    return False
                client = connection.get()
                current = client.get_positions()
                if len(current) != JOINT_COUNT:
                    self._reject(f"device returned {len(current)} positions")
                    return False
                command = clamp_step(current, target, self.max_step)
                client.set_positions(command)
                wrote = True
        except Exception as error:       # noqa: BLE001
            with self._lock:
                self._last_error = f"{type(error).__name__}: {error}"
            self._release(instance_id)
            self._stop_streams()
            self._reject(f"write failed: {error}")
            return False


        with self._lock:
            stream.update({
        "target": target,
        "flexions": dict(payload.get("flexions") or {}),
        "last_data_monotonic": now,
        "last_command_monotonic": now,
        "last_command_ms": now_ms,
        "commands": stream.get("commands", 0) + 1,
            })
        return wrote

    def accept(self, payload: dict) -> bool:
        """Admit one control/hand_flex sample via the shared ControlSink.

        ROS-free and directly testable. Returns True only when a position
        command was actually written.
        """
        with self._lock:
            streams = dict(self._streams)
        if not streams:
            return False
        if not isinstance(payload, dict) or payload.get("schema") != "motus.hand/1":
            self._reject("payload is not motus.hand/1")
            return False
        flexions_in = payload.get("flexions") or {}
        if not isinstance(flexions_in, dict):
            self._reject("flexions must be an object")
            return False
        # The five curls in descriptor order. ControlSink enforces 0..1 hard
        # limits, step bounds, source arbitration and freshness on these.
        values = [flexions_in.get(n, 0.0) for n in
                  ("thumb", "index", "middle", "ring", "pinky")]
        handedness = str(payload.get("handedness") or "").lower()
        obs_ms = int(payload.get("obs_stamp_ms") or payload.get("stamp_ms") or 0)
        stamp_ms = int(payload.get("stamp_ms") or obs_ms)
        wrote = False
        for instance_id, stream in streams.items():
            if stream.get("paused"):
                continue
            if self.require_handedness and handedness and handedness != stream["side"]:
                self._reject(f"handedness {handedness} does not match {stream['side']}")
                continue
            sink = stream.get("sink")
            if sink is None:
                self._reject("control sink not initialized")
                continue
            stream["sink_seq"] = stream.get("sink_seq", 0) + 1
            # Standard motus.control/1 envelope the shared sink expects.
            message = {
                "schema": "motus.control/1",
                "mode": "joint_position",
                "dof": 5,
                "values": values,
                "source": FLEX_SOURCE,
                "priority": 50,
                "seq": stream["sink_seq"],
                "stamp_ms": stamp_ms,
                "obs_stamp_ms": obs_ms if obs_ms > 0 else None,
                "ttl_ms": max(200, self.watchdog_ms),
            }
            outcome = sink.submit(message)
            if not outcome.applied:
                self._reject(f"control sink: {outcome.verdict.value} {outcome.reason}")
                continue
            admitted = stream.get("admitted")
            if admitted is None:
                continue
            now = time.monotonic()
            now_ms = int(time.time() * 1000)
            # Fresh data that passed the shared chain ended a watchdog hold.
            if stream.get("starved"):
                self._energize(instance_id)
                stream["starved"] = False
                with self._lock:
                    self._last_error = ""
            if outcome.verdict.value == "clamped":
                self._reject("control sink clamped curl values")
            if now - stream["last_command_monotonic"] < 1.0 / self.max_hz:
                self._reject("throttled")
                continue
            if self._apply_flexions(instance_id, stream, payload,
                                    admitted, now, now_ms):
                wrote = True
        return wrote

    def _on_watchdog(self):
        """Drive each stream's shared sink timer.

        Freshness, watchdog silence, stand-down escalation and force-torque
        abort are all owned by common/control.ControlSink. Its on_watchdog /
        on_abort callbacks release torque and mark the stream starved, so the
        only thing this timer must do is tick every live sink. Paused streams
        are skipped: they are disarmed and no data is expected.
        """
        with self._lock:
            streams = dict(self._streams)
        for instance_id, stream in streams.items():
            if stream.get("paused"):
                continue
            sink = stream.get("sink")
            if sink is None:
                continue
            outcome = sink.tick()
            if outcome is not None:
                self._reject(f"control sink: {outcome.verdict.value} {outcome.reason}")
        # Never close the node here: the subscription must survive a dropout
        # so recovery can happen without a fresh start.
