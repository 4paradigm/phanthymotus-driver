"""Bumi arm-only DetaForce motion previews; never sends SDK motor commands.

Angles use the bundled Bumi URDF, in radians. These kinematic clips are not
balance policies or hardware-qualified trajectories. Real playback uses taught
recordings through detaforce_rps.py instead of streaming these joint values.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


ARM_JOINTS = tuple(f"{side}_{name}_joint" for side in ("l", "r")
                   for name in ("arm_pitch", "arm_roll", "arm_yaw", "elbow_pitch"))
LABELS = {
    "invite_one": "单手邀请",
    "invite_both": "双手邀请",
    "countdown": "双臂交替转动",
    "double_punch": "双拳前后交替（猛攻）",
    "mouse": "鼠鼠举手",
    "confident": "花来",
}
INVITE_TEXT = "来，玩一局猜拳！"
# Preview limits only. They are deliberately slower than raw SDK examples.
MAX_VELOCITY = 2.0
MAX_ACCELERATION = 6.0


def arm(side, pitch=0.0, roll=0.0, yaw=0.0, elbow=0.0):
    if side not in ("l", "r"):
        raise ValueError("Expected l or r arm")
    sign = 1 if side == "l" else -1
    return dict(zip((f"{side}_{name}_joint" for name in
                     ("arm_pitch", "arm_roll", "arm_yaw", "elbow_pitch")),
                    (pitch, sign * roll, sign * yaw, elbow)))


def both(pitch=0.0, roll=0.0, yaw=0.0, elbow=0.0):
    return {**arm("l", pitch, roll, yaw, elbow),
            **arm("r", pitch, roll, yaw, elbow)}


@dataclass(frozen=True)
class Keyframe:
    time_s: float
    phase: str
    joints: tuple[float, ...]


@dataclass(frozen=True)
class Motion:
    action: str
    keyframes: tuple[Keyframe, ...]

    @property
    def duration_s(self):
        return self.keyframes[-1].time_s

    def sample(self, elapsed_s):
        if not math.isfinite(elapsed_s):
            raise ValueError("Expected finite sample time")
        elapsed_s = min(self.duration_s, max(0.0, elapsed_s))
        for first, last in zip(self.keyframes, self.keyframes[1:]):
            if elapsed_s <= last.time_s:
                u = (elapsed_s - first.time_s) / (last.time_s - first.time_s)
                blend = u**3 * (10 - 15*u + 6*u*u)
                joints = {name: a + blend*(b-a) for name, a, b in
                          zip(ARM_JOINTS, first.joints, last.joints)}
                return joints, last.phase
        return dict(zip(ARM_JOINTS, self.keyframes[-1].joints)), "rest"


def build_motion(action, *, cycles=2, tempo=1.0):
    if action not in LABELS:
        raise ValueError("Unknown DetaForce action")
    if type(cycles) is not int or not 1 <= cycles <= 3:
        raise ValueError("cycles must be an integer in 1..3")
    if type(tempo) not in (int, float) or not math.isfinite(tempo) or not .5 <= tempo <= 1.25:
        raise ValueError("tempo must be finite and in 0.5..1.25")
    neutral = (0.0,) * len(ARM_JOINTS)
    frames = [Keyframe(0.0, "rest", neutral)]

    def add(phase, pose, duration=.65):
        if set(pose) - set(ARM_JOINTS):
            raise ValueError("Only the eight arm joints may move")
        values = tuple(float(pose.get(name, 0.0)) for name in ARM_JOINTS)
        delta = max(abs(a-b) for a, b in zip(values, frames[-1].joints))
        # Quintic interpolation has peak slope 1.875 and peak curvature 5.774.
        duration = max(duration/tempo, 1.875*delta/MAX_VELOCITY,
                       math.sqrt(5.774*delta/MAX_ACCELERATION))
        frames.append(Keyframe(frames[-1].time_s+duration, phase, values))

    if action in ("invite_one", "invite_both"):
        pose = both if action == "invite_both" else lambda *a: arm("r", *a)
        for _ in range(cycles):
            add("reach", pose(-1.55, .22, 0.0, -.35))
            add("beckon_to_face", pose(-1.20, -.08, -.15, -2.10))
        add("invite_hold", pose(-1.20, -.08, -.15, -2.10), .55)
    elif action == "countdown":
        # Half a turn out of phase: one forearm rises as the other falls.
        # Shoulder and elbow phases differ to trace an orbit, not a clap.
        for index in range(cycles*8+1):
            angle = index*math.tau/8
            pose = {}
            for side, phase in (("l", angle), ("r", angle+math.pi)):
                pose.update(arm(side, -1.00+.30*math.sin(phase), .15,
                                .12, -1.10+.38*math.cos(phase)))
            add("alternate_orbit", pose, .28)
    elif action == "double_punch":
        extended = (-1.20, .20, .0, -.30)
        retracted = (-.35, .24, .10, -1.90)
        for _ in range(cycles):
            add("left_forward", {**arm("l", *extended), **arm("r", *retracted)})
            add("right_forward", {**arm("l", *retracted), **arm("r", *extended)})
        add("double_guard", both(-.55, .25, .08, -1.65), .55)
    elif action == "mouse":
        add("hands_up", both(-1.20, .65, .10, -1.85), 1.0)
        add("mouse_hold", both(-1.20, .65, .10, -1.85), 1.35)
    elif action == "confident":
        raised_fist = arm("r", -1.0, .22, .08, -1.85)
        add("fist_before_face", raised_fist, 1.0)
        add("confident_hold", raised_fist, .45)
        add("affirm_downstroke", arm("r", -.60, .22, .08, -1.95), .40)
        add("restore_fist", raised_fist, .70)
        add("confident_hold", raised_fist, .65)
    add("return_to_rest", {}, 1.2)
    return Motion(action, tuple(frames))
