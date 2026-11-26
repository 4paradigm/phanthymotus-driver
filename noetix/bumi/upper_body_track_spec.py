"""Upper-body retargeting maths for the `upper_body_track` card.

This module is deliberately dependency-free: no `rclpy`, no `cv2`, no `numpy`.
Everything here is a pure function over plain numbers and tuples, so the rules
that would otherwise fail silently — a sign flip, a joint pushed past its hard
limit, a nested payload the dashboard cannot render — are all pinned by tests
that run on a laptop.

Nothing in this module commands a motor. The card that imports it publishes
keypoints and joint angles and stops there; driving the arms is a separate card
that has to take over all 21 motors and keep its own balance (see the Bumi
professional guide §2.3 — `LowController.set_joint` rejects anything that is
not exactly 21 commands, so "move only the arms" is not an available option).

Frames
------
`camera` is the RealSense optical frame: x right, y down, z forward, metres.
`body` is the robot's own frame: x forward, y left, z up, metres. The depth card
publishes frames already registered onto the colour optical plane
(`device.py`, "depth-aligned-to-color"), so one pixel index addresses the same
surface in both streams and no extrinsic calibration is needed here.

Why directions and not positions
--------------------------------
The robot's shoulder-to-wrist reach is 312 mm against roughly 550-600 mm for an
adult. Copying hand *positions* would therefore be unsatisfiable by a factor of
nearly two. Copying the *direction* of each bone is scale-free and is the only
mapping the geometry actually supports; bone length is not used anywhere below.
"""

from __future__ import annotations

import math

# ── COCO-17 keypoints, the renderer's native pose vocabulary ──────────────────
# Same 17 points YOLO11-pose emits and `sensor/pose2d` draws; keeping the order
# identical means the keypoint list can be forwarded without a translation table.
COCO17_NAMES = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
)

COCO17_BONES = (
    (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (12, 16),
)

L_SHOULDER, R_SHOULDER = 5, 6
L_ELBOW, R_ELBOW = 7, 8
L_WRIST, R_WRIST = 9, 10
L_HIP, R_HIP = 11, 12

# ── Joint limits, from the 21-joint table in the Bumi professional guide §2.2 ─
# These are mechanical hard limits; a command outside them is not "aggressive",
# it is out of range. The roll ranges are asymmetric because the two arms are
# mirror-mounted, which is exactly the kind of thing a naive `abs()` hides.
ARM_LIMITS_DEG = {
    "left": {"pitch": (-135.0, 135.0), "roll": (-8.0, 111.0), "elbow": (-129.0, 0.0)},
    "right": {"pitch": (-135.0, 135.0), "roll": (-111.0, 8.0), "elbow": (-129.0, 0.0)},
}

# Shoulder-to-wrist reach, for the `reachable` hint only — never for the angles.
ROBOT_ARM_SPAN_M = 0.312

# A plausible adult shoulder-to-shoulder width, used solely to recover the
# camera focal length when the operator asks for `focal_px: auto`.
ADULT_SHOULDER_WIDTH_M = 0.38


# ── small vector helpers (tuples, so this module stays numpy-free) ────────────

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def unit(a):
    """Unit vector, or `None` when the input has no direction (coincident points)."""
    length = _norm(a)
    if length <= 1e-9:
        return None
    return (a[0] / length, a[1] / length, a[2] / length)


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


# ── projection ───────────────────────────────────────────────────────────────

def camera_to_body(point):
    """Optical frame (x right, y down, z forward) → robot body frame (x fwd, y left, z up)."""
    x_cam, y_cam, z_cam = point
    return (z_cam, -x_cam, -y_cam)


def backproject(u, v, depth_m, focal_px, width, height):
    """Pixel + depth → body-frame point in metres.

    The principal point is taken as the image centre. That assumption does not
    affect the arm angles: they come from *differences* between keypoints, where
    a constant offset cancels. It does move every point by the same small
    translation, which is why nothing downstream uses absolute position.
    """
    if depth_m is None or depth_m <= 0.0 or focal_px <= 0.0:
        return None
    x_cam = (u - width / 2.0) * depth_m / focal_px
    y_cam = (v - height / 2.0) * depth_m / focal_px
    return camera_to_body((x_cam, y_cam, depth_m))


def median(values):
    """Median of the positive entries, or `None` when there are none.

    RealSense returns 0 for "no measurement", so a naive mean over a patch that
    straddles a silhouette edge reports a surface that is not there.
    """
    usable = sorted(v for v in values if v is not None and v > 0.0)
    if not usable:
        return None
    mid = len(usable) // 2
    if len(usable) % 2:
        return usable[mid]
    return (usable[mid - 1] + usable[mid]) / 2.0


def estimate_focal_px(shoulder_px, depth_m, shoulder_width_m=ADULT_SHOULDER_WIDTH_M):
    """Focal length in pixels from an observed shoulder width at a known depth.

    Used only when the operator sets `focal_px: auto`. A wrong focal length
    does not rotate the arm directions uniformly — it scales the forward/back
    component against the vertical one — so it is worth measuring once instead
    of guessing.
    """
    if shoulder_px is None or shoulder_px <= 1e-6 or depth_m is None or depth_m <= 0.0:
        return None
    return shoulder_px * depth_m / shoulder_width_m


# ── the retarget itself ──────────────────────────────────────────────────────

def arm_angles(shoulder, elbow, wrist):
    """Three body-frame points → the four joint commands for one arm.

    Returns `{"pitch_deg", "roll_deg", "elbow_deg"}` in the robot's joint
    convention, before limits are applied.

    The shoulder is a 3-DOF ball joint and three points only ever determine two
    of its rotations: where the upper arm points. Humeral rotation (which way
    the elbow crease faces) is not observable without the hand's orientation,
    so it is reported as zero rather than invented; the card's description says
    so. What that costs is palm orientation, not reach.

    Signs come from the mirrored mounting. Writing `u` for the unit upper-arm
    direction and decomposing the rest pose (arm hanging down, `(0, 0, -1)`) as
    pitch about the lateral axis followed by roll about the forward axis gives

        u = (-cos(roll)·sin(pitch), sin(roll), -cos(roll)·cos(pitch))

    so `roll = -asin(u_y)` and `pitch = atan2(-u_x, -u_z)` are exact, not
    approximate. The negative sign on roll is what puts a person's right arm
    (abducting toward -y) onto the robot's left arm, whose abduction is the
    positive half of its own roll range — that is the mirror mapping, with no
    coordinate flip anywhere. A sign flip per axis is exposed in config for the
    case where the URDF disagrees with this derivation.
    """
    upper = unit(_sub(elbow, shoulder))
    fore = unit(_sub(wrist, elbow))
    if upper is None:
        raise ValueError("shoulder and elbow coincide; no upper-arm direction")

    roll = -math.degrees(math.asin(clamp(upper[1], -1.0, 1.0)))
    # Arm straight out to the side leaves no direction in the flexion plane, and
    # `atan2(0, -0.0)` answers 180 rather than 0 — which, after clamping, would
    # read as "arm swung fully behind the robot". Pure abduction is what the pose
    # actually is, so the pitch defaults to zero there.
    horizontal = math.hypot(upper[0], upper[2])
    pitch = (0.0 if horizontal < 1e-6
             else math.degrees(math.atan2(-upper[0], -upper[2])))

    if fore is None:
        elbow_deg = 0.0
    else:
        # Flexion is the angle between the segments; the robot's elbow range is
        # [-129, 0], so a straight arm is 0 and bending is negative.
        elbow_deg = -math.degrees(math.acos(clamp(_dot(upper, fore), -1.0, 1.0)))

    return {"pitch_deg": pitch, "roll_deg": roll, "elbow_deg": elbow_deg}


def apply_signs(angles, pitch_sign=1.0, roll_sign=1.0, elbow_sign=1.0):
    """Flip axes for a URDF whose positive direction disagrees with the derivation."""
    return {
        "pitch_deg": angles["pitch_deg"] * pitch_sign,
        "roll_deg": angles["roll_deg"] * roll_sign,
        "elbow_deg": angles["elbow_deg"] * elbow_sign,
    }


def limit_angles(angles, side):
    """Clamp to the mechanical limits and report which axes had to be clamped.

    Clamping silently would turn "the operator is asking for something the
    robot cannot do" into "the robot looks slightly wrong", which is the failure
    mode this bundle's spec layer exists to prevent. The caller forwards the
    flags so the panel can say why the pose is off.
    """
    limits = ARM_LIMITS_DEG[side]
    out = {}
    clamped = {"pitch": 0, "roll": 0, "elbow": 0}
    for axis in ("pitch", "roll", "elbow"):
        low, high = limits[axis]
        raw = angles[f"{axis}_deg"]
        value = clamp(raw, low, high)
        if abs(value - raw) > 1e-6:
            clamped[axis] = 1
        out[f"{axis}_deg"] = value
    return out, clamped


def reach_indicator(shoulder, elbow, wrist):
    """Human arm span as a fraction of the robot's, for the panel only.

    Reported so an operator can see at a glance that the person is asking for
    something twice the robot's reach; it never scales an angle.
    """
    span = _norm(_sub(elbow, shoulder)) + _norm(_sub(wrist, elbow))
    if span <= 1e-9:
        return None
    return round(span / ROBOT_ARM_SPAN_M, 3)


# ── person selection and gating ──────────────────────────────────────────────

def keypoints_of(person):
    """`[[x, y, v], ...]` → list of `(x, y, v)` tuples, tolerant of missing points."""
    points = []
    for kp in person.get("keypoints") or []:
        if not kp:
            points.append((0.0, 0.0, 0.0))
            continue
        x = float(kp[0]) if len(kp) > 0 else 0.0
        y = float(kp[1]) if len(kp) > 1 else 0.0
        v = float(kp[2]) if len(kp) > 2 else 1.0
        points.append((x, y, v))
    return points


def pick_person(persons, min_confidence=0.5):
    """The largest confident person in frame, or `None`.

    Largest bounding box rather than first-in-list: the person being mirrored is
    the one standing nearest the camera, and list order out of a detector is not
    stable across frames.
    """
    best = None
    best_area = -1.0
    for person in persons or []:
        confidence = float(person.get("confidence") or 0.0)
        if confidence < min_confidence:
            continue
        bbox = person.get("bbox") or [0.0, 0.0, 0.0, 0.0]
        if len(bbox) < 4:
            continue
        area = max(0.0, bbox[2] - bbox[0]) * max(0.0, bbox[3] - bbox[1])
        if area > best_area:
            best, best_area = person, area
    return best


def upper_body_visible(points, indices, min_conf):
    """True when every keypoint the retarget needs was seen clearly enough."""
    for index in indices:
        if index >= len(points):
            return False
        if points[index][2] < min_conf:
            return False
    return True


def posture_name(points, min_conf=0.3):
    """`standing` / `unknown`, from hips below shoulders in image coordinates.

    Both channels the renderer accepts are optional; this is the honest cheap
    one. `activity` is deliberately absent — it needs a time window, and a card
    that guesses is worse than one that abstains.
    """
    needed = (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)
    if not upper_body_visible(points, needed, min_conf):
        return "unknown"
    shoulder_y = (points[L_SHOULDER][1] + points[R_SHOULDER][1]) / 2.0
    hip_y = (points[L_HIP][1] + points[R_HIP][1]) / 2.0
    return "standing" if hip_y > shoulder_y else "unknown"


# ── payloads ─────────────────────────────────────────────────────────────────

def build_pose2d_payload(persons, width, height, posture=None):
    """Payload for `sensor/pose2d`.

    Keypoints stay in pixels and are never clipped: an off-image wrist is real
    information and the renderer handles out-of-bounds points. `image_size` is
    always sent because the renderer otherwise infers an extent that drifts as
    the person moves.
    """
    entries = []
    for index, person in enumerate(persons or []):
        points = keypoints_of(person)
        entry = {
            "keypoints": [[x, y, v] for (x, y, v) in points],
            "id": index,
        }
        bbox = person.get("bbox")
        if bbox and len(bbox) >= 4:
            entry["bbox"] = [float(v) for v in bbox[:4]]
        label = (posture or {}).get(index)
        if label:
            entry["posture"] = label
        entries.append(entry)
    return {
        "persons": entries,
        "count": len(entries),
        "image_size": [int(width), int(height)],
        "keypoint_names": list(COCO17_NAMES),
        "skeleton": [list(bone) for bone in COCO17_BONES],
    }


def build_angles_payload(*, tracked, person_count, target_id, angles_by_side,
                         clamped_by_side, depth_used, depth_coverage,
                         focal_px, latency_ms, frame_age_ms, reach,
                         pitch_sign, roll_sign, elbow_sign, degraded, note=""):
    """Payload for `data/json`.

    Flat by construction: the dashboard renders a nested object as raw JSON
    text, so a nested payload is a payload nobody reads. Every value is a
    scalar; `angles_by_side` is flattened here rather than by the caller.
    """
    payload = {
        "tracked": bool(tracked),
        "person_count": int(person_count),
        "target_id": int(target_id) if target_id is not None else -1,
        "depth_used": bool(depth_used),
        "depth_coverage": round(float(depth_coverage), 3),
        "focal_px": round(float(focal_px), 1),
        "latency_ms": round(float(latency_ms), 1),
        "frame_age_ms": round(float(frame_age_ms), 1),
        "reach_ratio": round(float(reach), 3) if reach is not None else -1.0,
        "pitch_sign": float(pitch_sign),
        "roll_sign": float(roll_sign),
        "elbow_sign": float(elbow_sign),
        "degraded": degraded or "",
        "note": note or "",
    }
    for side in ("left", "right"):
        angles = (angles_by_side or {}).get(side)
        clamped = (clamped_by_side or {}).get(side) or {}
        for axis in ("pitch", "roll", "elbow"):
            payload[f"{side[0]}_{axis}_deg"] = (
                round(float(angles[f"{axis}_deg"]), 1) if angles else -999.0)
            payload[f"{side[0]}_{axis}_clamped"] = int(clamped.get(axis, 0))
    return payload


# ── smoothing ────────────────────────────────────────────────────────────────

class OneEuroFilter:
    """1€ filter for a single scalar.

    Keypoint jitter is not noise you can average away with a fixed window: a
    slow-moving arm needs heavy smoothing and a fast wave needs none. A fixed
    low-pass strong enough to steady a resting hand makes a wave arrive late,
    and that lag is the difference between "mirroring" and "lagging".

    Kept here rather than in the card because a filter that is subtly wrong
    still runs and still produces plausible numbers.
    """

    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x_prev = None
        self._dx_prev = 0.0
        self._t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self._x_prev = None
        self._dx_prev = 0.0
        self._t_prev = None

    def __call__(self, value, timestamp_s):
        if self._x_prev is None or self._t_prev is None:
            self._x_prev = float(value)
            self._t_prev = float(timestamp_s)
            return self._x_prev

        dt = float(timestamp_s) - self._t_prev
        if dt <= 0.0:
            dt = 1e-3
        self._t_prev = float(timestamp_s)

        dx = (float(value) - self._x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        self._dx_prev = dx_hat

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * float(value) + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        return x_hat


class AngleBank:
    """Six filtered arm angles, one filter per joint.

    A joint that disappears (person turns away, arm occluded) keeps its last
    value instead of jumping: the callers report the gap through `degraded`, and
    a frozen pose plus an honest flag beats an interpolated guess.
    """

    AXES = ("l_pitch", "l_roll", "l_elbow", "r_pitch", "r_roll", "r_elbow")

    def __init__(self, **kwargs):
        self._filters = {axis: OneEuroFilter(**kwargs) for axis in self.AXES}
        self._last = {}

    def reset(self):
        for filt in self._filters.values():
            filt.reset()
        self._last.clear()

    def update(self, angles_by_side, timestamp_s):
        """Smooth a `{"left": {...}, "right": {...}}` mapping; missing sides hold."""
        for side in ("left", "right"):
            angles = (angles_by_side or {}).get(side)
            if not angles:
                continue
            for axis in ("pitch", "roll", "elbow"):
                key = f"{side[0]}_{axis}"
                self._last[key] = self._filters[key](
                    angles[f"{axis}_deg"], timestamp_s)
        return dict(self._last)
