"""Quaternion helpers for the `eef_pose` action space. Plain `math`, no numpy.

`common/control` is imported by every bundle in this repo and stays free of
ROS, of any vendor SDK and of numpy — the whole point is that the safety chain
is testable on a laptop with nothing installed. Four functions over 4-tuples is
not worth a dependency.

**Order is xyzw**, matching `geometry_msgs/Quaternion`, scipy and the
`motus.vla/1` standard layout. pinocchio and Eigen constructors take wxyz, so
anything crossing into those takes the conversion at the boundary and says so.
An order swap is silent: `(0,0,0,1)` and `(1,0,0,0)` are both unit quaternions,
one is identity and the other is a half turn about x.
"""

from __future__ import annotations

import math

# A quaternion that has travelled through JSON and a float32 policy head will
# not be exactly unit. 1e-3 is loose enough to survive that and tight enough
# that an R6 triple or an rpy triple read as a quaternion fails it: those have
# no reason to land within a thousandth of the unit sphere.
UNIT_TOLERANCE = 1e-3


def norm(quaternion) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in quaternion))


def is_unit(quaternion, tolerance: float = UNIT_TOLERANCE) -> bool:
    return abs(norm(quaternion) - 1.0) <= tolerance


def normalize(quaternion) -> tuple[float, float, float, float]:
    length = norm(quaternion)
    if length == 0.0:
        raise ValueError("cannot normalize a zero quaternion")
    x, y, z, w = (float(v) / length for v in quaternion)
    return (x, y, z, w)


def angle_between(a, b) -> float:
    """Rotation angle in radians between two orientations, in [0, pi].

    `q` and `-q` are the *same* orientation, so the absolute value of the dot
    product is what matters. Without it, a policy whose output flips sign
    between two steps — which costs nothing and happens — would read as a
    180° jump and be clamped or rejected every time.
    """
    dot = abs(sum(float(p) * float(q) for p, q in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def slerp_limit(previous, want, max_angle: float):
    """Move from `previous` toward `want` by at most `max_angle` radians.

    Returns `(quaternion, limited)`. This is the step clamp for an orientation,
    and it exists because the component-wise one is wrong here: clamping x, y,
    z, w independently produces a vector that is no longer unit length, which
    the contract check then rejects. The symptom of doing it the naive way is
    "every command is rejected as soon as the policy moves a little faster",
    and nothing in that message points at the clamp.
    """
    a = tuple(float(v) for v in previous)
    b = tuple(float(v) for v in want)
    dot = sum(p * q for p, q in zip(a, b))
    if dot < 0.0:                       # shortest path; -q is the same rotation
        b = tuple(-v for v in b)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))

    angle = 2.0 * math.acos(dot)
    if max_angle <= 0.0 or angle <= max_angle:
        return tuple(want), False

    half = math.acos(dot)               # slerp's parameter is the half-angle
    sin_half = math.sin(half)
    if sin_half < 1e-9:                 # numerically identical orientations
        return tuple(want), False

    t = max_angle / angle
    scale_a = math.sin((1.0 - t) * half) / sin_half
    scale_b = math.sin(t * half) / sin_half
    blended = tuple(scale_a * p + scale_b * q for p, q in zip(a, b))
    return normalize(blended), True
