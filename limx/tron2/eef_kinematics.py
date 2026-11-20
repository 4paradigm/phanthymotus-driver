"""TRON2 7-DoF arm kinematics; NumPy only, without hardware I/O.

Positions are metres, angles are radians, quaternions are [x, y, z, w].
The endpoint is the vendor manipulation controller TCP, L_ee_link or R_ee_link.
IK is a seeded local endpoint solver. Its iterations are NOT a commanded path;
joint interpolation to its result does not guarantee a Cartesian straight line.
No collision, velocity, acceleration, or robot calibration checks are performed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import xml.etree.ElementTree as ET

import numpy as np

from skeleton import JOINT_NAMES, MODEL_PATH
from vendor import ARM_LOWER, ARM_UPPER

GRASPER_FRAMES = {"left": "grasper_L_Link", "right": "grasper_R_Link"}
# LimX manipulation URDF 10af63c, tron2_manipulation_config/robot.urdf:
# wrist_roll -> L_ee_link/R_ee_link = (-.0338, 0, -.2125), identity rotation.
# Subtract the DACH model's wrist_roll -> grasper = (-.0317, 0, -.0812).
# Verified against request_get_move_pose (both arms); the Jacobian below
# uses this offset TCP origin as well as FK.
GRASPER_TO_TCP_M = np.array([-.0021, 0., -.1313])

# The same TRON2A MoveJ bounds used by autonomous_task (SDK guide §3.6.1).
# The solver intersects these with the installed URDF and driver bounds.
_CURRENT_MOVEJ_LOWER = (-3.14159, -.261799, -3.66519, -2.61799, -1.74533, -.7505, -.7505,
                        -3.14159, -2.96706, -1.48353, -2.61799, -1.39626, -.7505, -.7505)
_CURRENT_MOVEJ_UPPER = (2.60054, 2.96706, 1.48353, .523599, 1.39626, .7505, .7505,
                        2.60054, .261799, 3.66519, .523599, 1.74533, .7505, .7505)
MAX_ITERATIONS = 200
# A shortened desktop step stays on the requested Cartesian segment and on the
# current joint branch. These are the TRON2 task's single-move envelopes.
LOCAL_TRANSLATION_M = .1
LOCAL_ROTATION_RAD = math.radians(30)
LOCAL_JOINT_RAD = math.radians(35)
PREFIX_BISECTIONS = 8


def _vector(value, size: int, name: str) -> np.ndarray:
    try:
        raw = np.asarray(value, dtype=object)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {size} finite real numbers") from exc
    if raw.shape != (size,) or any(
        isinstance(x, (bool, np.bool_)) or not isinstance(x, Real) for x in raw.flat
    ):
        raise ValueError(f"{name} must contain exactly {size} real numbers (no bool/string)")
    result = np.asarray(raw, dtype=float)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    return result


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _exp(rotation_vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotation_vector))
    if not math.isfinite(theta):
        raise ValueError("rotation vector magnitude is not finite")
    if theta < 1e-8:
        k = _skew(rotation_vector)
        return np.eye(3) + k + 0.5 * k @ k
    k = _skew(rotation_vector / theta)
    return np.eye(3) + math.sin(theta) * k + (1. - math.cos(theta)) * k @ k


def _quaternion(rotation: np.ndarray) -> np.ndarray:
    """Stable matrix to canonical xyzw, including rotations near pi."""
    trace = float(np.trace(rotation))
    if trace > 0.:
        s = 2. * math.sqrt(max(0., trace + 1.))
        q = np.array([(rotation[2, 1] - rotation[1, 2]) / s,
                      (rotation[0, 2] - rotation[2, 0]) / s,
                      (rotation[1, 0] - rotation[0, 1]) / s, s / 4.])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2. * math.sqrt(max(0., 1. + rotation[i, i] - rotation[j, j] - rotation[k, k]))
        q = np.zeros(4)
        q[i] = s / 4.
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
    q /= np.linalg.norm(q)
    return -q if q[3] < 0. else q


def _rotation(quaternion) -> np.ndarray:
    q = _vector(quaternion, 4, "target_quaternion_xyzw")
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or abs(norm - 1.) > 1e-6:
        raise ValueError("target_quaternion_xyzw must be a unit quaternion (tolerance 1e-6)")
    x, y, z, w = q / norm
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def _log(rotation: np.ndarray) -> np.ndarray:
    q = _quaternion(rotation)
    length = float(np.linalg.norm(q[:3]))
    if length < 1e-10:
        return 2. * q[:3]
    return q[:3] * (2. * math.atan2(length, q[3]) / length)


def _xml_vector(text: str, name: str) -> np.ndarray:
    try:
        return _vector([float(x) for x in text.split()], 3, name)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"Invalid URDF {name}") from exc


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    child: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class NoFeasibleStep(ValueError):
    """No forward step was found along this otherwise valid TCP request."""


class Kinematics:
    """FK and seeded damped-least-squares IK for the two DACH arms.

    Arm names are ``left`` and ``right`` in the URDF/vendor identity mapping.
    The caller must verify that mapping against the operator's camera convention.
    """

    def __init__(self):
        """Read both arm chains from the platform's supplied URDF."""
        try:
            root = ET.parse(MODEL_PATH).getroot()
        except (ET.ParseError, TypeError) as exc:
            raise ValueError(f"Cannot read kinematic URDF: {exc}") from exc
        by_child = {}
        links = {element.get("name") for element in root.findall("link")}
        for element in root.findall("joint"):
            kind, name = element.get("type"), element.get("name")
            parent_node, child_node = element.find("parent"), element.find("child")
            if not name or parent_node is None or child_node is None:
                raise ValueError("URDF joint is missing name, parent, or child")
            parent, child = parent_node.get("link"), child_node.get("link")
            if parent not in links or child not in links or child in by_child:
                raise ValueError(f"Invalid URDF link topology at {name}")
            origin_node = element.find("origin")
            xyz = _xml_vector(origin_node.get("xyz", "0 0 0") if origin_node is not None else "0 0 0", "origin xyz")
            rpy = _xml_vector(origin_node.get("rpy", "0 0 0") if origin_node is not None else "0 0 0", "origin rpy")
            origin = np.eye(4)
            origin[:3, 3] = xyz
            origin[:3, :3] = (_exp(np.array([0., 0., rpy[2]])) @
                               _exp(np.array([0., rpy[1], 0.])) @
                               _exp(np.array([rpy[0], 0., 0.])))
            axis_node, limit = element.find("axis"), element.find("limit")
            axis = _xml_vector(axis_node.get("xyz", "1 0 0") if axis_node is not None else "1 0 0", "axis")
            norm = float(np.linalg.norm(axis))
            if norm < 1e-12 or not math.isfinite(norm):
                raise ValueError(f"Invalid URDF axis at {name}")
            axis /= norm
            lower = upper = 0.
            if kind == "revolute":
                try:
                    lower, upper = float(limit.get("lower")), float(limit.get("upper"))
                except (TypeError, ValueError, AttributeError) as exc:
                    raise ValueError(f"Missing URDF limits at {name}") from exc
                if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                    raise ValueError(f"Invalid URDF limits at {name}")
            by_child[child] = _Joint(name, parent, child, kind, origin, axis, lower, upper)
        self._chains = {}
        self._limits = {}
        self._measurement_limits = {}
        for arm, tip in GRASPER_FRAMES.items():
            chain, child, visited = [], tip, set()
            while child != "base_Link":
                if child in visited or child not in by_child:
                    raise ValueError(f"No valid base_Link -> {tip} chain")
                visited.add(child)
                joint = by_child[child]
                if joint.kind not in ("fixed", "revolute"):
                    raise ValueError(f"Unsupported joint type in arm chain: {joint.kind}")
                chain.append(joint)
                child = joint.parent
            chain.reverse()
            moving = [joint for joint in chain if joint.kind == "revolute"]
            offset = 0 if arm == "left" else 7
            if tuple(j.name for j in moving) != tuple(JOINT_NAMES[offset:offset + 7]):
                raise ValueError(f"Unexpected 7-DoF joint mapping for {arm}")
            lower = np.maximum.reduce([ARM_LOWER[offset:offset+7],
                                       _CURRENT_MOVEJ_LOWER[offset:offset+7], [j.lower for j in moving]])
            upper = np.minimum.reduce([ARM_UPPER[offset:offset+7],
                                       _CURRENT_MOVEJ_UPPER[offset:offset+7], [j.upper for j in moving]])
            if np.any(lower >= upper):
                raise ValueError(f"Empty joint-limit intersection for {arm}")
            self._chains[arm] = tuple(chain)
            self._limits[arm] = (lower, upper)
            # Read actual/historical configurations throughout the model's
            # mechanical range even when today's command interface is narrower.
            self._measurement_limits[arm] = (np.array([j.lower for j in moving]),
                                             np.array([j.upper for j in moving]))

    def _measured(self, arm, q7):
        if arm not in self._chains:
            raise ValueError("arm must be 'left' or 'right'")
        q = _vector(q7, 7, "joints_rad")
        lower, upper = self._measurement_limits[arm]
        if np.any(q < lower) or np.any(q > upper):
            raise ValueError(f"{arm} feedback exceeds URDF mechanical limits")
        return q

    def _fk(self, arm, q, *, jacobian=True):
        transform, origins, axes, index = np.eye(4), [], [], 0
        for joint in self._chains[arm]:
            transform = transform @ joint.origin
            if joint.kind == "revolute":
                if jacobian:
                    origins.append(transform[:3, 3].copy())
                    axes.append(transform[:3, :3] @ joint.axis)
                motion = np.eye(4)
                motion[:3, :3] = _exp(joint.axis * q[index])
                transform = transform @ motion
                index += 1
        transform[:3, 3] += transform[:3, :3] @ GRASPER_TO_TCP_M
        if not jacobian:
            return transform
        position = transform[:3, 3]
        jacobian = np.column_stack([np.r_[np.cross(a, position - p), a]
                                    for p, a in zip(origins, axes)])
        return transform, jacobian

    def forward(self, arm, q7):
        """Return the vendor TCP xyz in metres and xyzw quaternion in base_Link."""
        q = self._measured(arm, q7)
        transform = self._fk(arm, q, jacobian=False)
        return transform[:3, 3].tolist() + _quaternion(transform[:3, :3]).tolist()

    def solve(self, arm, target_position, target_quaternion, seed7):
        """Return a joint command for the requested TCP or a feasible local prefix.

        Position and base-frame rotation advance by the same fraction. This is
        endpoint IK; joint interpolation between endpoints can curve in space.
        """
        measured = self._measured(arm, seed7)
        lower, upper = self._limits[arm]
        seed = np.clip(measured, lower, upper)
        if np.any(np.abs(seed - measured) > math.radians(1)):
            raise ValueError(f"{arm} feedback exceeds command limits by more than one degree")
        target_p = _vector(target_position, 3, "target_position_m")
        target_r = _rotation(target_quaternion)
        start = self._fk(arm, measured, jacobian=False)
        translation = target_p - start[:3, 3]
        rotation = _log(target_r @ start[:3, :3].T)
        local = (math.hypot(*translation) <= LOCAL_TRANSLATION_M + 1e-12
                 and math.hypot(*rotation) <= LOCAL_ROTATION_RAD + 1e-12)
        if local:
            lower = np.maximum(lower, measured - LOCAL_JOINT_RAD)
            upper = np.minimum(upper, measured + LOCAL_JOINT_RAD)
        # Two TCPs in the model workspace are separated by at most its diameter.
        # Bound the search interval even for targets many metres away, retaining
        # the requested translation/rotation ratio for every trial on the segment.
        fraction_limit = 1.
        if not local:
            radius = sum(math.hypot(*j.origin[:3, 3]) for j in self._chains[arm]) + math.hypot(*GRASPER_TO_TCP_M)
            scale = max(abs(translation))
            if scale > radius / 2:
                fraction_limit = min(1., (2 * radius / math.hypot(*(translation / scale))) / float(scale))
        if fraction_limit == 1.:
            solution = self._solve_pose(arm, target_p, target_r, seed, lower, upper)
            if solution is not None:
                return solution.tolist()
        low, high = 0., fraction_limit
        best = seed
        for _ in range(PREFIX_BISECTIONS):
            fraction = (low + high) / 2.
            pose_p = start[:3, 3] + fraction * translation
            pose_r = _exp(fraction * rotation) @ start[:3, :3]
            solution = self._solve_pose(arm, pose_p, pose_r, best, lower, upper)
            if solution is None:
                high = fraction
            else:
                best, low = solution, fraction
        if low and not np.array_equal(best, measured):
            return best.tolist()
        raise NoFeasibleStep(f"IK found no feasible command from the current {arm} arm configuration")

    def _solve_pose(self, arm, target_p, target_r, seed, lower, upper):
        """Bounded local IK. The residual only controls numerical accuracy."""
        q = seed
        weights = np.array([1., 1., 1., .2, .2, .2])

        def residual(configuration):
            transform, jacobian = self._fk(arm, configuration)
            error = np.r_[target_p - transform[:3, 3],
                          _log(target_r @ transform[:3, :3].T)]
            return error, jacobian

        error, jacobian = residual(q)
        initial = np.linalg.norm(weights * error)
        # Resolve small nonzero commands just as accurately as large ones.
        # The absolute floor covers floating-point noise in composed rotations.
        numerical_residual = max(1e-13, min(1e-8, initial * 1e-5))
        for _ in range(MAX_ITERATIONS):
            weighted_j = weights[:, None] * jacobian
            weighted_e = weights * error
            cost = float(weighted_e @ weighted_e)
            if cost <= numerical_residual**2:
                return q
            accepted = False
            # Local trust region, damping, and backtracking preserve the seed
            # neighbourhood without silently accepting an unconverged solution.
            for damping in (0.002, 0.01, 0.05):
                # At a bound, clipping an outward step repeatedly loses that
                # column's contribution while the other joints still assume
                # it moved. Re-solve the same Cartesian request on free axes.
                free = np.ones(len(q), dtype=bool)
                step = np.zeros_like(q)
                while np.any(free):
                    jac = weighted_j[:, free]
                    step[free] = jac.T @ np.linalg.solve(
                        jac @ jac.T + damping*damping*np.eye(6), weighted_e)
                    outward = free & (((q <= lower + 1e-10) & (step < 0)) |
                                      ((q >= upper - 1e-10) & (step > 0)))
                    if not np.any(outward):
                        break
                    step[outward] = 0.
                    free[outward] = False
                step *= min(1., .15 / max(float(np.max(np.abs(step))), 1e-15))
                for scale in (1., .5, .25, .125, .0625):
                    candidate = np.clip(q + scale * step, lower, upper)
                    candidate_error, candidate_j = residual(candidate)
                    weighted_candidate = weights * candidate_error
                    if float(weighted_candidate @ weighted_candidate) < cost * (1. - 1e-10):
                        q, error, jacobian = candidate, candidate_error, candidate_j
                        accepted = True
                        break
                if accepted:
                    break
            if not accepted:
                break
        return None
