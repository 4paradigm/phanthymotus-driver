"""Read-only G1_23 fixed-hand geometry and right-arm direction planning.

All directions use ``torso_link``: +X forward, +Y left, +Z up.  Joint order is
right shoulder pitch/roll/yaw, elbow, wrist roll (LowState slots 22..26).
Geometry comes from the bundled ``motion/models/body23.urdf`` and the original
fixed-hand TCP in ``motion/body23_fixed_hand.json``.  The indicating axis is
wrist origin -> TCP, not elbow -> wrist or an independently moving finger.
The wrist roll is included in FK; the original TCP lies on its rotation axis,
so rolling this particular hand does not change the indicating direction.

``cfg`` accepts ``angular_tolerance_deg`` (default 3) and ``verified_poses``:
    [{"name": "site-pose-id", "q_rad": [five measured joint angles],
      "verified": True, "validation_id": "external acceptance record",
      "model_sha256": "LF-normalized URDF SHA256"}]
The default solver selects a recorded pose within angular tolerance. The
optional bounded_ik solver searches deterministic roll/elbow seeds and all
analytical shoulder branches inside the original URDF hard limits. It uses the measured start's wrist roll because
that joint has no influence on the pointing direction of the fixed hand.
Feedback FK estimates direction in the model. External precision and scene
clearance are separate from this calculation.

Bounded IK accepts configured yaw_limits_deg and pitch_limits_deg intervals
inside [-180,180] and [-90,90] degrees, respectively. Bounded mode defaults
to these full spherical coordinates; acceptance still needs IK and a modeled
collision-free nominal path. Finite search failure is not a proof that no
physical joint configuration exists.
Its execution tolerance may be at most 5 degrees (default 3); analytic IK
continues to require a residual below .01 degree.

Cross-body checks require measured left_q5. Left URDF transforms and collision
mesh covers replace the former assumption that every right-arm part stays at
negative Y. A command-path proof holds the supplied left configuration fixed;
separate measured-pose checks consume simultaneous actual left/right feedback.
Neither check proves safety between feedback frames or during stopping.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


JOINT_NAMES = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)
MOTOR_INDICES = (22, 23, 24, 25, 26)
FRAME = "torso_link"
_MOTION = Path(__file__).resolve().parent / "motion"
_MAX_CERTIFICATE_NODES = 4095
_MAX_IK_PATH_CANDIDATES = 32
_MAX_IK_TOTAL_CERTIFICATE_NODES = 4095

# Outward-rounded millimetre bounds from Unitree's g1_description collision
# meshes (unitree_ros 157ff950351e5f535e03c5d85a16b7d594b3dbcc).
# Each mesh triangle contributes to every slice it intersects. The shoulder
# roll cover is the cylinder specified in the bundled 23-DOF URDF.
_ARM_BOXES_MM = (
    ((-26,-70,-40, 26,-10,20),),
    ((-34,-36,-68, 26,24,-38),),
    ((-32,-23,-114, 49,42,-60), (-33,-31,-60, 46,42,-20), (-33,-34,-20, 33,33,32)),
    ((-30,-35,-36, 10,-20,30), (10,-35,-46, 53,32,29), (53,-35,-46, 101,32,25)),
    ((0,-30,-31, 60,31,31), (60,-29,-26, 125,26,29), (125,-25,-37, 190,15,62),
     (190,-20,-42, 254,43,65)),
)
# Torso slices, fixed head and chest logo, pelvis (yaw-independent box)
# and a standing lower-body keep-out region. Coordinates are in torso_link.
_BODY_BOXES_MM = (
    (-63,-97,-10, 73,97,70), (-66,-103,70, 81,103,150),
    (-67,-108,150, 84,108,220), (-67,-108,220, 79,108,301),
    (-63,-78,270, 79,78,477), (-69,-75,203, 81,75,233),
    (-86,-86,-203, 86,86,-51),
)
_BODY_BOXES = tuple(tuple(n/1000 for n in box) for box in _BODY_BOXES_MM)
_ARM_BOXES = tuple(tuple((tuple((box[i] + box[i+3])/2000 for i in range(3)),
                          tuple((box[i+3]-box[i])/2000 for i in range(3)))
                         for box in link) for link in _ARM_BOXES_MM)

# Left mesh boxes are independently derived from the pinned upstream STL
# triangles, not mirrored right boxes. See directional_geometry/point-left-collision-cover.json and
# directional_geometry/build_left_collision_boxes.py for triangle/slab coverage evidence.
# Pitch/roll are outward-rounded covers of the original URDF cylinders.
_LEFT_ARM_BOXES_MM = (((-26, 10, -40, 26, 70, 20),), ((-34, -24, -68, 26, 36, -38),), ((-32, -42, -114, 49, 23, -60), (-33, -42, -60, 45, 31, -20), (-33, -33, -20, 33, 34, 32)), ((-30, 20, -36, 10, 35, 30), (10, -32, -46, 53, 35, 29), (53, -32, -46, 101, 35, 25)), ((0, -31, -31, 60, 30, 31), (60, -26, -26, 125, 29, 28), (125, -15, -37, 190, 25, 62), (190, -43, -42, 254, 20, 65)))
_LEFT_MESH_SHA256 = {'left_shoulder_yaw_link.STL': '1aa97e9748e924336567992181f78c7cd0652fd52a4afcca3df6b2ef6f9e712e', 'left_elbow_link.STL': 'fa752198accd104d5c4c3a01382e45165b944fbbc5acce085059223324e5bed3', 'left_wrist_roll_rubber_hand.STL': 'e81030abd023bd9e4a308ef376d814a2c12d684d8a7670c335bbd5cd7809c909'}
_LEFT_ARM_BOXES = tuple(tuple((tuple((box[i]+box[i+3])/2000 for i in range(3)),
                             tuple((box[i+3]-box[i])/2000 for i in range(3)))
                            for box in link) for link in _LEFT_ARM_BOXES_MM)

def _outward_nonnegative(value):
    """Next larger binary64 value, including zero; compatible with Python 3.8."""
    bits = struct.unpack('>Q',struct.pack('>d',max(0.,value)))[0]
    return struct.unpack('>d',struct.pack('>Q',bits+1))[0]


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}: expected a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{label}: expected a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label}: expected a finite number")
    return result


def _vector(value, length, label):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) != length):
        raise ValueError(f"{label}: expected {length} finite numbers")
    return tuple(_number(x, label) for x in value)


def normalize_direction(value):
    """Validate and normalize a nonzero three-vector, without overflow."""
    vector = _vector(value, 3, "direction")
    scale = max(map(abs, vector))
    if scale == 0:
        raise ValueError("direction: zero vector")
    scaled = tuple(x / scale for x in vector)
    norm = math.hypot(*scaled)
    return [x / norm for x in scaled]


def angular_error_deg(actual, target):
    """Spherical angular distance, including across the +/-180 degree seam."""
    a, b = normalize_direction(actual), normalize_direction(target)
    cross = (a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0])
    return math.degrees(math.atan2(math.hypot(*cross), sum(x * y for x, y in zip(a, b))))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _rotate(matrix, vector):
    return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)


def _dot(a, b):
    return sum(x*y for x,y in zip(a,b))


def _cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def _axis_rotation(axis, angle):
    x, y, z = normalize_direction(axis)
    c, s = math.cos(angle), math.sin(angle)
    d = 1 - c
    return ((c + x*x*d, x*y*d - z*s, x*z*d + y*s),
            (y*x*d + z*s, c + y*y*d, y*z*d - x*s),
            (z*x*d - y*s, z*y*d + x*s, c + z*z*d))


def _rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    # URDF extrinsic fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    return _matmul(_matmul(_axis_rotation((0, 0, 1), yaw),
                           _axis_rotation((0, 1, 0), pitch)),
                   _axis_rotation((1, 0, 0), roll))


def _xml_vector(element, attribute, default):
    text = default if element is None else element.get(attribute, default)
    try:
        return _vector([float(x) for x in text.split()], 3, f"URDF {attribute}")
    except (ValueError, TypeError) as exc:
        raise ValueError(f"directional_model_invalid: {attribute}") from exc


@dataclass(frozen=True)
class _Joint:
    origin: tuple
    rotation: tuple
    axis: tuple
    lower: float
    upper: float


@dataclass(frozen=True)
class _Pose:
    name: str
    q_rad: tuple
    direction: tuple
    validation_id: str


class DirectionalKinematics:
    """Five-joint FK, recorded poses and bounded full-direction IK search."""

    frame = FRAME
    joint_names = JOINT_NAMES
    motor_indices = MOTOR_INDICES
    yaw_limits_deg = (-20.0, 20.0)
    pitch_limits_deg = (-5.0, 25.0)
    _supported_yaw_limits_deg = (-180.0, 180.0)
    _supported_pitch_limits_deg = (-90.0, 90.0)

    def __init__(self, cfg=None):
        if cfg is None:
            cfg = {}
        if not isinstance(cfg, Mapping):
            raise ValueError("directional_config: expected a mapping")
        self.angular_tolerance_deg = _number(cfg.get("angular_tolerance_deg", 3.0),
                                             "angular_tolerance_deg")
        if not 0 < self.angular_tolerance_deg < 90:
            raise ValueError("angular_tolerance_deg: expected 0 < tolerance < 90")
        self._load_model()
        self.solver_mode = cfg.get("solver_mode", "verified_poses")
        if self.solver_mode not in ("verified_poses", "bounded_ik"):
            raise ValueError("directional_unknown_solver_mode")
        if self.solver_mode == "bounded_ik":
            if cfg.get("verified_poses") or "commissioning_trial" in cfg:
                raise ValueError("directional_ambiguous_solver_config")
            if self.angular_tolerance_deg > 5.:
                raise ValueError("directional_ik_tolerance_exceeds_5_deg")
            for name,supported in (("yaw_limits_deg",self._supported_yaw_limits_deg),
                                   ("pitch_limits_deg",self._supported_pitch_limits_deg)):
                interval = _vector(cfg.get(name,supported),2,name)
                if not supported[0] <= interval[0] < interval[1] <= supported[1]:
                    raise ValueError("directional_unsupported_" + name)
                setattr(self,name,interval)
            if self.model_sha256 != "b1af86fb023c0b6f8e52723d224be6cad70916eaff2778e7dbe09e6f91faa9b9":
                raise ValueError("directional_clearance_model_mismatch")
            self._ik_reference = self._joint_angles(cfg.get("ik_reference_q_rad"))
            self._ik_start = self._joint_angles(cfg.get("verified_start_q_rad"))
        raw_poses = cfg.get("verified_poses", [])
        if not isinstance(raw_poses, (list, tuple)):
            raise ValueError("verified_poses: expected a list")
        poses, names = [], set()
        for record in raw_poses:
            if not isinstance(record, Mapping) or record.get("verified") is not True:
                raise ValueError("directional_pose_unverified")
            name, validation = record.get("name"), record.get("validation_id")
            if not isinstance(name, str) or not name.strip() or name in names:
                raise ValueError("directional_pose_name_missing_or_duplicate")
            if not isinstance(validation, str) or not validation.strip():
                raise ValueError("directional_validation_id_missing")
            if record.get("model_sha256") != self.model_sha256:
                raise ValueError("directional_pose_model_mismatch")
            q = self._joint_angles(record.get("q_rad"))
            poses.append(_Pose(name, q, tuple(self.direction(q)), validation))
            names.add(name)
        self._poses = tuple(poses)

    def _load_model(self):
        # Universal-newline reading makes the identity stable across Git's
        # Windows CRLF checkout and the LF deployment image.
        model_text = (_MOTION / "models" / "body23.urdf").read_text(encoding="utf-8")
        self.model_sha256 = hashlib.sha256(model_text.encode("utf-8")).hexdigest()
        profile = json.loads((_MOTION / "body23_fixed_hand.json").read_text(encoding="utf-8"))
        if (profile.get("urdf_sha256") != self.model_sha256
                or profile.get("arm_joint_names", [])[5:] != list(JOINT_NAMES)):
            raise ValueError("directional_model_profile_mismatch")
        tcp = _vector(profile["palm_frames"]["right"]["position"], 3, "fixed-hand TCP")
        if tcp != (0.2, 0.0, 0.0):
            raise ValueError("directional_fixed_hand_tcp_changed")
        self._tcp = tcp
        if (profile.get('arm_joint_names',[])[:5] != [name.replace('right_','left_',1) for name in JOINT_NAMES]
                or _vector(profile['palm_frames']['left']['position'],3,'left fixed-hand TCP') != (.2,0.,0.)):
            raise ValueError('directional_left_model_profile_mismatch')
        root = ET.fromstring(model_text)
        joints = {joint.get("name"): joint for joint in root.findall("joint")}
        chain, parent = [], FRAME
        for name in JOINT_NAMES:
            joint = joints.get(name)
            if (joint is None or joint.get("type") != "revolute"
                    or joint.find("parent").get("link") != parent):
                raise ValueError("directional_model_chain_mismatch")
            origin, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
            if axis is None or limit is None:
                raise ValueError("directional_model_joint_metadata_missing")
            lower, upper = (_number(float(limit.get(key)), f"URDF {key}")
                            for key in ("lower", "upper"))
            if lower >= upper:
                raise ValueError("directional_model_joint_limit_invalid")
            chain.append(_Joint(_xml_vector(origin, "xyz", "0 0 0"),
                                _rpy_rotation(_xml_vector(origin, "rpy", "0 0 0")),
                                tuple(normalize_direction(_xml_vector(axis, "xyz", "1 0 0"))),
                                lower, upper))
            parent = joint.find("child").get("link")
        if parent != "right_wrist_roll_rubber_hand":
            raise ValueError("directional_model_not_fixed_hand")
        self._chain = tuple(chain)
        left_chain, parent = [], FRAME
        for name in (name.replace('right_','left_',1) for name in JOINT_NAMES):
            joint = joints.get(name)
            if (joint is None or joint.get('type') != 'revolute'
                    or joint.find('parent').get('link') != parent):
                raise ValueError('directional_left_model_chain_mismatch')
            origin, axis, limit = joint.find('origin'),joint.find('axis'),joint.find('limit')
            lower,upper = (float(limit.get(key)) for key in ('lower','upper'))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError('directional_left_model_joint_limit_invalid')
            left_chain.append(_Joint(_xml_vector(origin,'xyz','0 0 0'),
                _rpy_rotation(_xml_vector(origin,'rpy','0 0 0')),
                tuple(normalize_direction(_xml_vector(axis,'xyz','1 0 0'))),lower,upper))
            parent = joint.find('child').get('link')
        if parent != 'left_wrist_roll_rubber_hand':
            raise ValueError('directional_left_model_not_fixed_hand')
        self._left_chain = tuple(left_chain)
        self._box_corners = tuple(tuple(tuple(
            tuple(center[i] + sign[i]*half[i] for i in range(3))
            for sign in itertools.product((-1, 1), repeat=3))
            for center,half in boxes) for boxes in _ARM_BOXES)
        self._box_rotation_radii = self._collision_box_rotation_radii()
        # Retained as a diagnostic summary; path certification uses each box.
        self._rotation_radii = tuple(max(row[i] for link in self._box_rotation_radii
                                         for row in link) for i in range(5))

    def _collision_box_rotation_radii(self):
        """Bound point travel separately for every box and upstream joint.

        The first downstream joint offset contributes only its distance to the
        current axis. Further offsets and box corners contribute their full
        length, allowing every downstream joint angle. A joint downstream of a
        box cannot move that box. For its own joint only the radius around the
        joint axis contributes. Thus changing joint j by d moves every point
        of a box by at most R[box,j]*abs(d), for all intermediate joint angles.
        """
        radii = []
        for link, boxes in enumerate(_ARM_BOXES):
            link_radii = []
            for center, half in boxes:
                corners = tuple(tuple(center[i] + sign[i]*half[i] for i in range(3))
                                for sign in itertools.product((-1, 1), repeat=3))
                row = []
                for index, joint in enumerate(self._chain):
                    if index > link:
                        radius = 0.0
                    elif link == index:
                        radius = max(math.hypot(*_cross(point, joint.axis)) for point in corners)
                    else:
                        radius = math.hypot(*_cross(self._chain[index+1].origin, joint.axis))
                        radius += sum(math.hypot(*self._chain[i].origin) for i in range(index+2, link+1))
                        radius += max(math.hypot(*point) for point in corners)
                    row.append(radius)
                link_radii.append(tuple(row))
            radii.append(tuple(link_radii))
        return tuple(radii)

    def _joint_angles(self, q5):
        q = _vector(q5, 5, "q_rad")
        for name, joint, angle in zip(JOINT_NAMES, self._chain, q):
            if not joint.lower <= angle <= joint.upper:
                raise ValueError(f"directional_joint_limit: {name}")
        return q

    def _left_angles(self, q5):
        if q5 is None:
            raise ValueError('directional_left_feedback_required')
        q = _vector(q5,5,'left_q_rad')
        for name,joint,angle in zip(JOINT_NAMES,self._left_chain,q):
            if not joint.lower <= angle <= joint.upper:
                raise ValueError('directional_joint_limit: '+name.replace('right_','left_',1))
        return q

    @staticmethod
    def _chain_frames(chain, q):
        rotation = ((1.0,0.0,0.0),(0.0,1.0,0.0),(0.0,0.0,1.0))
        position = (0.0,0.0,0.0)
        for joint,angle in zip(chain,q):
            offset = _rotate(rotation,joint.origin)
            position = tuple(a+b for a,b in zip(position,offset))
            rotation = _matmul(_matmul(rotation,joint.rotation),_axis_rotation(joint.axis,angle))
            yield position,rotation

    def _link_frames(self, q):
        return self._chain_frames(self._chain,q)

    def _left_obstacles(self, left_q5):
        q = self._left_angles(left_q5)
        result = []
        for (origin,rotation),boxes in zip(self._chain_frames(self._left_chain,q),_LEFT_ARM_BOXES):
            absolute = tuple(tuple(abs(value) for value in row) for row in rotation)
            for center,half in boxes:
                c = tuple(a+b for a,b in zip(origin,_rotate(rotation,center)))
                h = _rotate(absolute,half)
                result.append(tuple(a-b-1e-12 for a,b in zip(c,h))+tuple(a+b+1e-12 for a,b in zip(c,h)))
        return tuple(result)

    def forward(self, q5):
        """Return wrist/TCP positions (metres) and their unit axis in torso_link."""
        position, rotation = tuple(self._link_frames(self._joint_angles(q5)))[-1]
        offset = _rotate(rotation, self._tcp)
        return {"frame": FRAME, "wrist_position_m": list(position),
                "tcp_position_m": [a + b for a, b in zip(position, offset)],
                "direction": normalize_direction(offset)}

    def direction(self, q5):
        """Estimate wrist-to-TCP direction from five angles, including wrist roll."""
        return self.forward(q5)["direction"]

    def evaluate(self, q5, target):
        """Evaluate measured joint angles; this is model-derived feedback."""
        actual = self.direction(q5)
        return {"frame": FRAME, "direction": actual,
                "angular_error_deg": angular_error_deg(actual, target)}

    def check_measured_pose(self, q5, *, left_q5=None):
        """Read-only check of one measured pose; return model clearance in m.

        This checks current URDF hard limits and the unchanged 3 mm modeled
        keep-out margin. It does not compare against a command, run IK, search
        a path, or certify motion between feedback samples or stopping distance.
        Both arms use the same consumed feedback sample: actual left boxes replace
        the former whole-right-halfspace rule. Feedback freshness, mode, and
        stop handling belong to the caller.
        """
        q = self._joint_angles(q5)
        clearance = self._clearance(q,left_bounds=self._left_obstacles(left_q5))
        if clearance < .003:
            raise ValueError("directional_measured_body_clearance")
        return clearance

    def solve(self, direction, start_q5=None, *, left_q5=None):
        """Return the target joints accepted by the configured solver."""
        target = normalize_direction(direction)
        if self.solver_mode == "bounded_ik":
            start = self._ik_start if start_q5 is None else self._joint_angles(start_q5)
            return self._solve_bounded(target, start, left_q5)
        if not self._poses:
            raise ValueError("directional_no_verified_poses")
        error, _, pose = min((angular_error_deg(p.direction, target), i, p)
                             for i, p in enumerate(self._poses))
        if error > self.angular_tolerance_deg:
            raise ValueError("directional_outside_verified_coverage")
        return list(pose.q_rad)

    def _solve_bounded(self, target, start, left_q5):
        yaw = math.degrees(math.atan2(target[1], target[0]))
        pitch = math.degrees(math.atan2(target[2], math.hypot(*target[:2])))
        if any(value < low-1e-9 or value > high+1e-9 for value, (low, high) in
               ((yaw, self.yaw_limits_deg), (pitch, self.pitch_limits_deg))):
            raise ValueError("directional_outside_ik_sector")
        # A complete public direction range is not a promise of global IK.
        # Keep the measured wrist roll, enumerate a finite roll/elbow grid,
        # and test both analytical shoulder branches inside URDF limits.
        left_bounds = self._left_obstacles(left_q5)
        self.check_measured_pose(start,left_q5=left_q5)
        candidates = list(self._direction_candidates(target,start,left_bounds=left_bounds))
        if not candidates:
            raise ValueError("directional_no_feasible_solution")
        candidates.sort(key=lambda q: (max(abs(a-b) for a,b in zip(q,start)),
                                        sum((a-b)**2 for a,b in zip(q,start)), tuple(q)))
        remaining_nodes = [_MAX_IK_TOTAL_CERTIFICATE_NODES]
        for q in candidates[:_MAX_IK_PATH_CANDIDATES]:
            try:
                self.check_path(start,q,left_q5=left_q5,_node_budget=remaining_nodes)
            except ValueError as exc:
                if str(exc) not in ("directional_body_clearance", "directional_geometry_budget_exceeded"):
                    raise
                if remaining_nodes[0] <= 0:
                    break
            else:
                return list(q)
        raise ValueError("directional_no_feasible_path")

    @staticmethod
    def _periodic_angles(angle, joint):
        """Every 2*pi equivalent inside this finite URDF joint interval."""
        first = math.ceil((joint.lower-angle)/(2*math.pi))
        last = math.floor((joint.upper-angle)/(2*math.pi))
        return tuple(value for turn in range(first,last+1)
                     for value in (angle+turn*2*math.pi,) if joint.lower <= value <= joint.upper)

    def _direction_candidates(self, target, start, *, left_bounds=None):
        """At most 12 roll * 11 elbow seeds * two analytical yaw branches.

        Grid endpoints are URDF limits; measured/reference seeds retain useful
        local solutions. No random restarts or unbounded iterative optimizer.
        The wrist is retained from measured start because it does not affect
        this fixed hand's pointing axis. Geometry is evaluated for that wrist.
        """
        def seeds(index, special):
            joint = self._chain[index]
            values = tuple(special)+(joint.lower,)+tuple(
                joint.lower+(joint.upper-joint.lower)*i/8 for i in range(1,8))+(joint.upper,)
            return tuple(dict.fromkeys(value for value in values if joint.lower <= value <= joint.upper))

        rolls = seeds(1, (start[1], min(start[1],-.40), self._ik_reference[1]))
        elbows = seeds(3, (start[3], self._ik_reference[3]))
        axis = _rotate(self._chain[0].rotation, self._chain[0].axis)
        along = _dot(axis,target)
        seen = set()
        for roll,elbow in itertools.product(rolls,elbows):
            q = [0.,roll,0.,elbow,start[4]]
            def projected(yaw):
                q[2] = yaw
                return _dot(axis,self.direction(q))
            left,right = projected(math.pi/2),projected(-math.pi/2)
            c = (left+right)/2
            a,b = projected(0.)-c,(left-right)/2
            size = math.hypot(a,b)
            if size < 1e-10:
                if abs(along-c) > 1e-10:
                    continue
                # Degenerate projection: retain the measured shoulder yaw.
                yaw_branches = (start[2],)
            elif abs(along-c) > size+1e-10:
                continue
            else:
                offset = math.acos(max(-1.,min(1.,(along-c)/size)))
                phase = math.atan2(b,a)
                yaw_branches = (phase-offset,phase+offset)
            for branch in yaw_branches:
                for yaw in self._periodic_angles(branch,self._chain[2]):
                    q[0],q[2] = 0.,yaw
                    unpitched = self.direction(q)
                    cross = _dot(axis,_cross(unpitched,target))
                    dot = _dot(unpitched,target)-_dot(axis,unpitched)*along
                    pitch = start[0] if math.hypot(cross,dot) < 1e-12 else math.atan2(cross,dot)
                    for shoulder_pitch in self._periodic_angles(pitch,self._chain[0]):
                        q[0] = shoulder_pitch
                        key = tuple(round(value,12) for value in q)
                        if key in seen:
                            continue
                        seen.add(key)
                        self._joint_angles(q)
                        if (angular_error_deg(self.direction(q),target) < min(self.angular_tolerance_deg,.01)
                                and self._clearance(q,left_bounds=left_bounds) >= .003):
                            yield tuple(q)

    def _collision_bounds(self, q, joint_uncertainty=None):
        """AABBs enclosing each collision box over a whole joint-angle box.

        For each corner, J_j = world_axis_j cross (point - world_origin_j).
        Its coordinate-wise first-order variation is sum(abs(J_axis,j)*d_j).
        In this serial revolute chain, every mixed second derivative has norm
        at most R_box,max(i,j), at all intermediate joint configurations. Thus
        .5*sum_ij(R_box,max(i,j)*d_i*d_j) bounds the Taylor remainder in every
        coordinate. The sum includes both off-diagonal terms. Adding this to
        each corner's linear interval and taking their union encloses the whole
        rigid box: at each configuration its points form the corners' convex
        hull. A 1e-12 metre outward allowance covers arithmetic roundoff.

        With no uncertainty, retain the nominal geometry used for diagnostics.
        """
        frames = tuple(self._link_frames(q))
        axes = tuple(_rotate(rotation,joint.axis) for (_,rotation),joint in zip(frames,self._chain))
        for link, ((origin, rotation), boxes) in enumerate(zip(frames, _ARM_BOXES)):
            absolute = tuple(tuple(abs(x) for x in row) for row in rotation)
            for index, (center, half_size) in enumerate(boxes):
                if joint_uncertainty is None:
                    c = tuple(a+b for a,b in zip(origin, _rotate(rotation, center)))
                    h = _rotate(absolute, half_size)
                    yield tuple(a-b for a,b in zip(c,h)) + tuple(a+b for a,b in zip(c,h))
                    continue
                radii = self._box_rotation_radii[link][index]
                remainder = .5*sum(radii[max(i,j)]*joint_uncertainty[i]*joint_uncertainty[j]
                                    for i in range(link+1) for j in range(link+1)) + 1e-12
                lower,upper = [math.inf]*3,[-math.inf]*3
                for corner in self._box_corners[link][index]:
                    point = tuple(a+b for a,b in zip(origin,_rotate(rotation,corner)))
                    jacobian = tuple(_cross(axes[j],tuple(a-b for a,b in zip(point,frames[j][0])))
                                     for j in range(link+1))
                    for axis in range(3):
                        width = sum(abs(jacobian[j][axis])*joint_uncertainty[j]
                                    for j in range(link+1)) + remainder
                        lower[axis] = min(lower[axis],point[axis]-width)
                        upper[axis] = max(upper[axis],point[axis]+width)
                yield tuple(lower)+tuple(upper)

    def _clearance(self, q, joint_uncertainty=None, *, left_bounds=None):
        return self._clearance_with_box(q,joint_uncertainty,left_bounds=left_bounds)[0]

    def _clearance_with_box(self, q, joint_uncertainty=None, *, left_bounds=None):
        clearance = math.inf
        limiting_box = None
        identities = ((link,index) for link,boxes in enumerate(_ARM_BOXES)
                      for index in range(len(boxes)))
        for identity,(x,y,z,X,Y,Z) in zip(identities,self._collision_bounds(q,joint_uncertainty)):
            distance = math.inf
            # The shoulder-pitch mounting part retains its original torso
            # attachment exemption, but it does participate in arm/arm checks.
            if identity[0] != 0:
                radius = math.hypot(max(0.,x,-X),max(0.,y,-Y))
                distance = math.hypot(max(0.,radius-.18),max(0.,z+.115))
                for l,b,d,r,t,u in _BODY_BOXES:
                    distance = min(distance,math.hypot(max(0.,l-X,x-r),max(0.,b-Y,y-t),max(0.,d-Z,z-u)))
            for l,b,d,r,t,u in (() if left_bounds is None else left_bounds):
                distance = min(distance,math.hypot(max(0.,l-X,x-r),max(0.,b-Y,y-t),max(0.,d-Z,z-u)))
            if distance < clearance:
                clearance,limiting_box = distance,identity
        return clearance,limiting_box

    def check_path(self, start_q5, target_q5, tracking_tolerance_q_rad=None, *, left_q5=None, _node_budget=None):
        """Certify the right-arm joint-linear path and optional tracking tube.

        Omitting tracking_tolerance_q_rad checks the nominal command path only;
        it makes no statement about how closely the physical arm follows it.
        Each measured joint must remain within its supplied tolerance of the
        same point on this trajectory. The execution owner enforces that
        condition. Each collision box is expanded to cover allowed joint error
        and all motion between path samples, then must retain 3 mm clearance.
        The proof covers a six-dimensional domain: path fraction [0,1] and five
        independent tracking-error intervals. A cell maps into a joint box with
        center start+fraction*span+error and halfwidth abs(span)*fraction_radius
        +error_radius. Splitting any domain coordinate covers its entire parent
        by two closed cells, including time-varying errors and their boundary.
        All cells must be proved within a hard 4095-node budget; IK callers also
        share a single node budget across candidate paths. Exhausting either
        rejects the request. No result is inferred from sparse sampling. The
        complete original tube must fit URDF limits. Left-arm boxes are fixed at the supplied measured left_q5 during this
        nominal certificate. Actual moving left-arm feedback is checked by the
        caller separately; this is not a drifting-left or stopping-distance
        certificate. External environment remains a workcell constraint.
        """
        start = self._joint_angles(start_q5)
        target = self._joint_angles(target_q5)
        left_bounds = self._left_obstacles(left_q5)
        tolerance = ((0.0,)*5 if tracking_tolerance_q_rad is None else
                     _vector(tracking_tolerance_q_rad, 5, "tracking_tolerance_q_rad"))
        if any(value < 0 for value in tolerance):
            raise ValueError("directional_negative_tracking_tolerance")
        for joint, a, b, error in zip(self._chain, start, target, tolerance):
            if min(a,b)-error < joint.lower or max(a,b)+error > joint.upper:
                raise ValueError("directional_tracking_joint_limit")
        span = tuple(b-a for a,b in zip(start,target))
        minimum = math.inf
        pending = [((0.,)+tuple(-x for x in tolerance),(1.,)+tolerance)]
        visited = 0
        while pending:
            if (visited >= _MAX_CERTIFICATE_NODES
                    or (_node_budget is not None and _node_budget[0] <= 0)):
                raise ValueError("directional_geometry_budget_exceeded")
            if _node_budget is not None:
                _node_budget[0] -= 1
            lower,upper = pending.pop()
            visited += 1
            center = tuple((a+b)/2 for a,b in zip(lower,upper))
            radius = tuple(_outward_nonnegative(max(c-a,b-c))
                           for a,b,c in zip(lower,upper,center))
            q = tuple(a+d*center[0]+e for a,d,e in zip(start,span,center[1:]))
            uncertainty = tuple(_outward_nonnegative(abs(d)*radius[0]+error)
                                for d,error in zip(span,radius[1:]))
            clearance,box = self._clearance_with_box(q,uncertainty,left_bounds=left_bounds)
            if clearance >= .003:
                minimum = min(minimum,clearance)
                continue
            if self._clearance(q,left_bounds=left_bounds) < .003:
                raise ValueError("directional_body_clearance")
            # The selection changes proof efficiency, never the certified set.
            weights = self._box_rotation_radii[box[0]][box[1]]
            contributions = (sum(r*abs(d)*radius[0] for r,d in zip(weights,span)),) + tuple(
                r*error for r,error in zip(weights,radius[1:]))
            axis = max(range(6),key=contributions.__getitem__)
            midpoint = center[axis]
            if not lower[axis] < midpoint < upper[axis]:
                raise ValueError("directional_geometry_budget_exceeded")
            left_upper,right_lower = list(upper),list(lower)
            left_upper[axis] = right_lower[axis] = midpoint
            pending.extend(((lower,tuple(left_upper)),(tuple(right_lower),upper)))
        return minimum
