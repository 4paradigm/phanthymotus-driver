"""Kinematic tree maths for the `tf_tree` card.

This module is deliberately dependency-free: no `rclpy`, no `numpy`, no
`urdf_parser_py`. Everything is a pure function over plain numbers and tuples,
so the rules that would otherwise fail silently — a sign flip in a joint axis, a
URDF with two roots, a vision transform attached to a frame the tree does not
know — are all pinned by tests that run on a laptop.

The single decision that matters most lives in `forward_kinematics`: the arm
chain's joints are **not** rpy-zero. `l_arm_pitch_joint` is mounted at
rpy=+0.2618, `l_arm_roll_joint` at -0.2618 and `l_arm_yaw_joint` at +0.087266
(the right arm is mirrored). Summing joint `origin` xyz therefore does **not**
recover the real pose — the rotations compose. The golden values in the test
file are computed with the rotation-composing implementation below; a naive
xyz-sum fails them by a visible centimetre.

Frames and conventions
----------------------
Every link pose is expressed relative to the root (`base_link`). A pose is a
pair `(xyz, quat)` where `xyz` is a 3-tuple of metres and `quat` is a 4-tuple
`(x, y, z, w)` — the ROS `geometry_msgs/Quaternion` order. The URDF `origin`
rpy is interpreted as fixed-axis roll-pitch-yaw about X-Y-Z, which is the
convention `robot_state_publisher` and every URDF toolchain use.
"""

from __future__ import annotations

import math
import re
from xml.etree import ElementTree

# ── small matrix helpers (3x3 matrices as tuples of tuples) ──────────────────


def mat_mul(a, b):
    """3x3 @ 3x3."""
    return tuple(
        tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3))
        for r in range(3)
    )


def mat_vec(m, v):
    """3x3 @ 3-vector."""
    return tuple(sum(m[r][k] * v[k] for k in range(3)) for r in range(3))


def _identity():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rpy_to_matrix(rpy):
    """Fixed-axis roll-pitch-yaw (X-Y-Z) → rotation matrix.

    This is the URDF `<origin rpy>` convention. `rpy` is a 3-tuple of radians.
    """
    roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return mat_mul(mat_mul(_rot_z(yaw), _rot_y(pitch)), _rot_x(roll))


def axis_angle_matrix(axis, q):
    """Rotation about an arbitrary unit axis by angle `q` (Rodrigues)."""
    x, y, z = (float(axis[0]), float(axis[1]), float(axis[2]))
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-12:
        raise ValueError("zero-length joint axis")
    x, y, z = x / length, y / length, z / length
    c, s = math.cos(q), math.sin(q)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def quat_from_matrix(m):
    """Rotation matrix → unit quaternion `(x, y, z, w)`.

    Uses the numerically stable branch on the largest diagonal element
    (Shepperd's method). The matrix is assumed orthonormal.
    """
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(max(0.0, 1.0 + m[0][0] - m[1][1] - m[2][2])) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(max(0.0, 1.0 + m[1][1] - m[0][0] - m[2][2])) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + m[2][2] - m[0][0] - m[1][1])) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    # Normalise so accumulated rounding never drifts the quaternion off the unit sphere.
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


# ── URDF parsing ──────────────────────────────────────────────────────────────

_UNKNOWN_JOINT_ERROR = "unknown joint type"


def parse_urdf(text):
    """Parse a URDF into a link/joint topology.

    Returns ``{"links": {...}, "joints": {...}, "root": name}`` where each link
    is a bare marker and each joint is a dict with ``parent``, ``child``,
    ``type``, ``origin`` (an ``(xyz, rpy)`` pair), and ``axis`` (a 3-tuple for
    revolute joints, ``None`` otherwise).

    Rejects the inputs that would otherwise build a broken tree:
      * more than one root (a link no joint declares as a child);
      * a cycle in the parent/child graph;
      * a joint type this card does not implement (anything but ``fixed`` and
        ``revolute`` — the URDF has no prismatic/floating/planar joint, so a
        stray one is a real error, never something to silently treat as revolute);
      * a duplicated link or joint name;
      * a joint referencing an unknown link.

    Raises ``ValueError`` with a message naming the specific problem.
    """
    try:
        root = ElementTree.fromstring(text)
    except ElementTree.ParseError as exc:
        raise ValueError(f"URDF is not well-formed XML: {exc}")

    links = {}
    joints = {}

    for link_el in root.findall("link"):
        name = link_el.get("name")
        if not name:
            raise ValueError("a <link> is missing its name attribute")
        if name in links:
            raise ValueError(f"duplicate link name: {name}")
        links[name] = {}

    # Which link each joint points *to* (the child), for root detection and cycle checks.
    child_of = {}

    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        jtype = joint_el.get("type")
        if not name:
            raise ValueError("a <joint> is missing its name attribute")
        if not jtype:
            raise ValueError(f"joint '{name}' is missing its type attribute")
        if name in joints:
            raise ValueError(f"duplicate joint name: {name}")
        parent_el = joint_el.find("parent")
        child_el = joint_el.find("child")
        if parent_el is None or child_el is None:
            raise ValueError(f"joint '{name}' is missing <parent> or <child>")
        parent = parent_el.get("link")
        child = child_el.get("link")
        if not parent or not child:
            raise ValueError(f"joint '{name}' has an empty parent or child")
        if parent not in links:
            raise ValueError(f"joint '{name}' references unknown parent '{parent}'")
        if child not in links:
            raise ValueError(f"joint '{name}' references unknown child '{child}'")

        origin_el = joint_el.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin_el is not None:
            xyz = _parse_vec(origin_el.get("xyz"), f"joint '{name}' origin xyz")
            rpy = _parse_vec(origin_el.get("rpy"), f"joint '{name}' origin rpy")

        axis = None
        if jtype == "revolute":
            axis_el = joint_el.find("axis")
            if axis_el is None:
                raise ValueError(f"revolute joint '{name}' is missing <axis>")
            axis = _parse_vec(axis_el.get("xyz"), f"joint '{name}' axis")
        elif jtype != "fixed":
            raise ValueError(
                f"joint '{name}' has type '{jtype}', which this card does not "
                f"implement (only fixed and revolute are supported)")

        joints[name] = {
            "parent": parent,
            "child": child,
            "type": jtype,
            "origin": (xyz, rpy),
            "axis": axis,
        }
        child_of[child] = name

    if not links:
        raise ValueError("URDF has no links")

    # Root: a link that is never a child.
    roots = [name for name in links if name not in child_of]
    if len(roots) != 1:
        raise ValueError(
            f"expected exactly one root link, found {len(roots)}: {sorted(roots)}")

    _reject_cycle(links, joints, roots[0])

    return {"links": links, "joints": joints, "root": roots[0]}


def _parse_vec(text, what):
    """Parse a whitespace-separated 3-vector, defaulting missing entries to 0."""
    if text is None:
        return (0.0, 0.0, 0.0)
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f"{what} must have 3 values, got {text!r}")
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        raise ValueError(f"{what} is not numeric: {text!r}")


def _reject_cycle(links, joints, root):
    """Walk the tree from the root; a link reached twice means a cycle."""
    seen = set()

    def visit(link):
        if link in seen:
            raise ValueError(f"kinematic chain contains a cycle at '{link}'")
        seen.add(link)
        for joint in joints.values():
            if joint["parent"] == link:
                visit(joint["child"])

    visit(root)


# ── forward kinematics ───────────────────────────────────────────────────────


def _pose_compose(parent_pose, joint, q):
    """Compose `parent`'s pose with one joint to yield the child's pose.

    A pose is `(xyz, quat)`. The joint's `origin` is the fixed parent→joint
    transform; a revolute joint then rotates about its axis by `q`. This is the
    order `robot_state_publisher` uses, and the rotation composition that a
    naive "sum the xyz" implementation throws away.
    """
    parent_xyz, parent_quat = parent_pose
    parent_rot = _quat_to_matrix(parent_quat)

    origin_xyz, origin_rpy = joint["origin"]
    origin_rot = rpy_to_matrix(origin_rpy)

    if joint["type"] == "revolute":
        joint_rot = axis_angle_matrix(joint["axis"], q)
        rel_rot = mat_mul(origin_rot, joint_rot)
    else:
        rel_rot = origin_rot

    # Child rotation in the root frame.
    child_rot = mat_mul(parent_rot, rel_rot)
    # Child origin in the root frame: parent origin + parent_rot @ origin_xyz.
    child_xyz = tuple(
        parent_xyz[i] + mat_vec(parent_rot, origin_xyz)[i] for i in range(3))
    return (child_xyz, quat_from_matrix(child_rot))


def joint_local_pose(joint, q=0.0):
    """Parent→child pose of one joint, expressed in the **parent** frame.

    This is what a TF message carries: `header.frame_id` is the parent and the
    transform is local to it. It is deliberately different from
    `forward_kinematics`, which returns poses relative to the root. Publishing
    a root-relative pose with a parent frame_id misplaces every joint below the
    first one — the child's translation would contain the whole upstream chain.

    For a revolute joint the origin rotation is composed with the axis-angle
    rotation of the joint value, in that order (the URDF convention).
    """
    origin_xyz, origin_rpy = joint["origin"]
    rot = rpy_to_matrix(origin_rpy)
    if joint["type"] == "revolute":
        rot = mat_mul(rot, axis_angle_matrix(joint["axis"], float(q)))
    return (origin_xyz, quat_from_matrix(rot))


def _quat_to_matrix(q):
    """Unit quaternion `(x, y, z, w)` → rotation matrix."""
    x, y, z, w = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def forward_kinematics(tree, joint_values):
    """Compute every link's pose relative to the root.

    `tree` is the dict `parse_urdf` returns. `joint_values` maps joint name →
    angle in radians; a revolute joint missing from the map is treated as 0.
    The root is the identity pose. Returns ``{link: (xyz, quat)}``.
    """
    root = tree["root"]
    links = tree["links"]
    joints = tree["joints"]

    poses = {root: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))}

    # Iterate until every reachable link is placed. A well-formed tree needs at
    # most len(joints) passes, but this loop also tolerates any ordering of the
    # joint dict by repeatedly sweeping for newly-placeable children.
    remaining = set(links) - {root}
    while remaining:
        progress = False
        for name, joint in joints.items():
            parent, child = joint["parent"], joint["child"]
            if child not in remaining or parent not in poses:
                continue
            q = float(joint_values.get(name, 0.0))
            poses[child] = _pose_compose(poses[parent], joint, q)
            remaining.discard(child)
            progress = True
        if not progress:
            # Unreachable links — only possible if the tree is disconnected,
            # which `parse_urdf` already rejects (single root + no cycle).
            raise ValueError(
                f"unreachable links after FK: {sorted(remaining)}")
    return poses


# ── vision-transform merge ────────────────────────────────────────────────────

# The parent frames this card knows. A vision transform whose parent is not one
# of these would grow an orphan branch off the tree — exactly the "dangling"
# frame the single-publisher design exists to eliminate.
_KNOWN_PARENTS = frozenset(("camera_optical_frame", "camera_link", "base_link"))

# tf2 frame names are global and carry no namespace; `/` reads as a namespace
# separator and a leading digit is invalid. Reject, don't rewrite.
_INVALID_CHILD_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

# Clock skew tolerance between the vision producer and the TF node. A stamp
# further in the future than this is rejected, never broadcast as a future
# transform (tf2 lookups would fail and the entry would outlive its TTL).
_STAMP_SKEW_MS = 5000.0


def _finite_vec(value, n):
    """Return ``n`` finite floats, or None when the value is malformed.

    Vision edges arrive over an operator-wired ``data/json`` topic, so their
    translation/rotation cannot be trusted. ``_vision_transform`` indexes them
    directly when building a ROS message; a short vector raises IndexError in
    the subscription callback and a non-finite value produces an invalid TF.
    Reject both here, at the boundary, before anything reaches the broadcaster.
    """
    if not isinstance(value, (list, tuple)) or len(value) < n:
        return None
    try:
        out = [float(v) for v in value[:n]]
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(v) for v in out):
        return None
    return out


def merge_vision(transforms, now_ms, ttl_ms):
    """Split an incoming vision batch into live transforms and rejected ones.

    `transforms` is the list under T2's ``transforms`` key; each entry carries
    ``parent``, ``child``, ``stamp_ms``, ``translation`` (xyz) and ``rotation``
    (xyzw). Returns a ``(live, report)`` pair where ``live`` is the list of
    entries that should be published and ``report`` is a dict of the rejection
    counters: ``unknown_parent``, ``invalid_child``, ``expired`` and
    ``invalid_payload``.

    An entry is dropped when its parent frame is unknown, its child name is
    malformed, its ``stamp_ms`` is unusable, or its translation/rotation is not
    a finite, correctly-sized vector (a zero quaternion is invalid too).
    Rejections are counted, never silently published, so the status topic can
    say why a target disappeared. Payload validation matters because the input
    topic is operator-wired: a malformed edge must not raise inside the ROS
    callback or put an invalid TF on the wire.
    """
    live = []
    unknown_parent = 0
    invalid_child = 0
    expired = 0
    invalid_payload = 0

    for entry in transforms or []:
        parent = entry.get("parent", "")
        child = entry.get("child", "")
        if parent not in _KNOWN_PARENTS:
            unknown_parent += 1
            continue
        if not _INVALID_CHILD_RE.match(child or ""):
            invalid_child += 1
            continue
        stamp_ms = entry.get("stamp_ms")
        if stamp_ms is None:
            # No stamp at all: treat as fresh (the receiver stamped it) rather
            # than silently ageing it out on the first cycle.
            stamp_ms = now_ms
        else:
            try:
                stamp_ms = float(stamp_ms)
            except (TypeError, ValueError, OverflowError):
                expired += 1
                continue
            if not math.isfinite(stamp_ms):
                expired += 1
                continue
            # A future-dated stamp would stay "live" until wall time catches up
            # and would be published as a future ROS stamp that tf2 cannot use.
            # Allow a small clock-skew margin, then reject rather than clamp —
            # clamping would silently rewrite the observation's own time.
            if stamp_ms - now_ms > _STAMP_SKEW_MS:
                expired += 1
                continue
        if now_ms - stamp_ms > ttl_ms:
            expired += 1
            continue

        # Validate the payload the publisher will actually index into. A short
        # vector raises IndexError; a NaN/inf produces an invalid TransformStamped
        # that quietly poisons every downstream tf2 lookup. A zero quaternion is
        # not a rotation at all. Rewriting the validated values back keeps the
        # broadcaster free of re-parsing and guarantees finite output.
        raw_translation = entry.get("translation")
        raw_rotation = entry.get("rotation")
        # Absent fields keep the historical identity default; a PRESENT but
        # malformed value is rejected. This preserves callers that omit these
        # fields while stopping a wired-in bad edge from crashing the callback.
        translation = ([0.0, 0.0, 0.0] if raw_translation is None
                       else _finite_vec(raw_translation, 3))
        rotation = ([0.0, 0.0, 0.0, 1.0] if raw_rotation is None
                    else _finite_vec(raw_rotation, 4))
        if translation is None or rotation is None:
            invalid_payload += 1
            continue
        if math.sqrt(sum(v * v for v in rotation)) <= 1e-12:
            invalid_payload += 1
            continue
        validated = dict(entry)
        validated["translation"] = translation
        validated["rotation"] = rotation
        live.append(validated)

    return live, {
        "unknown_parent": unknown_parent,
        "invalid_child": invalid_child,
        "expired": expired,
        "invalid_payload": invalid_payload,
    }


# ── consumer-side freshness check ─────────────────────────────────────────────


def transform_is_fresh(header_stamp_ms, now_ms, max_age_ms):
    """Whether a TF transform's header stamp is still usable by a consumer.

    ROS ``/tf`` has no retract message: once this card publishes a vision edge,
    a ``tf2_ros`` buffer retains it for the consumer's cache duration even after
    the card's ``vision_ttl_ms`` has passed and it stopped republishing. A
    consumer using ``lookup_transform(target, source, Time())`` (latest) will
    therefore keep getting a stale localization with no error. The fix is on the
    consumer side: request the transform at a specific time and check the
    returned header stamp with this function before trusting it.

    Returns False for a missing, non-finite, future-dated, or too-old stamp —
    exactly the cases that must not be treated as a live object pose. This is a
    pure function (no rclpy/numpy) so downstream cards can use it offline too.
    """
    if header_stamp_ms is None:
        return False
    try:
        stamp = float(header_stamp_ms)
    except (TypeError, ValueError, OverflowError):
        return False
    if not math.isfinite(stamp):
        return False
    age = float(now_ms) - stamp
    # Future-dated stamps are as unusable as stale ones: wall clocks disagree.
    if age < -_STAMP_SKEW_MS or age > float(max_age_ms):
        return False
    return True


# ── status payload ────────────────────────────────────────────────────────────


def status_payload(*, state, stamp_ms, root_frame, link_count, dynamic_frames,
                   static_frames, tf_hz, last_joint_age_ms, joints_stale,
                   vision_targets, rejected_unknown_parent, expired_vision,
                   conflicting_parent, degraded, last_error,
                   camera_extrinsic_assumed=False, rejected_bad_payload=0,
                   rejected_bad_joints=0):
    """Build the flat `data/json` status payload.

    Every value is a top-level scalar, because the dashboard renders a nested
    object as raw JSON text. `degraded` is ``""`` when healthy, else one of
    ``no_joints`` / ``joints_stale`` / ``urdf_unparsed`` / ``conflicting_parent``
    / ``no_vision_input``. This payload is a *health* report about the tree, not
    a pose source — the description text says so.
    """
    return {
        "state": state,
        "stamp_ms": int(stamp_ms),
        "root_frame": root_frame,
        "link_count": int(link_count),
        "dynamic_frames": int(dynamic_frames),
        "static_frames": int(static_frames),
        "tf_hz": round(float(tf_hz), 1),
        "last_joint_age_ms": int(last_joint_age_ms),
        "joints_stale": bool(joints_stale),
        "vision_targets": int(vision_targets),
        "rejected_unknown_parent": int(rejected_unknown_parent),
        "expired_vision": int(expired_vision),
        "rejected_bad_payload": int(rejected_bad_payload),
        "rejected_bad_joints": int(rejected_bad_joints),
        "conflicting_parent": int(conflicting_parent),
        # True when the URDF camera mount translation is still the nominal
        # zero placeholder: every vision target below it carries this offset.
        "camera_extrinsic_assumed": bool(camera_extrinsic_assumed),
        "degraded": degraded or "",
        "last_error": last_error or "",
    }
