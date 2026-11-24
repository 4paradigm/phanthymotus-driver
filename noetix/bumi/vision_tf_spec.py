"""Pure maths and payload building for the `vision_tf` processor card.

This module is deliberately dependency-free: no `rclpy`, no `numpy`, no `cv2`.
Everything here is a pure function over plain numbers, dicts and tuples, so the
rules that would otherwise fail silently — a sign flip, a wrong quaternion
order, an invented zero for a missing field — are all pinned by tests that run
on a laptop.

What the card does
------------------
It turns "where a vision algorithm says an object is" into a tf-shaped edge:

    parent (a camera frame)  ->  child (the located object)

and emits those edges as a ``data/json`` payload for a downstream tf-tree card
to publish.  The card does **not** publish ``/tf`` and does not drive hardware.

Coordinate conventions
----------------------
Two frame families show up in visual-localisation JSON, and they are *not* the
same numbering:

  * optical (x right, y down, z depth) — what AR/YOLO/depth algorithms emit;
  * body    (x forward, y left, z up) — the robot's spatial convention.

``camera_to_body_point`` / ``camera_to_body_quaternion`` convert between them.
They are exposed here so the card can emit *either* family on request
(``output_frame: optical`` or ``body``).  The default is ``optical``: T1 already
encodes the very same rotation as a fixed joint in the URDF
(``camera_optical_frame``), so a caller that wires its vision card straight to a
tf tree should normally let the tree do the rotation and keep the numbers
untouched.  Use ``body`` only when you want the card itself to output
machine-frame numbers hanging off ``camera_link``.

Quaternions are **xyzw** everywhere in this module.
"""

from __future__ import annotations

import math
import re

ROTATION_ORDER = "xyzw"

# Optical -> body rotation, xyzw.  Same rotation as URDF
# ``camera_optical_joint`` rpy(-pi/2, 0, -pi/2) and as config.yaml's
# calibration.camera_to_body.quaternion.  Rotating (1,0,0) by this gives
# (0,-1,0): optical +x (right) becomes body -y, as expected.
OPTICAL_TO_BODY_QUAT = (-0.5, 0.5, -0.5, 0.5)

ADAPTERS = ("pose", "pose_tf_like", "ar_marker", "pixel_depth")
INPUT_FRAMES = ("optical", "body", "base")
OUTPUT_FRAMES = ("optical", "body")
POSE_DIRECTIONS = ("object_in_camera", "camera_in_object")
UNITS = ("m", "mm")

# Parent frame names for each input convention.  These are the frames T1 adds
# to the URDF; ``base`` is the root that already exists.
_PARENT_BY_INPUT_FRAME = {
    "optical": "camera_optical_frame",
    "body": "camera_link",
    "base": "base_link",
}

_SAFE_CHARS = re.compile(r"[^A-Za-z0-9_]")


# ── small vector / quaternion helpers (tuples, numpy-free) ───────────────────

def _qmul(a, b):
    """Hamilton product of two xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qnorm(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        raise ValueError("zero-length quaternion")
    return tuple(v / n for v in q)


def rotate_vector(v, q):
    """Rotate a 3-vector by an xyzw quaternion (``v' = q v q^-1``)."""
    q = _qnorm(q)
    x, y, z = v
    qx, qy, qz, qw = q
    # t = 2 * (q_vec x v)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def camera_to_body_point(point):
    """Optical (x right, y down, z depth) -> body (x fwd, y left, z up)."""
    x, y, z = point
    return (z, -x, -y)


def camera_to_body_quaternion(q):
    """Compose the optical->body rotation onto an optical-frame orientation."""
    return _qnorm(_qmul(OPTICAL_TO_BODY_QUAT, q))


def rodrigues_to_quaternion(rvec):
    """Rodrigues rotation vector (radians) -> xyzw quaternion."""
    x, y, z = (float(v) for v in rvec)
    angle = math.sqrt(x * x + y * y + z * z)
    if angle <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    half = angle / 2.0
    s = math.sin(half) / angle
    return _qnorm((x * s, y * s, z * s, math.cos(half)))


def invert_transform(translation, rotation):
    """Invert a rigid transform: ``t' = -R^T t``, ``q' = q^-1``."""
    q_inv = _qconj(_qnorm(rotation))
    # R^T t == rotate t by q^-1
    back = rotate_vector(translation, q_inv)
    return (-back[0], -back[1], -back[2]), q_inv


# ── field parsing (no guessing) ──────────────────────────────────────────────

def _as_vec3(value, aliases=()):
    """Accept ``[x,y,z]`` or ``{x,y,z}``; return None if absent/unparseable."""
    if value is None:
        return None
    try:
        if isinstance(value, dict):
            for name in ("x", "y", "z"):
                if name not in value:
                    return None
            return (float(value["x"]), float(value["y"]), float(value["z"]))
        if isinstance(value, (list, tuple)) and len(value) >= 3:
            return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None
    return None


def _as_quat(value, aliases=()):
    if value is None:
        return None
    try:
        if isinstance(value, dict) and all(k in value for k in ("x", "y", "z", "w")):
            return (float(value["x"]), float(value["y"]),
                    float(value["z"]), float(value["w"]))
        if isinstance(value, (list, tuple)) and len(value) >= 4:
            return tuple(float(v) for v in value[:4])
    except (TypeError, ValueError):
        return None
    return None


def _get(obj, *names, default=None):
    for name in names:
        if isinstance(obj, dict) and name in obj:
            return obj[name]
    return default


def _scale_translation(t, units):
    if units == "mm":
        return (t[0] / 1000.0, t[1] / 1000.0, t[2] / 1000.0)
    return t


def _target_id(obj, index):
    value = _get(obj, "marker_id", "id", "object_id", "track_id",
                 "child_frame_id", "label", "class", "name")
    if value is None:
        return str(index)
    return str(value)


def _stamp_ms(obj, receive_ms):
    """Use the observation stamp verbatim; never pretend an old result is fresh."""
    raw = _get(obj, "stamp_ms", "timestamp_ms")
    if raw is not None:
        try:
            return float(raw)
        except (TypeError, ValueError):
            pass
    header = _get(obj, "header")
    if isinstance(header, dict):
        stamp = header.get("stamp")
        if isinstance(stamp, dict):
            sec = stamp.get("sec", stamp.get("s"))
            nanosec = stamp.get("nanosec", stamp.get("nsec"))
            if sec is not None:
                return float(sec) * 1000.0 + float(nanosec or 0) / 1e6
        elif stamp is not None:
            # A bare number: seconds if small, milliseconds otherwise.
            try:
                value = float(stamp)
                return value * 1000.0 if abs(value) < 1e8 else value
            except (TypeError, ValueError):
                pass
    return float(receive_ms)


# ── adapters: raw object -> (translation, rotation, confidence) or error ─────

def adapt_pose(obj, units="m"):
    pos = _as_vec3(_get(obj, "position", "translation", "t"))
    if pos is None:
        return None, None, "missing position (expected [x,y,z] or {x,y,z})"
    rot = _as_quat(_get(obj, "orientation", "rotation", "quaternion", "q"))
    if rot is None:
        return None, None, "missing orientation (expected xyzw [x,y,z,w])"
    return (_scale_translation(pos, units), _qnorm(rot), None)


def adapt_pose_tf_like(obj, units="m"):
    pos = _as_vec3(_get(obj, "translation", "position", "t"))
    if pos is None:
        return None, None, "missing translation"
    rot = _as_quat(_get(obj, "rotation", "orientation", "quaternion", "q"))
    if rot is None:
        return None, None, "missing rotation"
    return (_scale_translation(pos, units), _qnorm(rot), None)


def adapt_ar_marker(obj, units="m"):
    tvec = _as_vec3(_get(obj, "tvec", "position", "translation"))
    if tvec is None:
        return None, None, "missing tvec"
    has_rvec = _get(obj, "rvec") is not None
    has_quat = (_get(obj, "quaternion") is not None
                or _get(obj, "orientation") is not None)
    if has_rvec and has_quat:
        return None, None, "ambiguous: both rvec and quaternion present"
    if has_quat:
        rot = _as_quat(_get(obj, "quaternion", "orientation"))
        if rot is None:
            return None, None, "invalid quaternion"
    elif has_rvec:
        rv = _as_vec3(_get(obj, "rvec"))
        if rv is None:
            return None, None, "invalid rvec (expected 3 numbers)"
        rot = rodrigues_to_quaternion(rv)
    else:
        return None, None, "missing orientation (rvec or quaternion required)"
    return (_scale_translation(tvec, units), rot, None)


def adapt_pixel_depth(obj, units="m", *, focal_px, width, height):
    u = _get(obj, "u", "px", "cx_px")
    v = _get(obj, "v", "py", "cy_px")
    depth = _get(obj, "depth_m", "depth", "z")
    if u is None or v is None or depth is None:
        return None, None, "pixel_depth needs u, v and depth_m"
    try:
        u, v, depth = float(u), float(v), float(depth)
    except (TypeError, ValueError):
        return None, None, "u/v/depth_m must be numbers"
    if depth <= 0.0 or focal_px <= 0.0:
        return None, None, "depth_m and focal_px must be positive"
    intrinsics = _get(obj, "intrinsics", default={}) or {}
    fx = float(_get(intrinsics, "fx", default=focal_px))
    fy = float(_get(intrinsics, "fy", default=focal_px))
    cx = float(_get(intrinsics, "cx", default=width / 2.0))
    cy = float(_get(intrinsics, "cy", default=height / 2.0))
    # Ideal pinhole, optical axes (x right, y down, z depth).  No distortion
    # correction: the caller must supply rectified pixels.
    pos = ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)
    return (_scale_translation(pos, units), (0.0, 0.0, 0.0, 1.0), None)


_ADAPTER_FNS = {
    "pose": lambda obj, units, **kw: adapt_pose(obj, units),
    "pose_tf_like": lambda obj, units, **kw: adapt_pose_tf_like(obj, units),
    "ar_marker": lambda obj, units, **kw: adapt_ar_marker(obj, units),
    "pixel_depth": adapt_pixel_depth,
}


# ── naming, parent frame, config ─────────────────────────────────────────────

def sanitize_child_name(source_name, ident):
    """``vision_<source>_<id>`` with illegal chars replaced; (name, changed)."""
    raw = f"vision_{source_name}_{ident}"
    clean = _SAFE_CHARS.sub("_", raw)
    changed = clean != raw
    if clean and clean[0].isdigit():
        clean = "v_" + clean
        changed = True
    if not clean:
        clean = "vision_object"
        changed = True
    return clean, changed


def resolve_parent_frame(input_frame, output_frame):
    """Pick the parent frame from the *conventions*, never guess a live name.

    ``output_frame: body`` means the card rotates the numbers into machine axes,
    so the parent becomes ``camera_link`` (not ``camera_optical_frame``).
    """
    if output_frame == "body":
        # The values are now body-axis numbers at the camera origin.
        return "camera_link"
    return _PARENT_BY_INPUT_FRAME.get(input_frame, "camera_optical_frame")


def validate_config(cfg):
    """Validate every tunable up front; raise ValueError, never clamp."""
    cfg = dict(cfg or {})
    adapter = str(cfg.get("adapter", "pose"))
    if adapter not in ADAPTERS:
        raise ValueError(f"adapter must be one of {ADAPTERS}, got {adapter!r}")
    input_frame = str(cfg.get("input_frame", "optical"))
    if input_frame not in INPUT_FRAMES:
        raise ValueError(f"input_frame must be one of {INPUT_FRAMES}")
    output_frame = str(cfg.get("output_frame", "optical"))
    if output_frame not in OUTPUT_FRAMES:
        raise ValueError(f"output_frame must be one of {OUTPUT_FRAMES}")
    direction = str(cfg.get("pose_direction", "object_in_camera"))
    if direction not in POSE_DIRECTIONS:
        raise ValueError(f"pose_direction must be one of {POSE_DIRECTIONS}")
    units = str(cfg.get("units", "m"))
    if units not in UNITS:
        raise ValueError(f"units must be one of {UNITS}, got {units!r}")

    ttl_ms = float(cfg.get("ttl_ms", 500))
    if not 50.0 <= ttl_ms <= 60000.0:
        raise ValueError(f"ttl_ms must be in [50, 60000], got {ttl_ms}")
    min_confidence = float(cfg.get("min_confidence", 0.0))
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError(f"min_confidence must be in [0, 1], got {min_confidence}")
    focal_px = float(cfg.get("focal_px", 460.0))
    if not 100.0 <= focal_px <= 2000.0:
        raise ValueError(f"focal_px must be in [100, 2000], got {focal_px}")
    rate = float(cfg.get("publish_rate_hz", 10.0))
    if not 1.0 <= rate <= 50.0:
        raise ValueError(f"publish_rate_hz must be in [1, 50], got {rate}")
    width = int(cfg.get("image_width", 640))
    height = int(cfg.get("image_height", 480))
    if not 16 <= width <= 4096 or not 16 <= height <= 4096:
        raise ValueError("image_width/height must be in [16, 4096]")
    source = str(cfg.get("source_name", "vision"))
    if not re.match(r"^[A-Za-z][A-Za-z0-9_]*$", source):
        raise ValueError("source_name must start with a letter and be alphanumeric/underscore")

    cfg.update(adapter=adapter, input_frame=input_frame, output_frame=output_frame,
               pose_direction=direction, units=units, ttl_ms=ttl_ms,
               min_confidence=min_confidence, focal_px=focal_px,
               publish_rate_hz=rate, image_width=width, image_height=height,
               source_name=source)
    cfg.pop("child_prefix", None)  # never a real knob; child name is vision_<source>_<id>
    return cfg


# ── the core transform builder ───────────────────────────────────────────────

def transforms_from_json(raw, cfg, *, receive_ms, targets):
    """Convert one vision JSON message into a batch of tf edges.

    ``targets`` is a ``dict`` mapping child frame -> last-seen stamp (ms) kept by
    the caller; this function updates it in place so TTL is stateless here.
    Returns a dict ready to merge into the output payload.
    """
    adapter_fn = _ADAPTER_FNS[cfg["adapter"]]
    parent = resolve_parent_frame(cfg["input_frame"], cfg["output_frame"])

    # A message may hold one object or a list of them under a common key.
    if isinstance(raw, list):
        objects = list(raw)
    elif isinstance(raw, dict):
        objects = None
        for key in ("targets", "objects", "detections", "markers", "results"):
            if isinstance(raw.get(key), list):
                objects = list(raw[key])
                break
        if objects is None:
            objects = [raw]
    else:
        return _batch(parent, [], 1, 0, [], receive_ms, "input JSON must be an object or list")

    edges = []
    rejected = 0
    sanitized = 0
    last_error = None
    seen = set()

    for index, obj in enumerate(objects):
        if not isinstance(obj, dict):
            rejected += 1
            last_error = "target is not a JSON object"
            continue
        kind = cfg["adapter"]
        kw = dict(focal_px=cfg["focal_px"], width=cfg["image_width"],
                  height=cfg["image_height"])
        pos, rot, err = adapter_fn(obj, cfg["units"], **kw)
        if err is not None:
            rejected += 1
            last_error = f"{kind}: {err}"
            continue

        confidence = float(_get(obj, "confidence", "score", "prob", default=1.0))
        if confidence < cfg["min_confidence"]:
            continue

        if cfg["pose_direction"] == "camera_in_object":
            pos, rot = invert_transform(pos, rot)

        if cfg["output_frame"] == "body":
            # Card-side optical->body rotation (see module docstring).
            pos = camera_to_body_point(pos)
            rot = camera_to_body_quaternion(rot)

        name, changed = sanitize_child_name(cfg["source_name"], _target_id(obj, index))
        if changed:
            sanitized += 1
        if name in seen:
            rejected += 1
            last_error = f"duplicate child frame after sanitising: {name}"
            continue
        seen.add(name)

        stamp = _stamp_ms(obj, receive_ms)
        edges.append({
            "parent": parent,
            "child": name,
            "stamp_ms": stamp,
            "stamp_source": "input" if _get(obj, "stamp_ms", "timestamp_ms", "header") is not None else "receive",
            "translation": [round(float(v), 9) for v in pos],
            "rotation": [round(float(v), 9) for v in rot],
            "confidence": round(confidence, 6),
        })
        targets[name] = receive_ms

    # Drop targets whose last observation is older than the TTL.
    dropped = sorted(name for name, last in targets.items()
                     if receive_ms - last > cfg["ttl_ms"])
    for name in dropped:
        targets.pop(name, None)
    return _batch(parent, edges, rejected, sanitized, dropped, receive_ms, last_error)


def _batch(parent, edges, rejected, sanitized, dropped, receive_ms, last_error):
    degraded = ""
    if last_error and not edges:
        degraded = "all_rejected"
    return {
        "parent_frame": parent,
        "rotation_order": ROTATION_ORDER,
        "units": "m",
        "transforms": edges,
        "rejected": rejected,
        "sanitized": sanitized,
        "dropped": dropped,
        "degraded": degraded,
        "last_error": last_error or "",
        "receive_ms": receive_ms,
    }


def build_payload(cfg, batch):
    """Final wire payload for the output topic (nested transforms are fine:
    this is a machine topic, not a panel-facing flat KV payload)."""
    return {
        "stamp_ms": batch["receive_ms"],
        "source": cfg["source_name"],
        "parent_frame": batch["parent_frame"],
        "pose_direction": cfg["pose_direction"],
        "rotation_order": batch["rotation_order"],
        "units": batch["units"],
        "transforms": batch["transforms"],
        "rejected": batch["rejected"],
        "sanitized": batch["sanitized"],
        "dropped": batch["dropped"],
        "degraded": batch["degraded"],
        "last_error": batch["last_error"],
    }
