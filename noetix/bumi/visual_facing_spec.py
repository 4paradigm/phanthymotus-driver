"""Pure decision logic for the Bumi `visual_facing` actuator card.

This module imports no ROS/SDK code.  Every rule that can fail silently on the
robot -- a sign flip, a stale observation treated as fresh, an expired
authorisation that still commands motion, a deadband that oscillates -- lives
here so a laptop test can pin it down.

What the card does
------------------
It consumes a vision card's ``data/json`` target pose and turns Bumi *in
place* until the target sits in the centre of the picture.  The error signal is
deliberately optical and extrinsic-free::

    yaw_error = atan2(-x_optical, z_optical)

where ``x_optical`` is right-positive and ``z_optical`` is depth (forward).
No camera mount translation, no URDF lookup and no target frame in the TF tree
is required.  ``camera_to_body_point`` in ``vision_tf_spec`` maps optical
(x, y, z) to body (z, -x, -y), so ``atan2(-x, z)`` is the body yaw of the
target; positive error means "turn left", which is the sign convention of
``loco_relative_move``'s ``publish_cmd(0, 0, +z)``.  ``yaw_sign`` is exposed in
config because that final axis calibration must be confirmed on hardware.

Only ``publish_cmd(0, 0, z)`` is used: x and y are always exactly zero.
"""

from __future__ import annotations

import math

DEFAULTS = {
    "target_id": "",
    "units": "m",
    "yaw_sign": 1,
    "center_tolerance_deg": 5.0,
    "exit_tolerance_deg": 12.0,
    "settle_ticks": 2,
    "target_ttl_ms": 500,
    "max_future_ms": 500,
    "control_hz": 20.0,
    "kp": 1.2,
    "min_z": 0.45,
    "max_z": 0.70,
    "fine_duty": 0.4,
    "auth_window_s": 8.0,
    "max_turn_s": 12.0,
    "estop_topic": "",
    "require_estop": False,
    "estop_ttl_ms": 1000,
}

_UNITS = ("m", "mm")


def validate_config(cfg):
    """Validate every tunable up front; raise ValueError, never clamp."""
    result = dict(DEFAULTS)
    result.update(cfg or {})

    result["target_id"] = str(result["target_id"] or "").strip()
    if result["units"] not in _UNITS:
        raise ValueError(f"units must be one of {_UNITS}, got {result['units']!r}")
    if result["yaw_sign"] not in (-1, 1):
        raise ValueError("yaw_sign must be +1 or -1")

    limits = {
        "center_tolerance_deg": (1.0, 20.0),
        "exit_tolerance_deg": (2.0, 60.0),
        "target_ttl_ms": (100.0, 5000.0),
        "max_future_ms": (0.0, 5000.0),
        "control_hz": (5.0, 50.0),
        "kp": (0.1, 3.0),
        "min_z": (0.41, 0.9),          # 0.40 is the measured stiction edge
        "max_z": (0.42, 1.0),
        "fine_duty": (0.1, 1.0),
        "auth_window_s": (1.0, 60.0),
        "max_turn_s": (1.0, 60.0),
        "estop_ttl_ms": (100.0, 10000.0),
    }
    values = {}
    for name, (low, high) in limits.items():
        value = _number(result[name], name)
        if not low <= value <= high:
            raise ValueError(f"{name} must be in [{low:g}, {high:g}]")
        values[name] = value
    result.update(values)

    if result["exit_tolerance_deg"] <= result["center_tolerance_deg"]:
        raise ValueError("exit_tolerance_deg must exceed center_tolerance_deg (hysteresis)")
    if result["max_z"] < result["min_z"]:
        raise ValueError("max_z must be >= min_z")

    settle = result["settle_ticks"]
    if isinstance(settle, bool) or not isinstance(settle, int) or not 1 <= settle <= 10:
        raise ValueError("settle_ticks must be an integer in [1, 10]")
    if not isinstance(result["require_estop"], bool):
        raise ValueError("require_estop must be a boolean")
    result["estop_topic"] = str(result["estop_topic"] or "").strip()
    return result


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number")
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite")
    return out


def _finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def _get(obj, *names, default=None):
    for name in names:
        if isinstance(obj, dict) and name in obj:
            return obj[name]
    return default


def _vec3(value):
    if value is None:
        return None
    try:
        if isinstance(value, dict):
            if not all(k in value for k in ("x", "y", "z")):
                return None
            out = (float(value["x"]), float(value["y"]), float(value["z"]))
        elif isinstance(value, (list, tuple)) and len(value) >= 3:
            out = (float(value[0]), float(value[1]), float(value[2]))
        else:
            return None
    except (TypeError, ValueError, OverflowError):
        return None
    return out if _finite(*out) else None


def _entries(payload):
    """Flatten the common vision JSON shapes into candidate target dicts."""
    if isinstance(payload, (list, tuple)):
        return list(payload)
    if not isinstance(payload, dict):
        return []
    for key in ("transforms", "targets", "objects", "markers"):
        value = payload.get(key)
        if isinstance(value, (list, tuple)):
            return list(value)
    return [payload]


def _stamp_ms(entry, fallback_ms):
    raw = _get(entry, "stamp_ms", "timestamp_ms", "obs_ms")
    if raw is None:
        return fallback_ms
    try:
        value = float(raw)
    except (TypeError, ValueError, OverflowError):
        return fallback_ms
    return value if math.isfinite(value) else fallback_ms


def yaw_error_deg(point, cfg):
    """Optical-frame target point -> body yaw error in degrees.

    Rejects targets at/behind the lens: a negative depth is not a 180 degree
    request, it is an observation the camera cannot make.
    """
    x, _y, z = point
    if not _finite(x, _y, z) or z <= 0.0:
        raise ValueError("target must be finite and in front of the camera (z > 0)")
    return math.degrees(math.atan2(-x, z)) * cfg["yaw_sign"]


def select_target(payload, now_ms, cfg):
    """Pick the one fresh target to look at.

    Returns a flat dict with ``point`` / ``target_id`` / ``stamp_ms`` on
    success, or ``error`` describing exactly why no safe target exists.  An
    explicit ``target_id`` is required when more than one candidate is present;
    silently picking one of several markers would make the robot turn toward
    an arbitrary object.
    """
    now_ms = float(now_ms)
    top_stamp = _stamp_ms(payload if isinstance(payload, dict) else {}, now_ms)
    candidates = []
    rejected = 0
    for entry in _entries(payload):
        if not isinstance(entry, dict):
            rejected += 1
            continue
        point = _vec3(_get(entry, "tvec", "position", "translation", "t"))
        if point is None:
            rejected += 1
            continue
        if cfg["units"] == "mm":
            point = tuple(v / 1000.0 for v in point)
        stamp = _stamp_ms(entry, top_stamp)
        age = now_ms - stamp
        if age > cfg["target_ttl_ms"] or age < -cfg["max_future_ms"]:
            rejected += 1
            continue
        ident = _get(entry, "marker_id", "id", "object_id", "track_id",
                     "child", "child_frame_id", "label", default="")
        confidence = _get(entry, "confidence", default=None)
        if confidence is not None:
            try:
                confidence = float(confidence)
            except (TypeError, ValueError, OverflowError):
                rejected += 1
                continue
        candidates.append({
            "point": point,
            "target_id": str(ident),
            "stamp_ms": stamp,
            "age_ms": max(0.0, age),
            "confidence": confidence,
        })

    wanted = cfg["target_id"]
    if wanted:
        matches = [c for c in candidates if c["target_id"] == wanted]
        if not matches:
            return {"error": "target_not_found", "rejected": rejected + len(candidates),
                    "valid": len(candidates)}
        candidates = matches
    if not candidates:
        return {"error": "no_fresh_target", "rejected": rejected, "valid": 0}
    if len(candidates) > 1:
        # Confidence is a tie-breaker only when it is explicitly supplied for
        # every candidate; otherwise the input is ambiguous on purpose.
        if all(c["confidence"] is not None for c in candidates):
            best = max(candidates, key=lambda c: c["confidence"])
            if sum(c["confidence"] == best["confidence"] for c in candidates) == 1:
                return {**best, "error": "", "rejected": rejected, "valid": len(candidates)}
        return {"error": "ambiguous_targets", "rejected": rejected,
                "valid": len(candidates)}

    return {**candidates[0], "error": "", "rejected": rejected, "valid": 1}


def control_for_error(err_deg, turning, cfg):
    """Hysteretic control law: one result per control tick.

    ``z`` is a normalised yaw velocity; ``duty`` is the fraction of ticks that
    velocity is actually applied.  Creeping in the fine band uses a duty cycle
    above the measured stiction floor instead of a too-small z that never
    moves the legs.
    """
    magnitude = abs(float(err_deg))
    sign = 1.0 if err_deg >= 0.0 else -1.0
    hold = {"z": 0.0, "duty": 0.0, "mode": "hold"}
    if not turning:
        # Resting state only wakes up outside the exit tolerance; the band
        # between the two tolerances is deliberately dead.
        if magnitude < cfg["exit_tolerance_deg"]:
            return hold
    else:
        if magnitude <= cfg["center_tolerance_deg"]:
            return {"z": 0.0, "duty": 0.0, "mode": "settle"}
        if magnitude < cfg["exit_tolerance_deg"]:
            return {"z": sign * cfg["min_z"], "duty": cfg["fine_duty"],
                    "mode": "fine"}
    speed = min(cfg["max_z"], max(cfg["min_z"], cfg["kp"] * math.radians(magnitude)))
    return {"z": sign * speed, "duty": 1.0, "mode": "coarse"}


def pulse_velocity(z, duty, accumulator):
    """Turn a duty cycle into this tick's velocity; return (z, next_acc)."""
    if duty >= 1.0:
        return z, 0.0
    if duty <= 0.0 or z == 0.0:
        return 0.0, 0.0
    accumulator += duty
    if accumulator >= 1.0:
        return z, accumulator - 1.0
    return 0.0, accumulator


def arm_authorization(now_ms, window_s, confirm):
    """Explicit enable; returns an absolute deadline in ms."""
    if confirm is not True:
        raise ValueError("enable requires confirm=true")
    return float(now_ms) + float(window_s) * 1000.0


def authorization_remaining_ms(now_ms, deadline_ms):
    if deadline_ms is None:
        return 0.0
    return max(0.0, float(deadline_ms) - float(now_ms))


def stop_reason(mode, estop_state, auth_reason, target_error,
                chassis_busy=False, require_estop=False, turn_elapsed_s=0.0,
                cfg=None):
    """Fail-closed gate.  Empty string means motion is allowed.

    ``estop_state`` is ``"clear"``, ``"latched"`` or ``"unknown"``.  An
    unobservable estop only blocks motion when the operator configured
    ``require_estop: true``.
    """
    if mode == 26:
        return "protection_mode"
    if mode != 2:
        return "not_walking"
    if estop_state == "latched":
        return "estop"
    if estop_state == "unknown" and require_estop:
        return "estop_unknown"
    if auth_reason:
        return auth_reason
    if target_error:
        return target_error
    if chassis_busy:
        return "chassis_busy"
    if cfg is not None and turn_elapsed_s > cfg["max_turn_s"]:
        return "turn_timeout"
    return ""


def status_payload(*, state, now_ms, target_id="", target_age_ms=-1.0,
                   yaw_error_deg_value=0.0, z_cmd=0.0, mode="hold",
                   authorized=False, auth_remaining_ms=0.0, workmode=-1,
                   estop="unknown", stop="", received=0, turns=0,
                   last_error=""):
    """Every field is a top-level scalar: `data/json` panels stay readable."""
    return {
        "state": str(state),
        "stamp_ms": int(now_ms),
        "target_id": str(target_id),
        "target_age_ms": round(float(target_age_ms), 3),
        "yaw_error_deg": round(float(yaw_error_deg_value), 3),
        "z_cmd": round(float(z_cmd), 4),
        "control_mode": str(mode),
        "authorized": bool(authorized),
        "auth_remaining_ms": int(auth_remaining_ms),
        "workmode": int(workmode),
        "estop": str(estop),
        "stop_reason": str(stop),
        "received": int(received),
        "turns": int(turns),
        "last_error": str(last_error),
    }
