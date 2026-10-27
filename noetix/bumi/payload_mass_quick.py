"""Session-only prototype setup and single-reference calibration.

No engineering-verification flags are fabricated. Parameters below are prototype
measurement tolerances, not certified robot limits or hardware accuracy claims.
"""
from __future__ import annotations

import math
import time

from payload_mass_estimator import ARM_IDS, NAMES, digest, summarize
from payload_mass_sampling import MassError, finite, vector, fixed_pose_motor_ids


def profile_from_home(frame, arm, hold_s):
    if arm not in ARM_IDS:
        raise MassError("BAD_PARAMETER", "arm must be left or right")
    q = vector(frame["q"], 21, "q")
    temperatures = vector(frame["temperature"], 21, "temperature")
    p = {"profile_id": "quick_session", "setup_mode": "quick", "arm": arm,
         "motor_ids": ARM_IDS[arm], "joint_names": NAMES[arm],
         "pose_q": q, "quaternion_xyzw": vector(frame["quaternion_xyzw"], 4, "quaternion"),
         "workmode": 23, "torque_units": "sdk_raw", "accept_host_only_freshness": False,
         "pose_tolerance_rad": .15, "body_tolerance_rad": .1,
         # Hardware preparation rejected dq=0.1343 at the old 0.1 cutoff.
         # This is a measurement tolerance; pose span still gates stable data.
         "max_velocity_rad_s": .2, "max_pose_span_rad": .03, "max_gyro_rad_s": .15,
         "max_torque_span": 1e9, "torque_abs_limit": 1e9,
         "max_temperature_c": max(temperatures) + 10,
         "max_temperature_drift_c": 10, "max_mass_kg": .5,
         "noise_floor_raw": 1e-6, "settle_timeout_s": 10,
         "min_samples": 15, "max_receive_gap_s": .5,
         "baseline_max_age_s": hold_s, "result_max_age_s": 30,
         "quality": "experimental_single_reference"}
    p["fixed_pose_motor_ids"] = list(fixed_pose_motor_ids(p))
    p["fingerprint"] = digest(p)
    return p


def capture_bend_profile(p, frames):
    """Learn the pose/noise from a complete empty, stationary observation."""
    summary = summarize(frames, p)
    span = max(max(f["tau"][j] for f in frames) - min(f["tau"][j] for f in frames)
               for j in p["motor_ids"])
    p = {**p, "pose_q": summary["pose_q"], "quaternion_xyzw": summary["quaternion_xyzw"],
         "max_torque_span": max(1e-4, 3 * span, 6 * max(summary["noise"]))}
    p.pop("fingerprint", None)
    p["fingerprint"] = digest(p)
    return p


def fit_reference(frames, baseline, p, known_mass_kg):
    mass = finite(known_mass_kg, "known_mass_kg", .001, .5)
    current = summarize(frames, p)
    delta = [a-b for a, b in zip(current["tau"], baseline["summary"]["tau"])]
    noise = [max(a, b, p["noise_floor_raw"]) for a, b in zip(current["noise"], baseline["summary"]["noise"])]
    k = [v/mass for v in delta]
    weights = [1/n**2 if abs(v) >= 5*n else 0 for v, n in zip(delta, noise)]
    sensitivity = sum(a*a*w for a, w in zip(k, weights))
    if sensitivity <= 1e-15:
        raise MassError("UNIDENTIFIABLE", "参考物体没有产生可分辨的力矩变化，请检查力矩反馈。")
    resolution = max(.01, 3 / math.sqrt(sensitivity))
    if resolution >= mass:
        raise MassError("UNIDENTIFIABLE", "参考物体响应不足以形成可用读数。")
    return {"profile_fingerprint": p["fingerprint"], "baseline_id": baseline["id"],
            "k": k, "weights": weights, "resolution_kg": resolution,
            "mass_range_kg": [0, .5], "residual_limit": 5.0,
            "reference_mass_kg": mass, "created_s": time.time(),
            "quality": "experimental_single_reference", "validation_status": "not_validated"}
