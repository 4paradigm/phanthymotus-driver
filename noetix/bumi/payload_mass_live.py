"""Rolling scale display: provisional readings while loading, stable flag later."""
from __future__ import annotations

import math

from payload_mass_estimator import check_baseline, projection, summarize
from payload_mass_sampling import MassError, check_frame, pose_span_violation


def evaluate_window(frames, baseline, model, profile, duration_s, now):
    """Never re-tare from loaded data. A provisional number is not a valid mass."""
    p = profile
    moving = False
    for frame in frames:
        try:
            check_frame(frame, p)
        except MassError as exc:
            if exc.code != "UNSTABLE":
                raise
            moving = True
    full = summarize(frames, p)
    check_baseline(baseline, full, p, now)
    pose_stable = p.get("setup_mode") != "quick" or pose_span_violation(frames, p) is None
    stable = (not moving and pose_stable and len(frames) >= p["min_samples"]
              and full["host_span_s"] >= duration_s
              and all(max(f["tau"][j] for f in frames) - min(f["tau"][j] for f in frames)
                      <= p["max_torque_span"] for j in p["motor_ids"]))
    result = {"state": "stabilizing", "stable": False, "sampling_stable": stable,
              "valid": False, "mass_kg": None, "unit": "kg", "sample_count": len(frames),
              "freshness_status": full["freshness_status"], "effective_sample_count": None}
    if model is None:
        result.update(state="uncalibrated", error_code="NOT_CALIBRATED",
                      message="正在采集，尚未标定；稳定数据会自动保存。")
        return result
    if model["profile_fingerprint"] != p["fingerprint"]:
        raise MassError("CONFIG_CHANGED", "calibration does not match this profile")
    # Fast display follows the last ~0.3 s; accepted stable readings use the
    # entire window. Neither path uses the object's known mass as an input.
    recent = [f for f in frames if f["host_monotonic_s"] >= frames[-1]["host_monotonic_s"] - .3]
    summary = full if stable else summarize(recent, p)
    delta = [a-b for a, b in zip(summary["tau"], baseline["summary"]["tau"])]
    mass = projection(model["k"], model["weights"], delta)
    resolution = model["resolution_kg"]
    upper = model["mass_range_kg"][1]
    rounding_epsilon = 1e-12 * max(1.0, upper)
    if mass < -resolution - rounding_epsilon or mass > upper + rounding_epsilon:
        result.update(state="out_of_range", error_code="OUT_OF_RANGE", message="超出标定范围。")
        return result
    mass = min(mass, upper)  # absorb arithmetic roundoff at the inclusive bound
    residual = math.sqrt(sum(w*(v-k*mass)**2 for w, v, k in zip(model["weights"], delta, model["k"]))
                         / max(1, sum(w > 0 for w in model["weights"])))
    if residual > model["residual_limit"]:
        result.update(error_code="RESIDUAL", message="挂载中，等待力矩响应稳定。")
        return result
    digits = max(0, min(3, -math.floor(math.log10(resolution))))
    result.update(state="stable" if stable else "stabilizing", stable=stable, valid=stable,
                  mass_kg=0.0 if abs(mass) < resolution else round(mass, digits),
                  below_resolution=abs(mass) < resolution, resolution_kg=resolution,
                  mass_range_kg=model["mass_range_kg"], residual=residual,
                  quality="empirical_fixed_pose", provisional=not stable,
                  message="重量稳定。" if stable else "实时估算中，请等待稳定。")
    if result["below_resolution"]:
        result["message"] = "低于分辨率，显示为0 kg。" if stable else "等待挂物。"
    return result
