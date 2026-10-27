"""Fixed-pose empirical net-mass model. Pure Python; no SDK, ROS or motion.

Signed per-kg response, zero intercept after fixture-inclusive tare. Holdout is
by entire independent trial. This is not a physical Nm/Jacobian backend.
"""
from __future__ import annotations

import hashlib
import json
import math
from statistics import median

from payload_mass_sampling import MassError, finite, vector, quaternion_angle, fixed_pose_motor_ids

VERSION = "bumi-payload-mass/1"
ARM_IDS = {"left": [0,1,2,3], "right": [10,11,12,13]}
NAMES = {"left": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_arm_yaw_joint", "l_elbow_pitch_joint"],
         "right": ["r_arm_pitch_joint", "r_arm_roll_joint", "r_arm_yaw_joint", "r_elbow_pitch_joint"]}


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()).hexdigest()


def validate_profile(profile_id, raw, arm, *, loading=False):
    if not isinstance(profile_id, str) or not profile_id or len(profile_id)>80 or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in profile_id):
        raise MassError("BAD_PARAMETER", "invalid profile_id")
    p = dict(raw)
    if arm not in ARM_IDS or p.get("arm") != arm:
        raise MassError("NOT_CALIBRATED", "profile does not belong to selected arm")
    for key in ("hardware_id", "sdk_version", "control_config", "hook_id", "mapping_evidence"):
        if not isinstance(p.get(key), str) or not p[key].strip():
            raise MassError("PROFILE_UNVERIFIED", f"missing {key}")
    ids = p.get("motor_ids")
    if (not isinstance(ids, list) or not ids or any(type(j) is not int or j not in ARM_IDS[arm] for j in ids)
            or len(set(ids)) != len(ids)):
        raise MassError("MAPPING", "select unique, verified shoulder/elbow IDs of this arm")
    if p.get("joint_names") != [NAMES[arm][ARM_IDS[arm].index(j)] for j in ids]:
        raise MassError("MAPPING", "URDF names must match selected SDK motor IDs")
    p["pose_q"] = vector(p.get("pose_q"), 21, "pose_q")
    p["quaternion_xyzw"] = vector(p.get("quaternion_xyzw"), 4, "quaternion_xyzw")
    quaternion_angle(p["quaternion_xyzw"],p["quaternion_xyzw"])
    if type(p.get("workmode")) is not int or p["workmode"] not in (0,1,2,23):
        raise MassError("PROFILE_UNVERIFIED", "only verified stationary high-level modes or verified playback hold supported")
    if p["workmode"] == 23 and (p.get("playback_hold_verified") is not True or not p.get("hold_evidence")):
        raise MassError("PROFILE_UNVERIFIED", "playback hold requires hardware verification")
    if p.get("torque_units") not in ("sdk_raw", "Nm") or (p["torque_units"] == "Nm" and not p.get("units_evidence")):
        raise MassError("UNITS_UNVERIFIED", "use sdk_raw unless Nm has been verified")
    defaults = {"settle_timeout_s":10, "max_receive_gap_s":0.5, "min_samples":20,
                "baseline_max_age_s":3600, "result_max_age_s":30, "min_snr":5,
                "min_resolution_kg":0.001, "min_axis_response":1e-6}
    for key,val in defaults.items():
        p.setdefault(key,val)
    for key in ("pose_tolerance_rad", "body_tolerance_rad", "max_velocity_rad_s", "max_gyro_rad_s",
                "max_torque_span", "torque_abs_limit", "max_temperature_c", "max_temperature_drift_c",
                "max_mass_kg", "noise_floor_raw", "max_calibration_error_kg", *defaults):
        p[key] = finite(p.get(key),key,1e-9)
    if p["settle_timeout_s"]>60 or p["max_receive_gap_s"]>5 or p["baseline_max_age_s"]>86400 or p["result_max_age_s"]>300:
        raise MassError("BAD_PARAMETER", "timeout/freshness limits exceed supported bounds")
    if p["min_samples"] != int(p["min_samples"]) or not 5 <= p["min_samples"] <= 2000:
        raise MassError("BAD_PARAMETER", "min_samples must be an integer 5..2000")
    if p.get("mapping_verified") is not True:
        raise MassError("PROFILE_UNVERIFIED", "actual ID/URDF mapping has not been confirmed")
    if loading:
        for flag in ("load_range_confirmed", "feedback_verified"):
            if p.get(flag) is not True:
                raise MassError("PROFILE_UNVERIFIED", f"{flag} must be confirmed from hardware evidence")
        if not p.get("feedback_evidence"):
            raise MassError("PROFILE_UNVERIFIED", "load response / refresh evidence is required")
    if type(p.get("accept_host_only_freshness", False)) is not bool:
        raise MassError("BAD_PARAMETER", "accept_host_only_freshness must be boolean")
    p["profile_id"] = profile_id
    p["fingerprint"] = digest(p)
    return p


def summarize(frames, p):
    ids = p["motor_ids"]
    center = [median([f["tau"][j] for f in frames]) for j in ids]
    noise = [max(p["noise_floor_raw"], 1.4826*median([abs(f["tau"][j]-c) for f in frames])) for j,c in zip(ids,center)]
    return {"tau":center, "noise":noise,
            "temperature": [median([f["temperature"][j] for f in frames]) for j in range(21)],
            "pose_q": [median([f["q"][j] for f in frames]) for j in range(21)],
            "quaternion_xyzw": frames[-1]["quaternion_xyzw"], "workmode":frames[-1]["workmode"],
            "sample_count":len(frames), "effective_sample_count":None,
            "host_span_s":frames[-1]["host_monotonic_s"]-frames[0]["host_monotonic_s"],
            "freshness_status":"source_sequence" if all(f.get("source_seq") is not None for f in frames) else "source_timestamp" if all(f.get("source_timestamp") is not None for f in frames) else "host_receipt_only",
            "dispersion_definition":"per-axis 1.4826 * median absolute deviation, floored by configured noise_floor_raw"}


def check_baseline(baseline, summary, p, now):
    if baseline["profile_fingerprint"] != p["fingerprint"]:
        raise MassError("CONFIG_CHANGED", "tare profile differs from current engineering config")
    if not 0 <= now-baseline["created_s"] <= p["baseline_max_age_s"]:
        raise MassError("BASELINE_EXPIRED", "tare expired or wall clock moved backwards")
    b = baseline["summary"]
    if max(abs(a-bb) for a,bb in zip(summary["temperature"],b["temperature"])) > p["max_temperature_drift_c"]:
        raise MassError("TEMPERATURE_DRIFT", "temperature differs from tare")
    j = max(fixed_pose_motor_ids(p), key=lambda i: abs(summary["pose_q"][i]-b["pose_q"][i]))
    delta = abs(summary["pose_q"][j]-b["pose_q"][j])
    if delta > p["pose_tolerance_rad"]:
        raise MassError("POSE_MISMATCH", f"pose differs from tare: motor_id={j}, "
                        f"actual={summary['pose_q'][j]:.4f}, target={b['pose_q'][j]:.4f}, "
                        f"delta={delta:.4f}, limit={p['pose_tolerance_rad']:.4f} rad")
    if quaternion_angle(summary["quaternion_xyzw"], b["quaternion_xyzw"]) > p["body_tolerance_rad"]:
        raise MassError("BODY_MISMATCH", "body orientation differs from tare")


def projection(k,w,y):
    denom = sum(a*a*b for a,b in zip(k,w))
    if denom <= 1e-15:
        raise MassError("UNIDENTIFIABLE", "near-zero weighted mass sensitivity")
    return sum(a*b*c for a,b,c in zip(k,w,y))/denom


def fit_model(trials, baseline, p):
    trials = [t for t in trials if t.get("baseline_id") == baseline["id"] and t.get("action") == "calibrate"]
    if len(baseline["zero_trial_ids"]) < 2:
        raise MassError("INSUFFICIENT_CALIBRATION", "at least two independent empty trials required before calibration")
    train=[t for t in trials if t["role"]=="train"]
    valid=[t for t in trials if t["role"]=="validation"]
    groups=sorted(set(t["known_mass_kg"] for t in train if t["known_mass_kg"]>0))
    if len(groups)<2 or any(sum(t["known_mass_kg"]==m for t in train)<2 for m in groups):
        raise MassError("INSUFFICIENT_CALIBRATION", "two distinct positive training masses with two independent trials each required")
    vm=sorted(set(t["known_mass_kg"] for t in valid))
    if len(vm)<2 or vm[0]>groups[0] or vm[-1]<groups[-1]:
        raise MassError("INSUFFICIENT_VALIDATION", "whole held-out trials must cover training mass endpoints")
    if len({t["id"] for t in trials}) != len(trials):
        raise MassError("TRIAL_LEAKAGE", "trial IDs cannot overlap training and validation")
    if any(t["profile_fingerprint"]!=p["fingerprint"] for t in trials):
        raise MassError("CONFIG_CHANGED", "calibration trial context mismatch")
    n=len(p["motor_ids"])
    # Fit coefficients ONLY from training trials; held-out truth is never a feature.
    base=baseline["summary"]
    delta=lambda t:[a-b for a,b in zip(t["summary"]["tau"],base["tau"])]
    denominator=sum(t["known_mass_kg"]**2 for t in train)
    k=[sum(t["known_mass_kg"]*delta(t)[j] for t in train)/denominator for j in range(n)]
    noise=[max([base["noise"][j], p["noise_floor_raw"]]+[t["summary"]["noise"][j] for t in train]) for j in range(n)]
    weights=[1/s**2 if abs(a)>=p["min_axis_response"] and abs(a)*groups[0]>=p["min_snr"]*s else 0.0 for a,s in zip(k,noise)]
    if not any(weights):
        raise MassError("UNIDENTIFIABLE", "no joint has repeatable, resolvable load response")
    predict=lambda t:projection(k,weights,delta(t))
    means=[median([predict(t) for t in train if t["known_mass_kg"]==m]) for m in groups]
    if any(b-a <= p["min_resolution_kg"] for a,b in zip(means,means[1:])):
        raise MassError("NON_MONOTONIC", "load response is not distinguishable/monotonic")
    train_errors=[abs(predict(t)-t["known_mass_kg"]) for t in train]
    errors=[abs(predict(t)-t["known_mass_kg"]) for t in valid]
    if max(train_errors+errors)>p["max_calibration_error_kg"]:
        raise MassError("CALIBRATION_RESIDUAL", "independent-trial error exceeds configured acceptance limit")
    normalized_residual=lambda t:math.sqrt(sum(w*(y-a*predict(t))**2 for w,y,a in zip(weights,delta(t),k))/max(1,sum(w>0 for w in weights)))
    residual_limit=max(5.0, 3*max(normalized_residual(t) for t in train))
    if any(normalized_residual(t)>residual_limit for t in valid):
        raise MassError("CALIBRATION_RESIDUAL", "held-out torque vector is inconsistent with training response")
    resolution=max(p["min_resolution_kg"],max(errors),3/math.sqrt(sum(a*a*w for a,w in zip(k,weights))))
    if resolution>=groups[-1]:
        raise MassError("UNIDENTIFIABLE", "resolution exceeds calibrated range")
    return {"version":VERSION,"profile":p,"k":k,"weights":weights,"residual_limit":residual_limit,
            "resolution_kg":resolution,"mass_range_kg":[0,min(groups[-1],vm[-1],p["max_mass_kg"])],
            "training_trial_ids":[t["id"] for t in train], "validation_trial_ids":[t["id"] for t in valid],
            "validation":{"mae_kg":sum(errors)/len(errors),"max_abs_error_kg":max(errors),
                          "training_max_abs_error_kg":max(train_errors),
                          "repeatability_max_range_kg":max(max(predict(t) for t in train if t["known_mass_kg"]==m)-min(predict(t) for t in train if t["known_mass_kg"]==m) for m in groups),
                          "trial_count":len(valid),
                          "failure_rate":0,"scope":"accepted stable held-out calibration trials only; not operational failure rate"},
            "baseline_id":baseline["id"],"profile_fingerprint":p["fingerprint"]}


def estimate(summary, baseline, model, p, now):
    check_baseline(baseline,summary,p,now)
    if model["profile_fingerprint"]!=p["fingerprint"] or model["baseline_id"]!=baseline["id"]:
        raise MassError("CONFIG_CHANGED", "model baseline/profile mismatch")
    y=[a-b for a,b in zip(summary["tau"],baseline["summary"]["tau"])]
    mass=projection(model["k"],model["weights"],y)
    if mass < -model["resolution_kg"] or mass > model["mass_range_kg"][1]:
        raise MassError("OUT_OF_RANGE", "signed response outside independently validated range")
    if mass < model["resolution_kg"]:
        raise MassError("BELOW_RESOLUTION", "signal below demonstrated mass resolution; not an exact zero")
    residual=math.sqrt(sum(w*(v-a*mass)**2 for w,v,a in zip(model["weights"],y,model["k"]))/max(1,sum(w>0 for w in model["weights"])))
    if residual>model["residual_limit"]:
        raise MassError("RESIDUAL", "load torque vector differs from calibrated one-dimensional response")
    # Display granularity comes from observed error/noise, not a hardcoded 0.001 kg.
    digits=max(0,min(3,-math.floor(math.log10(model["resolution_kg"])) ))
    return {"mass_kg":round(mass,digits),"resolution_kg":model["resolution_kg"],
            "residual":residual,"residual_definition":"RMS torque-vector residual in per-axis noise units (selected axes)",
            "mass_range_kg":model["mass_range_kg"],"quality":"empirical_fixed_pose",
            "validation":model["validation"]}
