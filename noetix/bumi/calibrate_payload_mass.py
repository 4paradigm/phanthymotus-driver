#!/usr/bin/env python3
"""Fit signed raw-torque/kg sensitivities from independent saved trial files.

This offline program never connects to or controls the robot. Manifest masses
are reference-scale labels for calibration, never inputs to unknown weighing.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

from payload_mass_estimator import (check_baseline, fit_model, summarize,
                                    validate_profile)
from payload_mass_sampling import MassError, check_frame, finite


def fit(config, manifest):
    p = validate_profile("weighing", config["profile"], config["profile"]["arm"], loading=True)
    records, seen = [], set()
    for entry in manifest:
        trial = json.loads(Path(entry["file"]).read_text())
        if trial["id"] in seen:
            raise MassError("TRIAL_LEAKAGE", "每个独立试次只能使用一次，不能复制或拆分同一试次。")
        seen.add(trial["id"])
        if entry.get("independent") is not True or not entry.get("note"):
            raise MassError("TRIAL_LEAKAGE", "记录重新挂载试次说明并确认 independent=true。")
        if trial["profile_fingerprint"] != p["fingerprint"]:
            raise MassError("CONFIG_CHANGED", "试次与配置不匹配。")
        for frame in trial["raw_frames"]:
            check_frame(frame, p)
        if len(trial["raw_frames"]) < p["min_samples"]:
            raise MassError("INSUFFICIENT_SAMPLES", "试次观测数不足。")
        summary = summarize(trial["raw_frames"], p)
        if summary["host_span_s"] < config.get("sample_duration_s", 3):
            raise MassError("INSUFFICIENT_SAMPLES", "试次时间不足。")
        for j in p["motor_ids"]:
            values = [f["tau"][j] for f in trial["raw_frames"]]
            if max(values) - min(values) > p["max_torque_span"]:
                raise MassError("UNSTABLE", "试次力矩波动过大。")
        mass = finite(entry["mass_kg"], "mass_kg", 0, p["max_mass_kg"])
        role = entry.get("role", "train")
        if role not in ("train", "validation"):
            raise MassError("BAD_PARAMETER", "role must be train or validation")
        if (mass == 0 and trial["action"] != "prepare") or (mass > 0 and trial["action"] != "weigh"):
            raise MassError("BAD_DATA", "零质量须为空载prepare；正质量须为挂载weigh。")
        records.append({**trial, "summary": summary, "known_mass_kg": mass, "role": role})
    zeros = [t for t in records if t["known_mass_kg"] == 0]
    if len(zeros) < 2:
        raise MassError("INSUFFICIENT_CALIBRATION", "需要至少两个独立空载试次。")
    baseline = {"id": zeros[-1]["id"], "profile_fingerprint": p["fingerprint"],
                "created_s": zeros[-1]["created_s"], "summary": zeros[-1]["summary"],
                "zero_trial_ids": [z["id"] for z in zeros]}
    for trial in records:
        check_baseline(baseline, trial["summary"], p, baseline["created_s"])
    for zero in zeros:
        if any(abs(a-b) > p["max_torque_span"] for a, b in zip(zero["summary"]["tau"], baseline["summary"]["tau"])):
            raise MassError("BASELINE_DRIFT", "独立空载试次发生漂移。")
    trials = [{**t, "action": "calibrate", "baseline_id": baseline["id"]}
              for t in records if t["known_mass_kg"] > 0]
    model = fit_model(trials, baseline, p)
    output = copy.deepcopy(config)
    output["model"] = model
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--manifest", required=True, help="JSON list: file, mass_kg, role, independent, note")
    parser.add_argument("--output", required=True, help="new file; refuses to overwrite")
    args = parser.parse_args()
    result = fit(json.loads(Path(args.config).read_text()), json.loads(Path(args.manifest).read_text()))
    with Path(args.output).open("x") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps(result["model"]["validation"], ensure_ascii=False))


if __name__ == "__main__":
    main()
