#!/usr/bin/env python3
"""Read-only Bumi scale report: no SDK initialization or motion commands.

Run inside the driver container. Saved weigh windows are historical observations,
not proof that a known reference was attached. Current pose/age are reported so
expired or returned-arm data is not mistaken for a valid load comparison.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean, median
import time
import urllib.request


def read_info(url):
    request = urllib.request.Request(url, data=json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "tools/call",
        "params": {"name": "payload_mass", "arguments": {"action": "info"}},
    }).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as response:
        envelope = json.load(response)
    if envelope.get("error"):
        raise ValueError("MCP error: " + json.dumps(envelope["error"], ensure_ascii=False))
    result = envelope.get("result", {})
    if result.get("isError"):
        raise ValueError("MCP tool error: " + json.dumps(result, ensure_ascii=False))
    for block in result.get("content", []):
        if block.get("type") == "text":
            data = json.loads(block["text"])
            if data.get("diagnostics", {}).get("latest"):
                return data
    raise ValueError("No scale diagnostics/latest frame in MCP response")


def torque_stats(frames, ids):
    result = []
    for j in ids:
        values = [f["tau"][j] for f in frames]
        result.append({"motor": j, "median": round(median(values), 6),
                       "mean": round(mean(values), 6),
                       "min": round(min(values), 6), "max": round(max(values), 6),
                       "nonzero_pct": round(100*sum(v != 0 for v in values)/len(values), 1)})
    return result


def report(url, root, samples=60):
    info = read_info(url)
    prepare = None
    recent = []
    for path in sorted(root.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            trial = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if trial.get("profile", {}).get("arm") != info.get("arm"):
            continue
        if trial.get("action") == "weigh" and len(recent) < 3:
            recent.append(trial)
        if trial.get("action") == "prepare":
            prepare = trial
            break
    if prepare is None:
        raise ValueError("No matching empty prepare trial under " + str(root))
    profile, baseline = prepare["profile"], prepare["summary"]
    ids = profile["motor_ids"]
    frames, seen = [], set()
    for _ in range(samples):
        info = read_info(url)
        frame = info["diagnostics"]["latest"]
        if frame["serial"] not in seen:
            seen.add(frame["serial"])
            frames.append(frame)
        time.sleep(.05)
    if not frames:
        raise ValueError("No current observations")
    def output(label, data):
        print(label + " " + json.dumps(data, ensure_ascii=False), flush=True)
    latest = frames[-1]
    output("SESSION", {"state": info["state"], "arm": info["arm"], "motor_ids": ids,
                       "remaining_s": info.get("remaining_s"),
                       "baseline_age_s": round(time.time()-prepare["created_s"], 1),
                       "workmode": latest["workmode"], "unique_observations": len(frames),
                       "source_age_s": info["diagnostics"]["host_age_s"]})
    drift = []
    for j in range(21):
        values = [f["temperature"][j] for f in frames]
        delta = max(abs(v-baseline["temperature"][j]) for v in values)
        drift.append({"motor": j, "measured_arm": j in ids,
                      "empty_c": baseline["temperature"][j],
                      "current_min_c": min(values), "current_max_c": max(values),
                      "max_delta_c": delta})
    output("TEMPERATURE", {"drift_limit_c": profile["max_temperature_drift_c"],
                           "absolute_limit_c": profile["max_temperature_c"],
                           "largest_changes": sorted(drift, key=lambda d: d["max_delta_c"], reverse=True)[:5]})
    output("EMPTY_TAU_RAW", torque_stats(prepare["raw_frames"], ids))
    for trial in reversed(recent):
        if trial.get("baseline", {}).get("id") != prepare["id"]:
            continue
        output("SAVED_WINDOW", {"seconds_after_empty": round(trial["created_s"]-prepare["created_s"], 1),
                                "tau_raw": torque_stats(trial["raw_frames"], ids)})
    output("CURRENT_POSE", {"max_arm_delta_from_empty_rad": round(max(
        abs(f["q"][j]-baseline["pose_q"][j]) for f in frames for j in ids), 5),
        "max_arm_span_rad": round(max(max(f["q"][j] for f in frames)-min(f["q"][j] for f in frames)
                                      for j in ids), 5)})
    output("CURRENT_TAU_RAW", torque_stats(frames, ids))
    print("NOTE: saved/current windows have no verified attached-weight label; pose and remaining time must match before comparing loads.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcp-url", default="http://127.0.0.1:15704/mcp")
    parser.add_argument("--trials-dir", type=Path, default=Path("/opt/phanthy-motus/data/bumi/weighing_trials"))
    args = parser.parse_args()
    try:
        report(args.mcp_url, args.trials_dir)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        raise SystemExit("DIAGNOSTIC_ERROR: " + str(exc))
