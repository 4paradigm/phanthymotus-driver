"""Offline displacement evaluation. No ROS, camera access or motion commands."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean

TOLERANCE_M = 0.05
MAX_GAP_NS = 500_000_000


def _finite(values, size):
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError("invalid vector")
    return result


def rotation(q):
    """Quaternion xyzw -> row-major rotation, accepting either quaternion sign."""
    q = _finite(q, 4)
    norm = math.hypot(*q)
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = (v / norm for v in q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def base_position(camera_position, camera_quaternion, camera_to_base_translation):
    p = _finite(camera_position, 3)
    offset = _finite(camera_to_base_translation, 3)
    r = rotation(camera_quaternion)
    return tuple(p[i] + sum(r[i][j] * offset[j] for j in range(3)) for i in range(3))


def project_displacement(start_position, end_position, start_quaternion):
    start, end = _finite(start_position, 3), _finite(end_position, 3)
    r = rotation(start_quaternion)
    norm = math.hypot(r[0][0], r[1][0])
    if norm < 1e-8:
        raise ValueError("forward axis has no horizontal projection")
    fx, fy = r[0][0] / norm, r[1][0] / norm
    dx, dy = end[0] - start[0], end[1] - start[1]
    return dx * fx + dy * fy, -dx * fy + dy * fx


def _quaternion(row):
    return tuple(row[k] for k in ("qx", "qy", "qz", "qw"))


def _position(row):
    return _finite((row[k] for k in ("x", "y", "z")), 3)


def _window(samples, bounds):
    if len(bounds) != 2 or any(type(v) is not int for v in bounds) or bounds[0] > bounds[1]:
        raise ValueError("invalid time window")
    rows = [row for row in samples if bounds[0] <= row["stamp_ns"] <= bounds[1]]
    if not rows:
        raise ValueError("empty endpoint window")
    # Endpoint samples must be stable; averaging a moving window biases the reference.
    positions = [_position(row) for row in rows]
    if any(max(p[i] for p in positions) - min(p[i] for p in positions) > 0.02
           for i in range(3)):
        raise ValueError("endpoint window is not stationary")
    return tuple(mean(p[i] for p in positions) for i in range(3)), _quaternion(rows[0])


def evaluate_trial(samples, truth):
    result = {k: truth.get(k) for k in ("trial_id", "scene", "speed_m_s", "split")}
    result["status"] = "invalid"
    try:
        if truth["split"] not in ("development", "acceptance"):
            raise ValueError("unknown dataset split")
        forward, lateral, uncertainty = _finite(
            (truth[k] for k in ("forward_m", "lateral_m", "uncertainty_m")), 3)
        if uncertainty < 0:
            raise ValueError("negative measurement uncertainty")
        if any(row.get("status") == "lost" for row in samples):
            # One recording is one trial. Late diagnostics can invalidate earlier predictions.
            raise ValueError("fault recorded in trial, including late diagnostics")
        previous = None
        normalized = []
        for row in samples:
            stamp = row["stamp_ns"]
            if isinstance(stamp, str):
                stamp = int(stamp)
            if type(stamp) is not int or (previous is not None and stamp <= previous):
                raise ValueError("timestamps must strictly increase")
            previous = stamp
            normalized.append(dict(row, stamp_ns=stamp))
        start, end = truth["start_ns"], truth["end_ns"]
        # Validate endpoint bounds before using them to select the whole trial.
        p0, q0 = _window(normalized, start)
        p1, _ = _window(normalized, end)
        if start[1] >= end[0]:
            raise ValueError("overlapping or reversed endpoint windows")
        segment = None
        previous = None
        for row in normalized:
            if not start[0] <= row["stamp_ns"] <= end[1]:
                continue
            if row["status"] != "tracking" or not row["segment_id"]:
                raise ValueError("tracking unavailable")
            if segment is not None and segment != row["segment_id"]:
                raise ValueError("odometry reset inside trial")
            segment = row["segment_id"]
            if previous is not None and row["stamp_ns"] - previous > MAX_GAP_NS:
                raise ValueError("odometry sample gap")
            previous = row["stamp_ns"]
            _position(row)
            rotation(_quaternion(row))
        estimate_forward, estimate_lateral = project_displacement(p0, p1, q0)
        error, lateral_error = estimate_forward - forward, estimate_lateral - lateral
        upper = abs(error) + uncertainty
        lower = max(0, abs(error) - uncertainty)
        status = ("passed" if upper <= TOLERANCE_M + 1e-12 else
                  "failed" if lower > TOLERANCE_M + 1e-12 else "inconclusive")
        result.update(status=status, estimated_forward_m=estimate_forward,
                      measured_forward_m=forward, estimated_lateral_m=estimate_lateral,
                      measured_lateral_m=lateral, forward_error_m=error,
                      lateral_error_m=lateral_error, horizontal_error_m=math.hypot(error, lateral_error),
                      uncertainty_m=uncertainty)
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        result["reason"] = str(exc)
    return result


def summarize(trials):
    acceptance = [t for t in trials if t.get("split") == "acceptance"]
    valid = [t for t in acceptance if t["status"] in ("passed", "failed", "inconclusive")]
    errors = [t["forward_error_m"] for t in valid]
    ids = [t.get("trial_id") for t in trials]
    return dict(
        trial_count=len(trials), acceptance_count=len(acceptance),
        valid_fraction=len(valid) / len(acceptance) if acceptance else 0,
        mean_error_m=mean(errors) if errors else None,
        rmse_m=math.sqrt(mean(e * e for e in errors)) if errors else None,
        max_absolute_error_m=max(map(abs, errors)) if errors else None,
        acceptance_passed=(len(acceptance) >= 20 and len(set(ids)) == len(ids)
                           and all(t["status"] == "passed" for t in acceptance)
                           and all(0.9 <= t["measured_forward_m"] <= 1.1 for t in acceptance)
                           and len({t.get("scene") for t in acceptance if t.get("scene")}) >= 2
                           and all(t.get("split") in ("development", "acceptance") for t in trials)),
    )


def evaluate_manifest(path):
    path = Path(path)
    manifest = json.loads(path.read_text())
    trials = []
    for item in manifest["trials"]:
        try:
            reference = json.loads((path.parent / item["truth"]).read_text())
            if reference["trial_id"] != item["trial_id"]:
                raise ValueError("manifest and truth trial_id differ")
            with (path.parent / item["samples"]).open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            trials.append(evaluate_trial(rows, reference))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # Missing data must never reduce the denominator or hide an attempt.
            trials.append(dict(trial_id=item.get("trial_id"), split="acceptance",
                               status="invalid", reason=str(exc)))
    summary = summarize(trials)
    return dict(data_source=manifest.get("data_source", "unknown"), trials=trials,
                summary=summary, physical_accuracy_validated=(
                    manifest.get("data_source") == "measured" and summary["acceptance_passed"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = evaluate_manifest(args.manifest)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"evaluation failed: {exc}\n")


if __name__ == "__main__":
    main()
