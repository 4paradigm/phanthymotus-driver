"""Distance must mean displacement, and failed trials must stay in the report."""
import copy
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.bumi_odometry.evaluate import (
    base_position, evaluate_trial, project_displacement, summarize,
)


def trajectory(distance=1.04):
    return [dict(stamp_ns=i * 100_000_000, x=distance * i / 10, y=0, z=0,
                 qx=0, qy=0, qz=0, qw=1, status="tracking", segment_id="one")
            for i in range(11)]


def truth():
    return dict(trial_id="a", start_ns=[0, 0], end_ns=[1_000_000_000, 1_000_000_000],
                forward_m=1.0, lateral_m=0.0, uncertainty_m=0.0,
                scene="room", speed_m_s=0.1, split="acceptance")


def test_distance_projects_on_start_forward_and_ignores_vertical_bobbing():
    q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
    assert project_displacement((0, 0, 0), (0, 1, 0.3), q) == pytest.approx((1, 0))
    assert project_displacement((0, 0, 0), (0, 1, 0.3), tuple(-x for x in q)) == pytest.approx((1, 0))


def test_camera_lever_arm_does_not_create_base_translation():
    assert base_position((0.2, 0, 0), (0, 0, 0, 1), (-0.2, 0, 0)) == pytest.approx((0, 0, 0))
    assert base_position((0, 0.2, 0), (0, 0, math.sqrt(0.5), math.sqrt(0.5)), (-0.2, 0, 0)) == pytest.approx((0, 0, 0))


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (0, float("nan"), 0, 1),
                              (0, math.sqrt(0.5), 0, math.sqrt(0.5))])
def test_unusable_start_orientation_is_rejected(q):
    with pytest.raises(ValueError):
        project_displacement((0, 0, 0), (1, 0, 0), q)


@pytest.mark.parametrize("distance,uncertainty,status", [
    (1.04, 0, "passed"), (1.06, 0, "failed"), (1.05, 0, "passed"),
    (1.04, 0.02, "inconclusive"), (1.06, 0.02, "inconclusive"),
])
def test_external_measurement_and_uncertainty_determine_result(distance, uncertainty, status):
    reference = truth()
    reference["uncertainty_m"] = uncertainty
    result = evaluate_trial(trajectory(distance), reference)
    assert result["status"] == status
    assert result["forward_error_m"] == pytest.approx(distance - 1)


@pytest.mark.parametrize("mutation", ["lost", "reset", "duplicate", "nan", "empty_window", "gap"])
def test_invalid_tracking_cannot_pass_distance_acceptance(mutation):
    rows, reference = trajectory(), truth()
    if mutation == "lost": rows[5]["status"] = "lost"
    if mutation == "reset": rows[5]["segment_id"] = "two"
    if mutation == "duplicate": rows[5]["stamp_ns"] = rows[4]["stamp_ns"]
    if mutation == "nan": rows[5]["x"] = float("nan")
    if mutation == "empty_window": reference["start_ns"] = [1, 2]
    if mutation == "gap": rows = rows[:2] + rows[9:]
    result = evaluate_trial(rows, reference)
    assert result["status"] == "invalid"
    assert result["reason"]


def test_report_retains_lateral_vector_error():
    rows = trajectory(1)
    rows[-1]["y"] = 0.1
    result = evaluate_trial(rows, truth())
    assert result["lateral_error_m"] == pytest.approx(0.1)
    assert result["horizontal_error_m"] == pytest.approx(0.1)


def test_acceptance_needs_twenty_trials_and_keeps_failures():
    result = evaluate_trial(trajectory(), truth())
    trials = [dict(copy.deepcopy(result), trial_id=str(i), scene=str(i % 2)) for i in range(20)]
    assert summarize(trials[:19])["acceptance_passed"] is False
    assert summarize(trials)["acceptance_passed"] is True
    trials[0]["status"] = "invalid"
    summary = summarize(trials)
    assert summary["acceptance_passed"] is False
    assert summary["trial_count"] == 20
    assert summary["valid_fraction"] == pytest.approx(0.95)


def test_cli_missing_trial_stays_in_report_and_synthetic_is_not_measurement(tmp_path):
    rows = trajectory()
    with (tmp_path / "a.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (tmp_path / "truth.json").write_text(json.dumps(truth()))
    manifest = dict(data_source="synthetic", trials=[
        dict(trial_id="a", samples="a.csv", truth="truth.json"),
        dict(trial_id="missing", samples="missing.csv", truth="truth.json"),
    ])
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    script = Path(__file__).resolve().parents[1] / "experiments/bumi_odometry/evaluate.py"
    run = subprocess.run([sys.executable, str(script), "--manifest", str(tmp_path / "manifest.json"),
                          "--output", str(tmp_path / "report.json")], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    report = json.loads((tmp_path / "report.json").read_text())
    assert len(report["trials"]) == 2
    assert report["trials"][1]["status"] == "invalid"
    assert report["physical_accuracy_validated"] is False
