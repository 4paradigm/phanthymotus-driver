"""Guardrails for Q5 pose_game Bot review findings."""
import json
from pathlib import Path

import pytest
from pose_recorder import PoseRecorder


def test_pose_recorder_hard_caps_and_total_raw_budget():
    with pytest.raises(ValueError, match="hard cap"):
        PoseRecorder(max_raw_bytes=1_048_576)
    with pytest.raises(ValueError, match="budget"):
        PoseRecorder(max_frames=1000, max_rounds=20,
                     max_frames_per_round=1000, max_raw_bytes=8_192)
    recorder = PoseRecorder()
    assert recorder.status()["max_frames"] <= 300


def test_pose_recorder_rejects_oversized_nested_persons():
    recorder = PoseRecorder()
    huge = json.dumps({"timestamp": 1.0, "persons": [{"id": 1,
                      "nested": list(range(600))}]})
    assert recorder.ingest(huge, 100, 101) is False
    assert recorder.status()["invalid_schema"] == 1


def test_pose_game_listed_in_driver_manifest():
    yaml_text = (Path(__file__).parent / "driver.yaml").read_text(encoding="utf-8")
    assert "name: pose_game" in yaml_text
    assert "type: sensor" in yaml_text
