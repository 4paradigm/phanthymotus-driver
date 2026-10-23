from __future__ import annotations

import json

import pytest

from pose_recorder import PoseRecorder


def _pose(timestamp=1000.25, person_id="p1", **person_fields):
    person = {"id": person_id, "posture": "standing", **person_fields}
    return json.dumps({"timestamp": timestamp, "persons": [person]},
                      separators=(",", ":"))


def test_ingest_preserves_raw_timestamps_people_and_future_fields():
    recorder = PoseRecorder(max_frames=3, clock_ns=lambda: 130)
    raw = _pose(person_id=7, keypoints=[[1, 2, 0.9]], future={"v": 2})

    assert recorder.ingest(raw, bridge_received_monotonic_ns=100,
                           driver_received_monotonic_ns=125)

    frame = recorder.recent_frames()[0]
    assert frame == {
        "raw_json": raw,
        "source_timestamp": 1000.25,
        "bridge_received_monotonic_ns": 100,
        "driver_received_monotonic_ns": 125,
        "persons": [{
            "id": 7,
            "posture": "standing",
            "keypoints": [[1, 2, 0.9]],
            "future": {"v": 2},
        }],
    }

    parsed = json.loads(raw)
    parsed["persons"][0]["future"]["v"] = 999
    assert recorder.recent_frames()[0]["persons"][0]["future"] == {"v": 2}


@pytest.mark.parametrize("raw", ["not json", "[]", "{}", '{"persons":{}}'])
def test_invalid_json_or_root_schema_is_rejected_without_stopping(raw):
    recorder = PoseRecorder()

    assert recorder.ingest(raw, 10, 11) is False
    assert recorder.ingest(_pose(), 12, 13) is True

    status = recorder.status()
    expected_json = 1 if raw == "not json" else 0
    assert status["invalid_json"] == expected_json
    assert status["invalid_schema"] == 1 - expected_json
    assert status["buffered_frames"] == 1


@pytest.mark.parametrize("timestamp", [None, "bad", float("nan"), float("inf")])
def test_invalid_source_timestamp_is_retained_as_null_and_counted(timestamp):
    recorder = PoseRecorder()
    payload = {"persons": [{"id": "p1", "posture": "standing"}]}
    if timestamp is not None:
        payload["timestamp"] = timestamp

    assert recorder.ingest(json.dumps(payload), 20, 21)

    assert recorder.recent_frames()[0]["source_timestamp"] is None
    assert recorder.status()["invalid_source_timestamps"] == 1


def test_missing_person_id_drops_only_that_person_and_counts_it():
    recorder = PoseRecorder()
    raw = json.dumps({
        "timestamp": 1,
        "persons": [
            {"posture": "standing"},
            {"id": "kept", "posture": "sitting"},
            "not-an-object",
        ],
    })

    assert recorder.ingest(raw, 30, 31)

    assert recorder.recent_frames()[0]["persons"] == [
        {"id": "kept", "posture": "sitting"}]
    assert recorder.status()["invalid_persons"] == 2


def test_driver_before_bridge_monotonic_is_counted_but_frame_is_retained():
    recorder = PoseRecorder()

    assert recorder.ingest(_pose(), 50, 49)

    assert recorder.status()["monotonic_anomalies"] == 1
    assert recorder.recent_frames()[0]["bridge_received_monotonic_ns"] == 50


def test_ingest_ring_is_bounded_and_reports_recorder_evictions_separately():
    recorder = PoseRecorder(max_frames=2)
    for index in range(3):
        assert recorder.ingest(_pose(timestamp=index, person_id=index),
                               100 + index, 200 + index)

    assert [frame["persons"][0]["id"]
            for frame in recorder.recent_frames()] == [1, 2]
    assert recorder.status(bridge_transport_drops=7) == {
        "active_round": None,
        "buffered_frames": 2,
        "max_frames": 2,
        "frozen_rounds": 0,
        "max_rounds": 20,
        "max_frames_per_round": 600,
        "max_raw_bytes": 1048576,
        "bridge_transport_drops": 7,
        "recorder_cache_evictions": 1,
        "frozen_round_evictions": 0,
        "snapshot_frame_truncations": 0,
        "invalid_json": 0,
        "invalid_schema": 0,
        "invalid_source_timestamps": 0,
        "invalid_persons": 0,
        "monotonic_anomalies": 0,
        "oversize_messages": 0,
    }


def test_oversize_pose_is_rejected_before_json_parsing_and_counted():
    recorder = PoseRecorder(max_raw_bytes=32)

    assert recorder.ingest(_pose(person_id="x" * 100), 1, 2) is False
    assert recorder.status()["oversize_messages"] == 1
    assert recorder.status()["buffered_frames"] == 0


def test_round_window_uses_inclusive_bridge_monotonic_boundaries_only():
    recorder = PoseRecorder(max_frames=10)
    recorder.start_round("r1", started_monotonic_ns=100)
    for bridge_ns, source_timestamp in (
            (99, 9_999_999), (100, -500), (200, 1), (201, 0)):
        recorder.ingest(_pose(timestamp=source_timestamp,
                              person_id=str(bridge_ns)),
                        bridge_ns, bridge_ns + 10)

    recorder.finish_round("r1", "PLAYBACK_FAILED", ended_monotonic_ns=200)

    exported = recorder.export_round("r1")
    assert [frame["bridge_received_monotonic_ns"]
            for frame in exported["frames"]] == [100, 200]
    assert [frame["source_timestamp"] for frame in exported["frames"]] == [-500.0, 1.0]


def test_completed_round_requires_trusted_evidence_and_full_three_second_window():
    recorder = PoseRecorder()
    trusted = {
        "result": "completed",
        "completion_level": "alsa_device_consumption",
        "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
        # Deliberately a different clock magnitude; it is metadata only.
        "playback_finished_at_ns": 999_999_999_999,
    }
    recorder.start_round("too_early", 10)
    with pytest.raises(ValueError, match="three-second"):
        recorder.finish_round(
            "too_early", "COMPLETED", ended_monotonic_ns=3_000_000_019,
            playback_result=trusted, completion_observed_monotonic_ns=20)
    recorder.finish_round("too_early", "TIMEOUT", 30)

    recorder.start_round("untrusted", 40)
    with pytest.raises(ValueError, match="trusted"):
        recorder.finish_round(
            "untrusted", "COMPLETED", ended_monotonic_ns=3_000_000_050,
            playback_result={"result": "completed"},
            completion_observed_monotonic_ns=50)
    recorder.finish_round("untrusted", "TIMEOUT", 60)

    recorder.start_round("complete", 100)
    recorder.finish_round(
        "complete", "COMPLETED", ended_monotonic_ns=3_000_000_200,
        playback_result=trusted, completion_observed_monotonic_ns=200)
    assert recorder.export_round("complete")["outcome"] == "COMPLETED"


@pytest.mark.parametrize("updates", [
    {"playback_finished_at_ns": True},
    {"playback_finished_at_ns": "100"},
    {"playback_finished_at_ns": -1},
    {"error_code": "ALSA_UNDERRUN"},
    {"error_code": 0},
    {"error_message": "drain failed"},
    {"error_message": []},
    {"underrun_count": 1},
])
def test_completed_round_rejects_malformed_or_terminal_error_evidence(updates):
    recorder = PoseRecorder()
    evidence = {
        "result": "completed",
        "completion_level": "alsa_device_consumption",
        "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
        "playback_finished_at_ns": 10,
        "error_code": None,
        "error_message": None,
        "underrun_count": 0,
    }
    evidence.update(updates)
    recorder.start_round("bad", 0)

    with pytest.raises(ValueError, match="trusted playback evidence"):
        recorder.finish_round(
            "bad", "COMPLETED", 3_000_000_000, evidence, 0)


def test_completed_round_rejects_boolean_local_completion_observation():
    recorder = PoseRecorder()
    recorder.start_round("bad-observation", 0)

    with pytest.raises(ValueError, match="local completion observation"):
        recorder.finish_round("bad-observation", "COMPLETED", 3_000_000_001, {
            "result": "completed",
            "completion_level": "alsa_device_consumption",
            "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
            "playback_finished_at_ns": 1,
        }, True)


def test_rounds_cannot_overlap_or_reuse_a_retained_id():
    recorder = PoseRecorder()
    recorder.start_round("r1", 1)
    with pytest.raises(RuntimeError, match="already active"):
        recorder.start_round("r2", 2)
    recorder.finish_round("r1", "MANUAL_TERMINATED", 3)
    with pytest.raises(ValueError, match="already exists"):
        recorder.start_round("r1", 4)


def test_next_round_does_not_overwrite_or_reclassify_previous_snapshot():
    recorder = PoseRecorder(max_rounds=3)
    recorder.start_round("r1", 100)
    recorder.ingest(_pose(person_id="r1"), 110, 111)
    recorder.finish_round("r1", "PLAYBACK_FAILED", 120)

    recorder.start_round("r2", 200)
    recorder.ingest(_pose(person_id="r2"), 210, 211)
    recorder.finish_round("r2", "TIMEOUT", 220)

    assert [f["persons"][0]["id"]
            for f in recorder.export_round("r1")["frames"]] == ["r1"]
    assert [f["persons"][0]["id"]
            for f in recorder.export_round("r2")["frames"]] == ["r2"]


def test_live_capacity_eviction_marks_round_snapshot_incomplete():
    recorder = PoseRecorder(max_frames=2)
    recorder.start_round("r1", 100)
    for value in (101, 102, 103):
        recorder.ingest(_pose(person_id=value), value, value)
    recorder.finish_round("r1", "PLAYBACK_FAILED", 103)

    exported = recorder.export_round("r1")
    assert exported["complete"] is False
    assert [f["persons"][0]["id"] for f in exported["frames"]] == [102, 103]


def test_evicting_only_pre_round_frames_does_not_mark_round_incomplete():
    recorder = PoseRecorder(max_frames=2)
    recorder.ingest(_pose(person_id="old-1"), 1, 1)
    recorder.ingest(_pose(person_id="old-2"), 2, 2)
    recorder.start_round("fresh", 100)
    recorder.ingest(_pose(person_id="fresh"), 101, 101)
    recorder.finish_round("fresh", "PLAYBACK_FAILED", 101)

    assert recorder.export_round("fresh")["complete"] is True


def test_round_snapshot_marks_bridge_transport_drops_incomplete():
    recorder = PoseRecorder()
    recorder.start_round("dropped", 100, bridge_transport_drops=4)
    recorder.ingest(_pose(person_id="kept"), 101, 101)
    snapshot = recorder.finish_round(
        "dropped", "PLAYBACK_FAILED", 102,
        bridge_transport_drops=6)

    assert snapshot["complete"] is False
    assert snapshot["bridge_transport_drops"] == 2
    assert snapshot["incomplete_reasons"] == ["bridge_transport_drop"]


def test_snapshot_and_completed_round_capacities_are_bounded_and_counted():
    recorder = PoseRecorder(max_frames=10, max_rounds=2,
                            max_frames_per_round=2)
    recorder.start_round("wide", 1)
    for value in (2, 3, 4):
        recorder.ingest(_pose(person_id=value), value, value)
    recorder.finish_round("wide", "PLAYBACK_FAILED", 4)
    assert recorder.export_round("wide")["complete"] is False
    assert [f["persons"][0]["id"]
            for f in recorder.export_round("wide")["frames"]] == [3, 4]

    for round_id, timestamp in (("r2", 10), ("r3", 20)):
        recorder.start_round(round_id, timestamp)
        recorder.finish_round(round_id, "MANUAL_TERMINATED", timestamp + 1)

    with pytest.raises(KeyError):
        recorder.export_round("wide")
    status = recorder.status()
    assert status["frozen_rounds"] == 2
    assert status["snapshot_frame_truncations"] == 1
    assert status["frozen_round_evictions"] == 1
