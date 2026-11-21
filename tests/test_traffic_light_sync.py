"""Exact ROS-stamp joins and bounded temporal identity; real JPEG decoding."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from common.traffic_light.processor import TrafficLightProcessor
from test_traffic_light import Clock, detection, engine, jpeg, lamp, observe, ros, stamp
from common.traffic_light.plugin import TrafficLightPlugin


@pytest.mark.parametrize("field,value", [
    ("source_mode", None), ("source_mode", "camera"),
    ("stream_id", None), ("stream_id", ""), ("stream_id", " "),
    ("stream_id", []), ("frame_seq", None), ("frame_seq", True),
    ("frame_seq", -1), ("frame_seq", 1.5), ("frame_seq", 2**63),
    ("source_clock_status", None), ("source_clock_status", "valid"),
    ("source_clock_status", "regressed"), ("temporal_eligible", None),
    ("temporal_eligible", 1), ("temporal_eligible", "true"),
    ("source_stamp_semantics", "publication_time"),
    ("source_stamp", {"sec": -1, "nanosec": 1}),
    ("source_stamp", {"sec": 0, "nanosec": 0}),
    ("source_stamp", {"sec": 1000, "nanosec": True}),
])
def test_invalid_provenance_never_joins_even_when_rgb_and_publish_time_match(engine, field, value):
    processor, clock = engine
    payload = detection(clock)
    processor.image(jpeg(lamp()), payload["source_stamp"])
    payload.update(timestamp=clock.now(), **{field: value})
    result = processor.detection(payload)
    assert result["state"] in ("error", "waiting")
    assert not processor.tracks and not processor.pending
    assert all(s["color"] == "unknown" for s in result["signals"])


def test_old_protocol_is_compatible_but_cannot_claim_an_rgb_frame(engine):
    processor, clock = engine
    payload = detection(clock)
    processor.image(jpeg(lamp()), payload["source_stamp"])
    result = processor.detection({"timestamp": clock.now(), "count": 1,
                                  "objects": payload["objects"], "latency_ms": 32})
    assert result["state"] == "waiting"
    assert result["reason"] == "source_provenance_unavailable"
    assert result["frame_stamp"] is None and not processor.tracks


@pytest.mark.parametrize("offset", [-100_000_000, -1, 1, 100_000_000])
def test_no_nearest_frame_or_float_rounding_substitution(engine, offset):
    processor, clock = engine
    processor.image(jpeg(lamp()), stamp(clock.ns()))
    payload = detection(clock, source=clock.ns()+offset)
    result = processor.detection(payload)
    assert result["state"] in ("waiting", "stale")
    assert not processor.tracks
    assert result["signals"][0]["color"] == "unknown"


def test_out_of_order_detections_cannot_append_older_votes(engine):
    processor, clock = engine
    older = detection(clock, seq=1)
    processor.image(jpeg(lamp("green")), older["source_stamp"])
    clock.advance(.1)
    newer = detection(clock, seq=2)
    processor.image(jpeg(lamp("red")), newer["source_stamp"])
    assert processor.detection(newer)["signals"][0]["observations"] == 1
    result = processor.detection(older)
    assert result["reason"] == "source_stamp_regressed"
    assert result["state"] == "stale"
    assert all(not track.history for track in processor.tracks.values())


@pytest.mark.parametrize("change", ["seq", "stamp", "regressed_seq"])
def test_one_sequence_and_one_stamp_identify_one_exposure(engine, change):
    processor, clock = engine
    first = detection(clock, seq=10)
    processor.image(jpeg(lamp()), first["source_stamp"])
    processor.detection(first)
    second = deepcopy(first)
    if change == "seq":
        second["frame_seq"] = 11
    else:
        clock.advance(.1)
        second["source_stamp"] = stamp(clock.ns())
        processor.image(jpeg(lamp()), second["source_stamp"])
        if change == "regressed_seq":
            second["frame_seq"] = 9
    result = processor.detection(second)
    assert result["reason"] == ("frame_seq_regressed" if change == "regressed_seq"
                                  else "conflicting_frame_identity")
    assert result["signals"][0]["color"] == "unknown"
    assert all(not track.history for track in processor.tracks.values())


def test_pending_duplicate_keeps_first_payload_and_first_receive_age(engine):
    processor, clock = engine
    first = detection(clock, seq=1)
    processor.detection(first)
    received = next(iter(processor.pending.values()))[1]
    second = deepcopy(first)
    second["objects"] = []
    clock.advance(.1)
    processor.detection(second)
    assert next(iter(processor.pending.values()))[1] == received
    matched = processor.image(jpeg(lamp()), first["source_stamp"])
    assert len(matched) == 1 and len(matched[0]["signals"]) == 1
    assert matched[0]["signals"][0]["observations"] == 1
    clock.advance(1.1)
    assert processor.detection(first)["state"] == "stale"
    assert not processor.frames and not processor.pending


def test_pending_sequence_collision_does_not_replace_original(engine):
    processor, clock = engine
    first = detection(clock, seq=1)
    processor.detection(first)
    clock.advance(.1)
    second = detection(clock, seq=1)
    result = processor.detection(second)
    assert result["reason"] == "conflicting_frame_identity"
    assert len(processor.pending) == 1
    assert processor.image(jpeg(lamp()), second["source_stamp"]) == []
    assert processor.image(jpeg(lamp()), first["source_stamp"])[0]["frame_stamp"] == first["source_stamp"]


def test_detection_before_rgb_each_time_still_needs_and_accumulates_three_real_frames(engine):
    processor, clock = engine
    for index in range(3):
        payload = detection(clock, seq=index+1)
        assert processor.detection(payload)["reason"] == "matching_image_unavailable"
        result = processor.image(jpeg(lamp()), payload["source_stamp"])[0]
        assert result["signals"][0]["observations"] == index+1
        assert result["signals"][0]["color"] == ("red" if index == 2 else "unknown")
        clock.advance(.1)


def test_stream_change_isolates_votes_and_rejects_retired_epoch(engine):
    processor, clock = engine
    for _ in range(3):
        stable = observe(engine, stream="old")
    assert stable["signals"][0]["color"] == "red"
    fresh = observe(engine, stream="new")
    assert fresh["signals"][0]["observations"] == 1
    assert fresh["signals"][0]["color"] == "unknown"
    retired = observe(engine, stream="old")
    assert retired["reason"] == "retired_stream"


def test_invalid_duplicate_cannot_return_an_old_stable_color(engine):
    processor, clock = engine
    for _ in range(3):
        payload = detection(clock)
        observe(engine)
    payload["temporal_eligible"] = False
    result = processor.detection(payload)
    assert result["state"] == "waiting"
    assert result["signals"][0]["color"] == "unknown"
    assert all(not track.history for track in processor.tracks.values())


def test_topic_provenance_is_checked_without_breaking_existing_stamped_contract(engine):
    processor, clock = engine
    processor.rgb_topic = "/ubuntu/camera/color"
    payload = detection(clock)
    processor.image(jpeg(lamp()), payload["source_stamp"])
    payload["source_topic"] = "/other/camera/color"
    assert processor.detection(payload)["reason"] == "source_topic_mismatch"
    payload["source_topic"] = processor.rgb_topic
    assert processor.detection(payload)["state"] == "ok"


def test_stop_restart_and_multi_instance_do_not_share_pending_rgb_or_votes(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    args = {"instance_id": "a", "input_topic": "/ubuntu/camera/color/objects"}
    plugin.dispatch("start", args)
    plugin.dispatch("start", {"instance_id": "b", "input_topic": "/other/camera/color/objects"})
    old, other = plugin._instances["a"], plugin._instances["b"]
    # Runtime engines use real monotonic time, but the fake ROS clock determines age.
    payload = detection(ros.clock)
    old.engine.detection(payload)
    assert not other.engine.pending
    plugin.dispatch("stop", {"instance_id": "a"})
    ros.clock.advance(.1)
    plugin.dispatch("start", args)
    new = plugin._instances["a"]
    assert new is not old and not new.engine.frames and not new.engine.pending
    assert new.engine.detection(payload)["reason"] == "before_observation_start"
    before = len(old.node.messages)
    old.image(SimpleNamespace(data=jpeg(lamp()), header=SimpleNamespace(
        stamp=SimpleNamespace(**payload["source_stamp"]))))
    assert len(old.node.messages) == before
    assert not other.engine.frames and not other.engine.tracks
    plugin.shutdown()
