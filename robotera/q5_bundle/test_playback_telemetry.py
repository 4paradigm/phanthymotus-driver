import io
import threading
from urllib.parse import quote

import pytest

from playback_telemetry import (
    PlaybackTracker,
    decode_packet,
    encode_packet,
    parse_audio_format,
)


def tracked(speech_id="speech_A", sequence=0, eof=False):
    return parse_audio_format(
        f"audio/pcm-16k;speech_id={quote(speech_id, safe='')};"
        f"sequence={sequence};eof={int(eof)}"
    )


def test_audio_format_preserves_legacy_and_parses_tracking_metadata():
    legacy = parse_audio_format("audio/pcm-16k")
    assert legacy.valid and not legacy.tracked
    assert legacy.base_format == "audio/pcm-16k"

    metadata = parse_audio_format(
        "audio/pcm-16k;speech_id=hello%2Fworld;sequence=7;eof=1"
    )
    assert metadata.valid and metadata.tracked
    assert metadata.speech_id == "hello/world"
    assert metadata.sequence == 7
    assert metadata.eof is True


@pytest.mark.parametrize("value", [
    "audio/pcm-16k;speech_id=x;eof=1",
    "audio/pcm-16k;speech_id=x;sequence=-1;eof=0",
    "audio/pcm-16k;speech_id=;sequence=0;eof=0",
    "audio/pcm-16k;speech_id=x;sequence=0;eof=maybe",
    "audio/pcm-16k;speech_id=x;sequence=0;unknown=1",
])
def test_audio_format_rejects_malformed_tracking_without_falling_back(value):
    metadata = parse_audio_format(value)
    assert metadata.valid is False
    assert metadata.tracked is False
    assert metadata.error


def test_packet_round_trip_keeps_metadata_separate_from_pcm():
    metadata = tracked("speech%20literal", sequence=3, eof=True)
    pcm = b"\x00\n\xff\x7d" * 20
    packet = encode_packet(metadata, pcm)

    decoded, decoded_pcm = decode_packet(io.BytesIO(packet))
    assert decoded.speech_id == "speech%20literal"
    assert decoded.sequence == 3
    assert decoded.eof is True
    assert decoded_pcm == pcm


def test_streaming_progress_is_unknown_until_eof_and_write_is_not_completion():
    tracker = PlaybackTracker(clock_ns=lambda: 1000)
    tracker.receive_chunk(tracked(sequence=0), input_samples=1600)
    tracker.record_submitted("speech_A", submitted_frames=4410, generation=1,
                             remote_timestamp_ns=10)

    status = tracker.get_status("speech_A")
    assert status["state"] == "PCM_SUBMITTED"
    assert status["total_output_frames"] is None
    assert status["progress"] is None

    tracker.receive_chunk(tracked(sequence=1, eof=True), input_samples=1600)
    tracker.record_submitted("speech_A", submitted_frames=8820,
                             total_output_frames=8820, generation=1,
                             remote_timestamp_ns=20)
    assert tracker.get_status("speech_A")["state"] == "PCM_SUBMITTED"

    tracker.record_device_progress("speech_A", consumed_frames=4410,
                                   generation=1, remote_timestamp_ns=30)
    status = tracker.get_status("speech_A")
    assert status["state"] == "PLAYING"
    assert status["progress"] == 0.5

    tracker.record_device_progress("speech_A", consumed_frames=8820,
                                   generation=1, remote_timestamp_ns=40)
    assert tracker.get_status("speech_A")["state"] == "COMPLETED"


def test_playing_state_does_not_regress_when_more_pcm_is_submitted():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", 0), input_samples=100)
    tracker.record_submitted("A", 275, generation=1, remote_timestamp_ns=10)
    tracker.record_device_progress(
        "A", 100, 1, 20, evidence="snd_pcm_delay")
    assert tracker.get_status("A")["state"] == "PLAYING"

    tracker.receive_chunk(tracked("A", 1, True), input_samples=100)
    tracker.record_submitted("A", 550, 550, 1, 30)

    assert tracker.get_status("A")["state"] == "PLAYING"


def test_duplicate_submission_fact_is_not_emitted_twice():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", 0), input_samples=100)
    assert tracker.record_submitted("A", 275, generation=1,
                                    remote_timestamp_ns=10)
    event_count = len(tracker.get_recent_events("A"))

    assert tracker.record_submitted("A", 275, generation=1,
                                    remote_timestamp_ns=11) is False
    assert len(tracker.get_recent_events("A")) == event_count


def test_xrun_tail_is_trusted_only_when_it_reaches_known_eof_total():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", 0, True), input_samples=100)
    tracker.record_submitted("A", 275, 275, 1, 10)

    tracker.record_device_progress(
        "A", 275, 1, 20, evidence="snd_pcm_xrun_tail")

    assert tracker.get_status("A")["state"] == "COMPLETED"


def test_completion_requires_eof_known_total_and_trusted_device_evidence():
    tracker = PlaybackTracker(clock_ns=lambda: 1000)
    tracker.receive_chunk(tracked("missing_eof"), input_samples=100)
    tracker.record_submitted("missing_eof", 275, generation=1,
                             remote_timestamp_ns=10)
    tracker.record_device_progress("missing_eof", 275, 1, 20,
                                   evidence="snd_pcm_delay")
    assert tracker.get_status("missing_eof")["state"] != "COMPLETED"

    tracker.receive_chunk(tracked("unknown_total", eof=True), input_samples=100)
    tracker.record_submitted("unknown_total", 275, generation=1,
                             remote_timestamp_ns=30)
    tracker.record_device_progress("unknown_total", 275, 1, 40,
                                   evidence="snd_pcm_delay")
    assert tracker.get_status("unknown_total")["state"] != "COMPLETED"

    tracker.receive_chunk(tracked("untrusted", eof=True), input_samples=100)
    tracker.record_submitted("untrusted", 275, 275, 1, 50)
    tracker.record_device_progress("untrusted", 275, 1, 60,
                                   evidence=None)
    status = tracker.get_status("untrusted")
    assert status["state"] != "COMPLETED"
    assert status["device_progress_trusted"] is False


def test_failed_speech_ignores_late_trusted_progress():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", eof=True), input_samples=100)
    tracker.record_submitted("A", 275, 275, 1, 10)
    tracker.record_underrun("A", 1, 20)

    assert tracker.record_device_progress(
        "A", 275, 1, 30, evidence="snd_pcm_drain") is False
    assert tracker.get_status("A")["state"] == "FAILED"


def test_three_speeches_complete_independently_and_terminal_state_does_not_regress():
    tracker = PlaybackTracker(clock_ns=lambda: 2000)
    for index, speech_id in enumerate(("A", "B", "C")):
        tracker.receive_chunk(tracked(speech_id, 0, True), input_samples=1600)
        tracker.record_submitted(speech_id, submitted_frames=4410,
                                 total_output_frames=4410, generation=1,
                                 remote_timestamp_ns=100 + index)

    tracker.record_device_progress("A", 4410, 1, 110)
    assert [tracker.get_status(item)["state"] for item in ("A", "B", "C")] == [
        "COMPLETED", "PCM_SUBMITTED", "PCM_SUBMITTED"
    ]
    tracker.record_device_progress("B", 100, 1, 120)
    tracker.record_device_progress("A", 1, 1, 130)
    assert tracker.get_status("A")["state"] == "COMPLETED"
    assert tracker.get_status("B")["state"] == "PLAYING"


def test_duplicate_or_out_of_order_chunks_are_rejected_without_double_counting():
    tracker = PlaybackTracker(clock_ns=lambda: 3000)
    assert tracker.receive_chunk(tracked(sequence=0), input_samples=100)
    assert tracker.receive_chunk(tracked(sequence=0), input_samples=100) is False
    assert tracker.receive_chunk(tracked(sequence=2), input_samples=100) is False
    assert tracker.get_status("speech_A")["total_input_samples"] == 100
    assert tracker.get_status("speech_A")["error_code"] == "CHUNK_SEQUENCE_ERROR"
    assert tracker.get_status("speech_A")["state"] == "FAILED"


def test_duplicate_chunk_does_not_poison_later_valid_completion():
    tracker = PlaybackTracker()
    assert tracker.receive_chunk(tracked("A", 0), input_samples=100)
    assert tracker.receive_chunk(tracked("A", 0), input_samples=100) is False
    assert tracker.receive_chunk(tracked("A", 1, True), input_samples=100)
    tracker.record_submitted("A", 550, 550, 1, 10)
    tracker.record_device_progress("A", 550, 1, 20, evidence="snd_pcm_delay")

    status = tracker.get_status("A")
    assert status["state"] == "COMPLETED"
    assert status["duplicate_chunk_count"] == 1


def test_first_nonzero_sequence_is_tombstoned_and_later_zero_is_rejected():
    tracker = PlaybackTracker()
    assert tracker.receive_chunk(tracked("A", 1), input_samples=100) is False
    assert tracker.receive_chunk(tracked("A", 0, True), input_samples=100) is False

    status = tracker.get_status("A")
    assert status["state"] == "FAILED"
    assert status["error_code"] == "CHUNK_SEQUENCE_ERROR"


def test_active_speech_capacity_rejects_new_task_without_evicting_active():
    tracker = PlaybackTracker(max_speeches=2)
    assert tracker.receive_chunk(tracked("A", 0), input_samples=100)
    assert tracker.receive_chunk(tracked("B", 0), input_samples=100)
    assert tracker.receive_chunk(tracked("C", 0), input_samples=100) is False

    assert tracker.get_status("A")["state"] == "SPEAKER_RECEIVED"
    assert tracker.get_status("B")["state"] == "SPEAKER_RECEIVED"
    assert tracker.get_status("C")["error_code"] == "PLAYBACK_CAPACITY"


def test_empty_speech_fails_instead_of_reporting_device_completion():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked(eof=True), input_samples=0)
    tracker.record_submitted("speech_A", 0, 0, 1, 10, 44100)
    status = tracker.get_status("speech_A")
    assert status["state"] == "FAILED"
    assert status["error_code"] == "EMPTY_PCM"


def test_remote_and_local_monotonic_timestamps_remain_in_separate_clock_domains():
    times = iter((5000, 6000, 7000))
    tracker = PlaybackTracker(clock_ns=lambda: next(times))
    tracker.receive_chunk(tracked(eof=True), input_samples=100)
    tracker.record_submitted("speech_A", 275, 275, 2, 123)

    event = tracker.get_recent_events("speech_A")[-1]
    assert event["source_clock_id"] == "q5_audio_host_monotonic"
    assert event["source_timestamp_ns"] == 123
    assert event["local_received_monotonic_ns"] == 6000
    assert "mapped_timestamp_ns" not in event


def test_backend_generation_reset_fails_active_task_and_ignores_stale_progress():
    tracker = PlaybackTracker(clock_ns=lambda: 8000)
    tracker.receive_chunk(tracked(eof=True), input_samples=100)
    tracker.record_submitted("speech_A", 275, 275, 4, 100)
    tracker.backend_reset(new_generation=5, reason="device reopened")
    assert tracker.get_status("speech_A")["state"] == "FAILED"
    assert tracker.get_status("speech_A")["error_code"] == "BACKEND_RESET"

    tracker.record_device_progress("speech_A", 275, 4, 120)
    assert tracker.get_status("speech_A")["state"] == "FAILED"


def test_untrusted_progress_cannot_reuse_earlier_trusted_boundary_to_complete():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", eof=True), input_samples=100)
    tracker.record_submitted("A", 275, 275, 1, 10)
    tracker.record_device_progress("A", 100, 1, 20, evidence="snd_pcm_delay")
    tracker.record_device_progress("A", 275, 1, 30, evidence=None)

    status = tracker.get_status("A")
    assert status["state"] != "COMPLETED"
    assert status["trusted_consumed_frames"] == 100


def test_later_lower_trusted_progress_does_not_adopt_untrusted_high_watermark():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked("A", eof=True), input_samples=100)
    tracker.record_submitted("A", 275, 275, 1, 10)
    tracker.record_device_progress("A", 275, 1, 20, evidence=None)
    tracker.record_device_progress("A", 100, 1, 30, evidence="snd_pcm_delay")

    status = tracker.get_status("A")
    assert status["state"] != "COMPLETED"
    assert status["trusted_consumed_frames"] == 100


def test_wait_timeout_does_not_turn_task_into_completed_or_timeout_state():
    tracker = PlaybackTracker(clock_ns=lambda: 9000)
    tracker.receive_chunk(tracked(), input_samples=100)
    result = tracker.wait_for_completion("speech_A", timeout_seconds=0.001)
    assert result["result"] == "timeout"
    assert tracker.get_status("speech_A")["state"] == "SPEAKER_RECEIVED"


def test_waiter_observes_completion_without_deadlock():
    tracker = PlaybackTracker()
    tracker.receive_chunk(tracked(eof=True), input_samples=100)
    result = {}

    def wait():
        result.update(tracker.wait_for_completion("speech_A", timeout_seconds=1))

    thread = threading.Thread(target=wait)
    thread.start()
    tracker.record_submitted("speech_A", 275, 275, 1, 10)
    tracker.record_device_progress("speech_A", 275, 1, 20)
    thread.join(timeout=1)

    assert not thread.is_alive()
    assert result["result"] == "completed"
    assert result["completion_level"] == "alsa_device_consumption"


def test_terminal_history_is_bounded_without_evicting_active_tasks():
    tracker = PlaybackTracker(max_speeches=2)
    tracker.receive_chunk(tracked("active", 0), input_samples=100)
    for speech_id in ("done_1", "done_2"):
        tracker.receive_chunk(tracked(speech_id, 0, True), input_samples=100)
        tracker.record_submitted(speech_id, 275, 275, 1, 10, 44100)
        tracker.record_device_progress(speech_id, 275, 1, 20)

    assert tracker.get_status("active")["state"] == "SPEAKER_RECEIVED"
    assert tracker.get_status("done_1")["state"] == "UNKNOWN"
    assert tracker.get_status("done_2")["state"] == "COMPLETED"


def test_events_include_local_wall_clock_without_mapping_remote_monotonic():
    tracker = PlaybackTracker(clock_ns=lambda: 10, wall_clock_ns=lambda: 20)
    tracker.receive_chunk(tracked(), input_samples=100)
    event = tracker.get_recent_events("speech_A")[-1]
    assert event["local_received_monotonic_ns"] == 10
    assert event["local_received_wall_ns"] == 20
    assert "mapped_timestamp_ns" not in event
