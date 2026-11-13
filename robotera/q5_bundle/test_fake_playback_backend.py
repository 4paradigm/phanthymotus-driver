from fake_playback_backend import FakePlaybackBackend
from playback_telemetry import PlaybackTracker, parse_audio_format
from playback_telemetry_soak import run_soak


def metadata(speech_id, sequence, eof=False):
    return parse_audio_format(
        f"audio/pcm-16k;speech_id={speech_id};sequence={sequence};eof={int(eof)}"
    )


def test_successful_write_precedes_delayed_device_completion():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=32000)

    status = tracker.get_status("A")
    assert status["submitted_frames"] == 88200
    assert status["consumed_frames"] == 0
    assert status["state"] == "PCM_SUBMITTED"

    backend.consume(44100)
    assert tracker.get_status("A")["progress"] == 0.5
    backend.consume(44100)
    assert tracker.get_status("A")["state"] == "COMPLETED"


def test_resampling_counts_output_frames_not_stereo_samples():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker, input_rate=16000, output_rate=44100,
                                  output_channels=2)
    backend.push(metadata("A", 0), input_frames=16000)
    backend.push(metadata("A", 1, True), input_frames=16000)

    status = tracker.get_status("A")
    assert status["total_input_samples"] == 32000
    assert status["total_output_frames"] == 88200
    assert status["duration_audio_seconds"] == 2.0


def test_partial_writes_and_eagain_count_only_accepted_frames():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=1600,
                 write_results=[1000, "EAGAIN", 2000, 1410])

    status = tracker.get_status("A")
    assert status["submitted_frames"] == 4410
    assert backend.write_attempts == 4
    assert backend.eagain_count == 1


def test_underrun_fails_speech_and_preserves_last_trusted_submission():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    accepted = backend.push(metadata("A", 0, True), input_frames=1600,
                            write_results=[1000, "EPIPE", 3410])

    status = tracker.get_status("A")
    assert status["submitted_frames"] == 1000
    assert status["underrun_count"] == 1
    assert status["state"] == "FAILED"
    assert status["error_code"] == "ALSA_UNDERRUN"
    assert accepted == 1000
    assert backend.submitted_global == 1000
    assert any(event["event"] == "underrun" for event in tracker.get_recent_events("A"))

    assert backend.push(metadata("A", 1, True), input_frames=1600) == 0
    backend.push(metadata("B", 0, True), input_frames=1600)
    backend.consume(4410)
    assert tracker.get_status("A")["state"] == "FAILED"
    assert tracker.get_status("B")["state"] == "COMPLETED"


def test_underrun_fails_every_active_speech_with_frames_still_buffered():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=1600)
    backend.push(metadata("B", 0, True), input_frames=1600,
                 write_results=["EPIPE"])

    assert tracker.get_status("A")["error_code"] == "ALSA_UNDERRUN"
    assert tracker.get_status("B")["error_code"] == "ALSA_UNDERRUN"


def test_underrun_does_not_fail_speech_already_confirmed_consumed():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("done", 0, True), input_frames=1600)
    backend.consume(4410)
    backend.push(metadata("affected", 0, True), input_frames=1600,
                 write_results=["EPIPE"])

    assert tracker.get_status("done")["state"] == "COMPLETED"
    assert tracker.get_status("affected")["state"] == "FAILED"


def test_drain_failure_fails_active_speech_without_false_completion():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=1600)

    backend.drain(success=False)

    status = tracker.get_status("A")
    assert status["state"] == "FAILED"
    assert status["error_code"] == "ALSA_DRAIN_FAILED"


def test_successful_drain_is_trusted_completion_evidence():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=1600)

    backend.drain(success=True)

    assert tracker.get_status("A")["state"] == "COMPLETED"


def test_record_capacity_rejects_new_speech_without_evicting_active_records():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker, max_records=2)
    backend.push(metadata("A", 0), input_frames=1600)
    backend.push(metadata("B", 0), input_frames=1600)

    assert backend.push(metadata("C", 0, True), input_frames=1600) == 0
    assert backend.record_count == 2
    assert backend.has_record("A") and backend.has_record("B")
    assert tracker.get_status("A")["state"] == "PCM_SUBMITTED"
    assert tracker.get_status("B")["state"] == "PCM_SUBMITTED"
    assert tracker.get_status("C")["error_code"] == "PLAYBACK_CAPACITY"


def test_terminal_record_is_cleaned_before_accepting_new_speech():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker, max_records=1)
    backend.push(metadata("A", 0, True), input_frames=1600)
    backend.consume(4410)

    backend.push(metadata("B", 0, True), input_frames=1600)

    assert backend.record_count == 1
    assert not backend.has_record("A")
    assert backend.has_record("B")


def test_repeated_delay_failures_end_active_tasks_instead_of_hanging():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker, max_progress_failures=3)
    backend.push(metadata("A", 0, True), input_frames=1600)

    for _ in range(3):
        backend.progress_query_failed()

    assert tracker.get_status("A")["error_code"] == "ALSA_DELAY_FAILED"


def test_repeated_no_progress_writes_fail_boundedly():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker, max_write_stalls=3)

    accepted = backend.push(
        metadata("A", 0, True), input_frames=1600,
        write_results=["EAGAIN", 0, "EAGAIN", 4410])

    assert accepted == 0
    assert backend.write_attempts == 3
    assert tracker.get_status("A")["error_code"] == "ALSA_WRITE_STALLED"


def test_restart_fails_old_speech_and_new_generation_can_complete():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    backend.push(metadata("A", 0, True), input_frames=1600)
    backend.restart()
    assert tracker.get_status("A")["state"] == "FAILED"

    backend.push(metadata("B", 0, True), input_frames=1600)
    backend.consume(4410)
    assert tracker.get_status("B")["state"] == "COMPLETED"


def test_consecutive_speech_frame_intervals_do_not_cross_complete():
    tracker = PlaybackTracker()
    backend = FakePlaybackBackend(tracker)
    for speech_id in ("A", "B", "C"):
        backend.push(metadata(speech_id, 0, True), input_frames=1600)

    backend.consume(4410)
    assert [tracker.get_status(item)["state"] for item in ("A", "B", "C")] == [
        "COMPLETED", "PCM_SUBMITTED", "PCM_SUBMITTED"
    ]


def test_bounded_soak_reports_requested_work_and_no_failures():
    result = run_soak(speeches=100, chunks_per_speech=10, seed=7)
    assert result["speeches"] == 100
    assert result["chunks"] == 1000
    assert result["queries"] == 100
    assert result["waits"] == 10
    assert result["interruptions"] == 10
    assert result["restarts"] == 1
    assert result["failures"] == 0
