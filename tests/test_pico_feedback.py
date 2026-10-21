"""Robot feedback is a display input, never a control grant."""
import copy

import pytest

from test_pico_device import feedback, frame
from ext_vr.feedback import RobotFeedback
from ext_vr.runtime import DeviceRuntime


def display_feedback(runtime, sequence=1):
    value = feedback(runtime, sequence=sequence)
    chain = [[0, .1, .3], [.2, .1, .3], [.4, .1, .3]]
    value["visualization"] = {"available": True, "feedback_fresh": True, "intent_age_ms": 0,
                              "measured": [chain, chain], "ik": [chain, chain]}
    arm = {"position_error_m": .01, "orientation_error_rad": .02,
           "position_tolerance_m": .03, "orientation_tolerance_rad": .1}
    value["alignment"] = {"state": "ready", "ready": True, "left": arm, "right": arm}
    return copy.deepcopy(value)


@pytest.fixture
def preview():
    now = [10_000_000_000]
    runtime = DeviceRuntime("vr-1", clock_ns=lambda: now[0], clock_id="clock")
    runtime.start()
    binding, epoch = runtime.bind_capture("device")
    runtime.submit_rtc_frame(frame(), authority=binding, rtc_generation=epoch)
    return runtime, RobotFeedback(runtime), now


def test_feedback_is_latest_read_only_and_ages_without_refresh(preview):
    runtime, cache, now = preview
    original = runtime.take_latest()
    cache.accept(display_feedback(runtime))
    assert cache.snapshot()["alignment"]["ready"]
    now[0] += 100_000_000
    assert cache.snapshot()["visualization"]["intent_age_ms"] == 100
    assert runtime.take_latest() is None
    assert not runtime.actuation_enabled
    assert runtime.command_identity()["connection_epoch"] == original["connection_epoch"]
    now[0] += 250_000_000
    assert not cache.snapshot()["alignment"]["ready"]


@pytest.mark.parametrize("change", [
    lambda v: v.update(instance_id="other"),
    lambda v: v.update(clock_id="other"),
    lambda v: v.update(connection_epoch=0),
    lambda v: v.update(space_epoch=0),
    lambda v: v.update(source_sequence=999),
    lambda v: v.update(source_sequence=0),
    lambda v: v.update(emitted_monotonic_ns=0),
    lambda v: v.update(sequence=0),
    lambda v: v.update(visualization=None),
    lambda v: v["visualization"].update(feedback_fresh=False),
    lambda v: v["visualization"].update(ik=[]),
    lambda v: v["visualization"].update(stale=True),
    lambda v: v["visualization"].update(stale="true"),
    lambda v: v["visualization"]["measured"][0][0].__setitem__(0, float("nan")),
    lambda v: v["alignment"].update(state="unaligned"),
    lambda v: v["alignment"]["left"].update(position_error_m=-1),
    lambda v: v.update(extra="x" * 65536),
])
def test_invalid_feedback_cannot_leave_ready_visible(preview, change):
    runtime, cache, _ = preview
    cache.accept(display_feedback(runtime))
    candidate = display_feedback(runtime, 2)
    change(candidate)
    cache.accept(candidate)
    assert not cache.snapshot()["alignment"]["ready"]


def test_conflicting_publishers_stay_unavailable_until_new_capture(preview):
    runtime, cache, _ = preview
    cache.accept(display_feedback(runtime))
    candidate = display_feedback(runtime, 2)
    candidate["server_epoch"] = "other-server"
    cache.accept(candidate)
    cache.accept(display_feedback(runtime, 3))
    assert cache.snapshot()["reason"] == "feedback_source_conflict"
    runtime.stop()
    runtime.start()
    binding, epoch = runtime.bind_capture("device")
    runtime.submit_rtc_frame(frame(), authority=binding, rtc_generation=epoch)
    cache.accept(display_feedback(runtime))
    assert cache.snapshot()["alignment"]["ready"]


def test_stop_reconnect_and_tracking_loss_clear_preview(preview):
    runtime, cache, _ = preview
    stale = display_feedback(runtime)
    cache.accept(stale)
    runtime.mark_capture_disconnected("device", runtime.generation)
    assert not cache.snapshot()["alignment"]["ready"]
    binding, epoch = runtime.bind_capture("device")
    runtime.submit_rtc_frame(frame(), authority=binding, rtc_generation=epoch)
    cache.accept(stale)
    assert not cache.snapshot()["alignment"]["ready"]
    cache.accept(display_feedback(runtime))
    runtime.last_tracking["left"] = False
    assert not cache.snapshot()["alignment"]["ready"]
    runtime.stop()
    assert not cache.snapshot()["alignment"]["ready"]


def test_missing_extension_and_unreachable_target_never_reuse_old_ik(preview):
    runtime, cache, _ = preview
    cache.accept(display_feedback(runtime))
    cache.accept(feedback(runtime, sequence=2))
    assert not cache.snapshot()["alignment"]["ready"]
    candidate = display_feedback(runtime, 3)
    candidate["alignment"].update(state="unreachable", ready=False)
    candidate["visualization"]["ik"] = []
    cache.accept(candidate)
    assert cache.snapshot()["visualization"]["ik"] == []
    assert cache.snapshot()["alignment"]["state"] == "unreachable"


def test_control_status_keeps_age_without_an_ik_preview(preview):
    runtime, cache, now = preview
    candidate = feedback(runtime)
    candidate.update(state="hold", reason="motion_feedback_stale")
    cache.accept(candidate)
    now[0] += 100_000_000
    result = cache.snapshot()
    assert result["state"] == "hold"
    assert result["reason"] == "motion_feedback_stale"
    assert result["feedback_age_ms"] == 100
    assert not result["alignment"]["ready"]
    assert "feedback_age_ms" not in candidate


def test_expired_feedback_keeps_geometry_without_refreshing_identity(preview):
    runtime, cache, now = preview
    original = display_feedback(runtime)
    cache.accept(original)
    for sequence, age in ((2, 301), (3, 800), (4, 1200)):
        now[0] = original["emitted_monotonic_ns"] + age * 1_000_000
        runtime.submit_rtc_frame(frame(sequence), authority=runtime._binding,
                                 rtc_generation=runtime.generation)
        result = cache.snapshot()
        assert result["visualization"]["ik"] == original["visualization"]["ik"]
        assert result["visualization"]["stale"]
        assert not result["visualization"]["feedback_fresh"]
        assert result["alignment"] == {"state": "unavailable", "ready": False}
        assert result["feedback_age_ms"] == age
        for key in ("sequence", "source_sequence", "emitted_monotonic_ns", "mapping_epoch"):
            assert result[key] == original[key]
    assert "stale" not in cache.value["visualization"]
    cache.accept(display_feedback(runtime, 2))
    assert cache.snapshot()["alignment"]["ready"]
