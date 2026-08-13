from __future__ import annotations

import copy
import sys
import threading
import time
import unittest
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dispatch import (
    AdapterAck,
    FinalDispatchArbiter,
    MotionIntent,
    RecordingAdapter,
    StopRequest,
)
from protocol import MAX_SEQUENCE


def binding(*, epoch: int = 1, fence: str = "f" * 32) -> dict:
    return {
        "boot_id": str(uuid.uuid4()),
        "session_id": str(uuid.uuid4()),
        "epoch": epoch,
        "fence": fence,
    }


def frame(authority: dict, sequence: int, *, deadman: bool = True) -> dict:
    pose = {"position": [0.0, 1.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    return {
        **authority,
        "sequence": sequence,
        "clutch_sequence": 1,
        "deadman": deadman,
        "tracking": {"head": True, "left_controller": True, "right_controller": True},
        "head": dict(pose),
        "left_controller": dict(pose),
        "right_controller": dict(pose),
        "controllers": {
            "left": {"axes": [0.0, 0.0], "buttons": [1.0]},
            "right": {"axes": [0.0, 0.0], "buttons": [1.0]},
        },
        "base_twist": {"linear": [0.1, 0.0, 0.0], "angular": [0.0, 0.0, 0.1]},
    }


class ControllableAdapter:
    def __init__(self, *, startup_ack: AdapterAck | None = None, fail_apply: bool = False):
        self.startup_ack = startup_ack or AdapterAck(True)
        self.fail_apply = fail_apply
        self.first_apply_started = threading.Event()
        self.allow_first_apply = threading.Event()
        self.block_first_apply = False
        self.events: list[tuple] = []
        self.snapshot_threads: list[str] = []
        self.closed = False
        self._lock = threading.Lock()

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck:
        with self._lock:
            self.events.append(("stop", "startup_safe", 0))
        return self.startup_ack

    def apply(self, intent: MotionIntent) -> AdapterAck:
        if self.block_first_apply and not self.first_apply_started.is_set():
            self.first_apply_started.set()
            self.allow_first_apply.wait(1.0)
        with self._lock:
            self.events.append(("apply", intent.sequence, intent.dispatch_generation))
        if self.fail_apply:
            return AdapterAck(False, "injected_apply_failure")
        return AdapterAck(True)

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        with self._lock:
            self.events.append(("stop", request.reason, request.dispatch_generation))
        return AdapterAck(True)

    def snapshot(self) -> dict:
        with self._lock:
            self.snapshot_threads.append(threading.current_thread().name)
            return {"kind": "test", "events": copy.deepcopy(self.events), "closed": self.closed}

    def close(self) -> None:
        with self._lock:
            self.events.append(("close",))
            self.closed = True


class HangingStartupAdapter(ControllableAdapter):
    def __init__(self):
        super().__init__()
        self.startup_entered = threading.Event()
        self.release_startup = threading.Event()
        self.concurrent_calls: list[str] = []

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck:
        self.startup_entered.set()
        self.release_startup.wait(1.0)
        return AdapterAck(True)

    def snapshot(self) -> dict:
        if not self.release_startup.is_set():
            self.concurrent_calls.append("snapshot")
        return super().snapshot()

    def close(self) -> None:
        if not self.release_startup.is_set():
            self.concurrent_calls.append("close")
        super().close()


class DelayedStopAdapter(ControllableAdapter):
    def __init__(self):
        super().__init__()
        self.stop_delay = 0.0

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        if self.stop_delay:
            time.sleep(self.stop_delay)
        return super().safe_stop(request)


class AdvancingStopAdapter(ControllableAdapter):
    def __init__(self, safety_time: list[float], *, advance: float):
        super().__init__()
        self.safety_time = safety_time
        self.advance = advance

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        self.safety_time[0] += self.advance
        return super().safe_stop(request)


class GatedRecordingAdapter(RecordingAdapter):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.apply_entered = threading.Event()
        self.release_apply = threading.Event()

    def apply(self, intent: MotionIntent) -> AdapterAck:
        self.apply_entered.set()
        self.release_apply.wait(1.0)
        return super().apply(intent)


class FalseCloseAdapter(ControllableAdapter):
    def close(self):
        super().close()
        return False


class ReentrantCloseAdapter(ControllableAdapter):
    def __init__(self):
        super().__init__()
        self.arbiter: FinalDispatchArbiter | None = None
        self.reentrant_ack: AdapterAck | None = None

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        if request.reason == "service_close":
            assert self.arbiter is not None
            self.reentrant_ack = self.arbiter.close()
        return super().safe_stop(request)


class FinalDispatchArbiterTests(unittest.TestCase):
    def setUp(self):
        self.adapter = ControllableAdapter()
        self.arbiter = FinalDispatchArbiter(self.adapter, io_timeout_ms=250)
        self.authority = binding()
        handle = self.arbiter.begin_prepare()
        self.assertTrue(self.arbiter.wait_safe(handle).ok)
        self.assertTrue(self.arbiter.complete_prepare(handle, self.authority, 7).ok)

    def tearDown(self):
        self.adapter.allow_first_apply.set()
        self.arbiter.close()

    def publish(self, sequence: int, *, expiry: float = 1.0, deadman: bool = True) -> AdapterAck:
        return self.arbiter.publish_latest(
            frame(self.authority, sequence, deadman=deadman),
            session_generation=7,
            expires_monotonic=time.monotonic() + expiry,
        )

    def wait_until(self, predicate, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.005)
        self.fail("condition not reached")

    def test_latest_only_mailbox_keeps_one_inflight_and_newest_pending(self):
        self.adapter.block_first_apply = True
        self.assertTrue(self.publish(1).ok)
        self.assertTrue(self.adapter.first_apply_started.wait(1.0))
        for sequence in range(2, 101):
            self.assertTrue(self.publish(sequence).ok)
            self.assertLessEqual(self.arbiter.snapshot()["mailbox_depth"], 1)
        self.adapter.allow_first_apply.set()
        self.assertTrue(self.arbiter.wait_applied(100))
        applies = [event[1] for event in self.adapter.events if event[0] == "apply"]
        self.assertEqual([1, 100], applies)
        self.assertEqual(98, self.arbiter.snapshot()["counters"]["mailbox_replacements"])

    def test_stop_clears_admitted_motion_and_ack_fences_old_generation(self):
        self.adapter.block_first_apply = True
        self.assertTrue(self.publish(1).ok)
        self.assertTrue(self.adapter.first_apply_started.wait(1.0))
        self.assertTrue(self.publish(2).ok)
        handle = self.arbiter.trip(
            "operator_release",
            target_state="safe_revoked",
            retain_authority=False,
        )
        self.adapter.allow_first_apply.set()
        self.assertTrue(self.arbiter.wait_safe(handle).ok)
        time.sleep(0.02)
        applies = [event[1] for event in self.adapter.events if event[0] == "apply"]
        self.assertEqual([1], applies)
        self.assertEqual(("stop", "operator_release"), self.adapter.events[-1][:2])
        self.assertFalse(self.publish(3).ok)
        self.assertTrue(self.arbiter.snapshot()["stop_acknowledged"])

    def test_final_deadline_check_drops_queued_motion_and_stops(self):
        self.adapter.block_first_apply = True
        self.assertTrue(self.publish(1).ok)
        self.assertTrue(self.adapter.first_apply_started.wait(1.0))
        self.assertTrue(self.publish(2, expiry=0.02).ok)
        time.sleep(0.03)
        self.adapter.allow_first_apply.set()
        self.wait_until(
            lambda: any(
                event[:2] == ("stop", "dispatch_deadline") for event in self.adapter.events
            )
        )
        applies = [event[1] for event in self.adapter.events if event[0] == "apply"]
        self.assertEqual([1], applies)
        self.assertEqual("safe_reclutch_required", self.arbiter.snapshot()["state"])

    def test_adapter_never_receives_unsafe_or_wrong_authority(self):
        self.assertEqual("unsafe_deadman", self.publish(1, deadman=False).code)
        missing_tracking = frame(self.authority, 1)
        missing_tracking["tracking"].pop("head")
        self.assertEqual(
            "unsafe_tracking",
            self.arbiter.publish_latest(
                missing_tracking,
                session_generation=7,
                expires_monotonic=time.monotonic() + 1,
            ).code,
        )
        wrong = binding(epoch=2, fence="x" * 32)
        ack = self.arbiter.publish_latest(
            frame(wrong, 2),
            session_generation=7,
            expires_monotonic=time.monotonic() + 1,
        )
        self.assertEqual("authority_mismatch", ack.code)
        self.assertFalse(any(event[0] == "apply" for event in self.adapter.events))

    def test_final_boundary_rejects_non_increasing_sequence(self):
        self.adapter.block_first_apply = True
        self.assertTrue(self.publish(5).ok)
        self.assertTrue(self.adapter.first_apply_started.wait(1.0))
        self.assertEqual("sequence_not_increasing", self.publish(5).code)
        self.assertEqual("sequence_not_increasing", self.publish(4).code)
        self.adapter.allow_first_apply.set()
        self.assertTrue(self.arbiter.wait_applied(5))

    def test_final_boundary_enforces_sequence_wire_limits(self):
        at_limit = frame(self.authority, MAX_SEQUENCE)
        at_limit["clutch_sequence"] = MAX_SEQUENCE
        self.assertTrue(
            self.arbiter.publish_latest(
                at_limit,
                session_generation=7,
                expires_monotonic=time.monotonic() + 1,
            ).ok
        )

        above_sequence = frame(self.authority, MAX_SEQUENCE + 1)
        self.assertEqual(
            "invalid_intent",
            self.arbiter.publish_latest(
                above_sequence,
                session_generation=7,
                expires_monotonic=time.monotonic() + 1,
            ).code,
        )

        above_clutch = frame(self.authority, MAX_SEQUENCE)
        above_clutch["clutch_sequence"] = MAX_SEQUENCE + 1
        self.assertEqual(
            "invalid_intent",
            self.arbiter.publish_latest(
                above_clutch,
                session_generation=7,
                expires_monotonic=time.monotonic() + 1,
            ).code,
        )

    def test_adapter_apply_failure_latches_fault_and_runs_safety_path(self):
        self.adapter.fail_apply = True
        self.assertTrue(self.publish(1).ok)
        self.wait_until(lambda: self.arbiter.snapshot()["fault_code"] is not None)
        self.wait_until(
            lambda: any(event[:2] == ("stop", "adapter_fault") for event in self.adapter.events)
        )
        state = self.arbiter.snapshot()
        self.assertEqual("fault_latched", state["state"])
        self.assertEqual("injected_apply_failure", state["fault_code"])
        self.assertFalse(self.publish(2).ok)

    def test_public_snapshot_contains_no_fence(self):
        self.assertNotIn(self.authority["fence"], repr(self.arbiter.snapshot()))

    def test_public_snapshot_uses_owner_cache_only(self):
        before = len(self.adapter.snapshot_threads)
        for _ in range(100):
            self.arbiter.snapshot()
        self.assertEqual(before, len(self.adapter.snapshot_threads))
        self.assertNotIn("MainThread", self.adapter.snapshot_threads)
        self.assertTrue(
            set(self.adapter.snapshot_threads)
            <= {"teleop-shadow-startup-safe", "teleop-shadow-final-dispatch"}
        )


class StartupAndRestartTests(unittest.TestCase):
    def test_failed_startup_safe_is_not_ready_or_armable(self):
        adapter = ControllableAdapter(startup_ack=AdapterAck(False, "no_stop_ack"))
        arbiter = FinalDispatchArbiter(adapter)
        try:
            self.assertFalse(arbiter.ready)
            handle = arbiter.begin_prepare()
            self.assertEqual("dispatch_unavailable", arbiter.wait_safe(handle).code)
            self.assertFalse(arbiter.complete_prepare(handle, binding(), 1).ok)
            self.assertFalse(arbiter.snapshot()["ready"])
        finally:
            first_close = arbiter.close()
            self.assertFalse(first_close.ok)
            self.assertEqual(first_close, arbiter.close())
            self.assertTrue(adapter.closed)

    def test_new_arbiter_starts_with_safe_stop_before_any_motion(self):
        adapter = ControllableAdapter()
        arbiter = FinalDispatchArbiter(adapter)
        try:
            self.assertEqual(("stop", "startup_safe", 0), adapter.events[0])
            self.assertIsNone(arbiter.snapshot()["last_would_apply_sequence"])
        finally:
            arbiter.close()

    def test_hung_startup_fails_closed_without_concurrent_adapter_calls(self):
        adapter = HangingStartupAdapter()
        started = time.monotonic()
        arbiter = FinalDispatchArbiter(adapter, io_timeout_ms=20)
        self.assertLess(time.monotonic() - started, 0.2)
        try:
            self.assertTrue(adapter.startup_entered.is_set())
            state = arbiter.snapshot()
            self.assertEqual("startup_safe_timeout", state["fault_code"])
            self.assertEqual("unavailable", state["adapter"]["kind"])
            close_ack = arbiter.close()
            self.assertFalse(close_ack.ok)
            self.assertEqual([], adapter.concurrent_calls)
        finally:
            adapter.release_startup.set()
            deadline = time.monotonic() + 1.0
            while not adapter.closed and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(adapter.closed)
            self.assertEqual([], adapter.concurrent_calls)


class DeadlineAndShutdownTests(unittest.TestCase):
    def test_frozen_protocol_clock_does_not_hide_real_io_stall(self):
        adapter = ControllableAdapter()
        adapter.block_first_apply = True
        arbiter = FinalDispatchArbiter(adapter, clock=lambda: 100.0, io_timeout_ms=20)
        authority = binding()
        prepare = arbiter.begin_prepare()
        self.assertTrue(arbiter.wait_safe(prepare).ok)
        self.assertTrue(arbiter.complete_prepare(prepare, authority, 1).ok)
        try:
            self.assertTrue(arbiter.publish_latest(
                frame(authority, 1),
                session_generation=1,
                expires_monotonic=101.0,
            ).ok)
            self.assertTrue(adapter.first_apply_started.wait(1.0))
            time.sleep(0.03)
            state = arbiter.snapshot()
            self.assertEqual("adapter_io_stalled", state["fault_code"])
            self.assertFalse(state["ready"])
            self.assertEqual("apply", state["io_inflight"])
        finally:
            adapter.allow_first_apply.set()
            arbiter.close()

    def test_stop_completion_after_total_deadline_is_not_acknowledged(self):
        adapter = DelayedStopAdapter()
        arbiter = FinalDispatchArbiter(adapter, clock=lambda: 100.0, io_timeout_ms=20)
        authority = binding()
        prepare = arbiter.begin_prepare()
        self.assertTrue(arbiter.wait_safe(prepare).ok)
        self.assertTrue(arbiter.complete_prepare(prepare, authority, 1).ok)
        adapter.stop_delay = 0.03
        try:
            handle = arbiter.trip(
                "deadline_test",
                target_state="safe_revoked",
                retain_authority=False,
            )
            ack = arbiter.wait_safe(handle, 0.2)
            self.assertFalse(ack.ok)
            self.assertEqual("adapter_stop_deadline_missed", ack.code)
            self.assertEqual("fault_latched", arbiter.snapshot()["state"])
        finally:
            arbiter.close()

    def test_queued_stops_each_keep_their_original_total_deadline(self):
        safety_time = [10.0]
        adapter = AdvancingStopAdapter(safety_time, advance=0.015)
        arbiter = FinalDispatchArbiter(
            adapter,
            safety_clock=lambda: safety_time[0],
            io_timeout_ms=20,
        )
        try:
            first = arbiter.trip(
                "first_stop",
                target_state="safe_revoked",
                retain_authority=False,
            )
            second = arbiter.trip(
                "second_stop",
                target_state="safe_revoked",
                retain_authority=False,
            )
            self.assertTrue(arbiter.wait_safe(first, 0.2).ok)
            second_ack = arbiter.wait_safe(second, 0.2)
            self.assertFalse(second_ack.ok)
            self.assertEqual("adapter_stop_deadline_missed", second_ack.code)
        finally:
            arbiter.close()

    def test_close_never_overtakes_blocked_apply_and_caches_failure(self):
        adapter = ControllableAdapter()
        adapter.block_first_apply = True
        arbiter = FinalDispatchArbiter(adapter, io_timeout_ms=200)
        authority = binding()
        prepare = arbiter.begin_prepare()
        self.assertTrue(arbiter.wait_safe(prepare).ok)
        self.assertTrue(arbiter.complete_prepare(prepare, authority, 1).ok)
        self.assertTrue(arbiter.publish_latest(
            frame(authority, 1),
            session_generation=1,
            expires_monotonic=time.monotonic() + 1,
        ).ok)
        self.assertTrue(adapter.first_apply_started.wait(1.0))

        results: list[AdapterAck] = []
        close_thread = threading.Thread(target=lambda: results.append(arbiter.close(0.03)))
        close_thread.start()
        time.sleep(0.05)
        self.assertFalse(adapter.closed)
        self.assertTrue(close_thread.is_alive())
        adapter.allow_first_apply.set()
        close_thread.join(1.0)
        self.assertFalse(close_thread.is_alive())
        self.assertFalse(results[0].ok)
        self.assertEqual(results[0], arbiter.close())
        kinds = [event[0] for event in adapter.events]
        self.assertLess(kinds.index("apply"), kinds.index("close"))
        self.assertLess(
            max(index for index, kind in enumerate(kinds) if kind == "stop"),
            kinds.index("close"),
        )

    def test_false_close_result_is_not_treated_as_success(self):
        adapter = FalseCloseAdapter()
        arbiter = FinalDispatchArbiter(adapter)
        ack = arbiter.close()
        self.assertFalse(ack.ok)
        self.assertEqual("invalid_close_ack", ack.code)
        self.assertEqual("fault_latched", arbiter.snapshot()["state"])

    def test_reentrant_owner_close_does_not_poison_external_close(self):
        adapter = ReentrantCloseAdapter()
        arbiter = FinalDispatchArbiter(adapter)
        adapter.arbiter = arbiter
        ack = arbiter.close()
        self.assertTrue(ack.ok)
        self.assertEqual(AdapterAck(False, "owner_cannot_self_close"), adapter.reentrant_ack)
        self.assertTrue(adapter.closed)
        self.assertEqual(ack, arbiter.close())

    def test_recording_adapter_rechecks_expiry_inside_final_output_lock(self):
        now = [100.0]
        adapter = GatedRecordingAdapter(clock=lambda: now[0])
        arbiter = FinalDispatchArbiter(adapter, clock=lambda: now[0], io_timeout_ms=100)
        authority = binding()
        prepare = arbiter.begin_prepare()
        self.assertTrue(arbiter.wait_safe(prepare).ok)
        self.assertTrue(arbiter.complete_prepare(prepare, authority, 1).ok)
        try:
            self.assertTrue(arbiter.publish_latest(
                frame(authority, 1),
                session_generation=1,
                expires_monotonic=100.05,
            ).ok)
            self.assertTrue(adapter.apply_entered.wait(1.0))
            now[0] = 100.06
            adapter.release_apply.set()
            deadline = time.monotonic() + 1
            while arbiter.snapshot()["fault_code"] is None and time.monotonic() < deadline:
                time.sleep(0.005)
            records = adapter.snapshot()["records"]
            self.assertFalse(any(record["kind"] == "would_apply" for record in records))
            self.assertEqual(
                "intent_expired_at_adapter",
                arbiter.snapshot()["fault_code"],
            )
        finally:
            adapter.release_apply.set()
            arbiter.close()


if __name__ == "__main__":
    unittest.main()
