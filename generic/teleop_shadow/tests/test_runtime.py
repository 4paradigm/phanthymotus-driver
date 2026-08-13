from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dispatch import FinalDispatchArbiter, RecordingAdapter, StopRequest
from protocol import ProtocolError
from runtime import ShadowRuntime

from tests.helpers import (
    FakeClock,
    contains_value,
    identity,
    new_session,
    rtc_wire_frame,
    valid_frame,
)


class GatedStopRecordingAdapter(RecordingAdapter):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.block_reason: str | None = None
        self.stop_entered = threading.Event()
        self.release_stop = threading.Event()

    def safe_stop(self, request: StopRequest):
        if request.reason == self.block_reason:
            self.stop_entered.set()
            self.release_stop.wait(1.0)
        return super().safe_stop(request)


class ShadowRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        # Keep synthetic mailbox tests on one deterministic clock. Real
        # monotonic adapter-stall behavior is covered by DeadlineAndShutdownTests.
        dispatcher = FinalDispatchArbiter(
            RecordingAdapter(clock=self.clock),
            clock=self.clock,
            safety_clock=self.clock,
            io_timeout_ms=100,
        )
        self.runtime = ShadowRuntime(
            lease_timeout_ms=1000,
            pose_timeout_ms=200,
            clock=self.clock,
            auto_watchdog=False,
            dispatch_io_timeout_ms=100,
            dispatcher=dispatcher,
        )
        self.addCleanup(self.runtime.close)
        self.session = new_session()
        self.prepared = self.runtime.prepare_shadow(self.session)

    def wait_until(self, predicate, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.005)
        self.fail("condition not reached")

    def gated_runtime(self):
        clock = FakeClock()
        adapter = GatedStopRecordingAdapter(clock=clock)
        dispatcher = FinalDispatchArbiter(
            adapter,
            clock=clock,
            io_timeout_ms=150,
        )
        runtime = ShadowRuntime(
            clock=clock,
            auto_watchdog=False,
            dispatch_io_timeout_ms=150,
            dispatch_ack_timeout_ms=200,
            dispatcher=dispatcher,
        )
        self.addCleanup(runtime.close)
        self.addCleanup(adapter.release_stop.set)
        return clock, adapter, runtime

    def test_prepare_and_public_status_never_disclose_fence(self):
        self.assertNotIn("fence", self.prepared)
        status = self.runtime.status()
        self.assertNotIn("fence", status)
        self.assertFalse(contains_value(status, self.session["fence"]))
        self.assertFalse(status["actuation_enabled"])
        self.assertEqual("shadow", status["mode"])

    def test_latest_only_and_public_frame_is_sanitized(self):
        for sequence in range(1, 1001):
            self.runtime.submit_shadow_frame(
                valid_frame(self.runtime, self.session, sequence=sequence), source="test"
            )
            # This synthetic producer is much faster than a headset. Yield the
            # GIL periodically so it tests mailbox replacement without
            # manufacturing an adapter-owner scheduling stall (>100 ms).
            if sequence % 32 == 0:
                time.sleep(0.001)
        status = self.runtime.status()
        self.assertEqual(1000, status["pose"]["latest_sequence"])
        self.assertEqual(1000, status["counters"]["frames_accepted"])
        self.assertNotIn("fence", status["pose"]["latest"])
        self.assertFalse(contains_value(status, self.session["fence"]))

    def test_valid_frame_reaches_visible_recording_dispatch(self):
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=7), source="test"
        )
        dispatch = self.wait_until(
            lambda: (
                state
                if (state := self.runtime.status()["dispatch"])["last_would_apply_sequence"] == 7
                else None
            )
        )
        self.assertEqual("would_apply", dispatch["last_decision"])
        self.assertEqual("recording", dispatch["kind"])
        self.assertEqual(0, dispatch["mailbox_depth"])
        self.assertNotIn(self.session["fence"], repr(dispatch))

    def test_unsafe_frame_stops_without_reaching_recording_apply(self):
        held = self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1, deadman=False), source="test"
        )
        self.assertEqual(("hold", "deadman_released"), (held["state"], held["reason"]))
        dispatch = self.wait_until(
            lambda: (
                state
                if (state := self.runtime.status()["dispatch"])["stop_acknowledged"]
                else None
            )
        )
        self.assertIsNone(dispatch["last_would_apply_sequence"])
        records = dispatch["adapter"]["records"]
        self.assertEqual("deadman_released", records[-1]["reason"])
        self.assertFalse(any(record["kind"] == "would_apply" for record in records))

    def test_soft_stop_is_latched_until_new_prepare(self):
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1, clutch_sequence=1), source="test"
        )
        self.wait_until(
            lambda: self.runtime.status()["dispatch"]["last_would_apply_sequence"] == 1
        )
        stopped = self.runtime.soft_stop(identity(self.runtime, self.session))
        self.assertEqual(("hold", "soft_stop"), (stopped["state"], stopped["reason"]))
        held = self.runtime.submit_shadow_frame(
            valid_frame(
                self.runtime,
                self.session,
                sequence=2,
                clutch_sequence=999,
            ),
            source="test",
        )
        self.assertEqual(("hold", "soft_stop"), (held["state"], held["reason"]))
        time.sleep(0.02)
        self.assertEqual(1, self.runtime.status()["dispatch"]["last_would_apply_sequence"])

    def test_prepare_over_active_waits_for_visible_stop_ack(self):
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1), source="test"
        )
        self.wait_until(
            lambda: self.runtime.status()["dispatch"]["last_would_apply_sequence"] == 1
        )
        newer = new_session(epoch=2, fence="n" * 32)
        prepared = self.runtime.prepare_shadow(newer)
        self.assertEqual("prepared_shadow", prepared["state"])
        self.assertTrue(prepared["dispatch"]["stop_acknowledged"])
        self.assertEqual("safe_waiting_frame", prepared["dispatch"]["state"])
        self.assertEqual(
            "prepare_shadow",
            prepared["dispatch"]["adapter"]["records"][-1]["reason"],
        )

    def test_prepare_stop_fences_old_rtc_callbacks_while_ack_is_pending(self):
        _, adapter, runtime = self.gated_runtime()
        first = new_session()
        runtime.prepare_shadow(first)
        old_generation = runtime.session_generation()
        adapter.block_reason = "prepare_shadow"
        adapter.stop_entered.clear()
        adapter.release_stop.clear()
        second = new_session(epoch=2, fence="n" * 32)
        results: list[dict] = []
        errors: list[Exception] = []
        worker = threading.Thread(
            target=lambda: self._capture_call(
                lambda: runtime.prepare_shadow(second), results, errors
            )
        )
        worker.start()
        self.assertTrue(adapter.stop_entered.wait(1.0))
        runtime.mark_channel(old_generation, "teleop-control", True)
        runtime.mark_rtc_disconnected(old_generation, "rtc_closed")
        adapter.release_stop.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual([], errors)
        self.assertEqual("prepared_shadow", results[0]["state"])
        self.assertEqual(second["session_id"], results[0]["session_id"])
        self.assertEqual(2, results[0]["counters"]["stale_rtc_callbacks"])

    def test_release_cannot_be_revived_while_stop_ack_is_pending(self):
        _, adapter, runtime = self.gated_runtime()
        session = new_session()
        runtime.prepare_shadow(session)
        old_generation = runtime.session_generation()
        adapter.block_reason = "operator_release"
        adapter.stop_entered.clear()
        adapter.release_stop.clear()
        results: list[dict] = []
        errors: list[Exception] = []
        worker = threading.Thread(
            target=lambda: self._capture_call(
                lambda: runtime.release(identity(runtime, session)), results, errors
            )
        )
        worker.start()
        self.assertTrue(adapter.stop_entered.wait(1.0))
        runtime.mark_channel(old_generation, "teleop-control", True)
        runtime.mark_rtc_disconnected(old_generation, "rtc_closed")
        pending = runtime.status()
        self.assertEqual("released", pending["state"])
        self.assertFalse(pending["authority_valid"])
        self.assertFalse(pending["dispatch"]["stop_acknowledged"])
        adapter.release_stop.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual([], errors)
        self.assertEqual("released", results[0]["state"])
        self.assertFalse(results[0]["authority_valid"])
        self.assertTrue(results[0]["dispatch"]["stop_acknowledged"])

    def test_watchdog_revocation_supersedes_pause_while_stop_is_pending(self):
        clock = FakeClock()
        adapter = GatedStopRecordingAdapter(clock=clock)
        dispatcher = FinalDispatchArbiter(
            adapter,
            clock=clock,
            safety_clock=clock,
            io_timeout_ms=150,
        )
        runtime = ShadowRuntime(
            lease_timeout_ms=100,
            clock=clock,
            auto_watchdog=False,
            dispatch_io_timeout_ms=150,
            dispatch_ack_timeout_ms=200,
            dispatcher=dispatcher,
        )
        self.addCleanup(runtime.close)
        self.addCleanup(adapter.release_stop.set)
        session = new_session()
        runtime.prepare_shadow(session)
        adapter.block_reason = "operator_pause"
        adapter.stop_entered.clear()
        adapter.release_stop.clear()
        results: list[dict] = []
        errors: list[Exception] = []
        worker = threading.Thread(
            target=lambda: self._capture_call(
                lambda: runtime.pause(identity(runtime, session)), results, errors
            )
        )
        worker.start()
        self.assertTrue(adapter.stop_entered.wait(1.0))
        clock.advance(0.11)
        expired = runtime.watchdog_tick()
        self.assertEqual(("hold", "lease_timeout"), (expired["state"], expired["reason"]))
        self.assertFalse(expired["authority_valid"])
        adapter.release_stop.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual([], errors)
        final = self.wait_until(
            lambda: (
                state
                if (state := runtime.status())["dispatch"]["stop_acknowledged"]
                else None
            )
        )
        self.assertEqual(("hold", "lease_timeout"), (final["state"], final["reason"]))
        reasons = [
            record["reason"]
            for record in final["dispatch"]["adapter"]["records"]
            if record["kind"] == "would_stop"
        ]
        self.assertLess(reasons.index("operator_pause"), reasons.index("lease_timeout"))
        self.assertIsNone(final["dispatch"]["last_would_apply_sequence"])

    @staticmethod
    def _capture_call(call, results: list[dict], errors: list[Exception]) -> None:
        try:
            results.append(call())
        except Exception as exc:  # noqa: BLE001 -- test captures thread failures
            errors.append(exc)

    def test_close_revokes_authority_stops_and_rejects_late_frame(self):
        late = valid_frame(self.runtime, self.session, sequence=1)
        self.runtime.close()
        state = self.runtime.status()
        self.assertEqual(("released", "service_close"), (state["state"], state["reason"]))
        self.assertFalse(state["authority_valid"])
        self.assertEqual("closed", state["dispatch"]["state"])
        self.assertTrue(state["dispatch"]["stop_acknowledged"])
        with self.assertRaisesRegex(ProtocolError, "process is closing"):
            self.runtime.submit_shadow_frame(late, source="late")

    def test_pose_never_renews_core_lease(self):
        self.clock.advance(0.9)
        self.runtime.submit_shadow_frame(valid_frame(self.runtime, self.session), source="test")
        self.clock.advance(0.15)
        status = self.runtime.watchdog_tick()
        self.assertEqual("hold", status["state"])
        self.assertEqual("lease_timeout", status["reason"])
        self.assertFalse(status["authority_valid"])
        self.assertTrue(status["lease"]["expired_latched"])
        self.assertIsNone(status["pose"]["latest_sequence"])

    def test_pose_timeout_holds_and_requires_higher_clutch(self):
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1, clutch_sequence=4), source="test"
        )
        self.clock.advance(0.21)
        self.assertEqual("pose_timeout", self.runtime.watchdog_tick()["reason"])
        held = self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=2, clutch_sequence=4), source="test"
        )
        self.assertEqual("hold", held["state"])
        resumed = self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=3, clutch_sequence=5), source="test"
        )
        self.assertEqual("active_shadow", resumed["state"])

    def test_old_identity_is_rejected_after_new_epoch(self):
        old_identity = identity(self.runtime, self.session)
        newer = new_session(epoch=2, fence="n" * 32)
        self.runtime.prepare_shadow(newer)
        with self.assertRaisesRegex(ProtocolError, "session_id"):
            self.runtime.heartbeat(old_identity)

    def test_stale_peer_callbacks_cannot_pollute_new_session(self):
        old_generation = self.runtime.session_generation()
        self.runtime.mark_channel(old_generation, "teleop-control", True)
        newer = new_session(epoch=2, fence="n" * 32)
        self.runtime.prepare_shadow(newer)
        before = self.runtime.status()

        self.runtime.mark_channel(old_generation, "teleop-control", True)
        self.runtime.mark_channel(old_generation, "teleop-pose", True)
        self.runtime.mark_rtc_disconnected(old_generation, "rtc_closed")

        after = self.runtime.status()
        self.assertEqual("prepared_shadow", after["state"])
        self.assertEqual(before["rtc"]["channels"], after["rtc"]["channels"])
        self.assertFalse(after["rtc"]["connected"])
        self.assertEqual(3, after["counters"]["stale_rtc_callbacks"])

    def test_rtc_ping_cannot_renew_lease_by_runtime_contract(self):
        initial = self.runtime.status()["lease"]["age_ms"]
        generation = self.runtime.session_generation()
        self.clock.advance(0.5)
        self.runtime.mark_channel(generation, "teleop-control", True)
        self.runtime.mark_channel(generation, "teleop-pose", True)
        later = self.runtime.status()["lease"]["age_ms"]
        self.assertGreater(later, initial)

    def test_release_revokes_old_authority_and_is_not_revivable(self):
        old_identity = identity(self.runtime, self.session)
        old_generation = self.runtime.session_generation()
        delayed_frame = valid_frame(
            self.runtime,
            self.session,
            sequence=2,
            clutch_sequence=999,
        )
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1), source="test"
        )

        released = self.runtime.release(old_identity)
        self.assertEqual("released", released["state"])
        self.assertIsNone(released["session_id"])
        self.assertIsNone(released["lease"]["age_ms"])
        self.assertIsNone(released["pose"]["latest_sequence"])
        terminal_generation = self.runtime.session_generation()
        self.assertGreater(terminal_generation, old_generation)

        for old_command in (
            self.runtime.heartbeat,
            self.runtime.pause,
            self.runtime.soft_stop,
        ):
            with self.assertRaisesRegex(ProtocolError, "new prepare_shadow"):
                old_command(old_identity)
            self.assertEqual("released", self.runtime.status()["state"])

        with self.assertRaisesRegex(ProtocolError, "new prepare_shadow"):
            self.runtime.submit_shadow_frame(delayed_frame, source="delayed_rtc")
        with self.assertRaisesRegex(ProtocolError, "new prepare_shadow"):
            self.runtime.ticket_binding()

        # Both real stale callbacks and a fabricated callback using the
        # terminal generation are diagnostics-only and cannot leave released.
        self.runtime.mark_channel(old_generation, "teleop-control", True)
        self.runtime.mark_rtc_disconnected(old_generation, "rtc_closed")
        self.runtime.mark_channel(terminal_generation, "teleop-control", True)
        self.runtime.mark_rtc_disconnected(terminal_generation, "rtc_closed")
        terminal = self.runtime.watchdog_tick()
        self.assertEqual("released", terminal["state"])
        self.assertFalse(terminal["rtc"]["connected"])
        self.assertEqual({"teleop-control": False, "teleop-pose": False}, terminal["rtc"]["channels"])

        newer = new_session(epoch=2, fence="n" * 32)
        prepared = self.runtime.prepare_shadow(newer)
        self.assertEqual("prepared_shadow", prepared["state"])
        with self.assertRaises(ProtocolError):
            self.runtime.submit_shadow_frame(delayed_frame, source="delayed_rtc")

    def test_pause_cannot_be_downgraded_to_recoverable_hold(self):
        old_identity = identity(self.runtime, self.session)
        self.runtime.pause(old_identity)
        with self.assertRaisesRegex(ProtocolError, "paused session"):
            self.runtime.soft_stop(old_identity)
        with self.assertRaisesRegex(ProtocolError, "state paused"):
            self.runtime.submit_shadow_frame(
                valid_frame(self.runtime, self.session, clutch_sequence=999), source="test"
            )
        self.assertEqual("paused", self.runtime.status()["state"])

    def test_rtc_pose_requires_both_channels_in_same_generation(self):
        generation = self.runtime.session_generation()
        self.runtime.mark_channel(generation, "teleop-pose", True)
        with self.assertRaisesRegex(ProtocolError, "both teleop-control and teleop-pose"):
            self.runtime.submit_shadow_frame(
                valid_frame(self.runtime, self.session, sequence=1),
                source="rtc",
                rtc_generation=generation,
            )
        pose_only = self.runtime.status()
        self.assertEqual("hold", pose_only["state"])
        self.assertFalse(pose_only["rtc"]["connected"])
        self.assertIsNone(pose_only["pose"]["latest_sequence"])

        # The diagnostic MCP path remains deliberately independent from RTC.
        diagnostic = self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1),
            source="mcp_diagnostic",
        )
        self.assertEqual("active_shadow", diagnostic["state"])

    def test_control_close_blocks_pose_recovery_until_full_transport(self):
        generation = self.runtime.session_generation()
        self.runtime.mark_channel(generation, "teleop-control", True)
        self.runtime.mark_channel(generation, "teleop-pose", True)
        active = self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1, clutch_sequence=4),
            source="rtc",
            rtc_generation=generation,
        )
        self.assertEqual("active_shadow", active["state"])

        self.runtime.mark_channel(generation, "teleop-control", False)
        with self.assertRaisesRegex(ProtocolError, "both teleop-control and teleop-pose"):
            self.runtime.submit_shadow_frame(
                valid_frame(self.runtime, self.session, sequence=2, clutch_sequence=999),
                source="rtc",
                rtc_generation=generation,
            )
        held = self.runtime.status()
        self.assertEqual("hold", held["state"])
        self.assertFalse(held["rtc"]["connected"])
        self.assertEqual(1, held["pose"]["latest_sequence"])

    def test_stale_generation_pose_cannot_enter_new_session(self):
        stale_generation = self.runtime.session_generation()
        newer = new_session(epoch=2, fence="n" * 32)
        self.runtime.prepare_shadow(newer)
        self.runtime.mark_channel(stale_generation, "teleop-control", True)
        self.runtime.mark_channel(stale_generation, "teleop-pose", True)
        with self.assertRaisesRegex(ProtocolError, "stale session generation"):
            self.runtime.submit_shadow_frame(
                valid_frame(self.runtime, newer, sequence=1),
                source="rtc",
                rtc_generation=stale_generation,
            )
        state = self.runtime.status()
        self.assertEqual("prepared_shadow", state["state"])
        self.assertFalse(state["rtc"]["connected"])
        self.assertIsNone(state["pose"]["latest_sequence"])

    def test_old_peer_authority_cannot_enter_new_session(self):
        old_authority, old_generation = self.runtime.rtc_authority_snapshot()
        delayed_wire_frame = rtc_wire_frame(
            self.runtime,
            self.session,
            sequence=1,
        )
        newer = new_session(epoch=2, fence="n" * 32)
        self.runtime.prepare_shadow(newer)
        current_generation = self.runtime.session_generation()
        self.runtime.mark_channel(current_generation, "teleop-control", True)
        self.runtime.mark_channel(current_generation, "teleop-pose", True)

        with self.assertRaises(ProtocolError) as raised:
            self.runtime.submit_rtc_frame(
                delayed_wire_frame,
                authority=old_authority,
                rtc_generation=old_generation,
            )

        self.assertEqual("session_mismatch", raised.exception.code)
        state = self.runtime.status()
        self.assertEqual("prepared_shadow", state["state"])
        self.assertIsNone(state["pose"]["latest_sequence"])

    def test_lease_timeout_latch_cannot_be_overwritten_or_reclutched(self):
        old_identity = identity(self.runtime, self.session)
        old_generation = self.runtime.session_generation()
        delayed_frame = valid_frame(
            self.runtime,
            self.session,
            sequence=2,
            clutch_sequence=999,
        )
        self.runtime.submit_shadow_frame(
            valid_frame(self.runtime, self.session, sequence=1), source="test"
        )
        self.clock.advance(1.01)
        expired = self.runtime.watchdog_tick()
        self.assertEqual(("hold", "lease_timeout"), (expired["state"], expired["reason"]))
        self.assertFalse(expired["authority_valid"])
        self.assertIsNone(expired["session_id"])
        self.assertIsNone(expired["lease"]["age_ms"])
        self.assertIsNone(expired["pose"]["latest_sequence"])
        expired_generation = self.runtime.session_generation()
        self.assertGreater(expired_generation, old_generation)

        for old_command in (
            self.runtime.soft_stop,
            self.runtime.pause,
            self.runtime.heartbeat,
        ):
            with self.assertRaisesRegex(ProtocolError, "lease expired"):
                old_command(old_identity)
            self.assertEqual(("hold", "lease_timeout"), (
                self.runtime.status()["state"], self.runtime.status()["reason"]
            ))
        with self.assertRaisesRegex(ProtocolError, "lease expired"):
            self.runtime.submit_shadow_frame(delayed_frame, source="mcp_diagnostic")
        with self.assertRaisesRegex(ProtocolError, "lease expired"):
            self.runtime.ticket_binding()

        self.runtime.mark_channel(old_generation, "teleop-control", True)
        self.runtime.mark_rtc_disconnected(old_generation, "rtc_closed")
        self.runtime.mark_channel(expired_generation, "teleop-control", True)
        self.assertEqual("hold", self.runtime.status()["state"])
        self.assertFalse(self.runtime.status()["authority_valid"])
        self.runtime.watchdog_tick()
        self.assertEqual(1, self.runtime.status()["counters"]["lease_timeouts"])

        newer = new_session(epoch=2, fence="n" * 32)
        prepared = self.runtime.prepare_shadow(newer)
        self.assertTrue(prepared["authority_valid"])
        self.assertEqual("prepared_shadow", prepared["state"])
        heartbeat = self.runtime.heartbeat(identity(self.runtime, newer))
        self.assertTrue(heartbeat["lease"]["fresh"])

    def test_authority_boundaries_latch_overdue_lease_without_watchdog(self):
        boundaries = (
            "heartbeat", "pause", "soft_stop", "frame", "ticket", "rtc_callback", "status"
        )
        for boundary in boundaries:
            with self.subTest(boundary=boundary):
                clock = FakeClock()
                runtime = ShadowRuntime(clock=clock, auto_watchdog=False)
                try:
                    session = new_session()
                    runtime.prepare_shadow(session)
                    generation = runtime.session_generation()
                    frame = valid_frame(runtime, session)
                    old_identity = identity(runtime, session)
                    clock.advance(1.01)

                    if boundary == "rtc_callback":
                        runtime.mark_channel(generation, "teleop-control", True)
                    elif boundary == "status":
                        runtime.status()
                    else:
                        with self.assertRaisesRegex(ProtocolError, "lease expired"):
                            if boundary == "heartbeat":
                                runtime.heartbeat(old_identity)
                            elif boundary == "pause":
                                runtime.pause(old_identity)
                            elif boundary == "soft_stop":
                                runtime.soft_stop(old_identity)
                            elif boundary == "frame":
                                runtime.submit_shadow_frame(frame, source="mcp_diagnostic")
                            else:
                                runtime.ticket_binding()

                    state = runtime.status()
                    self.assertEqual(
                        ("hold", "lease_timeout"),
                        (state["state"], state["reason"]),
                    )
                    self.assertFalse(state["authority_valid"])
                    self.assertTrue(state["lease"]["expired_latched"])
                    self.assertGreater(runtime.session_generation(), generation)
                finally:
                    runtime.close()


if __name__ == "__main__":
    unittest.main()
