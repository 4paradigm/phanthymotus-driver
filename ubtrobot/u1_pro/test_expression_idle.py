"""Exercise the expression idle lifecycle through the real card/audio adapter."""

import io
import json
import threading
import time
import unittest
from unittest import mock

from test_device import FakeNodes
import device
from common.vendor_runtime import DriverBundle, make_handler


class ExpressionIdleTests(unittest.TestCase):
    def setUp(self):
        self.notify_patch = mock.patch.object(device, "_acp_notify")
        self.notify = self.notify_patch.start()
        self.nodes = FakeNodes()
        self.audio = device.AudioPlugin(self.nodes)
        self.control = mock.Mock()
        self.control.set_enabled.side_effect = lambda enabled: {"ok": True, "enabled": enabled}
        self.plugin = device.ExpressionPlugin(self.audio, idle_control=self.control)
        self.loop = self.plugin.idle_loop
        self.loop.GAP_MIN = self.loop.GAP_MAX = 0.01
        self.loop.POLL_INTERVAL = 0.005
        self.bundle = DriverBundle([self.audio, self.plugin])

    def tearDown(self):
        self.loop.stop()
        if self.loop._thread is not None:
            self.loop._thread.join(timeout=2)
            self.assertFalse(self.loop._thread.is_alive())
        if self.loop._end_thread is not None:
            self.loop._end_thread.join(timeout=2)
            self.assertFalse(self.loop._end_thread.is_alive())
        self.notify_patch.stop()

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail("Timed out waiting for background test condition")
            time.sleep(0.002)

    def start_loop(self):
        result = self.bundle.dispatch("expression", {"action": "idle_start", "action_id": "start-loop"})
        self.assertEqual(result["state"], "starting")
        return result

    def finish_active(self, success=True):
        with self.audio._lock:
            vendor_uuid = self.audio._active["vendor_uuid"]
        self.audio._on_playback_state({"uuid": vendor_uuid, "phase": "result",
                                       "success": success, "state_name": "COMPLETED" if success else "FAILED"})

    def test_schema_and_start_preserve_normal_expression_behavior(self):
        schema = self.plugin.get_tool()["inputSchema"]
        self.assertEqual(schema["x-completion"], {"actions": ["start", "play", "idle_start", "idle_stop", "stop"], "timeout": 240})
        self.assertNotIn("x-stop", schema)
        self.assertEqual(schema["x-resource"], device.VENDOR_ACTION_RESOURCE)
        self.assertIn("idle_stop", schema["properties"]["action"]["enum"])
        self.bundle.start_all()
        self.assertFalse(self.loop.info()["enabled"])
        self.assertEqual(self.nodes.string_calls, [])
        result = self.bundle.dispatch("expression", {"action": "play", "name": "smile", "action_id": "smile-1"})
        self.assertEqual(result["state"], "queued")
        self.assertNotIn("background", self.audio._active)
        self.finish_active()
        self.notify.assert_called_once()
        self.assertEqual(self.notify.call_args.args[:2], ("smile-1", "completed"))
        self.control.set_enabled.assert_not_called()

    def test_mcp_lifecycle_serializes_repeats_and_suppresses_per_preset_acp(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.assertTrue(self.audio._active["background"])
        time.sleep(0.04)
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.audio._on_playback_state({"uuid": "unrelated", "phase": "result", "success": True})
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.finish_active()
        self.wait_for(lambda: len(self.nodes.string_calls) == 2)
        self.loop.stop()
        self.finish_active()
        self.loop._thread.join(2)
        self.assertEqual(len(self.nodes.string_calls), 2)
        self.assertTrue(all(kind == "play_action" and args["action"] in {"A025", "A003", "A024", "A005", "A008", "A007"}
                            for kind, args in self.nodes.string_calls))
        self.notify.assert_called_once()
        self.assertEqual(self.notify.call_args.args[:2], ("start-loop", "completed"))
        self.control.set_enabled.assert_called_once_with(False)
        self.assertFalse(self.bundle.dispatch("expression", {"action": "info"})["idle_loop"]["enabled"])

    def test_failed_native_disable_reports_startup_error_without_motion(self):
        self.control.set_enabled.side_effect = RuntimeError("native disable failed")
        self.start_loop()
        self.loop._thread.join(2)
        self.assertEqual(self.nodes.string_calls, [])
        self.assertEqual(self.loop.info()["state"], "error")
        self.assertIn("native disable failed", self.loop.info()["last_error"])
        self.assertEqual(self.control.method_calls, [mock.call.set_enabled(False)])
        self.assertEqual(self.notify.call_args.args[:2], ("start-loop", "error"))

    def test_preflight_is_asynchronous_and_can_be_stopped_without_motion(self):
        entered, release = threading.Event(), threading.Event()

        def disable(enabled):
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = disable
        try:
            self.start_loop()
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.loop._thread.is_alive())
            self.loop.stop()
            with self.assertRaises(ValueError):
                self.start_loop()
        finally:
            release.set()
        self.loop._thread.join(2)
        self.assertEqual(self.nodes.string_calls, [])
        self.notify.assert_called_once()
        self.assertEqual(self.notify.call_args.args[:2], ("start-loop", "cancelled"))

    def test_filtered_catalog_and_duplicate_start(self):
        self.plugin = device.ExpressionPlugin(self.audio, {"A003": {}}, True, idle_control=self.control)
        self.loop = self.plugin.idle_loop
        self.loop.GAP_MIN = self.loop.GAP_MAX = 0.01
        self.bundle = DriverBundle([self.plugin])
        self.start_loop()
        with self.assertRaises(ValueError):
            self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.assertEqual(self.nodes.string_calls[0][1]["action"], "A003")
        self.assertEqual(self.loop.info()["available_expressions"], ["gaze"])

    def test_waits_for_foreground_and_idle_stop_does_not_interrupt_it(self):
        self.audio.dispatch("speak", {"text": "hello", "action_id": "foreground"})
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        time.sleep(0.04)
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.loop.stop()
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertEqual(self.audio._active["action_id"], "foreground")
        self.finish_active()
        self.assertIn(("foreground", "completed"), [c.args[:2] for c in self.notify.call_args_list])

    def test_idle_stop_allows_current_preset_to_finish_and_normal_play_stays_available(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        action_id = self.audio._active["action_id"]
        self.loop.stop()
        self.loop._thread.join(2)
        self.assertEqual(self.audio._active["action_id"], action_id)
        self.assertEqual(self.nodes.interrupts, 0)
        self.finish_active()
        result = self.bundle.dispatch("expression", {"action": "play", "name": "smile", "action_id": "normal"})
        self.assertEqual(result["state"], "queued")
        self.finish_active()
        self.assertEqual(self.notify.call_args.args[:2], ("normal", "completed"))

    def test_interrupt_stops_loop_then_uses_existing_sdk_interrupt(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        result = self.bundle.dispatch("expression", {"action": "interrupt"})
        self.loop._thread.join(2)
        self.assertEqual(result["state"], "idle")
        self.assertEqual(self.nodes.interrupts, 1)
        self.assertIsNone(self.audio._active)
        self.assertFalse(self.loop.info()["enabled"])
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.notify.assert_called_once()  # startup only; background cancellation is internal

    def test_failed_completion_stops_loop_without_retry(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.finish_active(success=False)
        self.loop._thread.join(2)
        self.assertEqual(self.loop.info()["state"], "error")
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.notify.assert_called_once()

    def test_missing_completion_times_out_without_releasing_or_interrupting_task(self):
        self.loop.ACTION_TIMEOUT = 0.025
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "error")
        self.loop._thread.join(2)
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.assertIsNotNone(self.audio._active)
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertIn("timed out", self.loop.info()["last_error"])

    def test_sdk_request_failure_stops_without_retry_and_escapes_error(self):
        self.nodes.string_call = mock.Mock(side_effect=RuntimeError("vendor\x00\x1b\nerror"))
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "error")
        self.loop._thread.join(2)
        self.nodes.string_call.assert_called_once()
        self.assertIsNone(self.audio._active)
        self.assertEqual(self.loop.info()["last_error"], "vendor\\x00\\x1b\\nerror")
        self.notify.assert_called_once()

    def test_no_supported_idle_presets_and_invalid_ids_are_rejected(self):
        expression = device.ExpressionPlugin(self.audio, {"A029": {}}, True, idle_control=self.control)
        with self.assertRaises(ValueError):
            expression.dispatch("idle_start", {})
        with self.assertRaises(ValueError):
            self.plugin.dispatch("idle_start", {"action_id": "bad\nid"})
        self.assertEqual(self.nodes.string_calls, [])

    def test_requested_palette_is_uniform_and_avoids_immediate_repeats(self):
        expected = {"confused", "gaze", "distracted", "frown", "pout", "smile"}
        self.assertEqual(set(self.loop.expressions), expected)
        self.assertEqual(set(self.loop.WEIGHTS.values()), {1})
        for previous in expected:
            for _ in range(15):
                self.assertIn(self.loop._pick(previous), expected - {previous})

    def test_manual_expression_interrupts_idle_then_resumes_after_completion(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        with self.audio._lock:
            old_uuid = self.audio._active["vendor_uuid"]
        sequence = []
        original = self.nodes.string_call

        def interrupt(name):
            self.nodes.interrupts += 1
            sequence.append("interrupt")
            return {"success": True}

        def play(name, args):
            sequence.append(args["action"])
            return original(name, args)

        self.nodes.trigger_call = interrupt
        self.nodes.string_call = play
        result = self.bundle.dispatch("expression", {"action": "play", "name": "smile", "action_id": "manual"})
        self.assertEqual(result["state"], "queued")
        self.assertIn("preempting_action_id", result)
        self.wait_for(lambda: sequence == ["interrupt", "A007"] and self.audio._handoff is None)
        self.assertTrue(self.loop.info()["enabled"])
        time.sleep(0.04)
        self.assertEqual(len(self.nodes.string_calls), 2)
        self.audio._on_playback_state({"uuid": old_uuid, "phase": "result", "success": True})
        self.assertEqual(self.audio._active["action_id"], "manual")
        self.finish_active()
        self.assertEqual([c.args[:2] for c in self.notify.call_args_list],
                         [("start-loop", "completed"), ("manual", "completed")])
        self.wait_for(lambda: len(self.nodes.string_calls) == 3)
        self.assertTrue(self.audio._active["background"])
        self.control.set_enabled.assert_called_once_with(False)

    def test_handoff_blocks_other_tasks_even_if_old_completion_arrives_during_interrupt(self):
        entered, release = threading.Event(), threading.Event()
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)

        def interrupt(name):
            entered.set()
            release.wait(2)
            return {"success": True}

        self.nodes.trigger_call = interrupt
        try:
            result = self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
            self.assertEqual(result["state"], "queued")
            self.assertTrue(entered.wait(1))
            self.finish_active()  # Old slot clears before interrupt acknowledgement.
            rejected = self.audio.dispatch("speak", {"text": "hello", "action_id": "tts-other"})
            self.assertEqual(rejected["state"], "error")
            self.assertEqual(len(self.nodes.string_calls), 1)
        finally:
            release.set()
        self.wait_for(lambda: len(self.nodes.string_calls) == 2 and self.audio._handoff is None)
        self.assertEqual(self.audio._active["action_id"], "manual")
        self.finish_active()

    def test_interrupt_failure_preserves_old_slot_and_reports_foreground_error_once(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        old_id = self.audio._active["action_id"]
        self.nodes.trigger_call = mock.Mock(return_value={"success": False, "message": "denied"})
        self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.wait_for(lambda: self.audio._handoff is None)
        self.assertEqual(self.audio._active["action_id"], old_id)
        self.assertEqual(len(self.nodes.string_calls), 1)
        reports = [c for c in self.notify.call_args_list if c.args[0] == "manual"]
        self.assertEqual([c.args[1] for c in reports], ["error"])
        self.assertIn("did not confirm", reports[0].args[2]["message"])

    def test_interruption_transport_error_does_not_send_foreground(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.nodes.trigger_call = mock.Mock(side_effect=TimeoutError("interrupt\x00timeout"))
        self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.wait_for(lambda: self.audio._handoff is None)
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.assertIsNotNone(self.audio._active)
        self.assertEqual(self.notify.call_args.args[:2], ("manual", "error"))

    def test_inflight_idle_submission_finishes_before_interrupt_and_foreground(self):
        entered, release = threading.Event(), threading.Event()
        original = self.nodes.string_call
        sequence = []

        def play(name, args):
            if not sequence:
                sequence.append("idle-submit-entered")
                entered.set()
                release.wait(2)
                sequence.append("idle-submit-accepted")
            else:
                sequence.append("foreground-submit")
            return original(name, args)

        def interrupt(name):
            sequence.append("interrupt")
            return {"success": True}

        self.nodes.string_call = play
        self.nodes.trigger_call = interrupt
        self.start_loop()
        try:
            self.assertTrue(entered.wait(1))
            result = self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
            self.assertEqual(result["state"], "queued")
            self.assertEqual(sequence, ["idle-submit-entered"])
        finally:
            release.set()
        self.wait_for(lambda: self.audio._handoff is None and len(self.nodes.string_calls) == 2)
        self.assertEqual(sequence, ["idle-submit-entered", "idle-submit-accepted", "interrupt", "foreground-submit"])
        self.finish_active()

    def test_failed_inflight_idle_submission_does_not_send_interrupt_or_foreground(self):
        entered, release = threading.Event(), threading.Event()

        def play(name, args):
            entered.set()
            release.wait(2)
            raise TimeoutError("idle submission unconfirmed")

        self.nodes.string_call = mock.Mock(side_effect=play)
        self.start_loop()
        try:
            self.assertTrue(entered.wait(1))
            self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        finally:
            release.set()
        self.wait_for(lambda: self.audio._handoff is None)
        self.nodes.string_call.assert_called_once()
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertEqual(self.notify.call_args.args[:2], ("manual", "error"))

    def test_idle_completed_before_handoff_does_not_send_unnecessary_interrupt(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        with self.audio._command_lock:
            self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
            self.finish_active()
            self.assertEqual(self.audio.dispatch("speak", {"text": "hello"})["state"], "error")
        self.wait_for(lambda: len(self.nodes.string_calls) == 2 and self.audio._handoff is None)
        self.assertEqual(self.nodes.interrupts, 0)
        self.finish_active()

    def test_idle_in_gap_stays_enabled_and_manual_expression_needs_no_interrupt(self):
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        result = self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.assertEqual(result["state"], "queued")
        self.assertTrue(self.loop.info()["enabled"])
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertEqual(self.nodes.string_calls[0][1]["action"], "A007")
        self.finish_active()

    def test_normal_play_does_not_interrupt_existing_foreground_task(self):
        self.audio.dispatch("speak", {"text": "hello", "action_id": "foreground"})
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        result = self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertEqual(self.audio._active["action_id"], "foreground")
        self.assertEqual(len(self.nodes.string_calls), 1)
        self.finish_active()

    def test_explicit_stop_cancels_pending_handoff_without_duplicate_acp(self):
        entered, release = threading.Event(), threading.Event()
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)

        def interrupt(name):
            entered.set()
            release.wait(2)
            return {"success": True}

        self.nodes.trigger_call = interrupt
        self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.assertTrue(entered.wait(1))
        stopper = threading.Thread(target=self.plugin.stop)
        try:
            stopper.start()
            self.wait_for(lambda: self.audio._interrupting)
        finally:
            release.set()
        stopper.join(2)
        self.assertFalse(stopper.is_alive())
        self.wait_for(lambda: self.audio._handoff is None)
        self.assertEqual(len(self.nodes.string_calls), 1)
        reports = [c.args[1] for c in self.notify.call_args_list if c.args[0] == "manual"]
        self.assertEqual(reports, ["cancelled"])

    def test_stopped_idle_not_yet_sent_is_skipped_before_foreground(self):
        with self.audio._command_lock:
            self.start_loop()
            self.wait_for(lambda: self.audio._active is not None)
            self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
            self.assertEqual(self.nodes.string_calls, [])
        self.wait_for(lambda: self.audio._handoff is None and len(self.nodes.string_calls) == 1)
        self.assertEqual(self.nodes.string_calls[0][1]["action"], "A007")
        self.assertEqual(self.nodes.interrupts, 0)
        self.finish_active()
        self.wait_for(lambda: len(self.nodes.string_calls) == 2)
        self.assertTrue(self.loop.info()["enabled"])
        self.assertTrue(self.audio._active["background"])

    def test_explicit_stop_during_manual_expression_prevents_cycle_resumption(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.plugin.dispatch("play", {"name": "smile", "action_id": "manual"})
        self.wait_for(lambda: len(self.nodes.string_calls) == 2 and self.audio._handoff is None)
        self.loop.stop()
        self.finish_active()
        self.loop._thread.join(2)
        time.sleep(0.04)
        self.assertFalse(self.loop.info()["enabled"])
        self.assertEqual(len(self.nodes.string_calls), 2)

    def test_failed_explicit_interrupt_keeps_active_and_does_not_report_cancelled(self):
        self.audio.dispatch("speak", {"text": "hello", "action_id": "foreground"})
        self.nodes.trigger_call = mock.Mock(return_value={"success": False})
        result = self.audio.stop()
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.audio._active["action_id"], "foreground")
        self.notify.assert_not_called()
        self.finish_active()

    def test_foreground_submission_error_reports_once_and_releases_handoff(self):
        original = self.nodes.string_call

        def play(name, args):
            if args.get("action") == "A033":
                raise RuntimeError("foreground rejected")
            return original(name, args)

        self.nodes.string_call = play
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.plugin.dispatch("play", {"name": "laugh", "action_id": "manual"})
        self.wait_for(lambda: self.audio._handoff is None)
        self.assertIsNone(self.audio._active)
        reports = [c.args[1] for c in self.notify.call_args_list if c.args[0] == "manual"]
        self.assertEqual(reports, ["error"])

    def test_unconfirmed_interrupt_response_keeps_active(self):
        self.audio.dispatch("speak", {"text": "hello", "action_id": "foreground"})
        self.nodes.trigger_call = mock.Mock(return_value={})
        result = self.audio.stop()
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.audio._active["action_id"], "foreground")
        self.notify.assert_not_called()
        self.finish_active()

    def end_loop(self):
        result = self.bundle.dispatch("expression", {"action": "idle_stop", "action_id": "end-loop"})
        self.assertEqual(result["state"], "restoring")
        self.assertEqual(result["action_id"], "end-loop")
        return result

    def test_takeover_disables_native_before_first_expression(self):
        entered, release = threading.Event(), threading.Event()

        def disable(enabled):
            self.assertIs(enabled, False)
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": False}

        self.control.set_enabled.side_effect = disable
        try:
            self.start_loop()
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.nodes.string_calls, [])
            self.assertEqual(self.loop.info()["state"], "starting")
        finally:
            release.set()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)

    def test_end_waits_for_current_completion_then_enables_native(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.end_loop()
        self.assertEqual(self.audio.dispatch("speak", {"text": "blocked"})["state"], "error")
        time.sleep(0.02)
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False)])
        self.assertEqual(self.nodes.interrupts, 0)
        self.finish_active()
        self.loop._end_thread.join(2)
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False), mock.call(True)])
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "completed"))
        self.assertTrue(self.notify.call_args.args[2]["vendor_idle_enabled"])
        self.assertFalse(self.loop.info()["smart_control_active"])
        self.assertFalse(self.audio._native_restoring)
        self.assertEqual(len(self.nodes.string_calls), 1)

    def test_end_during_disable_serializes_disable_then_enable(self):
        entered, release = threading.Event(), threading.Event()

        def change(enabled):
            if not enabled:
                entered.set()
                release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = change
        try:
            self.start_loop()
            self.assertTrue(entered.wait(1))
            self.end_loop()
            with self.assertRaises(ValueError):
                self.start_loop()
            with self.assertRaises(ValueError):
                self.end_loop()
            self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False)])
        finally:
            release.set()
        self.loop._end_thread.join(2)
        self.assertEqual(self.nodes.string_calls, [])
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False), mock.call(True)])
        self.assertEqual([c.args[:2] for c in self.notify.call_args_list],
                         [("start-loop", "cancelled"), ("end-loop", "completed")])

    def test_restore_failure_is_reported_and_can_be_retried(self):
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        self.control.set_enabled.side_effect = RuntimeError("enable\x00failed")
        self.end_loop()
        self.loop._end_thread.join(2)
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "error"))
        self.assertEqual(self.loop.info()["last_error"], "enable\\x00failed")
        self.assertTrue(self.loop.info()["smart_control_active"])
        self.assertFalse(self.audio._native_restoring)
        self.control.set_enabled.side_effect = lambda enabled: {"ok": True, "enabled": enabled}
        self.end_loop()
        self.loop._end_thread.join(2)
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "completed"))
        self.assertFalse(self.loop.info()["smart_control_active"])

    def test_restore_timeout_does_not_enable_native_over_unconfirmed_task(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.loop.ACTION_TIMEOUT = 0.02
        self.end_loop()
        self.loop._end_thread.join(2)
        self.control.set_enabled.assert_called_once_with(False)
        self.assertIsNotNone(self.audio._active)
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "error"))
        self.assertTrue(self.loop.info()["smart_control_active"])
        self.finish_active()

    def test_native_restore_inflight_blocks_play_and_restart_until_readback(self):
        entered, release = threading.Event(), threading.Event()
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")

        def enable(enabled):
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = enable
        try:
            self.end_loop()
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.plugin.dispatch("play", {"name": "smile"})["state"], "error")
            with self.assertRaises(ValueError):
                self.start_loop()
            self.assertEqual(self.nodes.string_calls, [])
            self.assertTrue(self.loop.info()["restoring_vendor_idle"])
        finally:
            release.set()
        self.loop._end_thread.join(2)
        self.assertFalse(self.loop.info()["restoring_vendor_idle"])

    def test_manual_expression_keeps_native_disabled_until_explicit_end(self):
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        self.plugin.dispatch("play", {"name": "smile"})
        self.finish_active()
        self.control.set_enabled.assert_called_once_with(False)
        self.assertTrue(self.loop.info()["smart_control_active"])
        self.assertTrue(self.loop.info()["enabled"])
        self.end_loop()
        self.loop._end_thread.join(2)
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False), mock.call(True)])

    def test_explicit_stop_interrupts_and_restores_with_correlated_completion(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        response = self.bundle.dispatch("expression", {"action": "stop", "action_id": "card-stop"})
        self.assertEqual(response["action_id"], "card-stop")
        self.loop._end_thread.join(2)
        self.assertEqual(self.nodes.interrupts, 1)
        self.assertIsNone(self.audio._active)
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False), mock.call(True)])
        self.assertEqual(self.notify.call_args.args[:2], ("card-stop", "completed"))

    def test_shutdown_restores_owned_session_but_unused_card_does_not_write(self):
        self.plugin.stop()
        self.control.set_enabled.assert_not_called()
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        self.assertEqual(self.plugin.stop()["state"], "idle")
        self.assertEqual(self.control.set_enabled.call_args_list, [mock.call(False), mock.call(True)])

    def test_unconfirmed_disable_does_not_start_loop_and_end_can_restore(self):
        self.control.set_enabled.side_effect = None
        self.control.set_enabled.return_value = {"ok": True, "enabled": True}
        self.start_loop()
        self.loop._thread.join(2)
        self.assertEqual(self.loop.info()["state"], "error")
        self.assertTrue(self.loop.info()["smart_control_active"])
        self.assertEqual(self.nodes.string_calls, [])
        self.end_loop()
        self.loop._end_thread.join(2)
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "completed"))

    def test_shutdown_restores_when_an_existing_exit_is_waiting_for_motion(self):
        self.start_loop()
        self.wait_for(lambda: self.audio._active is not None)
        self.end_loop()
        self.nodes.trigger_call = mock.Mock(return_value={"success": False})
        result = self.plugin.stop()
        self.assertEqual(result["state"], "error")
        self.assertFalse(self.loop.info()["smart_control_active"])
        self.assertEqual(self.control.set_enabled.call_args.args, (True,))
        self.finish_active()

    def test_shutdown_reports_both_interrupt_and_restoration_failure(self):
        self.start_loop()
        self.wait_for(lambda: self.audio._active is not None)
        self.nodes.trigger_call = mock.Mock(return_value={"success": False})
        self.control.set_enabled.side_effect = RuntimeError("restore failed")
        result = self.plugin.stop()
        self.assertEqual(result["state"], "error")
        self.assertIn("interrupt_error", result["interrupt_result"])
        self.assertIn("restore failed", result["idle_loop"]["last_error"])
        self.assertTrue(result["idle_loop"]["smart_control_active"])
        self.control.set_enabled.assert_called_with(True)
        self.finish_active()

    def test_shutdown_restores_native_idle_even_when_sdk_interrupt_fails(self):
        for failure in ({"success": False}, TimeoutError("SDK interrupt timed out")):
            with self.subTest(failure=failure):
                self.start_loop()
                self.wait_for(lambda: self.audio._active is not None)
                if isinstance(failure, Exception):
                    self.nodes.trigger_call = mock.Mock(side_effect=failure)
                else:
                    self.nodes.trigger_call = mock.Mock(return_value=failure)
                result = self.plugin.stop()
                self.assertEqual(result["state"], "error")
                self.assertIn("interrupt_error", result)
                self.assertFalse(self.loop.info()["smart_control_active"])
                self.assertEqual(self.control.set_enabled.call_args.args, (True,))
                self.assertIsNotNone(self.audio._active)
                self.finish_active()

    def test_failed_stop_interrupt_does_not_restore_over_active_motion(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.nodes.trigger_call = mock.Mock(return_value={"success": False})
        self.plugin.dispatch("stop", {"action_id": "failed-stop"})
        self.loop._end_thread.join(2)
        self.assertEqual(self.notify.call_args.args[:2], ("failed-stop", "completed"))
        self.assertEqual(self.loop.info()["state"], "error")
        self.control.set_enabled.assert_called_once_with(False)
        self.assertIsNotNone(self.audio._active)
        self.assertTrue(self.loop.info()["smart_control_active"])
        self.finish_active()

    def test_end_waits_for_foreground_without_interrupting_it_and_can_restart(self):
        self.audio.dispatch("speak", {"text": "hello", "action_id": "foreground"})
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        self.end_loop()
        time.sleep(0.02)
        self.control.set_enabled.assert_called_once_with(False)
        self.assertEqual(self.nodes.interrupts, 0)
        self.finish_active()
        self.loop._end_thread.join(2)
        self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "completed"))
        self.start_loop()
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        self.assertEqual(self.control.set_enabled.call_args_list,
                         [mock.call(False), mock.call(True), mock.call(False)])

    def test_restoration_waits_for_startup_and_does_not_enable_if_it_is_still_unconfirmed(self):
        entered, release = threading.Event(), threading.Event()

        def disable(enabled):
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = disable
        self.loop.TRANSITION_TIMEOUT = 0.02
        try:
            self.start_loop()
            self.assertTrue(entered.wait(1))
            self.end_loop()
            self.loop._end_thread.join(2)
            self.assertEqual(self.notify.call_args.args[:2], ("end-loop", "error"))
            self.control.set_enabled.assert_called_once_with(False)
            self.assertTrue(self.loop.info()["smart_control_active"])
        finally:
            release.set()

    def call_expression_mcp(self, action, **arguments):
        request = json.dumps({"jsonrpc": "2.0", "id": "expression-request", "method": "tools/call",
                              "params": {"name": "expression", "arguments": {"action": action, **arguments}}}).encode()
        handler_type = make_handler(lambda: self.bundle, "u1-test", "u1-test")
        handler = object.__new__(handler_type)
        handler.path = "/mcp"
        handler.headers = {"Content-Length": str(len(request))}
        handler.rfile = io.BytesIO(request)
        handler.send_json = mock.Mock()
        handler.do_POST()
        http_status, response = handler.send_json.call_args.args
        self.assertEqual(http_status, 200)
        self.assertNotIn("error", response)
        return json.loads(response["result"]["content"][0]["text"])

    def test_canvas_start_automatically_takes_over_without_blocking_mcp(self):
        entered, release = threading.Event(), threading.Event()

        def disable(enabled):
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = disable
        try:
            self.bundle.start_all()
            self.control.set_enabled.assert_not_called()
            result = self.call_expression_mcp("start", instance_id="canvas-expression", action_id="canvas-start")
            self.assertEqual(result["state"], "loading")
            self.assertEqual(result["action_id"], "canvas-start")
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.call_expression_mcp("info")["state"], "loading")
            self.assertEqual(self.nodes.string_calls, [])
            self.notify.assert_not_called()
        finally:
            release.set()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        self.assertEqual(self.call_expression_mcp("info")["state"], "ready")
        self.control.set_enabled.assert_called_once_with(False)
        self.assertEqual(self.notify.call_args.args[:2], ("canvas-start", "completed"))

    def test_failed_canvas_start_is_visible_in_info_and_correlated_completion(self):
        self.control.set_enabled.side_effect = RuntimeError("native disable denied")
        result = self.call_expression_mcp("start", instance_id="canvas-expression")
        self.loop._thread.join(2)
        self.assertTrue(result["action_id"])
        info = self.call_expression_mcp("info")
        self.assertEqual(info["state"], "error")
        self.assertIn("native disable denied", info["message"])
        self.assertEqual(self.notify.call_args.args[:2], (result["action_id"], "error"))
        self.assertEqual(self.nodes.string_calls, [])

    def test_repeated_canvas_start_reuses_running_cycle(self):
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        first = self.call_expression_mcp("start")
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        worker = self.loop._thread
        again = self.call_expression_mcp("start")
        self.assertEqual(again["state"], "ready")
        self.assertEqual(again["action_id"], first["action_id"])
        self.assertIs(worker, self.loop._thread)
        self.control.set_enabled.assert_called_once_with(False)

    def test_canvas_stop_acknowledges_before_background_restore_finishes(self):
        self.loop.GAP_MIN = self.loop.GAP_MAX = 10
        self.call_expression_mcp("start")
        self.wait_for(lambda: self.loop.info()["state"] == "running")
        entered, release = threading.Event(), threading.Event()

        def enable(enabled):
            entered.set()
            release.wait(2)
            return {"ok": True, "enabled": enabled}

        self.control.set_enabled.side_effect = enable
        try:
            result = self.call_expression_mcp("stop", action_id="canvas-stop")
            self.assertEqual(result["state"], "idle")
            self.assertTrue(result["accepted"])
            self.assertEqual(result["completion_scope"], "request_accepted")
            self.assertTrue(entered.wait(1))
            info = self.call_expression_mcp("info")
            self.assertEqual(info["state"], "idle")
            self.assertTrue(info["idle_loop"]["restoring_vendor_idle"])
            self.assertEqual(self.notify.call_args.args[:2], ("canvas-stop", "completed"))
        finally:
            release.set()
        self.loop._end_thread.join(2)
        self.assertEqual(self.call_expression_mcp("info")["state"], "idle")
        self.assertEqual(self.notify.call_args.args[:2], ("canvas-stop", "completed"))


if __name__ == "__main__":
    unittest.main()
