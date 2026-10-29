"""Exercise the expression idle lifecycle through the real card/audio adapter."""

import threading
import time
import unittest
from unittest import mock

from test_device import FakeNodes
import device
from common.vendor_runtime import DriverBundle


class ExpressionIdleTests(unittest.TestCase):
    def setUp(self):
        self.notify_patch = mock.patch.object(device, "_acp_notify")
        self.notify = self.notify_patch.start()
        self.nodes = FakeNodes()
        self.audio = device.AudioPlugin(self.nodes)
        self.control = mock.Mock()
        self.control.status.return_value = {"ok": True, "data": {"enabled": False}}
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
        self.assertEqual(schema["x-completion"], {"actions": ["play", "idle_start"], "timeout": 120})
        self.assertEqual(schema["x-resource"], device.VENDOR_ACTION_RESOURCE)
        self.assertIn("idle_stop", schema["properties"]["action"]["enum"])
        self.assertEqual(self.bundle.dispatch("expression", {"action": "start"}), {"state": "ready"})
        self.assertFalse(self.loop.info()["enabled"])
        self.assertEqual(self.nodes.string_calls, [])
        result = self.bundle.dispatch("expression", {"action": "play", "name": "smile", "action_id": "smile-1"})
        self.assertEqual(result["state"], "queued")
        self.assertNotIn("background", self.audio._active)
        self.finish_active()
        self.notify.assert_called_once()
        self.assertEqual(self.notify.call_args.args[:2], ("smile-1", "completed"))
        self.control.status.assert_not_called()

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
        self.bundle.dispatch("expression", {"action": "idle_stop"})
        self.finish_active()
        self.loop._thread.join(2)
        self.assertEqual(len(self.nodes.string_calls), 2)
        self.assertTrue(all(kind == "play_action" and args["action"] in {"A001", "A003", "A024"}
                            for kind, args in self.nodes.string_calls))
        self.notify.assert_called_once()
        self.assertEqual(self.notify.call_args.args[:2], ("start-loop", "completed"))
        self.control.status.assert_called_once_with()
        self.assertFalse(self.bundle.dispatch("expression", {"action": "info"})["idle_loop"]["enabled"])

    def test_vendor_idle_enabled_is_reported_as_startup_error_without_motion_or_write(self):
        self.control.status.return_value = {"ok": True, "data": {"enabled": True}}
        self.start_loop()
        self.loop._thread.join(2)
        self.assertEqual(self.nodes.string_calls, [])
        self.assertEqual(self.loop.info()["state"], "error")
        self.assertIn("idle_actions.disable", self.loop.info()["last_error"])
        self.assertEqual(self.control.method_calls, [mock.call.status()])
        self.assertEqual(self.notify.call_args.args[:2], ("start-loop", "error"))

    def test_preflight_is_asynchronous_and_can_be_stopped_without_motion(self):
        entered, release = threading.Event(), threading.Event()

        def status():
            entered.set()
            release.wait(2)
            return {"ok": True, "data": {"enabled": False}}

        self.control.status.side_effect = status
        try:
            self.start_loop()
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.loop._thread.is_alive())
            self.bundle.dispatch("expression", {"action": "idle_stop"})
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
        self.bundle.dispatch("expression", {"action": "idle_stop"})
        self.assertEqual(self.nodes.interrupts, 0)
        self.assertEqual(self.audio._active["action_id"], "foreground")
        self.finish_active()
        self.assertIn(("foreground", "completed"), [c.args[:2] for c in self.notify.call_args_list])

    def test_idle_stop_allows_current_preset_to_finish_and_normal_play_stays_available(self):
        self.start_loop()
        self.wait_for(lambda: len(self.nodes.string_calls) == 1)
        action_id = self.audio._active["action_id"]
        self.bundle.dispatch("expression", {"action": "idle_stop"})
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
        expression = device.ExpressionPlugin(self.audio, {"A007": {}}, True, idle_control=self.control)
        with self.assertRaises(ValueError):
            expression.dispatch("idle_start", {})
        with self.assertRaises(ValueError):
            self.plugin.dispatch("idle_start", {"action_id": "bad\nid"})
        self.assertEqual(self.nodes.string_calls, [])


if __name__ == "__main__":
    unittest.main()
