"""Hardware-free tests of the actual SpeakerPlugin, with ROS/SDK test doubles.

Load the class and audio constants from device.py's AST so importing unrelated
camera/ROS drivers is unnecessary. No Speaker implementation is duplicated here.
Run: python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import ast
import copy
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from uuid import uuid4


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration

    def time(self):
        return self.now

    def time_ns(self):
        return int(self.now * 1_000_000_000)


class FakeNode:
    def __init__(self, name):
        self.subscriptions = []
        self.create_error = False
        self.destroy_failures = 0
        self.destroy_exceptions = 0
        self.destroy_success_result = True

    def create_subscription(self, message_type, topic, callback, qos):
        if self.create_error:
            raise RuntimeError("subscription failed")
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if self.destroy_exceptions:
            self.destroy_exceptions -= 1
            raise RuntimeError("subscription destruction failed")
        if self.destroy_failures:
            self.destroy_failures -= 1
            return False
        self.subscriptions.remove(subscription)
        return self.destroy_success_result

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warn=lambda _: None)


class FakeMedia:
    ROUTES = (
        "internal_capture_audio_data_to_agent",
        "external_custom_audio_data_to_agent",
        "internal_agent_audio_data_to_playback",
        "external_custom_audio_data_to_playback",
    )

    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.timed_calls = []
        self.config_calls = []
        self.routes = dict.fromkeys(self.ROUTES, True)
        self.work, self.reason = "READY", "NONE"
        self.status_queue = []
        self.failures = {}
        self.stale_getters = False
        self.volume = 100
        self.published = []
        self.hold_wakeup = False
        self.hold_sleep = False
        self.hold_reset = False
        self.reset_ack = True
        self.on_set = None

    def _record(self, name, *args):
        self.calls.append((name, args))
        self.timed_calls.append((name, self.clock.monotonic()))
        specific = (name, *args)
        key = specific if specific in self.failures else name
        count = self.failures.get(key, 0)
        if count:
            if count > 0:
                self.failures[key] -= 1
            raise RuntimeError(f"injected failure: {name}")

    def __getattr__(self, name):
        for route in self.ROUTES:
            if name == f"get_{route}_enable":
                def getter(route=route, name=name):
                    self._record(name)
                    return False if self.stale_getters else self.routes[route]
                return getter
            if name == f"set_{route}_enable":
                def setter(enabled, route=route, name=name):
                    start = self.clock.monotonic()
                    try:
                        if self.on_set:
                            self.on_set(name, enabled)
                        self._record(name, enabled)
                        self.routes[route] = enabled
                    finally:
                        self.clock.sleep(0.03)
                        self.config_calls.append((name, start, self.clock.monotonic()))
                return setter
        raise AttributeError(name)

    def get_system_status(self):
        self._record("get_system_status")
        if self.status_queue:
            self.work, self.reason = self.status_queue.pop(0)
        return SimpleNamespace(value=self.work, reason=self.reason)

    def get_system_error(self):
        self._record("get_system_error")
        return SimpleNamespace(code=0, message="")

    def pause_audio_playback(self):
        self._record("pause_audio_playback")

    def resume_audio_playback(self):
        self._record("resume_audio_playback")

    def resume_audio_capture(self):
        self._record("resume_audio_capture")

    def wakeup(self):
        self._record("wakeup")
        if not self.hold_wakeup:
            self.work, self.reason = "WAKEUPED", "CMD_WAKEUPED"

    def sleep(self):
        self._record("sleep")
        if not self.hold_sleep:
            self.work, self.reason = "SLEEPED", "CMD_SLEEPED"

    def restart(self):
        self._record("restart")
        self.routes = dict.fromkeys(self.ROUTES, True)
        if self.hold_reset:
            self.work, self.reason = "EXIT", "CMD_RESET"
            self.status_queue = []
            return
        self.status_queue = [("EXIT", "CMD_RESET")] if self.reset_ack else []
        self.status_queue.append(("READY", "NONE"))

    def publish_external_audio_playback_stream(self, stream):
        self._record("publish_external_audio_playback_stream")
        self.published.append(stream)

    def get_volume(self):
        self._record("get_volume")
        return self.volume

    def set_volume(self, volume):
        start = self.clock.monotonic()
        try:
            self._record("set_volume", volume)
            self.volume = volume
        finally:
            self.clock.sleep(0.03)
            self.config_calls.append(("set_volume", start, self.clock.monotonic()))


def load_speaker(clock, acp_notify=lambda *_: None):
    path = Path(__file__).resolve().parents[1] / "device.py"
    source = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [node for node in source.body if (
        isinstance(node, ast.ImportFrom) and node.module == "__future__"
        or isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id.startswith("_AUDIO_")
            for target in node.targets
        )
        or isinstance(node, ast.ClassDef) and node.name == "SpeakerPlugin"
    )]
    namespace = {
        "time": clock, "threading": threading, "copy": copy, "struct": struct,
        "Node": FakeNode, "AudioChunk": SimpleNamespace, "_LOW_LAT_QOS": object(),
        "uuid4": uuid4, "_acp_notify": acp_notify,
    }
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)
    return namespace["SpeakerPlugin"]


class SpeakerTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.media = FakeMedia(self.clock)
        self.acp_notifications = []
        self.acp_event = threading.Event()

        def notify(action_id, status, result, tool):
            self.acp_notifications.append((action_id, status, result, tool))
            self.acp_event.set()

        self.speaker = load_speaker(self.clock, notify)(
            {}, "bumi", SimpleNamespace(add_node=lambda _: None), self.media,
        )
        self.node = self.speaker._node
        self.addCleanup(patch.stopall)
        patch.dict(sys.modules, {"mediacontrol_py": SimpleNamespace(AudioStream=SimpleNamespace)}).start()

    def action(self, action, **args):
        if action in self.speaker._LONG_ACTIONS:
            return self.speaker._run_long_action(action, args)
        return self.speaker.dispatch(action, args)

    def names(self):
        return [name for name, _ in self.media.calls]

    def assert_isolated(self):
        for route in self.speaker._AGENT_ROUTES:
            self.assertFalse(self.media.routes[route], route)

    def assert_no_stream(self):
        self.assertFalse(self.speaker._playing)
        self.assertIsNone(self.speaker._sub)
        self.assertFalse(self.media.routes[self.speaker._EXTERNAL_PLAYBACK_ROUTE])

    def assert_config_gap(self):
        for previous, following in zip(self.media.config_calls, self.media.config_calls[1:]):
            self.assertGreaterEqual(following[1] - previous[2] + 1e-9, 0.8)

    @staticmethod
    def chunk(data=b"\x01\x00\xfe\xff", fmt="audio/pcm-16k"):
        return SimpleNamespace(format=fmt, data=data)

    def test_bootstrap_and_schema(self):
        self.speaker.start()
        self.assertEqual(self.media.calls, [])
        schema = self.speaker.get_tool()["inputSchema"]
        actions = set(schema["properties"]["action"]["enum"])
        self.assertEqual(actions, {"start", "play", "stop", "info", "get_volume", "set_volume", "wakeup", "sleep", "reset"})
        self.assertEqual(actions, set(schema["x-action-params"]))
        self.assertEqual(schema["x-action-params"]["play"]["params"], ["input_topic"])
        self.assertEqual(
            set(schema["x-completion"]["actions"]),
            {"start", "play", "wakeup", "sleep", "reset"},
        )
        self.assertEqual(schema["x-completion"]["timeout"], 60)
        self.assertEqual(schema["x-resource"], "mouth")

    def test_long_action_is_queued_and_conflicting_writes_return_busy(self):
        entered = threading.Event()
        release = threading.Event()

        def block_first_set(name, enabled):
            if not entered.is_set():
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test gate timed out")

        self.media.on_set = block_first_set
        queued = self.speaker.dispatch("wakeup", {})
        self.assertEqual(queued["state"], "queued")
        self.assertTrue(queued["action_id"].startswith("speaker_wakeup_"))
        self.assertTrue(entered.wait(1))
        active = self.speaker._active_action
        try:
            info = self.speaker.dispatch("info", {})
            self.assertEqual(info["active_action"]["action_id"], queued["action_id"])
            for action, args in (
                ("play", {"input_topic": "/new"}),
                ("stop", {}),
                ("set_volume", {"volume": 120}),
            ):
                with self.subTest(action=action):
                    busy = self.speaker.dispatch(action, args)
                    self.assertEqual(busy["state"], "busy")
                    self.assertEqual(busy["active_action_id"], queued["action_id"])
            self.assertEqual(self.speaker.dispatch("get_volume", {}), {"volume": 100})
        finally:
            release.set()
            active["thread"].join(2)
        self.assertFalse(active["thread"].is_alive())
        self.assertEqual(len(self.acp_notifications), 1)
        action_id, status, result, tool = self.acp_notifications[0]
        self.assertEqual((action_id, status, tool), (queued["action_id"], "completed", "speaker"))
        self.assertEqual(result["state"], "awake")
        self.assertIsNone(self.speaker._active_action)

    def test_failed_long_action_posts_one_error_completion(self):
        self.media.hold_wakeup = True
        queued = self.speaker.dispatch("wakeup", {})
        self.assertEqual(queued["state"], "queued")
        self.assertTrue(self.acp_event.wait(1))
        self.assertEqual(len(self.acp_notifications), 1)
        action_id, status, result, tool = self.acp_notifications[0]
        self.assertEqual((action_id, status, tool), (queued["action_id"], "error", "speaker"))
        self.assertEqual(result["stage"], "wakeup")
        active = self.speaker._active_action
        if active is not None:
            active["thread"].join(1)
        self.assertIsNone(self.speaker._active_action)

    def test_invalid_async_play_request_is_rejected_before_queueing(self):
        result = self.speaker.dispatch("play", {"input_topic": "  "})
        self.assertEqual(result["state"], "error")
        self.assertNotIn("action_id", result)
        self.assertIsNone(self.speaker._active_action)
        self.assertEqual(self.acp_notifications, [])

    def test_used_media_apis_exist_in_bundled_vendor_bindings(self):
        path = Path(__file__).resolve().parents[1] / "noetix_sdk_bumi/examples_py/mediacontrol_py.pyi"
        module = ast.parse(path.read_text(encoding="utf-8"))
        controller = next(node for node in module.body if isinstance(node, ast.ClassDef) and node.name == "MediaController")
        methods = {node.name for node in controller.body if isinstance(node, ast.FunctionDef)}
        expected = {
            "wakeup", "sleep", "restart", "pause_audio_playback", "resume_audio_playback",
            "resume_audio_capture", "get_system_status", "get_system_error",
            "get_volume", "set_volume", "publish_external_audio_playback_stream",
        }
        for route in self.media.ROUTES:
            expected.update({f"get_{route}_enable", f"set_{route}_enable"})
        self.assertEqual(expected - methods, set())

    def test_start_and_play_are_real_playback_and_preserve_pcm_contract(self):
        for action in ("start", "play"):
            with self.subTest(action=action):
                result = self.action(action, input_topic="/tts/audio")
                self.assertEqual(result["state"], "playing")
                self.assertEqual(result["action"], action)
                self.assertEqual(result["audio_mode"], "external_playback")
                self.assert_isolated()
                self.assertTrue(self.media.routes[self.speaker._EXTERNAL_PLAYBACK_ROUTE])
                self.speaker._sub.callback(self.chunk())
                stream = self.media.published[-1]
                self.assertEqual(stream.audio_data, [1, 1, -2, -2])
                self.assertEqual((stream.channels, stream.sample_rate, stream.format), (2, 16000, 2))
                self.assertEqual(self.speaker._frames_submitted, 1)
        self.assertNotIn("wakeup", self.names())
        self.assert_config_gap()

    def test_start_sleeps_awake_agent_before_opening_external_route(self):
        self.media.work = "WAKEUPED"
        result = self.action("start", input_topic="/audio")
        self.assertEqual(result["state"], "playing")
        sleep_index = self.names().index("sleep")
        enable_index = self.media.calls.index(("set_external_custom_audio_data_to_playback_enable", (True,)))
        self.assertLess(sleep_index, enable_index)
        self.assertEqual(self.media.work, "SLEEPED")
        self.assertNotIn("restart", self.names())

    def test_invalid_topic_has_no_side_effects_even_in_agent_mode(self):
        self.action("wakeup")
        self.media.calls.clear()
        result = self.action("play", input_topic="  ")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["audio_mode"], "vendor_agent")
        self.assertEqual(self.media.calls, [])

    def test_wakeup_stops_pcm_and_preserves_agent_on_both_stop_paths(self):
        self.action("play", input_topic="/audio")
        old_callback = self.speaker._sub.callback
        self.media.calls.clear()
        self.media.timed_calls.clear()
        self.media.config_calls.clear()
        result = self.action("wakeup")
        self.assertEqual(result["audio_mode"], "vendor_agent")
        self.assertEqual(result["state"], "awake")
        self.assert_no_stream()
        self.assertTrue(self.media.routes["internal_capture_audio_data_to_agent"])
        self.assertTrue(self.media.routes["internal_agent_audio_data_to_playback"])
        self.assertFalse(self.media.routes["external_custom_audio_data_to_agent"])
        self.assertNotIn("pause_audio_playback", self.names())
        wakeup_index = self.names().index("wakeup")
        pre_wakeup_sets = [
            call for call in self.media.calls[:wakeup_index]
            if call[0].startswith("set_")
        ]
        self.assertEqual(pre_wakeup_sets, [
            ("set_internal_capture_audio_data_to_agent_enable", (True,)),
            ("set_internal_agent_audio_data_to_playback_enable", (True,)),
        ])
        external_off = ("set_external_custom_audio_data_to_playback_enable", (False,))
        self.assertGreater(self.media.calls.index(external_off), wakeup_index)
        self.assertEqual(self.media.calls.count(external_off), 1)
        prerequisite_end = self.media.config_calls[1][2]
        resume_capture_time = next(
            timestamp for name, timestamp in self.media.timed_calls
            if name == "resume_audio_capture"
        )
        self.assertGreaterEqual(resume_capture_time - prerequisite_end + 1e-9, 0.8)
        resume_playback_time = next(
            timestamp for name, timestamp in self.media.timed_calls
            if name == "resume_audio_playback"
        )
        wakeup_time = next(
            timestamp for name, timestamp in self.media.timed_calls if name == "wakeup"
        )
        self.assertGreaterEqual(wakeup_time - resume_playback_time + 1e-9, 0.8)
        old_callback(self.chunk())
        self.assertEqual(self.media.published, [])
        # The policy is a mode, not synonymous with the SDK's work_status.
        self.media.work = "SLEEPED"
        self.media.calls.clear()
        stopped = self.action("stop")
        self.speaker.stop()
        self.assertEqual(stopped["audio_mode"], "vendor_agent")
        self.assertTrue(stopped["unchanged"])
        self.assertEqual(self.media.calls, [])

    def test_idle_wakeup_prepares_agent_before_state_transition(self):
        result = self.action("wakeup")
        self.assertEqual(result["state"], "awake")
        names = self.names()
        wakeup_index = names.index("wakeup")
        self.assertNotIn("pause_audio_playback", names)
        self.assertEqual(
            [name for name in names[:wakeup_index] if name.startswith("set_")],
            [
                "set_internal_capture_audio_data_to_agent_enable",
                "set_internal_agent_audio_data_to_playback_enable",
            ],
        )
        self.assertEqual(
            [name for name in names[wakeup_index + 1:] if name.startswith("set_")],
            [
                "set_external_custom_audio_data_to_agent_enable",
                "set_external_custom_audio_data_to_playback_enable",
            ],
        )
        expected_routes = {
            "set_internal_capture_audio_data_to_agent_enable",
            "set_external_custom_audio_data_to_agent_enable",
            "set_internal_agent_audio_data_to_playback_enable",
            "set_external_custom_audio_data_to_playback_enable",
        }
        self.assertEqual({name for name, _, _ in self.media.config_calls}, expected_routes)
        prerequisite_end = self.media.config_calls[1][2]
        resume_capture_time = next(
            timestamp for name, timestamp in self.media.timed_calls
            if name == "resume_audio_capture"
        )
        self.assertGreaterEqual(resume_capture_time - prerequisite_end + 1e-9, 0.8)
        resume_playback_time = next(
            timestamp for name, timestamp in self.media.timed_calls
            if name == "resume_audio_playback"
        )
        wakeup_time = next(
            timestamp for name, timestamp in self.media.timed_calls
            if name == "wakeup"
        )
        self.assertGreaterEqual(wakeup_time - resume_playback_time + 1e-9, 0.8)
        self.assert_config_gap()

    def test_wakeup_transition_failure_stops_before_external_isolation(self):
        self.media.hold_wakeup = True
        result = self.action("wakeup")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["stage"], "wakeup")
        self.assertEqual(result["desired_routes"], {
            "internal_capture_audio_data_to_agent": True,
            "internal_agent_audio_data_to_playback": True,
        })
        self.assertEqual(self.names().count("wakeup"), 1)
        self.assertEqual(
            [call for call in self.media.calls if call[0].startswith("set_")],
            [
                ("set_internal_capture_audio_data_to_agent_enable", (True,)),
                ("set_internal_agent_audio_data_to_playback_enable", (True,)),
            ],
        )
        self.assertIn("resume_audio_capture", self.names())
        self.assertIn("resume_audio_playback", self.names())
        for forbidden in ("pause_audio_playback", "sleep", "restart", "set_external_custom_audio_data_to_playback_enable"):
            self.assertNotIn(forbidden, self.names())

    def test_error_sleeped_wakeup_requires_explicit_reset(self):
        self.media.work, self.media.reason = "SLEEPED", "ERROR_SLEEPED"
        result = self.action("wakeup")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["stage"], "media_ready")
        self.assertEqual(result["desired_routes"], {})
        for forbidden in ("wakeup", "sleep", "restart", "pause_audio_playback"):
            self.assertNotIn(forbidden, self.names())
        self.assertFalse(any(name.startswith("set_") for name in self.names()))

    def test_wakeup_when_already_awake_does_not_send_duplicate_wakeup(self):
        self.media.work = "WAKEUPED"
        self.assertEqual(self.action("wakeup")["state"], "awake")
        self.assertNotIn("wakeup", self.names())

    def test_play_switches_back_from_agent_to_external_pcm(self):
        self.action("wakeup")
        result = self.action("play", input_topic="/tts")
        self.assertEqual(result["audio_mode"], "external_playback")
        self.assert_isolated()
        self.assertIn("sleep", self.names())
        self.assertEqual(len(self.node.subscriptions), 1)

    def test_stop_external_playback_removes_subscription(self):
        self.action("start", input_topic="/tts")
        callback = self.speaker._sub.callback
        result = self.action("stop")
        self.assertEqual(result["state"], "idle")
        self.assert_no_stream()
        self.assertEqual(self.node.subscriptions, [])
        callback(self.chunk())
        self.assertEqual(self.media.published, [])

    def test_destroy_subscription_accepts_void_style_success(self):
        self.action("start", input_topic="/tts")
        self.node.destroy_success_result = None
        result = self.action("stop")
        self.assertEqual(result["state"], "idle")
        self.assertIsNone(self.speaker._sub)
        self.assertEqual(self.node.subscriptions, [])

    def test_explicit_destroy_failure_retains_handle(self):
        self.action("start", input_topic="/tts")
        subscription = self.speaker._sub
        self.node.destroy_failures = 1
        with self.assertRaisesRegex(RuntimeError, "could not destroy"):
            self.speaker._destroy_subscription()
        self.assertIs(self.speaker._sub, subscription)
        self.assertEqual(self.node.subscriptions, [subscription])

    def test_destroy_exception_retains_handle(self):
        self.action("start", input_topic="/tts")
        subscription = self.speaker._sub
        self.node.destroy_exceptions = 1
        with self.assertRaisesRegex(RuntimeError, "destruction failed"):
            self.speaker._destroy_subscription()
        self.assertIs(self.speaker._sub, subscription)
        self.assertEqual(self.node.subscriptions, [subscription])

    def test_wakeup_destroy_failure_best_effort_closes_external_route(self):
        self.action("start", input_topic="/tts")
        old_callback = self.speaker._sub.callback
        self.node.destroy_failures = 1
        result = self.action("wakeup")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["stage"], "stop_previous_playback")
        self.assertTrue(result["cleanup"]["external_route_closed"])
        self.assertTrue(result["cleanup"]["stale_callbacks_invalidated"])
        self.assertFalse(self.media.routes[self.speaker._EXTERNAL_PLAYBACK_ROUTE])
        self.assertTrue(any(
            step["step"] == "destroy_subscription"
            for step in result["errors"]
        ))
        route_step = next(
            step for step in result["steps"]
            if step["step"] == self.speaker._EXTERNAL_PLAYBACK_ROUTE
        )
        self.assertEqual(route_step["phase"], "failure_cleanup")
        old_callback(self.chunk())
        self.assertEqual(self.media.published, [])

    def test_wakeup_destroy_and_route_failures_are_both_reported(self):
        self.action("start", input_topic="/tts")
        self.node.destroy_exceptions = 1
        route = self.speaker._EXTERNAL_PLAYBACK_ROUTE
        self.media.failures[f"set_{route}_enable"] = -1
        result = self.action("wakeup")
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["cleanup"]["external_route_closed"])
        self.assertEqual(
            {step["step"] for step in result["errors"]},
            {"destroy_subscription", route},
        )
        self.assertTrue(self.media.routes[route])

    def test_sleep_preserves_external_subscription_and_playback(self):
        self.action("play", input_topic="/tts")
        subscription = self.speaker._sub
        self.media.calls.clear()
        result = self.action("sleep")
        self.assertEqual(result["state"], "playing")
        self.assertIs(self.speaker._sub, subscription)
        self.assertTrue(self.media.routes[self.speaker._EXTERNAL_PLAYBACK_ROUTE])
        for forbidden in ("pause_audio_capture", "pause_audio_playback", "set_external_custom_audio_data_to_playback_enable", "restart"):
            self.assertNotIn(forbidden, self.names())
        subscription.callback(self.chunk())
        self.assertEqual(len(self.media.published), 1)

    def test_sleep_exits_vendor_agent_mode_without_pausing_capture(self):
        self.action("wakeup")
        self.media.calls.clear()
        result = self.action("sleep")
        self.assertEqual(result["state"], "sleeping")
        self.assertEqual(result["audio_mode"], "idle")
        self.assert_isolated()
        self.assertNotIn("pause_audio_capture", self.names())
        self.assertNotIn("pause_audio_playback", self.names())

    def test_reset_reasserts_routes_after_reset_defaults(self):
        self.action("wakeup")
        self.media.calls.clear()
        self.media.timed_calls.clear()
        self.media.config_calls.clear()
        result = self.action("reset")
        self.assertEqual(result["state"], "idle")
        self.assert_isolated()
        self.assert_no_stream()
        reset_index = self.names().index("restart")
        self.assertEqual(reset_index, 0)
        post_calls = self.media.calls[reset_index + 1:]
        for route in self.media.ROUTES:
            self.assertIn((f"set_{route}_enable", (False,)), post_calls)
        self.assertNotIn(("wakeup", ()), post_calls)
        self.assertNotIn("pause_audio_playback", self.names())
        self.assertNotIn("sleep", self.names())
        self.assert_config_gap()

    def test_error_sleeped_start_recovers_then_isolates(self):
        self.media.work, self.media.reason = "SLEEPED", "ERROR_SLEEPED"
        result = self.action("start", input_topic="/tts")
        self.assertEqual(result["state"], "playing")
        self.assertEqual(self.names().count("restart"), 1)
        self.assert_isolated()

    def test_error_reason_takes_priority_over_exit_transition(self):
        self.media.work, self.media.reason = "EXIT", "ERROR_SLEEPED"
        self.assertEqual(self.action("play", input_topic="/tts")["state"], "playing")
        self.assertEqual(self.names().count("restart"), 1)

    def test_missing_reset_ack_fails_even_if_status_looks_healthy(self):
        self.media.reset_ack = False
        result = self.action("reset")
        self.assertEqual(result["state"], "error")
        self.assertTrue(any(s["step"] == "wait_reset" for s in result["errors"]))
        self.assertFalse(self.speaker._playing)
        self.assertIsNone(self.speaker._sub)
        restart_index = self.names().index("restart")
        self.assertFalse(any(name.startswith("set_") for name in self.names()[restart_index + 1:]))

    def test_reset_route_failure_is_reported_without_secondary_cleanup(self):
        self.media.failures["set_internal_capture_audio_data_to_agent_enable"] = 2
        result = self.action("reset")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["stage"], "post_reset_isolation")
        self.assertIn("restart", self.names())
        self.assertNotIn("pause_audio_playback", self.names())
        self.assertNotIn("sleep", self.names())
        self.assertEqual(self.names().count("restart"), 1)

    def test_failed_restart_reports_first_failure_without_route_cleanup(self):
        self.media.failures["restart"] = 1
        result = self.action("reset")
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.names().count("restart"), 1)
        self.assertFalse(self.speaker._playing)
        self.assertIsNone(self.speaker._sub)
        self.assertFalse(any(name.startswith("set_") for name in self.names()))
        self.assertNotIn("pause_audio_playback", self.names())
        self.assertNotIn("sleep", self.names())

    def test_reset_timeout_reports_pending_without_writing_routes(self):
        self.media.hold_reset = True
        result = self.action("reset")
        self.assertEqual(result["state"], "resetting")
        self.assertEqual(result["stage"], "pending")
        self.assertTrue(result["pending"])
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["system_status"], {"work_status": "EXIT", "reason": "CMD_RESET"})
        self.assertFalse(any(name.startswith("set_") for name in self.names()))
        self.assertNotIn("pause_audio_playback", self.names())
        self.assertNotIn("sleep", self.names())

    def test_config_failure_retries_once_with_full_gap(self):
        self.media.failures["set_internal_capture_audio_data_to_agent_enable"] = 1
        result = self.action("start", input_topic="/audio")
        self.assertEqual(result["state"], "playing")
        step = next(s for s in result["steps"] if s["step"] == "internal_capture_audio_data_to_agent")
        self.assertEqual(step["attempts"], 2)
        self.assertEqual(len(step["attempt_errors"]), 1)
        self.assert_config_gap()

    def test_permanent_route_failure_never_subscribes_or_skips_other_cleanup(self):
        self.media.failures["set_internal_capture_audio_data_to_agent_enable"] = -1
        self.media.work = "WAKEUPED"
        result = self.action("play", input_topic="/audio")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["audio_mode"], "unknown")
        self.assert_no_stream()
        self.assertFalse(self.media.routes["external_custom_audio_data_to_agent"])
        self.assertFalse(self.media.routes["internal_agent_audio_data_to_playback"])
        self.assertIn("sleep", self.names())
        self.assertNotIn("restart", self.names())
        self.assertTrue(result["errors"])
        self.assertFalse(result["cleanup"]["agent_isolation_submitted"])
        self.assertTrue(any(step.get("phase") == "failure_cleanup" for step in result["errors"]))
        self.assert_config_gap()

    def test_sleep_timeout_fails_without_restarting_or_subscribing(self):
        self.media.work = "WAKEUPED"
        self.media.hold_sleep = True
        result = self.action("start", input_topic="/audio")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["audio_mode"], "unknown")
        self.assert_no_stream()
        self.assertNotIn("restart", self.names())
        self.assertEqual(self.names().count("sleep"), 1)

    def test_unreadable_status_fails_and_best_effort_closes_routes(self):
        self.media.failures["get_system_status"] = -1
        result = self.action("play", input_topic="/audio")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["audio_mode"], "unknown")
        self.assert_no_stream()
        self.assert_isolated()

    def test_unknown_status_fails(self):
        self.media.work = "UNRECOGNIZED"
        result = self.action("play", input_topic="/audio")
        self.assertEqual(result["state"], "error")
        self.assert_no_stream()

    def test_subscription_and_resume_failures_clean_up_routes(self):
        for failure in ("subscription", "resume_audio_playback"):
            with self.subTest(failure=failure):
                self.node.create_error = failure == "subscription"
                self.media.failures["resume_audio_playback"] = int(failure == "resume_audio_playback")
                result = self.action("play", input_topic="/audio")
                self.assertEqual(result["state"], "error")
                self.assert_no_stream()
                self.assert_isolated()

    def test_wakeup_failure_detaches_old_stream_without_post_failure_cleanup(self):
        self.action("play", input_topic="/old")
        old_callback = self.speaker._sub.callback
        self.media.calls.clear()
        self.media.timed_calls.clear()
        self.media.config_calls.clear()
        self.media.hold_wakeup = True
        result = self.action("wakeup")
        self.assertEqual(result["state"], "error")
        self.assertFalse(self.speaker._playing)
        self.assertIsNone(self.speaker._sub)
        self.assertTrue(self.media.routes[self.speaker._EXTERNAL_PLAYBACK_ROUTE])
        self.assertTrue(self.media.routes["internal_capture_audio_data_to_agent"])
        self.assertTrue(self.media.routes["internal_agent_audio_data_to_playback"])
        self.assertEqual(self.names().count("wakeup"), 1)
        self.assertEqual(
            [call for call in self.media.calls if call[0].startswith("set_")],
            [
                ("set_internal_capture_audio_data_to_agent_enable", (True,)),
                ("set_internal_agent_audio_data_to_playback_enable", (True,)),
            ],
        )
        self.assertNotIn("sleep", self.names())
        self.assertNotIn("pause_audio_playback", self.names())
        old_callback(self.chunk())
        self.assertEqual(self.media.published, [])

    def test_sleep_error_does_not_automatically_restart_or_stop_pcm(self):
        self.action("start", input_topic="/audio")
        subscription = self.speaker._sub
        self.media.work, self.media.reason = "SLEEPED", "ERROR_SLEEPED"
        result = self.action("sleep")
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["audio_mode"], "unknown")
        self.assertIs(self.speaker._sub, subscription)
        self.assertNotIn("restart", self.names())
        # Stop only stops PCM; it must not claim Agent isolation is known.
        self.assertEqual(self.action("stop")["audio_mode"], "unknown")

    def test_stale_getters_cannot_skip_route_writes(self):
        self.media.stale_getters = True
        result = self.action("start", input_topic="/audio")
        self.assertEqual(result["state"], "playing")
        self.assert_isolated()
        info = self.action("info")
        route = self.speaker._EXTERNAL_PLAYBACK_ROUTE
        self.assertTrue(info["desired_routes"][route])
        self.assertFalse(info["route_readback"][route])
        self.assertIn("not a per-command acknowledgement", info["route_confirmation"])

    def test_old_callback_cannot_publish_after_replacing_topic(self):
        self.action("start", input_topic="/old")
        old = self.speaker._sub.callback
        self.action("play", input_topic="/new")
        old(self.chunk())
        self.assertEqual(self.media.published, [])
        self.speaker._sub.callback(self.chunk())
        self.assertEqual(len(self.media.published), 1)

    def test_destroy_failure_is_retried_by_cleanup_without_replacing_stream(self):
        self.action("play", input_topic="/old")
        old = self.speaker._sub.callback
        self.node.destroy_failures = 1
        result = self.action("play", input_topic="/new")
        self.assertEqual(result["state"], "error")
        self.assert_no_stream()
        self.assertEqual(self.node.subscriptions, [])
        old(self.chunk())
        self.assertEqual(self.media.published, [])

    def test_bad_audio_reports_error_without_publishing_and_alias_still_works(self):
        self.action("play", input_topic="/audio")
        callback = self.speaker._sub.callback
        for msg in (self.chunk(data=b"\x01"), self.chunk(fmt="mp3"), self.chunk(data=b"")):
            callback(msg)
        self.assertEqual(self.media.published, [])
        self.assertIsNotNone(self.action("info")["last_error"])
        callback(self.chunk(fmt="pcm_16k_16bit_mono"))
        self.assertEqual(len(self.media.published), 1)

    def test_volume_error_obeys_gap_and_keeps_playback(self):
        self.action("play", input_topic="/audio")
        self.media.failures["set_volume"] = 1
        self.assertEqual(self.action("set_volume", volume=120)["state"], "error")
        self.assertEqual(self.action("set_volume", volume=120)["state"], "set")
        self.assertTrue(self.speaker._playing)
        self.assert_config_gap()
        self.assertEqual(self.action("get_volume"), {"volume": 120})
        count = len(self.media.calls)
        self.assertEqual(self.action("set_volume", volume=201)["state"], "error")
        self.assertEqual(len(self.media.calls), count)

    def test_info_reports_error_details_and_mode_independently_of_agent_status(self):
        self.action("wakeup")
        self.media.work, self.media.reason = "SLEEPED", "CMD_SLEEPED"
        info = self.action("info")
        self.assertEqual(info["audio_mode"], "vendor_agent")
        self.assertEqual(info["system_status"]["work_status"], "SLEEPED")
        self.assertEqual(info["last_control"]["action"], "wakeup")
        self.media.failures["get_volume"] = -1
        self.assertIn("error", self.action("info")["volume"])
        self.assertEqual(self.action("get_volume")["state"], "error")

    def test_controls_serialize_but_config_wait_does_not_block_pcm(self):
        self.action("play", input_topic="/audio")
        entered, release, volume_started, volume_done = (threading.Event() for _ in range(4))
        results = {}

        def block_first_set(name, enabled):
            if not entered.is_set():
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test gate timed out")

        def volume():
            volume_started.set()
            results["volume"] = self.action("set_volume", volume=123)
            volume_done.set()

        self.media.on_set = block_first_set
        sleep_thread = threading.Thread(target=lambda: results.update(sleep=self.action("sleep")))
        volume_thread = threading.Thread(target=volume)
        try:
            sleep_thread.start()
            self.assertTrue(entered.wait(1))
            volume_thread.start()
            self.assertTrue(volume_started.wait(1))
            self.assertFalse(volume_done.wait(0.05))
            self.speaker._sub.callback(self.chunk())
            self.assertEqual(len(self.media.published), 1)
        finally:
            release.set()
            sleep_thread.join(2)
            if volume_thread.ident is not None:
                volume_thread.join(2)
        self.assertFalse(sleep_thread.is_alive())
        self.assertFalse(volume_thread.is_alive())
        self.assertEqual(results["sleep"]["state"], "playing")
        self.assertEqual(results["volume"]["state"], "set")
        self.assert_config_gap()


if __name__ == "__main__":
    unittest.main()
