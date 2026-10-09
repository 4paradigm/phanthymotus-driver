"""Vendor-navigation lifecycle regressions with fake DDS and RPC transports."""
import importlib.util
import queue
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "as2w_spatial_lifecycle", Path(__file__).with_name("controlled_spatial.py"))
spatial = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(spatial)


class FakeRpc:
    instances = []

    def __init__(self, interface):
        self.interface = interface
        self.calls = []
        self.stops = 0
        self.on_call = None
        self.codes = {}
        self.instances.append(self)

    def call(self, action, data):
        if self.stops:
            raise AssertionError("RPC sent to a stopped worker")
        self.calls.append((action, data))
        if self.on_call:
            self.on_call(action)
        return {"code": self.codes.get(action, 0), "response": "{}"}

    def stop(self):
        self.stops += 1


class FakeSubscriber:
    instances = []
    fail_init = False

    def __init__(self, topic, message_type):
        self.topic = topic
        self.callback = None
        self.closes = 0
        self.instances.append(self)

    def Init(self, callback, queue_depth):
        self.callback = callback
        if self.fail_init:
            raise RuntimeError("DDS subscription temporarily unavailable")

    def Close(self):
        self.closes += 1

    def arrive(self):
        # Retain the callback after Close to model an already queued DDS event.
        self.callback(types.SimpleNamespace(
            data='{"type":"task_result","errorCode":0,"data":{"is_arrived":true}}'))


class FakeGuard:
    def __init__(self):
        self.reserved = False
        self.releases = 0

    def reserve_external_navigation(self):
        self.reserved = True
        return True

    def release_external_navigation(self):
        self.reserved = False
        self.releases += 1


class SpatialLifecycleTests(unittest.TestCase):
    def setUp(self):
        FakeRpc.instances = []
        FakeSubscriber.instances = []
        FakeSubscriber.fail_init = False
        channels = types.ModuleType("unitree_sdk2py.core.channel")
        channels.ChannelSubscriber = FakeSubscriber
        messages = types.ModuleType("unitree_sdk2py.idl.std_msgs.msg.dds_")
        messages.String_ = object
        patcher = patch.dict(sys.modules, {
            channels.__name__: channels, messages.__name__: messages})
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = patch.object(spatial, "_SpatialRpcProxy", FakeRpc)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.notifications = queue.Queue()
        patcher = patch.object(spatial, "_acp_notify", lambda *args: self.notifications.put(args))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.plugin = spatial.ControlledSpatialPlugin({}, "test", None, "eth-test")
        self.guard = FakeGuard()
        self.plugin.set_chassis_guard(self.guard)
        self.addCleanup(self.plugin.stop)

    def test_stop_start_navigation_uses_new_worker_and_receives_completion(self):
        first_client, first_subscriber = self.plugin._client, self.plugin._nav_sub
        self.plugin.dispatch("stop", {})
        self.assertEqual("idle", self.plugin.dispatch("info", {})["state"])
        self.assertFalse(self.plugin.dispatch("navigate_to", {"x": 1})["accepted"])
        started = self.plugin.dispatch("start", {})
        self.assertEqual("ready", started["state"])
        self.assertTrue(started["completion_available"])
        self.assertIsNot(first_client, self.plugin._client)
        self.assertIsNot(first_subscriber, self.plugin._nav_sub)
        self.assertEqual("eth-test", self.plugin._client.interface)
        self.plugin._client.on_call = lambda action: self.plugin._nav_sub.arrive() if action == "navigate_to" else None
        result = self.plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        notification = self.notifications.get(timeout=1)
        self.assertEqual((result["action_id"], "error"), notification[:2])
        self.assertTrue(notification[2]["arrival_reported"])
        self.assertFalse(notification[2]["arrival_verified"])
        self.assertTrue(notification[2]["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)
        self.assertEqual([], first_client.calls)
        self.assertEqual(1, first_client.stops)
        self.assertEqual(1, first_subscriber.closes)

    def test_repeated_start_and_stop_do_not_duplicate_or_reclose_resources(self):
        first_client, first_subscriber = self.plugin._client, self.plugin._nav_sub
        self.plugin.start()
        self.plugin.dispatch("start", {})
        self.assertEqual(1, len(FakeRpc.instances))
        self.assertEqual(1, len(FakeSubscriber.instances))
        self.plugin.stop()
        self.plugin.dispatch("stop", {})
        self.assertEqual(1, first_client.stops)
        self.assertEqual(1, first_subscriber.closes)
        self.plugin.start()
        self.plugin.start()
        self.assertEqual(2, len(FakeRpc.instances))
        self.assertEqual(2, len(FakeSubscriber.instances))

    def test_stop_wakes_cancelled_waiter_and_old_callback_cannot_complete_new_goal(self):
        threads = []
        actual_thread = threading.Thread

        def create_thread(*args, **kwargs):
            thread = actual_thread(*args, **kwargs)
            threads.append(thread)
            return thread

        old_subscriber = self.plugin._nav_sub
        with patch.object(spatial.threading, "Thread", create_thread):
            previous = self.plugin.dispatch("navigate_to", {"x": 1})
            self.plugin.stop()
            self.assertEqual((previous["action_id"], "cancelled"), self.notifications.get(timeout=1)[:2])
            threads[0].join(timeout=1)
            self.assertFalse(threads[0].is_alive(), "stopped goal must not leave a 180-second waiter")
            self.plugin.start()
            current = self.plugin.dispatch("navigate_to", {"x": 2})
            old_subscriber.arrive()
            self.assertFalse(self.plugin._nav_done.is_set())
            self.assertTrue(self.notifications.empty())
            self.plugin._nav_sub.arrive()
            self.assertEqual((current["action_id"], "error"), self.notifications.get(timeout=1)[:2])
            threads[1].join(timeout=1)
            self.assertFalse(threads[1].is_alive())

    def test_failed_subscription_can_retry_without_replacing_live_rpc(self):
        self.plugin.stop()
        FakeSubscriber.fail_init = True
        self.assertFalse(self.plugin.start()["completion_available"])
        client = self.plugin._client
        failed_subscriber = FakeSubscriber.instances[-1]
        self.assertEqual(1, failed_subscriber.closes)
        self.assertFalse(self.plugin.dispatch("navigate_to", {"x": 1})["accepted"])
        self.assertEqual([], client.calls)
        FakeSubscriber.fail_init = False
        self.assertTrue(self.plugin.start()["completion_available"])
        self.assertIs(client, self.plugin._client)
        self.assertEqual(2, len(FakeRpc.instances))

    def test_terminal_result_keeps_reservation_until_pause_ack_then_card_stop_retries(self):
        client = self.plugin._client
        client.codes["pause_navigation"] = 3104
        client.on_call = lambda action: self.plugin._nav_sub.arrive() if action == "navigate_to" else None
        action = self.plugin.dispatch("navigate_to", {"x": 1})
        result = self.notifications.get(timeout=1)
        self.assertEqual((action["action_id"], "error"), result[:2])
        self.assertFalse(result[2]["stop_acknowledged"])
        self.assertTrue(result[2]["chassis_reserved"])
        self.assertTrue(self.guard.reserved)
        self.assertIsNone(self.plugin._nav_action_id)
        self.assertTrue(self.plugin.dispatch("info", {})["requires_vendor_pause"])
        client.codes["pause_navigation"] = 0
        stopped = self.plugin.stop()
        self.assertTrue(stopped["ok"])
        self.assertTrue(stopped["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)
        self.assertEqual("pause_navigation", client.calls[-1][0])

    def test_failed_navigation_rpc_without_action_id_is_still_stopped_on_teardown(self):
        client = self.plugin._client
        client.codes["navigate_to"] = 3104
        result = self.plugin.dispatch("navigate_to", {"x": 1})
        self.assertEqual(3104, result["ret"])
        self.assertTrue(result["chassis_reserved"])
        self.assertIsNone(self.plugin._nav_action_id)
        self.assertTrue(self.plugin.stop()["ok"])
        self.assertEqual(["navigate_to", "pause_navigation"], [x[0] for x in client.calls])
        self.assertFalse(self.guard.reserved)

    def test_failed_stop_preserves_reservation_across_restart_until_ack(self):
        self.plugin._client.codes["navigate_to"] = 3104
        self.plugin.dispatch("navigate_to", {"x": 1})
        self.plugin._client.codes["pause_navigation"] = 3104
        result = self.plugin.stop()
        self.assertFalse(result["ok"])
        self.assertEqual("error", result["state"])
        self.assertTrue(result["chassis_reserved"])
        self.assertIsNone(self.plugin._client)
        self.assertTrue(self.plugin.start()["chassis_reserved"])
        acknowledged = self.plugin.dispatch("pause_navigation", {})
        self.assertTrue(acknowledged["stop_acknowledged"])
        self.assertFalse(acknowledged["chassis_reserved"])
        self.assertFalse(self.guard.reserved)

    def test_repeated_stop_can_recreate_closed_rpc_to_clear_retained_reservation(self):
        self.plugin._client.codes["navigate_to"] = 3104
        self.plugin.dispatch("navigate_to", {"x": 1})
        client = self.plugin._client
        client.codes["pause_navigation"] = 3104
        self.assertFalse(self.plugin.stop()["ok"])
        retried = self.plugin.stop()
        self.assertTrue(retried["ok"])
        self.assertEqual(2, len(FakeRpc.instances))
        self.assertEqual([("pause_navigation", {})], FakeRpc.instances[-1].calls)
        self.assertFalse(self.guard.reserved)

    def test_bool_or_noninteger_pause_reply_cannot_release_chassis(self):
        self.plugin._client.codes["navigate_to"] = 3104
        self.plugin.dispatch("navigate_to", {"x": 1})
        for code in (False, True, None, "0", 0.0):
            with self.subTest(code=code):
                self.plugin._client.codes["pause_navigation"] = code
                result = self.plugin.dispatch("pause_navigation", {})
                self.assertFalse(result["stop_acknowledged"])
                self.assertTrue(self.guard.reserved)
        self.assertTrue(self.plugin.dispatch("shutdown", {})["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)

    def test_untracked_resume_is_rejected_without_reservation_or_rpc(self):
        result = self.plugin.dispatch("resume_navigation", {})
        self.assertFalse(result["accepted"])
        self.assertFalse(self.guard.reserved)
        self.assertEqual([], self.plugin._client.calls)

    def test_resume_creates_new_supervised_action_for_explicitly_paused_target(self):
        first = self.plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        self.plugin.dispatch("pause_navigation", {})
        self.assertEqual((first["action_id"], "cancelled"), self.notifications.get(timeout=1)[:2])
        self.assertTrue(self.plugin.dispatch("info", {})["can_resume"])
        self.plugin._client.on_call = lambda action: self.plugin._nav_sub.arrive() if action == "resume_navigation" else None
        resumed = self.plugin.dispatch("resume_navigation", {})
        self.assertNotEqual(first["action_id"], resumed["action_id"])
        self.assertEqual(first["target_pose"], resumed["target_pose"])
        self.assertFalse(self.plugin.dispatch("info", {})["can_resume"])
        terminal = self.notifications.get(timeout=1)
        self.assertEqual((resumed["action_id"], "error"), terminal[:2])
        self.assertFalse(terminal[2]["arrival_verified"])
        self.assertTrue(terminal[2]["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)
        self.assertIn("resume_navigation", self.plugin.get_tool()["inputSchema"]["x-completion"]["actions"])

    def test_resumed_goal_timeout_retains_reservation_until_stop_ack(self):
        with patch.object(spatial.threading, "Thread"):
            self.plugin.dispatch("navigate_to", {"x": 3})
            self.plugin.dispatch("pause_navigation", {})
            self.notifications.get(timeout=1)
            resumed = self.plugin.dispatch("resume_navigation", {})
        self.plugin._client.codes["pause_navigation"] = 3104
        self.plugin._wait_for_navigation(resumed["action_id"], resumed["target_pose"],
                                         types.SimpleNamespace(wait=lambda timeout: False))
        terminal = self.notifications.get(timeout=1)
        self.assertEqual((resumed["action_id"], "error"), terminal[:2])
        self.assertFalse(terminal[2]["stop_acknowledged"])
        self.assertTrue(self.guard.reserved)
        self.plugin._client.codes["pause_navigation"] = 0
        self.assertTrue(self.plugin.dispatch("pause_navigation", {})["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)

    def test_resume_requires_completion_subscription_and_preserves_paused_target(self):
        self.plugin.dispatch("navigate_to", {"x": 4})
        self.plugin.dispatch("pause_navigation", {})
        self.notifications.get(timeout=1)
        self.plugin._nav_sub.Close()
        self.plugin._nav_sub = None
        calls = list(self.plugin._client.calls)
        self.assertFalse(self.plugin.dispatch("resume_navigation", {})["accepted"])
        self.assertTrue(self.plugin.dispatch("info", {})["can_resume"])
        self.assertFalse(self.guard.reserved)
        self.assertEqual(calls, self.plugin._client.calls)
        self.plugin.start()
        self.assertEqual(0, self.plugin.dispatch("resume_navigation", {})["ret"])

    def test_timeout_pauses_vendor_and_releases_only_after_ack(self):
        with patch.object(spatial.threading, "Thread"):
            action = self.plugin.dispatch("navigate_to", {"x": 1})
        self.plugin._wait_for_navigation(action["action_id"], {"x": 1},
                                         types.SimpleNamespace(wait=lambda timeout: False))
        result = self.notifications.get(timeout=1)
        self.assertEqual((action["action_id"], "error"), result[:2])
        self.assertTrue(result[2]["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)
        self.assertIn("timed out", result[2]["error"])

    def test_explicit_pause_cancels_waiter_and_late_result_cannot_complete_next_goal(self):
        first = self.plugin.dispatch("navigate_to", {"x": 1})
        old_subscriber = self.plugin._nav_sub
        self.plugin.dispatch("pause_navigation", {})
        self.assertEqual((first["action_id"], "cancelled"), self.notifications.get(timeout=1)[:2])
        second = self.plugin.dispatch("navigate_to", {"x": 2})
        # DDS carries no task identity, so even a current subscription may
        # deliver the previous goal's terminal event. Never report completion.
        old_subscriber.arrive()
        result = self.notifications.get(timeout=1)
        self.assertEqual((second["action_id"], "error"), result[:2])
        self.assertEqual("uncorrelated_vendor_task_result", result[2]["reason"])
        self.assertFalse(result[2]["arrival_verified"])
        self.assertTrue(result[2]["stop_acknowledged"])
        self.assertFalse(self.guard.reserved)


class ProxyStopTests(unittest.TestCase):
    def proxy(self):
        proxy = spatial._SpatialRpcProxy.__new__(spatial._SpatialRpcProxy)
        proxy._lock = threading.Lock()
        proxy._startup_error = None
        proxy._stopped = False
        proxy._next_request_id = 1
        proxy._commands, proxy._results = queue.Queue(), queue.Queue()
        return proxy

    def test_old_navigation_reply_is_not_a_pause_acknowledgement(self):
        proxy = self.proxy()
        proxy._results.put({"request_id": 1, "code": 0, "response": "old navigation"})
        proxy._results.put({"request_id": 2, "code": 3104, "response": "pause failed"})
        result = proxy.call("pause_navigation", {})
        self.assertEqual(3104, result["code"])
        self.assertEqual(2, result["request_id"])
        self.assertEqual({"request_id": 2, "action": "pause_navigation", "data": {}},
                         proxy._commands.get_nowait())

    def test_ignoring_old_reply_does_not_extend_absolute_rpc_timeout(self):
        proxy = self.proxy()
        proxy._results.put({"request_id": 1, "code": 0})
        with patch.object(spatial.time, "monotonic", side_effect=[100, 100, 120]):
            result = proxy.call("pause_navigation", {})
        self.assertEqual(3104, result["code"])
        self.assertEqual("SLAM service timeout", result["response"])

    def test_worker_echoes_request_identity_for_vendor_result(self):
        channel = types.ModuleType("unitree_sdk2py.core.channel")
        channel.ChannelFactoryInitialize = lambda domain, interface: None
        commands, results = queue.Queue(), queue.Queue()
        commands.put({"request_id": 37, "action": "pause_navigation", "data": {}})
        commands.put(None)
        with patch.dict(sys.modules, {channel.__name__: channel}), \
                patch.object(spatial, "_install_logsafe"), \
                patch.object(spatial, "_SlamClient", return_value=types.SimpleNamespace(call=lambda *_: (0, "{}"))):
            spatial._worker(commands, results, "eno1")
        self.assertEqual({"ready": True}, results.get_nowait())
        self.assertEqual({"request_id": 37, "code": 0, "response": "{}"}, results.get_nowait())

    def test_stuck_worker_is_terminated_and_subsequent_calls_are_rejected(self):
        proxy = spatial._SpatialRpcProxy.__new__(spatial._SpatialRpcProxy)
        proxy._lock = threading.Lock()
        proxy._startup_error = None
        proxy._stopped = False
        proxy._commands = queue.Queue()
        proxy._results = queue.Queue()
        joins, terminations = [], []
        proxy._process = types.SimpleNamespace(
            join=lambda timeout: joins.append(timeout),
            is_alive=lambda: True,
            terminate=lambda: terminations.append(True))
        proxy.stop()
        proxy.stop()
        self.assertEqual([3, 1], joins)
        self.assertEqual([True], terminations)
        self.assertIsNone(proxy._commands.get_nowait())
        self.assertEqual(3104, proxy.call("navigate_to", {})["code"])
        self.assertTrue(proxy._commands.empty())


if __name__ == "__main__":
    unittest.main()
