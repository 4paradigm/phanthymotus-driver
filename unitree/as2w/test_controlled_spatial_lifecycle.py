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
        self.instances.append(self)

    def call(self, action, data):
        if self.stops:
            raise AssertionError("RPC sent to a stopped worker")
        self.calls.append((action, data))
        if self.on_call:
            self.on_call(action)
        return {"code": 0, "response": "{}"}

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
        self.addCleanup(self.plugin.stop)

    def test_stop_start_navigation_uses_new_worker_and_receives_completion(self):
        first_client, first_subscriber = self.plugin._client, self.plugin._nav_sub
        self.plugin.dispatch("stop", {})
        self.assertEqual("idle", self.plugin.dispatch("info", {})["state"])
        self.assertFalse(self.plugin.dispatch("navigate_to", {"x": 1})["accepted"])
        started = self.plugin.dispatch("start", {})
        self.assertEqual({"state": "ready", "completion_available": True}, started)
        self.assertIsNot(first_client, self.plugin._client)
        self.assertIsNot(first_subscriber, self.plugin._nav_sub)
        self.assertEqual("eth-test", self.plugin._client.interface)
        self.plugin._client.on_call = lambda action: self.plugin._nav_sub.arrive() if action == "navigate_to" else None
        result = self.plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        notification = self.notifications.get(timeout=1)
        self.assertEqual((result["action_id"], "completed"), notification[:2])
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
            self.assertEqual((current["action_id"], "completed"), self.notifications.get(timeout=1)[:2])
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


class ProxyStopTests(unittest.TestCase):
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
