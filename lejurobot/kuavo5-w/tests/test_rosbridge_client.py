import importlib.util
import json
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("kuavo_rosbridge", ROOT / "rosbridge_client.py")
module = importlib.util.module_from_spec(spec)
# The offline suite must not require ROS or websocket-client, or pollute other tests.
with patch.dict(sys.modules, {"websocket": types.SimpleNamespace(WebSocketApp=object)}):
    spec.loader.exec_module(module)
RosbridgeClient = module.RosbridgeClient


class FakeSocket:
    def __init__(self):
        self.sent, self.ready, self.closed = [], threading.Event(), False
        self.fail = False

    def send(self, raw):
        if self.fail:
            raise ConnectionError("socket lost")
        self.sent.append(json.loads(raw))
        self.ready.set()

    def close(self):
        self.closed = True


class RosbridgeClientTests(unittest.TestCase):
    def setUp(self):
        self.client, self.sock = RosbridgeClient("ws://unused"), FakeSocket()
        self.client._ws = self.sock
        self.client._on_open(self.sock)

    def packet(self, value):
        self.client._on_message(self.sock, json.dumps(value))

    def call_async(self, timeout=1):
        results, errors = [], []
        def call():
            try:
                results.append(self.client.call_service("/enable_control", "std_srvs/SetBool", {"data": False}, timeout))
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=call)
        thread.start()
        self.assertTrue(self.sock.ready.wait(1))
        return thread, results, errors

    def test_publish_advertises_once(self):
        for _ in range(2):
            self.client.publish("/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 0.1}})
        self.assertEqual([p["op"] for p in self.sock.sent], ["advertise", "publish", "publish"])

    def test_get_param_decodes_json_and_sends_rosapi_default(self):
        with patch.object(self.client, 'call_service', return_value={'value': 'false'}) as service:
            self.assertIs(self.client.get_param('/use_shm_communication', False), False)
        service.assert_called_once_with('/rosapi/get_param', 'rosapi/GetParam',
                                        {'name': '/use_shm_communication', 'default': 'false'}, timeout=2.)
        with patch.object(self.client, 'call_service', return_value={'value': '"<robot/>"'}):
            self.assertEqual(self.client.get_param('/humanoid_description'), '<robot/>')
        with patch.object(self.client, 'call_service', return_value={'value': False}):
            with self.assertRaises(ValueError):
                self.client.get_param('/robot_type')

    def test_unsubscribe_is_not_restored_on_reconnect(self):
        self.client.subscribe('/old', 'std_msgs/String', lambda msg: None)
        self.client.unsubscribe('/old')
        self.assertEqual(self.sock.sent[-1], {'op': 'unsubscribe', 'id': 'sub:/old', 'topic': '/old'})
        self.client._on_close(self.sock, None, None)
        self.sock.sent.clear()
        self.client._on_open(self.sock)
        self.assertEqual(self.sock.sent, [])

    def test_subscription_receives_dict_and_ignores_malformed(self):
        received = []
        self.client.subscribe("/state", "std_msgs/Bool", received.append)
        self.packet({"op": "publish", "topic": "/state", "msg": {"data": True}})
        self.packet([])
        self.packet({"op": "publish", "topic": "/state", "msg": []})
        self.assertEqual(received, [{"data": True}])

    def test_native_utf8_decode_accepts_text_and_rejects_malformed_or_utf16(self):
        received = []
        self.client.subscribe('/state', 'std_msgs/String', received.append)
        packet = json.dumps({'op': 'publish', 'topic': '/state', 'msg': {'data': '实测'}}, ensure_ascii=False)
        self.client._on_message(self.sock, packet.encode('utf-8'))
        self.client._on_message(self.sock, packet.encode('utf-16'))
        self.client._on_message(self.sock, packet.encode('utf-8').replace('实测'.encode(), b'\xff'))
        self.assertEqual(received, [{'data': '实测'}])

    def test_transport_uses_native_decoder_without_slow_python_utf8_loop(self):
        calls = []
        client = self.client
        class App:
            def __init__(self, *args, **kwargs):
                pass
            def run_forever(self, **kwargs):
                calls.append(kwargs)
                client._closed.set()
        with patch.object(module.websocket, 'WebSocketApp', App):
            client._run()
        self.assertEqual(calls, [{'ping_interval': 10, 'ping_timeout': 5, 'skip_utf8_validation': True}])

    def test_actual_protocol_result_unblocks_service(self):
        thread, results, errors = self.call_async()
        request = self.sock.sent[0]
        self.assertNotIn("type", request)
        self.packet({"op": "service_response", "id": request["id"],
                     "service": "/enable_control", "result": True, "values": {"success": False, "message": "guard"}})
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        # Protocol success and vendor request acceptance are distinct.
        self.assertEqual(results, [{"success": False, "message": "guard"}])

    def test_protocol_failure_is_error(self):
        thread, results, errors = self.call_async()
        self.packet({"op": "service_response", "id": self.sock.sent[0]["id"],
                     "result": False, "values": "service not available"})
        thread.join(1)
        self.assertEqual(results, [])
        self.assertIsInstance(errors[0], RuntimeError)

    def test_disconnect_wakes_pending_service_and_cleans(self):
        thread, results, errors = self.call_async()
        self.client._on_close(self.sock, None, None)
        thread.join(0.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results, [])
        self.assertTrue(errors)
        self.assertFalse(self.client._services)

    def test_send_failure_cleans_pending(self):
        self.sock.fail = True
        with self.assertRaises(ConnectionError):
            self.client.call_service("/enable_control", "std_srvs/SetBool", {"data": False})
        self.assertFalse(self.client._services)
        self.assertFalse(self.client.connection[0])

    def test_timeout_does_not_retry_or_accept_late_response(self):
        thread, results, errors = self.call_async(0.03)
        thread.join(1)
        self.assertIsInstance(errors[0], TimeoutError)
        self.assertEqual(len(self.sock.sent), 1)
        self.assertFalse(self.client._services)
        self.packet({"op": "service_response", "id": self.sock.sent[0]["id"], "result": True, "values": {"success": True}})
        self.assertEqual(results, [])

    def test_reconnect_restores_subscriptions_and_ads_not_commands(self):
        self.client.subscribe("/state", "std_msgs/Bool", lambda msg: None)
        self.client.publish("/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 1}})
        old_generation = self.client.connection[1]
        self.client._on_close(self.sock, None, None)
        other = FakeSocket()
        self.client._ws = other
        self.client._on_open(other)
        self.assertEqual(self.client.connection[1], old_generation + 1)
        self.assertEqual([p["op"] for p in other.sent], ["advertise", "subscribe"])
        self.assertFalse(self.client.motion_ready)
        self.client._ready_at = 0
        self.assertTrue(self.client.motion_ready)

    def test_status_error_invalidates_connection(self):
        self.packet({"op": "status", "level": "error", "id": "pub:/cmd_vel", "msg": "bad message"})
        self.assertFalse(self.client.connection[0])
        self.assertTrue(self.sock.closed)

    def test_service_wait_can_be_superseded_without_replaying_command(self):
        pending = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        cancelled, errors = threading.Event(), []
        def wait():
            try:
                self.client.wait_service(pending, 2, cancelled)
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=wait)
        worker.start()
        cancelled.set()
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertIsInstance(errors[0], InterruptedError)
        self.assertEqual(len(self.sock.sent), 1)
        self.assertFalse(self.client._services)
        self.packet({'op': 'service_response', 'id': pending[0], 'result': True,
                     'values': {'success': True}})
        self.assertFalse(self.client._services)

    def test_pause_service_can_send_and_complete_while_older_service_is_pending(self):
        old = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        pause = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': False})
        self.assertEqual([p['args']['data'] for p in self.sock.sent], [True, False])
        self.packet({'op': 'service_response', 'id': pause[0], 'result': True, 'values': {'success': True}})
        self.assertEqual(self.client.wait_service(pause, .1), {'success': True})
        self.assertIn(old[0], self.client._services)
        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaises(InterruptedError):
            self.client.wait_service(old, .1, cancelled)
        self.assertFalse(self.client._services)

    def test_repeated_subscribe_uses_same_id_for_readonly_latch_refresh(self):
        self.client.subscribe('/enable_control_state', 'std_msgs/Bool', lambda msg: None)
        self.client.subscribe('/enable_control_state', 'std_msgs/Bool', lambda msg: None)
        self.assertEqual([p['op'] for p in self.sock.sent], ['subscribe', 'subscribe'])
        self.assertEqual(self.sock.sent[0]['id'], self.sock.sent[1]['id'])
        self.assertEqual(len(self.client._subscriptions), 1)

    def test_retained_resume_response_remains_observable_after_timeout(self):
        pending = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        self.client.retain_service(pending)
        with self.assertRaises(TimeoutError):
            self.client.wait_service(pending, 0.)
        self.assertIsNone(self.client.service_result(pending))
        self.assertEqual(len(self.client._services), 1)
        self.packet({'op': 'service_response', 'id': pending[0], 'result': True,
                     'values': {'success': True}})
        self.assertEqual(self.client.service_result(pending), {'success': True})
        self.assertFalse(self.client._services)
        self.assertFalse(self.client._retained_services)
        self.assertEqual(len(self.sock.sent), 1)  # Retention never resends enable.

    def test_retained_cancelled_resume_can_later_be_explicitly_rejected(self):
        pending = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        self.client.retain_service(pending)
        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaises(InterruptedError):
            self.client.wait_service(pending, .1, cancelled)
        self.assertIsNone(self.client.service_result(pending))
        self.packet({'op': 'service_response', 'id': pending[0], 'result': True,
                     'values': {'success': False}})
        self.assertEqual(self.client.service_result(pending), {'success': False})
        self.assertFalse(self.client._retained_services)

    def test_retained_transport_failure_is_not_controller_rejection(self):
        pending = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        self.client.retain_service(pending)
        self.client._on_close(self.sock, None, None)
        self.assertIsNone(self.client.service_result(pending))
        self.assertFalse(self.client._services)
        self.assertFalse(self.client._retained_services)

    def test_response_before_retention_does_not_leave_orphan_id(self):
        pending = self.client.start_service('/enable_control', 'std_srvs/SetBool', {'data': True})
        self.packet({'op': 'service_response', 'id': pending[0], 'result': True,
                     'values': {'success': False}})
        self.client.retain_service(pending)
        self.assertFalse(self.client._retained_services)
        self.assertEqual(self.client.service_result(pending), {'success': False})

    def test_callback_failures_are_sampled_and_recovery_is_logged_once(self):
        failures = [True]
        def callback(_msg):
            if failures[0]:
                raise ValueError('malformed\nframe')
        self.client.subscribe('/audit', 'std_msgs/String', callback)
        clock = [0.]
        with patch.object(module, 'time', types.SimpleNamespace(monotonic=lambda: clock[0])), patch('builtins.print') as log:
            for _ in range(100):
                self.packet({'op': 'publish', 'topic': '/audit', 'msg': {}})
            self.assertEqual(log.call_count, 1)
            clock[0] = 5.
            self.packet({'op': 'publish', 'topic': '/audit', 'msg': {}})
            self.assertEqual(log.call_count, 2)
            self.assertIn('count=101', log.call_args[0][0])
            self.assertIn('malformed\\nframe', log.call_args[0][0])
            failures[0] = False
            for _ in range(2):
                self.packet({'op': 'publish', 'topic': '/audit', 'msg': {}})
            self.assertEqual(log.call_count, 3)
            self.assertIn('recovered topic=/audit after 101 failures', log.call_args[0][0])
            failures[0] = True
            self.packet({'op': 'publish', 'topic': '/audit', 'msg': {}})
            self.assertEqual(log.call_count, 4)
            self.assertIn('count=1:', log.call_args[0][0])

    def test_callback_failure_sampling_is_per_topic_and_cleared_on_unsubscribe(self):
        def bad(_msg):
            raise TypeError('bad frame')
        for topic in ('/one', '/two'):
            self.client.subscribe(topic, 'std_msgs/String', bad)
        with patch('builtins.print') as log:
            for topic in ('/one', '/two', '/one', '/two'):
                self.packet({'op': 'publish', 'topic': topic, 'msg': {}})
        self.assertEqual(log.call_count, 2)
        self.client.unsubscribe('/one')
        self.assertNotIn('/one', self.client._callback_failures)
        self.assertIn('/two', self.client._callback_failures)

    def test_remote_protocol_logs_escape_and_bound_selected_fields(self):
        payload = '\x00\x1b[31m\n' + 'x' * 100_000
        with patch('builtins.print') as log:
            self.packet({'op': 'status', 'level': 'error', 'id': payload,
                         'msg': payload, 'irrelevant': 'not logged'})
        line = log.call_args[0][0]
        self.assertLess(len(line), 450)
        self.assertNotIn('not logged', line)
        self.assertNotIn('\x00', line)
        self.assertNotIn('\x1b', line)
        self.assertNotIn('\n', line)
        self.assertIn('\\x00', line)
        self.assertFalse(self.client.connection[0])
        self.assertTrue(self.sock.closed)

    def test_remote_log_fields_do_not_format_nested_objects(self):
        with patch('builtins.print') as log:
            self.packet({'op': 'status', 'level': 'error',
                         'id': {'nested': 'x' * 100_000}, 'msg': ['x' * 100_000]})
        self.assertIn('id=<dict>: <list>', log.call_args[0][0])

    def test_closed_client_rejects_late_open(self):
        self.client.close()
        self.client._on_open(self.sock)
        self.assertFalse(self.client.connection[0])

    def test_packets_from_previous_socket_ignored(self):
        received = []
        self.client.subscribe("/state", "std_msgs/Bool", received.append)
        self.client._on_message(FakeSocket(), '{"op":"publish","topic":"/state","msg":{"data":true}}')
        self.assertEqual(received, [])


if __name__ == "__main__":
    unittest.main()
