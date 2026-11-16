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
