"""Chassis RPC routing stays separate from read-only upper control feedback."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch

import test_controlled_spatial
from spatial_client import JatenClient
from mapping_keyboard import FactoryKeyboardStream

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('native_master_rpc', ROOT/'host/navigation_rpc.py')
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class NativeMasterTests(unittest.TestCase):
    def client(self, chassis_master=None):
        self.calls = []
        target = {'address': '192.168.26.22', 'ros_ip': '192.168.26.22'}
        if chassis_master:
            target['master'] = chassis_master
        def run(role, target, request, source, secrets, **kwargs):
            self.calls.append(request)
            return {'ok': True, 'result': {'owner': '/leju_node'}}
        def stream(role, target, request, source, secrets):
            self.calls.append(request)
            return types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda: 123))
        manager = types.SimpleNamespace(settings={'master': 'http://upper:11311'},
            state_dir=Path('/unused'), _target=lambda role: target,
            transport=types.SimpleNamespace(run=run, open_stream=stream))
        return JatenClient(manager)

    def test_native_services_default_to_chassis_and_keep_upper_guard_address(self):
        client = self.client()
        with patch('spatial_client.load_secrets', return_value={}):
            client.native('probe')
        self.assertEqual(self.calls[0]['master'], 'http://127.0.0.1:11311')
        self.assertEqual(self.calls[0]['upper_master'], 'http://upper:11311')

    def test_explicit_chassis_master_override_is_honored(self):
        client = self.client('http://chassis-site:11411')
        with patch('spatial_client.load_secrets', return_value={}):
            client.native('probe')
        self.assertEqual(self.calls[0]['master'], 'http://chassis-site:11411')
        self.assertEqual(self.calls[0]['upper_master'], 'http://upper:11311')

    def test_keyboard_guard_uses_the_same_chassis_master(self):
        client = self.client()
        with patch('mapping_keyboard.load_secrets', return_value={}), \
                patch('mapping_keyboard.os.set_blocking'):
            FactoryKeyboardStream(client)
        self.assertEqual(self.calls[0]['master'], 'http://127.0.0.1:11311')

    def test_rpc_keeps_fixed_failure_codes_and_redacts_arbitrary_peer_text(self):
        self.assertEqual(rpc.rpc_error_code(RuntimeError('native_navigation_services_need_one_common_owner')),
                         'native_navigation_services_need_one_common_owner')
        for message in ('https://user:secret@peer/path', 'secret_value', 'remote error with credentials'):
            self.assertEqual(rpc.rpc_error_code(RuntimeError(message)), 'native_navigation_rpc_failed')


class UpperObservationTests(unittest.TestCase):
    def setUp(self):
        self.topic, self.owner = '/joy', '/sdk'
        self.kind = type('Message', (), {'_type': 'sensor_msgs/Joy', '_md5sum': 'message-md5'})
        self.header = {'callerid': self.owner, 'type': self.kind._type, 'md5sum': self.kind._md5sum}
        self.closed = threading.Event()
        self.received = threading.Event()
        self.connection = types.SimpleNamespace(header=self.header,
            connect=lambda *args, **kwargs: None, close=self.closed.set)
        def receive():
            if not self.received.is_set():
                self.received.set()
                return ['sample1', 'sample2']
            self.closed.wait(2)
            raise ConnectionError('closed')
        self.connection.receive_once = receive
        self.master = types.SimpleNamespace(
            getSystemState=lambda caller: (1, '', [[(self.topic, [self.owner])], [], []]),
            lookupNode=lambda *args: (1, '', 'http://upper-publisher:12345'),
            requestTopic=lambda *args: (1, '', ['TCPROS', 'upper-publisher', 12346]))
        self.proxy_patch = patch('xmlrpc.client.ServerProxy')
        factory = self.proxy_patch.start()
        factory.return_value.__enter__.return_value = self.master
        self.addCleanup(self.proxy_patch.stop)
        self.protocol = unittest.mock.Mock(return_value=object())
        self.modules = patch.dict(sys.modules, {
            'rospy.impl.tcpros_base': types.SimpleNamespace(TCPROSTransport=lambda *args: self.connection),
            'rospy.impl.tcpros_pubsub': types.SimpleNamespace(TCPROSSub=self.protocol)})
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def test_upper_stream_is_read_directly_and_all_frames_are_processed(self):
        messages = []
        observer = rpc.UpperTopicObservation('http://upper:11311', self.topic, self.kind,
                                             self.owner, messages.append)
        try:
            self.assertTrue(self.received.wait(1))
            test_controlled_spatial.wait(lambda: len(messages) == 2)
            self.assertEqual(messages, ['sample1', 'sample2'])
            self.assertIsNone(self.protocol.call_args.kwargs['queue_size'])
        finally:
            observer.unregister()
        self.assertTrue(self.closed.is_set())
        self.assertFalse(observer.thread.is_alive())

    def test_unverified_publisher_is_rejected_before_connecting(self):
        self.master.getSystemState = lambda caller: (1, '', [[(self.topic, ['/other'])], [], []])
        with self.assertRaisesRegex(RuntimeError, 'publisher_unconfirmed'):
            rpc.UpperTopicObservation('http://upper:11311', self.topic, self.kind, self.owner, lambda msg: None)
        self.protocol.assert_not_called()

    def test_wrong_handshake_identity_closes_the_connection(self):
        self.header['callerid'] = '/other'
        with self.assertRaisesRegex(RuntimeError, 'identity_mismatch'):
            rpc.UpperTopicObservation('http://upper:11311', self.topic, self.kind, self.owner, lambda msg: None)
        self.assertTrue(self.closed.is_set())


if __name__ == '__main__':
    unittest.main()
