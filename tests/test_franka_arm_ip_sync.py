"""Controller configuration startup, persistence and trusted discovery."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'franka/fr3')]

from common.vendor_runtime import DriverBundle
from franka.fr3 import arm_ip_sync as controller

ARMS = {'left': '172.16.0.3', 'right': '172.16.0.2'}
HOST = '172.16.0.200'


class ControllerConfigurationTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.state = Path(temp.name)
        self.enterContext(patch.object(controller, 'STATE', self.state))
        self.plugin = controller.ControllerConfiguration(ARMS)
        self.addCleanup(self.stop)

    def stop(self):
        self.plugin.stop()
        if self.plugin.thread:
            self.plugin.thread.join(2)

    def ready(self):
        deadline = time.monotonic() + 2
        while True:
            try:
                return self.plugin.snapshot()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.01)

    def test_configuration_only_and_boot_discovery_without_canvas_call(self):
        card, = self.plugin.get_tools()
        self.assertEqual(card['title'], 'Controller Configuration')
        self.assertEqual(set(card['configSchema']['properties']), {'username', 'password'})
        password = card['configSchema']['properties']['password']
        self.assertEqual(password['default'], 'ros')
        self.assertTrue(password['x-sensitive'])
        self.assertEqual((card['topic_in'], card['topic_out']), ([], []))
        self.assertIsNone(self.plugin.dispatch('info', {}))
        with self.assertRaises(RuntimeError):
            self.plugin.snapshot()
        with patch.object(self.plugin, 'candidates', return_value=[HOST]), \
             patch.object(self.plugin, 'read_controller', return_value=ARMS) as read:
            DriverBundle([self.plugin]).start_all()
            self.assertEqual(self.ready(), (HOST, ARMS, 'pnp', 'ros'))
        read.assert_called_once_with(HOST, ('pnp', 'ros'))
        self.assertEqual(json.loads((self.state / 'binding.json').read_text()),
                         {'controller_ip': HOST, 'motion_arm_ips': ARMS})

    def test_credential_change_discards_unfinished_discovery_and_persists_privately(self):
        entered = threading.Event()
        release = threading.Event()
        self.addCleanup(release.set)

        def find(previous, credentials):
            if credentials == ('pnp', 'ros'):
                entered.set()
                release.wait(2)
            return HOST, ARMS

        with patch.object(self.plugin, 'find_binding', side_effect=find) as search:
            self.plugin.start()
            self.assertTrue(entered.wait(1))
            result = self.plugin.dispatch('config', {'username': 'operator', 'password': 'test-password'})
            self.assertEqual(result, {'ok': True, 'username': 'operator'})
            release.set()
            self.assertEqual(self.ready(), (HOST, ARMS, 'operator', 'test-password'))
        self.assertEqual(search.call_count, 2)
        self.assertEqual((self.state / 'credentials.json').stat().st_mode & 0o777, 0o600)
        restarted = controller.ControllerConfiguration(ARMS)
        self.assertEqual((restarted.username, restarted.password), ('operator', 'test-password'))
        with self.assertRaises(RuntimeError):
            restarted.snapshot()

    def test_discovery_precedes_saved_host_fallback(self):
        with patch.object(self.plugin, 'candidates', return_value=['172.16.0.10']), \
             patch.object(self.plugin, 'read_controller', side_effect=[ValueError('Invalid mapping'), ARMS]) as read:
            self.assertEqual(self.plugin.find_binding(HOST, ('pnp', 'ros')), (HOST, ARMS))
        self.assertEqual([c.args[0] for c in read.call_args_list], ['172.16.0.10', HOST])

    def test_disagreeing_desk_and_hardware_are_rejected(self):
        def configuration(_, path, credentials):
            if path == controller.ROBOT_CONFIG:
                return {'robots': {'left_fci': {'host': ARMS['left']}, 'right_fci': {'host': ARMS['right']}}}
            return {'robot_ip': '172.16.0.99'}

        with patch.object(self.plugin, 'read_file', side_effect=configuration):
            with self.assertRaisesRegex(ValueError, 'disagree'):
                self.plugin.read_controller(HOST, ('pnp', 'ros'))
