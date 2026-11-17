"""Arm mapping configuration and synchronization without network access."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'franka/fr3'))
spec = importlib.util.spec_from_file_location('fr3_arm_ip_sync_test', ROOT / 'franka/fr3/arm_ip_sync.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class ArmIPSyncTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.state = Path(self.tmp.name)
        self.patch = patch.object(m, 'STATE', self.state)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        with patch.object(m, 'load_binding', return_value={}):
            self.plugin = m.ArmIPSyncPlugin({'left': '172.16.0.3', 'right': '172.16.0.2'}, 'franka', None, callback=lambda *args: None)

    def test_sensor_lifecycle_publication_and_english_metadata(self):
        import re
        from types import SimpleNamespace
        from unittest.mock import Mock
        publisher = Mock()
        factory = Mock(return_value=publisher)
        self.plugin.ros2 = object()
        tool = self.plugin.get_tools()[0]
        self.assertEqual(tool['type'], 'sensor')
        self.assertFalse(tool['multiInstance'])
        self.assertEqual(tool['topic_out'][0]['format'], 'data/json')
        for prop in tool['configSchema']['properties'].values():
            self.assertIsNone(re.search(r'[\u4e00-\u9fff]', prop.get('title', '') + prop.get('description', '')))
        self.assertEqual(self.plugin.dispatch('info', {})['state'], 'idle')
        with patch.dict(sys.modules, {'device': SimpleNamespace(RosPublisher=factory)}):
            self.assertEqual(self.plugin.dispatch('start', {})['state'], 'running')
            self.plugin.dispatch('start', {})
        factory.assert_called_once()
        self.plugin.publish()
        publisher.publish.assert_called_once_with({'left': '172.16.0.3', 'right': '172.16.0.2'})
        info = self.plugin.dispatch('info', {})
        self.assertTrue(info['fresh'])
        self.assertEqual(info['frames_published'], 1)
        self.assertEqual(info['topic_out'], tool['topic_out'])
        self.assertEqual(self.plugin.dispatch('stop', {})['state'], 'idle')
        self.assertFalse(self.plugin.dispatch('info', {})['fresh'])
        publisher.close.assert_called_once()

    def test_password_default_and_user_override(self):
        self.assertEqual(self.plugin.password, 'ros')
        schema = self.plugin.get_tools()[0]['configSchema']['properties']['password']
        self.assertEqual(schema['default'], 'ros')
        self.assertTrue(schema['x-sensitive'])
        result = self.plugin.dispatch('config', {'password': 'test-custom-password'})
        self.assertTrue(result['ok'])
        self.assertEqual(self.plugin.password, 'test-custom-password')
        self.assertNotIn('password', result['config'])
        self.plugin.dispatch('config', {'password': ''})
        self.assertEqual(self.plugin.password, 'ros')

    def test_defaults_and_controller_only_in_configuration(self):
        tool = self.plugin.get_tools()[0]
        self.assertNotIn('controller_ip', tool['inputSchema']['properties'])
        self.assertEqual(tool['configSchema']['properties']['left_ip']['default'], '172.16.0.3')
        self.assertEqual(tool['configSchema']['properties']['controller_ip']['default'], '')
        self.assertEqual(self.plugin.dispatch('info', {})['motion_arm_ips'], self.plugin.arm_ips)

    def test_manual_configuration_persists_and_rejects_duplicates_atomically(self):
        result = self.plugin.dispatch('config', {'left_ip': '172.16.0.4', 'controller_ip': '172.16.0.200'})
        self.assertFalse(result['restart_required'])
        before = (self.state / 'binding.json').read_text()
        self.assertEqual(self.plugin.dispatch('config', {'right_ip': '172.16.0.4'})['state'], 'error')
        self.assertEqual((self.state / 'binding.json').read_text(), before)
        self.assertEqual(self.plugin.arm_ips['left'], '172.16.0.4')

    def sync_result(self):
        accepted = self.plugin.dispatch('sync', {})
        self.assertEqual(accepted['state'], 'running')
        deadline = time.monotonic() + 2
        while self.plugin.lock.locked() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(self.plugin.lock.locked())
        self.assertEqual(self.plugin.last_result['action_id'], accepted['action_id'])
        return self.plugin.last_result

    def test_sync_updates_config_output_and_preserves_on_failure(self):
        arms = {'left': '172.16.0.5', 'right': '172.16.0.6'}
        with patch.object(self.plugin, 'candidates', return_value=['172.16.0.200']), patch.object(self.plugin, 'read_controller', return_value=arms):
            terminal = self.sync_result()
        self.assertEqual(terminal['state'], 'completed')
        self.assertEqual(terminal['result']['config']['left_ip'], arms['left'])
        self.assertEqual(self.plugin.dispatch('info', {})['motion_arm_ips'], arms)
        before = (self.state / 'binding.json').read_text()
        with patch.object(self.plugin, 'candidates', return_value=[]):
            self.assertEqual(self.sync_result()['state'], 'error')
        self.assertEqual((self.state / 'binding.json').read_text(), before)
        with patch.object(m, 'load_binding', return_value=m.load_binding(self.state)):
            restarted = m.ArmIPSyncPlugin(arms, 'franka', None, callback=lambda *args: None)
        self.assertEqual(restarted.arm_ips, arms)

    def test_discovery_precedes_optional_fallback(self):
        from unittest.mock import Mock
        arms = {'left': '172.16.0.5', 'right': '172.16.0.6'}
        self.plugin.dispatch('config', {'controller_ip': '172.16.0.201'})
        reader = Mock(side_effect=[ValueError('Not a PNP controller'), arms])
        with patch.object(self.plugin, 'candidates', return_value=['172.16.0.184']), patch.object(self.plugin, 'read_controller', reader):
            self.assertEqual(self.sync_result()['state'], 'completed')
        self.assertEqual([c.args[0] for c in reader.call_args_list], ['172.16.0.184', '172.16.0.201'])
        with patch.object(self.plugin, 'candidates', return_value=['172.16.0.200']), patch.object(self.plugin, 'read_controller', return_value=arms) as reader:
            self.assertEqual(self.sync_result()['result']['controller_ip'], '172.16.0.200')
        reader.assert_called_once_with('172.16.0.200')
        self.assertEqual(self.plugin.configuration()['controller_ip'], '172.16.0.201')

    def test_discovery_failure_still_tries_fallback_and_retains_mapping_on_failure(self):
        self.plugin.dispatch('config', {'controller_ip': '172.16.0.201'})
        before = (self.state / 'binding.json').read_text()
        with patch.object(self.plugin, 'candidates', side_effect=OSError('No interface')), patch.object(self.plugin, 'read_controller', side_effect=ValueError('Unavailable')) as reader:
            self.assertEqual(self.sync_result()['state'], 'error')
        reader.assert_called_once_with('172.16.0.201')
        self.assertEqual((self.state / 'binding.json').read_text(), before)
        self.assertEqual(self.plugin.arm_ips, {'left': '172.16.0.3', 'right': '172.16.0.2'})

    def test_network_discovery_probes_ethernet_ssh_without_arm_or_local_addresses(self):
        devices = [{'ifname': 'eno1', 'flags': ['UP'], 'addr_info': [
            {'local': '172.16.0.1', 'prefixlen': 29, 'scope': 'global'}]},
            {'ifname': 'wlan0', 'flags': ['UP'], 'addr_info': [
                {'local': '10.0.0.1', 'prefixlen': 24, 'scope': 'global'}]}]
        with patch.dict(m.os.environ, {'FRANKA_ROBOT_INTERFACE': ''}), patch.object(m, 'network_json', return_value=devices), patch.object(m.socket, 'create_connection') as connect:
            self.assertEqual(self.plugin.candidates(), ['172.16.0.4', '172.16.0.5', '172.16.0.6'])
        self.assertEqual({c.args[0] for c in connect.call_args_list}, {('172.16.0.4', 22), ('172.16.0.5', 22), ('172.16.0.6', 22)})

    def test_controller_configuration_disagreement_is_rejected(self):
        def read(address, path):
            if path == m.ROBOT_CONFIG:
                return {'robots': {'left_fci': {'host': '172.16.0.3'}, 'right_fci': {'host': '172.16.0.2'}}}
            if path.endswith('left_franka_hardware.yaml'):
                return {'robot_ip': '172.16.0.8'}
            return {'robot_ip': '172.16.0.2'}
        with patch.object(self.plugin, 'read_file', side_effect=read):
            with self.assertRaisesRegex(ValueError, 'disagree'):
                self.plugin.read_controller('172.16.0.200')
