"""Offline deployment contracts: no Docker, ROS or robot required."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import yaml

from scripts.check_service_yml import check, env_map
from agilex.piper.deploy import diagnose
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
from test_piper_intent import IntentDriver


ROOT = Path(__file__).resolve().parents[1]
SERVICE = ROOT / "agilex/piper/deploy/service.yml"


class PiperDeploymentTests(unittest.TestCase):
    def setUp(self):
        self.service = yaml.safe_load(SERVICE.read_text())["agilex-piper"]
        self.environment = env_map(self.service["environment"])

    def test_pr_fragment_shares_host_can_and_dds_network(self):
        # run-pr-image.sh wraps this fragment; it does not inject host networking.
        self.assertEqual(self.service.get("network_mode"), "host")
        self.assertNotIn("ports", self.service)

    def test_exact_readonly_dds_profile_is_mounted_and_selected(self):
        profile = "/opt/phanthy-motus/dds-local.xml"
        self.assertIn(f"{profile}:{profile}:ro", self.service["volumes"])
        self.assertEqual(self.environment["FASTRTPS_DEFAULT_PROFILES_FILE"], profile)
        self.assertEqual(self.environment["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
        self.assertEqual(self.environment["ROS_DOMAIN_ID"], "42")
        self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", self.environment)

    def test_repo_deployment_checks_and_motion_default(self):
        violations, _ = check(str(SERVICE), "agilex/piper")
        self.assertEqual(violations, [])
        self.assertEqual(self.environment["PIPER_MOTION_ENABLED"], "${PIPER_MOTION_ENABLED:-0}")
        self.assertEqual(self.environment["PIPER_CAMERA_ENABLED"], "0")

    def test_can_service_exposes_no_host_devices_or_elevated_capabilities(self):
        self.assertFalse(self.service.get('privileged', False))
        self.assertEqual(self.service['cap_drop'], ['ALL'])
        self.assertFalse(self.service.get('cap_add'))
        self.assertFalse(self.service.get('devices'))
        self.assertFalse(self.service.get('device_cgroup_rules'))
        self.assertFalse(any(str(v).split(':', 1)[0] == '/dev' for v in self.service['volumes']))
        self.assertIn('no-new-privileges:true', self.service['security_opt'])

    def test_optional_camera_maps_only_an_explicit_color_device(self):
        override = yaml.safe_load((SERVICE.parent / 'camera.compose.yml').read_text())['services']['agilex-piper']
        self.assertEqual(set(override), {'devices', 'environment'})
        self.assertEqual(len(override['devices']), 1)
        self.assertEqual(override['devices'][0],
                         '${PIPER_CAMERA_DEVICE:?Set the explicit color V4L2 path}:'
                         '${PIPER_CAMERA_DEVICE:?Set the explicit color V4L2 path}:rw')
        self.assertEqual(env_map(override['environment'])['PIPER_CAMERA_ENABLED'], '1')


class PiperDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        motion_env = patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '0'})
        motion_env.start(); self.addCleanup(motion_env.stop)
        self.plugins = build_plugins({'data_dir': self.tmp.name, 'async_actions': False}, 'agilex', None)
        self.runtime, self.background = self.plugins[:2]
        self.background.auto_connect = False
        self.runtime.factory = IntentDriver
        self.sdk = self.runtime.connected().arm
        self.bundle = DriverBundle(self.plugins)
        for plugin in self.plugins:
            self.addCleanup(plugin.stop)
        self.requests = []
        self.read_count = 0
        def transport(endpoint, method, params=None):
            self.requests.append((method, params))
            if method == 'initialize':
                return {'serverInfo': {'name': 'PiPER'}}
            if method == 'tools/list':
                return {'tools': self.bundle.get_all_tools()}
            self.assertEqual(method, 'tools/call')
            name, args = params['name'], params['arguments']
            self.assertIn((name, args.get('action')),
                          [('arm_control', 'info'), ('model', None), ('piper_camera_live', 'info'),
                           ('piper_photo', 'capture'), ('piper_photo', 'info')])
            value = self.bundle.dispatch(name, args)
            if name == 'arm_control':
                self.read_count += 1
            return {'content': [{'type': 'text', 'text': json.dumps(value)}]}
        self.rpc = patch.object(diagnose, 'rpc', side_effect=transport)
        self.rpc.start(); self.addCleanup(self.rpc.stop)
        host = patch.object(diagnose, 'host_check', return_value={'returncode': 0, 'output': ''})
        self.host = host.start(); self.addCleanup(host.stop)

    def test_current_production_health_check_only_uses_passive_tools(self):
        for gate in (False, True):
            with self.subTest(gate=gate):
                self.runtime.motion_enabled = gate
                self.requests.clear()
                self.runtime.config['can_interface'] = 'can1'
                result = diagnose.diagnose('http://fake/mcp')
                self.assertTrue(result['passed'], result['errors'])
                self.assertEqual(set(result['cards']), {'arm_control', 'joints', 'model', 'piper_photo', 'piper_camera_live'})
                calls = [p for method, p in self.requests if method == 'tools/call']
                self.assertEqual(calls, [
                    {'name': 'arm_control', 'arguments': {'action': 'info'}},
                    {'name': 'model', 'arguments': {}},
                    {'name': 'piper_camera_live', 'arguments': {'action': 'info'}},
                    {'name': 'arm_control', 'arguments': {'action': 'info'}}])
                self.host.assert_any_call(['ip', '-details', 'link', 'show', 'can1'])
                self.assertTrue(result['unchanged'])
                self.assertEqual(result['control_commands_requested'], 0)
                self.assertEqual(self.sdk.calls, [])

    def test_removed_interface_fails_before_any_tool_call(self):
        with patch.object(diagnose, 'rpc', side_effect=lambda endpoint, method, params=None:
                {'tools': [{'name': 'piper_status'}]} if method == 'tools/list' else
                {'serverInfo': {'name': 'Old PiPER'}} if method == 'initialize' else
                self.fail('Mismatched image received a tool call')):
            result = diagnose.diagnose('http://fake/mcp')
        self.assertFalse(result['passed'])
        self.assertIn('Production tools mismatch', result['errors']['cards'])
        self.assertEqual(self.sdk.calls, [])

    def test_disconnected_or_stale_snapshot_is_not_a_pass(self):
        self.runtime.closed = True
        result = diagnose.diagnose('http://fake/mcp')
        self.assertFalse(result['passed'])
        self.assertIn('feedback', result['errors'])
        self.runtime.closed = False
        self.sdk.frozen = True
        result = diagnose.diagnose('http://fake/mcp')
        self.assertFalse(result['passed'])
        self.assertEqual(self.sdk.calls, [])

    def test_mode_or_enable_change_is_reported_without_recovery_controls(self):
        original = self.bundle.dispatch
        for field, value in [('mode', 'STANDBY(0x0)'), ('enabled', (False,) * 6)]:
            self.read_count = 0
            self.sdk.mode = 'CAN_CTRL(0x1)'; self.sdk.enabled = (True,) * 6
            def changed(name, args):
                if name == 'arm_control' and self.read_count == 1:
                    setattr(self.sdk, field, value)
                return original(name, args)
            with self.subTest(field=field), patch.object(self.bundle, 'dispatch', side_effect=changed):
                result = diagnose.diagnose('http://fake/mcp')
            self.assertFalse(result['passed'])
            self.assertIn('unchanged', result['errors'])
            self.assertEqual(self.sdk.calls, [])

    def test_optional_photo_is_explicit_and_never_stops_camera_or_moves_arm(self):
        original = self.bundle.dispatch
        def capture(name, args):
            if name == 'piper_photo':
                self.assertEqual(args, {'action': 'capture', 'capture_confirmed': True})
                return {'photo': 'synthetic'}
            return original(name, args)
        with patch.object(self.bundle, 'dispatch', side_effect=capture):
            result = diagnose.diagnose('http://fake/mcp', photos=1)
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(result['photos'], [{'photo': 'synthetic'}])
        self.assertEqual(self.sdk.calls, [])

    def test_cli_writes_new_snapshot_report_and_rejects_removed_firmware_option(self):
        output = Path(self.tmp.name) / 'report.json'
        with patch('sys.argv', ['diagnose.py', '--output', str(output)]), \
             self.assertRaises(SystemExit) as exit_:
            diagnose.main()
        self.assertEqual(exit_.exception.code, 0)
        self.assertTrue(json.loads(output.read_text())['before']['snapshot']['fresh'])
        self.requests.clear()
        with patch('sys.argv', ['diagnose.py', '--query-firmware']), \
             self.assertRaises(SystemExit) as exit_:
            diagnose.main()
        self.assertEqual(exit_.exception.code, 2)
        self.assertEqual(self.requests, [])


if __name__ == "__main__":
    unittest.main()
