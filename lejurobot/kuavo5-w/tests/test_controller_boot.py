import importlib.util
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'host' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

boot = module('controller_boot')
install = module('install_controller')


class ControllerBootTests(unittest.TestCase):
    def setUp(self):
        self.requests, self.handles = [], []
        def request(action):
            self.requests.append(action)
            pending = types.SimpleNamespace(done=threading.Event(), response=None, error=False)
            self.handles.append(pending)
            return pending
        self.sequence = boot.BootSequence(63, request)
        self.sample = {**boot.EXPECTED_DOF, 'robot_version': 63, 'raw_valid': True,
            'raw_at': 10., 'ready_to_start': 1, 'services_valid': True, 'pause_service_valid': True}

    def accept(self, success=True, error=False):
        pending = self.handles[-1]
        pending.response = types.SimpleNamespace(success=success)
        pending.error = error
        pending.done.set()

    def test_launch_identity_fresh_measurements_and_services_are_all_required(self):
        for key, value in (('robot_version', 53), ('robot_type', 2), ('armRealDof', True),
                           ('raw_valid', False), ('raw_at', 8), ('ready_to_start', 0),
                           ('services_valid', False)):
            self.sequence.tick(10, {**self.sample, key: value})
        self.assertEqual(self.requests, [])
        self.sequence.tick(10, self.sample)
        self.assertEqual(self.requests, ['initialize'])

    def test_initialized_feedback_then_fresh_pause_confirmed_exactly_once(self):
        self.sequence.tick(10, self.sample)
        self.accept()
        sample = {**self.sample, 'is_ready': 1, 'mode': 3, 'mode_at': 10.1, 'raw_at': 10.1}
        self.sequence.tick(10.1, sample)
        self.assertEqual(self.requests, ['initialize', 'pause'])
        self.accept()
        self.sequence.tick(10.2, {**sample, 'enabled': False, 'enabled_at': 9.9})
        self.assertFalse(self.sequence.pause_confirmed)
        self.sequence.tick(10.2, {**sample, 'enabled': False, 'enabled_at': 10.2})
        self.assertEqual(self.sequence.state, 'complete')
        for now in (20, 1000):
            self.sequence.tick(now, self.sample)
        self.assertEqual(self.requests, ['initialize', 'pause'])
        self.assertTrue(self.sequence.status()['boot_pause_confirmed'])
        self.assertFalse(self.sequence.status()['auto_resume'])
        self.assertEqual(self.sequence.status()['initial_control_mode'], 3)

    def test_initialization_requires_observed_base_arm_before_pause(self):
        self.sequence.tick(10, self.sample)
        self.accept()
        for mode in (None, False, 0, 1, 2, 4, 3.0):
            self.sequence.tick(10.1, {**self.sample, 'is_ready': 1, 'mode': mode,
                                     'mode_at': 10.1, 'raw_at': 10.1})
            self.assertEqual(self.requests, ['initialize'])
            self.assertIsNone(self.sequence.status()['initial_control_mode'])
        self.sequence.tick(10.2, {**self.sample, 'is_ready': 1, 'mode': 3,
                                 'mode_at': 10.2, 'raw_at': 10.2})
        self.assertEqual(self.requests, ['initialize', 'pause'])
        self.assertEqual(self.sequence.status()['initial_control_mode'], 3)

    def test_old_or_stale_mode_cannot_complete_initialization(self):
        self.sequence.tick(10, self.sample)
        self.accept()
        for now, mode_at in ((10.1, 9.9), (20., 10.1)):
            self.sequence.tick(now, {**self.sample, 'is_ready': 1, 'mode': 3,
                                    'mode_at': mode_at, 'raw_at': now})
        self.assertEqual(self.requests, ['initialize'])

    def test_late_initialization_does_not_retry_or_claim_pause_before_settled(self):
        self.sequence.tick(10, self.sample)
        self.sequence.tick(131, {**self.sample, 'raw_at': 131, 'is_ready': 1, 'mode': 3, 'mode_at': 131})
        self.assertEqual(self.sequence.error, 'initialization_timeout_waiting')
        self.assertEqual(self.requests, ['initialize'])
        self.assertFalse(self.sequence.pause_confirmed)
        self.accept()
        self.sequence.tick(132, {**self.sample, 'raw_at': 132, 'is_ready': 1, 'mode': 3, 'mode_at': 132})
        self.assertEqual(self.requests, ['initialize', 'pause'])
        self.accept()
        self.sequence.tick(133, {**self.sample, 'raw_at': 133, 'enabled': False, 'enabled_at': 133})
        self.assertTrue(self.sequence.pause_confirmed)

    def test_failed_or_uncertain_initialization_is_not_retried_or_marked_ready(self):
        for success, error in ((False, False), (True, True)):
            self.setUp()
            self.sequence.tick(10, self.sample)
            self.accept(success, error)
            self.sequence.tick(10.1, self.sample)
            self.sequence.tick(1000, self.sample)
            self.assertEqual(self.sequence.state, 'failed')
            self.assertEqual(self.requests, ['initialize'])
            self.assertFalse(self.sequence.pause_confirmed)

    def test_service_acceptance_alone_cannot_confirm_pause(self):
        self.sequence.tick(10, self.sample)
        self.accept()
        self.sequence.tick(10.1, {**self.sample, 'is_ready': 1, 'mode': 3, 'mode_at': 10.1})
        self.accept()
        self.sequence.tick(10.2, {**self.sample, 'enabled': True, 'enabled_at': 10.2})
        self.assertEqual(self.sequence.state, 'pausing')
        self.assertFalse(self.sequence.pause_confirmed)

    def test_existing_or_stale_controller_registration_refuses_duplicate_launch(self):
        boot.check_no_controller([[['/unrelated', ['/camera']]], [], []])
        # The boot monitor and rosbridge subscribe before the controller exists.
        boot.check_no_controller([[], [['/sensors_data_raw', ['/kuavo5w_controller_boot', '/rosbridge']]], []])
        for state in (([('/sensors_data_raw', ['/unknown'])], [], []),
                      ([], [['/state', ['/mobile_manipulator_sqp_mpc_node']]], []),
                      ([], [], [['/enable_control', ['/nodelet_manager']]])):
            with self.assertRaisesRegex(RuntimeError, 'existing_controller_owner'):
                boot.check_no_controller(state)

    def test_invalid_master_system_state_fails_closed(self):
        for value in (None, [], [[], []], 'invalid'):
            with self.subTest(value=value), self.assertRaisesRegex(RuntimeError, 'invalid_ros_master_system_state'):
                boot.check_no_controller(value)

    def test_normal_master_preflight_ignores_sensor_subscribers_but_not_real_owners(self):
        cases = (([[], [['/sensors_data_raw', ['/joy_node', '/kuavo5w_controller_boot']]], []], False),
                 ([[['/sensors_data_raw', ['/nodelet_manager']]], [], []], True))
        with patch.object(boot.xmlrpc.client, 'ServerProxy') as proxy, patch.dict(
                boot.os.environ, {'ROS_MASTER_URI': 'http://localhost:11311'}):
            master = proxy.return_value.__enter__.return_value
            for state, expected in cases:
                master.getSystemState.return_value = (1, '', state)
                self.assertIs(boot.wait_for_master(threading.Event(), adopt_existing=True), expected)
            master.getSystemState.return_value = (1, '', [])
            with self.assertRaisesRegex(RuntimeError, 'invalid_ros_master_system_state'):
                boot.wait_for_master(threading.Event(), adopt_existing=True)

    def test_systemd_controller_environment_supplies_drake_without_discarding_workspace(self):
        original = {'LD_LIBRARY_PATH': '/ws/lib:/opt/drake/lib/:/opt/ros/noetic/lib', 'ROBOT_VERSION': '63'}
        with patch.object(boot.Path, 'is_file', return_value=True):
            result = boot.controller_environment(original)
            self.assertEqual(result['LD_LIBRARY_PATH'], '/opt/drake/lib:/ws/lib:/opt/ros/noetic/lib')
            self.assertEqual(result['ROBOT_VERSION'], '63')
            self.assertEqual(boot.controller_environment({})['LD_LIBRARY_PATH'], '/opt/drake/lib')
        self.assertEqual(original['LD_LIBRARY_PATH'], '/ws/lib:/opt/drake/lib/:/opt/ros/noetic/lib')
        with patch.object(boot.Path, 'is_file', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'missing_controller_runtime'):
                boot.controller_environment(original)

    def test_existing_paused_controller_can_be_adopted_without_any_service_call(self):
        sequence = boot.BootSequence(63, lambda action: self.fail(action), existing=True)
        for extra in ({'is_ready': 0, 'enabled': False, 'enabled_at': 10},
                      {'is_ready': 1, 'enabled': True, 'enabled_at': 10},
                      {'is_ready': 1, 'enabled': False, 'enabled_at': 1}):
            sequence.tick(10, {**self.sample, **extra})
            self.assertEqual(sequence.state, 'waiting_hardware')
        # The vendor removes real_initial_start after initialization. Only the
        # pause service must remain present for adoption of an existing stack.
        sequence.tick(10, {**self.sample, 'services_valid': False,
                          'is_ready': 1, 'enabled': False, 'enabled_at': 10})
        self.assertEqual(sequence.state, 'complete')
        self.assertEqual(sequence.status()['launch_ownership'], 'external')
        self.assertFalse(sequence.initialization_requested)
        self.assertTrue(sequence.pause_confirmed)

    def test_systemd_unit_has_no_fault_restart_or_container_lifecycle_link(self):
        text = install.service_text('/home/lab/kuavo-ros-opensource', 'http://192.168.26.1:11311', '192.168.26.1')
        self.assertIn('Restart=no', text)
        self.assertIn('ROBOT_VERSION=63', text)
        self.assertIn('KillMode=mixed', text)
        self.assertNotIn('docker', text)
        self.assertNotIn('enable_control', text)
        for field in ('/tmp/a;bad', '/tmp/a\nBad=x', '/tmp/$(bad)'):
            with self.assertRaises(ValueError):
                install.service_text(field, 'http://192.168.26.1:11311', '192.168.26.1')


if __name__ == '__main__':
    unittest.main()
