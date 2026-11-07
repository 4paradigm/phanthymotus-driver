"""Health diagnostics, real bundle integration and lifecycle without hardware."""
from copy import deepcopy
import json
import tempfile
import threading
import unittest
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.health import PiperHealthPlugin, health_report
from common.vendor_runtime import DriverBundle
from test_piper_intent import IntentDriver
from test_piper_monitor import Transport


def snapshot():
    return {'connected': True, 'feedback_time': 1000., 'fresh': True,
            'joints_deg': [0., 90., -90., 0., 20., 0.],
            'joint_limits_deg': [[-150, 150], [0, 180], [-170, 0], [-100, 100], [-70, 70], [-120, 120]],
            'enabled': [True] * 6, 'control_mode': 'CAN_CTRL(0x1)',
            'faults': {'arm_status': 'NORMAL(0x0)', 'teach_status': 'DISABLED(0x0)',
                       'error_code': 0, 'motor_faults': []},
            'motion_enabled': True, 'busy': False, 'control_card_started': True}


class HealthReportTests(unittest.TestCase):
    def test_fresh_ready_report_does_not_mutate_source(self):
        data = snapshot()
        before = deepcopy(data)
        report = health_report(data, now=1000.1)
        self.assertEqual(report['status'], 'ready')
        self.assertTrue(report['ready_for_direct_control'])
        self.assertTrue(all(check['passed'] is True for check in report['checks']))
        self.assertEqual(report['display']['J6 使能'], True)
        self.assertEqual(data, before)
        json.dumps(report, allow_nan=False)

    def test_each_failed_prerequisite_blocks_readiness(self):
        cases = [('permission', {'motion_enabled': False}), ('idle', {'busy': True}),
                 ('mode', {'control_mode': 'TEACHING_MODE(0x2)'}),
                 ('enabled', {'enabled': [True, False, True, True, True, True]}),
                 ('control_card', {'control_card_started': False}),
                 ('limits', {'joints_deg': [160., 90., -90., 0., 20., 0.]})]
        for check_id, changes in cases:
            with self.subTest(check=check_id):
                report = health_report({**snapshot(), **changes}, now=1000.)
                self.assertFalse(report['ready_for_direct_control'])
                self.assertIn(check_id, [item['id'] for item in report['blockers']])

    def test_fault_and_teaching_details_are_retained_without_guessing_codes(self):
        data = snapshot()
        data['faults'].update(arm_status='SINGULARITY_POINT(0x4)', error_code=35,
                              motor_faults=['J2:motor_overheating'], teach_status='PAUSE_EXECUTION(0x5)')
        report = health_report(data, now=1000.)
        self.assertEqual(report['status'], 'fault')
        self.assertEqual(report['display']['故障码'], 35)
        self.assertIn('J2:motor_overheating', report['display']['电机故障'])
        self.assertIn('teaching', [item['id'] for item in report['blockers']])

    def test_disconnect_stale_and_future_feedback_never_show_current_enabled_or_fault_free(self):
        for changes, now in [({'connected': False}, 1000.), ({}, 1001.),
                             ({'feedback_time': 1001.}, 1000.), ({'fresh': False}, 1000.)]:
            with self.subTest(changes=changes, now=now):
                report = health_report({**snapshot(), **changes}, now=now)
                self.assertFalse(report['ready_for_direct_control'])
                self.assertFalse(report['feedback_fresh'])
                self.assertIsNone(report['display']['J1 使能'])
                self.assertIsNone(report['display']['故障码'])
                self.assertEqual(report['display']['电机故障'], '不可用')

    def test_missing_and_malformed_values_fail_closed_and_serialize(self):
        cases = [{}, {'connected': None}, {'enabled': [1] * 6}, {'faults': {}},
                 {'faults': {'arm_status': 'NORMAL', 'error_code': False, 'motor_faults': []}},
                 {'joints_deg': [float('nan')] * 6}, {'feedback_time': float('inf')},
                 {'feedback_time': True}, {'joint_limits_deg': [[float('nan'), 1]] * 6},
                 {'motion_enabled': 'true'}, {'busy': 0}]
        for data in cases:
            with self.subTest(data=data):
                report = health_report({**snapshot(), **data} if data else {}, now=1000.)
                self.assertFalse(report['ready_for_direct_control'])
                json.dumps(report, allow_nan=False)


class HealthLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.data = snapshot()
        self.plugin = PiperHealthPlugin(lambda: deepcopy(self.data), namespace='arm1/piper',
                                        ros2=object(), clock=lambda: 1000., publisher_factory=Transport)
        self.addCleanup(self.plugin.stop)

    def test_publishes_only_health_on_independent_topic_and_stops_idempotently(self):
        self.plugin.start()
        transport = self.plugin._transport
        self.assertEqual(transport.period, .5)
        self.assertEqual(transport.topics, [{'topic': '/arm1/piper/state/health', 'format': 'data/json'}])
        self.plugin.start()
        self.assertIs(self.plugin._transport, transport)
        transport.callback()
        self.assertEqual(transport.messages[0][1]['诊断结果'], '通过基础检查')
        self.data['fresh'] = False
        transport.callback()
        self.assertEqual(transport.messages[-1][1]['诊断结果'], '反馈过期或无效')
        self.plugin.stop()
        self.plugin.stop()
        transport.callback()
        self.assertEqual(len(transport.messages), 2)
        self.assertEqual(transport.closed, 1)

    def test_late_sample_cannot_publish_after_stop_or_into_new_generation(self):
        entered, release = threading.Event(), threading.Event()
        def slow_read():
            entered.set()
            release.wait(3)
            return snapshot()
        self.plugin.start()
        old = self.plugin._transport
        self.plugin._read_snapshot = slow_read
        worker = threading.Thread(target=old.callback)
        worker.start()
        self.assertTrue(entered.wait(1))
        # Avoid a second slow read in stop's info; the in-flight call already owns slow_read.
        self.plugin._read_snapshot = snapshot
        self.plugin.stop()
        self.plugin.start()
        new = self.plugin._transport
        release.set()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(old.messages, [])
        self.assertEqual(new.messages, [])
        new.callback()
        self.assertEqual(len(new.messages), 1)

    def test_read_failure_is_visible_and_next_valid_sample_recovers(self):
        self.plugin._read_snapshot = lambda: (_ for _ in ()).throw(RuntimeError('cache\nunavailable'))
        self.assertEqual(self.plugin.info()['health']['status'], 'read_error')
        self.assertEqual(self.plugin.info()['health']['read_error'], 'cache unavailable')
        self.plugin._read_snapshot = snapshot
        self.assertTrue(self.plugin.info()['health']['ready_for_direct_control'])

    def test_publish_failure_recovers_and_control_parameters_are_rejected(self):
        self.plugin.start()
        transport = self.plugin._transport
        transport.fail = True
        transport.callback()
        self.assertEqual(self.plugin.info()['state'], 'error')
        transport.fail = False
        transport.callback()
        self.assertEqual(self.plugin.info()['state'], 'running')
        for action, args in [('move', {}), ('info', {'speed_percent': 10}),
                             ('info', {'input_topic': 'other'}), ('info', {'_tool_name': 'arm_control'})]:
            with self.subTest(action=action, args=args), self.assertRaises(ValueError):
                self.plugin.dispatch(action, args)


class HealthBundleTests(unittest.TestCase):
    def test_real_bundle_health_is_passive_even_while_action_gate_is_held(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '1'}):
            plugins = build_plugins({'data_dir': directory, 'async_actions': False}, 'arm1', None)
            runtime = plugins[0]
            runtime.factory = IntentDriver
            plugins[1].auto_connect = False
            try:
                bundle = DriverBundle(plugins)
                # A health query must not establish even a read-only connection.
                with patch.object(runtime, 'connected', side_effect=AssertionError('Health attempted CAN connect')):
                    offline = bundle.dispatch('piper_health', {'action': 'info'})
                self.assertEqual(offline['health']['status'], 'disconnected')
                self.assertIsNone(runtime.driver)
                driver = runtime.connected()
                before = (runtime.motion_enabled, runtime.cancel.is_set(), runtime.last_signature)
                runtime.gate.acquire()
                try:
                    for action in ('start', 'info', 'stop'):
                        info = bundle.dispatch('piper_health', {'action': action, 'input_topic': ''})
                        self.assertFalse(info['health']['ready_for_direct_control'])
                        self.assertTrue(runtime.gate.locked())
                finally:
                    runtime.gate.release()
                self.assertEqual(driver.arm.calls, [])
                self.assertEqual(before, (runtime.motion_enabled, runtime.cancel.is_set(), runtime.last_signature))
                with runtime.connection_lock:
                    # Other threads must not wait behind a connection change.
                    responses = []
                    thread = threading.Thread(target=lambda: responses.append(bundle.dispatch('piper_health', {'action': 'info'})))
                    thread.start(); thread.join(1)
                    self.assertFalse(thread.is_alive())
                    self.assertFalse(responses[0]['health']['ready_for_direct_control'])
            finally:
                for plugin in reversed(plugins):
                    plugin.stop()

    def test_namespace_isolation(self):
        first = PiperHealthPlugin(snapshot, namespace='arm1/piper')
        second = PiperHealthPlugin(snapshot, namespace='arm2/piper')
        self.assertNotEqual(first.get_tools()[0]['topic_out'], second.get_tools()[0]['topic_out'])


if __name__ == '__main__':
    unittest.main()
