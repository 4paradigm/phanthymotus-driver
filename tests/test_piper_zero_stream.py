"""Read-only stream lifecycle, stale feedback and concurrent motion isolation."""
import json
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from agilex.piper.device import PiperPlugin
from common.vendor_runtime import DriverBundle
from test_piper_controls import Driver


class Node:
    def __init__(self, *args, **kwargs):
        self.messages = []
        self.destroyed = False
        self.publisher = SimpleNamespace(publish=lambda msg: self.messages.append(json.loads(msg.data)))

    def create_publisher(self, *args):
        self.topic = args[1]
        return self.publisher

    def create_timer(self, period, callback):
        self.period, self.callback = period, callback

    def destroy_node(self):
        self.destroyed = True

    def get_logger(self):
        return SimpleNamespace(error=lambda msg: None)


class ZeroStreamTests(unittest.TestCase):
    def setUp(self):
        modules = {'rclpy.node': SimpleNamespace(Node=Node),
                   'rclpy.qos': SimpleNamespace(QoSProfile=lambda **kw: kw,
                       ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
                       DurabilityPolicy=SimpleNamespace(VOLATILE=0)),
                   'std_msgs.msg': SimpleNamespace(String=SimpleNamespace)}
        stub = patch.dict(sys.modules, modules)
        stub.start()
        self.addCleanup(stub.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.ros = SimpleNamespace(ctx_core=object(), executor_core=Mock())
        self.plugin = PiperPlugin({'data_dir': self.tmp.name}, namespace='test_arm',
                                  ros2=self.ros, driver_factory=Driver)
        self.plugin.motion_enabled = False
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])

    def call(self, action, **kwargs):
        return self.bundle.dispatch('piper_zero_check', {'action': action, **kwargs})

    def start_stream(self):
        self.call('start')
        return self.plugin.zero_stream.node

    def test_service_start_is_passive_and_explicit_start_registers_one_stream(self):
        self.bundle.start_all()
        self.assertIsNone(self.plugin.driver)
        self.assertIsNone(self.plugin.zero_stream.node)
        definition = next(t for t in self.bundle.get_all_tools() if t['name'] == 'piper_zero_check')
        self.assertEqual(definition['type'], 'sensor')
        node = self.start_stream()
        self.assertEqual(node.period, .2)
        self.assertEqual(definition['topic_out'], self.call('info')['topic_out'])
        self.assertEqual(node.topic, '/test_arm/piper/zero_check')
        self.call('start', tolerance_deg=1)
        self.assertIs(self.plugin.zero_stream.node, node)
        self.ros.executor_core.add_node.assert_called_once_with(node)
        self.assertIsNone(self.plugin.driver)

    def test_canvas_actions_remain_visible_after_system_actions_are_filtered(self):
        definition = next(t for t in self.bundle.get_all_tools() if t['name'] == 'piper_zero_check')
        schema = definition['inputSchema']
        actions = schema['properties']['action']
        visible = [action for action in actions['enum'] if action not in {'start', 'stop', 'info', 'config'}]
        self.assertEqual(visible, ['stream', 'read', 'pause'])
        self.assertEqual(actions['default'], 'stream')
        self.assertEqual(schema['x-action-params']['stream']['params'], ['tolerance_deg'])
        self.assertEqual(schema['x-action-params']['pause']['params'], [])
        self.assertEqual(self.call('stream', tolerance_deg=1)['state'], 'running')
        node = self.plugin.zero_stream.node
        node.callback()
        sdk = self.plugin.driver.arm
        self.assertEqual(node.messages[-1]['tolerance_deg'], 1)
        self.assertEqual(self.call('pause')['state'], 'idle')
        self.assertIsNone(self.plugin.zero_stream.node)
        self.assertIsNotNone(self.plugin.driver)
        self.assertEqual(sdk.calls, [])
        self.assertFalse(self.plugin.motion_enabled)

    def test_ticks_track_changed_angles_and_tolerance_without_control_commands(self):
        node = self.start_stream()
        node.callback()
        sdk = self.plugin.driver.arm
        self.assertEqual(node.messages[-1]['joints_deg'][0], 1.)
        self.assertTrue(node.messages[-1]['within_tolerance'][0])
        sdk.raw = (3000, 0, 0, 0, 0, 0)
        node.callback()
        self.assertEqual(node.messages[-1]['joints_deg'][0], 3.)
        self.assertFalse(node.messages[-1]['feedback_near_zero'])
        self.call('start', tolerance_deg=4)
        node.callback()
        self.assertTrue(node.messages[-1]['feedback_near_zero'])
        self.assertEqual(sdk.calls, [])
        self.assertFalse(self.plugin.motion_enabled)

    def test_stale_and_failed_feedback_clear_values_then_recover(self):
        node = self.start_stream()
        node.callback()
        self.plugin.driver.arm.frozen = True
        node.callback()
        data = node.messages[-1]
        self.assertFalse(data['fresh'])
        self.assertIsNone(data['joints_deg'])
        self.assertIsNone(data['feedback_near_zero'])
        self.plugin.driver.arm.frozen = False
        with patch.object(self.plugin.driver, 'factory_zero_status', side_effect=RuntimeError('CAN lost')):
            node.callback()
            self.assertEqual(node.messages[-1]['reason'], 'CAN lost')
            self.assertIsNone(node.messages[-1]['within_tolerance'])
        node.callback()
        self.assertTrue(node.messages[-1]['fresh'])
        with patch('agilex.piper.zero_stream.time.time', return_value=time.time() + 1):
            self.assertFalse(self.call('info')['data']['fresh'])
            self.assertIsNone(self.call('info')['data']['joints_deg'])

    def test_read_and_stop_do_not_cancel_active_motion_or_invalidate_preview(self):
        node = self.start_stream()
        node.callback()
        sdk = self.plugin.driver.arm
        with self.plugin.operation('piper_joint', 'move'):
            self.plugin.cancel.set()
            self.plugin.driver.cancelled.set()
            with patch.object(self.plugin.previews, 'invalidate') as invalidated:
                node.callback()
                self.assertTrue(node.messages[-1]['fresh'])
                self.assertTrue(self.plugin.cancel.is_set())
                self.assertTrue(self.plugin.driver.cancelled.is_set())
                self.assertEqual(self.call('stop')['state'], 'idle')
                invalidated.assert_not_called()
            self.assertTrue(self.plugin.gate.locked())
            self.assertEqual(self.plugin.active_card, 'piper_joint')
        self.assertEqual(sdk.calls, [])
        self.assertTrue(node.destroyed)
        self.assertFalse(node.messages[-1]['fresh'])
        count = len(node.messages)
        node.callback()
        self.assertEqual(len(node.messages), count)
        self.assertIsNotNone(self.plugin.driver)
        self.call('start')
        self.assertIsNot(self.plugin.zero_stream.node, node)

    def test_invalid_tolerance_is_rejected_before_hardware_or_ros(self):
        for value in [True, '2', float('nan'), 0, 5.1]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.call('start', tolerance_deg=value)
        self.assertIsNone(self.plugin.driver)
        self.ros.executor_core.add_node.assert_not_called()

    def test_no_ros_fails_explicitly_but_single_read_still_works(self):
        self.plugin.zero_stream.ros2 = None
        with self.assertRaisesRegex(RuntimeError, 'ROS2'):
            self.call('start')
        self.assertEqual(self.call('info')['state'], 'idle')
        self.assertIsNone(self.plugin.driver)
        self.assertTrue(self.call('read')['fresh'])

    def test_shutdown_releases_node_even_when_final_publish_fails(self):
        node = self.start_stream()
        with patch.object(node.publisher, 'publish', side_effect=RuntimeError('DDS stopped')):
            self.plugin.stop()
        self.assertTrue(node.destroyed)
        self.assertTrue(self.plugin.closed)
        self.assertIsNone(self.plugin.zero_stream.node)

    def test_node_creation_failure_cleans_partial_node(self):
        node = Node()
        with patch.object(sys.modules['rclpy.node'], 'Node', return_value=node), patch.object(node, 'create_timer', side_effect=RuntimeError('timer')):
            with self.assertRaisesRegex(RuntimeError, 'timer'):
                self.call('start')
        self.assertTrue(node.destroyed)
        self.assertIsNone(self.plugin.zero_stream.node)
        self.assertIsNone(self.plugin.driver)
