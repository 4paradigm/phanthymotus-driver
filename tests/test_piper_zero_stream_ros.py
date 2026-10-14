"""Opt-in real ROS2 transport test with a simulated arm and an isolated DDS domain."""
import json
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

from agilex.piper.device import PiperPlugin
from common.vendor_runtime import DriverBundle
from test_piper_controls import Driver


@unittest.skipUnless(os.environ.get('PIPER_TEST_ROS2') == '1', 'set PIPER_TEST_ROS2=1 with ROS2 installed')
class ZeroStreamROSTests(unittest.TestCase):
    def test_real_timer_and_dds_deliver_changing_angles_until_stop(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String

        context = Context()
        rclpy.init(context=context, domain_id=221)
        self.addCleanup(context.shutdown)
        executor = SingleThreadedExecutor(context=context)
        self.addCleanup(executor.shutdown)
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        plugin = PiperPlugin({'data_dir': temp.name}, namespace='piper_stream_test',
                             ros2=SimpleNamespace(ctx_core=context, executor_core=executor),
                             driver_factory=Driver)
        self.addCleanup(plugin.stop)
        plugin.motion_enabled = False
        bundle = DriverBundle([plugin])
        received = []

        def receive(msg):
            data = json.loads(msg.data)
            received.append(data)
            if data['fresh']:
                plugin.driver.arm.raw = (round((data['joints_deg'][0] + 1) * 1000), 0, 0, 0, 0, 0)

        subscriber = Node('piper_zero_test_subscriber', context=context)
        self.addCleanup(subscriber.destroy_node)
        subscriber.create_subscription(String, '/piper_stream_test/piper/zero_check', receive, 1)
        executor.add_node(subscriber)
        self.addCleanup(executor.remove_node, subscriber)
        result = bundle.dispatch('piper_zero_check', {'action': 'start'})
        self.assertEqual(result['state'], 'running')
        deadline = time.monotonic() + 8
        while len(received) < 4 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        self.assertGreaterEqual(len(received), 4)
        self.assertTrue(all(frame['fresh'] for frame in received))
        self.assertGreater(len({frame['joints_deg'][0] for frame in received}), 1)
        self.assertGreater(received[-1]['timestamp_ms'], received[0]['timestamp_ms'])
        self.assertEqual(plugin.driver.arm.calls, [])
        self.assertFalse(plugin.motion_enabled)
        bundle.dispatch('piper_zero_check', {'action': 'stop'})
        deadline = time.monotonic() + 2
        while received[-1]['fresh'] and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        self.assertFalse(received[-1]['fresh'])
        count = len(received)
        deadline = time.monotonic() + .6
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        self.assertEqual(len(received), count)
        print(f'ROS2 received {count - 1} live frames and a stopped frame; domain 221, simulated SDK')
