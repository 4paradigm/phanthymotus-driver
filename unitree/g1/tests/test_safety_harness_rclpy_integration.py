"""Regression tests for SmartMotion's single-owner rclpy executor."""

import ast
import threading
import time
import unittest
from pathlib import Path


HARNESS = Path(__file__).resolve().parents[1] / "safety_harness.py"

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None


@unittest.skipUnless(rclpy is not None, "requires ROS2 rclpy and std_msgs")
class SafetyHarnessRclpyIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_executor_is_spun_only_by_main_loop(self):
        tree = ast.parse(HARNESS.read_text())
        spin_calls = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "spin_once"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "executor"
        ]
        self.assertEqual(len(spin_calls), 1)

    def test_waiter_can_observe_ros_callback_without_spinning(self):
        executor = SingleThreadedExecutor()
        publisher_node = Node("safety_harness_test_publisher")
        subscriber_node = Node("safety_harness_test_subscriber")
        received = threading.Event()
        callback_threads = []

        def on_message(_message):
            callback_threads.append(threading.get_ident())
            received.set()

        subscriber_node.create_subscription(String, "safety_harness_test", on_message, 10)
        publisher = publisher_node.create_publisher(String, "safety_harness_test", 10)
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        spin_finished = threading.Event()
        main_loop_thread_id = []

        def main_loop():
            main_loop_thread_id.append(threading.get_ident())
            deadline = time.monotonic() + 2
            while not received.is_set() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            spin_finished.set()

        def waiter():
            while not received.is_set() and not spin_finished.is_set():
                time.sleep(0.01)

        try:
            loop_thread = threading.Thread(target=main_loop)
            waiter_thread = threading.Thread(target=waiter)
            loop_thread.start()
            waiter_thread.start()
            time.sleep(0.1)
            message = String()
            message.data = "arrived"
            publisher.publish(message)
            loop_thread.join(timeout=3)
            waiter_thread.join(timeout=3)

            self.assertTrue(received.is_set())
            self.assertFalse(waiter_thread.is_alive())
            self.assertEqual(callback_threads, main_loop_thread_id)
        finally:
            executor.shutdown()
            publisher_node.destroy_node()
            subscriber_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
