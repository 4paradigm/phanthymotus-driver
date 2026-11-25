"""No-hardware tests for the AS2W lidar obstacle-monitor contract."""
import array
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).parent
SPEC = importlib.util.spec_from_file_location("as2w_obstacle_monitor_test", ROOT / "obstacle_monitor.py")
monitor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(monitor)


def cloud(*points):
    return struct.pack("<II", 12, len(points)) + b"".join(
        struct.pack("<fff", *point) for point in points)


class Clock:
    def __init__(self):
        self.elapsed = 0.0

    def monotonic(self):
        return self.elapsed

    def wall(self):
        return 1700000000.0 + self.elapsed


class TestObstacleMonitorCore(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = {"lidar_offset_x_m": 0, "lidar_offset_y_m": 0,
                       "lidar_offset_z_m": 0}
        self.core = monitor.ObstacleMonitorCore(self.config, self.clock.monotonic,
                                                self.clock.wall)

    def test_six_regions_and_consecutive_frame_confirmation(self):
        # Wire coordinates are (-base_z, base_x, -base_y).
        packet = cloud((0, 1, 0), (0, 1, -1), (0, 1, 1),
                       *((0, 0, -2),) * 3, *((0, 0, 2),) * 3,
                       *((0, -2, 0),) * 3)
        self.core.consume(packet)
        first = self.core.snapshot()
        self.assertEqual("unknown", first["obstacles"]["front"]["status"])
        self.assertEqual(1.0, first["obstacles"]["front"]["distance_m"])
        self.core.consume(packet)
        result = self.core.snapshot()
        self.assertEqual("healthy", result["sensor_status"])
        self.assertEqual("warning", result["overall_status"])
        for name in ("front", "front_left", "front_right"):
            self.assertEqual("warning", result["obstacles"][name]["status"])
        for name in ("left", "right", "rear"):
            self.assertEqual("clear", result["obstacles"][name]["status"])
            self.assertEqual(3, result["obstacles"][name]["point_count"])
        self.assertAlmostEqual(math.sqrt(2), result["obstacles"]["front_left"]["distance_m"], places=3)

    def test_mount_translation_is_applied_after_wire_axis_remapping(self):
        core = monitor.ObstacleMonitorCore({"confirm_frames": 1}, self.clock.monotonic,
                                           self.clock.wall)
        core.consume(cloud((0, 1, -0.25)))
        result = core.snapshot()["obstacles"]["front"]
        self.assertAlmostEqual(math.hypot(1.27666, 0.25), result["distance_m"], places=3)
        self.assertEqual("warning", result["status"])

    def test_padded_point_records_follow_declared_point_step(self):
        core = monitor.ObstacleMonitorCore({**self.config, "confirm_frames": 1},
                                           self.clock.monotonic, self.clock.wall)
        packet = (struct.pack("<II", 16, 2) + struct.pack("<fff", 0, 1, 0) + b"PAD1"
                  + struct.pack("<fff", 0, -2, 0) + b"PAD2")
        core.consume(packet)
        result = core.snapshot()
        self.assertEqual(1.0, result["obstacles"]["front"]["distance_m"])
        self.assertEqual(2.0, result["obstacles"]["rear"]["distance_m"])

    def test_empty_or_under_sampled_sector_is_unknown_not_clear(self):
        core = monitor.ObstacleMonitorCore({**self.config, "confirm_frames": 1},
                                           self.clock.monotonic, self.clock.wall)
        core.consume(cloud((0, 3, 0)))
        result = core.snapshot()
        self.assertEqual("unknown", result["obstacles"]["front"]["status"])
        self.assertEqual(3.0, result["obstacles"]["front"]["distance_m"])
        self.assertIsNone(result["obstacles"]["rear"]["distance_m"])
        self.assertEqual("unknown", result["overall_status"])

    def test_near_point_does_not_need_clear_coverage_count(self):
        core = monitor.ObstacleMonitorCore({**self.config, "confirm_frames": 1},
                                           self.clock.monotonic, self.clock.wall)
        core.consume(cloud((0, 0.25, 0)))
        result = core.snapshot()
        self.assertEqual("critical", result["obstacles"]["front"]["status"])
        self.assertEqual("critical", result["overall_status"])

    def test_a_new_close_observation_does_not_leave_previous_clear_status(self):
        distant = cloud((0, 3, 0), (0, 3.1, 0), (0, 3.2, 0))
        self.core.consume(distant)
        self.core.consume(distant)
        self.assertEqual("clear", self.core.snapshot()["obstacles"]["front"]["status"])
        self.core.consume(cloud((0, 0.25, 0)))
        self.assertEqual("unknown", self.core.snapshot()["obstacles"]["front"]["status"])
        self.core.consume(cloud((0, 0.25, 0)))
        self.assertEqual("critical", self.core.snapshot()["obstacles"]["front"]["status"])

    def test_filtered_and_nonfinite_points_do_not_become_obstacles(self):
        core = monitor.ObstacleMonitorCore({**self.config, "confirm_frames": 1},
                                           self.clock.monotonic, self.clock.wall)
        # First point is below the height window; the second is non-finite.
        core.consume(cloud((0.5, 1, 0), (0, float("nan"), 0)))
        result = core.snapshot()
        self.assertEqual("healthy", result["sensor_status"])
        self.assertEqual("unknown", result["obstacles"]["front"]["status"])
        self.assertIsNone(result["obstacles"]["front"]["distance_m"])

    def test_bad_or_empty_frames_invalidate_previous_observations(self):
        core = monitor.ObstacleMonitorCore({**self.config, "confirm_frames": 1},
                                           self.clock.monotonic, self.clock.wall)
        core.consume(cloud((0, 0.25, 0)))
        for bad in (b"", struct.pack("<II", 8, 1) + b"12345678",
                    struct.pack("<II", 12, 2) + struct.pack("<fff", 0, 1, 0),
                    cloud(), cloud((0, 0, 0)), cloud((float("nan"), 1, 0))):
            core.consume(bad)
            result = core.snapshot()
            self.assertEqual("invalid", result["sensor_status"])
            self.assertEqual("unknown", result["overall_status"])
            self.assertIsNone(result["obstacles"]["front"]["distance_m"])

    def test_timeout_and_recovery_restart_confirmation(self):
        self.core.consume(cloud((0, 1, 0)))
        self.core.consume(cloud((0, 1, 0)))
        self.assertEqual("warning", self.core.snapshot()["obstacles"]["front"]["status"])
        self.clock.elapsed = 2.0
        expired = self.core.snapshot()
        self.assertEqual("stale", expired["sensor_status"])
        self.assertEqual(2000, expired["receive_age_ms"])
        self.assertIsNone(expired["source_timestamp"])
        self.assertEqual("unverified", expired["geometry_status"])
        self.assertEqual("unknown", expired["overall_status"])
        self.core.consume(cloud((0, 1, 0)))
        self.assertEqual("unknown", self.core.snapshot()["obstacles"]["front"]["status"])
        self.core.consume(cloud((0, 1, 0)))
        self.assertEqual("warning", self.core.snapshot()["obstacles"]["front"]["status"])

    def test_invalid_config_is_rejected(self):
        for config in ({"critical_distance_m": 2, "warning_distance_m": 1},
                       {"timeout_s": 0}, {"confirm_frames": 0},
                       {"min_height_m": 1, "max_height_m": 0},
                       {"warning_distance_m": float("nan")},
                       {"min_clear_points": True}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                monitor.ObstacleMonitorCore(config)


class TestObstacleMonitorPlugin(unittest.TestCase):
    def test_ros_input_output_and_lifecycle(self):
        class Message:
            pass

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message.data)

        class FakeNode:
            instances = []

            def __init__(self, name):
                self.name = name
                self.destroyed = False
                self.instances.append(self)

            def create_publisher(self, kind, topic, qos):
                self.out_topic, self.out_qos = topic, qos
                self.publisher = FakePublisher()
                return self.publisher

            def create_subscription(self, kind, topic, callback, qos):
                self.in_topic, self.in_qos = topic, qos
                self.callback = callback
                return object()

            def create_timer(self, interval, callback):
                self.interval, self.timer = interval, callback
                return object()

            def destroy_node(self):
                self.destroyed = True

        class FakeExecutor:
            def __init__(self):
                self.nodes = []

            def add_node(self, node):
                self.nodes.append(node)

            def remove_node(self, node):
                self.nodes.remove(node)

        node_mod = types.ModuleType("rclpy.node")
        node_mod.Node = FakeNode
        qos_mod = types.ModuleType("rclpy.qos")
        qos_mod.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
        qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
        qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")
        qos_mod.QoSProfile = lambda **kwargs: kwargs
        messages = types.ModuleType("std_msgs.msg")
        messages.String = messages.UInt8MultiArray = Message
        modules = {"rclpy.node": node_mod, "rclpy.qos": qos_mod,
                   "std_msgs.msg": messages}
        executor = FakeExecutor()
        with patch.dict(sys.modules, modules):
            plugin = monitor.ObstacleMonitorPlugin(
                {"confirm_frames": 1, "lidar_offset_x_m": 0,
                 "lidar_offset_y_m": 0, "lidar_offset_z_m": 0}, "dog", executor)
            tool = plugin.get_tool()
            self.assertEqual("sensor", tool["type"])
            self.assertEqual("/dog/obstacle/monitor", tool["topic_out"][0]["topic"])
            self.assertEqual("no_data", plugin.dispatch("start", {})["sensor_status"])
            node = executor.nodes[0]
            self.assertEqual("/dog/lidar/cloud", node.in_topic)
            self.assertEqual("best_effort", node.in_qos["reliability"])
            message = Message()
            message.data = array.array("B", cloud((0, 1, 0)))
            node.callback(message)
            node.timer()
            self.assertEqual("warning", json.loads(node.publisher.messages[-1])["overall_status"])
            self.assertEqual("warning", plugin.dispatch("read", {})["overall_status"])
            self.assertEqual("not_running", plugin.dispatch("stop", {})["sensor_status"])
            self.assertTrue(node.destroyed)
            self.assertEqual([], executor.nodes)
            plugin.dispatch("start", {})
            self.assertEqual(1, len(executor.nodes))
            self.assertEqual("no_data", plugin.dispatch("read", {})["sensor_status"])
            plugin.stop()
            with patch.object(FakeNode, "create_subscription", side_effect=RuntimeError("subscribe failed")):
                with self.assertRaisesRegex(RuntimeError, "subscribe failed"):
                    plugin.start()
            self.assertTrue(FakeNode.instances[-1].destroyed)
            self.assertEqual([], executor.nodes)


if __name__ == "__main__":
    unittest.main()
