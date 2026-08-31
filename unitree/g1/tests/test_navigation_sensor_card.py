import ast
import importlib
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]


class NavigationSensorCardContractTest(unittest.TestCase):
    @staticmethod
    def load_bridge_module():
        if not hasattr(sys.modules.get("numpy"), "dtype"):
            sys.modules.pop("numpy", None)

        rclpy = sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = type("Node", (), {})
        rclpy_qos = types.ModuleType("rclpy.qos")

        class QoSProfile:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        policy = type(
            "Policy",
            (),
            {
                "BEST_EFFORT": "best_effort",
                "RELIABLE": "reliable",
                "KEEP_LAST": "keep_last",
                "VOLATILE": "volatile",
            },
        )
        rclpy_qos.QoSProfile = QoSProfile
        rclpy_qos.ReliabilityPolicy = policy
        rclpy_qos.HistoryPolicy = policy
        rclpy_qos.DurabilityPolicy = policy
        sys.modules["rclpy.node"] = rclpy_node
        sys.modules["rclpy.qos"] = rclpy_qos
        rclpy.node = rclpy_node
        rclpy.qos = rclpy_qos

        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        for name in ("Imu", "PointCloud2", "PointField"):
            setattr(sensor_msgs_msg, name, type(name, (), {}))
        sensor_msgs.msg = sensor_msgs_msg
        sys.modules["sensor_msgs"] = sensor_msgs
        sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

        class TransformStamped:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id="")
                self.child_frame_id = ""
                self.transform = types.SimpleNamespace(
                    translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
                )

        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TransformStamped = TransformStamped
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules["geometry_msgs"] = geometry_msgs
        sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

        tf2_ros = types.ModuleType("tf2_ros")
        tf2_static = types.ModuleType("tf2_ros.static_transform_broadcaster")
        tf2_static.StaticTransformBroadcaster = type(
            "StaticTransformBroadcaster", (), {}
        )
        tf2_ros.static_transform_broadcaster = tf2_static
        sys.modules["tf2_ros"] = tf2_ros
        sys.modules["tf2_ros.static_transform_broadcaster"] = tf2_static

        std_msgs = sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
        std_msgs_msg = sys.modules.setdefault(
            "std_msgs.msg", types.ModuleType("std_msgs.msg")
        )
        std_msgs_msg.String = type("String", (), {})
        std_msgs.msg = std_msgs_msg

        channel = types.ModuleType("unitree_sdk2py.core.channel")
        channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
        idl = types.ModuleType("unitree_sdk2py.idl.sensor_msgs.msg.dds_")
        idl.Imu_ = type("Imu_", (), {})
        idl.PointCloud2_ = type("PointCloud2_", (), {})
        sys.modules["unitree_sdk2py.core.channel"] = channel
        sys.modules["unitree_sdk2py.idl.sensor_msgs.msg.dds_"] = idl

        sys.modules.pop("navigation_sensor_bridge", None)
        return importlib.import_module("navigation_sensor_bridge")

    def test_bundle_registers_the_read_only_sensor_plugin(self):
        source = (G1_DIR / "main.py").read_text()
        self.assertIn('plugins_cfg.get("navigation_sensors"', source)
        self.assertIn("NavigationSensorPlugin", source)
        self.assertIn("network_iface,", source)
        ast.parse(source)

    def test_default_config_uses_mid360_raw_dds_and_native_ros_topics(self):
        config = (G1_DIR / "config.yaml").read_text()
        for expected in (
            "navigation_sensors:\n    enabled: true",
            "raw_cloud_topic: rt/utlidar/cloud_livox_mid360",
            "raw_imu_topic: rt/utlidar/imu_livox_mid360",
            "cloud_topic: /ubuntu/navigation/lidar",
            "imu_topic: /ubuntu/navigation/imu",
            "base_frame: base_link",
            "base_to_sensor_translation_m: [-0.00368, 0.00003, 0.46018]",
            "base_to_sensor_rotation_rpy_rad: [0.0, 0.04014257279586953, 0.0]",
        ):
            self.assertIn(expected, config)

    def test_tools_declare_native_types_qos_and_fail_closed_status(self):
        source = (G1_DIR / "navigation_sensor_bridge.py").read_text()
        for expected in (
            '"navigation_lidar"',
            '"navigation_imu"',
            '"sensor/pointcloud"',
            '"sensor_msgs/msg/PointCloud2"',
            '"sensor_msgs/msg/Imu"',
            '"RELIABLE + KEEP_LAST(depth=2) + VOLATILE"',
            '"RELIABLE + KEEP_LAST(depth=200) + VOLATILE"',
            'state = "running" if worker_running else "error"',
            'blockers.append("clock_not_ready")',
            'blockers.append("cloud_stale")',
            'blockers.append("imu_stale")',
            'blockers.append("worker_not_running")',
            'subprocess.Popen(',
        ):
            self.assertIn(expected, source)
        self.assertNotIn('"sensor/pointcloud2"', source)
        ast.parse(source)

    def test_runtime_exposes_only_generic_navigation_sensor_tools(self):
        module = self.load_bridge_module()
        plugin = module.NavigationSensorPlugin.__new__(module.NavigationSensorPlugin)
        plugin._status_node = types.SimpleNamespace(
            cloud_topic="/ubuntu/navigation/lidar",
            imu_topic="/ubuntu/navigation/imu",
            lidar_frame="custom_lidar_frame",
            imu_frame="custom_imu_frame",
            status=lambda worker_running: {
                "ready": False,
                "blockers": ["clock_not_ready"],
                "receive_age_ms": {"cloud": None, "imu": None},
                "clock": {"ready": False},
                "counters": {},
            },
        )
        plugin._proc = types.SimpleNamespace(poll=lambda: None, pid=1234)

        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        self.assertEqual(set(tools), {"navigation_lidar", "navigation_imu"})
        lidar = tools["navigation_lidar"]["topic_out"][0]
        imu = tools["navigation_imu"]["topic_out"][0]
        self.assertEqual(lidar["format"], "sensor/pointcloud")
        self.assertEqual(lidar["ros_type"], "sensor_msgs/msg/PointCloud2")
        self.assertEqual(lidar["qos"], "RELIABLE + KEEP_LAST(depth=2) + VOLATILE")
        self.assertEqual(lidar["frame_id"], "custom_lidar_frame")
        self.assertEqual(imu["format"], "sensor/imu")
        self.assertEqual(imu["ros_type"], "sensor_msgs/msg/Imu")
        self.assertEqual(imu["qos"], "RELIABLE + KEEP_LAST(depth=200) + VOLATILE")
        self.assertEqual(imu["frame_id"], "custom_imu_frame")

        info = plugin.dispatch("info", {"_tool_name": "navigation_lidar"})
        self.assertEqual(info["state"], "not_ready")
        self.assertEqual(info["blockers"], ["clock_not_ready"])

    def test_monitor_preserves_configured_sensor_frames(self):
        module = self.load_bridge_module()
        with mock.patch.object(module.Node, "__init__", return_value=None), mock.patch.object(
            module.Node,
            "create_subscription",
            return_value=object(),
            create=True,
        ):
            monitor = module._NavigationSensorMonitorNode(
                {
                    "lidar_frame": "configured_lidar_frame",
                    "imu_frame": "configured_imu_frame",
                },
                "ubuntu",
            )

        self.assertEqual(monitor.lidar_frame, "configured_lidar_frame")
        self.assertEqual(monitor.imu_frame, "configured_imu_frame")

    def test_worker_publishes_required_base_to_livox_static_transform(self):
        module = self.load_bridge_module()
        config = {
            "base_frame": "base_link",
            "base_to_sensor_translation_m": [-0.00368, 0.00003, 0.46018],
            "base_to_sensor_rotation_rpy_rad": [0.0, 0.04014257279586953, 0.0],
        }
        base_frame, translation, rpy = module._required_static_transform(
            config, "livox_frame"
        )
        node = module._NavigationSensorNode.__new__(module._NavigationSensorNode)
        node._base_frame = base_frame
        node._lidar_frame = "livox_frame"
        node._base_to_sensor_translation = translation
        node._base_to_sensor_rpy = rpy
        stamp = object()
        node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: stamp)
        )
        broadcaster = mock.Mock()

        with mock.patch.object(
            module, "StaticTransformBroadcaster", return_value=broadcaster
        ):
            node._publish_static_transform()

        transform = broadcaster.sendTransform.call_args.args[0]
        self.assertIs(transform.header.stamp, stamp)
        self.assertEqual(transform.header.frame_id, "base_link")
        self.assertEqual(transform.child_frame_id, "livox_frame")
        self.assertEqual(
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            translation,
        )
        quaternion = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)

        for invalid in (
            {},
            {**config, "base_frame": "livox_frame"},
            {**config, "base_to_sensor_translation_m": [0.0, 0.0]},
            {**config, "base_to_sensor_rotation_rpy_rad": [0.0, float("nan"), 0.0]},
        ):
            with self.assertRaises(ValueError):
                module._required_static_transform(invalid, "livox_frame")

    def test_driver_image_contains_the_sensor_card_runtime(self):
        dockerfile = (G1_DIR / "Dockerfile").read_text()
        for filename in (
            "navigation_sensor_bridge.py",
            "navigation_sensor_bridge_main.py",
            "navigation_pointcloud.py",
            "navigation_time.py",
        ):
            self.assertIn(f"COPY {filename} /work/{filename}", dockerfile)

    def test_worker_entry_owns_the_heavy_sensor_node(self):
        source = (G1_DIR / "navigation_sensor_bridge_main.py").read_text()
        self.assertIn("_NavigationSensorNode(plugin_config, namespace)", source)
        self.assertIn("ChannelFactoryInitialize(0, network_interface)", source)
        self.assertLess(
            source.index("logsafe.install(check_fd=False)"),
            source.index("import rclpy"),
        )
        self.assertNotIn("NavigationSensorPlugin(", source)
        ast.parse(source)

    def test_plugin_starts_and_stops_one_isolated_worker(self):
        module = self.load_bridge_module()
        plugin = module.NavigationSensorPlugin.__new__(module.NavigationSensorPlugin)
        plugin._namespace = "ubuntu"
        plugin._network_iface = "eth0"
        plugin._worker_path = G1_DIR / "navigation_sensor_bridge_main.py"
        plugin._proc = None
        plugin._executor = mock.Mock()
        plugin._status_node = mock.Mock()
        proc = mock.Mock(pid=4321)
        proc.poll.return_value = None

        with mock.patch.object(module.subprocess, "Popen", return_value=proc) as popen:
            plugin.start()
            plugin.start()

        popen.assert_called_once()
        self.assertEqual(
            popen.call_args.args[0],
            [sys.executable, str(plugin._worker_path), "eth0"],
        )
        plugin.stop()
        proc.terminate.assert_called_once_with()
        proc.wait.assert_called_once_with(timeout=5.0)
        self.assertIsNone(plugin._proc)
        plugin._executor.remove_node.assert_called_once_with(plugin._status_node)
        plugin._status_node.destroy_node.assert_called_once_with()

    def test_either_card_stop_terminates_shared_worker_and_allows_restart(self):
        module = self.load_bridge_module()

        for tool_name in ("navigation_lidar", "navigation_imu"):
            with self.subTest(tool_name=tool_name):
                plugin = module.NavigationSensorPlugin.__new__(
                    module.NavigationSensorPlugin
                )
                plugin._namespace = "ubuntu"
                plugin._network_iface = "eth0"
                plugin._worker_path = G1_DIR / "navigation_sensor_bridge_main.py"
                plugin._proc = None
                plugin._status_node = types.SimpleNamespace(
                    cloud_topic="/ubuntu/navigation/lidar",
                    imu_topic="/ubuntu/navigation/imu",
                    lidar_frame="livox_frame",
                    imu_frame="livox_frame",
                    status=lambda running: {
                        "ready": False,
                        "blockers": ["clock_not_ready"],
                    },
                )
                first = mock.Mock(pid=1001)
                second = mock.Mock(pid=1002)
                first.poll.return_value = None
                second.poll.return_value = None

                with mock.patch.object(
                    module.subprocess,
                    "Popen",
                    side_effect=(first, second),
                ) as popen:
                    plugin.start()
                    stopped = plugin.dispatch("stop", {"_tool_name": tool_name})
                    stopped_again = plugin.dispatch(
                        "stop", {"_tool_name": tool_name}
                    )
                    restarted = plugin.dispatch(
                        "start", {"_tool_name": tool_name}
                    )

                self.assertEqual(
                    stopped,
                    {"state": "idle", "worker_running": False, "worker_pid": None},
                )
                self.assertEqual(stopped_again, stopped)
                first.terminate.assert_called_once_with()
                first.wait.assert_called_once_with(timeout=5.0)
                self.assertEqual(popen.call_count, 2)
                self.assertEqual(restarted["state"], "running")
                self.assertFalse(restarted["ready"])
                self.assertEqual(restarted["blockers"], ["clock_not_ready"])
                self.assertEqual(restarted["worker_pid"], 1002)

    def test_status_monitor_fails_closed_for_dead_or_stale_worker(self):
        module = self.load_bridge_module()
        node = module._NavigationSensorMonitorNode.__new__(
            module._NavigationSensorMonitorNode
        )
        node._lock = threading.RLock()
        node._last_status = {"ready": True, "blockers": []}
        node._last_status_monotonic = time.monotonic() - 3.0

        stale = node.status(worker_running=True)
        self.assertFalse(stale["ready"])
        self.assertIn("status_stale", stale["blockers"])

        dead = node.status(worker_running=False)
        self.assertFalse(dead["ready"])
        self.assertIn("worker_not_running", dead["blockers"])

    def test_invalid_timestamp_warnings_are_sampled_per_stream(self):
        module = self.load_bridge_module()
        node = module._NavigationSensorNode.__new__(module._NavigationSensorNode)
        node._counters = {
            "cloud_invalid_timestamps": 0,
            "imu_invalid_timestamps": 0,
        }
        logger = mock.Mock()
        node.get_logger = lambda: logger

        for _ in range(201):
            self.assertIsNone(node._correct_stamp(object(), "imu"))
        self.assertIsNone(node._correct_stamp(object(), "cloud"))

        self.assertEqual(node._counters["imu_invalid_timestamps"], 201)
        self.assertEqual(node._counters["cloud_invalid_timestamps"], 1)
        self.assertEqual(logger.warning.call_count, 4)
        messages = [call.args[0] for call in logger.warning.call_args_list]
        self.assertIn("invalid imu timestamp count=1", messages[0])
        self.assertIn("invalid imu timestamp count=100", messages[1])
        self.assertIn("invalid imu timestamp count=200", messages[2])
        self.assertIn("invalid cloud timestamp count=1", messages[3])

    def test_duplicate_imu_timestamps_are_strictly_increasing(self):
        module = self.load_bridge_module()
        node = module._NavigationSensorNode.__new__(module._NavigationSensorNode)
        node._clock_offset = mock.Mock()
        node._clock_offset.correct_observation.return_value = 123456789
        node._last_stamp_ns = {"cloud": 0, "imu": 0}
        node._stamp_lock = threading.Lock()
        node._counters = {
            "imu_invalid_timestamps": 0,
            "stamp_clamped": 0,
        }
        node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=987654321)
        )
        stamp = types.SimpleNamespace(sec=1, nanosec=2)

        first = node._correct_stamp(stamp, "imu")
        second = node._correct_stamp(stamp, "imu")

        self.assertEqual(second, first + 1)
        self.assertEqual(node._counters["stamp_clamped"], 1)


if __name__ == "__main__":
    unittest.main()
