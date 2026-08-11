import ast
import importlib
from pathlib import Path
import sys
import types
import unittest


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
        ast.parse(source)

    def test_default_config_uses_mid360_raw_dds_and_native_ros_topics(self):
        config = (G1_DIR / "config.yaml").read_text()
        for expected in (
            "navigation_sensors:\n    enabled: true",
            "raw_cloud_topic: rt/utlidar/cloud_livox_mid360",
            "raw_imu_topic: rt/utlidar/imu_livox_mid360",
            "fast_livo_cloud_topic: /ubuntu/navigation/lidar_fast_livo",
            "imu_topic: /ubuntu/navigation/imu",
            "publish_raw_cloud: false",
            "publish_fast_livo_cloud: true",
        ):
            self.assertIn(expected, config)

    def test_tools_declare_native_types_qos_and_fail_closed_status(self):
        source = (G1_DIR / "navigation_sensor_bridge.py").read_text()
        for expected in (
            '"navigation_lidar_fast_livo"',
            '"navigation_imu"',
            '"navigation_sensor_diagnostics"',
            '"sensor/pointcloud"',
            '"sensor_msgs/msg/PointCloud2"',
            '"sensor_msgs/msg/Imu"',
            '"RELIABLE + KEEP_LAST(depth=2) + VOLATILE"',
            '"RELIABLE + KEEP_LAST(depth=200) + VOLATILE"',
            '"state": "ready" if status["ready"] else "not_ready"',
            'blockers.append("clock_not_ready")',
            'blockers.append("cloud_stale")',
            'blockers.append("imu_stale")',
        ):
            self.assertIn(expected, source)
        self.assertNotIn('"sensor/pointcloud2"', source)
        ast.parse(source)

    def test_runtime_tool_descriptors_match_fast_livo2_inputs(self):
        module = self.load_bridge_module()
        plugin = module.NavigationSensorPlugin.__new__(module.NavigationSensorPlugin)
        plugin._node = types.SimpleNamespace(
            _publish_fast_livo_cloud=True,
            _publish_raw_cloud=False,
            fast_livo_cloud_topic="/ubuntu/navigation/lidar_fast_livo",
            cloud_topic="/ubuntu/navigation/lidar",
            imu_topic="/ubuntu/navigation/imu",
            diagnostics_topic="/ubuntu/navigation/sensor_diagnostics",
            status=lambda: {
                "ready": False,
                "blockers": ["clock_not_ready"],
                "receive_age_ms": {"cloud": None, "imu": None},
                "clock": {"ready": False},
                "counters": {},
            },
        )

        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        lidar = tools["navigation_lidar_fast_livo"]["topic_out"][0]
        imu = tools["navigation_imu"]["topic_out"][0]
        self.assertEqual(lidar["format"], "sensor/pointcloud")
        self.assertEqual(lidar["ros_type"], "sensor_msgs/msg/PointCloud2")
        self.assertEqual(lidar["qos"], "RELIABLE + KEEP_LAST(depth=2) + VOLATILE")
        self.assertEqual(imu["format"], "sensor/imu")
        self.assertEqual(imu["ros_type"], "sensor_msgs/msg/Imu")
        self.assertEqual(imu["qos"], "RELIABLE + KEEP_LAST(depth=200) + VOLATILE")

        info = plugin.dispatch(
            "info", {"_tool_name": "navigation_lidar_fast_livo"}
        )
        self.assertEqual(info["state"], "not_ready")
        self.assertEqual(info["blockers"], ["clock_not_ready"])

    def test_driver_image_contains_the_sensor_card_runtime(self):
        dockerfile = (G1_DIR / "Dockerfile").read_text()
        for filename in (
            "navigation_sensor_bridge.py",
            "navigation_pointcloud.py",
            "navigation_time.py",
        ):
            self.assertIn(f"COPY {filename} /work/{filename}", dockerfile)


if __name__ == "__main__":
    unittest.main()
