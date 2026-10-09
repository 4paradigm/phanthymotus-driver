"""No-hardware checks for the lidar subprocess's robot DDS binding."""
import importlib.util
import os
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


ROOT = Path(__file__).parent


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _runtime_stubs():
    modules = {name: types.ModuleType(name) for name in (
        "yaml", "rclpy", "rclpy.executors", "rclpy.qos", "std_msgs",
        "std_msgs.msg", "unitree_sdk2py", "unitree_sdk2py.core",
        "unitree_sdk2py.core.channel", "unitree_sdk2py.idl",
        "unitree_sdk2py.idl.sensor_msgs", "unitree_sdk2py.idl.sensor_msgs.msg",
        "unitree_sdk2py.idl.sensor_msgs.msg.dds_", "rpc_proxy",
        "device", "multimedia", "controlled_spatial", "slam_mapping",
    )}
    ros = modules["rclpy"]
    ros.init, ros.shutdown = Mock(), Mock()
    ros.executors = modules["rclpy.executors"]
    ros.executors.SingleThreadedExecutor = Mock()
    qos = modules["rclpy.qos"]
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.QoSProfile = lambda **kwargs: kwargs
    modules["std_msgs.msg"].UInt8MultiArray = type("UInt8MultiArray", (), {})
    channel = modules["unitree_sdk2py.core.channel"]
    channel.ChannelSubscriber = Mock()
    channel.ChannelFactoryInitialize = Mock()
    modules["unitree_sdk2py.idl.sensor_msgs.msg.dds_"].PointCloud2_ = type("PointCloud2_", (), {})
    modules["rpc_proxy"].RpcProxy = Mock()
    for module_name, classes in {
        "device": ("LedPlugin", "StatePlugin", "LocoPlugin", "SpecialMotionPlugin"),
        "multimedia": ("CameraPlugin", "MicPlugin", "SpeakerPlugin"),
        "controlled_spatial": ("ControlledSpatialPlugin",),
        "slam_mapping": ("SlamMappingPlugin",),
    }.items():
        for name in classes:
            setattr(modules[module_name], name, type(name, (), {}))
    return modules


class LidarInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.modules = _runtime_stubs()
        self.module_patch = patch.dict(sys.modules, self.modules)
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.lidar = _load("as2w_lidar_interface_test", "lidar.py")
        self.worker = _load("as2w_lidar_worker_test", "sensor_worker.py")
        sys.modules["lidar"] = self.lidar
        sys.modules["sensor_worker"] = self.worker
        self.main = _load("as2w_lidar_main_test", "main.py")
        self.dds_init = self.modules["unitree_sdk2py.core.channel"].ChannelFactoryInitialize
        self.ros = self.modules["rclpy"]
        self.config = {"plugins": {name: {"enabled": False} for name in (
            "state", "loco", "special_motion", "mic", "speaker", "led",
            "camera", "controlled_spatial", "slam_mapping", "loco_servo",
        )}}
        self.config["plugins"]["lidar"] = {
            "enabled": True, "process": True, "source_topics": ["rt/test"],
            "max_render_points": 2000,
        }

    def test_auto_detected_interface_reaches_spawned_lidar_dds(self):
        interfaces = [
            {"name": "wlan0", "ipv4": "10.100.129.146", "up": True,
             "wireless": True, "virtual": False},
            {"name": "eno1", "ipv4": "192.168.123.222", "up": True,
             "wireless": False, "virtual": False},
        ]
        context = Mock()
        with patch.object(sys, "argv", ["main.py"]), \
                patch.dict(os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces), \
                patch.object(self.lidar.multiprocessing, "get_context", return_value=context) as get_context:
            interface = self.main.resolve_robot_interface({"robot_interface": "auto"})
            self.main.Bundle(self.config, "test", Mock(), Mock(), interface)

        self.assertEqual("eno1", interface)
        get_context.assert_called_once_with("spawn")
        context.Process.return_value.start.assert_called_once_with()
        process = context.Process.call_args.kwargs
        self.assertEqual("eno1", process["args"][-1])
        node = Mock()

        def create_node(*_args):
            # The child must bind robot DDS before creating any subscriptions.
            self.dds_init.assert_called_once_with(0, "eno1")
            return node

        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode", side_effect=create_node):
            process["target"](*process["args"])
        self.dds_init.assert_called_once_with(0, "eno1")
        self.ros.init.assert_called_once_with(args=None)
        node.close.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()

    def test_unresolved_child_interface_never_initializes_dds_or_ros(self):
        with patch.object(self.lidar, "_LidarNode") as node:
            for interface in (None, "", "  ", "auto", " AUTO "):
                with self.subTest(interface=interface), self.assertRaisesRegex(ValueError, "resolved"):
                    self.worker.run_lidar("/test/lidar/cloud", None, 2000, interface)
        self.dds_init.assert_not_called()
        self.ros.init.assert_not_called()
        node.assert_not_called()

    def test_dds_binding_failure_aborts_before_sensor_subscription(self):
        self.dds_init.side_effect = RuntimeError("adapter is unavailable")
        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode") as node, \
                self.assertRaisesRegex(RuntimeError, "DDS initialization failed"):
            self.worker.run_lidar("/test/lidar/cloud", None, 2000, "eno1")
        self.dds_init.assert_called_once_with(0, "eno1")
        self.ros.init.assert_not_called()
        node.assert_not_called()

    def test_main_without_robot_dds_does_not_spawn_lidar(self):
        with patch.object(self.lidar.multiprocessing, "get_context") as get_context:
            bundle = self.main.Bundle(self.config, "test", Mock(), Mock(), None, dds_ready=False)
        self.assertEqual([], bundle.plugins)
        get_context.assert_not_called()


if __name__ == "__main__":
    unittest.main()
