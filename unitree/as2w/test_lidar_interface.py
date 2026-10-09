"""No-hardware checks for the lidar subprocess's robot DDS binding."""
import importlib.util
import os
import sys
import time
import types
import unittest
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from pathlib import Path
from unittest.mock import Mock, patch


ROOT = Path(__file__).parent


def _ready_child(topic, source_topics, max_render_points, interface, writer):
    """Real spawned process for IPC/cleanup tests, with no ROS or robot SDK."""
    writer.send({"state": "ready"})
    while True:
        time.sleep(1)


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


class _ChildProcess:
    """No-process child double with a real one-way pipe and owned writer FD."""
    def __init__(self, on_start, **kwargs):
        self.kwargs = kwargs
        self.on_start = on_start
        self.alive = False
        self.exitcode = None
        self.writer = None
        self.closed = False
        self.terminated = self.killed = self.joined = 0
        self.ignore_terminate = False

    def start(self):
        self.alive = True
        self.writer = Connection(os.dup(self.kwargs["args"][-1].fileno()),
                                 readable=False, writable=True)
        self.on_start(self)

    def is_alive(self):
        return self.alive

    def terminate(self):
        self.terminated += 1
        if not self.ignore_terminate:
            self.alive = False
            self.exitcode = -15

    def kill(self):
        self.killed += 1
        self.alive = False
        self.exitcode = -9

    def join(self, timeout=None):
        self.joined += 1

    def close(self):
        if self.writer is not None:
            self.writer.close()
        self.closed = True


class _ProcessContext:
    def __init__(self, *on_start):
        self.actions = list(on_start)
        self.processes = []
        self.pipes = []

    def Pipe(self, duplex):
        pipe = Pipe(duplex=duplex)
        self.pipes.append(pipe)
        return pipe

    def Process(self, **kwargs):
        action = self.actions.pop(0) if self.actions else lambda p: p.writer.send({"state": "ready"})
        process = _ChildProcess(action, **kwargs)
        self.processes.append(process)
        return process


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

    def _plugin(self, context):
        with patch.object(self.lidar.multiprocessing, "get_context", return_value=context):
            plugin = self.lidar.LidarPlugin(
                self.config["plugins"]["lidar"], "test", Mock(), "eno1")
        self.addCleanup(plugin.stop)
        return plugin

    def test_auto_detected_interface_reaches_spawned_lidar_dds(self):
        interfaces = [
            {"name": "wlan0", "ipv4": "10.100.129.146", "up": True,
             "wireless": True, "virtual": False},
            {"name": "eno1", "ipv4": "192.168.123.222", "up": True,
             "wireless": False, "virtual": False},
        ]
        context = _ProcessContext()
        with patch.object(sys, "argv", ["main.py"]), \
                patch.dict(os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces), \
                patch.object(self.lidar.multiprocessing, "get_context", return_value=context) as get_context:
            interface = self.main.resolve_robot_interface({"robot_interface": "auto"})
            bundle = self.main.Bundle(self.config, "test", Mock(), Mock(), interface)
        self.addCleanup(bundle.stop_all)

        self.assertEqual("eno1", interface)
        get_context.assert_called_once_with("spawn")
        self.assertEqual("running", bundle.plugins[0].dispatch("info", {})["state"])
        process = context.processes[0].kwargs
        self.assertEqual("eno1", process["args"][3])
        node = Mock(subs=[Mock()])
        status_writer = Mock()
        self.ros.executors.SingleThreadedExecutor.return_value.spin.side_effect = RuntimeError("test spin exit")

        def create_node(*_args):
            # The child must bind robot DDS before creating any subscriptions.
            self.dds_init.assert_called_once_with(0, "eno1")
            status_writer.send.assert_not_called()
            return node

        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode", side_effect=create_node):
            with self.assertRaisesRegex(RuntimeError, "test spin exit"):
                process["target"](*process["args"][:-1], status_writer)
        self.dds_init.assert_called_once_with(0, "eno1")
        self.ros.init.assert_called_once_with(args=None)
        node.close.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()
        self.assertEqual({"state": "ready"}, status_writer.send.call_args_list[0].args[0])
        self.assertEqual("error", status_writer.send.call_args_list[1].args[0]["state"])
        status_writer.close.assert_called_once_with()

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
        writer = Mock()
        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode") as node, \
                self.assertRaisesRegex(RuntimeError, "DDS initialization failed"):
            self.worker.run_lidar("/test/lidar/cloud", None, 2000, "eno1", writer)
        self.dds_init.assert_called_once_with(0, "eno1")
        self.ros.init.assert_not_called()
        node.assert_not_called()
        self.assertEqual("error", writer.send.call_args.args[0]["state"])
        writer.close.assert_called_once_with()

    def test_main_without_robot_dds_does_not_spawn_lidar(self):
        with patch.object(self.lidar.multiprocessing, "get_context") as get_context:
            bundle = self.main.Bundle(self.config, "test", Mock(), Mock(), None, dds_ready=False)
        self.assertEqual([], bundle.plugins)
        get_context.assert_not_called()

    def test_child_requires_a_successful_subscription_before_ready(self):
        node = Mock(subs=[])
        writer = Mock()
        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode", return_value=node), \
                self.assertRaisesRegex(RuntimeError, "no initialized"):
            self.worker.run_lidar("/test/lidar/cloud", None, 2000, "eno1", writer)
        self.assertEqual(1, writer.send.call_count)
        self.assertEqual("error", writer.send.call_args.args[0]["state"])
        self.ros.executors.SingleThreadedExecutor.return_value.spin.assert_not_called()
        node.close.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()
        writer.close.assert_called_once_with()

    def test_child_node_creation_failure_cleans_initialized_ros(self):
        writer = Mock()
        with patch.object(self.worker, "_setup_logs"), \
                patch.object(self.lidar, "_LidarNode", side_effect=RuntimeError("node failed")), \
                self.assertRaisesRegex(RuntimeError, "node failed"):
            self.worker.run_lidar("/test/lidar/cloud", None, 2000, "eno1", writer)
        self.assertEqual("error", writer.send.call_args.args[0]["state"])
        self.ros.executors.SingleThreadedExecutor.return_value.shutdown.assert_called_once_with(timeout_sec=1.0)
        self.ros.shutdown.assert_called_once_with()
        writer.close.assert_called_once_with()

    def test_startup_error_is_visible_without_failing_bundle_construction(self):
        context = _ProcessContext(lambda p: p.writer.send({"state": "error", "error": "DDS init failed"}))
        with patch.object(self.lidar.multiprocessing, "get_context", return_value=context):
            bundle = self.main.Bundle(self.config, "test", Mock(), Mock(), "eno1")
        self.addCleanup(bundle.stop_all)
        self.assertEqual(1, len(bundle.plugins))
        plugin = bundle.plugins[0]
        status = plugin.dispatch("info", {})
        self.assertEqual("error", status["state"])
        self.assertIn("DDS init failed", status["error"])
        self.assertEqual("lidar_cloud", plugin.get_tools()[0]["name"])
        self.assertIsNone(plugin._process)
        self.assertIsNone(plugin._status_reader)
        self.assertTrue(context.processes[0].closed)
        self.assertTrue(all(endpoint.closed for endpoint in context.pipes[0]))

    def test_startup_timeout_terminates_and_reaps_child(self):
        context = _ProcessContext(lambda _p: None)
        with patch.object(self.lidar, "_PROCESS_STARTUP_TIMEOUT_SECONDS", 0.01):
            plugin = self._plugin(context)
        status = plugin.dispatch("info", {})
        self.assertEqual("error", status["state"])
        self.assertIn("timed out", status["error"])
        self.assertEqual(1, context.processes[0].terminated)
        self.assertGreater(context.processes[0].joined, 0)
        self.assertTrue(context.processes[0].closed)
        self.assertIsNone(plugin._status_reader)

    def test_early_exit_or_eof_cannot_report_running(self):
        def exit_child(process):
            process.alive = False
            process.exitcode = 1

        for action in (exit_child, lambda p: p.writer.close()):
            with self.subTest(action=action):
                context = _ProcessContext(action)
                plugin = self._plugin(context)
                self.assertEqual("error", plugin.dispatch("lidar_cloud", {})["state"])
                self.assertTrue(context.processes[0].closed)

    def test_runtime_exit_is_detected_and_start_retries_with_fresh_ipc(self):
        context = _ProcessContext()
        plugin = self._plugin(context)
        old_process = context.processes[0]
        old_process.alive = False
        old_process.exitcode = 2
        self.assertEqual("error", plugin.dispatch("info", {})["state"])
        self.assertTrue(old_process.closed)
        with patch.object(self.lidar.multiprocessing, "get_context", return_value=context):
            self.assertEqual("running", plugin.dispatch("start", {})["state"])
        self.assertEqual(2, len(context.processes))
        self.assertTrue(all(endpoint.closed for endpoint in context.pipes[0]))
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertEqual("idle", plugin.dispatch("info", {})["state"])
        self.assertTrue(all(endpoint.closed for pipe in context.pipes for endpoint in pipe))

    def test_runtime_error_channel_and_repeated_start_stop(self):
        context = _ProcessContext()
        plugin = self._plugin(context)
        self.assertEqual("running", plugin.dispatch("start", {})["state"])
        self.assertEqual(1, len(context.processes))
        context.processes[0].writer.send({"state": "error", "error": "executor failed"})
        status = plugin.dispatch("lidar_cloud", {})
        self.assertEqual("error", status["state"])
        self.assertIn("executor failed", status["error"])
        self.assertEqual("idle", plugin.stop()["state"])
        self.assertEqual("idle", plugin.stop()["state"])
        with patch.object(self.lidar.multiprocessing, "get_context", return_value=context):
            self.assertEqual("running", plugin.start()["state"])
        self.assertEqual(2, len(context.processes))

    def test_stuck_child_is_killed_before_restart(self):
        context = _ProcessContext()
        plugin = self._plugin(context)
        child = context.processes[0]
        child.ignore_terminate = True
        self.assertEqual("idle", plugin.stop()["state"])
        self.assertEqual(1, child.terminated)
        self.assertEqual(1, child.killed)
        self.assertEqual(2, child.joined)
        self.assertTrue(child.closed)

    def test_spawn_failure_is_an_error_and_closes_both_pipe_ends(self):
        def fail_start(_process):
            raise OSError("cannot start process")

        context = _ProcessContext(fail_start)
        plugin = self._plugin(context)
        status = plugin.dispatch("info", {})
        self.assertEqual("error", status["state"])
        self.assertIn("cannot start process", status["error"])
        self.assertTrue(context.processes[0].closed)
        self.assertTrue(all(endpoint.closed for endpoint in context.pipes[0]))

    def test_real_spawn_handshake_and_restart_reap_processes(self):
        with patch.object(self.lidar, "_run_lidar_process", _ready_child):
            plugin = self.lidar.LidarPlugin(
                self.config["plugins"]["lidar"], "test", Mock(), "eno1")
            self.addCleanup(plugin.stop)
            for attempt in range(2):
                if attempt:
                    self.assertEqual("running", plugin.start()["state"])
                self.assertEqual("running", plugin.dispatch("info", {})["state"])
                child = plugin._process
                reader = plugin._status_reader
                self.assertTrue(child.is_alive())
                self.assertEqual("idle", plugin.stop()["state"])
                self.assertIsNone(plugin._process)
                self.assertTrue(reader.closed)
                with self.assertRaises(ValueError):
                    child.is_alive()  # Process.close() only succeeds after exit.


if __name__ == "__main__":
    unittest.main()
