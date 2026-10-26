"""Offline transport tests: mock ROS calls, never send a robot request."""
import io
import json
import os
from pathlib import Path
import runpy
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import test_device
import head_transport


class TransportTests(unittest.TestCase):
    def process(self):
        process = mock.Mock()
        process.poll.return_value = None
        process.stdout.fileno.return_value = 123
        return process

    def test_probe_protocol_uses_fastdds_without_changing_parent_environment(self):
        process = self.process()
        with mock.patch.dict(os.environ, {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "FASTDDS_BUILTIN_TRANSPORTS": "SHM", "CYCLONEDDS_URI": "parent"}), mock.patch.object(head_transport.subprocess, "Popen", return_value=process) as popen:
            transport = head_transport.HeadServiceTransport()
            env = popen.call_args.kwargs["env"]
            self.assertEqual(env["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
            self.assertEqual(env["ROS_DOMAIN_ID"], "2")
            self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", env)
            self.assertNotIn("CYCLONEDDS_URI", env)
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
        with mock.patch.object(head_transport.select, "select", return_value=([process.stdout], [], [])), mock.patch.object(head_transport.os, "read", return_value=b'diagnostic\nU1_HEAD_REPLY {"ok":true,"ready":true}\n'):
            self.assertTrue(transport.wait_for_service())
        command = json.loads(process.stdin.write.call_args.args[0])
        self.assertEqual(command, {"operation": "probe"})
        transport.close()

    def test_timeout_closes_transport_without_reusing_late_replies(self):
        process = self.process()
        with mock.patch.object(head_transport.subprocess, "Popen", return_value=process):
            transport = head_transport.HeadServiceTransport()
        with self.assertRaises(TimeoutError):
            transport._rpc({"operation": "probe"}, 0)
        process.terminate.assert_called_once()
        process.stdout.close.assert_called_once()

    def test_child_rejects_turn_until_readonly_probe_succeeds(self):
        services = types.ModuleType("rcl_interfaces.srv")
        services.GetParameters = types.SimpleNamespace(Request=lambda: types.SimpleNamespace(names=[]))
        motion_module = types.ModuleType("uworld_motion_msgs.srv")
        motion_module.SoundSource = types.SimpleNamespace(Request=lambda: types.SimpleNamespace(revolute_setup=types.SimpleNamespace()))
        import rclpy
        from common import logsafe
        with mock.patch.dict(sys.modules, {"rcl_interfaces.srv": services, "uworld_motion_msgs.srv": motion_module}), mock.patch.object(logsafe, "install"):
            namespace = runpy.run_path(str(Path(__file__).with_name("head_service_worker.py")))
        node = mock.Mock()
        probe, motion = mock.Mock(), mock.Mock()
        node.create_client.side_effect = [probe, motion]
        probe.call_async.return_value.result.return_value = types.SimpleNamespace(values=[types.SimpleNamespace(type=1, bool_value=False)])
        motion.call_async.return_value.result.return_value = types.SimpleNamespace(success=True, status=0, message="accepted")
        replies = []
        function = namespace["main"]
        function.__globals__["reply"] = replies.append
        function.__globals__["validate_runtime"] = lambda: None
        turn = {"operation": "turn", "degree": 5, "max_vel": .1, "max_acc": .1}
        commands = [turn, {"operation": "probe"}, turn, turn]
        with mock.patch.object(rclpy, "create_node", return_value=node, create=True), mock.patch.object(rclpy, "spin_until_future_complete", create=True), mock.patch.object(sys, "stdin", io.StringIO("\n".join(json.dumps(c) for c in commands))):
            function()
        self.assertFalse(replies[0]["ok"])
        self.assertTrue(replies[1]["ok"])
        self.assertTrue(replies[2]["ok"])
        self.assertFalse(replies[3]["ok"])
        motion.call_async.assert_called_once()
        request = motion.call_async.call_args.args[0]
        self.assertFalse(request.with_waist)
        node.destroy_node.assert_called_once()

    def test_profile_and_worker_packaging(self):
        root = Path(__file__).parent
        from xml.etree import ElementTree as ET
        tree = ET.parse(root / "head-dds-local.xml")
        ns = {"d": "http://www.eprosima.com"}
        self.assertEqual([x.text for x in tree.findall(".//d:interfaceWhiteList/d:address", ns)], ["127.0.0.1"])
        self.assertEqual(tree.findtext(".//d:useBuiltinTransports", namespaces=ns), "false")
        docker = (root / "Dockerfile").read_text()
        for name in ("head_transport.py", "head_service_worker.py", "head-dds-local.xml", "ros-humble-rmw-fastrtps-cpp"):
            self.assertIn(name, docker)


if __name__ == "__main__":
    unittest.main()
