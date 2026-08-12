import importlib.util
import json
import os
import sys
import threading
import types
import unittest
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from http.server import ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_main_without_ros():
    fake_rclpy = types.ModuleType("rclpy")
    fake_executors = types.ModuleType("rclpy.executors")
    fake_context = types.ModuleType("rclpy.context")
    fake_executors.MultiThreadedExecutor = object
    fake_context.Context = object
    fake_rclpy.executors = fake_executors
    fake_yaml = types.ModuleType("yaml")
    fake_yaml.safe_load = lambda _value: {}
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.executors"] = fake_executors
    sys.modules["rclpy.context"] = fake_context
    sys.modules.setdefault("yaml", fake_yaml)
    spec = importlib.util.spec_from_file_location("t800_main_contract", ROOT / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeBundle:
    def get_all_tools(self):
        return [{"name": "echo", "type": "actuator", "inputSchema": {"type": "object"}}]

    def dispatch(self, name, arguments):
        if name == "echo":
            return {"echo": arguments}
        return None


class McpHttpContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_main_without_ros()
        cls.module._bundle = FakeBundle()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), cls.module.make_handler())
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def rpc(self, method, params=None, request_id=1):
        request = urllib.request.Request(
            self.url + "/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": request_id, "method": method, "params": params or {}}).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.loads(response.read())

    def test_initialize_contract(self):
        result = self.rpc("initialize")["result"]
        self.assertEqual("2024-11-05", result["protocolVersion"])
        self.assertEqual("engineai-t800-device-bundle", result["serverInfo"]["name"])

    def test_tools_list_contract(self):
        tools = self.rpc("tools/list")["result"]["tools"]
        self.assertEqual("echo", tools[0]["name"])

    def test_tools_call_wraps_plain_dict_once(self):
        response = self.rpc("tools/call", {"name": "echo", "arguments": {"value": 7}})
        content = response["result"]["content"]
        self.assertEqual({"echo": {"value": 7}}, json.loads(content[0]["text"]))

    def test_unknown_tool_returns_json_rpc_error(self):
        response = self.rpc("tools/call", {"name": "missing"})
        self.assertEqual(-32601, response["error"]["code"])

    def test_health_endpoint(self):
        with urllib.request.urlopen(self.url + "/health", timeout=2) as response:
            self.assertEqual("engineai-t800", json.loads(response.read())["driver"])

    def test_non_mcp_path_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as captured:
            urllib.request.urlopen(self.url + "/missing", timeout=2)
        self.assertEqual(404, captured.exception.code)

    def test_cyclonedds_interface_is_validated(self):
        previous = os.environ.pop("CYCLONEDDS_URI", None)
        previous_interface = os.environ.pop("NETWORK_INTERFACE", None)
        try:
            interface = self.module._configure_cyclonedds({"ros": {"robot_interface": "eno1"}})
            self.assertEqual("eno1", interface)
            self.assertIn("name='eno1'", os.environ["CYCLONEDDS_URI"])
            os.environ.pop("CYCLONEDDS_URI", None)
            with self.assertRaisesRegex(ValueError, "invalid"):
                self.module._configure_cyclonedds({"ros": {"robot_interface": "bad iface"}})
        finally:
            os.environ.pop("CYCLONEDDS_URI", None)
            if previous is not None:
                os.environ["CYCLONEDDS_URI"] = previous
            if previous_interface is not None:
                os.environ["NETWORK_INTERFACE"] = previous_interface


class VendoredContractTests(unittest.TestCase):
    def test_urdf_contains_every_driver_joint_name(self):
        sys.path.insert(0, str(ROOT))
        from control import T800_JOINT_NAMES

        tree = ET.parse(ROOT / "resource" / "serial_t800.urdf")
        names = {node.attrib["name"] for node in tree.findall("joint")}
        self.assertTrue(set(T800_JOINT_NAMES).issubset(names))

    def test_required_vendor_messages_are_present(self):
        message_dir = ROOT / "msgs" / "interface_protocol" / "msg"
        required = {
            "BodyVelCmd.msg", "ImuInfo.msg", "JointCommand.msg", "JointMotionPlanRequest.msg",
            "JointMotionPlanState.msg", "JointOverrideCommand.msg", "JointState.msg", "LedControl.msg",
            "MotionState.msg", "MotionStateRequest.msg", "MotorDebug.msg", "PowerInfo.msg", "Tts.msg",
            "NodeControl.msg", "DynamicVectorDouble.msg", "LinkInfo.msg", "Alert.msg", "MotorCommand.msg",
        }
        self.assertTrue(required.issubset({path.name for path in message_dir.glob("*.msg")}))
        self.assertTrue((ROOT / "msgs" / "interface_protocol" / "srv" / "JointMotionPlanRequest.srv").is_file())

    def test_metadata_and_config_use_same_port(self):
        config_text = (ROOT / "config.yaml").read_text()
        metadata_text = (ROOT / "driver.yaml").read_text()
        config_port = int(next(line.split(":", 1)[1] for line in config_text.splitlines()
                               if line.startswith("mcp_port:")))
        metadata_port = int(next(line.split(":", 1)[1] for line in metadata_text.splitlines()
                                 if line.startswith("port:")))
        self.assertEqual(config_port, metadata_port)
        self.assertIn(f":{config_port}/mcp", metadata_text)


if __name__ == "__main__":
    unittest.main()
