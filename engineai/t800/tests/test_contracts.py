import importlib.util
import json
import os
import sys
import threading
import time
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
    def __init__(self, state="running"):
        self.state = state

    def get_all_tools(self):
        return [{"name": "echo", "type": "actuator", "inputSchema": {"type": "object"}}]

    def health(self):
        return {"state": self.state, "driver": "engineai-t800"}

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
            payload = json.loads(response.read())
            self.assertEqual("engineai-t800", payload["driver"])
            self.assertEqual("running", payload["state"])

    def test_degraded_health_is_not_reported_as_healthy(self):
        previous = self.module._bundle
        self.module._bundle = FakeBundle(state="degraded")
        try:
            with self.assertRaises(urllib.error.HTTPError) as captured:
                urllib.request.urlopen(self.url + "/health", timeout=2)
            self.assertEqual(503, captured.exception.code)
            self.assertEqual("degraded", json.loads(captured.exception.read())["state"])
        finally:
            self.module._bundle = previous

    def test_non_mcp_path_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as captured:
            urllib.request.urlopen(self.url + "/missing", timeout=2)
        self.assertEqual(404, captured.exception.code)

    def test_legacy_sse_endpoint_and_messages_path(self):
        stream = urllib.request.urlopen(self.url + "/mcp/sse", timeout=2)
        try:
            self.assertEqual("text/event-stream", stream.headers.get_content_type())
            endpoint = ""
            deadline = time.time() + 2
            while time.time() < deadline:
                line = stream.readline().decode().strip()
                if line.startswith("data: "):
                    endpoint = line[len("data: "):]
                    break
            self.assertTrue(endpoint.startswith("/mcp/messages?session_id="), endpoint)
            request = urllib.request.Request(
                self.url + endpoint,
                data=json.dumps({"jsonrpc": "2.0", "id": 99, "method": "tools/list"}).encode(),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(request, timeout=2) as response:
                self.assertEqual(202, response.status)
            deadline = time.time() + 2
            payload = None
            while time.time() < deadline:
                line = stream.readline().decode().strip()
                if line.startswith("data: "):
                    candidate = json.loads(line[len("data: "):])
                    if candidate.get("jsonrpc") == "2.0":
                        payload = candidate
                        break
            self.assertIsNotNone(payload)
            self.assertEqual(99, payload["id"])
            self.assertEqual("echo", payload["result"]["tools"][0]["name"])
        finally:
            stream.close()

    def test_cyclonedds_interface_is_validated(self):
        previous = os.environ.pop("CYCLONEDDS_URI", None)
        previous_interface = os.environ.pop("NETWORK_INTERFACE", None)
        previous_if_nameindex = self.module.socket.if_nameindex
        self.module.socket.if_nameindex = lambda: [(1, "lo"), (2, "eno1")]
        try:
            interface = self.module._configure_cyclonedds({"ros": {"robot_interface": "eno1"}})
            self.assertEqual("eno1", interface)
            self.assertIn("name='eno1'", os.environ["CYCLONEDDS_URI"])
            os.environ.pop("CYCLONEDDS_URI", None)
            with self.assertRaisesRegex(ValueError, "invalid"):
                self.module._configure_cyclonedds({"ros": {"robot_interface": "bad iface"}})
            with self.assertRaisesRegex(ValueError, "does not exist"):
                self.module._configure_cyclonedds({"ros": {"robot_interface": "eth9"}})
        finally:
            self.module.socket.if_nameindex = previous_if_nameindex
            os.environ.pop("CYCLONEDDS_URI", None)
            if previous is not None:
                os.environ["CYCLONEDDS_URI"] = previous
            if previous_interface is not None:
                os.environ["NETWORK_INTERFACE"] = previous_interface

    def test_numeric_docker_hostname_becomes_valid_ros_namespace(self):
        previous_gethostname = self.module.socket.gethostname
        try:
            self.module.socket.gethostname = lambda: "20f7d0265d7d"
            self.assertEqual("t800_20f7d0265d7d", self.module._resolve_namespace({}))
            self.assertEqual("t800", self.module._resolve_namespace({"ros_namespace": "---"}))
        finally:
            self.module.socket.gethostname = previous_gethostname

    def test_bundle_hides_failed_plugins_and_reports_degraded(self):
        class Plugin:
            def __init__(self, name, fail=False):
                self.name = name
                self.fail = fail
                self.stops = 0

            def get_tool(self):
                return {"name": self.name, "type": "sensor", "inputSchema": {"type": "object"}}

            def start(self):
                if self.fail:
                    raise RuntimeError(f"{self.name} failed")

            def stop(self):
                self.stops += 1

            def dispatch(self, action, _args):
                return {"state": action}

        good = Plugin("good")
        bad = Plugin("bad", fail=True)
        bundle = self.module.T800DeviceBundle.__new__(self.module.T800DeviceBundle)
        bundle._plugins = [good, bad]
        bundle._active_plugins = []
        bundle._startup_errors = {}
        bundle._started = False
        bundle._motion_events = None

        bundle.start_all()
        self.assertEqual(["good"], [tool["name"] for tool in bundle.get_all_tools()])
        self.assertEqual("degraded", bundle.health()["state"])
        self.assertIn("Plugin", bundle.health()["startup_errors"])
        self.assertIsNone(bundle.dispatch("bad", {}))
        bundle.stop_all()
        bundle.stop_all()
        self.assertEqual(1, good.stops)


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
        self.assertIn('hardware_model: "t800"', metadata_text)
        self.assertNotIn("t800-dev", metadata_text)
        deploy_text = (ROOT / "deploy" / "service.yml").read_text()
        self.assertNotIn("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", deploy_text)


if __name__ == "__main__":
    unittest.main()
