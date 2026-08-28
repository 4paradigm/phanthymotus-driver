import importlib.util
import json
import os
import struct
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
        return [
            {"name": "echo", "type": "actuator", "inputSchema": {"type": "object"}},
            {
                "name": "gait",
                "type": "actuator",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "gait": {
                            "enum": ["basic", "balanced"],
                            "oneOf": [
                                {"const": "basic", "title": "拟人步态"},
                                {"const": "balanced", "title": "下肢平衡"},
                            ]
                        }
                    },
                },
            },
        ]

    def health(self):
        return {"state": self.state, "driver": "engineai-t800"}

    def dispatch(self, name, arguments):
        if name == "echo":
            return {"echo": arguments}
        if name == "gait":
            return {"selected_gait": arguments.get("gait")}
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

    def test_gait_stable_key_survives_actual_mcp_tools_call_envelope(self):
        response = self.rpc("tools/call", {
            "name": "gait",
            "arguments": {"action": "select", "gait": "basic"},
        })
        content = response["result"]["content"]
        self.assertEqual(
            {"selected_gait": "basic"},
            json.loads(content[0]["text"]),
        )

    def test_unknown_tool_returns_json_rpc_error(self):
        response = self.rpc("tools/call", {"name": "missing"})
        self.assertEqual(-32601, response["error"]["code"])

    def test_health_endpoint(self):
        with urllib.request.urlopen(self.url + "/health", timeout=2) as response:
            payload = json.loads(response.read())
            self.assertEqual("engineai-t800", payload["driver"])
            self.assertEqual("running", payload["state"])

    def test_registration_uses_shared_validated_agent_core_transport(self):
        transport_calls = []
        started_threads = []
        fake_context = object()
        fake_device = types.ModuleType("device")
        def transport(path):
            transport_calls.append(path)
            if len(transport_calls) == 1:
                raise ValueError("CA not provisioned yet")
            return (
                "https://phanthy-motus:15678/api/mcp",
                fake_context,
                "/certs/agent-core-ca.pem",
            )
        fake_device._t800_agent_core_transport = transport

        class DeferredThread:
            def __init__(self, *, target, daemon, name):
                self.target = target
                self.daemon = daemon
                self.name = name

            def start(self):
                started_threads.append(self)

        previous_device = sys.modules.get("device")
        original_thread = self.module.threading.Thread
        original_sleep = self.module.time.sleep
        original_urlopen = urllib.request.urlopen
        sleep_calls = []
        states_during_post = []

        class StopLoop(BaseException):
            pass

        def controlled_sleep(_seconds):
            sleep_calls.append(_seconds)
            if len(sleep_calls) >= 2:
                raise StopLoop()

        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        def urlopen(*_args, **_kwargs):
            states_during_post.append(
                self.module._registration_status()["state"]
            )
            return Response()

        sys.modules["device"] = fake_device
        self.module.threading.Thread = DeferredThread
        self.module.time.sleep = controlled_sleep
        urllib.request.urlopen = urlopen
        try:
            self.module._start_registration(15708, {"name": "T800"})
            with self.assertRaises(StopLoop):
                started_threads[0].target()
        finally:
            self.module.threading.Thread = original_thread
            self.module.time.sleep = original_sleep
            urllib.request.urlopen = original_urlopen
            if previous_device is None:
                sys.modules.pop("device", None)
            else:
                sys.modules["device"] = previous_device

        self.assertEqual(["/api/mcp", "/api/mcp"], transport_calls)
        self.assertEqual(1, len(started_threads))
        self.assertEqual("register", started_threads[0].name)
        self.assertEqual(["error"], states_during_post)
        self.assertEqual("ready", self.module._registration_status()["state"])

    def test_registration_failure_degrades_bundle_health_without_stopping_it(self):
        previous = self.module._registration_status()
        bundle = self.module.T800DeviceBundle.__new__(
            self.module.T800DeviceBundle
        )
        bundle._plugins = []
        bundle._active_plugins = []
        bundle._startup_errors = {}
        bundle._started = True
        bundle._acp_status = lambda: {
            "state": "ready",
            "configured": True,
            "last_error": None,
        }
        try:
            self.module._update_registration_status(
                state="error",
                configured=False,
                last_error="AGENT_CORE_CA_CERT is required",
            )
            health = bundle.health()
        finally:
            self.module._update_registration_status(**previous)

        self.assertEqual("degraded", health["state"])
        self.assertEqual("error", health["registration"]["state"])
        self.assertIn("AGENT_CORE_CA_CERT", health["registration"]["last_error"])

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

    def test_bundle_health_degrades_when_acp_configuration_is_invalid(self):
        bundle = self.module.T800DeviceBundle.__new__(
            self.module.T800DeviceBundle
        )
        bundle._plugins = []
        bundle._active_plugins = []
        bundle._startup_errors = {}
        bundle._started = True
        bundle._acp_status = lambda: {
            "state": "error",
            "configured": False,
            "last_error": "AGENT_CORE_CA_CERT is required for https",
        }

        health = bundle.health()

        self.assertEqual("degraded", health["state"])
        self.assertEqual("error", health["acp"]["state"])
        self.assertIn("AGENT_CORE_CA_CERT", health["acp"]["last_error"])

    def test_bundle_halts_physical_outputs_before_reverse_teardown(self):
        events = []

        class Plugin:
            def __init__(self, name, physical=False):
                self.name = name
                if physical:
                    self.halt = lambda: events.append(f"halt:{name}")

            def stop(self):
                events.append(f"stop:{self.name}")

        head = Plugin("head")
        motion = Plugin("motion", physical=True)
        tail = Plugin("tail")
        bundle = self.module.T800DeviceBundle.__new__(self.module.T800DeviceBundle)
        bundle._active_plugins = [head, motion, tail]
        bundle._started = True

        bundle.stop_all()

        self.assertEqual(
            ["halt:motion", "stop:tail", "stop:motion", "stop:head"],
            events,
        )

    def test_bundle_releases_real_virtual_gamepad_in_phase_one(self):
        sys.path.insert(0, str(ROOT))
        from virtual_gamepad import VirtualGamepadPlugin

        events = []

        class Lcm:
            def publish(self, _channel, payload):
                events.append(("gamepad", payload))

        class SlowTail:
            def stop(self):
                events.append(("tail_stop", None))

        gamepad = VirtualGamepadPlugin({}, "robot", None)
        gamepad._lcm = Lcm()
        gamepad.dispatch("sticks", {"left_y": 0.5, "duration": -1})
        events.clear()
        bundle = self.module.T800DeviceBundle.__new__(self.module.T800DeviceBundle)
        bundle._active_plugins = [gamepad, SlowTail()]
        bundle._started = True

        bundle.stop_all()

        self.assertEqual("gamepad", events[0][0])
        released = struct.unpack(">Qq12i6d", events[0][1])
        self.assertEqual((0,) * 12, released[2:14])
        self.assertEqual("tail_stop", events[1][0])

    def test_bundle_blocks_new_motion_while_interrupt_is_settling(self):
        class MotionPlugin:
            def __init__(self):
                self.calls = []

            def get_tool(self):
                return {
                    "name": "gesture",
                    "type": "actuator",
                    "inputSchema": {
                        "type": "object",
                        "properties": {"action": {"type": "string"}},
                        "x-hooks": {
                            "on_interrupt_motion": {"action": "stop_gesture"},
                        },
                    },
                }

            def dispatch(self, action, args):
                self.calls.append((action, dict(args)))
                return {"state": "called"}

        class InterruptGroup:
            def blocking_outputs(self):
                return ["motion_recorder"]

        plugin = MotionPlugin()
        bundle = self.module.T800DeviceBundle.__new__(self.module.T800DeviceBundle)
        bundle._active_plugins = [plugin]
        bundle._motion_events = None
        bundle._motion_interrupt_group = InterruptGroup()
        self.assertTrue(
            {
                "speaker", "loco", "gait", "gesture",
                "motion_recorder", "head",
            }.issubset(
                bundle._MOTION_OUTPUT_TOOLS
            )
        )

        blocked = bundle.dispatch("gesture", {"action": "sequence"})
        self.assertIn("active or still settling", blocked["error"])
        self.assertEqual(["motion_recorder"], blocked["blocking_outputs"])
        self.assertEqual([], plugin.calls)

        blocked_reset = bundle.dispatch(
            "gesture", {"action": "stop", "reset_after": True}
        )
        self.assertIn("active or still settling", blocked_reset["error"])
        self.assertEqual([], plugin.calls)

        stopped = bundle.dispatch("gesture", {"action": "stop_gesture"})
        self.assertEqual("called", stopped["state"])
        self.assertEqual("stop_gesture", plugin.calls[-1][0])


class VendoredContractTests(unittest.TestCase):
    def test_default_loco_profile_matches_approved_real_device_limits(self):
        config_text = (ROOT / "config.yaml").read_text()
        for declaration in (
            "max_vx: 2.0",
            "max_vy: 1.0",
            "max_vyaw: 2.0",
            "locomotion_prepare_duration_sec: 1.0",
        ):
            self.assertIn(declaration, config_text)

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
        for capability in ("speaker", "loco", "gait", "motion_recorder", "head"):
            self.assertIn(capability, metadata_text)
        deploy_text = (ROOT / "deploy" / "service.yml").read_text()
        self.assertNotIn("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", deploy_text)

    def test_cyclonedds_container_logs_are_muzzled_without_losing_interface_selection(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        self.assertIn("ENV RCUTILS_COLORIZED_OUTPUT=0", dockerfile)
        self.assertIn("<Tracing><Verbosity>severe</Verbosity>", dockerfile)
        self.assertIn("<OutputFile>/dev/null</OutputFile></Tracing>", dockerfile)
        self.assertIn("NetworkInterface name='${NETWORK_INTERFACE:-eth1}'", dockerfile)

    def test_acp_uses_agent_core_certificate_and_matching_hostname(self):
        service = (ROOT / "deploy" / "service.yml").read_text()
        self.assertIn(
            '"${T800_AGENT_CORE_HOSTNAME:-phanthy-motus}:'
            '${T800_AGENT_CORE_ADDRESS:-127.0.0.1}"',
            service,
        )
        self.assertIn(
            "AGENT_CORE_URL=${T800_AGENT_CORE_URL:-https://phanthy-motus:15678}",
            service,
        )
        self.assertIn(
            "AGENT_CORE_CA_CERT=${T800_AGENT_CORE_CA_CERT:-"
            "/opt/phanthy-motus/data/certs/cert.pem}",
            service,
        )


if __name__ == "__main__":
    unittest.main()
