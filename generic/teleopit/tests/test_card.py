"""Real MCP HTTP and stock card contract tests, without ROS/Teleopit installs."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen


DRIVER = Path(__file__).resolve().parents[1]
REPO = DRIVER.parents[1]
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(REPO))
spec = importlib.util.spec_from_file_location("teleopit_card_entrypoint", DRIVER / "main.py")
entrypoint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entrypoint)
from plugin import CoreTopicPublisher, TeleopitPlugin


class FakeManager:
    def __init__(self):
        self.state = "idle"
        self.runs = []
        self.preflights = []
        self.stops = 0
        self.jpeg = b"\xff\xd8test-jpeg\xff\xd9"

    def info(self):
        return {"state": self.state, "session_id": "test-session" if self.runs else None,
                "snapshot": {"target_joint_pos": [0.0] * 29}, "error": None,
                "hardware_output": False, "profile": "g1_29_sim"}

    def preflight(self, options=None):
        self.preflights.append(dict(options or {}))
        return {"ok": True, "hardware_output": False, "checks": {"policy": True}}

    def run(self, options=None):
        self.runs.append(dict(options or {}))
        self.state = "running"
        return self.info()

    def stop(self):
        self.stops += 1
        self.state = "idle"
        return self.info()

    def pause(self):
        self.state = "paused"
        return self.info()

    def resume(self):
        self.state = "running"
        return self.info()

    def preview(self):
        return self.jpeg

    def close(self):
        self.stop()


class FakePublisher:
    def __init__(self):
        self.states = []
        self.frames = []

    def publish_state(self, state):
        self.states.append(json.loads(json.dumps(state, allow_nan=False)))

    def publish_preview(self, jpeg):
        self.frames.append(jpeg)


class MCPCardTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.config = {"ros_namespace": "lab", "teleopit": {"max_steps": 123}}
        self.bundle = entrypoint.build_bundle(self.config, manager=self.manager)
        self.bundle.start_all()
        self.server = entrypoint.create_server(self.config, self.bundle, host="127.0.0.1", port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}/mcp"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.bundle.stop_all()

    def rpc(self, method, params=None):
        data = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                           "params": params or {}}).encode()
        with urlopen(Request(self.url, data=data, headers={"Content-Type": "application/json"}), timeout=2) as response:
            return json.load(response)

    def call(self, name="teleopit_sim", action="info", **options):
        response = self.rpc("tools/call", {"name": name, "arguments": {"action": action, **options}})
        self.assertNotIn("error", response, response)
        content = response["result"]["content"]
        self.assertEqual(len(content), 1)
        self.assertEqual(content[0]["type"], "text")
        result = json.loads(content[0]["text"])
        self.assertIsInstance(result, dict)
        return result

    def test_discovery_uses_stock_actuator_json_and_jpeg_cards(self):
        initialized = self.rpc("initialize")
        self.assertEqual(initialized["result"]["serverInfo"]["name"], "teleopit-simulation")
        tools = {item["name"]: item for item in self.rpc("tools/list")["result"]["tools"]}
        self.assertEqual(set(tools), {"teleopit_sim", "teleopit_state", "teleopit_preview"})
        simulation = tools["teleopit_sim"]
        self.assertEqual(simulation["type"], "actuator")
        self.assertFalse(simulation["multiInstance"])
        self.assertEqual(simulation["configSchema"]["properties"]["max_steps"]["default"], 123)
        self.assertTrue(simulation["configSchema"]["properties"]["pico_advertise_host"]["x-sensitive"])
        self.assertNotIn("upstream_root", simulation["configSchema"]["properties"])
        self.assertIn("run", simulation["inputSchema"]["x-action-params"])
        self.assertEqual(tools["teleopit_state"]["topic_out"],
                         [{"topic": "/lab/teleopit/state", "format": "data/json"}])
        self.assertEqual(tools["teleopit_preview"]["topic_out"],
                         [{"topic": "/lab/teleopit/preview", "format": "image/jpeg"}])
        self.assertEqual(self.manager.runs, [])

    def test_core_config_start_run_pause_resume_stop_round_trip(self):
        # Core sends config separately before start and before non-system actions.
        configured = self.call(action="config", source="pico", max_steps=500, human_height=1.8)
        self.assertTrue(configured["adapter_ok"])
        self.assertEqual(self.call(action="start")["state"], "ready")
        self.assertEqual(self.manager.runs, [])
        self.assertTrue(self.call(action="preflight")["ok"])
        self.assertEqual(self.manager.preflights[-1]["source"], "pico")
        self.assertEqual(self.manager.runs, [])
        result = self.call(action="run", max_steps=17)
        self.assertEqual(result["state"], "running")
        self.assertFalse(result["hardware_output"])
        self.assertEqual(self.manager.runs, [{"source": "pico", "max_steps": 17, "human_height": 1.8}])
        self.assertEqual(self.call(action="pause")["state"], "paused")
        self.assertEqual(self.call(action="resume")["state"], "running")
        self.call(action="config", source="bvh")
        self.assertEqual(self.manager.state, "running")
        self.assertEqual(len(self.manager.runs), 1)
        self.assertEqual(self.call(action="info")["configured_options"]["source"], "bvh")
        self.assertEqual(self.call(action="stop")["state"], "idle")
        rejected = self.call(action="run")
        self.assertFalse(rejected["ok"])
        self.assertEqual(len(self.manager.runs), 1)
        self.call(action="start")
        self.call(action="run")
        self.assertEqual(self.manager.runs[-1]["source"], "bvh")
        self.assertEqual(self.manager.runs[-1]["max_steps"], 500)

    def test_invalid_input_and_hidden_server_options_cannot_reach_worker(self):
        for options in ({"source": "real"}, {"max_steps": True}, {"max_steps": -1},
                        {"human_height": float("nan")}, {"render": "false"},
                        {"upstream_root": "/tmp/untrusted"}, {"policy_path": "/tmp/model"},
                        {"profile": "g1_23_live"}):
            with self.subTest(options=options):
                response = self.rpc("tools/call", {"name": "teleopit_sim",
                                    "arguments": {"action": "run", **options}})
                self.assertEqual(response["error"]["code"], -32602)
        self.assertEqual(self.manager.runs, [])

    def test_unknown_tool_or_action_is_explicit_jsonrpc_error(self):
        for name, action in (("unknown", "run"), ("teleopit_sim", "live")):
            response = self.rpc("tools/call", {"name": name, "arguments": {"action": action}})
            self.assertEqual(response["error"]["code"], -32601)

    def test_no_ros_mode_keeps_simulation_observable_and_reports_stream_gap(self):
        result = self.call("teleopit_preview", "start")
        self.assertEqual(result["state"], "unavailable")
        self.assertFalse(result["ok"])
        info = self.call(action="info")
        self.assertFalse(info["ros_available"])
        self.assertEqual(len(info["snapshot"]["target_joint_pos"]), 29)
        self.assertEqual(self.call("teleopit_state", "stop")["state"], "idle")
        self.assertEqual(self.manager.stops, 0)

    def test_health_and_request_size_limit(self):
        with urlopen(self.url.replace("/mcp", "/health"), timeout=2) as response:
            self.assertEqual(json.load(response)["driver"], "teleopit-driver")
        with self.assertRaises(HTTPError) as raised:
            urlopen(Request(self.url, data=b"x" * 65537), timeout=2)
        self.assertEqual(raised.exception.code, 413)


class SensorLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.publisher = FakePublisher()
        self.plugin = TeleopitPlugin({}, "test", self.manager, self.publisher)

    def call(self, name, action):
        return self.plugin.dispatch(action, {"_tool_name": name})

    def test_state_and_preview_producer_enable_disable_independently(self):
        self.plugin.publish_once()
        self.assertEqual(self.publisher.states, [])
        self.assertEqual(self.publisher.frames, [])
        self.call("teleopit_state", "start")
        self.call("teleopit_preview", "start")
        self.plugin.publish_once()
        self.assertEqual(len(self.publisher.states), 1)
        self.assertFalse(self.publisher.states[-1]["hardware_output"])
        self.assertEqual(self.publisher.states[-1]["profile"], "g1_29_sim")
        self.assertEqual(self.publisher.frames, [self.manager.jpeg])
        self.call("teleopit_preview", "stop")
        self.plugin.publish_once()
        self.assertEqual(len(self.publisher.states), 2)
        self.assertEqual(len(self.publisher.frames), 1)
        self.call("teleopit_state", "stop")
        self.plugin.publish_once()
        self.assertEqual(len(self.publisher.states), 2)
        self.assertEqual(self.manager.stops, 0)
        self.assertEqual(self.manager.runs, [])

    def test_no_frame_does_not_publish_placeholder(self):
        self.manager.jpeg = None
        self.call("teleopit_preview", "start")
        self.plugin.publish_once()
        self.assertEqual(self.publisher.frames, [])
        self.assertFalse(self.call("teleopit_preview", "info")["preview_available"])

    def test_shutdown_stops_simulation_and_publishers_without_restart(self):
        self.plugin.start()
        self.call("teleopit_state", "start")
        self.plugin.stop()
        self.plugin.publish_once()
        self.assertFalse(self.plugin._thread.is_alive())
        self.assertEqual(self.manager.stops, 1)
        self.assertEqual(self.publisher.states, [])
        self.assertEqual(self.plugin.start()["state"], "error")

    def test_start_and_run_cannot_overtake_in_progress_stop(self):
        self.plugin.start()
        stopping = threading.Event()
        finish = threading.Event()
        original_stop = self.manager.stop

        def slow_stop():
            stopping.set()
            if not finish.wait(2):
                raise AssertionError("test did not release stop")
            return original_stop()

        self.manager.stop = slow_stop
        thread = threading.Thread(target=lambda: self.call("teleopit_sim", "stop"))
        thread.start()
        try:
            self.assertTrue(stopping.wait(1))
            self.assertFalse(self.call("teleopit_sim", "start")["adapter_ok"])
            self.assertFalse(self.call("teleopit_sim", "run")["ok"])
            self.assertEqual(self.manager.runs, [])
        finally:
            finish.set()
            thread.join(timeout=2)
            self.plugin.stop()


class CoreTransportTests(unittest.TestCase):
    def test_ros_contract_is_core_domain_json_string_and_compressed_jpeg(self):
        initialized = []
        shutdown = []
        publishers = {}

        class Image:
            def __init__(self):
                self.header = SimpleNamespace(stamp=None, frame_id="")

        class Node:
            def __init__(self, name, context):
                self.context = context

            def create_publisher(self, message_type, topic, qos):
                messages = []
                publishers[topic] = (message_type, messages, qos)
                return SimpleNamespace(publish=messages.append)

            def get_clock(self):
                return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "ros-stamp"))

            def destroy_node(self):
                pass

        context = object()
        modules = {
            "rclpy": SimpleNamespace(init=lambda **kwargs: initialized.append(kwargs),
                                     ok=lambda **kwargs: True,
                                     shutdown=lambda **kwargs: shutdown.append(kwargs)),
            "rclpy.context": SimpleNamespace(Context=lambda: context),
            "rclpy.node": SimpleNamespace(Node=Node),
            "rclpy.qos": SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(VOLATILE="volatile"),
                HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
                ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"),
                QoSProfile=lambda **kwargs: kwargs),
            "sensor_msgs": SimpleNamespace(),
            "sensor_msgs.msg": SimpleNamespace(CompressedImage=Image),
            "std_msgs": SimpleNamespace(),
            "std_msgs.msg": SimpleNamespace(String=SimpleNamespace),
        }
        with patch.dict(sys.modules, modules):
            publisher = CoreTopicPublisher("lab", 42)
            publisher.publish_state({"state": "running", "hardware_output": False})
            publisher.publish_preview(b"jpeg-content")
            publisher.close()
        self.assertEqual(initialized, [{"context": context, "domain_id": 42}])
        self.assertEqual(shutdown, [{"context": context}])
        self.assertEqual(set(publishers), {"/lab/teleopit/state", "/lab/teleopit/preview"})
        state_type, states, _ = publishers["/lab/teleopit/state"]
        self.assertIs(state_type, SimpleNamespace)
        self.assertEqual(json.loads(states[0].data), {"state": "running", "hardware_output": False})
        image_type, images, qos = publishers["/lab/teleopit/preview"]
        self.assertIs(image_type, Image)
        self.assertEqual(images[0].data, b"jpeg-content")
        self.assertEqual(images[0].format, "jpeg")
        self.assertEqual(images[0].header.stamp, "ros-stamp")
        self.assertEqual(qos["depth"], 1)


class EntrypointTests(unittest.TestCase):
    def test_help_requires_neither_ros_nor_teleopit(self):
        result = subprocess.run([sys.executable, str(DRIVER / "main.py"), "--help"],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--no-ros", result.stdout)
        self.assertIn("--no-register", result.stdout)
        self.assertIn("15719", result.stdout)


if __name__ == "__main__":
    unittest.main()
