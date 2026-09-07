import importlib.util
import io
import json
import math
import os
from pathlib import Path
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "realman" / "rm75_6f_v"


def load_device():
    spec = importlib.util.spec_from_file_location("realman_rm75_device", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RealManRM75ImageContractTests(unittest.TestCase):
    def test_image_contains_only_minimal_api2_runtime(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("COPY vendor/Robotic_Arm/ /work/Robotic_Arm/", dockerfile)
        self.assertNotIn("RM_API2_LIB_URL", dockerfile)
        self.assertNotIn("ADD http", dockerfile)
        self.assertIn("COPY deploy/ /deploy/", dockerfile)
        self.assertNotIn("colcon", dockerfile)
        self.assertNotIn("rm_driver", dockerfile)
        self.assertNotIn("python3-pip", dockerfile)
        self.assertNotIn("pip3 install", dockerfile)
        self.assertFalse((DRIVER / "entrypoint.sh").exists())

    def test_vendor_shared_libraries_are_not_committed(self):
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_c.so")))
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_cpp.so")))

    def test_service_has_motion_capable_rm75_default(self):
        service = (DRIVER / "deploy" / "service.yml").read_text()
        self.assertIn("RM_DRIVER_ENABLED=1", service)
        self.assertIn("RM_MOTION_ENABLED=1", service)
        self.assertIn("RM_ARM_IP=${RM75_ARM_IP:-192.168.1.18}", service)
        self.assertIn("AGENT_CORE_CA_CERT=${RM75_AGENT_CORE_CA_CERT:-/opt/phanthy-motus/data/certs/cert.pem}", service)
        self.assertIn("AGENT_CORE_TOKEN=${RM75_AGENT_CORE_TOKEN:-}", service)
        self.assertIn("/opt/phanthy-motus/data:/opt/phanthy-motus/data:ro", service)
        self.assertIn("network_mode: host", service)
        self.assertIn("/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro", service)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml", service)
        self.assertIn(
            "${RM_API2_LIB_DIR:-/opt/realman/rm_api2/libs/linux_arm}:/work/Robotic_Arm/libs/linux_arm:ro",
            service,
        )
        self.assertNotIn("/opt/realman/rm_ws", service)
        self.assertNotIn("ipc:", service)


class RealManRM75SDKClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def test_disabled_by_default_and_motion_is_independently_locked(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        client.start()
        self.assertEqual("disabled", client.status()["state"])
        self.assertFalse(client.motion_enabled)
        tools = self.device.RM75Plugin(client, {}).get_tools()
        self.assertEqual(
            {"connection", "joint_states", "model", "robot_info", "software_info", "arm_all_state", "controller_state", "joint_control"},
            {item["name"].split(".")[-1] for item in tools},
        )
        joint_control = next(item for item in tools if item["name"] == "joint_control")
        self.assertEqual("actuator", joint_control["type"])
        self.assertEqual(["set"], joint_control["inputSchema"]["x-completion"]["actions"])
        self.assertEqual(
            {"on_interrupt_motion": {"action": "stopmotion"}},
            joint_control["inputSchema"]["x-hooks"],
        )
        self.assertIs(True, joint_control["inputSchema"]["x-is-dangerous"])
        self.assertEqual(10, joint_control["inputSchema"]["properties"]["speed_percent"]["maximum"])
        self.assertNotIn("timeout_seconds", joint_control["inputSchema"]["properties"])
        self.assertEqual(305, joint_control["inputSchema"]["x-completion"]["timeout"])
        descriptions = {
            name: joint_control["inputSchema"]["properties"][name]["description"]
            for name in (f"joint{i}_deg" for i in range(1, 8))
        }
        for index, (low, high) in enumerate(self.device.JOINT_LIMITS_DEG, 1):
            self.assertEqual(f"[{low:g}°, {high:g}°]", descriptions[f"joint{index}_deg"])

    def test_enabled_driver_reports_missing_host_sdk_mount(self):
        with mock.patch.dict(os.environ, {"RM_DRIVER_ENABLED": "1", "RM_ARM_IP": "192.0.2.1"}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        with mock.patch.object(self.device, "SDK_LIBRARY_PATH", Path("/definitely/missing/libapi_c.so")):
            with self.assertRaisesRegex(FileNotFoundError, "mount RM_API2_LIB_DIR"):
                client.start()

    def test_tool_start_returns_contract_lifecycle_state(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        self.assertEqual({"state": "running"}, plugin.dispatch("start", {"_tool_name": "joint_states"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "joint_control"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "model"}))

    def test_http_log_message_is_escaped_and_capped(self):
        runtime_spec = importlib.util.spec_from_file_location("realman_vendor_runtime", ROOT / "common" / "vendor_runtime.py")
        runtime = importlib.util.module_from_spec(runtime_spec)
        runtime_spec.loader.exec_module(runtime)
        handler = runtime.make_handler(lambda: None, "test", "test")

        class Request:
            @staticmethod
            def address_string():
                return "192.0.2.1"

        output = io.StringIO()
        with redirect_stdout(output):
            handler.log_message(Request(), "%s", "GET /bad\r\nINJECT " + "x" * 400)
        logged = output.getvalue().rstrip("\n")
        self.assertIn(r"GET /bad\r\nINJECT", logged)
        self.assertNotIn("\r", logged)
        self.assertNotIn("\nINJECT", logged)
        self.assertLessEqual(len(logged.removeprefix("[mcp] 192.0.2.1 ")), 200)

    def test_joint_degrees_are_converted_to_radians(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_joint_degree(self):
                return 0, [0, 90, -90, 180, -180, 45, -45]

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        result = client.joint_states()
        self.assertEqual(7, len(result["position"]))
        self.assertAlmostEqual(math.pi / 2, result["position"][1])
        self.assertAlmostEqual(-math.pi, result["position"][4])
        self.assertEqual("rad", result["position_unit"])

    def test_sdk_error_is_not_returned_as_sensor_data(self):
        with self.assertRaisesRegex(RuntimeError, "code 5"):
            self.device._sdk_result("rm_get_robot_info", (5, {}))

    def test_all_advertised_read_only_methods_accept_their_sdk_return_shapes(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_robot_info(self):
                return 0, {"arm_dof": 7}

            def rm_get_arm_software_info(self):
                return 0, {"product_version": "test"}

            def rm_get_arm_all_state(self):
                return 0, {"joint_en_flag": [1] * 7}

            def rm_get_controller_state(self):
                return {"return_code": 0, "voltage": 48.0, "current": 1.0,
                        "temperature": 30.0, "system_error": 0}

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        plugin = self.device.RM75Plugin(client, {})
        for name in plugin.METHODS:
            result = plugin.dispatch("get", {"_tool_name": name})
            self.assertIsInstance(result, dict, name)
        self.assertEqual(0, plugin.dispatch("get", {"_tool_name": "controller_state"})["return_code"])

    def test_controller_state_rejects_nonzero_return_code(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_controller_state(self):
                return {"return_code": -2}

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        with self.assertRaisesRegex(RuntimeError, "code -2"):
            client.call_dict("rm_get_controller_state")

    def test_acp_https_requires_ca_and_mirrors_terminal_event_to_canvas(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        with mock.patch.dict(os.environ, {"AGENT_CORE_URL": "https://phanthy-motus:15678"}, clear=True), \
                mock.patch.object(self.device.urllib.request, "urlopen") as urlopen:
            plugin._acp_callback("action-1", "completed", {})
            urlopen.assert_not_called()

        context = object()
        with mock.patch.dict(os.environ, {
                "AGENT_CORE_URL": "https://phanthy-motus:15678",
                "AGENT_CORE_CA_CERT": "/cert.pem",
                "AGENT_CORE_TOKEN": "secret-token",
            }, clear=True), \
                mock.patch.object(self.device.ssl, "create_default_context", return_value=context) as create_context, \
                mock.patch.object(self.device.urllib.request, "urlopen") as urlopen:
            plugin._acp_callback("action-2", "completed", {"max_error_deg": 0.1})
            create_context.assert_called_once_with(cafile="/cert.pem")
            self.assertEqual(2, urlopen.call_count)
            acp_call, canvas_call = urlopen.call_args_list
            self.assertEqual(
                "https://phanthy-motus:15678/api/acp/complete",
                acp_call.args[0].full_url,
            )
            self.assertEqual(
                "https://phanthy-motus:15678/api/event",
                canvas_call.args[0].full_url,
            )
            self.assertIs(context, acp_call.kwargs["context"])
            self.assertIs(context, canvas_call.kwargs["context"])
            self.assertEqual(
                "Bearer secret-token",
                canvas_call.args[0].get_header("Authorization"),
            )
            canvas_payload = json.loads(canvas_call.args[0].data)
            self.assertEqual("", canvas_payload["text"])
            self.assertEqual("rm75_canvas", canvas_payload["source"])
            self.assertEqual("completed", canvas_payload["payload"]["status"])
            self.assertEqual("canvas_action_complete", canvas_payload["payload"]["type"])
            self.assertEqual("action-2", canvas_payload["payload"]["action_id"])

    def test_canvas_event_failure_does_not_repeat_or_replace_acp_completion(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        first_response = mock.Mock()
        with mock.patch.dict(os.environ, {
                "AGENT_CORE_URL": "http://127.0.0.1:15678",
            }, clear=True), \
                mock.patch.object(
                    self.device.urllib.request,
                    "urlopen",
                    side_effect=[first_response, RuntimeError("canvas unavailable")],
                ) as urlopen:
            plugin._acp_callback("action-3", "completed", {})
        self.assertEqual(2, urlopen.call_count)
        first_response.close.assert_called_once_with()

    def _motion_plugin(self, *, motion_enabled=True, current=None, all_state=None, safety=None):
        current = current or [0.0] * 7
        all_state = all_state or {
            "joint_err_code": [0] * 7,
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        }

        class Handle:
            id = 1

        class Robot:
            def __init__(self):
                self.moves = []
                self.stops = 0

            def rm_get_joint_degree(self):
                return 0, list(current)

            def rm_get_arm_all_state(self):
                return 0, dict(all_state)

            def rm_get_joint_drive_min_pos(self):
                return 0, [item[0] for item in self_module.JOINT_LIMITS_DEG]

            def rm_get_joint_drive_max_pos(self):
                return 0, [item[1] for item in self_module.JOINT_LIMITS_DEG]

            def rm_movej(self, target, speed, radius, connect, block):
                self.moves.append((list(target), speed, radius, connect, block))
                return 0

            def rm_set_arm_slow_stop(self):
                self.stops += 1
                return 0

            def rm_delete_robot_arm(self):
                return 0

        self_module = self.device
        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client.motion_enabled = motion_enabled
        client._handle = Handle()
        client._robot = Robot()
        safety_config = {"poll_interval_seconds": 0.001}
        safety_config.update(safety or {})
        plugin = self.device.RM75Plugin(client, {"safety": safety_config})
        plugin._acp_callback = mock.Mock()
        return plugin, client._robot

    def test_complete_joint_target_is_sent_as_one_movej(self):
        plugin, robot = self._motion_plugin(current=[10, 20, 30, 40, 50, 60, 70])
        result = plugin._start_motion({
            "joint1_deg": 10, "joint2_deg": 20, "joint3_deg": 31,
            "joint4_deg": 40, "joint5_deg": 50, "joint6_deg": 60,
            "joint7_deg": 70, "speed_percent": 1, "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        self.assertEqual(([10, 20, 31, 40, 50, 60, 70], 1, 0, 0, 0), robot.moves[0])

    @staticmethod
    def _seven_targets(**overrides):
        values = {f"joint{i}_deg": 0 for i in range(1, 8)}
        values.update(overrides)
        return values

    def test_missing_joint_fields_keep_current_positions(self):
        current = [10, 20, 30, 40, 50, 60, 70]
        plugin, robot = self._motion_plugin(current=current)
        result = plugin._start_motion({"joint3_deg": 31, "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(([10, 20, 31, 40, 50, 60, 70], 5, 0, 0, 0), robot.moves[0])

    def test_motion_requires_both_interlocks(self):
        plugin, robot = self._motion_plugin(motion_enabled=False)
        with self.assertRaisesRegex(PermissionError, "motion is locked"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            plugin, robot = self._motion_plugin()
            plugin._start_motion(self._seven_targets(joint1_deg=1))
        self.assertEqual([], robot.moves)

    def test_motion_rejects_robot_error(self):
        plugin, robot = self._motion_plugin(all_state={
            "joint_err_code": [0, 0, 3, 0, 0, 0, 0],
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        })
        with self.assertRaisesRegex(RuntimeError, "joint error"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})

    def test_absolute_target_is_not_rejected_for_distance_from_current(self):
        plugin, robot = self._motion_plugin(current=[-10, 0, 0, 0, 0, 0, 0])
        result = plugin._start_motion({**self._seven_targets(joint1_deg=20), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(20, robot.moves[0][0][0])

    def test_zero_arm_error_code_is_not_treated_as_an_error(self):
        plugin, robot = self._motion_plugin(all_state={
            "joint_err_code": [0] * 7,
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 1, "err": ["0"]},
        })
        result = plugin._start_motion({**self._seven_targets(), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(1, len(robot.moves))

    def test_motion_reports_running_then_acp_completed(self):
        plugin, robot = self._motion_plugin(current=[0.0] * 7)
        result = plugin._start_motion({**self._seven_targets(), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("rm75_movej_"))
        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertEqual([0.0] * 7, completion["target_degree"])

    def test_motion_stall_requests_slow_stop_and_reports_acp_error(self):
        plugin, robot = self._motion_plugin(
            current=[0.0] * 7,
            safety={
                "start_grace_seconds": 0,
                "stall_timeout_seconds": 0.01,
                "progress_threshold_deg": 0.05,
            },
        )
        result = plugin._start_motion({
            **self._seven_targets(joint1_deg=1),
            "speed_percent": 1,
            "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("error", status)
        self.assertEqual("motion_stalled", completion["reason"])
        self.assertEqual(1, robot.stops)

    def test_agent_core_interrupt_hook_stops_pending_motion_through_mcp(self):
        runtime_spec = importlib.util.spec_from_file_location(
            "realman_interrupt_vendor_runtime", ROOT / "common" / "vendor_runtime.py"
        )
        runtime = importlib.util.module_from_spec(runtime_spec)
        runtime_spec.loader.exec_module(runtime)
        plugin, robot = self._motion_plugin(
            current=[0.0] * 7,
            safety={"start_grace_seconds": 60},
        )
        bundle = runtime.DriverBundle([plugin])
        handler_type = runtime.make_handler(lambda: bundle, "test", "test")

        def call_mcp(request_id, method, params):
            body = json.dumps({
                "jsonrpc": "2.0",
                "id": request_id,
                "method": method,
                "params": params,
            }).encode()
            handler = object.__new__(handler_type)
            handler.path = "/mcp"
            handler.headers = {"Content-Length": str(len(body))}
            handler.rfile = io.BytesIO(body)
            response = {}
            handler.send_json = lambda status, payload: response.update(
                status=status, payload=payload
            )
            handler.do_POST()
            self.assertEqual(200, response["status"])
            return response["payload"]

        listed = call_mcp(1, "tools/list", {})
        joint_control = next(
            item for item in listed["result"]["tools"] if item["name"] == "joint_control"
        )
        interrupt = joint_control["inputSchema"]["x-hooks"]["on_interrupt_motion"]
        self.assertEqual({"action": "stopmotion"}, interrupt)

        started_rpc = call_mcp(2, "tools/call", {
            "name": "joint_control",
            "arguments": {
                "action": "set",
                "joint1_deg": 1,
                "speed_percent": 1,
                "confirm_motion": True,
            },
        })
        started = json.loads(started_rpc["result"]["content"][0]["text"])
        self.assertEqual("running", started["state"])
        self.assertEqual(started["action_id"], plugin._active_action_id)

        stopped_rpc = call_mcp(3, "tools/call", {
            "name": "joint_control",
            "arguments": {"action": interrupt["action"]},
        })
        stopped = json.loads(stopped_rpc["result"]["content"][0]["text"])
        self.assertEqual("stop_requested", stopped["state"])
        self.assertEqual(started["action_id"], stopped["action_id"])
        self.assertEqual(1, robot.stops)

        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", completion["reason"])


if __name__ == "__main__":
    unittest.main()
