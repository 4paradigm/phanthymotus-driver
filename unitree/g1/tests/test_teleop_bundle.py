import contextlib
import importlib.util
import io
import json
import ssl
import subprocess
import sys
import threading
import types
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

from teleop.descriptor import tool_definitions


class FakeTeleopService:
    def __init__(self):
        self.blocked = False
        self.closed = False
        self.close_calls = 0
        self.dispatch_error = None
        self.runtime = types.SimpleNamespace(
            driver_id="unitree-g1",
            robot_id="unitree-g1",
            mode="shadow",
            profile_id="unitree_g1_23_dual_arm_controller_v1",
        )
        self._registration_status = {
            "state": "not_started",
            "attempts": 0,
            "successes": 0,
            "last_http_status": None,
            "last_error": None,
            "tls_verification": "pinned_certificate",
        }

    def get_tools(self):
        return [{"name": "teleop_session"}, {"name": "teleop_state"}]

    def dispatch(self, name, args):
        if self.dispatch_error is not None:
            raise self.dispatch_error
        return {"tool": name, "arguments": dict(args)}

    def blocks_arm_gesture(self):
        return self.blocked

    def close(self):
        self.close_calls += 1
        self.closed = True

    def preflight_status(self):
        return self.health()["preflight"]

    @property
    def driver_token(self):
        return "driver-token-123456789012"

    def authorized(self, header):
        return header == f"Bearer {self.driver_token}"

    def health(self):
        return {
            "ready": False,
            "preflight": {
                "schema": "motus.teleop.g1-preflight.v1",
                "ready": True,
                "mode": "shadow",
                "hardware_output": False,
                "publisher_created": False,
            },
        }

    @property
    def registration_status(self):
        return dict(self._registration_status)

    def update_registration_status(self, **updates):
        self._registration_status.update(updates)

    def launch_registration_worker(self, target):
        target()

    def registration_wait(self, timeout):
        return True


class FakeArmPlugin:
    def __init__(self, *args, **kwargs):
        pass

    def get_tool(self):
        return {"name": "arm", "type": "actuator"}

    def dispatch(self, action, args):
        return {"action": action, **args}

    def start(self):
        pass

    def stop(self):
        pass


def load_main_module():
    modules = {
        "yaml": types.SimpleNamespace(safe_load=lambda handle: {}),
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.executors": types.ModuleType("rclpy.executors"),
        "unitree_sdk2py.core.channel": types.SimpleNamespace(ChannelFactoryInitialize=lambda *args: None),
        "unitree_sdk2py.g1.audio.g1_audio_client": types.SimpleNamespace(AudioClient=object),
        "rpc_proxy": types.SimpleNamespace(RpcProxy=object),
        "unitree_sdk2py.g1.arm.g1_arm_action_client": types.SimpleNamespace(G1ArmActionClient=object),
        "unitree_sdk2py.g1.slam.slam_client": types.SimpleNamespace(SlamClient=object),
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client": types.SimpleNamespace(MotionSwitcherClient=object),
    }
    old = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        path = Path(__file__).parents[1] / "main.py"
        spec = importlib.util.spec_from_file_location("g1_main_under_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in old.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


class G1BundleTeleopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = load_main_module()

    def setUp(self):
        self.teleop = FakeTeleopService()
        self.bundle = self.main.G1DeviceBundle(
            {"plugins": {
                "arm": {"enabled": False},
                "smart_motion": {"enabled": False},
            }},
            "test",
            object(),
            object(),
            object(),
            object(),
            object(),
            object(),
            teleop_service=self.teleop,
        )

    def test_tools_list_exposes_teleop_on_the_existing_root_bundle(self):
        names = [tool["name"] for tool in self.bundle.get_all_tools()]
        self.assertEqual(names, ["teleop_session", "teleop_state"])
        self.assertEqual(
            self.bundle.dispatch("teleop_session", {"action": "status"}),
            {"tool": "teleop_session", "arguments": {"action": "status"}},
        )

    def test_teleop_enabled_never_constructs_or_exposes_arm_gesture(self):
        result = self.bundle.dispatch("arm", {"action": "wave"})
        self.assertEqual(result["code"], "teleop_arm_unavailable")

    def test_conflicting_teleop_and_arm_plugin_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "cannot share G1 arm authority"):
            self.main.G1DeviceBundle(
                {"plugins": {
                    "arm": {"enabled": True},
                    "smart_motion": {"enabled": False},
                }},
                "test",
                object(),
                object(),
                object(),
                object(),
                object(),
                object(),
                teleop_service=self.teleop,
            )

    def test_stop_all_closes_teleop_first_and_isolates_legacy_failures(self):
        events = []

        class OrderedTeleop(FakeTeleopService):
            def close(service_self):
                events.append("teleop")
                super().close()

        class Plugin:
            def __init__(self, name, *, fail=False):
                self.name = name
                self.fail = fail
                self.calls = 0

            def stop(self):
                self.calls += 1
                events.append(self.name)
                if self.fail:
                    raise RuntimeError("legacy stop failed")

        teleop = OrderedTeleop()
        bundle = self.main.G1DeviceBundle(
            {"plugins": {"smart_motion": {"enabled": False}}},
            "test",
            object(),
            object(),
            object(),
            object(),
            object(),
            object(),
            teleop_service=teleop,
        )
        failing = Plugin("failing", fail=True)
        healthy = Plugin("healthy")
        bundle._plugins = [failing, healthy]

        with (
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            bundle.stop_all()
            bundle.stop_all()

        self.assertEqual(["teleop", "failing", "healthy"], events)
        self.assertEqual(1, teleop.close_calls)
        self.assertEqual(1, failing.calls)
        self.assertEqual(1, healthy.calls)

    def test_main_closes_teleop_when_later_audio_initialization_fails(self):
        class FailingAudioClient:
            def SetTimeout(self, timeout):
                self.timeout = timeout

            def Init(self):
                raise RuntimeError("audio initialization failed")

        teleop = FakeTeleopService()
        config = {
            "teleop": {"enabled": True, "mode": "shadow"},
            "plugins": {"smart_motion": {"enabled": False}},
            "safety_harness": {"enabled": False},
        }
        previous_bundle = self.main._bundle
        previous_service = self.main._teleop_service
        try:
            with (
                patch.object(self.main, "_load_config", return_value=config),
                patch.object(self.main.sys, "argv", ["main.py", "eth0"]),
                patch(
                    "teleop.factory.build_g1_teleop_service",
                    return_value=teleop,
                ),
                patch.object(self.main, "AudioClient", FailingAudioClient),
                patch.object(self.main.os, "dup", return_value=100),
                patch.object(self.main.os, "open", return_value=101),
                patch.object(self.main.os, "dup2"),
                patch.object(self.main.os, "close"),
                patch.object(self.main.os, "fdopen", return_value=sys.stdout),
                self.assertRaisesRegex(RuntimeError, "audio initialization failed"),
            ):
                self.main.main()
        finally:
            self.main._bundle = previous_bundle
            self.main._teleop_service = previous_service

        self.assertTrue(teleop.closed)
        self.assertEqual(1, teleop.close_calls)

    def test_process_preflight_failure_is_nonzero_without_secret_in_either_stream(self):
        secret = "operator-secret-never-public"
        private_path = "/private/operator/g1-live.yaml"
        script = f'''\
from unittest.mock import patch
from tests.test_teleop_bundle import load_main_module

module = load_main_module()
config = {{"teleop": {{"enabled": True, "mode": "live"}}}}
with (
    patch.object(module, "_load_config", return_value=config),
    patch.object(module.sys, "argv", ["main.py", "eth0"]),
    patch(
        "teleop.factory.build_g1_teleop_service",
        side_effect=RuntimeError("failed at {private_path} token={secret}"),
    ),
):
    raise SystemExit(module.main())
'''
        completed = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).parents[1],
            capture_output=True,
            check=False,
            text=True,
            timeout=10,
        )

        self.assertEqual(1, completed.returncode)
        projected_line = next(
            line
            for line in completed.stdout.splitlines()
            if line.startswith("[teleop-preflight] ")
        )
        projected = json.loads(projected_line.removeprefix("[teleop-preflight] "))
        self.assertEqual("teleop_configuration_invalid", projected["code"])
        self.assertEqual(
            "G1 teleoperation configuration is invalid",
            projected["message"],
        )
        combined = completed.stdout + completed.stderr
        self.assertNotIn(secret, combined)
        self.assertNotIn(private_path, combined)
        self.assertEqual("", completed.stderr)

    def test_teleop_disabled_preserves_legacy_arm_tool(self):
        previous = sys.modules.get("device")
        sys.modules["device"] = types.SimpleNamespace(ArmActionPlugin=FakeArmPlugin)
        try:
            bundle = self.main.G1DeviceBundle(
                {"plugins": {
                    "arm": {"enabled": True},
                    "smart_motion": {"enabled": False},
                }},
                "test",
                object(),
                object(),
                object(),
                object(),
                object(),
                object(),
                teleop_service=None,
            )
        finally:
            if previous is None:
                sys.modules.pop("device", None)
            else:
                sys.modules["device"] = previous
        self.assertEqual(["arm"], [tool["name"] for tool in bundle.get_all_tools()])
        self.assertEqual("wave", bundle.dispatch("arm", {"action": "wave"})["action"])

    def test_same_origin_mcp_requires_exact_driver_bearer(self):
        previous_bundle = self.main._bundle
        previous_service = self.main._teleop_service
        self.main._bundle = self.bundle
        self.main._teleop_service = self.teleop
        server = self.main.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            self.main.make_handler(),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        endpoint = f"http://127.0.0.1:{server.server_port}/mcp"
        body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "initialize"}).encode()
        try:
            for header in (None, "Bearer wrong-token-123456789"):
                headers = {"Content-Type": "application/json"}
                if header is not None:
                    headers["Authorization"] = header
                request = urllib.request.Request(endpoint, data=body, headers=headers)
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    urllib.request.urlopen(request, timeout=1)
                self.assertEqual(401, caught.exception.code)
                caught.exception.read()
                caught.exception.close()

            request = urllib.request.Request(
                endpoint,
                data=body,
                headers={
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {self.teleop.driver_token}",
                },
            )
            with urllib.request.urlopen(request, timeout=1) as response:
                self.assertEqual(200, response.status)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1)
            self.main._bundle = previous_bundle
            self.main._teleop_service = previous_service

    def test_authenticated_health_exposes_zero_output_preflight(self):
        previous_service = self.main._teleop_service
        self.main._teleop_service = self.teleop
        server = self.main.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            self.main.make_handler(),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        endpoint = f"http://127.0.0.1:{server.server_port}/health"
        try:
            unauthorized = urllib.request.Request(endpoint)
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(unauthorized, timeout=1)
            self.assertEqual(401, caught.exception.code)
            caught.exception.close()

            request = urllib.request.Request(
                endpoint,
                headers={"Authorization": f"Bearer {self.teleop.driver_token}"},
            )
            with urllib.request.urlopen(request, timeout=1) as response:
                payload = json.load(response)
            self.assertTrue(payload["preflight"]["ready"])
            self.assertEqual("shadow", payload["preflight"]["mode"])
            self.assertFalse(payload["preflight"]["hardware_output"])
            self.assertFalse(payload["preflight"]["publisher_created"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1)
            self.main._teleop_service = previous_service

    def test_registration_tls_is_pinned_and_cannot_be_disabled(self):
        with self.assertRaisesRegex(ValueError, "requires TLS verification"):
            self.main.build_registration_ssl_context({"verify_tls": False})
        with self.assertRaisesRegex(ValueError, "pinned CA file is missing"):
            self.main.build_registration_ssl_context({"ca_file": "/missing/core-ca.pem"})
        ca_file = ssl.get_default_verify_paths().cafile
        if not ca_file or not Path(ca_file).is_file():
            self.skipTest("system CA bundle unavailable")
        context = self.main.build_registration_ssl_context({"ca_file": ca_file})
        self.assertEqual(ssl.CERT_REQUIRED, context.verify_mode)
        self.assertFalse(context.check_hostname)

    def test_registration_echoes_descriptor_identity_and_uses_bearer(self):
        ca_file = ssl.get_default_verify_paths().cafile
        if not ca_file or not Path(ca_file).is_file():
            self.skipTest("system CA bundle unavailable")

        class Response:
            status = 200

            def read(self, maximum):
                return json.dumps({
                    "code": 200,
                    "data": {"id": "unitree-g1", "trust_state": "trusted"},
                }).encode()

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        with patch("urllib.request.urlopen", return_value=Response()) as opened:
            evidence = self.main._start_registration(
                15701,
                "Unitree G1 Bundle",
                "driver",
                cfg={
                    "teleop": {
                        "registration": {
                            "agent_core_url": "https://localhost:15678",
                            "ca_file": ca_file,
                        }
                    }
                },
                teleop_service=self.teleop,
            )
        self.assertEqual("unitree-g1", evidence["payload"]["id"])
        self.assertEqual("unitree-g1", evidence["payload"]["robot_id"])
        for tool in tool_definitions(mode="shadow"):
            self.assertEqual(evidence["payload"]["id"], tool["x-teleop"]["driver_id"])
            self.assertEqual(
                evidence["payload"]["robot_id"],
                tool["x-teleop"]["robot_id"],
            )
        request = opened.call_args.args[0]
        self.assertEqual(
            f"Bearer {self.teleop.driver_token}",
            request.get_header("Authorization"),
        )
        self.assertEqual(ssl.CERT_REQUIRED, evidence["tls_verify_mode"])
        self.assertEqual("registered", self.teleop.registration_status["state"])
        self.assertEqual(1, self.teleop.registration_status["attempts"])
        self.assertEqual(1, self.teleop.registration_status["successes"])

    def test_registration_http_200_without_trust_never_turns_health_green(self):
        ca_file = ssl.get_default_verify_paths().cafile
        if not ca_file or not Path(ca_file).is_file():
            self.skipTest("system CA bundle unavailable")

        class Response:
            status = 200

            def read(self, maximum):
                return json.dumps({
                    "code": 200,
                    "data": {"id": "unitree-g1", "trust_state": "untrusted"},
                }).encode()

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        with patch("urllib.request.urlopen", return_value=Response()):
            self.main._start_registration(
                15701,
                "Unitree G1 Bundle",
                "driver",
                cfg={"teleop": {"registration": {"ca_file": ca_file}}},
                teleop_service=self.teleop,
            )
        self.assertEqual("trust_error", self.teleop.registration_status["state"])
        self.assertEqual(1, self.teleop.registration_status["attempts"])
        self.assertEqual(0, self.teleop.registration_status["successes"])

    def test_mcp_protocol_error_preserves_stable_code_in_jsonrpc_data(self):
        class FakeProtocolError(ValueError):
            def __init__(self, code, message):
                super().__init__(message)
                self.code = code

        previous_bundle = self.main._bundle
        previous_service = self.main._teleop_service
        previous_error = self.main.TeleopProtocolError
        self.main._bundle = self.bundle
        self.main._teleop_service = self.teleop
        self.main.TeleopProtocolError = FakeProtocolError
        self.teleop.dispatch_error = FakeProtocolError(
            "session_inactive",
            "a new prepare_shadow session is required",
        )
        server = self.main.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            self.main.make_handler(),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        endpoint = f"http://127.0.0.1:{server.server_port}/mcp"
        request = urllib.request.Request(
            endpoint,
            data=json.dumps({
                "jsonrpc": "2.0",
                "id": 7,
                "method": "tools/call",
                "params": {
                    "name": "teleop_session",
                    "arguments": {"action": "heartbeat"},
                },
            }).encode(),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.teleop.driver_token}",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=1) as response:
                payload = json.load(response)
            self.assertEqual(-32602, payload["error"]["code"])
            self.assertEqual(
                {"code": "session_inactive"},
                payload["error"]["data"],
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1)
            self.main._bundle = previous_bundle
            self.main._teleop_service = previous_service
            self.main.TeleopProtocolError = previous_error

    def test_legacy_plugin_value_error_remains_internal_error(self):
        class FailingBundle:
            def dispatch(self, name, args):
                raise ValueError("legacy plugin bug")

        previous_bundle = self.main._bundle
        previous_service = self.main._teleop_service
        self.main._bundle = FailingBundle()
        self.main._teleop_service = None
        server = self.main.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            self.main.make_handler(),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({
                "jsonrpc": "2.0",
                "id": 8,
                "method": "tools/call",
                "params": {"name": "arm", "arguments": {"action": "wave"}},
            }).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=1) as response:
                payload = json.load(response)
            self.assertEqual(-32603, payload["error"]["code"])
            self.assertNotIn("data", payload["error"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1)
            self.main._bundle = previous_bundle
            self.main._teleop_service = previous_service


if __name__ == "__main__":
    unittest.main()
