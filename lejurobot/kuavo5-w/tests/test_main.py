import importlib.util
import io
import json
import os
import shlex
import sys
import tempfile
import threading
import types
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("kuavo_main", ROOT / "main.py")
main = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"common": types.SimpleNamespace(logsafe=types.SimpleNamespace(install=lambda: None))}):
    spec.loader.exec_module(main)


class McpTests(unittest.TestCase):
    def request(self, value, path="/mcp"):
        body = value if isinstance(value, bytes) else json.dumps(value).encode()
        handler = main._handler().__new__(main._handler())
        handler.path, handler.rfile = path, io.BytesIO(body)
        handler.headers = {"Content-Length": str(len(body))}
        replies = []
        handler._reply = lambda status, payload: replies.append((status, payload))
        handler.do_POST()
        return replies[0]

    def rpc(self, name="loco", arguments=None):
        return {"jsonrpc": "2.0", "id": 7, "method": "tools/call",
                "params": {"name": name, "arguments": arguments or {}}}

    def test_http_path_and_invalid_json(self):
        self.assertEqual(self.request({}, "/other")[0], 404)
        self.assertEqual(self.request(b"{")[1]["error"]["code"], -32700)
        self.assertEqual(self.request([])[1]["error"]["code"], -32600)
        self.assertEqual(self.request({"jsonrpc": "2.0", "method": "tools/call", "params": []})[0], 400)
        self.assertEqual(self.request(b'{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"x":NaN}}')[0], 400)

    def test_wraps_plain_dict_exactly_once_and_reports_errors(self):
        with patch.object(main, "_bundle", types.SimpleNamespace(dispatch=lambda n, a: {"state": "error", "error": "paused"})):
            status, rpc = self.request(self.rpc())
        self.assertEqual(status, 200)
        self.assertTrue(rpc["result"]["isError"])
        self.assertEqual(json.loads(rpc["result"]["content"][0]["text"]), {"state": "error", "error": "paused"})

    def test_handler_catches_dispatch_exception(self):
        def broken(*args):
            raise RuntimeError("test failure")
        with patch.object(main, "_bundle", types.SimpleNamespace(dispatch=broken)):
            result = self.request(self.rpc())[1]["result"]
        self.assertTrue(result["isError"])
        self.assertIn("test failure", result["content"][0]["text"])

    def test_unknown_tool_and_bad_arguments(self):
        with patch.object(main, "_bundle", types.SimpleNamespace(dispatch=lambda n, a: None)):
            self.assertEqual(self.request(self.rpc("unknown"))[1]["error"]["code"], -32601)
        request = self.rpc()
        request["params"]["arguments"] = [1]
        self.assertEqual(self.request(request)[1]["error"]["code"], -32602)

    def test_initialize_and_list(self):
        request = {"jsonrpc": "2.0", "id": 1, "method": "initialize"}
        self.assertEqual(self.request(request)[1]["result"]["serverInfo"]["name"], "lejurobot-kuavo5-w")
        with patch.object(main, "_bundle", types.SimpleNamespace(get_all_tools=lambda: [{"name": "state"}])):
            request["method"] = "tools/list"
            self.assertEqual(self.request(request)[1]["result"]["tools"], [{"name": "state"}])


class DeploymentTests(unittest.TestCase):
    def test_registration_sends_standard_identity_and_transport(self):
        stopped, requests = threading.Event(), []
        def receive(request, **kwargs):
            requests.append(request)
            stopped.set()
            return io.BytesIO(b'{}')
        with patch.object(urllib.request, 'urlopen', side_effect=receive), patch.dict(os.environ, {'AGENT_CORE_URL': 'https://localhost:15678'}):
            thread = main._register({'name': 'Kuavo test', 'mcp_port': 15747}, stopped)
            try:
                thread.join(1)
                self.assertFalse(thread.is_alive())
            finally:
                stopped.set()
                thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].full_url, 'https://localhost:15678/api/mcp')
        self.assertEqual(json.loads(requests[0].data), {
            'id': 'lejurobot-kuavo5-w-driver', 'name': 'Kuavo test',
            'url': 'http://localhost:15747/mcp', 'transport': 'http', 'category': 'driver'})
        self.assertEqual(main.DRIVER_ID, yaml.safe_load((ROOT / 'driver.yaml').read_text())['id'])

    def test_dds_fails_closed_for_missing_or_directory_profile(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            with patch.object(main, "PROFILE", Path(directory)):
                with self.assertRaises(RuntimeError):
                    main._configure_dds()
            with patch.object(main, "PROFILE", Path(directory) / "missing.xml"):
                with self.assertRaises(RuntimeError):
                    main._configure_dds()

    def test_dds_profile_applied_before_init_and_conflicts_rejected(self):
        with tempfile.NamedTemporaryFile() as profile, patch.object(main, "PROFILE", Path(profile.name)), patch.dict(os.environ, {}, clear=True):
            main._configure_dds()
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "42")
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], profile.name)
            os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
            with self.assertRaises(RuntimeError):
                main._configure_dds()

    def test_metadata_config_compose_and_flat_build_context(self):
        meta = yaml.safe_load((ROOT / "driver.yaml").read_text())
        config = yaml.safe_load((ROOT / "config.yaml").read_text())
        self.assertIs(config['motion_enabled'], False)
        compose = next(iter(yaml.safe_load((ROOT / "deploy/service.yml").read_text()).values()))
        self.assertEqual(meta["port"], config["mcp_port"])
        self.assertEqual(meta["mcp_url"], f"http://localhost:{config['mcp_port']}/mcp")
        self.assertEqual(compose["network_mode"], "host")
        self.assertIn("ROS_DOMAIN_ID=42", compose["environment"])
        # build.sh flattens driver contents and copies extras under basename.
        for line in (ROOT / "Dockerfile").read_text().splitlines():
            if line.startswith("COPY "):
                for source in shlex.split(line)[1:-1]:
                    candidate = ROOT.parents[1] / "common" if source == "common/" else ROOT / source
                    self.assertTrue(candidate.exists(), f"COPY source missing: {source}")
        self.assertIn("COPY deploy/ /deploy/", (ROOT / "Dockerfile").read_text())
        self.assertIn("COPY resource/ /work/resource/", (ROOT / "Dockerfile").read_text())
        self.assertNotIn('ros-humble-geometry-msgs', (ROOT / 'Dockerfile').read_text())
        self.assertEqual({c["name"] for c in meta["cards"]}, set(config["plugins"]))

    def test_namespace_valid_first_character(self):
        self.assertEqual(main._namespace({"ros_namespace": "123robot-a"}), "robot_123robot_a")


if __name__ == "__main__":
    unittest.main()
