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
from email.message import Message
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("kuavo_main", ROOT / "main.py")
main = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"common": types.SimpleNamespace(logsafe=types.SimpleNamespace(install=lambda: None))}):
    spec.loader.exec_module(main)


class McpTests(unittest.TestCase):
    def setUp(self):
        self.log_patch = patch.object(main, '_request_log', main._McpRequestLog())
        self.log_patch.start()
        self.addCleanup(self.log_patch.stop)

    def http_request(self, headers=(), peer='127.0.0.1', method='POST'):
        # Exercise BaseHTTPRequestHandler's real HTTP parser and response writer.
        body = json.dumps(self.rpc()).encode()
        lines = [f'{method} /mcp HTTP/1.1', f'Content-Length: {len(body)}']
        lines += [f'{name}: {value}' for name, value in headers]
        wire = '\r\n'.join(lines).encode() + b'\r\n\r\n' + body
        class Connection:
            def __init__(self):
                self.output = bytearray()
            def makefile(self, *args):
                return io.BytesIO(wire)
            def sendall(self, data):
                self.output.extend(data)
        connection = Connection()
        main._handler()(connection, (peer, 12345), types.SimpleNamespace(server_port=15749))
        response = bytes(connection.output)
        head, content = response.split(b'\r\n\r\n', 1)
        return int(head.split(b' ')[1]), head, json.loads(content)

    def test_http_rejects_browser_remote_and_rebinding_requests_before_dispatch(self):
        cases = [
            ([('Host', 'localhost:15749'), ('Origin', 'https://attacker.example')], '127.0.0.1', 'POST'),
            ([('Host', 'localhost:15749'), ('Origin', 'null')], '127.0.0.1', 'POST'),
            ([('Host', 'localhost:15749'), ('Origin', '')], '127.0.0.1', 'POST'),
            ([('Host', 'attacker.example:15749')], '127.0.0.1', 'POST'),
            ([('Host', 'localhost:15749'), ('Host', 'attacker.example')], '127.0.0.1', 'POST'),
            ([], '127.0.0.1', 'POST'),
            ([('Host', 'localhost:15749')], '192.168.1.50', 'POST'),
            ([('Host', '127.0.0.1:15749'), ('Origin', 'https://attacker.example')], '127.0.0.1', 'OPTIONS'),
        ]
        bundle = types.SimpleNamespace(dispatch=lambda *a: self.fail('rejected HTTP request dispatched motion'))
        with patch.object(main, '_bundle', bundle):
            for headers, peer, method in cases:
                with self.subTest(headers=headers, peer=peer, method=method):
                    status, head, _ = self.http_request(headers, peer, method)
                    self.assertEqual(status, 403)
                    self.assertNotIn(b'Access-Control-Allow-Origin', head)

    def test_http_accepts_loopback_core_request_without_browser_origin(self):
        calls = []
        with patch.object(main, '_bundle', types.SimpleNamespace(dispatch=lambda *a: calls.append(a) or {'state': 'ready'})):
            for host in ('localhost:15749', '127.0.0.1:15749'):
                status, head, response = self.http_request([('Host', host)])
                self.assertEqual(status, 200)
                self.assertNotIn(b'Access-Control-Allow-Origin', head)
                self.assertFalse(response['result'].get('isError', False))
        self.assertEqual(calls, [('base_drive', {}), ('base_drive', {})])

    def test_successful_poll_logs_are_sampled_across_http_connections(self):
        rpc = self.rpc('state', {'action': 'info'})
        clock = [0.]
        with patch.object(self, 'rpc', return_value=rpc), patch.object(main, 'time', types.SimpleNamespace(monotonic=lambda: clock[0])), patch.object(
                main, '_bundle', types.SimpleNamespace(dispatch=lambda *args: {'state': 'running'})), patch('builtins.print') as log:
            for _ in range(100):
                self.assertEqual(self.http_request([('Host', 'localhost:15749')])[0], 200)
            self.assertEqual(log.call_count, 1)
            clock[0] = 5.
            self.http_request([('Host', 'localhost:15749')])
            self.assertEqual(log.call_count, 2)
            self.assertIn('suppressed=99', log.call_args[0][0])

    def test_http_200_tool_errors_are_not_classified_as_successful_polls(self):
        rpc = self.rpc('state', {'action': 'info'})
        values = iter([{'state': 'running'}, {'state': 'error', 'error': 'test failure'}])
        with patch.object(self, 'rpc', return_value=rpc), patch.object(main, '_bundle',
                types.SimpleNamespace(dispatch=lambda *args: next(values))), patch('builtins.print') as log:
            self.http_request([('Host', 'localhost:15749')])
            status, _, response = self.http_request([('Host', 'localhost:15749')])
        self.assertEqual(status, 200)
        self.assertTrue(response['result']['isError'])
        self.assertEqual(log.call_count, 2)
        self.assertIn('outcome=error', log.call_args[0][0])

    def test_meaningful_card_state_transitions_are_logged_separately(self):
        states = iter(['ready', 'ready', 'executing', 'executing', 'paused', 'paused'])
        with patch.object(main, '_bundle', types.SimpleNamespace(dispatch=lambda *args: {'state': next(states)})), patch('builtins.print') as log:
            for _ in range(6):
                self.http_request([('Host', 'localhost:15749')])
        self.assertEqual(log.call_count, 3)
        for call, state in zip(log.call_args_list, ('ready', 'executing', 'paused')):
            self.assertIn(f'tool=base_drive state={state}', call[0][0])

    def test_request_log_sampler_is_shared_and_bounded_under_concurrency(self):
        logger = main._McpRequestLog()
        def emit():
            for _ in range(100):
                logger.emit('success')
        with patch.object(main, 'time', types.SimpleNamespace(monotonic=lambda: 0.)), patch('builtins.print') as log:
            threads = [threading.Thread(target=emit) for _ in range(8)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(1)
                self.assertFalse(thread.is_alive())
        self.assertEqual(log.call_count, 1)
        self.assertEqual(logger._buckets['ok']['suppressed'], 799)
        self.assertEqual(len(logger._buckets), 1)

    def test_sampled_request_logs_still_escape_and_cap_untrusted_lines(self):
        handler_type = main._handler()
        handler = handler_type.__new__(handler_type)
        with patch('builtins.print') as log:
            handler.log_message('%s', '\n\x00\x1b' + 'x' * 10000)
        line = log.call_args[0][0]
        self.assertNotIn('\n', line)
        self.assertNotIn('\x00', line)
        self.assertNotIn('\x1b', line)
        self.assertLess(len(line), 300)

    def request(self, value, path="/mcp"):
        body = value if isinstance(value, bytes) else json.dumps(value).encode()
        handler = main._handler().__new__(main._handler())
        handler.path, handler.rfile = path, io.BytesIO(body)
        handler.headers = Message()
        handler.headers['Content-Length'] = str(len(body))
        handler.headers['Host'] = 'localhost:15749'
        handler.client_address = ('127.0.0.1', 12345)
        handler.server = types.SimpleNamespace(server_port=15749)
        replies = []
        handler._reply = lambda status, payload: replies.append((status, payload))
        handler.do_POST()
        return replies[0]

    def rpc(self, name="base_drive", arguments=None):
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
    def test_mcp_binds_loopback_only(self):
        with patch.object(main, 'ThreadingHTTPServer') as server:
            main._mcp_server(15749)
        self.assertEqual(server.call_args[0][0], ('127.0.0.1', 15749))
    def test_registration_sends_standard_identity_and_transport(self):
        stopped, requests = threading.Event(), []
        def receive(request, **kwargs):
            requests.append(request)
            stopped.set()
            return io.BytesIO(b'{}')
        with patch.object(urllib.request, 'urlopen', side_effect=receive), patch.dict(os.environ, {'AGENT_CORE_URL': 'https://localhost:15678'}):
            thread = main._register({'name': 'Kuavo test', 'mcp_port': 15749}, stopped)
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
            'url': 'http://localhost:15749/mcp', 'transport': 'http', 'category': 'driver'})
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
        self.assertIs(config['motion_enabled'], True)
        self.assertIsNone(config['sensors']['joint_limits'])
        compose = next(iter(yaml.safe_load((ROOT / "deploy/service.yml").read_text()).values()))
        self.assertEqual(meta["port"], config["mcp_port"])
        self.assertEqual(meta["mcp_url"], f"http://localhost:{config['mcp_port']}/mcp")
        self.assertEqual(compose["network_mode"], "host")
        self.assertFalse(compose.get('privileged', False))
        self.assertEqual(compose['volumes'], [
            '/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro',
            '/opt/phanthy-motus/data:/opt/phanthy-motus/data',
            {'type': 'bind', 'source': '/opt/phanthy-motus/drivers/lejurobot-kuavo5-w',
             'target': '/config', 'bind': {'create_host_path': True}}])
        self.assertIn('CONFIG_PATH=/config/config.yaml', compose['environment'])
        self.assertIn('KUAVO_CONFIG_SEED=1', compose['environment'])
        self.assertTrue(config['host_runtime']['controller_boot_required'])
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

    def test_operator_config_loads_complete_commissioning_record(self):
        config = yaml.safe_load((ROOT / 'config.yaml').read_text())
        # Loader round-trip only; not a valid or hardware-verified motion policy.
        config['sensors']['joint_limits'] = {'verification': 'OFFLINE LOADER TEST ONLY'}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'operator.yaml'
            path.write_text(yaml.safe_dump(config))
            with patch.dict(os.environ, {'CONFIG_PATH': str(path)}):
                self.assertEqual(main._load_config(), config)
        self.assertIsNone(yaml.safe_load((ROOT / 'config.yaml').read_text())['sensors']['joint_limits'])

    def test_explicit_operator_config_missing_or_invalid_never_falls_back(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'operator.yaml'
            with patch.dict(os.environ, {'CONFIG_PATH': str(path)}):
                with self.assertRaises(FileNotFoundError):
                    main._load_config()
                path.write_text('sensors: [')
                with self.assertRaises(yaml.YAMLError):
                    main._load_config()

    def test_legacy_port_migration_preserves_other_settings_and_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            for port, expected in ((15747, 15749), (15770, 15770)):
                original = yaml.safe_dump({'mcp_port': port, 'motion_enabled': False})
                path.write_text(original)
                with patch.dict(os.environ, {'CONFIG_PATH': str(path)}):
                    self.assertEqual(main._load_config(), {'mcp_port': expected, 'motion_enabled': False})
                self.assertEqual(path.read_text(), original)

    def test_fresh_deployment_seeds_only_designated_config_and_preserves_existing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'new/config.yaml'
            with patch.dict(os.environ, {'CONFIG_PATH': str(path), 'KUAVO_STATE_DIR': str(path.parent),
                                         'KUAVO_CONFIG_SEED': '1'}):
                config = main._load_config()
                self.assertTrue(config['provisioning']['enabled'])
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                path.write_text('motion_enabled: false\n')
                self.assertEqual(main._load_config(), {'motion_enabled': False})
            with patch.dict(os.environ, {'CONFIG_PATH': str(path.parent / 'missing.yaml'),
                                         'KUAVO_STATE_DIR': str(path.parent), 'KUAVO_CONFIG_SEED': '1'}):
                with self.assertRaises(FileNotFoundError):
                    main._load_config()

    def test_deployment_without_url_override_uses_operator_config(self):
        compose = next(iter(yaml.safe_load((ROOT / 'deploy/service.yml').read_text()).values()))
        self.assertIn('ROSBRIDGE_URL=${KUAVO_ROSBRIDGE_URL:-}', compose['environment'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text('rosbridge_url: ws://192.0.2.10:9090\n')
            with patch.dict(os.environ, {'CONFIG_PATH': str(path), 'ROSBRIDGE_URL': ''}):
                self.assertEqual(main._rosbridge_url(main._load_config()), 'ws://192.0.2.10:9090')
                os.environ['ROSBRIDGE_URL'] = 'ws://192.0.2.20:9090'
                self.assertEqual(main._rosbridge_url(main._load_config()), 'ws://192.0.2.20:9090')

    def test_url_defaults_when_neither_env_nor_config_specifies_one(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(main._rosbridge_url({}), 'ws://127.0.0.1:9090')
            self.assertEqual(main._rosbridge_url({'rosbridge_url': 'ws://192.0.2.10:9090'}),
                             'ws://192.0.2.10:9090')

    def test_namespace_valid_first_character(self):
        self.assertEqual(main._namespace({"ros_namespace": "123robot-a"}), "robot_123robot_a")


if __name__ == "__main__":
    unittest.main()
