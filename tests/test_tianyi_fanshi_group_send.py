"""Offline tests: fake Webhook/ACP, real bundle routing and local MCP HTTP."""
import ast
import base64
from concurrent.futures import ThreadPoolExecutor
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(ROOT))
import fanshi_group_send as card
from common import lifecycle


def bundle_scope():
    tree = ast.parse((DRIVER / "main.py").read_text())
    selected = [node for node in tree.body if
                isinstance(node, (ast.FunctionDef, ast.ClassDef)) and
                node.name in ("TianyiDeviceBundle", "make_handler")]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, *selected], type_ignores=[]))
    scope = {"json": json, "threading": threading, "BaseHTTPRequestHandler": BaseHTTPRequestHandler,
             "_lifecycle": lifecycle}
    exec(compile(module, str(DRIVER / "main.py"), "exec"), scope)
    bundle = scope["TianyiDeviceBundle"]({"plugins": {"fanshi_group_send": {"enabled": True}}},
                                          "test", None, None)
    bundle.start_all()
    scope["_bundle"] = bundle
    return scope, bundle


class FanshiTests(unittest.TestCase):
    def setUp(self):
        self.plugin = card.FanshiGroupSendPlugin({}, "test")
        self.config = {"webhook_url": "https://example.invalid/hook?access_token=TEST_ONLY",
                       "group_id": "123456"}
        post = patch.object(card, "_post_json", return_value=(200, b'{"errcode":0,"errmsg":"ok","data":{"fail":{}}}'))
        complete = patch.object(card, "_complete")
        self.post = post.start()
        self.complete = complete.start()
        self.addCleanup(post.stop)
        self.addCleanup(complete.stop)

    def arm(self, plugin=None):
        plugin = plugin or self.plugin
        self.assertTrue(plugin.dispatch("config", self.config)["ok"])
        self.assertEqual(plugin.dispatch("start", {})["state"], "ready")

    def send(self, plugin=None, **args):
        return (plugin or self.plugin).dispatch("send", {"text": "测试", **args})

    def finish(self, queued, plugin=None):
        plugin = plugin or self.plugin
        self.assertEqual(queued["status"], "queued", queued)
        plugin._worker_thread.join(timeout=2)
        self.assertFalse(plugin._worker_thread.is_alive())
        return plugin.dispatch("result", {"action_id": queued["action_id"]})

    def test_startup_and_configuration_never_send(self):
        self.plugin.start()
        self.arm()
        self.post.assert_not_called()
        self.assertTrue(self.plugin.dispatch("info", {})["configured"])

    def test_unconfigured_or_stopped_never_sends(self):
        self.assertFalse(self.send()["ok"])
        self.arm()
        self.plugin.dispatch("stop", {})
        self.plugin.dispatch("config", self.config)
        self.assertFalse(self.send()["ok"])
        self.post.assert_not_called()

    def test_restart_after_stop_works(self):
        self.arm()
        self.plugin.dispatch("stop", {})
        self.plugin.dispatch("start", {})
        self.assertEqual(self.finish(self.send())["status"], "accepted")

    def test_send_uses_fixed_integer_target(self):
        self.arm()
        result = self.finish(self.send(group_id=999, webhook_url="https://example.invalid/other"))
        self.assertEqual(result["status"], "accepted")
        url, payload = self.post.call_args.args
        self.assertEqual(url, self.config["webhook_url"])
        self.assertEqual(payload["message"]["header"]["toid"], [123456])
        self.complete.assert_called_once()

    def test_invalid_config_invalidates_old_configuration(self):
        for bad in ({"group_id": ""}, {"group_id": True}, {"webhook_url": "http://example.invalid"},
                    {"webhook_url": "https://[bad"}):
            with self.subTest(bad=bad):
                self.arm()
                result = self.plugin.dispatch("config", {**self.config, **bad})
                self.assertFalse(result["ok"])
                self.assertFalse(result["adapter_ok"])
                self.assertNotIn("TEST_ONLY", json.dumps(result))
                self.assertFalse(self.send()["ok"])
        self.post.assert_not_called()

    def test_config_change_requires_restart(self):
        self.arm()
        self.plugin.dispatch("config", {**self.config, "group_id": "234567"})
        self.assertFalse(self.send()["ok"])
        self.plugin.dispatch("start", {})
        self.finish(self.send())
        self.assertEqual(self.post.call_args.args[1]["message"]["header"]["toid"], [234567])

    def test_no_empty_message_and_explicit_at(self):
        self.arm()
        self.assertFalse(self.send(text=" ")["ok"])
        self.post.assert_not_called()
        self.finish(self.send(at_userid="manager"))
        self.assertEqual(self.post.call_args.args[1]["message"]["body"][-1],
                         {"type": "AT", "atuserids": ["manager"], "atall": False})

    def test_response_semantics(self):
        cases = [
            (200, b'{"errcode":0,"errmsg":"ok","data":{"fail":{}}}', "accepted"),
            (200, b'{"errorcode":0}', "accepted"),
            (200, b'{"errcode":0,"errorcode":0}', "accepted"),
            (200, b'{"errcode":0,"errorcode":-1}', "unknown"),
            (200, b'{"errcode":false}', "unknown"),
            (200, b'{"errcode":"0"}', "unknown"),
            (200, b'{"errmsg":"ok"}', "unknown"),
            (200, b'{"errcode":-1}', "failed"),
            (200, b'{"errorcode":-1}', "failed"),
            (200, b'{"errcode":0,"data":{"fail":{"123456":40000}}}', "failed"),
            (200, b'{"errorcode":0,"data":{"fail":{"123456":40000}}}', "failed"),
            (200, b'{"errcode":0,"data":[]}', "unknown"),
            (200, b'<html/>', "unknown"), (200, b'[]', "unknown"),
            (200, b' ' * 65537, "unknown"), (500, b'', "unknown"),
            (408, b'', "unknown"), (403, b'', "failed"), (429, b'', "failed"),
        ]
        for code, raw, expected in cases:
            with self.subTest(code=code, raw=raw[:90]):
                self.assertEqual(card._interpret(code, raw)["status"], expected)

    def test_timeout_is_unknown_without_retry_or_secret(self):
        self.arm()
        self.post.side_effect = TimeoutError("TEST_ONLY")
        result = self.finish(self.send())
        self.assertEqual(result["status"], "unknown")
        self.assertNotIn("TEST_ONLY", json.dumps(result))
        self.assertEqual(self.post.call_count, 1)

    def test_image_bytes_and_size_boundary(self):
        self.arm()
        raw = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jQioAAAAASUVORK5CYII=")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            self.plugin._image_roots = (root,)
            photo = root / "photo.png"
            photo.write_bytes(raw)
            self.assertEqual(self.finish(self.send(image_path=str(photo)))["status"], "accepted")
            payload = self.post.call_args.args[1]
            self.assertEqual(base64.b64decode(payload["message"]["body"][1]["content"]), raw)
            photo.write_bytes(b"\x89PNG\r\n\x1a\n" + b"0" * 750000)
            self.assertFalse(self.send(image_path=str(photo))["ok"])
            self.assertEqual(self.post.call_count, 1)

    def test_image_path_and_symlink_escape_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            allowed = root / "allowed"
            allowed.mkdir()
            self.plugin._image_roots = (allowed,)
            outside = root / "outside.png"
            outside.write_bytes(b"\x89PNG\r\n\x1a\n")
            link = allowed / "link.png"
            link.symlink_to(outside)
            for value in (str(outside), str(link), "relative.png"):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    self.plugin._image_base64(value)

    def test_stop_is_prompt_while_request_is_inflight(self):
        self.arm()
        entered, release = threading.Event(), threading.Event()
        def slow(*args):
            entered.set()
            if not release.wait(3):
                raise TimeoutError()
            return 200, b'{"errcode":0}'
        self.post.side_effect = slow
        queued = self.send()
        self.assertTrue(entered.wait(1))
        try:
            with ThreadPoolExecutor(max_workers=1) as pool:
                stopped = pool.submit(self.plugin.dispatch, "stop", {}).result(timeout=1)
            self.assertEqual(stopped["state"], "idle")
            self.assertFalse(self.send()["ok"])
        finally:
            release.set()
        # Submitted messages may still succeed after stop.
        self.assertEqual(self.finish(queued)["status"], "accepted")

    def test_same_config_replay_preserves_busy_and_target(self):
        self.arm()
        entered, release = threading.Event(), threading.Event()
        def slow(*args):
            entered.set()
            release.wait(3)
            return 200, b'{"errcode":0}'
        self.post.side_effect = slow
        queued = self.send()
        self.assertTrue(entered.wait(1))
        try:
            self.plugin.dispatch("config", self.config)
            self.assertFalse(self.send()["ok"])
        finally:
            release.set()
        self.finish(queued)
        self.assertEqual(self.post.call_count, 1)

    def test_stop_before_worker_submission_cancels(self):
        self.arm()
        with patch.object(card.threading.Thread, "start"):
            queued = self.send()
        self.plugin.stop()
        self.plugin._worker(queued["action_id"], self.plugin._settings, self.plugin._generation - 1, {})
        self.post.assert_not_called()
        result = self.plugin.dispatch("result", {"action_id": queued["action_id"]})
        self.assertEqual(result["status"], "cancelled")

    def test_rate_limit_and_unknown_record(self):
        self.arm()
        with patch.object(card.time, "monotonic", return_value=1000):
            for _ in range(30):
                self.assertEqual(self.finish(self.send())["status"], "accepted")
            self.assertFalse(self.send()["ok"])
        self.assertEqual(self.post.call_count, 30)
        self.assertEqual(self.plugin.dispatch("result", {"action_id": "missing"})["status"], "unknown")

    def test_callback_failure_does_not_erase_result(self):
        self.arm()
        self.complete.side_effect = ValueError("TEST_ONLY")
        self.assertEqual(self.finish(self.send())["status"], "accepted")

    def test_schema_uses_driver_upload_and_shared_sensitive_config(self):
        tool = self.plugin.get_tool()
        self.assertFalse(tool["multiInstance"])
        self.assertEqual(tool["inputSchema"]["properties"]["image_path"]["uploadTo"], "mcp")
        for name in ("webhook_url", "group_id"):
            self.assertNotIn(name, tool["inputSchema"]["properties"])
            self.assertTrue(tool["configSchema"]["properties"][name]["x-sensitive"])
        self.assertEqual(tool["inputSchema"]["x-completion"]["actions"], ["send"])

    def test_bundle_and_mcp_http_discover_configure_start_send(self):
        scope, bundle = bundle_scope()
        server = ThreadingHTTPServer(("127.0.0.1", 0), scope["make_handler"]())
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def rpc(method, params):
            connection = http.client.HTTPConnection(*server.server_address, timeout=3)
            try:
                connection.request("POST", "/mcp", json.dumps({"jsonrpc": "2.0", "id": 1,
                    "method": method, "params": params}), {"Content-Type": "application/json"})
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                return json.loads(response.read())["result"]
            finally:
                connection.close()
        try:
            tools = rpc("tools/list", {})["tools"]
            self.assertEqual([t["name"] for t in tools], [card.FanshiGroupSendPlugin.PREFIX])
            for action, args in (("config", self.config), ("start", {}), ("send", {"text": "test"})):
                wrapped = rpc("tools/call", {"name": card.FanshiGroupSendPlugin.PREFIX,
                                            "arguments": {"action": action, **args}})
                self.assertFalse(wrapped.get("isError", False))
                reply = json.loads(wrapped["content"][0]["text"])
                self.assertIsInstance(reply, dict)
            self.assertEqual(self.finish(reply, bundle._plugins[0])["status"], "accepted")
            bad = rpc("tools/call", {"name": card.FanshiGroupSendPlugin.PREFIX,
                                     "arguments": {"action": "send", "text": ""}})
            self.assertTrue(bad["isError"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
