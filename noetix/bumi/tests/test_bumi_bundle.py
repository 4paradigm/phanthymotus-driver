"""Actual bundle/HTTP methods, isolated from main's robot and ROS imports.

These tests run a loopback MCP server and inert test runtimes. They do not
initialize ROS2, a vendor SDK, a camera, or a real Bumi device.
"""

from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from uuid import uuid4


BUMI = Path(__file__).resolve().parents[1]
if str(BUMI) not in sys.path:
    sys.path.insert(0, str(BUMI))

from image_quality_guard import ImageQualityGuardPlugin


def definitions():
    tree = ast.parse((BUMI / "main.py").read_text(encoding="utf-8"))
    selected = [node for node in tree.body
                if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                and node.name in {"BumiDeviceBundle", "make_handler"}]
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[
        ast.alias(name="annotations")], level=0), *selected], type_ignores=[])
    namespace = {"uuid4": uuid4, "os": os, "json": json,
                 "BaseHTTPRequestHandler": BaseHTTPRequestHandler}
    exec(compile(ast.fix_missing_locations(module), "bumi_contract", "exec"), namespace)
    return namespace


class Runtime:
    def __init__(self):
        self.closes = 0
        self.started = False

    def start(self, **kwargs):
        self.started = True

    def close(self):
        self.closes += 1
        self.started = False


class LegacyPlugin:
    def __init__(self):
        self.calls = []

    def get_tool(self):
        return {"name": "legacy_card", "type": "processor", "inputSchema": {}}

    def start(self):
        self.calls.append("boot")

    def stop(self):
        self.calls.append("local_stop")

    def dispatch(self, action, args):
        self.calls.append((action, dict(args)))
        return {"state": "idle" if action == "stop" else "running"}


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.ns = definitions()
        self.bundle = self.ns["BumiDeviceBundle"].__new__(self.ns["BumiDeviceBundle"])
        self.runtime = Runtime()
        self.guard = ImageQualityGuardPlugin({}, "robot", object(),
            runtime_factory=lambda **kwargs: self.runtime)
        self.addCleanup(self.guard.shutdown)
        self.legacy = LegacyPlugin()
        self.bundle._plugins = [self.legacy, self.guard]
        self.epoch = str(uuid4())
        self.bundle.lifecycle_dispatch("motus.lifecycle.bind", {
            "tool": self.guard.PREFIX, "driver_boot_id": self.guard.info()["driver_boot_id"],
            "expected_epoch": None, "proposed_epoch": self.epoch})
        self.run_id = self.epoch + "." + str(uuid4())

    def start_guard(self):
        return self.bundle.dispatch(self.guard.PREFIX, {"action": "start",
            "input_topic": "/robot/camera/color", "run_id": self.run_id})

    def test_configured_bundle_registers_guard_without_boot_subscription(self):
        bundle = self.ns["BumiDeviceBundle"]({"plugins": {
            "image_quality_guard": {"enabled": True}}}, "robot", object(), None, None)
        self.addCleanup(bundle.stop_all)
        tools = bundle.get_all_tools()
        self.assertEqual([tool["name"] for tool in tools], ["image_quality_guard"])
        self.assertEqual(tools[0]["x-lifecycle"]["schema"], "motus.lifecycle/1")
        bundle.start_all()
        self.assertFalse(bundle._plugins[0].info()["resources_pending"])

    def test_legacy_card_arguments_and_local_stop_stay_compatible(self):
        args = {"action": "start", "run_id": "legacy-business-value", "value": 4}
        before = dict(args)
        self.assertEqual(self.bundle.dispatch("legacy_card", args)["state"], "running")
        self.assertEqual(args, before)
        self.assertEqual(self.legacy.calls[-1], ("start", {
            "run_id": "legacy-business-value", "value": 4, "_tool_name": "legacy_card"}))
        self.bundle.stop_all()
        self.assertIn("local_stop", self.legacy.calls)

    def test_dispatch_passes_ticket_unchanged_without_mutating_request(self):
        args = {"action": "start", "input_topic": "/robot/camera/color", "run_id": self.run_id}
        before = dict(args)
        self.assertEqual(self.bundle.dispatch(self.guard.PREFIX, args)["run_id"], self.run_id)
        self.assertEqual(args, before)

    def test_local_stop_all_uses_shutdown_and_external_unqualified_stop_is_rejected(self):
        self.start_guard()
        self.assertEqual(self.bundle.dispatch(self.guard.PREFIX, {"action": "stop"})["code"], "MISSING_RUN_ID")
        self.assertTrue(self.runtime.started)
        self.bundle.stop_all()
        self.bundle.stop_all()
        self.assertFalse(self.runtime.started)
        self.assertEqual(self.runtime.closes, 1)
        self.assertTrue(self.guard.info()["shutdown"])

    def test_bad_lifecycle_rpc_does_not_target_legacy_card(self):
        result = self.bundle.lifecycle_dispatch("motus.lifecycle.status", {"tool": "legacy_card"})
        self.assertEqual(result["code"], "UNKNOWN_LIFECYCLE_TOOL")
        self.assertEqual(self.bundle.lifecycle_dispatch("motus.lifecycle.status", [1])["code"], "INVALID_ARGUMENT")

    def test_shutdown_failure_keeps_owner_does_not_skip_legacy_and_retries(self):
        self.start_guard()
        original_close = self.runtime.close
        failures = [True]

        def fail_once():
            if failures:
                failures.pop()
                raise RuntimeError("temporary resource cleanup failure")
            original_close()

        self.runtime.close = fail_once
        self.bundle.stop_all()
        self.assertTrue(self.guard.info()["shutdown"])
        self.assertTrue(self.guard.info()["resources_pending"])
        self.assertEqual(self.guard.info()["run_id"], self.run_id)
        self.assertIn("local_stop", self.legacy.calls)
        self.assertEqual(self.start_guard()["code"], "SHUTDOWN")
        self.bundle.stop_all()
        self.assertFalse(self.guard.info()["resources_pending"])
        self.assertEqual(self.runtime.closes, 1)
        self.bundle.stop_all()
        self.assertEqual(self.runtime.closes, 1)

    def server(self):
        self.ns["_bundle"] = self.bundle
        server = ThreadingHTTPServer(("127.0.0.1", 0), self.ns["make_handler"]())
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
        thread.start()

        def cleanup():
            server.shutdown()
            server.server_close()
            thread.join(2)
            self.assertFalse(thread.is_alive())

        self.addCleanup(cleanup)
        return "http://127.0.0.1:" + str(server.server_port) + "/mcp"

    def rpc(self, url, method, params=None):
        request = Request(url, data=json.dumps({"jsonrpc": "2.0", "id": 1,
            "method": method, "params": params or {}}).encode(),
            headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=2) as response:
            return json.load(response)

    def test_real_http_tools_list_retains_top_level_capability(self):
        result = self.rpc(self.server(), "tools/list")["result"]
        guard = next(tool for tool in result["tools"] if tool["name"] == self.guard.PREFIX)
        self.assertEqual(guard["x-lifecycle"]["id_parameter"], "run_id")
        self.assertIn("run_id", guard["inputSchema"]["properties"])

    def test_real_http_initialize_cannot_reset_bound_session_or_active_run(self):
        self.start_guard()
        url = self.server()
        boot = self.guard.lifecycle_status()
        self.assertEqual(self.rpc(url, "initialize")["result"]["protocolVersion"], "2024-11-05")
        status = self.rpc(url, "motus.lifecycle.status", {"tool": self.guard.PREFIX})["result"]
        self.assertEqual(status["admission_epoch"], boot["admission_epoch"])
        self.assertEqual(status["run_id"], self.run_id)
        self.assertTrue(self.runtime.started)

    def test_real_http_call_returns_plain_plugin_result_wrapped_once(self):
        url = self.server()
        result = self.rpc(url, "tools/call", {"name": self.guard.PREFIX,
            "arguments": {"action": "start", "input_topic": "/robot/camera/color",
                          "run_id": self.run_id}})["result"]
        body = json.loads(result["content"][0]["text"])
        self.assertEqual(body["run_id"], self.run_id)
        self.assertEqual(body["state"], "running")
        self.assertNotIn("content", body)
        denied = self.rpc(url, "tools/call", {"name": self.guard.PREFIX,
            "arguments": {"action": "shutdown"}})
        self.assertEqual(denied["error"]["code"], -32601)


if __name__ == "__main__":
    unittest.main()
