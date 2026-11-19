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
from unittest.mock import patch
from urllib.request import Request, urlopen
from uuid import uuid4

import yaml


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

    def test_production_config_keeps_guard_out_of_marketplace_and_mcp_discovery(self):
        manifest = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
        self.assertNotIn(self.guard.PREFIX, {card["name"] for card in manifest["cards"]})
        config = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
        guard_cfg = config["plugins"][self.guard.PREFIX]
        self.assertIs(guard_cfg["enabled"], False)

        # Exercise the production constructor with the default Guard config
        # and with a persisted config from a build that enabled it.
        for settings in (guard_cfg, {**guard_cfg, "enabled": True}):
            with self.subTest(enabled=settings["enabled"]):
                bundle = self.ns["BumiDeviceBundle"]({"plugins": {
                    self.guard.PREFIX: settings}}, "robot", object(), None, None)
                bundle._plugins.append(self.legacy)
                url = self.server(bundle)
                self.rpc(url, "initialize")
                listed = self.rpc(url, "tools/list")["result"]["tools"]
                self.assertEqual([tool["name"] for tool in listed], ["legacy_card"])
                call = self.rpc(url, "tools/call", {"name": self.guard.PREFIX,
                    "arguments": {"action": "start", "input_topic": "/robot/camera/color"}})
                self.assertEqual(call["error"]["code"], -32601)
                status = self.rpc(url, "motus.lifecycle.status", {"tool": self.guard.PREFIX})
                self.assertEqual(status["result"]["code"], "UNKNOWN_LIFECYCLE_TOOL")

    def test_test_entry_point_respects_disabled_guard_config(self):
        bundle = self.ns["BumiDeviceBundle"]({"plugins": {
            self.guard.PREFIX: {"enabled": False}}}, "robot", object(), None, None,
            allow_image_quality_guard_for_test=True)
        self.assertEqual(bundle.get_all_tools(), [])

    def test_compatible_core_can_discover_bind_start_and_stop_in_test_bundle(self):
        tool = self.guard.PREFIX
        with patch("image_quality_guard._runtime_factory", return_value=self.runtime) as factory:
            bundle = self.ns["BumiDeviceBundle"]({"plugins": {
                tool: {"enabled": True}}}, "robot", object(), None, None,
                allow_image_quality_guard_for_test=True)
        self.addCleanup(bundle.stop_all)
        bundle.start_all()
        factory.assert_not_called()
        url = self.server(bundle)
        tools = self.rpc(url, "tools/list")["result"]["tools"]
        self.assertEqual([definition["name"] for definition in tools], [tool])
        self.assertEqual(tools[0]["x-lifecycle"]["schema"], "motus.lifecycle/1")

        actual = self.rpc(url, "motus.lifecycle.status", {"tool": tool})["result"]
        self.assertFalse(actual["resources_pending"])
        epoch = str(uuid4())
        bound = self.rpc(url, "motus.lifecycle.bind", {"tool": tool,
            "driver_boot_id": actual["driver_boot_id"], "expected_epoch": actual["admission_epoch"],
            "proposed_epoch": epoch})["result"]
        self.assertEqual(bound["admission_epoch"], epoch)
        run_id = epoch + "." + str(uuid4())
        started = self.rpc(url, "tools/call", {"name": tool, "arguments": {
            "action": "start", "run_id": run_id, "input_topic": "/robot/camera/color"}})["result"]
        self.assertEqual(json.loads(started["content"][0]["text"])["active_run_id"], run_id)
        self.assertTrue(self.runtime.started)
        stopped = self.rpc(url, "tools/call", {"name": tool, "arguments": {
            "action": "stop", "run_id": run_id}})["result"]
        self.assertEqual(json.loads(stopped["content"][0]["text"])["state"], "idle")
        self.assertFalse(self.runtime.started)
        self.assertEqual(self.runtime.closes, 1)

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

    def server(self, bundle=None):
        self.ns["_bundle"] = self.bundle if bundle is None else bundle
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

    def test_real_http_failed_stop_keeps_owner_until_same_ticket_retry(self):
        self.assertEqual(self.start_guard()["state"], "running")
        url = self.server()
        target = {"tool": self.guard.PREFIX}
        stop = {"name": self.guard.PREFIX, "arguments": {"action": "stop", "run_id": self.run_id}}

        with patch.object(self.runtime, "close", side_effect=RuntimeError("temporary close failure")):
            failed = self.rpc(url, "tools/call", stop)["result"]
        self.assertEqual(json.loads(failed["content"][0]["text"])["code"], "CLEANUP_FAILED")
        pending = self.rpc(url, "motus.lifecycle.status", target)["result"]
        self.assertTrue(pending["resources_pending"])
        self.assertEqual(pending["active_run_id"], self.run_id)

        retried = self.rpc(url, "tools/call", stop)["result"]
        self.assertEqual(json.loads(retried["content"][0]["text"])["state"], "idle")
        status = self.rpc(url, "motus.lifecycle.status", target)["result"]
        self.assertFalse(status["resources_pending"])
        self.assertIsNone(status["active_run_id"])
        self.assertEqual(self.runtime.closes, 1)


if __name__ == "__main__":
    unittest.main()
