"""MCP/ACP transport contracts against local servers, with no robot connection."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import ssl
import subprocess
import tempfile
import threading
import unittest
from unittest import mock
import urllib.request

import test_realman_pick_place as observation
import test_realman_pick_place_transfer as transfer
from common.vendor_runtime import DriverBundle, make_handler
from pick_place.completion import Completion


class PickPlaceHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("openssl") is None:
            raise unittest.SkipTest("openssl is required for the local HTTPS fixture")
        cls.certificates = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.certificates.cleanup)
        cls.cert = str(Path(cls.certificates.name) / "cert.pem")
        cls.key = str(Path(cls.certificates.name) / "key.pem")
        subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                        "-keyout", cls.key, "-out", cls.cert, "-days", "1",
                        "-subj", "/CN=localhost", "-addext", "subjectAltName=DNS:localhost,IP:127.0.0.1"],
                       check=True, capture_output=True, timeout=15)

    def setUp(self):
        self.fixture = transfer.TransferTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.plugin = self.fixture.plugin
        self.fixture.completion_factory.side_effect = Completion
        self.received = []
        self.acknowledge = lambda payload: {"ok": True, "action_id": payload["action_id"]}
        test = self

        class CoreHandler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                test.received.append((self.path, payload))
                body = json.dumps(test.acknowledge(payload)).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        core = ThreadingHTTPServer(("127.0.0.1", 0), CoreHandler)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)
        core.socket = context.wrap_socket(core.socket, server_side=True)
        self.serve(core)
        self.enterContext(mock.patch.dict(os.environ, {
            "AGENT_CORE_CA_CERT": self.cert, "AGENT_CORE_URL": f"https://localhost:{core.server_port}"}))
        bundle = DriverBundle([self.plugin])
        mcp = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "test", "test"))
        self.serve(mcp)
        self.mcp_url = f"http://127.0.0.1:{mcp.server_port}/mcp"

    def serve(self, server):
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        def cleanup():
            server.shutdown()
            worker.join(timeout=2)
            server.server_close()
        self.addCleanup(cleanup)

    def rpc(self, method, **params):
        request = urllib.request.Request(self.mcp_url, headers={"Content-Type": "application/json"},
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode())
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)

    def call(self, action, **arguments):
        rpc = self.rpc("tools/call", name="vision_pick_and_drop", arguments={"action": action, **arguments})
        return json.loads(rpc["result"]["content"][0]["text"])

    def completed(self, accepted):
        self.assertEqual(accepted["state"], "running", accepted)
        return observation.wait_for_completion(self.plugin, accepted)

    def test_disabled_follow_up_reports_skipped_observation_in_one_acp_completion(self):
        self.assertTrue(self.call("config", observe_after_transfer=False)["ok"])
        self.fixture.make_photo()
        result = self.completed(self.call("transfer_by", confirm_motion=True,
                                         start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0))
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(len(self.received), 1)
        payload = self.received[0][1]
        self.assertEqual(payload["action_id"], result["action_id"])
        self.assertEqual(payload["result"]["observation"], {"skipped": True, "reason": "disabled"})
        self.assertTrue(payload["result"]["observation_required"])
        self.assertTrue(payload["result"]["release_completed"])
        self.assertFalse(payload["result"]["holding_object_possible"])
        self.fixture.camera.snapshot.assert_not_called()

    def test_tools_list_exposes_confirmation_completion_and_interrupt_contract(self):
        schema = self.rpc("tools/list")["result"]["tools"][0]["inputSchema"]
        self.assertEqual(schema["x-completion"], {
            "actions": ["observe", "transfer_to", "transfer_by"], "timeout": 210})
        for action in schema["x-completion"]["actions"]:
            self.assertIn("confirm_motion", schema["x-action-params"][action]["params"])
            self.assertIn("confirm_motion=true", schema["x-action-params"][action]["description"])
        self.assertEqual(schema["properties"]["confirm_motion"]["type"], "boolean")
        self.assertIs(schema["properties"]["confirm_motion"]["const"], True)
        self.assertEqual(schema["allOf"][0]["then"]["required"], ["confirm_motion"])
        self.assertEqual(schema["x-action-params"]["cancel"]["params"], [])
        self.assertEqual(schema["x-hooks"]["on_interrupt_all"], {"action": "cancel"})
        self.assertIs(self.plugin.dispatch("unknown", {}), None)
        unknown = self.rpc("tools/call", name="vision_pick_and_drop", arguments={"action": "unknown"})
        self.assertEqual(unknown["error"]["code"], -32601)

    def test_every_motion_requires_literal_true_before_any_side_effect(self):
        photo = self.plugin._observation
        for action in ("observe", "transfer_to", "transfer_by"):
            for confirmation in ({}, {"confirm_motion": False}, {"confirm_motion": None},
                                 {"confirm_motion": 1}, {"confirm_motion": "true"}):
                with self.subTest(action=action, confirmation=confirmation):
                    result = self.call(action, **confirmation)
                    self.assertEqual(result["code"], "CONFIRMATION_REQUIRED")
                    self.assertNotIn("action_id", result)
        self.assertIs(self.plugin._observation, photo)
        self.assertEqual(self.fixture.commands, [])
        self.fixture.camera.snapshot.assert_not_called()
        self.fixture.completion_factory.assert_not_called()
        self.assertEqual(self.received, [])
        self.assertFalse(self.fixture.client.motion_lock.locked())

    def test_http_returns_while_worker_is_blocked_and_acp_delivers_photo(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def snapshot(after, cancel, check):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test did not release camera")
            check()
            return {**self.fixture.photo_data, "captured_at": after + .01}
        self.fixture.camera.snapshot.side_effect = snapshot
        accepted = self.call("observe", confirm_motion=True)
        self.assertTrue(entered.wait(2))
        self.assertEqual(accepted["state"], "running")
        self.assertNotIn("result", accepted)
        self.assertEqual(self.received, [])
        self.assertTrue(self.fixture.client.motion_lock.locked())
        self.assertEqual(self.call("observe", confirm_motion=True)["state"], "error")
        release.set()
        terminal = self.completed(accepted)
        self.assertEqual(terminal["state"], "completed", terminal)
        path, completion = self.received[0]
        self.assertEqual(path, "/api/acp/complete")
        self.assertEqual(completion["action_id"], accepted["action_id"])
        self.assertEqual(completion["tool"], "vision_pick_and_drop")
        self.assertEqual(completion["status"], "completed")
        self.assertTrue(completion["result"]["ok"])
        self.assertFalse(completion["result"]["observation_required"])
        self.assertTrue(Path(completion["result"]["file_path"]).exists())
        self.fixture.camera.stop.assert_not_called()
        self.assertIn("objects", completion["result"])
        self.assertFalse(self.fixture.client.motion_lock.locked())
        # A prior confirmation never authorizes a later call.
        self.assertEqual(self.call("observe")["code"], "CONFIRMATION_REQUIRED")
        self.assertEqual(len(self.received), 1)

    def test_both_transfer_actions_deliver_correlated_completion_and_consume_photo(self):
        ids = set()
        for action, args in (("transfer_to", {"target_point_x": .5, "target_point_y": 1/3}),
                             ("transfer_by", {"delta_x": -30, "delta_y": 0})):
            self.fixture.make_photo()
            accepted = self.call(action, confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, **args)
            terminal = self.completed(accepted)
            self.assertEqual(terminal["state"], "completed", terminal)
            payload = self.received[-1][1]
            self.assertEqual(payload["action_id"], accepted["action_id"])
            self.assertEqual(payload["status"], "completed")
            self.assertEqual(payload["result"]["pick_pixel"], [1, 1])
            self.assertFalse(payload["result"]["observation_required"])
            self.assertFalse(payload["result"]["grasp_checked"])
            self.assertTrue(payload["result"]["transfer_completed"])
            fresh = payload["result"]["observation"]
            self.assertTrue(fresh["ok"])
            self.assertEqual(fresh, self.plugin._observation)
            self.assertNotEqual(fresh["observation_id"], payload["result"]["observation_id"])
            self.assertTrue(Path(fresh["file_path"]).exists())
            self.assertEqual(fresh["objects"][0]["name"], "banana")
            ids.add(accepted["action_id"])
        self.assertEqual(len(ids), 2)
        self.assertEqual(len(self.received), 2)
        self.assertEqual(len(self.fixture.moves()), 12)
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.fixture.commands), 2)
        self.assertIsNotNone(self.fixture.plugin._observation)

    def test_transfer_waits_for_follow_up_observation_before_single_completion(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def snapshot(after, cancel, check):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test did not release follow-up observation")
            return self.fixture.snapshot(after, cancel, check)
        self.fixture.camera.snapshot.side_effect = snapshot
        accepted = self.call("transfer_by", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0)
        self.assertTrue(entered.wait(2))
        self.assertEqual(len(self.fixture.moves()), 6)
        self.assertEqual(self.received, [])
        self.assertTrue(self.fixture.client.motion_lock.locked())
        self.assertEqual(self.call("observe", confirm_motion=True)["state"], "error")
        release.set()
        terminal = self.completed(accepted)
        self.assertEqual(terminal["state"], "completed", terminal)
        self.assertEqual(len(self.received), 1)
        payload = self.received[0][1]
        self.assertEqual(payload["action_id"], accepted["action_id"])
        self.assertTrue(payload["result"]["observation"]["ok"])
        self.assertEqual(payload["result"]["observation"]["objects"][0]["name"], "banana")
        self.assertFalse(payload["result"]["observation_required"])

    def test_follow_up_failure_reports_completed_transfer_without_retrying(self):
        self.fixture.camera.snapshot.side_effect = RuntimeError("VOP input lost")
        accepted = self.call("transfer_to", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, target_point_x=.5, target_point_y=1/3)
        terminal = self.completed(accepted)
        self.assertEqual(terminal["state"], "error")
        self.assertEqual(len(self.received), 1)
        payload = self.received[0][1]
        self.assertEqual(payload["action_id"], accepted["action_id"])
        self.assertFalse(payload["result"]["ok"])
        self.assertTrue(payload["result"]["transfer_completed"])
        self.assertFalse(payload["result"]["observation"]["ok"])
        self.assertTrue(payload["result"]["observation_required"])
        self.assertEqual(len(self.fixture.moves()), 6)
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.fixture.commands), 1)

    def test_execution_error_is_delivered_with_matching_id_and_stop_result(self):
        self.fixture.camera.snapshot.side_effect = RuntimeError("camera disconnected")
        accepted = self.call("observe", confirm_motion=True)
        terminal = self.completed(accepted)
        self.assertEqual(terminal["state"], "error")
        payload = self.received[0][1]
        self.assertEqual(payload["action_id"], accepted["action_id"])
        self.assertEqual(payload["status"], "error")
        self.assertIn("camera disconnected", payload["result"]["message"])
        self.assertFalse(payload["result"]["ok"])
        self.assertTrue(payload["result"]["observation_required"])
        self.assertEqual([name for name, _ in self.fixture.commands], ["rm_movej", "rm_set_arm_slow_stop"])

    def test_cancel_without_confirmation_delivers_cancelled_once(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def snapshot(after, cancel, check):
            entered.set()
            release.wait(5)
            check()
        self.fixture.camera.snapshot.side_effect = snapshot
        accepted = self.call("observe", confirm_motion=True)
        self.assertTrue(entered.wait(2))
        self.assertEqual(self.call("cancel")["state"], "stopping")
        release.set()
        terminal = self.completed(accepted)
        self.assertEqual(terminal["state"], "cancelled")
        self.assertEqual(len(self.received), 1)
        self.assertEqual(self.received[0][1]["status"], "cancelled")
        self.assertEqual(self.received[0][1]["action_id"], accepted["action_id"])
        self.assertIsNone(self.fixture.plugin._observation)
        self.assertFalse(self.fixture.client.motion_lock.locked())

    def test_unregistered_completion_retries_identical_notification_only(self):
        self.acknowledge = lambda payload: {"ok": len(self.received) > 1, "action_id": payload["action_id"]}
        terminal = self.completed(self.call("transfer_by", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0))
        self.assertEqual(terminal["callback"], "accepted")
        self.assertEqual(len(self.received), 2)
        self.assertEqual(self.received[0], self.received[1])
        self.assertEqual(len(self.fixture.moves()), 6)

    def test_unacknowledged_callback_is_visible_without_changing_motion_outcome(self):
        self.acknowledge = lambda payload: {"ok": True, "action_id": "another-action"}
        terminal = self.completed(self.call("transfer_by", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0))
        self.assertEqual(terminal["state"], "completed")
        self.assertTrue(terminal["result"]["ok"])
        self.assertEqual(terminal["callback"], "failed")
        self.assertIn("acknowledge", terminal["callback_error"])
        self.assertEqual(len(self.received), Completion.ATTEMPTS)
        self.assertEqual(len(self.fixture.moves()), 6)
        self.assertFalse(self.fixture.client.motion_lock.locked())

    def test_missing_or_invalid_ca_rejects_before_motion(self):
        for ca in ("", "/nonexistent/pick-place-ca.pem"):
            with self.subTest(ca=ca), mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": ca}):
                result = self.call("observe", confirm_motion=True)
                self.assertEqual(result["state"], "error")
                self.assertNotIn("action_id", result)
        self.assertEqual(self.fixture.commands, [])
        self.fixture.camera.snapshot.assert_not_called()
        self.assertEqual(self.received, [])
        self.assertFalse(self.fixture.client.motion_lock.locked())

    def test_thread_start_failure_releases_reservation_without_motion(self):
        with mock.patch("pick_place.threading.Thread.start", side_effect=RuntimeError("cannot start worker")):
            result = self.plugin.dispatch("observe", {"confirm_motion": True})
        self.assertEqual(result["state"], "error")
        self.assertNotIn("action_id", result)
        self.assertFalse(self.fixture.client.motion_lock.locked())
        self.assertIsNone(self.plugin._active)
        self.assertEqual(self.fixture.commands, [])

    def test_late_acknowledgement_does_not_replace_newer_completion(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = Completion.send
        def send(completion, action_id, status, result):
            if not entered.is_set():
                entered.set()
                release.wait(5)
                response = original(completion, action_id, status, result)
                finished.set()
                return response
            return original(completion, action_id, status, result)
        with mock.patch.object(Completion, "send", send):
            first = self.call("transfer_by", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0)
            self.assertTrue(entered.wait(2))
            self.fixture.make_photo()
            second = self.call("transfer_by", confirm_motion=True, start_point_x=-.5, start_point_y=-1/3, delta_x=30, delta_y=0)
            terminal = self.completed(second)
            release.set()
            self.assertTrue(finished.wait(2))
            self.assertNotEqual(first["action_id"], second["action_id"])
            self.assertEqual(self.plugin.dispatch("info", {})["last_result"], terminal)
