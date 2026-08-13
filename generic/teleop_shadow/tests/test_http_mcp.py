from __future__ import annotations

import asyncio
import copy
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock
from unittest.mock import AsyncMock

from aiohttp.test_utils import TestClient, TestServer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from capture import CaptureError

from main import (
    CaptureTlsError,
    SERVICE_KEY,
    ShadowDriverService,
    create_app,
    load_config,
    registration_headers,
    registration_payload,
    tool_definitions,
)

from tests.helpers import TEST_SECRET, contains_value

TEST_CONFIG = {
    "mcp_port": 15711,
    "bind_host": "127.0.0.1",
    "teleop": {
        "lease_timeout_ms": 5000,
        "pose_timeout_ms": 500,
        "watchdog_interval_ms": 25,
        "ticket_ttl_max_seconds": 30,
        "ticket_replay_cache_entries": 128,
    },
    "capture": {
        "pairing_ttl_seconds": 60,
        "presence_interval_ms": 1000,
        "presence_timeout_ms": 5000,
        "public_wss_url": "wss://robot.test:15712/ws/teleop-capture",
        "ca_certificate_base64": "dGVzdC1jYQ==",
    },
    "registration": {"enabled": False},
}


class McpHttpTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.env = mock.patch.dict(os.environ, {
            # A legacy Core token must not change the stock-Core MCP contract.
            "MOTUS_DRIVER_TOKEN": "legacy-token-that-stock-core-will-not-send",
        })
        self.env.start()
        self.client = TestClient(TestServer(create_app(
            copy.deepcopy(TEST_CONFIG),
            allow_insecure_test_transport=True,
        )))
        await self.client.start_server()
        self.request_id = 0

    async def asyncTearDown(self):
        await self.client.close()
        self.env.stop()

    async def rpc(self, method: str, params: dict | None = None, *, headers: dict | None = None):
        self.request_id += 1
        response = await self.client.post("/mcp", json={
            "jsonrpc": "2.0",
            "id": self.request_id,
            "method": method,
            "params": params or {},
        }, headers=headers or {})
        return response.status, await response.json()

    async def call(self, name: str, arguments: dict) -> dict:
        status, payload = await self.rpc(
            "tools/call",
            {"name": name, "arguments": arguments},
        )
        self.assertEqual(200, status)
        self.assertNotIn("error", payload, payload)
        return json.loads(payload["result"]["content"][0]["text"])

    async def test_stock_core_can_initialize_list_and_call_without_authorization(self):
        status, initialized = await self.rpc("initialize")
        self.assertEqual(200, status)
        self.assertEqual(
            "teleop-shadow-driver",
            initialized["result"]["serverInfo"]["name"],
        )
        status, listed = await self.rpc("tools/list")
        self.assertEqual(200, status)
        self.assertEqual(
            {"teleop_session", "teleop_state"},
            {tool["name"] for tool in listed["result"]["tools"]},
        )
        self.assertNotIn("Authorization", registration_headers(
            self.client.server.app[SERVICE_KEY]
        ))

    async def test_non_loopback_mcp_request_is_rejected_even_with_legacy_token(self):
        with mock.patch("main._is_loopback_bind", return_value=False):
            status, payload = await self.rpc(
                "tools/list",
                headers={"Authorization": "Bearer legacy-token-that-stock-core-will-not-send"},
            )
        self.assertEqual(403, status)
        self.assertEqual("mcp_loopback_only", payload["error"])

    async def test_session_card_has_only_driver_owned_visible_actions(self):
        _, payload = await self.rpc("tools/list")
        tools = {tool["name"]: tool for tool in payload["result"]["tools"]}
        actions = tools["teleop_session"]["inputSchema"]["x-action-params"]
        self.assertEqual({
            "start", "stop", "info", "pair_headset", "revoke_headset", "pause", "release",
            "emergency_stop", "status",
        }, set(actions))
        self.assertTrue({"prepare_shadow", "heartbeat", "soft_stop"}.isdisjoint(actions))
        signaling = tools["teleop_session"]["x-teleop"]["signaling"]
        self.assertEqual("/ws/teleop-capture", signaling["path"])
        self.assertEqual("paired-capture-credential-only", signaling["access"])

    async def test_start_info_stop_tolerate_core_instance_id(self):
        started = await self.call("teleop_session", {
            "action": "start",
            "instance_id": "canvas-card-1",
        })
        self.assertEqual("prepared_shadow", started["state"])
        self.assertTrue(started["authority_valid"])
        self.assertFalse(started["lease"]["armed"])
        self.assertEqual("canvas-card-1", started["instance_id"])
        self.assertIsNotNone(started["session_id"])

        info = await self.call("teleop_session", {
            "action": "info",
            "instance_id": "canvas-card-1",
        })
        self.assertEqual(started["session_id"], info["session_id"])
        self.assertEqual([], info["topic_out"])

        stopped = await self.call("teleop_session", {
            "action": "stop",
            "instance_id": "canvas-card-1",
        })
        self.assertEqual("released", stopped["state"])
        self.assertFalse(stopped["authority_valid"])

    async def test_pairing_is_created_only_for_started_session(self):
        _, failed = await self.rpc("tools/call", {
            "name": "teleop_session",
            "arguments": {"action": "pair_headset", "instance_id": "card"},
        })
        self.assertEqual("session_inactive", failed["error"]["data"]["code"])
        await self.call("teleop_session", {"action": "start", "instance_id": "card"})
        pairing = await self.call("teleop_session", {
            "action": "pair_headset",
            "instance_id": "card",
        })
        self.assertEqual("pairing_ready", pairing["state"])
        self.assertEqual("/ws/teleop-capture", pairing["websocket_path"])
        self.assertEqual(36, len(pairing["pairing_id"]))
        self.assertGreaterEqual(len(pairing["pairing_code"]), 32)

    async def test_pause_release_and_emergency_stop_need_no_private_authority(self):
        await self.call("teleop_session", {"action": "start", "instance_id": "card"})
        paused = await self.call("teleop_session", {"action": "pause", "instance_id": "card"})
        self.assertEqual("paused", paused["state"])
        released = await self.call("teleop_session", {"action": "release", "instance_id": "card"})
        self.assertEqual("released", released["state"])

        await self.call("teleop_session", {"action": "start", "instance_id": "card"})
        stopped = await self.call("teleop_session", {
            "action": "emergency_stop",
            "instance_id": "card",
        })
        self.assertEqual("released", stopped["state"])
        self.assertEqual("emergency_stop", stopped["reason"])
        self.assertEqual(1, stopped["counters"]["emergency_stops"])

    async def test_core_issued_authority_and_pose_submission_are_not_public_actions(self):
        for action, extra in (
            ("prepare_shadow", {"session_id": "private", "epoch": 1, "fence": "x" * 32}),
            ("heartbeat", {}),
            ("submit_shadow_frame", {"frame": {}}),
        ):
            with self.subTest(action=action):
                _, payload = await self.rpc("tools/call", {
                    "name": "teleop_session",
                    "arguments": {"action": action, **extra},
                })
                self.assertIn("error", payload)

    async def test_direct_offer_is_fail_closed_and_health_discloses_no_secret(self):
        offer = await self.client.post("/offer", json={"type": "offer", "sdp": "x"})
        self.assertEqual(401, offer.status)
        self.assertEqual(
            "capture_control_required",
            (await offer.json())["error"]["code"],
        )
        health = await (await self.client.get("/health")).json()
        self.assertTrue(health["ready"])
        self.assertTrue(health["rtc_enabled"])
        self.assertEqual("loopback-only", health["mcp_access"])
        self.assertFalse(contains_value(health, TEST_SECRET))
        self.assertNotIn("legacy-token-that-stock-core-will-not-send", json.dumps(health))


class DriverOwnedCredentialTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def _pair_message(pairing: dict, *, app_version: str = "1.2.0") -> dict:
        return {
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": app_version,
        }

    @staticmethod
    def _credential_message(identity: dict, *, app_version: str) -> dict:
        return {
            "type": "credential",
            "capture_id": identity["capture_id"],
            "capture_credential": identity["capture_credential"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": app_version,
        }

    @staticmethod
    def _post_commit_fsync_failure():
        real_fsync = os.fsync

        def fail_directory_fsync(descriptor: int) -> None:
            if stat.S_ISDIR(os.fstat(descriptor).st_mode):
                raise OSError("directory fsync unavailable")
            real_fsync(descriptor)

        return mock.patch("capture.os.fsync", side_effect=fail_directory_fsync)

    async def test_signaling_offer_is_single_use_and_never_renews_capture_lease(self):
        service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
        try:
            await asyncio.to_thread(service.runtime.prepare_local_session)
            pairing = await service.capture.create_pairing()
            connection, _ack, _assignment = await service.capture.connect({
                "type": "pair",
                "pairing_id": pairing["pairing_id"],
                "pairing_code": pairing["pairing_code"],
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
            })
            await service.capture.presence(connection, {
                "type": "presence",
                "state": "xr_standby",
                "assignment_id": None,
            })
            assignment = (await connection.events.get())["assignment"]
            before = service.runtime.status()["counters"]["lease_heartbeats"]
            service.capture._rtc.accept_offer = AsyncMock(return_value={
                "type": "answer",
                "sdp": "answer-sdp",
            })
            offer = {
                "type": "signaling_offer",
                "assignment_id": assignment["id"],
                "offer": {"type": "offer", "sdp": "offer-sdp"},
            }
            answer = await service.capture.signaling_offer(connection, offer)
            self.assertEqual("signaling_answer", answer["type"])
            self.assertEqual(
                before,
                service.runtime.status()["counters"]["lease_heartbeats"],
            )
            with self.assertRaisesRegex(CaptureError, "capture_offer_already_consumed"):
                await service.capture.signaling_offer(connection, offer)
        finally:
            await service.close()

    async def test_combined_plain_http_capture_app_is_test_only(self):
        with self.assertRaisesRegex(CaptureTlsError, "test-only"):
            create_app(copy.deepcopy(TEST_CONFIG))

    async def test_missing_ticket_secret_uses_ephemeral_internal_secret(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
        try:
            self.assertTrue(service.rtc.enabled)
            self.assertEqual({"Content-Type": "application/json"}, registration_headers(service))
        finally:
            await service.close()

    async def test_registration_and_network_bind_do_not_require_core_bearer(self):
        config = copy.deepcopy(TEST_CONFIG)
        config["bind_host"] = "0.0.0.0"
        config["registration"]["enabled"] = True
        with mock.patch.dict(os.environ, {}, clear=True):
            service = ShadowDriverService(config)
        try:
            self.assertTrue(service.rtc.enabled)
            self.assertNotIn("Authorization", registration_headers(service))
        finally:
            await service.close()

    async def test_paired_capture_credential_survives_driver_restart_and_can_be_revoked(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(TEST_CONFIG)
            state_file = Path(directory) / "capture.json"
            config["capture"]["state_file"] = str(state_file)
            with mock.patch.dict(os.environ, {}, clear=True):
                service = ShadowDriverService(config)
            started = await service.dispatch_tool(
                "teleop_session",
                {"action": "start", "instance_id": "card"},
            )
            self.assertTrue(started["authority_valid"])
            pairing = await service.dispatch_tool(
                "teleop_session",
                {"action": "pair_headset", "instance_id": "card"},
            )
            connection, paired, _ = await service.capture.connect({
                "type": "pair",
                "pairing_id": pairing["pairing_id"],
                "pairing_code": pairing["pairing_code"],
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
            })
            await service.capture.presence(connection, {
                "type": "presence",
                "state": "xr_standby",
                "assignment_id": None,
            })
            first_assignment = await connection.events.get()
            self.assertEqual("assignment", first_assignment["type"])
            await service.capture.disconnect(connection)
            await service.close()
            self.assertEqual(0o600, stat.S_IMODE(state_file.stat().st_mode))

            with mock.patch.dict(os.environ, {}, clear=True):
                restarted = ShadowDriverService(config)
            try:
                await restarted.dispatch_tool(
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                reconnected, acknowledgement, _ = await restarted.capture.connect({
                    "type": "credential",
                    "capture_id": paired["capture_id"],
                    "capture_credential": paired["capture_credential"],
                    "capture_protocol": "motus.teleop.capture.v1",
                    "frame_protocol": "motus.teleop.rtc-frame.v1",
                    "client_kind": "native_openxr",
                    "app_version": "1.2.1",
                })
                self.assertEqual("connected", acknowledgement["type"])
                await restarted.capture.presence(reconnected, {
                    "type": "presence",
                    "state": "xr_standby",
                    "assignment_id": None,
                })
                second_assignment = await reconnected.events.get()
                self.assertEqual("assignment", second_assignment["type"])
                self.assertEqual(
                    1,
                    first_assignment["assignment"]["generation"],
                )
                self.assertEqual(
                    1,
                    second_assignment["assignment"]["generation"],
                )
                self.assertNotEqual(
                    first_assignment["assignment"]["session_id"],
                    second_assignment["assignment"]["session_id"],
                )
                await restarted.capture.disconnect(reconnected)
                revoked = await restarted.dispatch_tool(
                    "teleop_session",
                    {"action": "revoke_headset", "instance_id": "card"},
                )
                self.assertEqual("capture_revoked", revoked["reason"])
                persisted = json.loads(state_file.read_text(encoding="utf-8"))
                self.assertIsNone(persisted["capture"])
            finally:
                await restarted.close()

    async def test_capture_state_failure_rolls_back_identity_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(TEST_CONFIG)
            config["capture"]["state_file"] = str(Path(directory) / "capture.json")
            service = ShadowDriverService(config)
            try:
                await service.dispatch_tool(
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                first_pairing = await service.dispatch_tool(
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                first_connection, first_identity, _ = await service.capture.connect({
                    "type": "pair",
                    "pairing_id": first_pairing["pairing_id"],
                    "pairing_code": first_pairing["pairing_code"],
                    "capture_protocol": "motus.teleop.capture.v1",
                    "frame_protocol": "motus.teleop.rtc-frame.v1",
                    "client_kind": "native_openxr",
                    "app_version": "1.2.0",
                })
                await service.capture.disconnect(first_connection)

                replacement = await service.dispatch_tool(
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                with (
                    mock.patch.object(
                        service.capture,
                        "_persist_state",
                        side_effect=OSError("disk unavailable"),
                    ),
                    self.assertRaisesRegex(CaptureError, "capture_state_unavailable"),
                ):
                    await service.capture.connect({
                        "type": "pair",
                        "pairing_id": replacement["pairing_id"],
                        "pairing_code": replacement["pairing_code"],
                        "capture_protocol": "motus.teleop.capture.v1",
                        "frame_protocol": "motus.teleop.rtc-frame.v1",
                        "client_kind": "native_openxr",
                        "app_version": "1.2.0",
                    })

                restored, acknowledgement, _ = await service.capture.connect({
                    "type": "credential",
                    "capture_id": first_identity["capture_id"],
                    "capture_credential": first_identity["capture_credential"],
                    "capture_protocol": "motus.teleop.capture.v1",
                    "frame_protocol": "motus.teleop.rtc-frame.v1",
                    "client_kind": "native_openxr",
                    "app_version": "1.2.0",
                })
                self.assertEqual("connected", acknowledgement["type"])
                await service.capture.disconnect(restored)

                with (
                    mock.patch.object(
                        service.capture,
                        "_persist_state",
                        side_effect=OSError("disk unavailable"),
                    ),
                    self.assertRaisesRegex(CaptureError, "capture_state_unavailable"),
                ):
                    await service.capture.revoke_headset()
                self.assertEqual(1, (await service.capture.status())["paired_devices"])
            finally:
                await service.close()

    async def test_post_commit_pair_failure_keeps_memory_equal_to_replaced_state(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(TEST_CONFIG)
            state_file = Path(directory) / "capture.json"
            config["capture"]["state_file"] = str(state_file)
            service = ShadowDriverService(config)
            try:
                pairing = await service.capture.create_pairing()
                with (
                    self._post_commit_fsync_failure(),
                    self.assertRaisesRegex(CaptureError, "capture_state_unavailable"),
                ):
                    await service.capture.connect(self._pair_message(pairing))
                persisted = json.loads(state_file.read_text(encoding="utf-8"))
                self.assertIsNotNone(persisted["capture"])
                self.assertEqual(1, (await service.capture.status())["paired_devices"])
            finally:
                await service.close()

    async def test_post_commit_version_failure_keeps_new_version_in_memory_and_state(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(TEST_CONFIG)
            state_file = Path(directory) / "capture.json"
            config["capture"]["state_file"] = str(state_file)
            service = ShadowDriverService(config)
            try:
                pairing = await service.capture.create_pairing()
                connection, identity, _ = await service.capture.connect(
                    self._pair_message(pairing)
                )
                await service.capture.disconnect(connection)
                updated = self._credential_message(identity, app_version="1.2.1")
                with (
                    self._post_commit_fsync_failure(),
                    self.assertRaisesRegex(CaptureError, "capture_state_unavailable"),
                ):
                    await service.capture.connect(updated)
                self.assertEqual(
                    "1.2.1",
                    json.loads(state_file.read_text(encoding="utf-8"))["capture"]["app_version"],
                )
                with mock.patch.object(service.capture, "_persist_state") as persist:
                    reconnected, acknowledgement, _ = await service.capture.connect(updated)
                persist.assert_not_called()
                self.assertEqual("connected", acknowledgement["type"])
                await service.capture.disconnect(reconnected)
            finally:
                await service.close()

    async def test_post_commit_revoke_failure_keeps_memory_equal_to_tombstone(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(TEST_CONFIG)
            state_file = Path(directory) / "capture.json"
            config["capture"]["state_file"] = str(state_file)
            service = ShadowDriverService(config)
            try:
                pairing = await service.capture.create_pairing()
                connection, _identity, _ = await service.capture.connect(
                    self._pair_message(pairing)
                )
                await service.capture.disconnect(connection)
                with (
                    self._post_commit_fsync_failure(),
                    self.assertRaisesRegex(CaptureError, "capture_state_unavailable"),
                ):
                    await service.capture.revoke_headset()
                self.assertIsNone(
                    json.loads(state_file.read_text(encoding="utf-8"))["capture"]
                )
                self.assertEqual(0, (await service.capture.status())["paired_devices"])
                self.assertFalse((await service.capture.revoke_headset())["revoked"])
            finally:
                await service.close()

    async def test_malformed_auth_ids_use_native_terminal_error_codes(self):
        service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
        try:
            pairing = await service.capture.create_pairing()
            malformed_pair = self._pair_message(pairing)
            malformed_pair["pairing_id"] = "not-a-uuid"
            with self.assertRaises(CaptureError) as pairing_error:
                await service.capture.connect(malformed_pair)
            self.assertEqual("capture_pairing_invalid", pairing_error.exception.code)
            self.assertEqual(401, pairing_error.exception.status)

            malformed_credential = {
                "type": "credential",
                "capture_id": "not-a-uuid",
                "capture_credential": "x" * 32,
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
            }
            with self.assertRaises(CaptureError) as credential_error:
                await service.capture.connect(malformed_credential)
            self.assertEqual(
                "capture_credential_invalid",
                credential_error.exception.code,
            )
            self.assertEqual(401, credential_error.exception.status)
        finally:
            await service.close()

    async def test_browser_capture_client_is_not_an_implicit_fallback(self):
        service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
        try:
            await service.dispatch_tool(
                "teleop_session",
                {"action": "start", "instance_id": "card"},
            )
            pairing = await service.dispatch_tool(
                "teleop_session",
                {"action": "pair_headset", "instance_id": "card"},
            )
            with self.assertRaisesRegex(CaptureError, "capture_client_unsupported"):
                await service.capture.connect({
                    "type": "pair",
                    "pairing_id": pairing["pairing_id"],
                    "pairing_code": pairing["pairing_code"],
                    "capture_protocol": "motus.teleop.capture.v1",
                    "frame_protocol": "motus.teleop.rtc-frame.v1",
                    "client_kind": "browser_webxr",
                    "app_version": "1.2.0",
                })
        finally:
            await service.close()


class IdentityAndRegistrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_environment_overrides_and_tls_development_flag(self):
        with mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_ID": "teleop-shadow-env-instance",
            "MOTUS_DRIVER_NAME": "Environment Instance",
            "MOTUS_ROBOT_ID": "robot-env-1",
            "MOTUS_AGENT_CORE_VERIFY_TLS": "0",
        }):
            config = load_config(ROOT / "config.yaml")
        self.assertEqual("teleop-shadow-env-instance", config["driver_id"])
        self.assertEqual("Environment Instance", config["driver_name"])
        self.assertEqual("robot-env-1", config["robot_id"])
        self.assertFalse(config["registration"]["verify_tls"])

    async def test_driver_and_robot_ids_keep_stock_core_boundaries(self):
        for field in ("driver_id", "robot_id"):
            accepted = copy.deepcopy(TEST_CONFIG)
            accepted[field] = "x" * 64
            rejected = copy.deepcopy(TEST_CONFIG)
            rejected[field] = "x" * 65
            with mock.patch.dict(os.environ, {}, clear=True):
                service = ShadowDriverService(accepted)
                await service.close()
                with self.assertRaisesRegex(ValueError, "1-64"):
                    ShadowDriverService(rejected)

    async def test_two_services_have_distinct_authority_domains(self):
        config_a = copy.deepcopy(TEST_CONFIG)
        config_a.update({"driver_id": "shadow-a", "robot_id": "robot-a"})
        config_b = copy.deepcopy(TEST_CONFIG)
        config_b.update({"driver_id": "shadow-b", "robot_id": "robot-b"})
        with mock.patch.dict(os.environ, {}, clear=True):
            service_a = ShadowDriverService(config_a)
            service_b = ShadowDriverService(config_b)
        try:
            self.assertNotEqual(service_a.runtime.boot_id, service_b.runtime.boot_id)
            self.assertEqual({
                "name": service_a.driver_name,
                "url": "http://localhost:15711/mcp",
                "transport": "http",
                "category": "driver",
                "render_hint": "teleop",
            }, registration_payload(service_a))
            self.assertEqual(service_b.driver_name, registration_payload(service_b)["name"])
            self.assertEqual(
                "shadow-a",
                tool_definitions(driver_id="shadow-a")[0]["x-teleop"]["driver_id"],
            )
        finally:
            await service_a.close()
            await service_b.close()


if __name__ == "__main__":
    unittest.main()
