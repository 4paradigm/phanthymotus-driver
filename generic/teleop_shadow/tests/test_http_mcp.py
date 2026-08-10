from __future__ import annotations

import asyncio
import copy
import json
import os
import sys
import unittest
import uuid
from pathlib import Path
from unittest import mock

from aiohttp.test_utils import TestClient, TestServer
from aiortc import RTCPeerConnection, RTCSessionDescription

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from main import (
    SERVICE_KEY,
    ShadowDriverService,
    create_app,
    load_config,
    registration_headers,
    registration_payload,
    tool_definitions,
)
from protocol import TicketCodec, make_ticket_claims

from tests.helpers import TEST_DRIVER_TOKEN, TEST_SECRET, contains_value, valid_frame

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
    "registration": {"enabled": False},
}


class McpHttpTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.env = mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN,
            "MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET,
        })
        self.env.start()
        self.client = TestClient(TestServer(create_app(TEST_CONFIG)))
        await self.client.start_server()
        self.request_id = 0

    async def asyncTearDown(self):
        await self.client.close()
        self.env.stop()

    async def rpc(self, method: str, params: dict | None = None, *, authorized: bool = True):
        self.request_id += 1
        headers = {"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"} if authorized else {}
        response = await self.client.post("/mcp", json={
            "jsonrpc": "2.0",
            "id": self.request_id,
            "method": method,
            "params": params or {},
        }, headers=headers)
        return response.status, await response.json()

    async def call(self, name: str, arguments: dict) -> dict:
        status, payload = await self.rpc("tools/call", {"name": name, "arguments": arguments})
        self.assertEqual(200, status)
        self.assertNotIn("error", payload)
        return json.loads(payload["result"]["content"][0]["text"])

    async def test_mcp_requires_driver_token_when_configured(self):
        status, _ = await self.rpc("tools/list", authorized=False)
        self.assertEqual(401, status)
        status, payload = await self.rpc("initialize")
        self.assertEqual("teleop-shadow-driver", payload["result"]["serverInfo"]["name"])
        service = self.client.server.app[SERVICE_KEY]
        self.assertEqual(
            f"Bearer {TEST_DRIVER_TOKEN}", registration_headers(service)["Authorization"]
        )

    async def test_tools_are_real_and_start_is_passive(self):
        _, payload = await self.rpc("tools/list")
        tools = {tool["name"]: tool for tool in payload["result"]["tools"]}
        self.assertEqual({"teleop_session", "teleop_state"}, set(tools))
        self.assertTrue(tools["teleop_state"]["readOnly"])
        for tool in tools.values():
            self.assertEqual("motus.teleop.shadow.v1", tool["x-teleop"]["protocol"])
            self.assertEqual(
                "motus.teleop.dispatch.recording.v1",
                tool["x-teleop"]["dispatch_contract"],
            )
            self.assertEqual("shadow", tool["x-teleop"]["mode"])
            self.assertFalse(tool["x-teleop"]["actuation_enabled"])
            self.assertEqual(
                {
                    "protocol": "motus.teleop.webrtc-offer-answer.v1",
                    "path": "/offer",
                    "access": "authenticated-core-proxy-only",
                },
                tool["x-teleop"]["signaling"],
            )
        session_actions = tools["teleop_session"]["inputSchema"]["x-action-params"]
        self.assertEqual(
            {
                "start", "stop", "prepare_shadow", "heartbeat", "pause", "release",
                "soft_stop", "status", "submit_shadow_frame",
            },
            set(session_actions),
        )
        started = await self.call("teleop_session", {"action": "start"})
        self.assertEqual("ready", started["lifecycle_state"])
        self.assertEqual("idle", started["state"])
        self.assertFalse(started["actuation_enabled"])

    async def test_prepare_submit_and_state_are_sanitized(self):
        service = self.client.server.app[SERVICE_KEY]
        session = {"session_id": str(uuid.uuid4()), "epoch": 1, "fence": "x" * 32}
        prepared = await self.call("teleop_session", {"action": "prepare_shadow", **session})
        self.assertNotIn("fence", prepared)
        frame = valid_frame(service.runtime, session)
        submitted = await self.call("teleop_session", {
            "action": "submit_shadow_frame",
            "frame": frame,
        })
        self.assertEqual("active_shadow", submitted["state"])
        state = await self.call("teleop_state", {})
        self.assertEqual(1, state["pose"]["latest_sequence"])
        for _ in range(100):
            state = await self.call("teleop_state", {})
            if state["dispatch"]["last_would_apply_sequence"] == 1:
                break
            await asyncio.sleep(0.005)
        self.assertEqual(1, state["dispatch"]["last_would_apply_sequence"])
        self.assertEqual("recording", state["dispatch"]["kind"])
        self.assertFalse(contains_value(state, session["fence"]))
        self.assertFalse(contains_value(state, TEST_SECRET))

    async def test_stale_prepare_does_not_disconnect_current_rtc(self):
        service = self.client.server.app[SERVICE_KEY]
        service.rtc.close_all = mock.AsyncMock()
        session = {"session_id": str(uuid.uuid4()), "epoch": 1, "fence": "q" * 32}
        await self.call("teleop_session", {"action": "prepare_shadow", **session})
        service.rtc.close_all.reset_mock()

        _, payload = await self.rpc("tools/call", {
            "name": "teleop_session",
            "arguments": {
                "action": "prepare_shadow",
                "session_id": str(uuid.uuid4()),
                "epoch": 1,
                "fence": "w" * 32,
            },
        })
        self.assertEqual("stale_epoch", payload["error"]["data"]["code"])
        service.rtc.close_all.assert_not_awaited()
        self.assertEqual(session["session_id"], service.runtime.status()["session_id"])

    async def test_only_mcp_heartbeat_renews_lease(self):
        service = self.client.server.app[SERVICE_KEY]
        session = {"session_id": str(uuid.uuid4()), "epoch": 1, "fence": "h" * 32}
        await self.call("teleop_session", {"action": "prepare_shadow", **session})
        result = await self.call("teleop_session", {
            "action": "heartbeat",
            "boot_id": service.runtime.boot_id,
            **session,
        })
        self.assertEqual(1, result["counters"]["lease_heartbeats"])
        self.assertEqual("agent-core-mcp-heartbeat-only", result["lease"]["source"])

    async def test_invalid_frame_is_json_rpc_invalid_params(self):
        session = {"session_id": str(uuid.uuid4()), "epoch": 1, "fence": "z" * 32}
        await self.call("teleop_session", {"action": "prepare_shadow", **session})
        _, payload = await self.rpc("tools/call", {
            "name": "teleop_session",
            "arguments": {"action": "submit_shadow_frame", "frame": {"schema_version": 1}},
        })
        self.assertEqual(-32602, payload["error"]["code"])
        self.assertEqual("missing_field", payload["error"]["data"]["code"])

    async def test_health_discloses_no_secret(self):
        response = await self.client.get("/health")
        payload = await response.json()
        self.assertTrue(payload["rtc_enabled"])
        self.assertTrue(payload["ready"])
        self.assertEqual("recording", payload["dispatch"]["kind"])
        self.assertFalse(payload["actuation_enabled"])
        serialized = json.dumps(payload, sort_keys=True)
        self.assertNotIn(TEST_DRIVER_TOKEN, serialized)
        self.assertNotIn(TEST_SECRET, serialized)


class MissingSecretTests(unittest.IsolatedAsyncioTestCase):
    async def test_registration_disabled_loopback_is_explicit_diagnostic_mode(self):
        old_secret = os.environ.pop("MOTUS_TELEOP_TICKET_SECRET", None)
        try:
            with mock.patch.dict(os.environ, {"MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN}):
                client = TestClient(TestServer(create_app(TEST_CONFIG)))
                await client.start_server()
                try:
                    health = await (await client.get("/health")).json()
                    self.assertFalse(health["rtc_enabled"])
                    self.assertFalse(health["ready"])
                    self.assertEqual("diagnostic", health["state"])
                    self.assertEqual("disabled", health["registration"]["state"])
                    offer = await client.post(
                        "/offer",
                        json={"type": "offer", "sdp": "x", "ticket": "x.y"},
                        headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"},
                    )
                    self.assertEqual(503, offer.status)
                    mcp = await client.post("/mcp", json={
                        "jsonrpc": "2.0", "id": 1, "method": "tools/list", "params": {},
                    }, headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"})
                    self.assertEqual(200, mcp.status)
                    tools = (await mcp.json())["result"]["tools"]
                    self.assertTrue(tools)
                    for tool in tools:
                        self.assertNotIn("signaling", tool["x-teleop"])
                finally:
                    await client.close()
        finally:
            if old_secret is not None:
                os.environ["MOTUS_TELEOP_TICKET_SECRET"] = old_secret


class DeploymentAuthenticationTests(unittest.IsolatedAsyncioTestCase):
    async def test_driver_bearer_matches_core_length_and_character_contract(self):
        invalid_tokens = (
            "x" * 23,
            "x" * 24 + "!",
            "遥" * 24,
            "x" * 4097,
        )
        for token in invalid_tokens:
            with (
                self.subTest(length=len(token), suffix=token[-1:]),
                mock.patch.dict(os.environ, {
                    "MOTUS_DRIVER_TOKEN": token,
                    "MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET,
                }),
                self.assertRaisesRegex(
                    ValueError, "24–4096 restricted ASCII Bearer characters"
                ),
            ):
                ShadowDriverService(copy.deepcopy(TEST_CONFIG))

        for token in ("x" * 24, "A._~+/=-" * 3, "x" * 4096):
            with (
                self.subTest(valid_length=len(token)),
                mock.patch.dict(os.environ, {
                    "MOTUS_DRIVER_TOKEN": token,
                    "MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET,
                }),
            ):
                service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
            try:
                self.assertEqual(token, service.driver_token)
            finally:
                await service.close()

    async def test_ticket_secret_requires_32_utf8_bytes_when_configured(self):
        for invalid_secret in ("", "t" * 31):
            with (
                self.subTest(length=len(invalid_secret)),
                mock.patch.dict(os.environ, {
                    "MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN,
                    "MOTUS_TELEOP_TICKET_SECRET": invalid_secret,
                }),
                self.assertRaisesRegex(ValueError, "at least 32 bytes"),
            ):
                ShadowDriverService(copy.deepcopy(TEST_CONFIG))

        for secret in ("t" * 32, "遥" * 11):
            with (
                self.subTest(utf8_bytes=len(secret.encode("utf-8"))),
                mock.patch.dict(os.environ, {
                    "MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN,
                    "MOTUS_TELEOP_TICKET_SECRET": secret,
                }),
            ):
                service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
            try:
                self.assertTrue(service.rtc.enabled)
            finally:
                await service.close()

    async def test_registration_enabled_requires_driver_token(self):
        config = copy.deepcopy(TEST_CONFIG)
        config["registration"]["enabled"] = True
        with (
            mock.patch.dict(os.environ, {"MOTUS_DRIVER_TOKEN": ""}),
            self.assertRaisesRegex(ValueError, "MOTUS_DRIVER_TOKEN is required"),
        ):
            ShadowDriverService(config)

    async def test_registration_enabled_requires_ticket_secret(self):
        config = copy.deepcopy(TEST_CONFIG)
        config["registration"]["enabled"] = True
        with (
            mock.patch.dict(os.environ, {
                "MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN,
            }, clear=True),
            self.assertRaisesRegex(
                ValueError, "MOTUS_TELEOP_TICKET_SECRET is required"
            ),
        ):
            ShadowDriverService(config)

    async def test_non_loopback_bind_requires_driver_token(self):
        config = copy.deepcopy(TEST_CONFIG)
        config["bind_host"] = "0.0.0.0"
        with (
            mock.patch.dict(os.environ, {"MOTUS_DRIVER_TOKEN": ""}),
            self.assertRaisesRegex(ValueError, "MOTUS_DRIVER_TOKEN is required"),
        ):
            ShadowDriverService(config)

    async def test_registration_disabled_loopback_allows_local_diagnostics(self):
        with mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": "",
        }, clear=True):
            service = ShadowDriverService(copy.deepcopy(TEST_CONFIG))
        try:
            self.assertIsNone(service.driver_token)
            self.assertTrue(service.runtime.status()["dispatch"]["ready"])
            self.assertFalse(service.rtc.enabled)
        finally:
            await service.close()


class MultiInstanceIdentityTests(unittest.IsolatedAsyncioTestCase):
    async def test_instance_environment_overrides_are_loaded(self):
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

    async def test_driver_id_matches_core_64_character_boundary(self):
        accepted_config = copy.deepcopy(TEST_CONFIG)
        accepted_config["driver_id"] = "d" * 64
        rejected_config = copy.deepcopy(TEST_CONFIG)
        rejected_config["driver_id"] = "d" * 65
        with mock.patch.dict(os.environ, {"MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET}):
            accepted = ShadowDriverService(accepted_config)
            try:
                self.assertEqual("d" * 64, accepted.runtime.status()["driver_id"])
            finally:
                await accepted.close()
            with self.assertRaisesRegex(ValueError, "1-64"):
                ShadowDriverService(rejected_config)

    async def test_robot_id_matches_core_64_character_authority_boundary(self):
        accepted_config = copy.deepcopy(TEST_CONFIG)
        accepted_config["robot_id"] = "r" * 64
        rejected_config = copy.deepcopy(TEST_CONFIG)
        rejected_config["robot_id"] = "r" * 65
        with mock.patch.dict(os.environ, {"MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET}):
            accepted = ShadowDriverService(accepted_config)
            try:
                self.assertEqual("r" * 64, accepted.runtime.status()["robot_id"])
            finally:
                await accepted.close()
            with self.assertRaisesRegex(ValueError, "1-64"):
                ShadowDriverService(rejected_config)

    async def test_two_instances_have_distinct_consistent_ids(self):
        config_a = copy.deepcopy(TEST_CONFIG)
        config_a.update({
            "driver_id": "teleop-shadow-lab-a",
            "driver_name": "Lab A Quest Teleop",
            "robot_id": "teleop-shadow-lab-a",
        })
        config_b = copy.deepcopy(TEST_CONFIG)
        config_b.update({
            "driver_id": "teleop-shadow-lab-b",
            "driver_name": "Lab B Quest Teleop",
            "robot_id": "teleop-shadow-lab-b",
        })
        with mock.patch.dict(os.environ, {"MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET}):
            service_a = ShadowDriverService(config_a)
            service_b = ShadowDriverService(config_b)
        try:
            state_a = await service_a.dispatch_tool("teleop_state", {})
            state_b = await service_b.dispatch_tool("teleop_state", {})
            self.assertEqual("teleop-shadow-lab-a", state_a["driver_id"])
            self.assertEqual("teleop-shadow-lab-b", state_b["driver_id"])
            self.assertNotEqual(state_a["driver_id"], state_b["driver_id"])
            self.assertNotEqual(state_a["boot_id"], state_b["boot_id"])

            payload_a = registration_payload(service_a)
            payload_b = registration_payload(service_b)
            self.assertEqual(("teleop-shadow-lab-a", "teleop-shadow-lab-a"), (
                payload_a["id"], payload_a["robot_id"]
            ))
            self.assertEqual(("teleop-shadow-lab-b", "teleop-shadow-lab-b"), (
                payload_b["id"], payload_b["robot_id"]
            ))

            descriptor_a = tool_definitions(
                driver_id=service_a.driver_id,
                driver_name=service_a.driver_name,
                robot_id=service_a.robot_id,
            )[0]["x-teleop"]
            descriptor_b = tool_definitions(
                driver_id=service_b.driver_id,
                driver_name=service_b.driver_name,
                robot_id=service_b.robot_id,
            )[0]["x-teleop"]
            self.assertEqual(payload_a["id"], descriptor_a["driver_id"])
            self.assertEqual(payload_b["id"], descriptor_b["driver_id"])
        finally:
            await service_a.close()
            await service_b.close()

    async def test_two_instances_reject_cross_instance_credentials_without_disclosure(self):
        config_a = copy.deepcopy(TEST_CONFIG)
        config_a.update({
            "driver_id": "teleop-shadow-auth-a",
            "driver_name": "Auth A",
            "robot_id": "teleop-shadow-auth-a",
        })
        config_a["teleop"]["lease_timeout_ms"] = 15_000
        config_b = copy.deepcopy(TEST_CONFIG)
        config_b.update({
            "driver_id": "teleop-shadow-auth-b",
            "driver_name": "Auth B",
            "robot_id": "teleop-shadow-auth-b",
        })
        config_b["teleop"]["lease_timeout_ms"] = 15_000
        driver_token_a = "driver-a-private-sentinel"
        driver_token_b = "driver-b-private-sentinel"
        ticket_secret_a = "ticket-a-private-sentinel-at-least-32-bytes"
        ticket_secret_b = "ticket-b-private-sentinel-at-least-32-bytes"

        with mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": driver_token_a,
            "MOTUS_TELEOP_TICKET_SECRET": ticket_secret_a,
        }):
            client_a = TestClient(TestServer(create_app(config_a)))
        with mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": driver_token_b,
            "MOTUS_TELEOP_TICKET_SECRET": ticket_secret_b,
        }):
            client_b = TestClient(TestServer(create_app(config_b)))

        peer_a = RTCPeerConnection()
        await client_a.start_server()
        await client_b.start_server()
        try:
            services = (
                (client_a, driver_token_a, driver_token_b),
                (client_b, driver_token_b, driver_token_a),
            )
            request = {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "tools/list",
                "params": {},
            }
            for client, own_token, other_token in services:
                with self.subTest(driver=client.server.app[SERVICE_KEY].driver_id):
                    accepted = await client.post(
                        "/mcp",
                        json=request,
                        headers={"Authorization": f"Bearer {own_token}"},
                    )
                    self.assertEqual(200, accepted.status)

                    rejected = await client.post(
                        "/mcp",
                        json=request,
                        headers={"Authorization": f"Bearer {other_token}"},
                    )
                    self.assertEqual(401, rejected.status)
                    self.assertEqual({"error": "unauthorized"}, await rejected.json())

                    rejected_offer = await client.post(
                        "/offer",
                        json={},
                        headers={"Authorization": f"Bearer {other_token}"},
                    )
                    self.assertEqual(401, rejected_offer.status)

                    health = await (await client.get("/health")).json()
                    serialized = json.dumps(health, sort_keys=True)
                    for secret in (
                        driver_token_a,
                        driver_token_b,
                        ticket_secret_a,
                        ticket_secret_b,
                    ):
                        self.assertNotIn(secret, serialized)
                    self.assertTrue(health["mcp_auth_enabled"])
                    self.assertTrue(health["rtc_enabled"])

                    headers = registration_headers(client.server.app[SERVICE_KEY])
                    self.assertEqual(f"Bearer {own_token}", headers["Authorization"])

            control_open = asyncio.Event()
            pose_open = asyncio.Event()
            control = peer_a.createDataChannel("teleop-control", ordered=True)
            pose = peer_a.createDataChannel(
                "teleop-pose",
                ordered=False,
                maxRetransmits=0,
            )

            @control.on("open")
            def on_control_open():
                control_open.set()

            @pose.on("open")
            def on_pose_open():
                pose_open.set()

            await peer_a.setLocalDescription(await peer_a.createOffer())
            offer_sdp = peer_a.localDescription.sdp

            service_a = client_a.server.app[SERVICE_KEY]
            service_b = client_b.server.app[SERVICE_KEY]
            session_a = {
                "session_id": str(uuid.uuid4()),
                "epoch": 1,
                "fence": "a" * 32,
            }
            session_b = {
                "session_id": str(uuid.uuid4()),
                "epoch": 1,
                "fence": "b" * 32,
            }
            prepared_a = await service_a.dispatch_tool(
                "teleop_session",
                {"action": "prepare_shadow", **session_a},
            )
            ticket = TicketCodec(ticket_secret_a).sign(make_ticket_claims(
                session={
                    "boot_id": service_a.runtime.boot_id,
                    **session_a,
                    "capability_digest": prepared_a["capability_digest"],
                },
                sdp=offer_sdp,
                jti="instance_a_ticket_0001",
            ))
            offer_payload = {"type": "offer", "sdp": offer_sdp, "ticket": ticket}

            accepted_offer = await client_a.post(
                "/offer",
                json=offer_payload,
                headers={"Authorization": f"Bearer {driver_token_a}"},
            )
            accepted_text = await accepted_offer.text()
            self.assertEqual(200, accepted_offer.status, accepted_text)
            answer = json.loads(accepted_text)
            self.assertNotIn("fence", answer)
            self.assertNotIn("ticket", answer)
            await peer_a.setRemoteDescription(RTCSessionDescription(
                sdp=answer["sdp"],
                type=answer["type"],
            ))
            await asyncio.wait_for(
                asyncio.gather(control_open.wait(), pose_open.wait()),
                timeout=10,
            )

            await service_b.dispatch_tool(
                "teleop_session",
                {"action": "prepare_shadow", **session_b},
            )
            cross_instance_offer = await client_b.post(
                "/offer",
                json=offer_payload,
                headers={"Authorization": f"Bearer {driver_token_b}"},
            )
            self.assertEqual(401, cross_instance_offer.status)
            cross_instance_error = await cross_instance_offer.json()
            self.assertEqual(
                "invalid_signature",
                cross_instance_error["error"]["code"],
            )
            cross_instance_scan = json.dumps(cross_instance_error, sort_keys=True)
            for secret in (
                driver_token_a,
                driver_token_b,
                ticket_secret_a,
                ticket_secret_b,
                session_a["fence"],
                session_b["fence"],
            ):
                self.assertNotIn(secret, cross_instance_scan)
        finally:
            await peer_a.close()
            await client_a.close()
            await client_b.close()


if __name__ == "__main__":
    unittest.main()
