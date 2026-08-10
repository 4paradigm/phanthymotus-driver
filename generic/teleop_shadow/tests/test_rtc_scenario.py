from __future__ import annotations

import asyncio
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

from main import SERVICE_KEY, create_app
from protocol import TicketCodec, make_ticket_claims

from tests.helpers import (
    TEST_DRIVER_TOKEN,
    TEST_SECRET,
    rtc_wire_frame,
    valid_frame,
)

RTC_CONFIG = {
    "mcp_port": 15711,
    "bind_host": "127.0.0.1",
    "teleop": {
        "lease_timeout_ms": 15_000,
        "pose_timeout_ms": 1000,
        "watchdog_interval_ms": 25,
        "ticket_ttl_max_seconds": 30,
        "ticket_replay_cache_entries": 128,
    },
    "registration": {"enabled": False},
}


class LocalAiortcScenarioTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.env = mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": TEST_DRIVER_TOKEN,
            "MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET,
        })
        self.env.start()
        self.http = TestClient(TestServer(create_app(RTC_CONFIG)))
        await self.http.start_server()
        self.peer = RTCPeerConnection()

    async def asyncTearDown(self):
        await self.peer.close()
        await self.http.close()
        self.env.stop()

    async def rpc_call(self, name: str, arguments: dict) -> dict:
        response = await self.http.post("/mcp", json={
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {"name": name, "arguments": arguments},
        }, headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"})
        payload = await response.json()
        self.assertNotIn("error", payload, payload)
        return json.loads(payload["result"]["content"][0]["text"])

    async def wait_until(self, predicate, *, timeout: float = 10.0):
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            value = predicate()
            if value:
                return value
            await asyncio.sleep(0.02)
        self.fail("timed out waiting for RTC condition")

    async def test_real_offer_two_channels_pose_and_replay_protection(self):
        service = self.http.server.app[SERVICE_KEY]
        session = {"session_id": str(uuid.uuid4()), "epoch": 1, "fence": "r" * 32}
        prepared = await self.rpc_call("teleop_session", {"action": "prepare_shadow", **session})
        await self.rpc_call("teleop_session", {
            "action": "heartbeat",
            "boot_id": service.runtime.boot_id,
            **session,
        })

        control_open = asyncio.Event()
        pose_open = asyncio.Event()
        control_response = asyncio.Event()
        responses: list[dict] = []
        control = self.peer.createDataChannel("teleop-control", ordered=True)
        pose = self.peer.createDataChannel("teleop-pose", ordered=False, maxRetransmits=0)

        @control.on("open")
        def on_control_open():
            control_open.set()

        @pose.on("open")
        def on_pose_open():
            pose_open.set()

        @control.on("message")
        def on_control_message(message):
            responses.append(json.loads(message))
            control_response.set()

        await self.peer.setLocalDescription(await self.peer.createOffer())
        offer_sdp = self.peer.localDescription.sdp
        ticket_session = {
            "boot_id": service.runtime.boot_id,
            "session_id": session["session_id"],
            "epoch": session["epoch"],
            "fence": session["fence"],
            "capability_digest": prepared["capability_digest"],
        }
        ticket = TicketCodec(TEST_SECRET).sign(make_ticket_claims(
            session=ticket_session,
            sdp=offer_sdp,
            ttl_seconds=20,
            jti="local_aiortc_ticket_1234",
        ))
        offer_payload = {
            "type": "offer", "sdp": offer_sdp, "ticket": ticket,
        }
        unauthorized = await self.http.post("/offer", json=offer_payload)
        self.assertEqual(401, unauthorized.status)
        response = await self.http.post(
            "/offer",
            json=offer_payload,
            headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"},
        )
        self.assertEqual(200, response.status, await response.text())
        answer = await response.json()
        self.assertNotIn("fence", answer)
        self.assertFalse(answer["actuation_enabled"])
        await self.peer.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))
        await asyncio.wait_for(asyncio.gather(control_open.wait(), pose_open.wait()), timeout=10)
        await self.wait_until(lambda: service.runtime.status()["rtc"]["connected"])

        # A peer heartbeat is rejected and never renews the Core-owned lease.
        control.send(json.dumps({
            "type": "heartbeat",
            "request_id": "rtc-heartbeat",
        }))
        await asyncio.wait_for(control_response.wait(), timeout=2)
        self.assertFalse(responses[-1]["ok"])
        self.assertEqual("rtc_cannot_renew_lease", responses[-1]["error"]["code"])
        lease_count = service.runtime.status()["counters"]["lease_heartbeats"]
        self.assertEqual(1, lease_count)

        # A browser cannot submit or even echo private authority fields.  The
        # peer-bound RTC contract rejects them before the shared runtime path.
        pose.send(json.dumps(valid_frame(service.runtime, session, sequence=1)))
        await self.wait_until(
            lambda: service.runtime.status()["counters"].get(
                "protocol_unknown_field"
            )
        )
        self.assertIsNone(service.runtime.status()["pose"]["latest_sequence"])

        wire_frame = rtc_wire_frame(service.runtime, session, sequence=1)
        self.assertTrue(
            {"boot_id", "session_id", "epoch", "fence"}.isdisjoint(wire_frame)
        )
        pose.send(json.dumps(wire_frame))
        await self.wait_until(lambda: service.runtime.status()["pose"]["latest_sequence"] == 1)
        await self.wait_until(
            lambda: service.runtime.status()["dispatch"]["last_would_apply_sequence"]
            == 1
        )
        state = await self.rpc_call("teleop_state", {})
        self.assertEqual("active_shadow", state["state"])
        self.assertEqual(1, state["counters"]["frames_from_rtc"])
        self.assertEqual(1, state["dispatch"]["last_would_apply_sequence"])
        self.assertNotIn("fence", state["pose"]["latest"])

        # State-changing commands remain on the authenticated Core REST path so
        # Driver and Core cannot disagree about session ownership.
        control.send(json.dumps({"type": "pause", "request_id": "rtc-pause"}))
        await self.wait_until(lambda: len(responses) >= 2)
        self.assertFalse(responses[-1]["ok"])
        self.assertEqual(
            "rtc_control_requires_core",
            responses[-1]["error"]["code"],
        )
        self.assertEqual("active_shadow", service.runtime.status()["state"])

        replay = await self.http.post(
            "/offer",
            json={"type": "offer", "sdp": offer_sdp, "ticket": ticket},
            headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"},
        )
        self.assertEqual(401, replay.status)
        self.assertEqual("ticket_replayed", (await replay.json())["error"]["code"])

        delayed_ticket = TicketCodec(TEST_SECRET).sign(make_ticket_claims(
            session=ticket_session,
            sdp=offer_sdp,
            ttl_seconds=20,
            jti="post_release_ticket_1234",
        ))

        control.close()
        await self.wait_until(lambda: service.runtime.status()["state"] == "hold")
        self.assertIn(service.runtime.status()["reason"], ("rtc_disconnected", "rtc_closed"))
        stopped = await self.wait_until(
            lambda: (
                dispatch
                if (dispatch := service.runtime.status()["dispatch"])[
                    "stop_acknowledged"
                ]
                and dispatch["adapter"]["records"][-1]["kind"] == "would_stop"
                and dispatch["adapter"]["records"][-1]["reason"]
                in ("rtc_disconnected", "rtc_closed")
                else None
            )
        )
        self.assertEqual("safe_reclutch_required", stopped["state"])

        released = await self.rpc_call("teleop_session", {
            "action": "release",
            "boot_id": service.runtime.boot_id,
            **session,
        })
        self.assertEqual("released", released["state"])
        self.assertIsNone(released["session_id"])
        delayed_offer = await self.http.post(
            "/offer",
            json={"type": "offer", "sdp": offer_sdp, "ticket": delayed_ticket},
            headers={"Authorization": f"Bearer {TEST_DRIVER_TOKEN}"},
        )
        self.assertEqual(409, delayed_offer.status)
        self.assertEqual("session_inactive", (await delayed_offer.json())["error"]["code"])
        self.assertEqual("released", service.runtime.status()["state"])


if __name__ == "__main__":
    unittest.main()
