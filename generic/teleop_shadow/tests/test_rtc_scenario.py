from __future__ import annotations

import asyncio
import json
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer
from aiortc import RTCPeerConnection, RTCSessionDescription

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from main import SERVICE_KEY, create_app

from tests.helpers import new_session, rtc_wire_frame

RTC_CONFIG = {
    "mcp_port": 15711,
    "bind_host": "127.0.0.1",
    "teleop": {
        "lease_timeout_ms": 5000,
        "pose_timeout_ms": 1000,
        "watchdog_interval_ms": 25,
        "ticket_ttl_max_seconds": 30,
        "ticket_replay_cache_entries": 128,
    },
    "capture": {
        "pairing_ttl_seconds": 60,
        "presence_interval_ms": 250,
        "presence_timeout_ms": 1500,
        "public_wss_url": "wss://robot.test:15712/ws/teleop-capture",
        "ca_certificate_base64": "dGVzdC1jYQ==",
    },
    "registration": {"enabled": False},
}


class DriverOwnedCaptureE2ETests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.env = mock.patch.dict(os.environ, {})
        self.env.start()
        self.http = TestClient(TestServer(create_app(
            RTC_CONFIG,
            allow_insecure_test_transport=True,
        )))
        await self.http.start_server()
        self.peer = RTCPeerConnection()
        self.request_id = 0

    async def asyncTearDown(self):
        await self.peer.close()
        await self.http.close()
        self.env.stop()

    async def rpc_call(self, name: str, arguments: dict) -> dict:
        self.request_id += 1
        response = await self.http.post("/mcp", json={
            "jsonrpc": "2.0",
            "id": self.request_id,
            "method": "tools/call",
            "params": {"name": name, "arguments": arguments},
        })
        payload = await response.json()
        self.assertNotIn("error", payload, payload)
        return json.loads(payload["result"]["content"][0]["text"])

    async def receive_type(self, websocket, expected: str, *, timeout: float = 5.0) -> dict:
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            message = await websocket.receive(timeout=deadline - asyncio.get_running_loop().time())
            self.assertEqual(WSMsgType.TEXT, message.type, message)
            payload = json.loads(message.data)
            if payload.get("type") == expected:
                return payload
        self.fail(f"timed out waiting for Capture message {expected}")

    async def wait_until(self, predicate, *, timeout: float = 8.0):
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            result = predicate()
            if result:
                return result
            await asyncio.sleep(0.02)
        self.fail("timed out waiting for condition")

    async def wait_counter_stable(self, getter, *, timeout: float = 2.0):
        deadline = asyncio.get_running_loop().time() + timeout
        previous = getter()
        unchanged = 0
        while asyncio.get_running_loop().time() < deadline:
            await asyncio.sleep(0.05)
            current = getter()
            if current == previous:
                unchanged += 1
                if unchanged >= 4:
                    return current
            else:
                previous = current
                unchanged = 0
        self.fail("timed out waiting for counter to become stable")

    async def pair_and_focus(self):
        pairing = await self.rpc_call("teleop_session", {
            "action": "pair_headset",
            "instance_id": "canvas-teleop",
        })
        websocket = await self.http.ws_connect("/ws/teleop-capture")
        await websocket.send_json({
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": "1.2.0",
        })
        paired = await self.receive_type(websocket, "paired")
        await websocket.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        await self.receive_type(websocket, "presence_ack")
        assignment_message = await self.receive_type(websocket, "assignment")
        return websocket, paired, assignment_message["assignment"]

    async def test_stock_core_start_pair_assignment_rtc_pose_pause_release(self):
        started = await self.rpc_call("teleop_session", {
            "action": "start",
            "instance_id": "canvas-teleop",
        })
        self.assertEqual("prepared_shadow", started["state"])
        self.assertFalse(started["lease"]["armed"])

        websocket, paired, assignment = await self.pair_and_focus()
        self.assertEqual(started["session_id"], assignment["session_id"])
        self.assertEqual("shadow", assignment["mode"])
        self.assertEqual("generic_shadow_v1", assignment["profile_id"])
        self.assertEqual(["left_arm", "right_arm"], assignment["effectors"])
        self.assertNotIn("fence", json.dumps(assignment))

        control_open = asyncio.Event()
        pose_open = asyncio.Event()
        control_responses: list[dict] = []
        control_response = asyncio.Event()
        control = self.peer.createDataChannel("teleop-control", ordered=True)
        pose = self.peer.createDataChannel(
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

        @control.on("message")
        def on_control_message(message):
            control_responses.append(json.loads(message))
            control_response.set()

        async def keep_capture_lease_during_local_ice():
            while True:
                await websocket.send_json({
                    "type": "presence",
                    "state": "rtc_connecting",
                    "assignment_id": assignment["id"],
                })
                await asyncio.sleep(0.2)

        presence_task = asyncio.create_task(keep_capture_lease_during_local_ice())
        try:
            await self.peer.setLocalDescription(await self.peer.createOffer())
            await websocket.send_json({
                "type": "signaling_offer",
                "assignment_id": assignment["id"],
                "offer": {
                    "type": "offer",
                    "sdp": self.peer.localDescription.sdp,
                },
            })
            answer_message = await self.receive_type(
                websocket,
                "signaling_answer",
                timeout=12,
            )
            self.assertEqual(assignment["id"], answer_message["assignment_id"])
            self.assertEqual({"type", "sdp"}, set(answer_message["answer"]))
            await self.peer.setRemoteDescription(RTCSessionDescription(
                sdp=answer_message["answer"]["sdp"],
                type=answer_message["answer"]["type"],
            ))
            await asyncio.wait_for(
                asyncio.gather(control_open.wait(), pose_open.wait()),
                timeout=8,
            )
        finally:
            presence_task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await presence_task

        service = self.http.server.app[SERVICE_KEY]
        await self.wait_until(lambda: service.runtime.status()["rtc"]["connected"])
        # Let already-sent Capture presence messages drain before proving the
        # RTC control/pose paths themselves cannot extend the lease.
        before_pose_lease_count = await self.wait_counter_stable(
            lambda: service.runtime.status()["counters"]["lease_heartbeats"]
        )

        # DataChannel heartbeat is not the Capture control connection and may
        # never extend the lease.
        control.send(json.dumps({"type": "heartbeat", "request_id": "rtc-heartbeat"}))
        await asyncio.wait_for(control_response.wait(), timeout=2)
        self.assertFalse(control_responses[-1]["ok"])
        self.assertEqual(
            "rtc_cannot_renew_lease",
            control_responses[-1]["error"]["code"],
        )

        wire_frame = rtc_wire_frame(service.runtime, new_session(), sequence=1)
        self.assertTrue({"boot_id", "session_id", "epoch", "fence"}.isdisjoint(wire_frame))
        pose.send(json.dumps(wire_frame))
        await self.wait_until(
            lambda: service.runtime.status()["dispatch"]["last_would_apply_sequence"] == 1
        )
        state = await self.rpc_call("teleop_state", {})
        self.assertEqual("active_shadow", state["state"])
        self.assertEqual(1, state["pose"]["latest_sequence"])
        self.assertEqual(before_pose_lease_count, state["counters"]["lease_heartbeats"])
        self.assertTrue(state["capture_control"]["connected"])
        self.assertEqual(paired["capture_id"], state["capture_control"]["capture_id"])

        paused = await self.rpc_call("teleop_session", {
            "action": "pause",
            "instance_id": "canvas-teleop",
        })
        self.assertEqual("paused", paused["state"])
        revoked = await self.receive_type(websocket, "assignment_revoked")
        self.assertEqual(assignment["id"], revoked["assignment_id"])
        await websocket.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        paused_error = await self.receive_type(websocket, "error")
        self.assertEqual("session_paused", paused_error["code"])

        released = await self.rpc_call("teleop_session", {
            "action": "release",
            "instance_id": "canvas-teleop",
        })
        self.assertEqual("released", released["state"])
        self.assertIsNone(released["session_id"])
        await websocket.close()

    async def test_capture_disconnect_enters_hold_and_credential_can_reconnect(self):
        await self.rpc_call("teleop_session", {
            "action": "start",
            "instance_id": "canvas-teleop",
        })
        websocket, paired, _assignment = await self.pair_and_focus()
        service = self.http.server.app[SERVICE_KEY]
        await websocket.close()
        await self.wait_until(
            lambda: service.runtime.status()["state"] == "hold"
            and service.runtime.status()["reason"] == "capture_disconnected"
        )

        reconnect = await self.http.ws_connect("/ws/teleop-capture")
        await reconnect.send_json({
            "type": "credential",
            "capture_id": paired["capture_id"],
            "capture_credential": paired["capture_credential"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": "1.2.0",
        })
        connected = await self.receive_type(reconnect, "connected")
        self.assertEqual(paired["capture_id"], connected["capture_id"])
        await reconnect.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        await self.receive_type(reconnect, "presence_ack")
        replacement = await self.receive_type(reconnect, "assignment")
        self.assertGreater(
            replacement["assignment"]["generation"],
            _assignment["generation"],
        )
        await reconnect.close()

    async def test_presence_timeout_fails_closed_to_hold(self):
        await self.rpc_call("teleop_session", {
            "action": "start",
            "instance_id": "canvas-teleop",
        })
        websocket, _paired, _assignment = await self.pair_and_focus()
        timeout_error = await self.receive_type(
            websocket,
            "error",
            timeout=3,
        )
        self.assertEqual("capture_presence_timeout", timeout_error["code"])
        service = self.http.server.app[SERVICE_KEY]
        await self.wait_until(lambda: service.runtime.status()["state"] == "hold")
        self.assertIn(
            service.runtime.status()["reason"],
            {"capture_disconnected", "rtc_disconnected", "rtc_closed"},
        )

    async def test_headset_can_wait_in_standby_before_pc_starts_session(self):
        service = self.http.server.app[SERVICE_KEY]
        # Pairing is intentionally an operator card action, but after the
        # credential exists a subsequent boot may connect before the PC starts.
        await self.rpc_call("teleop_session", {
            "action": "start", "instance_id": "canvas-teleop",
        })
        pairing = await self.rpc_call("teleop_session", {
            "action": "pair_headset", "instance_id": "canvas-teleop",
        })
        await self.rpc_call("teleop_session", {
            "action": "stop", "instance_id": "canvas-teleop",
        })
        websocket = await self.http.ws_connect("/ws/teleop-capture")
        await websocket.send_json({
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": "1.2.0",
        })
        await self.receive_type(websocket, "paired")
        await websocket.send_json({
            "type": "presence", "state": "xr_standby", "assignment_id": None,
        })
        await self.receive_type(websocket, "presence_ack")
        self.assertFalse(service.runtime.status()["authority_valid"])
        self.assertIsNone((await service.capture.status())["assignment_id"])

        started = await self.rpc_call("teleop_session", {
            "action": "start", "instance_id": "canvas-teleop",
        })
        assignment = await self.receive_type(websocket, "assignment")
        self.assertEqual(started["session_id"], assignment["assignment"]["session_id"])
        await websocket.close()

    async def test_focus_loss_holds_immediately_while_wss_remains_connected(self):
        await self.rpc_call("teleop_session", {
            "action": "start", "instance_id": "canvas-teleop",
        })
        websocket, _paired, assignment = await self.pair_and_focus()
        service = self.http.server.app[SERVICE_KEY]
        await websocket.send_json({
            "type": "presence", "state": "xr_ended", "assignment_id": None,
        })
        acknowledged = await self.receive_type(websocket, "presence_ack")
        self.assertEqual("xr_ended", acknowledged["state"])
        revoked = await self.receive_type(websocket, "assignment_revoked")
        self.assertEqual(assignment["id"], revoked["assignment_id"])
        state = service.runtime.status()
        self.assertEqual("hold", state["state"])
        self.assertEqual("capture_xr_ended", state["reason"])
        self.assertTrue(state["dispatch"]["stop_acknowledged"])
        focus_loss_count = state["counters"]["capture_focus_losses"]
        await websocket.send_json({
            "type": "presence", "state": "browser_ready", "assignment_id": None,
        })
        await self.receive_type(websocket, "presence_ack")
        self.assertEqual(
            focus_loss_count,
            service.runtime.status()["counters"]["capture_focus_losses"],
        )
        self.assertFalse(websocket.closed)
        await websocket.close()


if __name__ == "__main__":
    unittest.main()
