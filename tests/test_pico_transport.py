import sys
from pathlib import Path
"""Real localhost WebSocket + WebRTC and TLS lifecycle; no PICO/robot I/O."""

import asyncio
import copy
import json
import secrets
import unittest
from unittest.mock import patch
from aiohttp.test_utils import TestClient, TestServer
from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pico/4ultra'))
from ext_vr.capture import CaptureManager
from ext_vr.capture_server import create_capture_app, visualization_stream
from ext_vr.protocol import TicketCodec, TicketVerifier
from ext_vr.rtc import RtcManager
from ext_vr.runtime import DeviceRuntime
from ext_vr.descriptor import CAPTURE_PROTOCOL, RTC_FRAME_PROTOCOL
from test_pico_device import frame


class RealTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_display_timeout_recovers_latest_instead_of_killing_stream(self):
        class Socket:
            closed = False
            count = 0
            values = []

            async def send_json(self, value):
                self.count += 1
                if self.count == 1:
                    raise asyncio.TimeoutError()
                self.values.append(value)
                self.closed = True

        socket = Socket()
        sequence = [0]

        def provider():
            sequence[0] += 1
            return {"schema": "motus.motion.feedback/1", "sequence": sequence[0]}

        await visualization_stream(socket, provider)
        self.assertEqual(socket.count, 2)
        self.assertEqual(socket.values[0]["visualization"]["sequence"], 2)

    async def test_pair_assignment_real_rtc_normalization_disconnect(self):
        # Constrain real ICE sockets to loopback; never probe LAN or robot interfaces.
        addresses = patch("aioice.ice.get_host_addresses", return_value=["127.0.0.1"])
        addresses.start()
        self.addCleanup(addresses.stop)
        runtime = DeviceRuntime("vr-1")
        runtime.start()
        codec = TicketCodec(secrets.token_bytes(32))
        rtc = RtcManager(runtime, TicketVerifier(codec))
        manager = CaptureManager(runtime, rtc, codec)
        client = TestClient(TestServer(create_capture_app(manager)))
        peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        await client.start_server()
        try:
            pair = await manager.create_pairing()
            ws = await client.ws_connect("/ws/teleop-capture")
            await ws.send_json(
                {
                    "type": "pair",
                    "pairing_id": pair["pairing_id"],
                    "pairing_code": pair["pairing_code"],
                    "capture_protocol": CAPTURE_PROTOCOL,
                    "frame_protocol": RTC_FRAME_PROTOCOL,
                    "client_kind": "native_openxr",
                    "app_version": "0.4.0-extvr1-operator1-ikview2",
                }
            )
            ack = await ws.receive_json(timeout=2)
            self.assertEqual(ack["type"], "paired")
            await ws.send_json(
                {"type": "presence", "state": "xr_standby", "assignment_id": None}
            )
            assignment = None
            for _ in range(2):
                value = await ws.receive_json(timeout=2)
                if value["type"] == "assignment":
                    assignment = value["assignment"]
            self.assertIsNotNone(assignment)
            pose = peer.createDataChannel(
                "teleop-pose", ordered=False, maxRetransmits=0
            )
            peer.createDataChannel("teleop-control", ordered=True)
            await peer.setLocalDescription(await peer.createOffer())
            await ws.send_json(
                {
                    "type": "signaling_offer",
                    "assignment_id": assignment["id"],
                    "offer": {"type": "offer", "sdp": peer.localDescription.sdp},
                }
            )
            answer = await ws.receive_json(timeout=5)
            self.assertEqual(answer["type"], "signaling_answer")
            await peer.setRemoteDescription(RTCSessionDescription(**answer["answer"]))
            for _ in range(100):
                if pose.readyState == "open":
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(pose.readyState, "open")
            pose.send(json.dumps(frame(10)))
            output = None
            for _ in range(100):
                output = runtime.take_latest()
                if output:
                    break
                await asyncio.sleep(0.01)
            self.assertIsNotNone(output)
            self.assertEqual(output["schema"], "motus.teleop.command/1")
            self.assertEqual(output["left"]["position"], [-3, -1, 2])
            self.assertEqual(output["device_id"], ack["capture_id"])
            self.assertFalse(runtime.actuation_enabled)
            generation = runtime.generation
            await ws.close()
            await asyncio.sleep(0.05)
            self.assertGreater(runtime.generation, generation)
            self.assertIsNone(runtime.take_latest())
        finally:
            await peer.close()
            await rtc.close_all()
            await client.close()


if __name__ == "__main__":
    unittest.main()
