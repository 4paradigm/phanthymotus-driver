from __future__ import annotations

import asyncio
import base64
import datetime
import ipaddress
import json
import os
import socket
import ssl
import stat
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from aiohttp import ClientSession, WSMsgType
from aiortc import RTCPeerConnection, RTCSessionDescription
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

from teleop.adapter import G1ControllerPoseMapper, G1DualArmAdapter
from teleop.capture import (
    MAX_CAPTURE_CA_BASE64_CHARS,
    MAX_CAPTURE_CA_PEM_BYTES,
    CaptureError,
    CaptureManager,
)
from teleop.capture_server import (
    CaptureTlsError,
    build_capture_ssl_context,
    capture_certificate_base64,
)
from teleop.dispatch import AdapterAck
from teleop.dispatch import RecordingAdapter
from teleop.protocol import TicketCodec
from teleop.runtime import G1TeleopRuntime
from teleop.service import G1TeleopService

from tests.helpers import (
    FakeIkDiagnostic,
    FakeIkSolver,
    FakeLowStateReader,
    rtc_frame,
    session,
    startup_preflight,
)


def _available_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


class ManualClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class InstantRtc:
    def __init__(self):
        self.offers = 0
        self.closes = 0

    async def accept_offer(self, payload):
        self.offers += 1
        return {"type": "answer", "sdp": f"answer-{self.offers}"}

    async def close_all(self):
        self.closes += 1


def _write_capture_identity(directory: Path, host: str = "127.0.0.1") -> tuple[Path, Path]:
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, host),
    ])
    try:
        san_name = x509.IPAddress(ipaddress.ip_address(host))
    except ValueError:
        san_name = x509.DNSName(host)
    now = datetime.datetime.now(datetime.timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=1))
        .not_valid_after(now + datetime.timedelta(days=2))
        .add_extension(x509.SubjectAlternativeName([san_name]), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .sign(key, hashes.SHA256())
    )
    certificate_path = directory / "capture-cert.pem"
    key_path = directory / "capture-key.pem"
    certificate_path.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
    key_path.write_bytes(key.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption(),
    ))
    certificate_path.chmod(0o644)
    key_path.chmod(0o600)
    return certificate_path, key_path


def _public_pem_with_size(size: int) -> bytes:
    prefix = b"-----BEGIN CERTIFICATE-----\n"
    suffix = b"\n-----END CERTIFICATE-----\n"
    if size < len(prefix) + len(suffix):
        raise ValueError("requested PEM size is too small")
    return prefix + (b"A" * (size - len(prefix) - len(suffix))) + suffix


class FakeArmSdk:
    publisher_count = 1

    def __init__(self):
        self.apply_calls = 0
        self.stop_calls = 0
        self.closed = False

    def startup_safe(self, deadline):
        del deadline
        return AdapterAck(True)

    def apply_target(self, *args, **kwargs):
        del args, kwargs
        self.apply_calls += 1
        return AdapterAck(True)

    def safe_stop(self, *args, **kwargs):
        del args, kwargs
        self.stop_calls += 1
        return AdapterAck(True)

    def snapshot(self):
        return {"arm_sdk_weight": 0.0, "fault_reason": None}

    def external_fault_code(self):
        return None

    def external_release_signal(self):
        return {"generation": 0, "reason": None, "acknowledged": True}

    def close(self):
        self.closed = True
        return AdapterAck(True)


class CaptureServiceHarness:
    def __init__(
        self,
        directory: Path,
        *,
        mode: str = "shadow",
        state_file: Path | None = None,
        presence_timeout_ms: int = 1500,
    ):
        self.port = _available_port()
        self.cert, self.key = _write_capture_identity(directory)
        self.low_state = FakeLowStateReader()
        self.arm_sdk = FakeArmSdk() if mode == "live" else None
        adapter = G1DualArmAdapter(
            mode=mode,
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=self.low_state,
            arm_sdk=self.arm_sdk,
        )
        self.runtime = G1TeleopRuntime(
            mode=mode,
            adapter=adapter,
            lease_timeout_ms=5000,
            pose_timeout_ms=1000,
            watchdog_interval_ms=25,
        )
        self.url = f"wss://127.0.0.1:{self.port}/ws/teleop-capture"
        capture_config = {
            "bind_host": "127.0.0.1",
            "port": self.port,
            "public_wss_url": self.url,
            "tls_cert_file": str(self.cert),
            "tls_key_file": str(self.key),
            "state_file": str(state_file) if state_file else None,
            "pairing_ttl_seconds": 60,
            "presence_interval_ms": 250,
            "presence_timeout_ms": presence_timeout_ms,
            "ticket_ttl_seconds": 20,
        }
        self.service = G1TeleopService(
            self.runtime,
            startup_preflight=startup_preflight(self.runtime),
            ik_diagnostic=FakeIkDiagnostic(),
            capture_config=capture_config,
            live_low_state_probe=(
                self.low_state.read_arm_state if mode == "live" else None
            ),
            offer_timeout_s=15.0,
        )
        self.client_ssl = ssl.create_default_context(cafile=str(self.cert))

    def close(self):
        self.service.close()


class G1DriverOwnedCaptureE2ETests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.harness = CaptureServiceHarness(self.root)
        self.http = ClientSession()
        self.peer = RTCPeerConnection()
        self.websockets = []

    async def asyncTearDown(self):
        for websocket in self.websockets:
            await websocket.close()
        await self.peer.close()
        await self.http.close()
        await asyncio.to_thread(self.harness.close)
        self.temporary.cleanup()

    async def _dispatch(self, action: str, *, instance_id: str = "capture-card") -> dict:
        return await asyncio.to_thread(
            self.harness.service.dispatch,
            "teleop_session",
            {"action": action, "instance_id": instance_id},
        )

    async def _receive_type(self, websocket, expected: str, *, timeout: float = 12.0) -> dict:
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            message = await websocket.receive(
                timeout=deadline - asyncio.get_running_loop().time()
            )
            self.assertEqual(WSMsgType.TEXT, message.type, message)
            payload = json.loads(message.data)
            if payload.get("type") == expected:
                return payload
        self.fail(f"timed out waiting for Capture message {expected}")

    async def _wait_until(self, predicate, *, timeout: float = 10.0):
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            result = predicate()
            if result:
                return result
            await asyncio.sleep(0.02)
        self.fail("timed out waiting for condition")

    async def _pair_and_focus(self):
        pairing = await self._dispatch("pair_headset")
        websocket = await self.http.ws_connect(
            self.harness.url,
            ssl=self.harness.client_ssl,
        )
        self.websockets.append(websocket)
        await websocket.send_json({
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": "1.2.0",
        })
        paired = await self._receive_type(websocket, "paired")
        await websocket.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        await self._receive_type(websocket, "presence_ack")
        assignment = (await self._receive_type(websocket, "assignment"))["assignment"]
        return websocket, paired, assignment

    async def _connect_rtc(self, websocket, assignment):
        control_open = asyncio.Event()
        pose_open = asyncio.Event()
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

        async def keep_presence():
            while True:
                await websocket.send_json({
                    "type": "presence",
                    "state": "rtc_connecting",
                    "assignment_id": assignment["id"],
                })
                await asyncio.sleep(0.25)

        presence_task = asyncio.create_task(keep_presence())
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
        finally:
            presence_task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await presence_task
        answer = await self._receive_type(websocket, "signaling_answer", timeout=20.0)
        await self.peer.setRemoteDescription(RTCSessionDescription(
            sdp=answer["answer"]["sdp"],
            type=answer["answer"]["type"],
        ))
        await asyncio.wait_for(
            asyncio.gather(control_open.wait(), pose_open.wait()),
            timeout=10.0,
        )
        await websocket.send_json({
            "type": "presence",
            "state": "streaming",
            "assignment_id": assignment["id"],
        })
        await self._receive_type(websocket, "presence_ack")
        await self._wait_until(lambda: self.harness.runtime.status()["rtc"]["connected"])
        return control, pose

    async def test_start_pair_real_rtc_frame_pause_release(self):
        started = await self._dispatch("start")
        self.assertEqual("prepared_shadow", started["state"])
        self.assertFalse(started["lease"]["armed"])
        self.assertFalse(started["publisher_present"])

        websocket, paired, assignment = await self._pair_and_focus()
        self.assertEqual(started["session_id"], assignment["session_id"])
        self.assertEqual("shadow", assignment["mode"])
        self.assertEqual("unitree_g1_23_dual_arm_controller_v1", assignment["profile_id"])
        self.assertEqual(["dual_arm"], assignment["effectors"])
        self.assertNotIn("fence", json.dumps(assignment))
        health_json = json.dumps(self.harness.service.health(), sort_keys=True)
        self.assertNotIn(paired["capture_credential"], health_json)
        self.assertNotIn("pairing_code", health_json)
        self.assertNotIn('"ticket":', health_json.lower())
        self.assertNotIn("ticket_secret", health_json.lower())

        control, pose = await self._connect_rtc(websocket, assignment)
        lease_count = self.harness.runtime.status()["counters"]["lease_heartbeats"]
        response_ready = asyncio.Event()
        responses = []

        @control.on("message")
        def on_control_message(message):
            responses.append(json.loads(message))
            response_ready.set()

        control.send(json.dumps({"type": "heartbeat", "request_id": "no-renew"}))
        await asyncio.wait_for(response_ready.wait(), timeout=2.0)
        self.assertFalse(responses[-1]["ok"])
        self.assertEqual("rtc_cannot_renew_lease", responses[-1]["error"]["code"])

        identity = session()
        pose.send(json.dumps(rtc_frame(
            self.harness.runtime,
            identity,
            sequence=1,
            clutch_sequence=0,
            deadman=False,
        )))
        await self._wait_until(
            lambda: self.harness.runtime.status()["pose"]["latest_sequence"] == 1
        )
        pose.send(json.dumps(rtc_frame(
            self.harness.runtime,
            identity,
            sequence=2,
            clutch_sequence=1,
            deadman=True,
        )))
        state = await self._wait_until(lambda: (
            snapshot
            if (snapshot := self.harness.runtime.status())["dispatch"].get(
                "last_would_apply_sequence"
            ) == 2
            else None
        ))
        self.assertEqual("active_shadow", state["state"])
        self.assertEqual("would_apply", state["output"]["state"])
        self.assertFalse(state["output_active"])
        self.assertEqual(lease_count, state["counters"]["lease_heartbeats"])
        self.assertEqual(paired["capture_id"], state["capture"]["capture_id"])

        paused = await self._dispatch("pause")
        self.assertEqual("paused", paused["state"])
        revoked = await self._receive_type(websocket, "assignment_revoked")
        self.assertEqual(assignment["id"], revoked["assignment_id"])
        released = await self._dispatch("release")
        self.assertEqual("released", released["state"])
        self.assertIsNone(released["session_id"])
        await websocket.close()

    async def test_focus_reassignment_is_exact_once_and_scoped_per_session(self):
        first = await self._dispatch("start")
        websocket, _paired, assignment = await self._pair_and_focus()
        before = self.harness.runtime.status()["dispatch"]["counters"]["stop_acks"]
        await websocket.send_json({
            "type": "presence",
            "state": "xr_ended",
            "assignment_id": None,
        })
        await self._receive_type(websocket, "presence_ack")
        await self._receive_type(websocket, "assignment_revoked")
        held = self.harness.runtime.status()
        self.assertEqual("capture_xr_ended", held["reason"])
        self.assertEqual(
            before + 1,
            held["dispatch"]["counters"]["stop_acks"],
        )
        for _ in range(2):
            await websocket.send_json({
                "type": "presence",
                "state": "browser_ready",
                "assignment_id": None,
            })
            await self._receive_type(websocket, "presence_ack")
        self.assertEqual(
            before + 1,
            self.harness.runtime.status()["dispatch"]["counters"]["stop_acks"],
        )

        await websocket.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        await self._receive_type(websocket, "presence_ack")
        replacement = (await self._receive_type(websocket, "assignment"))["assignment"]
        self.assertEqual(assignment["generation"] + 1, replacement["generation"])

        await self._dispatch("stop")
        await self._receive_type(websocket, "assignment_revoked")
        second = await self._dispatch("start")
        next_session = (await self._receive_type(websocket, "assignment"))["assignment"]
        self.assertNotEqual(first["session_id"], second["session_id"])
        self.assertEqual(1, next_session["generation"])
        await websocket.close()
        await self._wait_until(
            lambda: self.harness.runtime.status()["reason"] == "capture_disconnected"
        )

    async def test_capture_error_holds_exactly_once_before_socket_await(self):
        await self._dispatch("start")
        websocket, _paired, assignment = await self._pair_and_focus()
        before = self.harness.runtime.status()["dispatch"]["counters"]["stop_acks"]
        await websocket.send_json({
            "type": "presence",
            "state": "error",
            "assignment_id": assignment["id"],
        })
        await self._receive_type(websocket, "presence_ack")
        await self._receive_type(websocket, "assignment_revoked")
        errored = self.harness.runtime.status()
        self.assertEqual("capture_error", errored["reason"])
        self.assertEqual(
            before + 1,
            errored["dispatch"]["counters"]["stop_acks"],
        )
        await websocket.send_json({
            "type": "presence",
            "state": "error",
            "assignment_id": None,
        })
        await self._receive_type(websocket, "presence_ack")
        self.assertEqual(
            before + 1,
            self.harness.runtime.status()["dispatch"]["counters"]["stop_acks"],
        )
        await websocket.close()

    async def test_paired_headset_can_standby_before_pc_starts_next_session(self):
        first = await self._dispatch("start")
        websocket, _paired, assignment = await self._pair_and_focus()
        self.assertEqual(first["session_id"], assignment["session_id"])

        await self._dispatch("stop")
        await self._receive_type(websocket, "assignment_revoked")
        await websocket.send_json({
            "type": "presence",
            "state": "xr_standby",
            "assignment_id": None,
        })
        await self._receive_type(websocket, "presence_ack")
        with self.assertRaises(asyncio.TimeoutError):
            await websocket.receive(timeout=0.1)
        inactive = self.harness.runtime.status()
        self.assertEqual("released", inactive["state"])
        self.assertFalse(inactive["lease"]["armed"])

        second = await self._dispatch("start")
        next_assignment = (
            await self._receive_type(websocket, "assignment")
        )["assignment"]
        self.assertNotEqual(first["session_id"], second["session_id"])
        self.assertEqual(second["session_id"], next_assignment["session_id"])
        self.assertEqual(1, next_assignment["generation"])
        await websocket.close()

    async def test_presence_timeout_holds_before_socket_error(self):
        await asyncio.to_thread(self.harness.close)
        self.harness = CaptureServiceHarness(
            self.root,
            presence_timeout_ms=1000,
        )
        await self._dispatch("start")
        websocket, _paired, _assignment = await self._pair_and_focus()
        before = self.harness.runtime.status()["dispatch"]["counters"]["stop_acks"]
        error = await self._receive_type(websocket, "error", timeout=4.0)
        self.assertEqual("capture_presence_timeout", error["code"])
        state = self.harness.runtime.status()
        self.assertEqual("hold", state["state"])
        self.assertEqual("capture_disconnected", state["reason"])
        self.assertEqual(before + 1, state["dispatch"]["counters"]["stop_acks"])
        await websocket.close()

    async def test_driver_owned_auth_and_compatibility_error_codes_are_exact(self):
        await self._dispatch("start")

        async def rejected_pair(update, expected):
            pairing = await self._dispatch("pair_headset")
            websocket = await self.http.ws_connect(
                self.harness.url,
                ssl=self.harness.client_ssl,
            )
            self.websockets.append(websocket)
            message = {
                "type": "pair",
                "pairing_id": pairing["pairing_id"],
                "pairing_code": pairing["pairing_code"],
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
                **update,
            }
            await websocket.send_json(message)
            error = await self._receive_type(websocket, "error")
            self.assertEqual(expected, error["code"])
            closed = await websocket.receive(timeout=2.0)
            self.assertIn(
                closed.type,
                {WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.CLOSED},
            )

        await rejected_pair(
            {"capture_protocol": "motus.teleop.capture.v0"},
            "capture_protocol_unsupported",
        )
        await rejected_pair(
            {"frame_protocol": "motus.teleop.rtc-frame.v0"},
            "frame_protocol_unsupported",
        )
        await rejected_pair(
            {"client_kind": "browser_webxr"},
            "capture_client_unsupported",
        )
        await rejected_pair(
            {"pairing_code": "x" * 43},
            "capture_pairing_invalid",
        )
        await rejected_pair(
            {"pairing_id": "not-a-canonical-uuid"},
            "capture_pairing_invalid",
        )

        pairing = await self._dispatch("pair_headset")
        websocket = await self.http.ws_connect(
            self.harness.url,
            ssl=self.harness.client_ssl,
        )
        self.websockets.append(websocket)
        await websocket.send_json({
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": "1.2.0",
        })
        paired = await self._receive_type(websocket, "paired")
        await websocket.close()
        await self._wait_until(
            lambda: not self.harness.service._capture_status()["connected"]
        )

        for update in (
            {"capture_credential": "x" * 43},
            {"capture_id": "not-a-canonical-uuid"},
        ):
            reconnect = await self.http.ws_connect(
                self.harness.url,
                ssl=self.harness.client_ssl,
            )
            self.websockets.append(reconnect)
            await reconnect.send_json({
                "type": "credential",
                "capture_id": paired["capture_id"],
                "capture_credential": paired["capture_credential"],
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
                **update,
            })
            error = await self._receive_type(reconnect, "error")
            self.assertEqual("capture_credential_invalid", error["code"])
            closed = await reconnect.receive(timeout=2.0)
            self.assertIn(
                closed.type,
                {WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.CLOSED},
            )


class G1LiveCaptureE2ETests(G1DriverOwnedCaptureE2ETests):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.harness = CaptureServiceHarness(self.root, mode="live")
        self.http = ClientSession()
        self.peer = RTCPeerConnection()
        self.websockets = []

    async def test_live_real_rtc_frame_uses_the_unique_fake_publisher(self):
        started = await self._dispatch("start")
        self.assertTrue(started["actuation_enabled"])
        self.assertTrue(started["publisher_present"])
        self.assertFalse(started["output_active"])
        websocket, _paired, assignment = await self._pair_and_focus()
        self.assertEqual("live", assignment["mode"])
        _control, pose = await self._connect_rtc(websocket, assignment)
        identity = session()
        pose.send(json.dumps(rtc_frame(
            self.harness.runtime,
            identity,
            sequence=1,
            clutch_sequence=0,
            deadman=False,
        )))
        await self._wait_until(
            lambda: self.harness.runtime.status()["pose"]["latest_sequence"] == 1
        )
        pose.send(json.dumps(rtc_frame(
            self.harness.runtime,
            identity,
            sequence=2,
            clutch_sequence=1,
            deadman=True,
        )))
        state = await self._wait_until(lambda: (
            snapshot
            if (snapshot := self.harness.runtime.status())["output"].get("state")
            == "published"
            else None
        ))
        self.assertTrue(state["output_active"])
        self.assertEqual(1, self.harness.arm_sdk.publisher_count)
        self.assertGreaterEqual(self.harness.arm_sdk.apply_calls, 1)
        await websocket.close()

    # The inherited Shadow-specific scenarios are intentionally not duplicated
    # against Live output.
    test_start_pair_real_rtc_frame_pause_release = None
    test_capture_error_holds_exactly_once_before_socket_await = None
    test_focus_reassignment_is_exact_once_and_scoped_per_session = None
    test_paired_headset_can_standby_before_pc_starts_next_session = None
    test_presence_timeout_holds_before_socket_error = None
    test_driver_owned_auth_and_compatibility_error_codes_are_exact = None


class G1CapturePersistenceTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def _persistent_manager(root: Path):
        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=RecordingAdapter(),
            auto_watchdog=False,
        )
        manager = CaptureManager(
            runtime,
            InstantRtc(),
            TicketCodec(b"d" * 32),
            state_file=root / "state" / "capture.json",
            public_wss_url="wss://127.0.0.1:15702/ws/teleop-capture",
            ca_certificate_base64="dGVzdA==",
            presence_interval_ms=1000,
            presence_timeout_ms=5000,
        )
        runtime.prepare_local_session()
        return runtime, manager

    @staticmethod
    async def _pair_manager(manager: CaptureManager, *, app_version: str = "1.2.0"):
        pairing = await manager.create_pairing()
        return await manager.connect({
            "type": "pair",
            "pairing_id": pairing["pairing_id"],
            "pairing_code": pairing["pairing_code"],
            "capture_protocol": "motus.teleop.capture.v1",
            "frame_protocol": "motus.teleop.rtc-frame.v1",
            "client_kind": "native_openxr",
            "app_version": app_version,
        })

    @staticmethod
    def _post_commit_failure(state_file: Path, stage: str):
        if stage == "target_chmod":
            real_chmod = os.chmod

            def fail_target_chmod(path, mode):
                if Path(path) == state_file:
                    raise OSError("target chmod unavailable")
                return real_chmod(path, mode)

            return mock.patch(
                "teleop.capture.os.chmod",
                side_effect=fail_target_chmod,
            )
        if stage == "directory_fsync":
            real_fsync = os.fsync

            def fail_directory_fsync(descriptor):
                if stat.S_ISDIR(os.fstat(descriptor).st_mode):
                    raise OSError("directory fsync unavailable")
                return real_fsync(descriptor)

            return mock.patch(
                "teleop.capture.os.fsync",
                side_effect=fail_directory_fsync,
            )
        raise ValueError(f"unknown persistence failure stage: {stage}")

    async def test_state_commit_is_atomic_and_fsyncs_parent_after_mode_fix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state_file = root / "state" / "capture.json"
            harness = CaptureServiceHarness(root, state_file=state_file)
            events = []
            real_chmod = os.chmod
            real_fsync = os.fsync
            real_replace = os.replace

            def observed_chmod(path, mode):
                events.append(("chmod", str(path), mode))
                return real_chmod(path, mode)

            def observed_fsync(descriptor):
                kind = "directory" if stat.S_ISDIR(os.fstat(descriptor).st_mode) else "file"
                events.append(("fsync", kind))
                return real_fsync(descriptor)

            def observed_replace(source, target):
                events.append(("replace", str(source), str(target)))
                return real_replace(source, target)

            try:
                await asyncio.to_thread(
                    harness.service.dispatch,
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                pairing = await asyncio.to_thread(
                    harness.service.dispatch,
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                with (
                    mock.patch("teleop.capture.os.chmod", side_effect=observed_chmod),
                    mock.patch("teleop.capture.os.fsync", side_effect=observed_fsync),
                    mock.patch("teleop.capture.os.replace", side_effect=observed_replace),
                ):
                    connection, _ = await asyncio.to_thread(
                        harness.service._run_async,
                        harness.service.capture.connect({
                            "type": "pair",
                            "pairing_id": pairing["pairing_id"],
                            "pairing_code": pairing["pairing_code"],
                            "capture_protocol": "motus.teleop.capture.v1",
                            "frame_protocol": "motus.teleop.rtc-frame.v1",
                            "client_kind": "native_openxr",
                            "app_version": "1.2.0",
                        }),
                    )
                await asyncio.to_thread(
                    harness.service._run_async,
                    harness.service.capture.disconnect(connection),
                )
            finally:
                await asyncio.to_thread(harness.close)

            file_fsync = events.index(("fsync", "file"))
            replace = next(
                index for index, event in enumerate(events) if event[0] == "replace"
            )
            target_chmod = events.index(("chmod", str(state_file), 0o600))
            directory_fsync = events.index(("fsync", "directory"))
            self.assertLess(file_fsync, replace)
            self.assertLess(replace, target_chmod)
            self.assertLess(target_chmod, directory_fsync)
            self.assertEqual(0o600, stat.S_IMODE(state_file.stat().st_mode))
            self.assertEqual([], list(state_file.parent.glob(".*.tmp")))

    async def test_repeated_offers_cannot_refresh_presence_or_slide_lease_grace(self):
        clock = ManualClock()
        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=RecordingAdapter(clock=clock),
            lease_timeout_ms=1000,
            auto_watchdog=False,
            clock=clock,
        )
        manager = CaptureManager(
            runtime,
            InstantRtc(),
            TicketCodec(b"bounded-test-ticket-secret-32bytes"),
            public_wss_url=(
                "wss://127.0.0.1:15702/ws/teleop-capture"
            ),
            ca_certificate_base64="dGVzdA==",
            presence_interval_ms=1000,
            presence_timeout_ms=5000,
            clock=clock,
            wall_clock=clock,
        )
        try:
            runtime.prepare_local_session()
            pairing = await manager.create_pairing()
            connection, _ = await manager.connect({
                "type": "pair",
                "pairing_id": pairing["pairing_id"],
                "pairing_code": pairing["pairing_code"],
                "capture_protocol": "motus.teleop.capture.v1",
                "frame_protocol": "motus.teleop.rtc-frame.v1",
                "client_kind": "native_openxr",
                "app_version": "1.2.0",
            })
            await manager.presence(connection, {
                "type": "presence",
                "state": "xr_standby",
                "assignment_id": None,
            })
            assignment_id = connection.assignment["id"]
            self.assertEqual(0.0, connection.last_presence_monotonic)

            clock.value = 0.5
            await manager.signaling_offer(connection, {
                "type": "signaling_offer",
                "assignment_id": assignment_id,
                "offer": {"type": "offer", "sdp": "offer-one"},
            })
            clock.value = 4.0
            await manager.signaling_offer(connection, {
                "type": "signaling_offer",
                "assignment_id": assignment_id,
                "offer": {"type": "offer", "sdp": "offer-two"},
            })
            self.assertEqual(0.0, connection.last_presence_monotonic)

            clock.value = 5.001
            self.assertTrue(manager.presence_expired(connection))
            with self.assertRaises(CaptureError) as caught:
                await manager.signaling_offer(connection, {
                    "type": "signaling_offer",
                    "assignment_id": assignment_id,
                    "offer": {"type": "offer", "sdp": "offer-expired"},
                })
            self.assertEqual("capture_presence_timeout", caught.exception.code)
            self.assertTrue(runtime.status()["lease"]["negotiation_grace"])

            clock.value = 15.001
            expired = runtime.watchdog_tick()
            self.assertFalse(expired["authority_valid"])
            self.assertEqual("lease_timeout", expired["reason"])
        finally:
            runtime.close()

    async def test_credential_survives_restart_and_persist_failure_rolls_back(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state_file = root / "state" / "capture.json"
            first = CaptureServiceHarness(root, state_file=state_file)
            try:
                await asyncio.to_thread(
                    first.service.dispatch,
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                pairing = await asyncio.to_thread(
                    first.service.dispatch,
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                connection, paired = await asyncio.to_thread(
                    first.service._run_async,
                    first.service.capture.connect({
                        "type": "pair",
                        "pairing_id": pairing["pairing_id"],
                        "pairing_code": pairing["pairing_code"],
                        "capture_protocol": "motus.teleop.capture.v1",
                        "frame_protocol": "motus.teleop.rtc-frame.v1",
                        "client_kind": "native_openxr",
                        "app_version": "1.2.0",
                    }),
                )
                await asyncio.to_thread(
                    first.service._run_async,
                    first.service.capture.disconnect(connection),
                )
            finally:
                await asyncio.to_thread(first.close)
            self.assertEqual(0o600, stat.S_IMODE(state_file.stat().st_mode))

            restarted = CaptureServiceHarness(root, state_file=state_file)
            try:
                await asyncio.to_thread(
                    restarted.service.dispatch,
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                connection, acknowledged = await asyncio.to_thread(
                    restarted.service._run_async,
                    restarted.service.capture.connect({
                        "type": "credential",
                        "capture_id": paired["capture_id"],
                        "capture_credential": paired["capture_credential"],
                        "capture_protocol": "motus.teleop.capture.v1",
                        "frame_protocol": "motus.teleop.rtc-frame.v1",
                        "client_kind": "native_openxr",
                        "app_version": "1.2.1",
                    }),
                )
                self.assertEqual("connected", acknowledged["type"])
                await asyncio.to_thread(
                    restarted.service._run_async,
                    restarted.service.capture.disconnect(connection),
                )
                replacement = await asyncio.to_thread(
                    restarted.service.dispatch,
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                persisted_before_failure = state_file.read_bytes()
                with mock.patch.object(
                    os,
                    "replace",
                    side_effect=OSError("disk unavailable"),
                ):
                    with self.assertRaises(CaptureError):
                        await asyncio.to_thread(
                            restarted.service._run_async,
                            restarted.service.capture.connect({
                                "type": "pair",
                                "pairing_id": replacement["pairing_id"],
                                "pairing_code": replacement["pairing_code"],
                                "capture_protocol": "motus.teleop.capture.v1",
                                "frame_protocol": "motus.teleop.rtc-frame.v1",
                                "client_kind": "native_openxr",
                                "app_version": "1.2.1",
                            }),
                        )
                self.assertEqual(persisted_before_failure, state_file.read_bytes())
                self.assertEqual([], list(state_file.parent.glob(".*.tmp")))
                restored, _ = await asyncio.to_thread(
                    restarted.service._run_async,
                    restarted.service.capture.connect({
                        "type": "credential",
                        "capture_id": paired["capture_id"],
                        "capture_credential": paired["capture_credential"],
                        "capture_protocol": "motus.teleop.capture.v1",
                        "frame_protocol": "motus.teleop.rtc-frame.v1",
                        "client_kind": "native_openxr",
                        "app_version": "1.2.1",
                    }),
                )
                await asyncio.to_thread(
                    restarted.service._run_async,
                    restarted.service.capture.disconnect(restored),
                )
                with mock.patch.object(
                    os,
                    "replace",
                    side_effect=OSError("disk unavailable"),
                ):
                    with self.assertRaises(CaptureError):
                        await asyncio.to_thread(
                            restarted.service._run_async,
                            restarted.service.capture.revoke_headset(),
                        )
                status = await asyncio.to_thread(
                    restarted.service._run_async,
                    restarted.service.capture.status(),
                )
                self.assertEqual(1, status["paired_devices"])
                self.assertEqual(persisted_before_failure, state_file.read_bytes())
            finally:
                await asyncio.to_thread(restarted.close)

    async def test_post_replace_failure_keeps_replacement_identity_in_memory_and_disk(self):
        for stage in ("target_chmod", "directory_fsync"):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                state_file = root / "state" / "capture.json"
                runtime, manager = self._persistent_manager(root)
                try:
                    first_connection, first = await self._pair_manager(manager)
                    await manager.disconnect(first_connection)
                    pairing = await manager.create_pairing()
                    with (
                        self._post_commit_failure(state_file, stage),
                        self.assertRaises(CaptureError) as caught,
                    ):
                        await manager.connect({
                            "type": "pair",
                            "pairing_id": pairing["pairing_id"],
                            "pairing_code": pairing["pairing_code"],
                            "capture_protocol": "motus.teleop.capture.v1",
                            "frame_protocol": "motus.teleop.rtc-frame.v1",
                            "client_kind": "native_openxr",
                            "app_version": "1.2.0",
                        })
                    self.assertEqual("capture_state_unavailable", caught.exception.code)
                    persisted = json.loads(state_file.read_text(encoding="utf-8"))
                    self.assertNotEqual(
                        first["capture_id"],
                        persisted["capture"]["capture_id"],
                    )
                    with self.assertRaises(CaptureError) as old_identity:
                        await manager.connect({
                            "type": "credential",
                            "capture_id": first["capture_id"],
                            "capture_credential": first["capture_credential"],
                            "capture_protocol": "motus.teleop.capture.v1",
                            "frame_protocol": "motus.teleop.rtc-frame.v1",
                            "client_kind": "native_openxr",
                            "app_version": "1.2.0",
                        })
                    self.assertEqual(
                        "capture_credential_invalid",
                        old_identity.exception.code,
                    )
                    # A fresh pairing repairs the deliberately unacknowledged
                    # post-commit identity without restarting the Driver.
                    recovered, _identity = await self._pair_manager(manager)
                    await manager.disconnect(recovered)
                finally:
                    runtime.close()

    async def test_post_replace_failure_keeps_version_update_in_memory_and_disk(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state_file = root / "state" / "capture.json"
            runtime, manager = self._persistent_manager(root)
            try:
                connection, identity = await self._pair_manager(manager)
                await manager.disconnect(connection)
                credential_message = {
                    "type": "credential",
                    "capture_id": identity["capture_id"],
                    "capture_credential": identity["capture_credential"],
                    "capture_protocol": "motus.teleop.capture.v1",
                    "frame_protocol": "motus.teleop.rtc-frame.v1",
                    "client_kind": "native_openxr",
                    "app_version": "1.2.1",
                }
                with (
                    self._post_commit_failure(state_file, "directory_fsync"),
                    self.assertRaises(CaptureError) as caught,
                ):
                    await manager.connect(credential_message)
                self.assertEqual("capture_state_unavailable", caught.exception.code)
                persisted = json.loads(state_file.read_text(encoding="utf-8"))
                self.assertEqual("1.2.1", persisted["capture"]["app_version"])
                reconnected, acknowledged = await manager.connect(credential_message)
                self.assertEqual("connected", acknowledged["type"])
                await manager.disconnect(reconnected)
            finally:
                runtime.close()

    async def test_post_replace_failure_keeps_revocation_in_memory_and_disk(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state_file = root / "state" / "capture.json"
            runtime, manager = self._persistent_manager(root)
            try:
                connection, identity = await self._pair_manager(manager)
                with (
                    self._post_commit_failure(state_file, "directory_fsync"),
                    self.assertRaises(CaptureError) as caught,
                ):
                    await manager.revoke_headset()
                self.assertEqual("capture_state_unavailable", caught.exception.code)
                persisted = json.loads(state_file.read_text(encoding="utf-8"))
                self.assertIsNone(persisted["capture"])
                self.assertEqual(0, (await manager.status())["paired_devices"])
                revoked_event = await asyncio.wait_for(
                    connection.events.get(),
                    timeout=1.0,
                )
                self.assertEqual("capture_revoked", revoked_event["type"])
                self.assertEqual(0, connection.generation)
                self.assertIsNone(connection.assignment)
                await manager.disconnect(connection)
                with self.assertRaises(CaptureError) as revoked_identity:
                    await manager.connect({
                        "type": "credential",
                        "capture_id": identity["capture_id"],
                        "capture_credential": identity["capture_credential"],
                        "capture_protocol": "motus.teleop.capture.v1",
                        "frame_protocol": "motus.teleop.rtc-frame.v1",
                        "client_kind": "native_openxr",
                        "app_version": "1.2.0",
                    })
                self.assertEqual(
                    "capture_credential_invalid",
                    revoked_identity.exception.code,
                )
            finally:
                runtime.close()

    async def test_browser_webxr_is_not_an_implicit_native_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            harness = CaptureServiceHarness(Path(directory))
            try:
                await asyncio.to_thread(
                    harness.service.dispatch,
                    "teleop_session",
                    {"action": "start", "instance_id": "card"},
                )
                pairing = await asyncio.to_thread(
                    harness.service.dispatch,
                    "teleop_session",
                    {"action": "pair_headset", "instance_id": "card"},
                )
                with self.assertRaises(CaptureError) as caught:
                    await asyncio.to_thread(
                        harness.service._run_async,
                        harness.service.capture.connect({
                            "type": "pair",
                            "pairing_id": pairing["pairing_id"],
                            "pairing_code": pairing["pairing_code"],
                            "capture_protocol": "motus.teleop.capture.v1",
                            "frame_protocol": "motus.teleop.rtc-frame.v1",
                            "client_kind": "browser_webxr",
                            "app_version": "1.2.0",
                        }),
                    )
                self.assertEqual("capture_client_unsupported", caught.exception.code)
            finally:
                await asyncio.to_thread(harness.close)


class G1CaptureTlsTests(unittest.TestCase):
    def test_ca_downlink_accepts_32kib_decoded_pem_and_rejects_one_more_byte(self):
        exact_pem = _public_pem_with_size(MAX_CAPTURE_CA_PEM_BYTES)
        over_pem = _public_pem_with_size(MAX_CAPTURE_CA_PEM_BYTES + 1)
        exact_base64 = base64.b64encode(exact_pem).decode("ascii")
        over_base64 = base64.b64encode(over_pem).decode("ascii")
        self.assertEqual(32 * 1024, MAX_CAPTURE_CA_PEM_BYTES)
        self.assertEqual(43_692, MAX_CAPTURE_CA_BASE64_CHARS)
        self.assertEqual(MAX_CAPTURE_CA_BASE64_CHARS, len(exact_base64))
        # 32768 and 32769 decoded bytes deliberately share the same standard
        # base64 text length, so the decoded-byte guard is independently tested.
        self.assertEqual(MAX_CAPTURE_CA_BASE64_CHARS, len(over_base64))

        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=RecordingAdapter(),
            auto_watchdog=False,
        )
        service = G1TeleopService(
            runtime,
            startup_preflight=startup_preflight(runtime),
            ik_diagnostic=FakeIkDiagnostic(),
            capture_config={
                "public_wss_url": (
                    "wss://127.0.0.1:15702/ws/teleop-capture"
                ),
                "ca_certificate_base64": exact_base64,
                "presence_interval_ms": 1000,
                "presence_timeout_ms": 5000,
            },
            start_capture_listener=False,
        )
        try:
            service.dispatch(
                "teleop_session",
                {"action": "start", "instance_id": "ca-boundary-card"},
            )
            pairing = service.dispatch(
                "teleop_session",
                {"action": "pair_headset", "instance_id": "ca-boundary-card"},
            )
            self.assertEqual(exact_base64, pairing["ca_certificate_base64"])
            self.assertEqual(exact_pem, base64.b64decode(
                pairing["ca_certificate_base64"],
                validate=True,
            ))
        finally:
            service.close()

        rejected_runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=RecordingAdapter(),
            auto_watchdog=False,
        )
        try:
            with self.assertRaisesRegex(ValueError, "bounded public PEM"):
                G1TeleopService(
                    rejected_runtime,
                    startup_preflight=startup_preflight(rejected_runtime),
                    ik_diagnostic=FakeIkDiagnostic(),
                    capture_config={
                        "public_wss_url": (
                            "wss://127.0.0.1:15702/ws/teleop-capture"
                        ),
                        "ca_certificate_base64": over_base64,
                    },
                    start_capture_listener=False,
                )
        finally:
            rejected_runtime.close()

    def test_certificate_bootstrap_file_uses_the_same_decoded_32kib_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            certificate = Path(directory) / "capture-public-chain.pem"
            exact_pem = _public_pem_with_size(MAX_CAPTURE_CA_PEM_BYTES)
            certificate.write_bytes(exact_pem)
            encoded = capture_certificate_base64({
                "tls_cert_file": str(certificate),
            })
            self.assertEqual(exact_pem, base64.b64decode(encoded, validate=True))

            certificate.write_bytes(
                _public_pem_with_size(MAX_CAPTURE_CA_PEM_BYTES + 1)
            )
            with self.assertRaisesRegex(CaptureTlsError, "bounded public PEM"):
                capture_certificate_base64({
                    "tls_cert_file": str(certificate),
                })

    def test_wss_requires_matching_san_port_and_independent_material(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            certificate, key = _write_capture_identity(root)
            config = {
                "port": 15702,
                "public_wss_url": "wss://127.0.0.1:15702/ws/teleop-capture",
                "tls_cert_file": str(certificate),
                "tls_key_file": str(key),
            }
            context = build_capture_ssl_context(config)
            self.assertEqual(ssl.TLSVersion.TLSv1_2, context.minimum_version)
            for update in (
                {"public_wss_url": "ws://127.0.0.1:15702/ws/teleop-capture"},
                {"public_wss_url": "wss://127.0.0.1:15703/ws/teleop-capture"},
                {"public_wss_url": "wss://192.0.2.10:15702/ws/teleop-capture"},
                {"tls_cert_file": str(root / "missing.pem")},
            ):
                with self.subTest(update=update), self.assertRaises(CaptureTlsError):
                    build_capture_ssl_context({**config, **update})


if __name__ == "__main__":
    unittest.main()
