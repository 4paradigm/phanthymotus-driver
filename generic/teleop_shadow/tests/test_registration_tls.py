from __future__ import annotations

import asyncio
import base64
import datetime
import ipaddress
import json
import os
import ssl
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import aiohttp
from aiohttp import web
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from main import (
    CaptureTlsError,
    MAX_CAPTURE_CA_BASE64_CHARS,
    MAX_CAPTURE_CA_PEM_BYTES,
    RegistrationCoordinationError,
    RegistrationCoordinator,
    RegistrationTlsError,
    ShadowDriverService,
    _capture_certificate_base64,
    _post_registration_attempt,
    _registration_loop,
    _validated_capture_ca_base64,
    build_registration_ssl_context,
    build_capture_ssl_context,
)

from tests.helpers import TEST_SECRET


class _ControlledWallClock:
    def __init__(self, value: float):
        self.value = value

    def __call__(self) -> float:
        return self.value

    async def sleep(self, delay: float) -> None:
        self.value += max(0.0, delay)
        await asyncio.sleep(0)


def _write_self_signed_certificate(directory: Path, stem: str) -> tuple[Path, Path]:
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "phanthy-motus")])
    now = datetime.datetime.now(datetime.timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(subject)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=1))
        .not_valid_after(now + datetime.timedelta(days=1))
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .add_extension(
            x509.SubjectAlternativeName([
                x509.DNSName("phanthy-motus"),
                x509.IPAddress(ipaddress.ip_address("127.0.0.1")),
            ]),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )
    certificate_path = directory / f"{stem}.pem"
    key_path = directory / f"{stem}-key.pem"
    certificate_path.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
    key_path.write_bytes(key.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption(),
    ))
    return certificate_path, key_path


async def _registration_ok(_request: web.Request) -> web.Response:
    return web.json_response({"registered": True})


class RegistrationTlsTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        directory = Path(self.temporary_directory.name)
        self.core_certificate, core_key = _write_self_signed_certificate(directory, "core")
        self.wrong_certificate, _ = _write_self_signed_certificate(directory, "wrong")

        self.server_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        self.server_context.load_cert_chain(self.core_certificate, core_key)

    async def asyncSetUp(self):
        app = web.Application()
        app.router.add_post("/api/mcp", _registration_ok)
        self.runner = web.AppRunner(app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, "127.0.0.1", 0, ssl_context=self.server_context)
        await self.site.start()
        sockets = self.site._server.sockets  # type: ignore[union-attr]
        self.endpoint = f"https://127.0.0.1:{sockets[0].getsockname()[1]}/api/mcp"

    async def asyncTearDown(self):
        await self.runner.cleanup()

    def tearDown(self):
        self.temporary_directory.cleanup()

    async def test_pinned_core_certificate_succeeds(self):
        context = build_registration_ssl_context({
            "verify_tls": True,
            "ca_file": str(self.core_certificate),
        })
        self.assertIsInstance(context, ssl.SSLContext)
        self.assertEqual(ssl.CERT_REQUIRED, context.verify_mode)
        self.assertFalse(context.check_hostname)
        async with (
            aiohttp.ClientSession() as session,
            session.post(self.endpoint, ssl=context) as response,
        ):
            self.assertEqual(200, response.status)
            self.assertTrue((await response.json())["registered"])

    async def test_wrong_pinned_certificate_fails_handshake(self):
        context = build_registration_ssl_context({
            "verify_tls": True,
            "ca_file": str(self.wrong_certificate),
        })
        loop = asyncio.get_running_loop()
        previous_handler = loop.get_exception_handler()
        loop.set_exception_handler(lambda _loop, _context: None)
        try:
            with self.assertRaises(aiohttp.ClientError):
                async with aiohttp.ClientSession() as session:
                    await session.post(self.endpoint, ssl=context)
            await asyncio.sleep(0.05)
        finally:
            loop.set_exception_handler(previous_handler)

    async def test_missing_certificate_fails_before_connection(self):
        missing = Path(self.temporary_directory.name) / "missing.pem"
        with self.assertRaisesRegex(RegistrationTlsError, "pinned CA file is missing"):
            build_registration_ssl_context({"verify_tls": True, "ca_file": str(missing)})

    async def test_explicit_local_development_override_disables_verification(self):
        self.assertIs(False, build_registration_ssl_context({"verify_tls": False}))

    async def test_registration_reports_missing_pin_and_keeps_retrying(self):
        missing = Path(self.temporary_directory.name) / "never-created.pem"
        config = {
            "driver_id": "teleop-shadow-tls-retry",
            "driver_name": "TLS retry test",
            "mcp_port": 15711,
            "teleop": {},
            "registration": {
                "enabled": True,
                "agent_core_url": "https://127.0.0.1:1",
                "interval_seconds": 30,
                "verify_tls": True,
                "ca_file": str(missing),
            },
        }
        real_sleep = asyncio.sleep

        async def yield_immediately(_delay: float) -> None:
            await real_sleep(0)

        with mock.patch.dict(os.environ, {
            "MOTUS_DRIVER_TOKEN": "registration-tls-test-driver-token",
            "MOTUS_TELEOP_TICKET_SECRET": TEST_SECRET,
        }):
            service = ShadowDriverService(config)
        with (
            self.assertLogs("teleop-shadow", level="WARNING"),
            mock.patch("main.asyncio.sleep", new=yield_immediately),
        ):
            task = asyncio.create_task(_registration_loop(service))
            try:
                for _ in range(100):
                    if service.registration_status["attempts"] >= 2:
                        break
                    await real_sleep(0)
                self.assertGreaterEqual(service.registration_status["attempts"], 2)
                self.assertEqual("tls_error", service.registration_status["state"])
                self.assertIn("pinned CA file is missing", service.registration_status["last_error"])
            finally:
                task.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await task
                await service.close()

    async def test_capture_wss_tls_requires_matching_public_url_san(self):
        context = build_capture_ssl_context({
            "public_wss_url": "wss://127.0.0.1:15712/ws/teleop-capture",
            "tls_cert_file": str(self.core_certificate),
            "tls_key_file": str(Path(self.temporary_directory.name) / "core-key.pem"),
        })
        self.assertIsInstance(context, ssl.SSLContext)
        self.assertEqual(ssl.TLSVersion.TLSv1_2, context.minimum_version)

        with self.assertRaisesRegex(CaptureTlsError, "SAN does not match"):
            build_capture_ssl_context({
                "public_wss_url": "wss://192.0.2.110:15712/ws/teleop-capture",
                "tls_cert_file": str(self.core_certificate),
                "tls_key_file": str(Path(self.temporary_directory.name) / "core-key.pem"),
            })

    async def test_capture_wss_tls_missing_material_fails_closed(self):
        with self.assertRaisesRegex(CaptureTlsError, "certificate is missing"):
            build_capture_ssl_context({
                "public_wss_url": "wss://127.0.0.1:15712/ws/teleop-capture",
                "tls_cert_file": str(Path(self.temporary_directory.name) / "capture-missing.pem"),
                "tls_key_file": str(Path(self.temporary_directory.name) / "capture-key-missing.pem"),
            })

    async def test_capture_listener_rejects_non_wss_bootstrap_url(self):
        with self.assertRaisesRegex(CaptureTlsError, "must be wss"):
            build_capture_ssl_context({
                "public_wss_url": "ws://127.0.0.1:15712/ws/teleop-capture",
                "tls_cert_file": str(self.core_certificate),
                "tls_key_file": str(Path(self.temporary_directory.name) / "core-key.pem"),
            })

        with self.assertRaisesRegex(CaptureTlsError, "must be wss"):
            build_capture_ssl_context({
                "port": 15712,
                "public_wss_url": "wss://127.0.0.1:15713/ws/teleop-capture",
                "tls_cert_file": str(self.core_certificate),
                "tls_key_file": str(Path(self.temporary_directory.name) / "core-key.pem"),
            })

    async def test_capture_ca_bootstrap_matches_native_32_kib_boundary(self):
        exact = base64.b64encode(b"x" * MAX_CAPTURE_CA_PEM_BYTES).decode("ascii")
        self.assertEqual(MAX_CAPTURE_CA_BASE64_CHARS, len(exact))
        self.assertEqual(exact, _validated_capture_ca_base64(exact))
        oversized = base64.b64encode(
            b"x" * (MAX_CAPTURE_CA_PEM_BYTES + 1)
        ).decode("ascii")
        with self.assertRaisesRegex(ValueError, "decode to 1-32768"):
            _validated_capture_ca_base64(oversized)
        with self.assertRaisesRegex(ValueError, "at most 43692"):
            _validated_capture_ca_base64("A" * (MAX_CAPTURE_CA_BASE64_CHARS + 1))
        with self.assertRaisesRegex(ValueError, "valid base64"):
            _validated_capture_ca_base64("!not-base64!")

        oversized_certificate = Path(self.temporary_directory.name) / "large.pem"
        oversized_certificate.write_bytes(
            b"-----BEGIN CERTIFICATE-----\n"
            + b"A" * MAX_CAPTURE_CA_PEM_BYTES
            + b"\n-----END CERTIFICATE-----\n"
        )
        with self.assertRaisesRegex(CaptureTlsError, "bounded public PEM"):
            _capture_certificate_base64({
                "tls_cert_file": str(oversized_certificate),
            })


class RegistrationCoordinationTests(unittest.IsolatedAsyncioTestCase):
    async def _serve(self, handler):
        app = web.Application()
        app.router.add_post("/api/mcp", handler)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        sockets = site._server.sockets  # type: ignore[union-attr]
        endpoint = f"http://127.0.0.1:{sockets[0].getsockname()[1]}/api/mcp"
        return runner, endpoint

    def _coordinator(self, directory: Path, clock: _ControlledWallClock):
        return RegistrationCoordinator(
            directory / "registration.json",
            wall_clock=clock,
            barrier_sleep=clock.sleep,
            lock_poll_seconds=0.001,
        )

    async def _post(self, session, endpoint, coordinator):
        return await _post_registration_attempt(
            session,
            endpoint,
            payload={"name": "test", "url": "http://localhost:15711/mcp"},
            headers={"Content-Type": "application/json"},
            ssl_context=False,
            coordinator=coordinator,
        )

    async def test_two_drivers_post_serially_after_previous_response_in_new_second(self):
        clock = _ControlledWallClock(1_000.20)
        first_received = asyncio.Event()
        release_first = asyncio.Event()
        starts: list[float] = []
        finishes: list[float] = []
        active = 0
        maximum_active = 0

        async def fake_core(_request: web.Request) -> web.Response:
            nonlocal active, maximum_active
            index = len(starts)
            starts.append(clock())
            active += 1
            maximum_active = max(maximum_active, active)
            try:
                if index == 0:
                    first_received.set()
                    await release_first.wait()
                finishes.append(clock())
                return web.json_response({"registered": True})
            finally:
                active -= 1

        runner, endpoint = await self._serve(fake_core)
        try:
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary).resolve() / "coordination"
                directory.mkdir()
                first = self._coordinator(directory, clock)
                second = self._coordinator(directory, clock)
                async with aiohttp.ClientSession() as session:
                    first_task = asyncio.create_task(self._post(session, endpoint, first))
                    await asyncio.wait_for(first_received.wait(), timeout=2)
                    second_task = asyncio.create_task(self._post(session, endpoint, second))
                    await asyncio.sleep(0.02)
                    self.assertFalse(second_task.done())
                    clock.value = 1_000.75
                    release_first.set()
                    self.assertEqual([200, 200], await asyncio.gather(first_task, second_task))

                self.assertEqual(1, maximum_active)
                self.assertEqual(2, len(starts))
                self.assertGreater(int(starts[1]), int(finishes[0]))
                marker = json.loads(
                    (directory / "registration.json").read_text(encoding="ascii")
                )
                self.assertEqual(1_001, marker["last_attempt_barrier_unix_second"])
                self.assertEqual(
                    0o600,
                    stat.S_IMODE((directory / "registration.json").stat().st_mode),
                )
        finally:
            await runner.cleanup()

    async def test_lost_response_records_attempt_boundary_and_releases_lock(self):
        clock = _ControlledWallClock(2_000.20)
        starts: list[float] = []

        async def fake_core(request: web.Request) -> web.Response:
            starts.append(clock())
            if len(starts) == 1:
                clock.value = 2_000.80
                assert request.transport is not None
                request.transport.close()
                return web.Response(status=200)
            return web.json_response({"registered": True})

        runner, endpoint = await self._serve(fake_core)
        try:
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary).resolve() / "coordination"
                directory.mkdir()
                async with aiohttp.ClientSession() as session:
                    with self.assertRaises(aiohttp.ClientError):
                        await self._post(
                            session,
                            endpoint,
                            self._coordinator(directory, clock),
                        )
                    marker_after_loss = json.loads(
                        (directory / "registration.json").read_text(encoding="ascii")
                    )
                    self.assertEqual(
                        2_000,
                        marker_after_loss["last_attempt_barrier_unix_second"],
                    )
                    clock.value = 2_000.90
                    status = await asyncio.wait_for(
                        self._post(
                            session,
                            endpoint,
                            self._coordinator(directory, clock),
                        ),
                        timeout=2,
                    )
                    self.assertEqual(200, status)
                self.assertEqual(2, len(starts))
                self.assertGreater(int(starts[1]), 2_000)
        finally:
            await runner.cleanup()

    async def test_cancellation_before_post_releases_descriptor_without_marker(self):
        clock = _ControlledWallClock(3_000.25)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve() / "coordination"
            directory.mkdir()
            holder = self._coordinator(directory, clock)
            waiter = self._coordinator(directory, clock)
            await holder.__aenter__()

            async def wait_for_lock() -> None:
                async with waiter:
                    await waiter.wait_for_turn()

            task = asyncio.create_task(wait_for_lock())
            await asyncio.sleep(0.02)
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            await holder.__aexit__(None, None, None)
            self.assertFalse((directory / "registration.json").exists())

            successor = self._coordinator(directory, clock)
            await asyncio.wait_for(successor.__aenter__(), timeout=1)
            await successor.__aexit__(None, None, None)

    async def test_cancellation_after_post_persists_barrier_then_unlocks(self):
        clock = _ControlledWallClock(3_500.20)
        received = asyncio.Event()
        finish_handler = asyncio.Event()

        async def fake_core(_request: web.Request) -> web.Response:
            received.set()
            await finish_handler.wait()
            return web.json_response({"registered": True})

        runner, endpoint = await self._serve(fake_core)
        try:
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary).resolve() / "coordination"
                directory.mkdir()
                async with aiohttp.ClientSession() as session:
                    task = asyncio.create_task(
                        self._post(
                            session,
                            endpoint,
                            self._coordinator(directory, clock),
                        )
                    )
                    await asyncio.wait_for(received.wait(), timeout=2)
                    clock.value = 3_500.80
                    task.cancel()
                    with self.assertRaises(asyncio.CancelledError):
                        await task

                marker = json.loads(
                    (directory / "registration.json").read_text(encoding="ascii")
                )
                self.assertEqual(
                    3_500,
                    marker["last_attempt_barrier_unix_second"],
                )
                clock.value = 3_500.90
                successor = self._coordinator(directory, clock)
                async with successor:
                    await successor.wait_for_turn()
                    self.assertGreater(int(clock()), 3_500)
        finally:
            finish_handler.set()
            await runner.cleanup()

    async def test_symlink_marker_fails_closed_and_unlocks(self):
        clock = _ControlledWallClock(4_000.25)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            directory = root / "coordination"
            directory.mkdir()
            outside = root / "outside.json"
            outside.write_text("do-not-touch", encoding="ascii")
            (directory / "registration.json").symlink_to(outside)
            coordinator = self._coordinator(directory, clock)
            with self.assertRaisesRegex(
                RegistrationCoordinationError,
                "safe regular file",
            ):
                async with coordinator:
                    await coordinator.wait_for_turn()
            self.assertEqual("do-not-touch", outside.read_text(encoding="ascii"))

            (directory / "registration.json").unlink()
            successor = self._coordinator(directory, clock)
            async with successor:
                await successor.wait_for_turn()


if __name__ == "__main__":
    unittest.main()
