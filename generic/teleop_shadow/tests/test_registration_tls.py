from __future__ import annotations

import asyncio
import datetime
import ipaddress
import os
import ssl
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
    RegistrationTlsError,
    ShadowDriverService,
    _registration_loop,
    build_registration_ssl_context,
)

from tests.helpers import TEST_SECRET


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


if __name__ == "__main__":
    unittest.main()
