"""Discovery filters, scan bounds and cancellation; no physical networks used."""

import asyncio
import importlib.util
import ipaddress
import json
from pathlib import Path
import threading
import time
import unittest
from unittest import mock

from yarl import URL


spec = importlib.util.spec_from_file_location(
    "franka_discovery_test", Path(__file__).resolve().parents[1] / "franka/fr3/discovery.py")
discovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(discovery)


def nameplate(**changes):
    return {"status": "Connected", "robotCode": "fr3_arm",
            "robotLabel": "Franka Research 3 Arm", "robotSerial": "TEST-123", **changes}


def desk_session(message):
    response = mock.Mock(status=200, content_type="text/html")

    async def chunks(size):
        yield b'<title>Franka Robotics</title><script src="/app-shell/app.test.js"></script>'

    response.content.iter_chunked = chunks
    context = mock.MagicMock()
    context.__aenter__ = mock.AsyncMock(return_value=response)
    ws = mock.Mock()
    ws.receive = mock.AsyncMock(return_value=mock.Mock(
        type=discovery.aiohttp.WSMsgType.TEXT, data=message))
    ws_context = mock.MagicMock()
    ws_context.__aenter__ = mock.AsyncMock(return_value=ws)
    session = mock.Mock()
    session.get.return_value = context
    session.ws_connect.return_value = ws_context
    return session, ws


class IdentityTests(unittest.TestCase):
    def test_fr3_identity_and_invalid_responses(self):
        self.assertEqual(discovery.identify("172.16.0.3", nameplate())["serial"], "TEST-123")
        for value in [None, [], {}, nameplate(status="NotConnected"),
                      nameplate(robotCode="panda"), nameplate(robotLabel="router"),
                      nameplate(robotSerial=""), nameplate(robotSerial="bad\nlabel")]:
            self.assertIsNone(discovery.identify("172.16.0.3", value))

    def test_only_small_wired_physical_subnets_are_scanned(self):
        records = []
        for name, prefix, status in [("eno1", 24, "UP"), ("wlan0", 24, "UP"),
                                     ("docker0", 24, "UP"), ("eno2", 16, "UP"),
                                     ("eno3", 24, "DOWN")]:
            records.append({"ifname": name, "operstate": status, "addr_info": [
                {"family": "inet", "scope": "global", "local": "172.16.0.99", "prefixlen": prefix}]})

        def exists(path):
            return str(path).endswith("wlan0/wireless") or (
                str(path).endswith("/device") and "docker0" not in str(path))

        with mock.patch.object(discovery.subprocess, "check_output", return_value=json.dumps(records)), \
                mock.patch.object(Path, "exists", exists):
            networks, locals_ = discovery.wired_networks()
        self.assertEqual([(n, str(net)) for n, _, net in networks], [("eno1", "172.16.0.0/24")])
        self.assertIn("172.16.0.99", locals_)


class AsyncDiscoveryTests(unittest.IsolatedAsyncioTestCase):
    async def test_no_redirect_is_followed(self):
        with self.assertRaises(ValueError):
            await discovery.reject_redirect(None, None, None)

    async def test_scan_skips_local_addresses_and_isolates_bad_hosts(self):
        calls = []

        async def inspect(session, ip):
            calls.append(ip)
            if ip.endswith(".2"):
                raise ValueError("Not a robot")
            return discovery.identify(ip, nameplate())

        network = ipaddress.ip_network("172.16.0.0/29")
        with mock.patch.object(discovery, "inspect_robot", inspect):
            result = await discovery.scan_networks([("eno1", "172.16.0.1", network)], {"172.16.0.1"})
        self.assertNotIn("172.16.0.1", calls)
        self.assertEqual(result["state"], "ready")
        self.assertEqual([r["ip"] for r in result["robots"]],
                         ["172.16.0.3", "172.16.0.4", "172.16.0.5", "172.16.0.6"])

    async def test_probe_requests_uncompressed_html(self):
        async def inspect(session, ip):
            # Exercise aiohttp's actual request-header defaults without a socket.
            # auto_decompress=False alone still advertises gzip and deflate.
            request = discovery.aiohttp.ClientRequest(
                "GET", URL(f"https://{ip}/desk/"), headers=session.headers)
            self.assertEqual(request.headers["Accept-Encoding"], "identity")

        network = ipaddress.ip_network("172.16.0.0/30")
        with mock.patch.object(discovery, "inspect_robot", inspect):
            await discovery.scan_networks([("eno1", "172.16.0.1", network)], {"172.16.0.1"})

    async def test_page_filter_does_not_open_nameplate_on_generic_or_large_html(self):
        for body in [b"<title>Router</title>", b"x" * 140000]:
            response = mock.Mock(status=200, content_type="text/html")

            async def chunks(size, data=body):
                yield data

            response.content.iter_chunked = chunks
            context = mock.MagicMock()
            context.__aenter__ = mock.AsyncMock(return_value=response)
            session = mock.Mock()
            session.get.return_value = context
            self.assertIsNone(await discovery.inspect_robot(session, "172.16.0.3"))
            session.ws_connect.assert_not_called()

    async def test_matching_page_and_nameplate_produce_robot(self):
        session, ws = desk_session(json.dumps(nameplate()))
        self.assertEqual((await discovery.inspect_robot(session, "172.16.0.3"))["serial"], "TEST-123")
        ws.send.assert_not_called()

    async def test_malformed_nameplate_does_not_abort_other_hosts(self):
        inspect = discovery.inspect_robot

        async def host_response(session, ip):
            # Deep nesting is below the 8192-byte WebSocket limit but causes
            # json.loads to raise RecursionError rather than JSONDecodeError.
            message = "[" * 1100 + "]" * 1100 if ip.endswith(".1") else json.dumps(nameplate())
            session, _ = desk_session(message)
            return await inspect(session, ip)

        network = ipaddress.ip_network("172.16.0.0/30")
        with mock.patch.object(discovery, "inspect_robot", host_response):
            result = await discovery.scan_networks([("eno1", "172.16.0.99", network)], set())
        self.assertEqual(result["state"], "ready")
        self.assertEqual([robot["ip"] for robot in result["robots"]], ["172.16.0.2"])


class CacheTests(unittest.TestCase):
    def test_concurrent_refresh_is_single_flight_and_close_cancels(self):
        started = threading.Event()
        cancelled = threading.Event()

        async def scan(*args):
            started.set()
            try:
                await asyncio.sleep(100)
            finally:
                cancelled.set()

        cache = discovery.Discovery()
        with mock.patch.object(discovery, "wired_networks", return_value=([], set())), \
                mock.patch.object(discovery, "scan_networks", scan):
            cache.refresh()
            self.assertTrue(started.wait(2))
            thread = cache._thread
            cache.refresh()
            self.assertIs(cache._thread, thread)
            started_at = time.monotonic()
            cache.close()
            self.assertLess(time.monotonic() - started_at, 2)
            self.assertTrue(cancelled.is_set())
            self.assertFalse(thread.is_alive())
            cache.refresh()
            self.assertIs(cache._thread, thread)

    def test_cache_cooldown_and_copy_isolation(self):
        async def scan(*args):
            return {"state": "ready", "error": None,
                    "robots": [discovery.identify("172.16.0.3", nameplate())]}

        cache = discovery.Discovery()
        self.addCleanup(cache.close)
        with mock.patch.object(discovery, "wired_networks", return_value=([], set())), \
                mock.patch.object(discovery, "scan_networks", scan):
            cache.refresh()
            cache._thread.join(2)
            snapshot = cache.snapshot()
            self.assertEqual(snapshot["state"], "ready")
            snapshot["robots"].clear()
            self.assertEqual(len(cache.snapshot()["robots"]), 1)
            thread = cache._thread
            cache.refresh()
            self.assertIs(cache._thread, thread)

    def test_scan_budget_preserves_discovered_robots_and_cancels_pending_hosts(self):
        cancelled = threading.Event()
        scan = discovery.scan_networks
        networks = [("eno1", "172.16.0.99", ipaddress.ip_network("172.16.0.0/30"))]

        async def inspect(session, ip):
            if ip.endswith(".1"):
                return discovery.identify(ip, nameplate())
            try:
                await asyncio.sleep(100)
            finally:
                cancelled.set()

        async def short_scan(*args):
            return await scan(*args, timeout=0.1)

        cache = discovery.Discovery()
        self.addCleanup(cache.close)
        with mock.patch.object(discovery, "wired_networks", return_value=(networks, set())), \
                mock.patch.object(discovery, "inspect_robot", inspect), \
                mock.patch.object(discovery, "scan_networks", short_scan):
            cache.refresh()
            cache._thread.join(2)
        self.assertFalse(cache._thread.is_alive())
        self.assertTrue(cancelled.is_set())
        result = cache.snapshot()
        self.assertEqual(result["state"], "partial")
        self.assertEqual(result["error"], "TimeoutError")
        self.assertEqual([robot["ip"] for robot in result["robots"]], ["172.16.0.1"])


if __name__ == "__main__":
    unittest.main()
