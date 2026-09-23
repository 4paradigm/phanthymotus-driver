"""Real localhost TLS/MCP pairing and configuration; no robot or headset IO."""

import asyncio
import json
import socket
import ssl
import threading
import urllib.request
from pathlib import Path
from http.server import ThreadingHTTPServer
import pytest
from common.ext_vr.plugin import ExtVrPlugin
from test_pico_device import load_driver_module


@pytest.fixture
def device(tmp_path):
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = load_driver_module("identity").prepare_config(
        {
            "public_host": "127.0.0.1",
            "state_dir": str(tmp_path),
            "port": port,
            "bind_host": "127.0.0.1",
            "discovery_enabled": False,
        }
    )

    class Sink:
        def publish(self, value):
            pass

        def close(self):
            pass

    plugin = ExtVrPlugin(config, "pico", transport_factory=lambda *args: Sink())
    yield plugin, config
    plugin.close()


def test_real_mcp_configuration_and_null_error_success(device):
    plugin, config = device
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0), load_driver_module("main").make_handler(plugin)
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def call(action, **values):
        body = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": "teleop_device",
                "arguments": {"action": action, "instance_id": "test-vr", **values},
            },
        }
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
        with opener.open(request, timeout=3) as response:
            return json.load(response)

    try:
        rejected = call("start")
        assert "confirm_driver_installation" in rejected["error"]["message"]
        configured = call(
            "config",
            driver_installed=True,
            pairing_admin_password="long fixture password",
        )
        assert not configured["result"]["isError"]
        assert "long fixture password" not in json.dumps(configured)
        started = call("start")
        assert not started["result"]["isError"]
        assert json.loads(started["result"]["content"][0]["text"])["error"] is None
        assert not call("info")["result"]["isError"]
        assert not call("stop")["result"]["isError"]
        assert not call("info")["result"]["isError"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)


def test_https_pairing_management_requires_own_password_and_csrf(device):
    import aiohttp

    plugin, config = device
    plugin.dispatch(
        "config",
        {
            "instance_id": "test-vr",
            "driver_installed": True,
            "pairing_admin_password": "long fixture password",
        },
    )
    origin = config["public_wss_url"].replace("wss://", "https://").split("/ws/")[0]
    ssl_context = ssl.create_default_context(cafile=config["tls_cert_file"])

    async def run():
        async with aiohttp.ClientSession(
            cookie_jar=aiohttp.CookieJar(unsafe=True),
            connector=aiohttp.TCPConnector(ssl=ssl_context),
        ) as session:
            async with session.get(origin + "/onboarding") as response:
                assert (
                    response.status == 200 and "配对管理密码" in await response.text()
                )
            async with session.post(
                origin + "/manage/approve", json={}, headers={"Origin": origin}
            ) as response:
                assert response.status == 403
            async with session.post(
                origin + "/manage/login",
                json={"password": "long fixture password"},
                headers={"Origin": origin},
            ) as response:
                assert response.status == 200
                csrf = (await response.json())["csrf"]
                cookie = response.cookies["motus_pico_admin"]
                assert cookie["secure"] and cookie["httponly"]
            headers = {"Origin": origin, "X-Pico-CSRF": csrf}
            async with session.post(
                origin + "/manage/invite", json={}, headers=headers
            ) as response:
                assert response.status == 200
                invitation = await response.json()
                assert invitation["deep_link"].startswith("motus-teleop://connect#")
            async with session.post(
                origin + "/manage/status", json={}, headers=headers
            ) as response:
                status = await response.json()
                assert invitation["token"] not in json.dumps(status)
                assert "digest" not in json.dumps(status)
            async with session.post(
                origin + "/manage/revoke_invitation",
                json={},
                headers={"Origin": "https://other.invalid", "X-Pico-CSRF": csrf},
            ) as response:
                assert response.status == 403
            async with session.post(
                origin + "/manage/revoke_invitation", json={}, headers=headers
            ) as response:
                assert response.status == 200

    asyncio.run(run())
    info = plugin.dispatch("info", {"instance_id": "test-vr"})
    assert info["pairing_password_set"] and "digest" not in json.dumps(info)
    assert "long fixture password" not in json.dumps(info)


def test_concurrent_core_save_and_start_reapply_are_serialized(device):
    from concurrent.futures import ThreadPoolExecutor

    plugin, _ = device
    values = {
        "instance_id": "test-vr",
        "driver_installed": True,
        "pairing_admin_password": "long fixture password",
    }
    # Models Core's fire-and-forget save push overlapping auto-config on start.
    with ThreadPoolExecutor(max_workers=4) as pool:
        calls = [pool.submit(plugin.dispatch, "config", values) for _ in range(4)]
        assert all(call.result()["confirmed"] for call in calls)
    assert plugin.dispatch("start", {"instance_id": "test-vr"})["state"] == "collecting"
    assert plugin.dispatch("stop", {"instance_id": "test-vr"})["state"] == "idle"
