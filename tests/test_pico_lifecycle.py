import sys
from pathlib import Path
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
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pico/4ultra'))
from ext_vr.plugin import ExtVrPlugin
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
        started_without_checkbox = call("start")
        assert not started_without_checkbox["result"]["isError"]
        assert not call("stop")["result"]["isError"]
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


def test_https_pairing_management_without_password(device):
    import aiohttp
    plugin, config = device
    plugin.dispatch("config", {"instance_id": "test-vr"})
    origin = config["public_wss_url"].replace("wss://", "https://").split("/ws/")[0]
    context = ssl.create_default_context(cafile=config["tls_cert_file"])
    async def run():
        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=context)) as session:
            async with session.get(origin + "/onboarding") as response:
                html = await response.text()
                assert response.status == 200 and '无需密码' in html and 'id="login"' not in html
            async with session.post(origin + "/manage/open", json={}) as response:
                assert response.status == 200
                assert (await response.json())["pairing"]["window_open"]
            async with session.post(origin + "/manage/invite", json={}) as response:
                assert response.status == 200
                invitation = await response.json()
                assert invitation['deep_link'].startswith('motus-teleop://connect#')
            async with session.post(origin + "/manage/status", json={}) as response:
                assert response.status == 200
                assert invitation['token'] not in json.dumps(await response.json())
            async with session.post(origin + "/manage/revoke_invitation", json={}) as response:
                assert response.status == 200
    asyncio.run(run())
    info = plugin.dispatch("info", {"instance_id": "test-vr"})
    assert info['pairing_password_required'] is False


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
    assert plugin.dispatch("start", {"instance_id": "test-vr"})["state"] == "running"
    assert plugin.dispatch("stop", {"instance_id": "test-vr"})["state"] == "idle"


def test_legacy_settings_do_not_override_device_presets(device):
    plugin, _ = device
    result = plugin.dispatch("config", {"instance_id": "test-vr", "driver_installed": False,
        "display_name": "old name", "input_filter_ms": 123, "installation_url": "https://old-host/"})
    assert result["config"] == plugin._defaults()
    assert plugin.dispatch("start", {"instance_id": "test-vr"})["state"] == "running"
    plugin.dispatch("stop", {"instance_id": "test-vr"})


def test_legacy_password_is_ignored_and_not_persisted(device):
    plugin, config = device
    plugin.dispatch("config", {"instance_id": "test-vr", "pairing_admin_password": "short"})
    assert not (Path(config['state_dir']) / 'pairing-admin.json').exists()
    assert plugin.dispatch("info", {"instance_id": "test-vr"})['pairing_password_required'] is False
