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


def test_https_pairing_management_requires_pin(device):
    import aiohttp
    plugin, config = device
    plugin.dispatch("config", {"instance_id": "test-vr"})
    origin = config["public_wss_url"].replace("wss://", "https://").split("/ws/")[0]
    context = ssl.create_default_context(cafile=config["tls_cert_file"])

    async def run():
        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=context),
                                        cookie_jar=aiohttp.CookieJar(unsafe=True)) as session:
            async with session.get(origin + "/onboarding") as response:
                assert response.status == 200 and 'PIN' in await response.text()
            async with session.get(origin + "/onboarding/package") as response:
                assert response.status == 200
            async with session.post(origin + "/manage/login", json={"pin": "0412"}) as response:
                assert response.status == 503
            configured = plugin.dispatch("config", {"management_pin": "0412"})
            assert configured["management_pin_configured"] and "0412" not in json.dumps(configured)
            for operation in ("status", "open", "invite", "revoke_invitation", "approve", "reject", "revoke_headset"):
                async with session.post(origin + "/manage/" + operation, json={}) as response:
                    assert response.status == 401, operation
            assert not plugin.instances["test-vr"]["server"].enrollment.status()["window_open"]
            async with session.post(origin + "/manage/login", json={"pin": "9999"}) as response:
                assert response.status == 403
            async with session.post(origin + "/manage/login", json={"pin": "0412"}) as response:
                assert response.status == 200
                cookie = response.headers['Set-Cookie']
                assert all(flag in cookie for flag in ('Secure', 'HttpOnly', 'SameSite=Strict', 'Path=/'))
                assert '0412' not in cookie and '0412' not in await response.text()
            async with session.post(origin + "/manage/invite", json={}, headers={"Origin": "https://other.invalid"}) as response:
                assert response.status == 403
            async with session.post(origin + "/manage/invite", data="{}") as response:
                assert response.status == 415
            async with session.post(origin + "/manage/open", json={}) as response:
                assert response.status == 200 and (await response.json())["pairing"]["window_open"]
            async with session.post(origin + "/manage/invite", json={}) as response:
                assert response.status == 200
                invitation = await response.json()
                assert invitation['deep_link'].startswith('motus-teleop://connect#')
            plugin.dispatch("config", {"management_pin": ""})
            plugin.dispatch("config", {"management_pin": "0412"})
            async with session.post(origin + "/manage/status", json={}) as response:
                assert response.status == 200
                assert invitation['token'] not in json.dumps(await response.json())
            plugin.dispatch("config", {"management_pin": "1234"})
            async with session.post(origin + "/manage/status", json={}) as response:
                assert response.status == 401
            async with session.post(origin + "/pairing/invite", json={
                k: invitation[k] for k in ("invitation_id", "token", "device_id")
            } | {"device_name": "test"}) as response:
                assert response.status == 403
            async with session.post(origin + "/manage/login", json={"pin": "1234"}) as response:
                assert response.status == 200
            async with session.post(origin + "/manage/logout", json={}) as response:
                assert response.status == 200
            async with session.post(origin + "/manage/status", json={}) as response:
                assert response.status == 401
    asyncio.run(run())
    info = plugin.dispatch("info", {})
    assert info['pairing_password_required'] is False
    assert info['management_pin_configured'] is True
    assert '0412' not in json.dumps(info)


def test_single_instance_upgrade_preserves_pairing_identity(device):
    plugin, config = device
    path = Path(config["state_dir"]) / "old-card.config.json"
    path.write_text(json.dumps(plugin._defaults()))
    plugin.dispatch("config", {"management_pin": "0412"})
    assert list(plugin.instances) == ["old-card"]
    plugin.dispatch("config", {"instance_id": "new-card", "management_pin": ""})
    assert list(plugin.instances) == ["old-card"]
    assert not (path.parent / "new-card.config.json").exists()
    assert plugin.dispatch("info", {})["instance_id"] == "old-card"
    assert plugin.get_tool()["multiInstance"] is False


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


@pytest.mark.parametrize("legacy_without_config", [False, True])
def test_pin_changes_and_card_migration_preserve_headset_reconnect(device, legacy_without_config):
    from ext_vr.capture import CAPTURE_PROTOCOL, RTC_FRAME_PROTOCOL
    plugin, config = device
    plugin.dispatch("config", {"instance_id": "legacy-card", "management_pin": "0412"})
    manager = plugin.instances["legacy-card"]["manager"]

    async def pair():
        invitation = await manager.create_pairing()
        connection, ack = await manager.connect({
            "type": "pair", "pairing_id": invitation["pairing_id"],
            "pairing_code": invitation["pairing_code"],
            "capture_protocol": CAPTURE_PROTOCOL, "frame_protocol": RTC_FRAME_PROTOCOL,
            "client_kind": "native_openxr", "app_version": "0.4.4-pico-input",
        })
        await manager.disconnect(connection)
        return ack

    ack = asyncio.run_coroutine_threadsafe(pair(), plugin.loop).result(3)
    pairing_file = Path(config["state_dir"]) / "legacy-card.json"
    before = pairing_file.read_bytes()
    plugin.dispatch("config", {"management_pin": "5678"})
    assert pairing_file.read_bytes() == before
    plugin.close()
    if legacy_without_config:
        (Path(config["state_dir"]) / "legacy-card.config.json").unlink()
    restored = ExtVrPlugin(config, "pico")
    try:
        restored.dispatch("config", {"management_pin": ""})
        assert list(restored.instances) == ["legacy-card"]
        assert restored.instances["legacy-card"]["server"].management_pin.configured

        async def reconnect():
            capture = restored.instances["legacy-card"]["manager"]
            conn, reply = await capture.connect({
                "type": "credential", "capture_id": ack["capture_id"],
                "capture_credential": ack["capture_credential"],
                "capture_protocol": CAPTURE_PROTOCOL, "frame_protocol": RTC_FRAME_PROTOCOL,
                "client_kind": "native_openxr", "app_version": "0.4.4-pico-input",
            })
            assert reply["type"] == "connected"
            await capture.disconnect(conn)
        asyncio.run_coroutine_threadsafe(reconnect(), restored.loop).result(3)
    finally:
        restored.close()
