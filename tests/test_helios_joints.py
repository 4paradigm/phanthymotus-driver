"""State mapping, MCP contract and real local WebSocket failure/recovery tests."""

import asyncio
import copy
import json
import threading
import time
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

import pytest
from aiohttp import web

from common.vendor_runtime import DriverBundle, make_handler
from rokae.helios.device import HeliosJoints
from rokae.helios.feedback import FeedbackClient, GROUPS, JOINT_NAMES, parse_state
from test_helios_model import model_server


def frame():
    return {"jsonrpc": "2.0", "method": "joint/state", "params": {
        "pos": {g: [float(i + 1) / 10 for i in range(len(names))] for g, names in GROUPS.items()},
        "vel": {g: [0.02] * len(names) for g, names in GROUPS.items()},
    }}


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not met before deadline")


def test_mapping_units_and_optional_velocity():
    result = parse_state(frame())
    assert [j["name"] for j in result] == list(JOINT_NAMES)
    assert result[14] == {"idx": 14, "name": "body_joint_1", "q": 0.1, "dq": 0.02}
    assert result[-1]["q"] == 0.2
    msg = frame()
    del msg["params"]["vel"]
    assert all("dq" not in j for j in parse_state(msg))
    assert parse_state({"method": "heartbeat"}) is None


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), "0.1", None])
def test_invalid_position_rejected(value):
    msg = frame()
    msg["params"]["pos"]["head"][0] = value
    with pytest.raises(ValueError):
        parse_state(msg)


def test_partial_frame_and_wrong_velocity_length_rejected():
    for field in ("pos", "vel"):
        msg = frame()
        msg["params"][field]["left_arm"].pop()
        with pytest.raises(ValueError):
            parse_state(msg)
    msg = frame()
    del msg["params"]["pos"]["waist"]
    with pytest.raises(ValueError):
        parse_state(msg)


@contextmanager
def vendor_server():
    state = {"mode": "valid", "connections": 0, "requests": []}
    ready = threading.Event()
    loop = asyncio.new_event_loop()

    async def handle(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        state["connections"] += 1
        state["requests"].append(await ws.receive_json())
        try:
            while not ws.closed:
                mode = state["mode"]
                if mode == "close":
                    await ws.close()
                    break
                if mode != "silent":
                    msg = frame()
                    if mode == "invalid":
                        msg["params"]["pos"]["head"] = []
                    await ws.send_json(msg)
                try:
                    response = await ws.receive(timeout=0.03)
                    if response.type == web.WSMsgType.TEXT:
                        state["requests"].append(json.loads(response.data))
                except asyncio.TimeoutError:
                    pass
        except (ConnectionError, RuntimeError):
            pass
        return ws

    async def setup():
        app = web.Application()
        app.router.add_get('/ws', handle)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        state["url"] = f"ws://127.0.0.1:{site._server.sockets[0].getsockname()[1]}/ws"
        state["runner"] = runner
        ready.set()

    def run():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(setup())
        loop.run_forever()
        loop.close()

    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    assert ready.wait(3)
    try:
        yield state
    finally:
        asyncio.run_coroutine_threadsafe(state["runner"].cleanup(), loop).result(timeout=5)
        loop.call_soon_threadsafe(loop.stop)
        worker.join(timeout=2)


def test_disconnect_stale_invalid_recovery_and_stop():
    with vendor_server() as server:
        client = FeedbackClient(server["url"], stale_timeout_s=0.2, reconnect_delay_s=0.05)
        try:
            client.start()
            wait_for(lambda: client.snapshot()["available"])
            snap = client.snapshot()
            assert len(snap["joints"]) == 20
            assert snap["source_timestamp"] is None
            snap["joints"][0]["q"] = 999
            assert client.snapshot()["joints"][0]["q"] == 0.1
            server["mode"] = "invalid"
            wait_for(lambda: client.snapshot()["state"] == "invalid")
            assert client.snapshot()["joints"] == []
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            server["mode"] = "silent"
            wait_for(lambda: not client.snapshot()["available"])
            assert client.snapshot()["joints"] == []
            wait_for(lambda: server["connections"] >= 2)
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            server["mode"] = "close"
            wait_for(lambda: not client.snapshot()["available"])
            assert client.snapshot()["joints"] == []
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            client.stop()
            assert client.snapshot()["state"] == "idle"
            assert client.snapshot()["joints"] == []
            client.start()
            wait_for(lambda: client.snapshot()["available"])
            assert all(r == {"jsonrpc": "2.0", "method": "subscribe", "params": ["joint/state"]}
                       for r in server["requests"])
        finally:
            client.stop()


def test_stop_during_handshake_is_cancellable():
    import socket
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        client = FeedbackClient(f"ws://127.0.0.1:{listener.getsockname()[1]}/ws")
        try:
            client.start()
            connection, _ = listener.accept()
            with connection:
                started = time.monotonic()
                client.stop()
                assert time.monotonic() - started < 2
                assert client.snapshot()["state"] == "idle"
        finally:
            client.stop()


def test_model_and_mcp_resource_not_double_wrapped():
    with model_server() as source:
        _check_model_and_mcp_resource(source)


def _check_model_and_mcp_resource(source):
    plugin = HeliosJoints({"state_url": source['state_url']})
    bundle = DriverBundle([plugin])
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "helios", "helios"))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        def rpc(method, params):
            req = Request(f"http://127.0.0.1:{server.server_port}/mcp", data=json.dumps({
                "jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode(),
                headers={"Content-Type": "application/json"})
            with urlopen(req, timeout=2) as response:
                return json.load(response)["result"]
        tools = rpc("tools/list", {})["tools"]
        assert plugin.PREFIX == tools[0]['name']
        assert {t["name"]: t["type"] for t in tools} == {"joints": "sensor", "model": "resource"}
        assert tools[0]["configSchema"]["properties"]["state_url"]["x-sensitive"] is True
        resource = rpc("tools/call", {"name": "model", "arguments": {}})
        model = json.loads(resource["content"][0]["text"])
        assert "visual_packages" not in model  # Standard URDF skeleton, no STL downloads.
        root = ET.fromstring(model["urdf"])
        assert len(root.findall("link")) == 21  # Synthetic fixture, not manufacturer geometry.
        assert {j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"} == set(JOINT_NAMES)
        skeleton, status = plugin.payloads()
        assert skeleton["joints"] == [] and not skeleton["available"]
        assert status["state"] == "idle" and status["angles_rad"] == {}
        rpc("tools/call", {"name": "model", "arguments": {}})
        assert source['requests'] == ['/api/robot_3d_model', '/model.urdf']  # Cached per source.
    finally:
        server.shutdown()
        server.server_close()
        worker.join()
