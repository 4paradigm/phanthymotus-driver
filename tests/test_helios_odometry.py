"""Relative pose, standard contract and a genuinely passive WebSocket client."""

import asyncio
import json
import math
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from aiohttp import web

from common.odom import parse_interface
from common.vendor_runtime import DriverBundle
from rokae.helios.battery import HeliosBattery
from rokae.helios.chassis_pose_pb2 import UiDatagram
from rokae.helios.odometry import HeliosOdometry, PoseFeedbackClient, parse_pose_frame
from test_helios_joints import wait_for


CONFIG = {"battery_state_url": "ws://127.0.0.1/ws/AGVInfo"}


def frame(x=0, y=0, yaw=0, stamp=1000, speed=None):
    state = {"pose": {"x": x, "y": y, "theta": yaw}, "timeStamp": stamp}
    if speed is not None:
        state["speed"] = dict(zip(("vx", "vy", "w"), speed))
    return UiDatagram(current_state=state).SerializeToString()


def observe(plugin, x, y, yaw, stamp=1000, speed=None):
    with plugin.client._lock:
        plugin.client._running = plugin.client._connected = True
        plugin.client._received = time.monotonic()
        plugin.client._received_wall = 1234567890000
        plugin.client._data = parse_pose_frame(frame(x, y, yaw, stamp, speed))


def test_wire_projection_zero_presence_and_unknown_fields():
    assert parse_pose_frame(frame() + b"\x78\x01") == {
        "x": 0, "y": 0, "yaw": 0, "source_stamp_ms": 1000, "speed": None}
    assert parse_pose_frame(b"\x42\x00") is None  # battery-only datagram
    assert parse_pose_frame(frame(stamp=0))["source_stamp_ms"] is None
    for raw in [b"\xff", UiDatagram(current_state={"timeStamp": 2}).SerializeToString()]:
        with pytest.raises(ValueError):
            parse_pose_frame(raw)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("axis", ["x", "y", "yaw"])
def test_non_finite_pose_rejected(value, axis):
    with pytest.raises(ValueError):
        parse_pose_frame(frame(**{axis: value}))


def test_source_timestamp_replay_and_new_connection():
    client = PoseFeedbackClient(CONFIG["battery_state_url"])
    assert client._parse(frame(stamp=10))
    assert client._parse(frame(stamp=10)) is None
    assert client._parse(frame(stamp=9)) is None
    assert client._parse(frame(stamp=11))
    client._on_connect()
    assert client._parse(frame(stamp=1))


def test_relative_origin_axes_and_heading_wrap():
    plugin = HeliosOdometry({**CONFIG, "odometry_position_unit": "m"})
    observe(plugin, 10, 20, math.pi / 2)
    assert {k: plugin.payload()[k] for k in ("x", "y", "yaw")} == {"x": 0, "y": 0, "yaw": 0}
    observe(plugin, 10, 22, math.pi)
    value = plugin.payload()
    assert value["x"] == pytest.approx(2)
    assert value["y"] == pytest.approx(0, abs=1e-12)
    assert value["yaw"] == pytest.approx(math.pi / 2)
    plugin._origin = (0, 0, math.radians(179))
    observe(plugin, 0, 0, math.radians(-179))
    assert plugin.payload()["yaw"] == pytest.approx(math.radians(2))


def test_millimetres_convert_once_and_units_are_not_guessed():
    plugin = HeliosOdometry({**CONFIG, "odometry_position_unit": "mm"})
    observe(plugin, 1000, 2000, 0)
    plugin.payload()
    observe(plugin, 1250, 1500, .4)
    value = plugin.payload()
    assert value["x"] == .25 and value["y"] == -.5
    standard = plugin.odom_payload(value)
    assert standard["schema"] == "motus.odom/1"
    assert standard["frame"] == "body" and standard["twist"] == [None] * 6
    assert standard["pose"]["position"] == {"x": .25, "y": -.5, "z": 0}
    assert standard["pose"]["orientation"]["z"] == pytest.approx(math.sin(.2))
    assert standard["pose"]["stamp_ms"] == value["stamp_ms"]
    interface = parse_interface(plugin.dispatch("info", {})["odom_interface"])
    assert interface.provides == ("vx", "vy", "wz") and interface.pose_drift == "unbounded"
    unverified = HeliosOdometry(CONFIG)
    observe(unverified, 1, 2, 0)
    raw = unverified.payload()
    assert raw["position_unit"] == "source_unverified"
    assert unverified.odom_payload(raw)["pose"] is None


def test_navi_receives_body_speed_without_relabeling_the_relative_pose():
    plugin = HeliosOdometry({**CONFIG, "odometry_position_unit": "m"})
    display, standard = [], []
    plugin._publishers = [SimpleNamespace(publish=display.append),
                          SimpleNamespace(publish=standard.append)]
    plugin._message_type = SimpleNamespace
    observe(plugin, 10, 20, math.pi / 2, speed=(0.1, -0.02, 0.3))
    plugin._publish()
    value = json.loads(display[-1].data)
    sample = json.loads(standard[-1].data)
    assert {"speed", "twist"}.isdisjoint(value)
    assert sample["frame"] == "body"
    assert sample["twist"] == [0.1, -0.02, None, None, None, 0.3]
    assert sample["pose"]["frame_id"].endswith("/relative_start")
    assert sample["stamp_ms"] == value["stamp_ms"] == 1234567890000
    assert sample["pose"]["stamp_ms"] == sample["stamp_ms"]
    with plugin.client._lock:
        plugin.client._received -= 10
    plugin._publish()
    expired = json.loads(standard[-1].data)
    assert expired["pose"] is None and expired["twist"] == [None] * 6
    assert expired["stamp_ms"] == sample["stamp_ms"]


def test_speed_presence_preserves_measured_zero_and_drops_bad_axes():
    assert parse_pose_frame(frame(speed=(0, 0, 0)))["speed"] == [0, 0, 0]
    assert parse_pose_frame(frame(speed=(float("nan"), 0.02, float("inf"))))["speed"] == [
        None, 0.02, None]
    assert parse_pose_frame(frame(stamp=0, speed=(1, 0, 1)))["speed"] is None
    # UiDatagram field 5 is control_speed. Its payload is never feedback.
    assert parse_pose_frame(b"\x2a\x00") is None


def test_absence_expiry_stop_and_restart_never_invent_zero_pose():
    plugin = HeliosOdometry(CONFIG)
    assert plugin.payload()["x"] is None
    observe(plugin, 10, 20, .5)
    plugin.payload()
    with plugin.client._lock:
        plugin.client._received -= 10
    assert plugin.payload()["x"] is None
    assert plugin.odom_payload(plugin.payload())["pose"] is None
    assert plugin.dispatch("stop", {})["state"] == "idle"
    observe(plugin, 100, 200, 1)
    assert plugin.payload()["x"] == 0 and plugin.payload()["yaw"] == 0


def test_unit_configuration_is_atomic_and_restarts_reference(monkeypatch):
    plugin = HeliosOdometry({"odometry_state_url": CONFIG["battery_state_url"]})
    monkeypatch.setattr(plugin, "start", lambda: None)
    observe(plugin, 10, 20, .5)
    plugin.payload()
    old_client = plugin.client
    with pytest.raises(ValueError):
        plugin.dispatch("config", {"odometry_position_unit": "cm"})
    assert plugin.client is old_client and plugin.payload()["available"]
    plugin.dispatch("config", {"odometry_position_unit": "mm"})
    assert not old_client.snapshot()["available"]
    assert plugin.payload()["x"] is None
    observe(plugin, 1000, 2000, 0)
    assert plugin.payload()["x"] == 0
    observe(plugin, 1250, 2000, .2)
    assert plugin.payload()["x"] == .25
    assert plugin.odom_payload(plugin.payload())["pose"] is not None
    plugin.dispatch("config", {"odometry_position_unit": "unverified"})
    observe(plugin, 1, 2, 0)
    assert plugin.odom_payload(plugin.payload())["pose"] is None


def test_schema_and_independent_bundle_lifecycle():
    plugin = HeliosOdometry(CONFIG, "test_robot")
    battery = HeliosBattery(CONFIG)
    bundle = DriverBundle([battery, plugin])
    tool = next(t for t in bundle.get_all_tools() if t["name"] == "odometry")
    assert tool["type"] == "sensor" and not tool["multiInstance"]
    assert tool["configSchema"]["properties"]["odometry_state_url"]["x-sensitive"]
    assert tool["topic_out"] == [
        {"topic": "/test_robot/state/relative_pose", "format": "data/json"},
        {"topic": "/test_robot/state/odom", "format": "state/odom"},
    ]
    observe(plugin, 1, 2, .3)
    assert bundle.dispatch("odometry", {})["data"]["available"]
    bundle.dispatch("battery", {"action": "stop"})
    assert plugin.payload()["available"]
    with pytest.raises(ValueError):
        plugin.dispatch("move", {})
    for config in [{"odometry_publish_hz": 0}, {"odometry_publish_hz": 31},
                   {"odometry_stale_timeout_s": -1}, {"odometry_position_unit": "degrees"}]:
        with pytest.raises(ValueError):
            HeliosOdometry({**CONFIG, **config})


@contextmanager
def pose_server():
    state = {"mode": "valid", "connections": 0, "requests": [], "stamp": 100}
    ready = threading.Event()
    loop = asyncio.new_event_loop()

    async def handle(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        state["connections"] += 1
        try:
            while not ws.closed:
                await ws.send_bytes(b"\x42\x00")
                if state["mode"] == "valid":
                    state["stamp"] += 1
                    await ws.send_bytes(frame(1, 2, .3, state["stamp"]))
                elif state["mode"] == "close":
                    await ws.close()
                    break
                try:
                    msg = await ws.receive(timeout=.02)
                    if msg.type in (web.WSMsgType.TEXT, web.WSMsgType.BINARY):
                        state["requests"].append(msg.data)
                except asyncio.TimeoutError:
                    pass
        except (ConnectionError, RuntimeError):
            pass
        return ws

    async def setup():
        app = web.Application()
        app.router.add_get('/ws/AGVInfo', handle)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        state["url"] = f"ws://127.0.0.1:{site._server.sockets[0].getsockname()[1]}/ws/AGVInfo"
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


def test_live_socket_is_passive_expires_on_unrelated_data_and_recovers():
    with pose_server() as server:
        client = PoseFeedbackClient(server["url"], stale_timeout_s=.15, reconnect_delay_s=.05)
        plugin = HeliosOdometry(CONFIG, client=client)
        try:
            plugin.dispatch("start", {})
            wait_for(lambda: plugin.payload()["available"])
            assert plugin.payload()["x"] == 0
            server["mode"] = "unrelated"
            wait_for(lambda: not plugin.payload()["available"])
            assert plugin.payload()["x"] is None
            wait_for(lambda: server["connections"] >= 2)
            server["mode"] = "valid"
            wait_for(lambda: plugin.payload()["available"])
            server["mode"] = "close"
            wait_for(lambda: not plugin.payload()["available"])
            server["mode"] = "valid"
            wait_for(lambda: plugin.payload()["available"])
            plugin.dispatch("stop", {})
            assert plugin.payload()["state"] == "idle" and plugin.payload()["x"] is None
            plugin.dispatch("start", {})
            wait_for(lambda: plugin.payload()["available"])
            assert plugin.payload()["x"] == 0
            assert server["requests"] == []
        finally:
            plugin.stop()
