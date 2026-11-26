"""Native pose, standard contract and passive WebSocket speed feedback."""

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
from rokae.helios.odometry import HeliosOdometry, SpeedFeedbackClient, parse_speed_frame
from rokae.helios.sdk_odometry import SdkOdometryClient
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
        plugin.client._data = parse_speed_frame(frame(x, y, yaw, stamp, speed))


def test_wire_projection_speed_does_not_depend_on_map_pose():
    assert parse_speed_frame(frame() + b"\x78\x01") == {"source_stamp_ms": 1000, "speed": None}
    assert parse_speed_frame(b"\x42\x00") is None  # battery-only datagram
    assert parse_speed_frame(frame(stamp=0))["source_stamp_ms"] is None
    raw = UiDatagram(current_state={"timeStamp": 2, "speed": {"vx": 0.2}}).SerializeToString()
    assert parse_speed_frame(raw)["speed"] == [0.2, 0, 0]
    with pytest.raises(ValueError):
        parse_speed_frame(b"\xff")


def test_source_timestamp_replay_and_new_connection():
    client = SpeedFeedbackClient(CONFIG["battery_state_url"])
    assert client._parse(frame(stamp=10))
    assert client._parse(frame(stamp=10)) is None
    assert client._parse(frame(stamp=9)) is None
    assert client._parse(frame(stamp=11))
    client._on_connect()
    assert client._parse(frame(stamp=1))


def native_pose(x=10, y=-4, yaw=0.6):
    pose = {"x": x, "y": y, "yaw": yaw, "stamp_ms": 1234567890005,
            "source_stamp_ns": 1234567890002000000}
    return SimpleNamespace(installed=True, snapshot=lambda: dict(pose),
                           start=lambda: None, stop=lambda: None)


def test_both_ports_preserve_vendor_pose_and_independent_velocity():
    card = HeliosOdometry(CONFIG, native=native_pose())
    display, standard = [], []
    card._publishers = [SimpleNamespace(publish=display.append),
                        SimpleNamespace(publish=standard.append)]
    card._message_type = SimpleNamespace
    observe(card, 50, 60, math.pi / 2, speed=(0.1, -0.02, 0.3))
    card._publish()
    value = json.loads(display[-1].data)
    sample = json.loads(standard[-1].data)
    assert value["position"] == [10, -4, None] and value["rpy"] == [None, None, 0.6]
    assert value["available"]
    assert sample["schema"] == "motus.odom/1" and sample["frame"] == "body"
    assert sample["twist"] == [0.1, -0.02, None, None, None, 0.3]
    assert sample["pose"]["position"] == {"x": 10, "y": -4, "z": 0.0}
    assert sample["pose"]["frame_id"].endswith("/odom")
    assert sample["pose"]["orientation"]["z"] == pytest.approx(math.sin(.3))
    assert sample["stamp_ms"] == 1234567890000
    assert sample["pose"]["stamp_ms"] == 1234567890005
    assert sample["vendor"]["pose_source_stamp_ns"] == 1234567890002000000
    interface = parse_interface(card.dispatch("info", {})["odom_interface"])
    assert interface.provides == ("vx", "vy", "wz") and interface.pose_drift == "unbounded"
    with card.client._lock:
        card.client._received -= 10
    expired = card.odom_payload(card.payload())
    assert expired["pose"] == sample["pose"] and expired["twist"] == [None] * 6


def test_missing_sdk_does_not_publish_map_localization_as_odom():
    native = SimpleNamespace(installed=False, snapshot=lambda: None)
    card = HeliosOdometry(CONFIG, native=native)
    observe(card, 100, 200, 0.6, speed=(0, 0, 0))
    value = card.payload()
    assert value["position"] is None and value["pose_state"] == "sdk_unavailable"
    sample = card.odom_payload(value)
    assert sample["pose"] is None and sample["twist"] == [0, 0, None, None, None, 0]
    assert parse_interface(card.dispatch("info", {})["odom_interface"]).pose_drift == "none"


def test_speed_presence_preserves_measured_zero_and_drops_bad_axes():
    assert parse_speed_frame(frame(speed=(0, 0, 0)))["speed"] == [0, 0, 0]
    assert parse_speed_frame(frame(speed=(float("nan"), 0.02, float("inf"))))["speed"] == [
        None, 0.02, None]
    assert parse_speed_frame(frame(stamp=0, speed=(1, 0, 1)))["speed"] is None
    # UiDatagram field 5 is control_speed. Its payload is never feedback.
    assert parse_speed_frame(b"\x2a\x00") is None


def test_sdk_reader_ignores_duplicate_and_malformed_observations():
    client = SdkOdometryClient(CONFIG, 1.0)

    async def read():
        pipe = asyncio.StreamReader()
        pose = {"x": 1.0, "y": -2.0, "yaw": 0.5, "source_stamp_ns": 200}
        for data in (pose, {**pose, "x": 9}, {**pose, "x": float("inf"), "source_stamp_ns": 201}):
            pipe.feed_data(b"MOTUS_ODOM " + json.dumps(data).encode() + b"\n")
        pipe.feed_eof()
        await client._read(SimpleNamespace(stdout=pipe))

    asyncio.run(read())
    assert client.snapshot()["x"] == 1.0
    assert client.snapshot()["source_stamp_ns"] == 200
    client.stop()
    assert client.snapshot() is None


def test_schema_and_independent_bundle_lifecycle():
    plugin = HeliosOdometry(CONFIG, "test_robot", native=native_pose())
    battery = HeliosBattery(CONFIG)
    bundle = DriverBundle([battery, plugin])
    tool = next(t for t in bundle.get_all_tools() if t["name"] == "loco_state")
    assert tool["type"] == "sensor" and not tool["multiInstance"]
    assert tool["configSchema"]["properties"]["odometry_state_url"]["x-sensitive"]
    assert tool["topic_out"] == [
        {"topic": "/test_robot/loco/state", "format": "data/json"},
        {"topic": "/test_robot/state/odom", "format": "state/odom"},
    ]
    observe(plugin, 1, 2, .3, speed=(0, 0, 0))
    assert bundle.dispatch("loco_state", {})["data"]["available"]
    assert bundle.dispatch("loco_state", {"action": "info"})["topic_out"] == tool["topic_out"]
    assert bundle.dispatch("odometry", {}) is None
    bundle.dispatch("battery", {"action": "stop"})
    assert plugin.payload()["available"]
    with pytest.raises(ValueError):
        plugin.dispatch("move", {})
    for config in [{"odometry_publish_hz": 0}, {"odometry_publish_hz": 31},
                   {"odometry_stale_timeout_s": -1}]:
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
                    await ws.send_bytes(frame(1, 2, .3, state["stamp"], speed=(0, 0, 0)))
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
        client = SpeedFeedbackClient(server["url"], stale_timeout_s=.15, reconnect_delay_s=.05)
        plugin = HeliosOdometry(CONFIG, client=client, native=native_pose())
        try:
            plugin.dispatch("start", {})
            wait_for(lambda: plugin.payload()["available"])
            assert plugin.payload()["position"] == [10, -4, None]
            server["mode"] = "unrelated"
            wait_for(lambda: not plugin.payload()["available"])
            assert plugin.payload()["velocity"] == [None, None, None]
            wait_for(lambda: server["connections"] >= 2)
            server["mode"] = "valid"
            wait_for(lambda: plugin.payload()["available"])
            server["mode"] = "close"
            wait_for(lambda: not plugin.payload()["available"])
            server["mode"] = "valid"
            wait_for(lambda: plugin.payload()["available"])
            assert plugin.dispatch("stop", {})["state"] == "idle"
            assert plugin.payload()["available"]
            plugin.dispatch("start", {})
            wait_for(lambda: plugin.payload()["available"])
            assert plugin.payload()["position"] == [10, -4, None]
            assert server["requests"] == []
        finally:
            plugin.stop()
