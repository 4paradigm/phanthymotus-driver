"""Dual-pack wire parsing, read-only stream recovery, and independent card lifecycle."""

import asyncio
import json
import math
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import pytest
from aiohttp import web

from common.vendor_runtime import DriverBundle
from rokae.helios.battery import BatteryFeedbackClient, HeliosBattery, parse_battery_frame
from rokae.helios.chassis_battery_pb2 import UiDatagram
from rokae.helios.device import HeliosJoints
from test_helios_joints import wait_for


def pack(pack_id="1", soc=98):
    return {"id": pack_id, "soc": soc, "voltage": 53250, "current": -1200,
            "min_temperature": 25, "max_temperature": 26, "cycle_count": 10}


def frame(packs):
    msg = UiDatagram()
    msg.robot_base.robot_id = "1"
    msg.robot_base.data_pool = json.dumps({"multi_battery_info": json.dumps(packs)})
    return msg.SerializeToString()


def test_units_order_zero_soc_and_unknown_fields():
    # Unknown field 15 ensures the projection decoder tolerates the full UI schema.
    result = parse_battery_frame(frame([pack("2", 99), pack("1", 0)]) + b"\x78\x01")
    assert [p["id"] for p in result] == ["1", "2"]
    assert result[0]["available"] and result[0]["soc_pct"] == 0
    assert result[1]["soc_pct"] == 99
    assert result[0]["voltage_v"] == 53.25
    assert result[0]["current_ma"] == -1200
    assert result[0]["temperature_min_c"] == 25
    assert result[0]["cycle_count"] == 10
    assert "charging" not in result[0] and "left" not in result[0]


def test_real_installed_chassis_frame():
    raw = Path(__file__).with_name("fixtures").joinpath("helios/robot_base_battery.bin").read_bytes()
    rows = parse_battery_frame(raw)
    assert [(p["id"], p["soc_pct"], p["voltage_v"], p["current_ma"],
             p["temperature_max_c"], p["cycle_count"]) for p in rows] == [
                 ("1", 98, 53.25, 0, 26, 10), ("2", 99, 53.2, 0, 26, 18)]
    assert rows[0]["cell_voltage_min_v"] == 3.502
    assert rows[0]["cell_voltage_max_v"] == 3.580
    assert rows[0]["full_charge_capacity_mah"] == 17700
    assert rows[0]["remaining_capacity_mah"] == 17346
    assert rows[0]["bms_hardware_version"] == "H:v10"
    assert rows[0]["bms_software_version"] == "F:v10"


def test_extended_fields_reject_bad_values_without_losing_soc():
    bad = {**pack(), "min_cell_voltage": 3600, "max_cell_voltage": 3500,
           "fcc": -1, "rc": True, "hardware_version": "bad\nversion", "software_version": 123}
    row = parse_battery_frame(frame([bad]))[0]
    assert row["available"] and row["soc_pct"] == 98
    for key in ["cell_voltage_min_v", "cell_voltage_max_v", "full_charge_capacity_mah",
                "remaining_capacity_mah", "bms_hardware_version", "bms_software_version"]:
        assert row[key] is None
    good = {**pack(), "min_cell_voltage": 0, "max_cell_voltage": 3500, "rc": 0, "fcc": 0}
    row = parse_battery_frame(frame([good]))[0]
    assert row["cell_voltage_min_v"] == 0 and row["remaining_capacity_mah"] == 0


@pytest.mark.parametrize("soc", [None, True, "98", -1, 101, math.nan, math.inf])
def test_invalid_soc_never_becomes_zero(soc):
    rows = parse_battery_frame(frame([pack("1", soc), pack("2", 99)]))
    assert rows[0]["soc_pct"] is None and not rows[0]["available"]
    assert rows[1]["soc_pct"] == 99


def test_missing_pack_and_optional_fields():
    rows = parse_battery_frame(frame([{"id": "2", "soc": 99}]))
    assert rows[0]["state"] == "not_reported" and rows[0]["soc_pct"] is None
    assert rows[1]["available"] and rows[1]["voltage_v"] is None
    assert all(p["state"] == "not_reported" for p in parse_battery_frame(frame([])))


@pytest.mark.parametrize("packs", [[pack(), pack()], [pack("3")], {}, [None]])
def test_invalid_pack_identity_or_shape(packs):
    with pytest.raises(ValueError):
        parse_battery_frame(frame(packs))


def test_malformed_protocol_and_json():
    assert parse_battery_frame(b"\x32\x00") is None  # current_state, not battery data
    for raw in [b"\xff", UiDatagram(robot_base={"data_pool": "broken"}).SerializeToString(),
                UiDatagram(robot_base={"data_pool": "{}"}).SerializeToString()]:
        with pytest.raises(ValueError):
            parse_battery_frame(raw)


@contextmanager
def chassis_server():
    state = {"mode": "valid", "connections": 0, "requests": []}
    ready = threading.Event()
    loop = asyncio.new_event_loop()

    async def handle(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        state["connections"] += 1
        try:
            while not ws.closed:
                mode = state["mode"]
                if mode == "close":
                    await ws.close()
                    break
                # Other messages continue during battery outage: they must NOT refresh age.
                await ws.send_bytes(b"\x32\x00")
                if mode == "valid":
                    await ws.send_bytes(frame([pack("1"), pack("2", 99)]))
                elif mode == "invalid":
                    await ws.send_bytes(b"\xff")
                try:
                    msg = await ws.receive(timeout=0.025)
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


def test_stream_expiry_reconnect_invalid_stop_and_no_commands():
    with chassis_server() as server:
        client = BatteryFeedbackClient(server["url"], stale_timeout_s=0.15, reconnect_delay_s=0.05)
        try:
            client.start()
            wait_for(lambda: client.snapshot()["available"])
            assert client.snapshot()["batteries"][0]["soc_pct"] == 98
            snap = client.snapshot()
            snap["batteries"][0]["soc_pct"] = 1000
            assert client.snapshot()["batteries"][0]["soc_pct"] == 98
            server["mode"] = "invalid"
            wait_for(lambda: client.snapshot()["state"] == "invalid")
            assert client.snapshot()["batteries"] == []
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            server["mode"] = "unrelated_only"
            wait_for(lambda: not client.snapshot()["available"])
            assert client.snapshot()["batteries"] == []
            wait_for(lambda: server["connections"] >= 2)
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            server["mode"] = "close"
            wait_for(lambda: not client.snapshot()["available"])
            server["mode"] = "valid"
            wait_for(lambda: client.snapshot()["available"])
            client.stop()
            assert client.snapshot()["state"] == "idle" and client.snapshot()["batteries"] == []
            client.start()
            wait_for(lambda: client.snapshot()["available"])
            assert server["requests"] == []
        finally:
            client.stop()


def test_bundle_independent_lifecycle_partial_and_validation():
    config = {"state_url": "ws://127.0.0.1/joints", "battery_state_url": "ws://127.0.0.1/battery"}
    with chassis_server() as server:
        battery = HeliosBattery(config, client=BatteryFeedbackClient(server["url"]))
        joints = HeliosJoints(config)
        bundle = DriverBundle([joints, battery])
        assert {t["name"]: t["type"] for t in bundle.get_all_tools()} == {
            "joints": "sensor", "battery": "sensor", "model": "resource"}
        try:
            assert bundle.dispatch("battery", {"action": "start"})["state"] == "running"
            wait_for(lambda: battery.payload()["complete"])
            payload = bundle.dispatch("battery", {})["data"]
            assert payload["units"]["current_ma"] == "mA" and payload["battery_count"] == 2
            assert payload["source_timestamp"] is None
            assert not payload["position_mapping_confirmed"]
            bundle.dispatch("joints", {"action": "stop"})
            assert battery.payload()["available"]
            bundle.dispatch("battery", {"action": "stop"})
            assert battery.payload()["batteries"] == []
            assert battery.payload()["state"] == "idle"
            info = bundle.dispatch("battery", {"action": "info"})
            assert info["configSchema"]["properties"]["battery_state_url"]["x-sensitive"]
            assert info["topic_out"] == [{"topic": "/rokae_helios/state/battery", "format": "data/json"}]
            # A live frame with one missing/invalid pack is explicitly partial.
            with battery.client._lock:
                battery.client._running = battery.client._connected = True
                battery.client._received = time.monotonic()
                battery.client._data = parse_battery_frame(frame([pack("2", 0)]))
            assert battery.payload()["state"] == "partial"
            assert battery.payload()["available"] and not battery.payload()["complete"]
        finally:
            battery.stop()
            joints.stop()
    for key, val in [("battery_publish_hz", 0), ("battery_publish_hz", 11),
                     ("battery_stale_timeout_s", 0)]:
        with pytest.raises(ValueError):
            HeliosBattery({**config, key: val})
