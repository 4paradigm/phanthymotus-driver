"""Offline coverage for the L20 Lite RS485 fingertip-pressure protocol and sensor card."""

import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

import lite_tactile
import modbus_device
from common.vendor_runtime import DriverBundle, load_config
from modbus_client import L20LiteModbusClient, crc16
from modbus_device import build_plugins

FINGERS = lite_tactile.FINGER_NAMES


def make_hand(**overrides):
    config = load_config(str(DRIVER / "main.py"))
    return modbus_device.MultiHandPlugin({**config, **overrides}, config["plugins"]["hand"])


def cells(finger_id):
    return [finger_id * 100 + i for i in range(96)]


class TactileSerial:
    """Replays the vendor host app's finger-select + 98-register read exchange."""

    def __init__(self, **_kwargs):
        self.sent = []
        self.pending = bytearray()
        self.selected = 0
        self.wrong_echo = False
        self.silent_fingers = set()
        self.timeout = self.write_timeout = 0.5

    def reset_input_buffer(self):
        self.pending.clear()

    def write(self, frame):
        frame = bytes(frame)
        self.sent.append(frame)
        assert crc16(frame[:-2]) == frame[-2:]
        slave, function = frame[:2]
        if function == 0x10:
            assert frame[2:7] == bytes((0x00, 0x46, 0x00, 0x01, 0x02))
            self.selected = int.from_bytes(frame[7:9], "big")
            payload = bytes((slave, 0x10)) + frame[2:6]
        elif function == 0x04:
            assert frame[2:6] == bytes((0x00, 0x3C, 0x00, 0x62))
            if self.selected in self.silent_fingers:
                return len(frame)
            echo = self.selected + 1 if self.wrong_echo else self.selected
            values = [echo, 0, *cells(self.selected)]
            data = b"".join(value.to_bytes(2, "big") for value in values)
            payload = bytes((slave, 0x04, len(data))) + data
        else:
            raise AssertionError(function)
        self.pending.extend(payload + crc16(payload))
        return len(frame)

    def flush(self):
        pass

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def close(self):
        pass


class FakeHand:
    def __init__(self, rounds=None):
        self.config = {"side": "right"}
        self.rounds = list(rounds or [])
        self.calls = []

    def read_tactile(self, side, fingers, *, budget_s, settle_s, cancel=None):
        self.calls.append(side)
        value = self.rounds.pop(0) if self.rounds else {}
        if isinstance(value, Exception):
            raise value
        now = time.monotonic()
        return f"/dev/{side}", {f: value.get(f, (cells(FINGERS.index(f) + 1), now)) for f in fingers}


def test_fingertip_frames_match_vendor_host_app():
    serial = TactileSerial()
    client = L20LiteModbusClient("/dev/null", "right", serial_factory=lambda **_: serial)
    assert client.read_fingertip("middle", settle_s=0) == cells(3)
    select, read = serial.sent
    assert select.hex(" ") == "27 10 00 46 00 01 02 00 03 " + crc16(select[:-2]).hex(" ")
    assert read[:6].hex(" ") == "27 04 00 3c 00 62"
    assert not serial.pending  # whole 201-byte reply consumed

    serial.wrong_echo = True
    with pytest.raises(ValueError, match="finger 4"):
        client.read_fingertip("middle", settle_s=0)


def test_read_tactile_isolates_failed_finger_and_reuses_port(monkeypatch, tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    serial = TactileSerial()
    serial.silent_fingers = {2}
    monkeypatch.setattr(modbus_device, "L20LiteModbusClient",
                        lambda *a, **k: L20LiteModbusClient(*a, **k, serial_factory=lambda **_: serial))
    hand = make_hand(response_timeout_s=0.05)
    discoveries = []
    monkeypatch.setattr(hand, "_discover", lambda **_: discoveries.append(1) or {"right": str(port)})

    for _ in range(2):
        used, results = hand.read_tactile("right", ["thumb", "index"], budget_s=1, settle_s=0)
    assert used == str(port) and len(discoveries) == 1
    assert results["thumb"][0] == cells(1) and isinstance(results["thumb"][1], float)
    assert isinstance(results["index"], TimeoutError)

    cancel = threading.Event()
    cancel.set()
    with pytest.raises(InterruptedError):
        hand.read_tactile("right", ["thumb"], budget_s=1, settle_s=0, cancel=cancel)

    for _ in range(2):  # an absent hand is rediscovered at most every 2 s
        with pytest.raises(ValueError, match="左手"):
            hand.read_tactile("left", ["thumb"], budget_s=1, settle_s=0)
    assert len(discoveries) == 2


def test_tactile_and_control_share_serial_across_port_aliases(monkeypatch, tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    alias = tmp_path / "by-path"
    alias.symlink_to(port)
    transports = []

    def open_client(*args, **kwargs):
        serial = TactileSerial()
        serial.closed = False

        def close():
            serial.closed = True

        serial.close = close
        transports.append(serial)
        return L20LiteModbusClient(*args, **kwargs, serial_factory=lambda **_: serial)

    monkeypatch.setattr(modbus_device, "L20LiteModbusClient", open_client)
    hand = make_hand(serial_port=str(alias))
    control = hand._legacy.connection
    motion_client = control.get("right")
    hand._tactile_ports["left"] = str(port)

    _, results = hand.read_tactile("left", ["thumb"], budget_s=1, settle_s=0)

    assert results["thumb"][0] == cells(1)
    assert len(transports) == 1
    serial = transports[0]
    assert all(frame[0] == 0x28 for frame in serial.sent)
    assert not serial.closed
    assert control.get("right") is motion_client
    motion_client.read_fingertip("thumb", settle_s=0)
    assert all(frame[0] == 0x27 for frame in serial.sent[-2:])
    control.close()
    assert serial.closed


def test_tactile_serial_error_retires_port_until_control_releases(monkeypatch, tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    transports = []

    def open_client(*args, **kwargs):
        serial = TactileSerial()
        serial.close_count = 0

        def close():
            serial.close_count += 1

        serial.close = close
        transports.append(serial)
        return L20LiteModbusClient(*args, **kwargs, serial_factory=lambda **_: serial)

    monkeypatch.setattr(modbus_device, "L20LiteModbusClient", open_client)
    hand = make_hand(serial_port=str(port))
    control = hand._legacy.connection
    control_client = control.get("right")
    gesture = hand._connection(side="right", serial_port=str(port))
    gesture.get()
    hand._tactile_ports["left"] = str(port)
    old_port = transports[0]
    original_write = old_port.write

    def fail_tactile_only(frame):
        if frame[0] == 0x28:
            raise OSError("test tactile write failure")
        return original_write(frame)

    monkeypatch.setattr(old_port, "write", fail_tactile_only)
    with pytest.raises(OSError, match="test tactile write failure"):
        hand.read_tactile("left", ["thumb"], budget_s=1, settle_s=0)

    assert old_port.close_count == 0
    assert control_client.read_fingertip("thumb", settle_s=0) == cells(1)
    waiting = hand._connection(side="left", serial_port=str(port))
    with pytest.raises(OSError, match="waiting for existing connections"):
        waiting.get()
    with pytest.raises(OSError, match="waiting for existing connections"):
        gesture.get()
    assert old_port.close_count == 0
    assert len(transports) == 1

    replacement = control.get()
    assert old_port.close_count == 1
    assert len(transports) == 2
    assert replacement is not control_client
    gesture.close()
    assert waiting.get("right") is replacement
    waiting.close()
    assert transports[1].close_count == 0
    control.close()
    assert transports[1].close_count == 1


def test_busy_bus_does_not_block_motion(tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    hand = make_hand()
    hand._tactile_ports["right"] = str(port)
    held, release = threading.Event(), threading.Event()

    def motion():
        with hand._bus_lock:
            held.set()
            release.wait()

    thread = threading.Thread(target=motion)
    thread.start()
    held.wait()
    try:
        started = time.monotonic()
        _, results = hand.read_tactile("right", FINGERS, budget_s=0.05, settle_s=0)
        assert time.monotonic() - started < 0.5
        assert all(isinstance(results[f], TimeoutError) for f in FINGERS)
    finally:
        release.set()
        thread.join()


def test_silent_fingers_share_round_deadline_and_restore_serial_timeouts(monkeypatch, tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    serial = TactileSerial()
    serial.silent_fingers = set(range(1, 6))
    now = 0.0

    def sleep(delay):
        nonlocal now
        now += delay

    read = serial.read

    def timed_read(size):
        if not serial.pending:
            sleep(serial.timeout)
        return read(size)

    monkeypatch.setattr(serial, "read", timed_read)
    monkeypatch.setattr(time, "monotonic", lambda: now)
    monkeypatch.setattr(time, "sleep", sleep)
    monkeypatch.setattr(modbus_device, "L20LiteModbusClient",
                        lambda *a, **k: L20LiteModbusClient(*a, **k, serial_factory=lambda **_: serial))
    hand = make_hand(serial_port=str(port))
    hand._legacy.connection.get()
    hand._tactile_ports["right"] = str(port)
    _, results = hand.read_tactile("right", FINGERS, budget_s=0.75, settle_s=0)

    assert now == pytest.approx(0.75)
    assert all(isinstance(results[finger], TimeoutError) for finger in FINGERS)
    assert [frame[1] for frame in serial.sent] == [0x10, 0x04, 0x10, 0x04]
    assert serial.timeout == serial.write_timeout == 0.5

    serial.silent_fingers.clear()
    _, results = hand.read_tactile("right", ["thumb"], budget_s=0.75, settle_s=0)
    assert results["thumb"][0] == cells(1)
    hand._legacy.connection.close()


def test_tactile_discovery_does_not_probe_more_ports_after_deadline(monkeypatch, tmp_path):
    first = tmp_path / "ttyUSB0"
    second = tmp_path / "ttyUSB1"
    first.touch()
    second.touch()
    now = 0.0
    opened = []

    class SilentSerial(TactileSerial):
        def write(self, frame):
            self.sent.append(frame)
            return len(frame)

        def read(self, size):
            nonlocal now
            now += self.timeout
            return b""

    def open_client(port, *args, **kwargs):
        opened.append(port)
        return L20LiteModbusClient(port, *args, **kwargs, serial_factory=SilentSerial)

    monkeypatch.setattr(time, "monotonic", lambda: now)
    monkeypatch.setattr(modbus_device, "L20LiteModbusClient", open_client)
    hand = make_hand(serial_port=str(first))
    monkeypatch.setattr(hand, "_ports", lambda: [str(first), str(second)])

    with pytest.raises(TimeoutError, match="触觉采集超时"):
        hand.read_tactile("right", FINGERS, budget_s=0.2, settle_s=0)

    assert now == pytest.approx(0.2)
    assert opened == [str(first)]
    assert "right" not in hand._tactile_retry_at


def test_card_matches_l20_schema_and_snapshot(monkeypatch):
    stale = time.monotonic() - 5
    # A value above 255 contributes to the raw mean. The tail is ignored.
    light = [0] * 96
    light[0:5] = [10, 50, 120, 200, 300]
    light[5], light[80] = 9, 255
    saturated = [65535] * 72 + [0] * 24
    sample_time = time.monotonic()
    rounds = [{"thumb": (light, sample_time), "middle": (saturated, sample_time)},
              {"index": TimeoutError("timeout")},
              {"thumb": (cells(1), stale)}, RuntimeError("bus gone")]
    hand = FakeHand(rounds)
    monkeypatch.setattr(modbus_device.MultiHandPlugin, "read_tactile",
                        lambda _self, *a, **k: hand.read_tactile(*a, **k))
    bundle = DriverBundle(build_plugins(load_config(str(DRIVER / "main.py")), "test", None))
    tools = {item["name"]: item for item in bundle.get_all_tools()}
    card = tools["hand_tactile"]
    assert card["type"] == "sensor" and card["multiInstance"] is True
    assert card["title"] == "L20 Lite 触觉感知"
    assert set(card["configSchema"]["properties"]) == {"bound_side", "poll_hz"}
    schema = card["inputSchema"]
    assert set(schema["properties"]["action"]["enum"]) >= {"info", "start", "stop", "config", "get_state"}
    assert schema["x-action-params"]["get_state"]["params"] == []

    def call(action, **kwargs):
        return bundle.dispatch("hand_tactile", {"action": action, "instance_id": "a", **kwargs})

    assert call("config", poll_hz=3)["bound_side"] == "right"  # frequency-only config
    assert call("config", bound_side="left")["topic_out"] == [
        {"topic": "/test/l20_lite/hand/left/tactile", "format": "sensor/tactile"}]
    first = call("get_state")
    assert first["state"] == "ok" and first["serial_port"] == "/dev/left"
    assert first["scalar_valid"] and first["matrix_valid"] and first["matrix_fresh"]
    assert first["model"] == "l20_lite"
    assert first["touch_scalar"]["thumb"] == 10  # Raw sum 689 / 72, rounded.
    assert first["touch_scalar"]["index"] == round(sum(cells(2)[:72]) / 72)
    assert first["touch_scalar"]["middle"] == 255
    assert first["touch_cells"]["middle"] == saturated
    assert first["touch_sum"]["index"] == sum(cells(2))
    # All 96 cells are published, including the 24 the 12x6 renderer omits.
    assert first["cell_count"] == 96
    assert first["touch_cells"]["thumb"] == light and first["touch_cells"]["index"] == cells(2)
    # The display matrix clips each cell independently of the mean.
    assert first["touch_matrix"]["index"][1] == cells(2)[6:12]  # all under 255
    assert first["touch_matrix"]["thumb"][0] == [10, 50, 120, 200, 255, 9]
    assert len(first["touch_matrix"]["thumb"]) == 12
    assert all(0 <= v <= 255 for f in FINGERS for row in first["touch_matrix"][f] for v in row)
    assert first["matrix_sums"]["thumb"] == sum(min(v, 255) for v in light[:72])
    assert first["touch_cells"]["thumb"][4] == 300
    assert all(first["contact_flags"].values())
    assert first["force_fields_valid"] is False and first["normal_force"]["thumb"] is None

    second = call("get_state")
    assert second["state"] == "error" and not second["scalar_valid"]
    assert "index" in second["errors"]["scalar"] and second["touch_scalar"]["thumb"] is None
    assert second["touch_cells"]["thumb"] is None
    assert second["sequence"] == first["sequence"] + 1

    third = call("get_state")
    assert third["errors"]["scalar"] == "标量读数已过期" and third["touch_scalar"]["index"] is None
    assert third["touch_cells"]["index"] is None

    fourth = call("get_state")
    assert fourth["errors"]["scalar"] == "bus gone" and fourth["serial_port"] is None


def test_anonymous_stop_stops_unclaimed_auto_start_stream():
    plugin = lite_tactile.LiteTactilePlugin(FakeHand(), {}, {"side": "right"})
    plugin.start()
    worker = plugin._worker_thread
    try:
        assert worker is not None and worker.is_alive()
        assert plugin.dispatch("stop", {"instance_id": "unknown"})["state"] == "idle"
        assert worker.is_alive()

        assert plugin.dispatch("stop", {})["state"] == "idle"
        assert not worker.is_alive()
        assert plugin.dispatch("info", {})["state"] == "idle"
        assert plugin.dispatch("stop", {})["state"] == "idle"

        assert plugin.dispatch("start", {"instance_id": "named"})["state"] == "running"
        plugin.dispatch("stop", {})
        assert plugin.dispatch("info", {"instance_id": "named"})["state"] == "running"
    finally:
        plugin.stop()


def test_streams_are_shared_by_side_and_stop_when_last_instance_leaves():
    plugin = lite_tactile.LiteTactilePlugin(FakeHand(), {"poll_hz": 10}, {"side": "right"})
    plugin.start()  # auto_start polls the default side
    try:
        assert plugin._running() and set(plugin._streams) == set()
        for instance in ("one", "two"):
            assert plugin.dispatch("start", {"instance_id": instance})["state"] == "running"
        assert set(plugin._streams) == {"right"}
        plugin.dispatch("config", {"instance_id": "one", "bound_side": "left"})
        assert set(plugin._streams) == {"left", "right"} and plugin._streams["left"]._running()
        plugin.dispatch("stop", {"instance_id": "two"})
        assert set(plugin._streams) == {"left"} and not plugin._running()
        info = plugin.dispatch("info", {"instance_id": "one"})
        assert info["state"] == "running" and info["topic_out"][0]["topic"] == "/l20_lite/hand/left/tactile"
    finally:
        plugin.stop()
    assert not plugin._streams and not plugin._running()


@pytest.mark.parametrize("side", ["front", 1])
def test_invalid_binding_is_rejected(side):
    plugin = lite_tactile.LiteTactilePlugin(FakeHand(), {}, {"side": "right"})
    with pytest.raises(ValueError):
        plugin.dispatch("config", {"bound_side": side})


@pytest.mark.parametrize("config", [{"poll_hz": 20}, {"contact_threshold": 0}, {"contact_threshold": 256},
                                    {"auto_start": "yes"}])
def test_invalid_config_is_rejected(config):
    with pytest.raises(ValueError):
        lite_tactile.LiteTactilePlugin(FakeHand(), config, {"side": "right"})
