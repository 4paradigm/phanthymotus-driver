"""Offline tests for dual-hand glove preview and motion ownership."""

from __future__ import annotations

import json
import math
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

import glove_teleop_device as glove
from modbus_device import RIGHT_FIST_POSE, RIGHT_OPEN_POSE
from common.vendor_runtime import DriverBundle, load_config
import modbus_device


@pytest.fixture(autouse=True)
def explicit_motion_opt_in_for_offline_tests(monkeypatch):
    # Fake-UART motion tests deliberately opt in; production remains preview-only.
    monkeypatch.setenv("GLOVE_PREVIEW_ONLY", "0")


def test_default_preview_blocks_arm_before_discovery(monkeypatch):
    monkeypatch.delenv("GLOVE_PREVIEW_ONLY", raising=False)
    hand = FakeHand()
    monkeypatch.setattr(hand, "_discover", lambda: pytest.fail("must not open hardware"))
    config = load_config(str(DRIVER / "config.yaml"))["plugins"]["glove_teleop"]
    assert config["preview_only"] is True
    assert config["watchdog_ms"] == 300
    device = glove.GloveTeleopPlugin(hand, config, "test", None)
    with pytest.raises(RuntimeError, match="GLOVE_PREVIEW_ONLY=0"):
        device._arm()
    assert not hand._teleop_active


def test_invalid_preview_override_cannot_enable_motion(monkeypatch):
    monkeypatch.setenv("GLOVE_PREVIEW_ONLY", "false")
    device = glove.GloveTeleopPlugin(FakeHand(), {}, "test", None)
    with pytest.raises(ValueError, match="GLOVE_PREVIEW_ONLY"):
        device._arm()


@pytest.mark.parametrize("failed", [False, True])
def test_release_log_distinguishes_ack_from_failed_release(failed, capsys):
    class ReleaseClient(FakeClient):
        def set_torques(self, values):
            if failed:
                raise TimeoutError("no zero-torque reply")
            super().set_torques(values)

    device = glove.GloveTeleopPlugin(FakeHand(), {}, "test", None)
    client = ReleaseClient()
    device._clients["left"] = client
    device._side_received["left"] = time.monotonic() - 0.3
    device._finish_disarm()
    log = capsys.readouterr().out.strip()
    event = json.loads(log.removeprefix("[glove-release] "))
    assert event["zero_torque_acknowledged"] is (not failed)
    assert event["last_sample_to_release_ms"] >= 300
    assert event["release_io_ms"] >= 0
    assert ("close", None) in client.commands
    assert not device._clients


def packet(left=None, right=None, timestamp=1):
    return json.dumps(
        {
            "schemaId": glove.SCHEMA,
            "handType": "LinkerHand/L20Lite",
            "dof": 10,
            "labels": glove.LABELS,
            "timestampMs": timestamp,
            "leftHand": left if left is not None else [100] * 10,
            "rightHand": right if right is not None else [150] * 10,
        }
    ).encode()


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def close(self):
        pass


class FakeClient:
    def __init__(self):
        self.positions = [254] * 10
        self.commands = []

    def get_positions(self):
        return list(self.positions)

    def set_positions(self, values):
        self.commands.append(("position", list(values)))
        self.positions = list(values)

    def set_speeds(self, values):
        self.commands.append(("speed", list(values)))

    def set_torques(self, values):
        self.commands.append(("torque", list(values)))

    def close(self):
        self.commands.append(("close", None))


class FakeHand:
    def __init__(self):
        self.config = {"serial_port": "/dev/fake"}
        self.hand_config = {
            "open_pose": list(RIGHT_OPEN_POSE),
            "fist_pose": list(RIGHT_FIST_POSE),
            "left_open_pose": list(RIGHT_OPEN_POSE),
            "left_fist_pose": list(RIGHT_FIST_POSE),
        }
        self._bus_lock = threading.RLock()
        connection = type("Connection", (), {"close": lambda self: None})()
        self._legacy = type(
            "Controller", (), {"_active": None, "connection": connection}
        )()
        self._controllers = {}
        self._teleop_active = False

    def _discover(self):
        return {"left": "/dev/left", "right": "/dev/right"}


def test_source_mode_environment_switches_topology_without_changing_config(monkeypatch):
    monkeypatch.setenv("GLOVE_SOURCE_MODE", "direct")
    config = {"source_mode": "direct_udp"}
    device = glove.GloveTeleopPlugin(FakeHand(), config, "test", None)
    assert device.source_mode == "direct"
    assert config["source_mode"] == "direct_udp"


def test_source_mode_environment_rejects_invalid_modes(monkeypatch):
    monkeypatch.setenv("GLOVE_SOURCE_MODE", "invalid")
    with pytest.raises(ValueError, match="source_mode"):
        glove.GloveTeleopPlugin(FakeHand(), {"source_mode": "direct_udp"}, "test", None)


def plugin():
    hand = FakeHand()
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        hand,
        {"source_mode": "udp", "source_ip": "192.0.2.1"},
        "test",
        None,
        lambda _: publisher,
    )
    device._publisher = publisher
    device._socket = object()  # start() opened the preview, without a real port.
    return device, hand, publisher


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition did not become true"
        time.sleep(0.005)


def stop_simulated_device(device):
    # These tests simulate the receiver without starting ROS or opening a UART.
    if device._thread is not None and not hasattr(device._thread, "join"):
        device._thread = None
    if device._socket is not None and not hasattr(device._socket, "close"):
        device._socket = None
    device.dispatch("stop", {})
    assert device._control_thread is None
    assert device._watchdog_thread is None


def armed_direct_device(monkeypatch, client, watchdog_ms=800):
    hand = FakeHand()
    hand._discover = lambda: {"left": "/dev/left"}
    device = glove.GloveTeleopPlugin(
        hand,
        {"source_mode": "direct", "watchdog_ms": watchdog_ms},
        "test",
        None,
        lambda _: FakePublisher(),
    )
    device._thread = object()
    device._publisher = FakePublisher()
    monkeypatch.setattr(
        glove,
        "L20LiteConnection",
        lambda _: type("Connection", (), {"get": lambda self: client})(),
    )
    opened = {"side": "left", "pitch": [0.0] * 5, "yaw": [0.0] * 5}
    closed = {"side": "left", "pitch": [60.0] * 5, "yaw": [0.0] * 5}
    device.accept_direct_sample(opened)
    device.dispatch("calibrate_open", {"side": "left"})
    device.accept_direct_sample(closed)
    device.dispatch("calibrate_fist", {"side": "left"})
    device._arm()
    return device, opened, closed


class GatedClient(FakeClient):
    def __init__(self):
        super().__init__()
        self.block_next = False
        self.entered = threading.Event()
        self.release = threading.Event()

    def set_positions(self, values):
        if self.block_next:
            self.block_next = False
            self.entered.set()
            assert self.release.wait(3), "test did not release simulated serial write"
        super().set_positions(values)


def test_one_second_serial_wait_does_not_block_receiving_or_replay_old_targets(
    monkeypatch,
):
    client = GatedClient()
    device, opened, closed = armed_direct_device(monkeypatch, client)
    try:
        client.block_next = True
        device.accept_direct_sample(closed)
        assert client.entered.wait(1)
        packets = device._packets
        deadline = time.monotonic() + 1.05  # Longer than the real 800 ms watchdog.
        while time.monotonic() < deadline:
            device.accept_direct_sample(opened)
            assert device.dispatch("info", {})["armed"]
            time.sleep(0.02)
        assert device._packets > packets + 20
        assert device._last_watchdog_event is None
        client.release.set()
        wait_for(lambda: device._commands == 2 and device._apply_started is None)
        assert device._timing["max_apply_ms"] > 1000
        # One in-flight close step, followed by the newest open target only.
        positions = [
            values for action, values in client.commands if action == "position"
        ]
        assert len(positions) == 3  # Includes the setup's current-position hold.
        assert positions[-1][2] == 254
        time.sleep(0.1)
        assert device._commands == 2
        assert device._armed
    finally:
        client.release.set()
        stop_simulated_device(device)


def test_bus_wait_takes_the_latest_target_after_lock_acquisition(monkeypatch):
    client = FakeClient()
    client.positions = [128] * 10
    device, opened, closed = armed_direct_device(monkeypatch, client)
    try:
        with device.hand._bus_lock:
            device.accept_direct_sample(closed)
            wait_for(lambda: device._apply_started is not None)
            for _ in range(10):
                device.accept_direct_sample(opened)
                time.sleep(0.01)
        wait_for(lambda: device._commands == 1 and device._apply_started is None)
        positions = [
            values for action, values in client.commands if action == "position"
        ]
        assert positions[-1][2] == 144  # Latest open; stale close would be 112.
        assert device._timing["bus_wait_ms"] >= 80
        assert device._timing["write_ms"] < device._timing["bus_wait_ms"]
    finally:
        stop_simulated_device(device)


def test_true_stream_loss_disarms_even_while_serial_write_is_blocked(monkeypatch):
    client = GatedClient()
    device, opened, closed = armed_direct_device(monkeypatch, client, watchdog_ms=100)
    try:
        client.block_next = True
        device.accept_direct_sample(closed)
        assert client.entered.wait(1)
        wait_for(lambda: not device._armed)
        assert device._pending_disarm
        assert not client.release.is_set()
        assert device._last_watchdog_event["apply_in_progress_ms"] >= 90
        with pytest.raises(RuntimeError, match="cleanup is still pending"):
            device._arm()
        # Data recovery must not automatically re-arm or enqueue another pose.
        device.accept_direct_sample(opened)
        client.release.set()
        assert device._cleanup_done.wait(1)
        assert not device._armed
        assert not device.hand._teleop_active
        assert len([1 for action, _ in client.commands if action == "position"]) == 2
        assert ("torque", [0] * 10) in client.commands
        assert ("close", None) in client.commands
    finally:
        client.release.set()
        stop_simulated_device(device)


def test_disarm_during_bus_wait_prevents_any_target_write(monkeypatch):
    client = FakeClient()
    device, opened, closed = armed_direct_device(monkeypatch, client)
    outcome = {}
    stopper = threading.Thread(
        target=lambda: outcome.update(device.dispatch("disarm", {}))
    )
    try:
        with device.hand._bus_lock:
            device.accept_direct_sample(closed)
            wait_for(lambda: device._apply_started is not None)
            stopper.start()
            wait_for(lambda: not device._armed)
            assert device._pending_disarm
            device.accept_direct_sample(opened)  # Must not wait for serial cleanup.
        stopper.join(1)
        assert not stopper.is_alive()
        assert outcome["armed"] is False
        assert len([1 for action, _ in client.commands if action == "position"]) == 1
        assert ("torque", [0] * 10) in client.commands
    finally:
        if stopper.ident is not None:
            stopper.join(1)
        stop_simulated_device(device)


def test_serial_error_stops_motion_but_does_not_stop_preview(monkeypatch):
    class FailingClient(FakeClient):
        fail_next = False

        def set_positions(self, values):
            if self.fail_next:
                raise TimeoutError("simulated Modbus timeout")
            super().set_positions(values)

    client = FailingClient()
    device, opened, closed = armed_direct_device(monkeypatch, client)
    try:
        client.fail_next = True
        device.accept_direct_sample(closed)
        wait_for(lambda: not device._armed and device._cleanup_done.is_set())
        assert "simulated Modbus timeout" in device._error
        assert device._commands == 0
        packets = device._packets
        device.accept_direct_sample(opened)
        assert device._packets == packets + 1
        assert ("torque", [0] * 10) in client.commands
    finally:
        stop_simulated_device(device)


def test_stop_joins_workers_and_rejects_overlapping_restart(monkeypatch):
    client = GatedClient()
    device, opened, closed = armed_direct_device(monkeypatch, client)
    outcome = {}

    def stop():
        try:
            outcome["result"] = device.dispatch("stop", {})
        except Exception as exc:
            outcome["error"] = exc

    stopper = threading.Thread(target=stop)
    try:
        client.block_next = True
        device.accept_direct_sample(closed)
        assert client.entered.wait(1)
        device._thread = None  # The simulated reader has no thread to join.
        stopper.start()
        wait_for(lambda: device._stopping and not device._armed)
        with pytest.raises(RuntimeError, match="still shutting down"):
            device.dispatch("start", {})
        assert device.accept_direct_sample(opened) is False
        client.release.set()
        stopper.join(1)
        assert not stopper.is_alive()
        assert outcome["result"]["state"] == "idle", outcome.get("error")
        assert device._control_thread is None and device._watchdog_thread is None
        assert not device._stopping
        assert len([1 for action, _ in client.commands if action == "position"]) == 2
    finally:
        client.release.set()
        if stopper.ident is not None:
            stopper.join(1)
        stop_simulated_device(device)


def test_queued_stale_samples_do_not_refresh_the_watchdog():
    device, _, _ = plugin()
    device.source_mode = "direct"
    sample = {
        "side": "left",
        "pitch": [0.0] * 5,
        "yaw": [0.0] * 5,
        "timestamp_ms": int(time.time() * 1000) - 2000,
    }
    assert device.accept_direct_sample(sample) is False
    assert device._packets == 0
    assert "left" not in device._side_received


def test_arm_setup_can_receive_and_can_be_cancelled_before_torque_enable(monkeypatch):
    class GatedReadClient(FakeClient):
        def __init__(self):
            super().__init__()
            self.entered = threading.Event()
            self.release = threading.Event()

        def get_positions(self):
            self.entered.set()
            assert self.release.wait(3)
            return super().get_positions()

    client = GatedReadClient()
    hand = FakeHand()
    hand._discover = lambda: {"left": "/dev/left"}
    device = glove.GloveTeleopPlugin(
        hand, {"source_mode": "direct"}, "test", None, lambda _: FakePublisher()
    )
    device._thread = object()
    opened = {"side": "left", "pitch": [0.0] * 5, "yaw": [0.0] * 5}
    closed = {"side": "left", "pitch": [60.0] * 5, "yaw": [0.0] * 5}
    device.accept_direct_sample(opened)
    device.dispatch("calibrate_open", {"side": "left"})
    device.accept_direct_sample(closed)
    device.dispatch("calibrate_fist", {"side": "left"})
    monkeypatch.setattr(
        glove,
        "L20LiteConnection",
        lambda _: type("Connection", (), {"get": lambda self: client})(),
    )
    outcome = {}

    def arm():
        try:
            device._arm()
        except Exception as exc:
            outcome["error"] = str(exc)

    worker = threading.Thread(target=arm)
    stopper = threading.Thread(target=lambda: device.dispatch("disarm", {}))
    worker.start()
    try:
        assert client.entered.wait(1)
        packets = device._packets
        device.accept_direct_sample(opened)
        assert device._packets == packets + 1
        stopper.start()
        wait_for(lambda: device._pending_disarm)
        client.release.set()
        worker.join(1)
        stopper.join(1)
        assert not worker.is_alive() and not stopper.is_alive()
        assert "cancelled" in outcome["error"]
        assert ("torque", [0] * 10) in client.commands
        assert not any(
            action == "torque" and any(values) for action, values in client.commands
        )
        assert not device._armed and not hand._teleop_active
    finally:
        client.release.set()
        worker.join(1)
        if stopper.ident is not None:
            stopper.join(1)
        stop_simulated_device(device)


def test_frame_contract_and_dual_hand_urdf():
    device, _, publisher = plugin()
    root = ET.fromstring(device.dispatch("model", {})["urdf"])
    names = {joint.attrib["name"] for joint in root.findall("joint")}
    assert "left_index_mcp_pitch" in names
    assert "right_index_mcp_pitch" in names
    assert not device.accept_packet(packet(), "192.0.2.2", now=1)
    assert device.accept_packet(packet(), "192.0.2.1", now=1)
    preview = publisher.messages[-1]
    assert len(preview["joints"]) == 40
    assert {joint["name"] for joint in preview["joints"]} <= names
    assert preview["preview_is_glove_target"] is True
    assert device.dispatch("info", {})["sent_commands"] == 0


@pytest.mark.parametrize("preview_override", [None, "0", "1"])
def test_driver_registers_all_cards_and_model_in_preview_mode(monkeypatch, preview_override):
    if preview_override is None:
        monkeypatch.delenv("GLOVE_PREVIEW_ONLY", raising=False)
    else:
        monkeypatch.setenv("GLOVE_PREVIEW_ONLY", preview_override)

    def forbidden(*args, **kwargs):
        pytest.fail("registration and preview-only arm must not open hardware")

    monkeypatch.setattr(modbus_device.MultiHandPlugin, "_discover", forbidden)
    monkeypatch.setattr(modbus_device.L20LiteConnection, "get", forbidden)
    config = load_config(str(DRIVER / "main.py"))
    bundle = DriverBundle(modbus_device.build_plugins(config, "test", None))
    tools = bundle.get_all_tools()
    by_name = {item["name"]: item for item in tools}
    assert {"hand", "hand_gesture", "glove_teleop", "model"} <= by_name.keys()
    assert by_name["glove_teleop"]["topic_out"] == [
        {"topic": "/test/glove_teleop/skeleton", "format": "sensor/skeleton"}
    ]
    assert by_name["model"]["type"] == "resource"
    assert "<robot" in bundle.dispatch("model", {})["urdf"]
    if preview_override != "0":
        with pytest.raises(RuntimeError, match="GLOVE_PREVIEW_ONLY=0"):
            bundle.dispatch("glove_teleop", {"action": "arm", "confirm_motion": True})


def test_udp_preview_receives_without_moving_hardware():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    hand = FakeHand()
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        hand,
        {
            "source_mode": "udp",
            "source_ip": "127.0.0.1",
            "listen_host": "127.0.0.1",
            "listen_port": port,
        },
        "test",
        None,
        lambda _: publisher,
    )
    try:
        assert device.dispatch("start", {})["state"] == "running"
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(packet(), ("127.0.0.1", port))
        deadline = time.monotonic() + 1
        while not publisher.messages and time.monotonic() < deadline:
            time.sleep(0.01)
        assert publisher.messages
        assert device.dispatch("info", {})["received_frames"] == 1
        assert device.dispatch("info", {})["sent_commands"] == 0
    finally:
        device.dispatch("stop", {})


@pytest.mark.parametrize("with_quaternions", [False, True])
def test_direct_reader_udp_angles_reach_card_without_motor_commands(with_quaternions):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    hand = FakeHand()
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        hand,
        {
            "source_mode": "direct_udp",
            "source_ip": "127.0.0.1",
            "listen_host": "127.0.0.1",
            "listen_port": port,
        },
        "test",
        None,
        lambda _: publisher,
    )
    try:
        assert device.dispatch("start", {})["source_mode"] == "direct_udp"
        sample = {
            "side": "left",
            "pitch": [10.0, 20.0, 30.0, 40.0, 50.0],
            "yaw": [0.0] * 5,
            "sequence": 1,
        }
        if with_quaternions:
            sample.update(
                quaternion_layout="hex-root-top-v1",
                root_quaternions=[axis_quaternion(0)] * 6,
                tip_quaternions=[None] * 6,
            )
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(sample).encode(), ("127.0.0.1", port))
        deadline = time.monotonic() + 1
        while not publisher.messages and time.monotonic() < deadline:
            time.sleep(0.01)
        assert publisher.messages[-1]["glove_angles"]["left"] == sample["pitch"]
        assert publisher.messages[-1]["orientation_source"]["left"] == (
            "quaternion" if with_quaternions else "euler"
        )
        assert device.dispatch("info", {})["sent_commands"] == 0
    finally:
        device.dispatch("stop", {})


def test_ring_and_pinky_spikes_do_not_straighten_the_preview():
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None, lambda _: publisher
    )
    device._publisher = publisher
    device._thread = object()
    base = time.monotonic()

    def sample(pitch, when):
        device.accept_direct_sample(
            {"side": "left", "pitch": pitch, "yaw": [0] * 5}, now=base + when
        )
        return publisher.messages[-1]["raw_positions"]["left"]

    opened, closed = [0] * 5, [80] * 5
    sample(opened, 0)
    device.dispatch("calibrate_open", {"side": "left"})
    sample(closed, 0.02)
    assert device.dispatch("calibrate_fist", {"side": "left"})["ready"]
    initial = sample(closed, 0.04)
    spike = [80, 80, 80, 0, 0]
    assert sample(spike, 0.06)[4:6] == initial[4:6]
    assert sample(closed, 0.08)[4:6] == initial[4:6]
    sample(spike, 0.10)
    sample(spike, 0.20)
    sustained = sample(spike, 0.27)
    assert sustained[4] > initial[4] and sustained[5] > initial[5]


def test_quaternions_override_euler_flips_for_ring_and_pinky():
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None, lambda _: publisher
    )
    device._publisher = publisher
    device._thread = object()

    def quaternions(degrees):
        result = [[0.0, 0.0, 0.0, 1.0]]
        for value in degrees:
            half = math.radians(value) / 2
            result.append([math.sin(half), 0.0, 0.0, math.cos(half)])
        return result

    def sample(pitch, degrees, now):
        device.accept_direct_sample(
            {
                "side": "left",
                "pitch": pitch,
                "yaw": [0.0] * 5,
                "quaternion_layout": "hex-root-top-v1",
                "root_quaternions": quaternions(degrees),
                "tip_quaternions": [None] * 6,
            },
            now=now,
        )
        return publisher.messages[-1]["raw_positions"]["left"]

    opened, closed = [0.0] * 5, [80.0] * 5
    base = time.monotonic()
    sample(opened, opened, base)
    device.dispatch("calibrate_open", {"side": "left"})
    sample(closed, closed, base + 0.1)
    result = device.dispatch("calibrate_fist", {"side": "left"})
    assert result["ready"] and result["orientation_source"] == "quaternion"
    baseline = sample(closed, closed, base + 0.2)
    # Reproduce the vendor Euler discontinuity while physical orientations stay fixed.
    flipped_euler = [80.0, 80.0, 80.0, -20.0, -40.0]
    stable = sample(flipped_euler, closed, base + 0.3)
    assert stable[4:6] == baseline[4:6]
    assert publisher.messages[-1]["orientation_source"]["left"] == "quaternion"
    # Even a physically implausible quaternion change cannot snap a finger
    # through its complete calibrated stroke in one frame.
    sudden = sample(flipped_euler, opened, base + 0.4)
    assert sudden[4] < 30 and sudden[5] < 30
    assert publisher.messages[-1]["mapped_fractions"]["left"][3:] == [1.0, 1.0]
    # A persistent physical change is eventually accepted, then rate-limited.
    sample(flipped_euler, opened, base + 0.60)
    assert publisher.messages[-1]["mapped_fractions"]["left"][3:] == [0.95, 0.95]


def axis_quaternion(degrees):
    half = math.radians(degrees) / 2
    return [math.sin(half), 0.0, 0.0, math.cos(half)]


@pytest.mark.parametrize("fist_degrees", [160, 174])
def test_flexion_stays_closed_when_crossing_180_degrees(fist_degrees):
    fraction = glove.GloveTeleopPlugin._quaternion_fraction
    opened, closed = axis_quaternion(0), axis_quaternion(fist_degrees)
    assert fraction(opened, closed, axis_quaternion(-10)) == 0
    assert fraction(opened, closed, axis_quaternion(fist_degrees / 2)) == pytest.approx(
        0.5
    )
    for angle in range(175, 201):
        current = axis_quaternion(angle)
        assert fraction(opened, closed, current) == 1
        assert fraction(opened, closed, [-v for v in current]) == 1


@pytest.mark.parametrize(
    "opened,closed,current",
    [
        # Captured during the reported index/middle-finger opening fault.
        (
            [
                0.15272415438090065,
                -0.22695663403799562,
                0.10871204155326475,
                0.9556922679168769,
            ],
            [
                -0.9368890904025198,
                -0.05638259796326392,
                -0.14049768836644763,
                0.31515112961139274,
            ],
            [
                -0.9874694406974898,
                0.00020376263593054056,
                0.1305012156650263,
                0.0887327159469688,
            ],
        ),
        (
            [
                0.042396634408407186,
                -0.05748009406376439,
                0.002547884983139348,
                0.9974427665081209,
            ],
            [
                -0.9908620814275688,
                -0.06282917802400537,
                -0.07263665580100401,
                0.09470346462458584,
            ],
            [
                -0.9942542481632726,
                -0.08543975128463077,
                0.06448665418962236,
                -0.00010169136845022964,
            ],
        ),
    ],
)
def test_recorded_fist_poses_do_not_map_to_open(opened, closed, current):
    assert glove.GloveTeleopPlugin._quaternion_fraction(opened, closed, current) > 0.95


def root_top_sample(side, roots, tips):
    return {
        "side": side,
        "pitch": [0.0] * 5,
        "yaw": [0.0] * 5,
        "quaternion_layout": "hex-root-top-v1",
        "root_quaternions": roots,
        "tip_quaternions": tips,
    }


@pytest.mark.parametrize("side", ["left", "right"])
def test_multi_frame_calibration_is_stable_sign_invariant_and_side_specific(monkeypatch, side):
    clock = [1000.0]
    monkeypatch.setattr(glove.time, "monotonic", lambda: clock[0])
    device = glove.GloveTeleopPlugin(FakeHand(), {
        "source_mode": "direct", "calibration_window_ms": 300,
        "calibration_stability_deg": 8, "open_deadband": 0.10,
    }, "test", None)

    def send(degrees, sign=1):
        clock[0] += 0.02
        q = [sign*v for v in axis_quaternion(degrees)]
        sample = root_top_sample(side, [axis_quaternion(0)] + [q]*5, [None]*6)
        device.accept_direct_sample(sample, now=clock[0])

    send(0)
    assert device._stable_calibration_pose(side, clock[0]) is None
    for i in range(16):
        send(1 if i % 2 else -1, -1 if i % 2 else 1)
    opened = device.dispatch("calibrate_open", {"side": side})
    assert opened["sample_count"] >= 8
    assert not opened["ready"]
    assert set(device._calibration) == {side}
    original = [list(q) for q in device._quaternion_calibration[side]["open"]]
    for i in range(16):
        send(i * 5)
    assert device._stable_calibration_pose(side, clock[0]) is None
    assert device._quaternion_calibration[side]["open"] == original
    for i in range(16):
        send(80, -1 if i % 2 else 1)
    closed = device.dispatch("calibrate_fist", {"side": side})
    assert closed["sample_count"] >= 8 and closed["ready"]
    assert device._stable_calibration_pose(side, clock[0] + 1) is None


def test_natural_open_deadband_converges_without_bypassing_step_limit():
    device = glove.GloveTeleopPlugin(FakeHand(), {"open_deadband": 0.1}, "test", None)
    device._mapped_fractions["left"] = [1.0]*5
    values = [device._stabilize_fraction("left", 1, 0.08) for _ in range(22)]
    assert values[-1] == 0
    assert all(abs(b-a) <= device.max_fraction_step + 1e-10
               for a, b in zip([1.0] + values, values))
    device._mapped_fractions["left"][1] = 0.12
    assert device._stabilize_fraction("left", 1, 0.12) == pytest.approx(0.12)


def thumb_device(monkeypatch, side="left", complete=True, fist_roll=50):
    clock = [1000.0]
    monkeypatch.setattr(glove.time, "monotonic", lambda: clock[0])
    hand = FakeHand()
    hand._discover = lambda: {side: f"/dev/{side}"}
    device = glove.GloveTeleopPlugin(hand, {
        "source_mode": "direct", "thumb_mapping": {"enabled": True},
    }, "test", None)
    device._thread = object()
    sign = 1 if side == "left" else -1

    def rotation(degrees, axis):
        q = [0.0, 0.0, 0.0, math.cos(math.radians(degrees)/2)]
        q[axis] = math.sin(math.radians(degrees)/2)
        return q

    def send(flex=0, sway=0, roll=0, fingers=0, wrist=0, target_side=side):
        clock[0] += 0.02
        palm = rotation(wrist, 0)
        root = device._quaternion_multiply(rotation(sign*sway, 1), rotation(sign*roll, 2))
        tip = device._quaternion_multiply(root, rotation(flex, 0))
        sample = root_top_sample(target_side,
            [palm, device._quaternion_multiply(palm, root)] +
            [device._quaternion_multiply(palm, rotation(fingers, 0))]*4,
            [device._quaternion_multiply(palm, tip), None] + [None]*4)
        device.accept_direct_sample(sample, now=clock[0])
        return sample

    send()
    device.dispatch("calibrate_open", {"side": side})
    send(flex=80, roll=fist_roll, fingers=80)
    device.dispatch("calibrate_fist", {"side": side})
    if complete:
        send(sway=40)
        device.dispatch("calibrate_thumb_sway", {"side": side})
        send(roll=50)
        assert device.dispatch("calibrate_thumb_roll", {"side": side})["thumb_ready"]
    return device, send, clock


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("movement,motor", [("flex", 0), ("sway", 1), ("roll", 9)])
def test_thumb_channels_independently_reach_full_range_and_preserve_four_fingers(monkeypatch, side, movement, motor):
    device, send, _ = thumb_device(monkeypatch, side)
    # Let the existing ring/pinky jump guard settle after fist calibration.
    for _ in range(60):
        send()
    baseline = device._last_frame[side][2:9]
    for _ in range(25):
        send(**{movement: {"flex": 80, "sway": 40, "roll": 50}[movement]})
    target = device._last_frame[side]
    assert target[motor] == 0  # Previously thumb flex stopped at 93.
    assert all(target[i] == 254 for i in (0, 1, 9) if i != motor)
    assert target[2:9] == baseline
    assert device._thumb.ready(side)
    assert not device._thumb.ready("right" if side == "left" else "left")
    for _ in range(25):
        send()
    assert [device._last_frame[side][i] for i in (0, 1, 9)] == [254]*3


def test_thumb_mapping_is_wrist_invariant_and_other_hand_frames_do_not_advance_it(monkeypatch):
    device, send, _ = thumb_device(monkeypatch)
    for _ in range(60):
        send(flex=40, sway=20, wrist=55)
    before = list(device._last_frame["left"])
    for _ in range(25):
        send(flex=40, sway=20, wrist=-35)
    assert device._last_frame["left"] == before
    send(flex=80)
    fractions = list(device._thumb.fractions["left"])
    for _ in range(8):
        send(target_side="right")
    assert device._thumb.fractions["left"] == fractions


def test_thumb_missing_tip_does_not_refresh_watchdog_and_incomplete_calibration_cannot_arm(monkeypatch):
    device, send, clock = thumb_device(monkeypatch, complete=False)
    before = device._side_received["left"]
    invalid = root_top_sample("left", [axis_quaternion(0)]*6, [None]*6)
    with pytest.raises(ValueError, match="thumb"):
        device.accept_direct_sample(invalid, now=clock[0]+0.1)
    assert device._side_received["left"] == before
    monkeypatch.setattr(glove, "L20LiteConnection", lambda _: pytest.fail("must not enable torque"))
    with pytest.raises(RuntimeError, match="calibrate_thumb_sway"):
        device._arm()
    assert not device._armed


def test_overlapping_thumb_calibration_is_rejected_without_replacing_previous_pose(monkeypatch):
    device, send, _ = thumb_device(monkeypatch)
    previous = device._thumb.calibration["left"]
    send(sway=40)
    with pytest.raises(RuntimeError, match="overlap"):
        device.dispatch("calibrate_thumb_roll", {"side": "left"})
    assert device._thumb.calibration["left"] is previous
    send()
    device.dispatch("calibrate_open", {"side": "left"})
    assert not device._thumb.ready("left")


def test_three_channel_thumb_preview_includes_sway_roll_and_tip_joints(monkeypatch):
    device, _, _ = thumb_device(monkeypatch)
    opened = {j["name"]: j["q"] for j in device.skeleton.sample("left", [254]*10)}
    closed = {j["name"]: j["q"] for j in device.skeleton.sample("left", [0]*10)}
    for name in ("thumb_cmc_pitch", "thumb_cmc_yaw", "thumb_cmc_roll", "thumb_mcp", "thumb_ip"):
        assert closed[f"left_{name}"] > opened[f"left_{name}"]


def test_thumb_tip_movement_prevents_stable_capture_even_when_root_is_still(monkeypatch):
    device, send, clock = thumb_device(monkeypatch)
    device.calibration_window_s = 0.3
    for i in range(18):
        send(flex=i*5)
    assert device._thumb.stable_pose("left", clock[0], 0.3, 8) is None
    for _ in range(18):
        send(flex=30)
    assert device._thumb.stable_pose("left", clock[0], 0.3, 8) is not None


def test_thumb_tip_can_calibrate_with_stationary_thumb_root(monkeypatch):
    device, send, _ = thumb_device(monkeypatch, fist_roll=0)
    assert device._thumb.ready("left")
    for _ in range(25):
        send(flex=80)
    assert device._last_frame["left"][0] == 0
    assert device._last_frame["left"][1] == device._last_frame["left"][9] == 254


def test_three_channel_thumb_uses_existing_motor_step_limit(monkeypatch):
    device, send, _ = thumb_device(monkeypatch)
    client = FakeClient()
    monkeypatch.setattr(glove, "L20LiteConnection", lambda _: type(
        "Connection", (), {"get": lambda self: client})())
    # Keep this test synchronous so every target write is inspected explicitly.
    monkeypatch.setattr(device, "_ensure_control_threads_locked", lambda: None)
    try:
        send()
        assert device._arm()["sides"] == ["left"]
        for _ in range(25):
            send(flex=80, sway=40, roll=50)
        device._apply_latest()
        command = [v for kind, v in client.commands if kind == "position"][-1]
        assert all(abs(v-254) <= 16 for v in command)
        assert all(command[i] < 254 for i in (0, 1, 9))
    finally:
        stop_simulated_device(device)
    assert device._thumb.calibration == {} and device._thumb.current == {}


def test_vendor_root_top_mapping_ignores_thumb_tip_and_unused_slot():
    roots = [axis_quaternion(v) for v in (10, 20, 30, 40, 50, 60)]
    tips = [axis_quaternion(150), [float("nan")] * 4] + [axis_quaternion(160)] * 4
    selected = glove.GloveTeleopPlugin._root_top_quaternions(
        root_top_sample("left", roots, tips)
    )
    assert all(
        actual == pytest.approx(expected) for actual, expected in zip(selected, roots)
    )
    # Seven-sensor hardware can omit the four root IMUs, but never the palm/thumb root.
    missing_roots = roots[:2] + [None] * 4
    selected = glove.GloveTeleopPlugin._root_top_quaternions(
        root_top_sample("left", missing_roots, tips)
    )
    assert all(
        actual == pytest.approx(expected)
        for actual, expected in zip(selected, roots[:2] + tips[2:6])
    )
    for slot in (0, 1):
        invalid = list(roots)
        invalid[slot] = None
        with pytest.raises(ValueError, match="palm or finger root"):
            glove.GloveTeleopPlugin._root_top_quaternions(
                root_top_sample("left", invalid, tips)
            )


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("seven_sensor", [False, True])
def test_thumb_and_wrist_motion_do_not_move_stationary_ring_and_pinky(
    side, seven_sensor
):
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None
    )
    device._thread = object()
    base = time.monotonic()

    def send(roots, tips, when):
        if seven_sensor:
            roots = roots[:2] + [None] * 4
        device.accept_direct_sample(root_top_sample(side, roots, tips), now=base + when)

    identity = axis_quaternion(0)
    send([identity] * 6, [identity, None] + [identity] * 4, 0)
    device.dispatch("calibrate_open", {"side": side})
    closed = [identity] + [axis_quaternion(80)] * 5
    tips = [axis_quaternion(70), None] + [axis_quaternion(80)] * 4
    send(closed, tips, 0.1)
    assert device.dispatch("calibrate_fist", {"side": side})["ready"]
    for i in range(20):
        palm = axis_quaternion(30 + i)
        relative = [identity, axis_quaternion(30)] + [axis_quaternion(80)] * 4
        moved_roots = [device._quaternion_multiply(palm, q) for q in relative]
        moved_tips = [axis_quaternion(-60 + i), axis_quaternion(170)] + [
            device._quaternion_multiply(palm, axis_quaternion(80)) for _ in range(4)
        ]
        send(moved_roots, moved_tips, 0.2 + i * 0.02)
        fractions = device._mapped_fractions[side]
        assert fractions[3:] == [1.0, 1.0]
    assert device._mapped_fractions[side][0] < 0.5


def test_missing_quaternions_cannot_refresh_a_quaternion_calibrated_hand():
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None
    )
    device._thread = object()
    now = time.monotonic()
    device.accept_direct_sample(
        root_top_sample("left", [axis_quaternion(0)] * 6, [None] * 6), now=now
    )
    device.dispatch("calibrate_open", {"side": "left"})
    with pytest.raises(ValueError, match="missing root/top"):
        device.accept_direct_sample(
            {"side": "left", "pitch": [0] * 5, "yaw": [0] * 5}, now=now + 1
        )
    assert device._side_received["left"] == now


def test_ambiguous_old_quaternion_packets_are_rejected():
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None
    )
    with pytest.raises(ValueError, match="ambiguous"):
        device.accept_direct_sample(
            {
                "side": "left",
                "pitch": [0] * 5,
                "yaw": [0] * 5,
                "quaternions": [axis_quaternion(0)] * 7,
            }
        )


def test_uncalibrated_preview_also_uses_corrected_quaternions():
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        FakeHand(), {"source_mode": "direct"}, "test", None, lambda _: publisher
    )
    device._publisher = publisher
    roots = [axis_quaternion(0)] + [axis_quaternion(80)] * 5
    sample = root_top_sample(
        "left", roots, [axis_quaternion(0), None] + [axis_quaternion(80)] * 4
    )
    device.accept_direct_sample(sample, now=1)
    baseline = publisher.messages[-1]["raw_positions"]["left"]
    sample["tip_quaternions"][0] = axis_quaternion(60)
    sample["pitch"] = [80, 80, 80, -40, -50]
    device.accept_direct_sample(sample, now=2)
    assert publisher.messages[-1]["raw_positions"]["left"] == baseline


@pytest.mark.parametrize(
    "change",
    [
        {"schemaId": "wrong"},
        {"labels": list(reversed(glove.LABELS))},
        {"leftHand": [True] * 10},
        {"rightHand": [256] * 10},
        {"timestampMs": "1"},
    ],
)
def test_bad_glove_frames_are_rejected(change):
    data = json.loads(packet())
    data.update(change)
    with pytest.raises(ValueError):
        glove.GloveTeleopPlugin.parse_frame(json.dumps(data).encode())


@pytest.mark.parametrize("outcome", ["completed", "error", "cancelled", "callback_failure"])
def test_arm_dispatch_is_async_and_reports_one_terminal_completion(monkeypatch, outcome):
    device, hand, _ = plugin()
    schema = device.get_tools()[0]["inputSchema"]
    assert schema["x-completion"] == {"actions": ["arm"], "timeout": 60}
    assert schema["x-resource"] == ["l20_lite_left_hand", "l20_lite_right_hand"]
    device.accept_packet(packet(), "192.0.2.1")
    entered, release, completed = threading.Event(), threading.Event(), threading.Event()
    reports = []
    clients = {side: FakeClient() for side in ("left", "right")}

    def discover():
        entered.set()
        assert release.wait(2)
        if outcome == "error":
            raise OSError("discovery failed")
        return {"left": "/dev/left", "right": "/dev/right"}

    def callback(action_id, status, result):
        reports.append((action_id, status, result))
        completed.set()
        if outcome == "callback_failure":
            raise OSError("Core unavailable")

    hand._discover = discover
    monkeypatch.setattr(device, "_acp_callback", callback)
    monkeypatch.setattr(glove, "L20LiteConnection", lambda cfg: type(
        "Connection", (), {"get": lambda self: clients[cfg["side"]]})())
    try:
        reply = device.dispatch("arm", {"confirm_motion": True})
        # Discovery is blocked: the request has returned without waiting for I/O.
        assert reply["state"] == "running" and reply["action_id"].startswith("glove_arm_")
        assert entered.wait(1) and not completed.is_set()
        assert hand._teleop_active and device._arming
        with pytest.raises(RuntimeError, match="pending"):
            device.dispatch("arm", {"confirm_motion": True})
        if outcome == "cancelled":
            device.dispatch("disarm", {})
        device.accept_packet(packet(timestamp=2), "192.0.2.1")
        release.set()
        assert completed.wait(1)
        if outcome == "callback_failure":
            wait_for(lambda: device._arm_completion_failed and not device._armed
                     and device._cleanup_done.is_set() and device._arm_action is None)
            assert all(("torque", [0]*10) in c.commands for c in clients.values())
            with pytest.raises(RuntimeError, match="ACP"):
                device.dispatch("arm", {"confirm_motion": True})
        else:
            wait_for(lambda: device._arm_action is None)
            assert reports[0][1] == outcome
            assert device._armed is (outcome == "completed")
        assert len(reports) == 1 and reports[0][0] == reply["action_id"]
        assert device._last_arm_result["action_id"] == reply["action_id"]
        if outcome == "cancelled":
            assert not any(c.commands for c in clients.values())
    finally:
        release.set()
        stop_simulated_device(device)


@pytest.mark.parametrize("first_failure", ["setup", "callback"])
def test_failed_arm_can_recover_and_retry_without_restarting_driver(monkeypatch, first_failure):
    device, hand, _ = plugin()
    device.accept_packet(packet(), "192.0.2.1")
    found = {"right": "/dev/right"} if first_failure == "setup" else {
        "left": "/dev/left", "right": "/dev/right"}
    hand._discover = lambda: dict(found)
    clients = {side: FakeClient() for side in ("left", "right")}
    monkeypatch.setattr(glove, "L20LiteConnection", lambda cfg: type(
        "Connection", (), {"get": lambda self: clients[cfg["side"]]})())
    reports = []
    core_online = [first_failure != "callback"]

    def callback(action_id, status, result):
        reports.append((action_id, status))
        if not core_online[0]:
            raise OSError("Core unavailable")

    monkeypatch.setattr(device, "_acp_callback", callback)
    try:
        first = device.dispatch("arm", {"confirm_motion": True})
        wait_for(lambda: device._arm_action is None)
        assert not device._arming and not device._armed and not hand._teleop_active
        if first_failure == "callback":
            assert device._arm_completion_failed
            with pytest.raises(RuntimeError, match="ACP"):
                device.dispatch("arm", {"confirm_motion": True})
            command_counts = {side: len(c.commands) for side, c in clients.items()}
            core_online[0] = True
            assert device.dispatch("disarm", {})["completion_retry"] is True
            wait_for(lambda: device._arm_action is None)
            assert not device._arm_completion_failed
            assert reports[-1] == reports[0]  # Same terminal action, not a new motion.
            assert {side: len(c.commands) for side, c in clients.items()} == command_counts
        else:
            assert reports == [(first["action_id"], "error")]
        found.update(left="/dev/left", right="/dev/right")
        device.accept_packet(packet(timestamp=2), "192.0.2.1")
        second = device.dispatch("arm", {"confirm_motion": True})
        wait_for(lambda: device._arm_action is None)
        assert second["action_id"] != first["action_id"]
        assert reports[-1] == (second["action_id"], "completed")
        assert device._armed and hand._teleop_active
    finally:
        stop_simulated_device(device)


def test_arm_requires_confirmation_and_both_hands(monkeypatch):
    device, hand, _ = plugin()
    device.accept_packet(packet(), "192.0.2.1", now=time.monotonic())
    with pytest.raises(ValueError, match="confirm_motion"):
        device.dispatch("arm", {"confirm_motion": "true"})
    hand._discover = lambda: {"right": "/dev/right"}
    with pytest.raises(RuntimeError, match="both L20 Lite hands"):
        device._arm()
    assert not hand._teleop_active


def test_armed_motion_is_step_limited_and_times_out(monkeypatch):
    device, hand, _ = plugin()
    clients = {side: FakeClient() for side in ("left", "right")}

    class FakeConnection:
        def __init__(self, config):
            self.client = clients[config["side"]]

        def get(self):
            return self.client

    monkeypatch.setattr(glove, "L20LiteConnection", FakeConnection)
    now = time.monotonic()
    device.accept_packet(packet([0] * 10, [0] * 10), "192.0.2.1", now=now)
    assert device._arm()["state"] == "armed"
    assert hand._teleop_active
    device.accept_packet(
        packet([0] * 10, [0] * 10, timestamp=2), "192.0.2.1", now=now + 0.1
    )
    wait_for(lambda: device._commands == 1 and device._apply_started is None)
    for client in clients.values():
        assert ("position", [238] * 10) in client.commands
    device._tick_watchdog(now + 0.5)
    assert device._cleanup_done.wait(1)
    assert not hand._teleop_active
    assert not device.dispatch("info", {})["armed"]
    event = device.dispatch("info", {})["last_watchdog_event"]
    assert event["sent_commands"] == 1
    assert all(age > device.watchdog_s * 1000 for age in event["side_age_ms"].values())
    measured_apply = event["timing"]["apply_ms"]
    device._timing["apply_ms"] = 999
    assert event["timing"]["apply_ms"] == measured_apply
    for client in clients.values():
        assert ("torque", [0] * 10) in client.commands
        assert ("close", None) in client.commands
    stop_simulated_device(device)


def test_direct_left_only_requires_poses_and_never_commands_right(monkeypatch):
    hand = FakeHand()
    hand._discover = lambda: {"left": "/dev/left"}
    publisher = FakePublisher()
    device = glove.GloveTeleopPlugin(
        hand, {"source_mode": "direct"}, "test", None, lambda _: publisher
    )
    device._publisher = publisher
    device._thread = object()  # Simulate an active direct reader; no port is opened.
    client = FakeClient()

    class FakeConnection:
        def __init__(self, config):
            assert config["side"] == "left"

        def get(self):
            return client

    monkeypatch.setattr(glove, "L20LiteConnection", FakeConnection)
    opened = {"side": "left", "pitch": [0.0] * 5, "yaw": [0.0] * 5}
    closed = {"side": "left", "pitch": [60.0] * 5, "yaw": [0.0] * 5}
    device.accept_direct_sample(opened)
    with pytest.raises(RuntimeError, match="calibrate_open must be captured"):
        device.dispatch("calibrate_fist", {"side": "left"})
    with pytest.raises(RuntimeError, match="calibrated"):
        device._arm()
    device.accept_direct_sample(closed)
    assert publisher.messages[-1]["raw_positions"]["left"][2] < RIGHT_OPEN_POSE[2]
    device.accept_direct_sample(opened)
    device.dispatch("calibrate_open", {"side": "left"})
    device.accept_direct_sample(closed)
    assert device.dispatch("calibrate_fist", {"side": "left"})["ready"]
    device.accept_direct_sample(opened)
    assert device._arm()["sides"] == ["left"]
    device.accept_direct_sample(closed)
    wait_for(lambda: device._commands == 1)
    assert device.dispatch("info", {})["sent_commands"] == 1
    assert client.commands.count(("position", [254] * 10)) == 1
    assert publisher.messages[-1]["glove_angles"] == {"left": [60.0] * 5}
    device.dispatch("disarm", {})
    assert ("torque", [0] * 10) in client.commands
    stop_simulated_device(device)


def test_arm_keeps_receiving_glove_frames_while_discovering_hand(monkeypatch):
    hand = FakeHand()
    entered = threading.Event()
    release = threading.Event()
    refreshed = threading.Event()

    def slow_discover():
        entered.set()
        assert release.wait(1)
        return {"left": "/dev/left"}

    hand._discover = slow_discover
    device = glove.GloveTeleopPlugin(
        hand, {"source_mode": "direct"}, "test", None, lambda _: FakePublisher()
    )
    device._thread = object()
    opened = {"side": "left", "pitch": [0.0] * 5, "yaw": [0.0] * 5}
    closed = {"side": "left", "pitch": [60.0] * 5, "yaw": [0.0] * 5}
    device.accept_direct_sample(opened)
    device.dispatch("calibrate_open", {"side": "left"})
    device.accept_direct_sample(closed)
    device.dispatch("calibrate_fist", {"side": "left"})

    client = FakeClient()
    monkeypatch.setattr(
        glove,
        "L20LiteConnection",
        lambda _: type("Connection", (), {"get": lambda self: client})(),
    )
    outcome = {}

    def arm():
        try:
            outcome["result"] = device._arm()
        except Exception as exc:
            outcome["error"] = exc

    def refresh():
        device.accept_direct_sample(opened)
        refreshed.set()

    worker = threading.Thread(target=arm)
    feeder = threading.Thread(target=refresh)
    worker.start()
    try:
        assert entered.wait(1)
        feeder.start()
        assert refreshed.wait(0.2), "hand discovery blocked fresh glove frames"
    finally:
        release.set()
        worker.join(1)
        if feeder.ident is not None:
            feeder.join(1)
    assert outcome.get("result", {}).get("sides") == ["left"], outcome.get("error")
    device.dispatch("disarm", {})
    stop_simulated_device(device)
