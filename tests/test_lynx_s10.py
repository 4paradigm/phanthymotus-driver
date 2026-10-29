from __future__ import annotations

import importlib.util
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from deep_robotics.lynx_s10.basic_server import (
    HEADER,
    MAX_ASDU,
    SYNC,
    BasicServerClient,
    StreamDecoder,
    decode_frame,
    encode_frame,
)


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics" / "lynx_s10"


def _load_device(monkeypatch):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("lynx_s10_device_for_test", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeNative:
    def __init__(self, config):
        self.config = config
        self.calls = []
        self.reports = {}

    def snapshot(self):
        return {"reports": self.reports, "connected": True, "last_error": None}

    def request(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return {"state": "sent"}

    def send(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return {"state": "sent"}


def test_pro_config_registers_navigation_and_matches_marketplace_cards(monkeypatch):
    device = _load_device(monkeypatch)
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    driver = yaml.safe_load((DRIVER / "driver.yaml").read_text())
    monkeypatch.setattr(device, "BasicServerClient", FakeNative)

    plugins = device.build_plugins(config)
    card_names = {card["name"] for card in driver["cards"]}

    assert config["model_variant"] == "pro"
    assert "navigation" in card_names
    assert "loco" in card_names and "motion" not in card_names
    assert {"imu", "loco_state", "joints", "joint_state", "battery"} <= card_names
    assert not card_names.intersection({"motion_info", "native_basic_status", "native_device_status", "hard_estop", "charge_status"})
    assert any(plugin.__class__.__name__ == "S10NavigationPlugin" for plugin in plugins)
    assert config["mcp_port"] == driver["port"] == 15724
    assert config["mcp_port"] != yaml.safe_load((ROOT / "agibot/AimDK_X2/driver.yaml").read_text())["port"]
    assert "15724" in (DRIVER / "Dockerfile").read_text()


def test_standard_state_cards_read_native_reports(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    native.reports = {
        "1048577:15728640": {"PatrolDevice": {"Items": {
            "MotionStatus": {"AccX": 1, "Roll": 2},
            "MotorStatus": {"Joint": [{"q": 0.25}], "Motor": [{"q": 0.25, "dq": 0.1}]},
        }}},
        "1048578:15728640": {"PatrolDevice": {"Items": {"BatteryList": [{"SOC": 88}]}}},
        "1048676:15728640": {"PatrolDevice": {"Items": {"BasicStatus": {"Gait": 3, "HES": 0}}}},
    }
    plugin = device.S10StatePlugin(native)
    assert plugin.dispatch("imu", {"_tool_name": "imu"})["acceleration_m_s2"][0] == 1
    skeleton = plugin.dispatch("joints", {"_tool_name": "joints"})
    assert skeleton["format"] == "sensor/skeleton"
    assert skeleton["available"] is False
    assert len(skeleton["joints"]) == 16
    assert skeleton["joints"][0] == {
        "idx": 0, "name": "fl_hipx_joint", "q": 0.25,
        "dq": None, "tau": None, "temperature": None,
    }
    assert plugin.dispatch("joint_state", {"_tool_name": "joint_state"})["Motor"][0]["dq"] == 0.1
    assert plugin.dispatch("loco_state", {"_tool_name": "loco_state"})["basic_state"] == {"Gait": 3, "HES": 0}
    assert plugin.dispatch("battery", {"_tool_name": "battery"}) == {"batteries": [{"SOC": 88}]}


def test_model_resource_returns_official_urdf(monkeypatch):
    device = _load_device(monkeypatch)
    result = device.S10ModelPlugin().dispatch("model", {})
    root = ET.fromstring(result["urdf"])
    assert root.tag == "robot" and root.get("name") == "S10"
    assert root.find("joint[@name='fl_hipx_joint']") is not None


def test_skeleton_maps_all_s10_motor_fields_and_urdf_order(monkeypatch):
    device = _load_device(monkeypatch)
    raw = [{"q": idx / 10, "dq": 1, "tau": 2, "temperature": [30]} for idx in range(16)]
    result = device._skeleton_payload({"MotorStatus": {"Joint": raw}, "MotionStatus": {"Roll": 0, "Pitch": 0, "Yaw": 0}})
    assert result["available"] is True
    assert [joint["name"] for joint in result["joints"]] == list(device.JOINT_NAMES)
    assert result["joints"][-1]["q"] == 1.5
    assert result["joints"][0]["temperature"] == [30]
    assert result["imu_quat"] == [1, 0, 0, 0]


def test_navigation_actions_use_s10_pro_native_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({"navigation_task_timeout": 45})
    plugin = device.S10NavigationPlugin(native)

    assert plugin.dispatch("goal", {"x": 1.2, "y": -0.4, "yaw": 0.5}) == {"state": "sent"}
    args, kwargs = native.calls[-1]
    assert args[:2] == (0x00100003, 0x00100002)
    assert args[2]["Item"][0]["PosX"] == 1.2
    assert kwargs["response_timeout"] == 45

    plugin.dispatch("cancel", {})
    assert native.calls[-1][0][:2] == (0x00100003, 0x00100004)


def test_loco_card_uses_shared_action_names_and_native_velocity_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    definition = plugin.get_tool()
    assert definition["name"] == "loco"
    assert {"move", "stop_move", "stand_up", "stand_down", "damp"} <= set(definition["inputSchema"]["x-action-params"])
    try:
        result = plugin.dispatch("move", {"vx": 0.5, "vy": 0.1, "vyaw": 0.2, "duration": 1})
        assert result["state"] == "streaming"
        assert native.calls[-1][0][1:] == (0x00110002, {"X": 0.5, "Y": 0.1, "Z": 0.0,
                                                        "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.2})
        result = plugin.dispatch("stop_move", {})
        assert result["state"] == "stopped"
        assert native.calls[-2][0][1] == 0x00100002
        assert native.calls[-1][0][1] == 0x00110002
    finally:
        plugin.stop()


def test_loco_timed_move_expires_with_zero_velocity(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    plugin.start()
    try:
        plugin.dispatch("move", {"vx": 0.2, "duration": 0.02})
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if len(native.calls) >= 2 and native.calls[-1][0][2]["X"] == 0:
                break
            time.sleep(0.01)
        assert len(native.calls) >= 2
        assert native.calls[-1][0][1] == 0x00110002
        assert native.calls[-1][0][2] == {
            "X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0,
        }
    finally:
        plugin.stop()


def test_actuator_lifecycle_never_starts_charging_or_sends_idle_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({"navigation_task_timeout": 45})
    plugins = [
        device.S10MotionPlugin(native),
        device.S10DevicePlugin(native),
        device.S10ChargePlugin(native),
        device.S10NavigationPlugin(native),
    ]

    for plugin in plugins:
        assert plugin.dispatch("start", {}) == {"state": "ready"}
        assert plugin.dispatch("stop", {}) == {"state": "idle"}
        assert plugin.dispatch("info", {}) == {"state": "ready"}
    assert native.calls == []

    charging = plugins[2]
    charging.dispatch("start_charging", {})
    assert native.calls[-1][0] == (0x00100004, 0x00100002, {"Charge": 1})
    charging.dispatch("stop_charging", {})
    assert native.calls[-1][0] == (0x00100004, 0x00100002, {"Charge": 0})


def test_control_schema_exposes_supported_navigation_and_light_options(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    nav_params = device.S10NavigationPlugin(native).get_tool()["inputSchema"]["x-action-params"]["goal"]["params"]
    localize_params = device.S10NavigationPlugin(native).get_tool()["inputSchema"]["x-action-params"]["localize"]["params"]
    light_params = device.S10DevicePlugin(native).get_tool()["inputSchema"]["x-action-params"]["lights"]["params"]

    assert set(nav_params) == {"x", "y", "z", "yaw", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous"}
    assert set(localize_params) == {"x", "y", "z", "yaw"}
    assert {"head_cycle", "tail_cycle"} <= set(light_params)


def test_malformed_final_fragment_is_discarded_without_killing_decoder():
    decoder = StreamDecoder()
    first = b"{" + b" " * (MAX_ASDU - 1)
    second = b"not-json"
    frame1 = HEADER.pack(SYNC, len(first), 9, 1, 0, 1) + first
    frame2 = HEADER.pack(SYNC, len(second), 9, 1, 1, 1) + second

    assert decoder.feed(frame1) == []
    assert decoder.feed(frame2) == []
    assert 9 not in decoder.fragments


def test_basic_server_uses_tcp_for_request_response(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.ready = threading.Event()
            self.response = None

        def settimeout(self, _timeout):
            pass

        def sendall(self, frame):
            message_id, request = decode_frame(frame)
            body = request["PatrolDevice"]
            self.response = encode_frame(message_id, body["Type"], body["Command"], {"ErrorCode": 0})
            self.ready.set()

        def recv(self, _size):
            assert self.ready.wait(1)
            response, self.response = self.response, None
            return response or b""

        def close(self):
            self.ready.set()

    fake_socket = FakeSocket()
    monkeypatch.setattr(
        "deep_robotics.lynx_s10.basic_server.socket.create_connection",
        lambda address, timeout: fake_socket,
    )
    client = BasicServerClient({"basic_server": {
        "host": "127.0.0.1", "tcp_port": 30003, "timeout": 1,
    }})
    try:
        response = client.request(0x00100064, 0x00000005)
        assert response["PatrolDevice"]["Items"] == {"ErrorCode": 0}
        assert client.snapshot()["transport"] == "TCP"
        assert client.snapshot()["reports"]["1048676:5"] == response
    finally:
        client.close()
