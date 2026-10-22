from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

from deep_robotics.lynx_s10.basic_server import HEADER, MAX_ASDU, SYNC, StreamDecoder


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
    assert any(plugin.__class__.__name__ == "S10NavigationPlugin" for plugin in plugins)
    assert config["mcp_port"] == driver["port"] == 15724
    assert config["mcp_port"] != yaml.safe_load((ROOT / "agibot/AimDK_X2/driver.yaml").read_text())["port"]
    assert "15724" in (DRIVER / "Dockerfile").read_text()


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
