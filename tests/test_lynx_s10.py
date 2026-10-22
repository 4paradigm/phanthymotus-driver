from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml


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
