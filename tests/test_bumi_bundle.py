"""Bumi bundle plugin wiring, without ROS, SDK modules, or robot hardware."""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


class FakeHighCtrl:
    def publish_cmd(self, *_args):  # pragma: no cover - a safety tripwire
        raise AssertionError("bundle registration must not send motion commands")


class FakeLocoPlugin:
    def __init__(self, plugin_config, namespace, executor, high_ctrl):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor
        self.high_ctrl = high_ctrl
        self.servo = None
        self.sway = None
        self.started = False
        self.stopped = False

    def attach_servo(self, servo):
        self.servo = servo

    def attach_sway(self, sway):
        self.sway = sway

    def is_moving(self):
        return False

    def get_tools(self):
        return [{"name": "loco", "type": "actuator"}]

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def dispatch(self, action, _args):
        return {"state": action}


class FakeWaistSwayPlugin:
    def __init__(self, plugin_config, namespace, executor, high_ctrl,
                 loco_plugin=None):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor
        self.high_ctrl = high_ctrl
        self.loco = loco_plugin
        self.started = False
        self.stopped = False
        self.stop_order = None

    def get_tool(self):
        return {"name": "waist_sway", "type": "actuator"}

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def dispatch(self, action, _args):
        return {"state": action}


class FakeSpeakerPlugin:
    def __init__(self, plugin_config, namespace, executor, media_ctrl):
        self.media_ctrl = media_ctrl

    def get_tool(self):
        return {"name": "speaker", "type": "actuator"}

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def bundle_module(monkeypatch):
    """Load the real bundle and servo card with hardware imports replaced."""
    fake_rclpy = types.ModuleType("rclpy")
    fake_executors = types.ModuleType("rclpy.executors")
    fake_rclpy.executors = fake_executors
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)

    fake_device = types.ModuleType("device")
    fake_device.LocoPlugin = FakeLocoPlugin
    fake_device.SpeakerPlugin = FakeSpeakerPlugin
    monkeypatch.setitem(sys.modules, "device", fake_device)

    fake_waist_sway = types.ModuleType("waist_sway")
    fake_waist_sway.WaistSwayPlugin = FakeWaistSwayPlugin
    monkeypatch.setitem(sys.modules, "waist_sway", fake_waist_sway)
    monkeypatch.syspath_prepend(str(BUMI))
    previous_loco_servo = sys.modules.pop("loco_servo", None)

    # The host test runner is Python 3.9 while the Bumi image is Python 3.10+.
    # Postponing annotations lets this test execute the unchanged bundle source
    # without weakening the production runtime requirement.
    source = (BUMI / "main.py").read_text()
    if "from __future__ import annotations" not in source:
        source = "from __future__ import annotations\n" + source
    module = types.ModuleType("bumi_main_for_test")
    module.__file__ = str(BUMI / "main.py")
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    yield module
    sys.modules.pop("loco_servo", None)
    if previous_loco_servo is not None:
        sys.modules["loco_servo"] = previous_loco_servo


def _config(*, loco=True, loco_servo=True, servo_config=None, waist_sway=False):
    return {
        "plugins": {
            "loco": {"enabled": loco},
            "loco_servo": {
                "enabled": loco_servo,
                **(servo_config or {}),
            },
            "waist_sway": {"enabled": waist_sway},
        }
    }


def test_enabled_loco_servo_registers_and_reuses_loco(bundle_module):
    high_ctrl = FakeHighCtrl()
    executor = object()
    config = _config(servo_config={"dry_run": True, "expected_hz": 10})

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", executor, high_ctrl, media_ctrl=None)

    loco, servo = bundle._plugins
    assert isinstance(loco, FakeLocoPlugin)
    assert servo.PREFIX == "locoservo"
    assert servo._loco is loco
    assert loco.servo is servo
    assert servo._ns == "bumi_test"
    assert servo._executor is executor
    assert servo._high_ctrl is high_ctrl
    assert servo._dry_run is True
    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo"]


def test_loco_servo_enablement_is_not_coupled_to_loco_card(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(loco=False), "bumi_test", object(), FakeHighCtrl(), None)

    assert [tool["name"] for tool in bundle.get_all_tools()] == ["loco_servo"]
    assert bundle._plugins[0]._loco is None


@pytest.mark.parametrize(
    ("enabled", "high_ctrl"),
    [(False, FakeHighCtrl()), (True, None)],
)
def test_loco_servo_is_skipped_when_disabled_or_controller_missing(
        bundle_module, enabled, high_ctrl):
    bundle = bundle_module.BumiDeviceBundle(
        _config(loco=False, loco_servo=enabled),
        "bumi_test", object(), high_ctrl, None)

    assert bundle.get_all_tools() == []


def test_bundle_lifecycle_and_dispatch_include_loco_servo(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(), "bumi_test", object(), FakeHighCtrl(), None)
    loco, servo = bundle._plugins

    bundle.start_all()
    assert loco.started is True
    assert bundle.dispatch("loco_servo", {"action": "info"})["dry_run"] is True

    bundle.stop_all()
    assert loco.stopped is True
    assert servo._running is False


def test_existing_bumi_plugin_loading_is_unchanged(bundle_module):
    config = _config()
    config["plugins"]["speaker"] = {"enabled": True}

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), FakeHighCtrl(), media_ctrl=object())

    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo", "speaker"]


def test_waist_sway_registers_and_attaches_to_both_chassis_paths(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(waist_sway=True), "bumi_test", object(), FakeHighCtrl(), None)
    loco, servo, sway = bundle._plugins

    assert isinstance(sway, FakeWaistSwayPlugin)
    assert sway.loco is loco
    assert loco.sway is sway
    assert servo._sway is sway
    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo", "waist_sway"]
    assert bundle.dispatch("waist_sway", {"action": "info"}) == {"state": "info"}


def test_waist_sway_is_skipped_when_disabled_or_controller_missing(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(waist_sway=False), "bumi_test", object(), FakeHighCtrl(), None)
    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo"]


def test_stop_all_releases_the_waist_before_the_chassis_cards(bundle_module):
    stopped = []

    class RecordingSway(FakeWaistSwayPlugin):
        def stop(self):
            stopped.append("waist_sway")
            super().stop()

    import waist_sway as fake_waist_sway_module
    previous_cls = fake_waist_sway_module.WaistSwayPlugin
    fake_waist_sway_module.WaistSwayPlugin = RecordingSway
    try:
        bundle = bundle_module.BumiDeviceBundle(
            _config(waist_sway=True), "bumi_test", object(), FakeHighCtrl(), None)
        loco, _servo, _sway = bundle._plugins

        original_loco_stop = loco.stop

        def recording_loco_stop():
            stopped.append("loco")
            original_loco_stop()

        loco.stop = recording_loco_stop
        bundle.stop_all()
    finally:
        fake_waist_sway_module.WaistSwayPlugin = previous_cls

    assert stopped == ["waist_sway", "loco"]

