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
        self.started = False
        self.stopped = False

    def attach_servo(self, servo):
        self.servo = servo

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


class FakeSpeakerPlugin:
    def __init__(self, plugin_config, namespace, executor, media_ctrl):
        self.media_ctrl = media_ctrl

    def get_tool(self):
        return {"name": "speaker", "type": "actuator"}

    def start(self):
        pass

    def stop(self):
        pass


class FakeVisionTfPlugin:
    def __init__(self, plugin_config, namespace, executor):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor

    def get_tool(self):
        return {"name": "vision_tf", "type": "processor"}

    def start(self):
        pass

    def stop(self):
        pass


class FakeVisualFacingPlugin:
    def __init__(self, plugin_config, namespace, executor, high_ctrl,
                 loco_plugin=None, loco_servo_plugin=None):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor
        self.high_ctrl = high_ctrl
        self.loco = loco_plugin
        self.servo = loco_servo_plugin

    def get_tool(self):
        return {"name": "visual_facing", "type": "actuator"}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, _args):
        return {"state": action}


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

    # vision_tf is a pure data card; stub it so the real main.py registration
    # guard can be exercised without rclpy in the host test process.
    fake_vision_tf = types.ModuleType("vision_tf_card")
    fake_vision_tf.VisionTfPlugin = FakeVisionTfPlugin
    monkeypatch.setitem(sys.modules, "vision_tf_card", fake_vision_tf)

    fake_facing = types.ModuleType("visual_facing_card")
    fake_facing.VisualFacingPlugin = FakeVisualFacingPlugin
    monkeypatch.setitem(sys.modules, "visual_facing_card", fake_facing)

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


def _config(*, loco=True, loco_servo=True, servo_config=None):
    return {
        "plugins": {
            "loco": {"enabled": loco},
            "loco_servo": {
                "enabled": loco_servo,
                **(servo_config or {}),
            },
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


def test_tf_cards_are_no_longer_registered_as_standalone_plugins(bundle_module):
    """tf is an implicit capability of the odometry card, not canvas cards.

    Even if an old deployment still carries `plugins.vision_tf` / `tf_tree`
    keys, the bundle must not expose them: their logic now lives in the
    vision-odometry card's embedded tree.
    """
    config = _config(loco=False, loco_servo=False)
    config["plugins"]["vision_tf"] = {"enabled": True}
    config["plugins"]["tf_tree"] = {"enabled": True}

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), high_ctrl=None, media_ctrl=None)

    assert bundle.get_all_tools() == []


def test_embedded_tf_config_is_default_off(bundle_module):
    """The odometry card's tf switch is absent/false by default in config.yaml.

    Enabling the odometry card must never silently start publishing /tf: the
    operator has to opt in with plugins.visual_odometry.tf.enabled.
    """
    import yaml
    config = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
    tf_cfg = config["plugins"]["visual_odometry"].get("tf")
    assert isinstance(tf_cfg, dict)
    assert tf_cfg.get("enabled", False) is False



class FakeTfTreePlugin:
    def __init__(self, plugin_config, namespace, executor):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor

    def get_tool(self):
        return {"name": "tf_tree", "type": "processor"}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, _args):
        return {"state": action}


def test_bundle_source_has_no_standalone_tf_registration(bundle_module):
    """Guard the source itself: main.py must not mount either tf card.

    A config-level test alone would miss a reintroduced import/append, so pin
    the source text. The tf-tree *node* class is still imported by
    visual_odometry.py (as an embedded capability); only the two card plugins
    are retired.
    """
    import inspect
    source = inspect.getsource(bundle_module.BumiDeviceBundle.__init__)
    assert "VisionTfPlugin" not in source
    assert "TfTreePlugin" not in source
    assert "vision_tf_card" not in source


def test_visual_facing_registers_with_both_chassis_peers_when_enabled(bundle_module):
    config = _config()
    config["plugins"]["visual_facing"] = {"enabled": True}
    high_ctrl = FakeHighCtrl()

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), high_ctrl, None)
    loco, servo, facing = bundle._plugins

    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo", "visual_facing"]
    assert isinstance(facing, FakeVisualFacingPlugin)
    assert facing.high_ctrl is high_ctrl
    assert facing.loco is loco and facing.servo is servo


def test_visual_facing_is_skipped_when_disabled(bundle_module):
    config = _config()
    config["plugins"]["visual_facing"] = {"enabled": False}
    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), FakeHighCtrl(), None)
    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo"]


def test_visual_facing_cannot_register_without_a_controller(bundle_module):
    config = _config(loco=False, loco_servo=False)
    config["plugins"]["visual_facing"] = {"enabled": True}
    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), None, None)
    assert bundle.get_all_tools() == []

