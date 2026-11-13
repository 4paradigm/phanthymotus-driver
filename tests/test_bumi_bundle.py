"""Bumi bundle plugin wiring, without ROS, SDK modules, or robot hardware."""

from __future__ import annotations

import ast
import inspect
import sys
import textwrap
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


class FakeMediaStatusPlugin:
    def __init__(self, plugin_config, namespace, executor, media_ctrl):
        self.media_ctrl = media_ctrl

    def get_tool(self):
        return {"name": "media_status", "type": "sensor"}

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
    fake_device.MediaStatusPlugin = FakeMediaStatusPlugin
    monkeypatch.setitem(sys.modules, "device", fake_device)
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

@pytest.mark.parametrize("media_ctrl", [None, object()])
def test_media_status_registers_even_without_a_media_controller(
        bundle_module, media_ctrl):
    """Unlike the cards that consume the media unit, this one reports it. The
    controller failing to initialise is precisely the state it exists to make
    visible, so gating registration on `media_ctrl is not None` would delete the
    card in the only situation an operator needs it: `tools/list` would carry no
    `media_status`, no `/media/status` topic would ever appear, and a
    disconnected unit would look exactly like a driver without the card."""
    config = _config(loco=False, loco_servo=False)
    config["plugins"]["media_status"] = {"enabled": True}

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), FakeHighCtrl(), media_ctrl)

    assert [tool["name"] for tool in bundle.get_all_tools()] == ["media_status"]
    assert bundle._plugins[0].media_ctrl is media_ctrl

def test_the_media_probe_does_not_depend_on_the_leg_controller(
        bundle_module, monkeypatch):
    """A failed HighController probe must not decide whether the media unit is
    even attempted. The two controllers reach different subsystems, and the
    media probe itself runs in a child process that never builds a
    HighController, so initialisation cannot depend on one. Nesting it under the
    leg probe made an unreachable leg controller report a working media unit as
    `controller_present: false` — the card blaming the wrong subsystem in
    exactly the situation it exists to describe."""
    import time as _time
    monkeypatch.setattr(_time, "sleep", lambda _seconds: None)

    high = types.SimpleNamespace(init=lambda: None)
    media = types.SimpleNamespace(init=lambda: None)
    fake_high = types.ModuleType("highcontrol_py")
    fake_high.HighController = types.SimpleNamespace(instance=lambda: high)
    fake_media = types.ModuleType("mediacontrol_py")
    fake_media.MediaController = types.SimpleNamespace(instance=lambda: media)
    monkeypatch.setitem(sys.modules, "highcontrol_py", fake_high)
    monkeypatch.setitem(sys.modules, "mediacontrol_py", fake_media)

    def probe(cmd, **_kwargs):
        """The legs are unreachable, the media unit answers."""
        if "highcontrol_py" in cmd[-1]:
            return types.SimpleNamespace(stdout="", stderr="no route to host")
        return types.SimpleNamespace(stdout="OK", stderr="")

    monkeypatch.setattr(bundle_module.subprocess, "run", probe)

    assert bundle_module._init_high_controller() is None
    assert bundle_module._init_media_controller() is media

def test_main_calls_both_initialisers_unconditionally(bundle_module):
    """The defect review round 2 reported lived at the call site, not inside the
    function: `MediaController` init sat under `if high_ctrl is not None`, so an
    unreachable leg controller skipped the media probe entirely. A test that
    calls `_init_media_controller()` on its own passes against that shape — I
    checked by re-nesting the call, and the whole suite stayed green — so it
    cannot keep the regression out.

    Assert the shape of `main` instead: both initialisers are plain statements of
    the function body, in order. Nesting either one under a condition is what
    makes the two subsystems depend on each other again.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(bundle_module.main)))

    calls = {}
    for statement in tree.body[0].body:
        if not isinstance(statement, ast.Assign):
            continue
        if not isinstance(statement.value, ast.Call):
            continue
        function = getattr(statement.value.func, "id", "")
        if function in {"_init_high_controller", "_init_media_controller"}:
            calls[getattr(statement.targets[0], "id", "")] = function

    assert calls == {
        "high_ctrl": "_init_high_controller",
        "media_ctrl": "_init_media_controller",
    }
