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


@pytest.fixture
def bundle_module(monkeypatch):
    """Load the real bundle and servo card with hardware imports replaced."""
    monkeypatch.delenv("BUMI_ENABLE_EXPERIMENTAL_PAYLOAD_MASS", raising=False)
    fake_rclpy = types.ModuleType("rclpy")
    fake_executors = types.ModuleType("rclpy.executors")
    fake_rclpy.executors = fake_executors
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)

    fake_device = types.ModuleType("device")
    fake_device.LocoPlugin = FakeLocoPlugin
    fake_device.SpeakerPlugin = FakeSpeakerPlugin
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


@pytest.mark.parametrize("opt_in, expected", [(None, False), ("0", False), ("true", False), ("1", True)])
def test_experimental_scale_requires_explicit_deployment_opt_in(bundle_module, monkeypatch, opt_in, expected):
    import yaml
    defaults = yaml.safe_load((BUMI / "config.yaml").read_text())
    assert defaults["plugins"]["payload_mass"]["enabled"] is False
    cfg = {"plugins": {"payload_mass": defaults["plugins"]["payload_mass"]}}
    calls = []
    class Payload:
        def __init__(self, config, namespace, executor, sampler, loco, *, feedback):
            calls.append(feedback)
        def get_tool(self):
            return {"name": "payload_mass", "type": "actuator"}
    monkeypatch.setitem(sys.modules, "payload_mass", types.SimpleNamespace(PayloadMassPlugin=Payload))
    if opt_in is not None:
        monkeypatch.setenv("BUMI_ENABLE_EXPERIMENTAL_PAYLOAD_MASS", opt_in)
    bundle = bundle_module.BumiDeviceBundle(cfg, "test", object(), FakeHighCtrl(), None)
    assert bool(calls) is expected
    assert any(t["name"] == "payload_mass" for t in bundle.get_all_tools()) is expected


def test_payload_registration_uses_native_feedback_and_loco_and_gates_response(bundle_module, monkeypatch):
    class State:
        def __init__(self, *args):
            self._node = types.SimpleNamespace()

        def get_tools(self):
            return []

    class Payload:
        def __init__(self, cfg, ns, executor, sampler, loco, *, feedback):
            self.sampler, self.loco = sampler, loco
            self.feedback = feedback
            self.published = []

        def get_tool(self):
            return {"name": "payload_mass", "type": "actuator"}

        def dispatch(self, action, args):
            assert action == "prepare"
            assert args["_await_response_publication"] is True
            return {"action_id": "test_id"}

        def response_published(self, action_id, delivered):
            self.published.append((action_id, delivered))

    monkeypatch.setattr(sys.modules["device"], "StatePlugin", State, raising=False)
    monkeypatch.setitem(sys.modules, "payload_mass", types.SimpleNamespace(PayloadMassPlugin=Payload))
    cfg = {"plugins": {"state": {"enabled": True}, "loco": {"enabled": True},
                       "payload_mass": {"enabled": True}}}
    bundle = bundle_module.BumiDeviceBundle(cfg, "test", object(), FakeHighCtrl(), None)
    state, loco, card = bundle._plugins
    assert not hasattr(state._node, "payload_sampler")
    assert card.feedback._sampler is card.sampler
    assert card.loco is loco
    assert bundle.dispatch("payload_mass", {"action": "prepare"}) == {"action_id": "test_id"}
    bundle.response_published("test_id", False)
    assert card.published == [("test_id", False)]


def test_canvas_stop_releases_its_worker_gate_only_after_mcp_response(bundle_module, monkeypatch):
    import io
    import json
    events = []
    def dispatch(name, args):
        assert name == "payload_mass" and args["action"] == "stop"
        events.append("dispatch")
        return {"state": "running", "action_id": "stop_test"}
    def published(action_id, delivered):
        assert action_id == "stop_test" and delivered
        assert b"stop_test" in handler.wfile.getvalue()
        events.append("published")
    monkeypatch.setattr(bundle_module, "_bundle", types.SimpleNamespace(dispatch=dispatch, response_published=published))
    handler = bundle_module.make_handler().__new__(bundle_module.make_handler())
    raw = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                      "params": {"name": "payload_mass", "arguments": {"action": "stop"}}}).encode()
    handler.path = "/mcp"
    handler.headers = {"Content-Length": str(len(raw))}
    handler.rfile, handler.wfile = io.BytesIO(raw), io.BytesIO()
    handler.send_response = lambda *a: None
    handler.send_header = lambda *a: None
    handler.end_headers = lambda: None
    handler.do_POST()
    assert events == ["dispatch", "published"]


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

