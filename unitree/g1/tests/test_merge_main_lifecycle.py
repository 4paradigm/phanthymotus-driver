"""Offline regression tests for the combined G1 bundle lifecycle."""
import ast
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


MAIN = Path(__file__).resolve().parents[1] / "main.py"


def load_bundle():
    tree = ast.parse(MAIN.read_text())
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == "G1DeviceBundle")
    namespace = {name: object for name in
                 ("AudioClient", "RpcProxy", "G1ArmActionClient", "SlamClient")}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(MAIN), "exec"), namespace)
    return namespace["G1DeviceBundle"]


def test_loco_servo_packaged_and_enabled():
    root = MAIN.parent
    assert "COPY loco_servo.py /work/loco_servo.py" in (root / "Dockerfile").read_text()
    import yaml
    config = yaml.safe_load((root / "config.yaml").read_text())
    assert config["plugins"]["loco_servo"]["enabled"] is True
    assert config["plugins"]["loco_servo"]["dry_run"] is True
    metadata = yaml.safe_load((root / "driver.yaml").read_text())
    assert {"name": "loco_servo", "type": "actuator"} in metadata["cards"]


def test_loco_servo_registered_independently_with_dedicated_transport(monkeypatch):
    calls = []
    transport = object()
    state = SimpleNamespace(get_fsm=lambda: {})
    class Servo:
        def __init__(self, *args): calls.append(args)
    class State:
        def __init__(self, *args): self.node = state
    monkeypatch.setitem(sys.modules, "loco_servo", SimpleNamespace(LocoServoPlugin=Servo))
    monkeypatch.setitem(sys.modules, "loco_servo_transport", SimpleNamespace(create_g1_loco_servo_transport=lambda: transport))
    monkeypatch.setitem(sys.modules, "device", SimpleNamespace(LocoStatePlugin=State))
    cfg = {"plugins": {"loco_servo": {"enabled": True}, "smart_motion": {"enabled": False}}}
    executor, client = object(), object()
    bundle = load_bundle()(cfg, "robot", executor, object(), client, object(), object())
    assert calls == [(cfg["plugins"]["loco_servo"], "robot", executor, transport, state.get_fsm)]
    assert len(bundle._plugins) == 2
    assert "COPY loco_servo_transport.py /work/loco_servo_transport.py" in (MAIN.parent / "Dockerfile").read_text()


def test_motion_and_capture_stop_before_feedback_and_producers():
    events = []

    class Plugin:
        def __init__(self, name, priority=None):
            self.PREFIX = name
            if priority is not None:
                self.STOP_PRIORITY = priority

        def stop(self):
            events.append(self.PREFIX)

    cls = load_bundle()
    bundle = cls.__new__(cls)
    bundle._plugins = [Plugin("camera"), Plugin("navigation_sensors"),
                       Plugin("loco", 0), Plugin("vision_capture"),
                       Plugin("loco_relative_move")]
    bundle._low_state = SimpleNamespace(stop=lambda: events.append("low_state"))
    bundle.stop_all()
    assert events == ["loco_relative_move", "loco", "vision_capture", "camera",
                      "navigation_sensors", "low_state"]


def test_failure_still_stops_remaining_owners_and_keeps_feedback():
    events = []

    class Plugin:
        def __init__(self, name, priority=None, fail=False):
            self.PREFIX, self.fail = name, fail
            if priority is not None:
                self.STOP_PRIORITY = priority

        def stop(self):
            events.append(self.PREFIX)
            if self.fail:
                raise RuntimeError("unconfirmed stop")

    cls = load_bundle()
    bundle = cls.__new__(cls)
    bundle._plugins = [Plugin("camera"), Plugin("loco", 0, True),
                       Plugin("navigation_sensors"), Plugin("vision_capture")]
    bundle._low_state = SimpleNamespace(stop=lambda: events.append("low_state"))
    with pytest.raises(RuntimeError, match="unconfirmed stop"):
        bundle.stop_all()
    assert events == ["loco", "vision_capture", "camera", "navigation_sensors"]


def test_servo_stop_failure_raises_even_without_low_state():
    bundle = load_bundle().__new__(load_bundle())
    class Failure:
        PREFIX = "locoservo"
        def stop(self): raise RuntimeError("pending")
    bundle._plugins = [Failure()]
    bundle._low_state = None
    with pytest.raises(RuntimeError, match="pending"):
        bundle.stop_all()
