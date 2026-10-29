"""Offline regression tests for the combined G1 bundle lifecycle."""
import ast
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
