"""Explicit hand commands must stop the streaming owner before taking the bus."""

from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "linkerhand/l20_lite")]
from modbus_device import build_plugins


def plugins():
    config = yaml.safe_load((ROOT / "linkerhand/l20_lite/config.yaml").read_text())
    return build_plugins(config, "test", None)


def test_hand_motion_waits_for_teleop_disarm_before_dispatch(monkeypatch):
    hand, gesture, glove = plugins()
    order = []
    hand._teleop_active = True

    def disarm():
        order.append("stream_cleanup_finished")
        hand._teleop_active = False

    def dispatch(action, args):
        assert not hand._teleop_active
        order.append(action)
        return {"state": "accepted"}

    monkeypatch.setattr(glove, "_disarm", disarm)
    monkeypatch.setattr(hand._legacy, "dispatch", dispatch)
    hand.dispatch("set_positions", {"thumb_flex": 200, "confirm_motion": True})
    assert order == ["stream_cleanup_finished", "set_positions"]


def test_hand_and_gesture_stop_release_teleop_without_opening_uart(monkeypatch):
    hand, gesture, glove = plugins()
    cleanups = []

    def disarm():
        cleanups.append(True)
        hand._teleop_active = False

    monkeypatch.setattr(glove, "_disarm", disarm)
    for plugin in (hand, gesture):
        hand._teleop_active = True
        plugin.dispatch("stop", {})
        assert not hand._teleop_active
    assert len(cleanups) == 2
    assert hand._legacy.connection._client is None
