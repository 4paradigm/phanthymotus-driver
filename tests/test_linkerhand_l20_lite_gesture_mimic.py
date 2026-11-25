import importlib
import threading
import time
from pathlib import Path

import pytest


@pytest.fixture
def module(monkeypatch):
    driver = Path(__file__).resolve().parents[1] / "linkerhand" / "l20_lite"
    monkeypatch.syspath_prepend(str(driver))
    return importlib.import_module("gesture_mimic")


class FakeMotion:
    def __init__(self):
        self.side = None
        self.cancel = threading.Event()
        self.calls = []
        self.finished = []
        self.block = set()

    def claim(self, side, speed_ratio):
        self.side = side
        self.speed_ratio = speed_ratio

    def move(self, label):
        self.calls.append(label)
        if label in self.block and self.cancel.wait(1):
            self.finished.append(("cancelled", label))
            raise InterruptedError()
        self.finished.append(("completed", label))

    def release(self):
        self.side = None


def frame(plugin, *people):
    stamp = time.time()
    plugin._frame({"timestamp": stamp, "persons": [
        {"id": track, "position": [x, 0], "hand_gestures": {side: label} if label else {}}
        for track, x, side, label in people
    ]})
    return stamp


def event(plugin, label, track, stamp, side="right", kind="gesture_start"):
    plugin._event({"event": kind, "gesture": label, "track": track, "hand": side, "t": stamp})


def settle(plugin):
    timeout = time.monotonic() + 1
    while plugin.active is not None or plugin.pending is not None:
        assert time.monotonic() < timeout
        plugin._tick()
        time.sleep(0.002)


def configured(module):
    motion = FakeMotion()
    plugin = module.GestureMimicPlugin(motion, "test", None)
    plugin.dispatch("config", {"instance_id": "test", "bound_side": "left", "recognition_side": "right"})
    return plugin, motion


def enable(plugin):
    plugin.dispatch("enable", {"instance_id": "test", "confirm_motion": True})


def test_mapping_and_enable_guard(module):
    plugin, motion = configured(module)
    assert len(module.ACTIONS) == 6
    assert module.ACTIONS["pointing"] == ("number", 1)
    assert plugin.get_tool()["topic_in"][0]["format"] == "data/json"
    with pytest.raises(ValueError, match="确认"):
        plugin.dispatch("enable", {"instance_id": "test", "confirm_motion": False})
    with pytest.raises(ValueError, match="Pose"):
        enable(plugin)
    assert motion.side is None


def test_multiplayer_center_and_same_gesture_is_not_repeated(module):
    plugin, motion = configured(module)
    frame(plugin, (1, 0.8, "right", None), (2, 0.1, "right", None))
    enable(plugin)
    stamp = frame(plugin, (1, 0.8, "right", "fist"), (2, 0.1, "right", "victory"))
    event(plugin, "fist", 1, stamp)
    event(plugin, "victory", 2, stamp)
    plugin.acquisition_until = 0
    plugin._tick()
    settle(plugin)
    assert plugin.operator == (2, "right")
    assert motion.calls == ["victory"]
    stamp = frame(plugin, (1, 0, "right", "fist"), (2, 0.7, "right", "victory"))
    event(plugin, "fist", 1, stamp)
    event(plugin, "ok", 2, stamp, "left")
    event(plugin, "victory", 2, stamp)
    plugin._tick()
    assert motion.calls == ["victory"]
    assert plugin.status["state"] == "holding"
    frame(plugin, (2, 0.7, "right", None))
    plugin._tick()
    assert plugin.holding == "victory"
    frame(plugin, (1, 0.8, "right", None))
    plugin.operator_seen -= module.OPERATOR_GRACE + 0.1
    plugin._tick()
    assert plugin.operator is None and plugin.holding == "victory" and motion.side == "left"
    stamp = frame(plugin, (1, 0.8, "right", "fist"))
    event(plugin, "fist", 1, stamp)
    frame(plugin, (1, 0.8, "right", "fist"))
    plugin._tick()
    settle(plugin)
    assert motion.calls == ["victory", "fist"]


def test_new_gesture_cancels_before_next_move(module):
    plugin, motion = configured(module)
    motion.block.add("open_palm")
    frame(plugin, (5, 0, "right", None))
    enable(plugin)
    stamp = frame(plugin, (5, 0, "right", "open_palm"))
    event(plugin, "open_palm", 5, stamp)
    plugin._tick()
    assert plugin.active["label"] == "open_palm"
    stamp = frame(plugin, (5, 0, "right", "fist"))
    event(plugin, "fist", 5, stamp)
    settle(plugin)
    assert motion.finished == [("cancelled", "open_palm"), ("completed", "fist")]
    assert plugin.holding == "fist"
    plugin.dispatch("stop", {"instance_id": "test"})
    assert motion.side is None and plugin.status["state"] == "stopped"


def test_pose_stream_loss_fault_releases_and_does_not_enable_itself(module):
    plugin, motion = configured(module)
    frame(plugin, (5, 0, "right", "fist"))
    enable(plugin)
    plugin.last_frame -= module.STREAM_TIMEOUT + 0.1
    plugin._tick()
    plugin._tick()
    assert plugin.status["reason_code"] == "POSE_STREAM_LOST"
    assert motion.side is None and not plugin.armed
    stamp = frame(plugin, (5, 0, "right", "fist"))
    event(plugin, "fist", 5, stamp)
    plugin._tick()
    assert not motion.calls


def test_detail_actions_resolve_configured_instance(module):
    plugin, motion = configured(module)
    frame(plugin, (5, 0, "right", None))
    with pytest.raises(ValueError, match="确认"):
        plugin.dispatch("enable", {"confirm_motion": False})
    assert plugin.dispatch("enable", {"confirm_motion": True})["armed"]
    assert plugin.dispatch("stop", {})["state"] == "stopped"
    assert plugin.dispatch("info", {})["bound_side"] == "left"
    assert not plugin.armed and motion.side is None
    assert plugin.dispatch("enable", {"confirm_motion": True})["armed"]
    with pytest.raises(ValueError, match="一张"):
        plugin.dispatch("config", {"instance_id": "other", "bound_side": "right", "recognition_side": "left"})
    plugin.dispatch("stop", {})
    assert plugin.dispatch("config", {"instance_id": "other", "bound_side": "right",
                                      "recognition_side": "left"})["bound_side"] == "right"
    assert plugin.instance == "other" and not plugin.armed and plugin.last_frame == 0


def test_detail_enable_without_configured_instance_gives_context(module):
    plugin = module.GestureMimicPlugin(FakeMotion(), "test", None)
    with pytest.raises(ValueError, match="画布上配置"):
        plugin.dispatch("enable", {"confirm_motion": True})
    assert not plugin.dispatch("stop", {})["armed"]


def test_either_hand_can_acquire_and_speed_uses_instance_settings(module):
    plugin, motion = configured(module)
    assert "recognition_side" not in plugin.get_tool()["configSchema"]["properties"]
    plugin.dispatch("config", {"instance_id": "test", "bound_side": "left", "speed_ratio": "1.5x"})
    frame(plugin, (5, 0, "left", None))
    enable(plugin)
    assert motion.side == "left" and motion.speed_ratio == "1.5x"
    stamp = frame(plugin, (5, 0, "left", "ok"))
    event(plugin, "ok", 5, stamp, "left")
    plugin._tick()
    settle(plugin)
    assert plugin.operator == (5, "left") and motion.calls == ["ok"]
    stamp = frame(plugin, (5, 0, "right", "fist"))
    event(plugin, "fist", 5, stamp, "right")
    plugin._tick()
    assert motion.calls == ["ok"]


def test_mimic_default_speed_doubles_manual_baselines(module):
    plugin, motion = configured(module)
    assert plugin.get_tool()["configSchema"]["properties"]["speed_ratio"]["default"] == "2.0x"
    frame(plugin, (5, 0, "left", None))
    enable(plugin)
    assert motion.speed_ratio == "2.0x"
    assert 60 * module.SPEED_RATIOS[motion.speed_ratio] == 120
    assert 85 * module.SPEED_RATIOS[motion.speed_ratio] == 170


@pytest.mark.parametrize("found", [{"right": "/dev/ttyUSB1"}, {"left": "/dev/ttyUSB1"}])
def test_release_after_serial_reenumeration(module, found):
    from types import SimpleNamespace

    writes = []
    invalidated = []

    def disconnected(side):
        raise OSError("adapter removed")

    stale = SimpleNamespace(get=disconnected, close=lambda: None,
                            invalidate=lambda: invalidated.append(True))
    client = SimpleNamespace(probe_positions=lambda side: None,
                             set_torques=lambda values: writes.append(values))
    replacement = SimpleNamespace(get=lambda side: client, close=lambda: None)
    hand = SimpleNamespace(_bus_lock=threading.RLock(), _mimic_side="right",
                           _discover=lambda: found, _connection=lambda **kwargs: replacement)
    motion = module.MimicMotion.__new__(module.MimicMotion)
    motion.hand, motion.side, motion.port = hand, "right", "/dev/ttyUSB0"
    motion.controller = SimpleNamespace(connection=stale)
    if "right" in found:
        motion.release()
        assert invalidated and writes == [[0] * module.JOINT_COUNT]
        assert hand._mimic_side is None and motion.side is None
    else:
        with pytest.raises(RuntimeError, match="绑定灵巧手"):
            motion.release()
        assert invalidated and not writes and hand._mimic_side == "right"


def test_handover_prefers_center_and_uses_an_earlier_start(module):
    plugin, motion = configured(module)
    frame(plugin, (4, 0.7, "left", None))
    enable(plugin)
    stamp = frame(plugin, (4, 0.7, "left", "open_palm"))
    event(plugin, "open_palm", 4, stamp, "left")
    plugin._tick()
    settle(plugin)
    previous_sighting = plugin.operator_seen

    # The person remains in view, but their original hand is gone.
    stamp = frame(plugin, (4, 0.7, "right", "fist"), (8, 0.1, "right", "victory"))
    event(plugin, "fist", 4, stamp)
    event(plugin, "victory", 8, stamp)
    frame(plugin, (4, 0.7, "right", "fist"), (8, 0.1, "right", "victory"))
    plugin._tick()
    assert plugin.operator == (4, "left") and plugin.operator_seen == previous_sighting
    assert motion.calls == ["open_palm"]

    # Holding a gesture can outlive the freshness window for new events.
    plugin.active_gestures[(8, "right")]["t"] -= 10
    plugin.operator_seen -= module.OPERATOR_GRACE + 0.1
    plugin._tick()
    settle(plugin)
    assert plugin.operator == (8, "right")
    assert motion.calls == ["open_palm", "victory"]


def test_visible_unclassified_hand_keeps_lock_and_ended_candidate_cannot_take_over(module):
    plugin, motion = configured(module)
    frame(plugin, (3, 0.7, "left", None))
    enable(plugin)
    stamp = frame(plugin, (3, 0.7, "left", "open_palm"))
    event(plugin, "open_palm", 3, stamp, "left")
    plugin._tick()
    settle(plugin)

    plugin.operator_seen -= module.OPERATOR_GRACE + 0.1
    stamp = time.time()
    plugin._frame({"timestamp": stamp, "persons": [
        {"id": 3, "position": [0.7, 0], "hands_seen": ["left"]},
        {"id": 9, "position": [0, 0], "hands_seen": ["right"], "hand_gestures": {"right": "fist"}},
    ]})
    event(plugin, "fist", 9, stamp)
    plugin._tick()
    assert plugin.operator == (3, "left")

    stamp = frame(plugin, (9, 0, "right", "fist"))
    event(plugin, "fist", 9, stamp, kind="gesture_end")
    frame(plugin, (9, 0, "right", "fist"))
    plugin.operator_seen -= module.OPERATOR_GRACE + 0.1
    plugin._tick()
    assert plugin.operator is None and plugin.holding == "open_palm"
    assert motion.calls == ["open_palm"]
