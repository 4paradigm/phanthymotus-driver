"""Behavioral tests: player speech, three rounds, movement and loss of vision."""
import importlib.util
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "robotera/q5_bundle/wooden_man.py"
spec = importlib.util.spec_from_file_location("wooden_man", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Rig:
    def __init__(self):
        self.now = 0.0
        self.words = []
        self.game = module.Game(self.words.append, clock=lambda: self.now)

    def step(self, seconds):
        self.now += seconds
        self.game.tick()

    def watch(self):
        self.step(3.5)
        assert self.game.phase == "watch"

    def pose(self, dx=0, dy=0, track=1):
        # Torso is 100px high, with distinct joint positions.
        points = [[100 + i * 3 + dx, 100 + dy, 0.99] for i in range(17)]
        for i in range(11, 17):
            points[i][1] += 100
        self.game.pose({"persons": [{"id": track, "keypoints": points}]})

    def start(self):
        self.game.text({"text": "游戏开始！", "is_final": True})
        self.step(5)
        self.watch()

    def still_round(self):
        for _ in range(31):
            self.pose()
            self.step(0.1)


def test_only_final_exact_start_utterance_and_five_second_countdown():
    r = Rig()
    for text in ("请不要游戏开始", "开始", {"text": "游戏开始", "is_final": False}):
        r.game.text(text)
    assert r.game.phase == "waiting" and not r.words
    r.game.text("游戏开始。")
    assert r.game.phase == "countdown" and len(r.words) == 1
    r.step(4.99)
    assert r.game.round == 0
    r.game.text("游戏开始")
    r.step(0.01)
    assert r.game.round == 1 and r.game.phase == "cue"


def test_three_rounds_and_no_automatic_fourth_round():
    r = Rig()
    r.start()
    for round_number in range(1, 4):
        assert r.game.round == round_number
        r.still_round()
        if round_number < 3:
            r.watch()
    assert r.game.phase == "won"
    assert r.words.count("一，二，三，木头人！") == 3
    r.step(100)
    assert r.game.round == 3
    r.game.text("游戏开始")
    assert r.game.phase == "countdown" and r.game.round == 0


@pytest.mark.parametrize("dx,dy", [(15, 0), (0, 15)])
def test_movement_ends_immediately_without_later_rounds(dx, dy):
    r = Rig()
    r.start()
    r.pose()
    for _ in range(3):
        r.step(0.1)
        r.pose(dx, dy)
    assert r.game.phase == "lost"
    assert r.words[-1] == "你动了，游戏结束。"
    r.step(100)
    assert r.game.round == 1


def test_small_jitter_and_one_spike_do_not_lose():
    r = Rig()
    r.start()
    r.pose()
    for dx in (2, -2, 20, 1, 2):
        r.step(0.1)
        r.pose(dx)
    assert r.game.phase == "watch"


@pytest.mark.parametrize("joint", [0, 9, 15])
def test_only_head_wrist_or_ankle_moving_also_counts(joint):
    r = Rig()
    r.start()
    r.pose()
    for _ in range(3):
        r.step(0.1)
        points = [[100 + i * 3, 100 + (100 if i >= 11 else 0), 0.99] for i in range(17)]
        points[joint][0] += 20
        r.game.pose({"persons": [{"id": 1, "keypoints": points}]})
    assert r.game.phase == "lost"


def test_slow_movement_cannot_drift_the_baseline():
    r = Rig()
    r.start()
    r.pose()
    for dx in range(1, 12):
        r.step(0.05)
        r.pose(dx)
    assert r.game.phase == "lost"


@pytest.mark.parametrize("payload", [None, {}, {"persons": []}, {"persons": [{}, {}]},
                                  {"persons": [{"keypoints": []}]},
                                  {"persons": [{"keypoints": [[float("nan"), 0, 1]] * 17}]}])
def test_missing_or_invalid_vision_never_passes(payload):
    r = Rig()
    r.start()
    r.game.pose(payload)
    r.step(1.6)
    assert r.game.phase == "error" and r.game.reason == "pose_unavailable"


def test_track_change_does_not_silently_replace_player():
    r = Rig()
    r.start()
    r.pose(track=1)
    for _ in range(17):
        r.step(0.1)
        r.pose(track=2)
    assert r.game.phase == "error"


def test_motion_allowed_during_cue_and_baseline_is_per_round():
    r = Rig()
    r.game.text("游戏开始")
    r.step(5)
    for _ in range(10):
        r.pose(100)
    assert r.game.phase == "cue"
    r.watch()
    r.still_round()
    r.watch()
    r.pose(100)
    assert r.game.phase == "watch" and r.game.motion_count == 0


def test_plugin_registration_lifecycle_and_input_validation_without_hardware():
    p = module.make_plugin({}, "q5", None, None)
    p.start()
    assert p.process is None
    tool = p.get_tool()
    assert tool["type"] == "processor"
    assert {v["format"] for v in tool["topic_in"]} == {"sensor/pose2d", "text/plain"}
    assert p.dispatch("start", {})["state"] == "error"
    assert p.dispatch("start", {"input_topics": ["/asr", "/pose"], "moving_frames": 1.2})["state"] == "error"
    assert p.dispatch("start", {"input_topics": ["/asr", "/pose"], "cue_seconds": float("nan")})["state"] == "error"
    assert p.dispatch("stop", {})["state"] == "idle"
    assert p.dispatch("info", {})["topic_out"][0]["topic"] == "/q5/wooden_man/speech"
    assert p.dispatch("reset", {})["state"] == "error"
