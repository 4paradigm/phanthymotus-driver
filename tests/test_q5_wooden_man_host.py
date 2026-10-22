"""Voice-only hosting: no robot actions, feedback topics or hardware readiness."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1] / "robotera/q5_bundle"
spec = importlib.util.spec_from_file_location("wooden_man", ROOT / "wooden_man.py")
game_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(game_module)


def pose(dx=0, joint=None):
    return {"image_size": [320, 320], "persons": [{"id": 1, "keypoints": [
        [100 + i * 3 + (dx if joint is None or i == joint else 0),
         100 + (100 if i >= 11 else 0), .99] for i in range(17)]}]}


class Host:
    def __init__(self, **config):
        self.now, self.words = 0., []
        self.game = game_module.Game(self.words.append, config, clock=lambda: self.now)

    def step(self, seconds):
        self.now += seconds
        self.game.tick()

    def ready(self, dx=0):
        for _ in range(17):
            self.game.pose(pose(dx))
            self.step(.2)

    def watch(self):
        self.game.text("游戏开始")
        self.ready()
        assert self.game.phase == "countdown"
        self.step(5)
        self.step(3.5)
        assert self.game.phase == "watch"

    def still(self):
        for _ in range(31):
            self.game.pose(pose())
            self.step(.1)


def test_waiting_is_silent_and_never_checks_players_before_start():
    h = Host()
    for _ in range(50):
        h.game.pose({"persons": []})
        h.step(.1)
    h.ready()
    h.game.preparation({"persons": []})
    assert h.game.phase == "waiting" and not h.words
    assert not h.game.ready and h.game.ready_frames == 0 and h.game.last_observation is None
    h.game.text("游戏开始")
    h.game.pose({"persons": []})
    assert h.words[-1] == "没有看清玩家，请一个人站在镜头前。"


@pytest.mark.parametrize("dx", [-90, 150])
def test_clear_player_at_either_edge_enters_countdown_without_arm_preparation(dx):
    h = Host()
    h.game.text("游戏开始")
    h.ready(dx)
    assert h.game.phase == "countdown"
    assert h.words[-1] == "游戏开始，五秒后开始。"
    assert not any("中央" in words or "手臂" in words for words in h.words)


def test_preparation_failure_ends_with_speech_only():
    h = Host(detect_fingers=True)
    h.game.text("游戏开始")
    h.ready()  # Body-only data cannot pass finger-mode readiness.
    assert h.game.phase == "preparing" and any("双手" in text for text in h.words)
    h.step(21)
    assert h.game.phase == "error" and h.game.reason == "preparation_timeout"


@pytest.mark.parametrize("legacy_flag", [False, True])
def test_all_three_rounds_and_replay_ignore_old_robot_action_settings(legacy_flag):
    h = Host(robot_actions=legacy_flag, motion_feedback_topic="/old/feedback", motion_timeout=.1)
    h.watch()
    for number in range(1, 4):
        h.still()
        if number < 3:
            assert h.game.phase == "intermission" and h.game.round == number
            h.step(3.01)
            assert h.game.phase == "cue" and h.game.round == number + 1
            h.step(3.5)
    assert h.game.phase == "won" and h.game.round == 3
    assert h.words.count("一，二，三，木头人！") == 3
    h.step(3.01)
    assert h.game.replay_prompted and h.words[-1] == "再挑战一次吗？"
    h.game.text("再来")
    assert h.game.phase == "preparing" and h.game.round == 0


def test_part_loss_keeps_highlight_and_prompts_replay_without_motion_feedback():
    h = Host()
    h.watch()
    h.game.pose(pose())
    for _ in range(3):
        h.step(.1)
        h.game.pose(pose(20, joint=9))
    assert h.game.phase == "lost" and h.game.moved_part == "left_arm"
    assert h.words[-1] == "你的左手臂动了，游戏结束。"
    assert 9 in h.game.highlight
    frame = h.game.frame
    h.game.pose(pose(100))
    assert h.game.frame is frame
    h.step(3.1)
    assert h.game.replay_prompted
    h.game.text("不玩了")
    assert h.game.phase == "waiting" and not h.game.moved_part
    words = list(h.words)
    h.game.pose({"persons": []})
    h.step(10)
    assert h.words == words


def test_reset_returns_to_silent_waiting():
    h = Host()
    h.game.text("游戏开始")
    h.game.reset()
    words = list(h.words)
    for _ in range(100):
        h.game.pose({"persons": []})
        h.step(.1)
    assert h.game.phase == "waiting" and h.words == words


def test_saved_motion_config_cannot_restore_action_ports_or_settings():
    plugin = game_module.WoodenManPlugin({"robot_actions": True}, "q5")
    result = plugin.dispatch("config", {"robot_actions": True, "motion_feedback_topic": "/old/feedback",
                                        "motion_timeout": .1, "robot_settle_seconds": .1})
    assert result["status"] == "configured" and plugin.process is None
    tool = plugin.get_tool()
    assert {p["topic"] for p in tool["topic_out"]} == {"/q5/wooden_man/speech", "/q5/wooden_man/status", "/q5/wooden_man/highlight"}
    assert plugin.configured_topics == {"asr_topic": "", "pose_topic": ""}
    assert "robot_actions" not in plugin.config
    assert "robot_actions" not in tool["configSchema"]["properties"]
    assert "motion_feedback_topic" not in tool["configSchema"]["properties"]
