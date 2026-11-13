"""Rules for Bumi foot rock-paper-scissors. This module never controls hardware.

The robot's choice is committed before the player's spoken/text choice arrives.
requested_pose is a name for a future verified motion backend, not an SDK command.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import secrets
from typing import Callable


class Choice(str, Enum):
    ROCK = "rock"
    PAPER = "paper"
    SCISSORS = "scissors"


LABELS = {Choice.ROCK: "石头", Choice.PAPER: "布", Choice.SCISSORS: "剪刀"}
POSES = {
    Choice.ROCK: "feet_together",
    Choice.PAPER: "feet_apart",
    Choice.SCISSORS: "feet_staggered",
}
BEATS = {Choice.ROCK: Choice.SCISSORS, Choice.PAPER: Choice.ROCK, Choice.SCISSORS: Choice.PAPER}


def parse_player_choice(text: str) -> Choice:
    """Accept one unambiguous report; reject negation, questions and multiple picks."""
    if not isinstance(text, str):
        raise ValueError("请清楚说出：我出石头、我出剪刀或我出布。")
    value = "".join(text.split()).strip("，。！!.,")
    for prefix in ("我出了", "我出", "出"):
        if value.startswith(prefix):
            value = value[len(prefix):]
            break
    for choice, label in LABELS.items():
        if value in (label, choice.value):
            return choice
    raise ValueError("未识别到唯一出招，请说：我出石头、我出剪刀或我出布。")


def outcome(robot: Choice, player: Choice) -> str:
    robot, player = Choice(robot), Choice(player)
    if robot == player:
        return "draw"
    return "robot_win" if BEATS[robot] == player else "player_win"


@dataclass(frozen=True)
class RoundResult:
    round_number: int
    robot: Choice
    player: Choice
    outcome: str
    requested_pose: str

    def spoken_result(self) -> str:
        ending = {"draw": "这一局平局。", "robot_win": "这一局我赢了。", "player_win": "这一局你赢了。"}[self.outcome]
        return f"我出{LABELS[self.robot]}，你出{LABELS[self.player]}，{ending}"


class Game:
    def __init__(self, chooser: Callable | None = None):
        self._chooser = chooser or secrets.choice
        self._pending: Choice | None = None
        self.round_number = 0
        self.stopped = False
        self.scores = {"robot_win": 0, "player_win": 0, "draw": 0}

    def begin_round(self) -> dict:
        if self.stopped:
            raise RuntimeError("游戏已停止，请新建一局游戏。")
        if self._pending is not None:
            raise RuntimeError("当前一局仍在等待玩家出招。")
        self._pending = Choice(self._chooser(tuple(Choice)))
        self.round_number += 1
        # Do not reveal the robot's selection before the player reports a choice.
        return {"round_number": self.round_number, "phase": "awaiting_player",
                "prompt": "准备好，三、二、一，出！请说你出的是什么。"}

    def receive_player(self, text: str) -> RoundResult:
        if self.stopped or self._pending is None:
            raise RuntimeError("当前没有等待出招的对局。")
        player = parse_player_choice(text)
        robot = self._pending
        result = RoundResult(self.round_number, robot, player, outcome(robot, player), POSES[robot])
        self._pending = None
        self.scores[result.outcome] += 1
        return result

    def stop(self):
        self.stopped = True
        self._pending = None


if __name__ == "__main__":
    # A local rules demonstration, deliberately without ROS, audio, or SDK I/O.
    game = Game()
    print(game.begin_round()["prompt"])
    try:
        print(game.receive_player(input()).spoken_result())
        print("规则演示完成；本程序没有让机器人执行任何动作。")
    except (ValueError, RuntimeError) as error:
        print(str(error))
