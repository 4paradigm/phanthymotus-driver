"""Dialogue, opt-in multiplayer registration and independent motion verdicts."""
import importlib.util
import copy
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1] / "robotera/q5_bundle"
spec = importlib.util.spec_from_file_location("wooden_man", ROOT / "wooden_man.py")
game_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(game_module)


def person(identity=1, x=180, raised=False, seated=False, joint=None, delta=0):
    points = [[x, 70, .99] for _ in range(17)]
    for i, px, py in ((5,-20,110), (6,20,110), (7,-30,155), (8,30,155),
                       (9,-30,200), (10,30,200), (11,-15,210), (12,15,210),
                       (13,-15,290), (14,15,290), (15,-15,370), (16,15,370)):
        points[i] = [x+px, py, .99]
    if raised:
        points[9][1] = 65
    if seated:
        points[13][:2], points[14][:2] = [x+80,210], [x+105,210]
        points[15][:2], points[16][:2] = [x+80,290], [x+105,290]
    if joint is not None:
        points[joint][0] += delta
    return {"id": identity, "keypoints": points}


def frame(*people):
    return {"image_size": [900,500], "persons": list(people)}


def with_hands(payload, jitter=0, moved=None, delta=0, missing=None):
    payload = copy.deepcopy(payload)
    names = [f"body_{i}" for i in range(17)]
    for side in ("left", "right"):
        names.extend(f"{side}_hand_{name}" for name in game_module.HAND_NAMES)
    payload["keypoint_names"] = names
    for p in payload["persons"]:
        p["keypoints"] = p["keypoints"][:17]
        for side, wrist in (("left",9), ("right",10)):
            x, y = p["keypoints"][wrist][:2]
            hand = [[x+jitter,y,.99]]
            for finger in range(5):
                hand.extend([[x+(finger-2)*8+jitter,y-20-joint*9,.99] for joint in range(4)])
            if p["id"] == moved and side == "left":
                hand[8][0] += delta
            if p["id"] == missing:
                hand = [[0,0,0]]*21
            p["keypoints"].extend(hand)
    return payload


class Host:
    def __init__(self, **config):
        self.now, self.words = 0., []
        self.game = game_module.Game(self.words.append, config, clock=lambda: self.now)

    def step(self, seconds):
        self.now += seconds
        self.game.tick()

    def reply(self, text, background=True):
        self.step(max(0, self.game.listen_after-self.now)+.01)
        self.game.text({"text": text, "background": background})

    def feed(self, payload, seconds=1.2):
        for _ in range(round(seconds/.1)):
            self.game.pose(payload)
            self.step(.1)

    def enrollment(self, count=1):
        self.game.text("小范小范，我们来玩一二三木头人游戏吧！")
        assert self.game.phase == "rules_question"
        self.reply("不需要，直接开始")
        assert self.game.phase == "player_count"
        self.reply(f"{count}个人")
        assert self.game.phase == "registering"

    def watch(self, count=1, background=()):
        self.enrollment(count)
        raised = [person(i+1, x=180+i*260, raised=True) for i in range(count)]
        self.feed(frame(*raised, *background), self.game.listen_after-self.now+1.2)
        assert self.game.phase == "preparing"
        still = [person(i+1, x=180+i*260) for i in range(count)]
        self.feed(frame(*still, *background), self.game.listen_after-self.now+1.2)
        assert self.game.phase == "countdown"
        self.step(5)
        self.step(3.5)
        assert self.game.phase == "watch"
        return still


@pytest.mark.parametrize("text", ["小范小范，我们来玩木头人游戏吧", "我们来玩一二三木头人游戏", "木头人游戏", "游戏开始"])
def test_invitation_questions_before_any_visibility_check(text):
    h = Host()
    h.feed(frame())
    assert not h.words
    h.game.text(text)
    h.feed(frame())
    assert h.game.phase == "rules_question" and h.words == ["好啊好啊，需要我介绍规则吗？"]


@pytest.mark.parametrize("text", ["不要玩木头人游戏", "我不想玩木头人", "木头人游戏是什么", "怎么玩木头人游戏", "介绍一下木头人规则", "别玩木头人", "开始"])
def test_negative_or_information_request_never_starts(text):
    h = Host()
    h.game.text(text)
    assert h.game.phase == "waiting" and not h.words


def test_background_answers_only_after_speech_and_rules_intro_finishes_before_count():
    h = Host()
    h.game.text({"text": "来玩木头人游戏", "background": True})
    h.game.text({"text": "来玩木头人游戏", "is_final": False})
    assert h.game.phase == "waiting"
    h.game.text("来玩木头人游戏")
    h.game.text("不用")
    h.game.text(h.words[-1])
    assert h.game.phase == "rules_question"
    h.reply("需要你介绍一下规则")
    assert h.game.phase == "rules_intro"
    h.step(h.game.listen_after-h.now+.01)
    assert h.game.phase == "player_count" and any("任何一位" in text for text in h.words)


@pytest.mark.parametrize("text,count", [("一个人",1), ("我一个人",1), ("只有我",1), ("我们有两个人一起玩",2), ("三个人",3), ("2个人",2)])
def test_player_count(text, count):
    h = Host()
    h.game.text("来玩木头人游戏")
    h.reply("不用介绍")
    h.reply(text)
    assert h.game.phase == "registering" and h.game.expected_players == count


@pytest.mark.parametrize("text", ["零个人", "0个人", "六个人", "两个或者三个", "两个人不玩", "随便"])
def test_invalid_count_is_not_guessed(text):
    h = Host()
    h.game.text("来玩木头人游戏")
    h.reply("不用介绍")
    h.reply(text)
    assert h.game.phase == "player_count" and not h.game.players


def test_timeout_cancellation_and_timed_duplicate_asr():
    h = Host()
    h.game.text("来玩木头人游戏")
    h.step(h.game.deadline+.1)
    assert h.game.phase == "waiting"
    words = list(h.words)
    h.feed(frame(), 30)
    assert words == h.words
    h.game.text("来玩木头人游戏")
    h.step(h.game.listen_after-h.now+.01)
    answer = {"text": "不用", "audio_start_ts": 123, "audio_end_ts": 124}
    h.game.text(answer)
    assert h.game.phase == "player_count"
    h.step(h.game.listen_after-h.now+.01)
    words = list(h.words)
    h.game.text(answer)
    assert words == h.words
    h.game.text("取消游戏")
    assert h.game.phase == "waiting"


def test_registration_excludes_seated_and_unraised_bystanders_requires_exact_count():
    h = Host()
    h.enrollment(2)
    h.feed(frame(person(1, raised=True), person(2, x=440), person(9, x=700, raised=True, seated=True)), 14)
    assert h.game.phase == "registering" and h.game.registration_candidates == 1
    h.feed(frame(person(1, raised=True), person(2, x=440, raised=True), person(9, x=700, raised=True)), 2)
    assert h.game.phase == "registering" and not h.game.players
    h.feed(frame(person(2, x=440, raised=True), person(9, x=700, seated=True), person(1, raised=True)), max(2, h.game.listen_after-h.now+1.2))
    assert [p["track_id"] for p in h.game.players] == [1,2]


def test_registration_needs_continuous_identity_and_fresh_frames():
    h = Host()
    h.enrollment()
    h.step(h.game.listen_after-h.now+.01)
    h.feed(frame(person(1, raised=True)), .6)
    h.feed(frame(person(2, raised=True)), .6)
    assert h.game.phase == "registering"
    h.step(2)
    h.game.pose(frame(person(2, raised=True)))
    assert h.game.phase == "registering"
    h.feed(frame(person(2, raised=True)), 1.2)
    assert h.game.phase == "preparing"


@pytest.mark.parametrize("x", [40,810])
def test_off_center_player_is_valid(x):
    h = Host()
    h.enrollment()
    h.feed(frame(person(x=x, raised=True)), 14)
    assert h.game.phase == "preparing"
    h.feed(frame(person(x=x)), h.game.listen_after-h.now+1.2)
    assert h.game.phase == "countdown"


@pytest.mark.parametrize("legacy_flag", [False,True])
def test_three_rounds_bystanders_reordered_detections_and_replay(legacy_flag):
    h = Host(robot_actions=legacy_flag)
    still = h.watch(2, (person(99, x=720, seated=True),))
    for number in range(1,4):
        for index in range(31):
            h.game.pose(frame(person(99, x=700+index*3), *reversed(still)))
            h.step(.1)
        if number < 3:
            assert h.game.phase == "intermission" and h.game.round == number
            h.step(h.game.deadline-h.now+.01)
            h.step(3.5)
    assert h.game.phase == "won" and h.game.round == 3
    assert h.words.count("一，二，三，木头人！") == 3
    h.step(h.game.prompt_at-h.now+.01)
    h.reply("再来")
    assert h.game.phase == "rules_question" and not h.game.players


def test_second_player_loss_number_highlight_and_frozen_frame():
    h = Host()
    still = h.watch(2)
    h.game.pose(frame(*still))
    moved = frame(person(2, x=440, joint=9, delta=20), person(9, x=750), still[0])
    h.feed(moved, .4)
    assert h.game.phase == "lost" and h.game.moved_player == 2
    assert h.game.moved_track_id == 2 and h.game.moved_part == "left_arm"
    assert h.words[-1] == "2号玩家，你的左手臂动了，游戏结束。"
    h.game.pose(frame(*still))
    assert h.game.frame is moved and 9 in h.game.highlight
    assert game_module.render_highlight(moved, h.game.status()).startswith(b"\xff\xd8")


def test_crouching_player_still_counts_as_motion():
    h = Host()
    still = h.watch()
    h.game.pose(frame(*still))
    h.feed(frame(person(seated=True)), .4)
    assert h.game.phase == "lost"


def test_occlusion_pauses_restarts_same_round_and_never_adds_bystander():
    h = Host()
    still = h.watch(2)
    h.feed(frame(*still), .5)
    h.feed(frame(still[0], person(99, x=440)), 1.8)
    assert h.game.phase == "paused" and h.game.round == 1
    assert [p["track_id"] for p in h.game.players] == [1,2]
    h.feed(frame(*still), h.game.listen_after-h.now+1.2)
    assert h.game.phase == "intermission" and h.game.round == 1
    h.step(h.game.deadline-h.now+.01)
    assert h.game.phase == "cue" and h.game.round == 1
    h.step(3.5)
    h.feed(frame(*still), .8)
    assert h.game.phase == "watch" and not h.game.result


def test_changed_id_requires_reregistration_after_recovery_timeout():
    h = Host()
    h.watch()
    h.feed(frame(person(2)), 2)
    assert h.game.phase == "paused"
    h.feed(frame(person(2)), 50)
    assert h.game.phase == "error" and h.game.reason == "recovery_timeout"


def test_every_players_hands_required_in_finger_mode():
    h = Host(detect_fingers=True)
    h.enrollment(2)
    h.feed(frame(person(1, raised=True), person(2, x=440, raised=True)), 14)
    h.feed(frame(person(1), person(2, x=440)), 12)
    assert h.game.phase == "preparing" and any("双手" in text for text in h.words)


def test_finger_verdict_stays_associated_with_second_players_track():
    h = Host()
    still = h.watch(2)
    h.game.cfg["detect_fingers"] = True
    for p in h.game.players:
        p["detector"].cfg["detect_fingers"] = True
    h.game.pose(with_hands(frame(*still)))
    h.game.pose(with_hands(frame(*reversed(still)), moved=2, delta=12))
    h.game.pose(with_hands(frame(*still), moved=2, delta=13))
    assert h.game.phase == "lost" and h.game.moved_player == 2
    assert h.game.moved_part == "left_fingers"
    assert "2号玩家，你的左手手指" in h.words[-1]


def test_missing_one_players_hands_never_passes_group_round():
    h = Host()
    still = h.watch(2)
    h.game.cfg["detect_fingers"] = True
    for p in h.game.players:
        p["detector"].cfg["detect_fingers"] = True
    for i in range(31):
        h.game.pose(with_hands(frame(*still), jitter=i*.01, missing=2))
        h.step(.1)
    assert h.game.phase == "paused" and h.game.round == 1


def test_missing_final_frame_does_not_count_as_group_success():
    h = Host()
    still = h.watch(2)
    h.feed(frame(*still), 2.8)
    h.game.pose(frame(still[0]))
    h.step(.3)
    assert h.game.phase == "paused"


def test_motion_removed_and_background_companion_topics_added_once():
    plugin = game_module.WoodenManPlugin({"robot_actions": True}, "q5")
    plugin.dispatch("config", {"robot_actions": True, "motion_feedback_topic": "/old/feedback"})
    tool = plugin.get_tool()
    assert {p["topic"] for p in tool["topic_out"]} == {"/q5/wooden_man/speech", "/q5/wooden_man/status", "/q5/wooden_man/highlight"}
    assert "robot_actions" not in plugin.config
    assert game_module.dialogue_topics(["/mic/audio/asr", "/camera/poses/skeleton"], True) == ["/mic/audio/asr", "/camera/poses/skeleton", "/mic/audio/asr_background"]
    assert game_module.dialogue_topics(["/mic/audio/asr_background", "/mic/audio/asr"], True) == ["/mic/audio/asr_background", "/mic/audio/asr"]
    for value in (0,1.5,11,True):
        with pytest.raises(ValueError):
            plugin.dispatch("config", {"max_players": value})
