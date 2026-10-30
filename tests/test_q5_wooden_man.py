"""Behavioral tests: player speech, three rounds, movement and loss of vision."""
import importlib.util
import json
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "robotera/q5_bundle/wooden_man.py"
spec = importlib.util.spec_from_file_location("wooden_man", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Rig:
    def __init__(self, config=None):
        self.now = 0.0
        self.words = []
        self.game = module.Game(self.words.append, {"host_interaction": False, **(config or {})}, clock=lambda: self.now)

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
    assert r.words[-1].startswith("你的") and r.words[-1].endswith("动了，游戏结束。")
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
    assert {v["format"] for v in tool["topic_in"]} == {"sensor/pose2d", "data/json"}
    assert p.dispatch("start", {})["state"] == "error"
    assert p.dispatch("start", {"input_topics": ["/asr", "/pose"], "moving_frames": 1.2})["state"] == "error"
    assert p.dispatch("start", {"input_topics": ["/asr", "/pose"], "cue_seconds": float("nan")})["state"] == "error"
    assert p.dispatch("stop", {})["state"] == "idle"
    assert p.dispatch("info", {})["topic_out"][0]["topic"] == "/q5/wooden_man/speech"
    assert p.dispatch("reset", {})["state"] == "error"


def test_asr_background_json_triggers_and_speech_is_tts_json():
    wire = []
    game = module.Game(lambda words: wire.append(module.encode_speech(words)), {"host_interaction": False}, clock=lambda: 0)
    game.text({"text": "游戏开始", "background": True, "priority": 0,
               "log_type": True, "audio_start_ts": 1000, "audio_end_ts": 1001,
               "audio_duration_ms": 1000})
    assert game.phase == "countdown"
    assert json.loads(wire[0]) == {"text": "游戏开始，五秒后开始。"}


def test_canvas_ports_match_real_perception_formats():
    tool = module.make_plugin({}, "q5", None, None).get_tool()
    asr_outputs = [{"topic": "/mic/audio/asr", "format": "data/json"},
                   {"topic": "/mic/audio/asr_background", "format": "data/json"}]
    tts_inputs = [{"format": "data/json"}]
    for output in asr_outputs:
        assert any(p["format"] == output["format"] for p in tool["topic_in"])
    speech = next(p for p in tool["topic_out"] if p["topic"].endswith("/speech"))
    assert speech["format"] == tts_inputs[0]["format"]
    assert "TTS" in speech["desc"]
    assert "status" in tool["topic_out"][1]["desc"]


def test_canvas_config_is_handled_before_start_without_starting_game():
    p = module.make_plugin({}, "q5", None, None)
    result = p.dispatch("config", {"instance_id": "card-1", "cue_seconds": 4,
                                  "asr_topic": "/mic/asr_background", "pose_topic": "/camera/pose/skeleton"})
    assert result["status"] == "configured" and result["adapter_ok"]
    assert p.process is None and p.info()["phase"] == "waiting"
    assert p.config["cue_seconds"] == 4
    assert p.configured_topics["asr_topic"] == "/mic/asr_background"
    assert p.dispatch("config", {"asr_topic": "", "pose_topic": ""})["status"] == "configured"
    assert p.configured_topics == {"asr_topic": "", "pose_topic": ""}


def test_invalid_config_is_atomic_and_raises_to_the_mcp_error_handler():
    p = module.make_plugin({}, "q5", None, None)
    before = dict(p.config)
    with pytest.raises(ValueError, match="confidence"):
        p.dispatch("config", {"cue_seconds": 6, "confidence": -1})
    assert p.config == before and p.process is None
    with pytest.raises(ValueError, match="asr_topic"):
        p.dispatch("config", {"asr_topic": "not/a/ROS/topic"})
    assert p.configured_topics["asr_topic"] == ""


def test_reapplying_saved_config_does_not_stop_running_worker():
    from types import SimpleNamespace
    p = module.make_plugin({}, "q5", None, None)
    sentinel = SimpleNamespace(is_alive=lambda: True)
    p.process = sentinel
    assert p.dispatch("config", dict(module.DEFAULTS))["state"] == "running"
    assert p.process is sentinel


def hand_pose(delta=0, jitter=0, missing=None):
    points = [[100 + i * 3, 100 + (100 if i >= 11 else 0), 0.99] for i in range(17)]
    names = [f"body_{i}" for i in range(17)]
    for side, wrist in (("left", 9), ("right", 10)):
        x, y = points[wrist][:2]
        hand = [[x + jitter, y, 0.99]]
        for finger in range(5):
            hand.extend([[x + (finger - 2) * 8 + jitter, y - 20 - joint * 9, 0.99] for joint in range(4)])
        if side == "left":
            hand[8][0] += delta
        if side == missing:
            hand = [[0, 0, 0]] * 21
        points.extend(hand)
        names.extend(f"{side}_hand_{name}" for name in module.HAND_NAMES)
    return {"persons": [{"id": 1, "keypoints": points}], "keypoint_names": names,
            "image_size": [320, 320]}


@pytest.mark.parametrize("preset,dx,loses", [("relaxed", 8, False), ("standard", 8, True),
                                             ("standard", 5, False), ("sensitive", 5, True)])
def test_presets_change_actual_body_judgment(preset, dx, loses):
    r = Rig({"sensitivity": preset})
    r.start()
    r.pose()
    for _ in range(3):
        r.pose(dx)
    assert (r.game.phase == "lost") == loses


def test_motion_must_be_consecutive_in_the_same_part():
    r = Rig()
    r.start()
    r.pose()
    for joint in [9, 15] * 5:
        p = hand_pose()
        p["persons"][0]["keypoints"][joint][0] += 20
        r.game.pose(p)
    assert r.game.phase == "watch"


@pytest.mark.parametrize("joint,part,label", [(0, "head", "头部"), (9, "left_arm", "左手臂"),
                                              (16, "right_leg", "右腿")])
def test_body_part_speech_and_highlight_match(joint, part, label):
    r = Rig()
    r.start()
    r.game.pose(hand_pose())
    for _ in range(3):
        p = hand_pose()
        p["persons"][0]["keypoints"][joint][0] += 20
        r.game.pose(p)
    assert r.game.status()["moved_part"] == part
    assert joint in r.game.status()["highlight_indices"]
    assert r.words[-1] == f"你的{label}动了，游戏结束。"


def test_finger_articulation_and_held_model_results():
    r = Rig({"detect_fingers": True})
    r.start()
    r.game.pose(hand_pose())
    for _ in range(10):
        r.game.pose(hand_pose(delta=10))
    assert r.game.phase == "watch", "Held hand results must not count as new inference observations"
    r.game.pose(hand_pose(delta=11))
    assert r.game.phase == "lost"
    assert r.game.moved_part == "left_fingers"
    assert 25 in r.game.highlight
    assert r.words[-1] == "你的左手手指动了，游戏结束。"


def test_disabled_fingers_and_hand_translation_do_not_report_finger_motion():
    for config in ({}, {"detect_fingers": True}):
        r = Rig(config)
        r.start()
        r.game.pose(hand_pose())
        for i in range(1, 12):
            r.game.pose(hand_pose(delta=30 if not config else 0, jitter=i))
        assert r.game.phase == "watch"


@pytest.mark.parametrize("missing", ["left", "right", "metadata", "body_only", "held"])
def test_finger_mode_cannot_win_without_readable_both_hands(missing):
    r = Rig({"detect_fingers": True})
    r.start()
    for i in range(31):
        p = hand_pose(jitter=0 if missing == "held" else i * 0.01, missing=missing)
        if missing == "metadata":
            p.pop("keypoint_names")
        if missing == "body_only":
            p["persons"][0]["keypoints"] = p["persons"][0]["keypoints"][:17]
        r.game.pose(p)
        r.step(0.1)
    assert r.game.phase == "error" and r.game.reason == "hands_unavailable"


def test_three_rounds_with_still_hands_and_each_round_recalibrates():
    r = Rig({"detect_fingers": True})
    r.start()
    for round_number in range(3):
        for i in range(31):
            r.game.pose(hand_pose(jitter=i * 0.01, delta=round_number * 20))
            r.step(0.1)
        if round_number < 2:
            r.watch()
    assert r.game.phase == "won"


def test_old_held_hands_cannot_pass_even_after_three_initial_samples():
    r = Rig({"detect_fingers": True})
    r.start()
    for i in range(3):
        r.game.pose(hand_pose(jitter=i * 0.01))
        r.step(0.1)
    for _ in range(28):
        r.game.pose(hand_pose(jitter=0.02))
        r.step(0.1)
    assert r.game.phase == "error" and r.game.reason == "hands_unavailable"


def test_hand_names_follow_metadata_and_occlusion_recalibrates():
    r = Rig({"detect_fingers": True})
    r.start()
    def permute(p):
        points = p["persons"][0]["keypoints"]
        points[17:] = reversed(points[17:])
        p["keypoint_names"][17:] = reversed(p["keypoint_names"][17:])
        return p
    r.game.pose(permute(hand_pose()))
    r.game.pose(permute(hand_pose(delta=12)))
    r.game.pose(permute(hand_pose(missing="left")))
    r.game.pose(permute(hand_pose(delta=30)))
    assert r.game.phase == "watch"
    r.game.pose(permute(hand_pose(delta=42)))
    r.game.pose(permute(hand_pose(delta=43)))
    assert r.game.phase == "lost" and r.game.moved_part == "left_fingers"


@pytest.mark.parametrize("args", [{"sensitivity": "bogus"}, {"detect_fingers": "true"},
                                 {"finger_frames": 1.5}, {"finger_threshold": 0}])
def test_new_settings_validate_atomically(args):
    p = module.WoodenManPlugin({}, "q5")
    before = dict(p.config)
    with pytest.raises(ValueError):
        p.dispatch("config", args)
    assert p.config == before


def test_saved_numeric_thresholds_migrate_and_explicit_presets_take_precedence():
    p = module.WoodenManPlugin({}, "q5")
    p.dispatch("config", {"motion_threshold": 0.08})
    assert p.config["sensitivity"] == "custom" and p.config["motion_threshold"] == 0.08
    p.dispatch("config", {"sensitivity": "sensitive", "motion_threshold": 0.08})
    assert p.config["motion_threshold"] == 0.04
    assert p.get_tool()["configSchema"]["properties"]["detect_fingers"]["type"] == "boolean"


def test_highlight_jpeg_has_red_pixels_and_handles_missing_pose():
    from io import BytesIO
    from PIL import Image
    r = Rig({"detect_fingers": True})
    r.start()
    for delta in (0, 10, 11):
        r.game.pose(hand_pose(delta=delta))
    jpeg = module.render_highlight(r.game.frame, r.game.status())
    image = Image.open(BytesIO(jpeg)).convert("RGB")
    assert image.size == (800, 600)
    assert sum(1 for red, green, blue in image.getdata() if red > 180 and green < 120 and blue < 140) > 100
    r.game.reset()
    assert module.render_highlight(None, r.game.status()).startswith(b"\xff\xd8")
    assert module.render_highlight({"persons": None}, r.game.status()).startswith(b"\xff\xd8")
