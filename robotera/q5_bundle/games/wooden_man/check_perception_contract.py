"""Check against actual perception source, without loading inference models.

Usage: python check_perception_contract.py PATH_TO_PERCEPTION_PLUGINS
Reads asr.py, tts.py, pose.py, extracts their real declared ports and executes
the actual ASR background publisher / TTS text callback against lightweight
ROS message/queue doubles. Does not open the microphone or drive the robot.
"""
import argparse
import ast
import copy
import json
import logging
from pathlib import Path
import queue
import types

from wooden_man import Game, WoodenManPlugin, encode_speech, HAND_NAMES


class String:
    def __init__(self, data=""):
        self.data = data


def source(root, name):
    path = root / (name + ".py")
    if not path.exists():
        path = root / ("perception_plugins_" + name + ".py")
    return ast.parse(path.read_text(encoding="utf-8"))


def callback(tree, name, extra=None):
    node = copy.deepcopy(next(n for n in ast.walk(tree)
                              if isinstance(n, ast.FunctionDef) and n.name == name))
    node.decorator_list = []
    node.returns = None
    for arg in node.args.args + node.args.kwonlyargs:
        arg.annotation = None
    namespace = {"json": json, "String": String, "log": logging.getLogger("contract")}
    namespace.update(extra or {})
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 "<actual-perception-callback>", "exec"), namespace)
    return namespace[name]


def ports(tree, tool_name, key):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        fields = {k.value: v for k, v in zip(node.keys, node.values)
                  if isinstance(k, ast.Constant)}
        if isinstance(fields.get("name"), ast.Constant) and fields["name"].value == tool_name:
            expr = fields[key]
            return [n.value for d in ast.walk(expr) if isinstance(d, ast.Dict)
                    for k, n in zip(d.keys, d.values)
                    if isinstance(k, ast.Constant) and k.value == "format" and isinstance(n, ast.Constant)]
    raise AssertionError(f"Tool definition not found: {tool_name}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("perception_plugins", type=Path)
    args = parser.parse_args()
    asr, tts, pose = [source(args.perception_plugins, name) for name in ("asr", "tts", "pose")]
    tool = WoodenManPlugin({}, "contract").get_tool()
    input_formats = [p["format"] for p in tool["topic_in"]]
    for fmt in ports(asr, "asr", "topic_out"):
        assert fmt in input_formats, ("ASR -> wooden_man", fmt, input_formats)
    assert "sensor/pose2d" in ports(pose, "pose", "topic_out")
    assert "sensor/pose2d" in input_formats
    speech = next(p for p in tool["topic_out"] if p["topic"].endswith("/speech"))
    assert speech["format"] in ports(tts, "tts", "topic_in")

    text_queue = queue.Queue()
    tts_node = types.SimpleNamespace(state="running", _text_queue=text_queue, _current_gen=lambda: 0)
    actual_tts_callback = callback(tts, "_text_cb")

    def say(text):
        actual_tts_callback(tts_node, String(encode_speech(text)))

    game = Game(say, {"host_interaction": False}, clock=lambda: 0)
    asr_publisher = types.SimpleNamespace(publish=lambda msg: game.text(json.loads(msg.data)))
    asr_node = types.SimpleNamespace(_publish_background_enabled=True, _background_pub=asr_publisher)
    actual_asr_publish = callback(asr, "_publish_background")
    actual_asr_publish(asr_node, "游戏开始", 1000, 1001, b"\0" * 32000, {})
    assert game.phase == "countdown"
    assert text_queue.get_nowait()[0] == "游戏开始，五秒后开始。"
    print("PASS: actual ASR/Pose/TTS port declarations match the game card")
    print("PASS: actual ASR background publisher -> game -> actual TTS callback queue")

    # Execute the upstream skeleton serializer rather than assuming detection
    # order is identity. Both body and merged hands stay in the same record.
    skeleton_record = callback(pose, "_skeleton_record", {"_full_keypoints": lambda points: points})
    pose_node = types.SimpleNamespace(_payload_keypoints=lambda person: person["keypoints"])
    records = [skeleton_record(pose_node, {"id": identity, "score": .99,
               "box": [0,0,100,200], "keypoints": [[identity,10,.99]]*17,
               "verdict": {"posture": "standing", "activity": None}}) for identity in (12,7)]
    assert [record["id"] for record in records] == [12,7]
    assert records[1]["keypoints"][0][0] == 7
    print("PASS: actual Pose skeleton serializer preserves per-person track IDs and keypoints")

    clock = [0.]
    dialog = Game(say, clock=lambda: clock[0])
    dialog.text("小范小范，我们来玩木头人游戏吧")
    assert dialog.phase == "rules_question"
    clock[0] = dialog.listen_after+.01
    asr_publisher.publish = lambda msg: dialog.text(json.loads(msg.data))
    actual_asr_publish(asr_node, "不用介绍", 1002,1003,b"\0"*32000,{})
    assert dialog.phase == "player_count"
    clock[0] = dialog.listen_after+.01
    actual_asr_publish(asr_node, "两个人", 1004,1005,b"\0"*32000,{})
    assert dialog.phase == "registering" and dialog.expected_players == 2
    print("PASS: actual ASR background replies -> rules skip -> two-player enrollment")

    hand_path = args.perception_plugins / "hand_runtime.py"
    if not hand_path.exists():
        hand_path = args.perception_plugins / "perception_plugins_hand_runtime.py"
    if hand_path.exists():
        import numpy as np
        tree = ast.parse(hand_path.read_text(encoding="utf-8"))
        names = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                     and any(isinstance(t, ast.Name) and t.id == "HAND_KEYPOINTS" for t in n.targets))
        assert tuple(names) == HAND_NAMES
        merge = callback(tree, "merge_keypoints", {"np": np, "N_HAND_KEYPOINTS": 21})
        merge_names = callback(tree, "merged_keypoint_names", {"HAND_KEYPOINTS": names})
        body = np.array([[100 + i * 3, 100 + (100 if i >= 11 else 0), .99] for i in range(17)])
        hand = np.array([[140 + (i % 4) * 8, 100 - i * 2, .99] for i in range(21)])
        hand[9, :2] = [140, 65]
        clock = [0.0]
        game = Game(lambda _: None, {"detect_fingers": True, "host_interaction": False}, clock=lambda: clock[0])
        game.text("游戏开始")
        clock[0] = 5
        game.tick()
        clock[0] = 8.5
        game.tick()
        for delta in (0, 15, 16):
            left = hand.copy()
            left[8, 0] += delta
            game.pose({"persons": [{"id": 1, "keypoints": merge(body, left, hand).tolist()}],
                       "keypoint_names": merge_names([f"body_{i}" for i in range(17)])})
        assert game.phase == "lost" and game.moved_part == "left_fingers", game.status()
        print("PASS: actual hand-runtime merged keypoints and names -> finger movement judgment")
    else:
        print("SKIP: hand_runtime.py absent; finger wire compatibility not checked")

    core_path = args.perception_plugins / "agent-core_src_tool_config.py"
    if core_path.exists():
        core = ast.parse(core_path.read_text(encoding="utf-8"))
        split = callback(core, "split_config_by_scope")
        plan = callback(core, "plan_config_calls", {"split_config_by_scope": split})
        plugin = WoodenManPlugin({}, "contract")
        saved = {"cue_seconds": 4, "asr_topic": "", "pose_topic": ""}
        calls = plan(plugin.get_tool(), saved, {}, "card-contract")
        assert calls
        for body, extra in calls:
            result = plugin.dispatch("config", {**body, **extra})
            assert result["status"] == "configured", result
        assert plugin.config["cue_seconds"] == 4 and plugin.process is None
        print("PASS: actual Core plan_config_calls -> game config -> info, no premature game start")


if __name__ == "__main__":
    main()
