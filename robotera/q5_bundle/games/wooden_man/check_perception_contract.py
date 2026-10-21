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

from wooden_man import Game, WoodenManPlugin, encode_speech


class String:
    def __init__(self, data=""):
        self.data = data


def source(root, name):
    path = root / (name + ".py")
    if not path.exists():
        path = root / ("perception_plugins_" + name + ".py")
    return ast.parse(path.read_text(encoding="utf-8"))


def callback(tree, name):
    node = copy.deepcopy(next(n for n in ast.walk(tree)
                              if isinstance(n, ast.FunctionDef) and n.name == name))
    node.decorator_list = []
    node.returns = None
    for arg in node.args.args + node.args.kwonlyargs:
        arg.annotation = None
    namespace = {"json": json, "String": String, "log": logging.getLogger("contract")}
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

    game = Game(say, clock=lambda: 0)
    asr_publisher = types.SimpleNamespace(publish=lambda msg: game.text(json.loads(msg.data)))
    asr_node = types.SimpleNamespace(_publish_background_enabled=True, _background_pub=asr_publisher)
    actual_asr_publish = callback(asr, "_publish_background")
    actual_asr_publish(asr_node, "游戏开始", 1000, 1001, b"\0" * 32000, {})
    assert game.phase == "countdown"
    assert text_queue.get_nowait()[0] == "游戏开始，五秒后开始。"
    print("PASS: actual ASR/Pose/TTS port declarations match the game card")
    print("PASS: actual ASR background publisher -> game -> actual TTS callback queue")


if __name__ == "__main__":
    main()
