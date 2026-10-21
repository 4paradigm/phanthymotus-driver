"""一二三木头人: canvas ASR + Pose -> rules -> canvas TTS.

No camera, inference, synthesis or motion APIs live in this card. DDS runs in
a spawned Domain-42 process; the Q5 parent remains in vendor Domain 211.
"""
from __future__ import annotations

import json
import math
import multiprocessing as mp
import queue
import re
import threading
import time


DEFAULTS = dict(cue_seconds=3.0, settle_seconds=0.5, watch_seconds=3.0,
                motion_threshold=0.07, moving_frames=3, pose_timeout=1.5,
                confidence=0.5)


def encode_speech(words):
    """Motus TTS consumes std_msgs/String(JSON), with a top-level text field."""
    return json.dumps({"text": words}, ensure_ascii=False)


class Game:
    """Clock-injected game logic, independent of ROS and robot hardware."""

    def __init__(self, say, config=None, clock=time.monotonic):
        self.say, self.clock = say, clock
        self.cfg = dict(DEFAULTS, **(config or {}))
        self.reset()

    def reset(self):
        self.phase = "waiting"
        self.round = 0
        self.deadline = 0.0
        self.last_pose = None
        self.baseline = None
        self.track_id = None
        self.scale = 1.0
        self.motion_count = 0
        self.valid_frames = 0
        self.reason = ""

    def status(self):
        return dict(phase=self.phase, round=self.round, rounds=3, reason=self.reason,
                    valid_frames=self.valid_frames)

    def text(self, payload):
        if isinstance(payload, dict):
            if payload.get("is_final") is False or payload.get("final") is False:
                return
            payload = payload.get("text", payload.get("transcript", ""))
        if not isinstance(payload, str):
            return
        # Exact utterance avoids triggering on “不要游戏开始” or our own intro.
        text = re.sub(r"[\s，。！？,.!?：:]", "", payload)
        if self.phase in ("waiting", "won", "lost", "error") and text == "游戏开始":
            self.reset()
            self.phase = "countdown"
            self.deadline = self.clock() + 5.0
            self.say("游戏开始，五秒后开始。")

    def finish(self, phase, reason, words):
        self.phase, self.reason = phase, reason
        self.say(words)

    def cue(self):
        self.round += 1
        self.phase = "cue"
        self.baseline = None
        self.track_id = None
        self.motion_count = self.valid_frames = 0
        self.last_pose = None
        self.deadline = self.clock() + self.cfg["cue_seconds"] + self.cfg["settle_seconds"]
        self.say("一，二，三，木头人！")

    def pose(self, payload):
        if self.phase != "watch" or not isinstance(payload, dict):
            return
        now = self.clock()
        persons = payload.get("persons")
        # Single-player contract prevents silently changing the observed player.
        if not isinstance(persons, list) or len(persons) != 1:
            return
        person = persons[0]
        if not isinstance(person, dict):
            return
        points = person.get("keypoints")
        if not isinstance(points, list) or len(points) < 17:
            return
        visible = {}
        for i in range(17):  # Head, shoulders, elbows, wrists, hips, knees, ankles.
            p = points[i]
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                continue
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                   not math.isfinite(v) for v in p[:3]):
                continue
            if len(p) < 3 or p[2] >= self.cfg["confidence"]:
                visible[i] = (p[0], p[1])
        if not all(i in visible for i in (5, 6, 11, 12)) or sum(i >= 5 for i in visible) < 8:
            return
        if self.baseline is None:
            self.baseline = visible
            self.track_id = person.get("id")
            # Fixed torso size: retains translation, depth change and limb motion.
            shoulders = tuple((visible[5][j] + visible[6][j]) / 2 for j in (0, 1))
            hips = tuple((visible[11][j] + visible[12][j]) / 2 for j in (0, 1))
            self.scale = math.dist(shoulders, hips)
            if self.scale < 10:
                self.baseline = None
                return
        elif self.track_id != person.get("id"):
            return
        common = self.baseline.keys() & visible.keys()
        if sum(i >= 5 for i in common) < 8:
            return
        self.last_pose = now
        self.valid_frames += 1
        distances = sorted((math.dist(visible[i], self.baseline[i]) / self.scale
                            for i in common), reverse=True)
        # A moving wrist or head alone counts; consecutive frames reject spikes.
        self.motion_count = self.motion_count + 1 if distances[0] > self.cfg["motion_threshold"] else 0
        if self.motion_count >= self.cfg["moving_frames"]:
            self.finish("lost", "movement", "你动了，游戏结束。")

    def tick(self):
        now = self.clock()
        if self.phase == "countdown" and now >= self.deadline:
            self.cue()
        elif self.phase == "cue" and now >= self.deadline:
            self.phase = "watch"
            self.watch_start = now
            self.deadline = now + self.cfg["watch_seconds"]
        elif self.phase == "watch":
            if now - (self.last_pose if self.last_pose is not None else self.watch_start) > self.cfg["pose_timeout"]:
                self.finish("error", "pose_unavailable", "没有看清玩家，请一个人站在镜头前。游戏结束。")
            elif now >= self.deadline:
                if self.valid_frames < 5:
                    self.finish("error", "insufficient_pose", "姿态数据不足，游戏结束，请重新开始。")
                elif self.round == 3:
                    self.finish("won", "three_rounds", "三轮完成，你赢了，游戏结束！")
                else:
                    self.cue()


def _worker(topics, output, status_topic, config, commands, statuses):
    """Own a separate DDS context; never reuse the vendor executor."""
    import os
    from common import logsafe
    logsafe.install(check_fd=False)
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.pop("CYCLONEDDS_URI", None)
    from fastdds_transport import configure_bundled_fastdds_transport
    configure_bundled_fastdds_transport()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    node = None
    try:
        rclpy.init()
        node = Node("q5_wooden_man")
        pub = node.create_publisher(String, output, 10)
        status_pub = node.create_publisher(String, status_topic, 10)

        def say(words):
            msg = String()
            msg.data = encode_speech(words)
            pub.publish(msg)

        game = Game(say, config)

        def receive(msg):
            try:
                payload = json.loads(msg.data)
            except (ValueError, TypeError):
                payload = msg.data
            game.tick()  # Expire stale observation before accepting another frame.
            if isinstance(payload, dict) and "persons" in payload:
                game.pose(payload)
            else:
                game.text(payload)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [node.create_subscription(String, t, receive, qos) for t in topics]
        statuses.put({"state": "running", **game.status()})
        last_status = None
        while rclpy.ok():
            try:
                command = commands.get_nowait()
                if command == "stop":
                    break
                if command == "reset":
                    game.reset()
            except queue.Empty:
                pass
            rclpy.spin_once(node, timeout_sec=0.02)
            game.tick()
            status = {"state": "running", **game.status()}
            transition = (status["state"], status["phase"], status["round"], status["reason"])
            if transition != last_status:
                msg = String()
                msg.data = json.dumps(status, ensure_ascii=False)
                status_pub.publish(msg)
                try:
                    statuses.put_nowait(status)
                except queue.Full:
                    # Keep recent terminal states even after many replays
                    # without an MCP info poll. The DDS status stream remains
                    # authoritative for the canvas display.
                    try:
                        statuses.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        statuses.put_nowait(status)
                    except queue.Full:
                        pass
                last_status = transition
    except Exception as exc:
        statuses.put({"state": "error", "message": str(exc)})
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class WoodenManPlugin:
    def __init__(self, config, namespace):
        self.config = {k: config.get(k, v) for k, v in DEFAULTS.items()}
        self.configured_topics = {k: config.get(k, "") for k in ("asr_topic", "pose_topic")}
        self.lock = threading.RLock()
        self.prefix = f"/{namespace}/wooden_man"
        self.process = None
        self.topics = []
        self.status = {"state": "idle", "phase": "waiting"}

    def get_tool(self):
        props = {"action": {"type": "string", "enum": ["start", "stop", "info", "reset"]},
                 "input_topic": {"type": "string", "description": "画布上游 topic（兼容单输入）"},
                 "input_topics": {"type": "array", "items": {"type": "string"},
                                  "description": "画布连接的 ASR 和 Pose 输出 topic"},
                 "asr_topic": {"type": "string", "description": "ASR 输出 topic，画布未传递多输入时填写"},
                 "pose_topic": {"type": "string", "description": "Pose 输出 topic，画布未传递多输入时填写"}}
        return {"name": "wooden_man", "title": "一二三木头人", "type": "processor", "multiInstance": False,
                "description": "一二三木头人：喊游戏开始，等待5秒，固定三轮；消费画布ASR和Pose，输出播报给TTS。机器人保持原地，不发送运动命令。",
                "inputSchema": {"type": "object", "properties": props, "required": ["action"],
                                "x-action-params": {
                                    "start": {"params": ["input_topic", "input_topics", "asr_topic", "pose_topic"], "description": "连接画布，等待玩家喊游戏开始"},
                                    "stop": {"params": [], "description": "停止游戏并释放订阅"},
                                    "info": {"params": [], "description": "查看轮次和游戏状态"},
                                    "reset": {"params": [], "description": "回到等待口令，不直接开始游戏"}}},
                "topic_in": [{"format": "data/json", "desc": "ASR识别文字（asr或asr_background）"},
                             {"format": "sensor/pose2d", "desc": "Pose人体骨架（COCO-17）"}],
                "topic_out": [{"topic": self.prefix + "/speech", "format": "data/json",
                               "desc": "游戏播报 speech → TTS（text字段）"},
                              {"topic": self.prefix + "/status", "format": "data/json",
                               "desc": "游戏状态 status（仅监视，不接TTS）"}],
                "configSchema": {"type": "object", "properties": {
                    "asr_topic": {"type": "string", "description": "可选：画布ASR输出topic"},
                    "pose_topic": {"type": "string", "description": "可选：画布Pose输出topic"},
                    **{k: {"type": "number", "default": v, "exclusiveMinimum": 0}
                       for k, v in DEFAULTS.items()}}}}

    def start(self):
        pass  # Bundle startup must not enter game mode or subscribe before wiring.

    def stop(self):
        if self.process is not None:
            self.commands.put("stop")
            self.process.join(timeout=3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
            self.commands.close()
            self.statuses.close()
            self.process = None
        self.topics = []
        self.status = {"state": "idle", "phase": "waiting"}

    def info(self):
        if self.process is not None:
            try:
                while True:
                    self.status = self.statuses.get_nowait()
            except queue.Empty:
                pass
            if not self.process.is_alive() and self.status.get("state") != "error":
                self.status = {"state": "error", "message": "游戏DDS进程已退出，请重新启动卡片"}
        return {**self.status, "topic_out": self.get_tool()["topic_out"],
                "topic_in": self.get_tool()["topic_in"],
                "input_topics": self.topics, "config": dict(self.config)}

    def dispatch(self, action, args):
        with self.lock:
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        if action == "info":
            return self.info()
        if action == "stop":
            self.stop()
            return self.info()
        if action == "reset":
            if self.process is None or not self.process.is_alive():
                return {"state": "error", "message": "请先启动画布"}
            self.commands.put("reset")
            return {"state": "running", "message": "已请求回到等待口令"}
        if action != "start":
            return None
        config = dict(self.config)
        supplied = args.get("config") or {}
        if not isinstance(supplied, dict):
            return {"state": "error", "message": "config须为对象"}
        for key, default in DEFAULTS.items():
            value = args.get(key, supplied.get(key, config[key]))
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                return {"state": "error", "message": f"无效参数: {key}"}
            config[key] = value
        if config["moving_frames"] != int(config["moving_frames"]) or config["confidence"] > 1 or config["watch_seconds"] <= config["pose_timeout"]:
            return {"state": "error", "message": "moving_frames须为整数，confidence≤1，watch_seconds须大于pose_timeout"}
        topics = args.get("input_topics") or []
        if not isinstance(topics, list) or any(not isinstance(t, str) for t in topics):
            return {"state": "error", "message": "input_topics须为topic字符串数组"}
        topics = list(topics)
        for key in ("input_topic", "asr_topic", "pose_topic"):
            value = args.get(key) or supplied.get(key) or self.configured_topics.get(key)
            if value:
                topics.append(value)
        if any(not isinstance(t, str) or not t.startswith("/") or any(c.isspace() for c in t) for t in topics):
            return {"state": "error", "message": "请填写绝对ROS topic"}
        topics = list(dict.fromkeys(topics))
        if len(topics) < 2:
            return {"state": "error", "message": "请连接ASR和Pose两路输入；若画布只注入一路input_topic，请在卡片配置填写asr_topic和pose_topic"}
        if self.process is not None and self.process.is_alive() and topics == self.topics and config == self.config:
            return self.info()
        self.stop()
        self.config, self.topics = config, topics
        ctx = mp.get_context("spawn")
        self.commands, self.statuses = ctx.Queue(), ctx.Queue(maxsize=64)
        self.process = ctx.Process(target=_worker, args=(topics, self.prefix + "/speech",
                                   self.prefix + "/status", config, self.commands, self.statuses),
                                   name="q5_wooden_man", daemon=True)
        self.process.start()
        try:
            self.status = self.statuses.get(timeout=8)
        except queue.Empty:
            self.stop()
            return {"state": "error", "message": "游戏DDS启动超时"}
        return self.info()


def make_plugin(plugin_config, namespace, executor, client):
    del executor, client
    return WoodenManPlugin(plugin_config, namespace)
