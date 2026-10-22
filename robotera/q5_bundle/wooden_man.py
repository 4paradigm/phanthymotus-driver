"""一二三木头人: canvas ASR + Pose -> rules -> canvas TTS.

No camera, inference, synthesis or joint-control APIs live in this card. DDS runs in
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
import uuid

PRESETS = {"relaxed": (0.10, 0.22), "standard": (0.07, 0.15),
           "sensitive": (0.04, 0.10)}
OPTIONS = dict(sensitivity="standard", detect_fingers=False, host_interaction=True, robot_actions=True)
PARTS = {
    "head": ("头部", "Head", (0, 1, 2, 3, 4)),
    "torso": ("躯干", "Torso", (5, 6, 11, 12)),
    "left_arm": ("左手臂", "Left arm", (7, 9)),
    "right_arm": ("右手臂", "Right arm", (8, 10)),
    "left_leg": ("左腿", "Left leg", (13, 15)),
    "right_leg": ("右腿", "Right leg", (14, 16)),
    "left_fingers": ("左手手指", "Left fingers", ()),
    "right_fingers": ("右手手指", "Right fingers", ()),
}
HAND_NAMES = ("wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
              "index_mcp", "index_pip", "index_dip", "index_tip", "middle_mcp",
              "middle_pip", "middle_dip", "middle_tip", "ring_mcp", "ring_pip",
              "ring_dip", "ring_tip", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip")
BODY_EDGES = ((5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
              (11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (0, 1), (0, 2), (1, 3), (2, 4))


DEFAULTS = dict(cue_seconds=3.0, settle_seconds=0.5, watch_seconds=3.0,
                motion_threshold=0.07, moving_frames=3, pose_timeout=1.5,
                confidence=0.5, finger_threshold=0.15, finger_frames=2,
                robot_settle_seconds=1.0, motion_timeout=30.0, preparation_timeout=20.0)


def effective_config(config):
    cfg = dict(DEFAULTS, **OPTIONS)
    cfg.update(config or {})
    # Old callers that supplied raw thresholds retain their existing behavior.
    if config and "sensitivity" not in config and any(config.get(k, DEFAULTS[k]) != DEFAULTS[k] for k in ("motion_threshold", "finger_threshold")):
        cfg["sensitivity"] = "custom"
    if cfg["sensitivity"] in PRESETS:
        cfg["motion_threshold"], cfg["finger_threshold"] = PRESETS[cfg["sensitivity"]]
    return cfg


def visible_points(points, confidence):
    if not isinstance(points, list):
        return {}
    return {i: (p[0], p[1]) for i, p in enumerate(points)
            if isinstance(p, (list, tuple)) and len(p) >= 2
            and all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in p[:3])
            and (len(p) < 3 or p[2] >= confidence)}


def hand_indices(payload, side):
    names = payload.get("keypoint_names")
    if not isinstance(names, list) or any(not isinstance(n, str) for n in names):
        return []
    lookup = {n: i for i, n in enumerate(names)}
    return [lookup.get(f"{side}_hand_{name}", -1) for name in HAND_NAMES]


def encode_speech(words):
    """Motus TTS consumes std_msgs/String(JSON), with a top-level text field."""
    return json.dumps({"text": words}, ensure_ascii=False)


class Game:
    """Clock-injected game logic, independent of ROS and robot hardware."""

    def __init__(self, say, config=None, clock=time.monotonic, emit_action=None):
        self.say, self.clock = say, clock
        self.emit_action = emit_action
        self.cfg = effective_config(config)
        self.reset()

    def reset(self):
        self.cancel_motion()
        self.session = uuid.uuid4().hex
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
        self.moved_part = ""
        self.highlight = []
        self.part_counts = {}
        self.hands = {}
        self.frame = None
        self.pending_motion = None
        self.result = ""
        self.ready_since = None
        self.ready_frames = 0
        self.ready = False
        self.last_guidance = -math.inf
        self.last_observation = None
        self.prompt_at = None
        self.replay_prompted = False

    def cancel_motion(self):
        pending = getattr(self, "pending_motion", None)
        if pending and self.emit_action:
            self.emit_action({"kind": "wooden_man_motion_cancel", "request_id": pending["request_id"],
                              "issued_at": time.time()})

    def action(self, gesture, next_phase, target=None):
        if not self.cfg["robot_actions"] or self.emit_action is None:
            self.advance(next_phase)
            return
        request = {"kind": "wooden_man_motion_request", "request_id": self.session + ":" + uuid.uuid4().hex,
                   "gesture": gesture, "issued_at": time.time()}
        if target is not None:
            request["target"] = target
        self.pending_motion = request
        self.after_motion = next_phase
        self.motion_deadline = self.clock() + self.cfg["motion_timeout"]
        self.motion_not_before = self.clock() + self.cfg["cue_seconds"]
        self.last_motion_send = self.clock()
        self.phase = "robot_action"
        self.emit_action(dict(request))

    def advance(self, phase):
        if phase == "countdown":
            self.phase = phase
            self.deadline = self.clock() + 5.0
            self.say("游戏开始，五秒后开始。")
        elif phase == "cue":
            self.cue()
        else:
            self.phase = phase
            if phase in ("lost", "won", "error"):
                self.prompt_at = self.clock() + self.cfg["cue_seconds"]

    def motion_feedback(self, payload):
        if not isinstance(payload, dict) or payload.get("kind") != "wooden_man_motion_feedback" or not self.pending_motion:
            return
        if payload.get("request_id") != self.pending_motion["request_id"] or self.phase != "robot_action":
            return
        state = payload.get("state")
        if state == "completed":
            self.pending_motion = None
            self.phase = "robot_settle"
            self.deadline = max(self.clock() + self.cfg["robot_settle_seconds"], self.motion_not_before)
        elif state in ("error", "cancelled"):
            self.pending_motion = None
            self.phase = "error"
            self.reason = "robot_motion_failed"
            self.say("手臂动作没有完成，请检查动作卡和位置直控状态，再重新开始。")

    def preparation(self, payload):
        if self.phase != "preparing":
            return  # Visibility checks and guidance start only after the voice command.
        now = self.clock()
        persons = payload.get("persons")
        message = "没有看清玩家，请一个人站在镜头前。"
        valid = False
        if isinstance(persons, list) and len(persons) == 1 and isinstance(persons[0], dict):
            points = visible_points(persons[0].get("keypoints"), self.cfg["confidence"])
            valid = all(i in points for i in (5, 6, 11, 12, 15, 16)) and sum(5 <= i < 17 for i in points) >= 8
            if valid:
                shoulders = tuple((points[5][j] + points[6][j]) / 2 for j in (0, 1))
                hips = tuple((points[11][j] + points[12][j]) / 2 for j in (0, 1))
                valid = math.dist(shoulders, hips) >= 10
            message = "请露出肩膀和双腿，让我看清你的身体。"
            if valid and self.cfg["detect_fingers"]:
                for side in ("left", "right"):
                    indices = hand_indices(payload, side)
                    if len(indices) != 21 or any(i not in points for i in indices):
                        valid = False
                        break
                    hand = [points[i] for i in indices]
                    if max(math.dist(hand[0], hand[9]), math.dist(hand[5], hand[17])) < 12:
                        valid = False
                message = "请靠近一点，并把双手完整露出来。"
        if self.last_observation is None or now - self.last_observation > self.cfg["pose_timeout"]:
            self.ready_since, self.ready_frames = None, 0
        self.last_observation = now
        if valid:
            if self.ready_since is None:
                self.ready_since = now
            self.ready_frames += 1
            self.ready = self.ready_frames >= 5 and now - self.ready_since >= .8
            if self.ready and self.phase == "preparing" and now >= self.preparation_not_before:
                self.action("welcome", "countdown")
        else:
            self.ready_since, self.ready_frames, self.ready = None, 0, False
            if now - self.last_guidance >= 6:
                self.say(message)
                self.last_guidance = now

    def point_target(self):
        if not self.frame:
            return None
        size = self.frame.get("image_size")
        if not isinstance(size, list) or len(size) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in size):
            return None
        points = visible_points(self.frame["persons"][0]["keypoints"], self.cfg["confidence"])
        region = [points[i] for i in self.highlight if i in points]
        if not region:
            return None
        # Centre of the highlighted region, normalised in camera image coordinates.
        return {"x": max(0., min(1., sum(p[0] for p in region) / len(region) / size[0])),
                "y": max(0., min(1., sum(p[1] for p in region) / len(region) / size[1]))}

    def status(self):
        return dict(phase=self.phase, round=self.round, rounds=3, reason=self.reason,
                    valid_frames=self.valid_frames, moved_part=self.moved_part,
                    moved_part_label=PARTS[self.moved_part][0] if self.moved_part else "",
                    highlight_indices=list(self.highlight), sensitivity=self.cfg["sensitivity"],
                    detect_fingers=self.cfg["detect_fingers"],
                    result=self.result, ready=self.ready,
                    motion_request_id=self.pending_motion["request_id"] if self.pending_motion else None,
                    hands={side: {"samples": h["samples"], "visible": h["visible"],
                                  "ready": h["visible"] and h["samples"] >= 3 and h["last"] is not None and self.clock() - h["last"] <= self.cfg["pose_timeout"]}
                           for side, h in self.hands.items()})

    def text(self, payload):
        if isinstance(payload, dict):
            if payload.get("is_final") is False or payload.get("final") is False:
                return
            payload = payload.get("text", payload.get("transcript", ""))
        if not isinstance(payload, str):
            return
        # Exact utterance avoids triggering on “不要游戏开始” or our own intro.
        text = re.sub(r"[\s，。！？,.!?：:]", "", payload)
        restart = text == "游戏开始" or (self.phase in ("won", "lost", "error") and self.replay_prompted and text in ("好", "好的", "可以", "再来", "再来一次", "再玩一次"))
        if self.phase in ("waiting", "won", "lost", "error") and restart:
            self.reset()
            if self.cfg["host_interaction"]:
                self.phase = "preparing"
                self.deadline = self.clock() + self.cfg["preparation_timeout"]
                self.preparation_not_before = self.clock() + self.cfg["cue_seconds"]
                self.say("好，我先确认你站好了，请面向镜头。")
            else:
                self.action("welcome", "countdown")
        elif self.phase in ("won", "lost", "error") and self.replay_prompted and text in ("不玩了", "不玩", "结束", "不用了"):
            self.reset()
            self.say("好的，休息一下，想玩时再叫我。")

    def finish(self, phase, reason, words):
        self.phase, self.reason = phase, reason
        self.result = phase
        self.say(words)
        self.prompt_at = self.clock() + self.cfg["cue_seconds"]
        if phase == "lost" and reason == "movement":
            target = self.point_target()
            self.action("point" if target else "encourage", phase, target)
        elif phase == "won":
            self.action("celebrate", phase)
        elif self.cfg["host_interaction"]:
            self.action("encourage", phase)

    def cue(self):
        self.round += 1
        self.phase = "cue"
        self.baseline = None
        self.track_id = None
        self.motion_count = self.valid_frames = 0
        self.last_pose = None
        self.part_counts = {}
        self.hands = {}
        self.highlight = []
        self.moved_part = ""
        self.deadline = self.clock() + self.cfg["cue_seconds"] + self.cfg["settle_seconds"]
        self.say("一，二，三，木头人！")

    def invalid_pose(self):
        self.part_counts.clear()
        for h in self.hands.values():
            h.update(count=0, visible=False, signature=None, baseline=None, samples=0)

    def pose(self, payload):
        if not isinstance(payload, dict):
            return
        if self.cfg["host_interaction"] and self.phase == "preparing":
            self.preparation(payload)
            return
        if self.phase != "watch":
            return
        now = self.clock()
        persons = payload.get("persons")
        # Single-player contract prevents silently changing the observed player.
        if not isinstance(persons, list) or len(persons) != 1:
            self.invalid_pose()
            return
        person = persons[0]
        if not isinstance(person, dict):
            self.invalid_pose()
            return
        points = person.get("keypoints")
        if not isinstance(points, list) or len(points) < 17:
            self.invalid_pose()
            return
        all_visible = visible_points(points, self.cfg["confidence"])
        visible = {i: p for i, p in all_visible.items() if i < 17}
        if not all(i in visible for i in (5, 6, 11, 12)) or sum(i >= 5 for i in visible) < 8:
            self.invalid_pose()
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
            self.invalid_pose()
            return
        common = self.baseline.keys() & visible.keys()
        if sum(i >= 5 for i in common) < 8:
            self.invalid_pose()
            return
        self.last_pose = now
        self.frame = payload
        self.valid_frames += 1
        candidates = []
        for part, (_, _, indices) in PARTS.items():
            if not indices:
                continue
            score = max((math.dist(visible[i], self.baseline[i]) / self.scale
                         for i in indices if i in common), default=0)
            self.part_counts[part] = self.part_counts.get(part, 0) + 1 if score > self.cfg["motion_threshold"] else 0
            if self.part_counts[part] >= self.cfg["moving_frames"]:
                candidates.append((score / self.cfg["motion_threshold"], part, list(indices)))
        self.motion_count = max(self.part_counts.values(), default=0)
        if self.cfg["detect_fingers"]:
            candidates.extend(self.check_hands(payload, all_visible, now))
        if candidates:
            # One stable anatomical label, even if several regions moved.
            _, self.moved_part, self.highlight = max(candidates, key=lambda c: c[0])
            self.finish("lost", "movement", f"你的{PARTS[self.moved_part][0]}动了，游戏结束。")

    def check_hands(self, payload, visible, now):
        candidates = []
        for side in ("left", "right"):
            h = self.hands.setdefault(side, dict(samples=0, count=0, visible=False,
                                                 signature=None, baseline=None, last=None))
            indices = hand_indices(payload, side)
            if len(indices) != 21 or any(i not in visible for i in indices):
                h.update(visible=False, count=0, signature=None, baseline=None, samples=0)
                continue
            points = [visible[i] for i in indices]
            scale = max(math.dist(points[0], points[9]), math.dist(points[5], points[17]))
            if scale < 12:  # Tiny/degenerate palms cannot resolve fingers reliably.
                h.update(visible=False, count=0, signature=None, baseline=None, samples=0)
                continue
            h["visible"] = True
            signature = tuple(points)
            if signature == h["signature"]:
                continue  # Pose holds the hand result between slower hand-model passes.
            h["last"] = now
            h["signature"] = signature
            h["samples"] += 1
            # Wrist-relative coordinates separate finger articulation from arm translation.
            relative = [(p[0] - points[0][0], p[1] - points[0][1]) for p in points]
            if h["baseline"] is None:
                h["baseline"], h["scale"] = relative, scale
            score = max(math.dist(p, b) / h["scale"] for p, b in zip(relative[1:], h["baseline"][1:]))
            h["count"] = h["count"] + 1 if score > self.cfg["finger_threshold"] else 0
            if h["count"] >= self.cfg["finger_frames"]:
                candidates.append((score / self.cfg["finger_threshold"], side + "_fingers", indices))
        return candidates

    def tick(self):
        now = self.clock()
        if self.phase == "robot_action":
            if now >= self.motion_deadline:
                self.cancel_motion()
                self.pending_motion = None
                self.phase, self.reason = "error", "robot_motion_timeout"
                self.say("手臂动作反馈超时，游戏暂停，请检查动作卡。")
            elif now - self.last_motion_send >= .5:
                self.emit_action(dict(self.pending_motion))
                self.last_motion_send = now
        elif self.phase == "robot_settle" and now >= self.deadline:
            self.advance(self.after_motion)
        elif self.phase == "preparing" and now >= self.deadline:
            self.finish("error", "preparation_timeout", "还没有看清你，请调整站位后重新开始。")
        elif self.phase == "intermission" and now >= self.deadline:
            self.cue()
        elif self.phase == "countdown" and now >= self.deadline:
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
                elif self.cfg["detect_fingers"] and any(
                    not self.hands.get(side, {}).get("visible") or
                    self.hands[side]["samples"] < 3 or
                    now - self.hands[side]["last"] > self.cfg["pose_timeout"]
                    for side in ("left", "right")):
                    self.finish("error", "hands_unavailable", "没有看清双手，请靠近镜头并露出双手，游戏结束。")
                elif self.round == 3:
                    self.finish("won", "three_rounds", "三轮完成，你赢了，游戏结束！")
                else:
                    if self.cfg["host_interaction"]:
                        self.say(f"不错，第{('二' if self.round == 1 else '三')}轮来了，准备好。")
                        if self.cfg["robot_actions"] and self.emit_action:
                            self.action("round", "cue")
                        else:
                            self.phase = "intermission"
                            self.deadline = now + self.cfg["cue_seconds"]
                    else:
                        self.cue()
        elif self.phase in ("won", "lost", "error") and self.cfg["host_interaction"] and self.prompt_at is not None and now >= self.prompt_at and not self.replay_prompted:
            self.replay_prompted = True
            self.say("再挑战一次吗？")


def render_highlight(payload, status, confidence=0.5):
    """JPEG skeleton view, deliberately not an unsynchronised RGB overlay."""
    from io import BytesIO
    from PIL import Image, ImageDraw, ImageFont
    image = Image.new("RGB", (800, 600), "#101c2c")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
    draw.text((20, 12), f"WOODEN MAN | {status['phase'].upper()} | Round {status['round']}/3", fill="white", font=font)
    part = status.get("moved_part", "")
    subtitle = ("MOVED: " + PARTS[part][1] + " (player's side) - verdict frozen") if part else "Pose skeleton | red = judged moving part | not RGB"
    draw.text((20, 43), subtitle, fill="#ff6470" if part else "#a7c1db", font=font)
    persons = payload.get("persons", []) if isinstance(payload, dict) else []
    if not isinstance(persons, list):
        persons = []
    records = [visible_points(p.get("keypoints", []), confidence) for p in persons if isinstance(p, dict)]
    size = payload.get("image_size") if isinstance(payload, dict) else None
    if not isinstance(size, (list, tuple)) or len(size) != 2 or any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in size):
        size = (max((p[0] for r in records for p in r.values()), default=800) + 40,
                max((p[1] for r in records for p in r.values()), default=480) + 40)
    ratio = min(760 / size[0], 440 / size[1])
    ox, oy = (800 - size[0] * ratio) / 2, 90 + (440 - size[1] * ratio) / 2
    for visible in records:
        coords = {i: (ox + x * ratio, oy + y * ratio) for i, (x, y) in visible.items()
                  if 0 <= x <= size[0] and 0 <= y <= size[1]}
        selected = set(status.get("highlight_indices", [])) if len(records) == 1 else set()
        edges = list(BODY_EDGES)
        for side in ("left", "right"):
            hand = hand_indices(payload, side)
            if len(hand) == 21:
                for start in (1, 5, 9, 13, 17):
                    chain = [hand[0], *hand[start:start + 4]]
                    edges.extend(zip(chain, chain[1:]))
        for a, b in edges:
            if a in coords and b in coords:
                draw.line((coords[a], coords[b]), fill="#ff4058" if a in selected or b in selected else "#3bbfb4", width=4)
        for i, (x, y) in coords.items():
            radius = 5 if i in selected else 3
            draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill="#ff4058" if i in selected else "#b9ede7")
        focus = [p for i, p in coords.items() if i in selected]
        if focus:
            xs, ys = zip(*focus)
            draw.rectangle((min(xs)-12, min(ys)-12, max(xs)+12, max(ys)+12), outline="#ff4058", width=3)
    if status.get("detect_fingers"):
        hands = status.get("hands", {})
        label = " | ".join(f"{s}: {'READY' if hands.get(s, {}).get('ready') else 'NOT READY'} ({hands.get(s, {}).get('samples', 0)} samples)" for s in ("left", "right"))
        if status.get("reason") == "hands_unavailable":
            label = "Hands unavailable: show both hands closer; check Pose hands ON"
    else:
        label = "Finger detection OFF | enable hands in Pose and fingers in game"
    draw.text((20, 552), label, fill="#f9cc78", font=font)
    buffer = BytesIO()
    image.save(buffer, "JPEG", quality=85)
    return buffer.getvalue()


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
    from sensor_msgs.msg import CompressedImage

    node = None
    try:
        rclpy.init()
        node = Node("q5_wooden_man")
        pub = node.create_publisher(String, output, 10)
        action_pub = node.create_publisher(String, status_topic.rsplit("/", 1)[0] + "/actions", 10)
        status_pub = node.create_publisher(String, status_topic, 10)
        image_pub = node.create_publisher(CompressedImage, status_topic.rsplit("/", 1)[0] + "/highlight", 1)

        def say(words):
            msg = String()
            msg.data = encode_speech(words)
            pub.publish(msg)

        def emit_action(payload):
            action_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        game = Game(say, config, emit_action=emit_action)
        latest_frame = {}
        last_image = 0.0
        image_transition = None

        def receive(msg):
            nonlocal latest_frame
            try:
                payload = json.loads(msg.data)
            except (ValueError, TypeError):
                payload = msg.data
            game.tick()  # Expire stale observation before accepting another frame.
            if isinstance(payload, dict) and "persons" in payload:
                if not game.result:
                    latest_frame = payload
                game.pose(payload)
            elif isinstance(payload, dict) and payload.get("kind") == "wooden_man_motion_feedback":
                game.motion_feedback(payload)
            else:
                game.text(payload)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [node.create_subscription(String, t, receive, qos) for t in topics]
        statuses.put({"state": "running", **game.status()})
        last_status = None
        last_status_time = 0.0
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
            now = time.monotonic()
            if now - last_image >= 0.2 or transition != image_transition:
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "wooden_man_pose_view"
                msg.format = "jpeg"
                msg.data = render_highlight(game.frame if game.moved_part else latest_frame, status, config["confidence"])
                image_pub.publish(msg)
                last_image, image_transition = now, transition
            if transition != last_status or now - last_status_time >= 0.5:
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
                last_status_time = now
    except Exception as exc:
        statuses.put({"state": "error", "message": str(exc)})
    finally:
        if node is not None:
            if "game" in locals():
                game.cancel_motion()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class WoodenManPlugin:
    def __init__(self, config, namespace):
        self.config = effective_config({k: v for k, v in config.items() if k in DEFAULTS or k in OPTIONS})
        self.configured_topics = {k: config.get(k, "") for k in ("asr_topic", "pose_topic")}
        self.configured_topics["motion_feedback_topic"] = config.get("motion_feedback_topic", f"/{namespace}/wooden_man_motion/feedback")
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
                 "pose_topic": {"type": "string", "description": "Pose 输出 topic，画布未传递多输入时填写"},
                 "motion_feedback_topic": {"type": "string", "description": "动作卡feedback；通过配置订阅，不在画布拉回边"}}
        return {"name": "wooden_man", "title": "一二三木头人", "type": "processor", "multiInstance": False,
                "description": "单人三轮木头人主持：准备引导、语音重开、部位高亮及动作事件；连接wooden_man_motion执行手势并等回位反馈，身体原地站立。",
                "inputSchema": {"type": "object", "properties": props, "required": ["action"],
                                "x-action-params": {
                                    "start": {"params": ["input_topic", "input_topics", "asr_topic", "pose_topic", "motion_feedback_topic"], "description": "连接画布，等待玩家喊游戏开始"},
                                    "stop": {"params": [], "description": "停止游戏并释放订阅"},
                                    "info": {"params": [], "description": "查看轮次和游戏状态"},
                                    "reset": {"params": [], "description": "回到等待口令，不直接开始游戏"}}},
                "topic_in": [{"format": "data/json", "desc": "ASR识别文字（asr或asr_background）"},
                             {"format": "sensor/pose2d", "desc": "Pose人体骨架（COCO-17）"}],
                "topic_out": [{"topic": self.prefix + "/speech", "format": "data/json",
                               "desc": "游戏播报 speech → TTS（text字段）"},
                              {"topic": self.prefix + "/status", "format": "data/json",
                               "desc": "游戏状态 status（仅监视，不接TTS）"},
                              {"topic": self.prefix + "/highlight", "format": "image/jpeg",
                               "desc": "判定骨架高亮（非RGB），红色为移动部位，失败时定格"},
                              {"topic": self.prefix + "/actions", "format": "data/json",
                               "desc": "主持动作actions → wooden_man_motion，不能接TTS"}],
                "configSchema": {"type": "object", "properties": {
                    "sensitivity": {"type": "string", "title": "灵敏度预设",
                                    "enum": ["relaxed", "standard", "sensitive", "custom"],
                                    "default": "standard", "description": "relaxed 宽松 / standard 标准 / sensitive 灵敏 / custom 自定义。阈值越小越灵敏。"},
                    "detect_fingers": {"type": "boolean", "title": "检测手指动作", "default": False,
                                       "description": "需同时开启Pose的hands；双手须清晰可见，缺少手部数据不会判过关。"},
                    "host_interaction": {"type": "boolean", "title": "准备引导和语音重开", "default": True},
                    "robot_actions": {"type": "boolean", "title": "机器人主持动作", "default": True,
                                      "description": "需接wooden_man_motion动作卡，并先完成arm_control位置直控准备。"},
                    "asr_topic": {"type": "string", "advanced": True, "description": "可选：画布ASR输出topic"},
                    "pose_topic": {"type": "string", "advanced": True, "description": "可选：画布Pose输出topic"},
                    "motion_feedback_topic": {"type": "string", "title": "动作完成反馈topic", "default": self.configured_topics["motion_feedback_topic"],
                                              "description": "复制wooden_man_motion的feedback输出topic；同一Q5驱动已自动填写，无需反向连线。"},
                    **{k: {"type": "integer" if k in ("moving_frames", "finger_frames") else "number", "default": v, "exclusiveMinimum": 0,
                           "title": {"cue_seconds": "口令播报预算（秒）", "settle_seconds": "站稳等待（秒）",
                                     "watch_seconds": "每轮观察时长（秒）", "motion_threshold": "身体位移阈值／躯干长度",
                                     "moving_frames": "身体连续超阈帧数", "pose_timeout": "感知超时（秒）",
                                     "confidence": "关键点最低置信度", "finger_threshold": "手指位移阈值／手掌长度",
                                     "finger_frames": "手指连续超阈采样数", "robot_settle_seconds": "动作完成后站稳等待（秒）",
                                     "motion_timeout": "动作反馈超时（秒）", "preparation_timeout": "准备站位超时（秒）"}[k],
                           "advanced": True, **({"x-show-when": {"sensitivity": ["custom"]}} if k in ("motion_threshold", "finger_threshold") else {})}
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

    def _settings(self, args):
        """Validate a candidate atomically; config is sent before canvas start."""
        config, topics = dict(self.config), dict(self.configured_topics)
        supplied = args.get("config") or {}
        if not isinstance(supplied, dict):
            raise ValueError("config须为对象")
        for key in DEFAULTS:
            value = args.get(key, supplied.get(key, config[key]))
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"无效参数: {key}")
            config[key] = value
        if any(config[k] != int(config[k]) for k in ("moving_frames", "finger_frames")) or config["confidence"] > 1 or config["watch_seconds"] <= config["pose_timeout"]:
            raise ValueError("moving_frames和finger_frames须为整数，confidence≤1，watch_seconds须大于pose_timeout")
        for key in OPTIONS:
            config[key] = args.get(key, supplied.get(key, config[key]))
        if not isinstance(config["sensitivity"], str) or config["sensitivity"] not in (*PRESETS, "custom"):
            raise ValueError("无效参数: sensitivity")
        for key in ("detect_fingers", "host_interaction", "robot_actions"):
            if not isinstance(config[key], bool):
                raise ValueError(f"{key}须为布尔值")
        if "sensitivity" not in args and "sensitivity" not in supplied and any(
            config[k] != self.config[k] for k in ("motion_threshold", "finger_threshold")):
            config["sensitivity"] = "custom"  # Preserve old saved numeric settings.
        config = effective_config(config)
        for key in topics:
            value = args.get(key, supplied.get(key, topics[key]))
            if not isinstance(value, str):
                raise ValueError(f"{key}须为topic字符串")
            value = value.strip()
            if value and (not value.startswith("/") or any(c.isspace() for c in value)):
                raise ValueError(f"{key}须为绝对ROS topic或留空")
            topics[key] = value
        return config, topics

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
        if action not in ("config", "start"):
            return None
        try:
            config, configured_topics = self._settings(args)
        except ValueError as exc:
            if action == "config":
                # Core's auto-config path checks JSON-RPC errors before start;
                # a mere {state:error} config reply is otherwise ignored there.
                raise
            return {"state": "error", "message": str(exc)}
        if action == "config":
            changed = config != self.config or configured_topics != self.configured_topics
            if changed and self.process is not None:
                self.stop()  # Apply on the next explicit canvas start, never auto-start a game.
            self.config, self.configured_topics = config, configured_topics
            running = self.process is not None and self.process.is_alive()
            return {"status": "configured", "adapter_ok": True,
                    "state": "running" if running else "idle", "config": dict(self.config),
                    **self.configured_topics}
        supplied = args.get("config") or {}
        topics = args.get("input_topics") or []
        if not isinstance(topics, list) or any(not isinstance(t, str) for t in topics):
            return {"state": "error", "message": "input_topics须为topic字符串数组"}
        topics = list(topics)
        for key in ("input_topic", "asr_topic", "pose_topic"):
            value = (args.get(key) or supplied.get(key)) if key == "input_topic" else configured_topics[key]
            if value:
                topics.append(value)
        if any(not isinstance(t, str) or not t.startswith("/") or any(c.isspace() for c in t) for t in topics):
            return {"state": "error", "message": "请填写绝对ROS topic"}
        topics = list(dict.fromkeys(topics))
        if len(topics) < 2:
            return {"state": "error", "message": "请连接ASR和Pose两路输入；若画布只注入一路input_topic，请在卡片配置填写asr_topic和pose_topic"}
        if config["robot_actions"]:
            if not configured_topics["motion_feedback_topic"]:
                return {"state": "error", "message": "开启主持动作时须填写motion_feedback_topic"}
            topics = list(dict.fromkeys([*topics, configured_topics["motion_feedback_topic"]]))
        if self.process is not None and self.process.is_alive() and topics == self.topics and config == self.config:
            return self.info()
        self.stop()
        self.config, self.configured_topics, self.topics = config, configured_topics, topics
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
