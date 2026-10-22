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

PRESETS = {"relaxed": (0.10, 0.22), "standard": (0.07, 0.15),
           "sensitive": (0.04, 0.10)}
OPTIONS = dict(sensitivity="standard", detect_fingers=False, host_interaction=True)
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
                preparation_timeout=20.0, dialogue_timeout=20.0,
                registration_timeout=30.0, registration_seconds=1.0,
                registration_confidence=0.3, registration_grace=0.35,
                recovery_timeout=20.0, max_players=5)


def clean_text(text):
    return re.sub(r"[\s，。！？,.!?：:、]", "", text)


def invite(text):
    if any(word in text for word in ("不玩", "不要", "不想", "别玩", "不来", "取消", "什么", "怎么玩", "规则", "介绍")):
        return False
    return text == "游戏开始" or ("木头人" in text and
        (any(word in text for word in ("玩", "开始", "来")) or text in ("木头人游戏", "一二三木头人游戏")))


def player_count(text):
    if text in ("我一个人", "只有我", "就我一个人", "就我", "只有我一个人"):
        return 1
    match = re.fullmatch(r"(?:小范小范)?(?:我们|这次|就|一共|总共|有|我)?(?:有|就)?([一二两三四五六七八九十]|\d{1,2})(?:个人|人|位)(?:一起)?(?:玩|参加|玩游戏)?(?:吧|呀|啊)?", text)
    if not match:
        return None
    value = match[1]
    return int(value) if value.isdigit() else 2 if value == "两" else "〇一二三四五六七八九十".index(value)


def body_geometry(person, confidence):
    if not isinstance(person, dict):
        return None
    points = visible_points(person.get("keypoints"), confidence)
    if not all(i in points for i in (5, 6, 11, 12)) or sum(5 <= i < 17 for i in points) < 8:
        return None
    shoulder = tuple((points[5][j] + points[6][j]) / 2 for j in (0, 1))
    hip = tuple((points[11][j] + points[12][j]) / 2 for j in (0, 1))
    scale = math.dist(shoulder, hip)
    return (points, shoulder, hip, scale) if scale >= 10 else None


def standing(geometry):
    points, shoulder, hip, scale = geometry
    if hip[1] - shoulder[1] < .65 * scale:
        return False
    # A relaxed stance, an occluded leg or perspective foreshortening must not
    # require both knees to be perfectly straight. One clearly extended leg
    # below an upright torso is sufficient; a seated bent leg is not.
    for h, k, a in ((11, 13, 15), (12, 14, 16)):
        if not all(i in points for i in (h, a)) or points[a][1] - points[h][1] < 1.05 * scale:
            continue
        if k not in points:
            if points[a][1] - points[h][1] >= 1.3 * scale:
                return True
            continue
        upper = (points[h][0] - points[k][0], points[h][1] - points[k][1])
        lower = (points[a][0] - points[k][0], points[a][1] - points[k][1])
        length = math.hypot(*upper) * math.hypot(*lower)
        if length >= 1 and sum(x*y for x, y in zip(upper, lower)) / length <= -.4:
            return True
    return False


def registration_observation(person, confidence):
    """Return auditable per-track gates, not an opaque candidate count."""
    report = dict(track_id=person.get("id"), posture=person.get("posture"),
                  candidate=False, raised_side="", raise_checks={})
    points = visible_points(person.get("keypoints"), confidence)
    report["visible_body_joints"] = sum(5 <= i < 17 for i in points)
    body_y = [p[1] for i,p in points.items() if 5 <= i < 17]
    report["body_height_pixels"] = round(max(body_y)-min(body_y), 1) if body_y else 0
    geometry = body_geometry(person, confidence)
    reason = "body_not_clear"
    if geometry is not None:
        reason = "not_standing"
        if standing(geometry):
            reason = "hand_not_raised"
            for side, s, e, w in (("left", 5, 7, 9), ("right", 6, 8, 10)):
                check = dict(wrist_visible=w in points, elbow_visible=e in points)
                report["raise_checks"][side] = check
                if w not in points:
                    continue
                scale = geometry[3]
                above_shoulder = (points[s][1] - points[w][1]) / scale
                forearm_rise = (points[e][1] - points[w][1]) / scale if e in points else None
                check.update(wrist_above_shoulder=round(above_shoulder, 3),
                             wrist_above_elbow=round(forearm_rise, 3) if forearm_rise is not None else None)
                # Accept a high hand OR a bent-up forearm with the wrist near
                # its own shoulder. A hanging arm or horizontal T pose fails.
                raised = above_shoulder >= .05 or (
                    above_shoulder >= -.35 and forearm_rise is not None and forearm_rise >= .18
                    and math.dist(points[e], points[w]) >= .22 * scale)
                if raised and not report["raised_side"]:
                    report["raised_side"] = side
                    reason = "candidate"
    identity = person.get("id")
    if isinstance(identity, bool) or not isinstance(identity, (str, int)):
        reason = "missing_track_id"
    report.update(reason=reason, candidate=reason == "candidate", reason_label={
        "body_not_clear": "肩髋或身体关键点不清楚", "not_standing": "没有确认站姿或腿部不清楚",
        "hand_not_raised": "手腕不清楚或尚未确认举手", "missing_track_id": "缺少跟踪编号",
        "candidate": "已确认站立举手"}[reason])
    return geometry, report


def dialogue_topics(topics, enabled):
    result = list(topics)
    if enabled:
        for topic in topics:
            if topic.endswith("/asr"):
                result.append(topic + "_background")
            elif topic.endswith("/asr_background"):
                result.append(topic.removesuffix("_background"))
    return list(dict.fromkeys(result))


def effective_config(config):
    cfg = dict(DEFAULTS, **OPTIONS)
    cfg.update({k: v for k, v in (config or {}).items() if k in DEFAULTS or k in OPTIONS})
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

    def __init__(self, say, config=None, clock=time.monotonic):
        self._say, self.clock = say, clock
        self.cfg = effective_config(config)
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
        self.moved_part = ""
        self.highlight = []
        self.part_counts = {}
        self.hands = {}
        self.frame = None
        self.result = ""
        self.ready_since = None
        self.ready_frames = 0
        self.ready = False
        self.last_guidance = -math.inf
        self.last_observation = None
        self.prompt_at = None
        self.replay_prompted = False
        self.listen_after = 0.0
        self.spoken = []
        self.seen_utterances = []
        self.expected_players = 0
        self.players = []
        self.registration_key = None
        self.registration_since = None
        self.registration_frames = 0
        self.registration_last = None
        self.moved_player = None
        self.moved_track_id = None
        self.registration_candidates = 0
        self.registration_debug = []
        self.registration_pending = []
        self.registration_elapsed = 0.0
        self.observation_complete = False
        self.repeat_round = False

    def say(self, words):
        self._say(words)
        self.spoken = (self.spoken + [clean_text(words)])[-24:]
        # Canvas supplies no speaker-finished port. A configurable conservative
        # budget suppresses echo and starts the response timer after speech.
        if self.cfg["host_interaction"]:
            self.listen_after = self.clock() + max(self.cfg["cue_seconds"], len(clean_text(words)) / 4.0)

    def ask(self, phase, words):
        self.phase = phase
        self.say(words)
        self.deadline = self.listen_after + self.cfg["dialogue_timeout"]

    def ask_count(self):
        self.ask("player_count", "这次几个人一起玩？请告诉我人数。")

    def enroll(self):
        self.players = []
        self.registration_key = self.registration_since = self.registration_last = None
        self.registration_frames = self.registration_candidates = 0
        self.registration_debug, self.registration_pending = [], []
        self.registration_elapsed = 0.0
        self.ready = False
        self.phase = "registering"
        self.say(f"{self.expected_players}个人一起玩。请参加的人站到我看得到的地方，举起一只手，保持一下。")
        self.deadline = self.listen_after + self.cfg["registration_timeout"]
        self.last_guidance = self.clock()

    def hands_clear(self, payload, points):
        if not self.cfg["detect_fingers"]:
            return True
        for side in ("left", "right"):
            indices = hand_indices(payload, side)
            if len(indices) != 21 or any(i not in points for i in indices):
                return False
            hand = [points[i] for i in indices]
            if max(math.dist(hand[0], hand[9]), math.dist(hand[5], hand[17])) < 12:
                return False
        return True

    def registration(self, payload):
        now = self.clock()
        candidates = []
        observations = []
        for person in payload.get("persons", []):
            if not isinstance(person, dict):
                continue
            geometry, report = registration_observation(person, self.cfg["registration_confidence"])
            observations.append((geometry, report))
            if report["candidate"]:
                candidates.append((report["track_id"], geometry))
        self.registration_debug = [report for _, report in observations]
        self.registration_candidates = len(candidates)
        # Once a genuine hold is confirmed, it remains confirmed while those
        # same people stay visible and standing. They need not keep an arm up
        # for the entire (longer) enrollment speech budget.
        if self.registration_pending:
            current = []
            for identity, _ in self.registration_pending:
                matches = [g for g, r in observations if r["track_id"] == identity]
                if len(matches) == 1 and matches[0] is not None and standing(matches[0]):
                    current.append((identity, matches[0]))
            if len(current) == len(self.registration_pending):
                self.registration_pending = current
                self.registration_last = now
                if now >= self.listen_after:
                    self.complete_registration(current, now)
                return
            if now - self.registration_last <= self.cfg["registration_grace"]:
                return
            self.registration_pending = []
            self.registration_key = self.registration_since = None
            self.registration_frames, self.registration_elapsed = 0, 0.0
        ids = [p[0] for p in candidates]
        key = frozenset(ids)
        if len(ids) != self.expected_players or len(key) != len(ids):
            if self.registration_last is None or now - self.registration_last > self.cfg["registration_grace"]:
                self.registration_key = self.registration_since = None
                self.registration_frames, self.registration_elapsed = 0, 0.0
            if now - self.last_guidance >= 7 and now >= self.listen_after:
                unclear = any(r["reason"] in ("body_not_clear", "missing_track_id") for r in self.registration_debug)
                foreground = max(self.registration_debug, key=lambda r: r["body_height_pixels"], default={})
                if not ids and foreground.get("reason") == "not_standing":
                    self.say("还没确认站姿，请站好，让我看清肩膀、髋部和至少一条完整的腿。")
                elif not ids and unclear:
                    self.say("还没看清举手的玩家，请露出肩膀、手肘和手腕，并让腿部在画面里。")
                elif not ids:
                    self.say("还没确认举手，请站好，把一只手举到肩旁或更高，让手腕高于手肘。")
                else:
                    self.say(f"看到{len(ids)}位站立举手的候选玩家，需要{self.expected_players}位。请参加的人保持举手，其他人放下手。")
                self.last_guidance = now
            return
        if key != self.registration_key or self.registration_last is None or now - self.registration_last > self.cfg["registration_grace"]:
            self.registration_key, self.registration_since, self.registration_frames = key, now, 0
            self.registration_elapsed = 0.0
        else:
            self.registration_elapsed += min(now - self.registration_last, .2)
        self.registration_last = now
        self.registration_frames += 1
        if self.registration_frames >= 5 and self.registration_elapsed >= self.cfg["registration_seconds"]:
            self.registration_pending = list(candidates)
            if now >= self.listen_after:
                self.complete_registration(candidates, now)

    def complete_registration(self, candidates, now):
        # Initial left-to-right numbering only. Later frames use track IDs.
        candidates = sorted(candidates, key=lambda c: c[1][2][0])
        self.players = [dict(number=i+1, track_id=identity, geometry=geometry,
                             seen=now, detector=None, frame_index=None)
                        for i, (identity, geometry) in enumerate(candidates)]
        self.registration_pending = []
        self.phase = "preparing"
        self.say(f"找到{len(self.players)}位玩家了，按画面从左到右编号。请放下手，站好准备开始。")
        self.preparation_not_before = self.listen_after
        self.deadline = self.listen_after + self.cfg["preparation_timeout"]
        self.ready_since, self.ready_frames, self.last_observation = None, 0, None

    def matched_players(self, payload, now):
        matches = {}
        for index, person in enumerate(payload.get("persons", [])):
            if not isinstance(person, dict):
                continue
            identity = person.get("id")
            if isinstance(identity, bool) or not isinstance(identity, (int, str)):
                continue
            matches.setdefault(identity, []).append((index, person))
        found = []
        for player in self.players:
            candidates = matches.get(player["track_id"], [])
            threshold = self.cfg["registration_confidence"] if self.phase in ("preparing", "paused") else self.cfg["confidence"]
            geometry = body_geometry(candidates[0][1], threshold) if len(candidates) == 1 else None
            if geometry is None:
                found.append(None)
                continue
            old = player["geometry"]
            # A recycled ID or implausible discontinuity is not proof that the
            # participant moved: pause instead of accusing a replacement.
            if math.dist(geometry[2], old[2]) > 2.5 * old[3] or not .45 < geometry[3]/old[3] < 2.2:
                found.append(None)
                continue
            player.update(geometry=geometry, seen=now, frame_index=candidates[0][0])
            found.append(candidates[0][1])
        return found

    def host_pose(self, payload):
        now = self.clock()
        if self.phase == "registering":
            self.registration(payload)
            return
        if self.phase not in ("preparing", "countdown", "cue", "watch", "intermission", "paused") or not self.players:
            return
        found = self.matched_players(payload, now)
        if self.phase in ("preparing", "paused"):
            valid = all(person is not None and standing(player["geometry"]) and
                        body_geometry(person, self.cfg["confidence"]) is not None and
                        all(w in player["geometry"][0] and
                            player["geometry"][0][w][1] > player["geometry"][0][s][1] + .4*player["geometry"][3] and
                            (e not in player["geometry"][0] or player["geometry"][0][w][1] >= player["geometry"][0][e][1] - .1*player["geometry"][3])
                            for s,e,w in ((5,7,9),(6,8,10))) and
                        self.hands_clear(payload, visible_points(person.get("keypoints"), self.cfg["confidence"]))
                        for person, player in zip(found, self.players))
            if self.last_observation is None or now - self.last_observation > self.cfg["pose_timeout"]:
                self.ready_since, self.ready_frames = None, 0
            self.last_observation = now
            if not valid:
                self.ready_since, self.ready_frames, self.ready = None, 0, False
                if now - self.last_guidance >= 7 and now >= self.listen_after:
                    self.say("请已登记的玩家放下手站好，露出肩膀和双腿。" + ("双手也要清楚可见。" if self.cfg["detect_fingers"] else ""))
                    self.last_guidance = now
                return
            self.ready_since = now if self.ready_since is None else self.ready_since
            self.ready_frames += 1
            self.ready = self.ready_frames >= 5 and now - self.ready_since >= .8
            if self.ready and now >= self.preparation_not_before:
                if self.phase == "paused":
                    self.repeat_round = True
                    self.say("看清大家了，这一轮重新开始。")
                    self.phase = "intermission"
                    self.deadline = self.listen_after
                else:
                    self.advance("countdown")
            return
        if self.phase != "watch":
            return
        # A missing participant invalidates consecutive motion evidence.
        self.observation_complete = all(person is not None for person in found)
        if not self.observation_complete:
            for player in self.players:
                player["detector"].invalid_pose()
            return
        for player, person in zip(self.players, found):
            detector = player["detector"]
            detector.pose({**payload, "persons": [person]})
            if detector.phase == "lost":
                self.moved_player, self.moved_track_id = player["number"], player["track_id"]
                self.moved_part, self.highlight, self.hands = detector.moved_part, list(detector.highlight), detector.hands
                self.frame = payload
                self.finish("lost", "movement", f"{player['number']}号玩家，你的{PARTS[self.moved_part][0]}动了，游戏结束。")
                return
        self.valid_frames = min(p["detector"].valid_frames for p in self.players)
        self.last_pose = now

    def pause(self):
        self.phase = "paused"
        self.ready = False
        self.ready_since, self.ready_frames, self.last_observation = None, 0, None
        self.say("有玩家被遮挡或没有看清，暂停一下。请原来的玩家重新站好，这一轮会重新开始。")
        self.preparation_not_before = self.listen_after
        self.deadline = self.listen_after + self.cfg["recovery_timeout"]
        self.last_guidance = self.clock()


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


    def status(self):
        return dict(phase=self.phase, round=self.round, rounds=3, reason=self.reason,
                    valid_frames=self.valid_frames, moved_part=self.moved_part,
                    moved_part_label=PARTS[self.moved_part][0] if self.moved_part else "",
                    highlight_indices=list(self.highlight), sensitivity=self.cfg["sensitivity"],
                    detect_fingers=self.cfg["detect_fingers"],
                    result=self.result, ready=self.ready, expected_players=self.expected_players,
                    registered_players=len(self.players), registration_candidates=self.registration_candidates,
                    registration_debug=list(self.registration_debug),
                    registration_confidence=self.cfg["registration_confidence"],
                    registration_confirmed_ids=[p[0] for p in self.registration_pending],
                    registration_hold_seconds=round(self.registration_elapsed, 2),
                    moved_player=self.moved_player, moved_track_id=self.moved_track_id,
                    players=[dict(number=p["number"], track_id=p["track_id"],
                                  visible=self.clock() - p["seen"] <= self.cfg["pose_timeout"],
                                  frame_index=p["frame_index"],
                                  valid_frames=p["detector"].valid_frames if p["detector"] else 0,
                                  hands=p["detector"].status()["hands"] if p["detector"] else {})
                             for p in self.players],
                    hands={side: {"samples": h["samples"], "visible": h["visible"],
                                  "ready": h["visible"] and h["samples"] >= 3 and h["last"] is not None and self.clock() - h["last"] <= self.cfg["pose_timeout"]}
                           for side, h in self.hands.items()})

    def text(self, payload):
        background = False
        if isinstance(payload, dict):
            if payload.get("is_final") is False or payload.get("final") is False:
                return
            background = payload.get("background") is True
            if self.cfg["host_interaction"] and payload.get("audio_start_ts") is not None:
                key = (payload.get("audio_start_ts"), payload.get("audio_end_ts"), payload.get("text", payload.get("transcript")))
                if key in self.seen_utterances:
                    return
                self.seen_utterances = (self.seen_utterances + [key])[-32:]
            payload = payload.get("text", payload.get("transcript", ""))
        if not isinstance(payload, str):
            return
        text = clean_text(payload)
        if self.cfg["host_interaction"]:
            self.host_text(text, background)
            return
        restart = text == "游戏开始" or (self.phase in ("won", "lost", "error") and self.replay_prompted and text in ("好", "好的", "可以", "再来", "再来一次", "再玩一次"))
        if self.phase in ("waiting", "won", "lost", "error") and restart:
            self.reset()
            self.advance("countdown")
        elif self.phase in ("won", "lost", "error") and self.replay_prompted and text in ("不玩了", "不玩", "结束", "不用了"):
            self.reset()
            self.say("好的，休息一下，想玩时再叫我。")

    def host_text(self, text, background):
        text = text.removeprefix("小范小范")
        if text in self.spoken:
            return
        if self.phase != "waiting" and any(word in text for word in ("取消游戏", "结束游戏", "不玩了", "停止游戏")):
            self.reset()
            self.say("好的，游戏已取消，想玩时再叫我。")
            return
        if self.clock() < self.listen_after:
            return
        # Background sentences may answer an open question, but may not wake
        # an idle game. ASR remains responsible for the configured wake word.
        if self.phase in ("waiting", "won", "lost", "error"):
            replay = self.replay_prompted and text in ("好", "好的", "可以", "再来", "再来一次", "再玩一次")
            if (not background and invite(text)) or replay:
                self.reset()
                self.ask("rules_question", "好啊好啊，需要我介绍规则吗？")
            elif self.replay_prompted and text in ("不玩", "不用了", "结束"):
                self.reset()
                self.say("好的，休息一下，想玩时再叫我。")
        elif self.phase == "rules_question":
            if any(word in text for word in ("不用", "不需要", "不要介绍", "不想听", "别介绍", "不介绍", "直接开始", "直接玩", "跳过")) or text in ("不要", "不了"):
                self.ask_count()
            elif text in ("好", "好啊", "好的", "要", "需要", "是", "可以") or "介绍" in text or "规则" in text:
                self.phase = "rules_intro"
                self.say("我会说一二三木头人，说完以后大家保持不动。任何一位玩家动了，本局就结束。大家一起坚持三轮，就算挑战成功。")
                self.deadline = self.listen_after
        elif self.phase == "player_count":
            count = player_count(text)
            if count is not None and 1 <= count <= self.cfg["max_players"]:
                self.expected_players = count
                self.enroll()
            else:
                self.ask("player_count", f"请说一个人、两个人这样的具体人数，目前支持一到{int(self.cfg['max_players'])}人。")

    def finish(self, phase, reason, words):
        self.phase, self.reason = phase, reason
        self.result = phase
        self.say(words)
        self.prompt_at = self.listen_after if self.cfg["host_interaction"] else self.clock() + self.cfg["cue_seconds"]


    def cue(self):
        if not self.repeat_round:
            self.round += 1
        self.repeat_round = False
        self.phase = "cue"
        self.baseline = None
        self.track_id = None
        self.motion_count = self.valid_frames = 0
        self.last_pose = None
        self.part_counts = {}
        self.hands = {}
        self.highlight = []
        self.moved_part = ""
        self.moved_player = self.moved_track_id = None
        self.observation_complete = False
        for player in self.players:
            detector = Game(lambda _: None, {**self.cfg, "host_interaction": False}, self.clock)
            detector.phase = "watch"
            player["detector"] = detector
        self.deadline = self.clock() + self.cfg["cue_seconds"] + self.cfg["settle_seconds"]
        self.say("一，二，三，木头人！")

    def invalid_pose(self):
        self.part_counts.clear()
        for h in self.hands.values():
            h.update(count=0, visible=False, signature=None, baseline=None, samples=0)

    def pose(self, payload):
        if not isinstance(payload, dict):
            return
        if not isinstance(payload.get("persons"), list):
            return
        if self.cfg["host_interaction"]:
            self.host_pose(payload)
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
        if self.cfg["host_interaction"]:
            self.host_tick(now)
            return
        if self.phase == "preparing" and now >= self.deadline:
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
                        self.phase = "intermission"
                        self.deadline = now + self.cfg["cue_seconds"]
                    else:
                        self.cue()
        elif self.phase in ("won", "lost", "error") and self.cfg["host_interaction"] and self.prompt_at is not None and now >= self.prompt_at and not self.replay_prompted:
            self.replay_prompted = True
            self.say("再挑战一次吗？")

    def host_tick(self, now):
        if self.phase in ("rules_question", "player_count") and now >= self.deadline:
            self.reset()
            self.say("暂时没有听到回答，我先等你，想玩时再叫我。")
        elif self.phase == "rules_intro" and now >= self.deadline:
            self.ask_count()
        elif self.phase in ("registering", "preparing", "paused") and now >= self.deadline:
            reason = {"registering": "registration_timeout", "preparing": "preparation_timeout", "paused": "recovery_timeout"}[self.phase]
            self.finish("error", reason, "还没有确认所有玩家，请调整站位。重新玩时需要再次登记，游戏结束。")
        elif self.phase == "intermission" and now >= self.deadline:
            self.cue()
        elif self.phase == "countdown" and now >= self.deadline:
            self.cue()
        elif self.phase == "cue" and now >= self.deadline:
            self.phase = "watch"
            self.watch_start = now
            self.deadline = now + self.cfg["watch_seconds"]
        elif self.phase == "watch":
            if any(now - max(p["seen"], self.watch_start) > self.cfg["pose_timeout"] for p in self.players):
                self.pause()
            elif now >= self.deadline:
                # Every enrolled player must independently pass. A short loss
                # of vision is never evidence that nobody moved.
                detectors = [p["detector"] for p in self.players]
                complete = bool(detectors) and self.observation_complete and all(d.valid_frames >= 5 and d.last_pose is not None and
                           now - d.last_pose <= self.cfg["pose_timeout"] for d in detectors)
                if self.cfg["detect_fingers"]:
                    complete = complete and all(all(d.status()["hands"].get(s, {}).get("ready")
                                                     for s in ("left", "right")) for d in detectors)
                if not complete:
                    self.pause()
                elif self.round == 3:
                    self.finish("won", "three_rounds", "三轮完成，大家赢了，游戏结束！")
                else:
                    self.say(f"不错，第{('二' if self.round == 1 else '三')}轮来了，准备好。")
                    self.phase = "intermission"
                    self.deadline = self.listen_after
        elif self.phase in ("won", "lost", "error") and self.prompt_at is not None and now >= self.prompt_at and not self.replay_prompted:
            self.replay_prompted = True
            self.say("再挑战一次吗？")


def render_highlight(payload, status, confidence=0.5):
    """JPEG skeleton view, deliberately not an unsynchronised RGB overlay."""
    from io import BytesIO
    from PIL import Image, ImageDraw, ImageFont
    if status["phase"] == "registering":
        confidence = status.get("registration_confidence", confidence)
    image = Image.new("RGB", (800, 600), "#101c2c")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
    draw.text((20, 12), f"WOODEN MAN | {status['phase'].upper()} | Round {status['round']}/3", fill="white", font=font)
    part = status.get("moved_part", "")
    player_label = f"P{status['moved_player']} " if status.get("moved_player") else ""
    subtitle = ("MOVED: " + player_label + PARTS[part][1] + " (player's side) - verdict frozen") if part else "Pose skeleton | red = judged moving part | not RGB"
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
    for record_index, visible in enumerate(records):
        coords = {i: (ox + x * ratio, oy + y * ratio) for i, (x, y) in visible.items()
                  if 0 <= x <= size[0] and 0 <= y <= size[1]}
        person = [p for p in persons if isinstance(p, dict)][record_index]
        enrolled = next((p for p in status.get("players", []) if p["track_id"] == person.get("id")), None)
        selected = set(status.get("highlight_indices", [])) if (
            status.get("moved_track_id") is not None and person.get("id") == status["moved_track_id"]
            or status.get("moved_track_id") is None and len(records) == 1) else set()
        if coords:
            x, y = coords.get(0, next(iter(coords.values())))
            label = f"P{enrolled['number']}" if enrolled else "Background"
            if status["phase"] == "registering":
                report = next((r for r in status.get("registration_debug", []) if r["track_id"] == person.get("id")), {})
                reason = report.get("reason", "body_not_clear")
                label = f"#{person.get('id', '?')} " + ("CONFIRMED" if person.get("id") in status.get("registration_confirmed_ids", []) else {
                    "candidate": "HOLD HAND", "body_not_clear": "BODY UNCLEAR", "not_standing": "CHECK LEGS",
                    "hand_not_raised": "RAISE HAND / SHOW WRIST", "missing_track_id": "NO TRACK ID"}.get(reason, reason))
                if reason == "not_standing" and report.get("posture") in ("sitting", "lying"):
                    label = f"#{person.get('id', '?')} {report['posture'].upper()} / IGNORE"
            draw.text((x, max(80, y-24)), label, fill="#ff6470" if selected else "white", font=font)
        edges = list(BODY_EDGES)
        for side in ("left", "right"):
            hand = hand_indices(payload, side)
            if len(hand) == 21:
                for start in (1, 5, 9, 13, 17):
                    chain = [hand[0], *hand[start:start + 4]]
                    edges.extend(zip(chain, chain[1:]))
        for a, b in edges:
            if a in coords and b in coords:
                draw.line((coords[a], coords[b]), fill="#ff4058" if a in selected or b in selected else "#3bbfb4" if enrolled or not status.get("players") else "#526276", width=4)
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
        if status.get("players"):
            label = " | ".join(f"P{p['number']} hands: " + ("READY" if all(p["hands"].get(s, {}).get("ready") for s in ("left", "right")) else "NOT READY") for p in status["players"])
    else:
        label = "Finger detection OFF | enable hands in Pose and fingers in game"
    if status["phase"] == "registering":
        confirmed = len(status.get("registration_confirmed_ids", []))
        count = f"Confirmed {confirmed}" if confirmed else f"Candidates {status.get('registration_candidates', 0)}"
        label = f"{count}/{status.get('expected_players', 0)} | Hold {status.get('registration_hold_seconds', 0):.1f}s | wrist near shoulder, above elbow"
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
        status_pub = node.create_publisher(String, status_topic, 10)
        image_pub = node.create_publisher(CompressedImage, status_topic.rsplit("/", 1)[0] + "/highlight", 1)

        def say(words):
            msg = String()
            msg.data = encode_speech(words)
            pub.publish(msg)

        game = Game(say, config)
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class WoodenManPlugin:
    def __init__(self, config, namespace):
        self.config = effective_config({k: v for k, v in config.items() if k in DEFAULTS or k in OPTIONS})
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
                "description": "单人／多人三轮木头人：自然语言邀请、规则问答、人数确认、站立举手登记、姿态与手指识别、玩家部位播报和高亮；机器人固定站立，不控制关节。",
                "inputSchema": {"type": "object", "properties": props, "required": ["action"],
                                "x-action-params": {
                                    "start": {"params": ["input_topic", "input_topics", "asr_topic", "pose_topic"], "description": "连接画布，等待邀请玩木头人游戏"},
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
                               "desc": "判定骨架高亮（非RGB），红色为移动部位，失败时定格"}],
                "configSchema": {"type": "object", "properties": {
                    "sensitivity": {"type": "string", "title": "灵敏度预设",
                                    "enum": ["relaxed", "standard", "sensitive", "custom"],
                                    "default": "standard", "description": "relaxed 宽松 / standard 标准 / sensitive 灵敏 / custom 自定义。阈值越小越灵敏。"},
                    "detect_fingers": {"type": "boolean", "title": "检测手指动作", "default": False,
                                       "description": "需同时开启Pose的hands；双手须清晰可见，缺少手部数据不会判过关。"},
                    "host_interaction": {"type": "boolean", "title": "对话主持与多人登记", "default": True,
                                         "description": "自然邀请→规则问答→人数→站立举手登记。关闭仅保留旧版游戏开始口令的单人直入模式。"},
                    "asr_topic": {"type": "string", "advanced": True, "description": "可选：画布ASR输出topic"},
                    "pose_topic": {"type": "string", "advanced": True, "description": "可选：画布Pose输出topic"},
                    **{k: {"type": "integer" if k in ("moving_frames", "finger_frames", "max_players") else "number", "default": v, "exclusiveMinimum": 0,
                           "title": {"cue_seconds": "口令播报预算（秒）", "settle_seconds": "站稳等待（秒）",
                                     "watch_seconds": "每轮观察时长（秒）", "motion_threshold": "身体位移阈值／躯干长度",
                                     "moving_frames": "身体连续超阈帧数", "pose_timeout": "感知超时（秒）",
                                     "confidence": "关键点最低置信度", "finger_threshold": "手指位移阈值／手掌长度",
                                     "finger_frames": "手指连续超阈采样数", "preparation_timeout": "准备站位超时（秒）",
                                     "dialogue_timeout": "播报后等待回答（秒）", "registration_timeout": "举手登记超时（秒）",
                                     "registration_seconds": "连续举手确认（秒）", "recovery_timeout": "遮挡后恢复等待（秒）",
                                     "registration_confidence": "登记关键点最低置信度（不影响判动）",
                                     "registration_grace": "举手识别短暂抖动容忍（秒）",
                                     "max_players": "允许的最多玩家数"}[k],
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
        if config["max_players"] != int(config["max_players"]) or config["max_players"] > 10:
            raise ValueError("max_players须为1到10的整数")
        if config["registration_confidence"] > 1:
            raise ValueError("registration_confidence须在0到1之间")
        if any(config[k] != int(config[k]) for k in ("moving_frames", "finger_frames")) or config["confidence"] > 1 or config["watch_seconds"] <= config["pose_timeout"]:
            raise ValueError("moving_frames和finger_frames须为整数，confidence≤1，watch_seconds须大于pose_timeout")
        for key in OPTIONS:
            config[key] = args.get(key, supplied.get(key, config[key]))
        if not isinstance(config["sensitivity"], str) or config["sensitivity"] not in (*PRESETS, "custom"):
            raise ValueError("无效参数: sensitivity")
        for key in ("detect_fingers", "host_interaction"):
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
        topics = dialogue_topics(topics, config["host_interaction"])
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
