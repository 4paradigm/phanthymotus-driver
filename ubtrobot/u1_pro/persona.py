"""Persona card for UBTECH U1 Pro.

The card owns three things that a prompt alone cannot guarantee:

* deterministic routing of one ASR utterance to a persona
  (safety > stop > explicit request > keyword score);
* the identity announcement is spoken by the driver itself, before any reply
  submitted through ``persona.say`` (one serial queue, no concurrent playback);
* interrupt / safety handling that clears queued speech.

Reply content is still produced by the agent; this card only routes, speaks and
sequences.  Persona definitions are configurable through ``persona`` in
``config.yaml``; the four default personas are 晴姐 / 安宁 / 岚总 / 清照.
"""

from __future__ import annotations

import collections
import json
import re
import threading
import time
import uuid
from typing import Any, Callable

from common.vendor_runtime import action_schema, tool


DEFAULT_PERSONAS: list[dict[str, Any]] = [
    {
        "id": "qingjie",
        "name": "晴姐",
        "aliases": ["晴姐", "晴姐姐"],
        "announce": "现在由晴姐在线陪你。",
        "priority": 1,
        "keywords": ["沮丧", "失落", "失败", "挫败", "没用", "无力", "努力没结果",
                     "不想做", "提不起劲", "没做好", "灰心", "泄气"],
        "style": "知性温柔的大姐姐。先接住沮丧和失落，不训话、不催促振作；短句陪对方把感受说清，"
                 "再给一个小而可选的自我照顾方向。",
        "expressions": ["gaze", "smile", "sad", "blink"],
    },
    {
        "id": "anning",
        "name": "安宁",
        "aliases": ["安宁"],
        "announce": "现在由安宁在线听你说。",
        "priority": 2,
        "keywords": ["心情不好", "难受", "烦", "焦虑", "压力大", "睡不着", "想倾诉", "想聊聊",
                     "情绪不好", "内耗", "郁闷", "紧张"],
        "style": "平静、非评判的倾听者。先确认感受，再问对方想被倾听还是一起梳理。"
                 "不是执业心理医生：不诊断、不贴标签、不开药、不承诺疗效或保密。",
        "expressions": ["gaze", "thinking", "blink"],
    },
    {
        "id": "lanzong",
        "name": "岚总",
        "aliases": ["岚总"],
        "announce": "现在由岚总在线处理。",
        "priority": 3,
        "keywords": ["帮帮我", "怎么办", "帮我", "计划", "安排", "决定", "选择", "任务", "汇报",
                     "方案", "卡住", "解决问题", "截止", "deadline"],
        "style": "果断有条理的女总裁。先确认目标和限制，再给不超过三步的行动方案。"
                 "不羞辱、不施压，不替对方决定医疗、法律、金融或亲密关系等重大事项。",
        "expressions": ["got_it", "thinking", "smile"],
    },
    {
        "id": "qingzhao",
        "name": "清照",
        "aliases": ["清照", "易安", "李清照"],
        "announce": "此刻清照在线，与你共话一阕词。",
        "priority": 4,
        "keywords": ["李清照", "易安", "清照", "宋词", "婉约", "词牌", "如梦令", "声声慢", "醉花阴",
                     "武陵春", "写词", "填词", "背词", "对词", "赏词", "海棠", "黄花", "梧桐",
                     "相思", "古风", "诗词"],
        "style": "参考李清照诗词气质的文化角色，温婉敏锐。可引用已知李清照词句；新作须说明"
                 "“这是我此刻新写的两句”。不声称自己是真实历史人物，不编造传世作品。",
        "expressions": ["smile", "thinking", "shy", "gaze"],
    },
]

DEFAULT_SAFETY_KEYWORDS = [
    "自杀", "不想活", "活不下去", "想死", "轻生", "结束生命", "伤害自己", "自残", "割腕", "跳楼",
    "杀了他", "杀人", "伤害别人", "报复社会", "喘不上气", "胸口疼", "胸痛", "晕倒", "救命", "着火",
]
DEFAULT_SAFETY_MESSAGE = ("听到你这么说，我很担心你的安全。请马上告诉身边信任的人或现场工作人员；"
                          "如果有紧急危险，请立刻拨打急救电话一二零。")
DEFAULT_STOP_PHRASES = ["停下", "停一下", "先停", "停止", "暂停", "别说了", "不要说了", "安静", "闭嘴"]
DEFAULT_STOP_ACK = "好的，我先停下。"
NEGATION_CHARS = "不没别未无"
EXPLICIT_VERBS = r"(?:切换到|切换成|切到|换成|换到|换回|换|叫|请|让|找|要)"
_PUNCT = re.compile(r"[\s，。！？、,.!?；;：:“”\"'…~～]+")


def _clean(text: str) -> str:
    return _PUNCT.sub("", str(text or ""))


class PersonaRouter:
    """Pure routing logic; no I/O so it can be tested and reused."""

    def __init__(self, config: dict | None = None):
        config = config or {}
        personas = config.get("personas") or DEFAULT_PERSONAS
        self.personas: dict[str, dict[str, Any]] = {}
        for raw in personas:
            persona = dict(raw)
            pid = str(persona.get("id") or "").strip()
            if not pid or not persona.get("announce"):
                raise ValueError("each persona needs id and announce")
            persona.setdefault("name", pid)
            persona["aliases"] = list(persona.get("aliases") or [persona["name"]])
            persona["keywords"] = [str(k) for k in persona.get("keywords") or [] if str(k)]
            persona["priority"] = int(persona.get("priority", 0))
            persona.setdefault("style", "")
            persona["expressions"] = list(persona.get("expressions") or [])
            self.personas[pid] = persona
        self.safety_keywords = list(config.get("safety_keywords") or DEFAULT_SAFETY_KEYWORDS)
        self.stop_phrases = list(config.get("stop_phrases") or DEFAULT_STOP_PHRASES)
        self._explicit = {
            pid: re.compile("|".join(
                [f"{re.escape(a)}模式" for a in p["aliases"]] +
                [f"{EXPLICIT_VERBS}{re.escape(a)}" for a in p["aliases"]]))
            for pid, p in self.personas.items()
        }

    @staticmethod
    def _hits(text: str, keyword: str) -> bool:
        """True if keyword occurs at least once without a negation just before it."""
        start = text.find(keyword)
        while start >= 0:
            prefix = text[max(0, start - 2):start]
            if not any(ch in NEGATION_CHARS for ch in prefix):
                return True
            start = text.find(keyword, start + 1)
        return False

    def _is_stop(self, text: str) -> bool:
        if len(text) > 10:
            return False
        for phrase in self.stop_phrases:
            idx = text.find(phrase)
            if idx < 0:
                continue
            after = text[idx + len(phrase):idx + len(phrase) + 1]
            before = text[max(0, idx - 1):idx]
            if after != "不" and before not in ("不", "没"):
                return True
        return False

    def analyze(self, text: str, current: str | None) -> dict[str, Any]:
        cleaned = _clean(text)
        if not cleaned:
            return {"intent": "none", "persona": current, "matches": []}
        safety = [k for k in self.safety_keywords if k in cleaned]
        if safety:
            return {"intent": "safety", "persona": None, "matches": safety}
        if self._is_stop(cleaned):
            return {"intent": "stop", "persona": current, "matches": []}
        explicit = [pid for pid, pattern in self._explicit.items() if pattern.search(cleaned)]
        if explicit:
            pid = max(explicit, key=lambda p: self.personas[p]["priority"])
            return {"intent": "switch", "persona": pid, "matches": ["explicit"], "explicit": True}
        scores: dict[str, list[str]] = {}
        for pid, persona in self.personas.items():
            hits = [k for k in persona["keywords"] if self._hits(cleaned, k)]
            if hits:
                scores[pid] = hits
        if not scores:
            return {"intent": "none", "persona": current, "matches": []}
        best = max(len(h) for h in scores.values())
        tied = [pid for pid, hits in scores.items() if len(hits) == best]
        if current in tied:
            pid = current
        else:
            pid = max(tied, key=lambda p: self.personas[p]["priority"])
        return {"intent": "switch", "persona": pid, "matches": scores[pid], "scores":
                {p: len(h) for p, h in scores.items()}}


class _Job:
    __slots__ = ("kind", "payload", "action_id", "generation", "public")

    def __init__(self, kind: str, payload: dict, action_id: str, generation: int, public: bool):
        self.kind, self.payload, self.action_id = kind, payload, action_id
        self.generation, self.public = generation, public


class PersonaPlugin:
    """Agent-facing ``persona`` card."""

    PREFIX = "persona"

    def __init__(self, audio, notify: Callable[[str, str, dict, str], None],
                 config: dict | None = None, expression_names: list[str] | None = None,
                 subscribe: Callable[[str, Callable[[str], None]], Any] | None = None,
                 unsubscribe: Callable[[Any], None] | None = None):
        config = config or {}
        self.audio = audio
        self.notify = notify
        self.router = PersonaRouter(config)
        self.safety_message = str(config.get("safety_message") or DEFAULT_SAFETY_MESSAGE)
        self.stop_ack = str(config.get("stop_ack", DEFAULT_STOP_ACK) or "")
        self.playback_timeout = float(config.get("playback_timeout", 60.0))
        self.busy_timeout = float(config.get("busy_timeout", 30.0))
        self.poll_interval = float(config.get("poll_interval", 0.05))
        self.dedup_window = float(config.get("dedup_window", 5.0))
        self.expression_names = set(expression_names or [])
        self._subscribe, self._unsubscribe = subscribe, unsubscribe
        self._subscription = None
        self.input_topic = ""
        self.running = False
        self.current: str | None = None
        self.mode = "idle"  # idle | persona | safety
        self.last_error = ""
        self._last_route: tuple[str, float, dict] | None = None
        self._generation = 0
        self._jobs: collections.deque[_Job] = collections.deque()
        self._cond = threading.Condition()
        self._route_lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._closing = False

    # ------------------------------------------------------------------ schema
    def get_tool(self):
        actions = {
            "start": ([], "启动人格卡片；可选 input_topic 连接 asr 卡片的 data/json 输出，自动路由识别结果。"),
            "route": (["text"], "把本轮 ASR 文本交给卡片判定人格。若需切换，卡片会先自行播报身份，"
                                "之后提交的 say 一定排在播报之后。返回当前人格与风格要求。"),
            "say": (["text"], "以当前人格播报一段回复（1-3 句），可选 expression；严格串行，不与其他播报并发。"),
            "set": (["persona"], "直接切换到指定人格并播报身份（运营/调试用）。"),
            "reset": ([], "清空队列、打断播报并退出人格，不播报。"),
            "interrupt": ([], "立即打断当前播报并清空待播队列。"),
            "stop": ([], "停止卡片，打断播报并断开自动路由。"),
            "info": ([], "读取当前人格、模式、可用人格及其可用表情、队列长度。"),
        }
        schema = action_schema(actions, {
            "text": {"type": "string", "minLength": 1, "description": "ASR 文本（route）或要播报的回复（say）。"},
            "persona": {"type": "string", "enum": sorted(self.router.personas), "description": "人格 id。"},
            "expression": {"type": "string", "enum": sorted(self._all_expressions()) or [""],
                           "description": "可选；只能取当前人格允许且本机可用的表情。"},
            "input_topic": {"type": "string", "description": "可选；asr 卡片输出的 data/json 话题。"},
            "action_id": {"type": "string", "description": "可选 ACP 关联 ID。"},
        })
        schema["x-completion"] = {"actions": ["say"], "timeout": 120}
        # Hooks bypass the ACP barrier, so a user barge-in or e-stop clears the
        # queue even while a long reply is pending.  x-resource is intentionally
        # NOT declared: no other U1 Pro card declares it, and a partially declared
        # driver is worse than an undeclared one (README_dev, "Physical Resources").
        schema["x-hooks"] = {
            "on_interrupt_speak": {"action": "interrupt"},
            "on_interrupt_all": {"action": "interrupt"},
        }
        return tool(self.PREFIX, "actuator",
                    "U1 Pro 多人格陪伴：安全 > 停止 > 明确点名 > 关键词评分 路由人格；切换时由驱动先播报身份，"
                    "回复必须经 persona.say 串行播报。不生成回复内容。",
                    schema, topic_in=[{"format": "data/json"}])

    def _all_expressions(self) -> set[str]:
        names: set[str] = set()
        for persona in self.router.personas.values():
            names.update(self._allowed_expressions(persona))
        return names

    def _allowed_expressions(self, persona: dict | None) -> list[str]:
        if not persona:
            return []
        return [e for e in persona["expressions"] if e in self.expression_names]

    def _persona_view(self, pid: str | None) -> dict | None:
        persona = self.router.personas.get(pid) if pid else None
        if not persona:
            return None
        return {"id": pid, "name": persona["name"], "announce": persona["announce"],
                "style": persona["style"], "expressions": self._allowed_expressions(persona)}

    # --------------------------------------------------------------- lifecycle
    def start(self, input_topic: str = ""):
        with self._cond:
            self._closing = False
            if not self._worker or not self._worker.is_alive():
                self._worker = threading.Thread(target=self._run, name="u1-persona", daemon=True)
                self._worker.start()
        self.running = True
        result = {"state": "ready"}
        if input_topic and input_topic != self.input_topic:
            if not self._subscribe:
                return {"state": "error", "message": "auto routing is not available on this driver"}
            self._drop_subscription()
            self._subscription = self._subscribe(input_topic, self._on_asr_message)
            self.input_topic = input_topic
        if self.input_topic:
            result["input_topic"] = self.input_topic
        return result

    def stop(self):
        self._drop_subscription()
        self._cancel_all(interrupt=True)
        with self._cond:
            self._closing = True
            self._cond.notify_all()
        self.running = False
        return {"state": "idle"}

    def _drop_subscription(self):
        if self._subscription is not None and self._unsubscribe:
            try:
                self._unsubscribe(self._subscription)
            except Exception:
                pass
        self._subscription = None
        self.input_topic = ""

    # ------------------------------------------------------------------ routing
    def _on_asr_message(self, raw: str) -> None:
        try:
            data = json.loads(raw)
        except (TypeError, ValueError):
            data = {"text": raw}
        if not isinstance(data, dict):
            return
        if data.get("is_final") is False or data.get("final") is False:
            return
        text = str(data.get("text") or data.get("result") or "").strip()
        if text:
            try:
                self.route(text)
            except Exception as exc:  # never break the ROS callback
                self.last_error = str(exc)[:256]

    def route(self, text: str) -> dict:
        text = str(text or "").strip()
        now = time.monotonic()
        with self._route_lock:
            # The same utterance may arrive from the asr topic and from the agent.
            if self._last_route and self._last_route[0] == text and now - self._last_route[1] < self.dedup_window:
                return dict(self._last_route[2], deduplicated=True)
            decision = self.router.analyze(text, self.current)
            result = self._apply(decision)
            self._last_route = (text, now, result)
            return result

    def _apply(self, decision: dict) -> dict:
        intent = decision["intent"]
        previous = self.current
        announced = None
        if intent == "safety":
            self._cancel_all(interrupt=True)
            self.current, self.mode = None, "safety"
            announced = self._enqueue_text(self.safety_message, public=False)
            return {"intent": "safety", "persona": None, "previous": previous, "switched": previous is not None,
                    "spoken": announced, "matches": decision["matches"],
                    "guidance": "已退出人格并播报安全提示。继续用平静中性语气，经 persona.say 回应；"
                                "不开玩笑、不淡化、不承诺保密，建议联系身边人、工作人员或急救。"}
        if intent == "stop":
            self._cancel_all(interrupt=True)
            if self.stop_ack:
                announced = self._enqueue_text(self.stop_ack, public=False)
            return {"intent": "stop", "persona": self.current, "previous": previous, "switched": False,
                    "spoken": announced, "guidance": "已打断并清空播报。本轮不要再回答。"}
        if intent == "switch" and self.mode == "safety" and not decision.get("explicit") \
                and decision.get("matches") != ["set"]:
            # Stay neutral after a safety event; keywords such as "失败" must not
            # pull the conversation back into a playful persona.
            intent = "none"
        if intent == "switch":
            pid = decision["persona"]
            switched = pid != self.current
            self.current, self.mode = pid, "persona"
            if switched:
                announced = self._enqueue_text(self.router.personas[pid]["announce"], public=False)
            return {"intent": "switch", "persona": self._persona_view(pid), "previous": previous,
                    "switched": switched, "spoken": announced, "matches": decision["matches"],
                    "guidance": "身份播报已由卡片排队；直接用 persona.say 以该人格回复，无需自行播报身份。"}
        if self.mode == "safety":
            return {"intent": "none", "persona": None, "previous": previous, "switched": False, "spoken": None,
                    "guidance": "仍处于安全处置模式：保持中性语气；确认对方安全后可用 reset 或 set 恢复人格。"}
        return {"intent": "none", "persona": self._persona_view(self.current), "previous": previous,
                "switched": False, "spoken": None,
                "guidance": "未命中切换条件，沿用当前人格；无人格时用中性语气回应。"}

    # ------------------------------------------------------------------ speech
    def _enqueue_text(self, text: str, action_id: str | None = None, public: bool = True,
                      expression: str | None = None) -> dict:
        action_id = str(action_id or uuid.uuid4())[:128]
        with self._cond:
            generation = self._generation
            if expression:
                self._jobs.append(_Job("play_action", {"name": expression}, f"persona-expr-{uuid.uuid4()}",
                                       generation, False))
            self._jobs.append(_Job("play_text", {"text": text[:4096]}, action_id, generation, public))
            queued = len(self._jobs)
            self._cond.notify_all()
        if not self._worker or not self._worker.is_alive():
            self.start()
        return {"text": text, "action_id": action_id, "queued": queued}

    def _cancel_all(self, interrupt: bool) -> None:
        with self._cond:
            self._generation += 1
            dropped = list(self._jobs)
            self._jobs.clear()
            self._cond.notify_all()
        for job in dropped:
            if job.public:
                self.notify(job.action_id, "cancelled", {"state": "cancelled", "action_id": job.action_id},
                            self.PREFIX)
        if interrupt and self.audio.active_action_id():
            was_running = self.audio.running
            self.audio.stop()
            self.audio.running = was_running

    def _run(self) -> None:
        while True:
            with self._cond:
                while not self._jobs and not self._closing:
                    self._cond.wait()
                if self._closing and not self._jobs:
                    return
                job = self._jobs.popleft()
                current_generation = self._generation
            if job.generation != current_generation:
                self._drop(job)
                continue
            try:
                self._play(job)
            except Exception as exc:
                self.last_error = str(exc)[:256]

    def _stale(self, job: _Job) -> bool:
        with self._cond:
            return job.generation != self._generation

    def _drop(self, job: _Job) -> None:
        """Report a job that was cancelled before it reached the audio channel."""
        if job.public:
            self.notify(job.action_id, "cancelled", {"state": "cancelled", "action_id": job.action_id},
                        self.PREFIX)

    def _play(self, job: _Job) -> None:
        deadline = time.monotonic() + self.busy_timeout
        while True:
            if self._stale(job):
                self._drop(job)
                return
            if self.audio.active_action_id() is None:
                if job.kind == "play_action":
                    legacy = self.audio_expression(job.payload["name"])
                    if legacy is None:
                        return
                    result = self.audio._queue("play_action", {"action": legacy}, job.action_id, "expression")
                else:
                    result = self.audio._queue("play_text", job.payload, job.action_id, self.PREFIX)
                if result.get("state") == "queued":
                    break
            if time.monotonic() > deadline:
                self.last_error = "audio channel stayed busy"
                if job.public:
                    self.notify(job.action_id, "error", {"state": "error", "message": self.last_error,
                                                         "action_id": job.action_id}, self.PREFIX)
                return
            time.sleep(self.poll_interval)
        deadline = time.monotonic() + self.playback_timeout
        while self.audio.active_action_id() == job.action_id:
            if self._stale(job):
                return
            if time.monotonic() > deadline:
                self.last_error = "playback completion timed out"
                was_running = self.audio.running
                self.audio.stop()
                self.audio.running = was_running
                return
            time.sleep(self.poll_interval)

    # Resolved by build_plugins so this module does not import device.py.
    expression_lookup: Callable[[str], str | None] | None = None

    def audio_expression(self, name: str) -> str | None:
        return self.expression_lookup(name) if self.expression_lookup else None

    # ---------------------------------------------------------------- dispatch
    def dispatch(self, action, args):
        args = args or {}
        if action == "start":
            return self.start(str(args.get("input_topic") or ""))
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._cond:
                queued = len(self._jobs)
            return {"state": "ready" if self.running else "idle", "mode": self.mode,
                    "persona": self._persona_view(self.current), "queued": queued,
                    "input_topic": self.input_topic, "last_error": self.last_error,
                    "personas": [self._persona_view(pid) for pid in self.router.personas]}
        if action == "route":
            return self.route(str(args.get("text") or ""))
        if action == "say":
            text = str(args.get("text") or "").strip()
            if not text:
                raise ValueError("persona.say requires text")
            expression = str(args.get("expression") or "").strip() or None
            if expression:
                allowed = (self._allowed_expressions(self.router.personas.get(self.current))
                           if self.current else [])
                if expression not in allowed:
                    expression = None  # never block speech on a cosmetic action
            queued = self._enqueue_text(text, args.get("action_id"), True, expression)
            return {"state": "queued", "action_id": queued["action_id"], "queued": queued["queued"],
                    "persona": self.current, "expression": expression}
        if action == "set":
            pid = str(args.get("persona") or "")
            if pid not in self.router.personas:
                raise ValueError("persona.set requires a declared persona id")
            with self._route_lock:
                return self._apply({"intent": "switch", "persona": pid, "matches": ["set"]})
        if action == "reset":
            self._cancel_all(interrupt=True)
            self.current, self.mode = None, "idle"
            return {"state": "ready", "mode": "idle", "persona": None}
        if action == "interrupt":
            self._cancel_all(interrupt=True)
            return {"state": "ready", "persona": self.current}
        return None
