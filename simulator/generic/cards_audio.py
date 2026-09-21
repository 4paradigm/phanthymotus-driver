"""Speech: the `tts` and `speaker` cards.

Two different shapes, and they are not interchangeable. Every real `speaker` in
this repo — `unitree/g1`, `go2`, `r1`, `engineai/t800`, `noetix/bumi`,
`robotera/q5_bundle` — declares `topic_in: [{"format": "audio/pcm-16k"}]` and
plays whatever arrives on the stream. `x-humanoid/tianyi2.0`'s `voice_play` is
the other shape: call-driven `play_file` / `play_url` / `play_text`, no
`topic_in` because it consumes no stream. A `speaker` card once lived here that
copied the second while wearing the first's name, with no `topic_in` at all, so
nothing could be wired into it — it was removed, and this docstring said a
stream-consuming one was worth adding "when something needs the `topic_in` half
of the canvas contract".

**That is now the case.** A solution whose canvas binds perception's `tts`
synthesises real audio and publishes it — and the simulated world never hears a
word of it. On the benchmark's run log, the「世界真的做了什么」column stays empty
while the agent is plainly speaking, and every judgement about announcement
timing has nothing to read. The gap is not the agent's; it is that the world had
no ears.

So `speaker` is back, in the shape every real one has: it subscribes to the
stream the canvas wires into it and logs what a speaker can actually know —
**that audio played, and for how long**. Not what was said: PCM carries no text,
and a real speaker does not know it either. The run log's left column already
shows the text from the agent's own call; what the right column was missing is a
`speak_start` / `speak_end` pair at the right moments.

`tts` remains the call-driven mouth for solutions that want the simulator to own
synthesis too — it is the only way to check announcement ordering on an Orin,
neither of which has a real speaker.

Neither Orin has a real speaker, so a virtual `tts` card is the only way to
verify announcement ordering on those rigs at all. That is the single assertion
the exhibition tour exists for: the announcement for a waypoint must start
*after* the robot arrives there and finish *before* it leaves.

`tts` is named `tts` with an `interrupt` action on purpose — see
`cards_motion.py` for the `loco`/`tts` fallback lookup in `llm.py` that this
puts the card on.
"""

from __future__ import annotations

from simulator.generic import acp
from simulator.generic.card_base import Card


class _SpeechCard(Card):
    """Own the utterances this card created, and post ACP for exactly those.

    The ownership filter stays even with a single speech card: the world emits
    every terminal to every listener, so a second mouth added later would
    otherwise report this one's completions under its own tool name — silently,
    since ACP cannot tell which card a completion should have come from.
    """

    RESOURCES = ["mouth"]

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._owned: set[str] = set()
        world.add_speech_listener(self._on_terminal)

    def _on_terminal(self, payload: dict) -> None:
        action_id = payload.get("action_id")
        if action_id not in self._owned:
            return
        self._owned.discard(action_id)
        acp.notify(action_id, payload["status"], payload, tool=self.NAME)

    def _say(self, text: str) -> dict:
        utterance = self.world.speak(text)
        self._owned.add(utterance.id)
        return {"state": "running", "action_id": utterance.id,
                "text": utterance.text, "estimated_seconds": round(utterance.duration, 2)}


class SpeakerCard(Card):
    """A mouth that only listens to a stream.

    **Declares no `x-resource`.** The real ones do not either: the mouth is
    claimed by whatever *synthesises* (`tts`), and claiming it here too would
    make the barrier serialise a card against its own upstream — the speaker is
    playing precisely because tts is speaking.

    Silence is judged by a gap, not by an end-of-stream marker, because there is
    no such marker in a PCM stream. `QUIET_SECONDS` therefore sets how long a
    pause has to be before it counts as the end of an utterance; too short and a
    single sentence is logged as several.
    """

    NAME = "speaker"
    KIND = "actuator"
    DESCRIPTION = "虚拟扬声器 — 订阅 PCM 音频流并记录播报起止；接真实 tts 的输出"
    TOPIC = ""
    TOPIC_IN = [{"format": "audio/pcm-16k"}]
    ACTIONS = {
        "read": ([], "读取当前是否在播，以及累计播了几段"),
    }
    # `start` / `stop` 不列在这里：`Card.dispatch` 会把它们当生命周期动词拦下来，
    # 自己调无参的 `self.start()`，`do_start` 永远不会被调到。`cards_scenario.py`
    # 的注释记着这条，而这张卡第一版就是写成 `do_start` 的 —— 结果是启动成功、
    # `state: running`、`input_topic` 却始终为空：一张**聋着的**卡，看不出任何异常。
    PROPERTIES = {
        "input_topic": {"type": "string",
                        "description": "要订阅的 PCM 音频 topic；由画布连线提供"},
    }

    # 音频流里没有「说完了」这个标记，只能按静默判断。
    QUIET_SECONDS = 0.6

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._sub = None
        self._sub_node = None
        self._input_topic = ""
        self._speaking = False
        self._now = 0.0
        self._last_chunk = 0.0
        self._started_at = 0.0
        self._turns = 0
        self._action_id = ""
        world.add_step_listener(self._on_step)

    # ---- lifecycle -----------------------------------------------------

    def dispatch(self, action: str, args: dict) -> dict:
        """在生命周期动词被基类吞掉之前，把 `input_topic` 截下来。

        画布把上游 `tts` 的 `topic_out` 当作 `start` 的 `input_topic` 传进来，而
        `Card.dispatch` 调的是无参的 `self.start()` —— 不在这儿接，参数就丢了。
        """
        if action == "start":
            picked = str((args or {}).get("input_topic") or "")
            if picked:
                self._input_topic = picked
        return super().dispatch(action, args)

    def start(self) -> None:
        super().start()
        self._open_subscription()

    def stop(self) -> None:
        self._close_subscription()
        self._end_utterance()
        super().stop()

    def info(self) -> dict:
        """**把真正绑上的那条 topic 报出来。**

        agent-core 用 `info().topic_in[].topic` 判断一张卡到底接上了什么
        （`api/config.py::_bound_inputs`）。不报的话，一张聋着的卡和一张正常工作的卡
        在它眼里一模一样 —— 而这正是第一版的下场。
        """
        base = super().info()
        base["topic_in"] = ([{"topic": self._input_topic, "format": "audio/pcm-16k"}]
                            if self._input_topic else list(self.TOPIC_IN))
        base["speaking"] = self._speaking
        return base

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle",
                "input_topic": self._input_topic,
                "speaking": self._speaking, "turns": self._turns}

    # ---- the stream ----------------------------------------------------

    def _open_subscription(self) -> None:
        """没有 ROS 时安静地不订阅 —— pytest 与离线重放都走这条路。"""
        if self._ros2 is None or not self._input_topic or self._sub is not None:
            return
        try:
            from rclpy.node import Node
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import UInt8MultiArray

            self._sub_node = Node(f"{self.namespace}_speaker", context=self._ros2.ctx_core)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
            self._sub = self._sub_node.create_subscription(
                UInt8MultiArray, self._input_topic, self._on_audio, qos)
            self._ros2.executor_core.add_node(self._sub_node)
        except Exception as exc:                      # noqa: BLE001
            print(f"[sim-speaker] 订阅 {self._input_topic} 失败：{exc}", flush=True)

    def _close_subscription(self) -> None:
        node, self._sub_node, self._sub = self._sub_node, None, None
        if node is None:
            return
        try:
            self._ros2.executor_core.remove_node(node)
            # `destroy_node`，不只是 `remove_node` —— 否则发布者和 ROS 节点名都会泄漏，
            # 下一次订阅会撞上「name already registered」。
            node.destroy_node()
        except Exception:
            pass

    def _on_audio(self, _msg) -> None:
        # 时刻取自世界的事件记录，而不是墙钟：加速倍率下两者不是一回事，而这条事件
        # 要和同一条事实流里的 `arrive` 相减。
        self._last_chunk = self._now
        if not self._speaking:
            self._speaking = True
            self._started_at = self._now
            self._turns += 1
            self._action_id = f"spk-{self._turns}"
            self.world.log("speak_start", action_id=self._action_id,
                           source=self._input_topic or "stream")

    def _on_step(self, t: float, _dt: float) -> None:
        self._now = t
        if self._speaking and t - self._last_chunk > self.QUIET_SECONDS:
            self._end_utterance(t)

    def _end_utterance(self, now: float | None = None) -> None:
        if not self._speaking:
            return
        self._speaking = False
        ended = self._now if now is None else now
        self.world.log("speak_end", action_id=self._action_id, status="completed",
                       duration=round(max(0.0, ended - self._started_at), 2))


class TtsCard(_SpeechCard):
    NAME = "tts"
    KIND = "actuator"
    DESCRIPTION = "虚拟语音合成 — 播报讲解词，可被打断；播报时长按字数估算"
    TOPIC = ""
    COMPLETION = {"actions": ["speak"], "timeout": 180}
    HOOKS = {
        "on_interrupt_speak": {"action": "interrupt"},
        "on_interrupt_all": {"action": "interrupt"},
        "on_notify": {"action": "speak"},
    }
    ACTIONS = {
        "speak": (["text"], "播报一段文字；完成或被打断时通过 ACP 回调上报"),
        "interrupt": ([], "立即停止当前播报并清空排队内容"),
        "read": ([], "读取当前播报状态"),
    }
    PROPERTIES = {"text": {"type": "string", "description": "要播报的文字"}}

    def do_speak(self, text: str = "", **_):
        if not str(text).strip():
            return {"error": "speak requires non-empty text"}
        return self._say(str(text))

    def do_interrupt(self, **_):
        stopped = self.world.interrupt_speech("tts.interrupt")
        return {"state": "idle", "interrupted": len(stopped)}

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle",
                "speech": self.world.snapshot()["speech"],
                "queued": self.world.snapshot()["speech_queued"]}
