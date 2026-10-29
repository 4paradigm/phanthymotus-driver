"""Speech: the `tts` card.

`tts` 是这个 bundle 里唯一的嘴：调用驱动的合成，播报时长按字数估算，完成或被打断时
通过 ACP 回调上报。两台 Orin 都没有真实扬声器，所以一张虚拟的 `tts` 是在那上面验证
播报时序的唯一办法 —— 而那正是展区导览用例存在的理由：某一站的讲解必须在**到达之后**
开始、在**离开之前**结束。

**那个 gap 已经补上了，但不是在这里补的。** 补法是 agent-core 自己把派发出去的
异步动作记成事实（`agent-core/src/benchmark_facts.py`），再并进仿真器的事实流。
它按驱动申报的 `x-resource` 通道分类，perception 的 `tts` 申报了 `mouth`，所以
播报的起止两端在真机和仿真上都记得到。

**一张订阅 PCM 的 speaker 卡曾经也做过这件事，已经删掉。** 它要求镜像里有
`audio_msgs`（仓库里所有音频发布者用的类型），也就要 vendor 一份 .msg 再 colcon
build 一次；而它换来的只是「音频真的响了」这一点点额外保真度，agent-core 那条路
在真机上还照样能跑，它不能 —— 真机画布上没有仿真器。它还带着一个很难发现的失败
模式：ROS 的订阅按类型匹配，类型写错就根本不配对且**什么都不报**，节点在、topic
列得出来、回调永不触发。第一版正是订成了 `std_msgs/UInt8MultiArray`，于是 Orin6 上
「世界真的做了什么」那一列始终空着，而一切看起来都正常。

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
