"""Speech: the `tts` card.

There was a `speaker` card here too. It has been removed, because it was two
different things stitched together: the **name** of a stream sink and the
**behaviour** of a player, with no `topic_in` at all, so nothing could be wired
into it on the canvas.

Both shapes exist for real, and they are not the same card. Every real
`speaker` in this repo — `unitree/g1`, `go2`, `r1`, `engineai/t800`,
`noetix/bumi`, `robotera/q5_bundle` — declares
`topic_in: [{"format": "audio/pcm-16k"}]` and plays whatever arrives on the
stream. `x-humanoid/tianyi2.0`'s `voice_play` is the other shape: call-driven
`play_file` / `play_url` / `play_text`, and no `topic_in` because it consumes no
stream. Copying the second while naming it after the first produced a card that
answered to nobody.

`tts` covers everything the exhibition tour needs — ACP completion, interrupt,
and the announcement-ordering assertions — so the bundle has one mouth and one
card for it. A stream-consuming `speaker` is worth adding when something needs
the `topic_in` half of the canvas contract, which nothing here does yet.

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
