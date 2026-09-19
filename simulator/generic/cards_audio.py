"""Speech cards: `tts` and `speaker`.

Both drive the same mouth — one `SpeechQueue` in the world — which is why both
declare `x-resource: ["mouth"]`. Two cards that can talk at once while claiming
separate resources is how a robot ends up saying two things over each other.

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
    """Shared plumbing: own the utterances this card created, and post ACP for
    exactly those.

    Ownership matters because the world emits every terminal to every listener.
    Without the filter, `speaker` would report `tts`'s completions under its own
    tool name, and agent-core's transcript would attribute speech to the wrong
    card — silently, since ACP has no idea which card it should have come from.
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


class SpeakerCard(_SpeechCard):
    """Audio playback. Distinct from `tts` because a real robot has both, and
    they contend for the same mouth — which the shared `SpeechQueue` models."""

    NAME = "speaker"
    KIND = "actuator"
    DESCRIPTION = "虚拟扬声器 — 播放文字或音频文件，与 tts 共用同一个「嘴」"
    TOPIC = ""
    COMPLETION = {"actions": ["play_text", "play_file"], "timeout": 60}
    ACTIONS = {
        "play_text": (["text"], "朗读一段文字"),
        "play_file": (["path", "seconds"], "播放一个音频文件，seconds 为其时长"),
        "stop": ([], "停止播放"),
        "read": ([], "读取当前播放状态"),
    }
    PROPERTIES = {
        "text": {"type": "string"},
        "path": {"type": "string", "description": "音频文件路径"},
        "seconds": {"type": "number", "description": "音频时长，秒"},
    }

    def do_play_text(self, text: str = "", **_):
        if not str(text).strip():
            return {"error": "play_text requires non-empty text"}
        return self._say(str(text))

    def do_play_file(self, path: str = "", seconds: float = 3.0, **_):
        if not str(path).strip():
            return {"error": "play_file requires a path"}
        # The queue measures duration from text length, so a file is represented
        # by a placeholder of the right length. Nothing downstream reads the
        # characters; what matters is that it occupies the mouth for `seconds`.
        chars = max(1, int(float(seconds) * self.world._chars_per_sec))  # noqa: SLF001
        result = self._say("♪" * chars)
        result["path"] = path
        result["text"] = f"<file:{path}>"
        return result

    def do_stop(self, **_):
        stopped = self.world.interrupt_speech("speaker.stop")
        return {"state": "idle", "stopped": len(stopped)}

    def do_read(self, **_):
        snapshot = self.world.snapshot()
        return {"state": "running" if self._running else "idle",
                "speech": snapshot["speech"], "queued": snapshot["speech_queued"]}
