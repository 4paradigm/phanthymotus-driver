#!/usr/bin/env python3
"""Morse code card for the Noetix Bumi bundle.

The card takes Morse code as text, decodes it to English and speaks the result
on the robot speaker.  Everything happens locally: ``espeak-ng`` synthesises the
English text and ``ffmpeg`` converts it to the 16 kHz stereo PCM that the
vendor ``MediaController`` playback route expects.  No cloud TTS, no network.

Input convention: dots/dashes (``.``/``-``, full-width variants accepted),
letters separated by one space, words separated by ``/``, ``|`` or two or more
spaces.  Unrecognised codes are reported instead of being spoken, so a typo
never turns into confident robot speech.

The module has no ROS import at module level: ``tests/test_morse_card.py``
exercises the decoder and the card contract without rclpy or the vendor SDK.
"""

from __future__ import annotations

import re
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

# ── Morse table (ITU-R M.1677-1) ─────────────────────────────────────────────

_ALPHABET = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".",
    "F": "..-.", "G": "--.", "H": "....", "I": "..", "J": ".---",
    "K": "-.-", "L": ".-..", "M": "--", "N": "-.", "O": "---",
    "P": ".--.", "Q": "--.-", "R": ".-.", "S": "...", "T": "-",
    "U": "..-", "V": "...-", "W": ".--", "X": "-..-", "Y": "-.--",
    "Z": "--..",
    "0": "-----", "1": ".----", "2": "..---", "3": "...--", "4": "....-",
    "5": ".....", "6": "-....", "7": "--...", "8": "---..", "9": "----.",
    ".": ".-.-.-", ",": "--..--", "?": "..--..", "'": ".----.",
    "!": "-.-.--", "/": "-..-.", "(": "-.--.", ")": "-.--.-",
    "&": ".-...", ":": "---...", ";": "-.-.-.", "=": "-...-",
    "+": ".-.-.", "-": "-....-", "_": "..--.-", '"': ".-..-.",
    "$": "...-..-", "@": ".--.-.",
}

MORSE_TO_CHAR = {code: character for character, code in _ALPHABET.items()}

# Accept the full-width/typographic characters a Chinese keyboard or an LLM
# produces for dots, dashes and separators.
_TRANSLATION = str.maketrans({
    "．": ".", "。": ".", "·": ".", "•": ".", "⸳": ".",
    "－": "-", "—": "-", "–": "-", "−": "-", "ー": "-",
    "／": "/", "｜": "|", "\u00a0": " ", "\t": " ", "\r": " ", "\n": " ",
})

_WORD_BREAK = re.compile(r"\s*[/|]\s*|\s{2,}")
_TOKEN = re.compile(r"[.\-]+")

MAX_MORSE_CHARS = 1200
MAX_TEXT_CHARS = 200

_SAMPLE_RATE = 16000
_CHANNELS = 2
_FRAME_MS = 100
_FRAME_SECONDS = _FRAME_MS / 1000
_FRAME_BYTES = _SAMPLE_RATE * _CHANNELS * 2 * _FRAME_MS // 1000
_CONFIG_INTERVAL_S = 0.5

_VOICE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_+\-]{0,31}")


class MorseInputError(ValueError):
    """The supplied Morse text cannot be decoded at all."""


def decode_morse(value) -> dict:
    """Decode Morse code to English.

    Returns ``text`` (unknown codes shown as ``?``), the decoded ``words``,
    the number of valid ``letters``, a normalised representation and one entry
    per unrecognised token (``unknown``).  Raises :class:`MorseInputError`
    when the input is not usable at all.
    """
    if not isinstance(value, str):
        raise MorseInputError("morse 必须是字符串")
    if len(value) > MAX_MORSE_CHARS:
        raise MorseInputError(
            f"morse 过长：{len(value)} 个字符，最多 {MAX_MORSE_CHARS} 个")
    source = value.translate(_TRANSLATION)
    if not source.strip():
        raise MorseInputError("morse 为空")

    words, parts, unknown = [], [], []
    letters = 0
    for chunk in _WORD_BREAK.split(source):
        tokens = chunk.split()
        if not tokens:
            continue
        word_number = len(words) + 1
        decoded = []
        for letter_number, token in enumerate(tokens, start=1):
            character = MORSE_TO_CHAR.get(token)
            if character is None:
                decoded.append("?")
                unknown.append({
                    "word": word_number,
                    "letter": letter_number,
                    "code": token,
                    "reason": ("unknown_code" if _TOKEN.fullmatch(token)
                               else "invalid_symbol"),
                })
            else:
                decoded.append(character)
                letters += 1
        words.append("".join(decoded))
        parts.append(" ".join(tokens))

    text = " ".join(words)
    if not text:
        raise MorseInputError("morse 为空")
    if len(text) > MAX_TEXT_CHARS:
        raise MorseInputError(
            f"译文过长：{len(text)} 个字符，最多 {MAX_TEXT_CHARS} 个")
    return {
        "text": text,
        "words": words,
        "letters": letters,
        "unknown": unknown,
        "normalized": " / ".join(parts),
    }


def _bounded_int(value, default: int, low: int, high: int) -> int:
    if isinstance(value, bool):
        return default
    try:
        number = int(value)
    except (TypeError, ValueError):
        return default
    return min(high, max(low, number))


def _voice_name(value) -> str:
    return value if isinstance(value, str) and _VOICE.fullmatch(value) else "en-us"


def _find_engine():
    for name in ("espeak-ng", "espeak"):
        path = shutil.which(name)
        if path:
            return path
    return None


class MorsePlugin:
    """MCP card: ``morse`` — decode Morse code and speak it in English."""

    PREFIX = "morse"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._media_ctrl = media_ctrl
        self._voice = _voice_name(plugin_config.get("voice"))
        self._speed_wpm = _bounded_int(plugin_config.get("speed_wpm"), 150, 80, 300)
        self._engine = _find_engine()
        self._ffmpeg = shutil.which("ffmpeg")
        self._active = None
        self._thread = None
        self._last = None
        self._state_lock = threading.RLock()
        # Serialises start/stop so two speak calls cannot both pass the
        # cancellation point and end up feeding the same playback route.
        self._control_lock = threading.Lock()
        self._route_lock = threading.Lock()
        self._last_config_change = 0.0

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "morse",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "把摩斯电码翻译成英文并用机器人扬声器朗读。"
                "输入点划（. 和 -），字母之间用空格，单词之间用 / 或两个以上空格；"
                "支持 A-Z、0-9 和常用标点。"
                "示例：.... . .-.. .-.. --- / .-- --- .-. .-.. -.. 表示 HELLO WORLD。"
                "无法识别的电码会用 ? 标出并且不会朗读，避免读错内容。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["speak", "decode", "stop", "info", "start"],
                        "description": ("speak=翻译并朗读；decode=只翻译不朗读；"
                                        "stop=停止朗读；info=查看状态；"
                                        "start=检查语音组件是否就绪。"),
                    },
                    "morse": {
                        "type": "string",
                        "maxLength": MAX_MORSE_CHARS,
                        "description": ("摩斯电码文本。字母之间用空格，单词之间用 / "
                                        "或两个以上空格，例如：.... .. / -... ..- -- .."),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "speak": {"params": ["morse"],
                              "description": "解码摩斯电码并用英文朗读；新的请求会打断正在播放的语音。"},
                    "decode": {"params": ["morse"],
                               "description": "只把摩斯电码翻译成英文文本，不播放语音。"},
                    "stop": {"params": [],
                             "description": "停止当前正在播放的摩斯密码语音。"},
                    "info": {"params": [],
                             "description": "返回语音组件可用性、当前播放状态和最近一次结果。"},
                    "start": {"params": [],
                              "description": "检查 espeak-ng/ffmpeg 是否可用，不朗读任何内容。"},
                },
            },
            "topic_out": [],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        if self._tts_ready():
            return {"state": "ready"}
        message = self._unavailable_message()
        print(f"[morse] {message}", flush=True)
        return {"state": "unavailable", "message": message}

    def stop(self):
        with self._control_lock:
            stopped = self._cancel_active()
        if stopped and self._media_ctrl is not None:
            try:
                self._media_ctrl.pause_audio_playback()
            except Exception as exc:  # pragma: no cover - vendor runtime only
                print(f"[morse] pause_audio_playback failed: {exc}", flush=True)
        return {"state": "idle", "stopped": stopped}

    def dispatch(self, action: str, args: dict):
        args = args or {}
        if action == "morse":  # tool called without an explicit action
            action = "speak"
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action == "decode":
            return self._decode(args)
        if action == "speak":
            return self._speak(args)
        return None

    # ── Actions ──────────────────────────────────────────────────────────────

    def _decode(self, args: dict) -> dict:
        try:
            decoded = decode_morse(args.get("morse"))
        except MorseInputError as exc:
            return {"state": "error", "code": "INVALID_MORSE",
                    "message": str(exc)}
        return self._decode_result(decoded)

    @staticmethod
    def _decode_result(decoded: dict) -> dict:
        result = {
            "state": "invalid_morse" if decoded["unknown"] else "decoded",
            "text": decoded["text"],
            "letters": decoded["letters"],
            "normalized": decoded["normalized"],
        }
        if decoded["unknown"]:
            result["unknown"] = decoded["unknown"]
            result["message"] = "部分电码无法识别，已用 ? 标出；不会朗读未识别的内容。"
        return result

    def _speak(self, args: dict) -> dict:
        result = self._decode(args)
        if result["state"] != "decoded":
            if result["state"] == "invalid_morse":
                result["message"] = ("电码无法识别，未播放语音；"
                                     "请按“字母空格、单词斜杠”的格式修正后重试。")
            return result
        if not self._tts_ready():
            return {"state": "unavailable", "message": self._unavailable_message()}
        if self._media_ctrl is None:
            return {"state": "unavailable",
                    "message": "机器人 MediaController 未初始化，无法播放语音"}

        record = self._begin(result["text"], result["normalized"])
        return {
            "state": "speaking",
            "text": result["text"],
            "letters": result["letters"],
            "action_id": record["action_id"],
            "voice": self._voice,
            "speed_wpm": self._speed_wpm,
            "message": "正在用英文语音朗读。",
        }

    # ── Speech worker ────────────────────────────────────────────────────────

    def _begin(self, text: str, morse: str) -> dict:
        with self._control_lock:
            self._cancel_active()
            record = {
                "action_id": f"bumi_morse_speak_{uuid.uuid4().hex}",
                "state": "synthesizing",
                "text": text,
                "morse": morse,
                "started_at": time.time(),
                "stop_event": threading.Event(),
            }
            thread = threading.Thread(target=self._run_speech, args=(record,),
                                      daemon=True, name="bumi_morse_speak")
            with self._state_lock:
                self._active = record
                self._thread = thread
            thread.start()
        return record

    def _run_speech(self, record: dict) -> None:
        try:
            pcm = self._synthesize(record["text"])
        except Exception as exc:
            print(f"[morse] TTS failed: {exc}", flush=True)
            self._finish(record, "error", {"code": "TTS_FAILED",
                                          "message": str(exc)})
            return

        record["state"] = "speaking"
        try:
            if record["stop_event"].is_set():
                self._finish(record, "stopped", {})
                return
            self._open_route()
            stopped = self._stream(record, pcm)
        except Exception as exc:
            print(f"[morse] playback failed: {exc}", flush=True)
            self._finish(record, "error", {"code": "PLAYBACK_FAILED",
                                          "message": str(exc)})
            return
        self._finish(record, "stopped" if stopped else "completed", {})

    def _finish(self, record: dict, state: str, details: dict) -> None:
        with self._state_lock:
            if self._active is record:
                self._active = None
                self._thread = None
            if self._active is None:
                # A newer utterance owns the card: the late tail of an
                # interrupted one must not overwrite its status.
                self._last = {
                    "action_id": record["action_id"],
                    "text": record["text"],
                    "state": state,
                    "duration_s": round(time.time() - record["started_at"], 2),
                    "finished_at": time.time(),
                    **details,
                }

    def _cancel_active(self) -> bool:
        """Signal the active utterance and wait for its worker to exit."""
        with self._state_lock:
            record = self._active
            thread = self._thread
            if record is not None:
                record["stop_event"].set()
        if (thread is not None and thread.is_alive()
                and thread is not threading.current_thread()):
            thread.join(timeout=3)
        with self._state_lock:
            if self._active is record:
                self._active = None
                self._thread = None
        # The worker clears ``_active`` itself in ``_finish`` before the join
        # returns, so the caller's question is "was anything playing?", not
        # "is the record still installed?".
        return record is not None

    def _synthesize(self, text: str) -> bytes:
        if not self._engine or not self._ffmpeg:
            raise RuntimeError(self._unavailable_message())
        with tempfile.TemporaryDirectory(prefix="bumi_morse_") as directory:
            wav = Path(directory) / "speech.wav"
            raw = Path(directory) / "speech.pcm"
            # The leading space keeps a decoded message that starts with "-"
            # (Morse -....-) from being parsed as an espeak option.
            spoken = subprocess.run(
                [self._engine, "-v", self._voice, "-s", str(self._speed_wpm),
                 "-w", str(wav), " " + text],
                capture_output=True, timeout=30, check=False)
            if (spoken.returncode != 0 or not wav.is_file()
                    or wav.stat().st_size <= 44):
                detail = spoken.stderr.decode("utf-8", "replace").strip()[:300]
                raise RuntimeError(
                    f"espeak-ng 合成失败（退出码 {spoken.returncode}）：{detail}")
            converted = subprocess.run(
                [self._ffmpeg, "-nostdin", "-y", "-loglevel", "error",
                 "-i", str(wav), "-ar", str(_SAMPLE_RATE), "-ac", str(_CHANNELS),
                 "-f", "s16le", str(raw)],
                capture_output=True, timeout=30, check=False)
            if converted.returncode != 0 or not raw.is_file():
                detail = converted.stderr.decode("utf-8", "replace").strip()[:300]
                raise RuntimeError(
                    f"ffmpeg 转码失败（退出码 {converted.returncode}）：{detail}")
            pcm = raw.read_bytes()
        if len(pcm) % 4:
            pcm = pcm[:-(len(pcm) % 4)]
        if not pcm:
            raise RuntimeError("TTS 没有产生音频数据")
        return pcm

    def _open_route(self) -> None:
        media = self._media_ctrl
        if media is None:
            raise RuntimeError("MediaController 未初始化")
        with self._route_lock:
            # Write the route on every utterance instead of caching it: the
            # speaker card disables external playback when it stops, and a
            # cached "already open" flag would then leave this card silent.
            # The vendor SDK asks for at least 500 ms between config writes.
            remaining = _CONFIG_INTERVAL_S - (
                time.monotonic() - self._last_config_change)
            if remaining > 0:
                time.sleep(remaining)
            media.set_external_custom_audio_data_to_playback_enable(True)
            self._last_config_change = time.monotonic()
            media.resume_audio_playback()

    def _stream(self, record: dict, pcm: bytes) -> bool:
        from mediacontrol_py import AudioStream

        media = self._media_ctrl
        next_at = time.monotonic()
        for offset in range(0, len(pcm), _FRAME_BYTES):
            if record["stop_event"].is_set():
                return True
            chunk = pcm[offset:offset + _FRAME_BYTES]
            if len(chunk) % 2:
                chunk = chunk[:-1]
            if not chunk:
                break
            stream = AudioStream()
            stream.channels = _CHANNELS
            stream.sample_rate = _SAMPLE_RATE
            stream.format = 2  # S16_LE
            stream.duration_ms = _FRAME_MS
            stream.timestamp_us = time.time_ns() // 1000
            stream.audio_data = list(
                struct.unpack(f"<{len(chunk) // 2}h", chunk))
            media.publish_external_audio_playback_stream(stream)
            # Feed the audio agent at real time, like the speaker card receives
            # a live topic; publishing the whole file at once would overflow.
            next_at += _FRAME_SECONDS
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        return record["stop_event"].is_set()

    # ── Status ───────────────────────────────────────────────────────────────

    def _tts_ready(self) -> bool:
        return self._engine is not None and self._ffmpeg is not None

    def _unavailable_message(self) -> str:
        missing = [name for name, path in
                   (("espeak-ng", self._engine), ("ffmpeg", self._ffmpeg))
                   if not path]
        return ("缺少语音组件：" + "、".join(missing) +
                "；请重新构建驱动镜像（Dockerfile 已安装 espeak-ng 与 ffmpeg）")

    def _info(self) -> dict:
        with self._state_lock:
            active = None if self._active is None else {
                "action_id": self._active["action_id"],
                "state": self._active["state"],
                "text": self._active["text"],
                "started_at": self._active["started_at"],
            }
            last = dict(self._last) if self._last else None
        ready = self._tts_ready()
        result = {
            "state": "speaking" if active else ("idle" if ready else "unavailable"),
            "tts": {
                "available": ready,
                "engine": Path(self._engine).name if self._engine else None,
                "ffmpeg": bool(self._ffmpeg),
                "voice": self._voice,
                "speed_wpm": self._speed_wpm,
            },
            "active": active,
            "last": last,
        }
        if not ready:
            result["message"] = self._unavailable_message()
        return result
