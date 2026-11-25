"""Bumi live-stream card: push the camera and microphone it is wired to over RTMP.

On the canvas a JPEG camera and a PCM microphone are connected into this card.
`start` subscribes to both, `go_live` runs one ffmpeg that encodes them to FLV
and publishes to the RTMP address saved in the card's settings, and `end_live`
stops it. Agent Core re-sends the saved settings (`config`) before `start` and
`go_live`, so nothing is persisted here.

One pacing thread feeds ffmpeg at a fixed frame rate: every tick it writes the
latest JPEG (repeating the last one when the camera is late) and exactly one
tick's worth of PCM (padding with silence when the mic is late), so audio and
video stay on the same wall clock however irregular the inputs are.
"""

from __future__ import annotations

import os
import select
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlsplit

CARD = "live_stream"
_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
_AUDIO_TYPE = "audio_msgs/msg/AudioChunk"
_AUDIO_FORMATS = frozenset(("audio/pcm-16k", "pcm_16k_16bit_mono"))
_SAMPLE_RATE = 16000
_MAX_AUDIO_BACKLOG = _SAMPLE_RATE * 2 // 2      # 0.5 s of PCM16; older audio is dropped
_FIRST_FRAME_TIMEOUT_S = 3.0
_STALL_TIMEOUT_S = 10.0                          # camera silent this long ends the stream
_STARTUP_CHECK_S = 3.0                           # a bad address or key fails within this


def validate_rtmp_url(value) -> str:
    url = str(value or "").strip()
    parts = urlsplit(url)
    if parts.scheme not in ("rtmp", "rtmps") or not parts.hostname or len(parts.path) < 2:
        raise ValueError("推流地址应为 rtmp:// 或 rtmps:// 开头的完整地址（含推流码）")
    return url


class _InputNode:
    """Per-start ROS node holding the two subscriptions; destroyed on stop."""

    def __init__(self, topics: list, on_jpeg, on_pcm):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from audio_msgs.msg import AudioChunk
        from sensor_msgs.msg import CompressedImage

        self.node = Node("bumi_live_stream")
        types = self._topic_types(topics)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.VOLATILE)
        self.image_topic = next((t for t in topics if _IMAGE_TYPE in types.get(t, ())), "")
        self.audio_topic = next((t for t in topics if _AUDIO_TYPE in types.get(t, ())), "")
        if self.image_topic:
            self.node.create_subscription(
                CompressedImage, self.image_topic, lambda m: on_jpeg(bytes(m.data)), qos)
        if self.audio_topic:
            self.node.create_subscription(
                AudioChunk, self.audio_topic, lambda m: on_pcm(bytes(m.data), m.format), qos)

    def _topic_types(self, topics: list) -> dict:
        # The canvas passes topics without their types; ask the ROS graph.
        deadline = time.monotonic() + 3.0
        while True:
            types = dict(self.node.get_topic_names_and_types())
            if all(t in types for t in topics) or time.monotonic() > deadline:
                return types
            time.sleep(0.1)


class LiveStreamPlugin:
    PREFIX = CARD

    def __init__(self, plugin_config: dict, namespace: str, executor, *,
                 input_node=_InputNode, popen=subprocess.Popen):
        self._executor = executor
        self._input_node = input_node
        self._popen = popen
        self._fps = max(1, min(30, int(plugin_config.get("fps", 15))))
        self._bitrate_kbps = max(300, min(4000, int(plugin_config.get("video_bitrate_kbps", 1200))))
        self._lock = threading.RLock()
        self._url = ""
        self._inputs = None
        self._jpeg = None
        self._jpeg_at = 0.0
        self._pcm_lock = threading.Lock()   # shared with the pacing thread, never held long
        self._pcm = bytearray()
        self._pcm_at = 0.0
        self._generation = 0           # invalidates pending starts after stop/restart
        self._stream = None            # {"proc", "thread", "cancel", "log", "started_at", "error"}
        self._last_error = ""

    # ── canvas contract ───────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": CARD,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "把连到本卡片的摄像头画面和麦克风声音推流到直播平台（虎牙、小红书等任何提供 "
                "RTMP 推流地址的平台）。go_live 开播，end_live 停播。"),
            "configSchema": {
                "type": "object",
                "properties": {
                    "rtmp_url": {
                        "type": "string",
                        "title": "推流地址",
                        "description": "平台给出的完整 RTMP 地址，服务器地址后接推流码，如 "
                                       "rtmp://live-push.example.com/live/<推流码>",
                        "x-sensitive": True,
                        "scope": "shared",
                    },
                },
                "required": ["rtmp_url"],
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "info", "config", "go_live", "end_live"]},
                },
                "required": ["action"],
                "x-action-params": {
                    "go_live": {"params": [], "description": "开始直播：把当前画面和声音推到直播平台"},
                    "end_live": {"params": [], "description": "结束直播"},
                    "info": {"params": [], "description": "查看直播状态、时长和输入是否在更新"},
                },
            },
            "topic_in": [
                {"format": "image/jpeg", "message_type": _IMAGE_TYPE},
                {"format": "audio/pcm-16k", "message_type": _AUDIO_TYPE},
            ],
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._unsubscribe()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "config":
            return self._config(args)
        if action == "start":
            return self._subscribe(args)
        if action == "stop":
            return self._unsubscribe()
        if action == "go_live":
            return self._go_live()
        if action == "end_live":
            return self._end_live()
        if action == "info":
            return self._info()
        return None

    # ── actions ───────────────────────────────────────────────────────────

    def _config(self, args: dict) -> dict:
        if "rtmp_url" not in args:
            return {"state": "configured"}
        try:
            url = validate_rtmp_url(args["rtmp_url"]) if args["rtmp_url"] else ""
        except ValueError as exc:
            return {"state": "error", "error": str(exc)}
        with self._lock:
            self._url = url
        # A running stream keeps its address; a new one applies on the next go_live.
        return {"state": "configured"}

    def _reset_locked(self) -> int:
        self._generation += 1
        self._end_live_locked()
        self._destroy_inputs_locked()
        return self._generation

    def _subscribe(self, args: dict) -> dict:
        topics = list(args.get("input_topics") or [])
        if not topics and args.get("input_topic"):
            topics = [args["input_topic"]]
        with self._lock:
            generation = self._reset_locked()
        # Built outside the lock: resolving topic types may take a few seconds,
        # and a concurrent stop must not queue behind it.
        try:
            inputs = self._input_node(topics, self._on_jpeg, self._on_pcm)
        except Exception as exc:
            return {"state": "error", "error": f"订阅输入失败：{exc}"}
        with self._lock:
            if generation != self._generation:
                inputs.node.destroy_node()
                return {"state": "superseded", "message": "启动已被后续 start/stop 取消"}
            self._executor.add_node(inputs.node)
            self._inputs = inputs
        missing = [name for name, topic in (("摄像头", inputs.image_topic),
                                            ("麦克风", inputs.audio_topic)) if not topic]
        result = {"state": "ready", "topic_in": self._topic_in()}
        if missing:
            result["warning"] = f"没有连接{'和'.join(missing)}，请在画布上把它连到直播卡片"
        return result

    def _unsubscribe(self) -> dict:
        with self._lock:
            self._reset_locked()
        return {"state": "idle"}

    def _go_live(self) -> dict:
        with self._lock:
            if self._stream and self._stream["proc"].poll() is None:
                return self._info()
            if not self._url:
                return {"state": "error", "error": "请先在卡片设置里填写推流地址"}
            if not (self._inputs and self._inputs.image_topic):
                return {"state": "error", "error": "没有连接摄像头：请在画布上把摄像头连到直播卡片并启动"}
        deadline = time.monotonic() + _FIRST_FRAME_TIMEOUT_S
        while time.monotonic() - self._jpeg_at > 2.0:
            if time.monotonic() > deadline:
                return {"state": "error", "error": "摄像头没有画面，请检查摄像头卡片"}
            time.sleep(0.05)
        with self._lock:
            if self._stream and self._stream["proc"].poll() is None:
                return self._info()
            self._last_error = ""
            stream = self._launch_locked(self._url)
            if "error" in stream:
                return {"state": "error", "error": stream["error"]}
            self._stream = stream
        # A wrong address or rejected key makes ffmpeg exit within seconds;
        # report that here instead of answering "live" for a dead stream.
        try:
            stream["proc"].wait(timeout=_STARTUP_CHECK_S)
        except subprocess.TimeoutExpired:
            return {"state": "live", "message": "已开播，请在直播平台观众端确认画面和声音"}
        stream["thread"].join(timeout=5)
        return {"state": "error", "error": self._last_error or "推流进程意外退出"}

    def _end_live(self) -> dict:
        with self._lock:
            self._end_live_locked()
        return {"state": "idle"}

    def _info(self) -> dict:
        now = time.monotonic()
        with self._lock:
            stream = self._stream
            live = bool(stream and stream["proc"].poll() is None)
            return {
                "state": "live" if live else ("error" if self._last_error else "idle"),
                "configured": bool(self._url),
                "duration_s": round(now - stream["started_at"], 1) if live else 0,
                "video_age_s": round(now - self._jpeg_at, 1) if self._jpeg_at else None,
                "audio_age_s": round(now - self._pcm_at, 1) if self._pcm_at else None,
                "error": self._last_error or None,
                "topic_in": self._topic_in(),
            }

    # ── inputs ────────────────────────────────────────────────────────────

    def _on_jpeg(self, data: bytes) -> None:
        if data:
            self._jpeg, self._jpeg_at = data, time.monotonic()

    def _on_pcm(self, data: bytes, fmt: str) -> None:
        if fmt not in _AUDIO_FORMATS or not data or len(data) % 2:
            return
        with self._pcm_lock:
            self._pcm += data
            if len(self._pcm) > _MAX_AUDIO_BACKLOG:
                del self._pcm[:len(self._pcm) - _MAX_AUDIO_BACKLOG]
            self._pcm_at = time.monotonic()

    def _take_pcm(self, nbytes: int) -> bytes:
        with self._pcm_lock:
            chunk = bytes(self._pcm[:nbytes])
            del self._pcm[:nbytes]
        return chunk + bytes(nbytes - len(chunk))

    def _topic_in(self) -> list:
        inputs = self._inputs
        return [{"topic": topic, "format": fmt, "message_type": kind}
                for topic, fmt, kind in (
                    (inputs.image_topic if inputs else "", "image/jpeg", _IMAGE_TYPE),
                    (inputs.audio_topic if inputs else "", "audio/pcm-16k", _AUDIO_TYPE))
                if topic]

    def _destroy_inputs_locked(self) -> None:
        inputs, self._inputs = self._inputs, None
        if inputs is not None:
            self._executor.remove_node(inputs.node)
            inputs.node.destroy_node()
        self._jpeg, self._jpeg_at = None, 0.0
        with self._pcm_lock:
            self._pcm.clear()
            self._pcm_at = 0.0

    # ── ffmpeg ────────────────────────────────────────────────────────────

    def _command(self, url: str, audio_fd: int) -> list:
        return [
            "ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error",
            # Probe neither input: by default ffmpeg reads seconds of each pipe
            # in turn before opening the output, and the other pipe fills up
            # and blocks the pacing thread meanwhile. Both formats are given.
            "-probesize", "32", "-analyzeduration", "0",
            "-thread_queue_size", "64", "-f", "mjpeg", "-framerate", str(self._fps), "-i", "pipe:0",
            "-probesize", "32", "-analyzeduration", "0",
            "-thread_queue_size", "64", "-f", "s16le", "-ar", str(_SAMPLE_RATE), "-ac", "1",
            "-i", f"pipe:{audio_fd}",
            "-map", "0:v", "-map", "1:a",
            "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", "-b:v", f"{self._bitrate_kbps}k",
            "-maxrate", f"{self._bitrate_kbps}k", "-bufsize", f"{self._bitrate_kbps * 2}k",
            "-g", str(self._fps * 2),
            "-c:a", "aac", "-ar", "44100", "-b:a", "96k",
            "-f", "flv", url,
        ]

    def _launch_locked(self, url: str) -> dict:
        audio_r, audio_w = os.pipe()
        log = tempfile.TemporaryFile()   # a file, so a full stderr pipe cannot stall ffmpeg
        try:
            proc = self._popen(self._command(url, audio_r), stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=log,
                               pass_fds=(audio_r,), bufsize=0)
        except OSError as exc:
            os.close(audio_w)
            log.close()
            return {"error": f"无法启动 ffmpeg：{exc}"}
        finally:
            os.close(audio_r)
        os.set_blocking(proc.stdin.fileno(), False)
        os.set_blocking(audio_w, False)
        cancel = threading.Event()
        stream = {"proc": proc, "cancel": cancel, "log": log, "url": url,
                  "started_at": time.monotonic()}
        stream["thread"] = threading.Thread(
            target=self._pace, args=(stream, audio_w), daemon=True, name="bumi_live_stream")
        stream["thread"].start()
        return stream

    def _pace(self, stream: dict, audio_fd: int) -> None:
        proc, cancel = stream["proc"], stream["cancel"]
        error = ""
        try:
            tick, samples_sent, start = 0, 0, time.monotonic()
            while not cancel.is_set() and proc.poll() is None:
                if time.monotonic() - self._jpeg_at > _STALL_TIMEOUT_S:
                    error = f"摄像头超过 {int(_STALL_TIMEOUT_S)} 秒没有画面，直播已停止"
                    break
                tick += 1
                samples_due = tick * _SAMPLE_RATE // self._fps - samples_sent
                samples_sent += samples_due
                if not (_write(proc.stdin.fileno(), self._jpeg, cancel, proc)
                        and _write(audio_fd, self._take_pcm(samples_due * 2), cancel, proc)):
                    break
                cancel.wait(max(0.0, start + tick / self._fps - time.monotonic()))
        except (OSError, RuntimeError) as exc:
            error = str(exc)
        finally:
            os.close(audio_fd)
            _terminate(proc)
            stream["log"].seek(0)
            detail = stream["log"].read()[-500:].decode("utf-8", "replace").strip()
            stream["log"].close()
            # ffmpeg names the output in its errors; never echo the stream key.
            detail = detail.replace(stream["url"], "<推流地址>")
            # Not under self._lock: end_live holds it while joining this thread.
            if not cancel.is_set():
                self._last_error = error or ("推流失败：" + (detail or f"ffmpeg 退出码 {proc.returncode}"))

    def _end_live_locked(self) -> None:
        stream, self._stream = self._stream, None
        if stream is None:
            return
        stream["cancel"].set()
        _terminate(stream["proc"])
        stream["thread"].join(timeout=5)
        self._last_error = ""


def _write(fd: int, data: bytes, cancel: threading.Event, proc) -> bool:
    """Write all of `data` to a non-blocking pipe; False if cancelled."""
    pending = memoryview(data)
    deadline = time.monotonic() + 5.0
    while pending:
        if cancel.is_set():
            return False
        if proc.poll() is not None:
            raise RuntimeError("ffmpeg 已退出")
        if time.monotonic() > deadline:
            raise RuntimeError("ffmpeg 5 秒没有读取输入，直播已停止")
        if not select.select([], [fd], [], 0.1)[1]:
            continue
        try:
            pending = pending[os.write(fd, pending):]
        except BlockingIOError:
            continue
    return True


def _terminate(proc) -> None:
    try:
        if proc.stdin and not proc.stdin.closed:
            proc.stdin.close()
    except OSError:
        pass
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
