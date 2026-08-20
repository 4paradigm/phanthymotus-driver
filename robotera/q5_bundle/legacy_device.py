"""Verified sensor and audio cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module contains the verified state, battery, audio, and D455 camera cards.
"""

from __future__ import annotations

import io
import audioop
import json
import os
import re
import shlex
import ssl
import struct
import subprocess
import threading
import time
from pathlib import Path
import urllib.error
import urllib.parse
import urllib.request

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import AudioPlay
from xbot_common_interfaces.srv import SetVolume

# main.py resolves all card classes through this module. Keep the direct
# control cards here as explicit exports while their implementation remains
# consolidated in direct_control.py.
from legacy_direct_control import (
    ArmControlPlugin,
)


def _acp_notify(action_id: str, status: str, result: dict, tool: str) -> None:
    """Report completion of a long-running Q5 card action to Agent Core."""
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": tool, "ts": time.time()}).encode()
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    request = urllib.request.Request(f"{url}/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    context = ssl._create_unverified_context() if url.startswith("https://") else None
    try:
        urllib.request.urlopen(request, timeout=5, context=context).read()
    except Exception as exc:
        print(f"[Q5 ACP] callback failed for {action_id}: {exc}", flush=True)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

def _q5_ssh_args(command: str):
    return [
        "sshpass", "-p", "developer", "ssh", "-p", "2222",
        "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
        "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null", "developer@192.168.8.100",
        f"bash -lc {shlex.quote(command)}",
    ]


def _q5_remote_command(command: str, timeout: float = 20.0, stdin=None):
    """Run a noninteractive command in Q5's documented developer container."""
    return subprocess.run(_q5_ssh_args(command), input=stdin, capture_output=True, timeout=timeout)


_Q5_MIC_PIDFILE = "/tmp/phanthymotus-q5-mic-capture.pid"
_Q5_SPEAKER_PIDFILE = "/tmp/phanthymotus-q5-speaker-playback.pid"
_Q5_DEVELOPER_SUDO_PASSWORD = "developer"
_Q5_XOS_CHAT_LOCK = threading.RLock()
_Q5_SPEAKER_PLUGIN = None
_Q5_MIC_PLUGIN = None


def _stop_active_speaker_plugin() -> None:
    """Release both the local speaker pump and its remote ALSA process."""
    plugin = _Q5_SPEAKER_PLUGIN
    if plugin is not None:
        try:
            plugin.stop()
            return
        except Exception as exc:
            print(f"[AudioPlugin] speaker cleanup failed: {exc}", flush=True)
    _stop_remote_speaker_playback()


def _stop_active_mic_plugin() -> None:
    """Release Q5 capture before handing the full-duplex route to XOS/live PCM."""
    plugin = _Q5_MIC_PLUGIN
    if plugin is not None:
        try:
            plugin.stop()
            return
        except Exception as exc:
            print(f"[Q5AudioRoute] mic cleanup failed: {exc}", flush=True)
    _stop_remote_mic_capture()


def _start_active_mic_plugin() -> None:
    """Restore the bundle's normal microphone stream after a route handoff."""
    plugin = _Q5_MIC_PLUGIN
    if plugin is not None:
        try:
            plugin.start()
        except Exception as exc:
            print(f"[Q5AudioRoute] mic restore failed: {exc}", flush=True)


def _q5_xos_json_request(base: str, path: str, method: str = "POST", payload=None):
    body = None
    headers = {"Accept": "application/json", "X-Requested-With": "XMLHttpRequest"}
    if payload is not None:
        body = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(base.rstrip("/") + path, data=body,
                                     method=method, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=10.0) as reply:
            return json.loads(reply.read().decode("utf-8"))
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return {"code": 0, "msg": f"XOS chat API unavailable: {exc}"}


def _q5_xos_chat_is_on(base: str):
    response = _q5_xos_json_request(base, "/robot/chat/get_chat_launch_state?lang=zh")
    if response.get("code") != 200:
        return None, response.get("msg", "XOS chat state unavailable")
    return str(response.get("data", "")).upper() == "ON", None


def _q5_xos_chat_wait(base: str, wanted_on: bool, timeout: float = 8.0):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        state, error = _q5_xos_chat_is_on(base)
        if error:
            last_error = error
        elif state is wanted_on:
            return True, None
        time.sleep(0.2)
    return False, last_error or ("XOS chat did not reach " + ("ON" if wanted_on else "OFF"))


def _q5_xos_chat_set(base: str, path: str):
    response = _q5_xos_json_request(base, path, payload={})
    if response.get("code") != 200:
        return False, response.get("msg", "XOS chat request failed")
    return True, None


def _q5_root_command(command: str) -> str:
    """Run one noninteractive root command in Q5's developer container.

    XOS owns its replay directory as root. Audio bytes are always staged in a
    developer-writable temporary file before this helper is called, so the
    sudo password stream can never be mistaken for audio payload.
    """
    return (
        f"printf '%s\\n' {shlex.quote(_Q5_DEVELOPER_SUDO_PASSWORD)} | "
        f"sudo -S -p '' bash -c {shlex.quote(command)}"
    )


def _stop_remote_mic_capture() -> None:
    """Stop only the tagged microphone process left by this driver."""
    command = f"""pidfile={shlex.quote(_Q5_MIC_PIDFILE)}
if test -r \"$pidfile\"; then
  pid=$(cat \"$pidfile\" 2>/dev/null || true)
  if test -n \"$pid\" && test -r \"/proc/$pid/cmdline\" && grep -aq q5_mic_capture \"/proc/$pid/cmdline\"; then
    kill \"$pid\" 2>/dev/null || true
  fi
  rm -f \"$pidfile\"
fi"""
    try:
        _q5_remote_command(command, timeout=5.0)
    except Exception:
        pass


def _q5_mic_capture_shell(command: str) -> str:
    """Tag the remote PCM process so restart cleanup cannot leave it owning ALSA."""
    return f"""pidfile={shlex.quote(_Q5_MIC_PIDFILE)}
echo $$ > \"$pidfile\"
exec -a q5_mic_capture python3 -u -c {shlex.quote(command)}"""


def _stop_remote_speaker_playback() -> None:
    """Stop only a tagged direct-ALSA speaker left by this driver."""
    command = f"""pidfile={shlex.quote(_Q5_SPEAKER_PIDFILE)}
if test -r \"$pidfile\"; then
  pid=$(cat \"$pidfile\" 2>/dev/null || true)
  if test -n \"$pid\" && test -r \"/proc/$pid/cmdline\" && grep -aq q5_speaker_playback \"/proc/$pid/cmdline\"; then
    kill \"$pid\" 2>/dev/null || true
  fi
  rm -f \"$pidfile\"
fi"""
    try:
        _q5_remote_command(command, timeout=5.0)
    except Exception:
        pass


def _q5_speaker_playback_shell(command: str) -> str:
    """Tag the remote process so restarts cannot leave ALSA playback busy."""
    return f"""pidfile={shlex.quote(_Q5_SPEAKER_PIDFILE)}
echo $$ > \"$pidfile\"
exec -a q5_speaker_playback python3 -u -c {shlex.quote(command)}"""


def _q5_remote_playback_holders() -> str:
    """Return processes with an open Q5 playback PCM device for diagnostics."""
    probe = r"""from pathlib import Path
target = '/dev/snd/pcmC2D0p'
holders = []
for process in Path('/proc').glob('[0-9]*'):
    try:
        for fd in (process / 'fd').iterdir():
            if fd.resolve() == Path(target):
                cmdline = (process / 'cmdline').read_bytes().replace(b'\\0', b' ').decode(errors='replace').strip()
                holders.append('%s:%s' % (process.name, cmdline or '[no cmdline]'))
                break
    except OSError:
        continue
print('; '.join(holders) or 'none')"""
    try:
        result = _q5_remote_command("python3 -c " + shlex.quote(probe), timeout=5.0)
        if not result.returncode:
            return result.stdout.decode(errors="replace").strip() or "none"
    except Exception:
        pass
    return "unavailable"


def _wait_remote_playback_released(timeout: float = 5.0) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout
    holders = "none"
    while time.monotonic() < deadline:
        holders = _q5_remote_playback_holders()
        if holders == "none":
            return True, holders
        time.sleep(0.2)
    return False, holders


def _raise_if_remote_process_exited(process, label: str) -> None:
    """Surface setup failures before a card falsely reports a running stream."""
    time.sleep(0.25)
    returncode = process.poll()
    if returncode is None:
        return
    detail = b""
    if process.stderr:
        try:
            detail = process.stderr.read()
        except Exception:
            pass
    message = detail.decode(errors="replace").strip() or f"remote process exited with code {returncode}"
    raise RuntimeError(f"Q5 {label} stream failed to start: {message}")


def _find_remote_mic_device() -> str:
    """Find the documented Q5 capture endpoint without PortAudio enumeration.

    The manual uses the full-duplex ``USB Audio Device`` (its device index is
    not stable), while POROSVOC is a capture-only fallback on some machines.
    Return ``plughw`` for the full-duplex card so ALSA can convert its native
    USB rate to the bridge's 16 kHz mono contract.
    """
    probe = r"""from pathlib import Path
sound = Path('/sys/class/sound')
candidates = []
for card in sound.glob('card*'):
    try:
        index = int(card.name[4:])
        name = (card / 'id').read_text().strip()
    except (OSError, ValueError):
        continue
    capture = Path('/dev/snd/pcmC%dD0c' % index).exists()
    playback = Path('/dev/snd/pcmC%dD0p' % index).exists()
    if capture:
        candidates.append((index, name, playback))
preferred = [item for item in candidates if item[2]]
preferred.sort(key=lambda item: 'usb audio' not in item[1].lower())
if not preferred:
    preferred = [item for item in candidates if 'porosvoc' in item[1].lower()]
if not preferred:
    preferred = candidates
if preferred:
    index, name, playback = preferred[0]
    print(('plughw:' if playback else 'hw:') + '%d,0' % index)
else:
    print('')"""
    result = _q5_remote_command("python3 -c " + shlex.quote(probe), timeout=15.0)
    if result.returncode:
        detail = (result.stderr or result.stdout).decode(errors="replace").strip()
        raise RuntimeError(f"unable to enumerate Q5 microphone devices: {detail}")
    try:
        selected = result.stdout.decode(errors="replace").strip()
        if not re.fullmatch(r"(?:hw|plughw):\d+,\d+", selected):
            raise ValueError(selected)
        return selected
    except ValueError as exc:
        detail = result.stdout.decode(errors="replace").strip()
        raise RuntimeError(f"no Q5 ALSA microphone capture device found: {detail}") from exc


def _q5_alsa_mic_command(device: str, sample_rate: int, channels: int) -> str:
    """Return a remote ALSA capture process which writes PCM16 to stdout."""
    return """import ctypes, sys
alsa = ctypes.CDLL('libasound.so.2')
alsa.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
alsa.snd_pcm_set_params.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint]
alsa.snd_pcm_readi.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
alsa.snd_pcm_prepare.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_close.argtypes = [ctypes.c_void_p]
pcm = ctypes.c_void_p()
rc = alsa.snd_pcm_open(ctypes.byref(pcm), b'%s', 1, 0)
if rc < 0: raise RuntimeError('snd_pcm_open(%s) failed: %%d' %% rc)
rc = alsa.snd_pcm_set_params(pcm, 2, 3, %d, %d, 1, 200000)
if rc < 0: raise RuntimeError('snd_pcm_set_params failed: %%d' %% rc)
frames = 1600
buffer = ctypes.create_string_buffer(frames * %d)
try:
  while True:
    count = alsa.snd_pcm_readi(pcm, buffer, frames)
    if count < 0:
      alsa.snd_pcm_prepare(pcm)
      continue
    sys.stdout.buffer.write(buffer.raw[:count * %d])
    sys.stdout.buffer.flush()
finally:
  alsa.snd_pcm_close(pcm)
""" % (device, device, channels, sample_rate, channels * 2, channels * 2)


def _q5_alsa_speaker_command(device: str, output_rate: int, output_channels: int) -> str:
    """Return the remote PCM16 stdin -> Q5 ALSA playback process.

    The Q5's playback card is visible as ``hw:2,0`` but its incomplete ALSA
    configuration means PortAudio does not enumerate it as an output device.
    It accepts S16_LE stereo at 44.1/48 kHz.  Keep the public contract at
    16 kHz mono and convert only at this hardware boundary.
    """
    return """import audioop, ctypes, sys
alsa = ctypes.CDLL('libasound.so.2')
alsa.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
alsa.snd_pcm_set_params.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint]
alsa.snd_pcm_writei.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
alsa.snd_pcm_prepare.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_drain.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_close.argtypes = [ctypes.c_void_p]
pcm = ctypes.c_void_p()
rc = alsa.snd_pcm_open(ctypes.byref(pcm), b'%s', 0, 0)
if rc < 0: raise RuntimeError('snd_pcm_open(%s) failed: %%d' %% rc)
rc = alsa.snd_pcm_set_params(pcm, 2, 3, %d, %d, 1, 200000)
if rc < 0: raise RuntimeError('snd_pcm_set_params failed: %%d' %% rc)
state = None
pending = bytearray()
# Read the raw pipe so a short DDS frame is forwarded immediately. The
# previous BufferedReader.read(3200) waited for a full 100 ms block and made
# speech appear to play only after the utterance ended on bursty sources.
read_bytes = 3200
write_bytes = 640  # 20 ms of 16 kHz mono PCM; enough for sample alignment.
try:
  while True:
    chunk = sys.stdin.buffer.raw.read(read_bytes)
    if not chunk: break
    pending.extend(chunk)
    while len(pending) >= write_bytes:
      raw = bytes(pending[:write_bytes])
      del pending[:write_bytes]
      mono, state = audioop.ratecv(raw, 2, 1, 16000, %d, state)
      stereo = audioop.tostereo(mono, 2, 1, 1)
      frames = len(stereo) // %d
      offset = 0
      while offset < frames:
        portion = stereo[offset * %d:]
        buf = ctypes.create_string_buffer(portion)
        written = alsa.snd_pcm_writei(pcm, buf, frames - offset)
        if written < 0:
          alsa.snd_pcm_prepare(pcm)
          continue
        offset += written
finally:
  alsa.snd_pcm_drain(pcm)
  alsa.snd_pcm_close(pcm)
""" % (device, device, output_channels, output_rate, output_rate, output_channels * 2,
       output_channels * 2)


class MicPlugin:
    """Q5 developer-container microphone as a 16 kHz PCM stream."""

    def __init__(self, plugin_config, namespace, executor, client):
        global _Q5_MIC_PLUGIN
        del executor
        self._client = client
        self._topic = f"/{namespace}/mic/audio"
        configured_device = plugin_config.get("device", "auto")
        configured_text = str(configured_device).lower()
        self._device = (
            None if configured_text == "auto"
            else f"hw:{configured_text},0" if configured_text.isdigit()
            else str(configured_device)
        )
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._xos_http_base = str(plugin_config.get(
            "xos_http_base", "http://192.168.8.100:1888")).rstrip("/")
        self._process = None
        self._thread = None
        self._running = False
        self._wanted = False
        self._retry_thread = None
        self._retry_wakeup = threading.Event()
        self._frames_sent = 0
        self._lock = threading.RLock()
        _Q5_MIC_PLUGIN = self
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 mic only supports the shared 16 kHz mono PCM contract")

    def get_tool(self):
        return {
            "name": "mic", "type": "sensor", "multiInstance": False,
            "description": "Q5 microphone, live PCM 16 kHz/16-bit/mono for ASR.",
            "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        # The bundle start and a canvas sensor-start request may arrive nearly
        # simultaneously. ALSA allows only one capture owner for hw:1,0.
        with self._lock:
            self._wanted = True
            if self._running:
                return
            chat_on, chat_error = _q5_xos_chat_is_on(self._xos_http_base)
            if chat_on:
                print("[MicPlugin] capture deferred while XOS chat owns the audio route", flush=True)
                self._schedule_retry()
                return
            if chat_error:
                print(f"[MicPlugin] cannot query XOS chat state: {chat_error}; attempting capture", flush=True)
            try:
                device = self._device if self._device is not None else _find_remote_mic_device()
                command = _q5_alsa_mic_command(device, self._rate, self._channels)
                _stop_remote_mic_capture()
                self._process = subprocess.Popen(
                    _q5_ssh_args(_q5_mic_capture_shell(command)), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    bufsize=0)
                _raise_if_remote_process_exited(self._process, "microphone")
                self._running = True
                self._frames_sent = 0
                self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_mic_stream")
                self._thread.start()
                print(f"[MicPlugin] capture started from device {device} -> {self._topic}", flush=True)
            except Exception as exc:
                self._stop_capture()
                print(f"[MicPlugin] capture unavailable: {exc}", flush=True)
                self._schedule_retry()

    def _schedule_retry(self):
        if self._retry_thread is not None and self._retry_thread.is_alive():
            return
        self._retry_wakeup.clear()
        self._retry_thread = threading.Thread(
            target=self._retry_after_chat_releases, daemon=True, name="q5_mic_route_retry")
        self._retry_thread.start()

    def _retry_after_chat_releases(self):
        while self._wanted and not self._running:
            self._retry_wakeup.wait(1.0)
            self._retry_wakeup.clear()
            if not self._wanted or self._running:
                return
            chat_on, _ = _q5_xos_chat_is_on(self._xos_http_base)
            if chat_on is False:
                self.start()

    def _stop_capture(self):
        self._running = False
        _stop_remote_mic_capture()
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    def _pump(self):
        # 100 ms frames are the same size emitted by perception TTS.
        while self._running and self._process and self._process.stdout:
            chunk = self._process.stdout.read(3200)
            if not chunk:
                break
            sender = getattr(self._client, "publish_audio", None)
            if callable(sender):
                sender(chunk)
                self._frames_sent += 1
                if self._frames_sent == 1:
                    print(f"[MicPlugin] first 100 ms frame published -> {self._topic}", flush=True)
                elif self._frames_sent % 100 == 0:
                    print(f"[MicPlugin] {self._frames_sent} PCM frames forwarded to bridge", flush=True)
        if self._running:
            print("[MicPlugin] remote capture stream ended", flush=True)

    def stop(self):
        with self._lock:
            self._wanted = False
            self._retry_wakeup.set()
            self._stop_capture()

    def dispatch(self, action, args):
        del args
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "set_volume", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "frames_sent": self._frames_sent}
        return None


class SpeakerPlugin:
    """Play any canvas-connected PCM AudioChunk stream on Q5 ALSA output."""

    def __init__(self, plugin_config, namespace, executor, client):
        global _Q5_SPEAKER_PLUGIN
        del namespace
        self._client = client
        self._topic = ""
        self._device = str(plugin_config.get("device", "hw:2,0"))
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._output_rate = int(plugin_config.get("output_sample_rate_hz", 44100))
        self._output_channels = int(plugin_config.get("output_channels", 2))
        self._volume = max(0, min(100, int(plugin_config.get("volume", 100))))
        # Live PCM from remote_mic/TTS is substantially quieter than XOS's
        # stored-audio route. This is a source calibration, while `volume`
        # remains the user-facing 0-100 control.
        self._input_gain = max(1.0, min(16.0, float(plugin_config.get("input_gain", 6.0))))
        self._xos_http_base = str(plugin_config.get(
            "xos_http_base", "http://192.168.8.100:1888")).rstrip("/")
        self._chat_poll_timeout = max(1.0, float(plugin_config.get("chat_poll_timeout_s", 8.0)))
        self._chat_route_settle = max(0.0, float(plugin_config.get("chat_route_settle_s", 1.5)))
        self._system_volume = None
        self._node = Node("q5_speaker")
        executor.add_node(self._node)
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._process = None
        self._thread = None
        self._running = False
        self._frames_received = 0
        self._frames_written = 0
        _Q5_SPEAKER_PLUGIN = self
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 speaker only supports the shared 16 kHz mono PCM contract")
        if self._output_rate not in (44100, 48000) or self._output_channels != 2:
            raise ValueError("Q5 speaker hardware requires 44.1/48 kHz stereo output")

    def get_tool(self):
        return {
            "name": "speaker", "type": "actuator", "multiInstance": False,
            "description": "Q5 speaker. Connect any audio/pcm-16k output (TTS, microphone, or other PCM source) to play it live. set_volume controls live PCM gain and requests the Q5 system volume.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "set_volume", "get_volume", "stop", "info"]},
                "input_topic": {"type": "string", "description": "PCM 16 kHz AudioChunk topic from the canvas connection"},
                "volume": {"type": "integer", "title": "Speaker 音量", "minimum": 0, "maximum": 100,
                           "default": self._volume},
            }, "required": ["action"], "additionalProperties": False,
            "x-action-params": {
                "start": {"params": ["input_topic"], "description": "连接并开始实时播放 PCM。"},
                "set_volume": {"params": ["volume"], "description": "设置实时 PCM 音量并请求 Q5 系统音量，0 静音，100 最大。"},
                "get_volume": {"params": [], "description": "读取当前 speaker 音量和最近一次 XOS 系统音量设置结果。"},
                "stop": {"params": [], "description": "停止实时播放。"},
                "info": {"params": [], "description": "查看 speaker 状态。"},
            }},
            # Leave the topic unresolved until canvas supplies input_topic.
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self, input_topic=None):
        del input_topic
        # The canvas calls dispatch(start, {input_topic}) when an audio output
        # is connected. Do not subscribe to a hard-coded TTS topic at boot.
        return

    def _start_for_topic(self, requested: str) -> None:
        with _Q5_XOS_CHAT_LOCK:
            if self._running and requested == self._topic:
                return
            self.stop()
            try:
                self._prepare_xos_route()
                self._play_startup_sound()
                self._start_playback(requested)
                print(f"[SpeakerPlugin] playback subscribed <- {self._topic}", flush=True)
            except Exception as exc:
                self.stop()
                print(f"[SpeakerPlugin] playback unavailable: {exc}", flush=True)

    def _play_startup_sound(self) -> None:
        """Play a short PCM self-check before subscribing to the live stream."""
        path = Path(__file__).parent / "resource" / "startup_beep.pcm"
        proc = None
        try:
            pcm = path.read_bytes()
            command = _q5_alsa_speaker_command(self._device, self._output_rate, self._output_channels)
            # Use the exact tagged shell used by the live speaker. The bare
            # helper exited before SSH's stdin pipe was ready on Q5.
            proc = subprocess.Popen(_q5_ssh_args(_q5_speaker_playback_shell(command)), stdin=subprocess.PIPE,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0)
            _raise_if_remote_process_exited(proc, "speaker self-check")
            # The remote ALSA helper accepts the public 16 kHz mono contract
            # and converts it at the hardware boundary.
            block_size = 8000  # 250 ms at 16 kHz mono PCM16
            for offset in range(0, len(pcm), block_size):
                block = pcm[offset:offset + block_size]
                if proc.stdin:
                    proc.stdin.write(block)
                    proc.stdin.flush()
                time.sleep(len(block) / 32000.0)
            if proc.stdin:
                proc.stdin.close()
            proc.wait(timeout=3)
            print(f"[SpeakerPlugin] startup self-check OK ({len(pcm)} bytes)", flush=True)
        except Exception as exc:
            detail = ""
            if proc is not None and proc.stderr:
                try:
                    detail = proc.stderr.read().decode(errors="replace").strip()
                except Exception:
                    pass
            print(f"[SpeakerPlugin] startup self-check failed: {exc}{': ' + detail if detail else ''}", flush=True)

    def _prepare_xos_route(self):
        """Release XOS chat's route before taking ownership with live ALSA."""
        # Stop a queued/stale XOS clip before changing the chat state.  XOS can
        # report chat OFF while its player still owns the PCM handle; stopping
        # first makes the subsequent route transition deterministic.
        try:
            stopped = _q5_remote_command(
                "source /opt/ros/humble/setup.bash; "
                "export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
                "timeout 2 ros2 service call /audio_player/stop_play std_srvs/srv/Trigger '{}'",
                timeout=4.0)
            if stopped.returncode:
                detail = (stopped.stderr or stopped.stdout).decode(errors="replace").strip()
                print(f"[SpeakerPlugin] vendor player stop returned an error: {detail}", flush=True)
        except Exception as exc:
            print(f"[SpeakerPlugin] vendor player stop unavailable: {exc}", flush=True)
        state, error = _q5_xos_chat_is_on(self._xos_http_base)
        if error:
            raise RuntimeError(f"cannot query XOS chat state: {error}")
        if not state:
            released, holders = _wait_remote_playback_released()
            if not released:
                raise RuntimeError(f"XOS playback route is still occupied: {holders}")
            return
        ok, error = _q5_xos_chat_set(
            self._xos_http_base, "/robot/chat/quit_chat?lang=zh")
        if not ok:
            raise RuntimeError(f"cannot close XOS chat for speaker: {error}")
        ok, error = _q5_xos_chat_wait(
            self._xos_http_base, False, self._chat_poll_timeout)
        if not ok:
            raise RuntimeError(f"XOS chat did not release speaker route: {error}")
        # XOS reports chat OFF before its vendor ALSA player has necessarily
        # closed the PCM handle. Give that process a short grace period before
        # opening the direct speaker stream.
        if self._chat_route_settle:
            time.sleep(self._chat_route_settle)
        released, holders = _wait_remote_playback_released()
        if not released:
            raise RuntimeError(f"XOS playback route is still occupied: {holders}")

    def _start_playback(self, requested: str) -> None:
        self._topic = requested
        self._set_system_volume(self._volume)
        _stop_remote_speaker_playback()
        command = _q5_alsa_speaker_command(
            self._device, self._output_rate, self._output_channels)
        last_error = None
        for attempt in range(3):
            self._process = subprocess.Popen(
                _q5_ssh_args(_q5_speaker_playback_shell(command)), stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0)
            try:
                _raise_if_remote_process_exited(self._process, "speaker")
                break
            except RuntimeError as exc:
                last_error = exc
                try:
                    self._process.kill()
                    self._process.wait(timeout=1.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass
                if attempt < 2:
                    time.sleep(0.8)
        else:
            holders = _q5_remote_playback_holders()
            raise RuntimeError(f"{last_error}; playback device holders: {holders}") from last_error
        configure = getattr(self._client, "configure_speaker", None)
        if callable(configure):
            configure(self._topic)
        self._running = True
        self._frames_received = 0
        self._frames_written = 0
        self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_speaker_stream")
        self._thread.start()

    def _pump(self):
        while self._running and self._process and self._process.stdin:
            getter = getattr(self._client, "pop_speaker_chunk", None)
            chunk = getter() if callable(getter) else None
            if chunk is None:
                time.sleep(0.005)
                continue
            self._frames_received += 1
            if self._frames_received == 1:
                print(f"[SpeakerPlugin] first PCM frame received from {self._topic}", flush=True)
            elif self._frames_received % 100 == 0:
                print(f"[SpeakerPlugin] {self._frames_received} PCM frames received from {self._topic}", flush=True)
            try:
                # The input stream is commonly quieter than Q5 stored audio.
                # Keep the user-facing 0-100 control linear, then apply a
                # bounded source-gain calibration. audioop clips PCM safely.
                gain = (self._volume / 100.0) * self._input_gain
                if gain != 1.0:
                    chunk = audioop.mul(chunk, 2, gain)
                self._process.stdin.write(chunk)
                self._process.stdin.flush()
                self._frames_written += 1
                if self._frames_written == 1:
                    print("[SpeakerPlugin] first PCM frame written to ALSA", flush=True)
            except (BrokenPipeError, OSError):
                detail = ""
                if self._process and self._process.stderr:
                    try:
                        detail = self._process.stderr.read().decode(errors="replace").strip()
                    except Exception:
                        pass
                print(f"[SpeakerPlugin] remote playback stream ended: {detail}", flush=True)
                self._running = False
                break

    def _set_system_volume(self, volume: int) -> None:
        """Set XOS's global route volume on the robot's Domain-211 stack."""
        command = (
            "source /opt/ros/humble/setup.bash; "
            "if test -f /q5_ws/install/setup.bash; then source /q5_ws/install/setup.bash; fi; "
            "export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
            # The Q5 interface package is bundled with this image. Prefer its
            # stable type; older images may omit it, so fall back to the type
            # advertised by the live service after filtering CLI noise.
            "service_type='xbot_common_interfaces/srv/SetVolume'; "
            "if ! ros2 interface show \"$service_type\" >/dev/null 2>&1; then "
            "service_type=$(ros2 service type /audio_player/set_volume 2>/dev/null | "
            "sed -n 's/^[[:space:]]*\\([A-Za-z0-9_]*\\/srv\\/[A-Za-z0-9_]*\\)$/\\1/p' | head -n 1); fi; "
            "test -n \"$service_type\"; "
            f"timeout 3 ros2 service call /audio_player/set_volume \"$service_type\" "
            f"'{{volume: {int(volume)}}}'"
        )
        try:
            result = _q5_remote_command(command, timeout=5.0)
            output = (result.stdout or result.stderr).decode(errors="replace").strip()
            success = bool(re.search(r"success:\s*true", output, re.IGNORECASE))
            self._system_volume = {
                "state": "ok" if success else "error",
                "volume": volume,
                "message": output[-500:] if output else "no response",
            }
            if not success:
                print(f"[SpeakerPlugin] XOS volume was not set: {output}", flush=True)
        except Exception as exc:
            self._system_volume = {"state": "error", "volume": volume, "message": str(exc)}
            print(f"[SpeakerPlugin] XOS volume request failed: {exc}", flush=True)

    def stop(self):
        self._running = False
        if self._process is not None:
            try:
                if self._process.stdin:
                    self._process.stdin.close()
                self._process.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                self._process.terminate()
        self._process = None
        _stop_remote_speaker_playback()

    def dispatch(self, action, args):
        if action == "start":
            requested = str(args.get("input_topic") or "")
            if not requested:
                return {"ok": False, "code": "INPUT_TOPIC_REQUIRED",
                        "message": "Connect an audio/pcm-16k output to speaker before starting playback"}
            self._start_for_topic(requested)
        elif action == "set_volume":
            value = args.get("volume", self._volume)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
                return {"ok": False, "code": "INVALID_VOLUME", "message": "volume must be an integer from 0 to 100"}
            self._volume = value
            self._set_system_volume(value)
        elif action == "get_volume":
            return {"state": "running" if self._running else "idle",
                    "volume": self._volume, "input_gain": self._input_gain,
                    "system_volume": self._system_volume}
        elif action == "stop":
            self.stop()
        if action in ("start", "set_volume", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_in": ([{"topic": self._topic, "format": "audio/pcm-16k"}]
                                 if self._topic else [{"format": "audio/pcm-16k"}]),
                    "playback": {"device": self._device, "sample_rate_hz": self._output_rate,
                                 "channels": self._output_channels, "volume": self._volume,
                                 "input_gain": self._input_gain,
                                 "system_volume": self._system_volume},
                    "frames_received": self._frames_received,
                    "frames_written": self._frames_written}
        return None


class _Q5MediaPlugin:
    """Base for read-only Domain-211 media subscriptions.

    The developer container owns the D455 and SLAM services. These cards only
    subscribe to their DDS output and send bounded, already-processed payloads
    to the existing Domain-42 bridge worker.
    """

    def __init__(self, plugin_config, namespace, executor, client):
        self._ns = namespace
        self._client = client
        self._executor = executor
        self._running = False
        self._last_sent = 0.0
        self._max_hz = max(0.1, float(plugin_config.get("max_hz", 10.0)))
        self._subscription = None
        self._node = Node(self._node_name)
        executor.add_node(self._node)

    def _send_media(self, payload):
        sender = getattr(self._client, "publish_media", None)
        if callable(sender):
            sender(payload)

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        del args
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start":
            self.start()
        if action in ("start", "info"):
            result = {
                "state": "running" if self._running else "idle",
                "source_topic": self._source_topic,
                "topic_out": [{"topic": self._topic, "format": self._format}],
            }
            if hasattr(self, "_frames_received"):
                result["diagnostics"] = {
                    "frames_received": self._frames_received,
                    "frames_sent": self._frames_sent,
                }
            return result
        return None


class CameraRgbPlugin(_Q5MediaPlugin):
    """D455 RGB to JPEG, throttled before crossing into Agent Core's DDS domain."""

    _node_name = "q5_camera_rgb"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/color/image_raw"))
        self._topic = f"/{namespace}/camera/rgb"
        self._jpeg_quality = max(20, min(95, int(plugin_config.get("jpeg_quality", 70))))
        self._latest = None
        self._frames_received = 0
        self._frames_sent = 0
        self._lock = threading.Lock()
        self._encoder = None
        self._remote_start = dict(plugin_config.get("remote_start") or {})
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_rgb", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 RGB camera. The developer-container RealSense driver must already be running.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from PIL import Image as PilImage
        from sensor_msgs.msg import Image

        try:
            self._start_remote_realsense_if_configured()
        except Exception as exc:
            # The D455 may already be owned by XOS or take longer than the
            # remote readiness probe.  Do not turn that into a permanently
            # stopped RGB card: subscribe locally and let DDS discovery win.
            print(f"[CameraRgbPlugin] remote D455 verification warning: {exc}", flush=True)
        self._pil_image, self._np = PilImage, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, _LATEST_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_rgb_encoder")
        self._encoder.start()
        print(f"[CameraRgbPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _start_remote_realsense_if_configured(self):
        """Optionally start the D455 on its owning developer container via SSH.

        This is intentionally opt-in: XOS can also own the camera, and the
        launch never restarts a live driver. Q5 documents this developer
        account as part of its external-development workflow.
        """
        if not self._remote_start.get("enabled", False):
            return
        host = str(self._remote_start.get("host", "192.168.8.100"))
        user = str(self._remote_start.get("user", "developer"))
        password = str(self._remote_start.get("password", ""))
        try:
            port = int(self._remote_start.get("port", 2222))
        except (TypeError, ValueError):
            raise ValueError("camera_rgb.remote_start.port must be an integer")
        profiles = (
            str(self._remote_start.get("depth_profile", "848x480x30")),
            str(self._remote_start.get("color_profile", "848x480x30")),
        )
        if (not password or not re.fullmatch(r"[A-Za-z0-9.-]+", host) or
                not re.fullmatch(r"[A-Za-z0-9_-]+", user) or
                any(not re.fullmatch(r"[0-9]+x[0-9]+x[0-9]+", value) for value in profiles)):
            raise ValueError("invalid camera_rgb.remote_start configuration")
        # `nohup ... &` alone is not evidence that the remote launch survived
        # the SSH session. Keep a PID/log file and synchronously verify the
        # process, otherwise the UI only sees a permanent black frame.
        remote = f'''#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if ! pgrep -f realsense2_camera_node >/dev/null; then
  nohup ros2 launch realsense2_camera rs_align_depth_launch.py \\
    depth_module.depth_profile:={profiles[0]} \\
    rgb_camera.color_profile:={profiles[1]} \\
    </dev/null >/tmp/q5-realsense.log 2>&1 &
  echo $! >/tmp/q5-realsense.pid
fi
sleep 5
if ! pgrep -f realsense2_camera_node >/dev/null; then
  echo 'RealSense process did not remain running'
  test -f /tmp/q5-realsense.log && tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
if ! ros2 topic info /camera/camera/color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo 'RealSense RGB publisher is unavailable'
  tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
if ! ros2 topic info /camera/camera/aligned_depth_to_color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo 'RealSense aligned-depth publisher is unavailable'
  tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
echo 'RealSense process is running'
cat /tmp/q5-realsense.pid 2>/dev/null || true
'''
        command = ["sshpass", "-p", password, "ssh", "-p", str(port),
                   "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
                   "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
                   "-o", "UserKnownHostsFile=/dev/null",
                   f"{user}@{host}", "bash", "-s"]
        result = subprocess.run(command, input=remote, capture_output=True, text=True, timeout=20)
        if result.returncode:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"remote RealSense launch failed: {detail}")
        print(f"[CameraRgbPlugin] remote D455 verified on {user}@{host}:{port}: "
              f"{result.stdout.strip()}", flush=True)

    def _on_image(self, msg):
        if self._running:
            with self._lock:
                self._latest = msg
                self._frames_received += 1

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
                time.sleep(0.005)
                continue
            try:
                channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
                if channels is None or msg.step < msg.width * channels:
                    continue
                raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
                image = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
                image = image.reshape(msg.height, msg.width, channels)
                if msg.encoding == "bgr8":
                    image = image[:, :, ::-1]
                elif msg.encoding == "rgba8":
                    image = image[:, :, :3]
                elif msg.encoding == "bgra8":
                    image = image[:, :, [2, 1, 0]]
                image = self._np.ascontiguousarray(image)
                encoded = io.BytesIO()
                self._pil_image.fromarray(image, "RGB").save(
                    encoded, format="JPEG", quality=self._jpeg_quality)
                self._send_media({"kind": "rgb", "data": encoded.getvalue(),
                                  "width": int(msg.width), "height": int(msg.height),
                                  "encoding": msg.encoding, "timestamp_ms": int(time.time() * 1000)})
                self._frames_sent += 1
                self._last_sent = time.monotonic()
            except Exception as exc:
                self._node.get_logger().warn(f"RGB encode failed: {exc}")


class CameraDepthPlugin(_Q5MediaPlugin):
    """D455 aligned depth rendered as a dimension-preserving grayscale JPEG."""

    _node_name = "q5_camera_depth"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        # Preserve the canonical raw depth topic's ROS message type. The card
        # exposes a separate JPEG preview topic so Agent Core never sees two
        # incompatible message types on one DDS path.
        self._topic = f"/{namespace}/camera/depth_preview"
        self._near_depth_mm = max(1.0, float(plugin_config.get("near_depth_m", 0.25)) * 1000.0)
        self._far_depth_mm = max(self._near_depth_mm + 1.0,
                                 float(plugin_config.get("far_depth_m", 4.0)) * 1000.0)
        self._depth_gamma = max(0.25, min(2.0, float(plugin_config.get("gamma", 0.70))))
        self._jpeg_quality = max(60, min(95, int(plugin_config.get("jpeg_quality", 88))))
        self._auto_contrast = bool(plugin_config.get("auto_contrast", True))
        self._display_range_mm = None
        self._frames_received = 0
        self._frames_sent = 0
        self._latest = None
        self._lock = threading.Lock()
        self._encoder = None
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_depth", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 aligned depth preview. Adaptive sunrise/seafoam distance colors: near is warm and bright, far is muted teal; invalid depth is black.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from PIL import Image as PilImage
        from sensor_msgs.msg import Image
        self._pil_image, self._np = PilImage, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_depth_encoder")
        self._encoder.start()
        print(f"[CameraDepthPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_depth(self, msg):
        if not self._running:
            return
        # Do no image work on the ROS executor. Pointcloud shares this depth
        # source, and callback-side JPEG/XYZ work otherwise causes both cards
        # to miss source frames under a constrained container CPU quota.
        with self._lock:
            self._latest = msg
            self._frames_received += 1

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
                time.sleep(0.005)
                continue
            self._encode_depth(msg)

    def _encode_depth(self, msg):
        if msg.encoding not in ("16UC1", "mono16"):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        try:
            dtype = self._np.dtype(">u2" if msg.is_bigendian else "<u2")
            depth = self._np.frombuffer(msg.data[:needed], dtype=dtype).reshape(msg.height, msg.step // 2)
            depth = depth[:, :msg.width].astype(self._np.float32)
            # D455 Z16 is millimetres. Adapt the display range to the visible
            # scene so ordinary indoor geometry does not collapse into one
            # muddy color. Smooth the percentile range across frames to avoid
            # visual flicker while preserving configured physical bounds.
            low, high = self._near_depth_mm, self._far_depth_mm
            valid_depth = depth[depth > 0]
            if self._auto_contrast and valid_depth.size >= 32:
                target_low = float(self._np.clip(self._np.percentile(valid_depth, 2), low, high))
                target_high = float(self._np.clip(self._np.percentile(valid_depth, 98), low, high))
                if target_high - target_low >= 100.0:
                    if self._display_range_mm is None:
                        self._display_range_mm = (target_low, target_high)
                    else:
                        previous_low, previous_high = self._display_range_mm
                        self._display_range_mm = (
                            previous_low * 0.80 + target_low * 0.20,
                            previous_high * 0.80 + target_high * 0.20,
                        )
                    low, high = self._display_range_mm
            normalized = self._np.clip(
                (depth - low) / (high - low), 0.0, 1.0)
            normalized = normalized ** self._depth_gamma
            stops = self._np.array([
                # Keep the preview light and legible: warm near-field tones
                # provide separation, while distant geometry fades to a
                # muted seafoam instead of a heavy indigo background.
                (255, 248, 220), (255, 216, 157), (239, 151, 119),
                (137, 202, 181), (72, 145, 151),
            ], dtype=self._np.float32)
            scaled = normalized * (len(stops) - 1)
            lower = self._np.floor(scaled).astype(self._np.intp)
            upper = self._np.minimum(lower + 1, len(stops) - 1)
            fraction = (scaled - lower)[..., None]
            color = ((1.0 - fraction) * stops[lower] + fraction * stops[upper]).astype(self._np.uint8)
            color[depth <= 0] = 0
            encoded = io.BytesIO()
            # Depth color edges are especially sensitive to JPEG chroma
            # subsampling, so keep full chroma resolution for a clean preview.
            self._pil_image.fromarray(color, "RGB").save(
                encoded, format="JPEG", quality=self._jpeg_quality, subsampling=0)
            self._send_media({"kind": "depth_jpeg", "data": encoded.getvalue()})
        except Exception as exc:
            self._node.get_logger().warn(f"Depth preview encode failed: {exc}")
            return
        self._frames_sent += 1
        self._last_sent = time.monotonic()


class CameraPointCloudPlugin(_Q5MediaPlugin):
    """Reconstruct a bounded XYZ cloud from D455 aligned depth and intrinsics."""

    _node_name = "q5_camera_pointcloud"
    _format = "sensor/pointcloud"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get(
            "source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        self._info_topic = str(plugin_config.get(
            "camera_info_topic", "/camera/camera/color/camera_info"))
        self._topic = f"/{namespace}/camera/pointcloud"
        self._max_points = max(100, min(50000, int(plugin_config.get("max_points", 10000))))
        self._min_depth_m = max(0.0, float(plugin_config.get("min_depth_m", 0.25)))
        self._max_depth_m = max(self._min_depth_m, float(plugin_config.get("max_depth_m", 5.0)))
        self._camera_mount_pitch_rad = float(plugin_config.get("camera_mount_pitch_rad", 0.14655))
        self._floor_offset_m = max(0.0, float(plugin_config.get("floor_offset_m", 1.15)))
        self._intrinsics = None
        self._frames_received = 0
        self._frames_sent = 0
        self._info_subscription = None
        self._latest = None
        self._lock = threading.Lock()
        self._encoder = None
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_pointcloud", "type": "sensor", "multiInstance": False,
            "description": f"Q5 D455 aligned-depth XYZ point cloud, rendered as a forward-facing camera view. Limited to {self._max_points:,} points/frame; this is not 360-degree lidar.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from sensor_msgs.msg import CameraInfo, Image
        self._np = np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        if self._info_subscription is None:
            self._info_subscription = self._node.create_subscription(
                CameraInfo, self._info_topic, self._on_info, _RELIABLE_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_pointcloud_encoder")
        self._encoder.start()
        print(f"[CameraPointCloudPlugin] subscribed {self._source_topic} + {self._info_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_info(self, msg):
        fx, fy, cx, cy = float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        if fx > 0.0 and fy > 0.0:
            with self._lock:
                self._intrinsics = (fx, fy, cx, cy)

    def _on_depth(self, msg):
        if not self._running:
            return
        with self._lock:
            self._latest = msg
            self._frames_received += 1

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
                intrinsics = self._intrinsics
            if (msg is None or intrinsics is None or
                    time.monotonic() - self._last_sent < 1.0 / self._max_hz):
                time.sleep(0.005)
                continue
            self._encode_pointcloud(msg, intrinsics)

    def _encode_pointcloud(self, msg, intrinsics):
        if msg.encoding not in ("16UC1", "mono16"):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        try:
            dtype = self._np.dtype(">u2" if msg.is_bigendian else "<u2")
            depth = self._np.frombuffer(msg.data[:needed], dtype=dtype).reshape(msg.height, msg.step // 2)
            depth = depth[:, :msg.width].astype(self._np.float32) * 0.001
            stride = max(1, int(((msg.width * msg.height) / self._max_points) ** 0.5 + 0.999))
            z = depth[::stride, ::stride]
            valid = (z >= self._min_depth_m) & (z <= self._max_depth_m)
            if not valid.any():
                return
            rows, cols = self._np.indices(z.shape, dtype=self._np.float32)
            rows *= stride
            cols *= stride
            fx, fy, cx, cy = intrinsics
            camera_x = (cols - cx) * z / fx  # right
            camera_y = (rows - cy) * z / fy  # down
            # The depth image is in the D455 optical frame.  Render it in a
            # Q5 body-level frame instead: account for the fixed D455 mount
            # angle and the current neck pitch, then put the camera origin at
            # its configured height over the floor.
            joints = self._client.snapshot().get("joints") or {}
            neck_pitch = float(joints.get("neck_pitch_joint", 0.0))
            pitch = self._camera_mount_pitch_rad + neck_pitch
            cosine, sine = self._np.cos(pitch), self._np.sin(pitch)
            camera_up = -camera_y
            body_up = cosine * camera_up - sine * z + self._floor_offset_m
            body_forward = sine * camera_up + cosine * z
            # Agent Core's point-cloud renderer maps packet (x, y, z) to
            # display (y, -z, -x). The renderer's horizontal convention is
            # opposite the D455 optical axis, so mirror camera-right here.
            points = self._np.stack((-body_forward, -camera_x, -body_up), axis=-1)[valid]
            if not len(points):
                return
            points = self._np.ascontiguousarray(points.astype("<f4", copy=False))
            payload = struct.pack("<II", 12, len(points)) + points.tobytes()
            self._send_media({"kind": "pointcloud", "data": payload})
        except Exception as exc:
            self._node.get_logger().warn(f"Camera point-cloud encode failed: {exc}")
            return
        self._frames_sent += 1
        self._last_sent = time.monotonic()


def _wait_for_future(future, timeout_sec: float):
    """Wait for work completed by main.py's shared executor thread."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


class AudioPlugin:
    """Vendor audio playback via /audio_player/play and paired services."""

    _xos_audio_list_path = "/robot/replay/tts/list?lang=zh"
    _xos_audio_upload_path = "/robot/replay/tts/upload_audio?lang=zh"
    _xos_audio_check_path = "/robot/replay/tts/check_audio_exist?lang=zh"
    _xos_audio_delete_path = "/robot/replay/tts/delete?lang=zh"
    _xos_chat_launch_path = "/robot/chat/launch_chat?lang=zh"
    _xos_chat_state_path = "/robot/chat/get_chat_launch_state?lang=zh"
    _xos_chat_quit_path = "/robot/chat/quit_chat?lang=zh"

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")
        self._upload_max_bytes = max(1, int(plugin_config.get("upload_max_bytes", 20 * 1024 * 1024)))
        self._xos_http_base = str(plugin_config.get("xos_http_base", "http://192.168.8.100:1888")).rstrip("/")
        self._agent_core_url = str(plugin_config.get(
            "agent_core_url", os.environ.get("AGENT_CORE_URL", "https://localhost:15678"))).rstrip("/")
        self._agent_core_token = str(plugin_config.get(
            "agent_core_token", os.environ.get(
                "AGENT_CORE_TOKEN", os.environ.get(
                    "AGENT_CORE_ACCESS_TOKEN", os.environ.get("ACCESS_TOKEN", "")))))
        self._upload_dir = str(plugin_config.get("agent_core_upload_dir", "/tmp/uploads"))
        self._volume = max(0, min(100, int(plugin_config.get("volume", 50))))
        # XOS chat owns the vendor audio route. Serialize play/launch/quit so
        # two MCP calls cannot tear down one another's playback session.
        self._chat_poll_timeout = max(1.0, float(plugin_config.get("chat_poll_timeout_s", 8.0)))
        self._chat_route_settle = max(0.0, float(plugin_config.get("chat_route_settle_s", 1.5)))

    def get_tool(self):
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 XOS audio playback. Audio ensures XOS conversation is ON and leaves it ON; speaker owns the OFF/live-ALSA route.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": [
                    "play_local_file", "play_library_file",
                    "list_library", "upload_to_library", "delete_audio", "set_volume",
                    "get_volume", "stop_audio", "is_play", "stop"],
                    "oneOf": [
                        {"const": "play_local_file", "title": "播放本地音频文件"},
                        {"const": "play_library_file", "title": "播放机器人音频库文件"},
                        {"const": "list_library", "title": "查看挂载音频库"},
                        {"const": "upload_to_library", "title": "上传音频到机器人库"},
                        {"const": "delete_audio", "title": "从 XOS 音频库删除"},
                        {"const": "set_volume", "title": "设置音量"},
                        {"const": "get_volume", "title": "查看音量"},
                        {"const": "stop_audio", "title": "停止播放"},
                        {"const": "is_play", "title": "查询播放状态"},
                        {"const": "stop", "title": "停止音频卡"},
                    ]},
                "file_name": {"type": "string", "title": "音频文件名", "minLength": 1},
                "local_file": {"type": "string", "format": "file", "accept": "audio/*",
                               "title": "本地音频文件"},
                "force_play": {"type": "boolean", "title": "强制打断当前播放"},
                "timeout": {"type": "integer", "title": "超时 (s)", "minimum": 0},
                "channel": {"type": "string", "title": "播放通道",
                            "enum": ["default", "channel1", "channel2", "channel3"],
                            "default": "channel1"},
                "version": {"type": "string", "title": "音频版本", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "title": "音量", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "play_local_file": {"params": ["local_file", "force_play", "timeout", "channel", "version"],
                                        "description": "选择本地 WAV/MP3；自动从 Agent Core 读取、上传至 XOS 并播放。"},
                    "play_library_file": {"params": ["file_name", "force_play", "timeout", "channel", "version"],
                                         "description": "直接播放机器人 XOS 音频库已有的 audio_name。"},
                    "list_library": {"params": [], "description": "列出机器人 XOS 音频库；返回的 file_name 可用于 play_library_file。"},
                    "upload_to_library": {"params": ["local_file"], "description": "选择本地 WAV/MP3 并上传到机器人音频库，不播放。"},
                    "delete_audio": {"params": ["file_name"],
                                     "description": "从 XOS 音频库删除指定 audio_name。"},
                    "set_volume": {"params": ["volume"], "description": "设置厂商 AudioPlay 音量 0 到 100；不控制 live speaker。"},
                    "get_volume": {"params": [], "description": "查看本卡最近设置的 XOS 播放音量。"},
                    "stop_audio": {"params": [], "description": "停止当前厂商音频播放。"},
                    "is_play": {"params": [], "description": "查询当前是否正在播放。"},
                    "stop": {"params": [], "description": "停止音频卡并停止当前播放。"},
                }},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device,
                    "agent_core_upload_dir": self._upload_dir}
        if action in ("play_local_file", "play_library_file"):
            mode = {"play_local_file": None, "play_library_file": 3}[action]
            return self._queue_play(args, mode, action)
        # Compatibility-only entry points. `path` and `item` are vendor
        # advanced modes with no public resource-discovery contract, so they
        # are intentionally absent from the normal card schema.
        play_modes = {"play_by_path": 1, "play_by_item": 2, "play_by_file_name": 3}
        if action in play_modes:
            return self._play(args, play_modes[action])
        if action == "list_library":
            return self._list_robot_audio_files()
        if action == "upload_to_library":
            return self._upload_local_file(args)
        if action == "delete_audio":
            return self._delete_audio(args)
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "get_volume":
            return {"state": "ok", "volume": self._volume,
                    "note": "XOS exposes set_volume but no volume-read service; this is the most recent requested value."}
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    def _queue_play(self, args, mode, action):
        action_id = str(args.get("action_id") or f"audio_{action}_{int(time.time() * 1000)}")
        def run():
            try:
                if action == "play_local_file":
                    result = self._play_uploaded_local_file(args)
                else:
                    result = self._play(args, mode)
                _acp_notify(action_id, "completed" if result.get("state") in ("ok", "running") else "error",
                            result, "audio")
            except Exception as exc:
                _acp_notify(action_id, "error", {"state": "error", "message": str(exc)}, "audio")
        threading.Thread(target=run, daemon=True, name=f"q5_audio_{action}").start()
        return {"status": "queued", "action_id": action_id, "source": action}

    @staticmethod
    def _valid_upload_name(file_name) -> bool:
        if not isinstance(file_name, str) or file_name != os.path.basename(file_name):
            return False
        if not file_name or file_name.startswith(".") or any(ord(char) < 32 for char in file_name):
            return False
        return os.path.splitext(file_name)[1].lower() in (".wav", ".mp3")

    def _agent_core_file_payload(self, local_file):
        """Read a canvas-selected file through Agent Core's existing file API."""
        if not isinstance(local_file, str) or not local_file:
            return None, None, "local_file is required"
        normalized = os.path.normpath(local_file)
        upload_root = os.path.normpath(self._upload_dir)
        if os.path.dirname(normalized) != upload_root:
            return None, None, f"local_file must be selected from {upload_root}"
        file_name = os.path.basename(normalized)
        if not self._valid_upload_name(file_name):
            return None, None, "local_file must have a .wav or .mp3 filename"
        query = urllib.parse.urlencode({"path": normalized})
        request = urllib.request.Request(
            f"{self._agent_core_url}/api/file/download?{query}",
            headers={"Accept": "application/octet-stream",
                     **({"Authorization": f"Bearer {self._agent_core_token}"}
                        if self._agent_core_token else {})})
        context = ssl._create_unverified_context() if self._agent_core_url.startswith("https://") else None
        try:
            with urllib.request.urlopen(request, timeout=45.0, context=context) as reply:
                declared = reply.headers.get("Content-Length")
                if declared and int(declared) > self._upload_max_bytes:
                    return None, None, f"audio file exceeds {self._upload_max_bytes} byte upload limit"
                payload = reply.read(self._upload_max_bytes + 1)
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            return None, None, f"cannot read selected Agent Core file: {exc}"
        if len(payload) > self._upload_max_bytes:
            return None, None, f"audio file exceeds {self._upload_max_bytes} byte upload limit"
        if not payload:
            return None, None, "audio file is empty"
        return file_name, payload, None

    def _upload_local_file(self, args):
        file_name, payload, error = self._agent_core_file_payload(args.get("local_file"))
        if error:
            return {"state": "error", "message": error}
        return self._upload_payload(file_name, payload)

    def _play_uploaded(self, upload_result, args, source):
        if upload_result.get("state") != "ok":
            return upload_result
        play_args = {
            "file_name": upload_result["file_name"],
            "force_play": args.get("force_play", True),
            "timeout": args.get("timeout", 0),
            "channel": args.get("channel", "channel1"),
            "version": args.get("version", "v1"),
        }
        result = self._play(play_args, 3)
        return {**result, "source": source, "file_name": upload_result["file_name"],
                "bytes_uploaded": upload_result.get("bytes_uploaded")}

    def _play_uploaded_local_file(self, args):
        return self._play_uploaded(self._upload_local_file(args), args, "local_file")

    def _list_robot_audio_files(self):
        response = self._xos_json_request(self._xos_audio_list_path)
        if response.get("code") != 200:
            return {"state": "error", "message": response.get("msg", "XOS audio list failed")}
        files = []
        for entry in response.get("data") or []:
            if not isinstance(entry, dict) or not entry.get("audio_name"):
                continue
            files.append({"file_name": entry["audio_name"], "audio_name": entry["audio_name"],
                          "bytes": entry.get("size"), "file_size": entry.get("file_size"),
                          "duration_ms": entry.get("duration_ms"),
                          "duration_s": entry.get("duration_s"),
                          "create_time": entry.get("create_time")})
        return {"state": "ok", "files": files, "play_action": "play_library_file",
                "id_mapping_available": False,
                "note": "XOS exposes audio_name but no numeric audio-library ID in this endpoint."}

    def _delete_audio(self, args):
        file_name = args.get("file_name")
        if not isinstance(file_name, str) or not file_name.strip():
            return {"state": "error", "message": "file_name is required"}
        response = self._xos_json_request(
            self._xos_audio_delete_path, method="POST",
            payload={"audio_name": file_name})
        if response.get("code") != 200:
            return {"state": "error", "message": response.get("msg", "XOS audio delete failed")}
        return {"state": "ok", "audio_name": file_name, "message": response.get("msg", "success")}

    def _upload_payload(self, file_name: str, payload: bytes):
        if not payload:
            return {"state": "error", "message": "audio file is empty"}
        if len(payload) > self._upload_max_bytes:
            return {"state": "error", "message": f"audio file exceeds {self._upload_max_bytes} byte upload limit"}
        try:
            check = self._xos_json_request(
                self._xos_audio_check_path, method="POST",
                payload={"audio_name": file_name})
            if check.get("code") == 200:
                already_exists = bool((check.get("data") or {}).get("exist"))
            else:
                already_exists = None
            boundary = "----PhanthymotusQ5AudioBoundary"
            body = (f"--{boundary}\r\n"
                    f"Content-Disposition: form-data; name=\"file\"; filename=\"{file_name}\"\r\n"
                    "Content-Type: application/octet-stream\r\n\r\n").encode() + payload + \
                   f"\r\n--{boundary}--\r\n".encode()
            request = urllib.request.Request(
                self._xos_http_base + self._xos_audio_upload_path,
                data=body, method="POST",
                headers={"Content-Type": f"multipart/form-data; boundary={boundary}",
                         "X-Requested-With": "XMLHttpRequest"})
            with urllib.request.urlopen(request, timeout=45.0) as reply:
                response = json.loads(reply.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return {"state": "error", "message": f"XOS audio upload failed: {exc}"}
        if response.get("code") != 200:
            return {"state": "error", "message": response.get("msg", "XOS audio upload failed")}
        return {
            "state": "ok", "file_name": file_name, "audio_name": file_name,
            "bytes_uploaded": len(payload), "next_action": "play_library_file",
            "play_args": {"file_name": file_name, "force_play": True, "timeout": 0},
            "already_exists": already_exists,
            "note": "Registered through the XOS audio-library API. XOS exposes audio_name, not a numeric ID.",
        }

    def _xos_json_request(self, path, method="GET", payload=None):
        body = None
        headers = {"Accept": "application/json", "X-Requested-With": "XMLHttpRequest"}
        if payload is not None:
            body = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            self._xos_http_base + path,
            data=body, method=method, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=10.0) as reply:
                return json.loads(reply.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return {"code": 0, "msg": f"XOS audio API unavailable: {exc}"}

    def _xos_chat_is_on(self):
        response = self._xos_json_request(self._xos_chat_state_path, method="POST", payload={})
        if response.get("code") != 200:
            return None, response.get("msg", "XOS chat state unavailable")
        return str(response.get("data", "")).upper() == "ON", None

    def _xos_chat_set(self, path):
        response = self._xos_json_request(path, method="POST", payload={})
        if response.get("code") != 200:
            return False, response.get("msg", "XOS chat request failed")
        return True, None

    def _xos_chat_wait(self, wanted_on):
        deadline = time.monotonic() + self._chat_poll_timeout
        last_error = None
        while time.monotonic() < deadline:
            state, error = self._xos_chat_is_on()
            if error:
                last_error = error
            elif state is wanted_on:
                return True, None
            time.sleep(0.2)
        return False, last_error or ("XOS chat did not reach " + ("ON" if wanted_on else "OFF"))

    def _xos_chat_start_for_playback(self):
        """Ensure XOS conversation owns the route for stored-audio playback."""
        state, error = self._xos_chat_is_on()
        if error:
            return None, error
        if state:
            # Chat may already report ON while the previous direct speaker
            # process is still draining its ALSA handle.
            if self._chat_route_settle:
                time.sleep(self._chat_route_settle)
            return True, None
        ok, error = self._xos_chat_set(self._xos_chat_launch_path)
        if not ok:
            return None, error
        ok, error = self._xos_chat_wait(True)
        if not ok:
            # launch_chat may have succeeded even when the state endpoint is
            # slow to converge; do not leave XOS chat occupying the speaker.
            self._xos_chat_stop_after_playback()
            return None, error
        # XOS reports ON before the audio action route is fully released. The
        # web UI naturally leaves a short gap between these operations.
        if self._chat_route_settle:
            time.sleep(self._chat_route_settle)
        return True, None

    def _xos_chat_stop_after_playback(self):
        ok, error = self._xos_chat_set(self._xos_chat_quit_path)
        if not ok:
            return False, error
        return self._xos_chat_wait(False)

    def _wait_audio_finished(self, timeout: float) -> bool:
        """Wait for the vendor player, not merely the action goal, to finish."""
        deadline = time.monotonic() + max(2.0, min(180.0, timeout))
        # The action server can report success as soon as playback is queued.
        # Poll the vendor status service before tearing down XOS chat.
        time.sleep(0.2)
        while time.monotonic() < deadline:
            if not self._srv_is_play.service_is_ready():
                return True
            response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 1.0)
            if response is not None and not response.success:
                return True
            time.sleep(0.2)
        return False

    def _play(self, args, mode: int):
        # XOS refuses vendor playback while its chat session is stopped. Keep
        # the session scoped to this request, unless the user had it enabled.
        with _Q5_XOS_CHAT_LOCK:
            return self._play_with_chat(args, mode)

    def _play_with_chat(self, args, mode: int):
        source_fields = {1: "path", 2: "item", 3: "file_name"}
        source_field = source_fields[mode]
        if source_field not in args:
            return {"state": "error", "message": f"mode {mode} requires {source_field}"}
        # Canvas forms may serialize every optional field with an empty
        # default. Ignore those defaults; reject a populated alternate source.
        def _populated(field):
            if field not in args:
                return False
            candidate = args.get(field)
            return isinstance(candidate, str) and bool(candidate.strip())

        unrelated = sorted(field for field in source_fields.values()
                           if field != source_field and _populated(field))
        if unrelated:
            return {"state": "error", "message": (
                f"mode {mode} only accepts {source_field}; do not provide {', '.join(unrelated)}")}
        value = args.get(source_field)
        if not isinstance(value, str) or not value.strip():
            return {"state": "error", "message": f"{source_field} must be a non-empty string"}
        # XOS conversation uses the Q5 full-duplex route.  Release both
        # direct-ALSA cards before enabling it; their normal streams are
        # restored after this request returns and chat is closed again.
        _stop_active_speaker_plugin()
        _stop_active_mic_plugin()
        chat_ready, chat_error = self._xos_chat_start_for_playback()
        if chat_ready is None:
            _start_active_mic_plugin()
            return {"state": "error", "message": f"cannot enable XOS chat for playback: {chat_error}"}
        try:
            if not self._action_client.wait_for_server(timeout_sec=3.0):
                return {"state": "error", "message": "/audio_player/play is unavailable"}
            goal = AudioPlay.Goal()
            goal.mode = mode
            # XOS reports a successful action even when another playback session
            # owns the route. Make an explicit audio-card request preempt that
            # session unless the caller opts out.
            goal.force_play = bool(args.get("force_play", True))
            # Send precisely one source field. Besides making the action contract
            # unambiguous, this shields XOS from controls retained by a previous
            # card-mode selection.
            goal.id = 0
            goal.path = str(value) if mode == 1 else ""
            goal.item = str(value) if mode == 2 else ""
            goal.file_name = str(value) if mode == 3 else ""
            # Q5's AudioPlay definition documents channel1 as the hardware
            # default.  Sending the UI-only literal "default" is accepted by
            # some firmware revisions but can produce a successful, silent
            # action, so normalize it before crossing the ROS boundary.
            channel = str(args.get("channel", "channel1")).strip()
            goal.channel = "channel1" if not channel or channel == "default" else channel
            goal.timeout = int(args.get("timeout", 0))
            goal.version = str(args.get("version", "v1"))
            goal_handle = _wait_for_future(self._action_client.send_goal_async(goal), 5.0)
            if goal_handle is None:
                return {"state": "error", "message": "audio goal timed out"}
            if not goal_handle.accepted:
                return {"state": "error", "message": "audio goal rejected"}
            response = _wait_for_future(goal_handle.get_result_async(), max(10.0, goal.timeout + 2.0))
            if response is None:
                return {"state": "error", "message": "audio result timed out"}
            if response.result.success:
                # timeout=0 means vendor default; allow enough time for normal
                # XOS clips while avoiding an unbounded MCP request.
                wait_timeout = goal.timeout if goal.timeout > 0 else 180.0
                if not self._wait_audio_finished(wait_timeout):
                    return {"state": "error", "message": "audio playback status timed out"}
            return {"state": "ok" if response.result.success else "error", "message": response.result.message}
        finally:
            # Do not leave XOS holding the duplex route between audio calls.
            # This gives mic and direct speaker a stable OFF-state baseline,
            # regardless of whether the container started with chat ON/OFF.
            stopped, stop_error = self._xos_chat_stop_after_playback()
            if not stopped:
                print(f"[AudioPlugin] XOS chat did not close after playback: {stop_error}", flush=True)
            elif self._chat_route_settle:
                time.sleep(self._chat_route_settle)
            _start_active_mic_plugin()

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
        if response.success:
            self._volume = req.volume
        return {"state": "ok" if response.success else "error", "volume": req.volume, "message": response.message}

    def _stop_audio(self):
        if not self._srv_stop.service_is_ready():
            return {"state": "error", "message": "/audio_player/stop_play is unavailable"}
        response = _wait_for_future(self._srv_stop.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "stop-audio request timed out"}
        return {"state": "ok" if response.success else "error", "message": response.message}

    def _is_playing(self):
        if not self._srv_is_play.service_is_ready():
            return {"state": "error", "message": "/audio_player/is_play is unavailable"}
        response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "is-play request timed out"}
        return {"state": "ok", "is_playing": response.success, "message": response.message}
