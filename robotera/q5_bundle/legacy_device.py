"""Verified sensor and audio cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module contains the verified state, battery, audio, and D455 camera cards.
"""

from __future__ import annotations

import io
import re
import shlex
import subprocess
import threading
import time

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
    Q5ControlModePlugin,
)


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
# Handle and submit fixed 300 ms PCM blocks, matching the stable Unitree live
# speaker strategy.  The bridge has shown gaps up to 262 ms, so prefill two
# blocks before opening the audio gate.  This avoids USB-card underruns caused
# by scheduling a separate ALSA write for every arbitrary transport fragment.
input_block_bytes = 9600
prefill_bytes = input_block_bytes * 2
try:
  while True:
    chunk = sys.stdin.buffer.read(input_block_bytes)
    if not chunk: break
    pending.extend(chunk)
    if len(pending) < prefill_bytes:
      continue
    raw = bytes(pending[:input_block_bytes])
    del pending[:input_block_bytes]
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
        self._process = None
        self._thread = None
        self._running = False
        self._frames_sent = 0
        self._lock = threading.RLock()
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
            if self._running:
                return
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
                self.stop()
                print(f"[MicPlugin] capture unavailable: {exc}", flush=True)

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
            self._running = False
            _stop_remote_mic_capture()
            if self._process is not None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                self._process = None

    def dispatch(self, action, args):
        del args
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "frames_sent": self._frames_sent}
        return None


class SpeakerPlugin:
    """Play any canvas-connected PCM AudioChunk stream on Q5 ALSA output."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, executor
        self._client = client
        self._topic = ""
        self._device = str(plugin_config.get("device", "hw:2,0"))
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._output_rate = int(plugin_config.get("output_sample_rate_hz", 44100))
        self._output_channels = int(plugin_config.get("output_channels", 2))
        self._process = None
        self._thread = None
        self._running = False
        self._frames_received = 0
        self._frames_written = 0
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 speaker only supports the shared 16 kHz mono PCM contract")
        if self._output_rate not in (44100, 48000) or self._output_channels != 2:
            raise ValueError("Q5 speaker hardware requires 44.1/48 kHz stereo output")

    def get_tool(self):
        return {
            "name": "speaker", "type": "actuator", "multiInstance": False,
            "description": "Q5 speaker. Connect any audio/pcm-16k output (TTS, microphone, or other PCM source) to play it live.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
                "input_topic": {"type": "string", "description": "PCM 16 kHz AudioChunk topic from the canvas connection"},
            }, "required": ["action"], "additionalProperties": False},
            # Leave the topic unresolved until canvas supplies input_topic.
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self, input_topic=None):
        del input_topic
        # The canvas calls dispatch(start, {input_topic}) when an audio output
        # is connected. Do not subscribe to a hard-coded TTS topic at boot.
        return

    def _start_for_topic(self, requested: str) -> None:
        if self._running and requested == self._topic:
            return
        self.stop()
        try:
            self._start_playback(requested)
            print(f"[SpeakerPlugin] playback subscribed <- {self._topic}", flush=True)
        except Exception as exc:
            self.stop()
            print(f"[SpeakerPlugin] playback unavailable: {exc}", flush=True)

    def _start_playback(self, requested: str) -> None:
        self._topic = requested
        # Stop the vendor player when possible, but never let a temporarily
        # unavailable ROS service prevent the independent ALSA stream from
        # starting. The developer-container service discovery can block here.
        try:
            stopped = _q5_remote_command(
                "source /opt/ros/humble/setup.bash; "
                "timeout 2 ros2 service call /audio_player/stop_play std_srvs/srv/Trigger '{}'",
                timeout=4.0)
            if stopped.returncode:
                detail = (stopped.stderr or stopped.stdout).decode(errors="replace").strip()
                print(f"[SpeakerPlugin] vendor player was not stopped: {detail}", flush=True)
        except Exception as exc:
            print(f"[SpeakerPlugin] vendor player stop timed out; continuing with ALSA: {exc}", flush=True)
        command = _q5_alsa_speaker_command(
            self._device, self._output_rate, self._output_channels)
        self._process = subprocess.Popen(
            _q5_ssh_args("python3 -u -c " + shlex.quote(command)), stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0)
        _raise_if_remote_process_exited(self._process, "speaker")
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

    def dispatch(self, action, args):
        if action == "start":
            requested = str(args.get("input_topic") or "")
            if not requested:
                return {"ok": False, "code": "INPUT_TOPIC_REQUIRED",
                        "message": "Connect an audio/pcm-16k output to speaker before starting playback"}
            self._start_for_topic(requested)
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_in": ([{"topic": self._topic, "format": "audio/pcm-16k"}]
                                 if self._topic else [{"format": "audio/pcm-16k"}]),
                    "playback": {"device": self._device, "sample_rate_hz": self._output_rate,
                                 "channels": self._output_channels},
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

        self._start_remote_realsense_if_configured()
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
        self._frames_received = 0
        self._frames_sent = 0
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_depth", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 aligned depth preview. Grayscale: near is dark, far is bright.",
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
        print(f"[CameraDepthPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_depth(self, msg):
        if not self._running:
            return
        self._frames_received += 1
        if (msg.encoding not in ("16UC1", "mono16") or
                time.monotonic() - self._last_sent < 1.0 / self._max_hz):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        try:
            dtype = self._np.dtype(">u2" if msg.is_bigendian else "<u2")
            depth = self._np.frombuffer(msg.data[:needed], dtype=dtype).reshape(msg.height, msg.step // 2)
            depth = depth[:, :msg.width].astype(self._np.float32)
            # D455 Z16 is millimetres. A fixed 0.25-5.0 m window is stable
            # across frames and avoids the confusing red/green pseudo-colors.
            preview = self._np.clip((depth - 250.0) * (255.0 / 4750.0), 0, 255).astype(self._np.uint8)
            encoded = io.BytesIO()
            self._pil_image.fromarray(preview, "L").save(encoded, format="JPEG", quality=75)
            self._send_media({"kind": "depth_jpeg", "data": encoded.getvalue()})
        except Exception as exc:
            self._node.get_logger().warn(f"Depth preview encode failed: {exc}")
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

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")

    def get_tool(self):
        play_actions = {
            "play_by_id": {"mode": 0, "title": "按内置音频 ID 播放", "param": "id"},
            "play_by_path": {"mode": 1, "title": "按设备路径播放", "param": "path"},
            "play_by_item": {"mode": 2, "title": "按 item JSON 播放", "param": "item"},
            "play_by_file_name": {"mode": 3, "title": "按文件名播放", "param": "file_name"},
        }
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 vendor audio playback, volume, stop, and status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": [
                    "start", *play_actions, "set_volume", "stop_audio", "is_play", "stop", "info"],
                    "oneOf": [
                        {"const": "start", "title": "检查音频服务"},
                        *[{"const": action, "title": detail["title"]}
                          for action, detail in play_actions.items()],
                        {"const": "set_volume", "title": "设置音量"},
                        {"const": "stop_audio", "title": "停止播放"},
                        {"const": "is_play", "title": "查询播放状态"},
                        {"const": "stop", "title": "停止音频卡"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                "id": {"type": "integer", "title": "内置音频 ID"},
                "path": {"type": "string", "title": "设备音频路径", "minLength": 1},
                "item": {"type": "string", "title": "item JSON", "minLength": 1},
                "file_name": {"type": "string", "title": "音频文件名", "minLength": 1},
                "force_play": {"type": "boolean", "title": "强制打断当前播放"},
                "timeout": {"type": "integer", "title": "超时 (s)", "minimum": 0},
                "channel": {"type": "string", "title": "播放通道",
                            "enum": ["default", "channel1", "channel2", "channel3"]},
                "version": {"type": "string", "title": "音频版本", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "title": "音量", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 Q5 厂商音频服务。"},
                    **{action: {"params": [detail["param"], "force_play", "timeout", "channel", "version"],
                                  "description": f"模式 {detail['mode']}；只接受 {detail['param']} 作为播放来源。"}
                       for action, detail in play_actions.items()},
                    "set_volume": {"params": ["volume"], "description": "设置 0 到 100 的播放音量。"},
                    "stop_audio": {"params": [], "description": "停止当前厂商音频播放。"},
                    "is_play": {"params": [], "description": "查询当前是否正在播放。"},
                    "stop": {"params": [], "description": "停止音频卡并停止当前播放。"},
                    "info": {"params": [], "description": "查看音频服务状态。"},
                }},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device}
        play_modes = {"play_by_id": 0, "play_by_path": 1,
                      "play_by_item": 2, "play_by_file_name": 3}
        if action in play_modes:
            return self._play(args, play_modes[action])
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    def _play(self, args, mode: int):
        source_fields = {0: "id", 1: "path", 2: "item", 3: "file_name"}
        source_field = source_fields[mode]
        if source_field not in args:
            return {"state": "error", "message": f"mode {mode} requires {source_field}"}
        unrelated = sorted(field for field in source_fields.values()
                           if field != source_field and field in args)
        if unrelated:
            return {"state": "error", "message": (
                f"mode {mode} only accepts {source_field}; do not provide {', '.join(unrelated)}")}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "/audio_player/play is unavailable"}
        goal = AudioPlay.Goal()
        goal.mode = mode
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.file_name = str(args.get("file_name", ""))
        goal.channel = str(args.get("channel", "default"))
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
        return {"state": "ok" if response.result.success else "error", "message": response.result.message}

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
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
