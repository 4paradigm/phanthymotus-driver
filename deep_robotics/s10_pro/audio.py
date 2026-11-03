"""S10 Pro microphone capture and PCM speaker playback cards."""

from __future__ import annotations

import os
import queue
import shlex
import shutil
import socket
import subprocess
import threading
import time

from common.vendor_runtime import action_schema, tool
from domain_bridge import CoreAudioSubscriber

AUDIO_FORMAT = "audio/pcm-16k"
AUDIO_MESSAGE_TYPE = "audio_msgs/msg/AudioChunk"
FRAME_BYTES = 1024  # 32 ms of mono PCM S16_LE at 16 kHz.
FRAME_DURATION_NS = FRAME_BYTES * 1_000_000_000 // (16_000 * 2)
MAX_AUDIO_FRAME_BYTES = 256 * 1024
AUDIO_EOF_MAGIC = b"\x01\x00\xff\xff\x01\x00\xff\xff"


def _pcm_card(device):
    device = str(device or "default")
    return device if device == "default" or device.startswith(("plughw:", "hw:", "sysdefault:")) else f"plughw:{device}"


def _capture_command(device):
    return ["arecord", "-q", "-D", _pcm_card(device), "-t", "raw",
            "-f", "S16_LE", "-r", "16000", "-c", "1"]


def _playback_command(device):
    return ["aplay", "-q", "-D", _pcm_card(device), "-t", "raw",
            "-f", "S16_LE", "-r", "16000", "-c", "1"]


def _is_local_address(address):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((address, 0))
        return True
    except OSError:
        return address in {"127.0.0.1", "0.0.0.0"}


def _audio_launch(program, device, mode, host, user, password):
    command = _capture_command(device) if program == "arecord" else _playback_command(device)
    if mode == "local":
        return command, None
    target = f"{user}@{host}" if user else host
    remote_command = "exec " + shlex.join(command)
    ssh = ["ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", "--",
           target, remote_command]
    if password:
        if not shutil.which("sshpass"):
            raise RuntimeError("sshpass is unavailable for configured AOS audio password")
        env = os.environ.copy()
        env["SSHPASS"] = password
        return ["sshpass", "-e", *ssh], env
    return ["ssh", "-o", "BatchMode=yes", *ssh[1:]], None


def _audio_transport(config):
    audio = config.get("audio", {})
    mode = os.environ.get("S10_AUDIO_MODE", audio.get("mode", "auto"))
    host = os.environ.get("S10_AUDIO_SSH_HOST") or audio.get("ssh_host", "10.21.33.103")
    user = os.environ.get("S10_AUDIO_SSH_USER", audio.get("ssh_user", "user"))
    password = os.environ.get("S10_AUDIO_SSH_PASSWORD",
                              os.environ.get("S10_DRMAP_PASSWORD", ""))
    if mode not in {"auto", "local", "ssh"}:
        raise ValueError("S10_AUDIO_MODE must be auto, local, or ssh")
    local = _is_local_address(str(host))
    if mode == "local" and not local:
        raise ValueError("local audio mode requested but the configured AOS address is not local")
    selected = "local" if mode == "local" or (mode == "auto" and local) else "ssh"
    capture_device = os.environ.get("S10_AUDIO_CAPTURE_DEVICE") or audio.get("capture_device")
    playback_device = os.environ.get("S10_AUDIO_PLAYBACK_DEVICE") or audio.get("playback_device")
    if selected == "ssh":
        remote_default = "plughw:CARD=rockchipes8388,DEV=0"
        capture_device = remote_default if capture_device in (None, "", "default") else capture_device
        playback_device = remote_default if playback_device in (None, "", "default") else playback_device
    else:
        capture_device = capture_device or "default"
        playback_device = playback_device or "default"
    return selected, str(host), str(user), str(password), str(capture_device), str(playback_device)


def _audio_tool(name, description, actions, *, topic_out=None, topic_in=None):
    result = tool(name, "sensor" if name == "mic" else "actuator", description,
                  action_schema(actions, {}))
    if topic_out:
        result["topic_out"] = topic_out
    if topic_in:
        result["topic_in"] = topic_in
    return result


class S10MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, nodes):
        self.nodes = nodes
        self.topic = f"/{namespace}/mic/audio" if namespace else "/mic/audio"
        (self.mode, self.host, self.user, self.password,
         self.device, _) = _audio_transport(config)
        self._publisher = None
        self._process = None
        self._reader = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._frames = 0
        self._last_error = None

    def get_tool(self):
        return _audio_tool(
            self.PREFIX, "S10 内置麦克风；输出单声道 PCM S16LE/16 kHz，连接到 ASR 音频输入。",
            {"start": ([], "Start microphone capture and publish PCM-16k frames."),
             "stop": ([], "Stop microphone capture."),
             "info": ([], "Read microphone capture state and frame count.")},
            topic_out=[{"topic": self.topic, "format": AUDIO_FORMAT,
                        "message_type": AUDIO_MESSAGE_TYPE}],
        )

    def start(self):
        result = self._start_capture()
        if result.get("state") == "error":
            print(f"[s10-mic] capture not started: {result.get('error')}", flush=True)
        else:
            print(f"[s10-mic] capture running mode={self.mode} host={self.host} "
                  f"device={self.device} topic={self.topic}", flush=True)

    def stop(self):
        self._stop_capture()

    def _start_capture(self):
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return {"state": "running", "topic_out": self.get_tool()["topic_out"]}
            if self.nodes is None:
                return {"state": "error", "error": "ROS 2 bridge is unavailable"}
            if self.mode == "local" and not shutil.which("arecord"):
                return {"state": "error", "error": "arecord is unavailable in the driver image"}
            if self._publisher is None:
                from audio_msgs.msg import AudioChunk
                self._publisher = self.nodes.bridge.create_publisher(
                    "mic_audio", self.topic, AudioChunk, queue_depth=32, latest_only=False)
            try:
                command, env = _audio_launch("arecord", self.device, self.mode,
                                             self.host, self.user, self.password)
                self._process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                                 stderr=subprocess.PIPE, bufsize=0, env=env)
            except (OSError, RuntimeError) as exc:
                self._last_error = str(exc)
                return {"state": "error", "error": self._last_error}
            if self._process.poll() is not None:
                error = self._process.stderr.read(2048).decode(errors="replace")
                self._last_error = error or f"arecord exited with {self._process.returncode}"
                self._process = None
                return {"state": "error", "error": self._last_error}
            self._stop.clear()
            self._frames = 0
            self._last_error = None
            self._reader = threading.Thread(target=self._capture_loop, daemon=True,
                                            name="s10-mic-capture")
            self._reader.start()
        return {"state": "running", "topic_out": self.get_tool()["topic_out"]}

    def _capture_loop(self):
        from audio_msgs.msg import AudioChunk

        process = self._process
        buffer = bytearray()
        next_frame_stamp_ns = None
        try:
            while not self._stop.is_set() and process is not None:
                chunk = process.stdout.read(FRAME_BYTES)
                if not chunk:
                    if process.poll() is not None:
                        details = process.stderr.read(2048).decode(errors="replace")
                        self._last_error = details or f"arecord exited with {process.returncode}"
                        break
                    time.sleep(0.005)
                    continue
                if next_frame_stamp_ns is None:
                    next_frame_stamp_ns = time.time_ns() - FRAME_DURATION_NS
                buffer.extend(chunk)
                while len(buffer) >= FRAME_BYTES:
                    payload = bytes(buffer[:FRAME_BYTES])
                    del buffer[:FRAME_BYTES]
                    message = AudioChunk()
                    message.header.stamp.sec = next_frame_stamp_ns // 1_000_000_000
                    message.header.stamp.nanosec = next_frame_stamp_ns % 1_000_000_000
                    message.format = AUDIO_FORMAT
                    message.data = payload
                    self._publisher.publish(message)
                    self._frames += 1
                    next_frame_stamp_ns += FRAME_DURATION_NS
        except (OSError, ValueError) as exc:
            if not self._stop.is_set():
                self._last_error = str(exc)

    def _stop_capture(self):
        with self._lock:
            process, self._process = self._process, None
            reader, self._reader = self._reader, None
            self._stop.set()
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
            if process.stdout:
                process.stdout.close()
            if process.stderr:
                process.stderr.close()
        if reader and reader is not threading.current_thread():
            reader.join(timeout=1)
        return {"state": "idle"}

    def dispatch(self, action, _args):
        if action == "start":
            return self._start_capture()
        if action == "stop":
            return self._stop_capture()
        if action == "info":
            running = self._process is not None and self._process.poll() is None
            result = {"state": "running" if running else "idle", "frames": self._frames,
                      "topic_out": self.get_tool()["topic_out"], "mode": self.mode,
                      "audio_host": self.host if self.mode == "ssh" else None}
            if self._last_error:
                result["error"] = self._last_error
            return result
        raise ValueError(f"unsupported S10 microphone action: {action}")


class S10SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config):
        (self.mode, self.host, self.user, self.password,
         _, self.device) = _audio_transport(config)
        self.domain = int(config.get("ros", {}).get("core_domain_id") or 42)
        self._subscriber = None
        self._process = None
        self._worker = None
        self._stop = threading.Event()
        self._topic = ""
        self._frames = 0
        self._dropped = 0
        self._last_error = None
        self._lock = threading.RLock()

    def get_tool(self):
        return _audio_tool(
            self.PREFIX, "S10 内置扬声器；播放连接输入的单声道 PCM S16LE/16 kHz TTS 音频。",
            {"start": (["input_topic"], "Subscribe to the connected TTS PCM-16k stream and play it."),
             "stop": ([], "Stop speaker playback."),
             "info": ([], "Read speaker subscription and playback state.")},
            topic_in=[{"format": AUDIO_FORMAT, "message_type": AUDIO_MESSAGE_TYPE}],
        )

    def start(self):
        pass

    def stop(self):
        self._stop_playback()

    def _start_playback(self, topic):
        topic = str(topic or "").strip()
        if not topic:
            return {"state": "error", "error": "input_topic is required"}
        with self._lock:
            if self._worker and self._worker.is_alive() and topic == self._topic:
                return {"state": "playing", "input_topic": topic}
            self._stop_playback()
            if self.mode == "local" and not shutil.which("aplay"):
                return {"state": "error", "error": "aplay is unavailable in the driver image"}
            try:
                command, env = _audio_launch("aplay", self.device, self.mode,
                                             self.host, self.user, self.password)
                process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                           bufsize=0, env=env)
            except (OSError, RuntimeError) as exc:
                self._last_error = str(exc)
                return {"state": "error", "error": self._last_error}
            if process.poll() is not None:
                self._last_error = process.stderr.read(2048).decode(errors="replace") or "aplay failed to start"
                process.stderr.close()
                return {"state": "error", "error": self._last_error}
            profile = os.environ.get("S10_CORE_FASTDDS_PROFILE", "/tmp/s10-pro-core-fastdds.xml")
            try:
                self._subscriber = CoreAudioSubscriber(profile, topic, domain=self.domain)
            except Exception as exc:
                process.terminate()
                process.wait(timeout=1)
                process.stdin.close()
                process.stderr.close()
                self._last_error = str(exc)
                return {"state": "error", "error": self._last_error}
            self._process = process
            self._topic = topic
            self._frames = self._dropped = 0
            self._last_error = None
            self._stop.clear()
            self._worker = threading.Thread(target=self._playback_loop, daemon=True,
                                            name="s10-speaker-playback")
            self._worker.start()
        return {"state": "playing", "input_topic": topic,
                "topic_in": [{"topic": topic, "format": AUDIO_FORMAT,
                              "message_type": AUDIO_MESSAGE_TYPE}]}

    def _playback_loop(self):
        while not self._stop.is_set():
            subscriber, process = self._subscriber, self._process
            if subscriber is None or process is None:
                return
            if process.poll() is not None:
                self._last_error = f"aplay exited with {process.returncode}"
                return
            if not subscriber.is_alive():
                self._last_error = "Core-domain audio subscriber exited"
                return
            item = subscriber.get(timeout=0.1)
            if item is None:
                continue
            audio_format, payload = item
            if (audio_format != AUDIO_FORMAT or not payload or payload == AUDIO_EOF_MAGIC
                    or len(payload) % 2 or len(payload) > MAX_AUDIO_FRAME_BYTES):
                self._dropped += 1
                continue
            try:
                process.stdin.write(payload)
                self._frames += 1
            except (BrokenPipeError, OSError) as exc:
                if not self._stop.is_set():
                    self._last_error = str(exc)
                return

    def _stop_playback(self):
        with self._lock:
            self._stop.set()
            subscriber, self._subscriber = self._subscriber, None
            process, self._process = self._process, None
            worker, self._worker = self._worker, None
            self._topic = ""
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
        if subscriber:
            subscriber.stop()
        if worker and worker is not threading.current_thread():
            worker.join(timeout=1)
        if process is not None:
            for stream in (process.stdin, process.stderr):
                if stream:
                    stream.close()
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            return self._start_playback(args.get("input_topic"))
        if action == "stop":
            return self._stop_playback()
        if action == "info":
            return {"state": "playing" if self._worker and self._worker.is_alive() else "idle",
                    "input_topic": self._topic or None, "frames_played": self._frames,
                    "frames_dropped": self._dropped, "last_error": self._last_error,
                    "mode": self.mode, "audio_host": self.host if self.mode == "ssh" else None}
        raise ValueError(f"unsupported S10 speaker action: {action}")
