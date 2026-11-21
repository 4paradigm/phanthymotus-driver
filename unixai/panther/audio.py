"""Canvas audio cards using the host PulseAudio server on Agent Core domain 42."""

from __future__ import annotations

from array import array
import multiprocessing as mp
import glob
import os
import queue
import subprocess
import sys
import threading
import time


FORMAT = "audio/pcm-16k"
CHUNK_BYTES = 2048  # 64 ms, S16LE mono at 16 kHz.


def _validate_volume(value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
        raise ValueError("volume must be an integer from 0 to 100")
    return value


def _speaker_pcm(raw: bytes, volume: int) -> bytes:
    """Attenuate only this card's playback; leave the source topic and sink alone."""
    if volume == 100:
        return raw
    if volume == 0:
        return bytes(len(raw))
    samples = array("h")
    samples.frombytes(raw)
    if sys.byteorder != "little":
        samples.byteswap()
    # Truncate toward zero symmetrically, avoiding a negative DC bias.
    samples = array("h", ((1 if sample >= 0 else -1) * (abs(sample) * volume // 100)
                          for sample in samples))
    if sys.byteorder != "little":
        samples.byteswap()
    return samples.tobytes()


def _complete_chunks(pending: bytearray, incoming: bytes):
    pending.extend(incoming)
    while len(pending) >= CHUNK_BYTES:
        chunk = bytes(pending[:CHUNK_BYTES])
        del pending[:CHUNK_BYTES]
        yield chunk


def _pulse_command(mode: str, device: str) -> list[str]:
    executable = "parec" if mode == "mic" else "paplay"
    return [executable, "--raw", "--format=s16le", "--rate=16000",
            "--channels=1", "--latency-msec=80", "--process-time-msec=40",
            f"--device={device}"]


def _pulse_identity(config: dict) -> dict[str, int]:
    return {"user": int(config["uid"]), "group": int(config["gid"])}


def _pulse_user(config: dict) -> dict[str, int]:
    if os.geteuid() != 0:
        if os.geteuid() != int(config["uid"]):
            raise RuntimeError("audio must run as the configured PulseAudio user or root")
        return {}
    return _pulse_identity(config)


def _resolve_pulse(config: dict) -> dict:
    """Discover a live host server on every start; vendor sockets change on reboot."""
    resolved = dict(config)
    server = config["server"]
    candidates = [server] if server != "auto" else [
        f"unix:/run/user/{config['uid']}/pulse/native",
        *[f"unix:{p}" for p in sorted(glob.glob("/run/panther-host-tmp/pulse-*/native"))],
        *[f"unix:{p}" for p in sorted(glob.glob("/tmp/pulse-*/native"))],
    ]
    failures = []
    for candidate in candidates:
        env = dict(os.environ, PULSE_SERVER=candidate)
        try:
            probe = subprocess.run(["pactl", "info"], env=env, timeout=1,
                                   capture_output=True, text=True, **_pulse_user(config))
        except subprocess.TimeoutExpired:
            failures.append(candidate + ": connection timed out")
            continue
        if probe.returncode == 0:
            resolved["server"] = candidate
            return resolved
        failures.append(probe.stderr.strip()[-120:])
    raise RuntimeError("No reachable host PulseAudio server: " + "; ".join(failures))


def _worker(mode, topic, config, commands, replies):
    from common import logsafe
    logsafe.install(check_fd=False)
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("AGENT_CORE_ROS_DOMAIN_ID", "42")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from audio_msgs.msg import AudioChunk

    rclpy.init(domain_id=int(os.environ["ROS_DOMAIN_ID"]))
    node = Node(f"panther_{mode}_audio", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    stop_event = threading.Event()
    pcm_queue = queue.Queue(maxsize=256)  # At most 32 seconds of 4096-byte blocks.
    state = {"running": False, "frames": 0, "dropped": 0, "last_frame": 0.0, "error": "",
             "input_topic": "", "volume": config.get("volume", 100)}
    process = None
    thread = None
    subscription = None
    publisher = node.create_publisher(AudioChunk, topic, qos) if mode == "mic" else None

    def capture(proc):
        try:
            pending = bytearray()
            while not stop_event.is_set() and proc.poll() is None:
                raw = proc.stdout.read(CHUNK_BYTES)
                if not raw:
                    break
                first_sample_ns = time.time_ns() - (len(pending) + len(raw)) * 1_000_000_000 // 32000
                for chunk in _complete_chunks(pending, raw):
                    msg = AudioChunk()
                    msg.format = FORMAT
                    msg.data = list(chunk)
                    msg.header.stamp.sec = first_sample_ns // 1_000_000_000
                    msg.header.stamp.nanosec = first_sample_ns % 1_000_000_000
                    publisher.publish(msg)
                    state["frames"] += 1
                    state["last_frame"] = time.monotonic()
                    first_sample_ns += CHUNK_BYTES * 1_000_000_000 // 32000
        except Exception as exc:
            state["error"] = f"capture failed: {exc}"
        finally:
            if not stop_event.is_set():
                state["running"] = False
                state["error"] = state["error"] or "capture process stopped"

    def play(proc):
        try:
            deadline = time.monotonic()
            while not stop_event.is_set() and proc.poll() is None:
                try:
                    raw = pcm_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                raw = _speaker_pcm(raw, state["volume"])
                deadline = max(deadline, time.monotonic()) + len(raw) / 32000
                # Unbuffered pipe writes may be partial, especially for large frames.
                remaining = memoryview(raw)
                while remaining and not stop_event.is_set():
                    written = proc.stdin.write(remaining)
                    if not written:
                        raise RuntimeError("speaker pipe closed")
                    remaining = remaining[written:]
                proc.stdin.flush()
                stop_event.wait(max(0.0, deadline - time.monotonic() - 0.08))
        except Exception as exc:
            state["error"] = f"playback failed: {exc}"
        finally:
            if not stop_event.is_set():
                state["running"] = False
                state["error"] = state["error"] or "playback process stopped"

    def on_audio(msg):
        if not state["running"] or msg.format != FORMAT:
            return
        raw = bytes(msg.data)
        if not raw or len(raw) % 2:
            return
        state["frames"] += 1
        state["last_frame"] = time.monotonic()
        for offset in range(0, len(raw), 4096):
            block = raw[offset:offset + 4096]
            try:
                pcm_queue.put_nowait(block)
            except queue.Full:
                state["dropped"] += 1
                state["error"] = "speaker buffer full; restart the card before sending more audio"
                state["running"] = False
                stop_event.set()
                break

    def stop():
        nonlocal process, thread, subscription
        state["running"] = False
        stop_event.set()
        if subscription is not None:
            node.destroy_subscription(subscription)
            subscription = None
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
            process = None
        if thread is not None:
            thread.join(timeout=2)
            thread = None
        while not pcm_queue.empty():
            try:
                pcm_queue.get_nowait()
            except queue.Empty:
                break
        state["input_topic"] = ""

    def info():
        healthy = state["running"] and process is not None and process.poll() is None
        return {"state": "error" if state["error"] else "running" if healthy else "idle",
                "pulse_server": os.environ.get("PULSE_SERVER", config["server"]), "pulse_device": config["device"],
                "frames": state["frames"], "dropped": state["dropped"], "last_frame_age_ms":
                int((time.monotonic() - state["last_frame"]) * 1000) if state["last_frame"] else None,
                "input_topic": state["input_topic"], "queued_chunks": pcm_queue.qsize(),
                "error": state["error"],
                **({"volume": state["volume"], "volume_scope": "stream"} if mode == "speaker" else {})}

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                action, arguments = commands.get_nowait()
            except queue.Empty:
                continue
            if action == "close":
                break
            if action == "stop":
                stop()
                replies.put({"state": "idle"})
            elif action == "info":
                replies.put(info())
            elif action in {"get_volume", "set_volume"} and mode == "speaker":
                if action == "set_volume":
                    state["volume"] = _validate_volume(arguments.get("volume"))
                replies.put(info())
            elif action == "start":
                input_topic = str(arguments.get("input_topic", "")).strip()
                if mode == "speaker" and not input_topic:
                    replies.put({"state": "waiting_for_input"})
                    continue
                stop()
                state.update(error="", frames=0, dropped=0, last_frame=0.0, input_topic=input_topic)
                stop_event.clear()
                try:
                    resolved = _resolve_pulse(config)
                    os.environ["PULSE_SERVER"] = resolved["server"]
                    process = subprocess.Popen(
                        _pulse_command(mode, config["device"]),
                        stdin=subprocess.PIPE if mode == "speaker" else subprocess.DEVNULL,
                        stdout=subprocess.PIPE if mode == "mic" else subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, bufsize=0,
                        **_pulse_user(config))
                    time.sleep(0.1)
                    if process.poll() is not None:
                        raise RuntimeError(f"PulseAudio process exited ({process.returncode})")
                    if mode == "speaker":
                        subscription = node.create_subscription(AudioChunk, input_topic, on_audio, qos)
                    state["running"] = True
                    thread = threading.Thread(target=capture if mode == "mic" else play,
                                              args=(process,), daemon=True)
                    thread.start()
                    if mode == "mic":
                        deadline = time.monotonic() + 2.0
                        while state["frames"] == 0 and time.monotonic() < deadline:
                            time.sleep(0.02)
                        if state["frames"] == 0:
                            raise RuntimeError("no microphone PCM received within 2 seconds")
                    replies.put({"state": "running",
                                 "input_topic": input_topic})
                except Exception as exc:
                    stop()
                    state["error"] = str(exc)[:200]
                    replies.put(info())
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()


class AudioPlugin:
    def __init__(self, name: str, namespace: str, config: dict):
        self.name = name
        self.topic = f"/{namespace}/panther/mic/audio" if name == "mic" else ""
        self.config = {"server": os.environ.get("PULSE_SERVER") or config["server"],
                       "device": os.environ.get("PANTHER_PULSE_SOURCE" if name == "mic"
                                                else "PANTHER_PULSE_SINK")
                       or config["source" if name == "mic" else "sink"],
                       "uid": os.environ.get("PANTHER_PULSE_UID", config.get("uid", 1001)),
                       "gid": os.environ.get("PANTHER_PULSE_GID", config.get("gid", 1002))}
        self._ctx = mp.get_context("spawn")
        self._commands = None
        self._replies = None
        self._process = None
        self._rpc_lock = threading.RLock()
        self._volume = 100
        if name == "speaker":
            self._volume = _validate_volume(config.get("speaker_volume", 100))
            self.config["volume"] = self._volume

    def get_tool(self):
        schema = {"type": "object", "properties": {
            "action": {"type": "string", "enum": ["start", "stop", "info"]}},
            "required": ["action"], "additionalProperties": False}
        tool = {"name": self.name, "type": "sensor" if self.name == "mic" else "actuator",
                "multiInstance": False, "description": f"Panther PulseAudio {self.name}",
                "inputSchema": schema}
        if self.name == "mic":
            tool["topic_out"] = [{"topic": self.topic, "format": FORMAT}]
        else:
            schema["properties"]["action"]["enum"].extend(["get_volume", "set_volume"])
            schema["properties"]["input_topic"] = {"type": "string"}
            schema["properties"]["volume"] = {
                "type": "integer", "minimum": 0, "maximum": 100,
                "description": "Playback stream volume (%); 0=mute, 100=original PCM level. Host master volume is independent."}
            schema["x-action-params"] = {"start": {"params": ["input_topic"]},
                                         "stop": {"params": []}, "info": {"params": []},
                                         "get_volume": {"params": []},
                                         "set_volume": {"params": ["volume"]}}
            tool["topic_in"] = [{"format": FORMAT}]
        return tool

    def start(self):
        return None  # Capture/playback is armed by the canvas, not bundle startup.

    def stop(self):
        if self._process is not None:
            self.dispatch("stop", {})
            self._commands.put(("close", {}))
            self._process.join(timeout=3)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2)
            self._process = None

    def dispatch(self, action, args):
        # Each command must consume its own reply, including concurrent MCP calls.
        with self._rpc_lock:
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        allowed = {"start", "stop", "info"}
        if self.name == "speaker":
            allowed.update({"get_volume", "set_volume"})
        if action not in allowed:
            return {"error": "unknown action"}
        requested_volume = None
        if action == "set_volume":
            try:
                requested_volume = _validate_volume(args.get("volume"))
            except ValueError as exc:
                return {"state": "error", "error": str(exc)}
        if action != "start" and (self._process is None or not self._process.is_alive()):
            if requested_volume is not None:
                self._volume = requested_volume
                self.config["volume"] = self._volume
            result = {"state": "idle"}
            if self.name == "mic":
                result["topic_out"] = [{"topic": self.topic, "format": FORMAT}]
            else:
                result.update(volume=self._volume, volume_scope="stream")
            return result
        if self._process is None or not self._process.is_alive():
            self._commands = self._ctx.Queue()
            self._replies = self._ctx.Queue()
            self._process = self._ctx.Process(
                target=_worker, args=(self.name, self.topic, self.config,
                                      self._commands, self._replies), daemon=True)
            self._process.start()
        self._commands.put((action, {"volume": requested_volume} if action == "set_volume"
                            else {"input_topic": str(args.get("input_topic", "")).strip()}))
        try:
            result = self._replies.get(timeout=8)
        except queue.Empty:
            return {"state": "error", "message": "audio worker did not respond"}
        if self.name == "mic":
            result["topic_out"] = [{"topic": self.topic, "format": FORMAT}]
        elif "volume" in result:
            self._volume = result["volume"]
            self.config["volume"] = self._volume
        return result
