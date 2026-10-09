"""As2W microphone, speaker, and front-camera cards.

The hardware probe for this model established three different transports:

* microphone: A2 PCM multicast, 239.168.123.161:5555;
* speaker: A2 ``voice`` RPC service;
* camera: Go2-compatible ``videohub`` RPC service.

The RPC-backed devices run in dedicated spawned processes.  Camera timeouts or
audio pacing must never delay the SportClient process used for locomotion.
"""
import fcntl
import multiprocessing
import queue
import socket
import struct
import threading
import time
from array import array
from uuid import uuid4

from audio_msgs.msg import AudioChunk
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    from camera_specs import jpeg_dimensions
except ModuleNotFoundError as exc:
    if exc.name != "camera_specs":
        raise
    from unitree.as2w.camera_specs import jpeg_dimensions


_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)
_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_MIC_WAKEUP_INSTRUCTION = (
    "请同时按下 L1+L2，将语音状态切换为唤醒模式，"
    "然后重新开启智能控制。"
)


def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def _interface_ipv4(interface):
    """Return the IPv4 address assigned to an interface, without route guessing."""
    request = struct.pack("256s", interface[:15].encode("ascii"))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR
        return socket.inet_ntoa(packed[20:24])
    finally:
        sock.close()


def _pcm_has_variation(pcm: bytes) -> bool:
    """Return whether a PCM-16LE chunk contains more than one sample value."""
    sample_count = len(pcm) // 2
    if sample_count < 2:
        return False
    samples = struct.unpack_from(f"<{sample_count}h", pcm)
    return min(samples) != max(samples)


class _MicNode(Node):
    def __init__(self, topic, interface, group, port, chunk_bytes, startup_grace_s):
        super().__init__("as2w_mic")
        self.topic = topic
        self.interface = interface
        self.group = group
        self.port = port
        self.chunk_bytes = chunk_bytes
        self.startup_grace_s = startup_grace_s
        self.publisher = self.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.state = "idle"
        self.packet_count = 0
        self.varying_chunk_count = 0
        self.last_packet_ts = 0.0
        self.last_error = ""
        self._socket = None
        self._thread = None
        self._stop_event = threading.Event()
        self._started_at = 0.0

    def start_capture(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self.stop_capture()
        self.packet_count = 0
        self.varying_chunk_count = 0
        self.last_packet_ts = 0.0
        self.last_error = ""
        self._started_at = time.monotonic()
        self._stop_event = threading.Event()
        try:
            local_ip = _interface_ipv4(self.interface)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.port))
            membership = struct.pack(
                "4s4s", socket.inet_aton(self.group), socket.inet_aton(local_ip))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            sock.settimeout(0.5)
            self._socket = sock
        except Exception as exc:
            self.state = "error"
            self.last_error = str(exc)
            return
        self.state = "waiting"
        self._thread = threading.Thread(
            target=self._capture_loop,
            args=(self._socket, self._stop_event),
            name="as2w-mic-capture",
            daemon=True,
        )
        self._thread.start()

    def _capture_loop(self, sock, stop_event):
        buffered = bytearray()
        while not stop_event.is_set():
            try:
                data, _source = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self.packet_count += 1
            self.last_packet_ts = time.monotonic()
            buffered.extend(data)
            while len(buffered) >= self.chunk_bytes and not stop_event.is_set():
                chunk = bytes(buffered[:self.chunk_bytes])
                del buffered[:self.chunk_bytes]
                if _pcm_has_variation(chunk):
                    self.varying_chunk_count += 1
                    if self.state != "error":
                        self.state = "running"
                message = AudioChunk()
                if hasattr(message, "header"):
                    message.header.stamp = self.get_clock().now().to_msg()
                message.format = "audio/pcm-16k"
                message.data = list(chunk)
                self.publisher.publish(message)

    def stop_capture(self):
        self._stop_event.set()
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self.state = "idle"

    def status(self):
        now = time.monotonic()
        try:
            rival_publishers = max(
                0, len(self.get_publishers_info_by_topic(self.topic)) - 1)
        except Exception:
            rival_publishers = 0
        reported_state = self.state
        if rival_publishers:
            reported_state = "error"
            message = (
                "Another publisher is using the microphone topic; duplicate audio "
                "streams would corrupt downstream ASR."
            )
        elif self.state == "waiting" and now - self._started_at >= self.startup_grace_s:
            message = (
                "No microphone multicast packets received; enable the Unitree "
                "voice assistant / wake-up conversation mode."
            )
        else:
            message = self.last_error
        return {
            "state": reported_state,
            "packets": self.packet_count,
            "rival_publishers": rival_publishers,
            "last_packet_ago_ms": (
                int((now - self.last_packet_ts) * 1000) if self.last_packet_ts else -1),
            "message": message,
        }


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, executor, network_iface="eth0"):
        self._topic = "/{}/mic/audio".format(namespace)
        self._node = _MicNode(
            self._topic,
            network_iface,
            config.get("group", "239.168.123.161"),
            int(config.get("port", 5555)),
            int(config.get("chunk_bytes", 1024)),
            float(config.get("startup_grace_s", 3.0)),
        )
        executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "description": "As2W microphone — PCM 16 kHz/16-bit/mono multicast audio.",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        self._node.start_capture()

    def stop(self):
        self._node.stop_capture()

    def _self_check(self):
        capture_thread = getattr(self._node, "_thread", None)
        if (self._node.state == "error"
                and (capture_thread is None or not capture_thread.is_alive())):
            return "error", self._node.last_error

        if self._node.packet_count == 0:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and self._node.packet_count == 0:
                time.sleep(0.1)
        if self._node.packet_count == 0:
            self._node.state = "error"
            self._node.last_error = (
                "未收到麦克风组播数据。" + _MIC_WAKEUP_INSTRUCTION
            )
            return "error", self._node.last_error

        # The robot can keep sending flat PCM while voice wake-up mode is off.
        # Require a new chunk with actual sample variation before start succeeds.
        varying_before = self._node.varying_chunk_count
        deadline = time.monotonic() + 3.0
        while (
            time.monotonic() < deadline
            and self._node.varying_chunk_count == varying_before
        ):
            time.sleep(0.1)

        if self._node.varying_chunk_count == varying_before:
            self._node.state = "error"
            self._node.last_error = (
                "收到静音数据。" + _MIC_WAKEUP_INSTRUCTION
            )
            return "error", self._node.last_error

        self._node.state = "running"
        self._node.last_error = ""
        return "running", ""

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            result = self._node.status()
            if result["rival_publishers"] == 0:
                state, message = self._self_check()
                result = self._node.status()
                if state == "error":
                    result["state"] = state
                    result["message"] = message
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            if action != "start":
                result = self._node.status()
            result["topic_out"] = [{"topic": self._topic, "format": "audio/pcm-16k"}]
            return result
        return None


_AUDIO_EOF_MAGIC = b"\x01\x00\xff\xff\x01\x00\xff\xff"
_SPEAKER_APP_NAME = "as2w_speaker"
_SPEAKER_BYTES_PER_SECOND = 32000.0
_SPEAKER_EMPTY_POLL_S = 0.1
_SPEAKER_FLUSH_AFTER_IDLE = 5
_SPEAKER_PREFILL_FALLBACK_IDLE = 10
_SPEAKER_EXIT_AFTER_IDLE = 15


def _next_speaker_deadline(deadline, started_at, finished_at, duration):
    """Advance a bounded audio timeline, re-anchoring after a slow RPC."""
    deadline = (started_at if deadline is None else deadline) + duration
    if deadline < finished_at:
        deadline = finished_at
    return deadline


def _speaker_worker(
        control_queue, result_queue, pcm_queue, interface,
        block_bytes, startup_prefill_bytes, rebuffer_prefill_bytes, max_lead_s):
    _install_logsafe()
    try:
        from unitree_sdk2py.a2.audio.audio_client import AudioClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface)
        client = AudioClient()
        client.SetTimeout(5.0)
        client.Init()
        client.PlayStop(_SPEAKER_APP_NAME)
        result_queue.put({"id": "ready", "ok": True})
    except Exception as exc:
        result_queue.put({"id": "ready", "ok": False, "error": str(exc)})
        return

    buffered = bytearray()
    active = True
    paused = False
    muted_until_eof = False
    stream_id = "as2w_{}".format(uuid4().hex)
    deadline = None
    draining = False
    prefill_target = startup_prefill_bytes
    prefill_mode = "startup"
    continuation_pending = False
    rebuffering = False
    idle_polls = 0
    play_calls = 0
    play_errors = 0
    attempted_bytes = 0
    played_bytes = 0
    partial_flushes = 0
    prefill_fallbacks = 0
    underflows = 0
    rebuffer_count = 0
    continuation_resumes = 0
    underflow_active = False
    eof_count = 0
    rpc_total_ms = 0.0
    rpc_max_ms = 0.0
    last_play_error = ""
    first_input_at = None
    first_input_bytes = 0
    first_input_to_play_ms = -1.0
    first_play_rpc_ms = -1.0
    rebuffer_started_at = None
    last_rebuffer_wait_ms = -1.0

    def respond(request_id, **payload):
        payload["id"] = request_id
        result_queue.put(payload)

    def play(block, current_deadline):
        nonlocal play_calls, play_errors, attempted_bytes, played_bytes
        nonlocal rpc_total_ms, rpc_max_ms, last_play_error
        nonlocal first_input_to_play_ms, first_play_rpc_ms
        if not block or not active or paused:
            return current_deadline
        started_at = time.monotonic()
        first_play = first_input_at is not None and first_input_to_play_ms < 0
        if first_play:
            first_input_to_play_ms = (started_at - first_input_at) * 1000.0
        try:
            result = client.PlayStream(
                _SPEAKER_APP_NAME, stream_id, bytes(block))
            code = result[0] if isinstance(result, tuple) else result
            if code != 0:
                play_errors += 1
                last_play_error = "PlayStream returned {}".format(code)
            else:
                played_bytes += len(block)
        except Exception as exc:
            play_errors += 1
            last_play_error = "{}: {}".format(type(exc).__name__, exc)
        finished_at = time.monotonic()
        rpc_ms = (finished_at - started_at) * 1000.0
        play_calls += 1
        attempted_bytes += len(block)
        rpc_total_ms += rpc_ms
        rpc_max_ms = max(rpc_max_ms, rpc_ms)
        if first_play:
            first_play_rpc_ms = rpc_ms
        duration = len(block) / _SPEAKER_BYTES_PER_SECOND
        current_deadline = _next_speaker_deadline(
            current_deadline, started_at, finished_at, duration)
        wait = current_deadline - time.monotonic() - max_lead_s
        if wait > 0:
            time.sleep(wait)
        return current_deadline

    running = True
    while running:
        while True:
            try:
                command = control_queue.get_nowait()
            except queue.Empty:
                break
            request_id, operation, value = command
            try:
                if operation == "close":
                    client.PlayStop(_SPEAKER_APP_NAME)
                    respond(request_id, ok=True)
                    running = False
                    break
                if operation in ("reset", "interrupt", "stop"):
                    first_input_at = None
                    first_input_bytes = 0
                    first_input_to_play_ms = -1.0
                    first_play_rpc_ms = -1.0
                    rebuffer_started_at = None
                    last_rebuffer_wait_ms = -1.0
                    buffered.clear()
                    while True:
                        try:
                            pcm_queue.get_nowait()
                        except queue.Empty:
                            break
                    client.PlayStop(_SPEAKER_APP_NAME)
                    deadline = None
                    draining = False
                    prefill_target = startup_prefill_bytes
                    prefill_mode = "startup"
                    continuation_pending = False
                    rebuffering = False
                    idle_polls = 0
                    underflow_active = False
                    paused = False
                    active = operation != "stop"
                    muted_until_eof = operation == "interrupt"
                    respond(request_id, ok=True)
                elif operation == "pause":
                    paused = True
                    client.PlayStop(_SPEAKER_APP_NAME)
                    respond(request_id, ok=True)
                elif operation == "resume":
                    active = True
                    paused = False
                    deadline = None
                    respond(request_id, ok=True)
                elif operation == "get_volume":
                    respond(request_id, ok=True, result=client.GetVolume())
                elif operation == "set_volume":
                    respond(request_id, ok=True, result=client.SetVolume(int(value)))
                elif operation == "status":
                    respond(
                        request_id,
                        ok=True,
                        buffered_bytes=len(buffered),
                        draining=draining,
                        prefill_waiting=bool(buffered and not draining),
                        playback_lead_ms=(
                            max(0.0, (deadline - time.monotonic()) * 1000.0)
                            if deadline is not None else 0.0),
                        play_calls=play_calls,
                        play_errors=play_errors,
                        attempted_bytes=attempted_bytes,
                        played_bytes=played_bytes,
                        partial_flushes=partial_flushes,
                        prefill_fallbacks=prefill_fallbacks,
                        underflows=underflows,
                        rebuffer_count=rebuffer_count,
                        continuation_resumes=continuation_resumes,
                        eof_count=eof_count,
                        block_bytes=block_bytes,
                        block_ms=block_bytes / 32.0,
                        # Keep prefill_bytes as a compatibility alias for older
                        # diagnostics consumers.
                        prefill_bytes=startup_prefill_bytes,
                        startup_prefill_bytes=startup_prefill_bytes,
                        startup_prefill_ms=startup_prefill_bytes / 32.0,
                        rebuffer_prefill_bytes=rebuffer_prefill_bytes,
                        rebuffer_prefill_ms=rebuffer_prefill_bytes / 32.0,
                        prefill_target_bytes=prefill_target,
                        prefill_mode=prefill_mode,
                        first_input_bytes=first_input_bytes,
                        first_input_to_play_ms=first_input_to_play_ms,
                        first_play_rpc_ms=first_play_rpc_ms,
                        last_rebuffer_wait_ms=last_rebuffer_wait_ms,
                        max_lead_ms=max_lead_s * 1000.0,
                        rpc_avg_ms=(rpc_total_ms / play_calls if play_calls else 0.0),
                        rpc_max_ms=rpc_max_ms,
                        last_play_error=last_play_error,
                    )
                else:
                    respond(request_id, ok=False, error="unsupported operation")
            except Exception as exc:
                respond(request_id, ok=False, error=str(exc))
        if not running:
            break
        if paused:
            time.sleep(0.05)
            continue
        try:
            pcm = pcm_queue.get(timeout=_SPEAKER_EMPTY_POLL_S)
        except queue.Empty:
            idle_polls += 1
            if (draining and buffered and active and not paused
                    and idle_polls >= _SPEAKER_FLUSH_AFTER_IDLE):
                partial_flushes += 1
                deadline = play(buffered, deadline)
                buffered.clear()
                idle_polls = 0
            elif (not draining and buffered and active and not paused
                  and idle_polls >= _SPEAKER_PREFILL_FALLBACK_IDLE):
                # Sources are expected to send EOF.  This bounded fallback
                # preserves compatibility with short/non-EOF sources without
                # letting a 200-400ms synthesis stall bypass the jitter buffer.
                draining = True
                prefill_fallbacks += 1
                prefill_mode = "streaming"
                deadline = play(buffered, deadline)
                buffered.clear()
                idle_polls = 0
            elif (draining and not buffered and deadline is not None
                  and time.monotonic() >= deadline and not underflow_active):
                underflows += 1
                underflow_active = True
                # Do not resume a starved TTS stream one block at a time.  Go
                # back to the full jitter prefill so a delayed scheduler tick
                # cannot turn into repeated audible gaps.
                draining = False
                deadline = None
                prefill_target = rebuffer_prefill_bytes
                prefill_mode = "rebuffer"
                rebuffer_started_at = time.monotonic()
                continuation_pending = False
                rebuffering = True
                idle_polls = 0
            elif (draining and not buffered
                  and idle_polls >= _SPEAKER_EXIT_AFTER_IDLE):
                draining = False
                deadline = None
                underflow_active = False
                idle_polls = 0
            continue
        idle_polls = 0
        if pcm == _AUDIO_EOF_MAGIC:
            eof_count += 1
            if muted_until_eof:
                muted_until_eof = False
                buffered.clear()
                draining = False
                deadline = None
                prefill_target = startup_prefill_bytes
                prefill_mode = "startup"
                continuation_pending = False
                rebuffering = False
                continue
            if buffered:
                draining = True
                deadline = play(buffered, deadline)
                buffered.clear()
            # TTS emits EOF for every internally split text segment.  Keep the
            # drain timeline alive so the next segment does not pay another
            # fixed prefill delay.  If the playback deadline actually expires
            # before more PCM arrives, the normal underflow path below switches
            # back to the full jitter prefill.
            draining = deadline is not None
            prefill_target = startup_prefill_bytes
            prefill_mode = "streaming"
            continuation_pending = True
            rebuffering = False
            underflow_active = False
            continue
        if not active or muted_until_eof:
            continue
        if first_input_at is None:
            first_input_at = time.monotonic()
            first_input_bytes = len(pcm)
        buffered.extend(pcm)
        if draining and continuation_pending:
            continuation_resumes += 1
            continuation_pending = False
        if not draining and len(buffered) >= prefill_target:
            draining = True
            if rebuffering:
                rebuffer_count += 1
                if rebuffer_started_at is not None:
                    last_rebuffer_wait_ms = (
                        time.monotonic() - rebuffer_started_at) * 1000.0
            rebuffering = False
            rebuffer_started_at = None
            continuation_pending = False
            underflow_active = False
            prefill_target = startup_prefill_bytes
            prefill_mode = "streaming"
        while draining and len(buffered) >= block_bytes:
            block = bytes(buffered[:block_bytes])
            del buffered[:block_bytes]
            deadline = play(block, deadline)


def _put_speaker_pcm(pcm_queue, pcm, stats, stats_lock):
    """Enqueue PCM inside the speaker process and maintain input diagnostics."""
    now = time.monotonic()
    with stats_lock:
        stats["received_chunks"] += 1
        stats["received_bytes"] += len(pcm)
        if pcm == _AUDIO_EOF_MAGIC:
            stats["last_input_ts"] = 0.0
        else:
            if stats["last_input_ts"]:
                stats["max_input_gap_ms"] = max(
                    stats["max_input_gap_ms"],
                    (now - stats["last_input_ts"]) * 1000.0,
                )
            stats["last_input_ts"] = now
    try:
        pcm_queue.put_nowait(pcm)
    except queue.Full:
        with stats_lock:
            stats["queue_drops"] += 1
        try:
            pcm_queue.get_nowait()
            pcm_queue.put_nowait(pcm)
        except (queue.Empty, queue.Full):
            pass


def _speaker_process(
        control_queue, result_queue, interface, block_bytes,
        startup_prefill_bytes, rebuffer_prefill_bytes, max_lead_s):
    """Own the ROS subscription and A2 playback path in one child process.

    PCM remains in this process: the ROS callback feeds a thread-local queue and
    ``_speaker_worker`` is the sole owner of AudioClient.  Only small lifecycle
    and status dictionaries cross the multiprocessing boundary.
    """
    _install_logsafe()
    node = None
    executor = None
    spin_thread = None
    subscription = None
    playback_thread = None
    rclpy_started = False
    playback_control = queue.Queue()
    playback_results = queue.Queue()
    pcm_queue = queue.Queue(maxsize=64)
    stats_lock = threading.Lock()
    stats = {
        "received_chunks": 0,
        "received_bytes": 0,
        "queue_drops": 0,
        "last_input_ts": 0.0,
        "max_input_gap_ms": 0.0,
    }
    topic = ""
    state = "idle"

    def respond(request_id, **payload):
        payload["id"] = request_id
        result_queue.put(payload)

    def playback_call(operation, value=None, timeout=6.0):
        request_id = uuid4().hex
        playback_control.put((request_id, operation, value))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                result = playback_results.get(
                    timeout=max(0.01, deadline - time.monotonic()))
            except queue.Empty:
                break
            if result.get("id") == request_id:
                return result
        return {"ok": False, "error": "speaker playback operation timed out"}

    def on_audio(message):
        nonlocal state
        _put_speaker_pcm(pcm_queue, bytes(message.data), stats, stats_lock)
        if state == "ready":
            state = "playing"

    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        rclpy.init(args=None)
        rclpy_started = True
        node = Node("as2w_speaker")
        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(node)
        spin_thread = threading.Thread(
            target=executor.spin, name="as2w-speaker-ros", daemon=True)
        spin_thread.start()

        playback_thread = threading.Thread(
            target=_speaker_worker,
            args=(playback_control, playback_results, pcm_queue, interface,
                  block_bytes, startup_prefill_bytes, rebuffer_prefill_bytes,
                  max_lead_s),
            name="as2w-speaker-playback",
            daemon=True,
        )
        playback_thread.start()
        try:
            ready = playback_results.get(timeout=8.0)
        except queue.Empty:
            ready = {"ok": False, "error": "speaker playback startup timed out"}
        if not ready.get("ok"):
            raise RuntimeError(ready.get("error", "speaker playback unavailable"))
        state = "ready"
        result_queue.put({"id": "ready", "ok": True})

        running = True
        while running:
            try:
                request_id, operation, value = control_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if operation == "close":
                    if subscription is not None:
                        node.destroy_subscription(subscription)
                        subscription = None
                    playback_call("close", timeout=3.0)
                    respond(request_id, ok=True)
                    running = False
                elif operation == "start":
                    requested_topic = str(value or "")
                    if not requested_topic:
                        respond(request_id, ok=False, error="input_topic is required")
                        continue
                    if subscription is not None and topic != requested_topic:
                        node.destroy_subscription(subscription)
                        subscription = None
                    if subscription is None:
                        subscription = node.create_subscription(
                            AudioChunk, requested_topic, on_audio, _LOW_LAT_QOS)
                    topic = requested_topic
                    result = playback_call("reset")
                    state = "ready" if result.get("ok") else "error"
                    respond(request_id, **result)
                elif operation == "stop":
                    if subscription is not None:
                        node.destroy_subscription(subscription)
                        subscription = None
                    result = playback_call("stop")
                    topic = ""
                    state = "idle" if result.get("ok") else "error"
                    respond(request_id, **result)
                elif operation in ("interrupt", "pause", "resume",
                                   "get_volume", "set_volume"):
                    result = playback_call(operation, value)
                    if result.get("ok") and operation in ("interrupt", "pause", "resume"):
                        state = {
                            "interrupt": "ready",
                            "pause": "paused",
                            "resume": "playing",
                        }[operation]
                    respond(request_id, **result)
                elif operation == "status":
                    result = playback_call("status", timeout=2.0)
                    result.pop("id", None)
                    with stats_lock:
                        result.update({
                            "received_chunks": stats["received_chunks"],
                            "received_bytes": stats["received_bytes"],
                            "queue_drops": stats["queue_drops"],
                            "max_input_gap_ms": stats["max_input_gap_ms"],
                            "last_input_ago_ms": (
                                (time.monotonic() - stats["last_input_ts"]) * 1000.0
                                if stats["last_input_ts"] else -1
                            ),
                        })
                    result.update({"state": state, "topic": topic})
                    descriptor = {"format": "audio/pcm-16k"}
                    if topic:
                        descriptor["topic"] = topic
                    result["topic_in"] = [descriptor]
                    respond(request_id, **result)
                else:
                    respond(request_id, ok=False, error="unsupported operation")
            except Exception as exc:
                state = "error"
                respond(request_id, ok=False, error=str(exc))
    except Exception as exc:
        result_queue.put({"id": "ready", "ok": False, "error": str(exc)})
    finally:
        if playback_thread is not None and playback_thread.is_alive():
            playback_call("close", timeout=1.0)
            playback_thread.join(timeout=1.0)
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy_started:
            try:
                rclpy.shutdown()
            except Exception:
                pass


class _SpeakerBackend:
    def __init__(self, interface, block_bytes, startup_prefill_bytes,
                 rebuffer_prefill_bytes, max_lead_s):
        context = multiprocessing.get_context("spawn")
        self._control = context.Queue()
        self._results = context.Queue()
        self._lock = threading.Lock()
        self._process = context.Process(
            target=_speaker_process,
            args=(self._control, self._results, interface,
                  block_bytes, startup_prefill_bytes, rebuffer_prefill_bytes,
                  max_lead_s),
            name="as2w_speaker",
            daemon=True,
        )
        self._process.start()
        try:
            ready = self._results.get(timeout=8.0)
        except queue.Empty:
            ready = {"ok": False, "error": "speaker worker startup timed out"}
        self.error = "" if ready.get("ok") else ready.get("error", "speaker unavailable")

    def is_available(self):
        return not self.error and self._process.is_alive()

    def status(self):
        worker = self.call("status", timeout=2.0)
        worker.pop("id", None)
        return worker

    def call(self, operation, value=None, timeout=6.0):
        if self.error:
            return {"ok": False, "error": self.error}
        if not self._process.is_alive():
            return {"ok": False, "error": "speaker worker is not running"}
        with self._lock:
            request_id = uuid4().hex
            self._control.put((request_id, operation, value))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    result = self._results.get(timeout=max(0.01, deadline - time.monotonic()))
                except queue.Empty:
                    break
                if result.get("id") == request_id:
                    return result
            return {"ok": False, "error": "speaker operation timed out"}

    def close(self):
        if self._process.is_alive():
            self.call("close", timeout=3.0)
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)


class _SpeakerNode:
    def __init__(self, interface, block_bytes, startup_prefill_bytes,
                 rebuffer_prefill_bytes, max_lead_s):
        self._interface = interface
        self._block_bytes = block_bytes
        self._startup_prefill_bytes = startup_prefill_bytes
        self._rebuffer_prefill_bytes = rebuffer_prefill_bytes
        self._max_lead_s = max_lead_s
        self._backend = None
        self.topic = ""
        self.state = "idle"

    def start_backend(self):
        if self._backend is not None and self._backend.is_available():
            if self.state in ("idle", "error"):
                self.state = "ready"
            return {"ok": True}
        if self._backend is not None:
            self._backend.close()
        self._backend = _SpeakerBackend(
            self._interface,
            self._block_bytes,
            self._startup_prefill_bytes,
            self._rebuffer_prefill_bytes,
            self._max_lead_s,
        )
        if self._backend.error:
            self.state = "error"
            return {"ok": False, "error": self._backend.error}
        self.state = "ready"
        return {"ok": True}

    def call_backend(self, operation, value=None):
        if self._backend is None or not self._backend.is_available():
            return {
                "ok": False,
                "code": "PRECONDITION_FAILED",
                "error": "speaker backend is not running; start the card first",
            }
        return self._backend.call(operation, value)

    def start_play(self, topic):
        if not topic:
            return {"ok": False, "error": "input_topic is required"}
        ready = self.start_backend()
        if not ready.get("ok"):
            return ready
        result = self.call_backend("start", topic)
        if result.get("ok"):
            self.topic = topic
        self.state = "ready" if result.get("ok") else "error"
        return result

    def stop_play(self):
        result = self.call_backend("stop")
        self.topic = ""
        self.state = "idle" if result.get("ok") else "error"
        return result

    def close(self):
        if self._backend is not None:
            self._backend.close()
            self._backend = None
        self.topic = ""
        self.state = "idle"


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config, namespace, executor, network_iface="eth0"):
        block_ms = max(
            100, min(1000, int(config.get("block_ms", config.get("buffer_ms", 300)))))
        startup_prefill_ms = max(
            block_ms,
            min(3000, int(config.get(
                "startup_prefill_ms", config.get("prefill_ms", 300)))),
        )
        rebuffer_prefill_ms = max(
            block_ms,
            min(3000, int(config.get(
                "rebuffer_prefill_ms", config.get("prefill_ms", 500)))),
        )
        max_lead_ms = max(0, min(1000, int(config.get("max_lead_ms", 240))))
        self._node = _SpeakerNode(
            network_iface,
            block_ms * 32,
            startup_prefill_ms * 32,
            rebuffer_prefill_ms * 32,
            max_lead_ms / 1000.0,
        )

    def get_tool(self):
        actions = ["start", "stop", "interrupt", "pause", "resume", "get_volume", "set_volume", "info"]
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "As2W speaker — streams PCM-16k audio through the A2 voice service.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                    "input_topic": {"type": "string", "description": "ROS2 PCM audio topic."},
                    "volume": {"type": "integer", "minimum": 0, "maximum": 100},
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["input_topic"],
                        "description": (
                            "Subscribe to the wired PCM topic. Internal lifecycle "
                            "start without a topic only initializes the backend."
                        ),
                    },
                    "stop": {"params": []},
                    "interrupt": {"params": []},
                    "pause": {"params": []},
                    "resume": {"params": []},
                    "get_volume": {"params": []},
                    "set_volume": {"params": ["volume"]},
                    "info": {"params": []},
                },
            },
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self):
        return self._node.start_backend()

    def stop(self):
        self._node.close()

    def dispatch(self, action, args):
        reported_topic = self._node.topic
        if action == "start":
            input_topic = args.get("input_topic", "")
            result = (
                self._node.start_play(input_topic)
                if input_topic else self.start()
            )
        elif action == "play":
            result = self._node.start_play(args.get("input_topic", ""))
        elif action == "stop":
            self.stop()
            result = {"ok": True}
        elif action in ("interrupt", "pause", "resume"):
            result = self._node.call_backend(action)
            if result.get("ok"):
                self._node.state = {"interrupt": "ready", "pause": "paused", "resume": "playing"}[action]
        elif action == "get_volume":
            result = self._node.call_backend("get_volume")
        elif action == "set_volume":
            volume = max(0, min(100, int(args.get("volume", 50))))
            result = self._node.call_backend("set_volume", volume)
            result["volume"] = volume
        elif action == "info":
            backend = self._node._backend
            if backend is not None and backend.is_available():
                result = backend.status()
            else:
                result = {"ok": False, "state": self._node.state}
            input_topic = args.get("input_topic") or self._node.topic
            reported_topic = input_topic
            descriptor = {"format": "audio/pcm-16k"}
            if input_topic:
                descriptor["topic"] = input_topic
            result["topic_in"] = [descriptor]
        else:
            return None
        if action != "info":
            reported_topic = self._node.topic
        if action == "info" and result.get("state"):
            self._node.state = result["state"]
        result.update({"state": self._node.state, "topic": reported_topic})
        return result


def _put_latest(target_queue, item):
    try:
        target_queue.put_nowait(item)
        return False
    except queue.Full:
        pass
    try:
        target_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        target_queue.put_nowait(item)
    except queue.Full:
        return True
    return True


def _camera_worker(
        frame_queue, status_queue, stop_event, interface, fps, timeout_s,
        retry_s, metrics=None, metrics_lock=None):
    _install_logsafe()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient
        ChannelFactoryInitialize(0, interface)
        client = VideoClient()
        client.SetTimeout(timeout_s)
        client.Init()
        _put_latest(status_queue, ("ready", ""))
    except Exception as exc:
        _put_latest(status_queue, ("error", str(exc)))
        return

    period = 1.0 / max(1.0, fps)
    deadline = time.monotonic()
    failures = 0
    while not stop_event.is_set():
        try:
            rpc_started = time.monotonic()
            code, data = client.GetImageSample()
            rpc_ms = (time.monotonic() - rpc_started) * 1000.0
            convert_started = time.monotonic()
            frame = bytes(data or [])
            convert_ms = (time.monotonic() - convert_started) * 1000.0
            if code != 0:
                raise RuntimeError("videohub returned {}".format(code))
            if not (frame.startswith(b"\xff\xd8") and frame.endswith(b"\xff\xd9")):
                raise RuntimeError("videohub returned an invalid JPEG")
            failures = 0
            if metrics is not None and metrics_lock is not None:
                with metrics_lock:
                    metrics["capture_frames"] += 1
                    metrics["captured_bytes"] += len(frame)
                    metrics["rpc_total_ms"] += rpc_ms
                    metrics["rpc_max_ms"] = max(metrics["rpc_max_ms"], rpc_ms)
                    metrics["convert_total_ms"] += convert_ms
                    metrics["convert_max_ms"] = max(
                        metrics["convert_max_ms"], convert_ms)
            dropped = _put_latest(frame_queue, frame)
            if dropped and metrics is not None and metrics_lock is not None:
                with metrics_lock:
                    metrics["queue_drops"] += 1
            _put_latest(status_queue, ("running", ""))
        except Exception as exc:
            failures += 1
            _put_latest(status_queue, ("reconnecting", str(exc)))
            if failures >= 3:
                stop_event.wait(retry_s)
                failures = 0
        deadline += period
        wait = deadline - time.monotonic()
        if wait > 0:
            stop_event.wait(wait)
        else:
            deadline = time.monotonic()


def _camera_process(
        control_queue, result_queue, topic, interface, fps, timeout_s, retry_s):
    """Own videohub capture and ROS publication in one child process."""
    _install_logsafe()
    node = None
    publisher = None
    rclpy_started = False
    capture_thread = None
    publish_thread = None
    capture_stop = None
    frame_queue = None
    status_queue = None
    metrics_lock = threading.Lock()
    metrics = {
        "state": "idle",
        "frames": 0,
        "capture_frames": 0,
        "captured_bytes": 0,
        "queue_drops": 0,
        "last_frame_ts": 0.0,
        "frame_width": None,
        "frame_height": None,
        "last_error": "",
        "capture_started_ts": 0.0,
        "publish_started_ts": 0.0,
        "rpc_total_ms": 0.0,
        "rpc_max_ms": 0.0,
        "convert_total_ms": 0.0,
        "convert_max_ms": 0.0,
        "build_total_ms": 0.0,
        "build_max_ms": 0.0,
        "publish_total_ms": 0.0,
        "publish_max_ms": 0.0,
    }

    def respond(request_id, **payload):
        payload["id"] = request_id
        result_queue.put(payload)

    def camera_status():
        with metrics_lock:
            now = time.monotonic()
            captures = metrics["capture_frames"]
            published = metrics["frames"]
            capture_elapsed = (
                now - metrics["capture_started_ts"]
                if metrics["capture_started_ts"] else 0.0)
            publish_elapsed = (
                now - metrics["publish_started_ts"]
                if metrics["publish_started_ts"] else 0.0)
            return {
                "ok": metrics["state"] != "error",
                "state": metrics["state"],
                "frames": published,
                "frame_width": metrics["frame_width"],
                "frame_height": metrics["frame_height"],
                "capture_frames": captures,
                "capture_fps": captures / capture_elapsed if capture_elapsed else 0.0,
                "publish_fps": published / publish_elapsed if publish_elapsed else 0.0,
                "last_frame_ago_ms": (
                    int((now - metrics["last_frame_ts"]) * 1000)
                    if metrics["last_frame_ts"] else -1),
                "last_error": metrics["last_error"],
                "queue_drops": metrics["queue_drops"],
                "frame_bytes_avg": (
                    metrics["captured_bytes"] / captures if captures else 0.0),
                "rpc_avg_ms": (
                    metrics["rpc_total_ms"] / captures if captures else 0.0),
                "rpc_max_ms": metrics["rpc_max_ms"],
                "bytes_convert_avg_ms": (
                    metrics["convert_total_ms"] / captures if captures else 0.0),
                "bytes_convert_max_ms": metrics["convert_max_ms"],
                "message_build_avg_ms": (
                    metrics["build_total_ms"] / published if published else 0.0),
                "message_build_max_ms": metrics["build_max_ms"],
                "publish_call_avg_ms": (
                    metrics["publish_total_ms"] / published if published else 0.0),
                "publish_call_max_ms": metrics["publish_max_ms"],
            }

    def publish_loop():
        while capture_stop is not None and not capture_stop.is_set():
            try:
                while True:
                    camera_state, error = status_queue.get_nowait()
                    with metrics_lock:
                        if camera_state == "error":
                            metrics["state"] = "error"
                        elif camera_state == "reconnecting":
                            metrics["state"] = "reconnecting"
                        elif metrics["frames"] == 0:
                            metrics["state"] = "starting"
                        metrics["last_error"] = error
            except queue.Empty:
                pass
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # If publication falls behind, publish the newest frame instead of
            # increasing end-to-end latency with stale queued images.
            while True:
                try:
                    frame = frame_queue.get_nowait()
                    with metrics_lock:
                        metrics["queue_drops"] += 1
                except queue.Empty:
                    break

            build_started = time.monotonic()
            message = CompressedImage()
            message.header.stamp = node.get_clock().now().to_msg()
            message.format = "jpeg"
            # rosidl's bytes-to-uint8[] path converts element by element.  The
            # buffer-compatible array assignment is about three orders faster
            # for the approximately 300 KB frames returned by videohub.
            message.data = array("B", frame)
            dimensions = jpeg_dimensions(frame)
            build_ms = (time.monotonic() - build_started) * 1000.0
            publish_started = time.monotonic()
            publisher.publish(message)
            publish_ms = (time.monotonic() - publish_started) * 1000.0
            finished_at = time.monotonic()
            with metrics_lock:
                metrics["frames"] += 1
                if not metrics["publish_started_ts"]:
                    metrics["publish_started_ts"] = finished_at
                metrics["last_frame_ts"] = finished_at
                # These dimensions describe the published JPEG. They do not
                # establish intrinsics, distortion, or field of view.
                metrics["frame_width"] = dimensions[0] if dimensions else None
                metrics["frame_height"] = dimensions[1] if dimensions else None
                metrics["state"] = "running"
                metrics["last_error"] = ""
                metrics["build_total_ms"] += build_ms
                metrics["build_max_ms"] = max(metrics["build_max_ms"], build_ms)
                metrics["publish_total_ms"] += publish_ms
                metrics["publish_max_ms"] = max(
                    metrics["publish_max_ms"], publish_ms)

    def stop_threads():
        nonlocal capture_thread, publish_thread, capture_stop
        if capture_stop is not None:
            capture_stop.set()
        for worker in (capture_thread, publish_thread):
            if worker is not None and worker.is_alive():
                worker.join(timeout=max(3.0, timeout_s + 1.0))
        capture_thread = None
        publish_thread = None
        capture_stop = None
        with metrics_lock:
            metrics["state"] = "idle"

    try:
        import rclpy

        rclpy.init(args=None)
        rclpy_started = True
        node = Node("as2w_camera")
        publisher = node.create_publisher(CompressedImage, topic, _IMAGE_QOS)
        result_queue.put({"id": "ready", "ok": True})
        running = True
        while running:
            try:
                request_id, operation, _value = control_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if operation == "start":
                    stop_threads()
                    frame_queue = queue.Queue(maxsize=1)
                    status_queue = queue.Queue(maxsize=4)
                    capture_stop = threading.Event()
                    with metrics_lock:
                        for key in metrics:
                            metrics[key] = "" if key == "last_error" else 0.0
                        metrics["state"] = "starting"
                        metrics["frames"] = 0
                        metrics["capture_frames"] = 0
                        metrics["frame_width"] = None
                        metrics["frame_height"] = None
                        metrics["queue_drops"] = 0
                        metrics["capture_started_ts"] = time.monotonic()
                        metrics["publish_started_ts"] = metrics["capture_started_ts"]
                    publish_thread = threading.Thread(
                        target=publish_loop,
                        name="as2w-camera-publish",
                        daemon=True,
                    )
                    capture_thread = threading.Thread(
                        target=_camera_worker,
                        args=(frame_queue, status_queue, capture_stop, interface,
                              fps, timeout_s, retry_s, metrics, metrics_lock),
                        name="as2w-camera-capture",
                        daemon=True,
                    )
                    publish_thread.start()
                    capture_thread.start()
                    respond(request_id, ok=True, state="starting")
                elif operation == "stop":
                    stop_threads()
                    respond(request_id, **camera_status())
                elif operation == "status":
                    respond(request_id, **camera_status())
                elif operation == "close":
                    stop_threads()
                    respond(request_id, ok=True, state="idle")
                    running = False
                else:
                    respond(request_id, ok=False, error="unsupported operation")
            except Exception as exc:
                with metrics_lock:
                    metrics["state"] = "error"
                    metrics["last_error"] = str(exc)
                respond(request_id, ok=False, error=str(exc))
    except Exception as exc:
        result_queue.put({"id": "ready", "ok": False, "error": str(exc)})
    finally:
        stop_threads()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy_started:
            try:
                rclpy.shutdown()
            except Exception:
                pass


class _CameraBackend:
    def __init__(self, topic, interface, fps, timeout_s, retry_s):
        context = multiprocessing.get_context("spawn")
        self._control = context.Queue()
        self._results = context.Queue()
        self._lock = threading.Lock()
        self._process = context.Process(
            target=_camera_process,
            args=(self._control, self._results, topic, interface, fps,
                  timeout_s, retry_s),
            name="as2w_camera",
            daemon=True,
        )
        self._process.start()
        try:
            ready = self._results.get(timeout=8.0)
        except queue.Empty:
            ready = {"ok": False, "error": "camera process startup timed out"}
        self.error = "" if ready.get("ok") else ready.get("error", "camera unavailable")

    def is_available(self):
        return not self.error and self._process.is_alive()

    def call(self, operation, timeout=6.0):
        if self.error:
            return {"ok": False, "state": "error", "error": self.error}
        if not self._process.is_alive():
            return {"ok": False, "state": "error", "error": "camera process is not running"}
        with self._lock:
            request_id = uuid4().hex
            self._control.put((request_id, operation, None))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    result = self._results.get(
                        timeout=max(0.01, deadline - time.monotonic()))
                except queue.Empty:
                    break
                if result.get("id") == request_id:
                    return result
            return {"ok": False, "state": "error", "error": "camera operation timed out"}

    def status(self):
        result = self.call("status", timeout=2.0)
        result.pop("id", None)
        return result

    def close(self):
        if self._process.is_alive():
            self.call("close", timeout=3.0)
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)


class _CameraNode:
    def __init__(self, topic, interface, fps, timeout_s, retry_s):
        self.topic = topic
        self.interface = interface
        self.fps = fps
        self.timeout_s = timeout_s
        self.retry_s = retry_s
        self.state = "idle"
        self.frames = 0
        self.last_error = ""
        self._backend = None

    def start_capture(self):
        if self._backend is not None and self._backend.is_available():
            # Camera is an always-on state source. Repeated canvas lifecycle
            # starts must not reset the videohub stream or its frame counters.
            return self._backend.status()
        self.stop_capture()
        self._backend = _CameraBackend(
            self.topic, self.interface, self.fps, self.timeout_s, self.retry_s,
        )
        if self._backend.error:
            self.state = "error"
            self.last_error = self._backend.error
            return {"ok": False, "state": "error", "error": self.last_error}
        result = self._backend.call("start")
        self.state = result.get("state", "starting") if result.get("ok") else "error"
        self.last_error = result.get("error", "")
        return result

    def stop_capture(self):
        if self._backend is not None:
            self._backend.close()
            self._backend = None
        self.state = "idle"

    def status(self):
        if self._backend is not None and self._backend.is_available():
            result = self._backend.status()
            self.state = result.get("state", self.state)
            self.frames = result.get("frames", self.frames)
            self.last_error = result.get("last_error", result.get("error", ""))
            return result
        if self._backend is not None:
            self.state = "error"
            self.last_error = (
                self._backend.error or "camera process is not running")
            return {
                "ok": False,
                "state": "error",
                "frames": self.frames,
                "last_frame_ago_ms": -1,
                "last_error": self.last_error,
            }
        return {
            "state": self.state,
            "frames": self.frames,
            "last_frame_ago_ms": -1,
            "last_error": self.last_error,
        }


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, config, namespace, executor, network_iface="eth0"):
        backend = config.get("backend", "videohub")
        if backend != "videohub":
            raise ValueError("As2W camera backend must be videohub")
        self._topic = "/{}/camera/front".format(namespace)
        self._camera_info_config = config.get("camera_info", {})
        # Validate declarations before any camera child process is started.
        self._camera_info()
        self._node = _CameraNode(
            self._topic,
            network_iface,
            max(1.0, min(20.0, float(config.get("fps", 10)))),
            max(0.2, float(config.get("rpc_timeout", 1.0))),
            max(0.2, float(config.get("retry_interval", 2.0))),
        )

    def _camera_info(self, status=None):
        try:
            from camera_specs import declare
        except ModuleNotFoundError as exc:
            if exc.name != "camera_specs":
                raise
            from unitree.as2w.camera_specs import declare
        if status is not None and "frame_width" in status and "frame_height" in status:
            self._observed_dimensions = (status["frame_width"], status["frame_height"])
        return declare(self._topic, getattr(self, "_camera_info_config", {}),
                       observed_dimensions=getattr(self, "_observed_dimensions", None))

    def get_tool(self):
        return {
            "name": "camera",
            "type": "sensor",
            "multiInstance": False,
            "description": "As2W front camera — JPEG frames from the verified videohub service.",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
            "camera_info": self._camera_info(),
        }

    def start(self):
        return self._node.start_capture()

    def stop(self):
        self._node.stop_capture()

    def dispatch(self, action, args):
        if action == "start":
            started = self.start() or {}
            result = self._node.status()
            actual_state = result.get("state", started.get("state", "error"))
            result["readiness"] = actual_state
            if (not started.get("ok", True)
                    or not result.get("ok", actual_state != "error")
                    or actual_state == "error"):
                result["state"] = "error"
                if not result.get("last_error"):
                    result["last_error"] = started.get(
                        "error", "camera failed to start")
            else:
                # Sensor lifecycle is ready for the canvas while asynchronous
                # frame readiness remains visible in the separate field.
                result["state"] = "running"
            result["topic_out"] = [
                {"topic": self._topic, "format": "image/jpeg"}]
            result["camera_info"] = self._camera_info(result)
            return result
        elif action == "stop":
            # Match G1 camera_rgb: stopping intelligent control detaches the
            # logical card but keeps the state stream alive. Physical shutdown
            # remains CameraPlugin.stop(), called by Bundle.stop_all().
            result = self._node.status()
            result["stream_state"] = result.get("state", "error")
            result["state"] = "idle"
            result["topic_out"] = [{"topic": self._topic, "format": "image/jpeg"}]
            result["camera_info"] = self._camera_info(result)
            return result
        if action == "info":
            result = self._node.status()
            result["topic_out"] = [
                {"topic": self._topic, "format": "image/jpeg"}]
            result["camera_info"] = self._camera_info(result)
            return result
        return None
