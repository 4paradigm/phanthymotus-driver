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
from uuid import uuid4

from audio_msgs.msg import AudioChunk
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


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
            self.state = "running"
            buffered.extend(data)
            while len(buffered) >= self.chunk_bytes and not stop_event.is_set():
                chunk = bytes(buffered[:self.chunk_bytes])
                del buffered[:self.chunk_bytes]
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

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            result = self._node.status()
            result["topic_out"] = [{"topic": self._topic, "format": "audio/pcm-16k"}]
            return result
        return None


_AUDIO_EOF_MAGIC = b"\x01\x00\xff\xff\x01\x00\xff\xff"
_SPEAKER_APP_NAME = "as2w_speaker"


def _speaker_worker(control_queue, result_queue, pcm_queue, interface, merge_bytes):
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
    deadline = time.monotonic()
    last_pcm = 0.0

    def respond(request_id, **payload):
        payload["id"] = request_id
        result_queue.put(payload)

    def play(block):
        nonlocal deadline
        if not block or not active or paused:
            return
        result = client.PlayStream(_SPEAKER_APP_NAME, stream_id, bytes(block))
        duration = len(block) / 32000.0
        now = time.monotonic()
        deadline = max(deadline, now - 0.24) + duration
        wait = deadline - time.monotonic() - 0.24
        if wait > 0:
            time.sleep(wait)
        return result

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
                    buffered.clear()
                    while True:
                        try:
                            pcm_queue.get_nowait()
                        except queue.Empty:
                            break
                    client.PlayStop(_SPEAKER_APP_NAME)
                    deadline = time.monotonic()
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
                    deadline = time.monotonic()
                    respond(request_id, ok=True)
                elif operation == "get_volume":
                    respond(request_id, ok=True, result=client.GetVolume())
                elif operation == "set_volume":
                    respond(request_id, ok=True, result=client.SetVolume(int(value)))
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
            pcm = pcm_queue.get(timeout=0.1)
        except queue.Empty:
            if buffered and active and not paused and time.monotonic() - last_pcm >= 0.2:
                play(buffered)
                buffered.clear()
            continue
        if pcm == _AUDIO_EOF_MAGIC:
            if muted_until_eof:
                muted_until_eof = False
                buffered.clear()
                continue
            play(buffered)
            buffered.clear()
            continue
        if not active or muted_until_eof:
            continue
        buffered.extend(pcm)
        last_pcm = time.monotonic()
        while len(buffered) >= merge_bytes:
            block = bytes(buffered[:merge_bytes])
            del buffered[:merge_bytes]
            play(block)


class _SpeakerBackend:
    def __init__(self, interface, merge_bytes):
        context = multiprocessing.get_context("spawn")
        self._control = context.Queue()
        self._results = context.Queue()
        self._pcm = context.Queue(maxsize=64)
        self._lock = threading.Lock()
        self._process = context.Process(
            target=_speaker_worker,
            args=(self._control, self._results, self._pcm, interface, merge_bytes),
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

    def put(self, pcm):
        if self.error:
            return
        try:
            self._pcm.put_nowait(pcm)
        except queue.Full:
            try:
                self._pcm.get_nowait()
                self._pcm.put_nowait(pcm)
            except (queue.Empty, queue.Full):
                pass

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


class _SpeakerNode(Node):
    def __init__(self, interface, merge_bytes):
        super().__init__("as2w_speaker")
        self._interface = interface
        self._merge_bytes = merge_bytes
        self._backend = None
        self._subscription = None
        self.topic = ""
        self.state = "idle"

    def start_backend(self):
        if self._backend is not None and self._backend.is_available():
            if self.state in ("idle", "error"):
                self.state = "ready"
            return {"ok": True}
        if self._backend is not None:
            self._backend.close()
        self._backend = _SpeakerBackend(self._interface, self._merge_bytes)
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
        if self._subscription is not None and self.topic != topic:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._subscription is None:
            self._subscription = self.create_subscription(
                AudioChunk, topic, self._on_audio, _LOW_LAT_QOS)
        self.topic = topic
        result = self.call_backend("reset")
        self.state = "ready" if result.get("ok") else "error"
        return result

    def _on_audio(self, message):
        if self._backend is None or not self._backend.is_available():
            self.state = "error"
            return
        self._backend.put(bytes(message.data))
        if self.state == "ready":
            self.state = "playing"

    def stop_play(self):
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        result = self.call_backend("stop")
        self.topic = ""
        self.state = "idle" if result.get("ok") else "error"
        return result

    def close(self):
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._backend is not None:
            self._backend.close()
            self._backend = None
        self.topic = ""
        self.state = "idle"


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config, namespace, executor, network_iface="eth0"):
        merge_ms = max(100, min(1000, int(config.get("buffer_ms", 300))))
        self._node = _SpeakerNode(network_iface, merge_ms * 32)
        executor.add_node(self._node)

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
                    "start": {"params": ["input_topic"]},
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
        if action in ("start", "play"):
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
            result = {"ok": backend is not None and backend.is_available()}
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
        result.update({"state": self._node.state, "topic": reported_topic})
        return result


def _put_latest(target_queue, item):
    try:
        target_queue.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        target_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        target_queue.put_nowait(item)
    except queue.Full:
        pass


def _camera_worker(frame_queue, status_queue, stop_event, interface, fps, timeout_s, retry_s):
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
            code, data = client.GetImageSample()
            frame = bytes(data or [])
            if code != 0:
                raise RuntimeError("videohub returned {}".format(code))
            if not (frame.startswith(b"\xff\xd8") and frame.endswith(b"\xff\xd9")):
                raise RuntimeError("videohub returned an invalid JPEG")
            failures = 0
            _put_latest(frame_queue, frame)
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


class _CameraNode(Node):
    def __init__(self, topic, interface, fps, timeout_s, retry_s):
        super().__init__("as2w_camera")
        self.topic = topic
        self.interface = interface
        self.fps = fps
        self.timeout_s = timeout_s
        self.retry_s = retry_s
        self.publisher = self.create_publisher(CompressedImage, topic, _IMAGE_QOS)
        self.state = "idle"
        self.frames = 0
        self.last_frame_ts = 0.0
        self.last_error = ""
        self._process = None
        self._thread = None
        self._stop_event = None
        self._frames = None
        self._statuses = None

    def start_capture(self):
        if self._process is not None and self._process.is_alive():
            return
        self.stop_capture()
        context = multiprocessing.get_context("spawn")
        self._frames = context.Queue(maxsize=2)
        self._statuses = context.Queue(maxsize=4)
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_camera_worker,
            args=(self._frames, self._statuses, self._stop_event, self.interface,
                  self.fps, self.timeout_s, self.retry_s),
            name="as2w_camera",
            daemon=True,
        )
        self._process.start()
        self.state = "starting"
        self._thread = threading.Thread(
            target=self._publish_loop, name="as2w-camera-publish", daemon=True)
        self._thread.start()

    def _publish_loop(self):
        while self._stop_event is not None and not self._stop_event.is_set():
            try:
                while True:
                    state, error = self._statuses.get_nowait()
                    self.state, self.last_error = state, error
            except queue.Empty:
                pass
            try:
                frame = self._frames.get(timeout=0.2)
            except queue.Empty:
                continue
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.format = "jpeg"
            message.data = frame
            self.publisher.publish(message)
            self.frames += 1
            self.last_frame_ts = time.monotonic()
            self.state = "running"

    def stop_capture(self):
        if self._stop_event is not None:
            self._stop_event.set()
        process, self._process = self._process, None
        if process is not None:
            process.join(timeout=3.0)
            if process.is_alive():
                process.terminate()
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._stop_event = None
        self.state = "idle"

    def status(self):
        return {
            "state": self.state,
            "frames": self.frames,
            "last_frame_ago_ms": (
                int((time.monotonic() - self.last_frame_ts) * 1000)
                if self.last_frame_ts else -1),
            "last_error": self.last_error,
        }


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, config, namespace, executor, network_iface="eth0"):
        backend = config.get("backend", "videohub")
        if backend != "videohub":
            raise ValueError("As2W camera backend must be videohub")
        self._topic = "/{}/camera/front".format(namespace)
        self._node = _CameraNode(
            self._topic,
            network_iface,
            max(1.0, min(20.0, float(config.get("fps", 10)))),
            max(0.2, float(config.get("rpc_timeout", 1.0))),
            max(0.2, float(config.get("retry_interval", 2.0))),
        )
        executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": "camera",
            "type": "sensor",
            "multiInstance": False,
            "description": "As2W front camera — JPEG frames from the verified videohub service.",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._node.start_capture()

    def stop(self):
        self._node.stop_capture()

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            result = self._node.status()
            result["topic_out"] = [{"topic": self._topic, "format": "image/jpeg"}]
            return result
        return None
