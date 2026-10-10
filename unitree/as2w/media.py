"""As2W microphone, front camera, speaker, and LED cards.

The canvas cards are named mic, camera_rgb, speaker, and led. Camera frames are
published on ``/{namespace}/camera/front`` so perception can append ``/objects``
and ``/visual_depth_summary``. Microphone audio is ``/{namespace}/mic/audio``.

Stream addresses default to the Unitree body multicast used by Go2. Override
them in config.yaml after checking the robot. Speaker playback and the LED use
the vendored A2 ``voice`` service (PlayStream / LedControl) inside the existing
RPC process, so a missing audio service does not take down locomotion.
"""

import socket
import struct
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from audio_msgs.msg import AudioChunk

MIC_GROUP_IP = "239.168.123.161"
MIC_PORT = 5555
CHUNK_BYTES = 1024
CAMERA_STREAM_ADDR = "230.1.1.1"
CAMERA_STREAM_PORT = 1720

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)
_AUDIO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


def _iface_ipv4(name):
    if not name:
        return ""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = struct.pack("256s", name.encode()[:15])
        return socket.inet_ntoa(fcntl_ioctl(sock, packed)[20:24])
    except OSError:
        return ""
    finally:
        sock.close()


def fcntl_ioctl(sock, packed):
    import fcntl
    return fcntl.ioctl(sock.fileno(), 0x8915, packed)


class _MicNode(Node):
    def __init__(self, topic, group, port, interface):
        super().__init__("as2w_mic")
        self._topic = topic
        self._group = group
        self._port = int(port)
        self._interface = interface
        self._pub = self.create_publisher(AudioChunk, topic, _AUDIO_QOS)
        self._sock = None
        self._thread = None
        self.state = "idle"

    def start_capture(self):
        if self._sock is not None:
            return self._topic
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self._port))
        local = _iface_ipv4(self._interface)
        membership = struct.pack(
            "4s4s",
            socket.inet_aton(self._group),
            socket.inet_aton(local) if local else b"\x00\x00\x00\x00",
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sock.settimeout(0.5)
        self._sock = sock
        self.state = "running"
        self._thread = threading.Thread(target=self._pump, daemon=True, name="as2w-mic")
        self._thread.start()
        self.get_logger().info(f"mic {self._group}:{self._port} → {self._topic}")
        return self._topic

    def stop_capture(self):
        sock = self._sock
        self._sock = None
        self.state = "idle"
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _pump(self):
        buf = bytearray()
        while self._sock is not None:
            try:
                buf.extend(self._sock.recv(4096))
            except socket.timeout:
                continue
            except OSError:
                break
            while len(buf) >= CHUNK_BYTES:
                chunk = bytes(buf[:CHUNK_BYTES])
                del buf[:CHUNK_BYTES]
                msg = AudioChunk()
                msg.format = "pcm_16k_16bit_mono"
                msg.data = list(chunk)
                self._pub.publish(msg)


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, executor, interface):
        self._topic = f"/{namespace}/mic/audio"
        self._group = config.get("group", MIC_GROUP_IP)
        self._port = int(config.get("port", MIC_PORT))
        self._interface = config.get("interface") or interface
        self._executor = executor
        self._node = None

    def get_tool(self):
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "description": f"As2W microphone — PCM 16 kHz. Publishes to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        try:
            if self._node is None:
                self._node = _MicNode(self._topic, self._group, self._port, self._interface)
                self._executor.add_node(self._node)
            self._node.start_capture()
        except Exception as exc:
            print(f"[as2w-mic] capture unavailable: {exc}", flush=True)

    def stop(self):
        if self._node is not None:
            self._node.stop_capture()

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            state = self._node.state if self._node is not None else "idle"
            return {"state": state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return None


def _run_camera_process(topic, stream_addr, stream_port, interface):
    from common import logsafe
    logsafe.install(check_fd=False)
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import DurabilityPolicy as _Dur, HistoryPolicy as _Hist, QoSProfile as _QoS, ReliabilityPolicy as _Rel
    from sensor_msgs.msg import CompressedImage

    qos = _QoS(reliability=_Rel.BEST_EFFORT, history=_Hist.KEEP_LAST, depth=1, durability=_Dur.VOLATILE)
    _rclpy.init()
    node = _Node("as2w_camera")
    pub = node.create_publisher(CompressedImage, topic, qos)
    cmd = [
        "gst-launch-1.0", "-q",
        "udpsrc", f"address={stream_addr}", f"port={stream_port}", f"multicast-iface={interface}", "!",
        "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96", "!",
        "rtph264depay", "!", "avdec_h264", "!", "videoconvert", "!",
        "jpegenc", "quality=75", "!", "fdsink", "fd=1",
    ]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        node.get_logger().error("gst-launch-1.0 not found — camera_rgb disabled")
        node.destroy_node()
        _rclpy.shutdown()
        return
    node.get_logger().info(f"camera_rgb {stream_addr}:{stream_port} via {interface} → {topic}")
    buf = bytearray()
    try:
        while proc.poll() is None:
            data = proc.stdout.read(65536)
            if not data:
                break
            buf.extend(data)
            while True:
                start = buf.find(b"\xff\xd8")
                if start < 0:
                    buf.clear()
                    break
                end = buf.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start:
                        del buf[:start]
                    break
                frame = bytes(buf[start:end + 2])
                del buf[:end + 2]
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = frame
                pub.publish(msg)
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
        node.destroy_node()
        _rclpy.shutdown()


class CameraRgbPlugin:
    PREFIX = "camera_rgb"

    def __init__(self, config, namespace, interface):
        self._topic = f"/{namespace}/camera/front"
        self._addr = config.get("stream_addr", CAMERA_STREAM_ADDR)
        self._port = int(config.get("stream_port", CAMERA_STREAM_PORT))
        self._interface = config.get("multicast_iface") or interface or "eth0"
        self._proc = None
        self.state = "idle"

    def get_tool(self):
        return {
            "name": "camera_rgb",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                f"As2W front camera — H.264 multicast decoded to JPEG on {self._topic}. "
                "Wire this into vop and visual_depth."
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        if self.state == "running":
            return
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self._proc = ctx.Process(
            target=_run_camera_process,
            args=(self._topic, self._addr, self._port, self._interface),
            name="as2w_camera",
            daemon=True,
        )
        self._proc.start()
        self.state = "running"

    def stop(self):
        proc = self._proc
        self._proc = None
        self.state = "idle"
        if proc is not None and proc.is_alive():
            proc.terminate()
            proc.join(timeout=3)
            if proc.is_alive():
                proc.kill()

    def dispatch(self, action, args):
        topic_out = [{"topic": self._topic, "format": "image/jpeg"}]
        if action == "start":
            self.start()
            return {"state": self.state, "topic_out": topic_out}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self.state, "topic_out": topic_out}
        return None


class _SpeakerNode(Node):
    def __init__(self, proxy):
        super().__init__("as2w_speaker")
        self._proxy = proxy
        self._sub = None
        self._topic = ""
        self.state = "idle"

    def start_play(self, topic):
        if self._sub is not None and self._topic == topic:
            return topic
        self.stop_play()
        self._topic = topic
        self._sub = self.create_subscription(AudioChunk, topic, self._on_audio, _AUDIO_QOS)
        self.state = "playing"
        self.get_logger().info(f"speaker subscribed {topic}")
        return topic

    def stop_play(self):
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        self.state = "idle"
        try:
            self._proxy.PlayStop("as2w_speaker")
        except Exception:
            pass

    def _on_audio(self, msg):
        pcm = bytes(msg.data)
        if not pcm:
            return
        try:
            self._proxy.PlayStream("as2w_speaker", "0", pcm)
        except Exception as exc:
            self.get_logger().error(f"speaker PlayStream failed: {exc}")


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config, namespace, executor, proxy):
        self._executor = executor
        self._proxy = proxy
        self._node = None

    def get_tool(self):
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "As2W speaker — plays a PCM 16 kHz topic through the robot voice service.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]},
                    "input_topic": {"type": "string", "description": "PCM 16 kHz topic, usually /perception/tts"},
                },
                "required": ["action"],
            },
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self):
        pass

    def stop(self):
        if self._node is not None:
            self._node.stop_play()

    def _node_ready(self):
        if self._node is None:
            self._node = _SpeakerNode(self._proxy)
            self._executor.add_node(self._node)
        return self._node

    def dispatch(self, action, args):
        if action in ("start", "play"):
            topic = args.get("input_topic", "")
            if not topic:
                return {"error": "Missing input_topic"}
            topic = self._node_ready().start_play(topic)
            return {"state": "playing", "topic": topic, "topic_in": [{"topic": topic, "format": "audio/pcm-16k"}]}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            topic = self._node._topic if self._node is not None else ""
            state = self._node.state if self._node is not None else "idle"
            topic_in = [{"topic": topic, "format": "audio/pcm-16k"}] if topic else [{"format": "audio/pcm-16k"}]
            return {"state": state, "topic": topic, "topic_in": topic_in}
        return None


class LedPlugin:
    PREFIX = "led"

    def __init__(self, config, namespace, executor, proxy):
        self._proxy = proxy

    def get_tool(self):
        return {
            "name": "led",
            "type": "actuator",
            "multiInstance": False,
            "description": "As2W body LED via the voice service LedControl.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set_color", "info"]},
                    "r": {"type": "integer", "description": "Red 0-255"},
                    "g": {"type": "integer", "description": "Green 0-255"},
                    "b": {"type": "integer", "description": "Blue 0-255"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "ready"}
        if action in ("start", "set_color"):
            if action == "start" and "r" not in args and "g" not in args and "b" not in args:
                return {"state": "ready"}
            color = [max(0, min(255, int(args.get(channel, 0)))) for channel in ("r", "g", "b")]
            code = self._proxy.LedControl(*color)
            return {"ret": code, "r": color[0], "g": color[1], "b": color[2]}
        if action == "stop":
            return {"state": "idle"}
        return None
