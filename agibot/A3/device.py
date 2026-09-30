#!/usr/bin/env python3
"""AgiBot A3 (AimDK v3.2) adaptation layer.

Unlike X2, the A3's control surface is NOT a ROS 2 service package: AimDK v3.2 exposes
HTTP JSON RPC endpoints split across three compute units (§7 of the dev guide) plus ROS 2
topics for streaming/commands. So this driver is a requests-based RPC client for
control/query, plus rclpy topic mirroring for sensor streams, on top of the same
`common/vendor_runtime.py` bundle/MCP skeleton as `agibot/AimDK_X2`.

Compute-unit layout (fixed internal IPs, dev guide §3.2):
  - HDU 10.42.10.10 — interaction: :59301 TTS/AgentControl/MicSource, :56666 HalAudio
    volume/PlayFile/StopPlay, :51049 ResourceService
  - ADU 10.42.10.11 — :50807 Mapping/Localization/Topo, :53176 PncService navigation,
    :50583 SLAM relocalization
  - MDU 10.42.10.12 — :56322 MotionControlAction/MotionService, :56444 MotionCommand,
    :50587 HDSService alerts
The driver container must run on a third-party compute unit, never on the MDU.

Protobuf-carrier topics use ros2_plugin_proto/msg/RosMsgWrapper (serialization_type
"pb", payload in .data) and need the a3_aimdk wheel's aimdk.protocol_pb2 to encode.
Plain-ROS topics (sensor_msgs JointState/Image/PointCloud2/Imu) are mirrored directly
like on X2.
"""

from __future__ import annotations

import base64
import json
import math
import struct
import threading
import time
import zlib
from array import array
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, jsonable, tool


def _acp_notify(action_id: str, status: str, result: dict, tool: str = ""):
    """POST action completion to Agent Core (module-level ACP helper)."""
    import urllib.request as _urllib
    import ssl as _ssl
    import os as _os

    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = _ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = _ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        req = _urllib.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        _urllib.urlopen(req, timeout=5, context=ctx)
    except Exception as e:
        import sys
        print(f"[ACP] callback failed for {action_id}: {e}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Vendor constants (AimDK v3.2 dev guide §7 — values taken verbatim from the docs)
# ---------------------------------------------------------------------------

# MotionControlActionService/SetAction actions. GetAvailableActions returns the live list;
# these are the four the docs enumerate explicitly (GET_UP example + the FSM basics).
MC_ACTIONS = {
    "damping": ("MotionControlAction_DAMPING", "DAMPING"),
    "get_up": ("MotionControlAction_GET_UP", "GET_UP"),
    "lie_down": ("MotionControlAction_LIE_DOWN", "LIE_DOWN"),
    "passive": ("MotionControlAction_PASSIVE", "PASSIVE"),
}

# motion_play duration ceiling — must stay consistent with the 600 s x-completion
# timeout declared in MotionPlayPlugin.get_tool.
MOTION_PLAY_MAX_DURATION_MS = 600_000

# 14 fixed arm joints (docs §7.1.4) — order matters, matches arm_joint_state.name.
ARM_JOINTS = {
    side: [f"{side}_{part}_joint" for part in (
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
        "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw",
    )]
    for side in ("left", "right")
}

# Arm joint position limits in rad (docs §7.1.4 table; left/right shoulder_roll mirror).
# Keys are joint-name suffixes: `<side>_<part>_joint` matches by stripping the side prefix.
ARM_JOINT_LIMITS = {
    "shoulder_pitch": (-2.87979, 2.87979),
    "shoulder_roll": (-2.79253, 2.79253),  # placeholder; per-side values below override
    "shoulder_yaw": (-2.79253, 2.79253),
    "elbow": (-0.95993, 1.74533),
    "wrist_roll": (-2.79253, 2.79253),
    "wrist_pitch": (-1.62316, 1.62316),
    "wrist_yaw": (-1.62316, 1.62316),
}
ARM_JOINT_LIMITS["left_shoulder_roll_joint"] = (-0.08727, 2.61799)
ARM_JOINT_LIMITS["right_shoulder_roll_joint"] = (-2.61799, 0.08727)


def _joint_limit(name, limits):
    """Look up a limit by full joint name, falling back to the side-less suffix."""
    if name in limits:
        return limits[name]
    stripped = name.split("_", 1)[1] if name.startswith(("left_", "right_")) else name
    stripped = stripped[:-len("_joint")] if stripped.endswith("_joint") else stripped
    return limits[stripped]


def _check_joint_limits(joints: dict, limits: dict):
    for name, value in joints.items():
        low, high = _joint_limit(name, limits)
        _clamp(value, low, high, name)
ARM_COMMAND_RATE_HZ = 100.0          # docs: 100 Hz, gap <= 30 ms
ARM_MAX_VELOCITY = 4.0               # docs: joint velocity must be <= 4 rad/s

# Waist (docs §7.1.7): pitch/yaw in rad, height in m.
WAIST_LIMITS = {
    "waist_pitch": (-1.6, 1.6),
    "waist_yaw": (-1.6, 1.6),
    "waist_height": (-0.3, 0.0),
}

# Neck (docs §7.1.6): yaw/pitch in rad.
NECK_LIMITS = {
    "head_yaw_joint": (-1.04720, 1.04720),
    "head_pitch_joint": (-0.43633, 0.26180),
}
NECK_JOINTS = ("head_yaw_joint", "head_pitch_joint")

# Hand (docs §7.1.5): command position 0 (open) .. 2000 (closed); state 0..4096.
HAND_COMMAND_MAX = 2000
HAND_TYPES = {"AgiHand": "agi_hand", "O10Hand": "o10_hand"}

# Mic source (docs §7.3.5): 0=internal (v3.2 has a known hardware BUG — avoid),
# 1=external (recommended).
MIC_SOURCES = {"internal": 0, "external": 1}

# TTS priority levels (docs §7.2.1 PlayTTS): INTERACTION_L6 is the documented example.
TTS_PRIORITY_LEVELS = {"background": "BACKGROUND_L1", "service": "SERVICE_L2", "interaction": "INTERACTION_L6"}
TTS_MAX_TEXT_BYTES = 1024  # docs: text <= 1024 bytes (~200 chars)
# GetAudioStatus states (docs §7.2.2): 0 未播 / 1 播报中 / 2 播报完成 / 3 异常。
TTS_AUDIO_PLAYING, TTS_AUDIO_DONE, TTS_AUDIO_ERROR = 1, 2, 3
# TTS 完成契约超时（tianyi2.0 tts speak 同款 180 s）：1024 字节文本播报远短于此，
# 媒体文件播放留出充足裕量；轮询 GetAudioStatus，超时兜底。
TTS_COMPLETION_TIMEOUT_S = 180

# Volume (docs §7.2.3): 0-100 scale but >70 risks damage — clamped in dispatch too.
VOLUME_HARD_MAX = 70

# spatial_map renderer budget (same contract as tianyi2.0/g1 map-view cards):
# canonical binary = struct('<fffBI', x, y, display_yaw, flags=7, n) + xyz points
# + struct('<I', len) + meta JSON, on a core-domain UInt8MultiArray topic.
MAP_MAX_POINTS = 80000        # renderer hard cap
MAP_FLOOR_POINTS = 39000       # traversable (127) cells keep a denser floor layer
MAP_FEATURE_POINTS = 13000    # walls/obstacles stay on a finer grid
MAP_PUBLISH_INTERVAL = 0.35

# Resource types (docs §7.5.1 GetResourceList).
RESOURCE_TYPES = {
    "motion": 1, "emoticon": 2, "audio": 3, "skill": 6, "map": 7, "offring_work": 8,
}
RESOURCE_TYPE_NAMES = {
    "motion": "RESOURCE_TYPE_MOTION", "emoticon": "RESOURCE_TYPE_EMOTICON",
    "audio": "RESOURCE_TYPE_AUDIO", "skill": "RESOURCE_TYPE_SKILL",
    "map": "RESOURCE_TYPE_MAP", "offring_work": "RESOURCE_TYPE_OFFRING_WORK",
}

# Sensor topics (docs §7.6.2 table) — only the practically-useful subset gets cards.
# key -> (robot_topic, format, description)
# Formats must appear in README_dev § "Data Format & Dashboard Rendering": the dashboard
# picks its renderer by exact format match. `image/raw` has no renderer at all (falls
# back to the activity stream — no image shown), so RGB is re-encoded to JPEG and depth
# to zlib-compressed uint16 (both per README_dev's recommended patterns; the raw
# sensor_msgs/Image mirror would also cost 614KB/frame on the depth channel).
CAMERA_TOPICS = {
    "head_left_fisheye": ("/hal/head_left_fisheye_camera/rgb", "image/jpeg", "头部左鱼眼相机 RGB"),
    "head_right_fisheye": ("/hal/head_right_fisheye_camera/rgb", "image/jpeg", "头部右鱼眼相机 RGB"),
    "head_rear_fisheye": ("/hal/head_rear_fisheye_camera/rgb", "image/jpeg", "头部后鱼眼相机 RGB"),
    "chest_front_d457_rgb": ("/hal/chest_front_d457_camera/rgb", "image/jpeg", "胸前 D457 相机 RGB"),
    "chest_front_d457_depth": ("/hal/chest_front_d457_camera/depth", "image/depth-zlib", "胸前 D457 相机深度"),
    "waist_front_d415_rgb": ("/hal/waist_front_d415_camera/rgb", "image/jpeg", "腰前 D415 相机 RGB"),
    "waist_front_d415_depth": ("/hal/waist_front_d415_camera/depth", "image/depth-zlib", "腰前 D415 相机深度"),
    "wrist_left_d405_rgb": ("/hal/wrist_left_d405_camera/rgb", "image/jpeg", "左腕 D405 相机 RGB"),
    "wrist_right_d405_rgb": ("/hal/wrist_right_d405_camera/rgb", "image/jpeg", "右腕 D405 相机 RGB"),
}
# H265 foxglove CompressedVideo streams exist too, but consumer-side H265 decode support
# is inconsistent — the raw sensor_msgs/Image feeds are mirrored and re-encoded instead
# (JPEG for RGB, zlib uint16 for depth; see _encode_rgb/_encode_depth below).

RESOURCE_DIR = Path(__file__).with_name("resource")


# ---------------------------------------------------------------------------
# HTTP JSON RPC layer
# ---------------------------------------------------------------------------

def create_header(control_source: str = "ControlSource_SAFE") -> dict:
    """Request header per the SDK's own S_SetAction.py example (§7.1.2)."""
    now = datetime.now(timezone.utc)
    return {
        "timestamp": {
            "seconds": int(now.timestamp()),
            "nanos": now.microsecond * 1000,
            "ms_since_epoch": int(now.timestamp() * 1000),
        },
        "control_source": control_source,
        "uuid": "",
        "trace_id": "phanthymotus_a3",
        "domin": "",
    }


class RpcError(RuntimeError):
    """Vendor RPC rejected the request (nonzero header.code in the response)."""


class RpcClient:
    """Thin POST-only JSON RPC client aimed at one compute unit.

    Kept injectable (`transport`) so unit tests can stub the HTTP layer without
    any network, mirroring how X2's tests stub rclpy clients.
    """

    def __init__(self, host: str, timeout: float = 5.0, transport=None):
        self.host = host
        self.timeout = timeout
        self._transport = transport  # callable(url, payload, timeout) -> response dict

    def call(self, service: str, method: str, payload: dict | None = None) -> dict:
        url = f"http://{self.host}/rpc/aimdk.protocol.{service}/{method}"
        body = payload if payload is not None else {"header": create_header()}
        if self._transport is not None:
            response = self._transport(url, body, self.timeout)
        else:
            import requests
            response = requests.post(url, json=body, timeout=self.timeout).json()
        if not isinstance(response, dict):
            raise ValueError(f"{service}/{method}: non-dict response {response!r}")
        # Vendor protocol signals failures via header.code — an HTTP 200 with a
        # nonzero code means the robot REJECTED the command. Raising here keeps
        # callers from treating the rejection as success (e.g. motion_play must
        # not arm its completion worker and later report completed for a motion
        # the robot never started).
        if not _header_ok(response):
            header = response.get("header") or {}
            message = header.get("msg") or header.get("message") or ""
            raise RpcError(
                f"{service}/{method} 拒绝请求: header.code={header.get('code')!r}"
                f" msg={message!r}")
        return response


def _header_ok(response: dict) -> bool:
    header = response.get("header") or {}
    return str(header.get("code", "0")) == "0"


def _strip_prefix(value: str, prefix: str) -> str:
    return value[len(prefix):] if isinstance(value, str) and value.startswith(prefix) else value


def _require(condition: bool, message: str):
    if not condition:
        raise ValueError(message)


def _clamp(value: float, low: float, high: float, label: str) -> float:
    value = float(value)
    if value < low or value > high:
        raise ValueError(f"{label}={value} 超出范围 [{low}, {high}]")
    return value


def _check_joint_limits(joints: dict, limits: dict):
    for name, value in joints.items():
        low, high = _joint_limit(name, limits)
        _clamp(value, low, high, name)


# ---------------------------------------------------------------------------
# Node + topic plumbing
# ---------------------------------------------------------------------------

class _FakeClock:
    """Minimal clock stand-in used when the robot node provides none (tests)."""

    def now(self):
        import datetime as _dt
        return _FakeTime(_dt.datetime.now())


class _FakeTime:
    def __init__(self, when):
        self._when = when

    def to_msg(self):
        stamp = type("Stamp", (), {})()
        stamp.sec = int(self._when.timestamp())
        stamp.nanosec = 0
        return stamp


def _fill_protobuf(message, payload):
    """Recursively fill a protobuf message from a JSON-friendly dict."""
    from google.protobuf.message import Message as _PbMessage

    for key, value in payload.items():
        if value is None:
            continue
        field = getattr(message, key, None)
        if isinstance(value, dict):
            if field is None:
                continue
            _fill_protobuf(field, value)
        elif isinstance(value, list):
            if isinstance(field, _PbMessage):
                for item in value:
                    if isinstance(item, dict):
                        _fill_protobuf(field.add(), item)
                    else:
                        field.append(item)
            elif field is not None:
                try:
                    field.extend(value)
                except TypeError:
                    for item in value:
                        field.append(item)
        else:
            try:
                setattr(message, key, value)
            except (AttributeError, TypeError):
                pass


class A3Nodes:
    """Robot-side ROS 2 node: mirrors sensor topics into the core domain (same pattern
    as X2's AimdkNodes) and owns the command publishers that go straight to the robot."""

    def __init__(self, config, namespace, ros2, rpc: A3Rpc):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import Image, Imu, JointState, PointCloud2
        from std_msgs.msg import String

        self.config = config
        self.namespace = namespace
        self.rpc = rpc
        self.robot = Node("agibot_a3_driver_robot", context=ros2.ctx_robot)
        self.core = Node("agibot_a3_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)

        self.lock = threading.RLock()
        self.values = {}

        self._String = String
        self._JointState = JointState
        self._Image = Image

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.streams = {}
        self.clock = getattr(self.robot, 'get_clock', lambda: _FakeClock())()
        self._pb_topic = ''

        def mirror(key, msg_type, robot_topic, fmt, qos=None, json_filter=None,
                   re_encode=None):
            core_topic = f"/{namespace}/agibot_a3/{key}"
            as_json = fmt == "data/json"
            if re_encode is not None:
                core_msg_type = self._CompressedImage
            elif as_json:
                core_msg_type = String
            else:
                core_msg_type = msg_type
            pub = self.core.create_publisher(core_msg_type, core_topic, 5)

            def callback(msg):
                if re_encode is not None:
                    out = re_encode(msg)
                    if out is not None:
                        pub.publish(out)
                    return
                if as_json:
                    value = json_filter(msg) if json_filter else jsonable(msg)
                    output = self._String()
                    output.data = json.dumps(value, ensure_ascii=False)
                    pub.publish(output)
                    with self.lock:
                        self.values[key] = value
                else:
                    pub.publish(msg)

            self.robot.create_subscription(msg_type, robot_topic, callback, qos or sensor_qos)
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

        # -- joint / state streams (plain sensor_msgs types) --
        mirror("arm_state", JointState, "/motion/control/arm_joint_state", "data/json")
        mirror("hand_state", JointState, "/motion/control/hand_joint_state", "data/json")
        mirror("neck_state", JointState, "/motion/control/neck_joint_state", "data/json")
        mirror("lidar_cloud", PointCloud2, "/hal/neck_middle_livox_lidar/pointcloud", "sensor/pointcloud")
        mirror("imu_pelvis", Imu, "/ros2/body_drive/pelvis_imu/data", "data/json")
        mirror("imu_torso", Imu, "/ros2/body_drive/torso_imu/data", "data/json")

        # -- camera streams, config-selected subset (re-encoded on the fly: RGB → JPEG,
        # depth → zlib uint16, per README_dev § Data Format) --
        camera_cfg = config.get("plugins", {}).get("camera", {})
        selected = camera_cfg.get("streams") or ["head_left_fisheye", "chest_front_d457_rgb",
                                                 "chest_front_d457_depth"]
        for key in selected:
            topic, fmt, _ = CAMERA_TOPICS[key]
            encoder = self._encode_depth if fmt == "image/depth-zlib" else self._encode_rgb
            mirror(f"camera_{key}", Image, topic, fmt, re_encode=encoder)

        # -- protobuf-carrier streams (RosMsgWrapper) -- decoded to JSON via pb2 if the
        # wheel is importable, otherwise subscribed raw and passed through opaquely --
        self._pb = None
        try:
            from aimdk import protocol_pb2  # a3_aimdk wheel
            self._pb = protocol_pb2
        except ImportError:
            pass

        if self._pb is not None:
            mirror("battery", self._wrapper_type(ros2), "/aima/bms/data/pb_3Aaimdk_2Eprotocol_2EBmsStateChannel",
                   "data/json", json_filter=self._decode_bms)
            mirror("estop", self._wrapper_type(ros2), "/hal_state/emergency/pb_3Aaimdk_2Eprotocol_2EEmergencyStateChannel",
                   "data/json", json_filter=self._decode_emergency)
            mirror("skill_status", self._wrapper_type(ros2), "/skill/pilot/skill_status",
                   "data/json", json_filter=self._decode_skill_status)

        # -- command publishers (robot domain) --
        # RosMsgWrapper needs the dev-kit's ros2_plugin_proto package; without it the
        # driver still starts (Dockerfile documents a degraded non-robot dev mode) —
        # wrapper publishers are withheld and their tools reject clearly at dispatch.
        self.wrapper_available = self._probe_wrapper_type() is not None
        if self.wrapper_available:
            self.locomotion_pub = self.robot.create_publisher(
                self._wrapper_msg_type, "/motion/control/locomotion_velocity", 10)
            self.waist_pub = self.robot.create_publisher(
                self._wrapper_msg_type, "/motion/control/move_waist", 10)
            self.face_play_pub = self.robot.create_publisher(
                self._wrapper_msg_type, "/skill/pilot/face/play", 10)
        else:
            self.locomotion_pub = None
            self.waist_pub = None
            self.face_play_pub = None
            print("[warn] ros2_plugin_proto not importable — loco/waist_control/face_play "
                  "command publishers withheld (install the AimDK dev-kit prebuilt)")
        self.arm_command_pub = self.robot.create_publisher(JointState, "/motion/control/arm_joint_command", 10)
        self.neck_command_pub = self.robot.create_publisher(JointState, "/motion/control/neck_joint_command", 10)
        self.hand_command_pub = self.robot.create_publisher(JointState, "/motion/control/hand_joint_command", 10)

    # -- RosMsgWrapper helpers ------------------------------------------------

    @property
    def _CompressedImage(self):
        """sensor_msgs/CompressedImage, imported lazily (test stubs provide it)."""
        if getattr(self, "_compressed_image_type", None) is None:
            from sensor_msgs.msg import CompressedImage
            self._compressed_image_type = CompressedImage
        return self._compressed_image_type

    # -- camera re-encoders (README_dev § Data Format: JPEG for RGB, zlib uint16
    #    for depth; both fall back to passthrough when numpy/cv2 are missing) --

    # rgb8 → BGR before JPEG encoding; bgr8 needs no conversion —
    # cv2.imencode expects BGR natively and "COLOR_BGR2BGR" is not a real
    # OpenCV constant (a getattr on it raises and the frame is silently
    # dropped by the fallback below).
    _CV2_COLOR = {"rgb8": "COLOR_RGB2BGR"}

    def _encode_rgb(self, msg):
        """sensor_msgs/Image (rgb8/bgr8) → CompressedImage jpeg (quality 50)."""
        try:
            import cv2  # noqa: F401 — presence check only
            import numpy as np
        except ImportError:
            return None
        height, width = int(msg.height), int(msg.width)
        encoding = str(getattr(msg, "encoding", "rgb8"))
        if encoding not in ("rgb8", "bgr8") or not height or not width:
            return None
        try:
            import cv2
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(height, width, 3)
            conversion = self._CV2_COLOR.get(encoding)
            if conversion is not None:
                img = cv2.cvtColor(img, getattr(cv2, conversion))
            ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
            if not ok:
                return None
            out = self._CompressedImage()
            out.format = "jpeg"
            out.data = jpeg.tobytes()
            return out
        except Exception:
            return None

    def _encode_depth(self, msg):
        """sensor_msgs/Image (16UC1) → CompressedImage "16UC1; compressedDepth zlib"."""
        try:
            import numpy as np
        except ImportError:
            return None
        height, width = int(msg.height), int(msg.width)
        if str(getattr(msg, "encoding", "16UC1")) != "16UC1" or not height or not width:
            return None
        try:
            depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(height, width)
            out = self._CompressedImage()
            out.format = "16UC1; compressedDepth zlib"
            out.data = zlib.compress(depth.tobytes(), 1)
            return out
        except Exception:
            return None

    def _probe_wrapper_type(self):
        """Try importing ros2_plugin_proto's RosMsgWrapper; None when absent."""
        try:
            from ros2_plugin_proto.msg import RosMsgWrapper
        except ImportError:
            return None
        self._wrapper_msg_type = RosMsgWrapper
        return RosMsgWrapper

    def _wrapper_type(self, ros2):
        """Import ros2_plugin_proto/msg/RosMsgWrapper lazily (test stubs provide it)."""
        if getattr(self, "_wrapper_msg_type", None) is None:
            if self._probe_wrapper_type() is None:
                raise RuntimeError(
                    "ros2_plugin_proto not importable — install the AimDK dev-kit "
                    "prebuilt (ros2_plugin_proto_aarch64) to publish wrapper commands")
        return self._wrapper_msg_type

    def _make_wrapper(self, proto_dict: dict, ros2=None):
        """Serialize a protobuf payload dict into a RosMsgWrapper message.

        With the a3_aimdk wheel installed the payload dict is encoded to real
        protobuf bytes via the documented per-topic message types; without the
        wheel the JSON dict is carried directly (sufficient for tests/dev, and
        the on-robot image ships the wheel so production always takes the pb path).
        """
        wrapper = self._wrapper_type(ros2)()
        wrapper.serialization_type = "pb"
        wrapper.data = self._encode_pb(proto_dict)
        return wrapper

    _PB_TOPIC_MESSAGES = {
        "/motion/control/locomotion_velocity": "LocomotionVelocity",
        "/motion/control/move_waist": "MoveWaist",
        "/skill/pilot/face/play": "FacePlayInfo",
    }

    def _encode_pb(self, proto_dict):
        if self._pb is None:
            return json.dumps(proto_dict, ensure_ascii=False).encode()
        message_type = self._PB_TOPIC_MESSAGES.get(self._pb_topic, "")
        cls = getattr(self._pb, message_type, None) if message_type else None
        if cls is None:
            return json.dumps(proto_dict, ensure_ascii=False).encode()
        message = cls()
        try:
            _fill_protobuf(message, proto_dict)
            return message.SerializeToString()
        except Exception:
            return json.dumps(proto_dict, ensure_ascii=False).encode()

    def publish_wrapper(self, pub_attr, payload):
        """Fill + publish a RosMsgWrapper command onto a robot-domain topic."""
        publisher = getattr(self, pub_attr)
        if publisher is None:  # wrapper type unavailable — degraded dev mode
            raise RuntimeError(
                f"{pub_attr} unavailable: ros2_plugin_proto not importable "
                "(install the AimDK dev-kit prebuilt to publish wrapper commands)")
        self._pb_topic = getattr(publisher, "topic_name", "") or getattr(publisher, "topic", "")
        wrapper = self._make_wrapper(payload)
        publisher.publish(wrapper)
        self._pb_topic = ""

    def publish_joint_command(self, pub_attr, positions, duration_ms=None, frame_id=""):
        """Publish a JointState command frame, repeating at 100 Hz for duration_ms.

        Dev guide §7.3: joint commands must stream at 100 Hz (<=30 ms gap);
        velocity/effort must be zero. A single dispatch call therefore repeats
        the same frame for the requested hold window on a background thread.
        """
        publisher = getattr(self, pub_attr)
        JointState = self._JointState

        def frame():
            msg = JointState()
            try:
                stamp = self.clock.now().to_msg()
                msg.header.stamp = stamp
            except Exception:
                pass
            if frame_id:
                msg.header.frame_id = frame_id
            names = list(positions.keys())
            msg.name = names
            msg.position = [float(positions[name]) for name in names]
            msg.velocity = [0.0] * len(names)
            msg.effort = [0.0] * len(names)
            return msg

        publisher.publish(frame())
        if duration_ms:
            total = max(int(duration_ms), 0)

            def hold():
                deadline = time.monotonic() + total / 1000.0
                while time.monotonic() < deadline:
                    time.sleep(0.01)
                    try:
                        publisher.publish(frame())
                    except Exception:
                        break

            threading.Thread(target=hold, daemon=True).start()

    # -- protobuf decoders (best-effort; fall back to raw wrapper fields) ------

    def _decode_bms(self, msg):
        if self._pb is None:
            return jsonable(msg)
        channel = self._pb.BmsStateChannel()
        try:
            channel.ParseFromString(bytes(msg.data))
        except Exception:
            return jsonable(msg)
        return jsonable(channel)

    def _decode_emergency(self, msg):
        if self._pb is None:
            return jsonable(msg)
        channel = self._pb.EmergencyStateChannel()
        try:
            channel.ParseFromString(bytes(msg.data))
        except Exception:
            return jsonable(msg)
        return jsonable(channel)

    def _decode_skill_status(self, msg):
        if self._pb is None:
            return jsonable(msg)
        status = self._pb.SkillPilotStatus()
        try:
            status.ParseFromString(bytes(msg.data))
        except Exception:
            return jsonable(msg)
        return jsonable(status)

    # -- generic mirror callback ------------------------------------------------

    def snapshot(self, key):
        with self.lock:
            return self.values.get(key, {})

    def urdf_text(self, variant=None):
        path = RESOURCE_DIR / "a3_ultra.urdf"
        if not path.exists():
            raise ValueError("no URDF vendored for A3 (placeholder resource)")
        return path.read_text(encoding="utf-8")

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()


class A3Rpc:
    """The three compute-unit RPC clients, shared by every plugin.

    A3 runs the same `aimdk.protocol.*` services on different ports per unit
    (dev guide §7), so each (unit, port) pair gets its own RpcClient handle.
    """

    def __init__(self, config, transport=None):
        rpc_cfg = config.get("rpc", {})
        hdu = rpc_cfg.get("hdu", "10.42.10.10")
        adu = rpc_cfg.get("adu", "10.42.10.11")
        mdu = rpc_cfg.get("mdu", "10.42.10.12")
        timeout = float(rpc_cfg.get("timeout", 5.0))
        mk = lambda host: RpcClient(host, timeout, transport)
        self.hdu = mk(hdu)            # port-less client unused directly
        self.hdu_port = hdu
        self.adu_port = adu
        self.mdu_port = mdu
        self.timeout = timeout
        self.transport = transport

    def _unit(self, host, port):
        return RpcClient(f"{host}:{port}", self.timeout, self.transport)

    # -- MDU :56322 MotionControlAction/MotionService --------------------------------

    def set_action(self, action: str, ext_action: str) -> dict:
        payload = {"header": create_header(),
                   "command": {"action": action, "ext_action": ext_action}}
        return self._unit(self.mdu_port, 56322).call("MotionControlActionService", "SetAction", payload)

    def get_action(self) -> dict:
        return self._unit(self.mdu_port, 56322).call("MotionControlActionService", "GetAction", {})

    def get_available_actions(self) -> dict:
        return self._unit(self.mdu_port, 56322).call("MotionControlActionService", "GetAvailableActions")

    def arm_compliance(self, method: str) -> dict:
        # EnableArmCompliance / DisableArmCompliance / CheckArmCompliance all take {}
        rpc_method = {"enable": "EnableArmCompliance", "disable": "DisableArmCompliance",
                      "check": "CheckArmCompliance"}[method]
        return self._unit(self.mdu_port, 56322).call("MotionControlMotionService", rpc_method, {})

    # -- MDU :56444 MotionCommandService ----------------------------------------------

    def send_motion_command(self, motion_id: str, duration_ms: int = 0, cmd_end=True, cmd_pause=False,
                            cmd_reset=False, cmd_repeat=False) -> dict:
        payload = {"motion_id": str(motion_id), "duration_ms": int(duration_ms or 0),
                   "cmd_end": bool(cmd_end), "cmd_pause": bool(cmd_pause),
                   "cmd_reset": bool(cmd_reset), "cmd_repeat": bool(cmd_repeat)}
        return self._unit(self.mdu_port, 56444).call("MotionCommandService", "SendMotionCommand", payload)

    # -- MDU :50587 HDSService ---------------------------------------------------------

    def get_alert_list(self) -> dict:
        return self._unit(self.mdu_port, 50587).call("HDSService", "GetAlertList", {})

    # -- HDU :59301 TTSService / AgentControlService / HalAudioService mic ------------

    def play_tts(self, text: str, priority_level: str = "INTERACTION_L6", is_interrupted: bool = True,
                 trace_id: str = "") -> dict:
        payload = {"text": text, "priority_level": priority_level,
                   "domain": "phanthymotus", "trace_id": str(trace_id or ""),
                   "is_interrupted": bool(is_interrupted)}
        return self._unit(self.hdu_port, 59301).call("TTSService", "PlayTTS", payload)

    def play_media_file(self, file_name: str, is_interrupted: bool = True, trace_id: str = "") -> dict:
        payload = {"file_name": file_name, "priority_level": "INTERACTION_L6",
                   "domain": "phanthymotus", "trace_id": str(trace_id or ""),
                   "is_interrupted": bool(is_interrupted)}
        return self._unit(self.hdu_port, 59301).call("TTSService", "PlayMediaFile", payload)

    def get_audio_status(self, trace_id: str) -> dict:
        return self._unit(self.hdu_port, 59301).call("TTSService", "GetAudioStatus", {"trace_id": trace_id})

    def stop_tts_trace_id(self, trace_id: str) -> dict:
        return self._unit(self.hdu_port, 59301).call("TTSService", "StopTTSTraceId", {"trace_id": trace_id})

    def set_voice_enable(self, enable: bool) -> dict:
        return self._unit(self.hdu_port, 59301).call("AgentControlService", "SetVoiceEnable", {"enable_voice": bool(enable)})

    def get_voice_enable(self) -> dict:
        return self._unit(self.hdu_port, 59301).call("AgentControlService", "GetVoiceEnable", {})

    def set_agent_properties(self, mode: str) -> dict:
        # docs §7.3.4: properties {"2": "normal"|"only_voice"}; needs a reboot to apply
        value = "only_voice" if mode == "only_voice" else "normal"
        payload = {"contents": {"properties": {"2": value}}}
        return self._unit(self.hdu_port, 59301).call("AgentControlService", "SetAgentPropertiesRequest", payload)

    def get_agent_properties(self) -> dict:
        return self._unit(self.hdu_port, 59301).call("AgentControlService", "GetAgentPropertiesRequest", {"property_ids": [2]})

    def set_mic_source(self, source: int) -> dict:
        return self._unit(self.hdu_port, 59301).call("HalAudioService", "SetMicSourceRequest", {"mic_source": int(source)})

    def get_mic_source(self) -> dict:
        return self._unit(self.hdu_port, 59301).call("HalAudioService", "GetMicSourceRequest", {})

    # -- HDU :56666 HalAudioService volume / PlayFile --------------------------------

    def get_audio_volume(self) -> dict:
        return self._unit(self.hdu_port, 56666).call("HalAudioService", "GetAudioVolume", {})

    def set_audio_volume(self, volume: int, is_mute: bool = False) -> dict:
        payload = {"audio_volume": int(volume), "is_mute": bool(is_mute), "type": "SPEAKER_BUILT_IN"}
        return self._unit(self.hdu_port, 56666).call("HalAudioService", "SetAudioVolume", payload)

    def play_file(self, file_name: str, priority: str = "DEFAULT") -> dict:
        payload = {"pkg_name": "", "file_name": file_name, "file_path": "",
                   "priority": priority, "priority_weight": 0, "samplerate": 16000}
        return self._unit(self.hdu_port, 56666).call("HalAudioService", "PlayFile", payload)

    def stop_play(self) -> dict:
        return self._unit(self.hdu_port, 56666).call("HalAudioService", "StopPlay", {})

    # -- HDU :51049 ResourceService ---------------------------------------------------

    def resource_list(self, resource_type: str) -> dict:
        return self._unit(self.hdu_port, 51049).call(
            "ResourceService", "GetResourceList",
            {"header": create_header(), "resource_type": RESOURCE_TYPE_NAMES[resource_type]})

    # -- ADU :50807 MappingService / LocalizationService -------------------------------

    def get_2d_whole_map(self, map_id: str) -> dict:
        payload = {"command": "MappingCommand_GET_2D_WHOLE_MAP", "map_id": map_id}
        return self._unit(self.adu_port, 50807).call("MappingService", "Get2DWholeMap", payload)

    def get_stored_map_names(self) -> dict:
        payload = {"command": "MappingCommand_GET_STORED_MAP_NAME"}
        return self._unit(self.adu_port, 50807).call("MappingService", "GetStoredMapNames", payload)

    def get_current_working_map(self) -> dict:
        payload = {"command": "MappingCommand_GET_CURRENT_WORKING_MAP"}
        return self._unit(self.adu_port, 50807).call("MappingService", "GetCurrentWorkingMap", payload)

    def get_topo_msgs(self, map_id) -> dict:
        payload = {"command": "TopoCommand_GET_TOPO_MSG", "map_id": map_id}
        return self._unit(self.adu_port, 50807).call("LocalizationService", "GetTopoMsgs", payload)

    def start_mapping(self) -> dict:
        payload = {"header": {}, "command": "MappingCommand_START_MAPPING", "no_realtime_data": True}
        return self._unit(self.adu_port, 50807).call("MappingService", "StartMapping", payload)

    def stop_mapping(self, map_name: str | None = None) -> dict:
        if map_name:
            payload = {"command": "MappingCommand_SAVING_MAP", "map_name": map_name}
        else:
            payload = {"command": "MappingCommand_STOP_MAPPING"}
        return self._unit(self.adu_port, 50807).call("MappingService", "StopMapping", payload)

    def rename_map(self, map_id: str, old_name: str, new_name: str) -> dict:
        payload = {"command": "MappingCommand_RENAME_MAP", "map_id": map_id,
                   "old_name": old_name, "new_name": new_name}
        return self._unit(self.adu_port, 50807).call("MappingService", "RenameMap", payload)

    # -- ADU :53176 PncService ---------------------------------------------------------

    def navi(self, method: str, payload: dict) -> dict:
        return self._unit(self.adu_port, 53176).call("PncService", method, payload)

    def navi_state(self, task_id: int = 0) -> dict:
        return self._unit(self.adu_port, 53176).call("PncService", "ActionGetState", {"task_id": int(task_id)})

    # -- ADU :50583 SLAMRelocalization / SkillPilotService ------------------------------

    def slam_start_normal_relocalization(self, related_map_dir: str) -> dict:
        payload = {"header": {}, "related_map_dir": related_map_dir}
        return self._unit(self.adu_port, 50583).call("SLAMRelocalizationService", "SLAMStartNormalRelocalization", payload)

    def slam_stop_normal_relocalization(self, reloc_pose: dict | None = None) -> dict:
        payload = {"header": {}, "command_type": 0, "reloc_pose": reloc_pose or {}}
        return self._unit(self.adu_port, 50583).call("SLAMRelocalizationService", "SLAMStopNormalRelocalization", payload)

    def auto_charging(self, command: str, trigger: str) -> dict:
        payload = {"header": create_header(), "command": command, "trigger": trigger}
        return self._unit(self.adu_port, 50583).call("SkillPilotService", "AutoCharging", payload)

    def skill_package(self, command: str, path: str, session_id: str = "") -> dict:
        payload = {"source": "custom", "command": command, "path": path, "session_id": session_id}
        return self._unit(self.adu_port, 50583).call("SkillPilotService", "SkillPackage", payload)


# ---------------------------------------------------------------------------
# Plugins
# ---------------------------------------------------------------------------

def _stream_tool(key, stream, description):
    return tool(key, "sensor", description, topic_out=[{"topic": stream["topic"], "format": stream["format"]}])


class JointsPlugin:
    """joints 状态卡：手臂/灵巧手/头部三路关节状态流合一张卡（camera 卡同款
    多路复用模式——group 参数选择，镜像话题各自保持不变）。"""

    GROUPS = {
        "arm": ("arm_state", "14 自由度手臂关节（position/velocity/effort）"),
        "hand": ("hand_state", "手指（0-4096；frame_id 标识 AgiHand/O10Hand；O10Hand 含压力阵列）"),
        "neck": ("neck_state", "头部偏航/俯仰关节"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self.streams = {k: nodes.streams[k] for k in ("arm_state", "hand_state", "neck_state")}

    def get_tool(self):
        return tool("joints", "sensor", "关节状态流（按 group 选择 arm/hand/neck；"
                                     "config.yaml plugins.joints 控制启用）", {
            "type": "object",
            "properties": {
                "group": {"type": "string", "enum": list(self.GROUPS),
                          "description": "关节组：arm 双臂 14 关节 / hand 手指 / neck 头部"},
            },
        })

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            group = args.get("group", "arm")
            if group not in self.GROUPS:
                raise ValueError(f"joints: unknown group {group!r}; available: {list(self.GROUPS)}")
            stream = self.streams[self.GROUPS[group][0]]
            return {"state": "running",
                    "topic_out": [{"topic": stream["topic"], "format": stream["format"]}],
                    "streams": dict(self.streams)}
        group = args.get("group", "arm")
        if group not in self.GROUPS:
            raise ValueError(f"joints: unknown group {group!r}; available: {list(self.GROUPS)}")
        key = self.GROUPS[group][0]
        return {"state": "running", **self.streams[key]}


class ImuPlugin:
    """Two body IMUs (pelvis/torso), mirrored individually — `imu` merges their latest
    snapshots on demand instead of fabricating a second chained topic (X2 merges two
    30Hz callbacks into one publisher; here the IMU rates are low enough that the
    on-demand merge is simpler and avoids double-hop latency)."""

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("imu", "sensor", "骨盆+躯干 IMU 数据（pelvis/torso 最新快照）", {
            "type": "object",
            "properties": {},
        })

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            # Agent Core 用 info 推断可订阅主题：两路 IMU 流都要列出。
            return {"state": "running",
                    "topic_out": [
                        {"topic": self.nodes.streams[k]["topic"], "format": self.nodes.streams[k]["format"]}
                        for k in ("imu_pelvis", "imu_torso") if k in self.nodes.streams],
                    "streams": {k: self.nodes.streams[k]
                                for k in ("imu_pelvis", "imu_torso") if k in self.nodes.streams}}
        return {
            "pelvis": self.nodes.snapshot("imu_pelvis"),
            "torso": self.nodes.snapshot("imu_torso"),
        }


class CameraPlugin:
    """One card per configured camera stream; the tool name is camera_<key> minus the
    `camera_` prefix is NOT used — driver.yaml lists a single `camera` card, so all
    selected streams are multiplexed through this one tool via the `stream` param."""

    # Schema/dispatch speak the *unprefixed* stream name (as in config.yaml
    # plugins.camera.streams); internal self.streams keys keep the camera_ prefix.
    PREFIX = "camera_"

    def __init__(self, nodes):
        self.nodes = nodes
        self.streams = {k: v for k, v in nodes.streams.items() if k.startswith("camera_")}

    def _names(self):
        return [k[len(self.PREFIX):] for k in self.streams]

    def get_tool(self):
        return tool("camera", "sensor", "相机画面流（按 stream 参数选择；config.yaml plugins.camera.streams 决定可用路数）", {
            "type": "object",
            "properties": {
                "stream": {"type": "string", "enum": self._names(),
                           "description": "相机流 key（config.yaml plugins.camera.streams 中的名称）"},
            },
        })

    def start(self):
        pass

    def stop(self):
        pass

    def _resolve(self, args):
        name = args.get("stream") or next(iter(self.streams))[len(self.PREFIX):]
        key = f"{self.PREFIX}{name}"
        if key not in self.streams:
            raise ValueError(f"camera: unknown stream {name!r}; available: {self._names()}")
        return self.streams[key]

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            stream = self._resolve(args)
            return {"state": "running",
                    "topic_out": [{"topic": stream["topic"], "format": stream["format"]}],
                    "streams": {k: s for k, s in self.streams.items()}}
        return {"state": "running", **self._resolve(args)}


class LidarCloudPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return _stream_tool("lidar_cloud", self.nodes.streams["lidar_cloud"], "颈部 Livox 激光雷达点云")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            stream = self.nodes.streams["lidar_cloud"]
            return {"state": "running",
                    "topic_out": [{"topic": stream["topic"], "format": stream["format"]}]}
        return {"state": "running", **self.nodes.streams["lidar_cloud"]}


class BatteryPlugin:
    """RosMsgWrapper protobuf stream. With the a3_aimdk wheel the payload is decoded
    (bms_datas[1]=in-use pack, [0]=absent); without it the raw wrapper fields pass through."""

    def __init__(self, nodes):
        self.nodes = nodes
        self.has_stream = "battery" in nodes.streams

    def get_tool(self):
        if self.has_stream:
            return _stream_tool("battery", self.nodes.streams["battery"], "电池状态流（电压/电流/电量/充电状态，双电池包）")
        return tool("battery", "sensor", "电池状态（需要 a3_aimdk protobuf wheel 才能解码数据流）")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if self.has_stream:
            stream = self.nodes.streams["battery"]
            return {"state": "running",
                    "topic_out": [{"topic": stream["topic"], "format": stream["format"]}]}
        # a3_aimdk wheel 缺失 → 无可解码的数据流：显式声明不可用，而不是
        # 永远返回 running 的占位状态（Agent Core 会把 running 当作可订阅）。
        return {"state": "unavailable",
                "reason": "battery 数据流需要 a3_aimdk protobuf wheel；"
                          "在机器人上挂载 /agibot-devkit 开发包后重启驱动"}


class EstopPlugin:
    def __init__(self, nodes):
        self.nodes = nodes
        self.has_stream = "estop" in nodes.streams

    def get_tool(self):
        if self.has_stream:
            return _stream_tool("estop", self.nodes.streams["estop"],
                                "急停状态流（有线/无线/软件急停 + 各类传感器报警）")
        return tool("estop", "sensor", "急停状态（需要 a3_aimdk protobuf wheel 才能解码数据流）")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if self.has_stream:
            stream = self.nodes.streams["estop"]
            return {"state": "running",
                    "topic_out": [{"topic": stream["topic"], "format": stream["format"]}]}
        # 同 battery：降级时显式 unavailable，而不是伪装成 running。
        return {"state": "unavailable",
                "reason": "estop 数据流需要 a3_aimdk protobuf wheel；"
                          "在机器人上挂载 /agibot-devkit 开发包后重启驱动"}


class AlertsPlugin:
    """HDSService/GetAlertList — poll-on-demand sensor. Dev guide hard limit: poll
    frequency <= 0.2 Hz, enforced by a monotonic-clock cooldown between real RPCs."""

    def __init__(self, nodes):
        self.nodes = nodes
        interval = float(nodes.config.get("plugins", {}).get("alerts", {}).get("poll_interval", 5.0))
        self._cooldown = max(interval, 5.0)
        self._last_poll = 0.0
        self._cached = None

    def get_tool(self):
        return tool("alerts", "sensor", "查询 HDS 告警列表（GetAlertList；含告警码/等级/中英文描述，<=0.2Hz 限频）")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running"}
        now = time.monotonic()
        if self._cached is None or now - self._last_poll >= self._cooldown:
            response = self.nodes.rpc.get_alert_list()
            self._cached = (response.get("data") or {}).get("alerts", [])
            self._last_poll = now
        return {"alerts": self._cached}


class ModelPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("model", "resource", "返回 A3 Ultra 的 URDF 模型", {
            "type": "object",
            "properties": {},
        })

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        return {"urdf": self.nodes.urdf_text()}


# mc_mode fixed FSM (dev guide §7.1.2 + GetUp example flow). Current state → the
# SetAction commands allowed from it. The live source of truth is still
# GetAvailableActions (cross-checked at dispatch time); this map is the fixed
# doc-level skeleton that lets the driver reject impossible transitions with a
# suggestion instead of a failed RPC.
#   DAMPING / PASSIVE / LIE_DOWN / (unknown resting states) → only GET_UP
#   MOTION (standing after get_up) → DAMPING / LIE_DOWN / PASSIVE
MC_STATE_TRANSITIONS = {
    "MOTION": ("damping", "lie_down", "passive"),
    "DAMPING": ("get_up",),
    "PASSIVE": ("get_up",),
    "LIE_DOWN": ("get_up",),
    # IDLE/UNKNOWN and any unmapped state: allow all four, the runtime check decides
}
MC_TRANSITION_SUGGESTIONS = {
    "damping": "damping 用于关节卸力/跌倒保护，切换前确保机器人周围有足够空间；如当前未站立请先 get_up",
    "lie_down": "lie_down 仅可从 MOTION 站立状态进入；如需起身请先 get_up",
    "passive": "passive 拖动示教仅可从 MOTION 站立状态进入；退出示教请执行 get_up",
}


def _mc_allowed_actions(state: str):
    state = (state or "").strip().upper()
    if not state or state == "UNKNOWN":
        return tuple(MC_ACTIONS)
    # MOTION also accepts GET_UP as a no-op re-stand; resting states reject it too
    if state == "MOTION":
        return MC_STATE_TRANSITIONS["MOTION"] + ("get_up",)
    # IDLE and any unmapped state behave like a resting state: only get_up
    return MC_STATE_TRANSITIONS.get(state, ("get_up",))


def _mc_current_state(rpc):
    """GetAction → normalized current state string ('' when unparseable).

    Shared by McModePlugin and the MOTION-gate on loco/arm commands (5th PR
    review): the exact response field name is not in the dev guide, so probe
    action/state/current_action at the top level and under "data", plus
    command.action — '' means "could not tell", callers stay permissive then.
    """
    response = rpc.get_action()
    for container in (response, response.get("data") or {}):
        if isinstance(container, dict):
            for key in ("action", "state", "current_action"):
                value = container.get(key)
                if isinstance(value, str) and value:
                    return value.upper().replace("MOTIONCONTROLACTION_", "")
            command = container.get("command")
            if isinstance(command, dict):
                value = command.get("action")
                if isinstance(value, str) and value:
                    return value.upper().replace("MOTIONCONTROLACTION_", "")
    return ""


def _mc_suggestion(state: str, requested: str) -> str:
    state = (state or "").strip().upper() or "UNKNOWN"
    allowed = _mc_allowed_actions(state)
    if requested in allowed:
        return ""
    if "get_up" in allowed:
        return f"当前状态 {state} 仅允许 {list(allowed)}；建议先执行 get_up 恢复站立后再切换"
    return f"当前状态 {state} 仅允许 {list(allowed)}；建议先经 get_up 恢复站立（MOTION）后再进入 {requested}"


class McModePlugin:
    """mc_mode 卡片：运动控制状态机查询 + 模式切换（原 mc_state 状态卡并入）。

    对应 MDU MotionControlActionService：SetAction 切换 GetUp/LieDown/Damping/
    Passive（异步，返回 PENDING，结果经 get_state 轮询 GetAction 确认）；
    get_state / available 合并自原 mc_state 卡（GetAction / GetAvailableActions）。

    固定状态机（开发文档 §7.1.2 + GetUp 示例流程）：DAMPING/PASSIVE/LIE_DOWN/
    IDLE 等非站立态只能 get_up；MOTION（站立）可 damping/lie_down/passive。
    切换请求先与该固定迁移表核对，再与运行时 GetAvailableActions 交叉校验，
    不合法迁移直接拒绝并返回建议（不发起 RPC）。
    """

    ACTIONS = {
        name: ([], desc)
        for name, desc in (
            ("get_state", "查询当前运动控制动作状态（GetAction，异步切换结果确认）"),
            ("available", "查询当前可用动作列表（GetAvailableActions）"),
            ("damping", "进入 Damping 阻尼模式（关节卸力，用于软急停/跌倒保护）"),
            ("get_up", "执行 GetUp 起身动作，从坐/躺恢复到站立平衡（MOTION）"),
            ("lie_down", "执行 LieDown 坐/躺下动作（仅 MOTION 站立态可进入）"),
            ("passive", "进入 Passive 拖动示教模式（仅 MOTION 站立态可进入）"),
        )
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        # Deliberately NO x-resource (g1 switch_mode precedent): a posture transition
        # (get_up/lie_down/damping) moves the whole body, and nothing else should run
        # during one — undeclared means exclusive against everything, which is exactly
        # right here. Aborting a controlled get_up/lie_down partway is how a robot
        # falls over.
        return tool("mc_mode", "actuator", "运动控制状态机查询与模式切换（SetAction：get_up/lie_down/damping/passive；"
                                          "get_state/available 查询当前动作与可用动作；固定迁移：非站立态仅 get_up，"
                                          "MOTION 可 damping/lie_down/passive，非法迁移返回建议）",
                    action_schema(self.ACTIONS, {}))

    def start(self):
        pass

    def stop(self):
        pass

    def _current_state(self):
        """GetAction → normalized current state string ('' when unparseable)."""
        return _mc_current_state(self.nodes.rpc)

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "get_state":
            return jsonable(self.nodes.rpc.get_action())
        if action == "available":
            response = self.nodes.rpc.get_available_actions()
            return {"commands": response.get("commands", [])}
        if action not in MC_ACTIONS:
            raise ValueError(f"mc_mode: unknown action {action!r}")
        # 1) fixed doc-level FSM check — reject with a suggestion, no RPC
        current = self._current_state()
        normalized = current.upper().replace("MOTIONCONTROLACTION_", "")
        suggestion = _mc_suggestion(normalized, action)
        if suggestion:
            return {"state": "rejected", "current": normalized or "UNKNOWN",
                    "requested": action, "suggestion": suggestion,
                    "allowed": list(_mc_allowed_actions(normalized))}
        # 2) runtime cross-check — GetAvailableActions is the live source of truth
        service_action, short = MC_ACTIONS[action]
        runtime = self.nodes.rpc.get_available_actions()
        commands = runtime.get("commands") or []
        if commands and service_action not in commands:
            return {"state": "rejected", "current": normalized, "requested": action,
                    "suggestion": f"运行时可用动作列表不含 {service_action}（当前可用 {commands}）；"
                                  f"请先满足前置状态（通常为 get_up 站立）",
                    "available": commands}
        response = self.nodes.rpc.set_action(service_action, "")
        return {"requested": short, "current": normalized, "response": response}


class LocoPlugin:
    """loco 卡片：腿式底盘行走控制（命名对齐 noetix/bumi 的腿式 `loco`，与
    tianyi2.0/q5_bundle 轮式的 chassis_raw/base_drive 区分 —— A3 是双足人形）。

    对应 /motion/control/locomotion_velocity 话题（RosMsgWrapper + LocomotionVelocity
    消息）。forward/lateral 为归一化比例（对应最大 1.0 m/s），angular 对外用角度制
    deg/s（底层归一化，1.0 = 1 rad/s = 57.3 deg/s）。注意事项（开发文档 §7.3）：
      - 仅当运动控制处于 MOTION 状态时指令才生效；
      - 指令需以一定频率持续下发，停止行走时下发全零速度；
      - 大幅速度变化请分步过渡，避免急停/急转引起姿态失稳。

    长时间动作（走一个 duration）声明 ACP：驱动以 ~20 Hz 持续下发速度帧直到
    duration 结束，随后自动下发零速并回报 completed（q5_bundle base_drive 的
    duration worker 同款）；duration=-1 表示持续运动直到 stop，由 stop 以
    cancelled 结算。
    """

    # 20 Hz 重发满足“持续下发”要求（间隙 50 ms ≪ 官方示例的 100 ms）；停行走
    # 时零速帧连发 5 帧确保停稳。
    _PUBLISH_INTERVAL_S = 0.05
    _STOP_FRAMES = 5
    # duration 上界与 x-completion 超时一致（60 s；-1 持续模式同样受此兜底）。
    MAX_DURATION_S = 60.0
    COMPLETION_TIMEOUT_S = 60.0

    def __init__(self, nodes):
        self.nodes = nodes
        self._move_action_id = None
        # ThreadingHTTPServer serves MCP calls concurrently — the check-then-set
        # on _move_action_id (settle prior / arm new) must be atomic
        # (MotionPlayPlugin._play_lock 同款).
        self._move_lock = threading.Lock()

    def get_tool(self):
        schema = action_schema(
            {"walk": (["forward", "lateral", "angular", "duration"],
                      "按速度走行 duration 秒后自动停止（ACP 回报完成）；duration=-1 持续运动直到 stop"),
             "stop": ([], "立即下发零速并结算进行中的行走")},
            {
                "forward": {"type": "number", "description": "前进速度比例 [-1,1]，正为前进，负为后退", "default": 0.0},
                "lateral": {"type": "number", "description": "横移速度比例 [-1,1]，正为左移", "default": 0.0},
                "angular": {"type": "number",
                            "description": "旋转速度 (deg/s)，限速 ±57.3 deg/s（= 1 rad/s），正为逆时针",
                            "minimum": -57.3, "maximum": 57.3, "default": 0.0},
                "duration": {"type": "number",
                             "description": "行走持续时间 (秒)，范围 (0, 60]；-1 表示持续运动直到 stop",
                             "anyOf": [{"const": -1.0, "title": "持续运动（需 stop）"},
                                       {"minimum": 0.1, "maximum": 60.0}],
                             "default": -1.0},
            },
        )
        # 底盘行走 —— 与 controlled_spatial/auto_charging 同一通道（都驱动底盘）。
        schema["x-resource"] = "base"
        schema["x-completion"] = {"actions": ["walk"], "timeout": self.COMPLETION_TIMEOUT_S}
        return tool("loco", "actuator", "腿式底盘行走：forward/lateral ∈ [-1,1] 速度比例，angular 为 deg/s"
                                 "（限速 ±57.3），持续 duration 秒后自动停止；仅 MOTION 模式下生效",
                    schema)

    def start(self):
        pass

    def stop(self):
        with self._move_lock:
            prior = self._settle_active("cancelled", {"reason": "plugin_stopped"})
        if prior is not None:
            self._halt_motion()

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"state": "walking" if self._move_action_id else "idle"}
        if action in ("stop", "cancel"):
            return self._stop_walk()
        if action != "walk":
            raise ValueError(f"loco: unknown action {action!r}")
        forward = _clamp(args.get("forward", 0.0), -1.0, 1.0, "forward")
        lateral = _clamp(args.get("lateral", 0.0), -1.0, 1.0, "lateral")
        angular = _clamp(args.get("angular", 0.0), -57.3, 57.3, "angular")
        duration = args.get("duration", -1.0)
        try:
            duration = float(duration)
        except (TypeError, ValueError):
            raise ValueError("loco: duration must be a number")
        if duration != -1.0 and not 0.0 < duration <= self.MAX_DURATION_S:
            raise ValueError(f"loco: duration must be in (0, {self.MAX_DURATION_S:g}] or -1 for continuous")
        if forward == 0.0 and lateral == 0.0 and angular == 0.0:
            raise ValueError("loco: all velocities are zero — use stop to halt")
        # MOTION-state gate (dev guide §7.3: locomotion only takes effect in MOTION).
        # Unparseable state → permissive, matching mc_mode's UNKNOWN handling.
        current = _mc_current_state(self.nodes.rpc)
        if current and current != "MOTION":
            return {"state": "rejected", "current": current,
                    "suggestion": "loco 仅在 MOTION 站立状态生效；请先执行 mc_mode get_up 恢复站立"}
        # 新指令顶替旧指令：先结算旧的等待线程（发零速 + cancelled 回报），再武装新的。
        with self._move_lock:
            prior = self._settle_active("cancelled", {"reason": "replaced_by_new_command"})
        return self._arm_completion(forward, lateral, angular, duration)

    # -- 内部 ------------------------------------------------------------------

    def _velocity_payload(self, forward, lateral, angular_degps):
        return {
            "mode": "MotionControl_LocomotionMode_DEFAULT",
            "forward_velocity": forward,
            "lateral_velocity": lateral,
            # LocomotionVelocity.angular 为归一化比例（1.0 = 1 rad/s）。
            "angular_velocity": math.radians(angular_degps),
        }

    def _publish(self, payload):
        self.nodes.publish_wrapper("locomotion_pub", payload)

    def _settle_active(self, status, detail):
        """结算挂起的 ACP 等待线程（若有）。调用方必须持有 _move_lock：先清 id
        再发零速/回报，杜绝迟到回报。返回被结算的 action_id（None 表示无挂起），
        供调用方决定是否补发物理停障（零速帧在锁外重发有丢失风险）——本方法
        自己也会发一次零速，调用方锁外的重发仅是加固。"""
        action_id, self._move_action_id = self._move_action_id, None
        if action_id is not None:
            try:
                self._publish(self._velocity_payload(0.0, 0.0, 0.0))
            except Exception:
                pass  # 结算回报优先于停障帧；调用方在锁外重试
            _acp_notify(action_id, status, detail, "loco")
        return action_id

    def _halt_motion(self):
        """锁外连发零速帧确保停稳（发布通道瞬时故障时尽力而为）。"""
        for _ in range(self._STOP_FRAMES):
            try:
                self._publish(self._velocity_payload(0.0, 0.0, 0.0))
            except Exception:
                return
            time.sleep(self._PUBLISH_INTERVAL_S)

    def _stop_walk(self):
        with self._move_lock:
            prior = self._settle_active("cancelled", {"reason": "cancelled_by_request"})
            had_active = prior is not None
        if prior is not None:
            self._halt_motion()
        return {"state": "idle", "was_walking": had_active}

    def _arm_completion(self, forward, lateral, angular, duration):
        action_id = f"loco_walk_{uuid4().hex[:8]}"
        self._move_action_id = action_id
        threading.Thread(target=self._walk_worker,
                         args=(action_id, forward, lateral, angular, duration),
                         daemon=True, name=f"a3_loco_{action_id}").start()
        return {"state": "walking", "action_id": action_id,
                "forward": forward, "lateral": lateral, "angular_degps": angular,
                "duration": duration, "stops_automatically": duration != -1.0,
                "cancel_action": "stop" if duration == -1.0 else None}

    def _walk_worker(self, action_id, forward, lateral, angular, duration):
        """持续下发速度帧直到 duration 结束（或被新指令/stop 顶替），然后停并回报。"""
        payload = self._velocity_payload(forward, lateral, angular)
        self._publish(payload)
        deadline = time.monotonic() + (duration if duration > 0 else self.COMPLETION_TIMEOUT_S)
        while time.monotonic() < deadline:
            time.sleep(self._PUBLISH_INTERVAL_S)
            with self._move_lock:
                if self._move_action_id != action_id:   # superseded / stopped
                    return
            try:
                self._publish(payload)
            except Exception as exc:                # 发布通道挂了：立即停下并报错
                with self._move_lock:
                    self._settle_active("error", {"error": f"publish failed: {exc}"})
                return
        # 正常到点：原子地清 id 并认领终态（与 stop/顶替路径互斥，杜绝双回报）。
        with self._move_lock:
            if self._move_action_id != action_id:
                return
            self._move_action_id = None
        # 连发零速帧、回报 completed（锁外：纯输出，无状态竞争）。
        for _ in range(self._STOP_FRAMES):
            try:
                self._publish(self._velocity_payload(0.0, 0.0, 0.0))
            except Exception:
                break
            time.sleep(self._PUBLISH_INTERVAL_S)
        _acp_notify(action_id, "completed",
                    {"forward": forward, "lateral": lateral, "angular_degps": angular,
                     "duration": duration, "stopped_automatically": True},
                    "loco")


class ArmControlPlugin:
    """arm_control 卡片：左/右臂 14 关节位置控制 + 手臂柔顺开关（命名对齐
    tianyi2.0/q5_bundle 的 arm_control；arm_compliance 卡并入 —— 同属 MDU
    运动控制平面）。

    对应 /motion/control/arm_joint_command 话题（sensor_msgs/JointState）+
    MotionControlMotionService/{Enable,Disable,Check}ArmCompliance RPC。
    开发文档 §7.3 硬性要求：
      - 需以 100 Hz 持续下发，指令间隔 ≤ 30 ms，否则机械臂将回到阻尼状态；
      - 速度、力矩字段必须为 0（底层按位置插值规划）；
      - 关节速度上限 4 rad/s；
      - 仅 MOTION 状态下可用；直接控臂前必须先停止 motion_player（见 motion_play 卡片备注）。
    单次调用只发送一帧指令；维持时间由调用方循环下发实现（上层 Agent 按运动规划循环）。
    柔顺模式下手臂可被外力拖动（示教/人机交互安全），关闭后恢复刚度控制。
    """

    ACTIONS = {
        "compliance_enable": ([], "开启手臂柔顺模式（可被外力拖动，示教用）"),
        "compliance_disable": ([], "关闭手臂柔顺模式（恢复刚度控制）"),
        "compliance_check": ([], "查询手臂柔顺模式是否开启"),
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {
                "send": (["left", "right"], "下发一帧手臂关节位置指令（rad），需按 ~100Hz 循环调用"),
                **self.ACTIONS,
            },
            {
                "left": {"type": "array", "items": {"type": "number"},
                         "description": "左臂 7 关节 rad，顺序：shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw"},
                "right": {"type": "array", "items": {"type": "number"},
                          "description": "右臂 7 关节 rad，顺序同左臂"},
                "duration_ms": {"type": "integer", "description": "保持时长（毫秒），期间以 100Hz 重复下发同一帧指令", "default": 100},
            },
        )
        # 双臂关节 —— side 按调用变化而 schema 是静态的，只声明一侧会让双臂动作
        # 与单臂动作并发抢同一批关节（tianyi arm 同款结论）。
        schema["x-resource"] = ["arm_l", "arm_r"]
        return tool("arm_control", "actuator", "下发双臂 14 关节位置指令 + 手臂柔顺控制"
                                              "（话题 /motion/control/arm_joint_command 需 100Hz 连续下发、间隔 ≤30ms；"
                                              "velocity/effort 固定 0；柔顺 Enable/Disable/CheckArmCompliance RPC；"
                                              "先经 mc_mode get_up 站立并停止 motion_player）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action in self.ACTIONS:
            method = action.replace("compliance_", "")
            return jsonable(self.nodes.rpc.arm_compliance(method))
        if action != "send":
            raise ValueError(f"arm_control: unknown action {action!r}")
        # MOTION-state gate (dev guide §7.3: arm control only takes effect in MOTION;
        # the docs also require stopping motion_player first). Unparseable state →
        # permissive, matching mc_mode's UNKNOWN handling.
        current = _mc_current_state(self.nodes.rpc)
        if current and current != "MOTION":
            return {"state": "rejected", "current": current,
                    "suggestion": "arm_control 仅在 MOTION 站立状态生效（且需先停止 motion_player）；"
                                  "请先执行 mc_mode get_up 恢复站立"}
        positions = {}
        for side in ("left", "right"):
            values = args.get(side)
            if values is None:
                continue
            _require(len(values) == 7, f"{side} 臂需要 7 个关节值，收到 {len(values)} 个")
            for joint, value in zip(ARM_JOINTS[side], values):
                positions[joint] = float(value)
        _require(positions, "至少提供 left 或 right 关节位置")
        _check_joint_limits(positions, ARM_JOINT_LIMITS)
        duration_ms = int(args.get("duration_ms", 100))
        _require(duration_ms >= 0, "duration_ms 不能为负")
        self.nodes.publish_joint_command("arm_command_pub", positions, duration_ms)
        return {"joints": positions, "duration_ms": duration_ms, "state": "published"}


class HandControlPlugin:
    """hand_control 卡片：灵巧手张合控制（命名对齐 tianyi2.0/q5_bundle）。

    对应 /motion/control/hand_joint_command 话题（sensor_msgs/JointState）。
    frame_id 标识手部类型：AgiHand（默认）或 O10Hand，当前安装类型可从
    joints 卡片（group=hand）的 frame_id 读取。position 为 0~2000 的张合等级。
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {"send": (["left", "right"], "下发双手张合等级 0(张开)~2000(握紧)")},
            {
                "left": {"type": "array", "items": {"type": "number"}, "description": "左手指张合等级列表 0~2000"},
                "right": {"type": "array", "items": {"type": "number"}, "description": "右手指张合等级列表 0~2000"},
                "hand_type": {"type": "string", "enum": ["AgiHand", "O10Hand"], "default": "AgiHand"},
            },
        )
        # 手指关节 —— 与手臂是独立自由度，可以同时动（tianyi hand 同款结论）。
        schema["x-resource"] = ["hand_l", "hand_r"]
        return tool("hand_control", "actuator", "下发灵巧手张合指令（话题 /motion/control/hand_joint_command，"
                                                "position 0~2000，frame_id 区分 AgiHand/O10Hand）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action != "send":
            raise ValueError(f"hand_control: unknown action {action!r}")
        hand_type = args.get("hand_type", "AgiHand")
        _require(hand_type in HAND_TYPES, f"未知手部类型 {hand_type!r}，可选 {sorted(HAND_TYPES)}")
        positions = {}
        for side in ("left", "right"):
            values = args.get(side)
            if values is None:
                continue
            for i, value in enumerate(values):
                positions[f"{side}_hand_joint_{i}"] = _clamp(float(value), 0.0, HAND_COMMAND_MAX, f"{side} 手指 {i}")
        _require(positions, "至少提供 left 或 right 张合等级")
        self.nodes.publish_joint_command("hand_command_pub", positions, frame_id=hand_type)
        return {"hand_type": hand_type, "positions": positions, "state": "published"}


class HeadControlPlugin:
    """head_control 卡片：头部双关节控制（命名对齐 tianyi2.0/q5_bundle）。

    对应 /motion/control/neck_joint_command 话题（sensor_msgs/JointState）。
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {"send": (["yaw", "pitch"], "下发一帧头部关节指令（rad）")},
            {
                "yaw": {"type": "number", "description": "头部偏航角 rad（左正右负）"},
                "pitch": {"type": "number", "description": "头部俯仰角 rad（抬头为正）"},
                "duration_ms": {"type": "integer", "description": "保持时长（毫秒）", "default": 100},
            },
        )
        # 头部双关节 —— 相机云台同源，与 face_play 共用 head 通道。
        schema["x-resource"] = "head"
        return tool("head_control", "actuator", "下发头部姿态指令（话题 /motion/control/neck_joint_command，"
                                                "head_yaw ∈ [-1.047,1.047] rad，head_pitch ∈ [-0.436,0.262] rad）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action != "send":
            raise ValueError(f"head_control: unknown action {action!r}")
        positions = {}
        if args.get("yaw") is not None:
            positions["head_yaw_joint"] = float(args["yaw"])
        if args.get("pitch") is not None:
            positions["head_pitch_joint"] = float(args["pitch"])
        _require(positions, "至少提供 yaw 或 pitch")
        _check_joint_limits(positions, NECK_LIMITS)
        duration_ms = int(args.get("duration_ms", 100))
        _require(duration_ms >= 0, "duration_ms 不能为负")
        self.nodes.publish_joint_command("neck_command_pub", positions, duration_ms)
        return {"joints": positions, "duration_ms": duration_ms, "state": "published"}


class WaistControlPlugin:
    """waist_control 卡片：腰部三自由度控制（命名对齐 tianyi2.0/q5_bundle）。

    对应 /motion/control/move_waist 话题（RosMsgWrapper + MoveWaist 消息）：
    pitch（前倾后仰）、yaw（左右旋转）、height（升降，0 为最低位）。
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {"send": (["pitch", "yaw", "height"], "下发腰部俯仰/旋转/升降指令")},
            {
                "pitch": {"type": "number", "description": "腰部俯仰 rad，正为前倾"},
                "yaw": {"type": "number", "description": "腰部偏航 rad，正为左转"},
                "height": {"type": "number", "description": "腰部高度偏移 m（-0.3~0，0 为站立基准）"},
            },
        )
        # 腰部三自由度 —— 没有别的工具碰这三个自由度。
        schema["x-resource"] = "waist"
        return tool("waist_control", "actuator", "下发腰部控制指令（话题 /motion/control/move_waist："
                                                  "pitch/yaw ∈ [-1.6,1.6] rad，height ∈ [-0.3,0] m）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action != "send":
            raise ValueError(f"waist_control: unknown action {action!r}")
        payload = {}
        for field in ("pitch", "yaw", "height"):
            value = args.get(field)
            if value is None:
                continue
            low, high = WAIST_LIMITS[f"waist_{field}"]
            payload[f"waist_{field}"] = _clamp(float(value), low, high, field)
        _require(payload, "至少提供 pitch/yaw/height 之一")
        self.nodes.publish_wrapper("waist_pub", payload)
        return {**payload, "state": "published"}


class MotionPlayPlugin:
    """motion_play 卡片：动作文件播放控制（示教式回放）+ 动作资源列表。

    对应 MDU MotionCommandService/SendMotionCommand + HDU ResourceService/
    GetResourceList（motion 类）。注意事项（开发文档 §7.4）：
      - 播放动作前若 motion_player 在运行，必须先停止：
        登录 MDU 执行 curl -X POST http://127.0.0.1:50080/json/stop_app -d '{"app_name":"motion_player"}'
      - 仅 MOTION 状态下可下发；
      - 同一 motion_id 可重复下发实现连续播放，end 指令提前结束。
    """

    ACTIONS = {
        "list": ([], "列出可用 motion 动作资源（GetResourceList motion 类）"),
        "play": (["motion_id", "duration_ms"], "播放指定动作（motion_id 为 motion 资源文件绝对路径）"),
        "pause": ([], "暂停当前动作播放"),
        "stop_play": ([], "结束当前动作并自动恢复初始姿态（cmd_end）"),
        "reset": ([], "立即中止当前动作并恢复初始姿态（cmd_reset）"),
        "resume": ([], "恢复播放（撤销 pause）"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self._play_action_id = None
        # ThreadingHTTPServer serves MCP calls concurrently — the check-then-set
        # on _play_action_id (settle prior / arm new) must be atomic.
        self._play_lock = threading.Lock()
        # pause suspends the completion countdown (robot-side player is frozen
        # by cmd_pause, so the estimated duration no longer elapses in real time).
        self._paused = threading.Event()

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "motion_id": {"type": "string",
                          "description": "动作文件绝对路径（list → motion 取 path 字段；"
                                         "SendMotionCommand 要求绝对路径）"},
            "duration_ms": {"type": "integer",
                            "description": f"播放时长 ms（必填正数，"
                                           f"1 ≤ duration_ms ≤ {MOTION_PLAY_MAX_DURATION_MS}）"},
        })
        # A3 exposes no motion-status topic/RPC — completion is duration-based:
        # a daemon worker sleeps duration_ms then POSTs /api/acp/complete.
        schema["x-completion"] = {"actions": ["play"], "timeout": 600}
        # 动作文件驱动全身（底盘+双臂+手+腰+头），声明全通道列表让 ACP 排他仲裁。
        schema["x-resource"] = ["base", "arm_l", "arm_r", "hand_l", "hand_r",
                                "waist", "head"]
        return tool("motion_play", "actuator", "播放/暂停/停止动作文件 + 动作资源列表"
                                              "（SendMotionCommand + GetResourceList RPC；播放前先停止 MDU 上"
                                              "的 motion_player 应用，且需 MOTION 模式；播放为异步："
                                              "按 duration_ms 估时回报 ACP 完成事件）",
                    schema)

    def start(self):
        pass

    def _settle_active(self, status, detail):
        """结算挂起的 ACP 等待线程（若有）：清 id 后立刻回报终态。

        只清 id 会让 Agent Core 的 barrier 挂到 600 s 超时 —— 取消也必须显式
        POST cancelled（LocoPlugin._settle_active 同款语义）。调用方须持有
        _play_lock（除插件生命周期 stop() 自行加锁外）。
        """
        action_id, self._play_action_id = self._play_action_id, None
        if action_id is not None:
            _acp_notify(action_id, status, detail, "motion_play")

    def stop(self):
        # 插件生命周期卸载：与框架 stop 同款 —— 有动作在跑就先物理停掉并结算，
        # 避免卸载后悬挂的 worker 再 POST 幽灵 completed（LocoPlugin.stop 先例）。
        with self._play_lock:
            active = self._play_action_id is not None
            if active:
                self._settle_active("cancelled", {"reason": "plugin_stopped"})
        if active:
            try:
                self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_end=True)
            except Exception:
                pass

    _PLAY_WORKER_TICK_S = 0.2

    def _play_worker(self, action_id, motion_path, duration_ms):
        # Settle when the originally requested duration elapses; a pause suspends
        # the countdown (cmd_pause freezes the robot-side player, so wall-clock
        # time no longer maps to playback progress), a stop/reset invalidates
        # the waiter entirely. The terminal post is capture-and-clear under the
        # lock so pause/resume/stop racing the natural finish never double-report.
        remaining_s = duration_ms / 1000.0
        while remaining_s > 0:
            self._paused.wait(self._PLAY_WORKER_TICK_S)   # paused → clock frozen
            if self._play_action_id != action_id:
                return  # superseded by a new play, or settled by stop/reset
            if self._paused.is_set():
                continue  # frozen this tick — do not consume the countdown
            remaining_s -= self._PLAY_WORKER_TICK_S
        if self._play_action_id != action_id:
            return
        _acp_notify(action_id, "completed",
                    {"motion_id": motion_path, "duration_ms": duration_ms},
                    "motion_play")

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"state": "ready"}
        if action == "list":
            response = self.nodes.rpc.resource_list("motion")
            return {"resources": (response.get("data") or {}).get("resources", [])}
        if action == "play":
            # SendMotionCommand 入参为动作文件绝对路径 + 动作最长运行毫秒数；
            # cmd_end=true 时播放完自动回初始姿态（默认 True）。
            # A3 无动作状态反馈通道，异步完成按 duration_ms 估时回报（ACP）。
            motion_path = str(args.get("motion_id", ""))
            _require(motion_path, "motion_id 不能为空")
            duration_ms = int(args.get("duration_ms", 10000))
            # Bounds must match the declared x-completion timeout (600 s): a
            # negative or zero value would report completion instantly while the
            # RPC still runs — with no motion-status feedback channel the A3
            # cannot observe an open-ended playback, so a positive bound is
            # required (9th PR review); an over-timeout value would have Agent
            # Core give up while the robot is still moving.
            _require(1 <= duration_ms <= MOTION_PLAY_MAX_DURATION_MS,
                     f"duration_ms={duration_ms} 超出范围 [1, {MOTION_PLAY_MAX_DURATION_MS}]"
                     f"（必须为正数：0/缺省时长无法被驱动观测，会让 Agent Core 提前"
                     f"释放全身资源屏障）")
            # MOTION-state gate (dev guide §7.1: motion_player is a MOTION-mode
            # application; full-body motion while DAMPING/PASSIVE/LIE_DOWN would
            # command joints that are unpowered). Unparseable state → permissive,
            # matching the loco/arm gates.
            current = _mc_current_state(self.nodes.rpc)
            if current and current != "MOTION":
                return {"state": "rejected", "current": current,
                        "suggestion": "motion_play 仅在 MOTION 站立状态生效；"
                                      "请先执行 mc_mode get_up 恢复站立"}
            # Atomic settle-then-arm: a second concurrent play must cancel the
            # first (cmd_end + immediate ACP cancelled) instead of silently
            # orphaning its barrier — the superseded worker would otherwise
            # exit on the id mismatch and leave Agent Core pending 600 s
            # (ThreadingHTTPServer makes concurrent play reachable). The prior
            # motion is ended BEFORE the new SendMotionCommand fires so the
            # robot never receives two overlapping plays.
            with self._play_lock:
                if self._play_action_id is not None:
                    self._settle_active("cancelled", {"reason": "replaced_by_new_play"})
                    try:
                        self.nodes.rpc.send_motion_command(
                            motion_id="", duration_ms=0, cmd_end=True)
                    except Exception:
                        pass
            response = jsonable(self.nodes.rpc.send_motion_command(
                motion_path, duration_ms, cmd_end=True, cmd_pause=False))
            with self._play_lock:
                action_id = f"motion_play_{uuid4().hex[:8]}"
                self._play_action_id = action_id
            threading.Thread(target=self._play_worker,
                             args=(action_id, motion_path, duration_ms),
                             daemon=True).start()
            return {"state": "playing", "action_id": action_id,
                    "motion_id": motion_path, "duration_ms": duration_ms,
                    "response": response}
        if action == "pause":
            # cmd_pause freezes the robot-side player; the completion waiter
            # suspends its countdown too (resume continues the same motion) —
            # otherwise the worker would report completed while the robot is
            # mid-pose and a resume would move joints with no ACP armed.
            self._paused.set()
            return jsonable(self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_pause=True))
        if action == "resume":
            self._paused.clear()
            return jsonable(self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_pause=False))
        if action == "stop_play":
            # Stopping the motion also settles any pending ACP waiter — a
            # cancelled motion must not be reported as completed later, and
            # Agent Core must hear the cancellation now (not at its timeout).
            with self._play_lock:
                self._settle_active("cancelled", {"reason": "cancelled_by_request"})
            return jsonable(self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_end=True))
        if action == "reset":
            with self._play_lock:
                self._settle_active("cancelled", {"reason": "reset"})
            return jsonable(self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_reset=True))
        if action == "stop":
            # Framework stop must physically halt the motion — a bare "idle"
            # acknowledgement would leave the robot moving and the ACP worker
            # free to POST a phantom completed event later. With nothing armed
            # the call stays inert (canvas lifecycle toggles must not spam
            # cancellation RPCs when no motion is running).
            with self._play_lock:
                if self._play_action_id is None:
                    return {"state": "idle"}
                self._settle_active("cancelled", {"reason": "cancelled_by_framework"})
            response = jsonable(self.nodes.rpc.send_motion_command(motion_id="", duration_ms=0, cmd_end=True))
            return {"state": "stopped", "response": response}
        raise ValueError(f"motion_play: unknown action {action!r}")


class TtsPlugin:
    """tts 卡片：语音播报一体化（原 media_play 卡并入 —— 同为 HDU TTSService）。

      - speak：PlayTTS 文本转语音（text ≤ 1024 字节，priority_level 分
        BACKGROUND_L1 / SERVICE_L2 / INTERACTION_L6，is_interrupted 可打断）；
      - play_media：PlayMediaFile 播放音频/视频文件；
      - status：GetAudioStatus 按 trace_id 查询播报状态
        （0 未播 / 1 播报中 / 2 播报完成 / 3 异常）；
      - stop_trace_id：StopTTSByTraceId 按 trace_id 打断。

    speak / play_media 为长时间动作（tianyi2.0 tts 同款）：声明 ACP 完成契约，
    后台线程轮询 GetAudioStatus（状态 2 完成 / 3 异常）判定播报真正结束后回报。
    """

    ACTIONS = {
        "speak": (["text"], "播报一段中文/英文文本（异步：返回 action_id，完成后回报 ACP）"),
        "play_media": (["file_name"], "播放媒体文件（audio 资源文件名，含扩展名；"
                                       "异步：返回 action_id，完成后回报 ACP）"),
        "status": (["trace_id"], "查询播报状态（0 未播/1 播报中/2 播报完成/3 异常）"),
        "stop_trace_id": (["trace_id"], "按 trace_id 打断指定播报"),
    }

    _POLL_INTERVAL_S = 1.0

    def __init__(self, nodes):
        self.nodes = nodes
        self._play_action_id = None
        self._play_trace_id = None
        # ThreadingHTTPServer serves MCP calls concurrently — the check-then-set on
        # _play_action_id / _play_trace_id must be atomic (MotionPlayPlugin 同款).
        self._play_lock = threading.Lock()

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "text": {"type": "string", "description": "播报文本（UTF-8，≤1024 字节）"},
            "priority_level": {"type": "string",
                               "enum": ["BACKGROUND_L1", "SERVICE_L2", "INTERACTION_L6"],
                               "default": "INTERACTION_L6",
                               "description": "播报优先级，INTERACTION_L6 默认可打断低优先级"},
            "is_interrupted": {"type": "boolean", "default": True, "description": "是否打断当前播报"},
            "trace_id": {"type": "string", "description": "播报 id（可选自定义传入，用于状态查询与打断）"},
            "file_name": {"type": "string", "description": "媒体文件名（audio 卡片 list → audio）"},
        })
        # 与 audio 卡共用同一扬声器 —— 同一物理通道，两个工具需互相排队。
        schema["x-resource"] = "mouth"
        # 长播报动不出数秒不归：声明 ACP（tianyi2.0 tts 180 s 同款）。
        schema["x-completion"] = {"actions": ["speak", "play_media"],
                                  "timeout": TTS_COMPLETION_TIMEOUT_S}
        return tool("tts", "actuator", "语音播报：文本转语音/媒体文件播放/状态查询/按 id 打断"
                                      "（TTSService PlayTTS/PlayMediaFile/GetAudioStatus/StopTTSByTraceId RPC；"
                                      "speak/play_media 为异步：返回 action_id，轮询 GetAudioStatus "
                                      "播报完成后回报 ACP 完成事件）",
                    schema)

    def start(self):
        pass

    def _settle_active(self, status, detail):
        """结算挂起的 ACP 等待线程（若有）：清 id 后立刻回报终态。

        调用方必须持有 _play_lock。返回被结算的 (action_id, trace_id)
        （None 表示无挂起），供调用方在锁外执行物理打断 —— StopTTSTraceId
        只对旧 trace_id 有效，必须在被新播报覆盖前捕获。

        只清 id 会让 Agent Core 的 barrier 挂到 180 s 超时 —— 取消也必须显式
        POST cancelled（LocoPlugin._settle_active 同款语义）。
        """
        action_id, self._play_action_id = self._play_action_id, None
        if action_id is not None:
            _acp_notify(action_id, status, detail, "tts")
        return (action_id, self._play_trace_id) if action_id is not None else None

    def stop(self):
        # 插件生命周期卸载：与框架 stop 同款 —— 有播报在跑就先物理打断并结算，
        # 避免卸载后悬挂的 worker 再 POST 幽灵 completed（LocoPlugin.stop 先例）。
        with self._play_lock:
            prior = self._settle_active("cancelled", {"reason": "plugin_stopped",
                                                      "trace_id": self._play_trace_id or ""})
        if prior is not None:
            try:
                self.nodes.rpc.stop_tts_trace_id(prior[1] or "")
            except Exception:
                pass

    def _audio_state(self, response):
        """GetAudioStatus response → normalized play state int, or None.

        The response shape could not be verified against the online dev guide, so
        this accepts a top-level or `data`-nested `state`/`status`/`audio_status`
        int (or enum name string: 未播/播报中/完成/异常 → 0..3). Unrecognized
        shapes count as still-playing — the timeout is the backstop.
        """
        if not isinstance(response, dict):
            return None
        _NAMES = {"not_played": 0, "not_started": 0, "idle": 0,
                  "playing": TTS_AUDIO_PLAYING, "broadcasting": TTS_AUDIO_PLAYING,
                  "finished": TTS_AUDIO_DONE, "complete": TTS_AUDIO_DONE,
                  "completed": TTS_AUDIO_DONE, "done": TTS_AUDIO_DONE,
                  "error": TTS_AUDIO_ERROR, "failed": TTS_AUDIO_ERROR,
                  "abnormal": TTS_AUDIO_ERROR}
        scopes = [response]
        if isinstance(response.get("data"), dict):
            scopes.append(response["data"])
        for scope in scopes:
            for key in ("state", "status", "audio_status"):
                value = scope.get(key)
                if isinstance(value, bool):
                    continue
                if isinstance(value, int) and 0 <= value <= 3:
                    return value
                if isinstance(value, str):
                    return _NAMES.get(value.strip().lower())
        return None

    def _play_worker(self, action_id, trace_id, action, detail):
        # Superseded guard: a newer speak/play_media or a stop took over — never
        # fire a stale completion (g1 _acp_wait_nav pattern). The terminal posts
        # capture-and-clear the id under the lock so a concurrent cancel and a
        # natural finish never double-report.
        with self._play_lock:
            if self._play_action_id != action_id:
                return
        deadline = time.time() + TTS_COMPLETION_TIMEOUT_S
        result = {"action": action, "trace_id": trace_id, **detail}
        while time.time() < deadline:
            time.sleep(self._POLL_INTERVAL_S)
            with self._play_lock:
                if self._play_action_id != action_id:
                    return
            try:
                response = self.nodes.rpc.get_audio_status(trace_id)
            except Exception as exc:  # noqa: BLE001 — notify and keep polling
                result["error"] = f"GetAudioStatus poll failed: {exc}"
                self._finish(action_id, "error", result)
                return
            state = self._audio_state(response)
            # state 0（未播）也按进行中处理：PlayTTS 刚受理、状态尚未翻转时
            # 轮询到 0 是正常竞态，误报 error 会让 Agent Core 提前收尸。
            if state is None or state == TTS_AUDIO_PLAYING or state == 0:
                continue
            if state == TTS_AUDIO_DONE:
                result["final_state"] = state
                self._finish(action_id, "completed", result)
            else:
                result["error"] = f"playback ended in state {state}"
                result["final_state"] = state
                self._finish(action_id, "error", result)
            return
        result["error"] = f"playback not finished within {TTS_COMPLETION_TIMEOUT_S}s"
        self._finish(action_id, "error", result)

    def _finish(self, action_id, status, result):
        """原子终态：锁内确认仍持有该 action_id 才清 id，锁外回报。"""
        with self._play_lock:
            if self._play_action_id != action_id:
                return  # 已被取消/顶替，终态由对方回报
            self._play_action_id = None
        _acp_notify(action_id, status, result, "tts")

    def _arm_completion(self, action, trace_id, detail):
        action_id = f"tts_{action}_{uuid4().hex[:8]}"
        with self._play_lock:
            self._play_action_id = action_id
            # 记住当前 trace_id：框架 stop 打断当前播报时要按它调用
            # StopTTSTraceId，而 stop 入参里并不会带 trace_id。
            self._play_trace_id = trace_id
        threading.Thread(target=self._play_worker,
                         args=(action_id, trace_id, action, detail),
                         daemon=True).start()
        return {"state": "playing", "action_id": action_id,
                "trace_id": trace_id, **detail}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            # Framework stop must physically silence the audio — otherwise the
            # voice keeps playing and the ACP worker reports completion later.
            # With nothing armed the call stays inert (canvas lifecycle toggles
            # must not spam StopTTSTraceId when nothing is playing).
            with self._play_lock:
                if self._play_action_id is None:
                    return {"state": "idle"}
                prior = self._settle_active(
                    "cancelled", {"reason": "cancelled_by_framework",
                                  "trace_id": self._play_trace_id or ""})
                stop_trace = prior[1] or ""
            response = jsonable(self.nodes.rpc.stop_tts_trace_id(stop_trace))
            return {"state": "stopped", "response": response}
        if action == "info":
            return {"state": "ready"}
        if action == "speak":
            text = args.get("text", "")
            _require(len(text.encode("utf-8")) <= TTS_MAX_TEXT_BYTES,
                     f"播报文本超长（>{TTS_MAX_TEXT_BYTES} 字节）")
            priority = args.get("priority_level", "INTERACTION_L6")
            _require(priority in TTS_PRIORITY_LEVELS.values() or priority in TTS_PRIORITY_LEVELS,
                     f"未知优先级 {priority!r}")
            response = self.nodes.rpc.play_tts(
                text,
                priority_level=priority if priority in TTS_PRIORITY_LEVELS.values() else TTS_PRIORITY_LEVELS[priority],
                is_interrupted=bool(args.get("is_interrupted", True)),
                trace_id=str(args.get("trace_id") or ""),
            )
            # PlayTTS/PlayMediaFile 出参为扁平结构（is_sucess 官方拼写如此）
            trace_id = response.get("trace_id", "") or (response.get("data") or {}).get("trace_id", "")
            # 新播报顶替旧播报：PlayTTS 已受理后先结算旧等待线程（立即 cancelled）
            # 并在锁外用旧 trace_id 物理打断 —— 顺序不能反，否则打断的是新播报。
            with self._play_lock:
                prior = self._settle_active("cancelled", {"reason": "replaced_by_new_playback"})
            if prior is not None and prior[1]:
                try:
                    self.nodes.rpc.stop_tts_trace_id(prior[1])
                except Exception:
                    pass
            # 播报无独立 trace_id 时轮询不到指定会话，等待线程只能靠超时兜底；
            # 此时用空 trace_id 询问当前播报状态（GetAudioStatus 单会话语义）。
            return self._arm_completion("speak", trace_id, {"text": text[:50]})
        if action == "play_media":
            file_name = args.get("file_name", "")
            _require(file_name, "file_name 不能为空")
            response = self.nodes.rpc.play_media_file(file_name, is_interrupted=True)
            trace_id = response.get("trace_id", "") or (response.get("data") or {}).get("trace_id", "")
            # 同 speak：先结算并打断旧播报，再武装新的等待线程。
            with self._play_lock:
                prior = self._settle_active("cancelled", {"reason": "replaced_by_new_playback"})
            if prior is not None and prior[1]:
                try:
                    self.nodes.rpc.stop_tts_trace_id(prior[1])
                except Exception:
                    pass
            return self._arm_completion("play_media", trace_id,
                                        {"file_name": file_name, "response": response})
        if action == "status":
            return jsonable(self.nodes.rpc.get_audio_status(args.get("trace_id", "")))
        if action == "stop_trace_id":
            # 打断播报同时结算挂起的 ACP 等待线程（立刻回报 cancelled，
            # 不让 Agent Core 的 barrier 挂到超时）。
            with self._play_lock:
                self._settle_active("cancelled", {"reason": "cancelled_by_request",
                                                  "trace_id": args.get("trace_id", "")})
            return jsonable(self.nodes.rpc.stop_tts_trace_id(args.get("trace_id", "")))
        raise ValueError(f"tts: unknown action {action!r}")


class AudioPlugin:
    """audio 卡片：音频播放 + 音量控制一体化（原 audio_play/volume 两卡并入 —
    同为 HDU HalAudioService）。

      - play / stop_play：PlayFile 播放音频文件（DEFAULT 优先级）/ StopPlay 停止；
      - get_volume / set_volume / mute / unmute：GetAudioVolume / SetAudioVolume。
    开发文档硬性限制：音量 >70 有损坏硬件风险，本插件在 RPC 之外再次钳制
    max_volume（config plugins.audio.max_volume，默认 70）。
    """

    ACTIONS = {
        "list": ([], "列出可用 audio 音频资源（GetResourceList audio 类）"),
        "play": (["file_name"], "播放音频文件（audio 资源文件名）"),
        "stop_play": ([], "停止当前正在播放的音频（HalAudioService/StopPlay）"),
        "get_volume": ([], "查询当前音量（返回 is_sucess —— 官方接口拼写如此）"),
        "set_volume": (["volume"], f"设置音量 0~{VOLUME_HARD_MAX}"),
        "mute": ([], "静音（is_mute=true）"),
        "unmute": ([], "取消静音（is_mute=false）"),
    }

    def __init__(self, nodes, max_volume=VOLUME_HARD_MAX):
        self.nodes = nodes
        self.max_volume = min(int(max_volume), VOLUME_HARD_MAX)

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "file_name": {"type": "string", "description": "音频文件名（list → audio）"},
            "volume": {"type": "integer", "minimum": 0, "maximum": self.max_volume,
                       "description": f"目标音量 0~{self.max_volume}"},
        })
        # 与 tts 卡共用同一扬声器 —— 同一物理通道，两个工具需互相排队。
        schema["x-resource"] = "mouth"
        return tool("audio", "actuator", f"音频播放与音量控制 + 音频资源列表（HalAudioService/ResourceService RPC；"
                                          f"音量 0~{self.max_volume}，硬件上限 {VOLUME_HARD_MAX}，超限有损坏风险）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "max_volume": self.max_volume}
        if action == "list":
            response = self.nodes.rpc.resource_list("audio")
            return {"resources": (response.get("data") or {}).get("resources", [])}
        if action == "play":
            file_name = args.get("file_name", "")
            _require(file_name, "file_name 不能为空")
            return jsonable(self.nodes.rpc.play_file(file_name))
        if action == "stop_play":
            return jsonable(self.nodes.rpc.stop_play())
        if action == "get_volume":
            return jsonable(self.nodes.rpc.get_audio_volume())
        if action == "set_volume":
            volume = int(args.get("volume", 0))
            _require(0 <= volume <= self.max_volume,
                     f"音量必须在 0~{self.max_volume} 之间（开发文档限制 >70 有硬件损坏风险）")
            return jsonable(self.nodes.rpc.set_audio_volume(volume, is_mute=False))
        if action == "mute":
            return jsonable(self.nodes.rpc.set_audio_volume(self.max_volume, is_mute=True))
        if action == "unmute":
            return jsonable(self.nodes.rpc.set_audio_volume(self.max_volume, is_mute=False))
        raise ValueError(f"audio: unknown action {action!r}")


class InteractionPlugin:
    """interaction 卡片：语音交互开关、工作模式与拾音来源（原 mic_source 卡并入
    —— 同属 HDU 交互平面 AgentControlService/HalAudioService）。

      - voice_enable / voice_get：语音交互总开关（SetVoiceEnable/GetVoiceEnable）；
      - mode_normal / mode_only_voice / mode_get：工作模式（property_id=2，
        only_voice 纯语音 / normal 完整交互；修改需重启机器人后生效）；
      - mic_get / mic_internal / mic_external：拾音来源（0 机内 / 1 外部；
        v3.2 机内麦克风存在已知 BUG，推荐外部）。
    """

    ACTIONS = {
        "voice_enable": (["enable"], "开/关语音交互总开关"),
        "voice_get": ([], "查询语音交互总开关状态"),
        "mode_normal": ([], "设置为完整交互模式（重启后生效）"),
        "mode_only_voice": ([], "设置为纯语音交互模式（重启后生效）"),
        "mode_get": ([], "查询当前交互模式"),
        "mic_get": ([], "查询当前拾音来源（0 机内 / 1 外部）"),
        "mic_internal": ([], "切换到机内麦克风（v3.2 已知存在 BUG，不推荐）"),
        "mic_external": ([], "切换到外部麦克风（推荐）"),
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "enable": {"type": "boolean", "description": "true 开启 / false 关闭语音交互"},
        })
        # 拾音/交互配置会改动语音链路状态，与 tts/audio 的扬声器通道同属语音平面。
        schema["x-resource"] = "mouth"
        return tool("interaction", "actuator", "语音交互开关/工作模式/拾音来源（AgentControlService + HalAudioService RPC；"
                                              "模式修改需重启生效；机内麦克风 v3.2 有已知 BUG 推荐外部）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "voice_enable":
            return jsonable(self.nodes.rpc.set_voice_enable(bool(args.get("enable", True))))
        if action == "voice_get":
            return jsonable(self.nodes.rpc.get_voice_enable())
        if action == "mode_normal":
            return jsonable(self.nodes.rpc.set_agent_properties("normal"))
        if action == "mode_only_voice":
            return jsonable(self.nodes.rpc.set_agent_properties("only_voice"))
        if action == "mode_get":
            return jsonable(self.nodes.rpc.get_agent_properties())
        if action == "mic_get":
            return jsonable(self.nodes.rpc.get_mic_source())
        if action == "mic_internal":
            return jsonable(self.nodes.rpc.set_mic_source(0))
        if action == "mic_external":
            return jsonable(self.nodes.rpc.set_mic_source(1))
        raise ValueError(f"interaction: unknown action {action!r}")


class FacePlayPlugin:
    """face_play 卡片：表情播放 + 表情资源列表。

    对应 /skill/pilot/face/play 话题（RosMsgWrapper + FacePlayInfo）+ HDU
    ResourceService/GetResourceList（emoticon 类）。is_stop=true 可取消所有
    正在播放的表情（此场景其余字段可空）。
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {"list": ([], "列出可用 emoticon 表情资源（GetResourceList emoticon 类）"),
             "play": (["e_path"], "播放表情动画（e_id 可选，repeat 为重播次数）"),
             "cancel": ([], "取消所有表情播放（is_stop=true）")},
            {
                "e_path": {"type": "string", "description": "表情文件绝对路径（list → emoticon）"},
                "e_id": {"type": "integer", "description": "表情资源 id（可选，与 e_path 二选一）"},
                "repeat": {"type": "integer", "default": 1, "description": "重播次数"},
            },
        )
        # 表情显示在头部屏幕上 —— 与 head_control 同一 head 通道。
        schema["x-resource"] = "head"
        return tool("face_play", "actuator", "播放/取消表情 + 表情资源列表（话题 /skill/pilot/face/play + "
                                             "GetResourceList RPC；e_path 为 emoticon 资源绝对路径，priority 固定 440）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "list":
            response = self.nodes.rpc.resource_list("emoticon")
            return {"resources": (response.get("data") or {}).get("resources", [])}
        if action == "play":
            e_path = args.get("e_path", "")
            _require(e_path or args.get("e_id") is not None, "需要 e_path 或 e_id")
            payload = {
                "header": create_header(),
                "e_path": e_path,
                "e_id": int(args.get("e_id", 0)),
                "repeat": int(args.get("repeat", 1)),
                "priority": 440,
                "is_stop": False,
            }
            self.nodes.publish_wrapper("face_play_pub", payload)
            return {**payload, "state": "published"}
        if action == "cancel":
            payload = {"header": create_header(), "e_path": "", "e_id": 0,
                       "repeat": 0, "priority": 440, "is_stop": True}
            self.nodes.publish_wrapper("face_play_pub", payload)
            return {"state": "published", "is_stop": True}
        raise ValueError(f"face_play: unknown action {action!r}")


class SkillPlayPlugin:
    """skill_play 卡片：技能包（舞蹈）播放控制 + 技能状态查询 + 技能资源列表
    （原 skill_status 状态卡并入；resource_list 的 skill/offring_work 类散入）。

      - play / pause / stop_play：ADU SkillPilotService/SkillPackage。path 为
        skill 资源目录（list → skill）；Start 返回 session_id，Pause/Stop
        需回传。舞蹈播放需要约 2 米安全净空（开发文档要求）。
      - list / list_offring_work：GetResourceList skill / offring_work 类。
      - state：查询 /skill/pilot/skill_status 技能状态流最新快照
        （核数/电池/自主充电状态；流需要 a3_aimdk protobuf wheel + skillpilot
        开启 ros2 后端，wheel 缺失时返回 unknown）。
    """

    ACTIONS = {
        "list": ([], "列出可用 skill 技能包资源（GetResourceList skill 类）"),
        "list_offring_work": ([], "列出可用演出作品资源（GetResourceList offring_work 类）"),
        "play": (["path"], "播放技能包（异步：返回 action_id，完成后回报 ACP）"),
        "pause": (["session_id"], "暂停技能播放"),
        "stop_play": (["session_id"], "停止技能播放"),
        "state": ([], "查询技能状态流最新快照（核数/电池/自主充电状态）"),
    }

    # 技能（舞蹈）时长上界 —— x-completion 超时与之对齐（g1 导航 180 s 的同款思路）。
    PLAY_TIMEOUT_S = 600
    _POLL_INTERVAL_S = 1.0

    # /skill/pilot/skill_status 流快照里可视为“技能已结束”的字段（防御式：流是
    # SkillPilotStatus protobuf，在线文档无法核对字段名，命中任一即终态）。
    _TERMINAL_STATES = {"idle", "finished", "complete", "completed", "stop",
                        "stopped", "stop_play", "error", "failed", "failure"}

    def __init__(self, nodes):
        self.nodes = nodes
        self.has_stream = "skill_status" in nodes.streams
        self._play_action_id = None
        self._last_session_id = None
        # ThreadingHTTPServer serves MCP calls concurrently — the check-then-set on
        # _play_action_id / _last_session_id must be atomic (MotionPlayPlugin 同款).
        self._play_lock = threading.Lock()

    def get_tool(self):
        topic_out = None
        if self.has_stream:
            stream = self.nodes.streams["skill_status"]
            topic_out = [{"topic": stream["topic"], "format": stream["format"]}]
        schema = action_schema(self.ACTIONS, {
            "path": {"type": "string", "description": "skill 资源目录绝对路径（list → skill）"},
            "session_id": {"type": "string", "description": "技能会话 id（play 返回）"},
        })
        # 技能包/舞蹈驱动全身 —— 与 motion_play 同款全通道声明。
        schema["x-resource"] = ["base", "arm_l", "arm_r", "hand_l", "hand_r",
                                "waist", "head"]
        # play 为长时间动作（舞蹈动辄数十秒）：声明 ACP 完成契约，由后台线程
        # 轮询 skill_status 流快照判定终态后回报（g1/tianyi 导航同款）。
        schema["x-completion"] = {"actions": ["play"], "timeout": self.PLAY_TIMEOUT_S}
        return tool("skill_play", "actuator", "技能包/舞蹈播放控制 + 技能资源列表（SkillPackage + GetResourceList RPC；"
                                             "播放需 ~2m 安全净空，Start 返回 session_id 供暂停/停止使用；"
                                             "播放为异步：轮询 skill_status 流回报 ACP 完成事件；"
                                             "state 查询技能状态流）",
                    schema, topic_out=topic_out)

    def start(self):
        pass

    def _settle_active(self, status, detail):
        """结算挂起的 ACP 等待线程（若有）：清 id 后立刻回报终态。

        调用方必须持有 _play_lock。返回被结算的 (action_id, session_id)
        （None 表示无挂起），供调用方在锁外按旧 session_id 执行物理停止。

        只清 id 会让 Agent Core 的 barrier 挂到 600 s 超时 —— 取消也必须显式
        POST cancelled（LocoPlugin._settle_active 同款语义）。
        """
        action_id, self._play_action_id = self._play_action_id, None
        if action_id is not None:
            _acp_notify(action_id, status, detail, "skill_play")
        return (action_id, self._last_session_id) if action_id is not None else None

    def stop(self):
        # 插件生命周期卸载：与框架 stop 同款 —— 有技能在跑就先物理停止并结算，
        # 避免卸载后悬挂的 worker 再 POST 幽灵 completed（LocoPlugin.stop 先例）。
        with self._play_lock:
            prior = self._settle_active("cancelled", {"reason": "plugin_stopped",
                                                      "session_id": self._last_session_id or ""})
        if prior is not None:
            try:
                self.nodes.rpc.skill_package("Stop", "", prior[1] or "")
            except Exception:
                pass

    def _snapshot_terminal(self):
        """Probe the skill_status stream snapshot for a terminal state, defensively.

        The SkillPilotStatus protobuf's field names could not be verified against
        the online dev guide, so this accepts several shapes: a top-level or
        nested `state`/`status`/`skill_state`/`session_state` string whose value
        (name or enum-ish int ≥ 3) marks the skill no longer playing. Anything
        unrecognized counts as still-playing — the timeout is the backstop.
        """
        snapshot = self.nodes.snapshot("skill_status")
        if not snapshot:
            return None
        for key in ("state", "status", "skill_state", "session_state"):
            value = snapshot.get(key) if isinstance(snapshot, dict) else None
            if isinstance(value, str) and value.strip().lower() in self._TERMINAL_STATES:
                return value
            # Common protobuf enum pattern: 0=IDLE 1=RUNNING 2=PAUSED 3+=done/fail.
            if isinstance(value, int) and value >= 3:
                return value
            nested = snapshot.get("data") if isinstance(snapshot, dict) else None
            if isinstance(nested, dict):
                inner = nested.get(key)
                if isinstance(inner, str) and inner.strip().lower() in self._TERMINAL_STATES:
                    return inner
        return None

    def _play_worker(self, action_id, session_id, path):
        # Superseded guard: a newer play took over the stream — never fire a
        # stale completion (g1 _acp_wait_nav pattern). Terminal posts are
        # capture-and-clear under the lock so a concurrent cancel and a natural
        # finish never double-report.
        with self._play_lock:
            if self._play_action_id != action_id:
                return
        deadline = time.time() + self.PLAY_TIMEOUT_S
        result = {"path": path, "session_id": session_id}
        while time.time() < deadline:
            time.sleep(self._POLL_INTERVAL_S)
            with self._play_lock:
                if self._play_action_id != action_id:
                    return
            terminal = self._snapshot_terminal()
            if terminal is not None:
                result["final_state"] = terminal
                self._finish(action_id, "completed", result)
                return
        result["error"] = f"skill play not finished within {self.PLAY_TIMEOUT_S}s"
        self._finish(action_id, "error", result)

    def _finish(self, action_id, status, result):
        """原子终态：锁内确认仍持有该 action_id 才清 id，锁外回报。"""
        with self._play_lock:
            if self._play_action_id != action_id:
                return  # 已被取消/顶替，终态由对方回报
            self._play_action_id = None
        _acp_notify(action_id, status, result, "skill_play")

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            # Framework stop must physically stop the skill session — otherwise
            # the skill keeps running and the ACP worker reports completion later.
            # With nothing armed the call stays inert (canvas lifecycle toggles
            # must not spam SkillPilot Stop when no skill is playing).
            with self._play_lock:
                if self._play_action_id is None:
                    return {"state": "idle"}
                prior = self._settle_active(
                    "cancelled", {"reason": "cancelled_by_framework",
                                  "session_id": self._last_session_id or ""})
                stop_session = prior[1] or ""
            response = jsonable(self.nodes.rpc.skill_package("Stop", "", stop_session))
            return {"state": "stopped", "response": response}
        if action == "info":
            return {"state": "ready", "has_stream": self.has_stream}
        if action == "list":
            response = self.nodes.rpc.resource_list("skill")
            return {"resources": (response.get("data") or {}).get("resources", [])}
        if action == "list_offring_work":
            response = self.nodes.rpc.resource_list("offring_work")
            return {"resources": (response.get("data") or {}).get("resources", [])}
        if action == "play":
            path = args.get("path", "")
            _require(path, "path 不能为空")
            # 新播放顶替旧播放：先结算旧等待线程（立即 cancelled）并在锁外按旧
            # session_id 物理停止 —— 顺序不能反，否则 Stop 打断的是新会话
            # （ThreadingHTTPServer 使并发 play 可达，MotionPlayPlugin 同款）。
            with self._play_lock:
                prior = self._settle_active("cancelled", {"reason": "replaced_by_new_play"})
            if prior is not None and prior[1]:
                try:
                    self.nodes.rpc.skill_package("Stop", "", prior[1])
                except Exception:
                    pass
            response = jsonable(self.nodes.rpc.skill_package("Start", path))
            session_id = (response.get("data") or {}).get("session_id", "")
            action_id = f"skill_play_{uuid4().hex[:8]}"
            with self._play_lock:
                self._play_action_id = action_id
                # 记住当前会话：框架 stop 要按它调用 SkillPilot Stop，
                # 而 stop 入参里并不会带 session_id。
                self._last_session_id = session_id
            threading.Thread(target=self._play_worker,
                             args=(action_id, session_id, path), daemon=True).start()
            return {"state": "playing", "action_id": action_id,
                    "session_id": session_id, "path": path, "response": response}
        if action == "pause":
            return jsonable(self.nodes.rpc.skill_package("Pause", "", args.get("session_id", "")))
        if action == "stop_play":
            # Stopping the session also settles any pending ACP waiter (an
            # immediate cancelled post, not a silent id clear).
            with self._play_lock:
                self._settle_active("cancelled", {"reason": "cancelled_by_request",
                                                  "session_id": args.get("session_id", "")})
            return jsonable(self.nodes.rpc.skill_package("Stop", "", args.get("session_id", "")))
        if action == "state":
            snapshot = self.nodes.snapshot("skill_status")
            return snapshot or {"state": "unknown"}
        raise ValueError(f"skill_play: unknown action {action!r}")


class ControlledSpatialPlugin:
    """controlled_spatial 执行卡：建图 + 导航 + 重定位一体化（config 门控，默认关闭）。

    参照 tianyi2.0/g1 的 controlled_spatial 卡，把同一 ADU 上的空间能力合并为一张卡：
      - 建图/地图管理（MappingService）：StartMapping / StopMapping（保存/放弃）/
        GetStoredMapNames / GetCurrentWorkingMap / RenameMap；
      - 导航（PncService）：目标点/位姿规划导航、直线行走、平移/原地旋转、
        暂停/恢复/取消/查询（task_id=0 自动分配，驱动记住返回的 id 供后续控制复用）；
      - 重定位（SLAMRelocalizationService）：导航的硬性前置条件，在目标地图上
        StartNormalRelocalization / StopNormalRelocalization；
      - 地图数据查询（Get2DWholeMap）：含分辨率/原点/占用栅格，并支持物理坐标 →
        像素坐标换算（合并自原 map_get 处理卡）。
    前置条件（开发文档 §7.9）：MC 处于 MOTION 模式（mc_mode get_up）且已完成
    重定位，导航与重定位需工作在相同 map_id 上；到点精度最大约 0.4 米。
    """

    # 导航为长时间动作：声明 ACP 完成契约（tianyi2.0/g1 的 controlled_spatial
    # 同款 180 s 超时），由后台线程轮询 PncService/ActionGetState 判定到点。
    NAV_ACTIONS = ("navi_to_goal", "navi_to_pose", "linear_to_goal",
                   "linear_to_pose", "move_forward", "spin_turn")
    NAV_TIMEOUT_S = 180
    _NAV_POLL_INTERVAL_S = 1.0

    # ActionGetState 响应中可判定“任务已终态”的取值（防御式：响应字段名在线
    # 文档无法核对，state/status 里出现这些字符串即视为结束）。
    _NAV_TERMINAL_STATES = {"finished", "finish", "complete", "completed", "done",
                            "succeed", "succeeded", "success", "arrived", "stop",
                            "stopped", "cancelled", "canceled", "error", "failed",
                            "failure"}
    _NAV_ERROR_STATES = {"stop", "stopped", "cancelled", "canceled", "error",
                         "failed", "failure"}

    ACTIONS = {
        # -- 建图 / 地图管理 --
        "start_mapping": ([], "开始建图（机器人行走采集环境）"),
        "stop_save": (["map_name"], "结束建图并保存（map_name 为新地图名称）"),
        "stop_discard": ([], "结束建图不保存"),
        "list_maps": ([], "查询已保存地图列表"),
        "current_map": ([], "查询当前工作地图"),
        "rename_map": (["map_id", "new_name"], "重命名地图"),
        # -- 重定位 --
        "start_relocalization": (["map_dir"], "在指定地图目录上启动普通重定位（导航前置条件）"),
        "stop_relocalization": ([], "停止重定位（可选带 reloc_pose 位姿辅助收敛）"),
        # -- 导航 --
        "navi_to_goal": (["map_id", "target_id"], "按目标点 ID 规划导航（PlanningNaviToGoal；异步：返回 action_id，到点后回报 ACP）"),
        "navi_to_pose": (["map_id", "x", "y", "angle"], "按位姿规划导航（PlanningNaviToPose2D；异步：到点后回报 ACP）"),
        "linear_to_goal": (["map_id", "target_id"], "直线导航到目标点（LinearNaviToGoal，先转后走；异步回报 ACP）"),
        "linear_to_pose": (["map_id", "x", "y", "angle"], "直线导航到位姿（LinearNaviToPose2D；异步回报 ACP）"),
        "move_forward": (["map_id", "distance"], "直线平移指定距离（MoveForward，朝向不变；异步回报 ACP）"),
        "spin_turn": (["map_id", "angle"], "原地旋转指定角度（SpinTurn，rad；异步回报 ACP）"),
        "cancel": (["task_id"], "取消导航任务"),
        "pause": (["task_id"], "暂停导航任务"),
        "resume": (["task_id"], "恢复暂停的任务"),
        "nav_state": (["task_id"], "查询导航任务状态（task_id 缺省用最近一次任务）"),
        # -- 地图数据 --
        "get_map": (["map_id"], "获取 SLAM 2D 全量栅格地图数据；可选传 x/y 返回像素坐标换算"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self.last_task_id = None
        self._nav_action_id = None
        # ThreadingHTTPServer serves MCP calls concurrently — the check-then-set on
        # _nav_action_id (settle prior / arm new) must be atomic (MotionPlayPlugin 同款).
        self._nav_lock = threading.Lock()

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "map_name": {"type": "string", "description": "保存时的新地图名称"},
            "map_id": {"type": "integer", "description": "工作地图 id（需与重定位地图一致）"},
            "map_dir": {"type": "string", "description": "地图目录（list_maps → map_dir，重定位用）"},
            "new_name": {"type": "string", "description": "新名称（rename_map）"},
            "target_id": {"type": "integer", "description": "目标点 id（地图点位）"},
            "x": {"type": "number", "description": "目标/重定位位姿 x（m）"},
            "y": {"type": "number", "description": "目标/重定位位姿 y（m）"},
            "angle": {"type": "number", "description": "目标/重定位朝向角（rad）"},
            "distance": {"type": "number", "description": "平移距离（m，正为前进）"},
            "task_id": {"type": "integer", "description": "导航任务 id（0 自动分配；控制/查询动作可传空用最近任务）"},
        })
        # 建图/导航/重定位期间底盘由 ADU 独占 —— 与 loco/auto_charging 同通道。
        schema["x-resource"] = "base"
        # 导航动作为长时间动作：声明 ACP 完成契约（tianyi2.0/g1 同款 180 s），
        # 由后台线程轮询 PncService/ActionGetState 判定到点后回报。
        schema["x-completion"] = {"actions": list(self.NAV_ACTIONS),
                                  "timeout": self.NAV_TIMEOUT_S}
        # 建图（start_mapping）刻意不声明 x-completion：它是操作员驱动的开放
        # 过程（走多久由人决定，stop_save/stop_discard 才是终点），与 tianyi/g1
        # 只为 navigate 动作声明契约的处理一致。
        return tool("controlled_spatial", "actuator",
                    "建图/导航/重定位一体化控制（ADU MappingService/PncService/SLAMRelocalizationService RPC；"
                    "config 门控模块，默认关闭；导航前置：mc_mode get_up + start_relocalization 且 map_id 一致；"
                    "task_id 传 0 自动分配，控制/查询动作可缺省复用最近任务 id；"
                    "导航为异步：返回 action_id，轮询 ActionGetState 到点后回报 ACP 完成事件）",
                    schema)

    def start(self):
        pass

    def _settle_active(self, status, detail):
        """结算挂起的 ACP 等待线程（若有）：清 id 后立刻回报终态。

        调用方必须持有 _nav_lock。返回被结算的 (action_id, task_id)
        （None 表示无挂起），供调用方在锁外按旧 task_id 执行物理取消 ——
        ActionCancel 只对发起中的任务有效，必须在被新任务覆盖前捕获。

        只清 id 会让 Agent Core 的 barrier 挂到 180 s 超时 —— 取消也必须显式
        POST cancelled（LocoPlugin._settle_active 同款语义）。
        """
        action_id, self._nav_action_id = self._nav_action_id, None
        if action_id is not None:
            _acp_notify(action_id, status, detail, "controlled_spatial")
        return (action_id, self.last_task_id) if action_id is not None else None

    def stop(self):
        # 插件生命周期卸载：与框架 stop 同款 —— 有导航在跑就先物理取消并结算，
        # 避免卸载后悬挂的 worker 再 POST 幽灵 completed（LocoPlugin.stop 先例）。
        with self._nav_lock:
            prior = self._settle_active("cancelled", {"reason": "plugin_stopped",
                                                      "task_id": self.last_task_id or 0})
        if prior is not None:
            try:
                self.nodes.rpc.navi("ActionCancel", {"task_id": prior[1] or 0})
            except Exception:
                pass

    def _task_id(self, args):
        value = args.get("task_id")
        if value in (None, ""):
            if self.last_task_id is None:
                return 0
            return self.last_task_id
        return int(value)

    def _remember(self, response):
        task_id = (response or {}).get("task_id")
        if task_id:
            self.last_task_id = task_id
        return jsonable(response)

    def _nav_state_terminal(self, response):
        """Probe an ActionGetState response for a terminal task state, defensively.

        The exact response field names could not be verified against the online
        dev guide, so this accepts a `state`/`status`/`task_state` string (or
        enum-ish int ≥ 3) either top-level or nested under `data`. Unrecognized
        shapes count as still-navigating — the NAV_TIMEOUT_S is the backstop.
        Returns the raw value, or "error"/None marker handled by the caller via
        _NAV_ERROR_STATES.
        """
        if not isinstance(response, dict):
            return None
        for scope in (response, response.get("data") if isinstance(response.get("data"), dict) else {}):
            for key in ("state", "status", "task_state"):
                value = scope.get(key)
                if isinstance(value, str) and value.strip().lower() in self._NAV_TERMINAL_STATES:
                    return value
                # Common protobuf enum pattern: 0=IDLE 1=RUNNING 2=PAUSED 3+=done/fail.
                if isinstance(value, int) and value >= 3:
                    return value
        return None

    def _nav_worker(self, action_id, task_id, action):
        # Superseded guard: a newer nav action or a cancel took over — never
        # fire a stale completion (g1 _acp_wait_nav pattern). Terminal posts are
        # capture-and-clear under the lock so a concurrent cancel and a natural
        # arrival never double-report.
        with self._nav_lock:
            if self._nav_action_id != action_id:
                return
        deadline = time.time() + self.NAV_TIMEOUT_S
        result = {"action": action, "task_id": task_id}
        while time.time() < deadline:
            time.sleep(self._NAV_POLL_INTERVAL_S)
            with self._nav_lock:
                if self._nav_action_id != action_id:
                    return
            try:
                response = self.nodes.rpc.navi_state(task_id)
            except Exception as exc:  # noqa: BLE001 — notify and keep polling
                result["error"] = f"ActionGetState poll failed: {exc}"
                self._finish(action_id, "error", result)
                return
            terminal = self._nav_state_terminal(response)
            if terminal is None:
                continue
            state = terminal.strip().lower() if isinstance(terminal, str) else str(terminal)
            result["final_state"] = terminal
            if state in self._NAV_ERROR_STATES:
                result["error"] = f"navigation ended in {terminal}"
                self._finish(action_id, "error", result)
            else:
                self._finish(action_id, "completed", result)
            return
        result["error"] = f"navigation not finished within {self.NAV_TIMEOUT_S}s"
        self._finish(action_id, "error", result)

    def _finish(self, action_id, status, result):
        """原子终态：锁内确认仍持有该 action_id 才清 id，锁外回报。"""
        with self._nav_lock:
            if self._nav_action_id != action_id:
                return  # 已被取消/顶替，终态由对方回报
            self._nav_action_id = None
        _acp_notify(action_id, status, result, "controlled_spatial")

    def _nav_dispatch(self, action, method, payload):
        """Send a long-running navigation RPC and arm the ACP completion waiter.

        新导航顶替旧导航：先结算旧等待线程（立即 cancelled）并在锁外按旧
        task_id 物理取消 —— 不取消的话机器人会同时执行两个导航任务
        （ThreadingHTTPServer 使并发下发可达，MotionPlayPlugin 同款）。
        """
        with self._nav_lock:
            prior = self._settle_active("cancelled", {"reason": "replaced_by_new_navigation"})
            prior_task = prior[1] if prior is not None else None
        if prior is not None and prior_task:
            try:
                self.nodes.rpc.navi("ActionCancel", {"task_id": prior_task})
            except Exception:
                pass
        response = self._remember(self.nodes.rpc.navi(method, payload))
        task_id = (response or {}).get("task_id") or self.last_task_id or 0
        action_id = f"a3_nav_{uuid4().hex[:8]}"
        with self._nav_lock:
            self._nav_action_id = action_id
        threading.Thread(target=self._nav_worker,
                         args=(action_id, task_id, action), daemon=True).start()
        return {"state": "navigating", "action_id": action_id,
                "task_id": task_id, "response": response}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            # Framework stop must physically cancel the running nav task —
            # otherwise the robot keeps driving and the ACP worker reports
            # completion when the task finally ends on its own. With nothing
            # armed the call stays inert (canvas lifecycle toggles must not
            # spam ActionCancel when no task is navigating). The cancelled task
            # id is captured under the lock: args may omit task_id, and
            # last_task_id may already point at a newer task.
            with self._nav_lock:
                if self._nav_action_id is None:
                    return {"state": "idle"}
                prior = self._settle_active(
                    "cancelled", {"reason": "cancelled_by_framework",
                                  "task_id": self._task_id(args)})
                stop_task = prior[1] if prior is not None else self._task_id(args)
            response = jsonable(self.nodes.rpc.navi("ActionCancel", {"task_id": stop_task or 0}))
            return {"state": "stopped", "response": response}
        if action == "info":
            return {"state": "ready", "last_task_id": self.last_task_id}
        # -- 建图 / 地图管理 --
        if action == "start_mapping":
            return jsonable(self.nodes.rpc.start_mapping())
        if action == "stop_save":
            return jsonable(self.nodes.rpc.stop_mapping(args.get("map_name")))
        if action == "stop_discard":
            return jsonable(self.nodes.rpc.stop_mapping(None))
        if action == "list_maps":
            return jsonable(self.nodes.rpc.get_stored_map_names())
        if action == "current_map":
            return jsonable(self.nodes.rpc.get_current_working_map())
        if action == "rename_map":
            return jsonable(self.nodes.rpc.rename_map(int(args.get("map_id", 0)),
                                                      args.get("old_name", ""),
                                                      args.get("new_name", "")))
        # -- 重定位 --
        if action == "start_relocalization":
            return jsonable(self.nodes.rpc.slam_start_normal_relocalization(args.get("map_dir", "")))
        if action == "stop_relocalization":
            reloc_pose = None
            if args.get("x") is not None or args.get("y") is not None or args.get("angle") is not None:
                reloc_pose = {"x": float(args.get("x", 0)), "y": float(args.get("y", 0)),
                              "angle": float(args.get("angle", 0))}
            return jsonable(self.nodes.rpc.slam_stop_normal_relocalization(reloc_pose))
        # -- 导航（长时间动作：_nav_dispatch 发 RPC 后返回 action_id 并武装
        # ACP 完成等待线程，轮询 ActionGetState 判定到点） --
        if action == "navi_to_goal":
            return self._nav_dispatch(action, "PlanningNaviToGoal", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "target_id": int(args.get("target_id", 0)), "guide_line_id": 0,
                "ackerman_mode": False})
        if action == "navi_to_pose":
            return self._nav_dispatch(action, "PlanningNaviToPose2D", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "pose": {"position": {"x": float(args.get("x", 0)), "y": float(args.get("y", 0))},
                         "angle": float(args.get("angle", 0))},
                "ackerman_mode": False})
        if action == "linear_to_goal":
            return self._nav_dispatch(action, "LinearNaviToGoal", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "target_id": int(args.get("target_id", 0))})
        if action == "linear_to_pose":
            return self._nav_dispatch(action, "LinearNaviToPose2D", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "pose": {"position": {"x": float(args.get("x", 0)), "y": float(args.get("y", 0))},
                         "angle": float(args.get("angle", 0))}})
        if action == "move_forward":
            return self._nav_dispatch(action, "MoveForward", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "angle": 0, "distance": float(args.get("distance", 0))})
        if action == "spin_turn":
            return self._nav_dispatch(action, "SpinTurn", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "angle": float(args.get("angle", 0))})
        if action == "cancel":
            # 取消任务同时结算挂起的 ACP 等待线程（立刻回报 cancelled，
            # 不让 Agent Core 的 barrier 挂到超时）。
            with self._nav_lock:
                self._settle_active("cancelled", {"reason": "cancelled_by_request",
                                                  "task_id": self._task_id(args)})
            return jsonable(self.nodes.rpc.navi("ActionCancel", {"task_id": self._task_id(args)}))
        if action == "pause":
            return jsonable(self.nodes.rpc.navi("ActionPause", {"task_id": self._task_id(args)}))
        if action == "resume":
            return jsonable(self.nodes.rpc.navi("ActionResume", {"task_id": self._task_id(args)}))
        if action == "nav_state":
            return jsonable(self.nodes.rpc.navi_state(self._task_id(args)))
        # -- 地图数据（原 map_get 处理卡合并而来） --
        if action == "get_map":
            response = self.nodes.rpc.get_2d_whole_map(int(args.get("map_id", 0)))
            data = response.get("data") or {}
            result = jsonable(response)
            if args.get("x") is not None or args.get("y") is not None:
                resolution = data.get("resolution") or 0.05
                origin = data.get("origin") or {}
                origin_x = origin.get("x", 0) if isinstance(origin, dict) else 0
                origin_y = origin.get("y", 0) if isinstance(origin, dict) else 0
                # 逆变换 of the map transform (pixel → world used in
                # spatial_map._grid_layers): world → pixel must SUBTRACT the
                # metric origin and DIVIDE by resolution, not add/multiply.
                # Row 0 of the occupancy grid is the highest y (grid rows grow
                # downward), hence the y-axis inversion.
                px = int(round((float(args.get("x", 0)) - origin_x) / resolution))
                py = int(round((origin_y - float(args.get("y", 0))) / resolution))
                result["pixel"] = {"x": px, "y": py,
                                   "formula": "pixel_x = round((x - origin_x) / resolution); "
                                              "pixel_y = round((origin_y - y) / resolution)"}
            return result
        raise ValueError(f"controlled_spatial: unknown action {action!r}")


class AutoChargingPlugin:
    """auto_charging 卡片：自主充电控制（config 门控，默认关闭）。

    对应 ADU SkillPilotService/AutoCharging。command 取
    AutoChargingCommand_START/STOP/RESET；trigger 为触发来源
    （AGENT=1 智能体发起 / AIMMASTER=2 App 手动 / LOW_POWER=3 / IDLE_TIMEOUT=4）。
    失败后先 STOP 再 RESET 复位（不调用则 10s 自动恢复 Idle）。
    """

    ACTIONS = {
        "charge_start": ([], "启动自主充电流程（导航至充电桩并插枪）"),
        "charge_stop": ([], "停止自主充电（导航中停止导航；充电中拔枪结束）"),
        "reset": ([], "失败复位（在 charge_stop 之后调用，恢复 Idle 状态）"),
        "state": ([], "查询技能状态（skill_status 话题缓存）"),
    }

    TRIGGERS = {"charge_start": "AutoChargingCommand_START",
                "charge_stop": "AutoChargingCommand_STOP",
                "reset": "AutoChargingCommand_RESET"}

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {})
        # 自主充电会驱动底盘导航到充电桩 —— 与 loco/controlled_spatial 同通道。
        schema["x-resource"] = "base"
        return tool("auto_charging", "actuator", "自主充电控制（SkillPilotService AutoCharging RPC；config 门控模块，"
                                                "默认关闭；失败后先 stop 再 reset 复位）",
                    schema)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action in self.TRIGGERS:
            return jsonable(self.nodes.rpc.auto_charging(self.TRIGGERS[action], "AutoChargingTrigger_AGENT"))
        if action == "state":
            return self.nodes.snapshot("skill_status") or {"state": "unknown"}
        raise ValueError(f"auto_charging: unknown action {action!r}")


class SpatialMapPlugin:
    """spatial_map 状态卡：SLAM 2D 栅格地图可视化流（config 门控，默认关闭）。

    参照 tianyi2.0/g1 的空间状态卡（tianyi spatial_map / g1 controlled_spatial_map），
    周期拉取 Get2DWholeMap 的占用栅格，转换成渲染器约定的 sensor/mapping 二进制
    点云格式发布到 core 域：
      - 可通行栅格（值 127）→ z=-0.03 的地面层，点数预算 MAP_FLOOR_POINTS；
      - 墙壁/障碍（其余非零值）→ z=0.04 的特征层，点数预算 MAP_FEATURE_POINTS；
      - 二维格点抽样 + 总点数上限 MAP_MAX_POINTS（渲染器硬上限 80000）。
    A3 的颈部 Livox 仅为避障传感器、不提供带里程计位姿的点云，因此不做
    tianyi 式 3D 点云录制卡（controlled_spatial_map），机器人位姿经
    pose_available=false 告知渲染器。栅格字段名按防御式解析（文档在线版
    无法核对时兼容多种形状）。
    """

    def __init__(self, nodes, namespace):
        self.nodes = nodes
        self.namespace = namespace
        self.topic = f"/{namespace}/agibot_a3/spatial_map"
        cfg = nodes.config.get("plugins", {}).get("spatial_map", {})
        self.map_id = int(cfg.get("map_id", 0))
        interval = float(cfg.get("publish_interval", MAP_PUBLISH_INTERVAL))
        self.publish_interval = max(interval, MAP_PUBLISH_INTERVAL)
        from std_msgs.msg import UInt8MultiArray
        self.pub = nodes.core.create_publisher(UInt8MultiArray, self.topic, 5)
        self._running = False
        self._thread = None
        # None = never published yet — time.monotonic() starts near 0 on a fresh
        # process, so a numeric sentinel would suppress the very first frame.
        self._last_publish = None

    def get_tool(self):
        return tool("spatial_map", "sensor", "SLAM 2D 地图可视化流（占用栅格 → sensor/mapping 点云；"
                                            "config 门控模块，默认关闭；建图/导航状态经 controlled_spatial 卡控制）",
                    topic_out=[{"topic": self.topic, "format": "sensor/mapping"}])

    def start(self):
        """启动后台轮询线程（dispatch 的 start/stop 同样会启停该线程）。"""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)

    def _poll_loop(self):
        # Sleep first so card start (dispatch "start") returns before any RPC
        # fires — a card being dragged onto the canvas must not issue an
        # immediate Get2DWholeMap call.
        while self._running:
            time.sleep(self.publish_interval)
            if not self._running:
                return
            try:
                response = self.nodes.rpc.get_2d_whole_map(self.map_id)
                self.publish_map(response)
            except Exception:
                pass

    # -- 栅格 → 点云 ----------------------------------------------------------------

    @staticmethod
    def _extract_grid(data):
        """Defensively locate the occupancy grid row-list inside a Get2DWholeMap payload."""
        if not isinstance(data, dict):
            return None
        for key in ("occupancy_grid", "grid", "map_data", "data"):
            value = data.get(key)
            if isinstance(value, list) and value and isinstance(value[0], list):
                return value
        return None

    @staticmethod
    def _extract_origin(data, np):
        origin = data.get("origin")
        if isinstance(origin, dict):
            return float(origin.get("x", 0) or 0), float(origin.get("y", 0) or 0)
        if isinstance(origin, (list, tuple)) and len(origin) >= 2:
            return float(origin[0]), float(origin[1])
        return 0.0, 0.0

    @classmethod
    def _grid_layers(cls, data, np):
        """Occupancy grid → (floor_points, feature_points) float32 arrays.

        Floor cells (127) become the z=-0.03 traversable layer; every other
        nonzero cell becomes the z=0.04 obstacle layer. Both layers are lattice
        -sampled to their own budgets (tianyi `_load_stcm_grid` pattern).
        """
        grid = cls._extract_grid(data)
        if grid is None:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)
        rows = np.asarray(grid, dtype=np.uint8) if grid else np.zeros((0, 0), dtype=np.uint8)
        if rows.ndim != 2 or not rows.size:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)
        resolution = float(data.get("resolution") or 0.05)
        origin_x, origin_y = cls._extract_origin(data, np)

        def layer(mask, z, limit):
            row_idx, col_idx = np.nonzero(mask)
            if not len(row_idx):
                return np.zeros((0, 3), dtype=np.float32)
            stride = max(1, int(math.ceil(math.sqrt(len(row_idx) / max(limit, 1)))))
            keep = (row_idx % stride == 0) & (col_idx % stride == 0)
            row_idx, col_idx = row_idx[keep], col_idx[keep]
            # Grid rows grow downward (row 0 = highest y): world_x = origin_x +
            # (col + 0.5) * resolution, world_y = origin_y - (row + 0.5) *
            # resolution. get_map's world→pixel branch is the exact inverse.
            xs = origin_x + (col_idx + 0.5) * resolution
            ys = origin_y - (row_idx + 0.5) * resolution
            zs = np.full(len(xs), z, dtype=np.float32)
            return np.column_stack((xs, ys, zs)).astype(np.float32)

        floor = layer(rows == 127, -0.03, MAP_FLOOR_POINTS)
        feature = layer((rows != 0) & (rows != 127), 0.04, MAP_FEATURE_POINTS)
        return floor, feature

    def _encode_payload(self, data):
        import numpy as np
        floor, feature = self._grid_layers(data, np)
        parts = [part for part in (floor, feature) if len(part)]
        if sum(len(part) for part in parts) > MAP_MAX_POINTS:
            # keep the per-layer budgets intact by trimming the feature layer first
            budget = max(0, MAP_MAX_POINTS - len(floor))
            feature = feature[:budget]
            parts = [part for part in (floor, feature) if len(part)]
        points = np.vstack(parts) if parts else np.zeros((0, 3), dtype=np.float32)
        meta = {
            "version": 3,
            "active_map": data.get("map_id", self.map_id) if isinstance(data, dict) else self.map_id,
            "robot": {"x": 0.0, "y": 0.0, "yaw": 0.0, "pose_available": False},
            "maps": [],
            "tags": [],
            "boundary": None,
            "artifacts": {"walls": [], "tracks": [], "areas": {}},
            "trajectory_points": 0,
            "laser_points": 0,
            "grid_points": int(len(points)),
            "floor_points": int(len(floor)),
            "feature_points": int(len(feature)),
            "resolution": float(data.get("resolution") or 0.05) if isinstance(data, dict) else 0.05,
            "head_cloud_points": 0,
            "head_recording_enabled": False,
        }
        raw_meta = json.dumps(meta, ensure_ascii=False).encode()
        payload = struct.pack("<fffBI", 0.0, 0.0, 0.0, 7, len(points)) + points.tobytes()
        payload += struct.pack("<I", len(raw_meta)) + raw_meta
        return payload

    def publish_map(self, response):
        """Encode + publish one sensor/mapping frame from a Get2DWholeMap response."""
        data = (response or {}).get("data") or {}
        if not isinstance(data, dict):
            data = {}
        now = time.monotonic()
        if self._last_publish is not None and now - self._last_publish < MAP_PUBLISH_INTERVAL:
            return False
        self._last_publish = now
        from std_msgs.msg import UInt8MultiArray
        out = UInt8MultiArray()
        out.data = array("B", self._encode_payload(data))
        self.pub.publish(out)
        return True

    def dispatch(self, action, args):
        # Canvas card start/stop drives the polling thread too (tianyi pattern):
        # dragging the card off stops the periodic Get2DWholeMap pulls, dragging
        # it back on restarts them.
        if action == "start":
            if not self._running:
                self.start()
            return {"state": "running" if self._running else "idle"}
        if action == "stop":
            if self._running:
                self.stop()
            return {"state": "running" if self._running else "idle"}
        if action == "info":
            # Agent Core 用 info 推断可订阅主题：与静态 topic_out 声明一致。
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self.topic, "format": "sensor/mapping"}],
                    "topic": self.topic, "map_id": self.map_id}
        if action == "refresh":
            response = self.nodes.rpc.get_2d_whole_map(int(args.get("map_id", self.map_id)))
            published = self.publish_map(response)
            return {"state": "running", "topic": self.topic, "published": published}
        raise ValueError(f"spatial_map: unknown action {action!r}")


def build_plugins(config, namespace, ros2):
    """Instantiate every enabled plugin, mirroring X2's build_plugins.

    Card naming aligned with tianyi2.0/q5_bundle/g1: lidar_cloud (g1), battery /
    estop (tianyi2.0/q5_bundle), loco (noetix/bumi 腿式同款; 轮式驱动叫 base_drive/
    chassis_raw，A3 是双足人形故取 loco), arm_control /
    hand_control / head_control / waist_control (tianyi2.0/q5_bundle style).
    Card consolidation: joints merges the three joint-state streams; mc_mode
    absorbs mc_state and enforces the fixed FSM transition map; arm_control
    absorbs arm_compliance; tts absorbs media_play; audio absorbs audio_play +
    volume + the audio resource list; interaction absorbs mic_source; motion_play /
    face_play / skill_play absorb their own resource-list queries (resource_list
    card dissolved); skill_play absorbs skill_status; controlled_spatial merges
    mapping + navigation + relocalization + map_get (incl. the map resource
    list); spatial_map is the sensor/mapping visualization card. wakeup dropped
    (AimDK v3.2 exposes no raw mic stream — only wake-word events, useless
    without audio access). loco/waist_control/face_play need ros2_plugin_proto
    (dev-kit prebuilt) to publish their RosMsgWrapper commands and are withheld
    entirely when it is not importable.
    """
    rpc = A3Rpc(config)
    nodes = A3Nodes(config, namespace, ros2, rpc)
    plugins_cfg = config.get("plugins", {})

    def enabled(name):
        return bool(plugins_cfg.get(name, {}).get("enabled", False))

    plugins = {}
    if enabled("joints"):
        plugins["joints"] = JointsPlugin(nodes)
    if enabled("imu"):
        plugins["imu"] = ImuPlugin(nodes)
    if enabled("camera"):
        plugins["camera"] = CameraPlugin(nodes)
    if enabled("lidar_cloud"):
        plugins["lidar_cloud"] = LidarCloudPlugin(nodes)
    if enabled("battery"):
        plugins["battery"] = BatteryPlugin(nodes)
    if enabled("estop"):
        plugins["estop"] = EstopPlugin(nodes)
    if enabled("alerts"):
        plugins["alerts"] = AlertsPlugin(nodes)
    if enabled("mc_mode"):
        plugins["mc_mode"] = McModePlugin(nodes)
    # loco/waist_control/face_play publish RosMsgWrapper commands and are withheld
    # (not just broken at dispatch) when ros2_plugin_proto is absent — degraded
    # startup must expose no half-working command tools (5th PR review).
    wrapper_cmds = nodes.wrapper_available
    if enabled("loco") and wrapper_cmds:
        plugins["loco"] = LocoPlugin(nodes)
    if enabled("arm_control"):
        plugins["arm_control"] = ArmControlPlugin(nodes)
    if enabled("hand_control"):
        plugins["hand_control"] = HandControlPlugin(nodes)
    if enabled("head_control"):
        plugins["head_control"] = HeadControlPlugin(nodes)
    if enabled("waist_control") and wrapper_cmds:
        plugins["waist_control"] = WaistControlPlugin(nodes)
    if enabled("motion_play"):
        plugins["motion_play"] = MotionPlayPlugin(nodes)
    if enabled("tts"):
        plugins["tts"] = TtsPlugin(nodes)
    if enabled("audio"):
        plugins["audio"] = AudioPlugin(nodes,
                                        max_volume=plugins_cfg.get("audio", {}).get("max_volume", VOLUME_HARD_MAX))
    if enabled("interaction"):
        plugins["interaction"] = InteractionPlugin(nodes)
    if enabled("resources"):
        plugins["model"] = ModelPlugin(nodes)
    if enabled("face_play") and wrapper_cmds:
        plugins["face_play"] = FacePlayPlugin(nodes)
    if enabled("skill_play"):
        plugins["skill_play"] = SkillPlayPlugin(nodes)
    # --- advanced modules, config-gated (like X2's slam) ---
    if enabled("controlled_spatial"):
        plugins["controlled_spatial"] = ControlledSpatialPlugin(nodes)
    if enabled("spatial_map"):
        plugins["spatial_map"] = SpatialMapPlugin(nodes, namespace)
    if enabled("auto_charging"):
        plugins["auto_charging"] = AutoChargingPlugin(nodes)
    return plugins
