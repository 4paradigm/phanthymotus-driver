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
import os
import struct
import threading
import time
import queue
from concurrent.futures import ThreadPoolExecutor
import zlib
import atexit
import xml.etree.ElementTree as ET
from array import array
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, jsonable, tool
try:
    from .domain_bridge import BridgePublisher, CoreBridge
except ImportError:  # main.py loads this module as a top-level file in the image
    from domain_bridge import BridgePublisher, CoreBridge


def _core_topic(namespace: str, suffix: str) -> str:
    """Build a core-domain topic without duplicating the configured namespace."""
    return f"/{namespace.strip('/')}/{suffix.lstrip('/')}"


def _flatten_json(value, prefix, output):
    """Flatten ROS JSON objects into scalar dashboard fields."""
    if isinstance(value, dict):
        for key, item in value.items():
            _flatten_json(item, f"{prefix}_{key}" if prefix else str(key), output)
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _flatten_json(item, f"{prefix}_{index}", output)
    else:
        output[prefix] = value


def _flatten_joint_group(group, prefix, output):
    if not isinstance(group, dict):
        return
    names = group.get("name") or []
    fields = {field: group.get(field) or [] for field in ("position", "velocity", "effort")}
    for index, name in enumerate(names):
        safe_name = str(name).replace("/", "_")
        for field, values in fields.items():
            if index < len(values):
                output[f"{prefix}_{safe_name}_{field}"] = values[index]
    for field, item in group.items():
        if field not in ("name", "position", "velocity", "effort"):
            _flatten_json(item, f"{prefix}_{field}", output)


def _imu_quaternion(value):
    """Return a valid normalized IMU orientation, or ``None``.

    ``sensor_msgs/Imu`` reserves orientation_covariance[0] == -1 for an
    unavailable orientation.  The A3 HAL still places an identity quaternion
    in that field on some firmware, which makes a lying robot render upright.
    """
    if not isinstance(value, dict):
        return None
    covariance = value.get("orientation_covariance")
    try:
        if isinstance(covariance, (list, tuple)) and covariance and float(covariance[0]) < 0:
            return None
        orientation = value.get("orientation") or {}
        candidate = [float(orientation[key]) for key in ("w", "x", "y", "z")]
        norm = math.sqrt(sum(component * component for component in candidate))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(component) for component in candidate) or norm <= 1e-6:
        return None
    return [component / norm for component in candidate]


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
HAND_O10_MAX = 4096
HAND_TYPES = {"AgiHand": "agi_hand", "O10Hand": "o10_hand"}

# A3's O10Hand state is ten encoder channels per side. The channel order is
# from the official A3/O10 URDF; position counts are mapped through each
# calibrated joint's limits before entering the radian-valued skeleton.
HAND_SKELETON_JOINTS = {
    "left": (
        "L_thumb_roll_joint", "L_thumb_abad_joint", "L_thumb_mcp_joint",
        "L_index_abad_joint", "L_index_pip_joint", "L_middle_pip_joint",
        "L_ring_abad_joint", "L_ring_pip_joint", "L_pinky_abad_joint",
        "L_pinky_pip_joint",
    ),
    "right": (
        "R_thumb_roll_joint", "R_thumb_abad_joint", "R_thumb_mcp_joint",
        "R_index_abad_joint", "R_index_pip_joint", "R_middle_pip_joint",
        "R_ring_abad_joint", "R_ring_pip_joint", "R_pinky_abad_joint",
        "R_pinky_pip_joint",
    ),
}
HAND_STATE_COUNT_MAX = 4096
HAND_JOINT_LIMITS = {
    "L_thumb_roll_joint": (-1.1213740444063567, 0.029670597283903602),
    "L_thumb_abad_joint": (-0.04537856055185257, 1.642354826126664),
    "L_thumb_mcp_joint": (-0.8415977653116657, 0.0),
    "L_index_abad_joint": (0.0, 0.16406094968746698),
    "L_index_pip_joint": (0.0, 1.4835298641951802),
    "L_middle_pip_joint": (0.0, 1.4835298641951802),
    "L_ring_abad_joint": (-0.16929693744344995, 0.0),
    "L_ring_pip_joint": (0.0, 1.4835298641951802),
    "L_pinky_abad_joint": (-0.1850049007113989, 0.0),
    "L_pinky_pip_joint": (0.0, 1.4835298641951802),
    "R_thumb_roll_joint": (-0.029670597283903602, 1.1213740444063567),
    "R_thumb_abad_joint": (-1.642354826126664, 0.04537856055185257),
    "R_thumb_mcp_joint": (0.0, 0.8415977653116657),
    "R_index_abad_joint": (-0.16406094968746698, 0.0),
    "R_index_pip_joint": (0.0, 1.4835298641951802),
    "R_middle_pip_joint": (0.0, 1.4835298641951802),
    "R_ring_abad_joint": (0.0, 0.16929693744344995),
    "R_ring_pip_joint": (0.0, 1.4835298641951802),
    "R_pinky_abad_joint": (0.0, 0.1850049007113989),
    "R_pinky_pip_joint": (0.0, 1.4835298641951802),
}
HAND_INVERTED_COUNT_JOINTS = {
    "L_thumb_abad_joint", "R_thumb_abad_joint",
    "L_index_pip_joint", "L_middle_pip_joint",
    "L_ring_pip_joint", "L_pinky_pip_joint",
    "R_thumb_mcp_joint", "R_index_pip_joint", "R_middle_pip_joint",
    "R_ring_pip_joint", "R_pinky_pip_joint",
}
HAND_MIMIC_JOINTS = {
    "L_thumb_pip_joint": ("L_thumb_mcp_joint", 1.33),
    "L_thumb_dip_joint": ("L_thumb_mcp_joint", 1.3),
    "L_index_dip_joint": ("L_index_pip_joint", 1.097),
    "L_middle_dip_joint": ("L_middle_pip_joint", 1.097),
    "L_ring_dip_joint": ("L_ring_pip_joint", 1.097),
    "L_pinky_dip_joint": ("L_pinky_pip_joint", 1.097),
    "R_thumb_pip_joint": ("R_thumb_mcp_joint", 1.33),
    "R_thumb_dip_joint": ("R_thumb_mcp_joint", 1.3),
    "R_index_dip_joint": ("R_index_pip_joint", 1.097),
    "R_middle_dip_joint": ("R_middle_pip_joint", 1.097),
    "R_ring_dip_joint": ("R_ring_pip_joint", 1.097),
    "R_pinky_dip_joint": ("R_pinky_pip_joint", 1.097),
}
HAND_JOINT_ALIASES = {
    "left_thumb_roration_0": "L_thumb_roll_joint",
    "left_thumb_rotation_0": "L_thumb_roll_joint",
    "left_thumb_wiggles_1": "L_thumb_abad_joint",
    "left_thumb_bent_2": "L_thumb_mcp_joint",
    "left_index_wiggles_0": "L_index_abad_joint",
    "left_index_bent_1": "L_index_pip_joint",
    "left_middle_bent": "L_middle_pip_joint",
    "left_ring_wiggles_0": "L_ring_abad_joint",
    "left_ring_bent_1": "L_ring_pip_joint",
    "left_pinky_wiggles_0": "L_pinky_abad_joint",
    "left_pinky_bent_1": "L_pinky_pip_joint",
    "right_thumb_roration_0": "R_thumb_roll_joint",
    "right_thumb_rotation_0": "R_thumb_roll_joint",
    "right_thumb_wiggles_1": "R_thumb_abad_joint",
    "right_thumb_bent_2": "R_thumb_mcp_joint",
    "right_index_wiggles_0": "R_index_abad_joint",
    "right_index_bent_1": "R_index_pip_joint",
    "right_middle_bent": "R_middle_pip_joint",
    "right_ring_wiggles_0": "R_ring_abad_joint",
    "right_ring_bent_1": "R_ring_pip_joint",
    "right_pinky_wiggles_0": "R_pinky_abad_joint",
    "right_pinky_bent_1": "R_pinky_pip_joint",
}

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
    # A3 runtime names these as <side>_fisheye_camera/rgb. The similarly named
    # head_front_*_camera/image paths are documented variants with no publisher
    # on the deployed Ultra unit.
    "head_left_fisheye": ("/hal/head_left_fisheye_camera/rgb", "image/jpeg", "头部左鱼眼相机 RGB"),
    "head_right_fisheye": ("/hal/head_right_fisheye_camera/rgb", "image/jpeg", "头部右鱼眼相机 RGB"),
    "head_rear_fisheye": ("/hal/head_rear_fisheye_camera/rgb", "image/jpeg", "头部后鱼眼相机 RGB"),
    "head_stereo_left_fisheye": ("/hal/head_stereo_left_fisheye_camera/rgb", "image/jpeg", "头部双目交互左相机 RGB"),
    "head_stereo_right_fisheye": ("/hal/head_stereo_right_fisheye_camera/rgb", "image/jpeg", "头部双目交互右相机 RGB"),
    "armpit_right_fisheye": ("/hal/armpit_right_fisheye_camera/rgb", "image/jpeg", "右腋下鱼眼相机 RGB"),
    "chest_front_d457_rgb": ("/hal/chest_front_d457_camera/rgb", "image/jpeg", "胸前 D457 相机 RGB"),
    "chest_front_d457_depth": ("/hal/chest_front_d457_camera/depth", "image/depth-zlib", "胸前 D457 相机深度"),
    "waist_front_d415_rgb": ("/hal/waist_front_d415_camera/rgb", "image/jpeg", "腰前 D415 相机 RGB"),
    "waist_front_d415_depth": ("/hal/waist_front_d415_camera/depth", "image/depth-zlib", "腰前 D415 相机深度"),
}
# H265 foxglove CompressedVideo streams exist too, but consumer-side H265 decode support
# is inconsistent — the raw sensor_msgs/Image feeds are mirrored and re-encoded instead
# (JPEG for RGB, zlib uint16 for depth; see _encode_rgb/_encode_depth below).

RESOURCE_DIR = Path(__file__).with_name("resource")


def _a3_urdf_root():
    """Load the A3 model with the calibrated O10 hand kinematics attached."""
    root = ET.parse(RESOURCE_DIR / "a3_ultra.urdf").getroot()
    for element in list(root):
        name = element.get("name", "")
        if element.tag == "link" and (
                "_hand_channel_" in name or name in {
                    "left_thumb_tip", "left_index_tip", "left_middle_tip",
                    "left_ring_tip", "left_pinky_tip", "right_thumb_tip",
                    "right_index_tip", "right_middle_tip", "right_ring_tip",
                    "right_pinky_tip"}):
            root.remove(element)
        elif element.tag == "joint":
            parent = element.find("parent")
            child = element.find("child")
            names = (parent.get("link", "") if parent is not None else "",
                     child.get("link", "") if child is not None else "")
            if (any("_hand_channel_" in link for link in names)
                    or name.endswith("_tip_fixed")):
                root.remove(element)
    hand = ET.parse(RESOURCE_DIR / "a3_o10_hand.urdf").getroot()
    root.extend(list(hand))
    return root


def _skeleton_joint_indices() -> dict[str, int]:
    """Return stable indices for the A3 URDF's kinematic joints.

    The robot also reports sensor mounts and implementation-only fields in
    some JointState groups.  Those must not be assigned arbitrary stream-order
    indices: the canvas uses ``idx`` to look up the URDF joint transform.
    """
    try:
        root = _a3_urdf_root()
    except (OSError, ET.ParseError):
        return {}
    # Keep the original URDF element index.  The canvas model indexes joints
    # using the complete XML order, including fixed joints between actuators.
    return {
        joint.get("name"): index
        for index, joint in enumerate(root.findall("joint"))
        if joint.get("name") and (
            joint.get("type") in {"revolute", "continuous", "prismatic"}
            or "_hand_joint" in (joint.get("name") or "")
        )
    }


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
                descriptor = message.DESCRIPTOR.fields_by_name.get(key)
                if descriptor is not None and descriptor.enum_type is not None and isinstance(value, str):
                    enum_value = descriptor.enum_type.values_by_name.get(value)
                    if enum_value is not None:
                        value = enum_value.number
                setattr(message, key, value)
            except (AttributeError, TypeError):
                pass


class A3Nodes:
    """Robot-side ROS 2 node: mirrors sensor topics into the core domain (same pattern
    as X2's AimdkNodes) and owns the command publishers that go straight to the robot."""

    def __init__(self, config, namespace, ros2, rpc: A3Rpc):
        from rclpy.node import Node
        try:
            from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                                    QoSDurabilityPolicy, QoSHistoryPolicy,
                                    qos_profile_sensor_data)
        except ImportError:  # lightweight test doubles / older rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
            QoSDurabilityPolicy = type("QoSDurabilityPolicy", (), {"VOLATILE": None})
            QoSHistoryPolicy = type("QoSHistoryPolicy", (), {"KEEP_LAST": None})
            qos_profile_sensor_data = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        try:
            from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
        except ImportError:  # lightweight test doubles / older rclpy
            class ReentrantCallbackGroup:  # noqa: N801
                pass
            class MutuallyExclusiveCallbackGroup:  # noqa: N801
                pass
        from sensor_msgs.msg import Image, Imu, JointState, PointCloud2
        from std_msgs.msg import String, UInt8MultiArray
        try:
            from audio_msgs.msg import AudioCapture, AudioPlayback, AudioChunk
        except ImportError:
            AudioCapture = AudioPlayback = AudioChunk = None

        self.config = config
        self.namespace = namespace
        self.rpc = rpc
        self.robot = Node("agibot_a3_driver_robot", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self.robot)
        self.media_robot = None
        self._media_ros_executor = None
        self._media_ros_thread = None
        try:
            # Keep large Image/PointCloud2 and audio deserialization out of the
            # state/command executor.  A separate node is required: a node
            # cannot be attached to two executors, and a callback group alone
            # does not protect against executor-level starvation or a malformed
            # large sample stopping the shared spin loop.
            import rclpy.executors
            self.media_robot = Node("agibot_a3_media_robot", context=ros2.ctx_robot)
            # One worker per configured media lane (12 camera lanes plus lidar
            # and audio) keeps callbacks independent without creating a dozen
            # DDS participants.  FastDDS performs deserialization before these
            # callbacks, so this complements rather than replaces the larger
            # transport buffers in main.py.
            media_threads = max(16, len(CAMERA_TOPICS) + 4)
            self._media_ros_executor = rclpy.executors.MultiThreadedExecutor(
                context=ros2.ctx_robot, num_threads=media_threads)
            self._media_ros_executor.add_node(self.media_robot)
            self._media_ros_thread = threading.Thread(
                target=self._media_ros_executor.spin, daemon=True,
                name="a3-media-ros-executor")
            self._media_ros_thread.start()
        except Exception as exc:
            # Lightweight test doubles and old rclpy builds do not expose a
            # second executor; production falls back to the main node.
            self.media_robot = None
            self._media_ros_executor = None
            print(f"[media] dedicated executor unavailable: {exc}", flush=True)
        self.core_bridge = None
        if config.get("ros", {}).get("core_bridge", False):
            self.core_bridge = CoreBridge()
            self.core_bridge.start()
            atexit.register(self.core_bridge.stop)
        core_node = None
        if self.core_bridge is None:
            core_node = Node("agibot_a3_driver_core", context=ros2.ctx_core)
            ros2.executor_core.add_node(core_node)

        class _CoreFacade:
            def __init__(self, node, bridge):
                self._node, self._bridge = node, bridge

            def create_publisher(self, msg_type, topic, qos):
                if self._bridge is not None:
                    return BridgePublisher(self._bridge, topic, msg_type)
                return self._node.create_publisher(msg_type, topic, qos)

            def create_subscription(self, msg_type, topic, callback, qos):
                if self._bridge is not None:
                    return self._bridge.subscribe(topic, msg_type, callback, qos)
                return self._node.create_subscription(msg_type, topic, callback, qos)

            def destroy_subscription(self, subscription):
                if self._bridge is not None:
                    return self._bridge.unsubscribe(subscription)
                return self._node.destroy_subscription(subscription)

            def destroy_node(self):
                if self._node is not None:
                    return self._node.destroy_node()

            def __getattr__(self, name):
                if self._node is None:
                    raise AttributeError(name)
                return getattr(self._node, name)

        self.core = _CoreFacade(core_node, self.core_bridge)

        self.lock = threading.RLock()
        self.values = {}
        self._json_stream_pubs = {}

        self._String = String
        self._JointState = JointState
        self._Image = Image
        self._AudioCapture = AudioCapture
        self._AudioPlayback = AudioPlayback
        self._AudioChunk = AudioChunk
        self._UInt8MultiArray = UInt8MultiArray
        self._audio_buffers = {"mic": bytearray(), "ext_mic": bytearray()}
        self._audio_lock = threading.Lock()
        self._audio_received = {"mic": 0, "ext_mic": 0}
        self._audio_queues = {"mic": queue.Queue(maxsize=8),
                              "ext_mic": queue.Queue(maxsize=8)}
        self._audio_threads = {}
        self._audio_thread_lock = threading.Lock()

        # A3 HAL media publishers use ROS' sensor-data profile.  Keep this
        # exact profile: a RELIABLE subscriber is incompatible with the
        # robot's BEST_EFFORT writers under CycloneDDS.
        sensor_qos = qos_profile_sensor_data
        # The A3 audio publisher is RELIABLE. BEST_EFFORT does not match it in
        # DDS, so the old subscription could be discovered but receive zero
        # samples. Keep camera/lidar on sensor QoS, audio on a reliable profile.
        audio_qos = QoSProfile(depth=10, reliability=getattr(QoSReliabilityPolicy, "RELIABLE",
                                                              QoSReliabilityPolicy.BEST_EFFORT),
                                durability=QoSDurabilityPolicy.VOLATILE)

        self.streams = {}
        self._joint_cache = {}
        self._joint_cache_lock = threading.RLock()
        # Joint feedback can be high-rate. Give it a separate mutually
        # exclusive group so it cannot serialize IMU/BMS/estop callbacks on
        # the robot executor's default group.
        self._joint_callback_group = MutuallyExclusiveCallbackGroup()
        self._joint_skeleton_pub = self.core.create_publisher(
            String, _core_topic(namespace, "state/joints"), 5)
        self._joint_state_pub = self.core.create_publisher(
            String, _core_topic(namespace, "state/joint_state"), 5)
        # Publish an explicit unavailable skeleton until real feedback arrives.
        # Filling missing joints with zero would fabricate a standing pose.
        self._publish_joint_streams()
        self._joint_timer = None
        if hasattr(self.robot, "create_timer"):
            self._joint_timer = self.robot.create_timer(0.1, self._publish_joint_streams)
        self._imu_pub = self.core.create_publisher(
            String, _core_topic(namespace, "state/imu"), 5)
        self._media_executor = ThreadPoolExecutor(max_workers=8, thread_name_prefix="a3_media")
        # rclpy's default callback group is mutually exclusive.  A high-rate
        # JointState callback can otherwise occupy the only group while image
        # and PointCloud2 callbacks wait forever, even with a MultiThreadedExecutor.
        # Keep heavyweight media subscriptions in their own re-entrant group.
        self._media_callback_group = ReentrantCallbackGroup()
        self._encode_lock = threading.Lock()
        self._encoding_keys = set()
        self._media_failures = set()
        self._media_received = {}
        self._stream_errors = {}
        self.clock = getattr(self.robot, 'get_clock', lambda: _FakeClock())()
        self._wrapper_seq = {}
        self._wrapper_lock = threading.Lock()
        self._jazzy_relay = os.environ.get("A3_JAZZY_RELAY") == "1"
        if self._jazzy_relay:
            _ensure_relay_defaults()

        def mirror(key, msg_type, robot_topic, fmt, qos=None, json_filter=None,
                   re_encode=None):
            core_topic = _core_topic(namespace, key)
            as_json = fmt == "data/json"
            if key == "lidar_cloud":
                core_msg_type = UInt8MultiArray
            elif re_encode is not None or (self._jazzy_relay and key.startswith("camera_")):
                core_msg_type = self._CompressedImage
            elif as_json:
                core_msg_type = String
            else:
                core_msg_type = msg_type
            pub = self.core.create_publisher(core_msg_type, core_topic, 5)
            if as_json:
                self._json_stream_pubs[key] = pub

            def callback(msg):
                if (re_encode is not None or key in ("lidar_cloud", "mic", "ext_mic")
                        or (self._jazzy_relay and key.startswith("camera_"))):
                    count = self._media_received.get(key, 0) + 1
                    self._media_received[key] = count
                    interval = 300 if self._jazzy_relay and key.startswith("camera_") else 1000
                    if count == 1 or count % interval == 0:
                        print(f"[media] received={count} key={key} topic={robot_topic}", flush=True)
                if key in ("arm_state", "hand_state", "neck_state", "waist_state", "leg_state"):
                    with self._joint_cache_lock:
                        self._joint_cache[key] = jsonable(msg)
                        self._publish_joint_streams()
                elif key in ("imu_pelvis", "imu_torso"):
                    self._publish_imu_streams(key, jsonable(msg))
                    # The IMU quaternion is the root orientation in the
                    # skeleton contract. Re-emit the aggregate pose whenever
                    # IMU data changes, otherwise the model stays at the
                    # orientation captured when the last joint packet arrived.
                    self._publish_joint_streams()
                elif key in ("battery", "estop"):
                    count = self._media_received.get(key, 0) + 1
                    self._media_received[key] = count
                    if count == 1 or count % 100 == 0:
                        print(f"[state] received={count} key={key} topic={robot_topic}", flush=True)
                if key == "lidar_cloud":
                    if self._jazzy_relay:
                        pub.publish(msg)
                    else:
                        packet = self._pointcloud_packet(msg)
                        if packet is not None:
                            output = UInt8MultiArray()
                            output.data = list(packet)
                            pub.publish(output)
                    return
                if self._jazzy_relay and key.startswith("camera_"):
                    pub.publish(msg)
                    return
                if re_encode is not None:
                    # JPEG/depth conversion can be slower than the camera
                    # cadence. Never build an unbounded executor backlog: one
                    # frame per camera is in flight, newer frames replace the
                    # next sample naturally at the ROS callback.
                    if key not in self._encoding_keys:
                        with self._encode_lock:
                            if key not in self._encoding_keys:
                                self._encoding_keys.add(key)
                                future = self._media_executor.submit(
                                    self._encode_and_publish, re_encode, msg, pub, robot_topic)
                                future.add_done_callback(
                                    lambda _future, stream_key=key: self._clear_encoding(stream_key))
                    return
                if as_json:
                    try:
                        value = json_filter(msg) if json_filter else jsonable(msg)
                    except Exception as exc:
                        count = self._stream_errors.get(key, 0) + 1
                        self._stream_errors[key] = count
                        if count == 1 or count % 100 == 0:
                            print(f"[state] decode failed key={key} topic={robot_topic} "
                                  f"count={count} error={exc}", flush=True)
                        return
                    output = self._String()
                    output.data = json.dumps(value, ensure_ascii=False)
                    pub.publish(output)
                    with self.lock:
                        self.values[key] = value
                else:
                    pub.publish(msg)

            is_relay_camera = self._jazzy_relay and key.startswith("camera_")
            if key in ("arm_state", "hand_state", "neck_state", "waist_state", "leg_state"):
                callback_group = self._joint_callback_group
            else:
                callback_group = self._media_callback_group if (
                    re_encode is not None or key == "lidar_cloud" or is_relay_camera
                ) else None
            kwargs = {"callback_group": callback_group} if callback_group is not None else {}
            subscription_node = self.media_robot if (callback_group is not None and self.media_robot is not None) else self.robot
            if self._jazzy_relay and key == "lidar_cloud":
                msg_type = UInt8MultiArray
                robot_topic = _core_topic(namespace, "lidar_cloud")
            elif self._jazzy_relay and key.startswith("camera_"):
                msg_type = self._CompressedImage
                robot_topic = _core_topic(namespace, key)
            try:
                subscription_node.create_subscription(msg_type, robot_topic, callback, qos or sensor_qos, **kwargs)
            except TypeError:
                # Test doubles and old rclpy releases may not expose callback_group.
                subscription_node.create_subscription(msg_type, robot_topic, callback, qos or sensor_qos)
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

        self._encode_and_publish = self._encode_and_publish_frame

        # -- joint / state streams (plain sensor_msgs types) --
        mirror("arm_state", JointState, "/motion/control/arm_joint_state", "data/json")
        mirror("hand_state", JointState, "/motion/control/hand_joint_state", "data/json")
        mirror("neck_state", JointState, "/motion/control/neck_joint_state", "data/json")
        mirror("waist_state", JointState, "/motion/control/waist_joint_state", "data/json")
        mirror("leg_state", JointState, "/motion/control/leg_joint_state", "data/json")
        mirror("lidar_cloud", PointCloud2, "/hal/neck_middle_livox_lidar/pointcloud", "sensor/pointcloud")
        mirror("imu_pelvis", Imu, "/ros2/body_drive/pelvis_imu/data", "data/json")
        mirror("imu_torso", Imu, "/ros2/body_drive/torso_imu/data", "data/json")

        # -- camera streams, config-selected subset (re-encoded on the fly: RGB → JPEG,
        # depth → zlib uint16, per README_dev § Data Format) --
        camera_cfg = config.get("plugins", {}).get("camera", {})
        selected = camera_cfg.get("streams") or ["head_left_fisheye", "head_right_fisheye", "chest_front_d457_rgb",
                                                 "chest_front_d457_depth"]
        for key in selected:
            topic, fmt, _ = CAMERA_TOPICS[key]
            encoder = self._encode_depth if fmt == "image/depth-zlib" else self._encode_rgb
            mirror(f"camera_{key}", Image, topic, fmt, re_encode=None if self._jazzy_relay else encoder)

        # -- protobuf-carrier streams (RosMsgWrapper) -- decoded with the official
        # AimDK v3.2 generated modules.  There is no ``aimdk.protocol_pb2`` module
        # in the vendor wheel; importing that old aggregate name silently disabled
        # battery/estop on otherwise correctly provisioned robots.
        self._pb = None
        try:
            from aimdk.protocol.hal.bms import hal_bms_channel_pb2
            from aimdk.protocol.hal.state import hal_state_channel_pb2
            self._pb = {
                "bms": hal_bms_channel_pb2.BmsStateChannel,
                "estop": hal_state_channel_pb2.EmergencyStateChannel,
            }
        except (ImportError, AttributeError):
            self._pb = None
        if self._pb is not None:
            try:
                from aimdk.protocol.skill_pilot import skill_pilot_channel_pb2
                self._pb["skill"] = getattr(skill_pilot_channel_pb2, "SkillPilotStatus", None)
            except (ImportError, AttributeError):
                self._pb["skill"] = None
            try:
                from aimdk.protocol.motion_control.motion import mc_motion_channel_pb2
                self._pb.setdefault("commands", {}).update({
                    "MotionControlLocomotionVelocityChannel":
                        mc_motion_channel_pb2.MotionControlLocomotionVelocityChannel,
                    "MotionControlMoveWaistChannel":
                        mc_motion_channel_pb2.MotionControlMoveWaistChannel,
                })
            except (ImportError, AttributeError):
                pass
            try:
                from aimdk.protocol.aim_master import am_channel_pb2
                self._pb.setdefault("commands", {})["HFAEmoction"] = am_channel_pb2.HFAEmoction
            except (ImportError, AttributeError):
                pass
        # ProcessedAudioOutput is optional across A3 SDK wheel revisions. It
        # must never disable the independent BMS/emergency decoders.
        if self._pb is not None:
            try:
                from aimdk.protocol.agent import agent_data_pb2
                self._pb["processed_audio"] = agent_data_pb2.ProcessedAudioOutput
            except (ImportError, AttributeError):
                pass
        wrapper_type = self._probe_wrapper_type() if self._pb is not None else None
        if self._pb is not None and wrapper_type is not None:
            bms_topic = "/aima/bms/data/pb_3Aaimdk_2Eprotocol_2EBmsStateChannel"
            estop_topic = "/hal_state/emergency/pb_3Aaimdk_2Eprotocol_2EEmergencyStateChannel"
            # BMS/emergency writers on the robot use the sensor-data
            # (BEST_EFFORT) profile.  Do not force RELIABLE here: DDS treats
            # that as incompatible and the state stream then stays empty.
            mirror("battery", wrapper_type, bms_topic, "data/json",
                   json_filter=self._decode_bms)
            mirror("estop", wrapper_type, estop_topic, "data/json",
                   json_filter=self._decode_emergency)
            mirror("skill_status", wrapper_type, "/skill/pilot/skill_status",
                   "data/json", json_filter=self._decode_skill_status)
            print(f"[state] subscribed battery={bms_topic} estop={estop_topic}", flush=True)
        else:
            missing = "a3_aimdk protobuf" if self._pb is None else "ros2_plugin_proto RosMsgWrapper"
            print(f"[state] battery/estop subscriptions unavailable: missing {missing}", flush=True)

        # BMS and emergency channels are event/state reports, not high-rate
        # sensors.  Republish the latest decoded value once per second so the
        # dashboard stays fresh even when the robot only emits on change.
        if hasattr(self.robot, "create_timer"):
            self._slow_state_timer = self.robot.create_timer(1.0, self._republish_slow_states)

        self.audio_topics = {}
        self._audio_publishers = {}
        self.speaker_subscription = None
        if AudioCapture is not None and AudioPlayback is not None and AudioChunk is not None:
            # This A3 firmware has no writer on /agent/process_audio_output;
            # its live capture endpoint is /audiohal/audio/capture.  Keep the
            # external card on its own route rather than duplicating internal
            # microphone audio into it.
            for key, topic in (("mic", "/audiohal/audio/capture"),
                               ("ext_mic", "/agent/audio/data/external")):
                core_topic = _core_topic(namespace, f"{key}/audio")
                pub = self.core.create_publisher(AudioChunk, core_topic, 5)
                self._audio_publishers[key] = pub
                try:
                    (self.media_robot or self.robot).create_subscription(
                        AudioCapture, topic,
                        lambda msg, pub=pub, key=key: self._publish_audio(pub, key, msg), audio_qos,
                        callback_group=self._media_callback_group)
                except TypeError:
                    (self.media_robot or self.robot).create_subscription(
                        AudioCapture, topic,
                        lambda msg, pub=pub, key=key: self._publish_audio(pub, key, msg), audio_qos)
                self.audio_topics[key] = {"robot_topic": topic, "topic": core_topic,
                                          "format": "audio/pcm-16k"}


        # -- command publishers (robot domain) --
        self.wrapper_available = self._probe_wrapper_type() is not None
        if self.wrapper_available:
            self.locomotion_pub = self.robot.create_publisher(
                self._wrapper_msg_type, "/motion/control/locomotion_velocity/pb_3Aaimdk_2Eprotocol_2EMotionControlLocomotionVelocityChannel", 10)
            self.waist_pub = self.robot.create_publisher(
                self._wrapper_msg_type, "/motion/control/move_waist/pb_3Aaimdk_2Eprotocol_2EMotionControlMoveWaistChannel", 10)
            self.face_play_pub = self.robot.create_publisher(
                self._wrapper_msg_type,
                "/skill/pilot/face/play/pb_3Aaimdk_2Eprotocol_2EHFAEmoction", 10)
        else:
            self.locomotion_pub = None
            self.waist_pub = None
            self.face_play_pub = None
            print("[warn] ros2_plugin_proto not importable — loco/waist_control/face_play "
                  "command publishers withheld (install the AimDK dev-kit prebuilt)")
        self.arm_command_pub = self.robot.create_publisher(JointState, "/motion/control/arm_joint_command", 10)
        self.neck_command_pub = self.robot.create_publisher(JointState, "/motion/control/neck_joint_command", 10)
        self.hand_command_pub = self.robot.create_publisher(JointState, "/motion/control/hand_joint_command", 10)

    def _republish_slow_states(self):
        with self.lock:
            values = {key: self.values.get(key) for key in ("battery", "estop")}
        for key, value in values.items():
            pub = self._json_stream_pubs.get(key)
            if pub is None or not value:
                continue
            output = self._String()
            output.data = json.dumps(value, ensure_ascii=False)
            pub.publish(output)

    def _audio_chunk(self, msg):
        out = self._AudioChunk()
        # Match phanthymotus/deploy/ros-base/audio_msgs exactly.  Agent Core
        # resolves audio/pcm-16k to that AudioChunk type, which includes a
        # std_msgs/Header; omitting it gives the bridge a different ROS type
        # hash even though the publisher appears in logs.
        # AudioCapture timestamps describe capture time and can arrive in
        # bursts after a relay backlog.  Reusing them makes the frontend
        # treat freshly published chunks as stale/future audio.  The paced
        # publisher stamps each chunk at publication time instead.
        payload = getattr(msg, "data", None)
        if hasattr(payload, "data"):
            payload = payload.data
        if payload is None:
            payload = getattr(msg, "pcm", None)
        if payload is None:
            payload = getattr(msg, "samples", b"")
        try:
            payload = bytes(payload or b"")
        except (TypeError, ValueError):
            payload = b""
        payload = self._normalize_audio_payload(msg, payload)
        out.format = "audio/pcm-16k"
        out.data = list(payload)
        return out

    @staticmethod
    def _normalize_audio_payload(msg, payload):
        """Convert common HAL PCM formats to the Core mono 16 kHz contract."""
        # AudioCapture.msg defines AudioInfo info and AudioData data; the
        # metadata belongs to info while data contains only the byte sequence.
        info = getattr(msg, "info", None)
        rate = int(getattr(info, "sample_rate", 16000) or 16000)
        info_channels = int(getattr(info, "channels", 0) or 0)
        mic_channels = int(getattr(msg, "mic_channels", 0) or 0)
        ref_channels = int(getattr(msg, "ref_channels", 0) or 0)
        channels = max(1, info_channels, mic_channels + ref_channels)
        # Some SDK builds omit mic_channels but still report the reference
        # channel.  Averaging all channels cancels the true microphone signal.
        selected_channels = max(1, mic_channels or (info_channels - ref_channels) or info_channels or 1)
        sample_format = str(getattr(info, "sample_format", "") or "").upper().replace("-", "_")
        coding_format = str(getattr(info, "coding_format", "") or "").upper().replace("-", "_")
        if sample_format in {"S16_LE", "PCM16", "PCM_16", "SIGNED16"}:
            sample_format = "S16LE"
        if coding_format in {"PCM16", "PCM_16", "PCM_S16_LE"}:
            coding_format = "PCM_S16LE"
        if sample_format not in {"", "S16LE", "PCM_S16LE", "SIGNED_16"}:
            return b""
        if coding_format not in {"", "PCM", "PCM_S16LE", "S16LE"}:
            return b""
        if not payload:
            return b""
        channels = max(1, channels)
        frame_bytes = channels * 2
        payload = payload[:len(payload) - (len(payload) % frame_bytes)]
        if channels > 1:
            import struct
            samples = struct.unpack("<%dh" % (len(payload) // 2), payload)
            mono = [sum(samples[i:i + selected_channels]) // selected_channels
                    for i in range(0, len(samples), channels)]
            payload = struct.pack("<%dh" % len(mono), *mono)
        if rate != 16000 and rate > 0:
            import struct
            samples = struct.unpack("<%dh" % (len(payload) // 2), payload)
            target_count = max(1, round(len(samples) * 16000 / rate))
            resampled = [samples[min(len(samples) - 1,
                                     int(i * rate / 16000))]
                         for i in range(target_count)]
            payload = struct.pack("<%dh" % len(resampled), *resampled)
        return payload

    def _publish_audio(self, publisher, key, msg):
        """Queue PCM and publish ASR-sized 32 ms frames at real-time speed."""
        chunk = self._audio_chunk(msg)
        payload = bytes(getattr(chunk, "data", ()))
        self._queue_audio_payload(publisher, key, payload)

    def _publish_processed_audio(self, msg):
        """Decode the documented A3 processed-audio stream by stream_id."""
        message_type = (self._pb or {}).get("processed_audio")
        if message_type is None:
            return
        try:
            decoded = message_type()
            decoded.ParseFromString(self._wrapper_bytes(msg))
            key = {1: "mic", 2: "ext_mic"}.get(int(decoded.stream_id))
            payload = bytes(decoded.audio_data)
        except Exception as exc:
            if "processed_audio" not in self._media_failures:
                self._media_failures.add("processed_audio")
                print(f"[audio] processed audio decode failed: {type(exc).__name__}: {exc}", flush=True)
            return
        publisher = self._audio_publishers.get(key)
        if publisher is not None:
            self._queue_audio_payload(publisher, key, payload)

    def _queue_audio_payload(self, publisher, key, payload):
        """Queue already-normalized mono 16 kHz PCM with bounded latency."""
        if not payload:
            return
        self._audio_received[key] = self._audio_received.get(key, 0) + 1
        if self._audio_received[key] == 1 or self._audio_received[key] % 500 == 0:
            print(f"[audio] received={self._audio_received[key]} key={key} bytes={len(payload)}", flush=True)
        with self._audio_lock:
            buffer = self._audio_buffers.setdefault(key, bytearray())
            buffer.extend(payload)
            # Bound latency to roughly 128 ms; old audio is less useful than
            # dropping it when the consumer is behind.
            if len(buffer) > 4096:
                del buffer[:-4096]
            frames = []
            # The HAL supplies 320 bytes per 10 ms. Agent Core's ASR drops
            # frames below 1024 bytes, so coalesce to 512 mono S16 samples
            # (32 ms) before pacing them downstream.
            frame_bytes = 1024
            while len(buffer) >= frame_bytes:
                frames.append(bytes(buffer[:frame_bytes]))
                del buffer[:frame_bytes]
        q = self._audio_queues[key]
        for frame in frames:
            try:
                q.put_nowait((publisher, frame, None))
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    continue
                try:
                    q.put_nowait((publisher, frame, None))
                except queue.Full:
                    pass
        with self._audio_thread_lock:
            if key not in self._audio_threads:
                self._audio_threads[key] = threading.Thread(
                    target=self._audio_pump, args=(key,), daemon=True,
                    name=f"a3-{key}-pacer")
                self._audio_threads[key].start()

    def _audio_pump(self, key):
        q = self._audio_queues[key]
        next_deadline = time.monotonic()
        while True:
            publisher, data, header = q.get()
            delay = next_deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            output = self._AudioChunk()
            try:
                # Use the local clock after pacing, not the source capture
                # timestamp, so downstream buffering tracks this stream.
                output.header.stamp = self.clock.now().to_msg()
            except (AttributeError, TypeError):
                if header is not None:
                    try:
                        output.header = header
                    except AttributeError:
                        pass
            output.format = "audio/pcm-16k"
            output.data = list(data)
            publisher.publish(output)
            # Never catch up by publishing several old frames back-to-back.
            # Once late, restart the real-time clock and discard queued stale
            # frames through the bounded queue instead.
            next_deadline += 0.032
            now = time.monotonic()
            if next_deadline < now - 0.020:
                next_deadline = now

    def hand_command_layout(self, hand_type):
        """Use the real channel names reported by the installed hand."""
        state = self._joint_cache.get("hand_state") or {}
        live_type = str(state.get("header", {}).get("frame_id", ""))
        selected = live_type if hand_type == "AgiHand" and live_type in HAND_TYPES else hand_type
        names = [str(name) for name in state.get("name", [])]
        return selected, [n for n in names if n.startswith("left_")], [n for n in names if n.startswith("right_")]

    @staticmethod
    def _pointcloud_packet(msg):
        """Encode XYZ PointCloud2 as the UInt8MultiArray sensor/pointcloud contract."""
        try:
            import struct
            fields = {field.name: field.offset for field in msg.fields}
            if not all(name in fields for name in ("x", "y", "z")):
                return None
            endian = ">" if msg.is_bigendian else "<"
            points = []
            for row in range(int(msg.height)):
                base = row * int(msg.row_step)
                for col in range(int(msg.width)):
                    offset = base + col * int(msg.point_step)
                    try:
                        points.append((
                            struct.unpack_from(endian + "f", msg.data, offset + fields["x"])[0],
                            struct.unpack_from(endian + "f", msg.data, offset + fields["y"])[0],
                            struct.unpack_from(endian + "f", msg.data, offset + fields["z"])[0]))
                    except struct.error:
                        continue
            # Match phanthymotus' sensor/pointcloud contract.  The consumer
            # caps at 40000 points, while producers should decimate to 20000
            # to keep DDS/Core latency bounded.
            points = points[:20000]
            return struct.pack("<II", 12, len(points)) + b"".join(
                struct.pack("<fff", *point) for point in points)
        except Exception:
            return None

    def _clear_encoding(self, key):
        with self._encode_lock:
            self._encoding_keys.discard(key)

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
            raw = np.frombuffer(bytes(msg.data), np.uint8)
            step_value = getattr(msg, "step", 0)
            step = int(step_value) if isinstance(step_value, (int, float)) else width * 3
            if step < width * 3 or raw.size < step * height:
                raise ValueError(f"invalid image layout step={step} width={width} height={height} bytes={raw.size}")
            img = raw[:step * height].reshape(height, step)[:, :width * 3].reshape(height, width, 3)
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
            raw = np.frombuffer(bytes(msg.data), np.uint8)
            step_value = getattr(msg, "step", 0)
            step = int(step_value) if isinstance(step_value, (int, float)) else width * 2
            if step < width * 2 or raw.size < step * height:
                raise ValueError(f"invalid depth layout step={step} width={width} height={height} bytes={raw.size}")
            depth = raw[:step * height].reshape(height, step)[:, :width * 2].reshape(height, width, 2)
            depth = depth.view(np.uint16).reshape(height, width)
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

    def _make_wrapper(self, topic: str, proto_dict: dict):
        """Serialize a protobuf payload dict into a RosMsgWrapper message.

        Mirrors what AimRT's ros2 channel backend itself publishes (observed on
        the robot): ``serialization_type="pb"``, ``context`` as flat key/value
        pairs (``aimrt-pub_ts``/``aimrt-pub_seq``/``aimrt-serialization_type``)
        and ``data`` as protobuf bytes.  The Humble binding of ``byte[]`` only
        accepts a sequence of 1-byte ``bytes`` objects.
        """
        payload = self._encode_pb(topic, proto_dict)
        wrapper = self._wrapper_type(None)()
        seq = self._wrapper_seq.get(topic, 0) + 1
        self._wrapper_seq[topic] = seq
        wrapper.serialization_type = "pb"
        wrapper.context = ["aimrt-pub_ts", str(time.time_ns()),
                           "aimrt-pub_seq", str(seq),
                           "aimrt-serialization_type", "pb"]
        wrapper.data = [bytes([b]) for b in payload]
        return wrapper

    _PB_TOPIC_MESSAGES = {
        "/motion/control/locomotion_velocity/pb_3Aaimdk_2Eprotocol_2EMotionControlLocomotionVelocityChannel": "MotionControlLocomotionVelocityChannel",
        "/motion/control/move_waist/pb_3Aaimdk_2Eprotocol_2EMotionControlMoveWaistChannel": "MotionControlMoveWaistChannel",
        "/skill/pilot/face/play/pb_3Aaimdk_2Eprotocol_2EHFAEmoction": "HFAEmoction",
    }

    def _encode_pb(self, topic, proto_dict):
        """Encode to real protobuf bytes; never fall back to JSON on the wire
        (the robot-side subscriber is told ``pb`` and would mis-parse it)."""
        message_type = self._PB_TOPIC_MESSAGES.get(topic, "")
        cls = (self._pb or {}).get("commands", {}).get(message_type) if message_type else None
        if cls is None:
            raise RuntimeError(f"no protobuf type for {topic!r} (a3_aimdk wheel missing?)")
        message = cls()
        _fill_protobuf(message, proto_dict)
        return message.SerializeToString()

    def publish_wrapper(self, pub_attr, payload):
        """Fill + publish a RosMsgWrapper command onto a robot-domain topic."""
        publisher = getattr(self, pub_attr)
        if publisher is None:  # wrapper type unavailable — degraded dev mode
            raise RuntimeError(
                f"{pub_attr} unavailable: ros2_plugin_proto not importable "
                "(install the AimDK dev-kit prebuilt to publish wrapper commands)")
        topic = getattr(publisher, "topic_name", "") or getattr(publisher, "topic", "")
        with self._wrapper_lock:
            wrapper = self._make_wrapper(topic, payload)
        publisher.publish(wrapper)

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

    @staticmethod
    def _wrapper_bytes(msg):
        data = getattr(msg, "data", b"")
        if isinstance(data, (list, tuple)):
            # ROS byte[] bindings vary by rclpy version: integer octets,
            # one-byte ``bytes`` values, or larger bytes chunks are all seen in
            # the field.  Normalize each representation without interpreting a
            # byte value as a zero-filled length.
            if all(isinstance(part, int) for part in data):
                return bytes((part & 0xff) for part in data)
            return b"".join(
                bytes((part & 0xff,)) if isinstance(part, int) else bytes(part)
                for part in data
            )
        return bytes(data)

    def _decode_bms(self, msg):
        if self._pb is None:
            return self._wrapper_json(msg)
        channel = self._pb["bms"]()
        try:
            channel.ParseFromString(self._wrapper_bytes(msg))
        except Exception:
            self._log_pb_decode_failure("battery", msg)
            return self._wrapper_json(msg)
        return self._protobuf_flat_json(channel)

    def _decode_emergency(self, msg):
        if self._pb is None:
            return self._wrapper_json(msg)
        channel = self._pb["estop"]()
        try:
            channel.ParseFromString(self._wrapper_bytes(msg))
        except Exception:
            self._log_pb_decode_failure("estop", msg)
            return self._wrapper_json(msg)
        return self._protobuf_flat_json(channel)

    def _log_pb_decode_failure(self, key, msg):
        count = self._stream_errors.get(key, 0) + 1
        self._stream_errors[key] = count
        if count == 1 or count % 100 == 0:
            print(f"[state] protobuf decode failed key={key} count={count} "
                  f"bytes={len(self._wrapper_bytes(msg))}", flush=True)

    @staticmethod
    def _protobuf_json(message):
        """Convert generated protobufs without leaking Descriptor internals."""
        try:
            from google.protobuf.json_format import MessageToDict
            return MessageToDict(message, preserving_proto_field_name=True)
        except Exception:
            return {field.name: jsonable(value) for field, value in message.ListFields()}

    @classmethod
    def _protobuf_flat_json(cls, message):
        """Return scalar JSON fields suitable for the dashboard value grid."""
        nested = cls._protobuf_json(message)
        flat = {}
        _flatten_json(nested, "", flat)
        return flat

    @classmethod
    def _wrapper_json(cls, msg):
        return {
            "serialization_type": str(getattr(msg, "serialization_type", "")),
            "context": [str(item) for item in getattr(msg, "context", [])],
            "data_size": len(cls._wrapper_bytes(msg)),
        }

    def _decode_skill_status(self, msg):
        if self._pb is None:
            return jsonable(msg)
        status_type = self._pb.get("skill")
        if status_type is None:
            return jsonable(msg)
        status = status_type()
        try:
            status.ParseFromString(self._wrapper_bytes(msg))
        except Exception:
            return jsonable(msg)
        return jsonable(status)

    # -- generic mirror callback ------------------------------------------------

    def snapshot(self, key):
        with self.lock:
            return self.values.get(key, {})

    def _publish_joint_streams(self):
        with self._joint_cache_lock:
            groups = {key: self._joint_cache.get(key, {})
                      for key in ("arm_state", "hand_state", "neck_state", "waist_state", "leg_state")}
        raw = {}
        for prefix, group in (("arm", groups["arm_state"]), ("hand", groups["hand_state"]),
                              ("neck", groups["neck_state"]), ("waist", groups["waist_state"]),
                              ("leg", groups["leg_state"])):
            _flatten_joint_group(group, prefix, raw)
        output = self._String()
        output.data = json.dumps(raw, ensure_ascii=False)
        self._joint_state_pub.publish(output)
        joints = []
        seen = set()
        model_indices = _skeleton_joint_indices()
        group_aliases = {
            "arm_state": [*ARM_JOINTS["left"], *ARM_JOINTS["right"]],
            "neck_state": list(NECK_JOINTS),
            "waist_state": ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            "leg_state": [
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            ],
        }
        for group_key, group in groups.items():
            names = group.get("name", []) if isinstance(group, dict) else []
            positions = group.get("position", []) if isinstance(group, dict) else []
            velocities = group.get("velocity", []) if isinstance(group, dict) else []
            efforts = group.get("effort", []) if isinstance(group, dict) else []
            aliases = group_aliases.get(group_key, [])
            hand_side_counts = {"left": 0, "right": 0}
            hand_values = {}
            hand_names = [str(item).lower() for item in names]
            # Some AimDK builds publish one 20-channel JointState for both
            # O10 hands with generic names (hand_joint_0..19). Infer the side
            # from the documented left-then-right channel layout in that
            # case; named per-side channels still take precedence below.
            combined_hand_channels = (
                group_key == "hand_state" and len(names) >= 20 and
                not any(name.startswith(("left", "right", "l_", "r_"))
                        for name in hand_names))
            for index, name in enumerate(names):
                # AimDK firmware revisions have emitted generic channel names
                # (for example arm_joint_0) while preserving the documented
                # order. Map those names by position so the skeleton is still
                # usable; exact URDF names remain preferred.
                canonical = name if name in model_indices else (
                    aliases[index] if index < len(aliases) else name)
                if group_key == "hand_state":
                    text_name = str(name).lower()
                    if text_name in HAND_JOINT_ALIASES:
                        canonical = HAND_JOINT_ALIASES[text_name]
                    side = ("left" if text_name.startswith("left") else
                            "right" if text_name.startswith("right") else
                            "left" if text_name.startswith("l_") else
                            "right" if text_name.startswith("r_") else None)
                    if side is None and combined_hand_channels:
                        side = "left" if index < 10 else "right"
                    if canonical not in HAND_JOINT_LIMITS and side is not None:
                        channel = (index % 10 if combined_hand_channels
                                   else hand_side_counts[side])
                        if not combined_hand_channels:
                            hand_side_counts[side] += 1
                        if channel < len(HAND_SKELETON_JOINTS[side]):
                            canonical = HAND_SKELETON_JOINTS[side][channel]
                    if canonical not in HAND_JOINT_LIMITS:
                        continue
                # Only actuated joints that exist in the model belong in the
                # skeleton stream. Unknown metadata/fixed sensor joints remain
                # available in joint_state's raw JSON stream.
                if canonical not in model_indices:
                    continue
                position = positions[index] if index < len(positions) else 0.0
                velocity = velocities[index] if index < len(velocities) else 0.0
                if group_key == "hand_state":
                    low, high = HAND_JOINT_LIMITS[canonical]
                    count = min(HAND_STATE_COUNT_MAX, max(0.0, float(position)))
                    direction = 1.0
                    if canonical in HAND_INVERTED_COUNT_JOINTS:
                        # These feedback counts increase opposite to the
                        # matching URDF joint direction.
                        count = HAND_STATE_COUNT_MAX - count
                        direction = -1.0
                    scale = (high - low) / HAND_STATE_COUNT_MAX
                    position = low + count * scale
                    velocity = float(velocity) * scale * direction
                    hand_values[canonical] = (position, velocity)
                joints.append({"idx": model_indices[canonical], "name": canonical,
                               "q": position,
                               "dq": velocity,
                               "tau": efforts[index] if index < len(efforts) else 0.0})
                seen.add(canonical)
            for mimic, (source, multiplier) in HAND_MIMIC_JOINTS.items():
                if group_key != "hand_state" or source not in hand_values:
                    continue
                position, velocity = hand_values[source]
                joints.append({"idx": model_indices[mimic], "name": mimic,
                               "q": position * multiplier,
                               "dq": velocity * multiplier, "tau": 0.0})
                seen.add(mimic)
        output = self._String()
        imu_quat = _imu_quaternion(self.values.get("imu_pelvis") or {})
        output.data = json.dumps({"format": "sensor/skeleton", "available": bool(joints),
                                  "fresh": bool(joints), "complete": len(seen) == len(model_indices),
                                  "coverage": len(seen), "joint_count": len(joints),
                                  "timestamp_ms": int(time.time() * 1000),
                                  **({"imu_quat": imu_quat} if imu_quat is not None else {}),
                                  "joints": joints}, ensure_ascii=False)
        self._joint_skeleton_pub.publish(output)

    def _publish_imu_streams(self, key, value):
        self.values[key] = value
        payload = {"available": False}
        for prefix, key in (("pelvis", "imu_pelvis"), ("torso", "imu_torso")):
            value = self.values.get(key, {})
            if not isinstance(value, dict) or not value:
                continue
            payload["available"] = True
            header = value.get("header") or {}
            stamp = header.get("stamp") or {}
            if stamp:
                payload[f"{prefix}_stamp_sec"] = stamp.get("sec", 0)
                payload[f"{prefix}_stamp_nanosec"] = stamp.get("nanosec", 0)
            if header.get("frame_id"):
                payload[f"{prefix}_frame_id"] = header["frame_id"]
            for field in ("orientation", "angular_velocity", "linear_acceleration"):
                vector = value.get(field) or {}
                for axis in ("x", "y", "z", "w"):
                    if axis in vector:
                        payload[f"{prefix}_{field}_{axis}"] = vector[axis]
        output = self._String()
        output.data = json.dumps(payload, ensure_ascii=False)
        self._imu_pub.publish(output)

    def _encode_and_publish_frame(self, encoder, msg, pub, topic):
        try:
            out = encoder(msg)
            if out is not None:
                pub.publish(out)
            elif topic not in self._media_failures:
                self._media_failures.add(topic)
                print(f"[media] encoder produced no frame topic={topic} "
                      f"encoding={getattr(msg, 'encoding', '')} "
                      f"size={getattr(msg, 'width', 0)}x{getattr(msg, 'height', 0)} "
                      f"step={getattr(msg, 'step', 0)}", flush=True)
        except Exception as exc:
            print(f"[media] encode failed topic={topic}: {exc}", flush=True)

    def speaker_start(self, input_topic: str):
        if self._AudioChunk is None or self._AudioPlayback is None:
            raise RuntimeError("audio_msgs package is unavailable; source the AimDK ROS package")
        self.speaker_stop()
        self.speaker_pub = getattr(self, "speaker_pub", None) or self.robot.create_publisher(
            self._AudioPlayback, "/audiohal/audio/playback", 10)

        def callback(msg):
            out = self._AudioPlayback()
            if hasattr(out, "stamps") and hasattr(msg, "header"):
                out.stamps = msg.header.stamp
            out.data = msg.data
            out.pkg_name = "phanthymotus"
            out.token_id = "phanthymotus"
            self.speaker_pub.publish(out)

        self.speaker_subscription = self.core.create_subscription(
            self._AudioChunk, input_topic, callback, 5)

    def speaker_stop(self):
        sub = getattr(self, "speaker_subscription", None)
        if sub is not None:
            destroy = getattr(self.core, "destroy_subscription", None)
            if destroy:
                destroy(sub)
            self.speaker_subscription = None

    def urdf_text(self, variant=None):
        # The canvas skeleton contract needs the kinematic tree and limits,
        # not the vendor CAD package. Keep this compact model self-contained
        # so deployments do not carry dozens of megabytes of unused meshes.
        if not (RESOURCE_DIR / "a3_ultra.urdf").exists():
            raise ValueError("no URDF vendored for A3 (placeholder resource)")
        # The compact vendor-neutral URDF intentionally omits CAD geometry,
        # but Three.js still needs link offsets to draw a visible skeleton.
        # Supply the measured A3 kinematic proportions at the driver boundary;
        # no frontend-specific model changes are required.
        origins = {
            "waist_roll_joint": (0, 0, 0.08), "waist_yaw_joint": (0, 0, 0.12),
            "waist_pitch_joint": (0, 0, 0.12), "torso_joint": (0, 0, 0.22),
            "head_yaw_joint": (0, 0, 0.42), "head_pitch_joint": (0, 0, 0.10),
            "left_shoulder_pitch_joint": (0.22, 0, 0.30),
            "left_shoulder_roll_joint": (0.08, 0, 0),
            "left_shoulder_yaw_joint": (0.16, 0, 0),
            "left_elbow_joint": (0.24, 0, 0),
            "left_wrist_roll_joint": (0.20, 0, 0),
            "left_wrist_pitch_joint": (0.08, 0, 0),
            "left_wrist_yaw_joint": (0.08, 0, 0),
            "right_shoulder_pitch_joint": (-0.22, 0, 0.30),
            "right_shoulder_roll_joint": (-0.08, 0, 0),
            "right_shoulder_yaw_joint": (-0.16, 0, 0),
            "right_elbow_joint": (-0.24, 0, 0),
            "right_wrist_roll_joint": (-0.20, 0, 0),
            "right_wrist_pitch_joint": (-0.08, 0, 0),
            "right_wrist_yaw_joint": (-0.08, 0, 0),
            "left_hand_joint": (0.08, 0, 0), "right_hand_joint": (-0.08, 0, 0),
            "left_hip_pitch_joint": (0.12, 0, -0.12),
            "left_hip_roll_joint": (0, 0, -0.12), "left_hip_yaw_joint": (0, 0, -0.18),
            "left_knee_joint": (0, 0, -0.38), "left_ankle_pitch_joint": (0, 0, -0.38),
            "left_ankle_roll_joint": (0, 0, -0.08),
            "right_hip_pitch_joint": (-0.12, 0, -0.12),
            "right_hip_roll_joint": (0, 0, -0.12), "right_hip_yaw_joint": (0, 0, -0.18),
            "right_knee_joint": (0, 0, -0.38), "right_ankle_pitch_joint": (0, 0, -0.38),
            "right_ankle_roll_joint": (0, 0, -0.08),
        }
        root = _a3_urdf_root()
        for joint in root.findall("joint"):
            origin = origins.get(joint.get("name"))
            if origin is not None and joint.find("origin") is None:
                ET.SubElement(joint, "origin", xyz="%g %g %g" % origin, rpy="0 0 0")
        return ET.tostring(root, encoding="unicode")

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()
        if self.core_bridge is not None:
            self.core_bridge.stop()


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

    def _unit(self, host, port, timeout=None):
        return RpcClient(f"{host}:{port}", timeout or self.timeout, self.transport)

    # -- MDU :56322 MotionControlAction/MotionService --------------------------------

    def set_action(self, command: dict) -> dict:
        """SetAction exactly as the vendor mc_action_cli.py does: ``command`` is
        the entry returned by GetAvailableActions/GetNextActions, sent verbatim.
        Returns CommonResponse; a ``state`` other than SUCCESS is a rejection."""
        payload = {"header": create_header(),
                   "command": {k: v for k, v in command.items() if not str(k).startswith("_")}}
        response = self._unit(self.mdu_port, 56322).call("MotionControlActionService", "SetAction", payload)
        state = response.get("state")
        if state is not None and str(state) not in ("1", "CommonState_SUCCESS"):
            raise RpcError(f"MotionControlActionService/SetAction 拒绝请求: state={state!r} "
                           f"msg={(response.get('header') or {}).get('msg', '')!r}")
        return response

    def get_action(self, timeout=None) -> dict:
        return self._unit(self.mdu_port, 56322, timeout).call("MotionControlActionService", "GetAction")

    def get_next_actions(self) -> dict:
        return self._unit(self.mdu_port, 56322).call("MotionControlActionService", "GetNextActions")

    def get_velocity_limit(self) -> dict:
        return self._unit(self.mdu_port, 56322).call("MotionControlMotionService", "GetVelocityLimitValue")

    # -- MDU :52893 SkillPilotService (MDU gateway routes SkillPilotService.* here) ---

    def postural_transition(self, kind: str) -> dict:
        """PosturalTransition, the stand/lie-down entry the remote and App use.

        skillpilot keeps its own posture state (Core Initial = lying, Idle =
        standing) and only updates it in this flow; it then drives MC itself
        (GetUp: [PD_STAND →] GET_UP → MOTION, LieDown: LIE_DOWN → PASSIVE).
        ``kind``: GetUp / LieDown. Request per the descriptor embedded in
        libaimdk_protocol_skill_pilot_service_type_support.so: field 1 ``header``
        is a ResponseHeader (left empty), the request header goes in
        ``req_header``. Returns SkillCommandResponse; rejection = header.code≠0.
        """
        payload = {"type": f"PosturalTransitionType_{kind}", "req_header": create_header()}
        return self._unit(self.mdu_port, 52893).call("SkillPilotService", "PosturalTransition", payload)

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


def _set_relay_stream(stream_name, enabled):
    """Toggle one host-relay encoder through the host-mounted control file."""
    path = os.environ.get("A3_RELAY_CONTROL_FILE", "/opt/phanthy-motus/data/a3-relay/active_streams")
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        current = set()
        try:
            with open(path, encoding="utf-8") as handle:
                current = {line.strip() for line in handle if line.strip()}
        except OSError:
            pass
        if enabled:
            current.add(stream_name)
        else:
            current.discard(stream_name)
        temporary = f"{path}.tmp.{os.getpid()}"
        with open(temporary, "w", encoding="utf-8") as handle:
            handle.write("".join(f"{item}\n" for item in sorted(current)))
        os.replace(temporary, path)
    except OSError as exc:
        print(f"[relay] stream activation unavailable: {exc}", flush=True)


def _ensure_relay_defaults():
    """Seed the shared relay state once per deployment.

    Seed the relay with the configured startup set. Camera lifecycle callbacks
    may be absent when the frontend only subscribes to the advertised topic,
    so the relay must have a stable set of inputs before any card is opened.
    """
    path = os.environ.get("A3_RELAY_CONTROL_FILE", "/opt/phanthy-motus/data/a3-relay/active_streams")
    configured = os.environ.get("A3_RELAY_ACTIVE_STREAMS")
    defaults = configured if configured is not None else ""
    try:
        if configured is None and os.path.exists(path) and os.path.getsize(path) > 0:
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        temporary = f"{path}.tmp.{os.getpid()}"
        with open(temporary, "w", encoding="utf-8") as handle:
            handle.write("".join(f"{item.strip()}\n" for item in defaults.split(",") if item.strip()))
        os.replace(temporary, path)
    except OSError as exc:
        print(f"[relay] default activation unavailable: {exc}", flush=True)


class JointsPlugin:
    """joints 状态卡：手臂/灵巧手/头部三路关节状态流合一张卡（camera 卡同款
    多路复用模式——group 参数选择，镜像话题各自保持不变）。"""

    GROUPS = {
        "arm": ("arm_state", "14 自由度手臂关节（position/velocity/effort）"),
        "hand": ("hand_state", "手指（0-4096；frame_id 标识 AgiHand/O10Hand；O10Hand 含压力阵列）"),
        "neck": ("neck_state", "头部偏航/俯仰关节"),
        "waist": ("waist_state", "腰部偏航/滚转/俯仰关节"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self.streams = {k: nodes.streams[k] for k in
                        ("arm_state", "hand_state", "neck_state", "waist_state")}

    def get_tool(self):
        return tool("joints", "sensor", "关节状态流（按 group 选择 arm/hand/neck/waist；"
                                     "config.yaml plugins.joints 控制启用）", {
            "type": "object",
            "properties": {
                "group": {"type": "string", "enum": list(self.GROUPS),
                          "description": "关节组：arm 双臂 / hand 手指 / neck 头部 / waist 腰部"},
            },
        }, topic_out=[{"topic": _core_topic(self.nodes.namespace, "state/joints"),
                       "format": "sensor/skeleton"}])

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
            # The card renders the aggregated skeleton stream. Keep the selected
            # group's raw stream in metadata, but never replace topic_out with it:
            # Agent Core treats info().topic_out as authoritative after placement.
            return {"state": "running",
                    "topic_out": [{"topic": _core_topic(self.nodes.namespace, "state/joints"),
                                   "format": "sensor/skeleton"}],
                    "group": group,
                    "streams": dict(self.streams),
                    "urdf": self.nodes.urdf_text()}
        group = args.get("group", "arm")
        if group not in self.GROUPS:
            raise ValueError(f"joints: unknown group {group!r}; available: {list(self.GROUPS)}")
        key = self.GROUPS[group][0]
        return {"state": "running", **self.streams[key]}


class JointStatePlugin:
    """Raw grouped JointState JSON for consumers that do not render a URDF."""

    def __init__(self, nodes):
        self.nodes = nodes
        self.topic = _core_topic(nodes.namespace, "state/joint_state")

    def get_tool(self):
        return tool("joint_state", "sensor", "原始手臂/手/头 JointState 数据（JSON）",
                    {"type": "object", "properties": {}},
                    topic_out=[{"topic": self.topic, "format": "data/json"}])

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "stop":
            return {"state": "idle", "topic_out": [{"topic": self.topic, "format": "data/json"}]}
        return {"state": "running", "topic_out": [{"topic": self.topic, "format": "data/json"}]}


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
        }, topic_out=[{"topic": _core_topic(self.nodes.namespace, "state/imu"),
                       "format": "data/json"}])

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
                    "topic_out": [{"topic": _core_topic(self.nodes.namespace, "state/imu"),
                                   "format": "data/json"}],
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
        }, topic_out=self._default_topic_out())

    def _default_topic_out(self):
        stream = self._resolve({})
        return [{"topic": stream["topic"], "format": stream["format"]}]

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


class CameraStreamPlugin:
    """One fixed camera card, so canvas can subscribe to several cameras at once."""

    def __init__(self, nodes, card_name, stream_name):
        self.nodes = nodes
        self.card_name = card_name
        self.stream_name = stream_name
        self.stream = nodes.streams[f"camera_{stream_name}"]

    def get_tool(self):
        return tool(self.card_name, "sensor", f"A3 {self.stream_name} 相机流",
                    {"type": "object", "properties": {}},
                    topic_out=[{"topic": self.stream["topic"], "format": self.stream["format"]}])

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "stop":
            # Keep the raw subscription alive across canvas lifecycle events.
            # The frontend can stop rendering a card without sending a reliable
            # lifecycle event; removing the DDS subscription here makes the
            # advertised stream disappear until the next container restart.
            return {"state": "idle", "topic_out": [{"topic": self.stream["topic"], "format": self.stream["format"]}]}
        if action == "start":
            _set_relay_stream(self.stream_name, True)
            return {"state": "running", **self.stream}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self.stream["topic"], "format": self.stream["format"]}]}
        return {"state": "running", **self.stream}


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
        return tool("battery", "sensor", "电池状态（a3_aimdk protobuf 未加载）",
                    topic_out=[{"topic": _core_topic(self.nodes.namespace, "battery"),
                                "format": "data/json"}])

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
                "reason": "battery 数据流需要包含 a3_aimdk wheel 和 ros2_plugin_proto 的新 A3 镜像；"
                          "请重新构建并部署镜像后重启驱动"}


class EstopPlugin:
    def __init__(self, nodes):
        self.nodes = nodes
        self.has_stream = "estop" in nodes.streams

    def get_tool(self):
        if self.has_stream:
            return _stream_tool("estop", self.nodes.streams["estop"],
                                "急停状态流（有线/无线/软件急停 + 各类传感器报警）")
        return tool("estop", "sensor", "急停状态（a3_aimdk protobuf 未加载）",
                    topic_out=[{"topic": _core_topic(self.nodes.namespace, "estop"),
                                "format": "data/json"}])

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
                "reason": "estop 数据流需要包含 a3_aimdk wheel 和 ros2_plugin_proto 的新 A3 镜像；"
                          "请重新构建并部署镜像后重启驱动"}


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


# ---------------------------------------------------------------------------
# Motion-control state (MDU :56322 MotionControlActionService)
# ---------------------------------------------------------------------------
# The firmware's GetNextActions is the only source of truth for which posture
# switch is allowed right now — the vendor mc_action_cli.py enables an entry
# only when it is listed there and sends that entry verbatim to SetAction.  The
# driver therefore keeps no transition table of its own.

_MC_ACTION_PREFIX = "MotionControlAction_"
_MC_STATUS_PREFIX = "MotionControlActionStatus_"
# Postures the firmware holds once entered: status RUNNING is their steady state.
# GET_UP / LIE_DOWN are one-shot sequences reporting stages *_POSCTRL_ING →
# *_FORCECTRL_ING → *_FINISHED (or GET_UP_INITIAL_RESET_FAILED); names taken
# from the MDU motion_control binary, GET_UP_FINISHED also observed live.
_MC_STEADY = {"MOTION", "PD_STAND", "DAMPING", "PASSIVE"}
# Firmware action graph (action_manager/default.yaml, no indirect transitions):
# lying postures only lead to PASSIVE (zero torque), and PASSIVE → PD_STAND → GET_UP.
_MC_LYING = {"LIE_DOWN", "PACKAGE_LIE_DOWN"}


def _mc_down(info: dict) -> bool:
    """On the ground: lie-down finished, or the zero-torque PASSIVE skillpilot ends it in."""
    return _mc_settled(info, "PASSIVE") or any(_mc_settled(info, name) for name in _MC_LYING)


def _mc_name(value) -> str:
    return _strip_prefix(str(value or ""), _MC_ACTION_PREFIX).upper()


def _mc_info(rpc, timeout=None) -> dict:
    """GetAction → {action, status, stage} with enum prefixes stripped."""
    info = (rpc.get_action(timeout) if timeout else rpc.get_action()).get("info") or {}
    stage = info.get("stage")
    return {"action": _mc_name(info.get("current_action")) or "UNKNOWN",
            "status": _strip_prefix(str(info.get("status") or ""), _MC_STATUS_PREFIX).upper() or "UNKNOWN",
            "stage": str(stage.get("description") or "") if isinstance(stage, dict) else ""}


def _mc_settled(info: dict, target: str) -> bool:
    if info["action"] != target or info["status"] == "PREPARE":
        return False
    if target in _MC_STEADY:
        return True
    # Exact stage match: a stale "LIE_DOWN_FINISHED" must not count for GET_UP.
    return info["status"] == "DONE" or info["stage"] == f"ActionStage::{target}_FINISHED"


def _mc_failed(info: dict) -> bool:
    """Current action's own stage reports failure (e.g. GET_UP_INITIAL_RESET_FAILED)."""
    stage = info["stage"]
    return stage.startswith(f"ActionStage::{info['action']}_") and stage.endswith("_FAILED")


def _mc_next(rpc) -> dict:
    """GetNextActions → {short action name: command entry to send verbatim}."""
    out = {}
    for item in rpc.get_next_actions().get("commands") or ():
        if isinstance(item, dict) and item.get("action"):
            out[_mc_name(item["action"])] = item
    return out


def _rejected(code: str, reason: str, suggestion: str = "", **extra) -> dict:
    """Uniform actuator rejection: nothing was sent to the robot."""
    out = {"state": "rejected", "code": code, "reason": reason}
    if suggestion:
        out["suggestion"] = suggestion
    out.update(extra)
    return out


def _notify_async(action_id, status, result, tool_name):
    """ACP POST off the caller's thread (it may block up to 5 s)."""
    threading.Thread(target=_acp_notify, args=(action_id, status, result, tool_name),
                     daemon=True, name=f"a3_acp_{action_id}").start()


def _motion_gate(rpc, card: str):
    """Structured rejection for actuators that only act in MOTION, else None."""
    try:
        info = _mc_info(rpc)
    except Exception as exc:
        return _rejected("NOT_READY", f"无法读取运动控制状态: {exc}",
                         "检查 MDU 连接后重试", current="UNKNOWN")
    if info["action"] != "MOTION" or info["status"] == "PREPARE":
        return _rejected("NOT_READY",
                         f"{card} 需要 MOTION 状态，当前 {info['action']}/{info['status']}",
                         "先调用 mc_mode stand，等待其完成后重试", current=info["action"])
    return None


class _McAbort(Exception):
    def __init__(self, code, reason, info=None):
        super().__init__(reason)
        self.code, self.reason, self.info = code, reason, info


class McModePlugin:
    """mc_mode 卡片：整机姿态切换。

    - stand / lie_down 只走 MDU skillpilot 的 SkillPilotService/PosturalTransition，
      与遥控器、App 同一入口。skillpilot 自己记录姿态（Core Initial=躺、Idle=站），
      只在这条链路里更新；绕过它直接调 MC 会让记录失真，之后遥控器的躺下/接管
      都会被它以“状态异常”拒绝，所以本卡不直接对 MC 发姿态切换。skillpilot 再按
      厂商序列驱动 MC：起立 [PD_STAND →] GET_UP → MOTION，躺下 LIE_DOWN → PASSIVE。
    - 多秒级整机动作，走 ACP，按 GetAction 轮询确认到位后才回报 completed；
      进行中拒绝新的切换（中途打断起身/躺下会摔），不绑定 interrupt hook；
      切换期间 loco 拒绝 walk，切换前先让 loco 减速并确认零速帧已送达、推流已停，
      停不稳则不发切换。
    - damping 立即卸力，必须显式 confirm=true（固件急停语义，不经 skillpilot）。
    """

    STAND_TIMEOUT_S = 40.0
    LIE_DOWN_TIMEOUT_S = 40.0
    DAMPING_TIMEOUT_S = 5.0
    COMPLETION_TIMEOUT_S = 60.0
    POLL_S = 0.25
    STATE_FAILS = 8          # consecutive GetAction failures (~2 s) before giving up
    # skillpilot accepted but MC never left the start posture → its skill did not run.
    START_GRACE_S = 5.0
    # After MOTION, skillpilot's GetUp skill keeps polling until the robot stops
    # stepping (is_walking=false, ~4 s in its log) before switching Core to Idle;
    # other skillpilot requests in that window are rejected, so hold completion.
    STAND_SETTLE_S = 5.0
    RESOURCES = ["base", "leg", "arm_l", "arm_r", "hand_l", "hand_r", "waist", "head"]

    ACTIONS = {
        "get_state": ([], "查询当前姿态、切换状态，以及 stand/lie_down/damping 此刻是否可执行"),
        "stand": ([], "起立到可控站立态 MOTION（经 skillpilot，与遥控器同链路："
                      "[PD_STAND →] GET_UP → MOTION）；完成后 ACP 回报，已在 MOTION 时直接返回 done"),
        "lie_down": ([], "从站立态躺下（经 skillpilot：LIE_DOWN → PASSIVE 零力矩），完成后 ACP 回报；"
                         "已躺下时直接返回 done"),
        "damping": (["confirm"], "进入阻尼模式（关节立即卸力，机器人会倒下）；必须 confirm=true，"
                                 "仅用于人工确认的紧急场景"),
    }

    def __init__(self, nodes, loco=None):
        self.nodes = nodes
        self.loco = loco
        self._lock = threading.Lock()
        self._active = None      # (action_id, action) of the posture switch in flight
        self._closed = False

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "confirm": {"type": "boolean", "default": False,
                        "description": "damping 专用：必须为 true 才会执行"},
        })
        schema["x-resource"] = list(self.RESOURCES)
        schema["x-completion"] = {"actions": ["stand", "lie_down"],
                                  "timeout": self.COMPLETION_TIMEOUT_S}
        return tool("mc_mode", "actuator",
                    "整机姿态：stand 起立到可控站立态 MOTION（其他运动卡的前提），lie_down 躺下，"
                    "两者与遥控器同走 skillpilot；damping 紧急卸力（需 confirm）；get_state 查看当前姿态与可执行动作",
                    schema)

    def start(self):
        self._closed = False

    def stop(self):
        self._closed = True

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "busy" if self._active else "ready"}
        if action == "get_state":
            return self._get_state()
        if action not in ("stand", "lie_down", "damping"):
            raise ValueError(f"mc_mode: unknown action {action!r}")
        rpc = self.nodes.rpc
        with self._lock:
            if self._active is not None:
                return _rejected("BUSY", f"姿态切换 {self._active[1]} 进行中",
                                 "等待其完成后再试", action_id=self._active[0])
            try:
                info, nxt = _mc_info(rpc), _mc_next(rpc)
            except Exception as exc:
                return _rejected("NOT_READY", f"无法读取运动控制状态: {exc}", "检查 MDU 连接后重试")
            if info["status"] == "PREPARE":
                return _rejected("NOT_READY", f"运动控制正在切换（{info['action']}/PREPARE）",
                                 "稍后重试", current=info["action"])
            if action == "damping":
                return self._damping(args, info, nxt)
            plan = self._plan(action, info, nxt)
            if "call" not in plan:         # rejected / already done
                return plan
            action_id = f"mc_{action}_{uuid4().hex[:8]}"
            self._active = (action_id, action)
        # Block new walks first, then require the stream to be fully stopped (zero
        # frames delivered, publisher exited) before skillpilot moves the legs.
        if self.loco is not None and not self.loco.block(f"mc_mode {action}"):
            self._release(action_id)
            return _rejected("LOCO_NOT_STOPPED", "行走未能确认停稳（零速帧未送达或停止超时），未发送姿态切换",
                             "用 loco status 确认已停下后重试", current=info["action"])
        if plan["call"]:
            rejection = self._request_skillpilot(action, info)
            if rejection is not None:
                self._release(action_id)
                return rejection
        threading.Thread(target=self._run, args=(action_id, action, info["action"], plan),
                         daemon=True, name=f"a3_{action_id}").start()
        return {"state": "accepted", "action_id": action_id, "from": info["action"]}

    def _release(self, action_id):
        """End the posture switch ``action_id``: clear the BUSY slot and let loco walk again."""
        with self._lock:
            if self._active and self._active[0] == action_id:
                self._active = None
        if self.loco is not None:
            self.loco.unblock()

    # -- planning ---------------------------------------------------------------

    def _plan(self, action, info, nxt):
        """Return {"call", "targets", "allowed"} or a reply dict (rejected / already done).

        ``call`` False = skillpilot's skill is already running (started by the
        remote/App), only track it. ``allowed`` = MC actions its sequence passes.
        """
        current = info["action"]
        allowed = sorted(nxt)
        if _mc_failed(info):
            return _rejected("ROBOT_FAILED", f"固件报告 {current} 失败（{info['stage']}）",
                             "人工检查机器人姿态与周围环境后再操作", current=current, stage=info["stage"])
        if action == "stand":
            if _mc_settled(info, "MOTION"):
                return {"state": "done", "current": "MOTION", "changed": False}
            if current in _MC_LYING and not _mc_settled(info, current):
                return _rejected("NOT_READY", f"{current} 尚未完成（{info['stage'] or info['status']}）",
                                 "等待躺下完成后再 stand", current=current)
            if current not in ("GET_UP", "PD_STAND", "PASSIVE", "DAMPING") and current not in _MC_LYING:
                return _rejected("NOT_ALLOWED", f"当前 {current} 下不支持起立（可切换: {allowed}）",
                                 "确认机器人处于躺下/零力矩/阻尼姿态；AVATAR 等模式请先由厂商 App 退出",
                                 current=current, next_actions=allowed)
            return {"call": current != "GET_UP", "targets": ("MOTION",),
                    "allowed": {current, "PASSIVE", "PD_STAND", "GET_UP", "MOTION"}}
        # lie_down
        if _mc_down(info):
            return {"state": "done", "current": current, "changed": False}
        if current != "LIE_DOWN" and "LIE_DOWN" not in nxt:
            return _rejected("NOT_ALLOWED", f"当前 {current} 下固件不允许躺下（可切换: {allowed}）",
                             "先 mc_mode stand 到 MOTION 再躺下", current=current, next_actions=allowed)
        return {"call": current != "LIE_DOWN", "targets": ("PASSIVE",),
                "allowed": {current, "LIE_DOWN", "PASSIVE"}}

    def _request_skillpilot(self, action, info):
        """Send PosturalTransition; a rejection dict if skillpilot refused, else None."""
        kind = "GetUp" if action == "stand" else "LieDown"
        try:
            self.nodes.rpc.postural_transition(kind)
        except RpcError as exc:
            return _rejected("SKILLPILOT_REJECTED", str(exc),
                             "skillpilot 拒绝了姿态切换（如正在行走、被其他功能占用）；稍后重试，"
                             "或用遥控器/App 确认同样的操作是否可行", current=info["action"])
        except Exception as exc:
            return _rejected("SKILLPILOT_ERROR", f"调用 skillpilot PosturalTransition 失败: {exc}",
                             "检查 MDU skillpilot（:52893）是否在运行", current=info["action"])
        return None

    def _damping(self, args, info, nxt):
        if args.get("confirm") is not True:
            return _rejected("CONFIRM_REQUIRED", "damping 会让关节立即卸力、机器人倒下",
                             "确认安全后以 confirm=true 重新调用")
        if _mc_settled(info, "DAMPING"):
            return {"state": "done", "current": "DAMPING", "changed": False}
        command = nxt.get("DAMPING")
        if command is None:
            return _rejected("NOT_ALLOWED", f"当前 {info['action']} 下固件不允许 DAMPING",
                             current=info["action"], next_actions=sorted(nxt))
        if self.loco is not None:
            self.loco.halt("superseded_by_damping", wait=False)
        try:
            self.nodes.rpc.set_action(command)
        except RpcError as exc:
            return _rejected("ROBOT_REJECTED", str(exc), current=info["action"])
        deadline = time.monotonic() + self.DAMPING_TIMEOUT_S
        last = info
        while time.monotonic() < deadline:
            try:
                last = _mc_info(self.nodes.rpc)
            except Exception:
                pass
            if _mc_settled(last, "DAMPING"):
                return {"state": "done", "current": "DAMPING", "changed": True}
            time.sleep(self.POLL_S)
        return _rejected("TIMEOUT", "已发送 DAMPING，但未在时限内确认进入", current=last["action"])

    # -- worker -----------------------------------------------------------------

    def _run(self, action_id, action, start_action, plan):
        timeout = self.STAND_TIMEOUT_S if action == "stand" else self.LIE_DOWN_TIMEOUT_S
        deadline = time.monotonic() + timeout
        status, result = "error", {}
        try:
            info = self._wait(plan["targets"], plan["allowed"], deadline,
                              start=start_action if plan["call"] else None)
            if action == "stand":
                info = self._hold(info, self.STAND_SETTLE_S)
            status, result = "completed", {"current": info["action"]}
        except _McAbort as exc:
            status = "cancelled" if exc.code == "CANCELLED" else "error"
            result = {"code": exc.code, "reason": exc.reason,
                      "current": (exc.info or {}).get("action", "UNKNOWN")}
        except Exception as exc:
            result = {"code": "STATE_UNAVAILABLE", "reason": str(exc)}
        finally:
            self._release(action_id)
        _acp_notify(action_id, status, result, "mc_mode")

    def _hold(self, info, seconds):
        """Keep watching for ``seconds`` that the posture stays put.

        Only fresh readings count: the hold ends on a successful GetAction, and
        STATE_FAILS consecutive failures abort instead of reporting settled.
        """
        end = time.monotonic() + seconds
        fails = 0
        while time.monotonic() < end or fails:
            if self._closed:
                raise _McAbort("CANCELLED", "驱动停止，不再跟踪本次切换", info)
            time.sleep(self.POLL_S)
            try:
                latest = _mc_info(self.nodes.rpc)
                fails = 0
            except Exception as exc:
                fails += 1
                if fails >= self.STATE_FAILS:
                    raise _McAbort("STATE_UNAVAILABLE", f"确认站稳期间连续读取 GetAction 失败: {exc}", info)
                continue
            if latest["action"] != info["action"]:
                raise _McAbort("INTERRUPTED", f"到位后状态变为 {latest['action']}", latest)
        return info

    def _wait(self, targets, allowed, deadline, start=None):
        """Poll GetAction until a target settles; ``start`` arms the did-it-begin check."""
        last, fails = None, 0
        begun_by = time.monotonic() + self.START_GRACE_S if start else None
        while time.monotonic() < deadline:
            if self._closed:
                raise _McAbort("CANCELLED", "驱动停止，不再跟踪本次切换", last)
            try:
                info = _mc_info(self.nodes.rpc)
                fails = 0
            except Exception as exc:
                fails += 1
                if fails >= self.STATE_FAILS:
                    raise _McAbort("STATE_UNAVAILABLE", f"连续读取 GetAction 失败: {exc}", last)
                time.sleep(self.POLL_S)
                continue
            last = info
            if any(_mc_settled(info, target) for target in targets):
                return info
            if _mc_failed(info):
                raise _McAbort("ROBOT_FAILED", f"固件报告 {info['action']} 失败（{info['stage']}）", info)
            if info["stage"].startswith("ActionStage::FALL_PROTECT"):
                raise _McAbort("INTERRUPTED", f"切换过程中触发跌倒保护（{info['stage']}）", info)
            if info["action"] not in allowed:
                raise _McAbort("INTERRUPTED", f"切换过程中状态变为 {info['action']}", info)
            if begun_by is not None:
                if info["action"] != start:
                    begun_by = None
                elif time.monotonic() > begun_by:
                    raise _McAbort("NOT_STARTED", f"skillpilot 已受理，但 {self.START_GRACE_S:.0f} 秒内"
                                                  f"运动控制仍停在 {start}", info)
            time.sleep(self.POLL_S)
        raise _McAbort("TIMEOUT", f"等待 {'/'.join(targets)} 到位超时", last)

    def _get_state(self):
        try:
            info, nxt = _mc_info(self.nodes.rpc), _mc_next(self.nodes.rpc)
        except Exception as exc:
            return _rejected("NOT_READY", f"无法读取运动控制状态: {exc}", "检查 MDU 连接后重试")
        current = info["action"]
        switching = info["status"] == "PREPARE" or _mc_failed(info)
        down = _mc_down(info)
        stand_ok = (not switching and not _mc_settled(info, "MOTION")
                    and (down or current in ("PD_STAND", "DAMPING", "GET_UP")))
        return {"current": current, "status": info["status"], "stage": info["stage"],
                "ready_for_motion": _mc_settled(info, "MOTION"),
                "lying": down,
                "busy": self._active[1] if self._active else None,
                "can": {"stand": stand_ok,
                        "lie_down": not switching and not down and "LIE_DOWN" in nxt,
                        "damping": not switching and "DAMPING" in nxt},
                "next_actions": sorted(nxt)}


class LocoPlugin:
    """loco 卡片：双足行走速度控制。

    链路与厂商 T_LocomotionVelocity.py 一致：完整 MotionControlLocomotionVelocityChannel
    （header + data），control_source=SAFE，mode=NAVIGATION(20)，20 Hz 持续下发，
    停止时下发零速；ROS 侧经 AimRT ros2 后端的 RosMsgWrapper 话题送达 MDU。

    - 一次 walk = 一个 ACP 任务，duration 必填（≤60 s）。到时减速到零后回报 completed。
    - 新 walk 直接接管：旧任务 cancelled(superseded)，速度平滑过渡，不先停下。
    - stop / interrupt hook：减速到零后结束，旧任务 cancelled。
    - 速度按 ACCEL/DECEL 斜坡变化（厂商要求大幅变化分步过渡）。
    - 行走中 2 Hz 监视 GetAction，离开 MOTION 或超过 MONITOR_STALE_S 未读到状态即减速停下并报 error。
    - 任务在零速帧发出成功后才结算；零速帧发不出去报 PUBLISH_FAILED。
    - 固件速度限制读不到或不合法时拒绝 walk（不退回卡片默认限制）。
    - mc_mode 姿态切换期间（block）拒绝 walk。
    """

    RATE_HZ = 20.0
    STOP_FRAMES = 5
    MAX_DURATION_S = 60.0
    COMPLETION_TIMEOUT_S = 75.0
    LOCO_MODE = "MotionControl_LocomotionMode_NAVIGATION"   # vendor sample sends 20
    # Card limits; intersected with the firmware's GetVelocityLimitValue.
    CARD_LIMITS = {"forward": (-0.3, 0.6), "lateral": (-0.3, 0.3), "angular": (-45.0, 45.0)}
    ACCEL = {"forward": 0.5, "lateral": 0.5, "angular": 90.0}    # per second, speeding up
    DECEL = {"forward": 1.0, "lateral": 1.0, "angular": 180.0}   # per second, slowing down
    MONITOR_S = 0.5
    MONITOR_RPC_TIMEOUT_S = 0.4   # per GetAction; the client default (5 s) would mask an outage
    MONITOR_STALE_S = 1.5         # wall-clock since the last good reading, checked every frame
    BLOCK_STOP_TIMEOUT_S = 3.0
    AXES = ("forward", "lateral", "angular")

    def __init__(self, nodes):
        self.nodes = nodes
        self._lock = threading.Lock()
        self._task = None          # {"id", "target", "deadline", "ending": None | (status, result)}
        self._velocity = dict.fromkeys(self.AXES, 0.0)
        self._thread = None
        self._fault = None         # set by the MOTION monitor
        self._state_at = 0.0       # monotonic time of the last good GetAction
        self._publish_error = None # why the current/last stream died, None if it stopped cleanly
        self._blocked = None       # reason walks are refused (mc_mode transition)
        self._seq = 0
        self._limits = None

    def get_tool(self):
        lim = self.CARD_LIMITS
        schema = action_schema(
            {"walk": (["forward", "lateral", "angular", "duration"],
                      "按给定速度行走 duration 秒，到时自动减速停下并 ACP 回报；新的 walk 会直接接管当前行走"),
             "stop": ([], "减速停下并结束当前行走"),
             "status": ([], "查询当前行走任务、目标速度与实际下发速度")},
            {
                "forward": {"type": "number", "default": 0.0, "minimum": lim["forward"][0],
                            "maximum": lim["forward"][1], "description": "前进速度 m/s，负为后退"},
                "lateral": {"type": "number", "default": 0.0, "minimum": lim["lateral"][0],
                            "maximum": lim["lateral"][1], "description": "横移速度 m/s，正为左移"},
                "angular": {"type": "number", "default": 0.0, "minimum": lim["angular"][0],
                            "maximum": lim["angular"][1], "description": "转向速度 deg/s，正为逆时针（左转）"},
                "duration": {"type": "number", "exclusiveMinimum": 0, "maximum": self.MAX_DURATION_S,
                             "description": "行走时长（秒），必填"},
            },
        )
        schema["x-resource"] = ["base", "leg"]
        schema["x-completion"] = {"actions": ["walk"], "timeout": self.COMPLETION_TIMEOUT_S}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"},
                             "on_interrupt_all": {"action": "stop"}}
        return tool("loco", "actuator",
                    "双足行走：forward/lateral 为 m/s，angular 为 deg/s，走 duration 秒后自动停下；"
                    "需先 mc_mode stand 进入 MOTION", schema)

    def start(self):
        pass

    def stop(self):
        self.halt("plugin_stopped", wait=True)

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"state": "walking" if self._task else "idle"}
        if action == "stop":
            stopped = self.halt("cancelled_by_request", wait=False)
            return {"state": "idle", "stopping": stopped}
        if action == "status":
            return self._status()
        if action != "walk":
            raise ValueError(f"loco: unknown action {action!r}")
        return self._walk(args)

    # -- public helpers for other cards ----------------------------------------

    def halt(self, reason, wait=False):
        """Ramp to zero and end the current walk (cancelled). Returns its action_id."""
        with self._lock:
            task = self._task
            if task is not None and task["ending"] is None:
                task["target"] = dict.fromkeys(self.AXES, 0.0)
                task["ending"] = ("cancelled", {"reason": reason})
            thread = self._thread
        if wait and thread is not None:
            thread.join(timeout=3.0)
        return task["id"] if task else None

    def block(self, reason) -> bool:
        """Refuse new walks (BUSY) until unblock(), and stop the current stream.

        True once the stream that was running has exited after delivering its
        zero frames (or none was running); False if it is still running after
        BLOCK_STOP_TIMEOUT_S or died on a publish error. The block stays either way.
        """
        with self._lock:
            self._blocked = reason
            running = self._thread is not None
        if not running:
            return True
        self.halt(reason)
        deadline = time.monotonic() + self.BLOCK_STOP_TIMEOUT_S
        while True:
            with self._lock:
                if self._thread is None:
                    return self._publish_error is None
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def unblock(self):
        with self._lock:
            self._blocked = None

    # -- walk -------------------------------------------------------------------

    def _busy(self):
        return _rejected("BUSY", f"{self._blocked} 进行中，暂不接受行走", "等待姿态切换完成后重试")

    def _walk(self, args):
        try:
            target = {axis: float(args.get(axis, 0.0) or 0.0) for axis in self.AXES}
            duration = float(args["duration"])
        except KeyError:
            return _rejected("INVALID_ARG", "duration 必填", f"取值 (0, {self.MAX_DURATION_S:g}] 秒")
        except (TypeError, ValueError):
            return _rejected("INVALID_ARG", "forward/lateral/angular/duration 必须是数字")
        if not 0.0 < duration <= self.MAX_DURATION_S:
            return _rejected("INVALID_ARG", f"duration={duration:g} 超出 (0, {self.MAX_DURATION_S:g}]")
        if self._blocked is not None:
            return self._busy()
        try:
            limits = self._effective_limits()
        except Exception as exc:
            return _rejected("NOT_READY", f"无法获取固件速度限制: {exc}",
                             "检查 MDU 连接与 MotionControlMotionService/GetVelocityLimitValue 后重试")
        for axis in self.AXES:
            low, high = limits[axis]
            if not low <= target[axis] <= high:
                return _rejected("INVALID_ARG", f"{axis}={target[axis]:g} 超出 [{low:g}, {high:g}]",
                                 limits={k: list(v) for k, v in limits.items()})
        if not any(target.values()):
            return _rejected("INVALID_ARG", "三个速度都为 0", "停止请用 stop")
        rejected = _motion_gate(self.nodes.rpc, "loco")
        if rejected:
            return rejected
        action_id = f"loco_walk_{uuid4().hex[:8]}"
        with self._lock:
            # Re-checked under the lock block() takes: a walk either lands before
            # the block (and is halted by it) or is refused here.
            if self._blocked is not None:
                return self._busy()
            old = self._task
            self._task = {"id": action_id, "target": target, "duration": duration,
                          "deadline": time.monotonic() + duration, "ending": None}
            self._fault = None
            if self._thread is None:
                self._state_at = time.monotonic()    # the MOTION gate just read it
                self._publish_error = None
                self._thread = threading.Thread(target=self._stream, daemon=True,
                                                name="a3_loco_stream")
                self._thread.start()
        if old is not None:
            status, result = old["ending"] or ("cancelled", {"reason": "superseded", "by": action_id})
            _notify_async(old["id"], status, result, "loco")
        return {"state": "accepted", "action_id": action_id, "target": target,
                "duration": duration, "superseded": old["id"] if old else None}

    def _effective_limits(self):
        """Card limits ∩ firmware GetVelocityLimitValue (angular converted rad/s → deg/s).

        Fails closed: raises if the firmware limits are unreadable or malformed
        (non-numeric, min > max, a range excluding 0, or empty); only a valid
        reading is cached. A missing field is 0 (proto3 JSON omits zero values).
        """
        if self._limits is not None:
            return self._limits
        v = self.nodes.rpc.get_velocity_limit().get("vel_limit_value")
        if not isinstance(v, dict) or not v:
            raise ValueError(f"响应缺少 vel_limit_value: {v!r}")
        limits = {}
        for axis, scale in (("forward", None), ("lateral", None), ("angular", math.degrees)):
            pair = []
            for end in ("min", "max"):
                value = v.get(f"{axis}_velocity_limit_{end}", 0.0)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"{axis}_velocity_limit_{end}={value!r} 不是有效数字")
                pair.append(scale(value) if scale else float(value))
            low, high = pair
            if not low <= 0.0 <= high:
                raise ValueError(f"{axis} 固件限制 [{low:g}, {high:g}] 不合法（需 min ≤ 0 ≤ max）")
            card = self.CARD_LIMITS[axis]
            limits[axis] = (max(card[0], low), min(card[1], high))
        self._limits = limits
        return limits

    # -- streaming --------------------------------------------------------------

    def _frame(self, velocity):
        now = time.time()
        self._seq += 1
        return {
            "header": {"seq": self._seq,
                       "timestamp": {"seconds": int(now), "nanos": int((now % 1.0) * 1e9),
                                     "ms_since_epoch": int(now * 1000)},
                       "frame_id": "phanthymotus_a3",
                       "control_source": "ControlSource_SAFE",
                       "uuid": ""},
            "data": {"mode": self.LOCO_MODE,
                     "forward_velocity": velocity["forward"],
                     "lateral_velocity": velocity["lateral"],
                     "angular_velocity": math.radians(velocity["angular"])},
        }

    def _ramp(self, target, dt):
        for axis in self.AXES:
            cur, goal = self._velocity[axis], target[axis]
            speeding_up = abs(goal) > abs(cur) and goal * cur >= 0
            step = (self.ACCEL if speeding_up else self.DECEL)[axis] * dt
            self._velocity[axis] = goal if abs(goal - cur) <= step else cur + math.copysign(step, goal - cur)

    def _stream(self):
        """20 Hz publisher: ramps toward the active task's target, settles a task
        only after its zero frame was published, and exits after STOP_FRAMES
        zero frames with no task left."""
        dt = 1.0 / self.RATE_HZ
        monitor_stop = threading.Event()
        threading.Thread(target=self._monitor, args=(monitor_stop,), daemon=True,
                         name="a3_loco_monitor").start()
        zero_frames = 0
        next_tick = time.monotonic()
        try:
            while True:
                with self._lock:
                    task = self._task
                    if task is not None and task["ending"] is None:
                        fault = self._fault
                        stale = time.monotonic() - self._state_at
                        if fault is None and stale >= self.MONITOR_STALE_S:
                            fault = {"code": "STATE_UNAVAILABLE",
                                     "reason": f"{stale:.1f} 秒未读到 GetAction"}
                        if fault is not None:
                            task["ending"] = ("error", dict(fault))
                            task["target"] = dict.fromkeys(self.AXES, 0.0)
                        elif time.monotonic() >= task["deadline"]:
                            task["ending"] = ("completed", {"duration": task["duration"],
                                                            "target": dict(task["target"])})
                            task["target"] = dict.fromkeys(self.AXES, 0.0)
                    target = task["target"] if task is not None else dict.fromkeys(self.AXES, 0.0)
                    self._ramp(target, dt)
                    velocity = dict(self._velocity)
                    stopped = not any(velocity.values())
                    if task is None and stopped and zero_frames >= self.STOP_FRAMES:
                        self._thread = None
                        break
                try:
                    self.nodes.publish_wrapper("locomotion_pub", self._frame(velocity))
                except Exception as exc:
                    self._abort_publish(exc)
                    return
                settle = None
                with self._lock:
                    # A walk that superseded ``task`` meanwhile has already reported it.
                    if stopped and task is not None and task["ending"] is not None and self._task is task:
                        settle, self._task = task, None
                    zero_frames = zero_frames + 1 if stopped and self._task is None else 0
                if settle is not None:
                    _notify_async(settle["id"], *settle["ending"], "loco")
                next_tick += dt
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            monitor_stop.set()

    def _abort_publish(self, exc):
        """Publishing broke: nothing more can reach the robot, so end the task as error."""
        with self._lock:
            task, self._task, self._thread = self._task, None, None
            self._velocity = dict.fromkeys(self.AXES, 0.0)
            self._publish_error = str(exc)
        print(f"[loco] publish failed: {exc}", flush=True)
        if task is not None:
            _notify_async(task["id"], "error", {"code": "PUBLISH_FAILED", "reason": str(exc)}, "loco")

    def _monitor(self, stop_event):
        """Refresh ``_state_at`` on every good GetAction; the stream judges staleness."""
        while not stop_event.wait(self.MONITOR_S):
            try:
                info = _mc_info(self.nodes.rpc, timeout=self.MONITOR_RPC_TIMEOUT_S)
            except Exception:
                continue
            with self._lock:
                self._state_at = time.monotonic()
            if info["action"] != "MOTION":
                self._fault = {"code": "LEFT_MOTION",
                               "reason": f"行走中运动控制状态变为 {info['action']}/{info['status']}"}

    def _status(self):
        with self._lock:
            task = self._task
            velocity = dict(self._velocity)
        if task is None:
            return {"state": "stopping" if any(velocity.values()) else "idle", "velocity": velocity}
        return {"state": "stopping" if task["ending"] else "walking", "action_id": task["id"],
                "target": dict(task["target"]), "velocity": velocity,
                "remaining_s": max(0.0, round(task["deadline"] - time.monotonic(), 2))}


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

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        named_fields = {}
        for side, names in ARM_JOINTS.items():
            for name in names:
                suffix = name[len(side) + 1:-len("_joint")]
                low, high = _joint_limit(name, ARM_JOINT_LIMITS)
                named_fields[name] = {
                    "type": "number", "minimum": low, "maximum": high,
                    "description": f"{side} {suffix} 目标角度（rad）",
                }
        schema = action_schema(
            {
                "set_position": (list(named_fields), "按具名关节设置手臂目标位置（rad）"),
            },
            {
                "left": {"type": "array", "items": {"type": "number"},
                         "description": "左臂 7 关节 rad，顺序：shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw"},
                "right": {"type": "array", "items": {"type": "number"},
                          "description": "右臂 7 关节 rad，顺序同左臂"},
                **named_fields,
                "duration_ms": {"type": "integer", "description": "保持时长（毫秒），期间以 100Hz 重复下发同一帧指令", "default": 100},
            },
        )
        # 双臂关节 —— side 按调用变化而 schema 是静态的，只声明一侧会让双臂动作
        # 与单臂动作并发抢同一批关节（tianyi arm 同款结论）。
        schema["x-resource"] = ["arm_l", "arm_r"]
        return tool("arm_control", "actuator", "下发双臂 14 关节位置指令 + 手臂柔顺控制"
                                              "（话题 /motion/control/arm_joint_command 需 100Hz 连续下发、间隔 ≤30ms；"
                                              "velocity/effort 固定 0；柔顺 Enable/Disable/CheckArmCompliance RPC；"
                                              "先经 mc_mode stand 站立并停止 motion_player）",
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
        # Backward-compatible aliases for callers that used the former
        # arm-compliance card. They are intentionally omitted from the action
        # schema: compliance is a setup/safety operation, not a joint move.
        if action in ("compliance_enable", "compliance_disable", "compliance_check"):
            return jsonable(self.nodes.rpc.arm_compliance(action.replace("compliance_", "")))
        if action not in ("set_position", "send"):
            raise ValueError(f"arm_control: unknown action {action!r}")
        rejected = _motion_gate(self.nodes.rpc, "arm_control")
        if rejected:
            return rejected
        positions = {}
        if action == "set_position":
            positions.update({name: float(args[name]) for names in ARM_JOINTS.values()
                              for name in names if args.get(name) is not None})
        for side in ("left", "right"):
            values = args.get(side) if action == "send" else None
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
            {
                "open": (["side"], "指定手完全张开；side 可选 left/right/both"),
                "close": (["side"], "指定手完全握紧；side 可选 left/right/both"),
                "set_position": (["left", "right"], "设置指定手指张合等级；0 张开，2000 握紧"),
            },
            {
                "side": {"type": "string", "enum": ["left", "right", "both"],
                         "description": "选择左手、右手或双手"},
                "duration_ms": {"type": "integer", "minimum": 100, "maximum": 5000,
                                 "default": 500, "description": "持续下发时长；设备要求关节指令连续发布"},
                "left": {"type": "array", "minItems": 1, "items": {"type": "number", "minimum": 0, "maximum": HAND_O10_MAX}, "description": "左手各指张合等级；O10Hand 为 0~4096，AgiHand 为 0~2000"},
                "right": {"type": "array", "minItems": 1, "items": {"type": "number", "minimum": 0, "maximum": HAND_O10_MAX}, "description": "右手各指张合等级；O10Hand 为 0~4096，AgiHand 为 0~2000"},
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
        if action not in ("open", "close", "set_position", "send"):
            raise ValueError(f"hand_control: unknown action {action!r}")
        hand_type = args.get("hand_type", "AgiHand")
        _require(hand_type in HAND_TYPES, f"未知手部类型 {hand_type!r}，可选 {sorted(HAND_TYPES)}")
        hand_type, live_left, live_right = self.nodes.hand_command_layout(hand_type)
        max_value = HAND_O10_MAX if hand_type == "O10Hand" else HAND_COMMAND_MAX
        if action in ("open", "close"):
            side = args.get("side")
            _require(side in ("left", "right", "both"), "open/close 必须指定 side=left/right/both")
            closed = max_value
            count = max(len(live_left), len(live_right), 6)
            values = [0] * count if action == "open" else [closed] * count
            args = {**args, "left": values if side in ("left", "both") else None,
                    "right": values if side in ("right", "both") else None}
            action = "set_position"
        positions = {}
        for side in ("left", "right"):
            values = args.get(side)
            if values is None:
                continue
            for i, value in enumerate(values):
                live_names = live_left if side == "left" else live_right
                # Prefer the names reported by the active O10Hand/AgiHand.
                # Some firmware accepts only these channel labels; indexed
                # names remain the documented fallback when no feedback has
                # arrived yet.
                name = live_names[i] if i < len(live_names) else f"{side}_hand_joint_{i}"
                positions[name] = _clamp(float(value), 0.0, max_value, f"{side} 手指 {i}")
        _require(positions, "至少提供 left 或 right 张合等级")
        rejected = _motion_gate(self.nodes.rpc, "hand_control")
        if rejected:
            return rejected
        duration_ms = int(args.get("duration_ms", 500))
        _require(100 <= duration_ms <= 5000, "duration_ms 必须在 100~5000 毫秒之间")
        self.nodes.publish_joint_command("hand_command_pub", positions,
                                         duration_ms=duration_ms, frame_id=hand_type)
        return {"hand_type": hand_type, "positions": positions,
                "duration_ms": duration_ms, "state": "published"}


class HeadControlPlugin:
    """head_control 卡片：头部双关节控制（命名对齐 tianyi2.0/q5_bundle）。

    对应 /motion/control/neck_joint_command 话题（sensor_msgs/JointState）。
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        position_fields = {
            "yaw": {"type": "number", "minimum": NECK_LIMITS["head_yaw_joint"][0],
                    "maximum": NECK_LIMITS["head_yaw_joint"][1], "description": "头部偏航目标角（rad）"},
            "pitch": {"type": "number", "minimum": NECK_LIMITS["head_pitch_joint"][0],
                      "maximum": NECK_LIMITS["head_pitch_joint"][1], "description": "头部俯仰目标角（rad）"},
        }
        schema = action_schema(
            {
                "look_left": ([], "向左转头"),
                "look_right": ([], "向右转头"),
                "look_up": ([], "抬头"),
                "look_down": ([], "低头"),
                "center": ([], "头部回中"),
                "set_position": (["yaw", "pitch"], "设置头部偏航/俯仰目标角度（rad）"),
            },
            {
                **position_fields,
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
        if action not in ("look_left", "look_right", "look_up", "look_down", "center", "set_position", "send"):
            raise ValueError(f"head_control: unknown action {action!r}")
        presets = {
            "look_left": {"yaw": 0.6}, "look_right": {"yaw": -0.6},
            "look_up": {"pitch": 0.15}, "look_down": {"pitch": -0.3},
            "center": {"yaw": 0.0, "pitch": 0.0},
        }
        if action in presets:
            args = {**presets[action], **args}
        positions = {}
        if args.get("yaw") is not None:
            positions["head_yaw_joint"] = float(args["yaw"])
        if args.get("pitch") is not None:
            positions["head_pitch_joint"] = float(args["pitch"])
        _require(positions, "至少提供 yaw 或 pitch")
        rejected = _motion_gate(self.nodes.rpc, "head_control")
        if rejected:
            return rejected
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
            {"set_position": (["pitch", "yaw", "height"], "设置腰部俯仰/旋转/升降目标"),
             "send": (["pitch", "yaw", "height"], "兼容旧接口：设置腰部目标位置")},
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
        if action not in ("set_position", "send"):
            raise ValueError(f"waist_control: unknown action {action!r}")
        payload = {}
        for field in ("pitch", "yaw", "height"):
            value = args.get(field)
            if value is None:
                continue
            low, high = WAIST_LIMITS[f"waist_{field}"]
            payload[f"waist_{field}"] = _clamp(float(value), low, high, field)
        _require(payload, "至少提供 pitch/yaw/height 之一")
        rejected = _motion_gate(self.nodes.rpc, "waist_control")
        if rejected:
            return rejected
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

        同时清除 pause 标记：pause 只属于被结算的那个动作，若残留会让下一次
        播放的 worker 在 _paused.wait() 上永久冻结（10th PR review）。
        """
        self._paused.clear()
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
            if self._paused.is_set():
                # Event.wait() 在 set 状态下立即返回，直接用它做暂停等待会
                # 以满速空转打满一个核（最长 600 s，12th PR review）——
                # 暂停期间改用同粒度的 sleep 轮询，恢复/停止依然在一拍内被看到。
                time.sleep(self._PLAY_WORKER_TICK_S)
            else:
                self._paused.wait(self._PLAY_WORKER_TICK_S)
            if self._play_action_id != action_id:
                return  # superseded by a new play, or settled by stop/reset
            if self._paused.is_set():
                continue  # frozen this tick — do not consume the countdown
            remaining_s -= self._PLAY_WORKER_TICK_S
        if self._finish(action_id, "completed",
                        {"motion_id": motion_path, "duration_ms": duration_ms}):
            return
        # _finish 返回 False 表示已被取消/顶替接管，终态由对方回报。

    def _finish(self, action_id, status, result):
        """原子终态：锁内确认仍持有该 action_id 才清 id，锁外回报。

        与 TtsPlugin._finish 同款（10th PR review：worker 终态不做原子认领时，
        并发 stop/reset/顶替可在 check 与 POST 之间清 id 并回报 cancelled，
        之后 worker 再补一发同一 action 的第二终态事件）。返回是否已认领。
        """
        with self._play_lock:
            if self._play_action_id != action_id:
                return False  # 已被取消/顶替，终态由对方回报
            self._play_action_id = None
        _acp_notify(action_id, status, result, "motion_play")
        return True

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
            # command joints that are unpowered). Unknown state fails closed.
            rejected = _motion_gate(self.nodes.rpc, "motion_play")
            if rejected:
                return rejected
            # Atomic settle-then-arm: a second concurrent play must cancel the
            # first (cmd_end + immediate ACP cancelled) instead of silently
            # orphaning its barrier — the superseded worker would otherwise
            # exit on the id mismatch and leave Agent Core pending 600 s
            # (ThreadingHTTPServer makes concurrent play reachable). The prior
            # motion is ended BEFORE the new SendMotionCommand fires so the
            # robot never receives two overlapping plays. The whole transition
            # (settle → physical command → arm) holds the lock: two concurrent
            # plays can never both observe "no active action" and fire two
            # overlapping robot commands (10th PR review).
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
                action_id = f"motion_play_{uuid4().hex[:8]}"
                self._play_action_id = action_id
                # 每次武装新播放都清 pause 标记：_paused 只属于上一个动作，
                # 残留会让本动作的 worker 在 _paused.wait() 上永久冻结
                # （11th PR review）。
                self._paused.clear()
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
            # No active play → the flag must NOT be set (11th PR review): a
            # stale _paused with no armed worker would freeze the NEXT play's
            # countdown forever (pause while idle, then play → barrier hangs
            # to the 600 s timeout).
            with self._play_lock:
                if self._play_action_id is None:
                    return {"state": "idle"}
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
        # 调用方须持有 _play_lock（speak/play_media 在锁内完成结算→物理打断
        # →武装的整段转移，10th PR review）：并发播报不会重叠打断，也不会
        # 互相顶掉对方的 id。
        action_id = f"tts_{action}_{uuid4().hex[:8]}"
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
            with self._play_lock:
                prior = self._settle_active("cancelled", {"reason": "replaced_by_new_playback"})
                if prior is not None and prior[1]:
                    try:
                        self.nodes.rpc.stop_tts_trace_id(prior[1])
                    except Exception:
                        pass
                response = self.nodes.rpc.play_tts(
                    text,
                    priority_level=priority if priority in TTS_PRIORITY_LEVELS.values() else TTS_PRIORITY_LEVELS[priority],
                    is_interrupted=bool(args.get("is_interrupted", True)),
                    trace_id=str(args.get("trace_id") or ""),
                )
                trace_id = response.get("trace_id", "") or (response.get("data") or {}).get("trace_id", "")
                return self._arm_completion("speak", trace_id, {"text": text[:50]})
        if action == "play_media":
            file_name = args.get("file_name", "")
            _require(file_name, "file_name 不能为空")
            with self._play_lock:
                prior = self._settle_active("cancelled", {"reason": "replaced_by_new_playback"})
                if prior is not None and prior[1]:
                    try:
                        self.nodes.rpc.stop_tts_trace_id(prior[1])
                    except Exception:
                        pass
                response = self.nodes.rpc.play_media_file(file_name, is_interrupted=True)
                trace_id = response.get("trace_id", "") or (response.get("data") or {}).get("trace_id", "")
                return self._arm_completion("play_media", trace_id,
                                            {"file_name": file_name, "response": response})
        if action == "status":
            return jsonable(self.nodes.rpc.get_audio_status(args.get("trace_id", "")))
        if action == "stop_trace_id":
            # 打断播报同时结算挂起的 ACP 等待线程（立刻回报 cancelled，
            # 不让 Agent Core 的 barrier 挂到超时）。
            requested_trace = str(args.get("trace_id") or "")
            with self._play_lock:
                active_trace = self._play_trace_id or ""
                # An empty trace means "current" in the A3 API.  A non-empty
                # unrelated trace must never cancel the active ACP waiter.
                if not requested_trace or requested_trace == active_trace:
                    self._settle_active("cancelled", {"reason": "cancelled_by_request",
                                                      "trace_id": requested_trace or active_trace})
            return jsonable(self.nodes.rpc.stop_tts_trace_id(requested_trace))
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


class MicPlugin:
    def __init__(self, nodes, key):
        self.nodes = nodes
        self.key = key

    def get_tool(self):
        stream = self.nodes.audio_topics.get(self.key)
        if stream is None:
            # Keep the card discoverable in the canvas even on images that do
            # not include audio_msgs.  The topic is intentionally declared so
            # the UI can show the endpoint, while info/query report the actual
            # unavailable state until a compatible AudioCapture package is
            # installed in the container.
            topic = _core_topic(self.nodes.namespace, f"{self.key}/audio")
            return tool(self.key, "sensor", "A3 音频输入不可用（容器缺少 audio_msgs.msg.AudioCapture）",
                        {"type": "object", "properties": {}},
                        topic_out=[{"topic": topic, "format": "audio/pcm-16k"}])
        return tool(self.key, "sensor", "A3 音频输入流（AudioCapture，16 kHz PCM）",
                    {"type": "object", "properties": {}},
                    topic_out=[{"topic": stream["topic"], "format": "audio/pcm-16k"}])

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        stream = self.nodes.audio_topics.get(self.key, {})
        if not stream:
            if action in ("info", "query", "start"):
                return {"state": "unavailable", "reason": "audio_msgs.msg.AudioCapture is not installed",
                        "topic_out": [{"topic": _core_topic(self.nodes.namespace, f"{self.key}/audio"),
                                       "format": "audio/pcm-16k"}]}
            if action == "stop":
                return {"state": "idle"}
        if action == "stop":
            return {"state": "idle", **stream}
        return {"state": "running", **stream}


class SpeakerPlugin:
    def __init__(self, nodes):
        self.nodes = nodes
        self.topic = ""

    def get_tool(self):
        return tool("speaker", "actuator", "A3 扬声器（订阅 audio/pcm-16k 并输出到 AudioPlayback）",
                    {"type": "object", "properties": {
                        "input_topic": {"type": "string", "description": "输入音频 topic"},
                    }}, topic_in=[{"format": "audio/pcm-16k"}])

    def start(self):
        pass

    def stop(self):
        self.nodes.speaker_stop()

    def dispatch(self, action, args):
        if action in ("start", "play"):
            self.topic = str(args.get("input_topic") or "")
            if not self.topic:
                return {"state": "ready", "message": "等待 input_topic"}
            self.nodes.speaker_start(self.topic)
            return {"state": "running", "topic": self.topic}
        if action in ("stop", "interrupt"):
            self.nodes.speaker_stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self.topic else "idle", "topic": self.topic}
        raise ValueError(f"speaker: unknown action {action!r}")


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
        if self.nodes.config.get("plugins", {}).get("interaction", {}).get("full_takeover", True):
            try:
                self.nodes.rpc.set_voice_enable(False)
            except Exception as exc:
                print(f"[warn] failed to disable built-in interaction: {exc}", flush=True)

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

    对应 /skill/pilot/face/play/pb_3Aaimdk_2Eprotocol_2EHFAEmoction 话题
    （RosMsgWrapper + HFAEmoction）+ HDU
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
        return tool("face_play", "actuator", "播放/取消表情 + 表情资源列表（AimDK HFAEmoction 话题 + "
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
            rejected = _motion_gate(self.nodes.rpc, "face_play")
            if rejected:
                return rejected
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
            rejected = _motion_gate(self.nodes.rpc, "skill_play")
            if rejected:
                return rejected
            # 新播放顶替旧播放：先结算旧等待线程（立即 cancelled）并在锁内按旧
            # session_id 物理停止 —— 顺序不能反，否则 Stop 打断的是新会话
            # （ThreadingHTTPServer 使并发 play 可达，MotionPlayPlugin 同款）。
            # 结算→Stop→Start→武装在同一把锁内完成（10th PR review）：并发
            # play 不会重叠下发，也不会互相顶掉对方的 id。
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
    前置条件（开发文档 §7.9）：MC 处于 MOTION 模式（mc_mode stand）且已完成
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
                    "config 门控模块，默认关闭；导航前置：mc_mode stand + start_relocalization 且 map_id 一致；"
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

        新导航顶替旧导航：先结算旧等待线程（立即 cancelled）并在锁内按旧
        task_id 物理取消 —— 不取消的话机器人会同时执行两个导航任务
        （ThreadingHTTPServer 使并发下发可达，MotionPlayPlugin 同款）。
        结算→ActionCancel→新导航→武装在同一把锁内完成（10th PR review）：
        并发导航不会重叠下发，也不会互相顶掉对方的 id。
        """
        with self._nav_lock:
            prior = self._settle_active("cancelled", {"reason": "replaced_by_new_navigation"})
            if prior is not None and prior[1]:
                try:
                    self.nodes.rpc.navi("ActionCancel", {"task_id": prior[1]})
                except Exception:
                    pass
            response = self._remember(self.nodes.rpc.navi(method, payload))
            task_id = (response or {}).get("task_id") or self.last_task_id or 0
            action_id = f"a3_nav_{uuid4().hex[:8]}"
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
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
            return self._nav_dispatch(action, "PlanningNaviToGoal", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "target_id": int(args.get("target_id", 0)), "guide_line_id": 0,
                "ackerman_mode": False})
        if action == "navi_to_pose":
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
            return self._nav_dispatch(action, "PlanningNaviToPose2D", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "pose": {"position": {"x": float(args.get("x", 0)), "y": float(args.get("y", 0))},
                         "angle": float(args.get("angle", 0))},
                "ackerman_mode": False})
        if action == "linear_to_goal":
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
            return self._nav_dispatch(action, "LinearNaviToGoal", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "target_id": int(args.get("target_id", 0))})
        if action == "linear_to_pose":
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
            return self._nav_dispatch(action, "LinearNaviToPose2D", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "pose": {"position": {"x": float(args.get("x", 0)), "y": float(args.get("y", 0))},
                         "angle": float(args.get("angle", 0))}})
        if action == "move_forward":
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
            return self._nav_dispatch(action, "MoveForward", {
                "task_id": 0, "map_id": int(args.get("map_id", 0)),
                "angle": 0, "distance": float(args.get("distance", 0))})
        if action == "spin_turn":
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
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
            rejected = _motion_gate(self.nodes.rpc, "controlled_spatial")
            if rejected:
                return rejected
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
            if action == "charge_start":
                rejected = _motion_gate(self.nodes.rpc, "auto_charging")
                if rejected:
                    return rejected
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
        self.topic = _core_topic(namespace, "spatial_map")
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
    if enabled("joint_state"):
        plugins["joint_state"] = JointStatePlugin(nodes)
    if enabled("imu"):
        plugins["imu"] = ImuPlugin(nodes)
    camera_cfg = plugins_cfg.get("camera", {})
    selected_cameras = set(camera_cfg.get("streams") or [])
    if (enabled("camera") or enabled("camera_head_left")) and "head_left_fisheye" in selected_cameras:
        plugins["camera_head_left"] = CameraStreamPlugin(nodes, "camera_head_left", "head_left_fisheye")
    if (enabled("camera") or enabled("camera_head_right")) and "head_right_fisheye" in selected_cameras:
        plugins["camera_head_right"] = CameraStreamPlugin(nodes, "camera_head_right", "head_right_fisheye")
    fixed_cameras = {
        "camera_head_rear": "head_rear_fisheye",
        "camera_head_stereo_left": "head_stereo_left_fisheye",
        "camera_head_stereo_right": "head_stereo_right_fisheye",
        "camera_armpit_right": "armpit_right_fisheye",
        "camera_waist_rgb": "waist_front_d415_rgb",
        "camera_waist_depth": "waist_front_d415_depth",
    }
    for card_name, stream_name in fixed_cameras.items():
        if (enabled("camera") or enabled(card_name)) and stream_name in selected_cameras:
            plugins[card_name] = CameraStreamPlugin(nodes, card_name, stream_name)
    if (enabled("camera") or enabled("camera_chest_rgb")) and "chest_front_d457_rgb" in selected_cameras:
        plugins["camera_chest_rgb"] = CameraStreamPlugin(nodes, "camera_chest_rgb", "chest_front_d457_rgb")
    if (enabled("camera") or enabled("camera_chest_depth")) and "chest_front_d457_depth" in selected_cameras:
        plugins["camera_chest_depth"] = CameraStreamPlugin(nodes, "camera_chest_depth", "chest_front_d457_depth")
    # Keep the legacy multiplexed card available only when explicitly enabled.
    if enabled("camera") and not any(enabled(name) for name in ("camera_head_left", "camera_chest_rgb", "camera_chest_depth")):
        plugins["camera"] = CameraPlugin(nodes)
    if enabled("lidar_cloud"):
        plugins["lidar_cloud"] = LidarCloudPlugin(nodes)
    if enabled("battery"):
        plugins["battery"] = BatteryPlugin(nodes)
    if enabled("estop"):
        plugins["estop"] = EstopPlugin(nodes)
    # loco/waist_control/face_play publish RosMsgWrapper commands and are withheld
    # (not just broken at dispatch) when ros2_plugin_proto is absent — degraded
    # startup must expose no half-working command tools (5th PR review).
    wrapper_cmds = nodes.wrapper_available
    if enabled("loco") and wrapper_cmds:
        plugins["loco"] = LocoPlugin(nodes)
    if enabled("mc_mode"):
        # mc_mode brings loco to a stop before any posture switch.
        plugins["mc_mode"] = McModePlugin(nodes, loco=plugins.get("loco"))
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
    if enabled("mic"):
        plugins["mic"] = MicPlugin(nodes, "mic")
    if enabled("ext_mic"):
        plugins["ext_mic"] = MicPlugin(nodes, "ext_mic")
    if enabled("speaker"):
        plugins["speaker"] = SpeakerPlugin(nodes)
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
