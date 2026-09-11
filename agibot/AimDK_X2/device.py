#!/usr/bin/env python3
"""AgiBot X2 (AimDK SDK) ROS 2/rclpy adaptation layer.

AimDK's Python surface is plain rclpy + the `aimdk_msgs` interface package (confirmed via the
SDK's `py_examples/*.py`) — there is no proprietary AimRT middleware to bind against here, so
this driver is structured like `deep_robotics/lynx_m20` (pure rclpy over `vendor_runtime`), not
like `unitree/g1`'s raw-DDS `unitree_sdk2py` pattern.

Service names below use the vendor SDK's literal `/aimdk_5Fmsgs/srv/...` strings, taken
verbatim from the SDK's own `topics_and_services` catalog and `py_examples/*.py` clients
(e.g. `get_map.py`, `set_mic_source.py`) — this "_5F_" (hex for "_") is how the vendor's own
tooling names these services on the wire, not a typo introduced here.
"""

from __future__ import annotations

import json
import os
import math
import struct
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, jsonable, tool


def core_publisher(node, msg_type, topic, qos):
    """Route Agent Core output through the isolated X2 socket bridge."""
    from x2_bridged_publisher import create_bridged_publisher

    return create_bridged_publisher(msg_type, topic)


HAND_TYPES = {0: "none", 1: "nimble_hands", 2: "claw", 3: "leisai_nimble_hands", 255: "error"}

# The SDK enum is a compile-time superset.  The verified X2 firmware explicitly
# rejects STAND_UP_DEFAULT and ZERO_TORQUE_DEFAULT. PASSIVE_DEFAULT and
# STAND_DEFAULT returned success but have not produced the documented physical
# behavior on this unit, so only the observed working mode is exposed by default.
# Never expose an enum value merely because it exists in aimdk_msgs.
MC_ACTIONS = {
    "damping_default": 3,
}

PRESET_MOTIONS = {
    "raise_hand": 1001, "wave_hand": 1002, "shake_hand": 1003, "flying_kiss_hand": 1004,
    "clap_hand": 1008, "clipfist": 1009, "salute": 1013, "turn_wave_hand": 2001,
    "interaction_bow": 3001, "interaction_like": 3002, "interaction_ye": 3003,
    "interaction_sweatheart": 3004, "interaction_sad": 3006, "interaction_lightwave": 3007,
    "interaction_hug": 3008, "interaction_handx": 3009, "interaction_chestwave": 3010,
    "interaction_cheer": 3011, "interaction_blowkiss": 3012, "interaction_bassdance1": 3013,
    "interaction_bassdance2": 3014, "hitclap": 3015, "interaction_speak": 3016,
    "interaction_photoposture": 3018, "interaction_phototrippleposture": 3019,
    "point_head": 4001, "shake_head": 4002,
}

# 单臂动作：vendor 自带的 preset_motion_client.py 示例里这几个动作必须显式传 area=
# left_hand/right_hand（1/2），area=none(0) 时机器人不知道该动哪只手臂，SetMcPresetMotion
# 会返回 response.header.code=1（失败）。其余全身/头部交互动作用 area=none 即可。
PRESET_MOTIONS_REQUIRE_ARM_AREA = {
    "raise_hand", "wave_hand", "shake_hand", "flying_kiss_hand",
    "clap_hand", "clipfist", "salute", "turn_wave_hand",
}

MC_CONTROL_AREAS = {"none": 0, "left_hand": 1, "right_hand": 2, "head": 4, "waist": 8}

JOINT_AREAS = ("leg", "waist", "arm", "head")

LED_MODES = {"constant": 0, "breath": 1, "flash": 2, "flow": 3}

TTS_PRIORITY_LEVELS = {
    "background": 0x01, "service": 0x02, "mission": 0x04,
    "interaction": 0x06, "system": 0x07, "warning": 0x08, "safety": 0x0A,
}

EMOJI_IDS = {
    "idle_blink": 1, "idle_calm_1": 10, "idle_calm_2": 11, "idle_game": 20,
    "idle_cute_1": 30, "idle_cute_2": 31, "idle_cute_3": 32, "idle_cute_4": 33,
    "eye_close": 40, "eye_open": 50, "eye_boring_1": 60, "eye_abnormal": 70,
    "eye_sleepy": 80, "eye_happy": 90, "eye_extremehappy_1": 100, "eye_extremehappy_2": 101,
    "eye_sad": 110, "eye_sympathy": 120, "eye_confuse": 130, "eye_shock": 140,
    "eye_actcute": 150, "eye_serious": 160, "eye_thinking": 170, "eye_angry": 180,
    "eye_extremeangry": 190, "eye_adore": 200, "eye_extremeadore": 210, "eye_charge": 220,
}

MIC_SOURCES = {"internal": 0, "external": 1}
RESOURCE_DIR = Path(__file__).with_name("resource")
SKELETON_TOPIC = "state/joints"
SKELETON_MAX_HZ = 30.0
CAMERA_RGB_MAX_HZ = 10.0
# The vendor's DirectVelocityControl example publishes at 50 Hz.  The source
# watchdog is 1000 ms, but matching the reference cadence also keeps control
# latency bounded under DDS packet loss.
LOCOMOTION_HEARTBEAT_HZ = 50.0
# X2 field evidence: command forward=0.2 m/s for 5s produced ~2.0 m leg_odometry
# travel while ACP completed on time. That points at effective tracking gain, not a
# late stop timer. Scale outgoing velocity using config and/or odom twist feedback.
LOCOMOTION_VEL_SCALE_MIN = 0.35
LOCOMOTION_VEL_SCALE_MAX = 1.25
LOCOMOTION_VEL_SCALE_EMA = 0.15
LOCOMOTION_VEL_SCALE_MIN_SPEED = 0.05
# Safety only: abort if odom travel far exceeds speed*duration expectation.
LOCOMOTION_ODOM_SAFETY_FACTOR = 1.35
LOCOMOTION_ODOM_SAFETY_MIN_M = 0.15
LOCOMOTION_ODOM_SAFETY_MIN_DEG = 15.0


def skeleton_layout(variant):
    root = ET.parse(RESOURCE_DIR / f"x2_{variant}.urdf").getroot()
    names = [
        joint.get("name") for joint in root.findall("joint")
        if joint.get("name") and joint.get("type") != "fixed"
    ]
    groups = {"leg": [], "waist": [], "arm": [], "head": []}
    for name in names:
        if name.startswith(("left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")):
            groups["leg"].append(name)
        elif name.startswith("waist_"):
            groups["waist"].append(name)
        elif name.startswith(("left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist")):
            groups["arm"].append(name)
        elif name.startswith("head_"):
            groups["head"].append(name)
    indices = {name: index for index, name in enumerate(names)}
    return {area: tuple(area_names) for area, area_names in groups.items()}, indices


def call_service(client, request, timeout=5.0):
    """Blocking service call against a node whose executor is already spinning
    in the background (vendor_runtime.DualDomainROS2). Safe to call from any
    dispatch() thread — each call gets its own future."""
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(f"service {client.srv_name} unavailable")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f"service {client.srv_name} timed out after {timeout}s")
        time.sleep(0.01)
    exc = future.exception()
    if exc is not None:
        raise exc
    return future.result()


def _with_actuation_contract(schema, resource, completion=None):
    """Annotate an actuator schema for Agent Core's ACP resource barrier."""
    schema = dict(schema)
    schema["x-resource"] = resource
    if completion is not None:
        schema["x-completion"] = completion
    return schema


def _acp_notify(action_id, status, result, tool):
    """Report a bounded, local ACP completion event without blocking ROS callbacks."""
    import os
    import ssl
    import urllib.request

    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }, ensure_ascii=False).encode("utf-8")
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    try:
        request = urllib.request.Request(
            f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678').rstrip('/')}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(request, timeout=5, context=context).close()
    except Exception as exc:
        print(f"[acp] {tool} completion callback failed: {str(exc)[:200]}", flush=True)


class AimdkNodes:
    def __init__(self, config, namespace, ros2):
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
        from std_msgs.msg import String, UInt8MultiArray
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from aimdk_msgs.msg import CommonRequest, PmuState, TouchState
        try:
            from aimdk_msgs.msg import McCommonState
        except ImportError:
            McCommonState = None
        from aimdk_msgs.srv import (
            ExecuteActionResource, GetAllJointState, GetCurrentInputSource, GetHandType,
            GetMcAction, GetMicSourceRequest, GetRobotResources, GetStoredMapByName,
            GetSystemState, PlayEmoji, PlayTts, SetMcAction, SetMcInputSource,
            SetMcPresetMotion, SetMicSourceRequest, SetPmuLed,
        )
        from aimdk_msgs.msg import (
            HandCommand, HandCommandArray, HandStateArray, JointCommand, JointCommandArray,
            JointStateArray,
            McLocomotionVelocity,
        )
        from audio_msgs.msg import AudioChunk
        from aimdk_msgs.msg import AudioCapture

        self._msg = {
            "CommonRequest": CommonRequest, "String": String, "Pose": Pose,
            "UInt8MultiArray": UInt8MultiArray,
        }
        self._HandCommand = HandCommand
        self._HandCommandArray = HandCommandArray
        self._JointCommand = JointCommand
        self._JointCommandArray = JointCommandArray
        self._McLocomotionVelocity = McLocomotionVelocity
        self._AudioChunk = AudioChunk

        self.config = config
        self.end_effector = str(config.get("end_effector", "hand")).lower()
        self.skeleton_joints, self.skeleton_joint_indices = skeleton_layout(self.end_effector)
        self.namespace = namespace
        self.robot = Node("agibot_x2_driver_robot", context=ros2.ctx_robot)
        self.core = Node("agibot_x2_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)

        self.lock = threading.RLock()
        self.values = {}
        self._last_stream_publish = {}
        self._last_skeleton_publish = 0.0
        self.joint_groups = {}
        self._mc_mode_state = {"action_desc": "", "action_status": None, "fsm_state": None}
        self.mc_state_available = McCommonState is not None

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.streams = {}

        # This is the vendor's live state-machine report.  It lets ACP distinguish
        # a SetMcAction request being accepted from the requested mode being active.
        if self.mc_state_available:
            self.robot.create_subscription(McCommonState, "/aima/mc/common/state", self._mc_common_state_callback, sensor_qos)
        else:
            print("[x2] McCommonState is unavailable in this AimDK image; MC mode completion is service-acknowledged", flush=True)

        def stream_enabled(name, default=True):
            return bool(config.get("plugins", {}).get(name, {}).get("enabled", default))

        def mirror(key, msg_type, robot_topic, fmt, depth=10, qos=None, max_hz=None, transform=None):
            core_topic = f"/{namespace}/agibot_x2/{key}"
            as_json = fmt == "data/json"
            core_msg_type = String if as_json else msg_type
            pub = core_publisher(self.core, core_msg_type, core_topic, depth)
            self.robot.create_subscription(
                msg_type, robot_topic,
                self._callback(key, pub, as_json=as_json, max_hz=max_hz, transform=transform), qos or depth,
            )
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

        # Two physical IMUs (chest/torso) feed a single combined "imu" tool/topic — driver.yaml
        # lists one imu card, so both raw readings are merged into one data/json stream rather
        # than exposed as two separate tools.
        imu_topic = f"/{namespace}/agibot_x2/imu"
        imu_pub = core_publisher(self.core, String, imu_topic, 5)
        self.robot.create_subscription(Imu, "/aima/hal/imu/chest/state", self._imu_callback("chest", imu_pub), sensor_qos)
        self.robot.create_subscription(Imu, "/aima/hal/imu/torso/state", self._imu_callback("torso", imu_pub), sensor_qos)
        self.streams["imu"] = {"robot_topic": "/aima/hal/imu/{chest,torso}/state", "topic": imu_topic, "format": "data/json"}

        # AimDK's internal microphone publishes AudioCapture. Convert its raw
        # PCM payload to the shared AudioChunk contract used by Agent Core/ASR.
        mic_topic = f"/{namespace}/agibot_x2/mic/audio"
        self.mic_audio_pub = core_publisher(self.core, AudioChunk, mic_topic, 20)
        self._mic_audio_buffer = bytearray()
        self._mic_audio_lock = threading.Lock()
        self._mic_capture_stats = {"chunks": 0, "bytes": 0, "sample_rate": None, "channels": None}
        self.robot.create_subscription(
            AudioCapture, "/aima/hal/audio/capture", self._audio_capture_callback, sensor_qos,
        )
        self.streams["mic"] = {
            "robot_topic": "/aima/hal/audio/capture",
            "topic": mic_topic,
            "format": "audio/pcm-16k",
            "ros_type": "audio_msgs/msg/AudioChunk",
            "source_ros_type": "aimdk_msgs/msg/AudioCapture",
        }

        # Locomotion odometry is available on X2 even when SLAM is disabled. Keep this
        # separate from slam_odom: it reports leg/body-integrated motion, not map pose.
        if stream_enabled("leg_odometry", default=True):
            mirror(
                "leg_odometry", Odometry, "/aima/mc/leg_odometry", "data/json",
                qos=sensor_qos, transform=self._leg_odometry_payload,
            )

        if stream_enabled("hand_state", default=False):
            mirror(
                "hand_state", HandStateArray, "/aima/hal/joint/hand/state", "data/json",
                qos=sensor_qos, transform=self._hand_state_payload,
            )
        mirror("head_touch", TouchState, "/aima/hal/sensor/touch_head", "data/json", qos=sensor_qos)
        mirror("pmu_state", PmuState, "/aima/hal/pmu/state", "data/json", qos=sensor_qos)
        # SDK's topics_and_services catalog documents rgbd_head_front/* as the front camera, but
        # on real hardware that topic has zero publishers -- this unit's camera service actually
        # publishes RGB under rgb_head_front_center/* instead (confirmed live via `ros2 topic
        # info`, 30Hz). No depth topic is published anywhere on this hardware at all, so
        # camera_depth stays wired to the documented (currently inactive) topic.
        self._camera_calibration = None
        self._camera_sequence = 0
        camera_topic = "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed"
        camera_info_topic = "/aima/hal/sensor/rgb_head_front_center/camera_info"
        camera_rgb_topic = f"/{namespace}/agibot_x2/camera_rgb"
        camera_frame_topic = f"/{namespace}/agibot_x2/camera_rgb_frame"
        self.camera_rgb_pub = core_publisher(self.core, CompressedImage, camera_rgb_topic, 5)
        self.camera_frame_pub = core_publisher(self.core, UInt8MultiArray, camera_frame_topic, 5)
        self.robot.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, sensor_qos)
        self.robot.create_subscription(CompressedImage, camera_topic, self._camera_rgb_callback, sensor_qos)
        self.streams["camera_rgb"] = {"robot_topic": camera_topic, "topic": camera_rgb_topic, "format": "image/jpeg"}
        self.streams["camera_rgb_frame"] = {
            "robot_topic": camera_topic,
            "topic": camera_frame_topic,
            "format": "application/vnd.phanthy.sensor-envelope.v1",
            "ros_type": "std_msgs/msg/UInt8MultiArray",
            "schema": "phanthy.sensor.camera_rgb_frame.v1",
        }
        if stream_enabled("camera_depth", default=False):
            mirror("camera_depth", Image, "/aima/hal/sensor/rgbd_head_front/depth_image", "image/depth-z16", qos=sensor_qos)
        if stream_enabled("lidar", default=False):
            mirror("lidar", PointCloud2, "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud", "sensor/pointcloud", qos=sensor_qos)
        if stream_enabled("slam", default=False):
            mirror("slam_odom", Odometry, "/slam/lidar_odom", "data/json", qos=sensor_qos)

        skeleton_topic = f"/{namespace}/{SKELETON_TOPIC}"
        self.skeleton_pub = core_publisher(self.core, String, skeleton_topic, 5)
        self.streams["joints"] = {
            "robot_topic": "/aima/hal/joint/{leg,waist,arm,head}/state",
            "topic": skeleton_topic,
            "format": "sensor/skeleton",
        }
        # The shared runtime starts both executors before constructing this
        # object. Create the output before high-rate joint callbacks can fire.
        for area, topic in ((area, f"/aima/hal/joint/{area}/state") for area in JOINT_AREAS):
            self.robot.create_subscription(
                JointStateArray, topic, self._joint_state_callback(area), sensor_qos,
            )

        # /integrated_command and /relocalization_pose are outbound-only (SLAM control), not
        # mirrored streams -- they are plain publishers used by SlamControlPlugin.
        self.integrated_command_pub = self.robot.create_publisher(String, "/integrated_command", command_qos)
        self.relocalization_pose_pub = self.robot.create_publisher(Pose, "/relocalization_pose", 10)

        self.joint_command_pubs = {
            area: self.robot.create_publisher(JointCommandArray, f"/aima/hal/joint/{area}/command", 10)
            for area in JOINT_AREAS
        }
        self.hand_command_pub = self.robot.create_publisher(HandCommandArray, "/aima/hal/joint/hand/command", 10)
        self.locomotion_pub = self.robot.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)

        def client(srv_type, name):
            return self.robot.create_client(srv_type, name)

        self.get_all_joint_state = client(GetAllJointState, "/aimdk_5Fmsgs/srv/GetAllJointState")
        self.get_hand_type = client(GetHandType, "/aimdk_5Fmsgs/srv/GetHandType")
        self.get_mc_action = client(GetMcAction, "/aimdk_5Fmsgs/srv/GetMcAction")
        self.set_mc_action = client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        self.set_mc_preset_motion = client(SetMcPresetMotion, "/aimdk_5Fmsgs/srv/SetMcPresetMotion")
        self.set_mc_input_source = client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.get_current_input_source = client(GetCurrentInputSource, "/aimdk_5Fmsgs/srv/GetCurrentInputSource")
        self.get_system_state = client(GetSystemState, "/aimdk_5Fmsgs/srv/GetSystemState")
        self.get_robot_resources = client(GetRobotResources, "/aimdk_5Fmsgs/srv/GetRobotResources")
        self.execute_action_resource = client(ExecuteActionResource, "/aimdk_5Fmsgs/srv/ExecuteActionResource")
        self.set_pmu_led = client(SetPmuLed, "/aimdk_5Fmsgs/srv/SetPmuLed")
        self.play_tts = client(PlayTts, "/aimdk_5Fmsgs/srv/PlayTts")
        self.play_emoji = client(PlayEmoji, "/aimdk_5Fmsgs/srv/PlayEmoji")
        self.set_mic_source = client(SetMicSourceRequest, "/aimdk_5Fmsgs/srv/SetMicSourceRequest")
        self.get_mic_source = client(GetMicSourceRequest, "/aimdk_5Fmsgs/srv/GetMicSourceRequest")
        self.get_stored_map = client(GetStoredMapByName, "/aimdk_5Fmsgs/srv/GetStoredMapByName")

    def _callback(self, key, publisher, *, as_json=False, max_hz=None, transform=None):
        from std_msgs.msg import String

        def callback(msg):
            if max_hz is not None:
                now = time.monotonic()
                with self.lock:
                    previous = self._last_stream_publish.get(key, 0.0)
                    if now - previous < 1.0 / max_hz:
                        return
                    self._last_stream_publish[key] = now
            if as_json:
                # only the data/json path is ever read back via snapshot(); computing this for
                # binary streams (camera/lidar) would mean converting a JPEG/pointcloud byte
                # array into a full Python list on every frame for nothing, which was throttling
                # camera_rgb to ~8fps despite the source publishing at 30Hz.
                value = transform(msg) if transform else jsonable(msg)
                output = String()
                output.data = json.dumps(value, ensure_ascii=False)
                publisher.publish(output)
                with self.lock:
                    self.values[key] = value
            else:
                publisher.publish(msg)
        return callback

    @staticmethod
    def _hand_state_payload(msg):
        def hand_payload(hand_type, states, sensors):
            type_value = int(getattr(hand_type, "value", 0))
            joints = [
                {
                    "name": str(getattr(state, "name", "")),
                    "position": float(getattr(state, "position", 0.0)),
                    "velocity": float(getattr(state, "velocity", 0.0)),
                    "effort": float(getattr(state, "effort", 0.0)),
                    "state": int(getattr(state, "state", 0)),
                    "fault_code": int(getattr(state, "faultcode", 0)),
                }
                for state in states
            ]
            active_touch_channels = 0
            for field in (
                "palm_touch_data", "back_of_hand_touch_data", "thumb_touch_data",
                "index_finger_touch_data", "middle_finger_touch_data",
                "ring_finger_touch_data", "little_finger_touch_data",
            ):
                active_touch_channels += sum(bool(value) for value in getattr(sensors, field, []))
            return {
                "type": HAND_TYPES.get(type_value, "unknown"),
                "available": type_value not in (0, 255) or bool(joints),
                "joint_count": len(joints),
                "active_touch_channels": active_touch_channels,
                "joints": joints,
            }

        left = hand_payload(
            getattr(msg, "left_hand_type", None), getattr(msg, "left_hands", []),
            getattr(msg, "left_touch_sensors", None),
        )
        right = hand_payload(
            getattr(msg, "right_hand_type", None), getattr(msg, "right_hands", []),
            getattr(msg, "right_touch_sensors", None),
        )
        return {"available": left["available"] or right["available"], "left": left, "right": right}

    def _imu_callback(self, source, publisher):
        from std_msgs.msg import String

        def callback(msg):
            with self.lock:
                combined = self.values.setdefault("imu", {})
                combined.update(self._imu_payload(source, msg))
                snapshot = dict(combined)
            output = String()
            output.data = json.dumps(snapshot, ensure_ascii=False)
            publisher.publish(output)
        return callback

    @staticmethod
    def _float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _int(value, default=0):
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @classmethod
    def _fixed_float_fields(cls, prefix, values, length):
        """Preserve fixed-size numeric arrays without spamming empty covariances.

        Vendor often leaves covariance unset (all zeros). Those collapse into one
        nested list field so the canvas stays readable. Any non-zero content is
        still expanded to top-level scalars so nothing is dropped.
        """
        try:
            values = list(values)
        except TypeError:
            values = []
        floats = [cls._float(values[index] if index < len(values) else 0.0) for index in range(length)]
        if all(abs(item) <= 1e-12 for item in floats):
            return {prefix: floats}
        return {f"{prefix}_{index:02d}": floats[index] for index in range(length)}

    @classmethod
    def _quaternion_rpy_deg(cls, quaternion):
        x = cls._float(getattr(quaternion, "x", 0.0))
        y = cls._float(getattr(quaternion, "y", 0.0))
        z = cls._float(getattr(quaternion, "z", 0.0))
        w = cls._float(getattr(quaternion, "w", 1.0), 1.0)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(pitch_term)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))

    @classmethod
    def _imu_payload(cls, source, msg):
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        orientation = getattr(msg, "orientation", None)
        angular = getattr(msg, "angular_velocity", None)
        acceleration = getattr(msg, "linear_acceleration", None)
        roll, pitch, yaw = cls._quaternion_rpy_deg(orientation)
        return {
            f"{source}_frame_id": str(getattr(header, "frame_id", "")),
            f"{source}_stamp_sec": cls._int(getattr(stamp, "sec", 0)),
            f"{source}_stamp_nanosec": cls._int(getattr(stamp, "nanosec", 0)),
            f"{source}_orientation_x": cls._float(getattr(orientation, "x", 0.0)),
            f"{source}_orientation_y": cls._float(getattr(orientation, "y", 0.0)),
            f"{source}_orientation_z": cls._float(getattr(orientation, "z", 0.0)),
            f"{source}_orientation_w": cls._float(getattr(orientation, "w", 1.0), 1.0),
            f"{source}_roll_deg": roll,
            f"{source}_pitch_deg": pitch,
            f"{source}_yaw_deg": yaw,
            f"{source}_angular_x_rad_s": cls._float(getattr(angular, "x", 0.0)),
            f"{source}_angular_y_rad_s": cls._float(getattr(angular, "y", 0.0)),
            f"{source}_angular_z_rad_s": cls._float(getattr(angular, "z", 0.0)),
            f"{source}_accel_x_m_s2": cls._float(getattr(acceleration, "x", 0.0)),
            f"{source}_accel_y_m_s2": cls._float(getattr(acceleration, "y", 0.0)),
            f"{source}_accel_z_m_s2": cls._float(getattr(acceleration, "z", 0.0)),
            **cls._fixed_float_fields(
                f"{source}_orientation_covariance", getattr(msg, "orientation_covariance", ()), 9,
            ),
            **cls._fixed_float_fields(
                f"{source}_angular_velocity_covariance", getattr(msg, "angular_velocity_covariance", ()), 9,
            ),
            **cls._fixed_float_fields(
                f"{source}_linear_acceleration_covariance", getattr(msg, "linear_acceleration_covariance", ()), 9,
            ),
        }

    @classmethod
    def _leg_odometry_payload(cls, msg):
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        pose = getattr(msg, "pose", None)
        position = getattr(getattr(pose, "pose", None), "position", None)
        orientation = getattr(getattr(pose, "pose", None), "orientation", None)
        twist = getattr(getattr(msg, "twist", None), "twist", None)
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        roll, pitch, yaw = cls._quaternion_rpy_deg(orientation)
        return {
            "frame_id": str(getattr(header, "frame_id", "")),
            "child_frame_id": str(getattr(msg, "child_frame_id", "")),
            "stamp_sec": cls._int(getattr(stamp, "sec", 0)),
            "stamp_nanosec": cls._int(getattr(stamp, "nanosec", 0)),
            "position_x_m": cls._float(getattr(position, "x", 0.0)),
            "position_y_m": cls._float(getattr(position, "y", 0.0)),
            "position_z_m": cls._float(getattr(position, "z", 0.0)),
            "orientation_x": cls._float(getattr(orientation, "x", 0.0)),
            "orientation_y": cls._float(getattr(orientation, "y", 0.0)),
            "orientation_z": cls._float(getattr(orientation, "z", 0.0)),
            "orientation_w": cls._float(getattr(orientation, "w", 1.0), 1.0),
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
            "linear_x_m_s": cls._float(getattr(linear, "x", 0.0)),
            "linear_y_m_s": cls._float(getattr(linear, "y", 0.0)),
            "linear_z_m_s": cls._float(getattr(linear, "z", 0.0)),
            "angular_x_rad_s": cls._float(getattr(angular, "x", 0.0)),
            "angular_y_rad_s": cls._float(getattr(angular, "y", 0.0)),
            "angular_z_rad_s": cls._float(getattr(angular, "z", 0.0)),
            "yaw_rate_deg_s": math.degrees(cls._float(getattr(angular, "z", 0.0))),
            **cls._fixed_float_fields("pose_covariance", getattr(pose, "covariance", ()), 36),
            **cls._fixed_float_fields("twist_covariance", getattr(getattr(msg, "twist", None), "covariance", ()), 36),
        }

    def _audio_capture_callback(self, msg):
        """Forward vendor AudioCapture PCM in ASR-compatible 1 KiB chunks."""
        info = getattr(msg, "info", None)
        rate = int(getattr(info, "sample_rate", 16000) or 16000)
        channels = int(getattr(info, "channels", 1) or 1)
        mic_channels = int(getattr(msg, "mic_channels", channels) or channels)
        mic_channels = max(1, min(channels, mic_channels))
        sample_format = str(getattr(info, "sample_format", "S16LE") or "S16LE").upper()
        raw = getattr(getattr(msg, "data", None), "data", ())
        try:
            payload = bytes(int(value) & 0xFF for value in raw)
        except (TypeError, ValueError):
            return
        if rate != 16000 or sample_format not in {"S16LE", "PCM_S16LE", ""}:
            with self._mic_audio_lock:
                self._mic_capture_stats.update({"sample_rate": rate, "channels": channels, "sample_format": sample_format})
            return
        # X2 currently exposes six interleaved S16LE channels (mic + reference
        # channels).  The shared ASR contract is mono, so average the physical
        # microphone channels and discard only the reference channels.
        if channels > 1:
            sample_count = len(payload) // (2 * channels)
            try:
                samples = struct.unpack(f"<{sample_count * channels}h", payload[:sample_count * channels * 2])
            except struct.error:
                return
            mono = bytearray(sample_count * 2)
            for index in range(sample_count):
                start = index * channels
                value = round(sum(samples[start:start + mic_channels]) / mic_channels)
                try:
                    gain = float(self.config.get("plugins", {}).get("mic", {}).get("gain", 4.0))
                except (TypeError, ValueError):
                    gain = 4.0
                value = round(value * max(0.5, min(8.0, gain)))
                struct.pack_into("<h", mono, index * 2, max(-32768, min(32767, value)))
            payload = bytes(mono)
        with self._mic_audio_lock:
            self._mic_capture_stats.update({"sample_rate": rate, "channels": channels, "mic_channels": mic_channels})
            self._mic_audio_buffer.extend(payload)
            chunks = []
            while len(self._mic_audio_buffer) >= 1024:
                chunks.append(bytes(self._mic_audio_buffer[:1024]))
                del self._mic_audio_buffer[:1024]
        for chunk in chunks:
            output = self._AudioChunk()
            output.format = "audio/pcm-16k"
            output.data = list(chunk)
            self.mic_audio_pub.publish(output)
            with self._mic_audio_lock:
                self._mic_capture_stats["chunks"] += 1
                self._mic_capture_stats["bytes"] += len(chunk)

    def _camera_info_callback(self, msg):
        from x2_camera_frame import calibration_from_camera_info

        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "")) or "rgb_head_center_link"
        try:
            calibration = calibration_from_camera_info(msg, frame_id)
        except (TypeError, ValueError):
            return
        with self.lock:
            self._camera_calibration = calibration

    def _mc_common_state_callback(self, msg):
        action_info = getattr(msg, "action_info", None)
        status = getattr(action_info, "status", None)
        fsm_state = getattr(msg, "fsm_state", None)
        try:
            action_status = int(getattr(status, "value", None))
        except (TypeError, ValueError):
            action_status = None
        try:
            fsm_value = int(getattr(fsm_state, "current_state", None))
        except (TypeError, ValueError):
            fsm_value = None
        with self.lock:
            self._mc_mode_state = {
                "action_desc": str(getattr(action_info, "action_desc", "")),
                "action_status": action_status,
                "fsm_state": fsm_value,
            }

    def mc_mode_state(self):
        with self.lock:
            return dict(self._mc_mode_state)

    def _camera_rgb_callback(self, msg):
        from array import array
        from x2_camera_frame import build_rgb_metadata, encode_envelope

        now = time.monotonic()
        with self.lock:
            if now - self._last_stream_publish.get("camera_rgb", 0.0) < 1.0 / CAMERA_RGB_MAX_HZ:
                return
            self._last_stream_publish["camera_rgb"] = now
            calibration = self._camera_calibration
            self._camera_sequence += 1
            sequence = self._camera_sequence
        self.camera_rgb_pub.publish(msg)
        if calibration is None:
            return
        try:
            metadata = build_rgb_metadata(msg, calibration, sequence)
            envelope = encode_envelope(metadata, getattr(msg, "data", b""))
            framed = self._msg["UInt8MultiArray"]()
            framed.data = array("B", envelope)
            self.camera_frame_pub.publish(framed)
        except (TypeError, ValueError, OverflowError):
            return

    def _joint_state_callback(self, area):
        def callback(msg):
            with self.lock:
                first_update_for_area = self.joint_groups.get(area) is None
                self.joint_groups[area] = msg
                snapshot = self._skeleton_snapshot_locked()
                now = time.monotonic()
                if (
                    not first_update_for_area
                    and now - self._last_skeleton_publish < 1.0 / SKELETON_MAX_HZ
                ):
                    return
                self._last_skeleton_publish = now
            output = self._msg["String"]()
            output.data = json.dumps(snapshot, ensure_ascii=False)
            self.skeleton_pub.publish(output)
        return callback

    def _skeleton_snapshot_locked(self):
        joints = []
        unknown_names = []
        for area in self.skeleton_joints:
            msg = self.joint_groups.get(area)
            if msg is None:
                continue
            for state in getattr(msg, "joints", []):
                name = getattr(state, "name", "")
                idx = self.skeleton_joint_indices.get(name)
                if idx is None:
                    if name:
                        unknown_names.append(name)
                    continue
                item = {
                    "idx": idx,
                    "name": name,
                    "q": float(state.position),
                    "dq": float(state.velocity),
                    "tau": float(state.effort),
                }
                if getattr(state, "error_code", 0):
                    item["error_code"] = int(state.error_code)
                joints.append(item)
        payload = {"format": "sensor/skeleton", "joints": joints, "joint_count": len(joints), "position_unit": "rad"}
        if unknown_names:
            payload["diagnostics"] = {"unknown_joint_names": unknown_names}
        return payload

    def skeleton_snapshot(self):
        with self.lock:
            return self._skeleton_snapshot_locked()

    def request_header(self):
        # CommonRequest.header is typed RequestHeader, which per the vendor schema has only
        # a `stamp` field (no `frame_id` — that belongs to the separate MessageHeader type
        # used by outbound command messages, not by CommonRequest).
        request = self._msg["CommonRequest"]()
        request.header.stamp = self.robot.get_clock().now().to_msg()
        return request

    def snapshot(self, key):
        with self.lock:
            return self.values.get(key, {})

    def urdf_text(self, variant=None):
        variant = (variant or self.end_effector).lower()
        path = RESOURCE_DIR / f"x2_{variant}.urdf"
        if not path.exists():
            raise ValueError(f"no URDF vendored for end_effector variant '{variant}'")
        return path.read_text(encoding="utf-8")

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()


def _stream_tool(key, stream, description):
    entry = {"topic": stream["topic"], "format": stream["format"]}
    for field in ("ros_type", "schema"):
        if field in stream:
            entry[field] = stream[field]
    return tool(key, "sensor", description, topic_out=[entry])


class McStatePlugin:
    """GetMcAction — the FSM has no dedicated status *topic* in AimDK's catalog, so this is a
    synchronous service query rather than a mirrored stream."""

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("mc_state", "sensor", "查询当前 MC 运控状态机模式（GetMcAction）")

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
        from aimdk_msgs.srv import GetMcAction
        request = GetMcAction.Request()
        request.request = self.nodes.request_header()
        result = call_service(self.nodes.get_mc_action, request)
        return jsonable(result.info)


class JointsPlugin:
    """Publish X2 joint feedback in the sensor/skeleton contract for 3D rendering."""

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return _stream_tool("joints", self.nodes.streams["joints"], "X2 全身关节实时骨架（由 JointStateArray 驱动）")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "joints"):
            return {"state": "running", "data": self.nodes.skeleton_snapshot(), **self.nodes.streams["joints"]}
        return None


class JointStatePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("joint_state", "sensor", "查询全身关节状态：leg/waist/arm/head（GetAllJointState）")

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
        from aimdk_msgs.srv import GetAllJointState
        request = GetAllJointState.Request()
        request.request = self.nodes.request_header()
        result = call_service(self.nodes.get_all_joint_state, request)
        return {
            "leg": jsonable(result.leg_joints),
            "waist": jsonable(result.waist_joints),
            "arm": jsonable(result.arm_joints),
            "head": jsonable(result.head_joints),
        }


class HandStatePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        stream = self.nodes.streams["hand_state"]
        return _stream_tool("hand_state", stream, "手部关节 + 触摸传感器状态流（含 HandType）")

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
            return {
                "state": "running",
                "data": self.nodes.snapshot("hand_state"),
                **self.nodes.streams["hand_state"],
            }
        if action in ("read", "get", "hand_state"):
            return {"state": "running", "data": self.nodes.snapshot("hand_state"), **self.nodes.streams["hand_state"]}
        return None


class ImuPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return _stream_tool("imu", self.nodes.streams["imu"], "胸部+躯干 IMU 合并数据流")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "imu"):
            return {"state": "running", "data": self.nodes.snapshot("imu"), **self.nodes.streams["imu"]}
        return None


class CameraPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tools(self):
        tools = [
            _stream_tool("camera_rgb", self.nodes.streams["camera_rgb"], "前置 RGBD 相机彩色画面（压缩 JPEG）"),
            _stream_tool("camera_rgb_frame", self.nodes.streams["camera_rgb_frame"], "带时间、内参和名义机身外参的自描述前置 RGB 帧"),
        ]
        if "camera_depth" in self.nodes.streams:
            tools.append(_stream_tool("camera_depth", self.nodes.streams["camera_depth"], "前置 RGBD 相机深度画面"))
        return tools

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "camera_rgb", "camera_rgb_frame", "camera_depth"):
            if name not in self.nodes.streams:
                return None
            return {"state": "running", "data": self.nodes.snapshot(name), **self.nodes.streams[name]}
        return None


class ReadOnlyStreamPlugin:
    """Expose a JSON-mirrored vendor topic as a read-only sensor card."""

    def __init__(self, nodes, name, description):
        self.nodes = nodes
        self.name = name
        self.description = description

    def get_tool(self):
        return _stream_tool(self.name, self.nodes.streams[self.name], self.description)

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", self.name):
            return {"state": "running", "data": self.nodes.snapshot(self.name), **self.nodes.streams[self.name]}
        return None


class LidarPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return _stream_tool("lidar", self.nodes.streams["lidar"], "胸前激光雷达点云")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "lidar"):
            return {"state": "running", "data": self.nodes.snapshot("lidar"), **self.nodes.streams["lidar"]}
        return None


class SlamPosePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return _stream_tool("slam_pose", self.nodes.streams["slam_odom"], "SLAM 激光里程计位姿（/slam/lidar_odom）")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "slam_pose"):
            return {"state": "running", "data": self.nodes.snapshot("slam_odom"), **self.nodes.streams["slam_odom"]}
        return None


class SystemStatePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("system_state", "sensor", "查询系统状态机当前状态（GetSystemState）")

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
            return {"state": "running", "data": self.nodes.snapshot("system_state")}
        from aimdk_msgs.srv import GetSystemState
        request = GetSystemState.Request()
        request.header = self.nodes.request_header()
        result = call_service(self.nodes.get_system_state, request)
        return {"data": {"cur_state": result.cur_state, "status": jsonable(result.curr_status)}}


class LinkcraftCatalogPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("linkcraft_catalog", "sensor", "查询机上可用的灵创动作资源列表（GetRobotResources）")

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
            return {"state": "running", "data": self.nodes.snapshot("linkcraft_catalog")}
        from aimdk_msgs.srv import GetRobotResources
        request = GetRobotResources.Request()
        request.header = self.nodes.request_header()
        result = call_service(self.nodes.get_robot_resources, request)
        return {"data": {"resources": jsonable(result.robot_resources)}}


class ModelPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool(
            "model", "resource", "返回配置的末端执行器变体（fist/hand/ultra）对应的 URDF",
            {
                "type": "object",
                "properties": {"variant": {"type": "string", "enum": ["fist", "hand", "ultra"]}},
            },
        )

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running"}
        return {"urdf": self.nodes.urdf_text(args.get("variant"))}


class McModePlugin:
    ACTIONS = {
        "damping_default": ([], "进入阻尼模式：关节有阻尼但不保持姿态，机器人会缓慢倒地"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        configured = nodes.config.get("plugins", {}).get("mc_mode", {}).get("allowed_actions", list(MC_ACTIONS))
        self.actions = {name: MC_ACTIONS[name] for name in configured if name in MC_ACTIONS}

    def get_tool(self):
        actions = {name: self.ACTIONS[name] for name in self.actions}
        schema = action_schema(actions, {})
        return tool(
            "mc_mode", "actuator",
            "X2 已由实机固件确认的运控模式。平躺/平趴站起是遥控器专用恢复流程，不通过此服务提供。",
            _with_actuation_contract(
                schema,
                ["leg", "waist", "arm_l", "arm_r", "head"],
                {"actions": list(actions), "timeout": 30},
            ),
        )

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
        if action not in self.actions:
            raise ValueError(f"mc_mode: action {action!r} is unavailable on this X2 firmware")
        from aimdk_msgs.srv import SetMcAction
        request = SetMcAction.Request()
        request.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        request.source = self.nodes.config.get("plugins", {}).get("mc_mode", {}).get("input_source_name", "phanthymotus")
        # vendor's own set_mc_action.py example never sets command.action.value at all — the
        # firmware looks the mode up by action_desc's exact string, matching McAction.msg's
        # UPPERCASE constant name (e.g. "STAND_DEFAULT"), not the integer value or our
        # lowercase snake_case key. Sending action_desc="stand_body_control" is what produced
        # the literal firmware error "can not find action: stand_body_control".
        request.command.action.value = self.actions[action]
        request.command.action_desc = action.upper()
        timeout = float(self.nodes.config.get("plugins", {}).get("mc_mode", {}).get("service_timeout_sec", 20))
        result = call_service(self.nodes.set_mc_action, request, timeout=timeout)
        response = result.response
        try:
            accepted = int(response.header.code) == 0
        except (AttributeError, TypeError, ValueError):
            accepted = True  # SDK test doubles and old firmware responses omit a numeric code.
        response_data = jsonable(response)
        if not accepted:
            return {"state": "rejected", "action": action, "response": response_data}

        action_id = f"x2_mc_mode_{uuid4().hex[:12]}"
        if not self.nodes.mc_state_available:
            threading.Thread(
                target=_acp_notify,
                args=(action_id, "completed", {
                    "action": action,
                    "completion": "service_accepted_state_unavailable",
                    "response": response_data,
                }, "mc_mode"),
                daemon=True,
            ).start()
            return {
                "state": "accepted",
                "action": action,
                "action_id": action_id,
                "response": response_data,
                "confirmation": "service_accepted_state_unavailable",
            }
        threading.Thread(
            target=self._wait_for_mode_confirmation,
            args=(action_id, action.upper(), action, 30.0),
            daemon=True,
        ).start()
        return {
            "state": "accepted",
            "action": action,
            "action_id": action_id,
            "response": response_data,
        }

    def _wait_for_mode_confirmation(self, action_id, action_desc, action, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.nodes.mc_mode_state()
            if state["action_desc"] == action_desc and state["action_status"] == 100:
                _acp_notify(action_id, "completed", {
                    "action": action,
                    "action_desc": action_desc,
                    "action_status": state["action_status"],
                    "fsm_state": state["fsm_state"],
                    "completion": "mode_active",
                }, "mc_mode")
                return
            time.sleep(0.1)
        _acp_notify(action_id, "error", {
            "action": action,
            "action_desc": action_desc,
            "last_state": self.nodes.mc_mode_state(),
            "error": "mode_confirmation_timeout",
        }, "mc_mode")


class LocomotionPlugin:
    ACTIONS = {
        "move": (["forward", "lateral", "angular", "duration"],
                 "移动；duration=-1 持续移动，否则按时长刹停。速度按 m/s 与 °/s 下发，可用配置/里程计测速标定，避免只靠距离闭环掩盖速度误差"),
        "cancel": ([], "立即停止当前行走速度并取消定时动作"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self._registered = False
        self._lock = threading.RLock()
        self._stop_timer = None
        self._velocity_timer = None
        self._motion_generation = 0
        self._active_action_id = None
        self._command_forward = 0.0
        self._command_lateral = 0.0
        self._command_angular = 0.0
        self._motion_started_mono = None
        self._motion_duration = None
        self._adaptive_scale = 1.0
        self._last_measured = None
        # The vendor input-source registry is not re-entrant.  Serialize
        # ADD/MODIFY/ENABLE/DELETE, especially when a duration timer releases
        # the previous action while a new turn command is arriving.
        self._input_source_lock = threading.Lock()

    def get_tool(self):
        schema = action_schema(self.ACTIONS, {
            "forward": {"type": "number", "default": 0.2,
                        "description": "前进速度 m/s，+前进/-后退；默认 0.2（厂商单位，经 velocity 标定后下发）"},
            "lateral": {"type": "number", "description": "侧移速度 m/s，+左移/-右移"},
            "angular": {"type": "number", "description": "转向角速度 °/s，+左转/-右转；驱动内部换算为 rad/s"},
            "duration": {"type": "number", "minimum": -1, "maximum": 60, "default": -1,
                         "description": "持续时间（秒）。-1 持续移动；有限时长到点发零速。位移应约等于速度×时间；若明显偏离，检查速度标定而非只改刹停。"},
        })
        schema["allOf"] = [{
            "if": {"properties": {"action": {"const": "move"}}, "required": ["action"]},
            "then": {"required": ["duration"]},
        }]
        return tool("locomotion", "actuator", "MC 行走速度控制：动作开始时自动注册独立输入源，在稳定站立下直接进入走跑。动作期间该源优先于遥控器；"
                    "取消、定时完成和驱动停止都会删除它并把控制权还给遥控器。"
                    "有限 duration 以时钟为准刹停；用 leg_odometry 的 twist 标定实际跟踪增益，里程计位移仅作安全上限。",
                    _with_actuation_contract(schema, ["base", "leg"]))

    def start(self):
        pass

    def stop(self):
        self._cancel_motion("driver_stopped", send_stop=True)
        self._release_input_source()

    def _plugin_cfg(self):
        return self.nodes.config.get("plugins", {}).get("locomotion", {}) or {}

    def _configured_velocity_scale(self):
        raw = self._plugin_cfg().get("velocity_command_scale", 1.0)
        try:
            scale = float(raw)
        except (TypeError, ValueError):
            scale = 1.0
        return max(LOCOMOTION_VEL_SCALE_MIN, min(LOCOMOTION_VEL_SCALE_MAX, scale))

    def _adaptive_scale_enabled(self):
        return bool(self._plugin_cfg().get("adaptive_velocity_scale", True))

    @staticmethod
    def _wrap_angle_deg(delta):
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        return delta

    def _leg_odom_snapshot(self):
        snapshot = self.nodes.snapshot("leg_odometry") or {}
        if not snapshot:
            return None
        try:
            return {
                "x": float(snapshot["position_x_m"]),
                "y": float(snapshot["position_y_m"]),
                "yaw_deg": float(snapshot["yaw_deg"]),
                "linear_x": float(snapshot.get("linear_x_m_s", 0.0)),
                "linear_y": float(snapshot.get("linear_y_m_s", 0.0)),
                "angular_z": float(snapshot.get("angular_z_rad_s", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            return None

    def _effective_scale(self):
        scale = self._configured_velocity_scale()
        if self._adaptive_scale_enabled():
            scale *= self._adaptive_scale
        return max(LOCOMOTION_VEL_SCALE_MIN, min(LOCOMOTION_VEL_SCALE_MAX, scale))

    def _scaled_command(self, forward, lateral, angular):
        scale = self._effective_scale()
        return forward * scale, lateral * scale, angular * scale, scale

    def _update_adaptive_scale(self, forward, lateral, angular):
        if not self._adaptive_scale_enabled():
            return
        odom = self._leg_odom_snapshot()
        if odom is None:
            return
        cmd_lin = math.hypot(float(forward), float(lateral))
        meas_lin = math.hypot(odom["linear_x"], odom["linear_y"])
        cmd_ang = abs(float(angular))
        meas_ang = abs(odom["angular_z"])
        ratios = []
        # Prefer linear ratio when both are moving; angular is secondary.
        if cmd_lin >= LOCOMOTION_VEL_SCALE_MIN_SPEED and meas_lin >= LOCOMOTION_VEL_SCALE_MIN_SPEED:
            ratios.append(cmd_lin / meas_lin)
        elif cmd_ang >= 0.05 and meas_ang >= 0.05:
            ratios.append(cmd_ang / meas_ang)
        if not ratios:
            self._last_measured = {
                "linear_speed_m_s": meas_lin,
                "angular_z_rad_s": odom["angular_z"],
            }
            return
        sample = sum(ratios) / len(ratios)
        sample = max(LOCOMOTION_VEL_SCALE_MIN, min(LOCOMOTION_VEL_SCALE_MAX, sample))
        alpha = LOCOMOTION_VEL_SCALE_EMA
        self._adaptive_scale = (1.0 - alpha) * self._adaptive_scale + alpha * sample
        self._adaptive_scale = max(LOCOMOTION_VEL_SCALE_MIN, min(LOCOMOTION_VEL_SCALE_MAX, self._adaptive_scale))
        self._last_measured = {
            "linear_speed_m_s": meas_lin,
            "angular_z_rad_s": odom["angular_z"],
            "scale_sample": sample,
            "adaptive_scale": self._adaptive_scale,
        }

    def _safety_should_stop(self, start_pose, duration, forward, lateral, angular):
        if start_pose is None or duration is None or duration <= 0:
            return None
        odom = self._leg_odom_snapshot()
        if odom is None:
            return None
        distance = math.hypot(odom["x"] - start_pose["x"], odom["y"] - start_pose["y"])
        yaw_delta = abs(self._wrap_angle_deg(odom["yaw_deg"] - start_pose["yaw_deg"]))
        expect_m = abs(math.hypot(forward, lateral) * duration)
        expect_deg = abs(math.degrees(angular) * duration)
        over_m = expect_m >= LOCOMOTION_ODOM_SAFETY_MIN_M and distance >= expect_m * LOCOMOTION_ODOM_SAFETY_FACTOR
        over_yaw = expect_deg >= LOCOMOTION_ODOM_SAFETY_MIN_DEG and yaw_delta >= expect_deg * LOCOMOTION_ODOM_SAFETY_FACTOR
        if not (over_m or over_yaw):
            return None
        return {
            "distance_m": distance,
            "yaw_delta_deg": self._wrap_angle_deg(odom["yaw_deg"] - start_pose["yaw_deg"]),
            "expected_distance_m": expect_m,
            "expected_yaw_delta_deg": math.degrees(angular) * duration,
        }

    def _complete_motion(self, action_id, *, status, reason, extra=None):
        with self._lock:
            if self._active_action_id != action_id:
                return False
            self._motion_generation += 1
            self._active_action_id = None
            timer, self._stop_timer = self._stop_timer, None
            velocity_timer, self._velocity_timer = self._velocity_timer, None
            started = self._motion_started_mono
            duration = self._motion_duration
            measured = dict(self._last_measured) if self._last_measured else None
            scale = self._effective_scale()
            self._motion_started_mono = None
            self._motion_duration = None
        if timer is not None:
            timer.cancel()
        if velocity_timer is not None:
            velocity_timer.cancel()
        self._publish_velocity()
        payload = {
            "reason": reason,
            "topic": "/aima/mc/locomotion/velocity",
            "final_velocity": {"forward": 0.0, "lateral": 0.0, "angular": 0.0},
            "velocity_command_scale": scale,
            "adaptive_scale": self._adaptive_scale,
        }
        if duration is not None:
            payload["duration"] = duration
        if started is not None:
            payload["elapsed_s"] = max(0.0, time.monotonic() - started)
        if measured:
            payload["measured"] = measured
        if extra:
            payload.update(extra)
        # Notify ACP immediately after the velocity stop.  Vendor DELETE can
        # block for several seconds; it must not delay or suppress the action
        # completion event seen by the caller.
        _acp_notify(action_id, status, payload, "locomotion")
        try:
            self._release_input_source()
        except Exception as exc:
            print(f"[locomotion] input source cleanup failed: {str(exc)[:200]}", flush=True)
        return True

    def _publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = self.nodes._McLocomotionVelocity()
        msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        msg.source = self._source_name()
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.nodes.locomotion_pub.publish(msg)

    def _cancel_motion(self, reason, send_stop=False):
        with self._lock:
            self._motion_generation += 1
            timer, self._stop_timer = self._stop_timer, None
            velocity_timer, self._velocity_timer = self._velocity_timer, None
            action_id, self._active_action_id = self._active_action_id, None
            self._motion_started_mono = None
            self._motion_duration = None
        if timer is not None:
            timer.cancel()
        if velocity_timer is not None:
            velocity_timer.cancel()
        if send_stop and action_id is not None:
            self._publish_velocity()
        if action_id is not None:
            _acp_notify(action_id, "cancelled", {
                "reason": reason,
                "topic": "/aima/mc/locomotion/velocity",
            }, "locomotion")

    def _schedule_stop(self, duration, action_id):
        with self._lock:
            self._motion_generation += 1
            generation = self._motion_generation
            prior_timer, self._stop_timer = self._stop_timer, None
        if prior_timer is not None:
            prior_timer.cancel()

        def finish():
            with self._lock:
                if generation != self._motion_generation:
                    return
            self._complete_motion(
                action_id,
                status="completed",
                reason="duration_elapsed",
                extra={"stop_mode": "duration"},
            )

        timer = threading.Timer(duration, finish)
        timer.daemon = True
        with self._lock:
            if generation != self._motion_generation:
                return
            self._stop_timer = timer
        timer.start()

    def _start_velocity_stream(self, action_id, forward, lateral, angular, start_pose, duration):
        """Refresh the vendor input before its 1000 ms watchdog expires."""
        period = 1.0 / LOCOMOTION_HEARTBEAT_HZ
        with self._lock:
            generation = self._motion_generation

        def tick():
            with self._lock:
                if generation != self._motion_generation or self._active_action_id != action_id:
                    return
            # Root-cause path: compare commanded vs odom twist, adapt scale, then publish.
            self._update_adaptive_scale(forward, lateral, angular)
            safety = self._safety_should_stop(start_pose, duration, forward, lateral, angular)
            if safety is not None:
                self._complete_motion(
                    action_id,
                    status="completed",
                    reason="odom_safety_limit",
                    extra={"stop_mode": "odom_safety", **safety},
                )
                return
            pub_f, pub_l, pub_a, scale = self._scaled_command(forward, lateral, angular)
            self._publish_velocity(pub_f, pub_l, pub_a)
            timer = threading.Timer(period, tick)
            timer.daemon = True
            with self._lock:
                if generation != self._motion_generation or self._active_action_id != action_id:
                    return
                self._velocity_timer = timer
            timer.start()

        timer = threading.Timer(period, tick)
        timer.daemon = True
        with self._lock:
            if generation != self._motion_generation or self._active_action_id != action_id:
                return
            self._velocity_timer = timer
        timer.start()

    def _source_name(self):
        name = str(self._plugin_cfg().get("input_source_name", "motus_x2"))
        if not name or name in {"rc", "vr", "app_proxy", "interaction", "pnc"}:
            raise ValueError("locomotion: input_source_name must be a non-empty custom source name")
        return name

    def _set_input_source(self, mc_input_action):
        """Register/modify/delete the locomotion input source.

        AimDK's own DirectVelocityControl example retries SetMcInputSource because
        "remote peer is NOT handled well by ROS". On the live X2 driver the dual-domain
        MultiThreadedExecutor is also fed by high-rate sensor callbacks via spin_once,
        so a single 5s poll on the shared client often times out even though the MC
        service itself answers instantly on a dedicated node (verified on-robot).
        Production calls therefore use a short-lived domain-0 node that
        spin_until_future_complete's the request, matching the vendor example.
        """
        from aimdk_msgs.srv import SetMcInputSource
        plugin_cfg = self._plugin_cfg()
        name = self._source_name()
        priority = int(plugin_cfg.get("input_source_priority", 81))
        source_timeout_ms = int(plugin_cfg.get("input_source_watchdog_ms", 1000))
        attempts = max(1, int(plugin_cfg.get("input_source_attempts", 8)))
        attempt_timeout = float(plugin_cfg.get("input_source_attempt_timeout_sec", 0.5))

        def build_request():
            request = SetMcInputSource.Request()
            request.request = self.nodes.request_header()
            request.action.value = mc_input_action
            request.input_source.name = name
            request.input_source.priority = priority
            request.input_source.timeout = source_timeout_ms
            return request

        # Reuse the driver's already-running robot executor by default.  Creating
        # and tearing down a second rclpy Context/participant from inside the
        # dispatch process can crash the vendor Fast DDS binding (SIGSEGV on the
        # live X2).  A timeout is recoverable and will be surfaced to the card;
        # taking down the whole driver is not.
        # Production config opts into the isolated helper explicitly; retain a
        # shared-client default for embedded callers and ROS-free unit tests.
        transport = str(plugin_cfg.get("input_source_transport", "shared")).lower()
        if transport == "process":
            # The helper is isolated for DDS stability, but calls must still be
            # serialized: a timed stop/delete racing the next add is rejected
            # intermittently by the vendor input-source registry.
            with self._input_source_lock:
                return self._set_input_source_process(
                    action=mc_input_action,
                    name=name,
                    priority=priority,
                    timeout_ms=source_timeout_ms,
                    timeout_sec=max(8.0, attempt_timeout * attempts + 2.0),
                )
        if transport != "ephemeral":
            with self._input_source_lock:
                result = call_service(
                    self.nodes.set_mc_input_source,
                    build_request(),
                    timeout=max(5.0, attempt_timeout * attempts),
                )
            return jsonable(result.response)

        # Kept as an explicit opt-in for controlled experiments only.  It is not
        # the production default because the live SDK has demonstrated native
        # crashes when this context is repeatedly initialized and destroyed.
        return self._set_input_source_ephemeral(
            build_request=build_request,
            attempts=attempts,
            attempt_timeout=attempt_timeout,
            action=mc_input_action,
            name=name,
        )

    @staticmethod
    def _set_input_source_process(*, action, name, priority, timeout_ms, timeout_sec):
        """Call SetMcInputSource in a short-lived child process.

        The X2 vendor service is reliable from a standalone domain-0 ROS
        participant but can hang (or previously segfault) when called from the
        long-running driver participant. Isolating this probe keeps locomotion
        debuggable without taking down the MCP server.
        """
        helper = Path(__file__).with_name("x2_input_source_helper.py")
        command = [
            sys.executable, str(helper),
            "--action", str(int(action)),
            "--name", str(name),
            "--priority", str(int(priority)),
            "--timeout-ms", str(int(timeout_ms)),
            "--timeout-sec", str(float(timeout_sec)),
        ]
        try:
            completed = subprocess.run(
                command, capture_output=True, text=True,
                timeout=timeout_sec + 2.0, check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(f"SetMcInputSource helper exceeded {timeout_sec + 2.0:.1f}s") from exc
        # FastDDS on this vendor image can segfault during rclpy shutdown even
        # after the service response has already arrived. Parse stdout first so
        # an acknowledged request is not reported as a locomotion failure.
        parsed = None
        for line in reversed((completed.stdout or "").splitlines()):
            line = line.strip()
            if line.startswith("{"):
                try:
                    parsed = json.loads(line)
                except json.JSONDecodeError:
                    pass
                break
        if parsed is not None and completed.returncode in (0, -11):
            return parsed
        if completed.returncode in (124, -11):
            detail = (completed.stderr or completed.stdout or "helper timed out").strip()
            raise TimeoutError(f"SetMcInputSource helper timed out: {detail[-500:]}")
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "helper exited without detail").strip()
            raise RuntimeError(f"SetMcInputSource helper failed (exit {completed.returncode}): {detail[-500:]}")
        if parsed is not None:
            return parsed
        raise RuntimeError("SetMcInputSource helper returned no JSON response")

    def _set_input_source_ephemeral(self, *, build_request, attempts, attempt_timeout, action, name):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from aimdk_msgs.srv import SetMcInputSource

        domain_id = int(os.environ.get("ROBOT_DOMAIN_ID", self.nodes.config.get("ros", {}).get("robot_domain_id", 0)))
        ctx = Context()
        node = None
        executor = None
        last_error = None
        try:
            rclpy.init(context=ctx, domain_id=domain_id)
            node = Node(f"agibot_x2_mc_input_{action}", context=ctx)
            # Do not call rclpy.spin_until_future_complete(node, ...): on Humble
            # it uses the global executor, which is bound to the default context.
            # This driver owns two explicit contexts, so bind the executor explicitly.
            executor = SingleThreadedExecutor(context=ctx)
            executor.add_node(node)
            client = node.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
            if not client.wait_for_service(timeout_sec=max(2.0, attempt_timeout * 2)):
                raise TimeoutError("service /aimdk_5Fmsgs/srv/SetMcInputSource unavailable")
            for attempt in range(attempts):
                request = build_request()
                # Refresh stamp each attempt like the AimDK example.
                request.request = self.nodes.request_header()
                future = client.call_async(request)
                deadline = time.monotonic() + attempt_timeout
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
                if not future.done():
                    last_error = TimeoutError(
                        f"SetMcInputSource action={action} name={name!r} attempt {attempt + 1}/{attempts} timed out"
                    )
                    continue
                try:
                    result = future.result()
                except Exception as exc:  # noqa: BLE001 - surface vendor/DDS failures
                    last_error = exc
                    continue
                return jsonable(result.response)
            raise TimeoutError(
                f"service /aimdk_5Fmsgs/srv/SetMcInputSource timed out after {attempts} attempts "
                f"(action={action}, name={name!r}): {last_error}"
            )
        finally:
            if executor is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if rclpy.ok(context=ctx):
                try:
                    rclpy.shutdown(context=ctx)
                except Exception:
                    pass

    @staticmethod
    def _input_source_accepted(response):
        try:
            return int(response["header"]["code"]) == 0
        except (KeyError, TypeError, ValueError):
            # The offline test doubles and some older SDK responses do not expose a code.
            return True

    def _register_input_source(self):
        try:
            result = self._set_input_source(1001)  # INPUTACTION_ADD
        except TimeoutError:
            # A lost response can leave the vendor registry with a half-created
            # custom source.  DELETE is idempotent for this source and is known
            # to respond even when a subsequent ADD hangs; clean it and retry.
            try:
                self._set_input_source(1003)  # INPUTACTION_DELETE
            except Exception:
                pass
            result = self._set_input_source(1001)
        restored_entry = False
        if not self._input_source_accepted(result):
            # The MC input-source registry outlives this container.  Reconfigure
            # our own stale entry after a driver restart, but never touch vendor
            # sources because _source_name() rejects their names.
            add_result = result
            result = self._set_input_source(1002)  # INPUTACTION_MODIFY
            if not self._input_source_accepted(result):
                raise RuntimeError(
                    "locomotion: input source registration rejected: "
                    f"add={add_result}, modify={result}"
                )
            restored_entry = True
        self._registered = True
        # On this firmware MODIFY keeps the stale custom entry usable.  ENABLE
        # is an optional extra round-trip and has been observed to hang after a
        # previous DELETE timeout (the common turn-after-straight case).  Keep
        # it opt-in for vendor images that explicitly require it.
        if restored_entry and bool(self._plugin_cfg().get("enable_after_modify", False)):
            # MODIFY changes configuration but does not promise to reactivate a
            # source left behind by a previous driver instance.
            result = self._set_input_source(2001)  # INPUTACTION_ENABLE
            if not self._input_source_accepted(result):
                raise RuntimeError(f"locomotion: input source enable rejected: {result}")
        return result

    def _release_input_source(self):
        if not self._registered:
            return
        try:
            self._set_input_source(1003)  # INPUTACTION_DELETE
        finally:
            # A failed delete must not stop the next action from re-registering
            # the source; ADD will reject stale entries and then repair them via
            # MODIFY in _register_input_source().
            self._registered = False

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {
                "state": "ready",
                "registered": self._registered,
                "velocity_command_scale": self._configured_velocity_scale(),
                "adaptive_velocity_scale": self._adaptive_scale_enabled(),
                "adaptive_scale": self._adaptive_scale,
            }
        if action == "cancel":
            self._cancel_motion("cancel_requested", send_stop=True)
            self._release_input_source()
            return {"state": "cancelled", "topic": "/aima/mc/locomotion/velocity"}
        if action != "move":
            # Any other/unrecognized action used to fall through to the block below, which
            # silently auto-registers this driver as an input source and publishes a (default
            # zero) velocity command -- so a stray health-check probe with an unknown action
            # name could trigger a real actuator side effect. Refuse instead.
            raise ValueError(f"locomotion: unknown action {action!r}")
        if "duration" not in args:
            raise ValueError("locomotion: duration is required (-1 or between 0.1 and 60 seconds)")
        duration = float(args["duration"])
        if duration != -1 and not 0.1 <= duration <= 60.0:
            raise ValueError("locomotion: duration must be -1 or between 0.1 and 60 seconds")
        self._cancel_motion("superseded")
        if not self._registered:
            self._register_input_source()
        forward = float(args.get("forward", 0.2))
        lateral = float(args.get("lateral", 0.0))
        angular_deg = float(args.get("angular", 0.0))
        angular = math.radians(angular_deg)
        # Reset adaptive scale each move so a stale 0.5 from a previous run cannot
        # permanently starve a correctly tracking firmware after restart.
        self._adaptive_scale = 1.0
        self._last_measured = None
        start_pose = self._leg_odom_snapshot()
        pub_f, pub_l, pub_a, scale = self._scaled_command(forward, lateral, angular)
        self._publish_velocity(pub_f, pub_l, pub_a)
        action_id = f"x2_locomotion_{uuid4().hex[:12]}"
        with self._lock:
            self._active_action_id = action_id
            self._command_forward = forward
            self._command_lateral = lateral
            self._command_angular = angular
            self._motion_started_mono = time.monotonic()
            self._motion_duration = None if duration == -1 else duration
        if duration != -1:
            self._schedule_stop(duration, action_id)
        self._start_velocity_stream(
            action_id, forward, lateral, angular,
            None if start_pose is None else {
                "x": start_pose["x"], "y": start_pose["y"], "yaw_deg": start_pose["yaw_deg"],
            },
            None if duration == -1 else duration,
        )
        return {
            "state": "accepted", "action_id": action_id, "duration": duration,
            "topic": "/aima/mc/locomotion/velocity",
            "angular_input_unit": "deg/s",
            "stop_mode": "continuous" if duration == -1 else "duration",
            "command": {
                "forward_m_s": forward,
                "lateral_m_s": lateral,
                "angular_deg_s": angular_deg,
                "angular_rad_s": angular,
            },
            "published": {
                "forward_m_s": pub_f,
                "lateral_m_s": pub_l,
                "angular_rad_s": pub_a,
            },
            "velocity_command_scale": scale,
            "expected_distance_m": None if duration == -1 else abs(math.hypot(forward, lateral) * duration),
            "expected_yaw_delta_deg": None if duration == -1 else angular_deg * duration,
        }


class PresetMotionPlugin:
    # area 只对单臂动作有意义（vendor 自带的 preset_motion_client.py 示例只为 area 提供
    # 1=left/2=right 两个选项，从未演示 head/waist/none 配合任何 motion）；其余动作（全身交互
    # 动作、point_head/shake_head 头部动作）不接受 area，硬编码发送 none(0)，不作为可调参数暴露，
    # 避免出现给 raise_hand 传 area=head 这种语义上不成立的组合。
    ACTIONS = {
        name: (
            (["area", "interrupt"] if name in PRESET_MOTIONS_REQUIRE_ARM_AREA else ["interrupt"]),
            f"播放预设动作 {name}"
            + ("（单臂动作，area 必须传 left_hand/right_hand，否则返回 code=1 失败）"
               if name in PRESET_MOTIONS_REQUIRE_ARM_AREA else "")
        )
        for name in PRESET_MOTIONS
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            self.ACTIONS,
            {
                "area": {"type": "string", "enum": ["left_hand", "right_hand"], "description": "受控手臂；仅 raise_hand/wave_hand/shake_hand 等单臂动作需要，必须传 left_hand 或 right_hand"},
                "interrupt": {"type": "boolean", "default": True, "description": "是否打断当前动作"},
            },
        )
        return tool("preset_motion", "actuator", "播放预设动作库（SetMcPresetMotion）。官方手册规定上肢预设动作只可在稳定站立模式使用。",
                    _with_actuation_contract(schema, ["leg", "waist", "arm_l", "arm_r", "head"]))

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
        if action not in PRESET_MOTIONS:
            raise ValueError(f"preset_motion: unknown action {action!r}")
        if action in PRESET_MOTIONS_REQUIRE_ARM_AREA:
            area = args.get("area")
            if area not in ("left_hand", "right_hand"):
                raise ValueError(f"preset_motion '{action}' 是单臂动作，area 必须是 left_hand 或 right_hand（当前: {area!r}）")
        else:
            area = "none"  # 非单臂动作不接受 area，忽略任何传入值
        from aimdk_msgs.srv import SetMcPresetMotion
        request = SetMcPresetMotion.Request()
        request.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        request.area.value = MC_CONTROL_AREAS[area]
        request.motion.value = PRESET_MOTIONS[action]
        request.interrupt = bool(args.get("interrupt", True))
        request.ani_path = ""
        request.play_timestamp = 0
        result = call_service(self.nodes.set_mc_preset_motion, request)
        return {"state": "accepted", "task": jsonable(result.response)}


class JointCommandPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = {
            "type": "object",
            "properties": {
                "area": {"type": "string", "enum": list(JOINT_AREAS), "description": "关节分组"},
                "joints": {
                    "type": "array",
                    "description": "关节指令列表",
                    "items": {
                        "type": "object",
                        "properties": {
                            "name": {"type": "string"},
                            "position": {"type": "number"},
                            "velocity": {"type": "number", "default": 0},
                            "effort": {"type": "number", "default": 0},
                            "stiffness": {"type": "number", "default": 0},
                            "damping": {"type": "number", "default": 0},
                        },
                        "required": ["name", "position"],
                    },
                },
            },
            "required": ["area", "joints"],
        }
        return tool("joint_command", "actuator", "按 leg/waist/arm/head 分组下发底层关节指令；没有限位、轨迹或碰撞保护。",
                    _with_actuation_contract(schema, ["leg", "waist", "arm_l", "arm_r", "head"]))

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
        area = args["area"]
        pub = self.nodes.joint_command_pubs[area]
        msg = self.nodes._JointCommandArray()
        msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        for item in args["joints"]:
            entry = self.nodes._JointCommand()
            entry.name = item["name"]
            entry.position = float(item["position"])
            entry.velocity = float(item.get("velocity", 0))
            entry.effort = float(item.get("effort", 0))
            entry.stiffness = float(item.get("stiffness", 0))
            entry.damping = float(item.get("damping", 0))
            msg.joints.append(entry)
        pub.publish(msg)
        return {"state": "published", "topic": f"/aima/hal/joint/{area}/command", "count": len(args["joints"])}


class HandCommandPlugin:
    ACTIONS = {
        "open": ([], "张开手掌（左右手可分别指定）"),
        "close": ([], "握拳（左右手可分别指定）"),
        "set_positions": (["left", "right"], "自定义左右手各手指的位置数组"),
        "get_state": ([], "查询手部关节最新状态快照"),
    }
    # Modeled after BrainCo Revo2's finger-joint naming as a generic reference (X2's own
    # HandType enum has no BrainCo entry — this is a naming-convention analogy only).
    FINGERS = ("thumb", "index", "middle", "ring", "little")
    OPEN_POSITION = 0.0
    CLOSE_POSITION = 1.0

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            self.ACTIONS,
            {
                "left": {"type": "array", "items": {"type": "number"}, "description": "左手各手指位置 [thumb, index, middle, ring, little]"},
                "right": {"type": "array", "items": {"type": "number"}, "description": "右手各手指位置 [thumb, index, middle, ring, little]"},
            },
        )
        return tool("hand_command", "actuator", "手部指令：张开/握拳/自定义手指位置（HandCommandArray）",
                    _with_actuation_contract(schema, ["arm_l", "arm_r"]))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        from aimdk_msgs.msg import HandCommand

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "get_state":
            return self.nodes.snapshot("hand_state")

        if action in ("open", "close"):
            value = self.OPEN_POSITION if action == "open" else self.CLOSE_POSITION
            left = [value] * len(self.FINGERS)
            right = [value] * len(self.FINGERS)
        elif action == "set_positions":
            left = args.get("left", [])
            right = args.get("right", [])
        else:
            raise ValueError(f"hand_command: unknown action {action!r}")

        msg = self.nodes._HandCommandArray()
        msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        for finger, position in zip(self.FINGERS, left):
            cmd = HandCommand()
            cmd.name = finger
            cmd.position = float(position)
            msg.left_hands.append(cmd)
        for finger, position in zip(self.FINGERS, right):
            cmd = HandCommand()
            cmd.name = finger
            cmd.position = float(position)
            msg.right_hands.append(cmd)
        self.nodes.hand_command_pub.publish(msg)
        return {"state": "published", "topic": "/aima/hal/joint/hand/command", "left_count": len(msg.left_hands), "right_count": len(msg.right_hands)}


class LinkcraftPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = {
            "type": "object",
            "properties": {
                "resource_key": {"type": "string", "description": "资源 key，来自 linkcraft_catalog 工具"},
                "resource_version": {"type": "string", "description": "资源版本"},
                "resource_type": {"type": "string", "enum": ["BODY_MONTION", "ARM_MONTION"], "description": "vendor 原始拼写（保留 MONTION 拼写以匹配 meta JSON 字段）"},
            },
            "required": ["resource_key", "resource_version", "resource_type"],
        }
        return tool("linkcraft", "actuator", "执行灵创动作资源（ExecuteActionResource）；仅执行 linkcraft_catalog 返回的资源。",
                    _with_actuation_contract(schema, ["leg", "waist", "arm_l", "arm_r", "head"]))

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
        from aimdk_msgs.srv import ExecuteActionResource
        request = ExecuteActionResource.Request()
        request.header = self.nodes.request_header()
        request.resource_key = args["resource_key"]
        request.resource_version = args["resource_version"]
        request.slaves = []
        request.meta = json.dumps({"resource_type": args["resource_type"]}, ensure_ascii=False)
        result = call_service(self.nodes.execute_action_resource, request)
        return {"state": "accepted", "response": jsonable(result.header)}


class PmuLedPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = {
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": list(LED_MODES), "default": "constant"},
                "r": {"type": "integer", "minimum": 0, "maximum": 255, "default": 0},
                "g": {"type": "integer", "minimum": 0, "maximum": 255, "default": 0},
                "b": {"type": "integer", "minimum": 0, "maximum": 255, "default": 0},
                "priority": {
                    "type": "integer", "minimum": 0, "maximum": 100, "default": 100,
                    "description": "灯带控制优先级 (0-100)；实测除 100（最高）外的任何值都被 PMU"
                    "固件拒绝（返回 status_code 4132），推测系统默认状态灯以更高优先级占用了"
                    "灯带，只有最高优先级请求才能覆盖。",
                },
                "reset_priority": {"type": "boolean", "default": False},
            },
        }
        return tool("pmu_led", "actuator", "设置 PMU 灯带模式与颜色（SetPmuLed）",
                    _with_actuation_contract(schema, "indicator"))

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
        from aimdk_msgs.srv import SetPmuLed
        request = SetPmuLed.Request()
        request.request = self.nodes.request_header()
        request.trace_id = ""
        request.led_strip_mode = LED_MODES[args.get("mode", "constant")]
        request.r = int(args.get("r", 0))
        request.g = int(args.get("g", 0))
        request.b = int(args.get("b", 0))
        request.priority = int(args.get("priority", 100))
        request.reset_priority = bool(args.get("reset_priority", False))
        result = call_service(self.nodes.set_pmu_led, request)
        return {"state": "accepted" if int(result.status_code) == 0 else "rejected", "status_code": result.status_code}


class TtsPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = {
            "type": "object",
            "properties": {
                "text": {"type": "string"},
                "priority": {"type": "string", "enum": list(TTS_PRIORITY_LEVELS), "default": "interaction"},
                "interrupt": {"type": "boolean", "default": False, "description": "是否打断同等优先级播报"},
            },
            "required": ["text"],
        }
        return tool("tts", "actuator", "文字转语音播报（PlayTts）；服务响应表示请求已接受，当前固件没有可关联的播放完成事件。",
                    _with_actuation_contract(schema, "mouth"))

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
        from aimdk_msgs.srv import PlayTts
        request = PlayTts.Request()
        request.header = self.nodes.request_header()
        request.tts_req.text = args["text"]
        request.tts_req.priority_level.value = TTS_PRIORITY_LEVELS[args.get("priority", "interaction")]
        request.tts_req.priority_weight = 0
        request.tts_req.domain = "phanthymotus"
        request.tts_req.trace_id = ""
        request.tts_req.is_interrupted = bool(args.get("interrupt", False))
        result = call_service(self.nodes.play_tts, request)
        return {"state": "accepted", "response": jsonable(result.tts_resp)}


class EmojiPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = {
            "type": "object",
            "properties": {
                "emotion": {"type": "string", "enum": list(EMOJI_IDS)},
                "loop": {"type": "boolean", "default": False},
                "priority": {"type": "integer", "default": 0},
            },
            "required": ["emotion"],
        }
        return tool("emoji", "actuator", "播放屏幕表情（PlayEmoji）",
                    _with_actuation_contract(schema, "face"))

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
        from aimdk_msgs.srv import PlayEmoji
        request = PlayEmoji.Request()
        request.header = self.nodes.request_header()
        request.emotion_id = EMOJI_IDS[args["emotion"]]
        request.mode = 2 if args.get("loop", False) else 1
        request.priority = int(args.get("priority", 0))
        result = call_service(self.nodes.play_emoji, request)
        return {
            "state": "accepted" if result.success else "rejected",
            "success": result.success,
            "message": result.message,
        }


class MicSourcePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            {"set": (["source"], "设置麦克风来源"), "get": ([], "查询当前麦克风来源")},
            {"source": {"type": "string", "enum": list(MIC_SOURCES)}},
        )
        return tool("mic_source", "actuator", "切换内置/外置麦克风来源（SetMicSourceRequest）",
                    _with_actuation_contract(schema, "microphone"))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        from aimdk_msgs.srv import GetMicSourceRequest, SetMicSourceRequest
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "get":
            request = GetMicSourceRequest.Request()
            request.header = self.nodes.request_header()
            result = call_service(self.nodes.get_mic_source, request)
            return {"mic_source": result.mic_source}
        if action != "set":
            raise ValueError(f"mic_source: unknown action {action!r}")
        request = SetMicSourceRequest.Request()
        request.header = self.nodes.request_header()
        request.mic_source = MIC_SOURCES[args["source"]]
        result = call_service(self.nodes.set_mic_source, request)
        return {"state": "accepted", "response": jsonable(result.header)}


class SlamControlPlugin:
    """Gated behind config.yaml's plugins.slam.enabled — SLAM control here is a plain
    `/integrated_command` String publish, not a service (confirmed via SDK's `slam.py` and
    `relocate.py`)."""

    ACTIONS = {
        "start_mapping": ([], "开始建图"),
        "stop_mapping": (["map_name"], "结束建图并保存"),
        "start_relocalization": (["map_id"], "开始重定位"),
        "set_relocalization_pose": (["x", "y"], "发布重定位初始位姿"),
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        schema = action_schema(
            self.ACTIONS,
            {
                "map_name": {"type": "string"},
                "map_id": {"type": "integer", "description": "目标地图 ID（由 APP 或地图查询获得）"},
                "x": {"type": "number"}, "y": {"type": "number"},
            },
        )
        return tool("slam_control", "actuator", "建图与重定位控制（/integrated_command 字符串指令）",
                    _with_actuation_contract(schema, ["base", "leg"]))

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
        if action == "set_relocalization_pose":
            pose = self.nodes._msg["Pose"]()
            pose.position.x = float(args["x"])
            pose.position.y = float(args["y"])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            self.nodes.relocalization_pose_pub.publish(pose)
            return {"state": "published", "topic": "/relocalization_pose"}
        if action not in ("start_mapping", "stop_mapping", "start_relocalization"):
            # Previously any unrecognized action fell through to publishing whatever
            # string_msg.data happened to default to (empty string) as a real command on
            # /integrated_command -- a stray "info" health-check probe would have actually
            # commanded the robot. Refuse instead of guessing.
            raise ValueError(f"slam_control: unknown action {action!r}")
        string_msg = self.nodes._msg["String"]()
        if action == "start_mapping":
            string_msg.data = "start_mapping"
        elif action == "stop_mapping":
            string_msg.data = f"stop_mapping:{args['map_name']}"
        elif action == "start_relocalization":
            string_msg.data = f"start_relocalization:{int(args['map_id'])}"
        self.nodes.integrated_command_pub.publish(string_msg)
        return {"state": "published", "topic": "/integrated_command", "command": string_msg.data}


class MapGetPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("map_get", "processor", "按名称查询已保存地图（GetStoredMapByName）", {
            "type": "object",
            "properties": {"map_name": {"type": "string"}},
            "required": ["map_name"],
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
        from aimdk_msgs.srv import GetStoredMapByName
        request = GetStoredMapByName.Request()
        request.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        request.header.frame_id = ""
        request.map_name = args["map_name"]
        result = call_service(self.nodes.get_stored_map, request)
        return {
            "code": result.code, "map_path": result.map_path, "map_id": result.map_id,
            "map_info": jsonable(result.map_info),
        }


def build_plugins(config, namespace, ros2):
    nodes = AimdkNodes(config, namespace, ros2)
    plugin_config = config.get("plugins", {})

    def enabled(name, default=True):
        return plugin_config.get(name, {}).get("enabled", default)

    plugins = []

    def add(name, plugin, default=True):
        if enabled(name, default):
            plugins.append(plugin)

    add("mc_state", McStatePlugin(nodes))
    add("joints", JointsPlugin(nodes))
    add("joint_state", JointStatePlugin(nodes))
    if "hand_state" in nodes.streams:
        add("hand_state", HandStatePlugin(nodes), default=False)
    add("imu", ImuPlugin(nodes))
    if "mic" in nodes.streams:
        add("mic", ReadOnlyStreamPlugin(nodes, "mic", "X2 内置麦克风 PCM-16kHz 单声道音频流"))
    if "leg_odometry" in nodes.streams:
        add("leg_odometry", ReadOnlyStreamPlugin(nodes, "leg_odometry", "X2 腿部/机体里程计（非 SLAM 定位）"))
    add("camera", CameraPlugin(nodes))
    add("head_touch", ReadOnlyStreamPlugin(nodes, "head_touch", "头部触摸状态（未触摸时 is_touched=false）"))
    add("pmu_state", ReadOnlyStreamPlugin(nodes, "pmu_state", "PMU 电压、电流、温度和电源状态"))
    if "lidar" in nodes.streams:
        add("lidar", LidarPlugin(nodes), default=False)
    if "slam_odom" in nodes.streams:
        add("slam", SlamPosePlugin(nodes), default=False)
    add("system_state", SystemStatePlugin(nodes))
    add("linkcraft_catalog", LinkcraftCatalogPlugin(nodes))
    add("model", ModelPlugin(nodes))
    add("mc_mode", McModePlugin(nodes))
    add("locomotion", LocomotionPlugin(nodes))
    add("preset_motion", PresetMotionPlugin(nodes))
    add("joint_command", JointCommandPlugin(nodes))
    add("hand_command", HandCommandPlugin(nodes), default=False)
    add("linkcraft", LinkcraftPlugin(nodes))
    add("pmu_led", PmuLedPlugin(nodes))
    add("tts", TtsPlugin(nodes))
    add("emoji", EmojiPlugin(nodes))
    add("mic_source", MicSourcePlugin(nodes))
    add("map_get", MapGetPlugin(nodes))
    if enabled("slam", default=False):
        plugins.append(SlamControlPlugin(nodes))
    return plugins
