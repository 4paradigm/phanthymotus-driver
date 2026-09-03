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
import threading
import time
from pathlib import Path

from common.vendor_runtime import action_schema, jsonable, tool


HAND_TYPES = {0: "none", 1: "nimble_hands", 2: "claw", 3: "leisai_nimble_hands", 255: "error"}

MC_ACTIONS = {
    "passive_default": 1, "soft_emergency_stop": 2, "damping_default": 3, "zero_torque_default": 4,
    "joint_default": 100, "joint_freeze": 101, "stand_default": 200, "stand_body_control": 201,
    "locomotion_default": 300, "run_default": 301, "locomotion_step": 302, "vr_remote_controller": 400,
    "sit_down_default": 2000, "crouch_down_default": 2002, "lie_down_default": 2004,
    "stand_up_default": 2005, "ascend_stairs": 2006, "descend_stairs": 2008,
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


class AimdkNodes:
    def __init__(self, config, namespace, ros2):
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2
        from std_msgs.msg import String
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from aimdk_msgs.msg import CommonRequest
        from aimdk_msgs.srv import (
            ExecuteActionResource, GetAllJointState, GetCurrentInputSource, GetHandType,
            GetMcAction, GetMicSourceRequest, GetRobotResources, GetStoredMapByName,
            GetSystemState, PlayEmoji, PlayTts, SetMcAction, SetMcInputSource,
            SetMcPresetMotion, SetMicSourceRequest, SetPmuLed,
        )
        from aimdk_msgs.msg import (
            HandCommand, HandCommandArray, HandStateArray, JointCommand, JointCommandArray,
            McLocomotionVelocity,
        )

        self._msg = {"CommonRequest": CommonRequest, "String": String, "Pose": Pose}
        self._HandCommand = HandCommand
        self._HandCommandArray = HandCommandArray
        self._JointCommand = JointCommand
        self._JointCommandArray = JointCommandArray
        self._McLocomotionVelocity = McLocomotionVelocity

        self.config = config
        self.end_effector = str(config.get("end_effector", "hand")).lower()
        self.namespace = namespace
        self.robot = Node("agibot_x2_driver_robot", context=ros2.ctx_robot)
        self.core = Node("agibot_x2_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)

        self.lock = threading.RLock()
        self.values = {}

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.streams = {}

        def mirror(key, msg_type, robot_topic, fmt, depth=10, qos=None):
            core_topic = f"/{namespace}/agibot_x2/{key}"
            as_json = fmt == "data/json"
            core_msg_type = String if as_json else msg_type
            pub = self.core.create_publisher(core_msg_type, core_topic, depth)
            self.robot.create_subscription(
                msg_type, robot_topic, self._callback(key, pub, as_json=as_json), qos or depth,
            )
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

        # Two physical IMUs (chest/torso) feed a single combined "imu" tool/topic — driver.yaml
        # lists one imu card, so both raw readings are merged into one data/json stream rather
        # than exposed as two separate tools.
        imu_topic = f"/{namespace}/agibot_x2/imu"
        imu_pub = self.core.create_publisher(String, imu_topic, 5)
        self.robot.create_subscription(Imu, "/aima/hal/imu/chest/state", self._imu_callback("chest", imu_pub), sensor_qos)
        self.robot.create_subscription(Imu, "/aima/hal/imu/torso/state", self._imu_callback("torso", imu_pub), sensor_qos)
        self.streams["imu"] = {"robot_topic": "/aima/hal/imu/{chest,torso}/state", "topic": imu_topic, "format": "data/json"}

        mirror("hand_state", HandStateArray, "/aima/hal/joint/hand/state", "data/json", qos=sensor_qos)
        # SDK's topics_and_services catalog documents rgbd_head_front/* as the front camera, but
        # on real hardware that topic has zero publishers -- this unit's camera service actually
        # publishes RGB under rgb_head_front_center/* instead (confirmed live via `ros2 topic
        # info`, 30Hz). No depth topic is published anywhere on this hardware at all, so
        # camera_depth stays wired to the documented (currently inactive) topic.
        mirror("camera_rgb", CompressedImage, "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed", "image/jpeg", qos=sensor_qos)
        mirror("camera_depth", Image, "/aima/hal/sensor/rgbd_head_front/depth_image", "image/depth-z16", qos=sensor_qos)
        mirror("lidar", PointCloud2, "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud", "sensor/pointcloud", qos=sensor_qos)
        mirror("slam_odom", Odometry, "/slam/lidar_odom", "data/json", qos=sensor_qos)

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

    def _callback(self, key, publisher, *, as_json=False):
        from std_msgs.msg import String

        def callback(msg):
            value = jsonable(msg)
            if as_json:
                output = String()
                output.data = json.dumps(value, ensure_ascii=False)
                publisher.publish(output)
            else:
                publisher.publish(msg)
            with self.lock:
                self.values[key] = value
        return callback

    def _imu_callback(self, source, publisher):
        from std_msgs.msg import String

        def callback(msg):
            with self.lock:
                combined = self.values.setdefault("imu", {})
                combined[source] = jsonable(msg)
                snapshot = dict(combined)
            output = String()
            output.data = json.dumps(snapshot, ensure_ascii=False)
            publisher.publish(output)
        return callback

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
    return tool(key, "sensor", description, topic_out=[{"topic": stream["topic"], "format": stream["format"]}])


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
        from aimdk_msgs.srv import GetMcAction
        request = GetMcAction.Request()
        request.request = self.nodes.request_header()
        result = call_service(self.nodes.get_mc_action, request)
        return jsonable(result.info)


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
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            from aimdk_msgs.srv import GetHandType
            request = GetHandType.Request()
            request.request = self.nodes.request_header()
            result = call_service(self.nodes.get_hand_type, request)
            return {
                "left_hand_type": HAND_TYPES.get(result.left_hands_type.value, "unknown"),
                "right_hand_type": HAND_TYPES.get(result.right_hands_type.value, "unknown"),
            }
        return {"state": "running", **self.nodes.streams["hand_state"]}


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
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running", **self.nodes.streams["imu"]}


class CameraPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tools(self):
        return [
            _stream_tool("camera_rgb", self.nodes.streams["camera_rgb"], "前置 RGBD 相机彩色画面（压缩 JPEG）"),
            _stream_tool("camera_depth", self.nodes.streams["camera_depth"], "前置 RGBD 相机深度画面"),
        ]

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running", **self.nodes.streams[name]}


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
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running", **self.nodes.streams["lidar"]}


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
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running", **self.nodes.streams["slam_odom"]}


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
        from aimdk_msgs.srv import GetSystemState
        request = GetSystemState.Request()
        request.header = self.nodes.request_header()
        result = call_service(self.nodes.get_system_state, request)
        return {"cur_state": result.cur_state, "status": jsonable(result.curr_status)}


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
        from aimdk_msgs.srv import GetRobotResources
        request = GetRobotResources.Request()
        request.header = self.nodes.request_header()
        result = call_service(self.nodes.get_robot_resources, request)
        return {"resources": jsonable(result.robot_resources)}


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
        return {"urdf": self.nodes.urdf_text(args.get("variant"))}


class McModePlugin:
    ACTIONS = {name: ([], f"切换到 {name} 模式") for name in MC_ACTIONS}

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("mc_mode", "actuator", "切换 MC 运控状态机模式（SetMcAction）", action_schema(self.ACTIONS, {}))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        from aimdk_msgs.srv import SetMcAction
        request = SetMcAction.Request()
        request.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        request.source = self.nodes.config.get("plugins", {}).get("mc_mode", {}).get("input_source_name", "phanthymotus")
        # vendor's own set_mc_action.py example never sets command.action.value at all — the
        # firmware looks the mode up by action_desc's exact string, matching McAction.msg's
        # UPPERCASE constant name (e.g. "STAND_DEFAULT"), not the integer value or our
        # lowercase snake_case key. Sending action_desc="stand_body_control" is what produced
        # the literal firmware error "can not find action: stand_body_control".
        request.command.action.value = MC_ACTIONS[action]
        request.command.action_desc = action.upper()
        result = call_service(self.nodes.set_mc_action, request)
        return jsonable(result.response)


class LocomotionPlugin:
    ACTIONS = {
        "register": ([], "以本驱动名义注册一个 MC 输入源（SetMcInputSource ADD）"),
        "set_velocity": (["forward", "lateral", "angular"], "发布行走速度指令"),
        "disable": ([], "禁用本驱动的输入源"),
    }

    def __init__(self, nodes):
        self.nodes = nodes
        self._registered = False

    def get_tool(self):
        return tool("locomotion", "actuator", "MC 行走速度控制：需先 register 输入源再 set_velocity；"
                    "另外机器人 FSM 必须已处于 locomotion_default/run_default 模式（用 mc_mode 切换），"
                    "站立(stand_default)等模式下 register 会成功但 set_velocity 不会驱动实际行走", action_schema(
            self.ACTIONS,
            {
                "forward": {"type": "number", "description": "前进速度 m/s，+前进/-后退"},
                "lateral": {"type": "number", "description": "侧移速度 m/s，+左移/-右移"},
                "angular": {"type": "number", "description": "转向角速度 rad/s，+左转/-右转"},
            },
        ))

    def start(self):
        pass

    def stop(self):
        self._set_input_source(2002)  # INPUTACTION_DISABLE

    def _source_name(self):
        return self.nodes.config.get("plugins", {}).get("locomotion", {}).get("input_source_name", "phanthymotus")

    def _set_input_source(self, mc_input_action):
        from aimdk_msgs.srv import SetMcInputSource
        plugin_cfg = self.nodes.config.get("plugins", {}).get("locomotion", {})
        request = SetMcInputSource.Request()
        request.request = self.nodes.request_header()
        request.action.value = mc_input_action
        request.input_source.name = self._source_name()
        request.input_source.priority = int(plugin_cfg.get("input_source_priority", 50))
        request.input_source.timeout = 1000
        result = call_service(self.nodes.set_mc_input_source, request)
        return jsonable(result.response)

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "register":
            result = self._set_input_source(1001)  # INPUTACTION_ADD
            self._registered = True
            return result
        if action == "disable":
            result = self._set_input_source(2002)  # INPUTACTION_DISABLE
            self._registered = False
            return result
        if not self._registered:
            self._set_input_source(1001)  # INPUTACTION_ADD
            self._registered = True
        msg = self.nodes._McLocomotionVelocity()
        msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        msg.source = self._source_name()
        msg.forward_velocity = float(args.get("forward", 0.0))
        msg.lateral_velocity = float(args.get("lateral", 0.0))
        msg.angular_velocity = float(args.get("angular", 0.0))
        self.nodes.locomotion_pub.publish(msg)
        return {"state": "published", "topic": "/aima/mc/locomotion/velocity"}


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
        return tool("preset_motion", "actuator", "播放预设动作库（SetMcPresetMotion）", action_schema(
            self.ACTIONS,
            {
                "area": {"type": "string", "enum": ["left_hand", "right_hand"], "description": "受控手臂；仅 raise_hand/wave_hand/shake_hand 等单臂动作需要，必须传 left_hand 或 right_hand"},
                "interrupt": {"type": "boolean", "default": True, "description": "是否打断当前动作"},
            },
        ))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
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
        return jsonable(result.response)


class JointCommandPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("joint_command", "actuator", "按 leg/waist/arm/head 分组下发关节位置/速度/力矩/刚度/阻尼指令", {
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
        return tool("hand_command", "actuator", "手部指令：张开/握拳/自定义手指位置（HandCommandArray）", action_schema(
            self.ACTIONS,
            {
                "left": {"type": "array", "items": {"type": "number"}, "description": "左手各手指位置 [thumb, index, middle, ring, little]"},
                "right": {"type": "array", "items": {"type": "number"}, "description": "右手各手指位置 [thumb, index, middle, ring, little]"},
            },
        ))

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
        if action == "get_state":
            return self.nodes.snapshot("hand_state")

        if action in ("open", "close"):
            value = self.OPEN_POSITION if action == "open" else self.CLOSE_POSITION
            left = [value] * len(self.FINGERS)
            right = [value] * len(self.FINGERS)
        else:
            left = args.get("left", [])
            right = args.get("right", [])

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
        return tool("linkcraft", "actuator", "执行灵创动作资源（ExecuteActionResource）", {
            "type": "object",
            "properties": {
                "resource_key": {"type": "string", "description": "资源 key，来自 linkcraft_catalog 工具"},
                "resource_version": {"type": "string", "description": "资源版本"},
                "resource_type": {"type": "string", "enum": ["BODY_MONTION", "ARM_MONTION"], "description": "vendor 原始拼写（保留 MONTION 拼写以匹配 meta JSON 字段）"},
            },
            "required": ["resource_key", "resource_version", "resource_type"],
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
        from aimdk_msgs.srv import ExecuteActionResource
        request = ExecuteActionResource.Request()
        request.header = self.nodes.request_header()
        request.resource_key = args["resource_key"]
        request.resource_version = args["resource_version"]
        request.slaves = []
        request.meta = json.dumps({"resource_type": args["resource_type"]}, ensure_ascii=False)
        result = call_service(self.nodes.execute_action_resource, request)
        return jsonable(result.header)


class PmuLedPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("pmu_led", "actuator", "设置 PMU 灯带模式与颜色（SetPmuLed）", {
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
        return {"status_code": result.status_code}


class TtsPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("tts", "actuator", "文字转语音播报（PlayTts）", {
            "type": "object",
            "properties": {
                "text": {"type": "string"},
                "priority": {"type": "string", "enum": list(TTS_PRIORITY_LEVELS), "default": "interaction"},
                "interrupt": {"type": "boolean", "default": False, "description": "是否打断同等优先级播报"},
            },
            "required": ["text"],
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
        return jsonable(result.tts_resp)


class EmojiPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("emoji", "actuator", "播放屏幕表情（PlayEmoji）", {
            "type": "object",
            "properties": {
                "emotion": {"type": "string", "enum": list(EMOJI_IDS)},
                "loop": {"type": "boolean", "default": False},
                "priority": {"type": "integer", "default": 0},
            },
            "required": ["emotion"],
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
        from aimdk_msgs.srv import PlayEmoji
        request = PlayEmoji.Request()
        request.header = self.nodes.request_header()
        request.emotion_id = EMOJI_IDS[args["emotion"]]
        request.mode = 2 if args.get("loop", False) else 1
        request.priority = int(args.get("priority", 0))
        result = call_service(self.nodes.play_emoji, request)
        return {"success": result.success, "message": result.message}


class MicSourcePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("mic_source", "actuator", "切换内置/外置麦克风来源（SetMicSourceRequest）", action_schema(
            {"set": (["source"], "设置麦克风来源"), "get": ([], "查询当前麦克风来源")},
            {"source": {"type": "string", "enum": list(MIC_SOURCES)}},
        ))

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
        if action == "get":
            request = GetMicSourceRequest.Request()
            request.header = self.nodes.request_header()
            result = call_service(self.nodes.get_mic_source, request)
            return {"mic_source": result.mic_source}
        request = SetMicSourceRequest.Request()
        request.header = self.nodes.request_header()
        request.mic_source = MIC_SOURCES[args["source"]]
        result = call_service(self.nodes.set_mic_source, request)
        return jsonable(result.header)


class SlamControlPlugin:
    """Gated behind config.yaml's plugins.slam.enabled — SLAM control here is a plain
    `/integrated_command` String publish, not a service (confirmed via SDK's `slam.py` and
    `relocate.py`)."""

    ACTIONS = {
        "start_mapping": ([], "开始建图"),
        "stop_mapping": (["map_name"], "结束建图并保存"),
        "start_relocalization": (["timestamp_ms"], "开始重定位"),
        "set_relocalization_pose": (["x", "y"], "发布重定位初始位姿"),
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("slam_control", "actuator", "建图与重定位控制（/integrated_command 字符串指令）", action_schema(
            self.ACTIONS,
            {
                "map_name": {"type": "string"},
                "timestamp_ms": {"type": "integer", "description": "留空则使用当前时间"},
                "x": {"type": "number"}, "y": {"type": "number"},
            },
        ))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "set_relocalization_pose":
            pose = self.nodes._msg["Pose"]()
            pose.position.x = float(args["x"])
            pose.position.y = float(args["y"])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            self.nodes.relocalization_pose_pub.publish(pose)
            return {"state": "published", "topic": "/relocalization_pose"}
        string_msg = self.nodes._msg["String"]()
        if action == "start_mapping":
            string_msg.data = "start_mapping"
        elif action == "stop_mapping":
            string_msg.data = f"stop_mapping:{args['map_name']}"
        elif action == "start_relocalization":
            timestamp_ms = args.get("timestamp_ms") or int(time.time() * 1000)
            string_msg.data = f"start_relocalization:{timestamp_ms}"
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

    plugins = [
        McStatePlugin(nodes), JointStatePlugin(nodes), HandStatePlugin(nodes),
        ImuPlugin(nodes), CameraPlugin(nodes), LidarPlugin(nodes), SlamPosePlugin(nodes),
        SystemStatePlugin(nodes), LinkcraftCatalogPlugin(nodes), ModelPlugin(nodes),
        McModePlugin(nodes), LocomotionPlugin(nodes), PresetMotionPlugin(nodes),
        JointCommandPlugin(nodes), HandCommandPlugin(nodes), LinkcraftPlugin(nodes),
        PmuLedPlugin(nodes), TtsPlugin(nodes), EmojiPlugin(nodes), MicSourcePlugin(nodes),
        MapGetPlugin(nodes),
    ]
    if enabled("slam", default=False):
        plugins.append(SlamControlPlugin(nodes))
    return plugins
