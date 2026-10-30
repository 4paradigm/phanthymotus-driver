from __future__ import annotations

import importlib.util
import json
import math
import shutil
import socket
import struct
import subprocess
import threading
import time
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from deep_robotics.lynx_s10.basic_server import (
    HEADER,
    MAX_ASDU,
    SYNC,
    BasicServerClient,
    StreamDecoder,
    decode_frame,
    encode_frame,
)


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics" / "lynx_s10"


def _load_device(monkeypatch):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("lynx_s10_device_for_test", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeNative:
    def __init__(self, config):
        self.config = config
        self.calls = []
        self.reports = {}

    def snapshot(self):
        return {"reports": self.reports, "connected": True, "last_error": None}

    def request(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return {"state": "sent"}

    def send(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return {"state": "sent"}

    def start(self):
        pass


def test_pro_config_registers_navigation_and_matches_marketplace_cards(monkeypatch):
    device = _load_device(monkeypatch)
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    driver = yaml.safe_load((DRIVER / "driver.yaml").read_text())
    monkeypatch.setattr(device, "BasicServerClient", FakeNative)

    plugins = device.build_plugins(config)
    card_names = {card["name"] for card in driver["cards"]}

    assert config["model_variant"] == "pro"
    assert "navigation" in card_names
    assert "light" in card_names
    assert "loco" in card_names and "motion" not in card_names
    assert {"imu", "loco_state", "joints", "joint_state", "battery"} <= card_names
    assert "errors" not in card_names
    assert {"camera_0", "camera_1", "camera_2", "camera_3"} <= card_names
    assert "device" not in card_names
    assert not card_names.intersection({"motion_info", "native_basic_status", "native_device_status", "hard_estop", "charge_status"})
    assert any(plugin.__class__.__name__ == "S10NavigationPlugin" for plugin in plugins)
    assert config["mcp_port"] == driver["port"] == 15724
    assert config["mcp_port"] != yaml.safe_load((ROOT / "agibot/AimDK_X2/driver.yaml").read_text())["port"]
    dockerfile = (DRIVER / "Dockerfile").read_text()
    assert "15724" in dockerfile
    assert "COPY --from=drdds-builder /lynx_s10_ws/install/" in dockerfile
    assert "rm -rf build log src" in dockerfile


def test_standard_state_cards_read_native_reports(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    native.reports = {
        "1048577:15728640": {"PatrolDevice": {"Items": {
            "MotionStatus": {"AccX": 1, "Roll": 2},
            "MotorStatus": {"Joint": [{"q": 0.25}], "Motor": [{"q": 0.25, "dq": 0.1}]},
        }}},
        "1048578:15728640": {"PatrolDevice": {"Items": {"BatteryList": [{"SOC": 88}]}}},
        "1048676:15728640": {"PatrolDevice": {"Items": {"BasicStatus": {"Gait": 3, "HES": 0}}}},
    }
    plugin = device.S10StatePlugin(native)
    assert plugin.dispatch("imu", {"_tool_name": "imu"})["acceleration_m_s2"][0] == 1
    skeleton = plugin.dispatch("joints", {"_tool_name": "joints"})
    assert skeleton["format"] == "sensor/skeleton"
    assert skeleton["joint_count"] == 16
    assert len(skeleton["joints"]) == 16
    assert skeleton["joints"][0] == {
        "idx": 0, "name": "fl_hipx_joint",
        "q": pytest.approx(0.25 + math.radians(device.JOINT_OFFSETS_DEG[0])
                            - device.MODEL_REFERENCE_ANGLES[0]),
        "dq": None, "tau": None, "temperature": None,
    }
    assert plugin.dispatch("joint_state", {"_tool_name": "joint_state"})["Motor"][0]["dq"] == 0.1
    assert plugin.dispatch("loco_state", {"_tool_name": "loco_state"})["basic_state"] == {"Gait": 3, "HES": 0}
    assert plugin.dispatch("battery", {"_tool_name": "battery"}) == {"batteries": [{"SOC": 88}]}


def test_state_cards_expose_core_topics_when_ros_nodes_are_present(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    nodes = type("Nodes", (), {"topics": {
        "imu": {"topic": "/robot/lynx_s10/imu", "format": "data/json"},
        "joints": {"topic": "/robot/lynx_s10/joints", "format": "sensor/skeleton"},
        "joint_state": {"topic": "/robot/lynx_s10/joint_state", "format": "data/json"},
        "loco_state": {"topic": "/robot/lynx_s10/loco_state", "format": "data/json"},
        "battery": {"topic": "/robot/lynx_s10/battery", "format": "data/json"},
        "lidar": {"topic": "/robot/lynx_s10/lidar", "format": "sensor/pointcloud"},
    }, "stream_status": lambda _self: {"imu": {"received": 0, "published": 0}}})()
    plugin = device.S10StatePlugin(native, nodes)
    tools = {item["name"]: item for item in plugin.get_tools()}
    assert tools["joints"]["topic_out"] == [{"topic": "/robot/lynx_s10/joints", "format": "sensor/skeleton"}]
    assert plugin.dispatch("info", {"_tool_name": "imu"})["topic_out"][0]["topic"] == "/robot/lynx_s10/imu"
    assert plugin.dispatch("start", {"_tool_name": "imu"})["topic_out"][0]["topic"] == "/robot/lynx_s10/imu"
    assert plugin.dispatch("start", {"_tool_name": "joints"})["topic_out"][0]["format"] == "sensor/skeleton"
    assert plugin.dispatch("info", {"_tool_name": "lidar"})["topic_out"][0]["format"] == "sensor/pointcloud"


def test_robot_ros_samples_are_forwarded_to_agent_core(monkeypatch):
    device = _load_device(monkeypatch)

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class FakeNode:
        def __init__(self, name, context):
            self.name = name
            self.context = context
            self.publishers = {}
            self.subscriptions = {}

        def create_publisher(self, _msg_type, topic, _depth):
            publisher = FakePublisher()
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, _msg_type, topic, callback, _depth):
            self.subscriptions[topic] = callback

        def create_timer(self, *_args):
            return object()

        def get_topic_names_and_types(self):
            return [
                ("/s10/front/image", ["sensor_msgs/msg/Image"]),
                ("/s10/rear/image/compressed", ["sensor_msgs/msg/CompressedImage"]),
                ("/s10/left/image", ["sensor_msgs/msg/Image"]),
                ("/s10/right/image", ["sensor_msgs/msg/Image"]),
                ("/s10/depth", ["sensor_msgs/msg/PointCloud2"]),
            ]

        def destroy_node(self):
            pass

    class Executor:
        def add_node(self, _node):
            pass

    class String:
        def __init__(self):
            self.data = ""

    class UInt8MultiArray:
        def __init__(self):
            self.data = []

    class Imu: pass
    class PointCloud2: pass
    class Image: pass
    class CompressedImage:
        def __init__(self):
            self.header = None
            self.format = ""
            self.data = []
    class JointsData: pass
    class BatteryData: pass
    class MotionInfo: pass

    ros_node = ModuleType("rclpy.node")
    ros_node.Node = FakeNode
    qos_module = ModuleType("rclpy.qos")
    qos_module.qos_profile_sensor_data = object()
    class ReliabilityPolicy:
        RELIABLE = 1
        BEST_EFFORT = 2
    class DurabilityPolicy:
        VOLATILE = 2
    class HistoryPolicy:
        KEEP_LAST = 1
    class QoSProfile:
        def __init__(self, **kwargs): self.kwargs = kwargs
    qos_module.ReliabilityPolicy = ReliabilityPolicy
    qos_module.DurabilityPolicy = DurabilityPolicy
    qos_module.HistoryPolicy = HistoryPolicy
    qos_module.QoSProfile = QoSProfile
    sensor_msg = ModuleType("sensor_msgs.msg")
    sensor_msg.Imu, sensor_msg.PointCloud2 = Imu, PointCloud2
    sensor_msg.Image, sensor_msg.CompressedImage = Image, CompressedImage
    standard_msg = ModuleType("std_msgs.msg")
    standard_msg.String, standard_msg.UInt8MultiArray = String, UInt8MultiArray
    drdds_msg = ModuleType("drdds.msg")
    drdds_msg.JointsData, drdds_msg.BatteryData, drdds_msg.MotionInfo = JointsData, BatteryData, MotionInfo
    for module_name, module in {
        "rclpy.node": ros_node, "rclpy.qos": qos_module, "sensor_msgs.msg": sensor_msg,
        "std_msgs.msg": standard_msg, "drdds.msg": drdds_msg,
    }.items():
        monkeypatch.setitem(__import__("sys").modules, module_name, module)

    class FakeBridge:
        def __init__(self, *_args):
            self.publishers = {}
        def create_publisher(self, lane, topic, _msg_type):
            publisher = FakePublisher()
            self.publishers[lane] = publisher
            return publisher
        def stats(self, _lane):
            return {"enqueued": 1, "dropped": 0, "alive": True}
        def stop(self):
            pass
    monkeypatch.setattr(device, "CoreBridge", FakeBridge)

    ros2 = SimpleNamespace(
        ctx_robot=object(), ctx_core=object(),
        executor_robot=Executor(), executor_core=Executor(),
    )
    nodes = device.S10Nodes({"camera_topics": {"camera_0": "/CAMERA/RAW"}}, "robot", ros2)
    callbacks = nodes.robot.subscriptions

    imu = SimpleNamespace(linear_acceleration=SimpleNamespace(x=1.0, y=2.0, z=3.0))
    callbacks["/IMU"](imu)
    imu_payload = json.loads(nodes.publishers["imu"].messages[-1].data)
    assert imu_payload["linear_acceleration_z"] == 3.0

    joints = [SimpleNamespace(position=index, velocity=0.1, torque=0.2, motion_temp=30.0)
              for index in range(16)]
    callbacks["/JOINTS_DATA_10HZ"](SimpleNamespace(data=SimpleNamespace(joints_data=joints)))
    skeleton = json.loads(nodes.publishers["joints"].messages[-1].data)
    full_joint_state = json.loads(nodes.publishers["joint_state"].messages[-1].data)
    assert skeleton["format"] == "sensor/skeleton"
    assert skeleton["joint_count"] == 16
    assert skeleton["joints"][0]["name"] == "fl_hipx_joint"
    assert skeleton["joints"][15]["q"] == -15
    assert nodes.robot.subscriptions["/IMU"]
    assert next(iter(nodes.robot.subscriptions.values()))
    assert full_joint_state["fr_hipx_joint_q"] == 4

    callbacks["/BATTERY_DATA"](SimpleNamespace(data=[
        SimpleNamespace(voltage=0, current=0, cells=[0, 0]),
        SimpleNamespace(voltage=52000, current=0, cells=[3700, 3800],
                        serial=[0, 0, 0], battery_level=87),
    ]))
    battery = json.loads(nodes.publishers["battery"].messages[-1].data)
    assert battery == {
        "battery_1": {"voltage": 52000, "current": 0,
                      "cells": [3700, 3800], "battery_level": 87},
        "zero_placeholders": {
            "battery_0": {"all_zero": True},
            "battery_1_serial": {"all_zero": True, "length": 3},
        },
    }

    callbacks["/MOTION_INFO"](SimpleNamespace(data=SimpleNamespace(vel_x=0.3, vel_y=0.0)))
    loco_state = json.loads(nodes.publishers["loco_state"].messages[-1].data)
    assert loco_state["vel_x"] == 0.3

    camera_msg = SimpleNamespace(width=2, height=1, step=6, encoding="rgb8",
                                 data=bytes((255, 0, 0, 0, 255, 0)), header={"stamp": 1})
    callbacks["/CAMERA/RAW"](camera_msg)
    camera_output = nodes.camera_publishers["camera_0"].messages[-1]
    assert camera_output.format == "jpeg"
    assert bytes(camera_output.data).startswith(b"\xff\xd8")
    assert camera_output.header == camera_msg.header

    point_data = struct.pack("<ffffff", 1.0, 2.0, 3.0, math.nan, 5.0, 6.0)
    lidar = SimpleNamespace(
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
                for name, offset in (("x", 0), ("y", 4), ("z", 8))],
        is_bigendian=False, height=1, width=2, row_step=24, point_step=12, data=point_data,
    )
    callbacks["/LIDAR/POINTS"](lidar)
    encoded = bytes(nodes.lidar_publisher.messages[-1].data)
    point_step, point_count = struct.unpack_from("<II", encoded)
    assert (point_step, point_count) == (12, 1)
    assert nodes.lidar_publisher.messages[-1].data.typecode == "B"
    assert struct.unpack_from("<fff", encoded, 8) == (1.0, 2.0, 3.0)
    stream_status = nodes.stream_status()
    assert stream_status["imu"]["received"] == stream_status["imu"]["published"] == 1
    assert stream_status["joints"]["received"] == stream_status["joint_state"]["received"] == 1
    assert stream_status["lidar"]["received"] == stream_status["lidar"]["published"] == 1
    assert stream_status["camera_0"]["received"] == stream_status["camera_0"]["published"] == 1
    assert "/s10/front/image" in nodes.robot.subscriptions
    assert nodes._source_topics["camera_0"] == "/s10/front/image"
    assert nodes._source_topics["camera_1"] == "/s10/rear/image/compressed"
    assert nodes._source_topics["camera_2"] == "/s10/left/image"
    assert nodes._source_topics["camera_3"] == "/s10/right/image"
    assert "/s10/depth" in nodes.robot.subscriptions
    assert nodes._sensor_qos.kwargs == {
        "depth": 1, "history": 1, "reliability": 2, "durability": 2}
    nodes.close()


def test_lidar_packet_uniformly_caps_large_point_cloud(monkeypatch):
    device = _load_device(monkeypatch)
    width = 10000
    data = b"".join(struct.pack("<fff", float(index), 0.0, 0.0) for index in range(width))
    lidar = SimpleNamespace(
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
                for name, offset in (("x", 0), ("y", 4), ("z", 8))],
        is_bigendian=False, height=1, width=width, row_step=width * 12, point_step=12, data=data,
    )

    encoded = device.S10Nodes._pointcloud_packet(lidar)
    point_step, point_count = struct.unpack_from("<II", encoded)

    assert (point_step, point_count) == (12, 2000)
    assert struct.unpack_from("<f", encoded, 8)[0] == 0.0
    assert struct.unpack_from("<f", encoded, 8 + (point_count - 1) * point_step)[0] == 9999.0


def test_lidar_packet_supports_float64_big_endian_and_validates_layout(monkeypatch):
    device = _load_device(monkeypatch)
    point = struct.pack(">ddd", 1.25, -2.5, 3.75)
    fields = [SimpleNamespace(name=name, offset=offset, datatype=8, count=1)
              for name, offset in (("x", 0), ("y", 8), ("z", 16))]
    lidar = SimpleNamespace(fields=fields, is_bigendian=True, height=1, width=1,
                            row_step=24, point_step=24, data=point)
    encoded = device.S10Nodes._pointcloud_packet(lidar)
    assert struct.unpack_from("<II", encoded) == (12, 1)
    assert struct.unpack_from("<fff", encoded, 8) == (1.25, -2.5, 3.75)

    lidar.data = point[:-1]
    assert device.S10Nodes._pointcloud_packet(lidar) is None
    fields[0].datatype = 2
    lidar.data = point
    assert device.S10Nodes._pointcloud_packet(lidar) is None


def test_camera_ros_image_encodes_to_frontend_jpeg(monkeypatch):
    device = _load_device(monkeypatch)
    msg = SimpleNamespace(width=2, height=1, step=6, encoding="rgb8",
                          data=bytes((255, 0, 0, 0, 255, 0)))
    encoded = device.S10Nodes._jpeg_from_ros_image(msg)
    assert encoded.startswith(b"\xff\xd8") and encoded.endswith(b"\xff\xd9")


def test_model_resource_returns_official_urdf(monkeypatch):
    device = _load_device(monkeypatch)
    result = device.S10ModelPlugin().dispatch("model", {})
    root = ET.fromstring(result["urdf"])
    assert root.tag == "robot" and root.get("name") == "S10"
    assert all(child.tag in {"link", "joint", "material"} for child in root)
    assert root.find("joint[@name='fl_hipx_joint']") is not None
    checker = shutil.which("check_urdf")
    if checker:
        urdf_path = DRIVER / "resource" / "S10.urdf"
        subprocess.run([checker, str(urdf_path)], check=True, capture_output=True, text=True)


def test_main_builds_constrained_fastdds_profile_before_runtime(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("lynx_s10_main_for_test", DRIVER / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FakeSocket:
        def __enter__(self): return self
        def __exit__(self, *_args): pass
        def fileno(self): return 1

    monkeypatch.setattr(module.socket, "socket", lambda *_args: FakeSocket())

    def fake_ioctl(_fd, request_code, _request):
        value = "10.21.33.103" if request_code == 0x8915 else "255.255.255.0"
        return b"\0" * 20 + socket.inet_aton(value)

    monkeypatch.setattr(module.fcntl, "ioctl", fake_ioctl)
    monkeypatch.setenv("S10_FASTDDS_PROFILE", str(tmp_path / "fastdds.xml"))
    monkeypatch.setenv("NETWORK_INTERFACE", "eth0")
    monkeypatch.setenv("ROBOT_NETWORK_CIDR", "10.21.33.0/24")
    profile = module.configure_fastdds_profile({
        "ros": {"robot_interface": "eth0"},
        "basic_server": {"host": "10.21.33.103"},
    })
    root = ET.parse(profile).getroot()
    addresses = [node.text for node in root.findall(".//{*}interfaceWhiteList/{*}address")]
    assert addresses == ["127.0.0.1", "10.21.33.103"]
    assert root.find(".//{*}useBuiltinTransports").text == "false"
    assert module.os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] == str(profile)
    core_profile = ET.parse(module.os.environ["S10_CORE_FASTDDS_PROFILE"]).getroot()
    core_addresses = [node.text for node in core_profile.findall(".//{*}interfaceWhiteList/{*}address")]
    assert core_addresses == ["127.0.0.1"]


def test_main_fails_closed_for_missing_or_mismatched_robot_network(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location("lynx_s10_main_fail_closed_test", DRIVER / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FakeSocket:
        def __enter__(self): return self
        def __exit__(self, *_args): pass
        def fileno(self): return 1

    monkeypatch.setattr(module.socket, "socket", lambda *_args: FakeSocket())
    monkeypatch.setattr(
        module.fcntl, "ioctl",
        lambda _fd, request_code, _request: b"\0" * 20 + socket.inet_aton(
            "192.168.1.20" if request_code == 0x8915 else "255.255.255.0"))
    monkeypatch.setenv("S10_FASTDDS_PROFILE", str(tmp_path / "must-not-exist.xml"))
    monkeypatch.setenv("NETWORK_INTERFACE", "eth0")
    monkeypatch.delenv("ROBOT_NETWORK_CIDR", raising=False)
    config = {"basic_server": {"host": "10.21.33.103"}}
    with pytest.raises(ValueError, match="ROBOT_NETWORK_CIDR"):
        module.configure_fastdds_profile(config)

    monkeypatch.setenv("ROBOT_NETWORK_CIDR", "10.21.33.0/24")
    with pytest.raises(ValueError, match="not on configured robot subnet"):
        module.configure_fastdds_profile(config)
    assert not (tmp_path / "must-not-exist.xml").exists()

    monkeypatch.setattr(
        module.fcntl, "ioctl",
        lambda _fd, request_code, _request: b"\0" * 20 + socket.inet_aton(
            "10.21.33.2" if request_code == 0x8915 else "255.255.0.0"))
    monkeypatch.setenv("ROBOT_NETWORK_CIDR", "10.21.0.0/16")
    with pytest.raises(ValueError, match="/24 or narrower"):
        module.configure_fastdds_profile(config)

    # A narrow robot CIDR must contain the local interface address too, not
    # merely the robot address and the wider interface subnet.
    monkeypatch.setattr(
        module.fcntl, "ioctl",
        lambda _fd, request_code, _request: b"\0" * 20 + socket.inet_aton(
            "10.21.33.200" if request_code == 0x8915 else "255.255.255.0"))
    monkeypatch.setenv("ROBOT_NETWORK_CIDR", "10.21.33.0/25")
    monkeypatch.setenv("S10_FASTDDS_PROFILE", str(tmp_path / "outside-cidr.xml"))
    with pytest.raises(ValueError, match="address 10.21.33.200 is outside selected robot subnet"):
        module.configure_fastdds_profile(config)
    assert not (tmp_path / "outside-cidr.xml").exists()


def test_service_configures_verified_s10_robot_network_without_shell_exports():
    service_path = DRIVER / "deploy" / "service.yml"
    service = yaml.safe_load(service_path.read_text(encoding="utf-8"))["deep-robotics-lynx-s10"]
    environment = dict(item.split("=", 1) for item in service["environment"])
    assert environment["NETWORK_INTERFACE"] == "eth0"
    assert environment["ROBOT_NETWORK_CIDR"] == "10.21.33.0/24"
    config = yaml.safe_load((DRIVER / "config.yaml").read_text(encoding="utf-8"))
    assert config["basic_server"]["host"] == "10.21.33.103"


def test_core_domain_bridge_is_spawned_and_uses_bounded_latest_value_lanes():
    source = (DRIVER / "domain_bridge.py").read_text(encoding="utf-8")
    assert 'mp.get_context("spawn")' in source
    assert "Queue(maxsize=1)" in source
    assert 'os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile' in source
    assert 'os.environ["ROS_DOMAIN_ID"] = str(domain)' in source
    assert "interfaceWhiteList" not in source


def test_vendor_runtime_can_skip_core_context_for_process_isolated_bridge(monkeypatch):
    from common import vendor_runtime

    calls = []

    class Context:
        pass

    class Executor:
        def __init__(self, context):
            self.context = context
        def shutdown(self):
            pass

    class FakeRclpy:
        class executors:
            MultiThreadedExecutor = Executor
        @staticmethod
        def init(*, context, domain_id):
            calls.append((context, domain_id))
        @staticmethod
        def ok(*, context):
            return True
        @staticmethod
        def shutdown(*, context):
            calls.append(("shutdown", context))

    monkeypatch.setitem(__import__("sys").modules, "rclpy", FakeRclpy)
    monkeypatch.setitem(__import__("sys").modules, "rclpy.executors", FakeRclpy.executors)
    monkeypatch.setitem(__import__("sys").modules, "rclpy.context",
                        SimpleNamespace(Context=Context))
    ros = vendor_runtime.DualDomainROS2(0, None)
    assert len([call for call in calls if isinstance(call, tuple) and len(call) == 2
                and call[1] == 0]) == 1
    assert ros.ctx_core is None and ros.executor_core is None
    ros.shutdown()


def test_skeleton_maps_all_s10_motor_fields_and_urdf_order(monkeypatch):
    device = _load_device(monkeypatch)
    raw = [{"q": idx / 10, "dq": 1, "tau": 2, "temperature": [30]} for idx in range(16)]
    result = device._skeleton_payload({"MotorStatus": {"Joint": raw}, "MotionStatus": {"Roll": 0, "Pitch": 0, "Yaw": 0}})
    assert result["format"] == "sensor/skeleton"
    assert result["joint_count"] == 16
    assert [joint["name"] for joint in result["joints"]] == list(device.JOINT_NAMES)
    assert result["joints"][-1]["q"] == pytest.approx(
        -1.5 + math.radians(device.JOINT_OFFSETS_DEG[-1]) - device.MODEL_REFERENCE_ANGLES[-1])
    assert result["joints"][0]["temperature"] == [30]
    assert result["imu_quat"] == [1, 0, 0, 0]


def test_joints_are_reordered_by_data_id_and_name(monkeypatch):
    device = _load_device(monkeypatch)
    raw = [
        {"name": [70, 82, 72, 88], "data_id": 4, "position": 4.0},
        {"name": "FLHX", "data_id": 0, "position": 1.0},
    ]
    ordered = device._order_joints(raw)
    assert ordered[0]["position"] == 1.0
    assert ordered[4]["position"] == 4.0
    assert ordered[1] is None


def test_joints_ascii_motor_indices_follow_urdf_order(monkeypatch):
    device = _load_device(monkeypatch)
    raw = [{"name": list(str(index).encode("ascii")), "data_id": 14000 + index,
            "position": float(index)} for index in reversed(range(16))]
    ordered = device._order_joints(raw)
    assert [joint["position"] for joint in ordered] == list(map(float, range(16)))


def test_skeleton_applies_vendor_direction_offset_and_reference_pose(monkeypatch):
    device = _load_device(monkeypatch)
    raw = [{"name": list(str(index).encode("ascii")), "position": 0.0}
           for index in range(16)]
    skeleton = device._skeleton_payload({}, raw_joints=raw)
    for index, joint in enumerate(skeleton["joints"]):
        expected = (math.radians(device.JOINT_OFFSETS_DEG[index])
                    - device.MODEL_REFERENCE_ANGLES[index])
        assert joint["q"] == pytest.approx(expected)


def test_navigation_actions_use_s10_pro_native_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({"navigation_task_timeout": 45})
    plugin = device.S10NavigationPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))

    result = plugin.dispatch("goal", {"x": 1.2, "y": -0.4, "yaw_degrees": math.degrees(0.5), "confirm": True})
    assert result["state"] == "accepted"
    assert result["action_id"].startswith("s10_navigation_")
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and not terminals:
        time.sleep(0.01)
    assert terminals == [(result["action_id"], "completed", {"state": "completed", "response": {"state": "sent"}})]
    args, kwargs = native.calls[-1]
    assert args[:2] == (0x00100003, 0x00100002)
    assert args[2]["Item"][0]["PosX"] == 1.2
    assert args[2]["Item"][0]["AngleYaw"] == pytest.approx(0.5)
    assert kwargs["response_timeout"] == 45

    plugin.dispatch("localize", {"x": 1, "y": 2, "yaw_degrees": 90, "confirm": True})
    assert native.calls[-1][0][2]["Yaw"] == pytest.approx(math.pi / 2)

    plugin.dispatch("cancel", {})
    assert native.calls[-1][0][:2] == (0x00100003, 0x00100004)


def test_navigation_pose_reports_angles_in_degrees(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    native.request = lambda *_args, **_kwargs: {"PatrolDevice": {"Items": {
        "Roll": 0.0, "Pitch": math.pi / 4, "Yaw": -math.pi / 2, "PosX": 1,
    }}}

    result = device.S10NavigationPlugin(native).dispatch("pose", {})["PatrolDevice"]["Items"]

    assert result == {"PosX": 1, "RollDegrees": 0.0, "PitchDegrees": 45.0, "YawDegrees": -90.0}


def test_navigation_declares_completion_and_rejects_unconfirmed_goals(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({"navigation_task_timeout": 45})
    plugin = device.S10NavigationPlugin(native)
    schema = plugin.get_tool()["inputSchema"]
    assert schema["x-is-dangerous"] is True
    assert schema["x-completion"] == {"actions": ["goal"], "timeout": 55}
    assert "confirm" in schema["x-action-params"]["goal"]["params"]
    assert plugin.dispatch("goal", {"x": 1, "y": 2, "yaw_degrees": 0})["code"] == "INVALID_ARGUMENT"
    assert native.calls == []


def test_navigation_cancel_reports_one_terminal_callback(monkeypatch):
    device = _load_device(monkeypatch)
    started = threading.Event()
    release = threading.Event()

    class BlockingNative(FakeNative):
        def request(self, message_type, command, *args, **kwargs):
            self.calls.append(((message_type, command, *args), kwargs))
            if command == 0x00100002:
                started.set()
                assert release.wait(1)
                return {"state": "finished"}
            return {"state": "cancel_sent"}

    native = BlockingNative({"navigation_task_timeout": 2})
    plugin = device.S10NavigationPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    accepted = plugin.dispatch("goal", {"x": 1, "y": 2, "yaw_degrees": 0, "confirm": True})
    assert started.wait(1)
    conflict = plugin.dispatch("goal", {"x": 3, "y": 4, "yaw_degrees": 0, "confirm": True})
    assert conflict["code"] == "ACTION_IN_PROGRESS"
    assert plugin.dispatch("cancel", {}) == {"state": "cancel_sent"}
    release.set()
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and not terminals:
        time.sleep(0.01)
    assert terminals == [(accepted["action_id"], "cancelled", {
        "state": "cancelled", "reason": "navigation_cancelled",
    })]


def test_navigation_request_timeout_reports_acp_error(monkeypatch):
    device = _load_device(monkeypatch)

    class FailingNative(FakeNative):
        def request(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            raise TimeoutError("navigation task timed out")

    plugin = device.S10NavigationPlugin(FailingNative({"navigation_task_timeout": 1}))
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    accepted = plugin.dispatch("goal", {"x": 1, "y": 2, "yaw_degrees": 0, "confirm": True})
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and not terminals:
        time.sleep(0.01)
    assert terminals == [(accepted["action_id"], "error", {
        "state": "error", "message": "navigation task timed out",
    })]


def test_loco_card_uses_shared_action_names_and_native_velocity_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    plugin._acp_callback = lambda *_args: None
    definition = plugin.get_tool()
    assert set(definition["inputSchema"]["x-completion"]["actions"]) == {
        "move", "manual_control", "stand_up", "stand_down", "damp", "gait",
    }
    assert "enter_rl_control" not in definition["inputSchema"]["x-action-params"]
    assert definition["inputSchema"]["x-completion"]["timeout"] > 60
    assert "confirm" not in definition["inputSchema"]["properties"]
    assert all("confirm" not in item["params"] for item in definition["inputSchema"]["x-action-params"].values())
    assert "x-is-dangerous" not in definition["inputSchema"]
    assert "x-completion" not in definition
    assert definition["name"] == "loco"
    assert {"move", "stop_move", "stand_up", "stand_down", "damp"} <= set(definition["inputSchema"]["x-action-params"])
    try:
        result = plugin.dispatch("move", {"vx": 0.5, "vy": 0.1, "yaw_rate_deg_s": math.degrees(0.2), "duration": 1})
        assert result["state"] == "streaming"
        assert result["action_id"].startswith("s10_move_")
        assert native.calls[-1][0][1] == 0x00110002
        assert native.calls[-1][0][2] == pytest.approx({"X": 0.5, "Y": 0.1, "Z": 0.0,
                                                         "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.2})
        assert native.calls[-3][0][1:] == (0x00500002, {"Mode": 1})
        assert native.calls[-2][0][1:] == (0x00300002, {"GaitParam": 0x3002})
        result = plugin.dispatch("stop_move", {})
        assert result["state"] == "stopped"
        assert native.calls[-2][0][1] == 0x00100002
        assert native.calls[-1][0][1] == 0x00110002
    finally:
        plugin.stop()


def test_loco_stand_actions_return_action_ids_and_complete_from_state_reports(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    native.reports["basic"] = {"PatrolDevice": {"Items": {"BasicStatus": {"MotionState": 1}}}}
    stand_up = plugin.dispatch("stand_up", {})
    assert stand_up["state"] == "accepted"
    assert stand_up["action_id"].startswith("s10_stand_up_")
    time.sleep(0.05)
    assert terminals == []
    native.reports["basic"] = {"PatrolDevice": {"Items": {"BasicStatus": {"MotionState": 17}}}}
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and not terminals:
        time.sleep(0.01)
    assert terminals == [(stand_up["action_id"], "completed", {
        "state": "completed", "action": "stand_up", "observed": {"MotionState": 17},
    })]

    native.reports["basic"] = {"PatrolDevice": {"Items": {"BasicStatus": {"MotionState": 17}}}}
    stand_down = plugin.dispatch("stand_down", {})
    time.sleep(0.05)
    assert len(terminals) == 1
    native.reports["basic"] = {"PatrolDevice": {"Items": {"BasicStatus": {"MotionState": 4}}}}
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and len(terminals) < 2:
        time.sleep(0.01)
    assert stand_down["action_id"].startswith("s10_stand_down_")
    assert terminals[-1] == (stand_down["action_id"], "completed", {
        "state": "completed", "action": "stand_down", "observed": {"MotionState": 4},
    })
    assert [call[0][1:] for call in native.calls] == [
        (0x00200002, {"MotionParam": 1}),
        (0x00200002, {"MotionParam": 4}),
    ]
    assert all(call[1] == {} for call in native.calls)
    plugin.stop()


def test_loco_timed_move_expires_with_zero_velocity(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    plugin.start()
    try:
        accepted = plugin.dispatch("move", {"vx": 0.2, "duration": 0.02})
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if len(native.calls) >= 2 and native.calls[-1][0][2]["X"] == 0:
                break
            time.sleep(0.01)
        assert len(native.calls) >= 2
        assert native.calls[-1][0][1] == 0x00110002
        assert native.calls[-1][0][2] == {
            "X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0,
        }
        assert terminals == [(accepted["action_id"], "completed", {
            "state": "completed", "reason": "duration_elapsed",
        })]
    finally:
        plugin.stop()


def test_loco_stop_start_recreates_stream_worker(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    plugin.start()
    plugin.stop()
    plugin.start()
    try:
        accepted = plugin.dispatch("move", {"vx": 0.2, "duration": 0.02})
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not terminals:
            time.sleep(0.01)
        assert any(len(call[0]) > 2 and isinstance(call[0][2], dict)
                   and call[0][2].get("X") == 0.0 for call in native.calls)
        assert terminals == [(accepted["action_id"], "completed", {
            "state": "completed", "reason": "duration_elapsed",
        })]
    finally:
        plugin.stop()


def test_loco_explicit_stop_cancels_move_and_sends_zero(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    try:
        accepted = plugin.dispatch("move", {"vx": 0.2, "duration": -1})
        assert accepted["duration"] == 60
        assert plugin.dispatch("stop_move", {})["state"] == "stopped"
        assert terminals == [(accepted["action_id"], "cancelled", {
            "state": "cancelled", "reason": "stopped_or_replaced",
        })]
        assert native.calls[-1][0][2] == {
            "X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0,
        }
    finally:
        plugin.stop()


def test_axis_stream_has_action_id_and_acp_cancel(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    try:
        accepted = plugin.dispatch("manual_control", {"x": 0.2, "duration": 60})
        assert accepted["action_id"].startswith("s10_manual_control_")
        assert plugin.dispatch("stop_move", {})["state"] == "stopped"
        assert terminals == [(accepted["action_id"], "cancelled", {
            "state": "cancelled", "reason": "stopped_or_replaced",
        })]
    finally:
        plugin.stop()


def test_zero_duration_stream_completes_acp_immediately(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    result = plugin.dispatch("axis", {"x": 0, "duration": 0})
    assert result["state"] == "stopped"
    assert result["action_id"].startswith("s10_axis_")
    assert terminals == [(result["action_id"], "completed", {
        "state": "stopped", "reason": "zero_duration",
    })]


def test_loco_stream_send_error_reports_terminal_error(monkeypatch):
    device = _load_device(monkeypatch)

    class FailingNative(FakeNative):
        def send(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            if len(self.calls) == 4:
                raise OSError("socket closed")
            return {"state": "sent"}

    native = FailingNative({})
    plugin = device.S10MotionPlugin(native)
    terminals = []
    plugin._acp_callback = lambda action_id, status, result: terminals.append((action_id, status, result))
    plugin.start()
    try:
        accepted = plugin.dispatch("move", {"vx": 0.2, "duration": 1})
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not terminals:
            time.sleep(0.01)
        assert terminals[0][0:2] == (accepted["action_id"], "error")
        assert "socket closed" in terminals[0][2]["message"]
        assert native.calls[-1][0][2]["X"] == 0.0
    finally:
        plugin.stop()


def test_actuator_lifecycle_never_starts_charging_or_sends_idle_commands(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({"navigation_task_timeout": 45})
    plugins = [
        device.S10MotionPlugin(native),
        device.S10ChargePlugin(native),
        device.S10NavigationPlugin(native),
    ]

    for plugin in plugins:
        assert plugin.dispatch("start", {}) == {"state": "ready"}
        assert plugin.dispatch("stop", {}) == {"state": "idle"}
        assert plugin.dispatch("info", {}) == {"state": "ready"}
    assert native.calls == []

    charging = plugins[1]
    charging.dispatch("start_charging", {"confirm": True})
    assert native.calls[-1][0] == (0x00100004, 0x00100002, {"Charge": 1})
    charging.dispatch("stop_charging", {"confirm": True})
    assert native.calls[-1][0] == (0x00100004, 0x00100002, {"Charge": 0})


def test_state_card_stop_start_keeps_shared_tcp_client_alive(monkeypatch):
    device = _load_device(monkeypatch)

    class LifecycleNative(FakeNative):
        def __init__(self):
            super().__init__({})
            self.closed = False
            self.start_count = 0
            self.close_count = 0

        def start(self):
            self.start_count += 1

        def close(self):
            self.closed = True
            self.close_count += 1

    native = LifecycleNative()
    plugin = device.S10StatePlugin(native)
    assert plugin.dispatch("stop", {"_tool_name": "state"}) == {"state": "idle"}
    assert not native.closed
    assert plugin.dispatch("start", {"_tool_name": "state"}) == {"state": "running"}
    assert native.start_count == 1
    assert native.close_count == 0


def test_dangerous_s10_commands_require_literal_confirmation(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    cases = [
        (device.S10NavigationPlugin(native), "goal"),
        (device.S10NavigationPlugin(native), "localize"),
        (device.S10ChargePlugin(native), "start_charging"),
        (device.S10ChargePlugin(native), "stop_charging"),
    ]
    for plugin, action in cases:
        for confirm in (None, False, "true", 1):
            args = {} if confirm is None else {"confirm": confirm}
            result = plugin.dispatch(action, args)
            assert result["code"] == "INVALID_ARGUMENT", (action, confirm, result)
    assert native.calls == []


def test_control_schema_exposes_supported_navigation_and_light_options(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    nav_params = device.S10NavigationPlugin(native).get_tool()["inputSchema"]["x-action-params"]["goal"]["params"]
    localize_params = device.S10NavigationPlugin(native).get_tool()["inputSchema"]["x-action-params"]["localize"]["params"]
    light_tool = device.S10LightPlugin(native).get_tool()
    actions = light_tool["inputSchema"]["x-action-params"]
    light_params = actions["set"]["params"]

    assert set(nav_params) == {"x", "y", "z", "yaw_degrees", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous", "confirm"}
    assert set(localize_params) == {"x", "y", "z", "yaw_degrees", "confirm"}
    assert {"head_color", "tail_color", "head_effect", "tail_effect"} <= set(light_params)
    assert set(actions) == {"set"}
    assert {"head_cycle_seconds", "tail_cycle_seconds"} <= set(light_params)
    assert light_tool["name"] == "light"
    properties = light_tool["inputSchema"]["properties"]
    assert properties["head_effect"]["enum"] == list(device.LIGHT_EFFECTS)
    assert properties["head_cycle_seconds"]["x-show-when"] == {
        "head_effect": ["呼吸", "闪烁"]}

    charge_schema = device.S10ChargePlugin(native).get_tool()["inputSchema"]
    assert charge_schema["x-is-dangerous"] is True
    assert "confirm" in charge_schema["x-action-params"]["start_charging"]["params"]
    assert "confirm" in charge_schema["x-action-params"]["stop_charging"]["params"]


def test_light_uses_named_colors_and_effects_in_native_protocol(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    plugin = device.S10LightPlugin(native)
    plugin.dispatch("set", {"head_color": "绿色", "tail_color": "蓝色",
                                           "head_effect": "呼吸",
                                           "tail_effect": "闪烁",
                                           "head_cycle_seconds": 3, "tail_cycle_seconds": 2})

    assert native.calls[-1][0] == (0x00100005, 0x00200002, {
        "CustomMode": True,
        "Led": [
            {"Type": 4, "Color": [2], "Cycle": 3},
            {"Type": 5, "Color": [3], "Cycle": 2},
        ],
    })
    assert native.calls[-1][1] == {}


def test_light_uses_fire_and_forget_when_firmware_has_no_response(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    result = device.S10LightPlugin(native).dispatch("set", {})
    assert result["state"] == "sent"
    assert native.calls[-1][0][:2] == (0x00100005, 0x00200002)


def test_light_constant_effect_omits_cycle_from_native_payload(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    device.S10LightPlugin(native).dispatch("set", {
        "head_color": "白色", "tail_color": "蓝色",
        "head_effect": "常亮", "tail_effect": "填充流水",
    })
    leds = native.calls[-1][0][2]["Led"]
    assert "Cycle" not in leds[0]
    assert "Cycle" not in leds[1]


def test_light_single_action_can_mix_static_and_cyclic_side(monkeypatch):
    device = _load_device(monkeypatch)
    native = FakeNative({})
    device.S10LightPlugin(native).dispatch("set", {
        "head_effect": "闪烁", "head_cycle_seconds": 4,
        "tail_effect": "常亮",
    })
    leds = native.calls[-1][0][2]["Led"]
    assert leds[0]["Type"] == device.LIGHT_EFFECTS["闪烁"]
    assert leds[0]["Cycle"] == 4
    assert leds[1]["Type"] == device.LIGHT_EFFECTS["常亮"]
    assert "Cycle" not in leds[1]


def test_loco_completion_is_delivered_asynchronously(monkeypatch):
    device = _load_device(monkeypatch)
    completed = threading.Event()
    callback = {}

    def fake_callback(action_id, status, result, tool_name):
        callback.update(action_id=action_id, status=status, result=result, tool=tool_name)
        completed.set()

    monkeypatch.setattr(device, "_post_acp_callback", fake_callback)
    plugin = device.S10MotionPlugin(FakeNative({}))
    plugin._finish({"action_id": "s10-test"}, "completed", {"state": "completed"})
    assert completed.wait(1)
    assert callback["action_id"] == "s10-test"
    assert callback["status"] == "completed"
    assert callback["tool"] == "loco"


def test_malformed_final_fragment_is_discarded_without_killing_decoder():
    decoder = StreamDecoder()
    first = b"{" + b" " * (MAX_ASDU - 1)
    second = b"not-json"
    frame1 = HEADER.pack(SYNC, len(first), 9, 1, 0, 1) + first
    frame2 = HEADER.pack(SYNC, len(second), 9, 1, 1, 1) + second

    assert decoder.feed(frame1) == []
    assert decoder.feed(frame2) == []
    assert 9 not in decoder.fragments


def test_basic_server_uses_tcp_for_request_response(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.ready = threading.Event()
            self.response = None
            self.closed = False

        def settimeout(self, _timeout):
            pass

        def sendall(self, frame):
            message_id, request = decode_frame(frame)
            body = request["PatrolDevice"]
            self.response = encode_frame(message_id, body["Type"], body["Command"], {"ErrorCode": 0})
            self.ready.set()

        def recv(self, _size):
            assert self.ready.wait(1)
            if self.response is not None:
                response, self.response = self.response, None
                return response
            return b"" if self.closed else self.recv_wait()

        def recv_wait(self):
            while not self.closed:
                time.sleep(0.001)
            return b""

        def close(self):
            self.closed = True
            self.ready.set()

    fake_socket = FakeSocket()
    monkeypatch.setattr(
        "deep_robotics.lynx_s10.basic_server.socket.create_connection",
        lambda address, timeout: fake_socket,
    )
    client = BasicServerClient({"basic_server": {
        "host": "127.0.0.1", "tcp_port": 30003, "timeout": 1,
    }})
    try:
        response = client.request(0x00100064, 0x00000005)
        assert response["PatrolDevice"]["Items"] == {"ErrorCode": 0}
        assert client.snapshot()["transport"] == "TCP"
        assert client.snapshot()["reports"]["1048676:5"] == response
    finally:
        client.close()


def test_basic_server_matches_string_encoded_response_type_and_command(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.ready = threading.Event()
            self.response = None
            self.closed = False

        def settimeout(self, _timeout):
            pass

        def sendall(self, frame):
            _message_id, _request = decode_frame(frame)
            message_id = HEADER.unpack_from(frame)[2]
            payload = b'{"PatrolDevice":{"Type":"0X0010 0064","Command":"0X0000 0005","Items":{"ErrorCode":0}}}'
            self.response = HEADER.pack(SYNC, len(payload), message_id, 1, 0, 1) + payload
            self.ready.set()

        def recv(self, _size):
            assert self.ready.wait(1)
            if self.response is not None:
                response, self.response = self.response, None
                return response
            while not self.closed:
                time.sleep(0.001)
            return b""

        def close(self):
            self.closed = True
            self.ready.set()

    fake_socket = FakeSocket()
    monkeypatch.setattr(
        "deep_robotics.lynx_s10.basic_server.socket.create_connection",
        lambda address, timeout: fake_socket,
    )
    client = BasicServerClient({"basic_server": {
        "host": "127.0.0.1", "tcp_port": 30003, "timeout": 1,
    }})
    try:
        response = client.request(1048676, 5)
        assert response["PatrolDevice"]["Items"]["ErrorCode"] == 0
        assert client.snapshot()["last_error"] is None
        assert "1048676:5" in client.snapshot()["reports"]
    finally:
        client.close()
