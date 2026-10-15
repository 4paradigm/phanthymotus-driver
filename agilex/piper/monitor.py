"""Read-only PiPER monitor. Hardware access is confined to an injected snapshot.

No SDK, control runtime, motion gate, camera, or CAN connection is imported here.
See MONITOR_INTEGRATION.md for the snapshot contract and bundle integration.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import threading
import time
from typing import TypedDict
import xml.etree.ElementTree as ET

from common.vendor_runtime import action_schema, tool


UNAVAILABLE = "不可用"
MAX_FEEDBACK_AGE = 0.5
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
RESOURCE_DIR = Path(__file__).with_name("resource")
SOURCE_COMMIT = "ac41fcbcdda598f01b51cf6175ed9a24d0dacadc"


def renderer_urdf(source: str) -> str:
    """Export the pinned motion chain for Core, stripping mesh/inertia data.

    URDF RPY is Rz(yaw) Ry(pitch) Rx(roll); Core uses Three.Euler('XYZ').
    Separate mixed origins into single-axis fixed joints so both agree.
    This visualization model is never a source of command limits or targets.
    """
    upstream = ET.fromstring(source)
    robot = ET.Element("robot", name="piper_monitor")
    for link in upstream.findall("link"):
        ET.SubElement(robot, "link", name=link.attrib["name"])
    names = []
    for source_joint in upstream.findall("joint"):
        joint = deepcopy(source_joint)
        name = joint.attrib["name"]
        if joint.attrib["type"] == "revolute":
            names.append(name)
        elif joint.attrib["type"] == "prismatic":
            # Core only animates revolute joints; opening remains a parameter.
            joint.set("type", "fixed")
            for child in list(joint):
                if child.tag in {"axis", "limit", "mimic", "dynamics"}:
                    joint.remove(child)
        origin = joint.find("origin")
        if origin is not None:
            rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
            if len(rpy) != 3 or len(xyz) != 3 or not all(math.isfinite(x) for x in rpy + xyz):
                raise ValueError(f"Invalid model origin: {name}")
            if sum(x != 0 for x in rpy) > 1:
                parent = joint.find("parent")
                parent_name = parent.attrib["link"]
                for label, rotation in (("yaw", (0, 0, rpy[2])), ("pitch", (0, rpy[1], 0))):
                    helper = f"monitor_{name}_{label}"
                    ET.SubElement(robot, "link", name=helper)
                    fixed = ET.SubElement(robot, "joint", name=helper, type="fixed")
                    ET.SubElement(fixed, "parent", link=parent_name)
                    ET.SubElement(fixed, "child", link=helper)
                    ET.SubElement(fixed, "origin", xyz=origin.get("xyz", "0 0 0") if label == "yaw" else "0 0 0",
                                  rpy=" ".join(str(x) for x in rotation))
                    parent_name = helper
                parent.set("link", parent_name)
                origin.set("xyz", "0 0 0")
                origin.set("rpy", f"{rpy[0]} 0 0")
        robot.append(joint)
    if tuple(names) != JOINT_NAMES:
        raise ValueError("PiPER model must contain joint1 through joint6 in order")
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="unicode")


def load_model() -> str:
    robot = ET.fromstring(renderer_urdf(
        (RESOURCE_DIR / "upstream" / "piper_description.urdf").read_text(encoding="utf-8")))
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    shapes = json.loads((RESOURCE_DIR / "monitor_shapes.json").read_text(encoding="utf-8"))
    for name, primitives in shapes["links"].items():
        if name not in links:
            raise ValueError(f"Unknown display-shell link: {name}")
        for index, primitive in enumerate(primitives):
            kind = "box" if "box" in primitive else "cylinder"
            size, xyz, rpy, rgba = (primitive[kind], primitive.get("xyz", [0, 0, 0]),
                                    primitive.get("rpy", [0, 0, 0]), primitive["rgba"])
            if (not _vector(size, 3 if kind == "box" else 2) or any(v <= 0 for v in size)
                    or not _vector(xyz, 3) or not _vector(rpy, 3)
                    or not _vector(rgba, 4) or any(v < 0 or v > 1 for v in rgba)):
                raise ValueError(f"Invalid display shell: {name}/{index}")
            visual = ET.SubElement(links[name], "visual", name=f"monitor_{name}_{index}")
            ET.SubElement(visual, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, kind, **({"size": " ".join(map(str, size))} if kind == "box" else
                                           {"radius": str(size[0]), "length": str(size[1])}))
            material = ET.SubElement(visual, "material", name=f"monitor_material_{name}_{index}")
            ET.SubElement(material, "color", rgba=" ".join(map(str, rgba)))
    _add_skeleton_shells(robot, shapes)
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="unicode")


def _add_skeleton_shells(robot, shapes):
    """Display-only fixed branches for Core's existing bone renderer.

    Core ignores URDF visual geometry but draws significant fixed segments.
    Outline the major housings/links with those segments, never inserting
    anything into the original six-axis motion chain. Small decorative strips
    are omitted. These branches are illustrative, not additional sensors.
    """
    for link, primitives in shapes["links"].items():
        for index, primitive in enumerate(primitives):
            size = primitive.get("box", primitive.get("cylinder"))
            if min(size) < .012:
                continue
            prefix = f"display_{link}_{index}"
            if link == "gripper_base" and "box" in primitive and size[0] < .02:
                prefix += "_hand"  # existing renderer's slimmer finger bones
            parent = link
            # Single-axis factors also preserve URDF RPY in the Core renderer.
            roll, pitch, yaw = primitive.get("rpy", [0, 0, 0])
            for axis, rpy in (("yaw", [0, 0, yaw]), ("pitch", [0, pitch, 0]), ("roll", [roll, 0, 0])):
                name = f"{prefix}_{axis}"
                _fixed_display_joint(robot, name, parent,
                                     primitive.get("xyz", [0, 0, 0]) if axis == "yaw" else [0, 0, 0], rpy)
                parent = name
            if "box" in primitive:
                vertices = [(x*size[0]/2, y*size[1]/2, z*size[2]/2)
                            for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                edges = [(a, b) for a in range(8) for b in range(a+1, 8)
                         if sum(vertices[a][d] != vertices[b][d] for d in range(3)) == 1]
            else:
                radius, length = size
                vertices = [(radius*math.cos(i*math.tau/6), radius*math.sin(i*math.tau/6), z)
                            for z in (-length/2, length/2) for i in range(6)]
                edges = [(i, (i+1) % 6) for i in range(6)]
                edges += [(6+i, 6+(i+1) % 6) for i in range(6)]
                edges += [(i, 6+i) for i in range(6)]
            for i, vertex in enumerate(vertices):
                _fixed_display_joint(robot, f"{prefix}_vertex{i}", parent, vertex)
            for i, (a, b) in enumerate(edges):
                _fixed_display_joint(robot, f"{prefix}_edge{i}", f"{prefix}_vertex{a}",
                                     [vertices[b][d]-vertices[a][d] for d in range(3)])


def _fixed_display_joint(robot, name, parent, xyz, rpy=(0, 0, 0)):
    ET.SubElement(robot, "link", name=name)
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=name)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))


class MonitorSnapshot(TypedDict, total=False):
    device_name: str
    connected: bool
    feedback_time: float  # Unix seconds of the oldest required CAN feedback.
    feedback_fresh: bool
    joints_deg: list[float]
    joint_limits_deg: list[list[float]]  # Already intersected with SDK limits.
    control_mode: str
    arm_status: str
    enabled: list[bool]
    gripper: dict
    motion_enabled: bool
    error_code: int
    motor_faults: list[str]
    busy: bool
    active_card: str | None
    completed_actions: dict
    # Names used by plan one's deployed read_monitor_snapshot(); accepted aliases.
    fresh: bool
    faults: dict
    last_actions: dict


def _finite(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def _vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) == length and all(_finite(x) for x in value)


def _text(value):
    return value if isinstance(value, str) and value else UNAVAILABLE


def _flag(value, yes, no):
    return yes if value is True else no if value is False else UNAVAILABLE


def _safe_json(value):
    """Keep arbitrary completion records JSON-safe without inventing feedback."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else UNAVAILABLE
    if isinstance(value, Mapping):
        return {str(k): _safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_json(v) for v in value]
    return UNAVAILABLE


def _timestamp(value):
    if not _finite(value):
        return UNAVAILABLE
    try:
        return datetime.fromtimestamp(value, timezone.utc).isoformat(timespec="milliseconds")
    except (ValueError, OverflowError, OSError):
        return UNAVAILABLE


def monitor_frame(snapshot: Mapping, *, now: float, device_name: str,
                  last_valid_time: float | None = None, error: str | None = None):
    """Return (optional real skeleton, Chinese parameters, last valid timestamp)."""
    # Plan one's deployed passive snapshot uses fresh/faults/last_actions.
    # Accept those names without calling any control/status method or changing
    # the provider's data. Explicit canonical fields always take precedence.
    snapshot = dict(snapshot)
    if "feedback_fresh" not in snapshot and "fresh" in snapshot:
        snapshot["feedback_fresh"] = snapshot["fresh"]
    faults = snapshot.get("faults")
    if isinstance(faults, Mapping):
        for key in ("arm_status", "error_code", "motor_faults"):
            if key not in snapshot and key in faults:
                snapshot[key] = faults[key]
    if "completed_actions" not in snapshot and "last_actions" in snapshot:
        snapshot["completed_actions"] = snapshot["last_actions"]
    angles = snapshot.get("joints_deg")
    valid_angles = _vector(angles, 6)
    stamp = snapshot.get("feedback_time")
    age = now - stamp if _finite(stamp) else None
    fresh = (error is None and snapshot.get("connected") is True
             and snapshot.get("feedback_fresh") is True and valid_angles
             and age is not None and 0 <= age <= MAX_FEEDBACK_AGE)
    if fresh:
        last_valid_time = stamp
    if error:
        status = "读取失败"
    elif snapshot.get("connected") is not True:
        status = "未连接" if snapshot.get("connected") is False else UNAVAILABLE
    elif not valid_angles or age is None or age < 0:
        status = "反馈无效"
    else:
        status = "新鲜" if fresh else "反馈过期"
    parameters = {
        "设备名称": _text(snapshot.get("device_name")) if snapshot.get("device_name") else device_name,
        "连接状态": _flag(snapshot.get("connected"), "已连接", "未连接"),
        "反馈状态": status,
        "反馈时间（UTC）": _timestamp(stamp),
        "最后有效时间（UTC）": _timestamp(last_valid_time),
        "反馈年龄（秒）": round(age, 3) if age is not None and age >= 0 else UNAVAILABLE,
        "控制模式": UNAVAILABLE,
        "设备状态": _text(snapshot.get("arm_status")),
        "动作许可": _flag(snapshot.get("motion_enabled"), "开启", "关闭"),
        "动作状态": _flag(snapshot.get("busy"), "执行中", "空闲"),
        "当前动作卡": _text(snapshot.get("active_card")),
        "故障码": snapshot.get("error_code") if type(snapshot.get("error_code")) is int else UNAVAILABLE,
        "电机故障": _safe_json(snapshot.get("motor_faults", UNAVAILABLE)),
        "最近动作结果": _safe_json(snapshot.get("completed_actions", UNAVAILABLE)),
        "读取错误": error or "无",
    }
    if snapshot.get("control_policy") == "direct_operator":
        parameters["动作许可"] = "直接控制（逐次校验）"
    mode = snapshot.get("control_mode")
    if isinstance(mode, str) and mode:
        parameters["控制模式"] = next((label for prefix, label in (
            ("STANDBY", "待机"), ("CAN_CTRL", "电脑控制（CAN）"),
            ("TEACHING_MODE", "示教")) if mode.startswith(prefix)), mode)
    enabled = snapshot.get("enabled")
    limits = snapshot.get("joint_limits_deg")
    for i in range(6):
        parameters[f"J{i + 1} 角度（°）"] = angles[i] if valid_angles else UNAVAILABLE
        parameters[f"J{i + 1} 电机"] = _flag(
            enabled[i] if isinstance(enabled, (list, tuple)) and len(enabled) == 6 else None, "已使能", "已失能")
        bound = limits[i] if isinstance(limits, (list, tuple)) and len(limits) == 6 else None
        parameters[f"J{i + 1} 有效限位（°）"] = list(bound) if _vector(bound, 2) and bound[0] < bound[1] else UNAVAILABLE
    grip = snapshot.get("gripper")
    grip = grip if isinstance(grip, Mapping) and grip.get("available") is not False else {}
    parameters["夹爪电机"] = _flag(grip.get("enabled"), "已使能", "已失能")
    width = grip.get("width_mm")
    parameters["夹爪开度（mm）"] = width if _finite(width) and width >= 0 else UNAVAILABLE
    skeleton = None
    if fresh:
        skeleton = {"timestamp_ms": int(stamp * 1000), "format": "sensor/skeleton", "position_unit": "rad",
                    "joints": [{"idx": i, "name": name, "q": math.radians(angles[i])}
                               for i, name in enumerate(JOINT_NAMES)]}
    return skeleton, parameters, last_valid_time


def english_parameters(parameters, snapshot):
    names = {'设备名称': 'Device', '连接状态': 'Connection', '反馈状态': 'Feedback',
             '反馈时间（UTC）': 'Feedback time (UTC)', '最后有效时间（UTC）': 'Last valid time (UTC)',
             '反馈年龄（秒）': 'Feedback age (s)', '控制模式': 'Control mode', '设备状态': 'Arm status',
             '动作许可': 'Control policy', '动作状态': 'Activity', '当前动作卡': 'Active tool',
             '故障码': 'Fault code', '电机故障': 'Motor faults', '最近动作结果': 'Last result',
             '读取错误': 'Read error', '夹爪电机': 'Gripper motor', '夹爪开度（mm）': 'Gripper opening (mm)'}
    for i in range(1, 7):
        names.update({f'J{i} 角度（°）': f'J{i} angle (deg)', f'J{i} 电机': f'J{i} motor',
                      f'J{i} 有效限位（°）': f'J{i} limits (deg)'})
    values = {'不可用': 'unavailable', '读取失败': 'read failed', '未连接': 'disconnected',
              '反馈无效': 'invalid', '新鲜': 'fresh', '反馈过期': 'stale', '已连接': 'connected',
              '开启': 'enabled', '关闭': 'disabled', '执行中': 'busy', '空闲': 'idle', '无': 'none',
              '待机': 'standby', '电脑控制（CAN）': 'CAN control', '示教': 'teaching',
              '已使能': 'enabled', '已失能': 'disabled', '直接控制（逐次校验）': 'checked direct control'}
    def convert(value):
        if isinstance(value, str):
            return values.get(value, value)
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [convert(item) for item in value]
        return value
    result = {names[key]: convert(value) for key, value in parameters.items()}
    if snapshot.get('control_policy') == 'automatic_preparation':
        result['Control policy'] = 'automatic preparation, checked per task'
    pose = snapshot.get('end_pose') or {}
    result['End position (mm)'] = pose.get('position_mm', 'unavailable')
    result['End orientation (deg)'] = pose.get('orientation_deg', 'unavailable')
    result['Task'] = snapshot.get('task', {'state': 'unavailable'})
    if 'tcp_verified' in snapshot:
        result['Gripper-tip calibrated'] = snapshot['tcp_verified']
        result['Gripper-tip pose (mm, deg)'] = snapshot.get('tip_pose_mm_deg') or 'unavailable'
        result['Return preset configured'] = snapshot.get('return_preset_configured', False)
        result['Capabilities'] = snapshot.get('capabilities', {})
        result['Reset position'] = snapshot.get('return_position') or 'unavailable'
    return result


class _RosPublisher:
    """Lazy ROS transport, using the shared Core domain/executor only."""
    def __init__(self, ros2, namespace, topic_out, period, callback):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._ros2 = ros2
        self._message_type = String
        self._node = Node("piper_monitor_" + namespace.replace("/", "_"), context=ros2.ctx_core)
        self._attached = False
        try:
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.VOLATILE)
            self._publishers = {t["format"]: self._node.create_publisher(String, t["topic"], qos) for t in topic_out}
            self._node.create_timer(period, callback)
            ros2.executor_core.add_node(self._node)
            self._attached = True
        except BaseException:
            self.close()
            raise

    def publish(self, fmt, data):
        message = self._message_type()
        message.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self._publishers[fmt].publish(message)

    def close(self):
        node, self._node = self._node, None
        if node is not None:
            try:
                if self._attached:
                    self._ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()


class PiperMonitorPlugin:
    """One monitor per existing PiPER runtime/MCP, no ownership of hardware."""
    def __init__(self, read_monitor_snapshot: Callable[[], MonitorSnapshot], *, namespace: str,
                 device_name: str, ros2=None, publish_hz: float = 10,
                 clock: Callable[[], float] = time.time, publisher_factory=None, language='zh',
                 publish_parameters: bool = True):
        if not callable(read_monitor_snapshot):
            raise TypeError("read_monitor_snapshot must be callable")
        if type(publish_parameters) is not bool:
            raise TypeError('publish_parameters must be a boolean')
        self._publish_parameters = publish_parameters
        namespace = namespace.strip("/")
        if not namespace or not all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", part) for part in namespace.split("/")):
            raise ValueError("namespace must contain valid, explicit ROS name segments")
        if not _finite(publish_hz) or not 1 <= publish_hz <= 30:
            raise ValueError("publish_hz must be finite and between 1 and 30")
        if not isinstance(device_name, str) or not device_name.strip():
            raise ValueError("device_name is required")
        self._read_snapshot = read_monitor_snapshot
        self._language = language
        self._namespace, self._device_name = namespace, device_name
        self._ros2, self._publish_hz, self._clock = ros2, float(publish_hz), clock
        self._publisher_factory = publisher_factory or _RosPublisher
        self._lock, self._sample_lock = threading.RLock(), threading.Lock()
        self._transport = None
        self._running = False
        self._generation = 0
        self._last_valid_time = None
        self._last_parameters = {}
        self._publish_error = None
        # Fail visibly on a broken/missing model rather than serving an empty
        # URDF that Core silently replaces with a humanoid fallback.
        self._model = load_model()

    def _topics(self):
        topics = [{"topic": f"/{self._namespace}/state/joints", "format": "sensor/skeleton"}]
        if self._publish_parameters:
            topics.append({"topic": f"/{self._namespace}/state/parameters", "format": "data/json"})
        return topics

    def get_tools(self):
        actions = {"start": ([], "开始只读监控"), "stop": ([], "停止监控发布"),
                   "read": ([], "读取当前姿态与参数"), "info": ([], "查看监控状态")}
        schema = action_schema(actions, {})
        schema.update({"additionalProperties": False, "x-manual-execution": True})
        schema["properties"]["action"].update(title="操作", default="read", oneOf=[
            {"const": name, "title": label} for name, (_, label) in actions.items()])
        monitor = tool("piper_monitor", "sensor", "实测六轴骨架与中文参数；只读监控。", schema, topic_out=self._topics())
        monitor["title"] = f"{self._device_name}｜姿态与参数"
        model = tool("model", "resource", "PiPER 六轴骨架模型，供监控渲染器自动读取。")
        model["title"] = "PiPER 骨架模型（后台资源）"
        return [monitor, model]

    def start(self):
        with self._lock:
            if self._running:
                return self.info()
            self._generation += 1
            generation = self._generation
            if self._ros2 is not None:
                self._transport = self._publisher_factory(self._ros2, self._namespace, self._topics(),
                                                          1 / self._publish_hz, lambda: self._tick(generation))
            self._running = True
            return self.info()

    def stop(self):
        with self._lock:
            self._running = False
            self._generation += 1
            self._publish_error = None
            transport, self._transport = self._transport, None
        if transport is not None:
            transport.close()
        return self.info()

    def info(self):
        with self._lock:
            return {"state": "error" if self._publish_error else "running" if self._running else "idle",
                    "publishing": self._transport is not None, "publish_hz": self._publish_hz,
                    "topic_out": self._topics(), "device_name": self._device_name,
                    "parameters": dict(self._last_parameters), "publish_error": self._publish_error}

    def _sample(self, generation=None):
        error = None
        try:
            snapshot = self._read_snapshot()
            if not isinstance(snapshot, Mapping):
                raise ValueError("snapshot provider must return a mapping")
        except Exception as exc:
            snapshot = {}
            error = str(exc).replace("\n", " ").replace("\r", " ")[:200] or type(exc).__name__
        with self._lock:
            skeleton, parameters, last_valid_time = monitor_frame(
                snapshot, now=self._clock(), device_name=self._device_name,
                last_valid_time=self._last_valid_time, error=error)
            if self._language == 'en':
                parameters = english_parameters(parameters, snapshot)
            if generation is None or generation == self._generation:
                self._last_parameters = parameters
                self._last_valid_time = last_valid_time
        return {"skeleton": skeleton, "parameters": parameters}

    def read(self):
        if not self._sample_lock.acquire(blocking=False):
            raise RuntimeError("监控采样正忙，请稍后重试")
        try:
            return self._sample()
        finally:
            self._sample_lock.release()

    def _tick(self, generation):
        with self._lock:
            if not self._running or generation != self._generation:
                return
        if not self._sample_lock.acquire(blocking=False):
            return
        try:
            frame = self._sample(generation)
            with self._lock:
                if not self._running or generation != self._generation or self._transport is None:
                    return
                try:
                    if self._publish_parameters:
                        self._transport.publish("data/json", frame["parameters"])
                    if frame["skeleton"] is not None:
                        self._transport.publish("sensor/skeleton", frame["skeleton"])
                    self._publish_error = None
                except Exception as exc:
                    self._publish_error = str(exc).replace("\n", " ").replace("\r", " ")[:200]
        finally:
            self._sample_lock.release()

    def dispatch(self, action, args):
        name = args.get("_tool_name", "model" if action == "model" else "piper_monitor")
        if name == "model":
            return {"urdf": self._model}
        if name != "piper_monitor" or set(args) - {"_tool_name", "action", "instance_id"}:
            raise ValueError("监控卡仅接受只读监控操作")
        if action in {"start", "stop", "info", "read"}:
            return getattr(self, action)()
        raise ValueError("监控卡不支持此操作")
