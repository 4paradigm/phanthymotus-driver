"""ROS and MCP lifecycle for Bumi's two read-only transform processors."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import xml.etree.ElementTree as ET
import yaml

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from vision_tf import (OPTICAL, component, joint_positions, robot_description,
                       stamp, validate_edges, vision_transforms)


def _input_topics(args):
    topics = args.get("input_topics") or ([args["input_topic"]] if args.get("input_topic") else [])
    if not isinstance(topics, list) or len(topics) > 32:
        raise ValueError("Expected a list of up to 32 wired topics")
    from rclpy.validate_full_topic_name import validate_full_topic_name
    for topic in topics:
        validate_full_topic_name(topic)
    return tuple(dict.fromkeys(topics))


def _output(ns, key):
    suffix = hashlib.sha256(key.encode()).hexdigest()[:16]
    return f"/{ns}/vision_tf/i_{suffix}/transforms"


def _port(topic, desc):
    return {"topic": topic, "format": "data/json", "desc": desc}


def _message(edge):
    from geometry_msgs.msg import TransformStamped
    msg = TransformStamped()
    h, t = edge["header"], edge["transform"]
    msg.header.frame_id, msg.child_frame_id = h["frame_id"], edge["child_frame_id"]
    msg.header.stamp.sec, msg.header.stamp.nanosec = h["stamp"]["sec"], h["stamp"]["nanosec"]
    for key, value in t["translation"].items():
        setattr(msg.transform.translation, key, value)
    for key, value in t["rotation"].items():
        setattr(msg.transform.rotation, key, value)
    return msg


class _CardNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.input_errors = {}

    def consume(self, msg, handler, source):
        try:
            handler(json.loads(msg.data))
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self.input_errors[source] = str(exc)
            return
        self.input_errors.pop(source, None)


class VisionNode(_CardNode):
    def __init__(self, ns, key, topics, config):
        topic = _output(ns, key)
        super().__init__("bumi_vision_tf_" + topic.split("/")[-2])
        try:
            self.pub = self.create_publisher(String, topic, 5)
            source = component(config.get("object_source", "vision"))
            def convert(payload):
                result = vision_transforms(payload, ns, source, config.get("bumi_optical_frame", OPTICAL))
                self.pub.publish(String(data=json.dumps(result, allow_nan=False)))
            self.create_subscription(String, topics[0], lambda msg: self.consume(msg, convert, topics[0]), qos_profile_sensor_data)
        except Exception:
            self.destroy_node()
            raise


class TreeNode(_CardNode):
    def __init__(self, ns, topics, config):
        from ament_index_python.packages import get_package_prefix
        from sensor_msgs.msg import JointState
        from tf2_ros import TransformBroadcaster
        root = ET.parse(config.get("urdf_path") or Path(__file__).with_name("resource") / "bumi_model.urdf").getroot()
        xml, links, self.joints = robot_description(root, config.get("camera_mount"), config.get("hand_frames", []))
        self.frame_namespace = ns
        prefix = ns + "/"
        self.parents = {prefix + link for link in config.get("optical_frames", [OPTICAL]) if link in links}
        if topics and not self.parents:
            raise ValueError("Set camera_mount to measured camera extrinsics or supply an attached optical frame in the URDF before wiring vision")
        self.robot_frames = {prefix + name for name in links}
        self.owners = {}
        self.vision_subs = {}
        self.vision_lock = threading.RLock()
        self.process = None
        self.model_file = None
        super().__init__("bumi_tf_tree_" + ns)
        try:
            self.joint_pub = self.create_publisher(JointState, f"/{ns}/tf_tree/joint_states", 10)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(String, f"/{ns}/state/joints", lambda msg: self.consume(msg, self._joints, "joints"),
                                     qos_profile_sensor_data)
            self.bind_vision(topics)
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8", delete=False) as model:
                self.model_file = model.name
                yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": xml, "frame_prefix": prefix}}},
                               model, default_flow_style=False)
            executable = Path(get_package_prefix("robot_state_publisher")) / "lib/robot_state_publisher/robot_state_publisher"
            self.process = subprocess.Popen([str(executable), "--ros-args", "--params-file", self.model_file,
                "-r", "__node:=bumi_robot_state_" + ns,
                "-r", "joint_states:=" + self.joint_pub.topic_name,
                "-r", "robot_description:=/" + ns + "/tf_tree/robot_description"])
        except Exception:
            self.destroy_node()
            raise

    def bind_vision(self, topics):
        if topics and not self.parents:
            raise ValueError("Wire vision after configuring measured camera extrinsics")
        with self.vision_lock:
            added = {}
            try:
                for topic in topics:
                    if topic not in self.vision_subs:
                        added[topic] = self.create_subscription(String, topic,
                            lambda m, src=topic: self._on_vision(m, src), qos_profile_sensor_data)
            except Exception:
                for sub in added.values():
                    self.destroy_subscription(sub)
                raise
            for old in self.vision_subs.keys() - set(topics):
                self.destroy_subscription(self.vision_subs.pop(old))
                self.input_errors.pop(old, None)
                self.owners = {c: o for c, o in self.owners.items() if o[0] != old}
            self.vision_subs.update(added)

    def _on_vision(self, msg, source):
        with self.vision_lock:
            if source in self.vision_subs:
                self.consume(msg, lambda data: self._vision(data, source), source)

    def _joints(self, data):
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name, msg.position = joint_positions(data, self.joints)
        if "stamp" in data:
            when = stamp(data["stamp"])
            msg.header.stamp.sec, msg.header.stamp.nanosec = when["sec"], when["nanosec"]
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)

    def _vision(self, data, source):
        edges = validate_edges(data, self.parents, self.robot_frames, self.frame_namespace)
        pending = {e["child_frame_id"]: (source, e["header"]["frame_id"]) for e in edges}
        if any(self.owners.get(child, owner) != owner for child, owner in pending.items()):
            raise ValueError("Two sources or cameras claim the same TF child")
        if len(self.owners.keys() | pending.keys()) > 4096:
            raise ValueError("Too many distinct target frames")
        if edges:
            self.tf.sendTransform([_message(e) for e in edges])
            self.owners.update(pending)

    def destroy_node(self):
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        finally:
            if self.model_file is not None:
                Path(self.model_file).unlink(missing_ok=True)
            super().destroy_node()


class _TransformCard:
    def __init__(self, config, namespace, executor):
        self.tree = self.PREFIX == "tf_tree"
        self.config = config
        self.ns = component(namespace)
        self.executor = executor
        self.nodes = {}
        # Nodes keep their canvas IDs so info and stop still reach a renamed output.
        self.output_ids = {}
        self.lock = threading.RLock()

    def get_tool(self):
        desc = ("将 Bumi URDF、实时关节及左侧可选视觉 TF 合成 /tf 和 /tf_static。用 tf2 查询手与目标。"
                if self.tree else "把相机光学坐标系中的米制定位结果转换为 TFMessage JSON，保留图像时间及每个目标身份。")
        properties = {
            "action": {"type": "string", "enum": ["info", "start", "stop"]},
            "input_topic": {"type": "string"},
            "input_topics": {"type": "array", "items": {"type": "string"}},
        }
        return {"name": self.PREFIX, "type": "processor", "multiInstance": not self.tree, "description": desc,
                "inputSchema": {"type": "object", "required": ["action"], "properties": properties},
                "topic_in": [_port("", "TransformStamped JSON" if self.tree else "Metric camera poses")],
                "topic_out": [] if self.tree else [_port("", "TFMessage JSON")]}

    def start(self):
        return True  # Bundle boot registers the card. Canvas start activates subscriptions.

    def _remove(self, key):
        entry = self.nodes.pop(key, None)
        if entry:
            node = entry[1]
            self.executor.remove_node(node)
            node.destroy_node()

    def stop(self):
        with self.lock:
            for key in list(self.nodes):
                self._remove(key)

    def dispatch(self, action, args):
        topics = _input_topics(args)
        key = "tree" if self.tree else args.get("instance_id") or (topics[0] if topics else "default")
        if not isinstance(key, str):
            raise ValueError("Expected a string instance_id")
        with self.lock:
            output_id = self.output_ids.get(key, key)
            if action == "stop":
                if not self.tree and not topics and not args.get("instance_id"):
                    self.stop()
                else:
                    self._remove(key)
            elif action == "start":
                if not self.tree and len(topics) != 1:
                    raise ValueError("Wire exactly one metric vision source")
                if not self.tree and any(other != key and self.output_ids.get(other, other) == output_id
                                         for other in self.nodes):
                    raise ValueError("Another vision_tf card is already using this instance_id")
                entry = self.nodes.get(key)
                dead = entry and self.tree and entry[1].process.poll() is not None
                if self.tree and entry and not dead and entry[0] != topics:
                    entry[1].bind_vision(topics)
                    self.nodes[key] = (topics, entry[1])
                elif not entry or entry[0] != topics or dead:
                    if dead:
                        self._remove(key)
                    node = (TreeNode(self.ns, topics, self.config) if self.tree else
                            VisionNode(self.ns, output_id, topics, self.config))
                    try:
                        self.executor.add_node(node)
                    except Exception:
                        node.destroy_node()
                        raise
                    if entry and not dead:
                        self._remove(key)
                    self.nodes[key] = (topics, node)
            elif action != "info":
                raise ValueError("Unknown transform card action")
            entry = self.nodes.get(key)
            active_topics = entry[0] if entry else topics
            result = {"state": "running" if entry else "idle", "topic_in": [
                _port(t, "TFMessage JSON" if self.tree else "Metric camera poses") for t in active_topics or ("",)],
                "topic_out": [] if self.tree else [_port(_output(self.ns, output_id) if active_topics or args.get("instance_id") else "",
                                                         "TFMessage JSON")]}
            if entry and entry[1].input_errors:
                result["input_errors"] = entry[1].input_errors.copy()
            if self.tree and entry:
                result["camera_frames"] = sorted(entry[1].parents)
                if entry[1].process.poll() is not None:
                    result.update(state="error", message="robot_state_publisher exited; restart the card")
            return result


class VisionTfPlugin(_TransformCard):
    PREFIX = "vision_tf"

    def get_tool(self):
        tool = super().get_tool()
        tool["inputSchema"]["properties"]["action"]["enum"].append("config")
        tool["configSchema"] = {"type": "object", "properties": {
            "output_instance_id": {"type": "string", "title": "instance_id", "scope": "instance",
                                   "default": "", "description": "用于区分视觉 TF 输出话题。留空使用画布卡片 ID。"}
        }}
        return tool

    def dispatch(self, action, args):
        if action != "config":
            return super().dispatch(action, args)
        card_id = args.get("instance_id")
        value = args.get("output_instance_id")
        if not isinstance(card_id, str) or not card_id or not isinstance(value, str):
            raise ValueError("Expected a card instance_id and a string output_instance_id")
        value = value.strip()
        with self.lock:
            if card_id in self.nodes and (value or card_id) != self.output_ids.get(card_id, card_id):
                raise ValueError("Stop vision_tf before changing its instance_id")
            if value:
                self.output_ids[card_id] = value
            else:
                self.output_ids.pop(card_id, None)
        return {"output_instance_id": value}


class TfTreePlugin(_TransformCard):
    PREFIX = "tf_tree"
