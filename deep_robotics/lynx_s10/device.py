#!/usr/bin/env python3
"""云深处山猫 S10 原生 basic_server 驱动。"""

from __future__ import annotations

import array
import threading
import time
import math
import json
import os
import ssl
import struct
import urllib.request
from uuid import uuid4
from pathlib import Path

from basic_server import BasicServerClient, _protocol_int
from common.vendor_runtime import action_schema, tool
from domain_bridge import CoreBridge
from media_workers import MediaWorkers, encode_pointcloud

CAMERA_LANES = ("camera_rear", "camera_left", "camera_front", "camera_right")
LIDAR_LANES = ("lidar_cloud", "lidar_map")
MEDIA_MAX_AGE_S = 0.75


def _mark_dangerous(result, actions):
    schema = result["inputSchema"]
    schema["x-is-dangerous"] = True
    schema["properties"]["confirm"] = {
        "type": "boolean", "description": "Required true to confirm this physical or safety-relevant action."}
    for action in actions:
        schema["x-action-params"][action]["params"].append("confirm")
    return result


def _confirmation_error(action, args):
    if args.get("confirm") is True:
        return None
    return {"ok": False, "code": "INVALID_ARGUMENT",
            "error": f"{action} requires boolean confirm=true"}


def _post_acp_callback(action_id, status, result, tool_name):
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
    if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(("https://localhost:", "https://127.0.0.1:")):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                          "tool": tool_name, "ts": time.time()}).encode()
    request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    last_error = None
    for attempt in range(1, 4):
        try:
            with urllib.request.urlopen(request, timeout=3, context=context) as response:
                http_status = response.status
                if not 200 <= http_status < 300:
                    raise RuntimeError(f"Agent Core returned HTTP {http_status}")
            print(f"[ACP] {tool_name} action_id={action_id} status={status} "
                  f"http={http_status} attempt={attempt}", flush=True)
            return
        except Exception as exc:
            last_error = exc
            if attempt < 3:
                time.sleep(0.2 * attempt)
    raise RuntimeError(f"ACP callback failed after 3 attempts: {last_error}")


def _notify_acp_async(action_id, status, result, tool_name):
    """Deliver completion without blocking the motion/navigation worker."""
    def worker():
        try:
            _post_acp_callback(action_id, status, result, tool_name)
        except Exception as exc:
            endpoint = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
            print(f"[ACP] callback failed action_id={action_id} status={status} "
                  f"endpoint={endpoint}/api/acp/complete error={str(exc)[:180]}", flush=True)
    threading.Thread(target=worker, daemon=True, name=f"s10-acp-{tool_name}").start()


class S10Nodes:
    """Forward robot-domain ROS samples and native TCP state into Agent Core DDS."""
    def __init__(self, config, namespace, ros2):
        import json
        import time
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from drdds.msg import BatteryData, JointsData, MotionInfo
        from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2
        from std_msgs.msg import String

        self.json = json
        self.robot = Node("lynx_s10_driver_robot", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self.robot)
        self.bridge = CoreBridge(os.environ.get("S10_CORE_FASTDDS_PROFILE", "/tmp/lynx-s10-core-fastdds.xml"),
                                 int(config.get("ros", {}).get("core_domain_id") or 42))
        prefix = f"/{namespace}/" if namespace else "/"
        base = prefix + "lynx_s10/"
        self.topics = {
            "imu": {"topic": base + "imu", "format": "data/json"},
            "joints": {"topic": base + "joints", "format": "sensor/skeleton"},
            "joint_state": {"topic": base + "joint_state", "format": "data/json"},
            "loco_state": {"topic": base + "loco_state", "format": "data/json"},
            "battery": {"topic": base + "battery", "format": "data/json"},
            "lidar_cloud": {"topic": base + "lidar_cloud", "format": "sensor/pointcloud"},
            "lidar_map": {"topic": base + "lidar_map", "format": "sensor/pointcloud"},
        }
        # The Pro camera service numbers its physical inputs tail, left, head,
        # right. Subscribe to USB names and semantic aliases alike.
        camera_topics = dict(config.get("camera_topics", {
            "camera_rear": ["/camera/rear/image_raw", "/camera/rear/image_raw/compressed",
                         "/camera/rear/color/image_raw", "/camera/rear/color/image_raw/compressed",
                         "/camera_back/image_raw", "/camera_0/image_raw",
                         "/usb_camera_1/image_raw", "/usb_camera_1/image_raw/compressed",
                         "/DEPTH_IMAGE"],
            "camera_left": ["/camera/left/image_raw", "/camera/left/image_raw/compressed",
                         "/camera/left/color/image_raw", "/camera/left/color/image_raw/compressed",
                         "/camera_left/image_raw", "/camera_1/image_raw",
                         "/usb_camera_2/image_raw", "/usb_camera_2/image_raw/compressed",
                         ],
            "camera_front": ["/camera/front/image_raw", "/camera/front/image_raw/compressed",
                         "/camera/front/color/image_raw", "/camera/front/color/image_raw/compressed",
                         "/camera_front/image_raw", "/camera_2/image_raw",
                         ],
            "camera_right": ["/camera/right/image_raw", "/camera/right/image_raw/compressed",
                         "/camera/right/color/image_raw", "/camera/right/color/image_raw/compressed",
                         "/camera_right/image_raw", "/camera_3/image_raw",
                         "/usb_camera_1/image_raw_2", "/usb_camera_1/image_raw_2/compressed",
                         ],
        }))
        for lane in CAMERA_LANES:
            camera_topics.setdefault(lane, [])
        self.topics.update({key: {"topic": base + key, "format": "image/jpeg"}
                            for key in camera_topics})
        self.publishers = {
            key: self.bridge.create_publisher(key, spec["topic"], String)
            for key, spec in self.topics.items() if key not in LIDAR_LANES
            and not key.startswith("camera_")
        }
        from std_msgs.msg import UInt8MultiArray
        self.lidar_publishers = {
            lane: self.bridge.create_publisher(lane, self.topics[lane]["topic"], UInt8MultiArray)
            for lane in LIDAR_LANES
        }
        self.lidar_publisher = self.lidar_publishers["lidar_cloud"]
        from sensor_msgs.msg import CompressedImage
        self.camera_publishers = {
            key: self.bridge.create_publisher(key, self.topics[key]["topic"], CompressedImage)
            for key in camera_topics
        }
        self.media_workers = None
        if config.get("media_workers", False):
            self.media_workers = MediaWorkers(self._handle_media_result)
            for key, configured_topics in camera_topics.items():
                source_topics = [configured_topics] if isinstance(configured_topics, str) else configured_topics
                if source_topics and all(topic.endswith("/compressed") for topic in source_topics):
                    self.media_workers.add_compressed_camera(key)
                else:
                    self.media_workers.add_camera(key)
            for lane in LIDAR_LANES:
                self.media_workers.add_lidar(lane)
        topics = config.get("topics", {})
        # Sensor streams may publish best-effort; the sensor-data profile is
        # compatible with both best-effort and reliable writers.
        stream_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self._sensor_qos = stream_qos
        # The node's default callback group is mutually exclusive. Give each
        # media lane its own group so slow point-cloud ingress cannot block
        # camera callbacks, while aliases within one camera remain serialized.
        self._media_callback_groups = {
            lane: MutuallyExclusiveCallbackGroup() for lane in (*CAMERA_LANES, *LIDAR_LANES)
        }
        self._stream_lock = threading.Lock()
        self._camera_source_lock = threading.Lock()
        self._stream_stats = {
            name: {"received": 0, "published": 0, "dropped": 0,
                   "last_received": None, "last_published": None,
                   "last_rate_log": time.monotonic(), "last_rate_count": 0,
                   "received_hz": 0.0, "published_hz": 0.0}
            for name in self.topics
        }
        self._motion_lock = threading.Lock()
        self.motion_sequence = 0
        self.motion_state = None
        self.gait_state = None
        self._source_topics = {
            "imu": topics.get("imu", "/IMU"),
            "joints": topics.get("joints", "/JOINTS_DATA_10HZ"),
            "joint_state": topics.get("joints", "/JOINTS_DATA_10HZ"),
            "battery": topics.get("battery", "/BATTERY_DATA"),
            "loco_state": topics.get("motion_info", "/MOTION_INFO"),
            "lidar_cloud": topics.get("lidar_cloud", topics.get(
                "lidar", ["/LIDAR/POINTS", "/DEPTH_POINTS", "/local_scans"])),
            "lidar_map": topics.get("lidar_map", "/ACCUMULATED_POINTS_MAP"),
        }
        self._source_topics.update({key: value for key, value in camera_topics.items()})
        self.robot.create_subscription(Imu, topics.get("imu", "/IMU"), self._forward_imu, stream_qos)
        self.robot.create_subscription(JointsData, topics.get("joints", "/JOINTS_DATA_10HZ"), self._forward_joints, stream_qos)
        self.robot.create_subscription(BatteryData, topics.get("battery", "/BATTERY_DATA"), self._forward_battery, stream_qos)
        self.robot.create_subscription(MotionInfo, topics.get("motion_info", "/MOTION_INFO"), self._forward_loco_state, stream_qos)
        lidar_topics = topics.get("lidar_cloud", topics.get("lidar", "/LIDAR/POINTS"))
        self._lidar_topics = [lidar_topics] if isinstance(lidar_topics, str) else list(lidar_topics)
        lidar_map_topic = topics.get("lidar_map", "/ACCUMULATED_POINTS_MAP")
        self._lidar_map_topic = str(lidar_map_topic)
        self._active_lidar_source = None
        self._lidar_source_last_seen = {}
        self._configured_camera_lanes = {
            topic: key for key, values in camera_topics.items()
            for topic in ([values] if isinstance(values, str) else values)
        }
        # Reserve lane assignments while scanning the ROS graph, but only let
        # a lane's first callback claim its active source. This avoids assigning
        # several newly discovered image topics to one lane before any frame arrives.
        self._camera_topic_lanes = {}
        self._configured_lidar_topics = set(self._lidar_topics)
        self._media_topic_subscriptions = set()
        self._media_topic_types = {}
        self._media_subscription_handles = {}
        self._discovered_media = set()
        self._media_subscriptions = []
        self._camera_lane_sources = {}
        self._scan_media_topics()
        # Keep configured aliases as a compatibility fallback when a ROS graph
        # implementation does not report publishers until after discovery.
        for topic, key in self._configured_camera_lanes.items():
            if topic in self._media_topic_subscriptions:
                continue
            msg_type = CompressedImage if topic.endswith("/compressed") else Image
            callback = lambda msg, camera=key, source=topic: self._forward_camera_alias(
                camera, source, msg)
            handle = self.robot.create_subscription(
                msg_type, topic, callback, self._sensor_qos,
                callback_group=self._media_callback_groups[key])
            self._media_subscriptions.append(handle)
            self._media_topic_subscriptions.add(topic)
            self._media_topic_types[topic] = msg_type
            self._media_subscription_handles[topic] = handle
            if not self._source_topics.get(key):
                self._source_topics[key] = topic
        for topic in self._configured_lidar_topics:
            if topic in self._media_topic_subscriptions:
                continue
            handle = self.robot.create_subscription(
                PointCloud2, topic,
                lambda msg, source=topic: self._forward_lidar(source, msg), self._sensor_qos,
                callback_group=self._media_callback_groups["lidar_cloud"])
            self._media_subscriptions.append(handle)
            self._media_topic_subscriptions.add(topic)
            self._media_topic_types[topic] = PointCloud2
            self._media_subscription_handles[topic] = handle
        if self._lidar_map_topic not in self._media_topic_subscriptions:
            handle = self.robot.create_subscription(
                PointCloud2, self._lidar_map_topic, self._forward_lidar_map, self._sensor_qos,
                callback_group=self._media_callback_groups["lidar_map"])
            self._media_subscriptions.append(handle)
            self._media_topic_subscriptions.add(self._lidar_map_topic)
            self._media_topic_types[self._lidar_map_topic] = PointCloud2
            self._media_subscription_handles[self._lidar_map_topic] = handle
        camera_subscriptions = sum(
            1 for topic in self._media_topic_subscriptions
            if topic in self._configured_camera_lanes or
            topic in self._camera_topic_lanes
        )
        lidar_subscriptions = sum(
            1 for topic in self._media_topic_subscriptions
            if topic not in self._configured_camera_lanes and
            topic not in self._camera_topic_lanes
        )
        print(f"[ros2] media subscriptions ready: camera_topics={camera_subscriptions} "
              f"lidar_topics={lidar_subscriptions} lanes=6", flush=True)
        if hasattr(self.robot, "create_timer"):
            self._media_discovery_timer = self.robot.create_timer(2.0, self._scan_media_topics)

    def _scan_media_topics(self):
        """Discover actual image and point-cloud endpoints as robot nodes start."""
        try:
            discovered = self.robot.get_topic_names_and_types()
        except (AttributeError, RuntimeError):
            return
        from sensor_msgs.msg import CompressedImage, Image, PointCloud2
        for topic, type_names in discovered:
            types = set(type_names)
            if "sensor_msgs/msg/PointCloud2" in types:
                # Configured raw feeds are subscribed during initialization.
                # Never discover extra PointCloud2 endpoints: the graph also
                # contains maps, paths and diagnostics with incompatible size.
                continue
            msg_type = (CompressedImage if "sensor_msgs/msg/CompressedImage" in types else
                        Image if "sensor_msgs/msg/Image" in types else None)
            if msg_type is None:
                continue
            # Do not require a publisher at discovery time. The S10 camera
            # process can register its topic before DDS endpoints become
            # visible; creating the subscription now lets late publishers
            # connect instead of leaving the card permanently without a lane.
            # Do not let unrelated depth imagery (for example /DEPTH_IMAGE)
            # claim a directional camera card before the four RGB sources.
            if topic not in self._configured_camera_lanes and self._camera_key_for_topic(topic) is None:
                continue
            key = self._assign_camera_lane(topic)
            if key is None:
                print(f"[ros2] ignoring extra camera topic (four lanes are occupied): {topic}", flush=True)
                continue
            if topic in self._media_topic_subscriptions:
                if self._media_topic_types.get(topic) is msg_type:
                    continue
                old_handle = self._media_subscription_handles.get(topic)
                if old_handle is not None and hasattr(self.robot, "destroy_subscription"):
                    self.robot.destroy_subscription(old_handle)
                self._media_subscriptions = [handle for handle in self._media_subscriptions
                                             if handle is not old_handle]
                self._media_topic_subscriptions.discard(topic)
            identity = (topic, key)
            if identity in self._discovered_media:
                continue
            self._discovered_media.add(identity)
            callback = lambda msg, camera=key, source=topic: self._forward_camera_alias(
                camera, source, msg)
            handle = self.robot.create_subscription(
                msg_type, topic, callback, self._sensor_qos,
                callback_group=self._media_callback_groups[key])
            self._media_subscriptions.append(handle)
            self._media_topic_subscriptions.add(topic)
            self._media_topic_types[topic] = msg_type
            self._media_subscription_handles[topic] = handle
            if not self._source_topics.get(key):
                self._source_topics[key] = topic
            print(f"[ros2] discovered {key} subscription: {topic} ({msg_type.__name__})", flush=True)

    def _forward_camera_alias(self, key, topic, msg):
        """Use the first active configured alias for each physical camera lane."""
        from sensor_msgs.msg import CompressedImage

        stale_handles = []
        with self._camera_source_lock:
            source = self._camera_lane_sources.get(key)
            if source is not None and source != topic:
                # The S10 node exposes raw and compressed aliases for the same
                # USB camera. Prefer compressed once it produces a frame and
                # remove the redundant raw subscription so it cannot consume
                # executor time or feed a stale competing stream.
                if not (topic.endswith("/compressed") and not source.endswith("/compressed")):
                    return
                old_handle = self._media_subscription_handles.get(source)
                if old_handle is not None:
                    stale_handles.append(old_handle)
                    if hasattr(self.robot, "destroy_subscription"):
                        self.robot.destroy_subscription(old_handle)
                    self._media_subscriptions = [handle for handle in self._media_subscriptions
                                                 if handle is not old_handle]
                    self._media_topic_subscriptions.discard(source)
                    self._media_subscription_handles.pop(source, None)
            self._camera_lane_sources[key] = topic
            self._camera_topic_lanes[topic] = key
            self._source_topics[key] = topic
        self._record_stream(key, "received")
        if isinstance(msg, CompressedImage):
            self._forward_compressed_camera(key, msg, already_received=True)
        else:
            self._forward_camera(key, msg, already_received=True)

    def _assign_camera_lane(self, topic):
        key = self._configured_camera_lanes.get(topic) or self._camera_topic_lanes.get(topic)
        if key is None:
            key = self._camera_key_for_topic(topic)
        # Unknown USB inputs are not safe to assign by discovery order: doing
        # so can expose the left camera as front/right. Only use explicitly
        # mapped USB topics; semantic camera names may still use the fallback.
        if key is None and topic.lower().startswith("/usb_camera_"):
            return None
        if key is None:
            reserved = set(self._camera_topic_lanes.values())
            key = next((lane for lane in CAMERA_LANES if lane not in reserved), None)
        if key is not None:
            self._camera_topic_lanes[topic] = key
        return key

    @staticmethod
    def _camera_key_for_topic(topic):
        name = topic.lower()
        import re
        # usb_cam_multi_node uses one base topic and numbered suffixes.  Do
        # this before the generic camera_N rule, which otherwise interprets
        # the service's ``camera_1`` prefix as the lane for every image.
        usb = re.search(r"/usb_camera_(\d+)/image_raw(?:_(\d+))?(?:/|$)", name)
        if usb:
            camera_number, suffix = usb.groups()
            if suffix is not None:
                return None
            return {"1": "camera_rear", "2": "camera_left",
                    "3": "camera_front", "4": "camera_right"}.get(camera_number)
        # Do not let the generic camera_N matcher reinterpret aliases such as
        # /usb_camera_2/image_raw as a physical front camera. Only the vendor
        # image_raw suffix mapping above is trusted.
        if name.startswith("/usb_camera_"):
            return None
        tokens = set(filter(None, re.split(r"[^a-z0-9]+", name)))
        numbered = re.search(r"(?:^|[/_])camera[_-]?(\d+)(?:$|[/_])", name)
        if numbered:
            index = int(numbered.group(1))
            if 0 <= index < 4:
                return CAMERA_LANES[index]
        for directions, key in ((("rear", "back", "backward"), "camera_rear"),
                                (("left",), "camera_left"),
                                (("front", "forward"), "camera_front"),
                                (("right",), "camera_right")):
            if tokens.intersection(directions):
                return key
        return None

    def _record_stream(self, key, stage):
        now = time.time()
        monotonic_now = time.monotonic()
        with self._stream_lock:
            stats = self._stream_stats[key]
            stats[stage] += 1
            if stage in ("received", "published"):
                stats[f"last_{stage}"] = now
                previous_count = stats.setdefault(f"_{stage}_rate_count", stats[stage])
                previous_time = stats.setdefault(f"_{stage}_rate_time", monotonic_now)
                interval = monotonic_now - previous_time
                if interval >= 2.0:
                    stats[f"{stage}_hz"] = round((stats[stage] - previous_count) / interval, 1)
                    stats[f"_{stage}_rate_count"] = stats[stage]
                    stats[f"_{stage}_rate_time"] = monotonic_now
            first = stats[stage] == 1
            report_rate = (key in {"camera_left", "camera_rear", *LIDAR_LANES} and
                           monotonic_now - stats.get("last_rate_log", monotonic_now) >= 10.0)
            if report_rate:
                stats["last_rate_log"] = monotonic_now
                received_hz = stats.get("received_hz", 0.0)
                published_hz = stats.get("published_hz", 0.0)
                counts = (stats["received"], stats["published"], stats["dropped"])
        if first and stage == "received":
            print(f"[ros2] first {key} sample received from {self._source_topics[key]}", flush=True)
        if first and stage == "published":
            print(f"[ros2] first {key} sample published to {self.topics[key]['topic']}", flush=True)
        if report_rate:
            print(f"[ros2] media rate lane={key} received_hz={received_hz:.1f} "
                  f"published_hz={published_hz:.1f} received={counts[0]} "
                  f"published={counts[1]} dropped={counts[2]}", flush=True)

    def stream_status(self):
        with self._stream_lock:
            result = {key: dict(value) for key, value in self._stream_stats.items()}
        for key, spec in self.topics.items():
            try:
                source = self._source_topics[key]
                if key == "lidar_cloud":
                    source = self._lidar_topics
                sources = [source] if isinstance(source, str) else list(source)
                result[key]["robot_publishers"] = sum(
                    self.robot.count_publishers(topic) for topic in sources)
            except (AttributeError, RuntimeError, TypeError, ValueError):
                result[key]["robot_publishers"] = None
            lane = self.bridge.stats(key)
            result[key]["bridge_enqueued"] = lane["enqueued"]
            result[key]["bridge_dropped"] = lane["dropped"]
            result[key]["bridge_alive"] = lane["alive"]
            result[key]["bridge_errors"] = lane.get("errors", 0)
            result[key]["bridge_last_error"] = lane.get("last_error")
            if self.media_workers is not None and (key.startswith("camera_") or key in LIDAR_LANES):
                result[key]["worker_alive"] = self.media_workers.is_alive(key)
            if key == "lidar_cloud":
                result[key]["active_source"] = self._active_lidar_source
                result[key]["source_publishers"] = {
                    topic: self._count_publishers(topic) for topic in self._lidar_topics
                }
            elif key == "lidar_map":
                result[key]["source"] = self._lidar_map_topic
                result[key]["source_publishers"] = self._count_publishers(self._lidar_map_topic)
            elif key.startswith("camera_"):
                result[key]["active_source"] = self._camera_lane_sources.get(key)
                source = result[key]["active_source"]
                result[key]["source_publishers"] = (
                    self._count_publishers(source) if source else None)
        return result

    def _count_publishers(self, topic):
        try:
            return self.robot.count_publishers(topic)
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return None

    def _publish_json(self, key, value):
        from std_msgs.msg import String
        message = String()
        message.data = self.json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=str)
        self.publishers[key].publish(message)
        self._record_stream(key, "published")

    def _forward_imu(self, msg):
        self._record_stream("imu", "received")
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        # Keep the dashboard state card flat, matching the Unitree drivers.
        payload = {}
        for group in ("orientation", "angular_velocity", "linear_acceleration"):
            value = values.get(group, {}) if isinstance(values, dict) else {}
            if isinstance(value, dict):
                for axis in ("x", "y", "z", "w"):
                    if axis in value:
                        payload[f"{group}_{axis}"] = value[axis]
        header = values.get("header", {}) if isinstance(values, dict) else {}
        stamp = header.get("stamp", {}) if isinstance(header, dict) else {}
        if isinstance(stamp, dict):
            for key in ("sec", "nanosec"):
                if key in stamp:
                    payload[key] = stamp[key]
        for key, value in (values.items() if isinstance(values, dict) else ()):
            if key not in {"header", "orientation", "angular_velocity", "linear_acceleration"}:
                payload[key] = value
        self._publish_json("imu", payload)

    def _forward_joints(self, msg):
        self._record_stream("joints", "received")
        self._record_stream("joint_state", "received")
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        data = values.get("data", {})
        joints = values.get("joints_data", data.get("joints_data", []) if isinstance(data, dict) else data)
        ordered = _order_joints(joints)
        motor_status = {"Joint": ordered}
        self._publish_json("joints", _skeleton_payload({"MotorStatus": motor_status}, raw_joints=ordered))
        flat = {}
        for index, joint in enumerate(ordered):
            name = JOINT_NAMES[index]
            for field, aliases in {
                "q": ("q", "position"), "dq": ("dq", "velocity"),
                "tau": ("tau", "torque"), "temperature": ("temperature", "motion_temp"),
                "driver_temp": ("driver_temp",), "data_id": ("data_id",),
                "status_word": ("status_word",),
            }.items():
                value = _joint_value(joint, *aliases)
                if value is not None:
                    flat[f"{name}_{field}"] = value
        self._publish_json("joint_state", flat)

    def _forward_battery(self, msg):
        self._record_stream("battery", "received")
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        batteries = values.get("data", []) if isinstance(values, dict) else []
        slots = batteries if isinstance(batteries, list) else []
        # Keep actual packs and their fields addressable by slot. Fixed-size
        # arrays such as battery_serialnum contain reserved zero entries;
        # retain those as one compact field instead of expanding them into
        # dozens of dashboard values.
        payload = {}
        zero_placeholders = {}
        for index, battery in enumerate(slots):
            if not isinstance(battery, dict):
                if _all_zero_placeholder(battery):
                    zero_placeholders[f"battery_{index}"] = {"all_zero": True}
                else:
                    payload[f"battery_{index}"] = {"value": battery}
                continue
            if _all_zero_placeholder(battery):
                zero_placeholders[f"battery_{index}"] = {"all_zero": True}
                continue
            pack = {}
            for field, value in battery.items():
                if isinstance(value, (list, tuple)) and value and all(
                        _all_zero_placeholder(item) for item in value):
                    zero_placeholders[f"battery_{index}_{field}"] = {
                        "all_zero": True, "length": len(value)}
                else:
                    pack[field] = value
            if pack:
                payload[f"battery_{index}"] = pack
        if zero_placeholders:
            payload["zero_placeholders"] = zero_placeholders
        self._publish_json("battery", payload)

    def _forward_loco_state(self, msg):
        self._record_stream("loco_state", "received")
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        data = values.get("data", values)
        motion = data.get("motion_state", {}) if isinstance(data, dict) else {}
        gait = data.get("gait_state", {}) if isinstance(data, dict) else {}
        with self._motion_lock:
            self.motion_sequence += 1
            self.motion_state = motion.get("state") if isinstance(motion, dict) else None
            self.gait_state = gait.get("gait") if isinstance(gait, dict) else None
        self._publish_json("loco_state", data)

    @staticmethod
    def _jpeg_from_ros_image(msg):
        from io import BytesIO
        from PIL import Image as PillowImage

        encoding = str(msg.encoding).lower()
        mode, raw_mode = {
            "rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"),
            "rgba8": ("RGBA", "RGBA"), "bgra8": ("RGBA", "BGRA"),
            "mono8": ("L", "L"), "8uc1": ("L", "L"),
        }.get(encoding, (None, None))
        if mode is None:
            return None
        try:
            image = PillowImage.frombytes(mode, (int(msg.width), int(msg.height)),
                                          bytes(msg.data), "raw", raw_mode, int(msg.step), 1)
            if image.mode == "RGBA":
                image = image.convert("RGB")
            stream = BytesIO()
            image.save(stream, format="JPEG", quality=85, optimize=False)
            return stream.getvalue()
        except (ValueError, OSError):
            return None

    @staticmethod
    def _fresh_header(header):
        """Reject DDS samples that were already stale before this callback."""
        if header is None or not hasattr(header, "stamp"):
            return True
        stamp = header.stamp
        seconds = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
        if seconds <= 0:
            return True
        age = time.time() - seconds
        return -0.25 <= age <= MEDIA_MAX_AGE_S

    def _forward_camera(self, key, msg, already_received=False):
        # S10 camera timestamps come from the AOS clock and are not guaranteed
        # to share the container wall clock. Do not discard valid frames based
        # on that clock skew; the worker/bridge latest-only queues provide the
        # latency bound without blacking out the camera.
        if not already_received:
            self._record_stream(key, "received")
        # The S10 camera node declares MJPEG as sensor_msgs/Image. For a valid
        # JPEG payload, publish directly from the callback: sending it through
        # a worker only adds an IPC hop and a frame of latency. Raw/NV12 frames
        # still use the isolated encoder worker below.
        payload = msg.data
        if str(msg.encoding).lower() in {"jpeg", "jpg", "mjpeg"} and \
                len(payload) >= 4 and payload[0] == 0xff and payload[1] == 0xd8 and \
                payload[-2] == 0xff and payload[-1] == 0xd9:
            from sensor_msgs.msg import CompressedImage
            output = CompressedImage()
            output.header = msg.header
            output.format = "jpeg"
            output.data = payload
            self.camera_publishers[key].publish(output)
            self._record_stream(key, "published")
            return
        if self.media_workers is not None:
            header = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec)
            submitted = self.media_workers.submit(
                key, (header, msg.width, msg.height, msg.step, msg.encoding, bytes(msg.data)))
            if not submitted:
                self._record_stream(key, "dropped")
            return
        packet = self._jpeg_from_ros_image(msg)
        if packet is None:
            self._record_stream(key, "dropped")
            return
        from sensor_msgs.msg import CompressedImage
        output = CompressedImage()
        output.header = msg.header
        output.format = "jpeg"
        output.data = packet
        self.camera_publishers[key].publish(output)
        self._record_stream(key, "published")

    def _forward_compressed_camera(self, key, msg, already_received=False):
        # The vendor declares MJPEG as sensor_msgs/Image. Its timestamp may be
        # from a different clock domain, so freshness is diagnostic-only here.
        if not already_received:
            self._record_stream(key, "received")
        payload = bytes(msg.data)
        if payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9"):
            from sensor_msgs.msg import CompressedImage
            output = CompressedImage()
            output.header = msg.header
            output.format = "jpeg"
            output.data = payload
            self.camera_publishers[key].publish(output)
            self._record_stream(key, "published")
            return
        if self.media_workers is not None:
            header = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec)
            submitted = self.media_workers.submit(key, (header, bytes(msg.data)))
            if not submitted:
                self._record_stream(key, "dropped")
            return
        from io import BytesIO
        from PIL import Image as PillowImage
        from sensor_msgs.msg import CompressedImage

        try:
            image = PillowImage.open(BytesIO(bytes(msg.data))).convert("RGB")
            stream = BytesIO()
            image.save(stream, format="JPEG", quality=85, optimize=False)
            packet = stream.getvalue()
        except (OSError, ValueError):
            self._record_stream(key, "dropped")
            return
        output = CompressedImage()
        output.header = msg.header
        output.format = "jpeg"
        output.data = packet
        self.camera_publishers[key].publish(output)
        self._record_stream(key, "published")

    def _forward_lidar(self, source, msg=None):
        if msg is None:
            msg, source = source, None
        now = time.monotonic()
        if not self._fresh_header(getattr(msg, "header", None)):
            self._record_stream("lidar_cloud", "dropped")
            return
        if source is not None:
            self._lidar_source_last_seen[source] = now
            active_seen = self._lidar_source_last_seen.get(self._active_lidar_source, 0)
            # A discovered topic may produce one stale/diagnostic frame before
            # the configured live stream starts. Let an inactive source take
            # over after the current source has gone quiet.
            direct_sources = {"/DEPTH_POINTS", "/local_scans", "/LIDAR/POINTS"}
            # Prefer a live raw sensor feed over accumulated maps/debug clouds.
            # Derived clouds are fallback-only and must never steal the lane
            # while a direct source is producing frames.
            active_direct = self._active_lidar_source in direct_sources
            candidate_direct = source in direct_sources
            active_priority = (self._lidar_topics.index(self._active_lidar_source)
                               if self._active_lidar_source in self._lidar_topics else None)
            candidate_priority = (self._lidar_topics.index(source)
                                  if source in self._lidar_topics else None)
            if (self._active_lidar_source is None or
                    (candidate_direct and not active_direct) or
                    (candidate_priority is not None and
                     (active_priority is None or candidate_priority < active_priority)) or
                    (source != self._active_lidar_source and now - active_seen > 2.0)):
                self._active_lidar_source = source
            elif source != self._active_lidar_source:
                return
        self._forward_lidar_packet("lidar_cloud", msg)

    def _forward_lidar_map(self, msg):
        """Forward only the configured accumulated map; never mix it with raw lidar."""
        self._forward_lidar_packet("lidar_map", msg)

    def _forward_lidar_packet(self, lane, msg):
        if not self._fresh_header(getattr(msg, "header", None)):
            self._record_stream(lane, "dropped")
            return
        self._record_stream(lane, "received")
        if self.media_workers is not None:
            fields = [(field.name, field.offset, field.datatype, field.count)
                      for field in msg.fields]
            header = getattr(msg, "header", None)
            stamp = getattr(header, "stamp", None)
            media_header = ((getattr(header, "frame_id", "") if header else ""),
                            getattr(stamp, "sec", 0) if stamp else 0,
                            getattr(stamp, "nanosec", 0) if stamp else 0)
            submitted = self.media_workers.submit(
                lane, (media_header,
                           fields, msg.is_bigendian, msg.width, msg.height,
                           msg.row_step, msg.point_step, bytes(msg.data)))
            if not submitted:
                self._record_stream(lane, "dropped")
            return
        packet = self._pointcloud_packet(msg)
        if packet is None:
            self._record_stream(lane, "dropped")
            return
        from std_msgs.msg import UInt8MultiArray
        output = UInt8MultiArray()
        output.data = array.array("B", packet)
        self.lidar_publishers[lane].publish(output)
        self._record_stream(lane, "published")

    def _handle_media_result(self, lane, header, packet):
        if packet is None:
            self._record_stream(lane, "dropped")
            return
        if lane in LIDAR_LANES:
            from std_msgs.msg import UInt8MultiArray
            output = UInt8MultiArray()
            output.data = array.array("B", packet)
            self.lidar_publishers[lane].publish(output)
        else:
            from sensor_msgs.msg import CompressedImage
            output = CompressedImage()
            output.header.frame_id = header[0]
            output.header.stamp.sec = header[1]
            output.header.stamp.nanosec = header[2]
            output.format = "jpeg"
            output.data = packet
            self.camera_publishers[lane].publish(output)
        self._record_stream(lane, "published")

    @staticmethod
    def _pointcloud_packet(msg):
        """Encode a bounded, spatially distributed sample for the cloud renderer."""
        fields = [(field.name, field.offset, field.datatype, field.count)
                  for field in msg.fields]
        return encode_pointcloud(fields, msg.is_bigendian, msg.width, msg.height,
                                 msg.row_step, msg.point_step, bytes(msg.data),
                                 MAX_LIDAR_RENDER_POINTS)

    def close(self):
        if self.media_workers is not None:
            self.media_workers.close()
        self.robot.destroy_node()
        self.bridge.stop()


LOCO_STATES = {"stand_up": 1, "damp": 2, "stand_down": 4}
GAITS = {"basic_flat": 0x1001, "basic_stairs": 0x1003, "navigation_flat": 0x3002, "navigation_stairs": 0x3003}
MODES = {"normal": 0, "navigation": 1, "assist": 2}
JOINT_NAMES = (
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
)
URDF_PATH = Path(__file__).with_name("resource") / "S10.urdf"
MAX_LIDAR_RENDER_POINTS = 2000
JOINT_DIRECTIONS = (1, 1, -1, 1, 1, -1, 1, -1,
                    -1, 1, -1, 1, -1, -1, 1, -1)
JOINT_OFFSETS_DEG = (-35, -145, 156, 0, 35, -145, 156, 0,
                     -35, 145, -156, 0, 35, 145, -156, 0)
MODEL_REFERENCE_ANGLES = (0.0, -0.35, 0.65, 0.0,
                          0.0, -0.35, 0.65, 0.0,
                          0.0, 0.35, -0.65, 0.0,
                          0.0, 0.35, -0.65, 0.0)


def _model_joint_position(value, index):
    if value is None:
        return None
    try:
        return (float(value) * JOINT_DIRECTIONS[index]
                + math.radians(JOINT_OFFSETS_DEG[index])
                - MODEL_REFERENCE_ANGLES[index])
    except (TypeError, ValueError):
        return value


def _joint_value(joint, *keys):
    if not isinstance(joint, dict):
        return None
    for key in keys:
        if key in joint:
            return joint[key]
    return None


def _joint_name(value):
    if isinstance(value, (list, tuple)):
        try:
            return bytes(int(item) & 0xff for item in value).decode("ascii", "ignore").rstrip("\x00").upper()
        except (TypeError, ValueError):
            return ""
    return str(value or "").strip().upper()


_JOINT_ALIASES = {name.upper(): index for index, name in enumerate(JOINT_NAMES)}
_JOINT_ALIASES.update({
    f"{prefix}{axis}": index
    for index, (prefix, axis) in enumerate((
        ("FL", "HX"), ("FL", "HY"), ("FL", "KN"), ("FL", "WH"),
        ("FR", "HX"), ("FR", "HY"), ("FR", "KN"), ("FR", "WH"),
        ("HL", "HX"), ("HL", "HY"), ("HL", "KN"), ("HL", "WH"),
        ("HR", "HX"), ("HR", "HY"), ("HR", "KN"), ("HR", "WH"),
    ))
})


def _joint_index(joint):
    if not isinstance(joint, dict):
        return None
    name = _joint_name(joint.get("name", joint.get("Name", "")))
    # S10 firmware encodes motor indices as ASCII bytes (for example
    # [49, 53] -> "15").  data_id is a vendor message id, not an index.
    if name.isdigit():
        numeric = int(name)
        if 0 <= numeric < len(JOINT_NAMES):
            return numeric
    if name in _JOINT_ALIASES:
        return _JOINT_ALIASES[name]
    compact = name.replace("_JOINT", "").replace("_", "")
    if compact in _JOINT_ALIASES:
        return _JOINT_ALIASES[compact]
    try:
        data_id = int(joint.get("data_id", joint.get("DataId", -1)))
    except (TypeError, ValueError):
        data_id = -1
    if 0 <= data_id < len(JOINT_NAMES):
        return data_id
    return None


def _order_joints(raw_joints):
    if not isinstance(raw_joints, list):
        return [None] * len(JOINT_NAMES)
    ordered = [None] * len(JOINT_NAMES)
    names = [_joint_name(joint.get("name", joint.get("Name", "")))
             if isinstance(joint, dict) else "" for joint in raw_joints]
    numeric_names = [int(name) for name in names if name.isdigit()]
    use_numeric_names = (len(numeric_names) == len(JOINT_NAMES)
                         and len(set(numeric_names)) == len(JOINT_NAMES)
                         and set(numeric_names) == set(range(len(JOINT_NAMES))))
    indexed = ([int(name) if name.isdigit() else None for name in names]
               if use_numeric_names else [_joint_index(joint) for joint in raw_joints])
    usable = sum(index is not None for index in indexed)
    if usable:
        # Some firmware revisions number data_id from one through sixteen;
        # detect that convention only when the complete set is present.
        raw_ids = []
        for joint in raw_joints:
            try:
                raw_ids.append(int(joint.get("data_id", joint.get("DataId", -1))))
            except (AttributeError, TypeError, ValueError):
                pass
        if len(raw_ids) == len(JOINT_NAMES) and set(raw_ids) == set(range(1, len(JOINT_NAMES) + 1)):
            indexed = [int(joint.get("data_id", joint.get("DataId"))) - 1
                       if isinstance(joint, dict) else None for joint in raw_joints]
        for joint, index in zip(raw_joints, indexed):
            if index is not None and 0 <= index < len(ordered) and ordered[index] is None:
                ordered[index] = joint
        return ordered
    return (raw_joints + [None] * len(JOINT_NAMES))[:len(JOINT_NAMES)]


def _flatten_fields(value, prefix, output, omit_zero_placeholders=False):
    if isinstance(value, dict):
        for key, child in value.items():
            _flatten_fields(child, f"{prefix}_{key}", output, omit_zero_placeholders)
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            # Battery firmware reserves fixed-size array slots and fills them
            # with zero-valued records. Do not expose those placeholders as
            # dozens of dashboard fields, while retaining real non-zero cells.
            if omit_zero_placeholders and _all_zero_placeholder(child):
                continue
            _flatten_fields(child, f"{prefix}_{index}", output, omit_zero_placeholders)
    else:
        output[prefix] = value


def _all_zero_placeholder(value):
    if value is None or value is False or value == "":
        return True
    if isinstance(value, dict):
        values = value.values()
        return bool(value) and all(_all_zero_placeholder(item) for item in values)
    if isinstance(value, (list, tuple)):
        return bool(value) and all(_all_zero_placeholder(item) for item in value)
    if isinstance(value, (bytes, bytearray)):
        return bool(value) and not any(value)
    if isinstance(value, str):
        return not value.strip("\x00 ")
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value == 0


def _skeleton_payload(items, raw_joints=None):
    motor_status = items.get("MotorStatus", {})
    if raw_joints is None:
        raw_joints = motor_status.get("Joint", []) if isinstance(motor_status, dict) else []
    if isinstance(raw_joints, dict):
        raw_joints = raw_joints.get("Joint", raw_joints.get("Data", []))
    raw_joints = _order_joints(raw_joints)
    joints = []
    for idx, name in enumerate(JOINT_NAMES):
        raw = raw_joints[idx] if idx < len(raw_joints) else None
        joints.append({
            "idx": idx, "name": name,
            "q": _model_joint_position(
                _joint_value(raw, "q", "Q", "position", "Position", "Pos", "Angle"), idx),
            "dq": _joint_value(raw, "dq", "DQ", "velocity", "Velocity", "Speed"),
            "tau": _joint_value(raw, "tau", "Tau", "torque", "Torque", "TorqueEst"),
            "temperature": _joint_value(raw, "temperature", "Temperature", "Temp", "motion_temp", "driver_temp"),
        })
    # Match the standard skeleton contract used by the other robot drivers.
    payload = {"format": "sensor/skeleton", "joint_count": len(joints), "joints": joints}
    motion = items.get("MotionStatus", {})
    try:
        roll, pitch, yaw = (float(motion[key]) for key in ("Roll", "Pitch", "Yaw"))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        payload["imu_quat"] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                               cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
    except (KeyError, TypeError, ValueError):
        pass
    return payload


class S10ModelPlugin:
    def get_tool(self):
        return tool("model", "resource", "云深处山猫 S10 官方 URDF 运动学模型，用于 joints 骨架渲染")
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "model":
            return {"urdf": URDF_PATH.read_text(encoding="utf-8")}
        return {"state": "ready"}


class S10StatePlugin:
    def __init__(self, native, nodes=None): self.native, self.nodes = native, nodes
    def get_tools(self):
        cards = {
            "imu": "山猫 S10 IMU 加速度和角速度（由原生运控状态报告提供，10Hz）",
            "joints": "山猫 S10 16 个轮足关节角度（原生协议 10Hz）",
            "joint_state": "山猫 S10 轮足关节位置、速度、力矩和温度（原生协议 10Hz）",
            "loco_state": "山猫 S10 运动状态、步态、模式和速度（原生协议 10Hz）",
            "battery": "山猫 S10 电池电量、电压、温度和充电状态",
            "lidar_cloud": "山猫 S10 原始实时激光雷达点云",
            "lidar_map": "山猫 S10 累积/定位点云地图",
        }
        topics = self.nodes.topics if self.nodes else {}
        for camera in (key for key in topics if key.startswith("camera_")):
            cards[camera] = f"山猫 S10 机载相机 {camera.removeprefix('camera_')} 图像流；不推断物理方位"
        definitions = []
        for name, description in cards.items():
            outputs = [topics[name]] if name in topics else None
            definitions.append(tool(name, "sensor", description, topic_out=outputs))
        return definitions

    def _topic_result(self, name, state):
        result = {"state": state}
        if self.nodes and name in self.nodes.topics:
            result["topic_out"] = [self.nodes.topics[name]]
        return result

    def start(self): self.native.start()
    def stop(self):
        if self.nodes: self.nodes.close()
        self.native.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name", "")
        if action == "start":
            self.native.start()
            return self._topic_result(name, "running")
        if action == "stop":
            # The TCP client is shared by every S10 card and is closed only
            # when the driver bundle itself shuts down.
            return self._topic_result(name, "idle")
        if action == "info": return self._topic_result(name, "ready")
        reports = self.native.snapshot()["reports"]
        keys = {"imu": "1048577:15728640", "joints": "1048577:15728640",
                "joint_state": "1048577:15728640", "loco_state": "1048577:15728640",
                "battery": "1048578:15728640",
                }
        if name in LIDAR_LANES and self.nodes:
            return {"state": "streaming", **self.nodes.topics[name]}
        if name.startswith("camera_") and self.nodes:
            return {"state": "streaming", **self.nodes.topics[name]}
        if name not in keys: return {"state": "unsupported_card", "card": name}
        report = reports.get(keys[name])
        # Some firmware revisions serialize Type/Command as decimal strings,
        # while others use numeric JSON values.  Match both representations.
        if report is None:
            expected = tuple(int(part) for part in keys[name].split(":"))
            for candidate in reports.values():
                body = candidate.get("PatrolDevice", {}) if isinstance(candidate, dict) else {}
                try:
                    if (_protocol_int(body.get("Type")), _protocol_int(body.get("Command"))) == expected:
                        report = candidate
                        break
                except (TypeError, ValueError):
                    continue
        if not report: return {"state": "waiting_for_report", "report_key": keys[name]}
        items = report.get("PatrolDevice", {}).get("Items", {})
        if name in ("imu", "joints", "joint_state", "loco_state"):
            data = items.get("MotionStatus", {})
            if name == "imu": return {"acceleration_m_s2": [data.get("AccX"), data.get("AccY"), data.get("AccZ")], "angular_velocity_rad_s": [data.get("OmegaX"), data.get("OmegaY"), data.get("OmegaZ")], "orientation_rad": [data.get("Roll"), data.get("Pitch"), data.get("Yaw")]}
            if name == "joints": return _skeleton_payload(items)
            if name == "joint_state": return items.get("MotorStatus", {})
            basic_report = reports.get("1048676:15728640", {})
            basic_items = basic_report.get("PatrolDevice", {}).get("Items", {})
            basic = basic_items.get("BasicStatus", {})
            return {"motion_state": data, "basic_state": basic}
        if name == "battery": return {"batteries": items.get("BatteryList", [])}
        return items


class S10MotionPlugin:
    ACTIONS = {
        "move": (["vx", "vy", "yaw_rate_deg_s", "duration"], "导航模式下按速度移动；偏航角速度使用度/秒；duration=-1 最多持续 60 秒"),
        "stop_move": ([], "立即发送零速度停止运动"),
        "stand_up": ([], "切换到站立状态"), "stand_down": ([], "切换到趴下状态"),
        "damp": ([], "切换到关节阻尼状态"), "gait": (["gait"], "切换步态；请先确认机器人完全停止"),
        "mode": (["mode"], "切换常规、导航或辅助控制模式"),
        "manual_control": (["x", "y", "z", "roll", "pitch", "yaw", "duration"],
                           "六轴归一化手动控制，各分量为-1到1的控制强度，不是角度；仅常规/辅助模式有效"),
    }
    def __init__(self, native, nodes=None):
        self.native = native
        self.nodes = nodes
        self._condition = threading.Condition()
        self._active = None
        self._transition_lock = threading.Lock()
        self._active_transition = None
        self._shutdown = False
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name="s10-motion-stream")

    def _acp_callback(self, action_id, status, result):
        _notify_acp_async(action_id, status, result, "loco")

    def _finish(self, active, status, result):
        if active and active.get("action_id"):
            self._acp_callback(active["action_id"], status, result)

    def _reported_motion_value(self, field):
        if self.nodes and field in ("MotionState", "Gait"):
            with self.nodes._motion_lock:
                value = self.nodes.motion_state if field == "MotionState" else self.nodes.gait_state
                return value, self.nodes.motion_sequence
        reports = self.native.snapshot().get("reports", {})
        for report in reports.values():
            body = report.get("PatrolDevice", {}) if isinstance(report, dict) else {}
            items = body.get("Items", {}) if isinstance(body, dict) else {}
            basic = items.get("BasicStatus", {}) if isinstance(items, dict) else {}
            if field in basic:
                try:
                    return int(basic[field]), id(report)
                except (TypeError, ValueError):
                    return basic[field], id(report)
        return None, None

    def _finish_transition(self, action_id, status, result):
        with self._transition_lock:
            if not self._active_transition or self._active_transition["action_id"] != action_id:
                return
            self._active_transition = None
        self._finish({"action_id": action_id}, status, result)

    def _cancel_transition(self):
        with self._transition_lock:
            active = self._active_transition
            self._active_transition = None
        if active:
            self._finish(active, "cancelled", {"state": "cancelled", "reason": "stopped_or_replaced"})

    def _wait_transition(self, transition):
        deadline = time.monotonic() + transition["timeout"]
        while time.monotonic() < deadline:
            with self._transition_lock:
                if not self._active_transition or self._active_transition["action_id"] != transition["action_id"]:
                    return
            value, report = self._reported_motion_value(transition["field"])
            if report != transition["baseline_report"] and value in transition["expected"]:
                self._finish_transition(transition["action_id"], "completed", {
                    "state": "completed", "action": transition["action"],
                    "observed": {transition["field"]: value},
                })
                return
            time.sleep(0.1)
        self._finish_transition(transition["action_id"], "error", {
            "state": "error", "action": transition["action"],
            "message": f"timed out waiting for S10 {transition['field']} to reach {sorted(transition['expected'])}",
        })

    def _start_transition(self, action, message_type, command, items, field, expected, timeout=15):
        self._stop_stream()
        action_id = f"s10_{action}_{uuid4().hex}"
        _value, baseline_report = self._reported_motion_value(field)
        sent = self.native.send(message_type, command, items)
        transition = {"action_id": action_id, "action": action, "field": field,
                      "expected": set(expected), "timeout": timeout,
                      "baseline_report": baseline_report}
        with self._transition_lock:
            self._active_transition = transition
        threading.Thread(target=self._wait_transition, args=(transition,), daemon=True,
                         name=f"s10-{action}-completion").start()
        return {**sent, "state": "accepted", "action_id": action_id, "completion_timeout": timeout}
    def get_tool(self):
        result = tool("loco", "actuator", "山猫 S10 运动控制。move 在导航步态下按米/秒和度/秒移动；manual_control 是常规/辅助模式的归一化六轴指令；stand_up 通常会自动进入 RL 运控状态。", action_schema(self.ACTIONS, {
            "gait": {"type": "string", "enum": list(GAITS)},
            "mode": {"type": "string", "enum": list(MODES)},
            "x": {"type": "number", "minimum": -1, "maximum": 1, "description": "前后方向归一化控制强度，不是速度值"},
            "y": {"type": "number", "minimum": -1, "maximum": 1, "description": "左右方向归一化控制强度，不是速度值"},
            "z": {"type": "number", "minimum": -1, "maximum": 1, "description": "高度方向归一化控制强度；基础步态不响应"},
            "roll": {"type": "number", "minimum": -1, "maximum": 1, "description": "翻滚方向归一化控制强度；基础步态不响应"},
            "pitch": {"type": "number", "minimum": -1, "maximum": 1, "description": "俯仰方向归一化控制强度；基础步态不响应"},
            "yaw": {"type": "number", "minimum": -1, "maximum": 1, "description": "偏航归一化控制强度，不是角度"},
            "vx": {"type": "number", "minimum": -1.67, "maximum": 1.67, "description": "前后线速度，范围 -1.67 至 1.67 m/s"},
            "vy": {"type": "number", "minimum": -0.4, "maximum": 0.4, "description": "左右横移速度，范围 -0.4 至 0.4 m/s"},
            "yaw_rate_deg_s": {"type": "number", "minimum": -57.29577951308232, "maximum": 57.29577951308232,
                                "description": "偏航角速度，范围 -57.30 至 57.30 度/秒；发送前转换为弧度/秒"},
            "duration": {"type": "number", "minimum": -1, "maximum": 60, "default": 1,
                         "description": "Seconds; -1 is a compatibility alias for 60 seconds; 0 stops immediately"},
        }))
        schema = result["inputSchema"]
        schema["x-completion"] = {"actions": ["move", "manual_control", "stand_up", "stand_down",
                                               "damp", "gait"], "timeout": 65}
        return result
    def start(self):
        with self._condition:
            if self._thread.is_alive():
                return
            self._shutdown = False
            self._thread = threading.Thread(target=self._stream_loop, daemon=True,
                                             name="s10-motion-stream")
            self._thread.start()
    def stop(self):
        self._stop_stream()
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
        if self._thread.is_alive(): self._thread.join(timeout=1)
    def _send_zero(self, kind=None):
        zero = {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
        if kind in (None, "axis"): self.native.send(0x00100001, 0x00100002, zero)
        if kind in (None, "velocity"): self.native.send(0x00100001, 0x00110002, zero)
    def _stop_stream(self, send_both=False):
        self._cancel_transition()
        with self._condition:
            active = self._active
            self._active = None
            self._condition.notify_all()
        stop_error = None
        try:
            if send_both: self._send_zero()
            elif active: self._send_zero(active["kind"])
        except Exception as exc:
            stop_error = exc
        if active:
            if stop_error:
                self._finish(active, "error", {"state": "error", "message": f"zero command failed: {stop_error}"})
            else:
                self._finish(active, "cancelled", {"state": "cancelled", "reason": "stopped_or_replaced"})
        if stop_error:
            raise stop_error
    def _stream_loop(self):
        while True:
            with self._condition:
                while not self._shutdown and self._active is None: self._condition.wait()
                if self._shutdown: return
                command = self._active
                now = time.monotonic()
                if now >= command["deadline"]:
                    self._active = None
                    try:
                        self._send_zero(command["kind"])
                    except Exception as exc:
                        self._finish(command, "error", {"state": "error", "message": f"zero command failed: {exc}"})
                    else:
                        self._finish(command, "completed", {"state": "completed", "reason": "duration_elapsed"})
                    continue
                delay = command["next_send"] - now
                if delay > 0:
                    self._condition.wait(min(delay, command["deadline"] - now))
                    continue
                try:
                    self.native.send(command["type"], command["command"], command["items"])
                    command["next_send"] = time.monotonic() + 0.1
                except Exception as exc:
                    self._active = None
                    try:
                        self._send_zero(command["kind"])
                    except Exception as stop_exc:
                        exc = RuntimeError(f"{exc}; zero command failed: {stop_exc}")
                    self._finish(command, "error", {"state": "error", "message": str(exc)})
    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self._stop_stream()
            return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "stop_move":
            self._stop_stream(send_both=True)
            return {"state": "stopped", "sent_zero_to": ["axis", "velocity"]}
        transition_actions = {
            "stand_up": ("MotionState", {1, 17}),
            "stand_down": ("MotionState", {4}),
            "damp": ("MotionState", {2}),
        }
        if action in LOCO_STATES:
            field, expected = transition_actions.get(action, (None, None))
            if field:
                return self._start_transition(action, 0x00100001, 0x00200002,
                                              {"MotionParam": LOCO_STATES[action]}, field, expected)
        if action == "gait":
            return self._start_transition(action, 0x00100001, 0x00300002,
                                          {"GaitParam": GAITS[args["gait"]]}, "Gait",
                                          {GAITS[args["gait"]]})
        if action == "mode":
            self._stop_stream()
            return self.native.send(0x00100002, 0x00500002, {"Mode": MODES[args["mode"]]})
        if action in ("manual_control", "axis", "move"):
            axes = ("x", "y", "z", "roll", "pitch", "yaw")
            wire_names = {"x": "X", "y": "Y", "z": "Z", "roll": "Roll", "pitch": "Pitch", "yaw": "Yaw"}
            is_manual = action in ("manual_control", "axis")
            yaw_rate = args.get("yaw_rate_deg_s", math.degrees(float(args.get("vyaw", 0))))
            items = {wire_names[key]: float(args.get(key if is_manual else {"x": "vx", "y": "vy", "yaw": "_yaw_rate"}.get(key, key), 0)) for key in axes}
            if action == "move":
                items["Yaw"] = math.radians(float(yaw_rate))
                bounds = {"X": 1.67, "Y": 0.4, "Yaw": 1.0}
                if any(abs(items[key]) > limit for key, limit in bounds.items()):
                    raise ValueError("velocity exceeds documented limits: X=1.67 m/s, Y=0.4 m/s, yaw=57.3 degrees/s")
                kind, message_type, command = "velocity", 0x00100001, 0x00110002
            else:
                if any(abs(value) > 1 for value in items.values()): raise ValueError("axis values must be within [-1, 1]")
                kind, message_type, command = "axis", 0x00100001, 0x00100002
            duration = float(args.get("duration", 1))
            if not -1 <= duration <= 60: raise ValueError("duration must be within [-1, 60]")
            if duration == 0:
                self._stop_stream(send_both=True)
                action_id = f"s10_{action}_{uuid4().hex}"
                self._finish({"action_id": action_id}, "completed", {"state": "stopped", "reason": "zero_duration"})
                return {"state": "stopped", "sent_zero_to": ["axis", "velocity"], "action_id": action_id}
            self._stop_stream()
            if action == "move":
                self.native.send(0x00100002, 0x00500002, {"Mode": MODES["navigation"]})
                self.native.send(0x00100001, 0x00300002, {"GaitParam": GAITS["navigation_flat"]})
            action_name = "manual_control" if is_manual else action
            action_id = f"s10_{action_name}_{uuid4().hex}"
            sent = self.native.send(message_type, command, items)
            with self._condition:
                now = time.monotonic()
                self._active = {"kind": kind, "type": message_type, "command": command, "items": items,
                                "deadline": now + (60 if duration == -1 else duration), "next_send": now + 0.1,
                                "action_id": action_id}
                self._condition.notify_all()
            return {**sent, "state": "streaming", "frequency_hz": 10,
                    "duration": 60 if duration == -1 else duration, "auto_stop": True,
                    "action_id": action_id}
        raise ValueError(f"unsupported S10 loco action: {action}")


class S10NavigationPlugin:
    CONFIRM_ACTIONS = {"goal", "localize"}
    ACTIONS = {
        "goal": (["x", "y", "z", "yaw_degrees", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous"], "前往设备当前已加载地图中的目标点；不负责建图；航向角使用角度制"),
        "cancel": ([], "取消当前导航任务"), "status": ([], "查询导航任务状态"),
        "localize": (["x", "y", "z", "yaw_degrees"], "按地图坐标初始化/重置定位；航向角使用角度制"),
        "pose": ([], "查询机器人地图坐标和姿态"),
        "perception_status": ([], "查询定位与避障状态"),
    }
    def __init__(self, native):
        self.native = native
        self._goal_lock = threading.Lock()
        self._active_goal = None

    def get_tool(self):
        result = tool("navigation", "actuator", "山猫 S10 Pro 地图导航：使用设备当前已加载的地图和定位，不提供建图；目标导航和定位初始化需要显式 confirm=true。角度输入/输出均为度。", action_schema(self.ACTIONS, {
            "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number", "default": 0},
            "yaw_degrees": {"type": "number", "description": "地图坐标系中的目标朝向，单位为度"}, "point_type": {"type": "integer", "enum": [0, 1, 2, 3, 4], "default": 1},
            "gait": {"type": "string", "enum": ["navigation_flat", "navigation_stairs"], "default": "navigation_flat"},
            "speed": {"type": "integer", "enum": [0, 1, 2], "default": 0}, "reverse": {"type": "boolean", "default": False},
            "avoid_obstacles": {"type": "boolean", "default": True}, "autonomous": {"type": "boolean", "default": False},
            "confirm": {"type": "boolean", "description": "Required true for goal navigation and localization changes."},
        }))
        _mark_dangerous(result, self.CONFIRM_ACTIONS)
        timeout = max(10, math.ceil(float(self.native.config.get("navigation_task_timeout", 180))) + 10)
        result["inputSchema"]["x-completion"] = {"actions": ["goal"], "timeout": timeout}
        return result

    def _acp_callback(self, action_id, status, result):
        _notify_acp_async(action_id, status, result, "navigation")

    def _finish_goal(self, action_id, status, result):
        with self._goal_lock:
            if self._active_goal != action_id:
                return False
            self._active_goal = None
        self._acp_callback(action_id, status, result)
        return True

    @staticmethod
    def _response_error(response):
        body = response.get("PatrolDevice", response) if isinstance(response, dict) else {}
        items = body.get("Items", body) if isinstance(body, dict) else {}
        code = items.get("ErrorCode") if isinstance(items, dict) else None
        if code is None:
            return None
        try:
            raw_code = str(code).strip().lower()
            value = int(raw_code, 16 if raw_code.startswith("0x") else 10)
        except (TypeError, ValueError):
            return str(code)
        return None if value == 0 else value

    def _run_goal(self, action_id, item, timeout):
        try:
            response = self.native.request(0x00100003, 0x00100002, {"Item": [item]}, response_timeout=timeout)
            error = self._response_error(response)
            if error is not None:
                self._finish_goal(action_id, "error", {"state": "error", "error_code": error, "response": response})
                return
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                status = self.native.request(0x00100003, 0x00100001)
                terminal = self._navigation_terminal(status)
                if terminal is not None:
                    state, detail = terminal
                    self._finish_goal(action_id, state, {"state": state, "response": status,
                                                         "command_response": response, **detail})
                    return
                time.sleep(0.2)
            self._finish_goal(action_id, "error", {"state": "error", "message": "navigation status timed out",
                                                    "command_response": response})
        except Exception as exc:
            self._finish_goal(action_id, "error", {"state": "error", "message": str(exc)})

    @staticmethod
    def _navigation_terminal(response):
        """Translate vendor navigation status into an ACP terminal state."""
        body = response.get("PatrolDevice", response) if isinstance(response, dict) else {}
        items = body.get("Items", body) if isinstance(body, dict) else {}
        if not isinstance(items, dict):
            return None
        raw = next((items.get(key) for key in
                    ("NavigationState", "NavState", "TaskState", "Status", "State")
                    if items.get(key) is not None), None)
        if raw is None:
            return None
        value = str(raw).strip().lower()
        if value in {"completed", "complete", "success", "succeeded", "3", "4"}:
            return "completed", {}
        if value in {"cancelled", "canceled", "cancel", "5"}:
            return "cancelled", {"reason": "navigation_cancelled"}
        if value in {"failed", "failure", "error", "aborted", "6", "7"}:
            return "error", {"message": "navigation task failed", "status": raw}
        return None

    def _cancel_goal(self):
        with self._goal_lock:
            action_id = self._active_goal
        if action_id is None:
            return None
        try:
            response = self.native.request(0x00100003, 0x00100004)
        except Exception as exc:
            result = {"state": "error", "reason": "navigation_cancel_failed", "message": str(exc)}
            self._finish_goal(action_id, "error", result)
            return result
        self._finish_goal(action_id, "cancelled", {"state": "cancelled", "reason": "navigation_cancelled"})
        return response

    def start(self): pass
    def stop(self):
        if self._active_goal is not None:
            self._cancel_goal()

    def dispatch(self, action, args):
        if action in self.CONFIRM_ACTIONS:
            error = _confirmation_error(action, args)
            if error:
                return error
        if action == "start": return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "status": return self.native.request(0x00100003, 0x00100001)
        if action == "cancel":
            response = self._cancel_goal()
            return response if response is not None else self.native.request(0x00100003, 0x00100004)
        if action == "pose":
            response = self.native.request(0x00100003, 0x00200001)
            body = response.get("PatrolDevice", {}) if isinstance(response, dict) else {}
            items = body.get("Items", {}) if isinstance(body, dict) else {}
            if isinstance(items, dict):
                for angle in ("Roll", "Pitch", "Yaw"):
                    if angle in items:
                        try:
                            items[f"{angle}Degrees"] = math.degrees(float(items.pop(angle)))
                        except (TypeError, ValueError):
                            pass
            return response
        if action == "perception_status": return self.native.request(0x00100003, 0x00110001)
        if action == "localize":
            return self.native.request(0x00100003, 0x00200002, {"PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0), "Yaw": math.radians(float(args["yaw_degrees"]))})
        if action == "goal":
            item = {"Value": 0, "MapID": 0, "PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0),
                    "AngleYaw": math.radians(float(args["yaw_degrees"])), "PointInfo": int(args.get("point_type", 1)),
                    "Gait": GAITS[args.get("gait", "navigation_flat")], "Speed": int(args.get("speed", 0)),
                    "Manner": int(bool(args.get("reverse", False))), "ObsMode": 0 if args.get("avoid_obstacles", True) else 1,
                    "NavMode": int(bool(args.get("autonomous", False))), "Terrain": 0, "Posture": 0, "Waypoints": {}}
            timeout = float(self.native.config.get("navigation_task_timeout", 180))
            action_id = f"s10_navigation_{uuid4().hex}"
            with self._goal_lock:
                if self._active_goal is not None:
                    return {"ok": False, "code": "ACTION_IN_PROGRESS", "action_id": self._active_goal,
                            "error": "navigation goal already active; cancel it before sending another goal"}
                self._active_goal = action_id
            worker = threading.Thread(target=self._run_goal, args=(action_id, item, timeout),
                                      daemon=True, name="s10-navigation-goal")
            try:
                worker.start()
            except Exception as exc:
                with self._goal_lock:
                    if self._active_goal == action_id:
                        self._active_goal = None
                return {"ok": False, "code": "WORKER_START_FAILED", "error": str(exc)}
            return {"state": "accepted", "action": "goal", "action_id": action_id,
                    "task_timeout": timeout}
        raise ValueError(f"unsupported S10 navigation action: {action}")


LIGHT_COLORS = {"黑色": 0, "白色": 1, "绿色": 2, "蓝色": 3}
LIGHT_ACTION_EFFECTS = {
    "constant": ("常亮", 0), "fill_flow": ("填充流水", 1),
    "moving_flow": ("移动流水", 2), "gradient_flow": ("渐变流水", 3),
    "breathe": ("呼吸", 4), "blink": ("闪烁", 5),
}
LIGHT_EFFECTS = {name: code for name, code in LIGHT_ACTION_EFFECTS.values()}


class S10SingleLightPlugin:
    """Independent head/tail light card with effect-scoped parameters."""
    ACTIONS = {
        "constant": (['color'], "常亮"),
        "fill_flow": (['color'], "填充流水"),
        "moving_flow": (['color'], "移动流水"),
        "gradient_flow": (['color'], "按所选颜色执行渐变流水"),
        "breathe": (['color', 'cycle_seconds'], "呼吸"),
        "blink": (['color', 'cycle_seconds'], "闪烁"),
    }
    _state_lock = threading.Lock()
    _led_state = [
        {"Type": LIGHT_EFFECTS["常亮"], "Color": [LIGHT_COLORS["白色"]]},
        {"Type": LIGHT_EFFECTS["常亮"], "Color": [LIGHT_COLORS["白色"]]},
    ]

    def __init__(self, native, side):
        self.native = native
        self.side = side

    def get_tool(self):
        result = tool(f"{self.side}_light", "actuator",
                      f"山猫 S10 {self.side}灯控制；周期参数只用于呼吸和闪烁",
                      action_schema(self.ACTIONS, {
                          "color": {"type": "string", "enum": list(LIGHT_COLORS), "default": "白色"},
                          "cycle_seconds": {"type": "integer", "minimum": 1, "default": 1,
                                             "description": "该灯呼吸或闪烁周期（秒）"},
                      }))
        return result

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
        if action not in self.ACTIONS:
            raise ValueError(f"unsupported S10 {self.side} light action: {action}")
        effect_code = LIGHT_ACTION_EFFECTS[action][1]
        led = {"Type": effect_code}
        color = args.get("color", "白色")
        if color not in LIGHT_COLORS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "error": "unsupported color"}
        led["Color"] = [LIGHT_COLORS[color]]
        if action in {"breathe", "blink"}:
            cycle = args.get("cycle_seconds", 1)
            if isinstance(cycle, bool) or not isinstance(cycle, int) or cycle < 1:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "error": "cycle_seconds must be a positive integer"}
            led["Cycle"] = cycle
        index = 0 if self.side == "head" else 1
        with self._state_lock:
            self._led_state[index] = led
            leds = [dict(item) for item in self._led_state]
        return self.native.send(0x00100005, 0x00200002,
                                {"CustomMode": True, "Led": leds})


class S10ChargePlugin:
    CONFIRM_ACTIONS = {"start_charging", "stop_charging"}
    def __init__(self, native): self.native = native
    def get_tool(self):
        result = tool("charging", "actuator", "山猫 S10 自主充电控制；开始自主充电需要显式 confirm=true", action_schema(
            {"start_charging": ([], "开始自主充电"), "stop_charging": ([], "停止自主充电")}, {
                "confirm": {"type": "boolean", "description": "Required true to start autonomous charging."},
            }))
        return _mark_dangerous(result, self.CONFIRM_ACTIONS)
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action in self.CONFIRM_ACTIONS:
            error = _confirmation_error(action, args)
            if error:
                return error
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "start_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 1})
        if action == "stop_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 0})
        raise ValueError(f"unsupported S10 charging action: {action}")


def build_plugins(config, namespace=None, ros2=None):
    native = BasicServerClient(config)
    nodes = S10Nodes(config, namespace or "", ros2) if ros2 is not None else None
    plugins = [S10ModelPlugin(), S10StatePlugin(native, nodes), S10MotionPlugin(native, nodes),
               S10SingleLightPlugin(native, "head"),
               S10SingleLightPlugin(native, "tail"),
               S10ChargePlugin(native)]
    if str(config.get("model_variant", "standard")).lower() == "pro":
        plugins.append(S10NavigationPlugin(native))
    return plugins
