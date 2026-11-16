"""LinkerHand L20 joint feedback card (joints).

One fresh position query per round feeds two outputs for the same sample:
  /{namespace}/hand/{side}/joints           data/json        (every round, valid or not)
  /{namespace}/hand/{side}/joints_skeleton  sensor/skeleton  (valid rounds only)

A card in both-hands mode (bound_side=both) owns one worker per hand, each on
its own CAN interface. Both workers also publish their skeleton frames to one
shared topic, so Core draws the two hands in a single view:
  /{namespace}/hand/both[/left_can_<hex>][/right_can_<hex>]/joints_skeleton

The card only sends position-query frames: no motion, torque, calibration or
fault clearing. The dual-hand URDF is served by the `model` resource tool.
"""

from __future__ import annotations

import collections
import contextlib
import json
import logging
import math
import re
import threading
import time
import uuid
from typing import Any

from common.vendor_runtime import action_schema, tool
from can_client import JOINT_NAMES_16, LEFT_HAND_CAN_ID, RIGHT_HAND_CAN_ID
from hand_device import GLOBAL_BUS_LOCK, GLOBAL_CONTROLLERS, get_shared_controller
from joints_model import (ANGLE_SOURCE, JOINT_LABELS, RESERVED_VECTOR_INDEXES, URDF_JOINT,
                          VECTOR_INDEX, model, raw_to_angle, skeleton_joints)


logger = logging.getLogger("linkerhand.joints")


def _number(value, name, lower, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须是 {lower}–{upper} 之间的数值")
    return float(value)


def _valid_channel(channel) -> bool:
    return isinstance(channel, str) and re.fullmatch(r"[A-Za-z0-9_.:-]+", channel) is not None


class _JointsStream:
    """Background acquisition and publishing for one physical hand."""

    def __init__(self, plugin: "JointsPlugin", side: str, channel: str):
        self.plugin = plugin
        self.side = side
        self.channel = channel
        self.json_topic, self.skeleton_topic = plugin.topics(side, channel)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._json_pub = None
        self._skeleton_pub = None
        self._qos = None
        # Shared both-hands skeleton topics wanted by owning cards -> publishers.
        self.dual_topics: set[str] = set()
        self._dual_pubs: dict[str, Any] = {}
        self._pub_lock = threading.Lock()
        self._last_publisher_attempt = 0.0
        self._success_times: collections.deque[float] = collections.deque(maxlen=20)
        self.sequence = 0
        self.latest: dict[str, Any] = {}
        self.published_json = 0
        self.published_skeleton = 0
        self.last_publish_time = None
        self.publish_error = None

    # ── sampling ──────────────────────────────────────────────────────

    def read(self) -> dict[str, Any]:
        """One fresh position query, converted into the data/json payload."""
        plugin = self.plugin
        started = time.monotonic()
        raw20 = None
        reason = None
        details = None
        try:
            client = plugin._get_controller(self.side, self.channel).get()
            vector = client.get_positions_vector(timeout_s=plugin.query_timeout_s)
            if not isinstance(vector, list) or len(vector) != 20 or any(
                    isinstance(vector[i], bool) or not isinstance(vector[i], int) or not 0 <= vector[i] <= 255
                    for i in VECTOR_INDEX.values()):
                raise ValueError("本轮位置反馈不完整或不在 0–255 范围内")
            raw20 = vector
        except Exception as exc:
            reason = str(exc) or type(exc).__name__
            details = getattr(exc, "details", None)
        received = time.time()
        elapsed = time.monotonic() - started
        with self._lock:
            self.sequence += 1
            sequence = self.sequence
            if raw20 is not None:
                self._success_times.append(started + elapsed)
            measured_hz = self._measured_hz()
        joints = []
        for name in JOINT_NAMES_16:
            raw = raw20[VECTOR_INDEX[name]] if raw20 is not None else None
            angle = raw_to_angle(self.side, name, raw) if raw is not None else None
            joints.append({
                "name": name, "label": JOINT_LABELS[name], "vector_index": VECTOR_INDEX[name],
                "model_joint": f"{self.side}_{URDF_JOINT[name]}",
                "raw": raw, "angle_rad": angle,
                "angle_deg": math.degrees(angle) if angle is not None else None,
            })
        return {
            "state": "ok" if raw20 is not None else "error",
            "valid": raw20 is not None,
            "reason": reason,
            "error_details": details,
            "side": self.side, "bound_side": self.side, "can_channel": self.channel,
            "can_id": hex(plugin.can_id(self.side)),
            "sequence": sequence,
            # Time the replies were in hand; failed rounds carry no measurement time.
            "timestamp": received if raw20 is not None else None,
            "published_at": received,
            "source": "can_position_query",
            "query_elapsed_s": round(elapsed, 6),
            "measured_hz": measured_hz,
            "joint_count": len(JOINT_NAMES_16),
            "raw_unit": "protocol 0-255",
            "angle_unit": "rad",
            "angle_source": ANGLE_SOURCE,
            "reserved_vector_indexes": list(RESERVED_VECTOR_INDEXES),
            "joints": joints,
        }

    def _measured_hz(self) -> float | None:
        times = self._success_times
        if len(times) < 2 or times[-1] <= times[0]:
            return None
        return round((len(times) - 1) / (times[-1] - times[0]), 2)

    def skeleton(self, sample: dict[str, Any]) -> dict[str, Any]:
        raw = {joint["name"]: joint["raw"] for joint in sample["joints"]}
        return {
            "joints": skeleton_joints(self.side, raw),
            "status": {key: sample[key] for key in (
                "valid", "side", "can_channel", "sequence", "timestamp", "source", "angle_source")},
        }

    # ── ROS publishers ────────────────────────────────────────────────

    def _setup_publishers(self) -> bool:
        self._last_publisher_attempt = time.monotonic()
        ros2 = self.plugin._ros2
        if ros2 is None or not hasattr(ros2, "ctx_core"):
            self.publish_error = "没有可用的 ROS 2 Core 上下文，仅能本地读取"
            return False
        if self._json_pub is not None:
            return True
        node = None
        try:
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            )
            node = Node(f"hand_joints_{self.side}_{uuid.uuid4().hex[:8]}", context=ros2.ctx_core)
            json_pub = node.create_publisher(String, self.json_topic, qos)
            skeleton_pub = node.create_publisher(String, self.skeleton_topic, qos)
            ros2.executor_core.add_node(node)
            with self._pub_lock:
                self._node, self._json_pub, self._skeleton_pub, self._qos = node, json_pub, skeleton_pub, qos
                self._sync_dual_publishers()
            self.publish_error = None
            print(f"[joints] ROS2 publishers ready: {self.json_topic}, {self.skeleton_topic}", flush=True)
            return True
        except Exception as exc:
            if node is not None:
                try:
                    ros2.executor_core.remove_node(node)
                    node.destroy_node()
                except Exception:
                    pass
            self.publish_error = str(exc)
            logger.warning("Failed to setup ROS2 joints publishers: %s", exc)
            return False

    def set_dual_topics(self, topics: set[str]) -> None:
        with self._pub_lock:
            self.dual_topics = set(topics)
            self._sync_dual_publishers()

    def _sync_dual_publishers(self) -> None:
        """Called with _pub_lock held; publishers follow dual_topics."""
        for topic in list(self._dual_pubs):
            if topic not in self.dual_topics:
                pub = self._dual_pubs.pop(topic)
                try:
                    self._node.destroy_publisher(pub)
                except Exception:
                    pass
        if self._node is None:
            return
        from std_msgs.msg import String
        for topic in sorted(self.dual_topics - set(self._dual_pubs)):
            self._dual_pubs[topic] = self._node.create_publisher(String, topic, self._qos)
            print(f"[joints] ROS2 both-hands skeleton publisher ready: {topic}", flush=True)

    def _teardown_publishers(self) -> None:
        with self._pub_lock:
            node, self._node = self._node, None
            self._json_pub = self._skeleton_pub = None
            self._dual_pubs.clear()
        ros2 = self.plugin._ros2
        if node is None or ros2 is None:
            return
        try:
            ros2.executor_core.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass

    def publisher_ready(self) -> bool:
        return self._json_pub is not None

    def subscriber_count(self) -> int:
        try:
            pubs = (self._json_pub, self._skeleton_pub, *self._dual_pubs.values())
            return sum(pub.get_subscription_count() for pub in pubs if pub is not None)
        except Exception:
            return 0

    # ── worker ────────────────────────────────────────────────────────

    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running():
            return
        self._stop_event.clear()
        self._setup_publishers()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name=f"joints_{self.side}_{self.channel}")
        try:
            self._thread.start()
        except Exception:
            self._stop_event.set()
            self._thread = None
            self._teardown_publishers()
            raise

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.plugin.query_timeout_s + 2.0)
            if thread.is_alive():
                raise RuntimeError("关节采集尚未停止；保留线程及发布器，避免重复启动")
        self._thread = None
        self._publish_stopped()
        self._teardown_publishers()

    def stopped_sample(self) -> dict[str, Any]:
        """Final frame for a stopped stream; carries no measured value.

        Core keeps the last frame of a silent topic on screen and its renderer
        has no invalid state, so without this frame the last `ok/valid=true`
        sample would stay displayed as if it were live.
        """
        with self._lock:
            self.sequence += 1
            sequence = self.sequence
        return self.blank_sample("idle", "关节采集已停止；本帧不含实测值", sequence)

    def waiting_sample(self) -> dict[str, Any]:
        """Complete invalid sample for a running stream whose first round has not ended."""
        with self._lock:
            sequence = self.sequence
        return self.blank_sample("waiting", "关节采集已启动，第一轮位置查询尚未完成", sequence)

    def blank_sample(self, state: str, reason: str, sequence: int) -> dict[str, Any]:
        return {
            "state": state, "valid": False,
            "reason": reason,
            "error_details": None,
            "side": self.side, "bound_side": self.side, "can_channel": self.channel,
            "can_id": hex(self.plugin.can_id(self.side)),
            "sequence": sequence,
            # No new measurement happened, so no measurement time and no rate.
            "timestamp": None,
            "published_at": time.time(),
            "source": "can_position_query",
            "query_elapsed_s": None,
            "measured_hz": None,
            "joint_count": len(JOINT_NAMES_16),
            "raw_unit": "protocol 0-255",
            "angle_unit": "rad",
            "angle_source": ANGLE_SOURCE,
            "reserved_vector_indexes": list(RESERVED_VECTOR_INDEXES),
            "joints": [{
                "name": name, "label": JOINT_LABELS[name], "vector_index": VECTOR_INDEX[name],
                "model_joint": f"{self.side}_{URDF_JOINT[name]}",
                "raw": None, "angle_rad": None, "angle_deg": None,
            } for name in JOINT_NAMES_16],
        }

    def _publish_stopped(self) -> None:
        # Invalid, so _publish sends no skeleton frame and no shared both-hands
        # frame: the renderer would otherwise keep drawing an old pose as live.
        if self._json_pub is None:
            return
        try:
            self._publish(self.stopped_sample())
            # Give the executor a moment to hand the frame over before the
            # node is destroyed in _teardown_publishers().
            time.sleep(0.05)
        except Exception as exc:
            self.publish_error = str(exc)

    def _publish(self, sample: dict[str, Any]) -> None:
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(sample, ensure_ascii=False, allow_nan=False)
        self._json_pub.publish(msg)
        self.published_json += 1
        if sample["valid"]:
            # Invalid rounds publish no skeleton frame: the renderer has no
            # invalid state and would otherwise show an old pose as live.
            msg = String()
            msg.data = json.dumps(self.skeleton(sample), ensure_ascii=False, allow_nan=False)
            self._skeleton_pub.publish(msg)
            self.published_skeleton += 1
            with self._pub_lock:
                # Frames carry only this hand's joints; the renderer keeps the
                # other hand's last pose from its own worker's frames.
                for pub in self._dual_pubs.values():
                    pub.publish(msg)
        self.last_publish_time = time.time()
        self.publish_error = None

    def _loop(self) -> None:
        consecutive_errors = 0
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            # Sequential rounds: the next query starts only after this one ends.
            sample = self.read()
            with self._lock:
                self.latest = sample
            if sample["valid"]:
                if consecutive_errors:
                    logger.info("Joints %s/%s recovered after %d failed rounds", self.side, self.channel, consecutive_errors)
                consecutive_errors = 0
            else:
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 100 == 0:
                    logger.warning("Joints %s/%s position query failed (%d in a row): %s",
                                   self.side, self.channel, consecutive_errors, sample["reason"])
            if self._json_pub is None and time.monotonic() - self._last_publisher_attempt >= 2.0:
                self._setup_publishers()
            if self._json_pub is not None and not self._stop_event.is_set():
                try:
                    self._publish(sample)
                except Exception as exc:
                    self.publish_error = str(exc)
                    logger.warning("Joints publish failed: %s", exc)
                    self._teardown_publishers()
            elapsed = time.monotonic() - loop_start
            self._stop_event.wait(timeout=max(0.002, 1.0 / self.plugin.poll_hz - elapsed))


class JointsPlugin:
    PREFIX = "joints"
    MODEL_TOOL = "model"

    def __init__(
        self,
        hand: Any,
        config: dict,
        global_config: dict | None = None,
        ros2: Any = None,
        namespace: str = "",
    ):
        self.hand = hand
        self.config = config or {}
        self.global_config = dict(global_config or (hand.config if hand else {}))
        self.hand_config = (getattr(hand, "hand_config", None) if hand is not None else None) or \
            self.global_config.get("plugins", {}).get("hand", {})
        self._ros2 = ros2
        self.namespace = (namespace or "").strip("/")

        self.default_side = str(self.global_config.get("side", "left"))
        self.default_channel = str(self.global_config.get("can_channel", "can0"))
        if self.default_side not in ("left", "right"):
            raise ValueError("side 必须是 left 或 right")
        if not _valid_channel(self.default_channel):
            raise ValueError("can_channel 必须是有效的 CAN 接口名称")
        self.left_can_id = int(self.global_config.get("left_can_id", LEFT_HAND_CAN_ID))
        self.right_can_id = int(self.global_config.get("right_can_id", RIGHT_HAND_CAN_ID))

        self.poll_hz = _number(self.config.get("poll_hz", 10.0), "poll_hz", 1, 30)
        self.query_timeout_s = _number(self.config.get("query_timeout_s", 0.2), "query_timeout_s", 0.02, 2)
        self.stale_after_s = _number(self.config.get("stale_after_s", 1.0), "stale_after_s", 0.05, 30)

        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._bindings: dict[str, str] = {}
        self._channels: dict[str, str] = {}
        self._hand_channels: dict[str, dict[str, str]] = {}  # both mode: instance -> side -> channel
        # instance -> (owned (side, channel) targets, shared both-hands skeleton topic or None)
        self._owners: dict[str, tuple[tuple[tuple[str, str], ...], str | None]] = {}
        self._streams: dict[tuple[str, str], _JointsStream] = {}
        self._used_targets: set[tuple[str, str]] = set()

    # ── topics & definitions ──────────────────────────────────────────

    def can_id(self, side: str) -> int:
        return self.right_can_id if side == "right" else self.left_can_id

    def _adapter(self, channel: str) -> str:
        # Same adapter suffix rule as hand_tactile: can0 paths stay short.
        return "" if channel == self.default_channel else "_can_" + channel.encode().hex()

    def topics(self, side: str, channel: str) -> tuple[str, str]:
        adapter = self._adapter(channel).replace("_can_", "/can_", 1)
        base = f"/{self.namespace}/hand/{side}{adapter}/joints".replace("//", "/")
        return base, base + "_skeleton"

    def dual_topic(self, left_channel: str, right_channel: str) -> str:
        suffix = "".join(f"/{side}{self._adapter(channel)}" for side, channel in
                         (("left", left_channel), ("right", right_channel)) if self._adapter(channel))
        return f"/{self.namespace}/hand/both{suffix}/joints_skeleton".replace("//", "/")

    def topic_out(self, side: str | None = None, channel: str | None = None,
                  targets: tuple[tuple[str, str], ...] | None = None) -> list[dict]:
        side = side or self.default_side
        if side == "both":
            (_, left_channel), (_, right_channel) = targets
            # Each hand keeps its own JSON; one shared skeleton draws both hands.
            return [{"topic": self.topics(s, c)[0], "format": "data/json"} for s, c in targets] + \
                [{"topic": self.dual_topic(left_channel, right_channel), "format": "sensor/skeleton"}]
        json_topic, skeleton_topic = self.topics(side, channel or self.default_channel)
        # data/json first: Core's detail view opens the first output, and its
        # skeleton view cannot load a driver model yet.
        return [{"topic": json_topic, "format": "data/json"},
                {"topic": skeleton_topic, "format": "sensor/skeleton"}]

    def get_tools(self) -> list[dict]:
        schema = action_schema(
            {
                "info": ([], "获取关节卡片信息、当前绑定、实测刷新率及 topic_out"),
                "start": ([], "启动关节位置后台连续采集，同时输出 JSON 与骨架"),
                "stop": ([], "停止本卡片的关节数据流"),
                "config": ([], "按提供的字段更新实例左右手／CAN 接口或共享采集频率"),
            },
            {},
        )
        definition = tool(
            self.PREFIX,
            "sensor",
            "LinkerHand L20 16 主动关节实测位置流：原始值、按厂家 L20-10 标定表换算的角度与实时骨架",
            schema,
            topic_out=self.topic_out(),
        )
        definition["title"] = "L20 关节状态"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "bound_side": {
                    "type": "string",
                    "enum": ["left", "right", "both"],
                    "default": self.default_side,
                    "scope": "instance",
                    "title": "采集左右手",
                    "description": "left／right 只读一只手；both 为双手模式，一张卡片同时读两只手，骨架画在同一个窗口。",
                },
                "can_channel": {
                    "type": "string",
                    "default": self.default_channel,
                    "scope": "instance",
                    "title": "高级：CAN 接口",
                    "description": "与控制卡片使用同一个接口，例如 can0 或 can1。不是 CAN ID。双手模式下是两只手的默认接口。",
                },
                "left_can_channel": {
                    "type": "string",
                    "scope": "instance",
                    "title": "双手模式：左手 CAN 接口",
                    "description": "仅双手模式使用；留空则沿用上面的 CAN 接口。",
                },
                "right_can_channel": {
                    "type": "string",
                    "scope": "instance",
                    "title": "双手模式：右手 CAN 接口",
                    "description": "仅双手模式使用；留空则沿用上面的 CAN 接口。",
                },
                "poll_hz": {
                    "type": "number",
                    "default": self.poll_hz,
                    "minimum": 1.0,
                    "maximum": 30.0,
                    "title": "目标采集频率（共享，实测值见 measured_hz）",
                },
            },
        }
        model_tool = {
            "name": self.MODEL_TOOL,
            "type": "resource",
            "multiInstance": False,
            "description": "LinkerHand L20 左右手运动学 URDF（厂家模型，供骨架显示）",
            "inputSchema": {"type": "object", "properties": {}},
        }
        return [definition, model_tool]

    # ── lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        # Acquisition starts only when a card is started, never at driver boot.
        pass

    def stop(self) -> None:
        with self._lifecycle_lock:
            for stream in list(self._streams.values()):
                try:
                    stream.stop()
                except Exception as exc:
                    logger.warning("Joints stream %s/%s did not stop: %s", stream.side, stream.channel, exc)
            self._streams.clear()
            self._owners.clear()
            self._close_unshared_controllers()

    def _close_unshared_controllers(self) -> None:
        """Close CAN clients opened only for this card; others belong to the hand card."""
        for target in list(self._used_targets):
            self._close_if_unused(target)

    def _close_if_unused(self, target: tuple[str, str]) -> None:
        """Close the CAN client of a hand no worker of this card reads any more.

        Every other card reaches the shared controller through the hand card,
        which registers it under its lock before calling get(); checking under
        that lock means no other card is holding the client being closed, and a
        later get() simply reopens it.
        """
        if target not in self._used_targets or target in self._streams:
            return
        hand_lock = getattr(self.hand, "_lock", None) or contextlib.nullcontext()
        with hand_lock, GLOBAL_BUS_LOCK:
            self._used_targets.discard(target)
            if target in (getattr(self.hand, "_controllers", None) or ()):
                return
            ctrl = GLOBAL_CONTROLLERS.get(target)
            if ctrl is not None:
                ctrl.close()

    def _get_controller(self, side: str, channel: str):
        # Not registered on the hand card: its shutdown releases torque on every
        # hand it knows, which a read-only card must not cause.
        ctrl = get_shared_controller(self.hand_config, side, channel,
                                     can_id=self.can_id(side), bus_lock=GLOBAL_BUS_LOCK)
        self._used_targets.add((side, channel))
        return ctrl

    # ── instance ownership ────────────────────────────────────────────

    def _resolve_target(self, args: dict) -> tuple[str, str, tuple[tuple[str, str], ...]]:
        """(bound_side, can_channel, owned targets); both mode owns one target per hand."""
        instance_id = str(args.get("instance_id", ""))
        with self._lock:
            side = args.get("bound_side") or self._bindings.get(instance_id) or self.default_side
            channel = args.get("can_channel") or self._channels.get(instance_id) or self.default_channel
            stored = self._hand_channels.get(instance_id, {})
            # A per-hand field sent empty falls back to can_channel, not to an older value.
            hand_channels = {hand: (args[f"{hand}_can_channel"] if f"{hand}_can_channel" in args
                                    else stored.get(hand)) or channel for hand in ("left", "right")}
        if side not in ("left", "right", "both"):
            raise ValueError("bound_side 必须是 left、right 或 both")
        for value in (channel, *hand_channels.values()):
            if not _valid_channel(value):
                raise ValueError("can_channel 必须是有效的 CAN 接口名称")
        if side == "both":
            return side, channel, (("left", hand_channels["left"]), ("right", hand_channels["right"]))
        return side, channel, ((side, channel),)

    @staticmethod
    def _describe(targets) -> str:
        return " + ".join(f"{side}/{channel}" for side, channel in targets)

    def _select(self, instance_id: str, side: str, channel: str, targets: tuple[tuple[str, str], ...]) -> None:
        """One worker per physical hand; cards on the same hand share it."""
        with self._lifecycle_lock:
            dual = self.dual_topic(targets[0][1], targets[1][1]) if side == "both" else None
            owner = (targets, dual)
            current = self._owners.get(instance_id)
            if not instance_id and current is not None and current != owner:
                raise ValueError(
                    f"默认关节流为单实例（当前已绑定 {self._describe(current[0])}）；"
                    f"启动或切换不同目标 ({self._describe(targets)}) 必须指定非空 instance_id"
                )
            created = []
            try:
                for target in targets:
                    stream = self._streams.get(target)
                    if stream is None:
                        stream = _JointsStream(self, *target)
                        self._streams[target] = stream
                        created.append(target)
                    stream.start()
            except Exception:
                for target in created:
                    try:
                        self._streams.pop(target).stop()
                    except Exception:
                        pass
                raise
            self._owners[instance_id] = owner
            with self._lock:
                self._bindings[instance_id] = side
                # Both modes: an ID-only info/config later resolves the same interface.
                self._channels[instance_id] = channel
            if current is not None:
                for target in current[0]:
                    if target not in targets:
                        self._release_target(target)
            self._sync_dual_topics()

    def _release_target(self, target: tuple[str, str]) -> None:
        if any(target in targets for targets, _ in self._owners.values()):
            return
        stream = self._streams.get(target)
        if stream is not None:
            stream.stop()
            self._streams.pop(target, None)
        self._close_if_unused(target)

    def _sync_dual_topics(self) -> None:
        for target, stream in self._streams.items():
            stream.set_dual_topics({dual for targets, dual in self._owners.values() if dual and target in targets})

    def _stop_instance(self, instance_id: str) -> None:
        with self._lifecycle_lock:
            owner = self._owners.pop(instance_id, None)
            if owner is None:
                return
            try:
                for target in owner[0]:
                    self._release_target(target)
            except Exception:
                # A worker still blocked in a query keeps its card's ownership,
                # so a retried stop reaches it instead of leaving it orphaned.
                self._owners[instance_id] = owner
                raise
            finally:
                self._sync_dual_topics()

    # ── MCP dispatcher ────────────────────────────────────────────────

    def _sample_view(self, sample: dict[str, Any]) -> dict[str, Any]:
        """Cached sample with its age; too old means not usable as current."""
        if not sample:
            return {"sample_state": "waiting", "sample_valid": False, "sample_age_s": None}
        age = max(0.0, time.time() - sample["published_at"])
        valid = sample["valid"] and age <= self.stale_after_s
        return {
            "sample_state": "ok" if valid else "stale" if sample["valid"] else "error",
            "sample_valid": valid, "sample_age_s": age,
            "sample_sequence": sample["sequence"], "sample_reason": sample["reason"],
            "measured_hz": sample["measured_hz"],
        }

    def _hand_info(self, side: str, channel: str) -> dict:
        with self._lifecycle_lock:
            stream = self._streams.get((side, channel))
        latest = dict(stream.latest) if stream is not None else {}
        return {
            "state": "running" if stream is not None and stream.running() else "idle",
            "bound_side": side,
            "can_channel": channel,
            "can_id": hex(self.can_id(side)),
            "topic_out": self.topic_out(side, channel),
            "poll_hz": self.poll_hz,
            "query_timeout_s": self.query_timeout_s,
            "angle_source": ANGLE_SOURCE,
            "publisher_ready": stream is not None and stream.publisher_ready(),
            "subscriber_count": stream.subscriber_count() if stream is not None else 0,
            "published_json_frames": stream.published_json if stream is not None else 0,
            "published_skeleton_frames": stream.published_skeleton if stream is not None else 0,
            "last_publish_time": stream.last_publish_time if stream is not None else None,
            "publish_error": stream.publish_error if stream is not None else None,
            **self._sample_view(latest),
        }

    def _hand_state(self, side: str, channel: str) -> dict:
        with self._lifecycle_lock:
            stream = self._streams.get((side, channel))
        if stream is not None and stream.running():
            # Reuse the stream's rounds instead of adding a second query.
            latest = dict(stream.latest)
            view = self._sample_view(latest)
            if not latest:
                latest = stream.waiting_sample()
            elif not view["sample_valid"]:
                latest.update(valid=False, state="error",
                              reason=latest["reason"] or f"最近一帧已超过 {self.stale_after_s}s，不作为当前值")
        else:
            with self._lifecycle_lock:
                latest = _JointsStream(self, side, channel).read()
                # A one-off read must not leave the CAN client open behind it.
                self._close_if_unused((side, channel))
            view = self._sample_view(latest)
        return {**latest, **view, "topic_out": self.topic_out(side, channel)}

    def _target_fields(self, side: str, channel: str, targets) -> dict:
        fields = {"bound_side": side, "can_channel": channel}
        if side == "both":
            fields.update({f"{hand}_can_channel": ch for hand, ch in targets})
        fields["topic_out"] = self.topic_out(side, channel, targets)
        return fields

    def _both(self, per_hand: dict[str, dict], key: str) -> str:
        """Worst per-hand state, so one dead hand is never hidden by the other."""
        states = [hand[key] for hand in per_hand.values()]
        for state in ("error", "stale", "waiting"):
            if state in states:
                return state
        return states[0]

    def _remember_hand_channels(self, instance_id: str, args: dict) -> None:
        # Stored as sent: an empty value keeps following can_channel.
        for hand in ("left", "right"):
            if f"{hand}_can_channel" in args:
                self._hand_channels.setdefault(instance_id, {})[hand] = args[f"{hand}_can_channel"]

    def dispatch(self, action: str, args: dict) -> dict:
        if args.get("_tool_name") == self.MODEL_TOOL:
            return model()

        instance_id = str(args.get("instance_id", ""))
        side, channel, targets = self._resolve_target(args)
        hand_keys = ("left_can_channel", "right_can_channel")

        if action == "config":
            rate = _number(args.get("poll_hz", self.poll_hz), "poll_hz", 1, 30)
            with self._lock:
                # Core sends shared settings first; they must not create a binding.
                if "bound_side" in args:
                    self._bindings[instance_id] = side
                if "can_channel" in args:
                    self._channels[instance_id] = channel
                self._remember_hand_channels(instance_id, args)
                self.poll_hz = rate
            with self._lifecycle_lock:
                if instance_id in self._owners and any(
                        key in args for key in ("bound_side", "can_channel", *hand_keys)):
                    self._select(instance_id, side, channel, targets)
            return {"state": "configured", **self._target_fields(side, channel, targets), "poll_hz": self.poll_hz}

        if action == "info":
            if side != "both":
                return self._hand_info(side, channel)
            hands = {hand: self._hand_info(hand, ch) for hand, ch in targets}
            return {
                **self._target_fields(side, channel, targets),
                "state": "running" if all(h["state"] == "running" for h in hands.values()) else "idle",
                "poll_hz": self.poll_hz,
                "query_timeout_s": self.query_timeout_s,
                "angle_source": ANGLE_SOURCE,
                "publisher_ready": all(h["publisher_ready"] for h in hands.values()),
                "sample_state": self._both(hands, "sample_state"),
                "sample_valid": all(h["sample_valid"] for h in hands.values()),
                "hands": hands,
            }

        if action == "start":
            with self._lock:
                self._remember_hand_channels(instance_id, args)
            self._select(instance_id, side, channel, targets)
            return self.dispatch("info", args)

        if action == "stop":
            self._stop_instance(instance_id)
            return {"state": "idle", **self._target_fields(side, channel, targets)}

        if action == "get_state":
            if side != "both":
                return {"action": "get_state", **self._hand_state(side, channel)}
            hands = {hand: self._hand_state(hand, ch) for hand, ch in targets}
            return {"action": "get_state", **self._target_fields(side, channel, targets),
                    "valid": all(h["valid"] for h in hands.values()), "hands": hands}

        return {"state": "error", "message": f"Unknown action: {action}"}
