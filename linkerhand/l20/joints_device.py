"""LinkerHand L20 joint feedback card (joints).

One card reads one hand. Every round is one fresh five-frame position query that
feeds one output:

  /{namespace}/hand/{side}/joints_{card}_skeleton  sensor/skeleton  (valid rounds only)

Each card has its own topic suffix (the card's instance id), so two cards on the
same driver never share an output. The CAN interface is matched automatically:
the card claims a free hand of its side among the hands that answer on the local
socketcan interfaces. When no free hand exists the card keeps running in the
`waiting` state: it publishes no skeleton frame and re-probes every couple of
seconds so a hand plugged in later is picked up. The JSON sample is kept for
`info`/`get_state` diagnostics; it is not a public topic.

The card only sends position-query frames: no motion, torque, calibration or
fault clearing. The `model` tool serves a single-hand URDF to a panel that asks
with its topic; without a topic it serves the legacy dual-hand URDF.
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
from pathlib import Path
from typing import Any

from common.vendor_runtime import action_schema, tool
from can_client import JOINT_NAMES_16, L20CanClient, LEFT_HAND_CAN_ID, RIGHT_HAND_CAN_ID
from hand_device import (GLOBAL_BUS_LOCK, GLOBAL_CONTROLLERS, _discover_can_channels,
                         get_shared_controller)
from joints_model import (ANGLE_SOURCE, JOINT_LABELS, RESERVED_VECTOR_INDEXES, SIDE_LABELS, URDF_JOINT,
                          VECTOR_INDEX, model, raw_to_angle, skeleton_joints)


logger = logging.getLogger("linkerhand.joints")

# Fixed acquisition rate: the card exposes no rate setting.
POLL_HZ = 30.0
# A card without a free hand publishes no frames; it only re-probes.
# How often a waiting card looks for a hand that appeared since the last look.
MATCH_RETRY_INTERVAL_S = 2.0
# One probe query waits this long for the hand's position replies.
PROBE_TIMEOUT_S = 0.12
# Results of probes are reused this long, so several waiting cards starting at
# once do not send the same query burst.
PROBE_CACHE_S = 1.5
PRESENT_TTL_S = 3.0


def _number(value, name, lower, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须是 {lower}–{upper} 之间的数值")
    return float(value)


def _slug(instance_id: str) -> str:
    """Topic suffix for one card instance; stable across restarts."""
    return re.sub(r"[^0-9a-z]", "", str(instance_id).lower())[:16] or "card"


def _blank_sample(side: str, channel: str | None, sequence: int, state: str, reason: str,
                  can_id: str | None) -> dict[str, Any]:
    return {
        "state": state, "valid": False,
        "reason": reason,
        "error_details": None,
        "side": side, "bound_side": side, "can_channel": channel,
        "can_id": can_id,
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
            "model_joint": f"{side}_hand_{URDF_JOINT[name]}",
            "raw": None, "angle_rad": None, "angle_deg": None,
        } for name in JOINT_NAMES_16],
    }


class _JointsStream:
    """Background acquisition and publishing for one card (one physical hand).

    `channel` is None while the card waits for a free hand. The same worker then
    publishes invalid frames periodically and asks the plugin for a hand; when
    one is claimed it continues at the fixed rate without being recreated.
    """

    def __init__(self, plugin: "JointsPlugin", instance_id: str, side: str, channel: str | None):
        self.plugin = plugin
        self.instance_id = instance_id
        self.side = side
        self.channel = channel
        self.skeleton_topic = plugin.skeleton_topic(side, instance_id)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._skeleton_pub = None
        self._qos = None
        self._last_publisher_attempt = 0.0
        self._success_times: collections.deque[float] = collections.deque(maxlen=20)
        self.sequence = 0
        self.latest: dict[str, Any] = {}
        self.waiting_reason = ""
        self.published_skeleton = 0
        self.last_publish_time = None
        self.publish_error = None

    # ── sampling ──────────────────────────────────────────────────────

    def read(self) -> dict[str, Any]:
        """One fresh position query, converted into the sample behind the skeleton."""
        assert self.channel is not None
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
                "model_joint": f"{self.side}_hand_{URDF_JOINT[name]}",
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
        if self._skeleton_pub is not None:
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
            node = Node(f"hand_joints_{self.side}_{_slug(self.instance_id)}_{uuid.uuid4().hex[:8]}",
                        context=ros2.ctx_core)
            skeleton_pub = node.create_publisher(String, self.skeleton_topic, qos)
            ros2.executor_core.add_node(node)
            self._node, self._skeleton_pub, self._qos = node, skeleton_pub, qos
            self.publish_error = None
            print(f"[joints] ROS2 publisher ready: {self.skeleton_topic}", flush=True)
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

    def _teardown_publishers(self) -> None:
        node, self._node = self._node, None
        self._skeleton_pub = None
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
        return self._skeleton_pub is not None

    def subscriber_count(self) -> int:
        try:
            return self._skeleton_pub.get_subscription_count() if self._skeleton_pub is not None else 0
        except Exception:
            return 0

    # ── worker ────────────────────────────────────────────────────────

    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def set_channel(self, channel: str) -> None:
        with self._lock:
            self.channel = channel

    def start(self) -> None:
        if self.running():
            if self._stop_event.is_set():
                # The thread is only finishing its last query and then exits;
                # returning here would report a worker that is about to end.
                raise RuntimeError("关节采集上一次停止尚未完成；请稍后重试")
            return
        self._stop_event.clear()
        self._setup_publishers()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name=f"joints_{self.side}_{_slug(self.instance_id)}")
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
        self._teardown_publishers()

    def _publish(self, sample: dict[str, Any]) -> None:
        if not sample["valid"]:
            # Invalid rounds publish no skeleton frame: the renderer has no
            # invalid state and would otherwise show an old pose as live.
            return
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(self.skeleton(sample), ensure_ascii=False, allow_nan=False)
        self._skeleton_pub.publish(msg)
        self.published_skeleton += 1
        self.last_publish_time = time.time()
        self.publish_error = None

    def _loop(self) -> None:
        consecutive_errors = 0
        next_match = 0.0
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            if self.channel is None:
                # Waiting for a free hand: publish nothing, just keep looking
                # for a hand that appeared since the last look.
                now = time.monotonic()
                # stop() joins this worker while holding the lifecycle lock. If a
                # lifecycle change is in progress, retry on the next loop tick.
                if now >= next_match and self.plugin._lifecycle_lock.acquire(blocking=False):
                    try:
                        next_match = now + MATCH_RETRY_INTERVAL_S
                        target = self.plugin._match(self.instance_id, self.side)
                        if target is not None:
                            self.set_channel(target[1])
                            continue
                        self.waiting_reason = self.plugin._match_reason(self.side)
                    finally:
                        self.plugin._lifecycle_lock.release()
                if self._skeleton_pub is None and now - self._last_publisher_attempt >= 2.0:
                    self._setup_publishers()
                self._stop_event.wait(0.05)
                continue
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
            if self._skeleton_pub is None and time.monotonic() - self._last_publisher_attempt >= 2.0:
                self._setup_publishers()
            if self._skeleton_pub is not None and not self._stop_event.is_set():
                try:
                    self._publish(sample)
                except Exception as exc:
                    self.publish_error = str(exc)
                    logger.warning("Joints publish failed: %s", exc)
                    self._teardown_publishers()
            elapsed = time.monotonic() - loop_start
            self._stop_event.wait(timeout=max(0.002, 1.0 / POLL_HZ - elapsed))


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
        if self.default_side not in ("left", "right"):
            raise ValueError("side 必须是 left 或 right")
        self.left_can_id = int(self.global_config.get("left_can_id", LEFT_HAND_CAN_ID))
        self.right_can_id = int(self.global_config.get("right_can_id", RIGHT_HAND_CAN_ID))
        self.baudrate = int(self.global_config.get("baudrate", 1_000_000))

        self.query_timeout_s = _number(self.config.get("query_timeout_s", 0.2), "query_timeout_s", 0.02, 2)
        self.stale_after_s = _number(self.config.get("stale_after_s", 1.0), "stale_after_s", 0.05, 30)

        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._bindings: dict[str, str | None] = {}
        # instance -> the (side, channel) hand it owns; released only when its
        # worker actually stopped, so a stuck worker keeps its hand.
        self._claims: dict[str, tuple[str, str]] = {}
        self._streams: dict[str, _JointsStream] = {}
        # instance -> claim whose worker did not stop; its next stop retries it.
        self._leftovers: dict[str, tuple[str, str] | None] = {}
        self._used_targets: set[tuple[str, str]] = set()
        # Targets whose controller this plugin created. A controller that was
        # already in the global registry belongs to another consumer (hand,
        # gesture, grasp, tactile) and must not be closed here.
        self._created_targets: set[tuple[str, str]] = set()
        self._probe_cache: dict[tuple[str, str], float] = {}
        self._present_cache: dict[tuple[str, str], float] = {}

    # ── topics & definitions ──────────────────────────────────────────

    def can_id(self, side: str) -> int:
        return self.right_can_id if side == "right" else self.left_can_id

    def skeleton_topic(self, side: str, instance_id: str) -> str:
        return f"/{self.namespace}/hand/{side}/joints_{_slug(instance_id)}_skeleton".replace("//", "/")

    def topic_out(self, instance_id: str, side: str | None = None) -> list[dict]:
        side = side or self._bindings.get(instance_id)
        if side not in ("left", "right"):
            return []
        return [{"topic": self.skeleton_topic(side, instance_id), "format": "sensor/skeleton"}]

    def get_tools(self) -> list[dict]:
        # `action_schema` drives the card's action list. `get_state` is an
        # API-only action (same as hand_tactile's) and `instance_id` is injected
        # by the platform, so neither is declared here; the side lives in
        # configSchema and is sent as a separate config call or nested config.
        schema = action_schema(
            {
                "info": ([], "获取关节卡片信息、当前绑定的手、实测刷新率及 topic_out"),
                "start": ([], "启动关节位置后台连续采集，输出实时骨架"),
                "stop": ([], "停止本卡片的关节骨架流"),
                "config": ([], "按提供的字段更新实例选择的左右手"),
            },
            {},
        )
        definition = tool(
            self.PREFIX,
            "sensor",
            "LinkerHand L20 16 主动关节实时骨架（按厂家 L20-10 标定表换算；不输出 JSON 数据流）",
            schema,
            # Per-instance topics depend on the instance id, so the card asks
            # the driver (info) instead of using a static default.
            topic_out=[],
        )
        definition["title"] = "L20 关节骨架"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "bound_side": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "default": self.default_side,
                    "scope": "instance",
                    "title": "选择左手或右手",
                    "description": "一张卡片读一只手；CAN 接口自动匹配，不需要手选。",
                },
            },
        }
        model_tool = {
            "name": self.MODEL_TOOL,
            "type": "resource",
            "multiInstance": False,
            "description": "LinkerHand L20 单手运动学 URDF（厂家模型，供骨架显示）；带 topic 时按话题里的手侧返回",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {
                        "type": "string",
                        "description": "骨架面板所属话题；驱动按 /hand/left|right/ 选择单手模型，缺省返回双手模型。",
                    },
                },
            },
        }
        return [definition, model_tool]

    # ── lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        # Acquisition starts only when a card is started, never at driver boot.
        pass

    def stop(self) -> None:
        with self._lifecycle_lock:
            stuck = []
            for instance_id, stream in list(self._streams.items()):
                target = self._claims.get(instance_id)
                try:
                    stream.stop()
                except Exception as exc:
                    stuck.append(instance_id)
                    self._leftovers[instance_id] = target
                    logger.warning("Joints stream %s did not stop: %s", instance_id, exc)
                    continue
                self._streams.pop(instance_id, None)
                self._claims.pop(instance_id, None)
                self._leftovers.pop(instance_id, None)
                if target is not None:
                    self._close_if_unused(target)
            self._close_unshared_controllers()
            if stuck:
                raise RuntimeError(f"关节采集未能停止：{', '.join(sorted(stuck))}；保留线程，可再次停止")

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
        if any(stream.channel is not None and (stream.side, stream.channel) == target
               for stream in self._streams.values()):
            return
        if target not in self._used_targets:
            return
        hand_lock = getattr(self.hand, "_lock", None) or contextlib.nullcontext()
        with hand_lock, GLOBAL_BUS_LOCK:
            self._used_targets.discard(target)
            # Only a controller this plugin created is ours to close. One that
            # was already registered belongs to another consumer (hand,
            # gesture, grasp, tactile) and may still be in use; closing it or
            # dropping its registry entry would split that hand's replies.
            if target not in self._created_targets:
                return
            self._created_targets.discard(target)
            if target in (getattr(self.hand, "_controllers", None) or ()):
                return
            ctrl = GLOBAL_CONTROLLERS.get(target)
            if ctrl is not None:
                ctrl.close()
                # A closed controller must not keep making the hand look
                # occupied: the next card has to probe the hand again.
                if GLOBAL_CONTROLLERS.get(target) is ctrl:
                    del GLOBAL_CONTROLLERS[target]

    def _get_controller(self, side: str, channel: str):
        # Not registered on the hand card: its shutdown releases torque on every
        # hand it knows, which a read-only card must not cause.
        target = (side, channel)
        existed = target in GLOBAL_CONTROLLERS
        ctrl = get_shared_controller(self.hand_config, side, channel,
                                     can_id=self.can_id(side), bus_lock=GLOBAL_BUS_LOCK,
                                     request_metadata=False)
        self._used_targets.add(target)
        if not existed:
            self._created_targets.add(target)
        return ctrl

    # ── CAN matching ──────────────────────────────────────────────────

    def _channels(self) -> list[str]:
        return _discover_can_channels()

    @staticmethod
    def _channel_usable(channel: str) -> bool:
        """Skip an adapter that reports no bus (a CAN transceiver with nothing on it).

        can0 on this board is an unwired mttcan controller: it is up but has no
        carrier, and probing it every retry would only add timeouts. Boards that
        do wire it report carrier 1 once the hand answers.
        """
        try:
            return Path(f"/sys/class/net/{channel}/carrier").read_text().strip() == "1"
        except OSError:
            return True

    def _controller_responds(self, side: str, channel: str) -> bool:
        """Ask an existing shared controller whether its hand still answers.

        The registry entry alone is not evidence: an actuator may have opened a
        controller for a hand that was unplugged afterwards. Query through that
        controller's own client (its position-query lock is shared with the
        actuator), so a second client never reads the same replies.
        """
        with GLOBAL_BUS_LOCK:
            ctrl = GLOBAL_CONTROLLERS.get((side, channel))
        if ctrl is None:
            return False
        try:
            return bool(ctrl.get().probe_positions(PROBE_TIMEOUT_S))
        except Exception as exc:
            logger.debug("Shared controller %s/%s did not answer: %s", side, channel, exc)
            return False

    def _probe(self, side: str, channel: str) -> bool:
        """One short position query: does a hand of this side answer here?

        Never called for a (side, channel) that has a live controller or worker:
        two clients on one hand would read each other's replies.
        """
        target = (side, channel)
        now = time.monotonic()
        if self._present_cache.get(target, 0.0) > now - PRESENT_TTL_S:
            return True
        last = self._probe_cache.get(target)
        if last is not None and now - last < PROBE_CACHE_S:
            return False
        self._probe_cache[target] = now
        client = None
        try:
            client = L20CanClient(channel, side, baudrate=self.baudrate, can_id=self.can_id(side),
                                  recv_timeout_s=PROBE_TIMEOUT_S + 0.08, request_metadata=False)
            time.sleep(0.02)
            alive = bool(client.probe_positions(PROBE_TIMEOUT_S))
        except Exception as exc:
            logger.debug("Probe %s/%s failed: %s", side, channel, exc)
            alive = False
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        if alive:
            self._present_cache[target] = time.monotonic()
        return alive

    def _match(self, instance_id: str, side: str) -> tuple[str, str] | None:
        """Claim a free hand of this side, or None; caller holds no lock."""
        with self._lifecycle_lock:
            taken = {target for owner, target in self._claims.items() if owner != instance_id}
            for channel in self._channels():
                target = (side, channel)
                if target in taken or not self._channel_usable(channel):
                    continue
                ctrl = GLOBAL_CONTROLLERS.get(target)
                if ctrl is not None:
                    if not self._controller_responds(side, channel):
                        continue
                elif not self._probe(side, channel):
                    continue
                self._claims[instance_id] = target
                return target
            return None

    def _match_reason(self, side: str) -> str:
        label = SIDE_LABELS.get(side, side)
        channels = [channel for channel in self._channels() if self._channel_usable(channel)]
        taken = sorted(channel for owner_side, channel in self._claims.values() if owner_side == side)
        if taken:
            return f"没有空闲的{label}：{', '.join(taken)} 已被其他卡片占用"
        where = "、".join(channels) if channels else "任何 CAN 接口"
        return f"未在 {where} 上找到{label}"

    # ── instance ownership ────────────────────────────────────────────

    @staticmethod
    def _flatten_instance_config(args: dict) -> dict:
        """Accept the card's saved instance config both top-level and nested.

        Some platform paths send the saved gear settings as
        `arguments.config = {bound_side: ...}`; the canvas start path sends
        them as a separate `config` call. Top-level values win.
        """
        nested = args.get("config")
        if not isinstance(nested, dict):
            return args
        merged = dict(nested)
        merged.update(args)
        return merged

    def _reject_topic_collision(self, instance_id: str, side: str) -> None:
        """Two instances that normalize to the same suffix and side would clash.

        Distinct card IDs such as `left-card-a` and `left_card_a` both become
        `leftcarda`, so their JSON and skeleton topics would be identical while
        each card owns its own hand. Refuse the second card instead of letting
        two producers interleave frames on one topic.
        """
        slug = _slug(instance_id)
        for other, stream in self._streams.items():
            if other != instance_id and stream.side == side and _slug(other) == slug:
                raise ValueError(
                    f"topic 后缀冲突：卡片 {other} 与 {instance_id} 都归一化为 “{slug}”；"
                    f"请删除其中一张卡片后重新添加"
                )

    def _sideless(self, args: dict, instance_id: str) -> str | None:
        """Side chosen by the request or remembered for this instance."""
        if "bound_side" in args:
            value = args.get("bound_side")
            return value if value in ("left", "right") else None
        with self._lock:
            return self._bindings.get(instance_id)

    def _select(self, instance_id: str, side: str) -> None:
        """Make this card read `side`, claiming a free hand or waiting for one."""
        with self._lifecycle_lock:
            stream = self._streams.get(instance_id)
            if stream is not None:
                if stream.running() and stream.side == side:
                    if stream.channel is None:
                        target = self._match(instance_id, side)
                        if target is not None:
                            stream.set_channel(target[1])
                    return
                # A side change, or a worker that already ended without being
                # released: stop it and start clean instead of overwriting it.
                self._stop_stream(instance_id)
            self._reject_topic_collision(instance_id, side)
            target = self._match(instance_id, side)
            stream = _JointsStream(self, instance_id, side, None if target is None else target[1])
            self._streams[instance_id] = stream
            try:
                stream.start()
            except Exception:
                self._streams.pop(instance_id, None)
                self._claims.pop(instance_id, None)
                raise

    def _stop_stream(self, instance_id: str) -> None:
        """Stop one card's worker; a worker that does not stop keeps its claim."""
        with self._lifecycle_lock:
            stream = self._streams.get(instance_id)
            if stream is None:
                self._claims.pop(instance_id, None)
                self._leftovers.pop(instance_id, None)
                return
            target = self._claims.get(instance_id)
            try:
                stream.stop()
            except Exception:
                self._leftovers[instance_id] = target
                raise
            self._streams.pop(instance_id, None)
            self._claims.pop(instance_id, None)
            self._leftovers.pop(instance_id, None)
            if target is not None:
                self._close_if_unused(target)

    def _stop_instance(self, instance_id: str) -> None:
        with self._lifecycle_lock:
            self._stop_stream(instance_id)

    # ── MCP dispatcher ────────────────────────────────────────────────

    def _sample_view(self, sample: dict[str, Any]) -> dict[str, Any]:
        """Cached sample with its age; too old means not usable as current."""
        if not sample:
            return {"sample_state": "waiting", "sample_valid": False, "sample_age_s": None,
                    "measured_hz": None}
        if sample.get("state") == "waiting":
            return {"sample_state": "waiting", "sample_valid": False, "sample_age_s": None,
                    "sample_sequence": sample.get("sequence"), "sample_reason": sample.get("reason"),
                    "measured_hz": None}
        age = max(0.0, time.time() - sample["published_at"])
        valid = sample["valid"] and age <= self.stale_after_s
        return {
            "sample_state": "ok" if valid else "stale" if sample["valid"] else "error",
            "sample_valid": valid, "sample_age_s": age,
            "sample_sequence": sample["sequence"], "sample_reason": sample["reason"],
            "measured_hz": sample["measured_hz"],
        }

    def _info(self, instance_id: str) -> dict:
        with self._lifecycle_lock:
            side = self._bindings.get(instance_id)
            stream = self._streams.get(instance_id)
            claim = self._claims.get(instance_id)
            leftover = self._leftovers.get(instance_id)
        if side not in ("left", "right"):
            return {
                "state": "unset", "instance_id": instance_id, "bound_side": None,
                "can_channel": None, "can_id": None, "topic_out": [],
                "poll_hz": POLL_HZ, "query_timeout_s": self.query_timeout_s,
                "angle_source": ANGLE_SOURCE,
                "publisher_ready": False, "subscriber_count": 0,
                "published_skeleton_frames": 0,
                "last_publish_time": None, "publish_error": None,
                "sample_state": "unset", "sample_valid": False, "sample_age_s": None,
                "measured_hz": None, "reason": "请选择左手或右手",
            }
        latest = dict(stream.latest) if stream is not None else {}
        view = self._sample_view(latest)
        channel = claim[1] if claim is not None else (stream.channel if stream is not None else None)
        running = stream is not None and stream.running()
        state = "running" if running and channel is not None else "waiting" if running else "idle"
        return {
            "state": state,
            "instance_id": instance_id,
            "bound_side": side,
            "can_channel": channel,
            "can_id": hex(self.can_id(side)),
            "topic_out": self.topic_out(instance_id, side),
            "poll_hz": POLL_HZ,
            "query_timeout_s": self.query_timeout_s,
            "angle_source": ANGLE_SOURCE,
            "publisher_ready": stream is not None and stream.publisher_ready(),
            "subscriber_count": stream.subscriber_count() if stream is not None else 0,
            "published_skeleton_frames": stream.published_skeleton if stream is not None else 0,
            "last_publish_time": stream.last_publish_time if stream is not None else None,
            "publish_error": stream.publish_error if stream is not None else None,
            "waiting_reason": stream.waiting_reason if state == "waiting" and stream is not None else None,
            "leftover_claim": leftover[1] if leftover else None,
            "measured_hz": latest.get("measured_hz"),
            **view,
        }

    def _get_state(self, instance_id: str) -> dict:
        """Latest measured frame; never queries CAN while the card is idle."""
        with self._lifecycle_lock:
            side = self._bindings.get(instance_id)
            stream = self._streams.get(instance_id)
            claim = self._claims.get(instance_id)
        if side not in ("left", "right"):
            return {"state": "unset", "valid": False, "reason": "请选择左手或右手",
                    "topic_out": [], "sample_state": "unset", "sample_valid": False}
        channel = claim[1] if claim is not None else None
        if stream is None or not stream.running():
            return {**_blank_sample(side, channel, 0, "idle", "关节采集未启动；本帧不含实测值",
                                    hex(self.can_id(side))),
                    "sample_state": "idle", "sample_valid": False, "sample_age_s": None,
                    "topic_out": self.topic_out(instance_id, side)}
        latest = dict(stream.latest)
        view = self._sample_view(latest)
        if not latest:
            # First round still running: report waiting without bumping the
            # published sequence from inside an info call.
            latest = _blank_sample(side, channel, stream.sequence, "waiting",
                                   "关节采集已启动，第一轮位置查询尚未完成",
                                   hex(self.can_id(side)))
            view = self._sample_view(latest)
        elif not view["sample_valid"] and view["sample_state"] != "waiting":
            latest.update(valid=False, state="error",
                          reason=latest["reason"] or f"最近一帧已超过 {self.stale_after_s}s，不作为当前值")
        return {**latest, **view, "topic_out": self.topic_out(instance_id, side)}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if args.get("_tool_name") == self.MODEL_TOOL:
            return model(args.get("topic"))

        args = self._flatten_instance_config(args)
        instance_id = str(args.get("instance_id", ""))
        side = self._sideless(args, instance_id)

        if action == "config":
            with self._lock:
                if "bound_side" in args:
                    self._bindings[instance_id] = side
            # Legacy per-card fields (can_channel, left/right_can_channel,
            # poll_hz) are ignored: the interface is matched, the rate is fixed.
            with self._lifecycle_lock:
                stream = self._streams.get(instance_id)
                if stream is not None:
                    if side in ("left", "right"):
                        self._select(instance_id, side)
                    else:
                        self._stop_stream(instance_id)
                elif side in ("left", "right"):
                    self._bindings[instance_id] = side
            return {"state": "configured", "instance_id": instance_id,
                    "bound_side": side,
                    "can_channel": self._claims.get(instance_id, (None, None))[1],
                    "topic_out": self.topic_out(instance_id, side),
                    "poll_hz": POLL_HZ}

        if action == "info":
            return self._info(instance_id)

        if action == "start":
            with self._lock:
                self._bindings[instance_id] = side
            if side in ("left", "right"):
                self._select(instance_id, side)
            else:
                self._stop_instance(instance_id)
            return self._info(instance_id)

        if action == "stop":
            self._stop_instance(instance_id)
            result = self._info(instance_id)
            result["state"] = "idle"
            return result

        if action == "get_state":
            return self._get_state(instance_id)

        # Plugin contract (README_dev.md): None, which the MCP handler reports
        # as an error instead of wrapping it as a successful result.
        return None
