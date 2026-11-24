"""ROS/Canvas adapter for explicit, renewable Bumi Sound Facing sessions."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict

from sound_facing import FacingConfig, SoundFacing
from sound_facing_authority import OperatorAuthority


class SoundFacingPlugin:
    PREFIX = "bumi_sound_facing"

    def __init__(self, plugin_config, namespace, executor, loco_plugin, *, source_plugin=None,
                 authority=None):
        self._ns, self._executor = namespace, executor
        self._source = source_plugin
        settings = {k: v for k, v in (plugin_config or {}).items() if k != "enabled"}
        self._engine = SoundFacing(loco_plugin, settings, source_health=self._source_healthy,
                                  authority=authority if authority is not None else OperatorAuthority(namespace),
                                  device=namespace)
        self._lifecycle = threading.RLock()
        self._node = None
        self._worker = None
        self._halt = threading.Event()
        self._input_topic = f"/{namespace}/sound_direction"
        self._state_topic = f"/{namespace}/bumi_sound_facing/state"

    def _source_healthy(self):
        if self._source is None:
            return False
        info = self._source.dispatch("info", {})
        return (info.get("state") == "running"
                and {"front", "right"} <= set(info.get("calibrated_directions", [])))

    def get_tool(self):
        defaults = asdict(FacingConfig())
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Bumi 持续听声转身：需独立操作员签名授权，Agent 可在有效授权内 enable_follow/renew_follow。start 不启用；停止不需授权。",
            "inputSchema": {"type": "object", "additionalProperties": False,
                "required": ["action"],
                "allOf": [{"if": {"properties": {"action": {"const": "renew_follow"}}},
                           "then": {"required": ["session_id"]}}],
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "stop", "info", "prepare_authorization", "authorize_follow",
                        "enable_follow", "renew_follow", "disable_follow", "cancel", "config"]},
                    "input_topic": {"type": "string"},
                    "input_topics": {"type": "array", "items": {"type": "string"}, "minItems": 1, "maxItems": 1},
                    "session_id": {"type": "string", "description": "renew_follow 必须匹配当前启用会话；用于关联，不是访问凭据。"},
                    "authorization": {"type": "string", "maxLength": 4096,
                                      "description": "离机操作员签名 JSON；不是布尔确认或 session_id。"},
                    **FacingConfig.schema_properties(),
                },
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "开始只读订阅，不启用追踪。"},
                    "prepare_authorization": {"params": [], "description": "停用且释放后创建新授权请求；不授权或运动。"},
                    "authorize_follow": {"params": ["authorization"], "required": ["authorization"],
                                         "description": "验证操作员离机签名；不启动运动。"},
                    "enable_follow": {"params": [], "dangerous": True,
                                      "description": "仅在有效操作员授权内启用；会话续期不能延长操作员授权。"},
                    "renew_follow": {"params": ["session_id"], "required": ["session_id"],
                                     "description": "在到期前续期当前会话，不能恢复过期或故障会话。"},
                    "disable_follow": {"params": [], "description": "停止追踪，只取消本卡的转身；收尾期间保留身体占用。"},
                    "cancel": {"params": [], "description": "无需授权，取消本卡会话与运动。"},
                    "stop": {"params": [], "description": "取消本卡动作并释放本卡 ROS 资源。"},
                    "info": {"params": [], "description": "读取状态与拒绝原因。"},
                    "config": {"params": list(defaults), "description": "调整筛选参数；变更会结束追踪会话。"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "disable_follow"}},
                # enable_follow returns a session state, not a finite action.
                # Internal turns consume local receipts, never orphan ACP IDs.
            },
            "configSchema": {"type": "object", "additionalProperties": False,
                "properties": FacingConfig.schema_properties()},
            "topic_in": [{"topic": self._input_topic, "msg_type": "std_msgs/msg/String",
                          "format": "sensor/sound-direction", "description": "现有声源方向观测"}],
            "topic_out": [{"topic": self._state_topic, "msg_type": "std_msgs/msg/String",
                           "format": "data/json", "description": "追踪状态与动作结果；不代表物理验收"}],
        }

    def start(self):
        # Bundle boot creates only this card's read-only resources.
        with self._lifecycle:
            if self._node is not None:
                return True
            if self._worker is not None and self._worker.is_alive():
                return False
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import String

            node = Node(f"{self._ns}_bumi_sound_facing")
            try:
                self._publisher = node.create_publisher(String, self._state_topic, 1)
                node.create_subscription(String, self._input_topic, self._on_direction, qos_profile_sensor_data)
                node.create_timer(0.2, self._publish)
                self._executor.add_node(node)
                self._node = node
                self._halt.clear()
                self._engine.start()
                self._worker = threading.Thread(target=self._supervise, name="bumi_sound_facing_watchdog", daemon=True)
                self._worker.start()
            except Exception:
                self._engine.running = False
                self._node = None
                self._executor.remove_node(node)
                node.destroy_node()
                raise
            return True

    def _supervise(self):
        # Supervision only. All velocity output remains in LocoPlugin's worker.
        while not self._halt.wait(0.05):
            self._engine.tick()

    def _on_direction(self, msg):
        try:
            # A malformed/oversized message invalidates evidence, not the node.
            data = json.loads(msg.data) if len(msg.data) <= 4096 else None
        except (ValueError, TypeError):
            data = None
        self._engine.observe(data)

    def _publish(self):
        with self._lifecycle:
            if self._node is not None:
                from std_msgs.msg import String
                self._publisher.publish(String(data=json.dumps(self._engine.snapshot(), allow_nan=False)))

    def stop(self):
        self._engine.interrupt()  # signal before lifecycle waits or cleanup
        with self._lifecycle:
            self._engine.disable()
            self._engine.running = False
            self._halt.set()
            if self._worker is not None:
                self._worker.join(timeout=1)
            # Bounded drain of the EXISTING worker, including its final zero.
            # An unconfirmed stop retains the reservation and blocks enable.
            deadline = time.monotonic() + 6
            while self._engine.reserved and time.monotonic() < deadline:
                self._engine.tick()
                time.sleep(0.01)
            if self._engine.reserved:
                with self._engine.lock:
                    self._engine.state, self._engine.reason = "fault", "owned_stop_unconfirmed"
            if self._node is not None:
                node, self._node = self._node, None
                self._executor.remove_node(node)
                node.destroy_node()
            return self._engine.snapshot()

    def dispatch(self, action, args):
        if action in ("stop", "disable_follow", "cancel"):
            self._engine.interrupt()  # safety is independent of both locks/auth
        # Core adds trace/card IDs even for single-instance cards. Neither is
        # an owner capability or a selector for a different physical device.
        if "instance_id" in args and not isinstance(args["instance_id"], str):
            return {"state": "error", "error": "invalid_instance_id"}
        args = {k: v for k, v in args.items()
                if k not in ("_tool_name", "_trace_id", "instance_id", "action")}
        defaults = asdict(FacingConfig())
        allowed = (set(defaults) if action == "config" else
                   {"input_topic", "input_topics"} if action == "start" else
                   {"authorization"} if action == "authorize_follow" else
                   {"session_id"} if action == "renew_follow" else set())
        if set(args) - allowed:
            return {"state": "error", "error": "unknown_parameters"}
        # External probes / crypto never hold the ROS lifecycle lock.
        if action == "enable_follow":
            return self._engine.enable()
        if action == "renew_follow":
            return self._engine.renew(args.get("session_id"))
        if action == "prepare_authorization":
            return self._engine.prepare_authorization()
        if action == "authorize_follow":
            return self._engine.authorize(args.get("authorization"))
        if action in ("disable_follow", "cancel"):
            return self._engine.disable()
        with self._lifecycle:
            if action == "start":
                topic = args.get("input_topic", self._input_topic)
                topics = args.get("input_topics", [self._input_topic])
                if topic != self._input_topic or topics != [self._input_topic]:
                    return {"state": "error", "error": "requires_bundle_sound_direction_topic"}
                if not self.start():
                    return {"state": "error", "error": "supervisor_not_stopped"}
                return self._engine.snapshot()
            if action == "stop":
                return self.stop()
            if action == "info":
                return self._engine.snapshot()
            if action == "config":
                try:
                    return self._engine.configure(args)
                except ValueError as exc:
                    return {"state": "error", "error": str(exc)}
        return None
