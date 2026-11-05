"""Franka FR3 cards sharing one physical session per robot IP."""

from __future__ import annotations

import json
import logging
import math
import re
import threading
import time
from pathlib import Path

from common.vendor_runtime import action_schema, tool
from discovery import Discovery
from hardware import JOINT_LOWER, JOINT_UPPER, JOINT_MARGIN, SessionManager, canonical_ip


JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
JOINT_FORMAT = "sensor/skeleton"
log = logging.getLogger(__name__)


class RosPublisher:
    """One independently stoppable DDS publisher per canvas card."""

    def __init__(self, ros2, topic, hz, callback):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._ros2 = ros2
        self._message = String
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
        )
        self._node = Node("franka_" + topic.strip("/").replace("/", "_"), context=ros2.ctx_core)
        try:
            self._publisher = self._node.create_publisher(String, topic, qos)
            self._node.create_timer(1.0 / hz, callback)
            ros2.executor_core.add_node(self._node)
        except Exception:
            self._node.destroy_node()
            self._node = None
            raise

    def publish(self, payload):
        message = self._message()
        message.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self._publisher.publish(message)

    def close(self):
        node, self._node = self._node, None
        if node is not None:
            try:
                self._ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()


class _Instance:
    def __init__(self):
        self.lock = threading.RLock()
        self.selection = {}
        self.config = {}
        self.session = None
        self.publisher = None
        self.publish_error = None
        self.start_epoch = 0


class FrankaPlugin:
    """Canvas instances are consumers; sessions own physical connections."""

    PUBLISH_STATE = True
    PREFIX = "joint_states"
    CONFIG_FIELDS = ("robot_ip", "manual_robot_ip", "publish_hz")

    def __init__(self, config, namespace="franka", ros2=None, *, manager=None,
                 publisher_factory=None, discovery=None):
        self._namespace = namespace.strip("/") or "franka"
        self._manager = manager if manager is not None else SessionManager()
        self._publisher_factory = publisher_factory or (
            lambda topic, hz, callback: RosPublisher(ros2, topic, hz, callback)
        )
        self._lock = threading.RLock()
        self._instances = {}
        self._defaults = {}
        self._closed = False
        self._started = False
        self._discovery = discovery if discovery is not None else Discovery()
        resource = Path(__file__).with_name("resource") / "fr3.urdf"
        self._urdf = resource.read_text(encoding="utf-8")

    def get_tools(self):
        with self._lock:
            configs = [self._validate(self._defaults),
                       *(instance.config for instance in self._instances.values())]
            configured_ips = {cfg["robot_ip"] for cfg in configs if cfg.get("robot_ip")}
            can_scan = self._started and not self._closed and not any(
                instance.session is not None for instance in self._instances.values())
        if can_scan:
            self._discovery.refresh()
        discovery = self._discovery.snapshot()
        options = {robot["ip"]: robot["label"] for robot in discovery["robots"]}
        for ip in sorted(configured_ips):
            options.setdefault(ip, f"已配置 · {ip}（暂未发现）")
        ip_schema = {
            "type": "string", "title": "机械臂", "scope": "instance", "x-sensitive": True,
            "description": "自动发现有线网络中的 FR3，按序列号和 IP 选择。原设备不在列表时，请手动填回其 IP 后再保存。",
            "oneOf": [{"const": ip, "title": label} for ip, label in options.items()]
                     + [{"const": "__manual__", "title": "手动输入 IP（展开高级设置）"}],
        }
        if not options:
            ip_schema["description"] = (
                "正在自动发现 FR3，可稍后重新打开配置；也可选择手动输入 IP。"
                if discovery["state"] == "scanning" else
                "暂未发现 FR3，请选择手动输入 IP；自动发现仅支持直连有线 /24–/30 网段。"
            )
        definition = tool(
            self.PREFIX, "sensor",
            "Franka FR3：读取七个关节的实际角度并显示姿态；只读，不触发运动。",
            topic_out=[{"format": JOINT_FORMAT}],
        )
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "robot_ip": ip_schema,
                "publish_hz": {
                    "type": "number", "title": "显示频率 (Hz)", "scope": "instance",
                    "minimum": 1, "maximum": 30, "default": 10,
                    "description": "画布状态发布频率，不是机械臂控制频率。",
                },
                "manual_robot_ip": {
                    "type": "string", "title": "手动 IP", "scope": "instance",
                    "x-sensitive": True, "advanced": True,
                    "description": "仅在机械臂选择“手动输入 IP”时使用；选择已发现设备时忽略。",
                },
            },
            "required": ["robot_ip"],
        }
        return [definition, tool("model", "resource", "Franka FR3 七轴运动链 URDF，供姿态显示使用。")]

    def start(self):
        # Discovery reads Desk identity only; it never opens an FCI connection.
        with self._lock:
            if self._closed:
                return
            self._started = True
        self._discovery.refresh()

    def stop(self):
        with self._lock:
            self._closed = True
            instances = list(self._instances.items())
        errors = []
        self._discovery.close()
        for key, instance in instances:
            try:
                with instance.lock:
                    self._stop_instance(key, instance)
            except Exception as exc:
                errors.append(exc)
        try:
            self._manager.close()
        finally:
            if errors:
                raise errors[0]

    @staticmethod
    def _instance_id(args):
        value = args.get("instance_id", "")
        if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]{0,127}", value):
            raise ValueError("A valid instance_id is required")
        return value

    def _get_instance(self, instance_id):
        with self._lock:
            if self._closed:
                raise RuntimeError("Driver is shutting down")
            instance = self._instances.get(instance_id)
            if instance is not None:
                return instance
            for other_id in self._instances:
                if other_id != instance_id and other_id.replace("-", "_") == instance_id.replace("-", "_"):
                    raise ValueError("instance_id collides with another card's DDS topic")
            instance = self._instances[instance_id] = _Instance()
            instance.instance_id = instance_id
            return instance

    def _topic(self, instance_id):
        # Match the other robot drivers' state/joints suffix, with a namespace
        # per canvas instance so several FR3s can coexist without topic clashes.
        return [{"topic": f"/{self._namespace}/{instance_id.replace('-', '_')}/state/joints",
                 "format": JOINT_FORMAT}]

    def _owner_key(self, instance_id):
        return (self.PREFIX, instance_id)

    @staticmethod
    def _validate(cfg):
        cfg = dict(cfg)
        if "robot_ip" in cfg:
            value = cfg["robot_ip"]
            if value == "__manual__":
                value = cfg.get("manual_robot_ip", "")
                if not value:
                    raise ValueError("Fill in manual_robot_ip in advanced settings")
            if not isinstance(value, str):
                raise ValueError("robot_ip must be an IP address string")
            # Clearing a saved field is allowed while idle, but cannot connect.
            cfg["robot_ip"] = canonical_ip(value) if value else ""
        value = cfg.get("publish_hz", 10)
        if isinstance(value, bool):
            raise ValueError("publish_hz must be a number from 1 to 30")
        try:
            hz = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("publish_hz must be a number from 1 to 30") from exc
        if not math.isfinite(hz) or not 1 <= hz <= 30:
            raise ValueError("publish_hz must be a number from 1 to 30")
        cfg["publish_hz"] = hz
        # UI selection metadata must not change a running connection's identity.
        cfg.pop("manual_robot_ip", None)
        return cfg

    def _info(self, instance_id, instance):
        snapshot = instance.session.snapshot() if instance.session is not None else {}
        connected = snapshot.get("connected", False)
        active = instance.publisher is not None
        state = "idle"
        if active:
            state = "running" if connected else "loading"
            if instance.publish_error:
                state = "error"
        message = "Configure robot_ip and start this card"
        if active:
            message = "Receiving robot state" if connected else (
                snapshot.get("error")
                or "Waiting for robot state; check IP, FCI activation and SDK compatibility"
            )
            message = instance.publish_error or message
        return {
            "state": state,
            "readiness": snapshot.get("state", "idle"),
            "connected": connected,
            "robot_ip": instance.config.get("robot_ip", ""),
            "read_only": True,
            "data_age_ms": snapshot.get("data_age_ms"),
            "error": instance.publish_error or snapshot.get("error"),
            "publication_error": instance.publish_error,
            "message": message,
            "topic_in": [], "topic_out": self._topic(instance_id),
        }

    def _publish(self, instance, epoch):
        # Do not block DDS teardown on a callback waiting for this instance lock.
        if not instance.lock.acquire(blocking=False):
            return
        try:
            if self._start_cancelled(instance, epoch):
                return  # A queued timer from before stop/restart owns no stream.
            if instance.publisher is None or instance.session is None:
                return
            snapshot = instance.session.snapshot()
            if not snapshot.get("connected"):
                return  # Never republish an old pose with a fresh timestamp.
            positions = snapshot.get("q", [])
            if len(positions) != 7 or not all(math.isfinite(v) for v in positions):
                return
            payload = {
                "timestamp_ms": snapshot["timestamp_ms"],
                "format": JOINT_FORMAT, "position_unit": "rad", "angle_unit": "deg",
                "joint_count": 7, "robot_mode": snapshot.get("robot_mode"),
                "joints": [
                    {"idx": i, "name": JOINT_NAMES[i], "q": position, "degree": math.degrees(position)}
                    for i, position in enumerate(positions)
                ],
            }
            instance.publisher.publish(payload)
            instance.publish_error = None
        except Exception as exc:
            # Timer exceptions escape spin_once and stop the shared DDS
            # executor. Keep a failed publisher local to its card and retry on
            # the next tick; only log the outage transition, not every frame.
            error = f"State publication failed: {type(exc).__name__}: {str(exc)[:200]}"
            if instance.publish_error is None:
                log.warning("%s", error.encode("unicode_escape").decode("ascii"))
            instance.publish_error = error
        finally:
            instance.lock.release()

    def _stop_instance(self, key, instance):
        publisher, instance.publisher = instance.publisher, None
        session, instance.session = instance.session, None
        instance.publish_error = None
        try:
            if publisher is not None:
                publisher.close()
        finally:
            if session is not None:
                self._manager.release(instance.config["robot_ip"], self._owner_key(key))

    def _start_cancelled(self, instance, epoch):
        with self._lock:
            return self._closed or instance.start_epoch != epoch

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            return {"urdf": self._urdf}
        if name != self.PREFIX:
            return None
        if action not in ("config", "start", "stop", "info"):
            return None
        allowed = self.CONFIG_FIELDS
        supplied = {key: args[key] for key in allowed if key in args}
        if action == "config" and args.get("instance_id", "") == "":
            # Core's sidebar may send instance fields as shared defaults.
            with self._lock:
                if self._closed:
                    raise RuntimeError("Driver is shutting down")
                selection = {**self._defaults, **supplied}
                self._validate(selection)
                self._defaults = selection
            return {"state": "configured", "read_only": True}
        instance_id = self._instance_id(args)
        instance = self._get_instance(instance_id)
        with self._lock:
            epoch = instance.start_epoch
            if action == "stop":
                # Signal before waiting for initialization's instance lock.
                # A pending acquire/setup must not leave a running card after
                # the user has already asked to stop it.
                instance.start_epoch += 1
        with instance.lock:
            if action in ("config", "start"):
                with self._lock:
                    if self._closed:
                        raise RuntimeError("Driver is shutting down")
                    defaults = dict(self._defaults)
                if action == "start" and self._start_cancelled(instance, epoch):
                    return self._info(instance_id, instance)
                # Preserve manual mode for partial config updates, separately
                # from the resolved IP/frequency used by sessions and publishers.
                selection = {**defaults, **instance.selection, **supplied}
                try:
                    cfg = self._validate(selection)
                    if instance.session is not None and cfg != instance.config:
                        raise ValueError("Stop this card before changing its IP or publish frequency")
                except Exception as exc:
                    # A caller may persist a rejected config before applying it.
                    # Controllers must not silently move using the previous IP.
                    instance.configuration_error = str(exc)
                    raise
                instance.configuration_error = None
                instance.selection = {**selection, **{k: v for k, v in cfg.items() if k != "robot_ip"}}
                instance.config = cfg
                if action == "start" and instance.session is None:
                    if not cfg.get("robot_ip"):
                        raise ValueError("robot_ip is required; configure this card before starting")
                    instance.session = self._manager.acquire(cfg["robot_ip"], self._owner_key(instance_id))
                    try:
                        if self._start_cancelled(instance, epoch):
                            self._stop_instance(instance_id, instance)
                            return self._info(instance_id, instance)
                        if self.PUBLISH_STATE:
                            instance.publisher = self._publisher_factory(
                                self._topic(instance_id)[0]["topic"],
                                cfg["publish_hz"],
                                lambda: self._publish(instance, epoch),
                            )
                        if self._start_cancelled(instance, epoch):
                            self._stop_instance(instance_id, instance)
                    except Exception:
                        self._stop_instance(instance_id, instance)
                        raise
                return self._info(instance_id, instance)
            if action == "stop":
                self._stop_instance(instance_id, instance)
                return self._info(instance_id, instance)
            if action == "info":
                return self._info(instance_id, instance)
        return None


class JointControlPlugin(FrankaPlugin):
    """Discrete joint targets; the shared session owns hardware arbitration."""

    PUBLISH_STATE = False

    PREFIX = "joint_control"
    ARM_ACTIONS = ("set", "jog", "stopmotion", "Stop")
    CONFIG_FIELDS = FrankaPlugin.CONFIG_FIELDS + ("speed_deg_s",)

    def __init__(self, *args, acp_callback=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._acp_callback = acp_callback or self._post_completion
        config = args[0] if args else kwargs.get("config", {})
        self._arm_ips = {side: canonical_ip(ip) for side, ip in config.get("motion_arm_ips", {}).items()}
        if self._arm_ips and (set(self._arm_ips) != {"left", "right"} or len(set(self._arm_ips.values())) != 2):
            raise ValueError("motion_arm_ips requires distinct left and right IPs")
        self._arm_lock = threading.RLock()
        self._armed_cards = set()
        self._arm_settings = {}

    def _arm_schema(self, config, schema):
        if not self._arm_ips:
            return
        for key in ("robot_ip", "manual_robot_ip"):
            config["properties"].pop(key, None)
        schema["properties"].pop("target_robot_ip", None)
        schema["properties"]["arm"] = {"type": "string", "enum": ["left", "right"],
                                       "description": "按 Franka 自身方向选择左臂或右臂。"}
        for action in schema["x-action-params"].values():
            action["params"].remove("target_robot_ip")
            action["params"].append("arm")

    def _dispatch_arm(self, action, args):
        """Route by the action's side, independently of canvas layout."""
        key = args.get("instance_id")
        if action in ("config", "start", "info", "stop") and key:
            with self._arm_lock:
                if action in ("config", "start"):
                    previous = self._arm_settings.get(key, {})
                    settings = {**previous, **{k: args[k] for k in self.CONFIG_FIELDS
                                if k in args and k not in ("robot_ip", "manual_robot_ip", "publish_hz")}}
                    self._validate(settings)
                    if key in self._armed_cards and settings != previous:
                        raise ValueError("停止卡片后再修改默认参数")
                    self._arm_settings[key] = settings
                    if action == "start":
                        self._armed_cards.add(key)
                elif action == "stop" and key in self._armed_cards:
                    for side in self._arm_ips:
                        self._dispatch("stop", {**args, "instance_id": f"arm_{side}"})
                    self._armed_cards.remove(key)
                return {"state": "ready" if key in self._armed_cards else "idle", "topic_out": []}
        if action not in self.ARM_ACTIONS:
            return self._dispatch(action, args)
        with self._arm_lock:
            side = args.get("arm")
            if side is None and action == "stopmotion" and not key:
                return self._dispatch(action, args)
            if not isinstance(side, str) or side not in self._arm_ips:
                raise ValueError("Select left or right before executing")
            if not self._armed_cards or (key and key not in self._armed_cards):
                raise ValueError("Start the control card before executing")
            instance_id = f"arm_{side}"
            forwarded = {k: v for k, v in args.items() if k not in ("output_topic", "target_robot_ip")}
            forwarded["instance_id"] = instance_id
            if action != "stopmotion":
                options = {}
                if key:
                    options = self._arm_settings[key]
                else:
                    configs = [self._validate(self._arm_settings[card]) for card in self._armed_cards]
                    for field in self.CONFIG_FIELDS:
                        if field in FrankaPlugin.CONFIG_FIELDS:
                            continue
                        values = {cfg[field] for cfg in configs}
                        if len(values) == 1:
                            options[field] = values.pop()
                        elif field not in args and (field != "force_n" or action == "grasp") and action != "homing":
                            raise ValueError(f"卡片默认值不同，请在动作中填写 {field}")
                # Session configuration is stable while a motion is in flight.
                # Pass card defaults as action options so the backend arbitrates busy state.
                for field, value in options.items():
                    forwarded.setdefault(field, value)
                self._dispatch("start", {"_tool_name": self.PREFIX, "instance_id": instance_id,
                                         "robot_ip": self._arm_ips[side]})
            return self._dispatch(action, forwarded)

    def dispatch(self, action, args):
        if args.get("_tool_name", action) != self.PREFIX:
            return None
        if self._arm_ips:
            return self._dispatch_arm(action, args)
        return self._dispatch(action, args)

    @staticmethod
    def _number(value, name, low=None, high=None):
        error = f"{name} must be a finite number"
        if low is not None and high is not None:
            error += f" from {low:g} to {high:g}"
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError(error)
        try:
            value = float(value)
        except OverflowError:
            raise ValueError(error) from None
        if (not math.isfinite(value) or (low is not None and value < low)
                or (high is not None and value > high)):
            raise ValueError(error)
        return value

    @staticmethod
    def _validate(cfg):
        cfg = FrankaPlugin._validate(cfg)
        cfg["speed_deg_s"] = JointControlPlugin._number(
            cfg.get("speed_deg_s", 5), "speed_deg_s", 0.1, 10)
        return cfg

    def get_tools(self):
        definition = super().get_tools()[0]
        config = definition["configSchema"]
        # Official Core actuator click handler may omit instance_id. Its
        # preflight cannot resolve an instance-scoped required robot_ip then.
        # Backend still requires configured IP at start and exact card binding
        # plus fresh, control-ready state before any motion; never guess an IP.
        config.pop("required", None)
        config["properties"].pop("publish_hz", None)
        config["properties"]["speed_deg_s"] = {
            "type": "number", "title": "默认速度 (度/秒)", "scope": "instance",
            "minimum": 0.1, "maximum": 10, "default": 5,
        }
        properties = {
            f"joint{i + 1}_deg": {
                "type": "number", "title": f"J{i + 1} 目标角度 (度)",
                "minimum": math.degrees(low + JOINT_MARGIN),
                "maximum": math.degrees(high - JOINT_MARGIN),
                "description": "绝对角度；留空则保持本次开始时的实际角度。",
            } for i, (low, high) in enumerate(zip(JOINT_LOWER, JOINT_UPPER))
        }
        properties.update({
            "output_topic": {"type": "string",
                             "description": "兼容旧画布传入的输出话题，用于识别卡片实例。"},
            "speed_deg_s": {"type": "number", "title": "速度 (度/秒)",
                            "minimum": 0.1, "maximum": 10,
                            "description": "各轴速度上限；留空使用卡片默认速度。"},
            "joint": {"type": "integer", "title": "关节", "enum": list(range(1, 8))},
            "step_deg": {"type": "number", "title": "点动增量 (度)",
                         "default": 1,
                         "description": "有符号单次增量，不能为零；每次点击执行一次。"},
        })
        schema = action_schema({
            "set": ([*(f"joint{i}_deg" for i in range(1, 8)), "speed_deg_s"],
                    "设置一个或多个关节的绝对角度（度）；至少填写一个关节。"),
            "jog": (["joint", "step_deg", "speed_deg_s"], "单关节点动，按有符号增量移动一次。"),
            # Match RealMan's public name; lifecycle "stop" remains internal.
            "stopmotion": ([], "停止所选机械臂当前运动并保留本控制卡，不需要动作 ID；独立状态卡继续读取。"),
            "info": ([], "查看实际角度、目标角度、执行状态及控制就绪条件。"),
        }, properties)
        schema["properties"]["target_robot_ip"] = {
            "type": "string",
            "description": "填写要执行动作的机械臂 IP，须与已启动卡片的配置一致。",
        }
        for action in schema["x-action-params"].values():
            action["params"].append("target_robot_ip")
        self._arm_schema(config, schema)
        schema["x-completion"] = {"actions": ["set", "jog"], "timeout": 130}
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {key: {"action": "stopmotion"}
                             for key in ("on_interrupt_motion", "on_interrupt_all")}
        return [{**tool(self.PREFIX, "actuator",
                        "Franka FR3 关节控制：J1–J7 角度设置、单次点动及运动停止。", schema),
                 "multiInstance": True, "configSchema": config}]

    def _topic(self, instance_id):
        return [{"topic": f"/{self._namespace}/{instance_id.replace('-', '_')}/state/joint_control",
                 "format": "data/json"}]

    def _resolve_card(self, args):
        """Use the existing output-port binding when the canvas omits the ID."""
        instance_id = args.get("instance_id")
        topic = args.get("output_topic")
        target_ip = args.get("target_robot_ip")
        if "output_topic" in args and (not isinstance(topic, str) or not topic):
            raise ValueError("output_topic must identify this control card")
        if instance_id:
            instance_id = self._instance_id(args)
            if topic is not None and topic != self._topic(instance_id)[0]["topic"]:
                raise ValueError("Control card instance_id and output_topic do not match")
            if target_ip is not None and self._get_instance(instance_id).config.get("robot_ip") != target_ip:
                raise ValueError("target_robot_ip does not match this card")
            return args
        with self._lock:
            matches = [key for key in self._instances
                       if (topic is not None or target_ip is not None)
                       and (topic is None or self._topic(key)[0]["topic"] == topic)
                       and (target_ip is None or (
                           self._instances[key].session is not None
                           and self._instances[key].config.get("robot_ip") == target_ip))]
        if len(matches) != 1:
            raise ValueError("Control card identity is missing or unknown; start this card and refresh the canvas")
        return {**args, "instance_id": matches[0]}

    def _info(self, instance_id, instance):
        snapshot = instance.session.snapshot() if instance.session is not None else {}
        motion = instance.session.motion() if instance.session is not None else {}
        active = instance.session is not None
        connected = snapshot.get("connected", False)
        configuration_error = getattr(instance, "configuration_error", None)
        state = ("ready" if connected else "loading") if active else "idle"
        if motion.get("status") in ("submitted", "accepted", "queued", "running", "stopping") and active:
            state = "running"
        if configuration_error:
            state = "error"
        actual = snapshot.get("q")
        target = motion.get("target")
        return {
            "state": state, "robot_ip": instance.config.get("robot_ip", ""),
            "connected": connected, "readiness": snapshot.get("state", "idle"),
            "control_ready": active and connected and not configuration_error and snapshot.get("control_ready", False),
            "control_error": snapshot.get("control_error"),
            "rt_kernel": snapshot.get("rt_kernel"), "sched_fifo": snapshot.get("sched_fifo"),
            "data_age_ms": snapshot.get("data_age_ms"), "read_only": False,
            "actual_rad": actual,
            "actual_deg": None if actual is None else [math.degrees(q) for q in actual],
            "target_rad": target,
            "target_deg": None if target is None else [math.degrees(q) for q in target],
            "desired_rad": snapshot.get("q_d"), "robot_mode": snapshot.get("robot_mode"),
            "motion": motion, "last_result": getattr(instance, "last_result", None),
            "acp": getattr(instance, "acp", None),
            "error": configuration_error or snapshot.get("error"),
            "position_unit": "deg", "speed_unit": "deg/s",
            "topic_in": [], "topic_out": [],
        }

    def _stop_instance(self, key, instance):
        # Keep the session attached until backend termination is confirmed.
        if instance.session is not None:
            instance.session.release_owner(self._owner_key(key))
        super()._stop_instance(key, instance)

    def _prepare_target(self, action, args, instance):
        if getattr(instance, "configuration_error", None):
            raise RuntimeError("Resolve this card's rejected configuration before motion: "
                               + instance.configuration_error)
        if instance.session is None:
            raise RuntimeError("Start this joint_control card before requesting motion")
        snapshot = instance.session.snapshot()
        current = snapshot.get("q")
        if (not snapshot.get("connected") or current is None or len(current) != 7
                or any(not math.isfinite(q) for q in current)):
            raise RuntimeError("Fresh robot state is required before motion")
        target = list(current)
        if action == "set":
            supplied = [i for i in range(7) if f"joint{i + 1}_deg" in args]
            if not supplied:
                raise ValueError("Set at least one joint1_deg through joint7_deg")
            for i in supplied:
                name = f"joint{i + 1}_deg"
                target[i] = math.radians(self._number(
                    args[name], name, math.degrees(JOINT_LOWER[i] + JOINT_MARGIN),
                    math.degrees(JOINT_UPPER[i] - JOINT_MARGIN)))
        else:
            joint = args.get("joint")
            # Canvas enum selects serialize their values as strings.
            if isinstance(joint, str) and joint in ("1", "2", "3", "4", "5", "6", "7"):
                joint = int(joint)
            if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 7:
                raise ValueError("joint must be an integer from 1 to 7")
            step = self._number(args.get("step_deg", 1), "step_deg")
            if step == 0:
                raise ValueError("step_deg must not be zero")
            target[joint - 1] += math.radians(step)
        for i, value in enumerate(target):
            if not JOINT_LOWER[i] + JOINT_MARGIN <= value <= JOINT_UPPER[i] - JOINT_MARGIN:
                raise ValueError(f"J{i + 1} target is outside the allowed joint range")
        speed = self._number(args.get("speed_deg_s", instance.config["speed_deg_s"]),
                             "speed_deg_s", 0.1, 10)
        if not snapshot.get("control_ready"):
            raise RuntimeError(snapshot.get("control_error") or "Robot backend is not ready for motion")
        distance = max(abs(goal - start) for goal, start in zip(target, current))
        duration = max(1.0, 2.1875 * distance / math.radians(speed),
                       math.sqrt(7.514 * distance / 0.3), (52.5 * distance) ** (1 / 3))
        if duration + 2.0 > 120:
            raise ValueError(f"预计轨迹耗时 {duration:.1f} 秒，超过120秒动作限制；请减小角度或自行选择更高速度")
        return target, math.radians(speed)

    def _complete(self, instance, session, action_id):
        try:
            result = session.wait(action_id, timeout=125)
        except Exception as exc:
            # A lost acknowledgement must stop the actual backend, not just the waiter.
            try:
                session.stop_motion(command_id=action_id)
            except Exception as stop_error:
                result = {"status": "failed", "error": f"{exc}; stop failed: {stop_error}"}
            else:
                result = {"status": "failed", "error": str(exc)}
        status = {"succeeded": "completed", "cancelled": "cancelled"}.get(result.get("status"), "error")
        with instance.lock:
            if getattr(instance, "last_action_id", None) == action_id:
                instance.last_result = {"action_id": action_id, "status": status, "result": result}
                instance.acp = {"action_id": action_id, "state": "pending"}
        error = None
        # Completion IDs are idempotent at Core; bounded retry handles transient failures.
        for attempt in range(3):
            try:
                self._acp_callback(action_id, status, result)
                error = None
                break
            except Exception as exc:
                error = str(exc)[:240]
                if attempt < 2:
                    time.sleep(0.25 * (attempt + 1))
        with instance.lock:
            if getattr(instance, "last_action_id", None) == action_id:
                instance.acp = {"action_id": action_id, "state": "failed" if error else "accepted",
                                "error": error}

    def _post_completion(self, action_id, status, result):
        import os
        import ssl
        import urllib.request

        ca = os.environ.get("AGENT_CORE_CA_CERT")
        if not ca:
            raise RuntimeError("AGENT_CORE_CA_CERT is required for ACP completion")
        context = ssl.create_default_context(cafile=ca)
        opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), urllib.request.HTTPSHandler(context=context))
        body = {"action_id": action_id, "status": status, "result": result,
                "tool": self.PREFIX, "ts": time.time()}
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        request = urllib.request.Request(url + "/api/acp/complete",
                                         data=json.dumps(body, allow_nan=False).encode(),
                                         headers={"Content-Type": "application/json"}, method="POST")
        with opener.open(request, timeout=3) as response:
            acknowledgement = json.load(response)
        if (not isinstance(acknowledgement, dict) or acknowledgement.get("ok") is not True
                or acknowledgement.get("action_id") != action_id):
            raise RuntimeError("Agent Core did not acknowledge this action_id")

    def _dispatch(self, action, args):
        if args.get("_tool_name", action) != self.PREFIX:
            return None
        if action in ("set", "jog", "Stop") or (action == "stopmotion" and
                (args.get("instance_id") or "output_topic" in args or "target_robot_ip" in args)) or (
                action in ("info", "stop") and "output_topic" in args):
            args = self._resolve_card(args)
        # Keep old saved canvas actions working without advertising a second stop.
        if action == "Stop":
            action = "stopmotion"
        if action in ("config", "start", "info"):
            result = super().dispatch(action, args)
            if result is not None:
                result["read_only"] = False
            return result
        if action not in ("set", "jog", "stopmotion", "stop"):
            return None
        if action in ("stop", "stopmotion") and not args.get("instance_id"):
            # Core fires driver-wide interrupt hooks without a canvas instance.
            # Attempt every selected IP even when one backend rejects its stop.
            with self._lock:
                sessions = [instance.session for instance in self._instances.values()]
            outcomes, errors, seen = [], [], set()
            for session in sessions:
                if session is None or id(session) in seen:
                    continue
                seen.add(id(session))
                # Session submission/stop is atomic. A card lifecycle lock can
                # be waiting for confirmation and must not delay another arm.
                try:
                    outcomes.append({"robot_ip": session.address,
                                     "motion": session.stop_motion()})
                except Exception as exc:
                    errors.append(str(exc))
            return {"state": "error" if errors else "stop_requested", "robots": outcomes,
                    **({"error": "; ".join(errors)} if errors else {})}
        instance_id = self._instance_id(args)
        instance = self._get_instance(instance_id)
        if action == "stop":
            # This name is also the framework's lifecycle stop. Cancel pending
            # startup before waiting on its lock, then stop the selected robot
            # and detach this card only after backend confirmation.
            with self._lock:
                if instance.session is None:
                    instance.start_epoch += 1
        with instance.lock:
            if "target_robot_ip" in args and instance.config.get("robot_ip") != args["target_robot_ip"]:
                raise ValueError("target_robot_ip does not match this card")
            if action in ("set", "jog"):
                target, speed = self._prepare_target(action, args, instance)
                session = instance.session
                action_id = session.move(self._owner_key(instance_id), target,
                                         speed_rad_s=speed, tolerance_rad=math.radians(0.5), timeout_s=120)
                instance.last_action_id = action_id
                instance.acp = {"action_id": action_id, "state": "waiting_for_motion"}
                threading.Thread(target=self._complete, args=(instance, session, action_id),
                                 name=f"franka-acp-{instance_id}", daemon=True).start()
                return {"state": "running", "action_id": action_id,
                        "target_deg": [math.degrees(q) for q in target],
                        "speed_deg_s": math.degrees(speed)}
            motion = {"status": "idle"}
            if instance.session is not None:
                motion = instance.session.stop_motion()
                if motion.get("command_id") is not None:
                    try:
                        motion = instance.session.wait(motion["command_id"], timeout=3.5)
                    except TimeoutError as exc:
                        raise RuntimeError("Physical motion stop was not confirmed") from exc
                    if motion.get("status") not in ("succeeded", "cancelled", "rejected"):
                        raise RuntimeError("Physical motion stop was not confirmed: "
                                           + str(motion.get("error")))
            if action == "stop":
                with self._lock:
                    instance.start_epoch += 1
                self._stop_instance(instance_id, instance)
            return {**self._info(instance_id, instance), "motion": motion}


def build_plugins(config, namespace, ros2):
    from franka.fr3.camera import ExtCameraPlugin
    from franka.fr3.gripper import GripperPlugin
    sensor = FrankaPlugin(config, namespace, ros2)
    controller = JointControlPlugin(config, namespace, ros2,
        manager=sensor._manager, discovery=sensor._discovery)
    gripper = GripperPlugin(config, namespace, ros2, discovery=sensor._discovery)
    return [sensor, controller, gripper,
            ExtCameraPlugin(config.get("ext_camera", {}), namespace, ros2)]
