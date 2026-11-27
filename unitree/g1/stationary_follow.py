"""G1 in-place visual follower. Translation is never commanded."""
import math
import time
import json
import threading


def _finite_number(value):
    return (isinstance(value, (int, float)) and
            not isinstance(value, bool) and
            math.isfinite(value))


def _convert_person(person, size):
    """Convert one person; raise ValueError only for this person."""
    if not isinstance(person, dict):
        raise ValueError("invalid_person")
    person_id = person.get("id")
    if (not isinstance(person_id, (int, str)) or isinstance(person_id, bool)
            or person_id == ""):
        raise ValueError("missing_person_id")
    points = person.get("keypoints")
    leg_center = None
    if isinstance(points, list) and len(points) >= 17:
        def side_x(indices):
            xs = []
            for index in indices:
                point = points[index]
                if (isinstance(point, list) and len(point) >= 3 and
                        _finite_number(point[0]) and
                        _finite_number(point[2]) and
                        point[2] >= .3):
                    xs.append(point[0])
            return sum(xs) / len(xs) if xs else None
        left, right = side_x((13, 15)), side_x((14, 16))
        if left is not None and right is not None:
            leg_center = (left + right) / 2
    box = person.get("bbox")
    if leg_center is not None:
        center = leg_center
    elif isinstance(box, list) and len(box) == 4 and all(_finite_number(v) for v in box):
        center = (box[0] + box[2]) / 2
    else:
        if not isinstance(points, list):
            raise ValueError("missing_person_position")
        xs = [p[0] for p in points if isinstance(p, list) and len(p) >= 2
              and _finite_number(p[0])
              and (len(p) < 3 or (_finite_number(p[2]) and p[2] > 0))]
        if not xs:
            raise ValueError("missing_person_position")
        center = (min(xs) + max(xs)) / 2
    x = 2 * center / size[0] - 1
    if not -1.0 <= x <= 1.0:
        raise ValueError("position_out_of_range")
    return {"id": person_id, "position": [x, 0]}


def pose2d_to_follow_frame(payload, received_at):
    """Convert the documented sensor/pose2d pixel coordinates to [-1, 1]."""
    size = payload.get("image_size")
    persons = payload.get("persons")
    if (not isinstance(size, list) or len(size) != 2 or
            not _finite_number(size[0]) or size[0] <= 0 or
            not _finite_number(size[1]) or size[1] <= 0 or
            not isinstance(persons, list)):
        raise ValueError("invalid_pose2d")
    # 不能跳过坏的 person，否则多人画面可能被误判为单人。
    converted = [_convert_person(person, size) for person in persons]
    return {"timestamp": received_at, "persons": converted}


def vop_to_follow_frame(payload, received_at):
    """Use one fresh VOP person detection; VOP has no persistent person ID."""
    if not isinstance(payload, dict) or not _finite_number(payload.get("timestamp")):
        raise ValueError("invalid_vop_frame")
    latency = payload.get("latency_ms", 0)
    if not _finite_number(latency) or not 0 <= latency <= 1000:
        raise ValueError("invalid_vop_latency")
    age = time.time() - (payload["timestamp"] - latency / 1000)
    if not 0 <= age <= 1.0 or payload.get("available") is False or payload.get("error"):
        raise ValueError("stale_or_unavailable_vop")
    objects = payload.get("objects")
    if not isinstance(objects, list) or len(objects) > 1000:
        raise ValueError("invalid_vop_objects")
    if "count" in payload and (type(payload["count"]) is not int or
                               payload["count"] != len(objects)):
        raise ValueError("invalid_vop_count")
    persons = []
    for obj in objects:
        if not isinstance(obj, dict):
            raise ValueError("invalid_vop_object")
        if obj.get("name") not in ("person", "human", "人"):
            continue
        position, confidence = obj.get("position"), obj.get("confidence")
        if (not isinstance(position, list) or len(position) != 2 or
                not all(_finite_number(v) and -1 <= v <= 1 for v in position) or
                not _finite_number(confidence) or not 0 <= confidence <= 1):
            raise ValueError("invalid_vop_person")
        if confidence >= .55:
            persons.append({"id": "single_vop_person", "position": position})
    return {"timestamp": received_at, "persons": persons}


class FollowCard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.enabled = False
        self.target_id = None
        self.last_frame = None
        self.last_command = 0.0
        self.last_motion = False
        self.reason = "idle"

    def start(self):
        self.enabled = True
        self.target_id = None
        self.last_frame = None
        self.last_command = 0.0
        self.last_motion = False
        self.reason = "waiting_for_single_person"
        return {"command": "none", "reason": self.reason}

    def stop(self, reason="manual_stop"):
        """Hard stop: requires an explicit start() to resume."""
        self.enabled = False
        self.target_id = None
        self.reason = reason
        self.last_motion = False
        self.last_command = 0.0
        return {"command": "stop_move", "reason": reason}

    def pause(self, reason):
        """Recoverable error: keep enabled and target, resume on next valid frame."""
        self.reason = reason
        self.last_motion = False
        self.last_command = 0.0
        return {"command": "stop_move", "reason": reason}

    def ingest(self, payload):
        if not self.enabled:
            return {"command": "none", "reason": self.reason}
        stamp = payload.get("timestamp")
        persons = payload.get("persons")
        if (not isinstance(stamp, (float, int)) or not math.isfinite(stamp)
                or not isinstance(persons, list)):
            return self.pause("invalid_pose")
        if self.last_frame is not None and stamp <= self.last_frame:
            return self.pause("stale_pose")
        self.last_frame = stamp
        if self.target_id is None:
            if len(persons) != 1:
                self.reason = "waiting_for_single_person"
                return {"command": "none", "reason": self.reason}
            self.target_id = persons[0].get("id")
            if self.target_id is None:
                return self.pause("invalid_target")
        matches = [p for p in persons if p.get("id") == self.target_id]
        if len(matches) != 1:
            return self.pause("target_lost")
        position = matches[0].get("position")
        if (not isinstance(position, list) or len(position) < 1 or
                not _finite_number(position[0])):
            return self.pause("invalid_position")
        x = float(position[0])  # Motus pose normalized horizontal position, +right.
        if abs(x) > 1:
            return self.pause("position_out_of_range")
        # Hysteresis: start correction beyond 0.18, stop inside 0.08.
        if abs(x) > .18:
            self.last_motion = True
        elif abs(x) < .08:
            self.last_motion = False
        if not self.last_motion:
            self.last_command = 0.0
            self.reason = "centered"
            return {"command": "stop_move", "reason": self.reason}
        # Camera x is right-positive, G1 vyaw is left-positive.
        vyaw = -max(-.20, min(.20, x * .4))
        self.last_command = vyaw
        self.reason = "following"
        return {"command": "move", "vx": 0.0, "vy": 0.0,
                "vyaw": vyaw, "duration": .25, "reason": self.reason,
                "target_id": self.target_id}

    def tick(self, age_s):
        if self.enabled and age_s > .3 and self.reason != "pose_timeout":
            return self.pause("pose_timeout")
        return {"command": "none", "reason": self.reason}


class StationaryFollowPlugin:
    """Canvas card with bounded turns, a pose watchdog and visible decisions."""

    PREFIX = "g1_stationary_follow"

    def __init__(self, config, namespace, executor, motion=None,
                 admission=None, base_lock=None, clock=time.monotonic):
        self._executor = executor
        self._clock = clock
        self._lock = threading.RLock()
        self._controller = FollowCard(clock=clock)
        self._motion = motion
        self._admission = admission
        self._base_lock = base_lock or threading.RLock()
        self._node = None
        self._topic = ""
        self._output = f"/{namespace}/stationary_follow/decision"
        self._publisher = None
        self._message_type = None
        self._worker = None
        self._cleanup_in_progress = False
        self._shutdown = threading.Event()
        self._owned = False
        self._last_motion_at = 0.0
        self._last_received = None
        self._last_decision = {"command": "none", "reason": "idle"}

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "G1 原地小跟班：根据 VOP 单人位置低速原地转向，丢失目标或超时停止。",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]},
                    "input_topic": {"type": "string", "description": "VOP /objects 检测流话题"},
                },
                "x-action-params": {"start": {"params": ["input_topic"]},
                                    "stop": {"params": []}, "info": {"params": []}},
                "x-resource": "base",
            },
            "topic_in": [{"format": "data/json", "desc": "VOP 人物检测流（/objects）"}],
            "topic_out": [{"topic": self._output, "format": "data/json"}],
        }

    def start(self):
        # Bundle start does not arm a Canvas-controlled card.
        pass

    @property
    def active(self):
        with self._lock:
            return self._node is not None

    @property
    def blocks_base(self):
        with self._lock:
            return self._node is not None or self._cleanup_in_progress

    def stop(self):
        self._stop()

    def dispatch(self, action, args):
        if action == "start":
            raw_topic = args.get("input_topic")
            topic = raw_topic.strip() if isinstance(raw_topic, str) else ""
            if not topic:
                topics = args.get("input_topics") or []
                if topics and isinstance(topics[0], str):
                    topic = topics[0].strip()
            if not topic:
                return {"state": "error", "error": "请在画布连接 VOP /objects 输出"}
            with self._lock:
                if (self._node is not None or self._cleanup_in_progress or
                        self._worker is not None and self._worker.is_alive()):
                    return {"state": "error", "error": "卡片已经启动或正在收尾"}
                try:
                    from rclpy.node import Node
                    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                    from std_msgs.msg import String
                    node = Node("g1_stationary_follow")
                    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
                    node.create_subscription(String, topic, self._on_message, qos)
                    publisher = node.create_publisher(String, self._output, qos)
                    self._executor.add_node(node)
                except Exception as exc:
                    if "node" in locals():
                        node.destroy_node()
                    return {"state": "error", "error": f"无法订阅人物姿态流: {exc}"}
                self._node = node
                self._publisher = publisher
                self._message_type = String
                self._topic = topic
                self._last_received = None
                self._last_decision = self._controller.start()
                self._shutdown.clear()
                self._worker = threading.Thread(target=self._control_loop, daemon=True,
                                                name="g1_stationary_follow")
                self._worker.start()
            return {"state": "running", "input": topic,
                    "topic_out": [{"topic": self._output, "format": "data/json"}]}
        if action == "stop":
            return self._stop()
        if action == "info":
            with self._lock:
                return {"state": "running" if self._node is not None else "idle",
                        "input": self._topic,
                        "topic_out": [{"topic": self._output, "format": "data/json"}],
                        "last_decision": dict(self._last_decision)}
        return None

    def _on_message(self, message):
        with self._lock:
            if self._node is None:
                return
            received_at = self._clock()
            try:
                frame = vop_to_follow_frame(json.loads(message.data), received_at)
                self._last_decision = self._controller.ingest(frame)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError,
                    UnicodeDecodeError):
                # 坏帧不永久停用，只暂停，等待下一帧。
                self._last_decision = self._controller.pause("invalid_vop")
            self._last_received = received_at

    def _publish(self, decision, result=None):
        if self._publisher is None:
            return
        payload = {"timestamp": time.time(), "decision": decision,
                   "motion_result": result, "motion_enabled": self._motion is not None}
        self._publisher.publish(self._message_type(data=json.dumps(payload, ensure_ascii=False)))

    def _control_loop(self):
        while not self._shutdown.wait(.1):
            with self._lock:
                if self._node is None:
                    return
                if self._last_received is None:
                    decision = {"command": "none", "reason": "waiting_for_pose"}
                else:
                    watchdog = self._controller.tick(self._clock() - self._last_received)
                    if watchdog["command"] != "none":
                        self._last_decision = watchdog
                    decision = dict(self._last_decision)
            result = None
            if decision.get("command") == "move":
                if self._motion is None:
                    decision = {"command": "stop_move", "reason": "motion_unavailable"}
                    result = {"error": "motion_unavailable"}
                elif self._owned and time.monotonic() - self._last_motion_at < .30:
                    # 等上一条有限时长命令及底盘状态自然恢复为空闲。
                    result = {"status": "lease_active", "max_remaining_s": .30}
                else:
                    self._owned = False
                    with self._base_lock:
                        if self._shutdown.is_set():
                            break
                        try:
                            ready = self._admission() if self._admission else {"admitted": False}
                            state = self._motion.get_state()
                            if self._shutdown.is_set():
                                break
                            if (ready.get("admitted") is not True or
                                    state.get("base_available") is not True):
                                # 模式未就绪或底盘正忙是暂态，不永久停用卡片。
                                decision = {"command": "stop_move", "reason": "base_not_ready"}
                                result = {"status": "waiting_for_base",
                                          "admitted": ready.get("admitted"),
                                          "admission_reason": ready.get("reason"),
                                          "base_available": state.get("base_available")}
                            else:
                                result = self._motion.move(0.0, 0.0, decision["vyaw"], .25)
                                if result.get("ret") == 0:
                                    self._owned = True
                                    self._last_motion_at = time.monotonic()
                                else:
                                    with self._lock:
                                        self._last_decision = self._controller.stop("motion_error")
                                    decision = dict(self._last_decision)
                        except Exception as exc:
                            result = {"error": str(exc)}
                            with self._lock:
                                self._last_decision = self._controller.stop("motion_error")
                            decision = dict(self._last_decision)
            else:
                with self._lock:
                    owned = self._owned
                    if owned:
                        self._owned = False
                if owned:
                    # 已发送命令有 .25 s 的硬截止时间，不额外发送阻塞式 stop RPC。
                    result = {"status": "bounded_motion_expiry", "max_remaining_s": .25}
            self._publish(decision, result)

    def _stop(self):
        with self._lock:
            if self._cleanup_in_progress:
                return {"state": "idle"}
            self._cleanup_in_progress = True
            node, self._node = self._node, None
            self._shutdown.set()
            self._topic = ""
            self._last_decision = self._controller.stop()
            publisher = self._publisher
            message_type = self._message_type
            worker = self._worker
        # 发布最终停止决策，便于外部诊断。
        if publisher is not None and message_type is not None:
            try:
                payload = {"timestamp": time.time(),
                           "decision": dict(self._last_decision),
                           "motion_result": None,
                           "motion_enabled": self._motion is not None}
                publisher.publish(message_type(data=json.dumps(payload, ensure_ascii=False)))
            except Exception:
                pass
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1)

        def cleanup():
            # 无论如何都要重置 _cleanup_in_progress，避免 base 被永久锁住。
            try:
                if worker is not None and worker.is_alive():
                    worker.join(timeout=2)
                remaining = self._last_motion_at + .30 - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                if node is not None:
                    try:
                        self._executor.remove_node(node)
                    except Exception:
                        pass
                    try:
                        node.destroy_node()
                    except Exception:
                        pass
            finally:
                with self._lock:
                    self._owned = False
                    self._publisher = None
                    self._message_type = None
                    if self._worker is worker:
                        self._worker = None
                    self._cleanup_in_progress = False

        if worker is not None and worker.is_alive():
            threading.Thread(target=cleanup, daemon=True,
                             name="g1_stationary_follow_cleanup").start()
        else:
            cleanup()
        return {"state": "idle"}
