"""G1 in-place visual follower. Translation is never commanded."""
import math
import time
import json
import threading


def pose2d_to_follow_frame(payload, received_at):
    """Convert the documented sensor/pose2d pixel coordinates to [-1, 1]."""
    size = payload.get("image_size")
    persons = payload.get("persons")
    if (not isinstance(size, list) or len(size) != 2 or
            not isinstance(size[0], (int, float)) or
            not math.isfinite(size[0]) or size[0] <= 0 or
            not isinstance(persons, list)):
        raise ValueError("invalid_pose2d")
    converted = []
    for person in persons:
        if not isinstance(person, dict):
            raise ValueError("invalid_person")
        box = person.get("bbox")
        if isinstance(box, list) and len(box) == 4 and all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in box):
            center = (box[0] + box[2]) / 2
        else:
            points = person.get("keypoints")
            if not isinstance(points, list):
                raise ValueError("missing_person_position")
            xs = [p[0] for p in points if isinstance(p, list) and len(p) >= 2
                  and isinstance(p[0], (int, float)) and math.isfinite(p[0])
                  and (len(p) < 3 or isinstance(p[2], (int, float)) and p[2] > 0)]
            if not xs:
                raise ValueError("missing_person_position")
            center = (min(xs) + max(xs)) / 2
        x = 2 * center / size[0] - 1
        if not -1 <= x <= 1:
            raise ValueError("position_out_of_range")
        converted.append({"id": person.get("id", 0), "position": [x, 0]})
    return {"timestamp": received_at, "persons": converted}


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
        self.reason = "waiting_for_single_person"
        return {"command": "none", "reason": self.reason}

    def stop(self, reason="manual_stop"):
        self.enabled = False
        self.target_id = None
        self.reason = reason
        self.last_motion = False
        self.last_command = 0.0
        return {"command": "stop_move", "reason": reason}

    def ingest(self, payload):
        now = self.clock()
        if not self.enabled:
            return {"command": "none", "reason": self.reason}
        stamp = payload.get("timestamp")
        persons = payload.get("persons")
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or not isinstance(persons, list):
            return self.stop("invalid_pose")
        if self.last_frame is not None and stamp <= self.last_frame:
            return self.stop("stale_pose")
        self.last_frame = stamp
        if self.target_id is None:
            if len(persons) != 1:
                return {"command": "none", "reason": "waiting_for_single_person"}
            self.target_id = persons[0].get("id")
            if self.target_id is None:
                return self.stop("invalid_target")
        matches = [p for p in persons if p.get("id") == self.target_id]
        if len(matches) != 1:
            return self.stop("target_lost")
        position = matches[0].get("position")
        if not isinstance(position, list) or len(position) < 1 or not isinstance(position[0], (float, int)) or not math.isfinite(position[0]):
            return self.stop("invalid_position")
        x = float(position[0])  # Motus pose normalized horizontal position, +right.
        if abs(x) > 1:
            return self.stop("position_out_of_range")
        # Hysteresis: start correction beyond 0.18, stop inside 0.08.
        if abs(x) > .18:
            self.last_motion = True
        elif abs(x) < .08:
            self.last_motion = False
        if not self.last_motion:
            self.last_command = 0.0
            return {"command": "stop_move", "reason": "centered"}
        # Camera x is right-positive, G1 vyaw is left-positive.
        vyaw = -max(-.20, min(.20, x * .4))
        self.last_command = vyaw
        return {"command": "move", "vx": 0.0, "vy": 0.0,
                "vyaw": vyaw, "duration": .2, "reason": "following",
                "target_id": self.target_id}

    def tick(self, age_s):
        if self.enabled and age_s > .3:
            return self.stop("pose_timeout")
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
        self._mode = "legs"
        self._leg_detector = None

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "G1 原地小跟班：下视深度图检测唯一双腿候选，低速原地转向；识别不明确即停止。",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]},
                    "input_topic": {"type": "string", "description": "image/depth-zlib 下视深度流；pose 模式可手填姿态话题"},
                    "input_mode": {"type": "string", "enum": ["legs", "pose"], "default": "legs"},
                },
                "x-action-params": {"start": {"params": ["input_topic"]},
                                    "stop": {"params": []}, "info": {"params": []}},
                "x-resource": "base",
            },
            "topic_in": [{"format": "image/depth-zlib", "desc": "G1 下视深度相机；仅连接这一路"}],
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
            mode = args.get("input_mode") or "legs"
            if mode not in ("legs", "pose"):
                return {"state": "error", "error": "input_mode must be legs or pose"}
            topic = (args.get("input_topic") or "").strip()
            if not topic:
                topics = args.get("input_topics") or []
                topic = (topics[0] or "").strip() if topics else ""
            if not topic:
                return {"state": "error", "error": "请在画布连接 camera_depth 输入"}
            with self._lock:
                if (self._node is not None or self._cleanup_in_progress or
                        self._worker is not None and self._worker.is_alive()):
                    return {"state": "error", "error": "卡片已经启动或正在收尾"}
                try:
                    from rclpy.node import Node
                    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                    if mode == "legs":
                        from sensor_msgs.msg import CompressedImage as Message
                        from leg_detection import DepthLegDetector
                    else:
                        from std_msgs.msg import String as Message
                    node = Node("g1_stationary_follow")
                    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
                    node.create_subscription(Message, topic, self._on_message, qos)
                    from std_msgs.msg import String
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
                self._mode = mode
                self._leg_detector = DepthLegDetector() if mode == "legs" else None
                self._last_received = None
                self._last_decision = self._controller.start()
                self._shutdown.clear()
                self._worker = threading.Thread(target=self._control_loop, daemon=True,
                                                name="g1_stationary_follow")
                self._worker.start()
            return {"state": "running", "input": topic,
                    "mode": mode, "topic_out": [{"topic": self._output, "format": "data/json"}]}
        if action == "stop":
            return self._stop()
        if action == "info":
            with self._lock:
                return {"state": "running" if self._node is not None else "idle",
                        "input": self._topic,
                        "mode": self._mode,
                        "topic_out": [{"topic": self._output, "format": "data/json"}],
                        "last_decision": dict(self._last_decision)}
        return None

    def _on_message(self, message):
        with self._lock:
            if self._node is None:
                return
            received_at = self._clock()
            if self._mode == "legs":
                if message.format != "16UC1; compressedDepth zlib":
                    self._last_decision = self._controller.stop("invalid_depth_format")
                else:
                    center, reason = self._leg_detector.detect(message.data)
                    if center is None:
                        self._last_decision = (self._controller.stop(reason)
                                               if self._controller.target_id is not None
                                               else {"command": "none", "reason": reason})
                    else:
                        self._last_decision = self._controller.ingest(
                            {"timestamp": received_at, "persons":
                             [{"id": "leg_pair", "position": [center, 0]}]})
                self._last_received = received_at
                return
            try:
                frame = pose2d_to_follow_frame(json.loads(message.data), received_at)
                self._last_decision = self._controller.ingest(frame)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError):
                self._last_decision = self._controller.stop("invalid_pose2d")
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
            with self._base_lock:
                if self._shutdown.is_set():
                    break
                if decision.get("command") == "move" and self._motion is not None:
                    state = {}
                    try:
                        ready = self._admission() if self._admission else {"admitted": False}
                        state = self._motion.get_state()
                        if self._shutdown.is_set():
                            break
                        if ready.get("admitted") is not True or state.get("base_available") is not True:
                            result = {"error": "base_not_available_or_locomotion_not_ready"}
                        else:
                            result = self._motion.move(0.0, 0.0, decision["vyaw"], .12)
                            self._owned = result.get("ret") == 0
                            if self._owned:
                                self._last_motion_at = time.monotonic()
                    except Exception as exc:
                        result = {"error": str(exc)}
                    if (result.get("error") or result.get("ret", 0) != 0) and state.get("base_available") is True:
                        with self._lock:
                            self._last_decision = self._controller.stop("motion_error")
                        decision = dict(self._last_decision)
                elif self._owned:
                    # Every command has a 0.12 s deadline in TimedMotion. A second
                    # synchronous stop RPC can block the safety process well past
                    # Core's 5 s project-stop deadline after that command expires.
                    # Suppress new commands; the existing one expires on its timer.
                    result = {"status": "bounded_motion_expiry", "max_remaining_s": .12}
                    self._owned = False
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
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1)
        def cleanup():
            if worker is not None and worker.is_alive():
                worker.join()
            # A successful TimedMove can still be within its 0.12 s lease
            # after the worker exits. Keep base ownership until it expires.
            remaining = self._last_motion_at + .15 - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            if node is not None:
                self._executor.remove_node(node)
                node.destroy_node()
            with self._lock:
                self._owned = False
                self._publisher = None
                self._message_type = None
                self._leg_detector = None
                if self._worker is worker:
                    self._worker = None
                self._cleanup_in_progress = False
        if worker is not None and worker.is_alive():
            threading.Thread(target=cleanup, daemon=True,
                             name="g1_stationary_follow_cleanup").start()
        else:
            cleanup()
        return {"state": "idle"}
