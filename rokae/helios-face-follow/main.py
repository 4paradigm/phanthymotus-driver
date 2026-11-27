"""Single-card target tracking for Helios cameras and Perception VOP."""

import json
import os
import signal
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

from common.vendor_runtime import (
    DriverBundle, action_schema, load_config, make_handler, start_registration, tool,
)
from head_control import CONTROL_PERIOD, HEAD_JOINTS, SAMPLE_AGE, HeadFollower, UnixHeadBridge
from tracking import MAX_AGE, FollowPolicy, _Detector, finite, image_stamp

DRIVER_ID = "rokae-helios-face-follow"
MODES = ("nearest_face", "speaker", "object")
DEFAULT_DEPTH = "/rokae_helios/camera/depth/visual_depth"
DEFAULT_SPEECH = "/rokae_helios/speech/localized"
JOINTS = "/rokae_helios/state/joints"


def start_vop(topic):
    request = Request("http://127.0.0.1:15720/mcp", data=json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {
            "name": "vop", "arguments": {"action": "start", "input_topic": topic},
        },
    }).encode(), headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=3) as response:
        payload = json.load(response)
    result = json.loads(payload["result"]["content"][0]["text"])
    if result.get("state") not in ("running", "loading"):
        raise ValueError(result.get("message", "VOP did not start"))


class HeliosFaceFollow:
    def __init__(self, node, head_bridge=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.node = node
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speech_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.RLock()
        self.detector = None
        self.subscriptions = []
        self.session = None
        self.topics = {}
        self.policy = FollowPolicy("nearest_face")
        self.depth_frames = deque(maxlen=8)
        self.head = None
        self.motion = HeadFollower(head_bridge)
        self.motion_rgb_until = self.motion_target_until = 0.0
        self.target_sample = 0
        self.last_rgb = 0
        self.last_sample = -1
        self.result = None
        self.reason = "idle"
        self.last_target = None
        self.output = [{"topic": "/rokae_helios/face_follow/target", "format": "data/json"}]
        self.message_type = String
        self.publisher = node.create_publisher(String, self.output[0]["topic"], 1)
        node.create_subscription(String, JOINTS, self.on_joints, self.qos)
        node.create_timer(0.2, self.expire)
        node.create_timer(CONTROL_PERIOD, self.move_head)

    def get_tools(self):
        return [tool("helios_face_follow", "processor",
                     "Helios 目标随动，输出目标位置、深度和头部控制状态。",
                     action_schema({
                         "start": (["input_topic", "depth_topic", "speech_topic", "object_topic"], "连接输入"),
                         "set_mode": (["mode", "object_name"], "切换模式"),
                         "set_target": (["object_name"], "跟踪指定的 VOP 物体类别"),
                         "stop": ([], "结束跟踪"),
                         "info": ([], "查看目标、输入和关节反馈状态"),
                     }, {
                         "input_topic": {"type": "string", "description": "头部 JPEG 相机话题"},
                         "depth_topic": {"type": "string", "description": "配准至 RGB 的 640×480 毫米深度话题"},
                         "speech_topic": {"type": "string", "description": "相机坐标下的说话人事件"},
                         "object_topic": {"type": "string", "description": "同一相机的 VOP 检测话题"},
                         "mode": {"type": "string", "enum": list(MODES)},
                         "object_name": {"type": "string", "description": "指定 VOP 类别"},
                     }), topic_in=[{"format": "image/jpeg", "desc": "Helios 头部 RGB"}],
                     topic_out=self.output)]

    def dispatch(self, action, args):
        with self.lock:
            if action == "start":
                self.start(args)
            elif action == "stop":
                self.stop()
            elif action in ("set_mode", "set_target"):
                if self.session is None:
                    return {"state": "error", "error": "Start the card before selecting a mode"}
                mode = args.get("mode") if action == "set_mode" else "object"
                name = args.get("object_name", self.policy.label)
                if mode not in MODES or not isinstance(name, str) or (mode == "object" and not name.strip()):
                    return {"state": "error", "error": "Invalid mode or missing object_name"}
                self.motion.halt("waiting_for_target")
                # An explicitly wired topic belongs to its producer. For the
                # default topic, reuse a live VOP instance or start shared VOP.
                if (mode == "object" and self.topics["objects"] == self.topics["rgb"] + "/objects"
                        and self.node.count_publishers(self.topics["objects"]) == 0):
                    try:
                        start_vop(self.topics["rgb"])
                    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
                        return {"state": "error", "error": f"Cannot start VOP: {error}"}
                self.policy = FollowPolicy(mode, name.strip().casefold() if mode == "object" else "")
                self.emit(None, "waiting_for_" + mode)
            elif action != "info":
                raise ValueError(f"unsupported action {action}")
            return self.info()

    def start(self, args):
        from rclpy.validate_full_topic_name import validate_full_topic_name
        from sensor_msgs.msg import CompressedImage

        rgb = args.get("input_topic")
        if not rgb:
            raise ValueError("Connect the Helios head RGB card")
        topics = {
            "rgb": rgb,
            "depth": args.get("depth_topic") or DEFAULT_DEPTH,
            "speech": args.get("speech_topic") or DEFAULT_SPEECH,
            "objects": args.get("object_topic") or f"{rgb}/objects",
        }
        if self.session is not None:
            if topics != self.topics:
                raise ValueError("Stop the card before changing inputs")
            return
        for topic in topics.values():
            validate_full_topic_name(topic)
        self.detector = self.detector or _Detector()
        session = object()
        for key, callback, kind in (
            ("rgb", self.on_rgb, CompressedImage), ("depth", self.on_depth, CompressedImage),
            ("speech", self.on_speech, self.message_type), ("objects", self.on_objects, self.message_type),
        ):
            try:
                self.subscriptions.append(self.node.create_subscription(
                    kind, topics[key], lambda msg, cb=callback: cb(session, msg),
                    self.speech_qos if key == "speech" else self.qos))
            except Exception:
                self.stop()
                raise
        self.topics = topics
        self.session = session
        self.motion.reset()
        self.motion_rgb_until = 0.0
        self.last_rgb = 0
        self.last_sample = -1
        self.emit(None, "waiting_for_rgb")

    def stop(self):
        with self.lock:
            was_active = self.session is not None
            self.session = None
            self.motion.reset()
            for sub in self.subscriptions:
                self.node.destroy_subscription(sub)
            self.subscriptions.clear()
            self.depth_frames.clear()
            self.topics = {}
            self.policy = FollowPolicy("nearest_face")
            if was_active:
                self.emit(None, "idle")

    def info(self):
        return {
            "state": "running" if self.session is not None else "idle",
            "mode": self.policy.mode, "object_name": self.policy.label,
            "reason": self.reason, "target": self.result, "head_feedback": self.fresh_head(),
            "motion_available": self.motion.state == "controlling", "motion": self.motion.status(), "topic_in": (
                [{"topic": self.topics["rgb"], "format": "image/jpeg"}] if self.topics else []
            ), "auxiliary_inputs": {k: v for k, v in self.topics.items() if k != "rgb"},
            "topic_out": self.output,
        }

    def fresh_head(self):
        return self.head[1] if self.head and time.monotonic() - self.head[0] <= MAX_AGE else None

    def emit(self, target, reason, observation_time=None):
        self.reason, self.result = reason, target
        self.target_sample += 1
        self.motion_target_until = 0.0 if observation_time is None else observation_time + SAMPLE_AGE
        if target is not None:
            self.last_target = time.monotonic()
        self.publish()

    def publish(self):
        result = self.info()
        result.update(timestamp=time.time(), available=self.result is not None)
        self.publisher.publish(self.message_type(data=json.dumps(result, allow_nan=False)))

    def move_head(self):
        with self.lock:
            if self.session is None or self.motion.bridge is None:
                return
            before = self.motion.status()
            now = time.monotonic()
            target_until = min(self.motion_rgb_until, self.motion_target_until)
            target = self.result if now < target_until else None
            feedback = None
            feedback_until = self.head[2] if self.head else 0.0
            if now < feedback_until:
                feedback = self.head[1]
            self.motion.step(self.topics["rgb"], target, feedback, self.target_sample, now,
                             min(target_until, feedback_until))
            if self.motion.status() != before:
                self.publish()

    def expire(self):
        with self.lock:
            if self.session is None:
                return
            now = time.monotonic()
            if now - self.last_rgb > MAX_AGE and self.reason != "waiting_for_rgb":
                self.emit(None, "waiting_for_rgb")
            elif self.result is not None and now - self.last_target > MAX_AGE:
                self.emit(None, "target_stale")

    @staticmethod
    def parse(message):
        try:
            payload = json.loads(message.data)
        except (ValueError, RecursionError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def on_joints(self, message):
        payload = self.parse(message)
        joints = payload.get("joints")
        if payload.get("available") is not True or not isinstance(joints, list):
            with self.lock:
                self.head = None
            return
        positions = {j["name"]: j["q"] for j in joints if isinstance(j, dict)
                     and j.get("name") in HEAD_JOINTS and finite(j.get("q"))}
        with self.lock:
            received = payload.get("received_at_ms")
            age = time.time() - received / 1000 if finite(received) else float("inf")
            now = time.monotonic()
            self.head = ((now, positions, now + SAMPLE_AGE - age if 0 <= age <= SAMPLE_AGE else 0)
                         if len(positions) == len(HEAD_JOINTS) else None)

    def on_depth(self, session, message):
        with self.lock:
            if session is self.session and self.policy.mode == "nearest_face":
                self.depth_frames.append(message)

    def on_speech(self, session, message):
        with self.lock:
            if session is self.session and self.policy.mode == "speaker":
                payload = self.parse(message)
                if self.policy.speech_event(payload, time.time()) and payload["active"] is False:
                    self.emit(None, "waiting_for_speech")

    def on_objects(self, session, message):
        with self.lock:
            now = time.monotonic()
            if session is self.session and self.policy.mode == "object" and now - self.last_rgb <= MAX_AGE:
                payload = self.parse(message)
                wall = time.time()
                target, reason = self.policy.objects(payload, wall, now)
                observation = now - (wall - payload["timestamp"]) - payload.get("latency_ms", 0) / 1000 if target else None
                self.emit(target, reason, observation)

    def on_rgb(self, session, message):
        now = time.monotonic()
        with self.lock:
            if session is not self.session:
                return
            self.last_rgb = now
            if self.motion.bridge is not None:
                age = time.time() - image_stamp(message)
                self.motion_rgb_until = now + SAMPLE_AGE - age if 0 <= age <= SAMPLE_AGE else 0
            if self.policy.mode == "object" or now - self.last_sample < 0.12:
                return
            self.last_sample = now
            policy = self.policy
            rgb_stamp = image_stamp(message)
            observation = now - (time.time() - rgb_stamp)
            depth = min(self.depth_frames, key=lambda frame: abs(image_stamp(frame) - rgb_stamp),
                        default=None)
        try:
            boxes, size = self.detector.detect(message.data)
        except (ValueError, self.detector.cv2.error):
            with self.lock:
                if session is self.session and policy is self.policy:
                    self.emit(None, "invalid_rgb")
            return
        with self.lock:
            if session is self.session and policy is self.policy and time.monotonic() - now <= MAX_AGE:
                target, reason = policy.faces(boxes, size, message, depth, self.detector.np, now, time.time())
                self.emit(target, reason, observation if target and observation <= now else None)


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    config = load_config(__file__)
    port = config["mcp_port"]
    rclpy.init()
    node = rclpy.create_node("helios_face_follow")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    socket_path = os.environ.get("HELIOS_HEAD_CONTROL_SOCKET", "")
    plugin = HeliosFaceFollow(node, UnixHeadBridge(socket_path) if socket_path else None)
    bundle = DriverBundle([plugin])
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(lambda: bundle, DRIVER_ID, DRIVER_ID))

    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        start_registration(port, config, DRIVER_ID)
        server.serve_forever()
    finally:
        executor.shutdown()
        plugin.stop()
        node.destroy_node()
        server.server_close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
