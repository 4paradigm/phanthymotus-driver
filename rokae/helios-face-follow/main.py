"""Single-card target tracking for Helios cameras and Perception VOP."""

import json
import os
import re
import signal
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

from base_follow import CHEST_CAMERA, BaseFollower
from common.vendor_runtime import (
    DriverBundle, action_schema, load_config, make_handler, start_registration, tool,
)
from head_control import CONTROL_PERIOD, HEAD_JOINTS, SAMPLE_AGE, HeadFollower, UnixHeadBridge
from tracking import MAX_AGE, FollowPolicy, _Detector, decode_depth, finite, image_stamp

DRIVER_ID = "rokae-helios-face-follow"
MODES = ("person", "nearest_face", "speaker", "object")
VOP_MODES = ("person", "object")
DEFAULT_DEPTH = "/rokae_helios/camera/depth/visual_depth"
DEFAULT_SPEECH = "/rokae_helios/speech/localized"
JOINTS = "/rokae_helios/state/joints"


def call_local(port, name, action, **arguments):
    request = Request(f"http://127.0.0.1:{port}/mcp", data=json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {
            "name": name, "arguments": {"action": action, **arguments},
        },
    }).encode(), headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=3) as response:
        payload = json.load(response)
    if payload.get("error"):
        raise ValueError(payload["error"].get("message", f"{name} failed"))
    return json.loads(payload["result"]["content"][0]["text"])


def start_perception(name, topic):
    result = call_local(15720, name, "start", input_topic=topic)
    if result.get("state") not in ("running", "loading"):
        raise ValueError(result.get("message", f"{name} did not start"))


def start_vop(topic):
    start_perception("vop", topic)


class HeliosFaceFollow:
    def __init__(self, node, head_bridge=None, base=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.node = node
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speech_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.RLock()
        self.detector = None
        self.subscriptions = []
        self.session = None
        self.manual_camera = None
        self.follow_paused = False
        self.topics = {}
        self.policy = FollowPolicy("person", "person")
        self.depth_frames = deque(maxlen=8)
        self.person_depth = None
        self.rgb_size = None
        self.head = None
        self.base = base
        self.motion = base if base is not None else HeadFollower(head_bridge)
        self.motion_enabled = base is not None or head_bridge is not None
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
        node.create_timer(CONTROL_PERIOD, self.move_motion)

    def get_tools(self):
        entry = tool("helios_face_follow", "processor",
                     "Helios 目标随动，默认通过 RGB 跟踪人体，输出目标位置和朝向控制状态。",
                     action_schema({
                         "start": (["input_topic", "depth_topic", "speech_topic", "object_topic"], "连接输入"),
                         "toggle_person_follow": (["input_topic"], "开始或停止持续人体随动"),
                         "set_mode": (["mode", "object_name"], "切换模式"),
                         "set_target": (["object_name"], "跟踪指定的 VOP 物体类别"),
                         "pause_follow": ([], "暂停底盘随动，保留检测与目标设置"),
                         "resume_follow": ([], "恢复已启动的卡片的底盘随动"),
                         "follow_status": ([], "查询卡片模式、检测目标和底盘随动状态"),
                         "stop": ([], "结束跟踪"),
                         "info": ([], "查看目标、输入和朝向控制状态"),
                     }, {
                         "input_topic": {"type": "string", "description": "Helios JPEG，相机随底盘转向时用胸部 RGB"},
                         "depth_topic": {"type": "string", "description": "配准至 RGB 的 640×480 毫米深度话题"},
                         "speech_topic": {"type": "string", "description": "相机坐标下的说话人事件"},
                         "object_topic": {"type": "string", "description": "同一相机的 VOP 检测话题"},
                         "mode": {"type": "string", "enum": list(MODES), "default": "person"},
                         "object_name": {"type": "string", "description": "指定 VOP 类别"},
                     }), topic_in=[{"format": "image/jpeg", "desc": "Helios RGB"}],
                     topic_out=self.output)
        entry["inputSchema"]["x-helios-canvas-execute"] = "toggle_person_follow"
        del entry["inputSchema"]["x-action-params"]["toggle_person_follow"]
        entry["inputSchema"]["x-action-params"]["set_mode"]["required"] = ["mode"]
        entry["inputSchema"]["x-action-params"]["set_target"]["required"] = ["object_name"]
        return [entry]

    def dispatch(self, action, args):
        with self.lock:
            if action == "start":
                self.start(args)
                # Core now owns the upstream source for this running project.
                self.manual_camera = None
            elif action == "toggle_person_follow":
                if self.session is not None:
                    self.stop()
                else:
                    self.start_from_canvas(args)
                    if self.base is not None:
                        try:
                            base = call_local(15782, "base_drive", "info")
                            if not base.get("worker_running") or base.get("chassis_exception_level") != 0:
                                detail = base.get("connect_error") or "底盘尚未返回可用的运动反馈"
                                return {**self.info(), "启动说明": f"人体检测已启动，转向等待底盘：{detail}"}
                        except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
                            return {**self.info(), "启动说明": f"人体检测已启动，底盘接口暂未就绪：{error}"}
            elif action == "stop":
                self.stop()
            elif action == "pause_follow":
                self.follow_paused = True
                if self.motion.halt("paused") is False:
                    return {**self.info(), "error": "底盘未确认停止，请查看运动状态"}
                if self.session is not None:
                    self.publish()
            elif action == "resume_follow":
                if self.session is None:
                    return {**self.info(), "error": "请先从画布启动随动卡"}
                if self.follow_paused:
                    self.follow_paused = False
                    self.emit(None, "waiting_for_" + self.policy.mode)
            elif action in ("set_mode", "set_target"):
                mode = args.get("mode") if action == "set_mode" else "object"
                name = args.get("object_name", self.policy.label if self.policy.mode == "object" else "")
                if mode not in MODES or not isinstance(name, str) or (mode == "object" and not name.strip()):
                    return {"state": "error", "error": "Invalid mode or missing object_name"}
                stopped = self.motion.halt("waiting_for_target")
                if stopped is False:
                    return {"state": "error", "error": "Could not cancel the active base turn"}
                if mode in VOP_MODES and self.session is not None:
                    self.ensure_vop(self.topics)
                elif mode not in VOP_MODES:
                    self.detector = self.detector or _Detector()
                label = "person" if mode == "person" else name.strip().casefold() if mode == "object" else ""
                self.policy = FollowPolicy(mode, label)
                if self.session is not None:
                    self.emit(None, "waiting_for_" + mode)
            elif action not in ("info", "follow_status"):
                raise ValueError(f"unsupported action {action}")
            return self.info()

    def start_from_canvas(self, args):
        topic = args.get("input_topic")
        match = re.fullmatch(r"/rokae_helios/camera/card_([a-z0-9]+)/rgb", topic or "")
        if not match:
            raise ValueError("请先将 Helios 胸部 RGB 相机接到随动卡的输入端口")
        instance = "card-" + match[1]
        camera = call_local(15747, "camera_rgb", "info", instance_id=instance)
        declarations = camera.get("camera_info") or []
        declaration = next((d for d in declarations
                            if d.get("id") == CHEST_CAMERA and d.get("topic") == topic), None)
        if camera.get("camera") != "chest_rgb" or declaration is None:
            raise ValueError("随动卡需要接入胸部 RGB 相机")
        if camera.get("state") != "online":
            reply = call_local(15747, "camera_rgb", "start", instance_id=instance)
            if reply.get("state") in (None, "error", "idle"):
                raise ValueError(reply.get("error") or "胸部相机未能启动")
            self.manual_camera = instance
        try:
            self.start({"input_topic": topic, "camera_info": {topic: declaration}})
        except Exception:
            self.stop()
            raise

    def ensure_vop(self, topics):
        # An explicit topic belongs to its producer. Otherwise request VOP for this RGB feed.
        if topics["objects"] == topics["rgb"] + "/objects" and self.node.count_publishers(topics["objects"]) == 0:
            try:
                start_vop(topics["rgb"])
            except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
                raise ValueError(f"Cannot start VOP: {error}") from error

    def start(self, args):
        from rclpy.validate_full_topic_name import validate_full_topic_name
        from sensor_msgs.msg import CompressedImage

        rgb = args.get("input_topic")
        if not rgb:
            raise ValueError("Connect the Helios RGB camera card")
        topics = {
            "rgb": rgb,
            "depth": args.get("depth_topic") or DEFAULT_DEPTH,
            "speech": args.get("speech_topic") or DEFAULT_SPEECH,
            "objects": args.get("object_topic") or f"{rgb}/objects",
            "person_depth": f"{rgb}/visual_depth",
        }
        if self.session is not None:
            if topics != self.topics:
                raise ValueError("Stop the card before changing inputs")
            return
        for topic in topics.values():
            validate_full_topic_name(topic)
        self.ensure_vop(topics)
        if self.node.count_publishers(topics["person_depth"]) == 0:
            try:
                start_perception("visual_depth", rgb)
            except (OSError, ValueError, KeyError, IndexError, TypeError):
                # A single visible person can still be followed. With several
                # people, the selector waits for distance estimates.
                pass
        session = object()
        for key, callback, kind in (
            ("rgb", self.on_rgb, CompressedImage), ("depth", self.on_depth, CompressedImage),
            ("speech", self.on_speech, self.message_type), ("objects", self.on_objects, self.message_type),
            ("person_depth", self.on_person_depth, CompressedImage),
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
        self.follow_paused = False
        self.motion.reset()
        cameras = args.get("camera_info")
        camera = cameras.get(rgb) if isinstance(cameras, dict) else None
        if isinstance(camera, dict) and camera.get("topic") == rgb:
            size = (camera.get("width"), camera.get("height"))
            if all(type(value) is int and value > 0 for value in size):
                self.rgb_size = size
        if self.base is not None:
            self.base.camera_ok = (isinstance(camera, dict) and camera.get("topic") == rgb
                                   and camera.get("id") == CHEST_CAMERA)
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
            self.person_depth = self.rgb_size = None
            self.topics = {}
            self.policy = FollowPolicy(self.policy.mode, self.policy.label)
            if self.manual_camera:
                instance, self.manual_camera = self.manual_camera, None
                try:
                    call_local(15747, "camera_rgb", "stop", instance_id=instance)
                except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
                    print(f"[camera] cannot stop {instance}: {error}", flush=True)
            if was_active:
                self.emit(None, "idle")

    def info(self):
        return {
            "state": "running" if self.session is not None else "idle",
            "follow_paused": self.follow_paused,
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
        target_age = MAX_AGE if self.base is not None else SAMPLE_AGE
        self.motion_target_until = 0.0 if observation_time is None else observation_time + target_age
        if target is not None:
            self.last_target = time.monotonic()
        self.publish()

    def publish(self):
        result = self.info()
        result.update(timestamp=time.time(), available=self.result is not None)
        self.publisher.publish(self.message_type(data=json.dumps(result, allow_nan=False)))

    def move_motion(self):
        with self.lock:
            if self.session is None:
                if self.base is not None and self.base.action_id:
                    self.base.halt("idle")
                return
            if self.follow_paused:
                if self.base is not None and self.base.action_id:
                    self.base.halt("paused")
                return
            if not self.motion_enabled:
                return
            before = self.motion.status()
            now = time.monotonic()
            target_until = min(self.motion_rgb_until, self.motion_target_until)
            target = self.result if now < target_until else None
            if self.base is not None:
                self.base.step(target, self.target_sample, target_until)
            else:
                feedback_until = self.head[2] if self.head else 0.0
                feedback = self.head[1] if self.head and now < feedback_until else None
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

    def on_person_depth(self, session, message):
        if message.format != "16UC1; compressedDepth zlib":
            return
        import numpy as np

        received = time.monotonic()
        depth = decode_depth(message.data, np)
        with self.lock:
            if session is self.session and depth is not None:
                self.person_depth = (received, depth)

    def on_objects(self, session, message):
        with self.lock:
            now = time.monotonic()
            if session is self.session and self.policy.mode in VOP_MODES and now - self.last_rgb <= MAX_AGE:
                payload = self.parse(message)
                wall = time.time()
                depth = None
                if self.person_depth and now - self.person_depth[0] <= 0.35:
                    depth = self.person_depth[1]
                target, reason = self.policy.objects(payload, wall, now, depth, self.rgb_size)
                if reason == "stale_object_frame":
                    return  # A delayed or duplicate packet cannot invalidate a newer observation.
                observation = now - (wall - payload["timestamp"]) - payload.get("latency_ms", 0) / 1000 if target else None
                self.emit(target, reason, observation)

    def on_rgb(self, session, message):
        now = time.monotonic()
        with self.lock:
            if session is not self.session:
                return
            self.last_rgb = now
            if self.motion_enabled:
                age = time.time() - image_stamp(message)
                self.motion_rgb_until = now + SAMPLE_AGE - age if 0 <= age <= SAMPLE_AGE else 0
            if self.policy.mode in VOP_MODES or now - self.last_sample < 0.12:
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
    base = BaseFollower() if os.environ.get("HELIOS_BASE_FOLLOW_ENABLED") == "1" else None
    plugin = HeliosFaceFollow(node, UnixHeadBridge(socket_path) if socket_path and base is None else None, base)
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
        plugin.stop()
        executor.shutdown()
        node.destroy_node()
        server.server_close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
