"""Canvas actuator for the wooden-man host. DDS 42 never owns vendor DDS 211.

The parent executes bounded arm trajectories through the shared body router.
No position-mode transitions, locomotion, head, leg or finger commands.
"""
from __future__ import annotations

import json
import math
import multiprocessing as mp
import queue
import threading
import time
from collections import OrderedDict

GESTURES = ("welcome", "round", "point", "celebrate", "encourage")
JOINT_SUFFIXES = ("shoulder_pitch_joint", "shoulder_roll_joint", "elbow_pitch_joint")
PASSIVE_SUFFIXES = ("arm_yaw_joint", "elbow_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint")
OWNER = "wooden_man_motion"


def trajectory(request, mirrored=False):
    """Return a coarse camera sector and small absolute arm pose in radians."""
    gesture = request.get("gesture")
    if gesture not in GESTURES:
        raise ValueError("Unknown host gesture")
    sector = "centre"
    side = "both" if gesture == "celebrate" else "right"
    pitch, roll, elbow = {"welcome": (-42, 18, -35), "round": (-40, 12, -25),
                           "celebrate": (-48, 25, -35), "encourage": (-38, 12, -25),
                           "point": (-45, 10, -25)}[gesture]
    if gesture == "point":
        target = request.get("target")
        if not isinstance(target, dict) or any(isinstance(target.get(k), bool) or not isinstance(target.get(k), (int, float))
                or not math.isfinite(target[k]) or not 0 <= target[k] <= 1 for k in ("x", "y")):
            raise ValueError("point requires normalised target.x/y in [0,1]")
        x, y = target["x"], target["y"]
        if mirrored:
            x = 1 - x
        col = "left" if x < 1/3 else "right" if x > 2/3 else "centre"
        row = "upper" if y < 1/3 else "lower" if y > 2/3 else "middle"
        side = "left" if col == "left" else "right"
        sector = row + "_" + col
        pitch = {"upper": -65, "middle": -45, "lower": -25}[row]
        roll = 25 if col != "centre" else 10
        elbow = -15 if row == "lower" else -25
    positions = {}
    for arm in (("left", "right") if side == "both" else (side,)):
        for suffix, deg in zip(JOINT_SUFFIXES, (pitch, roll if arm == "left" else -roll, elbow)):
            positions[arm + "_" + suffix] = math.radians(deg)
    return sector, positions


def feedback(request_id, state, **extra):
    return {"kind": "wooden_man_motion_feedback", "request_id": request_id,
            "state": state, "timestamp": time.time(), **extra}


class MotionRunner:
    """One cancellable hardware worker; duplicate requests only refresh a lease."""
    def __init__(self, client, router, report, mirrored=False):
        self.client, self.router, self.report = client, router, report
        self.mirrored = mirrored
        self.lock = threading.RLock()
        self.cache = OrderedDict()
        self.thread = None
        self.cancel = threading.Event()
        self.lease = 0.0
        self.closed = False
        self.monitored = ()
        self.passive = ()

    def _record(self, message):
        with self.lock:
            self.cache[message["request_id"]] = message
            while len(self.cache) > 128:
                self.cache.popitem(last=False)
        self.report(dict(message))

    def submit(self, request):
        if not isinstance(request, dict) or request.get("kind") not in ("wooden_man_motion_request", "wooden_man_motion_cancel"):
            return
        rid = request.get("request_id")
        if not isinstance(rid, str) or not 1 <= len(rid) <= 128:
            return
        with self.lock:
            if request["kind"] == "wooden_man_motion_cancel":
                previous = self.cache.get(rid)
                if previous and previous["state"] == "running":
                    self.cancel.set()
                elif previous is None:
                    self._record(feedback(rid, "cancelled", message="Cancelled before execution"))
                return
            if rid in self.cache:
                previous = self.cache[rid]
                if previous["state"] == "running":
                    self.lease = time.monotonic()
                self.report(dict(previous))
                return
            stamp = request.get("issued_at")
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not -1 <= time.time() - stamp <= 5:
                self._record(feedback(rid, "error", message="Stale or invalid request timestamp"))
                return
            if self.closed or (self.thread is not None and self.thread.is_alive()):
                self._record(feedback(rid, "error", message="Motion card is stopped or busy"))
                return
            try:
                sector, target = trajectory(request, self.mirrored)
            except ValueError as exc:
                self._record(feedback(rid, "error", message=str(exc)))
                return
            self.cancel = threading.Event()
            self.lease = time.monotonic()
            self._record(feedback(rid, "running", gesture=request["gesture"], sector=sector))
            self.thread = threading.Thread(target=self._run, args=(request, target, sector), daemon=True,
                                           name="q5_wooden_man_arms")
            self.thread.start()

    def guard(self):
        from control_contract import q5_is_control_ready
        if self.cancel.is_set():
            raise RuntimeError("cancelled")
        if time.monotonic() - self.lease > 2.0:
            raise RuntimeError("Game request heartbeat lost")
        if self.client is None or self.router is None:
            raise RuntimeError("Q5 vendor connection unavailable")
        status = self.router.status()
        if not status.get("ros_publisher_available") or not status.get("endpoint_query_available"):
            raise RuntimeError("Body command endpoint unavailable")
        if status.get("other_publishers") or status.get("same_name_publisher_count", 0) > 1:
            raise RuntimeError("Competing body command publisher")
        if status.get("active_owner") not in (None, OWNER):
            raise RuntimeError("Another body card owns the command router")
        if self.client.get_lifecycle_state() != "active" or not q5_is_control_ready(self.client)[0]:
            raise RuntimeError("Q5 is not fresh READY/ACTIVE with active motion_manager")
        fsm_age = self.client.sensor_snapshot("robot_status").get("age_ms")
        if isinstance(fsm_age, bool) or not isinstance(fsm_age, (int, float)) or not math.isfinite(fsm_age) or not 0 <= fsm_age <= 1000:
            raise RuntimeError("Q5 FSM feedback is older than 1000 ms")
        if not getattr(self.client, "q5_position_control_prepared", False):
            raise RuntimeError("Prepare arm_control position control before starting this project")
        snapshot = self.client.snapshot()
        age = snapshot.get("age_ms")
        if not snapshot.get("fresh") or isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= 500:
            raise RuntimeError("Joint feedback is missing or older than 500 ms")
        joints = snapshot.get("joints", {})
        if not isinstance(joints, dict) or any(k not in joints or isinstance(joints[k], bool)
                or not isinstance(joints[k], (int, float)) or not math.isfinite(joints[k]) for k in (*self.monitored, *self.passive)):
            raise RuntimeError("Required arm joint feedback unavailable")
        if any(abs(joints[k]) > .35 for k in self.passive):
            raise RuntimeError("Arm yaw/wrists must be near neutral; use arm_gesture reset first")
        return joints

    def _move(self, target):
        start = self.guard()
        if any(k not in start or isinstance(start[k], bool) or not isinstance(start[k], (int, float)) or not math.isfinite(start[k]) for k in target):
            raise RuntimeError("Required arm joint feedback unavailable")
        delta = max(abs(target[k] - start[k]) for k in target)
        duration = max(.1, delta / .20)
        steps = max(1, math.ceil(delta / .01), math.ceil(duration * 20))
        for step in range(1, steps + 1):
            self.guard()
            point = {k: start[k] + (target[k] - start[k]) * step / steps for k in target}
            if not self.router.publish(point):
                raise RuntimeError("Body command publish failed")
            self.cancel.wait(duration / steps)
        # Completion means observed arrival, not merely the final command sent.
        until = time.monotonic() + 2.0
        while True:
            current = self.guard()
            if all(k in current and math.isfinite(current[k]) and abs(current[k] - target[k]) <= .08 for k in target):
                return
            if time.monotonic() >= until:
                raise RuntimeError("Arm did not reach target according to joint feedback")
            if not self.router.publish(target):
                raise RuntimeError("Body hold publish failed")
            self.cancel.wait(.05)

    def _run(self, request, target, sector):
        from joint_limits import JOINT_LIMITS
        acquired = False
        state, message = "error", "Motion failed"
        original = {}
        try:
            self.monitored = tuple(target)
            arms = {k.split("_", 1)[0] for k in target}
            self.passive = tuple(arm + "_" + suffix for arm in arms for suffix in PASSIVE_SUFFIXES)
            joints = self.guard()
            if any(k not in JOINT_LIMITS or not JOINT_LIMITS[k][0] <= v <= JOINT_LIMITS[k][1] for k, v in target.items()):
                raise RuntimeError("Target exceeds URDF limits")
            if any(k not in joints or not isinstance(joints[k], (int, float)) or isinstance(joints[k], bool) or not math.isfinite(joints[k]) for k in target):
                raise RuntimeError("Required arm joint feedback unavailable")
            original = {k: joints[k] for k in target}
            if any(not JOINT_LIMITS[k][0] <= v <= JOINT_LIMITS[k][1] or abs(v - target[k]) > .8 for k, v in original.items()):
                raise RuntimeError("Start with relaxed arms; excursion would exceed 0.8 rad")
            acquired = self.router.acquire(OWNER)
            if not acquired:
                raise RuntimeError("Body command router busy")
            self._move(target)
            until = time.monotonic() + .6
            while time.monotonic() < until:
                self.guard()
                if not self.router.publish(target):
                    raise RuntimeError("Body hold publish failed")
                self.cancel.wait(.05)
            self._move(original)
            state, message = "completed", "Gesture and measured return to initial arm pose completed"
        except Exception as exc:
            state = "cancelled" if self.cancel.is_set() else "error"
            message = str(exc)
            # No forced return on a fault. Only hold measured joints if the
            # vendor state remains valid; never continue a cancelled sequence.
            if acquired:
                try:
                    was_cancelled = self.cancel.is_set()
                    # guard deliberately rejects cancellation: cancellation
                    # stops publications, leaving the vendor controller holding.
                    if not was_cancelled:
                        current = self.guard()
                        hold = {k: current[k] for k in original if k in current and isinstance(current[k], (int, float)) and math.isfinite(current[k])}
                        if len(hold) == len(original):
                            self.router.publish(hold)
                except Exception:
                    pass
        finally:
            if acquired:
                self.router.release(OWNER)
            self._record(feedback(request["request_id"], state, gesture=request["gesture"], sector=sector, message=message))

    def shutdown(self):
        with self.lock:
            self.closed = True
            self.cancel.set()
            thread = self.thread
        if thread is not None:
            thread.join(timeout=5)
            if thread.is_alive():
                raise RuntimeError("Arm worker did not stop; refusing to start another worker")


def _bridge(input_topic, output_topic, requests, replies, stop, ready):
    """Domain-42 String transport only. Hardware objects never enter this process."""
    import os
    from common import logsafe
    logsafe.install(check_fd=False)
    os.environ.update(ROS_DOMAIN_ID="42", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    os.environ.pop("CYCLONEDDS_URI", None)
    from fastdds_transport import configure_bundled_fastdds_transport
    configure_bundled_fastdds_transport()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    node = None
    try:
        rclpy.init()
        node = Node("q5_wooden_man_motion")
        pub = node.create_publisher(String, output_topic, 10)
        def receive(msg):
            try:
                item = json.loads(msg.data)
                requests.put_nowait(item)
            except (ValueError, TypeError, queue.Full):
                pass
        sub = node.create_subscription(String, input_topic, receive, 10)
        ready.put({"state": "running"})
        last, sent_at = None, 0.
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.02)
            try:
                while True:
                    last = replies.get_nowait()
                    sent_at = 0.
            except queue.Empty:
                pass
            if last is not None and time.monotonic() - sent_at >= .25:
                pub.publish(String(data=json.dumps(last, ensure_ascii=False)))
                sent_at = time.monotonic()
    except Exception as exc:
        ready.put({"state": "error", "message": str(exc)})
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Plugin:
    def __init__(self, config, namespace, executor, client):
        self.prefix = f"/{namespace}/wooden_man_motion"
        self.client = client
        from body_command import get_router
        self.router = get_router(client, executor) if client is not None else None
        self.config = {"image_mirrored": config.get("image_mirrored", False)}
        self.process = self.listener = self.runner = None
        self.lock = threading.RLock()
        self.status = {"state": "idle"}
        self.input_topic = ""

    def get_tool(self):
        return {"name": OWNER, "title": "木头人主持动作", "type": "actuator", "multiInstance": False,
                "description": "消费wooden_man的actions，执行手臂方位示意、轮间和胜负手势，实测回位后反馈；不自动准备位置直控。",
                "inputSchema": {"type": "object", "required": ["action"], "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "cancel"]},
                    "input_topic": {"type": "string"}, "input_topics": {"type": "array", "items": {"type": "string"}}},
                    "x-action-params": {"start": {"params": ["input_topic", "input_topics"]}, "stop": {"params": []}, "info": {"params": []}, "cancel": {"params": []}},
                    "x-resource": ["arm_l", "arm_r"]},
                "configSchema": {"type": "object", "properties": {
                    "image_mirrored": {"type": "boolean", "title": "输入画面左右镜像", "default": False,
                                       "description": "仅在摄像头原始输出确实左右镜像时开启；不是网页预览的镜像。"}}},
                "topic_in": [{"format": "data/json", "desc": "只接wooden_man/actions，不能接speech或status"}],
                "topic_out": [{"topic": self.prefix + "/feedback", "format": "data/json", "desc": "动作完成反馈；填入木头人的motion_feedback_topic，不接TTS"}]}

    def start(self):
        pass  # Driver boot must never move an arm or start a DDS subscription.

    def _report(self, message):
        self.status = {"state": "running", "feedback": message}
        try:
            self.replies.put_nowait(message)
        except queue.Full:
            pass  # DDS retries and the request-ID cache repair a dropped reply.

    def _listen(self):
        while not self.stop_event.is_set():
            if not self.process.is_alive():
                self.runner.cancel.set()
                self.status = {"state": "error", "message": "Motion DDS bridge exited"}
                return
            try:
                self.runner.submit(self.requests.get(timeout=.05))
            except queue.Empty:
                pass

    def stop(self):
        if self.runner is not None:
            self.runner.shutdown()
        if self.process is not None:
            self.stop_event.set()
            if self.listener is not None:
                self.listener.join(timeout=2)
            self.process.join(timeout=3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
            for q in (self.requests, self.replies, self.ready):
                q.close()
            self.process = self.listener = self.runner = None
        self.status = {"state": "idle"}
        self.input_topic = ""

    def info(self):
        return {**self.status, "config": dict(self.config), "input_topic": self.input_topic,
                "topic_in": self.get_tool()["topic_in"], "topic_out": self.get_tool()["topic_out"]}

    def dispatch(self, action, args):
        with self.lock:
            if action == "info":
                return self.info()
            if action == "stop":
                self.stop()
                return self.info()
            if action == "cancel":
                if self.runner:
                    self.runner.cancel.set()
                return self.info()
            if action not in ("config", "start"):
                return None
            supplied = args.get("config") or {}
            if not isinstance(supplied, dict):
                raise ValueError("config须为对象")
            mirrored = args.get("image_mirrored", supplied.get("image_mirrored", self.config["image_mirrored"]))
            if not isinstance(mirrored, bool):
                raise ValueError("image_mirrored须为布尔值")
            if action == "config":
                if mirrored != self.config["image_mirrored"]:
                    self.stop()
                self.config = {"image_mirrored": mirrored}
                return {"status": "configured", "adapter_ok": True, **self.info()}
            topics = args.get("input_topics", [])
            if not isinstance(topics, list) or any(not isinstance(t, str) for t in topics):
                return {"state": "error", "message": "input_topics须为字符串数组"}
            singular = args.get("input_topic")
            if singular is not None and not isinstance(singular, str):
                return {"state": "error", "message": "input_topic须为字符串"}
            topics = list(dict.fromkeys([*topics, *([singular] if singular else [])]))
            if len(topics) != 1 or not isinstance(topics[0], str) or not topics[0].startswith("/") or any(c.isspace() for c in topics[0]):
                return {"state": "error", "message": "只连接wooden_man/actions这一路输入"}
            if self.process is not None and self.process.is_alive() and self.input_topic == topics[0] and self.config["image_mirrored"] == mirrored:
                return self.info()
            self.stop()
            self.input_topic, self.config = topics[0], {"image_mirrored": mirrored}
            ctx = mp.get_context("spawn")
            self.requests, self.replies, self.ready = ctx.Queue(64), ctx.Queue(64), ctx.Queue()
            self.stop_event = ctx.Event()
            self.process = ctx.Process(target=_bridge, args=(self.input_topic, self.prefix + "/feedback", self.requests, self.replies, self.stop_event, self.ready), daemon=True)
            self.process.start()
            try:
                state = self.ready.get(timeout=8)
            except queue.Empty:
                state = {"state": "error", "message": "Motion DDS bridge startup timed out"}
            if state["state"] != "running":
                self.stop()
                return state
            self.runner = MotionRunner(self.client, self.router, self._report, mirrored)
            self.status = state
            self.listener = threading.Thread(target=self._listen, daemon=True, name="q5_host_event_listener")
            self.listener.start()
            return self.info()


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
