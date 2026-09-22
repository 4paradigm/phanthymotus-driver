"""Independent left/right Franka actions, feedback and cancellation through ROS 2."""
import copy
import json
import math
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from common.vendor_runtime import action_schema, tool


def number(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    value = float(value)
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name} outside [{lo}, {hi}]")
    return value


def vector(value, name):
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError(f"{name} must contain seven values")
    return [number(v, name, -1e6, 1e6) for v in value]


def validate_config(config):
    number(config.get("telemetry_hz", 20), "telemetry_hz", 1, 100)
    arms = config.get("arms", {})
    if set(arms) != {"left", "right"}:
        raise ValueError("configure exactly left and right arms")
    endpoints = []
    topics = []
    for side, arm in arms.items():
        joints = arm.get("joints", [])
        if (len(joints) != 7 or len(set(joints)) != 7
                or any(not isinstance(n, str) or not n.strip() for n in joints)):
            raise ValueError(f"{side}: seven distinct joint names required")
        for key in ("trajectory_action", "joint_state_topic"):
            if not isinstance(arm.get(key), str) or not arm[key].startswith("/"):
                raise ValueError(f"{side}: configure an absolute {key}")
        endpoints.append(arm["trajectory_action"])
        topics.append(arm["joint_state_topic"])
        for key in ("desired_joint_state_topic", "eef_pose_topic", "external_wrench_topic"):
            if arm.get(key) and (not isinstance(arm[key], str) or not arm[key].startswith("/")):
                raise ValueError(f"{side}: {key} must be an absolute ROS topic")
        if config.get("motion_enabled") is True:
            for key in ("lower", "upper", "max_velocity", "max_acceleration"):
                vector(arm.get(key), key)
            for lo, hi, vel, acc in zip(arm["lower"], arm["upper"], arm["max_velocity"], arm["max_acceleration"]):
                if lo >= hi or vel <= 0 or acc <= 0:
                    raise ValueError(f"{side}: invalid joint limits")
        if arm.get("gripper_action"):
            if not arm["gripper_action"].startswith("/"):
                raise ValueError("gripper_action must be absolute")
            endpoints.append(arm["gripper_action"])
            number(arm.get("gripper_max_width"), "gripper_max_width", .001, .2)
            number(arm.get("gripper_max_force"), "gripper_max_force", .1, 200)
    if len(set(endpoints)) != len(endpoints) or len(set(topics)) != 2:
        raise ValueError("left/right endpoints and state topics must be distinct")
    for key in ("desired_joint_state_topic", "eef_pose_topic", "external_wrench_topic"):
        if arms["left"].get(key) and arms["left"][key] == arms["right"].get(key):
            raise ValueError(f"left/right {key} must be distinct")


def complete(action_id, status, result, tool_name):
    """Publish ACP with certificate verification and retain a useful failure state."""
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
    body = {"action_id": action_id, "status": status, "result": result,
            "tool": tool_name, "ts": time.time()}
    request = urllib.request.Request(url + "/api/acp/complete", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, context=context, timeout=3) as response:
        ack = json.loads(response.read())
    if ack.get("ok") is not True or ack.get("action_id") != action_id:
        raise RuntimeError("ACP completion was not acknowledged")


class Motion:
    """One action slot. Late acceptance after stop is cancelled, never forgotten."""
    def __init__(self, hardware, kind, name, callback=complete):
        self.hardware, self.kind, self.name = hardware, kind, name
        self.callback = callback
        self.lock = threading.RLock()
        self.active = None
        self.last = None
        self.closed = False

    def start(self, command, timeout):
        with self.lock:
            if self.closed or self.active:
                raise RuntimeError("motion slot is stopped, busy or awaiting cancellation")
            if not self.hardware.ready(self.kind):
                raise RuntimeError("configured action server is unavailable")
            record = {"action_id": uuid4().hex, "handle": None, "cancel": False,
                      "state": "awaiting_acceptance", "timer": None,
                      "goal": copy.deepcopy(command), "submitted_at_ns": time.time_ns()}
            self.active = record
            # Keep the reservation on send failure: submission can have reached
            # the server before transport failure; do not authorize another move.
            try:
                future = self.hardware.send(self.kind, command)
                future.add_done_callback(lambda future: self._accepted(record, future))
            except Exception:
                record["state"] = "submission_unknown"
                record["cancel"] = True
                raise RuntimeError("submission outcome unknown; motion remains locked") from None
            timer = threading.Timer(timeout, self._timeout, args=(record,))
            timer.daemon = True
            record["timer"] = timer
            if self.active is record:
                timer.start()
            return {"state": record["state"], "action_id": record["action_id"]}

    def _accepted(self, record, future):
        with self.lock:
            if self.active is not record:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    self._finish(record, "failed", "controller rejected goal")
                    return
                record["handle"] = handle
                record["accepted_at_ns"] = time.time_ns()
                record["state"] = "running"
                handle.get_result_async().add_done_callback(lambda future: self._result(record, future))
                if record["cancel"] and self.active is record:
                    self._cancel(record)
            except Exception:
                record["state"] = "acceptance_or_result_unknown"
                record["cancel"] = True
                if record["handle"]:
                    self._cancel(record)

    def _cancel(self, record):
        record["state"] = "cancel_requested"
        try:
            record["handle"].cancel_goal_async().add_done_callback(
                lambda future: self._cancel_ack(record, future))
        except Exception:
            record["state"] = "cancel_unconfirmed"

    def _cancel_ack(self, record, future):
        with self.lock:
            if self.active is not record:
                return
            try:
                response = future.result()
                if not response.goals_canceling:
                    record["state"] = "cancel_unconfirmed"
            except Exception:
                record["state"] = "cancel_unconfirmed"
            # An accepted cancellation is not yet a terminal action result.

    def stop(self):
        with self.lock:
            record = self.active
            if record:
                record["cancel"] = True
                if record["handle"]:
                    self._cancel(record)
                else:
                    record["state"] = "cancel_waiting_for_acceptance"
            return self.info()

    def _timeout(self, record):
        with self.lock:
            if self.active is record:
                record["timeout"] = True
                self.stop()

    def _result(self, record, future):
        with self.lock:
            if self.active is not record:
                return
            try:
                packet = future.result()
                if packet.status == 5:
                    status = "failed" if record.get("timeout") else "cancelled"
                    self._finish(record, status, "timeout" if record.get("timeout") else "controller cancelled goal")
                elif packet.status == 4 and not record.get("timeout"):
                    valid = (packet.result.error_code == 0 if self.kind == "arm"
                             else packet.result.reached_goal)
                    self._finish(record, "completed" if valid else "failed", "controller terminal result")
                elif packet.status in (4, 6):
                    self._finish(record, "failed", "controller aborted goal or motion timed out")
                else:
                    record["state"] = "unexpected_result_status"
                    self.stop()
            except Exception:
                record["state"] = "result_unknown"
                self.stop()

    def _finish(self, record, status, reason):
        if record.get("timer"):
            record["timer"].cancel()
        self.active = None
        self.last = {"action_id": record["action_id"], "status": status,
                     "result": {"reason": reason}, "callback": "pending",
                     "goal": record["goal"], "submitted_at_ns": record["submitted_at_ns"],
                     "completed_at_ns": time.time_ns()}
        report = self.last
        threading.Thread(target=self._report, args=(report,), daemon=True).start()

    def _report(self, report):
        try:
            self.callback(report["action_id"], report["status"], report["result"], self.name)
            outcome = "acknowledged"
        except Exception:
            outcome = "failed"
        with self.lock:
            report["callback"] = outcome

    def info(self):
        with self.lock:
            return {"state": self.active["state"] if self.active else "idle",
                    "action_id": self.active["action_id"] if self.active else None,
                    "goal": copy.deepcopy(self.active["goal"]) if self.active else None,
                    "submitted_at_ns": self.active["submitted_at_ns"] if self.active else None,
                    "accepted_at_ns": self.active.get("accepted_at_ns") if self.active else None,
                    "last_completion": copy.deepcopy(self.last)}


class FrankaPlugin:
    def __init__(self, config, hardware, callback=complete, *, ros2=None, namespace="franka"):
        validate_config(config)
        self.cfg, self.hardware = config, hardware
        self._lock = threading.RLock()
        self._closed = False
        self._ros2 = ros2
        self._telemetry_node = None
        self._publishers = {}
        self._stream_sides = set()
        self._telemetry_errors = {}
        self._topics = {side: f"/{namespace.strip('/') or 'franka'}/franka/{side}/state"
                        for side in ("left", "right")}
        self.motions = {}
        for side, hw in hardware.items():
            self.motions[f"franka_{side}_arm"] = Motion(hw, "arm", f"franka_{side}_arm", callback)
            if config["arms"][side].get("gripper_action"):
                name = f"franka_{side}_gripper"
                self.motions[name] = Motion(hw, "gripper", name, callback)

    def start(self):
        # Read-only telemetry never homes or moves a robot.
        with self._lock:
            if self._closed or self._telemetry_node is not None or self._ros2 is None:
                return
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import String
            self._message_type = String
            node = Node("franka_telemetry", context=self._ros2.ctx_core)
            self._publishers = {side: node.create_publisher(String, topic, qos_profile_sensor_data)
                                for side, topic in self._topics.items()}
            node.create_timer(1 / self.cfg.get("telemetry_hz", 20), self._publish_telemetry)
            self._telemetry_node = node
            self._stream_sides.update(("left", "right"))
            self._ros2.executor_core.add_node(node)

    def snapshot(self, side):
        observations = self.hardware[side].telemetry()
        # These channels have independent source timestamps. A JSON envelope is
        # not evidence of a synchronized sensor acquisition.
        return {"format": "data/json", "timestamp_ms": time.time_ns() // 1_000_000,
                "side": side, "units": {"position": "rad", "velocity": "rad/s", "effort": "Nm"},
                "observations": observations, "motion": self.motions[f"franka_{side}_arm"].info(),
                "gripper": (self.motions[f"franka_{side}_gripper"].info()
                            if f"franka_{side}_gripper" in self.motions else None)}

    def _publish_telemetry(self):
        with self._lock:
            if self._closed:
                return
            for side in self._stream_sides:
                try:
                    message = self._message_type()
                    message.data = json.dumps(self.snapshot(side), allow_nan=False)
                    self._publishers[side].publish(message)
                    self._telemetry_errors.pop(side, None)
                except Exception:
                    self._telemetry_errors[side] = "telemetry unavailable"

    def _topic_out(self, side):
        return [{"topic": self._topics[side], "format": "data/json"}]

    def stop(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stream_sides.clear()
            if self._telemetry_node is not None:
                self._ros2.executor_core.remove_node(self._telemetry_node)
                self._telemetry_node.destroy_node()
                self._telemetry_node = None
            for motion in self.motions.values():
                with motion.lock:
                    motion.closed = True
                    motion.stop()
        # Keep ROS alive briefly so native cancellation/results can be received.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and any(m.info()["action_id"] for m in self.motions.values()):
            threading.Event().wait(.02)
        if any(m.info()["action_id"] for m in self.motions.values()):
            print("[franka] shutdown: controller cancellation unconfirmed; physical stop is not established", flush=True)
        for hw in self.hardware.values():
            hw.close()

    def get_tools(self):
        definitions = []
        for side in ("left", "right"):
            definitions.append(tool(f"franka_{side}_state", "sensor", f"{side} measured joints (rad, rad/s)",
                                    action_schema({"get": ([], "Read fresh measured state"),
                                                   "start": ([], "Publish timed telemetry"),
                                                   "stop": ([], "Pause telemetry; leave motion unchanged"),
                                                   "diagnostics": ([], "Measured/desired joints, TCP, estimated wrench and goal status"),
                                                   "info": ([], "Controller and stream availability")}, {}),
                                    topic_out=self._topic_out(side)))
        for name, motion in self.motions.items():
            side = name.split("_")[1]
            arm = motion.kind == "arm"
            params = ["joint", "duration"] if arm else ["width", "force"]
            properties = ({"joint": {"type": "array", "items": {"type": "number"}, "minItems": 7, "maxItems": 7},
                           "duration": {"type": "number", "minimum": .5, "maximum": 30}}
                          if arm else {"width": {"type": "number", "description": "Full opening, metres"},
                                       "force": {"type": "number", "description": "Newtons"}})
            schema = action_schema({"move": (params, "Submit goal; completion comes from the controller"),
                                    "stop": ([], "Cancel this driver's active goal"),
                                    "start": ([], "Status only; no motion"), "info": ([], "Goal status")}, properties)
            schema["x-completion"] = {"actions": ["move"], "timeout": 40}
            schema["x-resource"] = ("arm_l" if side == "left" else "arm_r")
            schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"},
                                  "on_interrupt_all": {"action": "stop"}}
            definitions.append(tool(name, "actuator", f"Franka {side} {motion.kind}", schema))
        return definitions

    def dispatch(self, action, args):
        name = args.get("_tool_name", "")
        if name not in {d["name"] for d in self.get_tools()}:
            raise ValueError("unknown Franka tool")
        side = name.split("_")[1]
        hw, cfg = self.hardware[side], self.cfg["arms"][side]
        if name.endswith("_state"):
            if action == "get":
                return hw.state()
            if action == "diagnostics":
                return self.snapshot(side)
            if action == "start":
                self.start()
                with self._lock:
                    if self._closed or self._telemetry_node is None:
                        raise RuntimeError("ROS telemetry publisher unavailable")
                    self._stream_sides.add(side)
                    return {"state": "streaming", "topic_out": self._topic_out(side)}
            if action == "info":
                with self._lock:
                    return {"arm_controller_ready": hw.ready("arm"), "gripper_ready": hw.ready("gripper"),
                            "streaming": side in self._stream_sides,
                            "topic_out": self._topic_out(side), "publish_error": self._telemetry_errors.get(side)}
            if action == "stop":
                with self._lock:
                    self._stream_sides.discard(side)
                    return {"state": "idle"}
        motion = self.motions.get(name)
        if motion:
            if action in ("start", "info"):
                return motion.info()
            if action == "stop":
                # Serialize with move preflight as well as goal submission.
                # Otherwise stop can see an empty slot while an older move is
                # still reading feedback, then that move submits after stop.
                with self._lock:
                    return motion.stop()
            if action == "move":
                with self._lock:
                    if self._closed or self.cfg.get("motion_enabled") is not True:
                        raise RuntimeError("motion is disabled")
                    if motion.kind == "arm":
                        target = vector(args.get("joint"), "joint")
                        duration = number(args.get("duration"), "duration", .5, 30)
                        state = hw.state()
                        start = vector(state["position"], "measured position")
                        velocity = vector(state["velocity"], "measured velocity")
                        if any(abs(v) > .02 for v in velocity):
                            raise RuntimeError("arm must be stationary before starting a point-to-point trajectory")
                        for i, (q0, q1) in enumerate(zip(start, target)):
                            if not cfg["lower"][i] <= q0 <= cfg["upper"][i] or not cfg["lower"][i] <= q1 <= cfg["upper"][i]:
                                raise ValueError(f"joint {i + 1} outside configured limits")
                            distance = abs(q1 - q0)
                            if (1.875 * distance / duration > cfg["max_velocity"][i]
                                    or (10 / math.sqrt(3)) * distance / duration ** 2 > cfg["max_acceleration"][i]):
                                raise ValueError("duration is too short for configured quintic velocity/acceleration limits")
                        command = {"start": start, "target": target, "duration": duration}
                        return motion.start(command, duration + 3)
                    width = number(args.get("width"), "width", 0, cfg["gripper_max_width"])
                    force = number(args.get("force"), "force", .1, cfg["gripper_max_force"])
                    return motion.start({"width": width, "force": force}, 15)
        raise ValueError("unsupported action")


def build_plugins(config, namespace, ros2):
    from hardware import RosArm
    cfg = config["franka"]
    validate_config(cfg)
    hardware = {side: RosArm(ros2, arm, side) for side, arm in cfg["arms"].items()}
    return [FrankaPlugin(cfg, hardware, ros2=ros2, namespace=namespace)]
