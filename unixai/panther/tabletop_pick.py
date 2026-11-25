"""One-shot tabletop pick with rectified RGB-D and Perception's VOP photo action."""
from __future__ import annotations

import http.client
import json
import math
import os
import threading
import time
import zlib
from typing import NamedTuple
from uuid import uuid4

import numpy as np

from common.vendor_runtime import tool
from contracts import (HEAD_JOINTS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, byte_value,
                       finite_number, finite_vector, fresh, intersect_joint_limits, validate_joint_target)
from device import _notify_completion
from spatial_inputs import stamp_ns

CARD = "tabletop_pick"
MAX_AGE_S = 1.0
MAX_SKEW_S = 0.15
MAX_VOP_RESPONSE = 1024 * 1024
MAX_OBSERVATION_AGE_S = 4.0


class VopSnapshot:
    """Use Perception's existing upload and single-image VOP tools on loopback."""

    def __init__(self, port=15720):
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("perception_port must be a valid TCP port")
        self._port = port

    def _post(self, path, body, headers):
        connection = http.client.HTTPConnection("127.0.0.1", self._port, timeout=5)
        try:
            connection.request("POST", path, body, headers)
            with connection.getresponse() as response:
                if response.status != 200:
                    raise RuntimeError(f"Perception {path} returned HTTP {response.status}")
                raw = response.read(MAX_VOP_RESPONSE + 1)
                if len(raw) > MAX_VOP_RESPONSE:
                    raise RuntimeError("Perception response is too large")
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("JSON object required")
            return data
        except (OSError, ValueError, http.client.HTTPException) as exc:
            raise RuntimeError(f"Perception request failed at {path}") from exc
        finally:
            connection.close()

    def recognize(self, jpeg, confidence):
        boundary = uuid4().hex
        filename = f"panther-pick-{uuid4().hex}.jpg"
        prefix = (f"--{boundary}\r\n"
                  'Content-Disposition: form-data; name="subdir"\r\n\r\n'
                  f"panther-tabletop-pick\r\n--{boundary}\r\n"
                  f'Content-Disposition: form-data; name="file"; filename="{filename}"\r\n'
                  "Content-Type: image/jpeg\r\n\r\n").encode()
        headers = {"Content-Type": f"multipart/form-data; boundary={boundary}"}
        token = os.environ.get("ACCESS_TOKEN", "").strip()
        if token:
            headers["X-Access-Token"] = token
        upload = self._post("/file/upload", prefix + jpeg + f"\r\n--{boundary}--\r\n".encode(), headers)
        path = upload.get("path")
        if upload.get("ok") is not True or not isinstance(path, str) or not path.startswith("/"):
            raise RuntimeError("Perception did not return an uploaded image path")
        request = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                   "params": {"name": "vop", "arguments": {
                       "action": "recognize_by_photo", "image_path": path, "confidence": confidence}}}
        response = self._post("/mcp", json.dumps(request).encode(), {"Content-Type": "application/json"})
        try:
            detection = json.loads(response["result"]["content"][0]["text"])
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise RuntimeError("Perception did not return a structured VOP result") from exc
        if not isinstance(detection, dict) or detection.get("ok") is not True or detection.get("source") != path:
            raise RuntimeError("VOP did not recognize the uploaded observation")
        return detection


class PickSetup(NamedTuple):
    optical_frame: str
    base_from_color: np.ndarray
    bounds: np.ndarray
    orientation: list[float]
    head_rad: list[float]
    lift_position: float
    clearance: float


def load_setup(config):
    frame = config.get("color_optical_frame")
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError("configure the calibrated color_optical_frame")
    transform = np.array(finite_vector(config.get("base_from_color"), "base_from_color", 16)).reshape(4, 4)
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4)):
        raise ValueError("base_from_color must be a row-major rigid transform")
    bounds = np.array(finite_vector(config.get("workspace_m"), "workspace_m", 6)).reshape(2, 3)
    if np.any(bounds[0] >= bounds[1]):
        raise ValueError("workspace_m requires [min_x, min_y, min_z, max_x, max_y, max_z]")
    orientation = finite_vector(config.get("tool_orientation_xyzw"), "tool_orientation_xyzw", 4)
    if not np.isclose(np.linalg.norm(orientation), 1, atol=1e-4):
        raise ValueError("tool_orientation_xyzw must be a unit quaternion")
    head = finite_vector(config.get("head_rad"), "head_rad", 2)
    lift = finite_number(config.get("lift_position"), "lift_position")
    clearance = finite_number(config.get("clearance_m"), "clearance_m")
    if not 0.02 <= clearance <= 0.25:
        raise ValueError("clearance_m must be within [0.02, 0.25]")
    side = config.get("side")
    group, link = config.get("moveit_group"), config.get("tool_link")
    if side not in ("left", "right") or not all(isinstance(s, str) and s.strip() for s in (group, link)):
        raise ValueError("configure side, moveit_group and tool_link")
    return PickSetup(frame, transform, bounds, orientation, head, lift, clearance)


def select_object(data, name, color, confidence):
    objects = data.get("objects")
    if not isinstance(objects, list) or len(objects) > 1000:
        raise ValueError("VOP objects payload is invalid")
    matches = []
    for obj in objects:
        if not isinstance(obj, dict) or str(obj.get("name", "")).strip().casefold() != name.casefold():
            continue
        label = obj.get("color")
        if isinstance(label, dict):
            label = label.get("color_name") or label.get("dominant_hue")
        if color and color.casefold() not in str(label or "").casefold().split():
            continue
        if finite_number(obj.get("confidence"), "confidence") >= confidence:
            matches.append(obj)
    if len(matches) != 1:
        raise ValueError(f"expected one matching VOP detection, found {len(matches)}")
    return matches[0]


def grasp_points(box, payload, camera_matrix, setup, surface_offset):
    width, height = 640, 480
    x1, y1, x2, y2 = finite_vector(box, "VOP bbox", 4)
    if not (0 < x1 < x2 < width and 0 < y1 < y2 < height):
        raise ValueError("target bbox is clipped or outside the calibrated image")
    decoder = zlib.decompressobj()
    byte_count = width * height * 2
    raw = decoder.decompress(payload, byte_count + 1)
    if not decoder.eof or decoder.unused_data or len(raw) != byte_count:
        raise ValueError("invalid Panther depth frame")
    depth = np.frombuffer(raw, dtype="<u2").reshape(height, width)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    u, v = int(cx), int(cy)
    rx, ry = min(4, int((x2 - x1) / 4)), min(4, int((y2 - y1) / 4))
    if min(rx, ry) < 1:
        raise ValueError("target is too small for a depth sample")
    patch = depth[max(0, v - ry):v + ry + 1, max(0, u - rx):u + rx + 1]
    valid = patch[(patch >= 100) & (patch <= 3000)]
    if valid.size < 0.6 * patch.size or np.percentile(valid, 90) - np.percentile(valid, 10) > 45:
        raise ValueError("target center has missing or inconsistent depth")
    z = float(np.median(valid)) / 1000
    fx, fy, px, py = camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]
    point = (setup.base_from_color @ [(cx - px) * z / fx, (cy - py) * z / fy, z, 1.0])[:3]
    point[2] += surface_offset
    above = point + [0, 0, setup.clearance]
    if np.any(point < setup.bounds[0]) or np.any(above > setup.bounds[1]):
        raise ValueError("pick or clearance point is outside the configured workspace")
    return point.tolist(), above.tolist()


class TabletopPick:
    PREFIX = CARD

    def __init__(self, config, device, *, vop=None, notify=_notify_completion, motion=None):
        self._config, self._device = config, device
        self._motion = motion if motion is not None else PickMotion(device, config)
        self._vop = vop if vop is not None else VopSnapshot(config.get("perception_port", 15720))
        self._notify = notify
        self._lock = threading.Lock()
        self._started = False
        self._generation = 0
        self._active = None
        self._last = None
        self._done = threading.Event()
        self._done.set()

    def get_tool(self):
        actions = ["start", "info", "preview", "pick", "stop"]
        return tool(self.PREFIX, "actuator",
                    "按现有 VOP 类别与可选颜色单次抓取桌面物体。preview 只计算抓取点；pick 抓起后保持夹爪闭合。",
                    {"type": "object", "properties": {
                        "action": {"type": "string", "enum": actions},
                        "object_name": {"type": "string", "maxLength": 128},
                        "color": {"type": "string", "maxLength": 64},
                        "confirm_motion": {"type": "boolean", "const": True}},
                     "required": ["action"], "additionalProperties": False,
                     "x-action-params": {
                         "preview": {"params": ["object_name", "color"]},
                         "pick": {"params": ["object_name", "color", "confirm_motion"]}},
                     "x-resource": ["base", "waist", "head", "arm_l", "arm_r", "hand_l", "hand_r"],
                     "x-completion": {"actions": ["pick"], "timeout": 150},
                     "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                                 "on_interrupt_all": {"action": "stop"}}})

    def _info(self):
        try:
            load_setup(self._config)
            setup = "configured"
        except ValueError as exc:
            setup = str(exc)
        return {"state": "ready" if self._started else "idle",
                "calibration": setup, "allow_motion": self._config.get("allow_motion") is True,
                "busy": self._active is not None, "last_result": self._last}

    def start(self):
        pass  # Driver boot does not arm the card or request a camera frame.

    def dispatch(self, action, args):
        try:
            with self._lock:
                if action == "info":
                    return self._info()
                if action == "stop":
                    return self._stop()
                if action == "start":
                    if self._active is not None:
                        raise RuntimeError("pick is still active")
                    self._started = True
                    return self._info()
                if action not in ("preview", "pick"):
                    raise ValueError(f"unknown action {action!r}")
                if not self._started or self._active is not None:
                    raise RuntimeError("start tabletop_pick first and wait for any active pick to finish")
                generation = self._generation
                if action == "pick":
                    if args.get("confirm_motion") is not True:
                        raise ValueError("confirm_motion=true is required")
                    if self._config.get("allow_motion") is not True:
                        raise RuntimeError("tabletop_pick.allow_motion is disabled")
                    return self._start_pick(args)
            plan = self._preview(args)
            with self._lock:
                if generation != self._generation or self._active is not None:
                    raise RuntimeError("preview was superseded")
            return {"state": "ready", **plan}
        except (ValueError, RuntimeError, TimeoutError, zlib.error) as exc:
            return {"state": "error", "message": str(exc)}

    def _start_pick(self, args):
        cancel = threading.Event()
        self._motion.reserve_pick(cancel)
        action_id = str(args.get("action_id") or f"panther_pick_{uuid4().hex[:12]}")
        self._active = cancel
        self._done.clear()
        try:
            threading.Thread(target=self._run, args=(action_id, dict(args), cancel),
                             name=CARD, daemon=True).start()
        except Exception:
            self._active = None
            self._done.set()
            self._motion.release_pick(cancel)
            raise
        return {"state": "picking", "action_id": action_id}

    def _preview(self, args):
        setup = load_setup(self._config)
        name, color = args.get("object_name"), args.get("color", "")
        if not isinstance(name, str) or not name.strip() or not isinstance(color, str):
            raise ValueError("object_name must match a VOP class; color is optional")
        name, color = name.strip(), color.strip()
        profiles = self._config.get("objects") or {}
        profile = profiles.get(name.casefold())
        if not isinstance(profile, dict):
            raise ValueError(f"no commissioned grasp profile for {name!r}")
        confidence = finite_number(self._config.get("min_confidence", 0.5), "min_confidence")
        offset = finite_number(profile.get("surface_to_tcp_z_m"), "surface_to_tcp_z_m")
        opened = finite_number(profile.get("gripper_open"), "gripper_open")
        closed = finite_number(profile.get("gripper_close"), "gripper_close")
        if not (0 < confidence <= 1 and abs(offset) <= 0.15 and 0 <= opened <= 1 and
                0 <= closed <= 1 and opened != closed):
            raise ValueError("invalid confidence, tool offset or gripper settings")
        self._guard(setup)
        rgb, depth, camera_matrix = self._device._camera.pick_snapshot(MAX_AGE_S, MAX_SKEW_S)
        if rgb["frame_id"] != setup.optical_frame:
            raise ValueError("rectified frames do not match the calibrated color_optical_frame")
        data = self._vop.recognize(rgb["data"], confidence)
        self._guard(setup)
        if data.get("image_size") != [640, 480]:
            raise ValueError("VOP image dimensions differ from the calibrated observation")
        if time.monotonic() - rgb["received"] > MAX_OBSERVATION_AGE_S:
            raise RuntimeError("observation expired during VOP inference")
        obj = select_object(data, name, color, confidence)
        point, above = grasp_points(obj.get("bbox"), depth["data"], camera_matrix, setup, offset)
        return {"object_name": name, "color": color, "bbox": obj["bbox"], "point_m": point, "above_m": above,
                "gripper_open": opened, "gripper_close": closed}

    def _guard(self, setup):
        self._motion.guard(setup.head_rad, setup.lift_position)

    def _run(self, action_id, args, cancel):
        result = {"grasp_checked": False, "stage": "observe"}
        status = "error"
        attempted_grip = False
        try:
            setup = load_setup(self._config)
            side = self._config["side"]

            def guard():
                if cancel.is_set():
                    raise RuntimeError("pick cancelled")
                self._guard(setup)

            def move(stage, point, before_send=None):
                result["stage"] = stage
                self._motion.pick_move(side, point, setup.orientation, guard, before_send)

            def check_target():
                guard()
                fresh = self._preview(args)
                guard()
                if np.linalg.norm(np.subtract(plan["point_m"], fresh["point_m"])) > 0.015:
                    raise RuntimeError("target moved since preview; request another pick")

            guard()
            plan = self._preview(args)
            guard()
            result.update(object_name=plan["object_name"], point_m=plan["point_m"], stage="open")
            self._motion.pick_gripper(side, plan["gripper_open"], opening=True, guard=guard, owner=cancel)
            move("pregrasp", plan["above_m"], check_target)
            move("descend", plan["point_m"])
            result["stage"] = "close"
            attempted_grip = True
            self._motion.pick_gripper(side, plan["gripper_close"], opening=False, guard=guard, owner=cancel)
            move("lift", plan["above_m"])
            result["stage"] = "lifted"
            status = "completed"
        except Exception as exc:
            status = "cancelled" if cancel.is_set() else "error"
            result["message"] = str(exc)
        finally:
            result["may_be_holding"] = attempted_grip
            self._motion.release_pick(cancel)
            with self._lock:
                self._active = None
                self._last = {"action_id": action_id, "status": status, **result}
                self._done.set()
            self._notify(action_id, status, result, CARD)

    def _stop(self):
        self._started = False
        self._generation += 1
        if self._active is not None:
            self._active.set()
        return {"state": "stopping" if self._active is not None else "idle"}

    def stop(self):
        with self._lock:
            self._stop()
        if not self._done.wait(24):
            print("[panther] tabletop_pick stop timed out during driver shutdown", flush=True)


class PickMotion:
    """Plan Cartesian moves and execute them through Panther's existing ROS clients."""

    def __init__(self, device, config):
        self._device, self._config = device, config
        self._planner = None

    def _joints(self):
        msg, received = self._device._get("joints")
        if msg is None or not fresh(received, 0.5):
            raise RuntimeError("fresh /joint_states is required for tabletop_pick")
        return dict(zip(msg.name, (finite_number(float(value), "joint position") for value in msg.position)))

    def guard(self, head, lift):
        joints = self._joints()
        lift_msg, received = self._device._get("lifting")
        if (lift_msg is None or not fresh(received, 0.5) or not lift_msg.position
                or abs(finite_number(float(lift_msg.position[0]), "lift") - lift) > 0.005
                or any(name not in joints or abs(joints[name] - value) > 0.015
                       for name, value in zip(HEAD_JOINTS, head))):
            raise RuntimeError("head or lift differs from the calibrated observation pose")
        odom, received = self._device._get("odom")
        if odom is None or not fresh(received, 0.2) or stamp_ns(odom) <= 0:
            raise RuntimeError("fresh stationary odometry is required for tabletop_pick")
        twist = odom.twist.twist
        speeds = (twist.linear.x, twist.linear.y, twist.angular.z)
        if any(abs(finite_number(value, "base velocity")) > cap
               for value, cap in zip(speeds, (0.01, 0.01, 0.02))):
            raise RuntimeError("base must remain stationary for tabletop_pick")

    def _ready(self):
        self._device._ensure_motion_ready()
        diagnostics, received = self._device._get("diagnostics")
        name = self._config.get("estop_diagnostic_name")
        if not isinstance(name, str) or not name.strip():
            raise RuntimeError("configure the verified emergency-stop diagnostic name")
        matches = [item for item in getattr(diagnostics, "status", ()) if item.name == name]
        if not fresh(received, 3.0) or len(matches) != 1 or byte_value(matches[0].level, "emergency stop") != 0:
            raise RuntimeError("tabletop_pick requires a fresh, normal emergency-stop diagnostic")

    def reserve_pick(self, cancel):
        self._ready()
        self._device.claim_tabletop(cancel)

    def release_pick(self, cancel):
        self._device.release_tabletop(cancel)

    def pick_gripper(self, side, position, *, opening, guard, owner):
        device = self._device
        endpoints = finite_vector(self._config.get("gripper_feedback_open_closed"),
                                  "gripper_feedback_open_closed", 2)
        opened, closed = endpoints
        span = closed - opened
        if abs(span) < 1e-6:
            raise ValueError("gripper feedback endpoints must be distinct")

        def fraction(msg):
            if msg is None or not msg.position:
                raise RuntimeError("gripper position feedback is unavailable")
            value = finite_number(float(msg.position[0]), "gripper feedback")
            return (value - opened) / span

        msg, received = device._get(f"gripper_{side}")
        if not fresh(received, 0.5) or not -0.05 <= fraction(msg) <= 1.05:
            raise RuntimeError("gripper feedback is stale or outside the calibrated range")
        guard()
        self._ready()
        sent = time.monotonic()
        device.gripper({"action": "set", "side": side, "position": position}, owner=owner)
        previous = settled_since = None
        while time.monotonic() - sent < 3.0:
            guard()
            self._ready()
            msg, received = device._get(f"gripper_{side}")
            if received is not None and received > sent and fresh(received, 0.5):
                value = fraction(msg)
                acceptable = -0.05 <= value <= 0.05 if opening else 0.05 < value <= 1.05
                if not acceptable or previous is None or abs(value - previous) > 0.015:
                    previous, settled_since = value, received if acceptable else None
                elif settled_since is not None and received - settled_since >= 0.2:
                    return
            time.sleep(0.05)
        raise RuntimeError("gripper did not reach a stable position after the command")

    def _cartesian_path(self, current, point, orientation):
        from geometry_msgs.msg import Pose
        from moveit_msgs.srv import GetCartesianPath

        if self._planner is None:
            self._planner = self._device._node.create_client(
                GetCartesianPath, self._config.get("cartesian_service", "/compute_cartesian_path"))
        if not self._planner.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("MoveIt Cartesian path service is unavailable on the robot domain")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.group_name = self._config["moveit_group"]
        request.link_name = self._config["tool_link"]
        request.start_state.is_diff = True
        request.start_state.joint_state.name = list(current)
        request.start_state.joint_state.position = list(current.values())
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        request.waypoints = [pose]
        request.max_step = 0.005
        request.jump_threshold = 2.0
        request.avoid_collisions = True
        response = self._device._wait_future(self._planner.call_async(request), 5.0)
        if response.error_code.val != 1 or not 0.999999 <= response.fraction <= 1.0:
            raise RuntimeError(f"MoveIt did not return a complete collision-checked path ({response.fraction:.3f})")
        if response.solution.multi_dof_joint_trajectory.points:
            raise RuntimeError("tabletop_pick requires a fixed-base single-arm path")
        return response.solution.joint_trajectory

    def pick_move(self, side, point, orientation, guard, before_send=None):
        from control_msgs.action import FollowJointTrajectory

        device = self._device
        guard()
        self._ready()
        current = self._joints()
        names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        limits = intersect_joint_limits(names, device._model_joint_limits, device._joint_limits)
        if any(name not in current for name in names):
            raise RuntimeError("fresh feedback is required for all planned arm joints")
        trajectory = self._cartesian_path(current, point, orientation)
        duration = validate_path(trajectory, names, limits, current, device._cfg["plugins"]["arm_control"])
        states = device.controller_states()
        controller = f"{side}_arm_joint_trajectory_position_controller"
        forward = "dual_arm_forward_position_controller"
        device._require_controllers(states, (controller, forward))
        activate = [] if states[controller] == "active" else [controller]
        deactivate = [forward] if states[forward] == "active" else []
        client = device._arm_actions[side]
        handle = None
        result_future = None
        finished = False
        switched = False
        try:
            device._switch_controllers_exact(activate, deactivate)
            switched = bool(activate or deactivate)
            if not client.wait_for_server(timeout_sec=3.0):
                raise RuntimeError(f"{side} FollowJointTrajectory server is unavailable")
            if before_send is not None:
                before_send()
            guard()
            self._ready()
            latest = self._joints()
            if any(name not in latest or abs(latest[name] - value) > 0.025
                   for name, value in current.items()):
                raise RuntimeError("joint feedback changed while planning")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            pending = client.send_goal_async(goal)
            try:
                handle = device._wait_future(pending, 5.0)
            except TimeoutError:
                def cancel_late(future):
                    if future.exception() is None and future.result().accepted:
                        future.result().cancel_goal_async()

                pending.add_done_callback(cancel_late)
                raise
            if not handle.accepted:
                raise RuntimeError("tabletop trajectory goal was rejected")
            result_future = handle.get_result_async()
            arrived = threading.Event()
            result_future.add_done_callback(lambda _: arrived.set())
            deadline = time.monotonic() + duration + 3.0
            while not arrived.wait(0.15):
                guard()
                self._ready()
                if time.monotonic() >= deadline:
                    raise TimeoutError("tabletop trajectory did not finish")
            outcome = result_future.result()
            finished = True
            if outcome.status != 4 or outcome.result.error_code != 0:
                raise RuntimeError(f"tabletop trajectory failed: {outcome.result.error_string}")
            guard()
        finally:
            try:
                if handle is not None and handle.accepted and not finished:
                    device._wait_future(handle.cancel_goal_async(), 2.0)
                    if result_future is not None:
                        device._wait_future(result_future, 3.0)
            finally:
                if switched:
                    device._switch_controllers_exact(deactivate, activate)


def validate_path(trajectory, names, limits, current, arm_config):
    """Validate all planner waypoints and consistently scale timing and derivatives."""
    ordered = tuple(trajectory.joint_names)
    if len(ordered) != len(names) or set(ordered) != set(names) or len(trajectory.points) < 2:
        raise RuntimeError("MoveIt returned an empty trajectory or unexpected joints")
    max_step = math.radians(finite_number(arm_config["max_target_delta_deg"], "arm joint step"))
    speed = min(0.4, math.radians(finite_number(arm_config["speed_deg_s"], "arm speed")))
    max_duration = finite_number(arm_config["max_duration_s"], "arm duration")
    if speed <= 0 or max_duration <= 0:
        raise ValueError("invalid arm speed or duration cap")
    previous, last, slowdown = current, None, 1.0
    samples = []
    for sample in trajectory.points:
        stamp = sample.time_from_start
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if seconds < 0 or (last is not None and seconds <= last):
            raise RuntimeError("invalid Cartesian trajectory timing")
        positions = validate_joint_target(ordered, list(sample.positions), limits, previous, max_step)
        delta = max(abs(value - previous[name]) for name, value in zip(ordered, positions))
        if last is None:
            if delta > 0.025 or (seconds == 0 and delta > 0.002):
                raise RuntimeError("Cartesian trajectory does not start at the observed arm position")
            interval = seconds
        else:
            interval = seconds - last
        if interval > 0:
            slowdown = max(slowdown, delta / (speed * interval))
        velocities = finite_vector(list(sample.velocities), "trajectory velocities", len(ordered)) if sample.velocities else []
        accelerations = finite_vector(list(sample.accelerations), "trajectory accelerations", len(ordered)) if sample.accelerations else []
        if sample.effort:
            raise RuntimeError("tabletop trajectories must not command effort")
        slowdown = max(slowdown, max(map(abs, velocities), default=0.0) / speed)
        samples.append((sample, seconds, velocities, accelerations))
        previous, last = dict(zip(ordered, positions)), seconds
    duration = last * slowdown
    if not 0.15 <= duration <= max_duration:
        raise RuntimeError("Cartesian trajectory duration is outside the tabletop limits")
    for sample, seconds, velocities, accelerations in samples:
        sec, nsec = divmod(round(seconds * slowdown * 1e9), 1_000_000_000)
        sample.time_from_start.sec, sample.time_from_start.nanosec = sec, nsec
        sample.velocities = [value / slowdown for value in velocities]
        sample.accelerations = [value / slowdown**2 for value in accelerations]
    return duration
