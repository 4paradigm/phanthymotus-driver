"""RealMan-compatible tabletop transfer contract backed by Panther hardware."""

import copy
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from uuid import uuid4

import numpy as np

from base_servo import _receive
from contracts import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, fresh, intersect_joint_limits
from device import _notify_completion


NAME = "vision_pick_and_drop"
MOTION = ("observe", "grab_to", "grab_by")
START = ("start_point_x", "start_point_y")
TARGET = ("target_point_x", "target_point_y")
DELTA = ("delta_x", "delta_y")


def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"expected a finite number in [{low}, {high}]")
    return float(value)


def object_list(data):
    items = data["objects"]
    if not isinstance(items, list) or len(items) > 1000 or data.get("count") != len(items):
        raise ValueError("invalid VOP object list")
    result = []
    for item in items:
        x, y = item["position"]
        name = item["name"]
        if not isinstance(name, str) or len(name.encode("utf-8")) > 256:
            raise ValueError("invalid VOP name")
        result.append(dict(name=name, position=[number(x, -1, 1), number(y, -1, 1)],
                           confidence=number(item["confidence"], 0, 1)))
    return result


def resolve_targets(action, args, snapshot, cfg):
    depth, k, transform = (snapshot[key] for key in ("depth", "K", "transform"))
    height, width = depth.shape

    def project(keys):
        x, y = (number(args.get(key), -1, 1) for key in keys)
        u, v = min(round((x + 1) * width / 2), width - 1), min(round((y + 1) * height / 2), height - 1)
        z = float(depth[v, u]) * .001
        if z <= 0:
            raise ValueError("selected pixel has no valid depth")
        point = z * np.linalg.solve(k, [u, v, 1.])
        base_point = transform[:3, :3] @ point + transform[:3, 3]
        return base_point[:2] + cfg["xy_compensation_m"], [u, v]

    a, pixel = project(START)
    angle = number(args.get("rotation_deg", 0), -180, 180)
    if action == "grab_to":
        b, place_pixel = project(TARGET)
        details = {"place_pixel": place_pixel}
    else:
        x, y = (number(args.get(key), -1000, 1000) for key in DELTA)
        if not (x or y or angle):
            raise ValueError("displacement or rotation is required")
        # Panther camera installation is not RealMan's fixed XY mapping.
        # Project image right onto the horizontal plane and select the image-down sign.
        right = transform[:2, 0].copy()
        if np.linalg.norm(right) < .2:
            raise ValueError("image right cannot be resolved in the tabletop plane")
        right /= np.linalg.norm(right)
        down = np.array([-right[1], right[0]])
        direction = float(down @ transform[:2, 1])
        if abs(direction) < .2:
            raise ValueError("image down cannot be resolved in the tabletop plane")
        b = a + .001 * (x * right + y * math.copysign(1., direction) * down)
        details = {"delta_x": x, "delta_y": y, "direction_reference": "observation_image"}
    return [a.tolist(), b.tolist()], angle, {**details, "pick_pixel": pixel}


class VisionPickAndDrop:
    name = NAME

    def __init__(self, device, config):
        self.device, self.cfg = device, dict(config)
        self._config_error = None
        if config.get("calibration_file"):
            path = Path(config["calibration_file"])
            try:
                settings = json.loads(path.read_text())
                keys = {"allow_motion", "side", "base_frame", "tcp_frame", "observation_joints_deg",
                        "workspace_min_m", "workspace_max_m", "xy_compensation_m", "pick_descent_m",
                        "place_descent_m", "observe_after_transfer", "gripper_open_feedback",
                        "gripper_feedback_tolerance"}
                if not isinstance(settings, dict) or settings.keys() - keys:
                    raise ValueError("unknown or invalid calibration fields")
                self.cfg.update(settings)
            except FileNotFoundError:
                pass
            except (OSError, ValueError) as exc:
                self._config_error = str(exc)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._source = self._snapshot = self._last = self._worker = None
        self._blocked = False
        self._stop_requested = False
        self._ctx = mp.get_context("spawn")
        self._has_vop = False

    def get_tool(self):
        topics = self.device.topics
        actions = ("start", "info", *MOTION, "cancel", "stop")
        return {"name": NAME, "type": "actuator", "multiInstance": False,
                "description": "Panther tabletop transfer: observe, then grab_to or grab_by using the same RGB-D observation",
                "topic_in": [topics["camera_depth"], topics["camera_color"], {"format": "data/json", "desc": "VOP objects"}],
                "inputSchema": {"type": "object", "required": ["action"], "additionalProperties": False,
                    "properties": {"action": {"type": "string", "enum": list(actions)},
                        **{key: {"type": "number", "minimum": -1, "maximum": 1} for key in (*START, *TARGET)},
                        **{key: {"type": "number", "description": "Horizontal displacement in mm, relative to observation image"} for key in DELTA},
                        "rotation_deg": {"type": "number", "minimum": -180, "maximum": 180},
                        "confirm_motion": {"type": "boolean", "const": True}},
                    "x-action-params": {"observe": {"params": ["confirm_motion"]},
                        "grab_to": {"params": [*START, *TARGET, "rotation_deg", "confirm_motion"]},
                        "grab_by": {"params": [*START, *DELTA, "rotation_deg", "confirm_motion"]}},
                    "x-resource": ["base", "waist", "head", "arm_l", "arm_r", "hand_l", "hand_r"],
                    "x-is-dangerous": True, "x-completion": {"actions": list(MOTION)},
                    "x-hooks": {"on_interrupt_motion": {"action": "cancel"}, "on_interrupt_all": {"action": "stop"}}}}

    def start(self):
        pass  # Driver startup cannot move or acquire camera streams.

    def _start_input(self, args):
        rgb = self.device.topics["camera_color"]["topic"]
        depth = self.device.topics["camera_depth"]["topic"]
        inputs = args.get("input_topics", [])
        if not isinstance(inputs, list) or len(inputs) < 2 or len(inputs) > 3:
            raise ValueError("connect at least Panther depth and RGB; optionally VOP objects")
        valid_topics = {depth, rgb}
        if len(inputs) == 3:
            valid_topics.add(rgb + "/objects")
        if not all(isinstance(v, str) for v in inputs) or not set(inputs).issubset(valid_topics):
            raise ValueError("invalid input topics")
        has_vop = len(inputs) == 3
        if self._source and self._source[0].is_alive():
            return self.info()
        self._stop_requested = False
        self._snapshot = None
        self._close_input()
        inbox, closing, ready = self._ctx.Queue(maxsize=4), self._ctx.Event(), self._ctx.Event()
        if has_vop:
            vop_topic = rgb + "/objects"
            process = self._ctx.Process(target=_receive, args=(vop_topic,
                int(os.environ.get("AGENT_CORE_ROS_DOMAIN_ID", "42")),
                "/opt/phanthy-motus/dds-local.xml", inbox, ready, closing, "panther_pick_vop", 256 * 1024), daemon=True)
            process.start()
            self._source = process, inbox, closing, ready
            self._has_vop = True
        else:
            # No VOP: just arm the camera subscription (no ROS subscription needed)
            self._source = None
            self._has_vop = False
        return self.info()

    def _close_input(self):
        if self._source:
            process, inbox, closing, _ = self._source
            closing.set()
            process.join(1.)
            if process.is_alive():
                process.terminate()
                process.join(1.)
            inbox.close()
            self._source = None

    def stop(self):
        with self._lock:
            self._stop_requested = True
            self._cancel.set()
            self._snapshot = None
            worker = self._worker
        if worker:
            worker.join(timeout=12.)
        with self._lock:
            if self._worker is None:
                self._close_input()
            return self.info()

    def info(self):
        with self._lock:
            observation = self._snapshot
            if observation and time.monotonic() > observation["expiry"]:
                self._snapshot = observation = None
            # inputs_ready is True when depth+RGB are connected (VOP is optional)
            return {"state": "blocked" if self._blocked else "running" if self._worker else "idle",
                    "allow_motion": self.cfg.get("allow_motion") is True,
                    "inputs_ready": True,
                    "vop_connected": getattr(self, "_has_vop", False),
                    "config_error": self._config_error,
                    "observation_required": observation is None,
                    "observation": copy.deepcopy(observation["summary"]) if observation else None,
                    "last_result": copy.deepcopy(self._last)}

    def _checked_config(self):
        cfg = self.cfg
        if self._config_error:
            raise ValueError(self._config_error)
        if cfg.get("allow_motion") is not True:
            raise ValueError("vision_pick_and_drop is not commissioned for motion")
        if cfg.get("side") not in ("left", "right") or not cfg.get("tcp_frame") or not cfg.get("base_frame"):
            raise ValueError("configure the Panther arm, fixed base and calibrated TCP frames")
        def vec(key, count, low, high):
            values = cfg.get(key)
            if not isinstance(values, list) or len(values) != count:
                raise ValueError(f"configure {key} with {count} values")
            return [number(v, low, high) for v in values]
        vec("observation_joints_deg", 8, -180, 180)
        low, high = vec("workspace_min_m", 3, -3, 3), vec("workspace_max_m", 3, -3, 3)
        if any(a >= b for a, b in zip(low, high)):
            raise ValueError("invalid workspace bounds")
        vec("xy_compensation_m", 2, -.1, .1)
        for key in ("pick_descent_m", "place_descent_m"):
            number(cfg.get(key), .001, .25)
        for key in ("gripper_open_feedback", "gripper_feedback_tolerance"):
            number(cfg.get(key), .00001 if key.endswith("tolerance") else -100, 100)
        if not os.access(cfg["planner_python"], os.X_OK):
            raise ValueError("Panther Pink planner Python environment is missing")

    def dispatch(self, action, args):
        if action == "stop":
            return self.stop()
        with self._lock:
            if action == "info":
                return self.info()
            if action == "cancel":
                self._cancel.set()
                self._snapshot = None
                return self.info()
            if action == "start":
                if self._worker:
                    raise RuntimeError("vision_pick_and_drop is busy")
                return self._start_input(args)
            if action not in MOTION:
                raise ValueError(f"unsupported vision_pick_and_drop action: {action}")
            if args.get("confirm_motion") is not True:
                return {"state": "error", "code": "CONFIRMATION_REQUIRED"}
            if self._worker or self._blocked:
                raise RuntimeError("vision_pick_and_drop is busy or requires recovery")
            self._checked_config()
            if not self.info()["inputs_ready"]:
                raise RuntimeError("start the camera/VOP inputs before observing or transferring")
            if action != "observe" and not self._snapshot:
                return {"state": "error", "code": "OBSERVATION_REQUIRED", "observation_required": True}
            target = resolve_targets(action, args, self._snapshot, self.cfg) if action != "observe" else None
            self.device._ensure_motion_ready()
            self.device.reserve_pick(self)
            snapshot, self._snapshot = self._snapshot, None
            self._cancel.clear()
            action_id = str(args.get("action_id") or f"panther_pick_{uuid4().hex}")
            self._worker = threading.Thread(target=self._run, args=(action, action_id, snapshot, target), daemon=True)
            try:
                self._worker.start()
            except Exception:
                self._worker = None
                self.device.release_pick(self)
                raise
            return {"state": "running", "action_id": action_id}

    def _check_cancel(self):
        if self._cancel.is_set():
            raise RuntimeError("action cancelled")

    def _joints(self):
        msg, stamp = self.device._get("joints")
        if not fresh(stamp, .5) or msg is None:
            raise RuntimeError("fresh Panther joint feedback is required")
        return dict(zip(msg.name, msg.position))

    def _same_joints(self, reference):
        actual = self._joints()
        names = set(reference) - {n for n in reference if "wheel" in n.lower()}
        if any(n not in actual or not math.isfinite(actual[n] - reference[n]) or abs(actual[n] - reference[n]) > .008
               for n in names):
            raise RuntimeError("robot configuration changed after observation or planning")

    def _base_pose(self):
        msg, stamp = self.device._get("odom")
        if msg is None or not fresh(stamp, .5):
            raise RuntimeError("fresh base odometry is required")
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        raw = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in raw) or abs(sum(v*v for v in raw[2:]) - 1.) > .02:
            raise RuntimeError("invalid base odometry")
        return np.array(raw)

    def _same_base(self, reference):
        actual = self._base_pose()
        if (np.linalg.norm(actual[:2] - reference[:2]) > .005
                or abs(float(actual[2:] @ reference[2:])) < math.cos(.01 / 2)):
            raise RuntimeError("Panther base moved during tabletop operation")

    def _plan(self, action, measured, target):
        names = LEFT_ARM_JOINTS if self.cfg["side"] == "left" else RIGHT_ARM_JOINTS
        if not self.device._urdf:
            raise RuntimeError("live Panther URDF is required")
        limits = intersect_joint_limits(names, self.device._model_joint_limits, self.device._joint_limits)
        request = {"action": action, "measured": measured, "names": names, "urdf": self.device._urdf,
                   "limits": limits, "config": self.cfg,
                   "max_delta_deg": self.device._cfg["plugins"]["arm_control"]["max_target_delta_deg"]}
        if target:
            request.update(targets=target[0], rotation_deg=target[1])
        result = subprocess.run([self.cfg["planner_python"], str(Path(__file__).with_name("pick_ik.py"))],
                                input=json.dumps(request, allow_nan=False), text=True, capture_output=True,
                                timeout=25., check=True)
        reply = json.loads(result.stdout)
        if "error" in reply:
            raise RuntimeError(reply["error"])
        self._same_joints(measured)
        return names, reply["stages"]

    def _move(self, names, values, base):
        self._check_cancel()
        self._same_base(base)
        done = threading.Event()
        result = []

        def completion(status, details):
            result.append((status, details))
            done.set()

        self.device.arm({"action": "move", "side": self.cfg["side"], "positions_deg": values},
                        owner=self, completion=completion)
        try:
            while not done.wait(.05):
                self._check_cancel()
                self._same_base(base)
        except Exception:
            try:
                stopped = self.device.arm({"action": "stop"}, owner=self)
                if stopped["state"] == "stopping":
                    self._blocked = True
            except Exception:
                self._blocked = True
            raise
        self._check_cancel()
        status, details = result[0]
        if status != "completed" or not details.get("controllers_restored"):
            self._blocked |= not details.get("controllers_restored", False)
            raise RuntimeError(details.get("error") or details.get("restore_error") or status)
        limit = time.monotonic() + 1.
        while time.monotonic() < limit:
            self._check_cancel()
            self._same_base(base)
            actual = self._joints()
            if all(abs(actual.get(n, math.nan) - math.radians(v)) <= .015 for n, v in zip(names, values)):
                return
            self._cancel.wait(.05)
        raise RuntimeError("Panther arm did not reach the planned waypoint")

    def _grip(self, action):
        self._check_cancel()
        since = time.monotonic()
        side = self.cfg["side"]
        self.device.gripper({"action": action, "side": side}, owner=self)
        stable = anchor = None
        tolerance = self.cfg["gripper_feedback_tolerance"]
        while time.monotonic() - since < 5.:
            self._check_cancel()
            msg, stamp = self.device._get(f"gripper_{side}")
            value = msg.position[0] if msg and msg.position else math.nan
            valid = fresh(stamp) and stamp > since and math.isfinite(value)
            if action == "open":
                valid = valid and abs(value - self.cfg["gripper_open_feedback"]) <= tolerance
            if valid and (anchor is None or abs(value - anchor) > tolerance):
                anchor, stable = value, time.monotonic()
            if not valid:
                anchor = stable = None
            if stable is not None and time.monotonic() - stable >= .4 and time.monotonic() - since >= 1.:
                return
            self._cancel.wait(.05)
        raise RuntimeError(f"fresh stable gripper feedback unavailable after {action}")

    def _capture(self, base):
        reference = self._joints()
        after = time.time() + .6
        limit = time.monotonic() + 10.
        if self._has_vop:
            inbox = self._source[1]
        else:
            inbox = None
        while time.monotonic() < limit:
            self._check_cancel()
            self._same_joints(reference)
            self._same_base(base)
            photo = self.device._camera.pick_snapshot(after, time.time(), self.cfg["base_frame"])
            if photo is not None:
                self._same_joints(reference)
                self._check_cancel()
                height, width = photo["depth"].shape
                objects = []
                if inbox is not None:
                    try:
                        payload, _ = inbox.get(timeout=.1)
                        now = time.time()
                        stamp = number(payload["timestamp"], now - 2., now + .05)
                        started = stamp - number(payload["latency_ms"], 0, 2000) / 1000.
                        if started >= after:
                            objects = object_list(payload)
                    except (queue.Empty, KeyError, TypeError, ValueError):
                        pass
                summary = {"observation_id": uuid4().hex, "captured_at": photo["stamp"], "width": width,
                           "height": height, "objects": objects, "count": len(objects),
                           "synchronization": "stationary_window_approximate"}
                with self._lock:
                    self._check_cancel()
                    self._snapshot = {**photo, "summary": summary, "joints": reference,
                                      "base": base, "expiry": time.monotonic() + 30.}
                return summary
        raise RuntimeError("no fresh registered RGB-D observation; check camera and optical TF")

    def _run(self, action, action_id, snapshot, target):
        status = "error"
        result = {"ok": False, "stage": "planning", "holding_object_possible": False,
                  "release_completed": False, "transfer_completed": False,
                  "return_completed": False, "grasp_checked": False}
        try:
            base = self._base_pose()
            if action != "observe":
                self._same_joints(snapshot["joints"])
                self._same_base(snapshot["base"])
                result.update(observation_id=snapshot["summary"]["observation_id"], **target[2])
            names, stages = self._plan(action, self._joints(), target)
            if snapshot and action != "observe" and time.monotonic() > snapshot["expiry"]:
                raise RuntimeError("observation expired during planning")
            for label, command in stages:
                result["stage"] = label
                self._check_cancel()
                self._same_base(base)
                if command is not None:
                    if label == "return_observation":
                        result["transfer_completed"] = True
                    for waypoint in command:
                        self._move(names, waypoint, base)
                    if label == "return_observation":
                        result["return_completed"] = True
                else:
                    if label == "close":
                        result["holding_object_possible"] = True
                    self._grip("close" if label == "close" else "open")
                    if label == "release":
                        result.update(holding_object_possible=False, release_completed=True)
            if action == "observe" or self.cfg.get("observe_after_transfer") is True:
                result["stage"] = "observe"
                observation = self._capture(base)
                if action == "observe":
                    result.update(observation)
                else:
                    result["observation"] = observation
            self._check_cancel()
            result["ok"], status = True, "completed"
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            status = "cancelled" if self._cancel.is_set() else "error"
        finally:
            with self._lock:
                if status != "completed":
                    self._snapshot = None
                self._blocked = self._blocked or result["holding_object_possible"]
                if not self._blocked:
                    self.device.release_pick(self)
                result.update(recovery_required=self._blocked, observation_required=self._snapshot is None)
                self._last, self._worker = result, None
                if self._stop_requested:
                    self._close_input()
            _notify_completion(action_id, status, result, NAME)
