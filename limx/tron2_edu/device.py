"""MCP adapter for the documented TRON 2 EDU upper-level interfaces.

Arm telemetry is supported. Arm motion is intentionally not advertised: guide
§3.5.5 says the emergency-stop request cannot interrupt a moving arm.
"""
import math
import threading
import time

from common.vendor_runtime import action_schema, tool
from client import TronClient


PROFILES = {"fixed_arms", "mobile_arms", "biped", "wheeled_biped"}
LOWER = [-3.1416, -.2618, -3.6652, -2.618, -1.7453, -.7854, -1.5708,
         -3.1416, -3.194, -1.4835, -2.618, -1.3963, -.7854, -1.5708]
UPPER = [2.6005, 3.194, 1.4835, .2618, 1.3963, .7854, 1.5708,
         2.6005, .2618, 3.6652, .2618, 1.7453, .7854, 1.5708]


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} outside [{low}, {high}]")
    return value


class TronPlugin:
    def __init__(self, config, client=None):
        self.cfg = config
        self.profile = config.get("profile", "fixed_arms")
        if self.profile not in PROFILES:
            raise ValueError("unknown TRON 2 configuration profile")
        self.client = client or TronClient(config.get("endpoint", ""), config.get("accid", ""))
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread = None
        self._target = None
        self._expires = 0.0
        self._fault = None
        self._motion_enabled = config.get("motion_enabled", False) is True
        self._limit = number(config.get("speed_limit", .2), "speed_limit", .01, 1.0)

    @property
    def arms(self):
        return self.profile in ("fixed_arms", "mobile_arms")

    def start(self):
        if self.cfg.get("enabled", False) is True:
            self.client.connect()

    def stop(self):
        error = None
        with self._lock:
            self._shutdown.set()
            if self._target is not None:
                try:
                    self._zero()
                except Exception as exc:
                    error = exc
                finally:
                    self._target = None
        if self._thread:
            self._thread.join(timeout=3)
        self.client.close()
        if error:
            raise RuntimeError("disconnected; zero velocity delivery unconfirmed") from error

    def get_tools(self):
        actions = {"start": ([], "Connect without moving"),
                   "stop": ([], "Stop owned velocity stream and disconnect"),
                   "info": ([], "Connection and capability status"),
                   "get": ([], "Read fresh robot information")}
        definitions = [tool("tron2_connection", "sensor", "TRON 2 EDU connection", action_schema(actions, {}))]
        info_actions = dict(actions)
        info_actions["stop"] = ([], "Stop reading")
        definitions.append(tool("tron2_robot_info", "sensor", "Robot mode, diagnostics and battery",
                                action_schema(info_actions, {})))
        if self.arms:
            for suffix, description in (("joint_states", "Joint names, radians, rad/s and torques"),
                                        ("eef_pose", "Left/right XYZ metres and WXYZ quaternion")):
                definitions.append(tool(f"tron2_{suffix}", "sensor", description,
                                        action_schema({"get": ([], description), "start": ([], description),
                                                       "stop": ([], "Stop reading"), "info": ([], "Status")}, {})))
            schema = action_schema({"validate": (["joint"], "Validate a 14-joint target; never sends motion"),
                                    "info": ([], "Joint order and motion limitation"),
                                    "start": ([], "Status"), "stop": ([], "No motion is owned")},
                                   {"joint": {"type": "array", "items": {"type": "number"},
                                              "minItems": 14, "maxItems": 14}})
            definitions.append(tool("tron2_arm_target", "sensor", "Arm target validation only", schema))
        else:
            units = "dimensionless ratios" if self.profile == "biped" else "x: m/s; y: zero; z: vendor yaw command"
            schema = action_schema({"set_velocity": (["x", "y", "z", "lease"], units),
                                    "stop": ([], "Send zero velocity; not a physical emergency stop"),
                                    "start": ([], "Report status without moving"),
                                    "info": ([], "Stream status")},
                                   {key: {"type": "number"} for key in ("x", "y", "z", "lease")})
            schema["x-resource"] = "locomotion"
            schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"},
                                  "on_interrupt_all": {"action": "stop"}}
            definitions.append(tool("tron2_velocity", "actuator", "Leased velocity; requires WALK mode", schema))
        return definitions

    def info(self):
        with self._lock:
            return {"connected": self.client.connected, "profile": self.profile,
                    "motion_enabled": self._motion_enabled and not self.arms,
                    "velocity_stream": self._target is not None,
                    "fault": self._fault,
                    "arm_motion": "unavailable: SDK does not document a moving-arm interrupt",
                    "joint_order": "left shoulder to wrist (7), right shoulder to wrist (7)"}

    def _ready(self):
        data = self.client.notification("notify_robot_info")
        if data.get("motor") != "OK" or data.get("imu") != "OK":
            raise RuntimeError("robot motor/IMU diagnostics are not OK")
        return data

    def dispatch(self, action, args):
        name = args.get("_tool_name", "tron2_connection")
        if name not in {definition["name"] for definition in self.get_tools()}:
            raise ValueError("tool unavailable for this configuration")
        if action == "info":
            return self.info()
        if name == "tron2_connection":
            if action == "start":
                with self._lock:
                    if self._fault and self.client.connected:
                        raise RuntimeError("disconnect before reconnecting a faulted stream")
                    self._shutdown.clear()
                    self.client.connect()
                    self._fault = None
                return self.info()
            if action == "stop":
                self.stop()
                return self.info()
            if action == "get":
                return self.client.notification("notify_robot_info")
        elif name == "tron2_robot_info" and action in ("get", "start"):
            return self.client.notification("notify_robot_info")
        elif name == "tron2_robot_info" and action == "stop":
            return {"state": "idle"}
        elif name == "tron2_velocity":
            if action == "set_velocity":
                return self.set_velocity(args)
            if action == "stop":
                with self._lock:
                    self._target = None
                    self._expires = 0
                    self._zero()
                    return {"state": "zero_velocity_sent", "physical_stop_confirmed": False}
            if action == "start":
                return self.info()
        elif name == "tron2_arm_target":
            if action == "validate":
                joints = args.get("joint")
                if not isinstance(joints, list) or len(joints) != 14:
                    raise ValueError("joint must contain 14 radians")
                values = [number(v, f"joint[{i}]", LOWER[i], UPPER[i]) for i, v in enumerate(joints)]
                return {"valid": True, "joint": values, "sent": False,
                        "collision_checked": False, "motion_available": False}
            if action in ("start", "stop"):
                return self.info()
        elif name in ("tron2_joint_states", "tron2_eef_pose"):
            if action == "stop":
                return {"state": "idle"}
            if action in ("get", "start"):
                title = "request_get_joint_state" if name.endswith("joint_states") else "request_get_move_pose"
                return self.client.request(title)
        raise ValueError("unsupported action")

    def set_velocity(self, args):
        if self.arms or not self._motion_enabled:
            raise RuntimeError("velocity motion is disabled")
        lease = number(args.get("lease", .25), "lease", .05, .5)
        target = {key: number(args.get(key, 0), key, -self._limit, self._limit) for key in ("x", "y", "z")}
        if self.profile == "wheeled_biped" and target["y"] != 0:
            raise ValueError("wheeled biped does not support lateral velocity")
        # Identity is checked by the transport; different profiles have different units.
        expected = "SF_TRON2A_" if self.profile == "biped" else "WF_TRON2A_"
        if not self.client.accid.startswith(expected):
            raise RuntimeError("ACCID does not match the configured locomotion profile")
        with self._lock:
            if self._shutdown.is_set() or self._fault:
                raise RuntimeError("velocity stream is stopped/faulted; reconnect explicitly")
            if self._ready().get("status") != "WALK":
                raise RuntimeError("use the vendor controls to enter WALK before sending velocity")
            self._target, self._expires = target, time.monotonic() + lease
            self._tick()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._stream, daemon=True)
                self._thread.start()
            return {"state": "streaming", "lease": lease}

    def _zero(self):
        self.client.send("request_twist", {"x": 0.0, "y": 0.0, "z": 0.0})

    def _tick(self):
        # Caller owns _lock, so stop cannot race a subsequent nonzero send.
        if self._target is None:
            return
        try:
            if self._ready().get("status") != "WALK":
                raise RuntimeError("robot left WALK mode")
            try:
                fault = self.client.notification("notify_twist")
            except RuntimeError:
                fault = None
            if fault is not None and fault.get("result") != "success":
                raise RuntimeError("controller rejected velocity")
            if time.monotonic() >= self._expires:
                self._target = {"x": 0.0, "y": 0.0, "z": 0.0}
            self.client.send("request_twist", self._target)
        except Exception:
            self._fault = "velocity feedback/transport failure; physical stop unconfirmed"
            self._target = None
            try:
                self._zero()
            except Exception:
                pass
            raise

    def _stream(self):
        while not self._shutdown.wait(.02):
            with self._lock:
                try:
                    self._tick()
                except Exception:
                    self._thread = None
                    return


def build_plugins(config, namespace, ros2):
    return [TronPlugin(config["tron2"])]
