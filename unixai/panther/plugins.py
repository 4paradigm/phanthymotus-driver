"""MCP card declarations for Panther."""

from __future__ import annotations

from common.camera_info import build as build_camera_info
from contracts import ARM_JOINT_FIELDS
from panther_media import ENCODE_TIMEOUT_S, MAX_DURATION_S


ARM_JOINT_TITLES = (
    "Shoulder Rotation",
    "Shoulder Abduction",
    "Shoulder Raise",
    "Elbow Rotation",
    "Elbow Flexion/Extension",
    "Wrist 1",
    "Wrist 2",
    "Wrist 3",
)

def _sensor_schema():
    return {
        "type": "object",
        "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"]}},
        "required": ["action"],
        "additionalProperties": False,
    }


class SensorPlugin:
    def __init__(self, device, name: str, description: str, topic_cards: list[str]):
        self.device = device
        self.name = name
        self.description = description
        self.topic_cards = topic_cards

    def get_tool(self):
        return {
            "name": self.name,
            "type": "sensor",
            "multiInstance": False,
            "description": self.description,
            "inputSchema": _sensor_schema(),
            "topic_out": [self.device.topics[card] for card in self.topic_cards],
        }

    def start(self):
        return None

    def stop(self):
        return None

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action == "start" and self.name == "robot_state":
            # robot_state includes controller readiness. Query it now instead
            # of reporting failure merely because the background poll has not
            # completed its first cycle yet.
            self.device.controller_states()
        data = self.device.sensor_info(self.name)
        if action == "start":
            state = "running" if data.get("available") and data.get("fresh") else "waiting_for_data"
        else:
            state = "running" if data.get("fresh") else "unavailable"
        return {"state": state, "armed": action == "start", "data": data,
                "topic_out": [self.device.topics[card] for card in self.topic_cards]}


class JointsPlugin(SensorPlugin):
    def get_tools(self):
        return [super().get_tool(), {
            "name": "model", "type": "resource", "multiInstance": False,
            "description": "Panther articulated display URDF with matching joint stream",
            "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
        }]

    def dispatch(self, action, args):
        if action == "model" or args.get("_tool_name") == "model":
            return self.device.sensor_info("model")
        return super().dispatch(action, args)


class ActuatorPlugin:
    def __init__(self, device, name, description, schema, method):
        self.device = device
        self.name = name
        self.description = description
        self.schema = schema
        self.method = method

    def get_tool(self):
        return {"name": self.name, "type": "actuator", "multiInstance": False,
                "description": self.description, "inputSchema": self.schema}

    def start(self):
        return None

    def stop(self):
        if self.name == "base_drive":
            self.device.cancel_drive()
        elif self.name == "arm_control":
            self.device.arm({"action": "stop"})
        elif self.name == "head":
            self.device.head({"action": "stop"})
        elif self.name == "vision_capture":
            self.device.vision_capture({"action": "stop"})

    def dispatch(self, action, args):
        payload = dict(args); payload["action"] = action
        return self.method(payload)


class CameraPlugin:
    """Canvas-lazy camera card backed by the shared Panther camera runtime."""

    def __init__(self, device, name: str, kind: str, description: str):
        self.device = device
        self.name = name
        self.kind = kind
        self.description = description

    def get_tool(self):
        topic = self.device.topics[self.name]
        depth = self.kind == "depth"
        return {
            "name": self.name,
            "type": "sensor",
            "multiInstance": False,
            "description": self.description,
            "inputSchema": _sensor_schema(),
            "topic_out": [topic],
            "camera_info": [build_camera_info(
                topic=topic["topic"],
                id=f"unixai/panther/{self.name}",
                format=topic["format"],
                width=640 if depth else None,
                height=480 if depth else None,
                source="unknown",
                pipeline=[f"unixai/panther/{self.kind}"]
                         + (["unixai/panther/depth-crop-resize"] if depth else []),
                vendor={"camera": "Orbbec Gemini 2 RGB-D", "transport": "ros2"},
            )],
        }

    def start(self):
        # DriverBundle.start_all() runs at container startup. Camera hardware is
        # deliberately armed only by dispatch(action=start) from the canvas.
        return None

    def stop(self):
        self.device.camera(self.kind, {"action": "stop"})

    def dispatch(self, action, args):
        del args
        result = self.device.camera(self.kind, {"action": action})
        result["topic_out"] = [self.device.topics[self.name]]
        return result


def _action_schema(actions, properties, required_by_action=None, resource=None,
                   completion=None, interruptible=False):
    schema = {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": list(actions)},
            **properties,
        },
        "required": ["action"],
        "additionalProperties": False,
    }
    if required_by_action:
        schema["x-action-params"] = required_by_action
    if resource:
        schema["x-resource"] = resource
    if completion:
        schema["x-completion"] = completion
    if interruptible:
        schema["x-hooks"] = {
            "on_interrupt_motion": {"action": "stop"},
            "on_interrupt_all": {"action": "stop"},
        }
    return schema


def build_plugins(config, namespace, device):
    enabled = config.get("plugins", {})
    plugins = []

    def on(name):
        return bool(enabled.get(name, {}).get("enabled", False))

    if on("joints"):
        plugins.append(JointsPlugin(device, "joints", "Panther skeleton with live joint feedback", ["joints"]))
    for name, description in (
        ("robot_state", "Panther motion readiness, controller, diagnostic and hardware summary"),
        ("battery", "Panther battery percentage, voltage, current, temperature and power status"),
        ("imu", "Panther IMU state"),
        ("odom", "Panther wheel odometry"),
    ):
        if on(name):
            plugins.append(SensorPlugin(device, name, description, [name]))
    if on("laser_scan"):
        plugins.append(SensorPlugin(
            device, "laser_scan", "Panther live planar lidar point cloud",
            ["laser_cloud", "laser_scan"],
        ))
    if on("camera_color"):
        plugins.append(CameraPlugin(
            device, "camera_color", "rgb", "Panther Gemini 2 RGB stream (lazy host-service start)"
        ))
    if on("camera_depth"):
        plugins.append(CameraPlugin(
            device, "camera_depth", "depth", "Panther Gemini 2 depth stream (zlib 16UC1)"
        ))
    if on("vision_capture"):
        schema = _action_schema(
            ["capture_photo", "record_video", "list", "delete", "start", "info", "stop"],
            {"duration_s": {"type": "integer", "minimum": 1, "maximum": MAX_DURATION_S, "default": 5,
                            "description": "Video duration in seconds"},
             "name": {"type": "string", "description": "Complete filename returned by list"}},
            {"capture_photo": {"params": [], "description": "Save the next RGB frame as a JPEG"},
             "record_video": {"params": ["duration_s"], "description": "Record a silent RGB MP4"},
             "list": {"params": [], "description": "List saved photos and videos"},
             "delete": {"params": ["name"], "description": "Delete one saved photo or video"},
             "start": {"params": [], "description": "Check for a running RGB source"},
             "info": {"params": [], "description": "Read the camera status and media directories"},
             "stop": {"params": [], "description": "Cancel a recording and discard the unfinished video"}},
            completion={"actions": ["record_video"], "timeout": MAX_DURATION_S + ENCODE_TIMEOUT_S + 15},
        )
        plugins.append(ActuatorPlugin(
            device, "vision_capture", "Capture, list and delete Panther RGB photos and videos",
            schema, device.vision_capture,
        ))
    if on("mic") or on("speaker"):
        from audio import AudioPlugin
        for name in ("mic", "speaker"):
            if on(name):
                plugins.append(AudioPlugin(name, namespace, enabled["audio"]))

    if on("base_drive"):
        cfg = enabled["base_drive"]
        schema = _action_schema(
            ["start", "info", "move", "stop", "cancel"],
            {
                "linear_x": {"type": "number", "minimum": -cfg["max_linear_mps"], "maximum": cfg["max_linear_mps"],
                             "description": f"[-{cfg['max_linear_mps']:g}, {cfg['max_linear_mps']:g}] m/s; +forward, -backward"},
                "linear_y": {"type": "number", "minimum": -cfg["max_linear_mps"], "maximum": cfg["max_linear_mps"],
                             "description": f"[-{cfg['max_linear_mps']:g}, {cfg['max_linear_mps']:g}] m/s; +left, -right"},
                "angular_z_deg_s": {"type": "number", "minimum": -cfg["max_angular_deg_s"], "maximum": cfg["max_angular_deg_s"],
                                    "description": f"[-{cfg['max_angular_deg_s']:g}, {cfg['max_angular_deg_s']:g}] deg/s; +turn left, -turn right"},
                "duration_s": {"type": "number", "minimum": 0.1, "maximum": cfg["max_duration_s"],
                               "description": f"[0.1, {cfg['max_duration_s']:g}] s; auto-stops when elapsed"},
            },
            {"move": {"params": ["linear_x", "linear_y", "angular_z_deg_s", "duration_s"],
                      "description": "Move with bounded velocity for a finite duration"},
             "stop": {"params": [], "description": "Stop immediately"},
             "cancel": {"params": [], "description": "Cancel the active move and stop"},
             "start": {"params": [], "description": "Check controller readiness"},
             "info": {"params": [], "description": "Read controller readiness"}},
            "base",
            {"actions": ["move"], "timeout": cfg["max_duration_s"] + 10},
            True,
        )
        plugins.append(ActuatorPlugin(device, "base_drive", "Finite-duration omnidirectional Panther base drive", schema, device.drive))
    if on("arm_control"):
        joint_limits = device.schema_joint_limits_deg() or {}

        def format_range(bounds: tuple[float | None, float | None]) -> str:
            lower, upper = bounds
            if lower is None or upper is None:
                return "unavailable"
            return f"[{lower:.1f}, {upper:.1f}] deg"

        def joint_field(name: str, title: str) -> dict:
            lower, upper = joint_limits.get(name, (None, None))
            prefix = (f"Range {format_range((lower, upper))}; "
                      if lower is not None and upper is not None
                      else "Range unavailable; ")
            max_delta = enabled["arm_control"].get("max_target_delta_deg", 20.0)
            field = {"type": "number", "title": f"{title} (deg)",
                     "description": prefix + f"max step {max_delta:g} deg"}
            if lower is not None and upper is not None:
                field.update(minimum=lower, maximum=upper)
            return field

        separate_joint_fields = {
            **{f"l_{field}": joint_field(f"joint_la{i}", f"Left {ARM_JOINT_TITLES[i]}")
               for i, field in enumerate(ARM_JOINT_FIELDS)},
            **{f"r_{field}": joint_field(f"joint_ra{i}", f"Right {ARM_JOINT_TITLES[i]}")
               for i, field in enumerate(ARM_JOINT_FIELDS)},
        }

        def shared_joint_field(index: int) -> dict:
            left = joint_limits.get(f"joint_la{index}", (None, None))
            right = joint_limits.get(f"joint_ra{index}", (None, None))
            bounds = [value for value in (*left, *right) if value is not None]
            max_delta = enabled["arm_control"].get("max_target_delta_deg", 20.0)
            field = {
                "type": "number",
                "title": f"{ARM_JOINT_TITLES[index]} (deg)",
                "description": (
                    f"L/B {format_range(left)}; R {format_range(right)}; "
                    f"max step {max_delta:g} deg"
                ),
            }
            if len(bounds) == 4:
                field.update(minimum=min(bounds), maximum=max(bounds))
            return field

        shared_joint_fields = {
            field: shared_joint_field(index)
            for index, field in enumerate(ARM_JOINT_FIELDS)
        }
        legacy_joint_fields = {
            **{f"j{i}_deg": {**shared_joint_field(i), "x-ui-hidden": True}
               for i in range(8)},
            **{f"la{i}_deg": {**joint_field(f"joint_la{i}", f"Left {ARM_JOINT_TITLES[i]}"),
                                "x-ui-hidden": True}
               for i in range(8)},
            **{f"ra{i}_deg": {**joint_field(f"joint_ra{i}", f"Right {ARM_JOINT_TITLES[i]}"),
                                "x-ui-hidden": True}
               for i in range(8)},
        }
        schema = _action_schema(
            ["start", "info", "move", "move_separate", "home", "stop"],
            {
                "side": {"type": "string", "enum": ["left", "right", "both"], "default": "left",
                         "description": "left, right, or mirrored both arms"},
                **shared_joint_fields,
                **separate_joint_fields,
                **legacy_joint_fields,
                "positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8,
                                  "description": "8 joint targets in deg; selected arm; live URDF limits"},
                "left_positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8,
                                       "description": "8 left-arm targets in deg; live URDF limits"},
                "right_positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8,
                                        "description": "8 right-arm targets in deg; live URDF limits"},
                "duration_s": {"type": "number", "minimum": 1.0,
                               "maximum": enabled["arm_control"]["max_duration_s"],
                               "description": f"[1, {enabled['arm_control']['max_duration_s']:g}] s"},
            },
            {"move": {"params": ["side", *shared_joint_fields, "duration_s"],
                      "description": "Move one arm, or both arms as a mirrored 8-axis pose"},
             "move_separate": {"params": [*separate_joint_fields, "duration_s"],
                               "description": "Move both arms with 16 independent joint targets"},
             "home": {"params": ["side"], "description": "Move selected arms to the candidate home pose"},
             "stop": {"params": [], "description": "Cancel the active arm motion"},
             "start": {"params": [], "description": "Check controller and feedback readiness"},
             "info": {"params": [], "description": "Read controller and feedback readiness"}},
            ["arm_l", "arm_r"],
            {"actions": ["move", "move_separate", "home"], "timeout": enabled["arm_control"]["max_duration_s"] + 15},
            True,
        )
        plugins.append(ActuatorPlugin(device, "arm_control", "Safe P2P arm control with verified controller switching", schema, device.arm))
    if on("gripper"):
        schema = _action_schema(
            ["start", "info", "set", "open", "close", "stop"],
            {"side": {"type": "string", "enum": ["left", "right", "both"], "default": "both",
                      "description": "left, right, or both grippers"},
             "position": {"type": "number", "minimum": 0.0, "maximum": 1.0,
                          "description": "[0, 1]; 0=open, 1=closed"}},
            {"set": {"params": ["side", "position"], "description": "Set normalized gripper position"},
             "open": {"params": ["side"], "description": "Open selected grippers"},
             "close": {"params": ["side"], "description": "Close selected grippers"},
             "stop": {"params": [], "description": "End the card lifecycle"},
             "start": {"params": [], "description": "Check controller and feedback readiness"},
             "info": {"params": [], "description": "Read controller and feedback readiness"}}, ["hand_l", "hand_r"])
        plugins.append(ActuatorPlugin(device, "gripper", "Left/right Panther gripper position control", schema, device.gripper))
    if on("lifting"):
        schema = _action_schema(
            ["start", "info", "set_position", "home", "stop"],
            {"position_m": {"type": "number", "minimum": enabled["lifting"].get("min_position_m", 0.0), "maximum": enabled["lifting"]["max_position_m"],
                            "description": f"[{enabled['lifting'].get('min_position_m', 0.0):g}, {enabled['lifting']['max_position_m']:g}] m"}},
            {"set_position": {"params": ["position_m"], "description": "Set lifting height"},
             "home": {"params": [], "description": f"Move to {enabled['lifting'].get('home_position_m', 0.6):g} m home height"},
             "stop": {"params": [], "description": "End the card lifecycle"},
             "start": {"params": [], "description": "Check controller and feedback readiness"},
             "info": {"params": [], "description": "Read controller and feedback readiness"}}, "waist")
        plugins.append(ActuatorPlugin(device, "lifting", "Panther lifting position control", schema, device.lifting))
    if on("head"):
        schema = _action_schema(
            ["start", "info", "look", "home", "stop"],
            {"yaw_deg": {"type": "number", "minimum": enabled["head"].get("yaw_min_deg", -90.0),
                         "maximum": enabled["head"].get("yaw_max_deg", 90.0),
                         "description": f"[{enabled['head'].get('yaw_min_deg', -90.0):g}, {enabled['head'].get('yaw_max_deg', 90.0):g}] deg; +left, -right"},
             "pitch_deg": {"type": "number", "minimum": enabled["head"].get("pitch_min_deg", -86.0),
                           "maximum": enabled["head"].get("pitch_max_deg", 20.0),
                           "description": f"[{enabled['head'].get('pitch_min_deg', -86.0):g}, {enabled['head'].get('pitch_max_deg', 20.0):g}] deg; +down, -up"},
             "speed": {"type": "string", "enum": ["low", "high"], "default": "high",
                       "description": f"low={enabled['head'].get('low_speed_deg_s', 10.0):g} deg/s; high={enabled['head'].get('high_speed_deg_s', 20.0):g} deg/s"}},
            {"look": {"params": ["yaw_deg", "pitch_deg", "speed"], "description": "Move smoothly to a pan/tilt target"},
             "home": {"params": [], "description": "Return home at low speed"},
             "stop": {"params": [], "description": "Cancel the active head motion"},
             "start": {"params": [], "description": "Check controller and feedback readiness"},
             "info": {"params": [], "description": "Read controller and feedback readiness"}}, "head",
            {"actions": ["look", "home"], "timeout": 30}, True,
        )
        plugins.append(ActuatorPlugin(device, "head", "Panther head pan/tilt control using live URDF limits", schema, device.head))
    return plugins

