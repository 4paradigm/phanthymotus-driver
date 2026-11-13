"""MCP card declarations for Panther."""

from __future__ import annotations

from common.camera_info import build as build_camera_info

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
        data = self.device.sensor_info(self.name)
        return {"state": "running", "data": data,
                "topic_out": [self.device.topics[card] for card in self.topic_cards]}


class JointsPlugin(SensorPlugin):
    def get_tools(self):
        return [super().get_tool(), {
            "name": "model", "type": "resource", "multiInstance": False,
            "description": "Panther live URDF from /robot_description, with bundled fallback",
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
        schema["x-hooks"] = {"on_interrupt_motion": "stop", "on_interrupt_all": "stop"}
    return schema


def build_plugins(config, namespace, device):
    del namespace
    enabled = config.get("plugins", {})
    plugins = []

    def on(name):
        return bool(enabled.get(name, {}).get("enabled", False))

    if on("joints"):
        plugins.append(JointsPlugin(device, "joints", "Panther live URDF skeleton and joint state", ["joints"]))
    for name, description in (
        ("robot_state", "Panther low-rate health, battery, controllers, grippers and lifting state"),
        ("imu", "Panther IMU state"),
        ("odom", "Panther wheel odometry"),
    ):
        if on(name):
            plugins.append(SensorPlugin(device, name, description, [name]))
    if on("laser_scan"):
        plugins.append(SensorPlugin(
            device, "laser_scan", "Panther merged dual-lidar scan from /scan",
            ["laser_scan", "laser_view"],
        ))
    if on("camera_color"):
        plugins.append(CameraPlugin(
            device, "camera_color", "rgb", "Panther Gemini 2 RGB stream (lazy host-service start)"
        ))
    if on("camera_depth"):
        plugins.append(CameraPlugin(
            device, "camera_depth", "depth", "Panther Gemini 2 depth stream (zlib 16UC1)"
        ))

    confirm = {"type": "boolean", "const": True, "description": "Explicitly confirm physical motion"}
    if on("base_drive"):
        cfg = enabled["base_drive"]
        schema = _action_schema(
            ["start", "info", "move", "stop", "cancel"],
            {
                "linear_x": {"type": "number", "minimum": -cfg["max_linear_mps"], "maximum": cfg["max_linear_mps"]},
                "linear_y": {"type": "number", "minimum": -cfg["max_linear_mps"], "maximum": cfg["max_linear_mps"]},
                "angular_z_deg_s": {"type": "number", "minimum": -cfg["max_angular_deg_s"], "maximum": cfg["max_angular_deg_s"],
                                    "description": "Horizontal turn rate in degrees per second"},
                "duration_s": {"type": "number", "minimum": 0.1, "maximum": cfg["max_duration_s"]},
                "confirm_motion": confirm,
            },
            {"move": {"params": ["linear_x", "linear_y", "angular_z_deg_s", "duration_s", "confirm_motion"]},
             "stop": {"params": []}, "cancel": {"params": []},
             "start": {"params": []}, "info": {"params": []}},
            "base",
            {"actions": ["move"], "timeout": cfg["max_duration_s"] + 10},
            True,
        )
        plugins.append(ActuatorPlugin(device, "base_drive", "Finite-duration omnidirectional drive through /cmd_vel_teleop", schema, device.drive))
    if on("arm_control"):
        joint_fields = {
            **{f"la{i}_deg": {"type": "number", "description": f"Left arm joint la{i} target in degrees; omitted joints hold their current positions"}
               for i in range(8)},
            **{f"ra{i}_deg": {"type": "number", "description": f"Right arm joint ra{i} target in degrees; omitted joints hold their current positions"}
               for i in range(8)},
        }
        schema = _action_schema(
            ["start", "info", "move", "home", "stop"],
            {
                "side": {"type": "string", "enum": ["left", "right", "both"]},
                **joint_fields,
                "positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8},
                "left_positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8},
                "right_positions_deg": {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8},
                "duration_s": {"type": "number", "minimum": 1.0, "maximum": enabled["arm_control"]["max_duration_s"]},
                "confirm_motion": confirm,
            },
            {"move": {"params": ["side", *joint_fields, "duration_s", "confirm_motion"]},
             "home": {"params": ["side", "confirm_motion"]},
             "stop": {"params": []}, "start": {"params": []}, "info": {"params": []}},
            ["arm_l", "arm_r"],
            {"actions": ["move", "home"], "timeout": enabled["arm_control"]["max_duration_s"] + 15},
            True,
        )
        plugins.append(ActuatorPlugin(device, "arm_control", "Safe P2P arm control with verified controller switching", schema, device.arm))
    if on("gripper"):
        schema = _action_schema(
            ["start", "info", "set", "open", "close"],
            {"side": {"type": "string", "enum": ["left", "right", "both"], "default": "both"},
             "position": {"type": "number", "minimum": 0.0, "maximum": 1.0},
             "confirm_motion": confirm},
            {"set": {"params": ["side", "position", "confirm_motion"]},
             "open": {"params": ["side", "confirm_motion"]},
             "close": {"params": ["side", "confirm_motion"]},
             "start": {"params": []}, "info": {"params": []}}, ["hand_l", "hand_r"])
        plugins.append(ActuatorPlugin(device, "gripper", "Left/right Panther gripper position control", schema, device.gripper))
    if on("lifting"):
        schema = _action_schema(
            ["start", "info", "set_position"],
            {"position_m": {"type": "number", "minimum": 0.0, "maximum": enabled["lifting"]["max_position_m"]},
             "confirm_motion": confirm},
            {"set_position": {"params": ["position_m", "confirm_motion"]},
             "start": {"params": []}, "info": {"params": []}}, "waist")
        plugins.append(ActuatorPlugin(device, "lifting", "Panther lifting position control", schema, device.lifting))
    if on("head"):
        schema = _action_schema(
            ["start", "info", "look", "home", "stop"],
            {"yaw_deg": {"type": "number", "description": "Horizontal head angle in degrees"},
             "pitch_deg": {"type": "number", "description": "Head pitch angle in degrees"},
             "confirm_motion": confirm},
            {"look": {"params": ["yaw_deg", "pitch_deg", "confirm_motion"]},
             "home": {"params": ["confirm_motion"]}, "stop": {"params": []},
             "start": {"params": []}, "info": {"params": []}}, "head",
            {"actions": ["home"], "timeout": 30}, True)
        plugins.append(ActuatorPlugin(device, "head", "Panther head pan/tilt control using live URDF limits", schema, device.head))
    return plugins

