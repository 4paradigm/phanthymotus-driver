#!/usr/bin/env python3
"""U1 Pro capability layer with a configurable supplier ROS2 JSON bridge."""

from __future__ import annotations

from common.ros2_json_bridge import JsonCommandBridge
from common.vendor_runtime import action_schema, tool


def actuator(name, description, actions, properties):
    class Plugin:
        def __init__(self, bridge): self.bridge = bridge
        def get_tool(self): return tool(name, "actuator", description, action_schema(actions, properties))
        def start(self): pass
        def stop(self): pass
        def dispatch(self, action, args):
            if action == "start": return {"state": "ready"}
            if action == "stop" and "stop" not in actions: return {"state": "idle"}
            return self.bridge.publish(f"{name}.{action}", {key: value for key, value in args.items() if not key.startswith("_")})
    Plugin.__name__ = f"U1{name.title()}Plugin"
    return Plugin


U1SystemPlugin = actuator("system", "U1 Pro power, mode, recovery and emergency controls", {
    "enable": ([], "Enable robot actuators"), "disable": ([], "Disable robot actuators"),
    "home": ([], "Return to neutral pose"), "recover": ([], "Request recovery"),
    "emergency_stop": ([], "Request emergency stop"),
}, {})

U1LocomotionPlugin = actuator("locomotion", "U1 Pro walking and navigation command surface", {
    "velocity": (["linear_x", "linear_y", "angular_z", "duration"], "Command walking velocity"),
    "walk_to": (["x", "y", "yaw", "frame"], "Walk to a target pose"),
    "turn": (["angle"], "Turn in place"), "stop": ([], "Stop locomotion"),
}, {
    "linear_x": {"type": "number"}, "linear_y": {"type": "number"}, "angular_z": {"type": "number"}, "duration": {"type": "number"},
    "x": {"type": "number"}, "y": {"type": "number"}, "yaw": {"type": "number"}, "frame": {"type": "string"}, "angle": {"type": "number"},
})

U1BodyPlugin = actuator("body", "U1 Pro posture, joints and trajectory control", {
    "posture": (["name", "speed"], "Play a supplier posture"),
    "joint_pose": (["joints", "duration"], "Move named joints"),
    "trajectory": (["points", "repeat"], "Play a custom joint trajectory"),
    "cancel": ([], "Cancel active body motion"),
}, {
    "name": {"type": "string"}, "speed": {"type": "number"}, "joints": {"type": "object", "additionalProperties": {"type": "number"}},
    "duration": {"type": "number"}, "points": {"type": "array", "items": {"type": "object"}}, "repeat": {"type": "integer"},
})

U1HeadPlugin = actuator("head", "U1 Pro head, gaze and visual-attention controls", {
    "gaze": (["yaw", "pitch", "duration"], "Set head gaze angles"), "look_at": (["x", "y", "z", "frame"], "Look at a 3D target"),
    "nod": (["count"], "Nod the head"), "shake": (["count"], "Shake the head"), "track_face": (["enabled"], "Enable face tracking"),
}, {
    "yaw": {"type": "number"}, "pitch": {"type": "number"}, "duration": {"type": "number"}, "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
    "frame": {"type": "string"}, "count": {"type": "integer"}, "enabled": {"type": "boolean"},
})

U1HandsPlugin = actuator("hands", "U1 Pro hand and social gesture controls", {
    "open": (["side"], "Open one or both hands"), "close": (["side", "strength"], "Close one or both hands"),
    "gesture": (["name", "side"], "Play a hand/arm gesture"), "point": (["side", "x", "y", "z", "frame"], "Point to a target"),
}, {
    "side": {"type": "string", "enum": ["left", "right", "both"]}, "strength": {"type": "number"}, "name": {"type": "string"},
    "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "frame": {"type": "string"},
})

U1SpeechPlugin = actuator("speech", "U1 Pro speech synthesis, playback and listening controls", {
    "say": (["text", "voice", "volume", "interrupt"], "Speak text"), "stop": ([], "Stop speech/audio"),
    "listen": (["timeout", "language"], "Start speech recognition"), "play_audio": (["uri", "volume"], "Play an audio resource"),
}, {
    "text": {"type": "string"}, "voice": {"type": "string"}, "volume": {"type": "number"}, "interrupt": {"type": "boolean"},
    "timeout": {"type": "number"}, "language": {"type": "string"}, "uri": {"type": "string"},
})

U1InteractionPlugin = actuator("interaction", "U1 Pro multimodal interaction and presentation workflows", {
    "greet": (["name"], "Run a greeting interaction"), "presentation": (["text", "gesture"], "Coordinate speech and gesture"),
    "conversation": (["enabled"], "Enable or disable conversation mode"), "face_enrollment": (["name"], "Start supplier face enrollment"),
}, {
    "name": {"type": "string"}, "text": {"type": "string"}, "gesture": {"type": "string"}, "enabled": {"type": "boolean"},
})


class U1ChoreographyPlugin:
    PRESETS = ("wave", "bow", "greet", "dance", "dance_1", "dance_2")
    def __init__(self, bridge): self.bridge = bridge
    def get_tool(self):
        return tool("choreography", "actuator", "U1 Pro gestures, dances and synchronized speech/motion", action_schema(
            {"list": ([], "List driver choreography names"), "play": (["name", "repeat"], "Play named choreography"), "custom": (["timeline", "repeat"], "Submit a custom multimodal timeline"), "stop": ([], "Stop choreography")},
            {"name": {"type": "string", "enum": list(self.PRESETS)}, "repeat": {"type": "integer"}, "timeline": {"type": "array", "items": {"type": "object"}}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "list": return {"presets": list(self.PRESETS), "validation": "supplier_action_ids_required"}
        return self.bridge.publish(f"choreography.{action}", {k: v for k, v in args.items() if not k.startswith("_")})


class U1StatePlugin:
    def __init__(self, bridge): self.bridge = bridge
    def get_tools(self):
        return [
            tool("state", "sensor", "U1 Pro supplier state bridge", topic_out=[{"topic": self.bridge.state_topic, "format": "data/json"}]),
            tool("command_ack", "sensor", "Query correlated U1 Pro supplier command acknowledgement", {"type": "object", "properties": {"id": {"type": "string"}}, "required": ["id"]}),
            tool("capabilities", "sensor", "U1 Pro adapter capability and connection discovery"),
            tool("ros_graph", "sensor", "Live U1 Pro ROS2 graph"),
        ]
    def start(self): pass
    def stop(self): self.bridge.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "info":
            return {"state": "ready", "topic_out": [{"topic": self.bridge.state_topic, "format": "data/json"}]} if name == "state" else {"state": "ready"}
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if name == "state": return self.bridge.snapshot()
        if name == "command_ack": return self.bridge.acknowledgement(args["id"])
        if name == "ros_graph": return self.bridge.graph()
        if name == "capabilities": return {
            "model": "UBTECH U1 Pro", "binding": "configurable_ros2_json_bridge", "supplier_idl_connected": False,
            "capabilities": ["system", "locomotion", "navigation", "body_trajectory", "head_gaze", "hands", "gestures", "dance", "speech_synthesis", "speech_recognition", "audio", "face_tracking", "face_enrollment", "multimodal_interaction"],
        }
        return None


class U1VendorPlugin:
    def __init__(self, bridge): self.bridge = bridge
    def get_tool(self):
        return tool("vendor_command", "actuator", "Pass through an additional U1 Pro supplier command", action_schema(
            {"send": (["command", "params"], "Publish a raw correlated supplier command")},
            {"command": {"type": "string"}, "params": {"type": "object"}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        return self.bridge.publish(args["command"], args.get("params", {}))


def build_plugins(config, namespace, ros2):
    bridge = JsonCommandBridge(config, namespace, ros2, "u1_pro")
    return [U1StatePlugin(bridge), U1SystemPlugin(bridge), U1LocomotionPlugin(bridge), U1BodyPlugin(bridge), U1HeadPlugin(bridge), U1HandsPlugin(bridge), U1SpeechPlugin(bridge), U1InteractionPlugin(bridge), U1ChoreographyPlugin(bridge), U1VendorPlugin(bridge)]
