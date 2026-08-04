#!/usr/bin/env python3
"""BumiEDU Max plugins backed by Noetix's public native DDS SDK."""

from __future__ import annotations

import importlib
import json
import os
import sys
import threading
import time
from pathlib import Path

from common.vendor_runtime import action_schema, jsonable, tool


COMMANDS = (
    "WALK", "SWING", "SHAKE", "CHEER", "RUN", "START", "SWITCH",
    "STARTTEACH", "SAVETEACH", "ENDTEACH", "PLAYTEACH", "DANCE",
    "FALLTOSTAND", "STANDTOFALL", "DANCE1", "DANCE2", "TEAR", "DEFAULT",
)


def _sdk_path(config: dict) -> Path:
    configured = os.environ.get("BUMI_SDK_PATH") or config.get("sdk", {}).get("path", "/opt/noetix-bumi-sdk")
    return Path(str(configured)).resolve()


class BumiSDK:
    """Load the vendor pybind modules only when the driver starts."""

    def __init__(self, config):
        root = _sdk_path(config)
        build = root / "build"
        for candidate in (build, root):
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
        dds_file = Path(str(config.get("sdk", {}).get("dds_config", root / "config" / "dds.xml")))
        os.environ.setdefault("CYCLONEDDS_URI", f"file://{dds_file}")
        try:
            high = importlib.import_module("highcontrol_py")
        except ImportError as exc:
            raise RuntimeError(f"Noetix highcontrol_py not found under {root}; set BUMI_SDK_PATH") from exc
        self.high_module = high
        self.high = high.HighController.instance()
        self.high.init()
        self.low_module = None
        self.low = None
        if bool(config.get("sdk", {}).get("enable_low_level", True)):
            try:
                self.low_module = importlib.import_module("lowcontrol_py")
                self.low = self.low_module.LowController.instance()
                if self.low.init() is False:
                    raise RuntimeError("Noetix LowController.init() returned false")
            except Exception as exc:
                print(f"[bumi] low-level SDK unavailable: {exc}", flush=True)

    def command(self, name: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, index: int = 0):
        if name not in COMMANDS:
            raise ValueError(f"Unknown Bumi command: {name}")
        self.high.publish_cmd(float(x), float(y), float(yaw), getattr(self.high_module.ControlCmd, name), int(index))

    def snapshot(self) -> dict:
        battery = self.high.get_robot_bms_data()
        imu = self.high.get_imu_data()
        joints = self.high.get_joint_state()
        joystick = self.high.from_dds_get_joydata()
        return {
            "state": "connected",
            "mode": int(self.high.get_mode()),
            "battery": jsonable(battery),
            "imu": jsonable(imu),
            "joints": [jsonable(item) for item in joints],
            "joystick": jsonable(joystick),
            "low_level_available": self.low is not None,
        }


class BumiNodes:
    def __init__(self, config, namespace, ros2, sdk):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.config = config
        self.sdk = sdk
        self.robot = Node("bumi_driver_robot", context=ros2.ctx_robot)
        self.core = Node("bumi_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)
        self.state_topic = f"/{namespace}/bumi/state"
        self.state_pub = self.core.create_publisher(String, self.state_topic, 10)
        self.snapshot_lock = threading.Lock()
        self.last_snapshot = {"state": "starting"}
        self.closed = False

    def publish_snapshot(self):
        from std_msgs.msg import String

        snapshot = self.sdk.snapshot()
        with self.snapshot_lock:
            self.last_snapshot = snapshot
        msg = String()
        msg.data = json.dumps(snapshot, ensure_ascii=False)
        self.state_pub.publish(msg)

    def cached_snapshot(self):
        with self.snapshot_lock:
            return dict(self.last_snapshot)

    def close(self):
        if self.closed: return
        self.closed = True; self.robot.destroy_node(); self.core.destroy_node()


class BumiStatePlugin:
    def __init__(self, nodes, config):
        self.nodes = nodes
        self.interval = 1.0 / max(1.0, float(config.get("state_rate_hz", 20.0)))
        self.stop_event = threading.Event()
        self.thread = None

    def get_tools(self):
        return [
            tool("state", "sensor", "BumiEDU Max mode, joint, IMU, battery and joystick state", topic_out=[{"topic": self.nodes.state_topic, "format": "data/json"}]),
            tool("capabilities", "sensor", "BumiEDU Max native SDK capability discovery"),
        ]

    def start(self):
        def loop():
            while not self.stop_event.is_set():
                try:
                    self.nodes.publish_snapshot()
                except Exception as exc:
                    with self.nodes.snapshot_lock:
                        self.nodes.last_snapshot = {"state": "error", "error": str(exc)}
                self.stop_event.wait(self.interval)
        self.thread = threading.Thread(target=loop, daemon=True, name="bumi-state")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        self.nodes.close()

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "ready", "topic_out": [{"topic": self.nodes.state_topic, "format": "data/json"}]} if args.get("_tool_name") == "state" else {"state": "ready"}
        if action == "start": return {"state": "running"}
        if action == "stop": self.stop(); return {"state": "idle"}
        if args.get("_tool_name") == "state": return self.nodes.cached_snapshot()
        if args.get("_tool_name") == "capabilities":
            return {
                "model": "Noetix BumiEDU Max", "native_sdk": "noetix_sdk_bumi",
                "low_level_available": self.nodes.sdk.low is not None,
                "capabilities": ["walk", "run", "turn", "swing", "shake", "cheer", "dance", "dance1", "dance2", "fall_recovery", "controlled_fall", "teaching_record_playback", "low_level_joint", "imu", "battery", "joystick"],
            }
        return None


class BumiMotionPlugin:
    def __init__(self, sdk, config):
        self.sdk = sdk
        self.rate = float(config.get("control", {}).get("high_rate_hz", 100.0))
        self.cancel = threading.Event()
        self.thread = None
        self.status = {"state": "idle"}

    def get_tool(self):
        return tool("motion", "actuator", "BumiEDU Max walking, running, turning and high-level actions", action_schema(
            {
                "move": (["x", "y", "yaw", "duration", "gait"], "Move with the official WALK or RUN command"),
                "stop": ([], "Stop base motion"),
                "action": (["name", "index"], "Publish any official Noetix ControlCmd"),
                "status": ([], "Get active motion status"),
            },
            {
                "x": {"type": "number"}, "y": {"type": "number"}, "yaw": {"type": "number"},
                "duration": {"type": "number"}, "gait": {"type": "string", "enum": ["walk", "run"]},
                "name": {"type": "string", "enum": list(COMMANDS)}, "index": {"type": "integer"},
            },
        ))

    def start(self): pass
    def stop(self): self.cancel.set(); self.sdk.command("DEFAULT")

    def _launch_move(self, args):
        if self.thread and self.thread.is_alive():
            raise RuntimeError("Bumi motion controller is busy")
        self.cancel.clear()
        x, y, yaw = (float(args.get(key, 0.0)) for key in ("x", "y", "yaw"))
        duration = max(0.0, float(args.get("duration", 1.0)))
        command = "RUN" if str(args.get("gait", "walk")).lower() == "run" else "WALK"
        def run():
            started = time.monotonic()
            try:
                while not self.cancel.is_set() and time.monotonic() - started < duration:
                    self.sdk.command(command, x, y, yaw)
                    self.status = {"state": "running", "command": command, "elapsed": time.monotonic() - started}
                    self.cancel.wait(1.0 / max(1.0, self.rate))
                self.sdk.command("DEFAULT")
                self.status = {"state": "cancelled" if self.cancel.is_set() else "completed", "command": command}
            except Exception as exc:
                self.status = {"state": "failed", "error": str(exc)}
        self.thread = threading.Thread(target=run, daemon=True, name="bumi-motion")
        self.thread.start()
        return {"state": "running", "command": command, "duration": duration}

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "move": return self._launch_move(args)
        if action == "stop": self.cancel.set(); self.sdk.command("DEFAULT"); return {"state": "stopped"}
        if action == "status": return dict(self.status)
        if action == "action":
            name = str(args["name"]).upper()
            self.sdk.command(name, float(args.get("x", 0.0)), float(args.get("y", 0.0)), float(args.get("yaw", 0.0)), int(args.get("index", 0)))
            return {"state": "published", "command": name, "index": int(args.get("index", 0))}
        return None


class BumiDancePlugin:
    ACTIONS = ("SWING", "SHAKE", "CHEER", "DANCE", "DANCE1", "DANCE2")
    def __init__(self, sdk): self.sdk = sdk
    def get_tool(self):
        return tool("dance", "actuator", "BumiEDU Max official gestures and three dance programs", action_schema(
            {"list": ([], "List public SDK dances and gestures"), "play": (["name"], "Play an official dance or gesture"), "stop": ([], "Stop the current dance")},
            {"name": {"type": "string", "enum": list(self.ACTIONS)}},
        ))
    def start(self): pass
    def stop(self): self.sdk.command("DEFAULT")
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "list": return {"actions": list(self.ACTIONS)}
        if action == "play":
            name = str(args["name"]).upper()
            if name not in self.ACTIONS: raise ValueError(f"Unknown Bumi dance: {name}")
            self.sdk.command(name); return {"state": "published", "name": name}
        if action == "stop": self.sdk.command("DEFAULT"); return {"state": "stopped"}
        return None


class BumiTeachingPlugin:
    def __init__(self, sdk): self.sdk = sdk
    def get_tool(self):
        return tool("teaching", "actuator", "BumiEDU Max action teaching record/save/play workflow", action_schema(
            {"begin": ([], "Begin action teaching"), "save": (["index"], "Save taught action"), "end": ([], "End teaching"), "play": (["index"], "Play a saved taught action")},
            {"index": {"type": "integer", "minimum": 0}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        mapping = {"begin": "STARTTEACH", "save": "SAVETEACH", "end": "ENDTEACH", "play": "PLAYTEACH"}
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action in mapping:
            index = int(args.get("index", 0)); self.sdk.command(mapping[action], index=index)
            return {"state": "published", "command": mapping[action], "index": index}
        return None


class BumiLowLevelPlugin:
    def __init__(self, sdk, config):
        self.sdk = sdk
        self.joint_count = int(config.get("control", {}).get("joint_count", 21))
    def get_tool(self):
        return tool("joints", "actuator", "BumiEDU Max native low-level joint command", action_schema(
            {"command": (["commands"], "Set arbitrary motor position, velocity, torque and gains"), "state": ([], "Read native low-level motor state")},
            {"commands": {"type": "array", "items": {"type": "object", "properties": {"motor_id": {"type": "integer"}, "position": {"type": "number"}, "velocity": {"type": "number"}, "torque": {"type": "number"}, "kp": {"type": "number"}, "kd": {"type": "number"}}, "required": ["motor_id", "position"]}}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready", "available": self.sdk.low is not None}
        if action == "stop": return {"state": "idle"}
        if self.sdk.low is None: raise RuntimeError("Bumi low-level SDK is not available on this robot/image")
        if action == "state": return {"joints": jsonable(self.sdk.low.get_joint_state())}
        if action == "command":
            commands = list(args.get("commands", []))
            if not commands: raise ValueError("commands cannot be empty")
            result = []
            for values in commands:
                motor_id = int(values["motor_id"])
                if motor_id < 0 or motor_id >= self.joint_count: raise ValueError(f"motor_id outside 0..{self.joint_count - 1}")
                cmd = self.sdk.low_module.MotorCmd(); cmd.motor_id = motor_id
                cmd.pos = float(values["position"]); cmd.vel = float(values.get("velocity", 0.0)); cmd.tau = float(values.get("torque", 0.0)); cmd.kp = float(values.get("kp", 1.0)); cmd.kd = float(values.get("kd", 0.1))
                result.append(cmd)
            self.sdk.low.set_joint(result)
            return {"state": "published", "motors": [int(item.motor_id) for item in result]}
        return None


def build_plugins(config, namespace, ros2):
    sdk = BumiSDK(config)
    nodes = BumiNodes(config, namespace, ros2, sdk)
    return [BumiStatePlugin(nodes, config), BumiMotionPlugin(sdk, config), BumiDancePlugin(sdk), BumiTeachingPlugin(sdk), BumiLowLevelPlugin(sdk, config)]
