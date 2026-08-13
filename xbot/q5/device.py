"""Verified non-motion cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module deliberately contains only the verified read-only state/battery cards
and the vendor AudioPlay card. Unsupported legacy cards are not shipped.
"""

from __future__ import annotations

import json
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import AudioPlay
from xbot_common_interfaces.srv import SetVolume

# main.py resolves all card classes through this module. Keep the direct
# control cards here as explicit exports while their implementation remains
# consolidated in direct_control.py.
from direct_control import (
    ArmControlPlugin,
    BaseDrivePlugin,
    HandControlPlugin,
    HandGesturePlugin,
    HeadControlPlugin,
)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def _wait_for_future(future, timeout_sec: float):
    """Wait for work completed by main.py's shared executor thread."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


class StatePlugin:
    """Read-only joint and Q5 FSM state card."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "state", "type": "sensor", "multiInstance": False,
            "description": "Q5 joint feedback and READY/ACTIVE state.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            # q5_bridge_worker publishes these JSON topics in Agent Core's DDS domain.
            "topic_out": [
                {"topic": f"/{self._ns}/q5/joints_state", "format": "data/json"},
                {"topic": f"/{self._ns}/q5/robot_status", "format": "data/json"},
            ],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        joint = self._client.snapshot()
        status = self._client.sensor_snapshot("robot_status")
        if not status.get("available"):
            status = self._client.sensor_snapshot("query_state")
        return {
            "state": "running",
            "joint_state": {
                "available": joint.get("available", False),
                "fresh": joint.get("fresh", False),
                "age_ms": joint.get("age_ms"),
                "joint_count": joint.get("joint_count", 0),
                "position_unit": joint.get("position_unit", "rad"),
            },
            "robot_status": {
                "available": status.get("available", False),
                "fresh": status.get("fresh", False),
                "age_ms": status.get("age_ms"),
                "state": status.get("state"),
                "message": status.get("message", ""),
                "source": status.get("source_service", "/xbot_state"),
            },
            "motion_manager_lifecycle": self._client.get_lifecycle_state(),
        }


class BatteryPlugin:
    """Read-only battery state card, including verified board firmware."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "battery", "type": "sensor", "multiInstance": False,
            "description": "Q5 battery level, electrical readings, and power-board firmware.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": f"/{self._ns}/battery_state", "format": "data/json"}],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        battery = self._client.sensor_snapshot("battery")
        firmware = self._client.sensor_snapshot("battery_version")
        return {
            "state": "running",
            "available": battery.get("available", False),
            "fresh": battery.get("fresh", False),
            "age_ms": battery.get("age_ms"),
            "percentage": battery.get("percentage"),
            "voltage_v": battery.get("voltage"),
            "current_a": battery.get("current"),
            "temperature_c": battery.get("temperature"),
            "power_supply_status": battery.get("power_supply_status"),
            "firmware": firmware.get("components", {}),
        }


class AudioPlugin:
    """Vendor audio playback via /audio_player/play and paired services."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")

    def get_tool(self):
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 vendor audio playback, volume, stop, and status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "play", "set_volume", "stop_audio", "is_play", "info"]},
                "mode": {"type": "integer", "enum": [0, 1, 2, 3], "description": "0=id, 1=path, 2=item JSON, 3=file name"},
                "id": {"type": "integer"}, "path": {"type": "string"}, "item": {"type": "string"},
                "file_name": {"type": "string"}, "force_play": {"type": "boolean"},
                "timeout": {"type": "integer", "minimum": 0},
                "channel": {"type": "string", "enum": ["default", "channel1", "channel2", "channel3"]},
                "version": {"type": "string", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device}
        if action == "play":
            return self._play(args)
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    def _play(self, args):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "/audio_player/play is unavailable"}
        goal = AudioPlay.Goal()
        goal.mode = int(args.get("mode", 1))
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.file_name = str(args.get("file_name", ""))
        goal.channel = str(args.get("channel", "default"))
        goal.timeout = int(args.get("timeout", 0))
        goal.version = str(args.get("version", "v1"))
        goal_handle = _wait_for_future(self._action_client.send_goal_async(goal), 5.0)
        if goal_handle is None:
            return {"state": "error", "message": "audio goal timed out"}
        if not goal_handle.accepted:
            return {"state": "error", "message": "audio goal rejected"}
        response = _wait_for_future(goal_handle.get_result_async(), max(10.0, goal.timeout + 2.0))
        if response is None:
            return {"state": "error", "message": "audio result timed out"}
        return {"state": "ok" if response.result.success else "error", "message": response.result.message}

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
        return {"state": "ok" if response.success else "error", "volume": req.volume, "message": response.message}

    def _stop_audio(self):
        if not self._srv_stop.service_is_ready():
            return {"state": "error", "message": "/audio_player/stop_play is unavailable"}
        response = _wait_for_future(self._srv_stop.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "stop-audio request timed out"}
        return {"state": "ok" if response.success else "error", "message": response.message}

    def _is_playing(self):
        if not self._srv_is_play.service_is_ready():
            return {"state": "error", "message": "/audio_player/is_play is unavailable"}
        response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "is-play request timed out"}
        return {"state": "ok", "is_playing": response.success, "message": response.message}
