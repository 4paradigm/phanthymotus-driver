#!/usr/bin/env python3
"""RealMan 二指夹爪位置控制。"""

from __future__ import annotations

from common.ros2_json_bridge import JsonCommandBridge
from common.vendor_runtime import action_schema, tool

POSITION_MIN = 0
POSITION_MAX = 1000


def _position(value) -> int:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("position must be a number") from exc
    return int(max(POSITION_MIN, min(POSITION_MAX, round(numeric))))


class GripperNodes:
    def __init__(self, config, namespace, ros2):
        self.bridge = JsonCommandBridge(config, namespace, ros2, "realman_gripper")

    def close(self):
        self.bridge.close()


class GripperPlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self) -> dict:
        return tool(
            "gripper",
            "actuator",
            "RealMan 二指夹爪位置控制。位置范围 0~1000，对应夹爪行程 0~120 mm。",
            action_schema(
                {"set_position": (["position"], "设置二指夹爪目标位置")},
                {
                    "position": {
                        "type": "integer",
                        "minimum": POSITION_MIN,
                        "maximum": POSITION_MAX,
                        "description": "夹爪驱动器目标位置，0~1000，对应 0~120 mm",
                    }
                },
            ),
        )

    def start(self):
        pass

    def stop(self):
        self.nodes.close()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action != "set_position":
            return None
        position = _position(args.get("position"))
        return self.nodes.bridge.publish(
            "hand_follow_pos",
            {"hand_pos": [position]},
        )


def build_plugins(config, namespace, ros2):
    nodes = GripperNodes(config, namespace, ros2)
    return [GripperPlugin(nodes)]
