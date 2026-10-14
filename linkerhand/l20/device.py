#!/usr/bin/env python3
from __future__ import annotations

import threading

import can
from common.vendor_runtime import action_schema, tool


FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]


class LinkerHandNodes:
    """Own the SocketCAN connection and serialize its lifecycle and commands."""

    def __init__(self, config, namespace):
        self.can_iface = config.get("connection", {}).get("can_iface", "can0")
        hand = config.get("hands", [{}])[0]
        self.can_id = int(hand.get("can_id", 0x28))
        self.side = hand.get("side", "right")
        self.lock = threading.RLock()
        self.bus = None
        self.last_command = None

    def start(self):
        with self.lock:
            if self.bus is None:
                self.bus = can.interface.Bus(channel=self.can_iface, interface="socketcan")

    def close(self):
        with self.lock:
            if self.bus is not None:
                bus = self.bus
                bus.shutdown()
                self.bus = None

    def send(self, action, data):
        with self.lock:
            if self.bus is None:
                raise RuntimeError("Hand is stopped; start the card before sending commands")
            message = can.Message(
                arbitration_id=self.can_id,
                is_extended_id=False,
                data=data,
            )
            self.bus.send(message, timeout=1.0)
            self.last_command = action

    def info(self):
        with self.lock:
            return {
                "state": "ready" if self.bus is not None else "stopped",
                "can_iface": self.can_iface,
                "can_id": self.can_id,
                "side": self.side,
                "last_command": self.last_command,
                "feedback_available": False,
                "topic_in": [],
                "topic_out": [],
            }


class HandPlugin:
    def __init__(self, nodes, cfg):
        self.nodes = nodes

    def get_tool(self):
        return tool(
            "hand",
            "actuator",
            "LinkerHand L20 control",
            action_schema({"open": ([], "open"), "close": ([], "close")}, {}),
        )

    def start(self):
        self.nodes.start()

    def stop(self):
        self.nodes.close()

    def dispatch(self, action, args):
        if action == "info":
            return self.nodes.info()
        if action == "config":
            forbidden = {"can_iface", "can_id", "side", "hands", "connection", "mcp_port", "ros"} & set(args)
            if forbidden:
                raise ValueError("Hardware settings are fixed by this driver configuration")
            return {"state": "configured", "config": {}, "adapter_ok": True}
        if action == "start":
            self.start()
            return self.nodes.info()
        if action == "stop":
            self.stop()
            return self.nodes.info()
        if action not in ("open", "close"):
            raise ValueError("Unsupported hand action")

        value = 255 if action == "open" else 0
        self.nodes.send(action, [0x02, value, value, value, value, value])
        return {
            "state": "command_sent",
            "action": action,
            "feedback_available": False,
            "message": "CAN command sent; physical completion is not measured",
        }


def build_plugins(config, namespace, ros2=None):
    nodes = LinkerHandNodes(config, namespace)
    return [HandPlugin(nodes, config.get("plugins", {}).get("hand", {}))]
