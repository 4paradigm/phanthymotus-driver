#!/usr/bin/env python3
from __future__ import annotations
import threading
import can
from common.vendor_runtime import action_schema, tool

FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]

class LinkerHandNodes:
    def __init__(self, config, namespace):
        self.can_iface = config.get("connection", {}).get("can_iface", "can0")
        self.can_id = int(config.get("hands", [{}])[0].get("can_id", 0x28))
        self.bus = can.interface.Bus(channel=self.can_iface, interface="socketcan")

    def send(self, data):
        msg = can.Message(arbitration_id=self.can_id, is_extended_id=False, data=data[:8])
        self.bus.send(msg)

    def close(self):
        self.bus.shutdown()

class HandPlugin:
    def __init__(self, nodes, cfg):
        self.nodes = nodes

    def get_tool(self):
        return tool("hand", "actuator", "LinkerHand L20 control",
            action_schema({"open": ([], "open"), "close": ([], "close")}, {}))

    def start(self): pass
    def stop(self): self.nodes.close()

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "started"}
        if action == "stop":
            return {"state": "stopped"}
        if action == "info":
            return {"state": "ready", "tool": "hand"}
        if action == "open":
            self.nodes.send([0x02, 255, 255, 255, 255, 255])
            return {"state": "opening"}
        if action == "close":
            self.nodes.send([0x02, 0, 0, 0, 0, 0])
            return {"state": "closing"}
        return None

def build_plugins(config, namespace, ros2=None):
    nodes = LinkerHandNodes(config, namespace)
    return [HandPlugin(nodes, config.get("plugins", {}).get("hand", {}))]
