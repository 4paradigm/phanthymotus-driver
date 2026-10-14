#!/usr/bin/env python3
"""Manual, opt-in 20/255 index-finger round-trip through the MCP hand plugin."""

from __future__ import annotations

import sys
import threading
import time

from modbus_device import HandPlugin, L20LiteConnection


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in ("--read-only", "--move-index-and-restore"):
        print("usage: hardware_smoke.py --read-only | --move-index-and-restore", file=sys.stderr)
        return 2
    connection = L20LiteConnection({"serial_port": "/dev/ttyUSB0", "side": "right"})
    if sys.argv[1] == "--read-only":
        try:
            print("positions:", connection.get().get_positions())
            return 0
        finally:
            connection.close()
    plugin = HandPlugin(connection, {"motion_speed": 60, "motion_torque": 100})
    completed = threading.Event()
    terminal = {}

    def on_complete(action_id, status, result):
        terminal.update(action_id=action_id, status=status, result=result)
        completed.set()

    # This direct hardware test has no MCP/Agent Core pending action.
    plugin._acp_callback = on_complete

    def move(joint, position):
        completed.clear()
        accepted = plugin.dispatch("set_joint", {
            "joint": joint, "position": position, "confirm_motion": True,
        })
        if not completed.wait(20) or terminal.get("action_id") != accepted["action_id"]:
            raise RuntimeError("motion completion was not received")
        if terminal["status"] != "completed":
            raise RuntimeError(f"motion {terminal['status']}: {terminal['result']}")
        deadline = time.monotonic() + 2
        while plugin.dispatch("info", {})["active_action_id"] is not None:
            if time.monotonic() >= deadline:
                raise RuntimeError("motion finished but driver did not become idle")
            time.sleep(0.01)
        return terminal["result"]

    original = None
    try:
        original = plugin.dispatch("get_state", {})["positions"]["index_root"]
        target = original - 20 if original > 40 else original + 20
        print(f"index_root initial={original}, target={target}", flush=True)
        moved = move("index_root", target)
        print(f"move: {moved['state']}, readback={moved['positions']['index_root']}", flush=True)
        return 0 if moved["state"] == "completed" else 1
    finally:
        if original is not None:
            try:
                restored = move("index_root", original)
                print(f"restore: {restored['state']}, readback={restored['positions']['index_root']}", flush=True)
            except Exception as exc:
                print(f"restore failed: {exc}", file=sys.stderr)
        plugin.stop()


if __name__ == "__main__":
    raise SystemExit(main())
