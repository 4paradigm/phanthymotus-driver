#!/usr/bin/env python3
"""Manual hardware smoke testing script for LinkerHand L20 USB-CAN."""

from __future__ import annotations

import sys
import threading
import time

from common.vendor_runtime import load_config
from hand_device import HandPlugin, _SingleHandController
from can_client import L20CanClient, FINGER_NAMES


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in ("--read-only", "--move-index-and-restore", "--tactile-matrix"):
        print("Usage: python3 hardware_smoke.py --read-only | --move-index-and-restore | --tactile-matrix", file=sys.stderr)
        return 2

    config = load_config(__file__)
    hand_config = config.get("plugins", {}).get("hand", {})
    plugin = HandPlugin(config, hand_config)

    mode = sys.argv[1]

    if mode == "--read-only":
        print("=== LinkerHand L20 Hardware Smoke Test (Read-Only) ===")
        found = plugin._discover()
        print(f"Discovered hands on CAN bus: {found}")
        if not found:
            print("No L20 hand responded on CAN bus! Please check USB-CAN cable and 24V power.", file=sys.stderr)
            return 1

        for side, channel in found.items():
            print(f"\n--- Testing Hand: {side} on {channel} ---")
            ctrl = _SingleHandController(hand_config, side, channel, plugin._bus_lock)
            client = ctrl.get()
            try:
                positions = client.get_positions()
                print("16 Active Joint Positions (0–255):")
                for k, v in positions.items():
                    print(f"  {k:15s}: {v}")

                speeds = client.get_speeds()
                print(f"Joint Speeds: {speeds}")

                faults = client.get_faults()
                print(f"Joint Faults: {faults}")

                print("Reading Tactile Sensors...")
                client.request_tactile_scalar()
                client.request_normal_force()
                client.request_tangential_force()
                time.sleep(0.06)
                snap = client.get_tactile_snapshot()
                print("Fingertip Scalar Touch (0-255):", snap["touch_scalar"])
                print("Fingertip Normal Force:", snap["normal_force"])
                print("Fingertip Tangential Force:", snap["tangential_force"])
            finally:
                ctrl.close()
        return 0

    if mode == "--tactile-matrix":
        print("=== LinkerHand L20 Fingertip Tactile Matrix (12x6) ===")
        found = plugin._discover()
        if not found:
            print("No hand detected.", file=sys.stderr)
            return 1
        side, channel = list(found.items())[0]
        ctrl = _SingleHandController(hand_config, side, channel, plugin._bus_lock)
        client = ctrl.get()
        try:
            print("Querying 12x6 tactile matrix for all 5 fingers...")
            client.request_matrix_touch()
            time.sleep(0.1)
            matrices = client.get_matrix_touch()
            for finger, mat in matrices.items():
                print(f"\n--- {finger.upper()} (12 rows x 6 columns) ---")
                for row in mat:
                    print("  " + " ".join(f"{v:3d}" if v >= 0 else "  ." for v in row))
        finally:
            ctrl.close()
        return 0

    if mode == "--move-index-and-restore":
        print("=== LinkerHand L20 Index Finger Round-Trip Motion Test ===")
        completed = threading.Event()
        terminal: dict = {}

        def on_complete(action_id, status, result):
            terminal.update(action_id=action_id, status=status, result=result)
            completed.set()

        plugin._acp_callback = on_complete

        # Select the discovered side explicitly on every device request.
        found = plugin._discover()
        if not found:
            print("No hand detected on CAN bus.", file=sys.stderr)
            return 1
        side = list(found.keys())[0]
        inst_id = "smoke_test_inst"
        plugin.dispatch("config", {"can_channel": found[side]})

        def move(joint_name: str, pos_val: int):
            completed.clear()
            accepted = plugin.dispatch("set_positions", {
                "instance_id": inst_id,
                "bound_side": side,
                joint_name: pos_val,
                "confirm_motion": True,
            })
            print(f"Dispatched motion: {accepted}")
            if not completed.wait(15) or terminal.get("action_id") != accepted["action_id"]:
                raise RuntimeError("Motion completion timeout")
            if terminal["status"] != "completed":
                raise RuntimeError(f"Motion error: {terminal}")
            return terminal["result"]

        original_pos = None
        try:
            state = plugin.dispatch("get_state", {"bound_side": side})
            original_pos = state["positions"]["index_pitch"]
            target = original_pos - 20 if original_pos > 40 else original_pos + 20
            print(f"Moving index_pitch from {original_pos} -> {target}...")
            res1 = move("index_pitch", target)
            print(f"Move arrived: {res1['positions']['index_pitch']}")

            print(f"Restoring index_pitch back to {original_pos}...")
            res2 = move("index_pitch", original_pos)
            print(f"Restored to: {res2['positions']['index_pitch']}")
            print("Motion smoke test PASSED!")
            return 0
        finally:
            if original_pos is not None:
                try:
                    move("index_pitch", original_pos)
                except Exception:
                    pass
            plugin.stop()


if __name__ == "__main__":
    raise SystemExit(main())
