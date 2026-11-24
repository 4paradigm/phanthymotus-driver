"""Read-only device diagnostics on the shared PiPER CAN receiver."""
import time

from common.vendor_runtime import action_schema, tool


class PiperDiagnostics:
    name = "piper_diagnostics"

    def __init__(self, runtime):
        self.runtime = runtime

    def get_tools(self):
        description = "Read PiPER CAN connection, feedback freshness and controller faults."
        schema = action_schema({"read": ([], description)}, {})
        schema["additionalProperties"] = False
        schema["properties"]["action"].update(default="read", title="Action",
                                              oneOf=[{"const": "read", "title": "Read diagnostics"}])
        definition = tool(self.name, "sensor", description, schema)
        return [{**definition, "title": "Device diagnostics"}]

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def read(self):
        snapshot = self.runtime.read_monitor_snapshot()
        connected = snapshot["connected"]
        fresh = snapshot["fresh"]
        stamp = snapshot["feedback_time"]
        faults = snapshot["faults"] if fresh else None
        gripper = snapshot["gripper"]
        state = "feedback_unavailable"
        if fresh:
            arm_status = faults["arm_status"]
            normal = arm_status.startswith(("NORMAL", "TEACHING", "REPLAY", "TRAJECTORY", "RECORD"))
            has_fault = bool(faults["error_code"] or faults["motor_faults"] or not normal
                             or (gripper and gripper["error_code"]))
            state = "fault" if has_fault else "ok"
        elif not connected:
            state = "disconnected"
        return {
            "state": state,
            "model": "AgileX PiPER",
            "can_interface": snapshot["can_interface"],
            "can_receiver_connected": connected,
            "feedback_fresh": fresh,
            "feedback_time": stamp,
            "feedback_age_ms": max(0, round((time.time() - stamp) * 1000)) if stamp is not None else None,
            "control_mode": snapshot["control_mode"],
            "joint_enabled": snapshot["enabled"],
            "controller": faults,
            "gripper_error_code": gripper["error_code"] if gripper is not None else None,
            "reasons": {key: snapshot["reasons"][key]
                        for key in ("connection", "feedback", "gripper") if key in snapshot["reasons"]},
        }

    def dispatch(self, action, args):
        allowed = {"_tool_name", "instance_id"}
        if args.get("input_topic") == "":
            allowed.add("input_topic")
        if set(args) - allowed:
            raise ValueError("Device diagnostics does not accept command arguments.")
        if action in {self.name, "read", "info"}:
            return self.read()
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        raise ValueError("Device diagnostics only supports read and lifecycle actions.")
