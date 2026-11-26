"""Read-only device diagnostics on the shared PiPER CAN receiver."""
import os
import time

from common.vendor_runtime import action_schema, tool


DIAGNOSTIC_TEXT = {
    "en": {
        "title": "Device diagnostics",
        "action": "Read diagnostics",
        "description": "Read the current CAN connection, feedback freshness and controller faults.",
        "action_label": "Action",
        "ok": "Controller feedback is normal.",
        "fault": "Device fault detected. Open the details for controller and gripper fault codes.",
        "feedback_unavailable": "CAN is connected. No fresh feedback is available.",
        "disconnected": "The CAN feedback receiver is disconnected. Check the interface.",
        "age": "Feedback updated {ms} ms ago.",
    },
    "zh-CN": {
        "title": "设备诊断",
        "action": "读取诊断",
        "description": "读取本次 CAN 连接、反馈时效和设备故障，结果显示在卡片下方。",
        "action_label": "操作",
        "ok": "控制器反馈正常。",
        "fault": "检测到设备异常。请展开详情查看控制器和夹爪故障码。",
        "feedback_unavailable": "CAN 已连接，暂未收到有效反馈。",
        "disconnected": "CAN 接收未连接，请检查接口状态。",
        "age": "最近反馈距今 {ms} ms。",
    },
}


class PiperDiagnostics:
    name = "piper_diagnostics"

    def __init__(self, runtime):
        self.runtime = runtime
        language = os.environ.get("PIPER_DIAGNOSTICS_LANGUAGE", "en")
        self.text = DIAGNOSTIC_TEXT.get(language, DIAGNOSTIC_TEXT["en"])

    def get_tools(self):
        description = self.text["description"]
        schema = action_schema({"read": ([], description)}, {})
        schema.update({"additionalProperties": False, "x-manual-execution": True})
        schema["properties"]["action"].update(
            default="read", title=self.text["action_label"], oneOf=[{"const": "read", "title": self.text["action"]}],
        )
        definition = tool(self.name, "sensor", description, schema)
        return [{**definition, "title": self.text["title"]}]

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
        age_ms = max(0, round((time.time() - stamp) * 1000)) if stamp is not None else None
        interface = snapshot["can_interface"]
        message = f'{self.text[state]} CAN {interface}'
        if fresh and age_ms is not None:
            message += f' · {self.text["age"].format(ms=age_ms)}'
        return {
            "state": state,
            "message": message,
            "model": "AgileX PiPER",
            "can_interface": interface,
            "can_receiver_connected": connected,
            "feedback_fresh": fresh,
            "feedback_time": stamp,
            "feedback_age_ms": age_ms,
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
