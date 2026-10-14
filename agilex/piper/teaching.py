"""PiPER adapter for the shared teaching UI; all motion stays in #395's owner."""
import math
from pathlib import Path
import time

from agilex.piper.trajectory import read_recording
from common.teaching import TeachingPlugin
from common.teleop_contract import MAX_INPUT_AGE_NS


class PiperTeachingAdapter:
    title = "松灵 PiPER · 双臂"
    checks = (
        ("connection", "机器人连接", "请检查双臂连接和 CAN 配置。"),
        ("calibration", "标定与左右绑定", "请核验标定文件和左右臂绑定。"),
        ("feedback", "电机、夹爪与反馈", "请在原控制卡检查模式、使能、夹爪和反馈。"),
        ("permission", "运动许可", "请在操作许可卡完成本次授权后重新检查。"),
        ("control", "控制状态", "请先结束当前控制动作，再重新检查。"),
        ("transport", "VR 通道", "请连接 VR 输入到双臂遥操卡，并启动方案。"),
    )

    def __init__(self, paired):
        self.paired = paired
        self.motion = paired.motion

    def _call(self, card, action, args=None):
        return self.paired.dispatch(action, {**(args or {}), "_tool_name": card})

    @staticmethod
    def _confirmation(args):
        return {k: args[k] for k in ("execute", "preview_id", "workspace_clear", "take_control") if k in args}

    def info(self):
        with self.motion.lock:
            value = self.motion.status()
            latest = self.motion.latest
            fresh = bool(latest and 0 <= time.monotonic_ns() - latest["received_monotonic_ns"] < MAX_INPUT_AGE_NS)
            mapped = value["reason"] in {"mapped_waiting_for_both_grips", "following", "holding", "waiting_for_fresh_frame"}
            can_record = value["state"] == "teleop" and fresh and mapped and not self.motion.cancel.is_set()
        return {**value, "busy": value["state"] != "idle" or bool((value.get("recording") or {}).get("stopping")),
                "can_record": can_record,
                "input_fresh": value["state"] == "teleop" and fresh,
                "mapped": value["state"] == "teleop" and mapped,
                "checks": [{"id": key, "label": label, "ok": None} for key, label, _ in self.checks],
                "input_topic": self.paired.input_topic,
                "transport": self.paired.bus.status() if self.paired.bus else None,
                "recordings": self.motion.recordings()}

    def check(self):
        def transport():
            if not self.paired.input_topic:
                raise ValueError("请在画布连接 VR 输入 → 双臂 VR 遥操卡，并启动方案")
            if not self.paired.bus or not self.paired.bus.status()["running"]:
                raise ValueError("遥操 DDS 通道未就绪，请重新启动遥操卡")
        def control():
            if self.info()["busy"] or any(arm.gate.locked() for arm in self.paired.arms.values()):
                raise RuntimeError("robot_busy_wait_for_completion")
        operations = {
            "connection": lambda: [arm.connected() for arm in self.paired.arms.values()],
            "calibration": self.motion.configuration,
            # Read the same readiness conditions without conflating permission
            # with feedback. Neither this check nor the page enables hardware.
            "feedback": lambda: self.motion.read_pair(permission=False),
            "permission": lambda: [arm.require_motion_permission() for arm in self.paired.arms.values()],
            "control": control, "transport": transport,
        }
        checks = []
        for key, label, help_text in self.checks:
            try:
                operations[key]()
                checks.append({"id": key, "label": label, "ok": True})
            except Exception as exc:
                checks.append({"id": key, "label": label, "ok": False,
                               "reason": str(exc), "help": help_text})
        return {"ready": all(c["ok"] for c in checks), "checks": checks,
                "message": "此检查不证明头显正在跟踪；进入遥操后按实时输入状态操作。"}

    def begin(self, args):
        return self._call("piper_teleop", "begin", self._confirmation(args))

    def record(self):
        if not self.info()["can_record"]:
            raise RuntimeError("live_mapped_teleop_input_required")
        return self._call("piper_motion", "record_start")

    def finish(self):
        try:
            return self._call("piper_motion", "record_stop")
        finally:
            self._call("piper_teleop", "finish")

    def stop(self):
        return self.motion.request_stop()

    def inspect(self, filename):
        _, binding = self.motion.configuration()
        recording = read_recording(self.motion.directory, filename, binding)
        frames = recording["frames"]
        # A bounded, explicitly labelled target-joint preview; never fabricated camera footage.
        step = max(1, math.ceil(len(frames) / 180))
        sampled = frames[::step]
        if sampled[-1] is not frames[-1]:
            sampled.append(frames[-1])
        return {"filename": filename, "sha256": recording["sha256"], "binding": binding,
                "duration_seconds": recording["duration_seconds"], "frames": len(frames),
                "preview": [{"t": (f["t_ns"] - frames[0]["t_ns"]) / 1e9,
                             "joints": [round(math.degrees(q), 3) for q in f["target_q"]],
                             "widths": f["target_width_mm"]} for f in sampled]}

    def start_difference(self, filename):
        _, binding = self.motion.configuration()
        first = read_recording(self.motion.directory, filename, binding)["frames"][0]
        try:
            sample = self.motion.read_pair()
        except Exception as exc:
            return {"ready": False, "error": str(exc)}
        joint_delta = [math.degrees(a - b) for a, b in zip(sample["q"], first["target_q"])]
        width_delta = [a - b for a, b in zip(sample["widths"], first["target_width_mm"])]
        return {"ready": max(map(abs, joint_delta)) <= .05 and max(map(abs, width_delta)) <= .5,
                "target_joints_deg": [math.degrees(q) for q in first["target_q"]],
                "target_width_mm": first["target_width_mm"], "joint_delta_deg": joint_delta,
                "width_delta_mm": width_delta, "joint_tolerance_deg": .05, "width_tolerance_mm": .5}

    def play(self, filename, args):
        return self._call("piper_motion", "play", {"filename": filename, **self._confirmation(args)})

    def wait(self, action_id):
        thread = self.motion.thread
        if thread:
            thread.join(610)
        status = self.motion.status()
        outcome = status.get("last_action") or {}
        if outcome.get("action_id") == action_id and outcome.get("status") in {"completed", "cancelled", "error"}:
            return outcome
        self.stop()
        return {"action_id": action_id, "status": "error", "result": {"reason": "completion_missing_or_replaced"}}

def teaching_plugin(paired, config):
    return TeachingPlugin(PiperTeachingAdapter(paired),
                          Path(config.get("data_dir", "/opt/phanthy-motus/data/piper")) / "skills", "piper_teaching")
