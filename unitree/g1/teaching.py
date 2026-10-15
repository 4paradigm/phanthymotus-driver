"""Adapt G1's existing trajectory owner to the shared skill workbench."""
import math
import secrets
import time

from common.teaching import TeachingPlugin


class G1TeachingAdapter:
    title = "宇树 G1 · 双臂"
    resources = ["arm_l", "arm_r"]
    # Includes the original clip, transition to its start, and SDK handback.
    completion_timeout = 750
    presentation = {
        "session_start": "controller",
        "joint_groups": [{"label": "左臂", "count": 5}, {"label": "右臂", "count": 5}],
        "grippers": False,
        "recording_label": "双臂 · 10 关节",
        "prepare_hint": "检查本体反馈、机型、设备身份和遥操通道。",
        "next_hint": "在画布启动遥操，双方双臂下垂，松握并长按 X+A 建立基准。",
        "trial_hint": "恢复物体布置。试运行会先低速移动到录制起点，再按原时间回放。",
        "conditions_placeholder": "例如：物体位置与示教时一致，双臂到录制起点的路径畅通。",
    }
    replay_instruction = (
        "执行时由 G1 原控制器先低速移动到录制起点，再按原时间回放；"
        "本次确认须包含移向起点的路径和录制动作的活动空间。"
        "起点差异仅供显示，不要求 PiPER 的关节或夹爪容差。"
        "不得额外发回零、切模式、换设备或改速度指令。"
        "completed 表示轨迹发送及停止交权完成，实际动作效果仍需现场确认。"
    )
    checks = (
        ("feedback", "本体反馈与模式", "请启动 G1 驱动并在原控制卡检查反馈和本体模式。"),
        ("model", "机型与设备身份", "请检查 G1_23 模型及持久化设备身份挂载。"),
        ("control", "控制状态", "请结束回放、复位或其他控制动作，再重新检查。"),
        ("transport", "VR 通道", "请连接 VR 输入 → G1 遥操卡，并启动画布项目。"),
    )

    def __init__(self, control):
        self.control = control
        self.trajectory = control.trajectory
        self.arm = control.executor
        self.preview = None
        self.epoch = 0
        self.recording_session = None

    def _call(self, action, **args):
        result = self.control.dispatch(action, args)
        if result.get("error") or result.get("state") == "error":
            raise ValueError(result.get("code") or result.get("error") or "g1_control_error")
        return result

    def info(self):
        value = self._call("info")
        arm = self.arm.status()
        recording = value.get("recording") or {}
        writing = bool(recording.get("recording") or recording.get("stopping"))
        replay = value.get("replay_active", False)
        can_prepare = not (writing or replay or self.arm.opening or self.control._reset_operation)
        active = bool(value.get("operator_session_id") or arm["ownership_held"])
        running = bool(value.get("binding") and not self.control._closed.is_set())
        reason = value.get("reason")
        hints = {
            "waiting_baseline_confirmation": "双方双臂下垂，松开握把，长按 X+A 直到基准建立。",
            "release_buttons_then_regrip": "基准已建立。松开 X+A，开始录制后双握操作。",
            "regrip_required": "松开双握把，再次双握后继续示范。",
            "resetting": "正在发送原厂复位请求，请现场确认结果后重新建立基准。",
        }
        hint = hints.get(reason) or value.get("input_status", {}).get("hint")
        if not running:
            hint = "请在画布启动 G1 遥操，再松握并长按 X+A 建立基准。"
        elif value["can_record"]:
            hint = "可以开始录制。按住双握把示范，松握暂停。"
        return {"state": "teleop" if running else value["state"], "reason": reason,
                "busy": active or writing or replay or not can_prepare,
                "can_prepare": can_prepare, "session_id": value.get("operator_session_id"),
                "input_fresh": value["input_fresh"], "mapped": value["mapped"],
                "can_record": value["can_record"], "recording": recording,
                "teleop_hint": hint, "last_action": value.get("last_action"),
                "checks": [{"id": k, "label": label, "ok": None} for k, label, _ in self.checks],
                "recordings": self._call("recordings")["recordings"]}

    def check(self):
        def control():
            if not self.control.live:
                raise ValueError("live_teleop_required")
            if not self.info()["can_prepare"] or self.arm.state == "fault":
                raise ValueError("motion_busy_or_fault")

        def transport():
            state = self.control.input_status()
            if state["state"] in ("waiting_binding", "stopped", "transport_error"):
                raise ValueError(state.get("hint") or state["state"])

        operations = {"feedback": self.arm._fresh, "model": self.trajectory.binding,
                      "control": control, "transport": transport}
        checks = []
        for key, label, help_text in self.checks:
            try:
                operations[key]()
                checks.append({"id": key, "label": label, "ok": True})
            except Exception as exc:
                checks.append({"id": key, "label": label, "ok": False,
                               "reason": str(exc), "help": help_text})
        return {"ready": all(c["ok"] for c in checks), "checks": checks}

    def begin(self, args):
        # G1's operator creates the mapping with X+A; opening a page never does.
        raise ValueError("请从画布启动遥操，并在头显松握、长按 X+A 建立基准")

    def record(self):
        with self.control._lock:
            if not self.control.can_record():
                raise ValueError("live_mapped_teleop_required")
            session = self.control._operator
        result = self._call("record_start")
        if self.control._operator != session:
            self._call("record_stop")
            raise ValueError("teaching_session_changed")
        self.recording_session = session
        return result

    def finish(self):
        current = self.control._operator
        if current and self.recording_session and current != self.recording_session:
            raise ValueError("示教会话已改变，请先停止当前遥操")
        result = self._call("record_stop")
        if result.get("complete"):
            stopped = self._call("stop")
            if not stopped.get("authority_released"):
                raise ValueError("stop_unconfirmed")
            return {**result, "stop": stopped}
        return result

    def stop(self):
        self.epoch += 1
        self.preview = None
        return self._call("stop")

    def inspect(self, filename):
        recording = self._call("inspect", filename=filename)
        return {**recording, "preview": [
            {"t": f["t"], "kind": f["kind"],
             "joints": [math.degrees(q) for q in f["target_q"]],
             "feedback_joints": [math.degrees(q) for q in f["feedback_q"]]}
            for f in recording["preview"]]}

    def start_difference(self, filename):
        try:
            result = self._call("start_difference", filename=filename)
            return {"ready": result["ready"], "can_trial": True, "mode": "move_to_start",
                    "message": "将先低速移动到录制起点，请确认移动路径畅通。",
                    "target_joints_deg": [math.degrees(q) for q in result["target_q"]],
                    "joint_delta_deg": [math.degrees(q) for q in result["joint_delta_rad"]]}
        except Exception as exc:
            return {"ready": False, "can_trial": False, "error": str(exc)}

    def play(self, filename, args):
        epoch, generation = self.epoch, self.control._generation
        self.trajectory._idle()
        recording = self.inspect(filename)
        start = self.start_difference(filename)
        if not start["can_trial"]:
            raise ValueError(start["error"])
        if epoch != self.epoch or generation != self.control._generation:
            raise ValueError("replay_cancelled")
        signature = (filename, recording["sha256"], recording["binding"], generation, epoch)
        if not args.get("execute"):
            ident = secrets.token_hex(16)
            self.preview = (ident, time.monotonic()+60, signature)
            return {"preview_id": ident, "executed": False,
                    "duration_seconds": recording["duration_seconds"], "binding": recording["binding"],
                    "binding_label": "G1 · " + recording["binding"]["device_id"][:12],
                    "description": self.presentation["trial_hint"]}
        preview, self.preview = self.preview, None
        if (not preview or preview[0] != args.get("preview_id") or time.monotonic() >= preview[1]
                or preview[2] != signature):
            raise ValueError("teaching_preview_expired")
        if args.get("workspace_clear") is not True or args.get("take_control") is not True:
            raise ValueError("本次接管和移动路径需要确认")
        result = self._call("play", filename=filename, expected_generation=generation,
                            expected_sha256=recording["sha256"])
        return {**result, "state": result["status"]}

    def wait(self, action_id):
        deadline = time.monotonic()+self.completion_timeout
        while time.monotonic() < deadline:
            result = self._call("action_status", action_id=action_id)
            if result["status"] in {"completed", "cancelled", "error"}:
                return result
            time.sleep(.05)
        self.stop()
        return {"action_id": action_id, "status": "error", "result": {"reason": "completion_timeout"}}


def teaching_plugin(control):
    return TeachingPlugin(G1TeachingAdapter(control), control.trajectory.directory.parent / "skills", "g1_teaching")
