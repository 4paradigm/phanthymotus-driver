"""Teach card: bounded transitions, persistent recording, native SDK playback."""

from __future__ import annotations

import math
import os
import queue
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool

if __package__:
    from .completion import MotionCompletion
    from .feedback import positive_number
    from .model import JOINT_NAMES
    from .trajectory import TrajectoryRecorder, TrajectoryStore, at_rest_after_drag, drag_state_fault, protection_fault
else:
    from completion import MotionCompletion
    from feedback import positive_number
    from model import JOINT_NAMES
    from trajectory import TrajectoryRecorder, TrajectoryStore, at_rest_after_drag, drag_state_fault, protection_fault


class Teaching(MotionCompletion):
    ACTIONS = ("start_record", "stop_record", "go_to_start", "play", "pause", "resume", "stop_motion")

    def __init__(self, control, config, notify=None, store=None):
        super().__init__(control, config, notify=notify, tool_name="teach")
        settings = config.get("teach", {})
        self.max_duration = positive_number(settings.get("max_recording_s", 500), "teach.max_recording_s")
        if self.max_duration > 500:
            raise ValueError("teach.max_recording_s must not exceed 500")
        self.gap_ms = positive_number(settings.get("max_gap_ms", 50), "teach.max_gap_ms")
        if self.gap_ms > 50:
            raise ValueError("teach.max_gap_ms must not exceed the 50 ms sample interval")
        # Time for the operator to press the drag button after stop_record/stop_motion.
        self.drag_exit_timeout = positive_number(settings.get("drag_exit_timeout_s", 60), "teach.drag_exit_timeout_s")
        if self.drag_exit_timeout > 600:
            raise ValueError("teach.drag_exit_timeout_s must not exceed 600")
        self._shutting_down = False
        self.store = store if store is not None else TrajectoryStore(
            os.environ.get("DOBOT_TEACH_DATA_DIR", settings.get("data_dir", "data/dobot-teach")),
            os.environ.get("DOBOT_TEACH_CONTROLLER_DIR", settings.get("controller_directory", "")),
            os.environ.get("DOBOT_TEACH_CSV_PROFILE", settings.get("native_csv_profile", "")),
        )
        self.recorder = None
        self._drag_cleanup_failed = False
        self._resume_context = None
        self._held_lease = None
        # 500 seconds at 0.25x takes 2000 seconds. Include getting to the start,
        # preparation, SDK timeouts and a bounded completion/callback margin.
        self.contract_timeout += self.max_duration / 0.25 + 30

    def definition(self, config_schema):
        schema = action_schema({
            "info": ([], "查看示教卡功能和参数说明"),
            "start_record": (["name", "confirm_move"], "清错、按需使能后启动录制任务并等待首次手动拖拽（不发送 StartDrag）；完成通知仅表示已开始等待"),
            "stop_record": (["confirm_move"], "结束整次录制：不发送 StopDrag；仍在拖拽时请按机械臂拖拽键退出，以新反馈确认模式 5 停稳后保存；本地录制状态与原生 CSV 状态分开返回"),
            "list_records": ([], "列出本地轨迹名称、时长、点数、完整性和原生文件状态"),
            "delete_record": (["trajectory_id", "confirm_move"], "删除指定本地录制；保留控制器上的副本"),
            "go_to_start": (["trajectory_id", "velocity", "acceleration", "detect_collision", "confirm_move"],
                            "校验本地及控制器文件，按绝对关节运动到轨迹起点，等待实际到位"),
            "play": (["trajectory_id", "speed_multiplier", "detect_collision", "confirm_move"],
                     "先确认已在起点，准备使能/皮肤后调用原生 StartPath；保留录制时间比例，关闭额外滤波"),
            "pause": (["confirm_move"], "暂停当前回放/到起点运动，保留控制器队列"),
            "resume": (["confirm_move"], "直接 Continue 继续本卡暂停的队列，不清错、不重新 Enable、不设置安全皮肤"),
            "stop_motion": (["confirm_move"], "终止回放；录制时中断并保留标为不完整的数据，等待按拖拽键退出拖拽并确认停稳后释放机械臂"),
            "start": ([], "启动共享反馈连接"),
            "stop": ([], "结束本卡后台任务并释放资源"),
            "config": ([], "更新共享控制柜 IP/反馈端口；录制要求端口 30004"),
        }, {
            "name": {"type": "string", "maxLength": 80, "description": "录制名称；每次生成独立编号，不覆盖同名录制"},
            "trajectory_id": {"type": "string", "description": "start_record、stop_record 或 list_records 返回的轨迹编号"},
            "velocity": {"type": "integer", "minimum": 1, "maximum": 100, "default": 10,
                         "description": "到起点运动速度，1–100%，默认 10%"},
            "acceleration": {"type": "integer", "minimum": 1, "maximum": 100, "default": 10,
                             "description": "到起点运动加速度，1–100%，默认 10%"},
            "speed_multiplier": {"type": "number", "minimum": 0.25, "maximum": 2, "default": 1,
                                 "description": "回放相对于录制速度的倍率；1 为原速，不受全局速度比例控制"},
            "detect_collision": {"type": "boolean", "default": False,
                                 "description": "仅控制安全皮肤：true 开启并设各区域 low；false 关闭。不修改本体碰撞检测。resume 保留此前设置"},
            "confirm_move": {"type": "boolean", "description": "手动输入 true，确认本次操作"},
        })
        schema["x-completion"] = {"actions": list(self.ACTIONS), "timeout": self.contract_timeout}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop_motion", "params": {"confirm_move": True}}}
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definition = tool("teach", "actuator", "CR3A 拖拽示教录制、轨迹管理与原生回放；原生 CSV 格式须按控制器样本配置", schema)
        definition["configSchema"] = config_schema
        return definition

    def dispatch(self, action, args):
        if action == "list_records":
            return {"records": self.store.list_records()}
        if action == "delete_record":
            if args.get("confirm_move") is not True:
                raise ValueError("confirm_move=true is required")
            with self._lock:
                if self._active is not None or self._resume_context is not None:
                    raise RuntimeError("stop the teaching operation before deleting a trajectory")
                return self.store.delete(args.get("trajectory_id"))
        return self.start(action, args)

    def start(self, action, args, *, tool_name=None):
        if action not in self.ACTIONS:
            raise ValueError(f"unsupported teaching action: {action}")
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required")
        if action in ("go_to_start", "play"):
            self.store._directory(args.get("trajectory_id"))
            if type(args.get("detect_collision", False)) is not bool:
                raise ValueError("detect_collision must be a boolean")
        if action == "go_to_start":
            for field in ("velocity", "acceleration"):
                value = args.get(field, 10)
                if type(value) is not int or not 1 <= value <= 100:
                    raise ValueError(f"{field} must be an integer from 1 to 100")
        if action == "play":
            value = args.get("speed_multiplier", 1)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0.25 <= value <= 2:
                raise ValueError("speed_multiplier must be between 0.25 and 2")
        try:
            context = None if self._notify is not None else self._callback_context()
        except Exception:
            if action not in ("stop_record", "stop_motion"):
                raise
            context = None
        with self._lock:
            recording = self.recorder is not None and not self.recorder.finished.is_set()
            if action == "stop_record" and self.recorder is None:
                raise RuntimeError("no recording has been started")
            if action in ("pause", "resume") and self._resume_context is None and self._active is None:
                raise RuntimeError("no teaching motion is available to resume or pause")
            if self._resume_context is not None and self._active is None and action not in ("resume", "stop_motion"):
                raise RuntimeError("resume or stop the paused teaching motion first")
            if recording and action not in ("stop_record", "stop_motion"):
                raise RuntimeError("stop recording before starting another teaching action")
            if self._active is not None and action not in ("pause", "stop_motion"):
                raise RuntimeError("another teach action is active")
            lease = self.control.ownership.acquire("teach")
            self._held_lease = lease
            interrupted = self._active
            if interrupted is not None:
                interrupted["cancel"].set()
            identifier = f"dobot_cra_teach_{action}_{uuid4().hex}"
            active = {"action_id": identifier, "cancel": threading.Event(), "lease": lease,
                      "tool": tool_name or "teach", "interrupting": interrupted is not None}
            self._active = active
            self._jobs[identifier] = active
            thread = threading.Thread(target=self._run_teach, args=(active, action, dict(args), context),
                                      daemon=True, name="dobot-teach-action")
            active["thread"] = thread
            try:
                thread.start()
            except Exception:
                self._active = None
                self._jobs.pop(identifier, None)
                self.control.ownership.release(lease)
                raise
        return {"state": "running", "action_id": identifier}

    @staticmethod
    def _read_fresh(subscription, deadline):
        """Next packet delivered to this subscription, or None at the deadline."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                return subscription.read(timeout=min(remaining, 0.05))
            except queue.Empty:
                continue

    def _confirm_drag_exit(self, wait_s):
        """Never sends StopDrag: a button-initiated drag is left with the arm button
        (StopDrag mid-drag left the field arm damped in mode 6 without drag status).
        Success requires two consecutive packets, received after this request, at
        rest in mode 5."""
        subscription = self.control.feedback.subscribe()
        try:
            started = time.monotonic()
            previous, settled, last = None, 0, None
            while True:
                if self._shutting_down:
                    # A stopping driver cannot wait for the operator indefinitely.
                    wait_s = min(wait_s, self.control.enable_timeout)
                item = self._read_fresh(subscription, started + wait_s)
                if item is None:
                    if last is not None and (last["robot_mode"] in (0, 6) or last["drag_status"]):
                        raise TimeoutError(f"arm still in drag after {wait_s:g} s; press the drag button "
                                           "on the arm to exit drag, then run stop_motion or stop_record")
                    raise TimeoutError("fresh feedback did not confirm leaving drag at rest in mode 5")
                state = last = item["data"]
                fault = protection_fault(state) or drag_state_fault(state)
                if fault:
                    raise RuntimeError(f"cannot confirm exit from drag: {fault}")
                if at_rest_after_drag(state):
                    consecutive = previous is not None and item["packet"] == previous + 1
                    settled = settled + 1 if settled and consecutive else 1
                else:
                    settled = 0
                previous = item["packet"]
                if settled >= 2:
                    return
        finally:
            subscription.close()

    def _leave_drag(self, lease):
        try:
            self._confirm_drag_exit(self.drag_exit_timeout)
            self._drag_cleanup_failed = False
            self.control.ownership.release(lease)
            return None
        except Exception as exc:
            self._drag_cleanup_failed = True
            # Keep the arm reserved until an explicit recovery/stop succeeds.
            return f"could not confirm exit from drag: {exc}"

    def _begin_record(self, active, args):
        feedback = self.control.feedback
        if feedback.port != 30004:
            raise RuntimeError("recording requires feedback port 30004 (8 ms)")
        if not hasattr(feedback, "subscribe"):
            raise RuntimeError("recording feedback subscription is unavailable")
        record = self.store.create(args.get("name"), feedback.address)
        subscription = None
        try:
            # No StartDrag: the operator enters/leaves drag with the arm button,
            # as many times as needed, until stop_record or the recording limit.
            self.control.prepare_record(cancel=active["cancel"])
            subscription = feedback.subscribe()
            recorder = TrajectoryRecorder(self.store, record, subscription,
                                          lambda: self._leave_drag(active["lease"]),
                                          max_gap_ms=self.gap_ms, max_duration_s=self.max_duration,
                                          max_wait_s=self.max_duration)
            with self._lock:
                if active["cancel"].is_set():
                    raise RuntimeError("teaching interrupted")
                self.recorder = recorder
                recorder.start()
            return {**self.store.summary(record), "drag": "waiting_for_first_manual_drag"}
        except Exception as exc:
            if subscription is not None:
                subscription.close()
            record.update(state="incomplete", reason=str(exc))
            self.store.save(record)
            raise

    def _end_record(self, aborted=False):
        recorder = self.recorder
        if recorder is None:
            raise RuntimeError("no recording has been started")
        if not recorder.finished.is_set():
            recorder.stop("recording cancelled" if aborted else None)
            if not recorder.finished.wait(self.drag_exit_timeout + self.control.receive_timeout + 5):
                raise TimeoutError("recording cleanup did not finish")
        result = self.store.summary(recorder.record)
        if recorder.record["state"] != "recorded" and not aborted:
            raise RuntimeError(f"recording incomplete ({result['trajectory_id']}): {result.get('reason')}")
        return result

    def _ready_record(self, identifier):
        record, data = self.store.playback(identifier, self.control.feedback.address)
        start = data["points"][0]["joints_deg"]
        reported = self.control.get_start_pose(record["controller_file"])
        if max(abs(a - b) for a, b in zip(start, reported)) > 0.001:
            raise RuntimeError("controller file start does not match the recorded joint angles")
        state = self.control.feedback.snapshot().get("data")
        if state is None or any(state[key] != data["frames"][field] for key, field in (
                ("user_coordinate", "user"), ("tool_coordinate", "tool"),
                ("user_value", "user_value"), ("tool_value", "tool_value"))):
            raise RuntimeError("controller user/tool frames differ from the recording")
        return record, data

    def _monitor_trajectory(self, cancel, target, queue_id, baseline, timeout, *, observed_running=False):
        deadline = time.monotonic() + timeout
        last_packet = baseline
        settled = 0
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("teaching interrupted")
            sample = self.control.feedback.snapshot()
            state = sample.get("data")
            if not sample.get("connected") or sample.get("stale") or state is None:
                raise RuntimeError("feedback unavailable during trajectory")
            if (state["robot_mode"] in (9, 11) or state["error_status"] or state["collision_state"] or
                    state["drag_status"] or any(state["brake_released"]) or
                    any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6"))):
                raise RuntimeError("trajectory interrupted by protection state")
            packet = sample.get("packets_received")
            if baseline is None or packet is None or packet <= baseline:
                cancel.wait(self.poll_interval)
                continue
            running = state["robot_mode"] == 7 or bool(state["running_status"])
            observed_running |= running
            correlated = (queue_id is not None and state["current_command_id"] == queue_id) or observed_running
            idle = (state["robot_mode"] == 5 and state["enable_status"] and not running and
                    not state["jog_status"] and not state["run_queued_cmd"] and not state["pause_cmd_flag"] and
                    all(abs(v) <= 0.1 for v in state["joint_state"]["qd_actual"]))
            if packet != last_packet:
                last_packet = packet
                settled = settled + 1 if idle and correlated else 0
                if settled >= 2:
                    error = max(abs(a - b) for a, b in zip(state["joint_state"]["q_actual"], target))
                    if error > self.tolerance:
                        raise RuntimeError("trajectory ended before reaching its joint target")
                    return {"reason": "target_reached", "max_error_deg": error}
            cancel.wait(self.poll_interval)
        raise TimeoutError("trajectory completion deadline exceeded")

    def _wait_motion_started(self, cancel):
        # A user can press Pause between StartPath's ACK and its first running
        # feedback. Wait for our actual motion, not a preceding skin-setting ID.
        deadline = time.monotonic() + self.control.enable_timeout
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("teaching interrupted")
            sample = self.control.feedback.snapshot()
            state = sample.get("data")
            if (self._resume_context is not None and sample.get("connected") and not sample.get("stale")
                    and state is not None and state["robot_mode"] == 7):
                return
            cancel.wait(.01)
        raise RuntimeError("no running teaching motion was confirmed for pause")

    def _run_teach(self, active, action, args, context):
        status, result = "error", {"reason": "unknown"}
        submitted = False
        keep_owner = False
        try:
            if action == "start_record":
                result = self._begin_record(active, args)
                keep_owner = True
            elif action == "stop_record":
                if self._drag_cleanup_failed and self.recorder.finished.is_set():
                    self._leave_drag(active["lease"])
                result = self._end_record()
            elif action == "stop_motion" and self.recorder is not None and (
                    not self.recorder.finished.is_set() or self._drag_cleanup_failed):
                result = self._end_record(aborted=True)
                if self._drag_cleanup_failed:
                    error = self._leave_drag(active["lease"])
                    if error:
                        raise RuntimeError(error)
                self._resume_context = None
            elif action == "stop_motion" and self._drag_cleanup_failed:
                error = self._leave_drag(active["lease"])
                if error:
                    raise RuntimeError(error)
                result = {"reason": "drag_stopped"}
            elif action in ("pause", "stop_motion"):
                if action == "pause":
                    self._wait_motion_started(active["cancel"])
                response = self.control.execute_motion(action, args, cancel=active["cancel"], owner="teach")
                submitted = True
                result = self._monitor_control(action, active["cancel"], response["submission_baseline"])
                keep_owner = action == "pause"
                if action == "stop_motion":
                    self._resume_context = None
            elif action == "resume":
                pending = self._resume_context
                if pending is None:
                    raise RuntimeError("no teaching motion is available to resume")
                response = self.control.execute_motion("resume", args, cancel=active["cancel"], owner="teach")
                submitted = True
                queue_id = response.get("resumed_command_id") or response.get("result_id")
                result = self._monitor_trajectory(active["cancel"], pending["target"], queue_id,
                                                  response["submission_baseline"], pending["timeout"])
                result["trajectory_id"] = pending["trajectory_id"]
                self._resume_context = None
            else:
                record, data = self._ready_record(args["trajectory_id"])
                start, end = data["points"][0]["joints_deg"], data["points"][-1]["joints_deg"]
                if action == "go_to_start":
                    parameters = {**args, **{f"{joint}_deg": value for joint, value in zip(JOINT_NAMES, start)}}
                    response = self.control.execute_motion("move_j", parameters, cancel=active["cancel"], owner="teach")
                    target, timeout = start, self.timeout
                else:
                    sample = self.control.feedback.snapshot()
                    actual = sample["data"]["joint_state"]["q_actual"]
                    if max(abs(a - b) for a, b in zip(actual, start)) > self.tolerance:
                        raise RuntimeError("robot is not at the trajectory start; run go_to_start first")
                    response = self.control.execute_path(record["controller_file"], start,
                                                         {**args, "start_tolerance_deg": self.tolerance}, cancel=active["cancel"])
                    target = end
                    timeout = record["duration_s"] / args.get("speed_multiplier", 1) + 30
                submitted = True
                self._resume_context = {"trajectory_id": record["trajectory_id"], "target": target, "timeout": timeout}
                result = self._monitor_trajectory(active["cancel"], target, response.get("result_id"),
                                                  response["submission_baseline"], timeout)
                result["trajectory_id"] = record["trajectory_id"]
                self._resume_context = None
            status = "completed"
        except Exception as exc:
            status = "cancelled" if active["cancel"].is_set() else "error"
            result = {"reason": str(exc)}
            uncertain_motion = getattr(exc, "dobot_command_attempted", None) in ("MovJ", "StartPath", "Continue")
            if action == "stop_motion":
                keep_owner = True
            if action == "resume" and self._resume_context is not None:
                keep_owner = True
            if status == "error" and ((submitted and action in ("go_to_start", "play", "resume")) or
                                      uncertain_motion or (action == "pause" and active["interrupting"])):
                try:
                    stop = self.control.execute_motion("stop_motion", {"confirm_move": True}, owner="teach")
                    self._monitor_control("stop_motion", threading.Event(), stop["submission_baseline"])
                    self._resume_context = None
                    keep_owner = False
                except Exception as stop_error:
                    result["stop_error"] = str(stop_error)
                    keep_owner = True
        finally:
            with self._lock:
                if self._active is active:
                    self._active = None
                if not keep_owner and not self._drag_cleanup_failed:
                    self.control.ownership.release(active["lease"])
            try:
                self._complete(active["action_id"], status, result, context)
            finally:
                with self._lock:
                    self._jobs.pop(active["action_id"], None)

    def shutdown(self):
        self._shutting_down = True
        try:
            self._shutdown()
        finally:
            self._shutting_down = False

    def _shutdown(self):
        self.interrupt()
        if self.recorder is not None and not self.recorder.finished.is_set():
            self.recorder.stop("driver stopped during recording")
            self.recorder.finished.wait(self.control.enable_timeout + self.control.receive_timeout + 5)
        super().shutdown()
        if self._resume_context is not None:
            result = self.control.execute_motion("stop_motion", {"confirm_move": True}, owner="teach")
            self._monitor_control("stop_motion", threading.Event(), result["submission_baseline"])
            self._resume_context = None
            self.control.ownership.release(self._held_lease)
        if self._drag_cleanup_failed:
            error = self._leave_drag(self._held_lease)
            if error:
                raise RuntimeError(error)
