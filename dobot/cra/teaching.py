"""Teach card: bounded transitions, persistent recording, ServoJ stream playback."""

from __future__ import annotations

from collections import deque
import os
import queue
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool

if __package__:
    from .completion import MotionCompletion
    from .control import CommandRejected
    from .feedback import positive_number
    from .model import JOINT_NAMES
    from .trajectory import (SAMPLE_MS, TrajectoryRecorder, TrajectoryStore, at_rest_after_drag, drag_state_fault,
                             protection_fault)
else:
    from completion import MotionCompletion
    from control import CommandRejected
    from feedback import positive_number
    from model import JOINT_NAMES
    from trajectory import (SAMPLE_MS, TrajectoryRecorder, TrajectoryStore, at_rest_after_drag, drag_state_fault,
                            protection_fault)

# ServoJ stream: one follow target every 20 ms (field-tested at 50 Hz, ~5 ms ACK).
SERVO_PERIOD_S = 0.02
# Pause/resume change the playback rate linearly over this time, along the path.
SERVO_RAMP_S = 0.5
# A send this late means the controller has already run out of targets.
SERVO_STALL_S = 0.1
# Actual joints must stay this close to some target sent within the lag window.
SERVO_TRACKING_DEG = 3.0
SERVO_TRACKING_WINDOW_S = 0.5
SERVO_SETTLE_S = 10
# Playback speed relative to the recording, as selectable tiers.
SPEED_TIERS = {"0.25x": 0.25, "0.5x": 0.5, "1x": 1.0, "1.5x": 1.5, "2x": 2.0}
# MovJ speed tiers for reaching/returning to the start: percent used for both
# velocity and acceleration (the old default was 10 for each).
APPROACH_TIERS = {"slow": 10, "normal": 25, "fast": 50}


class ServoPlayback:
    """Position along one recording, in ms of the original take. `plan` carries the
    play options (pause_at_start, return_to_start, args) through pause/resume."""

    def __init__(self, trajectory_name, points, speed, plan=None):
        self.name = trajectory_name
        self.points = points
        self.speed = float(speed)
        self.plan = plan or {}
        self.duration_ms = (len(points) - 1) * SAMPLE_MS
        self.position_ms = 0.0

    def at(self, position_ms):
        index = min(int(position_ms // SAMPLE_MS), len(self.points) - 2)
        alpha = min(1.0, (position_ms - index * SAMPLE_MS) / SAMPLE_MS)
        return [a + alpha * (b - a) for a, b in zip(self.points[index], self.points[index + 1])]


class _Finished(Exception):
    """play/resume already produced their terminal status and result."""


class Teaching(MotionCompletion):
    ACTIONS = ("start_trajectory", "stop_trajectory", "play", "pause", "resume", "stop_motion")

    def __init__(self, control, config, notify=None, store=None):
        super().__init__(control, config, notify=notify, tool_name="teach")
        settings = config.get("teach", {})
        self.max_duration = positive_number(settings.get("max_recording_s", 500), "teach.max_recording_s")
        if self.max_duration > 500:
            raise ValueError("teach.max_recording_s must not exceed 500")
        self.gap_ms = positive_number(settings.get("max_gap_ms", 50), "teach.max_gap_ms")
        if self.gap_ms > 50:
            raise ValueError("teach.max_gap_ms must not exceed the 50 ms sample interval")
        # Time for the arm to report drag after StartDrag.
        self.drag_start_timeout = positive_number(settings.get("drag_start_timeout_s", 5), "teach.drag_start_timeout_s")
        if self.drag_start_timeout > 60:
            raise ValueError("teach.drag_start_timeout_s must not exceed 60")
        # Time for the arm to report rest in mode 5 after StopDrag (or after the operator's button).
        self.drag_exit_timeout = positive_number(settings.get("drag_exit_timeout_s", 10), "teach.drag_exit_timeout_s")
        if self.drag_exit_timeout > 600:
            raise ValueError("teach.drag_exit_timeout_s must not exceed 600")
        self._shutting_down = False
        self.store = store if store is not None else TrajectoryStore(
            os.environ.get("DOBOT_TEACH_DATA_DIR", settings.get("data_dir", "data/dobot-teach")))
        self.recorder = None
        self._drag_cleanup_failed = False
        self._resume_context = None
        self._held_lease = None
        # 500 seconds at 0.25x takes 2000 seconds. Include getting to the start and
        # returning to it, preparation, SDK timeouts and a bounded completion/callback margin.
        self.contract_timeout += self.max_duration / 0.25 + 30 + self.timeout

    def definition(self, config_schema):
        schema = action_schema({
            "info": ([], "查看示教卡功能和参数说明"),
            "start_trajectory": (["name", "confirm_move"], "清错、按需使能、关闭安全皮肤（拖拽会触发它）后由驱动发送 StartDrag 进入拖拽并开始录制；名称已存在则报错；完成通知表示机械臂已进入拖拽、正在录制；录制自行结束（操作者按拖拽键、达到上限、异常）时会再通知一次"),
            "stop_trajectory": (["confirm_move"], "结束录制：驱动发送 StopDrag 退出拖拽，以新反馈确认模式 5 停稳后保存；若操作者已提前退出拖拽，录制已自动结束，本操作返回其结果"),
            "list_trajectories": ([], "列出本地轨迹名称、录制时间、时长、点数和完整性"),
            "delete_trajectory": (["name", "confirm_move"], "按名称删除本地轨迹"),
            "play": (["name", "speed", "approach_speed", "detect_collision", "pause_at_start", "return_to_start", "confirm_move"],
                     "先运动到轨迹起点并核验到位（已在起点则跳过）；pause_at_start=true 时到位后停住等待 resume；否则准备使能/皮肤后由驱动以 50 Hz ServoJ 逐点发送录制关节角，保留录制时间比例；return_to_start=true 时回放结束后再运动回起点。安全皮肤默认开启"),
            "pause": (["confirm_move"], "回放中：沿轨迹 0.5 秒减速并停在路径上；到起点/回起点运动中：暂停控制器队列"),
            "resume": (["confirm_move"], "从暂停处继续：回放沿轨迹 0.5 秒加速，到起点/回起点运动直接 Continue（到起点后接着回放）；停在起点（pause_at_start）时开始回放；回放中不清错、不重新 Enable、不设置安全皮肤"),
            "stop_motion": (["confirm_move"], "终止回放或到起点运动；录制时中断并保留标为不完整的数据，StopDrag 并确认停稳后释放机械臂"),
            "start": ([], "启动共享反馈连接"),
            "stop": ([], "结束本卡后台任务并释放资源"),
            "config": ([], "更新共享控制柜 IP/反馈端口；录制要求端口 30004"),
        }, {
            "name": {"type": "string", "maxLength": 80, "description": "轨迹名称，唯一标识一条轨迹；录制时同名将报错，回放和删除按名称查找"},
            "speed": {"type": "string", "enum": list(SPEED_TIERS), "default": "1x",
                      "description": "回放速度档位，相对录制速度：1x 为原速，2x 两倍速，0.25x 四分之一速度"},
            "approach_speed": {"type": "string", "enum": list(APPROACH_TIERS), "default": "slow",
                               "description": "运动到起点/回到起点的速度档位：slow 慢（默认，速度/加速度 10%，与旧版默认一致）、normal 中（25%）、fast 快（50%）"},
            "pause_at_start": {"type": "string", "enum": ["true", "false"], "default": "false",
                               "description": "true：到达并核验起点后暂停，等待 resume 再开始回放；false：直接继续回放"},
            "return_to_start": {"type": "string", "enum": ["true", "false"], "default": "false",
                                "description": "true：回放结束后再运动回轨迹起点并核验；false：停在终点"},
            "detect_collision": {"type": "string", "enum": ["true", "false"], "default": "true",
                                 "description": "仅控制安全皮肤：true（默认）开启并设各区域 low；false 关闭。不修改本体碰撞检测。resume 保留此前设置"},
            "confirm_move": {"type": "boolean", "description": "手动输入 true，确认本次操作"},
        })
        schema["x-completion"] = {"actions": list(self.ACTIONS), "timeout": self.contract_timeout}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop_motion", "params": {"confirm_move": True}}}
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definition = tool("teach", "actuator", "CR3A 拖拽示教录制、轨迹管理与 ServoJ 关节轨迹回放", schema)
        definition["configSchema"] = config_schema
        return definition

    def dispatch(self, action, args):
        if action == "list_trajectories":
            return {"trajectories": self.store.list_trajectories()}
        if action == "delete_trajectory":
            if args.get("confirm_move") is not True:
                raise ValueError("confirm_move=true is required")
            with self._lock:
                if self._active is not None or self._resume_context is not None:
                    raise RuntimeError("stop the teaching operation before deleting a trajectory")
                recorder = self.recorder
                if recorder is not None and not recorder.finished.is_set() and recorder.record["name"] == self.store.normalize(args.get("name")):
                    raise RuntimeError("stop recording before deleting it")
                return self.store.delete(args.get("name"))
        return self.start(action, args)

    @staticmethod
    def _flag(args, field, default):
        """Optional boolean: omitted/null gives the default; "true"/"false" text is accepted
        because the platform may deliver form values as strings. Anything else is rejected."""
        value = args.get(field)
        if value is None:
            return default
        if type(value) is bool:
            return value
        if type(value) is str and value.strip().lower() in ("true", "false"):
            return value.strip().lower() == "true"
        raise ValueError(f"{field} must be true or false (received {value!r})")

    def start(self, action, args, *, tool_name=None):
        if action not in self.ACTIONS:
            raise ValueError(f"unsupported teaching action: {action}")
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required")
        if action == "start_trajectory":
            self.store.check_new(args.get("name"))
        if action == "play":
            self.store._directory(args.get("name"))
            speed, approach = args.get("speed", "1x"), args.get("approach_speed", "slow")
            if type(speed) is not str or speed not in SPEED_TIERS:
                raise ValueError(f"speed must be one of {', '.join(SPEED_TIERS)}")
            if type(approach) is not str or approach not in APPROACH_TIERS:
                raise ValueError(f"approach_speed must be one of {', '.join(APPROACH_TIERS)}")
            flags = {field: self._flag(args, field, field == "detect_collision")
                     for field in ("detect_collision", "pause_at_start", "return_to_start")}
            args = {"name": args.get("name"), "confirm_move": True, "speed": speed, "approach_speed": approach,
                    **flags}
        try:
            context = None if self._notify is not None else self._callback_context()
        except Exception:
            if action not in ("stop_trajectory", "stop_motion"):
                raise
            context = None
        with self._lock:
            recording = self.recorder is not None and not self.recorder.finished.is_set()
            if action == "stop_trajectory" and self.recorder is None:
                raise RuntimeError("no recording has been started")
            if action in ("pause", "resume") and self._resume_context is None and self._active is None:
                raise RuntimeError("no teaching motion is available to resume or pause")
            if self._resume_context is not None and self._active is None and action not in ("resume", "stop_motion"):
                raise RuntimeError("resume or stop the paused teaching motion first")
            if recording and action not in ("stop_trajectory", "stop_motion"):
                raise RuntimeError("stop recording before starting another teaching action")
            if self._active is not None and action not in ("pause", "stop_motion"):
                raise RuntimeError("another teach action is active")
            if action == "pause" and self._active is not None and self._active["action"] == "pause":
                raise RuntimeError("a pause is already in progress")
            lease = self.control.ownership.acquire("teach")
            self._held_lease = lease
            interrupted = self._active
            # A ServoJ stream is slowed to rest along its path; a move to the start is
            # paused through the controller queue, which needs that job cancelled.
            stream_pause = (action == "pause" and interrupted is not None and "pause" in interrupted
                            and not interrupted.get("moving"))
            if action == "stop_motion":
                # Includes a stream still decelerating under a pause request.
                for job in self._jobs.values():
                    job["cancel"].set()
            elif interrupted is not None:
                if stream_pause:
                    interrupted["pause"].set()
                else:
                    interrupted["cancel"].set()
            identifier = f"dobot_cra_teach_{action}_{uuid4().hex}"
            active = {"action_id": identifier, "action": action, "cancel": threading.Event(), "lease": lease,
                      "tool": tool_name or "teach", "interrupting": interrupted is not None,
                      "interrupted": interrupted, "stream_pause": stream_pause, "moving": False}
            pending = self._resume_context or {}
            if action == "play" or (action == "resume" and ("playback" in pending or "then" in pending)):
                active["pause"] = threading.Event()
            if action == "play" or (action == "resume" and "target" in pending):
                active["moving"] = True
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
        """Leave drag and confirm it. StopDrag is sent once, and only when a packet received
        after this request shows drag_status: never on stale feedback and never in the
        inconsistent state (mode 6 without drag status), where it left the field arm
        damped but unreported. Success requires two consecutive packets at rest in mode 5."""
        subscription = self.control.feedback.subscribe()
        try:
            started = time.monotonic()
            previous, settled, last, stop_sent = None, 0, None, False
            while True:
                if self._shutting_down:
                    # A stopping driver cannot wait for the operator indefinitely.
                    wait_s = min(wait_s, self.control.enable_timeout)
                item = self._read_fresh(subscription, started + wait_s)
                if item is None:
                    if last is not None and (last["robot_mode"] == 6 or last["drag_status"]):
                        raise TimeoutError(f"arm still in drag after {wait_s:g} s; press the drag button "
                                           "on the arm to exit drag, then run stop_motion or stop_trajectory")
                    raise TimeoutError("fresh feedback did not confirm leaving drag at rest in mode 5")
                state = last = item["data"]
                fault = protection_fault(state) or drag_state_fault(state)
                if fault:
                    raise RuntimeError(f"cannot confirm exit from drag: {fault}")
                if state["drag_status"] and not stop_sent:
                    self.control.stop_drag()
                    stop_sent, settled = True, 0
                    previous = item["packet"]
                    continue
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
        """Exit drag (StopDrag only per _confirm_drag_exit's guards), then release the arm."""
        try:
            self._confirm_drag_exit(self.drag_exit_timeout)
            self._drag_cleanup_failed = False
            self.control.ownership.release(lease)
            return None
        except Exception as exc:
            self._drag_cleanup_failed = True
            # Keep the arm reserved until an explicit recovery/stop succeeds.
            return f"could not confirm exit from drag: {exc}"

    def _begin_record(self, active, args, context=None):
        feedback = self.control.feedback
        if feedback.port != 30004:
            raise RuntimeError("recording requires feedback port 30004 (8 ms)")
        if not hasattr(feedback, "subscribe"):
            raise RuntimeError("recording feedback subscription is unavailable")
        record = self.store.create(args.get("name"), feedback.address)
        name = record["name"]
        subscription = None
        recorder = None
        drag_error = None
        try:
            self.control.prepare_record(cancel=active["cancel"])
            subscription = feedback.subscribe()
            try:
                self.control.start_drag(active["cancel"])
            except Exception as exc:
                # An unanswered StartDrag may still have taken effect: leave drag below.
                if getattr(exc, "dobot_command_attempted", None) and not isinstance(exc, CommandRejected):
                    drag_error = self._leave_drag(active["lease"])
                raise
            # Packets that arrived while StartDrag was in flight are stale for the recorder.
            while True:
                try:
                    subscription.queue.get_nowait()
                except queue.Empty:
                    break
            recorder = TrajectoryRecorder(self.store, record, subscription,
                                          lambda: self._leave_drag(active["lease"]),
                                          max_gap_ms=self.gap_ms, max_duration_s=self.max_duration,
                                          max_wait_s=self.drag_start_timeout)
            with self._lock:
                if active["cancel"].is_set():
                    raise RuntimeError("teaching interrupted")
                self.recorder = recorder
                recorder.start()
            deadline = time.monotonic() + self.drag_start_timeout + self.drag_exit_timeout + self.control.receive_timeout + 5
            while not recorder.drag_started.is_set():
                if active["cancel"].is_set():
                    raise RuntimeError("teaching interrupted")
                if recorder.finished.is_set():
                    raise RuntimeError(recorder.record.get("reason") or "recording ended before the arm entered drag")
                if time.monotonic() >= deadline:
                    raise TimeoutError("arm did not report drag after StartDrag")
                recorder.drag_started.wait(0.02)
            threading.Thread(target=self._report_recording_end, args=(recorder, context), daemon=True,
                             name="dobot-teach-recording-report").start()
            return {**self.store.summary(record), "drag": "entered"}
        except Exception as exc:
            if recorder is not None and not recorder.finished.is_set():
                self._claim_report(recorder)
                recorder.stop(str(exc))
                recorder.finished.wait(self.drag_exit_timeout + self.control.receive_timeout + 5)
            elif subscription is not None:
                subscription.close()
            if recorder is None or not recorder._points:
                # Nothing was captured: do not leave the name occupied.
                try:
                    self.store.discard(name)
                except Exception:
                    record.update(state="incomplete", reason=str(exc))
                    self.store.save(record)
            if drag_error:
                raise RuntimeError(f"{exc}; {drag_error}") from exc
            raise

    def _claim_report(self, recorder):
        """Exactly one party reports the end of a recording: the stop action that ends it
        (its own completion), or the watcher when the recording ends by itself."""
        with self._lock:
            if getattr(recorder, "reported", False):
                return False
            recorder.reported = True
            return True

    def _report_recording_end(self, recorder, context):
        """A recording that ends without a stop request (operator button, limit, fault)
        has no pending action to complete: send the agent an unsolicited ACP completion.
        The platform keeps only the status for a success, so the end reason is also in the id."""
        recorder.finished.wait()
        if not self._claim_report(recorder):
            return
        record = recorder.record
        result = {key: record[key] for key in ("name", "state", "end_reason", "reason", "note", "duration_s",
                                                "point_count") if key in record}
        status = "completed" if record.get("state") == "recorded" else "error"
        self._complete(f"dobot_cra_teach_recording_{record.get('end_reason', 'interrupted')}_{uuid4().hex}",
                       status, result, context)

    def _end_record(self, aborted=False):
        with self._lock:
            recorder = self.recorder
        if recorder is None:
            raise RuntimeError("no recording has been started")
        if not recorder.finished.is_set():
            self._claim_report(recorder)
            recorder.stop("recording cancelled" if aborted else None)
            if not recorder.finished.wait(self.drag_exit_timeout + self.control.receive_timeout + 5):
                raise TimeoutError("recording cleanup did not finish")
        result = self.store.summary(recorder.record)
        if recorder.record["state"] != "recorded" and not aborted:
            raise RuntimeError(f"recording incomplete ({result['name']}): {result.get('reason')}")
        return result

    def _recording_to_stop(self):
        # Pair this observation with _begin_record's install-and-start critical
        # section so stop cannot miss a recorder being handed off.
        with self._lock:
            return self.recorder is not None and (
                not self.recorder.finished.is_set() or self._drag_cleanup_failed)

    def _fresh_state(self, context):
        sample = self.control.feedback.snapshot()
        state = sample.get("data")
        if not sample.get("connected") or sample.get("stale") or state is None:
            raise RuntimeError(f"feedback unavailable during {context}")
        return sample, state

    def _check_servo_state(self, sent):
        """Per-target guard: protection, unexpected modes and path tracking."""
        _, state = self._fresh_state("playback")
        fault = protection_fault(state)
        if fault is None and state["drag_status"]:
            fault = "drag active"
        if fault is None and state["robot_mode"] not in (5, 7):
            fault = f"controller mode {state['robot_mode']}"
        if fault:
            raise RuntimeError(f"playback interrupted: {fault}")
        actual = state["joint_state"]["q_actual"]
        error = min(max(abs(a - b) for a, b in zip(actual, target)) for _, target in sent)
        if error > SERVO_TRACKING_DEG:
            raise RuntimeError(f"playback interrupted: arm is {error:.2f} deg away from the streamed path")

    def _stream(self, active, playback, *, ramp_in):
        """Send ServoJ targets along the recording until its end, a pause (after a
        deceleration along the path) or cancellation. Returns "finished" or "paused"."""
        cancel, pause = active["cancel"], active["pause"]
        if pause.is_set():
            return "paused"
        rate = 0.0 if ramp_in else playback.speed
        change = playback.speed * SERVO_PERIOD_S / SERVO_RAMP_S
        sent = deque([(time.monotonic(), playback.at(playback.position_ms))])
        deadline = time.monotonic()
        while True:
            if cancel.is_set():
                raise RuntimeError("teaching interrupted")
            wanted = 0.0 if pause.is_set() else playback.speed
            rate = min(wanted, rate + change) if rate < wanted else max(wanted, rate - change)
            if pause.is_set() and rate == 0:
                return "paused"
            if playback.position_ms >= playback.duration_ms:
                return "finished"
            position = min(playback.duration_ms, playback.position_ms + rate * SERVO_PERIOD_S * 1000)
            target = playback.at(position)
            now = time.monotonic()
            while len(sent) > 1 and now - sent[0][0] > SERVO_TRACKING_WINDOW_S:
                sent.popleft()
            self._check_servo_state(sent)
            self.control.servo_j(target, SERVO_PERIOD_S, cancel)
            playback.position_ms = position
            sent.append((time.monotonic(), target))
            deadline += SERVO_PERIOD_S
            delay = deadline - time.monotonic()
            if delay < 0:
                if -delay > SERVO_STALL_S:
                    raise RuntimeError(f"ServoJ stream stalled for {-delay * 1000:.0f} ms")
                deadline = time.monotonic()
            elif cancel.wait(delay):
                raise RuntimeError("teaching interrupted")

    def _confirm_rest(self, cancel, timeout):
        """Two consecutive fresh packets at rest in mode 5 after a ServoJ pause."""
        deadline = time.monotonic() + timeout
        last = self.control.feedback.snapshot().get("packets_received")
        settled = 0
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("teaching interrupted")
            sample, state = self._fresh_state("pause")
            fault = protection_fault(state)
            if fault:
                raise RuntimeError(f"pause interrupted: {fault}")
            packet = sample.get("packets_received")
            if packet is not None and last is not None and packet > last:
                settled = settled + 1 if at_rest_after_drag(state) else 0
                last = packet
                if settled >= 2:
                    return {"reason": "paused", "robot_mode": state["robot_mode"]}
            cancel.wait(0.005)
        raise TimeoutError("pause did not reach rest in mode 5")

    def _check_servo_resume(self, playback):
        _, state = self._fresh_state("resume")
        fault = protection_fault(state)
        if fault is None and not at_rest_after_drag(state):
            fault = f"arm must be at rest in mode 5 (mode {state['robot_mode']})"
        if fault:
            raise RuntimeError(f"cannot resume playback: {fault}")
        target = playback.at(playback.position_ms)
        if max(abs(a - b) for a, b in zip(state["joint_state"]["q_actual"], target)) > self.tolerance:
            raise RuntimeError("arm moved away from the paused playback position; stop_motion, then play again")

    def _play(self, active, playback, *, ramp_in):
        outcome = self._stream(active, playback, ramp_in=ramp_in)
        if outcome == "paused":
            self._resume_context = {"name": playback.name, "playback": playback}
            return "cancelled", {"reason": "paused", "name": playback.name,
                                 "position_s": round(playback.position_ms / 1000, 3)}
        baseline = self.control.feedback.snapshot().get("packets_received")
        result = self._monitor_trajectory(active["cancel"], playback.at(playback.duration_ms), None, baseline,
                                          SERVO_SETTLE_S, observed_running=True)
        result["name"] = playback.name
        if playback.plan.get("return_to_start"):
            with self._lock:
                if active["cancel"].is_set():
                    raise RuntimeError("teaching interrupted")
                # From here a pause goes through the controller queue, like the move to the start.
                active["moving"] = True
            try:
                # prepare_playback already set the skin for this play; do not redo it before the return.
                back = self._move_to_start(active, playback.name, playback.points[0], playback.plan["args"],
                                           skin_applied=True)
            except Exception as exc:
                if active["cancel"].is_set():
                    raise
                raise RuntimeError(f"playback finished but returning to the start failed: {exc}") from exc
            self._resume_context = None
            result.update(returned_to_start=True, return_max_error_deg=back["max_error_deg"])
        return "completed", result

    def _move_to_start(self, active, name, start, args, then=None, *, skin_applied=False):
        """MovJ to the first recorded pose; a pause during it uses the controller queue
        and keeps `then` so that resume continues into the stream."""
        parameters = {**args, **{f"{joint}_deg": value for joint, value in zip(JOINT_NAMES, start)}}
        response = self.control.execute_motion("move_j", parameters, cancel=active["cancel"], owner="teach",
                                               skin_applied=skin_applied)
        active["submitted"] = True
        context = {"name": name, "target": start, "timeout": self.timeout}
        if then is not None:
            context["then"] = then
        self._resume_context = context
        result = self._monitor_trajectory(active["cancel"], start, response.get("result_id"),
                                          response["submission_baseline"], self.timeout)
        result["name"] = name
        return result

    def _stream_trajectory(self, active, name, plan, *, moved):
        """Start position verified (by MovJ arrival or by prepare_playback): stream the take,
        or hold here until resume when the caller asked to pause at the start."""
        with self._lock:
            active["moving"] = False
            self._resume_context = None
            if active["cancel"].is_set():
                raise RuntimeError("teaching interrupted")
        points = plan["points"]
        # The MovJ that just arrived already set the skin with these same arguments.
        self.control.prepare_playback(points[0], {**plan["args"], "start_tolerance_deg": self.tolerance},
                                      cancel=active["cancel"], skin_applied=moved)
        active["submitted"] = True
        playback = ServoPlayback(name, points, plan["speed"], plan)
        if plan["pause_at_start"]:
            with self._lock:
                if active["cancel"].is_set():
                    raise RuntimeError("teaching interrupted")
                self._resume_context = {"name": name, "playback": playback}
                active["hold"] = True
            # Reported like pause: not a completed run, resume starts the playback.
            return "cancelled", {"reason": "paused_at_start", "name": name, "moved_to_start": moved}
        status, result = self._play(active, playback, ramp_in=False)
        result["moved_to_start"] = moved
        return status, result

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
        keep_owner = False
        began = time.monotonic()
        try:
            if action == "start_trajectory":
                result = self._begin_record(active, args, context)
                keep_owner = True
            elif action == "stop_trajectory":
                if self._drag_cleanup_failed and self.recorder.finished.is_set():
                    self._leave_drag(active["lease"])
                result = self._end_record()
            elif action == "stop_motion" and self._recording_to_stop():
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
            elif action == "pause" and active["stream_pause"]:
                stream = active["interrupted"]
                deadline = time.monotonic() + SERVO_RAMP_S + 2 * self.control.receive_timeout + 5
                while stream["thread"].is_alive():
                    if active["cancel"].is_set():
                        raise RuntimeError("teaching interrupted")
                    if time.monotonic() >= deadline:
                        raise TimeoutError("playback did not decelerate to a pause")
                    stream["thread"].join(0.02)
                if self._resume_context is None or "playback" not in self._resume_context:
                    raise RuntimeError("playback ended before it could be paused")
                result = self._confirm_rest(active["cancel"], self.control.enable_timeout)
                result["name"] = self._resume_context["name"]
                keep_owner = True
            elif action in ("pause", "stop_motion"):
                if action == "pause":
                    self._wait_motion_started(active["cancel"])
                response = self.control.execute_motion(action, args, cancel=active["cancel"], owner="teach")
                active["submitted"] = True
                result = self._monitor_control(action, active["cancel"], response["submission_baseline"])
                keep_owner = action == "pause"
                if action == "stop_motion":
                    self._resume_context = None
            elif action == "resume":
                pending = self._resume_context
                if pending is None:
                    raise RuntimeError("no teaching motion is available to resume")
                if "playback" in pending:
                    self._check_servo_resume(pending["playback"])
                    self._resume_context = None
                    active["submitted"] = True
                    status, result = self._play(active, pending["playback"], ramp_in=True)
                    raise _Finished
                response = self.control.execute_motion("resume", args, cancel=active["cancel"], owner="teach")
                active["submitted"] = True
                queue_id = response.get("resumed_command_id") or response.get("result_id")
                result = self._monitor_trajectory(active["cancel"], pending["target"], queue_id,
                                                  response["submission_baseline"], pending["timeout"])
                result["name"] = pending["name"]
                if "then" in pending:
                    status, result = self._stream_trajectory(active, pending["name"], pending["then"], moved=True)
                    raise _Finished
                self._resume_context = None
            else:
                record, data = self.store.playback(args["name"], self.control.feedback.address)
                points = [point["joints_deg"] for point in data["points"]]
                start = points[0]
                motion = {"velocity": APPROACH_TIERS[args["approach_speed"]],
                          "acceleration": APPROACH_TIERS[args["approach_speed"]],
                          "detect_collision": args["detect_collision"], "confirm_move": True}
                plan = {"points": points, "speed": SPEED_TIERS[args["speed"]], "args": motion,
                        "pause_at_start": args["pause_at_start"], "return_to_start": args["return_to_start"]}
                _, state = self._fresh_state("play")
                actual = state["joint_state"]["q_actual"]
                moved = max(abs(a - b) for a, b in zip(actual, start)) > self.tolerance
                if moved:
                    # Reach the start first; arrival is verified from feedback before streaming.
                    self._move_to_start(active, record["name"], start, motion, then=plan)
                status, result = self._stream_trajectory(active, record["name"], plan, moved=moved)
                raise _Finished
            status = "completed"
        except _Finished:
            if status == "completed":
                result["elapsed_s"] = round(time.monotonic() - began, 3)
        except Exception as exc:
            status = "cancelled" if active["cancel"].is_set() else "error"
            result = {"reason": str(exc)}
            uncertain_motion = getattr(exc, "dobot_command_attempted", None) in ("MovJ", "ServoJ", "Continue")
            if action == "stop_motion":
                keep_owner = True
            if action == "resume" and self._resume_context is not None:
                keep_owner = True
            # A driver shutdown cancels a ServoJ stream mid-path: stop the arm as well.
            if (status == "error" or self._shutting_down) and (
                    (active.get("submitted") and action in ("play", "resume")) or
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
                # Holding at the start (pause_at_start) keeps the arm reserved for resume.
                if active.get("hold") and self._resume_context is not None:
                    keep_owner = True
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
