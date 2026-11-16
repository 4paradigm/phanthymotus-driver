"""L20 Lite teach card: replay glove trajectories with the hand/gesture protections.

Modeled on the Dobot CRA teach card (PR #492). Built-in presets and takes recorded
from the local glove share the hand_trajectory pipeline and safety rules.
Recording streams live targets while saving glove data.
Recording and playback exclude each other.
The RS-485 bus lock is taken per frame, never for a whole playback, so the hand
card's stop is not held up behind a running trajectory.

`stop` is a lifecycle action that Core hides from the canvas, so the visible stop
is `stop_motion`. It is also the on_interrupt_motion hook, which exempts it from
the ACP barrier; hooks carry no instance_id, so it stops any playback.
"""

from __future__ import annotations

import json
import os
import shutil
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from modbus_client import JOINT_COUNT
from modbus_device import L20LiteConnection, _named
import hand_trajectory
from glove_input import GloveInput
import teach_recorder
from teach_messages import error_text, reason_text

ACTIONS = ("play", "pause", "resume")
PLAY_SPEEDS = {"0.25x": 0.25, "0.5x": 0.5, "1x": 1.0, "1.5x": 1.5, "2x": 2.0}
PLAYBACK_HZ = 20
PAUSE_RAMP_S = 0.5
# hand._move: command the midpoint first, then the target.
HALFWAY_S = 0.8
SETTLE_S = 2.0
TRACKING_DELAY_BUDGET_S = 20.0
DEFAULT_DATA_DIR = "/opt/phanthy-motus/data/l20-lite-teach"
# start_trajectory waits this long for the first glove frame.
FIRST_FRAME_S = 5.0


class PlaybackPaused(Exception):
    def __init__(self, position_ms, measured, command):
        self.position_ms = position_ms
        self.measured = measured
        self.command = command


class TeachPlugin:
    PREFIX = "hand_teach"

    def __init__(self, hand, config: dict, preset_dir=hand_trajectory.PRESET_DIR):
        self.hand = hand  # Shared discovery, open poses and RS-485 bus lock.
        self.config = config
        self.preset_dir = preset_dir
        self._lock = threading.RLock()
        self._bindings: dict[str, str] = {}
        self._connections: dict[str, L20LiteConnection] = {}
        self._active: dict | None = None
        self._paused: dict | None = None
        self._stop = threading.Event()
        self._delivery_failed = False
        self._last_result: dict | None = None
        self._recording_results: dict = {}
        self._recorder: teach_recorder.Recorder | None = None
        self._recording_owner = ""
        self._glove = GloveInput()
        self._glove_owner = None
        self._glove_settings = {name: os.environ.get("L20_GLOVE_" + name.upper()) or config.get("glove_" + name)
                                for name in ("library", "serial", "profile")}
        self.data_dir = os.environ.get("L20_TEACH_DATA_DIR") or config.get("data_dir", DEFAULT_DATA_DIR)
        self.store = teach_recorder.RecordedStore(self.data_dir)
        self.record_max_s = float(config.get("record_max_s", 60))
        self.record_gap_ms = int(config.get("record_gap_ms", hand_trajectory.MAX_GAP_MS))
        self.first_frame_s = float(config.get("first_frame_s", FIRST_FRAME_S))
        if not (1 <= self.record_max_s <= 300 and 20 <= self.record_gap_ms <= hand_trajectory.MAX_GAP_MS
                and 0.1 <= self.first_frame_s <= 30):
            raise ValueError("示教录制的时长、首帧等待时间或断流阈值配置无效")
        roll = config.get("thumb_roll_fixed", hand_trajectory.THUMB_ROLL_SAFE)
        if roll != hand_trajectory.THUMB_ROLL_SAFE:
            raise ValueError("拇指旋转固定值 thumb_roll_fixed 必须为 255")
        self.motion_speed = self._setting(config.get("motion_speed", 120), "motion_speed")
        self.approach_speed = self._setting(config.get("approach_speed", 60), "approach_speed")
        thumb_sway_speed = config.get("playback_thumb_sway_speed")
        self.playback_thumb_sway_speed = (
            None if thumb_sway_speed is None
            else self._setting(thumb_sway_speed, "playback_thumb_sway_speed")
        )
        self.torque = self._setting(config.get("motion_torque", 60), "motion_torque")
        self.thumb_speed_scaling = config.get("thumb_speed_scaling", True)
        if not isinstance(self.thumb_speed_scaling, bool):
            raise ValueError("thumb_speed_scaling 必须是布尔值")
        self.playback_thumb_speed_scaling = config.get("playback_thumb_speed_scaling", self.thumb_speed_scaling)
        if not isinstance(self.playback_thumb_speed_scaling, bool):
            raise ValueError("playback_thumb_speed_scaling 必须是布尔值")
        self.max_step = int(config.get("max_step_per_frame", 30))
        self.tracking_tolerance = int(config.get("tracking_tolerance", 40))
        self.stall_frames = int(config.get("stall_frames", 4))
        self.stall_error = int(config.get("stall_error", 20))
        self.late_s = int(config.get("stall_ms", 100)) / 1000
        self.max_pause_s = int(config.get("max_pause_ms", 3000)) / 1000
        self.tolerance = int(config.get("position_tolerance", 6))
        self.approach_timeout = float(config.get("approach_timeout_s", 8.0))
        self.halfway_s = float(config.get("halfway_s", HALFWAY_S))
        self.poll = float(config.get("poll_interval_s", 0.1))
        if not (1 <= self.max_step <= 255 and 1 <= self.tracking_tolerance <= 255
                and 2 <= self.stall_frames <= 20
                and 1 <= self.stall_error <= 255 and 0.05 <= self.late_s <= 1
                and self.late_s < self.max_pause_s <= 10
                and 0 <= self.tolerance <= 20 and 0.5 <= self.approach_timeout <= 30
                and 0 <= self.halfway_s <= 2 and 0.02 <= self.poll <= 1):
            raise ValueError("示教回放的速度、跟踪或超时配置无效")

    @staticmethod
    def _setting(value, name):
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ValueError(f"{name} 必须是 1–255 范围内的整数")
        return value

    def _completion_timeout(self):
        durations = [entry.get("duration_s", 0) for entry in hand_trajectory.list_presets(self.preset_dir)]
        longest = max([self.record_max_s, *durations])
        return int(longest / min(PLAY_SPEEDS.values()) + 3 * self.approach_timeout + TRACKING_DELAY_BUDGET_S + SETTLE_S + 25)

    def get_tool(self) -> dict:
        schema = action_schema({
            "info": ([], "只读：绑定的手、检测到的手、轨迹数量和上一次结果"),
            "start_trajectory": (["name", "confirm_motion"],
                                 "开始录制本机手套示教；录制期间同步驱动卡片绑定的灵巧手"),
            "stop_trajectory": (["confirm_motion"],
                                "结束录制并保存；实时遥操会停止并尝试释放力矩"),
            "list_trajectories": ([], "只读：列出已录制轨迹及校验结果，不驱动手指"),
            "delete_trajectory": (["name", "confirm_motion"], "删除一条录制轨迹；内置预设和正在回放的轨迹不能删除"),
            "play": (["name", "speed", "approach_speed", "return_to_start", "hold_torque", "confirm_motion"],
                     "从当前关节位置按 20 Hz 直接跟踪录制轨迹；每帧限幅，持续停滞或超时时中断并尝试释放力矩"),
            "pause": (["confirm_motion"], "回放中沿轨迹减速暂停，保持暂停手型"),
            "resume": (["confirm_motion"], "确认暂停手型后，从保存的轨迹进度加速继续"),
            "stop_motion": ([], "立即停止回放或过渡运动并释放全部关节力矩，也会中止录制（录制标为 incomplete）；不需要确认"),
            "calibrate_glove": ([], "保持手指自然伸直，校准卡片绑定手侧的手套"),
            "glove_status": ([], "查看本机手套校准及有效帧状态"),
            "disconnect_glove": ([], "断开手套接收器，下次连接需重新校准"),
            "config": (["bound_side"], "配置手侧"),
        }, {
            "bound_side": {"type": "string", "enum": ["left", "right"]},
            "instance_id": {"type": "string"},
            "name": {"type": "string", "minLength": 1, "maxLength": 80,
                     "description": "轨迹名称，与 list_trajectories 返回的 name 完全一致；录制时为新轨迹名称，不能重名"},
            "speed": {"type": "string", "enum": list(PLAY_SPEEDS), "default": "0.25x",
                      "description": "回放倍速；新轨迹先用 0.25x"},
            "approach_speed": {"type": "integer", "minimum": 1, "maximum": 255, "default": self.approach_speed,
                               "description": f"回放结束后回到起点时使用的速度寄存器原始值，默认 {self.approach_speed}"},
            "return_to_start": {"type": "boolean", "default": False,
                                "description": "回放结束后回到轨迹起点"},
            "hold_torque": {"type": "boolean", "default": False,
                            "description": "结束后保留力矩；默认 false，到位后释放"},
            "confirm_motion": {"type": "boolean", "const": True,
                               "description": "确认手附近无人和障碍物后，本次请求显式传入布尔值 true"},
        })
        schema["allOf"] = [
            {"if": {"properties": {"action": {"enum": ["play", "start_trajectory", "delete_trajectory"]}}},
             "then": {"required": ["name", "confirm_motion"]}},
            {"if": {"properties": {"action": {"enum": ["stop_trajectory", "pause", "resume"]}}},
             "then": {"required": ["confirm_motion"]}},
        ]
        schema["x-action-params"]["calibrate_glove"]["before_run"] = (
            "戴好卡片所绑定手侧的手套，手腕自然放平，五指自然伸直并拢。拇指保持伸直，轻靠食指。"
            "摆好后点击执行，保持手势约 3 秒，再用 glove_status 确认状态为 ready。"
        )
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {"actions": list(ACTIONS), "timeout": self._completion_timeout()}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop_motion"}}
        definition = tool(self.PREFIX, "actuator",
                          "L20 Lite 手套示教卡片：录制本机手套数据，可边录边实时遥操或保存后回放", schema)
        definition["title"] = "L20 Lite 示教"
        definition["multiInstance"] = True
        definition["configSchema"] = {"type": "object", "properties": {
            "bound_side": {"type": "string", "enum": ["left", "right"], "scope": "instance",
                           "title": "绑定灵巧手", "description": "选择本卡片控制的左手或右手"},
        }}
        return definition

    def start(self):
        try:
            self.store.recover()
        except OSError as exc:
            print(f"[hand_teach] cannot check recordings in {self.data_dir}: {exc}", flush=True)

    def stop(self):
        with self._lock:
            self._stop.set()
            self._paused = None
        recorder = self._recorder
        if recorder is not None:
            recorder.stop("interrupted")
            recorder.join(5)
        try:
            self._glove.close()
        except RuntimeError as exc:
            print(f"[hand_teach] {exc}", flush=True)
        with self.hand._bus_lock:
            for connection in self._connections.values():
                if connection._client is not None:
                    connection._client.set_torques([0] * JOINT_COUNT)
                connection.close()

    def _connection(self, instance_id, side, port):
        with self._lock:
            connection = self._connections.get(instance_id)
            if connection is not None and (connection.port != port or connection.side != side):
                connection.close()
                connection = None
            if connection is None:
                connection = L20LiteConnection({**self.hand.config, "side": side, "serial_port": port})
                self._connections[instance_id] = connection
            return connection

    def _check_stop(self):
        if self._stop.is_set():
            raise InterruptedError("示教回放已停止")

    def _wait(self, seconds):
        if seconds > 0 and self._stop.wait(seconds):
            raise InterruptedError("示教回放已停止")

    def _io(self, call, *args):
        """One bus transaction; a stop that took the lock first wins over any new position."""
        with self.hand._bus_lock:
            self._check_stop()
            return call(*args)

    def _joint_speeds(self, base, *, scale_thumb=None):
        speeds = [base] * JOINT_COUNT
        if scale_thumb is None:
            scale_thumb = self.thumb_speed_scaling
        if scale_thumb:
            speeds[0] = max(1, round(base * 29.11 / 63.11))
            speeds[1] = max(1, round(base * 29.11 / 63.21))
            speeds[9] = max(1, round(base * 29.11 / 50.90))
        return speeds

    def _near(self, measured, target):
        return all(abs(a - b) <= self.tolerance for a, b in zip(measured, target))

    def _move(self, client, target, timeout=None):
        """hand._move's two-stage approach; fail closed if the pose is not reached."""
        current = self._io(client.get_positions)
        if self._near(current, target):
            return current
        self._io(client.set_positions, [(a + b) // 2 for a, b in zip(current, target)])
        self._wait(self.halfway_s)
        self._io(client.set_positions, target)
        deadline = time.monotonic() + (self.approach_timeout if timeout is None else timeout)
        while True:
            measured = self._io(client.get_positions)
            if self._near(measured, target):
                return measured
            if time.monotonic() >= deadline:
                raise TimeoutError(f"灵巧手未在规定时间内到达过渡手型，目标 {target}，实测 {measured}")
            self._wait(self.poll)

    @staticmethod
    def _target(points, elapsed_ms):
        index = int(elapsed_ms // hand_trajectory.SAMPLE_MS)
        if index >= len(points) - 1:
            return list(points[-1]["positions"])
        alpha = (elapsed_ms - index * hand_trajectory.SAMPLE_MS) / hand_trajectory.SAMPLE_MS
        a, b = points[index]["positions"], points[index + 1]["positions"]
        return [round(x + alpha * (y - x)) for x, y in zip(a, b)]

    def _stream(self, client, points, ratio, active):
        """Stream targets at the requested rate while checking hardware feedback."""
        frame_s = 1 / PLAYBACK_HZ
        max_step = self.max_step
        stall_s = self.stall_frames * frame_s
        end_ms = points[-1]["time_ms"]
        final = list(points[-1]["positions"])
        resume = active.get("resume")
        elapsed_ms = resume["position_ms"] if resume else 0.0
        rate = 0.0 if resume else ratio
        measured = self._io(client.get_positions)
        command = list(resume["command"]) if resume else list(measured)
        anchors = list(measured)
        stalled_since = [None] * JOINT_COUNT
        stats = {"playback_hz": PLAYBACK_HZ, "frames": 0, "max_tracking_error": 0, "max_late_ms": 0,
                 "late_frames": 0, "tracking_lag_frames": 0, "limited_frames": 0}
        started = time.monotonic()
        scheduled = started if resume else started - frame_s
        deadline = started + (end_ms - elapsed_ms) / (1000 * ratio) + TRACKING_DELAY_BUDGET_S
        last_sample = started
        lagging_joints = [0] * JOINT_COUNT
        peak_errors = [0] * JOINT_COUNT
        schedule_late_s = 0.0
        while True:
            scheduled += frame_s
            self._wait(scheduled - time.monotonic())
            with self.hand._bus_lock:
                self._check_stop()
                now = time.monotonic()
                late = now - scheduled
                schedule_late_s += max(0.0, late)
                stats["max_late_ms"] = max(stats["max_late_ms"], round(late * 1000))
                if late > self.max_pause_s:
                    raise RuntimeError(f"回放第 {stats['frames'] + 1} 帧延迟 {late * 1000:.0f} ms，RS485 总线阻塞")
                if now >= deadline:
                    raise TimeoutError(f"回放超出执行时限，已到轨迹 {elapsed_ms:.0f} ms 处")
                measured = client.get_positions()
                sampled = time.monotonic()
                first_frame = resume is None and stats["frames"] == 0
                errors = [abs(target - actual) for target, actual in zip(command, measured)]
                stats["max_tracking_error"] = max(stats["max_tracking_error"], max(errors))
                for joint, error in enumerate(errors):
                    peak_errors[joint] = max(peak_errors[joint], error)
                    if error <= self.stall_error or abs(measured[joint] - anchors[joint]) > 2:
                        anchors[joint] = measured[joint]
                        stalled_since[joint] = None
                    else:
                        if stalled_since[joint] is None:
                            stalled_since[joint] = sampled
                        if sampled - stalled_since[joint] >= stall_s:
                            raise RuntimeError(
                                f"关节 {hand_trajectory.JOINT_NAMES[joint]} 停滞，实测 {measured[joint]}，"
                                f"目标 {command[joint]}，请检查是否受阻"
                            )
                if late > self.late_s:
                    stats["late_frames"] += 1
                    print(f"[hand_teach] playback frame {stats['frames'] + 1} was "
                          f"{late * 1000:.0f} ms late; sampling the current trajectory time", flush=True)
                if max(errors) > self.tracking_tolerance:
                    stats["tracking_lag_frames"] += 1
                    for joint, error in enumerate(errors):
                        if error > self.tracking_tolerance:
                            lagging_joints[joint] += 1
                with self._lock:
                    pausing = "pause_id" in active
                    wanted_rate = 0.0 if pausing else ratio
                    interval = 0.0 if first_frame else sampled - last_sample
                    if rate == wanted_rate:
                        advance = interval * rate
                    else:
                        # Integrate the linear ramp, including any time after
                        # it reaches the requested rate.
                        ramp_duration = abs(wanted_rate - rate) * PAUSE_RAMP_S / ratio
                        ramp = min(interval, ramp_duration)
                        next_rate = wanted_rate if ramp == ramp_duration else rate + (wanted_rate - rate) * ramp / ramp_duration
                        advance = ramp * (rate + next_rate) / 2 + (interval - ramp) * wanted_rate
                        rate = next_rate
                    elapsed_ms = min(end_ms, elapsed_ms + advance * 1000)
                    last_sample = sampled
                    if pausing and rate <= 1e-9:
                        # Hold a validated on-path command. Verify it has
                        # settled before reporting that the hand is paused.
                        active["streaming"] = False
                        break
                    if not pausing and elapsed_ms >= end_ms and command == final:
                        active["streaming"] = False
                        break
                goal = self._target(points, elapsed_ms)
                broken = hand_trajectory.violations(goal)
                if broken:
                    raise RuntimeError(f"回放目标手型违反 {broken[0]}，目标值为 {goal}")
                # Start from feedback and approach each valid target at the same
                # step limit, including fixed axes that start away from neutral.
                command = [c + max(-max_step, min(max_step, g - c)) for c, g in zip(command, goal)]
                if command != goal:
                    stats["limited_frames"] += 1
                self._check_stop()
                client.set_positions(command)
                if first_frame:
                    self._check_stop()
                    client.set_torques([self.torque] * JOINT_COUNT)
                stats["frames"] += 1
            scheduled = max(scheduled, now)
        if pausing:
            deadline = time.monotonic() + SETTLE_S
            while not self._near(measured, command):
                if time.monotonic() >= deadline:
                    raise TimeoutError("暂停失败：机械手未能到达暂停手型")
                self._wait(self.poll)
                measured = self._io(client.get_positions)
            self._check_stop()
            raise PlaybackPaused(elapsed_ms, measured, command)

        stream_s = time.monotonic() - started
        stats.update(
            trajectory_s=round(end_ms / 1000, 3),
            nominal_play_s=round(end_ms / (1000 * ratio), 3),
            stream_s=round(stream_s, 3),
            actual_send_hz=round(stats["frames"] / max(stream_s, frame_s), 2),
            schedule_late_s=round(schedule_late_s, 3),
            tracking_lag_by_joint={name: count for name, count in _named(lagging_joints).items() if count},
            peak_error_by_joint=_named(peak_errors),
        )
        deadline = time.monotonic() + SETTLE_S
        while not self._near(measured, final):
            if time.monotonic() >= deadline:
                raise TimeoutError(f"灵巧手未在规定时间内到达轨迹终点，目标 {final}，实测 {measured}")
            self._wait(self.poll)
            measured = self._io(client.get_positions)
        stats["settle_s"] = round(time.monotonic() - (deadline - SETTLE_S), 3)
        stats["positions"] = _named(measured)
        return stats

    def _play(self, client, active):
        options, points = active["options"], active["samples"]["points"]
        started = time.monotonic()
        start_pose = list(points[0]["positions"])
        resume = active.get("resume")
        if resume:
            if not self._near(self._io(client.get_positions), resume["measured"]):
                raise RuntimeError("暂停后手型已变化，请重新执行 play")
        else:
            # Preload the first bounded command before restoring motor torque.
            self._io(client.set_torques, [0] * JOINT_COUNT)
        speeds = self._joint_speeds(self.motion_speed, scale_thumb=self.playback_thumb_speed_scaling)
        if self.playback_thumb_sway_speed is not None:
            speeds[hand_trajectory.THUMB_SWAY] = self.playback_thumb_sway_speed
        self._io(client.set_speeds, speeds)
        approach_s = time.monotonic() - started
        with self._lock:
            active["streaming"] = True
        stats = self._stream(client, points, options["ratio"], active)
        stats["approach_s"] = round(approach_s, 3)
        if options["return_to_start"]:
            self._io(client.set_speeds, self._joint_speeds(options["approach_speed"]))
            stats["positions"] = _named(self._move(client, start_pose))
        if not options["hold"]:
            self._io(client.set_torques, [0] * JOINT_COUNT)
        stats["total_play_s"] = round(time.monotonic() - started, 3)
        return stats

    def _execute(self, active):
        side, port = active["side"], active["port"]
        client = None
        torque_released = False
        try:
            with self.hand._bus_lock:
                self._check_stop()
                found = self.hand._discover()
                if found.get(side) != port:
                    raise RuntimeError(f"回放前灵巧手已更换串口或断开（{side}）")
                client = self._connection(active["instance_id"], side, port).get(side)
                client.probe_positions(side)
            try:
                stats = self._play(client, active)
            except PlaybackPaused:
                raise
            except Exception:
                with self.hand._bus_lock:
                    try:
                        client.set_torques([0] * JOINT_COUNT)
                        torque_released = True
                    except Exception as exc:
                        print(f"[hand_teach] {active['action_id']} torque release failed: {exc}", flush=True)
                raise
            options = active["options"]
            result = {"state": "completed", "action": active["action"], "side": side,
                      "name": active["manifest"]["name"], "speed": options["speed"],
                      "returned_to_start": options["return_to_start"],
                      "torque_released": not options["hold"], **stats}
            status = "completed"
        except PlaybackPaused as exc:
            status, result = "cancelled", {"state": "paused", "reason": "paused", "message": "回放已暂停",
                                           "name": active["manifest"]["name"],
                                           "position_s": round(exc.position_ms / 1000, 3),
                                           "torque_released": False}
            with self._lock:
                if not self._stop.is_set():
                    self._paused = {**active, "resume": {"position_ms": exc.position_ms, "measured": exc.measured, "command": exc.command}}
                else:
                    result.update(state="cancelled", reason="stopped", message="示教回放已停止")
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc), "torque_released": torque_released}
        except Exception as exc:
            status, result = "error", {"state": "error", "message": error_text(exc, "回放失败，请查看驱动日志"), "torque_released": torque_released}
        if status != "completed" and result.get("state") != "paused":
            # The canvas only shows the immediate "running" reply; keep the reason in docker logs too.
            print(f"[hand_teach] {active['action_id']} {status}: {result.get('message', result.get('reason'))}", flush=True)
        terminal = {"action_id": active["action_id"], "status": status, "result": result}
        with self._lock:
            self._last_result = terminal
            active["streaming"] = False
            pause_id = active.get("pause_id")
        try:
            self._acp_callback(active["action_id"], status, result)
            if pause_id:
                with self._lock:
                    paused = self._paused is not None
                self._acp_callback(pause_id, "completed" if paused else "error",
                                   result if paused else {"state": "error", "message": "回放未能暂停", "playback": result})
        except Exception as exc:
            with self._lock:
                self._delivery_failed = True
                terminal["callback_error"] = "回放结果未送达 Agent Core，请查看驱动日志"
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)
        else:
            with self._lock:
                if self._active is active:
                    self._active = None

    def _acp_callback(self, action_id, status, result):
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
        if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(("https://localhost:", "https://127.0.0.1:")):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": self.PREFIX, "ts": time.time()}, ensure_ascii=False).encode()
        for attempt in range(3):
            try:
                request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                                 headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=5, context=context) as response:
                    acknowledgement = json.load(response)
                if acknowledgement.get("ok") is not True or acknowledgement.get("action_id") != action_id:
                    raise RuntimeError("Agent Core 未确认动作回执")
                return
            except Exception:
                if attempt == 2:
                    raise
                time.sleep(0.5)

    def _prepare(self, args):
        if args.get("confirm_motion") is not True:
            raise ValueError("请勾选 confirm_motion 确认本次运动")
        speed = args.get("speed", "0.25x")
        if speed not in PLAY_SPEEDS:
            raise ValueError(f"回放倍速须为 {', '.join(PLAY_SPEEDS)} 之一")
        approach = self._setting(args.get("approach_speed", self.approach_speed), "approach_speed")
        options = {"speed": speed, "ratio": PLAY_SPEEDS[speed], "approach_speed": approach}
        for key, default in (("return_to_start", False), ("hold_torque", False)):
            value = args.get(key, default)
            if not isinstance(value, bool):
                raise ValueError(f"{key} 必须是布尔值")
            options["hold" if key == "hold_torque" else key] = value
        name = hand_trajectory.normalize_name(args.get("name"))
        directory = hand_trajectory.find_preset(name, self.preset_dir) or self.store.find(name)
        if directory is None:
            raise ValueError(f"找不到轨迹「{name}」，请先执行 list_trajectories 检查名称")
        try:
            manifest, samples = hand_trajectory.load_trajectory(directory)
        except (OSError, ValueError, RuntimeError) as exc:
            raise ValueError(f"轨迹「{name}」无法回放：" + error_text(exc, "轨迹文件无法读取，请查看驱动日志")) from exc
        return options, manifest, samples

    def dispatch(self, action, args):
        try:
            return self._dispatch(action, args)
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            message = error_text(exc, "示教操作失败，请查看驱动日志")
            if message == str(exc):
                raise
            raise RuntimeError(message) from exc

    def _dispatch(self, action, args):
        instance_id = args.get("instance_id") or ""
        if action == "glove_status":
            return {**self._glove.status(self._bindings.get(instance_id) or "right"),
                    "configured": all(self._glove_settings.values())}
        if action in ("calibrate_glove", "disconnect_glove"):
            return self._glove_action(action, instance_id)
        if action == "config":
            if not isinstance(instance_id, str) or not instance_id:
                raise ValueError("缺少卡片实例标识 instance_id")
            with self._lock:
                previous_side = self._bindings.get(instance_id)
                side = args.get("bound_side", previous_side)
                if "bound_side" in args and side not in ("left", "right"):
                    raise ValueError("手侧只能选左手或右手")
                changed = side != previous_side
                if changed and (self._active is not None or self._paused is not None or self._live_recording()):
                    raise RuntimeError("示教运动期间无法修改卡片配置")
                if side is not None:
                    if any(owner != instance_id and bound == side for owner, bound in self._bindings.items()):
                        raise ValueError(f"该手侧已绑定到另一张示教卡片（{side}）")
                    if instance_id == self._glove_owner and side != (previous_side or "right"):
                        self._glove.close()
                        self._glove_owner = None
                    self._bindings[instance_id] = side
            return {"state": "configured", "instance_id": instance_id, "bound_side": side}
        if action in ("stop", "stop_motion"):
            with self._lock:
                active = self._active or self._paused
                if (action == "stop" and instance_id and active is not None
                        and active["instance_id"] != instance_id):
                    # Lifecycle stop of another card leaves this playback alone.
                    return {"state": "idle", "torque_released": False}
                recorder = self._recorder
                live = self._live_recording()
                if (action == "stop" and instance_id and active is None and recorder is not None
                        and instance_id != self._recording_owner):
                    return {"state": "idle", "torque_released": False}
                self._stop.set()
                self._paused = None
                if recorder is not None and not recorder.finished.is_set():
                    recorder.stop("interrupted")

            if live:
                result = recorder.join(5)
                if result is None:
                    raise RuntimeError("遥操已请求停止，尚未确认力矩释放")
                if recorder.torque_released is False:
                    raise RuntimeError(recorder.live_error)
                return {"state": "idle", "torque_released": recorder.torque_released}
            owners = {instance_id, active["instance_id"] if active else instance_id}
            with self.hand._bus_lock:
                for owner in owners:
                    connection = self._connections.get(owner)
                    if connection is not None and connection._client is not None:
                        connection._client.set_torques([0] * JOINT_COUNT)
            return {"state": "idle", "stopped_action_id": active["action_id"] if active else None,
                    "torque_released": True}
        if action == "list_trajectories":
            # Short entries: the activity panel only shows the start of a long result.
            keys = ("name", "valid", "state", "reason", "error", "source", "duration_s", "point_count")
            recorded = self.store.list()
            return {"recorded_count": len(recorded),
                    "recorded_names": [entry["name"] for entry in recorded],
                    "trajectories": [{key: entry[key] for key in keys if key in entry}
                                     for entry in recorded]}
        if action == "start_trajectory":
            return self._start_recording(instance_id, args)
        if action == "stop_trajectory":
            return self._stop_recording(instance_id, args)
        if action == "delete_trajectory":
            return self._delete(args)
        if action == "info":
            with self._lock:
                recording_id = args.get("recording_id")
                if isinstance(recording_id, str) and recording_id:
                    recorder = self._recorder
                    if recorder is not None and recorder.recording_id == recording_id:
                        return self._recording_state(recorder)
                    result = self._recording_results.get(recording_id)
                    return {"state": "ended", "result": result} if result else {"state": "unknown"}
                playing = self._active
                live = self._recorder if self._live_recording() else None
            # Discovery holds the bus ~0.5 s per absent hand; never stall our own playback with it.
            if playing:
                found = {playing["side"]: playing["port"]}
            elif live:
                found = {live.side: live.live_port} if live.live_port else {}
            else:
                found = self.hand._discover()
            side = self._bindings.get(instance_id)
            with self._lock:
                active_id = self._active["action_id"] if self._active else None
                last_result = self._last_result
                recorder = self._recorder
            return {"state": "connected" if side in found else "unbound" if side is None else "offline",
                    "bound_side": side, "detected_sides": list(found), "detected_count": len(found),
                    "serial_port": found.get(side),
                    "recorded_count": len(self.store.list()), "data_dir": str(self.data_dir),
                    "recording": self._recording_state(recorder), "active_action_id": active_id,
                    "last_result": last_result, "glove": self._glove.status()}
        if action == "start":
            return {"state": "ready" if self._bindings.get(instance_id) else "unbound",
                    "bound_side": self._bindings.get(instance_id)}
        if action in ("pause", "resume"):
            if args.get("confirm_motion") is not True:
                raise ValueError("请勾选 confirm_motion 确认操作")
            with self._lock:
                if self._delivery_failed:
                    raise RuntimeError("上一次动作结果未送达 Agent Core，请检查后重启驱动")
                if action == "pause":
                    active = self._active
                    if not active or not active.get("streaming") or "pause_id" in active:
                        raise RuntimeError("当前没有可暂停的轨迹回放")
                    if instance_id and active["instance_id"] != instance_id:
                        raise ValueError("请在启动回放的卡片上操作")
                    active["pause_id"] = f"hand_teach_pause_{uuid4().hex}"
                    return {"state": "running", "action_id": active["pause_id"]}
                paused = self._paused
                if self._active is not None or paused is None:
                    raise RuntimeError("当前没有可继续的暂停轨迹")
                if instance_id and paused["instance_id"] != instance_id:
                    raise ValueError("请在启动回放的卡片上操作")
                active = {**paused, "action": "resume", "action_id": f"hand_teach_resume_{uuid4().hex}"}
                active.pop("pause_id")
                result = self._start_playback(active)
                self._paused = None
                return result
        if action != "play":
            return None
        return self._request_play(instance_id, args)

    def _check_motion_instance(self, instance_id):
        if not instance_id and self._bindings:
            raise ValueError("缺少卡片实例标识，无法应用已配置的手侧，请刷新画布后重试")

    def _glove_action(self, action, instance_id):
        with self._lock:
            if self._recorder is not None and not self._recorder.finished.is_set():
                raise RuntimeError("请先停止轨迹录制")
            if self._glove_owner not in (None, instance_id):
                if self._glove.status()["state"] in ("idle", "error"):
                    self._glove_owner = None
                else:
                    raise RuntimeError("手套接收器正在由另一张卡片使用")
            if action == "calibrate_glove":
                self._check_motion_instance(instance_id)
                side = self._bindings.get(instance_id) or "right"
                settings = self._glove_settings
                if not all(settings.values()):
                    raise RuntimeError("请在驱动中配置手套 SDK 库、接收器串口和 .hexprofile")
                if self._glove_owner == instance_id:
                    self._glove.close()
                state = self._glove.calibrate(settings["library"], settings["serial"], settings["profile"], (side,))
                self._glove_owner = instance_id
                hand_name = "左手" if side == "left" else "右手"
                return {**state, "side": side,
                        "message": f"正在校准{hand_name}手套。请保持当前伸直手势，约 3 秒后用 glove_status 确认状态为 ready。"}
            self._glove.close()
            self._glove_owner = None
            return self._glove.status()

    def _request_play(self, instance_id, args):
        action = "play"
        options, manifest, samples = self._prepare(args)  # Validate before opening serial.
        with self._lock:
            self._check_motion_instance(instance_id)
            side = self._bindings.get(instance_id)
        found = self.hand._discover()
        if side is None:
            if len(found) != 1:
                raise RuntimeError("无法自动确定手侧，请检查连接，并在卡片配置中选择左手或右手")
            side = next(iter(found))
        if side != manifest["side"]:
            raise RuntimeError(f"轨迹「{manifest['name']}」的手侧是 {manifest['side']}，与当前卡片不匹配")
        port = found.get(side)
        if port is None:
            raise RuntimeError(f"该手侧的灵巧手未连接（{side}）")
        active = {"action": action, "action_id": f"hand_teach_{action}_{uuid4().hex}",
                  "instance_id": instance_id, "side": side, "port": port, "options": options,
                  "manifest": manifest, "samples": samples}
        with self._lock:
            if self._delivery_failed:
                raise RuntimeError("上一次动作结果未送达 Agent Core，请检查后重启驱动")
            if self._active is not None or self._paused is not None:
                raise RuntimeError("已有示教回放正在运行或暂停")
            if self._recorder is not None and not self._recorder.finished.is_set():
                raise RuntimeError("正在录制，请先执行 stop_trajectory")
            return self._start_playback(active)

    def _start_playback(self, active):
        self._stop.clear()
        self._active = active
        try:
            threading.Thread(target=self._execute, args=(active,), name=active["action_id"], daemon=True).start()
        except Exception:
            self._active = None
            raise
        return {"state": "running", "action_id": active["action_id"]}

    def _run_live(self, recorder, instance_id):
        client = None
        torque_attempted = False
        period = 1 / PLAYBACK_HZ
        try:
            # No serial access before a valid UDP frame.
            recorder.startup.wait()
            if recorder.stopped.is_set() or not recorder.first_frame.is_set():
                return
            with self.hand._bus_lock:
                if recorder.stopped.is_set():
                    return
                side = recorder.side
                port = self.hand._discover().get(side)
                if port is None:
                    raise RuntimeError(f"未连接{'左侧' if side == 'left' else '右侧'} L20 Lite 灵巧手")
                recorder.live_port = port
                client = self._connection(instance_id, side, port).get(side)
                command = list(client.probe_positions(side))
                anchors = list(command)
                stalled_since = [None] * JOINT_COUNT
                # The glove doesn't drive these axes. Start near their calibrated positions.
                fixed = (*hand_trajectory.FINGER_SWAYS, hand_trajectory.THUMB_ROLL)
                goal = self._live_goal(recorder)
                if goal is None:
                    return
                if any(abs(command[j] - goal[j]) > self.tolerance for j in fixed):
                    raise RuntimeError("请先用 hand 卡片将灵巧手张开，再启动实时遥操")
                speeds = self._joint_speeds(self.motion_speed, scale_thumb=self.playback_thumb_speed_scaling)
                if self.playback_thumb_sway_speed is not None:
                    speeds[hand_trajectory.THUMB_SWAY] = self.playback_thumb_sway_speed
                client.set_speeds(speeds)
            while not recorder.stopped.is_set():
                tick = time.monotonic()
                with self.hand._bus_lock:
                    goal = self._live_goal(recorder)
                    if goal is None:
                        break
                    measured = client.get_positions()
                    now = time.monotonic()
                    for j, position in enumerate(measured):
                        if abs(command[j] - position) <= self.stall_error or abs(anchors[j] - position) > 2:
                            anchors[j], stalled_since[j] = position, None
                        elif stalled_since[j] is None:
                            stalled_since[j] = now
                        elif now - stalled_since[j] >= self.stall_frames * period:
                            raise RuntimeError(f"关节 {hand_trajectory.JOINT_NAMES[j]} 停滞，已停止遥操")
                    goal = self._live_goal(recorder)
                    if goal is None:
                        break
                    command = [c + max(-self.max_step, min(self.max_step, g - c)) for c, g in zip(command, goal)]
                    for j in fixed:
                        command[j] = goal[j]
                    broken = hand_trajectory.violations(command)
                    if broken:
                        raise RuntimeError(f"遥操过渡手型违反 {broken[0]}，已停止遥操")
                    if not torque_attempted:
                        # Preload a checked target with torque off, so enabling
                        # the motor cannot execute a stale target from another card.
                        client.set_torques([0] * JOINT_COUNT)
                        if recorder.stopped.is_set():
                            break
                        client.set_positions(command)
                        if self._live_goal(recorder) is None:
                            break
                        torque_attempted = True
                        client.set_torques([self.torque] * JOINT_COUNT)
                    else:
                        client.set_positions(command)
                    recorder.live_frames += 1
                    recorder.live_ready.set()
                recorder.stopped.wait(max(0, tick + period - time.monotonic()))
        except Exception as exc:
            recorder.fail_live(error_text(exc, "实时遥操失败，请查看驱动日志"))
        finally:
            try:
                if torque_attempted:
                    with self.hand._bus_lock:
                        client.set_torques([0] * JOINT_COUNT)
                    recorder.torque_released = True
            except Exception as exc:
                recorder.torque_released = False
                reason = f"{recorder.live_error}；" if recorder.live_error else ""
                recorder.fail_live(f"{reason}释放力矩失败：" + error_text(exc, "请检查灵巧手连接并查看驱动日志"))
            finally:
                self.hand.release_live_teach()

    @staticmethod
    def _live_goal(recorder):
        if recorder.stopped.is_set():
            return None
        received, values = recorder.latest
        if time.monotonic() - received > recorder.gap_s:
            recorder.stop("gap")
            return None
        goal = hand_trajectory.glove_to_driver(values, local_glove=recorder.source == "local_glove")
        broken = hand_trajectory.violations(goal)
        if broken:
            raise RuntimeError(f"手套目标手型违反 {broken[0]}，已停止遥操")
        return goal

    def _live_recording(self):
        recorder = self._recorder
        return recorder is not None and recorder.live_control is not None and not recorder.finished.is_set()

    # UDP capture and RS-485 streaming run independently. The motor loop never queues old targets.

    def _recording_state(self, recorder):
        if recorder is None:
            return None
        if recorder.finished.is_set():
            return {"state": "ended", "name": recorder.name, "result": recorder.result}
        return {"state": "recording", "name": recorder.name, "frames": len(recorder.frames),
                "sender": recorder.sender,
                "live_teleop": recorder.live_control is not None}

    def _on_recording_end(self, recorder):
        result = recorder.result
        if not result.get("valid"):
            print(f"[hand_teach] recording {recorder.name} ended: {recorder.end_reason} -> {result}", flush=True)
        with self._lock:
            self._recording_results[recorder.recording_id] = result
            if len(self._recording_results) > 16:
                self._recording_results.pop(next(iter(self._recording_results)))
            if (self._recorder is recorder or self._recorder is None) and self._active is None:
                self._last_result = {"action": "record", "status": result.get("state"), "result": result}

    def _start_recording(self, instance_id, args):
        if args.get("confirm_motion") is not True:
            raise ValueError("请勾选 confirm_motion 确认录制及实时遥操")
        name = hand_trajectory.normalize_name(args.get("name"))
        if hand_trajectory.find_preset(name, self.preset_dir) is not None:
            raise ValueError(f"「{name}」是内置轨迹名称，请换名")
        with self._lock:
            self._check_motion_instance(instance_id)
            if self._active is not None or self._paused is not None:
                raise RuntimeError("回放运行或暂停期间无法开始录制")
            if self._recorder is not None and not self._recorder.finished.is_set():
                raise RuntimeError(f"正在录制「{self._recorder.name}」，请先执行 stop_trajectory")
            side = self._bindings.get(instance_id) or "right"
            if not all(self._glove_settings.values()):
                raise RuntimeError("请在驱动中配置手套 SDK 库、接收器串口和 .hexprofile")
            status = self._glove.status(side)
            if side not in status["calibrated_sides"]:
                raise RuntimeError("请对当前手侧重新执行 calibrate_glove，等待 glove_status 显示 ready")
            self.hand.reserve_live_teach()
            try:
                directory = self.store.create(name, hand_trajectory.recorded_at(time.time()), side=side)
                recorder = teach_recorder.Recorder(
                    directory, name, port=None, max_s=self.record_max_s, gap_ms=self.record_gap_ms,
                    on_end=self._on_recording_end, side=side,
                    local_packet=lambda: self._glove.latest_packet(side),
                    live_control=lambda take: self._run_live(take, instance_id),
                )
                try:
                    recorder.start()
                except Exception:
                    shutil.rmtree(directory, ignore_errors=True)
                    raise
            except Exception as exc:
                self.hand.release_live_teach()
                if isinstance(exc, OSError):
                    raise RuntimeError(f"无法启动录制：" + error_text(exc, "无法打开数据源或创建轨迹目录，请查看驱动日志")) from None
                raise
            self._recorder, self._recording_owner = recorder, instance_id
        recorder.startup.wait(self.first_frame_s)
        if recorder.first_frame.is_set() and not recorder.finished.is_set():
            if recorder.live_ready.wait(5) and not recorder.stopped.is_set():
                return {"state": "recording", "recording_id": recorder.recording_id, "name": name,
                        "sender": recorder.sender, "side": side,
                        "max_s": self.record_max_s, "live_teleop": True,
                        "message": "正在录制并实时遥操，达到时长上限会自动保存，也可调用 stop_trajectory 提前保存"}
            if not recorder.stopped.is_set():
                recorder.fail_live("灵巧手遥操启动超时")
        else:
            recorder.stop("no_frames")
        result = recorder.join(5)
        with self._lock:
            if result is not None and self._recorder is recorder:
                self._recorder = None
        if result is None:
            raise RuntimeError("本机手套录制启动失败，录制线程尚未退出")
        if result.get("valid"):
            return result
        if result["state"] == "error" or result.get("deleted") is False or recorder.first_frame.is_set():
            raise RuntimeError(result["message"])
        raise RuntimeError("未收到本机手套有效帧，请检查 glove_status")

    def _stop_recording(self, instance_id, args):
        if args.get("confirm_motion") is not True:
            raise ValueError("请勾选 confirm_motion 确认停止并保存录制")
        with self._lock:
            recorder = self._recorder
            if recorder is None:
                raise RuntimeError("当前没有可停止或保存的录制")
            if self._live_recording() and instance_id and instance_id != self._recording_owner:
                raise ValueError("请在开始遥操的卡片上停止录制")
            recorder.stop("stop_trajectory")
        result = recorder.join(10)
        if result is None:
            raise RuntimeError("等待录制保存超过 10 秒，请查看轨迹列表和驱动日志")
        with self._lock:
            if self._recorder is recorder:
                self._recorder = None
        if not result.get("valid"):
            raise RuntimeError(result["message"])
        return result

    def _delete(self, args):
        if args.get("confirm_motion") is not True:
            raise ValueError("请勾选 confirm_motion 确认删除轨迹")
        name = hand_trajectory.normalize_name(args.get("name"))
        if hand_trajectory.find_preset(name, self.preset_dir) is not None:
            raise ValueError(f"「{name}」是内置轨迹，无法删除")
        with self._lock:
            active = self._active or self._paused
            if active is not None and active["manifest"]["name"] == name:
                raise RuntimeError(f"「{name}」正在回放或暂停，请先停止动作")
            recorder = self._recorder
            if recorder is not None and recorder.name == name and not recorder.finished.is_set():
                raise RuntimeError(f"正在录制「{name}」，请先执行 stop_trajectory")
            self.store.delete(name)
        return {"state": "deleted", "name": name}
