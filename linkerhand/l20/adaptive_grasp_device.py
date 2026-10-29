"""LinkerHand L20 adaptive contact-stop grasp card (hand_grasp).

Features:
1. adaptive_grasp:
   - Moves from open_pose to calibrated prep_pose.
   - Progressively closes fingers towards close_limit_pose.
   - Monitors 12x6 fingertip matrix tactile peaks: when peak >= threshold,
     stops position advance on that finger and maintains holding torque.
   - Other fingers continue closing until tactile contact or close limit is reached.
   - Reads fresh actual positions during closing and freezes individual joints
     whose closing error persists while their measured position barely changes.
   - Advances faster while positions track, reverting to smaller steps per joint
     when actual positions lag. Existing targets are never rebased to actual.
   - Reports complete matrix samples, latched contacts and the actual stop reason.
   - Holds the commanded positions after all fingers settle or the test times out.
   - Stops and unloads on missing/invalid matrix data; scalar data is never used.
   - Holds targets during bounded re-acquisition after transient matrix timeouts.
2. release:
   - Returns fingers synchronously to prep_pose to free grasped object.
   - Safely returns to open_pose with clearance, then unloads motor torque.
3. get_state:
   - Reads progress and the last completion without moving or querying hardware.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from can_client import FINGER_NAMES, validate_position_vector, vector20_to_named16
from hand_device import (
    HandPlugin,
    SPEED_RATIOS,
    _SingleHandController,
    _setting,
    claim_motion,
    ensure_motion_available,
    release_motion,
    validate_bound_side,
    verify_hand_connection,
)

logger = logging.getLogger("linkerhand.adaptive_grasp_device")

DEFAULT_PREP_POSE = [
    255, 255, 255, 255, 255,  70, 200, 132,  55,   0,  73, 255, 255, 255, 255, 255, 255, 255, 255, 255
]

DEFAULT_CLOSE_LIMIT_POSE = [
     60,   0,   0,   0,   0,  70, 200, 132,  55,   0,  73, 255, 255, 255, 255, 142,   0,   0,   0,   0
]

FINGER_JOINTS = {
    "thumb":  {"root": 0, "tip": 15},
    "index":  {"root": 1, "tip": 16},
    "middle": {"root": 2, "tip": 17},
    "ring":   {"root": 3, "tip": 18},
    "pinky":  {"root": 4, "tip": 19},
}

TEST_ACTIONS = ("adaptive_grasp", "release")


class TestGraspFault(RuntimeError):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.details = details or {}


class _JointStallDetector:
    """Observe closing lag over time, using the full window's position spread."""

    def __init__(self, error_threshold: int, position_tolerance: int, duration_s: float):
        self.error_threshold = error_threshold
        self.position_tolerance = position_tolerance
        self.duration_s = duration_s
        self.windows: dict[int, dict] = {}

    def update(self, joint: int, command: int, actual: int, sampled_at: float,
               closing: bool) -> bool:
        # Smaller targets close these joints. Opposite-direction error, a
        # preparation pose or a recovered tracking error cannot prove blocking.
        if not closing or actual - command <= self.error_threshold:
            self.windows.pop(joint, None)
            return False
        window = self.windows.get(joint)
        if (window is None or max(window['maximum'], actual) - min(window['minimum'], actual)
                > self.position_tolerance):
            window = {'since': sampled_at, 'minimum': actual, 'maximum': actual, 'samples': 1}
            self.windows[joint] = window
        else:
            window['minimum'] = min(window['minimum'], actual)
            window['maximum'] = max(window['maximum'], actual)
            window['samples'] += 1
        return window['samples'] >= 3 and sampled_at - window['since'] >= self.duration_s


class AdaptiveGraspPlugin:
    PREFIX = "hand_grasp"
    __test__ = False

    def __init__(self, hand: HandPlugin | None, config: dict, global_config: dict | None = None):
        self.hand = hand
        self.config = config or {}
        self.global_config = global_config or (hand.config if hand else {})
        self._lock = threading.RLock()
        self._controllers: dict[tuple[str, str], _SingleHandController] = {}
        self._active: dict[tuple[str, str], dict] = {}
        self._sessions: dict[tuple[str, str], dict] = {}
        self._states: dict[tuple[str, str], str] = {}
        self._results: dict[tuple[str, str], dict] = {}
        self._stop = threading.Event()
        self._delivery_failed = False
        self._last_result: dict | None = None
        self._worker_context = threading.local()

        # Configuration parameters
        self.approach_speed = int(self.config.get("approach_speed", 85))
        self.release_speed = int(self.config.get("release_speed", 80))
        self.motion_current = _setting(self.config.get("motion_current", 230), "motion_current")
        self.tactile_threshold = _setting(self.config.get("tactile_threshold", 15), "tactile_threshold")
        self.step_size = int(self.config.get("step_size", 5))
        self.fast_step_size = self.config.get("fast_step_size", max(10, self.step_size))
        self.slowdown_error_threshold = self.config.get("slowdown_error_threshold", 10)
        if (isinstance(self.fast_step_size, bool) or not isinstance(self.fast_step_size, int)
                or not 1 <= self.step_size <= self.fast_step_size <= 255):
            raise ValueError("fast_step_size 必须为不小于 step_size 的1～255整数，step_size 必须为正数")
        if (isinstance(self.slowdown_error_threshold, bool)
                or not isinstance(self.slowdown_error_threshold, int)
                or not 1 <= self.slowdown_error_threshold < 255):
            raise ValueError("slowdown_error_threshold 必须为1～254之间的整数")
        self.poll_interval_s = float(self.config.get("poll_interval_s", 0.05))
        self.matrix_timeout_s = float(self.config.get("matrix_timeout_s", 0.25))
        self.matrix_retry_count = self.config.get("matrix_retry_count", 2)
        self.matrix_recovery_timeout_s = self.config.get("matrix_recovery_timeout_s", 1.0)
        if (isinstance(self.matrix_retry_count, bool) or not isinstance(self.matrix_retry_count, int)
                or self.matrix_retry_count < 0):
            raise ValueError("matrix_retry_count 必须为非负整数")
        if (isinstance(self.matrix_recovery_timeout_s, bool)
                or not isinstance(self.matrix_recovery_timeout_s, (int, float))
                or not math.isfinite(self.matrix_recovery_timeout_s) or self.matrix_recovery_timeout_s <= 0):
            raise ValueError("matrix_recovery_timeout_s 必须为有限正数")
        self.pose_timeout_s = float(self.config.get("pose_timeout_s", 5.0))
        self.timeout_s = float(self.config.get("timeout_s", 15.0))
        self.position_tolerance = int(self.config.get("position_tolerance", 20))

        # Trial defaults, independently configurable under plugins.hand_grasp.
        # These classify position lag, not physical contact or calibrated force.
        self.stall_error_threshold = self.config.get("stall_error_threshold", 20)
        self.stall_position_tolerance = self.config.get("stall_position_tolerance", 3)
        self.stall_duration_s = self.config.get("stall_duration_s", 0.5)
        for name in ("stall_error_threshold", "stall_position_tolerance"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 255:
                raise ValueError(f"{name} 必须为0～254之间的整数")
        if (isinstance(self.stall_duration_s, bool) or not isinstance(self.stall_duration_s, (int, float))
                or not math.isfinite(self.stall_duration_s) or self.stall_duration_s <= 0):
            raise ValueError("stall_duration_s 必须为有限正数")

        raw_prep = self.config.get("prep_pose", DEFAULT_PREP_POSE)
        self.prep_pose = validate_position_vector(raw_prep, "prep_pose")
        raw_limit = self.config.get("close_limit_pose", DEFAULT_CLOSE_LIMIT_POSE)
        self.close_limit_pose = validate_position_vector(raw_limit, "close_limit_pose")

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "adaptive_grasp": (
                    ["bound_side", "speed_ratio", "confirm_motion"],
                    "自适应抓取：触觉达阈停整指，位置持续受阻停对应关节，保持至 release；这是接触停止，不是恒压控制",
                ),
                "release": (
                    ["bound_side", "confirm_motion"],
                    "五指同步恢复准备姿态放开物体，再完全张开并卸力",
                ),
                "get_state": (
                    ["bound_side"],
                    "读取本次动作进度、任务编号和最终结果；不移动、不触发新抓取",
                ),
            },
            {
                "bound_side": {
                    "type": "string",
                    "enum": ["", "left", "right"],
                    "default": "",
                    "description": "本次操作的灵巧手，必须选择左手或右手",
                },
                "instance_id": {"type": "string"},
                "speed_ratio": {
                    "type": "string", "enum": list(SPEED_RATIOS), "default": "1.0x",
                    "description": "抓取电机速度：1.0x=85，1.5x=128，2.0x=170，2.5x=212，3.0x=255；不改变触觉阈值或位置步长",
                },
                "confirm_motion": {
                    "type": "boolean",
                    "const": True,
                    "description": "确认周围安全后传入 true",
                },
                "wait_for_result": {
                    "type": "boolean",
                    "const": False,
                    "default": False,
                    "description": "仅兼容 false；动作立即返回 action_id，最终结果通过 ACP 通知",
                },
            },
        )
        schema["allOf"] = [
            {
                "if": {"properties": {"action": {"enum": list(TEST_ACTIONS)}}},
                "then": {
                    "required": ["bound_side", "confirm_motion"],
                    "properties": {"bound_side": {"enum": ["left", "right"]}},
                },
            },
            {
                "if": {"properties": {"action": {"const": "get_state"}}},
                "then": {"required": ["bound_side"],
                         "properties": {"bound_side": {"enum": ["left", "right"]}}},
            },
        ]
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {
            "actions": list(TEST_ACTIONS),
            "timeout": max(30, math.ceil(4 * self.pose_timeout_s + self.timeout_s
                                         + self.matrix_recovery_timeout_s + 2)),
        }

        definition = tool(
            self.PREFIX,
            "actuator",
            "LinkerHand L20 自适应接触停止抓取、保持、释放与状态读取；使用五指12×6阵列峰值和实际位置反馈",
            schema,
        )
        definition["title"] = "L20 自适应抓取"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "motion_current": {
                    "type": "integer", "minimum": 1, "maximum": 255,
                    "scope": "shared", "default": self.motion_current,
                    "title": "电机输出限值（力矩档位）",
                    "description": "1～255 的原始输出限值，默认230；与厂家GUI的力矩设置对应，不是实测电流或牛顿值。接触停止后仍保留此限值和原位置目标，直到释放或停止。本卡片所有实例共用；调整前先释放。",
                },
                "tactile_threshold": {
                    "type": "integer", "minimum": 1, "maximum": 255,
                    "scope": "shared", "default": self.tactile_threshold,
                    "title": "高级：接触停止阈值",
                    "description": "1～255 的12×6阵列最大原始读数，默认15；某指达到阈值后，锁存该指根部和尖部的位置目标，不再进给。不是目标压力或力值上限，停止后读数仍可能上升。本卡片所有实例共用；调整前先释放。",
                },
            },
        }
        schema["properties"]["action"]["enum"].append("config")
        schema["x-action-params"]["config"] = {
            "params": ["motion_current", "tactile_threshold"],
            "description": "设定本卡片共享的电机输出限值和接触停止阈值；不修改其他卡片",
        }
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            for active in self._active.values():
                active["stop_event"].set()
            controllers = list(self._controllers.items())
            self._sessions.clear()
            for key, _ in controllers:
                self._states[key] = "stopping" if key in self._active else "idle"
        for device_key, ctrl in controllers:
            self._unload_owned(ctrl, (id(self), device_key))

    def _get_client_and_ctrl(self, side: str, channel: str | None = None):
        side = validate_bound_side(side)
        channel = channel or self.global_config.get("can_channel", "can0")
        if self.hand is not None:
            if isinstance(self.hand, HandPlugin):
                ctrl = self.hand.get_controller_for_side(side, channel)
            elif hasattr(self.hand, "get_controller_for_side"):
                ctrl = self.hand.get_controller_for_side(side, channel)
            else:
                raise RuntimeError("Invalid hand reference")
        else:
            from hand_device import get_shared_controller

            hand_config = self.global_config.get("plugins", {}).get("hand", {})
            ctrl = get_shared_controller(
                {**self.global_config, **hand_config, **self.config}, side, channel
            )

        with self._lock:
            self._controllers[(side, channel)] = ctrl
        return ctrl.get(), side, ctrl

    def _check_stop(self) -> None:
        event = getattr(self._worker_context, "stop_event", self._stop)
        if self._stop.is_set() or event.is_set():
            raise InterruptedError("motion stopped")

    def _wait(self, seconds: float) -> None:
        event = getattr(self._worker_context, "stop_event", self._stop)
        if event.wait(seconds):
            raise InterruptedError("motion stopped")
        self._check_stop()

    def _move_pose(self, client, ctrl, target: list[int], phase: str) -> list[int]:
        self._check_stop()
        with ctrl._motion_lock:
            self._check_stop()
            client.set_joint_positions(target)
        deadline = time.monotonic() + self.pose_timeout_s
        indices = [i for i in range(20) if i not in (11, 12, 13, 14)]
        while True:
            self._check_stop()
            with ctrl._motion_lock:
                measured = client.get_positions_vector()
            if all(abs(measured[i] - target[i]) <= self.position_tolerance for i in indices):
                return measured
            if time.monotonic() >= deadline:
                raise TestGraspFault("pose_timeout", f"{phase}未到位，已停止")
            self._wait(self.poll_interval_s)

    def _open_hand(self, client, ctrl) -> list[int]:
        with ctrl._motion_lock:
            current = client.get_positions_vector()
        target = validate_position_vector(ctrl._open_pose, "open_pose")
        # 1. 释放拇指指根与指尖
        free_tip = list(current)
        free_tip[0] = free_tip[15] = 255
        self._move_pose(client, ctrl, free_tip, "释放拇指指根与指尖")
        # 2. 拇指让位
        free_thumb = list(current)
        for i in (0, 5, 10, 15):
            free_thumb[i] = target[i]
        self._move_pose(client, ctrl, free_thumb, "拇指让位")
        # 3. 四指张开
        return self._move_pose(client, ctrl, target, "四指张开")

    @staticmethod
    def _matrix_peaks(matrix) -> dict[str, int]:
        """Reject missing/malformed taxels instead of treating them as zero."""
        if not isinstance(matrix, dict):
            raise TestGraspFault("invalid_tactile_matrix", "触觉查询未返回五指阵列")
        peaks = {}
        for finger in FINGER_NAMES:
            rows = matrix.get(finger)
            if (not isinstance(rows, list) or len(rows) != 12
                    or any(not isinstance(row, list) or len(row) != 6 for row in rows)
                    or any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255
                           for row in rows for value in row)):
                raise TestGraspFault("invalid_tactile_matrix", f"{finger} 本轮12×6触觉阵列缺失或无效",
                                     {"invalid_finger": finger})
            peaks[finger] = max(value for row in rows for value in row)
        return peaks

    def _set_tactile_status(self, *, report=True, **changes) -> dict:
        active = getattr(self._worker_context, "active", None)
        if active is None:
            return changes
        with self._lock:
            status = {**active["tactile_status"], **changes}
            active["tactile_status"] = status
        if report and status.get("message"):
            # The bundle does not enable INFO logging; stdout is visible in docker logs.
            print(f"[hand_grasp][{active['action_id']}] {status['message']}", flush=True)
        return status

    def _read_matrix_with_recovery(self, client, detector):
        """Hold the existing command while retrying only transient timeouts.

        Every attempt starts a whole fresh query. No partial matrices or older
        peaks can authorize a position advance. Malformed data is not retried.
        """
        started = time.monotonic()
        deadline = started + self.matrix_recovery_timeout_s
        attempts = 0
        last_error = None
        while attempts <= self.matrix_retry_count:
            self._check_stop()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            attempts += 1
            try:
                event = getattr(self._worker_context, "stop_event", self._stop)
                matrix = client.get_fresh_matrix_touch(
                    timeout_s=min(self.matrix_timeout_s, remaining), cancel=event)
                self._matrix_peaks(matrix)
                if time.monotonic() > deadline:
                    raise TimeoutError("完整触觉反馈返回时已超过本次恢复预算")
                recovery = {"matrix_query_attempts": attempts,
                            "matrix_retries": attempts - 1,
                            "matrix_recovery_elapsed_s": time.monotonic() - started}
                if attempts > 1:
                    self._set_tactile_status(
                        message=f"触觉采集已恢复：第{attempts}次查询收到五指完整12×6新阵列，继续读取实际位置后再判断进给",
                        **recovery)
                return matrix, recovery
            except TimeoutError as exc:
                last_error = exc
                # Missing observations cannot count as a continuous stall window.
                detector.windows.clear()
                self._set_tactile_status(
                    phase="recovering_tactile", sample_state="waiting", matrix_valid=False,
                    position_valid=False, actual_positions=None,
                    peaks={f: None for f in FINGER_NAMES},
                    matrix_query_attempts=attempts, matrix_retries=attempts - 1,
                    error_code=getattr(exc, "code", "tactile_query_timeout"),
                    error_details=getattr(exc, "details", {}),
                    message=f"触觉查询第{attempts}次超时，保持已有位置指令、不继续收紧；正在检查剩余重采集预算：{exc}",
                )
                if attempts > self.matrix_retry_count:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._wait(min(0.02, remaining))

        self._check_stop()
        details = {
            **getattr(last_error, "details", {}),
            "matrix_query_attempts": attempts, "matrix_retries": max(0, attempts - 1),
            "matrix_recovery_elapsed_s": time.monotonic() - started,
            "matrix_recovery_budget_s": self.matrix_recovery_timeout_s,
        }
        raise TestGraspFault(
            getattr(last_error, "code", "tactile_query_timeout"),
            f"触觉重新采集未能在次数或时间预算内恢复（已尝试{attempts}次）；{last_error or '恢复预算已耗尽'}",
            details,
        ) from last_error

    def _do_adaptive_grasp(self, client, ctrl, options: dict) -> dict:
        self._check_stop()
        threshold = options["tactile_threshold"]
        # 1. 设置电机速度与抓取力矩限制
        with ctrl._motion_lock:
            self._check_stop()
            client.set_speeds([options["approach_speed"]] * 5)
            client.set_currents([options["motion_current"]] * 5)

        # 2. 从完全张开过渡到准备姿态
        self._open_hand(client, ctrl)
        self._move_pose(client, ctrl, self.prep_pose, "自适应抓取准备姿态")

        # 3. 自适应多指收拢循环
        cmd = list(self.prep_pose)
        contact_detected = {f: False for f in FINGER_NAMES}
        reached_limit = {f: False for f in FINGER_NAMES}
        peaks = {f: None for f in FINGER_NAMES}
        sample_sequence = 0
        last_report = 0.0
        previous_contacts = dict(contact_detected)
        stalled_joints = {f: {joint: False for joint in FINGER_JOINTS[f]} for f in FINGER_NAMES}
        joint_states = {f: {joint: "advancing" for joint in FINGER_JOINTS[f]} for f in FINGER_NAMES}
        previous_joint_states = None
        stall_evidence = {f: {} for f in FINGER_NAMES}
        detector = _JointStallDetector(self.stall_error_threshold, self.stall_position_tolerance,
                                       self.stall_duration_s)
        actual = None
        matrix_retries_total = 0
        matrix_recoveries = 0
        sampled_cmd = list(cmd)
        end_reason = "all_fingers_settled"

        deadline = time.monotonic() + self.timeout_s

        while not self._stop.is_set():
            self._check_stop()
            loop_start = time.monotonic()
            if loop_start >= deadline:
                end_reason = "timeout"
                break

            # 3.1 查询触觉点阵：只有完整、有效的新阵列才允许继续进给。
            query_started = time.monotonic()
            try:
                matrix, recovery = self._read_matrix_with_recovery(client, detector)
                peaks = self._matrix_peaks(matrix)
            except InterruptedError:
                raise
            except Exception as exc:
                self._set_tactile_status(
                    phase="error", sample_state="error", matrix_valid=False,
                    position_valid=False, actual_positions=None,
                    peaks={f: None for f in FINGER_NAMES},
                    query_elapsed_s=time.monotonic() - query_started,
                    error_code=getattr(exc, "code", "invalid_tactile_matrix"),
                    error_details=getattr(exc, "details", {}),
                    message=f"12×6触觉阵列采集失败，未继续收紧：{exc}",
                )
                raise TestGraspFault(
                    getattr(exc, "code", "invalid_tactile_matrix"),
                    f"12×6触觉阵列采集失败，未继续收紧：{exc}",
                    getattr(exc, "details", {}),
                ) from exc

            matrix_query_elapsed_s = time.monotonic() - query_started
            matrix_sampled_at = time.time()
            matrix_retries_total += recovery["matrix_retries"]
            matrix_recoveries += int(recovery["matrix_retries"] > 0)
            self._check_stop()
            position_started = time.monotonic()
            try:
                with ctrl._motion_lock:
                    self._check_stop()
                    actual = client.get_positions_vector()
                if (not isinstance(actual, list) or len(actual) != 20
                        or any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255
                               for value in actual)):
                    raise TestGraspFault("invalid_position_feedback", "未收到有效的20项位置反馈")
            except InterruptedError:
                raise
            except Exception as exc:
                self._set_tactile_status(
                    phase="error", sample_state="error", matrix_valid=True, position_valid=False,
                    peaks=dict(peaks), matrix_sample_timestamp=matrix_sampled_at,
                    actual_positions=None, position_query_elapsed_s=time.monotonic() - position_started,
                    error_code=getattr(exc, "code", "invalid_position_feedback"),
                    error_details=getattr(exc, "details", {}),
                    message=f"实际位置反馈失败，未继续收紧：{exc}",
                )
                raise TestGraspFault(getattr(exc, "code", "invalid_position_feedback"),
                                     f"实际位置反馈失败，未继续收紧：{exc}",
                                     getattr(exc, "details", {})) from exc
            self._check_stop()
            position_sampled = time.monotonic()
            position_query_elapsed_s = position_sampled - position_started
            sampled_cmd = list(cmd)
            for f in FINGER_NAMES:
                if peaks[f] >= threshold:
                    contact_detected[f] = True
                for joint, index in FINGER_JOINTS[f].items():
                    if contact_detected[f]:
                        joint_states[f][joint] = "tactile_threshold"
                        continue
                    if not stalled_joints[f][joint] and detector.update(
                            index, cmd[index], actual[index], position_sampled,
                            closing=cmd[index] < self.prep_pose[index]):
                        stalled_joints[f][joint] = True
                        window = detector.windows[index]
                        stall_evidence[f][joint] = {
                            'command': cmd[index], 'actual': actual[index],
                            'tracking_error': actual[index] - cmd[index],
                            'observed_s': position_sampled - window['since'],
                            'samples': window['samples'],
                            'position_spread': window['maximum'] - window['minimum'],
                        }
                    joint_states[f][joint] = (
                        "position_stalled" if stalled_joints[f][joint] else
                        "command_limit" if cmd[index] <= self.close_limit_pose[index] else "advancing"
                    )
            # Select each joint's next increment from the same fresh feedback
            # used above. This slows reference advance when tracking lags;
            # it never retracts a target or imposes an extra travel boundary.
            planned_steps = {f: {} for f in FINGER_NAMES}
            for f in FINGER_NAMES:
                for joint, index in FINGER_JOINTS[f].items():
                    step = (self.step_size
                            if actual[index] - sampled_cmd[index] >= self.slowdown_error_threshold
                            else self.fast_step_size)
                    planned_steps[f][joint] = (
                        min(step, sampled_cmd[index] - self.close_limit_pose[index])
                        if joint_states[f][joint] == "advancing" else 0
                    )
            sample_sequence += 1
            sampled_at = time.time()
            peak_text = ", ".join(f"{f}={peaks[f]}" for f in FINGER_NAMES)
            sample_message = (
                f"已收到五指完整12×6阵列及新位置反馈，峰值 {peak_text}；"
                f"累计{sum(contact_detected.values())}指达到阈值{threshold}；"
                f"关节状态 {joint_states}；指令/实际 "
                + ", ".join(f"{f}({joint}:{sampled_cmd[index]}/{actual[index]})"
                            for f in FINGER_NAMES for joint, index in FINGER_JOINTS[f].items())
                + "；下次计划步长（根/尖） "
                + ", ".join(f"{f}={steps['root']}/{steps['tip']}"
                            for f, steps in planned_steps.items())
            )
            report = (sample_sequence == 1 or contact_detected != previous_contacts
                      or joint_states != previous_joint_states or loop_start - last_report >= 1.0)
            self._set_tactile_status(
                phase="closing", sample_state="ok", matrix_valid=True, position_valid=True,
                error_code=None, error_details={},
                matrix_retries_total=matrix_retries_total, matrix_recoveries=matrix_recoveries,
                sample_sequence=sample_sequence, sample_timestamp=sampled_at,
                matrix_sample_timestamp=matrix_sampled_at,
                position_sample_timestamp=sampled_at, position_query_elapsed_s=position_query_elapsed_s,
                query_elapsed_s=matrix_query_elapsed_s,
                sample_elapsed_s=time.monotonic() - query_started,
                peaks=dict(peaks), contacts=dict(contact_detected), message=sample_message,
                actual_positions=vector20_to_named16(actual),
                sampled_target_positions=vector20_to_named16(sampled_cmd),
                planned_steps=planned_steps,
                joint_states={f: dict(states) for f, states in joint_states.items()},
                stalled_joints={f: dict(joints) for f, joints in stalled_joints.items()},
                stall_evidence={f: dict(evidence) for f, evidence in stall_evidence.items()},
                report=report,
                **recovery,
            )
            if report:
                last_report = loop_start
            previous_contacts = dict(contact_detected)
            previous_joint_states = {f: dict(states) for f, states in joint_states.items()}

            # 3.2 Only advance unresolved joints. A frozen command must never
            # be rebased to actual: the sibling joint may still keep closing.
            moved_any = False
            for f in FINGER_NAMES:
                if not contact_detected[f]:
                    for joint, index in FINGER_JOINTS[f].items():
                        if joint_states[f][joint] == "advancing":
                            cmd[index] -= planned_steps[f][joint]
                            moved_any = True
                    reached_limit[f] = all(cmd[index] <= self.close_limit_pose[index]
                                           for index in FINGER_JOINTS[f].values())

            # 3.3 发送关节位置指令
            if moved_any:
                with ctrl._motion_lock:
                    self._check_stop()
                    client.set_joint_positions(cmd)

            # States correspond to the sampled command, so a newly reached
            # command boundary gets one fresh position/tactile read before completion.
            all_settled = all(all(state != "advancing" for state in states.values())
                              for states in joint_states.values())
            num_contacts = sum(1 for f in FINGER_NAMES if contact_detected[f])

            if all_settled:
                logger.info(
                    "[hand_grasp] 所有手指已就绪 (触觉接触: %d, 到位极限: %d)",
                    num_contacts,
                    sum(1 for f in FINGER_NAMES if reached_limit[f]),
                )
                break

            elapsed = time.monotonic() - loop_start
            sleep_time = max(0.002, self.poll_interval_s - elapsed)
            self._wait(sleep_time)

        num_contacts = sum(1 for f in FINGER_NAMES if contact_detected[f])
        contact_finger_names = [f for f in FINGER_NAMES if contact_detected[f]]

        peak_text = ", ".join(f"{f}={peaks[f]}" for f in FINGER_NAMES)
        reason = (
            f"收拢观察超时（{self.timeout_s:g}秒）"
            if end_reason == "timeout" else "五指关节均已触觉达阈、位置持续受阻或指令到界"
        )
        outcome = ("timeout_hold" if end_reason == "timeout" else
                   "contact_hold" if num_contacts else
                   "position_stop_hold" if any(any(joints.values()) for joints in stalled_joints.values()) else
                   "limit_hold")
        summary = {
            "timeout_hold": "收拢超时，已停止进给并保持已有位置目标；请检查物体位置或执行 release",
            "contact_hold": f"自适应收拢完成，{num_contacts}指检测到触觉接触，已保持至 release",
            "position_stop_hold": "自适应收拢完成，检测到关节持续受阻，已保持；没有指尖触觉接触证据",
            "limit_hold": "收拢指令已到界并保持，没有检测到指尖触觉接触；请检查是否抓空",
        }[outcome]
        msg = (
            f"{summary}；{reason}；已收到{sample_sequence}轮五指完整12×6阵列；"
            f"最后峰值 {peak_text}；累计{num_contacts}指达到阈值{threshold}"
            f"（{', '.join(contact_finger_names) or '无'}）；关节状态 {joint_states}；"
            "保持已有位置指令至release，未持续监控反馈，不代表已抓稳"
        )
        self._set_tactile_status(phase="holding", monitoring=False, message=msg)

        return {
            "message": msg,
            "state": "completed",
            "action": "adaptive_grasp",
            "command_sent": True,
            "holding": True,
            "outcome": outcome,
            "grasp_confirmed": False,
            "motion_current": options["motion_current"],
            "speed_ratio": options["speed_ratio"],
            "approach_speed": options["approach_speed"],
            "tactile_threshold": threshold,
            "contacts": contact_detected,
            "contact_fingers": contact_finger_names,
            "contact_count": num_contacts,
            "peaks": peaks,
            "end_reason": end_reason,
            "timed_out": end_reason == "timeout",
            "reached_limit": reached_limit,
            "actual_positions": vector20_to_named16(actual) if actual is not None else None,
            "sampled_target_positions": vector20_to_named16(sampled_cmd),
            "joint_states": joint_states,
            "stalled_joints": stalled_joints,
            "stall_evidence": stall_evidence,
            "matrix_retries_total": matrix_retries_total,
            "matrix_recoveries": matrix_recoveries,
            "target_positions": vector20_to_named16(cmd),
        }

    def _do_release(self, client, ctrl) -> dict:
        target = validate_position_vector(ctrl._open_pose, "open_pose")
        with ctrl._motion_lock:
            measured = client.get_positions_vector()
            self._check_stop()
            indices = [i for i in range(20) if i not in (11, 12, 13, 14)]
            if all(abs(measured[i] - target[i]) <= self.position_tolerance for i in indices):
                client.set_currents([0] * 5)
                return {
                    "message": "已处于释放姿态，已卸力，无需重复运动",
                    "state": "completed",
                    "action": "release",
                    "positions": vector20_to_named16(measured),
                    "already_released": True,
                    "hand_opened": True,
                    "torque_released": True,
                    "holding": False,
                }
            client.set_speeds([self.release_speed] * 5)
            client.set_currents([self.motion_current] * 5)

        # 1. 恢复准备姿态松开物体
        self._move_pose(client, ctrl, self.prep_pose, "恢复准备姿态并松开物体")
        # 2. 完全张开
        measured = self._open_hand(client, ctrl)
        # 3. 电机卸力
        with ctrl._motion_lock:
            self._check_stop()
            client.set_currents([0] * 5)

        return {
            "message": "已成功释放：五指恢复张开姿态并卸力",
            "state": "completed",
            "action": "release",
            "positions": vector20_to_named16(measured),
            "hand_opened": True,
            "torque_released": True,
            "holding": False,
        }

    def _unload_owned(self, ctrl, owner) -> bool:
        with ctrl._motion_lock:
            with self._lock:
                if vars(ctrl).get("_test_grasp_owner") != owner:
                    return False
                try:
                    client = vars(ctrl).get("_client")
                    if client is None:
                        return False
                    client.set_currents([0] * 5)
                    ctrl._holding_torque = 0
                    return True
                except Exception:
                    return False
                finally:
                    del ctrl._test_grasp_owner
                    if vars(ctrl).get("_grasp_owner") == owner:
                        del ctrl._grasp_owner

    def _acp_callback(self, action_id: str, status: str, result: dict) -> None:
        HandPlugin._acp_callback(self, action_id, status, result)

    def _run_action(self, device_key: tuple[str, str], active: dict) -> None:
        client, ctrl = active["client"], active["controller"]
        self._worker_context.active = active
        self._worker_context.stop_event = active["stop_event"]
        try:
            predecessor = active.get("predecessor")
            while predecessor is not None and not predecessor["done_event"].wait(0.02):
                self._check_stop()
            self._check_stop()
            with ctrl._motion_lock:
                ensure_motion_available(ctrl, active["owner"])
                verify_hand_connection(client)

            if active["action"] == "release":
                result = self._do_release(client, ctrl)
            else:
                result = self._do_adaptive_grasp(client, ctrl, active["options"])
            self._check_stop()
            status = "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"message": str(exc), "state": "cancelled", "holding": False}
        except Exception as exc:
            label = "释放" if active["action"] == "release" else "自适应抓取"
            status, result = "error", {
                "message": f"{label}失败：{exc}",
                "state": "error",
                "error_code": getattr(exc, "code", "device_error"),
                "holding": False,
                "hand_opened": False,
                **getattr(exc, "details", {}),
            }

        with ctrl._motion_lock:
            with self._lock:
                current = self._active.get(device_key) is active
                successor = device_key in self._active and not current
                if status == "completed" and (active["stop_event"].is_set() or self._stop.is_set() or not current):
                    status, result = "cancelled", {"message": "motion stopped", "state": "cancelled", "holding": False}

                keep = status == "completed" and active["action"] == "adaptive_grasp"
                if keep:
                    ctrl._grasp_owner = active["owner"]
                    ctrl._holding_torque = active["options"]["motion_current"]
                    self._sessions[device_key] = {
                        **active["options"],
                        "source_instance_id": active["instance_id"],
                    }
                elif not successor:
                    self._sessions.pop(device_key, None)
                    result["torque_released"] = self._unload_owned(ctrl, active["owner"])

                if not successor:
                    release_motion(ctrl, active["owner"])

                if status != "completed":
                    self._set_tactile_status(phase=status, monitoring=False,
                                             message=result["message"])
                elif active["action"] == "release":
                    self._set_tactile_status(phase="released", monitoring=False,
                                             message=result["message"])
                result = {
                    "tactile_status": dict(active["tactile_status"]),
                    "bound_side": active["side"],
                    "can_channel": device_key[1],
                    "action_id": active["action_id"],
                    **result,
                    "success": status == "completed",
                }
                terminal = {"action_id": active["action_id"], "status": status, "result": result}
                if current:
                    self._active.pop(device_key, None)
                    self._states[device_key] = "holding" if keep else "error" if status == "error" else "ready"
                self._results[device_key] = self._last_result = terminal
                active["terminal"] = terminal
                active["done_event"].set()

        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            self._delivery_failed = True
            logger.warning("[ACP] %s: %s", active["action_id"], exc)

    def _start_action(self, instance_id: str, action: str, args: dict) -> dict:
        side = validate_bound_side(args.get("bound_side"))
        channel = self.global_config.get("can_channel", "can0")
        device_key = (side, channel)
        owner = (id(self), device_key)

        with self._lock:
            options = {"action": action}
            if action == "adaptive_grasp":
                ratio = args.get("speed_ratio", "1.0x")
                if not isinstance(ratio, str) or ratio not in SPEED_RATIOS:
                    raise ValueError("speed_ratio 必须为 " + ", ".join(SPEED_RATIOS))
                options.update({
                    "speed_ratio": ratio,
                    "approach_speed": min(255, round(85 * SPEED_RATIOS[ratio])),
                    "motion_current": self.motion_current,
                    "tactile_threshold": self.tactile_threshold,
                })
            previous = self._active.get(device_key)
            if previous is not None:
                if action != "release":
                    raise RuntimeError("当前正在抓握或释放，请先执行 release 后再重新抓取")
                active = previous if previous["action"] == "release" else None
                client, side, ctrl = previous["client"], previous["side"], previous["controller"]
            else:
                if action != "release" and device_key in self._sessions:
                    raise RuntimeError("当前正在保持抓握，请先执行 release，再重新抓取，避免意外重新开手")
                client, side, ctrl = self._get_client_and_ctrl(side, channel)
                if not ctrl._motion_lock.acquire(blocking=False):
                    raise RuntimeError("该手正在执行其他动作，请等待结束后再抓握")
                try:
                    claim_motion(ctrl, owner, config=self.global_config)
                    verify_hand_connection(client)
                    ctrl._test_grasp_owner = owner
                except Exception:
                    release_motion(ctrl, owner, restore_previous=True)
                    raise
                finally:
                    ctrl._motion_lock.release()
                active = None

            if active is None:
                active = {
                    "action": action,
                    "action_id": f"test_{action}_{uuid4().hex[:12]}",
                    "instance_id": instance_id,
                    "device_key": device_key,
                    "options": options,
                    "client": client,
                    "side": side,
                    "controller": ctrl,
                    "owner": owner,
                    "predecessor": previous,
                    "stop_event": threading.Event(),
                    "done_event": threading.Event(),
                    "terminal": None,
                    "tactile_status": {
                        "source": "matrix_peak", "matrix_shape": [12, 6],
                        "position_valid": False, "actual_positions": None,
                        "stall_settings": {
                            "error_threshold": self.stall_error_threshold,
                            "position_tolerance": self.stall_position_tolerance,
                            "duration_s": self.stall_duration_s, "min_samples": 3,
                        },
                        "step_settings": {
                            "step_size": self.step_size, "fast_step_size": self.fast_step_size,
                            "slowdown_error_threshold": self.slowdown_error_threshold,
                        },
                        "threshold": options.get("tactile_threshold", self.tactile_threshold),
                        "phase": "releasing" if action == "release" else "preparing",
                        "sample_state": "not_requested" if action == "release" else "waiting",
                        "matrix_valid": False, "monitoring": action == "adaptive_grasp",
                        "sample_sequence": 0, "sample_timestamp": None,
                        "peaks": {f: None for f in FINGER_NAMES},
                        "message": "释放动作不依赖触觉" if action == "release" else "准备姿态中，尚未开始采集12×6触觉阵列",
                    },
                }
                self._stop.clear()
                self._active[device_key] = active
                self._states[device_key] = "releasing" if action == "release" else "grasping"
                if previous is not None:
                    previous["stop_event"].set()
                start_worker = True
            else:
                start_worker = False

        if start_worker:
            try:
                threading.Thread(
                    target=self._run_action,
                    args=(device_key, active),
                    name=active["action_id"],
                    daemon=True,
                ).start()
            except Exception as exc:
                with self._lock:
                    if self._active.get(device_key) is active:
                        self._active.pop(device_key, None)
                        self._sessions.pop(device_key, None)
                        self._states[device_key] = "error"
                self._unload_owned(ctrl, active["owner"])
                release_motion(ctrl, active["owner"])
                active["terminal"] = {
                    "status": "error",
                    "result": {"state": "error", "message": str(exc), "holding": False},
                }
                active["done_event"].set()
                raise

        msg = (
            "释放已开始"
            if action == "release"
            else "自适应抓取已开始：准备姿态后读取五指12×6阵列最大值和实际位置；触觉达阈停整指，位置持续受阻停对应关节；用 get_state 查看进度和最终结果，release 释放"
        )
        return {"message": msg, "state": "running", "action_id": active["action_id"],
                "tactile_status": dict(active["tactile_status"])}

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id") or ""
        if action == "config":
            with self._lock:
                settings = {name: _setting(args.get(name, getattr(self, name)), name)
                            for name in ("motion_current", "tactile_threshold")}
                changed = any(value != getattr(self, name) for name, value in settings.items())
                if changed and (self._active or self._sessions):
                    raise RuntimeError("自适应抓取正在运行或保持，请先 release，再修改力矩档位或接触停止阈值")
                for name, value in settings.items():
                    setattr(self, name, value)
                self.config.update(settings)
            return {"state": "configured", "control_mode": "adaptive_tactile", **settings}
        if action == "start":
            return {"state": "ready", "control_mode": "adaptive_tactile", "selection_required": True}
        if action in ("info", "get_state"):
            side = args.get("bound_side")
            if action == "get_state":
                side = validate_bound_side(side)
            if not side:
                return {
                    "state": "ready",
                    "bound_side": None,
                    "active_action_id": None,
                    "last_result": None,
                    "control_mode": "adaptive_tactile",
                    "selection_required": True,
                    "message": "请选择左手或右手后读取状态",
                }
            side = validate_bound_side(side)
            device_key = (side, self.global_config.get("can_channel", "can0"))
            with self._lock:
                active = self._active.get(device_key)
                state = self._states.get(device_key, "ready")
                last_result = self._results.get(device_key)
                sample = (active["tactile_status"] if active else
                          (last_result or {}).get("result", {}).get("tactile_status", {}))
                message = ("已停止；上一动作结果仅供查阅，张开请执行 release" if state == "idle" else
                           sample.get("message") or "当前没有活动任务或完成记录")
                return {
                    "state": state,
                    "message": message,
                    "bound_side": side,
                    "can_channel": device_key[1],
                    "action_id": active["action_id"] if active else (last_result or {}).get("action_id"),
                    "active_action_id": active["action_id"] if active else None,
                    "holding": device_key in self._sessions,
                    "monitoring": bool(active and sample.get("monitoring")),
                    "last_result": last_result,
                    "tactile_status": dict(sample),
                    "completion_delivery_failed": self._delivery_failed,
                    "control_mode": "adaptive_tactile",
                }
        if action == "stop":
            with self._lock:
                side = args.get("bound_side")
                if side:
                    side = validate_bound_side(side)
                    targets = [(side, self.global_config.get("can_channel", "can0"))]
                elif instance_id:
                    targets = [
                        key
                        for key in self._controllers
                        if (
                            self._active[key]["instance_id"] == instance_id
                            if key in self._active
                            else self._sessions.get(key, {}).get("source_instance_id") == instance_id
                        )
                    ]
                else:
                    targets = []
                controllers = []
                for key in targets:
                    active = self._active.get(key)
                    if active:
                        active["stop_event"].set()
                    self._sessions.pop(key, None)
                    self._states[key] = "stopping" if active else "idle"
                    ctrl = self._controllers.get(key)
                    if ctrl is not None:
                        controllers.append((key, ctrl))
            unloaded = False
            for key, ctrl in controllers:
                unloaded = self._unload_owned(ctrl, (id(self), key)) or unloaded
            return {
                "state": "idle",
                "active_action_id": None,
                "torque_released": unloaded,
                "target_resolved": bool(targets),
            }
        if action in TEST_ACTIONS:
            if args.get("confirm_motion") is not True:
                raise ValueError("confirm_motion 必须为 true")
            wait = args.get("wait_for_result", False)
            if not isinstance(wait, bool):
                raise ValueError("wait_for_result 必须为布尔值")
            if wait:
                raise ValueError(
                    "抓握测试动作使用异步完成协议，不支持 wait_for_result=true；请通过 action_id 接收通知"
                )
            return self._start_action(instance_id, action, args)

        return None
