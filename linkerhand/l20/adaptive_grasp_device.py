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

import json
import logging
import math
import os
import threading
import time
from typing import Any
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from dataclasses import asdict
from grasp_feedback import (GENTLE_DEFAULT_TRAVEL, GENTLE_MAX, SEARCH_WATCHDOG_S, SETTING_FIELDS, FeedbackGrasp,
                            contact_patches, empty_baseline, parameter_limits, settings_from)
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

# With the configured base yaw of 70, 115 aligns the thumb with the middle
# finger across the palm at the measured right-hand closing posture.
RIGHT_GENTLE_THUMB_YAW_OFFSET = 45
# Lower raw values curl the distal segment. Let the root complete the wrap
# while keeping the right thumb pad facing the opposing contact area.
RIGHT_GENTLE_THUMB_TIP_MIN_RAW = 195


def _finger_tip_travel_percent(side, gentle):
    # For the four opposing digits only. The thumb uses its own calibrated limit.
    return 75 if side == "right" and gentle else 100


FINGER_LABELS = dict(thumb="拇指", index="食指", middle="中指", ring="无名指", pinky="小指")

FINGER_JOINTS = {
    "thumb":  {"root": 0, "tip": 15},
    "index":  {"root": 1, "tip": 16},
    "middle": {"root": 2, "tip": 17},
    "ring":   {"root": 3, "tip": 18},
    "pinky":  {"root": 4, "tip": 19},
}

PLACEMENT_NOTICE = "请从上方摆好物品，人手避开闭合路径。说“放好了”立即抓，五秒无回复自动抓，说“等一下”或“停止”取消。"
PLACEMENT_SECONDS = 5.0

MOTION_ACTIONS = ("adaptive_grasp", "prepare", "close_until_contact", "release")


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

    def __init__(self, hand: HandPlugin | None, config: dict, global_config: dict | None = None, ros2=None):
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
        self._prepared = {}
        self._ros2 = ros2
        self._placement_node = None
        self.default_gentle_limits = os.environ.get('L20_DEFAULT_GENTLE_LIMITS') == '1'

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
        limits = parameter_limits(True)
        peak = limits["peak_ceiling"]
        contact = limits["contact_peak"]
        selection = (f"轻握 peak_ceiling 可选 {peak['minimum']}–{peak['maximum']}，"
                     f"contact_peak 可选 {contact['minimum']}–{contact['maximum']}，"
                     f"output_limit 最高 {limits['output_limit']['maximum']}。"
                     "调高峰值时选 peak_ceiling。执行 L20 轻柔抓取 Skill 时先激活其完整指令。")
        schema = action_schema(
            {
                "adaptive_grasp": (
                    ["bound_side", "speed_ratio", "confirm_motion"],
                    "自适应抓取：触觉达阈停整指，位置持续受阻停对应关节，保持至 release；这是接触停止，不是恒压控制",
                ),
                "preview_settings": (["bound_side", "gentle_limits", *SETTING_FIELDS],
                    "只读校验拟选参数，返回可选范围和预览数值，不移动手。"+selection),
                "prepare": (["bound_side", "gentle_limits", *SETTING_FIELDS, "confirm_motion"],
                    "收到抓取请求，空闲就直接张开并空手准备，旧回执不触发硬件诊断；持物或忙碌则保持。"+selection),
                "close_until_contact": (["bound_side", "prepare_id", "placement_notice", "confirm_motion"],
                    "右手轻握收到 prepared 后立即调用一次，携带本轮准备编号和摆放提示。驱动等放好或五秒后合手。三帧均值达标即动作成功，完整的过压帧也计入均值并在当帧保护；归零保持，自动监测至释放"),
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
                    "enum": ["left", "right"],
                    "description": "本次操作的灵巧手，必须选择左手或右手",
                },
                "instance_id": {"type": "string"},
                "gentle_limits": {"type": "boolean", "default": self.default_gentle_limits,
                                  "description": f"启用整手反馈抓取的统一参数边界；六项整数分别选值：{GENTLE_MAX}"},
                **{name: {"type": "integer", "minimum": low, "maximum": high,
                          "default": default, "description": desc}
                   for name, (low, high, default, desc) in SETTING_FIELDS.items()},
                "speed_ratio": {
                    "type": "string", "enum": list(SPEED_RATIOS), "default": "1.0x",
                    "description": "抓取电机速度：1.0x=85，1.5x=128，2.0x=170，2.5x=212，3.0x=255；不改变触觉阈值或位置步长",
                },
                "confirm_motion": {
                    "type": "boolean",
                    "const": True, "enum": [False, True],
                    "description": "现场确认后选择 true",
                },
                "prepare_id": {"type": "string", "description": "右手轻握本轮已经返回 prepared 的 action_id"},
                "placement_notice": {"type": "string", "enum":[PLACEMENT_NOTICE],
                                     "description": "画布本轮摆放提示，随工具调用显示；右手轻握必填"},
                "wait_for_result": {
                    "type": "boolean",
                    "const": False,
                    "default": False,
                    "description": "仅兼容 false；动作立即返回 action_id，最终结果通过 ACP 通知",
                },
            },
        )
        for name, bounds in limits.items():
            field = schema["properties"][name]
            field["description"] += f"；轻握可选整数 {bounds['minimum']}–{bounds['maximum']}"
        schema["properties"]["peak_ceiling"]["description"] += "；纸团可参考 15 左右，偏硬且轻按不变形的物品可参考 80–100，每轮仅依当前物品的材质和易变形程度自主选值。极软杯套从低值起，可参考纸团附近或更低。上轮单指过压只提示检查摆位，不得据此提高峰值或增加补夹次数。150 是允许上限"
        schema["properties"]["contact_peak"]["description"] += "；收拢时同一指连续三帧峰值的算术平均达到此值即记本轮动作成功，归零保持，达标后自动监测"
        if self.default_gentle_limits:
            travel = schema["properties"]["approach_travel"]
            travel["default"] = GENTLE_DEFAULT_TRAVEL
            travel["description"] += f"；本机轻握默认 {GENTLE_DEFAULT_TRAVEL}，允许独立选至 {GENTLE_MAX['approach_travel']}，无名指和小指预算增加 20%，每指触物即停"
        if self.global_config.get("side") == "right":
            schema["properties"]["bound_side"]["enum"] = ["right", "left"]
        schema["allOf"] = [
            {
                "if": {"properties": {"action": {"enum": list(MOTION_ACTIONS)}}},
                "then": {
                    "required": ["bound_side", "confirm_motion"],
                    "properties": {"bound_side": {"enum": ["left", "right"]}},
                },
            },
            {
                "if": {"properties": {"action": {"enum": ["get_state", "preview_settings"]}}},
                "then": {"required": ["bound_side"],
                         "properties": {"bound_side": {"enum": ["left", "right"]}}},
            },
        ]
        # The split model tools retain property descriptions; the raw MCP schema
        # also enforces the conditional bounds, preserving explicit generic use.
        condition = {"properties": {"action": {"enum": ["prepare", "preview_settings"]},
                                    "gentle_limits": {"const": True}}, "required": ["action"]}
        if not self.default_gentle_limits:
            condition["required"].append("gentle_limits")
        schema["allOf"].append({"if": condition, "then": {"properties": limits}})
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {
            "actions": list(MOTION_ACTIONS),
            "timeout": max(30, math.ceil(4 * self.pose_timeout_s + self.timeout_s
                                         + self.matrix_recovery_timeout_s + 2)),
        }

        definition = tool(
            self.PREFIX,
            "actuator",
            "L20 整手抓取、参数化收拢、保持与滑动监测、释放与状态读取；支持位置和触觉反馈",
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
        if self._ros2 is None or self._placement_node is not None:
            return
        from rclpy.node import Node
        from std_msgs.msg import String
        node = Node("l20_placement_" + uuid4().hex[:8], context=self._ros2.ctx_core)
        try:
            node.create_subscription(String, "/remote_control/message", self._placement_input, 10)
            self._ros2.executor_core.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        self._placement_node = node

    def _placement_input(self, message):
        try:
            data = json.loads(message.data)
            text = data["text"].strip().strip("。，！？.!? ").lower()
            sent = float(data["ts"])
        except (AttributeError, ValueError, TypeError, KeyError):
            return
        if not math.isfinite(sent) or sent > time.time() + 0.1:
            return
        cancel = any(word in text for word in ("等一下", "等等", "等下", "停止", "取消", "暂停", "别抓", "不要抓", "先别", "没放好", "没准备好")) or text in ("停","等","stop","cancel")
        ready = text in ("放好了", "摆好了", "已就位", "就位", "好了", "可以抓", "放好了，可以抓", "ok", "准备好了")
        if not (cancel or ready):
            return
        key = ("right", self.global_config.get("can_channel","can0"))
        with self._lock:
            active = self._active.get(key)
            if cancel:
                prepared = self._prepared.get(key)
                if prepared and prepared.get("timed_placement") and sent >= prepared["request_time"]:
                    self._prepared.pop(key)
                if (active and active["options"].get("timed_placement") and sent >= active["options"]["request_time"]
                        and (active["action"]=="prepare" or active.get("placement_open"))):
                    active["stop_event"].set()
                return
            if active and active.get("placement_open") and sent >= active["placement_start_wall"]:
                active["placement_ready"].set()

    def _input_available(self):
        node = self._placement_node
        if node is None:
            return False
        publishers = node.get_publishers_info_by_topic("/remote_control/message")
        return len(publishers) == 1 and publishers[0].node_name == "phanthy_bus_bridge"

    def _wait_for_placement(self, active):
        self._set_tactile_status(phase="waiting_for_placement", message=PLACEMENT_NOTICE)
        while True:
            self._check_stop()
            if not self._input_available():
                raise TestGraspFault("placement_input_unavailable","摆放回复通道不可用，本轮取消合手")
            with self._lock:
                self._check_stop()
                if active["placement_ready"].is_set() or time.monotonic() >= active["placement_deadline"]:
                    active["placement_open"] = False
                    return
            self._wait(0.025)

    def stop(self) -> None:
        self._stop.set()
        node, self._placement_node = self._placement_node, None
        if node is not None:
            self._ros2.executor_core.remove_node(node)
            node.destroy_node()
        with self._lock:
            self._prepared.clear()
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
                    phase="recovering_tactile", sample_state="waiting", matrix_valid=False, slip_monitoring_available=False,
                    tactile_contact_confirmed=False, contact_fingers=[], opposed_now=False,
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

    def _prepare_feedback_grasp(self, client, ctrl, active):
        profile = active["options"]["settings"]
        with ctrl._motion_lock:
            self._check_stop()
            client.set_speeds([profile.speed] * 5)
            client.set_currents([profile.output_limit] * 5)
        self._open_hand(client, ctrl)
        self._move_pose(client, ctrl, active["options"]["prep_pose"], "五指轻握准备姿态")
        frames = []
        for _ in range(3):
            self._check_stop()
            matrix = client.get_fresh_matrix_touch(timeout_s=self.matrix_timeout_s,
                                                  cancel=active["stop_event"])
            peaks = self._matrix_peaks(matrix)
            if max(peaks.values()) >= profile.contact_peak:
                raise TestGraspFault("tare_contact", "空手采样时已有压力，请移开物品再准备")
            frames.append(matrix)
        active["empty_baseline"] = empty_baseline(frames)
        limits = parameter_limits(active["options"]["gentle_limits"])
        return {"message": (f"准备完毕，本轮峰值 {profile.peak_ceiling}，"
                            f"可选上限 {limits['peak_ceiling']['maximum']}，整手截停值 {profile.hard_stop_peak_raw}；准备完成，可以发出本轮摆放提示"),
                "parameter_limits": limits, "settings": asdict(profile), "state": "prepared",
                "next_action": "close_until_contact", "prepare_id": active["action_id"],
                "placement_notice": PLACEMENT_NOTICE if active["options"].get("timed_placement") else None,
                "hard_stop_peak_raw": profile.hard_stop_peak_raw,
                "travel_by_finger": profile.travel_by_finger,
                "finger_tip_travel_percent": active["options"]["finger_tip_travel_percent"],
                "thumb_tip_min_raw": active["options"]["close_limit_pose"][15],
                "gentle_limits": active["options"]["gentle_limits"],
                "thumb_yaw_target": active["options"]["prep_pose"][5]}

    def _notify_contact(self, active, sample):
        verified = sample["tactile_contact_confirmed"]
        phase = sample["phase"]
        try:
            self._acp_callback(active["action_id"], "completed", {
                "state": phase,
                "holding": True, "grasp_action_succeeded": sample["grasp_action_succeeded"],
                "grasp_confirmed": sample["grasp_confirmed"], "tactile_contact_confirmed": verified,
                "contact_fingers": sample["contact_fingers"], "search_stop": sample["search_stop"],
                "thumb_middle_contact_seen": sample["thumb_middle_contact_seen"],
                "remaining_fingers": sample["remaining_fingers"], "travel_limited_fingers": sample["travel_limited_fingers"],
                "approach_contact_windows": sample["approach_contact_windows"], "peaks": sample["peaks"],
                "message": sample["message"],
                "action_id": active["action_id"], "bound_side": active["side"],
                "can_channel": active["device_key"][1],
            })
        except Exception as exc:
            self._delivery_failed = True
            logger.warning("轻握接触回调失败: %s", exc)

    def _run_feedback_grasp(self, client, ctrl, active):
        options = active["options"]
        profile = options["settings"]
        with ctrl._motion_lock:
            self._check_stop()
            initial = client.get_positions_vector()
            if any(abs(initial[i] - options["prep_pose"][i]) > self.position_tolerance
                   for i in range(20) if i not in (11, 12, 13, 14)):
                raise TestGraspFault("prepare_pose_changed", "准备姿态已改变，请移开物品重新准备")
            client.set_speeds([profile.speed] * 5)
            client.set_currents([profile.output_limit] * 5)
        control = FeedbackGrasp(profile, initial, options["close_limit_pose"],
                                finger_tip_travel_percent=options["finger_tip_travel_percent"])
        if options["prep_pose"][5] != self.prep_pose[5]:
            # Preserve the prepared yaw setpoint in subsequent whole-hand position packets.
            control.target[5] = options["prep_pose"][5]
        detector = _JointStallDetector(self.stall_error_threshold, self.stall_position_tolerance,
                                       self.stall_duration_s)
        seq = 0
        hold_since = None
        while True:
            self._check_stop()
            try:
                matrix, recovery = self._read_matrix_with_recovery(client, detector)
                if recovery["matrix_retries"]:
                    control.contact_history.clear()
                    control.loss_frames = 0
                    control.clear_frames = dict.fromkeys(FINGER_NAMES, 0)
                    control.at_limit_frames = dict.fromkeys(FINGER_NAMES, 0)
                    control.slip_frames = dict.fromkeys(FINGER_NAMES, 0)
                patches = contact_patches(matrix, options["baseline"])
                with ctrl._motion_lock:
                    self._check_stop()
                    actual = client.get_positions_vector()
                    now = time.monotonic()
                    if control.update(patches, actual, now, hold_since is not None):
                        self._check_stop()
                        client.set_joint_positions(control.target)
            except InterruptedError:
                raise
            except Exception as exc:
                if not active.get("contact_ack"):
                    raise
                control.paused = f"反馈中断，已停止补夹，请托住物品：{exc}"
                self._set_tactile_status(phase="needs_support", monitoring=False,
                                         tactile_contact_confirmed=False, grasp_confirmed=bool(control.best_contact_windows), contact_fingers=[], opposed_now=False,
                                         slip_monitoring_available=False,
                                         matrix_valid=False, position_valid=False,
                                         message=control.paused)
                break
            seq += 1
            if control.paused and not active.get("contact_ack"):
                code = "tactile_peak_limit" if control.overpressure_fingers else "approach_limit"
                peak_values = {f: p.peak for f, p in patches.items()}
                details = {"grasp_action_succeeded": bool(control.best_contact_windows),
                           "thumb_middle_contact_seen": control.thumb_middle_contact_seen,
                           "approach_contact_windows": control.best_contact_windows.copy(),
                           "trigger_peaks": peak_values, "trigger_fingers": list(control.overpressure_fingers),
                           "peak_ceiling_raw": profile.peak_ceiling, "hard_stop_peak_raw": profile.hard_stop_peak_raw,
                           "peak_limited_fingers": list(control.peak_limited_fingers),
                           "saturated_fingers": list(control.saturated_fingers), "search_stop": control.search_stop}
                self._set_tactile_status(report=False, phase="needs_support", monitoring=False,
                                         sample_sequence=seq, sample_timestamp=time.time(), matrix_valid=True,
                                         peaks=peak_values, stop_reason=code, **details,
                                         actual_positions=vector20_to_named16(actual), message=control.paused)
                raise TestGraspFault(code, control.paused, details)
            if hold_since is None and not control.paused and control.best_contact_windows and control.search_stop in ("contact_faded","fingers_settled"):
                hold_since = now
            elapsed = now - hold_since if hold_since is not None else 0.0
            if control.paused:
                phase, msg = "needs_support", control.paused
            elif control.waiting_for_signal:
                phase = "hold_without_signal"
                msg = "本轮抓握动作成功，保持姿态；当前持续触觉不足，等新信号后自动恢复滑动监测"
                means = "、".join(f"{FINGER_LABELS[f]} {w['mean_peak_raw']:g}" for f, w in control.best_contact_windows.items())
                msg += f"（本轮各指最强连续三帧均值：{means}）"
            elif control.seek_contact:
                if control.best_contact_windows:
                    phase, msg = "wrapping", "本轮抓握动作成功，其余手指仍在收拢，请继续承托"
                else:
                    phase, msg = "approaching", "逐指搜索接触，请继续托住物品"
            elif control.search_stop in ("no_progress", "watchdog"):
                reason = "关节持续无进展" if control.search_stop == "no_progress" else "已达保护时限"
                phase = "needs_support" if control.best_contact_windows else "unverified_hold"
                msg = f"{reason}，分指搜索已停，请继续承托并检查"
            elif not control.best_contact_windows:
                phase, msg = "unverified_hold", "各指已达限位，尚未确认夹持，请继续承托"
            elif not control.stable_fingers:
                phase, msg = "rechecking_contact", "正在重新确认指尖接触，请保持承托或接稳物品"
            else:
                phase, msg = "monitoring", "本轮抓握动作成功，按当前位置保持并自动监测滑动，直到明确要求释放"
            if control.paused and control.best_contact_windows:
                msg = "本轮抓握动作已成功；随后触发保护。" + msg
            if control.peak_limited_fingers and not control.paused:
                msg += f"；{', '.join(control.peak_limited_fingers)} 达到设定峰值，该指暂停加紧"
            fingers = [f for f in FINGER_NAMES if f in control.stable_fingers]
            confirmed = bool(fingers) and not control.paused
            succeeded = bool(control.best_contact_windows)
            sample = self._set_tactile_status(
                report=False, phase=phase, message=msg, settings=asdict(profile),
                grasp_action_succeeded=succeeded,
                thumb_middle_contact_seen=control.thumb_middle_contact_seen,
                thumb_alignment_enabled=control.align_thumb, thumb_rotation_locked=control.thumb_rotation_locked,
                finger_tip_travel_percent=control.finger_tip_travel_percent,
                thumb_tip_min_raw=options["close_limit_pose"][15],
                travel_by_finger=profile.travel_by_finger, remaining_fingers=control.remaining_fingers,
                travel_limited_fingers=[f for f in FINGER_NAMES if f in control.travel_limited],
                matrix_valid=True, position_valid=True, monitoring=not control.paused,
                sample_sequence=seq, sample_timestamp=time.time(), sample_state="fresh",
                peaks={f: p.peak for f, p in patches.items()}, actual_positions=vector20_to_named16(actual),
                peak_limited_fingers=list(control.peak_limited_fingers), hard_stop_peak_raw=profile.hard_stop_peak_raw,
                saturated_fingers=list(control.saturated_fingers),
                search_stop=control.search_stop, trigger_fingers=list(control.overpressure_fingers),
                stop_reason="tactile_peak_limit" if control.overpressure_fingers else control.search_stop,
                tactile_contact_confirmed=confirmed,
                approach_contact_windows=control.best_contact_windows.copy(),
                slip_monitoring_available=bool(hold_since is not None and confirmed and not control.waiting_for_signal
                                               and profile.max_corrections),
                target_positions=vector20_to_named16(control.target), contact_fingers=fingers,
                opposed_now=control.opposed, relief_count=control.relief_count, corrections=control.corrections, last_slip_fingers=control.last_evidence,
                output_limit_raw=profile.output_limit, peak_ceiling_raw=profile.peak_ceiling,
                hold_elapsed_s=round(elapsed, 2), grasp_confirmed=succeeded)
            if not control.seek_contact and not active.get("contact_ack"):
                with self._lock:
                    self._check_stop()
                    active["contact_ack"] = True
                    self._states[active["device_key"]] = "holding"
                threading.Thread(target=self._notify_contact, args=(active, sample), daemon=True).start()
            if control.paused:
                break
            self._wait(self.poll_interval_s)
        # Preserve the last bounded posture while the operator supports the load.
        while True:
            self._wait(0.2)

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
            options = getattr(self._worker_context, 'active', {}).get('options', {})
            client.set_speeds([options.get('release_speed', self.release_speed)] * 5)
            client.set_currents([options.get('release_limit', self.motion_current)] * 5)

        # 1. 恢复准备姿态松开物体
        self._move_pose(client, ctrl, options.get("prep_pose", self.prep_pose), "恢复准备姿态并松开物体")
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
            elif active["action"] == "prepare":
                result = self._prepare_feedback_grasp(client, ctrl, active)
            elif active["action"] == "close_until_contact":
                if active.get("placement_open"):
                    self._wait_for_placement(active)
                result = self._run_feedback_grasp(client, ctrl, active)
            else:
                result = self._do_adaptive_grasp(client, ctrl, active["options"])
            self._check_stop()
            status = "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"message": str(exc), "state": "cancelled", "holding": False}
        except Exception as exc:
            label = "释放" if active["action"] == "release" else "抓取控制"
            status, result = "error", {
                "message": f"{label}停止：{exc}",
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
                if status == "completed" and active["action"] == "prepare":
                    self._prepared[device_key] = {
                        "settings": active["options"]["settings"],
                        "prep_pose": active["options"]["prep_pose"],
                        "close_limit_pose": active["options"]["close_limit_pose"],
                        "finger_tip_travel_percent": active["options"]["finger_tip_travel_percent"],
                        "gentle_limits": active["options"]["gentle_limits"],
                        "timed_placement": active["options"]["timed_placement"],
                        "prepare_id": active["action_id"], "request_time": active["options"]["request_time"],
                        "baseline": active["empty_baseline"],
                    }
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
                    achieved = bool(active["tactile_status"].get("grasp_action_succeeded"))
                    if active["action"] == "close_until_contact":
                        result["grasp_action_succeeded"] = result["grasp_confirmed"] = achieved
                        if achieved:
                            windows = active["tactile_status"].get("approach_contact_windows") or {}
                            evidence = "、".join(f"{FINGER_LABELS[f]}三帧均值 {w['mean_peak_raw']:g}"
                                                for f, w in windows.items())
                            result["message"] = f"本轮抓握动作已成功（{evidence}）。" + result["message"]
                    self._set_tactile_status(phase=status, monitoring=False, grasp_confirmed=achieved, slip_monitoring_available=False,
                                             tactile_contact_confirmed=False, contact_fingers=[], opposed_now=False,
                                             message=result["message"])
                elif active["action"] == "release":
                    self._set_tactile_status(phase="released", monitoring=False, grasp_confirmed=False, slip_monitoring_available=False,
                                             tactile_contact_confirmed=False, contact_fingers=[], opposed_now=False,
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

        if active.get("contact_ack"):
            return
        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            self._delivery_failed = True
            logger.warning("[ACP] %s: %s", active["action_id"], exc)

    def _select_settings(self, args):
        gentle = args.get("gentle_limits", self.default_gentle_limits)
        return settings_from({**args, "gentle_limits": gentle}), gentle

    def _feedback_close_limit(self, side, gentle):
        limit = self.close_limit_pose.copy()
        if side == "right" and gentle:
            limit[15] = max(limit[15], RIGHT_GENTLE_THUMB_TIP_MIN_RAW)
        return limit

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
            if action == "prepare":
                options["settings"], gentle = self._select_settings(args)
                options["gentle_limits"] = gentle
                options["timed_placement"] = gentle and side == "right"
                options["request_time"] = time.time()
                options["finger_tip_travel_percent"] = _finger_tip_travel_percent(side, gentle)
                pose = self.prep_pose.copy()
                if side == "right" and gentle:
                    pose[5] = min(255, pose[5] + RIGHT_GENTLE_THUMB_YAW_OFFSET)
                options["prep_pose"] = pose
                options["close_limit_pose"] = self._feedback_close_limit(side, gentle)
            elif action == "close_until_contact":
                if device_key not in self._prepared:
                    raise ValueError("请先空手执行 prepare 并等待准备完成")
                options.update(self._prepared[device_key])
                if options.get("timed_placement"):
                    if args.get("prepare_id") != options["prepare_id"] or args.get("placement_notice") != PLACEMENT_NOTICE:
                        raise ValueError("请使用本轮 prepared 的 prepare_id 和卡片声明的 placement_notice 启动一次摆放倒计时")
                    if not self._input_available():
                        raise ValueError("摆放回复通道不可用，本轮未启动倒计时")
            elif action == "release":
                prepared = self._prepared.get(device_key)
                if previous and previous["action"] in ("prepare", "close_until_contact"):
                    prepared = previous["options"]
                if prepared:
                    profile = prepared["settings"]
                    options.update(release_limit=profile.output_limit, release_speed=profile.speed,
                                   prep_pose=prepared["prep_pose"])
            if previous is not None:
                if action != "release":
                    raise RuntimeError("已有动作或正在持物，本次抓取未执行；持物时只在用户明确说松开后释放")
                active = previous if previous["action"] == "release" else None
                client, side, ctrl = previous["client"], previous["side"], previous["controller"]
            else:
                if action != "release" and device_key in self._sessions:
                    raise RuntimeError("当前正在持物，保持至用户明确要求松开")
                client, side, ctrl = self._get_client_and_ctrl(side, channel)
                if not ctrl._motion_lock.acquire(blocking=False):
                    raise RuntimeError("该手正在执行其他动作，请等待结束后再抓握")
                try:
                    if action == "prepare" and vars(ctrl).get("_holding_torque", 0):
                        raise RuntimeError("该手仍有保持出力，等用户明确说松开")
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
                self._prepared.pop(device_key, None)
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
                        "grasp_action_succeeded": False,
                        "peaks": {f: None for f in FINGER_NAMES},
                        "message": "释放动作不依赖触觉" if action == "release" else "准备姿态中，尚未开始采集12×6触觉阵列",
                    },
                }
                if action == "close_until_contact" and options.get("timed_placement"):
                    # Core broadcasts the tool call including the literal placement notice before dispatch.
                    active.update(placement_open=True, placement_start_wall=time.time(),
                                  placement_deadline=time.monotonic()+PLACEMENT_SECONDS, placement_ready=threading.Event())
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
        if action == "prepare":
            msg = "开始张开并空手准备，等待 prepared 后直接发本轮摆放提示"
        elif action == "close_until_contact":
            msg = PLACEMENT_NOTICE if active["options"].get("timed_placement") else "开始逐指收拢并自动监测"
        result = {"message": msg, "state": "running", "action_id": active["action_id"],
                  "tactile_status": dict(active["tactile_status"])}
        if action == "prepare":
            result.update(settings=asdict(active["options"]["settings"]),
                          hard_stop_peak_raw=active["options"]["settings"].hard_stop_peak_raw,
                          finger_tip_travel_percent=active["options"]["finger_tip_travel_percent"],
                          thumb_tip_min_raw=active["options"]["close_limit_pose"][15],
                          travel_by_finger=active["options"]["settings"].travel_by_finger,
                          parameter_limits=parameter_limits(active["options"]["gentle_limits"]))
        if action == "close_until_contact":
            result["expected_duration_s"] = SEARCH_WATCHDOG_S + self.matrix_recovery_timeout_s
        return result

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id") or ""
        if action == "preview_settings":
            side = validate_bound_side(args.get("bound_side"))
            settings, gentle = self._select_settings(args)
            limits = parameter_limits(gentle)
            return {"state": "preview", "bound_side": side, "gentle_limits": gentle,
                    "hard_stop_peak_raw": settings.hard_stop_peak_raw,
                    "thumb_alignment_enabled": False,
                    "finger_tip_travel_percent": _finger_tip_travel_percent(side, gentle),
                    "thumb_tip_min_raw": self._feedback_close_limit(side, gentle)[15],
                    "settings": asdict(settings), "parameter_limits": limits, "travel_by_finger": settings.travel_by_finger,
                    "message": (f"拟用峰值 {settings.peak_ceiling}，允许上限 "
                                f"{limits['peak_ceiling']['maximum']}，整手截停值 {settings.hard_stop_peak_raw}；参数尚未应用")}
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
                if active and active["action"] == "close_until_contact" and sample.get("phase") == "unverified_hold":
                    state = "unverified_hold"
                message = ("已停止；上一动作结果仅供查阅，张开请执行 release" if state == "idle" else
                           sample.get("message") or "当前没有活动任务或完成记录")
                if active is None and not self._sessions.get(device_key) and state == "error":
                    message = "当前无活动或持物；上一次动作已结束，新抓取可直接 prepare。历史回执：" + message
                return {
                    "state": state,
                    "message": message,
                    "bound_side": side,
                    "can_channel": device_key[1],
                    "action_id": active["action_id"] if active else (last_result or {}).get("action_id"),
                    "active_action_id": active["action_id"] if active else None,
                    "holding": device_key in self._sessions or bool(active and (active.get("contact_ack") or sample.get("grasp_confirmed"))),
                    "monitoring": bool(active and sample.get("monitoring")),
                    "default_parameter_limits": parameter_limits(self.default_gentle_limits),
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
                    self._prepared.pop(key, None)
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
        if action in MOTION_ACTIONS:
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
