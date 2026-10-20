"""Shared ROS squat processor for Bumi and Q5 pose skeleton frames.

The counter runs at camera speed in this driver. Decision Core receives only
sparse progress events and can read the authoritative total with ``status``.
"""

from __future__ import annotations

import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from common.squat_contract import tool_definition, topic_out
from common.squat_logic import DEFAULT_CONFIG, SquatCounter


_POSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
_OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    durability=DurabilityPolicy.VOLATILE,
)
_DEBUG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    durability=DurabilityPolicy.VOLATILE,
)

_PHASE_ZH = {
    "waiting_target": "等待识别人体", "calibrating": "校准站姿",
    "top": "站立", "descent": "下蹲中", "bottom": "蹲底",
    "ascent": "起身中", "paused": "暂停", "rearming": "等待重新站稳",
    "finished": "本组已结束",
}
_WORKOUT_STATE_ZH = {
    "waiting_start": "等待语音开始", "active": "训练中", "finished": "训练已结束",
}
_SET_STATE_ZH = {
    "preparing": "等待站稳开始本组", "in_set": "本组计数中",
    "resting": "组间休息", "ready_next": "等待下一组口令",
    "finished": "训练计划完成",
}
_TARGET_STATE_ZH = {
    "none": "尚未锁定", "tracking": "跟踪中", "missing": "目标丢失",
}
_SIDE_ZH = {"left": "左侧", "right": "右侧", None: "未确定"}
_REASON_ZH = {
    "waiting_for_readable_person": "等待画面中出现关节清晰的人体",
    "non_monotonic_timestamp": "姿态帧时间未递增，已忽略此帧",
    "target_unreadable": "已锁定人体的关键关节不可读",
    "target_missing": "已锁定人体暂时离开画面",
    "target_changed": "已切换目标，开始新的计数会话",
    "target_locked": "已锁定目标，开始校准站姿",
    "target_geometry_invalid": "人体关节几何关系无效",
    "target_joint_unreadable": "当前侧的关节不可读",
    "calibration_pose_invalid": "校准站姿不符合要求",
    "calibration_collecting": "正在收集站姿校准数据",
    "calibration_waiting_still": "等待站稳以完成校准",
    "calibration_knee_unstable": "校准时膝角变化过大",
    "calibration_hip_unstable": "校准时髋角变化过大",
    "calibration_hip_position_unstable": "校准时髋部位置变化过大",
    "calibration_ankle_position_unstable": "校准时脚踝位置变化过大",
    "baseline_ready": "站姿基线已就绪",
    "ankle_motion_exceeded": "脚踝移动过大，本次动作不计数",
    "waiting_for_top_to_rearm": "等待重新站直以恢复计数",
    "rearmed": "已重新站稳，可以开始下一次深蹲",
    "reanchor_collecting": "检测到新的稳定站姿，正在自动更新基线",
    "reanchor_waiting_still": "等待站稳以自动更新基线",
    "baseline_reanchored": "站姿基线已自动更新，保留原有次数",
    "waiting_for_top_dwell_to_rearm": "等待站立保持足够时间以恢复计数",
    "descent_started": "检测到开始下蹲",
    "waiting_for_descent": "等待开始下蹲",
    "incomplete_squat_returned_top": "尚未达到蹲底条件就返回站立，不计数",
    "bottom_confirmed": "蹲底条件已确认",
    "waiting_for_bottom_dwell": "蹲底条件已满足，等待保持足够时间",
    "waiting_for_bottom": "等待达到蹲底条件",
    "ascent_started": "检测到开始起身",
    "bottom_held": "保持蹲底姿态",
    "returned_to_bottom": "起身途中重新回到蹲底",
    "waiting_for_full_stand": "等待完全站直",
    "rep_completed": "完成一次深蹲，计数加一",
    "waiting_for_top_dwell": "已站直，等待保持足够时间后计数",
    "rep_too_fast": "动作时间过短，不计数",
}
_MEASUREMENT_ZH = {
    "knee_angle_deg": "当前膝角（度）", "hip_angle_deg": "当前髋角（度）",
    "hip_y_px": "当前髋部纵坐标（像素）", "leg_scale_px": "当前腿长尺度（像素）",
    "ankle_x_px": "当前脚踝横坐标（像素）", "ankle_y_px": "当前脚踝纵坐标（像素）",
    "baseline_knee_angle_deg": "站姿基线膝角（度）",
    "baseline_hip_angle_deg": "站姿基线髋角（度）",
    "baseline_hip_y_px": "站姿基线髋部纵坐标（像素）",
    "baseline_leg_scale_px": "站姿基线腿长尺度（像素）",
    "knee_drop_deg": "膝角相对站姿减小（度）",
    "hip_drop_deg": "髋角相对站姿减小（度）",
    "hip_descent_ratio": "髋部下降／站姿腿长",
    "ankle_motion_ratio": "脚踝位移／站姿腿长",
}
_CHECK_ZH = {
    "baseline_pose": "站姿可用于校准",
    "start_knee_drop": "起蹲：膝角变化达标",
    "start_hip_drop": "起蹲：髋角变化达标",
    "start_hip_descent": "起蹲：髋部下降达标",
    "moving_down": "已开始下蹲",
    "bottom_knee_drop": "蹲底：膝角变化达标",
    "bottom_hip_drop": "蹲底：髋角变化达标",
    "bottom_hip_descent": "蹲底：髋部下降达标",
    "bottom": "蹲底条件全部达标",
    "top_knee_restored": "站起：膝角恢复",
    "top_hip_restored": "站起：髋角恢复",
    "top_height_restored": "站起：髋部高度恢复",
    "top": "站立条件全部达标",
    "ankle_stable": "脚踝位置稳定",
}
_TIMING_ZH = {
    "calibration_elapsed_s": "校准已持续（秒）",
    "calibration_samples": "校准样本数",
    "reanchor_elapsed_s": "自动更新基线已持续（秒）",
    "reanchor_samples": "自动更新基线样本数",
    "motion_elapsed_s": "本次动作已持续（秒）",
    "bottom_dwell_elapsed_s": "蹲底已保持（秒）",
    "top_dwell_elapsed_s": "站立已保持（秒）",
}
_THRESHOLD_ZH = {
    "min_visibility": "关节最低可见度",
    "baseline_dwell_s": "校准站立保持（秒）",
    "baseline_knee_min_deg": "校准膝角下限（度）",
    "baseline_hip_min_deg": "校准髋角下限（度）",
    "baseline_angle_jitter_deg": "校准角度最大波动（度）",
    "baseline_position_jitter_ratio": "校准位置最大波动／腿长",
    "reanchor_dwell_s": "自动更新基线站稳时长（秒）",
    "reanchor_knee_min_deg": "自动更新基线膝角下限（度）",
    "reanchor_hip_min_deg": "自动更新基线髋角下限（度）",
    "start_knee_drop_deg": "起蹲膝角最小变化（度）",
    "start_hip_drop_deg": "起蹲髋角最小变化（度）",
    "start_hip_drop_ratio": "起蹲髋部最小下降／腿长",
    "bottom_knee_drop_deg": "蹲底膝角最小变化（度）",
    "bottom_hip_drop_deg": "蹲底髋角最小变化（度）",
    "bottom_hip_drop_ratio": "蹲底髋部最小下降／腿长",
    "bottom_dwell_s": "蹲底最短保持（秒）",
    "top_restore_deg": "站起角度允许偏差（度）",
    "top_hip_restore_ratio": "站起髋部高度允许偏差／腿长",
    "top_dwell_s": "站立最短保持（秒）",
    "min_rep_s": "单次动作最短时长（秒）",
    "ankle_motion_max_ratio": "脚踝最大位移／腿长",
    "smoothing_frames": "平滑帧数",
    "max_frame_gap_s": "最大帧间隔（秒）",
    "lost_grace_s": "目标丢失宽限（秒）",
    "switch_after_s": "切换目标等待（秒）",
}


def _labeled(values: dict, labels: dict[str, str]) -> dict:
    return {labels[key]: value for key, value in values.items()}


def _yes_no(value: bool | None) -> str:
    return "暂无数据" if value is None else "是" if value else "否"


def _candidate_zh(candidate: dict) -> dict:
    result = {
        "人体ID": candidate["id"], "边界框（像素）": candidate["bbox"],
        "置信度": candidate.get("score"), "质量分": candidate["quality"],
        "关键关节可读": _yes_no(candidate["readable"]),
    }
    if "side" in candidate:
        result["跟踪侧"] = _SIDE_ZH.get(candidate["side"], "未确定")
    if "present" in candidate:
        result["当前帧可见"] = _yes_no(candidate["present"])
    return result


def _debug_zh(raw: dict) -> dict:
    """Localize dashboard diagnostics; status and Decision Core events stay stable."""
    measurements = raw["measurements"]
    baseline = measurements["baseline"]
    checks = raw["checks"]
    last_error = raw["last_error"]
    if last_error and last_error != "姿态数据必须是 JSON 对象":
        last_error = ("姿态输入不是有效 JSON" if "JSON" in last_error or
                      "Expecting" in last_error else
                      "姿态输入处理失败：" + last_error)
    return {
        "卡片": "深蹲计数", "动作": "深蹲",
        "运行状态": "运行中" if raw["state"] == "running" else "未运行",
        "动作阶段": _PHASE_ZH.get(raw["phase"], "未知阶段"),
        "判定原因": _REASON_ZH.get(raw["reason"], "未知判定原因"),
        "累计次数": raw["count_total"],
        "训练状态": _WORKOUT_STATE_ZH[raw["workout_state"]],
        "训练已持续（秒）": raw["workout_elapsed_s"],
        "离画面已持续（秒）": raw["absence_elapsed_s"],
        "训练总结": raw["summary"],
        "训练计划": raw["plan"],
        "组间阶段": _SET_STATE_ZH.get(raw["set_state"], "未设置"),
        "当前组": raw["set_index"],
        "当前组已完成": raw["set_count"],
        "当前组目标": raw["set_target"],
        "剩余休息（秒）": raw["rest_remaining_s"],
        "会话ID": raw["session_id"], "事件序号": raw["seq"],
        "目标ID": raw["target_id"],
        "目标状态": _TARGET_STATE_ZH.get(raw["target_state"], "未知状态"),
        "跟踪侧": _SIDE_ZH.get(raw["side"], "未确定"),
        "站姿基线已建立": _yes_no(raw["calibrated"]),
        "需要重新站稳": _yes_no(raw["needs_rearm"]),
        "目标上次出现时间": raw["last_seen_time"],
        "输入话题": raw["input_topic"],
        "距上一帧（秒）": raw["last_frame_age_s"],
        "输入已过期": _yes_no(raw["input_stale"]),
        "接收帧数": raw["frames_received"],
        "解析帧数": raw["frames_parsed"],
        "无效帧数": raw["invalid_frames"],
        "最近错误": last_error,
        "当前帧时间": raw["frame_time"],
        "帧间隔（秒）": raw["frame_gap_s"],
        "帧间隔超限": _yes_no(raw["frame_gap_exceeded"]),
        "当前目标": _candidate_zh(raw["selected_target"]),
        "候选人数": raw["candidate_count"],
        "候选人体": [_candidate_zh(item) for item in raw["candidates"]],
        "测量值": {
            **_labeled({key: value for key, value in measurements.items()
                        if key != "baseline"}, _MEASUREMENT_ZH),
            "站姿基线": (_labeled(baseline, _MEASUREMENT_ZH)
                     if baseline is not None else None),
        },
        "判定项": {_CHECK_ZH[key]: ("暂无数据" if value is None else
                              "通过" if value else "未通过")
                for key, value in checks.items()},
        "计时": _labeled(raw["timing"], _TIMING_ZH),
        "阈值": _labeled(raw["thresholds"], _THRESHOLD_ZH),
    }


def _nonnegative_interval(config: dict, name: str, default: float) -> float:
    value = config.get(name, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError(f"{name} must be a nonnegative finite number")
    return float(value)


def _plan_integer(value: object, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer from {minimum} to {maximum}")
    return value


class SquatCounterPlugin:
    """One local squat session, independent of the model's response speed."""

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 *, node_name: str = "bumi_squat_counter"):
        self._namespace = namespace
        self._counter = SquatCounter({
            key: value for key, value in plugin_config.items() if key in DEFAULT_CONFIG
        })
        self._status_interval_s = _nonnegative_interval(
            plugin_config, "status_interval_s", 0.5)
        self._debug_interval_s = _nonnegative_interval(
            plugin_config, "debug_interval_s", 0.5)
        self._min_rep_event_interval_s = _nonnegative_interval(
            plugin_config, "min_rep_event_interval_s", 3.0)
        self._input_stale_after_s = _nonnegative_interval(
            plugin_config, "input_stale_after_s", 1.5)
        explicit_start = plugin_config.get("require_explicit_workout_start", False)
        if not isinstance(explicit_start, bool):
            raise ValueError("require_explicit_workout_start must be a boolean")
        self._require_explicit_workout_start = explicit_start
        coach_mode = plugin_config.get("coach_mode", False)
        if not isinstance(coach_mode, bool):
            raise ValueError("coach_mode must be a boolean")
        self._coach_mode = coach_mode
        self._default_plan = {
            "sets": _plan_integer(plugin_config.get("default_sets", 2),
                                  "default_sets", 1, 5),
            "reps_per_set": _plan_integer(plugin_config.get("default_reps_per_set", 8),
                                          "default_reps_per_set", 1, 20),
            "rest_s": _plan_integer(plugin_config.get("default_rest_s", 90),
                                    "default_rest_s", 15, 180),
        }
        self._auto_finish_absence_s = _nonnegative_interval(
            plugin_config, "auto_finish_absence_s", 6.0)
        self._event_topic = f"/{namespace}/fitness/squat/events"
        self._status_topic = f"/{namespace}/fitness/squat/status"
        self._debug_topic = f"/{namespace}/fitness/squat/debug"
        self._node = Node(node_name)
        executor.add_node(self._node)
        self._event_pub = self._node.create_publisher(
            String, self._event_topic, _OUTPUT_QOS)
        self._status_pub = self._node.create_publisher(
            String, self._status_topic, _OUTPUT_QOS)
        self._debug_pub = self._node.create_publisher(
            String, self._debug_topic, _DEBUG_QOS)
        self._subscription = None
        self._input_topic = ""
        self._lock = threading.RLock()
        self._pending_rep: dict | None = None
        self._last_rep_published_at: float | None = None
        self._last_status_published_at = 0.0
        self._last_debug_published_at = 0.0
        self._last_input_at: float | None = None
        self._frames_received = 0
        self._frames_parsed = 0
        self._invalid_frames = 0
        self._last_error: str | None = None
        self._workout_state = "waiting_start"
        self._workout_started_at: float | None = None
        self._absence_started_at: float | None = None
        self._subject_seen = False
        self._final_summary: dict | None = None
        self._rep_duration_sum = 0.0
        self._rep_duration_count = 0
        self._rep_duration_min: float | None = None
        self._rep_duration_max: float | None = None
        self._coach_plan: dict | None = None
        self._set_state: str | None = None
        self._set_index = 0
        self._set_count = 0
        self._workout_total = 0
        self._set_results: list[dict] = []
        self._set_started_at: float | None = None
        self._rest_until: float | None = None
        self._ready_since: float | None = None
        self._node.create_timer(0.2, self._tick)

    def get_tool(self) -> dict:
        return tool_definition(self._namespace)

    def _topic_out(self) -> list[dict]:
        return topic_out(self._namespace)

    def start(self) -> None:
        # Bundle startup registers the card; the canvas supplies input_topic
        # later through dispatch("start", ...).
        pass

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        with self._lock:
            if action == "start":
                return self._start_locked(args.get("input_topic"))
            if action == "stop":
                self._stop_locked()
                return self._status_locked()
            if action == "reset":
                return self._begin_workout_locked(force=True, plan_args={})
            if action == "begin":
                return self._begin_workout_locked(force=False, plan_args=args)
            if action == "next_set":
                return self._next_set_locked()
            if action == "finish":
                self._finish_workout_locked("voice")
                return self._status_locked()
            if action == "status":
                return self._status_locked()
            if action == "debug":
                return self._debug_locked()
            if action == "info":
                info = self._status_locked()
                info["topic_in"] = [{
                    "format": "sensor/pose2d",
                    "message_type": "std_msgs/msg/String",
                    **({"topic": self._input_topic} if self._input_topic else {}),
                }]
                info["topic_out"] = self._topic_out()
                info["config"] = dict(self._counter.config)
                return info
        return None

    def _start_locked(self, topic: object) -> dict:
        if not isinstance(topic, str) or not topic.strip():
            return {"state": "error", "error": "input_topic is required"}
        topic = topic.strip()
        if self._subscription is not None and topic == self._input_topic:
            return self._status_locked()
        try:
            new_sub = self._node.create_subscription(
                String, topic,
                lambda message, source=topic: self._on_pose(message, source),
                _POSE_QOS,
            )
        except Exception as exc:
            self._last_error = str(exc)
            return {"state": "error", "error": self._last_error}
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
        self._subscription = new_sub
        self._input_topic = topic
        self._counter.reset()
        self._workout_state = "waiting_start"
        self._workout_started_at = None
        self._absence_started_at = None
        self._subject_seen = False
        self._final_summary = None
        self._reset_rep_metrics_locked()
        self._clear_coach_locked()
        if not self._require_explicit_workout_start:
            self._workout_state = "active"
            self._workout_started_at = time.monotonic()
        self._pending_rep = None
        self._last_rep_published_at = None
        self._last_input_at = time.monotonic()
        self._frames_received = 0
        self._frames_parsed = 0
        self._invalid_frames = 0
        self._last_error = None
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)
        return self._status_locked()

    def _stop_locked(self) -> None:
        if self._require_explicit_workout_start and self._workout_state == "active":
            self._finish_workout_locked("card_stopped")
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        self._input_topic = ""
        # If the model was busy, leave it one latest absolute count before
        # stopping; the final count is also available through status.
        if self._pending_rep is not None:
            self._publish_event_locked(self._pending_rep)
            self._pending_rep = None
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)

    def _reset_rep_metrics_locked(self) -> None:
        self._rep_duration_sum = 0.0
        self._rep_duration_count = 0
        self._rep_duration_min = None
        self._rep_duration_max = None

    def _clear_coach_locked(self) -> None:
        self._coach_plan = None
        self._set_state = None
        self._set_index = 0
        self._set_count = 0
        self._workout_total = 0
        self._set_results = []
        self._set_started_at = None
        self._rest_until = None
        self._ready_since = None

    def _begin_workout_locked(self, *, force: bool, plan_args: dict) -> dict:
        if self._subscription is None:
            if not force:
                return {"state": "error", "error": "start the squat card on the canvas first"}
            # Preserve the existing reset API while the canvas card is stopped.
            self._counter.reset()
            self._workout_state = "waiting_start"
            self._workout_started_at = None
            self._absence_started_at = None
            self._subject_seen = False
            self._final_summary = None
            self._pending_rep = None
            self._last_rep_published_at = None
            self._reset_rep_metrics_locked()
            self._clear_coach_locked()
            return self._status_locked()
        if self._workout_state == "active" and not force:
            return self._status_locked()
        if self._coach_mode:
            base = (self._coach_plan if force and self._coach_plan is not None
                    else self._default_plan)
            try:
                plan = {
                    "sets": _plan_integer(plan_args.get("sets", base["sets"]),
                                          "sets", 1, 5),
                    "reps_per_set": _plan_integer(
                        plan_args.get("reps_per_set", base["reps_per_set"]),
                        "reps_per_set", 1, 20),
                    "rest_s": _plan_integer(plan_args.get("rest_s", base["rest_s"]),
                                            "rest_s", 15, 180),
                }
            except ValueError as exc:
                return {"state": "error", "error": str(exc)}
        self._counter.reset()
        self._pending_rep = None
        self._last_rep_published_at = None
        self._workout_state = "active"
        self._workout_started_at = time.monotonic()
        self._absence_started_at = None
        self._subject_seen = False
        self._final_summary = None
        self._reset_rep_metrics_locked()
        self._clear_coach_locked()
        if self._coach_mode:
            self._coach_plan = plan
            self._set_state = "preparing"
            self._set_index = 1
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)
        return self._status_locked()

    def _next_set_locked(self) -> dict:
        if not self._coach_mode:
            return {"state": "error", "error": "next_set requires coach_mode"}
        if self._workout_state != "active":
            return {"state": "error", "error": "no active workout"}
        if self._set_state == "resting":
            # A voice command may arrive just after the deadline but before
            # the 200 ms timer callback. Accept it without a stale prompt.
            if self._rest_remaining_locked() != 0:
                return {"state": "error", "error": "rest period has not finished",
                        "rest_remaining_s": self._rest_remaining_locked()}
            self._set_state = "ready_next"
            self._rest_until = None
        if self._set_state != "ready_next":
            return self._status_locked() if self._set_index > 1 else {
                "state": "error", "error": "next set is not ready"}
        self._set_index += 1
        self._set_count = 0
        self._set_state = "preparing"
        self._set_started_at = None
        self._ready_since = None
        self._pending_rep = None
        self._last_rep_published_at = None
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)
        return self._status_locked()

    def _rest_remaining_locked(self, now: float | None = None) -> int | None:
        if self._rest_until is None:
            return None
        now = time.monotonic() if now is None else now
        return max(0, math.ceil(self._rest_until - now))

    def _advance_rest_locked(self, now: float) -> None:
        if (not self._coach_mode or self._workout_state != "active"
                or self._set_state != "resting" or self._rest_until is None
                or now < self._rest_until):
            return
        self._set_state = "ready_next"
        self._rest_until = None
        self._publish_event_locked({
            "type": "rest_finished", "session_id": self._counter.session_id,
            "set_index": self._set_index, "next_set_index": self._set_index + 1,
            "count_total": self._workout_total,
        })
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)

    def _coach_maybe_ready_locked(self, now: float) -> None:
        if (not self._coach_mode or self._workout_state != "active"
                or self._set_state != "preparing"):
            return
        snapshot = self._counter.snapshot()
        if (snapshot["target_state"] != "tracking" or not snapshot["calibrated"]
                or snapshot["state"] != "top"):
            self._ready_since = None
            return
        if self._ready_since is None:
            self._ready_since = now
            return
        if now - self._ready_since < 0.4:
            return
        self._set_state = "in_set"
        self._set_started_at = now
        self._ready_since = None
        self._publish_event_locked({
            "type": "set_ready", "session_id": snapshot["session_id"],
            "seq": snapshot["seq"], "set_index": self._set_index,
            "sets": self._coach_plan["sets"],
            "set_target": self._coach_plan["reps_per_set"],
            "count_total": self._workout_total,
        })

    def _coach_handle_event_locked(self, event: dict, now: float) -> None:
        kind = event.get("type")
        if kind == "rep_completed":
            if self._set_state != "in_set":
                return
            self._set_count += 1
            self._workout_total += 1
            self._record_rep_duration_locked(event)
            progress = {
                **event, "count_total": self._workout_total,
                "set_index": self._set_index, "set_count": self._set_count,
                "set_target": self._coach_plan["reps_per_set"],
                "sets": self._coach_plan["sets"],
            }
            if self._set_count < self._coach_plan["reps_per_set"]:
                # One or two spoken progress milestones per set keep a slow
                # Decision Core from queuing obsolete count announcements.
                midpoint = math.ceil(self._coach_plan["reps_per_set"] / 2)
                if self._set_count in (1, midpoint):
                    self._publish_event_locked(progress)
                return
            self._set_results.append({
                "set_index": self._set_index,
                "target": self._coach_plan["reps_per_set"],
                "count": self._set_count,
                "duration_s": round(max(0.0, now - self._set_started_at), 1),
            })
            self._pending_rep = None
            if self._set_index == self._coach_plan["sets"]:
                self._finish_workout_locked("plan_completed", announce=True)
                return
            self._set_state = "resting"
            self._rest_until = now + self._coach_plan["rest_s"]
            self._publish_event_locked({
                **progress, "type": "set_completed",
                "rest_s": self._coach_plan["rest_s"],
                "next_set_index": self._set_index + 1,
            })
            return
        if kind in ("target_lost", "target_reacquired"):
            self._publish_event_locked({
                **event, "set_index": self._set_index,
                "set_state": self._set_state,
                "count_total": self._workout_total,
            })
        elif kind == "ready_to_squat" and self._set_state == "in_set":
            self._publish_event_locked({
                **event, "set_index": self._set_index,
                "set_state": self._set_state,
                "count_total": self._workout_total,
            })

    def _record_rep_duration_locked(self, event: dict) -> None:
        duration = event.get("rep_duration_s")
        if isinstance(duration, (int, float)) and math.isfinite(duration):
            self._rep_duration_sum += duration
            self._rep_duration_count += 1
            self._rep_duration_min = (duration if self._rep_duration_min is None
                                      else min(self._rep_duration_min, duration))
            self._rep_duration_max = (duration if self._rep_duration_max is None
                                      else max(self._rep_duration_max, duration))

    def _finish_workout_locked(self, reason: str, *,
                               previous_session: dict | None = None,
                               announce: bool = False) -> None:
        if self._workout_state != "active":
            return
        now = time.monotonic()
        counter = previous_session or self._counter.snapshot()
        count = (self._workout_total if self._coach_mode and self._coach_plan
                 else counter["count_total"])
        summary = {
            "session_id": counter["session_id"],
            "target_id": counter["target_id"],
            "count_total": count,
            "duration_s": round(max(0.0, now - self._workout_started_at), 1),
            "end_reason": reason,
            "mean_rep_duration_s": (
                round(self._rep_duration_sum / self._rep_duration_count, 2)
                if self._rep_duration_count else None),
            "fastest_rep_s": (round(self._rep_duration_min, 2)
                              if self._rep_duration_min is not None else None),
            "slowest_rep_s": (round(self._rep_duration_max, 2)
                              if self._rep_duration_max is not None else None),
        }
        if self._coach_mode and self._coach_plan is not None:
            summary.update({
                "plan": dict(self._coach_plan),
                "completed_sets": len(self._set_results),
                "set_results": [dict(item) for item in self._set_results],
                "current_set_index": self._set_index,
                "current_set_count": self._set_count,
                "goal_reached": reason == "plan_completed",
            })
        self._final_summary = summary
        self._workout_state = "finished"
        if self._coach_mode:
            self._set_state = "finished"
            self._rest_until = None
        self._absence_started_at = None
        self._pending_rep = None
        self._counter.reset()
        if announce:
            self._publish_event_locked({
                "type": "workout_finished", "session_id": summary["session_id"],
                "count_total": count, "end_reason": reason, "summary": dict(summary),
            })
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)

    def _check_auto_finish_locked(self, now: float) -> None:
        if (not self._require_explicit_workout_start
                or self._workout_state != "active" or not self._subject_seen
                or self._absence_started_at is None
                or self._auto_finish_absence_s == 0):
            return
        if (self._last_input_at is None
                or now - self._last_input_at >= self._input_stale_after_s):
            # Camera or pose interruption is not evidence that the user left.
            self._absence_started_at = None
            return
        if now - self._absence_started_at >= self._auto_finish_absence_s:
            self._finish_workout_locked("left_frame", announce=True)

    def _on_pose(self, message: String, source: str) -> None:
        with self._lock:
            # A queued callback from a previous subscription must not mutate
            # the new session after switching inputs or stopping.
            if self._subscription is None or source != self._input_topic:
                return
            self._frames_received += 1
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("姿态数据必须是 JSON 对象")
                previous_frame_time = self._counter.last_frame_time
                events = (self._counter.process(payload)
                          if self._workout_state == "active" else [])
                fresh_frame = (self._workout_state != "active"
                               or self._counter.last_frame_time != previous_frame_time)
                self._frames_parsed += 1
            except (ValueError, TypeError, KeyError) as exc:
                self._invalid_frames += 1
                self._last_error = str(exc)
                if self._invalid_frames == 1 or self._invalid_frames % 100 == 0:
                    self._node.get_logger().warn(
                        f"Squat counter ignored {self._invalid_frames} invalid frame(s): {exc}")
                return
            except Exception as exc:
                self._invalid_frames += 1
                self._last_error = str(exc)
                self._node.get_logger().error(f"Squat counter frame error: {exc}")
                return
            now = time.monotonic()
            if fresh_frame:
                self._last_input_at = now
            self._last_error = None
            if self._workout_state == "active" and fresh_frame:
                snapshot = self._counter.snapshot()
                target_state = snapshot["target_state"]
                if target_state == "tracking":
                    self._subject_seen = True
                    self._absence_started_at = None
                elif (target_state == "missing" and self._subject_seen
                      and snapshot["candidate_count"] == 0):
                    if self._absence_started_at is None:
                        self._absence_started_at = now
                else:
                    self._absence_started_at = None
            for event in events:
                if (self._require_explicit_workout_start
                        and event.get("type") == "target_changed"):
                    self._finish_workout_locked("target_changed", previous_session={
                        "session_id": event["previous_session_id"],
                        "target_id": event["previous_target_id"],
                        "count_total": event["previous_count_total"],
                    }, announce=True)
                    break
                self._handle_event_locked(event, now)
            if fresh_frame:
                self._coach_maybe_ready_locked(now)
            self._check_auto_finish_locked(now)
            self._publish_status_locked(now)
            self._publish_debug_locked(now)

    def _handle_event_locked(self, event: dict, now: float) -> None:
        if self._coach_mode:
            self._coach_handle_event_locked(event, now)
            return
        kind = event.get("type")
        if kind == "rep_completed":
            self._record_rep_duration_locked(event)
            if (self._last_rep_published_at is None
                    or now - self._last_rep_published_at
                    >= self._min_rep_event_interval_s):
                self._publish_event_locked(event)
                self._last_rep_published_at = now
                self._pending_rep = None
            else:
                # Keep only the newest absolute count during a busy model turn.
                self._pending_rep = event
            return
        if kind in ("session_started", "target_changed", "target_lost"):
            self._pending_rep = None
        if kind in ("session_started", "target_changed"):
            self._last_rep_published_at = None
        self._publish_event_locked(event)

    def _publish_event_locked(self, event: dict) -> None:
        payload = {"source": "squat_counter", "exercise": "squat",
                   "priority": 1, **event}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self._event_pub.publish(message)

    def _status_locked(self) -> dict:
        counter = self._counter.snapshot()
        if self._coach_mode and self._workout_state == "active":
            counter["count_total"] = self._workout_total
        if self._final_summary is not None:
            counter.update({
                "session_id": self._final_summary["session_id"],
                "count_total": self._final_summary["count_total"],
                "target_id": self._final_summary["target_id"],
                "state": "finished", "target_state": "none",
                "calibrated": False,
            })
        age = (None if self._last_input_at is None
               else max(0.0, time.monotonic() - self._last_input_at))
        now = time.monotonic()
        return {
            "source": "squat_counter",
            "exercise": "squat",
            "state": "running" if self._subscription is not None else "idle",
            "phase": counter.pop("state"),
            **counter,
            "workout_state": self._workout_state,
            "workout_elapsed_s": (
                round(max(0.0, now - self._workout_started_at), 1)
                if self._workout_state == "active" and self._workout_started_at is not None
                else self._final_summary["duration_s"] if self._final_summary else None),
            "absence_elapsed_s": (
                round(max(0.0, now - self._absence_started_at), 1)
                if self._absence_started_at is not None else None),
            "summary": dict(self._final_summary) if self._final_summary else None,
            "plan": dict(self._coach_plan) if self._coach_plan else None,
            "set_state": self._set_state,
            "set_index": self._set_index if self._coach_plan else None,
            "set_count": self._set_count if self._coach_plan else None,
            "set_target": (self._coach_plan["reps_per_set"]
                           if self._coach_plan else None),
            "completed_sets": len(self._set_results) if self._coach_plan else None,
            "rest_remaining_s": self._rest_remaining_locked(now),
            "input_topic": self._input_topic or None,
            "last_frame_age_s": None if age is None else round(age, 3),
            "input_stale": (self._subscription is not None and age is not None
                            and age >= self._input_stale_after_s),
            "frames_received": self._frames_received,
            "frames_parsed": self._frames_parsed,
            "invalid_frames": self._invalid_frames,
            "last_error": self._last_error,
        }

    def _debug_locked(self) -> dict:
        # Only dashboard diagnostics are localized. Keep status/events stable.
        # Do not include raw pose keypoints.
        diagnostics = dict(self._counter.diagnostics())
        diagnostics.pop("state", None)
        return _debug_zh({**self._status_locked(), **diagnostics})

    def _publish_status_locked(self, now: float | None = None,
                               *, force: bool = False) -> None:
        now = time.monotonic() if now is None else now
        if (not force and now - self._last_status_published_at
                < self._status_interval_s):
            return
        message = String()
        message.data = json.dumps(self._status_locked(), ensure_ascii=False,
                                  allow_nan=False)
        self._status_pub.publish(message)
        self._last_status_published_at = now

    def _publish_debug_locked(self, now: float | None = None,
                              *, force: bool = False) -> None:
        now = time.monotonic() if now is None else now
        if (not force and now - self._last_debug_published_at
                < self._debug_interval_s):
            return
        message = String()
        message.data = json.dumps(self._debug_locked(), ensure_ascii=False,
                                  allow_nan=False)
        self._debug_pub.publish(message)
        self._last_debug_published_at = now

    def _tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            self._check_auto_finish_locked(now)
            self._advance_rest_locked(now)
            if (self._pending_rep is not None
                    and (self._last_rep_published_at is None
                         or now - self._last_rep_published_at
                         >= self._min_rep_event_interval_s)):
                self._publish_event_locked(self._pending_rep)
                self._pending_rep = None
                self._last_rep_published_at = now
            if self._subscription is not None:
                self._publish_status_locked(now)
                self._publish_debug_locked(now)
