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
                self._pending_rep = None
                self._last_rep_published_at = None
                self._counter.reset()
                self._publish_status_locked(force=True)
                self._publish_debug_locked(force=True)
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
                events = self._counter.process(payload)
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
            self._last_input_at = now
            self._last_error = None
            for event in events:
                self._handle_event_locked(event, now)
            self._publish_status_locked(now)
            self._publish_debug_locked(now)

    def _handle_event_locked(self, event: dict, now: float) -> None:
        kind = event.get("type")
        if kind == "rep_completed":
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
        age = (None if self._last_input_at is None
               else max(0.0, time.monotonic() - self._last_input_at))
        return {
            "source": "squat_counter",
            "exercise": "squat",
            "state": "running" if self._subscription is not None else "idle",
            "phase": counter.pop("state"),
            **counter,
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
