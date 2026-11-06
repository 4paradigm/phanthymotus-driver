"""Mac perception-only MCP card. No robot commands or raw images are exposed."""
from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pose_check import POSES, check_pose, compare_squat_height, squat_height_metrics
from pose_session import PoseSession

POSE_CHECK_VERSION = "socket-bridge-20261007"
POSE_RESULT_TOPIC = os.environ.get("POSE_RESULT_TOPIC", "/nvidia_desktop/pose_check/result")


def tool_definition():
    return {
        "name": "pose_check", "type": "processor", "multiInstance": False,
        "topic_in": [{"format": "image/jpeg", "desc": "人体 RGB 图像输入"}],
        "topic_out": [{"topic": POSE_RESULT_TOPIC,
                        "format": "data/json",
                        "desc": "事件驱动 JSON；只在任务开始、动作完成、校准/进度变化、结束、取消或超时时发布，含 session_id、event、count、target 和 narration"}],
        "description": "办公室全身跟练和持续姿态检查。相机逐帧识别，但 topic_out 只发布一次性事件，不逐帧刷屏。check(squat) 先站直校准，再根据身高变化与关节判定静态下蹲；稳定检测到和离开动作时各发布一次 pose_detected/pose_lost。begin 发布 task_started；每完成一次 repetitions 发布 rep_completed；完成整组发布 step_completed。",
        "inputSchema": {
            "type": "object", "required": ["action"],
            "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info", "check", "begin", "status", "cancel"]},
                "pose": {"type": "string", "enum": list(POSES)},
                "hold_seconds": {"type": "number", "minimum": 0, "maximum": 30, "default": 0,
                                  "description": "上肢每次动作的可选保持时间；默认 0，稳定到位即计一次。深蹲忽略此字段"},
                "repetitions": {"type": "integer", "minimum": 1, "maximum": 20, "default": 1},
                "timeout_seconds": {"type": "number", "minimum": 1, "maximum": 300, "default": 45},
                "session_id": {"type": "string", "description": "begin 返回的 ID，防止读取或取消其他跟练"},
            },
            "x-action-params": {
                "start": {"params": [], "description": "启用动作检查"},
                "stop": {"params": [], "description": "停止检查并取消当前跟练；相机由本地进程管理"},
                "info": {"params": [], "description": "查询卡片能力、输出话题与相机新鲜度；不会向结果话题发布消息"},
                "check": {"params": ["pose"], "description": "设置并持续检查目标动作；深蹲请先站直约 0.4 秒校准，再保持蹲姿；稳定检测到或动作离开时各发一次事件，不表示完成一次蹲起"},
                "begin": {"params": ["pose", "hold_seconds", "repetitions", "timeout_seconds"], "description": "开始一次跟练；所有动作都按 repetitions 计数。上肢每次需稳定到位、放下后才能计下一次；下蹲需先站直再蹲下站起"},
                "status": {"params": ["session_id"], "description": "按需查询当前跟练进度；只有新事件才向结果话题发布消息"},
                "cancel": {"params": [], "description": "直接取消当前跟练，或退出持续 check；单实例卡片无需 session_id"},
            },
        },
    }


class PoseCard:
    STABILITY_WINDOW = 5
    STABILITY_REQUIRED = 3
    POSE_LABELS = {
        "hands_up": "双手举高",
        "arms_open": "双臂展开",
        "one_hand_up": "单手举起",
        "squat": "深蹲",
    }

    def __init__(self, clock=time.monotonic, source="camera"):
        self.lock = threading.RLock()
        self.clock = clock
        self.enabled = True
        self.points = {}
        self.frame_id = 0
        self.frame_at = None
        self.error = None
        self.session = None
        self.source = source
        self._last_session_state = None
        self._last_session_count = 0
        self._last_session_calibrated = False
        self._calibration_event_emitted = False
        self._last_correction_key = None
        self._milestone_emitted = False
        self._pending_events = []
        self.target_pose = None
        self.check_mode = False
        self._last_check_stable = None
        self._match_history = deque(maxlen=self.STABILITY_WINDOW)
        self._latest_target_result = None
        self._check_squat_baseline = None
        self._check_squat_standing_since = None

    def _reset_stability(self):
        self._match_history.clear()
        self._latest_target_result = None
        self._last_check_stable = None
        self._check_squat_baseline = None
        self._check_squat_standing_since = None

    def _evaluate_squat_check(self, raw, points):
        """Classify a held squat relative to a standing image baseline."""
        result = dict(raw)
        result["calibrated"] = self._check_squat_baseline is not None
        if not raw.get("detected"):
            if self._check_squat_baseline is None:
                self._check_squat_standing_since = None
                return result
            # Once standing is calibrated, knees may be occluded at the squat
            # bottom while shoulders, hips and ankles remain trackable.
            height_points = squat_height_metrics(points)
            if height_points is None:
                return result
            result.update(height_points, detected=True, matched=False,
                          status="almost", phase="height_only",
                          reason="knee_unreliable_height_available")
        if self._check_squat_baseline is None:
            if raw.get("phase") == "standing" and raw.get("body_height") is not None:
                now = self.clock()
                if self._check_squat_standing_since is None:
                    self._check_squat_standing_since = now
                elif now - self._check_squat_standing_since >= 0.4:
                    self._check_squat_baseline = dict(raw)
            else:
                self._check_squat_standing_since = None
            calibrated = self._check_squat_baseline is not None
            result.update(calibrated=calibrated, matched=False,
                          status="almost" if calibrated else "retry", score=0.0,
                          feedback=("站姿已校准，请下蹲" if calibrated else
                                    "请让双肩、髋部和脚踝入框，站直片刻完成身高校准"
                                    if raw.get("body_height") is None else
                                    "请先站直，保持片刻完成身高校准"))
            if not calibrated:
                result["reason"] = "height_baseline_required"
            return result
        height = compare_squat_height(result, self._check_squat_baseline)
        result.update(height)
        if raw.get("matched") or height["height_matched"]:
            result.update(matched=True, status="completed", score=1.0,
                          feedback="已完成",
                          squat_evidence=("knee_angle" if raw.get("matched") else "body_height"))
            if height["height_matched"] and result.get("phase") != "down":
                result["raw_phase"] = result.get("phase")
                result["phase"] = "down"
        elif result.get("phase") == "standing":
            result["feedback"] = "已站直，请下蹲"
        elif result.get("phase") == "height_only":
            result["feedback"] = "膝盖暂时看不清，请保持全身入框"
        return result

    def _record_check_transition(self, result):
        """Emit one event when continuous check crosses its stable boundary."""
        if not self.check_mode or self.session is not None:
            return
        stable = bool(result.get("stable"))
        previous = self._last_check_stable
        if previous is None:
            self._last_check_stable = stable
            if not stable:
                return
        elif stable == previous:
            return
        self._last_check_stable = stable
        pose = self.target_pose
        label = self.POSE_LABELS.get(pose, pose)
        detected = stable
        self._pending_events.append({
            "event": "pose_detected" if detected else "pose_lost",
            "pose": pose,
            "status": "detected" if detected else "retry",
            "matched": detected,
            "narration": f"检测到{label}" if detected else f"{label}已离开动作",
        })

    def _evaluate_target(self, points, pose):
        """Return one selected pose with a small temporal completion gate."""
        raw = check_pose(points, pose)
        if pose == "squat" and self.check_mode:
            raw = self._evaluate_squat_check(raw, points)
        self._match_history.append(bool(raw.get("detected") and raw.get("matched")))
        matches = sum(self._match_history)
        stable = (len(self._match_history) >= self.STABILITY_REQUIRED and
                  matches >= self.STABILITY_REQUIRED)
        result = dict(raw)
        result.update({
            "raw_matched": bool(raw.get("matched")),
            "stable": stable,
            "stable_window": len(self._match_history),
            "stable_matches": matches,
            "stable_required": self.STABILITY_REQUIRED,
        })
        if raw.get("matched") and not stable:
            result["matched"] = False
            result["status"] = "almost"
            result["feedback"] = "动作已到位，请保持稳定"
        elif stable:
            result["matched"] = True
            result["status"] = "completed"
            result["feedback"] = "已完成"
        self._latest_target_result = result
        self._record_check_transition(result)
        return result

    def _record_session_result(self, result):
        """Turn state transitions into one-shot Activity/TTS events."""
        count = int(result.get("progress", {}).get("repetitions", 0))
        state = result.get("state")
        previous = self._last_session_state
        duration = result.get("progress", {}).get("last_round_seconds")
        progress = result.get("progress", {})
        calibrated = bool(progress.get("calibrated", True))
        if (self.session.pose == "squat" and state == "running" and
                not calibrated):
            self._maybe_emit_calibration_correction(result)
        if (self.session.pose == "squat" and calibrated and
                not self._calibration_event_emitted):
            self._pending_events.append({
                "event": "calibration_completed",
                "pose": self.session.pose,
                "narration": "站直校准完成，开始计时。请完成深蹲。",
            })
            self._calibration_event_emitted = True
        if count > self._last_session_count:
            duration_text = f"，用时 {duration:.1f} 秒" if isinstance(duration, (int, float)) else ""
            label = self.POSE_LABELS.get(self.session.pose, self.session.pose)
            self._pending_events.append({
                "event": "rep_completed",
                "pose": self.session.pose,
                "count": count,
                "target": self.session.target,
                "duration_seconds": duration,
                "narration": f"第 {count} 次{label}完成{duration_text}",
            })
            midpoint = math.ceil(self.session.target / 2)
            if (self.session.target > 1 and count >= midpoint and
                    not self._milestone_emitted):
                self._pending_events.append({
                    "event": "progress_milestone",
                    "pose": self.session.pose,
                    "count": count,
                    "target": self.session.target,
                    "narration": (f"{label}已完成一半，{count} 次，还剩 "
                                   f"{self.session.target - count} 次"),
                })
                self._milestone_emitted = True
        if state == "completed" and previous != "completed":
            label = self.POSE_LABELS.get(self.session.pose, self.session.pose)
            self._pending_events.append({
                "event": "step_completed",
                "task_completed": True,
                "pose": self.session.pose,
                "count": count,
                "target": self.session.target,
                "duration_seconds": result.get("progress", {}).get("elapsed_seconds"),
                "narration": (
                    f"深蹲完成，共 {count} 次，总用时 "
                    f"{result.get('progress', {}).get('elapsed_seconds', 0):.1f} 秒"
                    if self.session.pose == "squat"
                    else f"{label}完成，用时 {duration:.1f} 秒"
                    if isinstance(duration, (int, float)) else f"{label}完成"
                ),
            })
        if state == "timed_out" and previous != "timed_out":
            self._pending_events.append({
                "event": "session_timed_out",
                "task_completed": False,
                "pose": self.session.pose,
                "count": count,
                "target": self.session.target,
                "narration": (f"本轮{self.POSE_LABELS.get(self.session.pose, self.session.pose)}超时，"
                              f"完成了 {count} 次，还差 {max(0, self.session.target-count)} 次"),
            })
        if state == "cancelled" and previous != "cancelled":
            self._pending_events.append({
                "event": "session_cancelled",
                "task_completed": False,
                "pose": self.session.pose,
                "count": count,
                "target": self.session.target,
                "narration": "本轮动作已停止",
            })
        self._last_session_count = count
        self._last_session_calibrated = calibrated
        self._last_session_state = state
        return result

    def _maybe_emit_calibration_correction(self, result):
        """Give one throttled, actionable framing/standing prompt."""
        reason = result.get("reason")
        if reason in {"missing_keypoints", "transient_keypoint_loss"}:
            key = "frame"
            narration = "请后退一点，确保头、髋、膝盖和脚踝完整入框。"
        elif reason == "camera_stale":
            key = "camera"
            narration = "请保持相机画面稳定，确保全身在画面内。"
        else:
            key = "standing"
            narration = "请站直，双脚分开并保持不动，准备校准。"
        # A persistent framing problem is one state, not a timer that should
        # generate a new decision-core event every few seconds.
        if self._last_correction_key == key:
            return
        self._last_correction_key = key
        self._pending_events.append({
            "event": "correction",
            "pose": "squat",
            "status": "retry",
            "reason": reason or "standing_required",
            "narration": narration,
        })

    def ingest(self, points, captured_at=None):
        with self.lock:
            self.points = copy.deepcopy(points)
            self.frame_at = self.clock() if captured_at is None else captured_at
            self.frame_id += 1
            self.error = None
            if self.target_pose:
                self._evaluate_target(points, self.target_pose)
            if self.enabled and self.session:
                session_points = points
                # Upper-body holds and repetitions start only after the 3-of-5
                # stability gate. Raw points still reach the session while it
                # waits for a release, so a held pose cannot count twice.
                if self.session.pose != "squat":
                    stable = bool(self._latest_target_result and
                                  self._latest_target_result.get("stable"))
                    result = self.session.update(
                        session_points if self.clock()-self.frame_at <= PoseSession.MAX_GAP else {},
                        self.frame_id, stable_match=stable)
                else:
                    result = self.session.update(
                        session_points if self.clock()-self.frame_at <= PoseSession.MAX_GAP else {},
                        self.frame_id)
                return self._record_session_result(result)
            return None

    def selected_result(self):
        with self.lock:
            return copy.deepcopy(self._latest_target_result)

    def selected_observation(self):
        with self.lock:
            return {
                "pose": self.target_pose,
                "result": copy.deepcopy(self._latest_target_result),
            }

    def drain_events(self):
        """Return one-shot events for the canvas/decision core."""
        with self.lock:
            events, self._pending_events = self._pending_events, []
            return events

    def fail(self, error):
        with self.lock:
            self.error = error
            self.frame_at = None
            self.points = {}
            self.frame_id += 1
            if self.session:
                self.session.update({}, self.frame_id)

    def dispatch(self, action, args):
        with self.lock:
            age = None if self.frame_at is None else self.clock() - self.frame_at
            fresh = age is not None and age <= 0.6 and not self.error
            if action == "info":
                return {"state": "idle" if not self.enabled else "ready" if fresh else "waiting_for_camera",
                        "version": POSE_CHECK_VERSION,
                        "topic_out": tool_definition()["topic_out"],
                        "source": self.source, "frames": self.frame_id, "fresh": fresh,
                        "frame_age_ms": None if age is None else round(age*1000),
                        "error": self.error, "target_pose": self.target_pose,
                        "check_calibrated": self._check_squat_baseline is not None,
                        "stable_window": self.STABILITY_WINDOW,
                        "stable_required": self.STABILITY_REQUIRED,
                        "supported_poses": list(POSES)}
            if action == "start":
                self.enabled = True
                return self.dispatch("info", {})
            if action == "stop":
                self.enabled = False
                if self.session:
                    self.session.cancel()
                self._last_session_state = None
                self._last_session_count = 0
                self._last_session_calibrated = False
                self._calibration_event_emitted = False
                self._last_correction_key = None
                self._milestone_emitted = False
                self._pending_events.clear()
                self.target_pose = None
                self.check_mode = False
                self._reset_stability()
                return {"state": "idle"}
            if action in ("status", "cancel"):
                if not self.session:
                    if action == "cancel" and self.check_mode:
                        pose = self.target_pose
                        self.check_mode = False
                        self.target_pose = None
                        self._reset_stability()
                        self._pending_events.clear()
                        return {"state": "idle", "event": "check_cancelled", "pose": pose}
                    return {"error": "no_session"}
                if action == "status" and args.get("session_id") != self.session.session_id:
                    return {"error": "session_id_mismatch"}
                result = self.session.cancel() if action == "cancel" else self.session.status()
                result = self._record_session_result(result)
                if self.error:
                    result["error"] = self.error
                events = self.drain_events()
                if events:
                    result["events"] = events
                    result["narration"] = events[-1]["narration"]
                if action == "cancel":
                    # A terminal session (timed_out/completed) still needs to
                    # be explicitly dismissed before a fresh begin. Keep the
                    # cancellation result above, then clear all session state.
                    self.session = None
                    self.target_pose = None
                    self.check_mode = False
                    self._last_session_state = None
                    self._last_session_count = 0
                    self._last_session_calibrated = False
                    self._calibration_event_emitted = False
                    self._last_correction_key = None
                    self._milestone_emitted = False
                    self._reset_stability()
                return result
            if action not in ("begin", "check"):
                return {"error": "unsupported_action"}
            if not self.enabled:
                return {"error": "card_stopped", "status": "retry"}
            if not fresh:
                return {"error": self.error or "camera_no_fresh_data", "status": "retry",
                        "matched": False, "feedback": "请检查 Mac 相机画面"}
            pose = args.get("pose")
            if pose not in POSES:
                return {"error": "unsupported_pose", "supported_poses": list(POSES)}
            if action == "check":
                if self.target_pose != pose:
                    self.target_pose = pose
                    self._reset_stability()
                self.check_mode = True
                result = self._evaluate_target(self.points, pose)
                events = self.drain_events()
                if events:
                    result["events"] = events
                    result["narration"] = events[-1]["narration"]
                return result
            if self.session and self.session.status()["state"] == "running":
                return {"error": "session_busy", "session_id": self.session.session_id}
            try:
                self.target_pose = pose
                self.check_mode = False
                self._reset_stability()
                self.session = PoseSession(pose, args.get("hold_seconds", 0),
                                           args.get("repetitions", 1), args.get("timeout_seconds", 45),
                                           clock=self.clock)
                self._last_session_state = "running"
                self._last_session_count = 0
                self._last_session_calibrated = False
                self._calibration_event_emitted = False
                self._last_correction_key = None
                self._milestone_emitted = False
                self._pending_events.clear()
                self._pending_events.append({
                    "event": "task_started",
                    "pose": pose,
                    "count": 0,
                    "target": self.session.target,
                    "narration": ("深蹲任务开始，请先站直，准备校准。"
                                  if pose == "squat" else
                                  f"{self.POSE_LABELS.get(pose, pose)}任务开始，请开始动作。"),
                })
            except ValueError as exc:
                return {"error": "invalid_arguments", "detail": str(exc)}
            # Begin never evaluates the frame captured before the request.
            result = self._record_session_result(self.session.status())
            events = self.drain_events()
            if events:
                result["events"] = events
                result["narration"] = events[-1]["narration"]
            return result


def make_server(card, host="127.0.0.1", port=15740, on_dispatch=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, status, body):
            data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/health":
                self.reply(200, card.dispatch("info", {}))
            else:
                self.reply(404, {"error": "not_found"})

        def do_POST(self):
            if self.path != "/mcp":
                self.reply(404, {"error": "not_found"})
                return
            rid = None
            try:
                self.connection.settimeout(5)
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 16384:
                    raise ValueError("invalid request length")
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError("expected object")
                rid = request.get("id")
                method = request.get("method")
                if method == "initialize":
                    result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                              "serverInfo": {"name": "office-pose-check", "version": "0.1.0"}}
                elif method == "tools/list":
                    result = {"tools": [tool_definition()]}
                elif method == "tools/call":
                    params = request.get("params", {})
                    if not isinstance(params, dict) or params.get("name") != "pose_check":
                        raise ValueError("unknown tool")
                    args = params.get("arguments", {})
                    if not isinstance(args, dict):
                        raise ValueError("arguments must be an object")
                    value = card.dispatch(args.get("action", "check"), args)
                    if on_dispatch is not None:
                        on_dispatch(args.get("action", "check"), value)
                    result = {"content": [{"type": "text", "text": json.dumps(value, ensure_ascii=False)}]}
                else:
                    self.reply(200, {"jsonrpc": "2.0", "id": rid,
                                     "error": {"code": -32601, "message": "method not found"}})
                    return
                self.reply(200, {"jsonrpc": "2.0", "id": rid, "result": result})
            except (ValueError, TypeError, TimeoutError) as exc:
                self.reply(400, {"jsonrpc": "2.0", "id": rid,
                                 "error": {"code": -32602, "message": str(exc)}})

    return ThreadingHTTPServer((host, port), Handler)
