"""Bumi routine scheduler and single-card workbench; no ROS/SDK imports.

The SDK has no reliable physical-completion signal. Operator mode waits for
explicit confirmation. Timed-exit mode executes an explicit or previously
operator-calibrated playback window,
then exits; it never claims the recording physically finished. Simulation timing
never runs on the physical backend. A shared action lock serializes the routine with
all call-shaped body commands; streaming velocity is suppressed while reserved.
"""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
from uuid import uuid4

WALKING_MODE = 2
PLAY_TEACH_MODE = 23
PROTECTION_MODE = 26
MAX_PLAYS = 200
MAX_TASK_S = 3600
POLL_S = 0.05
TERMINAL = {"completed", "cancelled", "error"}


def notify_completion(action_id, state, result):
    """One ACP terminal POST, using the same endpoint as other Bumi actions."""
    import os as _os
    import ssl
    import urllib.request
    url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({"action_id": action_id, "status": state,
                          "result": result, "tool": "exercise_routine",
                          "ts": time.time()}).encode()
    request = urllib.request.Request(
        url + "/api/acp/complete", data=payload,
        headers={"Content-Type": "application/json"}, method="POST")
    context = ssl._create_unverified_context() if url == "https://localhost:15678" else None
    try:
        with urllib.request.urlopen(request, timeout=5, context=context) as response:
            response.read()
    except Exception as exc:
        print(f"[exercise_routine] ACP callback failed for {action_id}: {exc}", flush=True)


class RoutineError(ValueError):
    pass


class Cancelled(Exception):
    pass


def _integer(value, field, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise RoutineError(f"{field} 必须是 {minimum}–{maximum} 的整数")
    return value


def _seconds(value, field, maximum=600, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (float, int))
            or not 0 <= value <= maximum or not math.isfinite(value)
            or (positive and value == 0)):
        raise RoutineError(f"{field} 必须是有限秒数，范围 {'大于 0 至' if positive else '0–'}{maximum}")
    return float(value)


def _canvas_boolean(value, field, default):
    """Accept booleans or explicit form text, never Python string truthiness."""
    if type(value) is bool:
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if not text:
            return default
        if text in {"true", "false"}:
            return text == "true"
    raise RoutineError(f"{field} 必须是 true 或 false（不要加引号），留空使用默认值")


def _canvas_number(value, field, *, default, integer=False):
    if isinstance(value, str):
        value = value.strip()
        if not value:
            return default
        try:
            value = int(value) if integer else float(value)
        except ValueError as exc:
            raise RoutineError(f"{field} 需要{'整数' if integer else '数字'}") from exc
    return value


def normalize_canvas_args(action, args):
    """Normalize the canvas text boxes at the public boundary only.

    The plan's nested counts/types and physical-motion guards stay strict.
    Blank optional fields default to simulation and no confirmation.
    """
    result = dict(args)
    if action in {"validate", "run", "validate_plan", "run_plan"}:
        plan = result.get("plan")
        if isinstance(plan, str) and plan.strip():
            if len(plan) > 65536:
                raise RoutineError("plan JSON 文本不能超过 65536 个字符")
            try:
                plan = json.loads(plan)
            except ValueError as exc:
                raise RoutineError(f"plan 需要合法 JSON 对象，请仅粘贴花括号内的完整计划：{exc}") from exc
            if not isinstance(plan, dict):
                raise RoutineError("plan JSON 必须是对象，不能是列表、字符串或空值")
            result["plan"] = plan
        if plan is None or (isinstance(plan, str) and not plan.strip()):
            if action.endswith("_plan"):
                raise RoutineError("高级入口需要 plan；普通运行请在动作列表中选择")
            def number(key, default, integer=False):
                return _canvas_number(result.get(key, default), key, default=default, integer=integer)
            motion = result.get("motion")
            if isinstance(motion, str):
                motion = motion.strip()
            step = {"motion": motion, "play_count": number("play_count", 1, True),
                    "rest_s": number("rest_s", 0)}
            if result.get("completion_strategy", "operator") == "timed_exit":
                step["playback_s"] = number("playback_s", None)
            steps = [step]
            for slot in range(2, 8):
                selected = result.get(f"motion_{slot}")
                if isinstance(selected, str):
                    selected = selected.strip()
                if selected not in (None, ""):
                    steps.append({**step, "motion": selected})
            result["plan"] = {"rounds": number("rounds", 1, True),
                              "round_rest_s": number("round_rest_s", 0), "steps": steps}
        result["dry_run"] = _canvas_boolean(result.get("dry_run", True), "dry_run", True)
        result["confirm"] = _canvas_boolean(result.get("confirm", False), "confirm", False)
    if action in {"continue", "calibrate", "finish_calibration"}:
        result["confirm"] = _canvas_boolean(result.get("confirm", False), "confirm", False)
        for key in ("job_id", "confirmation_token"):
            if isinstance(result.get(key), str):
                result[key] = result[key].strip()
    if action == "calibrate":
        result["dry_run"] = _canvas_boolean(result.get("dry_run", True), "dry_run", True)
        if isinstance(result.get("motion"), str):
            result["motion"] = result["motion"].strip()
    if action == "status" and isinstance(result.get("job_id"), str):
        result["job_id"] = result["job_id"].strip() or None
    return result


def load_library(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise RoutineError("动作库需要 schema_version=1")
    entries = data.get("motions")
    if not isinstance(entries, list) or not entries:
        raise RoutineError("动作库 motions 必须是非空列表")
    motions = {}
    for item in entries:
        if (not isinstance(item, dict) or not isinstance(item.get("id"), str)
                or not item["id"] or item["id"] in motions
                or not isinstance(item.get("name"), str) or not item["name"].strip()):
            raise RoutineError("动作必须有唯一 id 和非空 name")
        motions[item["id"]] = copy.deepcopy(item)
    return motions


def saved_playback_window(item, robot_id):
    """A reusable operator-measured window, never an SDK completion event."""
    evidence = item.get("playback_window")
    if (not isinstance(evidence, dict)
            or evidence.get("source") != "operator_confirmation"
            or evidence.get("robot_id") != robot_id
            or evidence.get("recording_id") != item.get("recording_id")
            or not evidence.get("confirmed_at")):
        raise RoutineError(f"{item['name']}: 缺少当前素材的测时记录；请先 calibrate，或显式填写 playback_s")
    return _seconds(evidence.get("seconds"), f"{item['id']}.playback_window.seconds", positive=True)


def check_plan(plan, library, *, dry_run, robot_id, completion_strategy="operator", calibration=False):
    """Return a detached, bounded plan and preview; never touch a controller."""
    if completion_strategy not in ("operator", "timed_exit"):
        raise RoutineError("completion_strategy 只支持 operator 或 timed_exit")
    if type(dry_run) is not bool:
        raise RoutineError("dry_run 必须是布尔值")
    if not isinstance(plan, dict) or set(plan) - {"rounds", "round_rest_s", "steps"}:
        raise RoutineError("plan 只允许 rounds、round_rest_s、steps")
    rounds = _integer(plan.get("rounds", 1), "rounds", 1, 20)
    round_rest = _seconds(plan.get("round_rest_s", 0), "round_rest_s")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not 1 <= len(steps) <= 20:
        raise RoutineError("steps 必须包含 1–20 个步骤")
    if calibration and (completion_strategy != "operator" or rounds != 1 or len(steps) != 1
                        or not isinstance(steps[0], dict) or steps[0].get("play_count", 1) != 1):
        raise RoutineError("测时只允许 operator 模式单动作、单次、单轮")
    normalized, warnings, plays_per_round, rest_per_round = [], [], 0, 0.0
    duration_per_round, upper_per_round = 0.0, 0.0
    durations_known = True
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or set(step) - {"motion", "play_count", "rest_s", "playback_s"}:
            raise RoutineError("步骤只允许 motion、play_count、rest_s、playback_s")
        key = step.get("motion")
        if not isinstance(key, str) or key not in library:
            raise RoutineError(f"未知动作：{key!r}")
        item = copy.deepcopy(library[key])
        count = _integer(step.get("play_count", 1), "play_count", 1, 20)
        rest = _seconds(step.get("rest_s", 0), "rest_s")
        window = None
        window_source = None
        if completion_strategy == "timed_exit":
            if "playback_s" in step and step["playback_s"] is not None:
                window = _seconds(step["playback_s"], f"{key}.playback_s", positive=True)
                window_source = "explicit"
            else:
                window = saved_playback_window(item, robot_id)
                window_source = "operator_calibration"
        elif "playback_s" in step:
            raise RoutineError("playback_s 仅用于 timed_exit 定时退出模式")
        issues = []
        if item.get("verified") is not True and not calibration:
            issues.append("素材尚未真机验证")
        rid = item.get("recording_id")
        if type(rid) is not int or not 0 <= rid <= 65535:
            issues.append("未绑定有效 recording_id")
        if not isinstance(robot_id, str) or not robot_id.strip() or item.get("robot_id") != robot_id:
            issues.append("未绑定当前机器人 robot_id")
        if (not calibration and
                (not isinstance(item.get("verification_notes"), str) or not item["verification_notes"].strip())):
            issues.append("缺少回放验证记录")
        if item.get("completion_strategy") != "operator":
            issues.append("当前只支持 operator 人工确认；自动物理完成策略待验证")
        timeout = item.get("timeout_s")
        if timeout is None:
            issues.append("未配置人工确认超时")
            timeout = 120.0  # Planning bound only; never a physical end time.
        timeout = _seconds(timeout, f"{key}.timeout_s", positive=True)
        duration = item.get("duration_s")
        if duration is not None:
            duration = _seconds(duration, f"{key}.duration_s", positive=True)
            if duration > timeout:
                raise RoutineError(f"{key}: 实测时长不能大于 timeout_s")
        repetitions = item.get("internal_repetitions")
        if repetitions is not None:
            _integer(repetitions, f"{key}.internal_repetitions", 1, 1000)
        if issues and not dry_run:
            raise RoutineError(f"{item['name']} 不能用于真机运行：" + "；".join(issues))
        if issues:
            warnings.append({"motion": key, "issues": issues})
        item.update(timeout_s=timeout, duration_s=duration)
        normalized.append({"motion": key, "play_count": count, "rest_s": rest,
                           "playback_s": window, "playback_window_source": window_source, "asset": item})
        plays_per_round += count
        rest_per_round += rest * (count - 1 + (index < len(steps) - 1))
        upper_per_round += ((window if window is not None else timeout) + 6) * count
        if duration is None or item.get("verified") is not True:
            durations_known = False
        else:
            duration_per_round += duration * count
    total = rounds * plays_per_round
    if total > MAX_PLAYS:
        raise RoutineError(f"整套流程最多 {MAX_PLAYS} 次片段播放")
    rest_total = rounds * rest_per_round + (rounds - 1) * round_rest
    budget = rounds * upper_per_round + rest_total
    if budget > MAX_TASK_S:
        raise RoutineError(f"按片段超时计算的流程上限不能超过 {MAX_TASK_S} 秒")
    normalized_plan = {"rounds": rounds, "round_rest_s": round_rest, "steps": normalized,
                       "completion_strategy": completion_strategy}
    preview = {
        "ok": True, "state": "validated", "dry_run": dry_run,
        "completion_strategy": "simulated" if dry_run else completion_strategy,
        "rounds": rounds, "total_steps": len(steps), "total_plays": total,
        "total_rest_s": round(rest_total, 6), "timeout_budget_s": round(budget, 6),
        "estimated_duration_s": (round(rounds * duration_per_round + rest_total, 3)
                                 if durations_known else None),
        "estimate_note": "仅按已验证实测素材时长估算；不含启动、退出及操作员确认等待，不是承诺时长。",
        "warnings": warnings,
        "execution_note": ("按设定窗口退出回放并重播；流程完成不证明录制动作完整播放。"
                           if completion_strategy == "timed_exit" else "每段完整结束后由现场人员确认，程序退出回放。"),
        "scheduled_duration_s": (round(rounds * sum(s["playback_s"] * s["play_count"] for s in normalized)
                                       + rest_total, 3) if completion_strategy == "timed_exit" else None),
        "steps": [{"motion": s["motion"], "name": s["asset"]["name"],
                   "play_count": s["play_count"], "rest_s": s["rest_s"], "playback_s": s["playback_s"],
                   "playback_window_source": s["playback_window_source"],
                   "recording_id": s["asset"].get("recording_id"),
                   "verified": s["asset"].get("verified") is True,
                   "internal_repetitions": s["asset"].get("internal_repetitions")}
                  for s in normalized],
    }
    return normalized_plan, preview


class LocoRoutineBackend:
    """One job's revocable body lease, shared with LocoPlugin.dispatch.

    The lock is reentrant because explicit dispatch invokes preempt_locked while
    holding it. No worker join or plugin-state lock is taken inside this lease.
    """
    def __init__(self, loco, cancel):
        self.loco = loco
        self.cancel = cancel
        self.playback_pending = False
        self.preempt_reason = None
        self.exit_result = None

    def owns(self):
        return self.loco._routine_owner is self

    def claim(self):
        # Do not queue an old run behind a long direct action.
        if not self.loco._action_lock.acquire(blocking=False):
            raise RoutineError("其他动作正在执行，请等待或先停止后再开始编排")
        try:
            if self.cancel.is_set():
                raise Cancelled()
            if self.loco._routine_owner is not None or self.loco.is_moving():
                raise RoutineError("机器人已有编排或移动任务，请先停止")
            self._mode({WALKING_MODE})
            self.loco._routine_owner = self
            try:
                self.loco._preempt_servo("exercise_routine.run")
            except Exception as exc:
                self.loco._routine_owner = None
                raise RoutineError(f"无法暂停流式控制：{exc}") from exc
        finally:
            self.loco._action_lock.release()

    def _mode(self, allowed):
        # A successful get_mode cannot prove the SDK's cached DDS data is fresh.
        try:
            mode = int(self.loco._high_ctrl.get_mode())
        except Exception as exc:
            raise RoutineError(f"无法读取机器人状态：{exc}") from exc
        if mode == PROTECTION_MODE:
            raise RoutineError("机器人进入保护模式，已停止调度；不会强行 WALK 恢复")
        if mode not in allowed:
            raise RoutineError(f"工作模式异常：{mode}，预期 {sorted(allowed)}")
        return mode

    def check(self, playing=False):
        with self.loco._action_lock:
            if not self.owns() or self.cancel.is_set():
                raise Cancelled()
            return self._mode({WALKING_MODE, PLAY_TEACH_MODE} if playing else {WALKING_MODE})

    def play(self, asset):
        with self.loco._action_lock:
            self.check()
            # Bypass _prepare_workmode: this card must never secretly enable,
            # stand up, or transition from an unrelated mode into walking.
            self.exit_result = None  # Never carry the previous clip's exit confirmation.
            self.playback_pending = True  # A partial/uncertain start still needs cleanup.
            result = self.loco._trigger_user_action(
                "play_recording", "PLAYTEACH", {PLAY_TEACH_MODE}, [],
                self.loco._safety_requirements("play_recording"),
                index=asset["recording_id"], recording_id=asset["recording_id"])
            if result.get("state") != "running" or result.get("confirmed_started") is not True:
                raise RoutineError(f"回放启动未确认：{result}")
            if self.cancel.is_set():
                raise Cancelled()

    def exit(self):
        with self.loco._action_lock:
            if not self.owns() or not self.playback_pending:
                return
            result = self.loco._do_stop_playback(self.loco._safety_requirements("stop_playback"))
            self.exit_result = result
            if result.get("confirmed") is not True or result.get("workmode") != WALKING_MODE:
                raise RoutineError(f"回放退出未确认，禁止后续运动：{result}")
            self.playback_pending = False

    def release(self):
        with self.loco._action_lock:
            # Keep the lease on failed exit: explicit stop/recovery must resolve
            # it before another job can acquire the body.
            if self.owns() and not self.playback_pending:
                self.loco._routine_owner = None

    def preempt_locked(self, reason):
        """External direct command invalidates this job even in the same mode."""
        self.preempt_reason = reason
        self.cancel.set()  # Always before exit or any subsequent direct command.
        try:
            self.exit()
            self.release()
            return None
        except Exception as exc:
            return {"state": "error", "error": str(exc),
                    "code": "ROUTINE_EXIT_UNCONFIRMED", "preempted_routine": True}


class ExerciseRoutinePlugin:
    PREFIX = "exercise_routine"

    def __init__(self, plugin_config, loco_plugin, *, on_terminal=None):
        self._loco = loco_plugin
        self._robot_id = plugin_config.get("robot_id", "")
        self._library_path = Path(plugin_config.get("library_path") or
                                  Path(__file__).with_name("resource") / "exercise_motions.json")
        self._persistent_library = bool(plugin_config.get("library_path"))
        self._sim_clip_s = _seconds(plugin_config.get("dry_run_clip_s", 0.05),
                                    "dry_run_clip_s", maximum=5, positive=True)
        self._lock = threading.RLock()
        self._job = None
        self._closed = False
        self._on_terminal = on_terminal if on_terminal is not None else notify_completion
        self._library_error = None
        try:
            self._library = load_library(self._library_path)
        except Exception as exc:
            self._library = {}
            self._library_error = str(exc)

    def get_tool(self):
        choices = [{"const": key, "title": item["name"] +
                    (f"（{item['recording_id']}）" if type(item.get("recording_id")) is int else "（未绑定）")}
                   for key, item in self._library.items()]
        actions = {"ui": "打开动作编排面板（需要Core工作台支持）", "validate": "预检顺序编排", "run": "运行顺序编排",
                   "status": "查看进度", "cancel": "停止当前流程",
                   "register_recordings": "加入101–107（保留现有绑定，新素材需逐个测时验证）",
                   "calibrate": "播放一个动作并开始测时", "finish_calibration": "确认本次动作完整结束并保存测时",
                   "continue": "确认当前片段完整结束",
                   "validate_plan": "高级：预检多动作计划", "run_plan": "高级：运行多动作计划"}
        slots = [f"motion_{slot}" for slot in range(2, 8)]
        simple = ["motion", *slots, "rounds", "play_count", "rest_s", "round_rest_s",
                  "completion_strategy", "playback_s", "dry_run"]
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Bumi 动作编排：打开面板选择动作、滑动设置轮数/次数/间歇，查询、确认结束和停止均在同一面板。定时退出模式自动退出并重播，不证明物理动作已完整结束。",
            "inputSchema": {
                "type": "object", "properties": {
                    "action": {"type": "string", "enum": list(actions),
                               "oneOf": [{"const": k, "title": v} for k, v in actions.items()]},
                    "motion": {"type": "string", "description": "选择动作库中的素材",
                               **({"enum": list(self._library), "oneOf": choices} if choices else {})},
                    **{key: {"type": "string", "default": "", "description": f"第{index}个动作；留空跳过，按槽位顺序执行",
                             "enum": ["", *self._library],
                             "oneOf": [{"const": "", "title": "不添加"}, *choices]}
                       for index, key in enumerate(slots, 2)},
                    "rounds": {"type": "integer", "minimum": 1, "maximum": 20, "default": 1,
                               "description": "整套轮数，1–20"},
                    "play_count": {"type": "integer", "minimum": 1, "maximum": 20, "default": 1,
                                   "description": "每轮每个所选动作的播放次数，1–20"},
                    "rest_s": {"type": "number", "minimum": 0, "maximum": 600, "default": 0,
                               "description": "片段之间休息秒数，0–600"},
                    "round_rest_s": {"type": "number", "minimum": 0, "maximum": 600, "default": 0,
                                     "description": "轮间休息秒数，0–600"},
                    "completion_strategy": {"type": "string", "enum": ["operator", "timed_exit"],
                                            "oneOf": [{"const": "operator", "title": "人工确认完整结束"},
                                                      {"const": "timed_exit", "title": "按设定秒数退出并重播"}],
                                            "default": "operator"},
                    "playback_s": {"type": "number", "minimum": 0.1, "maximum": 600,
                                   "description": "留空自动使用每个动作保存的测时窗口；填写则覆盖所有所选动作，可能截断动作"},
                    "job_id": {"type": "string", "description": "当前任务的job_id（由开始测时或status返回）"},
                    "confirmation_token": {"type": "string", "description": "status返回的当前片段确认令牌"},
                    "plan": {"type": "object", "additionalProperties": False,
                             "properties": {"rounds": {"type": "integer", "minimum": 1, "maximum": 20},
                                            "round_rest_s": {"type": "number", "minimum": 0, "maximum": 600},
                                            "steps": {"type": "array", "minItems": 1, "maxItems": 20,
                                                      "items": {"type": "object", "additionalProperties": False,
                                                                "properties": {"motion": {"type": "string"},
                                                                               "play_count": {"type": "integer", "minimum": 1, "maximum": 20},
                                                                               "rest_s": {"type": "number", "minimum": 0, "maximum": 600},
                                                                               "playback_s": {"type": "number", "exclusiveMinimum": 0, "maximum": 600}},
                                                                "required": ["motion"]}}}, "required": ["steps"]},
                    "dry_run": {"type": "boolean", "enum": [True, False], "default": True,
                                "description": "true=模拟，false=真机"},
                    "confirm": {"type": "boolean", "enum": [False, True], "default": False,
                                "description": "现场确认站稳及空间；定时模式同时确认所填窗口可能截断动作"},
                }, "required": ["action"],
                "x-action-params": {
                    "ui": {"params": [], "description": "打开含滑块和控制按钮的面板；不发送动作。"},
                    "validate": {"params": simple, "required": ["motion"], "description": "表单预检；不发送运动指令。"},
                    "run": {"params": simple + ["confirm"], "required": ["motion"], "description": "按表单启动；默认模拟。"},
                    "status": {"params": [], "description": "查询当前任务。"},
                    "cancel": {"params": [], "description": "停止后续调度并退出本流程回放。"},
                    "register_recordings": {"params": [], "description": "注册当前机器人101–107；不播放、不覆盖已有绑定或测时。随后刷新服务能力列表。"},
                    "calibrate": {"params": ["motion", "dry_run", "confirm"], "required": ["motion"],
                                  "description": "一次只测试一个动作。dry_run=false且confirm=true启动，完整结束后finish_calibration。"},
                    "finish_calibration": {"params": ["job_id", "confirm"], "required": ["job_id"],
                                           "description": "只确认指定测时任务；confirm=true表示现场已看到完整结束。成功退出后才保存。"},
                    "continue": {"params": ["job_id", "confirmation_token", "confirm"],
                                 "required": ["job_id", "confirmation_token"], "description": "确认当前任务和片段。"},
                    "validate_plan": {"params": ["plan", "completion_strategy", "dry_run"], "required": ["plan"],
                                      "description": "高级多动作计划预检；普通使用请打开面板。"},
                    "run_plan": {"params": ["plan", "completion_strategy", "dry_run", "confirm"], "required": ["plan"],
                                 "description": "高级多动作计划；人工确认请使用面板按钮。"},
                },
                "x-workbench": {"protocol": "motus.workbench/1", "kind": "routine",
                                "title": "Bumi 动作编排", "entry": "ui",
                                "actions": ["ui", "info", "validate_plan", "run_plan", "continue", "stop"]},
                "x-completion": {"actions": ["run", "run_plan", "calibrate"], "timeout": MAX_TASK_S + 15},
                "x-hooks": {"on_interrupt_motion": {"action": "stop"}, "on_interrupt_all": {"action": "stop"}},
                "x-stop": {"pending_flags": {"control_reserved": True}},
                "x-is-dangerous": True, "x-resource": ["base", "leg", "arm_l", "arm_r"],
            }, "topic_out": [],
        }

    def get_tools(self):
        return [self.get_tool()]

    def start(self):
        return {"state": "ready"}

    def stop(self):
        result = self._request_stop(close=True)
        with self._lock:
            job = self._job
        if job and job["thread"].ident is not None and job["thread"] is not threading.current_thread():
            job["thread"].join(timeout=9)
        return self._status() if job else result

    def dispatch(self, action, args):
        if args.get("_tool_name") == "exercise_routine_control" and action not in {
                "start", "stop", "status", "continue", "info"}:
            return {"ok": False, "state": "error", "error": "编排控制入口不能启动新计划或任意动作"}
        try:
            args = normalize_canvas_args(action, args)
        except (ValueError, TypeError) as exc:
            return {"ok": False, "state": "error", "error": str(exc)}
        if action == "start":
            return {"state": "ready" if not self._closed else "error"}
        if action == "ui":
            return {"protocol": "motus.workbench/1",
                    "html": (Path(__file__).with_name("resource") / "exercise_routine.html").read_text(encoding="utf-8")}
        if action == "info":
            return {"state": "ready" if not self._closed else "closed",
                    "library_path": str(self._library_path), "library_error": self._library_error,
                    "robot_id": self._robot_id, "motions": copy.deepcopy(list(self._library.values())),
                    "completion_strategy": "operator", "automatic_physical_completion": False,
                    "sdk_data_freshness": "unverifiable", "progress_source": "scheduler",
                    "task_protocol": "acp_with_operator_control",
                    "control_tool": "exercise_routine", "busy": self._busy(), "current": self._status(),
                    "available_completion_strategies": ["operator", "timed_exit"]}
        if action == "status":
            return self._status(args.get("job_id"))
        if action in {"stop", "cancel"}:
            return self._request_stop()
        if action == "continue":
            return self._confirm(args)
        if action == "finish_calibration":
            with self._lock:
                if not self._job or not self._job.get("calibration") or args.get("job_id") != self._job["id"]:
                    return {"ok": False, "state": "error", "error": "需要当前单动作测时任务的job_id"}
                return self._confirm({**args, "confirmation_token": self._job["status"]["confirmation_token"]})
        if action in {"register_recordings", "calibrate"}:
            try:
                self._require_persistent_library()
                if action == "register_recordings":
                    return self._register_recordings()
                if args.get("dry_run") is not False or args.get("confirm") is not True:
                    raise RoutineError("测时需现场确认：dry_run=false、confirm=true；会真实播放一个动作")
                p = {"rounds": 1, "steps": [{"motion": args.get("motion")}]}
                plan, preview = check_plan(p, self._library, dry_run=False, robot_id=self._robot_id,
                                           calibration=True)
                return self._run(plan, preview, False, calibration=True,
                                 await_publication=args.get("_await_response_publication") is True)
            except (ValueError, TypeError, OSError) as exc:
                return {"ok": False, "state": "error", "error": str(exc)}
        if action in {"validate", "run", "validate_plan", "run_plan"}:
            try:
                if self._library_error:
                    raise RoutineError(f"动作库读取失败：{self._library_error}")
                dry = args.get("dry_run", True)
                plan, preview = check_plan(args.get("plan"), self._library,
                                           dry_run=dry, robot_id=self._robot_id,
                                           completion_strategy=args.get("completion_strategy", "operator"))
                if action in {"validate", "validate_plan"}:
                    return preview
                if not dry and args.get("confirm") is not True:
                    raise RoutineError("真机启动需要现场确认稳定站立与周围空间，并传入 confirm=true")
                return self._run(plan, preview, dry,
                                 await_publication=args.get("_await_response_publication") is True)
            except (ValueError, TypeError) as exc:
                return {"ok": False, "state": "error", "error": str(exc)}
        return None

    def _busy(self):
        with self._lock:
            job = self._job
            return bool(job and (not job["finished"] or not job["notification_done"].is_set()
                                 or (job["backend"] and job["backend"].owns())))

    def _status(self, job_id=None):
        with self._lock:
            job = self._job
            if job_id is not None and (not job or job_id != job["id"]):
                return {"ok": False, "state": "error", "error": "任务不存在；只保留最近一项任务"}
            if not job:
                return {"state": "idle"}
            result = copy.deepcopy(job["status"])
            result["rest_remaining_s"] = round(max(0.0, job.get("rest_until", 0) - time.monotonic()), 3)
            backend = job["backend"]
            result["playback_exit_confirmed"] = (None if job["dry"] else
                                                backend.exit_result.get("confirmed") if backend.exit_result else None)
            result["control_reserved"] = bool(backend and backend.owns())
            return result

    def _update(self, job, **values):
        with self._lock:
            if job["cancel"].is_set() and "state" in values and values["state"] not in TERMINAL:
                values["state"] = "stopping"
            job["status"].update(values)

    def _require_persistent_library(self):
        if self._library_error:
            raise RoutineError(f"动作库读取失败：{self._library_error}")
        if not self._persistent_library or not isinstance(self._robot_id, str) or not self._robot_id.strip():
            raise RoutineError("请先配置当前机器人的robot_id和持久library_path；不会修改默认候选库")

    def _persist_library(self, motions):
        """Called under _lock; back up and atomically replace only this robot's library."""
        self._require_persistent_library()
        if load_library(self._library_path) != self._library:
            raise RoutineError("磁盘动作库已被外部修改，请重启Driver加载后再操作；未覆盖")
        data = json.loads(self._library_path.read_text(encoding="utf-8"))
        data["motions"] = list(motions.values())
        backup = self._library_path.with_name(self._library_path.name + f".{time.time_ns()}.bak")
        shutil.copy2(self._library_path, backup)
        staged = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self._library_path.parent,
                                             prefix=".bumi-library-", delete=False) as output:
                staged = Path(output.name)
                json.dump(data, output, ensure_ascii=False, indent=2, allow_nan=False)
                output.write("\n")
                output.flush()
                os.fsync(output.fileno())
            os.replace(staged, self._library_path)
        finally:
            if staged is not None and staged.exists():
                staged.unlink()
        self._library = copy.deepcopy(motions)
        return str(backup)

    def _register_recordings(self):
        with self._lock:
            if self._closed or self._busy():
                raise RoutineError("请先停止编排并确认已退出回放，再注册动作")
            motions = copy.deepcopy(self._library)
            added, existing = [], []
            for recording_id in range(101, 108):
                matches = [m for m in motions.values() if type(m.get("recording_id")) is int
                           and m["recording_id"] == recording_id]
                if len(matches) > 1:
                    raise RoutineError(f"录制{recording_id}有多个绑定，请先核对素材库")
                if matches:
                    existing.append(matches[0]["id"])
                    continue
                key = f"recording_{recording_id}"
                if key in motions:
                    raise RoutineError(f"素材键{key}已占用，未覆盖")
                motions[key] = {"id": key, "name": f"动作{recording_id}", "recording_id": recording_id,
                                "robot_id": self._robot_id, "verified": False,
                                "verification_notes": "", "duration_s": None, "timeout_s": 120,
                                "completion_strategy": "operator"}
                added.append(key)
            backup = self._persist_library(motions) if added else None
            return {"ok": True, "state": "registered", "added": added, "existing": existing,
                    "backup": backup, "motion_sent": False,
                    "note": "新动作需逐个calibrate验证；重新检测服务能力列表后显示新选项"}

    def _save_calibration(self, job):
        with self._lock:
            if job["cancel"].is_set():
                raise Cancelled()
            step = job["plan"]["steps"][0]
            key, asset = step["motion"], step["asset"]
            seconds = _seconds(job["measured_window_s"], "测时窗口", positive=True)
            updated = copy.deepcopy(self._library)
            if (updated[key].get("robot_id") != asset["robot_id"]
                    or updated[key].get("recording_id") != asset["recording_id"]):
                raise RoutineError("测时期间素材绑定发生变化，未保存")
            stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            evidence = {"seconds": seconds, "source": "operator_confirmation", "confirmed_at": stamp,
                        "robot_id": self._robot_id, "recording_id": asset["recording_id"]}
            updated[key].update(verified=True, playback_window=evidence)
            previous_notes = updated[key].get("verification_notes", "")
            updated[key]["verification_notes"] = (previous_notes + "\n" if previous_notes else "") + (
                f"{stamp} 现场确认完整回放且退出成功；窗口{seconds:.3f}秒含人工反应时间，非SDK自动完成事件。")
            backup = self._persist_library(updated)
            job["status"].update(calibration_saved=True, calibrated_window=evidence, library_backup=backup)

    def _run(self, plan, preview, dry, *, await_publication=False, calibration=False):
        with self._lock:
            if self._closed:
                return {"ok": False, "state": "error", "error": "驱动正在关闭"}
            if self._job and (not self._job["finished"]
                              or not self._job["notification_done"].is_set()):
                return {"ok": False, "state": "busy", "job_id": self._job["id"]}
            if not dry and self._loco is None:
                raise RoutineError("动作编排依赖已启用的 loco 插件")
            job_id = "bumi_routine_" + uuid4().hex
            cancel = threading.Event()
            job = {"id": job_id, "dry": dry, "cancel": cancel, "ack": threading.Event(),
                   "calibration": calibration,
                   "finished": False, "plan": plan, "rest_until": 0.0,
                   "publication": threading.Event(), "notification_done": threading.Event(),
                   "await_publication": await_publication, "published_at": None, "notify": True,
                   "backend": None if dry else LocoRoutineBackend(self._loco, cancel),
                   "status": {"ok": True, "job_id": job_id, "action_id": job_id, "state": "running", "dry_run": dry,
                              "completion_strategy": "simulated" if dry else plan["completion_strategy"],
                              "execution_note": preview["execution_note"], "full_playback_confirmed": False,
                              "round": 0, "total_rounds": plan["rounds"], "step": 0,
                              "total_steps": len(plan["steps"]), "play_index": 0,
                              "play_count": 0, "completed_plays": 0, "exited_plays": 0, "total_plays": preview["total_plays"],
                              "confirmation_token": None, "error": None,
                              "calibration": calibration, "calibration_saved": False,
                              "progress_source": "scheduler", "sdk_data_freshness": "unverifiable"}}
            if not await_publication:
                job["publication"].set()
            job["thread"] = threading.Thread(target=self._worker, args=(job,),
                                              name="bumi_exercise_routine", daemon=True)
            # Reserve before returning run: an external command arriving before
            # the worker starts must still invalidate this exact job.
            if job["backend"]:
                job["backend"].claim()
            self._job = job
            try:
                job["thread"].start()
            except Exception as exc:
                job["finished"] = True
                job["status"].update(state="error", ok=False, error=f"无法启动编排线程：{exc}")
                if job["backend"]:
                    job["backend"].release()
                job["notification_done"].set()
                result = self._status()
                # No worker was started, so do not register an ACP pending.
                result.pop("action_id", None)
                return result
            return self._status()

    def response_published(self, action_id, delivered=True):
        """Called by the MCP HTTP handler only after write+flush, or on failure."""
        with self._lock:
            job = self._job
            if not job or action_id != job["id"] or not job["await_publication"]:
                return
            if job["publication"].is_set():
                return
            job["published_at"] = time.monotonic() if delivered else None
            job["notify"] = bool(delivered)
            if not delivered:
                job["cancel"].set()
                job["status"]["cancel_reason"] = "MCP run response was not delivered"
            job["publication"].set()

    def _request_stop(self, close=False):
        with self._lock:
            if close:
                self._closed = True
            job = self._job
            if not job:
                return {"state": "idle"}
            job["cancel"].set()  # Before cleanup/locks: no next clip may start.
            if close and not job["publication"].is_set():
                # No response/action_id was delivered; do not strand a daemon
                # waiting for a client while shutting down the driver.
                job["notify"] = False
                job["publication"].set()
            if not job["finished"]:
                job["status"]["state"] = "stopping"
                return self._status()
            backend = job["backend"]
        # A previous unconfirmed exit retains the lease. An explicit later stop
        # can retry the bounded SDK exit, without emitting a second terminal event.
        if backend and backend.owns():
            with backend.loco._action_lock:
                error = backend.preempt_locked("exercise_routine.stop_retry")
            if error:
                self._update(job, error=error["error"])
        return self._status()

    def _confirm(self, args):
        with self._lock:
            job = self._job
            if (not job or job["dry"] or job["finished"] or job["cancel"].is_set()
                    or job["status"]["state"] != "awaiting_confirmation"
                    or args.get("job_id") != job["id"] or args.get("confirm") is not True
                    or not args.get("confirmation_token")
                    or args["confirmation_token"] != job["status"]["confirmation_token"]
                    or job["ack"].is_set()):
                return {"ok": False, "state": "error", "error": "需要当前任务、当前片段令牌及 confirm=true；不能提前或重复确认"}
            job["ack"].set()
            job["acknowledged_at"] = time.monotonic()
            return {"ok": True, "state": "confirmation_received", "job_id": job["id"]}

    def _guard(self, job, playing=False):
        if job["cancel"].is_set():
            raise Cancelled()
        if time.monotonic() > job["deadline"]:
            raise RoutineError("整套流程超时")
        if job["backend"]:
            job["backend"].check(playing)

    def _wait(self, job, seconds, *, resting=False):
        until = time.monotonic() + seconds
        with self._lock:
            job["rest_until"] = until if resting else 0.0
        try:
            while True:
                self._guard(job)
                remaining = until - time.monotonic()
                if remaining <= 0:
                    return
                job["cancel"].wait(min(POLL_S, remaining))
        finally:
            with self._lock:
                job["rest_until"] = 0.0

    def _play(self, job, step):
        asset = step["asset"]
        self._update(job, state="running", confirmation_token=None)
        if job["dry"]:
            self._wait(job, self._sim_clip_s)
            return
        backend = job["backend"]
        backend.play(asset)
        started_at = time.monotonic()
        if job["plan"]["completion_strategy"] == "timed_exit":
            # A commanded window is not evidence of physical completion.
            self._update(job, state="playing_window", playback_s=step["playback_s"],
                         playback_window_source=step["playback_window_source"])
            until = time.monotonic() + step["playback_s"]
            while True:
                self._guard(job, playing=True)
                remaining = until - time.monotonic()
                self._update(job, playback_remaining_s=round(max(0.0, remaining), 3))
                if remaining <= 0:
                    break
                job["cancel"].wait(min(POLL_S, remaining))
            self._guard(job, playing=True)
            self._update(job, state="exiting_playback", playback_remaining_s=0)
            backend.exit()
            self._guard(job)
            return
        token = uuid4().hex
        with self._lock:
            if job["cancel"].is_set():
                raise Cancelled()
            job["ack"].clear()
            job["status"].update(state="awaiting_confirmation", confirmation_token=token)
        deadline = time.monotonic() + asset["timeout_s"]
        while True:
            self._guard(job, playing=True)
            if time.monotonic() >= deadline:
                raise RoutineError("等待操作员确认片段结束超时；流程中止，尝试退出回放")
            if job["ack"].wait(POLL_S):
                self._guard(job, playing=True)
                break
        self._update(job, state="running", confirmation_token=None)
        backend.exit()  # Only after explicit confirmation; mode alone never completes.
        self._guard(job)
        if job["calibration"]:
            # Capture the acknowledgement time, not SDK-exit latency. A small
            # upward rounding avoids shortening the observed window on reuse.
            job["measured_window_s"] = math.ceil((job["acknowledged_at"] - started_at) * 1000) / 1000

    def _worker(self, job):
        final_state, error = "completed", None
        job["deadline"] = time.monotonic() + MAX_TASK_S
        backend = job["backend"]
        try:
            plan = job["plan"]
            for r in range(1, plan["rounds"] + 1):
                for i, step in enumerate(plan["steps"], 1):
                    for p in range(1, step["play_count"] + 1):
                        self._guard(job)
                        self._update(job, round=r, step=i, motion=step["motion"],
                                     motion_name=step["asset"]["name"], play_index=p, play_count=step["play_count"])
                        self._play(job, step)
                        with self._lock:
                            job["status"]["exited_plays"] += 1
                            if job["dry"] or plan["completion_strategy"] == "operator":
                                job["status"]["completed_plays"] += 1
                        if p < step["play_count"] or i < len(plan["steps"]):
                            self._update(job, state="resting", rest_kind="between_plays_or_steps")
                            self._wait(job, step["rest_s"], resting=True)
                if r < plan["rounds"]:
                    self._update(job, state="resting", rest_kind="between_rounds")
                    self._wait(job, plan["round_rest_s"], resting=True)
            self._guard(job)
            if job["calibration"]:
                self._save_calibration(job)
        except Cancelled:
            final_state = "cancelled"
        except Exception as exc:
            final_state, error = "error", str(exc)
        finally:
            if backend:
                try:
                    backend.exit()
                except Exception as exc:
                    final_state = "error"
                    error = (error + "; " if error else "") + str(exc)
                finally:
                    backend.release()
            with self._lock:
                if final_state == "completed" and job["cancel"].is_set():
                    final_state = "cancelled"
                if backend and backend.preempt_reason:
                    job["status"]["cancel_reason"] = backend.preempt_reason
                job["rest_until"] = 0.0
                job["status"].update(state=final_state, ok=final_state == "completed",
                                      error=error, confirmation_token=None,
                                      full_playback_confirmed=(not job["dry"] and final_state == "completed"
                                                               and job["plan"]["completion_strategy"] == "operator"))
                job["finished"] = True
                result = self._status()
            # Fast completion must not race ahead of the HTTP run response.
            # Keep this job addressable/busy until publication and notification
            # finish, even if its physical/simulated state is already terminal.
            try:
                job["publication"].wait()
                if job["notify"]:
                    if job["await_publication"] and job["published_at"] is not None:
                        # Match the existing Piper ACP registration grace, but
                        # start it after the actual HTTP write+flush handshake.
                        delay = job["published_at"] + 0.1 - time.monotonic()
                        if delay > 0:
                            time.sleep(delay)
                    self._on_terminal(job["id"], final_state, result)
            except Exception as exc:
                print(f"[exercise_routine] terminal observer failed: {exc}", flush=True)
            finally:
                job["notification_done"].set()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="无 ROS/SDK 的 Bumi 编排模拟；绝不连接机器人")
    parser.add_argument("--demo", action="store_true", help="运行内置短模拟")
    parser.add_argument("--plan", type=Path, help="只模拟 JSON 文件中的计划")
    args = parser.parse_args()
    if not args.demo and args.plan is None:
        parser.error("请选择 --demo 或 --plan；此入口只做 dry-run")
    plan = json.loads(args.plan.read_text()) if args.plan else {
        "rounds": 2, "round_rest_s": 0.1,
        "steps": [{"motion": "chest_expand", "play_count": 2, "rest_s": 0.1},
                  {"motion": "arms_up", "play_count": 1, "rest_s": 0.1},
                  {"motion": "arms_forward_push", "play_count": 1, "rest_s": 0}]}
    # Standalone simulation has no Agent Core; disable only the transport.
    plugin = ExerciseRoutinePlugin({}, None, on_terminal=lambda *_args: None)
    preview = plugin.dispatch("validate", {"plan": plan, "dry_run": True})
    print(json.dumps(preview, ensure_ascii=False, indent=2))
    if not preview.get("ok"):
        raise SystemExit(1)
    result = plugin.dispatch("run", {"plan": plan, "dry_run": True})
    try:
        while result["state"] not in TERMINAL:
            time.sleep(0.05)
            result = plugin.dispatch("status", {})
    except KeyboardInterrupt:
        result = plugin.stop()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["state"] == "completed" else 1)
