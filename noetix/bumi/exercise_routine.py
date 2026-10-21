"""Bumi routine scheduler. No ROS/SDK imports; physical completion is manual.

The SDK's workmode is not a trajectory-completion or freshness signal. Only
operator confirmation authorizes a real clip to finish. Simulated timing never
runs on the physical backend. A shared action lock serializes the routine with
all call-shaped body commands; streaming velocity is suppressed while reserved.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
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


def check_plan(plan, library, *, dry_run, robot_id):
    """Return a detached, bounded plan and preview; never touch a controller."""
    if type(dry_run) is not bool:
        raise RoutineError("dry_run 必须是布尔值")
    if not isinstance(plan, dict) or set(plan) - {"rounds", "round_rest_s", "steps"}:
        raise RoutineError("plan 只允许 rounds、round_rest_s、steps")
    rounds = _integer(plan.get("rounds", 1), "rounds", 1, 20)
    round_rest = _seconds(plan.get("round_rest_s", 0), "round_rest_s")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not 1 <= len(steps) <= 20:
        raise RoutineError("steps 必须包含 1–20 个步骤")
    normalized, warnings, plays_per_round, rest_per_round = [], [], 0, 0.0
    duration_per_round, upper_per_round = 0.0, 0.0
    durations_known = True
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or set(step) - {"motion", "play_count", "rest_s"}:
            raise RoutineError("步骤只允许 motion、play_count、rest_s")
        key = step.get("motion")
        if not isinstance(key, str) or key not in library:
            raise RoutineError(f"未知动作：{key!r}")
        item = copy.deepcopy(library[key])
        count = _integer(step.get("play_count", 1), "play_count", 1, 20)
        rest = _seconds(step.get("rest_s", 0), "rest_s")
        issues = []
        if item.get("verified") is not True:
            issues.append("素材尚未真机验证")
        rid = item.get("recording_id")
        if type(rid) is not int or not 0 <= rid <= 65535:
            issues.append("未绑定有效 recording_id")
        if not isinstance(robot_id, str) or not robot_id.strip() or item.get("robot_id") != robot_id:
            issues.append("未绑定当前机器人 robot_id")
        if not isinstance(item.get("verification_notes"), str) or not item["verification_notes"].strip():
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
        normalized.append({"motion": key, "play_count": count, "rest_s": rest, "asset": item})
        plays_per_round += count
        rest_per_round += rest * (count - 1 + (index < len(steps) - 1))
        upper_per_round += (timeout + 6) * count
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
    normalized_plan = {"rounds": rounds, "round_rest_s": round_rest, "steps": normalized}
    preview = {
        "ok": True, "state": "validated", "dry_run": dry_run,
        "completion_strategy": "simulated" if dry_run else "operator",
        "rounds": rounds, "total_steps": len(steps), "total_plays": total,
        "total_rest_s": round(rest_total, 6), "timeout_budget_s": round(budget, 6),
        "estimated_duration_s": (round(rounds * duration_per_round + rest_total, 3)
                                 if durations_known else None),
        "estimate_note": "仅按已验证实测素材时长估算；不含启动、退出及操作员确认等待，不是承诺时长。",
        "warnings": warnings,
        "steps": [{"motion": s["motion"], "name": s["asset"]["name"],
                   "play_count": s["play_count"], "rest_s": s["rest_s"],
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
        self._sim_clip_s = _seconds(plugin_config.get("dry_run_clip_s", 0.05),
                                    "dry_run_clip_s", maximum=5, positive=True)
        self._lock = threading.RLock()
        self._job = None
        self._closed = False
        self._on_terminal = on_terminal
        self._library_error = None
        try:
            self._library = load_library(self._library_path)
        except Exception as exc:
            self._library = {}
            self._library_error = str(exc)

    def get_tool(self):
        step = {"type": "object", "additionalProperties": False,
                "properties": {"motion": {"type": "string", "enum": list(self._library)},
                               "play_count": {"type": "integer", "minimum": 1, "maximum": 20, "default": 1},
                               "rest_s": {"type": "number", "minimum": 0, "maximum": 600, "default": 0}},
                "required": ["motion"]}
        if not self._library:
            step["properties"]["motion"].pop("enum")
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Bumi 动作编排：配置顺序、片段播放次数、轮数和间歇。默认 dry-run；真机逐段等待现场操作员确认结束，不测量人的动作。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["validate", "run", "status", "stop", "continue", "info"]},
                    "plan": {"type": "object", "additionalProperties": False,
                             "properties": {"rounds": {"type": "integer", "minimum": 1, "maximum": 20, "default": 1},
                                            "round_rest_s": {"type": "number", "minimum": 0, "maximum": 600, "default": 0},
                                            "steps": {"type": "array", "items": step, "minItems": 1, "maxItems": 20}},
                             "required": ["steps"]},
                    "dry_run": {"type": "boolean", "default": True},
                    "confirm": {"type": "boolean", "default": False,
                                "description": "真机启动：确认现场稳定站立及空间；continue：确认当前片段已完整结束。"},
                    "job_id": {"type": "string"},
                    "confirmation_token": {"type": "string"},
                }, "required": ["action"],
                "x-action-params": {
                    "validate": {"params": ["plan", "dry_run"], "description": "检查并预览；不发送运动指令。"},
                    "run": {"params": ["plan", "dry_run", "confirm"], "description": "异步启动；默认模拟。真机须 confirm=true，且素材已验证。"},
                    "status": {"params": ["job_id"], "description": "查询流程状态和当前确认令牌。"},
                    "continue": {"params": ["job_id", "confirmation_token", "confirm"], "description": "仅在现场确认当前片段完整结束后提交；不是轨迹暂停续播。"},
                    "stop": {"params": [], "description": "立即取消后续调度，异步退出本卡回放；随后查询 status。不是硬件急停。"},
                    "info": {"params": [], "description": "查看动作库、限制及当前状态。"},
                },
                # Interactive operator confirmation must remain callable. No ACP
                # pending/barrier is held; job_id is deliberately not action_id.
                "x-resource": ["base", "leg", "arm_l", "arm_r"],
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
            job["thread"].join(timeout=4)
        return self._status() if job else result

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if not self._closed else "error"}
        if action == "info":
            return {"state": "ready" if not self._closed else "closed",
                    "library_path": str(self._library_path), "library_error": self._library_error,
                    "robot_id": self._robot_id, "motions": copy.deepcopy(list(self._library.values())),
                    "completion_strategy": "operator", "automatic_physical_completion": False,
                    "sdk_data_freshness": "unverifiable", "progress_source": "scheduler",
                    "task_protocol": "interactive_job_no_acp_barrier", "current": self._status()}
        if action == "status":
            return self._status(args.get("job_id"))
        if action == "stop":
            return self._request_stop()
        if action == "continue":
            return self._confirm(args)
        if action in {"validate", "run"}:
            try:
                if self._library_error:
                    raise RoutineError(f"动作库读取失败：{self._library_error}")
                dry = args.get("dry_run", True)
                plan, preview = check_plan(args.get("plan"), self._library,
                                           dry_run=dry, robot_id=self._robot_id)
                if action == "validate":
                    return preview
                if not dry and args.get("confirm") is not True:
                    raise RoutineError("真机启动需要现场确认稳定站立与周围空间，并传入 confirm=true")
                return self._run(plan, preview, dry)
            except (ValueError, TypeError) as exc:
                return {"ok": False, "state": "error", "error": str(exc)}
        return None

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

    def _run(self, plan, preview, dry):
        with self._lock:
            if self._closed:
                return {"ok": False, "state": "error", "error": "驱动正在关闭"}
            if self._job and not self._job["finished"]:
                return {"ok": False, "state": "busy", "job_id": self._job["id"]}
            if not dry and self._loco is None:
                raise RoutineError("动作编排依赖已启用的 loco 插件")
            job_id = "bumi_routine_" + uuid4().hex
            cancel = threading.Event()
            job = {"id": job_id, "dry": dry, "cancel": cancel, "ack": threading.Event(),
                   "finished": False, "plan": plan, "rest_until": 0.0,
                   "backend": None if dry else LocoRoutineBackend(self._loco, cancel),
                   "status": {"ok": True, "job_id": job_id, "state": "running", "dry_run": dry,
                              "completion_strategy": "simulated" if dry else "operator",
                              "round": 0, "total_rounds": plan["rounds"], "step": 0,
                              "total_steps": len(plan["steps"]), "play_index": 0,
                              "play_count": 0, "completed_plays": 0, "total_plays": preview["total_plays"],
                              "confirmation_token": None, "error": None,
                              "progress_source": "scheduler", "sdk_data_freshness": "unverifiable"}}
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
                return self._status()
            return self._status()

    def _request_stop(self, close=False):
        with self._lock:
            if close:
                self._closed = True
            job = self._job
            if not job:
                return {"state": "idle"}
            job["cancel"].set()  # Before cleanup/locks: no next clip may start.
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

    def _play(self, job, asset):
        self._update(job, state="running", confirmation_token=None)
        if job["dry"]:
            self._wait(job, self._sim_clip_s)
            return
        backend = job["backend"]
        backend.play(asset)
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
                        self._play(job, step["asset"])
                        with self._lock:
                            job["status"]["completed_plays"] += 1
                        if p < step["play_count"] or i < len(plan["steps"]):
                            self._update(job, state="resting", rest_kind="between_plays_or_steps")
                            self._wait(job, step["rest_s"], resting=True)
                if r < plan["rounds"]:
                    self._update(job, state="resting", rest_kind="between_rounds")
                    self._wait(job, plan["round_rest_s"], resting=True)
            self._guard(job)
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
                                      error=error, confirmation_token=None)
                job["finished"] = True
                result = self._status()
            # One worker, one terminal publication. Callback failures never
            # rewrite a physical outcome or publish a second terminal result.
            if self._on_terminal:
                try:
                    self._on_terminal(job["id"], final_state, result)
                except Exception as exc:
                    print(f"[exercise_routine] terminal observer failed: {exc}", flush=True)


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
    plugin = ExerciseRoutinePlugin({}, None)
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
