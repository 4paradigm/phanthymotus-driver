"""Three-step fixed-pose weighing, sharing the existing SDK and motion owner.

No low-level joint commands. Motion uses site-verified teaching recordings.
Calibration uses signed empirical torque/kg, not an assumed SDK torque unit.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

from payload_mass_estimator import (check_baseline, digest, estimate, summarize,
                                    validate_profile)
from payload_mass_sampling import MassError, check_frame, finite, vector


def notify_completion(action_id, status, result):
    import ssl
    import urllib.request
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    req = urllib.request.Request(url + "/api/acp/complete", data=json.dumps({
        "action_id": action_id, "status": status, "result": result,
        "tool": "payload_mass", "ts": time.time()}, allow_nan=False).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    context = ssl._create_unverified_context() if url == "https://localhost:15678" else None
    with urllib.request.urlopen(req, timeout=5, context=context) as response:
        response.read()


def save_trial(root, data):
    path = Path(root) / (uuid4().hex + ".json")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(data, stream, ensure_ascii=False, allow_nan=False)
    return str(path)


class PayloadMassPlugin:
    PREFIX = "payload_mass"

    def __init__(self, cfg, namespace, executor=None, sampler=None,
                 loco_plugin=None, *, publish=None, notify=None):
        self._sampler = sampler
        self._loco = loco_plugin
        self._lock = loco_plugin._action_lock if loco_plugin else threading.RLock()
        self._file = cfg.get("profiles_file", "/opt/phanthy-motus/data/bumi/weighing.json")
        self._root = cfg.get("data_dir", "/opt/phanthy-motus/data/bumi/weighing_trials")
        self._identity = cfg.get("hardware_id") or os.environ.get("BUMI_PAYLOAD_ROBOT_ID", "")
        self._sdk = cfg.get("sdk_version") or os.environ.get("BUMI_PAYLOAD_SDK_VERSION", "")
        self._session = None
        self._job = None
        self._closed = False
        self._result_profile = None
        self._topic = f"/{namespace}/payload_mass/result"
        self._last = self._result("idle", "请选择曲肘；曲肘前保持空载，保留同一挂具。")
        self._publish = publish or (lambda data: None)
        self._notify = notify or notify_completion
        self._node = None
        self._executor = executor
        if executor is not None:
            from rclpy.node import Node
            from std_msgs.msg import String
            self._node = Node("bumi_payload_mass")
            publisher = self._node.create_publisher(String, self._topic, 10)
            def publish_result(data):
                msg = String()
                msg.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
                publisher.publish(msg)
            self._publish = publish_result
            self._node.create_timer(0.5, self._tick)
            executor.add_node(self._node)

    @staticmethod
    def _result(state, message, **extra):
        return dict(state=state, message=message, valid=False, mass_kg=None, unit="kg", **extra)

    def _emit(self, result):
        self._last = copy.deepcopy(result)
        try:
            self._publish(copy.deepcopy(result))
        except Exception as exc:
            print(f"[payload_mass] publish failed: {exc}", flush=True)
        return copy.deepcopy(result)

    def get_tool(self):
        actions = {"prepare": "曲肘：空载调用已验证示教动作，稳定后记录空载力矩。",
                   "weigh": "计算重量：挂物稳定后采集力矩差，显示估算 kg。",
                   "finish": "测量完毕：先卸下物体，调用已验证的双臂下垂动作并核验归位。",
                   "start": "准备卡片。", "stop": "取消任务及本卡示教回放，不自动收臂。",
                   "info": "查看最近结果和配置状态。"}
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False,
                "description": "Bumi 静态手臂估重：曲肘 → 计算重量(kg) → 测量完毕。首次使用须配置示教与实物标定。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(actions), "oneOf": [
                        {"const": a, "title": actions[a].split("：")[0]} for a in actions]}},
                    "required": ["action"],
                    "x-action-params": {a: {"params": [], "description": d} for a, d in actions.items()},
                    "x-completion": {"actions": ["prepare", "weigh", "finish"], "timeout": 120},
                    "x-resource": ["arm_l", "arm_r", "base"]},
                "configSchema": {"type": "object", "properties": {
                    "profiles_file": {"type": "string", "scope": "shared", "default": self._file,
                                      "description": "此机器人固定姿态、示教及标定文件的绝对路径"}}},
                "topic_out": [{"topic": self._topic, "format": "data/json"}]}

    def _load(self):
        try:
            data = json.loads(Path(self._file).read_text())
            p = validate_profile("weighing", data["profile"], data["profile"]["arm"], loading=True)
            motion = data["motion"]
            if not self._identity or not self._sdk or (p["hardware_id"], p["sdk_version"]) != (self._identity, self._sdk):
                raise MassError("IDENTITY_MISMATCH", "配置须匹配本机 hardware_id 和 SDK/固件版本。")
            if motion.get("verified") is not True or not motion.get("evidence"):
                raise MassError("MOTION_UNVERIFIED", "尚未验证曲肘保持和双臂下垂示教动作。")
            for key in ("bend_recording_id", "home_recording_id"):
                if type(motion[key]) is not int or not 0 <= motion[key] <= 65535:
                    raise MassError("BAD_PARAMETER", "示教编号必须为0..65535整数。")
            if motion["bend_recording_id"] == motion["home_recording_id"]:
                raise MassError("BAD_PARAMETER", "曲肘与归位须使用不同示教编号。")
            motion = dict(motion)
            motion["home_q"] = vector(motion["home_q"], 21, "home_q")
            if type(motion.get("home_workmode", 2)) is not int or motion.get("home_workmode", 2) not in (2, 23):
                raise MassError("BAD_PARAMETER", "home_workmode must be 2 or 23")
            if motion.get("home_workmode", 2) == 23 and not p.get("playback_hold_verified"):
                raise MassError("MOTION_UNVERIFIED", "归位后的回放保持模式尚未验证。")
            motion["timeout_s"] = finite(motion.get("timeout_s", 30), "timeout_s", 1, 45)
            duration = finite(data.get("sample_duration_s", 3), "sample_duration_s", 1, 10)
            if duration + p["settle_timeout_s"] + motion["timeout_s"] > 100:
                raise MassError("BAD_PARAMETER", "动作及采样总超时不得超过100秒。")
            model = data.get("model")
            if model is not None:
                if model["profile_fingerprint"] != p["fingerprint"]:
                    raise MassError("CONFIG_CHANGED", "标定与当前配置不匹配。")
                k = vector(model["k"], len(p["motor_ids"]), "k")
                weights = vector(model["weights"], len(k), "weights")
                if any(w < 0 for w in weights) or sum(a*a*w for a, w in zip(k, weights)) <= 1e-15:
                    raise MassError("UNIDENTIFIABLE", "标定质量响应不可辨识。")
                finite(model["resolution_kg"], "resolution_kg", 1e-9, p["max_mass_kg"])
                finite(model["residual_limit"], "residual_limit", 1e-9)
                bounds = vector(model["mass_range_kg"], 2, "mass_range_kg")
                if bounds[0] != 0 or not model["resolution_kg"] < bounds[1] <= p["max_mass_kg"]:
                    raise MassError("BAD_DATA", "无效的标定范围。")
                if (len(set(model["training_trial_ids"])) < 4 or len(set(model["validation_trial_ids"])) < 2
                        or set(model["training_trial_ids"]) & set(model["validation_trial_ids"])):
                    raise MassError("TRIAL_LEAKAGE", "缺少独立训练与验证试次。")
                finite(model["validation"]["max_abs_error_kg"], "validation_error", 0, p["max_calibration_error_kg"])
            return data, p, motion, duration, digest(data)
        except MassError:
            raise
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise MassError("NOT_CONFIGURED", f"请先配置真机示教、姿态和标定：{exc}") from exc

    def start(self):
        return True

    def _fresh(self, p):
        d = self._sampler.diagnostics()
        if d["host_age_s"] is None or d["host_age_s"] > p["max_receive_gap_s"]:
            raise MassError("STALE_SOURCE", "未收到及时反馈。")
        return d

    def _tick(self):
        with self._lock:
            if self._session and not (self._job and self._job["thread"].is_alive()):
                try:
                    d = self._fresh(self._session["p"])
                    check_frame(d["latest"], self._session["p"])
                    check_baseline(self._session["baseline"], summarize([d["latest"]], self._session["p"]),
                                   self._session["p"], time.time())
                    if d["read_error_count"] != self._session["error_epoch"]:
                        raise MassError("STALE_SOURCE", "准备后反馈曾中断，请重新曲肘采空载。")
                    if time.monotonic() - self._session["prepared"] > 300:
                        raise MassError("PREPARATION_EXPIRED", "空载记录已过期。")
                    if self._load()[-1] != self._session["fingerprint"]:
                        raise MassError("CONFIG_CHANGED", "配置改变，请重新曲肘采空载。")
                    if self._last.get("valid") and any(
                            abs(d["latest"]["tau"][j] - self._result_summary["tau"][i]) > self._session["p"]["max_torque_span"]
                            for i, j in enumerate(self._session["p"]["motor_ids"])):
                        self._emit(self._result("expired", "载荷条件改变，请重新计算重量。"))
                except Exception as exc:
                    self._session = None
                    self._emit(self._result("error", str(exc), error_code=getattr(exc, "code", "INVALID_STATE")))
            if self._last.get("valid") and time.monotonic() - self._last["completed_monotonic_s"] > self._result_profile["result_max_age_s"]:
                self._emit(self._result("expired", "结果已过期，请重新计算重量。"))
            self._emit(self._last)

    def info(self):
        self._tick()
        with self._lock:
            result = copy.deepcopy(self._last)
            result.update(prepared=bool(self._session), profiles_file=self._file,
                          active=bool(self._job and self._job["thread"].is_alive()),
                          motion_reserved=bool(self._loco and self._loco._payload_owner is self),
                          diagnostics=self._sampler.diagnostics() if self._sampler else None)
            return result

    def dispatch(self, action, args):
        args = dict(args)
        args.pop("_tool_name", None)
        args.pop("instance_id", None)
        gate = args.pop("_await_response_publication", False)
        with self._lock:
            if action == "info":
                return self.info()
            if action == "stop":
                return self.cancel_locked()
            if action == "start":
                return self._result("ready", "请空载执行曲肘。")
            if self._closed:
                return self._result("error", "卡片已关闭。", error_code="CLOSED")
            if self._job and self._job["thread"].is_alive():
                return self._result("error", "任务正在执行，请等待或停止。", error_code="BUSY")
            try:
                if action == "config":
                    if set(args) != {"profiles_file"} or not isinstance(args["profiles_file"], str) or not Path(args["profiles_file"]).is_absolute():
                        raise MassError("BAD_PARAMETER", "须提供配置文件绝对路径。")
                    if self._loco and self._loco._payload_owner is self:
                        raise MassError("BUSY", "请先完成或停止称重，再修改配置。")
                    self._file = args["profiles_file"]
                    return self._emit(self._result("configured", "配置已更新。"))
                if action not in ("prepare", "weigh", "finish"):
                    return None
                if args:
                    raise MassError("BAD_PARAMETER", "本操作不接受额外参数或待测物体真值。")
                if self._loco is None or self._sampler is None:
                    raise MassError("UNAVAILABLE", "需要现有 state 和 loco 插件。")
                loaded = self._load()
                data, p, motion, duration, fingerprint = loaded
                if action == "prepare" and self._loco._payload_owner is self:
                    raise MassError("ALREADY_PREPARED", "请先卸物并执行测量完毕，再采新的空载。")
                if action == "prepare":
                    if int(self._loco._high_ctrl.get_mode()) != 2:
                        raise MassError("MODE_MISMATCH", "曲肘前须已稳定站立，不会中断其他回放。")
                    check_frame(self._fresh(p)["latest"], {**p, "pose_q": motion["home_q"], "workmode": 2})
                if action == "weigh":
                    self._tick()
                    if not self._session:
                        raise MassError("NOT_PREPARED", "请先空载曲肘，成功后再挂物。")
                    if self._session["fingerprint"] != fingerprint:
                        raise MassError("CONFIG_CHANGED", "配置改变，请先结束本次测量。")
                if action == "finish" and self._loco._payload_owner is not self:
                    raise MassError("NOT_PREPARED", "本卡没有活动会话；未发送收臂命令。")
                # Take exclusive body ownership before pausing the stream, so
                # even a concurrent resume cannot slip a velocity publication in.
                with self._loco._lock:
                    self._loco._payload_owner = self
                self._loco._preempt_servo("payload_mass")
                if self._loco.is_moving():
                    self._loco._stop_move()
                job = {"id": "weigh_" + uuid4().hex, "cancel": threading.Event(),
                       "published": threading.Event(), "notify": True, "action": action,
                       "loaded": loaded, "session": copy.deepcopy(self._session)}
                if not gate:
                    job["published"].set()
                job["thread"] = threading.Thread(target=self._run, args=(job,), daemon=True, name="bumi_weighing")
                self._job = job
                result = self._emit(self._result("running", "正在执行，请等待。", action_id=job["id"]))
                job["thread"].start()
                return result
            except Exception as exc:
                return self._emit(self._result("error", str(exc), error_code=getattr(exc, "code", "INVALID_CONFIG")))

    def response_published(self, action_id, delivered=True):
        with self._lock:
            if self._job and self._job["id"] == action_id:
                if not delivered:
                    self._job["cancel"].set()
                    self._job["notify"] = False
                self._job["published"].set()

    def cancel_locked(self):
        """Called with the shared action lock, including by external stop."""
        if self._job:
            self._job["cancel"].set()
        self._session = None
        if self._loco and self._loco._payload_owner is self:
            # Use the established playback stop, never low-level disable or home.
            try:
                if int(self._loco._high_ctrl.get_mode()) == 23:
                    result = self._loco._do_stop_playback("取消称重")
                    if result.get("state") == "error" or not result.get("confirmed"):
                        raise MassError("STOP_UNCONFIRMED", "回放停止未确认。")
            except Exception as exc:
                return self._emit(self._result("error", f"停止未确认，保留互斥：{exc}", error_code="STOP_UNCONFIRMED"))
            with self._loco._lock:
                self._loco._payload_owner = None
        return self._emit(self._result("cancelled", "已取消；没有自动收臂。"))

    def _check_job(self, job):
        if job["cancel"].is_set() or self._closed:
            raise MassError("CANCELLED", "已取消测量。")
        if self._load()[-1] != job["loaded"][-1]:
            raise MassError("CONFIG_CHANGED", "采样期间配置改变。")

    def _pose(self, job, home=False):
        _, p, motion, _, _ = job["loaded"]
        with self._lock:
            self._check_job(job)
            mode = int(self._loco._high_ctrl.get_mode())
            if not home and mode != 2:
                raise MassError("MODE_MISMATCH", "曲肘只从稳定站立walking模式开始；不会中断其他回放。")
            before = self._fresh(p)["latest"]
            check_frame(before, {**p, "pose_q": before["q"], "workmode": mode})
            if mode not in (2, 23):
                raise MassError("MODE_MISMATCH", "当前模式不允许称重动作。")
            if mode == 23:
                stopped = self._loco._do_stop_playback("切换称重姿态")
                if not stopped.get("confirmed"):
                    raise MassError("STOP_UNCONFIRMED", "上一回放尚未确认停止。")
            # Never auto-enable, stand up or switch an arbitrary posture.
            if int(self._loco._high_ctrl.get_mode()) != 2:
                raise MassError("MODE_MISMATCH", "请先让机器人处于稳定站立walking模式。")
            target = motion["home_q"] if home else p["pose_q"]
            profile = {**p, "pose_q": target,
                       "workmode": motion.get("home_workmode", 2) if home else p["workmode"]}
            if not home:
                initial = {**p, "pose_q": motion["home_q"], "workmode": 2}
                check_frame(self._fresh(initial)["latest"], initial)
            result = self._loco._do_teaching_action("play_recording", {
                "recording_id": motion["home_recording_id" if home else "bend_recording_id"]})
            if result.get("state") == "error":
                raise MassError("MOTION_FAILED", str(result))
        deadline = time.monotonic() + motion["timeout_s"]
        while time.monotonic() < deadline:
            self._check_job(job)
            try:
                check_frame(self._fresh(profile)["latest"], profile)
                return profile
            except MassError as exc:
                if exc.code not in ("POSE_MISMATCH", "MODE_MISMATCH", "UNSTABLE"):
                    raise
                # Protection and disable are terminal, never wait for recovery.
                if int(self._loco._high_ctrl.get_mode()) in (26, 30):
                    raise MassError("MOTION_FAILED", "机器人进入保护或失能模式。")
            job["cancel"].wait(0.05)
        raise MassError("POSE_TIMEOUT", "未观测到示教目标姿态及保持模式；不能认定动作成功。")

    def _run(self, job):
        data, p, _, duration, fingerprint = job["loaded"]
        trial_path = None
        session = job["session"]
        try:
            if not job["published"].wait(5):
                job["notify"] = False
                raise MassError("RESPONSE_TIMEOUT", "未送达动作确认，未启动动作。")
            self._check_job(job)
            if job["action"] in ("prepare", "finish"):
                sample_profile = self._pose(job, home=job["action"] == "finish")
            else:
                sample_profile = p
                check_baseline(session["baseline"], summarize([self._fresh(p)["latest"]], p), p, time.time())
            frames = self._sampler.collect(duration, sample_profile, job["cancel"])
            summary = summarize(frames, sample_profile)
            self._check_job(job)
            if job["action"] == "finish":
                result = self._result("finished", "已核验双臂回到配置的下垂姿态。", home_confirmed=True)
                session = None
            else:
                trial = {"id": uuid4().hex, "profile": p, "profile_fingerprint": p["fingerprint"],
                         "action": job["action"], "created_s": time.time(), "summary": summary, "raw_frames": frames}
                if job["action"] == "prepare":
                    baseline = {"id": trial["id"], "created_s": time.time(),
                                "profile_fingerprint": p["fingerprint"], "summary": summary}
                    session = {"baseline": baseline, "p": p, "fingerprint": fingerprint,
                               "prepared": time.monotonic(), "error_epoch": self._sampler.diagnostics()["read_error_count"]}
                    result = self._result("ready", "曲肘和空载采样完成；现在挂物，稳定后执行计算重量。")
                else:
                    trial["baseline"] = session["baseline"]
                    # Keep failed/no-model attempts for calibration and diagnosis.
                    trial_path = save_trial(self._root, trial)
                    model = copy.deepcopy(data.get("model"))
                    result = self._result("uncalibrated", "已保存挂载数据；需要已知质量实物标定后才能输出 kg。", error_code="NOT_CALIBRATED")
                    if model:
                        if model["profile_fingerprint"] != p["fingerprint"]:
                            raise MassError("CONFIG_CHANGED", "标定不属于当前姿态/挂具/机器人。")
                        # A fresh empty baseline removes session-to-session offset;
                        # only signed per-kg sensitivity is reused from calibration.
                        model["baseline_id"] = session["baseline"]["id"]
                        measured = estimate(summary, session["baseline"], model, p, time.time())
                        result = {**self._result("completed", "估算完成。"), **measured, "valid": True,
                                  "completed_monotonic_s": time.monotonic()}
                if trial_path is None:
                    trial_path = save_trial(self._root, trial)
                result.update(trial_file=trial_path, sample_count=len(frames), arm=p["arm"],
                              freshness_status=summary["freshness_status"], quality="empirical_fixed_pose")
        except Exception as exc:
            result = self._result("cancelled" if getattr(exc, "code", "") == "CANCELLED" else "error",
                                  str(exc), error_code=getattr(exc, "code", "INTERNAL"))
            if job["action"] != "weigh" or getattr(exc, "code", "") not in (
                    "UNSTABLE", "INSUFFICIENT_SAMPLES", "BELOW_RESOLUTION", "OUT_OF_RANGE", "RESIDUAL"):
                session = None
            if trial_path:
                result["trial_file"] = trial_path
        with self._lock:
            if job["cancel"].is_set() or self._closed:
                result = self._result("cancelled", "已取消；本次结果不再有效。")
                session = None
            self._session = session
            if job["action"] == "finish" and result["state"] == "finished":
                with self._loco._lock:
                    self._loco._payload_owner = None
            self._result_profile = p
            self._result_summary = summary if result.get("valid") else None
            result["action_id"] = job["id"]
            self._emit(result)
        if job["notify"]:
            try:
                status = "cancelled" if result["state"] == "cancelled" else "error" if result["state"] in ("error", "uncalibrated") else "completed"
                self._notify(job["id"], status, copy.deepcopy(result))
            except Exception as exc:
                print(f"[payload_mass] completion failed: {exc}", flush=True)

    def stop(self):
        with self._lock:
            self._closed = True
            self.cancel_locked()
            job = self._job
            if job:
                job["published"].set()
                job["notify"] = False
        if job and job["thread"] is not threading.current_thread():
            job["thread"].join(timeout=7)
            if job["thread"].is_alive():
                raise RuntimeError("weighing worker did not stop")
        if self._node is not None:
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            self._node = None
