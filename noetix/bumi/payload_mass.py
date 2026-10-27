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

from payload_mass_estimator import (check_baseline, digest, summarize,
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
                 loco_plugin=None, *, publish=None, notify=None, feedback=None):
        self._sampler = sampler
        self._feedback = feedback
        self._loco = loco_plugin
        self._lock = loco_plugin._action_lock if loco_plugin else threading.RLock()
        self._file = cfg.get("profiles_file", "/opt/phanthy-motus/data/bumi/weighing.json")
        self._root = cfg.get("data_dir", "/opt/phanthy-motus/data/bumi/weighing_trials")
        self._identity = cfg.get("hardware_id") or os.environ.get("BUMI_PAYLOAD_ROBOT_ID", "")
        self._sdk = cfg.get("sdk_version") or os.environ.get("BUMI_PAYLOAD_SDK_VERSION", "")
        self._session = None
        self._job = None
        self._threads = []
        self._playback = {"sent": False, "observed": False, "stopped": True}
        self._owned_loaded = None
        self._setup_mode = cfg.get("setup_mode", "profile")
        self._recording_id = cfg.get("recording_id")
        self._arm = cfg.get("arm", "left")
        self._hold_duration_s = cfg.get("hold_duration_s", 300)
        self._quick_loaded = None
        self._stable_window = None
        self._stable_captured_at = None
        self._playback_started = None
        self._closed = False
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
        actions = {"prepare": "曲肘：空载曲肘并保持，记录空载力矩。",
                   "weigh": "测量重量：点击执行后再轻挂物体，数字实时变化，稳定后读取。",
                   "start": "准备卡片。", "stop": "取下物体后停止实时测量，退出回放并核验自然收臂。",
                   "info": "查看实时重量和状态。"}
        if self._setup_mode == "quick":
            actions["calibrate"] = "校准参考重量：首次挂已知重量并稳定后，填参考kg执行一次。"
        params = {a: {"params": ["known_mass_kg"] if a == "calibrate" else [], "description": d}
                  for a, d in actions.items()}
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False,
                "description": "Bumi 实时称重：曲肘保持 → 测量重量（先点击再挂物）→ 取下物体后点击画布停止并自然收臂。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(actions), "oneOf": [
                        {"const": a, "title": actions[a].split("：")[0]} for a in actions]},
                    "known_mass_kg": {"type": "number", "minimum": .001, "maximum": .5,
                                      "description": "仅校准：电子秤测得的参考物体重量，单位kg。"}},
                    "required": ["action"],
                    "x-action-params": params,
                    "x-completion": {"actions": ["prepare", "weigh", "stop"], "timeout": 120},
                    "x-resource": ["arm_l", "arm_r", "base"]},
                "configSchema": {"type": "object", "properties": {
                    "setup_mode": {"type": "string", "scope": "shared", "enum": ["quick", "profile"],
                                   "default": self._setup_mode, "description": "quick=五分钟快速试用；profile=工程标定文件"},
                    "recording_id": {"type": "integer", "scope": "shared", "minimum": 0, "maximum": 65535,
                                     "description": "已录制的五分钟曲肘动作编号。"},
                    "arm": {"type": "string", "scope": "shared", "enum": ["left", "right"],
                            "default": self._arm, "description": "挂载侧：left左臂，right右臂"},
                    "hold_duration_s": {"type": "number", "scope": "shared", "minimum": 10, "maximum": 3600,
                                        "default": self._hold_duration_s, "description": "本段示教回放总时长，默认300秒"},
                    "profiles_file": {"type": "string", "scope": "shared", "default": self._file,
                                      "description": "此机器人固定姿态、示教及标定文件的绝对路径"}}},
                "topic_out": [{"topic": self._topic, "format": "data/json"}]}

    def _load(self):
        if self._setup_mode == "quick":
            return self._load_quick()
        try:
            data = json.loads(Path(self._file).read_text())
            p = validate_profile("weighing", data["profile"], data["profile"]["arm"], loading=True)
            motion = data["motion"]
            if not self._identity or not self._sdk or (p["hardware_id"], p["sdk_version"]) != (self._identity, self._sdk):
                raise MassError("IDENTITY_MISMATCH", "配置须匹配本机 hardware_id 和 SDK/固件版本。")
            if motion.get("verified") is not True or not motion.get("evidence"):
                raise MassError("MOTION_UNVERIFIED", "尚未验证曲肘持续保持和停止后自然归位。")
            if type(motion["bend_recording_id"]) is not int or not 0 <= motion["bend_recording_id"] <= 65535:
                raise MassError("BAD_PARAMETER", "曲肘示教编号必须为0..65535整数。")
            if motion.get("return_on_stop_verified") is not True:
                raise MassError("MOTION_UNVERIFIED", "停止回放后的自然收臂尚未验证。")
            motion = dict(motion)
            motion["home_q"] = vector(motion["home_q"], 21, "home_q")
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

    def _load_quick(self):
        from payload_mass_quick import profile_from_home
        if type(self._recording_id) is not int or not 0 <= self._recording_id <= 65535:
            raise MassError("NOT_CONFIGURED", "在卡片齿轮里填写五分钟曲肘动作的recording_id即可开始。")
        hold = finite(self._hold_duration_s, "hold_duration_s", 10, 3600)
        fingerprint = digest({"recording_id": self._recording_id, "arm": self._arm, "hold_s": hold})
        if self._quick_loaded is None:
            d = self._sampler.diagnostics()
            if d["host_age_s"] is None or d["host_age_s"] > .5:
                raise MassError("STALE_SOURCE", "等待机器人关节反馈。")
            p = profile_from_home(d["latest"], self._arm, hold)
            motion = {"bend_recording_id": self._recording_id, "home_q": list(p["pose_q"]), "timeout_s": 30}
            self._quick_loaded = ({"profile": p, "model": None}, p, motion, 3.0, fingerprint)
        if self._quick_loaded[-1] != fingerprint:
            raise MassError("CONFIG_CHANGED", "配置改变，请停止后重新曲肘。")
        return self._quick_loaded

    def _remaining_s(self):
        if (self._setup_mode != "quick" or self._playback_started is None
                or not self._loco or self._loco._payload_owner is not self):
            return None
        return max(0.0, float(self._hold_duration_s) - (time.monotonic() - self._playback_started))

    def start(self):
        return self._feedback.start() if self._feedback is not None else True

    def _fresh(self, p):
        d = self._sampler.diagnostics()
        if d["host_age_s"] is None or d["host_age_s"] > p["max_receive_gap_s"]:
            raise MassError("STALE_SOURCE", "未收到及时反馈。")
        return d

    def _tick(self):
        with self._lock:
            running = self._job and self._job["thread"].is_alive()
            if self._session and not running:
                try:
                    self._verify_session(self._session)
                except Exception as exc:
                    self._session = None
                    self._emit(self._result("error", str(exc), error_code=getattr(exc, "code", "INVALID_STATE")))
            self._emit(self._last)

    def _verify_session(self, session):
        p = session["p"]
        d = self._fresh(p)
        check_frame(d["latest"], p)
        check_baseline(session["baseline"], summarize([d["latest"]], p), p, time.time())
        if d["read_error_count"] != session["error_epoch"]:
            raise MassError("STALE_SOURCE", "准备后反馈曾中断，请重新采空载。")
        if time.monotonic() - session["prepared"] > session["p"]["baseline_max_age_s"]:
            raise MassError("PREPARATION_EXPIRED", "本次空载记录已过期。")
        if self._load()[-1] != session["fingerprint"]:
            raise MassError("CONFIG_CHANGED", "配置改变，请先停止称重。")
        return d

    def info(self):
        self._tick()
        with self._lock:
            result = copy.deepcopy(self._last)
            result.update(prepared=bool(self._session), profiles_file=self._file,
                          active=bool(self._job and self._job["thread"].is_alive()),
                          motion_reserved=bool(self._loco and self._loco._payload_owner is self),
                          playback=copy.deepcopy(self._playback),
                          setup_mode=self._setup_mode, recording_id=self._recording_id,
                          remaining_s=self._remaining_s(),
                          diagnostics=self._sampler.diagnostics() if self._sampler else None)
            return result

    def _new_job(self, action, loaded, gate, session=None):
        job = {"id": "weigh_" + uuid4().hex, "cancel": threading.Event(),
               "published": threading.Event(), "notify": True, "notified": False,
               "action": action, "loaded": loaded, "session": copy.deepcopy(session)}
        if not gate:
            job["published"].set()
        job["thread"] = threading.Thread(target=self._run, args=(job,), daemon=True, name="bumi_weighing")
        self._job = job
        self._threads = [j for j in self._threads if j["thread"].is_alive()] + [job]
        message = {"prepare": "正在曲肘和采集空载，请等待就绪。",
                   "weigh": "实时测量已启动，请现在轻挂物体；数字稳定后读取。",
                   "finish": "正在停止测量并核验收臂，请等待。"}[action]
        result = self._emit(self._result("measuring" if action == "weigh" else "running", message,
                                         stable=False, action_id=job["id"]))
        job["thread"].start()
        return result

    def dispatch(self, action, args):
        args = dict(args)
        args.pop("_tool_name", None)
        args.pop("instance_id", None)
        gate = args.pop("_await_response_publication", False)
        # The canvas retains hidden parameter inputs after switching actions.
        # Reference truth affects calibration ONLY, never unknown weighing.
        if self._setup_mode == "quick" and action != "calibrate":
            args.pop("known_mass_kg", None)
        with self._lock:
            if action == "info":
                return self.info()
            if self._closed:
                return self._result("error", "卡片已关闭。", error_code="CLOSED")
            if action == "start":
                return self._result("ready", "请先空载曲肘；就绪后点击测量重量，再挂物。")
            try:
                if action == "calibrate":
                    return self._calibrate_quick(args)
                if action in ("stop", "finish"):
                    if args:
                        raise MassError("BAD_PARAMETER", "停止不接受额外参数。")
                    if not self._loco or self._loco._payload_owner is not self:
                        return self._emit(self._result("idle", "当前没有本卡动作。", stable=False))
                    if self._job and self._job["thread"].is_alive():
                        if self._job["action"] == "finish":
                            return copy.deepcopy(self._last)
                        self._job["cancel"].set()
                    self._session = None
                    # Return uses the configuration captured on admission, even
                    # if a file is changed/deleted while an object is hanging.
                    return self._new_job("finish", self._owned_loaded, gate)
                if self._job and self._job["thread"].is_alive():
                    return self._result("error", "任务正在执行；取下物体后点击画布停止。", error_code="BUSY")
                if action == "config":
                    if self._loco and self._loco._payload_owner is self:
                        raise MassError("BUSY", "请先停止称重再修改配置。")
                    self._configure(args)
                    return self._emit(self._result("configured", "配置已更新。"))
                if action not in ("prepare", "weigh"):
                    return None
                if args:
                    raise MassError("BAD_PARAMETER", "不接受额外参数或待测物体真值。")
                if self._loco is None or self._sampler is None:
                    raise MassError("UNAVAILABLE", "需要现有state和loco插件。")
                if action == "prepare" and self._setup_mode == "quick":
                    if self._loco._payload_owner is not None:
                        raise MassError("ALREADY_PREPARED", "请先卸物并停止，再开始下一次测量。")
                    self._quick_loaded = None
                    self._stable_window = None
                    self._playback_started = None
                loaded = self._load()
                data, p, motion, duration, fingerprint = loaded
                if action == "prepare":
                    if self._loco._payload_owner is not None:
                        raise MassError("ALREADY_PREPARED", "请先取下物体并停止，再采新的空载。")
                    if int(self._loco._high_ctrl.get_mode()) != 2:
                        raise MassError("MODE_MISMATCH", "请先稳定站立；不会中断其他回放。")
                    check_frame(self._fresh(p)["latest"], {**p, "pose_q": motion["home_q"], "workmode": 2})
                    # _lock below is released BEFORE acquiring the servo lock.
                    # The callback takes servo -> loco._lock; never reverse it.
                    with self._loco._lock:
                        self._loco._payload_owner = self
                    self._owned_loaded = loaded
                    self._playback = {"sent": False, "observed": False, "stopped": True}
                    self._loco._preempt_servo("payload_mass")
                    if self._loco.is_moving():
                        self._loco._stop_move()
                else:
                    if not self._session:
                        raise MassError("NOT_PREPARED", "请先空载曲肘。")
                    d = self._verify_session(self._session)
                    if any(abs(d["latest"]["tau"][j] - self._session["baseline"]["summary"]["tau"][i]) > p["max_torque_span"]
                           for i, j in enumerate(p["motor_ids"])):
                        raise MassError("NOT_EMPTY", "请先取下物体，点击测量重量后再挂物；不会重新去皮。")
                return self._new_job(action, loaded, gate, self._session)
            except Exception as exc:
                return self._emit(self._result("error", str(exc), stable=False,
                                               error_code=getattr(exc, "code", "INVALID_CONFIG")))

    def response_published(self, action_id, delivered=True):
        with self._lock:
            if self._job and self._job["id"] == action_id:
                if not delivered:
                    self._job["cancel"].set()
                    self._job["notify"] = False
                self._job["published"].set()

    def _stop_playback_locked(self):
        mode = int(self._loco._high_ctrl.get_mode())
        if mode == 23:
            self._playback["observed"] = True
            result = self._loco._do_stop_playback("结束称重并归位")
            if not result.get("confirmed") or int(self._loco._high_ctrl.get_mode()) != 2:
                raise MassError("STOP_UNCONFIRMED", "回放停止未确认，保留控制权。")
            self._playback["stopped"] = True
        elif mode == 2:
            if self._playback["sent"] and not self._playback["observed"] and not self._playback["stopped"]:
                raise MassError("STOP_UNCONFIRMED", "回放命令已发送，仍在等待启动反馈，不能提前释放控制权。")
            self._playback["stopped"] = True
        else:
            raise MassError("STOP_UNCONFIRMED", f"当前模式{mode}不能确认回放停止。")

    def cancel_locked(self):
        """Emergency/external cancellation; canvas stop uses verified return."""
        if self._job:
            self._job["cancel"].set()
            self._job["suppress_result"] = True
        self._session = None
        if self._loco and self._loco._payload_owner is self:
            try:
                self._stop_playback_locked()
            except Exception as exc:
                return self._emit(self._result("error", str(exc), error_code="STOP_UNCONFIRMED", stable=False))
            with self._loco._lock:
                self._loco._payload_owner = None
        return self._emit(self._result("cancelled", "已中断测量和本卡回放。", stable=False))

    def _check_job(self, job):
        if job["cancel"].is_set() or self._closed or self._job is not job:
            raise MassError("CANCELLED", "已取消测量。")
        if job["action"] != "finish" and self._load()[-1] != job["loaded"][-1]:
            raise MassError("CONFIG_CHANGED", "采样期间配置改变。")
        if job["action"] != "finish" and self._remaining_s() == 0:
            raise MassError("RECORDING_ENDED", "示教时间已到，请卸物并点击停止。")

    def _configure(self, args):
        allowed = {"profiles_file", "setup_mode", "recording_id", "arm", "hold_duration_s"}
        if not args or set(args) - allowed:
            raise MassError("BAD_PARAMETER", "不支持的配置字段。")
        mode = args.get("setup_mode", self._setup_mode)
        arm = args.get("arm", self._arm)
        recording = args.get("recording_id", self._recording_id)
        hold = finite(args.get("hold_duration_s", self._hold_duration_s), "hold_duration_s", 10, 3600)
        file = args.get("profiles_file", self._file)
        if mode not in ("quick", "profile") or arm not in ("left", "right"):
            raise MassError("BAD_PARAMETER", "请选择quick/profile及left/right。")
        if recording is not None and (type(recording) is not int or not 0 <= recording <= 65535):
            raise MassError("BAD_PARAMETER", "示教编号必须为0..65535整数。")
        if not isinstance(file, str) or not Path(file).is_absolute():
            raise MassError("BAD_PARAMETER", "工程配置文件须为绝对路径。")
        self._setup_mode, self._arm, self._recording_id = mode, arm, recording
        self._hold_duration_s, self._file = hold, file
        self._quick_loaded, self._stable_window, self._playback_started = None, None, None

    def _calibrate_quick(self, args):
        from payload_mass_quick import fit_reference
        if self._setup_mode != "quick" or set(args) != {"known_mass_kg"}:
            raise MassError("BAD_PARAMETER", "快速校准需要known_mass_kg，范围0.001..0.5。")
        if not self._session or not self._job or self._job["action"] != "weigh" or not self._job["thread"].is_alive():
            raise MassError("NOT_MEASURING", "先曲肘、启动测量，再挂参考物体。")
        p = self._session["p"]
        self._check_job(self._job)
        if (not self._stable_window or self._stable_captured_at is None
                or time.monotonic() - self._stable_captured_at > p["max_receive_gap_s"]):
            raise MassError("UNSTABLE", "参考物体尚未稳定，请稍等后执行校准。")
        latest = self._fresh(p)["latest"]
        check_frame(latest, p)
        center = summarize(self._stable_window, p)["tau"]
        if any(abs(latest["tau"][j] - center[i]) > p["max_torque_span"] for i, j in enumerate(p["motor_ids"])):
            raise MassError("UNSTABLE", "参考载荷已经变化，请重新等待稳定。")
        model = fit_reference(self._stable_window, self._session["baseline"], p, args["known_mass_kg"])
        self._job["loaded"][0]["model"] = model
        return {"state": "calibrated", "unit": "kg", "reference_mass_kg": model["reference_mass_kg"],
                "quality": model["quality"], "validation_status": "not_validated",
                "message": "本次快速校准完成；取下参考物体后，直接换待测物体，数字会持续刷新。"}

    def _pose(self, job):
        _, p, motion, _, _ = job["loaded"]
        home = job["action"] == "finish"
        profile = {**p, "pose_q": motion["home_q"] if home else p["pose_q"],
                   "workmode": 2 if home else p["workmode"]}
        if not home:
            with self._lock:
                self._check_job(job)
                if int(self._loco._high_ctrl.get_mode()) != 2:
                    raise MassError("MODE_MISMATCH", "曲肘只从walking模式开始。")
                check_frame(self._fresh(p)["latest"], {**p, "pose_q": motion["home_q"], "workmode": 2})
                # Set the latch BEFORE publishing. Exceptions and a delayed
                # mode transition must retain ownership until resolved.
                self._playback_started = time.monotonic()
                self._playback = {"sent": True, "observed": False, "stopped": False}
                result = self._loco._do_teaching_action("play_recording", {"recording_id": motion["bend_recording_id"]})
                if result.get("command_sent") is False:
                    self._playback = {"sent": False, "observed": False, "stopped": True}
                if result.get("workmode") == 23 and result.get("confirmed_started"):
                    self._playback["observed"] = True
                if result.get("state") == "error":
                    raise MassError("MOTION_FAILED", str(result))
        deadline = time.monotonic() + motion["timeout_s"]
        last_error = None
        while time.monotonic() < deadline:
            self._check_job(job)
            try:
                with self._lock:
                    self._check_job(job)
                    mode = int(self._loco._high_ctrl.get_mode())
                    if mode in (26, 30):
                        raise MassError("MOTION_FAILED", "机器人进入保护或失能模式。")
                    if mode == 23:
                        self._playback["observed"] = True
                    if home:
                        self._stop_playback_locked()
                    frame = self._fresh(profile)["latest"]
                    if not home and self._setup_mode == "quick":
                        elbow_id = p["motor_ids"][-1]
                        if abs(frame["q"][elbow_id] - motion["home_q"][elbow_id]) < .2:
                            raise MassError("POSE_MISMATCH", "等待实际曲肘。")
                        profile = {**profile, "pose_q": list(frame["q"])}
                    check_frame(frame, profile)
                    return profile
            except MassError as exc:
                if exc.code not in ("POSE_MISMATCH", "MODE_MISMATCH", "UNSTABLE", "STOP_UNCONFIRMED"):
                    raise
                last_error = exc
            job["cancel"].wait(.05)
        if last_error and last_error.code == "STOP_UNCONFIRMED":
            raise last_error
        raise MassError("POSE_TIMEOUT", "未观测到目标姿态；未确认曲肘保持或自然收臂。")

    def _notify_once(self, job, status, result):
        with self._lock:
            if not job["notify"] or job["notified"]:
                return
            job["notified"] = True
        try:
            self._notify(job["id"], status, copy.deepcopy(result))
        except Exception as exc:
            print(f"[payload_mass] completion failed: {exc}", flush=True)

    def _live(self, job):
        from payload_mass_live import evaluate_window
        data, p, _, duration, fingerprint = job["loaded"]
        session = job["session"]
        d = self._fresh(p)
        cursor = d["received_count"]
        window = [d["latest"]]
        emitted_at = 0.0
        was_stable = False
        saved_center = None
        saved_count = 0
        # Completion means streaming has STARTED, not that a future hanging
        # object is already weighed. Never hold an ACP barrier until Stop.
        started = self._result("measuring", "实时测量已启动，请现在轻挂物体。", stable=False, stream_id=job["id"])
        with self._lock:
            self._check_job(job)
            self._emit(started)
        self._notify_once(job, "completed", started)
        while not job["cancel"].is_set():
            self._check_job(job)
            d = self._fresh(p)
            if d["read_error_count"] != session["error_epoch"]:
                raise MassError("DATA_ERROR", "空载后反馈曾中断，不能继续沿用基线。")
            if time.monotonic() - session["prepared"] > session["p"]["baseline_max_age_s"]:
                raise MassError("PREPARATION_EXPIRED", "空载已过期，请取下物体并停止。")
            incoming = self._sampler.read_since(cursor, session["error_epoch"])
            for frame in incoming:
                if frame["host_monotonic_s"] - window[-1]["host_monotonic_s"] > p["max_receive_gap_s"]:
                    raise MassError("STALE_SOURCE", "连续反馈出现空档。")
                # Hard faults/pose changes abort immediately, even between two
                # display ticks. Ordinary hand loading may be temporarily moving.
                try:
                    check_frame(frame, p)
                except MassError as exc:
                    if exc.code != "UNSTABLE":
                        raise
                window.append(frame)
                cursor = frame["serial"]
                cutoff = frame["host_monotonic_s"] - duration
                while len(window) > 1 and window[1]["host_monotonic_s"] < cutoff:
                    window.pop(0)
            now = time.monotonic()
            if incoming and now - emitted_at >= .2:
                result = evaluate_window(window, session["baseline"], data.get("model"), p, duration, time.time())
                result.update(stream_id=job["id"], arm=p["arm"], baseline_id=session["baseline"]["id"])
                if self._setup_mode == "quick":
                    result.update(remaining_s=round(self._remaining_s(), 1), experimental=True,
                                  quality="experimental_single_reference", validation_status="not_validated",
                                  unknown_status_codes=sorted(set(window[-1]["errors"]) - {0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14}))
                center = summarize(window, p)["tau"]
                save = (result["sampling_stable"] and saved_count < 256 and
                        (not was_stable or saved_center is None or
                         any(abs(a-b) > p["max_torque_span"] for a, b in zip(center, saved_center))))
                if save:
                    trial = {"id": uuid4().hex, "profile": p, "profile_fingerprint": p["fingerprint"],
                             "action": "weigh", "created_s": time.time(), "summary": summarize(window, p),
                             "raw_frames": copy.deepcopy(window), "baseline": session["baseline"],
                             "stream_id": job["id"]}
                with self._lock:
                    self._check_job(job)
                    if self._setup_mode == "quick":
                        self._stable_window = copy.deepcopy(window) if result["sampling_stable"] else None
                        self._stable_captured_at = time.monotonic() if result["sampling_stable"] else None
                    if save:
                        result["trial_file"] = save_trial(self._root, trial)
                        saved_center, saved_count = center, saved_count + 1
                    self._emit(result)
                was_stable = result["sampling_stable"]
                emitted_at = now
            job["cancel"].wait(.05)

    def _run(self, job):
        _, p, _, duration, fingerprint = job["loaded"]
        session = job["session"]
        try:
            if not job["published"].wait(5):
                job["notify"] = False
                raise MassError("RESPONSE_TIMEOUT", "未送达动作确认，未启动动作。")
            self._check_job(job)
            if job["action"] == "weigh":
                self._live(job)
                raise MassError("CANCELLED", "实时测量已停止。")
            sample_profile = self._pose(job)
            if job["action"] == "prepare" and self._setup_mode == "quick":
                capture_profile = {**sample_profile,
                                   "settle_timeout_s": job["loaded"][2]["timeout_s"]}
                frames = self._sampler.collect(duration, capture_profile, job["cancel"],
                                               learn_pose_from=job["loaded"][2]["home_q"])
            else:
                frames = self._sampler.collect(duration, sample_profile, job["cancel"])
            if job["action"] == "prepare" and self._setup_mode == "quick":
                from payload_mass_quick import capture_bend_profile
                p = capture_bend_profile(sample_profile, frames)
                with self._lock:
                    self._check_job(job)
                    data, _, motion, _, fingerprint = job["loaded"]
                    data["profile"] = p
                    job["loaded"] = (data, p, motion, duration, fingerprint)
                    self._quick_loaded = self._owned_loaded = job["loaded"]
            summary = summarize(frames, sample_profile)
            self._check_job(job)
            if job["action"] == "finish":
                result = self._result("finished", "测量已停止，已核验手臂自然下垂。", home_confirmed=True, stable=False)
                session = None
            else:
                trial = {"id": uuid4().hex, "profile": p, "profile_fingerprint": p["fingerprint"],
                         "action": "prepare", "created_s": time.time(), "summary": summary, "raw_frames": frames}
                baseline = {"id": trial["id"], "created_s": time.time(),
                            "profile_fingerprint": p["fingerprint"], "summary": summary}
                session = {"baseline": baseline, "p": p, "fingerprint": fingerprint,
                           "prepared": time.monotonic(), "error_epoch": self._sampler.diagnostics()["read_error_count"]}
                result = self._result("ready", "曲肘保持且空载已记录；先点击测量重量，再轻挂物体。",
                                      trial_file=save_trial(self._root, trial), stable=False)
        except Exception as exc:
            result = self._result("cancelled" if getattr(exc, "code", "") == "CANCELLED" else "error", str(exc),
                                  error_code=getattr(exc, "code", "INTERNAL"), stable=False)
            session = None
        with self._lock:
            if self._job is job and not job.get("suppress_result"):
                if job["cancel"].is_set() or self._closed:
                    result = self._result("cancelled", "本次任务已取消。", stable=False)
                    session = None
                self._session = session
                if job["action"] == "finish" and result["state"] == "finished":
                    with self._loco._lock:
                        self._loco._payload_owner = None
                result["action_id"] = job["id"]
                self._emit(result)
        self._notify_once(job, "cancelled" if job["cancel"].is_set() else "error" if result["state"] == "error" else "completed", result)

    def stop(self):
        with self._lock:
            self._closed = True
            self.cancel_locked()
            jobs = list(self._threads)
            for job in jobs:
                job["cancel"].set()
                job["published"].set()
                job["notify"] = False
        for job in jobs:
            if job["thread"] is not threading.current_thread():
                job["thread"].join(timeout=7)
                if job["thread"].is_alive():
                    raise RuntimeError("weighing worker did not stop")
        if self._feedback is not None:
            self._feedback.stop()
        if self._node is not None:
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            self._node = None
