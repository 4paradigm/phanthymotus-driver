"""Bumi foot observations and an operator-validated factory recording adapter.

No raw joint command, inverse-kinematics guess, or low-controller fallback exists.
An empty asset manifest supports previews only. Physical execution stays blocked.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

from foot_pose_state import describe_feet, source_freshness

POSES = ("feet_together", "feet_apart", "feet_staggered", "neutral")
LABELS = {"feet_together": "双脚靠拢", "feet_apart": "双脚分开",
          "feet_staggered": "双脚一前一后错开", "neutral": "中立站姿"}
LEG_IDS = set(range(4, 10)) | set(range(14, 20))
DEFAULT_ASSETS = "/opt/phanthy-motus/data/foot_pose/assets.json"


def number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite in [{low}, {high}]")
    return float(value)


def load_assets(path):
    """Validation metadata is operator evidence, never inferred from a recording ID."""
    if not path or not Path(path).is_file():
        return {}, "No validated foot-pose assets installed", None
    raw = Path(path).read_bytes()
    if len(raw) > 128 * 1024:
        raise ValueError("Foot-pose manifest exceeds 128 KiB")
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("Foot-pose manifest must be an object")
    if data.get("schema") != "bumi.foot-pose-assets/1" or data.get("robot_model") != "Bumi-EDU":
        raise ValueError("Unsupported foot-pose manifest or robot model")
    if not isinstance(data.get("poses"), dict):
        raise ValueError("poses must be an object")
    if not data["poses"]:
        return {}, "The asset manifest contains no validated motions", hashlib.sha256(raw).hexdigest()
    if not isinstance(data.get("validated_by"), str) or not data["validated_by"].strip():
        raise ValueError("A physical validation operator must be recorded")
    if not isinstance(data.get("verification_record"), str) or not data["verification_record"].strip():
        raise ValueError("A physical validation record must be identified")
    poses = data["poses"]
    if not set(poses) <= set(POSES):
        raise ValueError("Unknown foot-pose name in manifest")
    checked = {}
    ids = set()
    for name, item in poses.items():
        if not isinstance(item, dict) or item.get("verified_on_device") is not True or item.get("safe_stop_verified") is not True:
            raise ValueError(f"{name}: physical motion and safe stop must both be verified")
        index = item.get("recording_id")
        if type(index) is not int or not 0 <= index <= 65535 or index in ids:
            raise ValueError("Recording IDs must be distinct integers in [0, 65535]")
        ids.add(index)
        signature = item.get("joint_signature_rad")
        if (not isinstance(signature, dict) or len(signature) != len(LEG_IDS)
                or {int(k) for k in signature} != LEG_IDS
                or any(str(int(k)) != str(k) for k in signature)):
            raise ValueError(f"{name}: all twelve leg motor positions are required")
        signature = {int(k): number(v, "joint position", -math.pi, math.pi) for k, v in signature.items()}
        min_s = number(item.get("min_elapsed_s"), "min_elapsed_s", 0.2, 25)
        timeout_s = number(item.get("timeout_s"), "timeout_s", 1, 30)
        if timeout_s <= min_s + 0.6:
            raise ValueError("Motion timeout must allow the stable observation window")
        checked[name] = {"recording_id": index, "joint_signature_rad": signature,
                         "tolerance_rad": number(item.get("tolerance_rad"), "tolerance_rad", 0.005, 0.15),
                         "min_elapsed_s": min_s, "timeout_s": timeout_s}
    return checked, None, hashlib.sha256(raw).hexdigest()


def healthy_snapshot(sample, *, stationary=False):
    if not isinstance(sample, dict) or sample.get("fresh") is not True:
        raise RuntimeError("Fresh robot telemetry is required")
    mode = sample.get("workmode", {})
    if not isinstance(mode, dict):
        raise RuntimeError("Robot workmode is unavailable")
    if mode.get("protection") is not False or sample.get("motor_faults") != []:
        raise RuntimeError("Protection or confirmed motor fault blocks foot-pose execution")
    if mode.get("code") not in (2, 23):
        raise RuntimeError("Foot poses require walking or this Card's owned playback mode")
    if stationary and sample.get("activity") != "stationary":
        raise RuntimeError("Robot must be stationary before a foot-pose transition")
    return sample


def signature_matches(sample, profile):
    joints = sample.get("joint_states")
    if not isinstance(joints, list):
        return False
    positions = {j.get("motor_id"): j.get("position") for j in joints if isinstance(j, dict)}
    velocities = {j.get("motor_id"): j.get("velocity") for j in joints if isinstance(j, dict)}
    for motor_id, target in profile["joint_signature_rad"].items():
        value = positions.get(motor_id)
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value - target) > profile["tolerance_rad"]:
            return False
        velocity = velocities.get(motor_id)
        if type(velocity) not in (int, float) or not math.isfinite(velocity) or abs(velocity) >= 0.15:
            return False
    return sample.get("activity") == "stationary"


class FootPosePlugin:
    """One physical body, one operation; shares the existing locomotion lease."""
    PREFIX = "foot_pose"

    def __init__(self, cfg, namespace, executor, high_ctrl, loco_plugin, snapshot_reader=None, notifier=None):
        self._high = high_ctrl
        self._loco = loco_plugin
        self._reader = snapshot_reader
        self._notify = notifier or (lambda *args: None)
        self._lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._worker = None
        self._cancel = threading.Event()
        self._lease = "foot_pose:" + uuid.uuid4().hex
        self._owned = False
        self._dry_run = cfg.get("dry_run", True)
        if type(self._dry_run) is not bool:
            raise ValueError("foot_pose.dry_run must be boolean")
        self._assets_file = str(cfg.get("assets_file", DEFAULT_ASSETS))
        try:
            self._profiles, self._asset_problem, self._asset_hash = load_assets(self._assets_file)
        except (ValueError, TypeError, OSError) as error:
            self._profiles, self._asset_problem, self._asset_hash = {}, str(error), None
        # Real output is never armed just by a YAML flag or a container restart.
        self._armed = False
        self._state = {"state": "idle", "command_sent": False, "pose_confirmed": False, "last_result": None}

    def start(self):
        # The shipped HighController has getters, not a receive callback API.
        return self.info()

    def get_tools(self):
        actions = ["info", "status", "config", "read_feet", "check_pose", "reload_assets", "preview", *POSES, "stop"]
        mapping = {name: {"params": [], "description": LABELS[name] + "；默认仅预览，真实模式需要已验证动作资产。"} for name in POSES}
        mapping.update({"info": {"params": [], "description": "查询已验证动作、真实执行门槛和缺失条件。"},
                        "read_feet": {"params": [], "description": "只读腿部关节，估算机身坐标系中的双脚位置、间距及前后差；不执行动作，不证明着地或平衡。"},
                        "check_pose": {"params": ["pose"], "description": "只读比对已有资产的关节终态；不播放动作，不把缓存匹配当作真机动作完成。"},
                        "reload_assets": {"params": [], "description": "重新载入资产清单，支持逐个姿态接入；同时关闭真实输出授权。"},
                        "status": {"params": [], "description": "查询当前任务和真实完成结果。"},
                        "preview": {"params": ["pose"], "description": "预览动作定义，不调用机器人 SDK。"},
                        "stop": {"params": [], "description": "停止本卡拥有的播放，报告是否返回 walking；不代表所有关节已停止。"}})
        return [{"name": self.PREFIX, "type": "actuator", "multiInstance": False,
                 "description": "Bumi 脚步基础卡：读取双脚位置与间距估计、比对脚姿、预览并载入已验证录制资产。真实脚姿执行需动作资产和可验证的新鲜遥测；不会用普通走动代替指定脚步。",
                 "inputSchema": {"type": "object", "properties": {
                     "action": {"type": "string", "enum": actions},
                     "pose": {"type": "string", "enum": list(POSES)}}, "required": ["action"],
                     "x-action-params": mapping, "x-resource": "base",
                     "x-completion": {"actions": list(POSES), "timeout": 40},
                     "x-hooks": {"on_interrupt_all": {"action": "stop"}}},
                 "configSchema": {"type": "object", "properties": {
                     "dry_run": {"type": "boolean", "default": True, "scope": "shared", "title": "只预览，不执行动作"},
                     "assets_file": {"type": "string", "default": DEFAULT_ASSETS, "scope": "shared", "title": "已验证脚步动作清单"},
                     "confirm": {"type": "boolean", "scope": "shared", "description": "关闭空跑时确认真实输出；缺失或未验证资产仍会被拒绝。"}}}}]

    def _busy(self):
        return self._worker is not None and self._worker.is_alive()

    def _read_sample(self):
        if self._reader is not None:
            sample = self._reader()
            if not isinstance(sample, dict):
                raise ValueError("Telemetry reader must return an object")
            return sample
        # Observation-only fallback uses the actual vendor getter surface.
        getter = getattr(self._high, "get_joint_state", None)
        if not callable(getter):
            raise RuntimeError("SDK joint state reader is unavailable")
        joints = [{"motor_id": int(j.motor_id), "position": float(j.pos),
                   "velocity": float(j.vel)} for j in getter()]
        return {"joint_states": joints, "source_freshness_verified": False}

    def _snapshot(self):
        sample = self._read_sample()
        if not source_freshness(sample)["source_recent"]:
            raise RuntimeError("Verified source receive time is unavailable or stale; physical execution is blocked")
        return healthy_snapshot(sample)

    def read_feet(self):
        return describe_feet(self._read_sample())

    def check_pose(self, pose):
        with self._lock:
            if pose not in POSES:
                raise ValueError("Unknown foot pose")
            if pose not in self._profiles:
                raise ValueError("No validated asset is loaded for this pose")
            sample = self._read_sample()
            result = describe_feet(sample)
            result.update(requested_pose=pose,
                          joint_signature_matches=signature_matches(sample, self._profiles[pose]),
                          verification="read_only_joint_signature_comparison")
            return result

    def reload_assets(self):
        with self._lock:
            if self._busy() or self._owned:
                raise RuntimeError("Stop the owned task before reloading assets")
            # Even an invalid newly edited file must not leave real output armed.
            self._armed = False
            self._dry_run = True
            return self.configure({"dry_run": True, "assets_file": self._assets_file})

    def info(self):
        with self._lock:
            blockers = []
            if self._asset_problem:
                blockers.append(self._asset_problem)
            if not self._profiles:
                blockers.append("No operator-validated physical recording is loaded")
            if self._profiles and "neutral" not in self._profiles:
                blockers.append("A validated neutral recording is required before physical execution")
            if self._loco is None:
                blockers.append("Locomotion ownership adapter is unavailable")
            try:
                self._snapshot()
            except (ValueError, TypeError, RuntimeError, OSError) as error:
                blockers.append(str(error))
            if self._dry_run or not self._armed:
                blockers.append("Physical execution is not armed")
            return {"name": "FootPose", "state": "running" if self._busy() else "ready" if not blockers else "preview_only",
                    "dry_run": self._dry_run, "physical_execution_ready": not blockers,
                    "available_poses": list(self._profiles), "preview_poses": list(POSES),
                    "missing_poses": [p for p in POSES if p not in self._profiles],
                    "observation_available": self._reader is not None or callable(getattr(self._high, "get_joint_state", None)),
                    "assets_file": self._assets_file, "assets_sha256": self._asset_hash,
                    "blockers": blockers, "task": copy.deepcopy(self._state)}

    def dispatch(self, action, args):
        try:
            if action in ("info", "status"):
                return self.info()
            if action == "start":
                return self.start()
            if action == "config":
                return self.configure(args)
            if action == "read_feet":
                return self.read_feet()
            if action == "check_pose":
                return self.check_pose(args.get("pose"))
            if action == "reload_assets":
                return self.reload_assets()
            if action == "stop":
                return self.stop()
            if action == "preview":
                return self.preview(args.get("pose"))
            if action in POSES:
                return self.execute(action)
            raise ValueError("Unknown foot-pose action")
        except (ValueError, TypeError, RuntimeError, OSError) as error:
            return {"state": "error", "command_sent": False, "pose_confirmed": False, "error": str(error)}

    def preview(self, pose):
        if pose not in POSES:
            raise ValueError("Choose feet_together, feet_apart, feet_staggered, or neutral")
        return {"state": "preview", "requested_pose": pose, "description": LABELS[pose],
                "command_sent": False, "pose_confirmed": False, "trajectory_available": pose in self._profiles,
                "note": "This is a pose definition preview, not a kinematic or physical simulation"}

    def configure(self, args):
        with self._lock:
            unknown = set(args) - {"dry_run", "assets_file", "confirm", "_tool_name", "instance_id", "_trace_id"}
            if unknown:
                raise ValueError("Unsupported foot-pose setting: " + sorted(unknown)[0])
            if self._busy() or self._owned:
                raise RuntimeError("Stop the owned foot-pose task before changing configuration")
            dry = args.get("dry_run", self._dry_run)
            if type(dry) is not bool:
                raise ValueError("dry_run must be boolean")
            path = str(args.get("assets_file", self._assets_file))
            profiles, problem, fingerprint = load_assets(path)
            if not dry and (args.get("confirm") is not True or "neutral" not in profiles or self._loco is None):
                raise RuntimeError("Real mode requires explicit confirm, a validated neutral asset, and locomotion ownership")
            if not dry:
                healthy_snapshot(self._snapshot(), stationary=True)
            old = (self._dry_run, self._assets_file, self._profiles, self._asset_problem, self._asset_hash, self._armed)
            self._dry_run, self._assets_file = dry, path
            self._profiles, self._asset_problem, self._asset_hash = profiles, problem, fingerprint
            self._armed = not dry
            try:
                self.start()
            except Exception:
                (self._dry_run, self._assets_file, self._profiles, self._asset_problem, self._asset_hash, self._armed) = old
                raise
            return self.info()

    def _claim(self):
        if self._loco is None:
            raise RuntimeError("Locomotion ownership adapter is unavailable")
        with self._loco._rotate_lock:
            if self._loco._body_action or self._loco.is_moving():
                raise RuntimeError("Another locomotion operation owns the robot")
            self._loco._body_action = self._lease
            self._owned = True

    def _release(self):
        if self._loco is None:
            self._owned = False
            return
        with self._loco._rotate_lock:
            if self._loco._body_action == self._lease:
                self._loco._body_action = None
            self._owned = False

    def execute(self, pose):
        with self._lock:
            if self._busy() or self._owned:
                raise RuntimeError("A foot pose still owns the body; stop it before another transition")
            if self._dry_run:
                return self.preview(pose)
            if not self._armed or pose not in self._profiles:
                raise RuntimeError("No armed, validated physical recording for this pose")
            before = healthy_snapshot(self._snapshot(), stationary=True)
            if before["workmode"]["code"] != 2:
                raise RuntimeError("Begin a foot-pose transition only from walking mode")
            self._claim()
            self._cancel = threading.Event()
            action_id = "footpose-" + uuid.uuid4().hex[:12]
            self._state = {"state": "accepted", "action_id": action_id, "requested_pose": pose,
                           "command_sent": False, "pose_confirmed": False, "last_result": None}
            self._worker = threading.Thread(target=self._run, args=(pose, action_id), name="bumi_foot_pose", daemon=True)
            try:
                self._worker.start()
            except Exception:
                self._release()
                raise
            return {"state": "accepted", "action_id": action_id, "requested_pose": pose,
                    "pose_confirmed": False}

    def _stop_owned_playback(self):
        with self._stop_lock:
            if not self._owned or self._loco._body_action != self._lease:
                return {"command_sent": False, "returned_to_walking": False}
            try:
                mode = int(self._high.get_mode())
                if mode == 23:
                    result = self._loco._do_stop_playback("Validated foot-pose playback cancellation")
                    return {"command_sent": result.get("command_sent", False),
                            "returned_to_walking": result.get("workmode") == 2, "driver_result": result}
                # Never switch modes from protection, disabled, teaching, or another action.
                return {"command_sent": False, "returned_to_walking": mode == 2, "observed_workmode": mode}
            except Exception as error:
                return {"command_sent": None, "returned_to_walking": False, "error": str(error)}

    def _run(self, pose, action_id):
        profile = self._profiles[pose]
        started = time.monotonic()
        stable_since = None
        result = None
        terminal = "failed"
        hold_lease = False
        try:
            healthy_snapshot(self._snapshot(), stationary=True)
            if self._cancel.is_set():
                raise RuntimeError("Foot-pose task was cancelled before dispatch")
            self._loco._preempt_servo("foot_pose." + pose)
            if self._cancel.is_set():
                raise RuntimeError("Foot-pose task was cancelled before dispatch")
            # All commands go through the existing SDK publisher and rate limiter.
            reply = self._loco._trigger_user_action(
                "foot_pose." + pose, "PLAYTEACH", {23}, [],
                "Use only the independently validated recording on its tested Bumi.",
                index=profile["recording_id"], recording_id=profile["recording_id"])
            with self._lock:
                self._state.update(state="running", command_sent=reply.get("command_sent") is True)
            if reply.get("command_sent") is not True or reply.get("confirmed_started") is not True:
                raise RuntimeError("Playback start is unconfirmed; no retry was sent")
            while not self._cancel.is_set():
                elapsed = time.monotonic() - started
                if elapsed > profile["timeout_s"]:
                    raise RuntimeError("Foot pose was not confirmed before timeout")
                sample = self._snapshot()
                matches = elapsed >= profile["min_elapsed_s"] and signature_matches(sample, profile)
                if matches:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= 0.6:
                        hold_lease = sample["workmode"]["code"] == 23
                        terminal = "completed"
                        result = {"pose_confirmed": True, "requested_pose": pose,
                                  "verification": "validated_recording_and_leg_joint_signature",
                                  "foot_positions_directly_measured": False,
                                  "observed_workmode": sample["workmode"]["code"],
                                  "playback_still_owned": hold_lease}
                        break
                else:
                    stable_since = None
                time.sleep(0.05)
            if self._cancel.is_set():
                terminal = "cancelled"
                result = {"pose_confirmed": False, "stop_result": self._stop_owned_playback()}
                hold_lease = self._owned and not result["stop_result"].get("returned_to_walking")
        except Exception as error:
            result = {"pose_confirmed": False, "error": str(error), "stop_result": self._stop_owned_playback()}
            hold_lease = self._owned and not result["stop_result"].get("returned_to_walking")
            if self._cancel.is_set():
                terminal = "cancelled"
        finally:
            if not hold_lease:
                self._release()
            with self._lock:
                self._state.update(state=terminal, pose_confirmed=bool(result and result.get("pose_confirmed")),
                                   last_result=result, finished_at=time.time())
            self._notify(action_id, terminal, result, "foot_pose")

    def stop(self):
        with self._lock:
            self._cancel.set()
            worker = self._worker
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join(timeout=8)
            if worker.is_alive():
                return {"state": "stopping", "pose_confirmed": False, "stopped": False}
        had_ownership = self._owned
        stopped = self._stop_owned_playback()
        confirmed = not had_ownership or stopped.get("returned_to_walking") is True
        if confirmed:
            self._release()
        with self._lock:
            self._state.update(state="stopped" if confirmed else "stop_unconfirmed", pose_confirmed=False)
        return {"state": "stopped" if confirmed else "stop_unconfirmed", "stopped": confirmed, "pose_confirmed": False,
                "stop_result": stopped, "all_joints_stopped_confirmed": False}
