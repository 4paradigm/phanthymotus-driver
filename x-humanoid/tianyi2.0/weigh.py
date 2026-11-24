"""x-humanoid/tianyi2.0/weigh.py — 天轶2.0 腕部六维力托举称重卡（实验性）。

用法（上层 Skill / 大模型）：
  0. 每个动作都必须填写 ``side``（left / right / both），并与 pose 时一致；
  1. ``pose``：把手臂摆到托举姿态（掌心朝上），再把四指立起围住掌心（``hand=cup``），
     然后自动 ``tare``（必做；掌心须空着，去皮失败则 pose 失败）；
  2. 也可以分开调用 ``tare`` 去皮；
  3. 放上物体后调用 ``weigh``，得到重量；
  4. ``calibrate`` 设定比例系数：放已知重量物体实测（known_mass_kg），或直接写入（scale）；
  5. ``read`` 不阻塞，返回去皮状态、上一次结果和实时读数；
  6. ``retract``：称完收回手臂（掌心上还有物体时拒绝，避免掉落；张不开手指时不动手臂）。

pose / retract / tare / weigh / calibrate 超过约 0.5 s 时走 ACP：先返回 ``action_id``，
做完后把完整结果 POST 到 Agent Core 的 /api/acp/complete（见 README_dev.md § ACP）。

只有 ``pose`` / ``retract`` 会让手臂和手指运动，而且不自己发指令：经 bundle 调用 arm 卡的
``move_pos``（复用它的急停/故障/限位预检、电流上限和运动反馈）和 hand 卡的 ``set_fingers_raw``。其余动作只订阅 ``/arm_6dof_{side}``（WrenchStamped，约 100 Hz）
和 ``/arm/status``（MotorStatusMsg，抽帧到约 50 Hz），用本卡自己的执行器线程。

原理：传感器读数 = 零点偏置 + 手自身重力 + 物体重力。姿态不变时前两项不变，
所以 mass = |F − F₀| / g / scale，用三维力向量，不只看 fz。姿态一变 F₀ 就失效，
因此去皮时记录 7 个关节角，称重时偏差超过阈值就判无效。

2026-11-23 可行性测试（右手，肘 −90°、腕 yaw −90°，0.2–1.28 kg）：
  - 原始读数偏高约 7%（scale ≈ 1.068，见 DEFAULT_SCALE），0.2–1.2 kg 符合同一个比例；
  - 20 分钟内零点漂移 ≤ 8 g；空载 |ΔF| 有 5–8 g 的正偏差（噪声向量的长度）；
  - 1.28 kg 时肩、肘角度不变，腕 roll 下压 0.76°，电流约为额定的 40%。
真实重量当时是估计值，精度待用电子秤复测，所以结果一律标 ``experimental``。

计算部分（``WeighEngine``）不依赖 ROS，单元测试见 tests/test_tianyi_weigh.py。
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time
from collections import deque
from pathlib import Path
from uuid import uuid4

G = 9.81

SIDES = ("right", "left")

ARM_JOINT_IDS = {
    "right": (21, 22, 23, 24, 25, 26, 27),
    "left": (11, 12, 13, 14, 15, 16, 17),
}

# 与 device.py 的 _RATED_MOTOR_CURRENT_A 相同（天轶 2.0 关节规格表）。
# 这里单独保留一份，让本模块不依赖 rclpy 也能导入和测试。
RATED_CURRENT_A = {
    11: 35.0, 12: 23.0, 13: 8.0, 14: 8.0, 15: 8.0, 16: 5.0, 17: 5.0,
    21: 35.0, 22: 23.0, 23: 8.0, 24: 8.0, 25: 8.0, 26: 5.0, 27: 5.0,
}

FORCE_TOPICS = {"right": "/arm_6dof_right", "left": "/arm_6dof_left"}
ARM_STATUS_TOPIC = "/arm/status"

# 2026-11-23 用一加 13（官方 213 g）称 5 次得 1.068 ± 0.021；同一系数下 570 ml 百岁山
# 在 cup 手型中读数 602 g（按整箱 14.72 kg 推算约 600 g），两种参照一致。未经电子秤验证。
DEFAULT_SCALE = 1.068
DEFAULT_SCALE_SOURCE = "default_phone_ref_2026-11-23"

DEFAULTS = {
    "sides": ["left", "right"],      # 订阅并允许使用的手
    "state_file": "/opt/phanthy-motus/data/tianyi-weigh.json",
    "default_scale": DEFAULT_SCALE,
    "scale_min": 0.85,
    "scale_max": 1.15,
    "block_s": 0.5,                  # 分块长度
    "block_step_n": 0.15,            # 相邻块均值差上限（约 15 g）
    "window_drift_n": 0.25,          # 任一块与首块的差上限
    "block_sd_n": 0.5,               # 块内标准差上限
    "min_rate_hz": 80.0,             # 力数据最低频率
    "max_gap_s": 0.5,                # 力/关节数据断流上限
    "joint_speed_rad_s": 0.02,       # 关节静止判据
    "joint_span_deg": 0.5,           # 窗口内关节角跨度上限
    # 相对去皮时的关节偏差上限。重力方向偏 θ 时读数误差约 1−cos θ：1.5° 时 < 0.04%，可以忽略；
    # 0.5° 太紧——600 g 物体就把腕部压下 0.3–0.4°，1.28 kg 压下 0.76°。
    "joint_dev_deg": 1.5,
    # 电流 / 额定电流 上限，同时充当负载上限（腕部电机最先吃紧）。实测 0.6 kg 约 19–24%，
    # 1.28 kg（掌心摊开）约 40%；60% 按比例估算掌心约 2 kg，未实测。
    "current_ratio_max": 0.6,
    "tare_ttl_s": 900.0,             # 去皮有效期
    "load_limit_kg": None,           # 固定重量上限（kg）；None = 不设，由 current_ratio_max 判定
    "resolution_kg": 0.02,           # 分辨率；低于此值标 below_resolution
    "min_calibration_kg": 0.2,       # 实测校准的最小参考质量
    "max_calibration_kg": 3.0,       # 未设 load_limit_kg 时，实测校准参考质量的上限（防输错单位）
    "default_duration_s": 3.0,
    "default_timeout_s": 15.0,
    "max_timeout_s": 45.0,           # agent-core 对 actuator 调用的 HTTP 超时是 60 s
    "data_grace_s": 2.0,             # 刚启动时等数据到达的宽限
    "history_s": 12.0,               # 缓冲区保留时长（≥ 最长窗口 10 s；/arm/status 约 500 Hz）
    "poll_s": 0.2,
    "joint_min_interval_s": 0.02,    # /arm/status 抽帧间隔（原始约 490 Hz）
    # ── pose / retract（只支持右手）──
    # 托举姿态：2026-11-23 可行性测试实测姿态，[肩pitch,肩roll,肩yaw,肘pitch,腕yaw,腕pitch,腕roll]（度）
    "pose_deg": [5.0, 5.6, 0.0, -90.0, -90.0, 9.0, 0.1],
    "rest_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 没有记录到出发姿态时，retract 回到这里
    "rest_zone_deg": 20.0,           # pose 只允许从"手臂基本垂下"出发：每个关节离 rest_deg 不超过此值
    "pose_speed": 0.3,               # rad/s，与可行性测试一致
    "arrive_tol_deg": 3.0,           # 到位容差（与 arm 卡的反馈容差一致）
    "arrive_speed_rad_s": 0.02,
    "pose_settle_s": 1.0,            # 到位后静置再去皮
    "pose_extra_timeout_s": 8.0,     # 每段超时 = 最大角度差 / 速度 + 此值
    "arm_start_settle_s": 1.0,       # arm 卡刚被懒启动时，等发布者被发现
    "retract_max_load_kg": 0.05,     # 掌上读数超过此值拒绝收回
    # pose 的手型（灵巧手卡 set_fingers_raw，0=张开 100=握紧）。cup：四指立起围住掌心，
    # 2026-11-23 现场确认 65 合适；物体落在掌心、力臂一致，比摊开时读数稳定。
    "hand_shapes": {
        "cup": {"little": 65, "ring": 65, "middle": 65, "index": 65, "thumb_bend": 0, "thumb_rotation": 0},
        "open": {"little": 0, "ring": 0, "middle": 0, "index": 0, "thumb_bend": 0, "thumb_rotation": 0},
    },
    "default_hand": "cup",
    "hand_settle_s": 1.5,            # 手指动作后静置再去皮 / 再动手臂
}

# 托举序列的中间段：先抬肘、掌心朝内（腕 yaw 保持出发值），再转腕 yaw 让掌心朝上。
# 2026-11-23 现场确认：腕 yaw 负方向掌心朝上；先转腕再抬肘则掌心朝下，所以顺序不能反。
_WRIST_YAW = 4

# 这些原因说明"再等也没用"，立即返回。
_FATAL = {"no_force_data", "no_joint_data", "joint_error"}


def _log(message: str) -> None:
    print(message, flush=True)


def _fmt_deg(values) -> str:
    if not values:
        return "None"
    return "[" + ",".join(f"{float(v):.1f}" for v in values) + "]"


def _brief(result) -> str:
    """调用日志用的一行摘要：只取排查需要的字段。"""
    if not isinstance(result, dict):
        return repr(result)[:200]
    keys = ("state", "code", "error", "valid", "stable", "mass_g", "raw_kg", "scale", "scale_source",
            "calibrated", "reasons", "warnings", "noise_kg", "max_joint_dev_deg", "max_current_ratio",
            "lever_arm_m", "tare_age_s", "samples", "waited_s", "timed_out", "rate_hz",
            "old_scale", "reference_mass_kg", "computed_scale", "load_kg", "already_in_pose",
            "already_retracted", "tared", "tare_valid", "joints_deg")
    parts = [f"{k}={result[k]}" for k in keys if k in result and result[k] not in (None, [], "")]
    if isinstance(result.get("tare"), dict):
        parts.append("tare={" + _brief(result["tare"]) + "}")
    if isinstance(result.get("steps"), list):
        parts.append("steps=" + ";".join(
            f"{x.get('stage')}:{'ok ' + str(x.get('seconds')) + 's' if x.get('ok') else x.get('code')}"
            for x in result["steps"]))
    if isinstance(result.get("live"), dict):
        live = result["live"]
        parts.append(f"live=rate {live.get('rate_hz')}Hz issues {live.get('issues')}")
    if isinstance(result.get("provisional"), dict):
        parts.append(f"provisional_kg={result['provisional'].get('mass_kg')}")
    if "provisional_mass_kg" in result:
        parts.append(f"provisional_kg={result['provisional_mass_kg']}")
    return " ".join(parts)[:900]


def _iso(ts: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(ts))


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _norm(v) -> float:
    return math.sqrt(sum(x * x for x in v))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _mean_rows(rows, n):
    return [sum(r[i] for r in rows) / len(rows) for i in range(n)]


def _r(x, nd=4):
    return None if x is None else round(float(x), nd)


class Cancelled(BaseException):
    """卡片 stop 时取消进行中的动作。继承 BaseException，免得被各处的 except Exception 吞掉。"""


class WeighEngine:
    """缓冲 + 稳定判据 + 去皮/称重/校准。线程安全；不依赖 ROS。

    ``clock`` 是单调时钟，``wall`` 是墙钟（只用于记录），``sleep`` 用于等待。
    测试时三者都可以替换成假时钟。
    """

    def __init__(self, config: dict | None = None, clock=time.monotonic, wall=time.time,
                 sleep=time.sleep, log=_log, arm_call=None, hand_call=None, servo_call=None):
        cfg = dict(DEFAULTS)
        cfg.update(config or {})
        cfg["sides"] = [s for s in SIDES if s in (cfg.get("sides") or SIDES)] or ["right"]
        self.cfg = cfg
        self._clock = clock
        self._wall = wall
        self._raw_sleep = sleep
        self._log = log
        self._cancel = threading.Event()             # stop() 置位：不再下发任何手臂/手指指令
        self._lock = threading.Lock()
        self._force = {s: deque() for s in SIDES}    # (t, fx, fy, fz, tx, ty, tz)
        self._joints = {s: deque() for s in SIDES}   # (t, pos[7], speed[7], current[7], error_max)
        self._latest = {}                            # motor_id → (pos, speed, current, error)
        self._latest_t = {}                          # motor_id → 收到时间
        self._arm_call = arm_call                    # args → arm 卡 dispatch 结果（经 bundle）
        self._arm_started = False
        self._hand_call = hand_call                  # args → hand 卡 dispatch 结果（经 bundle）
        self._hand_started = False
        self._servo_call = servo_call                # args → servo 卡 dispatch 结果；servo 卡未加载时返回 None
        self._origin = {}                            # side → pose 出发时的手臂姿态（度），retract 回到这里
        self._tare = {s: None for s in SIDES}
        self._last = {s: None for s in (*SIDES, "both")}
        self._scales = self._load_scales()

    # ── 数据输入（ROS 回调线程调用，必须很快） ─────────────────────────────

    def add_force(self, side: str, values, t: float | None = None) -> None:
        t = self._clock() if t is None else t
        row = (t,) + tuple(float(v) for v in values[:6])
        with self._lock:
            buf = self._force[side]
            buf.append(row)
            self._trim(buf, t)

    def add_motor_status(self, entries, t: float | None = None) -> None:
        """entries: 可迭代的 (motor_id, pos, speed, current, error)。"""
        t = self._clock() if t is None else t
        touched = set()
        with self._lock:
            for mid, pos, speed, current, error in entries:
                try:
                    mid = int(mid)
                except (TypeError, ValueError):
                    continue
                self._latest[mid] = (float(pos), float(speed), float(current), int(error))
                self._latest_t[mid] = t
                for side, ids in ARM_JOINT_IDS.items():
                    if mid in ids:
                        touched.add(side)
            for side in touched:
                ids = ARM_JOINT_IDS[side]
                if not all(i in self._latest for i in ids):
                    continue
                vals = [self._latest[i] for i in ids]
                row = (t, tuple(v[0] for v in vals), tuple(v[1] for v in vals),
                       tuple(v[2] for v in vals), max(abs(v[3]) for v in vals))
                buf = self._joints[side]
                buf.append(row)
                self._trim(buf, t)

    def clear_streams(self) -> None:
        with self._lock:
            for s in SIDES:
                self._force[s].clear()
                self._joints[s].clear()
            self._latest.clear()
            self._latest_t.clear()

    def _trim(self, buf, t):
        horizon = t - self.cfg["history_s"]
        while buf and buf[0][0] < horizon:
            buf.popleft()

    # ── 比例系数 ────────────────────────────────────────────────────────────

    def _scale_ok(self, value) -> bool:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        return math.isfinite(v) and self.cfg["scale_min"] <= v <= self.cfg["scale_max"]

    def _load_scales(self) -> dict:
        path = Path(self.cfg["state_file"])
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except Exception as exc:
            self._log(f"[weigh] WARNING: ignoring unreadable {path}: {exc}")
            return {}
        out = {}
        for side, entry in (data.get("sides") or {}).items():
            if side in SIDES and isinstance(entry, dict) and self._scale_ok(entry.get("scale")):
                out[side] = dict(entry, scale=float(entry["scale"]))
            else:
                self._log(f"[weigh] WARNING: ignoring invalid scale entry for {side!r} in {path}")
        return out

    def _save_scales(self) -> None:
        path = Path(self.cfg["state_file"])
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"version": 1, "sides": self._scales}
        fd, tmp = tempfile.mkstemp(prefix=path.name + ".tmp-", dir=str(path.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
                fh.write("\n")
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, path)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def scale_info(self, side: str) -> dict:
        entry = self._scales.get(side)
        if entry:
            return {"scale": entry["scale"], "scale_source": entry.get("source", "unknown"),
                    "calibrated": True, "scale_updated_at": entry.get("updated_at")}
        return {"scale": float(self.cfg["default_scale"]), "scale_source": DEFAULT_SCALE_SOURCE,
                "calibrated": False, "scale_updated_at": None}

    # ── 窗口分析 ────────────────────────────────────────────────────────────

    def analyze(self, side: str, duration: float, now: float | None = None) -> dict:
        """分析最近 duration 秒。返回均值、稳定性和问题列表（不改状态）。"""
        cfg = self.cfg
        now = self._clock() if now is None else now
        start = now - duration
        with self._lock:
            force_all = list(self._force[side])
            joints_all = list(self._joints[side])
        force = [r for r in force_all if r[0] >= start]
        joints = [r for r in joints_all if r[0] >= start]
        reasons = []
        out = {"side": side, "duration_s": duration, "samples": len(force), "reasons": reasons,
               "stable": False, "F": None, "q": None}

        # 力数据：是否有、是否新鲜、频率、断流
        if not force_all:
            reasons.append("no_force_data")
        else:
            age = now - force_all[-1][0]
            out["force_age_s"] = _r(age, 3)
            if age > cfg["max_gap_s"]:
                reasons.append("force_stream_stale")
        if force:
            times = [start] + [r[0] for r in force] + [now]
            gap = max(b - a for a, b in zip(times, times[1:]))
            out["max_gap_s"] = _r(gap, 3)
            rate = len(force) / duration
            out["rate_hz"] = _r(rate, 1)
            if gap > cfg["max_gap_s"] and "force_stream_stale" not in reasons:
                reasons.append("force_stream_gap")
            if rate < cfg["min_rate_hz"]:
                reasons.append("force_rate_low")

        # 0.5 s 分块的稳定判据
        block = cfg["block_s"]
        n_blocks = max(1, int(round(duration / block)))
        min_count = max(3, int(block * cfg["min_rate_hz"] * 0.8))
        blocks = []
        for k in range(n_blocks):
            lo, hi = start + k * block, start + (k + 1) * block
            rows = [r[1:4] for r in force if lo <= r[0] < hi or (k == n_blocks - 1 and r[0] == hi)]
            if len(rows) < min_count:
                blocks = None
                break
            m = _mean_rows(rows, 3)
            sd = max(math.sqrt(sum((r[i] - m[i]) ** 2 for r in rows) / len(rows)) for i in range(3))
            blocks.append((m, sd))
        if force:
            out["F"] = _mean_rows([r[1:7] for r in force], 6)
        if blocks:
            steps = [_norm(_sub(b[0], a[0])) for a, b in zip(blocks, blocks[1:])]
            drift = max(_norm(_sub(b[0], blocks[0][0])) for b in blocks)
            sd = max(b[1] for b in blocks)
            out["block_step_max_n"] = _r(max(steps) if steps else 0.0, 3)
            out["drift_n"] = _r(drift, 3)
            out["block_sd_max_n"] = _r(sd, 3)
            out["noise_kg"] = _r(max(drift, max(steps) if steps else 0.0) / G, 4)
            if (steps and max(steps) >= cfg["block_step_n"]) or drift >= cfg["window_drift_n"] \
                    or sd >= cfg["block_sd_n"]:
                reasons.append("force_unstable")
        elif force_all:
            reasons.append("force_window_incomplete")

        # 关节：是否有、是否新鲜、是否静止、error、电流
        if not joints_all:
            reasons.append("no_joint_data")
        else:
            jage = now - joints_all[-1][0]
            out["joint_age_s"] = _r(jage, 3)
            if jage > cfg["max_gap_s"]:
                reasons.append("joint_stream_stale")
            window = joints or [joints_all[-1]]
            ids = ARM_JOINT_IDS[side]
            q = [sum(r[1][i] for r in window) / len(window) for i in range(7)]
            span = max(max(r[1][i] for r in window) - min(r[1][i] for r in window) for i in range(7))
            speed = max(abs(v) for r in window for v in r[2])
            ratio = max(abs(r[3][i]) / RATED_CURRENT_A[ids[i]] for r in window for i in range(7))
            err = max(r[4] for r in window)
            out.update({"q": q, "joint_span_deg": _r(math.degrees(span), 3),
                        "max_joint_speed": _r(speed, 4), "max_current_ratio": _r(ratio, 3),
                        "joint_error": int(err)})
            if err:
                reasons.append("joint_error")
            if speed >= cfg["joint_speed_rad_s"] or math.degrees(span) > cfg["joint_span_deg"]:
                reasons.append("joint_moving")

        out["stable"] = not reasons
        return out

    # ── 动作 ────────────────────────────────────────────────────────────────

    def _sides_of(self, side) -> list:
        """side 参数 → 实际要处理的手列表。both = 左右两只手，结果相加。"""
        side = str(side or "right")
        sides = list(SIDES) if side == "both" else [side]
        bad = [s for s in sides if s not in SIDES or s not in self.cfg["sides"]]
        if bad:
            raise ValueError(f"side {side!r} is not enabled (enabled: {self.cfg['sides']})")
        return sides

    @staticmethod
    def _side_arg(args: dict) -> str:
        """引擎内部缺省为 right。经 MCP 的调用在 WeighPlugin.dispatch 处就要求显式填写 side：
        卡片被多个调用方共用，不能保存一只"当前手"去替调用方做选择。"""
        value = (args or {}).get("side")
        return "right" if value in (None, "") else str(value)

    def _wait_stable_multi(self, sides, duration, timeout, extra_fatal=()):
        """轮询直到所有手的窗口都稳定、遇到致命问题或超时。返回 ({side: analysis}, waited_s, timed_out)。

        只接受调用之后采到的数据：窗口起点必须晚于调用时刻。否则"刚放上物体就调用
        weigh"会直接拿到缓冲区里放物体之前那 3 秒的空载数据。
        """
        t0 = self._clock()
        while True:
            now = self._clock()
            waited = now - t0
            res = {s: self.analyze(s, duration, now) for s in sides}
            fresh = waited >= duration - 1e-6
            if fresh and all(a["stable"] for a in res.values()):
                return res, waited, False
            fatal = [r for a in res.values() for r in a["reasons"] if r in _FATAL or r in extra_fatal]
            # 刚启动订阅时数据还没到，给一个宽限期
            if fatal and not (waited < self.cfg["data_grace_s"]
                              and set(fatal) <= {"no_force_data", "no_joint_data"}):
                return res, waited, False
            if waited >= timeout:
                return res, waited, True
            self._sleep(self.cfg["poll_s"])

    def _wait_stable(self, side, duration, timeout, extra_fatal=()):
        res, waited, timed_out = self._wait_stable_multi([side], duration, timeout, extra_fatal)
        return res[side], waited, timed_out

    def _params(self, args: dict):
        side = self._side_arg(args)
        sides = self._sides_of(side)
        try:
            duration = float(args.get("duration_s") or self.cfg["default_duration_s"])
            timeout = float(args.get("timeout_s") or self.cfg["default_timeout_s"])
        except (TypeError, ValueError):
            raise ValueError("duration_s / timeout_s must be numbers")
        if not math.isfinite(duration) or not math.isfinite(timeout):
            raise ValueError("duration_s / timeout_s must be finite")
        duration = min(max(duration, 1.0), 10.0)
        timeout = min(max(timeout, duration + 1.0), self.cfg["max_timeout_s"])
        return side, sides, duration, timeout

    def _common_warnings(self, side):
        return []

    @staticmethod
    def _prefixed(side, items, sides):
        """both 时给原因/警告加上 left:/right: 前缀，单手时原样返回。"""
        return list(items) if len(sides) == 1 else [f"{side}:{x}" for x in items]

    def _fail_multi(self, code, side, sides, res, timed_out, hint):
        if len(sides) == 1:
            return self._fail(code, side, res[sides[0]], timed_out, hint)
        reasons = [r for s in sides for r in self._prefixed(s, res[s]["reasons"], sides)]
        out = self._error(code, side, hint, valid=False, stable=False, timed_out=timed_out,
                          reasons=reasons)
        out["per_side"] = {s: self._fail(code, s, res[s], timed_out, None) for s in sides}
        return out

    # ── tare ──

    def _record_tare(self, side, a, waited):
        now_wall = self._wall()
        warnings = self._common_warnings(side)
        prev = self._tare[side]
        info = {}
        if prev is not None:
            dev = max(abs(x - y) for x, y in zip(a["q"], prev["q"]))
            if math.degrees(dev) <= self.cfg["joint_dev_deg"]:
                shift = _norm(_sub(a["F"][:3], prev["F"][:3])) / G
                info["shift_from_previous_tare_kg"] = _r(shift)
                if shift > 0.05:
                    # 同一姿态下零点变了 50 g 以上：可能手上有东西，或零点漂移
                    warnings.append("differs_from_previous_tare")
        self._tare[side] = {"F": a["F"], "q": a["q"], "t": self._clock(), "wall": now_wall,
                            "noise_kg": a.get("noise_kg")}
        self._last[side] = None
        self._log(f"[weigh] tare {side}: noise={a.get('noise_kg')} kg")
        return {"state": "tared", "side": side, "experimental": True,
                "tared_at": _iso(now_wall), "noise_kg": a.get("noise_kg"),
                "joints_deg": [_r(math.degrees(x), 2) for x in a["q"]],
                "waited_s": _r(waited, 2), "warnings": warnings, **info}

    def tare(self, args: dict) -> dict:
        side, sides, duration, timeout = self._params(args)
        res, waited, timed_out = self._wait_stable_multi(sides, duration, timeout)
        if not all(res[s]["stable"] for s in sides):
            return self._fail_multi("tare_failed", side, sides, res, timed_out,
                                    "掌心空着、手臂静止时再去皮" if timed_out else None)
        per = {s: self._record_tare(s, res[s], waited) for s in sides}
        if len(sides) == 1:
            return per[side]
        self._last["both"] = None
        return {"state": "tared", "side": "both", "experimental": True,
                "noise_kg": max(p["noise_kg"] or 0 for p in per.values()),
                "waited_s": _r(waited, 2),
                "warnings": [w for s in sides for w in self._prefixed(s, per[s]["warnings"], sides)],
                "per_side": per}

    def _tare_problem(self, side):
        tare = self._tare[side]
        if tare is None:
            return "not_tared", None
        age = self._clock() - tare["t"]
        if age > self.cfg["tare_ttl_s"]:
            return "tare_expired", age
        return None, age

    def _measure(self, side, a, tare, scale):
        cfg = self.cfg
        dF = _sub(a["F"][:3], tare["F"][:3])
        dT = _sub(a["F"][3:6], tare["F"][3:6])
        n = _norm(dF)
        raw = n / G
        mass = raw / scale
        lever = None
        if n > 0.5:
            lever = _norm(_cross(dF, dT)) / (n * n)
        dev = math.degrees(max(abs(x - y) for x, y in zip(a["q"], tare["q"])))
        reasons = []
        if dev > cfg["joint_dev_deg"]:
            reasons.append("pose_changed")
        if a["max_current_ratio"] > cfg["current_ratio_max"]:
            reasons.append("current_high")
        if cfg["load_limit_kg"] and mass > cfg["load_limit_kg"]:
            reasons.append("over_load_limit")
        return {"raw_kg": raw, "mass_kg": mass, "lever_arm_m": lever,
                "max_joint_dev_deg": dev, "reasons": reasons,
                "force_direction": [x / n for x in dF] if n > 0.5 else None}

    # ── weigh ──

    def _weigh_result(self, side, a, waited, duration, tare_age):
        sc = self.scale_info(side)
        m = self._measure(side, a, self._tare[side], sc["scale"])
        warnings = self._common_warnings(side)
        if m["mass_kg"] < self.cfg["resolution_kg"]:
            warnings.append("below_resolution")
        if not sc["calibrated"]:
            warnings.append("uncalibrated_default_scale")
        result = {
            "state": "weighed", "side": side, "experimental": True,
            "valid": not m["reasons"], "stable": True, "reasons": m["reasons"],
            "mass_kg": _r(m["mass_kg"]), "mass_g": _r(m["mass_kg"] * 1000, 1),
            "raw_kg": _r(m["raw_kg"]), **{k: (_r(v) if isinstance(v, float) else v) for k, v in sc.items()},
            "lever_arm_m": _r(m["lever_arm_m"], 3),
            "max_joint_dev_deg": _r(m["max_joint_dev_deg"], 3),
            "max_current_ratio": a["max_current_ratio"],
            "noise_kg": a.get("noise_kg"),
            "tare_age_s": _r(tare_age, 1),
            "window_s": duration, "samples": a["samples"], "waited_s": _r(waited, 2),
            "measured_at": _iso(self._wall()),
            "warnings": warnings,
        }
        return result, m

    def weigh(self, args: dict, *, _for_calibration: bool = False) -> dict:
        side, sides, duration, timeout = self._params(args)
        for s in sides:
            problem, _ = self._tare_problem(s)
            if problem:
                return self._error(problem, side, f"{s} 手先在空载、静止的托举姿态下调用 tare")
        res, waited, timed_out = self._wait_stable_multi(sides, duration, timeout)
        ages = {}
        for s in sides:
            problem, ages[s] = self._tare_problem(s)
            if problem:
                return self._error(problem, side, f"等待期间 {s} 手的去皮已过期，请重新 tare")
        if not all(res[s]["stable"] for s in sides):
            out = self._fail_multi("not_stable", side, sides, res, timed_out, "物体或手臂还在晃动，或数据异常")
            prov = []
            for s in sides:
                a = res[s]
                if a.get("F") is not None and a.get("q") is not None:
                    prov.append(self._measure(s, a, self._tare[s], self.scale_info(s)["scale"])["mass_kg"])
            if len(prov) == len(sides):
                out["provisional_mass_kg"] = _r(sum(prov))
            return out
        per, raw = {}, {}
        for s in sides:
            per[s], raw[s] = self._weigh_result(s, res[s], waited, duration, ages[s])
        if len(sides) == 1:
            result = per[side]
            if _for_calibration:
                result["_raw"] = raw[side]
            else:
                self._last[side] = dict(result)
            return result
        total = sum(raw[s]["mass_kg"] for s in sides)
        reasons = [r for s in sides for r in self._prefixed(s, per[s]["reasons"], sides)]
        if self.cfg["load_limit_kg"] and total > self.cfg["load_limit_kg"] * len(sides):
            reasons.append("over_load_limit")
        warnings = [w for s in sides for w in self._prefixed(s, per[s]["warnings"], sides)]
        if total < self.cfg["resolution_kg"]:
            warnings.append("below_resolution")
        result = {"state": "weighed", "side": "both", "experimental": True,
                  "valid": not reasons, "stable": True, "reasons": reasons,
                  "mass_kg": _r(total), "mass_g": _r(total * 1000, 1),
                  "max_joint_dev_deg": max(per[s]["max_joint_dev_deg"] for s in sides),
                  "max_current_ratio": max(per[s]["max_current_ratio"] for s in sides),
                  "tare_age_s": max(per[s]["tare_age_s"] for s in sides),
                  "window_s": duration, "waited_s": _r(waited, 2),
                  "measured_at": _iso(self._wall()), "warnings": warnings,
                  "per_side": per}
        self._last["both"] = dict(result)
        return result

    # ── calibrate ──

    def _store_scale(self, side, entry):
        prev_entry = self._scales.get(side)
        self._scales[side] = entry
        try:
            self._save_scales()
        except Exception as exc:
            if prev_entry is None:
                self._scales.pop(side, None)
            else:
                self._scales[side] = prev_entry
            return f"写入 {self.cfg['state_file']} 失败：{exc}"
        self._last[side] = None
        return None

    def calibrate(self, args: dict) -> dict:
        side = self._side_arg(args)
        try:
            sides = self._sides_of(side)
        except ValueError as exc:
            return self._error("bad_side", side, str(exc))
        has_mass = args.get("known_mass_kg") not in (None, "")
        has_scale = args.get("scale") not in (None, "")
        if has_mass == has_scale:
            return self._error("bad_arguments", side,
                               "known_mass_kg（实测校准）和 scale（手动设定）必须且只能给一个")
        if has_mass and len(sides) > 1:
            return self._error("bad_arguments", side,
                               "实测校准请每只手分别做（side=left / right）；side=both 只能配合 scale 手动设定")
        lo, hi = self.cfg["scale_min"], self.cfg["scale_max"]
        if has_scale:
            try:
                new = float(args["scale"])
            except (TypeError, ValueError):
                return self._error("bad_arguments", side, "scale 必须是数字")
            if not self._scale_ok(new):
                return self._error("scale_out_of_range", side, f"scale 必须在 {lo}–{hi} 之间，已保留原值")
            entries = {s: {"scale": new, "source": "manual", "updated_at": _iso(self._wall())} for s in sides}
        else:
            try:
                known = float(args["known_mass_kg"])
            except (TypeError, ValueError):
                return self._error("bad_arguments", side, "known_mass_kg 必须是数字")
            upper = self.cfg["load_limit_kg"] or self.cfg["max_calibration_kg"]
            if not (math.isfinite(known) and self.cfg["min_calibration_kg"] <= known <= upper):
                return self._error("bad_arguments", side,
                                   f"known_mass_kg 必须在 {self.cfg['min_calibration_kg']}–{upper} kg 之间")
            r = self.weigh(args, _for_calibration=True)
            if r.get("state") != "weighed":
                return r
            m = r.pop("_raw")
            blocking = [x for x in m["reasons"] if x != "over_load_limit"]
            if blocking:
                return self._error("calibration_rejected", side,
                                   "测量无效：" + ",".join(blocking), reasons=blocking)
            new = m["raw_kg"] / known
            if not self._scale_ok(new):
                return self._error("scale_out_of_range", side,
                                   f"算出 scale={new:.4f}，不在 {lo}–{hi} 之间。检查物体是否放稳、"
                                   "重量是否填对、去皮时掌心是否空着。已保留原值",
                                   computed_scale=_r(new))
            entries = {side: {"scale": new, "source": "measured", "updated_at": _iso(self._wall()),
                              "reference_mass_kg": known, "raw_kg": _r(m["raw_kg"]),
                              "lever_arm_m": _r(m["lever_arm_m"], 3)}}
        per = {}
        for s, entry in entries.items():
            old = self.scale_info(s)
            err = self._store_scale(s, entry)
            if err:
                return self._error("save_failed", side, err)
            self._log(f"[weigh] calibrate {s}: {old['scale']:.4f} -> {entry['scale']:.4f} ({entry['source']})")
            per[s] = {"state": "calibrated", "side": s, "experimental": True,
                      "old_scale": _r(old["scale"]), "old_scale_source": old["scale_source"],
                      "scale": _r(entry["scale"]), "scale_source": entry["source"],
                      "reference_mass_kg": entry.get("reference_mass_kg"),
                      "state_file": self.cfg["state_file"],
                      "warnings": self._common_warnings(s)}
        if len(sides) == 1:
            return per[side]
        self._last["both"] = None
        return {"state": "calibrated", "side": "both", "experimental": True,
                "scale": _r(new), "scale_source": "manual", "per_side": per}

    # ── pose / retract：经 arm 卡移动手臂、经 hand 卡设手型 ────────────────────

    def _arm_deg(self, side: str, now: float):
        """当前 7 关节角（度）。数据缺失或过期返回 None。"""
        ids = ARM_JOINT_IDS[side]
        with self._lock:
            if not all(i in self._latest for i in ids):
                return None
            if any(now - self._latest_t[i] > self.cfg["max_gap_s"] for i in ids):
                return None
            return [math.degrees(self._latest[i][0]) for i in ids]

    def _arm_state(self, side: str):
        ids = ARM_JOINT_IDS[side]
        with self._lock:
            vals = [self._latest.get(i) for i in ids]
        if any(v is None for v in vals):
            return None
        return {"deg": [math.degrees(v[0]) for v in vals], "speed": max(abs(v[1]) for v in vals),
                "error": max(abs(v[3]) for v in vals)}

    @staticmethod
    def _within(a, b, tol):
        return max(abs(x - y) for x, y in zip(a, b)) <= tol

    @staticmethod
    def _mirror(deg):
        """右臂角度 → 左臂镜像：肩 roll、肩 yaw、腕 yaw、腕 roll（索引 1/2/4/6）取反（同 arm 卡说明）。"""
        return [-v if i in (1, 2, 4, 6) else v for i, v in enumerate(deg)]

    def _pose_deg(self, side):
        pose = [float(x) for x in self.cfg["pose_deg"]]
        if side == "left":
            left = self.cfg.get("pose_deg_left")
            return [float(x) for x in left] if left else self._mirror(pose)
        return pose

    def _rest_deg(self, side):
        rest = [float(x) for x in self.cfg["rest_deg"]]
        return self._mirror(rest) if side == "left" else rest

    def cancel(self) -> None:
        self._cancel.set()

    def clear_cancel(self) -> None:
        self._cancel.clear()

    def _check_cancel(self) -> None:
        if self._cancel.is_set():
            raise Cancelled()

    def _sleep(self, dt: float) -> None:
        """所有等待都经这里：取消后立即退出，不再进入下一步。"""
        self._check_cancel()
        self._raw_sleep(dt)
        self._check_cancel()

    def _servo_state(self):
        """servo 卡（motus.control/1 指令流，同样驱动双臂和双手）的状态：
        None=未加载 / 'idle' / 'paused' / 'running'，查询失败返回 'unknown'。"""
        if self._servo_call is None:
            return None
        try:
            r = self._servo_call({"action": "info"})
        except Exception as exc:
            self._log(f"[weigh] servo info failed: {exc}")
            return "unknown"
        if r is None:
            return None
        return r.get("state", "unknown") if isinstance(r, dict) else "unknown"

    def _take_over_from_servo(self):
        """显式动作优先于指令流：servo 正在推流时先暂停它（保持当前姿态，不自动恢复）。
        返回 (problem, paused)。problem 非 None 时不得下发任何手臂/手指指令。"""
        self._check_cancel()
        state = self._servo_state()
        if state in (None, "idle", "paused"):
            return None, False
        if state != "running":
            return f"servo state {state!r}", False
        try:
            r = self._servo_call({"action": "pause"})
        except Exception as exc:
            return f"servo pause failed: {exc}", False
        if not isinstance(r, dict) or r.get("state") != "paused":
            return f"servo pause failed: {_brief(r)}", False
        self._log("[weigh] servo stream paused before moving arms/fingers; resume it explicitly when done")
        return None, True

    def _servo_guard(self):
        """每条手臂/手指指令之前调用：有人在动作进行中恢复了 servo 就停下，不和指令流抢关节。"""
        state = self._servo_state()
        if state in (None, "idle", "paused"):
            return None
        return f"servo {state}"

    def _call_arm(self, args: dict):
        self._check_cancel()
        if self._arm_call is None:
            return {"state": "error", "code": "arm_card_unavailable",
                    "error": "arm 卡未启用，pose/retract 不可用"}
        try:
            r = self._arm_call(dict(args))
        except Exception as exc:
            return {"state": "error", "code": "arm_call_failed", "error": str(exc)}
        if r is None:
            return {"state": "error", "code": "arm_card_unavailable", "error": "arm 卡未启用"}
        return r

    def _freeze(self):
        """超时未到位：把两臂目标设成当前位置，停止继续用力。只在没有电机故障时调用。"""
        now = self._clock()
        left, right = self._arm_deg("left", now), self._arm_deg("right", now)
        if left is None or right is None:
            return {"sent": False, "reason": "no_fresh_joint_data"}
        problem = self._servo_guard()
        if problem:
            # 有人在动作进行中恢复了 servo：关节归它管，不再补发冻结指令去抢。
            self._log(f"[weigh] freeze skipped: {problem}")
            return {"sent": False, "reason": "servo_active"}
        self._log(f"[weigh] freeze: holding right at {_fmt_deg(right)}, left at {_fmt_deg(left)}")
        r = self._call_arm({"action": "move_pos", "left_positions": [round(x, 2) for x in left],
                            "right_positions": [round(x, 2) for x in right],
                            "speed": self.cfg["pose_speed"]})
        return {"sent": True, "arm_result": r.get("state"), "arm_error": r.get("error")}

    def _move_stage(self, targets: dict, label):
        """targets: {side: 目标角度（度）}。未列出的手臂保持当前角度。阻塞到到位、故障或超时。"""
        now = self._clock()
        for s in SIDES:
            cur = self._arm_deg(s, now)
            if s in targets:
                self._log(f"[weigh] stage {label}: {s} {_fmt_deg(cur)} -> {_fmt_deg(targets[s])}")
            else:
                self._log(f"[weigh] stage {label}: {s} held at {_fmt_deg(cur)}")
        problem = self._servo_guard()
        if problem:
            r = {"ok": False, "stage": label, "code": "servo_active",
                 "error": f"servo 卡在推流（{problem}），不和它抢关节；已停止，没有下发本段"}
        else:
            r = self._move_stage_inner(targets, label)
        if r["ok"]:
            self._log(f"[weigh] stage {label}: arrived in {r['seconds']}s at "
                      + " ".join(f"{s}={_fmt_deg(r['joints_deg'][s])}" for s in targets))
        else:
            self._log(f"[weigh] stage {label}: FAILED {r['code']} {r.get('error') or ''} "
                      f"joints={r.get('joints_deg')} freeze={r.get('freeze')} "
                      f"arm={_brief(r.get('arm_result'))}")
        return r

    def _move_stage_inner(self, targets, label):
        cfg = self.cfg
        now = self._clock()
        cur = {s: self._arm_deg(s, now) for s in SIDES}
        if any(v is None for v in cur.values()):
            return {"ok": False, "code": "joint_data_unavailable", "stage": label,
                    "error": "没有新鲜的 /arm/status，不能下发运动"}
        cmd = {s: targets.get(s, cur[s]) for s in SIDES}
        args = {"action": "move_pos", "left_positions": [round(x, 2) for x in cmd["left"]],
                "right_positions": [round(x, 2) for x in cmd["right"]], "speed": cfg["pose_speed"]}
        r = self._call_arm(args)
        if not isinstance(r, dict) or r.get("state") == "error" or r.get("error"):
            return {"ok": False, "code": "arm_rejected", "stage": label,
                    "error": (r or {}).get("error"), "arm_result": r}
        delta = max(abs(a - b) for s in targets for a, b in zip(cur[s], targets[s]))
        timeout = math.radians(delta) / cfg["pose_speed"] + cfg["pose_extra_timeout_s"]
        t0 = self._clock()
        while True:
            st = {s: self._arm_state(s) for s in SIDES}
            if all(v is not None for v in st.values()):
                degs = {s: [_r(x, 2) for x in st[s]["deg"]] for s in SIDES}
                if any(st[s]["error"] for s in SIDES):
                    return {"ok": False, "code": "joint_error", "stage": label,
                            "error": "电机报错，已停止，不重试", "joints_deg": degs}
                if all(self._within(st[s]["deg"], targets[s], cfg["arrive_tol_deg"])
                       and st[s]["speed"] < cfg["arrive_speed_rad_s"] for s in targets):
                    return {"ok": True, "stage": label, "seconds": _r(self._clock() - t0, 2),
                            "joints_deg": degs}
            if self._clock() - t0 > timeout:
                frozen = self._freeze()
                return {"ok": False, "code": "arrive_timeout", "stage": label,
                        "error": f"{timeout:.0f} s 内未到位，已把目标设为当前位置并停止，不重试",
                        "freeze": frozen,
                        "joints_deg": {s: [_r(x, 2) for x in st[s]["deg"]] for s in SIDES if st[s]}}
            self._sleep(0.1)

    def _set_hand(self, shape: str, side: str = "right"):
        """把一只或两只手（side=left/right/both）设成 hand_shapes[shape]。
        返回 None 表示成功，否则返回警告字符串（不阻断称重）。"""
        fingers = self.cfg["hand_shapes"].get(shape)
        if fingers is None:
            return f"unknown_hand_shape:{shape}"
        if self._hand_call is None:
            self._log(f"[weigh] hand {shape}: hand card unavailable, fingers unchanged")
            return "hand_card_unavailable"
        try:
            if not self._hand_started:
                self._check_cancel()
                r = self._hand_call({"action": "start"})
                if not isinstance(r, dict) or r.get("state") == "error" or r.get("error"):
                    self._log(f"[weigh] hand start failed: {_brief(r)}")
                    return "hand_start_failed"
                self._hand_started = True
                self._sleep(self.cfg["arm_start_settle_s"])
            args = {"action": "set_fingers_raw", "side": side}
            args.update({k: max(0, min(100, int(v))) for k, v in fingers.items()})
            self._check_cancel()
            if self._servo_guard():
                self._log(f"[weigh] hand {shape}: servo is streaming, fingers unchanged")
                return "servo_active"
            r = self._hand_call(args)
        except Exception as exc:
            self._log(f"[weigh] hand {shape} raised {exc}")
            return "hand_call_failed"
        if not isinstance(r, dict) or r.get("state") == "error" or r.get("error"):
            self._log(f"[weigh] hand {shape} ({side}) rejected: {_brief(r)}")
            return "hand_rejected"
        self._log(f"[weigh] hand {shape} ({side}): {fingers}")
        self._sleep(self.cfg["hand_settle_s"])
        return None

    def _ensure_arm_started(self):
        if self._arm_started:
            return None
        r = self._call_arm({"action": "start"})
        if not isinstance(r, dict) or r.get("state") == "error" or r.get("error"):
            return r
        self._arm_started = True
        self._sleep(self.cfg["arm_start_settle_s"])
        return None

    @staticmethod
    def _merge_stages(order, per_side):
        """per_side: {side: [(label, target), ...]} → 按 order 合并成 [(label, {side: target})]。"""
        merged = []
        for label in order:
            targets = {s: t for s, stages in per_side.items() for lb, t in stages if lb == label}
            if targets:
                merged.append((label, targets))
        return merged

    def pose(self, args: dict) -> dict:
        side = self._side_arg(args)
        try:
            sides = self._sides_of(side)
        except ValueError as exc:
            return self._error("bad_side", side, str(exc))
        cfg = self.cfg
        now = self._clock()
        if any(self._arm_deg(s, now) is None for s in SIDES):
            return self._error("joint_data_unavailable", side, "没有新鲜的 /arm/status")
        hand = str(args.get("hand") or cfg["default_hand"])
        if hand != "keep" and hand not in cfg["hand_shapes"]:
            return self._error("bad_hand", side, f"未知手型 {hand!r}（可选 {sorted(cfg['hand_shapes'])} 或 keep）")
        tol = cfg["arrive_tol_deg"]
        per_side, origins, poses = {}, {}, {}
        for s in sides:
            pose, rest = self._pose_deg(s), self._rest_deg(s)
            poses[s] = pose
            cur = self._arm_deg(s, now)
            if self._arm_state(s)["error"]:
                return self._error("joint_error", side, f"{s} 臂电机报错，不运动")
            if self._within(cur, pose, tol):
                per_side[s] = []                         # 已在托举姿态
                continue
            mid = list(pose)
            mid[_WRIST_YAW] = cur[_WRIST_YAW] if self._within(cur, rest, cfg["rest_zone_deg"]) else rest[_WRIST_YAW]
            if self._within(cur, mid, tol):
                per_side[s] = [("palm_up", pose)]
            elif self._within(cur, rest, cfg["rest_zone_deg"]):
                per_side[s] = [("raise_forearm", mid), ("palm_up", pose)]
                origins[s] = [_r(x, 2) for x in cur]
            else:
                return self._error("unknown_start_pose", side,
                                   f"{s} 臂不在垂下的姿态附近，也不在托举姿态，出于安全不自动摆姿态。"
                                   "请先用 arm 卡把手臂放下",
                                   joints_deg=[_r(x, 2) for x in cur], rest_zone_deg=cfg["rest_zone_deg"])
        stages = self._merge_stages(("raise_forearm", "palm_up"), per_side)
        steps = []
        servo_paused = False
        if stages or hand != "keep":
            problem, servo_paused = self._take_over_from_servo()
            if problem:
                return self._error("servo_active", side,
                                   f"servo 卡正在驱动手臂/手指，且无法暂停（{problem}）；没有下发任何指令")
        if stages:
            bad = self._ensure_arm_started()
            if bad is not None:
                return self._error("arm_start_failed", side, (bad or {}).get("error"), arm_result=bad)
            self._origin.update(origins)
            for label, targets in stages:
                r = self._move_stage(targets, label)
                steps.append(r)
                if not r["ok"]:
                    for s in sides:
                        self._tare[s] = None
                    return self._error(r["code"], side, r.get("error"), steps=steps)
        for s in sides:
            self._tare[s] = None                         # 姿态或手型变了，旧零点作废
            self._last[s] = None
        if hand != "keep":
            # 手型没摆成（例如四指没立起来）就不算 pose 成功：不去皮、不切换当前手，
            # 免得上层以为可以往掌心放东西。
            problem = self._set_hand(hand, side)
            if problem:
                return self._error("hand_failed", side,
                                   f"手臂已到托举姿态，但手型 {hand} 没有设置成功（{problem}）。"
                                   f"不要往掌心放东西；修好 hand 卡后重新 pose，或用 retract side={side} 收回",
                                   hand_problem=problem, arm_posed=True, steps=steps)
        out = {"state": "posed", "side": side, "experimental": True, "hand": hand,
               **({"servo_paused": True} if servo_paused else {}),
               "already_in_pose": not stages, "pose_deg": poses if len(sides) > 1 else poses[side],
               "steps": steps}
        warnings = self._common_warnings(side)
        # pose 总是去皮：这是"掌心是空的"这一前提唯一的检查点，不提供跳过的开关。
        # 去皮失败就不算 pose 成功，免得之后在有东西的掌心上补一次去皮。
        self._sleep(cfg["pose_settle_s"])
        t = self.tare({"side": side, "duration_s": args.get("duration_s"),
                       "timeout_s": args.get("timeout_s")})
        out["tare"] = t
        if t.get("state") != "tared":
            return self._error("tare_failed", side,
                               "手臂和手指已到托举姿态，但去皮没有成功（掌心须空着、手臂静止）。"
                               "不要往掌心放东西；确认掌心为空后重新 pose，或 retract",
                               arm_posed=True, hand=hand, tare=t, steps=steps)
        if warnings:
            out["warnings"] = warnings
        return out

    def retract(self, args: dict) -> dict:
        side = self._side_arg(args)
        try:
            sides = self._sides_of(side)
        except ValueError as exc:
            return self._error("bad_side", side, str(exc))
        cfg = self.cfg
        now = self._clock()
        if any(self._arm_deg(s, now) is None for s in SIDES):
            return self._error("joint_data_unavailable", side, "没有新鲜的 /arm/status")
        tol = cfg["arrive_tol_deg"]
        per_side = {}
        for s in sides:
            pose = self._pose_deg(s)
            rest = list(self._origin.get(s) or self._rest_deg(s))
            cur = self._arm_deg(s, now)
            if self._arm_state(s)["error"]:
                return self._error("joint_error", side, f"{s} 臂电机报错，不运动")
            if self._within(cur, rest, tol):
                per_side[s] = []
                continue
            mid = list(pose)
            mid[_WRIST_YAW] = rest[_WRIST_YAW]
            if self._within(cur, pose, tol):
                per_side[s] = [("palm_inward", mid), ("lower_arm", rest)]
            elif self._within(cur, mid, tol):
                per_side[s] = [("lower_arm", rest)]
            else:
                return self._error("unknown_start_pose", side,
                                   f"{s} 臂不在托举姿态，出于安全不自动收回。请用 arm 卡操作",
                                   joints_deg=[_r(x, 2) for x in cur])
            # 只要这只手臂要动，就先确认掌上是空的（托举姿态和中间姿态都一样）：
            # 张开手指、转腕或放下手臂都可能让物体掉下去。去皮有效时看读数，否则要求确认。
            # 零点只在去皮时的姿态下有意义：姿态变了（例如停在中间姿态）就不能靠读数判断。
            tare = self._tare[s]
            problem, _ = self._tare_problem(s)
            if tare is not None and problem is None:
                live = self.analyze(s, cfg["block_s"], now)
                if live.get("F") is None or live.get("q") is None:
                    return self._error("force_data_unavailable", side,
                                       f"{s} 手没有新鲜的力数据，无法判断掌上是否有物体")
                dev = math.degrees(max(abs(x - y) for x, y in zip(live["q"], tare["q"])))
                if dev > cfg["joint_dev_deg"]:
                    problem = "pose_changed_since_tare"
            if tare is not None and problem is None:
                load = _norm(_sub(live["F"][:3], tare["F"][:3])) / G / self.scale_info(s)["scale"]
                if load > cfg["retract_max_load_kg"]:
                    return self._error("object_on_hand", side,
                                       f"{s} 手掌上还有约 {load * 1000:.0f} g，先取走物体再收回",
                                       load_kg=_r(load))
            elif args.get("confirm_empty") not in (True, "true", 1, "1"):
                return self._error("load_unknown", side,
                                   f"{s} 手没有有效的去皮（{problem or 'not_tared'}），无法判断掌上是否有物体。"
                                   "确认掌心已空后传 confirm_empty=true")
        stages = self._merge_stages(("palm_inward", "lower_arm"), per_side)
        if not stages:
            return {"state": "retracted", "side": side, "already_retracted": True, "steps": []}
        problem, servo_paused = self._take_over_from_servo()
        if problem:
            return self._error("servo_active", side,
                               f"servo 卡正在驱动手臂/手指，且无法暂停（{problem}）；没有下发任何指令")
        # 先张开手指（四指立起时转腕可能卡到东西），再动手臂。张不开就不动手臂：
        # 手指还立着时转腕会碰到或甩掉掌上的东西。
        hand_problem = self._set_hand("open", side)
        if hand_problem:
            return self._error("hand_open_failed", side,
                               f"张开手指失败（{hand_problem}），手臂没有动。"
                               "请先用 hand 卡张开手指，再调用 retract",
                               hand_problem=hand_problem)
        bad = self._ensure_arm_started()
        if bad is not None:
            return self._error("arm_start_failed", side, (bad or {}).get("error"), arm_result=bad)
        steps = []
        for label, targets in stages:
            r = self._move_stage(targets, label)
            steps.append(r)
            if not r["ok"]:
                return self._error(r["code"], side, r.get("error"), steps=steps)
        for s in sides:
            self._tare[s] = None
            self._last[s] = None
            self._origin.pop(s, None)
        return {"state": "retracted", "side": side, "experimental": True, "steps": steps,
                **({"servo_paused": True} if servo_paused else {})}

    # ── read ──

    def _read_one(self, side, now):
        live = self.analyze(side, self.cfg["block_s"], now)
        tare = self._tare[side]
        problem, age = self._tare_problem(side)
        sc = self.scale_info(side)
        out = {"state": "ok", "side": side, "experimental": True,
               "tared": tare is not None, "tare_valid": tare is not None and problem is None,
               "tare_age_s": _r(age, 1), "tared_at": _iso(tare["wall"]) if tare else None,
               **{k: (_r(v) if isinstance(v, float) else v) for k, v in sc.items()},
               "last_result": self._last[side],
               "live": {"samples": live["samples"], "rate_hz": live.get("rate_hz"),
                        "force_age_s": live.get("force_age_s"),
                        "max_current_ratio": live.get("max_current_ratio"),
                        "joint_error": live.get("joint_error"),
                        "issues": live["reasons"]},
               "warnings": self._common_warnings(side)}
        if tare is not None and live.get("F") is not None and live.get("q") is not None:
            m = self._measure(side, live, tare, sc["scale"])
            out["provisional"] = {"mass_kg": _r(m["mass_kg"]), "raw_kg": _r(m["raw_kg"]),
                                  "max_joint_dev_deg": _r(m["max_joint_dev_deg"], 3),
                                  "issues": m["reasons"],
                                  "note": "最近 0.5 s 的实时值，未经稳定判据，只作参考"}
        return out

    def read(self, args: dict) -> dict:
        side = self._side_arg(args)
        try:
            sides = self._sides_of(side)
        except ValueError as exc:
            return self._error("bad_side", side, str(exc))
        now = self._clock()
        per = {s: self._read_one(s, now) for s in sides}
        if len(sides) == 1:
            return per[side]
        out = {"state": "ok", "side": "both", "experimental": True,
               "tared": all(p["tared"] for p in per.values()),
               "tare_valid": all(p["tare_valid"] for p in per.values()),
               "last_result": self._last["both"], "per_side": per,
               "warnings": self._common_warnings("both")}
        if all("provisional" in p for p in per.values()):
            out["provisional"] = {"mass_kg": _r(sum(p["provisional"]["mass_kg"] for p in per.values())),
                                  "note": "两只手最近 0.5 s 实时值之和，未经稳定判据，只作参考"}
        return out

    # ── 结果格式 ────────────────────────────────────────────────────────────

    def _error(self, code, side, message=None, **extra):
        out = {"state": "error", "code": code, "side": side, "experimental": True,
               "error": message or code}
        out.update(extra)
        return out

    def _fail(self, code, side, a, timed_out, hint):
        out = self._error(code, side, hint, valid=False, stable=False, timed_out=timed_out,
                          reasons=a["reasons"])
        for k in ("samples", "rate_hz", "max_gap_s", "noise_kg", "drift_n", "block_step_max_n",
                  "block_sd_max_n", "joint_span_deg", "max_joint_speed", "max_current_ratio",
                  "joint_error"):
            if k in a:
                out[k] = a[k]
        return out


# ══════════════════════════════════════════════════════════════════════════════
# WeighPlugin — MCP 卡片（name=weigh）
# ══════════════════════════════════════════════════════════════════════════════

_RELIABLE_DEPTH = 10


# 会等待物理运动或稳定窗口的动作（> 3 s）走 ACP：先返回 action_id，做完再回调 Agent Core。
# 在 SYNC_GRACE_S 内就结束的调用（参数错误、未去皮、手动设定比例系数……）仍同步返回，
# 不带 action_id，Agent Core 也就不登记 pending。
ASYNC_ACTIONS = ("pose", "retract", "tare", "weigh", "calibrate")
RUNNING_STATE = {"pose": "posing", "retract": "retracting", "tare": "taring",
                 "weigh": "weighing", "calibrate": "calibrating"}
# 最坏情况：两段运动各约 13 s + 起臂/手指约 3 s + 去皮最多 45 s ≈ 75 s，留余量。
COMPLETION_TIMEOUT_S = 120
SYNC_GRACE_S = 0.5


def _default_acp_notify(action_id: str, status: str, result: dict, tool: str = "") -> None:
    """用 bundle 里通用的 ACP 回调（device._acp_notify）。延迟导入，保持本模块不依赖 ROS。"""
    from device import _acp_notify
    _acp_notify(action_id, status, result, tool)


class WeighPlugin:
    """腕部六维力称重卡。自己不创建任何发布者；pose/retract 经 arm 卡下发运动。"""

    PREFIX = "weigh"

    def __init__(self, plugin_config: dict, namespace: str, ros2, arm_call=None, hand_call=None,
                 acp_notify=None, servo_call=None):
        """arm_call(args) → dict：由 bundle 提供，等同于对 arm 卡调用 tools/call。
        acp_notify(action_id, status, result, tool)：异步动作完成时回调 Agent Core。"""
        self._acp_notify = acp_notify or _default_acp_notify
        self._sync_grace_s = SYNC_GRACE_S
        self._ns = namespace
        self._ros2 = ros2
        self._engine = WeighEngine(plugin_config, arm_call=arm_call, hand_call=hand_call,
                                   servo_call=servo_call)
        self._sides = self._engine.cfg["sides"]
        self._node = None
        self._executor = None
        self._thread = None
        self._stop_event = None
        self._start_lock = threading.Lock()
        self._busy = threading.Lock()
        self._worker = None
        self._stopping = False

    # ── 卡片定义 ────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        cfg = self._engine.cfg
        window = {"type": "number", "minimum": 1, "maximum": 10, "default": cfg["default_duration_s"],
                  "description": "稳定窗口长度（秒），默认 3"}
        timeout = {"type": "number", "minimum": 2, "maximum": cfg["max_timeout_s"],
                   "default": cfg["default_timeout_s"],
                   "description": "最长等待读数稳定的时间（秒），默认 15"}
        return {
            "name": "weigh",
            "type": "actuator",
            "description": (
                "天轶2.0 掌心托举称重（实验性）。side 选 left / right / both，both 时两只手同时托、"
                "分别称重后相加（例如一只手放一件，或一个大件两手一起托）。"
                "流程：pose 把手臂摆到托举姿态（掌心朝上、四指立起围住掌心）并自动去皮——调用前手臂须垂下、"
                "掌心空着、周围无人无物；然后请人把物体放到掌心，调用 weigh 得到重量；称完先取走物体，"
                "再 retract（先张开手指再收回手臂）。只有 pose/retract 会移动手臂和手指，其余动作只读。"
                "每次调用都必须填写 side，并与 pose 时一致。手臂或手型变了必须重新 tare。"
                f"适用约 {cfg['resolution_kg']} kg 以上的静止物体，"
                + (f"上限 {cfg['load_limit_kg']} kg；" if cfg["load_limit_kg"] else
                   f"上限由腕部电流决定（超过额定 {int(cfg['current_ratio_max'] * 100)}% 判无效，估计掌心约 2 kg，未实测）；")
                + "结果带 valid 标记，valid=false 时不要使用 mass_kg。不能替代电子秤计量。"
                "pose/retract/tare/weigh/calibrate 需要较长时间时先返回 action_id，"
                "完成后结果（含 mass_kg、valid 等）随完成通知送达，也可以再用 read 查看 last_result。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["pose", "tare", "weigh", "calibrate", "read", "retract"],
                               "description": "pose=摆托举姿态并去皮 tare=去皮 weigh=称重 "
                                              "calibrate=设定比例系数 read=查看状态 retract=收回手臂"},
                    "side": {"type": "string",
                             "enum": list(self._sides) + (["both"] if len(self._sides) == 2 else []),
                             "description": "用哪只手：left / right / both（两手结果相加）。必填，"
                                            "pose 之后的各个动作要用同一个值"},
                    "duration_s": window,
                    "timeout_s": timeout,
                    "known_mass_kg": {"type": "number", "minimum": cfg["min_calibration_kg"],
                                      "maximum": cfg["load_limit_kg"] or cfg["max_calibration_kg"],
                                      "description": "实测校准：手上物体的已知重量（kg）。与 scale 二选一"},
                    "scale": {"type": "number", "minimum": cfg["scale_min"], "maximum": cfg["scale_max"],
                              "description": "手动设定比例系数（原始读数 / 真实重量）。与 known_mass_kg 二选一"},
                    "hand": {"type": "string", "enum": ["cup", "open", "keep"], "default": "cup",
                             "description": "pose 的手型：cup=四指立起围住掌心（默认，物体放得稳）"
                                            " open=手指摊开 keep=不动手指"},
                    "confirm_empty": {"type": "boolean", "default": False,
                                      "description": "retract：没有有效去皮、无法检测掌上物体时，确认掌心已空"},
                },
                "required": ["action", "side"],
                "x-action-params": {
                    "pose": {"params": ["side", "hand", "duration_s", "timeout_s"],
                             "description": "移动手臂到托举姿态（先抬前臂、再转腕让掌心朝上，约 12 秒；both 两臂同时动），"
                                            "再把四指立起围住掌心（hand=cup，默认），然后自动去皮。"
                                            "只能从手臂垂下或已在托举姿态时调用；"
                                            "调用前确认掌心空着、手臂周围无人无物"},
                    "tare": {"params": ["side", "duration_s", "timeout_s"],
                             "description": "去皮：手臂已停在托举姿态且掌心空着时调用，记录零点和关节角"},
                    "weigh": {"params": ["side", "duration_s", "timeout_s"],
                              "description": "称重：放上物体后调用，等读数稳定后返回重量（kg）；需先 tare。"
                                             "both 返回两手之和，per_side 里是各手的读数"},
                    "calibrate": {"params": ["side", "known_mass_kg", "scale", "duration_s", "timeout_s"],
                                  "description": "设定比例系数（每只手各自一个）：传 known_mass_kg 放已知重量物体实测校准"
                                                 "（需先 tare，只能 left 或 right），或传 scale 直接写入（可 both）；结果持久保存"},
                    "read": {"params": ["side"],
                             "description": "立即返回去皮状态、上一次称重结果和实时读数，不等待"},
                    "retract": {"params": ["side", "confirm_empty"],
                                "description": "收回手臂：先张开手指，再转腕让掌心朝内、放下前臂，回到 pose 之前的姿态。"
                                               "掌上检测到物体时拒绝，须先取走物体"},
                },
                # 与 arm / arm_gesture / hand 同一通道：pose/retract 经 arm 卡驱动双臂指令（未选中的手臂保持原位）、
                # 经 hand 卡设手型；tare/weigh 也要等手臂和手指上的其他动作做完。
                "x-completion": {"actions": list(ASYNC_ACTIONS), "timeout": COMPLETION_TIMEOUT_S},
                "x-resource": ["arm_l", "arm_r", "hand_l", "hand_r"],
            },
        }

    # ── 生命周期 ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """订阅力和关节状态，用本卡**自己的**执行器和线程。

        不挂到共用的 ros2.executor_tianyi 上：2026-11-23 在 release.261123 镜像上实测，
        共用的单线程执行器已经被各插件的高频话题占满——force_sensor 卡的右腕读数每秒
        只更新约 0.2 次，本卡挂在上面也只收到 26–42 Hz 的力数据（单独进程是 100 Hz）。

        /arm/status 约 490 Hz、每帧 14 个电机。这里先收原始字节（raw=True，不反序列化），
        按 joint_min_interval_s（默认 20 ms，约 50 Hz）抽一帧再解码，把额外开销降到最低。
        """
        with self._start_lock:
            if self._node is not None:
                return
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from rclpy.serialization import deserialize_message
            from geometry_msgs.msg import WrenchStamped
            from bodyctrl_msgs.msg import MotorStatusMsg

            ctx = self._ros2.ctx_tianyi
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             depth=_RELIABLE_DEPTH, durability=DurabilityPolicy.VOLATILE)
            node = Node("tianyi2_weigh_sub", context=ctx)
            engine = self._engine
            for side in self._sides:
                def on_force(msg, side=side):
                    w = msg.wrench
                    engine.add_force(side, (w.force.x, w.force.y, w.force.z,
                                            w.torque.x, w.torque.y, w.torque.z))
                node.create_subscription(WrenchStamped, FORCE_TOPICS[side], on_force, qos)

            min_interval = float(engine.cfg.get("joint_min_interval_s", 0.02))
            last = [0.0]

            def on_status_raw(data):
                now = time.monotonic()
                if now - last[0] < min_interval:
                    return
                last[0] = now
                msg = deserialize_message(data, MotorStatusMsg)
                engine.add_motor_status(
                    ((s.name, s.pos, s.speed, s.current, s.error) for s in msg.status), t=now)
            node.create_subscription(MotorStatusMsg, ARM_STATUS_TOPIC, on_status_raw, qos, raw=True)

            executor = SingleThreadedExecutor(context=ctx)
            executor.add_node(node)
            stop_event = threading.Event()

            def spin():
                failures = 0
                while not stop_event.is_set() and rclpy.ok(context=ctx):
                    try:
                        executor.spin_once(timeout_sec=0.1)
                    except Exception as exc:
                        failures += 1
                        if failures == 1 or failures % 100 == 0:
                            print(f"[weigh] callback failed (count={failures}): {str(exc)[:200]}",
                                  flush=True)
            thread = threading.Thread(target=spin, name="weigh-spin", daemon=True)
            thread.start()
            self._node, self._executor, self._stop_event, self._thread = node, executor, stop_event, thread
            print(f"[weigh] subscribed {[FORCE_TOPICS[s] for s in self._sides]} + {ARM_STATUS_TOPIC} "
                  f"on its own executor (subscribe-only; pose/retract go through the arm card; scale state {self._engine.cfg['state_file']})",
                  flush=True)

    def stop(self) -> bool:
        """取消进行中的动作并等它退出，再拆订阅。返回 True=已停止；False=动作还没退出
        （例如卡在 arm/hand 卡的调用里），此时保持订阅、报 stopping，由后台线程等它退出后再拆，
        期间占着 busy，新动作进不来。取消标志一直保持，动作醒来后不会再下发指令。"""
        worker = self._worker
        if worker is not None and worker.is_alive():
            self._engine.cancel()
            worker.join(timeout=5.0)
            if worker.is_alive():
                print("[weigh] stop: in-flight action still running after 5 s; "
                      "teardown deferred until it exits", flush=True)
                if not self._stopping:
                    self._stopping = True
                    threading.Thread(target=self._finish_stop, args=(worker,),
                                     name="weigh-stop", daemon=True).start()
                return False
        self._teardown()
        return True

    def _finish_stop(self, worker) -> None:
        worker.join()
        with self._busy:
            self._teardown()
            self._stopping = False
        print("[weigh] stop: deferred teardown done", flush=True)

    def _teardown(self) -> None:
        with self._start_lock:
            node, self._node = self._node, None
            if node is None:
                return
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            try:
                self._executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception as exc:
                print(f"[weigh] node teardown failed: {exc}", flush=True)
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            self._executor = self._thread = self._stop_event = None
            self._engine.clear_streams()

    # ── 调用 ────────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = {k: v for k, v in (args or {}).items() if not k.startswith("_")}
        if action == "start":
            try:
                self.start()
            except Exception as exc:
                return {"state": "error", "error": f"weigh start failed: {exc}"}
            return {"state": "ready"}
        if action == "stop":
            if self.stop():
                return {"state": "idle"}
            return {"state": "stopping", "error": "进行中的动作还没退出；它不会再下发指令，退出后自动停止"}
        if action == "info":
            return {"state": "ready" if self._node is not None else "idle", "sides": list(self._sides)}
        if action not in ("pose", "tare", "weigh", "calibrate", "read", "retract"):
            return {"error": f"unknown action: {action}"}
        if args.get("side") in (None, ""):
            result = {"state": "error", "code": "side_required", "experimental": True,
                      "error": "必须填写 side（left / right / both），并与 pose 时一致"}
            print(f"[weigh] {action} args={args} -> {_brief(result)}", flush=True)
            return result
        try:
            self.start()   # 卡片未 start 也能直接调用：首次调用时订阅
        except Exception as exc:
            return {"state": "error", "error": f"weigh start failed: {exc}"}
        t0 = time.monotonic()
        if action in ("pose", "retract"):
            print(f"[weigh] {action} called args={args}", flush=True)
        result = self._engine.read(args) if action == "read" else self._start_job(action, args, t0)
        print(f"[weigh] {action} args={args} -> {_brief(result)} ({time.monotonic() - t0:.1f}s)",
              flush=True)
        return result

    def _call_engine(self, action: str, args: dict) -> dict:
        try:
            return getattr(self._engine, action)(args)
        except Cancelled:
            print(f"[weigh] {action} cancelled by stop", flush=True)
            return {"state": "cancelled", "code": "cancelled", "experimental": True,
                    "error": "卡片已停止，动作被取消；手臂停在当前位置或上一段的目标位置"}
        except ValueError as exc:
            return {"state": "error", "code": "bad_arguments", "error": str(exc)}
        except Exception as exc:
            print(f"[weigh] {action} raised {type(exc).__name__}: {exc}", flush=True)
            return {"state": "error", "code": "internal_error", "error": f"{type(exc).__name__}: {exc}"}

    def _start_job(self, action: str, args: dict, t0: float) -> dict:
        """在工作线程里执行一个动作。SYNC_GRACE_S 内结束就同步返回结果；否则返回 action_id，
        做完后经 ACP 回调完成状态和完整结果。同一时间只允许一个动作。"""
        if self._stopping:
            return {"state": "error", "code": "stopping", "error": "卡片正在停止，等上一个动作退出后再调用"}
        if not self._busy.acquire(blocking=False):
            return {"state": "error", "code": "busy", "error": "本卡另一个动作正在进行"}
        if self._stopping:
            self._busy.release()
            return {"state": "error", "code": "stopping", "error": "卡片正在停止，等上一个动作退出后再调用"}
        self._engine.clear_cancel()       # 持有 busy 说明上一个动作已退出
        action_id = f"weigh_{action}_{uuid4().hex[:8]}"
        done = threading.Event()
        lock = threading.Lock()
        job = {"result": None, "async": False}

        def work():
            try:
                res = self._call_engine(action, args)
            finally:
                self._busy.release()      # 先释放，Agent Core 收到完成后的下一次调用不会撞上 busy
            with lock:
                job["result"] = res
                done.set()
                handed_off = job["async"]
            if not handed_off:
                return
            status = {"error": "error", "cancelled": "cancelled"}.get(res.get("state"), "completed")
            print(f"[weigh] {action} {action_id} {status} -> {_brief(res)} "
                  f"({time.monotonic() - t0:.1f}s)", flush=True)
            try:
                self._acp_notify(action_id, status, {"action": action, **res}, "weigh")
            except Exception as exc:
                print(f"[weigh] ACP callback failed for {action_id}: {exc}", flush=True)

        self._worker = threading.Thread(target=work, name=f"weigh-{action}", daemon=True)
        self._worker.start()
        if done.wait(self._sync_grace_s):
            return job["result"]
        with lock:
            if done.is_set():
                return job["result"]
            job["async"] = True
        return {"state": RUNNING_STATE[action], "action_id": action_id,
                "side": self._engine._side_arg(args), "experimental": True,
                "message": "动作在后台进行，完成后会收到结果；也可以之后用 read 查看 last_result"}
