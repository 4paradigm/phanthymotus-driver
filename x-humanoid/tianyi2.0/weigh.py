"""x-humanoid/tianyi2.0/weigh.py — 天轶2.0 腕部六维力托举称重卡（实验性）。

卡片上只有两个动作（side 选 right / left / both，默认 right；both = 两只手各放一件、重量相加）：
  - ``weigh``：一次走完整个流程，不需要大模型编排——
      1. 手臂已在托举姿态、cup 手型、去皮有效：直接进入第 2 步；
         否则告诉用户"正在准备，等说可以放了再放"，把手臂抬到托举姿态（掌心朝上），
         四指立起围住掌心（cup），空手去皮（去皮时掌心必须空着）；
      2. 告诉用户"可以放了"，等人把物品放到掌心（默认最多 30 秒）；
      3. 检测到物品后等读数稳定，测量；
      4. 告诉用户重量。称完手臂保持举起，可以接着称下一件。
  - ``reset``：张开手指、放下手臂、清除去皮。掌上可能还有物品、或判断不了时先不动，
    请用户确认；60 秒内再调用一次 reset 才动。

卡片自己不出声。每一步要让用户知道的事作为"给用户的话"（to_user）发到 ``weigh_state``
传感器（``WeighStatePlugin``，``/{ns}/state/weigh``），只在有新消息时发布一条。画布上把它连到
decision_core，由 decision_core 决定转不转告、怎么转告（接了语音就说出来，接了飞书就发消息）；
没连时卡片照常工作，只是没人转告。转告可能晚几秒，所以这些话只说"已经是这样了"的事，
不预告马上要做的动作；卡片自己的时机只看传感器读数，不看话什么时候被转告。

两个动作都走 ACP（先返回 action_id 和一段说明，做完通知 Agent Core）。完成通知声明为静默，
而且 Agent Core 对成功的通知只保留 elapsed_s/distance_m/duration_s（collector._slim_acp_result），
所以重量和要对用户说的话都只经 weigh_state 发布：数值看 ``sides.<side>.last``（both 看 ``both_last``）。

维护用的 ``calibrate``（设定比例系数）和 ``read``（查看状态）不在卡片上显示，
经 MCP 直接调用，同步返回。

已知限制：零点只存在内存里。驱动重启时如果手臂还举着，本卡不知道掌上有没有东西，
下一次 weigh 会直接重新去皮；此时掌上若有物品，读数会错。重启后先 reset 再称最稳妥
（零点丢失时 reset 会请用户确认，再调用一次 reset 才动）。

只有 ``weigh`` / ``reset`` 会让手臂和手指运动，而且不自己发指令：经 bundle 调用 arm 卡的
``move_pos``（复用它的急停/故障/限位预检、电流上限和运动反馈）和 hand 卡的
``set_fingers_raw``。力和关节数据只订阅 ``/arm_6dof_{side}``
（WrenchStamped，约 100 Hz）和 ``/arm/status``（MotorStatusMsg，抽帧到约 50 Hz），
用本卡自己的执行器线程。

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
MAX_DURATION_S = 5.0

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
    "default_timeout_s": 8.0,
    "max_timeout_s": 10.0,           # 同步测量时限须远小于 agent-core 的 60 s actuator 超时
    "data_grace_s": 2.0,             # 刚启动时等数据到达的宽限
    "history_s": 12.0,               # 缓冲区保留时长（≥ 最长窗口 5 s；/arm/status 约 500 Hz）
    "poll_s": 0.2,
    "joint_min_interval_s": 0.02,    # /arm/status 抽帧间隔（原始约 490 Hz）
    # ── 托举姿态（左臂默认取右臂的镜像）──
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
    # ── 一键称重（卡片的 weigh 动作）──
    # 等人把物品放到掌心的时间，从发出"可以放了"算起。卡片自己不出声，这句话经 decision_core
    # 转告用户（语音或消息），什么时候真正传到人那里不确定，所以给得宽一些。
    "place_window_s": 30.0,
    "place_window_both_s": 30.0,     # side=both 时两只手都要放
    # 返回给调用方的准备时间估计。2026-11-25 实测：从垂下到去皮完成约 18 s；
    # 手臂已举着、只需重新摆手势和去皮约 6 s。
    "prepare_long_s": 20,
    "prepare_short_s": 10,
    "detect_kg": 0.05,               # 掌上读数超过此值算"放上了物品"（与 retract_max_load_kg 一致）
    "measure_timeout_s": 10.0,       # 检测到物品后，等读数稳定并测量的上限
    # reset 判断"掌上是否还有物品"时，同一姿态下的旧零点最多沿用多久。比去皮有效期长：
    # 称完手臂保持举起，过了 15 分钟再复位很常见；零点 20 分钟漂移 ≤ 8 g，远小于 50 g 的判据。
    "empty_check_max_age_s": 3600.0,
    # reset 判断掌上可能有东西、或判断不了（例如驱动重启后零点丢了）时，先拒绝并请用户确认；
    # 这么多秒内再调用一次 reset（同一个 side）才真正动作。从发出这句话算起；转告可能晚几秒，
    # 所以给得宽一些。
    "reset_confirm_window_s": 60.0,
    # 轻物体偏高的修正（分段线性）。每只手一张表 [[原始读数 kg, 修正系数], ...]，按原始读数升序：
    # mass = raw / (scale × f(raw))，f 在表内线性插值，低于第一个点取第一个点的值，高于最后一个点取 1。
    # 数据：2026-11-25 电子秤实测（水瓶 102–813 g，每个点 3 次、每组之前重新去皮）。400 g 以上
    # 原始读数和真实重量是同一个比例（右 1.066、左 1.075，写在比例文件里），越轻越偏高：
    # 100 g 时右手高约 11%、左手高约 17%。原因还没查清（三个方向分量没有记录），先按实测曲线修正；
    # 留一法估计，表内没测过的重量误差约 ±2.5%。
    "low_load_correction": {
        "right": [[0.1213, 1.115], [0.2296, 1.0845], [0.5397, 1.0]],
        "left": [[0.1308, 1.1695], [0.2291, 1.0758], [0.3381, 1.0242], [0.4310, 1.0]],
    },
}

SIDE_NAMES = {"right": "右手", "left": "左手", "both": "双手"}

# 托举序列的中间段：先抬肘、掌心朝内（腕 yaw 保持出发值），再转腕 yaw 让掌心朝上。
# 2026-11-23 现场确认：腕 yaw 负方向掌心朝上；先转腕再抬肘则掌心朝下，所以顺序不能反。
_WRIST_YAW = 4

# 这些原因说明"再等也没用"，立即返回。
_FATAL = {"no_force_data", "no_joint_data", "joint_error"}

# ── 给用户的话（to_user） ──────────────────────────────────────────────────────
# 卡片自己不出声：这些话发到 weigh_state 传感器，画布上把它连到 decision_core，由 decision_core
# 决定转不转告、怎么转告（接了语音就说出来，接了飞书就发消息）。转告可能晚几秒，所以只说
# "已经是这样了"的事，不预告马上要做的动作；每句话单独听都成立，晚到或被合并也不会误导人。
# 失败时按错误码选一句；没列出的错误码说一句通用的。
_FAIL_TO_USER = {
    "no_object": "这次没等到东西，称重先结束了。想称的话再叫我。",
    "not_stable": "东西一直在晃，没称出来。请放稳后再称一次。",
    "object_on_hand": "我手上好像有东西，称重前要先空着。请拿走后再称一次。",
    "unknown_start_pose": "我的手臂不在起始位置，没法开始称重，需要先复位。",
    "tare_failed": "准备的时候手上不够稳，没准备好。请别碰我的手，再称一次。",
    "hand_failed": "手指没摆好，称重停下了。",
    "pose_changed_since_tare": "我的手臂位置变了，需要先复位再称。",
    "joint_error": "手臂电机报错，称重停下了。",
    "servo_active": "手臂正在被别的程序控制，称重停下了。",
}
_FAIL_TO_USER_DEFAULT = "称重没有完成。"
_PALM = {"right": "右手掌心", "left": "左手掌心", "both": "两只手掌心"}
_ARM = {"right": "右臂", "left": "左臂", "both": "双臂"}


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
            "already_retracted", "tared", "tare_valid", "joints_deg", "load_kg", "stage", "seq",
            "to_user")
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
        self._hand = {s: None for s in SIDES}        # 本卡最后一次设成功的手型；None = 不确定
        self._last = {s: None for s in (*SIDES, "both")}
        # 发布给 weigh_state 传感器：每只手最近一次测出的结果（不随去皮/收回清除），
        # 以及最近一次 weigh 调用的结局。seq 每次 weigh 调用加一。
        self._seq = 0
        self._results = {s: None for s in (*SIDES, "both")}
        self._attempt = None
        self._reset_confirm = {}                     # side → reset 请用户确认后，可以确认的截止时间
        # 给用户的话：最近一条（weigh_state 的 to_user），以及收到新消息时要通知的回调
        # （weigh_state 用它立即发布一条）。
        self._msg_id = 0
        self._to_user = None
        self._listeners = []
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
        """缺省为 right（卡片上的默认值）。每次调用各自决定用哪只手，卡片不保存"当前手"
        去替别的调用方做选择。"""
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
        duration = min(max(duration, 1.0), MAX_DURATION_S, self.cfg["max_timeout_s"] - 1.0)
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

    def _low_load_factor(self, side, raw) -> float:
        """low_load_correction 表在 raw（kg，未除比例）处的修正系数；没有表或表无效时为 1。"""
        table = (self.cfg.get("low_load_correction") or {}).get(side) or []
        try:
            pts = sorted((float(r), float(f)) for r, f in table)
        except (TypeError, ValueError):
            return 1.0
        if not pts or any(not (0.5 <= f <= 2.0) for _, f in pts):
            return 1.0
        if raw <= pts[0][0]:
            return pts[0][1]
        for (r0, f0), (r1, f1) in zip(pts, pts[1:]):
            if raw <= r1:
                return f0 + (f1 - f0) * (raw - r0) / (r1 - r0) if r1 > r0 else f1
        return 1.0

    def _measure(self, side, a, tare, scale):
        cfg = self.cfg
        dF = _sub(a["F"][:3], tare["F"][:3])
        dT = _sub(a["F"][3:6], tare["F"][3:6])
        n = _norm(dF)
        raw = n / G
        f = self._low_load_factor(side, raw)
        mass = raw / (scale * f)
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
        return {"raw_kg": raw, "mass_kg": mass, "low_load_factor": f, "lever_arm_m": lever,
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
            "raw_kg": _r(m["raw_kg"]), "low_load_factor": _r(m["low_load_factor"], 4),
            **{k: (_r(v) if isinstance(v, float) else v) for k, v in sc.items()},
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
            # 比例系数对应 400 g 以上的直线段；轻物体先扣掉 low_load_correction 的修正
            new = m["raw_kg"] / (known * m.get("low_load_factor", 1.0))
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
            # 运动途中 servo 被恢复（或开始推流）：关节交给指令流，立即退出，不再发任何指令
            # （包括冻结），免得两边同时指挥同一组关节。
            problem = self._servo_guard()
            if problem:
                st = {s: self._arm_state(s) for s in SIDES}
                return {"ok": False, "code": "servo_active", "stage": label,
                        "error": f"运动途中 servo 卡开始推流（{problem}），本卡已停止发送指令",
                        "joints_deg": {s: [_r(x, 2) for x in st[s]["deg"]] for s in SIDES if st[s]}}
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
        # 每条 move_pos 都带两只手臂（未选中的保持原位），所以两只手臂任一报错都不动
        faulted = [s for s in SIDES if self._arm_state(s)["error"]]
        if faulted:
            return self._error("joint_error", side, f"{','.join(faulted)} 臂电机报错，不运动")
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
            for s in sides:
                self._hand[s] = None if problem else hand
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
        faulted = [s for s in SIDES if self._arm_state(s)["error"]]
        if faulted:
            return self._error("joint_error", side, f"{','.join(faulted)} 臂电机报错，不运动")
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
            # 张开手指、转腕或放下手臂都可能让物体掉下去。有可用的零点时看读数，否则要求确认。
            # 零点只在去皮时的姿态下有意义：姿态变了（例如停在中间姿态）就不能靠读数判断。
            load, problem = self._palm_load(s, cfg["empty_check_max_age_s"], now)
            if problem == "force_data_unavailable":
                return self._error("force_data_unavailable", side,
                                   f"{s} 手没有新鲜的力数据，无法判断掌上是否有物体")
            if problem is None:
                if load > cfg["retract_max_load_kg"] and args.get("_override_load"):
                    # 人在 reset 提示后再点了一次：读数说有东西，但人确认是空的
                    # （典型情况：去皮时手上已经有东西，拿走后读数反而"多"了）。
                    self._log(f"[weigh] retract {s}: reading {load * 1000:.0f} g, confirmed empty by user")
                elif load > cfg["retract_max_load_kg"]:
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
        for s in sides:
            self._hand[s] = None if hand_problem else "open"
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

    # ── 一键称重（卡片的 weigh / reset） ────────────────────────────────────────

    def _palm_load(self, side, max_age, now=None):
        """用同一姿态下的零点估计掌上重量（kg）。返回 (load, problem)，problem 非 None 时 load 为 None：
        not_tared / tare_expired / force_data_unavailable / pose_changed_since_tare。
        max_age 可以比去皮有效期长——这里只回答"掌上有没有东西"，不是给出称重结果。"""
        tare = self._tare[side]
        if tare is None:
            return None, "not_tared"
        now = self._clock() if now is None else now
        if now - tare["t"] > max_age:
            return None, "tare_expired"
        live = self.analyze(side, self.cfg["block_s"], now)
        if live.get("F") is None or live.get("q") is None:
            return None, "force_data_unavailable"
        dev = math.degrees(max(abs(x - y) for x, y in zip(live["q"], tare["q"])))
        if dev > self.cfg["joint_dev_deg"]:
            return None, "pose_changed_since_tare"
        return _norm(_sub(live["F"][:3], tare["F"][:3])) / G / self.scale_info(side)["scale"], None

    def _in_pose(self, side, now=None) -> bool:
        now = self._clock() if now is None else now
        cur = self._arm_deg(side, now)
        return cur is not None and self._within(cur, self._pose_deg(side), self.cfg["arrive_tol_deg"])

    def _ready(self, side) -> bool:
        """手臂在托举姿态、手型是 cup、去皮有效且姿态没变：可以直接请人放物品。"""
        if not self._in_pose(side) or self._hand.get(side) != self.cfg["default_hand"]:
            return False
        if self._tare_problem(side)[0]:
            return False
        return self._palm_load(side, self.cfg["tare_ttl_s"])[1] is None

    def add_listener(self, fn) -> None:
        """fn(msg)：每有一条新的给用户的话就调用一次（在动作线程里，必须很快）。"""
        self._listeners.append(fn)

    def _tell(self, kind: str, side: str, text: str, seq=None) -> dict:
        """记下一条给用户的话，并通知 weigh_state 发布。不等、不出声，也不判断有没有人在听。
        kind：preparing / ready / result / failed / reset_done / reset_confirm。"""
        self._msg_id += 1
        msg = {"id": self._msg_id, "kind": kind, "side": side,
               "seq": self._seq if seq is None else seq, "text": text,
               "at_ms": int(self._wall() * 1000)}
        self._to_user = msg
        self._log(f"[weigh] to_user #{msg['id']} {kind}: {text}")
        for fn in list(self._listeners):
            try:
                fn(dict(msg))
            except Exception as exc:
                self._log(f"[weigh] to_user listener failed: {exc}")
        return msg

    @staticmethod
    def _speak_mass(mass_g) -> str:
        if mass_g < 1000:
            return f"{int(round(mass_g))}克"
        return f"{mass_g / 1000:.2f}".rstrip("0").rstrip(".") + "千克"

    def _result_text(self, res) -> str:
        if res.get("state") == "weighed":
            if res.get("valid"):
                if "below_resolution" in (res.get("warnings") or []):
                    return "读数太小，没称出来。请把东西放在掌心中间，再称一次。"
                total = "两边一共" if res.get("side") == "both" else ""
                return f"称好了，{total}大约{self._speak_mass(res['mass_g'])}。东西可以拿走了。"
            if any(r.endswith(("current_high", "over_load_limit")) for r in res.get("reasons") or []):
                return "东西可能太重了，这次没称准，请换轻一点的。"
            return "这次没称准，请再称一次。"
        return _FAIL_TO_USER.get(res.get("code"), _FAIL_TO_USER_DEFAULT)

    def start_message(self, action: str, side: str) -> str:
        """weigh / reset 转入后台时返回给调用方的说明：现在在做什么、接下来会收到什么、
        这段时间不要做什么。按调用时的状态估计（手臂是否已举着、零点是否可用）。"""
        cfg = self.cfg
        arm = _ARM.get(side, side)
        if action == "reset":
            return (f"已开始复位（{SIDE_NAMES.get(side, side)}）：张开手指、放下{arm}。"
                    "复位完成，或者因为掌上可能还有东西而没有动时，weigh_state 传感器会发一条给用户的话"
                    "（to_user）；后一种情况要等用户确认掌心是空的（不要替用户确认），"
                    f"{cfg['reset_confirm_window_s']:g} 秒内再调用一次 reset（同一个 side）才会动。"
                    "完成通知是静默的。")
        sides = self._sides_of(side)
        if all(self._ready(s) for s in sides):
            prep = "手臂已在托举姿态、零点可用，直接等人放物品"
        elif all(self._in_pose(s) for s in sides):
            prep = f"手臂已举着，正在重新摆手势、去皮，约 {cfg['prepare_short_s']:g} 秒"
        else:
            prep = f"正在抬起{arm}、摆手势、去皮，约 {cfg['prepare_long_s']:g} 秒"
        window = cfg["place_window_both_s"] if len(sides) > 1 else cfg["place_window_s"]
        last = "both_last" if side == "both" else f"sides.{side}.last"
        return (f"已开始称重（{SIDE_NAMES.get(side, side)}）。{prep}；准备好后最多等 {window:g} 秒，"
                f"让人把物品放到{_PALM.get(side, side)}，放稳后约 {cfg['default_duration_s']:g} 秒出结果。"
                "这期间要让用户知道的事（正在准备、可以放了、称出的重量、出错原因）会作为给用户的话"
                "（to_user）发到 weigh_state 传感器，由你决定是否、如何转告用户（语音、消息等）。"
                f"完成通知是静默的、不带重量；重量见 weigh_state 的 {last}.mass_g，valid=true 才可用。"
                f"结束前不要再调用 weigh，也不要操作{arm}和手。")

    def _wait_for_object(self, sides, window):
        """等每只手的读数都超过 detect_kg（both 时两只手都放上了）；到时间只放了一部分也照样测，
        一件都没有才是 no_object。返回 (load_kg, problem)；problem 为 None、no_object，
        或零点不能用的原因（例如有人把手臂推离了去皮时的姿态）。"""
        t0 = self._clock()
        detect = self.cfg["detect_kg"]
        while True:
            loads = []
            for s in sides:
                load, problem = self._palm_load(s, self.cfg["tare_ttl_s"])
                if problem:
                    return None, problem
                loads.append(load)
            total = sum(loads)
            if all(x >= detect for x in loads):
                return total, None
            if self._clock() - t0 >= window:
                return total, (None if any(x >= detect for x in loads) else "no_object")
            self._sleep(self.cfg["poll_s"])

    def auto_weigh(self, args: dict) -> dict:
        """卡片的 weigh：必要时摆姿态并去皮 → 告诉用户可以放了 → 等放稳 → 测量 → 告诉用户结果。
        称完手臂不动。每一步要让用户知道的事经 _tell 发到 weigh_state，不等它被转告。"""
        side = self._side_arg(args)
        try:
            sides = self._sides_of(side)
        except ValueError as exc:
            return self._error("bad_side", side, str(exc))
        cfg = self.cfg
        self._seq += 1
        seq = self._seq
        palm = _PALM.get(side, side)

        if not all(self._ready(s) for s in sides):
            # 手臂已在托举姿态、零点却不能直接用（过期、驱动重启后手型不确定等）：重新去皮之前
            # 先确认掌上是空的，否则会把物品当成零点，之后的读数全错。
            for s in sides:
                if self._in_pose(s):
                    load, _ = self._palm_load(s, cfg["empty_check_max_age_s"])
                    if load is not None and load > cfg["detect_kg"]:
                        return self._finish_auto(self._error(
                            "object_on_hand", side,
                            f"{SIDE_NAMES[s]}掌上还有约 {load * 1000:.0f} g，去皮时须空着；请先取走再调用 weigh",
                            load_kg=_r(load)), "prepare", side, seq,
                            f"我{SIDE_NAMES[s]}掌心好像有东西，称重前要先空着。请拿走后再称一次。")
            # 不说"我要抬手"：这句话可能在手臂已经举好之后才传到人那里。只说正在准备，
            # 并且要等"可以放了"——去皮时掌心必须空着，提前放上去会被当成零点。
            self._tell("preparing", side,
                       f"我在准备称重，等我说“可以放了”，再把东西放到{palm}。在这之前先别碰我的手。", seq)
            prepared = self.pose({"side": side})
            if prepared.get("state") != "posed":
                return self._finish_auto(prepared, "prepare", side, seq)
        loads = [self._palm_load(s, cfg["tare_ttl_s"])[0] for s in sides]
        if not all(x is not None and x >= cfg["detect_kg"] for x in loads):
            window = cfg["place_window_both_s"] if len(sides) > 1 else cfg["place_window_s"]
            where = "请在两只手掌心各放一件" if side == "both" else f"请把东西放到{palm}"
            self._tell("ready", side, f"可以放了，{where}。放好后手拿开，几秒就能称好。", seq)
            _, problem = self._wait_for_object(sides, window)
            if problem == "no_object":
                return self._finish_auto(self._error(
                    "no_object", side,
                    f"可以放物品之后 {window:g} 秒内没有检测到物品（阈值 {cfg['detect_kg']} kg）。"
                    "手臂保持举起，可以再次调用 weigh"), "place", side, seq)
            if problem:
                return self._finish_auto(self._error(
                    problem, side, f"等待放物品时零点不能用了（{problem}），请 reset 后重新称重"),
                    "place", side, seq)
        res = self.weigh({"side": side, "timeout_s": cfg["measure_timeout_s"]})
        return self._finish_auto(res, "measure", side, seq)

    def _finish_auto(self, res, stage, side, seq, text=None):
        """记录结果（供 weigh_state 发布），再把结果作为给用户的话发出去。
        先记结果后发消息：weigh_state 发布这条消息时，结果已经在里面了。"""
        res = dict(res, stage=stage, seq=seq)
        if res.get("state") == "weighed":
            self._results[side] = {
                "seq": seq, "side": side, "valid": bool(res.get("valid")),
                "mass_kg": res.get("mass_kg"), "mass_g": res.get("mass_g"),
                "reasons": list(res.get("reasons") or []), "warnings": list(res.get("warnings") or []),
                "scale": res.get("scale"), "calibrated": res.get("calibrated"),
                "measured_at": res.get("measured_at"), "measured_at_ms": int(self._wall() * 1000)}
        self._attempt = {"seq": seq, "side": side, "stage": stage, "state": res.get("state"),
                         "code": res.get("code"), "at_ms": int(self._wall() * 1000)}
        text = text or self._result_text(res)
        ok = res.get("state") == "weighed" and res.get("valid")
        self._tell("result" if ok else "failed", side, text, seq)
        res["to_user"] = text
        return res

    def reset(self, args: dict) -> dict:
        """卡片的 reset：张开手指、放下手臂、清除去皮，完成后告诉用户。

        读数显示掌上有东西（object_on_hand），或判断不了（load_unknown，例如驱动重启后零点丢了）
        时，先拒绝并请用户确认；reset_confirm_window_s 秒内对同一个 side 再调用一次 reset，视为
        人已确认掌心是空的，照常复位。读数可能是错的：去皮时手上已经有东西，拿走后 |ΔF|
        同样会显示"有东西"（2026-11-25 真机上就这样卡住过），所以 object_on_hand 也允许人确认。"""
        side = self._side_arg(args)
        window = self.cfg["reset_confirm_window_s"]
        deadline = self._reset_confirm.pop(side, None)
        confirmed = deadline is not None and self._clock() <= deadline
        res = self.retract({"side": side, "confirm_empty": confirmed, "_override_load": confirmed})
        if res.get("state") == "retracted":
            for s in self._sides_of(res["side"]):
                self._tare[s] = None
                self._last[s] = None
            text = "好了，两只手都放下来了。" if side == "both" else "好了，手已经放下来了。"
            self._tell("reset_done", side, text)
            return dict(res, state="reset", to_user=text, **({"confirmed_empty": True} if confirmed else {}))
        text = {"object_on_hand": "我手上好像还有东西，请先拿走。确认是空的话，再复位一次就行。",
                "load_unknown": "我没法确认手上有没有东西。确认是空的话，再复位一次就行。"}.get(res.get("code"))
        if text:
            self._reset_confirm[side] = self._clock() + window
            self._tell("reset_confirm", side, text)
            what = "掌上读数显示还有物体" if res.get("code") == "object_on_hand" else "无法判断掌上是否有物体"
            res = dict(res, error=f"{what}；先取走物体，用户确认掌心已空后 {window:g} 秒内再调用一次 reset",
                       confirm_within_s=window, to_user=text)
        return res

    def snapshot(self) -> dict:
        """weigh_state 传感器的内容：每只手是否就绪、去皮状态和最近一次测出的结果。不阻塞。"""
        now = self._clock()
        sides = {}
        for s in self.cfg["sides"]:
            problem, age = self._tare_problem(s)
            sides[s] = {"in_pose": self._in_pose(s, now), "hand": self._hand.get(s),
                        "tare_valid": problem is None, "tare_age_s": _r(age, 1),
                        "last": self._results.get(s)}
        out = {"to_user": dict(self._to_user) if self._to_user else None,
               "experimental": True, "seq": self._seq, "sides": sides, "last_attempt": self._attempt}
        if self._results.get("both"):
            out["both_last"] = self._results["both"]
        return out

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


# 卡片上只露 weigh / reset，两者都会动手臂、要等人放物品，走 ACP：超过 SYNC_GRACE_S 时
# 先返回 action_id，做完通知 Agent Core。卡片不出声，也不靠完成通知传结果：
# agent-core/src/collector.py 的 _slim_acp_result 对成功的通知只保留 elapsed_s、distance_m、
# duration_s，而且完成通知声明为静默。要让用户知道的话和重量都发到 weigh_state 传感器。
CARD_ACTIONS = ("weigh", "reset")
ASYNC_ACTIONS = CARD_ACTIONS
# 维护用，不在卡片上显示，经 MCP 直接调用、同步返回：calibrate 设定比例系数，read 查看状态。
MAINTENANCE_ACTIONS = ("calibrate", "read")
ENGINE_METHODS = {"weigh": "auto_weigh", "reset": "reset", "calibrate": "calibrate"}
RUNNING_STATE = {"weigh": "weighing", "reset": "resetting"}
# 最坏情况：两段摆臂各约 13 s + 手指 2 × 1.5 s + 去皮 ≤ 10 s + 放物品 ≤ 30 s + 测量 ≤ 10 s ≈ 80 s，
# 留出余量。
COMPLETION_TIMEOUT_S = 150
# weigh_state 发布的消息顶层带的 priority：>0 时 agent-core 直接送 decision_core（见 WeighStatePlugin._produce）
WAKE_PRIORITY = 1
SYNC_GRACE_S = 0.5


def _default_acp_notify(action_id: str, status: str, result: dict, tool: str = "") -> None:
    """用 bundle 里通用的 ACP 回调（device._acp_notify）。延迟导入，保持本模块不依赖 ROS。"""
    from device import _acp_notify
    _acp_notify(action_id, status, result, tool)


class WeighPlugin:
    """腕部六维力称重卡。自己不创建任何发布者；手臂和手指经 bundle 调用对应的卡，
    给用户的话经 weigh_state 传感器发布。"""

    PREFIX = "weigh"

    def __init__(self, plugin_config: dict, namespace: str, ros2, arm_call=None, hand_call=None,
                 acp_notify=None, servo_call=None):
        """arm_call / hand_call / servo_call(args) → dict：由 bundle 提供，等同于对
        对应的卡调用 tools/call。
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

    def _side_choices(self) -> list:
        return list(self._sides) + (["both"] if len(self._sides) == 2 else [])

    def get_tool(self) -> dict:
        cfg = self._engine.cfg
        return {
            "name": "weigh",
            "type": "actuator",
            "description": (
                "天轶2.0 掌心称重（实验性）。weigh：机器人自己抬起手臂、四指立起托住掌心并去皮，"
                "然后等人把物品放到掌心，放稳后测量；称完手臂保持举起，可以接着称下一件。"
                "reset：张开手指、放下手臂（掌上可能还有物品时先请用户确认）。"
                "side 选 right（默认）、left，或 both（两只手各放一件，重量相加）。"
                "调用前手臂须垂下或已在托举姿态，周围无人无物。"
                "卡片自己不出声：过程中要让用户知道的事（正在准备、可以放了、称出的重量、出错原因、"
                "复位结果）作为给用户的话（to_user）发到 weigh_state 传感器，由你决定是否、如何转告用户。"
                "完成通知是静默的、不带重量；重量看 weigh_state 的 sides.<side>.last.mass_g"
                "（both 看 both_last；valid=true 才可用）。"
                f"适用约 {cfg['detect_kg']} kg 以上的静止物体，不能替代电子秤计量。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(CARD_ACTIONS),
                               "description": "weigh=称重（全自动） reset=复位：张开手指、放下手臂"},
                    "side": {"type": "string", "enum": self._side_choices(),
                             "default": "right" if "right" in self._sides else self._sides[0],
                             "description": "用哪只手，默认右手；both=两只手各放一件"},
                },
                "required": ["action"],
                "x-action-params": {
                    "weigh": {"params": ["side"],
                              "description": "称重：必要时抬臂并去皮（约 20 秒），等人放物品"
                                             f"（最多 {cfg['place_window_s']:g} 秒），放稳后测量；"
                                             "称完手臂保持举起"},
                    "reset": {"params": ["side"],
                              "description": "复位：张开手指、放下手臂、清除去皮。掌上可能还有东西时不动、"
                                             "请用户确认；用户确认掌心是空的以后再调用一次"},
                },
                # silent_completion：完成通知只解锁资源、不唤醒大模型。要让用户知道的事已经经
                # weigh_state 发过了，再唤醒一次只会重复（2026-11-25 实测主代理每收到一条完成
                # 通知就醒一次，开始长篇解读）。
                "x-completion": {"actions": list(ASYNC_ACTIONS), "timeout": COMPLETION_TIMEOUT_S,
                                 "silent_completion": True},
                # 经 arm 卡驱动双臂指令（未选中的手臂保持原位）、经 hand 卡设手型。
                "x-resource": ["arm_l", "arm_r", "hand_l", "hand_r"],
            },
        }

    def add_message_listener(self, fn) -> None:
        """weigh_state 用：每有一条新的给用户的话，fn(msg) 被调用一次。"""
        self._engine.add_listener(fn)

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
        if action not in (*CARD_ACTIONS, *MAINTENANCE_ACTIONS):
            return {"error": f"unknown action: {action}"}
        if action in CARD_ACTIONS:
            # 卡片动作：side 可省略，默认右手；只接受卡片上列出的选项。confirm_empty 不再对外
            # 提供（reset 改成"请用户确认后再调用一次"），外部传进来也忽略。
            args.pop("confirm_empty", None)
            side = args.get("side")
            if side in (None, ""):
                args["side"] = "right" if "right" in self._sides else self._sides[0]
            elif side not in self._side_choices():
                result = {"state": "error", "code": "bad_side", "experimental": True,
                          "error": f"side 只能是 {self._side_choices()}，收到 {side!r}"}
                print(f"[weigh] {action} args={args} -> {_brief(result)}", flush=True)
                return result
        try:
            self.start()   # 卡片未 start 也能直接调用：首次调用时订阅
        except Exception as exc:
            return {"state": "error", "error": f"weigh start failed: {exc}"}
        t0 = time.monotonic()
        if action in ASYNC_ACTIONS:
            print(f"[weigh] {action} called args={args}", flush=True)
        result = self._engine.read(args) if action == "read" else self._start_job(action, args, t0)
        print(f"[weigh] {action} args={args} -> {_brief(result)} ({time.monotonic() - t0:.1f}s)",
              flush=True)
        return result

    def _call_engine(self, action: str, args: dict) -> dict:
        try:
            return getattr(self._engine, ENGINE_METHODS[action])(args)
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
        """在可取消的工作线程中执行动作。卡片动作超过宽限期走 ACP，维护动作同步返回。"""
        if self._stopping:
            return {"state": "error", "code": "stopping", "error": "卡片正在停止，等上一个动作退出后再调用"}
        if not self._busy.acquire(blocking=False):
            return {"state": "error", "code": "busy", "error": "本卡另一个动作正在进行"}
        if self._stopping:
            self._busy.release()
            return {"state": "error", "code": "stopping", "error": "卡片正在停止，等上一个动作退出后再调用"}
        self._engine.clear_cancel()       # 持有 busy 说明上一个动作已退出
        action_id = f"weigh_{action}_{uuid4().hex[:8]}" if action in ASYNC_ACTIONS else None
        side = self._engine._side_arg(args)
        message = None
        if action_id is not None:
            # 按动作开始前的状态说明接下来会发生什么（手臂一动，这些状态就变了）
            try:
                message = self._engine.start_message(action, side)
            except Exception as exc:
                print(f"[weigh] start_message failed: {exc}", flush=True)
                message = f"{action} 进行中；给用户的话和结果见 weigh_state 传感器"
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
        if action_id is None:
            done.wait()
            return job["result"]
        if done.wait(self._sync_grace_s):
            return job["result"]
        with lock:
            if done.is_set():
                return job["result"]
            job["async"] = True
        return {"state": RUNNING_STATE[action], "action_id": action_id,
                "side": side, "experimental": True, "message": message}

    def snapshot(self) -> dict:
        """weigh_state 传感器读取的状态（见 WeighEngine.snapshot）。"""
        return self._engine.snapshot()


# ══════════════════════════════════════════════════════════════════════════════
# WeighStatePlugin — 传感器卡（name=weigh_state）：发布最近一次称重结果
# ══════════════════════════════════════════════════════════════════════════════

class WeighStatePlugin:
    """weigh 卡给用户的话和最近一次结果，发布到 ``/{ns}/state/weigh``（data/json）。

    只在 weigh 卡有话要让用户知道时发布一条（to_user 换成这条话，其余字段是当时的状态），
    平时不发：画布上把它连到 decision_core 时，每条消息都会唤醒一次大模型，定时发布会让它
    每秒醒好几次。``read`` 随时返回实时状态。只发布 JSON，不发任何控制指令；sensor 类型
    不受 ACP barrier 阻塞，weigh 进行中也能随时读。
    """

    PREFIX = "weigh_state"

    def __init__(self, plugin_config: dict, namespace: str, ros2, weigh_plugin):
        self._ros2 = ros2
        self._weigh = weigh_plugin
        self._topic = f"/{namespace}/state/weigh"
        self._lock = threading.Lock()
        self._running = False
        self._pub = None
        self._thread = None
        self._queue = deque()
        self._wake = threading.Event()
        weigh_plugin.add_message_listener(self._on_message)

    def get_tool(self) -> dict:
        return {
            "name": "weigh_state",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "天轶2.0 掌心称重的进展和结果（实验性，配合 weigh 卡使用）。只在 weigh 卡有话要让用户"
                "知道时发布一条，平时不发。to_user.text 是给用户的话：正在准备、可以放物品了、称出的重量、"
                "出错原因、复位结果等，由你决定是否转告、怎么转告（语音、消息等）；它说的是发出时已经"
                "成立的事，晚一点转告也不会错。weigh 卡自己不出声，你不转告用户就听不到。每条消息都带 "
                "priority=1，直接送到你这里。to_user.id 每条加一，to_user.kind 是 preparing / ready / "
                "result / failed / reset_done / reset_confirm。sides.<right|left>.last 是该手最近一次测出的"
                "结果：mass_kg / mass_g、valid（false 时不要使用重量）、reasons、seq；side=both 的结果在 "
                "both_last。in_pose / tare_valid 是消息发出时手臂是否在托举姿态、去皮是否有效。"
                "last_attempt 是最近一次 weigh 调用的结局（含失败，例如 no_object）。"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def _on_message(self, msg: dict) -> None:
        """weigh 的动作线程调用：排队，由发布线程发出。没有 start 时不发。"""
        with self._lock:
            if not self._running:
                return
            self._queue.append(msg)
        self._wake.set()

    def _produce(self, to_user: dict | None = None) -> dict:
        snap = self._weigh.snapshot()
        if to_user is not None:
            snap["to_user"] = to_user           # 这一条，而不是发布时最新的那条
        payload = {"to_user": snap.pop("to_user", None), **snap}
        if to_user is not None:
            # 发布出去的每一条都是要转告用户的话。agent-core 的 collector 把没带
            # priority 的话题消息当传感器读数（P=0），先交给后台监控子代理判断
            # 要不要上报；它不知道本卡不出声，实测会把"可以放了"之类判成不急、
            # 只记进记忆，decision_core 不被唤醒，话就没人说。JSON 顶层的
            # priority 由 collector 原样采用（_extract_priority），P>0 直接送
            # decision_core。read 读到的快照不是新消息，不带。
            payload["priority"] = WAKE_PRIORITY
        payload["timestamp_ms"] = int(time.time() * 1000)
        return payload

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from std_msgs.msg import String
            if self._pub is None:
                node = Node("tianyi2_weigh_state_pub", context=self._ros2.ctx_core)
                self._ros2.executor_core.add_node(node)
                qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                                 depth=_RELIABLE_DEPTH, durability=DurabilityPolicy.VOLATILE)
                self._pub = (node.create_publisher(String, self._topic, qos), String)
            try:
                self._weigh.start()      # 订阅力和关节数据，in_pose / tare 状态才是实时的
            except Exception as exc:
                print(f"[weigh_state] weigh subscription failed: {exc}", flush=True)
            self._running = True
            self._thread = threading.Thread(target=self._loop, name="weigh-state", daemon=True)
            self._thread.start()
        print(f"[weigh_state] publishing {self._topic} on each message to the user", flush=True)

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._queue.clear()
            thread, self._thread = self._thread, None
        self._wake.set()
        if thread is not None:
            thread.join(timeout=2.0)

    def _loop(self) -> None:
        pub, String = self._pub
        failures = 0
        while self._running:
            self._wake.wait(timeout=1.0)
            self._wake.clear()
            while self._running:
                with self._lock:
                    msg = self._queue.popleft() if self._queue else None
                if msg is None:
                    break
                try:
                    out = String()
                    out.data = json.dumps(self._produce(msg), ensure_ascii=False)
                    pub.publish(out)
                    print(f"[weigh_state] published to_user #{msg.get('id')}", flush=True)
                except Exception as exc:
                    failures += 1
                    if failures == 1 or failures % 100 == 0:
                        print(f"[weigh_state] publish failed (count={failures}): {exc}", flush=True)

    def dispatch(self, action: str, args: dict) -> dict | None:
        topic_out = [{"topic": self._topic, "format": "data/json"}]
        if action == "start":
            try:
                self.start()
            except Exception as exc:
                return {"state": "error", "error": f"weigh_state start failed: {exc}"}
            return {"state": "running", "topic_out": topic_out}
        if action == "stop":
            self.stop()
            return {"state": "idle", "topic_out": topic_out}
        if action == "info":
            return {"state": "running" if self._running else "idle", "topic_out": topic_out}
        if action == "read":
            return self._produce()
        return {"state": "error", "error": f"unknown action: {action}"}
