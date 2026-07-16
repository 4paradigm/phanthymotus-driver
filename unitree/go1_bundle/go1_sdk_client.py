"""
go1_sdk_client.py — Unitree Go1 (EDU) 原始 unitree_legged_sdk 封装（go1_bundle bundle · 只读版）。

本 bundle 只做**状态读取**（loco_state / battery 两张卡），因此这里只保留一个
**只读的 HIGHLEVEL 客户端**：后台以 ~500Hz 走 UDP 收状态（HighState），把 comm.h 结构
解析成一份线程安全的 `snapshot()` dict。所有状态卡都读这同一份 snapshot，不各自开 UDP。

【实现基座】官方 unitree_legged_sdk（Go1 分支 v3.8.6）的 pybind11 绑定 `robot_interface`
（HighCmd/HighState/...）。镜像内 `cmake -DPYTHON_BUILD=ON` 按容器 python 版本构建，
rclpy 可同进程共存 → 状态卡能发 ROS2 topic 在画布渲染。

【降级 / STUB】导入不到 robot_interface（如开发机 Mac、无硬件）时进入 STUB：不收发、
snapshot 为空（fresh=False）。MCP server 仍能起、注册、列 tool，方便无硬件时跑通链路。

【控制原语（后来者看这里）】本文件原为只读；为支持控制卡（spin），已按 CONTRIBUTING.md
"新增控制卡"一节加入**最小高层下发能力**：`move(vx,vy,vyaw,gait)` / `stop_move()` 加锁写
目标，`_loop()` 每周期 `_compose_cmd()` 合成 `HighCmd` 后 `SetSend`。默认无目标时仍发 idle
心跳（mode=0），状态卡不受影响；move 带 0.5s 看门狗，停发即自动回 idle。低层 LowCmd/Safety
（关节控制，目标 .10:8007，与高层互斥）仍未引入，需要时另起 LOWLEVEL client。
"""

from __future__ import annotations

import struct
import threading
import time

# 控制字（unitree_legged_sdk 约定）
HIGHLEVEL = 0xEE

# 默认网络参数（Go1 板载网段；来自 udp.h：HIGH 目标 .161:8082）
DEFAULT_TARGET_IP = "192.168.123.161"   # UDP_SERVER_IP_SPORT（高层运动服务）
HIGH_TARGET_PORT = 8082
HIGH_LOCAL_PORT = 8090

LOOP_HZ = 500.0        # 后台收发频率（高层 2ms 亦可）

# ── 控制原语量程（高层 HighCmd 运动；spin 等控制卡用）───────────────────────────
#   量程取 Go1 高层安全值；控制卡应先自校验拒绝越界，client 再 clamp 作兜底。
VX_MAX = 1.0            # 前后 m/s
VY_MAX = 0.6            # 平移 m/s
VYAW_MAX = 2.0          # 偏航 rad/s
MOVE_WATCHDOG_S = 0.5   # 看门狗：超过此时长无新 move → 自动回 idle(mode=0) 停下（控制卡停发即自停）
POWER_OFF_CODE = 0xA5   # BmsCmd.off 关机命令字（power_control 卡用；终态、不可逆）

# HighState.mode（Go1 legacy comm.h）
MODE_NAMES = {0: "idle", 1: "force_stand", 2: "walk", 5: "stand_down",
              6: "stand_up", 7: "damp", 8: "recovery", 10: "jump_yaw", 11: "straight_hand"}

GAIT_NAMES = {0: "idle", 1: "trot", 2: "trot_run", 3: "climb_stair", 4: "trot_obstacle"}

FOOT_ORDER = ["FR", "FL", "RR", "RL"]
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

BMS_STATUS_NAMES = {0: "wakeup", 1: "discharge", 2: "charge", 3: "charger", 4: "precharge",
                    5: "charge_error", 6: "waterfall_light", 7: "self_discharge", 8: "junk"}


def _r(v, nd=4):
    try:
        return round(float(v), nd)
    except Exception:
        return 0.0


def _g(obj, name, default=None):
    """从 pybind 对象或 dict 防御式取字段（字段名对照 comm.h 逐一核实）。"""
    try:
        if isinstance(obj, dict):
            return obj.get(name, default)
        return getattr(obj, name, default)
    except Exception:
        return default


def _cartesian_list(v):
    """[Cartesian(x,y,z)] × 4 → [{'x','y','z'}]；缺失/异常返回 None。"""
    if v is None:
        return None
    try:
        out = []
        for e in list(v):
            if hasattr(e, "x"):
                out.append({"x": _r(e.x), "y": _r(e.y), "z": _r(e.z)})
            else:
                seq = list(e)
                out.append({"x": _r(seq[0]), "y": _r(seq[1]), "z": _r(seq[2])})
        return out
    except Exception:
        return None


def _to_bytes40(v) -> bytes:
    try:
        return bytes(bytearray([int(x) & 0xFF for x in list(v)][:40])).ljust(40, b"\x00")
    except Exception:
        return b"\x00" * 40


# ── comm.h 结构 → dict（可复用的解析块；新卡按需取用）──────────────────────────

def parse_imu(imu) -> dict | None:
    if imu is None:
        return None
    return {
        "quaternion_wxyz": [_r(q, 5) for q in _g(imu, "quaternion", [0, 0, 0, 0])],
        "gyroscope_rad_s": [_r(x) for x in _g(imu, "gyroscope", [0, 0, 0])],
        "accelerometer_m_s2": [_r(x) for x in _g(imu, "accelerometer", [0, 0, 0])],
        "rpy_rad": [_r(x) for x in _g(imu, "rpy", [0, 0, 0])],
        "temperature_c": int(_g(imu, "temperature", 0)),
    }


def parse_joints(motor_state) -> list | None:
    if motor_state is None:
        return None
    joints = []
    for i, m in enumerate(list(motor_state)[:12]):   # Go1 只用前 12 个腿部电机
        joints.append({
            "i": i, "mode": int(_g(m, "mode", 0)), "q": _r(_g(m, "q", 0.0)),
            "dq": _r(_g(m, "dq", 0.0)), "ddq": _r(_g(m, "ddq", 0.0)),
            "tau": _r(_g(m, "tauEst", 0.0)), "temp": int(_g(m, "temperature", 0)),
        })
    return joints


def parse_battery(bms) -> dict | None:
    if bms is None:
        return None
    status = int(_g(bms, "bms_status", 0))
    return {
        "version": {"high": int(_g(bms, "version_h", 0)), "low": int(_g(bms, "version_l", 0))},
        "status_code": status, "status_name": BMS_STATUS_NAMES.get(status, "unknown"),
        "soc_percent": int(_g(bms, "SOC", 0)), "current_ma": int(_g(bms, "current", 0)),
        "cycle_count": int(_g(bms, "cycle", 0)),
        "bq_ntc_c": [int(t) for t in _g(bms, "BQ_NTC", [])],
        "mcu_ntc_c": [int(t) for t in _g(bms, "MCU_NTC", [])],
        "cell_voltage_mv": [int(v) for v in _g(bms, "cell_vol", [])],
    }


def parse_wireless_remote(raw: bytes) -> dict:
    """按 joystick.h(xRockerBtnDataStruct) 解析 40 字节 → {buttons, axes}。
    布局: head[2] | btn(uint16 LE) | lx f | rx f | ry f | L2 f | ly f | idle[16]"""
    if not raw or len(raw) < 24:
        return {"buttons": {}, "axes": {}}
    try:
        btn = struct.unpack_from("<H", raw, 2)[0]
        lx, rx, ry, l2, ly = struct.unpack_from("<fffff", raw, 4)
        names = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
                 "A", "B", "X", "Y", "up", "right", "down", "left"]
        return {"buttons": {n: bool((btn >> i) & 1) for i, n in enumerate(names)},
                "axes": {"lx": _r(lx), "rx": _r(rx), "ry": _r(ry), "L2": _r(l2), "ly": _r(ly)}}
    except Exception:
        return {"buttons": {}, "axes": {}}


# ── HIGHLEVEL 只读客户端 ─────────────────────────────────────────────────────

class Go1HighSdkClient:
    """原始 SDK 高层**只读**客户端：后台 UDP 收 HighState → 线程安全 snapshot()。

    所有状态卡共用同一个实例（唯一 UDP 收发线程）。这里不下发任何控制命令：
    每个循环发一个由 InitCmdData 初始化的空闲 HighCmd 只是为了维持 UDP 会话（Go1 需
    持续心跳才回状态），mode 恒为 0(idle)，不会让机器人动。
    """

    def __init__(self, network_iface: str = "",
                 target_ip: str = DEFAULT_TARGET_IP,
                 target_port: int = HIGH_TARGET_PORT,
                 local_port: int = HIGH_LOCAL_PORT):
        self._target_ip = target_ip
        self._target_port = target_port
        self._local_port = local_port
        self._sdk = None
        self._udp = None
        self._cmd = None
        self._state = None
        self.available = False
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._snapshot: dict = {}
        # 控制目标（move()/stop_move() 写，_loop 读并合成 HighCmd）；None=只发 idle 心跳。
        self._move_cmd = None       # (vx, vy, vyaw, gait) 或 None
        self._move_deadline = 0.0   # monotonic 截止；过期即回 idle
        self._posture = None        # 【纯新增】dict: mode/euler/body_height/foot_raise/speed_level；持续保持(loco/gesture 用)
        self._power_off = False     # 【power_control 卡】一次置位后持续下发 BmsCmd.off（终态，不复位）
        self._desired_gait = 1      # 常驻期望步态（switch_gait 卡写；move 未显式指定 gait 时用它）。默认 1=trot
        # ── UDP 诊断计数（udp_diagnostics 卡读取；纯新增，不影响只读语义）──
        self._diag_lock = threading.Lock()
        self._diag = {
            "total_count": 0, "send_count": 0, "recv_count": 0,
            "send_error": 0, "flag_error": 0,
            "recv_crc_error": 0, "recv_lose_error": 0, "accessible": False,
        }
        self._init_sdk()

    def _init_sdk(self) -> None:
        try:
            import robot_interface as sdk  # unitree_legged_sdk pybind11 绑定
            self._sdk = sdk
            self._udp = sdk.UDP(HIGHLEVEL, self._local_port, self._target_ip, self._target_port)
            self._cmd = sdk.HighCmd()
            self._state = sdk.HighState()
            self._udp.InitCmdData(self._cmd)   # 空闲心跳命令（mode=0），只为维持会话
            self.available = True
            print(f"[Go1HighSdk] robot_interface ready → {self._target_ip}:{self._target_port}", flush=True)
        except Exception as e:
            print(f"[Go1HighSdk] ⚠ STUB（robot_interface 不可用: {e}）", flush=True)

    def start(self) -> None:
        if self.available and not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True, name="go1_high_udp")
            self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        period = 1.0 / LOOP_HZ
        while self._running:
            try:
                self._udp.Recv()
                self._udp.GetRecv(self._state)
                self._bump("recv_count")
                self._compose_cmd()            # 合成 _cmd：默认 idle；有 move 目标且未过期则下发速度
                self._udp.SetSend(self._cmd)
                self._udp.Send()
                self._bump("send_count")
                self._set_diag("accessible", True)
                self._read_udp_state()         # 尽力拉底层 CRC/丢包/标志错误计数
                self._parse_state(self._state)
            except Exception as e:
                self._bump("send_error")
                self._set_diag("accessible", False)
                print(f"[Go1HighSdk] loop error: {e}", flush=True)
            self._bump("total_count")
            time.sleep(period)

    # ── UDP 诊断计数（纯新增，供 udp_diagnostics 卡读取）─────────────────────────

    def _bump(self, key: str, n: int = 1) -> None:
        with self._diag_lock:
            self._diag[key] = self._diag.get(key, 0) + n

    def _set_diag(self, key: str, value) -> None:
        with self._diag_lock:
            self._diag[key] = value

    def _read_udp_state(self) -> None:
        """若 robot_interface.UDP 暴露了 udpState，则把底层 CRC/丢包/标志错误计数填进诊断。
        没暴露就静默跳过——绝不抛异常污染上面的 send_error 计数。"""
        us = getattr(self._udp, "udpState", None)
        if us is None:
            return
        for src, dst in (("FlagError", "flag_error"),
                         ("RecvCRCError", "recv_crc_error"),
                         ("RecvLoseError", "recv_lose_error")):
            try:
                v = getattr(us, src, None)
                if v is not None:
                    self._set_diag(dst, int(v))
            except Exception:
                pass

    def diagnostics(self) -> dict:
        """UDP 通信健康快照（线程安全拷贝）。STUB 下计数恒为 0、accessible=False。"""
        with self._diag_lock:
            return dict(self._diag)

    def _parse_state(self, s) -> None:
        """HighState → 一份完整 snapshot dict。

        当前 loco_state 只用其中 mode/gait/velocity/position/body_height/yaw_speed，
        battery 只用 battery。其余字段（imu/joints/foot_*/range_obstacle/wireless_remote）
        一并解析好放进 snapshot，是为了给后来者加卡（imu/joints/feet/... 卡）现成的数据源——
        新卡只需写一个 builder 读这些字段即可，不必再碰本文件。见 CONTRIBUTING.md。
        """
        try:
            out = {"fresh": True, "control_level": "HIGHLEVEL"}
            out["mode"] = int(_g(s, "mode", 0))
            out["mode_name"] = MODE_NAMES.get(out["mode"], "unknown")
            gt = int(_g(s, "gaitType", 0))
            out["gait_type"] = gt
            out["gait_name"] = GAIT_NAMES.get(gt, "unknown")
            out["progress"] = _r(_g(s, "progress", 0.0))
            out["body_height"] = _r(_g(s, "bodyHeight", 0.0))
            out["foot_raise_height"] = _r(_g(s, "footRaiseHeight", 0.0))
            out["yaw_speed"] = _r(_g(s, "yawSpeed", 0.0))
            vel = _g(s, "velocity", None)
            out["velocity"] = [_r(v) for v in vel] if vel is not None else None
            pos = _g(s, "position", None)
            out["position"] = [_r(p) for p in pos] if pos is not None else None
            out["imu"] = parse_imu(_g(s, "imu", None))
            out["joints"] = parse_joints(_g(s, "motorState", None))
            ff = _g(s, "footForce", None)
            out["foot_force"] = [int(f) for f in ff] if ff is not None else None
            out["foot_pos"] = _cartesian_list(_g(s, "footPosition2Body", None))
            out["foot_speed"] = _cartesian_list(_g(s, "footSpeed2Body", None))
            ro = _g(s, "rangeObstacle", None)
            out["range_obstacle"] = [_r(x) for x in ro] if ro is not None else None
            wr = _g(s, "wirelessRemote", None)
            out["wireless_remote"] = _to_bytes40(wr) if wr is not None else None
            out["battery"] = parse_battery(_g(s, "bms", None))
            with self._lock:
                self._snapshot = out
        except Exception as e:
            print(f"[Go1HighSdk] parse_state error: {e}", flush=True)

    # ── 控制原语（CONTRIBUTING §4：让只读 client 具备下发能力，供 spin 等控制卡用）──
    #   默认不动：无 move 目标时 _compose_cmd 发 idle(mode=0)，loco_state/battery 等状态卡不受影响。
    @staticmethod
    def _clamp(v, lim):
        try:
            return max(-lim, min(lim, float(v)))
        except Exception:
            return 0.0

    def move(self, vx=0.0, vy=0.0, vyaw=0.0, gait=None):
        """设置一次高层速度目标（mode=2）并刷新看门狗；后台 _loop 下发。
        控制卡按节奏（如每 50ms）重发以持续运动，停发 0.5s 后自动回 idle 停下。
        gait=None 时用常驻期望步态 self._desired_gait（由 switch_gait 卡设定）；
        显式传 gait 则以传入为准（向后兼容，spin 等卡不受影响）。"""
        vx = self._clamp(vx, VX_MAX)
        vy = self._clamp(vy, VY_MAX)
        vyaw = self._clamp(vyaw, VYAW_MAX)
        with self._lock:
            g = self._desired_gait if gait is None else int(gait)
            self._move_cmd = (vx, vy, vyaw, g)
            self._move_deadline = time.monotonic() + MOVE_WATCHDOG_S
            self._posture = None   # 速度命令优先，清掉姿态
        return {"vx": vx, "vy": vy, "vyaw": vyaw, "gait": g}

    def stop_move(self):
        """清除速度目标 → 下一循环回 idle(mode=0) 停下站稳。"""
        with self._lock:
            self._move_cmd = None
            self._move_deadline = 0.0
            self._posture = None

    def set_posture(self, mode, euler=(0.0, 0.0, 0.0), body_height=0.0,
                    foot_raise=0.0, speed_level=0):
        """【纯新增】设置并持续保持一个姿态命令(供 loco 站起/姿态、gesture 用)。
        mode: 0 idle / 1 force_stand(受 euler+bodyHeight 控) / 5 stand_down / 6 stand_up /
              7 damp / 8 recovery。持续下发直到被 move()/stop_move()/新 set_posture 覆盖。
        不影响任何现有 move/idle/状态读取逻辑。"""
        with self._lock:
            self._move_cmd = None
            self._posture = {"mode": int(mode),
                             "euler": [float(euler[0]), float(euler[1]), float(euler[2])],
                             "body_height": float(body_height),
                             "foot_raise": float(foot_raise),
                             "speed_level": int(speed_level)}
        return dict(self._posture)

    def request_power_off(self):
        """【power_control 卡】置位关机：之后 _compose_cmd 持续写 BmsCmd.off=0xA5
        （终态、不可逆、不复位）。一经置位不再清除，直到进程退出。"""
        with self._lock:
            self._power_off = True

    @staticmethod
    def _set_bms_off(c) -> None:
        """尽力写 HighCmd.bms.off=0xA5（不同绑定字段名可能不同 → 防御式，绝不抛异常）。"""
        try:
            bms = getattr(c, "bms", None)
            if bms is not None and hasattr(bms, "off"):
                bms.off = POWER_OFF_CODE
                c.bms = bms
        except Exception:  # noqa: BLE001
            pass
    def set_gait(self, gait) -> int:
        """设置常驻期望步态（switch_gait 卡用）；此后未显式指定 gait 的 move 都用它。
        若当前正在移动，立即把进行中的 move 目标步态也换成新步态（走时切步态）。"""
        g = int(gait)
        with self._lock:
            self._desired_gait = g
            if self._move_cmd is not None:
                vx, vy, vyaw, _ = self._move_cmd
                self._move_cmd = (vx, vy, vyaw, g)
        return g

    def desired_gait(self) -> int:
        """当前常驻期望步态（gaitType 整数）。"""
        with self._lock:
            return self._desired_gait

    def _compose_cmd(self) -> None:
        """按当前 move 目标 + 看门狗合成 _cmd：默认 idle(mode=0)；目标有效则速度(mode=2)。"""
        if self._cmd is None:
            return
        now = time.monotonic()
        with self._lock:
            mc = self._move_cmd if (self._move_cmd is not None and now < self._move_deadline) else None
            pose = None if mc is not None else self._posture   # 【纯新增】move 优先,否则用 posture
            power_off = self._power_off                        # 【power_control 卡】终态,优先级最高
        try:
            if power_off:                                      # 【power_control 卡】一旦置位持续下发关机字
                self._set_bms_off(self._cmd)
                return
            if mc is not None:
                vx, vy, vyaw, gait = mc
                self._cmd.mode = 2
                self._cmd.gaitType = self._desired_gait
                self._cmd.velocity = [float(vx), float(vy)]
                self._cmd.yawSpeed = float(vyaw)
            elif pose is not None:                             # 【纯新增分支】姿态命令(loco 站起/姿态、gesture)
                self._cmd.mode = int(pose["mode"])
                self._cmd.gaitType = self._desired_gait
                self._cmd.velocity = [0.0, 0.0]
                self._cmd.yawSpeed = 0.0
                self._cmd.euler = [pose["euler"][0], pose["euler"][1], pose["euler"][2]]
                self._cmd.bodyHeight = pose["body_height"]
                self._cmd.footRaiseHeight = pose["foot_raise"]
                self._cmd.speedLevel = pose["speed_level"]
            else:
                self._cmd.mode = 0
                self._cmd.gaitType = self._desired_gait
                self._cmd.velocity = [0.0, 0.0]
                self._cmd.yawSpeed = 0.0
        except Exception as e:  # noqa: BLE001
            print(f"[Go1HighSdk] compose_cmd error: {e}", flush=True)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snapshot) if self._snapshot else {"fresh": False}
