#!/usr/bin/env python3
"""
go1_ctrl.py — Go1 经典 SDK 控制核心(MT 卡片专用,单一硬件入口)。

依据 MT《Go1 动作/运控能力卡片》与官方 unitree_legged_sdk Go1(v3.8.6)的
comm.h / joystick.h / safety.h / example_walk.cpp。与旧 go1_hl(proprietary
robot_interface_high_level 后端)不同:本核心走**标准经典 UDP** 绑定
(module `robot_interface`,导出 UDP/HighCmd/HighState/LowCmd/LowState/Safety/BmsCmd),
拿得到 motorState/footForce/rangeObstacle/bms/wirelessRemote 全量状态,并支持低层关节控制。

设计(对齐 MT §8 Driver 实现要求):
  • 单一 UDP 收发线程 + 单一命令状态机;卡片只更新"期望命令",不各自建 UDP。
  • 按 1~10ms 周期持续发送(默认 2ms);启动 InitCmdData;校验收到状态包后才 ready。
  • loco/switch_gait/body_pose/special_motion 共享同一份 HighCmd,加锁由状态机合成。
  • 高层 move 看门狗:超保活时间速度清零 + mode=0。
  • 低层 joint_control 独占 LOWLEVEL;安全(PositionLimit/PowerProtect/PositionProtect/温度/超时)
    任一异常 → 全关节 Damping。
  • control_level 由启动配置定(HIGHLEVEL|LOWLEVEL),不运行中切换。
  • 无 SDK(开发 Mac / import 失败)→ 离线 fake:MCP 照常起,状态为合成数据、控制为回显。

⚠️ 经典 UDP 能否驱动本机 Legged_sport 尚待实机复核(见 memory go1-driver-project);
   LowCmd/Safety 细节按官方 API 编写,标 VERIFY 处连狗核对。Python 3.7 兼容。
"""
from __future__ import annotations

import importlib
import math
import struct
import sys
import threading
import time

HIGHLEVEL = 0xee
LOWLEVEL = 0xff

# 高层运动模式(comm.h / example_walk / MT)
MODE_IDLE = 0        # idle, default stand
MODE_FORCE_STAND = 1  # forced stand(姿态/高度在此模式下调)
MODE_WALK = 2
MODE_STAND_DOWN = 5
MODE_STAND_UP = 6
MODE_DAMP = 7
MODE_RECOVERY = 8
MODE_JUMP_YAW_LEFT = 10
MODE_STRAIGHT_HAND = 11

# 步态
GAITS = {"idle": 0, "trot": 1, "trot_run": 2, "climb_stair": 3, "trot_obstacle": 4}
GAIT_NAMES = {v: k for k, v in GAITS.items()}
MODE_NAMES = {0: "idle", 1: "force_stand", 2: "walk", 5: "stand_down",
              6: "stand_up", 7: "damp", 8: "recovery_stand",
              10: "jump_yaw_left", 11: "straight_hand"}

# 速度档位(speed_gear 卡:自然语言"走快点/慢点"→ 档位)。映射到闭环便捷动作
# (move_distance/turn_angle/spin)缺省速度;直接 move 显式给速度时不受影响。
SPEED_GEARS = {"slow":   {"linear_mps": 0.15, "yaw_rad_s": 0.3},
               "normal": {"linear_mps": 0.3,  "yaw_rad_s": 0.6},
               "fast":   {"linear_mps": 0.6,  "yaw_rad_s": 1.0}}


def _wrap_pi(a):
    """把角度归一化到 [-pi, pi](闭环导航算朝向误差用)。"""
    a = float(a)
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# 12 关节名与索引(MT 卡片映射;SDK 数组长 20,Go1 只用 0~11)
JOINT_NAMES = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
               "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]
JOINT_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}

# 关节位置硬限制(rad),按关节类型(MT §5.1)
JOINT_LIMIT = {"hip": (-1.047, 1.047), "thigh": (-0.663, 2.966), "calf": (-2.721, -0.837)}

# 低层电机模式常量(comm.h)
MOTOR_SERVO = 0x0A
MOTOR_DAMPING = 0x00
POS_STOP_F = 2.146e9    # PosStopF:关闭位置环
VEL_STOP_F = 16000.0    # VelStopF:关闭速度环
FEET_ORDER = ["FR", "FL", "RR", "RL"]
BMS_STATUS = {0: "wakeup", 1: "discharge", 2: "charge", 3: "charger", 4: "precharge",
              5: "charge_error", 6: "waterfall_light", 7: "self_discharge", 8: "junk"}


def _joint_kind(idx: int) -> str:
    return ("hip", "thigh", "calf")[idx % 3]


# ══════════════════════════════════════════════════════════════════════════════
#  wirelessRemote(40 字节)→ 按键 / 摇杆(joystick.h 布局)
# ══════════════════════════════════════════════════════════════════════════════
_BTN_ORDER = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
              "A", "B", "X", "Y", "up", "right", "down", "left"]


def parse_wireless_remote(arr):
    """arr: 长度 40 的 int(字节)序列 → {buttons, axes}。解析失败给全 0。"""
    try:
        b = bytes(int(x) & 0xFF for x in list(arr)[:40])
        if len(b) < 24:
            b = b + b"\x00" * (24 - len(b))
        btn = struct.unpack_from("<H", b, 2)[0]          # 2 字节位掩码
        lx, rx, ry, l2, ly = struct.unpack_from("<5f", b, 4)
        buttons = {name: bool(btn & (1 << i)) for i, name in enumerate(_BTN_ORDER)}
        axes = {"lx": round(lx, 4), "rx": round(rx, 4), "ry": round(ry, 4),
                "L2": round(l2, 4), "ly": round(ly, 4)}
        return {"buttons": buttons, "axes": axes}
    except Exception:  # noqa: BLE001
        return {"buttons": {n: False for n in _BTN_ORDER},
                "axes": {"lx": 0.0, "rx": 0.0, "ry": 0.0, "L2": 0.0, "ly": 0.0}}


# ══════════════════════════════════════════════════════════════════════════════
#  防御性取值
# ══════════════════════════════════════════════════════════════════════════════
def _get(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def _num(obj, name, default=0.0):
    try:
        return float(_get(obj, name, default))
    except Exception:  # noqa: BLE001
        return default


def _flist(v, n):
    try:
        return [round(float(x), 6) for x in list(v)[:n]]
    except Exception:  # noqa: BLE001
        return [0.0] * n


def _ilist(v, n):
    try:
        return [int(x) for x in list(v)[:n]]
    except Exception:  # noqa: BLE001
        return [0] * n


# ══════════════════════════════════════════════════════════════════════════════
#  HighState / LowState → dict
# ══════════════════════════════════════════════════════════════════════════════
def snapshot_high(s) -> dict:
    imu = _get(s, "imu")
    imu_d = {
        "quaternion_wxyz": _flist(_get(imu, "quaternion", []), 4) if imu is not None else [1.0, 0, 0, 0],
        "gyroscope_rad_s": _flist(_get(imu, "gyroscope", []), 3) if imu is not None else [0, 0, 0],
        "accelerometer_m_s2": _flist(_get(imu, "accelerometer", []), 3) if imu is not None else [0, 0, 0],
        "rpy_rad": _flist(_get(imu, "rpy", []), 3) if imu is not None else [0, 0, 0],
        "temperature_c": int(_num(imu, "temperature", 0)) if imu is not None else 0,
    }
    motors = []
    ms = _get(s, "motorState", []) or []
    for i in range(min(12, len(ms))):
        m = ms[i]
        motors.append({"q_rad": round(_num(m, "q"), 6), "dq_rad_s": round(_num(m, "dq"), 6),
                       "ddq_rad_s2": round(_num(m, "ddq"), 6),
                       "tau_est_nm": round(_num(m, "tauEst", _num(m, "tau")), 5),
                       "temperature_c": int(_num(m, "temperature", 0)),
                       "mode": int(_num(m, "mode", 0))})
    foot_pos = []
    foot_spd = []
    fp = _get(s, "footPosition2Body", []) or []
    fs = _get(s, "footSpeed2Body", []) or []
    for i in range(4):
        p = fp[i] if i < len(fp) else None
        v = fs[i] if i < len(fs) else None
        foot_pos.append({"x": round(_num(p, "x"), 5), "y": round(_num(p, "y"), 5), "z": round(_num(p, "z"), 5)})
        foot_spd.append({"x": round(_num(v, "x"), 5), "y": round(_num(v, "y"), 5), "z": round(_num(v, "z"), 5)})
    return {
        "mode": int(_num(s, "mode", 0)),
        "gait_type": int(_num(s, "gaitType", 0)),
        "body_height_m": round(_num(s, "bodyHeight", 0), 5),
        "foot_raise_height_m": round(_num(s, "footRaiseHeight", 0), 5),
        "position_m": _flist(_get(s, "position", []), 3),
        "velocity": _flist(_get(s, "velocity", []), 3),
        "yaw_speed_rad_s": round(_num(s, "yawSpeed", 0), 5),
        "imu": imu_d,
        "motors": motors,
        "foot_force_raw": _ilist(_get(s, "footForce", []), 4),
        "foot_position_to_body": foot_pos,
        "foot_speed_to_body": foot_spd,
        "range_obstacle_raw": _flist(_get(s, "rangeObstacle", []), 4),
        "wireless_remote": parse_wireless_remote(_get(s, "wirelessRemote", []) or []),
        "bms": _snapshot_bms(_get(s, "bms")),
    }


def _snapshot_bms(bms) -> dict:
    if bms is None:
        return {}
    ver = _get(bms, "version_high", _get(bms, "version_h"))
    status = int(_num(bms, "bms_status", _num(bms, "status", 0)))
    return {
        "version": {"high": int(_num(bms, "version_high", 0)), "low": int(_num(bms, "version_low", 0))}
        if ver is not None else {},
        "status_code": status,
        "status_name": BMS_STATUS.get(status, "unknown"),
        "soc_percent": int(_num(bms, "SOC", _num(bms, "soc", 0))),
        "current_ma": int(_num(bms, "current", 0)),
        "cycle_count": int(_num(bms, "cycle", 0)),
        "bq_ntc_c": _ilist(_get(bms, "BQ_NTC", []), 2),
        "mcu_ntc_c": _ilist(_get(bms, "MCU_NTC", []), 2),
        "cell_voltage_mv": _ilist(_get(bms, "cell_vol", []), 10),
    }


def snapshot_low(s) -> dict:
    motors = []
    ms = _get(s, "motorState", []) or []
    for i in range(min(12, len(ms))):
        m = ms[i]
        motors.append({"index": i, "name": JOINT_NAMES[i], "mode": int(_num(m, "mode", 0)),
                       "q_rad": round(_num(m, "q"), 6), "dq_rad_s": round(_num(m, "dq"), 6),
                       "ddq_rad_s2": round(_num(m, "ddq"), 6),
                       "tau_est_nm": round(_num(m, "tauEst", _num(m, "tau")), 5),
                       "temperature_c": int(_num(m, "temperature", 0))})
    imu = _get(s, "imu")
    return {
        "motors": motors,
        "foot_force_raw": _ilist(_get(s, "footForce", []), 4),
        "imu": {
            "quaternion_wxyz": _flist(_get(imu, "quaternion", []), 4) if imu is not None else [1.0, 0, 0, 0],
            "gyroscope_rad_s": _flist(_get(imu, "gyroscope", []), 3) if imu is not None else [0, 0, 0],
            "accelerometer_m_s2": _flist(_get(imu, "accelerometer", []), 3) if imu is not None else [0, 0, 0],
            "rpy_rad": _flist(_get(imu, "rpy", []), 3) if imu is not None else [0, 0, 0],
            "temperature_c": int(_num(imu, "temperature", 0)) if imu is not None else 0,
        },
        "bms": _snapshot_bms(_get(s, "bms")),
        "wireless_remote": parse_wireless_remote(_get(s, "wirelessRemote", []) or []),
    }


# ══════════════════════════════════════════════════════════════════════════════
#  Go1Control:控制核心
# ══════════════════════════════════════════════════════════════════════════════
class Go1Control:
    def __init__(self, cfg: dict):
        self._cfg = cfg or {}
        self.control_level = str(self._cfg.get("control_level", "HIGHLEVEL")).upper()
        self._module = self._cfg.get("sdk_module", "robot_interface")
        self._build_path = self._cfg.get("sdk_build_path", "")
        # 后端选择:auto|standard|factory|fake。
        #   standard = 标准 unitree_legged_sdk UDP(HighCmd/HighState,本机 Recv 恒 -1 暂不可用)
        #   factory  = 工厂 robot_interface_high_level(RobotInterface,已验证能读本机真实 HighState;
        #              只高层、无 motorState/rangeObstacle/BmsState → joints/obstacle_range/battery 残缺)
        #   fake     = 离线合成。auto = 先试 standard 再 factory,都不行则 fake。
        self._backend = str(self._cfg.get("backend", "auto")).lower()
        self._factory_module = self._cfg.get("factory_module", "robot_interface_high_level")
        self._factory_build_path = self._cfg.get(
            "factory_build_path", "/home/pi/Unitree/autostart/programming/build")
        self._mode_backend = None      # 实际生效后端:standard|factory|fake
        self._robot = None             # factory 后端的 RobotInterface 实例
        self._ip = self._cfg.get("robot_ip", "192.168.123.161")
        self._local_port = int(self._cfg.get("local_port", 8080))
        self._target_port = int(self._cfg.get("target_port", 8082))
        self._dt = float(self._cfg.get("send_period_s", 0.002))       # 默认 2ms
        self._move_timeout = float(self._cfg.get("move_timeout_s", 0.5))
        self._power_factor = int(self._cfg.get("power_protect_factor", 1))  # 1~10
        self._pos_err_limit = float(self._cfg.get("position_protect_limit_rad", 0.087))

        self._lock = threading.Lock()
        self._sdk = None
        self._udp = None
        self._cmd = None
        self._state = None
        self._connected = False
        self._running = False
        self._thread = None
        self._level_flag = HIGHLEVEL if self.control_level == "HIGHLEVEL" else LOWLEVEL

        # 高层期望(状态机合成用)
        self._hi = {"mode": MODE_IDLE, "gaitType": 0, "speedLevel": 0,
                    "footRaiseHeight": 0.0, "bodyHeight": 0.0,
                    "euler": [0.0, 0.0, 0.0], "velocity": [0.0, 0.0], "yawSpeed": 0.0,
                    "move_deadline": 0.0}
        # special_motion 序列器:None 或 {"mode":10/11,"phase":"stabilize|active","deadline":t}
        self._special = None
        self._special_stabilize_s = float(self._cfg.get("special_stabilize_s", 1.0))
        self._special_active_s = float(self._cfg.get("special_active_s", 4.0))
        self._seq_id = 0

        # 闭环运动宏(move_distance/turn_angle):后台线程读里程计/yaw,走够即停。
        # 供大模型下"向前走2米/左转90度"这种单条指令(不用它自己拿速度+看门狗凑)。
        self._macro_cancel = threading.Event()
        self._macro_thread = None
        self._macro_status = {"active": False}

        # 里程计累加(odometry 卡):收发循环内按 position 水平增量累计总路程(防跳变)。
        self._odometer_m = 0.0
        self._odo_last = None
        # 速度档位(speed_gear 卡):闭环便捷动作缺省速度取此档;可 config 预设。
        self._speed_gear = str(self._cfg.get("speed_gear", "normal")).lower()
        if self._speed_gear not in SPEED_GEARS:
            self._speed_gear = "normal"

        # 低层期望:每关节 MotorCmd 目标(默认全 damping)
        self._lo = [dict(q=POS_STOP_F, dq=VEL_STOP_F, Kp=0.0, Kd=0.0, tau=0.0, mode=MOTOR_DAMPING)
                    for _ in range(12)]
        self._power_off_pending = False

        # UDP 诊断计数
        self._diag = {"total_count": 0, "send_count": 0, "recv_count": 0, "send_error": 0,
                      "flag_error": 0, "recv_crc_error": 0, "recv_lose_error": 0, "accessible": False}
        self._last_state_mono = 0.0
        self._safety_tripped = False

    # ── 生命周期 ──────────────────────────────────────────────────────────────
    def start(self):
        order = {"auto": ["standard", "factory"], "standard": ["standard"],
                 "factory": ["factory"], "fake": []}.get(self._backend, ["standard", "factory"])
        for name in order:
            try:
                if name == "standard":
                    self._start_standard()
                else:
                    self._start_factory()
                self._mode_backend = name
                self._connected = True
                print(f"[go1_ctrl] ✓ 后端={name} level={self.control_level}", flush=True)
                break
            except Exception as e:  # noqa: BLE001
                print(f"[go1_ctrl] 后端 {name} 不可用: {e}", flush=True)
                self._connected = False
        if not self._connected:
            self._mode_backend = "fake"
            print("[go1_ctrl] → 离线 fake(MCP 照常起,状态合成/控制回显)", flush=True)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="go1_ctrl")
        self._thread.start()

    def _start_standard(self):
        """标准 unitree_legged_sdk:UDP + HighCmd/HighState(或 LowCmd/LowState)。"""
        if self._build_path and self._build_path not in sys.path:
            sys.path.append(self._build_path)
        self._sdk = importlib.import_module(self._module)
        self._udp = self._sdk.UDP(self._level_flag, self._local_port, self._ip, self._target_port)
        if self.control_level == "HIGHLEVEL":
            self._cmd = self._sdk.HighCmd()
            self._state = self._sdk.HighState()
        else:
            self._cmd = self._sdk.LowCmd()
            self._state = self._sdk.LowState()
            self._safety = self._sdk.Safety(self._sdk.LeggedType.Go1)
        self._udp.InitCmdData(self._cmd)

    def _start_factory(self):
        """工厂 robot_interface_high_level:RobotInterface(robotControl/UDPSend/UDPRecv/getState)。
        只高层,无 LowState/关节/BmsState → joints/obstacle_range/battery 由 capabilities() 标残缺。"""
        if self.control_level != "HIGHLEVEL":
            raise RuntimeError("factory 后端仅支持 HIGHLEVEL(工厂 .so 无 LowState/关节控制)")
        if self._factory_build_path and self._factory_build_path not in sys.path:
            sys.path.append(self._factory_build_path)
        self._sdk = importlib.import_module(self._factory_module)
        self._robot = self._sdk.RobotInterface()
        self._state = self._robot.getState()      # 首帧(验证能读)

    def stop(self):
        self.cancel_macro()
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def is_connected(self) -> bool:
        return self._connected

    # ── 控制循环(唯一 UDP 收发处) ─────────────────────────────────────────────
    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            if self._mode_backend == "standard":
                self._loop_standard(t0)
            elif self._mode_backend == "factory":
                self._loop_factory(t0)
            else:
                self._fake_step(t0)
            dt = self._dt - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    def _loop_standard(self, t0):
        try:
            self._diag["total_count"] += 1
            r = self._udp.Recv()
            self._udp.GetRecv(self._state)
            if isinstance(r, int) and r > 0:
                self._diag["recv_count"] += 1
                self._last_state_mono = t0
                self._diag["accessible"] = True
                self._accum_odo()
            with self._lock:
                self._compose_cmd(t0)
            self._udp.SetSend(self._cmd)
            s = self._udp.Send()
            self._diag["send_count"] += 1
            if isinstance(s, int) and s < 0:
                self._diag["send_error"] += 1
            # 反馈超时判定
            if self._last_state_mono and (t0 - self._last_state_mono) > 0.5:
                self._diag["accessible"] = False
        except Exception as e:  # noqa: BLE001
            self._diag["send_error"] += 1
            print(f"[go1_ctrl] loop err: {e}", flush=True)

    def _loop_factory(self, t0):
        # 工厂只高层:合成目标 → robotControl → UDPSend/UDPRecv → getState(读真实 HighState)。
        # 状态卡阶段 self._hi 默认 idle(mode=0),下发中性命令不会让狗动。
        try:
            self._diag["total_count"] += 1
            with self._lock:
                mode, gait, speed, foot_raise, body_h, euler, vel, yaw = self._resolve_high(t0)
            self._robot.robotControl(int(mode), int(gait), int(speed), float(foot_raise), float(body_h),
                                     float(euler[0]), float(euler[1]), float(euler[2]),
                                     float(vel[0]), float(vel[1]), float(yaw))
            self._robot.UDPSend()
            self._diag["send_count"] += 1
            self._robot.UDPRecv()
            st = self._robot.getState()
            if st is not None:
                self._state = st
                self._diag["recv_count"] += 1
                self._last_state_mono = t0
                self._diag["accessible"] = True
                self._accum_odo()
            if self._last_state_mono and (t0 - self._last_state_mono) > 0.5:
                self._diag["accessible"] = False
        except Exception as e:  # noqa: BLE001
            self._diag["send_error"] += 1
            print(f"[go1_ctrl] factory loop err: {e}", flush=True)

    def _compose_cmd(self, now):
        c = self._cmd
        if self.control_level == "HIGHLEVEL":
            self._compose_high(c, now)
        else:
            self._compose_low(c, now)

    def _resolve_high(self, now):
        """期望 + special 序列器 + move 看门狗 → 实际下发目标(standard/factory 共用)。
        返回 (mode, gait, speed, foot_raise, body_h, euler[3], vel[2], yaw)。"""
        h = self._hi
        # special_motion 序列器优先
        if self._special is not None:
            sp = self._special
            if now >= sp["deadline"]:
                if sp["phase"] == "stabilize":
                    sp["phase"] = "active"; sp["deadline"] = now + self._special_active_s
                else:
                    self._special = None                       # 超时回落
            mode = MODE_FORCE_STAND if (self._special and self._special["phase"] == "stabilize") \
                else (self._special["mode"] if self._special else MODE_FORCE_STAND)
            return (mode, 0, 0, 0.0, 0.0, [0.0, 0.0, 0.0], [0.0, 0.0], 0.0)
        # move 看门狗
        mode = h["mode"]
        vel = list(h["velocity"]); yaw = h["yawSpeed"]
        if mode == MODE_WALK and now > h["move_deadline"]:
            mode = MODE_IDLE                    # 保活超时 → 回 idle
        # 速度只在 WALK 模式有意义;其余模式一律清零,防陈旧速度泄漏到
        # 站立/姿态/特殊动作(set_attitude 后 velocity 仍留在 _hi)。
        if mode != MODE_WALK:
            vel = [0.0, 0.0]; yaw = 0.0
        return (mode, h["gaitType"], h["speedLevel"], h["footRaiseHeight"], h["bodyHeight"],
                list(h["euler"]), vel, yaw)

    def _compose_high(self, c, now):
        mode, gait, speed, foot_raise, body_h, euler, vel, yaw = self._resolve_high(now)
        self._write_high(c, mode=mode, gait=gait, speed=speed, foot_raise=foot_raise,
                         body_h=body_h, euler=euler, vel=vel, yaw=yaw)

    def _write_high(self, c, mode=0, gait=0, speed=0, foot_raise=0.0, body_h=0.0,
                    euler=(0.0, 0.0, 0.0), vel=(0.0, 0.0), yaw=0.0):
        try:
            c.mode = int(mode); c.gaitType = int(gait); c.speedLevel = int(speed)
            c.footRaiseHeight = float(foot_raise); c.bodyHeight = float(body_h)
            c.euler = [float(euler[0]), float(euler[1]), float(euler[2])]
            c.velocity = [float(vel[0]), float(vel[1])]; c.yawSpeed = float(yaw)
            if self._power_off_pending:
                try:
                    c.bms.off = 0xA5
                except Exception:  # noqa: BLE001
                    pass
        except Exception:  # noqa: BLE001
            pass

    def _compose_low(self, c, now):
        # 安全:反馈超时 → 全关节 damping
        if self._last_state_mono and (now - self._last_state_mono) > 0.5:
            self._safety_tripped = True
        for i in range(12):
            j = self._lo[i]
            try:
                mc = c.motorCmd[i]
                if self._safety_tripped:
                    mc.q = POS_STOP_F; mc.dq = VEL_STOP_F; mc.Kp = 0.0; mc.Kd = 0.0; mc.tau = 0.0; mc.mode = MOTOR_DAMPING
                else:
                    mc.q = float(j["q"]); mc.dq = float(j["dq"]); mc.Kp = float(j["Kp"])
                    mc.Kd = float(j["Kd"]); mc.tau = float(j["tau"]); mc.mode = int(j["mode"])
            except Exception:  # noqa: BLE001
                pass
        # 安全函数(按官方 API;VERIFY 实机)
        try:
            self._safety.PositionLimit(c)
            self._safety.PowerProtect(c, self._state, self._power_factor)
            self._safety.PositionProtect(c, self._state, self._pos_err_limit)
        except Exception:  # noqa: BLE001
            pass
        if self._power_off_pending:
            try:
                c.bms.off = 0xA5
            except Exception:  # noqa: BLE001
                pass

    # ── 离线 fake ──────────────────────────────────────────────────────────────
    def _fake_step(self, now):
        wob = 0.02 * math.sin(now)
        motors = [{"q_rad": round(0.05 * math.sin(now + i), 4), "dq_rad_s": 0.0, "ddq_rad_s2": 0.0,
                   "tau_est_nm": 0.0, "temperature_c": 34, "mode": MOTOR_SERVO} for i in range(12)]
        self._fake = {
            "mode": self._hi["mode"], "gait_type": self._hi["gaitType"],
            "body_height_m": 0.28 + self._hi["bodyHeight"],
            "foot_raise_height_m": 0.08 + self._hi["footRaiseHeight"],
            "position_m": [0.0, 0.0, 0.0],
            "velocity": [self._hi["velocity"][0], self._hi["velocity"][1], 0.0],
            "yaw_speed_rad_s": self._hi["yawSpeed"],
            "imu": {"quaternion_wxyz": [1.0, round(wob, 4), 0.0, 0.0], "gyroscope_rad_s": [0, 0, 0],
                    "accelerometer_m_s2": [0, 0, 9.81], "rpy_rad": [round(wob, 4), 0.0, 0.0], "temperature_c": 35},
            "motors": motors, "foot_force_raw": [90, 92, 88, 91],
            "foot_position_to_body": [{"x": 0.18, "y": -0.13, "z": -0.28}] * 4,
            "foot_speed_to_body": [{"x": 0.0, "y": 0.0, "z": 0.0}] * 4,
            "range_obstacle_raw": [0.0, 0.0, 0.0, 0.0],
            "wireless_remote": parse_wireless_remote([0] * 40),
            "bms": {"version": {"high": 1, "low": 0}, "status_code": 1, "status_name": "discharge",
                    "soc_percent": 82, "current_ma": -1500, "cycle_count": 42,
                    "bq_ntc_c": [30, 31], "mcu_ntc_c": [33, 34], "cell_voltage_mv": [3900] * 10},
            "_offline": True,
        }
        self._diag["total_count"] += 1
        self._diag["accessible"] = False

    # ── 状态读取 ────────────────────────────────────────────────────────────────
    def get_high_state(self) -> dict:
        if not self._connected:
            return getattr(self, "_fake", None) or {"_offline": True}
        with self._lock:
            return snapshot_high(self._state)

    def get_low_state(self) -> dict:
        if not self._connected:
            f = getattr(self, "_fake", None) or {}
            fm = f.get("motors", [])
            motors = []
            for i in range(12):
                src = fm[i] if i < len(fm) else {}
                motors.append({"index": i, "name": JOINT_NAMES[i], "mode": src.get("mode", MOTOR_SERVO),
                               "q_rad": src.get("q_rad", 0.0), "dq_rad_s": src.get("dq_rad_s", 0.0),
                               "ddq_rad_s2": src.get("ddq_rad_s2", 0.0),
                               "tau_est_nm": src.get("tau_est_nm", 0.0),
                               "temperature_c": src.get("temperature_c", 34)})
            return {"motors": motors, "foot_force_raw": f.get("foot_force_raw", [0, 0, 0, 0]),
                    "imu": f.get("imu", {}), "bms": f.get("bms", {}),
                    "wireless_remote": f.get("wireless_remote", parse_wireless_remote([0] * 40)),
                    "_offline": True}
        with self._lock:
            return snapshot_low(self._state)

    def get_state(self) -> dict:
        """按 control_level 返回对应快照(状态卡通用)。"""
        return self.get_high_state() if self.control_level == "HIGHLEVEL" else self.get_low_state()

    def _accum_odo(self):
        """收发循环内调:按 position 水平增量累计总路程;单帧跳变(>0.5m)丢弃防污染。"""
        try:
            p = _flist(_get(self._state, "position", []), 3)
        except Exception:  # noqa: BLE001
            return
        if self._odo_last is not None:
            d = math.hypot(p[0] - self._odo_last[0], p[1] - self._odo_last[1])
            if d < 0.5:
                self._odometer_m += d
        self._odo_last = p

    def get_odometry(self) -> dict:
        """里程计快照:当前 position/yaw + 累计总路程(total_distance_m 由收发循环累加)。"""
        s = self.get_high_state()
        pos = s.get("position_m", [0.0, 0.0, 0.0])
        yaw = s.get("imu", {}).get("rpy_rad", [0.0, 0.0, 0.0])[2]
        return {"position_m": pos, "yaw_rad": round(float(yaw), 5),
                "total_distance_m": round(self._odometer_m, 3),
                "offline": bool(s.get("_offline"))}

    def get_diagnostics(self) -> dict:
        d = dict(self._diag)
        d["accessible"] = bool(d["accessible"]) and self._connected
        return d

    def capabilities(self) -> dict:
        """各状态卡的数据源是否可用(供状态卡诚实标注,避免把"无数据"伪装成真实值)。
        factory(工厂高层 .so):无 motorState/rangeObstacle/BmsState → joints/obstacle_range/battery 残缺。
        standard(标准 SDK):全字段可用。fake:合成数据,视作全可用。"""
        caps = {"loco_state": True, "imu": True, "feet": True, "remote_controller": True,
                "udp_diagnostics": True, "joints": True, "obstacle_range": True, "battery": True}
        if self._mode_backend == "factory":
            caps["joints"] = False        # 工厂 HighState 无 motorState
            caps["obstacle_range"] = False  # 工厂 HighState 无 rangeObstacle
            caps["battery"] = False       # 工厂 BmsState 未注册,读取抛异常
        return caps

    def backend_name(self) -> str:
        return self._mode_backend or "fake"

    def state_fresh(self) -> bool:
        """是否在超时内收到过有效状态包(NO_FEEDBACK 判定用)。"""
        if not self._connected:
            return True   # fake 恒有合成数据
        return bool(self._last_state_mono) and (time.monotonic() - self._last_state_mono) <= 0.5

    # ── HIGHLEVEL 期望设定(卡片调用;仅更新目标,由 loop 合成下发) ────────────
    def set_move(self, vx, vy, vyaw):
        with self._lock:
            self._hi.update(mode=MODE_WALK, velocity=[float(vx), float(vy)], yawSpeed=float(vyaw),
                            move_deadline=time.monotonic() + self._move_timeout)

    def stop_move(self):
        with self._lock:
            self._hi.update(mode=MODE_IDLE, velocity=[0.0, 0.0], yawSpeed=0.0)

    def set_simple_mode(self, mode):
        with self._lock:
            self._hi.update(mode=int(mode), velocity=[0.0, 0.0], yawSpeed=0.0)

    def set_gait(self, gait_type):
        with self._lock:
            self._hi["gaitType"] = int(gait_type)

    def current_gait(self) -> int:
        return int(self._hi["gaitType"])

    def set_speed_gear(self, gear) -> bool:
        """设速度档位 slow|normal|fast(speed_gear 卡)。未知档位返回 False。"""
        g = str(gear).lower()
        if g not in SPEED_GEARS:
            return False
        with self._lock:
            self._speed_gear = g
        return True

    def speed_gear(self) -> str:
        return self._speed_gear

    def gear_speed(self) -> dict:
        """当前档位对应的缺省线速/转速(闭环便捷动作用)。"""
        return dict(SPEED_GEARS[self._speed_gear])

    def is_moving(self) -> bool:
        """当前是否在运动(只读,不推进任何状态机):运动宏(walk/turn/perform)进行中、
        special 进行中,或未超看门狗的有效 move。非 WALK 模式速度已在合成时清零,故仅
        WALK+未超时算 move。供 power_control 静止前置 / robot_status 用。"""
        if self._macro_status.get("active"):   # walk_distance/turn_angle/perform 后台宏
            return True
        with self._lock:
            if self._special is not None:
                return True
            h = self._hi
            return h["mode"] == MODE_WALK and time.monotonic() <= h["move_deadline"]

    def is_standing(self) -> bool:
        """粗判是否站立态(供 robot_status/大模型判断能否运动):
        IDLE(默认站)/FORCE_STAND/WALK/RECOVERY 视作站立;stand_down/damp 视作未站立。"""
        m = int(self.get_high_state().get("mode", 0))
        return m in (MODE_IDLE, MODE_FORCE_STAND, MODE_WALK, MODE_RECOVERY)

    def set_attitude(self, roll, pitch, yaw):
        with self._lock:
            self._hi.update(mode=MODE_FORCE_STAND, euler=[float(roll), float(pitch), float(yaw)])

    def set_body_height(self, off):
        with self._lock:
            self._hi["bodyHeight"] = float(off)

    def set_foot_raise_height(self, off):
        with self._lock:
            self._hi["footRaiseHeight"] = float(off)

    def pose_reset(self):
        with self._lock:
            self._hi.update(euler=[0.0, 0.0, 0.0], bodyHeight=0.0, footRaiseHeight=0.0)

    def trigger_special(self, mode) -> int:
        with self._lock:
            self._seq_id += 1
            self._special = {"mode": int(mode), "phase": "stabilize",
                             "deadline": time.monotonic() + self._special_stabilize_s}
            return self._seq_id

    def special_state(self):
        with self._lock:
            if self._special is None:
                return {"active": False}
            return {"active": True, "phase": self._special["phase"], "mode": self._special["mode"]}

    # ── 闭环运动宏(供大模型下"走2米/转90度"单指令;异步后台线程,读里程计/yaw) ──────
    def cancel_macro(self):
        """打断正在跑的运动宏。任何手动运动指令/急停都应先调它。可跨线程调(防 join 自身)。"""
        self._macro_cancel.set()
        t = self._macro_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.5)

    def macro_status(self) -> dict:
        return dict(self._macro_status)

    def _start_macro(self, kind, target, loop_fn):
        self.cancel_macro()                       # 先取消上一个宏
        self._macro_cancel.clear()
        self._macro_status = {"active": True, "kind": kind, "target": round(float(target), 4), "progress": 0.0}

        def run():
            try:
                loop_fn()
            except Exception as e:  # noqa: BLE001
                print(f"[go1_ctrl] macro {kind} err: {e}", flush=True)
            finally:
                cancelled = self._macro_cancel.is_set()
                if not cancelled:
                    self.stop_move()              # 正常完成主动停;被取消则保留取消者(如 damp)的设定
                self._macro_status = {"active": False, "kind": kind, "target": round(float(target), 4),
                                      "progress": self._macro_status.get("progress", 0.0), "cancelled": cancelled}

        self._macro_thread = threading.Thread(target=run, daemon=True, name="go1_macro")
        self._macro_thread.start()

    def walk_distance(self, distance_m, speed_mps=0.3, max_time_s=20.0):
        """闭环前进/后退 distance_m(读里程计位移,走够即停)。异步:立即返回,后台执行。
        负值后退。speed 限 0.05~0.8。看门狗兜底(线程停发 move → 狗 0.5s 内自停)。"""
        target = abs(float(distance_m))
        sgn = 1.0 if distance_m >= 0 else -1.0
        sp = min(max(abs(float(speed_mps)), 0.05), 0.8) * sgn
        start = list(self.get_high_state().get("position_m", [0.0, 0.0, 0.0]))
        # 到达提前量:补偿里程上报延迟 + 停下滑行(实测 0.3m/s 无补偿过冲 ~0.15m,约目标的 30%)。
        lead = min(abs(sp) * 0.5, target * 0.5)

        def loop():
            self.set_gait(GAITS["trot"])
            t0 = time.monotonic()
            while not self._macro_cancel.is_set() and (time.monotonic() - t0) < max_time_s:
                pos = self.get_high_state().get("position_m", [0.0, 0.0, 0.0])
                travelled = math.hypot(pos[0] - start[0], pos[1] - start[1])
                self._macro_status["progress"] = round(travelled, 3)
                if travelled >= target - lead:
                    break
                self.set_move(sp, 0.0, 0.0)       # 持续刷新看门狗
                time.sleep(0.05)

        self._start_macro("walk_distance", target, loop)
        return {"started": True, "target_m": round(target, 3), "speed_mps": round(abs(sp), 3)}

    def turn_angle(self, angle_rad, yaw_rate=0.5, max_time_s=15.0):
        """闭环旋转 angle_rad(读 IMU yaw 累计,含 unwrap,转够即停)。异步。左正右负。
        yaw_rate 限 0.1~1.5。闭环读 yaw 用 signed 累计(噪声正负抵消,大角度不虚高提前停/少转)。"""
        target = abs(float(angle_rad))
        sgn = 1.0 if angle_rad >= 0 else -1.0
        rate = min(max(abs(float(yaw_rate)), 0.1), 1.5) * sgn

        def _yaw():
            return self.get_high_state().get("imu", {}).get("rpy_rad", [0.0, 0.0, 0.0])[2]

        def loop():
            self.set_gait(GAITS["trot"])
            prev = _yaw(); acc = 0.0; t0 = time.monotonic()
            while not self._macro_cancel.is_set() and (time.monotonic() - t0) < max_time_s:
                if abs(acc) >= target:
                    break
                self.set_move(0.0, 0.0, rate)
                time.sleep(0.05)
                cur = _yaw(); d = cur - prev
                if d > math.pi:
                    d -= 2 * math.pi
                elif d < -math.pi:
                    d += 2 * math.pi
                acc += d; prev = cur      # signed 累计:yaw 噪声正负抵消,不虚高(修大角度少转)
                self._macro_status["progress"] = round(abs(acc), 3)

        self._start_macro("turn_angle", target, loop)
        return {"started": True, "target_rad": round(target, 3), "yaw_rate": round(abs(rate), 3)}

    # ── 闭环导航(nav 卡:里程计世界系走到 (x,y);单后台槽、异步、可打断) ────────────
    def _turn_to_yaw(self, target_yaw, rate=0.5, tol=0.1, max_time_s=8.0):
        """闭环转到里程计绝对 yaw=target_yaw(在宏内跑,cancel 可打断)。逐帧读绝对 yaw 误差,
        进 tol 即停。用于 go_to 的终点朝向对齐。"""
        t0 = time.monotonic()
        while not self._macro_cancel.is_set() and (time.monotonic() - t0) < max_time_s:
            cur = self.get_odometry().get("yaw_rad", 0.0)
            err = _wrap_pi(target_yaw - cur)
            if abs(err) <= tol:
                return
            self.set_move(0.0, 0.0, (abs(rate) if err > 0 else -abs(rate)))
            time.sleep(0.05)

    def _goto_segment(self, x, y, speed, tol, final_yaw=None, max_time_s=30.0):
        """朝里程计世界坐标 (x,y) 走过去的比例控制器(在宏内同步跑,cancel 可打断)。
        朝向误差大 → 原地转对准;对准 → 带小转向分量前进;进 tol 米即到,
        可选到点后对齐 final_yaw。返回 True=到达 / False=被取消或超时。"""
        self.set_gait(GAITS["trot"])
        t0 = time.monotonic()
        reached = False
        while not self._macro_cancel.is_set() and (time.monotonic() - t0) < max_time_s:
            odo = self.get_odometry()
            pos = odo.get("position_m", [0.0, 0.0, 0.0])
            cur_yaw = odo.get("yaw_rad", 0.0)
            dx = x - pos[0]
            dy = y - pos[1]
            dist = math.hypot(dx, dy)
            self._macro_status["progress"] = round(dist, 3)   # 剩余距离(越小越近)
            if dist <= tol:
                reached = True
                break
            err = _wrap_pi(math.atan2(dy, dx) - cur_yaw)
            if abs(err) > 0.35:
                self.set_move(0.0, 0.0, 0.6 if err > 0 else -0.6)   # 原地转对准
            else:
                steer = max(-0.6, min(0.6, 1.5 * err))               # 带转向前进
                self.set_move(float(speed), 0.0, steer)
            time.sleep(0.05)
        if reached and final_yaw is not None and not self._macro_cancel.is_set():
            self._turn_to_yaw(final_yaw)
        return reached

    def nav_goto(self, x, y, yaw=None, speed=None, tol=None, max_time_s=30.0):
        """闭环走到里程计世界坐标 (x,y)(可选终点朝向 yaw)。异步:立即返回、后台执行、可打断。
        speed 缺省跟随 speed_gear;看门狗兜底(线程停发 move → 狗 0.5s 内自停)。"""
        tx, ty = float(x), float(y)
        fy = None if yaw is None else float(yaw)
        sp = self.gear_speed().get("linear_mps", 0.3) if speed is None else float(speed)
        sp = min(max(abs(sp), 0.05), 0.8)
        tl = 0.15 if tol is None else max(0.05, float(tol))

        def loop():
            self._goto_segment(tx, ty, sp, tl, final_yaw=fy, max_time_s=max_time_s)

        self._start_macro("nav_goto", math.hypot(tx, ty), loop)
        return {"started": True, "target_m": [round(tx, 3), round(ty, 3)],
                "final_yaw_rad": (round(fy, 3) if fy is not None else None),
                "speed_mps": round(sp, 3), "tol_m": round(tl, 3)}

    def patrol_points(self, points, loops=1, speed=None, tol=None, max_time_s=30.0):
        """依次巡逻一串航点(可多圈)。异步、单宏、可打断;任一段被取消/超时即终止整程。
        points: [{"x":float, "y":float, "yaw":float|None, "name":str}, ...]。"""
        pts = list(points or [])
        n = max(1, int(loops))
        sp = self.gear_speed().get("linear_mps", 0.3) if speed is None else float(speed)
        sp = min(max(abs(sp), 0.05), 0.8)
        tl = 0.2 if tol is None else max(0.05, float(tol))

        def loop():
            for _ in range(n):
                for p in pts:
                    if self._macro_cancel.is_set():
                        return
                    fy = p.get("yaw")
                    ok = self._goto_segment(float(p["x"]), float(p["y"]), sp, tl,
                                            final_yaw=(None if fy is None else float(fy)),
                                            max_time_s=max_time_s)
                    if not ok:
                        return

        self._start_macro("patrol", len(pts) * n, loop)
        return {"started": True, "points": [p.get("name") for p in pts],
                "count": len(pts), "loops": n, "speed_mps": round(sp, 3), "tol_m": round(tl, 3)}

    # ── 姿态时间线(表情/复合动作;供 perform 卡与大模型下"打招呼/点头/开心"这类指令) ──────
    def _sleep_cancellable(self, dur):
        """分片睡 dur 秒,期间可被 cancel_macro 打断;被打断返回 False,否则 True。"""
        end = time.monotonic() + max(0.0, float(dur))
        while time.monotonic() < end:
            if self._macro_cancel.is_set():
                return False
            time.sleep(0.02)
        return not self._macro_cancel.is_set()

    def play_routine(self, name, steps, settle_s=0.4):
        """播放一段"姿态时间线"复合动作。steps: 有序列表,每步
        {roll,pitch,yaw (rad), body_height,foot_raise (偏移 m), dur (秒)}。
        异步:立即返回,后台线程逐步 set_attitude/高度;与 walk/turn 宏共用**唯一后台槽**
        (同时只一个动作);任何手动运动指令/damp/perform stop 都能打断;结束自动归中性站好。
        ⚠️ 需狗已站立(软件无法从地面起立,靠遥控)。幅值内部再夹一层,防越硬件安全区。"""
        def _c(v, lo, hi):
            try:
                v = float(v)
            except (TypeError, ValueError):
                return 0.0
            return lo if v < lo else hi if v > hi else v

        steps = list(steps or [])

        def loop():
            for i, st in enumerate(steps):
                if self._macro_cancel.is_set():
                    return
                self.set_attitude(_c(st.get("roll", 0.0), -0.5, 0.5),
                                  _c(st.get("pitch", 0.0), -0.5, 0.5),
                                  _c(st.get("yaw", 0.0), -0.5, 0.5))
                if "body_height" in st:
                    self.set_body_height(_c(st["body_height"], -0.1, 0.03))
                if "foot_raise" in st:
                    self.set_foot_raise_height(_c(st["foot_raise"], -0.05, 0.03))
                self._macro_status["progress"] = i + 1
                self._sleep_cancellable(st.get("dur", settle_s))
            self.pose_reset()      # 归中性姿态;_start_macro 收尾 stop_move → IDLE 站好

        self._start_macro("perform:" + str(name), len(steps), loop)
        return {"started": True, "routine": name, "steps": len(steps)}

    # ── LOWLEVEL 期望设定 ──────────────────────────────────────────────────────
    def set_joint(self, idx, q=None, dq=None, kp=0.0, kd=0.0, tau=0.0, mode=MOTOR_SERVO):
        with self._lock:
            j = self._lo[idx]
            j.update(q=(POS_STOP_F if q is None else float(q)),
                     dq=(VEL_STOP_F if dq is None else float(dq)),
                     Kp=float(kp), Kd=float(kd), tau=float(tau), mode=int(mode))

    def set_joint_damping(self, idxs):
        with self._lock:
            for i in idxs:
                self._lo[i].update(q=POS_STOP_F, dq=VEL_STOP_F, Kp=0.0, Kd=0.0, tau=0.0, mode=MOTOR_DAMPING)

    def joint_feedback(self, idx) -> dict:
        st = self.get_low_state()
        for m in st.get("motors", []):
            if m.get("index") == idx:
                return {"q_rad": m.get("q_rad", 0.0), "dq_rad_s": m.get("dq_rad_s", 0.0),
                        "tau_est_nm": m.get("tau_est_nm", 0.0), "temperature_c": m.get("temperature_c", 0)}
        return {"q_rad": 0.0, "dq_rad_s": 0.0, "tau_est_nm": 0.0, "temperature_c": 0}

    # ── 电源 ────────────────────────────────────────────────────────────────────
    def request_power_off(self):
        with self._lock:
            self._power_off_pending = True
