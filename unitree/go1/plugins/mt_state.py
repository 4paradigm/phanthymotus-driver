from __future__ import annotations

#!/usr/bin/env python3
"""
plugins/mt_state.py — MT 8 张状态卡:
  loco_state / imu / joints / feet / obstacle_range / remote_controller / battery / udp_diagnostics

全部只读,数据取自 go1_ctrl(唯一硬件入口)的快照;按 MT §6 的字段命名与单位。
"""
import json
import math
import os

from go1_ctrl import MODE_NAMES, GAIT_NAMES, GAITS, JOINT_NAMES, FEET_ORDER  # noqa: F401
from plugins.mt_base import MtStateCard, now_ms  # noqa: F401

# lowstate_reader(低层 SDK 连主控板 192.168.123.10)写的共享文件 —— battery/joints 从这取真数据。
_LOWSTATE_FILE_DEFAULT = "/tmp/go1_lowstate.json"
_LOWSTATE_MAX_AGE_S = 5.0


def _read_lowstate(path):
    """读 lowstate_reader 写的共享文件;不存在/过期/损坏 → None(卡片据此标 available:false)。"""
    try:
        age = (now_ms() / 1000.0) - os.path.getmtime(path)
        if age > _LOWSTATE_MAX_AGE_S:
            return None
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class LocoStateCard(MtStateCard):
    CARD = "loco_state"; CONTROL_LEVEL = "HIGHLEVEL"
    TOPIC = "loco/state"; FMT = "data/json"; HZ = 10.0
    DESC = "Go1 运动状态 —— 模式/步态/里程/速度/机身高度。10Hz。"

    def _payload(self):
        s = self._ctrl.get_high_state()
        vel = s.get("velocity", [0.0, 0.0, 0.0])
        return {
            "mode": s.get("mode", 0), "mode_name": MODE_NAMES.get(s.get("mode", 0), "unknown"),
            "gait_type": s.get("gait_type", 0), "gait_name": GAIT_NAMES.get(s.get("gait_type", 0), "unknown"),
            "foot_raise_height_m": s.get("foot_raise_height_m", 0.0),
            "position_m": s.get("position_m", [0.0, 0.0, 0.0]),
            "body_height_m": s.get("body_height_m", 0.0),
            "velocity_body_mps": {"forward": vel[0] if len(vel) > 0 else 0.0,
                                  "lateral": vel[1] if len(vel) > 1 else 0.0},
            # 官方 velocity[2] 语义与独立 yawSpeed 冲突 → 只保留原始值,不命名(MT §6.1)
            "velocity_index_2_raw": vel[2] if len(vel) > 2 else 0.0,
            "yaw_speed_rad_s": s.get("yaw_speed_rad_s", 0.0),
        }


class ImuCard(MtStateCard):
    CARD = "imu"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/imu"; FMT = "data/json"; HZ = 20.0
    DESC = "Go1 IMU —— 四元数(wxyz)/角速度/加速度/欧拉角/温度。20Hz。"

    def _payload(self):
        imu = self._ctrl.get_state().get("imu", {})
        d = dict(imu)
        d["attitude_may_drift"] = True   # 加速运动时姿态漂移,提示上层(MT §6.2)
        return d


class JointsCard(MtStateCard):
    CARD = "joints"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/joints"; FMT = "sensor/skeleton"; HZ = 5.0
    DESC = ("Go1 12 关节 —— 角度/角速度/力矩/温度。经**低层 SDK**(连主控板 192.168.123.10)读 "
            "LowState.motorState,由 lowstate_reader 供数(高层 factory 绑定读不到关节)。")

    def __init__(self, cfg, namespace, bridge, ctrl):
        super().__init__(cfg, namespace, bridge, ctrl)
        self._file = self._cfg.get("lowstate_file", _LOWSTATE_FILE_DEFAULT)

    def _produce(self):
        # 读低层共享文件(不受 factory 高层通道新鲜度抑制;与运动不冲突)
        ls = _read_lowstate(self._file)
        if ls and ls.get("joints"):
            d = {"available": True, "joints": ls["joints"], "source": "lowlevel_sdk"}
        else:
            d = {"available": False, "joints": [],
                 "reason": "无低层数据(需 lowstate_reader 连主控板 192.168.123.10 供数)"}
        d["timestamp_ms"] = now_ms()
        d["control_level"] = self._ctrl.control_level
        return d


class FeetCard(MtStateCard):
    CARD = "feet"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/feet"; FMT = "data/json"; HZ = 10.0
    DESC = "Go1 足端 —— 足底力原始值(高层控制时另有足端相对机身的位置/速度)。"

    def _payload(self):
        s = self._ctrl.get_state()
        out = {"order": FEET_ORDER, "foot_force_raw": s.get("foot_force_raw", [0, 0, 0, 0])}
        # 足端相对机身位置/速度仅高层提供(MT §6.4)
        if "foot_position_to_body" in s:
            out["position_to_body"] = s["foot_position_to_body"]
            out["speed_to_body"] = s.get("foot_speed_to_body", [])
        return out


class ObstacleRangeCard(MtStateCard):
    CARD = "obstacle_range"; CONTROL_LEVEL = "HIGHLEVEL"
    TOPIC = "state/obstacle_range"; FMT = "data/json"; HZ = 10.0
    DESC = "Go1 最近障碍距离原始值[4](方向/单位官方未定义 —— 仅只读状态,非避障)。"

    def _payload(self):
        # 工厂高层 .so 无 rangeObstacle 字段 → 诚实标注,不输出误导性的 [0,0,0,0]
        if not self._ctrl.capabilities().get("obstacle_range", True):
            return {"available": False, "range_raw": None,
                    "reason": "高层 .so 无 rangeObstacle 字段;需标准 SDK"}
        s = self._ctrl.get_high_state()
        # 官方未定义四项方向顺序与单位 → 原样输出,不命名前后左右(MT §6.5)
        return {"range_raw": s.get("range_obstacle_raw", [0.0, 0.0, 0.0, 0.0]),
                "direction_mapping": "undocumented", "unit": "undocumented", "available": True}


class RemoteControllerCard(MtStateCard):
    CARD = "remote_controller"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/remote_controller"; FMT = "data/json"; HZ = 10.0
    DESC = "Go1 无线遥控器 —— 16 个按键 + 5 个摇杆轴(来自 wirelessRemote[40])。"

    def _payload(self):
        wr = self._ctrl.get_state().get("wireless_remote", {})
        return {"buttons": wr.get("buttons", {}), "axes": wr.get("axes", {})}


class BatteryCard(MtStateCard):
    CARD = "battery"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/battery"; FMT = "data/json"; HZ = 1.0
    DESC = ("Go1 电池 —— SOC 电量%/状态/电流/温度/循环次数。经**低层 SDK**(连主控板 192.168.123.10)读 "
            "LowState.bms,由 lowstate_reader 供数(高层 factory 绑定读不到电量)。")

    def __init__(self, cfg, namespace, bridge, ctrl):
        super().__init__(cfg, namespace, bridge, ctrl)
        self._file = self._cfg.get("lowstate_file", _LOWSTATE_FILE_DEFAULT)

    def _produce(self):
        ls = _read_lowstate(self._file)
        if ls and ls.get("bms"):
            b = ls["bms"]
            d = {"available": True, "soc_percent": b.get("soc"), "status": b.get("status"),
                 "current_a": b.get("current_a"), "cycle": b.get("cycle"),
                 "temperatures_c": b.get("temps_c"), "source": "lowlevel_sdk"}
        else:
            d = {"available": False, "soc_percent": None,
                 "reason": "无低层数据(需 lowstate_reader 连主控板 192.168.123.10 供数)"}
        d["timestamp_ms"] = now_ms()
        d["control_level"] = self._ctrl.control_level
        return d


class UdpDiagnosticsCard(MtStateCard):
    CARD = "udp_diagnostics"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/udp_diagnostics"; FMT = "data/json"; HZ = 2.0
    DESC = "Go1 驱动 UDP 链路健康 —— 发送/接收/错误/CRC/丢包计数 + 可达性。"

    def _payload(self):
        return dict(self._ctrl.get_diagnostics())


class RobotStatusCard(MtStateCard):
    """驱动增值卡(非 MT 8 卡之一):把散在多张状态卡里的信息汇成一条给大模型的态势快照,
    让它一次调用就知道:后端/链路、模式/步态、站没站、在不在动、位置、正在跑的宏进度、
    电量、以及能不能立刻接受运动指令(ready_for_motion + 人话 hint)。只读、安全。"""
    CARD = "robot_status"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/robot_status"; FMT = "data/json"; HZ = 2.0
    DESC = ("Go1 一键态势快照,供高层规划一次调用即掌握:后端/链路、模式/步态、站立/移动状态、"
            "位置、正在运行的宏(walk_distance/turn_angle/perform)、电量(若可用),以及 ready_for_motion"
            "(能否立即运动)+ 一句人话提示下一步该做什么。")

    def _payload(self):
        s = self._ctrl.get_high_state()
        mode = int(s.get("mode", 0))
        macro = self._ctrl.macro_status()
        moving = self._ctrl.is_moving()
        standing = self._ctrl.is_standing()
        link_ok = self._ctrl.is_connected() and self._ctrl.state_fresh()
        batt = {"available": False, "soc_percent": None}
        if self._ctrl.capabilities().get("battery", False):
            b = s.get("bms", {})
            if b:
                batt = {"available": True, "soc_percent": b.get("soc_percent")}
        ready = bool(standing and not moving and link_ok)
        if not link_ok:
            hint = "无新状态/离线 —— 请检查驱动与 Legged_sport 通道"
        elif not standing:
            hint = "狗未站立 —— 运动前需人用遥控器把它扶起来"
        elif moving:
            hint = "忙碌 —— 有运动/宏正在执行(用 perform 的 stop 或 loco 的 stop_move 打断)"
        else:
            hint = "就绪 —— 现在可以调用 loco / perform"
        return {
            "backend": self._ctrl.backend_name(), "offline": bool(s.get("_offline")),
            "link_ok": link_ok, "mode": mode, "mode_name": MODE_NAMES.get(mode, "unknown"),
            "gait_type": s.get("gait_type", 0),
            "gait_name": GAIT_NAMES.get(s.get("gait_type", 0), "unknown"),
            "standing": standing, "moving": moving,
            "position_m": s.get("position_m", [0.0, 0.0, 0.0]),
            "body_height_m": s.get("body_height_m", 0.0),
            "macro": (macro if macro.get("active") else None),
            "battery": batt, "ready_for_motion": ready, "hint": hint,
        }


class FallAlarmCard(MtStateCard):
    """跌倒/侧翻告警(驱动增值卡,非 MT 14 卡之一):由 IMU 的 roll/pitch 幅度判定
    ok/tilted/fallen,给运动安全兜底。阈值可 config(tilt_warn_rad/fall_rad)。
    依赖 IMU 帧,连接后无新帧则按 NO_FEEDBACK 抑制(不误报)。"""
    CARD = "fall_alarm"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/fall_alarm"; FMT = "data/json"; HZ = 10.0
    DESC = "Go1 跌倒/侧翻告警 —— 由 IMU 的 roll/pitch 幅度判定 ok/tilted/fallen + 倾角。10Hz。"

    def _payload(self):
        imu = self._ctrl.get_state().get("imu", {})
        rpy = imu.get("rpy_rad", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
        roll = float(rpy[0]) if len(rpy) > 0 else 0.0
        pitch = float(rpy[1]) if len(rpy) > 1 else 0.0
        warn = float(self._cfg.get("tilt_warn_rad", 0.6))   # ≈34°
        fall = float(self._cfg.get("fall_rad", 1.2))        # ≈69°
        tilt = max(abs(roll), abs(pitch))
        if tilt >= fall:
            status, hint = "fallen", "已跌倒/翻倒 —— 立即停止运动,需人工扶正后再操作"
        elif tilt >= warn:
            status, hint = "tilted", "机身明显倾斜 —— 谨慎,可能即将失稳"
        else:
            status, hint = "ok", "姿态正常"
        return {"status": status, "roll_rad": round(roll, 4), "pitch_rad": round(pitch, 4),
                "roll_deg": round(math.degrees(roll), 1), "pitch_deg": round(math.degrees(pitch), 1),
                "tilt_warn_rad": warn, "fall_rad": fall, "hint": hint}


class NetCard(MtStateCard):
    """网络健康(驱动增值卡):主机名/IPv4/Wi-Fi 信号。联调时无线易掉,这张卡让大模型/人
    一眼看清链路。纯系统读(不经硬件)→ 不套硬件状态新鲜度抑制,始终可读。"""
    CARD = "net"; CONTROL_LEVEL = "ANY"
    TOPIC = "state/net"; FMT = "data/json"; HZ = 0.5
    DESC = "Go1 网络健康 —— 主机名/IPv4/Wi-Fi 信号强度(联调掉线排查用)。"

    def _produce(self):
        # 系统级读取,不依赖硬件帧 → 不套 MtStateCard 的 fresh 抑制
        d = self._payload()
        d["timestamp_ms"] = now_ms()
        d["control_level"] = self._ctrl.control_level
        return d

    def _payload(self):
        import socket
        out = {"available": True, "hostname": None, "ipv4": None, "wifi": {"available": False}}
        try:
            out["hostname"] = socket.gethostname()
        except Exception:  # noqa: BLE001
            pass
        out["ipv4"] = self._primary_ipv4()
        out["wifi"] = self._wifi_stats()
        return out

    @staticmethod
    def _primary_ipv4():
        import socket
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))    # 不真发包,只用于选默认出口网卡
            return s.getsockname()[0]
        except Exception:  # noqa: BLE001
            return None
        finally:
            if s is not None:
                try:
                    s.close()
                except Exception:  # noqa: BLE001
                    pass

    @staticmethod
    def _wifi_stats():
        # Linux(狗)读 /proc/net/wireless;Mac/无该文件 → available:false(诚实标注)
        try:
            with open("/proc/net/wireless") as f:
                lines = f.read().splitlines()
        except Exception:  # noqa: BLE001
            return {"available": False, "reason": "无 /proc/net/wireless(非 Linux 或无无线网卡)"}
        for ln in lines[2:]:
            parts = ln.split()
            if len(parts) >= 4 and parts[0].endswith(":"):
                iface = parts[0].rstrip(":")
                try:
                    link = float(parts[2].rstrip("."))
                    level = float(parts[3].rstrip("."))
                except (ValueError, IndexError):
                    link, level = None, None
                return {"available": True, "iface": iface,
                        "link_quality": link, "signal_dbm": level}
        return {"available": False, "reason": "无活动无线网卡"}


class OdometryCard(MtStateCard):
    """里程计(驱动增值卡):当前 position/yaw + 累计总路程 + 相对起点位移。
    起点默认取首帧,可用 reset_origin 动作重置。仅高层有 position(HIGHLEVEL)。"""
    CARD = "odometry"; CONTROL_LEVEL = "HIGHLEVEL"
    TOPIC = "state/odometry"; FMT = "data/json"; HZ = 5.0
    DESC = ("Go1 里程计 —— 当前 position/yaw、累计总路程 total_distance_m、相对起点位移 displacement"
            "(read 读取;reset_origin 重置起点)。让高层规划知道'走了多远/在哪'。")

    def __init__(self, cfg, namespace, bridge, ctrl):
        super().__init__(cfg, namespace, bridge, ctrl)
        self._origin = None    # [x, y] 起点;首次读或 reset_origin 时设

    # get_tool 用基类(只读、无执行按钮,与其它状态卡一致;reset_origin 仍可经 MCP 调用)

    def _payload(self):
        odo = self._ctrl.get_odometry()
        pos = odo.get("position_m", [0.0, 0.0, 0.0])
        if self._origin is None:
            self._origin = [pos[0], pos[1]]
        dx = pos[0] - self._origin[0]
        dy = pos[1] - self._origin[1]
        return {"position_m": pos, "yaw_rad": odo.get("yaw_rad", 0.0),
                "total_distance_m": odo.get("total_distance_m", 0.0),
                "origin_m": list(self._origin),
                "displacement_m": {"dx": round(dx, 3), "dy": round(dy, 3),
                                   "distance": round(math.hypot(dx, dy), 3)},
                "offline": odo.get("offline", False)}

    def dispatch(self, action, args):
        if action == "reset_origin":
            pos = self._ctrl.get_odometry().get("position_m", [0.0, 0.0, 0.0])
            self._origin = [pos[0], pos[1]]
            return self._ok("reset_origin", {"origin_m": list(self._origin)})
        return super().dispatch(action, args)
