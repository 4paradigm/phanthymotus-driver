#!/usr/bin/env python3
"""
go1_hl.py — Unitree Go1 高层控制客户端(双后端)。

对外语义 API 不变(move/stop_move/stand_up/stand_down/damp/balance_stand/euler/body_height/
get_state + 运动仲裁 acquire/release/motion_owner)。内部把"目标"交给当前后端翻译成各自 SDK 调用。

两个后端(config: robot.backend = auto|programming|legged_sdk;auto 依次尝试):
  • programming  : robot_interface_high_level(Go1 "programming" pybind,**py3.7**,Pi 直跑,已实机验证)。
                   HighState 无 motorState、BmsState 取不到 → joints 空、电量 None。
  • legged_sdk   : 标准 unitree_legged_sdk 的 UDP HighCmd/HighState(**可在 py3.10 容器重编**)。
                   HighState 带 motorState(关节角)、rangeObstacle(超声避障)、bms(电量)→ 状态更全。
                   ★ 这是"卡片"(容器 py3.10)能真正运动 + 拿全状态的关键后端。

⚠️ legged_sdk 后端按标准 unitree_legged_sdk v3.8.0 API 编写,**需连狗后对着实机 SDK 版本核对**
   (HighCmd 字段名、mode 语义、python 绑定模块名 robot_interface、UDP 端口)。已标 VERIFY。
"""
from __future__ import annotations

import importlib
import json
import math
import select
import subprocess
import sys
import threading
import time


# ══════════════════════════════════════════════════════════════════════════════
#  后端 A:programming(robot_interface_high_level,Pi py3.7,已验证)
# ══════════════════════════════════════════════════════════════════════════════

class _ProgrammingBackend:
    name = "programming"

    def __init__(self, cfg):
        self._build_path = cfg.get("sdk_build_path", "/home/pi/Unitree/autostart/programming/build")
        self._module = cfg.get("sdk_module", "robot_interface_high_level")
        m = cfg.get("modes", {}) or {}
        self.M_IDLE = int(m.get("idle", 0)); self.M_STAND = int(m.get("stand", 1)); self.M_WALK = int(m.get("walk", 2))
        # 从趴/阻尼态"起立"必须用 position-stand-up(mode 6);mode 1 只在已站立时力控调姿。
        # position-stand-down = mode 5;damping(急停软腿)= mode 7。实机 2026-07-10 验证。
        self.M_STANDUP = int(m.get("stand_up", 6)); self.M_STANDDOWN = int(m.get("stand_down", 5)); self.M_DAMP = int(m.get("damp", 7))
        self.G_TROT = int((cfg.get("gaits", {}) or {}).get("trot", 1))
        self._robot = None

    def connect(self):
        if self._build_path and self._build_path not in sys.path:
            sys.path.append(self._build_path)
        mod = importlib.import_module(self._module)
        self._robot = mod.RobotInterface()

    def step(self, t) -> dict:
        mode, gait = self.M_IDLE, 0
        roll = pitch = yaw = fwd = side = rot = bh = 0.0
        a = t["action"]
        if a == "walk":
            mode, gait = self.M_WALK, self.G_TROT; fwd, side, rot = t["vx"], t["vy"], t["vyaw"]
        elif a in ("stand", "balance"):
            mode = self.M_STAND
        elif a == "stand_up":
            mode = self.M_STANDUP          # 6 = position stand up(从趴/阻尼起立)
        elif a == "stand_down":
            mode = self.M_STANDDOWN        # 5 = position stand down(趴下)
        elif a == "euler":
            mode = self.M_STAND; roll, pitch, yaw = t["roll"], t["pitch"], t["yaw"]
        elif a == "body_height":
            mode = self.M_STAND; bh = t["height"]
        elif a == "damp":
            mode = self.M_DAMP             # 7 = damping(急停软腿)
        else:  # idle
            mode = self.M_IDLE
        self._robot.robotControl(mode, gait, 0, 0.0, bh, roll, pitch, yaw, fwd, side, rot)
        self._robot.UDPSend()
        self._robot.UDPRecv()
        return _normalize(self._robot.getState(), with_motors=False)


# ══════════════════════════════════════════════════════════════════════════════
#  后端 B:legged_sdk(标准 unitree_legged_sdk,UDP HighCmd,容器 py3.10)
# ══════════════════════════════════════════════════════════════════════════════

class _LeggedSdkBackend:
    name = "legged_sdk"
    HIGHLEVEL = 0xee

    def __init__(self, cfg):
        self._module = cfg.get("legged_sdk_module", "robot_interface")   # unitree_legged_sdk 的 py 绑定
        self._ip = cfg.get("robot_ip", "192.168.123.161")
        self._local_port = int(cfg.get("local_port", 8080))
        self._target_port = int(cfg.get("target_port", 8082))
        self._sdk = None; self._udp = None; self._cmd = None; self._hs = None

    def connect(self):
        self._sdk = importlib.import_module(self._module)          # VERIFY: 绑定模块名
        self._udp = self._sdk.UDP(self.HIGHLEVEL, self._local_port, self._ip, self._target_port)
        self._cmd = self._sdk.HighCmd()
        self._hs = self._sdk.HighState()
        self._udp.InitCmdData(self._cmd)

    def step(self, t) -> dict:
        # 先收状态
        self._udp.Recv()
        self._udp.GetRecv(self._hs)
        # 目标 → HighCmd(VERIFY: 字段名/ mode 语义按实机 SDK 核对)
        c = self._cmd
        c.mode = 0; c.gaitType = 0; c.speedLevel = 0
        c.footRaiseHeight = 0.0; c.bodyHeight = 0.0
        c.euler = [0.0, 0.0, 0.0]; c.velocity = [0.0, 0.0]; c.yawSpeed = 0.0
        a = t["action"]
        if a == "walk":
            c.mode = 2; c.gaitType = 1; c.velocity = [float(t["vx"]), float(t["vy"])]; c.yawSpeed = float(t["vyaw"])
        elif a in ("stand", "balance"):
            c.mode = 1
        elif a == "stand_up":
            c.mode = 1; c.bodyHeight = 0.1
        elif a == "stand_down":
            c.mode = 1; c.bodyHeight = -0.1
        elif a == "euler":
            c.mode = 1; c.euler = [float(t["roll"]), float(t["pitch"]), float(t["yaw"])]
        elif a == "body_height":
            c.mode = 1; c.bodyHeight = float(t["height"])
        elif a == "damp":
            c.mode = 7    # VERIFY: 标准 SDK 阻尼模式
        else:
            c.mode = 0    # idle / default stand
        self._udp.SetSend(c)
        self._udp.Send()
        return _normalize(self._hs, with_motors=True)


# ══════════════════════════════════════════════════════════════════════════════
#  后端 C:subproc(py3.7 子进程跑 programming .so,给 py3.10 卡片容器用)
# ══════════════════════════════════════════════════════════════════════════════
# 卡片容器是 py3.10,import 不了工厂 py3.7 的 robot_interface_high_level.so;而标准 UDP
# 后端实测被这条狗的 Legged_sport 静默丢弃(Recv 恒 -1,协议/版本不匹配)。所以容器运动
# = 把"能驱动这条狗的那个 py3.7 .so"隔离到 motion_worker.py 子进程,主进程经 stdin/stdout
# 同步 lockstep 下发目标/读状态(类比 go2 rpc_proxy 的子进程隔离,这里隔离的是 python 版本)。
# 用户 2026-07-10 拍板走此路。

class _SubprocBackend:
    name = "subproc"

    def __init__(self, cfg):
        from pathlib import Path
        self._py = cfg.get("py37_bin", "python3.7")
        self._worker = cfg.get("worker_path", str(Path(__file__).parent / "motion_worker.py"))
        self._build_path = cfg.get("sdk_build_path", "/home/pi/Unitree/autostart/programming/build")
        self._module = cfg.get("sdk_module", "robot_interface_high_level")
        m = cfg.get("modes", {}) or {}
        g = cfg.get("gaits", {}) or {}
        # mode 语义与 _ProgrammingBackend 一致,整包传给 worker(worker 负责 action→mode 映射)
        self._modes = {"idle": int(m.get("idle", 0)), "stand": int(m.get("stand", 1)),
                       "walk": int(m.get("walk", 2)), "stand_up": int(m.get("stand_up", 6)),
                       "stand_down": int(m.get("stand_down", 5)), "damp": int(m.get("damp", 7)),
                       "trot": int(g.get("trot", 1))}
        # 容器里:.so 是 buster 编的,依赖 liblcm/glib 等——给 worker 单独的 LD_LIBRARY_PATH
        # (挂载的宿主库),隔离到子进程,不污染 py3.10 主进程。宿主直跑时留空即可。
        self._ld = cfg.get("worker_ld_path", "")
        self._proc = None

    def connect(self):
        import os
        args = [self._py, self._worker, self._build_path, self._module, json.dumps(self._modes)]
        env = dict(os.environ)
        if self._ld:
            prev = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = self._ld + ((":" + prev) if prev else "")
        # stderr 直通父进程 → driver.log;stdout 只走状态 JSON
        self._proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=None, bufsize=1, universal_newlines=True, env=env)
        line = self._read_line(15.0)                 # 等 worker 的 READY 握手
        if line is None:
            self._kill(); raise RuntimeError("motion_worker 15s 内无响应(py3.7? .so?)")
        if "READY" not in line:
            self._kill(); raise RuntimeError("motion_worker 未就绪: %s" % line.strip())

    def _read_line(self, timeout):
        """带超时读一行 stdout;None = 超时或 worker 已退出(EOF)。"""
        try:
            r, _, _ = select.select([self._proc.stdout], [], [], timeout)
        except Exception:  # noqa: BLE001
            return None
        if not r:
            return None
        line = self._proc.stdout.readline()
        return None if line == "" else line

    def step(self, t) -> dict:
        p = self._proc
        if p is None or p.poll() is not None:
            raise RuntimeError("motion_worker 已退出")
        p.stdin.write(json.dumps(t) + "\n"); p.stdin.flush()
        line = self._read_line(2.0)
        if line is None:
            raise RuntimeError("motion_worker 读状态超时/退出")
        try:
            return json.loads(line)
        except Exception:  # noqa: BLE001
            return {}

    def _kill(self):
        try:
            if self._proc:
                self._proc.kill()
        except Exception:  # noqa: BLE001
            pass

    def close(self):
        try:
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.stdin.close()
                except Exception:  # noqa: BLE001
                    pass
                self._proc.terminate()
                self._proc.wait(timeout=2)
        except Exception:  # noqa: BLE001
            self._kill()


# ══════════════════════════════════════════════════════════════════════════════
#  HighState → dict(两后端共用;逐字段防御)
# ══════════════════════════════════════════════════════════════════════════════

def _normalize(s, with_motors: bool) -> dict:
    def _list(v, n):
        try:
            return [round(float(x), 5) for x in list(v)[:n]]
        except Exception:  # noqa: BLE001
            return [0.0] * n

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

    imu = _get(s, "imu")
    imu_d = {
        "quaternion":    _list(_get(imu, "quaternion", []), 4) if imu is not None else [1, 0, 0, 0],
        "gyroscope":     _list(_get(imu, "gyroscope", []), 3) if imu is not None else [0, 0, 0],
        "accelerometer": _list(_get(imu, "accelerometer", []), 3) if imu is not None else [0, 0, 0],
        "rpy":           _list(_get(imu, "rpy", []), 3) if imu is not None else [0, 0, 0],
        "temperature":   int(_num(imu, "temperature", 0)) if imu is not None else 0,
    }

    # 电量:bms.SOC(programming 的 BmsState 取不到 → None;legged_sdk 一般可取)
    soc = None
    try:
        bms = getattr(s, "bms")
        soc = int(getattr(bms, "SOC", getattr(bms, "soc", 0)))
    except Exception:  # noqa: BLE001
        soc = None

    try:
        foot = [int(x) for x in list(_get(s, "footForce", []) or [])[:4]] or [0, 0, 0, 0]
    except Exception:  # noqa: BLE001
        foot = [0, 0, 0, 0]

    # 关节:仅 legged_sdk 有 motorState(前 12 个为腿部电机);programming 无 → 空
    motors = []
    if with_motors:
        try:
            ms = _get(s, "motorState", []) or []
            for i in range(min(12, len(ms))):
                m = ms[i]
                motors.append({"q": round(_num(m, "q"), 5), "dq": round(_num(m, "dq"), 5),
                               "tau": round(_num(m, "tauEst", _num(m, "tau")), 4),
                               "temperature": int(_num(m, "temperature", 0))})
        except Exception:  # noqa: BLE001
            motors = []

    # 超声避障距离:仅 legged_sdk 有 rangeObstacle[4](m)
    obstacles = None
    if with_motors:
        r = _list(_get(s, "rangeObstacle", []), 4)
        if any(r):
            obstacles = {"front": r[0], "left": r[1], "right": r[2], "rear": r[3]}

    out = {
        "mode":        int(_num(s, "mode", 0)),
        "gait":        int(_num(s, "gaitType", 0)),
        "body_height": round(_num(s, "bodyHeight", 0), 4),
        "position":    _list(_get(s, "position", []), 3),
        "velocity":    _list(_get(s, "velocity", []), 3),
        "yaw_speed":   round(_num(s, "yawSpeed", 0), 4),
        "foot_force":  foot,
        "imu":         imu_d,
        "battery_soc": soc,
        "motors":      motors,
    }
    if obstacles is not None:
        out["obstacles_m"] = obstacles
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  Go1HighLevel:对外唯一入口(公共 API 不变)
# ══════════════════════════════════════════════════════════════════════════════

class Go1HighLevel:
    def __init__(self, robot_cfg: dict):
        self._cfg = robot_cfg or {}
        self._backend_pref = (self._cfg.get("backend", "auto") or "auto").lower()
        self._rate_hz = float(self._cfg.get("rate_hz", 100))
        self._move_timeout = float(self._cfg.get("move_timeout", 1.0))

        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._thread: threading.Thread | None = None
        self._backend = None
        self._state = None          # 最新 normalized dict
        self._motion_owner = None   # None | "avoid" | "nav"(运动仲裁)

        # 语义目标(后端无关)
        self._t = {"action": "idle", "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "height": 0.0, "move_deadline": 0.0}

    # ── 生命周期 ────────────────────────────────────────────────────────────
    def start(self) -> None:
        order = {"auto": ["programming", "subproc", "legged_sdk"],
                 "programming": ["programming"],
                 "subproc": ["subproc"],
                 "legged_sdk": ["legged_sdk"]}.get(self._backend_pref, ["programming", "subproc", "legged_sdk"])
        for name in order:
            try:
                if name == "programming":
                    b = _ProgrammingBackend(self._cfg)
                elif name == "subproc":
                    b = _SubprocBackend(self._cfg)
                else:
                    b = _LeggedSdkBackend(self._cfg)
                b.connect()
                self._backend = b; self._connected = True
                print(f"[go1_hl] ✓ backend={name} 就绪", flush=True)
                break
            except Exception as e:  # noqa: BLE001
                print(f"[go1_hl] backend '{name}' 不可用: {e}", flush=True)
        if not self._connected:
            print("[go1_hl] ⚠️  无可用后端 → 离线模式:MCP 照常起,控制/状态为合成数据。", flush=True)

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="go1_hl")
        self._thread.start()

    def stop(self) -> None:
        try:
            self.stop_move(); time.sleep(0.1)
        except Exception:  # noqa: BLE001
            pass
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        # 关闭后端子进程(subproc 后端)
        b = self._backend
        if b is not None and hasattr(b, "close"):
            try:
                b.close()
            except Exception:  # noqa: BLE001
                pass

    # ── 后台收发循环(唯一调 SDK 的地方) ──────────────────────────────────
    def _loop(self) -> None:
        dt = 1.0 / self._rate_hz
        while self._running:
            t0 = time.monotonic()
            if self._connected and self._backend is not None:
                try:
                    with self._lock:
                        t = dict(self._t)
                    # move 看门狗:行走超时 → 转为站立(防失控续跑)
                    if t["action"] == "walk" and time.monotonic() > t["move_deadline"]:
                        t["action"] = "stand"; t["vx"] = t["vy"] = t["vyaw"] = 0.0
                    st = self._backend.step(t)
                    with self._lock:
                        self._state = st
                except Exception as e:  # noqa: BLE001
                    print(f"[go1_hl] loop error: {e}", flush=True)
            sleep = dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    # ── 运动仲裁 ──────────────────────────────────────────────────────────
    def is_connected(self) -> bool:
        return self._connected

    def acquire_motion(self, owner: str) -> bool:
        with self._lock:
            if self._motion_owner in (None, owner):
                self._motion_owner = owner
                return True
            return False

    def release_motion(self, owner: str) -> None:
        with self._lock:
            if self._motion_owner == owner:
                self._motion_owner = None

    def motion_owner(self) -> str | None:
        with self._lock:
            return self._motion_owner

    # ── 语义 API(只改目标) ──────────────────────────────────────────────
    def _set(self, action, **kw) -> int:
        base = {"action": action, "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "height": 0.0}
        base.update(kw)
        with self._lock:
            self._t.update(base)
            if action != "walk":
                self._t["move_deadline"] = 0.0
        return 0 if self._connected else 3104

    def move(self, vx: float, vy: float, vyaw: float) -> int:
        return self._set("walk", vx=vx, vy=vy, vyaw=vyaw,
                         move_deadline=time.monotonic() + self._move_timeout)

    def stop_move(self) -> int:
        return self._set("stand")

    def stand_up(self) -> int:
        return self._set("stand_up")

    def stand_down(self) -> int:
        return self._set("stand_down")

    def damp(self) -> int:
        return self._set("damp")

    def balance_stand(self) -> int:
        return self._set("stand")

    def euler(self, roll: float, pitch: float, yaw: float) -> int:
        return self._set("euler", roll=roll, pitch=pitch, yaw=yaw)

    def body_height(self, h: float) -> int:
        return self._set("body_height", height=h)

    # ── 状态读取 ──────────────────────────────────────────────────────────
    def get_state(self) -> dict | None:
        if not self._connected:
            return self._fake_state()
        with self._lock:
            return self._state

    def _fake_state(self) -> dict:
        """离线(无后端)时的合成状态。忠实保留高层无关节角(motors 空)。"""
        t = time.monotonic()
        wobble = 0.02 * math.sin(t)
        return {
            "mode": 1, "gait": 0, "body_height": 0.0,
            "position": [0.0, 0.0, 0.0], "velocity": [0.0, 0.0, 0.0], "yaw_speed": 0.0,
            "foot_force": [90, 90, 90, 90],
            "imu": {"quaternion": [1.0, 0.0, 0.0, round(wobble, 4)],
                    "gyroscope": [0.0, 0.0, 0.0], "accelerometer": [0.0, 0.0, 9.81],
                    "rpy": [round(wobble, 4), 0.0, 0.0], "temperature": 35},
            "battery_soc": 88, "motors": [], "_offline": True,
        }
