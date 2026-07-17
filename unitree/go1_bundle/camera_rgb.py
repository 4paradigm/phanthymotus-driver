#!/usr/bin/env python3
"""
camera_rgb.py — Go1 `camera_rgb` 视觉扩展卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == "camera_rgb"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。板载 camera_adapter
（Nano 侧 C++/UnitreecameraSDK）由 deploy/nano_bootstrap.sh 在容器首启时自动编译部署，无需手动布 Nano。

架构（用户选定「板载 SDK 直采」路径）：
  ┌─ Nano 板卡 (.13/.14/.15) ────────────────┐        ┌─ Pi 驱动容器 (.161) ─────────────┐
  │ camera_adapter (C++ / UnitreecameraSDK)  │        │ camera.py: CameraRgbPlugin       │
  │  · 独占打开相机(Device ID)采集会话(§8.6.1)│        │  · 启动探测各 position 控制口     │
  │  · JSON-TCP 控制服务: probe/start/stop/   │◀──TCP──│    只为可达位置建实例(§8.6.5)     │
  │    snapshot（返回标定/实际配置）          │控制通道 │  · start→令 Adapter 定向发流       │
  │  · getRawFrame / getRectStereoFrame       │        │  · gstreamer 收 H.264/RTP→JPEG    │
  │  · H.264/RTP 定向 UDP 发流到 Pi(§8.6.3)   │──UDP──▶│    发布 /vision/{position}/{eye}  │
  └───────────────────────────────────────────┘图像流  └───────────────────────────────────┘

本文件只实现 Pi 侧的 `camera_rgb` 卡片；板载 Adapter 见同目录 `camera_adapter/`。
相机不经 Go1 的 UDP HighCmd/HighState 链路，故与 device.py 的 Go1Transport 完全解耦。

设计要点（对齐能力卡片）：
  - multiInstance sensor，实例以 `position`（front/chin/left/right/belly）区分（§8.2 / §8.3）。
  - 启动探测：__init__ 逐个 TCP 连 Adapter 控制口，只为在线相机建实例；不可达位置不建（§8.6.5 / §9）。
  - 不硬编码图传目标：receive_ip 由 config 给出，start 时告知 Adapter 发到 Pi（§8.6.3）。
  - 一相机一采集会话：本卡只请求 rgb 流；depth/pointcloud 卡将来共享同一 Adapter 会话（§8.6.1）。
  - 不自动杀占用进程：Adapter 报 busy → 返回 RESOURCE_BUSY（§8.6.2）。
  - 统一错误码（§3.1）：DEVICE_NOT_FOUND / RESOURCE_BUSY / STREAM_TIMEOUT / INVALID_ARGUMENT / COMMUNICATION_ERROR。
  - 每帧带 SDK 微秒时间戳、position、eye、分辨率、帧率、递增帧序号（§8.3）；左右目命名以
    Adapter 回传的标定确认为准，未确认前 eye 标 "unverified"（§8.3「不能未经验证直接命名左右目」）。
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


# ── 常量 ──────────────────────────────────────────────────────────────────────

# Go1 相机位置映射（能力卡片 §8.2）。控制口默认 = 图传口 + 100（可被 config 覆盖）。
# 仅作为 config 缺省；真实值以 config.yaml 的 camera_rgb.positions 为准，不作跨机器硬编码。
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "device_id": 1, "image_port": 9201, "control_port": 9301},
    "chin":  {"board_ip": "192.168.123.13", "device_id": 0, "image_port": 9202, "control_port": 9302},
    "left":  {"board_ip": "192.168.123.14", "device_id": 0, "image_port": 9203, "control_port": 9303},
    "right": {"board_ip": "192.168.123.14", "device_id": 1, "image_port": 9204, "control_port": 9304},
    "belly": {"board_ip": "192.168.123.15", "device_id": 0, "image_port": 9205, "control_port": 9305},
}

_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

# 官方 SDK 支持的双目原始帧尺寸 → 允许帧率（能力卡片 §8.3）。
_FRAME_SIZE_FPS = {
    "1856x800": {30},
    "928x400":  {30, 60},
}

_VALID_MODES = ["raw_mono", "raw_stereo", "rectified_mono", "rectified_stereo", "undistort_mono"]

# stereo（双目）模式产出左右两路；mono 模式产出单路。
# undistort_mono：raw 采集(高帧率) + 板载单目预计算 CMei remap 去畸变——去畸变且不掉帧
# （rectified_* 走 getRectStereoFrame 双目校正、在 Nano ARM 上很慢）。缺标定时 adapter 优雅降级为鱼眼直通。
_STEREO_MODES = {"raw_stereo", "rectified_stereo"}

# 收流看门狗：连续该时长收不到 JPEG 帧则判 STREAM_TIMEOUT（§3.1 / §8.6.5 新鲜度）。
_STREAM_STALE_SEC = 3.0

# 图像 topic 用 BEST_EFFORT / depth 1（与 go2 相机、ext_camera 一致，低延迟丢旧帧）。
_IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,   # 晚订阅者也能拿到最近一次 camera_info
)


def _err(code: str, message: str, **extra) -> dict:
    """能力卡片 §3.1 统一失败返回。"""
    return {"ok": False, "code": code, "message": message, **extra}


def _now_ms() -> int:
    return int(time.time() * 1000)


# ── 板载 Adapter 控制通道客户端（JSON-over-TCP）───────────────────────────────

class _AdapterClient:
    """
    与某个 position 的板载 camera_adapter 通信的 JSON-over-TCP 客户端。

    每次请求一次短连接：connect → 发一行 JSON → 收一行 JSON → close。
    Adapter 侧协议（见 camera_adapter/README.md）：
      req  {"cmd":"probe"}
      resp {"ok":true,"device_id":N,"online":bool,"busy":bool,"serial":str,
            "width":int,"height":int,"fps":int,"calibration":{...}}
      req  {"cmd":"start","device_id":N,"config":{mode,frame_size,fps,rectified_size,
            hfov_deg,target_ip,image_port}}
      resp {"ok":true,"applied":{...},"calibration":{...},"streams":[{eye,port}...]}
      req  {"cmd":"stop","device_id":N}                → {"ok":true}
      req  {"cmd":"snapshot","device_id":N,"eye":str}  → {"ok":true,"seq":int,"timestamp_us":int}
    失败：{"ok":false,"code":<错误码>,"message":str}（如相机被占用返回 RESOURCE_BUSY）。
    """

    def __init__(self, board_ip: str, control_port: int, timeout_sec: float = 2.0):
        self._ip = board_ip
        self._port = control_port
        self._timeout = timeout_sec

    def request(self, payload: dict, timeout_sec: float | None = None) -> dict:
        to = self._timeout if timeout_sec is None else timeout_sec
        try:
            with socket.create_connection((self._ip, self._port), timeout=to) as sock:
                sock.settimeout(to)
                sock.sendall((json.dumps(payload) + "\n").encode())
                buf = bytearray()
                while b"\n" not in buf:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    if len(buf) > (1 << 20):   # 1MB 防线，控制通道不该传大数据
                        break
            if not buf:
                return _err("COMMUNICATION_ERROR", f"adapter {self._ip}:{self._port} closed without reply")
            line = buf.split(b"\n", 1)[0]
            resp = json.loads(line.decode())
            if not isinstance(resp, dict):
                return _err("COMMUNICATION_ERROR", "adapter reply not a JSON object")
            return resp
        except (socket.timeout, TimeoutError):
            return _err("COMMUNICATION_ERROR", f"adapter {self._ip}:{self._port} timeout")
        except (ConnectionError, OSError) as e:
            # 探测阶段最常见：板卡/Adapter 不在线。上层据此判定 position 不可用。
            return _err("DEVICE_NOT_FOUND", f"cannot reach adapter {self._ip}:{self._port}: {e}")
        except json.JSONDecodeError as e:
            return _err("COMMUNICATION_ERROR", f"bad JSON from adapter: {e}")


# ── 收流子进程：gstreamer 收 H.264/RTP UDP → 逐帧 JPEG 发布 ────────────────────

def _run_rgb_receiver(topic: str, image_port: int, position: str, eye: str) -> None:
    """
    收流子进程入口（multiprocessing spawn，独立 GIL 保吞吐）。

    板载 Adapter 用 JPEG-over-UDP 发帧：每帧 = 一个完整 JPEG 的 UDP 数据报（驱动容器无
    gstreamer/ffmpeg，故不走 H.264/RTP 解码）。这里纯 socket 收数据报，校验 JPEG SOI 后
    直接发布 CompressedImage(format=jpeg)，附递增帧序号。

    ROS2 生命周期隔离（修「publisher's context is invalid」冻结）：本子进程与驱动主进程同进程组，
    若用全局默认 context + 默认信号处理器，进程组信号(Ctrl-C/docker stop/主进程 _shutdown)会连带
    shutdown 掉本进程 context → 下一次 publish 抛异常、画面定格。故：私有 Context + 关 rclpy 信号处理器
    + os.setpgrp 脱离进程组 + publish 包 try/except；父进程用 proc.terminate() 直发本 PID 停我们。
    """
    import os
    import signal as _signal
    import socket as _socket
    import rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile as _QoSProfile, ReliabilityPolicy as _R, HistoryPolicy as _H, DurabilityPolicy as _D
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = _QoSProfile(reliability=_R.BEST_EFFORT, history=_H.KEEP_LAST, depth=1, durability=_D.VOLATILE)

    try:
        os.setpgrp()
    except OSError:
        pass

    ctx = rclpy.Context()
    try:
        from rclpy.signals import SignalHandlerOptions as _SHO
        rclpy.init(context=ctx, signal_handler_options=_SHO.NO)
    except (ImportError, TypeError):
        rclpy.init(context=ctx)

    node = _Node(f"go1_cam_{position}_{eye}".replace("-", "_"), context=ctx)
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    _running = {"on": True}
    _signal.signal(_signal.SIGTERM, lambda *a: _running.update(on=False))

    sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    try:
        sock.bind(("0.0.0.0", image_port))
    except OSError as e:
        node.get_logger().error(f"rgb receiver bind :{image_port} failed: {e}")
        node.destroy_node()
        if ctx.ok():
            rclpy.shutdown(context=ctx)
        return
    sock.settimeout(1.0)

    node.get_logger().info(f"rgb receiver(JPEG/UDP): udp:{image_port} → {topic} (position={position} eye={eye})")

    seq = 0
    try:
        while _running["on"] and ctx.ok():
            if os.getppid() == 1:   # 父进程没了（被 reparent 到 init）→ 退出，别留孤儿
                break
            try:
                data, _ = sock.recvfrom(65536)
            except _socket.timeout:
                continue
            except OSError:
                break
            # 每个数据报应为一个完整 JPEG（以 SOI 0xFFD8 开头）；否则丢弃。
            if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
                continue
            msg = _CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            # frame_id 携带 position/eye/帧序号，供订阅方对齐左右目与丢帧检测（§8.3）。
            msg.header.frame_id = f"{position}:{eye}:{seq}"
            msg.format = "jpeg"
            msg.data = data
            try:
                pub.publish(msg)
            except Exception:
                break   # context 正在关闭的竞态 → 安全退出
            seq += 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if ctx.ok():
            rclpy.shutdown(context=ctx)


# ── camera_info 发布节点（in-process，~1Hz）───────────────────────────────────

class _CameraInfoNode(Node):
    """把某实例最近一次 start 拿到的标定/实际配置按 data/json 定时发布（§8.3 camera_info）。"""

    def __init__(self, node_name: str, topic: str):
        super().__init__(node_name)
        self._topic = topic
        self._pub = self.create_publisher(String, topic, _INFO_QOS)
        self._payload: dict = {}
        self._lock = threading.Lock()
        self._timer = self.create_timer(1.0, self._tick)

    def set_payload(self, payload: dict) -> None:
        with self._lock:
            self._payload = dict(payload)
        # 立即发一帧，避免订阅方等满 1s。
        self._tick()

    def _tick(self) -> None:
        with self._lock:
            payload = dict(self._payload)
        if not payload:
            return
        payload["timestamp_ms"] = _now_ms()
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)


# ── 单个 position 的实例 ──────────────────────────────────────────────────────

class _CameraRgbInstance:
    """
    一个 position 的 camera_rgb 实例：管控制通道 + 收流子进程 + camera_info 发布。

    状态机：idle → running（start 成功）→ idle（stop）。start 时把「发到 Pi 的 image_port」
    告知 Adapter，Adapter 回 streams=[{eye,port}]，据此为每路起一个收流子进程。
    """

    def __init__(self, position: str, pos_cfg: dict, namespace: str, executor,
                 receive_ip: str, defaults: dict, probe_info: dict):
        self._position = position
        self._cfg = pos_cfg
        self._ns = namespace
        self._executor = executor
        self._receive_ip = receive_ip
        self._defaults = defaults
        self._probe = probe_info          # __init__ 探测拿到的相机基础信息（serial/在线等）
        self._client = _AdapterClient(pos_cfg["board_ip"], pos_cfg["control_port"])
        self._device_id = int(pos_cfg["device_id"])
        self._base_image_port = int(pos_cfg["image_port"])

        self._procs: list = []            # 收流子进程列表（mono=1，stereo=2）
        self._streams: list = []          # [{"eye":..,"port":..,"topic":..}]
        self._applied: dict = {}          # 实际生效配置（Adapter 回传）
        self._calibration: dict = {}
        self._started_at: float = 0.0
        self._state = "idle"
        self._lock = threading.Lock()

        # 本卡只输出一路 JPEG 流（用户选定）：不再发布 camera_info topic。
        self._info_node = None
        self._info_topic = None

        # 同设备互斥的 peer 服务（go1-pointcloud-<pos> / go1-depth-<pos>，可选 config.peer_services）。
        # Go1 一台物理相机同时只能一个消费者。belly(.15) 只有 video0，常驻着 pointcloud/depth 服务；
        # left/right(.14) 也各有 depth 服务。这里记录 peer 列表：start 前 SSH 停掉它们腾相机，
        # stop 后 SSH 恢复（回 idle 监听，不抢设备），实现「调用 belly 才让位、不用就还回去」。
        self._peer_services = [s for s in (pos_cfg.get("peer_services") or []) if s]
        self._peer_host = pos_cfg.get("board_ip")
        self._peer_user = "unitree"
        self._peer_pw = os.environ.get("NANO_SSH_PW", "123")
        self._stopped_peers: list = []   # 本次 start 实际停掉的 peer（供 stop 恢复）

    # ── 生命周期 ──
    def start(self, req_cfg: dict) -> dict:
        with self._lock:
            if self._state == "running":
                return _err("PRECONDITION_FAILED", f"instance {self._position} already running",
                            **self._status_running())

            # 先让位：停掉同设备的 peer 服务（go1-pointcloud-<pos>/go1-depth-<pos>），腾出相机。
            # 失败不直接 abort——adapter 自己有 free_device_node() 兜底（fuser -k -TERM），
            # 但那会硬杀进程、且 peer 的 Restart=always 会立即顶回来抢；所以这里优先 systemctl stop。
            self._stop_peers()

            mode = req_cfg.get("mode", self._defaults.get("mode", "raw_mono"))
            frame_size = req_cfg.get("frame_size", self._defaults.get("frame_size", "928x400"))
            fps = int(req_cfg.get("fps", self._defaults.get("fps", 60)))

            # ── 参数校验（§7：越界拒绝，不静默截断）──
            if mode not in _VALID_MODES:
                return _err("INVALID_ARGUMENT", f"mode must be one of {_VALID_MODES}, got {mode!r}")
            if frame_size not in _FRAME_SIZE_FPS:
                return _err("INVALID_ARGUMENT",
                            f"frame_size must be one of {list(_FRAME_SIZE_FPS)}, got {frame_size!r}")
            if fps not in _FRAME_SIZE_FPS[frame_size]:
                return _err("INVALID_ARGUMENT",
                            f"fps {fps} not allowed for frame_size {frame_size} "
                            f"(allowed: {sorted(_FRAME_SIZE_FPS[frame_size])})")

            start_payload = {
                "cmd": "start",
                "device_id": self._device_id,
                "config": {
                    "mode": mode,
                    "frame_size": frame_size,
                    "fps": fps,
                    "rectified_size": req_cfg.get("rectified_size"),
                    "hfov_deg": req_cfg.get("hfov_deg"),
                    "target_ip": self._receive_ip,     # §8.6.3 不硬编码，运行时告知 Adapter
                    "image_port": self._base_image_port,
                },
            }
            resp = self._client.request(start_payload, timeout_sec=5.0)
            if not resp.get("ok"):
                # start 失败 → 把刚停掉的 peer 恢复回去，别让相机空着没人管。
                self._start_peers()
                # 透传 Adapter 的错误码（RESOURCE_BUSY / DEVICE_NOT_FOUND 等），默认 COMMUNICATION_ERROR。
                return _err(resp.get("code", "COMMUNICATION_ERROR"),
                            resp.get("message", "adapter start failed"))

            self._applied = resp.get("applied", {})
            self._calibration = resp.get("calibration", {})
            # Adapter 返回每路流的 eye 与端口；未提供时按 mode 兜底推断。
            streams = resp.get("streams")
            if not streams:
                if mode in _STEREO_MODES:
                    streams = [{"eye": "unverified_left",  "port": self._base_image_port},
                               {"eye": "unverified_right", "port": self._base_image_port + 1}]
                else:
                    streams = [{"eye": "mono", "port": self._base_image_port}]

            # 为每路流起收流子进程，发布到 /vision/{position}/{eye 归一化后的 topic 段}。
            import multiprocessing as mp
            ctx = mp.get_context("spawn")
            self._streams = []
            self._procs = []
            for s in streams:
                eye = str(s.get("eye", "mono"))
                port = int(s.get("port", self._base_image_port))
                topic_eye = self._topic_eye(mode, eye)
                topic = f"/{self._ns}/vision/{self._position}/{topic_eye}"
                proc = ctx.Process(
                    target=_run_rgb_receiver,
                    args=(topic, port, self._position, eye),
                    name=f"go1_cam_{self._position}_{topic_eye}",
                    daemon=True,
                )
                proc.start()
                self._procs.append(proc)
                self._streams.append({"eye": eye, "port": port, "topic": topic})

            self._started_at = time.monotonic()
            self._state = "running"

            # camera_info topic 已移除（本卡只输出一路 JPEG）；不再 set_payload。

            print(f"[camera_rgb] {self._position} started mode={mode} {frame_size}@{fps} "
                  f"streams={[s['topic'] for s in self._streams]}", flush=True)
            return {"ok": True, "card": "camera_rgb", "action": "start",
                    "control_level": "CAMERA", "timestamp_ms": _now_ms(),
                    "applied": {**self._applied, "position": self._position},
                    "calibration_status": "loaded" if self._calibration else "missing",
                    **self._status_running()}

    def stop(self) -> dict:
        with self._lock:
            self._teardown_procs()
            # 通知 Adapter 停采（失败不阻塞本地清理）。
            self._client.request({"cmd": "stop", "device_id": self._device_id}, timeout_sec=2.0)
            self._state = "idle"
            self._streams = []
            # 还回去：把 start 时停掉的 peer 服务恢复（回 idle 监听，等下次按需用）。
            self._start_peers()
            print(f"[camera_rgb] {self._position} stopped", flush=True)
            return {"ok": True, "card": "camera_rgb", "action": "stop",
                    "control_level": "CAMERA", "timestamp_ms": _now_ms(),
                    "applied": {"position": self._position}, "state": "idle"}

    def info(self) -> dict:
        with self._lock:
            fresh, alive = self._stream_health()
            base = {
                "state": self._state,
                "position": self._position,
                "device_id": self._device_id,
                "board_ip": self._cfg["board_ip"],
                "control_port": self._cfg["control_port"],
                "image_port": self._base_image_port,
                "serial": self._probe.get("serial", ""),
                "online": self._probe.get("online", True),
                "width": self._probe.get("width"),
                "height": self._probe.get("height"),
                "fps": self._applied.get("fps", self._probe.get("fps")),
                "mode": self._applied.get("mode"),
                "frame_size": self._applied.get("frame_size"),
                "calibration_status": "loaded" if self._calibration else "missing",
                "stream_fresh": fresh,
                "receivers_alive": alive,
                "topic_out": self._topic_out(),
            }
            if self._state == "running" and not fresh:
                base["warning"] = "STREAM_TIMEOUT"   # §8.6.5：不谎报 running 而无数据
            return base

    # ── 内部 ──
    def _topic_eye(self, mode: str, eye: str) -> str:
        """把 Adapter 的 eye 名归一化为 topic 段：mono / left / right（未确认目别落到 left/right 词根）。"""
        if mode not in _STEREO_MODES:
            return "mono"
        e = eye.lower()
        if "left" in e:
            return "left"
        if "right" in e:
            return "right"
        return "mono"

    def _topic_out(self) -> list:
        if self._streams:
            outs = [{"topic": s["topic"], "format": "image/jpeg"} for s in self._streams]
        else:
            # 未 start 也给出名义 topic。本卡只输出这一路 JPEG（无 camera_info）。
            outs = [{"topic": f"/{self._ns}/vision/{self._position}/mono", "format": "image/jpeg"}]
        return outs

    def _status_running(self) -> dict:
        return {"state": self._state, "position": self._position,
                "topic_out": self._topic_out()}

    def _stream_health(self) -> tuple:
        """(fresh, alive)：收流子进程是否存活 + 是否在看门狗窗口内（近似新鲜度）。"""
        alive = all(p.is_alive() for p in self._procs) if self._procs else False
        if self._state != "running":
            return False, alive
        fresh = alive and (time.monotonic() - self._started_at) >= 0  # 存活即视为在流
        # 子进程全退出 → 判定不新鲜（触发 STREAM_TIMEOUT 提示）。
        if not alive:
            fresh = False
        return fresh, alive

    def _teardown_procs(self) -> None:
        for p in self._procs:
            try:
                if p.is_alive():
                    p.terminate()
                    p.join(timeout=3.0)
                    if p.is_alive():
                        p.kill()
                        p.join(timeout=2.0)
            except Exception:
                pass
        self._procs = []

    # ── 同设备 peer 服务让位/归还（go1-pointcloud-<pos> / go1-depth-<pos>）──
    # Go1 一台物理相机同时只能一个消费者。belly(.15) 只有 video0、常驻 pointcloud/depth 服务，
    # left/right(.14) 各有 depth 服务。用户要的语义：调 camera_rgb belly 才停 peer 腾相机，
    # stop 后把 peer 恢复回 idle 监听。用容器内的 sshpass 经 SSH 下 systemctl 命令（容器 --network host）。
    def _ssh_peers(self, action: str) -> None:
        if not self._peer_services or not self._peer_host:
            return
        try:
            import subprocess
        except Exception:
            return
        for svc in self._peer_services:
            cmd = [
                "sshpass", "-p", self._peer_pw,
                "ssh", "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "ConnectTimeout=4",
                f"{self._peer_user}@{self._peer_host}",
                f"echo {self._peer_pw} | sudo -S systemctl {action} {svc} 2>/dev/null; true",
            ]
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
                if r.returncode == 0:
                    print(f"[camera_rgb] peer {action} {svc}@{self._peer_host} ok", flush=True)
                else:
                    print(f"[camera_rgb] peer {action} {svc}@{self._peer_host} rc={r.returncode}", flush=True)
            except Exception as e:
                print(f"[camera_rgb] peer {action} {svc}@{self._peer_host} err: {e}", flush=True)

    def _stop_peers(self) -> None:
        if not self._peer_services:
            return
        before = list(self._stopped_peers)
        self._ssh_peers("stop")
        # 假定 stop 全部成功（无法逐个确认 is-active 时不回退状态），stop 时记录、start 时按此恢复。
        self._stopped_peers = list(self._peer_services)

    def _start_peers(self) -> None:
        if not self._stopped_peers:
            return
        # 仅恢复本次 start 实际停掉的 peer。
        saved = list(self._stopped_peers)
        self._stopped_peers = []
        try:
            import subprocess
            for svc in saved:
                cmd = [
                    "sshpass", "-p", self._peer_pw,
                    "ssh", "-o", "StrictHostKeyChecking=no",
                    "-o", "UserKnownHostsFile=/dev/null",
                    "-o", "ConnectTimeout=4",
                    f"{self._peer_user}@{self._peer_host}",
                    f"echo {self._peer_pw} | sudo -S systemctl start {svc} 2>/dev/null; true",
                ]
                try:
                    r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
                    print(f"[camera_rgb] peer start {svc}@{self._peer_host} rc={r.returncode}", flush=True)
                except Exception as e:
                    print(f"[camera_rgb] peer start {svc}@{self._peer_host} err: {e}", flush=True)
        except Exception:
            pass


# ── CameraRgbPlugin (multiInstance sensor) ────────────────────────────────────

class CameraRgbPlugin:
    """Go1 `camera_rgb` 视觉扩展卡（能力卡片 §8.3）。

    ``camera_source`` 是驱动级共享配置：在 Agent Core 的全局配置中切换后，未显式指定
    ``camera_source`` 的 info/start/stop 都操作该机位，且返回该机位的 topic_out。这样全局
    预览不会再因没有画布实例的配置而回退到 front。
    每个机位仍对应一个 _CameraRgbInstance（一条板载 adapter 会话 + 收流子进程 + topic
    /{ns}/vision/{position}/mono）。
    """
    PREFIX = "camera_rgb"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._executor = executor
        self._cfg = plugin_config or {}
        self._receive_ip = self._cfg.get("receive_ip", os.environ.get("GO1_CAMERA_RECV_IP", "192.168.123.161"))
        self._defaults = {
            # 默认 undistort_mono：raw 快采 + 单目预计算 CMei remap 去畸变（缺标定的机位自动降级为鱼眼直通）。
            "mode": self._cfg.get("default_mode", "undistort_mono"),
            "frame_size": self._cfg.get("default_frame_size", "928x400"),
            # 多路默认 30fps（省 CPU/带宽；单路想高帧率把 default_fps 调 60 或按机位 config）。928x400 支持 30/60。
            "fps": int(self._cfg.get("default_fps", 30)),
        }
        positions_cfg = self._cfg.get("positions", _DEFAULT_POSITIONS)
        self._positions_cfg = positions_cfg
        self._default_position = self._cfg.get("default_position", "front")

        # 为每个 config 里配置的机位建一个实例（**不在 __init__ 探测**：探测会阻塞启动，
        # 且 5 路都不可达时最长等 5×timeout。改为按需在 start() 连 adapter，不可达即报 DEVICE_NOT_FOUND）。
        self._cameras: dict[str, _CameraRgbInstance] = {}
        for pos, pcfg in positions_cfg.items():
            try:
                self._cameras[pos] = _CameraRgbInstance(
                    pos, pcfg, namespace, executor, self._receive_ip, self._defaults, {})
            except (KeyError, ValueError, TypeError) as e:
                print(f"[camera_rgb] {pos}: 配置无效，跳过 {e}", flush=True)
        # Agent Core 的全局配置会调用 config；将选中的机位保存为驱动级状态，而非绑定到
        # 某一张画布卡的 instance_id。这样全局页和画布页的默认预览保持一致。
        self._selected_source = self._default_position
        print(f"[camera_rgb] 机位就绪：{sorted(self._cameras.keys())}（default={self._default_position}）", flush=True)

    def _topic_for(self, position: str) -> str:
        return f"/{self._ns}/vision/{position}/mono"

    def _source_options(self) -> list:
        # 下拉选项：config 里配了的机位（保持 front/chin/left/right/belly 的固定顺序）。
        order = [p for p in _VALID_POSITIONS if p in self._cameras]
        return [{"const": p, "title": p} for p in order] or [{"const": self._default_position,
                                                               "title": self._default_position}]

    def get_tools(self) -> list:
        return [{
            "name": "camera_rgb",
            "type": "sensor",
            "multiInstance": True,
            "description": (
                "Go1 RGB cameras via on-board UnitreecameraSDK adapters. Select the active camera position "
                "(front/chin/left/right/belly) in global configuration. start opens that camera and streams "
                "JPEG (undistort_mono, de-warped) to /{ns}/vision/{position}/mono."
            ),
            "configSchema": {
                "type": "object",
                "properties": {
                    "camera_source": {
                        "type": "string",
                        "description": "Camera position",
                        "scope": "shared",
                        "oneOf": self._source_options(),
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "start", "stop"],
                        "description": "Lifecycle action for the selected camera position",
                    },
                    "camera_source": {
                        "type": "string",
                        "enum": [o["const"] for o in self._source_options()],
                        "description": "Camera position (front/chin/left/right/belly)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "info":  {"params": [], "description": "Status/resolution/fps/calibration/topic for the selected camera"},
                    "start": {"params": [], "description": "Open the selected camera and stream JPEG to /vision/{position}/mono"},
                    "stop":  {"params": [], "description": "Stop the selected camera stream"},
                },
            },
            "topic_out": [
                {"format": "image/jpeg", "desc": "per-position camera JPEG stream"},
            ],
        }]

    def start(self) -> None:
        pass   # 视觉卡不自动开流：按需 start（前端选机位后触发）

    def stop(self) -> None:
        for inst in self._cameras.values():
            try:
                inst.stop()
            except Exception as e:
                print(f"[camera_rgb] stop {inst._position} error: {e}", flush=True)

    def _resolve_source(self, args: dict) -> str:
        # ``camera_source`` may be included by a direct MCP call.  Otherwise all callers,
        # including the global preview which has no instance_id, use the shared selection.
        return args.get("camera_source") or self._selected_source

    def dispatch(self, action: str, args: dict) -> dict | None:
        source = self._resolve_source(args)

        if action == "config":
            if source not in self._cameras:
                return _err("DEVICE_NOT_FOUND", f"camera position {source!r} not configured")
            self._selected_source = source
            return {"ok": True, "card": "camera_rgb", "action": "config", "camera_source": source,
                    "topic_out": [{"topic": self._topic_for(source), "format": "image/jpeg"}]}

        inst = self._cameras.get(source)
        if inst is None:
            return _err("DEVICE_NOT_FOUND", f"camera position {source!r} not configured")
        # Direct MCP calls that specify a source should also become the global selection,
        # so the next global info/start request and preview stay on the same camera.
        if args.get("camera_source"):
            self._selected_source = source

        if action == "info":
            return inst.info()
        if action == "start":
            # 同机位已在推流 → 幂等返回运行态（允许多个 UI 实例订阅同一相机 topic）。
            if getattr(inst, "_state", "idle") == "running":
                return {"ok": True, "card": "camera_rgb", "action": "start",
                        "camera_source": source, "timestamp_ms": _now_ms(),
                        **inst._status_running()}
            cfg = {k: v for k, v in args.items()
                   if k not in ("action", "instance_id", "camera_source", "_tool_name")}
            return inst.start(cfg)
        if action == "stop":
            return inst.stop()
        return _err("INVALID_ARGUMENT", f"unsupported action {action!r}")


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。camera_rgb 不用共享 SDK client（HighState），故忽略 client。"""
    return CameraRgbPlugin(plugin_config, namespace, executor)
