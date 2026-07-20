#!/usr/bin/env python3
"""
camera_rgb.py — Go1 `camera_rgb` 视觉扩展卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == "camera_rgb"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。板载 rgb_stream
（Nano 侧 C++/UnitreecameraSDK）由 deploy/nano_bootstrap.sh 在容器首启时自动编译部署，无需手动布 Nano。

架构（按需 TCP 长度前缀，镜像 test_camera_depth / depth_stream 路线）：
  ┌─ Nano 板卡 (.13/.14/.15) ─────────────────┐        ┌─ Pi 驱动容器 (.161) ─────────────┐
  │ rgb_stream (C++ / UnitreeCameraSDK)        │        │ camera_rgb.py: CameraRgbPlugin   │
  │  · 每路一个常驻 systemd 服务(9201~9205)     │        │  · multiInstance sensor          │
  │  · 空闲时只监听 TCP,不占相机                 │  TCP   │  · 卡 start → 连对应机位 920x     │
  │  · 客户端连上才开相机(getRectStereoFrame      │◀──────▶│    收 [4B 长度][JPEG] → 发        │
  │    取去鱼眼左目 + flip 翻正) → 推 JPEG        │ 图像流  │    CompressedImage 到             │
  │  · 客户端断开 → _exit(0) 释放相机(systemd 重启)│        │    /{ns}/vision/{position}/mono  │
  └────────────────────────────────────────────┘        │  · stop → 断 TCP → Nano 释放相机   │
                                                         └──────────────────────────────────┘

为什么放弃旧 camera_adapter(JSON 控制口 + UDP 图传)路线改用这条：
  · 旧路线靠 UDP 图传口 + JSON 控制口双通道,.14/.15 没装 adapter 服务就没流 → 只有 front/chin。
  · 旧路线 adapter 发原始/rectified 帧但没翻正 → 鱼眼 + 上下颠倒。
  · 旧路线 UDP 每帧一个数据报,丢包整帧丢 → 帧率几帧且不稳。
  本路线:五路各自常驻服务(Nano 现编)、TCP 不丢包、rectified 去鱼眼 + flip 翻正、连上才开相机不抢设备。
"""

from __future__ import annotations

import socket
import struct
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False
    _QOS = None

CARD = "camera_rgb"
TYPE = "sensor"
FMT = "image/jpeg"

# 超时分三段：连接握手短、等第一帧长、稳态短。
#   Nano 侧 rgb_stream 连上后要 startStereoCompute() 暖机,实测第一帧 ~5-6.5s 才出(去鱼眼立体管线固有)。
#   旧代码用 create_connection(timeout=5) → 这个 5s 同时成了读超时 → 第一帧必然超时 → 断开 →
#   Nano _exit(0) 重启 → 重连又卡在暖机 → 永远黑屏死循环。故读超时必须显著大于第一帧延迟。
_CONNECT_TIMEOUT = 8.0      # TCP 建连
_FIRST_FRAME_TIMEOUT = 20.0 # 等第一帧(含 Nano 立体管线暖机,留足余量)
_STEADY_TIMEOUT = 8.0       # 稳态:出帧后 ~14fps,8s 收不到即判定流断

# 机位 → 板卡 IP / 图传端口(与 nano_bootstrap.sh RGB_ROWS 对齐;device_id 在 Nano 侧服务里定)。
# 端口 92xx 与深度 91xx、点云 94xx 错开。config.positions 可覆盖 board_ip/image_port。
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "image_port": 9201},
    "chin":  {"board_ip": "192.168.123.13", "image_port": 9202},
    "left":  {"board_ip": "192.168.123.14", "image_port": 9203},
    "right": {"board_ip": "192.168.123.14", "image_port": 9204},
    "belly": {"board_ip": "192.168.123.15", "image_port": 9205},
}
_POS_TITLE = {"front": "Front (头部前 dev1)", "chin": "Chin (头部下 dev0)",
              "left": "Left (侧左 dev0)", "right": "Right (侧右 dev1)", "belly": "Belly (腹部 dev0)"}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

DESC = ("Go1 RGB cameras via on-board UnitreecameraSDK rgb_stream (rectified, de-warped, flip-corrected) → "
        "JPEG @ ~20-30Hz. multiInstance, pick `position` per card. start → connect & stream to "
        "/{ns}/vision/{position}/mono; stop → disconnect & release the camera. Change the position "
        "dropdown to hot-switch source (~3-4s to first frame). One position at a time per camera "
        "(shares each camera with depth/point cloud — mutually exclusive).")


def _err(code: str, message: str, **extra) -> dict:
    return {"ok": False, "code": code, "message": message, **extra}


def _now_ms() -> int:
    return int(time.time() * 1000)


def _recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


# ── 单实例 RGB 桥:连某机位 board_ip:image_port 收 [4B 长度][JPEG] → 发 CompressedImage ──

class _RgbStream:
    """连板载 rgb_stream,逐帧收 JPEG 发布到实例专属 topic。

    连上才开相机(Nano 侧),断开即释放(同 depth_stream 路线)。
    """

    def __init__(self, node: "Node", topic: str):
        self._node = node
        self._topic = topic
        self._pub = node.create_publisher(CompressedImage, topic, _QOS) if _HAS_ROS2 else None
        self._run = False
        self._gen = 0
        self.connected = False
        self.frames = 0
        self.position = None

    def start(self, position: str, host: str, port: int):
        self._run = True
        self._gen += 1
        gen = self._gen
        self.position = position
        self.connected = False
        threading.Thread(target=self._loop, args=(gen, position, host, port), daemon=True).start()

    def stop(self):
        self._run = False
        self._gen += 1        # 让在跑的 loop 线程退出并断开 → Nano 侧 _exit(0) 释放相机
        self.connected = False

    def _loop(self, gen, position, host, port):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=_CONNECT_TIMEOUT)
                # create_connection 会把 timeout 留在 socket 上当读超时;第一帧要等 Nano 暖机 ~5-6.5s,
                # 故显式放宽读超时到 _FIRST_FRAME_TIMEOUT,出帧后再收紧到 _STEADY_TIMEOUT。
                s.settimeout(_FIRST_FRAME_TIMEOUT)
                self.connected = True
                self._node.get_logger().info(f"[{position}] 已连上 rgb_stream {host}:{port}(等第一帧,暖机中~5-6s)")
            except Exception:
                self.connected = False
                time.sleep(2)
                continue
            try:
                got_first = False
                while self._run and gen == self._gen:
                    hdr = _recvall(s, 4)
                    if hdr is None:
                        break
                    n = struct.unpack(">I", hdr)[0]
                    if n <= 0 or n > 5_000_000:
                        break
                    data = _recvall(s, n)
                    if data is None:
                        break
                    if not got_first:
                        got_first = True
                        s.settimeout(_STEADY_TIMEOUT)   # 第一帧已到 → 收紧读超时以便快速发现流断
                        self._node.get_logger().info(f"[{position}] 首帧到达,进入稳态推流")
                    if self._pub is not None:
                        msg = CompressedImage()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_rgb"
                        msg.format = "jpeg"
                        msg.data = data
                        try:
                            self._pub.publish(msg)
                        except Exception:
                            break
                    self.frames += 1
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{position}] rgb stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()          # 断开 → rgb_stream _exit(0) 释放相机
                except Exception:
                    pass


# ── CameraRgbPlugin (multiInstance sensor) ────────────────────────────────────

class CameraRgbPlugin:
    """Go1 `camera_rgb` 视觉扩展卡。

    multiInstance:每张画布卡实例用 instance_id 区分,各自选一个 position、各自一条 topic
    `/{ns}/vision/{position}/mono`。公共默认机位由 config.default_position 给(拖入前用)。
    """

    PREFIX = "camera_rgb"

    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._positions = {p: dict(v) for p, v in _DEFAULT_POSITIONS.items()}
        for pos, ov in (c.get("positions") or {}).items():
            if pos in self._positions and isinstance(ov, dict):
                self._positions[pos].update(ov)
        self._default_pos = str(c.get("default_position", "front")).lower()
        if self._default_pos not in self._positions:
            self._default_pos = "front"
        self._node = None
        self._streams: dict = {}          # instance_id -> _RgbStream
        self._cfg: dict = {}              # instance_id -> {"position": ...}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_camera_rgb")
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None
        print(f"[{CARD}] 机位就绪：{sorted(self._positions.keys())}（default={self._default_pos}）", flush=True)

    def _topic(self, iid: str) -> str:
        # 实例 topic 用 instance_id 区分；instance_id 默认就是 position 名 → topic 即 /{ns}/vision/{pos}/mono
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/vision/{safe}/mono"

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        # 兼容平台多种下发字段:新平台 config.position / 顶层 position / 旧 camera_source /
        # instance_id 直接就是机位名(front/.../belly);都缺才回 default_position。
        cand = (cfg.get("position") or args.get("position") or args.get("camera_source")
                or self._cfg.get(iid, {}).get("position"))
        if not cand:
            cand = iid if iid in self._positions else self._default_pos
        pos = str(cand).lower()
        return pos if pos in self._positions else self._default_pos

    def _stream_for(self, iid: str) -> "_RgbStream":
        st = self._streams.get(iid)
        if st is None:
            st = _RgbStream(self._node, self._topic(iid))
            self._streams[iid] = st
        return st

    # 框架加载时调用:multiInstance 按实例 start,这里不连、不占相机。
    def start(self):
        if self._node is None:
            print(f"[{CARD}] 无 rclpy/executor,推流不可用(仅登记 tool)", flush=True)

    def stop(self):
        for st in self._streams.values():
            try:
                st.stop()
            except Exception:
                pass

    def _start_instance(self, iid: str, position: str) -> dict:
        if position not in self._positions:
            return _err("INVALID_ARGUMENT", f"unknown position {position!r}; valid: {_VALID_POSITIONS}")
        if self._node is None:
            return _err("COMMUNICATION_ERROR", "no rclpy/executor")
        p = self._positions[position]
        st = self._stream_for(iid)
        st.stop()
        st.start(position, p["board_ip"], int(p["image_port"]))
        return {"ok": True, "card": CARD, "action": "start", "timestamp_ms": _now_ms(),
                "state": "running", "position": position,
                "topic_out": [{"topic": self._topic(iid), "format": FMT}]}

    def get_tools(self) -> list:
        return [{
            "name": CARD, "type": TYPE, "multiInstance": True,
            "description": DESC + (" — ROS2 CompressedImage" if self._node else " — no rclpy, poll via MCP"),
            "configSchema": {
                "type": "object",
                "properties": {
                    "position": {
                        "type": "string",
                        "description": "读取哪一路相机的 RGB(改此项即热切源)",
                        "scope": "instance",
                        "oneOf": [{"const": p, "title": _POS_TITLE[p]} for p in _VALID_POSITIONS],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"],
                                          "description": "start=连并推流 / stop=断开并释放相机 / info=查状态"}},
                "required": ["action"],
            },
            "topic_out": [],
        }]

    def dispatch(self, action, args) -> dict | None:
        iid = args.get("instance_id") or "default"

        if action == "config":
            pos = self._resolve_pos(iid, args)
            if pos not in self._positions:
                return _err("INVALID_ARGUMENT", f"unknown position {pos!r}; valid: {_VALID_POSITIONS}")
            self._cfg[iid] = {"position": pos}
            st = self._streams.get(iid)
            if st is not None and st._run and st.position != pos:   # 运行中改机位 → 热重启
                self._start_instance(iid, pos)
            return {"ok": True, "position": pos,
                    "topic_out": [{"topic": self._topic(iid), "format": FMT}]}

        if action == "start":
            return self._start_instance(iid, self._resolve_pos(iid, args))

        if action == "stop":
            st = self._streams.get(iid)
            if st is not None:
                st.stop()
            return {"ok": True, "card": CARD, "action": "stop", "timestamp_ms": _now_ms(),
                    "state": "idle", "position": self._cfg.get(iid, {}).get("position", self._default_pos)}

        if action in ("info", "read", "get", CARD):
            pos = self._resolve_pos(iid, args)
            p = self._positions.get(pos, {})
            st = self._streams.get(iid)
            state = "running" if (st and st.connected) else ("waiting" if st and st._run else "idle")
            base = {
                "state": state, "position": pos,
                "positions_available": _VALID_POSITIONS,
                "format": "sensor_msgs/CompressedImage (jpeg)",
                "source": f"{pos} @ {p.get('board_ip')}:{p.get('image_port')} (rgb_stream)",
                "connected_to_nano": bool(st and st.connected),
                "frames_published": st.frames if st else 0,
                "topic_out": [{"topic": self._topic(iid), "format": FMT}] if self._node else [],
            }
            if state == "running":
                base["note"] = "streaming rectified+flip-corrected JPEG; stop to release camera"
            else:
                base["note"] = "start to connect (Nano opens camera on connect); stop releases it"
            return base
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。camera_rgb 不用共享 SDK client(HighState),故忽略 client。"""
    return CameraRgbPlugin(plugin_config, namespace, executor, client)
