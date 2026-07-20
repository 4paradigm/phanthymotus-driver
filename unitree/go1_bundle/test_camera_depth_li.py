#!/usr/bin/env python3
"""
test_camera_depth_li.py — Go1 深度推流卡（li 版，修复机位解析，对齐 camera_rgb 实现）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == "test_camera_depth_li"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。

为什么写这张卡：
  test_camera_depth 的 _resolve_pos() 在平台未显式下发 position 字段时，
  始终退回 default_position("belly")，导致五机位只能识别 belly。
  camera_rgb 的正确做法：当 instance_id 本身就是合法机位名时直接用 iid 推断机位，
  本卡照搬该逻辑修复此问题，五机位均可正常选通。

架构（与 test_camera_depth 同路，与 camera_rgb 完全对齐）：
  ┌─ Nano 板卡 (.13/.14/.15) ──────────────────┐        ┌─ Pi 驱动容器 (.161) ──────────────────────┐
  │ depth_stream (C++ / UnitreeCameraSDK)       │        │ test_camera_depth_li.py                   │
  │  · 五路常驻 systemd 服务 (9101~9105)         │        │  · multiInstance sensor                   │
  │  · 空闲时只监听 TCP,不占相机                  │  TCP   │  · 卡 start → 连对应机位 910x              │
  │  · 客户端连上才开相机(getDepthFrame 彩色深度图 │◀──────▶│    收 [4B 长度][JPEG] → 发                │
  │    → JPEG) → 推流                           │ 图像流  │    CompressedImage 到                     │
  │  · 客户端断开 → _exit(0) 释放相机            │        │    /{ns}/camera/{position}/depth           │
  └─────────────────────────────────────────────┘        │  · stop → 断 TCP → Nano 释放相机           │
                                                          └───────────────────────────────────────────┘

验收后可改名 camera_depth（与 test_camera_depth 同意图的更完整实现）。
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

CARD = "test_camera_depth_li"
TYPE = "sensor"
FMT = "image/jpeg"

# 超时分三段：连接短、等首帧宽松、稳态收紧。
#   depth_stream SDK 初始化约 3~4s，首帧略快于 rgb_stream，但仍需给足余量。
_CONNECT_TIMEOUT = 8.0       # TCP 建连超时
_FIRST_FRAME_TIMEOUT = 15.0  # 等首帧（含 Nano SDK 初始化，留足余量）
_STEADY_TIMEOUT = 8.0        # 稳态：~10fps，8s 无帧即判流断

# 机位 → 板卡 IP / 深度端口（与 nano_bootstrap.sh DEPTH_ROWS 对齐；device_id 在 Nano 侧定）。
# 端口 91xx 与点云 94xx、RGB 图传 92xx 全部错开。config.positions 可覆盖 board_ip/depth_port。
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "depth_port": 9101},
    "chin":  {"board_ip": "192.168.123.13", "depth_port": 9102},
    "left":  {"board_ip": "192.168.123.14", "depth_port": 9103},
    "right": {"board_ip": "192.168.123.14", "depth_port": 9104},
    "belly": {"board_ip": "192.168.123.15", "depth_port": 9105},
}
_POS_TITLE = {"front": "Front (头部前 dev1)", "chin": "Chin (头部下 dev0)",
              "left": "Left (侧左 dev0)", "right": "Right (侧右 dev1)", "belly": "Belly (腹部 dev0)"}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

DESC = ("Go1 五机位深度流（~10Hz，彩色 JPEG：近红/远青）— multiInstance，position 下拉框选机位。"
        "Nano 侧 depth_stream(getDepthFrame) 按需开相机（连才开/断即放），"
        "TCP 收帧桥接到 ROS2 sensor_msgs/CompressedImage。"
        "start=连接并推流 / stop=断开并释放相机 / config.position=热切机位（~3-4s 首帧）。"
        "一次一路（立体重运算）；与点云指向同一相机时互斥（谁连谁占）。")


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


# ── 单实例深度桥：连某机位 board_ip:depth_port 收 [4B 长度][JPEG] → 发布 CompressedImage ──

class _DepthStream:
    """连板载 depth_stream，逐帧收彩色深度 JPEG 发布到实例专属 topic。

    连上才开相机（Nano 侧），断开即释放（同 camera_rgb 路线）。
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
                # create_connection 会把 timeout 留在 socket 上当读超时；首帧需等 SDK 初始化，
                # 故显式放宽到 _FIRST_FRAME_TIMEOUT，出帧后再收紧到 _STEADY_TIMEOUT。
                s.settimeout(_FIRST_FRAME_TIMEOUT)
                self.connected = True
                self._node.get_logger().info(
                    f"[{position}] 已连上 depth_stream {host}:{port}（等首帧，SDK 初始化~3-4s）")
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
                        s.settimeout(_STEADY_TIMEOUT)   # 首帧已到 → 收紧读超时以便快速发现流断
                        self._node.get_logger().info(f"[{position}] 首帧到达，进入稳态推流")
                    if self._pub is not None:
                        msg = CompressedImage()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_depth"
                        msg.format = "jpeg"
                        msg.data = data
                        try:
                            self._pub.publish(msg)
                        except Exception:
                            break
                    self.frames += 1
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{position}] depth stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()          # 断开 → depth_stream _exit(0) 释放相机
                except Exception:
                    pass


# ── CameraDepthPlugin (multiInstance sensor) ─────────────────────────────────

class CameraDepthPlugin:
    """Go1 `test_camera_depth_li` 深度视觉扩展卡。

    multiInstance：每张画布卡实例用 instance_id 区分，各自选一个 position、各自一条 topic
    `/{ns}/camera/{position}/depth`。公共默认机位由 config.default_position 给（拖入前用）。

    关键修复（对比 test_camera_depth）：
      _resolve_pos() 当无显式 position 时，先尝试以 iid 作为机位名（平台 instance_id 通常
      等于机位名）；仅在 iid 不合法时才退回 default_position。这一逻辑与 camera_rgb 完全一致。
    """

    PREFIX = "test_camera_depth_li"

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
        self._streams: dict = {}          # instance_id -> _DepthStream
        self._cfg: dict = {}              # instance_id -> {"position": ...}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_test_camera_depth_li")
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None
        print(f"[{CARD}] 机位就绪：{sorted(self._positions.keys())}（default={self._default_pos}）", flush=True)

    def _topic(self, iid: str) -> str:
        # 实例 topic 用 instance_id 区分；instance_id 默认就是 position 名 → topic 即 /{ns}/camera/{pos}/depth
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/depth"

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        # 兼容平台多种下发字段：新平台 config.position / 顶层 position / 旧 camera_source /
        # instance_id 直接就是机位名（front/.../belly）；都缺才回 default_position。
        # ★ 关键修复：当 iid 本身是合法机位名时使用 iid（test_camera_depth 遗漏此分支 → 只认 belly）
        cand = (cfg.get("position") or args.get("position") or args.get("camera_source")
                or self._cfg.get(iid, {}).get("position"))
        if not cand:
            cand = iid if iid in self._positions else self._default_pos
        pos = str(cand).lower()
        return pos if pos in self._positions else self._default_pos

    def _stream_for(self, iid: str) -> "_DepthStream":
        st = self._streams.get(iid)
        if st is None:
            st = _DepthStream(self._node, self._topic(iid))
            self._streams[iid] = st
        return st

    # 框架加载时调用：multiInstance 按实例 start，这里不连、不占相机。
    def start(self):
        if self._node is None:
            print(f"[{CARD}] 无 rclpy/executor，推流不可用（仅登记 tool）", flush=True)

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
        st.start(position, p["board_ip"], int(p["depth_port"]))
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
                        "description": "读取哪一路相机的深度（改此项即热切源）",
                        "scope": "instance",
                        "oneOf": [{"const": p, "title": _POS_TITLE[p]} for p in _VALID_POSITIONS],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"],
                                          "description": "start=连接并推流 / stop=断开并释放相机 / info=查询状态"}},
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
                "format": "sensor_msgs/CompressedImage (jpeg, colorized depth: red=near/cyan=far)",
                "source": f"{pos} @ {p.get('board_ip')}:{p.get('depth_port')} (depth_stream)",
                "connected_to_nano": bool(st and st.connected),
                "frames_published": st.frames if st else 0,
                "topic_out": [{"topic": self._topic(iid), "format": FMT}] if self._node else [],
            }
            if state == "running":
                base["note"] = "streaming colorized depth JPEG; stop to release camera"
            else:
                base["note"] = "start to connect (Nano opens camera on connect); stop releases it"
            return base
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。test_camera_depth_li 不用共享 SDK client(HighState)，故忽略 client。"""
    return CameraDepthPlugin(plugin_config, namespace, executor, client)
