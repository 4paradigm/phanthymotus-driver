#!/usr/bin/env python3
"""
camera.py — Go1 五机位视觉卡（RGB / 深度 / 点云，三合一单文件）。

约定：config.yaml 里一个 key "camera"，type 字段选 rgb / depth / pointcloud。
main.py import 本模块后，make_plugin() 根据 type 创建对应的 _Plugin 实例。

三张卡的功能共享同一文件的实现：
  camera type=rgb      — RGB 去畸变翻正 JPEG      (TCP :9201~9205)
  camera type=depth    — 彩色深度 JPEG            (TCP :9101~9105)
  camera type=pointcloud — PointCloud2 + JPEG 俯视投影 (TCP :9401~9405)

架构（与之前各自独立的三文件完全一致）：
  ┌─ Nano 板卡 (.13/.14/.15) ────────────────────┐     ┌─ Pi 驱动容器 (.161) ────────┐
  │ rgb_stream (TCP :9201~9205)                  │     │                             │
  │ depth_stream (TCP :9101~9105)                │     │ CameraPlugin                │
  │ pointcloud_stream (TCP :9401~9405)           │  TCP │  根据 type 实例化           │
  │  · 客户端连上才开相机,断开释放                │◀────▶│  同一物理相机三路互斥       │
  └──────────────────────────────────────────────┘     └─────────────────────────────┘

约束：同一物理相机三路互斥（谁连谁占）；一次只能打开一种类型。
"""

from __future__ import annotations

import socket
import select
import struct
import sys
import threading
import time
from typing import Any

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False
    _QOS = None

CARD = "camera"
TYPE = "sensor"

# ── 机位 + 端口配置 ────────────────────────────────

_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13"},
    "chin":  {"board_ip": "192.168.123.13"},
    "left":  {"board_ip": "192.168.123.14"},
    "right": {"board_ip": "192.168.123.14"},
    "belly": {"board_ip": "192.168.123.15"},
}
_POS_TITLE = {"front": "Front (头部前)", "chin": "Chin (头部下)",
              "left": "Left (侧左)", "right": "Right (侧右)", "belly": "Belly (腹部)"}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

# 各 type 的默认端口
_TYPE_PORT_KEY = {"rgb": "image_port", "depth": "depth_port", "pointcloud": "pcl_port"}
_TYPE_DEFAULT_PORT = {"rgb": 9201, "depth": 9101, "pointcloud": 9401}
_TYPE_TOPIC_ROOT = {"rgb": "vision", "depth": "camera", "pointcloud": "camera"}
_TYPE_TOPIC_SUFFIX = {"rgb": "mono", "depth": "depth", "pointcloud": "pointcloud"}
_TYPE_DESC = {
    "rgb": "Go1 五机位 RGB 相机：去畸变矫正推流，可热切机位，与深度/点云互斥",
    "depth": "Go1 五机位深度流（~10Hz，彩色 JPEG：近红/远青）— multiInstance，position 下拉框选机位",
    "pointcloud": "Go1 五机位点云（XYZ, 米, 相机系）— multiInstance, position 下拉框选机位",
}
_TYPE_FMT = {
    "rgb": "sensor_msgs/CompressedImage (jpeg)",
    "depth": "sensor_msgs/CompressedImage (jpeg, colorized depth)",
    "pointcloud": "sensor_msgs/PointCloud2 (XYZ, meters)",
}
_TYPE_FRAME_ID_SUFFIX = {"rgb": "_rgb", "depth": "_depth", "pointcloud": ""}
_TYPE_HAS_PREVIEW = {"rgb": False, "depth": False, "pointcloud": True}
_VALID_TYPES = list(_TYPE_PORT_KEY.keys())
_TYPE_TITLE = {"rgb": "RGB 彩色", "depth": "深度", "pointcloud": "点云"}

# RGB 使用非阻塞批量读取：首帧可留出 Nano 初始化时间，稳态及时发现断流。
_CONNECT_TIMEOUT = 8.0
_FIRST_FRAME_TIMEOUT = 20.0
_STEADY_TIMEOUT = 8.0

# ── 共享工具函数 ──────────────────────────────────

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

def _resolve_positions_raw(plugin_config: dict | None) -> dict:
    c = plugin_config or {}
    positions = {p: dict(v) for p, v in _DEFAULT_POSITIONS.items()}
    for pos, ov in (c.get("positions") or {}).items():
        if pos in positions and isinstance(ov, dict):
            positions[pos].update(ov)
    return positions

# ── 点云 JPEG 俯视投影 ──────────────────────────

_HAS_PIL = False
try:
    from PIL import Image as _PILImage
    _HAS_PIL = True
except ImportError:
    pass

_PCL_W, _PCL_H = 480, 480
_PCL_R = 2
_PCL_XR, _PCL_YR = 4.0, 3.0
_PCL_ZMIN, _PCL_ZMAX = 0.3, 5.0

def _pcl_jet_t(t):
    t = max(0.0, min(1.0, float(t)))
    r = max(0.0, min(1.0, 1.5 - abs(4 * t - 3)))
    g = max(0.0, min(1.0, 1.5 - abs(4 * t - 2)))
    b = max(0.0, min(1.0, 1.5 - abs(4 * t - 1)))
    return r, g, b

def _pcl_to_jpeg(xyz_blob: bytes, num_points: int) -> bytes | None:
    if not _HAS_PIL or num_points == 0:
        return None
    try:
        import numpy as np
    except ImportError:
        return None
    pts = np.frombuffer(xyz_blob, dtype="<f4").reshape(num_points, 3)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    mask = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & (z > _PCL_ZMIN) & (z < _PCL_ZMAX)
            & (np.abs(x) < _PCL_XR / 2) & (np.abs(y) < _PCL_YR / 2))
    x, y, z = x[mask], y[mask], z[mask]
    img = np.zeros((_PCL_H, _PCL_W, 3), dtype=np.uint8)
    if x.size > 0:
        px = np.clip(((x + _PCL_XR / 2) / _PCL_XR * (_PCL_W - 1)).astype(np.int32), 0, _PCL_W - 1)
        py = np.clip(((y + _PCL_YR / 2) / _PCL_YR * (_PCL_H - 1)).astype(np.int32), 0, _PCL_H - 1)
        t_arr = np.clip((z - _PCL_ZMIN) / (_PCL_ZMAX - _PCL_ZMIN), 0, 1)
        colors = np.zeros((len(px), 3), dtype=np.uint8)
        for i in range(len(px)):
            colors[i] = tuple(int(v * 255) for v in _pcl_jet_t(t_arr[i]))
        for i in range(len(px)):
            x0, y0 = int(px[i]), int(py[i])
            for dy in range(-_PCL_R, _PCL_R + 1):
                for dx in range(-_PCL_R, _PCL_R + 1):
                    if dx * dx + dy * dy <= _PCL_R * _PCL_R:
                        xi, yi = x0 + dx, y0 + dy
                        if 0 <= xi < _PCL_W and 0 <= yi < _PCL_H:
                            img[yi, xi] = colors[i]
    buf = __import__('io').BytesIO()
    _PILImage.fromarray(img).save(buf, format="JPEG", quality=75)
    return buf.getvalue()

# ── TCP 流接收器 ─────────────────────────────────

class _BaseStream:
    """流接收基类：管理 TCP 连接生命周期 + 线程。"""

    def __init__(self, node: Node | None, topic: str):
        self._node = node
        self._topic = topic
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
        self._gen += 1
        self.connected = False


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
        self._last_publish_ms = 0
        self._MIN_INTERVAL_MS = 30       # 最多发布约 30fps
        # 单次最多从内核接收队列取 1 MiB；循环会立即继续 drain，避免在持续来帧时长期霸占线程。
        self._MAX_DRAIN_BYTES = 1_048_576

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
                # 使用非阻塞批量读取：每轮解析所有已完整帧，只发布最新一帧。
                # TCP 是有序字节流；若逐帧 recv 再按时间丢弃，旧 JPEG 会堆在内核接收队列，
                # 画面就会越看越滞后。保留未完整的数据，下一轮继续拼帧。
                s.setblocking(False)
                self.connected = True
                self._node.get_logger().info(f"[{position}] 已连上 rgb_stream {host}:{port}(等第一帧,暖机中~5-6s)")
            except Exception:
                self.connected = False
                time.sleep(2)
                continue
            try:
                got_first = False
                rx = bytearray()
                while self._run and gen == self._gen:
                    timeout = _STEADY_TIMEOUT if got_first else _FIRST_FRAME_TIMEOUT
                    readable, _, _ = select.select([s], [], [], timeout)
                    if not readable:
                        raise TimeoutError("timed out")

                    received = 0
                    peer_closed = False
                    while received < self._MAX_DRAIN_BYTES:
                        try:
                            chunk = s.recv(min(65_536, self._MAX_DRAIN_BYTES - received))
                        except BlockingIOError:
                            break
                        if not chunk:
                            peer_closed = True
                            break
                        rx.extend(chunk)
                        received += len(chunk)
                    if peer_closed:
                        break

                    # 丢弃本批次中已过期的完整帧，仅留下最后一帧待发布；不完整尾帧保留到下一次 recv。
                    latest = None
                    complete_frames = 0
                    while len(rx) >= 4:
                        n = struct.unpack(">I", rx[:4])[0]
                        if n <= 0 or n > 5_000_000:
                            raise ValueError(f"invalid JPEG frame length: {n}")
                        end = 4 + n
                        if len(rx) < end:
                            break
                        latest = bytes(rx[4:end])
                        del rx[:end]
                        complete_frames += 1
                    self.frames += complete_frames
                    if latest is None:
                        continue

                    if not got_first:
                        got_first = True
                        self._node.get_logger().info(f"[{position}] 首帧到达,进入稳态推流")

                    # 发布节流只影响 ROS2 输出；接收端仍持续 drain TCP 队列，避免旧帧重新积压。
                    now_ms = int(time.time() * 1000)
                    if now_ms - self._last_publish_ms < self._MIN_INTERVAL_MS:
                        continue

                    if self._pub is not None:
                        msg = CompressedImage()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_rgb"
                        msg.format = "jpeg"
                        msg.data = latest
                        try:
                            self._pub.publish(msg)
                            self._last_publish_ms = now_ms
                        except Exception:
                            break
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{position}] rgb stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()          # 断开 → rgb_stream _exit(0) 释放相机
                except Exception:
                    pass

class _DepthStream(_BaseStream):
    """深度流：[4B 长度大端][JPEG payload] → CompressedImage。"""

    def __init__(self, node: Node, topic: str):
        super().__init__(node, topic)
        self._pub = node.create_publisher(CompressedImage, topic, _QOS) if _HAS_ROS2 else None

    def _loop(self, gen, position, host, port):
        t_connect, t_first, t_steady = 8.0, 15.0, 8.0
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=t_connect)
                s.settimeout(t_first)
                self.connected = True
                if self._node:
                    self._node.get_logger().info(f"[{position}] 已连 depth_stream {host}:{port}")
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
                        s.settimeout(t_steady)
                        if self._node:
                            self._node.get_logger().info(f"[{position}] 首帧到达")
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
                if self._node:
                    self._node.get_logger().warn(f"[{position}] depth stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()
                except Exception:
                    pass


class _PclStream(_BaseStream):
    """点云流：[4B total][total payload] → [4B numPoints][N×3×float32]。

    同时发布：PointCloud2 + JPEG 俯视投影。
    """

    def __init__(self, node: Node, topic_pcl: str, topic_preview: str):
        super().__init__(node, topic_preview)
        self._pub_pcl = node.create_publisher(PointCloud2, topic_pcl, _QOS) if _HAS_ROS2 else None
        self._pub_jpeg = node.create_publisher(CompressedImage, topic_preview, _QOS) if (_HAS_ROS2 and _HAS_PIL) else None
        self.last_points = 0

    def _make_pcl_msg(self, num_points: int, xyz_blob: bytes):
        msg = PointCloud2()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = f"go1_{self.position}"
        msg.height = 1
        msg.width = num_points
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * num_points
        msg.data = xyz_blob
        msg.is_dense = True
        return msg

    def _loop(self, gen, position, host, port):
        t_connect, t_first, t_steady = 8.0, 15.0, 8.0
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=t_connect)
                s.settimeout(t_first)
                self.connected = True
                if self._node:
                    self._node.get_logger().info(f"[{position}] 已连 pointcloud_stream {host}:{port}")
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
                    total = struct.unpack(">I", hdr)[0]
                    if total < 4 or total > 50_000_000:
                        break
                    payload = _recvall(s, total)
                    if payload is None:
                        break
                    num_points = struct.unpack(">I", payload[:4])[0]
                    xyz_blob = payload[4:]
                    if len(xyz_blob) != num_points * 12:
                        continue
                    if self._pub_pcl is not None:
                        self._pub_pcl.publish(self._make_pcl_msg(num_points, xyz_blob))
                    if self._pub_jpeg is not None:
                        jpeg = _pcl_to_jpeg(xyz_blob, num_points)
                        if jpeg:
                            pmsg = CompressedImage()
                            pmsg.header.stamp = self._node.get_clock().now().to_msg()
                            pmsg.header.frame_id = f"go1_{position}_pcl_preview"
                            pmsg.format = "jpeg"
                            pmsg.data = jpeg
                            try:
                                self._pub_jpeg.publish(pmsg)
                            except Exception:
                                pass
                    self.frames += 1
                    self.last_points = num_points
                    if not got_first:
                        got_first = True
                        s.settimeout(t_steady)
                        if self._node:
                            self._node.get_logger().info(f"[{position}] 首帧到达")
            except Exception as e:  # noqa: BLE001
                if self._node:
                    self._node.get_logger().warn(f"[{position}] pointcloud stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()
                except Exception:
                    pass

# ── Plugin 类 ──────────────────────────────────────

class Plugin:
    """Go1 视觉扩展卡（RGB / 深度 / 点云三合一）。

    根据 config.camera.type 选择功能模式。
    """

    def __init__(self, plugin_config: dict | None, namespace: str,
                 executor: Any | None, client: Any | None):
        c = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._type = str(c.get("type", "rgb")).lower()
        if self._type not in _TYPE_PORT_KEY:
            self._type = "rgb"

        self._port_key = _TYPE_PORT_KEY[self._type]
        self._default_port = _TYPE_DEFAULT_PORT[self._type]
        self._topic_root = _TYPE_TOPIC_ROOT[self._type]
        self._topic_suffix = _TYPE_TOPIC_SUFFIX[self._type]
        self._frame_id_suffix = _TYPE_FRAME_ID_SUFFIX[self._type]
        self._has_preview = _TYPE_HAS_PREVIEW[self._type]
        self._desc = _TYPE_DESC[self._type]
        self._fmt = _TYPE_FMT[self._type]

        # 选择流类
        _stream_cls = {"rgb": _RgbStream, "depth": _DepthStream, "pointcloud": _PclStream}[self._type]
        self._stream_cls = _stream_cls

        self._positions = _resolve_positions_raw(plugin_config)
        for p in _VALID_POSITIONS:
            if p in self._positions and self._port_key not in self._positions[p]:
                self._positions[p][self._port_key] = self._default_port
        self._default_pos = str(c.get("default_position", "front")).lower()
        if self._default_pos not in self._positions:
            self._default_pos = "front"
        self._node = None
        self._streams: dict[str, _BaseStream] = {}
        self._cfg: dict[str, dict] = {}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(f"go1_camera_{self._type}")
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None
        print(f"[{CARD}][{self._type}] 机位就绪：{sorted(self._positions.keys())}（default={self._default_pos}）", flush=True)

    # ── topic 路由 ──

    def _topic(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/{self._topic_root}/{safe}/{self._topic_suffix}"

    def _topic_preview(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/{self._topic_root}/{safe}/{self._topic_suffix}_preview"

    # ── 机位解析 ──

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        cand = (cfg.get("position") or args.get("position") or args.get("camera_source")
                or self._cfg.get(iid, {}).get("position"))
        if not cand:
            cand = iid if iid in self._positions else self._default_pos
        pos = str(cand).lower()
        return pos if pos in self._positions else self._default_pos

    # ── 实例管理 ──

    def _stream_for(self, iid: str) -> _BaseStream:
        if iid not in self._streams:
            if self._has_preview:
                self._streams[iid] = _PclStream(self._node, self._topic(iid), self._topic_preview(iid))
            else:
                self._streams[iid] = self._stream_cls(self._node, self._topic(iid))
        return self._streams[iid]

    # ── 生命周期 ──

    def start(self):
        if self._node is None:
            print(f"[{CARD}] 无 rclpy/executor,推流不可用(仅登记 tool)", flush=True)

    def stop(self):
        for st in self._streams.values():
            try:
                st.stop()
            except Exception:
                pass

    # ── tool 注册 ──

    def get_tool(self) -> dict:
        return self.get_tools()[0]

    def get_tools(self) -> list:
        topic_out = []
        if self._node:
            topic_out.append({"topic": self._topic("default"), "format": self._fmt})
            if self._has_preview and _HAS_PIL:
                topic_out.append({"topic": self._topic_preview("default"), "format": "image/jpeg"})
        return [{
            "name": CARD, "type": TYPE, "multiInstance": True,
            "description": self._desc + (" — ROS2" if self._node else " — no rclpy, poll via MCP"),
            "configSchema": {
                "type": "object",
                "properties": {
                    "type": {
                        "type": "string",
                        "description": "相机模式：rgb / depth / pointcloud",
                        "scope": "card",
                        "oneOf": [{"const": t, "title": _TYPE_TITLE[t]} for t in _VALID_TYPES],
                    },
                    "position": {
                        "type": "string",
                        "description": "读取哪一路相机（改此项即热切源）",
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
            "topic_out": topic_out,
        }]

    # ── dispatch ──

    def dispatch(self, action: str, args: dict) -> dict | None:
        iid = args.get("instance_id") or "default"

        if action == "config":
            config = args.get("config", {})

            # 检查是否需要切换类型
            new_type = config.get("type")
            if new_type and new_type != self._type:
                if new_type not in _TYPE_PORT_KEY:
                    return _err("INVALID_ARGUMENT", f"unknown type {new_type!r}; valid: {_VALID_TYPES}")

                # 更新类型相关配置
                self._type = new_type
                self._port_key = _TYPE_PORT_KEY[self._type]
                self._default_port = _TYPE_DEFAULT_PORT[self._type]
                self._topic_root = _TYPE_TOPIC_ROOT[self._type]
                self._topic_suffix = _TYPE_TOPIC_SUFFIX[self._type]
                self._frame_id_suffix = _TYPE_FRAME_ID_SUFFIX[self._type]
                self._has_preview = _TYPE_HAS_PREVIEW[self._type]
                self._desc = _TYPE_DESC[self._type]
                self._fmt = _TYPE_FMT[self._type]

                # 更新流类
                self._stream_cls = {"rgb": _RgbStream, "depth": _DepthStream, "pointcloud": _PclStream}[self._type]

                # 重启所有现有实例
                for existing_iid in list(self._streams.keys()):
                    st = self._streams.pop(existing_iid, None)
                    if st is not None:
                        st.stop()

                print(f"[{CARD}] 切换到 {self._type} 模式", flush=True)

            # 更新位置配置
            pos = self._resolve_pos(iid, args)
            if pos not in self._positions:
                return _err("INVALID_ARGUMENT", f"unknown position {pos!r}; valid: {_VALID_POSITIONS}")
            self._cfg[iid] = {"position": pos}

            # 如果实例正在运行，重启它
            st = self._streams.get(iid)
            if st is not None and st._run and st.position != pos:
                self._start_instance(iid, pos)

            return {"ok": True, "type": self._type, "position": pos}

        if action == "start":
            return self._start_instance(iid, self._resolve_pos(iid, args))

        if action == "stop":
            st = self._streams.get(iid)
            if st is not None:
                st.stop()
            return {"ok": True, "card": CARD, "action": "stop", "timestamp_ms": _now_ms(),
                    "state": "idle", "position": self._cfg.get(iid, {}).get("position", self._default_pos)}

        if action in ("info", "read", "get", CARD):
            return self._do_info(iid)
        return None

    def _do_info(self, iid: str) -> dict:
        pos = self._resolve_pos(iid, args := {})
        p = self._positions.get(pos, {})
        st = self._streams.get(iid)
        state = "running" if (st and st.connected) else ("waiting" if st and st._run else "idle")
        topic_out = []
        if self._node:
            topic_out.append({"topic": self._topic(iid), "format": self._fmt})
            if self._has_preview and _HAS_PIL:
                topic_out.append({"topic": self._topic_preview(iid), "format": "image/jpeg"})
        base = {
            "state": state, "position": pos,
            "positions_available": _VALID_POSITIONS,
            "type": self._type,
            "format": self._fmt,
            "source": f"{pos} @ {p.get('board_ip')}:{p.get(self._port_key)} ({self._type}_stream)",
            "connected_to_nano": bool(st and st.connected) if st else False,
            "frames_published": st.frames if st else 0,
            "topic_out": topic_out,
        }
        if self._has_preview:
            base["last_frame_points"] = st.last_points if st else 0
        base["note"] = "streaming; stop to release camera" if state == "running" else "start to connect; stop releases it"
        return base

    def _start_instance(self, iid: str, position: str) -> dict:
        if position not in self._positions:
            return _err("INVALID_ARGUMENT", f"unknown position {position!r}; valid: {_VALID_POSITIONS}")
        if self._node is None:
            return _err("COMMUNICATION_ERROR", "no rclpy/executor")
        p = self._positions[position]
        # 清理旧实例
        if iid in self._streams:
            self._streams.pop(iid, None).stop()
        # 创建新实例
        if self._has_preview:
            st = _PclStream(self._node, self._topic(iid), self._topic_preview(iid))
        else:
            st = self._stream_cls(self._node, self._topic(iid))
        self._streams[iid] = st
        st.start(position, p["board_ip"], int(p.get(self._port_key, self._default_port)))
        topic_out = [{"topic": self._topic(iid), "format": self._fmt}]
        if self._has_preview and _HAS_PIL:
            topic_out.append({"topic": self._topic_preview(iid), "format": "image/jpeg"})
        return {"ok": True, "card": CARD, "action": "start", "timestamp_ms": _now_ms(),
                "state": "running", "position": position, "type": self._type,
                "topic_out": topic_out}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
