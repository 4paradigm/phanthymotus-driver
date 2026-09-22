#!/usr/bin/env python3
"""
camera.py — Go1 五机位视觉卡（RGB / 深度 / 点云，三卡合一单文件）。

约定：本文件提供 camera_rgb / camera_depth / camera_pointcloud 三张独立卡。
main.py 按 config.yaml 的三个 key 分别调用对应 make_* 工厂函数；实现仍全部留在本文件。

三张卡的功能共享同一文件的实现：
  camera_rgb        — RGB 去畸变翻正 JPEG      (TCP :9201~9205)
  camera_depth      — 彩色深度 JPEG            (TCP :9101~9105)
  camera_pointcloud — 3D 点云(UInt8MultiArray, agent-core Three.js 渲染) (TCP :9401~9405)

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
    from std_msgs.msg import UInt8MultiArray
    from sensor_msgs.msg import CompressedImage
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False
    _QOS = None

TYPE = "sensor"
_CARD_BY_TYPE = {"rgb": "camera_rgb", "depth": "camera_depth", "pointcloud": "camera_pointcloud"}

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
    "pointcloud": "Go1 五机位点云（3D 渲染, 相机前方视图）— multiInstance, position 下拉框选机位",
}
_TYPE_FMT = {
    # Agent Core 的网页渲染器按 MIME 类型选择 renderer；虽然 ROS2
    # 载体是 CompressedImage，payload 本身是 JPEG，必须向画布声明为
    # image/jpeg，避免二进制帧被当作普通传感器文本而显示为空白。
    "rgb": "image/jpeg",
    "depth": "image/jpeg",
    # 点云走 agent-core 的 Three.js 3D 渲染器（sensor/pointcloud →
    # UInt8MultiArray 载体，payload=[12 LE][N LE][N×xyz f32 LE]）。
    "pointcloud": "sensor/pointcloud",
}
_TYPE_FRAME_ID_SUFFIX = {"rgb": "_rgb", "depth": "_depth", "pointcloud": ""}
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
    """深度流：[4B 长度大端][JPEG payload] → CompressedImage。

    与 _RgbStream 同款的非阻塞批量读取：每轮 drain 接收队列、丢弃过期完整帧、
    只发布最新一帧 —— 深度计算(Nano 端 CPU 立体匹配)速率波动大，若逐帧阻塞收发，
    旧帧会在内核接收队列堆积，画面越看越滞后。
    """

    # 单次最多从内核接收队列取 2 MiB；循环会立即继续 drain，避免持续来帧时长期霸占线程。
    _MAX_DRAIN_BYTES = 2_097_152

    def __init__(self, node: Node, topic: str):
        super().__init__(node, topic)
        self._pub = node.create_publisher(CompressedImage, topic, _QOS) if _HAS_ROS2 else None

    def _loop(self, gen, position, host, port):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=_CONNECT_TIMEOUT)
                s.setblocking(False)
                self.connected = True
                if self._node:
                    self._node.get_logger().info(f"[{position}] 已连 depth_stream {host}:{port}(等第一帧,暖机中)")
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

                    # 丢弃本批次中已过期的完整帧，仅留最后一帧待发布；不完整尾帧保留到下一次 recv。
                    latest = None
                    complete_frames = 0
                    while len(rx) >= 4:
                        n = struct.unpack(">I", rx[:4])[0]
                        if n <= 0 or n > 5_000_000:
                            raise ValueError(f"invalid depth frame length: {n}")
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

                    if self._pub is not None:
                        msg = CompressedImage()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_depth"
                        msg.format = "jpeg"
                        msg.data = latest
                        try:
                            self._pub.publish(msg)
                        except Exception:
                            break
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

    单路输出：UInt8MultiArray，payload = [12 LE][N LE][N×xyz f32 LE]，
    直接对接 agent-core 的 Three.js 点云渲染器（sensor/pointcloud）。
    """

    # 渲染器 MAX_POINTS 上限
    _MAX_POINTS = 40000
    # 单帧点云最大 40000×12+8 ≈ 480KB；单次 drain 上限取 4 MiB(约 8 帧余量)。
    _MAX_DRAIN_BYTES = 4_194_304

    def __init__(self, node: Node, topic: str):
        super().__init__(node, topic)
        self._pub = node.create_publisher(UInt8MultiArray, topic, _QOS) if _HAS_ROS2 else None
        self.last_points = 0

    def _loop(self, gen, position, host, port):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=_CONNECT_TIMEOUT)
                s.setblocking(False)
                self.connected = True
                if self._node:
                    self._node.get_logger().info(f"[{position}] 已连 pointcloud_stream {host}:{port}(等第一帧,暖机中)")
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

                    # 丢弃本批次中已过期的完整帧，仅留最后一帧；不完整尾帧保留到下一次 recv。
                    # 帧结构: [4B 大端 total][4B 大端 numPoints][numPoints×3×f32]。
                    latest = None
                    complete_frames = 0
                    while len(rx) >= 8:
                        total = struct.unpack(">I", rx[:4])[0]
                        if total < 4 or total > 50_000_000:
                            raise ValueError(f"invalid pcl frame length: {total}")
                        end = 4 + total
                        if len(rx) < end:
                            break
                        latest = bytes(rx[4:end])
                        del rx[:end]
                        complete_frames += 1
                    self.frames += complete_frames
                    if latest is None:
                        continue

                    num_points = struct.unpack(">I", latest[:4])[0]
                    xyz_blob = latest[4:]
                    if len(xyz_blob) != num_points * 12:
                        continue

                    if not got_first:
                        got_first = True
                        self._node.get_logger().info(f"[{position}] 首帧到达,进入稳态推流")

                    frame = self._encode_frame(xyz_blob, num_points)
                    if frame is not None and self._pub is not None:
                        msg = UInt8MultiArray()
                        msg.data = frame
                        try:
                            self._pub.publish(msg)
                        except Exception:
                            break
                    self.last_points = num_points
            except Exception as e:  # noqa: BLE001
                if self._node:
                    self._node.get_logger().warn(f"[{position}] pointcloud stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()
                except Exception:
                    pass

    def _encode_frame(self, xyz_blob: bytes, num_points: int) -> bytes | None:
        """相机系 XYZ → agent-core 渲染系打包。

        Nano 送来的是相机光学系（x 右, y 下, z 前）。Go1 相机物理 180° 翻转安装，
        机身系（右/上/前）= (-x_cam, +y_cam, +z_cam)。渲染器把 packet (x,y,z)
        显示为 (y, -z, -x)（q5 已验证此约定），要得到 显示 X=右、Y=上、Z=前，
        需打包 packet = (-前, -右, -上) = (-z_cam, +x_cam, -y_cam)。
        """
        try:
            import numpy as np
        except ImportError:
            return None
        pts = np.frombuffer(xyz_blob, dtype="<f4").reshape(num_points, 3)
        if num_points > self._MAX_POINTS:
            pts = pts[:: (num_points + self._MAX_POINTS - 1) // self._MAX_POINTS]
        out = np.empty((len(pts), 3), dtype="<f4")
        out[:, 0] = -pts[:, 2]   # -z_cam
        out[:, 1] = pts[:, 0]    # +x_cam
        out[:, 2] = -pts[:, 1]   # -y_cam
        return struct.pack("<II", 12, len(out)) + out.tobytes()

# ── Plugin 类 ──────────────────────────────────────

class Plugin:
    """一张固定类型的 Go1 视觉卡；三个实例共享本文件内的流实现。"""

    def __init__(self, plugin_config: dict | None, namespace: str,
                 executor: Any | None, client: Any | None, camera_type: str):
        c = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._type = camera_type
        self._card = _CARD_BY_TYPE[camera_type]

        self._port_key = _TYPE_PORT_KEY[self._type]
        self._default_port = _TYPE_DEFAULT_PORT[self._type]
        self._topic_root = _TYPE_TOPIC_ROOT[self._type]
        self._topic_suffix = _TYPE_TOPIC_SUFFIX[self._type]
        self._frame_id_suffix = _TYPE_FRAME_ID_SUFFIX[self._type]
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
                self._node = Node(f"go1_{self._card}")
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{self._card}] ROS2 不可用: {e}", flush=True)
                self._node = None
        print(f"[{self._card}] 机位就绪：{sorted(self._positions.keys())}（default={self._default_pos}）", flush=True)

    # ── topic 路由 ──

    def _topic(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/{self._topic_root}/{safe}/{self._topic_suffix}"

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
            self._streams[iid] = self._stream_cls(self._node, self._topic(iid))
        return self._streams[iid]

    # ── 生命周期 ──

    def start(self):
        if self._node is None:
            print(f"[{self._card}] 无 rclpy/executor,推流不可用(仅登记 tool)", flush=True)

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
        return [{
            "name": self._card, "type": TYPE, "multiInstance": True,
            "description": self._desc + (" — ROS2" if self._node else " — no rclpy, poll via MCP"),
            "configSchema": {
                "type": "object",
                "properties": {
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
            # 更新位置配置
            pos = self._resolve_pos(iid, args)
            if pos not in self._positions:
                return _err("INVALID_ARGUMENT", f"unknown position {pos!r}; valid: {_VALID_POSITIONS}")
            self._cfg[iid] = {"position": pos}

            # 如果实例正在运行，重启它
            st = self._streams.get(iid)
            if st is not None and st._run and st.position != pos:
                self._start_instance(iid, pos)

            return {"ok": True, "card": self._card, "type": self._type, "position": pos}

        if action == "start":
            return self._start_instance(iid, self._resolve_pos(iid, args))

        if action == "stop":
            st = self._streams.get(iid)
            if st is not None:
                st.stop()
            return {"ok": True, "card": self._card, "action": "stop", "timestamp_ms": _now_ms(),
                    "state": "idle", "position": self._cfg.get(iid, {}).get("position", self._default_pos)}

        if action in ("info", "read", "get", self._card):
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
        if self._type == "pointcloud":
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
        st = self._stream_cls(self._node, self._topic(iid))
        self._streams[iid] = st
        st.start(position, p["board_ip"], int(p.get(self._port_key, self._default_port)))
        topic_out = [{"topic": self._topic(iid), "format": self._fmt}]
        return {"ok": True, "card": self._card, "action": "start", "timestamp_ms": _now_ms(),
                "state": "running", "position": position, "type": self._type,
                "topic_out": topic_out}


def make_camera_rgb(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client, "rgb")


def make_camera_depth(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client, "depth")


def make_camera_pointcloud(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client, "pointcloud")
