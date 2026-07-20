"""
test_camera_pointcloud.py — Go1 双目点云推流卡(sensor,可选机位·multiInstance)。【test 前缀 = 未验收】

点云在对应 Nano 板用 UnitreeCameraSDK 的 getPointCloud 算出,由该板常驻的 pointcloud_stream(C++)
按 device_id 开相机、循环出帧并 TCP 推送。本卡在 go1_bundle 容器(py3.10+rclpy)充当 ROS2 桥:
连选定机位的 board_ip:port 收帧 → 同时发布两个 topic:
  · /{ns}/camera/{iid}/pointcloud  (sensor_msgs/PointCloud2) — 原始点云,供 agent-core 空间计算
  · /{ns}/camera/{iid}/pointcloud/preview (image/jpeg)       — XZ 平面俯视投影伪彩色图,供画布渲染

俯视投影说明:将相机坐标系(x=右,y=下,z=前)的所有点投影到 XZ 水平面,z 值(前向距离)映射为
  jet 伪彩色(蓝=近,红=远)。效果类似从正上方俯视看机器人前方地形/障碍分布。

★ 画布两层设置(对齐 dji camera_stream):
  · 公共(拖入前):config.yaml 的 default_position —— 驱动级默认机位。
  · 个人(拖入后):卡的 configSchema.position 下拉框(scope:instance)。平台下发 config 动作,
    本卡按 instance_id 缓存;运行中改机位 → 断旧连新热重启(免重启热切)。

协议(每帧):[4字节大端 totalLen][totalLen 字节 payload]
           payload = [4字节大端 numPoints][numPoints × 3 × float32 (小端, x/y/z 米,相机系)]

约束:立体计算吃 Nano CPU + device 独占 → 一路一路来;同板两路(front/chin、left/right)不能并流;
  头部/腹部与 depth_stream 就同一 device 互斥(pointcloud_stream 开相机时会 fuser 顶掉占用者)。
  切换/首连时相机 SDK 初始化约 3~4s,期间无帧属正常。验收后去掉 test 前缀改名 camera_pointcloud。
"""

from __future__ import annotations

import io
import socket
import struct
import threading
import time

import numpy as np

try:
    from PIL import Image as _PILImage
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

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

CARD = "test_camera_pointcloud"
TYPE = "sensor"
FMT_PCL = "sensor_msgs/PointCloud2"
FMT_JPEG = "image/jpeg"

# 投影参数（相机自身视角：x=左右, y=上下, z=前向距离→颜色）
_IMG_W = 480
_IMG_H = 480
_DOT_R = 2       # 每个点绘制半径(像素)
_X_RANGE = 4.0   # 左右各 2m
_Y_RANGE = 3.0   # 上下各 1.5m
_Z_MIN   = 0.3   # 最近有效距离(m)
_Z_MAX   = 5.0   # 最远有效距离(m)


def _jet_colormap(t: np.ndarray) -> np.ndarray:
    """t: [0,1] float array → RGB uint8, jet 伪彩色(蓝=近/0, 红=远/1)。"""
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)


def _draw_dots(img: np.ndarray, px: np.ndarray, py: np.ndarray, colors: np.ndarray):
    """在 img 上把每个点画成半径 _DOT_R 的小圆。"""
    r = _DOT_R
    h, w = img.shape[:2]
    for i in range(len(px)):
        x0, y0 = int(px[i]), int(py[i])
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    xi, yi = x0 + dx, y0 + dy
                    if 0 <= xi < w and 0 <= yi < h:
                        img[yi, xi] = colors[i]


def _pcl_to_jpeg(xyz_blob: bytes, num_points: int, position: str) -> bytes | None:
    """XYZ 点云 → 相机视角投影 JPEG（x=左右, y=上下, z→颜色，各机位统一）。"""
    if not _HAS_PIL or num_points == 0:
        return None
    pts = np.frombuffer(xyz_blob, dtype="<f4").reshape(num_points, 3)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    mask = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & (z > _Z_MIN) & (z < _Z_MAX)
            & (np.abs(x) < _X_RANGE / 2)
            & (np.abs(y) < _Y_RANGE / 2))
    x, y, z = x[mask], y[mask], z[mask]

    img = np.zeros((_IMG_H, _IMG_W, 3), dtype=np.uint8)
    if x.size > 0:
        px = np.clip(((x + _X_RANGE / 2) / _X_RANGE * (_IMG_W - 1)).astype(np.int32), 0, _IMG_W - 1)
        py = np.clip(((y + _Y_RANGE / 2) / _Y_RANGE * (_IMG_H - 1)).astype(np.int32), 0, _IMG_H - 1)
        t = np.clip((z - _Z_MIN) / (_Z_MAX - _Z_MIN), 0, 1)
        _draw_dots(img, px, py, _jet_colormap(t))

    buf = io.BytesIO()
    _PILImage.fromarray(img).save(buf, format="JPEG", quality=75)
    return buf.getvalue()

# 机位 → 板卡 IP / 点云端口(与 nano_bootstrap.sh PCL_ROWS 对齐;device_id 在 Nano 侧服务里定)。
# 端口 94xx 与深度 9101、RGB 图传 92xx、RGB 控制 93xx 全部错开。config.positions 可覆盖 board_ip/pcl_port。
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "pcl_port": 9401},
    "chin":  {"board_ip": "192.168.123.13", "pcl_port": 9402},
    "left":  {"board_ip": "192.168.123.14", "pcl_port": 9403},
    "right": {"board_ip": "192.168.123.14", "pcl_port": 9404},
    "belly": {"board_ip": "192.168.123.15", "pcl_port": 9405},
}
_POS_TITLE = {"front": "Front (头部前 dev1)", "chin": "Chin (头部下 dev0)",
              "left": "Left (侧左 dev0)", "right": "Right (侧右 dev1)", "belly": "Belly (腹部 dev0)"}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

DESC = ("Go1 stereo point cloud (XYZ, meters, camera frame) — multiInstance, pick `position` per card. "
        "Computed on the position's Nano (pointcloud_stream via getPointCloud), bridged to ROS2 PointCloud2 here. "
        "Change the position dropdown to hot-switch source (~3-4s to first frame). 【test = 未验收】 "
        "One position at a time (stereo compute is Nano-CPU heavy); head/belly share the camera with depth.")


def _recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class _PclStream:
    """单实例点云桥:连某机位 board_ip:port 收帧 → 同时发布 PointCloud2 和俯视预览 JPEG。"""

    def __init__(self, node: "Node", topic_pcl: str, topic_preview: str):
        self._node = node
        self._topic_pcl = topic_pcl
        self._topic_preview = topic_preview
        self._pub_pcl = node.create_publisher(PointCloud2, topic_pcl, _QOS)
        self._pub_preview = node.create_publisher(CompressedImage, topic_preview, _QOS) if _HAS_PIL else None
        self._run = False
        self._gen = 0
        self.connected = False
        self.frames = 0
        self.last_points = 0
        self.position = None

    def start(self, position: str, host: str, port: int):
        with threading.Lock():
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
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=5)
                s.settimeout(15)  # 相机 SDK 初始化需 5~7s 才出第一帧,给足等待时间
                self.connected = True
                self._node.get_logger().info(f"[{position}] 已连上 pointcloud_stream {host}:{port}")
            except Exception:
                self.connected = False
                time.sleep(2)
                continue
            try:
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
                    self._pub_pcl.publish(self._make_pcl_msg(num_points, xyz_blob))
                    jpeg = _pcl_to_jpeg(xyz_blob, num_points, position)
                    if jpeg is not None and self._pub_preview is not None:
                        preview_msg = CompressedImage()
                        preview_msg.header.stamp = self._node.get_clock().now().to_msg()
                        preview_msg.header.frame_id = f"go1_{self.position}_pcl_preview"
                        preview_msg.format = "jpeg"
                        preview_msg.data = jpeg
                        self._pub_preview.publish(preview_msg)
                    self.frames += 1
                    self.last_points = num_points
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{position}] pointcloud stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()
                except Exception:
                    pass


class Plugin:
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
        self._streams: dict = {}          # instance_id -> _PclStream
        self._cfg: dict = {}              # instance_id -> {"position": ...}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_test_camera_pointcloud")
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None

    def _topic_pcl(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/pointcloud"

    def _topic_preview(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/pointcloud/preview"

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        return (cfg.get("position") or args.get("position")
                or self._cfg.get(iid, {}).get("position") or self._default_pos).lower()

    def _stream_for(self, iid: str) -> "_PclStream":
        st = self._streams.get(iid)
        if st is None:
            st = _PclStream(self._node, self._topic_pcl(iid), self._topic_preview(iid))
            self._streams[iid] = st
        return st

    def start(self):
        pass  # multiInstance:按实例 start

    def stop(self):
        for st in self._streams.values():
            try:
                st.stop()
            except Exception:
                pass

    def _start_instance(self, iid: str, position: str) -> dict:
        if position not in self._positions:
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": f"unknown position {position!r}; valid: {_VALID_POSITIONS}"}
        if self._node is None:
            return {"state": "error", "message": "no rclpy/executor"}
        p = self._positions[position]
        st = self._stream_for(iid)
        st.stop()
        st.start(position, p["board_ip"], int(p["pcl_port"]))
        topic_out = [{"topic": self._topic_pcl(iid), "format": FMT_PCL}]
        if _HAS_PIL:
            topic_out.append({"topic": self._topic_preview(iid), "format": FMT_JPEG})
        return {"state": "running", "position": position, "topic_out": topic_out}

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": True,
            "description": DESC + (" — ROS2 PointCloud2 + JPEG preview" if self._node else " — no rclpy, poll via MCP"),
            "configSchema": {
                "type": "object",
                "properties": {
                    "position": {
                        "type": "string",
                        "description": "读取哪一路相机的点云(改此项即热切源)",
                        "scope": "instance",
                        "oneOf": [{"const": p, "title": _POS_TITLE[p]} for p in _VALID_POSITIONS],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"]}},
                "required": ["action"],
            },
            "topic_out": [],
        }

    def dispatch(self, action, args):
        iid = args.get("instance_id") or "default"

        if action == "config":
            pos = self._resolve_pos(iid, args)
            if pos not in self._positions:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": f"unknown position {pos!r}; valid: {_VALID_POSITIONS}"}
            self._cfg[iid] = {"position": pos}
            st = self._streams.get(iid)
            if st is not None and st._run and st.position != pos:   # 运行中改机位 → 热重启
                self._start_instance(iid, pos)
            return {"ok": True, "position": pos}

        if action == "start":
            return self._start_instance(iid, self._resolve_pos(iid, args))

        if action == "stop":
            st = self._streams.get(iid)
            if st is not None:
                st.stop()
            return {"state": "idle", "position": self._cfg.get(iid, {}).get("position", self._default_pos)}

        if action in ("info", "read", "get", CARD):
            pos = self._resolve_pos(iid, args)
            p = self._positions.get(pos, {})
            st = self._streams.get(iid)
            return {"state": "running" if (st and st.connected) else ("waiting" if st and st._run else "idle"),
                    "data": {"timestamp_ms": int(time.time() * 1000),
                             "control_level": "HIGHLEVEL",
                             "position": pos,
                             "positions_available": _VALID_POSITIONS,
                             "source": f"{pos} @ {p.get('board_ip')}:{p.get('pcl_port')} (pointcloud_stream)",
                             "connected_to_nx": bool(st and st.connected),
                             "frames_published": st.frames if st else 0,
                             "last_frame_points": st.last_points if st else 0,
                             "note": ("hot-switch via config.position; one at a time (stereo heavy); "
                                      "needs that position's pointcloud_stream service running")},
                    "topic_out": ([{"topic": self._topic_pcl(iid), "format": FMT_PCL},
                                   {"topic": self._topic_preview(iid), "format": FMT_JPEG}]
                                  if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
