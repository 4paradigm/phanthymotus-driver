"""
test_camera_pointcloud.py — Go1 双目点云推流卡(sensor,可选机位)。【test 前缀 = 未验收】

camera_depth.py / camera_rgb.py 的思路合体:点云在对应 Nano 板(.13/.14/.15)用 UnitreeCameraSDK 的
getPointCloud 算出,由该板上常驻的 pointcloud_stream(C++)循环出帧并 TCP 推送。本卡在 go1_bundle
容器(py3.10+rclpy)里充当 ROS2 桥:连选定机位的 board_ip:port 收帧 → 组 sensor_msgs/PointCloud2
到固定 topic /{ns}/camera/pointcloud,画布订阅即可看点云。

★ 画布选路:卡的 `position` 配置(front/chin/left/right/belly)决定读哪一路;改配置(config 动作)
  即切换到对应板卡的点云流。一次只读一路(见下"约束")。

协议(每帧):[4字节大端 totalLen][totalLen 字节 payload]
           payload = [4字节大端 numPoints][numPoints × 3 × float32 (小端, x/y/z 米,相机系)]

约束(硬):立体计算吃 Nano CPU + 相机独占(§8.6.1)。5 路分布在 3 块板(.13=front+chin、
  .14=left+right、.15=belly),同板两路不能同时出点云。故本卡"5 选 1、可切换",非"5 路齐开"。
  且与 camera_depth 就头部(.13)相机互斥。
前提:选定机位的板上 pointcloud_stream 正在跑(见 camera/README.md);否则本卡持续等待连接。
真机验收通过后可去掉 test 前缀改名 camera_pointcloud。
"""

from __future__ import annotations

import socket
import struct
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "test_camera_pointcloud"
TYPE = "sensor"
TOPIC = "/{ns}/camera/pointcloud"     # 固定输出 topic;内容随选定 position 变
FMT = "sensor_msgs/PointCloud2"
NODE = "go1_test_camera_pointcloud"

# 机位 → 板卡/设备/点云端口。board_ip/device_id 对齐 camera_rgb 的 _DEFAULT_POSITIONS;
# 点云端口用 94xx，与深度 9101、RGB 图传 92xx、RGB 控制 93xx 全部错开。config.positions 可覆盖。
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "device_id": 1, "pcl_port": 9401},
    "chin":  {"board_ip": "192.168.123.13", "device_id": 0, "pcl_port": 9402},
    "left":  {"board_ip": "192.168.123.14", "device_id": 0, "pcl_port": 9403},
    "right": {"board_ip": "192.168.123.14", "device_id": 1, "pcl_port": 9404},
    "belly": {"board_ip": "192.168.123.15", "device_id": 0, "pcl_port": 9405},
}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

DESC = ("Go1 stereo point cloud (XYZ, meters, camera frame) from a SELECTABLE position. "
        "Computed on the position's Nano (pointcloud_stream via UnitreeCameraSDK getPointCloud), "
        "bridged to ROS2 sensor_msgs/PointCloud2 here. Set `position` (front/chin/left/right/belly) "
        "in card config to pick which camera; changing it switches the source. 【test = 未验收】 "
        "One position at a time (stereo compute is Nano-CPU heavy; same-board pairs can't co-stream); "
        "mutually exclusive with camera_depth on the head (.13).")


def _recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._positions = dict(_DEFAULT_POSITIONS)
        for pos, ov in (c.get("positions") or {}).items():   # config 覆盖(board_ip/device_id/pcl_port)
            if pos in self._positions and isinstance(ov, dict):
                self._positions[pos].update(ov)
        self._position = str(c.get("default_position", "front")).lower()
        if self._position not in self._positions:
            self._position = "front"
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._run = False
        self._gen = 0               # 代际:切换 position 时让旧收流线程自然退出
        self._n = 0                 # 已发布帧数
        self._last_points = 0
        self._connected = False
        self._lock = threading.Lock()
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(PointCloud2, self._topic, _QOS)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 pointcloud stream → {self._topic}")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用: {e}", flush=True)
                self._node = None

    # ── 目标解析 ──
    def _target(self):
        p = self._positions[self._position]
        return p["board_ip"], int(p["pcl_port"])

    # ── 生命周期 ──
    def start(self):
        if self._node is None:
            print(f"[{CARD}] 无 rclpy/executor,推流不可用(仅登记 tool)", flush=True)
            return
        self._restart_bridge()

    def stop(self):
        with self._lock:
            self._run = False
            self._gen += 1
            self._connected = False

    def _restart_bridge(self):
        """(重新)按当前 position 起收流线程;切换机位时调用。"""
        with self._lock:
            self._run = True
            self._gen += 1
            gen = self._gen
            host, port = self._target()
            self._connected = False
        threading.Thread(target=self._loop, args=(gen, host, port), daemon=True).start()

    def _set_position(self, pos: str) -> bool:
        pos = str(pos).lower()
        if pos not in self._positions:
            return False
        changed = pos != self._position
        self._position = pos
        if changed and self._run and self._node is not None:
            self._restart_bridge()   # 切到新板卡:旧线程按 gen 退出,新线程连新目标
        return True

    def _make_msg(self, num_points: int, xyz_blob: bytes) -> "PointCloud2":
        msg = PointCloud2()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = f"go1_{self._position}"   # 携带当前机位
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

    def _loop(self, gen: int, host: str, port: int):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=5)
                self._connected = True
                self._node.get_logger().info(f"[{self._position}] 已连上 pointcloud_stream {host}:{port}")
            except Exception:
                self._connected = False
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
                    self._pub.publish(self._make_msg(num_points, xyz_blob))
                    self._n += 1
                    self._last_points = num_points
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{self._position}] pointcloud stream 中断: {e}")
            finally:
                self._connected = False
                try:
                    s.close()
                except Exception:
                    pass

    def get_tool(self):
        desc = DESC + (f" — → {self._topic}" if self._node else " — no rclpy, poll via MCP")
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object",
                                "properties": {"action": {"type": "string", "enum": ["info"],
                                                           "description": "Query point cloud stream status"}}},
                "configSchema": {"type": "object", "properties": {
                    "position": {"type": "string", "enum": _VALID_POSITIONS,
                                 "description": "读取哪一路相机的点云(改此项即切换源)"}}},
                "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}

    def dispatch(self, action, args):
        # 画布在非系统 action 前会下发 config(位置选择);也允许 start/args 带 position。
        if action == "config":
            cfg = args.get("config") or {}
            pos = cfg.get("position") or args.get("position")
            if pos is not None and not self._set_position(pos):
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": f"unknown position {pos!r}; valid: {_VALID_POSITIONS}"}
            return {"adapter_ok": True, "position": self._position}
        if action == "start":
            pos = (args.get("config") or {}).get("position") or args.get("position")
            if pos is not None and not self._set_position(pos):
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": f"unknown position {pos!r}; valid: {_VALID_POSITIONS}"}
            self.start()
            return {"state": "running", "position": self._position}
        if action == "stop":
            self.stop()
            return {"state": "idle", "position": self._position}
        if action in ("info", "read", "get", CARD):
            host, port = self._target()
            return {"state": "running" if self._connected else "waiting",
                    "data": {"timestamp_ms": int(time.time() * 1000),
                             "control_level": "HIGHLEVEL",
                             "position": self._position,
                             "positions_available": _VALID_POSITIONS,
                             "stream_topic": self._topic,
                             "format": FMT,
                             "connected_to_nx": self._connected,
                             "source": f"{self._position} @ {host}:{port} (pointcloud_stream)",
                             "frames_published": self._n,
                             "last_frame_points": self._last_points,
                             "note": ("selectable 5→1 via config.position, HOT-SWITCH (no restart): switching "
                                      "disconnects the old board's stream and connects the new one, whose "
                                      "pointcloud_stream opens the camera on-connect (~3-4s to first frame) and "
                                      "releases it on-disconnect. One at a time (stereo compute heavy). Each "
                                      "position's pointcloud_stream must be running (idle=no camera held), else waiting")},
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
