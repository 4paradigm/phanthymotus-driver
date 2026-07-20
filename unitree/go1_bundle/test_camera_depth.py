"""
camera_depth.py — Go1 双目深度推流卡(sensor,可选机位·multiInstance,按需开/停)。【test 前缀 = 未验收,验收后改名 camera_depth】

深度在对应 Nano 板用 UnitreeCameraSDK 的 getDepthFrame 算出,由该板常驻的 depth_stream(C++)
按 device_id 开相机、循环出帧(彩色深度图 JPEG,[4字节大端长度][JPEG])并 TCP 推送:
**客户端连上才开相机、断开就释放**(见 camera/depth_stream.cc)。本卡在 go1_bundle 容器(py3.10+rclpy)
充当 ROS2 桥:卡在画布上 start 时才连选定机位的 board_ip:port 收帧 → 发布 sensor_msgs/CompressedImage
(format=jpeg,对齐 camera_rgb / 画布只渲染 jpeg)到实例专属 topic;stop 时断开 → 相机立即释放。

★ 画布两层设置(对齐 test_camera_pointcloud / dji camera_stream):
  · 公共(拖入前):config.yaml 的 default_position —— 驱动级默认机位。
  · 个人(拖入后):卡的 configSchema.position 下拉框(scope:instance)。平台下发 config 动作,
    本卡按 instance_id 缓存;运行中改机位 → 断旧连新热重启(免重启热切)。

约束:深度也吃 device 独占 + 立体计算 → 一路一路来;与点云指向同一相机时互斥
  (depth_stream 开相机时 fuser 顶掉占用者)。切换/首连时相机 SDK 初始化约 3~4s,期间无帧属正常。
前提:该机位的 depth_stream 服务在运行(nano_bootstrap.sh 首启自动编译+装成空闲 systemd 服务)。
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

CARD = "test_camera_depth"
TYPE = "sensor"
FMT = "image/jpeg"

# 机位 → 板卡 IP / 深度端口(与 nano_bootstrap.sh DEPTH_ROWS 对齐;device_id 在 Nano 侧服务里定)。
# 端口 91xx 与点云 94xx、RGB 图传 92xx、RGB 控制 93xx 全部错开。config.positions 可覆盖 board_ip/depth_port。
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

DESC = ("Go1 stereo depth stream (~10Hz, colorized JPEG: red=near/cyan=far) — multiInstance, pick `position` "
        "per card. Computed on the position's Nano (depth_stream via getDepthFrame, opens camera only while "
        "connected), bridged to ROS2 sensor_msgs/CompressedImage here. start → connect & stream; stop → "
        "disconnect & release the camera. Change the position dropdown to hot-switch source (~3-4s to first "
        "frame). One position at a time (stereo heavy); shares each camera with point cloud (mutually exclusive).")


def _recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class _DepthStream:
    """单实例深度桥:连某机位 board_ip:port 收帧([4字节大端长度][JPEG])→ 发布 CompressedImage 到实例专属 topic。"""

    def __init__(self, node: "Node", topic: str):
        self._node = node
        self._topic = topic
        self._pub = node.create_publisher(CompressedImage, topic, _QOS)
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
        self._gen += 1        # 让在跑的 loop 线程退出并断开 → Nano 侧相机释放
        self.connected = False

    def _loop(self, gen, position, host, port):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=5)
                self.connected = True
                self._node.get_logger().info(f"[{position}] 已连上 depth_stream {host}:{port}")
            except Exception:
                self.connected = False
                time.sleep(2)
                continue
            try:
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
                    msg = CompressedImage()
                    msg.header.stamp = self._node.get_clock().now().to_msg()
                    msg.header.frame_id = f"go1_{position}_depth"
                    msg.format = "jpeg"
                    msg.data = data
                    self._pub.publish(msg)
                    self.frames += 1
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"[{position}] depth stream 中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()          # 断开 → depth_stream 释放相机
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
        self._default_pos = str(c.get("default_position", "belly")).lower()
        if self._default_pos not in self._positions:
            self._default_pos = "belly"
        self._node = None
        self._streams: dict = {}          # instance_id -> _DepthStream
        self._cfg: dict = {}              # instance_id -> {"position": ...}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_test_camera_depth")
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None

    def _topic(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/depth"

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        return (cfg.get("position") or args.get("position")
                or self._cfg.get(iid, {}).get("position") or self._default_pos).lower()

    def _stream_for(self, iid: str) -> "_DepthStream":
        st = self._streams.get(iid)
        if st is None:
            st = _DepthStream(self._node, self._topic(iid))
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
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": f"unknown position {position!r}; valid: {_VALID_POSITIONS}"}
        if self._node is None:
            return {"state": "error", "message": "no rclpy/executor"}
        p = self._positions[position]
        st = self._stream_for(iid)
        st.stop()
        st.start(position, p["board_ip"], int(p["depth_port"]))
        return {"state": "running", "position": position,
                "topic_out": [{"topic": self._topic(iid), "format": FMT}]}

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": True,
            "description": DESC + (" — ROS2 CompressedImage" if self._node else " — no rclpy, poll via MCP"),
            "configSchema": {
                "type": "object",
                "properties": {
                    "position": {
                        "type": "string",
                        "description": "读取哪一路相机的深度(改此项即热切源)",
                        "scope": "instance",
                        "oneOf": [{"const": p, "title": _POS_TITLE[p]} for p in _VALID_POSITIONS],
                    },
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"],
                                          "description": "start=连并推流 / stop=断开并释放相机 / info=查状态"}},
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
                             "format": "sensor_msgs/CompressedImage (jpeg)",
                             "source": f"{pos} @ {p.get('board_ip')}:{p.get('depth_port')} (depth_stream)",
                             "connected_to_nx": bool(st and st.connected),
                             "frames_published": st.frames if st else 0,
                             "note": ("start to connect (Nano opens camera on connect); stop releases it; "
                                      "hot-switch via config.position; one at a time (stereo heavy); "
                                      "shares each camera with point cloud")},
                    "topic_out": ([{"topic": self._topic(iid), "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
