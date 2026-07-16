"""
camera_depth.py — Go1 头部双目深度推流卡(sensor)。

深度在头部 Jetson NX 用 UnitreeCameraSDK 计算,由 NX 上常驻的 depth_stream(C++)相机常开循环
出帧并通过 TCP 推送(每帧 = [4字节大端长度][PNG])。本卡在 go1_bundle 容器(py3.10 + rclpy)里
充当 ROS2 桥:后台线程连 NX:9101 收帧 → 发布 sensor_msgs/CompressedImage 到 topic_out,
Agent Core 订阅 → 画布"查看数据流"看到 ~10Hz 深度流。

前提:NX 上 depth_stream 在运行(见 go1_bundle/camera/ 的部署说明);否则本卡持续等待连接。
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

CARD = "camera_depth"
TYPE = "sensor"
TOPIC = "/{ns}/camera/depth"
FMT = "image/png"
NODE = "go1_camera_depth"
DESC = ("Go1 head stereo depth stream (~10Hz, colorized PNG: red=near/cyan=far). Computed on head NX "
        "(depth_stream), bridged to ROS2 sensor_msgs/CompressedImage here.")


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
        self._nx_host = c.get("nx_host", "192.168.123.15")
        self._nx_port = int(c.get("nx_port", 9101))
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._run = False
        self._n = 0
        self._connected = False
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(CompressedImage, self._topic, _QOS)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 depth stream → {self._topic}")
            except Exception as e:  # noqa: BLE001
                print(f"[camera_depth] ROS2 发布不可用: {e}", flush=True)
                self._node = None

    def start(self):
        if self._node is None:
            print("[camera_depth] 无 rclpy/executor,推流不可用(仅登记 tool)", flush=True)
            return
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._run = False

    def _loop(self):
        while self._run:
            try:
                s = socket.create_connection((self._nx_host, self._nx_port), timeout=5)
                self._connected = True
                self._node.get_logger().info(f"已连上 NX depth_stream {self._nx_host}:{self._nx_port}")
            except Exception:
                self._connected = False
                time.sleep(2)
                continue
            try:
                while self._run:
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
                    msg.header.frame_id = "go1_head_depth"
                    msg.format = "png"
                    msg.data = data
                    self._pub.publish(msg)
                    self._n += 1
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(f"depth stream 中断: {e}")
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
                                                           "description": "Query depth stream status"}}},
                "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running" if self._connected else "waiting",
                    "data": {"timestamp_ms": int(time.time() * 1000),
                             "control_level": "HIGHLEVEL",
                             "stream_topic": self._topic,
                             "format": "sensor_msgs/CompressedImage (png)",
                             "connected_to_nx": self._connected,
                             "frames_published": self._n,
                             "source": f"head NX({self._nx_host}:{self._nx_port}) depth_stream",
                             "note": "needs NX depth_stream running; otherwise keeps waiting for connection"},
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
