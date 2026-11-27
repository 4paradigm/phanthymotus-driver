"""On-demand, read-only Helios chassis and ROS 2 stream diagnostics."""

import json
import os
import re
import ssl
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from http.client import HTTPException
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, tool


RECOVERY_RECOMMENDATION = "建议先等待片刻后重新查询。若异常持续，请重启机器人后再查询。"
COMPLETION_TIMEOUT = 20


@dataclass
class StreamSample:
    arrivals: list[float] = field(default_factory=list)
    stamps: dict[str, list[int | None]] = field(default_factory=dict)

    def record(self, **stamps):
        self.arrivals.append(time.monotonic())
        for source, stamp in stamps.items():
            self.stamps.setdefault(source, []).append(stamp)


def _stamp_ns(value, scale=1):
    if isinstance(value, str) and value.isascii() and value.isdecimal() and len(value) <= 20:
        value = int(value)
    return value * scale if type(value) is int and value > 0 else None


def record_odometry(sample, raw):
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError):
        return
    if not isinstance(payload, dict) or payload.get("schema") != "motus.odom/1":
        return
    vendor = payload.get("vendor")
    if not isinstance(vendor, dict):
        vendor = {}
    sample.record(
        pose=_stamp_ns(vendor.get("pose_source_stamp_ns")),
        velocity=_stamp_ns(vendor.get("source_stamp_ms"), 1_000_000),
    )


def stream_health(topic, sample, publishers, now, stale_after_s=2.5):
    arrivals = sample.arrivals
    age = now - arrivals[-1] if arrivals else None
    interval = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
    sources = {}
    for name, stamps in sample.stamps.items():
        valid = [(arrival, stamp) for arrival, stamp in zip(arrivals, stamps) if stamp is not None]
        elapsed = (valid[-1][1] - valid[0][1]) / 1_000_000 if len(valid) > 1 else None
        received_elapsed = (valid[-1][0] - valid[0][0]) * 1_000 if len(valid) > 1 else None
        last_change = next(
            (valid[i][0] for i in range(len(valid) - 1, 0, -1) if valid[i][1] != valid[i - 1][1]),
            None,
        )
        if stamps[-1] is None:
            state = "unavailable"
        elif len(valid) < 2:
            state = "waiting"
        elif elapsed <= 0 or last_change is None or now - last_change > stale_after_s:
            state = "stale"
        else:
            state = "running"
        sources[name] = {
            "state": state,
            "stamp_ns": str(stamps[-1]) if stamps[-1] is not None else None,
            "advance_ms": round(elapsed, 1) if elapsed is not None else None,
            "receive_span_ms": round(received_elapsed, 1) if received_elapsed is not None else None,
            "lag_growth_ms": round(received_elapsed - elapsed, 1) if elapsed and elapsed > 0 else None,
        }

    if not arrivals:
        state = "waiting" if publishers else "unavailable"
    elif age > stale_after_s:
        state = "stale"
    else:
        states = {source["state"] for source in sources.values()}
        state = states.pop() if len(states) == 1 else "degraded"
    return {
        "state": state,
        "topic": topic,
        "publishers": publishers,
        "received_frames": len(arrivals),
        "receive_age_ms": round(age * 1000, 1) if age is not None else None,
        "observed_hz": round((len(arrivals) - 1) / interval, 2) if interval > 0 else None,
        "source_timestamps": sources,
    }


def read_link(interface, sysfs=Path("/sys/class/net")):
    if not re.fullmatch(r"[a-zA-Z0-9_.:-]+", interface) or interface in (".", ".."):
        raise ValueError("Invalid chassis network interface")
    root = sysfs / interface

    def read(name):
        try:
            return (root / name).read_text().strip()
        except OSError:
            return None

    carrier = read("carrier")
    speed = read("speed")
    down_count = read("carrier_down_count")
    return {
        "state": {"1": "running", "0": "offline"}.get(carrier, "unavailable"),
        "interface": interface,
        "carrier": carrier == "1" if carrier in ("0", "1") else None,
        "speed_mbps": int(speed) if speed and speed.isdecimal() else None,
        "carrier_down_count": int(down_count) if down_count and down_count.isdecimal() else None,
    }


def _read_tool(port, name, action):
    if not port:
        return {"error": "未配置 MCP 端口"}
    packet = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
              "params": {"name": name, "arguments": {"action": action}}}
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}/mcp", data=json.dumps(packet).encode(),
        headers={"Content-Type": "application/json", "Accept": "application/json, text/event-stream"},
    )
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request, timeout=0.8) as response:
            reply = json.loads(response.read(128 * 1024))
        result = json.loads(reply["result"]["content"][0]["text"])
        if not isinstance(result, dict):
            raise ValueError("无效的 MCP 返回")
        return result
    except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
        return {"error": f"{name} 不可用（{type(exc).__name__}）"}


def chassis_health(data):
    if data.get("error"):
        return {"state": "unavailable", "error": data["error"]}
    level = data.get("chassis_exception_level")
    sdk = data.get("sdk_available")
    worker = data.get("worker_running")
    if data.get("connect_error") or sdk is False:
        state = "error"
    elif isinstance(level, int) and level > 0:
        state = "degraded"
    elif sdk is True and worker and level == 0:
        state = "running"
    elif worker is False:
        state = "idle"
    else:
        state = "waiting"
    return {
        "state": state,
        "sdk_available": sdk,
        "worker_running": worker,
        "reported_state": data.get("state"),
        "connect_error": data.get("connect_error"),
        "exception_level": level,
        "exceptions": data.get("chassis_exceptions"),
    }


class HeliosDriverHealth:
    NAME = "driver_health"

    def __init__(self, config, namespace, ros2):
        self._ros2 = ros2
        self._interface = os.getenv("HELIOS_HEALTH_INTERFACE", config.get("health_interface", "eth0"))
        self._ports = {
            key: int(os.getenv(f"HELIOS_HEALTH_{key.upper()}_PORT", config.get(f"health_{key}_port", port)))
            for key, port in (("base", 15782), ("lidar", 15751), ("odometry", 0))
        }
        if any(not 0 <= port <= 65535 for port in self._ports.values()):
            raise ValueError("Invalid driver health MCP port")
        self._topics = {
            "left_front": f"/{namespace}/lidar/left_front/points",
            "right_rear": f"/{namespace}/lidar/right_rear/points",
            "odometry": f"/{namespace}/state/odom",
        }
        self._state_lock = threading.Lock()
        self._active_id = None
        self._latest = None
        self._worker = None

    def get_tool(self):
        schema = action_schema({
            "status": ([], "启动约 4 秒的健康采样，完成后调用 latest 获取完整结果"),
            "latest": ([], "读取最近完成的健康快照，包含异常项及建议"),
        }, {})
        schema["x-completion"] = {"actions": ["status"], "timeout": COMPLETION_TIMEOUT}
        return tool(
            self.NAME, "actuator",
            "按需查看底盘 SDK、机身网口、双路雷达和里程计的数据状态。",
            schema,
        )

    def start(self):
        pass

    def stop(self):
        with self._state_lock:
            worker = self._worker
        if worker is not None:
            worker.join(timeout=COMPLETION_TIMEOUT)

    def _sample_streams(self):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String

        context = self._ros2.ctx_core
        node = Node("helios_driver_health", context=context)
        executor = SingleThreadedExecutor(context=context)
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        samples = {key: StreamSample() for key in self._topics}

        def lidar_frame(msg, key):
            stamp = msg.header.stamp
            samples[key].record(cloud=_stamp_ns(stamp.sec * 1_000_000_000 + stamp.nanosec))

        try:
            for key in ("left_front", "right_rear"):
                node.create_subscription(PointCloud2, self._topics[key],
                                         lambda msg, key=key: lidar_frame(msg, key), qos)
            node.create_subscription(String, self._topics["odometry"],
                                     lambda msg: record_odometry(samples["odometry"], msg.data), qos)
            executor.add_node(node)
            deadline = time.monotonic() + 4
            while context.ok() and (remaining := deadline - time.monotonic()) > 0:
                executor.spin_once(timeout_sec=min(remaining, 0.15))
            now = time.monotonic()
            return {
                key: stream_health(
                    topic, samples[key], sum(
                        endpoint.topic_type == ("std_msgs/msg/String" if key == "odometry"
                                                else "sensor_msgs/msg/PointCloud2")
                        for endpoint in node.get_publishers_info_by_topic(topic)
                    ), now,
                )
                for key, topic in self._topics.items()
            }
        finally:
            executor.shutdown()
            node.destroy_node()

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return self.get_tool()
        if action == "latest":
            with self._state_lock:
                if self._active_id:
                    return {"state": "sampling", "action_id": self._active_id}
                return self._latest or {"state": "waiting", "message": "请先执行 status。"}
        if action not in (self.NAME, "status"):
            raise ValueError(f"Unsupported driver health action: {action}")

        with self._state_lock:
            if self._active_id:
                return {"state": "busy", "message": "健康检查正在采样，请稍后执行 latest。"}
            action_id = f"helios_health_{uuid4().hex}"
            worker = threading.Thread(target=self._run_status, args=(action_id,),
                                      name="helios-driver-health", daemon=True)
            self._active_id = action_id
            try:
                worker.start()
            except RuntimeError:
                self._active_id = None
                raise
            self._worker = worker
        return {"state": "sampling", "action_id": action_id,
                "message": "采样约 4 秒，完成后执行 latest 读取结果。"}

    def _run_status(self, action_id):
        terminal = "completed"
        try:
            result = self._collect_health()
        except Exception as exc:
            terminal = "error"
            result = {
                "state": "error",
                "health_summary": "健康检查失败",
                "issues": ["health_check error"],
                "error": type(exc).__name__,
                "recommendation": RECOVERY_RECOMMENDATION,
            }
        result = {**result, "action_id": action_id}
        with self._state_lock:
            self._latest = result
            self._active_id = None
        self._post_completion(action_id, terminal, result)

    def _post_completion(self, action_id, status, result):
        endpoint = os.getenv("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        payload = json.dumps({
            "action_id": action_id, "status": status, "result": result,
            "tool": self.NAME, "ts": time.time(),
        }, ensure_ascii=False).encode()
        try:
            ca_file = os.getenv("AGENT_CORE_CA_CERT") or None
            context = ssl.create_default_context(cafile=ca_file)
            context.check_hostname = False
            if ca_file is None:
                context.verify_mode = ssl.CERT_NONE
            request = urllib.request.Request(
                f"{endpoint}/api/acp/complete", data=payload,
                headers={"Content-Type": "application/json"}, method="POST",
            )
            with urllib.request.urlopen(request, timeout=3, context=context):
                pass
        except (OSError, ValueError, HTTPException) as exc:
            print(f"[driver_health] ACP callback failed for {action_id} ({type(exc).__name__})",
                  file=sys.stderr, flush=True)

    def _collect_health(self):
        try:
            streams = self._sample_streams()
        except (ImportError, OSError, RuntimeError) as exc:
            streams = {key: {"state": "error", "topic": topic, "error": type(exc).__name__}
                       for key, topic in self._topics.items()}
        chassis = chassis_health(_read_tool(self._ports["base"], "base_drive", "info"))
        lidar = _read_tool(self._ports["lidar"], "lidar_cloud", "status")
        lidar_sources = lidar.get("sources")
        if not isinstance(lidar_sources, dict):
            lidar_sources = {}
        for key in ("left_front", "right_rear"):
            streams[key]["driver_input"] = lidar.get("error") or lidar_sources.get(key)
        odometry_port = self._ports["odometry"]
        odometry_input = {"state": "disabled"}
        if odometry_port:
            odom = _read_tool(odometry_port, "loco_state", "status")
            odom_data = odom.get("data")
            if not isinstance(odom_data, dict):
                odom_data = {}
            odometry_input = odom.get("error") or {
                key: odom_data.get(key) for key in ("pose_state", "speed_state", "sample_timestamps_ms")
            }
        streams["odometry"]["driver_input"] = odometry_input
        link = read_link(self._interface)
        checks = {"chassis": chassis, "chassis_link": link, **streams}
        issues = [f"{name} {value['state']}" for name, value in checks.items()
                  if value["state"] != "running"]
        result = {
            "state": "degraded" if issues else "running",
            "health_summary": f"{len(checks) - len(issues)}/{len(checks)} 项正常",
            "issues": issues,
            "chassis": chassis,
            "chassis_link": link,
            "streams": streams,
            "timestamp_ms": time.time_ns() // 1_000_000,
        }
        if issues:
            result["recommendation"] = RECOVERY_RECOMMENDATION
        return result
