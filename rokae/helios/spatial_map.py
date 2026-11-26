"""Helios map and station sensor. The SDK worker runs only while the card is active."""

import json
import os
from pathlib import Path
import subprocess
import threading
import time

from common.vendor_runtime import tool


class HeliosSpatialMap:
    PREFIX = "spatial_map"

    def __init__(self, config, namespace="rokae_helios", ros2=None):
        self._ros2 = ros2
        self._node = self._publisher = None
        self._topics = [{"topic": f"/{namespace}/state/spatial_map", "format": "data/json"}]
        self._python = Path(os.getenv("HELIOS_SDK_PYTHON_DIR", "/opt/sdk-python")) / "bin/python3.12"
        self._sdk = Path(os.getenv("HELIOS_VMR_SDK_DIR", "/opt/vmr-sdk")) / "lib"
        self._host = os.getenv("HELIOS_VMR_HOST") or config.get("spatial_map_sdk_host", "192.168.0.100")
        self._port = int(config.get("spatial_map_sdk_port", 9003))
        self._lock = threading.RLock()
        self._enabled = False
        self._process = None
        self._data = {"state": "idle"}
        self._last_read = self._next_attempt = 0.0

    def get_tool(self):
        return tool("spatial_map", "sensor",
                    "读取当前地图名称、定位状态码与置信度，以及工作点和充电点的名称、类型和 ID。",
                    topic_out=self._topics)

    def start(self):
        if self._ros2 is None or self._node is not None:
            return
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        node = Node("helios_spatial_map", context=self._ros2.ctx_core)
        try:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._publisher = node.create_publisher(String, self._topics[0]["topic"], qos)
            self._message = String
            node.create_timer(1.0, self._tick)
            self._ros2.executor_core.add_node(node)
            self._node = node
        except Exception:
            self._publisher = None
            node.destroy_node()
            raise

    def _launch(self):
        env = dict(os.environ, PYTHONPATH=str(self._sdk),
                   LD_LIBRARY_PATH=os.pathsep.join([str(self._sdk), str(self._python.parent.parent / "lib")]))
        worker = Path(__file__).with_name("spatial_map_worker.py")
        proc = subprocess.Popen([str(self._python), str(worker), self._host, str(self._port)],
                                env=env, stdout=subprocess.PIPE, text=True, encoding="utf-8")
        self._process = proc
        self._last_read = time.monotonic()
        threading.Thread(target=self._receive, args=(proc,), name="helios-spatial-map", daemon=True).start()

    def _receive(self, proc):
        with proc.stdout as stream:
            for line in stream:
                if line.startswith("SPATIAL_MAP "):
                    data = json.loads(line.removeprefix("SPATIAL_MAP "))
                    with self._lock:
                        if proc is not self._process:
                            return
                        self._data = data
                        self._last_read = time.monotonic()

    def _end_process(self):
        proc, self._process = self._process, None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    def _tick(self):
        with self._lock:
            if not self._enabled:
                return
            now = time.monotonic()
            if self._process is not None:
                if self._process.poll() is not None or now - self._last_read > 10:
                    self._end_process()
                    if self._data["state"] != "unavailable":
                        self._data = {"state": "unavailable", "message": "VMR worker stopped reporting"}
                    self._next_attempt = now + 2
            if self._process is None and now >= self._next_attempt:
                try:
                    self._launch()
                except OSError as exc:
                    self._data = {"state": "unavailable", "message": str(exc)}
                    self._next_attempt = now + 2
            self._publish()

    def payload(self):
        with self._lock:
            data = self._data
            if data["state"] in ("online", "partial") and time.monotonic() - self._last_read > 6:
                data = {"state": "stale"}
            return {"map_name": None, "stations": None, "localization": None, **data}

    def _publish(self):
        if self._publisher is not None:
            self._publisher.publish(self._message(data=json.dumps(self.payload(), ensure_ascii=False)))

    def _stop_capture(self):
        self._enabled = False
        self._end_process()
        self._data = {"state": "idle"}
        self._publish()

    def stop(self):
        with self._lock:
            self._stop_capture()
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = self._publisher = None

    def dispatch(self, action, args):
        with self._lock:
            if action == "info":
                return self.get_tool()
            if action in ("spatial_map", "read"):
                return {"data": self.payload(), "topic_out": self._topics}
            if action == "start":
                if not self._enabled:
                    if not self._python.is_file() or not self._sdk.is_dir():
                        raise ValueError("spatial_map requires the VMR SDK and its Python runtime mounts")
                    self._data = {"state": "waiting"}
                    self._launch()
                    self._enabled = True
                return {"state": "running", "topic_out": self._topics}
            if action == "stop":
                self._stop_capture()
                return {"state": "idle", "topic_out": self._topics}
            if action == "config":
                return {"state": "configured", "topic_out": self._topics}
            raise ValueError(f"unsupported spatial_map action: {action}")
