"""Helios map and station sensor. The SDK worker runs only while the card is active."""

from array import array
import base64
from concurrent.futures import Future
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
import uuid

from common.vendor_runtime import tool
from .controlled_spatial import ControlledSpatial, definition
from .spatial_state import ChassisMapState
from .spatial_cloud import MAX_POINTS, XYZ, SavedCloud, mapping_frame


class HeliosSpatialMap:
    PREFIX = "spatial_map"

    def __init__(self, config, namespace="rokae_helios", ros2=None):
        self._ros2 = ros2
        self._node = self._publisher = None
        self._map_publishers = {}
        base = f"/{namespace}/state/spatial_map"
        self._topics = [{"topic": base, "format": "sensor/mapping"},
                        {"topic": base + "/status", "format": "data/json"}]
        self._control_topic = {"topic": f"/{namespace}/controlled_spatial/map", "format": "sensor/mapping"}
        chassis_url = os.getenv("HELIOS_BATTERY_STATE_URL") or config.get("battery_state_url", "ws://192.168.0.100/ws/AGVInfo")
        endpoint = urlsplit(chassis_url)
        self.cloud = SavedCloud(("https://" if endpoint.scheme == "wss" else "http://") + endpoint.netloc)
        self.selected_cloud = SavedCloud(self.cloud.origin)
        self.chassis = ChassisMapState(chassis_url)
        self.control = ControlledSpatial(config.get("spatial_db", "/var/lib/helios-spatial/tags.db"))
        self.voxels = {}
        self._pending = {}
        self._operations = threading.RLock()
        self._python = Path(os.getenv("HELIOS_SDK_PYTHON_DIR", "/opt/sdk-python")) / "bin/python3.12"
        self._sdk = Path(os.getenv("HELIOS_VMR_SDK_DIR", "/opt/vmr-sdk")) / "lib"
        self._host = os.getenv("HELIOS_VMR_HOST") or config.get("spatial_map_sdk_host", "192.168.0.100")
        self._port = int(config.get("spatial_map_sdk_port", 9003))
        self._lock = threading.RLock()
        self._users = set()
        self._process = None
        self._data = {"state": "idle"}
        self._last_read = self._next_attempt = 0.0

    def get_tool(self):
        definition = tool("spatial_map", "sensor",
                          "查看原厂三维点云地图、机器人位置和标记点；同时输出地图、定位和站点。",
                          topic_out=self._topics)
        definition["preview_topic"] = self._topics[0]["topic"]
        return definition

    def get_tools(self):
        return [self.get_tool(), definition(self._control_topic)]

    def start(self):
        if self._ros2 is None or self._node is not None:
            return
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String, UInt8MultiArray

        node = Node("helios_spatial_map", context=self._ros2.ctx_core)
        try:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._publisher = node.create_publisher(String, self._topics[1]["topic"], qos)
            for key, topic in (("spatial_map", self._topics[0]), ("controlled_spatial", self._control_topic)):
                self._map_publishers[key] = node.create_publisher(UInt8MultiArray, topic["topic"], qos)
            self._message, self._cloud_message = String, UInt8MultiArray
            node.create_timer(1.0, self._tick)
            self._ros2.executor_core.add_node(node)
            self._node = node
        except Exception:
            self._publisher = None
            self._map_publishers.clear()
            node.destroy_node()
            raise

    def _launch(self):
        driver_root = Path(__file__).resolve().parents[2]
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(self._sdk), str(driver_root))),
                   LD_LIBRARY_PATH=os.pathsep.join([str(self._sdk), str(self._python.parent.parent / "lib")]))
        worker = Path(__file__).with_name("spatial_map_worker.py")
        read_fd, write_fd = os.pipe()
        try:
            with os.fdopen(write_fd, "wb") as output:
                proc = subprocess.Popen(
                    [str(self._python), str(worker), self._host, str(self._port), str(output.fileno()), self.control.building or ""],
                    pass_fds=(output.fileno(),), env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace")
        except OSError:
            os.close(read_fd)
            raise
        self._process = proc
        self._last_read = time.monotonic()
        threading.Thread(target=self._receive, args=(proc, read_fd), name="helios-spatial-map", daemon=True).start()
        threading.Thread(target=self._relay_logs, args=(proc,), name="helios-spatial-map-logs", daemon=True).start()

    @staticmethod
    def _relay_logs(proc):
        with proc.stdout as stream:
            for line in stream:
                print(line.rstrip("\n"), file=sys.stderr)

    def _receive(self, proc, read_fd):
        with os.fdopen(read_fd, encoding="utf-8") as stream:
            for line in stream:
                if not line.endswith("\n"):  # Worker stopped mid-snapshot.
                    return
                data = json.loads(line)
                with self._lock:
                    if proc is not self._process:
                        return
                    if "reply" in data:
                        future = self._pending.pop(data["reply"], None)
                        if future:
                            if "error" in data:
                                future.set_exception(ValueError(data["error"]))
                            else:
                                future.set_result(data["result"])
                        continue
                    scan = data.pop("scan", "")
                    if scan and self.control.building:
                        for pt in XYZ.iter_unpack(base64.b64decode(scan)):
                            cell = tuple(math.floor(v * 10) for v in pt)
                            self.voxels[cell] = pt
                            if len(self.voxels) > MAX_POINTS:
                                self.voxels.pop(next(iter(self.voxels)))
                    self._data = data
                    self._last_read = time.monotonic()

    def _end_process(self):
        proc, self._process = self._process, None
        for future in self._pending.values():
            future.set_exception(RuntimeError("VMR connection closed; check chassis state before retrying"))
        self._pending.clear()
        if proc is not None and proc.poll() is None:
            proc.stdin.close()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    def _tick(self):
        with self._lock:
            if not self._users and not self.control.building:
                return
            now = time.monotonic()
            if self._process is not None:
                if self._process.poll() is not None or (not self._pending and now - self._last_read > 10):
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

    def command(self, action, **args):
        with self._lock:
            proc = self._process
            if proc is None or proc.poll() is not None:
                raise RuntimeError("VMR worker unavailable")
            ident = uuid.uuid4().hex
            future = Future()
            self._pending[ident] = future
            try:
                proc.stdin.write(json.dumps({"id": ident, "action": action, **args}) + "\n")
                proc.stdin.flush()
            except OSError:
                self._pending.pop(ident, None)
                raise
        try:
            return future.result(timeout=25)
        except TimeoutError as exc:
            raise RuntimeError("VMR command timed out; check chassis state before retrying") from exc
        finally:
            with self._lock:
                self._pending.pop(ident, None)

    def _publish(self):
        if self._publisher is None:
            return
        data = self.payload()
        name = data["map_name"]
        if self.control.building:
            name = self.control.building
            points = b"".join(XYZ.pack(*pt) for pt in self.voxels.values())
        else:
            points = self.cloud.read(name)
        tags = self.control.tags(name)
        pose = data["localization"] if name == data["map_name"] else None
        if "spatial_map" in self._users:
            status = {**data, "display_map": name, "cloud_points": len(points)//12,
                      "cloud_error": self.cloud.error, "tags": tags}
            self._publisher.publish(self._message(data=json.dumps(status, ensure_ascii=False)))
        if self._users:
            msg = self._cloud_message()
            msg.data = array("B", mapping_frame(points, pose, tags, name))
            for key in self._users:
                if (key == "controlled_spatial" and not self.control.building and
                        self.control.selected and self.control.selected != name):
                    selected = self.control.selected
                    archive = self.selected_cloud.read(selected)
                    selected_msg = self._cloud_message()
                    selected_msg.data = array("B", mapping_frame(archive, None, self.control.tags(selected), selected))
                    self._map_publishers[key].publish(selected_msg)
                else:
                    self._map_publishers[key].publish(msg)

    def _enable(self, key):
        with self._lock:
            if not self._python.is_file() or not self._sdk.is_dir():
                raise ValueError("spatial_map requires the VMR SDK and its Python runtime mounts")
            self.control.open()
            if self._process is None:
                self._data = {"state": "waiting"}
                self._launch()
                self.chassis.start()
            self._users.add(key)

    def _disable(self, key):
        with self._lock:
            self._users.discard(key)
            if key in self._map_publishers:
                msg = self._cloud_message()
                msg.data = array("B", mapping_frame(b"", None, [], None))
                self._map_publishers[key].publish(msg)
            if key == "spatial_map" and self._publisher:
                self._publisher.publish(self._message(data=json.dumps({"state": "idle", "map_name": None,
                                                                        "stations": None, "localization": None})))
            if not self._users and not self.control.building:
                self._end_process()
                self.chassis.stop()
                self.control.close()
                self.control.selected = None
                self.cloud.clear()
                self.selected_cloud.clear()
                self._data = {"state": "idle"}

    def stop(self):
        with self._operations, self._lock:
            self._users.clear()
            self._end_process()
            self.chassis.stop()
            self.control.close()
            self.cloud.close()
            self.selected_cloud.close()
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = self._publisher = None
                self._map_publishers.clear()

    def dispatch(self, action, args):
        key = args.get("_tool_name", "spatial_map")
        topics = [self._control_topic] if key == "controlled_spatial" else self._topics
        with self._operations:
            if action == "info":
                return definition(self._control_topic) if key == "controlled_spatial" else self.get_tool()
            if action == "config":
                return {"state": "configured", "topic_out": topics}
            if action == "start":
                self._enable(key)
                return {"state": "running", "topic_out": topics}
            if action == "stop":
                if key == "controlled_spatial" and key in self._users:
                    self.command("stop_nav")
                self._disable(key)
                return {"state": "idle", "topic_out": topics}
            if key == "controlled_spatial":
                self._enable(key)
                return self.control.execute("status" if action == key else action, args, self)
            if action in ("spatial_map", "read"):
                return {"data": self.payload(), "topic_out": topics}
            raise ValueError(f"unsupported spatial_map action: {action}")
