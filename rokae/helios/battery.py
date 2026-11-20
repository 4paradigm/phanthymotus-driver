"""Read the two BMS packs from the chassis UI's existing binary status stream."""

from __future__ import annotations

import json
import math
import os
import threading
import time

from google.protobuf.message import DecodeError

from common.vendor_runtime import tool
from .chassis_battery_pb2 import UiDatagram
from .feedback import StreamFeedbackClient, positive_number, validate_url


def _number(record, key, *, minimum=None, maximum=None, integer=False):
    value = record.get(key)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        return None
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        return None
    if integer and value != int(value):
        return None
    return int(value) if integer else float(value)


def _version(record, key):
    value = record.get(key)
    return value if isinstance(value, str) and 0 < len(value) <= 80 and value.isprintable() else None


def parse_battery_frame(raw):
    """Skip unrelated datagrams; do not confuse absent/invalid SOC with zero %."""
    message = UiDatagram()
    try:
        message.ParseFromString(raw)
    except DecodeError as exc:
        raise ValueError("invalid chassis Protobuf frame") from exc
    if not message.HasField("robot_base"):
        return None
    try:
        pool = json.loads(message.robot_base.data_pool)
        packs = json.loads(pool["multi_battery_info"])
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError("missing or invalid multi_battery_info") from exc
    if not isinstance(packs, list) or len(packs) > 2:
        raise ValueError("expected up to two Helios battery packs")
    by_id = {}
    for pack in packs:
        if not isinstance(pack, dict):
            raise ValueError("invalid battery pack record")
        pack_id = pack.get("id")
        if not isinstance(pack_id, str) or pack_id not in ("1", "2") or pack_id in by_id:
            raise ValueError("unexpected or duplicate battery ID")
        by_id[pack_id] = pack
    result = []
    for pack_id in ("1", "2"):
        pack = by_id.get(pack_id, {})
        soc = _number(pack, "soc", minimum=0, maximum=100)
        voltage_mv = _number(pack, "voltage", minimum=0)
        cell_min = _number(pack, "min_cell_voltage", minimum=0)
        cell_max = _number(pack, "max_cell_voltage", minimum=0)
        # Cell voltage mV scale matches the captured BMS values and pack voltage.
        # Explicit manufacturer unit declaration is pending; see BATTERY_INTERFACE.md.
        # Reject an inverted range.
        if cell_min is not None and cell_max is not None and cell_min > cell_max:
            cell_min = cell_max = None
        result.append({
            "id": pack_id, "label": f"{pack_id}号电池", "robot_id": message.robot_base.robot_id,
            "available": soc is not None,
            "state": "online" if soc is not None else "not_reported" if pack_id not in by_id else "invalid_soc",
            "soc_pct": soc,
            "voltage_v": None if voltage_mv is None else voltage_mv / 1000,
            "current_ma": _number(pack, "current"),
            "temperature_min_c": _number(pack, "min_temperature"),
            "temperature_max_c": _number(pack, "max_temperature"),
            "cycle_count": _number(pack, "cycle_count", minimum=0, integer=True),
            "cell_voltage_min_v": None if cell_min is None else cell_min / 1000,
            "cell_voltage_max_v": None if cell_max is None else cell_max / 1000,
            "full_charge_capacity_mah": _number(pack, "fcc", minimum=0),
            "remaining_capacity_mah": _number(pack, "rc", minimum=0),
            "bms_hardware_version": _version(pack, "hardware_version"),
            "bms_software_version": _version(pack, "software_version"),
        })
    return result


class BatteryFeedbackClient(StreamFeedbackClient):
    def __init__(self, url, **kwargs):
        # No online/heartbeat/control message is required by this installed UI.
        super().__init__(url, parser=parse_battery_frame, data_key="batteries", binary=True,
                         max_msg_size=8 * 1024 * 1024, **kwargs)


class HeliosBattery:
    PREFIX = "battery"

    def __init__(self, config, namespace="rokae_helios", ros2=None, client=None):
        self._config = config
        self._ros2 = ros2
        self._node = None
        self._publisher = None
        self._lock = threading.RLock()
        self._hz = positive_number(config.get("battery_publish_hz", 1), "battery_publish_hz")
        if self._hz > 10:
            raise ValueError("battery_publish_hz must not exceed 10")
        self._topics = [{"topic": f"/{namespace}/state/battery", "format": "data/json"}]
        self.client = client or self._new_client(os.getenv(
            "HELIOS_BATTERY_STATE_URL", config["battery_state_url"]))

    def _new_client(self, url):
        return BatteryFeedbackClient(url, stale_timeout_s=self._config.get("battery_stale_timeout_s", 3),
                                     connect_timeout_s=self._config.get("connect_timeout_s", 3),
                                     reconnect_delay_s=self._config.get("reconnect_delay_s", 1))

    def get_tools(self):
        sensor = tool("battery", "sensor",
                      "Helios-2.1 双电池状态（battery_1_* / battery_2_*）：soc(%)、voltage(V)、current(mA，原始符号)、"
                      "cell_vol_min/max(V)、temperature_min/max(℃)、cycle(次)、remaining/full_charge_capacity(mAh)。"
                      "只读；1/2 为厂家编号，左右安装位置待核对；不推断充电状态，断线/过期数值为空。",
                      topic_out=self._topics)
        sensor["configSchema"] = {"type": "object", "properties": {
            "battery_state_url": {"type": "string", "scope": "shared", "x-sensitive": True,
                                  "description": "底盘状态 WebSocket 地址（留空使用部署配置）"},
        }}
        return [sensor]

    def start(self):
        with self._lock:
            if self._node is None and self._ros2 is not None:
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                from std_msgs.msg import String
                node = Node("helios_battery", context=self._ros2.ctx_core)
                try:
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
                    self._publisher = node.create_publisher(String, self._topics[0]["topic"], qos)
                    self._message_type = String
                    node.create_timer(1 / self._hz, self._publish)
                    self._ros2.executor_core.add_node(node)
                    self._node = node
                except Exception:
                    self._publisher = None
                    node.destroy_node()
                    raise
            self.client.start()

    def stop(self):
        with self._lock:
            self.client.stop()
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            self._publisher = None

    # Flat, G1-style top-level fields (battery_1_soc, ...). Core's generic log
    # view puts each top-level key on its own line and the "latest" view gives
    # each one a cell. Units are fixed and documented in the tool description.
    _PACK_FIELDS = (("soc", "soc_pct"), ("voltage", "voltage_v"), ("current", "current_ma"),
                    ("cell_vol_min", "cell_voltage_min_v"), ("cell_vol_max", "cell_voltage_max_v"),
                    ("temperature_min", "temperature_min_c"), ("temperature_max", "temperature_max_c"),
                    ("cycle", "cycle_count"), ("remaining_capacity", "remaining_capacity_mah"),
                    ("full_charge_capacity", "full_charge_capacity_mah"))

    def payload(self):
        snap = self.client.snapshot()
        by_id = {p["id"]: p for p in snap["batteries"]}
        reported = sum(p["available"] for p in snap["batteries"])
        state = snap["state"]
        if snap["available"]:
            state = "online" if reported == 2 else "partial" if reported else "unavailable"
        result = {}
        for pack_id in ("1", "2"):
            pack = by_id.get(pack_id)
            prefix = f"battery_{pack_id}_"
            # Missing or invalid values stay null; never fill with zero.
            for name, source in self._PACK_FIELDS:
                result[prefix + name] = pack[source] if pack else None
            result[prefix + "state"] = pack["state"] if pack else state
        return {**result, "state": state,
                "message": {"online": "两块电池状态正常", "partial": "部分电池电量未报告或无效",
                            "unavailable": "未收到有效电池电量", "idle": "电池订阅已停止",
                            "reconnecting": "底盘未连接，正在重连", "waiting": "等待底盘电池数据",
                            "stale": "电池数据过期", "invalid": "电池数据格式无效"}[state],
                "age_ms": snap["age_ms"], "timestamp_ms": int(time.time() * 1000)}

    def _publish(self):
        with self._lock:
            if self._publisher is not None:
                msg = self._message_type()
                msg.data = json.dumps(self.payload(), ensure_ascii=False, allow_nan=False)
                self._publisher.publish(msg)

    def dispatch(self, action, args):
        if args.get("_tool_name", "battery") != "battery":
            return None
        with self._lock:
            if action == "info":
                return self.get_tools()[0]
            if action in ("battery", "read"):
                return {"data": self.payload(), "topic_out": self._topics}
            if action in ("start", "config"):
                url = args.get("battery_state_url")
                if url and validate_url(url) != self.client.url:
                    replacement = self._new_client(url)
                    self.client.stop()
                    self.client = replacement
                    self.client.start()
                if action == "start":
                    self.client.start()
                return {"state": "running" if action == "start" else "configured", "topic_out": self._topics}
            if action == "stop":
                self.client.stop()
                self._publish()
                return {"state": "idle", "topic_out": self._topics}
            raise ValueError(f"unsupported battery action: {action}")
