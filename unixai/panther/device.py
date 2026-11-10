#!/usr/bin/env python3
"""UniX AI Panther driver: read-only sensor cards.

The robot (ROS 2 Humble, CycloneDDS, domain 0, loopback-bound) is read in this process;
finished payloads are handed to `panther_bridge.CoreBridge`, whose child processes publish
them to Agent Core on domain 42. Nothing here publishes to the robot, calls one of its
services or switches a controller: every card is a `sensor` or a `resource`.

`Hub` holds all the logic and knows nothing about ROS, so it is tested with plain objects.
`PantherNodes` is the thin rclpy layer that feeds it. `PantherPlugin` is the MCP surface.
"""
from __future__ import annotations

import json
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from common.camera_info import build as build_camera_info
from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms
from common.vendor_runtime import tool
from panther_bridge import CoreBridge, DEFAULT_PROFILE, safe_text
from panther_payloads import (
    DEPTH_FORMAT, DEPTH_OUTPUT_SIZE, UnsupportedImage, battery_payload, diagnostic_entries,
    encode_color_jpeg, encode_depth_zlib, estop_payload, health_payload, imu_payload,
    joint_state_payload, laser_payload, laser_topdown_jpeg, odom_reading, single_joint,
)

URDF_PATH = Path(__file__).with_name("resource") / "panther.urdf"
CAMERA_ID = "unixai/panther/{}"

# Conversion work that is skipped until the card is started: the camera encodes, and the
# lasers' top-down picture. Everything else is cheap and always publishes. Skipping matters
# for the cameras: a colour JPEG and a resampled depth frame every 100-200 ms are real load
# on a Jetson that is also running the robot.
LASER_CARDS = ("laser_scan_front", "laser_scan_rear")
CAMERA_CARDS = ("camera_color", "camera_depth")

DESCRIPTIONS = {
    "model": "UniX AI Panther 运动学骨架（URDF，机器人名 mh3），供 joints 骨架渲染使用",
    "joints": "Panther 18 个关节（双臂 16 + 头部 2）实时角度，约 10Hz，3D 骨架。"
              "升降和夹爪不在此骨架内，见 gripper_lift_state",
    "battery": "Panther 电池：电量百分比、电压、电流、温度、充放电状态，约 1Hz",
    "imu": "Panther IMU：姿态四元数/欧拉角、角速度、线加速度，约 10Hz。传感器未提供的量为 null",
    "odom": "Panther 底盘速度里程计（机体坐标系 vx/vy/wz，不含位姿），约 10Hz",
    "gripper_lift_state": "Panther 左右夹爪和升降的原始关节读数（位置/速度/力矩），约 10Hz。"
                          "夹爪数值的开合含义和升降单位以厂商文档为准，本卡片不做换算",
    "health": "Panther 硬件诊断汇总：CAN 总线、串口通信、底盘电机、手臂电机、急停，约 2Hz",
    "estop": "Panther 急停诊断（厂商原文原样透传，附数据年龄和是否过期），约 1Hz。"
             "stale 为 true 时不代表安全",
    "laser_scan_front": "Panther 前激光雷达：最近障碍物降采样距离 JSON，约 5Hz；启动后另发 1Hz 俯视图",
    "laser_scan_rear": "Panther 后激光雷达：最近障碍物降采样距离 JSON，约 5Hz；启动后另发 1Hz 俯视图。"
                       "该话题目前可能没有数据",
    "camera_color": "Panther 头部 RGB 相机（奥比中光 Gemini 2）彩色图像。需要机器人上已启动相机驱动",
    "camera_depth": "Panther 头部 RGB-D 相机深度图（640x480 uint16 毫米，zlib）。需要机器人上已启动相机驱动",
}
CARD_ORDER = tuple(DESCRIPTIONS)

NO_DATA_HINTS = {
    "camera_color": "没有收到图像：机器人上的相机驱动没有运行。请在机器人上启动 "
                    "`ros2 launch orbbec_camera gemini2.launch.py`（本驱动不会替你启动）",
    "camera_depth": "没有收到深度图：机器人上的相机驱动没有运行。请在机器人上启动 "
                    "`ros2 launch orbbec_camera gemini2.launch.py`（本驱动不会替你启动）",
    "laser_scan_rear": "没有收到后雷达数据：该话题可能没有发布者",
}


def _urdf_joint_names(path: Path = URDF_PATH) -> frozenset:
    root = ET.parse(path).getroot()
    return frozenset(j.get("name") for j in root.findall("joint")
                     if j.get("type") in ("revolute", "continuous", "prismatic"))


class BridgeSink:
    """What `Hub` publishes through. `CoreBridge` underneath; tests substitute a recorder."""

    def __init__(self, bridge: CoreBridge):
        self._bridge = bridge
        bridge.register_lane("state", queue_depth=256, latest_only=False)
        bridge.register_lane("views", queue_depth=4, latest_only=False)
        bridge.register_lane("camera_color", queue_depth=1, latest_only=True)
        bridge.register_lane("camera_depth", queue_depth=1, latest_only=True)

    def json(self, lane: str, topic: str, text: str) -> bool:
        return self._bridge.send(lane, ("json", topic, text))

    def image(self, lane: str, topic: str, image_format: str, data: bytes, frame_id: str) -> bool:
        return self._bridge.send(lane, ("image", topic, image_format, data, frame_id))

    def stats(self) -> dict:
        return self._bridge.stats()

    def close(self) -> None:
        self._bridge.stop()


class Hub:
    def __init__(self, config: dict, namespace: str, sink, *, clock=time.time,
                 monotonic=time.monotonic, known_joints=None):
        self._config = config or {}
        self._sink = sink
        self._clock = clock
        self._mono = monotonic
        self._known_joints = known_joints if known_joints is not None else _urdf_joint_names()
        self._lock = threading.RLock()
        self._started = self._mono()

        rates = {"joints": 10, "battery": 1, "imu": 10, "odom": 10, "gripper_lift_state": 10,
                 "health": 2, "estop": 1, "laser_scan": 5, "laser_view": 1,
                 "camera_color": 10, "camera_depth": 5}
        rates.update(self._config.get("rates_hz", {}))
        self._rates = rates
        stale = {"joints": 1.0, "battery": 5.0, "imu": 1.0, "odom": 1.0, "gripper_lift_state": 1.0,
                 "health": 5.0, "estop": 3.0, "laser_scan": 1.5, "camera": 2.0}
        stale.update(self._config.get("stale_after_s", {}))
        self._stale = stale
        camera = self._config.get("camera", {})
        self._jpeg_quality = int(camera.get("jpeg_quality", 80))
        self._max_width = int(camera.get("max_width", 640))
        laser = self._config.get("laser", {})
        self._laser_points = int(laser.get("max_points", 360))
        self._view_range = float(laser.get("view_range_m", 6.0))

        base = f"/{namespace}/" if namespace else "/"
        self.topics = {
            "joints": {"topic": base + "joints", "format": "sensor/skeleton"},
            "battery": {"topic": base + "battery", "format": "data/json"},
            "imu": {"topic": base + "imu", "format": "data/json"},
            "odom": {"topic": base + "state/odom", "format": "state/odom"},
            "gripper_lift_state": {"topic": base + "gripper_lift_state", "format": "data/json"},
            "health": {"topic": base + "health", "format": "data/json"},
            "estop": {"topic": base + "estop", "format": "data/json"},
            "laser_scan_front": {"topic": base + "laser_scan_front", "format": "data/json"},
            "laser_scan_rear": {"topic": base + "laser_scan_rear", "format": "data/json"},
            "camera_color": {"topic": base + "camera_color", "format": "image/jpeg"},
            "camera_depth": {"topic": base + "camera_depth", "format": "image/depth-zlib"},
        }
        self.views = {card: {"topic": base + card + "_view", "format": "image/jpeg"}
                      for card in LASER_CARDS}
        self.sources = dict(self._config.get("topics", {}))

        self._stats = {card: self._blank_stat() for card in CARD_ORDER if card != "model"}
        self._active = {card: False for card in CARD_ORDER if card != "model"}
        self._next: dict = {}
        self._latest: dict = {}
        self._odom_burst: list = []
        self._odom_meta: dict = {}
        self._diag: dict = {}
        self._diag_signature = None
        self._estop_signature = None
        self._scans: dict = {}
        self._image_info: dict = {}
        self._unknown_joint_warned = False
        self._link_logged = 0.0

    # ── bookkeeping ──────────────────────────────────────────────────────────

    @staticmethod
    def _blank_stat() -> dict:
        return {"received": 0, "published": 0, "dropped": 0, "errors": 0, "last_error": None,
                "last_rx": None}

    def _rx(self, card: str) -> dict:
        stat = self._stats[card]
        stat["received"] += 1
        stat["last_rx"] = self._mono()
        return stat

    def _due(self, key: str, hz: float) -> bool:
        """True `hz` times a second on average, however fast the source and however it jitters.

        Scheduled rather than "time since last": a source at 100 Hz asked for 10 Hz must
        give 10, not 11, and a 10 Hz timer asked for 10 Hz must not lose every other tick to
        a tick that arrives 0.1 ms early. The slot advances by whole periods, so an early
        tick is accepted without the rate drifting up.
        """
        now = self._mono()
        period = 1.0 / hz
        slot = self._next.get(key)
        if slot is not None and now < slot - 0.1 * period:
            return False
        self._next[key] = slot + period if slot is not None and now - slot < period else now + period
        return True

    def _mark(self, key: str, hz: float) -> None:
        """Restart `key`'s schedule after publishing it out of turn (a change worth sending at once)."""
        self._next[key] = self._mono() + 1.0 / hz

    def _now_ms(self) -> int:
        return int(self._clock() * 1000)

    def _run(self, card: str, function, *args) -> None:
        """No exception may reach rclpy: one escaping a callback ends the executor thread, and
        with it every card, not just this one."""
        try:
            function(*args)
        except Exception as exc:  # noqa: BLE001
            stat = self._stats[card]
            stat["errors"] += 1
            stat["last_error"] = f"{type(exc).__name__}: {exc}"
            if stat["errors"] == 1 or stat["errors"] % 100 == 0:
                print(f"[panther] {card} handler failed (#{stat['errors']}): "
                      f"{type(exc).__name__}: {safe_text(exc, 160)}", flush=True)

    def _emit(self, card: str, payload: dict, topic_key: str | None = None, lane: str = "state") -> None:
        # allow_nan=False turns a NaN that slipped past the payload code into an error here,
        # not into invalid JSON on the wire.
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        topic = self.topics[topic_key or card]["topic"]
        stat = self._stats[card]
        if self._sink.json(lane, topic, text):
            stat["published"] += 1
        else:
            stat["dropped"] += 1

    def _emit_image(self, card: str, lane: str, topic: str, image_format: str, data: bytes,
                    frame_id: str) -> None:
        stat = self._stats[card]
        if self._sink.image(lane, topic, image_format, data, frame_id):
            stat["published"] += 1
        else:
            stat["dropped"] += 1

    # ── activation ───────────────────────────────────────────────────────────

    def activate(self, card: str) -> None:
        self._active[card] = True

    def deactivate(self, card: str) -> None:
        self._active[card] = False

    def is_active(self, card: str) -> bool:
        return self._active.get(card, False)

    # ── handlers (rclpy callbacks) ───────────────────────────────────────────

    def on_joint_state(self, msg) -> None:
        self._run("joints", self._joint_state, msg)

    def _joint_state(self, msg) -> None:
        self._rx("joints")
        if not self._due("joints", self._rates["joints"]):
            return
        if not self._unknown_joint_warned:
            unknown = sorted(set(msg.name) - self._known_joints)
            if unknown:
                self._unknown_joint_warned = True
                print(f"[panther] /joint_states names not in the URDF are left out of the "
                      f"skeleton: {safe_text(unknown, 200)}", flush=True)
        self._emit("joints", joint_state_payload(msg, self._known_joints))

    def on_battery(self, msg) -> None:
        self._run("battery", self._battery, msg)

    def _battery(self, msg) -> None:
        self._rx("battery")
        if self._due("battery", self._rates["battery"]):
            self._emit("battery", battery_payload(msg))

    def on_imu(self, msg) -> None:
        self._run("imu", self._imu, msg)

    def _imu(self, msg) -> None:
        self._rx("imu")
        if self._due("imu", self._rates["imu"]):
            self._emit("imu", imu_payload(msg))

    def on_odom(self, msg) -> None:
        self._run("odom", self._odom, msg)

    def _odom(self, msg) -> None:
        """Average every reading in the window rather than keep one: see README_dev
        'Downsampling: average, do not pick'."""
        self._rx("odom")
        reading, meta = odom_reading(msg)
        with self._lock:
            self._odom_burst.append(reading)
            self._odom_meta = meta
        if not self._due("odom", self._rates["odom"]):
            return
        with self._lock:
            burst, self._odom_burst = self._odom_burst, []
        received_ms = self._now_ms()
        stamp, provenance = resolve_stamp_ms(vendor_ms=meta["vendor_ms"], received_ms=received_ms)
        sample = build_sample(
            stamp_ms=stamp, twist=mean_twist(burst), frame="body",
            vendor={**provenance, "samples": len(burst), "frame_id": meta["frame_id"],
                    "child_frame_id": meta["child_frame_id"]})
        self._emit("odom", sample)

    def on_gripper_left(self, msg) -> None:
        self._run("gripper_lift_state", self._store_joint, "left_gripper", msg)

    def on_gripper_right(self, msg) -> None:
        self._run("gripper_lift_state", self._store_joint, "right_gripper", msg)

    def on_lift(self, msg) -> None:
        self._run("gripper_lift_state", self._store_joint, "lift", msg)

    def _store_joint(self, key: str, msg) -> None:
        self._rx("gripper_lift_state")
        with self._lock:
            self._latest[key] = (self._mono(), single_joint(msg))

    def on_diagnostics(self, msg) -> None:
        self._run("health", self._diagnostics, msg)

    def _diagnostics(self, msg) -> None:
        self._rx("health")
        self._stats["estop"]["last_rx"] = self._mono()
        self._stats["estop"]["received"] += 1
        received_ms = self._now_ms()
        with self._lock:
            for entry in diagnostic_entries(msg):
                self._diag[entry["name"]] = {"entry": entry, "received_ms": received_ms}
            signature = tuple(sorted((n, r["entry"]["level"], r["entry"]["message"])
                                     for n, r in self._diag.items()))
            changed = signature != self._diag_signature
            self._diag_signature = signature
        if changed or self._due("health", self._rates["health"]):
            self._mark("health", self._rates["health"])
            self._emit("health", health_payload(self._snapshot_diag(), received_ms,
                                                int(self._stale["health"] * 1000)))
        self._publish_estop(force_if_changed=True)

    def _snapshot_diag(self) -> dict:
        with self._lock:
            return dict(self._diag)

    def _publish_estop(self, force_if_changed: bool = False) -> None:
        now_ms = self._now_ms()
        payload = estop_payload(self._snapshot_diag(), now_ms, int(self._stale["estop"] * 1000))
        signature = (payload.get("found"), payload.get("level"), payload.get("message"),
                     payload.get("stop_button_status"), payload.get("stale"))
        changed = signature != self._estop_signature
        if (force_if_changed and changed) or self._due("estop", self._rates["estop"]):
            self._mark("estop", self._rates["estop"])
            self._estop_signature = signature
            self._emit("estop", payload)

    def on_laser(self, side: str, msg) -> None:
        card = f"laser_scan_{side}"
        self._run(card, self._laser, card, msg)

    def _laser(self, card: str, msg) -> None:
        self._rx(card)
        scan = None
        if self._due(card, self._rates["laser_scan"]):
            scan = laser_payload(msg, self._laser_points)
            with self._lock:
                self._scans[card] = scan
            self._emit(card, scan)
        if self.is_active(card) and self._due(card + "_view", self._rates["laser_view"]):
            scan = scan or self._scans.get(card) or laser_payload(msg, self._laser_points)
            jpeg = laser_topdown_jpeg(scan, max_range_m=self._view_range)
            if jpeg is None:
                self._stats[card]["last_error"] = "top-down view unavailable: OpenCV is not installed"
                return
            self._emit_image(card, "views", self.views[card]["topic"], "jpeg", jpeg,
                             scan.get("frame_id", ""))

    def on_image(self, card: str, msg) -> None:
        self._run(card, self._image, card, msg)

    def _image(self, card: str, msg) -> None:
        self._rx(card)
        if not self.is_active(card) or not self._due(card, self._rates[card]):
            return
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", ""))
        encoding = str(getattr(msg, "encoding", ""))
        try:
            if card == "camera_color":
                data, size = encode_color_jpeg(msg, self._jpeg_quality, self._max_width)
                image_format, source = "jpeg", size
            else:
                data, size, source = encode_depth_zlib(msg, DEPTH_OUTPUT_SIZE)
                image_format = DEPTH_FORMAT
        except UnsupportedImage as exc:
            # Say so once in the log, and always in `info`: a frame silently dropped here
            # looks exactly like a camera that is not running.
            stat = self._stats[card]
            if stat["last_error"] != str(exc):
                print(f"[panther] {card}: {safe_text(exc, 200)}", flush=True)
            stat["errors"] += 1
            stat["last_error"] = str(exc)
            return
        self._image_info[card] = {"width": size[0], "height": size[1], "encoding": encoding,
                                  "source_width": source[0], "source_height": source[1]}
        self._emit_image(card, card, self.topics[card]["topic"], image_format, data, frame_id)

    # ── timer ────────────────────────────────────────────────────────────────

    def tick(self) -> None:
        self._run("gripper_lift_state", self._tick_gripper_lift)
        self._run("estop", self._publish_estop)
        self._link_watch()

    def _tick_gripper_lift(self) -> None:
        with self._lock:
            latest = dict(self._latest)
        if not latest or not self._due("gripper_lift_state", self._rates["gripper_lift_state"]):
            return
        now = self._mono()
        payload = {"stamp_ms": self._now_ms()}
        for key in ("left_gripper", "right_gripper", "lift"):
            if key in latest:
                seen, record = latest[key]
                payload[key] = {**record, "age_ms": int(round((now - seen) * 1000))}
            else:
                payload[key] = None
        self._emit("gripper_lift_state", payload)

    def _link_watch(self) -> None:
        """If the robot has never produced anything, say so loudly: that is the signature of
        a container whose CycloneDDS is not on the robot's loopback, and the symptom --
        topics visible, no data -- otherwise looks like a healthy idle driver."""
        if self.robot_link()["state"] != "never_received":
            return
        now = self._mono()
        if now - self._started >= 30 and now - self._link_logged >= 120:
            self._link_logged = now
            print("[panther] no data from the robot's DDS after "
                  f"{int(now - self._started)} s. Check that this container uses host "
                  "networking and CYCLONEDDS_URI binds 127.0.0.1 like the robot's own nodes, "
                  "and that the robot's controllers are running.", flush=True)

    # ── introspection ────────────────────────────────────────────────────────

    def robot_link(self) -> dict:
        received = [s["last_rx"] for c, s in self._stats.items()
                    if c not in ("estop",) and s["last_rx"] is not None]
        if not received:
            return {"state": "never_received", "uptime_s": round(self._mono() - self._started, 1),
                    "message": "尚未从机器人 DDS 收到任何数据：检查容器网络模式和 CYCLONEDDS_URI"}
        age = self._mono() - max(received)
        return {"state": "receiving" if age < 3.0 else "silent", "last_data_age_s": round(age, 2)}

    def _stale_after(self, card: str) -> float:
        if card in LASER_CARDS:
            return self._stale["laser_scan"]
        if card in CAMERA_CARDS:
            return self._stale["camera"]
        return self._stale[card]

    def _source_of(self, card: str) -> str:
        keys = {"joints": "joints", "battery": "battery", "imu": "imu", "odom": "odom",
                "health": "diagnostics", "estop": "diagnostics", "laser_scan_front": "laser_front",
                "laser_scan_rear": "laser_rear", "camera_color": "camera_color",
                "camera_depth": "camera_depth"}
        if card == "gripper_lift_state":
            return ", ".join(str(self.sources.get(k, "?")) for k in ("gripper_left", "gripper_right", "lift"))
        return str(self.sources.get(keys.get(card, ""), "?"))

    def stream(self, card: str) -> dict:
        stat = self._stats[card]
        age = None if stat["last_rx"] is None else round(self._mono() - stat["last_rx"], 2)
        fresh = age is not None and age <= self._stale_after(card)
        return {"source_topic": self._source_of(card), "received": stat["received"],
                "published": stat["published"], "dropped": stat["dropped"], "errors": stat["errors"],
                "last_error": stat["last_error"], "last_data_age_s": age, "fresh": fresh}

    def card_info(self, card: str) -> dict:
        result = {"state": "running" if self.is_active(card) else "idle",
                  "topic_out": self.topic_out(card), "stream": self.stream(card),
                  "robot_link": self.robot_link()}
        stream = result["stream"]
        # A camera that has not been started is deliberately not converted, so its silence says
        # nothing about the robot. Every other card publishes regardless and is judged on its feed.
        camera_off = card in CAMERA_CARDS and not self.is_active(card)
        # `start` must succeed even for a card with nothing to show (agent-core rolls the whole
        # project back on one failed start), so a missing feed is reported here, not by failing.
        if not camera_off:
            if stream["last_data_age_s"] is None:
                result["warning"] = NO_DATA_HINTS.get(
                    card, f"从未收到 {stream['source_topic']} 的数据")
            elif not stream["fresh"]:
                result["warning"] = (f"{stream['source_topic']} 已 {stream['last_data_age_s']}s "
                                     "没有新数据，下面的内容已过期")
        if camera_off:
            result["note"] = "相机卡片未启动，不转换图像（节省 CPU）；启动后才会发布"
        elif card in LASER_CARDS and not self.is_active(card):
            result["note"] = "俯视图在卡片启动后才发布；距离 JSON 始终发布"
        if card == "odom":
            result["odom_interface"] = build_interface(
                provides=["vx", "vy", "wz"], frame="body",
                rate_hz=self._rates["odom"], pose_drift="none")
        if card in CAMERA_CARDS:
            result["camera_info"] = [self.camera_info(card)]
            if card in self._image_info:
                result["image"] = self._image_info[card]
        if card in LASER_CARDS:
            scan = self._scans.get(card)
            if scan:
                result["scan"] = {k: scan[k] for k in ("frame_id", "beams", "bins", "valid_ratio",
                                                       "nearest_m", "range_min", "range_max")}
        return result

    def topic_out(self, card: str) -> list:
        outputs = [self.topics[card]]
        if card in self.views:
            outputs.append(self.views[card])
        return outputs

    def camera_info(self, card: str) -> dict:
        """Optics are not known for this camera, so they are declared unknown rather than guessed."""
        shown = self._image_info.get(card, {})
        depth = card == "camera_depth"
        return build_camera_info(
            topic=self.topics[card]["topic"], id=CAMERA_ID.format(card),
            format=self.topics[card]["format"],
            width=DEPTH_OUTPUT_SIZE[0] if depth else shown.get("width"),
            height=DEPTH_OUTPUT_SIZE[1] if depth else shown.get("height"),
            source="unknown",
            pipeline=[CAMERA_ID.format(card)] + (["unixai/panther/depth-resize"] if depth else []),
            vendor={"transport": "ros2", "source_topic": self._source_of(card),
                    "source_encoding": shown.get("encoding"),
                    "source_size": ([shown["source_width"], shown["source_height"]]
                                    if "source_width" in shown else None),
                    "camera": "Orbbec Gemini 2 RGB-D (per vendor manual)"})

    def urdf(self) -> str:
        return URDF_PATH.read_text(encoding="utf-8")


class PantherNodes:
    """The rclpy side: subscriptions on the robot domain only. Nothing is published to it."""

    def __init__(self, config: dict, ros2, hub: Hub):
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import BatteryState, Image, Imu, JointState, LaserScan

        self._ros2 = ros2
        # No rosout publisher and no parameter services: the driver adds nothing to the
        # robot's graph beyond its subscriptions.
        self.node = Node("panther_driver", context=ros2.ctx_robot, enable_rosout=False,
                         start_parameter_services=False)
        ros2.executor_robot.add_node(self.node)

        def qos(depth: int) -> QoSProfile:
            # Best-effort is compatible with a reliable publisher and cannot back-pressure one.
            return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)

        topics = hub.sources
        groups = {name: MutuallyExclusiveCallbackGroup() for name in
                  ("color", "depth", "laser_front", "laser_rear", "timer")}
        subscribe = self.node.create_subscription
        subscribe(JointState, topics["joints"], hub.on_joint_state, qos(1))
        subscribe(BatteryState, topics["battery"], hub.on_battery, qos(2))
        subscribe(Imu, topics["imu"], hub.on_imu, qos(10))
        subscribe(Odometry, topics["odom"], hub.on_odom, qos(10))
        subscribe(JointState, topics["gripper_left"], hub.on_gripper_left, qos(1))
        subscribe(JointState, topics["gripper_right"], hub.on_gripper_right, qos(1))
        subscribe(JointState, topics["lift"], hub.on_lift, qos(1))
        subscribe(DiagnosticArray, topics["diagnostics"], hub.on_diagnostics, qos(10))
        subscribe(LaserScan, topics["laser_front"], lambda m: hub.on_laser("front", m), qos(1),
                  callback_group=groups["laser_front"])
        subscribe(LaserScan, topics["laser_rear"], lambda m: hub.on_laser("rear", m), qos(1),
                  callback_group=groups["laser_rear"])
        subscribe(Image, topics["camera_color"], lambda m: hub.on_image("camera_color", m), qos(1),
                  callback_group=groups["color"])
        subscribe(Image, topics["camera_depth"], lambda m: hub.on_image("camera_depth", m), qos(1),
                  callback_group=groups["depth"])
        self._timer = self.node.create_timer(0.1, hub.tick, callback_group=groups["timer"])
        print("[panther] subscribed (read-only) to: " +
              ", ".join(sorted(str(t) for t in topics.values())), flush=True)

    def close(self) -> None:
        try:
            self._ros2.executor_robot.remove_node(self.node)
        except Exception:  # noqa: BLE001
            pass
        self.node.destroy_node()


class PantherPlugin:
    PREFIX = "panther"

    def __init__(self, hub: Hub, nodes=None, sink=None, enabled=None):
        self._hub = hub
        self._nodes = nodes
        self._sink = sink
        self._enabled = tuple(c for c in CARD_ORDER if enabled is None or enabled.get(c, True))

    def get_tools(self) -> list:
        tools = []
        for card in self._enabled:
            if card == "model":
                tools.append(tool("model", "resource", DESCRIPTIONS[card]))
                continue
            definition = tool(card, "sensor", DESCRIPTIONS[card], topic_out=self._hub.topic_out(card))
            if card in CAMERA_CARDS:
                definition["camera_info"] = [self._hub.camera_info(card)]
            tools.append(definition)
        return tools

    def start(self) -> None:
        pass  # subscriptions are created with the plugin; cards are started individually

    def stop(self) -> None:
        if self._nodes is not None:
            self._nodes.close()
        if self._sink is not None:
            self._sink.close()

    def dispatch(self, action, args):
        card = args.get("_tool_name", "")
        if card == "model" and "model" in self._enabled:
            return {"urdf": self._hub.urdf(), "robot": "mh3"}
        if card not in self._enabled or card == "model":
            return {"state": "error", "message": f"unknown card {card!r}"}
        if action == "start":
            self._hub.activate(card)
            return self._hub.card_info(card)
        if action == "stop":
            self._hub.deactivate(card)
            return {"state": "idle", "topic_out": self._hub.topic_out(card)}
        if action == "info" or action == card:
            return self._hub.card_info(card)
        return {"state": "error", "message": f"unknown action {action!r}; "
                                              "sensor cards take start, stop and info"}


def build_plugins(config, namespace=None, ros2=None):
    bridge_config = config.get("core_bridge", {})
    bridge = CoreBridge(bridge_config.get("profile", DEFAULT_PROFILE),
                        int(bridge_config.get("domain", 42)))
    sink = BridgeSink(bridge)
    hub = Hub(config, namespace or "panther", sink)
    nodes = PantherNodes(config, ros2, hub)
    enabled = {name: bool(spec.get("enabled", True))
               for name, spec in (config.get("plugins") or {}).items()}
    return [PantherPlugin(hub, nodes, sink, enabled)]
