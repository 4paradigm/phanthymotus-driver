#!/usr/bin/env python3
"""UniX AI Panther driver: sensor cards and the five motion cards.

The robot (ROS 2 Humble, CycloneDDS, domain 0, loopback-bound) is read and commanded in this
process; finished payloads are handed to `panther_bridge.CoreBridge`, whose child processes
publish them to Agent Core on domain 42. Robot command topics never cross that bridge.

`Hub` holds all the sensor logic and knows nothing about ROS, so it is tested with plain
objects. `panther_motion.Motion` holds the motion rules, also ROS-free. `PantherNodes` is the
thin rclpy layer that feeds the first and carries out the second (`RosMotionIO`).
`PantherPlugin` is the MCP surface.

There is no motion switch and no confirmation field: with intelligent control on, a motion card
call moves the robot, provided the robot's state allows it (fresh joints, a normal emergency-stop
diagnostic, the controller active) and every value is inside the range its box shows.
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
from contracts import HEAD_JOINTS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, parse_joint_limits, populate_switch_request
from host_camera import HostCamera
from panther_bridge import CoreBridge, DEFAULT_PROFILE, safe_text
from panther_motion import MOTION_CARDS, Motion, motion_tools
from panther_payloads import (
    DEPTH_FORMAT, DEPTH_OUTPUT_SIZE, StaticTf, UnsupportedImage, battery_display, battery_payload,
    diagnostic_entries, encode_color_jpeg, encode_depth_zlib, estop_display, estop_payload,
    health_payload, imu_payload, joint_display, joint_state_payload, laser_topdown_jpeg, merge_scans,
    odom_reading, single_joint,
)

URDF_PATH = Path(__file__).with_name("resource") / "panther.urdf"
CAMERA_ID = "unixai/panther/{}"

# Conversion work that is skipped until the card is started: the camera encodes, and the
# laser's top-down picture. Everything else is cheap and always publishes. Skipping matters
# for the cameras: a colour JPEG and a resampled depth frame every 100-200 ms are real load
# on a Jetson that is also running the robot.
CAMERA_CARDS = ("camera_color", "camera_depth")
SENSOR_CARDS = ("model", "joints", "imu", "odom", "robot_state", "laser_scan") + CAMERA_CARDS
CARD_ORDER = SENSOR_CARDS + MOTION_CARDS
STREAM_CARDS = tuple(c for c in SENSOR_CARDS if c != "model")

DESCRIPTIONS = {
    "model": "UniX AI Panther 运动学骨架（URDF，机器人名 mh3），供 joints 骨架渲染使用",
    "joints": "Panther 18 个关节（双臂 16 + 头部 2）实时角度，约 10Hz，3D 骨架。"
              "升降和夹爪不在此骨架内，见 robot_state",
    "imu": "Panther IMU：姿态四元数/欧拉角、角速度、线加速度，约 10Hz。传感器未提供的量为 null",
    "odom": "Panther 底盘速度里程计（机体坐标系 vx/vy/wz，不含位姿），约 10Hz",
    "robot_state": "Panther 低频状态汇总，约 2Hz：电池（电量/电压/电流/温度）、硬件诊断（CAN、电机等）、"
                   "急停诊断（厂商原文原样透传，附数据年龄和是否过期，stale 为 true 时不代表安全）、"
                   "左右夹爪与升降的原始读数、ros2_control 控制器状态、是否允许运动。每一块各带自己的 age_ms",
    "laser_scan": "Panther 激光雷达：以 base_link 为中心的最近障碍物距离 JSON，约 5Hz；启动后另发 1Hz 俯视图。"
                  "雷达安装位姿取自机器人的 /tf_static。驱动支持前后两个雷达并合成一圈，"
                  "但这台机器人只装了一个前雷达，后半圈被中间的立柱挡住，没有数据是正常的",
    "camera_color": "Panther 头部 RGB 相机（奥比中光 Gemini 2）彩色图像。启动卡片时在机器人本机拉起相机驱动",
    "camera_depth": "Panther 头部 RGB-D 相机深度图（640x480 uint16 毫米，zlib）。启动卡片时在机器人本机拉起相机驱动",
}

NO_DATA_HINTS = {
    "camera_color": "启动后仍未收到图像：检查机器人本机的 panther_orbbec.service"
                    "（`journalctl -u panther_orbbec`），或相机是否被别的程序占用",
    "camera_depth": "启动后仍未收到深度图：检查机器人本机的 panther_orbbec.service"
                    "（`journalctl -u panther_orbbec`），或相机是否被别的程序占用",
    "laser_scan": "没有收到任何雷达数据：前雷达话题没有数据（这台机器人只有前雷达；后雷达话题没有数据是正常的）",
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

        rates = {"joints": 10, "imu": 10, "odom": 10, "robot_state": 2, "laser_scan": 5,
                 "laser_view": 1, "camera_color": 10, "camera_depth": 5}
        rates.update(self._config.get("rates_hz", {}))
        self._rates = rates
        stale = {"joints": 1.0, "imu": 1.0, "odom": 1.0, "robot_state": 5.0, "laser_scan": 1.5,
                 "camera": 2.0, "battery": 5.0, "diagnostics": 5.0, "estop": 3.0,
                 "grippers": 1.0, "lift": 1.0, "controllers": 3.0}
        stale.update(self._config.get("stale_after_s", {}))
        self._stale = stale
        camera = self._config.get("camera", {})
        self._jpeg_quality = int(camera.get("jpeg_quality", 80))
        self._max_width = int(camera.get("max_width", 640))
        laser = self._config.get("laser", {})
        self._laser_points = int(laser.get("max_points", 360))
        self._view_range = float(laser.get("view_range_m", 6.0))
        self._base_frame = str(laser.get("base_frame", "base_link"))
        self._self_filter_radius = float(laser.get("self_filter_radius_m", 0.0))

        base = f"/{namespace}/" if namespace else "/"
        self.topics = {
            "joints": {"topic": base + "joints", "format": "sensor/skeleton"},
            "imu": {"topic": base + "imu", "format": "data/json"},
            "odom": {"topic": base + "state/odom", "format": "state/odom"},
            "robot_state": {"topic": base + "robot_state", "format": "data/json"},
            "laser_scan": {"topic": base + "laser_scan", "format": "data/json"},
            "camera_color": {"topic": base + "camera_color", "format": "image/jpeg"},
            "camera_depth": {"topic": base + "camera_depth", "format": "image/depth-zlib"},
        }
        self.views = {"laser_scan": {"topic": base + "laser_scan_view", "format": "image/jpeg"}}
        self.sources = dict(self._config.get("topics", {}))

        self._stats = {card: self._blank_stat() for card in STREAM_CARDS}
        self._active = {card: False for card in STREAM_CARDS}
        self._next: dict = {}
        self._latest: dict = {}                 # battery, gripper_left/right, lift: (seen, record)
        self._odom_burst: list = []
        self._odom_meta: dict = {}
        self._diag: dict = {}
        self._diag_seen = None
        self._diag_signature = None
        self._controllers = None                # (seen, {name: state})
        self._joint_pos = None                  # (seen, {name: position})
        self._live_limits = None
        self._scans: dict = {}                  # side -> (seen, msg)
        self._last_scan = None
        self._tf = StaticTf()
        self._tf_logged = False
        self._bundled_limits = None
        self._image_info: dict = {}
        self._unknown_joint_warned = False
        self._link_logged = 0.0
        self._image_log: dict = {}

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

    def _emit(self, card: str, payload: dict, lane: str = "state") -> None:
        # allow_nan=False turns a NaN that slipped past the payload code into an error here,
        # not into invalid JSON on the wire.
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        stat = self._stats[card]
        if self._sink.json(lane, self.topics[card]["topic"], text):
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
        positions = {}
        for name, value in zip(msg.name, msg.position):
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if number == number and abs(number) != float("inf"):
                positions[name] = number
        with self._lock:
            self._joint_pos = (self._mono(), positions)
        if not self._due("joints", self._rates["joints"]):
            return
        if not self._unknown_joint_warned:
            unknown = sorted(set(msg.name) - self._known_joints)
            if unknown:
                self._unknown_joint_warned = True
                print(f"[panther] /joint_states names not in the URDF are left out of the "
                      f"skeleton: {safe_text(unknown, 200)}", flush=True)
        self._emit("joints", joint_state_payload(msg, self._known_joints))

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

    # robot_state inputs: stored, then published by the timer (or at once on a diagnostics change)

    def on_battery(self, msg) -> None:
        self._run("robot_state", self._store, "battery", battery_payload, msg)

    def on_gripper_left(self, msg) -> None:
        self._run("robot_state", self._store, "gripper_left", single_joint, msg)

    def on_gripper_right(self, msg) -> None:
        self._run("robot_state", self._store, "gripper_right", single_joint, msg)

    def on_lift(self, msg) -> None:
        self._run("robot_state", self._store, "lift", single_joint, msg)

    def _store(self, key: str, shape, msg) -> None:
        self._rx("robot_state")
        record = shape(msg)
        with self._lock:
            self._latest[key] = (self._mono(), record)

    def on_controllers(self, states: dict) -> None:
        self._run("robot_state", self._controller_states, states)

    def _controller_states(self, states: dict) -> None:
        self._rx("robot_state")
        with self._lock:
            self._controllers = (self._mono(), dict(states))

    def on_diagnostics(self, msg) -> None:
        self._run("robot_state", self._diagnostics, msg)

    def _diagnostics(self, msg) -> None:
        self._rx("robot_state")
        received_ms = self._now_ms()
        with self._lock:
            self._diag_seen = self._mono()
            for entry in diagnostic_entries(msg):
                self._diag[entry["name"]] = {"entry": entry, "received_ms": received_ms}
            signature = tuple(sorted((n, r["entry"]["level"], r["entry"]["message"])
                                     for n, r in self._diag.items()))
            changed = signature != self._diag_signature
            self._diag_signature = signature
        if changed:
            # A level change, above all the emergency stop's, is worth sending now, not at the
            # next 500 ms slot.
            self._publish_state(force=True)

    def on_robot_description(self, msg) -> None:
        self._run("joints", self._robot_description, msg)

    def _robot_description(self, msg) -> None:
        limits = parse_joint_limits(msg.data)
        if not limits:
            return
        with self._lock:
            first = self._live_limits is None
            self._live_limits = limits
        if first:
            print(f"[panther] live /robot_description: {len(limits)} joint limits "
                  "(used to check motion targets)", flush=True)
            differ = self.limit_differences(limits)
            if differ:
                print("[panther] WARNING: the robot's joint limits differ from the bundled URDF that the "
                      "motion cards show as their ranges; commands are checked against the robot's. "
                      + safe_text("; ".join(differ), 400), flush=True)

    def bundled_joint_limits(self) -> dict:
        """The limits from the URDF shipped with the driver: what the motion cards show."""
        with self._lock:
            if self._bundled_limits is None:
                self._bundled_limits = parse_joint_limits(self.urdf())
            return dict(self._bundled_limits)

    def limit_differences(self, live: dict) -> list:
        """Where the robot's limits for the joints the cards show differ from the bundled ones."""
        bundled = self.bundled_joint_limits()
        shown = [*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS, *HEAD_JOINTS]
        out = []
        for joint in shown:
            if joint not in live:
                out.append(f"{joint}: not in the robot's URDF")
            elif joint in bundled and any(abs(a - b) > 1e-3 for a, b in zip(live[joint], bundled[joint])):
                out.append(f"{joint}: robot {live[joint][0]:g}~{live[joint][1]:g}, "
                           f"shown {bundled[joint][0]:g}~{bundled[joint][1]:g}")
        return out

    def on_tf_static(self, msg) -> None:
        self._run("laser_scan", self._tf_static, msg)

    def _tf_static(self, msg) -> None:
        with self._lock:
            kept = self._tf.add_message(msg)
            frames = len(self._tf)
        if kept and not self._tf_logged:
            self._tf_logged = True
            print(f"[panther] /tf_static: {frames} fixed frames known", flush=True)

    # ── robot_state ──────────────────────────────────────────────────────────

    def _aged(self, seen, record: dict, limit_key: str) -> dict:
        age = None if seen is None else self._mono() - seen
        return {**record, "age_ms": None if age is None else int(round(age * 1000)),
                "fresh": age is not None and age <= self._stale[limit_key]}

    def _state_payload(self) -> dict:
        """What the robot_state card shows. Trimmed for reading: the full readings stay in the
        hub (the motion gate and `lift_position` use those), only this view drops fields that
        are constant, always empty or duplicated on this robot."""
        now_ms = self._now_ms()
        with self._lock:
            latest, controllers = dict(self._latest), self._controllers
            diag, diag_seen = dict(self._diag), self._diag_seen
        battery = latest.get("battery")
        diagnostics = None
        if diag:
            age = self._mono() - diag_seen
            diagnostics = {**health_payload(diag, now_ms, int(self._stale["diagnostics"] * 1000)),
                           "age_ms": int(round(age * 1000)), "fresh": age <= self._stale["diagnostics"]}
        ready, reasons = self.motion_ready()
        grippers = {}
        for side in ("left", "right"):
            record = latest.get(f"gripper_{side}")
            grippers[side] = (self._aged(record[0], joint_display(record[1]), "grippers")
                              if record else None)
        lift = latest.get("lift")
        return {
            "stamp_ms": now_ms,
            "ready_for_motion": ready, "not_ready_reasons": reasons,
            "battery": (self._aged(battery[0], battery_display(battery[1]), "battery")
                        if battery else None),
            "diagnostics": diagnostics,
            "estop": estop_display(estop_payload(diag, now_ms, int(self._stale["estop"] * 1000))),
            "grippers": grippers,
            "lift": self._aged(lift[0], joint_display(lift[1]), "lift") if lift else None,
            "controllers": (self._aged(controllers[0], {"states": dict(sorted(controllers[1].items()))},
                                       "controllers") if controllers else None),
        }

    def _publish_state(self, force: bool = False) -> None:
        if force:
            self._mark("robot_state", self._rates["robot_state"])
        elif not self._due("robot_state", self._rates["robot_state"]):
            return
        self._emit("robot_state", self._state_payload())

    # ── the facts the motion gate needs ──────────────────────────────────────

    def motion_ready(self) -> tuple:
        """`(ready, reasons)`: fresh joints and a fresh, normal emergency-stop diagnostic.

        The emergency-stop test uses only the diagnostic's standard `level` (0 is OK) and its
        age. The vendor's text is never interpreted: only "normal" has ever been seen, so no
        other string is known to mean anything. The other diagnostics are not part of the test:
        a robot can report an unrelated error (an absent camera, say) and still be safe to move.
        """
        reasons = []
        now = self._mono()
        with self._lock:
            joints = self._joint_pos
            diag = dict(self._diag)
        if joints is None or now - joints[0] > self._stale["joints"]:
            reasons.append("no fresh /joint_states")
        estop = estop_payload(diag, self._now_ms(), int(self._stale["estop"] * 1000))
        if not estop["found"]:
            reasons.append("no emergency-stop diagnostic has been received")
        elif estop["stale"]:
            reasons.append("the emergency-stop diagnostic is stale")
        elif estop["level"] != 0:
            reasons.append(f"emergency-stop diagnostic level is {estop['level']} ({estop['level_name']}): "
                           f"{safe_text(estop['message'], 80)}")
        return not reasons, reasons

    def joint_positions(self, max_age_s: float | None = None):
        """`{joint: position}` if `/joint_states` is fresh, else None."""
        limit = self._stale["joints"] if max_age_s is None else max_age_s
        with self._lock:
            joints = self._joint_pos
        if joints is None or self._mono() - joints[0] > limit:
            return None
        return dict(joints[1])

    def lift_position(self):
        with self._lock:
            lift = self._latest.get("lift")
        if lift is None or self._mono() - lift[0] > self._stale["lift"]:
            return None
        return lift[1]["position"]

    def joint_limits(self) -> tuple:
        """`(limits, source)`: the robot's live `/robot_description` if it has been received,
        otherwise the bundled copy of the same vendor URDF."""
        with self._lock:
            if self._live_limits:
                return dict(self._live_limits), "live /robot_description"
        return self.bundled_joint_limits(), "bundled URDF"

    # ── laser ────────────────────────────────────────────────────────────────

    def on_laser(self, side: str, msg) -> None:
        self._run("laser_scan", self._laser, side, msg)

    def _laser(self, side: str, msg) -> None:
        self._rx("laser_scan")
        with self._lock:
            self._scans[side] = (self._mono(), msg)
        scan = None
        if self._due("laser_scan", self._rates["laser_scan"]):
            scan = self._merged_scan()
            if scan is not None:
                self._emit("laser_scan", scan)
        if self.is_active("laser_scan") and self._due("laser_scan_view", self._rates["laser_view"]):
            scan = scan or self._last_scan or self._merged_scan()
            if scan is None:
                return
            jpeg = laser_topdown_jpeg(scan, max_range_m=self._view_range)
            if jpeg is None:
                self._stats["laser_scan"]["last_error"] = "top-down view unavailable: OpenCV is not installed"
                return
            self._emit_image("laser_scan", "views", self.views["laser_scan"]["topic"], "jpeg", jpeg,
                             scan.get("frame_id", ""))

    def _merged_scan(self):
        """Front and rear lidars as one scan round `base_link`; None if neither is fresh.

        A side is left out, with the reason in `sources`, if it is stale or if no fixed
        transform from its frame to `base_link` is known. If *no* side has a transform yet, the
        first fresh one is shown in its own frame rather than nothing, and `transformed` says so.
        """
        now = self._mono()
        with self._lock:
            scans = dict(self._scans)
        sources, usable = {}, []
        for side in ("front", "rear"):
            record = scans.get(side)
            if record is None:
                sources[side] = {"used": False, "reason": "no data received"}
                continue
            seen, msg = record
            frame = str(getattr(getattr(msg, "header", None), "frame_id", ""))
            entry = {"frame_id": frame, "age_ms": int(round((now - seen) * 1000))}
            sources[side] = entry
            if now - seen > self._stale["laser_scan"]:
                entry.update(used=False, reason="stale")
                continue
            with self._lock:
                transform = self._tf.lookup(frame, self._base_frame)
            usable.append((side, msg, frame, transform))
        if not usable:
            return None
        placed = [u for u in usable if u[3] is not None]
        if placed:
            parts, target, transformed = [(u[1], u[3]) for u in placed], self._base_frame, True
            for side, _, frame, transform in usable:
                if transform is None:
                    sources[side].update(used=False, reason=f"no fixed transform from {frame} to {self._base_frame}")
        else:
            side, msg, frame, _ = usable[0]
            parts, target, transformed = [(msg, None)], frame, False
            sources[side]["note"] = (f"no /tf_static transform to {self._base_frame} yet; drawn in its "
                                     "own frame, so the robot's own body is not filtered out")
        used = {u[0] for u in (placed or usable[:1])}
        for side in used:
            sources[side]["used"] = True
        scan = merge_scans(parts, target, self._laser_points, self._self_filter_radius)
        scan.update(sources=sources, transformed=transformed, stamp_ms=self._now_ms())
        with self._lock:
            self._last_scan = scan
        return scan

    # ── images ───────────────────────────────────────────────────────────────

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
            # A frame silently dropped here looks exactly like a camera that is not running, so
            # it is counted and always visible in `info`. In the log it is a transition plus a
            # sample at most every 30 s: a stream that is wrong on every frame would otherwise
            # write a line per frame.
            stat = self._stats[card]
            stat["errors"] += 1
            changed = stat["last_error"] != str(exc)
            stat["last_error"] = str(exc)
            now = self._mono()
            if changed or now - self._image_log.get(card, -1e9) >= 30.0:
                self._image_log[card] = now
                print(f"[panther] {card}: {safe_text(exc, 200)} ({stat['errors']} frames dropped so far)",
                      flush=True)
            return
        self._image_info[card] = {"width": size[0], "height": size[1], "encoding": encoding,
                                  "source_width": source[0], "source_height": source[1]}
        self._emit_image(card, card, self.topics[card]["topic"], image_format, data, frame_id)

    # ── timer ────────────────────────────────────────────────────────────────

    def tick(self) -> None:
        self._run("robot_state", self._publish_state)
        self._link_watch()

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
        received = [s["last_rx"] for s in self._stats.values() if s["last_rx"] is not None]
        if not received:
            return {"state": "never_received", "uptime_s": round(self._mono() - self._started, 1),
                    "message": "尚未从机器人 DDS 收到任何数据：检查容器网络模式和 CYCLONEDDS_URI"}
        age = self._mono() - max(received)
        return {"state": "receiving" if age < 3.0 else "silent", "last_data_age_s": round(age, 2)}

    def _stale_after(self, card: str) -> float:
        return self._stale["camera"] if card in CAMERA_CARDS else self._stale[card]

    def _source_of(self, card: str) -> str:
        if card == "robot_state":
            return ", ".join(str(self.sources.get(k, "?")) for k in
                             ("battery", "diagnostics", "gripper_left", "gripper_right", "lift"))
        if card == "laser_scan":
            return ", ".join(str(self.sources.get(k, "?")) for k in ("laser_front", "laser_rear"))
        return str(self.sources.get(card, "?"))

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
        elif card == "laser_scan" and not self.is_active(card):
            result["note"] = "俯视图在卡片启动后才发布；距离 JSON 始终发布"
        if card == "odom":
            result["odom_interface"] = build_interface(
                provides=["vx", "vy", "wz"], frame="body",
                rate_hz=self._rates["odom"], pose_drift="none")
        if card in CAMERA_CARDS:
            result["camera_info"] = [self.camera_info(card)]
            if card in self._image_info:
                result["image"] = self._image_info[card]
        if card == "laser_scan":
            scan = self._last_scan
            if scan:
                result["scan"] = {k: scan[k] for k in ("frame_id", "beams", "bins", "valid_ratio",
                                                       "nearest_m", "range_min", "range_max",
                                                       "self_filter_radius_m", "self_filtered_beams",
                                                       "transformed", "sources")}
        if card == "robot_state":
            ready, reasons = self.motion_ready()
            result["ready_for_motion"], result["not_ready_reasons"] = ready, reasons
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


class RosMotionIO:
    """The ROS side of `panther_motion.Motion`: publishers, the controller-manager clients and the
    two trajectory action clients, all on the robot-domain node. Created only if a motion card is
    enabled, so a sensor-only configuration adds nothing but subscriptions to the robot's graph."""

    HEAD_TOPIC = "/head_forward_position_controller/commands"
    LIFT_TOPIC = "/lifting_controller/commands"
    GRIPPER_TOPIC = "/{side}_gripper/gripper_command"
    ARM_ACTION = "/{side}_arm_joint_trajectory_position_controller/follow_joint_trajectory"

    def __init__(self, node, config: dict, reliable_qos):
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._node = node
        self._results: dict = {}
        # The drive publisher is created when a move picks its topic: the robot's graph decides
        # which of the configured names is really listened to (see `select_twist_topic`).
        self._twist = None
        self._twist_topic = None
        self._reliable_qos = reliable_qos
        self._head = node.create_publisher(Float64MultiArray, self.HEAD_TOPIC, reliable_qos)
        self._lift = node.create_publisher(JointState, self.LIFT_TOPIC, reliable_qos)
        self._gripper_pub = None
        try:
            from unix_msgs.msg import GripperCommand
            self._gripper_pub = {side: node.create_publisher(
                GripperCommand, self.GRIPPER_TOPIC.format(side=side), reliable_qos)
                for side in ("left", "right")}
        except ImportError:
            print("[panther] unix_msgs is not installed: the gripper card cannot send commands", flush=True)
        self._list = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._switch = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self._arm = {side: ActionClient(node, FollowJointTrajectory, self.ARM_ACTION.format(side=side))
                     for side in ("left", "right")}
        self._poll_future = None

    @staticmethod
    def _wait(future, timeout_s: float):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            raise TimeoutError(f"ROS operation timed out after {timeout_s:g}s")
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    # reads

    def list_controllers(self) -> dict:
        from controller_manager_msgs.srv import ListControllers
        if not self._list.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/controller_manager/list_controllers is unavailable")
        result = self._wait(self._list.call_async(ListControllers.Request()), 3.0)
        return {entry.name: entry.state for entry in result.controller}

    def poll_controllers(self, deliver) -> None:
        """Non-blocking 1 Hz poll for robot_state; `deliver(states)` runs on the executor."""
        from controller_manager_msgs.srv import ListControllers
        if self._poll_future is not None or not self._list.service_is_ready():
            return
        future = self._list.call_async(ListControllers.Request())
        self._poll_future = future

        def done(finished):
            self._poll_future = None
            try:
                deliver({entry.name: entry.state for entry in finished.result().controller})
            except Exception as exc:  # noqa: BLE001
                print(f"[panther] controller poll failed: {type(exc).__name__}: {exc}", flush=True)
        future.add_done_callback(done)

    # commands

    def switch_controllers(self, activate: list, deactivate: list) -> None:
        from controller_manager_msgs.srv import SwitchController
        if not self._switch.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/controller_manager/switch_controller is unavailable")
        request = SwitchController.Request()
        populate_switch_request(request, activate, deactivate)
        if not self._wait(self._switch.call_async(request), 7.0).ok:
            raise RuntimeError("the controller manager rejected the controller switch")

    def twist_candidates(self, candidates: list) -> dict:
        """How many robot nodes subscribe to each candidate topic, right now."""
        return {topic: int(self._node.count_subscribers(topic)) for topic in candidates}

    def select_twist_topic(self, candidates: list) -> tuple:
        """`(topic, counts)`: the first candidate with a subscriber, creating its publisher.

        Nothing here assumes which name the robot listens on: the vendor SDK documents `/cmd_vel`,
        a teleoperation node may own `/cmd_vel_teleop`, and on a robot the configured name has to
        have a subscriber or the base does not move at all. Returns `(None, counts)` when none
        does, and the caller refuses rather than publishing into the void.
        """
        from geometry_msgs.msg import Twist

        counts = self.twist_candidates(candidates)
        for topic in candidates:
            if counts.get(topic):
                if self._twist is None or self._twist_topic != topic:
                    self._twist = self._node.create_publisher(Twist, topic, self._reliable_qos)
                    self._twist_topic = topic
                    print(f"[panther] base_drive publishing on {topic} "
                          f"({counts[topic]} subscriber(s); all: {counts})", flush=True)
                return topic, counts
        return None, counts

    def publish_twist(self, vx: float, vy: float, wz: float) -> None:
        """Publish on the topic `select_twist_topic` chose. Before any move chose one there is no
        publisher, so this sends nothing (and a stop has nothing to stop)."""
        from geometry_msgs.msg import Twist
        if self._twist is None:
            return
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
        self._twist.publish(message)

    def publish_head(self, values: list) -> None:
        from std_msgs.msg import Float64MultiArray
        message = Float64MultiArray()
        message.data = [float(v) for v in values]
        self._head.publish(message)

    def publish_lift(self, position: float) -> None:
        from sensor_msgs.msg import JointState
        message = JointState()
        message.name, message.position = [""], [float(position)]
        self._lift.publish(message)

    def publish_gripper(self, side: str, mode: str, position: float) -> None:
        from sensor_msgs.msg import JointState
        from unix_msgs.msg import GripperCommand
        if self._gripper_pub is None:
            raise RuntimeError("unix_msgs is not installed in this image")
        command = JointState()
        command.name, command.position = [""], [float(position)]
        message = GripperCommand()
        message.mode, message.command = mode, command
        self._gripper_pub[side].publish(message)

    def send_arm_goal(self, side: str, names: list, positions: list, duration_s: float):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        client = self._arm[side]
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"the {side} FollowJointTrajectory action server is unavailable")
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1.0) * 1e9))
        trajectory = JointTrajectory()
        trajectory.joint_names, trajectory.points = list(names), [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self._wait(client.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError(f"the {side} trajectory goal was rejected")
        return handle

    def poll_arm_result(self, handle, timeout_s: float):
        future = self._results.get(id(handle))
        if future is None:
            future = self._results[id(handle)] = handle.get_result_async()
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            return None
        self._results.pop(id(handle), None)
        wrapped = future.result()
        return {"status": int(wrapped.status), "error_code": int(wrapped.result.error_code),
                "error_string": str(wrapped.result.error_string)}

    def cancel_arm_goal(self, handle) -> None:
        self._results.pop(id(handle), None)
        handle.cancel_goal_async()


class PantherNodes:
    """The rclpy side: subscriptions on the robot domain, and `RosMotionIO` if motion is enabled."""

    def __init__(self, config: dict, ros2, hub: Hub):
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import BatteryState, Image, Imu, JointState, LaserScan
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self._ros2 = ros2
        # No rosout publisher and no parameter services: the driver adds nothing to the
        # robot's graph beyond what it is configured to use.
        self.node = Node("panther_driver", context=ros2.ctx_robot, enable_rosout=False,
                         start_parameter_services=False)
        ros2.executor_robot.add_node(self.node)

        def qos(depth: int) -> QoSProfile:
            # Best-effort is compatible with a reliable publisher and cannot back-pressure one.
            return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)

        def latched() -> QoSProfile:
            # /tf_static and /robot_description are published once and latched.
            return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

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
        subscribe(TFMessage, topics["tf_static"], hub.on_tf_static, latched())
        subscribe(String, topics["robot_description"], hub.on_robot_description, latched())
        self._timer = self.node.create_timer(0.1, hub.tick, callback_group=groups["timer"])

        self.motion_io = None
        plugins = config.get("plugins") or {}
        if any((plugins.get(card) or {}).get("enabled", False) for card in MOTION_CARDS):
            def reliable() -> QoSProfile:
                return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.RELIABLE)
            self.motion_io = RosMotionIO(self.node, config, reliable())
            self._poll = self.node.create_timer(1.0, lambda: self.motion_io.poll_controllers(hub.on_controllers),
                                                callback_group=groups["timer"])
        print("[panther] subscribed to: " + ", ".join(sorted(str(t) for t in topics.values())) +
              ("; motion cards enabled" if self.motion_io else "; no motion cards enabled"), flush=True)

    def close(self) -> None:
        try:
            self._ros2.executor_robot.remove_node(self.node)
        except Exception:  # noqa: BLE001
            pass
        self.node.destroy_node()


class PantherPlugin:
    PREFIX = "panther"

    def __init__(self, hub: Hub, nodes=None, sink=None, enabled=None, motion=None,
                 camera=None, config=None):
        self._hub = hub
        self._nodes = nodes
        self._sink = sink
        self._motion = motion
        self._camera = camera
        self._config = config or {}
        self._enabled = tuple(c for c in CARD_ORDER if enabled is None or enabled.get(c, True))

    def get_tools(self) -> list:
        tools = []
        motion_defs = ({t["name"]: t for t in motion_tools(self._config, self._hub.bundled_joint_limits())}
                       if self._motion else {})
        for card in self._enabled:
            if card in MOTION_CARDS:
                if card in motion_defs:
                    tools.append(motion_defs[card])
                continue
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
        # Stop anything moving first, then release the hardware, then the ROS node and bridge.
        if self._motion is not None:
            self._motion.stop_all()
        if self._camera is not None:
            self._camera.close()
        if self._nodes is not None:
            self._nodes.close()
        if self._sink is not None:
            self._sink.close()

    @staticmethod
    def _kind(card: str) -> str:
        return "color" if card == "camera_color" else "depth"

    def dispatch(self, action, args):
        card = args.get("_tool_name", "")
        if card == "model" and "model" in self._enabled:
            return {"urdf": self._hub.urdf(), "robot": "mh3"}
        if card not in self._enabled or card == "model":
            return {"state": "error", "message": f"unknown card {card!r}"}
        if card in MOTION_CARDS:
            if self._motion is None:
                return {"state": "error", "message": "motion is not available in this process"}
            if action == "stop":
                try:
                    return self._motion.stop(card)
                except Exception as exc:  # noqa: BLE001
                    return {"state": "error", "message": f"stop failed: {type(exc).__name__}: {exc}"}
            return self._motion.dispatch(card, action, {k: v for k, v in args.items() if k != "_tool_name"})
        if action == "start":
            self._hub.activate(card)
            camera = self._camera.start(self._kind(card)) if card in CAMERA_CARDS and self._camera else None
            info = self._hub.card_info(card)
            if camera is not None:
                info["camera_runtime"] = camera
            return info
        if action == "stop":
            self._hub.deactivate(card)
            if card in CAMERA_CARDS and self._camera:
                self._camera.stop(self._kind(card))
            return {"state": "idle", "topic_out": self._hub.topic_out(card)}
        if action == "info" or action == card:
            info = self._hub.card_info(card)
            if card in CAMERA_CARDS and self._camera:
                info["camera_runtime"] = self._camera.info(self._kind(card))
            return info
        return {"state": "error", "message": f"unknown action {action!r}; "
                                              "sensor cards take start, stop and info"}


def build_plugins(config, namespace=None, ros2=None):
    bridge_config = config.get("core_bridge", {})
    bridge = CoreBridge(bridge_config.get("profile", DEFAULT_PROFILE),
                        int(bridge_config.get("domain", 42)))
    sink = BridgeSink(bridge)
    hub = Hub(config, namespace or "panther", sink)
    nodes = PantherNodes(config, ros2, hub)
    motion = Motion(config, nodes.motion_io, hub) if nodes.motion_io is not None else None
    topics = hub.sources
    camera = HostCamera(
        config.get("camera", {}).get("host", {}),
        {"color": topics["camera_color"], "depth": topics["camera_depth"]},
        publisher_count=nodes.node.count_publishers,
        frame_age=lambda kind: hub.stream("camera_" + kind)["last_data_age_s"])
    enabled = {name: bool(spec.get("enabled", True))
               for name, spec in (config.get("plugins") or {}).items()}
    return [PantherPlugin(hub, nodes, sink, enabled, motion=motion, camera=camera, config=config)]
