"""Hex glove direct receiver or UDP preview and opt-in L20 Lite teleoperation."""

from __future__ import annotations

import json
import ipaddress
import math
import os
import socket
import subprocess
import sys
import threading
import time
import statistics
from collections import deque
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from glove_skeleton import GloveSkeleton
from modbus_client import JOINT_COUNT, _bytes
from modbus_device import HandPlugin, L20LiteConnection

LABELS = [
    "thumb_pitch",
    "thumb_side",
    "index_pitch",
    "middle_pitch",
    "ring_pitch",
    "pinky_pitch",
    "index_side",
    "ring_side",
    "pinky_side",
    "thumb_roll",
]
FORMAT = "sensor/skeleton"
SCHEMA = "linker.stroke10.flat.v1"


class SkeletonPublisher:
    def __init__(self, ros2, topic):
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        self._ros2 = ros2
        self._message = String
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._node = Node("glove_l20lite_preview", context=ros2.ctx_core)
        try:
            self._publisher = self._node.create_publisher(String, topic, qos)
            ros2.executor_core.add_node(self._node)
        except Exception:
            self._node.destroy_node()
            raise

    def publish(self, payload):
        message = self._message()
        message.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self._publisher.publish(message)

    def close(self):
        if self._node is not None:
            node, self._node = self._node, None
            try:
                self._ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()


class GloveTeleopPlugin:
    PREFIX = "glove_teleop"

    def __init__(self, hand, config, namespace, ros2, publisher_factory=None):
        self.hand = hand
        self.config = config
        self.topic = f"/{namespace.strip('/') or 'linkerhand'}/glove_teleop/skeleton"
        self.source_mode = str(
            os.environ.get("GLOVE_SOURCE_MODE") or config.get("source_mode", "direct")
        )
        if self.source_mode not in ("direct", "direct_udp", "udp"):
            raise ValueError("glove source_mode must be direct, direct_udp, or udp")
        self.glove_port = str(
            os.environ.get("GLOVE_SERIAL_PORT")
            or config.get("glove_port", "/dev/ttyUSB0")
        )
        self.source_ip = str(
            os.environ.get("GLOVE_SOURCE_IP") or config.get("source_ip", "127.0.0.1")
        )
        if ipaddress.ip_address(self.source_ip).version != 4:
            raise ValueError("GLOVE_SOURCE_IP must be an IPv4 address")
        self.listen_host = str(config.get("listen_host", "0.0.0.0"))
        self.listen_port = int(
            config.get(
                "listen_port", 19012 if self.source_mode == "direct_udp" else 19011
            )
        )
        self.max_hz = float(config.get("max_hz", 15))
        self.watchdog_s = float(config.get("watchdog_ms", 300)) / 1000
        self.max_delta = int(config.get("max_delta_per_step", 16))
        self.max_fraction_step = float(config.get("max_fraction_step", 0.05))
        self.open_deadband = float(config.get("open_deadband", 0.05))
        # Zero preserves single-frame capture for configurations predating this option.
        self.calibration_window_s = float(config.get("calibration_window_ms", 0)) / 1000
        self.calibration_stability_deg = float(config.get("calibration_stability_deg", 8))
        if not 0 <= self.open_deadband <= 0.2:
            raise ValueError("open_deadband must be in 0..0.2")
        if not ((self.calibration_window_s == 0 or 0.2 <= self.calibration_window_s <= 1)
                and 1 <= self.calibration_stability_deg <= 20):
            raise ValueError("invalid glove calibration window or stability limit")
        self._calibration_history = {}
        self.speed = int(config.get("speed", 50))
        self.torque = int(config.get("torque", 50))
        if not 1 <= self.listen_port <= 65535 or not 1 <= self.max_hz <= 30:
            raise ValueError("invalid glove UDP port or max_hz")
        if (
            not 0.1 <= self.watchdog_s <= 2
            or not 1 <= self.max_delta <= 255
            or not 0.01 <= self.max_fraction_step <= 0.25
        ):
            raise ValueError("invalid glove watchdog or step limit")
        if not 1 <= self.speed <= 255 or not 1 <= self.torque <= 255:
            raise ValueError("glove speed and torque must be 1..255")
        hand_cfg = hand.hand_config
        self.skeleton = GloveSkeleton(
            {
                "left": (hand_cfg["left_open_pose"], hand_cfg["left_fist_pose"]),
                "right": (hand_cfg["open_pose"], hand_cfg["fist_pose"]),
            }
        )
        self._publisher_factory = publisher_factory or (
            lambda topic: SkeletonPublisher(ros2, topic)
        )
        self._publisher = None
        self._socket = None
        self._process = None
        self._thread = None
        self._shutdown = threading.Event()
        self._lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._control_wake = threading.Event()
        self._cleanup_done = threading.Event()
        self._cleanup_done.set()
        self._control_thread = None
        self._watchdog_thread = None
        self._arming = False
        self._stopping = False
        self._pending_disarm = False
        self._session = 0
        self._last_command_stamp = None
        self._apply_started = None
        self._armed = False
        self._clients = {}
        self._last_command = {}
        self._last_frame = None
        self._last_received = 0.0
        self._last_sent = 0.0
        self._last_timestamp = None
        self._raw = {}
        self._filtered_pitch = {}
        self._pitch_candidates = {}
        self._relative_quaternions = {}
        self._filtered_quaternions = {}
        self._quaternion_candidates = {}
        self._quaternion_calibration = {}
        self._quaternion_preview_origin = {}
        self._mapped_fractions = {}
        self._preview_origin = {}
        self._side_received = {}
        self._calibration = {}
        self._active_sides = []
        self._error = None
        self._packets = 0
        self._commands = 0
        self._source_stamps = {}
        self._timing = {}
        self._reader_status = None
        self._last_watchdog_event = None
        self._arm_action = None
        self._arm_completion_failed = False
        self._last_arm_result = None

    def get_tools(self):
        schema = action_schema(
            {
                "start": ([], "启动手套直读和双手 3D 预览；不会驱动灵巧手"),
                "calibrate_open": (["side"], "自然张开指定手套并保持稳定约半秒，记录张开姿态"),
                "calibrate_fist": (
                    ["side"],
                    "先记录张开姿态，再保持指定手套握拳并记录角度",
                ),
                "arm": (["confirm_motion"], "确认安全后启用已校准且已连接的灵巧手遥操"),
                "disarm": ([], "立即停止遥操并释放输出力矩"),
                "info": ([], "查看连接、接收帧数、最近数据和错误"),
                "stop": ([], "停止接收和预览，并退出遥操"),
            },
            {
                "confirm_motion": {
                    "type": "boolean",
                    "const": True,
                    "description": "确认灵巧手周围安全后传入布尔值 true",
                },
                "side": {"type": "string", "enum": ["left", "right"]},
            },
        )
        schema["x-is-dangerous"] = True
        schema["allOf"] = [
            {
                "if": {"properties": {"action": {"const": "arm"}}},
                "then": {"required": ["confirm_motion"]},
            }
        ]
        definition = tool(
            self.PREFIX,
            "actuator",
            "Hex 数据手套直接读取、遥操 L20 Lite，并显示双手 3D 骨架",
            schema,
            topic_out=[{"topic": self.topic, "format": FORMAT}],
        )
        definition["title"] = "数据手套遥操 L20 Lite"
        schema["x-resource"] = ["l20_lite_left_hand", "l20_lite_right_hand"]
        schema["x-completion"] = {"actions": ["arm"], "timeout": 60}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "disarm"}}
        return [definition, tool("model", "resource", "L20 Lite 双手预览骨架 URDF")]

    def start(self):
        pass  # No glove receiver or hand is opened before the card's start action.

    def stop(self):
        self._stop()

    @staticmethod
    def parse_frame(packet):
        if len(packet) > 4096:
            raise ValueError("glove UDP frame is too large")
        data = json.loads(packet)
        if (
            data.get("schemaId") != SCHEMA
            or data.get("handType") != "LinkerHand/L20Lite"
        ):
            raise ValueError("expected Hex L20 Lite stroke10 broadcast")
        if data.get("dof") != JOINT_COUNT or data.get("labels") != LABELS:
            raise ValueError("unexpected L20 Lite joint layout")
        stamp = data.get("timestampMs")
        if type(stamp) is not int or stamp < 0:
            raise ValueError("timestampMs must be an integer")
        return {
            "timestampMs": stamp,
            "left": _bytes(data.get("leftHand")),
            "right": _bytes(data.get("rightHand")),
        }

    def _preview(self, frame):
        payload = {
            "format": FORMAT,
            "timestamp_ms": frame["timestampMs"],
            "source": (
                "Hex receiver direct"
                if self.source_mode != "udp"
                else "Hex L20 Lite stroke broadcast"
            ),
            "approximate": True,
            "preview_is_glove_target": True,
            "raw_positions": {side: frame[side] for side in ("left", "right")},
            "glove_angles": {
                side: sample["pitch"] for side, sample in self._raw.items()
            },
            "filtered_glove_angles": self._filtered_pitch.copy(),
            "orientation_source": {
                side: ("quaternion" if side in self._relative_quaternions else "euler")
                for side in self._raw
            },
            "mapped_fractions": {
                side: list(values) for side, values in self._mapped_fractions.items()
            },
            "joints": (
                self.skeleton.sample("left", frame["left"])
                + self.skeleton.sample("right", frame["right"])
            ),
        }
        self._publisher.publish(payload)

    @staticmethod
    def _angle_delta(value, origin):
        return (value - origin + 180) % 360 - 180

    @staticmethod
    def _quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]

    @classmethod
    def _relative_finger_quaternions(cls, quaternions):
        palm = quaternions[0]
        inverse = [-palm[0], -palm[1], -palm[2], palm[3]]
        return [
            cls._quaternion_multiply(inverse, finger) for finger in quaternions[1:6]
        ]

    @staticmethod
    def _root_top_quaternions(sample, return_sources=False):
        """Resolve the vendor's two channel layouts into palm + five roots."""
        if sample.get("quaternion_layout") != "hex-root-top-v1":
            raise ValueError("ambiguous glove quaternion layout; update the reader")
        roots, tips = sample.get("root_quaternions"), sample.get("tip_quaternions")
        if not all(isinstance(row, list) and len(row) == 6 for row in (roots, tips)):
            raise ValueError(
                "expected separate six-slot root and top quaternion arrays"
            )

        def unit(q):
            if q is None:
                return None
            if (
                not isinstance(q, list)
                or len(q) != 4
                or any(type(v) not in (float, int) or not math.isfinite(v) for v in q)
            ):
                raise ValueError("invalid glove quaternion")
            norm = math.sqrt(sum(v * v for v in q))
            return [v / norm for v in q] if 0.5 <= norm <= 1.5 else None

        selected = [unit(roots[0]), unit(roots[1])]
        sources = ["root"]
        for index in range(2, 6):
            root = unit(roots[index])
            selected.append(root or unit(tips[index]))
            sources.append("root" if root is not None else "top")
        if any(q is None for q in selected):
            raise ValueError("missing valid palm or finger root quaternion")
        # top[0] is a thumb tip, top[1] is unused here; neither is a palm/root.
        return (selected, sources) if return_sources else selected

    @staticmethod
    def _quaternion_log(quaternion):
        q = list(quaternion)
        if q[3] < 0:
            q = [-value for value in q]
        length = math.sqrt(sum(value * value for value in q[:3]))
        if length < 1e-8:
            return [0.0, 0.0, 0.0]
        angle = 2 * math.atan2(length, max(-1.0, min(1.0, q[3])))
        return [value * angle / length for value in q[:3]]

    @classmethod
    def _quaternion_delta_vector(cls, origin, value):
        inverse = [-origin[0], -origin[1], -origin[2], origin[3]]
        return cls._quaternion_log(cls._quaternion_multiply(inverse, value))

    @classmethod
    def _quaternion_fraction(cls, opened, closed, current):
        span = cls._quaternion_delta_vector(opened, closed)
        motion = cls._quaternion_delta_vector(opened, current)
        denominator = sum(value * value for value in span)
        if denominator < math.radians(5) ** 2:
            return None
        # The principal quaternion logarithm flips its axis at 180 degrees.
        # Choose the equivalent rotation vector nearest the calibrated segment
        # instead of turning a slightly deeper fist into a negative/open value.
        length = math.sqrt(sum(value * value for value in motion))
        candidates = [motion]
        if length > 1e-8:
            candidates += [
                [value * (length + turn) / length for value in motion]
                for turn in (-2 * math.pi, 2 * math.pi)
            ]

        def segment_distance(vector):
            fraction = max(
                0.0, min(1.0, sum(a * b for a, b in zip(vector, span)) / denominator)
            )
            return sum(
                (value - fraction * axis) ** 2 for value, axis in zip(vector, span)
            )

        motion = min(candidates, key=segment_distance)
        return max(
            0.0, min(1.0, sum(a * b for a, b in zip(motion, span)) / denominator)
        )

    def _filter_pitch(self, side, pitch, now):
        """Hold implausible ring/pinky jumps until they persist for 150 ms."""
        accepted = self._filtered_pitch.setdefault(side, list(pitch))
        candidates = self._pitch_candidates.setdefault(side, {})
        for finger in (3, 4):
            delta = self._angle_delta(pitch[finger], accepted[finger])
            if abs(delta) <= 15:
                accepted[finger] = pitch[finger]
                candidates.pop(finger, None)
                continue
            candidate = candidates.get(finger)
            if (
                candidate is None
                or abs(self._angle_delta(pitch[finger], candidate[0])) > 15
            ):
                candidates[finger] = (pitch[finger], now)
            elif now - candidate[1] >= 0.15:
                accepted[finger] += max(-15, min(15, delta))
        for finger in (0, 1, 2):
            accepted[finger] = pitch[finger]
        return list(accepted)

    def _filter_relative_quaternions(self, side, relative, now):
        """Apply the ring/pinky 150 ms jump confirmation in rotation space."""
        accepted = self._filtered_quaternions.setdefault(
            side, [list(q) for q in relative]
        )
        candidates = self._quaternion_candidates.setdefault(side, {})
        limit = math.radians(15)
        for finger, current in enumerate(relative):
            motion = self._quaternion_delta_vector(accepted[finger], current)
            distance = math.sqrt(sum(v * v for v in motion))
            if finger not in (3, 4) or distance <= limit:
                accepted[finger] = list(current)
                candidates.pop(finger, None)
                continue
            candidate = candidates.get(finger)
            difference = (
                self._quaternion_delta_vector(candidate[0], current)
                if candidate is not None
                else None
            )
            if candidate is None or math.sqrt(sum(v * v for v in difference)) > limit:
                candidates[finger] = (list(current), now)
            elif now - candidate[1] >= 0.15:
                half = min(distance, limit) / 2
                step = [v / distance * math.sin(half) for v in motion] + [
                    math.cos(half)
                ]
                accepted[finger] = self._quaternion_multiply(accepted[finger], step)
        return [list(q) for q in accepted]

    def _stabilize_fraction(self, side, finger, fraction):
        # Keep calibrated open/closed poses steady and ensure even a bad source
        # sample cannot make the preview or hand jump through its full stroke.
        if fraction >= 0.9:
            fraction = 1.0
        elif fraction <= self.open_deadband:
            fraction = 0.0
        state = self._mapped_fractions.setdefault(side, [fraction] * 5)
        previous = state[finger]
        value = max(
            previous - self.max_fraction_step,
            min(previous + self.max_fraction_step, fraction),
        )
        state[finger] = value
        return value

    def _map_direct(self, side, pitch):
        open_pose, fist_pose = self.skeleton.poses[side]
        target = list(open_pose)
        relative = self._filtered_quaternions.get(side)
        quaternion_calibration = self._quaternion_calibration.get(side, {})
        if relative is not None:
            opened = quaternion_calibration.get(
                "open", self._quaternion_preview_origin[side]
            )
            closed = quaternion_calibration.get("fist")
            for finger, joint in enumerate((0, 2, 3, 4, 5)):
                if closed is None:
                    motion = self._quaternion_delta_vector(
                        opened[finger], relative[finger]
                    )
                    fraction = min(
                        1.0,
                        math.sqrt(sum(value * value for value in motion))
                        / math.radians(75),
                    )
                else:
                    fraction = self._quaternion_fraction(
                        opened[finger], closed[finger], relative[finger]
                    )
                    if fraction is None:
                        fraction = 0.0
                fraction = self._stabilize_fraction(side, finger, fraction)
                target[joint] = max(
                    0,
                    min(
                        255,
                        round(
                            open_pose[joint]
                            + fraction * (fist_pose[joint] - open_pose[joint])
                        ),
                    ),
                )
            return target
        calibration = self._calibration.get(side, {})
        if "open" not in calibration or "fist" not in calibration:
            origin = calibration.get("open", self._preview_origin.get(side, pitch))
            for finger, joint in enumerate((0, 2, 3, 4, 5)):
                fraction = min(
                    1.0, abs(self._angle_delta(pitch[finger], origin[finger])) / 75
                )
                target[joint] = round(
                    open_pose[joint] + fraction * (fist_pose[joint] - open_pose[joint])
                )
            return target
        for finger, joint in enumerate((0, 2, 3, 4, 5)):
            origin = calibration["open"][finger]
            span = self._angle_delta(calibration["fist"][finger], origin)
            fraction = max(
                0.0, min(1.0, self._angle_delta(pitch[finger], origin) / span)
            )
            target[joint] = max(
                0,
                min(
                    255,
                    round(
                        open_pose[joint]
                        + fraction * (fist_pose[joint] - open_pose[joint])
                    ),
                ),
            )
        return target

    def accept_direct_sample(self, sample, now=None):
        """Consume an isolated reader process's validated glove angle sample."""
        now = time.monotonic() if now is None else now
        side = sample.get("side")
        pitch = sample.get("pitch")
        yaw = sample.get("yaw")
        quaternions = None
        quaternion_sources = None
        if any(
            key in sample
            for key in (
                "quaternion_layout",
                "root_quaternions",
                "tip_quaternions",
                "quaternions",
            )
        ):
            quaternions, quaternion_sources = self._root_top_quaternions(
                sample, return_sources=True
            )
        if side not in ("left", "right") or not all(
            isinstance(row, list)
            and len(row) == 5
            and all(
                type(value) in (float, int)
                and math.isfinite(value)
                and abs(value) <= 360
                for value in row
            )
            for row in (pitch, yaw)
        ):
            raise ValueError("invalid direct glove sample")
        with self._lock:
            if self._shutdown.is_set():
                return False
            if quaternions is None and self._quaternion_calibration.get(side):
                raise ValueError(
                    "missing root/top quaternions after quaternion calibration"
                )
            source_stamp = sample.get("timestamp_ms")
            if (
                type(source_stamp) is int
                and time.time() * 1000 - source_stamp > self.watchdog_s * 1000
            ):
                return False  # A queued old frame must not revive the motion watchdog.
            previous_received = self._side_received.get(side)
            if previous_received is not None:
                gap = max(0.0, (now - previous_received) * 1000)
                self._timing["callback_gap_ms"] = round(gap, 1)
                self._timing["max_callback_gap_ms"] = round(
                    max(gap, self._timing.get("max_callback_gap_ms", 0)), 1
                )
            stamp = sample.get("timestamp_ms")
            if type(stamp) is int:
                previous_stamp = self._source_stamps.get(side)
                self._source_stamps[side] = stamp
                if previous_stamp is not None:
                    gap = max(0, stamp - previous_stamp)
                    self._timing["source_gap_ms"] = gap
                    self._timing["max_source_gap_ms"] = max(
                        gap, self._timing.get("max_source_gap_ms", 0)
                    )
                self._timing["source_lag_ms"] = round(
                    max(0, time.time() * 1000 - stamp), 1
                )
            self._raw[side] = {
                "pitch": list(pitch),
                "yaw": list(yaw),
                "sequence": sample.get("sequence"),
            }
            self._reader_status = {
                "state": "streaming",
                "side": side,
                "observed_at_ms": int(time.time() * 1000),
                "sample_timestamp_ms": source_stamp,
            }
            if quaternions is not None:
                self._raw[side].update(
                    quaternion_sources=quaternion_sources,
                    root_quaternions=sample["root_quaternions"],
                    tip_quaternions=sample["tip_quaternions"],
                )
            self._filter_pitch(side, pitch, now)
            if quaternions is not None:
                self._relative_quaternions[side] = self._relative_finger_quaternions(
                    quaternions
                )
                self._quaternion_preview_origin.setdefault(
                    side, [list(q) for q in self._relative_quaternions[side]]
                )
                self._filter_relative_quaternions(
                    side, self._relative_quaternions[side], now
                )
            else:
                self._relative_quaternions.pop(side, None)
                self._filtered_quaternions.pop(side, None)
                self._quaternion_candidates.pop(side, None)
                self._quaternion_preview_origin.pop(side, None)
            self._preview_origin.setdefault(side, list(pitch))
            self._side_received[side] = now
            history = self._calibration_history.setdefault(side, deque(maxlen=128))
            history.append((now, list(pitch), (
                [list(q) for q in self._relative_quaternions[side]]
                if side in self._relative_quaternions else None
            )))
            while history and now - history[0][0] > max(self.calibration_window_s, 0.02):
                history.popleft()
            frame = {
                "timestampMs": max(
                    int(time.time() * 1000), (self._last_timestamp or 0) + 1
                )
            }
            for name in ("left", "right"):
                frame[name] = (
                    self._map_direct(name, self._filtered_pitch[name])
                    if name in self._raw
                    else list(self.skeleton.poses[name][0])
                )
        return self._accept_frame(frame, now)

    def accept_packet(self, packet, source_ip, now=None):
        """Parse one datagram, publish it, and command only when explicitly armed."""
        now = time.monotonic() if now is None else now
        if self._shutdown.is_set() or source_ip != self.source_ip:
            return False
        frame = self.parse_frame(packet)
        return self._accept_frame(frame, now)

    def _accept_frame(self, frame, now):
        with self._lock:
            if self._shutdown.is_set():
                return False
            if (
                self._last_timestamp is not None
                and frame["timestampMs"] <= self._last_timestamp
            ):
                return False
            self._last_timestamp = frame["timestampMs"]
            self._last_frame = frame
            self._last_received = now
            if self.source_mode == "udp":
                self._side_received = {"left": now, "right": now}
            self._packets += 1
            if self._publisher is not None:
                self._preview(frame)
            self._control_wake.set()
        return True

    def _ensure_control_threads_locked(self):
        for attribute, target, name in (
            ("_control_thread", self._control_loop, "glove-l20lite-control"),
            ("_watchdog_thread", self._watchdog_loop, "glove-l20lite-watchdog"),
        ):
            thread = getattr(self, attribute)
            if thread is None or not thread.is_alive():
                thread = threading.Thread(target=target, daemon=True, name=name)
                setattr(self, attribute, thread)
                thread.start()

    def _control_loop(self):
        try:
            while not self._shutdown.is_set():
                self._control_wake.wait(0.02)
                self._control_wake.clear()
                with self._lock:
                    cleanup = self._pending_disarm
                    due = (
                        self._armed
                        and self._last_timestamp != self._last_command_stamp
                        and time.monotonic() - self._last_sent >= 1.0 / self.max_hz
                    )
                if cleanup:
                    self._finish_disarm()
                elif due:
                    started = time.monotonic()
                    with self._lock:
                        self._apply_started = started
                    try:
                        self._apply_latest()
                    except Exception as exc:
                        with self._lock:
                            self._error = f"{type(exc).__name__}: {exc}"
                            self._request_disarm_locked()
                    finally:
                        with self._lock:
                            elapsed = (time.monotonic() - started) * 1000
                            self._timing["apply_ms"] = round(elapsed, 1)
                            self._timing["max_apply_ms"] = round(
                                max(elapsed, self._timing.get("max_apply_ms", 0)), 1
                            )
                            self._apply_started = None
        finally:
            self._finish_disarm()

    def _watchdog_loop(self):
        while not self._shutdown.wait(0.02):
            self._tick_watchdog()

    def _apply_latest(self):
        # Never hold the receive/state lock while waiting for the bus or UART.
        with self._io_lock:
            waiting = time.monotonic()
            with self.hand._bus_lock:
                with self._lock:
                    self._timing["bus_wait_ms"] = round(
                        (time.monotonic() - waiting) * 1000, 1
                    )
                    if (
                        not self._armed
                        or self._shutdown.is_set()
                        or self._last_timestamp == self._last_command_stamp
                    ):
                        return
                    session = self._session
                    sides = list(self._active_sides)
                    if any(
                        time.monotonic() - self._side_received.get(side, 0)
                        > self.watchdog_s
                        for side in sides
                    ):
                        return
                    stamp = self._last_timestamp
                    # Take the newest target AFTER acquiring the serial bus.
                    frame = {side: list(self._last_frame[side]) for side in sides}
                wrote = False
                for side in sides:
                    with self._lock:
                        if (
                            not self._armed
                            or session != self._session
                            or self._shutdown.is_set()
                            or time.monotonic() - self._side_received.get(side, 0)
                            > self.watchdog_s
                        ):
                            break
                        previous = self._last_command[side]
                        target = [
                            max(old - self.max_delta, min(old + self.max_delta, value))
                            for old, value in zip(previous, frame[side])
                        ]
                    started = time.monotonic()
                    self._clients[side].set_positions(target)
                    with self._lock:
                        self._timing["write_ms"] = round(
                            (time.monotonic() - started) * 1000, 1
                        )
                    self._last_command[side] = target
                    wrote = True
                if wrote:
                    with self._lock:
                        self._last_sent = time.monotonic()
                        self._last_command_stamp = stamp
                        self._commands += 1

    def _request_disarm_locked(self):
        # Disable targets immediately; the I/O owner releases torque as soon as
        # any already-in-flight transaction returns. No future target is queued.
        self._armed = False
        self._session += 1
        self._pending_disarm = True
        self._cleanup_done.clear()
        self._control_wake.set()

    def _finish_disarm(self):
        with self._io_lock, self.hand._bus_lock:
            for side, client in self._clients.items():
                started = time.monotonic()
                acknowledged = False
                try:
                    client.set_torques([0] * JOINT_COUNT)
                    acknowledged = True
                except Exception as exc:
                    with self._lock:
                        self._error = f"torque release failed: {exc}"
                finally:
                    finished = time.monotonic()
                    with self._lock:
                        age = self._side_received.get(side)
                    print("[glove-release] " + json.dumps({
                        "side": side,
                        "timestamp_ms": int(time.time() * 1000),
                        "zero_torque_acknowledged": acknowledged,
                        "release_io_ms": round((finished - started) * 1000, 1),
                        "last_sample_to_release_ms": (
                            round((finished - age) * 1000, 1) if age is not None else None
                        ),
                    }), flush=True)
                    try:
                        client.close()
                    except Exception as exc:
                        with self._lock:
                            self._error = f"serial close failed: {exc}"
            self._clients.clear()
            self._last_command.clear()
            with self._lock:
                self._armed = False
                self._active_sides.clear()
                self._pending_disarm = False
                self.hand._teleop_active = False
                self._cleanup_done.set()

    def _disarm(self):
        with self._lock:
            self._request_disarm_locked()
            worker = self._control_thread
        if worker is None or not worker.is_alive():
            self._finish_disarm()
        elif worker is not threading.current_thread():
            deadline = time.monotonic() + 5
            while not self._cleanup_done.wait(0.02):
                # Stop may race a worker that has just finished its final
                # cleanup and is exiting, after this request cleared the event.
                if not worker.is_alive():
                    self._finish_disarm()
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "teleop is disabled but serial cleanup is still pending"
                    )

    def _stable_calibration_pose(self, side, now):
        """Return robust endpoints only after a fresh, stable multi-frame window."""
        if self.calibration_window_s == 0:
            return (list(self._raw[side]["pitch"]),
                    self._relative_quaternions.get(side), 1)
        samples = [s for s in self._calibration_history.get(side, ())
                   if now - s[0] <= self.calibration_window_s]
        if len(samples) < 8 or samples[-1][0] - samples[0][0] < self.calibration_window_s * 0.8:
            return None
        angles = [samples[0][1][i] + statistics.median(
            self._angle_delta(s[1][i], samples[0][1][i]) for s in samples
        ) for i in range(5)]
        has_quaternions = samples[-1][2] is not None
        if any((s[2] is not None) != has_quaternions for s in samples):
            return None
        relative = [] if has_quaternions else None
        for finger in range(5):
            if has_quaternions:
                values = [s[2][finger] for s in samples]
                def distance(a, b):
                    return math.degrees(math.sqrt(sum(v*v for v in self._quaternion_delta_vector(a, b))))
                # A medoid handles q/-q equivalence and avoids averaging Euler angles.
                center = min(values, key=lambda q: sum(distance(q, v) for v in values))
                if max(distance(center, v) for v in values) > self.calibration_stability_deg:
                    return None
                relative.append(list(center))
            elif max(abs(self._angle_delta(s[1][finger], angles[finger])) for s in samples) > self.calibration_stability_deg:
                return None
        return angles, relative, len(samples)

    def _calibrate(self, side, pose):
        if self.source_mode == "udp":
            raise RuntimeError("pose calibration is only used with direct glove input")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        deadline = time.monotonic() + 2
        while True:
            with self._lock:
                if self._armed or self._arming or self._pending_disarm:
                    raise RuntimeError("disarm before changing glove calibration")
                calibration = self._calibration.setdefault(side, {})
                if pose == "fist" and "open" not in calibration:
                    raise RuntimeError(
                        "calibrate_open must be captured before calibrate_fist"
                    )
                if (
                    side in self._raw
                    and time.monotonic() - self._side_received[side] <= self.watchdog_s
                ):
                    captured = self._stable_calibration_pose(side, time.monotonic())
                    if captured is None:
                        if time.monotonic() >= deadline:
                            raise RuntimeError("hold the glove pose still for stable calibration")
                    else:
                        return self._store_calibration(side, pose, *captured)
            if time.monotonic() >= deadline:
                raise RuntimeError(f"no fresh {side} glove frame within 2 seconds")
            time.sleep(0.02)

    def _store_calibration(self, side, pose, angles, relative, sample_count):
        # Called with the state lock held; failed sampling never changes endpoints.
        calibration = self._calibration.setdefault(side, {})
        quaternion_calibration = self._quaternion_calibration.setdefault(
            side, {}
        )
        if pose == "open":
            calibration.clear()
            quaternion_calibration.clear()
            self._mapped_fractions[side] = [0.0] * 5
            self._preview_origin[side] = angles
            self._filtered_pitch[side] = list(angles)
            self._pitch_candidates.pop(side, None)
        calibration[pose] = angles
        if relative is not None:
            quaternion_calibration[pose] = [
                list(value) for value in relative
            ]
            self._filtered_quaternions[side] = [
                list(value) for value in relative
            ]
            self._quaternion_candidates.pop(side, None)
            if pose == "open":
                self._quaternion_preview_origin[side] = [
                    list(value) for value in relative
                ]
        if "open" in calibration and "fist" in calibration:
            if {"open", "fist"} <= quaternion_calibration.keys():
                small = [
                    index
                    for index in range(5)
                    if self._quaternion_fraction(
                        quaternion_calibration["open"][index],
                        quaternion_calibration["fist"][index],
                        quaternion_calibration["fist"][index],
                    )
                    is None
                ]
            else:
                small = [
                    index
                    for index in range(5)
                    if abs(
                        self._angle_delta(
                            calibration["fist"][index],
                            calibration["open"][index],
                        )
                    )
                    < 5
                ]
            if small:
                calibration.pop("fist", None)
                quaternion_calibration.pop("fist", None)
                raise RuntimeError(
                    f"glove flexion changed less than 5 degrees on fingers {small}; repeat poses"
                )
            self._filtered_pitch[side] = list(angles)
            self._pitch_candidates.pop(side, None)
            self._mapped_fractions[side] = [1.0] * 5
        return {
            "side": side,
            "captured": pose,
            "sample_count": sample_count,
            "ready": "open" in calibration and "fist" in calibration,
            "pitch": angles,
            "orientation_source": (
                "quaternion" if relative is not None else "euler"
            ),
        }

    def _reserve_arm(self):
        preview = os.environ.get("GLOVE_PREVIEW_ONLY")
        if preview not in (None, "0", "1"):
            raise ValueError("GLOVE_PREVIEW_ONLY must be 0 or 1")
        preview_only = (
            preview == "1"
            if preview is not None
            else self.config.get("preview_only", True)
        )
        if preview_only:
            raise RuntimeError(
                "默认仅骨架预览；完成时序验收后设置 GLOVE_PREVIEW_ONLY=0，"
                "重新创建容器，再显式 arm(confirm_motion=true)"
            )
        with self._lock:
            if self._armed:
                raise RuntimeError("already armed; disarm before arming again")
            if self._arm_completion_failed:
                raise RuntimeError("ACP completion delivery failed; restore Core and disarm to retry notification")
            if self._arming or self._pending_disarm or not self._cleanup_done.is_set():
                raise RuntimeError("teleop setup or cleanup is still pending")
            if self._shutdown.is_set():
                raise RuntimeError("start preview before arming")
            if (
                self._socket is None and self._thread is None
            ) or self._last_frame is None:
                raise RuntimeError(
                    "start preview and receive a valid glove frame first"
                )
            if time.monotonic() - self._last_received > self.watchdog_s:
                raise RuntimeError("glove stream is stale")
            self._arming = True
            self._session += 1
            session = self._session
            self.hand._teleop_active = True
            return session

    # Reuse the existing retry/acknowledgement contract with this tool's PREFIX.
    _acp_callback = HandPlugin._acp_callback

    def _start_arm(self):
        with self._lock:
            if self._arm_action is not None:
                raise RuntimeError("another arm action is pending")
            session = self._reserve_arm()
            active = {"action_id": f"glove_arm_{uuid4().hex}", "session": session}
            self._arm_action = active
            try:
                threading.Thread(target=self._arm_worker, args=(active,),
                                 name=active["action_id"], daemon=True).start()
            except Exception:
                self._arm_action = None
                self._arming = False
                self.hand._teleop_active = False
                self._session += 1
                raise
        return {"state": "running", "action_id": active["action_id"]}

    def _arm_worker(self, active):
        try:
            result = self._arm(session=active["session"])
            status = "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc)}
        except Exception as exc:
            status, result = "error", {"state": "error", "message": str(exc)}
        terminal = {"action_id": active["action_id"], "status": status, "result": result}
        with self._lock:
            self._last_arm_result = terminal
        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            with self._lock:
                self._arm_completion_failed = True
                terminal["callback_error"] = str(exc)
            # Core cannot confirm ownership: do not leave streaming torque enabled.
            try:
                self._disarm()
            except Exception as cleanup_error:
                terminal["cleanup_error"] = str(cleanup_error)
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)
        finally:
            with self._lock:
                if self._arm_action is active:
                    self._arm_action = None

    def _retry_arm_completion(self):
        """Retry only a terminal notification, never hardware setup or movement."""
        with self._lock:
            if not self._arm_completion_failed or self._arm_action is not None:
                return False
            terminal = self._last_arm_result
            active = {"action_id": terminal["action_id"]}
            self._arm_action = active

        def retry():
            try:
                self._acp_callback(terminal["action_id"], terminal["status"], terminal["result"])
            except Exception as exc:
                with self._lock:
                    terminal["callback_error"] = str(exc)
            else:
                with self._lock:
                    self._arm_completion_failed = False
                    terminal.pop("callback_error", None)
            finally:
                with self._lock:
                    if self._arm_action is active:
                        self._arm_action = None

        try:
            threading.Thread(target=retry, name="glove-arm-completion-retry", daemon=True).start()
        except Exception:
            with self._lock:
                if self._arm_action is active:
                    self._arm_action = None
            raise
        return True

    def _arm(self, session=None):
        # Private synchronous setup is owned exclusively by the ACP worker.
        if session is None:
            session = self._reserve_arm()
        arm_started = time.monotonic()

        def check_setup():
            with self._lock:
                if session != self._session or self._shutdown.is_set():
                    raise InterruptedError("teleop setup was cancelled")
                if time.monotonic() - arm_started > 20:
                    raise TimeoutError("teleop setup exceeded 20 seconds")

        try:
            check_setup()
            # Discovery and all setup I/O run without the frame-receive lock.
            found = self.hand._discover()
            with self._io_lock, self.hand._bus_lock:
                check_setup()
                controllers = [self.hand._legacy, *self.hand._controllers.values()]
                if any(controller._active is not None for controller in controllers):
                    raise RuntimeError("an L20 Lite hand action is still running")
                gesture = getattr(self.hand, "_gesture_plugin", None)
                if gesture is not None and gesture._active is not None:
                    raise RuntimeError("an L20 Lite gesture is still running")
                with self._lock:
                    if self.source_mode == "udp":
                        if set(found) != {"left", "right"}:
                            raise RuntimeError(
                                f"both L20 Lite hands are required; found {sorted(found)}"
                            )
                        sides = ["left", "right"]
                    else:
                        sides = [
                            side
                            for side in ("left", "right")
                            if side in found
                            and side in self._raw
                            and side in self._calibration
                            and {"open", "fist"} <= self._calibration[side].keys()
                            and time.monotonic() - self._side_received[side]
                            <= self.watchdog_s
                        ]
                        if not sides:
                            raise RuntimeError(
                                "no calibrated glove and matching L20 Lite hand with fresh data"
                            )
                for controller in controllers:
                    controller.connection.close()
                if gesture is not None:
                    for connection in gesture._connections.values():
                        connection.close()
                for side in sides:
                    check_setup()
                    client = L20LiteConnection(
                        {**self.hand.config, "side": side, "serial_port": found[side]}
                    ).get()
                    self._clients[side] = client
                    current = client.get_positions()
                    self._last_command[side] = current
                    check_setup()
                    client.set_speeds([self.speed] * JOINT_COUNT)
                    check_setup()
                    client.set_positions(current)
                    check_setup()
                    client.set_torques([self.torque] * JOINT_COUNT)
                with self._lock:
                    check_setup()
                    if any(
                        time.monotonic() - self._side_received.get(side, 0)
                        > self.watchdog_s
                        for side in sides
                    ):
                        raise RuntimeError(
                            "glove stream became stale during teleop setup"
                        )
                    self._last_sent = 0.0
                    self._last_command_stamp = self._last_timestamp
                    self._armed = True
                    self._active_sides = sides
                    self._error = None
                    self._timing.clear()
                    self._timing["arm_ms"] = round(
                        (time.monotonic() - arm_started) * 1000, 1
                    )
                    self._ensure_control_threads_locked()
            return {
                "state": "armed",
                "sides": sides,
                "serial_ports": {side: found[side] for side in sides},
            }
        except Exception:
            self._disarm()
            raise
        finally:
            with self._lock:
                self._arming = False

    def _receive_loop(self, receiver):
        while not self._shutdown.is_set():
            try:
                packet, addr = receiver.recvfrom(4097)
                if self.source_mode == "direct_udp":
                    if addr[0] == self.source_ip and len(packet) <= 4096:
                        sample = json.loads(packet)
                        if "side" in sample:
                            self.accept_direct_sample(sample)
                        elif "error" in sample:
                            with self._lock:
                                self._error = str(sample["error"])
                        elif sample.get("state") in ("connected", "waiting_for_glove"):
                            with self._lock:
                                self._reader_status = {
                                    **sample,
                                    "observed_at_ms": int(time.time() * 1000),
                                }
                else:
                    self.accept_packet(packet, addr[0])
            except socket.timeout:
                pass
            except OSError:
                if self._shutdown.is_set():
                    break
            except Exception as exc:
                with self._lock:
                    self._error = f"glove stream error: {type(exc).__name__}: {exc}"
                    if self._armed:
                        self._request_disarm_locked()
            self._tick_watchdog()

    def _tick_watchdog(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._armed and any(
                now - self._side_received.get(side, 0) > self.watchdog_s
                for side in self._active_sides
            ):
                self._last_watchdog_event = {
                    "timestamp_ms": int(time.time() * 1000),
                    "side_age_ms": {
                        side: round((now - self._side_received.get(side, 0)) * 1000, 1)
                        for side in self._active_sides
                    },
                    "received_frames": self._packets,
                    "sent_commands": self._commands,
                    "apply_in_progress_ms": (
                        round((now - self._apply_started) * 1000, 1)
                        if self._apply_started is not None
                        else None
                    ),
                    "timing": dict(self._timing),
                    "reader_status": self._reader_status,
                }
                self._error = "glove stream timed out"
                self._request_disarm_locked()
                print(
                    "[glove-watchdog] " + json.dumps(self._last_watchdog_event),
                    flush=True,
                )

    def _direct_loop(self):
        worker = os.path.join(os.path.dirname(__file__), "glove_direct_reader.py")
        while not self._shutdown.is_set():
            env = os.environ.copy()
            try:
                process = subprocess.Popen(
                    [sys.executable, "-u", worker, "--port", self.glove_port],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                    env=env,
                )
                with self._lock:
                    self._process = process
                for line in process.stdout:
                    if self._shutdown.is_set():
                        break
                    try:
                        sample = json.loads(line)
                        if "error" in sample:
                            with self._lock:
                                self._error = sample["error"]
                        elif "side" in sample:
                            self.accept_direct_sample(sample)
                        elif sample.get("state") in ("connected", "waiting_for_glove"):
                            with self._lock:
                                self._reader_status = {
                                    **sample,
                                    "observed_at_ms": int(time.time() * 1000),
                                }
                    except Exception as exc:
                        with self._lock:
                            self._error = f"direct glove sample error: {type(exc).__name__}: {exc}"
                            if self._armed:
                                self._request_disarm_locked()
                    self._tick_watchdog()
                process.wait(timeout=1)
            except Exception as exc:
                with self._lock:
                    self._error = (
                        f"direct glove reader error: {type(exc).__name__}: {exc}"
                    )
            finally:
                with self._lock:
                    if self._process is not None:
                        if self._process.poll() is None:
                            self._process.terminate()
                        self._process = None
                self._tick_watchdog()
            if not self._shutdown.is_set():
                for _ in range(20):
                    if self._shutdown.wait(0.05):
                        break
                    self._tick_watchdog()

    def _start(self):
        with self._lock:
            if self._stopping:
                raise RuntimeError("previous teleop session is still shutting down")
            if self.source_mode == "direct":
                if self._thread is None:
                    if not os.path.exists(self.glove_port):
                        raise RuntimeError(
                            f"glove receiver port is missing: {self.glove_port}"
                        )
                    self._publisher = self._publisher_factory(self.topic)
                    self._shutdown.clear()
                    self._thread = threading.Thread(
                        target=self._direct_loop,
                        daemon=True,
                        name="glove-l20lite-direct",
                    )
                    self._thread.start()
                return {
                    "state": "running",
                    "armed": self._armed,
                    "source_mode": "direct",
                    "glove_port": self.glove_port,
                    "topic_out": [{"topic": self.topic, "format": FORMAT}],
                }
            if self._socket is None:
                receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                receiver.settimeout(0.05)
                try:
                    receiver.bind((self.listen_host, self.listen_port))
                    publisher = self._publisher_factory(self.topic)
                except Exception:
                    receiver.close()
                    raise
                self._socket = receiver
                self._publisher = publisher
                self._shutdown.clear()
                self._thread = threading.Thread(
                    target=self._receive_loop,
                    args=(receiver,),
                    daemon=True,
                    name="glove-l20lite-udp",
                )
                self._thread.start()
            return {
                "state": "running",
                "armed": self._armed,
                "source_mode": self.source_mode,
                "topic_out": [{"topic": self.topic, "format": FORMAT}],
                "listen_port": self.listen_port,
                "source_ip": self.source_ip,
            }

    def _stop(self):
        with self._lock:
            self._stopping = True
            self._shutdown.set()
            self._control_wake.set()
            receiver, self._socket = self._socket, None
            process, self._process = self._process, None
            thread, self._thread = self._thread, None
        if receiver is not None:
            receiver.close()
        if process is not None and process.poll() is None:
            process.terminate()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2)
        self._disarm()
        for worker in (self._control_thread, self._watchdog_thread):
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=5)
                if worker.is_alive():
                    raise RuntimeError("teleop worker is still shutting down")
        with self._lock:
            self._control_thread = None
            self._watchdog_thread = None
            publisher, self._publisher = self._publisher, None
            if publisher is not None:
                publisher.close()
            self._last_frame = None
            self._last_timestamp = None
            self._raw.clear()
            self._calibration_history.clear()
            self._filtered_pitch.clear()
            self._pitch_candidates.clear()
            self._relative_quaternions.clear()
            self._filtered_quaternions.clear()
            self._quaternion_candidates.clear()
            self._quaternion_calibration.clear()
            self._quaternion_preview_origin.clear()
            self._mapped_fractions.clear()
            self._preview_origin.clear()
            self._side_received.clear()
            self._calibration.clear()
            self._source_stamps.clear()
            self._timing.clear()
            self._reader_status = None
            self._stopping = False
        return {"state": "idle", "armed": False}

    def dispatch(self, action, args):
        if action == "model":
            return {
                "urdf": self.skeleton.urdf,
                "approximate": True,
                "description": "Both hands; mirrored placement is approximate",
            }
        if action == "start":
            return self._start()
        if action in ("calibrate_open", "calibrate_fist"):
            return self._calibrate(args.get("side"), action.removeprefix("calibrate_"))
        if action == "arm":
            if args.get("confirm_motion") is not True:
                raise ValueError("arm requires boolean confirm_motion=true")
            return self._start_arm()
        if action == "disarm":
            self._disarm()
            retrying = self._retry_arm_completion()
            return {"state": "running", "armed": False, "completion_retry": retrying}
        if action == "stop":
            return self._stop()
        if action == "info":
            with self._lock:
                age = (
                    time.monotonic() - self._last_received if self._last_frame else None
                )
                return {
                    "state": "running" if self._socket or self._thread else "idle",
                    "armed": self._armed,
                    "source_mode": self.source_mode,
                    "watchdog_ms": self.watchdog_s * 1000,
                    "preview_only_config": self.config.get("preview_only", True),
                    "preview_only_override": os.environ.get("GLOVE_PREVIEW_ONLY"),
                    "arming": self._arming,
                    "arm_action_id": self._arm_action["action_id"] if self._arm_action else None,
                    "last_arm_result": self._last_arm_result,
                    "arm_completion_delivery_failed": self._arm_completion_failed,
                    "cleanup_pending": self._pending_disarm,
                    "glove_port": (
                        self.glove_port if self.source_mode != "udp" else None
                    ),
                    "source_ip": (
                        self.source_ip if self.source_mode != "direct" else None
                    ),
                    "listen_port": self.listen_port,
                    "stream_age_ms": round(age * 1000) if age is not None else None,
                    "received_frames": self._packets,
                    "sent_commands": self._commands,
                    "glove_angles": self._raw,
                    "filtered_glove_angles": self._filtered_pitch.copy(),
                    "quaternion_debug": {
                        side: {
                            "relative": self._relative_quaternions.get(side),
                            "filtered": self._filtered_quaternions.get(side),
                            "calibration": self._quaternion_calibration.get(side, {}),
                        }
                        for side in self._relative_quaternions
                    },
                    "orientation_source": {
                        side: (
                            "quaternion"
                            if side in self._relative_quaternions
                            else "euler"
                        )
                        for side in self._raw
                    },
                    "mapped_fractions": {
                        side: list(values)
                        for side, values in self._mapped_fractions.items()
                    },
                    "calibrated_sides": [
                        side
                        for side, calibration in self._calibration.items()
                        if {"open", "fist"} <= calibration.keys()
                    ],
                    "last_raw": (
                        {side: self._last_frame[side] for side in ("left", "right")}
                        if self._last_frame
                        else None
                    ),
                    "error": self._error,
                    "timing": dict(self._timing),
                    "reader_status": self._reader_status,
                    "last_watchdog_event": self._last_watchdog_event,
                    "topic_out": [{"topic": self.topic, "format": FORMAT}],
                }
        return None
