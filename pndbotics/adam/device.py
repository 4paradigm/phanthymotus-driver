"""PNDbotics Adam driver — plugin classes.

Plugins:
  StatePlugin  — DDS rt/lowstate → ROS2 skeleton/IMU/battery
  EStopPlugin  — read-only PAC physical emergency-stop state
  LocoPlugin   — gRPC locomotion control
  PosturePlugin / MotionPlugin / TrackingMotionPlugin — focused RL execution cards
  ArmPlugin    — DDS rt/lowcmd upper body control
  HandPlugin   — DDS rt/handcmd finger control and hand-state query
  ModelPlugin  — URDF resource for 3D visualization
"""

from __future__ import annotations

import json
import io
import math
import os
import queue
import re
import struct
import subprocess
import sys
import threading
import time
import uuid
import zlib
from pathlib import Path

import numpy as np

from estop import EStopPlugin
try:
    from common import lifecycle as _lifecycle
except ImportError:  # a checkout rather than the container image, where
    # common/ is copied in beside this file. Load-bearing, so it resolves the
    # repo root rather than degrading to a no-op the way logsafe does.
    import sys as _sys, pathlib as _pathlib
    _sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parents[2]))
    from common import lifecycle as _lifecycle

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    HAS_ROS2 = True
except Exception:
    HAS_ROS2 = False

    class Node:
        """Import-time fallback so DDS-only cards can load without ROS2."""

        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is unavailable")

    QoSProfile = None
    ReliabilityPolicy = None
    HistoryPolicy = None
    JointState = None
    String = None

try:
    from pndbotics_sdk_py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import (
        LowState_,
        LowCmd_,
        HandCmd_,
    )
    from pndbotics_sdk_py.idl.default import (
        pnd_adam_msg_dds__HandCmd_,
        pnd_adam_msg_dds__LowCmd_,
    )

    HAS_PND_SDK = True
except Exception:
    HAS_PND_SDK = False


def _notify_action_completion(action_id, status, result, tool):
    """Report completion of a long-running MCP action to Agent Core."""
    import ssl
    import urllib.request

    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    request = urllib.request.Request(
        f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')}/api/acp/complete",
        data=payload, headers={"Content-Type": "application/json"})
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    try:
        urllib.request.urlopen(request, context=context, timeout=5).close()
    except Exception as exc:
        print(f"[{tool}] ACP completion callback failed: {exc}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Joint definitions per variant
# ---------------------------------------------------------------------------

ADAM_LITE_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
]

ADAM_SP_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

ADAM_PRO_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "neckYaw", "neckPitch",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

VARIANT_JOINTS = {
    "lite": ADAM_LITE_JOINTS,
    "sp": ADAM_SP_JOINTS,
    "pro": ADAM_PRO_JOINTS,
}

VARIANT_DOF = {"lite": 23, "sp": 29, "pro": 31}

# Adam hand indices are the same for the two hands.  Each hand has five
# physical fingers but six motor channels: the thumb has flexion and rotation
# channels.  The first six values are the left hand and the last six are right.
HAND_CHANNEL_NAMES = (
    "pinky", "ring", "middle", "index", "thumb_flex", "thumb_rotate",
)
HAND_CHANNEL_LABELS = {
    "pinky": "小指",
    "ring": "无名指",
    "middle": "中指",
    "index": "食指",
    "thumb_flex": "拇指屈伸",
    "thumb_rotate": "拇指旋转",
}
HAND_POSITION_COUNT = 12
HAND_POSITION_MIN = 0
HAND_POSITION_MAX = 1000
HAND_DEFAULT_OPEN = [
    HAND_POSITION_MAX, HAND_POSITION_MAX, HAND_POSITION_MAX, HAND_POSITION_MAX,
    HAND_POSITION_MAX, HAND_POSITION_MIN,
    HAND_POSITION_MAX, HAND_POSITION_MAX, HAND_POSITION_MAX, HAND_POSITION_MAX,
    HAND_POSITION_MAX, HAND_POSITION_MIN,
]
HAND_DEFAULT_CLOSED = [0] * HAND_POSITION_COUNT
HAND_DEFAULT_THUMB_CLOSE = [100, 1000, 100, 1000]


def _coerce_hand_positions(values, *, limit: int, expected: int = HAND_POSITION_COUNT) -> list[int]:
    """Validate and clamp a raw hand position vector.

    The SDK message uses uint32 values, but accepting arbitrary JSON numbers at
    the MCP boundary would otherwise allow NaN, strings, or a wrong-length
    vector to reach the DDS writer.
    """
    if values is None:
        raise ValueError("hand position vector is required")
    try:
        raw_values = list(values)
    except TypeError as exc:
        raise ValueError("hand position vector must be an array") from exc
    if len(raw_values) != expected:
        raise ValueError(f"hand position vector must contain {expected} values")

    result = []
    for raw in raw_values:
        if isinstance(raw, bool):
            raise ValueError("hand positions must be numbers")
        try:
            value = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("hand positions must be numbers") from exc
        if not math.isfinite(value):
            raise ValueError("hand positions must be finite numbers")
        result.append(int(max(HAND_POSITION_MIN, min(limit, round(value)))))
    return result


def _normalize_hand_state_positions(position) -> list[int]:
    """Return HandState_ positions in Adam's effective 0..1000 range."""
    try:
        raw_positions = list(position)[:HAND_POSITION_COUNT]
    except (TypeError, ValueError):
        raw_positions = []

    try:
        positions = _coerce_hand_positions(
            raw_positions,
            limit=HAND_POSITION_MAX,
            expected=len(raw_positions),
        )
    except ValueError:
        positions = []
    if len(positions) < HAND_POSITION_COUNT:
        positions.extend([HAND_POSITION_MIN] * (HAND_POSITION_COUNT - len(positions)))
    return positions[:HAND_POSITION_COUNT]


def _hand_state_payload(position, received_at_ms: int, *, fresh: bool) -> dict:
    """Convert HandState_ into the payload returned by hand.get_state."""
    positions = _normalize_hand_state_positions(position)

    def side(values):
        return {
            "position": values,
            "channels": dict(zip(HAND_CHANNEL_NAMES, values)),
            "finger_count": 5,
            "motor_channel_count": 6,
        }

    now_ms = int(time.time() * 1000)
    age_ms = max(0, now_ms - int(received_at_ms))
    return {
        "timestamp_ms": now_ms,
        "received_at_ms": int(received_at_ms),
        "age_ms": age_ms,
        "fresh": bool(fresh),
        "position_max": HAND_POSITION_MAX,
        "position": positions,
        "left": side(positions[:6]),
        "right": side(positions[6:12]),
    }


def _destroy_ros_node(executor, node):
    if node is None:
        return
    if executor is not None:
        try:
            executor.remove_node(node)
        except Exception:
            pass
    try:
        node.destroy_node()
    except Exception:
        pass


class HandStateCache:
    """Own one DDS hand-state reader and fan its samples out to plugins."""

    def __init__(self, subscriber=None):
        self._subscriber = subscriber
        self._closed = False
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._thread = None
        self._stop_event = None
        self._latest_position = None
        self._received_at_ms = 0
        self._received_monotonic = None
        self._last_read_error = None

    def start(self) -> bool:
        with self._lifecycle_lock:
            with self._lock:
                if self._closed or self._subscriber is None:
                    return False
                thread = self._thread
                stop_event = self._stop_event
            if thread is not None and thread.is_alive():
                if stop_event is None or not stop_event.is_set():
                    return True
                thread.join(1.5)
                if thread.is_alive():
                    return False
                with self._lock:
                    if self._thread is thread:
                        self._thread = None
                        self._stop_event = None
            with self._lock:
                if self._closed or self._subscriber is None:
                    return False
                stop_event = threading.Event()
                self._stop_event = stop_event
                self._thread = threading.Thread(
                    target=self._poll_loop,
                    args=(stop_event,),
                    daemon=True,
                    name="adam_hand_state_poll",
                )
                self._thread.start()
                return True

    def stop(self, timeout: float = 1.5) -> bool:
        with self._lifecycle_lock:
            thread = self._thread
            stop_event = self._stop_event
            if stop_event is not None:
                stop_event.set()
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout)
            stopped = thread is None or not thread.is_alive()
            if stopped and self._thread is thread:
                self._thread = None
                self._stop_event = None
            return stopped

    def close(self):
        with self._lifecycle_lock:
            with self._lock:
                if self._closed:
                    return
                self._closed = True
                subscriber = self._subscriber
        self.stop()
        if subscriber is not None:
            try:
                subscriber.Close()
            except Exception as exc:
                print(f"[adam] WARNING: hand-state reader close failed: {exc}", flush=True)
            # A normal SDK Read is timeout-bounded, but closing the underlying
            # reader also gives an implementation that is blocked in Read a
            # chance to wake up before we return from close().
            with self._lock:
                thread = self._thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(0.5)
        with self._lock:
            self._subscriber = None

    def _poll_loop(self, stop_event: threading.Event):
        while not stop_event.is_set():
            with self._lock:
                subscriber = self._subscriber
            if subscriber is None:
                break
            try:
                msg = subscriber.Read(timeout=0.2)
                if msg:
                    position = list(getattr(msg, "position", []))
                    with self._lock:
                        self._latest_position = position
                        self._received_at_ms = int(time.time() * 1000)
                        self._received_monotonic = time.monotonic()
                        self._last_read_error = None
                elif stop_event.wait(0.01):
                    break
            except Exception as exc:
                with self._lock:
                    self._last_read_error = str(exc)
                # Do not spin if a broken DDS handle fails immediately.
                if stop_event.wait(0.1):
                    break
        with self._lock:
            if self._thread is threading.current_thread():
                self._thread = None
                self._stop_event = None

    def _fresh(self, received_monotonic, timeout_sec: float) -> bool:
        return (
            received_monotonic is not None
            and time.monotonic() - received_monotonic <= timeout_sec
        )

    def fresh_positions(self, timeout_sec: float) -> list[int] | None:
        with self._lock:
            position = list(self._latest_position) if self._latest_position is not None else None
            received_monotonic = self._received_monotonic
        if position is None or not self._fresh(received_monotonic, timeout_sec):
            return None
        return position

    def snapshot(self, timeout_sec: float) -> dict | None:
        with self._lock:
            position = list(self._latest_position) if self._latest_position is not None else None
            received_at_ms = self._received_at_ms
            received_monotonic = self._received_monotonic
        if position is None:
            return None
        return _hand_state_payload(
            position,
            received_at_ms,
            fresh=self._fresh(received_monotonic, timeout_sec),
        )

    def status(self, timeout_sec: float) -> dict:
        with self._lock:
            reader_available = self._subscriber is not None and not self._closed
            received_at_ms = self._received_at_ms
            received_monotonic = self._received_monotonic
            last_read_error = self._last_read_error
        fresh = self._fresh(received_monotonic, timeout_sec)
        result = {
            "reader_available": reader_available,
            "fresh": fresh,
            "last_sample_age_ms": (
                max(0, int(time.time() * 1000) - received_at_ms)
                if received_monotonic is not None else None
            ),
        }
        if last_read_error:
            result["last_read_error"] = last_read_error
        return result

# ROS2 JointState joint names for upper body control (used by ArmPlugin)
ROS2_UPPER_BODY_JOINTS = [
    "dof_pos/waistRoll", "dof_pos/waistPitch", "dof_pos/waistYaw",
    "dof_pos/shoulderPitch_Left", "dof_pos/shoulderRoll_Left",
    "dof_pos/shoulderYaw_Left", "dof_pos/elbow_Left",
    "dof_pos/wristYaw_Left", "dof_pos/wristPitch_Left", "dof_pos/wristRoll_Left",
    "dof_pos/shoulderPitch_Right", "dof_pos/shoulderRoll_Right",
    "dof_pos/shoulderYaw_Right", "dof_pos/elbow_Right",
    "dof_pos/wristYaw_Right", "dof_pos/wristPitch_Right", "dof_pos/wristRoll_Right",
    "root_pos/z",
    "dof_pos/hand_pinky_Left", "dof_pos/hand_ring_Left",
    "dof_pos/hand_middle_Left", "dof_pos/hand_index_Left",
    "dof_pos/hand_thumb_1_Left", "dof_pos/hand_thumb_2_Left",
    "dof_pos/hand_pinky_Right", "dof_pos/hand_ring_Right",
    "dof_pos/hand_middle_Right", "dof_pos/hand_index_Right",
    "dof_pos/hand_thumb_1_Right", "dof_pos/hand_thumb_2_Right",
]

# Human-facing Adam Pro upper-body controls. Limits come from the vendor's
# product overview, converted from radians to degrees. The card deliberately
# uses stable semantic ids rather than leaking ROS topic/joint names.
ARM_JOINT_CONTROLS = {
    "left_shoulder_pitch": ("左肩前后摆", "shoulderPitch_Left", -207.0, 117.0),
    "right_shoulder_pitch": ("右肩前后摆", "shoulderPitch_Right", -207.0, 117.0),
    "left_shoulder_roll": ("左肩向内/外摆", "shoulderRoll_Left", -36.0, 160.0),
    "right_shoulder_roll": ("右肩向内/外摆", "shoulderRoll_Right", -160.0, 36.0),
    "left_shoulder_yaw": ("左上臂旋转", "shoulderYaw_Left", -148.0, 148.0),
    "right_shoulder_yaw": ("右上臂旋转", "shoulderYaw_Right", -148.0, 148.0),
    "left_elbow": ("左肘弯曲", "elbow_Left", -143.0, 12.0),
    "right_elbow": ("右肘弯曲", "elbow_Right", -143.0, 12.0),
    "left_wrist_yaw": ("左手腕旋转", "wristYaw_Left", -153.0, 153.0),
    "right_wrist_yaw": ("右手腕旋转", "wristYaw_Right", -153.0, 153.0),
    "left_wrist_pitch": ("左手腕俯仰", "wristPitch_Left", -55.0, 55.0),
    "right_wrist_pitch": ("右手腕俯仰", "wristPitch_Right", -55.0, 55.0),
    "left_wrist_roll": ("左手腕侧摆", "wristRoll_Left", -55.0, 55.0),
    "right_wrist_roll": ("右手腕侧摆", "wristRoll_Right", -55.0, 55.0),
}

ARM_POSES = {
    "neutral": ("自然下垂", {}),
    "arms_forward": ("双臂向前", {
        "left_shoulder_pitch": -30.0, "right_shoulder_pitch": -30.0,
        "left_elbow": -45.0, "right_elbow": -45.0,
    }),
    "arms_open": ("双臂张开", {
        "left_shoulder_pitch": -15.0, "right_shoulder_pitch": -15.0,
        "left_shoulder_roll": 35.0, "right_shoulder_roll": -35.0,
        "left_elbow": -25.0, "right_elbow": -25.0,
    }),
    "hands_up": ("双手举起", {
        "left_shoulder_pitch": -95.0, "right_shoulder_pitch": -95.0,
        "left_elbow": -30.0, "right_elbow": -30.0,
    }),
    # One-armed poses are written on the right side only.  A caller that asks
    # for the other arm gets the mirrored target (see
    # ``ArmControlPlugin.mirror_targets``), so no pose is duplicated per side.
    # A salute brings the hand to the brow, not the crown.  Adam Pro carries a
    # neck (neckYaw/neckPitch) with the ZED on top, so driving shoulder_pitch
    # past horizontal together with a deep elbow fold sends the forearm into
    # the head.  Keep the upper arm at ~90° forward and fold the elbow to ~90°,
    # which leaves the hand beside the temple while the elbow stays clear.
    "salute": ("单手敬礼（抬臂至额角）", {
        "right_shoulder_pitch": -80.0, "right_shoulder_roll": -32.0,
        "right_shoulder_yaw": -28.0, "right_elbow": -88.0,
        "right_wrist_pitch": 10.0, "right_wrist_roll": -5.0,
    }),
    "arm_forward_high": ("单手肩高前伸（击掌预备）", {
        "right_shoulder_pitch": -75.0, "right_shoulder_roll": -8.0,
        "right_elbow": -20.0, "right_wrist_pitch": 15.0,
    }),
    "handshake_ready": ("单手屈肘前伸（握手预备）", {
        "right_shoulder_pitch": -30.0, "right_shoulder_roll": -12.0,
        "right_shoulder_yaw": -15.0, "right_elbow": -80.0,
    }),
    "wave_ready": ("屈肘上抬（挥手起势）", {
        "right_shoulder_pitch": -85.0, "right_shoulder_roll": -18.0,
        "right_elbow": -55.0, "right_wrist_pitch": 10.0,
    }),
    "wave_out": ("挥手外摆", {
        "right_shoulder_pitch": -85.0, "right_shoulder_roll": -48.0,
        "right_elbow": -45.0, "right_wrist_pitch": 10.0,
    }),
    "wave_in": ("挥手内摆", {
        "right_shoulder_pitch": -85.0, "right_shoulder_roll": 12.0,
        "right_elbow": -65.0, "right_wrist_pitch": 10.0,
    }),
}

ARM_ACTIONS = {f"set_{control}": control for control in ARM_JOINT_CONTROLS}

# The waist and neck are split out of ``ARM_JOINT_CONTROLS`` into their own
# cards so an agent can discover "turn the head" or "bow the waist" without
# having to know those joints live in the arm controller.  All three cards
# still share the single safe rt/lowcmd owner below.
WAIST_JOINT_CONTROLS = {
    "roll": ("腰部侧倾", "waistRoll", -16.0, 16.0),
    "pitch": ("腰部前后俯仰", "waistPitch", -48.0, 78.0),
    "yaw": ("腰部左右转动", "waistYaw", -47.0, 47.0),
}

# The ZED Mini is mounted on the head, so the neck card is effectively the
# "camera aiming" card.  Vendor limits are ±60° for both axes.
HEAD_JOINT_CONTROLS = {
    "yaw": ("头部左右转动", "neckYaw", -60.0, 60.0),
    "pitch": ("头部上下俯仰", "neckPitch", -60.0, 60.0),
}

WAIST_ACTIONS = {f"set_{control}": control for control in WAIST_JOINT_CONTROLS}
HEAD_ACTIONS = {f"set_{control}": control for control in HEAD_JOINT_CONTROLS}


def _arm_target_radians(control: str, angle_deg: object) -> tuple[str, float]:
    """Validate a human-facing upper-body request and return ROS target."""
    if control not in ARM_JOINT_CONTROLS:
        raise ValueError("joint must be one of the advertised Adam upper-body controls")
    if isinstance(angle_deg, bool):
        raise ValueError("angle_deg must be a finite number")
    try:
        value = float(angle_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("angle_deg must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError("angle_deg must be a finite number")
    _, ros_name, minimum, maximum = ARM_JOINT_CONTROLS[control]
    if value < minimum or value > maximum:
        raise ValueError(
            f"{control} angle must be within [{minimum:g}, {maximum:g}] degrees")
    return ros_name, math.radians(value)


def _waist_target_radians(control: str, angle_deg: object) -> tuple[str, float]:
    """Validate a waist request against WAIST_JOINT_CONTROLS and return ROS target."""
    if control not in WAIST_JOINT_CONTROLS:
        raise ValueError("joint must be one of the advertised Adam waist controls")
    if isinstance(angle_deg, bool):
        raise ValueError("angle_deg must be a finite number")
    try:
        value = float(angle_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("angle_deg must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError("angle_deg must be a finite number")
    _, joint_name, minimum, maximum = WAIST_JOINT_CONTROLS[control]
    if value < minimum or value > maximum:
        raise ValueError(
            f"{control} angle must be within [{minimum:g}, {maximum:g}] degrees")
    return joint_name, math.radians(value)


def _head_target_radians(control: str, angle_deg: object) -> tuple[str, float]:
    """Validate a neck request against HEAD_JOINT_CONTROLS and return ROS target."""
    if control not in HEAD_JOINT_CONTROLS:
        raise ValueError("joint must be one of the advertised Adam head controls")
    if isinstance(angle_deg, bool):
        raise ValueError("angle_deg must be a finite number")
    try:
        value = float(angle_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("angle_deg must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError("angle_deg must be a finite number")
    _, joint_name, minimum, maximum = HEAD_JOINT_CONTROLS[control]
    if value < minimum or value > maximum:
        raise ValueError(
            f"{control} angle must be within [{minimum:g}, {maximum:g}] degrees")
    return joint_name, math.radians(value)


def _best_effort_qos():
    """Shallow best-effort queue for high-rate optional telemetry."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _battery_payload(battery, timestamp_ms: int | None = None) -> dict:
    """Normalize Adam's embedded DDS BMS sample for the battery card.

    The BMS is carried inside ``rt/lowstate``.  Keep this conversion separate
    from joint/IMU serialization so an incomplete sample from either of those
    sensors cannot stop the battery card's data flow.
    """
    def number(name: str) -> float | None:
        try:
            value = float(getattr(battery, name))
            return value if math.isfinite(value) else None
        except (AttributeError, TypeError, ValueError):
            return None

    status = getattr(battery, "status", None)
    data = {
        "timestamp_ms": int(timestamp_ms or time.time() * 1000),
        "voltage": number("voltage"),
        "current": number("current"),
        "power": number("power"),
        "wh_accumulated": number("wh_accumulated"),
        # The vendor's BatteryData_ DDS definition has no SOC/capacity field.
        # Do not invent a percentage from voltage: its discharge curve changes
        # under load and would make this safety-relevant card misleading.
        "percentage": None,
        "percentage_available": False,
        "percentage_message": "The Adam DDS BMS message does not provide state of charge",
        "status": str(status) if status not in (None, "") else "unknown",
        "source_topic": "rt/lowstate",
    }
    return data


# ===========================================================================
# StatePlugin — subscribes DDS rt/lowstate, publishes to ROS2
# ===========================================================================

def _reliable_qos():
    """QoS for Dashboard-facing state and camera streams."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class _StatePublisherNode(Node):
    """ROS2 node that publishes skeleton, motor, robot, IMU, and battery data."""

    _BATTERY_INTERVAL_S = 1.0

    def __init__(self, namespace: str, variant: str, publish_rate_hz: float):
        super().__init__("adam_state_publisher")
        self._namespace = namespace
        self._variant = variant
        self._joints = VARIANT_JOINTS[variant]

        qos = _reliable_qos()

        self._topic_skeleton = f"/{namespace}/state/joints"
        self._topic_imu = f"/{namespace}/state/imu"
        self._topic_battery = f"/{namespace}/state/battery"
        self._topic_robot_state = f"/{namespace}/state/robot"
        self._topic_motor_state = f"/{namespace}/state/motors"

        self._pub_skeleton = self.create_publisher(String, self._topic_skeleton, qos)
        self._pub_imu = self.create_publisher(String, self._topic_imu, qos)
        self._pub_battery = self.create_publisher(String, self._topic_battery, qos)
        self._pub_robot_state = self.create_publisher(String, self._topic_robot_state, qos)
        self._pub_motor_state = self.create_publisher(String, self._topic_motor_state, qos)

        self._latest_state = None
        self._latest_state_at_ms = None
        self._active = False
        self._lock = threading.Lock()

        interval = 1.0 / publish_rate_hz
        self._timer = self.create_timer(interval, self._publish)
        self._battery_timer = self.create_timer(
            self._BATTERY_INTERVAL_S, self._publish_battery)

    def update_state(self, state):
        with self._lock:
            self._latest_state = state
            self._latest_state_at_ms = int(time.time() * 1000)

    def set_active(self, active: bool):
        with self._lock:
            self._active = bool(active)

    def _publish(self):
        with self._lock:
            state = self._latest_state
            active = self._active

        if not active or state is None:
            return

        robot_data = {"mode_pr": int(state.mode_pr), "tick": int(state.tick)}
        for index, value in enumerate(state.wireless_remote):
            if float(value) != 0.0:
                robot_data[f"wireless_remote_{index:02d}"] = float(value)
        msg_robot = String()
        msg_robot.data = json.dumps(robot_data)
        self._pub_robot_state.publish(msg_robot)

        # Skeleton (joints)
        joints = []
        for idx, name in enumerate(self._joints):
            if idx < len(state.motor_state):
                joints.append({
                    "idx": idx,
                    "name": name,
                    "q": float(state.motor_state[idx].q),
                })
        msg = String()
        msg.data = json.dumps({"joints": joints})
        self._pub_skeleton.publish(msg)

        motor_data = {}
        for idx, motor in enumerate(state.motor_state):
            if idx >= len(self._joints):
                break
            # Flat scalar fields let the dashboard render one signal per cell
            # instead of requiring it to understand a nested motor array.
            prefix = f"motor_{idx:02d}_{self._joints[idx]}"
            motor_data[f"{prefix}_position_rad"] = float(motor.q)
            motor_data[f"{prefix}_velocity_rad_s"] = float(motor.dq)
            motor_data[f"{prefix}_torque_nm"] = float(motor.tau_est)
            # ``ddq`` is commonly an all-zero firmware placeholder. Omit it
            # until a meaningful acceleration estimate is available.
            if float(motor.ddq) != 0.0:
                motor_data[f"{prefix}_acceleration_rad_s2"] = float(motor.ddq)
            if int(motor.mode) != 0:
                motor_data[f"{prefix}_mode"] = int(motor.mode)
            if int(motor.state) != 0:
                motor_data[f"{prefix}_state"] = int(motor.state)
        msg_motor = String()
        msg_motor.data = json.dumps(motor_data)
        self._pub_motor_state.publish(msg_motor)

        # IMU
        imu = state.imu_state
        imu_data = {"temperature": int(imu.temperature)}
        for prefix, values, names in (
                ("quaternion", list(imu.quaternion), ("w", "x", "y", "z")),
                ("gyroscope_rad_s", list(imu.gyroscope), ("x", "y", "z")),
                ("accelerometer_m_s2", list(imu.accelerometer), ("x", "y", "z")),
                ("ypr_rad", list(imu.ypr), ("yaw", "pitch", "roll"))):
            for index, value in enumerate(values):
                label = names[index] if index < len(names) else str(index)
                imu_data[f"{prefix}_{label}"] = float(value)
        msg_imu = String()
        msg_imu.data = json.dumps(imu_data)
        self._pub_imu.publish(msg_imu)

    def _publish_battery(self):
        """Publish BMS independently at 1Hz.

        Adam's low-state message contains all three state classes.  A malformed
        joint or IMU reading must not prevent the dashboard from receiving the
        BMS stream, and a 50Hz battery stream is unnecessary for this card.
        """
        with self._lock:
            state = self._latest_state
            received_at_ms = self._latest_state_at_ms
            active = self._active

        if not active or state is None:
            return

        bat_data = _battery_payload(
            getattr(state, "battery_data", None), received_at_ms)
        msg_bat = String()
        msg_bat.data = json.dumps(bat_data)
        self._pub_battery.publish(msg_bat)


class StatePlugin:
    """Subscribes DDS rt/lowstate and publishes body state to ROS2."""

    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 variant: str, dds_lowstate_sub=None, **kwargs):
        self._namespace = namespace
        self._variant = variant
        self._running = False
        self._executor = executor
        self._poll_lifecycle_lock = threading.Lock()
        self._poll_thread = None
        self._poll_stop_event = None

        rate = plugin_config.get("publish_rate_hz", 50)
        self._node = _StatePublisherNode(namespace, variant, rate)
        executor.add_node(self._node)

        # DDS subscribers (pre-created in main.py before rclpy.init to avoid conflict)
        self._lowstate_sub = dds_lowstate_sub

    def _poll_dds(self, stop_event: threading.Event):
        """Poll DDS subscribers in a background thread."""
        while not stop_event.is_set():
            try:
                msg = self._lowstate_sub.Read(timeout=0.2)
                if msg:
                    self._node.update_state(msg)
                elif stop_event.wait(0.01):
                    break
            except Exception:
                if stop_event.wait(0.1):
                    break
    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "Adam joint state — real-time skeleton visualization",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_skeleton, "format": "sensor/skeleton"}
                ],
            },
            {
                "name": "motor_state",
                "type": "sensor",
                "description": "Adam motor feedback — position, velocity, acceleration, torque estimate and state",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_motor_state, "format": "data/json"}
                ],
            },
            {
                "name": "robot_state",
                "type": "sensor",
                "description": "Adam low-level state — mode, tick and wireless remote channels",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_robot_state, "format": "data/json"}
                ],
            },
            {
                "name": "imu",
                "type": "sensor",
                "description": "Adam IMU — quaternion, gyroscope, accelerometer",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_imu, "format": "data/json"}
                ],
            },
            {
                "name": "battery",
                "type": "sensor",
                "description": f"Adam BMS battery — voltage, current, power, accumulated energy and status. The vendor DDS message has no SOC percentage. Publishes at 1Hz to {self._node._topic_battery}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_battery, "format": "data/json"}
                ],
            },
        ]

    def start(self):
        self._running = True
        self._node.set_active(True)
        if self._lowstate_sub:
            with self._poll_lifecycle_lock:
                if self._poll_thread is None or not self._poll_thread.is_alive():
                    stop_event = threading.Event()
                    self._poll_stop_event = stop_event
                    self._poll_thread = threading.Thread(
                        target=self._poll_dds,
                        args=(stop_event,),
                        daemon=True,
                        name="adam_lowstate_poll",
                    )
                    self._poll_thread.start()

    def stop(self):
        self._running = False
        self._node.set_active(False)
        with self._poll_lifecycle_lock:
            thread = self._poll_thread
            stop_event = self._poll_stop_event
            if stop_event is not None:
                stop_event.set()
            if thread is not None and thread is not threading.current_thread():
                thread.join(1.5)
            if thread is None or not thread.is_alive():
                self._poll_thread = None
                self._poll_stop_event = None

    def close(self):
        self.stop()
        _destroy_ros_node(self._executor, self._node)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get("_tool_name", "joints")
            if tool_name == "imu":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_imu, "format": "data/json"}]}
            if tool_name == "motor_state":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_motor_state, "format": "data/json"}]}
            if tool_name == "robot_state":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_robot_state, "format": "data/json"}]}
            if tool_name == "battery":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_battery, "format": "data/json"}]}
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._node._topic_skeleton, "format": "sensor/skeleton"}]}
        return None


# ===========================================================================
# LocoPlugin — gRPC locomotion control
# ===========================================================================

class LocoPlugin:
    """High-level locomotion via the Adam RL gRPC service."""

    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 grpc_client, **kwargs):
        self._grpc = grpc_client
        self._namespace = namespace

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "description": "Adam locomotion — walk, turn, stop, gestures, mode switching",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "set_mode", "move", "stop", "stand_motion",
                            "stand_action", "stand_dynamic", "get_state",
                            "list_actions", "clear_error", "carry_box",
                        ],
                    },
                    "mode": {
                        "type": "string",
                        "minLength": 1,
                        "description": "Target state name returned by get_state.switchable_states",
                    },
                    "vx": {"type": "number", "description": "Forward velocity (m/s)"},
                    "vy": {"type": "number", "description": "Lateral velocity (m/s)"},
                    "vyaw": {"type": "number", "description": "Yaw angular velocity (rad/s)"},
                    "motion_id": {"type": "integer", "description": "Predefined motion ID"},
                    "action_id": {"type": "integer", "description": "Predefined action/gesture ID"},
                    "pitch": {"type": "number", "description": "Body pitch (rad)"},
                    "roll": {"type": "number", "description": "Body roll (rad)"},
                    "yaw": {"type": "number", "description": "Body yaw (rad)"},
                    "height": {"type": "number", "description": "Body height (m)"},
                    "enable": {"type": "boolean", "description": "Enable/disable flag"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_mode": {
                        "params": ["mode"],
                        "description": "Switch robot mode (e.g., stand, walk)",
                    },
                    "move": {
                        "params": ["vx", "vy", "vyaw"],
                        "description": "Walk with specified velocities",
                    },
                    "stop": {
                        "params": [],
                        "description": "Stop all movement",
                    },
                    "stand_motion": {
                        "params": ["motion_id"],
                        "description": "Execute predefined standing pose",
                    },
                    "stand_action": {
                        "params": ["action_id"],
                        "description": "Execute predefined gesture/action",
                    },
                    "stand_dynamic": {
                        "params": ["pitch", "roll", "yaw", "height"],
                        "description": "Adjust body orientation and height while standing",
                    },
                    "get_state": {
                        "params": [],
                        "description": "Query current robot state (mode, gait, battery)",
                    },
                    "list_actions": {
                        "params": [],
                        "description": "List available motions and actions",
                    },
                    "clear_error": {
                        "params": [],
                        "description": "Clear error state",
                    },
                    "carry_box": {
                        "params": ["enable"],
                        "description": "Enable/disable carry box mode",
                    },
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "set_mode":
            return self._grpc.set_mode(args.get("mode", ""))
        if action == "move":
            return self._grpc.set_speed(
                args.get("vx", 0.0), args.get("vy", 0.0), args.get("vyaw", 0.0)
            )
        if action == "stop_move" or action == "stop":
            return self._grpc.set_speed(0.0, 0.0, 0.0)
        if action == "stand_motion":
            return self._grpc.set_stand_motion(args.get("motion_id", 0))
        if action == "stand_action":
            return self._grpc.set_stand_action(args.get("action_id", 0))
        if action == "stand_dynamic":
            return self._grpc.set_stand_dynamic(
                pitch=args.get("pitch", 0.0),
                roll=args.get("roll", 0.0),
                yaw=args.get("yaw", 0.0),
                height=args.get("height", 0.0),
            )
        if action == "get_state":
            return self._grpc.get_robot_state()
        if action == "list_actions":
            return self._grpc.get_stand_list()
        if action == "clear_error":
            return self._grpc.set_error_clear()
        if action == "carry_box":
            return self._grpc.set_carry_box(args.get("enable", False))
        if action == "info":
            return {"state": "ready"}
        return None


# ===========================================================================
# RlLocoPlugin — extended reinforcement-learning gRPC locomotion control
# ===========================================================================

class RlLocoPlugin:
    """Adam RL movement card; controller state is an internal detail."""

    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 grpc_client, **kwargs):
        self._grpc = grpc_client
        self._namespace = namespace
        self._move_lock = threading.Lock()
        self._move_generation = 0
        # action_id of the timed stop currently armed, if any.  Agent Core
        # holds a pending action until this card reports it finished.
        self._pending_action_id = None

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "description": "Adam RL locomotion — limited-duration walking, turning and body-height adjustment",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move", "set_height", "stop"],
                               "oneOf": [
                                   {"const": "move", "title": "定时移动"},
                                   {"const": "set_height", "title": "设置机身高度"},
                                   {"const": "stop", "title": "立即停止移动"},
                               ]},
                    "vx": {"type": "number", "title": "前进速度（m/s）", "minimum": -1.0,
                           "maximum": 1.0, "multipleOf": 0.01,
                           "description": "正值前进、负值后退；范围 -1.00 至 1.00 m/s。"},
                    "vy": {"type": "number", "title": "横移速度（m/s）", "minimum": -1.0,
                           "maximum": 1.0, "multipleOf": 0.01,
                           "description": "正负方向由机器人坐标系定义；范围 -1.00 至 1.00 m/s。"},
                    "vyaw": {"type": "number", "title": "转向速度（rad/s）", "minimum": -1.0,
                             "maximum": 1.0, "multipleOf": 0.01,
                             "description": "正负方向由机器人坐标系定义；范围 -1.00 至 1.00 rad/s。"},
                    "duration_s": {"type": "number", "title": "移动时长（秒）", "minimum": 0.1,
                                   "maximum": 30.0, "multipleOf": 0.1,
                                   "description": "范围 0.1-30.0 秒；到时自动发送零速度。新的移动或停止会取消此前计时。"},
                    "height": {"type": "number", "title": "机身高度目标（m）", "minimum": -1.0,
                               "maximum": 1.0, "multipleOf": 0.01,
                               "description": "RL SetHeight 的高度目标，范围 -1.00 至 1.00 m。"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration_s"],
                             "description": "按设定速度移动指定时长，到时自动停止。"},
                    "set_height": {"params": ["height"],
                                   "description": "设置 RL 控制下的机身高度目标。"},
                    "stop": {"params": []},
                },
                # A timed move outlives the call that starts it: the robot keeps
                # walking after dispatch() returns.  Declaring the completion
                # keeps Agent Core from treating the accepted velocity as the
                # finished movement and scheduling dependent work mid-stride.
                # The timeout covers the 30s maximum plus the reporting round
                # trip.
                "x-completion": {"actions": ["move"], "timeout": 45},
            },
        }

    def start(self):
        return None


    def stop(self):
        # A plugin stop is a local lifecycle event; explicit shutdown is
        # required before asking the robot controller to exit.
        self._cancel_timed_move()
        return None

    def _cancel_timed_move(self, reason: str | None = None):
        """Invalidate the armed timed stop and return the new generation.

        When the discarded timer belonged to an action Agent Core is still
        waiting on, the reason is reported as its completion: a superseded move
        ends early, and leaving the pending action open would hold the actuator
        barrier until the declared timeout expired.
        """
        with self._move_lock:
            superseded, self._pending_action_id = self._pending_action_id, None
            self._move_generation += 1
            generation = self._move_generation
        if superseded is not None and reason is not None:
            _notify_action_completion(superseded, "cancelled",
                                      {"reason": reason}, self.PREFIX)
        return generation

    def _schedule_stop(self, generation: int, duration_s: float, action_id: str):
        def stop_when_due():
            time.sleep(duration_s)
            with self._move_lock:
                if generation != self._move_generation:
                    return
                self._pending_action_id = None
            self._grpc.set_velocity(0.0, 0.0, 0.0)
            _notify_action_completion(
                action_id, "completed",
                {"duration_s": duration_s, "auto_stop": True}, self.PREFIX)

        threading.Thread(target=stop_when_due, daemon=True,
                         name="adam_loco_timed_stop").start()

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"state": "ready"}
        # Never transition an FSM merely to issue a zero-velocity request.
        # This keeps stop usable as the least surprising command when another
        # controller or motion card currently owns the robot.
        if action == "stop":
            self._cancel_timed_move("stopped by request")
            return self._grpc.set_velocity(0.0, 0.0, 0.0)
        state = _ensure_rl_locomotion(self._grpc)
        if not state.get("success", False):
            return state
        if action == "move":
            try:
                duration_s = float(args.get("duration_s"))
                if not math.isfinite(duration_s) or not 0.1 <= duration_s <= 30.0:
                    raise ValueError
            except (TypeError, ValueError):
                return {"success": False, "code": "INVALID_ARGUMENT",
                        "message": "duration_s must be a number in [0.1, 30.0]"}
            action_id = f"adam_loco_move_{uuid.uuid4().hex[:8]}"
            generation = self._cancel_timed_move("superseded by a newer move")
            result = self._grpc.set_velocity(
                args.get("vx", 0.0), args.get("vy", 0.0), args.get("vyaw", 0.0))
            if result.get("success", False):
                with self._move_lock:
                    self._pending_action_id = action_id
                self._schedule_stop(generation, duration_s, action_id)
                result = dict(result, duration_s=duration_s,
                              auto_stop=True, action_id=action_id)
            return result
        if action == "set_height":
            return self._grpc.set_height(args.get("height", 0.0))
        return None


def _ensure_rl_locomotion(grpc_client):
    """Select RL and enter its walking state without exposing FSM controls."""
    control = grpc_client.set_control_mode(1)
    if not control.get("success", False):
        return {"success": False, "code": "RL_CONTROL_UNAVAILABLE",
                "message": "unable to select RL control domain", "details": control}
    state = grpc_client.get_robot_state()
    if not state.get("success", False):
        return state
    if state.get("fsm_state") == "STAND_WALK":
        return state
    states = state.get("switchable_states")
    if not isinstance(states, list):
        return {"success": False, "code": "STATE_UNAVAILABLE",
                "message": "GetRobotState did not return switchable_states", "state": state}
    if "STAND_WALK" not in states:
        return {"success": False, "code": "NOT_ALLOWED",
                "message": "STAND_WALK is not available on the robot", "switchable_states": states}
    result = grpc_client.set_mode("STAND_WALK")
    if not result.get("success", False):
        return result
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        state = grpc_client.get_robot_state()
        if state.get("success", False) and state.get("fsm_state") == "STAND_WALK":
            return state
        time.sleep(0.2)
    return {"success": False, "code": "FSM_TIMEOUT",
            "message": "timed out entering STAND_WALK", "state": state}


class _RlActionPlugin:
    """Small, responsibility-focused cards over the RL gRPC contract."""
    PREFIX = ""

    def __init__(self, plugin_config: dict, namespace: str, executor, grpc_client, **kwargs):
        self._grpc = grpc_client

    def start(self):
        return None

    def stop(self):
        return None

    def _state(self):
        return self._grpc.get_robot_state()

    def _ensure_rl_state(self, target_state=None):
        """Acquire RL control and enter the state required by an action card."""
        control = self._grpc.set_control_mode(1)
        if not control.get("success", False):
            return {"success": False, "code": "RL_CONTROL_UNAVAILABLE",
                    "message": "unable to select RL control domain", "details": control}
        state = self._state()
        if not state.get("success", False):
            return state
        if not target_state or state.get("fsm_state") == target_state:
            return state
        denied = self._allowed(state, "switchable_states", target_state)
        if denied:
            return denied
        result = self._grpc.set_mode(target_state)
        if not result.get("success", False):
            return result
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            state = self._state()
            if state.get("fsm_state") == target_state:
                return state
            time.sleep(0.2)
        return {"success": False, "code": "FSM_TIMEOUT",
                "message": f"timed out entering {target_state}", "state": state}

    @staticmethod
    def _allowed(state, key, value):
        if not isinstance(state, dict) or not state.get("success", False):
            return {"success": False, "code": "STATE_UNAVAILABLE",
                    "message": "GetRobotState did not return a usable state", "state": state}
        if key not in state:
            return {"success": False, "code": "STATE_UNAVAILABLE",
                    "message": f"GetRobotState did not return {key}", "state": state}
        values = state.get(key) or []
        # Firmware revisions have reported either RPC names (SetMotion) or
        # short action names (motion); accept both spellings while still
        # refusing commands absent from the robot's advertised capability.
        aliases = {value, value.removeprefix("Set").lower()}
        if not any(item in values for item in aliases):
            return {"success": False, "code": "NOT_ALLOWED", "message":
                    f"{value!r} is not present in robot {key}", key: values}
        return None


class PosturePlugin(_RlActionPlugin):
    PREFIX = "posture"

    def get_tool(self):
        return {"name": "posture", "type": "actuator",
                "description": "Adam FSM posture and state transitions",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["get_state", "set_mode", "wait_mode", "info"]},
                    "target_state": {"type": "string", "minLength": 1},
                    "timeout_s": {"type": "number", "minimum": 0, "maximum": 60},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {"get_state": {"params": []},
                    "set_mode": {"params": ["target_state"]},
                    "wait_mode": {"params": ["target_state", "timeout_s"]},
                    "info": {"params": []}}}}

    def dispatch(self, action, args):
        if action in ("get_state", "info"):
            return self._state()
        if action == "set_mode":
            target = args.get("target_state", "")
            state = self._state()
            denied = self._allowed(state, "switchable_states", target)
            return denied or self._grpc.set_mode(target)
        if action == "wait_mode":
            target = args.get("target_state", "")
            try:
                timeout = max(0.0, min(60.0, float(args.get("timeout_s", 10.0))))
            except (TypeError, ValueError):
                return {"success": False, "code": "INVALID_ARGUMENT",
                        "message": "timeout_s must be a number"}
            started = time.monotonic()
            state = self._state()
            denied = self._allowed(state, "switchable_states", target)
            if denied:
                return denied
            result = self._grpc.set_mode(target)
            if not result.get("success", False):
                return result
            while time.monotonic() - started < timeout:
                state = self._state()
                if state.get("fsm_state") == target:
                    return {"success": True, "state": state, "completed": True}
                time.sleep(0.2)
            return {"success": False, "code": "TIMEOUT", "state": state, "completed": False}
        return None


class MotionPlugin(_RlActionPlugin):
    def get_tool(self):
        return {"name": "motion", "type": "actuator",
                "description": "播放机器人端上半身动作文件；适用于挥手、招手等动作，不控制行走轨迹。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["play", "stop", "get_state", "info"],
                               "oneOf": [{"const": "play", "title": "播放上半身动作"},
                                         {"const": "stop", "title": "停止上半身动作"},
                                         {"const": "get_state", "title": "读取机器人状态"},
                                         {"const": "info", "title": "读取机器人状态"}]},
                    "motion_file": {"type": "string", "title": "机器人端动作文件", "pattern": r".+\.txt$",
                                    "description": "机器人控制器上的 .txt 文件路径，例如 Sources/motion/Wave.txt；文件必须已在机器人端存在。"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {"play": {"params": ["motion_file"], "description": "播放机器人端已有的上半身 .txt 动作文件。"}, "stop": {"params": [], "description": "停止当前上半身动作。"},
                    "get_state": {"params": []}, "info": {"params": []}}}}

    def dispatch(self, action, args):
        # The framework sends start/info to every card on the canvas whatever
        # the tool does.  Answering start here instead of falling off the end
        # of dispatch matters: a bare None is reported as an unknown action,
        # and a strict project start then rolls the whole project back.
        if action == "start":
            return {"state": "ready"}
        if action in ("get_state", "info"):
            return self._state()
        if action == "play":
            state = self._ensure_rl_state("MULTI_AGENT")
            if not state.get("success", False):
                return state
            denied = self._allowed(state, "available_actions", "SetMotion")
            if denied:
                return denied
            return self._grpc.set_motion("PLAY", args.get("motion_file", ""))
        if action == "stop":
            return self._grpc.set_motion("STOP", "")
        return None


class TrackingMotionPlugin(_RlActionPlugin):
    def get_tool(self):
        return {"name": "tracking_motion", "type": "actuator",
                "description": "执行机器人端全身轨迹文件；可同时驱动躯干和腿部，执行前须确保周围空间安全。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["play", "get_state", "info"],
                               "oneOf": [{"const": "play", "title": "执行全身轨迹"},
                                         {"const": "get_state", "title": "读取机器人状态"},
                                         {"const": "info", "title": "读取机器人状态"}]},
                    "motion_file": {"type": "string", "title": "机器人端全身轨迹文件", "pattern": r".+\.txt$",
                                    "description": "机器人控制器上的 .txt 轨迹文件，例如 Sources/tracking/Walk.txt；文件必须已在机器人端存在。"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {"play": {"params": ["motion_file"], "description": "执行机器人端已有的全身 .txt 轨迹。"},
                    "get_state": {"params": []}, "info": {"params": []}}}}

    def dispatch(self, action, args):
        if action in ("get_state", "info"):
            return self._state()
        if action == "play":
            state = self._ensure_rl_state("MOTION_TRACK")
            if not state.get("success", False):
                return state
            denied = self._allowed(state, "available_actions", "SetTrackingMotion")
            return denied or self._grpc.set_tracking_motion(args.get("motion_file", ""))
        return None


class ControlModePlugin(_RlActionPlugin):
    def get_tool(self):
        return {"name": "control_mode", "type": "actuator",
                "description": "Switch Adam between Traditional and RL control domains",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["set_rl", "set_traditional", "get_state", "info"]},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {"set_rl": {"params": []}, "set_traditional": {"params": []},
                    "get_state": {"params": []}, "info": {"params": []}}}}

    def dispatch(self, action, args):
        if action in ("get_state", "info"):
            return self._grpc.get_control_state()
        if action == "set_rl":
            return self._grpc.set_control_mode(1)
        if action == "set_traditional":
            return self._grpc.set_control_mode(0)
        return None


# ===========================================================================
# ArmPlugin — DDS rt/lowcmd upper body control
# ===========================================================================

class ArmControlPlugin:
    """Human-facing Adam Pro arm control over the vendor's DDS lowcmd API.

    The official arm-control example continuously publishes a complete LowCmd:
    all non-arm joints hold their measured startup positions, while only the
    fourteen arm joints receive the requested targets.  Sending a sparse
    command is unsafe because ``rt/lowcmd`` owns the full body.
    """

    PREFIX = "arm"
    _DOF = 31
    _RATE_HZ = 50.0
    _MAX_VELOCITY_RAD_S = 0.5
    _DEFAULT_TRANSITION_SECONDS = 2.0
    _RELEASE_SECONDS = 1.5
    # Peak derivative of the quintic ease in `_ease`: 30u^2(1-u)^2 reaches
    # 1.875 at u=0.5.  A segment whose duration came straight from
    # distance / _MAX_VELOCITY_RAD_S therefore passes 1.875x the configured
    # limit at its midpoint, so the duration must budget for the peak.
    _EASE_PEAK_RATE = 1.875
    # Official arm_control_config.json gains. LowCmd owns all 31 motors, so
    # non-arm joints must also be held with their vendor gains while an arm
    # command is active; zero gains there causes intermittent posture loss.
    _JOINT_PD = {
        "hipPitch": (400.0, 6.1), "hipRoll": (700.0, 30.0),
        "hipYaw": (405.0, 6.1), "kneePitch": (400.0, 8.0),
        "anklePitch": (40.0, 2.5), "ankleRoll": (0.0, 0.35),
        "waistRoll": (405.0, 6.1), "waistPitch": (405.0, 6.1),
        "waistYaw": (205.0, 4.1), "neckYaw": (40.0, 1.0),
        "neckPitch": (40.0, 1.0),
        "shoulderPitch": (150.0, 4.0), "shoulderRoll": (150.0, 4.0),
        "shoulderYaw": (40.0, 1.0), "elbow": (100.0, 2.0),
        "wristYaw": (15.0, 0.9), "wristPitch": (15.0, 0.9),
        "wristRoll": (15.0, 0.9),
    }

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 grpc_client=None, dds_lowcmd_pub=None,
                 dds_arm_lowstate_sub=None, variant="pro", **kwargs):
        if str(variant).lower() != "pro":
            raise ValueError(
                "arm_control currently supports only Adam Pro's 31-DOF lowcmd layout")
        self._namespace = namespace
        self._publisher = dds_lowcmd_pub
        self._lowstate_sub = dds_arm_lowstate_sub
        self._rate_hz = float(plugin_config.get("control_rate_hz", self._RATE_HZ))
        self._rate_hz = max(10.0, min(100.0, self._rate_hz))
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_ready = threading.Event()
        self._thread = None
        self._hold_q = None
        self._current_q = None
        self._target_q = {}
        self._active = False
        self._streaming = False
        self._soft_arms = False
        self._release_started_at = None
        self._writes = 0
        self._last_error = None
        # Smooth-segment state. Each _set_targets call opens a segment that
        # minimum-jerk blends from the pose being written when the call lands
        # (_seg_start, indexed by joint like _target_q) to the newest targets
        # over _seg_span seconds, so retargets mid-motion never snap.
        self._seg_current = [0.0] * self._DOF
        self._seg_start = {}
        self._seg_span = self._DEFAULT_TRANSITION_SECONDS
        self._seg_started_at = time.monotonic()

    @staticmethod
    def _joint_index(joint_name: str) -> int:
        return ADAM_PRO_JOINTS.index(joint_name)

    @classmethod
    def _pd_for_joint(cls, joint_name: str) -> tuple[float, float]:
        for prefix, gains in cls._JOINT_PD.items():
            if joint_name.startswith(prefix):
                return gains
        return (0.0, 0.0)

    # Axes that invert between the left and the right limb.  The vendor limit
    # table states the same mirroring rule: left shoulder roll is
    # [-36, 160] against right [-160, 36], while pitch and elbow keep their
    # range and sign on both sides.
    _MIRROR_NEGATED_AXES = ("roll", "yaw")

    @classmethod
    def mirror_targets(cls, targets: dict[str, float]) -> dict[str, float]:
        """Mirror a one-sided upper-body target onto the opposite arm.

        Lets a pose be written once for the right arm and reused on the left,
        which is why ``ARM_POSES`` keeps a single definition per one-armed
        pose.  A control without a ``left_``/``right_`` prefix passes through
        unchanged (defensive; the arm card currently has none).
        """
        mirrored = {}
        for control, value in targets.items():
            if control.startswith("left_"):
                opposite = "right_" + control[len("left_"):]
            elif control.startswith("right_"):
                opposite = "left_" + control[len("right_"):]
            else:
                mirrored[control] = value
                continue
            axis = control.rsplit("_", 1)[-1]
            mirrored[opposite] = (
                -value if axis in cls._MIRROR_NEGATED_AXES else value)
        return mirrored

    def _read_initial_state(self):
        if self._lowstate_sub is None:
            return
        while not self._stop_event.is_set() and not self._state_ready.is_set():
            try:
                state = self._lowstate_sub.Read(timeout=0.2)
                motors = getattr(state, "motor_state", None) if state else None
                if motors is not None and len(motors) >= self._DOF:
                    hold_q = [float(motors[index].q) for index in range(self._DOF)]
                    if all(math.isfinite(value) for value in hold_q):
                        with self._lock:
                            self._hold_q = hold_q
                            self._current_q = hold_q.copy()
                            self._seg_current = hold_q.copy()
                            self._seg_start = {}
                            self._seg_started_at = time.monotonic()
                        self._state_ready.set()
                        return
            except Exception as exc:
                self._last_error = f"rt/lowstate read failed: {exc}"
                self._stop_event.wait(0.1)

    @classmethod
    def _ease(cls, progress: float) -> float:
        """Quintic (minimum-jerk) easing: zero velocity/acceleration at both ends.

        Unlike a fixed-step rate limiter, this never introduces a velocity
        corner, so arm motion starts and ends smoothly instead of creeping at
        max speed and then stopping dead.
        """
        u = max(0.0, min(1.0, progress))
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

    def _sample_segment(self, now: float) -> list[float]:
        current_q = list(self._seg_current)
        eased = self._ease((now - self._seg_started_at) / max(1e-6, self._seg_span))
        for index, start in self._seg_start.items():
            current_q[index] = start + (self._target_q[index] - start) * eased
        return current_q

    def _write_command(self, dt: float):
        with self._lock:
            if not self._state_ready.is_set() or self._hold_q is None:
                return
            now = time.monotonic()
            active = self._active
            streaming = self._streaming
            soft_arms = self._soft_arms
            release_started_at = self._release_started_at
            targets = self._target_q.copy()
            hold_q = self._hold_q.copy()

            release_ratio = 0.0
            finish_release = False
            if release_started_at is not None:
                release_ratio = min(1.0, (now - release_started_at) / self._RELEASE_SECONDS)
                if release_ratio >= 1.0:
                    # Publish one final complete command with zero arm gains;
                    # stopping before this write would leave the prior PD
                    # gains latched in the robot controller.
                    finish_release = True
            elif not active and not streaming:
                return

            # Sample a minimum-jerk blend between the pose commanded at the
            # start of the current segment and the newest target.  The span is
            # the longer of the configured transition and the shortest duration
            # whose easing peak stays inside the velocity limit, so a full
            # gesture eases over several seconds — and retargeting mid-motion
            # starts from the current output rather than jumping back to the
            # old start.
            current_q = self._sample_segment(now)
            self._seg_current = list(current_q)

        try:
            command = pnd_adam_msg_dds__LowCmd_(self._DOF)
            command.mode_pr = 0
            for index, joint_name in enumerate(ADAM_PRO_JOINTS):
                motor = command.motor_cmd[index]
                motor.mode = 1
                motor.q = current_q[index] if index in targets else hold_q[index]
                motor.dq = 0.0
                motor.tau = 0.0
                kp, kd = self._pd_for_joint(joint_name)
                is_arm = joint_name.startswith((
                    "waist", "neck", "shoulder", "elbow", "wrist"))
                arm_scale = 1.0
                if is_arm and (soft_arms or release_started_at is not None):
                    arm_scale = 1.0 - release_ratio
                motor.kp = kp * arm_scale
                motor.kd = kd
                motor.ki = 0.0
            self._publisher.Write(command)
            self._writes += 1
            self._last_error = None
            if finish_release:
                with self._lock:
                    self._release_started_at = None
                    self._active = False
                    self._target_q.clear()
                    self._soft_arms = True
        except Exception as exc:
            self._last_error = f"rt/lowcmd write failed: {exc}"

    def _run(self):
        self._read_initial_state()
        interval = 1.0 / self._rate_hz
        previous = time.monotonic()
        while not self._stop_event.wait(interval):
            now = time.monotonic()
            self._write_command(min(0.1, now - previous))
            previous = now

    # Grouped moves are the primary way an agent steers one body segment: a
    # shoulder is pitch+roll+yaw on one side, an elbow is the single bend, a
    # wrist is yaw+pitch+roll.  They resolve to the same per-joint targets as
    # the 14 fine-grained ``set_<joint>`` verbs below, which stay available
    # for calibration and edge cases.
    _GROUP_JOINTS = {
        "set_shoulder": (("pitch_deg", "shoulder_pitch"),
                         ("roll_deg", "shoulder_roll"),
                         ("yaw_deg", "shoulder_yaw")),
        "set_elbow": (("bend_deg", "elbow"),),
        # Field names are namespaced per axis so that merging the three groups
        # into one property bag stays unambiguous: shoulder owns
        # pitch/roll/yaw, wrist owns wrist_yaw/wrist_pitch/wrist_roll.
        "set_wrist": (("wrist_yaw_deg", "wrist_yaw"),
                      ("wrist_pitch_deg", "wrist_pitch"),
                      ("wrist_roll_deg", "wrist_roll")),
    }
    _GROUP_TITLES = {
        "set_shoulder": "设置一侧肩膀（前后摆+内外摆+上臂旋转）",
        "set_elbow": "设置一侧肘部弯曲",
        "set_wrist": "设置一侧手腕（旋转+俯仰+侧摆）",
    }

    def get_tool(self) -> dict:
        actions = [*self._GROUP_JOINTS, "reset"]
        action_options = [
            {"const": action, "title": title}
            for action, title in self._GROUP_TITLES.items()
        ]
        action_options.append({"const": "reset", "title": "回到起始手臂角度"})
        side_ranges = {
            side: {
                "pitch_deg": (-207.0, 117.0, "前后摆"),
                "roll_deg": ((-36.0, 160.0, "向内/外摆") if side == "left"
                             else (-160.0, 36.0, "向内/外摆")),
                "yaw_deg": (-148.0, 148.0, "上臂旋转"),
                "bend_deg": (-143.0, 12.0, "肘部弯曲"),
                "wrist_yaw_deg": (-153.0, 153.0, "手腕旋转"),
                "wrist_pitch_deg": (-55.0, 55.0, "手腕俯仰"),
                "wrist_roll_deg": (-55.0, 55.0, "手腕侧摆"),
            }
            for side in ("left", "right")
        }
        side_field_kinds = {
            "shoulder_pitch": "pitch_deg", "shoulder_roll": "roll_deg",
            "shoulder_yaw": "yaw_deg",
            "elbow": "bend_deg",
            "wrist_yaw": "wrist_yaw_deg",
            "wrist_pitch": "wrist_pitch_deg",
            "wrist_roll": "wrist_roll_deg",
        }
        group_fields = {}
        group_params = {}
        for action, fields in self._GROUP_JOINTS.items():
            params = ["side"]
            schemas = {}
            for field, joint_kind in fields:
                key = side_field_kinds[joint_kind]
                left = side_ranges["left"][key]
                right = side_ranges["right"][key]
                # The roll axis is asymmetric across sides; use the union that
                # fits both and let the dispatcher clamp per-side.
                minimum = min(left[0], right[0])
                maximum = max(left[1], right[1])
                label = left[2]
                schemas[field] = {
                    "type": "number", "title": f"{label}目标角度（度）",
                    "minimum": minimum, "maximum": maximum,
                    "multipleOf": 1.0,
                    "description": (
                        f"绝对目标角度。左臂范围 [{left[0]:g}, {left[1]:g}] 度；"
                        f"右臂范围 [{right[0]:g}, {right[1]:g}] 度。"
                    ),
                }
                params.append(field)
            group_fields[action] = schemas
            group_params[action] = params

        properties = {
            "action": {"type": "string", "enum": actions, "oneOf": action_options},
            "side": {
                "type": "string", "title": "手臂",
                "enum": ["left", "right"], "default": "right",
                "description": "复合动作（set_shoulder/set_elbow/set_wrist）作用的手臂。",
            },
            **{field: schema for schemas in group_fields.values()
               for field, schema in schemas.items()},
            "duration_s": {
                "type": "number", "title": "动作时长（秒）",
                "minimum": 0.1, "maximum": 60.0,
                "description": (
                    "可选，本次过渡的期望时长。只能把动作放慢；"
                    "小于安全限速所需时长时会被驱动自动钳位，不会突破限速。"
                    "省略时使用默认平滑时长。"
                ),
            },
        }
        action_params = {
            action: {
                "params": [*params, "duration_s"],
                "description": self._GROUP_TITLES[action],
            }
            for action, params in group_params.items()
        }
        action_params["reset"] = {
            "params": ["duration_s"],
            "description": "回到开始控制时的手臂角度。",
        }
        return {
            "name": "arm_control",
            "type": "actuator",
            "description": (
                "Adam Pro 上肢（双臂+手腕，14 个关节）实时位置控制，走厂商 "
                "DDS rt/lowcmd 通道。角度单位为度(°)，取值为绝对值，"
                f"关节限位见各字段 minimum/maximum。"
                "通过 set_shoulder/set_elbow/set_wrist 选择部位，再设置该部位的一个或多个角度。"
                "可选 duration_s 放慢动作，加速请求会被限速自动钳位。"
                "前置条件：机器人已站立，且没有其它卡片正在占用上肢通道；"
                "执行前确认手臂活动范围内无人和障碍物。stop 会先按厂商顺序释放增益再停止下发。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": properties,
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": action_params,
                "x-resource": ["adam_upper_body"],
            },
        }

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="adam_arm_lowcmd")
            self._thread.start()

    def stop(self):
        with self._lock:
            if self._active or self._streaming:
                self._release_started_at = time.monotonic()
        # Give the official Kp ramp-down sequence a chance to reach zero.
        deadline = time.monotonic() + self._RELEASE_SECONDS + 0.2
        while time.monotonic() < deadline:
            with self._lock:
                if self._release_started_at is None:
                    break
            time.sleep(0.02)
        self._stop_event.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(1.0)
        self._thread = None

    def _stop_and_wait(self):
        """Stop action lifecycle: ramp gains down, then end lowcmd ownership."""
        with self._lock:
            if not (self._active or self._streaming):
                return {"success": True, "state": "idle"}
            self._release_started_at = time.monotonic()
        deadline = time.monotonic() + self._RELEASE_SECONDS + 0.25
        while time.monotonic() < deadline:
            with self._lock:
                releasing = self._release_started_at is not None
            if not releasing:
                break
            time.sleep(0.02)
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(1.0)
        self._thread = None
        with self._lock:
            self._streaming = False
            self._active = False
        return {"success": True, "state": "idle",
                "message": "Arm lowcmd control stopped after gain release"}

    def _ready_error(self):
        if self._publisher is None:
            return {"success": False, "code": "DDS_UNAVAILABLE",
                    "message": "rt/lowcmd publisher is unavailable"}
        if not self._state_ready.is_set():
            return {"success": False, "code": "LOWSTATE_UNAVAILABLE",
                    "message": "Waiting for a complete rt/lowstate message in developer mode"}
        if self._last_error:
            return {"success": False, "code": "DDS_WRITE_FAILED", "message": self._last_error}
        return None

    def _set_targets(self, targets: dict[str, float],
                     *, preferred_span: float | None = None,
                     velocity_limit: float | None = None):
        error = self._ready_error()
        if error:
            return error
        if not targets:
            # Activating rt/lowcmd ownership without a single joint target
            # would take the whole body away from whatever currently owns it
            # and then hold every joint where it stands, which looks like a
            # silent freeze to the caller.  Refuse instead.
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": "no upper-body joints were selected"}
        with self._lock:
            target_q = self._target_q.copy()
            target_q.update({self._joint_index(name): value
                             for name, value in targets.items()})
            # Sample the in-flight segment at the retarget instant, including
            # time elapsed since the worker's most recent write.
            now = time.monotonic()
            self._seg_current = self._sample_segment(now)
            self._target_q = target_q
            self._seg_start = {index: self._seg_current[index]
                               for index in self._target_q}
            self._seg_started_at = now
            max_distance = 0.0
            for index in self._target_q:
                max_distance = max(
                    max_distance,
                    abs(self._target_q[index] - self._seg_current[index]),
                )
            # Budget for the easing peak, not the average: the shortest span
            # whose midpoint stays at or below _MAX_VELOCITY_RAD_S is
            # 1.875 * distance / velocity.  The configured transition remains a
            # floor, so short moves are never quicker than the smoothing
            # constant and long moves ease within the velocity limit.
            # A caller may hand in a stricter velocity ceiling (semantic arm
            # gestures use a slower one so a salute or a wave reads as calm
            # rather than hurried).  When omitted the controller-wide limit
            # applies, so the raw arm_control/waist/head cards keep their
            # existing rhythm.
            limit = (velocity_limit if velocity_limit is not None
                     else self._MAX_VELOCITY_RAD_S)
            peak_span = (self._EASE_PEAK_RATE * max_distance / limit)
            if preferred_span is None:
                self._seg_span = max(
                    0.25, peak_span,
                    self._DEFAULT_TRANSITION_SECONDS if max_distance > 0 else 0.25)
            else:
                # A caller-chosen duration may only ever slow a move down: the
                # request is clamped up to the shortest span whose easing peak
                # still respects _MAX_VELOCITY_RAD_S, so speed control can
                # never defeat the limit the default policy enforces.  This is
                # what makes a short, quick gesture such as a wave possible
                # without punching through the velocity ceiling.
                requested = max(0.1, float(preferred_span))
                self._seg_span = (max(requested, peak_span)
                                  if max_distance > 0 else 0.25)
            self._release_started_at = None
            self._active = True
            self._streaming = True
            self._soft_arms = False
        # The worker may need one 20ms tick. This confirms the protocol write,
        # not physical movement, which DDS does not acknowledge.
        before = self._writes
        deadline = time.monotonic() + 0.15
        while self._writes <= before and time.monotonic() < deadline:
            time.sleep(0.005)
        if self._writes <= before:
            return {"success": False, "code": "DDS_WRITE_FAILED",
                    "message": self._last_error or "rt/lowcmd was not written"}
        return None

    def _preferred_span(self, args: dict) -> tuple[float | None, dict | None]:
        """Parse the optional ``duration_s`` request into a target span.

        Returns ``(span, error)``; ``span`` is ``None`` when the caller did not
        ask for a specific duration, which keeps the default smoothing policy.
        """
        if args.get("duration_s") is None:
            return None, None
        raw = args.get("duration_s")
        message = "duration_s must be a number in [0.1, 60.0]"
        if isinstance(raw, bool):
            return None, {"success": False, "code": "INVALID_ARGUMENT",
                          "message": message}
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None, {"success": False, "code": "INVALID_ARGUMENT",
                          "message": message}
        if not math.isfinite(value) or not 0.1 <= value <= 60.0:
            return None, {"success": False, "code": "INVALID_ARGUMENT",
                          "message": message}
        return value, None

    def _active_segment_span(self) -> float:
        with self._lock:
            return self._seg_span

    def _joint_state(self) -> dict:
        """Report the angles this card is writing, and how far from target.

        These are commanded values, not measured ones: the card only reads
        ``rt/lowstate`` once to capture its hold pose, so live joint feedback
        stays on the ``joints`` and ``motor_state`` sensor cards.
        """
        with self._lock:
            targets = dict(self._target_q)
            commanded = list(self._seg_current)
            span = self._seg_span
            started_at = self._seg_started_at
            active = self._active
        max_error = 0.0
        joints = {}
        for control, (label, joint, minimum, maximum) in ARM_JOINT_CONTROLS.items():
            index = self._joint_index(joint)
            target = targets.get(index)
            error = abs(target - commanded[index]) if target is not None else None
            if error is not None:
                max_error = max(max_error, error)
            joints[control] = {
                "label": label,
                "commanded_deg": round(math.degrees(commanded[index]), 2),
                "target_deg": (round(math.degrees(target), 2)
                               if target is not None else None),
                "error_deg": (round(math.degrees(error), 2)
                              if error is not None else None),
                "limits_deg": {"minimum": minimum, "maximum": maximum},
            }
        tracking = len(targets)
        return {
            "state": "active" if active else "idle",
            "tracking_joint_count": tracking,
            "settled": bool(tracking) and max_error < math.radians(0.5),
            "segment": {"span_s": round(span, 3),
                        "elapsed_s": round(max(0.0, time.monotonic() - started_at), 3)},
            "joints": joints,
            "protocol": "rt/lowcmd",
            "angle_source": "commanded (rt/lowcmd targets); measured feedback is on "
                            "the joints and motor_state sensor cards",
        }

    def _group_targets(self, action: str, args: dict) -> dict | None:
        """Resolve a grouped shoulder/elbow/wrist request into radian targets.

        Returns ``None`` when ``action`` is not one of the compound verbs.
        Raises ``ValueError`` for a bad side or an out-of-range angle, which the
        caller turns into the standard INVALID_ARGUMENT reply.
        """
        fields = self._GROUP_JOINTS.get(action)
        if fields is None:
            return None
        side = args.get("side", "right")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        targets = {}
        for field, joint_kind in fields:
            value = args.get(field)
            if value is None:
                continue
            control = f"{side}_{joint_kind}"
            joint_name, radians = _arm_target_radians(control, value)
            targets[joint_name] = radians
        if not targets:
            raise ValueError(
                "provide at least one of the advertised degree fields for "
                f"{action} (side={side})")
        return targets

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            # A canvas stop tears the lowcmd worker down (_stop_and_wait sets
            # the stop event and clears the thread).  A later start must bring
            # the writer back, or every arm target fails DDS_WRITE_FAILED until
            # the container restarts.  ``start`` is idempotent: it reuses a
            # live thread and only spawns one when the previous one exited.
            self.start()
            return {"state": "ready"}
        if action == "stop":
            return self._stop_and_wait()
        if action == "reset":
            error = self._ready_error()
            if error:
                return error
            if self._hold_q is None:
                return {"success": False, "code": "LOWSTATE_UNAVAILABLE",
                        "message": "No startup arm pose captured yet"}
            targets = {
                joint: self._hold_q[self._joint_index(joint)]
                for _, joint, _, _ in ARM_JOINT_CONTROLS.values()
            }
            span, span_error = self._preferred_span(args)
            if span_error:
                return span_error
            error = self._set_targets(targets, preferred_span=span)
            if error:
                return error
            return {"success": True, "state": "active", "action": "reset",
                    "joints_set": len(targets), "duration_s": span,
                    "protocol": "rt/lowcmd"}
        if action == "get_state":
            return self._joint_state()
        # Compound shoulder/elbow/wrist verbs land in the same segment as a
        # single-joint request, so they smooth together instead of racing.
        if action in self._GROUP_JOINTS:
            try:
                targets = self._group_targets(action, args)
            except (TypeError, ValueError) as exc:
                return {"success": False, "code": "INVALID_ARGUMENT",
                        "message": str(exc)}
            span, error = self._preferred_span(args)
            if error:
                return error
            error = self._set_targets(targets, preferred_span=span)
            if error:
                return error
            return {
                "success": True, "state": "active", "action": action,
                "side": args.get("side", "right"),
                "joints_set": len(targets), "duration_s": span,
                "protocol": "rt/lowcmd",
            }
        if action in ARM_ACTIONS:
            try:
                control = ARM_ACTIONS[action]
                field = f"{control}_deg"
                joint_name, radians = _arm_target_radians(control, args.get(field))
            except (TypeError, ValueError) as exc:
                return {"success": False, "code": "INVALID_ARGUMENT", "message": str(exc)}
            span, error = self._preferred_span(args)
            if error:
                return error
            error = self._set_targets({joint_name: radians}, preferred_span=span)
            if error:
                return error
            _, _, minimum, maximum = ARM_JOINT_CONTROLS[control]
            return {"success": True, "state": "active", "joint": control,
                    "angle_deg": float(args[field]), "duration_s": span,
                    "protocol": "rt/lowcmd",
                    "limits_deg": {"minimum": minimum, "maximum": maximum}}
        if action == "set_joints":
            raw = args.get("joints")
            if not isinstance(raw, dict) or not raw:
                return {"success": False, "code": "INVALID_ARGUMENT",
                        "message": "joints must be a non-empty mapping of joint "
                                   "name to absolute target degrees"}
            try:
                targets = dict(_arm_target_radians(control, degrees)
                               for control, degrees in raw.items())
            except (TypeError, ValueError) as exc:
                return {"success": False, "code": "INVALID_ARGUMENT", "message": str(exc)}
            span, error = self._preferred_span(args)
            if error:
                return error
            error = self._set_targets(targets, preferred_span=span)
            if error:
                return error
            return {"success": True, "state": "active", "action": "set_joints",
                    "joints_set": len(targets),
                    "joints_deg": {control: float(degrees)
                                   for control, degrees in raw.items()},
                    "duration_s": span, "protocol": "rt/lowcmd"}
        if action == "preset":
            pose = args.get("pose")
            if pose not in ARM_POSES:
                return {"success": False, "code": "INVALID_ARGUMENT",
                        "message": "pose must be one of the advertised Adam upper-body poses"}
            error = self._ready_error()
            if error:
                return error
            _, targets_deg = ARM_POSES[pose]
            try:
                targets = dict(_arm_target_radians(control, degrees)
                               for control, degrees in targets_deg.items())
                if pose == "neutral":
                    targets = {joint: self._hold_q[self._joint_index(joint)]
                               for _, joint, _, _ in ARM_JOINT_CONTROLS.values()}
            except (TypeError, ValueError) as exc:
                return {"success": False, "code": "INVALID_ARGUMENT", "message": str(exc)}
            span, span_error = self._preferred_span(args)
            if span_error:
                return span_error
            error = self._set_targets(targets, preferred_span=span)
            if error:
                return error
            return {"success": True, "state": "active", "pose": pose,
                    "joints_set": len(targets), "duration_s": span,
                    "protocol": "rt/lowcmd"}
        if action == "info":
            return {"state": "active" if self._active else "idle",
                    "lowstate_ready": self._state_ready.is_set(),
                    "dds_writer_ready": self._publisher is not None,
                    "writes": self._writes, "last_error": self._last_error,
                    "protocol": "rt/lowcmd"}
        return None


ArmPlugin = ArmControlPlugin


class WaistControlPlugin:
    """Dedicated Adam Pro waist card sharing the safe lowcmd controller.

    The waist joints were previously folded into ``arm_control``.  Splitting
    them out lets an agent discover "bow" or "turn the waist" directly, while
    every target still routes through the single ``ArmControlPlugin`` that owns
    ``rt/lowcmd``.
    """

    PREFIX = "waist_control"

    def __init__(self, control: ArmControlPlugin):
        self._control = control

    def get_tool(self):
        actions = ["set_angles", "reset"]
        action_options = [
            {"const": "set_angles", "title": "设置腰部角度"},
            {"const": "reset", "title": "回到起始腰部角度"},
        ]
        properties = {
            "action": {"type": "string", "enum": actions, "oneOf": action_options},
            "duration_s": {
                "type": "number", "title": "动作时长（秒）",
                "minimum": 0.1, "maximum": 60.0,
                "description": (
                    "可选，本次过渡的期望时长。只能把动作放慢；"
                    "小于安全限速所需时长时会被驱动自动钳位。省略时使用默认平滑时长。"
                ),
            },
        }
        angle_fields = []
        for control, (label, _, minimum, maximum) in WAIST_JOINT_CONTROLS.items():
            field = f"{control}_deg"
            angle_fields.append(field)
            properties[field] = {
                "type": "number", "title": f"{label}目标角度（度）",
                "minimum": minimum, "maximum": maximum, "multipleOf": 1.0,
                "description": f"绝对目标角度，范围 [{minimum:g}, {maximum:g}] 度。",
            }
        action_params = {
            "set_angles": {
                "params": [*angle_fields, "duration_s"],
                "description": "一次设置腰部 roll、pitch、yaw 中的一个或多个角度。",
            },
            "reset": {"params": ["duration_s"],
                      "description": "回到开始控制时的腰部角度。"},
        }
        return {
            "name": "waist_control",
            "type": "actuator",
            "description": (
                "Adam Pro 腰部（侧倾/前后俯仰/左右转动，3 个关节）实时位置控制，"
                "走厂商 DDS rt/lowcmd 通道。角度单位为度(°)。可选 duration_s 放慢动作。"
                "前置条件：机器人已站立，且没有其它卡片正在占用上肢通道。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": properties,
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": action_params,
                "x-resource": ["adam_upper_body"],
            },
        }

    def start(self):
        return self._control.start()

    def stop(self):
        return self._control.stop()

    def dispatch(self, action, args):
        # start/stop/info describe the controller this card delegates to.
        if action in ("start", "stop", "info"):
            return self._control.dispatch(action, args)
        if action == "reset":
            error = self._control._ready_error()
            if error:
                return error
            targets = {
                joint: self._control._hold_q[self._control._joint_index(joint)]
                for _, joint, _, _ in WAIST_JOINT_CONTROLS.values()
            }
            span, span_error = self._control._preferred_span(args)
            if span_error:
                return span_error
            error = self._control._set_targets(targets, preferred_span=span)
            return error or {"success": True, "state": "active",
                             "action": "reset", "duration_s": span}
        if action != "set_angles":
            return None
        try:
            targets = {}
            angles = {}
            for control in WAIST_JOINT_CONTROLS:
                field = f"{control}_deg"
                if args.get(field) is None:
                    continue
                joint, radians = _waist_target_radians(control, args[field])
                targets[joint] = radians
                angles[control] = float(args[field])
            if not targets:
                raise ValueError("provide at least one of roll_deg, pitch_deg or yaw_deg")
        except (TypeError, ValueError) as exc:
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": str(exc)}
        span, span_error = self._control._preferred_span(args)
        if span_error:
            return span_error
        error = self._control._set_targets(targets, preferred_span=span)
        if error:
            return error
        return {"success": True, "state": "active", "action": "set_angles",
                "angles_deg": angles, "joints_set": len(targets),
                "duration_s": span, "protocol": "rt/lowcmd"}


class HeadControlPlugin:
    """Dedicated Adam Pro head card sharing the safe lowcmd controller.

    The ZED Mini is mounted on the head, so this card is effectively the
    "camera aiming" control: neckYaw and neckPitch steer where the robot looks.
    """

    PREFIX = "head_control"

    def __init__(self, control: ArmControlPlugin):
        self._control = control

    def get_tool(self):
        actions = ["set_angles", "reset"]
        action_options = [
            {"const": "set_angles", "title": "设置头部角度"},
            {"const": "reset", "title": "回到起始头部角度"},
        ]
        properties = {
            "action": {"type": "string", "enum": actions, "oneOf": action_options},
            "duration_s": {
                "type": "number", "title": "动作时长（秒）",
                "minimum": 0.1, "maximum": 60.0,
                "description": (
                    "可选，本次过渡的期望时长。只能把动作放慢；"
                    "小于安全限速所需时长时会被驱动自动钳位。省略时使用默认平滑时长。"
                ),
            },
        }
        angle_fields = []
        for control, (label, _, minimum, maximum) in HEAD_JOINT_CONTROLS.items():
            field = f"{control}_deg"
            angle_fields.append(field)
            properties[field] = {
                "type": "number", "title": f"{label}目标角度（度）",
                "minimum": minimum, "maximum": maximum, "multipleOf": 1.0,
                "description": f"绝对目标角度，范围 [{minimum:g}, {maximum:g}] 度。",
            }
        action_params = {
            "set_angles": {
                "params": [*angle_fields, "duration_s"],
                "description": "一次设置头部 yaw、pitch 中的一个或两个角度。",
            },
            "reset": {"params": ["duration_s"],
                      "description": "回到开始控制时的头部角度。"},
        }
        return {
            "name": "head_control",
            "type": "actuator",
            "description": (
                "Adam Pro 头部（左右转动/上下俯仰，2 个关节）实时位置控制，"
                "走厂商 DDS rt/lowcmd 通道。头部装有 ZED 相机，因此本卡即"
                "「相机指向」控制。角度单位为度(°)，限位 ±60°。可选 duration_s 放慢动作。"
                "前置条件：机器人已站立，且没有其它卡片正在占用上肢通道。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": properties,
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": action_params,
                "x-resource": ["adam_upper_body"],
            },
        }

    def start(self):
        return self._control.start()

    def stop(self):
        return self._control.stop()

    def dispatch(self, action, args):
        # start/stop/info describe the controller this card delegates to.
        if action in ("start", "stop", "info"):
            return self._control.dispatch(action, args)
        if action == "reset":
            error = self._control._ready_error()
            if error:
                return error
            targets = {
                joint: self._control._hold_q[self._control._joint_index(joint)]
                for _, joint, _, _ in HEAD_JOINT_CONTROLS.values()
            }
            span, span_error = self._control._preferred_span(args)
            if span_error:
                return span_error
            error = self._control._set_targets(targets, preferred_span=span)
            return error or {"success": True, "state": "active",
                             "action": "reset", "duration_s": span}
        if action != "set_angles":
            return None
        try:
            targets = {}
            angles = {}
            for control in HEAD_JOINT_CONTROLS:
                field = f"{control}_deg"
                if args.get(field) is None:
                    continue
                joint, radians = _head_target_radians(control, args[field])
                targets[joint] = radians
                angles[control] = float(args[field])
            if not targets:
                raise ValueError("provide at least one of yaw_deg or pitch_deg")
        except (TypeError, ValueError) as exc:
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": str(exc)}
        span, span_error = self._control._preferred_span(args)
        if span_error:
            return span_error
        error = self._control._set_targets(targets, preferred_span=span)
        if error:
            return error
        return {"success": True, "state": "active", "action": "set_angles",
                "angles_deg": angles, "joints_set": len(targets),
                "duration_s": span, "protocol": "rt/lowcmd"}


class ArmGesturePlugin:
    """Named arm gestures played through the shared upper-body controller.

    A gesture is a trajectory, not a second controller: every target still goes
    through ``ArmControlPlugin``, which owns ``rt/lowcmd`` and writes the
    complete 31-motor packet.  Each entry declares the pose it plays, whether
    that pose is symmetric, and the arm to use when the caller does not choose
    one — a symmetric pose defaults to both arms while a one-armed pose
    defaults to the right arm.  The previous flat ``side="right"`` default made
    ``welcome`` and ``raise`` silently drive one arm only.
    """

    PREFIX = "arm_gesture"

    # Semantic gestures are performances, not raw positioning: they play at a
    # slower joint-velocity ceiling than the bare arm_control card so a salute
    # or a welcome reads calm and deliberate instead of snapping to the pose.
    # wave keeps the controller-wide limit (its side-to-side rhythm is meant to
    # be quick), and a caller-chosen duration_s still clamps below this.
    _GESTURE_VELOCITY_RAD_S = 0.3

    # name -> (ARM_POSES key, symmetric, default side)
    _GESTURES = {
        # One-armed poses: the ARM_POSES table defines them on the right arm and
        # the left is generated by mirroring, so each pose is written once.
        "salute": ("salute", False, "right"),
        "high_five": ("arm_forward_high", False, "right"),
        "handshake": ("handshake_ready", False, "right"),
        "wave": ("wave_ready", False, "right"),
        "welcome": ("arms_open", True, "both"),
        "raise": ("hands_up", True, "both"),
        "reset": ("neutral", True, "both"),
    }

    # Three side-to-side cycles, played after the synchronous raise.  Each span
    # is short on purpose: `_set_targets` clamps it up to the shortest span
    # whose easing peak stays inside `_MAX_VELOCITY_RAD_S`, so the rhythm is
    # preserved whenever the safety limit allows it and never breaks it.
    _WAVE_SEQUENCE = (
        ("wave_out", 0.7), ("wave_in", 0.7),
        ("wave_out", 0.7), ("wave_in", 0.7),
        ("wave_out", 0.7), ("wave_in", 0.7),
    )
    _WAVE_LOWER_SECONDS = 0.9
    _HANDSHAKE_ELBOW_SEQUENCE = (-72.0, -88.0, -72.0, -88.0, -80.0)
    _HANDSHAKE_SEGMENT_SECONDS = 0.5
    _SEQUENCE_TIMEOUT_S = 60

    # A gesture is a whole performance, so the hand shape it demands is part
    # of the card rather than a second card the caller has to sequence.
    # `None` means the arm move stands alone (the hand keeps whatever it is
    # currently doing).
    _GESTURE_HAND_SHAPES = {
        "salute": "flat_hand",
        "high_five": "open_palm",
        "handshake": "handshake",
        "wave": "open_palm",
    }

    def __init__(self, control: ArmControlPlugin, hand=None):
        self._control = control
        # Optional hand card.  Keeping it optional preserves the single-argument
        # construction the contract tests use, and lets a deployment without
        # hands keep the arm-only gestures working.
        self._hand = hand
        self._sequence_lock = threading.Lock()
        self._sequence_id = None

    @staticmethod
    def _sides_in(values: dict) -> set:
        return {control.split("_", 1)[0] for control in values
                if control.startswith(("left_", "right_"))}

    @classmethod
    def _symmetric_pose(cls, pose: str) -> bool:
        """A pose is symmetric when it is written for both arms."""
        return cls._sides_in(ARM_POSES[pose][1]) == {"left", "right"}

    def _targets_for(self, pose: str, side: str) -> dict:
        """Resolve one pose plus a requested arm into absolute radian targets."""
        if pose == "neutral":
            # `neutral` has no joint table of its own: it means "return the
            # selected arm axes to their zero target", which is also how
            # `reset` has always been expressed.
            selected = {control: 0.0 for control in ARM_JOINT_CONTROLS
                        if side == "both" or control.startswith(f"{side}_")}
        else:
            values = ARM_POSES[pose][1]
            if self._symmetric_pose(pose):
                selected = {control: degrees for control, degrees in values.items()
                            if side == "both" or control.startswith(f"{side}_")}
            else:
                if side == "left":
                    values = self._control.mirror_targets(values)
                selected = dict(values)
        return dict(_arm_target_radians(control, value)
                    for control, value in selected.items())

    def get_tool(self):
        one_armed = [name for name, (_, symmetric, _) in self._GESTURES.items()
                     if not symmetric]
        return {
            "name": "arm_gesture", "type": "actuator",
            "description": (
                "Adam 上肢语义动作：salute 单手敬礼、high_five 单手肩高前伸（击掌预备）、"
                "handshake 单手屈肘前伸并往复握手、wave 单手挥手（自动完成抬手-摆动-放下）、"
                "welcome 双臂张开、raise 双手举起、reset 归位。"
                f"单臂动作（{'/'.join(one_armed)}）只能选 side=left 或 side=right，"
                "对称动作（welcome/raise/reset）可用 side=both。"
                "绑定了手型的动作（salute=flat_hand 并拢伸掌、high_five/wave=open_palm 张开手掌、"
                "handshake=handshake 握手手型）会在手臂到位的同时把对应手型一并下发；"
                "未绑定的动作只动手臂。"
                "前置条件：机器人已站立，上肢通道没有被其它卡片占用，"
                "执行前确认手臂活动范围内无人和障碍物。"
            ),
            "inputSchema": {"type": "object", "properties": {
                "action": {
                    "type": "string",
                    "enum": [*self._GESTURES, "stop", "info"],
                    "oneOf": [{"const": name, "title": title}
                              for name, title in (
                                  ("salute", "单手敬礼"),
                                  ("high_five", "单手肩高前伸（击掌预备）"),
                                  ("handshake", "单手屈肘前伸并往复握手"),
                                  ("wave", "单手挥手（抬手-摆动-放下）"),
                                  ("welcome", "双臂张开"),
                                  ("raise", "双手举起"),
                                  ("reset", "上肢归位"),
                                  ("stop", "停止发上肢目标"),
                                  ("info", "查看上肢状态"),
                              )],
                },
                "side": {
                    "type": "string", "title": "手臂",
                    "enum": ["left", "right", "both"],
                    "default": "right",
                    "description": (
                        "选择执行动作的手臂。对称动作默认双臂(both)，"
                        "单臂动作默认右臂(right)；单臂动作不支持 both。"
                    ),
                },
                "duration_s": {
                    "type": "number", "title": "动作时长（秒）",
                    "minimum": 0.1, "maximum": 60.0,
                    "description": "可选，只能放慢动作；小于安全限速所需时长会被自动钳位。",
                },
            }, "required": ["action"], "additionalProperties": False,
            "x-action-params": {
                **{name: {"params": ["side", "duration_s"],
                          "description": f"{ARM_POSES[pose][0]}，可选放慢动作"}
                   for name, (pose, _, _) in self._GESTURES.items()},
                "stop": {"params": [],
                         "description": "取消正在进行的挥手序列并停止上肢目标发布。"},
                "info": {"params": [], "description": "查看上肢目标发布状态。"},
            },
            "x-resource": ["adam_upper_body"],
            # Sequence gestures return once accepted and report their terminal
            # status through Agent Core completion notifications.
            "x-completion": {"actions": ["wave", "handshake"],
                             "timeout": self._SEQUENCE_TIMEOUT_S},
            },
        }

    def start(self):
        return self._control.start()

    def stop(self):
        self._cancel_sequence()
        return self._control.stop()

    def _cancel_sequence(self):
        """Drop ownership of the running wave so its worker reports cancelled."""
        with self._sequence_lock:
            self._sequence_id = None

    def _sequence_cancelled(self, action_id: str) -> bool:
        with self._sequence_lock:
            return self._sequence_id != action_id

    def _hold_sequence(self, action_id: str, seconds: float) -> bool:
        """Wait out one segment, returning False as soon as it is cancelled."""
        deadline = time.monotonic() + max(0.0, float(seconds))
        while True:
            if self._sequence_cancelled(action_id):
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True
            time.sleep(min(0.05, remaining))

    def _apply_gesture_hand(self, action: str, side: str):
        """Apply the bound hand shape, returning an error dict or ``None``.

        ``side == "both"`` poses keep both hands free, so there is nothing to
        apply; arm-only gestures (``None`` in the table) are the same.  When a
        shape is bound the call goes through ``HandGesturePlugin.dispatch`` so
        the shape still honours the hand card's own side validation and safety
        ramp.
        """
        shape = self._GESTURE_HAND_SHAPES.get(action)
        if shape is None or self._hand is None or side == "both":
            return None
        result = self._hand.dispatch(shape, {"side": side})
        if isinstance(result, dict) and (result.get("state") == "error"
                                         or result.get("success") is False):
            return result
        return None

    def _finish_sequence(self, action_id: str, status: str, result: dict):
        with self._sequence_lock:
            if self._sequence_id == action_id:
                self._sequence_id = None
        _notify_action_completion(action_id, status, result, self.PREFIX)

    def _play_wave(self, side: str, action_id: str, ready_span: float):
        status = "completed"
        result = {"gesture": "wave", "side": side}
        # The final step lowers the arm again: a greeting that leaves the hand
        # in the air would need a second call to clean up, which the canvas does
        # not make.  Keeping it inside the loop means a stop also stops the
        # lowering, instead of moving the arm once more after the caller asked
        # it to stand still.
        steps = [*self._WAVE_SEQUENCE, ("neutral", self._WAVE_LOWER_SECONDS)]
        try:
            if not self._hold_sequence(action_id, ready_span):
                status = "cancelled"
                result = {"gesture": "wave", "side": side,
                          "reason": "superseded or stopped"}
                return
            for pose, hold_seconds in steps:
                with self._sequence_lock:
                    if self._sequence_id != action_id:
                        status = "cancelled"
                        result = {"gesture": "wave", "side": side,
                                  "reason": "superseded or stopped"}
                        return
                    error = self._control._set_targets(
                        self._targets_for(pose, side), preferred_span=hold_seconds)
                    actual_span = self._control._active_segment_span()
                if error:
                    status = "failed"
                    result = {"gesture": "wave", "side": side, "error": error}
                    return
                if not self._hold_sequence(action_id, actual_span):
                    status = "cancelled"
                    result = {"gesture": "wave", "side": side,
                              "reason": "superseded or stopped"}
                    return
        except Exception as exc:
            status = "failed"
            result = {"gesture": "wave", "side": side, "error": str(exc)}
        finally:
            self._finish_sequence(action_id, status, result)

    def _play_handshake(self, side: str, action_id: str, ready_span: float):
        status = "completed"
        result = {"gesture": "handshake", "side": side}
        elbow_control = f"{side}_elbow"
        try:
            if not self._hold_sequence(action_id, ready_span):
                status = "cancelled"
                result = {"gesture": "handshake", "side": side,
                          "reason": "superseded or stopped"}
                return
            for degrees in self._HANDSHAKE_ELBOW_SEQUENCE:
                joint, target = _arm_target_radians(elbow_control, degrees)
                with self._sequence_lock:
                    if self._sequence_id != action_id:
                        status = "cancelled"
                        result = {"gesture": "handshake", "side": side,
                                  "reason": "superseded or stopped"}
                        return
                    error = self._control._set_targets(
                        {joint: target},
                        preferred_span=self._HANDSHAKE_SEGMENT_SECONDS,
                        velocity_limit=self._GESTURE_VELOCITY_RAD_S)
                    actual_span = self._control._active_segment_span()
                if error:
                    status = "failed"
                    result = {"gesture": "handshake", "side": side,
                              "error": error}
                    return
                if not self._hold_sequence(action_id, actual_span):
                    status = "cancelled"
                    result = {"gesture": "handshake", "side": side,
                              "reason": "superseded or stopped"}
                    return
        except Exception as exc:
            status = "failed"
            result = {"gesture": "handshake", "side": side,
                      "error": str(exc)}
        finally:
            self._finish_sequence(action_id, status, result)

    def dispatch(self, action, args):
        # start/stop/info describe the controller this card delegates to.
        # Forwarding them keeps the card startable from the canvas: a bare None
        # is reported as an unknown action, and a strict project start then
        # rolls the whole project back.
        if action == "stop":
            self._cancel_sequence()
            return self._control.dispatch("stop", args)
        if action in ("start", "info"):
            return self._control.dispatch(action, args)
        entry = self._GESTURES.get(action)
        if entry is None:
            return None
        pose, symmetric, default_side = entry
        side = args.get("side") or default_side
        if side not in ("left", "right", "both"):
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": "side must be left, right or both"}
        if not symmetric and side == "both":
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": f"{action} is a one-armed gesture; "
                               "choose side=left or side=right"}
        span, error = self._control._preferred_span(args)
        if error:
            return error
        try:
            targets = self._targets_for(pose, side)
        except (TypeError, ValueError) as exc:
            return {"success": False, "code": "INVALID_ARGUMENT", "message": str(exc)}

        # A newly accepted gesture owns the shared controller.  Cancel any old
        # sequence before setting its first target so a stale worker cannot
        # overwrite the new command on its next segment.
        sequence_action = action in ("wave", "handshake")
        action_id = (f"adam_arm_{action}_{uuid.uuid4().hex[:8]}"
                     if sequence_action else None)
        with self._sequence_lock:
            self._sequence_id = None
            error = self._control._set_targets(
                targets, preferred_span=span,
                velocity_limit=self._GESTURE_VELOCITY_RAD_S)
            if error:
                return error
            ready_span = self._control._active_segment_span()
            hand_error = self._apply_gesture_hand(action, side)
            if not hand_error:
                self._sequence_id = action_id
        if hand_error:
            return {
                "success": False,
                "code": hand_error.get("code", hand_error.get("error", "HAND_FAILED")),
                "message": (
                    f"arm target accepted but the hand shape failed: "
                    f"{hand_error.get('message', hand_error.get('error', 'hand rejected'))}"),
                "arm_accepted": True,
            }

        if action not in ("wave", "handshake"):
            result = {"success": True, "state": "active", "gesture": action,
                      "side": side, "duration_s": span, "protocol": "rt/lowcmd"}
            if (self._GESTURE_HAND_SHAPES.get(action) is not None
                    and self._hand is not None and side != "both"):
                result["hand_gesture"] = self._GESTURE_HAND_SHAPES[action]
            return result

        # The ready target is accepted synchronously, while the worker waits for
        # its actual velocity-limited span before starting the repeated motion.
        worker = self._play_wave if action == "wave" else self._play_handshake
        threading.Thread(target=worker, args=(side, action_id, ready_span),
                         daemon=True, name=f"adam_arm_{action}_{side}").start()
        sequence_segments = (len(self._WAVE_SEQUENCE) + 1 if action == "wave"
                             else len(self._HANDSHAKE_ELBOW_SEQUENCE))
        result = {"success": True, "state": "active", "gesture": action,
                  "side": side, "action_id": action_id,
                  "sequence_segments": sequence_segments,
                  "protocol": "rt/lowcmd"}
        if action == "wave":
            result["auto_lower"] = True
        if self._hand is not None:
            result["hand_gesture"] = self._GESTURE_HAND_SHAPES[action]
        return result


# ===========================================================================
# HandPlugin — DDS rt/handcmd finger control
# ===========================================================================

class HandPlugin:
    """Continuous finger position control via DDS ``rt/handcmd``.

    Adam's hand controller expects the target to be refreshed continuously.
    The old implementation sent one message per MCP call, which made a
    command look successful while leaving the robot without a control stream.
    This plugin keeps the latest target and writes it at the configured rate.
    """

    PREFIX = "hand"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 dds_hand_pub=None, state_cache=None, **kwargs):
        self._namespace = namespace
        self._hand_type = str(plugin_config.get("hand_type", "pnd")).lower()
        try:
            configured_max = int(plugin_config.get("position_max", 0))
        except (TypeError, ValueError):
            configured_max = 0
        if configured_max <= 0:
            configured_max = HAND_POSITION_MAX
        # Adam's firmware only moves through 0..1000. Values above 1000 are
        # accepted by the uint32 DDS field but map to the same endpoint and
        # do not produce any additional motion.
        self._max_val = min(configured_max, HAND_POSITION_MAX)

        default_open = (
            HAND_DEFAULT_OPEN
            if self._hand_type in {"adam", "adam_client"}
            else [self._max_val] * HAND_POSITION_COUNT
        )
        self._open_positions = self._load_profile(
            plugin_config.get("open_positions"), default_open,
        )
        self._close_positions = self._load_profile(
            plugin_config.get("close_positions"), HAND_DEFAULT_CLOSED,
        )
        self._thumb_close_positions = self._load_profile(
            plugin_config.get("thumb_close_positions"),
            HAND_DEFAULT_THUMB_CLOSE,
            expected=4,
        )
        try:
            thumb_min = int(plugin_config.get("thumb_close_min_flex_position", 100))
        except (TypeError, ValueError):
            thumb_min = 100
        self._thumb_close_min_flex_position = max(0, min(self._max_val, thumb_min))
        try:
            self._control_rate_hz = float(plugin_config.get("control_rate_hz", 400))
        except (TypeError, ValueError):
            self._control_rate_hz = 400.0
        if self._control_rate_hz <= 0:
            self._control_rate_hz = 400.0
        # Limit the per-channel slew so a new hand target ramps in over
        # ``transition_seconds`` instead of snapping. The Adam hand firmware
        # applies no interpolation of its own, so without this every command
        # step arrives as a discontinuity at 400 Hz.
        try:
            self._transition_seconds = float(
                plugin_config.get("transition_seconds", 0.4))
        except (TypeError, ValueError):
            self._transition_seconds = 0.4
        self._transition_seconds = max(0.02, min(10.0, self._transition_seconds))
        try:
            self._state_timeout_sec = float(plugin_config.get("state_timeout_sec", 1.0))
        except (TypeError, ValueError):
            self._state_timeout_sec = 1.0
        if self._state_timeout_sec <= 0:
            self._state_timeout_sec = 1.0

        self._hand_pub = dds_hand_pub
        self._state_cache = state_cache or HandStateCache()
        self._owns_state_cache = state_cache is None
        self._lock = threading.Lock()
        self._target_positions = None
        self._command_positions = None
        self._active = False
        self._lifecycle_lock = threading.Lock()
        self._control_stop_event = None
        self._wake_event = threading.Event()
        self._control_thread = None
        self._closed = False
        self._last_write_ok = None
        self._last_write_at_ms = None
        self._last_write_error = None

    def _load_profile(self, configured, fallback: list[int], *, expected: int = HAND_POSITION_COUNT) -> list[int]:
        if configured is None:
            return _coerce_hand_positions(fallback, limit=self._max_val, expected=expected)
        try:
            return _coerce_hand_positions(configured, limit=self._max_val, expected=expected)
        except ValueError as exc:
            print(f"[hand] invalid position profile ({exc}); using default", flush=True)
            return _coerce_hand_positions(fallback, limit=self._max_val, expected=expected)

    def _base_positions(self):
        # Preserve the last commanded full target when applying a partial
        # update. Feedback may lag the command stream or be temporarily
        # unavailable; using it unconditionally would restore the other 11
        # channels from an old sample/open fallback on every set_fingers call.
        with self._lock:
            target = list(self._target_positions) if self._target_positions is not None else None
        if target is not None:
            return target

        positions = self._state_cache.fresh_positions(self._state_timeout_sec)
        if positions is None:
            return list(self._open_positions)
        try:
            return _coerce_hand_positions(positions, limit=self._max_val)
        except ValueError:
            return list(self._open_positions)

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "description": (
                "Adam 灵巧手抓取控制，走 DDS rt/handcmd，400Hz 持续下发目标位置。"
                "每只手 5 根手指、6 个电机通道，左右手共 12 个通道；"
                f"位置范围 0-{self._max_val}，0 为完全合上、{self._max_val} 为完全张开，"
                "中间值线性对应手指弯曲程度，超范围会被饱和处理。"
                "拇指有两个通道：屈伸和旋转（拇指旋转用于把拇指收拢到掌心侧）。"
                "主要用途：1) open/close 整体张开或握紧一只手或双手；"
                "2) grip 按百分比分级握持；3) set_positions 一次下发整手通道；"
                "4) set_fingers 微调单个通道。"
                "需要抓取前先分次合手时，优先用 grip；需要精确手型时用 set_positions。"
                "get_state 可读取左右手全部通道的当前位置用于确认是否到位。"
                "执行前确认手指和手掌周围没有障碍物。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "open", "close", "grip", "set_fingers", "set_positions",
                            "start", "stop", "info", "get_state",
                        ],
                    },
                    "side": {
                        "type": "string",
                        "title": "手",
                        "enum": ["left", "right", "both"],
                        "oneOf": [
                            {"const": "left", "title": "左手"},
                            {"const": "right", "title": "右手"},
                            {"const": "both", "title": "双手"},
                        ],
                        "description": "选择要控制的手；open/close/grip 支持双手(both)",
                    },
                    "channel": {
                        "type": "string",
                        "title": "通道",
                        "enum": list(HAND_CHANNEL_NAMES),
                        "oneOf": [
                            {"const": name, "title": HAND_CHANNEL_LABELS[name]}
                            for name in HAND_CHANNEL_NAMES
                        ],
                        "description": "每只手 6 个电机通道；左右手合计 12 个通道",
                    },
                    "value": {
                        "type": "integer",
                        "title": "目标值",
                        "minimum": 0,
                        "maximum": self._max_val,
                        "description": "该通道的目标位置值，0为合上，1000为张开。",
                    },
                    "grip_percent": {
                        "type": "number",
                        "title": "握持程度（%）",
                        "minimum": 0,
                        "maximum": 100,
                        "description": (
                            "0 为完全张开，100 为完全握紧，中间值按比例插值；"
                            "用于 grip 动作，比逐通道设值更方便。"
                        ),
                    },
                    "positions": {
                        "type": "array",
                        "title": "整手通道目标值",
                        "items": {"type": "integer", "minimum": 0,
                                  "maximum": self._max_val},
                        "minItems": 6,
                        "maxItems": HAND_POSITION_COUNT,
                        "description": (
                            "整只手的通道目标值，顺序为 "
                            + "、".join(HAND_CHANNEL_LABELS[name]
                                       for name in HAND_CHANNEL_NAMES)
                            + "。side=left/right 时传 6 个值；side=both 时传 12 个值"
                            "（前 6 个左手、后 6 个右手）。"
                            f"0 为合上，{self._max_val} 为张开。"
                        ),
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "open": {
                        "params": ["side"],
                        "description": "让左手、右手或双手张开到配置的张开姿势。",
                    },
                    "close": {
                        "params": ["side"],
                        "description": (
                            "让左手、右手或双手握紧：小指/无名指/中指/食指同时合拢，"
                            "拇指同步旋转并屈到安全位置。"
                        ),
                    },
                    "grip": {
                        "params": ["side", "grip_percent"],
                        "description": (
                            "按百分比设置握持程度，0 完全张开、100 完全握紧，"
                            "适合抓取前的分级合手。"
                        ),
                    },
                    "set_positions": {
                        "params": ["side", "positions"],
                        "description": (
                            "一次下发整只手的全部通道目标值，比 set_fingers 逐通道"
                            "设置更连贯。"
                        ),
                    },
                    "set_fingers": {
                        "params": ["side", "channel", "value"],
                        "description": (
                            "选择一只手的一个通道，持续下发该通道的目标位置值"
                        ),
                    },
                    "start": {"params": [], "description": "Enable the hand control worker"},
                    "stop": {"params": [], "description": "Stop sending new hand targets"},
                    "info": {"params": [], "description": "Return hand card status and configuration"},
                    "get_state": {
                        "params": [],
                        "description": "获取左右手全部通道的当前 position",
                    },
                },
            },
        }

    def _get_state(self) -> dict:
        payload = self._state_cache.snapshot(self._state_timeout_sec)
        if payload is not None:
            return payload
        cache_status = self._state_cache.status(self._state_timeout_sec)
        return {
            "state": "unavailable" if not cache_status["reader_available"] else "waiting",
            "fresh": False,
            "reader_available": cache_status["reader_available"],
            "source_topic": "rt/handstate",
            "message": (
                "DDS hand state reader is unavailable"
                if not cache_status["reader_available"]
                else "No hand state received yet"
            ),
        }

    def _publisher_available(self) -> bool:
        # A DDS writer is usable immediately after Init().  IsMatched() can
        # remain false while the robot-side subscriber is starting, and must
        # not turn a transient discovery delay into a permanently unavailable
        # card.  Write() below is the authoritative health check.
        return bool(HAS_PND_SDK and self._hand_pub is not None)

    def _publisher_matched(self):
        if not self._publisher_available():
            return False
        is_matched = getattr(self._hand_pub, "IsMatched", None)
        if not callable(is_matched):
            return True
        try:
            return bool(is_matched())
        except Exception:
            return False

    def start(self):
        with self._lifecycle_lock:
            if self._closed:
                return {
                    "state": "error",
                    "error": "HAND_CLOSED",
                    "message": "hand worker has been closed",
                }
            thread = self._control_thread
            stop_event = self._control_stop_event
            if thread is not None and thread.is_alive():
                if stop_event is None or not stop_event.is_set():
                    return self._status("ready")
                thread.join(1.5)
                if thread.is_alive():
                    return {
                        "state": "stopping",
                        "action": "start",
                        "control_active": False,
                        "worker_alive": True,
                    }
                self._control_thread = None
                self._control_stop_event = None

            self._state_cache.start()
            if not self._publisher_available():
                return self._status("unavailable")
            self._wake_event.clear()
            stop_event = threading.Event()
            self._control_stop_event = stop_event
            self._control_thread = threading.Thread(
                target=self._control_loop,
                args=(stop_event,),
                daemon=True,
                name="adam_hand_control",
            )
            self._control_thread.start()
        return self._status("ready")

    def stop(self):
        with self._lock:
            self._active = False
        with self._lifecycle_lock:
            thread = self._control_thread
            stop_event = self._control_stop_event
            if stop_event is not None:
                stop_event.set()
            self._wake_event.set()
            if thread is not None and thread is not threading.current_thread():
                thread.join(1.5)
            worker_alive = thread is not None and thread.is_alive()
            if not worker_alive:
                self._control_thread = None
                self._control_stop_event = None
        return {
            "state": "stopping" if worker_alive else "stopped",
            "action": "stop",
            "control_active": False,
            "worker_alive": worker_alive,
        }

    def close(self):
        with self._lifecycle_lock:
            self._closed = True
        self.stop()
        if self._owns_state_cache:
            self._state_cache.close()

    def _status(self, state: str | None = None) -> dict:
        with self._lock:
            active = self._active
            target = list(self._target_positions) if self._target_positions is not None else None
            last_write_ok = self._last_write_ok
            last_write_at_ms = self._last_write_at_ms
            last_write_error = self._last_write_error
        cache_status = self._state_cache.status(self._state_timeout_sec)
        result = {
            "state": state or ("active" if active else "idle"),
            "control_active": active,
            "worker_alive": (
                self._control_thread is not None
                and self._control_thread.is_alive()
            ),
            "closed": self._closed,
            "publisher_available": self._publisher_available(),
            "publisher_matched": self._publisher_matched(),
            "control_rate_hz": self._control_rate_hz,
            "position_max": self._max_val,
            "open_positions": list(self._open_positions),
            "close_positions": list(self._close_positions),
            "thumb_close_positions": list(self._thumb_close_positions),
            "thumb_close_min_flex_position": self._thumb_close_min_flex_position,
            "state_reader_available": cache_status["reader_available"],
            "state_fresh": cache_status["fresh"],
            "target": target,
            "last_write_ok": last_write_ok,
            "last_write_at_ms": last_write_at_ms,
        }
        if not result["publisher_available"]:
            result["error"] = "DDS_UNAVAILABLE"
        if last_write_error:
            result["last_write_error"] = last_write_error
        return result

    def _record_write_failure(self, message: str):
        with self._lock:
            self._last_write_ok = False
            self._last_write_at_ms = int(time.time() * 1000)
            self._last_write_error = message

    def _send_hand_cmd(self, positions: list[int]) -> bool:
        if not self._publisher_available():
            self._record_write_failure("DDS hand publisher is unavailable")
            return False
        try:
            cmd = pnd_adam_msg_dds__HandCmd_()
            for i, value in enumerate(positions):
                cmd.position[i] = value
            # Keep a broken/disconnected DDS writer from holding the worker
            # forever.  ChannelPublisher.Write supports this timeout and older
            # wrappers simply ignore it at the underlying DataWriter call.
            write_result = self._hand_pub.Write(cmd, timeout=0.2)
            ok = write_result is not False
            error = None if ok else "DDS hand command write returned false"
        except Exception as exc:
            ok = False
            error = str(exc)
        with self._lock:
            self._last_write_ok = ok
            self._last_write_at_ms = int(time.time() * 1000)
            self._last_write_error = error
        return ok

    def _control_loop(self, stop_event: threading.Event):
        period = 1.0 / self._control_rate_hz
        while not stop_event.is_set():
            try:
                if stop_event.is_set():
                    break
                with self._lock:
                    active = self._active
                    target = list(self._target_positions) if self._target_positions is not None else None
                    command = list(self._command_positions) if self._command_positions is not None else None
                if active and target is not None:
                    if command is None:
                        command = list(target)
                    max_step = max(1.0, self._max_val / (self._transition_seconds * self._control_rate_hz))
                    for i in range(len(command)):
                        diff = target[i] - command[i]
                        if diff > max_step:
                            command[i] += max_step
                        elif diff < -max_step:
                            command[i] -= max_step
                        elif diff != 0:
                            command[i] = target[i]
                    write_positions = [round(p) for p in command]
                    if not self._send_hand_cmd(write_positions):
                        if self._wake_event.wait(0.1):
                            self._wake_event.clear()
                        continue
                    with self._lock:
                        self._command_positions = list(command)
                if self._wake_event.wait(period):
                    self._wake_event.clear()
                    if stop_event.is_set():
                        break
            except Exception as exc:
                self._record_write_failure(str(exc))
                if self._wake_event.wait(0.1):
                    self._wake_event.clear()

    def _close_target(self) -> list[int]:
        """Build one close target for all four fingers and both thumb axes."""
        target = list(self._close_positions)
        for side_offset, thumb_offset in ((0, 0), (6, 2)):
            # The current Adam client mapping becomes more closed as the
            # flexion position decreases. Send both thumb axes in the same
            # target as the four non-thumb fingers so they move concurrently.
            target[side_offset + 4] = max(
                self._thumb_close_positions[thumb_offset],
                self._thumb_close_min_flex_position,
            )
            target[side_offset + 5] = self._thumb_close_positions[thumb_offset + 1]
        return target

    def _activate(self, positions: list[int], action: str) -> dict:
        with self._lifecycle_lock:
            if self._closed:
                return {
                    "state": "error",
                    "error": "HAND_CLOSED",
                    "message": "hand worker has been closed",
                }
        if not self._publisher_available():
            return {
                "state": "error",
                "error": "DDS_UNAVAILABLE",
                "message": "rt/handcmd publisher is unavailable",
            }
        result = self.start()
        if result.get("state") not in ("ready",):
            return result
        with self._lock:
            if self._command_positions is None:
                # First activation: start ramping from the current state so the
                # hand doesn't snap to the new target in one 400Hz step.
                base = self._state_cache.fresh_positions(self._state_timeout_sec)
                if base is None:
                    base = list(self._open_positions)
                else:
                    try:
                        base = _coerce_hand_positions(base, limit=self._max_val)
                    except ValueError:
                        base = list(self._open_positions)
                self._command_positions = list(base)
            self._target_positions = list(positions)
            self._active = True
        self._wake_event.set()
        return {
            "state": "active",
            "action": action,
            "target": list(positions),
            "control_rate_hz": self._control_rate_hz,
        }

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "get_state":
            return self._get_state()
        if action == "open":
            return self._activate_side(args.get("side"), self._open_positions, "open")
        if action == "close":
            return self._activate_side(args.get("side"), self._close_target(), "close")
        if action == "grip":
            side = args.get("side")
            if side not in self._SIDES:
                return self._invalid_side(side)
            raw_percent = args.get("grip_percent")
            message = "grip_percent must be a number in [0, 100]"
            if isinstance(raw_percent, bool) or not isinstance(
                    raw_percent, (int, float)):
                return {"state": "error", "error": "INVALID_ARGUMENT",
                        "message": message}
            percent = float(raw_percent)
            if not math.isfinite(percent) or not 0.0 <= percent <= 100.0:
                return {"state": "error", "error": "INVALID_ARGUMENT",
                        "message": message}
            # Interpolate between the configured open and safe-close shapes so
            # a partial grip still uses the tuned thumb targets rather than a
            # naive halfway value on both thumb axes.
            ratio = percent / 100.0
            shape = [round(open_value + (close_value - open_value) * ratio)
                     for open_value, close_value
                     in zip(self._open_positions, self._close_target())]
            result = self._activate_side(side, shape, "grip")
            if result.get("state") != "error":
                result["grip_percent"] = percent
            return result
        if action == "set_positions":
            return self._activate_positions(
                args.get("side"), args.get("positions"), "set_positions")
        if action == "set_fingers":
            side = args.get("side")
            channel = args.get("channel")
            if side not in ("left", "right"):
                return {
                    "state": "error",
                    "error": "INVALID_ARGUMENT",
                    "message": "side must be either left or right",
                }
            if channel not in HAND_CHANNEL_NAMES:
                return {
                    "state": "error",
                    "error": "INVALID_ARGUMENT",
                    "message": f"channel must be one of {list(HAND_CHANNEL_NAMES)}",
                }
            if "value" not in args:
                return {
                    "state": "error",
                    "error": "INVALID_ARGUMENT",
                    "message": "value is required",
                }
            try:
                value = _coerce_hand_positions(
                    [args.get("value")], limit=self._max_val, expected=1,
                )[0]
            except ValueError as exc:
                return {"state": "error", "error": "INVALID_ARGUMENT", "message": str(exc)}

            base = self._base_positions()
            positions = list(base)
            offset = 0 if side == "left" else 6
            positions[offset + HAND_CHANNEL_NAMES.index(channel)] = value
            return self._activate(positions, "set_fingers")
        if action == "info":
            return self._status()
        return None

    _SIDES = ("left", "right", "both")

    @staticmethod
    def _invalid_side(side):
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"side must be one of {list(HandPlugin._SIDES)}, got {side!r}"}

    def _apply_side(self, positions: list[int], side: str, values: list[int]):
        """Write a 6-value shape onto the requested hand(s) in place."""
        if side == "both":
            positions[0:6] = values[0:6]
            positions[6:12] = values[6:12]
        else:
            offset = 0 if side == "left" else 6
            positions[offset:offset + 6] = values[offset:offset + 6]
        return positions

    def _activate_side(self, side, source, action):
        if side not in self._SIDES:
            return self._invalid_side(side)
        positions = self._apply_side(self._base_positions(), side, source)
        result = self._activate(positions, action)
        result["side"] = side
        return result

    def _activate_positions(self, side, raw, action: str):
        """Validate a caller-supplied channel vector and send it as one target."""
        if side not in self._SIDES:
            return self._invalid_side(side)
        expected = HAND_POSITION_COUNT if side == "both" else 6
        if not isinstance(raw, (list, tuple)):
            return {"state": "error", "error": "INVALID_ARGUMENT",
                    "message": "positions must be an array of channel values"}
        if len(raw) != expected:
            return {"state": "error", "error": "INVALID_ARGUMENT",
                    "message": (f"positions must contain {expected} values for "
                                f"side={side} (6 per hand, 12 for both)")}
        try:
            values = _coerce_hand_positions(raw, limit=self._max_val,
                                            expected=expected)
        except ValueError as exc:
            return {"state": "error", "error": "INVALID_ARGUMENT",
                    "message": str(exc)}
        positions = self._apply_side(self._base_positions(), side, values)
        result = self._activate(positions, action)
        result["side"] = side
        return result


class HandGesturePlugin:
    """Named Adam hand shapes, composed from the DDS hand controller.

    Every shape is stored once as a six-value per-hand vector in
    ``HAND_CHANNEL_NAMES`` order — pinky, ring, middle, index, ``thumb_flex``,
    ``thumb_rotate`` — so a gesture means the same thing on either hand.  Four
    fingers and ``thumb_flex`` run 0 (fully curled) to 1000 (fully extended);
    ``thumb_rotate`` tucks the thumb across the palm.
    """

    PREFIX = "hand_gesture"

    _CURLED = [0, 0, 0, 0, 100, 1000]
    _EXTENDED = [1000, 1000, 1000, 1000, 1000, 0]
    _OPEN_KEY = "open_palm"
    _CLOSE_KEY = "fist"
    _GESTURES = {
        # `open_palm` and `fist` are resolved from the configured open/close
        # profile at dispatch time (see `_shape_for`) so a retuned robot keeps
        # its calibrated shapes; the rest are literal.
        _OPEN_KEY: None,
        _CLOSE_KEY: None,
        # A thumbs up folds the four fingers but extends the thumb, which is
        # the opposite thumb axes from a fist.  The previous definition reused
        # the fist vector verbatim, so the two were physically identical.
        "thumbs_up": [0, 0, 0, 0, 1000, 0],
        "victory": [0, 0, 1000, 1000, 100, 1000],
        "point": [0, 0, 0, 1000, 100, 1000],
        "rock": [1000, 0, 0, 1000, 100, 1000],
        # Half-closed four fingers with the thumb held out and partly rotated,
        # i.e. the shape that wraps around another hand.
        "handshake": [300, 300, 300, 300, 1000, 600],
        # flat_hand: fingers fully extended like open_palm, but the thumb
        # flexes in to lie alongside the fingers （并拢） instead of spreading
        # out, so the two shapes stay physically distinguishable.
        "flat_hand": [1000, 1000, 1000, 1000, 0, 0],
        "finger_gun": [0, 0, 0, 1000, 1000, 600],
        "ok_sign": [1000, 1000, 1000, 0, 0, 0],
        "call_me": [1000, 0, 0, 0, 1000, 0],
        "claw": [400, 400, 400, 400, 400, 600],
    }
    _LABELS = {
        "open_palm": "张开手掌",
        "fist": "握拳",
        "thumbs_up": "点赞（拇指伸出）",
        "victory": "V 手势（食指+中指）",
        "point": "指向（食指）",
        "rock": "摇滚手势（食指+小指）",
        "handshake": "握手手型（半握+拇指张开）",
        "flat_hand": "并拢伸掌",
        "finger_gun": "手枪手型（食指+拇指伸出）",
        "ok_sign": "OK 手型",
        "call_me": "打电话手型（拇指+小指伸出）",
        "claw": "爪型（五指半屈）",
    }

    def __init__(self, control: HandPlugin):
        self._control = control

    def _shape_for(self, gesture: str, side: str) -> list[int]:
        """Return the six per-hand channel values for a gesture."""
        if gesture == self._CLOSE_KEY:
            close = self._control._close_target()
            return close[0:6] if side == "left" else close[6:12]
        if gesture == self._OPEN_KEY:
            opened = self._control._open_positions
            return opened[0:6] if side == "left" else opened[6:12]
        return list(self._GESTURES[gesture])

    def get_tool(self):
        return {"name": "hand_gesture", "type": "actuator",
                "description": (
                    "Adam 手部语义手型：open_palm 张开手掌、fist 握拳、"
                    "thumbs_up 点赞（拇指伸出）、victory V 手势、point 指向、"
                    "rock 摇滚手势、handshake 握手手型。"
                    "只改变指定手的手指形状，另一只手的当前目标保持不变；"
                    "位置范围 0-1000，0 为完全弯曲、1000 为完全伸直。"
                    "需要完整动作而不是静止手型时，配合 arm_gesture 使用"
                    "（例如迎宾用 arm_gesture.wave + open_palm，握手用 "
                    "arm_gesture.handshake + handshake）。"
                    "执行前确认手指和手掌周围没有障碍物。"
                ),
                "inputSchema": {"type": "object", "properties": {
                    "action": {
                        "type": "string",
                        "enum": [*self._GESTURES, "stop", "info"],
                        "oneOf": [{"const": name, "title": self._LABELS[name]}
                                  for name in self._GESTURES],
                    },
                    "side": {
                        "type": "string", "title": "手",
                        "enum": ["left", "right", "both"],
                        "default": "right",
                        "description": "选择执行手型的手；both 表示双手同时做同一手型。",
                    },
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    **{gesture: {"params": ["side"],
                                 "description": self._LABELS[gesture]}
                       for gesture in self._GESTURES},
                    "stop": {"params": [],
                             "description": "停止发送手部目标，保持当前形状。"},
                    "info": {"params": [], "description": "查看手部控制状态与配置。"},
                },
                "x-resource": ["adam_hands"]},
        }

    def start(self):
        return self._control.start()

    def stop(self):
        return self._control.stop()

    def dispatch(self, action, args):
        # start/stop/info describe the hand controller this card delegates to.
        # Forwarding them keeps the card startable from the canvas: a bare None
        # is reported as an unknown action, and a strict project start then
        # rolls the whole project back.
        if action in ("start", "info", "stop"):
            return self._control.dispatch(action, args)
        if action not in self._GESTURES:
            return None
        side = args.get("side") or "right"
        if side not in self._control._SIDES:
            return self._control._invalid_side(side)
        positions = self._control._base_positions()
        # `both` reuses the same per-hand shape on each side, so it needs the
        # six channels applied twice rather than one twelve-value vector.
        if side == "both":
            shape = self._shape_for(action, "right")
            positions[0:6] = shape
            positions[6:12] = shape
        else:
            offset = 0 if side == "left" else 6
            positions[offset:offset + 6] = self._shape_for(action, side)
        result = self._control._activate(positions, action)
        result["side"] = side
        result["gesture"] = action
        return result


# ---------------------------------------------------------------------------
# Hand-state sensor card
# ---------------------------------------------------------------------------

class _HandStatePublisherNode(Node):
    """Publishes the shared DDS hand-state cache as JSON."""

    def __init__(self, namespace: str, state_cache: HandStateCache,
                 publish_rate_hz: float, state_timeout_sec: float):
        super().__init__("adam_hand_state_publisher")
        self._state_cache = state_cache
        self._state_timeout_sec = state_timeout_sec
        self._topic = f"/{namespace}/state/hand"
        self._active = False
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(String, self._topic, _best_effort_qos())
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)

    def set_active(self, active: bool):
        with self._lock:
            self._active = bool(active)

    def _publish(self):
        with self._lock:
            active = self._active
        if not active:
            return
        payload = self._state_cache.snapshot(self._state_timeout_sec)
        if payload is None:
            return
        message = String()
        message.data = json.dumps(payload)
        self._publisher.publish(message)


class HandStatePlugin:
    """Exposes actual 12-channel hand positions as a read-only sensor."""

    PREFIX = "hand_state"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 state_cache: HandStateCache, **kwargs):
        self._executor = executor
        self._state_cache = state_cache
        self._state_timeout_sec = float(plugin_config.get("state_timeout_sec", 1.0))
        rate = float(plugin_config.get("publish_rate_hz", 50))
        self._node = _HandStatePublisherNode(
            namespace, state_cache, rate, self._state_timeout_sec)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "hand_state",
            "type": "sensor",
            "description": "Adam hand state — actual positions for both 6-channel hands",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._node._topic, "format": "data/json"}],
        }

    def start(self):
        self._state_cache.start()
        self._node.set_active(True)

    def stop(self):
        self._node.set_active(False)

    def close(self):
        self.stop()
        _destroy_ros_node(self._executor, self._node)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "hand_state"):
            payload = self._state_cache.snapshot(self._state_timeout_sec)
            if payload is not None:
                return {
                    **payload,
                    "topic_out": [{"topic": self._node._topic, "format": "data/json"}],
                }
            status = self._state_cache.status(self._state_timeout_sec)
            return {
                "state": "unavailable" if not status["reader_available"] else "waiting",
                "fresh": False,
                "topic_out": [{"topic": self._node._topic, "format": "data/json"}],
                **status,
            }
        return None


# ---------------------------------------------------------------------------
# Local ZED Mini camera cards
# ---------------------------------------------------------------------------

class _LatestFrameQueue:
    """A bounded queue that keeps the newest frame and drops stale frames."""

    def __init__(self):
        self._queue = queue.Queue(maxsize=1)

    def put_latest(self, frame):
        while True:
            try:
                self._queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    continue

    def get(self, timeout):
        return self._queue.get(timeout=timeout)


class ZedCameraPlugin:
    """Publish the Adam ZED Mini through the local ZED Python SDK.

    The camera is physically attached to the Jetson running this container, so
    there is no reason to consume the separate ZED network-stream sender.  One
    capture thread owns the SDK camera, copies requested streams into bounded
    latest-frame queues, and dedicated workers perform the expensive encoding,
    conversion and ROS2 publication for RGB, depth and the optional point
    cloud.
    """

    PREFIX = "camera"

    _CARD_NAMES = (
        "camera_head",
        "camera_depth",
        "camera_pointcloud",
    )

    _FORMATS = {
        "camera_head": "image/jpeg",
        "camera_depth": "image/depth-zlib",
        "camera_pointcloud": "sensor/pointcloud",
    }

    def __init__(self, plugin_config, namespace, executor):
        self._config = dict(plugin_config or {})
        self._namespace = namespace
        self._topics = {
            "camera_head": f"/{namespace}/camera/head",
            "camera_depth": f"/{namespace}/camera/head/depth",
            "camera_pointcloud": f"/{namespace}/camera/head/points",
        }

        pointcloud_config = self._config.get("pointcloud", {})
        if not isinstance(pointcloud_config, dict):
            pointcloud_config = {}
        self._pointcloud_config = pointcloud_config
        self._pointcloud_enabled = bool(
            pointcloud_config.get(
                "enabled", self._config.get("pointcloud_enabled", True)))
        # The three cards share one ZED capture thread, but each card has its
        # own publication lifecycle.  Keep every advertised card live by
        # default: dashboard card creation is not guaranteed to invoke a
        # separate legacy start action.
        self._card_enabled = {
            # The dashboard treats these as live sensor cards. Start the RGB
            # and depth flows with the driver so opening a card never depends
            # on a separate, legacy MCP start request succeeding first.
            "camera_head": True,
            "camera_depth": True,
            "camera_pointcloud": self._pointcloud_enabled,
        }
        self._rgb_hz = max(1.0, min(float(self._config.get("rgb_hz", 15)), 30.0))
        self._depth_hz = max(1.0, min(float(self._config.get("depth_hz", 8)), 15.0))
        self._pointcloud_hz = max(
            0.2, min(float(pointcloud_config.get("hz", 2)), 10.0))
        self._jpeg_quality = max(
            20, min(int(self._config.get("jpeg_quality", 70)), 95))
        self._max_points = max(
            1000, min(int(pointcloud_config.get("max_points", 10000)), 40000))
        self._max_point_distance_m = max(
            1.0, min(float(pointcloud_config.get("max_distance_m", 8.0)), 30.0))
        mount_rotation = pointcloud_config.get("mount_rotation_deg", {})
        if not isinstance(mount_rotation, dict):
            mount_rotation = {}
        self._pointcloud_mount_rotation_deg = {
            axis: float(mount_rotation.get(axis, 0.0))
            for axis in ("x", "y", "z")
        }
        self._pointcloud_mount_rotation = self._rotation_matrix_xyz(
            *(math.radians(self._pointcloud_mount_rotation_deg[axis])
              for axis in ("x", "y", "z")))
        mount_translation = pointcloud_config.get("mount_translation_m", {})
        if not isinstance(mount_translation, dict):
            mount_translation = {}
        self._pointcloud_mount_translation_m = {
            axis: float(mount_translation.get(axis, 0.0))
            for axis in ("x", "y", "z")
        }
        self._resolution_name = str(self._config.get("resolution", "VGA")).upper()
        self._depth_mode_name = str(
            self._config.get("depth_mode", "PERFORMANCE")).upper()
        self._camera_fps = max(1, min(int(self._config.get("fps", 15)), 60))
        # The ZED Mini on Adam's head is physically mounted upside down.  Let
        # the SDK rotate the complete camera data path (RGB, depth and point
        # cloud) together so the three outputs remain pixel/geometry aligned.
        self._camera_flip = bool(self._config.get("camera_flip", True))

        self._running = False
        self._available = False
        self._camera = None
        self._capture_thread = None
        self._worker_threads = []
        self._frame_queues = {}
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self._publish_locks = {
            "rgb": threading.Lock(),
            "depth": threading.Lock(),
            "pointcloud": threading.Lock(),
        }
        self._rgb_pub = None
        self._depth_pub = None
        self._pointcloud_pub = None
        self._lock = threading.Lock()
        # A one-shot photo card shares this capture loop rather than opening a
        # second ZED SDK handle.  The condition protects the JPEG cache and
        # lets a caller wait specifically for a frame newer than its request.
        self._photo_condition = threading.Condition(self._lock)
        self._photo_waiters = 0
        self._latest_rgb = None
        self._rgb_sequence = 0
        self._state = {
            "state": "idle",
            "available": False,
            "source": "zed-sdk-local",
            "error": None,
            "pointcloud_enabled": self._pointcloud_enabled,
            "left_intrinsics": None,
            "right_intrinsics": None,
            "stereo_baseline_m": None,
            "stereo_translation_m": None,
        }

        self._pub_node = Node("adam_zed_camera")
        executor.add_node(self._pub_node)

    @staticmethod
    def _tool(name, description, topic, fmt, input_schema=None):
        return {
            "name": name,
            "type": "sensor",
            "multiInstance": False,
            "description": description,
            "inputSchema": input_schema or {"type": "object", "properties": {}},
            "topic_out": [{"topic": topic, "format": fmt}],
        }

    def get_tools(self):
        return [
            self._tool(
                "camera_head",
                "Adam ZED Mini left RGB image from the local Jetson ZED SDK",
                self._topics["camera_head"], self._FORMATS["camera_head"]),
            self._tool(
                "camera_depth",
                "Adam ZED Mini depth image, zlib-compressed little-endian uint16 millimetres",
                self._topics["camera_depth"], self._FORMATS["camera_depth"]),
            self._tool(
                "camera_pointcloud",
                "Adam ZED Mini XYZ point cloud for the Phanthymotus 3D renderer; runtime-toggleable",
                self._topics["camera_pointcloud"], self._FORMATS["camera_pointcloud"],
                {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["start", "stop", "info"],
                            "description": "Enable or disable point-cloud publishing without reopening the camera",
                        },
                    },
                }),
        ]

    def start(self):
        with self._lifecycle_lock:
            if self._running:
                if (self._capture_thread is None
                        or self._capture_thread.is_alive()):
                    return True
                # The capture thread died outside its normal error path.  Let
                # the cleanup below close any stale camera before restarting.
                self._running = False

            # A capture loop can stop unexpectedly after opening the camera.
            # Do not create a replacement thread until the old one and all of
            # its camera calls have definitely finished.
            if (self._capture_thread is not None
                    or self._worker_threads
                    or self._camera is not None):
                if not self.stop():
                    return False

            self._stop_event.clear()
            self._running = True
            try:
                from sensor_msgs.msg import CompressedImage
                from std_msgs.msg import UInt8MultiArray

                qos = _reliable_qos()
                self._CompressedImage = CompressedImage
                self._UInt8MultiArray = UInt8MultiArray
                self._rgb_pub = self._pub_node.create_publisher(
                    CompressedImage, self._topics["camera_head"], qos)
                self._depth_pub = self._pub_node.create_publisher(
                    CompressedImage, self._topics["camera_depth"], qos)
                self._pointcloud_pub = self._pub_node.create_publisher(
                    UInt8MultiArray, self._topics["camera_pointcloud"], qos)
            except Exception as exc:
                self._running = False
                self._stop_event.set()
                self._destroy_publishers()
                self._set_error(f"ROS2 camera publisher setup failed: {exc}")
                return False

            self._frame_queues = {
                "rgb": _LatestFrameQueue(),
                "depth": _LatestFrameQueue(),
                "pointcloud": _LatestFrameQueue(),
            }
            self._worker_threads = [
                threading.Thread(
                    target=self._rgb_worker,
                    daemon=True,
                    name="adam_zed_rgb_worker"),
                threading.Thread(
                    target=self._depth_worker,
                    daemon=True,
                    name="adam_zed_depth_worker"),
                threading.Thread(
                    target=self._pointcloud_worker,
                    daemon=True,
                    name="adam_zed_pointcloud_worker"),
            ]
            for worker in self._worker_threads:
                worker.start()

            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True, name="adam_zed_capture")
            self._capture_thread.start()
            return True

    def _destroy_publishers(self):
        # Destroy each publisher under its own lock so RGB publication cannot
        # wait behind a slow depth or point-cloud publication.
        for attr, lock_name in (
                ("_rgb_pub", "rgb"),
                ("_depth_pub", "depth"),
                ("_pointcloud_pub", "pointcloud")):
            with self._publish_locks[lock_name]:
                publisher = getattr(self, attr, None)
                setattr(self, attr, None)
                if publisher is None:
                    continue
                try:
                    self._pub_node.destroy_publisher(publisher)
                except Exception:
                    pass

    def stop(self):
        with self._lifecycle_lock:
            self._running = False
            self._stop_event.set()

            # Closing before join is intentional: it gives a blocking SDK
            # grab() a chance to return so that the capture thread can exit.
            camera = self._camera
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass

            thread = self._capture_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=3.0)
            workers = list(self._worker_threads)
            for worker in workers:
                if worker is not threading.current_thread():
                    worker.join(timeout=3.0)

            # Never clear the thread handles or destroy publishers while the
            # capture or processing workers can still call publish().  A later
            # start() will retry this cleanup before creating replacement
            # threads.
            capture_alive = thread is not None and thread.is_alive()
            workers_alive = any(worker.is_alive() for worker in workers)
            if capture_alive or workers_alive:
                self._available = False
                self._set_error(
                    "ZED capture or processing thread did not stop within "
                    "3 seconds; camera publishers were kept alive")
                return False

            self._capture_thread = None
            self._worker_threads = []
            self._frame_queues = {}
            self._camera = None
            self._available = False
            with self._lock:
                self._state.update({
                    "state": "idle",
                    "available": False,
                    "pointcloud_enabled": self._pointcloud_enabled,
                })
            self._destroy_publishers()
            return True

    def _set_error(self, message):
        with self._lock:
            self._available = False
            self._state.update({
                "state": "error",
                "available": False,
                "error": str(message),
            })
        print(f"[ZedCameraPlugin] {message}", flush=True)

    @staticmethod
    def _enum_name(value):
        name = getattr(value, "name", None)
        return str(name if name is not None else value)

    @staticmethod
    def _resolution_dict(resolution):
        return {
            "width": int(getattr(resolution, "width", 0)),
            "height": int(getattr(resolution, "height", 0)),
        }

    @staticmethod
    def _float_list(value):
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            return []

    def _camera_metadata(self, sdk_camera_info, params):
        configuration = sdk_camera_info.camera_configuration
        calibration = configuration.calibration_parameters
        left = calibration.left_cam
        right = calibration.right_cam
        translation = calibration.stereo_transform.get_translation().get()
        return {
            "state": "running",
            "available": True,
            "connected": True,
            "source": "zed-sdk-local",
            "resolution": self._resolution_dict(configuration.resolution),
            "fps": int(configuration.fps),
            "depth_mode": self._enum_name(params.depth_mode),
            "coordinate_units": self._enum_name(params.coordinate_units),
            "left_intrinsics": {
                "fx": float(left.fx), "fy": float(left.fy),
                "cx": float(left.cx), "cy": float(left.cy),
                "distortion": self._float_list(left.disto),
            },
            "right_intrinsics": {
                "fx": float(right.fx), "fy": float(right.fy),
                "cx": float(right.cx), "cy": float(right.cy),
                "distortion": self._float_list(right.disto),
            },
            "stereo_baseline_m": float(calibration.get_camera_baseline()),
            "stereo_translation_m": self._float_list(translation),
            "error": None,
            "pointcloud_enabled": self._pointcloud_enabled,
        }

    @staticmethod
    def _load_zed_module():
        try:
            import pyzed.sl as sl
            return sl
        except ImportError as first_error:
            # The deployment mounts the host's architecture-specific pyzed
            # extension at /opt/pyzed instead of baking a licensed SDK into
            # the driver image.
            candidate = os.environ.get("ZED_PYTHON_PATH", "/opt/pyzed")
            if candidate:
                candidate_path = Path(candidate)
                search_paths = [candidate_path]
                # When the package directory itself is mounted at
                # /opt/pyzed, Python needs its parent (/opt) on sys.path.
                if (candidate_path / "__init__.py").exists() or list(candidate_path.glob("sl*.so")):
                    search_paths.append(candidate_path.parent)
                for search_path in reversed(search_paths):
                    if search_path.exists() and str(search_path) not in sys.path:
                        sys.path.insert(0, str(search_path))
            try:
                import pyzed.sl as sl
                return sl
            except ImportError as second_error:
                raise ImportError(
                    "pyzed.sl is unavailable; mount the Jetson ZED SDK and set "
                    "ZED_PYTHON_PATH (or PYTHONPATH) accordingly"
                ) from second_error
            except Exception:
                raise first_error

    def _capture_active(self):
        return self._running and not self._stop_event.is_set()

    @staticmethod
    def _advance_deadline(deadline, period, now):
        """Advance a stream on an absolute schedule without jitter drift."""
        if deadline <= 0.0:
            return now + period
        deadline += period
        if deadline <= now:
            return now + period
        return deadline

    def _publish_capture_message(self, publisher, message, card_name=None):
        """Publish while keeping teardown from racing the ROS call."""
        if publisher is None:
            return False
        lock_name = {
            "camera_head": "rgb",
            "camera_depth": "depth",
            "camera_pointcloud": "pointcloud",
        }.get(card_name, "rgb")
        with self._publish_locks[lock_name]:
            if not self._capture_active():
                return False
            if card_name is not None:
                with self._lock:
                    if not self._card_enabled.get(card_name, False):
                        return False
            publisher.publish(message)
            return True

    def _capture_loop(self):
        try:
            self._capture_loop_body()
        except Exception as exc:
            if self._capture_active():
                self._running = False
                self._set_error(f"ZED capture loop failed: {exc}")
        finally:
            self._available = False

    def _capture_loop_body(self):
        try:
            import numpy as np
            sl = self._load_zed_module()
        except Exception as exc:
            if not self._capture_active():
                return
            self._running = False
            self._set_error(f"local ZED SDK import failed: {exc}")
            return

        if not self._capture_active():
            return

        params = sl.InitParameters()
        params.camera_resolution = getattr(
            sl.RESOLUTION, self._resolution_name, sl.RESOLUTION.VGA)
        depth_mode = getattr(sl.DEPTH_MODE, self._depth_mode_name, None)
        if depth_mode is None:
            depth_mode = getattr(sl.DEPTH_MODE, "NEURAL_LIGHT", sl.DEPTH_MODE.PERFORMANCE)
        params.depth_mode = depth_mode
        params.camera_fps = self._camera_fps
        params.coordinate_units = sl.UNIT.METER
        if self._camera_flip:
            if hasattr(params, "camera_image_flip"):
                flip_modes = getattr(sl, "FLIP_MODE", None)
                params.camera_image_flip = getattr(flip_modes, "ON", 1)
            else:
                print(
                    "[ZedCameraPlugin] camera_flip requested but this ZED SDK "
                    "does not expose camera_image_flip",
                    flush=True,
                )
        if hasattr(params, "depth_maximum_distance"):
            params.depth_maximum_distance = self._max_point_distance_m

        if not self._capture_active():
            return

        camera = sl.Camera()
        try:
            status = camera.open(params)
        except Exception as exc:
            if not self._capture_active():
                try:
                    camera.close()
                except Exception:
                    pass
                return
            self._running = False
            self._set_error(f"ZED camera open failed: {exc}")
            try:
                camera.close()
            except Exception:
                pass
            return
        if status != sl.ERROR_CODE.SUCCESS:
            if not self._capture_active():
                try:
                    camera.close()
                except Exception:
                    pass
                return
            self._running = False
            self._set_error(f"ZED camera open failed: {status}")
            try:
                camera.close()
            except Exception:
                pass
            return

        # stop() may have been called while the SDK was opening the camera.
        # Never publish or enter grab() after that stop request.
        if not self._capture_active():
            try:
                camera.close()
            except Exception:
                pass
            return
        self._camera = camera
        self._available = True
        try:
            metadata = self._camera_metadata(
                camera.get_camera_information(), params)
        except Exception as exc:
            self._available = False
            if self._capture_active():
                self._running = False
                self._set_error(f"ZED camera metadata read failed: {exc}")
            try:
                camera.close()
            except Exception:
                pass
            return
        if not self._capture_active():
            return
        with self._lock:
            self._state = metadata

        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        depth = sl.Mat()
        pointcloud = sl.Mat()
        next_rgb = 0.0
        next_depth = 0.0
        next_pointcloud = 0.0
        rgb_period = 1.0 / self._rgb_hz
        depth_period = 1.0 / self._depth_hz
        pointcloud_period = 1.0 / self._pointcloud_hz
        rgb_queue = self._frame_queues["rgb"]
        depth_queue = self._frame_queues["depth"]
        pointcloud_queue = self._frame_queues["pointcloud"]
        last_grab_error = None

        try:
            while self._capture_active():
                try:
                    status = camera.grab(runtime)
                except Exception as exc:
                    if not self._capture_active():
                        break
                    self._running = False
                    self._set_error(f"ZED grab failed: {exc}")
                    break
                if status != sl.ERROR_CODE.SUCCESS:
                    if not self._capture_active():
                        break
                    if status != last_grab_error:
                        print(f"[ZedCameraPlugin] grab status: {status}", flush=True)
                        last_grab_error = status
                    self._stop_event.wait(0.01)
                    continue
                last_grab_error = None
                if not self._capture_active():
                    break
                now = time.monotonic()

                with self._lock:
                    rgb_enabled = (
                        self._card_enabled["camera_head"]
                        or self._photo_waiters > 0)
                    depth_enabled = self._card_enabled["camera_depth"]
                    pointcloud_enabled = (
                        self._card_enabled["camera_pointcloud"]
                        and self._pointcloud_enabled)

                if rgb_enabled and now >= next_rgb and self._capture_active():
                    try:
                        camera.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU)
                        rgb_queue.put_latest(
                            np.array(image.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"RGB capture failed: {exc}")
                    next_rgb = self._advance_deadline(
                        next_rgb, rgb_period, now)

                need_depth = depth_enabled and now >= next_depth
                need_pointcloud = pointcloud_enabled and now >= next_pointcloud

                if need_depth and self._capture_active():
                    try:
                        camera.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
                        depth_queue.put_latest(
                            np.array(depth.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"depth capture failed: {exc}")
                    next_depth = self._advance_deadline(
                        next_depth, depth_period, now)

                if need_pointcloud and self._capture_active():
                    try:
                        camera.retrieve_measure(
                            pointcloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
                        pointcloud_queue.put_latest(
                            np.array(pointcloud.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"pointcloud capture failed: {exc}")
                    next_pointcloud = self._advance_deadline(
                        next_pointcloud, pointcloud_period, now)
        finally:
            self._available = False
            if self._running and not self._stop_event.is_set():
                self._running = False
                self._set_error("ZED capture loop stopped unexpectedly")

    def _rgb_worker(self):
        try:
            import numpy as np
            from PIL import Image as PillowImage
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"RGB worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["rgb"]
        while self._capture_active():
            try:
                image = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            try:
                jpeg = self._encode_jpeg(image, np, PillowImage)
                with self._photo_condition:
                    self._rgb_sequence += 1
                    self._latest_rgb = {
                        "data": jpeg,
                        "timestamp_ms": int(time.time() * 1000),
                        "sequence": self._rgb_sequence,
                    }
                    self._photo_condition.notify_all()
                    publish_rgb = self._card_enabled["camera_head"]
                if not publish_rgb:
                    continue
                msg = self._CompressedImage()
                msg.format = "jpeg"
                msg.data = jpeg
                self._publish_capture_message(
                    self._rgb_pub, msg, "camera_head")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"RGB processing failed: {exc}")

    def _depth_worker(self):
        try:
            import numpy as np
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"depth worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["depth"]
        while self._capture_active():
            try:
                depth = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            with self._lock:
                if not self._card_enabled["camera_depth"]:
                    continue
            try:
                depth_mm = self._normalize_depth(depth, np)
                msg = self._CompressedImage()
                msg.format = "16UC1; compressedDepth zlib"
                msg.data = zlib.compress(
                    depth_mm.astype("<u2", copy=False).tobytes(), level=1)
                self._publish_capture_message(
                    self._depth_pub, msg, "camera_depth")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"depth processing failed: {exc}")

    def _pointcloud_worker(self):
        try:
            import numpy as np
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"pointcloud worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["pointcloud"]
        while self._capture_active():
            try:
                pointcloud = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            with self._lock:
                if not (self._card_enabled["camera_pointcloud"]
                        and self._pointcloud_enabled):
                    continue
            try:
                payload = self._pack_pointcloud(pointcloud, np)
                if payload is None:
                    continue
                msg = self._UInt8MultiArray()
                msg.data = list(payload)
                self._publish_capture_message(
                    self._pointcloud_pub, msg, "camera_pointcloud")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"pointcloud processing failed: {exc}")

    def _encode_jpeg(self, image, np, pillow_image):
        if image.ndim == 3 and image.shape[2] >= 3:
            # ZED's default U8_C4 CPU image is BGRA.  The dashboard expects
            # ordinary RGB JPEG bytes, so drop alpha and reverse BGR->RGB.
            rgb = image[:, :, :3][:, :, ::-1]
        elif image.ndim == 2:
            rgb = np.repeat(image[:, :, None], 3, axis=2)
        else:
            raise ValueError(f"unexpected ZED image shape {image.shape}")
        output = io.BytesIO()
        pillow_image.fromarray(np.ascontiguousarray(rgb), "RGB").save(
            output, format="JPEG", quality=self._jpeg_quality, optimize=False)
        return output.getvalue()

    @staticmethod
    def _normalize_depth(depth, np):
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        valid = np.isfinite(depth) & (depth > 0.0)
        millimetres = np.zeros(depth.shape, dtype=np.uint16)
        millimetres[valid] = np.clip(
            depth[valid] * 1000.0, 0.0, 65535.0).astype(np.uint16)

        # The stock Phanthymotus depth renderer consumes a fixed 640x480
        # matrix.  Crop the ZED 16:9 image centrally, then nearest-neighbour
        # resample without introducing an OpenCV dependency.
        height, width = millimetres.shape
        if width * 3 > height * 4:
            crop_width = max(1, (height * 4) // 3)
            left = max(0, (width - crop_width) // 2)
            millimetres = millimetres[:, left:left + crop_width]
        elif width * 3 < height * 4:
            crop_height = max(1, (width * 3) // 4)
            top = max(0, (height - crop_height) // 2)
            millimetres = millimetres[top:top + crop_height, :]
        source_height, source_width = millimetres.shape
        rows = (np.arange(480) * source_height / 480).astype(np.int64)
        cols = (np.arange(640) * source_width / 640).astype(np.int64)
        return millimetres[rows[:, None], cols[None, :]]

    @staticmethod
    def _rotation_matrix_xyz(x_rad, y_rad, z_rad):
        """Return a renderer-frame rotation that applies X, then Y, then Z."""
        sx, cx = math.sin(x_rad), math.cos(x_rad)
        sy, cy = math.sin(y_rad), math.cos(y_rad)
        sz, cz = math.sin(z_rad), math.cos(z_rad)
        # Column-vector convention: Rz @ Ry @ Rx.
        return (
            (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
            (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
            (-sy, cy * sx, cy * cx),
        )

    def _pack_pointcloud(self, points, np):
        if points.ndim != 3 or points.shape[2] < 3:
            raise ValueError(f"unexpected ZED point-cloud shape {points.shape}")
        xyz = points[:, :, :3].reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        valid &= xyz[:, 2] > 0.05
        valid &= xyz[:, 2] <= self._max_point_distance_m
        xyz = xyz[valid]
        if xyz.size == 0:
            return None
        if xyz.shape[0] > self._max_points:
            stride = int(math.ceil(xyz.shape[0] / self._max_points))
            xyz = xyz[::stride][:self._max_points]

        # First express the optical ZED frame in the renderer's default frame:
        # (right, down, forward) -> (right, up, backward).  Correct the fixed
        # camera mounting angle in that frame, then invert the renderer's
        # configurable default mapping display=(packed_y,-packed_z,-packed_x).
        rotation = self._pointcloud_mount_rotation
        display = np.empty_like(xyz, dtype="<f4")
        camera_x = xyz[:, 0]
        camera_y = -xyz[:, 1]
        camera_z = -xyz[:, 2]
        display[:, 0] = (
            rotation[0][0] * camera_x
            + rotation[0][1] * camera_y
            + rotation[0][2] * camera_z)
        display[:, 1] = (
            rotation[1][0] * camera_x
            + rotation[1][1] * camera_y
            + rotation[1][2] * camera_z)
        display[:, 2] = (
            rotation[2][0] * camera_x
            + rotation[2][1] * camera_y
            + rotation[2][2] * camera_z)
        display[:, 0] += self._pointcloud_mount_translation_m["x"]
        display[:, 1] += self._pointcloud_mount_translation_m["y"]
        display[:, 2] += self._pointcloud_mount_translation_m["z"]
        packed_xyz = np.empty((xyz.shape[0], 3), dtype="<f4")
        packed_xyz[:, 0] = -display[:, 2]
        packed_xyz[:, 1] = display[:, 0]
        packed_xyz[:, 2] = -display[:, 1]
        return struct.pack("<II", 12, int(packed_xyz.shape[0])) + packed_xyz.tobytes()

    def _card_state(self, tool_name):
        with self._lock:
            enabled = self._card_enabled[tool_name]
            state = self._state.get("state", "idle")
            running = self._running

        if tool_name == "camera_pointcloud" and not enabled:
            return "disabled"
        if not enabled:
            return "idle"
        # start() launches the SDK capture loop asynchronously.  Report the
        # card as running during that short opening window; an SDK failure is
        # reported asynchronously through the shared error state.
        if state == "idle" and running:
            return "running"
        return state

    def _card_response(self, tool_name):
        response = {
            "state": self._card_state(tool_name),
            "topic_out": [{
                "topic": self._topics[tool_name],
                "format": self._FORMATS[tool_name],
            }],
        }
        if tool_name == "camera_pointcloud":
            response["pointcloud_enabled"] = self._pointcloud_enabled
        return response

    def _stop_if_no_cards_enabled(self):
        with self._lock:
            should_stop = not any(self._card_enabled.values())
        if should_stop:
            self.stop()

    def capture_photo(self, timeout_s=5.0):
        """Return a JPEG captured after this call starts.

        The ZED remains lazy while only the photo card is installed: a raw RGB
        frame is retrieved only while at least one caller is waiting here.
        """
        timeout_s = max(0.1, min(float(timeout_s), 15.0))
        deadline = time.monotonic() + timeout_s
        with self._photo_condition:
            if not self._running:
                if not self.start():
                    raise RuntimeError("Adam ZED camera could not be started")
            start_sequence = self._rgb_sequence
            self._photo_waiters += 1
            try:
                while True:
                    frame = self._latest_rgb
                    if frame is not None and frame["sequence"] > start_sequence:
                        return dict(frame)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        detail = self._state.get("error") or "no fresh RGB frame arrived"
                        raise RuntimeError(detail)
                    self._photo_condition.wait(remaining)
            finally:
                self._photo_waiters -= 1


    def dispatch(self, action, args):
        tool_name = args.get("_tool_name", action)
        if tool_name not in self._CARD_NAMES:
            return {"state": self._state.get("state", "idle")}

        if tool_name == "camera_pointcloud" and action in ("start", "enable"):
            with self._lock:
                self._card_enabled[tool_name] = True
                self._pointcloud_enabled = True
                self._state["pointcloud_enabled"] = True
            if not self._running:
                self.start()
            return self._card_response(tool_name)

        if tool_name == "camera_pointcloud" and action in ("stop", "disable"):
            with self._lock:
                self._card_enabled[tool_name] = False
                self._pointcloud_enabled = False
                self._state["pointcloud_enabled"] = False
            self._stop_if_no_cards_enabled()
            return self._card_response(tool_name)

        if action == "start":
            with self._lock:
                self._card_enabled[tool_name] = True
            if not self._running:
                self.start()
            return self._card_response(tool_name)

        if action == "stop":
            with self._lock:
                self._card_enabled[tool_name] = False
            self._stop_if_no_cards_enabled()
            return self._card_response(tool_name)

        if action in ("info", tool_name):
            return self._card_response(tool_name)
        return {"state": self._state.get("state", "idle")}


# ---------------------------------------------------------------------------
# Resource card and bundle
# ---------------------------------------------------------------------------

class VisionCapturePlugin:
    """Capture and manage Adam ZED media with the Tianyi card contract."""

    CARD = "vision_capture"

    def __init__(self, plugin_config, camera):
        self._camera = camera
        config = dict(plugin_config or {})
        self._output_dir = Path(str(config.get(
            "output_dir", "/opt/phanthy-motus/data/images"))).expanduser()
        self._channel_dir = self._derive_channel_dir(self._output_dir)
        self._timeout_s = max(1, min(int(config.get("timeout_s", 5)), 15))
        self._video_fps = max(1.0, min(30.0, float(config.get("video_fps", 15))))
        self._max_video_seconds = max(1.0, min(60.0, float(config.get("max_video_seconds", 60))))
        self._default_video_seconds = max(
            1.0, min(self._max_video_seconds, float(config.get("default_video_seconds", 5))))
        self._recording_lock = threading.Lock()
        self._recording_stop = None
        self._recording_thread = None
        self._recording_path = None
        self._recording_error = None

    @staticmethod
    def _derive_channel_dir(native_dir):
        override = os.environ.get("PHANTHY_CHANNEL_OUTPUT_DIR")
        if override:
            return str(Path(override))
        try:
            return str(Path("/work/resource") / native_dir.relative_to(
                Path("/opt/phanthy-motus/data")))
        except ValueError:
            return str(native_dir)

    @staticmethod
    def _default_stem(prefix):
        return f"{prefix}_{time.time_ns()}"

    @staticmethod
    def _file_stem(args, key):
        value = args.get(key)
        if value is None or value == "":
            return None
        value = str(value).strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,99}", value):
            raise ValueError("name must be 1-100 chars: letters, numbers, '.', '_' or '-' only")
        return value

    def get_tool(self):
        return {
            "name": self.CARD,
            "type": "actuator",
            "description": (
                "拍照、录制视频以及管理 /opt/phanthy-motus/data/images 中的媒体文件。"
                "照片 name 不含 .jpg，视频 name 不含 .mp4；拍摄成功后可使用返回的 "
                "channel_reply_path 通过消息渠道发送。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["capture_image", "record_video", "start_recording",
                                 "stop_recording", "list", "delete", "info", "start", "stop"],
                        "description": "操作类型",
                    },
                    "image_name": {"type": "string", "description": "照片文件名（不含 .jpg），该项可以不填"},
                    "video_name": {"type": "string", "description": "视频文件名（不含 .mp4），该项可以不填"},
                    "name": {"type": "string", "description": "删除时填写完整文件名，必须包含 .jpg 或 .mp4"},
                    "duration": {"type": "number", "description": "视频时长（秒），默认 5，最大 60"},
                },
                "required": ["action"],
                "x-completion": {"actions": ["record_video"], "timeout": 60},
                "x-action-params": {
                    "capture_image": {"params": ["image_name"], "description": "拍照；不填 image_name 则使用 IMG_时间戳.jpg"},
                    "record_video": {"params": ["video_name", "duration"], "description": "录制指定时长的视频；不填 video_name 则使用 VID_时间戳.mp4；duration 默认 5 秒、最大 60 秒"},
                    "start_recording": {"params": ["video_name"], "description": "开始持续录制；不填 video_name 则使用 VID_时间戳.mp4"},
                    "stop_recording": {"params": [], "description": "结束当前持续录制并保存视频"},
                    "list": {"params": [], "description": "查询已保存的照片和视频"},
                    "delete": {"params": ["name"], "description": "删除指定媒体；name 必须填写完整文件名，例如 test.jpg 或 test.mp4"},
                    "info": {"params": [], "description": "查看相机和录制状态"},
                    "start": {"params": [], "description": "启动相机"},
                    "stop": {"params": [], "description": "停止相机"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        self._stop_recording()
        return {"state": "idle"}

    def _info(self):
        with self._camera._lock:
            state = dict(self._camera._state)
        return {
            "state": "running" if state.get("available") else state.get("state", "idle"),
            "frame_available": bool(state.get("available")),
            "source": "adam-zed-sdk-local",
            "output_dir": str(self._output_dir),
            "channel_output_dir": self._channel_dir,
            "recording": bool(self._recording_thread and self._recording_thread.is_alive()),
            "error": state.get("error"),
        }

    def _capture_image(self, args):
        try:
            stem = self._file_stem(args, "image_name") or self._default_stem("IMG")
            frame = self._camera.capture_photo(self._timeout_s)
            self._output_dir.mkdir(parents=True, exist_ok=True)
            filename = f"{stem}.jpg"
            path = self._output_dir / filename
            if path.exists():
                return {"error": f"file already exists: {filename}"}
            path.write_bytes(frame["data"])
            return {
                "state": "captured", "filename": filename, "path": str(path),
                "channel_reply_path": str(Path(self._channel_dir) / filename),
                "mime": "image/jpeg", "size": path.stat().st_size,
            }
        except Exception as exc:
            return {"error": f"failed to save JPEG: {exc}"}

    def _list(self):
        if not self._output_dir.exists():
            return {"state": "listed", "files": []}
        files = sorted(
            (p for p in self._output_dir.iterdir()
             if p.is_file() and p.suffix.lower() in (".jpg", ".mp4")),
            key=lambda p: p.stat().st_mtime, reverse=True)
        return {"state": "listed", "files": [
            {"filename": p.name, "path": str(p), "size": p.stat().st_size,
             "mime": "image/jpeg" if p.suffix.lower() == ".jpg" else "video/mp4"}
            for p in files]}

    def _delete(self, args):
        filename = args.get("name")
        if not isinstance(filename, str) or not re.fullmatch(
                r"[A-Za-z0-9][A-Za-z0-9._-]{0,99}\.(?:jpg|mp4)", filename, re.IGNORECASE):
            return {"error": "name is required and must be a complete .jpg or .mp4 filename"}
        path = self._output_dir / filename
        if not path.is_file():
            return {"error": f"file not found: {filename}"}
        path.unlink()
        return {"state": "deleted", "filename": [filename]}

    def _video_result(self, path, duration=None):
        result = {
            "state": "recorded", "filename": path.name, "path": str(path),
            "channel_reply_path": str(Path(self._channel_dir) / path.name),
            "mime": "video/mp4", "size": path.stat().st_size,
        }
        if duration is not None:
            result["duration"] = duration
        return result

    def _target_frame_count(self, elapsed_s, current_count):
        """Keep encoded duration aligned with wall time if camera frames lag."""
        return max(current_count + 1, int(round(elapsed_s * self._video_fps)))

    def _record_loop(self, path, stop_event, duration=None):
        command = [
            "ffmpeg", "-loglevel", "error", "-y", "-f", "mjpeg",
            "-framerate", str(self._video_fps), "-i", "pipe:0", "-an",
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(path),
        ]
        process = None
        started = time.monotonic()
        frames_written = 0
        error = None
        try:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
            while not stop_event.is_set():
                if duration is not None and time.monotonic() - started >= duration:
                    break
                frame = self._camera.capture_photo(self._timeout_s)
                elapsed = time.monotonic() - started
                if duration is not None:
                    elapsed = min(elapsed, duration)
                # ZED frame delivery can be a little slower than the declared
                # MP4 frame rate. Duplicate the newest frame when needed so a
                # requested two-second recording remains two seconds instead
                # of being shortened by the encoder's fixed frame timestamps.
                target_count = self._target_frame_count(elapsed, frames_written)
                while frames_written < target_count:
                    process.stdin.write(frame["data"])
                    frames_written += 1
                process.stdin.flush()
            process.stdin.close()
            process.stdin = None
            _, stderr = process.communicate(timeout=10)
            if process.returncode:
                error = stderr.decode("utf-8", "replace").strip() or f"ffmpeg exited {process.returncode}"
        except Exception as exc:
            error = str(exc)
            if process is not None:
                process.kill()
                process.communicate()
        finally:
            self._recording_error = error

    def _start_recording(self, args, duration=None):
        try:
            stem = self._file_stem(args, "video_name") or self._default_stem("VID")
        except ValueError as exc:
            return {"error": str(exc)}
        self._output_dir.mkdir(parents=True, exist_ok=True)
        path = self._output_dir / f"{stem}.mp4"
        if path.exists():
            return {"error": f"file already exists: {path.name}"}
        with self._recording_lock:
            if self._recording_thread and self._recording_thread.is_alive():
                return {"error": f"recording already active: {self._recording_path.name}"}
            self._recording_stop = threading.Event()
            self._recording_path = path
            self._recording_error = None
            self._recording_thread = threading.Thread(
                target=self._record_loop, args=(path, self._recording_stop, duration),
                daemon=True, name="adam-vision-record")
            self._recording_thread.start()
        return {"state": "recording", "filename": path.name, "path": str(path),
                "channel_reply_path": str(Path(self._channel_dir) / path.name), "mime": "video/mp4"}

    def _stop_recording(self):
        with self._recording_lock:
            thread = self._recording_thread
            path = self._recording_path
            stop_event = self._recording_stop
        if not thread or not stop_event:
            return None
        stop_event.set()
        thread.join(timeout=self._timeout_s + 12)
        with self._recording_lock:
            if thread.is_alive():
                return {"error": "timed out while stopping video recording"}
            error = self._recording_error
            self._recording_thread = self._recording_stop = self._recording_path = None
        if error:
            return {"error": f"failed to save MP4: {error}"}
        if path and path.is_file() and path.stat().st_size:
            return self._video_result(path)
        return {"error": "recording produced no video frames"}

    def dispatch(self, action, args):
        args = args or {}
        if action == "capture_image":
            return self._capture_image(args)
        if action == "list":
            return self._list()
        if action == "delete":
            return self._delete(args)
        if action == "start_recording":
            return self._start_recording(args)
        if action == "stop_recording":
            return self._stop_recording() or {"state": "idle", "message": "no active recording"}
        if action == "record_video" and not args.get("_background"):
            from uuid import uuid4
            action_id = f"camera_record_video_{uuid4().hex[:8]}"
            background_args = dict(args, _background=True)
            threading.Thread(target=self._record_video_async,
                             args=(action_id, background_args), daemon=True,
                             name="adam-record-video-action").start()
            return {"state": "recording", "action_id": action_id,
                    "video_name": args.get("video_name"),
                    "duration": args.get("duration", self._default_video_seconds)}
        if action == "record_video":
            return self._record_video(args)
        if action == "info":
            return self._info()
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        return {"error": f"unknown action: {action}"}

    def _record_video(self, args):
        try:
            duration = max(1.0, min(self._max_video_seconds, float(
                args.get("duration", self._default_video_seconds))))
        except (TypeError, ValueError) as exc:
            return {"error": str(exc)}
        started = self._start_recording(args, duration)
        if started.get("state") != "recording":
            return started
        thread = self._recording_thread
        thread.join(duration + self._timeout_s + 12)
        result = self._stop_recording()
        if result and result.get("state") == "recorded":
            result["duration"] = duration
        return result or {"error": "recording did not finish"}

    def _record_video_async(self, action_id, args):
        result = self._record_video(args)
        status = "completed" if result.get("state") == "recorded" else "error"
        _notify_action_completion(action_id, status, result, self.CARD)


class ModelPlugin:
    """Returns URDF for 3D skeleton visualization on dashboard."""

    PREFIX = "model"

    # Map variant to available URDF file (repo only has lite, sp, standard)
    _VARIANT_URDF = {
        "lite": "adam_lite.urdf",
        "sp": "adam_sp.urdf",
        "pro": "adam_pro.urdf",       # adam_standard used as fallback for pro
        "standard": "adam_pro.urdf",  # adam_standard stored as adam_pro
    }

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 variant: str, **kwargs):
        self._variant = variant
        self._namespace = namespace
        # Resolve URDF file path
        urdf_name = self._VARIANT_URDF.get(variant, f"adam_{variant}.urdf")
        self._urdf_path = Path(__file__).parent / "resource" / urdf_name

    def get_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": f"Adam {self._variant} URDF model for 3D visualization",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        # Return URDF content
        if self._urdf_path.exists():
            return {"urdf": self._urdf_path.read_text()}
        # Try any available URDF as fallback
        resource_dir = Path(__file__).parent / "resource"
        urdfs = list(resource_dir.glob("adam_*.urdf"))
        if urdfs:
            return {"urdf": urdfs[0].read_text(), "note": f"Fallback URDF ({urdfs[0].name})"}
        return {"error": f"No URDF found for variant '{self._variant}'"}


# ===========================================================================
# AdamDeviceBundle — aggregates all plugins
# ===========================================================================

class AdamDeviceBundle:
    """Loads and manages all Adam plugins based on config."""

    def __init__(self, config: dict, namespace: str, executor, grpc_client,
                 dds_lowstate_sub=None, dds_handstate_sub=None,
                 dds_hand_pub=None, dds_lowcmd_pub=None,
                 dds_arm_lowstate_sub=None, ros2_enabled: bool | None = None):
        self._plugins = []
        self._tool_map = {}  # tool_name → plugin

        variant = config.get("variant", "sp")
        plugins_cfg = config.get("plugins", {})
        self._ros2_enabled = bool(
            HAS_ROS2
            and executor is not None
            and (ros2_enabled is None or ros2_enabled)
        )
        hand_enabled = plugins_cfg.get("hand", {}).get("enabled", True)
        hand_state_enabled = plugins_cfg.get("hand_state", {}).get("enabled", True)
        self._hand_state_cache = (
            HandStateCache(dds_handstate_sub)
            if hand_enabled or hand_state_enabled else None
        )

        # StatePlugin
        if plugins_cfg.get("state", {}).get("enabled", True) and self._ros2_enabled:
            p = StatePlugin(
                plugins_cfg.get("state", {}), namespace, executor,
                variant=variant,
                dds_lowstate_sub=dds_lowstate_sub,
            )
            self._plugins.append(p)

        # Physical emergency-stop sensor. This is intentionally read-only and
        # observes PAC actuator/RCU power; the software FSM remains STOP both
        # before and after Adam's physical emergency-stop button is pressed.
        if plugins_cfg.get("estop", {}).get("enabled", True):
            p = EStopPlugin(
                plugins_cfg.get("estop", {}), namespace, executor,
                grpc_client=grpc_client,
            )
            self._plugins.append(p)

        # The default LocoPlugin preserves the historic dashboard contract;
        # the RL variant exposes the full pnd.robot gRPC API.
        loco_cfg = plugins_cfg.get("loco", {})
        if loco_cfg.get("enabled", True):
            p = RlLocoPlugin(
                loco_cfg, namespace, executor,
                grpc_client=grpc_client,
            )
            self._plugins.append(p)

        # RL execution is also exposed as focused cards.  ``loco`` remains
        # available as a backwards-compatible aggregate card, while these
        # cards make state transitions, motions, control ownership and safety
        # actions independently discoverable to an agent.
        rl_cards = (("motion", MotionPlugin),
                    ("tracking_motion", TrackingMotionPlugin))
        for card_name, card_class in rl_cards:
            card_cfg = plugins_cfg.get(card_name, {})
            if card_cfg.get("enabled", False):
                self._plugins.append(card_class(
                    card_cfg, namespace, executor, grpc_client=grpc_client,
                ))

        # CameraPlugin
        camera_plugin = None
        if plugins_cfg.get("camera", {}).get("enabled", False) and self._ros2_enabled:
            camera_plugin = ZedCameraPlugin(
                plugins_cfg.get("camera", {}), namespace, executor)
            self._plugins.append(camera_plugin)

        # Capture is intentionally a separate card, while sharing the single
        # locally attached ZED SDK instance owned by ``camera_plugin``.
        if (plugins_cfg.get("vision_capture", {}).get("enabled", False)
                and camera_plugin is not None):
            self._plugins.append(VisionCapturePlugin(
                plugins_cfg.get("vision_capture", {}), camera_plugin))

        # HandPlugin and the read-only hand-state sensor share one DDS cache.
        if hand_enabled:
            p = HandPlugin(plugins_cfg.get("hand", {}), namespace, executor,
                           dds_hand_pub=dds_hand_pub,
                           state_cache=self._hand_state_cache)
            self._plugins.append(p)
            if plugins_cfg.get("hand_gesture", {}).get("enabled", True):
                self._plugins.append(HandGesturePlugin(p))

        # Direct upper-body control is DDS-only and intentionally remains
        # available when ROS2 is absent or isolated on the Jetson.
        if plugins_cfg.get("arm", {}).get("enabled", True):
            p = ArmControlPlugin(
                plugins_cfg.get("arm", {}), namespace, executor,
                grpc_client=grpc_client,
                dds_lowcmd_pub=dds_lowcmd_pub,
                dds_arm_lowstate_sub=dds_arm_lowstate_sub,
                variant=variant,
            )
            self._plugins.append(p)
            if plugins_cfg.get("waist", {}).get("enabled", True):
                self._plugins.append(WaistControlPlugin(p))
            if plugins_cfg.get("head", {}).get("enabled", True):
                self._plugins.append(HeadControlPlugin(p))
        if hand_state_enabled and self._hand_state_cache is not None and self._ros2_enabled:
            p = HandStatePlugin(
                plugins_cfg.get("hand_state", {}), namespace, executor,
                state_cache=self._hand_state_cache)
            self._plugins.append(p)

        # ModelPlugin
        if plugins_cfg.get("model", {}).get("enabled", True):
            p = ModelPlugin(
                plugins_cfg.get("model", {}), namespace, executor,
                variant=variant,
            )
            self._plugins.append(p)

        # Build tool map
        for plugin in self._plugins:
            if hasattr(plugin, "get_tools"):
                for tool in plugin.get_tools():
                    self._tool_map[tool["name"]] = plugin
            elif hasattr(plugin, "get_tool"):
                tool = plugin.get_tool()
                self._tool_map[tool["name"]] = plugin

    def start_all(self):
        if self._hand_state_cache is not None:
            self._hand_state_cache.start()
        for p in self._plugins:
            p.start()

    def stop_all(self):
        for p in reversed(self._plugins):
            try:
                p.stop()
            except Exception as exc:
                print(f"[adam] WARNING: plugin stop failed: {exc}", flush=True)
        if self._hand_state_cache is not None:
            self._hand_state_cache.stop()

    def close_all(self):
        self.stop_all()
        for p in reversed(self._plugins):
            close = getattr(p, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    print(f"[adam] WARNING: plugin close failed: {exc}", flush=True)
        if self._hand_state_cache is not None:
            self._hand_state_cache.close()

    def get_all_tools(self) -> list:
        tools = []
        for plugin in self._plugins:
            if hasattr(plugin, "get_tools"):
                tools.extend(plugin.get_tools())
            elif hasattr(plugin, "get_tool"):
                tools.append(plugin.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict:
        plugin = self._tool_map.get(tool_name)
        if plugin is None:
            return {"error": f"Unknown tool: {tool_name}"}
        action = args.pop("action", tool_name)
        args["_tool_name"] = tool_name
        result = plugin.dispatch(action, args)
        # A plugin that only knows its own verbs declines these rather than
        # failing at them — see common/lifecycle.py. Checked before the None is
        # turned into an error below, since returning nothing is also a decline.
        if action in _lifecycle.LIFECYCLE_ACTIONS and _lifecycle.is_declined(result):
            return _lifecycle.reply(action)
        if result is None:
            return {"error": f"Unknown action '{action}' for tool '{tool_name}'"}
        return result
