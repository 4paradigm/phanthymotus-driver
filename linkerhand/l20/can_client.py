"""LinkerHand L20 USB-CAN transport client.

The LinkerHand L20 uses a USB-to-CAN adapter (socketcan on Linux, pcan/candle on Windows)
at 1 Mbit/s. It features 16 active joints and 5 passive joints (21 joints total, 16 DOF).

CAN IDs (LinkerHand standard):
  Right hand: 0x28 (default) or 0x27
  Left hand:  0x27 (default) or 0x28
  Both IDs are fully configurable via config.yaml.

CAN Protocol Frame Mapping:
  Frame 0x01 / 0x11: Finger base flexion (pitch) - 5 fingers: thumb, index, middle, ring, pinky
  Frame 0x02 / 0x12: Finger yaw (abduction/adduction) - 5 fingers: thumb, index, middle, ring, pinky
  Frame 0x03 / 0x13: Thumb roll (rotation to palm) - thumb roll + 4 reserved bytes
  Frame 0x04 / 0x14: Finger tip flexion (tip/dip) - 5 fingers: thumb, index, middle, ring, pinky
  Frame 0x05 / 0x15: Joint speed control (5 bytes)
  Frame 0x06 / 0x16: Motor current control / feedback
  Frame 0x07 / 0x17: Motor fault status / clear faults (send [1, 1, 1, 1, 1])
  Frame 0x09: Request all joint state
  Frame 0xC0: Device UID
  Frame 0xC1: Hardware version
  Frame 0xC2: Software version

Tactile / Pressure Sensor Protocol:
  Frame 0x20: Normal force (5 fingers)
  Frame 0x21: Tangential force (5 fingers)
  Frame 0x22: Tangential force direction (5 fingers)
  Frame 0x23: Approach increment (5 fingers)
  Frame 0xB0: Touch sensor type query
  Frame 0xB1-0xB5: Fingertip pressure (0xb1: thumb, 0xb2: index, 0xb3: middle, 0xb4: ring, 0xb5: pinky)
                   Empty request -> returns [type, scalar_pressure] (0-255)
                   Request [0xC6] -> returns matrix rows (12 rows x 6 taxels)
"""

from __future__ import annotations

import logging
import errno
from functools import wraps
import math
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

logger = logging.getLogger(__name__)

ACTIVE_JOINT_COUNT = 16
VECTOR_JOINT_COUNT = 20
FINGER_COUNT = 5

LEFT_HAND_CAN_ID = 0x28
RIGHT_HAND_CAN_ID = 0x27

# CAN Frame properties / types
# G20 / L20-10 serial frame properties (per-finger, 6 bytes payload)
FT_THUMB_POS   = 0x41  # [thumb_roll, thumb_yaw, thumb_pitch, res, res, thumb_tip]
FT_INDEX_POS   = 0x42  # [index_yaw, res, index_pitch, res, res, index_tip]
FT_MIDDLE_POS  = 0x43  # [middle_yaw, res, middle_pitch, res, res, middle_tip]
FT_RING_POS    = 0x44  # [ring_yaw, res, ring_pitch, res, res, ring_tip]
FT_LITTLE_POS  = 0x45  # [pinky_yaw, res, pinky_pitch, res, res, pinky_tip]

FT_THUMB_SPEED  = 0x49
FT_INDEX_SPEED  = 0x4A
FT_MIDDLE_SPEED = 0x4B
FT_RING_SPEED   = 0x4C
FT_LITTLE_SPEED = 0x4D

FT_THUMB_TORQUE  = 0x51
FT_INDEX_TORQUE  = 0x52
FT_MIDDLE_TORQUE = 0x53
FT_RING_TORQUE   = 0x54
FT_LITTLE_TORQUE = 0x55

# Legacy L20 parallel frame properties (retained for backward compatibility)
# NR = No Return (fast write), R = Return feedback
FT_INVALID     = 0x00
FT_PITCH_R     = 0x01   # Finger base pitch (flexion) - 5 fingers
FT_YAW_R       = 0x02   # Finger yaw - 5 fingers
FT_ROLL_R      = 0x03   # Thumb roll - [thumb_roll, 0, 0, 0, 0]
FT_TIP_R       = 0x04   # Fingertip flexion - 5 fingers
FT_SPEED_R     = 0x05   # Motor speed
FT_CURRENT_R   = 0x06   # Motor current
FT_FAULT_R     = 0x07   # Motor fault

FT_REQUEST_ALL = 0x09   # Request all state

FT_PITCH_NR    = 0x11   # Fast write pitch (or ROLL_TORQUE on G20)
FT_YAW_NR      = 0x12   # Fast write yaw (or YAW_TORQUE on G20)
FT_ROLL_NR     = 0x13   # Fast write roll (or ROOT1_TORQUE on G20)
FT_TIP_NR      = 0x14   # Fast write tip (or ROOT2_TORQUE on G20)
FT_SPEED_NR    = 0x15   # Fast write speed (or ROOT3_TORQUE on G20)
FT_CURRENT_NR  = 0x16   # Fast write current (or TIP_TORQUE on G20)
FT_FAULT_NR    = 0x17   # Fast clear faults

# Tactile frames
FT_NORMAL_FORCE   = 0x20
FT_TANGENTIAL     = 0x21
FT_TANGENTIAL_DIR = 0x22
FT_APPROACH       = 0x23

FT_TOUCH_TYPE  = 0xB0
FT_TOUCH_B1    = 0xB1   # Thumb tactile
FT_TOUCH_B2    = 0xB2   # Index tactile
FT_TOUCH_B3    = 0xB3   # Middle tactile
FT_TOUCH_B4    = 0xB4   # Ring tactile
FT_TOUCH_B5    = 0xB5   # Pinky tactile

FT_MATRIX_REQ  = 0xC6   # Payload to request matrix tactile rows

# System & metadata
FT_UID         = 0xC0
FT_HW_VER      = 0xC1
FT_SW_VER      = 0xC2
FT_COMM_ID     = 0xC3
FT_SAVE_PARAM  = 0xCF

# 16 Active Joint Names in canonical order
JOINT_NAMES_16 = [
    # Thumb (4 active joints)
    "thumb_pitch", "thumb_yaw", "thumb_roll", "thumb_tip",
    # Index (3 active joints)
    "index_pitch", "index_yaw", "index_tip",
    # Middle (3 active joints)
    "middle_pitch", "middle_yaw", "middle_tip",
    # Ring (3 active joints)
    "ring_pitch", "ring_yaw", "ring_tip",
    # Pinky (3 active joints)
    "pinky_pitch", "pinky_yaw", "pinky_tip",
]

CHINESE_JOINT_NAMES_16 = [
    "大拇指指根弯曲", "大拇指侧摆", "大拇指旋转", "大拇指指尖弯曲",
    "食指指根弯曲", "食指侧摆", "食指指尖弯曲",
    "中指指根弯曲", "中指侧摆", "中指指尖弯曲",
    "无名指指根弯曲", "无名指侧摆", "无名指指尖弯曲",
    "小拇指指根弯曲", "小拇指侧摆", "小拇指指尖弯曲",
]

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

MATRIX_OFFSET_MAP = {i * 16: i for i in range(12)}


def vector20_to_named16(p: list[int]) -> dict[str, int]:
    """Convert a 20-element SDK vector to a dict of 16 active joints."""
    if len(p) != 20:
        raise ValueError(f"Expected 20-element vector, got {len(p)}")
    return {
        "thumb_pitch": p[0],
        "thumb_yaw": p[5],
        "thumb_roll": p[10],
        "thumb_tip": p[15],

        "index_pitch": p[1],
        "index_yaw": p[6],
        "index_tip": p[16],

        "middle_pitch": p[2],
        "middle_yaw": p[7],
        "middle_tip": p[17],

        "ring_pitch": p[3],
        "ring_yaw": p[8],
        "ring_tip": p[18],

        "pinky_pitch": p[4],
        "pinky_yaw": p[9],
        "pinky_tip": p[19],
    }


def named16_to_vector20(d: dict[str, int], default_val: int = 0) -> list[int]:
    """Convert a 16 active joint dict to a 20-element SDK vector."""
    return [
        d.get("thumb_pitch", default_val),
        d.get("index_pitch", default_val),
        d.get("middle_pitch", default_val),
        d.get("ring_pitch", default_val),
        d.get("pinky_pitch", default_val),

        d.get("thumb_yaw", default_val),
        d.get("index_yaw", default_val),
        d.get("middle_yaw", default_val),
        d.get("ring_yaw", default_val),
        d.get("pinky_yaw", default_val),

        d.get("thumb_roll", default_val),
        0, 0, 0, 0,  # 4 reserved slots in the roll block

        d.get("thumb_tip", default_val),
        d.get("index_tip", default_val),
        d.get("middle_tip", default_val),
        d.get("ring_tip", default_val),
        d.get("pinky_tip", default_val),
    ]


def validate_position_vector(values: Union[list, dict], name: str = "positions") -> list[int]:
    """Validate and convert input positions into a canonical 20-element integer vector (0..255)."""
    if isinstance(values, dict):
        # Validate values in dict
        for k, v in values.items():
            if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255:
                raise ValueError(f"{name}: joint '{k}' value must be an integer in 0..255, got {v}")
        return named16_to_vector20(values)

    if isinstance(values, (list, tuple)):
        lst = list(values)
        if len(lst) == 16:
            # Assume 16 active joints in canonical JOINT_NAMES_16 order
            for v in lst:
                if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255:
                    raise ValueError(f"{name}: all 16 values must be integers in 0..255, got {v}")
            named = dict(zip(JOINT_NAMES_16, lst))
            return named16_to_vector20(named)
        elif len(lst) == 20:
            for v in lst:
                if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255:
                    raise ValueError(f"{name}: all 20 values must be integers in 0..255, got {v}")
            return lst
        else:
            raise ValueError(f"{name}: expected 16 or 20 values, got {len(lst)}")

    raise ValueError(f"{name}: expected list, tuple, or dict of joint positions")


class TactileQueryTimeout(TimeoutError):
    """A bounded telemetry failure with the stage and missing feedback attached."""

    code = 'tactile_query_timeout'

    def __init__(self, message: str, details: dict):
        super().__init__(message)
        self.details = details


class PositionQueryTimeout(TimeoutError):
    """Position feedback timeout with a snapshot of the failed exchange."""

    code = 'position_query_timeout'

    def __init__(self, message: str, details: dict):
        super().__init__(message)
        self.details = details


def _command_batch(method):
    """Keep one multi-frame command contiguous with respect to telemetry."""
    @wraps(method)
    def send_batch(self, *args, **kwargs):
        with self._command_lock:
            return method(self, *args, **kwargs)
    return send_batch


class _NonblockingSocketReader:
    """Let python-can decode ready frames without changing socket defaults."""

    def __init__(self, sock):
        self._socket = sock

    def recvmsg(self, bufsize, ancbufsize=0, flags=0):
        return self._socket.recvmsg(bufsize, ancbufsize, flags | socket.MSG_DONTWAIT)


class L20CanClient:
    """Thread-safe USB-CAN client for the LinkerHand L20 dexterous hand."""

    def __init__(
        self,
        can_channel: str = "can0",
        side: str = "right",
        *,
        baudrate: int = 1_000_000,
        can_id: Optional[int] = None,
        recv_timeout_s: float = 1.0,
        bus_factory=None,
        legacy_l20_mode: bool = False,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        qemu_compat = os.environ.get("LINKERHAND_L20_QEMU_COMPAT", "0")
        if qemu_compat not in ("0", "1"):
            raise ValueError("LINKERHAND_L20_QEMU_COMPAT must be 0 or 1")
        self._qemu_compat = qemu_compat == "1"
        self.can_channel = can_channel
        self.side = side
        self.baudrate = baudrate
        self._recv_timeout_s = recv_timeout_s
        self.legacy_l20_mode = legacy_l20_mode

        if can_id is not None:
            self.can_id = can_id
        else:
            self.can_id = RIGHT_HAND_CAN_ID if side == "right" else LEFT_HAND_CAN_ID

        self._lock = threading.RLock()
        self._command_lock = threading.RLock()
        self._position_query_lock = threading.Lock()
        self._position_condition = threading.Condition(self._lock)
        self._position_updates: dict[int, int] = {}
        self._position_query_context = threading.local()
        self._tactile_query_lock = threading.Lock()
        self._tactile_query_condition = threading.Condition()
        self._tactile_query_waiters: list[object] = []
        self._tactile_updates: dict[int, int] = {}
        self._tactile_times: dict[int, float] = {}
        self._matrix_updates: dict[tuple[int, int], int] = {}

        # Joint position buffers (0x01: pitch, 0x02: yaw, 0x03: roll, 0x04: tip)
        self._pos_pitch: list[int] = [-1] * 5
        self._pos_yaw: list[int]   = [-1] * 5
        self._pos_roll: list[int]  = [-1, 0, 0, 0, 0]
        self._pos_tip: list[int]   = [-1] * 5

        self._speeds: list[int]    = [255] * 5
        self._currents: list[int]  = [-1] * 5
        self._faults: list[int]    = [0] * 5

        # Tactile sensor data
        self._touch_type: int = -1
        self._touch: list[int] = [-1] * FINGER_COUNT          # Scalar fingertip pressure (0-255)
        self._normal_force: list[float] = [-1.0] * FINGER_COUNT
        self._tangential_force: list[float] = [-1.0] * FINGER_COUNT
        self._tangential_dir: list[float] = [-1.0] * FINGER_COUNT
        self._approach_inc: list[float] = [-1.0] * FINGER_COUNT

        # 12x6 tactile matrix per finger: index 0: thumb ... 4: pinky
        self._matrix_touch: list[list[list[int]]] = [
            [[-1] * 6 for _ in range(12)] for _ in range(FINGER_COUNT)
        ]

        # Device info
        self._uid: list[int] | None = None
        self._hw_version: list[int] | None = None
        self._sw_version: list[int] | None = None

        # Open CAN interface
        if bus_factory is not None:
            self._bus = bus_factory(can_channel, baudrate, self.can_id)
        else:
            try:
                self._bus = self._open_bus(can_channel, baudrate)
            except Exception as e:
                logger.warning("Failed to open CAN bus at init (%s), will retry on demand: %s", can_channel, e)
                self._bus = None

        # Start receive loop
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name=f"l20-can-recv-{side}-{hex(self.can_id)}", daemon=True,
        )
        self._recv_thread.start()

        # Request device info and initial status
        try:
            self.request_device_info()
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────
    # CAN Bus Setup & Helpers
    # ──────────────────────────────────────────────────────────────────

    @staticmethod
    def _patch_socketcan_for_qemu() -> None:
        """Install the optional development compatibility hooks only on request."""
        from qemu_compat import patch_socketcan_for_qemu
        patch_socketcan_for_qemu()

    def _open_bus(self, channel: str, baudrate: int):
        import can
        from can.exceptions import CanError

        try:
            if sys.platform == "linux":
                if self._qemu_compat:
                    self._patch_socketcan_for_qemu()
                self._bring_up_socketcan(channel, baudrate)
                bus = can.interface.Bus(
                    channel=channel, interface="socketcan", bitrate=baudrate,
                    can_filters=[{"can_id": self.can_id, "can_mask": 0x7FF}],
                )
                return bus
            elif sys.platform == "win32":
                try:
                    bus = can.interface.Bus(
                        channel=channel, interface="pcan", bitrate=baudrate,
                        can_filters=[{"can_id": self.can_id, "can_mask": 0x7FF}],
                    )
                    return bus
                except CanError:
                    bus = can.Bus(
                        interface="candle", channel=channel, bitrate=baudrate,
                        can_filters=[{"can_id": self.can_id, "can_mask": 0x7FF}],
                    )
                    return bus
            else:
                raise EnvironmentError(f"Unsupported platform: {sys.platform}")
        except Exception as exc:
            raise RuntimeError(
                f"Cannot open CAN interface '{channel}' for {self.side} hand (ID {hex(self.can_id)}): {exc}"
            ) from exc

    @staticmethod
    def _bring_up_socketcan(channel: str, baudrate: int) -> None:
        """Enable CAN without forking from a multithreaded QEMU process.

        A child stuck before exec can block subprocess.Popen itself; a run()
        timeout cannot bound that startup wait. Read sysfs for the fast path,
        then use a bounded RTM_NEWLINK request, like ip link, only when down.
        Layouts follow Linux uapi rtnetlink.h, if_link.h and can/netlink.h.
        """
        entry = Path('/sys/class/net') / channel
        if int((entry / 'type').read_text().strip()) != 280:  # ARPHRD_CAN
            raise OSError(errno.EPROTOTYPE, 'Interface is not a CAN device', channel)
        if int((entry / 'flags').read_text().strip(), 16) & 1:  # IFF_UP
            return
        index = int((entry / 'ifindex').read_text().strip())

        def attribute(kind, payload):
            length = 4 + len(payload)
            return struct.pack('=HH', length, kind) + payload + bytes((-length) % 4)

        # IFLA_CAN_BITTIMING=1: struct can_bittiming contains eight u32s.
        timing = attribute(1, struct.pack('=8I', baudrate, 0, 0, 0, 0, 0, 0, 0))
        # IFLA_LINKINFO=18, IFLA_INFO_KIND=1, IFLA_INFO_DATA=2.
        linkinfo = attribute(18 | 0x8000,
                             attribute(1, b'can\0') + attribute(2 | 0x8000, timing))
        info = struct.pack('=BxHiII', socket.AF_UNSPEC, 0, index, 1, 1)
        payload = info + linkinfo
        # RTM_NEWLINK=16; NLM_F_REQUEST | NLM_F_ACK=5. One request per socket.
        request = struct.pack('=IHHII', 16 + len(payload), 16, 5, 1, 0) + payload
        with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as route:
            route.settimeout(1.0)
            route.bind((0, 0))
            route.sendto(request, (0, 0))
            deadline = time.monotonic() + 1.0
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Timed out enabling CAN interface ' + channel)
                route.settimeout(remaining)
                packet = route.recv(65535)
                offset = 0
                while offset < len(packet):
                    if len(packet) - offset < 16:
                        raise OSError(errno.EPROTO, 'Truncated CAN netlink header')
                    length, kind, _, sequence, _ = struct.unpack_from('=IHHII', packet, offset)
                    if length < 16 or offset + length > len(packet):
                        raise OSError(errno.EPROTO, 'Invalid CAN netlink message length')
                    if kind == 2 and sequence == 1:  # NLMSG_ERROR, including success ACK=0
                        if length < 20:
                            raise OSError(errno.EPROTO, 'Truncated CAN netlink acknowledgement')
                        error = struct.unpack_from('=i', packet, offset + 16)[0]
                        if error:
                            # Two hands may race to enable the same adapter.
                            if error == -errno.EBUSY and int((entry / 'flags').read_text().strip(), 16) & 1:
                                return
                            raise OSError(abs(error), os.strerror(abs(error)), channel)
                        return
                    offset += (length + 3) & ~3

    def send_command(self, frame_type: int, data: list[int], sleep_s: float = 0.002) -> None:
        """Send a single CAN frame to the hand."""
        import can
        payload = bytes([frame_type] + [int(v) for v in data])
        msg = can.Message(arbitration_id=self.can_id, data=payload, is_extended_id=False)
        with self._command_lock, self._lock:
            try:
                if self._bus is None:
                    self._bus = self._open_bus(self.can_channel, self.baudrate)
                self._bus.send(msg)
            except (can.CanError, AttributeError) as exc:
                try:
                    self._bus = self._open_bus(self.can_channel, self.baudrate)
                    self._bus.send(msg)
                except Exception:
                    raise RuntimeError(f"CAN send failed ({self.side}, ID={hex(self.can_id)}): {exc}") from exc
        time.sleep(sleep_s)

    # ──────────────────────────────────────────────────────────────────
    # Motion Control API (16 Active Joints, 20-Vector Format)
    # ──────────────────────────────────────────────────────────────────

    @_command_batch
    def set_joint_positions(self, position: Union[list[int], dict[str, int]]) -> list[int]:
        """Command all 16 active joints.

        Accepts either a 20-element vector, a 16-element list, or a dict of joint names.
        Returns the 20-element vector sent.
        For L20-10 / G20 (default), sends serial per-finger frames (0x41..0x45).
        For legacy L20, sends parallel frames (0x13, 0x14, 0x11, 0x12).
        """
        vec20 = validate_position_vector(position)

        if self.legacy_l20_mode:
            finger_base = vec20[0:5]      # 5 fingers pitch
            yaw_angles  = vec20[5:10]     # 5 fingers yaw
            thumb_roll  = vec20[10:15]    # thumb roll [val, 0, 0, 0, 0]
            finger_tip  = vec20[15:20]    # 5 fingers tip flexion
            self.send_command(FT_ROLL_NR, thumb_roll)
            self.send_command(FT_TIP_NR, finger_tip)
            self.send_command(FT_PITCH_NR, finger_base)
            self.send_command(FT_YAW_NR, yaw_angles)
        else:
            # G20 / L20-10 Serial protocol
            # Mapping:
            # thumb:  [10, 5, 0, 11, 12, 15] -> [roll, yaw, root, 0, 0, tip]
            # index:  [6, 11, 1, 13, 14, 16] -> [yaw, 0, root, 0, 0, tip]
            # middle: [7, 12, 2, 13, 14, 17] -> [yaw, 0, root, 0, 0, tip]
            # ring:   [8, 13, 3, 14, 15, 18] -> [yaw, 0, root, 0, 0, tip]
            # little: [9, 14, 4, 15, 16, 19] -> [yaw, 0, root, 0, 0, tip]
            thumb_data  = [vec20[10], vec20[5],  vec20[0], vec20[11], vec20[12], vec20[15]]
            index_data  = [vec20[6],  vec20[11], vec20[1], vec20[13], vec20[14], vec20[16]]
            middle_data = [vec20[7],  vec20[12], vec20[2], vec20[13], vec20[14], vec20[17]]
            ring_data   = [vec20[8],  vec20[13], vec20[3], vec20[14], vec20[15], vec20[18]]
            little_data = [vec20[9],  vec20[14], vec20[4], vec20[15], vec20[16], vec20[19]]

            self.send_command(FT_THUMB_POS,  thumb_data,  sleep_s=0.003)
            self.send_command(FT_INDEX_POS,  index_data,  sleep_s=0.003)
            self.send_command(FT_MIDDLE_POS, middle_data, sleep_s=0.003)
            self.send_command(FT_RING_POS,   ring_data,   sleep_s=0.003)
            self.send_command(FT_LITTLE_POS, little_data, sleep_s=0.003)

        return vec20

    def set_positions(self, position: Union[list[int], dict[str, int]]) -> list[int]:
        """Alias for set_joint_positions."""
        return self.set_joint_positions(position)

    @_command_batch
    def set_speeds(self, speeds: list[int]) -> list[int]:
        """Set joint motion speeds (1..255)."""
        if len(speeds) < 5:
            speeds = (list(speeds) + [speeds[-1]] * 5)[:5]
        speeds = [min(255, max(1, int(v))) for v in speeds[:5]]
        if self.legacy_l20_mode:
            self.send_command(FT_SPEED_NR, speeds)
        else:
            for prop, val in zip([FT_THUMB_SPEED, FT_INDEX_SPEED, FT_MIDDLE_SPEED, FT_RING_SPEED, FT_LITTLE_SPEED], speeds):
                self.send_command(prop, [val] * 6, sleep_s=0.003)
        self._speeds = speeds
        return speeds

    @_command_batch
    def set_currents(self, currents: list[int]) -> list[int]:
        """Set motor current / torque thresholds (0..255)."""
        if len(currents) < 5:
            currents = (list(currents) + [currents[-1]] * 5)[:5]
        currents = [min(255, max(0, int(v))) for v in currents[:5]]
        if self.legacy_l20_mode:
            self.send_command(FT_CURRENT_NR, currents)
        else:
            for prop, val in zip([FT_THUMB_TORQUE, FT_INDEX_TORQUE, FT_MIDDLE_TORQUE, FT_RING_TORQUE, FT_LITTLE_TORQUE], currents):
                self.send_command(prop, [val] * 6, sleep_s=0.003)
            self.send_command(0x82, currents, sleep_s=0.003)
        self._currents = currents
        return currents

    @_command_batch
    def set_joint_torque_limits(self, limits: Union[list[int], dict[str, int]]) -> list[int]:
        """Command individual G20/L20-10 joint torque limits, raw 0..255.

        These are commanded limits, not measured current or fingertip force.
        Do not follow with 0x82: its per-finger values overwrite independent
        joint limits. Existing set_currents remains the whole-finger API.
        """
        if self.legacy_l20_mode:
            raise NotImplementedError('独立关节出力调节需要 L20-10 / G20 串联协议')
        values = validate_position_vector(limits, 'joint_torque_limits')
        self.send_command(FT_THUMB_TORQUE,
                          [values[10], values[5], values[0], 0, 0, values[15]], sleep_s=0.003)
        for finger, prop in enumerate((FT_INDEX_TORQUE, FT_MIDDLE_TORQUE, FT_RING_TORQUE, FT_LITTLE_TORQUE), 1):
            self.send_command(prop, [values[5 + finger], 0, values[finger], 0, 0, values[15 + finger]], sleep_s=0.003)
        return values

    def clear_faults(self) -> None:
        """Clear motor faults by writing 1 to all fault registers."""
        if self.legacy_l20_mode:
            self.send_command(FT_FAULT_NR, [1, 1, 1, 1, 1])
        else:
            self.send_command(0x83, [1, 1, 1, 1, 1])

    # ──────────────────────────────────────────────────────────────────
    # State Query API
    # ──────────────────────────────────────────────────────────────────

    def request_positions(self) -> None:
        """Trigger async feedback of position frames."""
        frames = (
            (FT_PITCH_R, FT_YAW_R, FT_ROLL_R, FT_TIP_R)
            if self.legacy_l20_mode else
            (FT_THUMB_POS, FT_INDEX_POS, FT_MIDDLE_POS, FT_RING_POS, FT_LITTLE_POS)
        )
        context = getattr(self._position_query_context, 'current', None)
        lock_started = time.monotonic()
        # Same batch lock as the decorator, made explicit to measure its wait.
        with self._command_lock:
            send_started = time.monotonic()
            if context is not None:
                context['command_lock_wait_s'] = send_started - lock_started
                context['query_phase'] = 'send_request'
            try:
                for prop in frames:
                    frame_started = time.monotonic()
                    if context is not None:
                        context['in_flight_frame'] = prop
                    try:
                        self.send_command(prop, [], sleep_s=0.003)
                    finally:
                        if context is not None:
                            context['request_elapsed_s'][prop] = time.monotonic() - frame_started
                    if context is not None:
                        context['sent_frames'].append(prop)
                        context['in_flight_frame'] = None
            finally:
                if context is not None:
                    context['send_elapsed_s'] = time.monotonic() - send_started

    def get_positions_vector(self) -> list[int]:
        """Read fresh feedback for every finger; never return old cached positions."""
        return self._query_positions(self._recv_timeout_s)

    def _query_positions(self, timeout_s: float) -> list[int]:
        started = time.monotonic()
        frames = (
            (FT_PITCH_R, FT_YAW_R, FT_ROLL_R, FT_TIP_R)
            if self.legacy_l20_mode else
            (FT_THUMB_POS, FT_INDEX_POS, FT_MIDDLE_POS, FT_RING_POS, FT_LITTLE_POS)
        )
        # Keep overlapping readers from treating one another's replies as their own.
        with self._position_query_lock:
            context = {
                'started': started, 'query_lock_wait_s': time.monotonic() - started,
                'command_lock_wait_s': 0.0, 'send_elapsed_s': 0.0,
                'receive_started': None, 'query_phase': 'send_request',
                'sent_frames': [], 'in_flight_frame': None, 'request_elapsed_s': {},
            }
            with self._position_condition:
                before = {ft: self._position_updates.get(ft, 0) for ft in frames}
            # A direct request_positions() in another thread must not write to
            # this exchange's diagnostics. Clear the context on every exit.
            self._position_query_context.current = context
            try:
                self.request_positions()
            except TimeoutError as exc:
                raise self._position_timeout(frames, before, context, timeout_s, exc) from exc
            finally:
                del self._position_query_context.current
            context['query_phase'] = 'receive_reply'
            context['receive_started'] = time.monotonic()
            with self._position_condition:
                fresh = lambda: all(self._position_updates.get(ft, 0) > before[ft] for ft in frames)
                if not self._position_condition.wait_for(fresh, timeout=timeout_s):
                    raise self._position_timeout(frames, before, context, timeout_s)
                vec = list(self._pos_pitch + self._pos_yaw + self._pos_roll + self._pos_tip)
                vec[11:15] = [0, 0, 0, 0]
                return vec

    def _position_timeout(self, frames, before, context, timeout_s, cause=None):
        # Snapshot while holding the same lock as the receiver: late replies
        # must not rewrite the evidence captured when this query failed.
        with self._position_condition:
            now = time.monotonic()
            received = [ft for ft in frames if self._position_updates.get(ft, 0) > before[ft]]
        sent = context['sent_frames']
        in_flight = context['in_flight_frame']
        missing = [ft for ft in sent if ft not in received]
        frame_name = lambda ft: f'0x{ft:02X}'
        details = {
            'query_type': 'position', 'query_phase': context['query_phase'],
            'protocol': 'legacy' if self.legacy_l20_mode else 'serial',
            'side': self.side, 'can_channel': self.can_channel, 'can_id': hex(self.can_id),
            # Preserve the existing receive-only timeout, not a total deadline.
            'timeout_s': float(timeout_s), 'timeout_scope': 'receive_reply',
            'elapsed_s': round(now - context['started'], 6),
            'query_lock_wait_s': round(context['query_lock_wait_s'], 6),
            'command_lock_wait_s': round(context['command_lock_wait_s'], 6),
            'send_elapsed_s': round(context['send_elapsed_s'], 6),
            'receive_wait_s': round(now - context['receive_started'], 6)
                              if context['receive_started'] is not None else 0.0,
            'expected_frames': [frame_name(ft) for ft in frames],
            'sent_frames': [frame_name(ft) for ft in sent],
            'received_frames': [frame_name(ft) for ft in received],
            'missing_frames': [frame_name(ft) for ft in missing],
            'in_flight_frame': frame_name(in_flight) if in_flight is not None else None,
            'not_sent_frames': [frame_name(ft) for ft in frames if ft not in sent and ft != in_flight],
            'request_elapsed_s': {frame_name(ft): round(value, 6)
                                  for ft, value in context['request_elapsed_s'].items()},
        }
        if self.legacy_l20_mode:
            details['missing_groups'] = [dict(zip(frames, ('pitch', 'yaw', 'roll', 'tip')))[ft]
                                         for ft in missing]
        else:
            details['missing_fingers'] = [dict(zip(frames, FINGER_NAMES))[ft] for ft in missing]
        if cause is not None:
            details.update(cause_type=type(cause).__name__, cause_message=str(cause))
        side = "左手" if self.side == "left" else "右手"
        message = (
            f"绑定的{side}未响应完整位置查询（CAN 接口 {self.can_channel}，ID {hex(self.can_id)}）；"
            "请检查电源及连接；若已更换左右手，请在本卡片实例配置中重新绑定；"
            f"查询阶段 {details['query_phase']}，耗时 {details['elapsed_s']:.3f}s，"
            f"已下发但缺失的回复帧：{', '.join(details['missing_frames']) or '无'}"
        )
        return PositionQueryTimeout(message, details)

    def get_positions(self) -> dict[str, int]:
        """Return current positions as a dictionary of the 16 active joints."""
        vec = self.get_positions_vector()
        return vector20_to_named16(vec)

    def probe_positions(self, timeout_s: float = 0.3) -> bool:
        """Non-destructive query to test if the hand responds on the CAN bus."""
        try:
            self._query_positions(timeout_s)
        except TimeoutError:
            return False
        return True

    def get_speeds(self) -> list[int]:
        self.send_command(FT_SPEED_R, [0], sleep_s=0.003)
        return list(self._speeds)

    def get_faults(self) -> list[int]:
        self.send_command(FT_FAULT_R, [], sleep_s=0.003)
        return list(self._faults)

    def request_device_info(self) -> None:
        """Query UID, Hardware version, and Software version."""
        self.send_command(FT_UID, [0], sleep_s=0.002)
        self.send_command(FT_HW_VER, [0], sleep_s=0.002)
        self.send_command(FT_SW_VER, [0], sleep_s=0.002)

    def get_version(self) -> dict[str, Any]:
        return {
            "uid": self._uid,
            "hw_version": self._hw_version,
            "sw_version": self._sw_version,
            "can_id": hex(self.can_id),
            "side": self.side,
        }

    # ──────────────────────────────────────────────────────────────────
    # Tactile / Force Sensing API
    # ──────────────────────────────────────────────────────────────────

    def request_tactile_scalar(self) -> None:
        """Query scalar pressure sensor on all 5 fingertips."""
        for ft in (FT_TOUCH_B1, FT_TOUCH_B2, FT_TOUCH_B3, FT_TOUCH_B4, FT_TOUCH_B5):
            self.send_command(ft, [], sleep_s=0.01)

    def _acquire_tactile_query(self, remaining_time) -> None:
        """Retain FIFO order across cancellation polls, including scalar reads."""
        ticket = object()
        with self._tactile_query_condition:
            self._tactile_query_waiters.append(ticket)
        acquired = []
        try:
            with self._tactile_query_condition:
                while self._tactile_query_waiters[0] is not ticket:
                    self._tactile_query_condition.wait(min(remaining_time(), 0.02))
            # Keep the ticket at the head while waiting for the active query.
            # A streaming reader cannot reacquire ahead of an existing waiter.
            while not self._tactile_query_lock.acquire(timeout=min(remaining_time(), 0.02)):
                pass
            acquired.append(self._tactile_query_lock)
            # Keep position exchanges from interleaving with the bounded
            # multi-row query, including while its receiver is catching up.
            while not self._position_query_lock.acquire(timeout=min(remaining_time('position_lock'), 0.02)):
                pass
            acquired.append(self._position_query_lock)
            # Exclude other commands too, without blocking the receive thread's
            # state lock. Reentrancy permits this query's own request frames.
            while not self._command_lock.acquire(timeout=min(remaining_time('command_lock'), 0.02)):
                pass
            acquired.append(self._command_lock)
        except BaseException:
            for lock in reversed(acquired):
                lock.release()
            raise
        finally:
            with self._tactile_query_condition:
                self._tactile_query_waiters.remove(ticket)
                self._tactile_query_condition.notify_all()

    def _release_tactile_query(self) -> None:
        self._command_lock.release()
        self._position_query_lock.release()
        self._tactile_query_lock.release()

    def _tactile_timeout(self, kind, phase, started, timeout_s, acquired_at,
                         names, requests_sent, missing) -> TactileQueryTimeout:
        now = time.monotonic()
        lock_wait = (now if acquired_at is None else acquired_at) - started
        details = {
            'query_type': kind, 'query_phase': phase, 'side': self.side,
            'can_channel': self.can_channel, 'can_id': hex(self.can_id),
            'timeout_s': timeout_s, 'elapsed_s': now - started,
            'lock_wait_s': lock_wait,
            'active_query_s': 0.0 if acquired_at is None else now - acquired_at,
            'requested_fingers': list(names), 'requests_sent': list(requests_sent),
            'missing_rows' if kind == 'matrix' else 'missing_fingers': missing,
        }
        label = '触觉阵列' if kind == 'matrix' else '指尖触觉'
        suffix = ', '.join(missing) if missing else '无'
        return TactileQueryTimeout(
            f'{label}查询超时，未能在预算内确认本轮完整反馈：阶段={phase}，等待查询锁={lock_wait:.3f}s，'
            f'总耗时={now - started:.3f}s，缺失手指={suffix}', details)

    def get_fresh_tactile(self, fingers: list[str] | None = None, *,
                          timeout_s: float | None = None, cancel=None) -> dict[str, Any]:
        """Require new scalar replies within a cancellable, whole-query budget."""
        names = list(FINGER_NAMES if fingers is None else fingers)
        if not names or any(name not in FINGER_NAMES for name in names):
            raise ValueError("fingers must contain valid finger names")
        timeout_s = self._recv_timeout_s if timeout_s is None else timeout_s
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError("timeout_s must be a finite positive number")
        frames = [FT_TOUCH_B1 + FINGER_NAMES.index(name) for name in names]
        started = time.monotonic()
        deadline = started + timeout_s
        phase, acquired_at, before = 'query_lock', None, None
        requests_sent = []

        def remaining_time(next_phase=None):
            nonlocal phase
            if next_phase is not None:
                phase = next_phase
            if cancel is not None and cancel.is_set():
                raise InterruptedError("触觉查询已中断")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = []
                if before is not None:
                    with self._position_condition:
                        missing = [name for name, ft in zip(names, frames)
                                   if self._tactile_updates.get(ft, 0) <= before[ft]]
                raise self._tactile_timeout('scalar', phase, started, timeout_s,
                                            acquired_at, names, requests_sent, missing)
            return remaining

        self._acquire_tactile_query(remaining_time)
        try:
            acquired_at = time.monotonic()
            remaining_time()
            phase = 'send_request'
            with self._position_condition:
                before = {ft: self._tactile_updates.get(ft, 0) for ft in frames}
            for name, ft in zip(names, frames):
                remaining_time()
                self.send_command(ft, [], sleep_s=0.003)
                requests_sent.append(name)
                remaining_time()
            phase = 'receive_reply'
            with self._position_condition:
                while any(self._tactile_updates.get(ft, 0) <= before[ft] for ft in frames):
                    self._position_condition.wait(min(remaining_time(), 0.02))
                phase = 'complete'
                remaining_time()
                return self.get_tactile_snapshot()
        finally:
            self._release_tactile_query()

    def get_fresh_matrix_touch(self, fingers: list[str] | None = None, *,
                               timeout_s: float = 1.0,
                               cancel: threading.Event | None = None) -> dict[str, list[list[int]]]:
        """Require all 12 new rows per requested finger within one query deadline.

        Omitting ``fingers`` retains the five-finger, 60-row result. Cached rows
        never satisfy a new query, including when only some fingers are selected.
        """
        if fingers is not None and not isinstance(fingers, list):
            raise ValueError("fingers must contain valid finger names")
        names = list(FINGER_NAMES if fingers is None else fingers)
        if (not names or any(name not in FINGER_NAMES for name in names)
                or len(set(names)) != len(names)):
            raise ValueError("fingers must contain distinct valid finger names")
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError("timeout_s must be a finite positive number")
        fingers_by_index = [(name, FINGER_NAMES.index(name)) for name in names]
        keys = [(finger, row) for _, finger in fingers_by_index for row in range(12)]
        started = time.monotonic()
        deadline = started + timeout_s
        phase, acquired_at, before = 'query_lock', None, None
        requests_sent = []
        retry_fingers = []

        def remaining_time(next_phase=None) -> float:
            nonlocal phase
            if next_phase is not None:
                phase = next_phase
            if cancel is not None and cancel.is_set():
                raise InterruptedError("触觉阵列查询已中断")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = {}
                if before is not None:
                    with self._position_condition:
                        missing = {name: [row for row in range(12)
                                          if self._matrix_updates.get((finger, row), 0) <= before[(finger, row)]]
                                   for name, finger in fingers_by_index}
                        missing = {name: rows for name, rows in missing.items() if rows}
                error = self._tactile_timeout('matrix', phase, started, timeout_s,
                                             acquired_at, names, requests_sent, missing)
                error.details.update(retry_count=int(bool(retry_fingers)), retry_fingers=list(retry_fingers))
                raise error
            return remaining

        self._acquire_tactile_query(remaining_time)
        try:
            acquired_at = time.monotonic()
            remaining_time()
            phase = 'send_request'
            with self._position_condition:
                before = {key: self._matrix_updates.get(key, 0) for key in keys}
            for request_index, (name, finger) in enumerate(fingers_by_index):
                remaining = remaining_time()
                # Preserve the existing spacing between requests; waiting for
                # replies supplies the final request's delay without extra sleep.
                sleep_s = min(0.01, remaining) if request_index + 1 < len(fingers_by_index) else 0
                self.send_command(FT_TOUCH_B1 + finger, [FT_MATRIX_REQ], sleep_s=sleep_s)
                requests_sent.append(name)
                remaining_time()
            phase = 'receive_rows'
            # Reserve half of the remaining reply budget for a single recovery
            # request. The original whole-query deadline never moves.
            retry_at = time.monotonic() + remaining_time() / 2
            while True:
                with self._position_condition:
                    missing = [(name, finger) for name, finger in fingers_by_index
                               if any(self._matrix_updates.get((finger, row), 0) <= before[(finger, row)]
                                      for row in range(12))]
                    if not missing:
                        phase = 'complete'
                        remaining_time()
                        return {name: [list(row) for row in self._matrix_touch[finger]]
                                for name, finger in fingers_by_index}
                    remaining = remaining_time()
                    if retry_fingers or time.monotonic() < retry_at:
                        wait = min(remaining, 0.02)
                        if not retry_fingers:
                            wait = min(wait, max(0, retry_at - time.monotonic()))
                        self._position_condition.wait(wait)
                        continue
                    retry_fingers = [name for name, _ in missing]
                    # Require all 12 rows anew for every retried finger, rather
                    # than combining old partial rows with a later partial reply.
                    for _, finger in missing:
                        for row in range(12):
                            before[(finger, row)] = self._matrix_updates.get((finger, row), 0)
                phase = 'resend_request'
                for request_index, (name, finger) in enumerate(missing):
                    remaining = remaining_time()
                    sleep_s = min(0.01, remaining) if request_index + 1 < len(missing) else 0
                    self.send_command(FT_TOUCH_B1 + finger, [FT_MATRIX_REQ], sleep_s=sleep_s)
                    requests_sent.append(name)
                    remaining_time()
                phase = 'receive_rows'
        finally:
            self._release_tactile_query()

    def request_normal_force(self) -> None:
        self.send_command(FT_NORMAL_FORCE, [], sleep_s=0.003)
        if not self.legacy_l20_mode:
            self.send_command(0x90, [], sleep_s=0.003)

    def request_tangential_force(self) -> None:
        self.send_command(FT_TANGENTIAL, [], sleep_s=0.003)
        if not self.legacy_l20_mode:
            self.send_command(0x91, [], sleep_s=0.003)

    def request_all_tactile(self) -> None:
        """Query scalar pressure, normal force, tangential force, direction, and approach on all fingertips."""
        self.request_tactile_scalar()
        self.send_command(FT_NORMAL_FORCE, [], sleep_s=0.003)
        self.send_command(FT_TANGENTIAL, [], sleep_s=0.003)
        if not self.legacy_l20_mode:
            self.send_command(0x90, [], sleep_s=0.003)
            self.send_command(0x91, [], sleep_s=0.003)
            self.send_command(0x92, [], sleep_s=0.003)
            self.send_command(0x93, [], sleep_s=0.003)

    def request_matrix_touch(self) -> None:
        """Trigger update of the 12x6 tactile matrix for all 5 fingertips."""
        for ft in (FT_TOUCH_B1, FT_TOUCH_B2, FT_TOUCH_B3, FT_TOUCH_B4, FT_TOUCH_B5):
            self.send_command(ft, [FT_MATRIX_REQ], sleep_s=0.01)

    def get_tactile_snapshot(self) -> dict[str, Any]:
        """Return the latest cached readings of all tactile sensors."""
        with self._lock:
            return {
                "touch_scalar": {
                    name: self._touch[i] for i, name in enumerate(FINGER_NAMES)
                },
                "normal_force": {
                    name: self._normal_force[i] for i, name in enumerate(FINGER_NAMES)
                },
                "tangential_force": {
                    name: self._tangential_force[i] for i, name in enumerate(FINGER_NAMES)
                },
                "tangential_dir": {
                    name: self._tangential_dir[i] for i, name in enumerate(FINGER_NAMES)
                },
                "approach_inc": {
                    name: self._approach_inc[i] for i, name in enumerate(FINGER_NAMES)
                },
                "scalar_age_s": {
                    name: (max(0.0, time.monotonic() - self._tactile_times[FT_TOUCH_B1 + i])
                           if FT_TOUCH_B1 + i in self._tactile_times else None)
                    for i, name in enumerate(FINGER_NAMES)
                },
            }


    def get_matrix_touch(self) -> dict[str, list[list[int]]]:
        """Return the 12x6 tactile matrices for all fingers."""
        return {
            name: self._matrix_touch[i] for i, name in enumerate(FINGER_NAMES)
        }

    # ──────────────────────────────────────────────────────────────────
    # Background Receive Loop & Frame Parsing
    # ──────────────────────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        import can
        while self._running:
            try:
                if self._bus is None:
                    time.sleep(0.5)
                    try:
                        self._bus = self._open_bus(self.can_channel, self.baudrate)
                    except Exception:
                        continue
                bus = self._bus
                msg = bus.recv(timeout=self._recv_timeout_s)
                if msg and msg.arbitration_id == self.can_id:
                    self._process_message(msg)
                if msg is not None:
                    self._drain_ready_messages(bus)
            except can.CanError:
                pass
            except Exception:
                pass

    def _drain_ready_messages(self, bus) -> None:
        """Consume an already queued SocketCAN burst without selecting per row."""
        if sys.platform != 'linux' or not hasattr(bus, 'socket'):
            return
        from can.interfaces.socketcan import socketcan
        if not isinstance(bus, socketcan.SocketcanBus):
            return
        reader = _NonblockingSocketReader(bus.socket)
        # Bound each burst so shutdown and reconnect are checked regularly.
        # A full five-finger matrix contains 60 frames. Never wait for an
        # additional frame here: the normal bus.recv above owns that wait.
        for _ in range(64):
            if not self._running or self._bus is not bus:
                return
            try:
                msg = socketcan.capture_message(reader, bus.channel == '')
            except socketcan.can.CanOperationError as exc:
                if exc.error_code in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                raise
            if msg and msg.arbitration_id == self.can_id:
                self._process_message(msg)

    def _process_message(self, msg) -> None:
        # Publishing the frame counters and its position values is atomic for readers.
        with self._position_condition:
            self._update_message(msg)
            data = list(msg.data)
            if not data:
                return
            frame_type = data[0]
            if FT_TOUCH_B1 <= frame_type <= FT_TOUCH_B5:
                if len(data) == 3:
                    self._tactile_updates[frame_type] = self._tactile_updates.get(frame_type, 0) + 1
                    self._tactile_times[frame_type] = time.monotonic()
                    self._position_condition.notify_all()
                elif len(data) == 8 and data[1] in MATRIX_OFFSET_MAP:
                    key = (frame_type - FT_TOUCH_B1, MATRIX_OFFSET_MAP[data[1]])
                    self._matrix_updates[key] = self._matrix_updates.get(key, 0) + 1
                    # Normal replies end at row 11. Avoid waking a reader to
                    # rescan every requested row after each individual frame;
                    # under QEMU that can delay the receiver itself. Readers
                    # also poll every 20 ms, so reordered rows remain visible.
                    if key[1] == 11:
                        self._position_condition.notify_all()
            if (
                frame_type in (FT_THUMB_POS, FT_INDEX_POS, FT_MIDDLE_POS, FT_RING_POS, FT_LITTLE_POS)
                and len(data) >= 7
            ) or (
                frame_type in (FT_PITCH_R, FT_YAW_R, FT_ROLL_R, FT_TIP_R)
                and len(data) >= 6
            ):
                self._position_updates[frame_type] = self._position_updates.get(frame_type, 0) + 1
                self._position_condition.notify_all()

    def _update_message(self, msg) -> None:
        data = list(msg.data)
        if not data:
            return
        frame_type = data[0]
        payload = data[1:]

        # G20 / L20-10 Serial Position feedback (6 bytes per finger)
        # thumb: [roll, yaw, root, 0, 0, tip]
        if frame_type == FT_THUMB_POS and len(payload) >= 6:
            self._pos_roll[0]  = payload[0]
            self._pos_yaw[0]   = payload[1]
            self._pos_pitch[0] = payload[2]
            self._pos_tip[0]   = payload[5]
        # other fingers: [yaw, 0, root, 0, 0, tip]
        elif frame_type == FT_INDEX_POS and len(payload) >= 6:
            self._pos_yaw[1]   = payload[0]
            self._pos_pitch[1] = payload[2]
            self._pos_tip[1]   = payload[5]
        elif frame_type == FT_MIDDLE_POS and len(payload) >= 6:
            self._pos_yaw[2]   = payload[0]
            self._pos_pitch[2] = payload[2]
            self._pos_tip[2]   = payload[5]
        elif frame_type == FT_RING_POS and len(payload) >= 6:
            self._pos_yaw[3]   = payload[0]
            self._pos_pitch[3] = payload[2]
            self._pos_tip[3]   = payload[5]
        elif frame_type == FT_LITTLE_POS and len(payload) >= 6:
            self._pos_yaw[4]   = payload[0]
            self._pos_pitch[4] = payload[2]
            self._pos_tip[4]   = payload[5]

        # Legacy L20 parallel frame feedback
        elif frame_type == FT_PITCH_R and len(payload) >= 5:
            self._pos_pitch = payload[:5]
        elif frame_type == FT_YAW_R and len(payload) >= 5:
            self._pos_yaw = payload[:5]
        elif frame_type == FT_ROLL_R and len(payload) >= 5:
            self._pos_roll = payload[:5]
        elif frame_type == FT_TIP_R and len(payload) >= 5:
            self._pos_tip = payload[:5]
        elif frame_type == FT_SPEED_R and len(payload) >= 5:
            self._speeds = payload[:5]
        elif frame_type == FT_CURRENT_R and len(payload) >= 5:
            self._currents = payload[:5]
        elif frame_type == FT_FAULT_R and len(payload) >= 5:
            self._faults = payload[:5]

        elif frame_type in (FT_NORMAL_FORCE, 0x90) and len(payload) >= FINGER_COUNT:
            self._normal_force = [float(v) for v in payload[:FINGER_COUNT]]
        elif frame_type in (FT_TANGENTIAL, 0x91) and len(payload) >= FINGER_COUNT:
            self._tangential_force = [float(v) for v in payload[:FINGER_COUNT]]
        elif frame_type in (FT_TANGENTIAL_DIR, 0x92) and len(payload) >= FINGER_COUNT:
            self._tangential_dir = [float(v) for v in payload[:FINGER_COUNT]]
        elif frame_type in (FT_APPROACH, 0x93) and len(payload) >= FINGER_COUNT:
            self._approach_inc = [float(v) for v in payload[:FINGER_COUNT]]

        elif frame_type == FT_TOUCH_TYPE and len(payload) >= 1:
            self._touch_type = payload[0]

        elif frame_type in (FT_TOUCH_B1, FT_TOUCH_B2, FT_TOUCH_B3, FT_TOUCH_B4, FT_TOUCH_B5):
            finger_idx = frame_type - FT_TOUCH_B1
            if len(payload) == 2:
                # [flag, scalar_pressure_0_to_255]
                self._touch[finger_idx] = payload[1]
            elif len(payload) == 7:
                # Matrix row: payload[0] is row offset (0, 16, 32... 176), payload[1..7] are 6 taxels
                row_offset = payload[0]
                row_idx = MATRIX_OFFSET_MAP.get(row_offset)
                if row_idx is not None and row_idx < 12:
                    self._matrix_touch[finger_idx][row_idx] = payload[1:7]

        elif frame_type == FT_UID:
            self._uid = payload
        elif frame_type == FT_HW_VER:
            self._hw_version = payload
        elif frame_type == FT_SW_VER:
            self._sw_version = payload

    def close(self) -> None:
        """Stop receive loop and release CAN bus resources."""
        self._running = False
        try:
            self._bus.shutdown()
        except Exception:
            pass
