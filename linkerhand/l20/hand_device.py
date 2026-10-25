"""PhanthyMotus hand card for the LinkerHand L20 (USB-CAN, 16 active joints).

Architecture mirrors the L20 Lite modbus_device.py but adapts to the USB-CAN transport
at 1 Mbit/s and the L20 16-active-joint layout (21 joints total, 16 DOF).
Each device request explicitly selects a hand. The ACP callback pattern and
confirm_motion safety gate remain in place.

Joint Layout (16 active joints in canonical order):
  Thumb (4):  thumb_pitch, thumb_yaw, thumb_roll, thumb_tip
  Index (3):  index_pitch, index_yaw, index_tip
  Middle (3): middle_pitch, middle_yaw, middle_tip
  Ring (3):   ring_pitch, ring_yaw, ring_tip
  Pinky (3):  pinky_pitch, pinky_yaw, pinky_tip
"""

from __future__ import annotations

import json
import os
import ssl
import subprocess
import threading
import time
import urllib.request
from typing import Any, Dict, List, Optional, Tuple, Union
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from can_client import (
    ACTIVE_JOINT_COUNT,
    VECTOR_JOINT_COUNT,
    FINGER_COUNT,
    RIGHT_HAND_CAN_ID,
    LEFT_HAND_CAN_ID,
    JOINT_NAMES_16,
    CHINESE_JOINT_NAMES_16,
    FINGER_NAMES,
    L20CanClient,
    validate_position_vector,
    vector20_to_named16,
    named16_to_vector20,
)

JOINT_NAMES = list(JOINT_NAMES_16)
CHINESE_JOINT_NAMES = list(CHINESE_JOINT_NAMES_16)
JOINT_FIELDS = {name: idx for idx, name in enumerate(JOINT_NAMES)}
STRUCTURED_POSITION_FIELDS = list(JOINT_NAMES)

MOTION_ACTIONS = ("open", "close", "reset", "relax", "set_positions")
SPEED_RATIOS = {f"{r:.1f}x": r for r in (1.0, 1.5, 2.0, 2.5, 3.0)}


def _named(values: Union[list[int], dict[str, int]]) -> dict[str, int]:
    if isinstance(values, dict):
        return values
    if len(values) == 20:
        return vector20_to_named16(values)
    if len(values) == 16:
        return dict(zip(JOINT_NAMES, values))
    raise ValueError(f"Cannot name vector of length {len(values)}")


def _setting(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
        raise ValueError(f"{name} must be an integer in 1..255")
    return value


def validate_hold_torque(value: Any, default: int = 0) -> int:
    """Validate hold_torque as an integer 0..255; None or '' defaults to default (0)."""
    if value is None or value == "":
        return default
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError(f"hold_torque 必须是 0–255 之间的整数，收到: {value!r}")
    return value


def verify_hand_connection(client: L20CanClient) -> None:
    """Require a fresh read-only response from the selected CAN ID before motion."""
    side = "左手" if client.side == "left" else "右手"
    message = (
        f"所选{side}未响应（CAN 接口 {client.can_channel}，ID {hex(client.can_id)}）；"
        "请检查电源、连接及本次选择的左右手"
    )
    try:
        online = client.probe_positions()
    except Exception as exc:
        raise RuntimeError(f"{message}：{exc}") from exc
    if not online:
        raise RuntimeError(message)


# ──────────────────────────────────────────────────────────────────────
# CAN device discovery
# ──────────────────────────────────────────────────────────────────────

def _discover_can_channels() -> list[str]:
    """Return all available socketcan channel names on Linux."""
    try:
        out = subprocess.check_output(
            ["ip", "-o", "link", "show", "type", "can"],
            text=True, timeout=2,
        )
        channels = []
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 2:
                name = parts[1].strip().split("@")[0]
                if name:
                    channels.append(name)
        return channels if channels else ["can0"]
    except Exception:
        return ["can0"]


def _probe_hand(channel: str, side: str, can_id: Optional[int] = None) -> bool:
    """Return True if the given hand side responds on this CAN channel."""
    try:
        client = L20CanClient(channel, side, can_id=can_id, recv_timeout_s=0.4)
        time.sleep(0.08)
        alive = client.probe_positions()
        client.close()
        return alive
    except Exception:
        return False


# ──────────────────────────────────────────────────────────────────────
# Low-level hand controller (one physical hand)
# ──────────────────────────────────────────────────────────────────────

class _SingleHandController:
    """Controls one physical L20 hand bound to a specific side."""

    def __init__(
        self,
        config: dict,
        side: str,
        channel: str,
        motion_lock: threading.RLock,
        can_id: Optional[int] = None,
    ):
        self.config = config
        self.side = side
        self.channel = channel
        self.can_id = can_id
        self._motion_lock = motion_lock

        self._client: L20CanClient | None = None
        self._client_lock = threading.Lock()
        self._stop_evt = threading.Event()

        self.motion_timeout_s = float(config.get("motion_timeout_s", 8.0))
        self.position_tolerance = int(config.get("position_tolerance", 18))
        self.poll_interval_s = float(config.get("poll_interval_s", 0.08))

        # Default calibrated 20-element vectors (L20-10 user-calibrated)
        default_open = [255, 255, 255, 255, 255, 70, 200, 132, 55, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        default_fist = [110, 0, 0, 0, 0, 140, 193, 158, 128, 91, 255, 255, 255, 255, 255, 119, 0, 0, 0, 0]
        default_natural = [135, 255, 255, 255, 255, 70, 110, 100, 99, 100, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]

        prefix = f"{side}_" if side == "left" else ""
        open_cfg = config.get(f"{prefix}open_pose") or config.get("open_pose") or default_open
        fist_cfg = config.get(f"{prefix}fist_pose") or config.get("fist_pose") or default_fist
        natural_cfg = config.get(f"{prefix}natural_pose") or config.get("natural_pose") or default_natural

        self._open_pose: list[int] = validate_position_vector(open_cfg, f"{side}_open_pose")
        self._fist_pose: list[int] = validate_position_vector(fist_cfg, f"{side}_fist_pose")
        self._natural_pose: list[int] = validate_position_vector(natural_cfg, f"{side}_natural_pose")

    def get(self) -> L20CanClient:
        with self._client_lock:
            if self._client is None:
                self._client = L20CanClient(
                    self.channel, self.side,
                    baudrate=self.config.get("baudrate", 1_000_000),
                    can_id=self.can_id,
                )
            return self._client

    def close(self) -> None:
        with self._client_lock:
            client, self._client = self._client, None
        if client is not None:
            client.close()

    def release_torque(self) -> None:
        try:
            self.get().set_currents([0] * 5)
        except Exception:
            pass

    def target_for_action(self, action: str, joint_overrides: dict | None, client: L20CanClient) -> list[int]:
        if action == "set_positions":
            target_vec = client.get_positions_vector()
            if joint_overrides:
                named = vector20_to_named16(target_vec)
                for k, v in joint_overrides.items():
                    if k in named:
                        named[k] = v
                return named16_to_vector20(named)
            return target_vec

        if action in ("relax", "reset"):
            return list(self._natural_pose)
        return list(self._open_pose if action == "open" else self._fist_pose)

    def move(
        self,
        target_vec: list[int],
        action: str,
        *,
        speed: int,
        torque: int,
        hold_torque: int = 0,
    ) -> dict:
        client = self.get()
        with self._motion_lock:
            ensure_motion_available(self)
            if self._stop_evt.is_set():
                raise InterruptedError("hand motion stopped")

            current_vec = client.get_positions_vector()
            # Flexion/rotation vs abduction tolerance check
            diffs_flexion = [abs(current_vec[i] - target_vec[i]) for i in (0, 1, 2, 3, 4, 10, 15, 16, 17, 18, 19)]
            diffs_yaw = [abs(current_vec[i] - target_vec[i]) for i in (5, 6, 7, 8, 9)]
            if (
                all(d <= self.position_tolerance for d in diffs_flexion)
                and all(d <= max(self.position_tolerance, 35) for d in diffs_yaw)
            ):
                client.set_currents([hold_torque] * 5)
                return {
                    "state": "already_at_target", "action": action,
                    "side": self.side, "positions": _named(current_vec),
                    "torque": hold_torque,
                    "hold_torque": hold_torque,
                    "torque_released": (hold_torque == 0),
                }

            energized = False
            try:
                # Set speeds and currents/torques
                client.set_speeds([speed] * 5)
                client.set_currents([min(255, max(10, torque))] * 5)
                energized = True

                if self._stop_evt.is_set():
                    raise InterruptedError("hand motion stopped")

                client.set_joint_positions(target_vec)
                deadline = time.monotonic() + self.motion_timeout_s
                measured_vec = current_vec

                while time.monotonic() < deadline:
                    if self._stop_evt.wait(self.poll_interval_s):
                        raise InterruptedError("hand motion stopped")
                    measured_vec = client.get_positions_vector()
                    diffs_flexion = [abs(measured_vec[i] - target_vec[i]) for i in (0, 1, 2, 3, 4, 10, 15, 16, 17, 18, 19)]
                    diffs_yaw = [abs(measured_vec[i] - target_vec[i]) for i in (5, 6, 7, 8, 9)]
                    if (
                        all(d <= self.position_tolerance for d in diffs_flexion)
                        and all(d <= max(self.position_tolerance, 35) for d in diffs_yaw)
                    ):
                        client.set_currents([hold_torque] * 5)
                        return {
                            "state": "completed", "action": action,
                            "side": self.side,
                            "requested_positions": _named(target_vec),
                            "positions": _named(measured_vec),
                            "speed": speed,
                            "torque": hold_torque,
                            "hold_torque": hold_torque,
                            "torque_released": (hold_torque == 0),
                        }

                # Timed out -> check if near target (settled under physical load)
                if all(d <= max(self.position_tolerance * 2, 35) for d in diffs_flexion):
                    client.set_currents([hold_torque] * 5)
                    return {
                        "state": "completed", "action": action,
                        "side": self.side,
                        "requested_positions": _named(target_vec),
                        "positions": _named(measured_vec),
                        "speed": speed,
                        "torque": hold_torque,
                        "hold_torque": hold_torque,
                        "torque_released": (hold_torque == 0),
                    }

                client.set_currents([hold_torque] * 5)
                return {
                    "state": "released_timeout", "action": action,
                    "side": self.side,
                    "requested_positions": _named(target_vec),
                    "positions": _named(measured_vec),
                    "torque": hold_torque,
                    "hold_torque": hold_torque,
                    "torque_released": (hold_torque == 0),
                }
            except Exception:
                if energized:
                    try:
                        client.set_currents([0] * 5)
                    except Exception:
                        pass
                raise


GLOBAL_BUS_LOCK = threading.RLock()
GLOBAL_CONTROLLERS: dict[tuple[str, str], _SingleHandController] = {}


def validate_bound_side(value: Any) -> str:
    if not isinstance(value, str) or value not in ("left", "right"):
        raise ValueError("请选择本次操作的左手或右手（bound_side 必须是 left 或 right）")
    return value


def ensure_motion_available(ctrl, owner=None) -> None:
    """A position grasp reserves its hand until release or terminal cleanup."""
    current = vars(ctrl).get("_grasp_owner")
    if current is not None and current != owner:
        raise RuntimeError("该手正在抓握或保持，请先选择该手执行 release")


def claim_motion(ctrl, owner, *, config: dict | None = None) -> None:
    """Reserve one physical hand before starting a worker, across all actuators."""
    with GLOBAL_BUS_LOCK:
        if config is not None and config.get("can_channel", "can0") != ctrl.channel:
            raise RuntimeError("执行器 CAN 接口刚被修改，请重新执行本次动作")
        ensure_motion_available(ctrl, owner)
        current = vars(ctrl).get("_command_owner")
        if current is not None and current != owner:
            raise RuntimeError("该手已有动作正在执行，请等待完成或显式停止该手")
        if current is None:
            ctrl._command_previous_owner = vars(ctrl).get("_last_command_owner")
        ctrl._command_owner = owner
        ctrl._last_command_owner = owner


def release_motion(ctrl, owner, *, restore_previous: bool = False) -> None:
    with GLOBAL_BUS_LOCK:
        if vars(ctrl).get("_command_owner") == owner:
            if restore_previous:
                previous = vars(ctrl).get("_command_previous_owner")
                if previous is None:
                    vars(ctrl).pop("_last_command_owner", None)
                else:
                    ctrl._last_command_owner = previous
            del ctrl._command_owner
            vars(ctrl).pop("_command_previous_owner", None)


def get_shared_controller(
    config: dict,
    side: str,
    channel: str = "can0",
    can_id: Optional[int] = None,
    bus_lock: Optional[threading.RLock] = None,
) -> _SingleHandController:
    key = (side, channel)
    lock = bus_lock or GLOBAL_BUS_LOCK
    with lock:
        ctrl = GLOBAL_CONTROLLERS.get(key)
        if ctrl is None:
            if can_id is None:
                left_id = int(config.get("left_can_id", LEFT_HAND_CAN_ID))
                right_id = int(config.get("right_can_id", RIGHT_HAND_CAN_ID))
                can_id = right_id if side == "right" else left_id
            # Registry synchronization is separate from per-hand motor ownership:
            # a long motion on the other hand must not block pressure monitoring.
            ctrl = _SingleHandController(config, side, channel, threading.RLock(), can_id=can_id)
            GLOBAL_CONTROLLERS[key] = ctrl
        return ctrl


# ──────────────────────────────────────────────────────────────────────
# MultiHandPlugin — the MCP tool exposed to Agent Core
# ──────────────────────────────────────────────────────────────────────

class HandPlugin:
    """Multi-instance MCP hand card for LinkerHand L20."""

    PREFIX = "hand"

    def __init__(self, config: dict, hand_config: dict):
        self.config = config
        self.hand_config = hand_config
        self._bus_lock = GLOBAL_BUS_LOCK
        self._lock = threading.RLock()
        self._controllers: dict[tuple[str, str], _SingleHandController] = {}

        # Device state is shared by cards selecting the same physical hand.
        self._active: dict[tuple[str, str], dict] = {}
        self._last_result: dict[tuple[str, str], dict] = {}
        self._delivery_failed: dict[tuple[str, str], bool] = {}
        self._held: dict[tuple[str, str], dict] = {}

        self.motion_speed   = _setting(hand_config.get("motion_speed",  80), "motion_speed")
        self.motion_torque  = _setting(hand_config.get("motion_torque", 120), "motion_torque")
        self.motion_timeout_s = float(hand_config.get("motion_timeout_s", 8.0))

        self.right_can_id = int(config.get("right_can_id", RIGHT_HAND_CAN_ID))
        self.left_can_id  = int(config.get("left_can_id", LEFT_HAND_CAN_ID))

    # ── tool schema ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "info": (["bound_side"], "只读检测所选手及设备状态，不驱动手指"),
                "start": ([], "启动卡片控制"),
                "stop": (["bound_side"], "停止所选手的基础控制动作并释放本工具保持的力矩"),
                "get_state": (["bound_side"], "读取所选手 16 个主动关节当前位置（0–255）、速度及电机状态"),
                "open": (["bound_side", "speed_ratio", "set_torque", "hold_torque", "confirm_motion"],
                         "张开手掌到已校准姿态；set_torque 设置运动力矩；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）；每次运动须确认"),
                "close": (["bound_side", "speed_ratio", "set_torque", "hold_torque", "confirm_motion"],
                          "握拳到已校准姿态；set_torque 设置运动力矩；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）；每次运动须确认"),
                "reset": (["bound_side", "confirm_motion"],
                          "移动到自然放松姿态后释放所有关节电机力矩/电流；每次运动须确认"),
                "relax": (["bound_side", "speed_ratio", "set_torque", "hold_torque", "confirm_motion"],
                          "移动到自然下垂常态姿态；set_torque 设置运动力矩；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）；每次运动须确认"),
                "set_positions": (
                    ["bound_side", *STRUCTURED_POSITION_FIELDS, "speed_ratio", "hold_torque", "confirm_motion"],
                    "按需设置 16 个主动关节位置（0–255 协议值，非角度），留空保持当前位置；hold_torque 设置到位保持力矩（0–255，默认 0 断力）",
                ),
            },
            {
                "hold_torque": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 255,
                    "default": 0,
                    "description": "动作到位后保持力矩上限（0–255 整数）；留空默认为 0（到位自动断电卸力防发热）；>0 则以该力矩保持锁定",
                },
                **{
                    name: {
                        "type": "integer", "minimum": 0, "maximum": 255,
                        "description": (
                            f"{CHINESE_JOINT_NAMES[JOINT_FIELDS[name]]}："
                            "位置原始值 0–255（非角度）；留空保持当前位置"
                        ),
                    }
                    for name in STRUCTURED_POSITION_FIELDS
                },
                "speed_ratio": {
                    "type": "string", "enum": list(SPEED_RATIOS),
                    "description": "相对当前设置速度的倍率（1.0x 到 3.0x）",
                },
                "set_torque": {
                    "type": "integer", "minimum": 1, "maximum": 255,
                    "title": "SET_TORQUE",
                    "default": self.motion_torque,
                    "description": (
                        f"本次 open/close 运动的力矩/电流上限（1–255）；"
                        f"默认 {self.motion_torque}"
                    ),
                },
                "confirm_motion": {
                    "type": "boolean", "const": True,
                    "description": "确认手附近无人和障碍物后传入布尔值 true",
                },
                "bound_side": {"type": "string", "enum": ["", "left", "right"], "default": "",
                               "description": "本次操作的灵巧手：请选择 left（左手）或 right（右手）；空白不能执行"},
                "can_channel": {
                    "type": "string",
                    "description": "CAN 接口名称，如 can0、can1（Linux socketcan）",
                },
                "instance_id": {"type": "string"},
            },
        )
        schema["allOf"] = [{
            "if": {"properties": {"action": {"enum": list(MOTION_ACTIONS)}}},
            "then": {"required": ["confirm_motion"]},
        }, {
            "if": {"properties": {"action": {"enum": ["info", "get_state", *MOTION_ACTIONS]}}},
            "then": {"required": ["bound_side"], "properties": {"bound_side": {"enum": ["left", "right"]}}},
        }]
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {
            "actions": list(MOTION_ACTIONS),
            "timeout": int(self.motion_timeout_s) + 25,
        }

        definition = tool(
            "hand", "actuator",
            "LinkerHand L20 USB-CAN 16主动关节驱动与状态控制。每次运动须显式传 confirm_motion=true。",
            schema,
        )
        definition["title"] = "L20 灵巧手"
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "can_channel": {
                    "type": "string", "scope": "shared",
                    "title": "高级：执行器共享 CAN 接口", "default": self.config.get("can_channel", "can0"),
                    "description": "三张执行器共同使用的 socketcan 接口，例如 can0；触觉卡片独立配置，运动或保持期间不可修改",
                },
            },
        }
        schema["properties"]["action"]["enum"].append("config")
        schema["x-action-params"]["config"] = {
            "params": ["can_channel"],
            "description": "设定三张执行器共享 CAN 接口；不保存左右手绑定",
        }
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        for ctrl in list(self._controllers.values()):
            ctrl._stop_evt.set()
            ctrl.release_torque()
            ctrl.close()

    # ── discovery ────────────────────────────────────────────────────

    def _discover(self) -> dict[str, str]:
        """Return {side: channel} for all responding hands."""
        channels = _discover_can_channels()
        found: dict[str, str] = {}
        for channel in channels:
            for side in ("right", "left"):
                target_id = self.right_can_id if side == "right" else self.left_can_id
                if side not in found and _probe_hand(channel, side, can_id=target_id):
                    found[side] = channel
        return found

    # ── controller access ────────────────────────────────────────────

    def get_controller_for_side(self, side: str, channel: str | None = None) -> _SingleHandController:
        ch = channel or self.config.get("can_channel", "can0")
        target_id = self.right_can_id if side == "right" else self.left_can_id
        ctrl = get_shared_controller(self.hand_config, side, ch, can_id=target_id, bus_lock=self._bus_lock)
        with self._lock:
            self._controllers[(side, ch)] = ctrl
        return ctrl

    def _get_controller(self, side: str, channel: str | None = None) -> _SingleHandController:
        return self.get_controller_for_side(validate_bound_side(side), channel)

    # ── motion options ───────────────────────────────────────────────

    def _motion_options(self, action: str, args: dict) -> tuple[int, int]:
        if action == "reset":
            return self.motion_speed, self.motion_torque
        ratio = args.get("speed_ratio", "1.0x")
        if not isinstance(ratio, str) or ratio not in SPEED_RATIOS:
            raise ValueError(f"speed_ratio must be one of {', '.join(SPEED_RATIOS)}")
        speed = min(255, max(1, round(self.motion_speed * SPEED_RATIOS[ratio])))
        requested_torque = None
        if action in ("open", "close", "relax"):
            requested_torque = args.get("set_torque") or args.get("torque")
        torque = (
            self.motion_torque if requested_torque in (None, "")
            else _setting(requested_torque, "set_torque")
        )
        return speed, torque

    # ── ACP callback ─────────────────────────────────────────────────

    def _acp_callback(self, action_id: str, status: str, result: dict) -> None:
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        ctx = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
        if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(
            ("https://localhost:", "https://127.0.0.1:")
        ):
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        payload = json.dumps({
            "action_id": action_id, "status": status, "result": result,
            "tool": self.PREFIX, "ts": time.time(),
        }, ensure_ascii=False).encode()
        error = None
        for attempt in range(3):
            try:
                req = urllib.request.Request(
                    url + "/api/acp/complete", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with urllib.request.urlopen(req, timeout=5, context=ctx) as resp:
                    ack = json.load(resp)
                if ack.get("ok") is not True or ack.get("action_id") != action_id:
                    raise RuntimeError("Agent Core did not acknowledge action_id")
                return
            except Exception as exc:
                error = exc
                if attempt < 2:
                    time.sleep(0.5)
        raise RuntimeError(f"ACP completion delivery failed: {error}")

    # ── async motion thread ──────────────────────────────────────────

    def _run_motion(self, instance_id: tuple[str, str], active: dict) -> None:
        action = active["action"]
        ctrl = active["controller"]
        try:
            with ctrl._motion_lock:
                if ctrl._stop_evt.is_set():
                    raise InterruptedError("hand motion stopped")
                verify_hand_connection(ctrl.get())
                client = ctrl.get()
                target_vec = ctrl.target_for_action(
                    action, active.get("joint_positions"), client,
                )
                hold_torque = 0 if action == "reset" else active.get("hold_torque", 0)
                result = ctrl.move(
                    target_vec, action,
                    speed=active["speed"], torque=active["torque"],
                    hold_torque=hold_torque,
                )
            status = "error" if result["state"] == "released_timeout" else "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc)}
        except Exception as exc:
            status, result = "error", {"state": "error", "message": str(exc)}

        if status != "completed":
            with ctrl._motion_lock:
                if vars(ctrl).get("_last_command_owner") == active["owner"] and not vars(ctrl).get("_grasp_owner"):
                    result["torque_release_requested"] = True
                    try:
                        ctrl.get().set_currents([0] * 5)
                    except Exception:
                        result["torque_released"] = False
                    else:
                        result.update(torque=0, hold_torque=0, torque_released=True)
        terminal = {"action_id": active["action_id"], "status": status, "result": result}
        with self._lock:
            self._last_result[instance_id] = terminal
            ctrl._holding_torque = (active["hold_torque"] if status == "completed" else
                                    0 if result.get("torque_released") else
                                    max(vars(ctrl).get("_holding_torque", 0), active["hold_torque"]))
            if ctrl._holding_torque:
                self._held[instance_id] = active
            elif self._held.get(instance_id, {}).get("owner") == active["owner"]:
                self._held.pop(instance_id, None)
        release_motion(ctrl, active["owner"])
        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            with self._lock:
                self._delivery_failed[instance_id] = True
                terminal["callback_error"] = str(exc)
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)
        else:
            with self._lock:
                if self._active.get(instance_id) is active:
                    self._active.pop(instance_id, None)

    def _start_motion(self, instance_id: str, action: str, args: dict) -> dict:
        side = validate_bound_side(args.get("bound_side"))
        channel = self.config.get("can_channel", "can0")
        key = (side, channel)
        speed, torque = self._motion_options(action, args)
        hold_torque = 0 if action == "reset" else validate_hold_torque(args.get("hold_torque"), default=0)
        active: dict = {
            "action": action,
            "action_id": f"hand_{action}_{uuid4().hex}",
            "speed": speed, "torque": torque,
            "hold_torque": hold_torque,
            "instance_id": instance_id,
        }
        if action == "set_positions":
            joint_positions: dict[str, int] = {}
            for field in STRUCTURED_POSITION_FIELDS:
                val = args.get(field)
                if val is not None and val != "":
                    if isinstance(val, bool) or not isinstance(val, int) or not 0 <= val <= 255:
                        raise ValueError(f"{field} must be an integer in 0..255")
                    joint_positions[field] = val
            if not joint_positions:
                raise ValueError("at least one joint position must be specified")
            active["joint_positions"] = joint_positions

        with self._lock:
            if self._delivery_failed.get(key):
                raise RuntimeError("ACP completion delivery failed; restart driver after checking Core")
            if key in self._active:
                raise RuntimeError("该手已有基础控制动作正在执行")
            ctrl = self._get_controller(side, channel)
            active["owner"] = (id(self), active["action_id"])
            claim_motion(ctrl, active["owner"], config=self.config)
            try:
                verify_hand_connection(ctrl.get())
            except Exception:
                release_motion(ctrl, active["owner"], restore_previous=True)
                raise
            ctrl._stop_evt.clear()
            active["controller"] = ctrl
            previous_hold = self._held.pop(key, None)
            self._active[key] = active

        try:
            threading.Thread(
                target=self._run_motion, args=(key, active),
                name=active["action_id"], daemon=True,
            ).start()
        except Exception:
            with self._lock:
                self._active.pop(key, None)
                if previous_hold is not None:
                    self._held[key] = previous_hold
            release_motion(ctrl, active["owner"], restore_previous=True)
            raise
        return {"state": "running", "action_id": active["action_id"], "bound_side": side,
                "can_channel": channel}

    # ── dispatch ─────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id") or ""

        if action == "config":
            channel = args.get("can_channel", self.config.get("can_channel", "can0"))
            if not isinstance(channel, str) or not channel or channel.strip() != channel or any(c.isspace() or c == "/" for c in channel):
                raise ValueError("can_channel 必须是有效的 CAN 接口名称，例如 can0")
            with GLOBAL_BUS_LOCK:
                if channel != self.config.get("can_channel", "can0"):
                    controllers = [*GLOBAL_CONTROLLERS.values(), *self._controllers.values()]
                    if any(vars(ctrl).get("_command_owner") is not None
                           or vars(ctrl).get("_grasp_owner") is not None
                           or vars(ctrl).get("_holding_torque", 0)
                           for ctrl in controllers):
                        raise RuntimeError("动作或保持尚未结束，请先停止／释放，再修改执行器共享 CAN 接口")
                    self.config["can_channel"] = channel
            return {"state": "configured", "can_channel": channel, "scope": "shared",
                    "binding_saved": False}

        if action == "start":
            return {"state": "ready", "target_required": True}

        if action == "stop":
            if "bound_side" in args:
                side = validate_bound_side(args["bound_side"])
                targets = [(side, self.config.get("can_channel", "can0"))]
            else:
                with self._lock:
                    targets = [key for key, task in {**self._held, **self._active}.items()
                               if instance_id and task.get("instance_id") == instance_id]
            stopped = []
            unloaded = False
            for key in targets:
                with self._lock:
                    task = self._active.get(key) or self._held.get(key)
                    if not task or ("bound_side" not in args and task.get("instance_id") != instance_id):
                        continue
                    ctrl = task["controller"]
                    if vars(ctrl).get("_last_command_owner") != task["owner"]:
                        continue
                    ctrl._stop_evt.set()
                with ctrl._motion_lock:
                    if (vars(ctrl).get("_last_command_owner") == task["owner"]
                            and not vars(ctrl).get("_grasp_owner")):
                        ctrl.release_torque()
                        ctrl._holding_torque = 0
                        unloaded = True
                with self._lock:
                    if self._held.get(key) is task:
                        self._held.pop(key, None)
                stopped.append(key[0])
            return {"state": "idle", "stop_status": "stopped" if stopped else "unmatched",
                    "target_resolved": bool(stopped), "stopped_sides": stopped,
                    "torque_released": unloaded,
                    "message": "已停止匹配的基础控制任务" if stopped else "没有匹配的基础控制任务；匿名动作请明确选择左右手停止"}

        if action == "info":
            side = validate_bound_side(args.get("bound_side"))
            channel = self.config.get("can_channel", "can0")
            key = (side, channel)
            found = self._discover()
            with self._lock:
                active_id = (self._active.get(key) or {}).get("action_id")
                last = self._last_result.get(key)
            return {
                "state": "connected" if found.get(side) == channel else "offline",
                "bound_side": side,
                "can_channel": channel,
                "detected_sides": list(found),
                "detected_channels": found,
                "active_action_id": active_id,
                "last_result": last,
                "active_joint_count": ACTIVE_JOINT_COUNT,
                "joint_names": JOINT_NAMES,
            }

        if action == "get_state":
            ctrl = self._get_controller(validate_bound_side(args.get("bound_side")))
            with self._bus_lock:
                client = ctrl.get()
                positions = client.get_positions()
                speeds = client.get_speeds()
                faults = client.get_faults()
            return {
                "state": "connected", "side": ctrl.side,
                "can_channel": ctrl.channel,
                "positions": positions,
                "speeds": speeds,
                "faults": faults,
                "timestamp_ms": int(time.time() * 1000),
            }

        if action in MOTION_ACTIONS:
            if args.get("confirm_motion") is not True:
                raise ValueError("confirm_motion must be true for every movement request")
            return self._start_motion(instance_id, action, args)

        return None


# ──────────────────────────────────────────────────────────────────────
# Plugin builder (called by vendor_runtime)
# ──────────────────────────────────────────────────────────────────────

def build_plugins(config: dict, _namespace: str, _ros2) -> list:
    plugins: list = []
    hand_config = config.get("plugins", {}).get("hand", {})
    hand_plugin: HandPlugin | None = None

    if hand_config.get("enabled", True):
        hand_plugin = HandPlugin(config, hand_config)
        plugins.append(hand_plugin)

    gesture_config = config.get("plugins", {}).get("hand_gesture", {})
    if gesture_config.get("enabled", True):
        from gesture_device import GesturePlugin
        plugins.append(GesturePlugin(hand_plugin, gesture_config, global_config=config))

    grasp_config = config.get("plugins", {}).get("hand_grasp", {})
    if grasp_config.get("enabled", True):
        from grasp_device import GraspPlugin
        plugins.append(GraspPlugin(hand_plugin, grasp_config, global_config=config))

    tactile_config = config.get("plugins", {}).get("hand_tactile")
    if tactile_config and tactile_config.get("enabled", True):
        from tactile_device import TactilePlugin
        plugins.append(TactilePlugin(hand_plugin, tactile_config, global_config=config, ros2=_ros2, namespace=_namespace))

    return plugins
