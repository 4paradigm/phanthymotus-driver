"""Pure Bunker Mini v2 SocketCAN frame codec, checked against AgileX ugv_sdk."""

from __future__ import annotations

import math

MOTION_COMMAND_ID = 0x111
SYSTEM_STATE_ID = 0x211
MOTION_STATE_ID = 0x221
ODOMETRY_ID = 0x311
REMOTE_CONTROLLER_ID = 0x241
MOTOR_HIGH_STATE_IDS = (0x252, 0x253)
MOTOR_LOW_STATE_IDS = (0x262, 0x263)
BATTERY_STATE_ID = 0x361
BATTERY_ALARM_ID = 0x362
CONTROL_MODE_CONFIG_ID = 0x421

VEHICLE_NORMAL = 0x00
VEHICLE_ESTOP = 0x01
VEHICLE_EXCEPTION = 0x02

CONTROL_STANDBY = 0x00
CONTROL_CAN = 0x01
CONTROL_RC = 0x03


def _require_dlc(data: bytes, length: int, name: str) -> None:
    if len(data) != length:
        raise ValueError(f"{name} requires {length} bytes, received {len(data)}")


def _int16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "big", signed=True)


def decode_system_state(data: bytes) -> dict:
    _require_dlc(data, 8, "system state")
    return {
        "vehicle_state": data[0],
        "control_mode": data[1],
        "battery_voltage_v": int.from_bytes(data[2:4], "big") * 0.1,
        "error_flags": data[5],
        "counter": data[7],
    }


def decode_motion_state(data: bytes) -> dict:
    _require_dlc(data, 8, "motion state")
    return {
        "linear_velocity_m_s": _int16(data, 0) / 1000.0,
        "steering_angle_deg": _int16(data, 2) / 100.0,
    }


def decode_odometry(data: bytes) -> dict:
    _require_dlc(data, 8, "wheel odometry")
    return {
        "left_wheel_mm": int.from_bytes(data[0:4], "big", signed=True),
        "right_wheel_mm": int.from_bytes(data[4:8], "big", signed=True),
    }


def decode_motor_high_state(data: bytes) -> dict:
    _require_dlc(data, 8, "motor high-speed state")
    return {
        "speed_rpm": _int16(data, 0),
        "current_a": _int16(data, 2) / 10.0,
        "position_pulses": int.from_bytes(data[4:8], "big", signed=True),
    }


def decode_motor_low_state(data: bytes) -> dict:
    _require_dlc(data, 8, "motor low-speed state")
    return {
        "driver_voltage_v": int.from_bytes(data[0:2], "big") / 10.0,
        "driver_temperature_c": _int16(data, 2),
        "motor_temperature_c": int.from_bytes(data[4:5], "big", signed=True),
        "status_flags": data[5],
    }


def decode_remote_controller(data: bytes) -> dict:
    _require_dlc(data, 8, "remote controller")
    switches = data[0]
    return {
        "swa": switches & 0x03,
        "swb": (switches >> 2) & 0x03,
        "swc": (switches >> 4) & 0x03,
        "swd": (switches >> 6) & 0x03,
        "right_stick_x": int.from_bytes(data[1:2], "big", signed=True),
        "right_stick_y": int.from_bytes(data[2:3], "big", signed=True),
        "left_stick_y": int.from_bytes(data[3:4], "big", signed=True),
        "left_stick_x": int.from_bytes(data[4:5], "big", signed=True),
        "vra": int.from_bytes(data[5:6], "big", signed=True),
        "counter": data[7],
    }


def decode_battery_state(data: bytes) -> dict:
    _require_dlc(data, 8, "battery BMS state")
    return {
        "soc_percent": data[0],
        "soh_percent": data[1],
        "voltage_v": int.from_bytes(data[2:4], "big") / 100.0,
        "current_a": _int16(data, 4) / 10.0,
        "temperature_c": _int16(data, 6) / 10.0,
    }


def decode_battery_alarms(data: bytes) -> dict:
    _require_dlc(data, 4, "battery BMS alarms")
    return {
        "alarm_status_1": data[0],
        "alarm_status_2": data[1],
        "warning_status_1": data[2],
        "warning_status_2": data[3],
    }


def _encode_velocity(value: float, name: str) -> bytes:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    raw = int(float(value) * 1000.0)
    if raw < -32768 or raw > 32767:
        raise ValueError(f"{name} exceeds the CAN int16 range")
    return raw.to_bytes(2, "big", signed=True)


def encode_motion_command(linear_m_s: float, angular_rad_s: float,
                          lateral_m_s: float = 0.0,
                          steering_rad: float = 0.0) -> bytes:
    """Encode the SDK's 0x111 linear/angular/lateral/steering vector."""
    return b"".join((
        _encode_velocity(linear_m_s, "linear_m_s"),
        _encode_velocity(angular_rad_s, "angular_rad_s"),
        _encode_velocity(lateral_m_s, "lateral_m_s"),
        _encode_velocity(steering_rad, "steering_rad"),
    ))


def encode_control_mode(mode: int = CONTROL_CAN) -> bytes:
    if mode not in (CONTROL_STANDBY, CONTROL_CAN):
        raise ValueError(f"unsupported control mode: {mode}")
    return bytes([mode])
