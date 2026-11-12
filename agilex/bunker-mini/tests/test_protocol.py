import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import (
    CONTROL_CAN,
    CONTROL_MODE_CONFIG_ID,
    MOTION_COMMAND_ID,
    REMOTE_CONTROLLER_ID,
    MOTOR_HIGH_STATE_IDS,
    MOTOR_LOW_STATE_IDS,
    BATTERY_STATE_ID,
    BATTERY_ALARM_ID,
    decode_motion_state,
    decode_odometry,
    decode_system_state,
    decode_motor_high_state,
    decode_motor_low_state,
    decode_remote_controller,
    decode_battery_state,
    decode_battery_alarms,
    encode_control_mode,
    encode_motion_command,
)
from common.control.descriptor import parse_descriptor
from common.odom import build_interface, build_sample, parse_interface


def test_twist_descriptor_rejects_unsupported_axes_and_declares_watchdog():
    descriptor = {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-0.2, 0, 0, 0, 0, -0.4],
                    "upper": [0.2, 0, 0, 0, 0, 0.4],
                    "max_delta_per_step": [0.05, 1e-6, 1e-6, 1e-6, 1e-6, 0.1]},
        "frame": "base_link", "rate": {"max_hz": 50, "expected_hz": 20,
            "watchdog_ms": 350, "max_obs_age_ms": 1000}, "force_torque": None,
    }
    parsed = parse_descriptor(descriptor)
    assert parsed.watchdog_ms == 350
    assert parsed.max_velocity is None
    assert parsed.lower[1:5] == (0, 0, 0, 0)


def test_odom_reports_only_measured_axes():
    interface = build_interface(provides=["vx"], rate_hz=10, pose_drift="none")
    sample = build_sample(stamp_ms=1234,
        twist=[0.2, None, None, None, None, None], frame="body", pose=None)
    assert parse_interface(interface).provides == ("vx",)
    assert sample["twist"] == [0.2, None, None, None, None, None]


def test_motion_command_uses_signed_big_endian_milli_units():
    payload = encode_motion_command(0.3, -0.42)
    assert MOTION_COMMAND_ID == 0x111
    assert payload == bytes.fromhex("012cfe5c00000000")


def test_state_feedback_uses_sdk_mode_values_and_voltage_scale():
    state = decode_system_state(bytes.fromhex("000101fe00000000"))
    assert state == {
        "vehicle_state": 0,
        "control_mode": 1,
        "battery_voltage_v": 51.0,
        "error_flags": 0,
        "counter": 0,
    }


def test_motion_feedback_decodes_linear_speed_and_steering_angle():
    state = decode_motion_state(bytes.fromhex("03e8ff3800000000"))
    assert state == {
        "linear_velocity_m_s": 1.0,
        "steering_angle_deg": -2.0,
    }


def test_wheel_odometry_is_kept_as_raw_signed_counts():
    assert decode_odometry(bytes.fromhex("ffffffff00000002")) == {
        "left_wheel_mm": -1,
        "right_wheel_mm": 2,
    }


def test_control_mode_payload_matches_documented_one_byte_frame():
    assert CONTROL_MODE_CONFIG_ID == 0x421
    assert encode_control_mode(CONTROL_CAN) == b"\x01"


def test_motor_feedback_frames_decode_documented_units():
    assert MOTOR_HIGH_STATE_IDS == (0x252, 0x253)
    assert decode_motor_high_state(bytes.fromhex("0064ffec00000002")) == {
        "speed_rpm": 100, "current_a": -2.0, "position_pulses": 2}
    assert MOTOR_LOW_STATE_IDS == (0x262, 0x263)
    assert decode_motor_low_state(bytes.fromhex("01fe001efd810000")) == {
        "driver_voltage_v": 51.0, "driver_temperature_c": 30,
        "motor_temperature_c": -3, "status_flags": 0x81}


def test_remote_controller_switches_sticks_and_counter():
    assert REMOTE_CONTROLLER_ID == 0x241
    assert decode_remote_controller(bytes.fromhex("e701fe7f8001002a")) == {
        "swa": 3, "swb": 1, "swc": 2, "swd": 3,
        "right_stick_x": 1, "right_stick_y": -2, "left_stick_y": 127,
        "left_stick_x": -128, "vra": 1, "counter": 42}


def test_battery_bms_state_and_alarm_frames():
    assert BATTERY_STATE_ID == 0x361
    assert decode_battery_state(bytes.fromhex("5a550edcfff600fa")) == {
        "soc_percent": 90, "soh_percent": 85, "voltage_v": 38.04,
        "current_a": -1.0, "temperature_c": 25.0}
    assert BATTERY_ALARM_ID == 0x362
    assert decode_battery_alarms(bytes.fromhex("04810401")) == {
        "alarm_status_1": 4, "alarm_status_2": 129,
        "warning_status_1": 4, "warning_status_2": 1}
