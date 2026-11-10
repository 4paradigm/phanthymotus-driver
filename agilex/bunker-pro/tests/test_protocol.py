import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import (
    CONTROL_CAN,
    CONTROL_MODE_CONFIG_ID,
    MOTION_COMMAND_ID,
    decode_motion_state,
    decode_odometry,
    decode_system_state,
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
    interface = build_interface(provides=["vx", "wz"], rate_hz=10, pose_drift="none")
    sample = build_sample(stamp_ms=1234,
        twist=[0.2, None, None, None, None, -0.1], frame="body", pose=None)
    assert parse_interface(interface).provides == ("vx", "wz")
    assert sample["twist"] == [0.2, None, None, None, None, -0.1]


def test_control_mode_frame_matches_official_v2_layout():
    assert CONTROL_MODE_CONFIG_ID == 0x421
    assert encode_control_mode(CONTROL_CAN) == bytes([1, 0, 0, 0, 0, 0, 0, 0])


def test_motion_command_uses_signed_big_endian_milli_units():
    payload = encode_motion_command(0.3, -0.42)
    assert MOTION_COMMAND_ID == 0x111
    assert payload == bytes.fromhex("012cfe5c00000000")
    assert decode_motion_state(payload)["linear_velocity_m_s"] == 0.3
    assert decode_motion_state(payload)["angular_velocity_rad_s"] == -0.42


def test_state_feedback_uses_sdk_mode_values_and_voltage_scale():
    state = decode_system_state(bytes.fromhex("000101fe00000000"))
    assert state == {
        "vehicle_state": 0,
        "control_mode": 1,
        "battery_voltage_v": 51.0,
        "error_code": 0,
    }


def test_motion_feedback_decodes_all_four_fields():
    state = decode_motion_state(bytes.fromhex("03e8ff3801f4ff9c"))
    assert state == {
        "linear_velocity_m_s": 1.0,
        "angular_velocity_rad_s": -0.2,
        "lateral_velocity_m_s": 0.5,
        "steering_angle_rad": -0.1,
    }


def test_wheel_odometry_is_kept_as_raw_signed_counts():
    assert decode_odometry(bytes.fromhex("ffffffff00000002")) == {
        "left_wheel_raw": -1,
        "right_wheel_raw": 2,
    }
