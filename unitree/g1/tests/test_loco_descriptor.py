"""Offline descriptor validation only; example values are not G1 parameters."""
import copy

import pytest

from common.control.descriptor import DescriptorError
from loco_descriptor import validate_loco_descriptor


@pytest.fixture
def example():
    # Invented test-only numbers, not specifications or suitable for execution.
    return {
        "control_interface": "motus.control/1",
        "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "frame": "base_link",
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-1, -2, 0, 0, 0, -3],
                   "upper": [1, 2, 0, 0, 0, 3],
                   "max_delta_per_step": [0.1] * 6},
        "rate": {"max_hz": 20, "expected_hz": 10, "watchdog_ms": 100,
                 "max_obs_age_ms": 200},
        "force_torque": None,
    }


def test_explicit_standard_descriptor_parses_without_mutating_input(example):
    original = copy.deepcopy(example)
    result = validate_loco_descriptor(example)
    assert example == original
    assert result.mode == "twist" and result.dof == 6
    assert result.joint_names == tuple(example["joint_names"])
    assert result.frame == "base_link"
    example["limits"]["upper"][0] = 999
    assert result.upper[0] == 1
    assert result.raw["limits"]["upper"][0] == 1


@pytest.mark.parametrize("key,value", [
    ("control_interface", "other"), ("mode", "joint_velocity"), ("dof", 3),
    ("dof", True), ("joint_names", ["vy", "vx", "vz", "wx", "wy", "wz"]),
    ("joint_names", None), ("frame", None), ("frame", "map"), ("frame", 1),
    ("units", {"linear": "m/s", "angular": "deg/s", "time": "s"}),
])
def test_schema_axes_frame_and_units_fail_closed(example, key, value):
    example[key] = value
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)


@pytest.mark.parametrize("name,index,value", [
    ("lower", 0, float("nan")), ("upper", 1, float("inf")),
    ("lower", 0, 0), ("upper", 5, 0), ("lower", 2, -0.01),
    ("upper", 3, 0.01), ("upper", 4, float("nan")),
    ("max_delta_per_step", 1, 0), ("max_delta_per_step", 4, float("inf")),
    ("max_delta_per_step", 2, True),
])
def test_invalid_bounds_and_step_limits_rejected(example, name, index, value):
    example["limits"][name][index] = value
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)


@pytest.mark.parametrize("field,value", [
    ("max_hz", float("nan")), ("expected_hz", float("inf")),
    ("expected_hz", 21), ("max_hz", 0), ("watchdog_ms", 0.5),
    ("watchdog_ms", float("inf")), ("max_obs_age_ms", 0),
    ("max_obs_age_ms", True),
])
def test_rate_must_be_explicit_positive_finite_and_consistent(example, field, value):
    example["rate"][field] = value
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)


@pytest.mark.parametrize("section,key", [
    (None, "control_interface"), (None, "frame"), (None, "joint_names"),
    (None, "force_torque"), ("limits", "max_delta_per_step"),
    ("limits", "lower"), ("rate", "watchdog_ms"), ("rate", "max_hz"),
    ("rate", "expected_hz"), ("rate", "max_obs_age_ms"),
])
def test_missing_required_data_is_not_defaulted(example, section, key):
    (example if section is None else example[section]).pop(key)
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)


def test_optional_standard_fields_validate_and_nonfinite_force_fails(example):
    example["limits"]["max_velocity"] = [1] * 6
    example["limits"]["min_magnitude"] = [0.1, 0.1, 0, 0, 0, 0.1]
    example["force_torque"] = [2, 3]
    assert validate_loco_descriptor(example).force_torque == (2, 3)
    example["force_torque"][0] = float("nan")
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)


def test_advisory_or_mixed_mode_group_rejected(example):
    example["groups"] = [{"name": "base", "offset": 0, "count": 6,
                          "advisory": True}]
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)
    example["groups"][0]["advisory"] = False
    example["groups"][0]["mode"] = "joint_velocity"
    with pytest.raises(DescriptorError):
        validate_loco_descriptor(example)
    example["groups"][0]["mode"] = "twist"
    assert validate_loco_descriptor(example).groups[0].mode == "twist"
