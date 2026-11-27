"""Pure-function contract for the visual_facing yaw loop and its safety gates."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

BUMI = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, BUMI)

import visual_facing_spec as spec  # noqa: E402


CFG = spec.validate_config({})


def test_a_target_on_the_right_asks_for_a_right_turn():
    # Optical x is right-positive; body yaw is atan2(-x, z), matching
    # loco_relative_move's positive-z-is-left convention.
    error = spec.yaw_error_deg((0.2, 0.0, 1.0), CFG)
    assert error == pytest.approx(-math.degrees(math.atan2(0.2, 1.0)), abs=1e-9)


def test_a_target_on_the_left_asks_for_a_left_turn():
    assert spec.yaw_error_deg((-0.2, 0.0, 1.0), CFG) > 0.0


def test_a_centred_target_asks_for_no_turn():
    assert spec.yaw_error_deg((0.0, 0.0, 1.0), CFG) == 0.0


def test_a_target_behind_the_lens_is_rejected_not_turned_into_a_u_turn():
    with pytest.raises(ValueError, match="in front"):
        spec.yaw_error_deg((0.0, 0.0, -0.5), CFG)


def test_the_resting_state_ignores_the_hysteresis_band():
    # 8 degrees is outside the 5-degree stop tolerance but inside the
    # 12-degree restart tolerance: a stopped robot must stay stopped.
    assert spec.control_for_error(8.0, False, CFG)["mode"] == "hold"


def test_an_already_turning_robot_keeps_creeping_inside_the_band():
    fine = spec.control_for_error(8.0, True, CFG)
    assert fine["mode"] == "fine"
    assert fine["z"] == pytest.approx(CFG["min_z"])  # above stiction
    assert fine["duty"] < 1.0


def test_a_centred_sample_inside_tolerance_finishes_the_turn():
    assert spec.control_for_error(3.0, True, CFG) == {
        "z": 0.0, "duty": 0.0, "mode": "settle"}


def test_a_large_error_gets_a_bounded_normalised_yaw_velocity():
    coarse = spec.control_for_error(30.0, False, CFG)
    assert coarse["mode"] == "coarse"
    assert coarse["z"] == pytest.approx(min(CFG["max_z"], CFG["kp"] * math.radians(30.0)))
    assert CFG["min_z"] <= coarse["z"] <= CFG["max_z"]


def test_a_duty_cycle_never_sends_a_below_stiction_velocity():
    z, acc = spec.pulse_velocity(0.45, 0.4, 0.0)
    assert (z, acc) == (0.0, pytest.approx(0.4))
    z, acc = spec.pulse_velocity(0.45, 0.4, acc)
    assert (z, acc) == (0.0, pytest.approx(0.8))
    z, acc = spec.pulse_velocity(0.45, 0.4, acc)
    assert (z, acc) == (0.45, pytest.approx(0.2))
    assert spec.pulse_velocity(0.45, 0.0, 0.0) == (0.0, 0.0)


def test_an_expired_observation_is_not_a_live_target():
    out = spec.select_target(
        {"tvec": [0.1, 0.0, 1.0], "stamp_ms": 1000}, now_ms=2000, cfg=CFG)
    assert out["error"] == "no_fresh_target"


def test_a_future_timestamp_is_rejected_rather_than_believed():
    out = spec.select_target(
        {"tvec": [0.1, 0.0, 1.0], "stamp_ms": 5000}, now_ms=1000, cfg=CFG)
    assert out["error"] == "no_fresh_target"


def test_a_missing_tvec_is_rejected_instead_of_zero_filled():
    out = spec.select_target({"position": None}, now_ms=1000, cfg=CFG)
    assert out["error"] == "no_fresh_target"
    assert out["rejected"] == 1


def test_millimetres_are_converted_only_when_declared():
    cfg = spec.validate_config({"units": "mm"})
    out = spec.select_target({"tvec": [200, 0, 1000], "stamp_ms": 1000},
                             now_ms=1000, cfg=cfg)
    assert out["point"] == pytest.approx((0.2, 0.0, 1.0))
    assert spec.yaw_error_deg(out["point"], cfg) < 0.0


def test_two_markers_without_a_selection_are_ambiguous_not_arbitrary():
    payload = {"targets": [
        {"marker_id": 17, "tvec": [0.1, 0, 1], "stamp_ms": 1000},
        {"marker_id": 23, "tvec": [-0.1, 0, 1], "stamp_ms": 1000},
    ]}
    assert spec.select_target(payload, 1000, CFG)["error"] == "ambiguous_targets"
    cfg = spec.validate_config({"target_id": "23"})
    assert spec.select_target(payload, 1000, cfg)["target_id"] == "23"


def test_enable_requires_an_explicit_confirmation():
    with pytest.raises(ValueError, match="confirm"):
        spec.arm_authorization(1000.0, 8.0, False)
    deadline = spec.arm_authorization(1000.0, 8.0, True)
    assert deadline == 9000.0
    assert spec.authorization_remaining_ms(5000.0, deadline) == 4000.0
    assert spec.authorization_remaining_ms(10000.0, deadline) == 0.0


def test_protection_mode_estop_expiry_and_target_loss_all_stop():
    good = dict(mode=2, estop_state="clear", auth_reason="", target_error="")
    assert spec.stop_reason(**good) == ""
    assert spec.stop_reason(**{**good, "mode": 26}) == "protection_mode"
    assert spec.stop_reason(**{**good, "mode": 30}) == "not_walking"
    assert spec.stop_reason(**{**good, "estop_state": "latched"}) == "estop"
    assert spec.stop_reason(**{**good, "estop_state": "unknown",
                               "require_estop": True}) == "estop_unknown"
    assert spec.stop_reason(**{**good, "auth_reason": "authorization_expired"}) == \
        "authorization_expired"
    assert spec.stop_reason(**{**good, "target_error": "no_fresh_target"}) == \
        "no_fresh_target"


def test_a_turn_budget_stops_a_robot_that_never_reaches_centre():
    assert spec.stop_reason(2, "clear", "", "", turn_elapsed_s=13.0, cfg=CFG) == \
        "turn_timeout"


def test_config_rejects_bad_ranges_instead_of_clamping():
    with pytest.raises(ValueError, match="exit_tolerance"):
        spec.validate_config({"center_tolerance_deg": 10, "exit_tolerance_deg": 8})
    with pytest.raises(ValueError, match="min_z"):
        spec.validate_config({"min_z": 0.2})
    with pytest.raises(ValueError, match="yaw_sign"):
        spec.validate_config({"yaw_sign": 0})


def test_status_payload_stays_flat_for_the_dashboard():
    payload = spec.status_payload(state="turning", now_ms=1234.0)
    assert payload["stamp_ms"] == 1234
    assert all(not isinstance(value, (dict, list, tuple))
               for value in payload.values())
