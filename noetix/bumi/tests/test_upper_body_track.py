"""The upper-body retarget's rules, exercised off a robot.

`upper_body_track_spec` imports no ROS, no OpenCV and no numpy, so these run on
a laptop with nothing installed. What is worth pinning down is not that the
dictionary has the expected keys, but the decisions that would fail silently:

  * a sideways arm must not read as "arm swung behind the robot";
  * the two arms have asymmetric roll limits, so a clamp that looks right on one
    side is wrong on the other;
  * the dashboard renders a nested payload as raw JSON text, so flatness is a
    contract, not a style choice;
  * a joint disappears when the operator turns away — it must hold, not jump.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import upper_body_track_spec as spec  # noqa: E402


def _person(keypoints, confidence=0.9, bbox=(10.0, 10.0, 110.0, 210.0)):
    return {"keypoints": keypoints, "confidence": confidence, "bbox": list(bbox)}


def _blank_points():
    """17 keypoints that are confidently on-screen but carry no geometry."""
    return [[100.0, 100.0, 1.0] for _ in range(17)]


# ── the retarget itself ──────────────────────────────────────────────────────

def test_an_arm_hanging_straight_down_is_all_zeros():
    angles = spec.arm_angles((0, 0, 1.0), (0, 0, 0.7), (0, 0, 0.4))
    assert angles["pitch_deg"] == pytest.approx(0.0, abs=1e-6)
    assert angles["roll_deg"] == pytest.approx(0.0, abs=1e-6)
    assert angles["elbow_deg"] == pytest.approx(0.0, abs=1e-6)


def test_an_arm_reaching_forward_is_ninety_degrees_of_pitch():
    angles = spec.arm_angles((0, 0, 1.0), (0.3, 0, 1.0), (0.6, 0, 1.0))
    assert angles["pitch_deg"] == pytest.approx(-90.0, abs=1e-6)
    assert angles["roll_deg"] == pytest.approx(0.0, abs=1e-6)


def test_a_sideways_arm_is_pure_abduction_not_a_hundred_and_eighty():
    """Regression: `atan2(0, -0.0)` answers 180 here, which clamps to "behind"."""
    angles = spec.arm_angles((0, 0, 1.0), (0, -0.3, 1.0), (0, -0.6, 1.0))
    assert angles["roll_deg"] == pytest.approx(90.0, abs=1e-6)
    assert angles["pitch_deg"] == pytest.approx(0.0, abs=1e-6)


def test_a_bent_elbow_reads_as_negative_flexion():
    angles = spec.arm_angles((0, 0, 1.0), (0.3, 0, 1.0), (0.3, 0, 1.3))
    assert angles["elbow_deg"] == pytest.approx(-90.0, abs=1e-6)


def test_a_straight_up_arm_asks_for_more_than_the_shoulder_has():
    """Honest answer: 180 degrees of flexion, which the 135 limit must then cap.

    Which way round the 180 comes out depends on the sign of zero in `atan2`;
    both readings mean "as far as it goes", so only the magnitude is pinned.
    """
    angles = spec.arm_angles((0, 0, 1.0), (0, 0, 1.3), (0, 0, 1.6))
    limited, clamped = spec.limit_angles(angles, "left")
    assert clamped["pitch"] == 1
    assert abs(limited["pitch_deg"]) == pytest.approx(135.0, abs=1e-6)


def test_coincident_shoulder_and_elbow_is_an_error_not_a_zero():
    with pytest.raises(ValueError):
        spec.arm_angles((0, 0, 1.0), (0, 0, 1.0), (0, 0, 0.7))


# ── limits ───────────────────────────────────────────────────────────────────

def test_the_two_arms_have_mirrored_roll_limits():
    """-8 on the left is a clamp; -8 on the right is a legal angle."""
    left, left_clamped = spec.limit_angles(
        {"pitch_deg": 0.0, "roll_deg": -30.0, "elbow_deg": 0.0}, "left")
    right, right_clamped = spec.limit_angles(
        {"pitch_deg": 0.0, "roll_deg": -30.0, "elbow_deg": 0.0}, "right")

    assert left["roll_deg"] == pytest.approx(-8.0)
    assert left_clamped["roll"] == 1
    assert right["roll_deg"] == pytest.approx(-30.0)
    assert right_clamped["roll"] == 0


def test_an_angle_inside_the_range_is_reported_as_unclamped():
    limited, clamped = spec.limit_angles(
        {"pitch_deg": 10.0, "roll_deg": 20.0, "elbow_deg": -30.0}, "left")
    assert clamped == {"pitch": 0, "roll": 0, "elbow": 0}
    assert limited["pitch_deg"] == pytest.approx(10.0)


def test_signs_flip_every_axis_independently():
    angles = {"pitch_deg": 10.0, "roll_deg": 20.0, "elbow_deg": -30.0}
    flipped = spec.apply_signs(angles, pitch_sign=-1.0, roll_sign=1.0, elbow_sign=-1.0)
    assert flipped == {"pitch_deg": -10.0, "roll_deg": 20.0, "elbow_deg": 30.0}


# ── projection ───────────────────────────────────────────────────────────────

def test_a_point_straight_ahead_is_straight_ahead_in_the_body_frame():
    point = spec.backproject(320, 240, 2.0, 460.0, 640, 480)
    assert point == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)


def test_a_point_right_of_centre_lands_on_the_robot_right():
    point = spec.backproject(640, 240, 2.0, 460.0, 640, 480)
    assert point[1] < 0.0


def test_a_point_below_centre_lands_below_the_robot_origin():
    point = spec.backproject(320, 480, 2.0, 460.0, 640, 480)
    assert point[2] < 0.0


def test_zero_depth_is_an_absent_measurement_not_a_point_at_the_origin():
    assert spec.backproject(320, 240, 0.0, 460.0, 640, 480) is None


def test_the_median_ignores_the_zeros_realsense_writes_for_no_return():
    assert spec.median([0.0, 0.0, 2.0, 2.2, 1.8]) == pytest.approx(2.0)


def test_a_patch_with_no_usable_echo_has_no_depth():
    assert spec.median([0.0, 0.0, None]) is None


def test_focal_length_falls_out_of_shoulder_width_and_depth():
    focal = spec.estimate_focal_px(120.0, 2.0, 0.38)
    assert focal == pytest.approx(120.0 * 2.0 / 0.38)


# ── selection ────────────────────────────────────────────────────────────────

def test_the_nearest_person_wins_over_the_first_one_listed():
    near = _person(_blank_points(), bbox=(0.0, 0.0, 200.0, 300.0))
    far = _person(_blank_points(), bbox=(0.0, 0.0, 50.0, 80.0))
    assert spec.pick_person([far, near]) is near


def test_a_low_confidence_detection_is_not_a_person_to_mirror():
    weak = _person(_blank_points(), confidence=0.2)
    assert spec.pick_person([weak], min_confidence=0.5) is None


def test_an_occluded_arm_is_detected_as_not_visible():
    points = _blank_points()
    points[spec.R_WRIST] = [0.0, 0.0, 0.0]
    assert not spec.upper_body_visible(points, (spec.R_SHOULDER, spec.R_ELBOW, spec.R_WRIST), 0.4)
    assert spec.upper_body_visible(points, (spec.L_SHOULDER, spec.L_ELBOW, spec.L_WRIST), 0.4)


def test_hips_below_shoulders_reads_as_standing():
    points = _blank_points()
    points[spec.L_SHOULDER] = [100.0, 100.0, 1.0]
    points[spec.R_SHOULDER] = [150.0, 100.0, 1.0]
    points[spec.L_HIP] = [100.0, 200.0, 1.0]
    points[spec.R_HIP] = [150.0, 200.0, 1.0]
    assert spec.posture_name(points) == "standing"


# ── payloads ─────────────────────────────────────────────────────────────────

def test_the_angle_payload_is_flat_because_the_panel_prints_nested_objects():
    payload = spec.build_angles_payload(
        tracked=True, person_count=1, target_id=0,
        angles_by_side={
            "left": {"pitch_deg": 1.0, "roll_deg": 2.0, "elbow_deg": -3.0},
            "right": {"pitch_deg": 4.0, "roll_deg": 5.0, "elbow_deg": -6.0},
        },
        clamped_by_side={"left": {"roll": 1}, "right": {}},
        depth_used=True, depth_coverage=1.0, focal_px=460.0,
        latency_ms=12.5, frame_age_ms=30.0, reach=1.6,
        pitch_sign=1.0, roll_sign=1.0, elbow_sign=1.0, degraded="")

    for key, value in payload.items():
        assert isinstance(value, (int, float, str, bool)), f"{key} is not scalar"
    assert payload["l_roll_deg"] == pytest.approx(2.0)
    assert payload["l_roll_clamped"] == 1
    assert payload["r_elbow_deg"] == pytest.approx(-6.0)


def test_an_untracked_frame_still_answers_with_numbers_not_with_omissions():
    """A missing key reads as 'no data' on the panel; -999 reads as 'not measured'."""
    payload = spec.build_angles_payload(
        tracked=False, person_count=0, target_id=None,
        angles_by_side=None, clamped_by_side=None, depth_used=False,
        depth_coverage=0.0, focal_px=460.0, latency_ms=1.0, frame_age_ms=2.0,
        reach=None, pitch_sign=1.0, roll_sign=1.0, elbow_sign=1.0,
        degraded="no_person")

    assert payload["tracked"] is False
    assert payload["target_id"] == -1
    assert payload["l_pitch_deg"] == pytest.approx(-999.0)
    assert payload["degraded"] == "no_person"


def test_the_pose_payload_carries_the_image_size_so_the_overlay_does_not_drift():
    payload = spec.build_pose2d_payload([_person(_blank_points())], 640, 480)
    assert payload["image_size"] == [640, 480]
    assert payload["count"] == 1
    assert len(payload["keypoint_names"]) == 17
    assert payload["persons"][0]["keypoints"][spec.R_WRIST][:2] == [100.0, 100.0]


def test_nobody_in_frame_is_an_empty_list_not_a_missing_key():
    assert spec.build_pose2d_payload([], 640, 480)["persons"] == []


# ── smoothing ────────────────────────────────────────────────────────────────

def test_the_first_filtered_sample_passes_through_untouched():
    filt = spec.OneEuroFilter()
    assert filt(12.0, 0.0) == pytest.approx(12.0)


def test_smoothing_lags_a_step_but_does_not_ignore_it():
    filt = spec.OneEuroFilter(min_cutoff=1.0, beta=0.0)
    filt(0.0, 0.0)
    value = filt(30.0, 0.1)
    assert 0.0 < value < 30.0


def test_a_bank_holds_the_last_value_for_a_joint_that_disappeared():
    bank = spec.AngleBank()
    bank.update({"left": {"pitch_deg": 10.0, "roll_deg": 20.0, "elbow_deg": -30.0},
                 "right": {"pitch_deg": 0.0, "roll_deg": 0.0, "elbow_deg": 0.0}}, 0.0)
    held = bank.update({"left": {"pitch_deg": 11.0, "roll_deg": 21.0, "elbow_deg": -29.0}}, 0.1)
    assert held["r_pitch"] == pytest.approx(0.0)
    assert "l_pitch" in held


# ── the image must actually contain the module ───────────────────────────────

def test_both_modules_reach_the_image():
    """Local tests pass with a missing COPY line; the robot then loses the card."""
    dockerfile = (Path(__file__).resolve().parent.parent / "Dockerfile").read_text(encoding="utf-8")
    for name in ("upper_body_track_spec.py", "upper_body_track_card.py"):
        assert f"COPY {name} /work/{name}" in dockerfile
