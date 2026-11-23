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


def test_pose_runtime_is_an_opt_in_pinned_build_arg():
    """A default Bumi image must not silently carry torch for a disabled card."""
    dockerfile = (Path(__file__).resolve().parent.parent / "Dockerfile").read_text(encoding="utf-8")
    assert "ARG INSTALL_POSE_DEPS=0" in dockerfile
    assert 'ultralytics==' in dockerfile
    assert 'ultralytics &&' not in dockerfile  # no unpinned install remains


# ── end-to-end through the card's own `_measure` ──────────────────────────────

def _load_card_module(monkeypatch):
    """Import the card with its ROS/OpenCV/numpy dependencies stubbed out.

    `_measure` — where the payload is actually assembled — is a pure function
    of keypoints plus the AngleBank. The spec tests above cannot see a held
    angle being dropped at that layer, so this stub lets the regression run on
    a laptop without ROS, cv2 or numpy installed.
    """
    import importlib.util
    import sys
    import types

    cv2 = types.ModuleType("cv2")
    cv2.imdecode = lambda *a, **k: None
    cv2.IMREAD_COLOR = 1
    np = types.ModuleType("numpy")
    np.uint8 = object
    np.uint16 = object
    np.frombuffer = lambda *a, **k: None
    # pytest.approx consults numpy if it is importable, so the stub must answer
    # its scalar check or the assertion itself errors before comparing.
    # (pytest.approx needs more numpy internals than are worth faking here;
    #  the assertions below use plain float comparison instead.)
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = object
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **kw: None
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_mod.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.CompressedImage = object
    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = lambda data="": types.SimpleNamespace(data=data)
    for name, module in (
        ("cv2", cv2), ("numpy", np), ("rclpy", rclpy),
        ("rclpy.node", node_mod), ("rclpy.qos", qos_mod),
        ("sensor_msgs", sensor_msgs), ("sensor_msgs.msg", sensor_msg),
        ("std_msgs", std_msgs), ("std_msgs.msg", std_msg),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    path = Path(__file__).resolve().parent.parent / "upper_body_track_card.py"
    loader = importlib.util.spec_from_file_location("ubt_card_under_test", path)
    module = importlib.util.module_from_spec(loader)
    loader.loader.exec_module(module)
    return module


def _bare_plugin(module):
    """A plugin with only the state `_measure` actually reads."""
    import types

    plugin = object.__new__(module.UpperBodyTrackPlugin)
    plugin._min_kp_conf = 0.4
    plugin._focal_setting = 460.0
    plugin._signs = {"pitch_sign": 1.0, "roll_sign": 1.0, "elbow_sign": 1.0}
    plugin._bank = spec.AngleBank()
    plugin._depth_cache = types.SimpleNamespace(
        decode=lambda: (None, None, None), age_s=lambda: None)
    return plugin


def _arms_person():
    """Keypoints with both arms visible on a flat plane (no depth needed)."""
    points = _blank_points()
    # Person's LEFT arm drives the robot's right — this is the one we occlude.
    points[spec.L_SHOULDER] = [360.0, 200.0, 0.9]
    points[spec.L_ELBOW] = [400.0, 200.0, 0.9]
    points[spec.L_WRIST] = [440.0, 200.0, 0.9]
    # Person's RIGHT arm drives the robot's left.
    points[spec.R_SHOULDER] = [280.0, 200.0, 0.9]
    points[spec.R_ELBOW] = [240.0, 200.0, 0.9]
    points[spec.R_WRIST] = [200.0, 200.0, 0.9]
    points[spec.L_HIP] = [350.0, 400.0, 0.9]
    points[spec.R_HIP] = [290.0, 400.0, 0.9]
    return points


def test_an_occluded_arm_holds_its_last_angle_in_the_payload(monkeypatch):
    """Regression: a blink of missing keypoints must not read as 'not measured'.

    The bank still holds the last filtered angle, but the payload used to be
    built only from sides visible *this* frame, so an occluded arm emitted
    -999 — a discontinuity exactly when the filter should be holding.
    """
    import time

    card = _load_card_module(monkeypatch)
    plugin = _bare_plugin(card)

    first = plugin._measure([_person(_arms_person())], 640, 480, time.monotonic(), 0.0)
    assert first["tracked"] is True
    assert first["l_pitch_deg"] != -999.0
    assert first["r_pitch_deg"] != -999.0
    held = (first["r_pitch_deg"], first["r_roll_deg"], first["r_elbow_deg"])

    occluded = _arms_person()
    occluded[spec.L_WRIST] = [440.0, 200.0, 0.0]  # robot-right loses its wrist
    second = plugin._measure([_person(occluded)], 640, 480, time.monotonic(), 0.0)

    assert abs(second["r_pitch_deg"] - held[0]) < 1e-6
    assert abs(second["r_roll_deg"] - held[1]) < 1e-6
    assert abs(second["r_elbow_deg"] - held[2]) < 1e-6
    assert second["r_pitch_deg"] != -999.0
    assert second["degraded"] != ""  # the gap is still reported honestly


def test_frame_failures_are_throttled(monkeypatch):
    """One corrupt stream is one outage, not a warning per frame."""
    import threading
    import types

    card = _load_card_module(monkeypatch)
    plugin = object.__new__(card.UpperBodyTrackPlugin)
    warnings, infos = [], []

    class _Event:
        def __init__(self):
            self.n = 0

        def is_set(self):
            return self.n >= 101

        def wait(self, _timeout=0):
            self.n += 1
            return False

    def _fail(_jpeg, _received_at):
        raise ValueError("bad frame")

    plugin._stop_event = _Event()
    plugin._generation = 0
    plugin._rate_hz = 30.0
    plugin._frame_lock = threading.Lock()
    plugin._latest_jpeg = b"x"
    plugin._latest_jpeg_at = 0.0
    plugin._frame_failures = 0
    plugin._process = _fail
    plugin._node = types.SimpleNamespace(get_logger=lambda: types.SimpleNamespace(
        warning=warnings.append, info=infos.append))
    plugin._worker_loop(0)

    assert len(warnings) == 2  # first failure and the 100th, not 101
    assert "bad frame" in warnings[0]
    assert "100" in warnings[1]
