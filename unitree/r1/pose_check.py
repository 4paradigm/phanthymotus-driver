"""Reusable, model-independent upper-body pose checks for the R1 MVP.

The geometry is separate from image inference. The MediaPipe adapter returns
normalized anatomical keypoints; local capture and the R1 card share these rules.
"""

from __future__ import annotations

import math

POSES = ("hands_up", "arms_open", "one_hand_up", "squat")
_MAX_HANDS_UP_TILT_DEG = 25.0
_REQUIRED = tuple(f"{side}_{joint}" for side in ("left", "right")
                  for joint in ("shoulder", "elbow", "wrist"))


def _point(value):
    if isinstance(value, dict):
        x, y = value.get("x"), value.get("y")
        visibility = value.get("visibility", 1.0)
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x, y = value[:2]
        visibility = value[2] if len(value) > 2 else 1.0
    else:
        return None
    try:
        point = float(x), float(y), float(visibility)
        return point if all(math.isfinite(v) for v in point) else None
    except (TypeError, ValueError):
        return None


def normalize_keypoints(keypoints: dict) -> dict:
    """Normalize list/dict point representations and discard malformed points."""
    if not isinstance(keypoints, dict):
        return {}
    return {
        name: point
        for name, value in keypoints.items()
        if (point := _point(value)) is not None
    }


def _result(pose: str, detected: bool, score: float, feedback: str, **extra) -> dict:
    return {
        "detected": bool(detected),
        "matched": bool(detected and feedback == "已完成"),
        "status": ("completed" if detected and feedback == "已完成" else
                   "almost" if detected and score > 0 else "retry"),
        "score": round(max(0.0, min(1.0, float(score))), 3),
        "pose": pose,
        "feedback": feedback,
        **extra,
    }


def check_pose(keypoints: dict, pose: str, min_visibility: float = 0.5) -> dict:
    """Evaluate one simple upper-body pose from normalized 2D keypoints.

    Coordinates use image convention: x grows right and y grows downward.
    """
    if pose not in POSES:
        return {"error": "unsupported_pose", "pose": pose, "supported_poses": list(POSES)}

    if pose == "squat":
        return check_squat(keypoints, min_visibility)

    points = normalize_keypoints(keypoints)
    if not points:
        return _result(pose, False, 0.0, "未检测到人", reason="no_person")

    missing = [name for name in _REQUIRED if name not in points]
    hidden = [name for name in _REQUIRED
              if name in points and points[name][2] < min_visibility]
    outside = [name for name in _REQUIRED if name in points and
               not all(0 <= v <= 1 for v in points[name][:2])]
    if missing or hidden or outside:
        return _result(
            pose, False, 0.0, "未检测到完整上半身关键点",
            reason="missing_keypoints", missing=missing, low_visibility=hidden,
            out_of_frame=outside,
        )

    # Use same-axis distances: no pixel angles computed from anisotropic
    # normalized x/y coordinates. Vertical margins scale with arm y extent.
    def raised(side):
        shoulder, elbow, wrist = (points[f"{side}_{j}"] for j in
                                  ("shoulder", "elbow", "wrist"))
        label = "左" if side == "left" else "右"
        margin = max(0.015, abs(shoulder[1] - wrist[1]) * 0.1)
        if shoulder[1] - elbow[1] <= margin:
            return False, f"{label}侧大臂再抬高，让肘部高于肩膀"
        if elbow[1] - wrist[1] <= margin:
            return False, f"{label}手再抬高，让手腕高于肘部"
        # A raised arm should point mostly upward, rather than merely putting
        # the hand above the shoulder while extending sideways.
        vertical = abs(shoulder[1] - wrist[1])
        horizontal = abs(shoulder[0] - wrist[0])
        if vertical <= 1e-6 or math.degrees(math.atan2(horizontal, vertical)) > _MAX_HANDS_UP_TILT_DEG:
            return False, f"{label}臂再向上伸直，保持接近垂直"
        return True, "已完成"

    left_up, left_feedback = raised("left")
    right_up, right_feedback = raised("right")
    if pose == "hands_up":
        matched = left_up and right_up
        return _result(pose, True, (int(left_up) + int(right_up)) / 2,
                       "已完成" if matched else
                       left_feedback if not left_up else right_feedback)

    if pose == "one_hand_up":
        if left_up and right_up:
            return _result(pose, True, 0, "请只举起一只手")
        if not left_up and not right_up:
            # Correct the arm whose wrist is highest relative to its shoulder.
            side = max(("left", "right"), key=lambda side:
                       points[f"{side}_shoulder"][1] - points[f"{side}_wrist"][1])
            return _result(pose, True, 0, raised(side)[1])
        side = "left" if left_up else "right"
        other = "right" if left_up else "left"
        shoulder, elbow, wrist = (points[f"{other}_{j}"] for j in
                                  ("shoulder", "elbow", "wrist"))
        if elbow[1] <= shoulder[1] + 0.015 or wrist[1] <= elbow[1] + 0.015:
            label = "右" if other == "right" else "左"
            return _result(pose, True, 0.5, f"请放下{label}臂，只举起一只手")
        return _result(pose, True, 1, "已完成", raised_side=side)

    direction = 1 if points["right_shoulder"][0] > points["left_shoulder"][0] else -1
    def opened(side):
        shoulder, elbow, wrist = (points[f"{side}_{j}"] for j in
                                  ("shoulder", "elbow", "wrist"))
        label = "左" if side == "left" else "右"
        outward = -direction if side == "left" else direction
        upper = (elbow[0] - shoulder[0]) * outward
        lower = (wrist[0] - elbow[0]) * outward
        if upper <= 0.025:
            return False, f"{label}侧大臂向外展开"
        if abs(elbow[1] - shoulder[1]) > 0.04:
            return False, f"{label}侧大臂调整到肩膀高度"
        if lower <= 0.025:
            return False, f"{label}肘再伸直一点，让手腕向外展开"
        if max(abs(wrist[1] - elbow[1]), abs(wrist[1] - shoulder[1])) > 0.04:
            return False, f"{label}小臂调整到肩膀高度"
        return True, "已完成"

    left_open, left_feedback = opened("left")
    right_open, right_feedback = opened("right")
    return _result(pose, True, (int(left_open) + int(right_open)) / 2,
                   "已完成" if left_open and right_open else
                   left_feedback if not left_open else right_feedback)


def squat_height_metrics(keypoints, min_visibility=0.5):
    """Return 2D shoulder/hip/ankle heights without requiring visible knees."""
    points = normalize_keypoints(keypoints)
    required = [f"{side}_{joint}" for side in ("left", "right")
                for joint in ("shoulder", "hip", "ankle")]
    if any(name not in points or points[name][2] < min_visibility or
           not all(0 <= value <= 1 for value in points[name][:2])
           for name in required):
        return None
    shoulders = [points[f"{side}_shoulder"][1] for side in ("left", "right")]
    hips = [points[f"{side}_hip"][1] for side in ("left", "right")]
    ankles = [points[f"{side}_ankle"][1] for side in ("left", "right")]
    shoulder_height = sum(shoulders) / 2
    hip_height = sum(hips) / 2
    ankle_height = sum(ankles) / 2
    body_height = ankle_height - shoulder_height
    if not (shoulder_height < hip_height < ankle_height and body_height > 0.1):
        return None
    return {"shoulder_heights": shoulders, "hip_heights": hips,
            "hip_height": hip_height, "ankle_height": ankle_height,
            "body_height": body_height}


def check_squat(keypoints, min_visibility=0.5):
    """Single-frame shallow-squat geometry; a session must count down AND up.

    Requires image dimensions for metric-consistent 2D angles. This is a
    front/oblique-view demo rule, not a biomechanical assessment.
    """
    points = normalize_keypoints(keypoints)
    required = [f"{s}_{j}" for s in ("left", "right") for j in ("hip", "knee", "ankle")]
    missing = [n for n in required if n not in points]
    hidden = [n for n in required if n in points and points[n][2] < min_visibility]
    outside = [n for n in required if n in points and
               not all(0 <= v <= 1 for v in points[n][:2])]
    if missing or hidden or outside:
        return _result("squat", False, 0, "请让髋、膝和脚踝完整入画",
                       reason="missing_keypoints", missing=missing,
                       low_visibility=hidden, out_of_frame=outside)
    size = keypoints.get("__image_size__", {})
    try:
        width, height = float(size["width"]), float(size["height"])
        if not (math.isfinite(width) and math.isfinite(height) and width > 0 and height > 0):
            raise ValueError()
    except (TypeError, KeyError, ValueError):
        return _result("squat", False, 0, "缺少图像尺寸，无法判断膝关节角度",
                       error="image_size_required")
    angles = []
    leg_lengths = []
    thigh_lengths = []
    shin_lengths = []
    for side in ("left", "right"):
        hip, knee, ankle = [points[f"{side}_{j}"] for j in ("hip", "knee", "ankle")]
        a = ((hip[0]-knee[0])*width, (hip[1]-knee[1])*height)
        b = ((ankle[0]-knee[0])*width, (ankle[1]-knee[1])*height)
        na, nb = math.hypot(*a), math.hypot(*b)
        if min(na, nb) < 0.02 * height:
            return _result("squat", False, 0, "腿部关键点重叠，请调整机位",
                           reason="degenerate_keypoints")
        angle = math.degrees(math.acos(max(-1, min(1, (a[0]*b[0]+a[1]*b[1])/(na*nb)))))
        angles.append(angle)
        thigh_lengths.append(na / height)
        shin_lengths.append(nb / height)
        leg_lengths.append((na + nb) / height)
    # This is a shallow-squat exercise, not a deep squat benchmark. Keep a
    # small hysteresis band so a nearly straight knee is a valid calibration
    # pose while a modest bend can still enter the down phase.
    standing = min(angles) >= 160
    down = max(angles) <= 150 and min(angles) >= 65
    height_metrics = squat_height_metrics(points, min_visibility)
    ankle_height = sum(points[f"{s}_ankle"][1] for s in ("left", "right")) / 2
    return _result("squat", True, 1 if down else 0,
                   "已完成" if down else "请站直建立基准" if standing else "请缓慢屈膝浅蹲",
                   phase="standing" if standing else "down" if down else "transition",
                   knee_angles=[round(a, 1) for a in angles],
                   thigh_lengths=[round(value, 4) for value in thigh_lengths],
                   shin_lengths=[round(value, 4) for value in shin_lengths],
                   leg_lengths=[round(value, 4) for value in leg_lengths],
                   hip_heights=[points[f"{s}_hip"][1] for s in ("left", "right")],
                   hip_height=sum(points[f"{s}_hip"][1] for s in ("left", "right"))/2,
                   shoulder_heights=(height_metrics or {}).get("shoulder_heights"),
                   body_height=(height_metrics or {}).get("body_height"),
                   ankle_height=ankle_height,
                   leg_length=sum(leg_lengths)/2)


def compare_squat_height(result: dict, standing: dict) -> dict:
    """Compare a crouch to one person's standing image geometry.

    This is a relative 2D motion cue, not a physical height measurement.
    Missing or implausible landmarks never constitute positive evidence.
    """
    current_height = result.get("body_height")
    standing_height = standing.get("body_height") if standing else None
    if (not isinstance(current_height, (int, float)) or
            not isinstance(standing_height, (int, float)) or
            not math.isfinite(current_height) or
            not math.isfinite(standing_height) or
            min(current_height, standing_height) <= 0.1):
        return {"height_ratio": None, "height_matched": False,
                "height_recovered": False}
    ratio = current_height / standing_height
    leg_length = standing["leg_length"]
    hip_drop = result["hip_height"] - standing["hip_height"]
    ankle_shift = abs(result["ankle_height"] - standing["ankle_height"])
    current_hips, standing_hips = result.get("hip_heights"), standing.get("hip_heights")
    both_hips_down = (
        isinstance(current_hips, (list, tuple)) and len(current_hips) == 2 and
        isinstance(standing_hips, (list, tuple)) and len(standing_hips) == 2 and
        all(current - baseline >= 0.12 * leg_length
            for current, baseline in zip(current_hips, standing_hips))
    )
    current_shoulders = result.get("shoulder_heights")
    standing_shoulders = standing.get("shoulder_heights")
    both_shoulders_down = (
        isinstance(current_shoulders, (list, tuple)) and len(current_shoulders) == 2 and
        isinstance(standing_shoulders, (list, tuple)) and len(standing_shoulders) == 2 and
        all(current - baseline >= 0.08 * standing_height
            for current, baseline in zip(current_shoulders, standing_shoulders))
    )
    feet_stable = ankle_shift <= 0.35 * leg_length
    matched = (ratio <= 0.78 and hip_drop >= max(0.06, 0.25 * leg_length) and
               both_hips_down and both_shoulders_down and feet_stable)
    recovered = (ratio >= 0.9 and
                 abs(hip_drop) <= 0.16 * leg_length and feet_stable)
    return {"height_ratio": round(ratio, 3), "height_matched": matched,
            "height_recovered": recovered}


class UnavailablePoseEstimator:
    """Explicit fallback when the optional inference backend is not configured."""

    name = "unavailable"

    def estimate(self, _jpeg: bytes) -> dict | None:
        return None
