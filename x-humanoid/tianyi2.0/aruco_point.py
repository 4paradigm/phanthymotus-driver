"""Calibrated, one-axis arm pointing at an ArUco marker; the base never moves.

This is deliberately a 1-D pointing demonstration, not a 3-D hand-eye solver.
No motor command is possible until an operator has supplied and checked a
camera focal length and an arm pose/bearing calibration on the actual robot.
"""

import json
import math
import threading


JOINT_INDEX = {"shoulder_roll": 1, "shoulder_yaw": 2}
MAX_BEARING_DEG = 35.0
MAX_JOINT_STEP_DEG = 12.0
MAX_POSTURE_ERROR_DEG = 5.0


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是数字") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数字")
    return number


def camera_bearing(center_x, width, fx_px, cx_px):
    """Signed horizontal bearing in degrees, positive to image right."""
    width = _finite(width, "image_width")
    center_x = _finite(center_x, "marker_center_x")
    fx_px = _finite(fx_px, "camera_fx_px")
    cx_px = _finite(cx_px, "camera_cx_px")
    if width <= 0 or fx_px <= 0 or not 0 <= center_x <= 1 or not 0 <= cx_px < width:
        raise ValueError("相机参数或码中心超出范围")
    return math.degrees(math.atan((center_x * width - cx_px) / fx_px))


def parse_calibration(raw):
    """Validate operator-measured pose and camera-to-joint angle mapping."""
    required = ("camera_fx_px", "camera_cx_px", "side", "joint",
                "reference_pose", "reference_bearing_deg", "joint_deg_per_bearing_deg")
    missing = [key for key in required if raw.get(key) in (None, "")]
    if missing:
        raise ValueError("缺少真机标定: " + ", ".join(missing))
    side, joint = raw["side"], raw["joint"]
    if side not in ("left", "right") or joint not in JOINT_INDEX:
        raise ValueError("side 或 joint 无效")
    pose = raw["reference_pose"]
    if isinstance(pose, str):
        try:
            pose = json.loads(pose)
        except json.JSONDecodeError as exc:
            raise ValueError("reference_pose 必须是七关节 JSON 数组") from exc
    if not isinstance(pose, list) or len(pose) != 7:
        raise ValueError("reference_pose 必须是七关节数组")
    pose = [_finite(value, "reference_pose") for value in pose]
    fx = _finite(raw["camera_fx_px"], "camera_fx_px")
    cx = _finite(raw["camera_cx_px"], "camera_cx_px")
    zero = _finite(raw["reference_bearing_deg"], "reference_bearing_deg")
    gain = _finite(raw["joint_deg_per_bearing_deg"], "joint_deg_per_bearing_deg")
    if fx <= 0 or cx < 0 or abs(zero) > MAX_BEARING_DEG or not 0.05 <= abs(gain) <= 3:
        raise ValueError("标定值超出允许范围")
    # Existing ArmPlugin supplies the checked-in URDF joint limits.
    from device import ArmPlugin
    violations = ArmPlugin._pose_violations({side: pose})
    if violations:
        raise ValueError(f"reference_pose 超出关节限位: {violations}")
    return {"camera_fx_px": fx, "camera_cx_px": cx, "side": side, "joint": joint,
            "reference_pose": pose, "reference_bearing_deg": zero,
            "joint_deg_per_bearing_deg": gain}


def plan_point(data, marker_id, calibration):
    """Compute a calibrated absolute joint target from a fresh marker result."""
    try:
        marker_id = int(marker_id)
    except (TypeError, ValueError) as exc:
        raise ValueError("请选择有效的 ArUco ID") from exc
    matches = [marker for marker in data.get("detected_markers", [])
               if marker["id"] == marker_id]
    if len(matches) != 1:
        raise ValueError("目标码当前未唯一识别")
    width = data["image_width"]
    bearing = camera_bearing(matches[0]["center_normalized"][0], width,
                             calibration["camera_fx_px"], calibration["camera_cx_px"])
    if abs(bearing) > MAX_BEARING_DEG:
        raise ValueError("目标超出允许的水平指向范围")
    joint_index = JOINT_INDEX[calibration["joint"]]
    target = list(calibration["reference_pose"])
    target[joint_index] += calibration["joint_deg_per_bearing_deg"] * (
        bearing - calibration["reference_bearing_deg"])
    from device import ArmPlugin
    violations = ArmPlugin._pose_violations({calibration["side"]: target})
    if violations:
        raise ValueError(f"目标角度超出关节限位: {violations}")
    return {"marker_id": marker_id, "bearing_deg": round(bearing, 3),
            "side": calibration["side"], "joint": calibration["joint"],
            "target_joint_deg": round(target[joint_index], 3),
            "target_pose_deg": target, "method": "calibrated_horizontal_pointing"}


class ArucoPointPlugin:
    def __init__(self, plugin_config, zones, arm, hand):
        self._zones, self._arm, self._hand = zones, arm, hand
        self._config = {key: value for key, value in plugin_config.items()
                        if key not in ("enabled",)}
        self._lock = threading.Lock()

    def get_tool(self):
        properties = {
            "marker_id": {"type": "integer", "minimum": 0, "description": "用户选中的 ArUco ID"},
            "camera_fx_px": {"type": "number", "description": "RGB 相机标定焦距，单位像素"},
            "camera_cx_px": {"type": "number", "description": "RGB 相机主点横坐标，单位像素"},
            "side": {"type": "string", "enum": ["left", "right"]},
            "joint": {"type": "string", "enum": list(JOINT_INDEX)},
            "reference_pose": {"type": "string", "description": "真机验证过的七关节指向姿势，JSON 数组，单位度"},
            "reference_bearing_deg": {"type": "number", "description": "参考姿势所指方向相对相机的水平角"},
            "joint_deg_per_bearing_deg": {"type": "number", "description": "实测关节角/视觉角比例，可为负"},
            "confirm_motion": {"type": "boolean", "description": "真机确认手臂周围安全后，执行时填 true"},
        }
        return {"name": "aruco_point", "type": "actuator",
                "description": "按选中 ArUco ID 的视觉水平角及真机标定驱动单臂指向；底盘不动。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["info", "config", "preview", "point"]},
                    **properties}, "required": ["action"],
                    "x-resource": ["arm_l", "arm_r", "hand_l", "hand_r"],
                    "x-is-dangerous": True,
                    "x-action-params": {
                        "info": {"params": []},
                        "config": {"params": [key for key in properties if key not in ("marker_id", "confirm_motion")]},
                        "preview": {"params": ["marker_id"]},
                        "point": {"params": ["marker_id", "confirm_motion"]}}},
                "configSchema": {"type": "object", "properties": {
                    key: value for key, value in properties.items()
                    if key not in ("marker_id", "confirm_motion")}}}

    def start(self):
        return None

    def stop(self):
        return None

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "ready", "calibrated": self._is_calibrated(),
                    "base_motion": False}
        if action == "config":
            changes = {key: value for key, value in args.items()
                       if key in self.get_tool()["configSchema"]["properties"]}
            config = {**self._config, **changes}
            parse_calibration(config)
            with self._lock:
                self._config = config
            return self.dispatch("info", {})
        if action not in ("preview", "point"):
            raise ValueError(f"Unsupported action: {action}")
        with self._lock:
            calibration = parse_calibration(dict(self._config))
            status = self._zones.dispatch("info", {})
            if (status["state"] != "running" or status["latest"] is None
                    or status["latest_frame_age_s"] is None
                    or status["latest_frame_age_s"] > 1.0):
                raise ValueError("ArUco 画面未运行或已过期")
            plan = plan_point(status["latest"], args.get("marker_id"), calibration)
            if action == "preview":
                return {"state": "preview", **plan, "motion_enabled": False}
            if args.get("confirm_motion") is not True:
                raise ValueError("执行前须明确设置 confirm_motion=true")
            side = plan["side"]
            base = 11 if side == "left" else 21
            motor_ids = list(range(base, base + 7))
            check = self._arm._feedback.preflight(self._arm._pos_publisher, motor_ids)
            if check is not None:
                return check
            baseline_seq, positions = self._arm._feedback.snapshot(motor_ids)
            current = [math.degrees(positions[base + i]) for i in range(7)]
            index = JOINT_INDEX[plan["joint"]]
            reference = calibration["reference_pose"]
            if any(abs(current[i] - reference[i]) > MAX_POSTURE_ERROR_DEG
                   for i in range(7) if i != index):
                raise ValueError("手臂尚未处于实测参考姿势；请先人工校准")
            if abs(current[index] - plan["target_joint_deg"]) > MAX_JOINT_STEP_DEG:
                raise ValueError("单次关节转角超过 12°，拒绝执行")
            commanded_pose = list(current)
            commanded_pose[index] = plan["target_joint_deg"]
            poses = {side: commanded_pose}
            from device import ArmPlugin
            violations = ArmPlugin._pose_violations(poses)
            if violations:
                raise ValueError(f"当前手臂姿势超出关节限位: {violations}")
            result = self._arm._send_pos(poses, 0.2)
            if result.get("error") or result.get("state") == "error":
                return result
            feedback = self._arm._feedback.wait_for_motion(
                self._arm._target_positions(poses), baseline_seq, positions)
            if feedback.get("state") == "error":
                return feedback
            hand_result = self._hand.dispatch("point", {"side": side})
            return {"state": "commanded" if "error" not in hand_result else "arm_only",
                    "plan": plan, "arm": result, "feedback": feedback, "hand": hand_result,
                    "base_motion": False}

    def _is_calibrated(self):
        try:
            parse_calibration(self._config)
            return True
        except ValueError:
            return False
