"""Estimate PiPER grasp status from calibrated, passive gripper feedback."""

from math import inf
from time import monotonic, sleep, time

from agilex.piper.runtime import number
from common.vendor_runtime import action_schema, tool


MIN_SAMPLES = 3
MIN_WINDOW_S = 0.12
CHECK_TIMEOUT_S = 0.45
SAMPLE_INTERVAL_S = 0.03
MAX_WIDTH_DRIFT_MM = 0.5


GRASP_PARAMETERS = {
    "min_width_mm": {"type": "number", "minimum": 0, "title": "目标开度下限（mm）",
                     "description": "夹稳目标物后多次读取开度，填稳定读数中的最小值。须大于空夹开度加空夹容差。"},
    "max_width_mm": {"type": "number", "minimum": 0, "title": "目标开度上限（mm）",
                     "description": "夹稳目标物后多次读取开度，填稳定读数中的最大值。"},
    "empty_width_mm": {"type": "number", "minimum": 0, "title": "空夹开度（mm）",
                       "description": "填空夹稳定开度。若读数略低于 0，填 0，并用空夹容差覆盖实测偏差。"},
    "empty_tolerance_mm": {"type": "number", "exclusiveMinimum": 0, "title": "空夹容差（mm）",
                           "description": "多次空夹读数相对空夹开度的允许偏差。依据实测波动填写，须大于 0。"},
    "holding_effort_threshold_nm": {
        "type": "number", "title": "保持力矩阈值（N·m）",
        "description": "按夹稳目标物时的力矩标定。填同方向的非零阈值，保留正负号。",
    },
}


def _calibration_value(args, key, low):
    try:
        return number(args, key, None, low, inf)
    except ValueError as exc:
        label = GRASP_PARAMETERS[key]["title"]
        requirement = "大于等于 0" if low == 0 else "有限"
        raise ValueError(f"请在「{label}」填写{requirement} 的数值。") from exc


def grasp_status(samples, minimum, maximum, empty_width, empty_tolerance, effort_threshold):
    widths = [sample["width_mm"] for sample in samples]
    low, high = min(widths), max(widths)
    if high - low > MAX_WIDTH_DRIFT_MM:
        return "indeterminate", "opening_unstable"
    if empty_width - empty_tolerance <= low and high <= empty_width + empty_tolerance:
        return "empty", "within_calibrated_empty_band"
    if low < minimum or high > maximum:
        return "indeterminate", "outside_expected_opening"
    # Preserve the calibrated effort direction so opening torque does not
    # satisfy a closing threshold with the same magnitude.
    sign = 1 if effort_threshold > 0 else -1
    if any(sign * sample["effort_nm"] < abs(effort_threshold) for sample in samples):
        return "indeterminate", "holding_effort_not_confirmed"
    return "grasped", "stable_opening_and_holding_effort"


class PiperGraspCheck:
    name = "piper_grasp_check"

    def __init__(self, runtime):
        self.runtime = runtime

    def get_tools(self):
        actions = {
            "read_feedback": ([], "读取夹爪开度、力矩、使能与故障状态，无需填写参数。"),
            "check_grasp": (list(GRASP_PARAMETERS), "填写五项实测标定参数，判断夹爪是否稳定夹持。"),
        }
        schema = action_schema(actions, {key: dict(definition) for key, definition in GRASP_PARAMETERS.items()})
        schema.update({"additionalProperties": False, "x-manual-execution": True})
        schema["properties"]["action"].update(
            title="操作", default="read_feedback",
            oneOf=[{"const": "read_feedback", "title": "读取反馈（无需填写）"},
                   {"const": "check_grasp", "title": "判断是否夹住"}],
        )
        description = "先在空夹闭合和夹稳目标物时分别读取反馈。开度看 width_mm，力矩看 effort_nm，再填写标定参数。"
        return [{**tool(self.name, "sensor", description, schema), "title": "夹持确认"}]

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def read_feedback(self):
        snapshot = self.runtime.read_monitor_snapshot()
        feedback = snapshot["gripper"]
        if feedback is None:
            reasons = snapshot["reasons"]
            return {"state": "unavailable", "gripper": None,
                    "reason": reasons.get("gripper") or reasons.get("connection") or "No fresh gripper feedback."}
        summary = f"当前开度 {feedback['width_mm']:g} mm，力矩 {feedback['effort_nm']:+g} N·m"
        return {"state": "ok", "message": summary, "gripper": feedback,
                "feedback_age_ms": max(0, round((time() - feedback["feedback_time"]) * 1000))}

    def check_grasp(self, args):
        minimum = _calibration_value(args, "min_width_mm", 0)
        maximum = _calibration_value(args, "max_width_mm", 0)
        if maximum < minimum:
            raise ValueError("目标开度上限不能小于目标开度下限。")
        empty_width = _calibration_value(args, "empty_width_mm", 0)
        empty_tolerance = _calibration_value(args, "empty_tolerance_mm", 0)
        if not empty_tolerance:
            raise ValueError("空夹容差须大于 0 mm，请按实测空夹波动填写。")
        effort_threshold = _calibration_value(args, "holding_effort_threshold_nm", -inf)
        if not effort_threshold:
            raise ValueError("保持力矩阈值不能为 0，请按实测夹持力矩填写并保留正负号。")
        if minimum <= empty_width + empty_tolerance:
            return {"state": "indeterminate", "is_grasped": None, "reason": "expected_opening_overlaps_empty_band",
                    "message": "目标开度下限须大于空夹开度与空夹容差之和，请核对标定读数。"}

        samples = []
        deadline = monotonic() + CHECK_TIMEOUT_S
        task_id = None
        first_snapshot = True
        feedback = None
        while monotonic() < deadline:
            snapshot = self.runtime.read_monitor_snapshot()
            feedback = snapshot["gripper"]
            current_task = (snapshot.get("task") or {}).get("task_id")
            if first_snapshot:
                task_id = current_task
                first_snapshot = False
            if snapshot["busy"] or current_task != task_id:
                reason = "arm_operation_in_progress"
                break
            if feedback is None:
                reason = "feedback_unavailable"
                break
            if feedback["error_code"] or not feedback["enabled"]:
                reason = "gripper_fault" if feedback["error_code"] else "gripper_disabled"
                break
            stamp = feedback["feedback_time"]
            if samples and stamp < samples[-1]["feedback_time"]:
                reason = "feedback_time_changed"
                break
            if not samples or stamp > samples[-1]["feedback_time"]:
                samples.append(feedback)
                span = stamp - samples[0]["feedback_time"]
                if len(samples) >= MIN_SAMPLES and span >= MIN_WINDOW_S:
                    state, reason = grasp_status(samples, minimum, maximum, empty_width, empty_tolerance, effort_threshold)
                    return {"state": state, "is_grasped": {"grasped": True, "empty": False}.get(state),
                            "reason": reason, "gripper": feedback, "sample_count": len(samples),
                            "sample_span_ms": round(span * 1000)}
            sleep(SAMPLE_INTERVAL_S)
        else:
            reason = "insufficient_distinct_feedback"
        return {"state": "indeterminate", "is_grasped": None, "reason": reason,
                "gripper": feedback, "sample_count": len(samples)}

    def dispatch(self, action, args):
        if action in {self.name, "read_feedback", "info"}:
            return self.read_feedback()
        if action == "check_grasp":
            return self.check_grasp(args)
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        raise ValueError("Grasp check only supports read-only queries and lifecycle actions.")
