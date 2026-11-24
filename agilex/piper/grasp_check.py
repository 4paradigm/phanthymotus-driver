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
        properties = {
            "min_width_mm": {"type": "number", "minimum": 0,
                             "description": "Minimum expected opening around the workpiece in mm."},
            "max_width_mm": {"type": "number", "minimum": 0,
                             "description": "Maximum expected opening around the workpiece in mm."},
            "empty_width_mm": {"type": "number", "minimum": 0,
                               "description": "Measured opening with the jaws closed empty, in mm."},
            "empty_tolerance_mm": {"type": "number", "exclusiveMinimum": 0,
                                   "description": "Measured tolerance around the empty-closed opening in mm."},
            "holding_effort_threshold_nm": {
                "type": "number",
                "description": "Nonzero signed gripper motor torque threshold in N m, calibrated while holding a known part.",
            },
        }
        actions = {
            "read_feedback": ([], "Read current gripper opening, motor torque, enable and fault status."),
            "check_grasp": (list(properties), "Estimate the grasp from fresh feedback. Supply all five calibrated inputs."),
        }
        schema = action_schema(actions, properties)
        schema["additionalProperties"] = False
        schema["properties"]["action"].update(
            title="Action", default="read_feedback",
            oneOf=[{"const": "read_feedback", "title": "Read gripper feedback"},
                   {"const": "check_grasp", "title": "Check grasp"}],
        )
        definition = tool(self.name, "sensor", "Read-only PiPER gripper feedback and calibrated grasp estimation.", schema)
        return [{**definition, "title": "Grasp check"}]

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
        return {"state": "ok", "gripper": feedback,
                "feedback_age_ms": max(0, round((time() - feedback["feedback_time"]) * 1000))}

    def check_grasp(self, args):
        minimum = number(args, "min_width_mm", None, 0, inf)
        maximum = number(args, "max_width_mm", None, minimum, inf)
        empty_width = number(args, "empty_width_mm", None, 0, inf)
        empty_tolerance = number(args, "empty_tolerance_mm", None, 0, inf)
        effort_threshold = number(args, "holding_effort_threshold_nm", None, -inf, inf)
        if not empty_tolerance or not effort_threshold:
            raise ValueError("Empty tolerance and signed holding torque threshold must be nonzero.")
        if minimum <= empty_width + empty_tolerance:
            return {"state": "indeterminate", "is_grasped": None, "reason": "expected_opening_overlaps_empty_band"}

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
