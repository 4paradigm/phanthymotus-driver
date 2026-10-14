"""Read-only robot preview. Never grants authority or changes device input."""

import copy
import math

from common.teleop_contract import FEEDBACK_SCHEMA, validate_feedback


def validate_display(value):
    """Validate optional display fields without computing robot alignment."""
    def number(value, upper):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= upper:
            raise ValueError("invalid_display_number")

    visual = value.get("visualization")
    if visual is not None:
        if not isinstance(visual, dict) or type(visual.get("available")) is not bool:
            raise ValueError("invalid_visualization")
        if visual["available"]:
            if type(visual.get("feedback_fresh")) is not bool:
                raise ValueError("invalid_feedback_fresh")
            number(visual.get("intent_age_ms"), 1e12)
            for key, count in (("measured", 2), ("ik", 2), ("torso", 16)):
                paths = visual.get(key, [])
                if not isinstance(paths, list) or len(paths) > count:
                    raise ValueError("invalid_display_chains")
                for path in paths:
                    if not isinstance(path, list) or not 2 <= len(path) <= 32:
                        raise ValueError("invalid_display_chain")
                    for point in path:
                        if not isinstance(point, list) or len(point) != 3:
                            raise ValueError("invalid_display_point")
                        for coordinate in point:
                            if type(coordinate) not in (int, float) or not math.isfinite(coordinate) or abs(coordinate) > 4:
                                raise ValueError("invalid_display_point")
    alignment = value.get("alignment")
    if alignment is not None:
        if not isinstance(alignment, dict) or alignment.get("state") not in {
            "unaligned", "ready", "active", "unreachable", "unavailable"
        } or type(alignment.get("ready")) is not bool:
            raise ValueError("invalid_alignment")
        if alignment["ready"] != (alignment["state"] == "ready"):
            raise ValueError("invalid_alignment_ready")
        if alignment["state"] != "unavailable":
            for side in ("left", "right"):
                arm = alignment.get(side)
                if not isinstance(arm, dict):
                    raise ValueError("invalid_alignment_arm")
                for key, upper in (("position_error_m", 8), ("orientation_error_rad", math.pi),
                                   ("position_tolerance_m", 8), ("orientation_tolerance_rad", math.pi)):
                    number(arm.get(key), upper)
        if alignment["ready"] and (not visual or not visual.get("available")
                or not visual.get("feedback_fresh") or len(visual.get("measured", [])) != 2
                or len(visual.get("ik", [])) != 2):
            raise ValueError("alignment_without_current_skeletons")


class RobotFeedback:
    def __init__(self, runtime):
        self.runtime = runtime
        self.generation = None
        self._reset()

    def _reset(self):
        self.value = None
        self.source = None
        self.sequence = self.mapping_epoch = self.source_sequence = -1
        self.conflict = False
        self.reason = "feedback_unavailable"

    def _identity(self):
        if self.generation != self.runtime.generation:
            self._reset()
            self.generation = self.runtime.generation
        return self.runtime.command_identity()

    def accept(self, message):
        # Share the capture lock so a late callback cannot repopulate an old epoch.
        with self.runtime._lock:
            try:
                identity = self._identity()
                value = validate_feedback(message, instance_id=identity["instance_id"],
                    clock_id=identity["clock_id"], now_ns=self.runtime.clock_ns(), max_age_ns=750_000_000)
                if any(value[key] != identity[key] for key in ("connection_epoch", "space_epoch")):
                    raise ValueError("feedback_epoch_mismatch")
                if value["source_sequence"] is None or not 0 <= value["source_sequence"] <= identity["sequence"]:
                    raise ValueError("feedback_input_mismatch")
                source = (value["control_instance_id"], value["server_epoch"])
                if self.source is not None and source != self.source:
                    self.conflict = True
                if self.conflict:
                    raise ValueError("feedback_source_conflict")
                if (value["sequence"] <= self.sequence or value["mapping_epoch"] < self.mapping_epoch
                        or value["source_sequence"] < self.source_sequence):
                    raise ValueError("feedback_out_of_order")
                validate_display(value)
                self.source = source
                self.sequence, self.mapping_epoch = value["sequence"], value["mapping_epoch"]
                self.source_sequence = value["source_sequence"]
                self.value = value
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
                self.value = None
                self.reason = "feedback_source_conflict" if self.conflict else "feedback_invalid"

    def snapshot(self):
        with self.runtime._lock:
            unavailable = {"schema": FEEDBACK_SCHEMA, "available": False,
                           "reason": self.reason, "alignment": {"state": "unavailable", "ready": False}}
            try:
                self._identity()
            except ValueError:
                self.value = None
                return unavailable
            if self.value is None:
                return unavailable
            age_ms = (self.runtime.clock_ns() - self.value["emitted_monotonic_ns"]) / 1e6
            status = self.runtime.status()
            if age_ms < 0 or age_ms >= 750 or not status["input_fresh"] or not status["tracking"] or not all(status["tracking"].values()):
                return {**unavailable, "reason": "feedback_stale"}
            value = copy.deepcopy(self.value)
            visual = value.get("visualization")
            if visual is not None and visual.get("available"):
                visual["intent_age_ms"] += age_ms
                visual["feedback_age_ms"] = age_ms
                if visual["intent_age_ms"] >= 300:
                    visual["ik"] = []
                    value["alignment"] = {"state": "unavailable", "ready": False}
            value.setdefault("alignment", {"state": "unavailable", "ready": False})
            return value
