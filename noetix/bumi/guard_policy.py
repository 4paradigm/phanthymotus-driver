"""Hardware-free, fail-closed policy for Bumi face-whitelist observations.

This module does not read cameras, store biometrics, speak, or move the robot.
The caller must supply a *new camera frame's* timestamp for every observation;
repeated calls against the same cached frame cannot count as new evidence.
``now`` and ``frame_ts`` must use the same monotonically increasing clock.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Tuple


WELCOME_TEXT = "欢迎来到范式智能。"
NEUTRAL_TEXT = "您好，您的身份尚未核验，请不要进入，请联系工作人员。"


@dataclass(frozen=True)
class GuardEvent:
    """A fixed, non-motion response to one confirmed visitor arrival."""

    kind: str
    text: str
    person_id: Optional[str] = None

    def as_agent_event(self) -> Dict[str, Any]:
        """Sparse event suitable for Agent Core, without photos or names."""
        event = {"priority": 1, "event": self.kind, "text": self.text}
        if self.person_id is not None:
            event["person_id"] = self.person_id
        return event


class GuardPolicy:
    """Turn repeated face-whitelist results into at-most-once greeting events.

    Accepted normalized input shapes are ``{"status": "matched",
    "person_id": "p-1"}``, ``{"status": "unknown"}``, ``{"status":
    "no_face"}``, and ``{"status": "multiple_faces"}``. ``state`` may be
    used instead of ``status``. Any other result, disagreement between those
    keys, missing matched ID, stale frame, or invalid timestamp is rejected.

    A new event requires ``confirm_frames`` consecutive, distinct frames of
    the same outcome (and ID for a match). After an event, the visitor must
    disappear for ``rearm_no_face_frames`` frames *and* the cooldown must pass
    before a new event can be emitted. Errors and multiple faces reset pending
    evidence and cannot re-arm the policy.
    """

    def __init__(
        self,
        confirm_frames: int = 3,
        rearm_no_face_frames: int = 3,
        cooldown_s: float = 30.0,
        max_frame_age_s: float = 2.0,
        max_gap_s: float = 2.5,
        future_slack_s: float = 0.25,
    ) -> None:
        if type(confirm_frames) is not int or confirm_frames < 1:
            raise ValueError("confirm_frames must be a positive integer")
        if type(rearm_no_face_frames) is not int or rearm_no_face_frames < 1:
            raise ValueError("rearm_no_face_frames must be a positive integer")
        for label, value in (
            ("cooldown_s", cooldown_s),
            ("max_frame_age_s", max_frame_age_s),
            ("max_gap_s", max_gap_s),
            ("future_slack_s", future_slack_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError("%s must be a finite nonnegative number" % label)
        if max_frame_age_s == 0 or max_gap_s == 0:
            raise ValueError("frame age and gap limits must be positive")

        self.confirm_frames = confirm_frames
        self.rearm_no_face_frames = rearm_no_face_frames
        self.cooldown_s = float(cooldown_s)
        self.max_frame_age_s = float(max_frame_age_s)
        self.max_gap_s = float(max_gap_s)
        self.future_slack_s = float(future_slack_s)

        self._last_frame_ts: Optional[float] = None
        self._last_now: Optional[float] = None
        self._candidate: Optional[Tuple[str, Optional[str]]] = None
        self._candidate_count = 0
        self._candidate_last_ts: Optional[float] = None
        self._armed = True
        self._no_face_count = 0
        self._empty_latched = False
        self._cooldown_until = 0.0
        self._event_unpublished = False

    def _clear_candidate(self) -> None:
        self._candidate = None
        self._candidate_count = 0
        self._candidate_last_ts = None

    def _fail_closed(self) -> None:
        self._clear_candidate()
        self._no_face_count = 0
        self._empty_latched = False

    def pause(self) -> None:
        """Discard partial evidence without resetting a completed encounter."""
        self._fail_closed()

    def mark_published(self) -> None:
        """Commit an emitted event after the output publisher accepts it."""
        self._event_unpublished = False

    def abandon_unpublished(self) -> None:
        """Require fresh confirmation when an event was never sent."""
        if not self._event_unpublished:
            return
        self._event_unpublished = False
        self._armed = True
        self._cooldown_until = 0.0
        self._fail_closed()

    @staticmethod
    def _classify(result: Mapping[str, Any]) -> Optional[Tuple[str, Optional[str]]]:
        if not isinstance(result, Mapping):
            return None
        status = result.get("status", result.get("state"))
        if "status" in result and "state" in result and result["status"] != result["state"]:
            return None
        if status == "matched":
            person_id = result.get("person_id")
            if not isinstance(person_id, str) or not person_id.strip():
                return None
            return "matched", person_id.strip()
        if status in ("unknown", "no_face", "multiple_faces"):
            return status, None
        return None

    def observe(
        self,
        result: Mapping[str, Any],
        *,
        frame_ts: float,
        now: float,
    ) -> Optional[GuardEvent]:
        """Consume one camera observation; return an event only when confirmed.

        ``frame_ts`` is the original camera frame timestamp, not the time a
        recognition request was repeated. This method never calls hardware.
        """
        if (
            isinstance(frame_ts, bool)
            or isinstance(now, bool)
            or not isinstance(frame_ts, (int, float))
            or not isinstance(now, (int, float))
            or not math.isfinite(frame_ts)
            or not math.isfinite(now)
            or (self._last_now is not None and now < self._last_now)
            or frame_ts > now + self.future_slack_s
            or now - frame_ts > self.max_frame_age_s
        ):
            self._fail_closed()
            return None
        self._last_now = float(now)
        if self._last_frame_ts is not None and frame_ts <= self._last_frame_ts:
            # Duplicate/out-of-order cached frames are not independent votes.
            return None
        self._last_frame_ts = float(frame_ts)

        classification = self._classify(result)
        if classification is None:
            self._fail_closed()
            return None
        status, person_id = classification
        if status == "multiple_faces":
            self._fail_closed()
            return None
        if status == "no_face":
            self._clear_candidate()
            if not self._armed:
                self._no_face_count += 1
                if self._no_face_count >= self.rearm_no_face_frames:
                    self._empty_latched = True
                if self._empty_latched and now >= self._cooldown_until:
                    self._armed = True
                    self._empty_latched = False
                    self._no_face_count = 0
            return None

        self._no_face_count = 0
        if not self._armed:
            if not self._empty_latched or now < self._cooldown_until:
                return None
            self._armed = True
            self._empty_latched = False

        candidate = (status, person_id)
        if (
            self._candidate != candidate
            or self._candidate_last_ts is None
            or frame_ts - self._candidate_last_ts > self.max_gap_s
        ):
            self._candidate = candidate
            self._candidate_count = 1
        else:
            self._candidate_count += 1
        self._candidate_last_ts = float(frame_ts)

        if self._candidate_count < self.confirm_frames:
            return None
        self._clear_candidate()
        self._armed = False
        self._empty_latched = False
        self._cooldown_until = float(now) + self.cooldown_s
        self._event_unpublished = True
        if status == "matched":
            return GuardEvent("known_visitor_arrived", WELCOME_TEXT, person_id)
        return GuardEvent("visitor_needs_assistance", NEUTRAL_TEXT)
