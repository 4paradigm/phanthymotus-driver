"""Offline admission of standard G1 twist messages; this never executes motion.

The caller supplies an already validated G1 descriptor and externally authorized
session/source. Authorization is not authenticated here. Message and descriptor
inputs must not be concurrently mutated during the call that snapshots them.
"""
from __future__ import annotations

import copy
import math
import threading

from common.control import Descriptor
from common.control.sink import check_control_contract, check_control_freshness
if __package__:
    from .loco_descriptor import validate_loco_descriptor
else:
    from loco_descriptor import validate_loco_descriptor


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _nonempty(value, name):
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value


class LocoAdmission:
    """Validate raw values and freshness, not arbitration or robot safety.

    Constructing or calling authorize() is an externally authorized grant and
    sequence reset. Every grant requires a session ID never previously used by
    this instance. The caller must ensure uniqueness across instance lifetimes.
    The caller-supplied session count and ID length caps bound the local history;
    when full, new grants fail rather than evicting replay protection. Session
    IDs are out of band, not authenticated or parsed from the standard message.
    Every check and sequence commit is serialized. Times are Unix milliseconds.
    """

    def __init__(self, descriptor: Descriptor, *, session: str, source: str,
                 max_command_age_ms, max_ttl_ms, max_obs_age_ms,
                 future_tolerance_ms, max_session_grants, max_session_id_length):
        if not isinstance(descriptor, Descriptor):
            raise ValueError("a validated Descriptor is required")
        supplied = copy.deepcopy(descriptor)
        try:
            snapshot = validate_loco_descriptor(copy.deepcopy(supplied.raw))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("invalid descriptor") from exc
        if snapshot != supplied or snapshot.mode != "twist" or snapshot.dof != 6:
            raise ValueError("descriptor must be the validated six-axis twist descriptor")
        for field in (snapshot.lower, snapshot.upper, snapshot.max_velocity,
                      snapshot.max_delta_per_step, snapshot.force_torque):
            if field is not None and not all(_finite(value) for value in field):
                raise ValueError("descriptor limits must be finite")
        if not all(_finite(value) for value in (snapshot.max_hz, snapshot.expected_hz,
                                                snapshot.watchdog_ms)):
            raise ValueError("descriptor rate must be finite")
        for name, value in (("max_command_age_ms", max_command_age_ms),
                            ("max_ttl_ms", max_ttl_ms),
                            ("max_obs_age_ms", max_obs_age_ms)):
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not _finite(future_tolerance_ms) or future_tolerance_ms < 0:
            raise ValueError("future_tolerance_ms must be nonnegative and finite")
        for name, value in (("max_session_grants", max_session_grants),
                            ("max_session_id_length", max_session_id_length)):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if len(_nonempty(session, "session")) > max_session_id_length:
            raise ValueError("session ID is too long")
        self._max_session_grants = max_session_grants
        self._max_session_id_length = max_session_id_length
        self._descriptor = snapshot
        self._unsupported_axes = frozenset((2, 3, 4))
        self._max_command_age_ms = max_command_age_ms
        self._max_ttl_ms = max_ttl_ms
        self._max_obs_age_ms = max_obs_age_ms
        self._future_tolerance_ms = future_tolerance_ms
        self._lock = threading.Lock()
        self._session = _nonempty(session, "session")
        self._source = _nonempty(source, "source")
        self._used_sessions = {self._session}
        self._sequences = {}

    def authorize(self, *, session: str, source: str):
        """Reset sequence state only after an explicit external authorization."""
        session = _nonempty(session, "session")
        source = _nonempty(source, "source")
        if len(session) > self._max_session_id_length:
            raise ValueError("session ID is too long")
        with self._lock:
            if session in self._used_sessions:
                raise ValueError("session ID was already used")
            if len(self._used_sessions) >= self._max_session_grants:
                raise ValueError("session grant capacity exhausted")
            self._used_sessions.add(session)
            self._session = session
            self._source = source
            self._sequences = {}

    def validate(self, message, *, session: str, now_ms):
        """Return immutable raw values on admission, or raise ValueError.

        A successful return means only that these offline checks passed. The
        caller must neither infer execution nor treat this as complete safety.
        """
        msg = copy.deepcopy(message)
        if not _finite(now_ms):
            raise ValueError("now_ms must be finite")
        with self._lock:
            outcome = check_control_contract(self._descriptor, msg)
            if outcome is not None:
                raise ValueError(outcome.reason)
            if session != self._session or msg["source"] != self._source:
                raise ValueError("session or source is not authorized")
            outcome = check_control_freshness(self._descriptor, msg, now_ms,
                                              self._sequences)
            if outcome is not None:
                raise ValueError(outcome.reason)
            if not _finite(msg["seq"]):
                raise ValueError("seq must be finite")
            if msg["ttl_ms"] > self._max_ttl_ms:
                raise ValueError("ttl exceeds configured maximum")
            obs = msg.get("obs_stamp_ms")
            if not _finite(obs):
                raise ValueError("observation timestamp is required and finite")
            if obs > msg["stamp_ms"]:
                raise ValueError("observation is later than the command")
            age, obs_age = now_ms - msg["stamp_ms"], now_ms - obs
            if not _finite(age) or not _finite(obs_age):
                raise ValueError("invalid timestamp difference")
            if age > self._max_command_age_ms or obs_age > self._max_obs_age_ms:
                raise ValueError("command or observation is too old")
            if age < -self._future_tolerance_ms or obs_age < -self._future_tolerance_ms:
                raise ValueError("command or observation is in the future")
            if "priority" in msg and not _finite(msg["priority"]):
                raise ValueError("priority must be finite")
            if msg.get("gripper") is not None and not _finite(msg["gripper"]):
                raise ValueError("gripper must be finite")
            values = tuple(msg["values"])
            for index, (value, lower, upper) in enumerate(zip(
                    values, self._descriptor.lower, self._descriptor.upper)):
                if index in self._unsupported_axes and value != 0:
                    raise ValueError(f"unsupported axis {index} is nonzero")
                if value < lower or value > upper:
                    raise ValueError(f"axis {index} exceeds descriptor bounds")
            self._sequences[msg["source"]] = msg["seq"]
            return values
