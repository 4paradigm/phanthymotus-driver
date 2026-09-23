"""Tianyi latest-target executor. No ROS imports; all physical I/O is injected.

Wire units: rad, normalized hand closure, CLOCK_MONOTONIC nanoseconds. Producers
must share the host/boot clock. A lease secret travels only over loopback MCP;
DDS carries an HMAC, never the secret. This is not a cross-machine protocol.
"""
from __future__ import annotations

import copy
import hashlib
import hmac
import json
import math
import secrets
import threading
import time
from contextlib import contextmanager
from collections import OrderedDict

PROTOCOL = "motus.motion-target.v1"
POSITION_LEAD_SECONDS = .2
FIELDS = {"protocol", "boot_id", "session_id", "seq", "generated_ns", "valid_for_ms", "q", "hands"}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sign(value, secret):
    return hmac.new(bytes.fromhex(secret), canonical(value), hashlib.sha256).hexdigest()


def vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"invalid_{name}")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"invalid_{name}")
    return list(map(float, value))


class MotionGate:
    def __init__(self, snapshot, emit, limits, *, live_enabled=False,
                 clock=time.monotonic_ns, velocity=0.2, acceptance_check=lambda: False,
                 hands_enabled=True, continuation_timeout_ms=300, feedback_fault_timeout_ms=300):
        for name, value in (("continuation_timeout_ms", continuation_timeout_ms),
                            ("feedback_fault_timeout_ms", feedback_fault_timeout_ms)):
            if type(value) is not int or not 100 <= value <= 1000:
                raise ValueError(f"invalid_{name}")
        self.continuation_timeout_ns = continuation_timeout_ms * 1_000_000
        self.feedback_fault_timeout_ns = feedback_fault_timeout_ms * 1_000_000
        if type(hands_enabled) is not bool:
            raise ValueError("invalid_hands_enabled")
        self.hands_enabled = hands_enabled
        if type(velocity) not in (int, float) or not math.isfinite(velocity) or not 0 < velocity <= 1.5:
            raise ValueError("velocity_limit_must_be_in_0_1_5")
        self.lock = threading.RLock()
        self.snapshot = snapshot
        self.emit = emit
        if len(limits) != 14 or any(not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi for lo, hi in limits):
            raise ValueError("invalid_joint_limits")
        self.limits = limits
        self.live_enabled = live_enabled
        self.acceptance_check = acceptance_check
        self.clock = clock
        self.velocity = velocity
        self.boot_id = secrets.token_hex(16)
        self.session_id = None
        self.secret = None
        self.state = "idle"
        self.reason = None
        self.seq = -1
        self.applied_seq = -1
        self.latest = None
        self.last_emit = None
        self.last_q = None
        self.last_hands = None
        self.stop_sent_ns = None
        self.stop_target = None
        self._stop_settle = None
        self._stop_reheld = False
        self._stop_started_ns = None
        self._stop_confirmed_ns = None
        self._stop_confirmed = False
        self.release_requested = False
        self.output_active = False
        self.lease_deadline = None
        # Only an explicit onsite preparation sets this process-local window.
        # Resume and new sessions never extend it; a restart forgets it.
        self.first_acceptance_deadline_ns = None
        self._legacy_calls = 0
        self._legacy_admission = threading.Lock()
        self._recoverable_hold = False
        self._continuation = False
        self._continuation_deadline = None
        self.diagnostics = {"first_hold": None, "first_fault": None, "last_fault": None,
                            "last_command": None, "last_rejected_command": None,
                            "last_tick_ns": None, "max_tick_gap_ms": 0., "continuations": 0}
        self._checked_feedback = None
        self._motion_envelopes = OrderedDict()
        self._motion_command = None
        self._motion_envelope = None
        self._continuous_v2 = False
        self.acceleration = None
        self._command_velocity = [0.] * 14

    def _event(self, code):
        now = self.clock()
        s = self._checked_feedback or self.snapshot()
        return {"code": code, "monotonic_ns": now, "sequence": self.seq,
                "applied_sequence": self.applied_seq, "target_deadline_ns": self.lease_deadline,
                "feedback_age_ms": {k: (now-s[k])/1e6 if type(s.get(k)) is int else None
                                    for k in ("arm_ns", "power_ns", "fixed_ns", "hand_ns")},
                "command": copy.deepcopy(self.diagnostics["last_command"])}

    def _fault(self, code):
        event = self._event(code)
        if self.diagnostics["first_fault"] is None:
            self.diagnostics["first_fault"] = event
        self.diagnostics["last_fault"] = event
        self.state, self.reason, self.latest = "fault", self.diagnostics["first_fault"]["code"], None
        self._recoverable_hold = False
        self._continuation = False
        self._stop_confirmed = False

    def _can_continue(self):
        return (self.state == "hold" and self._continuation and not self.release_requested
                and (self._recoverable_hold or (self._continuation_deadline is not None
                     and self.clock() < self._continuation_deadline)))

    def _feedback(self, *, stopping=False):
        s = self.snapshot()
        self._checked_feedback = s
        now = self.clock()
        fields = ("arm_ns", "power_ns", "fixed_ns")
        if self.hands_enabled:
            fields += ("hand_ns",)
        q = vector(s.get("q"), 14, "feedback_q")
        dq = vector(s.get("dq"), 14, "feedback_dq")
        if not s.get("power_on") or s.get("estop") is not False or s.get("fault") is not False:
            raise ValueError("robot_safety_not_ready")
        for field in fields:
            stamp = s.get(field)
            if type(stamp) is not int or not 0 <= now - stamp <= 100_000_000:
                raise ValueError(f"{field}_stale")
        if not stopping and s.get("fixed_body") is not True:
            raise ValueError("calibrated_body_joints_changed")
        return s, q, dq

    @contextmanager
    def legacy(self):
        """One legacy call at a time; asynchronous sequence ownership stays visible."""
        if not self._legacy_admission.acquire(blocking=False):raise ValueError("legacy_motion_pending")
        try:
            with self.lock:
                if self.session_id:raise ValueError("motion_owned_by_teleop")
                self._legacy_calls += 1
            try:yield
            finally:
                with self.lock:self._legacy_calls -= 1
        finally:self._legacy_admission.release()

    def claim(self, legacy_busy=False):
        with self.lock:
            if self.session_id:
                raise ValueError("motion_already_owned")
            if not self.live_enabled or not self.acceptance_check():
                raise ValueError("live_acceptance_missing")
            if self._legacy_calls or (legacy_busy() if callable(legacy_busy) else legacy_busy):
                raise ValueError("legacy_motion_pending")
            _, q, dq = self._feedback()
            if max(map(abs, dq)) > 0.02:
                raise ValueError("robot_not_stopped")
            return self._new_session(q)

    def _new_session(self, q):
        self.session_id = secrets.token_hex(16)
        self.secret = secrets.token_hex(32)
        self.state, self.reason = "ready", None
        self.seq, self.latest = -1, None
        self.applied_seq = -1
        self.last_q, self.last_emit = q, self.clock()
        self._command_velocity = [0.] * 14
        self.stop_target = self.stop_sent_ns = None
        self.release_requested = False
        self.output_active = False
        self._stop_confirmed = False
        # No motion target exists yet. Allow the management reply and next
        # fresh IK frame to arrive; accepting a target still uses its <=100ms TTL.
        self.lease_deadline = self.clock() + self.continuation_timeout_ns
        self._continuation = False
        self._continuation_deadline = None
        self._recoverable_hold = False
        self._motion_envelopes.clear()
        self._motion_command = self._motion_envelope = None
        self._continuous_v2 = False
        self.diagnostics = {**self.diagnostics, "first_hold": None, "first_fault": None,
                            "last_fault": None, "last_command": None, "last_rejected_command": None,
                            "continuations": 0}
        return {"boot_id": self.boot_id, "session_id": self.session_id,
                "secret": self.secret, "protocol": PROTOCOL}

    def resume(self):
        """Explicit re-enable after a confirmed, recoverable hold. Never replay."""
        with self.lock:
            if (not self.session_id or self.state != "hold" or not self._stop_confirmed
                    or self.release_requested
                    or self.reason not in ("operator_pause", "command_timeout", "command_expired", "ik_recoverable",
                                           "arm_ns_stale", "power_ns_stale", "fixed_ns_stale", "hand_ns_stale")):
                raise ValueError("hold_not_resumable")
            if not self.live_enabled or not self.acceptance_check():
                raise ValueError("live_acceptance_missing")
            _, q, dq = self._feedback()
            if max(map(abs, dq)) > 0.02:
                raise ValueError("robot_not_stopped")
            return self._new_session(q)

    def install_motion_envelope(self, record):
        """Install an internal numerical proof, never a public wire assertion."""
        from common.motion.envelope import MotionEnvelope
        proof = MotionEnvelope.from_record(record, dof=14)
        with self.lock:
            if (not self.session_id or record['boot_id'] != self.boot_id
                    or record['session_id'] != self.session_id):
                raise ValueError('motion_envelope_session')
            if not proof.matches(record, now_ns=self.clock()):
                raise ValueError('motion_envelope_expired')
            self._motion_envelopes[record['seq']] = proof
            self._motion_envelopes.move_to_end(record['seq'])
            while len(self._motion_envelopes) > 8:
                self._motion_envelopes.popitem(last=False)

    def accept(self, packet, *, max_valid_for_ms=100, motion_command=None):
        with self.lock:
            continuing = self._can_continue()
            if not self.session_id or (self.state not in ("ready", "active") and not continuing):
                raise ValueError("motion_not_armed")
            received = self.clock()
            expired_continuation = False
            foreign_session = False
            proof = None
            try:
                if max_valid_for_ms not in (100, 300) or (max_valid_for_ms == 300 and motion_command is None):
                    raise ValueError('invalid_deadline_policy')
                if not isinstance(packet, dict) or set(packet) != FIELDS | {"mac"}:
                    raise ValueError("invalid_command_fields")
                body = {k: v for k, v in packet.items() if k != "mac"}
                # Delayed DDS packets from a released session cannot be
                # authenticated with the NEW secret. Reject them without
                # poisoning the new owner's state or extending its deadline.
                if body['boot_id'] != self.boot_id or body['session_id'] != self.session_id:
                    foreign_session = True
                    raise ValueError('stale_session')
                if not isinstance(packet["mac"], str) or not hmac.compare_digest(sign(body, self.secret), packet["mac"]):
                    raise ValueError("invalid_command_mac")
                if (body["protocol"] != PROTOCOL or body["boot_id"] != self.boot_id
                        or body["session_id"] != self.session_id):
                    raise ValueError("stale_session")
                if type(body["seq"]) is not int or not self.seq < body["seq"] < 2**53:
                    raise ValueError("stale_sequence")
                if type(body["generated_ns"]) is not int or type(body["valid_for_ms"]) is not int:
                    raise ValueError("invalid_deadline")
                ttl = body["valid_for_ms"]
                age = self.clock() - body["generated_ns"]
                if not 1 <= ttl <= max_valid_for_ms or age < 0:
                    raise ValueError("command_expired")
                q = vector(body["q"], 14, "q")
                hands = vector(body["hands"], 2, "hands")
                if any(not lo <= x <= hi for x, (lo, hi) in zip(q, self.limits)):
                    raise ValueError("joint_limit")
                if any(not 0 <= x <= 1 for x in hands):
                    raise ValueError("hand_limit")
                if age >= ttl * 1_000_000:
                    # Authenticated, otherwise-valid but late packet: discard it
                    # without extending the last valid command's wait window.
                    expired_continuation = True
                    raise ValueError("command_expired")
                if motion_command is not None:
                    # The coordinator already authenticates v2; bind that exact
                    # command to the independently issued internal proof here.
                    proof = self._motion_envelopes.get(body['seq'])
                    if (not isinstance(motion_command, dict)
                            or motion_command.get('values') != q
                            or any(motion_command.get(k) != body[k]
                                   for k in ('boot_id', 'session_id', 'seq', 'generated_ns'))
                            or not body['generated_ns'] + ttl*1_000_000 <= motion_command.get('valid_until_ns', 0)
                            or proof is None or not proof.matches(motion_command, now_ns=self.clock())):
                        raise ValueError('motion_envelope_unavailable')
                if continuing:
                    # Do not buffer a packet while waiting for a confirmed hold.
                    # Only a later, still-valid packet can resume the same stream.
                    if (not self._stop_confirmed or self._stop_confirmed_ns is None
                            or body["generated_ns"] < self._stop_confirmed_ns):
                        return False
                    if not self.live_enabled or not self.acceptance_check():
                        raise ValueError("live_acceptance_missing")
                    self._feedback()
                    if self.clock() >= body["generated_ns"] + ttl * 1_000_000:
                        expired_continuation = True
                        raise ValueError("command_expired")
                    self.state, self.reason = "ready", None
                    self.last_emit = self.clock()  # No motion credit from the hold interval.
                    self._command_velocity = [0.] * 14
                    self._continuation = False
                    self._recoverable_hold = False
                    self._stop_confirmed = False
                    self.stop_target = self.stop_sent_ns = None
                    self.diagnostics["continuations"] += 1
                self.seq = body["seq"]
                self.latest = {**body, "q": q, "hands": hands}
                self._motion_command = copy.deepcopy(motion_command)
                self._motion_envelope = proof
                self._continuous_v2 = motion_command is not None
                self.lease_deadline = body["generated_ns"] + ttl * 1_000_000
                self._continuation_deadline = received + self.continuation_timeout_ns
                self.diagnostics["last_command"] = {
                    "sequence": self.seq, "generated_ns": body["generated_ns"],
                    "received_ns": received, "accepted_ns": self.clock(),
                    "deadline_ns": self.lease_deadline, "valid_for_ms": ttl,
                    "receive_age_ms": age/1e6, "applied_ns": None}
                return True
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self.diagnostics["last_rejected_command"] = {"code": str(exc), "received_ns": received,
                    "foreign_session": foreign_session,
                    "sequence": packet.get('seq') if isinstance(packet,dict) and type(packet.get('seq')) is int else None}
                if not foreign_session:
                    recoverable = (str(exc) == 'motion_envelope_unavailable'
                                   and self.state in ('ready', 'active'))
                    self.hold(str(exc), continuation=expired_continuation,
                              recoverable=recoverable)
                raise ValueError(str(exc)) from exc

    def hold(self, reason="operator_pause", release=False, *, continuation=False, recoverable=False):
        with self.lock:
            if recoverable and (not self.session_id or self.release_requested
                    or self.state == "fault"
                    or self.state == "hold" and not self._continuation):
                raise ValueError("hold_not_resumable")
            if not self.session_id:
                return self.status()
            self.release_requested |= release
            # Explicit pause/release or any other rejection closes continuation,
            # even when an earlier timeout is still the displayed hold reason.
            self._recoverable_hold = bool(recoverable and not self.release_requested
                                          and self.state != "fault")
            self._continuation = bool((continuation and self.seq >= 0 or self._recoverable_hold)
                                      and not self.release_requested and self.state != "fault")
            self.latest = None
            self._command_velocity = [0.] * 14
            self._motion_command = self._motion_envelope = None
            if self.diagnostics["first_hold"] is None:
                self.diagnostics["first_hold"] = self._event(reason)
            if self.state not in ("hold", "fault"):
                self.state, self.reason = "hold", reason
                self.stop_sent_ns = None
                self._stop_confirmed = False
            return self.status()

    def tick(self):
        with self.lock:
            if not self.session_id:
                return
            try:
                self._checked_feedback = None
                tick_ns = self.clock()
                previous = self.diagnostics["last_tick_ns"]
                if previous is not None:
                    self.diagnostics["max_tick_gap_ms"] = max(
                        self.diagnostics["max_tick_gap_ms"], (tick_ns-previous)/1e6)
                self.diagnostics["last_tick_ns"] = tick_ns
                _, measured, dq = self._feedback(stopping=self.state in ("hold", "fault"))
                now = self.clock()
                if self.state in ("ready", "active") and now >= self.lease_deadline:
                    self.hold("command_timeout", release=self.state=="ready" and self.seq==-1,
                              continuation=True, recoverable=self._continuous_v2 and self.seq >= 0)
                if self.state in ("hold", "fault"):
                    if self.stop_sent_ns is None:
                        self.emit(measured, None)  # Never open the hands on stop.
                        self.last_q, self.last_emit = list(measured), now
                        self.stop_target, self.stop_sent_ns = measured, now
                        self._stop_started_ns = now
                        self._stop_settle, self._stop_reheld = None, False
                        return
                    newer = self._checked_feedback["arm_ns"] > self.stop_sent_ns
                    close = max(abs(a-b) for a, b in zip(measured, self.stop_target)) <= 0.02
                    if newer and close and max(map(abs, dq)) <= 0.02:
                        self.output_active = False
                        if not self._stop_confirmed:self._stop_confirmed_ns = now
                        self._stop_confirmed = True
                        if self.release_requested:
                            self.session_id = self.secret = None
                            self.state = "idle"
                            self.reason = "stop_confirmed"
                    elif now - self._stop_started_ns > 2_000_000_000:
                        raise ValueError("stop_not_confirmed")
                    elif (newer and not self._stop_reheld and self.state == "hold"
                          and max(map(abs, dq)) <= 0.02
                          and max(abs(a-b) for a,b in zip(measured, self.stop_target)) <= self.velocity*0.1):
                        # A bounded settling offset is not a stop receipt. Re-hold
                        # once only after 100 ms of distinct, stationary samples;
                        # then require a NEW receipt against the unchanged tolerances.
                        stamp = self._checked_feedback["arm_ns"]
                        if (self._stop_settle is None or
                                max(abs(a-b) for a,b in zip(measured, self._stop_settle[1])) > 0.002):
                            self._stop_settle = (stamp, list(measured))
                        elif stamp - self._stop_settle[0] >= 100_000_000:
                            self.emit(measured, None)
                            self.last_q, self.last_emit = list(measured), now
                            self.stop_target, self.stop_sent_ns = list(measured), now
                            self._stop_reheld = True
                    else:
                        self._stop_settle = None
                    return
                if not self.latest:
                    return
                # Slew the command, rather than repeatedly resetting it to measured.
                # Bound outstanding position travel to 200 ms at the configured
                # velocity, including when an actuator stops responding.
                dt = min((now - self.last_emit) / 1e9, 0.1 if self._continuous_v2 else 0.02)
                limit = self.velocity * max(dt, 0)
                lead = self.velocity * POSITION_LEAD_SECONDS
                if self.acceleration is not None and self._continuous_v2:
                    steps = []
                    for previous, target_q, speed in zip(self.last_q, self.latest["q"], self._command_velocity):
                        delta = target_q - previous
                        braking_speed = math.sqrt((self.acceleration*dt)**2+2*self.acceleration*abs(delta))-self.acceleration*dt
                        desired = math.copysign(min(self.velocity, max(0., braking_speed)), delta)
                        speed += max(-self.acceleration*dt, min(self.acceleration*dt, desired-speed))
                        step = speed*dt
                        if abs(step) > abs(delta) and step*delta >= 0:step = delta
                        steps.append(step)
                else:
                    steps = [max(-limit, min(limit, t-previous)) for previous,t in zip(self.last_q,self.latest["q"])]
                target = [max(lo, m-lead, min(hi, m+lead, previous+step))
                          for m, previous, step, (lo, hi) in zip(measured, self.last_q, steps, self.limits)]
                if self._motion_command is not None:
                    if self._motion_envelope is not None:
                        # The full IK reference may extend past this short
                        # proof. Advance only to its verified boundary without
                        # turning that ordinary boundary into a pause/resume.
                        target = [max(lo,min(hi,q)) for q,lo,hi in zip(target,
                            self._motion_envelope.lower,self._motion_envelope.upper)]
                    if (self._motion_envelope is None
                            or not self._motion_envelope.allows(self._motion_command,
                                measured, self.last_q, target, now_ns=self.clock())):
                        self.hold('motion_envelope_exceeded', recoverable=True)
                        # Holding uses measured feedback; it never steps toward
                        # an unproved target while the worker recomputes.
                        self.emit(measured, None)
                        self.last_q, self.last_emit = list(measured), self.clock()
                        self.stop_target, self.stop_sent_ns = list(measured), self.last_emit
                        self._stop_started_ns = self.last_emit
                        self._stop_settle, self._stop_reheld = None, False
                        return
                # Numerical/observer work never grants extra source lifetime.
                if self.clock() >= self.lease_deadline:
                    self.hold('command_timeout', continuation=True,
                              recoverable=self._continuous_v2)
                    return
                self._command_velocity = [(t-p)/dt if dt > 0 else 0. for t,p in zip(target,self.last_q)]
                self.emit(target, self.latest["hands"])
                self.last_q, self.last_hands, self.last_emit = target, self.latest["hands"], self.clock()
                self.applied_seq = self.latest["seq"]
                if self.diagnostics["last_command"]["applied_ns"] is None:
                    self.diagnostics["last_command"]["applied_ns"] = self.clock()
                self.diagnostics["last_command"]["last_applied_ns"] = self.clock()
                self.state, self.output_active = "active", True
            except Exception as exc:
                code = str(exc)
                snapshot = self._checked_feedback or {}
                fields = ("arm_ns", "power_ns", "fixed_ns") + (("hand_ns",) if self.hands_enabled else ())
                ages = [self.clock()-snapshot[k] if type(snapshot.get(k)) is int else -1 for k in fields]
                transient = (code in {f"{k}_stale" for k in fields} and self.state != "fault"
                             and all(0 <= age <= self.feedback_fault_timeout_ns for age in ages))
                if transient:
                    continuing = self.state in ("ready", "active") or self._continuation
                    self.hold(code, continuation=continuing, recoverable=self._recoverable_hold)
                    # Never use stale positions to manufacture a hold command or receipt.
                    # Preserve an existing hold command and its original deadline.
                    # Fresh feedback must confirm it; staleness is not permission
                    # to repeatedly reissue a different measured target.
                    self._stop_confirmed = False
                else:
                    self._fault(code)

    def status(self):
        with self.lock:
            feedback = self.snapshot()
            return {"state": self.state, "reason": self.reason, "boot_id": self.boot_id,
                    "session_id": self.session_id, "sequence": self.seq, "applied_sequence": self.applied_seq,
                    "output_active": self.output_active, "ownership_held": bool(self.session_id),
                    "stop_confirmed": self._stop_confirmed,
                    "stop_diagnostics": {"started_ns": self._stop_started_ns,
                        "sent_ns": self.stop_sent_ns, "reheld": self._stop_reheld,
                        "settle_ns": self._stop_settle[0] if self._stop_settle else None,
                        "target_q": self.stop_target},
                    "hold_confirmed": self._stop_confirmed and self.state == "hold",
                    "hold_confirmed_ns": self._stop_confirmed_ns,
                    "actuation_enabled": self.live_enabled,
                    "position_lead_rad": self.velocity * POSITION_LEAD_SECONDS,
                    "first_acceptance_deadline_ns": self.first_acceptance_deadline_ns,
                    "continuation_allowed": self._can_continue(),
                    "continuation_ready": self._can_continue() and self._stop_confirmed,
                    "timing_policy": {"target_max_ms": 300 if self._continuous_v2 else 100, "feedback_hold_ms": 100,
                                      "management_retry_ms": 300, "recoverable_hold": True,
                                      "ready_timeout_ms": self.continuation_timeout_ns//1_000_000,
                                      "continuation_timeout_ms": self.continuation_timeout_ns//1_000_000,
                                      "feedback_fault_timeout_ms": self.feedback_fault_timeout_ns//1_000_000},
                    "diagnostics": copy.deepcopy(self.diagnostics),
                    "feedback": feedback, "commanded_q": self.last_q, "monotonic_ns": self.clock()}
