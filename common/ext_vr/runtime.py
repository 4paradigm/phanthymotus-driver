"""Device-only RTC adapter. Capture sessions never confer robot authority."""

from __future__ import annotations
import copy
import secrets
import threading
import time
import uuid
import math
from pathlib import Path
from .descriptor import CAPABILITIES, CAPABILITY_DIGEST
from .protocol import ProtocolError, bind_rtc_frame_v1


def host_clock_id():
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        # Non-Linux offline tools must share this explicit identity with consumers.
        return "process-" + str(uuid.uuid4())


class DeviceRuntime:
    mode = "shadow"  # Wire compatibility only; device has no live motion mode.
    actuation_enabled = False
    profile_id = CAPABILITIES["profile_id"]
    capabilities = CAPABILITIES
    capability_digest = CAPABILITY_DIGEST

    def __init__(
        self,
        instance_id,
        *,
        clock_ns=time.monotonic_ns,
        clock_id=None,
        filter_time_ms=0,
    ):
        self.instance_id = instance_id
        self.clock_ns = clock_ns
        self.clock_id = clock_id or host_clock_id()
        self._lock = threading.RLock()
        self.running = False
        # Same-host process restart must not reuse a previous connection epoch.
        self.generation = self.clock_ns()
        self._binding = None
        self._capture_id = None
        self._latest = None
        self._sequence = -1
        self._source_time = -1
        self._space_epoch = 0
        self.error = None
        self.channels = {}
        self.last_received_ns = None
        self.last_tracking = None
        self.filter_time_ms = filter_time_ms
        self._filtered = {}
        self._filter_stamp = None

    def start(self):
        with self._lock:
            self.running = True

    def stop(self):
        with self._lock:
            self.running = False
            self.generation += 1
            self._binding = None
            self._latest = None

    def bind_capture(self, capture_id):
        with self._lock:
            if not self.running:
                raise ProtocolError("session_inactive", "device collection is stopped")
            if self._capture_id != capture_id or self._binding is None:
                self.generation += 1
                self._capture_id = capture_id
                self._sequence = self._source_time = -1
                # Reconnection cannot prove continuity of the headset tracking space.
                # Conservatively invalidate the mapping instead of reusing it.
                self._space_epoch = self.generation
                self._filtered.clear()
                self._filter_stamp = None
                self._binding = {
                    "boot_id": str(uuid.uuid4()),
                    "session_id": str(uuid.uuid4()),
                    "epoch": self.generation,
                    "fence": secrets.token_urlsafe(32),
                    "capability_digest": self.capability_digest,
                }
            return copy.deepcopy(self._binding), self.generation

    def rtc_authority_snapshot(self):
        with self._lock:
            if not self.running or self._binding is None:
                raise ProtocolError("session_inactive", "capture session unavailable")
            return copy.deepcopy(self._binding), self.generation

    def generation_matches(self, generation):
        return self.running and generation == self.generation

    def renew_capture_lease(self, capture_id, generation):
        # Compatibility name: renews no robot lease and never refreshes pose age.
        with self._lock:
            if capture_id != self._capture_id or not self.generation_matches(
                generation
            ):
                raise ProtocolError("capture_stale", "capture connection changed")

    def begin_capture_negotiation(self, capture_id, generation, **kwargs):
        self.renew_capture_lease(capture_id, generation)

    def capture_hold(self, capture_id, generation, reason):
        with self._lock:
            if capture_id == self._capture_id and generation == self.generation:
                self.generation += 1
                self._binding = None
                self._latest = None
                self.error = reason

    def mark_capture_disconnected(self, capture_id, generation):
        self.capture_hold(capture_id, generation, "capture_disconnected")

    def mark_rtc_disconnected(self, generation, reason):
        self.capture_hold(self._capture_id, generation, reason)

    def mark_channel(self, generation, label, active):
        if self.generation_matches(generation):
            self.channels[label] = active

    def record_protocol_error(self, code):
        self.error = code

    @staticmethod
    def _pose(pose):
        if pose is None:
            return {"tracked": False, "position": None, "orientation_xyzw": None}
        x, y, z = pose["position"]
        qx, qy, qz, qw = pose["orientation"]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return {
            "tracked": True,
            "position": [-z, -x, y],
            "orientation_xyzw": [-qz / norm, -qx / norm, qy / norm, qw / norm],
        }

    def submit_rtc_frame(
        self, wire, *, authority, rtc_generation, include_status=False
    ):
        frame = bind_rtc_frame_v1(wire, authority=authority, expected_mode=self.mode)
        with self._lock:
            if (
                not self.generation_matches(rtc_generation)
                or authority != self._binding
            ):
                raise ProtocolError("stale_rtc_message", "connection changed")
            if (
                frame["sequence"] <= self._sequence
                or frame["client_monotonic_ns"] <= self._source_time
            ):
                raise ProtocolError(
                    "stale_input", "sequence or acquisition time did not advance"
                )
            out = {
                "schema": "motus.teleop.command/1",
                "kind": "input",
                "instance_id": self.instance_id,
                "device_id": self._capture_id,
                "connection_epoch": self.generation,
                "space_epoch": self._space_epoch,
                "sequence": frame["sequence"],
                "source_monotonic_ns": frame["client_monotonic_ns"],
                "received_monotonic_ns": self.clock_ns(),
                "clock_id": self.clock_id,
                "tracking_frame": "tracking_x_forward_y_left_z_up",
                "head_reference": self._pose(frame["head"]),
            }
            for side in ("left", "right"):
                out[side] = self._pose(frame[side + "_controller"])
                buttons = frame["controllers"][side]["buttons"]
                if len(buttons) < 2:
                    raise ProtocolError("missing_controls", "grip and trigger required")
                out[side].update(grip=buttons[1], trigger=buttons[0])
            self._filter(out)
            # Ordinary Core can display the safe plain numeric text summary.
            # No user-provided names, identifiers or errors enter this field.
            out["text"] = f"seq={out['sequence']} " + " ".join(
                f"{side}: grip={out[side]['grip']:.2f} tracked={int(out[side]['tracked'])} "
                + (
                    "xyz=" + ",".join(f"{x:.3f}" for x in out[side]["position"])
                    if out[side]["tracked"]
                    else "xyz=invalid"
                )
                for side in ("left", "right")
            )
            self._sequence = frame["sequence"]
            self._source_time = frame["client_monotonic_ns"]
            self._latest = out
            self.last_received_ns = out["received_monotonic_ns"]
            self.last_tracking = {
                key: out[key]["tracked"] for key in ("head_reference", "left", "right")
            }
            self.error = None
            return copy.deepcopy(out)

    def _filter(self, value):
        """Filter pose only; grip transitions never reset the tracking space."""
        stamp = value["source_monotonic_ns"]
        elapsed = (
            None if self._filter_stamp is None else (stamp - self._filter_stamp) / 1e6
        )
        self._filter_stamp = stamp
        alpha = (
            1.0
            if not self.filter_time_ms or elapsed is None or elapsed > 300
            else 1 - math.exp(-elapsed / self.filter_time_ms)
        )
        for name in ("head_reference", "left", "right"):
            pose = value[name]
            previous = self._filtered.get(name)
            if not pose["tracked"]:
                self._filtered.pop(name, None)
                continue
            if previous and alpha < 1:
                pose["position"] = [
                    a + alpha * (b - a)
                    for a, b in zip(previous["position"], pose["position"])
                ]
                a, b = previous["orientation_xyzw"], pose["orientation_xyzw"]
                dot = sum(x * y for x, y in zip(a, b))
                if dot < 0:
                    b, dot = [-x for x in b], -dot
                if dot > 0.9995:
                    q = [x + alpha * (y - x) for x, y in zip(a, b)]
                else:
                    theta = math.acos(max(-1.0, min(1.0, dot)))
                    q = [
                        (
                            math.sin((1 - alpha) * theta) * x
                            + math.sin(alpha * theta) * y
                        )
                        / math.sin(theta)
                        for x, y in zip(a, b)
                    ]
                norm = math.sqrt(sum(x * x for x in q))
                pose["orientation_xyzw"] = [x / norm for x in q]
            self._filtered[name] = copy.deepcopy(pose)

    def command_identity(self, *, require_capture=True):
        with self._lock:
            if (
                not self.running
                or self._capture_id is None
                or (require_capture and self._binding is None)
            ):
                raise ValueError("capture_not_ready")
            return {
                "instance_id": self.instance_id,
                "device_id": self._capture_id,
                "connection_epoch": self.generation,
                "space_epoch": self._space_epoch,
                "sequence": max(0, self._sequence),
                "received_monotonic_ns": self.clock_ns(),
                "clock_id": self.clock_id,
            }

    def take_latest(self):
        with self._lock:
            result = self._latest
            self._latest = None
            return result

    def status(self):
        with self._lock:
            age_ms = (
                None
                if self.last_received_ns is None
                else max(0, (self.clock_ns() - self.last_received_ns) / 1e6)
            )
            fresh = (
                self.running
                and self._binding is not None
                and age_ms is not None
                and age_ms <= 300
            )
            return {
                "state": "collecting" if self.running else "idle",
                "generation": self.generation,
                "last_sequence": self._sequence,
                "error": self.error,
                "actuation_enabled": False,
                "input_fresh": fresh,
                "input_age_ms": age_ms,
                "tracking": copy.deepcopy(self.last_tracking),
                "clock_id": self.clock_id,
            }
