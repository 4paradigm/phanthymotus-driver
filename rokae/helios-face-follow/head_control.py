"""Closed-loop head positioning through a local, leased iNex adapter."""

import json
import socket
import time
import uuid

from tracking import finite

HEAD_JOINTS = ("head_joint_1", "head_joint_2")
CONTROL_PERIOD = 0.1
SAMPLE_AGE = 0.35
LEASE_MS = 250
MAX_SPEED = 0.15
DEADBAND = 0.06


class UnixHeadBridge:
    def __init__(self, path):
        self.path = path

    def request(self, operation, **fields):
        deadline = time.monotonic() + 0.05
        body = json.dumps({"version": 1, "operation": operation, **fields}, allow_nan=False)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(0.05)
            conn.connect(self.path)
            conn.sendall(body.encode() + b"\n")
            response = bytearray()
            while b"\n" not in response and len(response) < 4096:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("head adapter response timed out")
                conn.settimeout(remaining)
                chunk = conn.recv(4096 - len(response))
                if not chunk:
                    break
                response.extend(chunk)
        if not response.endswith(b"\n") or response.count(b"\n") != 1:
            raise ValueError("invalid head adapter response framing")
        payload = json.loads(response)
        if not isinstance(payload, dict) or type(payload.get("version")) is not int or payload["version"] != 1:
            raise ValueError("invalid head adapter protocol version")
        return payload


def axis_limits(payload):
    if not isinstance(payload, dict) or set(payload) != set(HEAD_JOINTS):
        raise ValueError("expected exactly the two Helios head axes")
    limits = {}
    for joint, axis in payload.items():
        if not isinstance(axis, dict):
            raise ValueError("invalid head axis limits")
        image_axis, sign = axis.get("image_axis"), axis.get("image_sign")
        lower, upper, speed = axis.get("lower_rad"), axis.get("upper_rad"), axis.get("max_rad_s")
        if (image_axis not in ("x", "y") or type(sign) is not int or sign not in (-1, 1)
                or not all(map(finite, (lower, upper, speed)))
                or not -6.3 <= lower < upper <= 6.3 or speed <= 0):
            raise ValueError("invalid calibrated head axis limits")
        limits[joint] = (image_axis, sign, lower, upper, min(speed, MAX_SPEED))
    if {axis[0] for axis in limits.values()} != {"x", "y"}:
        raise ValueError("head adapter must provide both image axes")
    return limits


class HeadFollower:
    def __init__(self, bridge=None):
        self.bridge = bridge
        self.state = "idle" if bridge else "disabled"
        self.error = None
        self.lease = None
        self.lease_until = 0.0
        self.limits = {}
        self.last_sample = None
        self.track = None
        self.next_acquire = 0.0

    def status(self):
        return {"state": self.state, "error": self.error}

    def reset(self):
        self.halt("idle")
        if self.lease is None:
            self.error = None
            self.state = "idle" if self.bridge else "disabled"
        self.last_sample = None
        self.track = None
        self.next_acquire = 0.0

    def halt(self, state):
        if self.lease is not None:
            try:
                reply = self.bridge.request("halt", lease=self.lease)
                if reply.get("ok") is not True:
                    raise ValueError("head adapter did not acknowledge halt")
            except (OSError, ValueError, RecursionError) as error:
                self.state, self.error = "fault", str(error)
                return
            self.lease = None
            self.lease_until = 0.0
            self.limits = {}
        if self.bridge and self.error is None:
            self.state = state

    def step(self, rgb_topic, target, feedback, sample, now, valid_until):
        if not self.bridge or self.error is not None:
            return
        if target is None:
            self.halt("waiting_for_target")
            return
        if feedback is None:
            self.halt("waiting_for_feedback")
            return
        if self.lease is not None and (now >= self.lease_until or target["track_id"] != self.track):
            self.halt("waiting_for_target")
            return
        if sample == self.last_sample:
            return
        self.last_sample = sample
        self.track = target["track_id"]
        x, y = target["offset"]
        if max(abs(x), abs(y)) <= DEADBAND:
            self.halt("centered")
            return
        if self.lease is None:
            if now < self.next_acquire:
                return
            self.next_acquire = now + 1
            lease = uuid.uuid4().hex
            try:
                reply = self.bridge.request("acquire", lease=lease, camera_topic=rgb_topic, ttl_ms=LEASE_MS)
                if reply.get("ok") is not True:
                    self.state = "waiting_for_adapter"
                    return
                self.lease = lease
                self.lease_until = now + LEASE_MS / 1000
                if reply.get("camera_topic") != rgb_topic:
                    raise ValueError("head adapter approved a different camera topic")
                self.limits = axis_limits(reply.get("axes"))
            except (OSError, ValueError, RecursionError) as error:
                if self.lease is not None:
                    self.halt("fault")
                    self.error = str(error)
                else:
                    self.state = "waiting_for_adapter"
                return

        offset = {"x": x, "y": y}
        positions = {}
        speeds = {}
        for joint, (axis, sign, lower, upper, speed) in self.limits.items():
            value = feedback[joint]
            if not lower <= value <= upper:
                self.halt("outside_limits")
                return
            error = offset[axis]
            delta = sign * error * speed * CONTROL_PERIOD if abs(error) > DEADBAND else 0.0
            positions[joint] = min(upper, max(lower, value + delta))
            speeds[joint] = speed
        if all(positions[joint] == feedback[joint] for joint in HEAD_JOINTS):
            self.halt("at_limit")
            return
        try:
            reply = self.bridge.request("position", lease=self.lease, ttl_ms=LEASE_MS,
                                        valid_until_monotonic_ns=int(valid_until * 1e9),
                                        positions_rad=positions, max_rad_s=speeds)
            if reply.get("ok") is not True:
                raise ValueError("head adapter rejected position")
        except (OSError, ValueError, RecursionError) as error:
            self.halt("fault")
            self.error = str(error)
            return
        self.lease_until = min(now + LEASE_MS / 1000, valid_until)
        self.state = "controlling"
