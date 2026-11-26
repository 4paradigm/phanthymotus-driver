"""Experimental G1 in-place follower decision layer for pose-like messages.

Dry-run by default. This module does not connect to the robot or send commands.
The input shape and coordinate sign are hypotheses until checked against the
live perception card. A future adapter must implement the commands returned
by ``ingest`` and arbitrate ownership of the base before issuing motion.
"""
import math
import time


class FollowCard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.enabled = False
        self.target_id = None
        self.last_frame = None
        self.last_command = 0.0
        self.last_motion = False
        self.reason = "idle"

    def start(self):
        self.enabled = True
        self.target_id = None
        self.last_frame = None
        self.reason = "waiting_for_single_person"
        return {"command": "none", "reason": self.reason}

    def stop(self, reason="manual_stop"):
        self.enabled = False
        self.target_id = None
        self.reason = reason
        self.last_motion = False
        self.last_command = 0.0
        return {"command": "stop_move", "reason": reason}

    def ingest(self, payload):
        now = self.clock()
        if not self.enabled:
            return {"command": "none", "reason": self.reason}
        stamp = payload.get("timestamp")
        persons = payload.get("persons")
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or not isinstance(persons, list):
            return self.stop("invalid_pose")
        if self.last_frame is not None and stamp <= self.last_frame:
            return self.stop("stale_pose")
        self.last_frame = stamp
        if self.target_id is None:
            if len(persons) != 1:
                return {"command": "none", "reason": "waiting_for_single_person"}
            self.target_id = persons[0].get("id")
            if self.target_id is None:
                return self.stop("invalid_target")
        matches = [p for p in persons if p.get("id") == self.target_id]
        if len(matches) != 1:
            return self.stop("target_lost")
        position = matches[0].get("position")
        if not isinstance(position, list) or len(position) < 1 or not isinstance(position[0], (float, int)) or not math.isfinite(position[0]):
            return self.stop("invalid_position")
        x = float(position[0])  # Motus pose normalized horizontal position, +right.
        if abs(x) > 1:
            return self.stop("position_out_of_range")
        # Hysteresis: start correction beyond 0.18, stop inside 0.08.
        if abs(x) > .18:
            self.last_motion = True
        elif abs(x) < .08:
            self.last_motion = False
        if not self.last_motion:
            self.last_command = 0.0
            return {"command": "stop_move", "reason": "centered"}
        # Camera x is right-positive, G1 vyaw is left-positive.
        vyaw = -max(-.20, min(.20, x * .4))
        self.last_command = vyaw
        return {"command": "move", "vx": 0.0, "vy": 0.0,
                "vyaw": vyaw, "duration": .2, "reason": "following",
                "target_id": self.target_id}

    def tick(self, age_s):
        if self.enabled and age_s > .3:
            return self.stop("pose_timeout")
        return {"command": "none", "reason": self.reason}
