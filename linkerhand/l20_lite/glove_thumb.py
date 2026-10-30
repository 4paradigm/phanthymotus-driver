"""Calibrated three-motor thumb mapping using palm, root and tip IMUs.

This is an independent mapping, not a reproduction of the vendor's Unity IK.
Motor 0 follows tip-relative-to-root flexion; motors 1/9 follow two separately
calibrated root-relative-to-palm movements. No hardware I/O lives here.
"""

from collections import deque
import math

MOTORS = (0, 1, 9)
POSES = ("open", "fist", "thumb_sway", "thumb_roll")


def unit(q):
    if (not isinstance(q, list) or len(q) != 4
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in q)):
        raise ValueError("three-channel thumb requires valid palm/root/tip quaternions")
    norm = math.sqrt(sum(v*v for v in q))
    if not 0.5 <= norm <= 1.5:
        raise ValueError("three-channel thumb quaternion has invalid norm")
    return [v/norm for v in q]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


class ThumbMapping:
    def __init__(self, config, delta, multiply, fraction, max_step, open_deadband):
        self.delta, self.multiply, self.fraction = delta, multiply, fraction
        self.max_step, self.open_deadband = max_step, open_deadband
        self.endpoints = {}
        for side in ("left", "right"):
            settings = config.get(side, {})
            opened = settings.get("open", [254, 254, 254])
            closed = settings.get("closed", [0, 0, 0])
            if any(not isinstance(row, list) or len(row) != 3
                   or any(type(v) is not int or not 0 <= v <= 255 for v in row)
                   for row in (opened, closed)) or any(a == b for a, b in zip(opened, closed)):
                raise ValueError("thumb endpoints need three distinct 0..255 open/closed motor values")
            self.endpoints[side] = (list(opened), list(closed))
        self.history = {}
        self.calibration = {}
        self.current = {}
        self.fractions = {}
        self._versions, self._mapped_versions = {}, {}

    def relative(self, origin, value):
        return self.multiply([-origin[0], -origin[1], -origin[2], origin[3]], value)

    def decode(self, sample):
        try:
            palm, root, tip = (unit(sample["root_quaternions"][0]),
                               unit(sample["root_quaternions"][1]),
                               unit(sample["tip_quaternions"][0]))
        except (KeyError, IndexError, TypeError):
            raise ValueError("three-channel thumb requires palm, root[1] and top[0]") from None
        return {"root": self.relative(palm, root), "tip": self.relative(root, tip)}

    def observe(self, side, pose, now, window):
        self.current[side] = pose
        self._versions[side] = self._versions.get(side, 0) + 1
        history = self.history.setdefault(side, deque(maxlen=128))
        history.append((now, pose))
        while history and now - history[0][0] > max(window, 0.02):
            history.popleft()

    def stable_pose(self, side, now, window, stability_deg):
        if window == 0:
            return self.current.get(side)
        samples = [s for s in self.history.get(side, ()) if now - s[0] <= window]
        if len(samples) < 8 or samples[-1][0] - samples[0][0] < window * 0.8:
            return None
        result = {}
        for name in ("root", "tip"):
            values = [s[1][name] for s in samples]
            def distance(a, b):
                return math.degrees(math.sqrt(dot(self.delta(a, b), self.delta(a, b))))
            center = min(values, key=lambda q: sum(distance(q, v) for v in values))
            if max(distance(center, v) for v in values) > stability_deg:
                return None
            result[name] = list(center)
        return result

    def candidate(self, side, name, pose):
        calibration = dict(self.calibration.get(side, {})) if name != "open" else {}
        if name != "open" and "open" not in calibration:
            raise RuntimeError("calibrate_open must precede thumb calibration")
        calibration[name] = pose
        if "open" in calibration and name != "open":
            source = "tip" if name == "fist" else "root"
            motion = self.delta(calibration["open"][source], pose[source])
            if dot(motion, motion) < math.radians(5)**2:
                raise RuntimeError(f"{name}: thumb motion is less than 5 degrees; repeat the pose")
        if all(key in calibration for key in ("open", "thumb_sway", "thumb_roll")):
            self._basis(calibration)  # Reject ambiguous axes before changing saved poses.
        return calibration

    def commit(self, side, calibration):
        self.calibration[side] = calibration
        self.fractions[side] = [0.0]*3
        self._mapped_versions.pop(side, None)

    def ready(self, side):
        return all(key in self.calibration.get(side, {}) for key in POSES)

    def clear(self):
        for values in (self.history, self.calibration, self.current, self.fractions,
                       self._versions, self._mapped_versions):
            values.clear()

    def _basis(self, calibration):
        opened = calibration["open"]["root"]
        sway = self.delta(opened, calibration["thumb_sway"]["root"])
        roll = self.delta(opened, calibration["thumb_roll"]["root"])
        aa, bb, ab = dot(sway, sway), dot(roll, roll), dot(sway, roll)
        if min(aa, bb) < math.radians(5)**2 or ab*ab >= 0.9*aa*bb:
            raise RuntimeError("thumb sway and root-flexion poses overlap; calibrate distinct movements")
        return sway, roll, aa, bb, ab

    def targets(self, side):
        if not self.ready(side):
            return None
        state = self.fractions.setdefault(side, [0.0]*3)
        if self._mapped_versions.get(side) != self._versions.get(side):
            calibration, current = self.calibration[side], self.current[side]
            flex = self.fraction(calibration["open"]["tip"], calibration["fist"]["tip"], current["tip"])
            if flex is None:
                raise RuntimeError("thumb tip calibration span is invalid; repeat open/fist")
            sway, roll, aa, bb, ab = self._basis(calibration)
            motion = self.delta(calibration["open"]["root"], current["root"])
            a, b = dot(motion, sway), dot(motion, roll)
            raw = [flex, (a*bb-b*ab)/(aa*bb-ab*ab), (b*aa-a*ab)/(aa*bb-ab*ab)]
            for index, value in enumerate(raw):
                value = max(0.0, min(1.0, value))
                value = 0.0 if value <= self.open_deadband else 1.0 if value >= 0.9 else value
                state[index] = max(state[index]-self.max_step, min(state[index]+self.max_step, value))
            self._mapped_versions[side] = self._versions[side]
        opened, closed = self.endpoints[side]
        return [round(a+f*(b-a)) for a, b, f in zip(opened, closed, state)]
