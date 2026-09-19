"""Direct RM_ARM+ control for the installed ZX one-DOF gripper."""

import math
import time

from .motion import FeedbackPending


class Gripper:
    def __init__(self, client, motion, send, check):
        self.client, self.motion, self.send, self.check = client, motion, send, check

    def read(self, method, *args):
        def sample():
            self.motion.check_cancel()
            started = time.monotonic()
            result = self.client.call(method, *args)
            # A slow response containing a real fault must not be discarded.
            if method == "rm_get_rm_plus_state_info":
                self.validate_state(result)
            stale = time.monotonic() - started > 1
            self.check()
            if stale:
                raise FeedbackPending("Gripper feedback is stale")
            return result
        return self.motion.retry_feedback(sample)

    def verify(self):
        info = self.read("rm_get_rm_plus_base_info")
        if info.get("manu") != "ZX" or info.get("type") != 1 or info.get("dof") != 1 or not info.get("force"):
            raise RuntimeError("Expected a ZX one-DOF gripper with force control")
        self.state()

    def state(self):
        return self.read("rm_get_rm_plus_state_info")

    @staticmethod
    def validate_state(state):
        errors = state.get("dof_err")
        if not isinstance(errors, list) or not errors or any(int(value) for value in errors) or int(state.get("sys_state", -1)):
            raise RuntimeError("Gripper feedback is missing or faulted")
        for name in ("pos", "speed", "dof_state"):
            values = state.get(name)
            if not isinstance(values, list) or not values or not math.isfinite(float(values[0])):
                raise RuntimeError(f"Gripper {name} feedback is missing")

    def force(self, value):
        self.verify()
        self.send("rm_set_rm_plus_reg", 1220, 1, [value])
        deadline = time.monotonic() + 5
        matches = 0
        while time.monotonic() < deadline:
            for length in (2, 1):
                values = self.read("rm_get_rm_plus_reg", 1220, length)
                if not isinstance(values, list) or len(values) != length:
                    raise RuntimeError("Invalid gripper force readback")
                actual = values[0]
                if type(actual) is not int or not 0 <= actual <= 100:
                    raise RuntimeError("Invalid gripper force readback")
                if length == 2:
                    self.motion.cancel.wait(0.15)
            matches = matches + 1 if actual == value else 0
            if matches >= 3 and time.monotonic() < deadline:
                return
            self.motion.cancel.wait(0.15)
        raise RuntimeError("Gripper force setting was not confirmed")

    def move(self, position):
        self.state()
        self.send("rm_set_hand_follow_pos", [position, 0, 0, 0, 0, 0], True)
        deadline = time.monotonic() + (1 if position == 0 else 15)
        stable, previous = 0, None
        while True:
            state = self.state()
            actual = state["pos"][0]
            if position == 0:
                # An object may prevent zero opening; do not gate lifting on it.
                if time.monotonic() >= deadline:
                    return
            else:
                at_target = abs(actual - position) <= 5 and state["dof_state"][0] == 2 and abs(state["speed"][0]) <= 1
                stable = stable + 1 if at_target and (previous is None or abs(actual - previous) <= 2) else 0
                previous = actual
                if stable >= 4:
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError("Gripper opening did not reach its target")
            self.motion.cancel.wait(0.15)
