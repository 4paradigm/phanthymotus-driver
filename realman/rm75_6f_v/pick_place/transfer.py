"""Synchronous pick and place by normalized image position or horizontal displacement."""

import math

import numpy as np

from .geometry import load_targets
from .gripper import Gripper
from .motion import FeedbackPending, pose_close, rotation, vector


class Transfer:
    def __init__(self, client, motion, observation, config, selected, send, stage, displacement_mm=None):
        self.client, self.motion, self.config = client, motion, config
        self.send, self.stage = send, stage
        self.displacement_mm = displacement_mm
        self.metadata, self.pixels, self.targets = load_targets(
            observation, config, client.status()["endpoint"], selected, displacement_mm)
        self.reference = vector(self.metadata["pose"], 6, "observation pose")
        self.frames = self.metadata["frames"]
        work = vector(self.frames["work"]["pose"], 6, "work frame")
        self.work_rotation, self.work_translation = rotation(work), np.asarray(work[:3])
        self.reference_z = self.base(self.reference)[2]
        self.path_guard = self.plane_guard
        self.gripper = Gripper(client, motion, self._send, self.check)
        self.current_stage = "checking"
        self.last_completed_stage = None
        self.holding_object_possible = False
        self.release_completed = False

    def set_stage(self, name):
        if name != "checking":
            self.last_completed_stage = self.current_stage
        self.current_stage = name
        self.stage(name)

    def progress(self):
        return {"stage": self.current_stage, "last_completed_stage": self.last_completed_stage,
                "holding_object_possible": self.holding_object_possible,
                "release_completed": self.release_completed,
                "feedback_recoveries": self.motion.feedback_recoveries}

    @staticmethod
    def position_guard(error, tolerance, message):
        # Nominal reach tolerances are unchanged. A <=2 mm transient may settle
        # naturally; no next command is sent until it is back inside tolerance.
        if error > max(0.002, tolerance) + 1e-9:
            raise RuntimeError(message)
        if error > tolerance:
            raise FeedbackPending(message)

    def base(self, pose):
        return self.work_rotation @ np.asarray(pose[:3]) + self.work_translation

    def pose(self, base_position):
        position = self.work_rotation.T @ (np.asarray(base_position) - self.work_translation)
        return [*position.tolist(), *self.reference[3:]]

    def orientation_guard(self, pose):
        if not pose_close(pose, self.reference, distance=math.inf, angle=1):
            raise RuntimeError("Transfer orientation changed")
        direction = (self.work_rotation @ rotation(pose))[2, 2]
        if -direction < math.cos(math.radians(1)):
            raise RuntimeError("Transfer requires the installed gripper to point vertically down")

    def plane_guard(self, feedback):
        self.position_guard(abs(self.base(feedback["pose"])[2] - self.reference_z), 0.001,
                            "Arm left the observation height; run observe again")

    def guard(self, feedback):
        if self.motion.frames() != self.frames:
            raise RuntimeError("Work or tool frame changed; run observe again")
        self.orientation_guard(feedback["pose"])
        self.path_guard(feedback)

    def check(self):
        def sample():
            feedback = self.motion.read()
            self.guard(feedback)
            return feedback
        return self.motion.retry_feedback(sample)

    def _send(self, method, *args):
        self.check()
        self.send(method, *args)

    def hold(self, pose):
        anchor = self.base(pose)
        def stationary(feedback):
            actual = self.base(feedback["pose"])
            self.position_guard(np.linalg.norm(actual[:2] - anchor[:2]), 0.001,
                                "Arm moved during gripper operation")
            self.position_guard(abs(actual[2] - anchor[2]), 0.001,
                                "Arm moved during gripper operation")
            if not feedback["idle"]:
                raise FeedbackPending("Arm moved during gripper operation")
        self.path_guard = stationary

    def move(self, target, *, vertical):
        start = self.check()["pose"]
        a, b = self.base(start), self.base(target)
        def path(feedback):
            actual = self.base(feedback["pose"])
            if vertical:
                xy_limit = 0.001 if feedback["idle"] else 0.002
                self.position_guard(np.linalg.norm(actual[:2] - b[:2]), xy_limit,
                                    "Vertical motion left its fixed XY column")
                self.position_guard(max(min(a[2], b[2]) - actual[2], actual[2] - max(a[2], b[2]), 0),
                                    0.001, "Vertical motion exceeded its height interval")
            else:
                self.position_guard(max(abs(actual[2] - a[2]), abs(actual[2] - self.reference_z)),
                                    0.001, "Horizontal motion changed height")

        def reached(feedback):
            actual = self.base(feedback["pose"])
            return np.linalg.norm(actual[:2] - b[:2]) <= 0.001 and abs(actual[2] - b[2]) <= 0.001

        self.path_guard = path
        self._send("rm_movel", target, self.config["speed_percent"], 0, 0, 0)
        self.motion.settled(timeout=45, guard=self.guard,
                            check=self.gripper.state, reached=reached)
        self.hold(target)
        self.check()
        return list(target)

    def horizontal(self, xy):
        current = self.check()["pose"]
        base = self.base(current)
        base[:2] = xy
        base[2] = self.reference_z
        return self.move(self.pose(base), vertical=False)

    def cycle(self, kind, current):
        top = [*current[:3], *self.reference[3:]]
        bottom_base = self.base(top)
        bottom_base[2] -= self.config[f"{kind}_descent_mm"] / 1000
        bottom = self.pose(bottom_base)
        self.hold(top)
        if kind == "pick":
            self.set_stage("pick_open")
            self.gripper.force(100)
            self.gripper.move(1000)
            self.gripper.force(self.config["pick_grip_force"])
        self.set_stage(f"{kind}_descend")
        self.move(bottom, vertical=True)
        self.set_stage("pick_close" if kind == "pick" else "place_open")
        if kind == "place":
            self.gripper.force(100)
        if kind == "pick":
            # Even an uncertain command acknowledgement may mean it closed.
            self.holding_object_possible = True
        self.gripper.move(0 if kind == "pick" else 1000)
        if kind == "place":
            self.holding_object_possible = False
            self.release_completed = True
        self.set_stage(f"{kind}_lift")
        current = self.move(top, vertical=True)
        if kind == "place":
            self.set_stage("place_close")
            self.gripper.move(0)
        return current

    def run(self):
        self.set_stage("checking")
        current = self.motion.settled(guard=self.guard)["pose"]
        self.hold(current)
        # Targets and gripper compatibility are checked before any movement.
        self.gripper.verify()
        self.set_stage("move_to_pick")
        current = self.horizontal(self.targets[0])
        current = self.cycle("pick", current)
        self.set_stage("move_to_place")
        current = self.horizontal(self.targets[1])
        current = self.cycle("place", current)
        final = self.check()["pose"]
        self.last_completed_stage = self.current_stage
        result = {"ok": True, "observation_id": self.metadata["observation_id"],
                "pick_pixel": self.pixels[0],
                "pick_base_xy_mm": (self.targets[0] * 1000).tolist(),
                "place_base_xy_mm": (self.targets[1] * 1000).tolist(),
                "final_pose": final, "grasp_checked": False, **self.progress()}
        if self.displacement_mm is None:
            result["place_pixel"] = self.pixels[1]
        else:
            result.update(dx_mm=self.displacement_mm[0], dy_mm=self.displacement_mm[1],
                          direction_reference="observation_image")
        return result
