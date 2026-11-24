"""Plan and execute one supervised pick-and-drop on the shared PiPER runtime."""

from dataclasses import asdict
import math

from agilex.piper.intent_motion import cartesian_start_check, checked_state, end_pose
from agilex.piper.user_motion import (
    Transform, execute_target, orientation_error, require_controller_idle, target_pose, vector,
)


def inside(point, workspace):
    return all(low <= value <= high for value, (low, high) in zip(point, workspace))


class PickTransfer:
    def __init__(self, config, tcp):
        if not isinstance(config, dict):
            raise ValueError("Configure vision_pick.transfer before enabling visio_pick_and_drop")
        box = config.get("workspace_mm")
        if not isinstance(box, list) or len(box) != 3:
            raise ValueError("workspace_mm must have X, Y and Z intervals")
        self.workspace = tuple(vector(bounds, 2, "workspace_mm") for bounds in box)
        if any(low >= high for low, high in self.workspace):
            raise ValueError("Workspace intervals must be increasing")
        self.pick_offset = vector(config.get("pick_tcp_offset_mm"), 3, "pick_tcp_offset_mm")
        self.drop = vector(config.get("drop_tcp_xyz_mm"), 3, "drop_tcp_xyz_mm")
        self.orientation = vector(config.get("tool_rpy_deg"), 3, "tool_rpy_deg")
        self.transit_z, = vector([config.get("transit_z_mm")], 1, "transit_z_mm")
        self.opening, self.closing = vector(config.get("gripper_open_close_mm"), 2, "gripper_open_close_mm")
        self.contact_width = vector(config.get("contact_width_mm"), 2, "contact_width_mm")
        self.contact_effort, = vector([config.get("min_contact_effort_nm")], 1, "min_contact_effort_nm")
        self.speed = config.get("speed_percent")
        if type(self.speed) is not int or not 1 <= self.speed <= 5:
            raise ValueError("speed_percent must be an integer from 1 to 5")
        low, high = self.contact_width
        if (self.closing < 0 or not self.closing + .5 < low <= high < self.opening - .5
                or self.contact_effort <= 0):
            raise ValueError("Configure non-overlapping open, contact and empty-close widths and positive effort")
        if max(map(abs, self.orientation)) > 180 or self.transit_z - self.drop[2] < 10:
            raise ValueError("Tool angles must be within ±180 degrees and transit must clear the drop")
        if not all(inside(point, self.workspace) for point in (self.drop, (*self.drop[:2], self.transit_z))):
            raise ValueError("Drop and drop approach must fit within workspace_mm")
        self.tcp = tcp

    def waypoints(self, surface):
        pick = tuple(a + b for a, b in zip(surface, self.pick_offset))
        if self.transit_z - pick[2] < 10:
            raise ValueError("Transit height must be at least 10 mm above the pick TCP")
        points = {
            "above_pick": (*pick[:2], self.transit_z),
            "at_pick": pick,
            "above_drop": (*self.drop[:2], self.transit_z),
            "at_drop": self.drop,
        }
        if not all(inside(point, self.workspace) for point in points.values()):
            raise ValueError("Pick or approach lies outside the configured TCP workspace")
        return {name: (*point, *self.orientation) for name, point in points.items()}

    def in_contact(self, grip):
        low, high = self.contact_width
        return (grip.enabled and not grip.error_code and low <= grip.width_mm <= high
                and math.isfinite(grip.effort_nm) and abs(grip.effort_nm) >= self.contact_effort)

    def execute(self, runtime, plan, verify_scene, report):
        def check():
            if runtime.cancel.is_set():
                raise RuntimeError("Pick-and-drop was stopped")
            runtime.require_motion_permission()

        check()
        joints = verify_scene(plan)
        driver = runtime.connected(cancel_event=runtime.cancel)
        state = cartesian_start_check(driver, checked_state(driver, ready=True))
        require_controller_idle(driver)
        if joints is not None and max(abs(a - b) for a, b in zip(joints, state.joints_deg)) > .5:
            raise RuntimeError("Arm joints changed since the wrist observation or do not match CAN feedback")
        low, high = driver.gripper_range()
        grip = driver.get_gripper()
        if not low <= self.closing < self.opening <= high or not grip.enabled or grip.error_code:
            raise RuntimeError("Check gripper enable, fault state and effective width limits")
        current = end_pose(driver)
        start = Transform.from_pose((*current.position_mm, *current.orientation_deg)).compose(self.tcp).values()
        if (not inside(start[:3], self.workspace) or abs(start[2] - self.transit_z) > 2
                or orientation_error(start[3:], self.orientation) > 1):
            raise RuntimeError("Start with the calibrated TCP inside the workspace at transit height and tool orientation")
        targets = {name: target_pose(driver, "pose", {"pose_mm_deg": point}, runtime.config)
                   for name, point in plan["waypoints"].items()}
        verify_scene(plan)
        previous_permission = getattr(driver, "require_control_permission", None)
        driver.require_control_permission = runtime.require_motion_permission

        def move(name):
            check()
            stage(name)
            target, tip = targets[name]
            execute_target(driver, target, self.speed, "pose", require_permission=check,
                           tcp_offset=self.tcp, tip_goal=tip)

        def gripper(name, width, *, at_target=None):
            check()
            checked_state(driver, ready=True)
            require_controller_idle(driver)
            stage(name)
            return driver.move_gripper(width, runtime.gripper_torque, at_target=at_target)

        def stage(name):
            report.update(stage=name, recovery_required=True)

        try:
            move("above_pick")
            gripper("open", self.opening)
            move("at_pick")
            report["holding_object_possible"] = True
            grasp = gripper("grasp", self.closing, at_target=self.in_contact)
            report["grasp_feedback"] = asdict(grasp)
            move("above_pick")
            if not self.in_contact(driver.get_gripper()):
                raise RuntimeError("Gripper contact was lost during lift")
            move("above_drop")
            if not self.in_contact(driver.get_gripper()):
                raise RuntimeError("Gripper contact was lost during transfer")
            move("at_drop")
            if not self.in_contact(driver.get_gripper()):
                raise RuntimeError("Gripper contact was lost before release")
            gripper("release", self.opening)
            report.update(holding_object_possible=False, release_confirmed=True)
            move("above_drop")
            report.update(state="completed", stage="done", executed=True, recovery_required=False)
            return dict(report)
        finally:
            driver.require_control_permission = previous_permission
