"""Use MoveIt for arm motion and retain the local Franka Hand commands."""
import math
import time

import numpy as np
from scipy.interpolate import BPoly
from scipy.spatial.transform import Rotation

from .execution import OBSERVATION_TOLERANCE_DEG, PickExecution
from .hand_collision import check_table, check_descent_curve, check_path_table, frame, table_clearance, vertices
from .kinematics import Kinematics


# FR3 FCI joint limits, with a 1% margin for strict native comparisons.
JOINT_ACCELERATION_LIMIT = 10.0 * .99  # rad/s²
JOINT_JERK_LIMIT = 5000.0 * .99  # rad/s³


def quintic_segments(times, positions, velocities, accelerations):
    """Construct all Bernstein coefficients and derivative bounds at once."""
    dt = np.diff(times)[:, None]
    p0, p1 = positions[:-1], positions[1:]
    v0, v1 = velocities[:-1] * dt / 5, velocities[1:] * dt / 5
    a0, a1 = accelerations[:-1] * dt**2 / 20, accelerations[1:] * dt**2 / 20
    controls = np.stack((p0, p0 + v0, 2 * (p0 + v0) - p0 + a0,
                         2 * (p1 - v1) - p1 + a1, p1 - v1, p1), axis=1)
    bounds = []
    coefficients = controls
    for degree in (5, 4, 3):
        coefficients = np.diff(coefficients, axis=1) * degree / dt[:, None]
        bounds.append(float(np.max(np.abs(coefficients))))
    return controls, bounds


class GraspMiss(RuntimeError):
    """A finished grasp command with fresh feedback reporting no object."""


class RemoteExecution(PickExecution):
    def __init__(self, active, arm, hand, options, state, profile, scene=None):
        super().__init__(active, arm, hand, options)
        self.profile, self.scene = profile, scene
        self.flange_to_ee = np.asarray(state['F_T_EE']).reshape(4, 4, order='F')
        self.hand_points = vertices(frame(profile, state['F_T_EE']))
        self.q = list(state['q'])
        self.goals = {}
        self.visible_surface = (np.asarray(scene['visible_surface_base_mm']) / 1000
                                if scene and scene.get('visible_surface_base_mm') else None)
        actual = np.asarray(state['O_T_EE']).reshape(4, 4, order='F')
        model_flange = np.asarray(arm.flange_at(self.q))
        sdk_flange = np.asarray(self.flange_pose(actual))
        angle = (Rotation.from_quat(model_flange[3:]).inv() * Rotation.from_quat(sdk_flange[3:])).magnitude()
        if np.linalg.norm(model_flange[:3] - sdk_flange[:3]) > .005 or angle > math.radians(2):
            raise ValueError('MoveIt flange frame does not match the measured SDK tool frame')
        from .spline_worker import initialize
        initialize(self, state)

    def idle(self):
        deadline = time.monotonic() + 3
        while True:
            state = self.fresh(self.arm, wait_for_arm=True)
            if (state.get('robot_mode') == 1 and len(state['dq']) == 7
                    and max(map(abs, state['dq'])) < .01):
                return state
            if time.monotonic() >= deadline:
                raise RuntimeError('Arm did not settle under MoveIt control')
            self.active['cancel'].wait(.025)

    def flange_pose(self, ee):
        target = np.asarray(ee) @ np.linalg.inv(self.flange_to_ee)
        return [*target[:3, 3], *Rotation.from_matrix(target[:3, :3]).as_quat()]

    def prepare(self, stage, target, *, cartesian=False, joint=False, use_ik=False):
        self.check()
        started = time.monotonic()
        self.goals[stage] = (target, dict(cartesian=cartesian, joint=joint, use_ik=use_ik))
        if use_ik:
            target_joints = self.arm.solve_ik(self.flange_pose(target), self.q)
        elif joint:
            target_joints = target
        else:
            target_joints = None

        if stage in ('approach', 'move_to_observation', 'return_to_observation') and target_joints is not None:
            candidates, failure = self.joint_candidates(stage, target_joints)
            for _, plan in sorted(candidates, key=lambda candidate: candidate[0]):
                try:
                    return self.prepare_trajectory(stage, target, plan, cartesian, joint, started)
                except ValueError as error:
                    failure = error
                except RuntimeError as error:
                    self.check()
                    failure = error
            raise failure

        if target_joints is not None:
            plan = self.arm.plan_joint(target_joints, self.q)
        elif cartesian:
            plan = self.arm.plan_cartesian([self.flange_pose(p) for p in target], .1, self.q)
        else:
            raise ValueError(f'Missing planning mode for {stage}')
        return self.prepare_trajectory(stage, target, plan, cartesian, joint, started)

    def joint_candidates(self, stage, target_joints):
        candidates = []
        failure = None
        for attempt in range(1, 4):
            self.check()
            started = time.monotonic()
            plan = None
            try:
                plan = self.arm.plan_joint(target_joints, self.q)
            finally:
                detail = plan or {}
                print(f'[fr3 joint planning] action={self.active.get("action_id")} stage={stage} '
                      f'attempt={attempt} rpc_s={time.monotonic()-started:.3f} '
                      f'moveit_s={detail.get("planning_s")} code={detail.get("code")}', flush=True)
            if plan.get('code') != 1 or not plan.get('succeeded') or not plan.get('points'):
                failure = RuntimeError(f'MoveIt failed to plan {stage} (code={plan.get("code")})')
                continue
            positions = np.asarray([point['positions'] for point in plan['points']], dtype=float)
            if positions.ndim != 2 or positions.shape[1] != 7 or not np.isfinite(positions).all():
                failure = ValueError(f'Invalid MoveIt joint positions during {stage}')
                continue
            candidates.append((float(np.abs(np.diff(positions, axis=0)).sum()), plan))
        return candidates, failure

    def approach_candidates(self, target):
        started = time.monotonic()
        try:
            joints = self.arm.solve_ik(self.flange_pose(target), self.q)
        finally:
            print(f'[fr3 approach IK] action={self.active.get("action_id")} '
                  f'rpc_s={time.monotonic()-started:.3f}', flush=True)
        candidates, failure = self.joint_candidates('approach', joints)
        if not candidates:
            raise failure
        return candidates

    def prepare_trajectory(self, stage, target, plan, cartesian, joint, started=None):
        from .spline_worker import checked_spline
        self.check()
        requested = time.monotonic()
        trajectory, end = checked_spline(self, stage, target, plan, cartesian, joint)
        self.check()
        if trajectory is not None:
            self.q = end
        rpc = f'planning_rpc_s={requested-started:.3f} ' if started is not None else ''
        print(f'[fr3 spline] action={self.active.get("action_id")} stage={stage} '
              f'{rpc}controller_s={time.monotonic()-requested:.3f}', flush=True)
        return stage, trajectory

    def build_trajectory(self, stage, target, plan, cartesian, joint):
        if (plan.get('code') != 1 or not plan.get('points')
                or (cartesian and plan.get('fraction') != 1.)
                or (not cartesian and not plan.get('succeeded'))):
            raise RuntimeError(f'MoveIt failed to plan {stage} (code={plan.get("code")}, fraction={plan.get("fraction")})')
        planned = time.monotonic()
        names = plan['joint_names']
        order = [names.index(f'fr3_joint{i}') for i in range(1, 8)]
        samples = plan['points']
        positions = np.asarray([sample['positions'] for sample in samples])[:, order]
        if np.max(np.abs(positions[0] - self.q)) > .003:
            raise RuntimeError(f'MoveIt {stage} trajectory starts at a different pose')
        if len(samples) == 1:
            if not joint or np.max(np.abs(positions[0] - target)) > .003:
                raise RuntimeError(f'MoveIt returned no motion for {stage}')
            return stage, None
        speed = math.radians(self.options['speed_deg_s'])
        times = np.array([sample['time_ns'] for sample in samples], dtype=float) / 1e9
        if times[0] != 0 or not np.all(np.diff(times) > 0):
            raise RuntimeError('MoveIt returned invalid trajectory timing')
        if stage in ('descend', 'miss_retreat', 'place_descend'):
            descent_planes, minimum_gap = self.descent_limits()
        # Make every segment quintic, with a stationary start and end, so
        # controller interpolation is continuous through acceleration.
        derivatives = []
        for key in ('velocities', 'accelerations'):
            values = [sample[key] or [0.] * 7 for sample in samples]
            if any(len(value) != 7 for value in values):
                raise ValueError(f'Invalid MoveIt {key} during {stage}')
            values = np.asarray(values, dtype=float)
            if not np.isfinite(values).all():
                raise ValueError(f'Invalid MoveIt {key} during {stage}')
            values[[0, -1]] = 0.
            derivatives.append(values)
        velocities, accelerations = derivatives
        self.check()
        controls, (velocity, acceleration, jerk) = quintic_segments(
            times, positions, velocities[:, order], accelerations[:, order])
        limits = {'sample_period': .001001 / float(np.min(np.diff(times))),
                  'speed': velocity / speed * 1.01,
                  'acceleration': math.sqrt(acceleration / JOINT_ACCELERATION_LIMIT),
                  'jerk': (jerk / JOINT_JERK_LIMIT) ** (1./3.)}
        limiting = max(limits, key=limits.get)
        factor = limits[limiting]
        if times[-1] * factor > 359.:
            raise ValueError(f'Continuous position trajectory for {stage} exceeds execution time limit')
        for sample, v, a in zip(samples, (velocities / factor).tolist(),
                                (accelerations / factor**2).tolist()):
            sample['time_ns'] = round(sample['time_ns'] * factor)
            sample['velocities'] = v
            sample['accelerations'] = a
        if stage in ('descend', 'miss_retreat', 'place_descend'):
            plan['descent_planes'] = descent_planes
            plan['descent_min_gap_m'] = minimum_gap
        plan['segments'] = list(zip((times[1:] * factor).tolist(), controls.tolist()))
        retimed = time.monotonic()
        self.validate_path(stage, plan, sample_scale=factor)
        print(f"[fr3 timing] action={self.active.get('action_id')} stage={stage} "
              f"checks_s={time.monotonic()-planned:.3f} "
              f"retime_s={retimed-planned:.3f} validation_s={time.monotonic()-retimed:.3f} "
              f"speed_deg_s={math.degrees(speed):.2f} nominal_s={times[-1]:.3f} "
              f"retimed_s={times[-1]*factor:.3f} limiting={limiting} factor={factor:.2f}", flush=True)
        self.q = positions[-1].tolist()
        return stage, plan

    def validate_path(self, stage, plan, *, sample_scale=1.):
        plane = self.scene or self.profile.get('table_plane')
        if plane:
            table = np.asarray(plane.get('table_point_base_mm', plane.get('point_base_mm'))) / 1000
            normal = np.asarray(plane.get('table_normal_base', plane.get('normal_base')))
            if (table.shape != (3,) or normal.shape != (3,) or not np.isfinite(table).all()
                    or not np.isclose(np.linalg.norm(normal), 1., atol=.001)):
                raise ValueError('Invalid table plane')
        elif self.profile.get('tool_frame'):
            raise ValueError('Calibrated table plane is required for remote motion')
        minimum = math.inf
        start = 0.
        for stop, controls in plan['segments']:
            self.check()
            curve = BPoly(np.asarray(controls)[:, None, :], [start, stop])
            if 'descent_planes' in plan:
                minimum = min(minimum, check_descent_curve(
                    self.model, curve, self.hand_points, plan['descent_planes'], self.check,
                    minimum_gap=plan['descent_min_gap_m']))
            interval = np.linspace(start, stop, max(8, int(np.ceil((stop - start) / (.01 * sample_scale))) + 1))
            samples = curve(interval)
            for first in range(0, len(samples), 64):
                self.check()
                batch = samples[first:first+64]
                chains, starts, ends = self.model.geometry(batch)
                clearances = self.model.sample_clearances(starts, ends)
                for q, chain, a, b, clearance in zip(batch, chains, starts, ends, clearances):
                    self.check()
                    self.model.cache_sample(q, chain, a, b)
                    if np.any(q < self.model.low) or np.any(q > self.model.high):
                        j = np.flatnonzero((q < self.model.low) | (q > self.model.high))[0]
                        lo, hi, actual = map(math.degrees, (self.model.low[j], self.model.high[j], q[j]))
                        raise ValueError(f'Joint {j+1} at {actual:.3f} deg outside [{lo:.3f}, {hi:.3f}] during {stage}')
                    if clearance <= .02:
                        raise ValueError(f'Arm self-collision clearance violated during {stage}')
                    if plane:
                        check_path_table(self.model, q, self.hand_points, table, normal, stage, self.profile)
                    if self.visible_surface is not None and stage in ('approach', 'descend'):
                        ee = self.model.pose(q)
                        local = (self.visible_surface - ee[:3, 3]) @ ee[:3, :3]
                        if np.any(np.all((local >= [-.03164, -.10399, -.12944])
                                         & (local <= [.03164, .10043, -.03744]), axis=1)):
                            raise ValueError(f'Hand housing intersects the target during {stage}')
            start = stop
        if 'descent_planes' in plan:
            plan['descent_gap_bound_mm'] = minimum * 1000

    def descent_limits(self):
        plane = self.scene or self.profile.get('table_plane')
        if not plane:
            raise ValueError('A table plane is required to verify descent clearance')
        table = np.asarray(plane.get('table_point_base_mm', plane.get('point_base_mm'))) / 1000
        normal = np.asarray(plane.get('table_normal_base', plane.get('normal_base')))
        if (table.shape != (3,) or normal.shape != (3,) or not np.isfinite(table).all()
                or not np.isclose(np.linalg.norm(normal), 1., atol=.001)):
            raise ValueError('Invalid table plane')
        allowance = self.profile.get('descent_clearance_allowance_mm', 0)
        if type(allowance) not in (int, float) or not math.isfinite(allowance) or allowance < 0:
            raise ValueError('Invalid descent clearance allowance')
        # Apply the measured allowance to collision checking only.
        descent_planes = [(table + normal * (allowance / 1000), normal)]
        clearance_mm = self.profile.get('descent_min_clearance_mm', 5)
        if type(clearance_mm) not in (int, float) or not math.isfinite(clearance_mm) or clearance_mm < 1:
            raise ValueError('Invalid descent minimum clearance')
        minimum_gap = clearance_mm / 1000
        fixed = self.profile.get('table_plane')
        if fixed and fixed is not plane:
            fixed_table = np.asarray(fixed['point_base_mm']) / 1000
            fixed_normal = np.asarray(fixed['normal_base'])
            if (fixed_table.shape != (3,) or fixed_normal.shape != (3,)
                    or not np.isfinite(fixed_table).all()
                    or not np.isclose(np.linalg.norm(fixed_normal), 1., atol=.001)):
                raise ValueError('Invalid calibrated table plane')
            descent_planes.append((fixed_table, fixed_normal))
        return descent_planes, minimum_gap

    def check_descent_target(self, target, stage):
        planes, minimum_gap = self.descent_limits()
        for table, normal in planes:
            check_table(target, self.hand_points, table, normal, stage, minimum_gap=minimum_gap)

    def move_remote(self, step):
        self.check()
        stage, trajectory = step
        self.active['stage'] = stage
        if trajectory:
            names = trajectory['joint_names']
            order = [names.index(f'fr3_joint{i}') for i in range(1, 8)]
            start = np.asarray(trajectory['points'][0]['positions'])[order]
            measured = np.asarray(self.idle()['q'])
            deviation = np.max(np.abs(measured - start))
            tolerance = math.radians(OBSERVATION_TOLERANCE_DEG)
            if deviation > tolerance:
                raise RuntimeError(f'Arm moved more than 1 degree before {stage}; trajectory withheld')
            if deviation > .0005:
                if self.active.get('grasp_retreat_mm') and stage in ('approach', 'descend'):
                    raise RuntimeError(f'Arm moved from the verified start of {stage}; shallower pick withheld')
                goal, options = self.goals[stage]
                self.q = measured.tolist()
                _, trajectory = self.prepare(stage, goal, **options)
                if trajectory is None:
                    self.active['last_completed_stage'] = stage
                    return
                names = trajectory['joint_names']
                order = [names.index(f'fr3_joint{i}') for i in range(1, 8)]
                start = np.asarray(trajectory['points'][0]['positions'])[order]
                if np.max(np.abs(np.asarray(self.idle()['q']) - start)) > .001:
                    raise RuntimeError(f'Arm moved after replanning {stage}; trajectory withheld')
            local = self.arm.local
            duration = trajectory['segments'][-1][0]
            tolerance = math.radians(.1)
            if stage in ('approach', 'descend', 'miss_retreat'):
                hand = self.fresh(self.hand)
                if hand['is_grasped'] or abs(hand['width'] - self.opening) > .001:
                    raise RuntimeError('Hand left its open position before grasp')
            def check_live_descent(state):
                actual = np.asarray(state['O_T_EE']).reshape(4, 4, order='F')
                gap = min(table_clearance(actual, self.hand_points, table, normal)
                          for table, normal in trajectory['descent_planes'])
                minimum_gap = trajectory['descent_min_gap_m']
                if not math.isfinite(gap) or gap < minimum_gap:
                    raise RuntimeError(f'Measured hand clearance {gap*1000:.2f} mm below {minimum_gap*1000:g} mm during {stage}')

            if stage in ('descend', 'miss_retreat', 'place_descend'):
                check_live_descent(self.fresh(self.arm))
            command = local.move_timed(self.active['owner'], trajectory['segments'],
                                       math.radians(self.options['speed_deg_s']), tolerance, duration + 8)
            self.wait(self.arm, command, duration + 10, 'command_id',
                      arm_feedback=check_live_descent if stage in ('descend', 'miss_retreat', 'place_descend') else None)
            target = np.asarray(trajectory['points'][-1]['positions'])[order]
            deadline = time.monotonic() + 5
            while True:
                sample = self.fresh(self.arm)
                measured = np.asarray(sample['q'])
                errors = np.abs(measured - target)
                worst = int(np.argmax(errors))
                speed = max(map(abs, sample['dq']))
                if errors[worst] <= tolerance and speed < .01:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'Arm did not settle after {stage}: J{worst+1} '
                        f'target={math.degrees(target[worst]):.3f} deg, '
                        f'actual={math.degrees(measured[worst]):.3f} deg, '
                        f'error={math.degrees(errors[worst]):.3f} deg (limit={math.degrees(tolerance):.3f} deg), '
                        f'max speed={math.degrees(speed):.3f} deg/s (limit=0.573 deg/s)'
                    )
                self.active['cancel'].wait(.05)
            if stage in ('descend', 'miss_retreat', 'place_descend'):
                check_live_descent(sample)
            self.arm_commands += 1
        self.active['last_completed_stage'] = stage

    def open_hand(self, stage):
        self.active['stage'] = stage
        hand = self.fresh(self.hand)
        opening = self.opening = min(hand['max_width'], .08)
        if opening <= 0:
            raise RuntimeError('Invalid calibrated hand opening')
        if abs(hand['width'] - opening) <= .001 and not hand['is_grasped']:
            return opening

        requested_at = time.monotonic_ns()
        command = self.hand.command(self.active['owner'], 'set_position', width_m=opening,
                                    speed_m_s=self.options['speed_mm_s']/1000, timeout_s=15)
        self.wait(self.hand, command, 17, 'hand_command_id')
        # The native success event can arrive before the matching width sample.
        deadline = time.monotonic() + .5
        while True:
            self.check()
            hand = self.hand.snapshot()
            if (hand.get('sdk_received_ns', 0) > requested_at
                    and hand.get('connected') and hand.get('control_ready')
                    and 0 <= hand['data_age_ms'] <= 220
                    and abs(hand['width'] - opening) <= .001 and not hand['is_grasped']):
                return opening
            if time.monotonic() >= deadline:
                width = hand.get('width')
                actual = f'{width*1000:.2f} mm' if width is not None else 'unavailable'
                raise RuntimeError(f'Hand opening was not confirmed (target={opening*1000:.2f} mm, '
                                   f'feedback={actual}, is_grasped={hand.get("is_grasped")})')
            self.active['cancel'].wait(.02)

    def grasp(self, width):
        self.active.update(stage='pick_close', holding_object_possible=True)
        failure = None
        for attempt in range(2):
            requested_at = time.monotonic_ns()
            command = self.hand.command(self.active['owner'], 'grasp', width_m=width,
                                        speed_m_s=self.options['speed_mm_s']/1000, force_n=self.options['force_n'],
                                        epsilon_inner_m=.010, epsilon_outer_m=.010, timeout_s=15)
            try:
                self.wait(self.hand, command, 17, 'hand_command_id')
                break
            except RuntimeError as error:
                if (self.hand.motion(command).get('status') != 'failed'
                        or str(error) != 'Gripper command returned false'):
                    raise
                measured = self.fresh(self.hand)
                if (attempt or not .005 < measured['width'] < min(measured['max_width'], .08)-.002):
                    failure = error
                    break
                width = measured['width']

        deadline = time.monotonic() + 2
        while True:
            measured = self.fresh(self.hand)
            sdk_ns = measured.get('sdk_received_ns', 0)
            if requested_at < sdk_ns and time.monotonic_ns() - sdk_ns <= 220_000_000:
                if measured['is_grasped']:
                    if failure:
                        raise failure
                    return
                if time.monotonic() >= deadline:
                    raise GraspMiss('Grasp was not confirmed')
            if time.monotonic() >= deadline:
                raise RuntimeError('Fresh grasp feedback was not confirmed; lift withheld')
            self.active['cancel'].wait(.02)
