"""Use MoveIt for arm motion and retain the local Franka Hand commands."""
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .execution import OBSERVATION_TOLERANCE_DEG, PickExecution
from .hand_collision import check_descent_curve, check_path_table, check_table, frame, vertices
from .kinematics import Kinematics


class RemoteExecution(PickExecution):
    def __init__(self, active, arm, hand, options, state, profile, scene=None):
        super().__init__(active, arm, hand, options)
        self.profile, self.scene = profile, scene
        self.flange_to_ee = np.asarray(state['F_T_EE']).reshape(4, 4, order='F')
        self.model = Kinematics(state['q'], np.asarray(state['O_T_EE']).reshape(4, 4, order='F'))
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

    def idle(self):
        deadline = time.monotonic() + 3
        while True:
            state = self.fresh(self.arm)
            if (state.get('robot_mode') in (1, 2) and len(state['dq']) == 7
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
        if joint:
            plan = self.arm.plan_joint(target, self.q)
        elif use_ik:
            goal = self.arm.solve_ik(self.flange_pose(target), self.q)
            plan = self.arm.plan_joint(goal, self.q)
        elif cartesian:
            plan = self.arm.plan_cartesian([self.flange_pose(p) for p in target], .1, self.q)
        else:
            plan = self.arm.plan_pose(self.flange_pose(target), self.q)
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
        plane = self.scene or self.profile.get('table_plane')
        if not plane and (self.scene or self.profile.get('tool_frame')):
            raise ValueError('Calibrated table plane is required for remote motion')
        if plane:
            table = np.asarray(plane.get('table_point_base_mm', plane.get('point_base_mm'))) / 1000
            normal = np.asarray(plane.get('table_normal_base', plane.get('normal_base')))
            if (table.shape != (3,) or normal.shape != (3,) or not np.isfinite(table).all()
                    or not np.isclose(np.linalg.norm(normal), 1., atol=.001)):
                raise ValueError('Invalid table plane')
        if stage == 'descend':
            if not plane:
                raise ValueError('A table plane is required to verify descent clearance')
            descent_planes = [(table, normal)]
            fixed = self.profile.get('table_plane')
            if fixed and fixed is not plane:
                fixed_table = np.asarray(fixed['point_base_mm']) / 1000
                fixed_normal = np.asarray(fixed['normal_base'])
                if (fixed_table.shape != (3,) or fixed_normal.shape != (3,)
                        or not np.isfinite(fixed_table).all()
                        or not np.isclose(np.linalg.norm(fixed_normal), 1., atol=.001)):
                    raise ValueError('Invalid calibrated table plane')
                descent_planes.append((fixed_table, fixed_normal))
        # Make every segment quintic, with a stationary start and end, so
        # controller interpolation is continuous through acceleration.
        for sample in samples:
            for key in ('velocities', 'accelerations'):
                if not sample[key]:
                    sample[key] = [0.] * 7
                if len(sample[key]) != 7 or not all(map(math.isfinite, sample[key])):
                    raise ValueError(f'Invalid MoveIt {key} during {stage}')
        for sample in (samples[0], samples[-1]):
            sample['velocities'] = [0.] * 7
            sample['accelerations'] = [0.] * 7
        derivative_bounds = [0., 0., 0.]
        segments = []
        from scipy.interpolate import BPoly
        for idx, (start, stop) in enumerate(zip(times, times[1:])):
            self.check()
            values = []
            for j in (idx, idx+1):
                node = [positions[j]]
                for key in ('velocities', 'accelerations'):
                    if not samples[j][key]:
                        break
                    node.append(np.asarray(samples[j][key])[order])
                values.append(node)
            if len(values[0]) != len(values[1]):
                raise ValueError('Inconsistent MoveIt interpolation data')
            curve = BPoly.from_derivatives([start, stop], values)
            if stage == 'descend':
                check_descent_curve(self.model, curve, self.hand_points, descent_planes, self.check)
            segments.append((stop, curve.c[:, 0].tolist()))
            interval = np.linspace(start, stop, max(8, int(np.ceil((stop-start)/.01))+1))
            # Bernstein coefficients bound each derivative over the interval.
            for derivative in (1, 2, 3):
                bound = float(np.max(np.abs(curve.derivative(derivative).c)))
                derivative_bounds[derivative-1] = max(derivative_bounds[derivative-1], bound)
            for q in curve(interval):
                if np.any(q < self.model.low) or np.any(q > self.model.high):
                    j = np.flatnonzero((q < self.model.low) | (q > self.model.high))[0]
                    lo, hi, actual = map(math.degrees, (self.model.low[j], self.model.high[j], q[j]))
                    raise ValueError(f'Joint {j+1} at {actual:.3f} deg outside [{lo:.3f}, {hi:.3f}] during {stage}')
                if self.model.clearance(q) <= .02:
                    raise ValueError(f'Arm self-collision clearance violated during {stage}')
                if plane:
                    check_path_table(self.model, q, self.hand_points, table, normal, stage, self.profile)
                if self.visible_surface is not None and stage in ('approach', 'descend'):
                    ee = self.model.pose(q)
                    local = (self.visible_surface - ee[:3, 3]) @ ee[:3, :3]
                    if np.any(np.all((local >= [-.03164, -.10399, -.12944])
                                     & (local <= [.03164, .10043, -.03744]), axis=1)):
                        raise ValueError(f'Hand housing intersects the target during {stage}')
        velocity, acceleration, jerk = derivative_bounds
        limits = {'original': 1., 'speed': velocity / speed * 1.01,
                  'acceleration': math.sqrt(acceleration / .3), 'jerk': jerk ** (1./3.)}
        limiting = max(limits, key=limits.get)
        factor = limits[limiting]
        print(f"[fr3 timing] action={self.active.get('action_id')} stage={stage} "
              f"rpc_s={planned-started:.3f} checks_s={time.monotonic()-planned:.3f} "
              f"speed_deg_s={math.degrees(speed):.2f} nominal_s={times[-1]:.3f} "
              f"retimed_s={times[-1]*factor:.3f} limiting={limiting} factor={factor:.2f}", flush=True)
        if times[-1] * factor > 359.:
            raise ValueError(f'Continuous position trajectory for {stage} exceeds execution time limit')
        for sample in samples:
            sample['time_ns'] = round(sample['time_ns'] * factor)
            sample['velocities'] = [v / factor for v in sample['velocities']]
            sample['accelerations'] = [v / factor**2 for v in sample['accelerations']]
        plan["segments"] = [(end * factor, controls) for end, controls in segments]
        self.q = positions[-1].tolist()
        return stage, plan

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
            if stage in ('approach', 'descend'):
                hand = self.fresh(self.hand)
                if hand['is_grasped'] or abs(hand['width'] - self.opening) > .001:
                    raise RuntimeError('Hand left its open position before grasp')
            command = local.move_timed(self.active['owner'], trajectory['segments'],
                                       math.radians(self.options['speed_deg_s']), tolerance, duration + 8)
            self.wait(self.arm, command, duration + 10, 'command_id')
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
            if stage == 'descend':
                actual = np.asarray(sample['O_T_EE']).reshape(4, 4, order='F')
                for table, keys in ((self.scene, ('table_point_base_mm', 'table_normal_base')),
                                    (self.profile.get('table_plane'), ('point_base_mm', 'normal_base'))):
                    if table:
                        check_table(actual, self.hand_points, np.asarray(table[keys[0]]) / 1000,
                                    np.asarray(table[keys[1]]), stage)
            self.arm_commands += 1
        self.active['last_completed_stage'] = stage

    def open_hand(self, stage):
        self.active['stage'] = stage
        hand = self.fresh(self.hand)
        opening = self.opening = min(hand['max_width'], .08)
        if opening <= 0:
            raise RuntimeError('Invalid calibrated hand opening')
        if abs(hand['width'] - opening) > .001 or hand['is_grasped']:
            command = self.hand.command(self.active['owner'], 'set_position', width_m=opening,
                                        speed_m_s=self.options['speed_mm_s']/1000, timeout_s=15)
            self.wait(self.hand, command, 17, 'hand_command_id')
        if abs(self.fresh(self.hand)['width'] - opening) > .001:
            raise RuntimeError('Hand opening was not confirmed')
        return opening

    def grasp(self, width):
        self.active.update(stage='pick_close', holding_object_possible=True)
        for attempt in range(2):
            command = self.hand.command(self.active['owner'], 'grasp', width_m=width,
                                        speed_m_s=self.options['speed_mm_s']/1000, force_n=self.options['force_n'],
                                        epsilon_inner_m=.010, epsilon_outer_m=.010, timeout_s=15)
            try:
                self.wait(self.hand, command, 17, 'hand_command_id')
                break
            except RuntimeError as error:
                measured = self.fresh(self.hand)
                if (attempt or self.hand.motion(command).get('status') != 'failed'
                        or str(error) != 'Gripper command returned false'
                        or not .005 < measured['width'] < min(measured['max_width'], .08)-.002):
                    raise
                width = measured['width']
                self.active['hand_command_id'] = None
        deadline = time.monotonic() + 2
        while not self.fresh(self.hand)['is_grasped']:
            if time.monotonic() >= deadline:
                raise RuntimeError('Grasp was not confirmed; lift withheld')
            self.active['cancel'].wait(.02)
