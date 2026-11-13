"""Execute a previously checked pick/lift path through shared FR3 sessions."""
import math
import time

import numpy as np

from hardware import JOINT_LOWER, JOINT_UPPER


def arm_speed(args):
    speed = args.get('speed_deg_s')
    if type(speed) not in (int, float) or not math.isfinite(speed) or not .1 <= speed <= 5:
        raise ValueError('speed_deg_s must be supplied on the action in [0.1, 5] deg/s')
    return speed


def parameters(config, args):
    values = dict(speed_deg_s=arm_speed(args))
    for name, low, high in (('speed_mm_s', .1, 100), ('force_n', 30, 70)):
        value = args.get(name) if name == 'speed_mm_s' else config.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            location = 'supplied on the action' if name == 'speed_mm_s' else 'configured'
            raise ValueError(f'{name} must be {location} in [{low}, {high}]')
        values[name] = value
    return values


class PickExecution:
    def __init__(self, active, arm, hand, options):
        self.active, self.arm, self.hand, self.options = active, arm, hand, options
        self.deadline = time.monotonic()+1100
        self.arm_commands = 0
        self.peak_speed = 0.0

    def check(self):
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Pick execution exceeded its deadline')
        if self.active['cancel'].is_set():
            raise RuntimeError('Pick cancelled')

    def fresh(self, session):
        self.check()
        value = session.snapshot()
        age = value.get('data_age_ms')
        if (not value.get('connected') or type(age) not in (int, float) or not 0 <= age <= 200
                or not value.get('control_ready')):
            raise RuntimeError(value.get('control_error') or 'Fresh control-ready state is required')
        return value

    def wait(self, session, command, timeout, key):
        self.active[key] = command
        deadline = time.monotonic()+timeout
        while True:
            self.check()
            if key == 'command_id' and self.active.get('stage') in ('lift', 'move_to_place') and not self.fresh(self.hand)['is_grasped']:
                raise RuntimeError('Grasp feedback lost during lift')
            if key == 'command_id' and self.active.get('stage') in ('approach', 'descend'):
                hand = self.fresh(self.hand)
                if hand['is_grasped'] or abs(hand['width'] - self.opening) > .001:
                    raise RuntimeError('Hand left its open position before grasp')
            if key == 'command_id':
                state = self.fresh(self.arm)
                self.peak_speed = max(self.peak_speed, max(map(abs, state['dq'])))
            result = session.motion(command)
            if result['status'] in ('succeeded', 'failed', 'cancelled'):
                if key == 'hand_command_id' or result['status'] == 'succeeded':
                    self.active[key] = None
                if result['status'] != 'succeeded':
                    raise RuntimeError(result.get('error') or result['status'])
                return result
            if time.monotonic() >= deadline:
                raise TimeoutError('Command completion timed out')
            self.active['cancel'].wait(.02)

    def idle(self):
        deadline = time.monotonic()+2
        while True:
            value = self.fresh(self.arm)
            if value.get('robot_mode') == 1 and max(map(abs, value['dq'])) < .01:
                return value
            if value.get('robot_mode') not in (1, 2) or time.monotonic() >= deadline:
                raise RuntimeError('Arm did not settle in Idle')
            self.active['cancel'].wait(.02)

    def move(self, points):
        target = points[-1]
        value = self.idle()
        if max(abs(a-b) for a,b in zip(value['q'], self.previous)) > math.radians(.2):
            raise RuntimeError('Arm deviated from the planned path')
        command = self.arm.move_path(self.active['owner'], points,
            speed_rad_s=math.radians(self.options['speed_deg_s']),
            tolerance_rad=math.radians(.1), timeout_s=360)
        self.arm_commands += 1
        self.wait(self.arm, command, 362, 'command_id')
        value = self.idle()
        if max(abs(a-b) for a,b in zip(value['q'],target)) > math.radians(.2):
            raise RuntimeError('Arm did not reach its planned target')
        self.previous = target
        self.active['last_completed_stage'] = self.active['stage']

    def run(self, plan):
        path = np.asarray(plan['joint_path_rad'], dtype=float)
        if (path.shape != (301, 7) or not np.isfinite(path).all()
                or not np.all(path >= np.asarray(JOINT_LOWER)+.05)
                or not np.all(path <= np.asarray(JOINT_UPPER)-.05)
                or np.max(np.abs(np.diff(path,axis=0))) > math.radians(2)
                or plan.get('phase_end_indices') != [100,200,300]):
            raise ValueError('Invalid executable path')
        state = self.idle()
        self.previous = path[0].tolist()
        if max(abs(a-b) for a,b in zip(state['q'],self.previous)) > math.radians(.2):
            raise RuntimeError('Arm moved since observation; observe again')
        hand = self.fresh(self.hand)
        width = plan.get('grasp_width_mm', plan['scene']['diameter_mm'])/1000
        opening = self.opening = min(hand['max_width'], .08)
        if hand['is_grasped']:
            raise RuntimeError('Hand already reports a held object')
        if not 0 < width < opening or hand['max_width'] <= 0:
            raise ValueError('Target does not fit the calibrated opening')
        self.active['stage'] = 'pick_open'
        command = self.hand.command(self.active['owner'], 'set_position',
            width_m=opening, speed_m_s=self.options['speed_mm_s']/1000, timeout_s=15)
        self.wait(self.hand, command, 17, 'hand_command_id')
        deadline = time.monotonic()+1
        while abs(self.fresh(self.hand)['width']-opening) > .001:
            if time.monotonic() >= deadline:
                raise RuntimeError('Hand did not reach its requested opening')
            self.active['cancel'].wait(.02)
        for index, points in ((1,path[:101:5]), (101,path[100:201:5]), (201,path[200:301:5])):
            self.active['stage'] = 'approach' if index <= 100 else 'descend' if index <= 200 else 'lift'
            if index == 201:
                self.active['stage'] = 'pick_close'
                self.active['holding_object_possible'] = True
                for attempt in range(2):
                    command = self.hand.command(self.active['owner'], 'grasp',
                        width_m=width, speed_m_s=self.options['speed_mm_s']/1000,
                        force_n=self.options['force_n'], epsilon_inner_m=.010,
                        epsilon_outer_m=.010, timeout_s=15)
                    try:
                        self.wait(self.hand, command, 17, 'hand_command_id')
                    except RuntimeError as error:
                        result = self.hand.motion(command)
                        measured = self.fresh(self.hand)
                        if (attempt or result.get('status') != 'failed'
                                or str(error) != 'Gripper command returned false'
                                or not .005 < measured['width'] < opening-.002):
                            raise
                        # One retry at the measured contact opening accounts
                        # for outline error and compliant objects.
                        width = measured['width']
                        self.active['hand_command_id'] = None
                        continue
                    deadline = time.monotonic()+2
                    while not self.fresh(self.hand)['is_grasped']:
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Grasp was not confirmed; lift withheld')
                        self.active['cancel'].wait(.02)
                    break
                self.active['stage'] = 'lift'
            if index > 200 and not self.fresh(self.hand)['is_grasped']:
                raise RuntimeError('Grasp feedback lost during lift')
            self.move(points.tolist())

    def transfer(self, plan):
        opening=min(self.fresh(self.hand)['max_width'],.08)
        self.active.update(transfer_completed=False,return_completed=False)
        for index,(stage,points) in enumerate(zip(plan['stages'],plan['paths'])):
            self.active['stage']=stage
            if index==0 and not self.fresh(self.hand)['is_grasped']:
                raise RuntimeError('Grasp feedback lost before transfer')
            self.move(points)
            if index==1: self.active['transfer_completed']=True
            if index==2: self.active['return_completed']=True
            if index==0:
                self.active['rotation_completed']=bool(plan['result']['rotation_deg'])
                self.active['stage']='place_open'
                command=self.hand.command(self.active['owner'],'set_position',
                    width_m=opening,speed_m_s=self.options['speed_mm_s']/1000,timeout_s=15)
                self.wait(self.hand,command,17,'hand_command_id')
                deadline=time.monotonic()+2
                while abs(self.fresh(self.hand)['width']-opening)>.001:
                    if time.monotonic()>deadline:
                        raise RuntimeError('Release opening was not confirmed')
                    self.active['cancel'].wait(.02)
                self.active.update(holding_object_possible=False,release_completed=True)
        from . import pose_vector
        return dict(ok=True,stage='completed',**plan['result'],release_completed=True,
                    rotation_completed=bool(plan['result']['rotation_deg']),
                    transfer_completed=True,return_completed=True,recovery_required=False,
                    observe_after_transfer=self.active['config']['observe_after_transfer'],
                    observation=dict(skipped=True,reason='disabled'),grasp_checked=True,holding_object_possible=False,
                    last_completed_stage='return_to_observation',feedback_recoveries=0,
                    final_pose=pose_vector(self.fresh(self.arm)['O_T_EE']),
                    arm_path_commands=self.arm_commands,
                    peak_measured_speed_deg_s=math.degrees(self.peak_speed))

    def stop_pending(self):
        for session, key in ((self.arm, 'command_id'), (self.hand, 'hand_command_id')):
            command = self.active.get(key)
            if command:
                session.stop_motion(owner=self.active['owner'], command_id=command)
                result = session.wait(command, timeout=4)
                if result['status'] not in ('succeeded', 'cancelled'):
                    raise RuntimeError('Pending motion stop was not confirmed')
