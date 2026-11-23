"""Shared feedback and cancellation for FR3 motion."""
import math
import time

OBSERVATION_TOLERANCE_DEG = 1.0

SPEED_CONFIG = {
    'speed_deg_s': (.1, math.inf, 5, 'Arm joint speed limit (deg/s)'),
    'speed_mm_s': (.1, 100, 20, 'Gripper opening and closing speed (mm/s)'),
}


def parameters(config):
    values = {}
    limits = [(name, low, high, default)
              for name, (low, high, default, _) in SPEED_CONFIG.items()]
    for name, low, high, default in [*limits, ('force_n', 30, 70, None)]:
        value = config.get(name, default)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be finite and at least {low}' if math.isinf(high)
                             else f'{name} must be configured in [{low}, {high}]')
        values[name] = value
    return values


class PickExecution:
    def __init__(self, active, arm, hand, options):
        self.active, self.arm, self.hand, self.options = active, arm, hand, options
        self.deadline = time.monotonic()+1100
        self.planning_deadline = None
        self.arm_commands = 0
        self.peak_speed = 0.0

    def check(self):
        now = time.monotonic()
        if self.planning_deadline is not None and now >= self.planning_deadline:
            raise TimeoutError('MoveIt candidate planning deadline')
        if now >= self.deadline:
            raise TimeoutError('Pick execution exceeded its deadline')
        if self.active['cancel'].is_set():
            raise RuntimeError('Pick cancelled')

    def fresh(self, session, *, wait_for_arm=False):
        self.check()
        is_gripper = session is self.hand
        max_age_ms = 220 if is_gripper else 300
        deadline = time.monotonic() + 2 if wait_for_arm and not is_gripper else None
        waited = False
        while True:
            value = session.snapshot()
            age = value.get('data_age_ms')
            ready = value.get('connected') and value.get('control_ready')
            valid_age = type(age) in (int, float) and age >= 0
            if ready and valid_age and age <= max_age_ms:
                if waited:
                    self.active['feedback_recoveries'] = self.active.get('feedback_recoveries', 0) + 1
                return value
            if ready and valid_age and deadline is not None and time.monotonic() < deadline:
                # Only wait before a new motion. Keep the in-motion watchdog immediate.
                waited = True
                self.active['cancel'].wait(.025)
                self.check()
                continue
            device = 'Gripper' if is_gripper else 'Arm'
            detail = value.get('control_error') or value.get('error') or 'fresh control feedback required'
            if not is_gripper:
                timing = {key: value.get(key) for key in
                          ('capture_gap_ms', 'capture_to_publish_ms', 'publish_to_python_ms')}
                print(f'[fr3 feedback timeout] action={self.active.get("action_id")} '
                      f'stage={self.active.get("stage")} age_ms={age} timing={timing}', flush=True)
            raise RuntimeError(f"{device}: {detail} (connected={value.get('connected')}, "
                               f"ready={value.get('control_ready')}, age_ms={age})")

    def wait(self, session, command, timeout, key, *, arm_feedback=None):
        self.active[key] = command
        deadline = time.monotonic()+timeout
        while True:
            self.check()
            result = session.motion(command)
            if result['status'] in ('succeeded', 'failed', 'cancelled', 'rejected'):
                if key == 'hand_command_id' or result['status'] in ('succeeded', 'rejected'):
                    self.active[key] = None
                if result['status'] != 'succeeded':
                    raise RuntimeError(result.get('error') or result['status'])
                return result
            if key == 'command_id' and self.active.get('stage') in ('lift', 'move_to_place', 'place_descend') and not self.fresh(self.hand)['is_grasped']:
                raise RuntimeError('Grasp feedback lost during lift')
            if key == 'command_id' and self.active.get('stage') in ('approach', 'descend', 'miss_retreat'):
                hand = self.fresh(self.hand)
                if hand['is_grasped'] or abs(hand['width'] - self.opening) > .001:
                    raise RuntimeError('Hand left its open position before grasp')
            if key == 'command_id':
                state = self.fresh(self.arm)
                self.peak_speed = max(self.peak_speed, max(map(abs, state['dq'])))
                if arm_feedback is not None:
                    arm_feedback(state)
            if time.monotonic() >= deadline:
                raise TimeoutError('Command completion timed out')
            self.active['cancel'].wait(.02)

    def stop_pending(self):
        for session, key in ((self.arm, 'command_id'), (self.hand, 'hand_command_id')):
            command = self.active.get(key)
            if command:
                session.stop_motion(owner=self.active['owner'], command_id=command)
                result = session.wait(command, timeout=4)
                if result['status'] not in ('succeeded', 'cancelled'):
                    raise RuntimeError(f"Pending motion stop was not confirmed ({key}, "
                                       f"status={result['status']}, error={result.get('error')})")
