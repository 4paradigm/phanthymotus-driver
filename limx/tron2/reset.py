"""Read-only fixed home pose; execute is dispatched by the shared MoveJ path."""
import hashlib
import json
import math
from pathlib import Path

from vendor import validate_movej

ARM_NAMES = ['proximal_pitch_L_Joint', 'proximal_roll_L_Joint', 'proximal_yaw_L_Joint',
             'elbow_L_Joint', 'wrist_yaw_L_Joint', 'wrist_pitch_L_Joint', 'wrist_roll_L_Joint',
             'proximal_pitch_R_Joint', 'proximal_roll_R_Joint', 'proximal_yaw_R_Joint',
             'elbow_R_Joint', 'wrist_yaw_R_Joint', 'wrist_pitch_R_Joint', 'wrist_roll_R_Joint']


RESET_SPEED_SCALE = 1.2


def duration_for_change(delta_degrees):
    """Shared pose-preparation/arm reset timing;20% faster than the old rule."""
    if type(delta_degrees) not in (int,float) or not math.isfinite(delta_degrees) or delta_degrees<0:
        raise ValueError('Reset requires a finite non-negative joint change; no motion sent')
    return max(8,math.ceil(delta_degrees/5))/RESET_SPEED_SCALE



class ResetPose:
    def __init__(self, path, reader):
        self.path, self.reader = Path(path), reader

    def load(self):
        try:
            with self.path.open('rb') as file:
                raw = file.read(16385)
            if len(raw) > 16384:
                raise ValueError('Home pose file is too large')
            pose = json.loads(raw)
        except (OSError, ValueError, UnicodeError) as exc:
            raise ValueError('Configure a valid saved home.json on the Jetson; no motion sent') from exc
        if not isinstance(pose, dict) or pose.get('schema') != 'tron2-arms-reset-pose-v1' \
                or pose.get('scope') != 'arms_only' or pose.get('unit') != 'rad' \
                or pose.get('joint_names') != ARM_NAMES \
                or pose.get('head_policy') != 'unchanged' \
                or pose.get('gripper_policy') != 'unchanged' \
                or not isinstance(pose.get('accid'), str) or not pose['accid'] \
                or not isinstance(pose.get('firmware'), str) or not pose['firmware']:
            raise ValueError('Invalid home pose identity, format, units or arm order; no motion sent')
        target, _ = validate_movej(pose.get('target_q'), 8)
        return pose, target, hashlib.sha256(raw).hexdigest()

    def call(self, action):
        if action == 'info':
            try:
                _, target, digest = self.load()
                return {'configured': True, 'joint_count': len(target), 'pose_sha256': digest,
                        'motion_sent': False}
            except ValueError as exc:
                return {'configured': False, 'error': str(exc), 'motion_sent': False}
        if action != 'execute':
            raise ValueError('Unsupported reset action')
        pose, target, digest = self.load()
        status = self.reader.sample('robot_status')
        joint = self.reader.sample('joint_state')
        if not status['available'] or not joint['available']:
            raise ValueError('Fresh robot status and joint samples are required; no motion sent')
        if pose['accid'] != self.reader.serial \
                or pose['firmware'] != status['data'].get('sw_version'):
            raise ValueError('Home pose robot identity or firmware mismatch; no motion sent')
        current = joint['data']['q']
        if len(current) < 14:
            raise ValueError('Expected at least 14 arm joint positions; no motion sent')
        names = joint['data']['names']
        if names:
            if not set(ARM_NAMES) <= set(names):
                raise ValueError('Unsupported named arm feedback; no motion sent')
            current = [current[names.index(name)] for name in ARM_NAMES]
        delta = max(abs(math.degrees(a-b)) for a, b in zip(current[:14], target))
        # Share the same20% faster timing with automatic pose preparation.
        duration = duration_for_change(delta)
        validate_movej(target, duration)
        result = {'action': action, 'motion_sent': False, 'target_q': target,
                  'duration_seconds': duration, 'max_joint_change_deg': delta,
                  'pose_sha256': digest, 'head_policy': 'unchanged',
                  'gripper_policy': 'unchanged', 'path_validated': False,
                  'home_target': 'natural_arms_down'}
        return result
