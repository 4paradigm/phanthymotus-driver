#!/usr/bin/env python3
"""Pure fixed-pose data validation and preview; contains no network transport."""
import json
import math
import os
from pathlib import Path
import time

SERIAL = os.environ.get('TRON2_SERIAL', '')
SDK = 'https://www.limxdynamics.com/zh/documents/847884267345285120'
MANUAL = 'https://www.limxdynamics.com/zh/documents/844648486841487360'
NAMES = [
    'proximal_pitch_L_Joint', 'proximal_roll_L_Joint', 'proximal_yaw_L_Joint',
    'elbow_L_Joint', 'wrist_yaw_L_Joint', 'wrist_pitch_L_Joint', 'wrist_roll_L_Joint',
    'proximal_pitch_R_Joint', 'proximal_roll_R_Joint', 'proximal_yaw_R_Joint',
    'elbow_R_Joint', 'wrist_yaw_R_Joint', 'wrist_pitch_R_Joint', 'wrist_roll_R_Joint',
]
LOWER = [-3.1416, -.2618, -3.6652, -2.6180, -1.7453, -.7854, -1.5708,
         -3.1416, -3.1940, -1.4835, -2.6180, -1.3963, -.7854, -1.5708]
UPPER = [2.6005, 3.1940, 1.4835, .2618, 1.3963, .7854, 1.5708,
         2.6005, .2618, 3.6652, .2618, 1.7453, .7854, 1.5708]


def vector(values, count, label):
    if not isinstance(values, list) or len(values) != count:
        raise ValueError(label + ': expected ' + str(count) + ' numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in values):
        raise ValueError(label + ': invalid number')
    return values


def arms(values):
    vector(values, 14, 'arms')
    for i, q in enumerate(values):
        if not LOWER[i] <= q <= UPPER[i]:
            raise ValueError('Outside SDK limits: ' + NAMES[i])
    return values


def validate_snapshot(data):
    if not SERIAL or not isinstance(data, dict) or data.get('accid') != SERIAL:
        raise ValueError('Unexpected robot serial')
    if data.get('connected') is not True or data.get('fresh') is not True:
        raise ValueError('Robot is disconnected or state is stale')
    for key in ('info_age_seconds', 'joint_age_seconds'):
        age = data.get(key)
        if isinstance(age, bool) or not isinstance(age, (int, float)) \
                or not math.isfinite(age) or not 0 <= age <= 3:
            raise ValueError('State not recent enough: ' + key)
    robot = data.get('robot')
    state = data.get('joint_state')
    if not isinstance(robot, dict) or robot.get('motor') != 'OK' or robot.get('imu') != 'OK':
        raise ValueError('Motor/IMU is not reporting OK')
    if not isinstance(robot.get('sw_version'), str) or not robot['sw_version'].strip():
        raise ValueError('Missing robot firmware identity')
    if not isinstance(state, dict) or state.get('result') != 'success':
        raise ValueError('Joint query is not successful')
    q = vector(state.get('q'), 16, 'q')
    dq = vector(state.get('dq'), 16, 'dq')
    arms(q[:14])
    names = state.get('names')
    if names not in (None, []) and names != NAMES + ['head_pitch_Joint', 'head_yaw_Joint']:
        raise ValueError('Firmware joint names conflict with the manual ordering')
    if max(abs(v) for v in dq) > .01:
        raise ValueError('Robot is moving; wait until stationary')
    stamp = state.get('robot_timestamp')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) \
            or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('Missing/invalid robot state timestamp')
    return q


def make_pose(samples):
    if len(samples) != 3:
        raise ValueError('Three fresh stationary samples required')
    positions = [validate_snapshot(s) for s in samples]
    stamps = [s['joint_state']['robot_timestamp'] for s in samples]
    if not stamps[0] < stamps[1] < stamps[2]:
        raise ValueError('Joint timestamps did not advance; cannot record cached data')
    if max(max(q[i] for q in positions) - min(q[i] for q in positions)
           for i in range(16)) > .01:
        raise ValueError('Pose changed while recording')
    return {'schema': 'tron2-arms-reset-pose-v1', 'accid': SERIAL,
            'recorded_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'firmware': samples[-1]['robot'].get('sw_version'),
            'scope': 'arms_only', 'unit': 'rad', 'joint_names': NAMES,
            'target_q': positions[-1][:14],
            'head_policy': 'unchanged', 'gripper_policy': 'unchanged',
            'mapping_source': MANUAL + ' section 1.5; ' + SDK + ' section 3.6.1',
            'mapping_basis': 'manual_index_order',
            'motion_enabled': False, 'path_validated': False}


def validate_pose(pose):
    if not isinstance(pose, dict) or pose.get('schema') != 'tron2-arms-reset-pose-v1' \
            or not SERIAL or pose.get('accid') != SERIAL or pose.get('scope') != 'arms_only' \
            or pose.get('unit') != 'rad' or pose.get('joint_names') != NAMES \
            or pose.get('head_policy') != 'unchanged' \
            or pose.get('gripper_policy') != 'unchanged' \
            or pose.get('motion_enabled') is not False:
        raise ValueError('Invalid pose format, serial, order or scope')
    if not isinstance(pose.get('firmware'), str) or not pose['firmware'].strip():
        raise ValueError('Missing recorded firmware identity')
    return arms(pose.get('target_q'))


def save_pose(path, pose):
    validate_pose(pose)
    encoded = json.dumps(pose, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    # Never overwrite a previously recorded pose.
    with Path(path).open('x', encoding='utf-8') as file:
        file.write(encoded)


def preview(pose, current):
    target = validate_pose(pose)
    q = validate_snapshot(current)
    if pose.get('firmware') != current['robot'].get('sw_version'):
        raise ValueError('Firmware changed since recording; recheck the pose')
    changes = [math.degrees(t - c) for t, c in zip(target, q[:14])]
    return {'status': 'preview_only', 'motion_sent': False,
            'scope': 'arms_only', 'head_policy': 'unchanged', 'gripper_policy': 'unchanged',
            'max_joint_change_deg': max(abs(d) for d in changes),
            'joints': [{'index': i, 'name': name, 'current_rad': q[i],
                        'target_rad': target[i], 'change_deg': changes[i]}
                       for i, name in enumerate(NAMES)],
            'path_validated': False,
            'next': 'Confirm high-level mode and teach inactivity; commission then authorize one fixed-pose test.'}
