"""DACH_TRON2A motor order and limits; no hardware or numerical runtime."""
import hashlib
import math
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

MODEL = Path(__file__).with_name('resource') / 'tron2_dach.urdf'
MODEL_SHA256 = '4472bebd4188911c189f95414a182f5fb66cceea3456c72c76a9a6e5654b5e2c'
ARM_NAMES = [f'{joint}_{side}_Joint' for side in ('L', 'R') for joint in
             ('proximal_pitch', 'proximal_roll', 'proximal_yaw', 'elbow',
              'wrist_yaw', 'wrist_pitch', 'wrist_roll')]
JOINT_NAMES = ARM_NAMES + ['head_pitch_Joint', 'head_yaw_Joint']
# SDK 3.6.1/3.6.2; intersect with URDF limits (notably the elbow differs).
VENDOR_LOWER = [-3.1416, -.2618, -3.6652, -2.6180, -1.7453, -.7854, -1.5708,
                -3.1416, -3.1940, -1.4835, -2.6180, -1.3963, -.7854, -1.5708, -.78, -1.57]
VENDOR_UPPER = [2.6005, 3.1940, 1.4835, .2618, 1.3963, .7854, 1.5708,
               2.6005, .2618, 3.6652, .2618, 1.7453, .7854, 1.5708, 1.04, 1.57]


def numbers(values, size):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError('invalid_joint_count')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('invalid_joint_number')
    return list(values)


@lru_cache(maxsize=1)
def limits():
    raw = MODEL.read_bytes()
    if hashlib.sha256(raw).hexdigest() != MODEL_SHA256:
        raise ValueError('tron2_model_mismatch')
    tree = ET.fromstring(raw)
    lower, upper = [], []
    for index, name in enumerate(JOINT_NAMES):
        limit = tree.find(f"joint[@name='{name}']/limit")
        lower.append(max(float(limit.attrib['lower']), VENDOR_LOWER[index]))
        upper.append(min(float(limit.attrib['upper']), VENDOR_UPPER[index]))
    return tuple(lower), tuple(upper)


def validate_q(values, size=16):
    q = numbers(values, size)
    lower, upper = limits()
    if any(not lo <= v <= hi for v, lo, hi in zip(q, lower, upper)):
        raise ValueError('joint_limit')
    return q


def feedback_q(data):
    if not isinstance(data, dict) or data.get('result') != 'success':
        raise ValueError('joint_feedback_failed')
    q = numbers(data.get('q'), 16)
    names = data.get('names', [])
    if names:
        if not isinstance(names, list) or len(names) != 16 or set(names) != set(JOINT_NAMES):
            raise ValueError('unsupported_joint_names')
        q = [q[names.index(name)] for name in JOINT_NAMES]
    elif names != []:
        raise ValueError('invalid_joint_names')
    return validate_q(q)
