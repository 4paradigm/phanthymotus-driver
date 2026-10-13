"""Motus sensor/skeleton contract, using the official DACH_TRON2A model."""
from pathlib import Path
import math

from reset import ARM_NAMES

JOINT_NAMES = ARM_NAMES + ['head_pitch_Joint', 'head_yaw_Joint']
MODEL_PATH = Path(__file__).with_name('resource') / 'tron2_dach.urdf'
ARM_LABELS = ['肩俯仰', '肩侧摆', '肩旋转', '肘', '腕偏航', '腕俯仰', '腕旋转']
JOINT_LABELS = ARM_LABELS + ARM_LABELS + ['头部俯仰', '头部偏航']


def model():
    return {'urdf': MODEL_PATH.read_text(encoding='utf-8')}


def skeleton_stream(sample):
    status = {key: sample[key] for key in (
        'connected', 'available', 'sample_status', 'received_at',
        'reception_age_seconds', 'measurement_time_verified')}
    # IMU orientation is not supplied by this vendor interface. Omit the
    # optional field: [] is truthy in JS and the Motus renderer destructures
    # it as [w,x,y,z], corrupting the model root with undefined components.
    result = {'joints': [], 'status': status}
    if not sample['available']:
        return result
    data = sample['data']
    if len(data['q']) != len(JOINT_NAMES):
        status.update(available=False, sample_status='unsupported_joint_layout')
        return result
    names = data['names'] or JOINT_NAMES
    if set(names) != set(JOINT_NAMES):
        status.update(available=False, sample_status='unsupported_joint_names')
        return result
    status['name_source'] = 'vendor' if data['names'] else 'official_sdk_index_order'
    for index, name in enumerate(names):
        motor_index = JOINT_NAMES.index(name)
        joint = {'idx': motor_index, 'name': name, 'q': data['q'][index],
                 'q_deg': math.degrees(data['q'][index]),
                 'group': '左臂' if motor_index < 7 else '右臂' if motor_index < 14 else '头部',
                 'label': JOINT_LABELS[motor_index]}
        for field in ('dq', 'tau'):
            if data[field]:
                joint[field] = data[field][index]
        result['joints'].append(joint)
    return result
