"""Motus sensor/skeleton contract, using the official DACH_TRON2A model."""
from pathlib import Path

from reset import ARM_NAMES

JOINT_NAMES = ARM_NAMES + ['head_pitch_Joint', 'head_yaw_Joint']
MODEL_PATH = Path(__file__).with_name('resource') / 'tron2_dach.urdf'


def model():
    return {'urdf': MODEL_PATH.read_text(encoding='utf-8')}


def skeleton_stream(sample):
    status = {key: sample[key] for key in (
        'connected', 'available', 'sample_status', 'received_at',
        'reception_age_seconds', 'measurement_time_verified')}
    result = {'joints': [], 'imu_quat': [], 'status': status}
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
        joint = {'idx': JOINT_NAMES.index(name), 'name': name, 'q': data['q'][index]}
        for field in ('dq', 'tau'):
            if data[field]:
                joint[field] = data[field][index]
        result['joints'].append(joint)
    return result
