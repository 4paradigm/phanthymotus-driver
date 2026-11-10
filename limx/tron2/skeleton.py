"""Motus sensor/skeleton contract, using official DACH/WF/SF TRON2A models."""
from pathlib import Path
import math
import hashlib
from functools import lru_cache

from reset import ARM_NAMES
from state import configuration as reported_configuration

JOINT_NAMES = ARM_NAMES + ['head_pitch_Joint', 'head_yaw_Joint']
MODEL_PATH = Path(__file__).with_name('resource') / 'tron2_dach.urdf'
ARM_LABELS = ['肩俯仰', '肩侧摆', '肩旋转', '肘', '腕偏航', '腕俯仰', '腕旋转']
JOINT_LABELS = ARM_LABELS + ARM_LABELS + ['头部俯仰', '头部偏航']


LEG_PREFIXES = ['proximal_pitch', 'proximal_roll', 'proximal_yaw', 'knee']
WF_JOINT_NAMES = [prefix + '_' + side + '_Joint'
                  for side in ('L', 'R') for prefix in LEG_PREFIXES + ['wheel']]
SF_JOINT_NAMES = [prefix + '_' + side + '_Joint'
                  for side in ('L', 'R') for prefix in LEG_PREFIXES + ['ankle_pitch']]
MODELS = {
    'upper_limb': ('DACH_TRON2A', MODEL_PATH, JOINT_NAMES, JOINT_LABELS),
    'wheel_foot': ('WF_TRON2A', MODEL_PATH.with_name('tron2_wf.urdf'), WF_JOINT_NAMES,
                   ['髋俯仰', '髋侧摆', '髋旋转', '膝', '轮'] * 2),
    'sole_foot': ('SF_TRON2A', MODEL_PATH.with_name('tron2_sf.urdf'), SF_JOINT_NAMES,
                  ['髋俯仰', '髋侧摆', '髋旋转', '膝', '踝俯仰'] * 2),
}


def selected_mode(profile):
    # The omitted argument preserves the existing pure DACH model/FK API.
    # Runtime callers always supply the actual reader configuration explicitly.
    return 'upper_limb' if profile is None else profile.get('mode', 'unknown')


@lru_cache(maxsize=4)
def model_content(path):
    urdf = path.read_text(encoding='utf-8')
    return urdf, hashlib.sha256(urdf.encode('utf-8')).hexdigest()


def model(profile=None):
    mode = selected_mode(profile)
    if mode not in MODELS:
        # Honest empty model for an unidentified configuration. This only takes
        # effect if the frontend actually fetches this driver's model resource.
        empty = '<robot name="tron2_configuration_unavailable"><link name="base_Link"/></robot>'
        return {'urdf': empty, 'model_revision': hashlib.sha256(empty.encode('utf-8')).hexdigest(),
                'available': False, 'configuration': mode, 'model_id': None}
    model_id, path, _, _ = MODELS[mode]
    urdf, revision = model_content(path)
    return {'urdf': urdf, 'model_revision': revision, 'available': True,
            'configuration': mode, 'model_id': model_id}


def skeleton_stream(sample, profile=None):
    status = {key: sample[key] for key in (
        'connected', 'available', 'sample_status', 'received_at',
        'reception_age_seconds', 'measurement_time_verified')}
    # IMU orientation is not supplied by this vendor interface. Omit the
    # optional field: [] is truthy in JS and the Motus renderer destructures
    # it as [w,x,y,z], corrupting the model root with undefined components.
    mode = selected_mode(profile)
    spec = MODELS.get(mode)
    result = {'joints': [], 'status': status, 'configuration': mode,
              'model_id': spec[0] if spec else None}
    selected_model = model(profile)
    result.update(urdf=selected_model['urdf'], model_revision=selected_model['model_revision'])
    if not sample['available']:
        return result
    if spec is None:
        status.update(available=False, sample_status='configuration_pending'
                      if profile and profile.get('state') == 'pending' else 'unsupported_configuration')
        return result
    reported = reported_configuration(sample.get('accid'))
    if reported['state'] == 'identified' and reported['mode'] != mode:
        status.update(available=False, sample_status='configuration_mismatch')
        return result
    _, _, joint_names, labels = spec
    data = sample['data']
    if len(data['q']) != len(joint_names):
        status.update(available=False, sample_status='unsupported_joint_layout')
        return result
    names = data['names'] or joint_names
    if len(names) != len(joint_names) or set(names) != set(joint_names):
        status.update(available=False, sample_status='unsupported_joint_names')
        return result
    status['name_source'] = 'vendor' if data['names'] else 'official_sdk_index_order'
    for index, name in enumerate(names):
        motor_index = joint_names.index(name)
        joint = {'idx': motor_index, 'name': name, 'q': data['q'][index],
                 'q_deg': math.degrees(data['q'][index]),
                 'group': ('左臂' if motor_index < 7 else '右臂' if motor_index < 14 else '头部')
                          if mode == 'upper_limb' else ('左腿' if motor_index < 5 else '右腿'),
                 'label': labels[motor_index]}
        for field in ('dq', 'tau'):
            if data[field]:
                joint[field] = data[field][index]
        result['joints'].append(joint)
    return result


def joints_stream(sample, profile):
    """Public Go2/G1/K1-style wire payload; model is a separate MCP resource.

    The diagnostic/legacy stream retains configuration, labels and URDF, but the
    shared renderer only needs official motor names and radian measurements.
    Do not invent an IMU quaternion or motor temperature when not reported.
    """
    packet=skeleton_stream(sample,profile)
    return {'joints':[{key:joint[key] for key in ('idx','name','q','dq','tau') if key in joint}
                      for joint in packet['joints']]}
