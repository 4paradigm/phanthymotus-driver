"""Motus sensor/skeleton contract, using official DACH/WF/SF TRON2A models."""
from pathlib import Path
import math
import hashlib
import xml.etree.ElementTree as ET
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


def _dot(a, b):
    return sum(x*y for x,y in zip(a,b))


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _unit(v):
    length = math.sqrt(_dot(v,v))
    if length < 1e-9:
        raise ValueError('Degenerate shin display reference')
    return tuple(x/length for x in v)


def _multiply(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _axis_rotation(axis, angle):
    x,y,z = _unit(axis); c,s = math.cos(angle),math.sin(angle); v=1-c
    return ((c+x*x*v,x*y*v-z*s,x*z*v+y*s),
            (y*x*v+z*s,c+y*y*v,y*z*v-x*s),
            (z*x*v-y*s,z*y*v+x*s,c+z*z*v))


def lower_leg_segments(urdf, mode, q):
    """Native-base FK: knee origin -> wheel/ankle origin, no display transform."""
    if mode not in ('wheel_foot','sole_foot') or len(q)!=10 or any(
            isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in q):
        raise ValueError('Expected 10 finite native joint positions')
    root=ET.fromstring(urdf); joints={j.get('name'):j for j in root.findall('joint')}
    segments=[]
    for side,offset in (('L',0),('R',5)):
        rotation=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.)); point=(0.,0.,0.); knee=None
        names=[prefix+'_'+side+'_Joint' for prefix in LEG_PREFIXES+['wheel' if mode=='wheel_foot' else 'ankle_pitch']]
        for index,name in enumerate(names):
            joint=joints[name]; origin=joint.find('origin')
            xyz=tuple(map(float,origin.get('xyz','0 0 0').split()))
            point=tuple(point[i]+_dot(rotation[i],xyz) for i in range(3))
            roll,pitch,yaw=map(float,origin.get('rpy','0 0 0').split())
            for axis,angle in (((0,0,1),yaw),((0,1,0),pitch),((1,0,0),roll)):
                rotation=_multiply(rotation,_axis_rotation(axis,angle))
            if index==3:knee=point
            axis=tuple(map(float,joint.find('axis').get('xyz').split()))
            rotation=_multiply(rotation,_axis_rotation(axis,q[offset+index]))
        segments.append((knee,point))
    return segments


@lru_cache(maxsize=16)
def head_up_model(path, mode, reference):
    """One fixed head-up display frame per captured pose; never rewrite live q/IMU.

    The folded pose rests on its lower legs, so the averaged knee-to-wheel/ankle
    direction is display down and the body stays above its supports (head up).
    If the legs do not point below the body, keep the native head-up frame
    instead of tipping the torso. This is a display choice, not measured
    gravity or IMU attitude.
    """
    urdf,_=model_content(path); segments=lower_leg_segments(urdf,mode,reference)
    vectors=[tuple(foot[i]-knee[i] for i in range(3)) for knee,foot in segments]
    total=tuple(vectors[0][i]+vectors[1][i] for i in range(3))
    down=_unit(total if _dot(total,total)>1e-12 else vectors[0])
    up=tuple(-x for x in down)
    if _dot(up,(0.,0.,1.))<.5:
        # Lower legs are not below the body; do not rotate the head sideways.
        up=(0.,0.,1.)
    forward=(1.,0.,0.)
    projected=tuple(forward[i]-up[i]*_dot(forward,up) for i in range(3))
    if _dot(projected,projected)<1e-9:
        forward=(0.,1.,0.)
        projected=tuple(forward[i]-up[i]*_dot(forward,up) for i in range(3))
    forward=_unit(projected)
    lateral=_unit(_cross(up,forward)); forward=_cross(lateral,up)
    rotation=(forward,lateral,up)
    pitch=math.asin(max(-1.,min(1.,-rotation[2][0])))
    if abs(math.cos(pitch))>1e-8:
        roll=math.atan2(rotation[2][1],rotation[2][2]);yaw=math.atan2(rotation[1][0],rotation[0][0])
    else:
        roll=0.;yaw=math.atan2(-rotation[0][1],rotation[1][1])
    root=ET.fromstring(urdf)
    origin=root.find("joint[@name='display_world_Joint']/origin")
    origin.set('rpy',' '.join(format(v,'.16g') for v in (roll,pitch,yaw)))
    ET.indent(root,space='  '); text=ET.tostring(root,encoding='unicode')
    return text,hashlib.sha256(text.encode()).hexdigest()


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
    reference = (profile or {}).get('display_reference_q')
    if mode in ('wheel_foot','sole_foot') and reference is not None:
        urdf, revision = head_up_model(path,mode,tuple(reference))
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
