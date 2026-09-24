"""Packaged G1_23 geometry and fresh session posture; no hardware writes."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
from arm_stream import ArmStreamExecutor, LOCKED_NAMES
from g1_motion.profile import session_profile

PROFILE = DRIVER / 'g1_motion/g1_23_fixed_hand.json'


def sample(now=1_000_000_000):
    return {'arm_ns': now, 'q': [0.]*10, 'dq': [0.]*10,
            'locked_joints': dict.fromkeys(LOCKED_NAMES, 0.)}


def test_default_registration_and_packaged_geometry_are_complete():
    config = yaml.safe_load((DRIVER/'config.yaml').read_text())
    assert config['teleop_control']['enabled'] is True
    assert config['teleop_control']['calibration_path'] == '/work/g1_motion/g1_23_fixed_hand.json'
    market = yaml.safe_load((DRIVER/'driver.yaml').read_text())
    assert {'name': 'teleop_control', 'type': 'actuator'} in market['cards']
    profile = json.loads(PROFILE.read_text())
    assert 'acceptance' not in profile and 'locked_joints' not in profile
    assert profile['runtime_baseline'] == 'fresh_low_state'
    assert len(profile['arm_joint_names']) == 10
    assert hashlib.sha256((PROFILE.parent/profile['urdf_path']).read_bytes()).hexdigest() == profile['urdf_sha256']
    for side in ('left', 'right'):
        assert profile['palm_frames'][side] == {'position': [.2, 0., 0.], 'orientation': [0., 0., 0., 1.]}
        assert profile['controller_to_palm'][side] == {'position': [0., 0., 0.], 'orientation': [0., 0., 0., 1.]}


def test_default_registration_and_idle_stop_never_create_sdk_or_claim():
    cfg = yaml.safe_load((DRIVER/'config.yaml').read_text())['teleop_control']
    cfg.update(calibration_path=str(PROFILE), servo_position=True)
    calls = []
    arm = ArmStreamExecutor(cfg, 'test', SimpleNamespace(ExecuteAction=lambda x: calls.append(x)),
                            snapshot=sample, clock=lambda: 1_000_000_000)
    assert arm.profile_error is None
    assert arm._channel is None and arm.session_id is None
    for _ in range(3):
        assert arm.dispatch('stop', {})['no_op']
    assert arm._channel is None and not arm.status()['ownership_held'] and calls == []


def test_posture_materialized_fresh_each_session_without_mutating_template():
    original = PROFILE.read_bytes()
    for value in (.01, -.02):
        feedback = sample()
        feedback['locked_joints']['waist_yaw_joint'] = value
        path, directory = session_profile(PROFILE, feedback, feedback['arm_ns'])
        try:
            result = json.loads(Path(path).read_text())
            assert result['locked_joints']['waist_yaw_joint'] == value
            assert Path(result['urdf_path']).is_absolute()
            assert result['baseline_sample_ns'] == feedback['arm_ns']
            assert PROFILE.read_bytes() == original
        finally:
            directory.cleanup()
        assert not Path(path).exists()


@pytest.mark.parametrize('change,error', [
    (lambda s: s.update(arm_ns=0), 'arm_feedback_stale'),
    (lambda s: s.pop('locked_joints'), 'body_feedback_missing'),
    (lambda s: s['locked_joints'].update(waist_yaw_joint=float('nan')), 'body_feedback_invalid'),
])
def test_no_guessed_posture_when_feedback_unavailable(change, error):
    feedback = sample(); change(feedback)
    with pytest.raises(ValueError, match=error):
        session_profile(PROFILE, feedback, 1_000_000_000)


def test_bundle_can_register_teleop_with_servo_cards(monkeypatch):
    import ast
    tree = ast.parse((DRIVER/'main.py').read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
    namespace = {n: object for n in ('AudioClient','RpcProxy','G1ArmActionClient','SlamClient','MotionSwitcherClient')}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(DRIVER/'main.py'), 'exec'), namespace)
    class Arm:
        PREFIX = 'arm'
        def __init__(self, *args): pass
    class Stub:
        def __init__(self, *args): pass
    for name, attrs in {
        'device': {'ArmActionPlugin': Arm},
        'arm_stream': {'ArmStreamExecutor': Stub},
        'motion_control': {'MotionControl': Stub},
        'motion_bus': {'MotionBus': Stub},
        'teleop_control': {'TeleopControl': Stub},
        'teleop_bus': {'TeleopBus': Stub},
        'servo': {'G1ServoPlugin': Stub},
        'servo_eef': {'G1ServoEefPlugin': Stub},
    }.items():
        monkeypatch.setitem(sys.modules, name, SimpleNamespace(**attrs))
    config = {'plugins': {name: {'enabled': True} for name in ('arm','servo','servo_eef')},
              'teleop_control': {'enabled': True}}
    config['plugins']['smart_motion'] = {'enabled': False}
    bundle = namespace['G1DeviceBundle'](config, 'offline', None, None, None, None, None, None)
    assert len(bundle._plugins) == 4
    assert bundle._plugins[0]._teleop_control is bundle._plugins[1]
