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
from motion.profile import session_profile

PROFILE = DRIVER / 'motion/body23_fixed_hand.json'


def sample(now=1_000_000_000):
    return {'arm_ns': now, 'q': [0.]*10, 'dq': [0.]*10,
            'locked_joints': dict.fromkeys(LOCKED_NAMES, 0.)}


def test_default_registration_and_packaged_geometry_are_complete():
    config = yaml.safe_load((DRIVER/'config.yaml').read_text())
    assert 'motion_control' not in config
    assert config['teleop_control']['enabled'] is True
    assert config['teleop_control']['calibration_path'] == '/work/motion/body23_fixed_hand.json'
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
        result = arm.dispatch('stop', {})
        assert result['no_op'] and result['authority_released']
        assert not result['physical_confirmed']
        arm.tick()
        assert arm._operation is None
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


@pytest.mark.parametrize('legacy,teleop', [(False, True), (True, True), (True, False)])
def test_bundle_can_register_teleop_with_servo_cards(monkeypatch, legacy, teleop):
    import ast
    tree = ast.parse((DRIVER/'main.py').read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
    namespace = {n: object for n in ('AudioClient','RpcProxy','G1ArmActionClient','SlamClient','MotionSwitcherClient')}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(DRIVER/'main.py'), 'exec'), namespace)
    class Arm:
        PREFIX = 'arm'
        def __init__(self, *args): pass
    class Stub:
        def __init__(self, *args, **kwargs): pass
    for name, attrs in {
        'device': {'ArmActionPlugin': Arm, '_loco_acp_notify': lambda *a, **k: None},
        'arm_stream': {'ArmStreamExecutor': Stub},
        'motion_control': {'MotionControl': Stub},
        'teleop_control': {'TeleopControl': Stub},
        'teleop_bus': {'TeleopBus': Stub},
        'teaching': {'teaching_plugin': lambda control: Stub()},
        'servo': {'G1ServoPlugin': Stub},
        'servo_eef': {'G1ServoEefPlugin': Stub},
    }.items():
        monkeypatch.setitem(sys.modules, name, SimpleNamespace(**attrs))
    config = {'plugins': {name: {'enabled': True} for name in ('arm','servo','servo_eef')},
              'teleop_control': {'enabled': teleop}, 'motion_control': {'enabled': legacy}}
    config['plugins']['smart_motion'] = {'enabled': False}
    if legacy:
        with pytest.raises(ValueError, match='motion_control is removed; use teleop_control'):
            namespace['G1DeviceBundle'](config, 'offline', None, None, None, None, None)
        return
    bundle = namespace['G1DeviceBundle'](config, 'offline', None, None, None, None, None)
    assert len(bundle._plugins) == 5
    assert bundle._plugins[0]._teleop_control is bundle._plugins[1]


@pytest.mark.parametrize('failure', [None, 'cards', 'bus', 'safety', 'loco', 'executor'])
def test_bundle_shutdown_cleans_all_owners_even_after_failure(failure):
    import ast
    tree = ast.parse((DRIVER/'main.py').read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    cleanup = main.body[-1].finalbody
    calls = []
    def close(name):
        def run():
            calls.append(name)
            if failure == name: raise RuntimeError(name)
        return run
    executor = SimpleNamespace(shutdown=close('executor'))
    bus = SimpleNamespace(shared_executor=executor, close=close('bus'))
    namespace = {'_bundle': SimpleNamespace(stop_all=close('cards'), _motion_bus=bus),
                 'smart_motion': SimpleNamespace(shutdown=close('safety')),
                 'loco_client': SimpleNamespace(stop=close('loco')),
                 'executor': executor, 'rclpy': SimpleNamespace(shutdown=close('ros'))}
    code = compile(ast.Module(body=cleanup, type_ignores=[]), str(DRIVER/'main.py'), 'exec')
    if failure:
        with pytest.raises(RuntimeError, match=failure): exec(code, namespace)
    else:
        exec(code, namespace)
    assert calls == ['cards', 'bus', 'safety', 'loco', 'executor', 'ros']


@pytest.mark.parametrize('state,teleop,loco', [(s,t,l) for s in (False,True) for t in (False,True) for l in (False,True)])
def test_bundle_owns_one_source_independent_of_display_and_arm(monkeypatch, state, teleop, loco):
    import ast
    tree = ast.parse((DRIVER/'main.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
    namespace = {n:object for n in ('AudioClient','RpcProxy','G1ArmActionClient','SlamClient')}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(DRIVER/'main.py'), 'exec'), namespace)
    events, sources, displays, arms = [], [], [], []
    class Source:
        def __init__(self): sources.append(self)
        def start(self): events.append('source_start')
        def stop(self): events.append('source_stop')
    class Plugin:
        PREFIX = 'other'
        def __init__(self, *args, **kwargs): self.node = object()
        def start(self): events.append('consumer_start')
        def stop(self): events.append('consumer_stop')
    class Arm(Plugin): PREFIX = 'arm'
    class State(Plugin):
        def __init__(self, *args, low_state):
            super().__init__(); self.low_state = low_state; displays.append(self)
    class Stream:
        def __init__(self, *args, low_state): self.low_state = low_state; arms.append(self)
    for name, attrs in {
        'low_state': {'LowStateSource':Source},
        'device': {'ArmActionPlugin':Arm, 'StatePlugin':State, 'LocoStatePlugin':Plugin,
                   'LocoPlugin':Plugin, '_loco_acp_notify':lambda *a,**k:None},
        'arm_stream': {'ArmStreamExecutor':Stream},
        'motion_control': {'MotionControl':Plugin},
        'teleop_control': {'TeleopControl':Plugin},
        'teleop_bus': {'TeleopBus':Plugin},
        'teaching': {'teaching_plugin':lambda control: Plugin()},
    }.items(): monkeypatch.setitem(sys.modules, name, SimpleNamespace(**attrs))
    cfg = {'plugins':{'state':{'enabled':state}, 'loco':{'enabled':loco}, 'arm':{'enabled':teleop},
                      'smart_motion':{'enabled':False}}, 'teleop_control':{'enabled':teleop}}
    bundle = namespace['G1DeviceBundle'](cfg,'offline',None,None,None,None,None)
    assert len(sources) == int(state or teleop)
    assert len(displays) == int(state) and len(arms) == int(teleop)
    assert all(c.low_state is bundle._low_state for c in displays+arms)
    bundle.start_all(); bundle.stop_all()
    if sources: assert events[0] == 'source_start' and events[-1] == 'source_stop'
    else: assert not any(e.startswith('source') for e in events)
    if sources and bundle._plugins:
        events.clear()
        bundle._plugins[-1].stop = lambda: (_ for _ in ()).throw(RuntimeError('stop_unconfirmed'))
        with pytest.raises(RuntimeError, match='stop_unconfirmed'): bundle.stop_all()
        assert 'source_stop' not in events
