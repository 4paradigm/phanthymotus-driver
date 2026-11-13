"""PR closure regressions: strict lease, revocation wakeup and authority diagnostics."""
import threading
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from sound_facing import FacingConfig, SourceHealthProbe
from sound_facing_authority import OperatorAuthority
from sound_authorization_fixtures import approved_authority, signed
from test_sound_facing import Clock, Motion
from sound_facing import SoundFacing


def test_session_lease_cannot_exceed_thirty_seconds():
    assert FacingConfig.schema_properties()['session_timeout_s']['maximum'] == 30
    for value in (30.001, 31, 60):
        with pytest.raises(ValueError, match='session_timeout_s'):
            FacingConfig.parse({'session_timeout_s': value})
    assert FacingConfig.parse({'session_timeout_s':30}).session_timeout_s == 30


def test_invalidation_wakes_waiters_without_waiting_for_external_callback():
    entered, release = threading.Event(), threading.Event()
    def callback():
        entered.set()
        release.wait(2)
        return True
    probe=SourceHealthProbe(callback)
    result=[]
    reader=threading.Thread(target=lambda:result.append(probe.read()))
    try:
        reader.start()
        assert entered.wait(1)
        query=probe._query
        probe.invalidate()
        assert query['done'].is_set(), 'revocation must wake query waiters immediately'
        reader.join(.2)
        assert not reader.is_alive() and result == [False]
        assert probe.worker.is_alive(), 'external callback is bounded, not force-killed'
    finally:
        release.set();reader.join(1)
        if probe.worker:probe.worker.join(1)


def test_missing_pin_is_safe_denial_and_valid_signature_resolves_software_path():
    clock=Clock()
    unconfigured=OperatorAuthority('sim',public_key_path='',monotonic=clock.mono)
    engine=SoundFacing(Motion(),monotonic=clock.mono,wall_ns=clock.wall,authority=unconfigured)
    engine.start()
    out=engine.enable()
    assert out['error']=='operator_authorization_unavailable'
    assert out['authorization']['error']=='operator_public_key_unconfigured'
    assert not engine.reserved
    authority=approved_authority(clock)
    engine=SoundFacing(Motion(),monotonic=clock.mono,wall_ns=clock.wall,authority=authority)
    engine.start()
    assert engine.enable()['enabled'] is True
    session=engine.session_id
    authority.invalidate()
    assert not engine._motion_authorized(session)
    engine.tick()
    assert not engine.session_id and not engine.reserved
    assert engine.renew(session)['error']=='session_not_active_or_mismatched'


@pytest.mark.parametrize('angle,expected',[(0,0),(90,90),(-90,-90),(180,180),(-180,-180),(179,179),(-179,-179)])
def test_body_frame_angles_and_capture_windows(angle,expected):
    from sound_facing import DirectionGate
    from test_sound_facing import observation
    clock=Clock();gate=DirectionGate(FacingConfig())
    result=None
    for _ in range(3):
        clock.advance(.25)
        result,reason=gate.observe(observation(clock,angle),clock.wall()//1000,clock.epoch)
    assert result==pytest.approx(expected) and reason=='stable'


def test_arm64_context_contains_runtime_and_excludes_operator_and_tests(tmp_path):
    import importlib.util
    tool=Path(__file__).parents[1]/'tools/build_sound_facing_arm64.py'
    spec=importlib.util.spec_from_file_location('bumi_arm64_builder',tool)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    context,receipt=mod.prepare(tmp_path/'new-build')
    assert receipt['built'] is False and receipt['pushed'] is False
    for name in ('sound_facing.py','sound_facing_card.py','sound_facing_authority.py','common'):
        assert (context/name).exists()
    assert not (context/'tools').exists() and not (context/'tests').exists()
    assert '-r /work/requirements.sound-facing.lock' in (context/'Dockerfile').read_text()
    assert 'cryptography==50.0.2' in (context/'requirements.sound-facing.lock').read_text()


@pytest.mark.parametrize('bearing',[0,45,-45,90,-90,179,-179])
def test_existing_acoustic_estimator_and_card_use_the_same_body_sign(bearing):
    import math
    from sound_direction import estimate_angle
    from sound_facing import delta_deg
    front=(4.,0.,0.);right=(0.,4.,0.)
    # The measured right basis is clockwise, while body yaw is CCW-positive.
    radians=math.radians(bearing)
    signature=(4*math.cos(radians),-4*math.sin(radians),0.)
    raw=estimate_angle(signature,front,right)
    normalized=raw if raw<=180 else raw-360
    assert abs(delta_deg(normalized,bearing))<.01


def test_build_failure_cleans_only_the_builder_created_for_this_run(tmp_path,monkeypatch):
    import importlib.util,json
    tool=Path(__file__).parents[1]/'tools/build_sound_facing_arm64.py'
    spec=importlib.util.spec_from_file_location('bumi_builder_cleanup',tool)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    commands=[]
    def run(command,log,**kwargs):
        commands.append(command)
        if command[1:3]==['buildx','build']:raise RuntimeError('simulated builder failure')
    monkeypatch.setattr(mod,'run',run)
    monkeypatch.setattr(sys,'argv',[str(tool),'--output',str(tmp_path/'build')])
    assert mod.main()==1
    created=next(cmd[cmd.index('--name')+1] for cmd in commands if cmd[1:3]==['buildx','create'])
    assert commands[-1]==['docker','buildx','rm',created]
    assert not json.loads((tmp_path/'build/BUILD.json').read_text())['built']
    assert all('prune' not in cmd and '--push' not in cmd for cmd in commands)
