"""Fictional deployment and provider fixtures only; no ROS/robot/network I/O."""
import copy
import hashlib
import json
import os
from pathlib import Path

import pytest

from deep_robotics.s10_pro.control_runtime import (
    SCHEMA, RuntimeBinding, contract_fingerprint, load_runtime_binding, read_authorization, BindingError,
)
from test_s10_action_contract import fixture_contract


class Clock:
    wall = 1000.
    monotonic = 10.
    def time(self): return self.wall
    def mono(self): return self.monotonic


@pytest.fixture
def setup_binding(tmp_path):
    clock = Clock()
    contract = fixture_contract('stand', 'velocity')
    contract.update(robot_identity='FICTIONAL_ROBOT', firmware_identity='FICTIONAL_FIRMWARE')
    lease = {'owned': True, 'fresh': True, 'age_s': .1, 'received_monotonic': 9.9,
             'expires_monotonic': 10.4, 'regular_mode_confirmed': True,
             'navigation_mode_confirmed': False, 'allowed_actions': ['stand', 'velocity'],
             'robot_identity': 'FICTIONAL_ROBOT', 'firmware_identity': 'FICTIONAL_FIRMWARE'}
    manifest = {'schema': SCHEMA, 'execution_authorized': True,
                'adapter': 'fictional.module:create', 'robot_identity': 'FICTIONAL_ROBOT',
                'firmware_identity': 'FICTIONAL_FIRMWARE',
                'contract_sha256': contract_fingerprint(contract), 'allowed_actions': ['stand'],
                'issued_at_ms': 999000, 'expires_at_ms': 1060000}
    path = tmp_path / 'approval.json'
    def write(value=None):
        path.write_text(json.dumps(manifest if value is None else value))
        path.chmod(0o600)
    write()
    calls, closed = [], []
    def loader(reference, config):
        calls.append(reference)
        return {'contract': copy.deepcopy(contract), 'authority_provider': lambda: copy.deepcopy(lease),
                'close': lambda: closed.append(True)}
    def load():
        return load_runtime_binding({}, environ={'S10_EXECUTION_AUTHORIZATION_FILE': str(path)},
            adapter_loader=loader, wall_time=clock.time, clock=clock.mono, owner_uid=os.getuid())
    return clock, contract, lease, manifest, path, write, load, calls, closed


def test_default_or_disabled_manifest_never_imports_an_adapter(tmp_path):
    def forbidden(*args): raise AssertionError('must not import adapter')
    binding = load_runtime_binding({}, environ={}, adapter_loader=forbidden)
    assert not binding.authorized and not binding.authorized_now()
    assert binding.public_status()['reason'] == 'authorization_not_configured'
    path = tmp_path/'disabled.json'
    path.write_text(json.dumps({'schema':SCHEMA,'execution_authorized':False}))
    path.chmod(0o600)
    binding = load_runtime_binding({}, environ={'S10_EXECUTION_AUTHORIZATION_FILE':str(path)},
        adapter_loader=forbidden, owner_uid=os.getuid())
    assert binding.public_status()['reason'] == 'authorization_disabled'


def test_exact_contract_machine_scope_and_original_authority_time_are_bound(setup_binding):
    clock, _, lease, _, _, _, load, calls, closed = setup_binding
    binding = load()
    assert binding.authorized and binding.authorized_now() and calls == ['fictional.module:create']
    assert set(binding.contract['actions']) == {'stand'}
    observed = binding.authority_provider()
    assert observed['allowed_actions'] == ['stand']
    assert observed['received_monotonic'] == 9.9 and observed['expires_monotonic'] == 10.4
    clock.monotonic += .05
    observed = binding.authority_provider()
    assert observed['received_monotonic'] == 9.9 and observed['age_s'] == pytest.approx(.15)
    clock.monotonic += .4
    assert binding.authority_provider() == {}
    assert binding.public_status()['authority_observation'] == 'authority_observation_expired'
    binding.close()
    assert closed == [True] and not binding.authorized_now()


@pytest.mark.parametrize('mutation', ['digest','robot','firmware','scope','expiry','future','unknown','mode','uid','symlink'])
def test_bad_authorization_never_grants_execution_or_leaks_secrets(setup_binding, mutation):
    clock, _, _, manifest, path, write, load, calls, _ = setup_binding
    if mutation == 'digest': manifest['contract_sha256'] = '0'*64
    elif mutation == 'robot': manifest['robot_identity'] = 'OTHER'
    elif mutation == 'firmware': manifest['firmware_identity'] = 'OTHER'
    elif mutation == 'scope': manifest['allowed_actions'] = ['stand','charging']
    elif mutation == 'expiry': manifest['expires_at_ms'] = 1000000
    elif mutation == 'future': manifest['issued_at_ms'] = 1000101
    elif mutation == 'unknown': manifest['SECRET_VALUE'] = 'must-not-leak'
    write()
    if mutation == 'mode': path.chmod(0o666)
    if mutation == 'uid':
        with pytest.raises(BindingError):
            read_authorization(path, wall_time=clock.time, owner_uid=os.getuid()+1)
        return
    if mutation == 'symlink':
        target=path.with_name('real.json');path.rename(target);path.symlink_to(target)
    binding=load()
    assert not binding.authorized and not binding.authorized_now() and binding.authority_provider()=={}
    assert 'must-not-leak' not in json.dumps(binding.public_status())
    assert 'SECRET_VALUE' not in json.dumps(binding.public_status())
    if mutation not in ('digest','robot','firmware'):
        assert not calls


@pytest.mark.parametrize('mutation', ['identity','firmware','owned','future_receipt','expired','clock','scope'])
def test_authority_source_must_be_real_current_machine_scoped_snapshot(setup_binding, mutation):
    _, _, lease, _, _, _, load, _, _ = setup_binding
    binding=load()
    if mutation=='identity': lease['robot_identity']='OTHER'
    elif mutation=='firmware': lease['firmware_identity']='OTHER'
    elif mutation=='owned': lease['owned']=False
    elif mutation=='future_receipt': lease['received_monotonic']=10.1
    elif mutation=='expired': lease['expires_monotonic']=10.
    elif mutation=='clock': lease['age_s']=.49
    elif mutation=='scope': lease['allowed_actions']='stand'
    assert binding.authority_provider()=={}


def test_revocation_is_sticky_and_clock_rollback_cannot_extend_approval(setup_binding):
    clock, _, _, manifest, path, write, load, _, _ = setup_binding
    binding=load()
    original=path.read_bytes()
    manifest['allowed_actions']=['velocity'];write()
    assert not binding.authorized_now()
    path.write_bytes(original)
    assert not binding.authorized_now()
    fresh=load()
    clock.wall=900.
    clock.monotonic+=61.
    assert not fresh.authorized_now()


def test_changed_contract_is_rejected_and_adapter_is_closed(setup_binding):
    _, contract, _, _, _, _, load, _, closed=setup_binding
    contract['actions']['stand']['max_uncontrolled_s']=.3
    binding=load()
    assert binding.public_status()['reason']=='contract_digest_mismatch'
    assert closed==[True]


def test_provider_cannot_modify_approval_during_snapshot_and_keep_the_grant(setup_binding):
    _,_,lease,manifest,_,write,load,_,_=setup_binding
    binding=load()
    def revoke():
        manifest['execution_authorized']=False;write()
        return copy.deepcopy(lease)
    binding._authority=revoke
    assert binding.authority_provider()=={}
    assert not binding.authorized_now()


def test_malformed_disabled_manifest_cannot_take_read_only_sensor_startup_down(setup_binding):
    _,_,_,_,_,write,load,calls,_=setup_binding
    write({'schema':SCHEMA,'execution_authorized':False,'expires_at_ms':'invalid'})
    binding=load()
    assert not binding.authorized
    assert binding.public_status()['reason']=='disabled_authorization_fields_invalid'
    assert calls==[]


def test_production_main_uses_bound_authorization_across_three_layers_without_starting_io(tmp_path, monkeypatch):
    import importlib.util
    import sys
    import time
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'deep_robotics/s10_pro'))
    spec=importlib.util.spec_from_file_location('s10_runtime_binding_integration',Path(__file__).resolve().parents[1]/'deep_robotics/s10_pro/main.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    now=time.monotonic()
    contract=fixture_contract('velocity')
    contract.update(robot_identity='ISOLATED',firmware_identity='ISOLATED')
    manifest={'schema':SCHEMA,'execution_authorized':True,'adapter':'fictional.adapter:create',
        'robot_identity':'ISOLATED','firmware_identity':'ISOLATED','contract_sha256':contract_fingerprint(contract),
        'allowed_actions':['velocity'],'issued_at_ms':int(time.time()*1000)-100,'expires_at_ms':int(time.time()*1000)+10000}
    path=tmp_path/'integration.json';path.write_text(json.dumps(manifest));path.chmod(0o600)
    observed={'owned':True,'fresh':True,'age_s':0.,'received_monotonic':now,'expires_monotonic':now+.5,
        'robot_identity':'ISOLATED','firmware_identity':'ISOLATED','navigation_mode_confirmed':True,
        'regular_mode_confirmed':False,'allowed_actions':['velocity']}
    binding=load_runtime_binding({},environ={'S10_EXECUTION_AUTHORIZATION_FILE':str(path)},owner_uid=os.getuid(),
        adapter_loader=lambda *_:{'contract':contract,'authority_provider':lambda:copy.deepcopy(observed)})
    instances=[]
    class Transport:
        def __init__(self, config, *, execution_enabled, rpc_budget_s):
            self.enabled=execution_enabled;self.budget=rpc_budget_s;instances.append(self)
        def status(self):return {'worker_running':False,'execution_enabled':self.enabled}
        def start(self, **kwargs):raise AssertionError('read-only startup must not start command worker')
        def close(self):pass
    class Bridge:
        def snapshot(self,source):
            return {'fresh':True,'age_s':time.monotonic()-now,'received_monotonic':now,'sequence':1,
                'data':{'data':{'motion_state':{'state':0},'gait_state':{'gait':0},'vel_x':0.,'vel_y':0.,'vel_yaw':0.}}}
        def status(self):return {'reader_running':False}
        def acquire(self,*args):pass
        def release(self,*args):pass
    class Notifier:
        def submit(self,*args):raise AssertionError('no task accepted')
        def status(self,*args):return {}
    monkeypatch.setattr(module,'DDSProcessTransport',Transport)
    plugins=module.build_runtime_plugins({'motion':{'enabled':True}},'fiction',Bridge(),lambda *_:None,
                                         notifier=Notifier(),runtime_binding=binding)
    try:
        motion=plugins[-1];status=motion.start()
        assert status['execution_authorized']
        assert status['readiness']['process_authorization']['granted']
        assert status['readiness']['actions']['velocity']['contract_verified']
        assert status['readiness']['actions']['velocity']['command_worker_preparable']
        assert not status['readiness']['actions']['stand']['contract_verified']
        assert instances[0].enabled and instances[0].budget==.02
        assert not status['readiness']['command_worker']['running']
        path.unlink()
        assert not motion.status()['execution_authorized']
    finally:
        for p in reversed(plugins):
            p.stop()
            if hasattr(p,'close'):p.close()
        binding.close()
