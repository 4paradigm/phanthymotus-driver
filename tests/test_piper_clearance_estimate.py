"""Explicit model estimates retain their uncalibrated status and expire."""
from copy import deepcopy
from itertools import product
import json

import pytest

from agilex.piper.clearance_estimate import estimate, load_profile
from agilex.piper.table_clearance import TableClearance, clearance_settings


def profile_file(tmp_path):
    identity = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    joints=[]
    parent='world'
    for i in range(1,7):
        origin=deepcopy(identity)
        if i==1: origin[2][3]=.1
        child=f'link{i}'
        joints.append(dict(name=f'joint{i}',kind='revolute',parent=parent,child=child,
            origin=origin,axis=[1,0,0],mimic=None))
        parent=child
    for name,child in [('flange','flange_link'),('body','gripper_base')]:
        joints.append(dict(name=name,kind='fixed',parent=parent,child=child,origin=identity,mimic=None))
        parent=child
    joints.append(dict(name='gripper',kind='prismatic',parent='gripper_base',child='opening',
        origin=identity,axis=[0,1,0],mimic=None))
    for i,sign in [(1,1),(2,-1)]:
        joints.append(dict(name=f'finger{i}',kind='prismatic',parent='gripper_base',child=f'gripper_link{i}',
            origin=identity,axis=[0,1,0],mimic=dict(joint='gripper',multiplier=sign*.5,offset=0)))
    points=list(map(list,product([-.001,.001],repeat=3)))
    p=dict(schema='piper.gripper_estimate/1',root='world',commit='synthetic',source='synthetic-only',
        joints=joints,vertices_m={n:points for n in ['flange_link','gripper_base','gripper_link1','gripper_link2']},
        joint_bounds_deg=[[-180,180]]*6,opening_range_mm=[0,100])
    path=tmp_path/'profile.json';path.write_text(json.dumps(p))
    return path,p


def inputs(path):
    cfg=clearance_settings({'table_clearance':dict(enabled=True,mode='model_estimate',
        verified=False,can_interface='can0',estimate_profile_path=str(path),table_z_mm=0)})
    snapshot=dict(connected=True,can_interface='can0',joints_deg=[0.]*6,feedback_time=100.,
        gripper=dict(width_mm=20.,feedback_time=100.))
    return cfg,snapshot


def test_estimate_fk_opening_plane_and_no_pose_dependency(tmp_path):
    path,_=profile_file(tmp_path);cfg,snapshot=inputs(path);model=load_profile(path)
    result=estimate(snapshot,cfg,100.,model)
    assert result['clearance_mm']==pytest.approx(99.)
    assert result['state']=='estimated' and result['calibration_verified'] is False
    assert result['conservative_clearance_mm'] is None
    snapshot['joints_deg'][-1]=90.
    assert estimate(snapshot,cfg,100.,model)['clearance_mm']==pytest.approx(89.)
    snapshot['gripper']['width_mm']=40.
    assert estimate(snapshot,cfg,100.,model)['clearance_mm']==pytest.approx(79.)
    cfg['table_z_mm']=5.
    assert estimate(snapshot,cfg,100.,model)['clearance_mm']==pytest.approx(74.)
    json.dumps(result,allow_nan=False)


@pytest.mark.parametrize('change', [dict(verified=True),dict(table_z_mm=None),dict(can_interface=''),
    dict(estimate_profile_path=''),dict(mode='guess')])
def test_configuration_never_claims_estimate_is_verified(tmp_path,change):
    path,_=profile_file(tmp_path);cfg,_=inputs(path)
    with pytest.raises(ValueError):clearance_settings({'table_clearance':{**cfg,**change}})


@pytest.mark.parametrize('damage', ['axis','transform','coverage','chain','nan'])
def test_reject_bad_model(tmp_path,damage):
    path,p=profile_file(tmp_path)
    if damage=='axis':p['joints'][0]['axis']=[0,0,0]
    if damage=='transform':p['joints'][0]['origin'][3][3]=2
    if damage=='coverage':del p['vertices_m']['flange_link']
    if damage=='chain':p['joints'][0]['parent']='unknown'
    if damage=='nan':p['vertices_m']['flange_link'][0][0]=float('nan')
    path.write_text(json.dumps(p))
    with pytest.raises(ValueError):load_profile(path)


def test_estimated_card_expiry_and_failures_clear_distance(tmp_path):
    path,_=profile_file(tmp_path);cfg,snapshot=inputs(path)
    now=[100.]
    card=TableClearance(lambda:snapshot,{'table_clearance':cfg},'test',clock=lambda:now[0],monotonic=lambda:now[0])
    card.start();card.tick(card.generation)
    assert card.info()['result']['clearance_mm']==pytest.approx(99.)
    assert '估算' in card.get_tools()[0]['title']
    now[0]+=.6
    assert card.info()['result']['clearance_mm'] is None
    card.tick(card.generation)
    assert card.info()['result']['state']=='unavailable'
    now[0]=100.;snapshot['gripper']['width_mm']=101.
    card.tick(card.generation)
    assert card.info()['result']['reason']=='opening_out_of_model_range'
    snapshot['gripper']['width_mm']=20.;snapshot['joints_deg'][0]=181
    card.tick(card.generation)
    assert card.info()['result']['reason']=='joint_reference_range'
    card.stop()


def test_estimated_build_plugins_uses_shared_sdk_without_commands(tmp_path):
    from agilex.piper.device import build_plugins
    from common.vendor_runtime import DriverBundle
    from test_piper_intent import IntentDriver
    path,_=profile_file(tmp_path);cfg,_=inputs(path)
    plugins=build_plugins({'data_dir':str(tmp_path/'data'),'table_clearance':cfg},'test',None)
    runtime,background=plugins[:2];background.auto_connect=False
    driver=IntentDriver();runtime.driver=driver;driver.arm.raw=(0,0,0,0,0,0)
    card=plugins[-1]
    try:
        card.start();card.tick(card.generation)
        assert card.info()['result']['clearance_mm']==pytest.approx(99.)
        assert driver.arm.calls==[]
    finally:
        card.stop();runtime.driver=None;DriverBundle(plugins).stop_all()


def test_shipped_config_includes_working_official_model_without_site_files(tmp_path):
    from pathlib import Path
    import yaml
    from agilex.piper.device import build_plugins
    from common.vendor_runtime import DriverBundle
    from test_piper_intent import IntentDriver
    root=Path(__file__).resolve().parents[1]/'agilex/piper'
    cfg=yaml.safe_load((root/'config.yaml').read_text(encoding='utf-8'))
    cfg['data_dir']=str(tmp_path/'data')
    assert cfg['table_clearance']['enabled'] is True
    assert cfg['table_clearance']['estimate_profile_path']=='builtin:piper_100mm'
    assert cfg['table_clearance']['verified'] is False
    plugins=build_plugins(cfg,'shipped_default',None)
    runtime,background=plugins[:2];background.auto_connect=False
    driver=IntentDriver();runtime.driver=driver
    driver.arm.raw=(25344,38,-1131,10879,18161,-48470)
    driver.arm.width=99.7
    card=plugins[-1]
    try:
        card.start();card.tick(card.generation)
        result=card.info()['result']
        assert result['state']=='estimated'
        assert result['clearance_mm']==pytest.approx(112.0788112340331,abs=1e-8)
        assert result['calibration_verified'] is False
        assert result['model_commit']=='680fa925cd00db7e2628563d2edc5540963045de'
        assert result['model_sha256']=='549be81f8571e355f9f671fff2559a0a90b76b3eaaece6b769dc9a288a15d0a7'
        assert (root/'resource/clearance/LICENSE.agx_arm_urdf').is_file()
        assert driver.arm.calls==[]
        json.dumps(result,allow_nan=False)
    finally:
        card.stop();runtime.driver=None;DriverBundle(plugins).stop_all()


def test_modified_bundled_profile_rejected(monkeypatch):
    from pathlib import Path
    original=Path.read_bytes
    def tampered(path):
        data=original(path)
        return data+b' ' if path.name=='piper_100mm.json' else data
    monkeypatch.setattr(Path,'read_bytes',tampered)
    with pytest.raises(ValueError,match='bundled_estimate_profile_hash_mismatch'):
        load_profile('builtin:piper_100mm')
