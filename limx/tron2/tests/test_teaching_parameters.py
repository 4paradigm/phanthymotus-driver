"""Persisted per-skill parameters and native Core execution grants; no hardware."""
import asyncio
import json

import pytest

from device import Adapter
from teaching import TronTeachingPlugin
from teaching_replay import gripper_parameters
from test_sensors import settings
from test_teaching_episode import BINDING, episode
from test_teaching_runtime import Reader


def test_saved_parameters_survive_restart_and_only_explicit_values_override(tmp_path, monkeypatch):
    constructed=[];reports=[];grants={}
    class Worker:
        phase='replaying'
        def __init__(self,url,frames,head,initial,speed,force):constructed.append((speed,force))
        def run(self):
            return dict(status='completed',physical_stop_verified=True,output_stopped=True,motion_may_continue=False)
    monkeypatch.setattr('teaching_replay.SDKReplay',Worker)
    def policy(tool,instance_id,device_binding,asset_id,asset_binding):
        enabled=grants.get(asset_id)==(device_binding,asset_binding)
        return dict(enabled=enabled,source='skill' if enabled else 'manual',binding_changed=not enabled)
    monkeypatch.setattr('common.teaching.service.read_execution_policy',policy)
    async def run():
        reader=Reader();reader.settings={'url':'ws://127.0.0.1:5000'}
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        owner=Adapter(lambda *a:None,cfg,reader=reader)
        adapter=owner.teaching.adapter;adapter.capture.binding=lambda:BINDING
        async def report(*args):reports.append(args);return True
        owner._report_completion=report
        writer=episode(tmp_path/'episodes')
        plugin=owner.teaching;assets=[]
        for speed,force in [(50,0),(30,20)]:
            draft=await asyncio.to_thread(plugin.dispatch,'new',dict(filename=writer.ident))
            plugin.draft['trial']=dict(status='completed',result=dict(
                execution_parameters=dict(gripper_speed=speed,gripper_force=force)))
            saved=await asyncio.to_thread(plugin.dispatch,'save',dict(draft_id=draft['id'],
                effect_confirmed=True,name=f'action-{speed}',conditions='fixed workspace'))
            assets.append(saved['asset_id'])
        # A new instance reads defaults and permission bindings from the saved assets.
        plugin=TronTeachingPlugin(adapter)
        for ident in assets:
            binding=plugin._execution_binding(ident)
            grants[ident]=(binding['device_binding'],binding['asset_binding'])
        for ident,override,expected in [(assets[0],{},(50,0)),(assets[1],{},(30,20)),
                                        (assets[1],{'gripper_force':0},(30,0))]:
            preview=await asyncio.to_thread(plugin.dispatch,'preview',dict(asset_id=ident,**override))
            assert preview['confirmation_required'] is False
            assert preview['execution_arguments']['exclusive_control_confirmed'] is True
            args=preview['execution_arguments']
            assert (args['gripper_speed'],args['gripper_force'])==expected
            result=await asyncio.to_thread(plugin.dispatch,'run',args)
            await asyncio.to_thread(plugin.observer.join,3)
            assert plugin.last_run['action_id']==result['action_id']
            assert plugin.last_run['status']=='completed' and plugin.pending is None
            assert constructed[-1]==expected
        persisted=json.loads((plugin.directory/(assets[1]+'.json')).read_text())
        assert persisted['execution_parameters']==dict(gripper_speed=30,gripper_force=20)
        assert len(reports)==3 and not reader.commands
        # Revocation after preview still prevents execution, even with an old argument set.
        preview=await asyncio.to_thread(plugin.dispatch,'preview',dict(asset_id=assets[0]))
        del grants[assets[0]]
        with pytest.raises(ValueError,match='user_confirmation_required'):
            await asyncio.to_thread(plugin.dispatch,'run',preview['execution_arguments'])
        manual=await asyncio.to_thread(plugin.dispatch,'preview',dict(asset_id=assets[0]))
        assert manual['confirmation_required'] and manual['execution_arguments']['gripper_force']==0
        with pytest.raises(ValueError,match='invalid_teaching_argument__execution_context'):
            plugin.dispatch('run',dict(asset_id=assets[0],_execution_context=BINDING))
        # Changing saved defaults invalidates the old native grant, without rewriting trial evidence.
        previous_trial=persisted['trial']
        await asyncio.to_thread(plugin.dispatch,'parameters',dict(asset_id=assets[1],gripper_speed=40))
        changed=json.loads((plugin.directory/(assets[1]+'.json')).read_text())
        assert changed['execution_parameters']==dict(gripper_speed=40,gripper_force=20)
        assert changed['trial']==previous_trial
        preview=await asyncio.to_thread(plugin.dispatch,'preview',dict(asset_id=assets[1]))
        assert preview['confirmation_required'] and preview['execution_parameters']['gripper_speed']==40
        assert len(constructed)==3
    asyncio.run(run())


@pytest.mark.parametrize('value',[None,True,-1,101,float('nan'),float('inf'),'50'])
@pytest.mark.parametrize('key',['gripper_speed','gripper_force'])
def test_parameter_defaults_and_overrides_reject_invalid_values(key,value):
    with pytest.raises(ValueError):gripper_parameters({**dict(gripper_speed=50,gripper_force=0),key:value})


def test_zero_force_is_valid_but_zero_speed_is_not():
    assert gripper_parameters(dict(gripper_speed=50,gripper_force=0))['gripper_force']==0
    with pytest.raises(ValueError,match='positive'):
        gripper_parameters(dict(gripper_speed=0,gripper_force=20))
