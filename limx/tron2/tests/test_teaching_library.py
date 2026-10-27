"""Persistent recording groups and segment isolation; no robot attached."""
import asyncio

import pytest

from device import Adapter
from episode import EpisodeWriter, load_episode, atomic_json
from teaching import TronTeachingPlugin
from test_sensors import settings
from test_teaching_episode import BINDING, episode, state
from test_teaching_runtime import Reader


def test_groups_survive_restart_without_reusing_unsaved_trials(tmp_path):
    async def run():
        reader=Reader()
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        owner=Adapter(lambda *a:None,cfg,reader=reader)
        adapter=owner.teaching.adapter;adapter.capture.binding=lambda:BINDING
        plugin=owner.teaching
        writer=episode(tmp_path/'episodes')
        async def call(action,**args):return await asyncio.to_thread(plugin.dispatch,action,args)
        first=await call('new',filename=writer.ident)
        ident=plugin.record_id
        with pytest.raises(ValueError,match='successful_trial_required'):
            await call('draft_preferences',draft_id=first['id'],effect_confirmed=True)
        plugin.draft['trial']=dict(status='completed',result={'execution_parameters':{'gripper_speed':50,'gripper_force':0}})
        await call('draft_preferences',draft_id=first['id'],effect_confirmed=True,allow_auto_execute=False)
        await call('save',draft_id=first['id'],effect_confirmed=True,name='挥手',conditions='固定布置')
        second=await call('new',record_id=ident,filename=writer.ident)
        assert 'allow_auto_execute' not in second
        with pytest.raises(ValueError,match='create_new_draft'):
            await call('record',draft_id=second['id'])
        plugin.draft['trial']=dict(status='completed',result={})
        plugin._persist_draft()
        plugin=TronTeachingPlugin(adapter)
        records=(await call('library'))['records']
        assert len(records)==1 and records[0]['name']=='挥手'
        assert len(records[0]['segments'])==2
        opened=await call('record_open',record_id=ident,draft_id=second['id'])
        assert opened['draft']['trial'] is None
        opened=await call('record_open',record_id=ident,draft_id=first['id'])
        assert opened['draft']['saved']['execution_parameters']['gripper_force']==0
        assert opened['draft']['trial']['status']=='completed'
        assert opened['draft']['effect_confirmed'] and opened['draft']['allow_auto_execute'] is False
        await call('record_open',record_id=ident,draft_id=first['id'])
        # Viewing saved data does not change its recent-update ordering.
        after=(await call('library'))['records'][0]['updated_at']
        await call('record_open',record_id=ident,draft_id=first['id'])
        assert (await call('library'))['records'][0]['updated_at']==after
        with pytest.raises(ValueError,match='invalid_episode_id'):
            await call('record_open',record_id='../escape',draft_id=first['id'])
        plugin.pending={'action_id':'running'}
        with pytest.raises(RuntimeError,match='robot_busy'):
            await call('new',record_id=ident)
        with pytest.raises(RuntimeError,match='robot_busy'):
            await call('record_open',record_id=ident,draft_id=first['id'])
        assert not reader.commands
    asyncio.run(run())


def test_legacy_read_is_read_only_and_cancel_does_not_reset(tmp_path):
    async def run():
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        plugin=Adapter(lambda *a:None,cfg,reader=Reader()).teaching
        plugin.adapter.capture.binding=lambda:BINDING
        writer=episode(tmp_path/'episodes')
        records=plugin.dispatch('library',{})['records']
        assert len(records)==1 and records[0]['legacy']
        assert not (plugin.directory/'records').exists()
        opened=await asyncio.to_thread(plugin.dispatch,'record_open',dict(record_id=writer.ident,draft_id=writer.ident))
        assert opened['draft']['recording']['filename']==writer.ident
        plugin.lifecycle_started=True
        await asyncio.to_thread(plugin.dispatch,'cancel',{})
        assert plugin.lifecycle_started and plugin.teardown_future is None
        assert not plugin.adapter.owner.reader.commands
        assert 'id="home"' in plugin.dispatch('ui',{})['html']
    asyncio.run(run())


def test_record_metadata_write_survives_leftover_temp_and_failed_replace(tmp_path,monkeypatch):
    path=tmp_path/'record.json'
    path.with_suffix('.pending').write_text('interrupted write')
    atomic_json(path,{'name':'before'})
    before=path.read_bytes()
    def fail(*args):raise OSError('disk failure')
    monkeypatch.setattr('episode.os.replace',fail)
    with pytest.raises(OSError,match='disk failure'):atomic_json(path,{'name':'after'})
    assert path.read_bytes()==before
    assert not list(tmp_path.glob('.record.json.*.pending'))


def test_recording_allows_empty_task_but_export_rejects_it(tmp_path):
    writer=EpisodeWriter(tmp_path/'episodes',BINDING,'')
    for i in range(3):assert writer.put('state',state(i))
    assert writer.finish()['complete']
    assert load_episode(tmp_path/'episodes',writer.ident)[1]['task']==''
    async def run():
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        plugin=Adapter(lambda *a:None,cfg,reader=Reader()).teaching
        with pytest.raises(ValueError,match='task_required'):
            await asyncio.to_thread(plugin.dispatch,'training_export',{'episode_id':writer.ident})
        result=await asyncio.to_thread(plugin.dispatch,'annotate',dict(episode_id=writer.ident,task='放入托盘'))
        assert result['episode']['task']=='放入托盘'
        assert load_episode(tmp_path/'episodes',writer.ident)[1]['task']=='放入托盘'
    asyncio.run(run())


def test_export_receipts_retry_without_overwriting_and_detect_new_labels(tmp_path,monkeypatch):
    # The writer is stubbed here; official LeRobot decoding has its separate integration test.
    attempts=[]
    def export(root,ident,destination):
        destination.mkdir(parents=True)
        attempts.append(destination)
        if len(attempts)==1:raise ValueError('decoder failed')
        return dict(official_reader_frames=2,episodes=1,excluded=[],action_source='measured_future_state')
    monkeypatch.setattr('vla_export.export_dataset',export)
    async def run():
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        plugin=Adapter(lambda *a:None,cfg,reader=Reader()).teaching
        writer=episode(tmp_path/'episodes')
        async def call(action,**args):return await asyncio.to_thread(plugin.dispatch,action,args)
        async def finish():
            for _ in range(200):
                if plugin.export_job['state']!='running':return
                await asyncio.sleep(.01)
            raise AssertionError('export worker did not complete')
        await call('training_export',episode_id=writer.ident);await finish()
        assert plugin.export_job['state']=='error'
        plugin=TronTeachingPlugin(plugin.adapter)
        assert (await call('episode_details',episode_id=writer.ident))['export']['error']=='decoder failed'
        await call('training_export',episode_id=writer.ident);await finish()
        assert plugin.export_job['state']=='completed'
        assert attempts[0]!=attempts[1] and attempts[0].is_dir()
        plugin=TronTeachingPlugin(plugin.adapter)
        assert (await call('episode_details',episode_id=writer.ident))['export']['state']=='completed'
        result=await call('annotate',episode_id=writer.ident,task='new task')
        assert result['export']['stale']
    asyncio.run(run())
