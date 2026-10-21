"""Fake vendor capture and replay boundaries; never connect to hardware."""
import asyncio
from contextlib import contextmanager
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import aiohttp
from aiohttp import web
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teaching_capture import Capture
from teaching_replay import Replay, pack_trajectory, read_trajectory
from test_teaching_episode import BINDING, jpeg, state
from skeleton import JOINT_NAMES
from episode import load_episode


class Reader:
    def __init__(self):
        self.serial='test-device';self.connected=True;self.teaching_observer=None;self._joint_poll_wake=None
        self.q=[0]*14+[.2,.3];self.grip=[0,20];self.count=0;self.commands=[]

    def sample(self,name):
        data=dict(working_mode='developer_mode',working_mode_valid=True,collision_enable_valid=True,
            compliant_enable_valid=True,collision_enable=True,compliant_enable=False,motor='OK',sw_version='test') if name=='robot_status' else dict(q=self.q[:],dq=[0]*16,robot_timestamp=self.count)
        self.count+=1
        return dict(available=self.connected,reception_age_seconds=0,data=data)

    async def read_gripper(self):
        self.count+=1
        return dict(result='success',timestamp=self.count,left_opening=self.grip[0],right_opening=self.grip[1],
                    left_speed=0,right_speed=0,left_force=20,right_force=20)

    @contextmanager
    def teaching_feedback(self):
        yield

    @contextmanager
    def pose_servo(self,job):
        yield {'job':job}

    async def send_pose_servoj(self,q,token):
        self.q=q[:];self.commands.append(('arm',q[:]))

    async def send_gripper(self,data):
        side=0 if 'left_opening' in data else 1
        self.grip[side]=data[('left' if side==0 else 'right')+'_opening']
        self.commands.append(('gripper',dict(data)))


async def allowed():return True


def recording(count=6):
    frames=[state(i) for i in range(count)]
    for i,s in enumerate(frames):s['gripper']=[i,20-i]
    return pack_trajectory(BINDING,frames)


def test_replay_binding_speed_corruption_and_start_rejected():
    async def run():
        reader=Reader();replay=Replay(reader,allowed,lambda:True)
        reader.q[0]=.5
        with pytest.raises(ValueError,match='start_mismatch'):await replay.play(recording(),BINDING,{})
        assert reader.commands==[]
        with pytest.raises(ValueError,match='binding'):read_trajectory(recording(),dict(BINDING,serial='other'))
        with pytest.raises(ValueError):read_trajectory(recording().replace(b'0.0001',b'0.0002'))
        bad=[state(0),state(1)];bad[1]['q'][0]=1
        with pytest.raises(ValueError,match='speed'):read_trajectory(pack_trajectory(BINDING,bad))
    asyncio.run(run())


def test_replay_hold_head_grippers_and_terminal_feedback():
    async def run():
        reader=Reader();replay=Replay(reader,allowed,lambda:True)
        raw=recording();p=await replay.play(raw,BINDING,{})
        result=await replay.play(raw,BINDING,dict(execute=True,preview_id=p['preview_id'],workspace_clear=True,
            take_control=True,gripper_speed=30,gripper_force=20))
        outcome=await replay.wait(result['action_id'])
        assert outcome['status']=='completed',outcome
        assert outcome['result']['physical_stop_verified']
        assert all(q[14:]==[.2,.3] for kind,q in reader.commands if kind=='arm')
        assert reader.grip==[5,15]
        assert sum(k=='arm' for k,v in reader.commands)==6
    asyncio.run(run())


@pytest.mark.parametrize('failure',['conflict','disconnect','stop'])
def test_control_loss_and_stop_never_replay_pending_frames(failure):
    async def run():
        reader=Reader();exclusive=True
        async def guard():return exclusive
        replay=Replay(reader,guard,lambda:True);raw=recording(20)
        p=await replay.play(raw,BINDING,{})
        await replay.play(raw,BINDING,dict(execute=True,preview_id=p['preview_id'],workspace_clear=True,
            take_control=True,gripper_speed=30,gripper_force=20))
        await asyncio.sleep(.045)
        if failure=='conflict':exclusive=False
        elif failure=='disconnect':reader.connected=False
        else:await replay.stop()
        await replay.task
        count=len(reader.commands);await asyncio.sleep(.05)
        assert count==len(reader.commands)
        assert replay.outcome['status'] in ('cancelled','error')
        assert not replay.outcome['result']['physical_stop_verified']
        assert replay.preview is None
    asyncio.run(run())


def test_unverified_control_never_sends():
    async def run():
        reader=Reader()
        async def unknown():return False
        replay=Replay(reader,unknown,lambda:True)
        with pytest.raises(ValueError,match='exclusive_control'):await replay.play(recording(),BINDING,{})
        assert not reader.commands
    asyncio.run(run())


def test_capture_original_mjpeg_and_no_motion(tmp_path):
    async def run():
        reader=Reader();raw=jpeg()
        async def camera(request):
            response=web.StreamResponse(headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame'})
            await response.prepare(request)
            try:
                for _ in range(100):
                    await response.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(raw)).encode()+b'\r\n\r\n'+raw+b'\r\n')
                    await asyncio.sleep(.03)
            except (ConnectionResetError,asyncio.CancelledError):pass
            return response
        app=web.Application();app.router.add_get('/camera',camera)
        runner=web.AppRunner(app);await runner.setup();site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        port=site._server.sockets[0].getsockname()[1]
        try:
            async with aiohttp.ClientSession() as session:
                urls={c:f'http://127.0.0.1:{port}/camera' for c in ('camera','camera_left','camera_right')}
                capture=Capture(reader,session,urls,tmp_path)
                await capture.start('Move a block','unlabelled','fake vendor')
                for i in range(20):
                    reader.teaching_observer(dict(q=reader.q,names=JOINT_NAMES,dq=[0]*16,tau=[0]*16,robot_timestamp=i),time.monotonic_ns())
                    await asyncio.sleep(.02)
                result=await capture.finish()
                assert result['complete'] and result['training_complete'],result
                path,m,states=load_episode(tmp_path,result['filename'])
                assert m['cameras']['head']['size']==[64,48]
                assert (path/'head.mjpeg').read_bytes().startswith(raw)
                assert all(s['vendor_target'] is None for s in states)
                assert reader.commands==[] and reader.teaching_observer is None
        finally:await runner.cleanup()
    asyncio.run(run())


def test_shared_service_packages_and_skill_gate(tmp_path):
    async def run():
        from device import Adapter
        from test_sensors import settings
        from test_teaching_episode import episode
        from teaching import TronTeachingPlugin
        from common.teaching.package import pack,unpack
        reader=Reader()
        cfg=settings(enabled_cards=['tron2_teaching','joint_state'])
        cfg.update(teaching_directory=str(tmp_path),camera_urls={})
        owner=Adapter(lambda *a:None,cfg,reader=reader)
        adapter=owner.teaching.adapter
        assert adapter.resources==['arm_l','arm_r','head']
        adapter.capture.binding=lambda:BINDING
        writer=episode(tmp_path/'episodes')
        plugin=owner.teaching
        draft=await asyncio.to_thread(plugin.dispatch,'new',{'filename':writer.ident})
        assert draft['recording']['training_complete']
        with pytest.raises(ValueError,match='successful_trial_required'):
            await asyncio.to_thread(plugin.dispatch,'save',{'draft_id':draft['id'],'effect_confirmed':True,'name':'test','conditions':'test'})
        inspected=await asyncio.to_thread(plugin.dispatch,'inspect',{'draft_id':draft['id']})
        assert not inspected['start']['can_trial']
        raw=adapter.export_recording(draft['recording'])
        manifest,archive=pack(dict(name='test',description='test',conditions='test',phrase='test'),adapter.compatibility(),raw,'motus.tron2.trajectory/1')
        parsed,raw=unpack(archive)
        imported=adapter.import_recording(parsed,raw)
        assert imported['complete'] and not imported['training_complete']
        assert imported['binding']==BINDING
        new_plugin=TronTeachingPlugin(adapter)
        package=tmp_path/'test.motus-skill';package.write_bytes(archive)
        result=await asyncio.to_thread(new_plugin.dispatch,'import',{'package':str(package)})
        assert result['requires_local_trial'] and not result['executed']
        assert new_plugin.draft['trial'] is None
        assert reader.commands==[]
    asyncio.run(run())


def test_capture_rejects_source_clock_reset_and_does_not_repeat_old_samples(tmp_path):
    reader=Reader();capture=Capture(reader,None,{},tmp_path)
    frames=[];errors=[]
    capture.writer=SimpleNamespace(accepting=True,put=lambda kind,frame:frames.append(frame) is None,
                                  fail=lambda reason:errors.append(reason))
    sample=dict(q=reader.q,names=JOINT_NAMES,dq=[],tau=[],robot_timestamp=10)
    capture.observe(sample,100);capture.observe(sample,200)
    assert len(frames)==1 and capture.duplicate_samples==1
    capture.observe(dict(sample,robot_timestamp=9),300)
    assert errors==['source_clock_reversed'] and len(frames)==1


def test_teaching_poll_lease_only_changes_read_cadence():
    from vendor import RobotReader
    from test_sensors import settings
    reader=RobotReader(settings(poll_interval_seconds=.5))
    assert reader.joint_poll_interval()==.5
    with reader.teaching_feedback():assert reader.joint_poll_interval()==.02
    assert reader.joint_poll_interval()==.5 and reader.ws is None
    reader.teaching_observer=lambda *args:None
    assert reader.joint_poll_interval()==.02
