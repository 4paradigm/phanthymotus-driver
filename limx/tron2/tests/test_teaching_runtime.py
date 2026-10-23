import asyncio


import json


import math


from contextlib import contextmanager


from collections import deque


from pathlib import Path


import sys


import time


from types import SimpleNamespace


import aiohttp


from aiohttp import web


import pytest


import teaching_replay


from teaching_capture import Capture


from teaching_replay import Replay, pack_trajectory, read_trajectory, movej_start_target


from test_teaching_episode import BINDING, jpeg, state


from skeleton import JOINT_NAMES


from episode import load_episode


from vendor import ARM_LOWER, ARM_UPPER, MoveError, validate_movej


class Reader:
    def __init__(self):
        self.serial='test-device';self.connected=True;self.teaching_observer=None;self._joint_poll_wake=None
        self.q=[0]*14+[.2,.3];self.grip=[0,20];self.count=0;self.commands=[]
        self.replaying=False;self.replay_sends=[];self.expected_sends=None;self.feedback_source=None

    def now(self):
        return teaching_replay.time.monotonic()

    def sample(self,name):
        now=self.now()
        if name=='joint_state' and self.replaying and self.feedback_source is not None:
            q,dq,stamp,started,received=self.feedback_source(now)
        else:
            q,dq,stamp,started,received=self.q[:],[0]*16,self.count,now,now
        data=dict(working_mode='developer_mode',working_mode_valid=True,collision_enable_valid=True,
            compliant_enable_valid=True,collision_enable=True,compliant_enable=False,motor='OK',sw_version='test') if name=='robot_status' else dict(q=q,dq=dq,robot_timestamp=stamp)
        self.count+=1
        return dict(available=self.connected,reception_age_seconds=0 if received is None else max(0,now-received),
            received_monotonic=received,query_started_monotonic=started,
            query_round_trip_seconds=None if started is None or received is None else max(0,received-started),
            measurement_time_verified=False,data=data)

    async def read_gripper(self):
        self.count+=1
        return dict(result='success',timestamp=self.count,left_opening=self.grip[0],right_opening=self.grip[1],
                    left_speed=0,right_speed=0,left_force=20,right_force=20)

    @contextmanager
    def teaching_feedback(self):
        yield

    @contextmanager
    def pose_feedback(self):
        yield

    @contextmanager
    def pose_servo(self,job):
        self.replaying=True
        try:
            yield {'job':job}
        finally:
            self.replaying=False

    async def send_pose_servoj(self,q,token):
        self.replay_sends.append((self.now(),q[:]))
        self.q=q[:];self.commands.append(('arm',q[:]))

    async def send_movej(self,q,duration):
        validate_movej(q,duration)
        self.q[:14]=q;self.commands.append(('movej',q[:]))
        return {'status':'accepted','motion_sent':True}

    async def send_gripper(self,data,on_sent=None,expected_connection=None):
        side=0 if 'left_opening' in data else 1
        self.grip[side]=data[('left' if side==0 else 'right')+'_opening']
        self.commands.append(('gripper',dict(data)))
        if on_sent is not None:on_sent()


async def allowed():return True


def recording(count=6):
    frames=[state(i) for i in range(count)]
    for i,s in enumerate(frames):s['gripper']=[i,20-i]
    return pack_trajectory(BINDING,frames)


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
        schema=plugin.get_tools()[0]['inputSchema']
        assert set(schema['x-action-params'])=={'start','info','preview','run','stop','import','export','reset'}
        for action in ('episode_list','episode_details','annotate','training_export','training_info','unlock'):
            assert action in schema['x-workbench']['actions']
            plugin._validate(action,{})
        assert plugin.dispatch('episode_list',{})['episodes']
        with pytest.raises(ValueError,match='unlock_operator_confirmation_required'):
            await asyncio.to_thread(plugin.dispatch,'unlock',{})
        unlocked=await asyncio.to_thread(plugin.dispatch,'unlock',dict(physical_stop_confirmed=True,exclusive_control_confirmed=True))
        assert unlocked['unlocked'] and not reader.commands
        draft=await asyncio.to_thread(plugin.dispatch,'new',{'filename':writer.ident})
        assert draft['recording']['training_complete']
        with pytest.raises(ValueError,match='successful_trial_required'):
            await asyncio.to_thread(plugin.dispatch,'save',{'draft_id':draft['id'],'effect_confirmed':True,'name':'test','conditions':'test'})
        inspected=await asyncio.to_thread(plugin.dispatch,'inspect',{'draft_id':draft['id']})
        assert inspected['start']['can_trial']
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
