"""Fake vendor capture and replay boundaries; never connect to hardware."""
import asyncio
import json
import math
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
from teaching_replay import Replay, pack_trajectory, read_trajectory, movej_start_target
from test_teaching_episode import BINDING, jpeg, state
from skeleton import JOINT_NAMES
from episode import load_episode
from vendor import ARM_LOWER, ARM_UPPER, validate_movej


@pytest.fixture(autouse=True)
def short_preparation(monkeypatch):
    monkeypatch.setattr('teaching_replay.duration_for_change',lambda delta:.1)
    monkeypatch.setattr('teaching_replay.START_STABILITY_SECONDS',.03)
    monkeypatch.setattr('teaching_replay.START_ARRIVAL_TIMEOUT',.25)
    monkeypatch.setattr('teaching_replay.START_GRIPPER_TIMEOUT',.25)


@pytest.fixture
def replay_clock(monkeypatch):
    clock=[0.0];sleep=asyncio.sleep
    async def advance(delay):
        clock[0]+=max(0,delay)
        await sleep(0)
    monkeypatch.setattr('teaching_replay.time',SimpleNamespace(monotonic=lambda:clock[0]))
    monkeypatch.setattr(asyncio,'sleep',advance)
    return clock


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
    def pose_feedback(self):
        yield

    @contextmanager
    def pose_servo(self,job):
        yield {'job':job}

    async def send_pose_servoj(self,q,token):
        self.q=q[:];self.commands.append(('arm',q[:]))

    async def send_movej(self,q,duration):
        validate_movej(q,duration)
        self.q[:14]=q;self.commands.append(('movej',q[:]))
        return {'status':'accepted','motion_sent':True}

    async def send_gripper(self,data):
        side=0 if 'left_opening' in data else 1
        self.grip[side]=data[('left' if side==0 else 'right')+'_opening']
        self.commands.append(('gripper',dict(data)))


async def allowed():return True


@pytest.mark.parametrize('refresh',[True,False])
@pytest.mark.parametrize('stage',['preflight','execution'])
def test_replay_refreshes_idle_feedback_before_sending(refresh,stage):
    async def run():
        reader=Reader();reader.fresh=False;reader.leased=False
        original=reader.sample
        def sample(name):
            result=original(name)
            if name=='joint_state' and not reader.fresh:
                result['reception_age_seconds']=.4
            return result
        reader.sample=sample
        @contextmanager
        def lease():
            reader.leased=True
            timer=asyncio.get_running_loop().call_later(.03,setattr,reader,'fresh',refresh)
            try:yield
            finally:
                timer.cancel();reader.leased=False
        reader.pose_feedback=lease
        reader.teaching_feedback=lease
        replay=Replay(reader,allowed,lambda:True)
        if stage=='execution':
            _,frames=read_trajectory(recording(),BINDING)
            await replay._run('test',frames,reader.q[14:],10,10)
            assert replay.outcome['status']==('completed' if refresh else 'error')
            assert not replay.outcome['result']['motion_may_continue']
            assert bool(reader.commands)==refresh
        elif refresh:
            await replay.preflight(recording(),BINDING)
        else:
            with pytest.raises(ValueError,match='replay_feedback_stale'):
                await replay.preflight(recording(),BINDING)
        assert not reader.leased
        if stage=='preflight' or not refresh:assert not reader.commands
    asyncio.run(run())


def recording(count=6):
    frames=[state(i) for i in range(count)]
    for i,s in enumerate(frames):s['gripper']=[i,20-i]
    return pack_trajectory(BINDING,frames)


def test_replay_binding_corruption_and_offset_preview():
    async def run():
        reader=Reader();replay=Replay(reader,allowed,lambda:True)
        reader.q[0]=.5
        preview=await replay.play(recording(),BINDING,{})
        assert preview['executed'] is False and '起点' in preview['description']
        assert reader.commands==[]
        with pytest.raises(ValueError,match='binding'):read_trajectory(recording(),dict(BINDING,serial='other'))
        with pytest.raises(ValueError):read_trajectory(recording().replace(b'0.0001',b'0.0002'))
    asyncio.run(run())


def test_replay_moves_arms_then_grippers_before_recorded_frames(replay_clock):
    async def run():
        reader=Reader();reader.q[0]=.5;reader.grip=[94,0]
        move=reader.send_movej;servo=reader.send_pose_servoj;not_before=[0]
        async def timed_move(q,duration):
            not_before[0]=replay_clock[0]+duration
            return await move(q,duration)
        async def timed_servo(q,token):
            assert replay_clock[0]>=not_before[0]
            return await servo(q,token)
        reader.send_movej=timed_move;reader.send_pose_servoj=timed_servo
        replay=Replay(reader,allowed,lambda:True)
        raw=recording();preview=await replay.play(raw,BINDING,{})
        assert not reader.commands
        action=await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        outcome=await replay.wait(action['action_id'])
        assert outcome['status']=='completed',outcome
        assert outcome['result']['start_verified']
        assert [kind for kind,_ in reader.commands[:4]]==['movej','gripper','gripper','arm']
        assert reader.commands[0][1]==[0]*14
        assert reader.commands[1][1]=={'left_opening':0,'left_speed':30,'left_force':20}
        assert reader.commands[2][1]=={'right_opening':20,'right_speed':30,'right_force':20}
        assert sum(kind=='movej' for kind,_ in reader.commands)==1
    asyncio.run(run())


def test_late_frame_after_preparation_is_not_sent(replay_clock):
    async def run():
        reader=Reader();original=reader.read_gripper
        async def slow_gripper():
            if replay.phase=='replaying':replay_clock[0]+=.051
            return await original()
        reader.read_gripper=slow_gripper
        replay=Replay(reader,allowed,lambda:True)
        frames,head=await replay.preflight(recording(),BINDING)
        await replay._run('late-frame',frames,head,30,20)
        assert replay.outcome['result']['reason']=='replay_deadline_missed_no_catchup'
        assert [kind for kind,_ in reader.commands]==['movej']
    asyncio.run(run())


@pytest.mark.parametrize('failure',['not_sent','unknown_ack','arm_stuck','gripper_stuck','connection_changed','frozen_timestamp'])
def test_preparation_failure_never_enters_servoj(failure):
    from vendor import MoveError
    async def run():
        reader=Reader();reader.q[0]=.5;reader.grip=[94,0]
        original_move=reader.send_movej;original_sample=reader.sample
        async def move(q,duration):
            if failure=='not_sent':raise MoveError('not sent',False)
            if failure=='unknown_ack':
                reader.commands.append(('movej',q[:]))
                raise MoveError('ack unknown',None)
            await original_move(q,duration)
            if failure=='arm_stuck':reader.q[0]=.5
            if failure=='connection_changed':reader.ws=object()
        def sample(name):
            result=original_sample(name)
            if failure=='frozen_timestamp' and reader.commands and name=='joint_state':
                result['data']['robot_timestamp']=10000
            return result
        async def stuck_gripper(data):reader.commands.append(('gripper',dict(data)))
        reader.send_movej=move;reader.sample=sample
        if failure=='gripper_stuck':reader.send_gripper=stuck_gripper
        replay=Replay(reader,allowed,lambda:True)
        frames,head=await replay.preflight(recording(),BINDING)
        await replay._run('preparation-failure',frames,head,30,20)
        assert replay.outcome['status']=='error',replay.outcome
        assert not any(kind=='arm' for kind,_ in reader.commands)
        assert sum(kind=='movej' for kind,_ in reader.commands)<=1
        if replay.release_task:await replay.release_task
        assert bool(replay.active)==(failure in ('unknown_ack','connection_changed','frozen_timestamp'))
        if failure in ('arm_stuck','gripper_stuck'):
            assert replay.outcome['result']['release_method']=='automatic_stable_feedback'
            assert replay.outcome['status']=='error'
    asyncio.run(run())


@pytest.mark.parametrize('stage',['movej','gripper'])
def test_stop_during_preparation_keeps_uncertain_motion_blocked(stage):
    async def run():
        reader=Reader();reader.grip=[94,0];entered=asyncio.Event()
        async def blocked(*args):
            reader.commands.append((stage,args[0]));entered.set()
            await asyncio.Event().wait()
        if stage=='movej':reader.send_movej=blocked
        else:reader.send_gripper=blocked
        replay=Replay(reader,allowed,lambda:True)
        preview=await replay.play(recording(),BINDING,{})
        await replay.play(recording(),BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        await asyncio.wait_for(entered.wait(),2)
        stopped=await replay.stop()
        assert replay.outcome['status']=='cancelled'
        assert stopped['motion_may_continue'] and not stopped['authority_released']
        assert replay.active['state']=='blocked_requires_operator_check'
        assert not any(kind=='arm' for kind,_ in reader.commands)
    asyncio.run(run())



def test_recorded_wrist_start_uses_controller_movej_range(replay_clock):
    async def run():
        reader=Reader();states=[state(i) for i in range(6)]
        for row in states:
            row['q'][13]=-.7837002873420715
            row['gripper']=[0,20]
        raw=pack_trajectory(BINDING,states)
        original=reader.send_movej
        async def controller_move(q,duration):
            assert q[13]==-.7505
            return await original(q,duration)
        reader.send_movej=controller_move
        replay=Replay(reader,allowed,lambda:True)
        frames,head=await replay.preflight(raw,BINDING)
        await replay._run('recorded-start',frames,head,30,20)
        assert replay.outcome['status']=='completed',replay.outcome
        assert abs(reader.commands[0][1][13]-states[0]['q'][13])<math.radians(2)
        assert all(q[13]==states[0]['q'][13] for kind,q in reader.commands if kind=='arm')
        assert pack_trajectory(BINDING,states)==raw
        states[0]['q'][13]=-.80
        with pytest.raises(ValueError,match='MoveJ.*2°'):
            await replay.preflight(pack_trajectory(BINDING,states),BINDING)
    asyncio.run(run())


def test_cancelled_movej_unlocks_only_after_duration_and_stationary_feedback():
    async def run():
        reader=Reader();entered=asyncio.Event();original=reader.send_movej
        async def move(q,duration):
            result=await original(q,duration);entered.set();return result
        reader.send_movej=move
        replay=Replay(reader,allowed,lambda:True)
        preview=await replay.play(recording(),BINDING,{})
        await replay.play(recording(),BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        await entered.wait()
        stopped=await replay.stop()
        assert not stopped['authority_released']
        assert replay.recovering
        assert time.monotonic()<replay.movej_not_before
        await replay.release_task
        assert not replay.active
        assert replay.outcome['status']=='cancelled'
        assert replay.outcome['result']['release_method']=='automatic_stable_feedback'
        assert [kind for kind,_ in reader.commands]==['movej']
    asyncio.run(run())


@pytest.mark.parametrize('failure',[None,'confirmation','moving','frozen','disconnect','busy','duration','peer'])
def test_manual_unlock_checks_feedback_without_sending(failure):
    async def run():
        reader=Reader();replay=Replay(reader,allowed,lambda:failure!='peer')
        replay.motion_sent=True
        replay.outcome={'action_id':'failed','status':'error','result':{'reason':'arrival_failed','motion_may_continue':True}}
        replay.preview={'id':'old'}
        args=dict(physical_stop_confirmed=True,exclusive_control_confirmed=True)
        if failure=='confirmation':args['physical_stop_confirmed']=False
        if failure=='disconnect':reader.connected=False
        if failure=='duration':replay.movej_not_before=time.monotonic()+10
        if failure=='busy':replay.task=asyncio.create_task(asyncio.sleep(10))
        original=reader.sample
        def sample(name):
            r=original(name)
            if name=='joint_state':
                if failure=='moving':r['data']['dq'][0]=.1
                if failure=='frozen':r['data']['robot_timestamp']=1
            return r
        reader.sample=sample
        try:
            if failure:
                with pytest.raises(ValueError):await replay.unlock(args)
                assert replay.active
            else:
                result=await replay.unlock(args)
                assert result['unlocked'] and not result['motion_sent']
                assert not replay.active and replay.preview is None
                assert replay.outcome['status']=='error'
                assert replay.outcome['result']['reason']=='arrival_failed'
                assert replay.outcome['result']['release_method']=='operator_confirmed_stable_feedback'
            assert not reader.commands
        finally:
            if replay.task:
                replay.task.cancel()
                await asyncio.gather(replay.task,return_exceptions=True)
    asyncio.run(run())


def test_replay_above_old_speed_limit_preserves_frames_and_timing(replay_clock):
    async def run():
        states=[state(i) for i in range(6)]
        for i,s in enumerate(states):
            s['q'][0]=i*.02
            s['gripper']=[0,20]
        raw=pack_trajectory(BINDING,states)
        _,frames=read_trajectory(raw,BINDING)
        assert (frames[1]['q'][0]-frames[0]['q'][0])*1e9/(frames[1]['t_ns']-frames[0]['t_ns'])>.8
        assert [f['t_ns'] for f in frames]==[s['received_ns']-states[0]['received_ns'] for s in states]
        reader=Reader();replay=Replay(reader,allowed,lambda:True)
        preview=await replay.play(raw,BINDING,{})
        assert preview['duration_seconds']==frames[-1]['t_ns']/1e9
        result=await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        outcome=await replay.wait(result['action_id'])
        assert outcome['status']=='completed',outcome
        assert [q[:14] for kind,q in reader.commands if kind=='arm']==[s['q'][:14] for s in states]
    asyncio.run(run())


@pytest.mark.parametrize('direction',[-1,1])
def test_recording_tolerance_clamps_commands_only(direction,replay_clock):
    async def run():
        limits=ARM_UPPER if direction==1 else ARM_LOWER
        states=[state(i) for i in range(6)]
        for i,s in enumerate(states):
            s['q'][:14]=[v+direction*math.radians(5)*i/5 for v in limits]
            s['gripper']=[0,20]
        states[0]['q'][:14]=[0]*14
        raw=pack_trajectory(BINDING,states)
        _,frames=read_trajectory(raw,BINDING)
        assert [f['q'] for f in frames]==[s['q'][:14] for s in states]
        reader=Reader()
        replay=Replay(reader,allowed,lambda:True)
        preview=await replay.play(raw,BINDING,{})
        result=await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        outcome=await replay.wait(result['action_id'])
        assert outcome['status']=='completed',outcome
        commands=[q[:14] for kind,q in reader.commands if kind=='arm']
        assert commands==[[0]*14]+[list(limits)]*(len(states)-1)
        for q in commands:validate_movej(q,.1)
        assert pack_trajectory(BINDING,states)==raw
        assert read_trajectory(raw,BINDING)[1]==frames
        states[-1]['q'][11]+=direction*math.radians(.001)
        with pytest.raises(ValueError,match='5 degree tolerance at index 11'):
            read_trajectory(pack_trajectory(BINDING,states),BINDING)
    asyncio.run(run())


@pytest.mark.parametrize('value',[True,'1',None,float('nan'),float('inf')])
def test_recording_tolerance_rejects_invalid_numbers(value):
    states=[state(i) for i in range(2)]
    states[0]['q'][11]=value
    with pytest.raises(ValueError):
        read_trajectory(pack_trajectory(BINDING,states),BINDING)


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


def test_manual_exclusive_confirmation_is_required_for_each_execution():
    async def run():
        reader=Reader();replay=Replay(reader,None,lambda:True);raw=recording()
        for confirmation in (None,False,'true'):
            preview=await replay.play(raw,BINDING,{})
            with pytest.raises(ValueError,match='operator_exclusive_confirmation_required'):
                await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
                    workspace_clear=True,take_control=True,exclusive_control_confirmed=confirmation,
                    gripper_speed=30,gripper_force=20))
            assert reader.commands==[]
        preview=await replay.play(raw,BINDING,{})
        result=await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
            workspace_clear=True,take_control=True,exclusive_control_confirmed=True,
            gripper_speed=30,gripper_force=20))
        assert (await replay.wait(result['action_id']))['status']=='completed'
        reader.q=[0]*14+[.2,.3];reader.grip=[0,20]
        count=len(reader.commands);preview=await replay.play(raw,BINDING,{})
        with pytest.raises(ValueError,match='operator_exclusive_confirmation_required'):
            await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
                workspace_clear=True,take_control=True,gripper_speed=30,gripper_force=20))
        assert len(reader.commands)==count
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
        schema=plugin.get_tools()[0]['inputSchema']
        assert set(schema['x-action-params'])=={'start','info','preview','run','stop','import','export'}
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


def test_static_replay_from_idle_uses_real_reader_and_websocket():
    from vendor import RobotReader
    from test_sensors import settings,info,joints
    async def run():
        commands=[]
        async def vendor(request):
            ws=web.WebSocketResponse();await ws.prepare(request)
            await ws.send_json(info())
            async for msg in ws:
                packet=json.loads(msg.data);title=packet['title'];payload=None
                if title=='request_get_joint_state':
                    payload=joints(q=[0]*16,timestamp=time.monotonic())
                elif title=='request_get_limx_2fclaw_state':
                    payload=dict(result='success',timestamp=time.monotonic(),left_opening=0,right_opening=0,
                        left_speed=0,right_speed=0,left_force=0,right_force=0)
                elif title=='request_movej':
                    commands.append(packet);payload={'result':'success'}
                else:commands.append(packet)
                if payload is not None:
                    await ws.send_json(dict(title=title.replace('request_','response_'),
                        accid=packet['accid'],guid=packet['guid'],data=payload))
            return ws
        app=web.Application();app.router.add_get('/',vendor)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        try:
            async with aiohttp.ClientSession() as session:
                reader=RobotReader(settings(url='ws://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])))
                poll=asyncio.create_task(reader.run(session))
                try:
                    for _ in range(100):
                        if reader.sample('joint_state')['available']:break
                        await asyncio.sleep(.01)
                    await asyncio.sleep(.2)
                    replay=Replay(reader,None,lambda:True)
                    frames=[state(i) for i in range(10)]
                    for frame in frames:frame.update(q=[0]*16,gripper=[0,0])
                    raw=pack_trajectory(BINDING,frames)
                    preview=await replay.play(raw,BINDING,{})
                    await asyncio.sleep(.2)
                    result=await replay.play(raw,BINDING,dict(execute=True,preview_id=preview['preview_id'],
                        workspace_clear=True,take_control=True,exclusive_control_confirmed=True,
                        gripper_speed=10,gripper_force=10))
                    outcome=await replay.wait(result['action_id'])
                    assert outcome['status']=='completed',outcome
                    assert commands[0]['title']=='request_movej'
                    assert len(commands)==11 and all(p['title']=='request_servoj' for p in commands[1:])
                    assert reader.joint_poll_interval()==.5
                finally:
                    poll.cancel();await asyncio.gather(poll,return_exceptions=True)
        finally:await runner.cleanup()
    asyncio.run(run())
