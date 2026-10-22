"""SDK lifecycle and result checks, with no physical robot connection."""
import asyncio
import json
import logging
import math
import time
from types import SimpleNamespace

import pytest
from aiohttp import web

from test_teaching_runtime import Reader, recording, allowed
from test_teaching_episode import BINDING, state
from teaching_replay import Replay, read_trajectory, pack_trajectory
from teaching_sdk import StableWindow, SDKReplay
from vendor import ARM_UPPER


def test_stability_uses_window_range_not_adjacent_velocity():
    window = StableWindow()
    for i in range(61):
        # 0.03 degrees over 10 ms exceeds 1 deg/s, but the observed range is tiny.
        ok = window.add(i*.01, i, [math.radians(.03*(i%2))]*16, [0,0])
    assert ok


@pytest.mark.parametrize('failure', ['drift','frozen','gap'])
def test_stability_requires_continuous_advancing_stationary_feedback(failure):
    w = StableWindow()
    for i in range(100):
        now = i*(.2 if failure=='gap' else .01)
        assert not w.add(now, 0 if failure=='frozen' else i,
                         [i*.002 if failure=='drift' else 0]*16, [0,0])


def test_stability_rejects_clock_regression():
    w = StableWindow();w.add(0, 2, [0]*16, [0,0])
    with pytest.raises(ValueError, match='regressed'):w.add(.01, 1, [0]*16, [0,0])


def test_preflight_validation_and_preview_do_not_construct_sdk(monkeypatch):
    def forbidden(*args):raise AssertionError('SDK construction during preview')
    monkeypatch.setattr('teaching_replay.SDKReplay', forbidden)
    async def run():
        r = Reader();replay = Replay(r,allowed,lambda:True)
        raw = recording()
        p = await replay.play(raw,BINDING,{})
        assert not p['executed'] and not r.commands
        with pytest.raises(ValueError,match='integrity'):
            await replay.preflight(raw.replace(b'"frames":6',b'"frames":7'),BINDING)
        with pytest.raises(ValueError,match='paired_preview'):
            await replay.play(raw,BINDING,dict(execute=True,preview_id='wrong'))
    asyncio.run(run())


def test_execution_clamps_copy_and_preserves_source():
    async def run():
        states = [state(i) for i in range(4)]
        for row in states:row['q'][4] = ARM_UPPER[4]+math.radians(1)
        raw = pack_trajectory(BINDING,states)
        frames,_ = await Replay(Reader(),allowed,lambda:True).preflight(raw,BINDING)
        assert frames[0]['q'][4] == ARM_UPPER[4]
        assert read_trajectory(raw,BINDING)[1][0]['q'][4] > ARM_UPPER[4]
        for row in states:row['q'][4] = ARM_UPPER[4]+math.radians(6)
        with pytest.raises(ValueError,match='tolerance'):read_trajectory(pack_trajectory(BINDING,states))
    asyncio.run(run())


@pytest.mark.parametrize('failure', [None,'sdk_error','stop','cleanup'])
def test_worker_submits_all_frames_and_never_completes_on_failure(failure):
    _,frames = read_trajectory(recording(6),BINDING)
    w = SDKReplay('ws://127.0.0.1:5000',frames,[0,0],frames[0]['q'],30,20)
    sent = [];closed = []
    class Controller:
        def set_gripper(self,*g):pass
        def raise_if_faulted(self):pass
        def command_joints(self,q,eta):
            sent.append(q)
            if failure=='sdk_error':logging.getLogger('tron2_env.motion').warning('simulated send failure')
            if failure=='stop':w.stop.set()
        def pause(self,timeout):closed.append('pause')
        def disconnect(self):
            closed.append('disconnect')
            w.fail('in-flight query interrupted by expected close')
            if failure=='cleanup':raise RuntimeError('cleanup failed')
    def create():w.controller=Controller();w.motion_sent=True
    w.create_controller=create
    w.wait_stable=lambda *args:dict(stable_seconds=.5)
    result=w.run()
    assert closed==['pause','disconnect']
    if failure is None:
        assert result['status']=='completed' and len(sent)==6 and result['target_submission_complete']
    else:
        assert result['status']!='completed'
        if failure in ('sdk_error','stop'):assert len(sent)==1
        if failure=='cleanup':assert not result['output_stopped'] and result['motion_may_continue']


def test_stop_waits_for_worker_cleanup_and_keeps_task_mutex(monkeypatch):
    import threading
    started=threading.Event();cleanup=threading.Event()
    class Worker:
        phase='replaying'
        def __init__(self,*args):self.stop=threading.Event()
        def run(self):
            started.set();self.stop.wait(2);cleanup.wait(2)
            return dict(status='cancelled',output_stopped=True,physical_stop_verified=False,motion_may_continue=True)
    monkeypatch.setattr('teaching_replay.SDKReplay',Worker)
    async def run():
        r=Reader();r.settings={'url':'ws://127.0.0.1:5000'}
        replay=Replay(r,None,lambda:True);raw=recording();p=await replay.play(raw,BINDING,{})
        args=dict(execute=True,preview_id=p['preview_id'],workspace_clear=True,take_control=True,
                  gripper_speed=30,gripper_force=20)
        with pytest.raises(ValueError,match='exclusive'):await replay.play(raw,BINDING,args)
        p=await replay.play(raw,BINDING,{});args.update(preview_id=p['preview_id'],exclusive_control_confirmed=True)
        await replay.play(raw,BINDING,args)
        await asyncio.to_thread(started.wait,1)
        stop=asyncio.create_task(replay.stop());await asyncio.sleep(.02)
        assert replay.busy and not stop.done()
        with pytest.raises(ValueError,match='other_motion'):await replay.play(raw,BINDING,{})
        cleanup.set();await stop
        assert not replay.busy and replay.active
        assert replay.outcome['status']=='cancelled'
    asyncio.run(run())


@pytest.mark.parametrize('mode', ['success','rejected','stop_init','disconnect'])
def test_pinned_sdk_with_fake_vendor_full_lifecycle(mode):
    pytest.importorskip('tron2_env')
    async def run():
        q=[.1]*14+[0,0];g=[0,0];commands=[];connections=[]
        async def robot(request):
            connections.append(1)
            ws=web.WebSocketResponse();await ws.prepare(request)
            await ws.send_json(dict(title='notify_robot_info',accid='test',data={}))
            async for message in ws:
                packet=json.loads(message.data);title=packet['title'];data=packet.get('data',{})
                reply={'result':'success'}
                if title=='request_get_joint_state':reply.update(q=q[:])
                elif title=='request_get_limx_2fclaw_state':reply.update(left_opening=g[0],right_opening=g[1])
                elif title=='request_movej':
                    q[:14]=data['joint'];commands.append(title)
                    if mode=='rejected':reply['result']='failed'
                    if mode=='stop_init':worker.stop.set()
                    if mode=='disconnect':await ws.close();return ws
                elif title=='request_moveh':q[14:]=data['joint'];commands.append(title)
                elif title=='request_set_limx_2fclaw_cmd':
                    g[:]=[data['left_opening'],data['right_opening']];commands.append((title,g[:],data['left_speed'],data['left_force']))
                elif title=='request_servoj':q[:]=data['q'];commands.append(title)
                await ws.send_json(dict(title=title.replace('request_','response_'),accid='test',
                    guid=packet.get('guid'),timestamp=time.monotonic(),data=reply))
            return ws
        app=web.Application();app.router.add_get('/',robot)
        runner=web.AppRunner(app);await runner.setup();site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        try:
            frames=[dict(t_ns=i*20_000_000,q=[.1+i*.001]*14,gripper=[i,20-i]) for i in range(6)]
            worker=SDKReplay('ws://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1]),frames,[0,0],frames[0]['q'],30,20)
            result=await asyncio.to_thread(worker.run)
            if mode!='success':
                assert result['status']!='completed',result
                assert result['samples_sent']==0 and result['output_stopped']
                n=len(commands);await asyncio.sleep(.1)
                assert len(commands)==n and len(connections)==1
                return
            assert result['status']=='completed',result
            assert result['samples_sent']==6 and result['end_check']['stable_seconds']>=.5
            assert commands[0]=='request_movej' and 'request_servoj' in commands
            grips=[v for v in commands if isinstance(v,tuple)]
            assert all(v[2:]==(30,20) for v in grips)
            assert [v[1] for v in grips]==[[100,100]]+[f['gripper'] for f in frames]
            n=len(commands);await asyncio.sleep(.05);assert len(commands)==n
        finally:await runner.cleanup()
    asyncio.run(run())


@pytest.mark.parametrize('refresh', [True, False])
def test_preflight_waits_for_fresh_idle_feedback(refresh):
    async def run():
        reader=Reader();sample=reader.sample;fresh=False
        def current(name):
            out=sample(name)
            if name=='joint_state' and not fresh:out['reception_age_seconds']=.4
            return out
        reader.sample=current
        def update():
            nonlocal fresh
            fresh=refresh
        timer=asyncio.get_running_loop().call_later(.02,update)
        try:
            replay=Replay(reader,allowed,lambda:True)
            if refresh:await replay.preflight(recording(),BINDING)
            else:
                with pytest.raises(ValueError,match='stale'):await replay.preflight(recording(),BINDING)
            assert not reader.commands
        finally:timer.cancel()
    asyncio.run(run())
