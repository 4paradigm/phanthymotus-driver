"""No hardware: authenticated deadline, ownership and physical-stop receipt tests."""
from pathlib import Path
import pytest
from motion_stream import MotionGate, PROTOCOL, sign


def rig():
    now = [1_000_000_000]
    status = {k: now[0] for k in ('arm_ns', 'power_ns', 'hand_ns', 'fixed_ns')}
    status.update(q=[0.0]*14, dq=[0.0]*14, power_on=True, estop=False, fault=False, fixed_body=True)
    writes = []
    gate = MotionGate(lambda: dict(status), lambda q,h: writes.append((q,h)),
                      [(-1,1)]*14, live_enabled=True, clock=lambda: now[0], acceptance_check=lambda: True)
    lease = gate.claim()
    def advance(ms, fresh=True):
        now[0] += int(ms*1e6)
        if fresh:
            for k in ('arm_ns','power_ns','hand_ns','fixed_ns'):status[k]=now[0]
    def packet(**kw):
        p = dict(protocol=PROTOCOL, boot_id=lease['boot_id'], session_id=lease['session_id'],
                 seq=0, generated_ns=now[0], valid_for_ms=100, q=[0.1]*14, hands=[0.2,0.4])
        p.update(kw)
        return {**p, 'mac': sign(p, lease['secret'])}
    return gate,status,writes,advance,packet


def test_latest_target_and_actual_stop_receipt():
    gate,status,writes,advance,packet=rig()
    gate.accept(packet())
    advance(20);gate.tick()
    assert writes[-1][0]==[0.004]*14
    assert gate.output_active
    gate.hold(release=True)
    advance(20);gate.tick()
    assert writes[-1][1] is None
    assert gate.session_id and not gate.status()['stop_confirmed']
    gate.tick()  # Same feedback cannot confirm the stop.
    assert gate.session_id
    advance(20);gate.tick()
    assert gate.session_id is None and gate.status()['stop_confirmed']
    assert not gate.output_active


@pytest.mark.parametrize('change', [dict(seq=-1),dict(seq=True),dict(valid_for_ms=101),
    dict(generated_ns=2_000_000_000), dict(generated_ns=1), dict(q=[2]*14),
    dict(q=[True]*14), dict(hands=[1.1,0]), dict(session_id='old'),dict(extra='field')])
def test_bad_command_cannot_move(change):
    gate,_,writes,_,packet=rig()
    with pytest.raises(ValueError):gate.accept(packet(**change))
    assert not writes and gate.state==('ready' if 'session_id' in change else 'hold') and gate.session_id


def test_invalid_signature_and_replay():
    gate,_,writes,_,packet=rig()
    p=packet();p['q'][0]=0.3
    with pytest.raises(ValueError,match='mac'):gate.accept(p)
    assert not writes
    gate,_,writes,_,packet=rig();p=packet();gate.accept(p)
    with pytest.raises(ValueError,match='sequence'):gate.accept(p)
    assert gate.state=='hold'


def test_stale_feedback_retains_owner_and_does_not_write_hold_from_old_data():
    gate,_,writes,advance,packet=rig()
    gate.accept(packet());advance(20);gate.tick();count=len(writes)
    advance(110,False);gate.hold(release=True);gate.tick()
    assert gate.state=='hold' and gate.session_id and len(writes)==count
    assert not gate.status()['stop_confirmed']
    advance(191,False);gate.tick()
    assert gate.state=='fault' and gate.session_id and len(writes)==count


def test_command_expiry_stops_even_with_fresh_robot_feedback():
    gate,_,writes,advance,packet=rig()
    gate.accept(packet());advance(101);gate.tick()
    assert gate.state=='hold' and gate.reason=='command_timeout' and writes[-1][1] is None


def test_no_legacy_claim_race_or_takeover():
    gate,_,_,_,_=rig()
    with pytest.raises(ValueError,match='owned'):
        with gate.legacy():pass
    gate.session_id=None
    with gate.legacy():
        with pytest.raises(ValueError,match='pending'):gate.claim()
    with pytest.raises(ValueError,match='pending'):gate.claim(legacy_busy=True)


def test_unaccepted_live_never_claims():
    gate,_,_,_,_=rig();gate.session_id=None;gate.acceptance_check=lambda:False
    with pytest.raises(ValueError,match='acceptance'):gate.claim()
    assert gate.session_id is None


def test_cancelled_sequence_remains_busy_until_worker_exits():
    import ast, threading, time
    source=(Path(__file__).parents[1]/'device.py').read_text()
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='_ActionSequence')
    namespace={'threading':threading}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'device.py','exec'),namespace)
    sequence=namespace['_ActionSequence']('test')
    release=threading.Event();entered=threading.Event()
    def worker(cancel):entered.set();release.wait(5)
    sequence.start(worker);assert entered.wait(.2)
    try:
        sequence.cancel()
        assert sequence._thread is not None and sequence._thread.is_alive()
        with pytest.raises(RuntimeError, match='previous_action_still_running'):
            sequence.start(lambda cancel: None)
        gate,_,_,_,_=rig();gate.session_id=None
        with pytest.raises(ValueError,match='pending'):gate.claim(lambda:sequence._thread is not None)
    finally:release.set();sequence._thread.join(.5)


def test_shared_bundle_guard_covers_every_legacy_motion_entry():
    import ast
    from teleop_executor import MOTION_TOOLS
    source=(Path(__file__).parents[1]/'main.py').read_text()
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='TianyiDeviceBundle')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='dispatch')
    namespace={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'main.py','exec'),namespace)
    gate,_,_,_,_=rig()
    class Bundle:
        _teleop=type('Teleop',(),{'gate':gate,'legacy_busy':staticmethod(lambda:False)})()
        calls=[]
        def _dispatch(self,name,args):self.calls.append(name);return {'state':'ok'}
    bundle=Bundle()
    for name in MOTION_TOOLS:
        assert namespace['dispatch'](bundle,name,{'action':'reset'})['code']=='motion_owned_by_teleop'
    assert not bundle.calls
    assert namespace['dispatch'](bundle,'camera_head',{'action':'info'})=={'state':'ok'}


def test_unused_lost_claim_expires_with_confirmed_hold_and_releases():
    gate,_,writes,advance,_=rig()
    advance(101);gate.tick()
    assert gate.state=='ready' and not writes
    advance(200);gate.tick()
    assert gate.session_id and gate.release_requested and writes[-1][1] is None
    advance(20);gate.tick()
    assert gate.session_id is None and gate.status()['stop_confirmed']


def test_foreign_ros_publisher_refuses_the_shared_actuator_boundary():
    from teleop_executor import TeleopExecutor
    from types import SimpleNamespace as NS
    own=NS(get_namespace=lambda:'/',get_name=lambda:'driver_arm')
    executor=TeleopExecutor.__new__(TeleopExecutor)
    executor.plugins=[NS(_pub_node=own)]
    executor.node=NS(get_publishers_info_by_topic=lambda topic:[NS(node_namespace='/',node_name='driver_arm'),
        NS(node_namespace='/',node_name='teleop_dispatcher')] if topic=='/arm/cmd_pos' else [])
    assert executor.foreign_publishers()==[{'topic':'/arm/cmd_pos','node':'/teleop_dispatcher'}]


def test_hand_fault_and_missing_streams_fail_closed(monkeypatch):
    from teleop_executor import TeleopExecutor
    from types import SimpleNamespace as NS
    import threading,time,types
    import sys
    monkeypatch.setitem(sys.modules,'device',types.SimpleNamespace(
        _HEAD_JOINTS={1:'h'},_WAIST_JOINTS={31:'w'},_LEG_JOINTS={51:'l'}))
    executor=TeleopExecutor.__new__(TeleopExecutor);executor._lock=threading.RLock()
    executor._feedback_error=None
    now=time.monotonic_ns()
    executor._streams={p:(now,{mid:(0.,0.,0) for mid in mids}) for p,mids in
        [('arm',list(range(11,18))+list(range(21,28))),('head',[1]),('waist',[31]),('leg',[51])]}
    executor._power=(now,True,False);executor._base=(now,[0.,0.,0.,0.,0.,0.,1.],True)
    executor.profile={'fixed_motor_positions_rad':{'1':0.,'31':0.,'51':0.}}
    executor._fixed_baseline={'1':0.,'31':0.,'51':0.}
    executor._hands={s:(now,[1.]*6) for s in ('left','right')}
    executor._hand_errors={s:(now,[0]*6) for s in ('left','right')}
    assert not executor.feedback()['fault']
    executor._feedback_error='RuntimeError'
    assert executor.feedback()['fault']
    executor._feedback_error=None
    executor._base = None
    assert executor.feedback()['fixed_body']
    assert executor.feedback()['fixed_ns'] == now
    executor._streams['waist'] = (now, {31: (.1, 0., 0)})
    assert not executor.feedback()['fixed_body']
    executor._hand_error_cb('left',NS(data=[0,0,1,0,0,0]))
    assert executor.feedback()['fault']
    del executor._hand_errors['left']
    assert executor.feedback()['fault'] and executor.feedback()['hand_ns']==0


def test_competitor_after_claim_and_graph_failure_hold_before_target():
    from teleop_executor import TeleopExecutor
    gate,status,writes,advance,packet=rig()
    executor=TeleopExecutor.__new__(TeleopExecutor);executor.gate=gate
    executor.foreign_publishers=lambda:[{'node':'competitor'}]
    gate.accept(packet());advance(20);executor.tick()
    assert gate.state=='hold' and gate.reason=='external_motion_publishers_present'
    assert writes == [([0.]*14,None)]  # Only the measured hold target.
    executor.foreign_publishers=lambda:(_ for _ in ()).throw(RuntimeError('graph unavailable'))
    advance(20);executor.tick()
    assert gate.state=='hold' and gate.session_id


def test_vendor_power_documentation_is_not_an_admission_gate():
    from teleop_executor import accepted
    flags=('model_verified','workspace_verified','stop_verified','driver_crash_verified',
           'pico_verified','external_control_excluded')
    evidence={k:True for k in flags}
    evidence.update(operator='test',date='test',evidence_sha256='a'*64)
    evidence['power_feedback_verified']=False
    assert accepted({'acceptance':evidence})
    evidence['stop_verified']=False
    assert not accepted({'acceptance':evidence})  # Measured normal-Live stop acceptance remains.


def test_expired_packet_resume_requires_confirmed_stop_and_discards_old_session():
    gate, status, writes, advance, packet = rig()
    old = packet()
    advance(101)
    with pytest.raises(ValueError, match='command_expired'):
        gate.accept(old)
    with pytest.raises(ValueError, match='hold_not_resumable'):
        gate.resume()
    gate.tick(); advance(20); gate.tick()
    old_session = gate.session_id
    lease = gate.resume()
    assert lease['session_id'] != old_session
    assert gate.latest is None and gate.applied_seq == -1
    before = len(writes)
    with pytest.raises(ValueError, match='command_mac|stale_session'):
        gate.accept(old)
    assert len(writes) == before
    gate.tick(); advance(20); gate.tick()
    assert gate.state == 'ready'  # Foreign traffic did not create a HOLD.
    with pytest.raises(ValueError, match='hold_not_resumable'):
        gate.resume()  # No second resume is needed for the new session.


def test_power_staleness_remains_a_fault_after_timeout_hold():
    gate, status, writes, advance, packet = rig()
    gate.accept(packet());advance(101);gate.tick();advance(20);gate.tick()
    assert gate.status()['hold_confirmed']
    status['power_ns'] -= 301_000_000
    before = len(writes)
    gate.tick()
    assert gate.state == 'fault' and gate.reason == 'power_ns_stale'
    assert gate.session_id and not gate.status()['stop_confirmed']
    assert len(writes) == before
    advance(20);gate.tick()
    with pytest.raises(ValueError, match='hold_not_resumable'):
        gate.resume()  # Fresh feedback does not silently erase a hardware fault.


def test_command_progresses_past_actuator_deadband_without_unbounded_windup():
    gate,status,writes,advance,packet=rig()
    gate.velocity=1.
    # Deliberately stalled plant: commands may progress but cannot wind up.
    for seq in range(20):
        gate.accept(packet(seq=seq,q=[.5]*14));advance(20);gate.tick()
    goals=[q for q,h in writes]
    assert goals[-1]==pytest.approx([.2]*14)
    assert all(max(abs(b-a) for a,b in zip(x,y))<=.0200001
               for x,y in zip([[0.]*14]+goals,goals))
    # A synthetic deadband plant exposes the measured-reset stall; this is
    # a regression model, not evidence of the actual vendor deadband value.
    for seq in range(20,70):
        for i,goal in enumerate(writes[-1][0]):
            delta=goal-status['q'][i]
            if abs(delta)>.025:status['q'][i]+=max(-.01,min(.01,delta))
        gate.accept(packet(seq=seq,q=[.5]*14));advance(20);gate.tick()
        assert max(abs(a-b) for a,b in zip(writes[-1][0],status['q']))<=.2000001
    assert status['q'][0]>.3
    gate.hold();advance(20);gate.tick()
    assert writes[-1][0]==status['q'] and writes[-1][1] is None
    assert gate.status()['commanded_q']==status['q']


def test_stop_reholds_settled_offset_once_and_requires_new_receipt():
    gate,status,writes,advance,_=rig()
    gate.velocity=1.0
    gate.hold(release=True);gate.tick()
    status['q']=[0.0318]+[0.0]*13
    advance(20);gate.tick()
    count=len(writes)
    for _ in range(5):
        gate.tick()  # Re-reading one feedback sample cannot prove stationarity.
    assert len(writes)==count and not gate.status()['stop_confirmed']
    advance(100);gate.tick()
    assert len(writes)==count+1 and writes[-1][0]==status['q']
    assert gate.session_id and not gate.status()['stop_confirmed']
    gate.tick()
    assert gate.session_id
    advance(20);gate.tick()
    assert gate.session_id is None and gate.status()['stop_confirmed']


@pytest.mark.parametrize('moving,offset', [(True,0.0318),(False,0.11)])
def test_stop_does_not_rehold_moving_or_out_of_bound_robot(moving,offset):
    gate,status,writes,advance,_=rig();gate.velocity=1.0
    gate.hold(release=True);gate.tick();count=len(writes)
    status['q']=[offset]+[0.0]*13
    status['dq']=[0.03 if moving else 0.0]*14
    for _ in range(101):
        advance(20);gate.tick()
    assert len(writes)==count and gate.state=='fault'
    assert gate.session_id and not gate.status()['stop_confirmed']


def test_stop_rehold_cannot_chase_second_offset_or_extend_deadline():
    gate,status,writes,advance,_=rig();gate.velocity=1.0
    gate.hold(release=True);gate.tick()
    status['q']=[0.0318]+[0.0]*13
    advance(20);gate.tick();advance(100);gate.tick()
    assert len(writes)==2
    status['q']=[0.065]+[0.0]*13
    for _ in range(100):
        advance(20);gate.tick()
    assert len(writes)==2 and gate.state=='fault'
    assert gate.session_id and not gate.status()['stop_confirmed']


def test_slow_settling_has_bounded_receipt_window_and_stale_cannot_reset_it():
    gate,status,writes,advance,_=rig();gate.velocity=1.
    gate.hold(release=True);gate.tick();started=gate._stop_started_ns
    for i in range(30):
        status['q'][0]=.03+.003*i
        advance(20);gate.tick()
    assert gate.state=='hold' and len(writes)==1
    status['q'][0]=.09
    advance(20);gate.tick();advance(120);gate.tick()
    assert len(writes)==2 and gate._stop_reheld
    status['q'][0]=.06
    advance(120,fresh=False);gate.tick()
    assert gate._stop_started_ns==started and gate.stop_sent_ns is not None
    advance(20);gate.tick()
    assert gate._stop_reheld and len(writes)==2
    advance(1500);gate.tick()
    assert gate.state=='fault' and len(writes)==2 and gate.session_id


def test_identified_slow_position_response_with_bounded_lead(monkeypatch):
    import motion_stream
    def simulate(horizon):
        monkeypatch.setattr(motion_stream,'POSITION_LEAD_SECONDS',horizon)
        gate,status,writes,advance,packet=rig();gate.velocity=1.
        errors=[]
        for seq in range(200):
            if writes:
                for j in range(14):
                    dq=max(-1.,min(1.,1.8*(writes[-1][0][j]-status['q'][j])))
                    status['dq'][j]=dq;status['q'][j]+=dq*.02
            target=min(.6,seq*.02*.25)
            gate.accept(packet(seq=seq,q=[target]*14));advance(20);gate.tick()
            assert gate.state=='active'
            assert max(abs(c-q) for c,q in zip(writes[-1][0],status['q']))<=horizon+1e-9
            errors.append(abs(target-status['q'][0]))
        gate.hold();advance(20);gate.tick()
        assert writes[-1][0]==status['q']
        status['dq']=[0.]*14;advance(20);gate.tick()
        assert gate.status()['hold_confirmed']
        return sum(errors)
    # Identified first-order approximation, not a physical acceptance test.
    assert simulate(.2)<.9*simulate(.1)


def test_old_session_packet_cannot_stop_or_extend_new_session():
    gate,status,writes,advance,packet=rig();old=packet()
    gate.accept(old);advance(20);gate.tick()
    gate.hold();advance(20);gate.tick();advance(20);gate.tick()
    lease=gate.resume();deadline=gate.lease_deadline;count=len(writes)
    with pytest.raises(ValueError,match='stale_session'):gate.accept(old)
    assert gate.state=='ready' and gate.seq==-1 and gate.latest is None
    assert gate.lease_deadline==deadline and len(writes)==count
    assert gate.diagnostics['last_rejected_command']['foreign_session'] is True
    fresh={k:v for k,v in packet(seq=1).items() if k!='mac'}
    fresh['session_id']=lease['session_id'];fresh['mac']=sign(fresh,lease['secret'])
    gate.accept(fresh);advance(20);gate.tick();assert gate.applied_seq==1
    deadline=gate.lease_deadline;count=len(writes)
    with pytest.raises(ValueError,match='stale_session'):gate.accept(old)
    assert gate.state=='active' and gate.lease_deadline==deadline and len(writes)==count
    forged=dict(fresh);forged['seq']=2  # Current owner, wrong MAC: still HOLD.
    with pytest.raises(ValueError,match='invalid_command_mac'):gate.accept(forged)
    assert gate.state=='hold' and gate.reason=='invalid_command_mac'
