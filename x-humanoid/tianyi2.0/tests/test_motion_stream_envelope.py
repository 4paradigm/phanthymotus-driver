"""Execution-only envelope/clock/recovery tests; no IK, DDS or robot."""
import copy
import threading
from types import SimpleNamespace

import pytest
from common.motion.protocol import envelope
from motion_stream import sign
from teleop_executor import TeleopExecutor
from test_motion_stream import rig


def command(gate, packet, seq=1, *, q=.5, width=.8, lifetime=300):
    now=gate.clock()
    body=envelope({'boot_id':gate.boot_id,'session_id':gate.session_id,'secret':gate.secret},
        seq=seq,source_seq=seq,mapping_epoch=1,generated_ns=now,valid_until_ns=now+lifetime*1_000_000,
        mode='joint_position',values=[q]*14,model_version='m',calibration_version='c',frame='torso')
    proof={k:body[k] for k in ('boot_id','session_id','seq','source_seq','mapping_epoch',
        'model_version','calibration_version','frame','valid_until_ns')}
    proof.update(schema='motus.motion.envelope/1',measured_ns=now,lower=[-width]*14,upper=[width]*14)
    old=packet(seq=seq,q=[q]*14,valid_for_ms=lifetime)
    return body,proof,old


def accept(gate, item):
    body,proof,old=item
    gate.install_motion_envelope(proof)
    return gate.accept(old,max_valid_for_ms=300,motion_command=body)


def test_twenty_hz_targets_progress_on_each_execution_tick_and_stop_at_source_expiry():
    gate,state,writes,advance,packet=rig();gate.velocity=1.
    for seq in range(10):
        accept(gate,command(gate,packet,seq=seq))
        for _ in range(5):
            advance(10);gate.tick()
            # Finite-speed plant, independently advances by <= .01 rad each tick.
            for i,t in enumerate(writes[-1][0]):state['q'][i]+=max(-.01,min(.01,t-state['q'][i]))
    assert len(writes)==50 and writes[-1][0][0]==pytest.approx(.5)
    assert all(0<=b[0][0]-a[0][0]<=.010000001 for a,b in zip(writes,writes[1:]))
    advance(251);gate.tick()
    assert gate.state=='hold' and gate.latest is None and writes[-1][1] is None


def test_full_reference_clips_at_verified_box_without_cycling_hold_and_resume():
    gate,state,writes,advance,packet=rig();gate.velocity=1.
    item=command(gate,packet,width=.025)
    accept(gate,item)
    advance(20);gate.tick();assert writes[-1][0][0]==.02
    state['q']=[.02]*14
    advance(20);gate.tick()
    assert writes[-1][0]==pytest.approx([.025]*14) and gate.state=='active'
    state['q']=[.025]*14
    advance(20);gate.tick();assert writes[-1][0]==pytest.approx([.025]*14)
    accept(gate,command(gate,packet,seq=2))
    advance(20);gate.tick();assert gate.state=='active'
    assert writes[-1][0][0]>.025


def test_measured_escape_from_proof_still_holds_without_unproved_output():
    gate,state,writes,advance,packet=rig();gate.velocity=1.
    accept(gate,command(gate,packet,width=.025))
    state['q']=[.03]*14
    advance(20);gate.tick()
    assert gate.reason=='motion_envelope_exceeded' and writes[-1]==(state['q'],None)


@pytest.mark.parametrize('field,value',[('seq',99),('mapping_epoch',2),('model_version','wrong'),('calibration_version','wrong')])
def test_identity_changed_proof_never_authorizes_command(field,value):
    gate,_,writes,_,packet=rig();body,proof,old=command(gate,packet)
    proof[field]=value
    gate.install_motion_envelope(proof)
    with pytest.raises(ValueError,match='motion_envelope'):gate.accept(old,max_valid_for_ms=300,motion_command=body)
    assert not writes and gate.latest is None


def test_long_gap_does_not_accrue_motion_credit_and_resume_preserves_session():
    gate,state,writes,advance,packet=rig();gate.velocity=1.
    accept(gate,command(gate,packet));advance(20);gate.tick()
    session=gate.session_id
    advance(301);gate.tick();advance(20);gate.tick()
    advance(2000);gate.tick()
    assert gate.status()['continuation_ready']
    accept(gate,command(gate,packet,seq=2));gate.tick()
    assert gate.session_id==session and max(writes[-1][0])<=.02


def test_state_congestion_is_latest_only_and_does_not_close_owner():
    gate,_,_,_,_=rig();item=TeleopExecutor({},'test',None,None,None,[]);item.gate=gate
    item.info=lambda:{'state':'ready'}
    item._send_bus=lambda raw:(_ for _ in ()).throw(BlockingIOError())
    item._status_once()
    assert gate.state=='ready' and gate.session_id and item._bus_health['status_dropped']==1
    sent=[];item._send_bus=sent.append;item._status_once();assert len(sent)==1


def test_bus_recovery_changes_transport_only_and_keeps_release_receipt():
    gate,_,_,advance,packet=rig();accept(gate,command(gate,packet))
    item=TeleopExecutor({},'test',None,None,None,[]);item.gate=gate
    item._communication_hold('local_dds_process_exited')
    session,secret=gate.session_id,gate.secret
    gate.release_requested=True
    receipt={'id':'release','result':{'state':'pending'}};item._management_receipt=receipt
    old=SimpleNamespace(close=lambda:None);item._bus_socket=old
    item._bus_process=SimpleNamespace(poll=lambda:0,wait=lambda **k:0)
    new=object();process=object();item._spawn_bus=lambda:(new,process)
    item._recover_bus()
    assert item._bus_socket is new and item._bus_process is process
    assert gate.session_id==session and gate.secret==secret and gate.release_requested
    assert item._management_receipt is receipt and gate.latest is None


def test_status_work_is_outside_watchdog_even_when_observer_blocks():
    gate,_,_,_,_=rig();item=TeleopExecutor({},'test',None,None,None,[]);item.gate=gate
    entered,release=threading.Event(),threading.Event()
    def blocked():entered.set();release.wait(1);return {}
    item.info=blocked;item._send_bus=lambda _:None
    thread=threading.Thread(target=item._status_once);thread.start();assert entered.wait(.5)
    try:
        gate.hold(release=True)
        assert gate.release_requested
    finally:release.set();thread.join(.5)


def test_twenty_hz_execution_uses_actual_elapsed_not_fixed_twenty_ms():
    gate,state,writes,advance,packet=rig();gate.velocity=1.
    for seq in range(8):
        accept(gate,command(gate,packet,seq=seq))
        advance(50);gate.tick()
        state['q']=list(writes[-1][0])
    assert writes[-1][0][0]==pytest.approx(.4)
    assert all(b[0][0]-a[0][0]==pytest.approx(.05) for a,b in zip(writes,writes[1:]))


def test_optional_acceleration_limits_ramp_and_arrival_speed():
    gate,state,writes,advance,packet=rig();gate.velocity=1.;gate.acceleration=2.
    velocities=[];previous=0.
    for seq in range(100):
        accept(gate,command(gate,packet,seq=seq,q=.2))
        advance(20);gate.tick()
        now=writes[-1][0][0]
        velocities.append((now-previous)/.02)
        state['q']=list(writes[-1][0]);previous=now
    assert previous==pytest.approx(.2,abs=1e-4)
    assert max(abs(v) for v in velocities)<=1.
    assert max(abs(b-a) for a,b in zip([0.]+velocities,velocities))<=.0400001
    assert abs(velocities[-1])<.001
