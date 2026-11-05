"""G1 card/codec fixtures; explicit executor and numerical substitutes, no SDK."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(DRIVER))
from motion_control import MotionControl, JOINT_NAMES
from common.motion.protocol import envelope, validate
from common.motion.envelope import MotionEnvelope


class Gate:
    def __init__(self):
        self.clock=time.monotonic_ns; self.boot_id='boot'; self.session_id='session'; self.secret='12'*32
        self.seq=-1; self.state='ready'; self.holds=[]
    def status(self):
        return {'state':self.state,'boot_id':self.boot_id,'session_id':self.session_id,
            'ownership_held':bool(self.session_id),'output_active':False,'hold_confirmed':self.state=='hold',
            'monotonic_ns':self.clock(),'commanded_q':[0.]*10,
            'feedback':{'q':[0.]*10,'dq':[0.]*10,'arm_ns':self.clock()}}
    def hold(self,reason,**kwargs): self.state='hold'; self.holds.append(reason)


class Worker:
    profile={'urdf_sha256':'model','torso_frame':'pelvis','arm_joint_names':list(JOINT_NAMES)}
    profile_sha256='calibration'; lower=[-2.]*10; upper=[2.]*10; velocity=1.;last_ms=4.
    def __init__(self): self.frames=[];self.last_envelope=None
    def solve_frame(self,body,state,generation):
        self.frames.append(body['source_seq'])
        self.last_envelope={'lower':[-.2]*10,'upper':[.2]*10,'measured_ns':state['feedback']['arm_ns']}
        return [.4]*10
    def render(self,state,*args):return {'poses':[[0,0,0,0,0,0,1]]*2,'visualization':{'available':True}}


def controller():
    gate=Gate(); published=[];proofs=[]
    executor=SimpleNamespace(gate=gate,ns='g1',profile_sha256='calibration',start=lambda:None,
        info=gate.status,install_motion_envelope=lambda r:proofs.append(MotionEnvelope.from_record(r,dof=10)),
        publish_joint_command=published.append)
    c=MotionControl({},executor);c.solver=Worker();c._live_session=gate.session_id
    return c,published,proofs


def frame(c,seq=1,epoch=0):
    now=time.monotonic_ns()
    return envelope(c._lease(),seq=seq,source_seq=seq,mapping_epoch=epoch,
        generated_ns=now,valid_until_ns=now+300_000_000,mode='eef_pose',
        values=[0,0,0,0,0,0,1]*2,**c.versions)


def test_same_schema_eef14_to_g1_joint10_with_proof_and_original_deadline():
    c,published,proofs=controller();p=frame(c)
    c.receive_eef(p);assert c.process_latest()
    q=published[0]
    assert q['dof']==10 and q['values']==[.4]*10
    assert q['schema']=='motus.control/2' and q['mode']=='joint_position'
    assert q['valid_until_ns']==p['valid_until_ns']
    assert proofs[0].matches(q,now_ns=time.monotonic_ns())
    assert not proofs[0].contains(q['values'])  # Full reference; arm can only execute proved nearby part.
    assert proofs[0].allows(q,[0.]*10,[0.]*10,[.02]*10,now_ns=time.monotonic_ns())


def test_latest_pending_only_not_history_replay():
    c,published,_=controller()
    for seq in range(1,31):c.receive_eef(frame(c,seq))
    assert c.process_latest()
    assert c.solver.frames==[30] and len(published)==1


def test_solver_failure_holds_then_new_frame_can_continue_same_mapping():
    c,published,_=controller();original=c.solver.solve_frame
    def failed(*args):raise ValueError('ik_target_unreachable')
    c.solver.solve_frame=failed;c.receive_eef(frame(c));assert not c.process_latest()
    assert c.gate.holds==['ik_recoverable'] and not published
    c.gate.state='ready';c.solver.solve_frame=original
    c.receive_eef(frame(c,2));assert c.process_latest()
    assert published[0]['session_id']=='session' and published[0]['mapping_epoch']==0


def test_old_mapping_and_wrong_physical_joint_dimension_rejected():
    c,_,_=controller();c.receive_eef(frame(c,1,2))
    with pytest.raises(ValueError,match='stale_mapping_epoch'):c.receive_eef(frame(c,2,1))
    now=time.monotonic_ns();packet=envelope(c._lease(),seq=3,source_seq=3,mapping_epoch=2,
        generated_ns=now,valid_until_ns=now+300_000_000,mode='joint_position',values=[0.]*14,**c.versions)
    with pytest.raises(ValueError,match='invalid_control_interface'):
        validate(packet,c._lease(),mode='joint_position',now=now,dof=10,**c.versions)


def test_public_feedback_and_port_contract_is_single_topic():
    c,_,_=controller();info=c.info()
    assert info['schema']=='motus.motion.feedback/1'
    assert [p['topic'] for p in info['topic_out']]==['/g1/motion/arm/command','/g1/motion/teleop/feedback']
    assert info['control_interface']['dof']==14
    joint=info['control_interfaces']['joints']
    assert joint['dof']==10 and joint['force_torque'] is None and joint['joint_names']==list(JOINT_NAMES)


@pytest.mark.parametrize('changed', ['measured', 'commanded'])
def test_moving_snapshot_discards_candidate_without_holding(changed):
    c, published, proofs = controller()
    state = c.gate.status()
    c.gate.status = lambda: state
    solve = c.solver.solve_frame
    def moving_solve(*args):
        result = solve(*args)
        if changed == 'measured': state['feedback']['q'] = [.3]*10
        else: state['commanded_q'] = [.3]*10
        state['feedback']['arm_ns'] = time.monotonic_ns()
        return result
    c.solver.solve_frame = moving_solve
    c.receive_eef(frame(c))
    assert not c.process_latest()
    assert c._decision['state'] == 'rejected'
    assert c._decision['reason'] == 'motion_envelope_stale_start'
    assert not published and not proofs and not c.gate.holds
    assert c._intent is None and c.gate.state == 'ready'


def test_rejected_candidates_do_not_renew_previous_executor_deadline(tmp_path):
    from test_arm_stream import rig
    arm, sample, channel, advance, _, _ = rig(tmp_path)
    arm.set_joint_publisher(arm.accept_control)
    c = MotionControl({}, arm)
    c.solver = Worker()
    c.solver.profile, c.solver.profile_sha256 = arm.profile, arm.profile_sha256
    c._live_session = arm.session_id
    solve = c.solver.solve_frame
    reject = False
    def candidate(*args):
        solve(*args)
        if reject: c.solver.last_envelope.update(lower=[1.]*10, upper=[2.]*10)
        return [.1]*10
    c.solver.solve_frame = candidate
    def submit(seq):
        now = arm.clock()
        body = envelope(c._lease(), seq=seq, source_seq=seq, mapping_epoch=0,
            generated_ns=now, valid_until_ns=now+300_000_000, mode='eef_pose',
            values=[0,0,0,0,0,0,1]*2, **c.versions)
        c.receive_eef(body)
        return c.process_latest()
    assert submit(1), c._decision
    advance(10); arm.tick()
    deadline, previous_seq, intent = arm.latest['valid_until_ns'], arm.seq, c._intent
    reject = True
    for seq in range(2, 8):
        assert not submit(seq)
        assert arm.latest['valid_until_ns'] == deadline and arm.seq == previous_seq
        assert c._intent is intent and c._decision['state'] == 'rejected'
        assert arm.state == 'active'
        advance(50); arm.tick()
    assert arm.clock() > deadline
    assert arm.state == 'hold' and arm.reason == 'command_timeout'
    assert arm.latest is None and not arm.status()['output_active']
    assert arm.applied_seq == previous_seq
