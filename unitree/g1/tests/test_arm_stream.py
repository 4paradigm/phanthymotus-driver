"""Offline actuator/state-machine evidence, with explicit SDK/plant substitutes.

Synthetic calibration is never evidence for robot acceptance. No start() or
ChannelFactory call is made; the actual protocol, envelope, limiter and release
state machine run with a deterministic clock.
"""
import ast
import hashlib
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

DRIVER=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(DRIVER))
from arm_stream import ArmStreamExecutor, GravityCompensation, JOINT_NAMES, MOTOR_IDS
from common.motion.protocol import envelope


class Channel:
    def __init__(self):self.writes=[];self.closed=False;self.success=True
    def publish_stream(self,q,tau,*,weight):
        if not self.success:return False
        self.writes.append((list(q),None if tau is None else list(tau),weight));return True
    def close(self):self.closed=True


def calibration(tmp_path):
    data=json.loads((DRIVER/'g1_motion/calibration.example.json').read_text())
    data['urdf_path']=str(DRIVER/'g1_motion/models/g1_body23.urdf')
    data['locked_joints']=dict.fromkeys(data['locked_joints'],0.)
    path=tmp_path/'synthetic.json';path.write_text(json.dumps(data));return path


def rig(tmp_path,*,gravity=None,client=None,claim=True,servo=False):
    now=[1_000_000_000];q=[0.]*10
    sample={'q':q,'dq':[0.]*10,'arm_ns':now[0],'fault':False,'model_verified':True}
    channel=Channel();gravity_calls=[];sdk=[]
    def compensate(values):gravity_calls.append(list(values));return [x+.1 for x in values]
    arm=ArmStreamExecutor({'live_enabled':True,'servo_position':servo},'g1',client or SimpleNamespace(ExecuteAction=lambda x:sdk.append(x) or 0),
        snapshot=lambda:sample,channel=channel,gravity=gravity or compensate,clock=lambda:now[0])
    arm.configure_profile(calibration(tmp_path));arm._prepared=True
    if claim:assert arm.dispatch('claim',{})['state']=='ready'
    def advance(ms,*,fresh=True,follow=True):
        now[0]+=int(ms*1e6)
        if fresh:sample['arm_ns']=now[0]
        if follow and channel.writes:sample['q']=channel.writes[-1][0][:]
    return arm,sample,channel,advance,gravity_calls,sdk


def packet(arm,seq=1,*,target=.5,width=1.,life=300,changes=None):
    now=arm.clock()
    body=envelope(dict(boot_id=arm.boot_id,session_id=arm.session_id,secret=arm.secret),
        seq=seq,source_seq=seq,mapping_epoch=1,generated_ns=now,valid_until_ns=now+life*1_000_000,
        mode='joint_position',dof=10,values=[target]*10,**arm.versions)
    record={k:body[k] for k in ('boot_id','session_id','seq','source_seq','mapping_epoch','model_version',
        'calibration_version','frame','valid_until_ns')}
    record.update(schema='motus.motion.envelope/1',measured_ns=now,lower=[-width]*10,upper=[width]*10)
    record.update(changes or {})
    arm.install_motion_envelope(record)
    return body


def test_twenty_hz_inputs_execute_continuously_at_one_rad_per_second(tmp_path):
    arm,s,ch,advance,gravity,_=rig(tmp_path)
    for seq in range(10):
        assert arm.accept_control(packet(arm,seq))
        for _ in range(5):advance(10);arm.tick()
    assert len(ch.writes)==50
    assert ch.writes[-1][0]==pytest.approx([.5]*10)
    assert all(b[0][0]-a[0][0]<=.010000001 for a,b in zip(ch.writes,ch.writes[1:]))
    assert gravity==[w[0] for w in ch.writes]
    assert ch.writes[-1][1]==pytest.approx([.6]*10)
    assert ch.writes[-1][2]==pytest.approx(.25)
    advance(251);arm.tick()
    assert arm.state=='hold' and arm.latest is None and ch.writes[-1][1] is None


def test_clock_delay_cannot_accumulate_motion_and_old_packets_cannot_resume(tmp_path):
    arm,_,ch,advance,_,_=rig(tmp_path)
    old=packet(arm);arm.accept_control(old)
    advance(200);arm.tick();assert ch.writes[-1][0]==pytest.approx([.02]*10)
    advance(101);arm.tick();advance(10);arm.tick()
    assert arm.status()['continuation_ready']
    session=arm.session_id
    advance(2000)
    with pytest.raises(ValueError):arm.accept_control(old)
    arm.accept_control(packet(arm,2));arm.tick()
    assert arm.session_id==session and ch.writes[-1][0]==pytest.approx([.04]*10)


def test_nearby_proof_does_not_authorize_complete_endpoint(tmp_path):
    arm,_,ch,advance,_,_=rig(tmp_path)
    arm.accept_control(packet(arm,width=.025));advance(20);arm.tick()
    advance(20);arm.tick()
    assert arm.reason=='motion_envelope_exceeded' and len(ch.writes)==1
    advance(10);arm.tick();advance(10);arm.tick()
    assert arm.status()['continuation_ready']
    arm.accept_control(packet(arm,2));advance(20);arm.tick()
    assert arm.state=='active'


@pytest.mark.parametrize('changes',[{'seq':99},{'mapping_epoch':2},{'calibration_version':'bad'},{'model_version':'bad'},
                                    {'frame':'bad'},{'source_seq':8}])
def test_proof_identity_mismatch_never_writes(tmp_path,changes):
    arm,_,ch,_,_,_=rig(tmp_path)
    with pytest.raises(ValueError,match='motion_envelope'):arm.accept_control(packet(arm,changes=changes))
    assert not ch.writes


@pytest.mark.parametrize('failure',['gravity','torque','sdk'])
def test_compensation_or_sdk_failure_is_not_zero_torque_or_success(tmp_path,failure):
    arm,_,ch,advance,_,_=rig(tmp_path)
    if failure=='gravity':arm._gravity=lambda _:(_ for _ in ()).throw(RuntimeError('model_failed'))
    if failure=='torque':arm._gravity=lambda _:[999.]*10
    if failure=='sdk':ch.success=False
    arm.accept_control(packet(arm));advance(10);arm.tick()
    assert arm.state=='fault' and arm.applied_seq==-1 and not ch.writes
    assert arm.first_fault and arm.latest is None


def test_transient_feedback_loss_holds_then_fresh_frame_can_resume_without_restart(tmp_path):
    arm,s,ch,advance,_,_=rig(tmp_path)
    arm.accept_control(packet(arm));advance(10);arm.tick()
    advance(101,fresh=False);arm.tick();assert arm.state=='hold' and len(ch.writes)==1
    advance(10);arm.tick();advance(10);arm.tick()
    assert arm.status()['continuation_ready']
    arm.accept_control(packet(arm,2));advance(10);arm.tick();assert arm.state=='active'
    s['error']='invalid_sample';advance(10);arm.tick();assert arm.state=='fault'
    s.pop('error');advance(10);arm.tick();assert arm.state=='fault'


def progress_release(arm,advance,until):
    for _ in range(300):
        advance(10);arm.tick()
        if arm._rpc_thread:arm._rpc_thread.join(.05)
        if arm._operation['state']==until:return
    pytest.fail(str(arm._operation))


def test_release_requires_real_zero_weight_writes_then_new_measured_confirmation(tmp_path):
    arm,s,ch,advance,_,sdk=rig(tmp_path)
    arm.accept_control(packet(arm));advance(10);arm.tick()
    credentials={'session_id':arm.session_id,'secret':arm.secret}
    args={**credentials,'operation_id':'release-one'}
    first=arm.dispatch('release',args)
    assert first['state']=='holding' and not first['return_completed']
    assert arm.dispatch('release',args)==first
    with pytest.raises(ValueError):arm.accept_control(packet(arm,2))
    progress_release(arm,advance,'awaiting_feedback')
    op=arm.dispatch('finish_status',{'operation_id':'release-one'})
    assert sdk==[99] and op['zero_writes']==5 and op['handback_confirmed']
    assert op['sdk_return']==0 and not op['physical_confirmed'] and arm.session_id
    stamp=s['arm_ns'];advance(10,fresh=False);arm.tick();assert not arm._operation['physical_confirmed']
    progress_release(arm,advance,'completed')
    assert s['arm_ns']>stamp and ch.closed and arm.session_id is None
    final=arm.dispatch('release',{'operation_id':'release-one'})
    assert final['physical_confirmed'] and final['return_completed'] and sdk==[99]
    assert arm.dispatch('release',args)==final  # Retry may carry the retired lease.
    assert arm.stop()['authority_released'] and arm._closed.is_set()


def test_unknown_sdk_receipt_never_retries_implicitly(tmp_path):
    invoked=[]
    def fail(action):invoked.append(action);raise TimeoutError('unknown_sdk')
    arm,_,_,advance,_,_=rig(tmp_path,client=SimpleNamespace(ExecuteAction=fail))
    arm.dispatch('release',{'operation_id':'one'});progress_release(arm,advance,'unknown')
    assert arm.dispatch('release',{'operation_id':'two'})['code']=='release_result_unknown'
    assert arm.dispatch('release',{'operation_id':'one'})['state']=='unknown'
    assert invoked==[99]
    assert arm.status()['ownership_held']
    with pytest.raises(ValueError,match='motion_owned_by_teleop'):
        with arm.legacy():pass
    assert arm.dispatch('release',{'operation_id':'two','retry':True})['state']=='holding'


def test_stop_cancels_pending_99_but_still_completes_safe_sdk_handback(tmp_path):
    arm,_,_,advance,_,sdk=rig(tmp_path)
    arm.dispatch('release',{'operation_id':'cancel-me'});advance(10);arm.tick()
    stopped=arm.dispatch('stop',{})
    assert stopped['cancel_requested'] and stopped['action_id'] is None
    progress_release(arm,advance,'completed')
    assert sdk==[] and arm._operation['handback_confirmed'] and arm.session_id is None


def test_stop_after_99_was_sent_reports_vendor_cancel_unknown(tmp_path):
    arm,_,_,advance,_,sdk=rig(tmp_path)
    arm.dispatch('release',{'operation_id':'already-sent'})
    progress_release(arm,advance,'awaiting_feedback')
    stopped=arm.dispatch('stop',{})
    assert sdk==[99] and stopped['state']=='unknown'
    assert stopped['reason']=='release_action_stop_unconfirmed'
    assert arm.session_id and not stopped['physical_confirmed']


def test_legacy_rpc_return_does_not_release_ownership_and_stop_uses_99(tmp_path):
    arm,_,ch,advance,_,sdk=rig(tmp_path,claim=False)
    arm._channel_open=False
    with arm.legacy():pass
    assert arm.dispatch('claim',{})['code']=='legacy_motion_pending'
    result=arm.dispatch('stop',{'operation_id':'legacy-stop'})
    assert result['action_id']==99
    progress_release(arm,advance,'awaiting_feedback')
    assert sdk==[99] and arm._operation['zero_writes']==0 and not ch.writes
    assert arm._legacy_pending
    progress_release(arm,advance,'completed');assert not arm._legacy_pending


def test_release_rpc_timeout_does_not_block_stop_or_issue_second_rpc(tmp_path):
    entered,finish=threading.Event(),threading.Event()
    def blocked(_):entered.set();finish.wait(2);return 0
    arm,_,_,advance,_,_=rig(tmp_path,client=SimpleNamespace(ExecuteAction=blocked),claim=False)
    arm._channel_open=False
    arm.dispatch('release',{'operation_id':'blocked'});advance(10);arm.tick()
    assert entered.wait(.5)
    try:
        advance(10001);arm.tick()
        assert arm._operation['state']=='unknown'
        assert arm.dispatch('stop',{})['code']=='release_result_unknown'
    finally:finish.set();arm._rpc_thread.join(.5)
    assert arm._operation['state']=='unknown'


def test_management_retry_and_lost_resume_reply_cancel_exact_new_lease(tmp_path):
    arm,_,_,advance,_,_=rig(tmp_path,claim=False)
    request={'request_id':'a'*32,'request_valid_until_ns':arm.clock()+300_000_000}
    result=arm.dispatch('claim',request)
    assert arm.dispatch('claim',request)==result
    arm.dispatch('pause',result);advance(10);arm.tick();advance(10);arm.tick()
    request={**result,'request_id':'b'*32,'request_valid_until_ns':arm.clock()+300_000_000}
    resumed=arm.dispatch('resume',request)
    assert resumed['session_id']!=result['session_id'] and arm.dispatch('resume',request)==resumed
    cancelled=arm.dispatch('release',request)
    assert cancelled['state']=='holding' and arm.release_requested
    assert arm.dispatch('resume',request)['code']=='management_cancelled'


def test_configuration_digest_rejection_preserves_previous_model(tmp_path):
    arm,_,_,_,_,_=rig(tmp_path,claim=False)
    previous=(arm.profile,arm.profile_sha256,arm._gravity)
    with pytest.raises(ValueError,match='executor_calibration_mismatch'):
        arm.configure_profile(calibration(tmp_path),expected_sha256='0'*64)
    assert (arm.profile,arm.profile_sha256,arm._gravity)==previous


def test_arm_start_checks_actual_topic_and_joint_descriptor_before_start(tmp_path):
    from motion_control import MotionControl
    arm,_,_,_,_,_=rig(tmp_path,claim=False)
    card=MotionControl({},arm)
    card.solver=SimpleNamespace(profile=arm.profile,profile_sha256=arm.profile_sha256,
        lower=[p[0] for p in arm.limits],upper=[p[1] for p in arm.limits],velocity=arm.velocity)
    arm.motion_control=card
    starts=[];arm.start=lambda:starts.append(True)
    descriptor=card.control_interface('joint_position')
    assert arm.dispatch('start',{'input_topic':'/g1/motion/control/command'})['code']=='arm_input_topic_mismatch'
    assert arm.dispatch('start',{'control_interface':card.control_interface('eef_pose')})['code']=='invalid_control_descriptor'
    wrong={**descriptor,'dof':14}
    assert arm.dispatch('start',{'control_interfaces':{'joints':wrong}})['code']=='invalid_control_descriptor'
    assert not starts
    assert arm.dispatch('start',{'input_topic':'/g1/motion/arm/command',
        'control_interface':descriptor,'control_interfaces':{'joints':descriptor}})['state']=='ready'
    assert starts==[True]


def test_actual_rnea_is_ten_joint_nonzero_gravity_and_no_ik(tmp_path):
    pytest.importorskip('pinocchio')
    path=calibration(tmp_path);p=json.loads(path.read_text())
    model=GravityCompensation(p,p['urdf_path'])
    result=model([.2]*10)
    assert model.model.nq==10 and tuple(model.model.names)[1:]==JOINT_NAMES
    assert len(result)==10 and max(map(abs,result))>.1


def test_actual_sdk_stream_writes_only_ten_selected_joints_and_checks_receipt():
    from arm_sdk import ArmSdkChannel,WEIGHT_MOTOR_ID
    channel=ArmSdkChannel(grippers=False,waist=False,driven_arm_ids=MOTOR_IDS)
    message=SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.,dq=0.,tau=0.) for _ in range(35)],crc=0)
    written=[];publisher=SimpleNamespace(Write=lambda m:written.append(m) or True,Close=lambda:None)
    channel._arm_pub=publisher;channel._message=message
    assert channel.publish_stream([.2]*10,[.3]*10,weight=.4)
    assert [message.motor_cmd[i].tau for i in MOTOR_IDS]==[.3]*10
    assert all(message.motor_cmd[i].mode==1 for i in MOTOR_IDS)
    assert all((message.motor_cmd[i].kp,message.motor_cmd[i].kd)==((40.,1.5) if i in (19,26) else (80.,3.)) for i in MOTOR_IDS)
    assert message.motor_cmd[WEIGHT_MOTOR_ID].q==.4
    assert all(message.motor_cmd[i].q==0 for i in set(range(35))-set(MOTOR_IDS)-{WEIGHT_MOTOR_ID})
    publisher.Write=lambda _:False
    with pytest.raises(RuntimeError,match='write_failed'):channel.publish_stream([.4]*10,[.5]*10,weight=.4)
    assert channel._last_target==[.2]*10


def test_readonly_matched_count_migration_uses_subscription_event():
    source=DRIVER/'unitree_sdk2py/core/channel.py';tree=ast.parse(source.read_text())
    channel=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Channel')
    reader=next(n for n in channel.body if isinstance(n,ast.ClassDef) and n.name=='__Reader')
    # Compile the actual nested class, retaining its mangled names; no DDS import.
    ns={'DomainParticipant':object,'Topic':object,'Qos':object,'Callable':object,'DataReader':object}
    exec(compile(ast.Module(body=[reader],type_ignores=[]),str(source),'exec'),ns)
    obj=ns['__Reader']()
    assert obj.MatchedPublisherCount()==0
    obj._Reader__OnSubscriptionMatched(None,SimpleNamespace(current_count=2))
    assert obj.MatchedPublisherCount()==2


def test_readonly_publication_handle_preserves_unsigned_value_and_reports_error(monkeypatch):
    import ctypes
    source=DRIVER/'unitree_sdk2py/core/channel.py';tree=ast.parse(source.read_text())
    channel=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Channel')
    writer=next(n for n in channel.body if isinstance(n,ast.ClassDef) and n.name=='__Writer')
    dds=SimpleNamespace(instance_handle=ctypes.c_int64,publication_matched_status=object)
    ns={'DomainParticipant':object,'Topic':object,'Qos':object,'Any':object,'DataWriter':object,'dds_c_t':dds}
    monkeypatch.setitem(sys.modules,'cyclonedds.internal',SimpleNamespace(dds_c_t=dds))
    exec(compile(ast.Module(body=[writer],type_ignores=[]),str(source),'exec'),ns)
    obj=ns['__Writer']()
    def handle(ref,pointer):pointer._obj.value=-1;return 0
    obj._Writer__writer=SimpleNamespace(_ref=123,_get_instance_handle=handle)
    assert obj.PublicationHandle()==2**64-1
    obj._Writer__writer._get_instance_handle=lambda *args:-1
    with pytest.raises(RuntimeError,match='writer_handle_unavailable'):obj.PublicationHandle()


@pytest.mark.parametrize('records', [{}, {'acceptance': {}}, {'first_acceptance': {'operator': '', 'evidence_sha256': 'invalid'}}])
def test_prepare_uses_live_feedback_without_manual_acceptance_records(tmp_path, records):
    arm, sample, channel, advance, _, _ = rig(tmp_path, claim=False, servo=True)
    arm._prepared = False
    arm.profile.pop('acceptance', None)
    arm.profile.pop('first_acceptance', None)
    arm.profile.update(records)
    assert arm.dispatch('prepare_operator_session', {}) == {'state': 'ready', 'prepared': True}
    assert not channel.writes and arm.session_id is None
    arm._prepared = False
    advance(101, fresh=False)
    assert arm.dispatch('prepare_operator_session', {})['code'] == 'arm_feedback_stale'
    assert not arm._prepared and not channel.writes


def test_removed_paperwork_does_not_enable_disabled_live(tmp_path):
    arm, _, channel, _, _, _ = rig(tmp_path, claim=False, servo=True)
    arm._prepared = False
    arm.cfg['live_enabled'] = False
    assert arm.dispatch('prepare_operator_session', {})['code'] == 'live_disabled'
    assert not arm._prepared and not channel.writes


def test_other_dds_publishers_do_not_block_fresh_feedback(tmp_path):
    arm, sample, *_ = rig(tmp_path, claim=False)
    arm._snapshot_provider = None
    arm._feedback = sample
    arm._fsm = {'arm_ns': arm.clock(), 'fsm_id': 801}
    arm.profile['safety']['allowed_fsm_ids'] = [500, 801]
    class Observer:
        def MatchedPublisherCount(self):
            raise AssertionError('Publisher count must not gate teleoperation')
    arm._arm_observer = Observer()
    assert arm._fresh()[1] == sample['q']
    sample['fault'] = True
    assert arm._fresh()[1] == sample['q']


def test_raw_motor_telemetry_does_not_gate_arm_but_invalid_sample_does(tmp_path):
    arm, _, *_ = rig(tmp_path, claim=False)
    arm._snapshot_provider = None
    arm._fsm = {'arm_ns':arm.clock(), 'fsm_id':801}
    arm.profile['safety']['allowed_fsm_ids']=[500,801]
    low,high=arm.profile['safety']['voltage_range']
    motors=[SimpleNamespace(q=0.,dq=0.,mode=0,motorstate=0,
        vol=(low+high)/2,temperature=[20,20]) for _ in range(29)]
    for i in MOTOR_IDS: motors[i].mode=1
    motors[10].q=.02009
    motors[12].q=-.02193
    msg=SimpleNamespace(tick=1,motor_state=motors,wireless_remote=[0]*40,mode_machine=4)
    arm._on_state(msg)
    state,_,_=arm._fresh()
    assert 'fault' not in state
    assert state['motor_telemetry'][0]['motorstate']==0
    motors[15].motorstate=5;msg.tick=2;arm._on_state(msg)
    assert arm._fresh()[0]['motor_telemetry'][0]['motorstate']==5
    motors[15].mode=0;motors[15].vol=0;motors[15].temperature=[150,150]
    msg.tick=3;arm._on_state(msg)
    assert arm._fresh()[0]['motor_telemetry'][0]['temperature']==[150,150]
    msg.tick=4;msg.motor_state=[];arm._on_state(msg)
    with pytest.raises(ValueError,match='arm_feedback_invalid'): arm._fresh()


def test_idle_canvas_stop_does_not_manufacture_release_ownership(tmp_path):
    arm,_,ch,advance,_,sdk=rig(tmp_path,claim=False)
    arm._channel_open=False  # Real unclaimed executor has not opened an SDK writer.
    for _ in range(3):
        result=arm.dispatch('stop',{})
        assert result['no_op'] and result['authority_released']
        assert not result['physical_confirmed']
        advance(10);arm.tick()
        assert not arm.status()['ownership_held'] and arm._operation is None
    assert not ch.writes and not sdk


def test_duplicate_stop_while_awaiting_feedback_does_not_invent_vendor_action(tmp_path):
    arm,_,ch,advance,_,sdk=rig(tmp_path)
    arm.dispatch('stop',{})
    progress_release(arm,advance,'awaiting_feedback')
    ident=arm._operation['operation_id']
    for _ in range(3):
        result=arm.dispatch('stop',{})
        assert result['state']=='awaiting_feedback' and result['operation_id']==ident
    progress_release(arm,advance,'completed')
    assert not sdk and not arm.status()['ownership_held']


def test_idle_stop_does_not_clear_unknown_prior_release(tmp_path):
    arm,_,_,_,_,_=rig(tmp_path,claim=False);arm._channel_open=False
    arm._operation={'state':'unknown'}
    result=arm.dispatch('stop',{})
    assert result['code']=='release_result_unknown' and arm.status()['ownership_held']
