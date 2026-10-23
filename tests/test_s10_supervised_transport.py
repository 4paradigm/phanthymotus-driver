"""Explicit operator posture scope with fake ROS and private synthetic workers.

No ROS participant, robot endpoint, production permit or physical command is
created. Operator permits deliberately contain no vendor ownership claim.
"""
import threading
import time

import pytest

from deep_robotics.s10_pro.motion_dds import (
    AuthorityUnavailable, DDSCommandAdapter, MessageCodec, OPERATOR_PERMIT_SCHEMA,
)
from deep_robotics.s10_pro.motion_process import DDSProcessTransport, MotionProcessDisabled
from deep_robotics.s10_pro.motion_worker import dispatch_request, main as worker_main
from test_s10_motion_dds import CLASSES, Clock, Node, permit as vendor_permit
from test_s10_motion_process import Factory, request as rpc_request


def operator_permit(clock, action='stand', **changes):
    return {'schema': OPERATOR_PERMIT_SCHEMA, 'session_id': 'synthetic-session',
        'request_id': 'synthetic-'+action, 'action': action, 'robot_session': 'synthetic-robot-session',
        'issued_monotonic': clock.now, 'expires_monotonic': clock.now+60,
        'action_deadline_monotonic': clock.now+3, **changes}


@pytest.fixture
def supervised():
    clock,node,loaded=Clock(),Node(),[]
    def getter(name): loaded.append(name);return CLASSES[name]
    codec=MessageCodec(getter,clock_ns=lambda:123_000_000_000)
    adapter=DDSCommandAdapter(node,execution_enabled=True,supervised_posture=True,codec=codec,clock=clock)
    yield clock,node,loaded,codec,adapter
    adapter.close()


def test_constructor_and_metadata_preparation_never_create_publisher(supervised):
    clock,node,loaded,codec,adapter=supervised
    assert not loaded and not node.created
    codec.prepare(supervised_posture=True)
    assert 'drdds/msg/MotionState' in loaded and 'drdds/msg/NavCmd' not in loaded
    assert not node.created
    assert adapter.graph_snapshot()['state']['authority_proven'] is False
    assert not node.created


@pytest.mark.parametrize('action,state',[('stand',1),('lie',4)])
def test_single_explicit_state_publish_has_no_ownership_claim_or_automatic_rl(supervised,action,state):
    clock,node,_,_,adapter=supervised
    permit=operator_permit(clock,action)
    result=adapter.publish_state(state,authority=permit)
    assert result['sent'] is True and result['accepted'] is None
    assert result['confirmed'] is result['completed'] is False
    assert not any(key in result for key in ['owned','fresh','control_ownership','authority_valid'])
    assert len(node.created)==1 and node.created[0][1]=='/MOTION_STATE'
    assert [frame.data.state for frame in node.publishers[0].frames]==[state]
    with pytest.raises(AuthorityUnavailable):adapter.publish_state(state,authority=permit)
    assert len(node.publishers[0].frames)==1


@pytest.mark.parametrize('state',[0,4,17,True,1.0])
def test_stand_permit_never_authorizes_other_states(supervised,state):
    clock,node,_,_,adapter=supervised
    with pytest.raises(AuthorityUnavailable):adapter.publish_state(state,authority=operator_permit(clock))
    assert not node.created


@pytest.mark.parametrize('axes',[(0.,0.,0.),(.1,0.,0.)])
def test_supervised_never_authorizes_any_velocity_even_zero_or_forged_navigation(supervised,axes):
    clock,node,_,_,adapter=supervised
    for permit in [operator_permit(clock),{**operator_permit(clock),'navigation_mode_confirmed':True},vendor_permit(clock)]:
        with pytest.raises(AuthorityUnavailable):adapter.publish_velocity(*axes,authority=permit)
        with pytest.raises(AuthorityUnavailable):adapter.publish_zero_velocity(authority=permit)
    assert not node.created


def test_normal_and_supervised_permits_cannot_cross_modes(supervised):
    clock,node,_,codec,adapter=supervised
    normal_node=Node();normal=DDSCommandAdapter(normal_node,execution_enabled=True,codec=codec,clock=clock)
    try:
        with pytest.raises(AuthorityUnavailable):adapter.publish_state(1,authority=vendor_permit(clock))
        with pytest.raises(AuthorityUnavailable):normal.publish_state(1,authority=operator_permit(clock))
        forged={**operator_permit(clock),**vendor_permit(clock)}
        with pytest.raises(AuthorityUnavailable):normal.publish_state(1,authority=forged)
        with pytest.raises(AuthorityUnavailable):adapter.publish_state(1,authority=forged)
        assert not node.created and not normal_node.created
        assert normal.publish_state(17,authority=vendor_permit(clock))['sent']
    finally:normal.close()


@pytest.mark.parametrize('changes',[
    {'schema':'other'}, {'session_id':''},{'session_id':' '*3},{'request_id':'x'*129},
    {'robot_session':None},{'action':'velocity'},{'owned':True},{'fresh':True},
    {'issued_monotonic':100.1},{'issued_monotonic':True},{'issued_monotonic':float('nan')},
    {'issued_monotonic':-1},{'expires_monotonic':100.},{'expires_monotonic':3700.1},
    {'expires_monotonic':float('inf')},{'action_deadline_monotonic':100.},
    {'action_deadline_monotonic':161.},{'action_deadline_monotonic':False},
])
def test_invalid_operator_scope_or_clock_never_creates_publisher(supervised,changes):
    clock,node,_,_,adapter=supervised
    with pytest.raises(AuthorityUnavailable):adapter.publish_state(1,authority=operator_permit(clock,**changes))
    assert not node.created


def test_same_operator_session_cannot_renew_original_receipt_or_change_robot(supervised):
    clock,node,_,_,adapter=supervised
    permit=operator_permit(clock)
    adapter.publish_state(1,authority=permit)
    clock.now+=.1
    for changes in [{'issued_monotonic':clock.now},{'expires_monotonic':161.}, {'robot_session':'different'}]:
        altered={**permit,'request_id':'second','action_deadline_monotonic':clock.now+2,**changes}
        with pytest.raises(AuthorityUnavailable):adapter.publish_state(1,authority=altered)
    assert len(node.publishers[0].frames)==1
    unchanged={**permit,'request_id':'new-lie','action':'lie','action_deadline_monotonic':clock.now+2}
    assert adapter.publish_state(4,authority=unchanged)['sent']
    assert [m.data.state for m in node.publishers[0].frames]==[1,4]


def test_expiry_is_rechecked_after_type_load_and_before_publish(supervised):
    clock,node,_,codec,adapter=supervised
    original=codec._getter
    def late_getter(name): clock.now=104.;return original(name)
    codec._getter=late_getter
    with pytest.raises(AuthorityUnavailable):adapter.publish_state(1,authority=operator_permit(clock))
    assert not node.created


def test_cancelled_explicit_send_creates_no_publisher(supervised):
    clock,node,_,_,adapter=supervised;event=threading.Event();event.set()
    with pytest.raises(InterruptedError):adapter.publish_state(1,authority=operator_permit(clock),cancel_event=event)
    assert not node.created


def test_worker_dispatch_preserves_short_rpc_deadline_for_operator(supervised):
    clock,node,_,_,adapter=supervised;permit=operator_permit(clock)
    req=rpc_request('state',{'state':1},permit,deadline=100.1)
    assert dispatch_request(adapter,req,clock=clock)['sent']
    req=rpc_request('state',{'state':1},{**permit,'request_id':'other'},deadline=100.201)
    with pytest.raises(TimeoutError):dispatch_request(adapter,req,clock=clock)
    assert len(node.publishers[0].frames)==1


def test_supervised_process_requires_explicit_enabled_start_and_exclusive_flag():
    factory=Factory();client=DDSProcessTransport({'reader':{'domain_id':0}},supervised_posture=True,process_factory=factory)
    try:
        assert not factory.calls and not client.status()['worker_running']
        with pytest.raises(MotionProcessDisabled):client.start()
        with pytest.raises(ValueError):client.start(probe_only=True)
        assert not factory.calls
    finally:client.close()
    factory=Factory();client=DDSProcessTransport({'reader':{'domain_id':0}},execution_enabled=True,
        supervised_posture=True,process_factory=factory)
    try:
        assert not factory.calls
        client.start()
        args,options=factory.calls[0]
        assert '--allow-supervised-posture' in args and '--allow-execution' not in args
        assert options['env']['ROS_DOMAIN_ID']=='0'
        clock=Clock();clock.now=time.monotonic();permit=operator_permit(clock)
        assert client.publish_state(1,authority=permit)['sent']
        with pytest.raises(AuthorityUnavailable):client.publish_state(17,authority=permit)
        with pytest.raises(AuthorityUnavailable):client.publish_velocity(authority=permit)
        with pytest.raises(AuthorityUnavailable):client.publish_state(1,authority=vendor_permit(clock))
    finally:client.close()


def test_normal_process_rejects_operator_permit_without_starting_worker():
    factory=Factory();client=DDSProcessTransport({'reader':{'domain_id':0}},execution_enabled=True,process_factory=factory)
    try:
        clock=Clock();clock.now=time.monotonic()
        with pytest.raises(AuthorityUnavailable):client.publish_state(1,authority=operator_permit(clock))
        with pytest.raises(AuthorityUnavailable):client.publish_velocity(authority=operator_permit(clock))
        assert not factory.calls
    finally:client.close()


def test_supervised_process_cannot_accept_buffered_reply_after_rpc_deadline(monkeypatch):
    clock=Clock();factory=Factory()
    client=DDSProcessTransport({'reader':{'domain_id':0}},execution_enabled=True,
        supervised_posture=True,process_factory=factory,clock=clock)
    try:
        client.start()
        def late_response(*args,**kwargs):
            clock.now+=.21
            return {'id':1,'ok':True,'result':{'sent':True}}
        monkeypatch.setattr(client,'_read',late_response)
        with pytest.raises(TimeoutError):client.publish_state(1,authority=operator_permit(clock))
        assert client.status()['closed'] and not client.status()['worker_running']
    finally:client.close()


def test_worker_execution_modes_are_mutually_exclusive_before_ros_import():
    with pytest.raises(SystemExit):worker_main(['--allow-execution','--allow-supervised-posture'])
