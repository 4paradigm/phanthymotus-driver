"""Real sequencing logic with explicit fictional authority and stop adapters."""
import copy
import threading
import time

import pytest

from deep_robotics.s10_pro.motion_backend import DDSMotionBackend
from deep_robotics.s10_pro.motion import MotionPlugin, validate_contract
from test_s10_action_contract import fixture_contract, Notifier


class Clock:
    now = 100.
    def __call__(self): return self.now


class Bridge:
    def __init__(self,clock):
        self.clock=clock; self.sequence=1; self.state=0; self.received=clock(); self.active={}
    def acquire(self,owner,sources,settings): self.active[owner]=sources
    def release(self,owner): self.active.pop(owner,None)
    def put(self,state): self.sequence+=1;self.state=state;self.received=self.clock()
    def snapshot(self,source):
        age=self.clock()-self.received
        return {'fresh':age<=.5,'age_s':age,'received_monotonic':self.received,'sequence':self.sequence,
                'data':{'data':{'motion_state':{'state':self.state},'gait_state':{'gait':0},
                               'vel_x':0.,'vel_y':0.,'vel_yaw':0.}}}


class Transport:
    def __init__(self): self.started=False;self.starts=[];self.states=[];self.velocities=[];self.closed=False;self.fail=False
    def status(self): return {'worker_running':self.started and not self.closed,'execution_enabled':True}
    def start(self,**kwargs):
        self.starts.append(kwargs)
        if self.fail: raise RuntimeError('fictional startup failure')
        self.started=True
    def close(self): self.closed=True
    def publish_state(self,state,**kwargs): self.states.append(state);return {'sent':True,'accepted':None}
    def publish_velocity(self,*axes,**kwargs): self.velocities.append(axes);return {'sent':True,'accepted':None}


class PostureStop:
    def __init__(self,clock): self.clock=clock;self.sends=[];self.sample={}
    def send(self,rid,action,reason,authority,cancel_event):
        self.sends.append((rid,action,reason,copy.deepcopy(authority)))
        return {'sent':True,'accepted':None,'confirmed':True} # This cannot confirm physical stop.
    def feedback(self,rid,action): return copy.deepcopy(self.sample)
    def put(self,rid,seq,**changes):
        self.sample={'request_id':rid,'fresh':True,'age_s':0.,'received_monotonic':self.clock(),
                     'sequence':seq,'confirmed':True,**changes}


@pytest.fixture
def subject():
    clock=Clock();bridge=Bridge(clock);transport=Transport();stop=PostureStop(clock)
    contract=fixture_contract('stand','lie','velocity')
    contract.update(robot_identity='FICTIONAL_ROBOT',firmware_identity='FICTIONAL_FIRMWARE')
    lease={'owned':True,'fresh':True,'age_s':0.,'received_monotonic':clock(),'expires_monotonic':clock()+.5,
           'allowed_actions':['stand','lie','velocity'],'regular_mode_confirmed':True,
           'navigation_mode_confirmed':False,'robot_identity':'FICTIONAL_ROBOT','firmware_identity':'FICTIONAL_FIRMWARE'}
    grant=[True]
    backend=DDSMotionBackend(bridge,transport,contract=contract,authority_provider=lambda:copy.deepcopy(lease),
        execution_authorized=True,authorization_provider=lambda:grant[0],posture_stop=stop,clock=clock)
    backend.start_observation()
    yield backend,bridge,transport,stop,clock,lease,grant
    backend.close()


def request(clock,action='stand'):
    return {'action':action,'request_id':'fiction-request','expires_monotonic':clock()+3}


def test_readiness_never_starts_worker_and_posture_does_not_require_navigation(subject):
    backend,bridge,transport,stop,clock,lease,_=subject
    assert backend.action_readiness('stand')['command_worker_startable']
    assert not backend.action_readiness('velocity')['command_worker_startable']
    backend.diagnostics();backend.readiness_status()
    assert not transport.starts and not transport.states
    backend.prepare_execution('stand')
    assert len(transport.starts)==1 and not transport.states
    backend.send('stand',request(clock),threading.Event())
    assert transport.states==[1] and not transport.velocities
    backend.progress('stand',request(clock),threading.Event())
    assert transport.states==[1]
    original = copy.deepcopy(backend._request)
    for _ in range(2):
        clock.now+=.01;bridge.put(1);backend.progress('stand',original,threading.Event())
    assert transport.states==[1,17]


def test_missing_posture_interruption_adapter_masks_only_posture_actions(subject):
    backend,_,transport,_,_,lease,_=subject
    backend._posture_stop=None
    assert not validate_contract(backend.contract(),'stand')
    assert validate_contract(backend.contract(),'velocity')
    with pytest.raises(PermissionError):backend.prepare_execution('stand')
    assert not transport.starts


@pytest.mark.parametrize('failure',['mode','scope','identity','expiry','grant'])
def test_invalid_action_authority_never_starts_worker(subject,failure):
    backend,_,transport,_,clock,lease,grant=subject
    if failure=='mode':lease['regular_mode_confirmed']=False
    elif failure=='scope':lease['allowed_actions']=['velocity']
    elif failure=='identity':lease['robot_identity']='OTHER'
    elif failure=='expiry':clock.now+=.6
    elif failure=='grant':grant[0]=False
    assert not backend.action_readiness('stand')['command_worker_startable']
    with pytest.raises(PermissionError):backend.prepare_execution('stand')
    assert not transport.starts and not transport.states


def test_worker_failure_is_not_retried_implicitly(subject):
    backend,_,transport,*_=subject
    transport.fail=True
    with pytest.raises(RuntimeError):backend.prepare_execution('stand')
    transport.fail=False
    with pytest.raises(RuntimeError):backend.prepare_execution('stand')
    assert len(transport.starts)==1 and transport.closed


def test_posture_cancel_requires_new_distinct_adapter_feedback_never_zero_velocity(subject):
    backend,bridge,transport,stop,clock,lease,_=subject
    backend.prepare_execution('stand');backend.send('stand',request(clock),threading.Event())
    result=backend.stop('user_cancel',threading.Event())
    assert not result['confirmed'] and len(stop.sends)==1 and not transport.velocities
    for _ in range(2):
        clock.now+=.01;bridge.put(0)
        assert not backend.feedback('stopmotion','fiction-request')['confirmed']
    stop.put('fiction-request',1)
    assert not backend.feedback('stopmotion','fiction-request')['confirmed']
    assert not backend.feedback('stopmotion','fiction-request')['confirmed']
    # A sequence counter alone cannot turn a single receipt into two observations.
    stop.put('fiction-request',2)
    assert not backend.feedback('stopmotion','fiction-request')['confirmed']
    clock.now+=.01;stop.put('fiction-request',3)
    assert not backend.feedback('stopmotion','fiction-request')['confirmed']
    clock.now+=.01;stop.put('fiction-request',4)
    assert backend.feedback('stopmotion','fiction-request')['confirmed']
    backend.stop('repeat_cancel',threading.Event())
    assert len(stop.sends)==1 and not transport.velocities
    backend.finish('fiction-request')
    assert not backend.feedback('stopmotion','fiction-request')['confirmed']


def test_stale_wrong_request_or_presend_posture_feedback_never_confirms(subject):
    backend,_,_,stop,clock,_,_=subject
    backend.prepare_execution('stand');backend.send('stand',request(clock),threading.Event())
    backend.stop('cancel',threading.Event())
    for seq,changes in enumerate([{'request_id':'other'},{'received_monotonic':99.9,'age_s':.1},
                                  {'fresh':False},{'age_s':.6},{'sequence':True}],1):
        stop.put('fiction-request',seq,**changes)
        assert not backend.feedback('stopmotion','fiction-request')['confirmed']


def test_navigation_mode_is_required_for_velocity_and_zero_stop(subject):
    backend,_,transport,_,clock,lease,_=subject
    with pytest.raises(PermissionError):backend.prepare_execution('velocity')
    lease['navigation_mode_confirmed']=True
    backend.prepare_execution('velocity')
    backend.send('velocity',request(clock,'velocity'),threading.Event())
    lease['navigation_mode_confirmed']=False
    with pytest.raises(PermissionError):backend.stop('cancel',threading.Event())
    assert len(transport.velocities)==1


def test_provider_cannot_revoke_authorization_mid_permit_and_still_send(subject):
    backend,_,transport,_,clock,lease,grant=subject
    backend.prepare_execution('stand')
    def revoke():
        grant[0]=False
        return copy.deepcopy(lease)
    backend._authority=revoke
    with pytest.raises(PermissionError):backend.send('stand',request(clock),threading.Event())
    assert not transport.states
