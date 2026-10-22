"""Management reply loss/reordering, with fake time and no ROS or hardware."""
import secrets
from types import SimpleNamespace
import pytest
from test_motion_stream import rig
from teleop_executor import TeleopExecutor


def executor():
    gate,state,writes,advance,_=rig()
    e=TeleopExecutor({},'isolated',None,SimpleNamespace(_pos_publisher=True),
                     SimpleNamespace(_left_pub=True,_right_pub=True),[])
    e.gate=gate;e.start=lambda:None;e.foreign_publishers=lambda:[];e.profile={'hands_enabled':False}
    gate.hold();gate.tick();advance(20);gate.tick()
    args={'request_id':secrets.token_hex(16),'request_valid_until_ns':gate.clock()+300_000_000,
          'session_id':gate.session_id,'secret':gate.secret}
    return e,writes,advance,args


def test_lost_resume_reply_retries_same_result_without_rotating_again():
    e,writes,advance,args=executor()
    first=e.dispatch('resume',args);owner=e.gate.session_id;count=len(writes)
    advance(120);e.gate.tick()
    assert e.gate.state=='ready' and len(writes)==count
    assert e.dispatch('resume',args)==first and e.gate.session_id==owner
    assert e.info().get('request_id') is None
    advance(181)
    assert e.dispatch('resume',args)['code']=='management_request_expired'
    e.gate.tick();advance(20);e.gate.tick()
    assert e.gate.session_id is None and e.gate.status()['stop_confirmed']


@pytest.mark.parametrize('delivered',[False,True])
def test_cancel_with_original_credentials_fences_delayed_resume(delivered):
    e,writes,advance,args=executor()
    if delivered:assert not e.dispatch('resume',args).get('error')
    assert not e.dispatch('release',args).get('error')
    e.gate.tick();advance(20);e.gate.tick()
    assert e.gate.session_id is None
    count=len(writes)
    assert e.dispatch('resume',args)['code']=='management_cancelled'
    assert len(writes)==count and e.gate.session_id is None


def test_request_cannot_release_a_later_owner():
    e,writes,advance,args=executor()
    e.dispatch('resume',args)
    e.gate.hold(release=True);e.gate.tick();advance(20);e.gate.tick()
    other=e.gate.claim();count=len(writes)
    assert e.dispatch('resume',args)['code']=='management_lease_released'
    assert e.dispatch('release',args)['code']=='management_owner_changed'
    assert e.gate.session_id==other['session_id'] and len(writes)==count


@pytest.mark.parametrize('change',[{'secret':'wrong'},{'session_id':'wrong'},
                                  {'request_valid_until_ns':9_000_000_000}])
def test_receipt_does_not_disclose_rotated_lease_to_mismatched_request(change):
    e,_,_,args=executor();e.dispatch('resume',args)
    result=e.dispatch('resume',{**args,**change})
    assert result['code']=='invalid_management_request' and 'secret' not in result


def test_lost_claim_reply_is_idempotent_and_pause_cancels_retry():
    e,writes,advance,_=executor()
    e.gate.hold(release=True);e.gate.tick();advance(20);e.gate.tick()
    args={'request_id':secrets.token_hex(16),'request_valid_until_ns':e.gate.clock()+300_000_000}
    first=e.dispatch('claim',args);count=len(writes)
    assert e.dispatch('claim',args)==first and len(writes)==count
    assert not e.dispatch('pause',first).get('error')
    assert e.dispatch('claim',args)['code']=='management_cancelled'


def test_delayed_earlier_claim_cannot_reacquire_after_resume_and_release():
    e,writes,advance,_=executor()
    e.gate.hold(release=True);e.gate.tick();advance(20);e.gate.tick()
    claim={'request_id':secrets.token_hex(16),'request_valid_until_ns':e.gate.clock()+300_000_000}
    first=e.dispatch('claim',claim)
    e.dispatch('pause',first);e.gate.tick();advance(20);e.gate.tick()
    resume={**first,'request_id':secrets.token_hex(16),'request_valid_until_ns':e.gate.clock()+300_000_000}
    second=e.dispatch('resume',resume)
    e.dispatch('release',second);e.gate.tick();advance(20);e.gate.tick()
    count=len(writes)
    assert e.dispatch('claim',claim)['code']=='management_cancelled'
    assert e.gate.session_id is None and len(writes)==count


def test_recoverable_hold_mcp_requires_current_owner_and_cannot_override_pause():
    e,writes,advance,args=executor()
    lease=e.gate.resume()
    assert 'recoverable_hold' in e.get_tool()['inputSchema']['properties']['action']['enum']
    count=len(writes)
    assert e.dispatch('recoverable_hold',{**lease,'secret':'wrong'})['code']=='invalid_lease'
    assert len(writes)==count and e.gate.state=='ready'
    assert not e.dispatch('recoverable_hold',lease).get('error')
    e.gate.tick();advance(20);e.gate.tick()
    assert e.gate.status()['continuation_ready']
    assert not e.dispatch('pause',lease).get('error')
    assert e.dispatch('recoverable_hold',lease)['code']=='hold_not_resumable'
