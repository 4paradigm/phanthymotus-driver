"""No ROS/hardware needed: lifecycle ownership and transport diagnostics."""
import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_spin import SpinHealth, supervised_spin
from teleop_bus import TeleopBus
from test_teleop_settings import input_card


class InvalidHandle(Exception):
    pass


def test_shared_spin_survives_destroyed_handle_but_reports_unknown_failure():
    now = [0.]
    health = SpinHealth(clock=lambda:now[0])
    actions = [InvalidHandle('destroyed'), None, RuntimeError('unrelated')]
    observed=[]; sleeps=[]
    def spin(**kw):
        action=actions.pop(0)
        if action: raise action
        observed.append('received_after_exception')
    supervised_spin(SimpleNamespace(spin_once=spin), lambda:True, health,
                    InvalidHandle, sleep=sleeps.append)
    assert observed == ['received_after_exception'] and sleeps == [.05]
    status=health.status()
    assert status['invalid_handle_count']==1 and status['fatal'] and not status['running']
    assert 'unrelated' in status['last_error']


def test_heartbeat_expires_even_when_thread_has_not_exited():
    now=[0.];health=SpinHealth(clock=lambda:now[0]);health.beat()
    assert health.status()['healthy']
    now[0]=1.1
    assert health.status()['running'] and not health.status()['healthy']


def bus_stub():
    bus=TeleopBus.__new__(TeleopBus)
    bus.binding=bus.sub=bus.pub=None
    bus.subscription_generation=0
    bus.String=object;bus.qos=object()
    events=[]; callbacks=[]
    def sub(*args):
        events.append('create_sub');callbacks.append(args[2]);return object()
    bus.node=SimpleNamespace(create_publisher=lambda *a:object(), create_subscription=sub,
                             destroy_subscription=lambda *a:events.append('destroy_sub'),
                             destroy_publisher=lambda *a:None)
    bus._status_lock=threading.Lock()
    bus.health=SpinHealth();bus.shared_executor=object()
    bus.receive_error=bus.publish_error=None
    bus.last_received_sequence=bus.last_accepted_sequence=None
    bus.last_received_ns=bus.last_accepted_ns=None
    bus.rejected_count=0
    return bus,events,callbacks


def test_first_source_identity_does_not_recreate_subscription(tmp_path):
    card, frame=input_card(tmp_path)
    bus,events,callbacks=bus_stub();bus.control=card
    bus._bind(dict(card.binding))
    callbacks[0](SimpleNamespace(data=json.dumps(frame)))
    assert card.binding['instance_id']=='card_random'
    bus._bind(dict(card.binding))
    assert events==['create_sub'] and bus.subscription_generation==1
    assert bus.status()['last_accepted_sequence']==1
    bus._bind(dict(card.binding,command_topic='/another/command'))
    callbacks[0](SimpleNamespace(data=json.dumps(dict(frame,sequence=2))))
    assert card._input_sequence==1  # The retired callback must not admit input.
    assert events==['create_sub','destroy_sub','create_sub']


def test_receive_error_survives_successful_monitor_publication(tmp_path):
    card,frame=input_card(tmp_path);bus,_,_=bus_stub();bus.control=card
    bus.receive(SimpleNamespace(data='not json'))
    error=bus.receive_error
    bus.publish_error=None
    assert error and bus.status()['receive_error']==error
    bus.receive(SimpleNamespace(data=json.dumps(frame)))
    assert bus.receive_error is None and bus.rejected_count==1


def test_restart_reports_binding_missing_not_stale(tmp_path):
    card,_=input_card(tmp_path);card.binding=None
    status=card.info()
    assert status['input_status']['state']=='waiting_binding'
    assert '无需重新连线' in status['input_status']['hint']
    assert card._operator is None


def test_duplicate_canvas_start_keeps_pinned_identity_and_baseline(tmp_path):
    card,frame=input_card(tmp_path);card.receive(frame)
    card._thread=SimpleNamespace(is_alive=lambda:True)
    card.info=lambda:{'ok':True}
    card.mapping.anchor=object();anchor=card.mapping.anchor
    assert card.start({'input_topic':'/teleop/command'})=={'ok':True}
    assert card.mapping.anchor is anchor and card.binding['instance_id']=='card_random'


def test_stop_without_authority_is_idempotent_and_does_not_call_sdk(tmp_path):
    card,_=input_card(tmp_path)
    card.motion.cancel_pending=lambda:None;card.motion._lock=threading.Lock()
    card.executor.cancel_preparation=lambda:None
    card.executor.opening=False
    card.executor.status=lambda:{'ownership_held':False}
    card.executor.dispatch=lambda *a: (_ for _ in ()).throw(AssertionError('must not call hardware'))
    assert card.stop()['authority_released']
    assert card.stop()['authority_released']


def test_old_topic_packet_rejected_before_owner_rebinds(tmp_path):
    card,frame=input_card(tmp_path);bus,_,callbacks=bus_stub();bus.control=card
    bus._bind(dict(card.binding))
    card.binding=dict(card.binding,command_topic='/new/topic')
    callbacks[0](SimpleNamespace(data=json.dumps(frame)))
    assert card._input_sequence==-1 and bus.last_accepted_sequence is None
