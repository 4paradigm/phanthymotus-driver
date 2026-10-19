"""No ROS/hardware needed: lifecycle ownership and transport diagnostics."""
import json
import sys
import threading

import pytest
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


def test_input_does_not_recreate_subscription(tmp_path):
    card, frame=input_card(tmp_path)
    bus,events,callbacks=bus_stub();bus.control=card
    bus._bind(dict(card.binding))
    callbacks[0](SimpleNamespace(data=json.dumps(frame)))
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


def test_duplicate_canvas_start_keeps_baseline(tmp_path):
    card,frame=input_card(tmp_path);card.receive(frame)
    card._thread=SimpleNamespace(is_alive=lambda:True)
    card.info=lambda:{'ok':True}
    card.mapping.anchor=object();anchor=card.mapping.anchor
    assert card.start({'input_topic':'/teleop/command'})=={'ok':True}
    assert card.mapping.anchor is anchor


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


def test_canvas_topic_is_used_without_source_path_convention(tmp_path):
    card, _ = input_card(tmp_path)
    card.motion.start = lambda: None
    card._thread = SimpleNamespace(is_alive=lambda: True)
    card.info = lambda: {'ok': True}
    assert card.start({'input_topic': '/canvas/custom_input'}) == {'ok': True}
    assert card.binding == {'command_topic': '/canvas/custom_input', 'feedback_topic': '/teleop/state'}
    bus, _, _ = bus_stub()
    subscribed = []
    bus.node.create_subscription = lambda _, topic, *args: subscribed.append(topic)
    bus._bind(card.binding)
    assert subscribed == ['/canvas/custom_input']


@pytest.mark.parametrize('input_hz,stall', [(0, 0.), (72, 0.), (72, .2)])
def test_feedback_publishes_at_thirty_hz_without_catchup(monkeypatch, input_hz, stall):
    bus, _, _ = bus_stub()
    now, next_input, spins, received = [0.], [1/input_hz if input_hz else float('inf')], [0], [0]
    published = []
    monkeypatch.setattr('teleop_bus.time', SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setitem(sys.modules, 'rclpy.impl.implementation_singleton',
        SimpleNamespace(rclpy_implementation=SimpleNamespace(InvalidHandle=InvalidHandle)))
    monkeypatch.setitem(sys.modules, 'rclpy.executors',
        SimpleNamespace(ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}),
                        ShutdownException=type('ShutdownException', (Exception,), {})))
    bus.closed = threading.Event()
    bus.String = SimpleNamespace
    bus.control = SimpleNamespace(_lock=threading.RLock(), binding={}, feedback=lambda: {'sequence': received[0]})
    bus._bind = lambda binding: None
    def spin(timeout_sec):
        spins[0] += 1
        now[0] = min(now[0]+timeout_sec, next_input[0])
        if now[0] >= next_input[0]:
            received[0] += 1; next_input[0] += 1/input_hz
        if spins[0] == 10 and stall:
            now[0] += stall
            next_input[0] = now[0]+1/input_hz
    def publish(message):
        published.append((now[0], json.loads(message.data)['sequence']))
        if len(published) == 31: bus.closed.set()
    bus.pub = SimpleNamespace(publish=publish)
    bus.executor = SimpleNamespace(context=SimpleNamespace(ok=lambda: True), spin_once=spin,
                                   remove_node=lambda node: None, shutdown=lambda: None)
    bus.node.destroy_node = lambda: None
    bus._run()
    intervals = [b[0]-a[0] for a,b in zip(published, published[1:])]
    assert min(intervals) >= 1/30-1e-9
    if stall: assert max(intervals) >= stall
    else: assert intervals == pytest.approx([1/30]*30)
    if input_hz: assert published[-1][1] >= 70
    assert not bus.health.status()['fatal']
