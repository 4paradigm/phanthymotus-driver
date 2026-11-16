"""Exercise private card control across the real parent/child capture boundary."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('bumi_control_capture_fixture',
    ROOT / 'tests/bumi_visual_odometry/test_bundle_hook.py')
_fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_fixture)
capture = _fixture.capture


def parent_control(device):
    parent = device.CameraPlugin.__new__(device.CameraPlugin)
    parent._namespace = 'robot'
    parent._body_imu_calibration = None
    parent._color_topic, parent._depth_topic = '/robot/camera/color', '/robot/camera/depth'
    parent._intrinsics = {}
    parent._rgbd_status = dict(ready=False, camera_info=[], error='awaiting calibrated capture')
    parent._stop_generations = dict(camera=0, depth=0, imu=0)
    parent._cards = {'camera', 'depth'}
    parent._proc = NS(poll=lambda: None)
    parent._lifecycle_lock = threading.RLock()
    parent._intrinsics_lock = threading.Lock()
    parent._control_condition = threading.Condition()
    parent._control_generation = 0
    parent._activations = dict(camera=0, depth=0, imu=0)
    parent._acked_generation = -1
    parent._control_packet = None
    return parent


def test_stop_waits_for_child_ack_after_inflight_depth_publish(capture, monkeypatch):
    device, state = capture
    parent = parent_control(device)
    pending = []
    sent = threading.Event()
    returned = threading.Event()
    timeline = []
    failures = []
    premature = []
    state.frame_limit = 3
    def send(message):
        pending[:] = [message]  # KEEP_LAST=1 transport
        sent.set()
    parent._cards_pub = NS(publish=send)
    def stop():
        try:
            parent.stop_card('depth')
            timeline.append('stop-returned')
            returned.set()
        except BaseException as error:
            failures.append(error)
    worker = threading.Thread(target=stop)
    def published(topic, message):
        if topic == '/robot/camera/depth':
            timeline.append('depth-publish')
            if not worker.ident:
                worker.start()
                assert sent.wait(1.)
                worker.join(.02)
                premature.append(returned.is_set())
        elif topic == '/robot/camera/cards_applied':
            if json.loads(message.data)['generation'] > 0:
                timeline.append('ack')
            parent._cards_applied(message)
    def spin(node):
        if pending:
            node.subscriptions['/robot/camera/cards_enabled'](pending.pop())
    state.on_publish, state.on_spin = published, spin
    try:
        device._camera_subprocess('robot')
    finally:
        if worker.ident:
            worker.join(1.)
    assert not failures
    assert premature == [False], 'stop returned while depth publication was still in flight'
    assert returned.is_set()
    assert timeline.index('depth-publish') < timeline.index('ack') < timeline.index('stop-returned')
    assert len(state.published['/robot/camera/depth']) == 1
    assert len(state.published['/robot/camera/color']) == 3
    assert parent._cards == {'camera'} and parent.is_running()


def test_parent_control_records_restart_even_when_transport_coalesces(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    parent = parent_control(device)
    packets = []
    parent._cards_pub = NS(publish=lambda message: packets.append(json.loads(message.data)))
    parent._publish_cards()
    parent._cards.discard('depth')
    parent._publish_cards()
    # Model a timed-out stop followed by restart before the child drains ROS.
    parent.start_card('depth')
    assert packets[0]['enabled'] == packets[-1]['enabled']
    assert packets[-1]['generation'] > packets[0]['generation']
    assert packets[-1]['activations']['depth'] > packets[0]['activations']['depth']



def test_body_imu_stop_ack_waits_for_publish_and_restart_rejects_queued_reads(capture, monkeypatch):
    from test_bumi_body_imu import MOUNT, reading
    device,state=capture
    monkeypatch.setenv('BUMI_BODY_IMU_MOUNT',json.dumps(MOUNT))
    state.frame_limit=3
    workers=[]
    blocked=[]
    def control(node,generation,enabled,activation):
        node.subscriptions['/robot/camera/cards_enabled'](NS(data=json.dumps(dict(
            generation=generation, enabled=enabled,
            activations=dict(camera=0,depth=0,imu=activation)))))
    def publish(topic,message):
        if topic=='/robot/state/imu_calibrated' and not workers:
            previous=len(state.published['/robot/camera/cards_applied'])
            worker=threading.Thread(target=lambda:control(state.node,1,['camera','depth'],0))
            workers.append(worker)
            worker.start()
            worker.join(.02)
            blocked.append(len(state.published['/robot/camera/cards_applied'])==previous)
    state.on_publish=publish
    def spin(node):
        if state.iterations==1:
            body=node.subscriptions['/robot/state/imu']
            body(NS(data=json.dumps(reading(10_030_000_000))))
            workers[0].join(1.)
            assert not workers[0].is_alive()
            assert json.loads(state.published['/robot/camera/cards_applied'][-1].data)['generation']==1
            body(NS(data=json.dumps(reading(10_035_000_000))))
            assert len(state.published['/robot/state/imu_calibrated'])==1
            control(node,2,['camera','depth','imu'],1)
            body(NS(data=json.dumps(reading(10_030_000_000))))  # queued before activation
            assert len(state.published['/robot/state/imu_calibrated'])==1
            body(NS(data=json.dumps(reading(10_040_000_000))))
            assert len(state.published['/robot/state/imu_calibrated'])==1  # cached SDK values
            body(NS(data=json.dumps(reading(10_045_000_000,angular_vel=[1.,2.,3.1]))))
            control(node,1,[],0)  # delayed control cannot retire this activation
            assert json.loads(state.published['/robot/camera/cards_applied'][-1].data)['generation']==2
    state.on_spin=spin
    device._camera_subprocess('robot')
    packets=[json.loads(m.data) for m in state.published['/robot/state/imu_calibrated']]
    assert blocked==[True]
    assert len(packets)==2 and packets[0]['epoch']!=packets[1]['epoch']
    assert packets[1]['stamp_ns']==10_045_000_000
    assert len(state.published['/robot/camera/color'])==3
    assert len(state.published['/robot/camera/depth'])==3


@pytest.mark.parametrize('dead', [False, True])
def test_unconfirmed_stop_reports_failure_and_keeps_sibling_owner(monkeypatch, dead):
    device = _fixture._load_device(monkeypatch)
    parent = parent_control(device)
    parent._proc = NS(poll=lambda: 1 if dead else None)
    parent._cards_pub = NS(publish=lambda message: None)
    wait = parent._wait_for_cards_applied
    parent._wait_for_cards_applied = lambda generation: wait(generation, timeout_s=.02)
    response = parent.dispatch('stop', {'_tool_name': 'depth'})
    assert response['state'] == 'error'
    assert 'confirmed' in response['error']
    assert parent._cards == {'camera'}
    assert parent._proc is not None  # A missing ACK cannot kill the sibling owner.
    info = parent.dispatch('info', {'_tool_name': 'depth'})
    assert info['state'] == ('idle' if dead else 'stopping')
    assert info['sourceRequestDisabled'] is True
    assert 'depth' not in parent._cards
    parent._cards_applied(NS(data=json.dumps(parent._control_packet)))
    assert parent.dispatch('info', {'_tool_name': 'depth'})['state'] == 'idle'


def test_old_ack_cannot_confirm_newer_stop(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    parent = parent_control(device)
    parent._cards_pub = NS(publish=lambda message: None)
    parent._publish_cards()
    old_packet = parent._control_packet
    parent._cards.remove('depth')
    parent._publish_cards()
    parent._cards_applied(NS(data=json.dumps(old_packet)))
    assert parent._acked_generation < parent._control_generation
    parent._cards_applied(NS(data=json.dumps(parent._control_packet)))
    parent._wait_for_cards_applied(parent._control_generation, timeout_s=.01)


def test_body_imu_stop_waits_for_its_inflight_json_publish(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    node = device._BumiStateNode.__new__(device._BumiStateNode)
    node._imu_enabled = node._running = True
    node.get_logger = lambda: NS(warn=lambda message: None)
    node._imu_lock = threading.RLock()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    state = device.StatePlugin.__new__(device.StatePlugin)
    state._namespace, state._node, state._camera_plugin = 'robot', node, None
    completed = threading.Event()
    premature = []
    responses = []
    def stop():
        responses.append(state.dispatch('stop', {'_tool_name': 'imu'}))
        completed.set()
    worker = threading.Thread(target=stop)
    def publish(message):
        worker.start()
        worker.join(.02)
        premature.append(completed.is_set())
    node._imu_pub = NS(publish=publish)
    def stop_polling():
        node._running = False
        raise RuntimeError('offline poll finished')
    node._high_ctrl = NS(get_imu_data=lambda: NS(ori=[0.,0.,0.,1.],
        angular_vel=[0.]*3, linear_acc=[0.]*3), get_joint_state=stop_polling)
    node._poll_loop()
    worker.join(1.)
    assert premature == [False]
    assert responses[0]['state'] == 'idle' and node._imu_enabled is False


def test_imu_card_does_not_report_idle_when_camera_stop_is_unconfirmed(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    state = device.StatePlugin.__new__(device.StatePlugin)
    state._namespace = 'robot'
    state._node = NS(_imu_enabled=True)
    state._node.set_imu_enabled = lambda enabled: setattr(state._node, '_imu_enabled', enabled)
    def unconfirmed(name):
        raise TimeoutError('camera card stop was not confirmed')
    state._camera_plugin = NS(stop_card=unconfirmed)
    response = state.dispatch('stop', {'_tool_name': 'imu'})
    assert response['state'] == 'error' and 'not confirmed' in response['error']


def test_coalesced_restart_flushes_sdk_blocking_video_queue(capture, monkeypatch):
    device, state = capture
    pipeline_type = sys.modules['pyrealsense2'].pipeline
    original_wait = pipeline_type.wait_for_frames
    stale = []
    flushed = []
    state.frame_limit = 2
    def wait(pipeline, timeout_ms):
        if stale:
            return stale.pop()
        frames = original_wait(pipeline, timeout_ms)
        if state.iterations == 1:
            stale.append(frames)
        return frames
    def poll(pipeline):
        if stale:
            flushed.append(stale[-1])
            return stale.pop()
        return None
    monkeypatch.setattr(pipeline_type, 'wait_for_frames', wait)
    monkeypatch.setattr(pipeline_type, 'poll_for_frames', poll)
    def spin(node):
        if state.iterations == 1:
            packet = dict(generation=2, enabled=['camera','depth','imu'],
                activations=dict(camera=0,depth=1,imu=0))
            node.subscriptions['/robot/camera/cards_enabled'](NS(data=json.dumps(packet)))
    state.on_spin = spin
    device._camera_subprocess('robot')
    assert len(flushed) == 1
    depth = state.published['/robot/camera/depth']
    assert [m.header.stamp.nanosec for m in depth] == [30_000_000,60_000_000]



def test_confirmed_depth_stop_stays_idle_while_another_card_start_is_unconfirmed(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    parent = parent_control(device)
    parent._cards_pub = NS(publish=parent._cards_applied)
    parent.stop_card('depth')
    assert parent.dispatch('info', {'_tool_name': 'depth'})['state'] == 'idle'
    parent._cards_pub = NS(publish=lambda message: None)
    parent.start_card('imu')
    assert parent._acked_generation < parent._control_generation
    info = parent.dispatch('info', {'_tool_name': 'depth'})
    assert info['state'] == 'idle' and info['sourceRequestDisabled'] is True


def test_imu_info_remains_stopping_until_camera_stop_is_confirmed_or_owner_exits(monkeypatch):
    device = _fixture._load_device(monkeypatch)
    parent = parent_control(device)
    parent._cards.add('imu')
    parent._activations['imu'] = 1
    parent._cards_pub = NS(publish=lambda message: None)
    wait = parent._wait_for_cards_applied
    parent._wait_for_cards_applied = lambda generation: wait(generation, timeout_s=.02)
    state = device.StatePlugin.__new__(device.StatePlugin)
    state._namespace, state._camera_plugin = 'robot', parent
    state._node = NS(_imu_enabled=True)
    state._node.set_imu_enabled = lambda enabled: setattr(state._node, '_imu_enabled', enabled)
    assert state.dispatch('stop', {'_tool_name': 'imu'})['state'] == 'error'
    assert state._node._imu_enabled is False and 'imu' not in parent._cards
    info = state.dispatch('info', {'_tool_name': 'imu'})
    assert info['state'] == 'stopping' and info['sourceRequestDisabled'] is True
    parent._proc = NS(poll=lambda: 1)
    assert state.dispatch('info', {'_tool_name': 'imu'})['state'] == 'idle'
    parent._proc = NS(poll=lambda: None)
    parent._cards_applied(NS(data=json.dumps(parent._control_packet)))
    assert state.dispatch('info', {'_tool_name': 'imu'})['state'] == 'idle'
