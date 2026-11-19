"""Exercise the public G1 card contract and real output gates without hardware."""
import ast
import json
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace as NS
import zlib

import numpy as np
import pytest

from test_g1_rgbd_camera import configured, Frames

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_class(name, scope):
    tree = ast.parse((ROOT / 'device.py').read_text())
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == name)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(ROOT / 'device.py'), 'exec'), scope)
    return scope[name]


def plugin():
    cache = lambda topic: NS(destroy_node=lambda: None)
    scope = dict(threading=threading, queue=queue, time=time,
                 _CameraFrameNode=cache, RS_COLOR_W=1920, RS_COLOR_H=1080,
                 RS_COLOR_FPS=15, RS_DEPTH_W=640, RS_DEPTH_H=480, RS_DEPTH_FPS=15)
    cls = load_class('RealSensePlugin', scope)
    obj = cls({}, 'g1', NS(add_node=lambda node: None, remove_node=lambda node: None))
    obj._proc = NS(is_alive=lambda: True)
    obj._rgbd_status = dict(state='unavailable', reason='test capture process disabled', configuration_error='test capture process disabled', camera_info=[])
    obj.start = lambda: None  # Only the physical process creation is substituted.
    return obj


def test_original_cards_share_one_public_depth_and_real_paired_declarations():
    obj = plugin()
    source, _, _ = configured()
    source.depth_topic = '/g1/camera/depth'
    source.encode(Frames(), 1700000000130000000)
    status = source.info()
    status['last_capture_ns'] = time.time_ns()
    obj._rgbd_status = status
    tools = {tool['name']: tool for tool in obj.get_tools()}
    assert set(tools) == {'camera_rgb', 'camera_depth', 'camera_distance'}
    assert tools['camera_depth']['topic_out'] == [
        {'topic': '/g1/camera/depth', 'format': 'image/depth-zlib'}]
    assert tools['camera_rgb']['topic_out'][0] == {'topic': '/g1/camera/rgb', 'format': 'image/jpeg'}
    assert tools['camera_rgb']['topic_out'][1]['topic'] == '/g1/camera/rgbd/color'
    depth = tools['camera_depth']['camera_info'][0]
    color = tools['camera_rgb']['camera_info'][1]
    assert depth['topic'] == '/g1/camera/depth'
    assert depth['K'] == color['K']
    assert depth['id'] == color['id']
    assert depth['vendor']['clock_id'] == color['vendor']['clock_id']
    assert depth['vendor']['frame_id'] == color['vendor']['frame_id']


def test_individual_card_lifecycle_is_idempotent_and_keeps_other_cards_running():
    obj = plugin()
    obj.dispatch('start', {'_tool_name': 'camera_rgb'})
    obj.dispatch('start', {'_tool_name': 'camera_depth'})
    epoch = list(obj._card_epochs)
    obj.dispatch('start', {'_tool_name': 'camera_depth'})
    assert list(obj._card_epochs) == epoch
    obj.dispatch('stop', {'_tool_name': 'camera_depth'})
    assert obj.is_running()
    assert obj._card_epochs[0] == epoch[0] and obj._card_epochs[1] == 0
    assert obj.dispatch('info', {'_tool_name': 'camera_depth'})['state'] == 'idle'
    obj.dispatch('start', {'_tool_name': 'camera_depth'})
    assert obj._card_epochs[1] > epoch[1]


def test_body_imu_card_preserves_body_json_and_adds_named_camera_motion():
    obj = plugin()
    state, _, _, _ = body_imu_card(obj)
    tool = state._imu_tool()
    assert tool['topic_out'][0] == {'topic': '/g1/state/imu', 'format': 'data/json'}
    assert tool['topic_out'][1] == {'topic': '/g1/camera/rgbd/imu', 'format': 'sensor/imu',
                                    'name': 'camera_imu', 'optional': True}
    state.dispatch('start', {'_tool_name': 'imu'})
    assert obj._card_epochs[3] > 0 and obj._card_epochs[0] == 0
    info = state.dispatch('info', {'_tool_name': 'imu'})
    assert info['topic_out'] == tool['topic_out'] and 'imu_status' in info
    state.dispatch('stop', {'_tool_name': 'imu'})
    assert obj._card_epochs[3] == 0 and obj.is_running()


def owner(gates):
    class Node:
        def __init__(self, *args): pass
    class Message:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
    scope = dict(_Node=Node, queue=queue, threading=threading, time=time, json=json,
                 CompressedImage=Message, _String=Message, card_epochs=gates)
    cls = load_class('_RealSenseNode', scope)
    obj = cls.__new__(cls)
    return obj, Message


def test_normalized_encoder_is_the_only_depth_publisher_and_stopped_cards_are_silent():
    gates = mp.get_context('spawn').Array('q', [1, 1, 0, 0])
    obj, _ = owner(gates)
    source, _, _ = configured()
    output = [[], []]
    obj._rgbd = source
    obj._rgbd_pubs = [NS(publish=output[0].append), NS(publish=output[1].append)]
    obj._rgbd_q = queue.Queue(maxsize=1)
    obj._rgbd_q.put((Frames(), 1700000000130000000))
    running = iter([False, True])
    obj._worker_stop = NS(is_set=lambda: next(running))
    obj._publish_rgbd_status = lambda: None
    # Fixture capture is old in wall time; preserve the real encoder's clock gate.
    clock = obj._rgbd_loop.__globals__
    clock['time'] = NS(time_ns=lambda: 1700000000130000000)
    obj._rgbd_loop()
    assert len(output[1]) == 1
    msg = output[1][0]
    assert np.all(np.frombuffer(zlib.decompress(msg.data), dtype='<u2') == 1000)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1700000000, 128000000)
    assert msg.header.frame_id == 'camera_depth_optical_frame'
    with gates.get_lock(): gates[1] = 0
    assert not obj._publish_for('camera_depth', NS(), 1700000000158000000, obj._rgbd_pubs[1])
    assert len(output[1]) == 1
    assert obj._publish_for('camera_rgb', NS(), 1700000000153000000, obj._rgbd_pubs[0])
    # A frame captured before a restart must not be revived by an active sibling.
    with gates.get_lock(): gates[1] = 1700000000200000000
    assert not obj._publish_for('camera_depth', NS(), 1700000000158000000, obj._rgbd_pubs[1])
    assert obj._publish_for('camera_depth', NS(), 1700000000218000000, obj._rgbd_pubs[1])


def test_invalid_shared_depth_never_falls_back_to_raw_z16():
    gates = mp.get_context('spawn').Array('q', [1, 1, 0, 0])
    obj, _ = owner(gates)
    source, _, _ = configured()
    source.geometry = None
    output = []
    obj._rgbd = source
    obj._rgbd_pubs = [NS(publish=output.append), NS(publish=output.append)]
    obj._rgbd_q = queue.Queue(maxsize=1)
    obj._rgbd_q.put((Frames(), 1700000000130000000))
    running = iter([False, True])
    obj._worker_stop = NS(is_set=lambda: next(running))
    obj._publish_rgbd_status = lambda: None
    obj._rgbd_loop()
    assert output == [] and source.info()['camera_info'] == []
    assert not hasattr(type(obj), '_depth_loop')


def test_pre_restart_capture_cannot_escape_with_a_future_sdk_stamp():
    gates = mp.get_context('spawn').Array('q', [100, 200, 0, 0])
    obj, _ = owner(gates)
    output = []
    pub = NS(publish=output.append)
    assert not obj._publish_for('camera_depth', NS(), 250, pub, acquired_ns=190)
    assert output == []
    assert obj._publish_for('camera_depth', NS(), 250, pub, acquired_ns=210)


def test_camera_imu_restart_does_not_reuse_an_earlier_accel_sample():
    gates = mp.get_context('spawn').Array('q', [0, 0, 0, 200])
    obj, _ = owner(gates)
    output = []
    obj._imu_pub = NS(publish=output.append)
    obj._publish_imu(dict(stamp_ns=210, accel_stamp_ns=190))
    assert output == []
    obj._publish_imu(dict(stamp_ns=220, accel_stamp_ns=205))
    assert len(output) == 1


def test_stopped_shared_cards_do_not_emit_body_mount_sidecars():
    from test_g1_rgbd_camera import Mount
    gates = mp.get_context('spawn').Array('q', [0, 0, 0, 0])
    obj, _ = owner(gates)
    source, _, _ = configured()
    source.body_mount = Mount()
    obj._rgbd = source
    output = []
    obj._rgbd_pubs = [NS(publish=output.append), NS(publish=output.append)]
    obj._rgbd_q = queue.Queue(maxsize=1)
    obj._rgbd_q.put((Frames(), 1700000000130000000))
    running = iter([False, True])
    obj._worker_stop = NS(is_set=lambda: next(running))
    obj._publish_rgbd_status = lambda: None
    obj._rgbd_loop.__globals__['time'] = NS(time_ns=lambda: 1700000000130000000)
    obj._rgbd_loop()
    assert output == [] and source.body_mount.calls == []


def test_capture_owner_start_does_not_implicitly_activate_sibling_cards(monkeypatch):
    obj = plugin()
    obj._proc = None
    processes = []
    class Process:
        def __init__(self, **kwargs):
            processes.append(kwargs)
            self.alive = False
            self.pid = 42
        def start(self):
            self.alive = True
            obj._rgbd_status = dict(state='unavailable', reason='test capture process disabled', configuration_error='test capture process disabled', camera_info=[])
        def is_alive(self): return self.alive
        def terminate(self): self.alive = False
        def join(self, **kwargs): pass
    class Queue(queue.Queue):
        def close(self): pass
    channel = NS(close=lambda: None)
    ctx = NS(Pipe=lambda **kwargs: (channel, channel), Queue=lambda **kwargs: Queue(), Process=Process)
    monkeypatch.setattr(mp, 'get_context', lambda mode: ctx)
    scope = type(obj).start.__globals__
    monkeypatch.setitem(scope, 'threading', NS(Thread=lambda **kwargs: NS(start=lambda: None, is_alive=lambda: False)))
    monkeypatch.setitem(scope, 'run_realsense_process', object())
    del obj.start  # Exercise the real lifecycle, substituting the SDK process boundary.
    obj.start()
    assert list(obj._card_epochs) == [0, 0, 0, 0]
    obj.dispatch('start', {'_tool_name': 'camera_depth'})
    obj.dispatch('start', {'_tool_name': 'camera_depth'})
    assert len(processes) == 1
    assert obj._card_epochs[1] > 0 and obj._card_epochs[0] == obj._card_epochs[2] == obj._card_epochs[3] == 0
    obj.stop()
    assert list(obj._card_epochs) == [0, 0, 0, 0] and not obj.is_running()
    # A worker that died outside bundle shutdown must not keep old card epochs.
    obj._card_epochs[1] = 1
    obj.start()
    assert obj._card_epochs[1] > 1
    obj.stop()


def test_vision_capture_cache_does_not_reuse_frames_from_a_stopped_rgb_card():
    cls = load_class('_CameraFrameNode', dict(Node=object, threading=threading, time=time))
    cache = cls.__new__(cls)
    cache._condition = threading.Condition()
    cache._sequence, cache._latest = 0, None
    obj = plugin()
    obj._frame_node = cache
    def frame(stamp):
        return NS(data=b'\xff\xd8old\xff\xd9', format='jpeg',
                  header=NS(stamp=NS(sec=0, nanosec=stamp)))
    cache._on_frame(frame(90))
    assert obj.wait_for_color_frame(timeout_s=0)[0] is None
    obj._card_epochs[0] = 100
    assert obj.wait_for_color_frame(timeout_s=0)[0] is None
    cache._on_frame(frame(110))
    assert obj.wait_for_color_frame(timeout_s=0)[0]['data'] == b'\xff\xd8old\xff\xd9'
    obj._card_epochs[0] = 0
    assert obj.wait_for_color_frame(timeout_s=0)[0] is None


def test_concurrent_card_starts_cannot_create_two_realsense_owners(monkeypatch):
    obj = plugin()
    obj._proc = None
    processes = []
    class Process:
        def __init__(self, **kwargs):
            processes.append(self)
            self.alive, self.pid = False, 42
        def start(self):
            time.sleep(.03)  # Expose the spawn window to a second HTTP handler.
            self.alive = True
            obj._rgbd_status = dict(state='unavailable', reason='test capture process disabled', configuration_error='test capture process disabled', camera_info=[])
        def is_alive(self): return self.alive
        def terminate(self): self.alive = False
        def join(self, **kwargs): pass
    class Queue(queue.Queue):
        def close(self): pass
    channel = NS(poll=lambda timeout: True, recv=lambda: {}, close=lambda: None)
    ctx = NS(Pipe=lambda **kwargs: (channel, channel), Queue=lambda **kwargs: Queue(), Process=Process)
    monkeypatch.setattr(mp, 'get_context', lambda mode: ctx)
    monkeypatch.setitem(type(obj).start.__globals__, 'run_realsense_process', object())
    del obj.start
    workers = [threading.Thread(target=obj.dispatch, args=('start', {'_tool_name': name}))
               for name in ('camera_rgb', 'camera_depth')]
    for worker in workers: worker.start()
    for worker in workers: worker.join(timeout=2)
    try:
        assert len(processes) == 1
        assert obj._card_epochs[0] and obj._card_epochs[1]
    finally:
        obj.stop()


def test_camera_motion_restart_renews_source_epoch_and_requires_new_accel():
    from common.realsense_imu import MotionSource
    from test_realsense_camera_imu import sdk, profile, device, motion
    gates = mp.get_context('spawn').Array('q', [0, 0, 0, 1])
    obj, _ = owner(gates)
    profiles = [profile('gyro'), profile('accel')]
    active = NS(get_stream=lambda kind: next((p for p in profiles if p.stream_type() == kind), NS()),
                get_device=lambda: device(profiles))
    obj._capture_profile = active
    obj._imu_card_epoch = 0
    obj.get_logger = lambda: NS(error=lambda message: None)
    output = []
    obj._imu_pub = NS(publish=lambda message: output.append(json.loads(message.data)))
    obj._imu = MotionSource('/g1/camera/rgbd/imu', 'camera_depth_optical_frame', sdk(),
                            lambda ms, domain: int(ms * 1e6), obj._publish_imu,
                            clock_id=lambda: 'realsense_sdk_global_time_unix')
    obj._imu.configure(active)
    def deliver(kind, stamp):
        frame = motion(kind, stamp, [0., 0., 0.])
        frame.is_frameset = lambda: False
        obj._on_frame(frame)
    deliver('accel', 1020)
    deliver('gyro', 1022)
    assert len(output) == 1
    before = output[0]['epoch']
    # Old accel is slightly future in mapped SDK time, so a stamp cutoff alone
    # cannot distinguish it from a sample actually acquired after the restart.
    gates[3] = 1_010_000_000
    deliver('gyro', 1025)
    assert len(output) == 1
    deliver('accel', 1030)
    deliver('gyro', 1035)
    assert len(output) == 2 and output[1]['epoch'] != before


@pytest.mark.parametrize('reason', [None, 'SDK global capture clock unavailable',
                                    'missing color or measured depth capture',
                                    'stale, repeated, future or skewed capture time'])
@pytest.mark.parametrize('card,topic', [('camera_depth', '/g1/camera/depth'),
                                        ('camera_rgb', '/g1/camera/rgbd/color')])
def test_cold_card_start_waits_for_fresh_child_calibration(card, topic, reason):
    obj = plugin()
    obj._rgbd_status = None
    obj._rgbd_status_queue = queue.Queue()
    if reason is not None:
        obj._rgbd_status_queue.put(dict(state='unavailable', reason=reason, camera_info=[], last_capture_ns=0))
    source, _, _ = configured()
    source.depth_topic = '/g1/camera/depth'
    source.encode(Frames(), 1700000000130000000)
    def deliver():
        time.sleep(.03)
        status = source.info()
        status['last_capture_ns'] = time.time_ns()
        obj._rgbd_status_queue.put(status)
    worker = threading.Thread(target=deliver)
    worker.start()
    try:
        result = obj.dispatch('start', {'_tool_name': card})
        assert any(entry['topic'] == topic and entry['K'][0] == 400.
                   for entry in result['camera_info'])
    finally:
        worker.join(timeout=1)


def test_cold_card_start_wait_is_bounded_without_capture(monkeypatch):
    obj = plugin()
    obj._rgbd_status = None
    clock = [0.]
    waits = []
    def sleep(delay):
        waits.append(delay)
        clock[0] += delay
    monkeypatch.setitem(type(obj).dispatch.__globals__, 'time',
                        NS(time_ns=time.time_ns, monotonic=lambda: clock[0], sleep=sleep))
    result = obj.dispatch('start', {'_tool_name': 'camera_depth'})
    assert result['state'] == 'unavailable' and result['camera_info'] == []
    assert 10. <= sum(waits) <= 10.1


def test_cold_card_start_does_not_wait_after_explicit_source_failure(monkeypatch):
    obj = plugin()
    def sleep(delay):
        pytest.fail('a terminal source error must not wait for the startup deadline')
    monkeypatch.setitem(type(obj).dispatch.__globals__, 'time',
                        NS(time_ns=time.time_ns, monotonic=time.monotonic, sleep=sleep))
    result = obj.dispatch('start', {'_tool_name': 'camera_depth'})
    assert result['state'] == 'unavailable' and result['reason'] == 'test capture process disabled'


def test_cold_card_start_stops_waiting_if_owner_exits(monkeypatch):
    obj = plugin()
    obj._rgbd_status = None
    waits = []
    def sleep(delay):
        waits.append(delay)
        obj._proc = None
    monkeypatch.setitem(type(obj).dispatch.__globals__, 'time',
                        NS(time_ns=time.time_ns, monotonic=time.monotonic, sleep=sleep))
    result = obj.dispatch('start', {'_tool_name': 'camera_rgb'})
    assert result['state'] == 'unavailable' and len(waits) == 1
    assert len(result['camera_info']) == 1  # Only the independent display declaration remains.


@pytest.mark.parametrize('action', ['capture_photo', 'record_video'])
def test_media_request_enables_only_rgb_and_waits_for_new_display_frame(tmp_path, monkeypatch, action):
    from datetime import datetime
    import shutil
    from uuid import uuid4
    camera = plugin()
    cache_cls = load_class('_CameraFrameNode', dict(Node=object, threading=threading, time=time))
    cache = cache_cls.__new__(cache_cls)
    cache._condition = threading.Condition()
    cache._sequence, cache._latest = 0, None
    camera._frame_node = cache
    def deliver(data):
        stamp = time.time_ns()
        cache._on_frame(NS(data=data, format='jpeg', header=NS(stamp=NS(
            sec=stamp // 10**9, nanosec=stamp % 10**9))))
    deliver(b'\xff\xd8old\xff\xd9')
    media_cls = load_class('VisionCapturePlugin', dict(
        Path=Path, threading=threading, time=time, datetime=datetime, uuid4=uuid4, shutil=shutil,
        _VISION_FIRST_FRAME_TIMEOUT_S=.5, _VISION_MAX_FRAME_AGE_S=3.))
    media = media_cls(dict(output_dir=str(tmp_path)), 'g1', object(), camera)
    recorded = []
    media._record_video_async = lambda active: recorded.append(media._frame(timeout_s=.5)[0]['data'])
    monkeypatch.setattr(shutil, 'which', lambda command: '/test/ffmpeg')
    # Media needs the existing display RGB, not computation calibration.
    camera._rgbd_info = lambda: pytest.fail('media must not wait for RGB-D calibration')
    def fresh_frame():
        deadline = time.monotonic() + .5
        while not camera._card_epochs[0] and time.monotonic() < deadline:
            time.sleep(.001)
        deliver(b'\xff\xd8new\xff\xd9')
    producer = threading.Thread(target=fresh_frame)
    producer.start()
    try:
        result = media.dispatch(action, {'duration_s': 1})
        if action == 'capture_photo':
            assert result['ok'], result
            assert Path(result['file_path']).read_bytes() == b'\xff\xd8new\xff\xd9'
        else:
            assert result['ok'], result
            media._active_recording['thread'].join(timeout=1)
            assert recorded == [b'\xff\xd8new\xff\xd9']
        assert camera._card_epochs[0] > 0
        assert list(camera._card_epochs)[1:] == [0, 0, 0]
        assert camera.is_running()
        del camera._rgbd_info
        assert camera.dispatch('info', {'_tool_name': 'camera_rgb'})['state'] == 'running'
    finally:
        producer.join(timeout=1)


@pytest.mark.parametrize('configuration_error', [None, 'unsupported RealSense depth distortion model'])
def test_child_status_distinguishes_configuration_failure_from_capture_warmup(configuration_error):
    obj, _ = owner(mp.get_context('spawn').Array('q', [0, 0, 0, 0]))
    source, _, _ = configured()
    source.reason = 'SDK global capture clock unavailable'
    obj._rgbd = source
    obj._imu = NS(status=lambda: dict(state='unavailable'))
    obj._rgbd_configuration_error = configuration_error
    statuses = queue.Queue()
    obj._publish_rgbd_status.__globals__['rgbd_status_queue'] = statuses
    obj._publish_rgbd_status()
    status = statuses.get_nowait()
    assert status['configuration_error'] == configuration_error
    assert status['camera_info'] == []


def body_imu_card(camera=None):
    import math
    node_cls = load_class('_LowStateNode', dict(Node=object, threading=threading, json=json,
                                               math=math, time=time, String=lambda: NS()))
    node = node_cls.__new__(node_cls)
    node._lock = threading.Lock()
    node._imu_publish_lock = threading.RLock()
    node._imu_enabled = True
    node._state_samples = 0
    node._last_imu, node._last_posture, node._last_battery = {}, {}, {}
    node._last_bms_time = node._last_mainboard_time = 0.
    outputs = {name: [] for name in ('imu', 'joints', 'battery', 'mainboard')}
    for name, messages in outputs.items():
        setattr(node, '_' + name + '_pub', NS(publish=messages.append))
    state_cls = load_class('StatePlugin', {})
    card = state_cls.__new__(state_cls)
    card._camera, card._node = camera, node
    card._imu_topic, card._battery_topic = '/g1/state/imu', '/g1/state/battery'
    card._joints_topic, card._mainboard_topic = '/g1/state/joints', '/g1/state/mainboard'
    sample = NS(imu_state=NS(quaternion=[1., 0., 0., 0.], gyroscope=[.1, .2, .3],
                            accelerometer=[0., 0., 9.81], rpy=[0., .1, 0.], temperature=32.),
                motor_state=[NS(q=.2, dq=.1, tau_est=.3, temperature=[30, 31]) for _ in range(10)])
    return card, node, sample, outputs


@pytest.mark.parametrize('camera_enabled', [False, True])
def test_body_imu_card_stop_persists_and_leaves_lowstate_siblings_running(camera_enabled):
    camera = plugin() if camera_enabled else None
    if camera is not None:
        camera.ensure_rgb_output()
        camera.dispatch('start', {'_tool_name': 'camera_depth'})
    card, node, sample, outputs = body_imu_card(camera)
    node._on_state(sample)
    original = json.loads(outputs['imu'][0].data)
    card.dispatch('start', {'_tool_name': 'imu'})
    if camera is not None:
        assert camera._card_epochs[3] > 0
    assert card.dispatch('stop', {'_tool_name': 'imu'})['state'] == 'idle'
    node._on_state(sample)
    node._on_state(sample)
    assert len(outputs['imu']) == 1
    assert card.dispatch('info', {'_tool_name': 'imu'})['state'] == 'idle'
    assert node._last_imu == original and node.get_posture()['posture'] == 'standing'
    assert len(outputs['joints']) == 2
    node._on_bms(NS(soc=90, soh=98, current=10, bmsvoltage=[24], cell_vol=[4],
                    temperature=[30], cycle=5))
    node._on_mainboard(NS(temperature=[31], fan_state=[1], value=[2.], state=[1]))
    assert len(outputs['battery']) == len(outputs['mainboard']) == 1
    assert card.dispatch('info', {'_tool_name': 'joints'})['state'] == 'running'
    if camera is not None:
        assert camera._card_epochs[3] == 0 and camera._card_epochs[0] and camera._card_epochs[1]
        assert camera.is_running()
    assert card.dispatch('start', {'_tool_name': 'imu'})['state'] == 'running'
    node._on_state(sample)
    assert len(outputs['imu']) == 2 and json.loads(outputs['imu'][-1].data) == original
    assert card.dispatch('info', {'_tool_name': 'imu'})['state'] == 'running'
    if camera is not None:
        assert camera._card_epochs[3] > 0


def test_body_imu_stop_waits_for_inflight_publication_and_blocks_following_output():
    card, node, sample, outputs = body_imu_card()
    publishing, release, stopped = threading.Event(), threading.Event(), threading.Event()
    def publish(message):
        publishing.set()
        assert release.wait(timeout=1)
        outputs['imu'].append(message)
    node._imu_pub.publish = publish
    producer = threading.Thread(target=node._on_state, args=(sample,))
    producer.start()
    assert publishing.wait(timeout=1)
    def stop():
        card.dispatch('stop', {'_tool_name': 'imu'})
        stopped.set()
    stopper = threading.Thread(target=stop)
    stopper.start()
    try:
        assert not stopped.wait(timeout=.03)
    finally:
        release.set()
        producer.join(timeout=1)
        stopper.join(timeout=1)
    assert stopped.is_set() and len(outputs['imu']) == 1
    node._on_state(sample)
    assert len(outputs['imu']) == 1
