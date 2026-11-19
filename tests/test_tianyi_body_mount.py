"""Real Tianyi RGB-D worker integration with offline ROS/TF fixtures."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import queue
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

PATH = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0/rgbd_camera.py'
SPEC = importlib.util.spec_from_file_location('tianyi_body_mount', PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)
STAMP = 12_345_678_901


@pytest.fixture
def source(monkeypatch):
    from test_tianyi_rgbd_imu import CameraContext, CallbackExecutor
    monkeypatch.setattr(m.time, "time_ns", lambda: STAMP + 100_000_000)
    events = []
    class Node:
        def __init__(self, *args, context=None):
            self.context = context; self.publishers = {}; self.destroyed = []
        def create_publisher(self, kind, topic, qos):
            def publish(msg):
                events.append((topic, deepcopy(msg)))
                if topic.endswith('/depth'): plugin.stop_event.set()
            self.publishers[topic] = NS(publish=publish)
            return self.publishers[topic]
        def destroy_publisher(self, publisher): self.destroyed.append(publisher)
        def create_subscription(self, *args): return args
        def create_client(self, *args): return NS(service_is_ready=lambda: False)
        def create_timer(self, *args): pass
    class Buffer:
        def __init__(self): self.calls = []; self.fail = False
        def lookup_transform(self, target, child, stamp):
            self.calls.append((target, child, stamp.nanoseconds))
            if self.fail: raise LookupError('missing base_link optical capture TF')
            s = math.sqrt(.5)
            return NS(header=NS(frame_id=target, stamp=NS(sec=stamp.nanoseconds//10**9, nanosec=stamp.nanoseconds%10**9)), child_frame_id=child,
                      transform=NS(translation=NS(x=.2, y=.1, z=1.), rotation=NS(x=0., y=0., z=s, w=s)))
    class Listener:
        def __init__(self, buffer, node): self.context = node.context; self.closed = False
        def unregister(self): self.closed = True
    class Image:
        def __init__(self): self.header = NS()
    monkeypatch.setitem(sys.modules, 'rclpy.node', NS(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy', NS(init=lambda **kw: None))
    monkeypatch.setitem(sys.modules, 'rclpy.context', NS(Context=CameraContext))
    monkeypatch.setitem(sys.modules, 'rclpy.executors', NS(SingleThreadedExecutor=CallbackExecutor,
        ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}),
        ShutdownException=type('ShutdownException', (Exception,), {})))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(ClockType=NS(ROS_TIME='ros')))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=Image, CameraInfo=object, CompressedImage=Image))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=object))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=Listener))
    context = CameraContext()
    ros = NS(ctx_tianyi=context, ctx_core='output-domain', executor_tianyi=NS(add_node=lambda n: None),
             executor_core=NS(add_node=lambda n: None))
    plugin = m.RgbdCameraPlugin({'imu_enabled': False}, 'test', ros)
    worker = plugin._work
    plugin._work = lambda: None
    plugin.start()
    plugin.worker.join()
    plugin._work = worker
    plugin.registered = True
    k = [400., 0., 320., 0., 400., 240., 0., 0., 1.]
    calibration = NS(width=640, height=480, k=k, d=[0.]*5, distortion_model='plumb_bob', header=NS(frame_id='head_optical'))
    plugin.geometry = m.make_geometry(calibration, calibration, registered=True)
    plugin.events = events
    yield plugin
    plugin.stop()
    context.try_shutdown()
    plugin._source_spin.join(timeout=1)
    assert not plugin._source_spin.is_alive()


def emit(source, stamp=STAMP, color_stamp=None):
    header = NS(frame_id='head_optical', stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9))
    color = NS(header=deepcopy(header), encoding='bgr8', width=640, height=480, step=1920,
               data=np.zeros((480, 640, 3), np.uint8).tobytes())
    depth = NS(header=deepcopy(header), encoding='16UC1', is_bigendian=False, width=640, height=480, step=1280,
               data=np.ones((480, 640), '<u2').tobytes())
    if color_stamp is not None:
        color.header.stamp = NS(sec=color_stamp//10**9, nanosec=color_stamp%10**9)
    source.stop_event.clear()
    source.pending.put((color, depth, dict(source.client_epochs)) if source.managed else (color, depth))
    source._work()


@pytest.mark.parametrize('kind, defect', [('color', 'stride'), ('color', 'size'),
                                        ('depth', 'stride'), ('depth', 'size'),
                                        ('depth', 'encoding')])
def test_malformed_frame_does_not_stop_shared_rgbd(source, kind, defect):
    def pair(stamp):
        header = NS(frame_id='head_optical', stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9))
        return dict(color=NS(header=header, encoding='bgr8', width=640, height=480, step=1924,
                             data=bytes(1924*480)),
                    depth=NS(header=header, encoding='16UC1', is_bigendian=False,
                             width=640, height=480, step=1284, data=bytes(1284*480)))

    bad = pair(STAMP)
    image = bad[kind]
    if defect == 'stride':
        image.step = image.width * (3 if kind == 'color' else 2) - 1
        image.data = bytes(image.step * image.height)
    elif defect == 'size':
        image.data = image.data[:-1]
    else:
        image.is_bigendian = True

    good = pair(STAMP+1)
    source.pending = queue.Queue()
    source.pending.put_nowait(tuple(bad.values()))
    source.pending.put_nowait(tuple(good.values()))
    worker = threading.Thread(target=source._work, daemon=True)
    worker.start()
    try:
        assert source.stop_event.wait(2)
    finally:
        source.stop_event.set()
        worker.join(2)
    assert not worker.is_alive()
    assert source.running and source.error is None
    assert [(t, m._stamp(img)) for t, img in source.events if t in (source.color_topic, source.depth_topic)] == [
        (source.color_topic, STAMP+1), (source.depth_topic, STAMP+1)]


def test_restart_waits_for_stopped_worker_and_discards_pending_capture(source, monkeypatch):
    old_worker_waiting, release_old_worker, restarted = (threading.Event() for _ in range(3))
    pending_at_restart = []

    def old_work():
        old_worker_waiting.set()
        release_old_worker.wait(3)

    source.stop()
    monkeypatch.setattr(source, '_work', old_work)
    source.start()
    assert old_worker_waiting.wait(1)
    old_worker = source.worker
    source.pending.put_nowait(('queued color from old run', 'queued depth from old run'))
    monkeypatch.setattr(source, '_work', lambda: pending_at_restart.append(source.pending.qsize()))
    stopper = threading.Thread(target=source.stop)

    def restart():
        source.start()
        restarted.set()

    starter = threading.Thread(target=restart)
    stopper.start()
    try:
        assert source.stop_event.wait(1)
        starter.start()
        assert not restarted.wait(.25)
    finally:
        release_old_worker.set()
        stopper.join(2)
        if starter.ident is not None:
            starter.join(2)
        old_worker.join(2)
    assert restarted.is_set() and not stopper.is_alive()
    source.worker.join(2)
    assert source.running and pending_at_restart == [0]


def test_ready_pair_survives_worker_draining_the_previous_slot(source):
    from test_tianyi_rgbd_camera import info

    class WorkerDrainsSlot(queue.Queue):
        def full(self):
            full = super().full()
            if full:
                self.get_nowait()  # Interleave a worker dequeue after the producer checks capacity.
            return full

    source.pending = WorkerDrainsSlot(maxsize=1)
    source.pending.put_nowait(('old color', 'old depth'))
    source.infos = {kind: info(frame='head_optical') for kind in ('color', 'depth')}
    msg = NS(width=640, height=480, header=NS(frame_id='head_optical',
             stamp=NS(sec=STAMP//10**9, nanosec=STAMP%10**9)))
    source._image('color', msg)
    source._image('depth', msg)
    assert source.pending.get_nowait() == (msg, msg)


def test_fourth_port_declarations_actual_capture_tf_and_cross_domain_output(source):
    ports = source.get_tool()['topic_out']
    assert [port['format'] for port in ports] == ['image/jpeg', 'image/depth-zlib', 'sensor/imu', 'state/extrinsic']
    declarations = source.get_tool()['camera_info']
    mount = declarations[0]['vendor']['body_mount']
    assert mount == declarations[1]['vendor']['body_mount']
    assert mount['parent_frame'] == 'base_link' and mount['child_frame'] == 'head_optical'
    assert mount['status'] == 'unavailable'
    emit(source)
    assert [topic for topic, _ in source.events] == [source.root+'/body_mount', '/test/camera/head', source.depth_topic]
    packet = json.loads(source.events[0][1].data)
    assert packet['stamp_ns'] == STAMP and packet['clock_id'] == source.clock
    assert packet['matrix'][0][3] == .2
    assert source.body_mount.buffer.calls == [('base_link', 'head_optical', STAMP)]
    assert source.body_mount.listener.context is source.sub_node.context
    assert source.root+'/body_mount' in source.pub_node.publishers
    assert source.root+'/body_mount' not in source.sub_node.publishers
    assert m._stamp(source.events[-1][1]) == STAMP
    assert source.dispatch('info', {})['body_mount_status']['status'] == 'available'


def test_rgbd_payloads_use_ros_uint8_buffers_and_preserve_pixels(source):
    from array import array
    import cv2, zlib
    emit(source)
    color, depth = [msg for topic, msg in source.events if topic in (source.color_topic, source.depth_topic)]
    assert isinstance(color.data, array) and color.data.typecode == 'B'
    assert isinstance(depth.data, array) and depth.data.typecode == 'B'
    image = cv2.imdecode(np.frombuffer(color.data, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (480, 640, 3) and not image.any()
    metric = np.frombuffer(zlib.decompress(depth.data), '<u2').reshape(480, 640)
    assert metric[240, 320] == 1
    assert np.count_nonzero(metric) > metric.size*.99


@pytest.mark.parametrize('focal', [400, 520])
def test_encoded_common_view_has_no_padding_and_keeps_measured_depth_holes(source, focal):
    import cv2, zlib
    from test_tianyi_rgbd_camera import info
    source.geometry = m.make_geometry(info(focal, frame='head_optical'),
                                      info(frame='head_optical'), registered=True)
    header = NS(frame_id='head_optical', stamp=NS(sec=STAMP//10**9, nanosec=STAMP%10**9))
    measured = np.full((480, 640), 1250, '<u2')
    measured[220:260, 300:340] = 0
    color = NS(header=deepcopy(header), encoding='bgr8', width=640, height=480, step=1920,
               data=np.full((480, 640, 3), 200, np.uint8).tobytes())
    depth = NS(header=deepcopy(header), encoding='16UC1', is_bigendian=False,
               width=640, height=480, step=1280, data=measured.tobytes())
    source.pending.put((color, depth))
    source._work()
    assert source.error is None
    rgb, metric = [msg for topic, msg in source.events if topic in (source.color_topic, source.depth_topic)]
    image = cv2.imdecode(np.frombuffer(rgb.data, np.uint8), cv2.IMREAD_COLOR)
    metric_image = np.frombuffer(zlib.decompress(metric.data), '<u2').reshape(480, 640)
    assert image.min() >= 199
    assert metric_image[240, 320] == 0
    assert np.all(metric_image[[0, -1]] == 1250)
    assert np.all(metric_image[:, [0, -1]] == 1250)
    assert set(np.unique(metric_image)) == {0, 1250}
    assert m._stamp(rgb) == m._stamp(metric) == STAMP
    rgb_info, depth_info = source._declarations()
    assert rgb_info['K'] == depth_info['K'] == source.geometry['K'].ravel().tolist()


def test_missing_tf_emits_no_held_body_matrix_and_images_continue(source):
    emit(source)
    source.body_mount.buffer.fail = True
    emit(source, STAMP+1)
    packets = [json.loads(msg.data) for topic, msg in source.events if topic.endswith('/body_mount')]
    assert len(packets) == 2 and packets[-1]['status'] == 'unavailable'
    assert 'matrix' not in packets[-1]
    assert source.count == 2 and source.running and source.error is None


def test_missing_tf_dependency_preserves_real_image_worker(source, monkeypatch):
    source.stop()
    monkeypatch.setitem(sys.modules, 'tf2_ros', None)
    worker = source._work
    source._work = lambda: None
    source.start(); source.worker.join(); source._work = worker
    source.registered = True
    source.geometry = {'frame_id': 'head_optical', 'K': np.eye(3)}
    # Keep real image resampling geometry from a validated source calibration.
    k = [400.,0.,320.,0.,400.,240.,0.,0.,1.]
    info = NS(width=640,height=480,k=k,d=[0.]*5,distortion_model='plumb_bob',header=NS(frame_id='head_optical'))
    source.geometry = m.make_geometry(info, info, registered=True)
    emit(source)
    assert source.running and source.count == 1 and source.error is None
    packet = json.loads(source.events[0][1].data)
    assert packet['status'] == 'unavailable' and 'tf2' in packet['reason']


def test_restart_closes_listener_and_changes_epoch_with_driver_body_frame_override(source, monkeypatch):
    old_epoch = source._declarations()[0]['vendor']['body_mount']['epoch']
    listener = source.body_mount.listener
    source.stop()
    assert listener.closed
    monkeypatch.setenv('CAMERA_BODY_FRAME', 'torso_reference')
    source._work = lambda: None
    source.start(); source.worker.join()
    source.geometry = {'frame_id': 'head_optical', 'K': np.eye(3)}
    declaration = source._declarations()[0]['vendor']['body_mount']
    assert declaration['parent_frame'] == 'torso_reference'
    assert declaration['epoch'] != old_epoch and declaration['status'] == 'unavailable'


def test_mount_publisher_runtime_failure_preserves_images(source):
    def fail(message): raise RuntimeError('mount output transport failed')
    source.body_mount.publisher.publish = fail
    emit(source)
    assert [topic for topic, _ in source.events] == [source.color_topic, source.depth_topic]
    assert source.count == 1 and source.running and source.error is None
    assert source.dispatch('info', {})['body_mount_status']['status'] == 'unavailable'


def test_restarting_after_source_error_releases_previous_mount_listener(source):
    listener = source.body_mount.listener
    source.running = False
    source.stop_event.set()
    source._work = lambda: None
    source.start(); source.worker.join()
    assert listener.closed
    assert source.body_mount.listener is not listener


@pytest.mark.parametrize('streams', [('color',), ('depth',), ('color', 'depth')])
@pytest.mark.parametrize('offset', [-1_000_000, 0, 1_000_000])
def test_mount_matches_each_published_capture(source, streams, offset):
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-2_000_000))
    for stream in streams:
        source.acquire(stream)
    last = source.color_pub if streams == ('color',) else source.depth_pub
    publish = last.publish
    def finish(message):
        publish(message)
        source.stop_event.set()
    last.publish = finish
    emit(source, color_stamp=STAMP+offset)
    images = []
    if 'color' in streams:
        images.append((source.color_topic, STAMP+offset))
    if 'depth' in streams:
        images.append((source.depth_topic, STAMP))
    expected = [(source.root+'/body_mount', stamp) for stamp in sorted({s for _, s in images})]
    expected += images
    actual = [(topic, json.loads(msg.data)['stamp_ns'] if topic.endswith('/body_mount')
               else m._stamp(msg)) for topic, msg in source.events]
    assert actual == expected


def test_clock_reference_only_declared_after_global_source_confirmation(source, monkeypatch):
    reference = dict(clock_id='host_realtime', domain_id='actual-boot', timestamp_basis='host_realtime')
    monkeypatch.setattr(m, 'host_clock_reference', lambda: deepcopy(reference))
    source.registered = True
    values = source._declarations()
    assert values[0]['vendor']['clock_reference'] == reference
    assert values[0]['vendor']['clock_id'] == source.clock
    source.registered = False
    assert 'clock_reference' not in source._declarations()[0]['vendor']


def test_missing_tf_does_not_delay_first_or_age_out_next_pair(source, monkeypatch):
    now = [STAMP + 451_000_000]
    monkeypatch.setattr(m.time, 'time_ns', lambda: now[0])
    monkeypatch.setitem(sys.modules, 'rclpy.duration', NS(Duration=lambda **kw: NS(**kw)))
    source.body_mount.buffer.fail = True
    waits = []
    def no_transform(*args, timeout):
        waits.append(timeout.seconds)
        now[0] += round(timeout.seconds * 1e9)
        return False
    monkeypatch.setattr(source.body_mount.buffer, 'can_transform', no_transform, raising=False)
    original_get = source.pending.get
    def get_once(*args, **kwargs):
        if source.pending.empty():
            source.stop_event.set()
            raise queue.Empty
        return original_get(*args, **kwargs)
    monkeypatch.setattr(source.pending, 'get', get_once)

    emit(source)
    assert source.count == 1 and now[0] == STAMP + 451_000_000
    next_stamp = STAMP + source.frame_interval_ns
    now[0] = next_stamp + 451_000_000
    emit(source, stamp=next_stamp)
    assert source.count == 2 and not waits
    mounts = [json.loads(msg.data) for topic, msg in source.events if topic.endswith('/body_mount')]
    assert [packet['stamp_ns'] for packet in mounts] == [STAMP, next_stamp]
    assert all(packet['status'] == 'unavailable' and 'matrix' not in packet for packet in mounts)


def test_wait_for_capture_tf_does_not_block_camera_executor(source, monkeypatch):
    import threading
    source.count = 1
    monkeypatch.setitem(sys.modules, 'rclpy.duration', NS(Duration=lambda **kw: NS(**kw)))
    source.body_mount.buffer.fail = True
    waits = []
    def available(target, child, stamp, timeout):
        # TF delivery shares the image executor; an image callback must be able
        # to acquire the camera lock while the worker waits for the transform.
        def callback():
            with source.lock:
                source.body_mount.buffer.fail = False
        worker = threading.Thread(target=callback)
        worker.start(); worker.join(timeout=.2)
        assert not worker.is_alive()
        waits.append((target, child, stamp.nanoseconds, timeout.seconds))
        return True
    source.body_mount.buffer.can_transform = available
    emit(source)
    assert waits == [('base_link', 'head_optical', STAMP, .15)]
    packets = [json.loads(msg.data) for topic, msg in source.events if topic.endswith('/body_mount')]
    assert packets[-1]['status'] == 'available'
    assert packets[-1]['stamp_ns'] == STAMP


def test_stop_during_tf_wait_does_not_publish_images(source, monkeypatch):
    source.count = 1
    monkeypatch.setitem(sys.modules, 'rclpy.duration', NS(Duration=lambda **kw: NS(**kw)))
    def stop(*args, **kwargs):
        source.stop_event.set()
        return False
    source.body_mount.buffer.can_transform = stop
    emit(source)
    assert not source.events


def test_tf_wait_timeout_keeps_images_but_never_reuses_transform(source, monkeypatch):
    source.count = 1
    monkeypatch.setitem(sys.modules, 'rclpy.duration', NS(Duration=lambda **kw: NS(**kw)))
    source.body_mount.buffer.can_transform = lambda *args, **kwargs: False
    source.body_mount.buffer.fail = True
    emit(source)
    packets = [json.loads(msg.data) for topic, msg in source.events if topic.endswith('/body_mount')]
    assert packets[-1]['status'] == 'unavailable'
    assert 'matrix' not in packets[-1]
    assert any(topic.endswith('/depth') for topic, msg in source.events)


def test_tf_arriving_after_50ms_retains_exact_capture_stamp(source, monkeypatch):
    source.count = 1
    monkeypatch.setitem(sys.modules, 'rclpy.duration', NS(Duration=lambda **kw: NS(**kw)))
    source.body_mount.buffer.fail = True
    def delayed(target, child, stamp, timeout):
        if timeout.seconds >= .12:
            source.body_mount.buffer.fail = False
        return not source.body_mount.buffer.fail
    source.body_mount.buffer.can_transform = delayed
    emit(source)
    packets = [json.loads(msg.data) for topic, msg in source.events if topic.endswith('/body_mount')]
    assert packets[-1]['status'] == 'available'
    assert packets[-1]['stamp_ns'] == STAMP


@pytest.mark.parametrize('age', [500_000_001, -50_000_001])
def test_worker_discards_capture_that_expires_during_tf_wait(source, monkeypatch, age):
    source.count = 1
    # Finish one queue item, then stop the worker without publishing an image.
    original_get = source.pending.get
    def get(*args, **kwargs):
        if source.pending.empty():
            source.stop_event.set()
            raise queue.Empty
        return original_get(*args, **kwargs)
    monkeypatch.setattr(source.pending, 'get', get)
    monkeypatch.setattr(source.body_mount, 'wait_for_capture',
                        lambda stamp: monkeypatch.setattr(m.time, 'time_ns', lambda: STAMP + age))
    emit(source)
    assert source.events == []
    assert source.error is None and source.running


def test_active_sibling_cards_can_restart_failed_source(source):
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP))
    cards = [m.RgbdInputCard(source, kind) for kind in ('color', 'depth', 'imu')]
    for card in cards:
        card.start()
    source.running = False
    source.stop_event.set()
    source.error = 'encoding failed'
    cards[1].start()
    assert source.running and source.error is None
    assert set(source.clients) == {'color', 'depth', 'imu'}
    for card in cards:
        card.stop()


@pytest.mark.parametrize('age', [500_000_001, -50_000_001])
def test_worker_discards_pair_that_expires_in_pending_queue(source, monkeypatch, age):
    original_get = source.pending.get
    def get(*args, **kwargs):
        if source.pending.empty():
            source.stop_event.set()
            raise queue.Empty
        item = original_get(*args, **kwargs)
        monkeypatch.setattr(m.time, 'time_ns', lambda: STAMP + age)
        # Expired work must be dropped before encoding can fault the source.
        item[0].encoding = 'unsupported'
        return item
    monkeypatch.setattr(source.pending, 'get', get)
    emit(source)
    assert source.events == []
    assert source.error is None and source.running


def test_card_restart_after_encoding_fault_restores_sibling_outputs(source, monkeypatch):
    import cv2
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    cards = [m.RgbdInputCard(source, kind) for kind in ('color', 'depth', 'imu')]
    for card in cards:
        card.start()
    geometry = source.geometry
    listener = source.body_mount.listener
    with monkeypatch.context() as patch:
        patch.setattr(cv2, 'imencode', lambda *args: (False, None))
        emit(source)
    assert not source.running and source.error == 'RGB-D color encoding failed'
    source.pending.put(('previous start', 'previous start'))
    source.frames['color'].append('previous start')
    # Keep the restarted worker deterministic while exercising its lifecycle.
    worker = source._work
    source._work = lambda: None
    cards[1].start()
    source.worker.join()
    source._work = worker
    assert listener.closed and source.body_mount.listener is not listener
    assert source.pending.empty() and not any(source.frames.values())
    assert source.geometry is None and not source.registered and source.count == 0
    source.geometry, source.registered = geometry, True
    emit(source, STAMP+1)
    assert [topic for topic, _ in source.events] == [source.root+'/body_mount', source.color_topic, source.depth_topic]
    assert source.running and source.error is None and source.count == 1
    assert set(source.clients) == {'color', 'depth', 'imu'}
    for card in cards:
        card.stop()
