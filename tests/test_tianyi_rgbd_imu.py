"""Offline Orbbec camera motion contract; no native ROS or camera access."""
import importlib.util
import json
import math
import queue
from pathlib import Path
import sys
import threading
from collections import deque
from types import SimpleNamespace as NS

import numpy as np
import pytest

PATH = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0/rgbd_camera.py'
SPEC = importlib.util.spec_from_file_location('tianyi_rgbd_imu', PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)
NOW = 10_000_000_000


def motion(stamp=NOW, frame='gyro_optical', xyz=(1., 2., 3.)):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9)),
              angular_velocity=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
              linear_acceleration=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
              orientation_covariance=[-1.] + [0.] * 8)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


class Node:
    def __init__(self, *args, **kwargs):
        self.context = kwargs.get('context')
        self.subscriptions = {}
        self.subscription_qos = {}
        self.publishers = {}
        self.now_ns = NOW
    def create_publisher(self, msg_type, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]
    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        self.subscription_qos[topic] = qos
        return callback
    def create_client(self, *args): return NS(service_is_ready=lambda: False)
    def create_timer(self, *args): pass
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=self.now_ns))


class Buffer:
    def __init__(self):
        self.calls = []
        s = math.sqrt(.5)
        self.quaternions = {'gyro_optical': (0., 0., s, s), 'accel_frame': (s, 0., 0., s)}
        self.translations = {}
    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp.nanoseconds))
        if source not in self.quaternions: raise LookupError('unknown source frame')
        x, y, z, w = self.quaternions[source]
        transform = NS(rotation=NS(x=x, y=y, z=z, w=w))
        if source in self.translations:
            transform.translation = self.translations[source]
        return NS(transform=transform)


class CallbackExecutor:
    def __init__(self, context):
        self.context = context; self.nodes = []; self.stopped = threading.Event(); self.error = None
        self.jobs = queue.Queue()
    def add_node(self, node): self.nodes.append(node)
    def spin_once(self, timeout_sec):
        if self.error: raise RuntimeError(self.error)
        try: job = self.jobs.get(timeout=timeout_sec)
        except queue.Empty: return
        if job is not None: job()
    def wake(self): self.stopped.set(); self.jobs.put(None)
    def shutdown(self, **kw): self.wake()


class CameraContext:
    def __init__(self): self.callbacks = []; self.closed = False
    def on_shutdown(self, callback): self.callbacks.append(callback)
    def ok(self): return not self.closed
    def try_shutdown(self):
        if not self.closed:
            self.closed = True
            for callback in self.callbacks: callback()


@pytest.fixture
def source(monkeypatch, request):
    monkeypatch.setitem(sys.modules, 'rclpy.node', NS(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=NS(depth=5),
        QoSProfile=lambda **kw: NS(**kw), ReliabilityPolicy=NS(BEST_EFFORT='best_effort')))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=NS(from_msg=lambda s: NS(nanoseconds=s.sec*10**9+s.nanosec))))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=object, CameraInfo=object, CompressedImage=object, Imu=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=object))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=lambda b, n: NS(buffer=b, node=n)))
    context = CameraContext()
    monkeypatch.setitem(sys.modules, 'rclpy', NS(init=lambda **kw: None))
    monkeypatch.setitem(sys.modules, 'rclpy.context', NS(Context=CameraContext))
    monkeypatch.setitem(sys.modules, 'rclpy.executors', NS(SingleThreadedExecutor=CallbackExecutor,
        ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}),
        ShutdownException=type('ShutdownException', (Exception,), {})))
    ros = NS(ctx_tianyi=context, ctx_core=object(), executor_tianyi=NS(add_node=lambda n: None),
             executor_core=NS(add_node=lambda n: None))
    plugin = m.RgbdCameraPlugin(getattr(request, 'param', {}), 'test', ros)
    plugin._work = lambda: None
    plugin.start()
    plugin.registered = True
    plugin.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    plugin.count = 1
    yield plugin
    plugin.stop()
    context.try_shutdown()
    if hasattr(plugin, '_source_spin'):
        plugin._source_spin.join(timeout=1)
        assert not plugin._source_spin.is_alive()
    if plugin._camera_executor is not None:
        plugin._camera_spin.join(timeout=1)
        assert not plugin._camera_spin.is_alive()


def feed(source, gyro=None, accel=None):
    source._imu('accel', accel or motion(NOW-5_000_000, 'accel_frame', (0., 0., 9.81)))
    source._imu('gyro', gyro or motion())


def packets(source):
    publisher = source.pub_node.publishers.get('/test/camera/rgbd/imu')
    return [] if publisher is None else [json.loads(msg.data) for msg in publisher.messages]


def test_image_publication_lock_cannot_block_motion_ingress(source):
    done = threading.Event()
    def receive():
        feed(source)
        done.set()
    worker = threading.Thread(target=receive)
    with source.lock:
        worker.start()
        independent = done.wait(timeout=.2)
    worker.join(timeout=1)
    assert independent, 'image work held the IMU callback until its captures aged'
    assert len(packets(source)) == 1


def test_motion_ingress_and_tf_have_no_image_callbacks_on_their_executor(source):
    assert source.imu_node is not source.sub_node
    assert source._camera_executor.nodes == [source.imu_node]
    assert source.imu_listener.node is source.imu_node
    assert set(source.imu_node.subscriptions) == {
        '/ob_camera_head/gyro/sample', '/ob_camera_head/accel/sample'}
    assert not any('/sample' in topic for topic in source.sub_node.subscriptions)


def test_camera_callbacks_run_without_body_executor_and_keep_imu_separate(source):
    delivered = threading.Event()
    source._source_executor.jobs.put(delivered.set)
    assert delivered.wait(timeout=.2)
    assert source._source_executor.nodes == [source.sub_node]
    assert source._source_executor is not source._camera_executor
    assert source._camera_executor.nodes == [source.imu_node]
    assert source._source_executor.context is not source._camera_executor.context
    assert source._source_context is not source._body_context
    worker = source._source_spin
    source.stop(); source.start()
    assert source._source_spin is worker and worker.is_alive()


def test_slow_tf_work_cannot_publish_a_packet_past_capture_age_limit(source):
    lookup = source.imu_buffer.lookup_transform
    def delayed(*args):
        transform = lookup(*args)
        source.sub_node.now_ns += 275_000_000
        return transform
    source.imu_buffer.lookup_transform = delayed
    feed(source)
    assert packets(source) == []
    assert 'stale' in source._imu_status()['reason']


def test_motion_publication_diagnostics_preserve_and_report_real_capture_gaps(source):
    feed(source)
    source.sub_node.now_ns = NOW+120_000_000
    feed(source, gyro=motion(NOW+120_000_000), accel=motion(NOW+120_000_000, 'accel_frame'))
    status = source._imu_status()
    assert status['publication_attempts'] == 2
    assert status['max_capture_gap_ns'] == 120_000_000
    feed(source, gyro=motion(NOW+100_000_000), accel=motion(NOW+120_000_000, 'accel_frame'))
    assert source._imu_status()['publication_attempts'] == 2


def test_imu_release_waits_for_publication_then_rejects_late_motion(source):
    source.managed = True
    source.acquire('color'); source.acquire('imu')
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    publisher = source.pub_node.publishers['/test/camera/rgbd/imu']
    publish = publisher.publish
    def blocked(message):
        entered.set()
        assert release.wait(timeout=2)
        publish(message)
    publisher.publish = blocked
    receiver = threading.Thread(target=lambda: feed(source, accel=motion(NOW, 'accel_frame')))
    receiver.start()
    assert entered.wait(timeout=1)
    def stop():
        source.release('imu'); stopped.set()
    stopper = threading.Thread(target=stop); stopper.start()
    waiting = not stopped.wait(timeout=.05)
    release.set(); receiver.join(timeout=1); stopper.join(timeout=1)
    assert waiting and stopped.is_set() and source.running
    assert len(packets(source)) == 1
    feed(source, gyro=motion(NOW+1_000_000))
    assert len(packets(source)) == 1


def test_third_port_emits_capture_clock_raw_vectors_and_separate_90_degree_rotations(source):
    feed(source)
    assert source.get_tool()['topic_out'][2] == {'topic': '/test/camera/rgbd/imu', 'format': 'sensor/imu'}
    packet, = packets(source)
    assert packet['schema'] == 'motus.imu/1'
    assert packet['source_id'] and packet['epoch']
    assert packet['stamp_ns'] == NOW
    assert packet['accel_stamp_ns'] == NOW-5_000_000
    assert packet['stamp_source'] == 'measurement'
    assert packet['clock_id'] == source._declarations()[0]['vendor']['clock_id']
    assert packet['clock_id'] == source._declarations()[1]['vendor']['clock_id']
    assert packet['camera_frame_id'] == 'color_optical'
    assert (packet['gyro_frame_id'], packet['accel_frame_id']) == ('gyro_optical', 'accel_frame')
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert packet['linear_acceleration'] == [0., 0., 9.81]
    np.testing.assert_allclose(packet['camera_from_gyro'], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-12)
    np.testing.assert_allclose(packet['camera_from_accel'], [[1,0,0],[0,0,-1],[0,1,0]], atol=1e-12)
    assert packet['rotation_source'] == 'tf'
    assert packet['units'] == {'angular_velocity': 'rad/s', 'linear_acceleration': 'm/s^2'}
    assert set(source.imu_buffer.calls) == {('color_optical', 'gyro_optical', NOW), ('color_optical', 'accel_frame', NOW-5_000_000)}
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_reuses_vendor_streams_on_dedicated_imu_node_without_new_camera_owner(source):
    assert '/ob_camera_head/gyro/sample' in source.imu_node.subscriptions
    assert '/ob_camera_head/accel/sample' in source.imu_node.subscriptions
    assert source.imu_listener.node is source.imu_node


def test_motion_capture_intervals_survive_a_200ms_callback_backlog(source):
    # DDS keeps only the newest `depth` samples while the shared source executor
    # is occupied. Exercise the real pairing/publisher after that ingress boundary.
    feed(source, gyro=motion(xyz=(0., 0., 0.)))
    queues = {}
    for kind, frame in [('gyro', 'gyro_optical'), ('accel', 'accel_frame')]:
        topic = '/ob_camera_head/' + kind + '/sample'
        qos = source.imu_node.subscription_qos[topic]
        queue = deque(maxlen=qos.depth)
        for i in range(1, 21):
            queue.append(motion(NOW+i*10_000_000, frame, (0., 0., 0.) if kind == 'gyro' else (0., 0., 9.81)))
        queues[kind] = topic, queue
    source.sub_node.now_ns = NOW+200_000_000
    while any(queue for _, queue in queues.values()):
        for topic, queue in queues.values():
            if queue: source.imu_node.subscriptions[topic](queue.popleft())
    stamps = [p['stamp_ns'] for p in packets(source)]
    assert stamps == [NOW+i*10_000_000 for i in range(21)]
    assert max(b-a for a, b in zip(stamps, stamps[1:])) == 10_000_000


def test_overloaded_motion_ingress_keeps_recent_history_instead_of_half_second_latency(source):
    queues = {}
    for kind, frame in [('gyro', 'gyro_optical'), ('accel', 'accel_frame')]:
        topic = '/ob_camera_head/' + kind + '/sample'
        queue = deque(maxlen=source.imu_node.subscription_qos[topic].depth)
        for i in range(100):
            queue.append(motion(NOW-(99-i)*10_000_000, frame,
                                (0., 0., 0.) if kind == 'gyro' else (0., 0., 9.81)))
        queues[kind] = topic, queue
    while any(queue for _, queue in queues.values()):
        for topic, queue in queues.values():
            if queue:
                source.imu_node.subscriptions[topic](queue.popleft())
    stamps = [p['stamp_ns'] for p in packets(source)]
    assert stamps and stamps[-1] == NOW
    assert NOW-stamps[0] <= 200_000_000
    assert max(b-a for a, b in zip(stamps, stamps[1:])) == 10_000_000


def test_backlog_beyond_retained_history_exposes_the_real_capture_gap(source):
    feed(source, gyro=motion(xyz=(0., 0., 0.)))
    queues = {}
    for kind, frame in [('gyro', 'gyro_optical'), ('accel', 'accel_frame')]:
        topic = '/ob_camera_head/' + kind + '/sample'
        queue = deque(maxlen=source.imu_node.subscription_qos[topic].depth)
        for i in range(1, 32):
            queue.append(motion(NOW+i*10_000_000, frame,
                                (0., 0., 0.) if kind == 'gyro' else (0., 0., 9.81)))
        queues[kind] = topic, queue
    source.sub_node.now_ns = NOW+310_000_000
    while any(queue for _, queue in queues.values()):
        for topic, queue in queues.values():
            if queue:
                source.imu_node.subscriptions[topic](queue.popleft())
    stamps = [p['stamp_ns'] for p in packets(source)]
    assert stamps[:2] == [NOW, NOW+120_000_000]
    assert stamps[-1] == NOW+310_000_000


def test_motion_callback_executor_persists_across_card_restart(source):
    executor, worker = source._camera_executor, source._camera_spin
    assert executor.nodes == [source.imu_node] and worker.is_alive()
    source.stop()
    assert worker.is_alive()  # stopped subscriptions continue discarding old data
    source.start()
    assert source._camera_spin is worker and worker.is_alive()


def test_motion_callback_failure_marks_source_unavailable(source):
    source._camera_executor.error = 'callback failed'
    source._camera_executor.stopped.set()
    source._camera_spin.join(timeout=1)
    assert not source.running and source.stop_event.is_set()
    assert 'callback failed' in source.error
    with pytest.raises(RuntimeError, match='callback failed'):
        source.start()
    assert not source.running and 'callback failed' in source.error


@pytest.mark.parametrize('executor,worker,message', [
    ('_camera_executor', '_camera_spin', 'camera callbacks: callback failed'),
    ('_source_executor', '_source_spin', 'camera source callbacks: callback failed')])
def test_callback_failure_blocks_restart_before_executor_cleanup(source, executor, worker, message):
    lock, errors = source.lock, []
    class Lock:
        attempted = False
        def __enter__(self): lock.acquire()
        def __exit__(self, *args):
            lock.release()
            if source.error and not self.attempted:
                self.attempted = True
                try:
                    source.start()
                except RuntimeError as error:
                    errors.append(str(error))
    source.lock = Lock()
    getattr(source, executor).error = 'callback failed'
    getattr(source, executor).stopped.set()
    getattr(source, worker).join(timeout=1)
    assert errors == [message]
    assert not source.running and source.stop_event.is_set()


@pytest.mark.parametrize('frame', ['', 'unknown_body_frame'])
def test_missing_motion_frame_or_tf_withholds_only_imu(source, frame):
    feed(source, gyro=motion(frame=frame))
    assert packets(source) == []
    result = source.dispatch('info', {})
    assert result['state'] == 'running' and result['registered_source']
    assert result['camera_info'] and 'error' not in result
    assert result['imu_status']['state'] == 'unavailable'
    assert result['imu_status']['reason']


def test_tf_failure_recovers_without_restarting_rgbd(source):
    feed(source, gyro=motion(frame='unknown'))
    feed(source, gyro=motion(NOW+1_000_000))
    assert len(packets(source)) == 1
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_missing_accel_capture_does_not_block_later_matching_pairs(source):
    source._imu('accel', motion(NOW-50_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW-25_000_000))
    assert packets(source) == []
    # The older accel remains nearest to the unmatched gyro even after a
    # newer accel arrives; that hole must not pin the entire gyro queue.
    source._imu('accel', motion(NOW+5_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW+5_000_000))
    assert [p['stamp_ns'] for p in packets(source)] == [NOW+5_000_000]
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'


def test_waits_for_matching_accel_when_only_older_samples_exist(source):
    source._imu('accel', motion(NOW-50_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW-25_000_000))
    assert packets(source) == []
    source._imu('accel', motion(NOW-24_000_000, 'accel_frame'))
    assert [p['stamp_ns'] for p in packets(source)] == [NOW-25_000_000]


def test_matching_discards_accel_history_older_than_the_nearest_preceding_sample(source):
    for offset in range(-400, 1):
        source._imu('accel', motion(NOW+offset*1_000_000, 'accel_frame'))
    source._imu('gyro', motion())
    assert len(packets(source)) == 1
    assert len(source.imu_samples['accel']) == 1


@pytest.mark.parametrize('gyro_stamp,accel_stamp', [
    (NOW, NOW-20_000_001), (NOW-500_000_001, NOW-500_000_001),
    (NOW+50_000_001, NOW+50_000_001), (0, 0),
])
def test_skew_stale_future_or_zero_capture_stamp_is_not_restamped(source, gyro_stamp, accel_stamp):
    feed(source, motion(gyro_stamp), motion(accel_stamp, 'accel_frame'))
    assert packets(source) == []
    assert source.dispatch('info', {})['imu_status']['state'] == 'unavailable'


def test_no_source_global_clock_confirmation_means_no_imu_even_if_tf_exists(source):
    source.registered = False
    feed(source)
    assert packets(source) == []
    assert 'clock' in source.dispatch('info', {})['imu_status']['reason'].lower()


def test_bad_raw_vector_or_nonunit_tf_is_not_published(source):
    feed(source, gyro=motion(xyz=(float('nan'), 0., 0.)))
    assert packets(source) == []
    source.imu_buffer.quaternions['gyro_optical'] = (0., 0., 0., 2.)
    feed(source, gyro=motion(NOW+1))
    assert packets(source) == []
    assert source.running and source.error is None


def test_accel_arriving_after_gyro_can_complete_one_pair_without_duplicate_replay(source):
    source._imu('gyro', motion())
    source._imu('accel', motion(NOW, 'accel_frame'))
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 1


def test_status_becomes_unavailable_when_source_stops_producing(source):
    feed(source)
    source.sub_node.now_ns += 500_000_001
    result = source.dispatch('info', {})
    assert result['imu_status']['state'] == 'unavailable'
    assert 'stale' in result['imu_status']['reason']
    assert result['state'] == 'running' and 'error' not in result


def test_restart_changes_epoch_and_does_not_reuse_old_acceleration(source):
    feed(source)
    epoch = packets(source)[0]['epoch']
    source.stop()
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source._imu('gyro', motion(NOW+1))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 2
    assert packets(source)[1]['epoch'] != epoch


def test_disabled_source_keeps_rgbd_ready_with_declared_optional_port(source):
    source.stop()
    source.imu_enabled = False
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source.count = 1  # RGB-D has already emitted; only motion is disabled.
    feed(source)
    assert packets(source) == []
    result = source.dispatch('info', {})
    assert result['camera_info'] and result['state'] == 'running'
    assert result['imu_status']['state'] == 'disabled'
    assert 'error' not in result


@pytest.mark.parametrize('source', [{'imu_enabled': False}], indirect=True)
def test_disabled_config_never_attaches_motion_or_tf_subscriptions(source):
    assert source.imu_node is None
    assert '/ob_camera_head/gyro/sample' not in source.sub_node.subscriptions
    assert '/ob_camera_head/accel/sample' not in source.sub_node.subscriptions
    assert not hasattr(source, 'imu_listener')
    assert source.dispatch('info', {})['imu_status']['state'] == 'disabled'
    assert source.dispatch('info', {})['camera_info']


def test_missing_tf2_dependency_withholds_motion_without_failing_rgbd_start(source, monkeypatch):
    source.stop()
    source.imu_attached = False
    monkeypatch.setitem(sys.modules, 'tf2_ros', None)
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source.count = 1  # RGB-D has already emitted; only motion TF is missing.
    result = source.dispatch('info', {})
    assert result['state'] == 'running' and result['camera_info']
    assert 'error' not in result
    assert result['imu_status']['state'] == 'unavailable'
    assert 'tf2' in result['imu_status']['reason']


def test_source_capture_interruption_changes_epoch_on_resume(source):
    feed(source)
    epoch = packets(source)[0]['epoch']
    source.sub_node.now_ns += 600_000_000
    feed(source, motion(NOW+600_000_000), motion(NOW+600_000_000, 'accel_frame'))
    assert len(packets(source)) == 2
    assert packets(source)[1]['epoch'] != epoch


def test_restart_resets_capture_sampling_epoch(source):
    source.last_emit = NOW
    source.stop(); source.start()
    assert source.last_emit == 0


def test_long_interruption_cannot_be_hidden_by_reusing_stale_acceleration(source):
    feed(source)
    source.sub_node.now_ns += 600_000_000
    source._imu('gyro', motion(NOW+600_000_000))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+600_000_000, 'accel_frame'))
    assert len(packets(source)) == 2


def test_out_of_order_capture_is_not_published_twice(source):
    feed(source)
    source._imu('gyro', motion(NOW-1))
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    assert len(packets(source)) == 1



def test_source_time_tf_translation_and_rotation_use_one_lookup_per_sensor(source):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.012, y=-.034, z=.056),
                                      'accel_frame': NS(x=-.078, y=.09, z=-.011)}
    feed(source)
    packet, = packets(source)
    assert packet['camera_from_gyro_translation'] == [.012, -.034, .056]
    assert packet['camera_from_accel_translation'] == [-.078, .09, -.011]
    assert packet['translation_source'] == 'tf'
    np.testing.assert_allclose(packet['camera_from_gyro'], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-12)
    assert source.imu_buffer.calls == [('color_optical', 'gyro_optical', NOW),
                                       ('color_optical', 'accel_frame', NOW-5_000_000)]
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'available'


@pytest.mark.parametrize('translation', [None, NS(x=float('nan'), y=0., z=0.),
                                         NS(x=0., y=float('inf'), z=0.), NS(x=0., y=0.)])
def test_missing_or_invalid_tf_translation_preserves_attitude_only(source, translation):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.01, y=.02, z=.03)}
    if translation is not None:
        source.imu_buffer.translations['accel_frame'] = translation
    feed(source)
    packet, = packets(source)
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert 'camera_from_gyro_translation' not in packet
    assert 'camera_from_accel_translation' not in packet
    assert 'translation_source' not in packet
    status = source.dispatch('info', {})['imu_status']
    assert status['state'] == 'available'
    assert status['fusion']['state'] == 'unavailable'
    assert 'translation' in status['fusion']['reason']


def test_restart_does_not_reuse_old_tf_translation_or_acceleration(source):
    source.imu_buffer.translations = {'gyro_optical': NS(x=.01, y=.02, z=.03),
                                      'accel_frame': NS(x=.04, y=.05, z=.06)}
    feed(source)
    first, = packets(source)
    source.stop()
    source.start()
    source.registered = True
    source.geometry = {'frame_id': 'color_optical', 'K': np.eye(3)}
    source.imu_buffer.translations.clear()
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'unavailable'
    source._imu('gyro', motion(NOW+1))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+1, 'accel_frame'))
    resumed = packets(source)[1]
    assert resumed['epoch'] != first['epoch']
    assert 'camera_from_accel_translation' not in resumed
    assert source.dispatch('info', {})['imu_status']['state'] == 'available'
    assert source.dispatch('info', {})['imu_status']['fusion']['state'] == 'unavailable'


def test_managed_imu_restart_drops_late_pre_restart_accel(source):
    source.managed = True
    source.acquire('color')
    source.acquire('imu')
    feed(source, accel=motion(NOW, 'accel_frame'))
    source.release('imu')
    source.sub_node.now_ns = NOW+10_000_000
    source.acquire('imu')
    source._imu('accel', motion(NOW+1_000_000, 'accel_frame'))
    source._imu('gyro', motion(NOW+11_000_000))
    assert len(packets(source)) == 1
    source._imu('accel', motion(NOW+12_000_000, 'accel_frame'))
    assert len(packets(source)) == 2
    assert packets(source)[-1]['accel_stamp_ns'] == NOW+12_000_000
