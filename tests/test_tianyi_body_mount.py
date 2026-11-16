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
