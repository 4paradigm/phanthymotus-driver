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
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(ClockType=NS(ROS_TIME='ros')))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=Image, CameraInfo=object, CompressedImage=Image))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=object))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=Listener))
    ros = NS(ctx_tianyi='robot-domain', ctx_core='output-domain', executor_tianyi=NS(add_node=lambda n: None),
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


def emit(source, stamp=STAMP):
    header = NS(frame_id='head_optical', stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9))
    color = NS(header=deepcopy(header), encoding='bgr8', width=640, height=480, step=1920,
               data=np.zeros((480, 640, 3), np.uint8).tobytes())
    depth = NS(header=deepcopy(header), encoding='16UC1', is_bigendian=False, width=640, height=480, step=1280,
               data=np.ones((480, 640), '<u2').tobytes())
    source.stop_event.clear()
    source.pending.put((color, depth))
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
    assert [topic.rsplit('/', 1)[-1] for topic, _ in source.events] == ['body_mount', 'color', 'depth']
    packet = json.loads(source.events[0][1].data)
    assert packet['stamp_ns'] == STAMP and packet['clock_id'] == source.clock
    assert packet['matrix'][0][3] == .2
    assert source.body_mount.buffer.calls == [('base_link', 'head_optical', STAMP)]
    assert source.body_mount.listener.context == 'robot-domain'
    assert source.root+'/body_mount' in source.pub_node.publishers
    assert source.root+'/body_mount' not in source.sub_node.publishers
    assert m._stamp(source.events[-1][1]) == STAMP
    assert source.dispatch('info', {})['body_mount_status']['status'] == 'available'


def test_rgbd_payloads_use_ros_uint8_buffers_and_preserve_pixels(source):
    from array import array
    import cv2, zlib
    emit(source)
    color, depth = [msg for topic, msg in source.events if topic.endswith(('/color', '/depth'))]
    assert isinstance(color.data, array) and color.data.typecode == 'B'
    assert isinstance(depth.data, array) and depth.data.typecode == 'B'
    image = cv2.imdecode(np.frombuffer(color.data, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (480, 640, 3) and not image.any()
    metric = np.frombuffer(zlib.decompress(depth.data), '<u2').reshape(480, 640)
    assert metric[240, 320] == 1
    assert np.count_nonzero(metric) > metric.size*.99


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
    assert [topic.rsplit('/', 1)[-1] for topic, _ in source.events] == ['color', 'depth']
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
