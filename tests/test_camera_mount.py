"""Capture-time body-camera TF contract, without ROS or hardware."""
import importlib
import json
import math
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest


class Node:
    def __init__(self):
        self.messages = []
        self.destroyed = []
        self.fail_publish = False
    def create_publisher(self, kind, topic, qos):
        self.topic = topic
        def publish(message):
            if self.fail_publish:
                raise RuntimeError('transport unavailable')
            self.messages.append(json.loads(message.data))
        return NS(publish=publish)
    def destroy_publisher(self, publisher): self.destroyed.append(publisher)


def transform(parent='base_link', child='optical', offset=.25):
    s = math.sqrt(.5)
    return NS(header=NS(frame_id=parent, stamp=NS(sec=0, nanosec=0)), child_frame_id=child,
              transform=NS(translation=NS(x=offset, y=-.5, z=1.2),
                           rotation=NS(x=0., y=0., z=s, w=s)))


@pytest.fixture
def source(monkeypatch):
    class Buffer:
        def __init__(self): self.calls = []; self.value = transform(); self.failure = None
        def lookup_transform(self, target, child, stamp):
            self.calls.append((target, child, stamp.nanoseconds, stamp.clock_type))
            if self.failure: raise self.failure
            if not getattr(self, 'preserve_stamp', False):
                self.value.header.stamp = NS(sec=stamp.nanoseconds//10**9, nanosec=stamp.nanoseconds%10**9)
            return self.value
    class Listener:
        def __init__(self, buffer, node): self.node = node; self.closed = False
        def unregister(self): self.closed = True
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(ClockType=NS(ROS_TIME='ros')))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=Listener))
    module = importlib.import_module('common.camera_mount')
    node = Node()
    producer = module.BodyMountSource(node, '/camera', body_frame='base_link',
                                      camera_frame=lambda: 'optical', clock_id=lambda: 'capture')
    return producer, node


def test_exact_capture_time_nonzero_transform_and_dynamic_change(source):
    mount, node = source
    assert mount.publish(12_345_678_901)
    packet, = node.messages
    assert mount.buffer.calls == [('base_link', 'optical', 12_345_678_901, 'ros')]
    np.testing.assert_allclose(packet['matrix'], [[0,-1,0,.25],[1,0,0,-.5],[0,0,1,1.2],[0,0,0,1]], atol=1e-12)
    assert packet['stamp_ns'] == 12_345_678_901 and packet['clock_id'] == 'capture'
    assert packet['stamp_source'] == 'measurement' and packet['accuracy_validated'] is False
    assert packet['mode'] == 'dynamic' and packet['transform_direction'] == 'body_from_camera'
    assert packet['length_unit'] == 'm' and packet['status'] == 'available'
    mount.buffer.value = transform(offset=.75)
    assert mount.publish(12_345_678_902)
    assert node.messages[-1]['matrix'][0][3] == .75
    assert node.messages[-1]['epoch'] == packet['epoch']
    declaration = mount.declaration()
    assert declaration.pop('topic') == '/camera/body_mount'
    assert {k: node.messages[-1][k] for k in declaration} == declaration


def test_missing_tf_never_uses_latest_or_replays_held_matrix(source):
    mount, node = source
    assert mount.publish(100)
    mount.buffer.failure = LookupError('base_link to optical unavailable at capture time')
    assert mount.publish(200) is False
    assert [call[2] for call in mount.buffer.calls] == [100, 200]
    packet = node.messages[-1]
    assert packet['status'] == 'unavailable' and 'matrix' not in packet
    assert 'base_link' in packet['reason'] and 'optical' in packet['reason']
    assert mount.status()['status'] == 'unavailable'


@pytest.mark.parametrize('change', [
    lambda t: setattr(t.header, 'frame_id', 'odom'),
    lambda t: setattr(t, 'child_frame_id', 'other_optical'),
    lambda t: setattr(t.transform.translation, 'x', float('nan')),
    lambda t: setattr(t.transform.rotation, 'w', 2.),
    lambda t: setattr(t.transform.rotation, 'x', float('inf')),
])
def test_invalid_tf_is_explicitly_unavailable(source, change):
    mount, node = source
    change(mount.buffer.value)
    assert mount.publish(123) is False
    assert node.messages[-1]['status'] == 'unavailable'
    assert 'matrix' not in node.messages[-1]


@pytest.mark.parametrize('stamp', [0, -1, 1.5, True])
def test_invalid_capture_time_never_queries_tf(source, stamp):
    mount, node = source
    assert mount.publish(stamp) is False
    assert mount.buffer.calls == []
    assert mount.status()['reason']


def test_missing_tf_package_still_publishes_unavailable_diagnostic(source, monkeypatch):
    _, node = source
    monkeypatch.setitem(sys.modules, 'tf2_ros', None)
    module = importlib.import_module('common.camera_mount')
    mount = module.BodyMountSource(node, '/camera', body_frame='base_link', camera_frame='optical', clock_id='capture')
    assert mount.publish(123) is False
    assert node.messages[-1]['status'] == 'unavailable' and 'tf2' in node.messages[-1]['reason']
    assert mount.declaration()['child_frame'] == 'optical'


def test_reset_epoch_and_close_release_resources_without_reusing_transform(source):
    mount, node = source
    assert mount.publish(123)
    epoch = mount.declaration()['epoch']
    listener, publisher = mount.listener, mount.publisher
    mount.reset()
    assert mount.declaration()['epoch'] != epoch
    assert mount.status()['status'] == 'unavailable'
    mount.close()
    mount.close()
    assert listener.closed and node.destroyed == [publisher]
    assert mount.publish(124) is False
    assert len(node.messages) == 1


def test_publisher_failure_is_contained(source):
    mount, node = source
    node.fail_publish = True
    assert mount.publish(123) is False
    assert mount.status()['status'] == 'unavailable'
    assert 'transport' in mount.status()['reason']


def test_unready_callable_identity_still_has_declaration_and_unavailable_packet(source):
    mount, node = source
    mount.camera_frame = lambda: None
    assert mount.publish(123) is False
    assert mount.declaration()['child_frame'] is None
    assert node.messages[-1]['status'] == 'unavailable'
    assert mount.buffer.calls == []


def test_publisher_constructor_failure_does_not_raise(source):
    _, node = source
    def fail(*args): raise RuntimeError('publisher allocation failed')
    node.create_publisher = fail
    module = importlib.import_module('common.camera_mount')
    mount = module.BodyMountSource(node, '/camera', body_frame='base_link', camera_frame='optical', clock_id='capture')
    assert mount.publish(123) is False
    assert mount.declaration()['status'] == 'unavailable'
    assert 'publisher allocation' in mount.status()['reason']


@pytest.mark.parametrize('returned_stamp', [0, 122, 124])
def test_stale_or_latest_tf_response_is_rejected_without_retry(source, returned_stamp):
    mount, node = source
    mount.buffer.preserve_stamp = True
    mount.buffer.value.header.stamp = NS(sec=0, nanosec=returned_stamp)
    assert mount.publish(123) is False
    assert len(mount.buffer.calls) == 1
    assert 'matrix' not in node.messages[-1]
    assert 'timestamp' in node.messages[-1]['reason']


def test_missing_tf_reason_names_selected_frames_even_for_generic_lookup_error(source):
    mount, node = source
    mount.buffer.failure = LookupError('not found')
    assert mount.publish(123) is False
    assert 'base_link' in node.messages[-1]['reason'] and 'optical' in node.messages[-1]['reason']

def test_estimated_capture_gate_never_falls_back_to_interpolated_tf(source):
    mount, node = source
    class Evidence:
        provenance = dict(calibration_kind='estimate', accuracy_validated=False)
        def lookup(self, *args):
            raise ValueError('no exact estimated capture certificate')
    mount.capture_evidence = Evidence()
    assert not mount.publish(12_345_678_901)
    assert not mount.buffer.calls
    packet = node.messages[-1]
    assert packet['status'] == 'unavailable' and 'matrix' not in packet
    assert packet['provenance']['calibration_kind'] == 'estimate'

def test_estimated_capture_preserves_provenance_and_matrix(source):
    mount, node = source
    class Evidence:
        provenance = dict(calibration_kind='estimate', accuracy_validated=False)
        def lookup(self, stamp, parent, child, clock):
            assert (stamp,parent,child,clock) == (12_345_678_901,'base_link','optical','capture')
            matrix = np.eye(4);matrix[2,3]=1.5
            return matrix.tolist()
    mount.capture_evidence=Evidence()
    assert mount.publish(12_345_678_901)
    assert not mount.buffer.calls
    assert node.messages[-1]['matrix'][2][3] == 1.5
    assert node.messages[-1]['accuracy_validated'] is False
