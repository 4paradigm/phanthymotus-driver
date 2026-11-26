"""Run Tianyi pointcloud class methods with no ROS, robot or SDK installed.

Use AST extraction of this one class so the production device module does not
import unrelated ROS/servo dependencies on a developer laptop.
"""
import ast
import json
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace as NS
from uuid import uuid4

import pytest

BASE = Path(__file__).resolve().parents[1] / 'x-humanoid' / 'tianyi2.0'


class FakeString:
    def __init__(self, data=''):
        self.data = data


class Publisher:
    def __init__(self):
        self.samples = []

    def publish(self, msg):
        self.samples.append(msg)


@pytest.fixture
def plugin_class(monkeypatch):
    monkeypatch.syspath_prepend(str(BASE))
    source = BASE / 'device.py'
    module = ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
    classes = [item for item in module.body
               if isinstance(item, ast.ClassDef) and item.name == 'PointCloudPlugin']
    assert len(classes) == 1
    extracted = ast.Module(body=classes, type_ignores=[])
    ast.fix_missing_locations(extracted)
    class Node:
        def __init__(self, *args, **kwargs):
            self.subscriptions = []

        def create_subscription(self, *args):
            self.subscriptions.append(args)

        def create_publisher(self, *args):
            return Publisher()

    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg',
                        NS(PointCloud2=object, Image=object, CameraInfo=object, Imu=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(UInt8MultiArray=object))
    namespace = dict(json=json, String=FakeString, time=NS(monotonic=lambda: 100.), Node=Node,
                     uuid4=uuid4, threading=threading, QoSProfile=lambda **kwargs: NS(**kwargs),
                     ReliabilityPolicy=NS(BEST_EFFORT=1), HistoryPolicy=NS(KEEP_LAST=1),
                     DurabilityPolicy=NS(VOLATILE=1), _LOW_LAT_QOS=object(), _RELIABLE_QOS=object())
    exec(compile(extracted, str(source), 'exec'), namespace)
    return namespace['PointCloudPlugin']


@pytest.fixture
def plugin(plugin_class):
    executor = NS(add_node=lambda node: None)
    ros = NS(ctx_tianyi=None, ctx_core=None, executor_tianyi=executor, executor_core=executor)
    return plugin_class({}, 'robot', ros)


def native_cloud():
    return NS(header=NS(frame_id='raw_optical',
                        stamp=NS(sec=8, nanosec=23)),
              fields=[NS(name=k, offset=i*4, datatype=7, count=1)
                      for i, k in enumerate('xyz')],
              width=1, height=1, point_step=12, row_step=12,
              is_bigendian=False, data=struct.pack('<fff', 1., 2., 3.))


def test_native_output_and_digest_are_not_affected_by_display_transform(plugin):
    plugin.start()

    msg = native_cloud()
    plugin._forward_native(msg)
    assert plugin._native_pub.samples == [msg]
    assert plugin._native_pub.samples[0] is msg
    assert plugin._native_forwarded == 1
    packet = json.loads(plugin._native_metadata_pub.samples[0].data)
    assert packet['frame_id'] == msg.header.frame_id
    assert packet['stamp_ns'] == 8_000_000_023
    assert packet['source_generation'] == plugin._native_generation
    assert packet['data_sha256'] == plugin._native_evidence['data_sha256']
    assert packet['mapping']['verified'] is False

    assert [p['format'] for p in plugin._outputs()] == [
        'sensor/pointcloud', 'sensor/pointcloud2']
    declaration = plugin._native_info()['input_metadata'][0]
    assert declaration['clock']['verified'] is False
    assert declaration['stamp_source'] == 'unknown'


def test_invalid_native_frame_is_not_republished(plugin):
    plugin.start()
    bad = native_cloud()
    bad.header.stamp.sec = 0
    bad.header.stamp.nanosec = 0
    plugin._forward_native(bad)
    assert plugin._native_pub.samples == []
    assert plugin._native_metadata_pub.samples == []
    assert plugin._native_forwarded == 0
    assert 'zero ROS header' in plugin._native_error


def test_native_cloud_restart_clears_cadence_evidence_and_generation(plugin):
    plugin.start()
    original_publisher = plugin._native_pub
    plugin._forward_native(native_cloud())
    prior = json.loads(plugin._native_metadata_pub.samples[-1].data)
    assert plugin.dispatch('info', {})['native_status']['frames_forwarded'] == 1
    plugin.start()  # Repeated start while running must keep the same run.
    assert plugin.dispatch('info', {})['native_status']['frames_forwarded'] == 1
    plugin.stop()
    plugin.start()
    status = plugin.dispatch('info', {})['native_status']
    assert status['source_header'] is None and status['frames_forwarded'] == 0
    assert len(plugin._sub_node.subscriptions) == 4
    assert plugin._native_pub is original_publisher

    fresh = native_cloud()
    fresh.header.stamp.sec -= 1  # Source clock can restart behind its prior header.
    plugin._forward_native(fresh)  # Same monotonic time as last run, no rate delay.
    assert plugin._native_pub.samples[-1] is fresh
    packet = json.loads(plugin._native_metadata_pub.samples[-1].data)
    assert packet['source_generation'] != prior['source_generation']
    assert packet['stamp_ns'] == 7_000_000_023
    plugin._forward_native(native_cloud())
    assert len(plugin._native_pub.samples) == 2  # Cadence still applies within a run.


def test_pointcloud_card_rearms_after_stop_without_remaking_subscriptions(plugin):
    assert plugin.dispatch('start', {})['state'] == 'running'
    assert plugin.dispatch('stop', {})['state'] == 'idle'
    assert plugin.dispatch('start', {})['state'] == 'running'
    assert len(plugin._sub_node.subscriptions) == 4
    assert plugin.dispatch('info', {})['topic_out'][0]['format'] == 'sensor/pointcloud'


@pytest.mark.parametrize('sidecar_order', ['before', 'after', 'absent'])
def test_native_output_routes_through_core_and_maps_without_sidecar_dependency(
        plugin, monkeypatch, sidecar_order):
    import importlib.util
    companion = BASE.parents[2] / 'pr371-current-review'
    if not (companion / 'agent-core/src/api/slam_metadata.py').exists():
        pytest.skip('companion PR371 checkout is not present')
    monkeypatch.syspath_prepend(str(companion / 'perception'))
    from plugins.pointcloud_slam_input import resolve_inputs
    from plugins.pointcloud_slam_stream import SlamStream
    from plugins.pointcloud_normalization import decode_cloud
    from slam_pointcloud import native_cloud_evidence, frame_declaration

    spec = importlib.util.spec_from_file_location(
        'tianyi_companion_routing', companion / 'agent-core/src/api/slam_metadata.py')
    routing = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(routing)
    msg = native_cloud()
    source = plugin
    source._topic, source._format = '/display', 'sensor/pointcloud'
    source._native_topic, source._native_metadata_topic = '/native', '/metadata'
    source._native_evidence = native_cloud_evidence(msg)
    source._native_forwarded, source._native_error = 1, None
    pose_declaration = dict(schema='motus.slam_input/1', topic='/odom', format='state/odom')
    connections = [dict(fromCardId='camera', fromPortIdx='1', fromTopic='/native'),
                   dict(fromCardId='pose', fromPortIdx='0', fromTopic='/odom')]
    infos = dict(camera=source._native_info(), pose=dict(
        topic_out=[dict(topic='/odom', format='state/odom')], input_metadata=[pose_declaration]))
    selected = routing.collect_slam_inputs(connections, infos, lambda c: c['fromTopic'])
    assert selected['input_bindings'] == {'/native': 'sensor/pointcloud2', '/odom': 'state/odom'}
    contract = resolve_inputs(selected['input_metadata'], selected['input_bindings'])
    accepted = []
    stream = SlamStream(**contract['settings'], max_gap_ms=1, max_age_s=1,
                        max_pending=4, max_cloud_points=10, input_contract=contract,
                        on_frame=lambda *args: accepted.append(args))
    sample = dict(schema='motus.odom/1', stamp_ms=8000,
                  pose=dict(position=[0., 0., 0.], quaternion=[0., 0., 0., 1.]),
                  vendor=dict(motus_slam=dict(stamp_ns=8_000_000_023)))
    stream.odom(sample, now=100)
    packet = frame_declaration(source._native_evidence, 'test-generation')
    if sidecar_order == 'before':
        stream.cloud_metadata(packet, now=100)
    stamp, frame, points = decode_cloud(msg, max_points=10)
    stream.cloud(stamp, frame, points, now=100)
    assert len(accepted) == 1  # Never waits for the diagnostic sidecar.
    if sidecar_order == 'after':
        stream.cloud_metadata(packet, now=101)
    assert len(accepted) == 1  # Later diagnostics do not replay the cloud.
    assert accepted[0][0] == 8_000_000_023
    assert accepted[0][1].tolist() == [[1., 2., 3.]]
