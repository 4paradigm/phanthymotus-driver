"""G1 native computational evidence without DDS, ROS or robot connections."""
import ast
from dataclasses import make_dataclass
import hashlib
import json
from pathlib import Path
import queue
import struct
import sys
import threading
from types import SimpleNamespace as NS
from uuid import uuid4

import pytest

import common.slam_input as slam_input
from unitree.g1.slam_odometry import MappingOdometry
from unitree.g1.sport_odometry import SportOdometry
from unitree.g1.pointcloud_utils import gravity_align_inplace, pack_slam_cloud


DEVICE = Path(__file__).resolve().parents[1] / 'unitree/g1/device.py'


def extract(namespace, *names):
    tree = ast.parse(DEVICE.read_text())
    selected = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name in names]
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(DEVICE), 'exec'), namespace)
    return [namespace[name] for name in names]


@pytest.fixture
def runtime(monkeypatch):
    class Message:
        def __init__(self, **values):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
            self.__dict__.update(values)

    class Node:
        def __init__(self, name):
            self.topics, self.order, self.qos = {}, [], {}

        def create_publisher(self, cls, topic, qos):
            messages = self.topics.setdefault(topic, [])
            self.qos[topic] = qos

            def publish(message):
                messages.append(message)
                self.order.append(topic)

            return NS(publish=publish, get_subscription_count=lambda: 1)

        def get_logger(self):
            return NS(info=lambda _: None, warn=lambda _: None)

    class Thread:
        def __init__(self, **kwargs):
            pass

        def start(self):
            pass

    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=Message, UInt8MultiArray=Message))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(PointCloud2=Message, PointField=Message))
    subscriptions = []

    def subscribe(topic, cls):
        sub = NS(topic=topic, Init=lambda callback, depth: subscriptions.append((topic, callback, depth)))
        return sub

    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.core.channel', NS(ChannelSubscriber=subscribe))
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.idl.nav_msgs.msg.dds_', NS(Odometry_=Message))
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.idl.sensor_msgs.msg.dds_', NS(PointCloud2_=Message, Imu_=Message))
    clock = NS(monotonic=lambda: 100., time_ns=lambda: 900_000_000_000,
               monotonic_ns=lambda: 800_000_000_000)
    monkeypatch.setattr(slam_input, 'time', clock)
    namespace = dict(Node=Node, String=Message, queue=queue, time=clock, json=json,
                     __file__=str(DEVICE), Path=Path,
                     struct=struct, uuid4=uuid4, threading=NS(Thread=Thread, Lock=threading.Lock),
                     _LOW_LAT_QOS=object(), LIDAR_CLOUD_INTERVAL=.05,
                     QoSProfile=lambda **values: NS(**values),
                     ReliabilityPolicy=NS(BEST_EFFORT='best_effort'),
                     HistoryPolicy=NS(KEEP_LAST='keep_last'),
                     DurabilityPolicy=NS(VOLATILE='volatile'),
                     gravity_align_inplace=gravity_align_inplace, pack_slam_cloud=pack_slam_cloud,
                     input_declaration=slam_input.input_declaration,
                     validate_extrinsic_candidate=slam_input.validate_extrinsic_candidate,
                     MappingOdometry=MappingOdometry, SportOdometry=SportOdometry)
    if hasattr(slam_input, 'native_frame_metadata'):
        namespace['native_frame_metadata'] = slam_input.native_frame_metadata
    lidar, plugin = extract(namespace, '_LidarNode', 'LidarPlugin')
    return NS(namespace=namespace, node=lidar('/alpha/lidar/cloud'), plugin=plugin, clock=clock, subscriptions=subscriptions)


def cloud(*, padded=False, bigendian=False):
    data = struct.pack(('>' if bigendian else '<')+'fffHBB', 1., 2., 3., 54321, 17, 29)
    if padded:
        data += b'padding!'
    return NS(header=NS(stamp=NS(sec=27, nanosec=123_456_789), frame_id='livox_native'),
              height=1, width=1, point_step=16, row_step=len(data), is_bigendian=bigendian,
              is_dense=False, data=bytearray(data), fields=[
                  NS(name='x', offset=0, datatype=7, count=1),
                  NS(name='y', offset=4, datatype=7, count=1),
                  NS(name='z', offset=8, datatype=7, count=1),
                  NS(name='reflectivity', offset=12, datatype=4, count=1),
                  NS(name='line', offset=14, datatype=2, count=1),
                  NS(name='tag', offset=15, datatype=2, count=1)])


def drain(node):
    node._cloud_queue.put_nowait(None)
    node._process_loop()


@pytest.mark.parametrize('bigendian', [False, True])
def test_padded_display_with_rotation_keeps_native_stream_and_channels(runtime, bigendian):
    node = runtime.node
    node._imu_roll, node._imu_pitch = .3, -.2
    source = cloud(padded=True, bigendian=bigendian)
    original = bytes(source.data)
    for tick in (100., 101.):
        runtime.clock.monotonic = lambda: tick
        node._on_cloud(source)
    drain(node)
    native = node.topics['/alpha/lidar/cloud_native']
    assert len(native) == 2
    expected = struct.pack('<fffHBB', 1., 2., 3., 54321, 17, 29)
    assert all(bytes(frame.data) == expected for frame in native)
    assert len(node.topics['/alpha/lidar/cloud_native/metadata']) == 2
    display = node.topics['/alpha/lidar/cloud']
    assert len(display) == 2
    rotated = gravity_align_inplace(expected, 16, 1, .3, -.2)
    assert all(bytes(frame.data) == struct.pack('<II', 16, 1) + rotated for frame in display)
    assert bytes(source.data) == original


def test_display_failure_drops_only_that_display_frame(runtime):
    node = runtime.node
    publish = node._cloud_pub.publish
    calls = []

    def fail_once(msg):
        calls.append(msg)
        if len(calls) == 1:
            raise RuntimeError('display transport failed')
        publish(msg)

    node._cloud_pub.publish = fail_once
    for tick in (100., 101.):
        runtime.clock.monotonic = lambda: tick
        node._on_cloud(cloud())
    drain(node)
    assert len(node.topics['/alpha/lidar/cloud_native']) == 2
    assert len(node.topics['/alpha/lidar/cloud_native/metadata']) == 2
    assert len(node.topics['/alpha/lidar/cloud']) == 1
    assert node._worker_count == 2


def test_rotation_failure_cannot_interrupt_following_native_frame(runtime):
    node = runtime.node
    runtime.namespace['gravity_align_inplace'] = lambda *args: (_ for _ in ()).throw(ValueError('bad display layout'))
    for tick in (100., 101.):
        runtime.clock.monotonic = lambda: tick
        node._on_cloud(cloud())
    drain(node)
    assert len(node.topics['/alpha/lidar/cloud_native']) == 2
    assert len(node.topics['/alpha/lidar/cloud_native/metadata']) == 2
    assert not node.topics['/alpha/lidar/cloud']
    assert node._display_error == 'bad display layout'


def test_display_layout_rejection_keeps_native_fields_and_next_display_frame(runtime):
    source = cloud()
    source.fields[0].name, source.fields[2].name = 'z', 'x'
    runtime.node._on_cloud(source)
    runtime.clock.monotonic = lambda: 101.
    runtime.node._on_cloud(cloud())
    drain(runtime.node)
    native = runtime.node.topics['/alpha/lidar/cloud_native']
    assert len(native) == 2
    assert native[0].fields[0].name == 'z' and native[0].fields[2].name == 'x'
    assert len(runtime.node.topics['/alpha/lidar/cloud']) == 1


def test_padded_display_is_normalized_without_native_subscribers(runtime):
    node = runtime.node
    node._native_pub.get_subscription_count = lambda: 0
    node._imu_roll, node._imu_pitch = .3, -.2
    source = cloud(padded=True, bigendian=True)
    node._on_cloud(source)
    drain(node)
    expected = gravity_align_inplace(pack_slam_cloud(source), 16, 1, .3, -.2)
    assert bytes(node.topics['/alpha/lidar/cloud'][0].data) == struct.pack('<II', 16, 1) + expected
    assert not node.topics['/alpha/lidar/cloud_native']


def test_display_does_not_rotate_sdk_buffer_before_native_consumer_connects(runtime):
    node = runtime.node
    node._native_pub.get_subscription_count = lambda: 0
    node._imu_roll, node._imu_pitch = .3, -.2
    source = cloud()
    original = bytes(source.data)
    node._on_cloud(source)
    drain(node)
    assert bytes(source.data) == original
    node._native_pub.get_subscription_count = lambda: 1
    runtime.clock.monotonic = lambda: 101.
    node._on_cloud(source)
    drain(node)
    assert bytes(node.topics['/alpha/lidar/cloud_native'][0].data) == original


@pytest.mark.parametrize('padded,bigendian', [(False, False), (True, True)])
def test_native_pointcloud_preserves_frozen_header_layout_channels_and_bytes(runtime, padded, bigendian):
    source = cloud(padded=padded, bigendian=bigendian)
    original = bytes(source.data)
    packed = struct.pack('<fffHBB', 1., 2., 3., 54321, 17, 29)
    runtime.node._on_cloud(source)
    source.header.stamp.sec = 999
    source.header.frame_id = 'reused'
    source.fields[3].name = 'reused'
    source.data[:] = b'\0' * len(source.data)
    source.row_step = 999
    source.is_dense = True
    runtime.clock.time_ns = lambda: 999_000_000_000
    runtime.clock.monotonic_ns = lambda: 888_000_000_000
    drain(runtime.node)
    output = runtime.node.topics.get('/alpha/lidar/cloud_native', [])
    assert len(output) == 1
    raw = output[0]
    assert (raw.header.stamp.sec, raw.header.stamp.nanosec, raw.header.frame_id) == (27, 123_456_789, 'livox_native')
    assert (raw.height, raw.width, raw.point_step, raw.row_step) == (1, 1, 16, 16)
    assert raw.is_bigendian is False and raw.is_dense is False
    assert [(field.name, field.offset, field.datatype, field.count) for field in raw.fields] == [
        ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1),
        ('reflectivity', 12, 4, 1), ('line', 14, 2, 1), ('tag', 15, 2, 1)]
    assert bytes(raw.data) == packed
    packet = json.loads(runtime.node.topics['/alpha/lidar/cloud_native/metadata'][0].data)
    assert packet['stamp_ns'] == 27_123_456_789
    assert packet['frame_id'] == 'livox_native'
    assert packet['data_sha256'] == hashlib.sha256(packed).hexdigest()
    assert packet['clock_id'] is None and packet['clock_epoch'] is None
    assert packet['mapping']['verified'] is False
    assert packet['source_generation']
    assert packet['received_ns'] == 900_000_000_000
    assert packet['received_monotonic_ns'] == 800_000_000_000
    assert runtime.node.order == ['/alpha/lidar/cloud_native', '/alpha/lidar/cloud_native/metadata', '/alpha/lidar/cloud']


def test_gravity_alignment_only_changes_display_and_never_raw_computational_bytes(runtime):
    source = cloud()
    original = bytes(source.data)
    runtime.node._imu_roll, runtime.node._imu_pitch = .3, -.2
    runtime.node._on_cloud(source)
    drain(runtime.node)
    native = runtime.node.topics.get('/alpha/lidar/cloud_native', [])
    assert len(native) == 1
    assert bytes(native[0].data) == original
    display = bytes(runtime.node.topics['/alpha/lidar/cloud'][0].data)
    assert display[:8] == struct.pack('<II', 16, 1)
    assert display[8:20] != original[:12]
    assert display[20:] == original[12:]
    assert bytes(source.data) == original


def test_native_branch_failure_keeps_display_running(runtime):
    source = cloud()
    del source.fields
    runtime.node._on_cloud(source)
    drain(runtime.node)
    assert len(runtime.node.topics['/alpha/lidar/cloud']) == 1
    assert not runtime.node.topics.get('/alpha/lidar/cloud_native', [])


def test_native_publisher_failure_cannot_kill_display_worker(runtime):
    assert hasattr(runtime.node, '_native_pub')
    runtime.node._native_pub.publish = lambda _: (_ for _ in ()).throw(RuntimeError('optional transport failed'))
    runtime.node._on_cloud(cloud())
    drain(runtime.node)
    assert len(runtime.node.topics['/alpha/lidar/cloud']) == 1


def test_native_subscription_query_failure_keeps_old_display_alive(runtime):
    runtime.node._native_pub.get_subscription_count = lambda: (_ for _ in ()).throw(RuntimeError('optional query failed'))
    runtime.node._on_cloud(cloud())
    drain(runtime.node)
    assert len(runtime.node.topics['/alpha/lidar/cloud']) == 1


def test_native_publisher_creation_failure_does_not_prevent_display_node_start(runtime, monkeypatch):
    base = runtime.namespace['Node'].create_publisher

    def create(node, cls, topic, qos):
        if topic.endswith('cloud_native'):
            raise RuntimeError('optional native publisher unavailable')
        return base(node, cls, topic, qos)

    monkeypatch.setattr(runtime.namespace['Node'], 'create_publisher', create)
    node = runtime.namespace['_LidarNode']('/alpha/lidar/cloud')
    node._on_cloud(cloud())
    drain(node)
    assert len(node.topics['/alpha/lidar/cloud']) == 1
    assert node._native_error == 'optional native publisher unavailable'


def test_shared_worker_remains_bounded_and_retains_existing_throttle_and_drop_behavior(runtime):
    node = runtime.node
    assert node._cloud_queue.maxsize == 10
    node._on_cloud(cloud())
    node._on_cloud(cloud())
    assert node._cloud_queue.qsize() == 1
    for index in range(1, 11):
        runtime.clock.monotonic = lambda index=index: 100.+index
        node._on_cloud(cloud())
    assert node._cloud_queue.qsize() == 10
    assert node._cb_count == 12 and node._cb_accepted == 11 and node._cb_dropped == 1


def test_without_native_subscribers_no_raw_conversion_or_metadata_cost_is_added(runtime):
    assert hasattr(runtime.node, '_native_pub')
    runtime.node._native_pub.get_subscription_count = lambda: 0
    source = cloud()
    del source.header
    del source.fields
    runtime.node._on_cloud(source)
    drain(runtime.node)
    assert len(runtime.node.topics['/alpha/lidar/cloud']) == 1
    assert not runtime.node.topics['/alpha/lidar/cloud_native']
    assert not runtime.node.topics['/alpha/lidar/cloud_native/metadata']


def test_native_history_is_two_frames_without_changing_display_qos(runtime):
    assert runtime.node.qos['/alpha/lidar/cloud'] is runtime.namespace['_LOW_LAT_QOS']
    native_qos = runtime.node.qos.get('/alpha/lidar/cloud_native')
    assert getattr(native_qos, 'depth', None) == 2
    assert native_qos.history == 'keep_last'
    assert native_qos.reliability == 'best_effort'
    assert native_qos.durability == 'volatile'
    assert runtime.node.qos['/alpha/lidar/cloud_native/metadata'] is native_qos


def test_native_port_declarations_match_info_and_keep_original_display_first(runtime):
    nodes = []
    plugin = runtime.plugin({}, 'alpha', NS(add_node=nodes.append))
    expected = [{'topic': '/alpha/lidar/cloud', 'format': 'sensor/pointcloud'},
                {'topic': '/alpha/lidar/cloud_native', 'format': 'sensor/pointcloud2'}]
    assert plugin.get_tools()[0]['topic_out'] == expected
    info = plugin.dispatch('info', {})
    assert info['topic_out'] == expected
    declaration = info['input_metadata'][0]
    assert declaration['topic'] == '/alpha/lidar/cloud_native'
    assert declaration['frame_metadata_topic'] == '/alpha/lidar/cloud_native/metadata'
    assert declaration['source_topic'] == 'rt/utlidar/cloud_livox_mid360'
    assert declaration['source_type'] == 'sensor_msgs.msg.dds_.PointCloud2_'
    for field in ('stamp_source', 'clock_id', 'length_unit', 'reference_point',
                  'cloud_semantics', 'timing_model', 'observation_model', 'extrinsic', 'frame_id'):
        assert declaration[field] is None
    plugin._node._on_cloud(cloud())
    observed = plugin.dispatch('info', {})['input_metadata'][0]
    assert observed['frame_id'] == 'livox_native'
    assert observed['source_header'] == {'stamp': {'sec': 27, 'nanosec': 123_456_789}, 'frame_id': 'livox_native'}
    observed['source_header']['stamp']['sec'] = 999
    unchanged = plugin.dispatch('info', {})['input_metadata'][0]
    assert unchanged['source_header']['stamp']['sec'] == 27


def test_info_observes_header_without_native_subscribers_or_payload_metadata_work(runtime):
    plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._native_pub.get_subscription_count = lambda: 0

    def unexpected_hash(*args, **kwargs):
        raise AssertionError('no raw consumer requested a metadata packet')

    runtime.namespace['native_frame_metadata'] = unexpected_hash
    source = cloud()
    plugin._node._on_cloud(source)
    source.header.stamp.sec = 999
    source.header.frame_id = 'reused'
    declaration = plugin.dispatch('info', {})['input_metadata'][0]
    assert declaration['frame_id'] == 'livox_native'
    assert declaration['source_header'] == {'frame_id': 'livox_native',
                                             'stamp': {'sec': 27, 'nanosec': 123_456_789}}
    assert plugin._node._native_error is None
    assert not plugin._node.topics['/alpha/lidar/cloud_native']
    assert not plugin._node.topics['/alpha/lidar/cloud_native/metadata']
    runtime.clock.monotonic = lambda: 101.
    del source.header
    plugin._node._on_cloud(source)
    declaration = plugin.dispatch('info', {})['input_metadata'][0]
    assert declaration['frame_id'] is None and declaration['source_header'] is None
    drain(plugin._node)
    assert len(plugin._node.topics['/alpha/lidar/cloud']) == 2


def sdk_dataclass(name, **values):
    """Build a ROS-free double of the checked-in generated unitree_go DDS dataclass."""
    path = DEVICE.parent / 'unitree_sdk2py/idl/unitree_go/msg/dds_' / ('_'+name+'.py')
    cls = next(node for node in ast.parse(path.read_text()).body if isinstance(node, ast.ClassDef))
    names = [node.target.id for node in cls.body if isinstance(node, ast.AnnAssign)]
    return make_dataclass(name, names)(**values)


def sport_state():
    imu = sdk_dataclass('IMUState_', quaternion=[.1, .2, .3, .4], gyroscope=[1., 2., 3.],
                        accelerometer=[4., 5., 6.], rpy=[.7, .8, .9], temperature=37)
    point = sdk_dataclass('PathPoint_', t_from_start=1., x=2., y=3., yaw=4., vx=5., vy=6., vyaw=7.)
    return sdk_dataclass('SportModeState_', stamp=sdk_dataclass('TimeSpec_', sec=7, nanosec=45),
                         error_code=0, imu_state=imu, mode=2, progress=.75, gait_type=3,
                         foot_raise_height=.12, position=[8., 9., 10.], body_height=.8,
                         velocity=[.1, .2, .3], yaw_speed=.4, range_obstacle=[1., 2., 3., 4.],
                         foot_force=[11, 12, 13, 14], foot_position_body=[.5]*12,
                         foot_speed_body=[.6]*12, path_point=[point]*10)


def test_loco_info_retains_complete_native_facts_without_claiming_a_slam_pose(runtime):
    node_cls, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    source = sport_state()
    plugin._node._on_odom(source)
    legacy = json.loads(plugin._node.topics['/alpha/loco/state'][0].data)
    assert legacy == {
        'mode': 2, 'gait_type': 3, 'body_height': .8, 'position': [8., 9., 10.],
        'velocity': [.1, .2, .3], 'yaw_speed': .4, 'foot_force': [11, 12, 13, 14],
        'imu': {'quaternion': [.1, .2, .3, .4], 'gyroscope': [1., 2., 3.],
                'accelerometer': [4., 5., 6.], 'rpy': [.7, .8, .9]},
    }
    source.position[:] = [999.]*3
    source.stamp.sec = 999
    source.imu_state.quaternion[:] = [999.]*4
    info = plugin.dispatch('info', {'_tool_name': 'loco_state'})
    assert info['topic_out'] == [{'topic': '/alpha/loco/state', 'format': 'data/json'},
                                 {'topic': '/alpha/slam/odom', 'format': 'state/odom'}]
    assert plugin.get_tools()[0]['topic_out'] == info['topic_out']
    assert info.get('slam_pose', {}).get('available') is False
    assert set(info['slam_pose']['missing_facts']) >= {'measurement_stamp_semantics',
                                                    'clock_domain_and_epoch',
                                                    'pose_reference_frame', 'tracking_quality'}
    evidence = info['native_snapshot']
    assert evidence['source_topic'] == 'rt/odommodestate'
    assert evidence['source_type'] == 'unitree_go.msg.dds_.SportModeState_'
    assert evidence['received_ns'] == 900_000_000_000
    assert evidence['received_monotonic_ns'] == 800_000_000_000
    assert evidence['native'] == {
        'stamp': {'sec': 7, 'nanosec': 45}, 'error_code': 0,
        'imu_state': {'quaternion': [.1, .2, .3, .4], 'gyroscope': [1., 2., 3.],
                      'accelerometer': [4., 5., 6.], 'rpy': [.7, .8, .9], 'temperature': 37},
        'mode': 2, 'progress': .75, 'gait_type': 3, 'foot_raise_height': .12,
        'position': [8., 9., 10.], 'body_height': .8, 'velocity': [.1, .2, .3], 'yaw_speed': .4,
        'range_obstacle': [1., 2., 3., 4.], 'foot_force': [11, 12, 13, 14],
        'foot_position_body': [.5]*12, 'foot_speed_body': [.6]*12,
        'path_point': [{'t_from_start': 1., 'x': 2., 'y': 3., 'yaw': 4., 'vx': 5., 'vy': 6., 'vyaw': 7.}]*10,
    }
    odom_interface = info['odom_interface']
    assert odom_interface['schema'] == 'motus.odom/1'
    assert odom_interface['frame'] == 'world'
    assert odom_interface['provides'] == []
    assert odom_interface['rate_hz'] == 0.
    assert odom_interface['pose_drift'] == 'unbounded'
    assert odom_interface['slam_pose']['profile'] == 'motus_slam/1'
    assert odom_interface['slam_pose']['available'] is False
    from common.odom import parse_interface
    parse_interface(odom_interface)
    from common.spatial_slam import SpatialSlamError, validate_slam_capability
    with pytest.raises(SpatialSlamError, match='availability'):
        validate_slam_capability(odom_interface)
    assert info['mapping_odom']['available'] is False
    assert info['mapping_odom']['sample'] is None
    assert info['regular_odom']['sample']['vendor']['source_topic'] == 'rt/odommodestate'
    assert info['regular_odom']['sample']['vendor']['motus_slam']['status'] == 'unavailable'
    assert info['regular_odom']['input_metadata'][0]['source_topic'] == 'rt/odommodestate'
    assert info['regular_odom']['sample']['pose'] is None


@pytest.mark.parametrize('stamp', [(0, 0), (9, 1_000_000_000), (-1, 0)])
def test_invalid_cloud_stamp_is_not_replaced_with_receipt_time(runtime, stamp):
    source = cloud()
    source.header.stamp.sec, source.header.stamp.nanosec = stamp
    runtime.node._on_cloud(source)
    drain(runtime.node)
    raw = runtime.node.topics['/alpha/lidar/cloud_native'][0]
    assert (raw.header.stamp.sec, raw.header.stamp.nanosec) == stamp
    packet = json.loads(runtime.node.topics['/alpha/lidar/cloud_native/metadata'][0].data)
    assert packet['stamp_ns'] is None
    assert packet['clock_epoch'] is None and packet['mapping']['verified'] is False


def test_time_rollback_and_frame_reset_do_not_invent_clock_epoch(runtime):
    for i, (sec, frame) in enumerate([(27, 'a'), (27, 'a'), (26, 'a'), (1, 'b')]):
        runtime.clock.monotonic = lambda: 100. + i
        source = cloud()
        source.header.stamp.sec, source.header.frame_id = sec, frame
        runtime.node._on_cloud(source)
    drain(runtime.node)
    packets = [json.loads(m.data) for m in runtime.node.topics['/alpha/lidar/cloud_native/metadata']]
    assert [p['stamp_ns'] for p in packets] == [27_123_456_789, 27_123_456_789, 26_123_456_789, 1_123_456_789]
    assert all(p['clock_epoch'] is None and p['clock_id'] is None for p in packets)
    assert len({p['source_generation'] for p in packets}) == 1
    assert runtime.plugin({}, 'beta', NS(add_node=lambda _: None))._node._source_generation != packets[0]['source_generation']


def test_generated_cloud_against_perception_layout_and_admission(runtime, monkeypatch):
    import os
    import importlib
    import numpy as np
    root = os.environ.get('PERCEPTION_REFERENCE_ROOT')
    if not root:
        pytest.skip('set PERCEPTION_REFERENCE_ROOT to the read-only platform checkout')
    monkeypatch.syspath_prepend(root)
    consumer = Path(root) / 'perception/plugins/pointcloud_slam_ros.py'
    tree = ast.parse(consumer.read_text())
    decode = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'decode_cloud')
    callback = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                    for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_cloud')
    ns = dict(np=np, PointField=NS(FLOAT32=7), hashlib=hashlib)
    exec(compile(ast.Module(body=[decode, callback], type_ignores=[]), str(consumer), 'exec'), ns)
    runtime.node._on_cloud(cloud(padded=True, bigendian=True))
    drain(runtime.node)
    raw = runtime.node.topics['/alpha/lidar/cloud_native'][0]
    stamp, frame, points = ns['decode_cloud'](raw, max_points=10)
    assert (stamp, frame) == (27_123_456_789, 'livox_native')
    assert points.tolist() == [[1., 2., 3.]]
    admitted = []
    target = NS(_stopped=lambda: False, _lock=threading.Lock(), _wake=NS(set=lambda: None),
                _receipt_clock=lambda: 1., _measurement_now=lambda: None,
                stream=NS(max_cloud_points=10, input_contract=True,
                          cloud=lambda *a, **kw: admitted.append((a, kw)), last_error=None))
    ns['_cloud'](target, raw)
    assert len(admitted) == 1 and target.stream.last_error is None
    packet = json.loads(runtime.node.topics['/alpha/lidar/cloud_native/metadata'][0].data)
    assert admitted[0][1]['data_sha256'] == packet['data_sha256']
    contract = importlib.import_module('perception.plugins.pointcloud_slam_input')
    with pytest.raises(ValueError, match='clock_id'):
        contract.frame_identity(packet, clock_id='unknown', payload=True)
    plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None))
    decl = plugin._cloud_tool()['input_metadata'][0]
    with pytest.raises(contract.InputContractError) as error:
        contract.resolve_inputs({decl['topic']: decl},
                                {'/alpha/loco/odom': 'state/odom', decl['topic']: 'sensor/pointcloud2'})
    assert any(d['field'] == 'input_metadata' for d in error.value.diagnostics)


def test_loco_reports_officially_documented_facts_without_claiming_clock_or_tracking(runtime):
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    info = plugin_cls({}, 'alpha', NS(add_node=lambda _: None)).dispatch('info', {})
    pose = info['slam_pose']
    assert 'quaternion_order' not in pose['missing_facts']
    facts = pose['documented_facts']
    assert facts['length_unit'] == 'm'
    assert facts['quaternion_order'] == 'wxyz'
    assert facts['pose_reference'] == 'robot_base_center'
    assert facts['axes'] == ['forward', 'left', 'up']
    assert facts['requires_high_level_motion_control'] is True
    assert facts['source'].endswith('/odometer_service_interface')
    assert pose['available'] is False
    assert 'clock_domain_and_epoch' in pose['missing_facts']
    assert info['mapping_odom']['available'] is False


def test_mapping_port_subscribes_official_source_and_publishes_separately(runtime):
    from test_g1_mapping_odom import message
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    tool = plugin.get_tools()[0]
    assert tool['topic_out'] == [
        {'topic': '/alpha/loco/state', 'format': 'data/json'},
        {'topic': '/alpha/slam/odom', 'format': 'state/odom'}]
    assert any(t == 'rt/unitree/slam_mapping/odom' for t, _, _ in runtime.subscriptions)
    assert not any('relocation' in t for t, _, _ in runtime.subscriptions)
    callback = next(cb for t, cb, _ in runtime.subscriptions if t == 'rt/unitree/slam_mapping/odom')
    callback(message())
    emitted = json.loads(plugin._node.topics['/alpha/slam/mapping/odom'][0].data)
    assert emitted['pose']['position'] == [1.25, -2., .3]
    assert emitted['vendor']['motus_slam']['status'] == 'unavailable'
    assert not plugin._node.topics['/alpha/loco/state']
    plugin._node._on_odom(sport_state())
    assert len(plugin._node.topics['/alpha/loco/state']) == 1
    info = plugin.dispatch('info', {})
    assert info['topic_out'] == tool['topic_out']
    assert info['mapping_odom']['sample'] == emitted
    assert info['input_metadata'][0]['frame_id'] is None
    assert info['input_metadata'][0]['source_topic'] == 'rt/odommodestate'
    info['mapping_odom']['sample']['pose']['position'][0] = 99
    assert plugin.dispatch('info', {})['mapping_odom']['sample']['pose']['position'][0] == 1.25
    assert plugin.dispatch('info', {'_tool_name': 'loco_motion_state'})['topic_out'] == [
        {'topic': '/alpha/loco/motion_state', 'format': 'data/json'}]


def test_invalid_mapping_message_clears_pose_and_preserves_legacy_json(runtime):
    from test_g1_mapping_odom import message
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._on_mapping_odom(message())
    plugin._node._on_mapping_odom(NS())
    packets = plugin._node.topics['/alpha/slam/mapping/odom']
    assert json.loads(packets[-1].data)['pose'] is None
    assert plugin.dispatch('info', {})['mapping_odom']['sample']['pose'] is None
    plugin._node._on_odom(sport_state())
    assert json.loads(plugin._node.topics['/alpha/loco/state'][0].data)['position'] == [8., 9., 10.]


def test_mapping_subscription_failure_is_visible_without_breaking_legacy(runtime, monkeypatch):
    def unavailable(*args):
        raise RuntimeError('DDS mapping unavailable')
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.core.channel', NS(ChannelSubscriber=unavailable))
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    assert 'DDS mapping unavailable' in plugin.dispatch('info', {})['mapping_odom']['error']
    plugin._node._on_odom(sport_state())
    assert len(plugin._node.topics['/alpha/loco/state']) == 1


def test_mapping_publisher_failure_cannot_break_old_json(runtime):
    from test_g1_mapping_odom import message
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._mapping_pub.publish = lambda _: (_ for _ in ()).throw(RuntimeError('transport unavailable'))
    plugin._node._on_mapping_odom(message())
    assert plugin.dispatch('info', {})['mapping_odom']['error'] == 'transport unavailable'
    plugin._node._on_odom(sport_state())
    assert len(plugin._node.topics['/alpha/loco/state']) == 1


def test_actual_mapping_packets_and_metadata_are_rejected_by_perception(runtime, monkeypatch):
    import os
    import importlib
    import numpy as np
    from test_g1_mapping_odom import message
    root = os.environ.get('PERCEPTION_REFERENCE_ROOT')
    if not root:
        pytest.skip('set PERCEPTION_REFERENCE_ROOT to the read-only platform checkout')
    monkeypatch.syspath_prepend(root)
    parser = importlib.import_module('perception.plugins.pointcloud_slam_odom')
    contract = importlib.import_module('perception.plugins.pointcloud_slam_input')
    stream_cls = importlib.import_module('perception.plugins.pointcloud_slam_stream').SlamStream
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    # The dummy consumer setup deliberately has no admitted input contract.
    # Even this permissive harness must reject every real producer packet.
    accepted = []
    stream = stream_cls(cloud_frame='livox_native', child_frame='source_imu',
                        clock_id='test_only_clock', child_from_cloud=np.eye(4),
                        max_gap_ms=250, max_age_s=1, max_pending=2, max_cloud_points=10,
                        on_frame=lambda *a: accepted.append(a))
    for msg in (message(), message(), message(sec=26), NS(), message(sec=28, frame='changed')):
        plugin._node._on_mapping_odom(msg)
    for emitted in plugin._node.topics['/alpha/slam/mapping/odom']:
        packet = json.loads(emitted.data)
        with pytest.raises(ValueError):
            parser.decode_slam_odom(packet)
        stream.odom(packet, now=1.)
        with pytest.raises(ValueError, match='odometry stale or unavailable'):
            stream.cloud(27123456789, 'livox_native', [[1., 2., 3.]], now=1.)
    assert accepted == []
    assert stream.status(now=100.)['admitted_frames'] == 0
    assert stream.status(now=100.)['odometry'] != 'tracking'
    pose = plugin.dispatch('info', {})['input_metadata'][0]
    lidar = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None))
    lidar._node._on_cloud(cloud())
    cloud_decl = lidar._cloud_tool()['input_metadata'][0]
    metadata = {pose['topic']: pose, cloud_decl['topic']: cloud_decl}
    for experimental in (False, True):
        with pytest.raises(contract.InputContractError) as error:
            contract.resolve_inputs(metadata, {pose['topic']: 'state/odom',
                                               cloud_decl['topic']: 'sensor/pointcloud2'},
                                    experimental=experimental)
        fields = {d['field'] for d in error.value.diagnostics}
        assert {'clock_id', 'stamp_source', 'extrinsic', 'clock.relation_id'} <= fields


def test_unverified_lidar_extrinsic_candidate_is_visible_but_not_admissible(runtime):
    matrix = [[1., 0., 0., 0.1], [0., 1., 0., -0.2],
              [0., 0., 1., 0.3], [0., 0., 0., 1.]]
    plugin = runtime.plugin({
        'slam_extrinsic_candidate': {
            'parent_frame': 'source_imu', 'child_frame': 'livox_native',
            'source': 'operator_candidate', 'version': 'unverified-v1',
            'matrix': matrix,
        },
    }, 'alpha', NS(add_node=lambda _: None))
    initial = plugin.dispatch('info', {})
    declaration = initial['input_metadata'][0]
    assert declaration['extrinsic'] is None
    candidate = declaration['extrinsic_candidate']
    assert candidate['matrix'] == matrix
    assert candidate['calibrated'] is False and candidate['verified'] is False
    assert initial['extrinsic_candidate_error'] is None
    plugin._node._on_cloud(cloud())
    observed = plugin.dispatch('info', {})['input_metadata'][0]
    assert observed['extrinsic_candidate']['child_frame'] == 'livox_native'
    assert observed['extrinsic'] is None


def test_candidate_frame_mismatch_diagnostic_does_not_stop_display(runtime):
    plugin = runtime.plugin({
        'slam_extrinsic_candidate': {
            'parent_frame': 'source_imu', 'child_frame': 'incorrect_frame',
            'source': 'operator_candidate', 'version': 'v1',
            'matrix': [[1, 0, 0, 0], [0, 1, 0, 0],
                       [0, 0, 1, 0], [0, 0, 0, 1]],
        },
    }, 'alpha', NS(add_node=lambda _: None))
    plugin._node._on_cloud(cloud())
    info = plugin.dispatch('info', {})
    assert 'candidate cloud frame differs' in info['extrinsic_candidate_error']
    assert info['input_metadata'][0]['extrinsic_candidate'] is None
    assert info['input_metadata'][0]['extrinsic'] is None
    drain(plugin._node)
    assert len(plugin._node.topics['/alpha/lidar/cloud']) == 1


def test_invalid_candidate_is_diagnostic_only_and_legacy_lidar_survives(runtime):
    plugin = runtime.plugin({
        'slam_extrinsic_candidate': {
            'parent_frame': 'source_imu', 'child_frame': 'livox_native',
            'source': 'operator', 'version': 'v1',
            'matrix': [[2, 0, 0, 0], [0, 1, 0, 0],
                       [0, 0, 1, 0], [0, 0, 0, 1]],
        },
    }, 'alpha', NS(add_node=lambda _: None))
    info = plugin.dispatch('info', {})
    assert 'not orthonormal' in info['extrinsic_candidate_error']
    assert info['input_metadata'][0]['extrinsic'] is None
    plugin._node._on_cloud(cloud())
    drain(plugin._node)
    assert len(plugin._node.topics['/alpha/lidar/cloud']) == 1


def test_slam_consumer_canonical_validator_never_admits_g1_packet(runtime):
    from common.spatial_slam import (
        SpatialSlamError, validate_input_port, validate_tracking_pose,
        validate_frame_identity,
    )
    from test_g1_mapping_odom import message
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._on_mapping_odom(message())
    packet = json.loads(plugin._node.topics['/alpha/slam/mapping/odom'][-1].data)
    with pytest.raises(SpatialSlamError, match='tracking pose'):
        validate_tracking_pose(packet)
    with pytest.raises(SpatialSlamError, match='clock_id'):
        validate_input_port(plugin.get_tools()[0]['input_metadata'][0])
    cloud_plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None))
    cloud_plugin._node._on_cloud(cloud())
    drain(cloud_plugin._node)
    meta_packet = json.loads(cloud_plugin._node.topics[
        '/alpha/lidar/cloud_native/metadata'][0].data)
    with pytest.raises(SpatialSlamError, match='clock_id'):
        validate_frame_identity(meta_packet, clock_id='unverified')


def test_mapping_odom_info_reports_quarantine_without_claiming_segment(runtime):
    from test_g1_mapping_odom import message
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._on_mapping_odom(message(sec=27))
    plugin._node._on_mapping_odom(message(sec=26))
    info = plugin.dispatch('info', {})
    observation = info['mapping_odom']['observation']
    assert observation['continuity_quarantined'] is True
    assert observation['possible_discontinuities'] == 1
    assert observation['last_accepted_frames'] == ['source_map', 'source_imu']
    assert observation['clock_epoch'] is None and observation['segment_id'] is None
    assert info['odom_interface']['slam_pose']['available'] is False
    assert info['mapping_odom']['sample']['pose'] is None


def test_user_verified_g1_urdf_mount_admitted_when_real_frames_match(runtime):
    plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None),
                            pose_frame_provider=lambda: 'torso_link')
    raw = cloud()
    raw.header.frame_id = 'mid360_link'
    plugin._node._on_cloud(raw)
    info = plugin.dispatch('info', {})
    d = info['input_metadata'][0]
    mount = d['extrinsic']
    assert mount is not None
    assert mount['parent_frame'] == 'torso_link'
    assert mount['child_frame'] == 'mid360_link'
    assert mount['calibrated'] is True
    assert mount['source'] == 'g1_model.urdf:operator_confirmed'
    assert mount['version'].startswith('sha256:')
    assert mount['matrix'][0][3] == pytest.approx(.0002835)
    assert mount['matrix'][1][3] == pytest.approx(.00003)
    assert mount['matrix'][2][3] == pytest.approx(.40618)
    assert mount['matrix'][0][2] == pytest.approx(.04013179, abs=2e-5)
    assert mount['calibration_evidence'] == 'operator_confirmed_prior_g1_test'
    assert info['urdf_mount']['calibrated'] is True
    assert info['urdf_calibration_error'] is None
    # An explicitly verified geometric transform is not proof of odometry or time.
    assert d['clock_id'] is None and d['stamp_source'] is None
    assert d['clock']['verified'] is False


def test_explicit_urdf_frame_aliases_use_fixed_imu_to_lidar_chain(runtime):
    plugin = runtime.plugin({
        'slam_urdf_frame_aliases': {
            'source_imu': 'imu_in_torso',
            'livox_native': 'mid360_link',
        },
    }, 'alpha', NS(add_node=lambda _: None),
       pose_frame_provider=lambda: 'source_imu')
    raw = cloud()
    raw.header.frame_id = 'livox_native'
    plugin._node._on_cloud(raw)
    info = plugin.dispatch('info', {})
    mount = info['input_metadata'][0]['extrinsic']
    assert mount['calibrated'] is True
    assert mount['parent_frame'] == 'source_imu'
    assert mount['child_frame'] == 'livox_native'
    assert mount['urdf_frames'] == {'parent': 'imu_in_torso',
                                    'child': 'mid360_link'}
    assert mount['matrix'][0][3] == pytest.approx(.0398735)
    assert mount['matrix'][1][3] == pytest.approx(.00227)
    assert mount['matrix'][2][3] == pytest.approx(.26826)
    assert info['urdf_calibration_error'] is None


def test_unknown_native_frames_do_not_inherit_verified_model_mount(runtime):
    plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None),
                            pose_frame_provider=lambda: 'source_imu')
    plugin._node._on_cloud(cloud())  # livox_native is not an URDF link
    info = plugin.dispatch('info', {})
    assert info['urdf_mount']['calibrated'] is True
    assert info['input_metadata'][0]['extrinsic'] is None
    assert 'URDF does not contain' in info['urdf_calibration_error']


def test_dynamic_joint_chain_cannot_be_misdeclared_as_static_mount(runtime):
    plugin = runtime.plugin({}, 'alpha', NS(add_node=lambda _: None),
                            pose_frame_provider=lambda: 'pelvis')
    raw = cloud()
    raw.header.frame_id = 'mid360_link'
    plugin._node._on_cloud(raw)
    info = plugin.dispatch('info', {})
    assert info['input_metadata'][0]['extrinsic'] is None
    assert 'not joined by a fixed chain' in info['urdf_calibration_error']


def test_conflicting_manual_candidate_cannot_override_calibrated_urdf(runtime):
    plugin = runtime.plugin({
        'slam_extrinsic_candidate': {
            'parent_frame': 'torso_link', 'child_frame': 'mid360_link',
            'source': 'operator_candidate', 'version': 'manual-v1',
            'matrix': [[1, 0, 0, 0], [0, 1, 0, 0],
                       [0, 0, 1, 0], [0, 0, 0, 1]],
        },
    }, 'alpha', NS(add_node=lambda _: None),
       pose_frame_provider=lambda: 'torso_link')
    raw = cloud()
    raw.header.frame_id = 'mid360_link'
    plugin._node._on_cloud(raw)
    info = plugin.dispatch('info', {})
    assert info['input_metadata'][0]['extrinsic'] is None
    assert info['input_metadata'][0]['extrinsic_candidate']['calibrated'] is False
    assert 'conflicts with calibrated G1 URDF' in info['urdf_calibration_error']


def test_urdf_model_is_packaged_in_g1_image():
    docker = (DEVICE.parent / 'Dockerfile').read_text()
    assert 'COPY resource/' in docker
    assert (DEVICE.parent / 'resource/g1_model.urdf').exists()


def test_explicit_alias_is_bound_to_exact_source_frame_name(runtime):
    plugin = runtime.plugin({
        'slam_urdf_frame_aliases': {
            'source_imu': 'imu_in_torso',
            'livox_native': 'mid360_link',
        },
    }, 'alpha', NS(add_node=lambda _: None),
       pose_frame_provider=lambda: 'source_imu')
    plugin._node._on_cloud(cloud())
    first = plugin.dispatch('info', {})['input_metadata'][0]
    assert first['extrinsic']['calibrated'] is True
    changed = cloud()
    changed.header.frame_id = 'another_livox_frame'
    plugin._node._on_cloud(changed)  # header observation precedes throttling
    info = plugin.dispatch('info', {})
    assert info['input_metadata'][0]['extrinsic'] is None
    assert 'URDF does not contain' in info['urdf_calibration_error']


def test_invalid_urdf_alias_config_never_admits_calibrated_input(runtime):
    plugin = runtime.plugin({
        'slam_urdf_frame_aliases': {'source_imu': 37},
    }, 'alpha', NS(add_node=lambda _: None),
       pose_frame_provider=lambda: 'torso_link')
    source = cloud()
    source.header.frame_id = 'mid360_link'
    plugin._node._on_cloud(source)
    info = plugin.dispatch('info', {})
    assert info['urdf_mount'] is None
    assert 'slam_urdf_frame_aliases' in info['urdf_calibration_error']
    assert info['input_metadata'][0]['extrinsic'] is None


def test_g1_regular_odom_publishes_slam_candidate_independently_of_vendor_mapping(runtime):
    from common.spatial_slam import SpatialSlamError, validate_tracking_pose
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({
        'slam_regular_odometry': {'frame_id': 'odom', 'child_frame_id': 'imu_in_torso'}
    }, 'alpha', NS(add_node=lambda _: None))
    source = sport_state()
    source.imu_state.quaternion = [1., 0., 0., 0.]
    plugin._node._on_odom(source)
    regular = json.loads(plugin._node.topics['/alpha/slam/odom'][0].data)
    assert regular['vendor']['source_topic'] == 'rt/odommodestate'
    assert regular['pose']['position'] == [8., 9., 10.]
    assert regular['pose']['quaternion'] == [0., 0., 0., 1.]
    assert regular['vendor']['motus_slam']['status'] == 'unavailable'
    assert regular['vendor']['motus_slam']['segment_id'] is None
    assert plugin.dispatch('info', {})['input_metadata'][0]['source_topic'] == 'rt/odommodestate'
    assert plugin._node.get_regular_child_frame() == 'imu_in_torso'
    assert plugin._node.topics['/alpha/slam/mapping/odom'] == []
    with pytest.raises(SpatialSlamError):
        validate_tracking_pose(regular)
    # Old visualization/state JSON is unaffected.
    legacy = json.loads(plugin._node.topics['/alpha/loco/state'][0].data)
    assert legacy['position'] == [8., 9., 10.]


def test_regular_odom_invalid_packet_does_not_break_old_loco_state(runtime):
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    legacy = sport_state()
    plugin._node._on_odom(legacy)
    regular = json.loads(plugin._node.topics['/alpha/slam/odom'][0].data)
    assert regular['pose'] is None
    assert regular['vendor']['motus_slam']['rejection'] == 'non_unit_imu_quaternion'
    assert json.loads(plugin._node.topics['/alpha/loco/state'][0].data)['position'] == [8., 9., 10.]
    assert plugin.dispatch('info', {})['regular_odom']['sample']['pose'] is None


def test_regular_odom_publish_failure_is_isolated_from_legacy_json(runtime):
    _, plugin_cls = extract(runtime.namespace, '_LocoStateNode', 'LocoStatePlugin')
    plugin = plugin_cls({}, 'alpha', NS(add_node=lambda _: None))
    plugin._node._sport_pub.publish = lambda _: (_ for _ in ()).throw(RuntimeError('not reachable'))
    plugin._node._on_odom(sport_state())
    assert 'not reachable' in plugin.dispatch('info', {})['regular_odom']['error']
    assert len(plugin._node.topics['/alpha/loco/state']) == 1
