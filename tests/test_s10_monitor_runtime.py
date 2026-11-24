"""Offline monitor contracts for the standalone reader and the deployed Pro bundle."""
import importlib.util
import json
import struct
import sys
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

import pytest
import yaml

from common.vendor_runtime import DriverBundle, make_handler
from deep_robotics.s10_pro.pointcloud import convert_pointcloud
from deep_robotics.s10_pro.reader import DEFAULT_SOURCES, SubscriptionReader, LatestEventWriter, normalize_control
from test_s10_pro_control import DRIVER as SOURCE, FakeNative, s10_ros


@pytest.fixture
def pro_bundle(s10_ros, monkeypatch):
    device, ros2 = s10_ros
    config = yaml.safe_load((SOURCE / 'config.yaml').read_text())
    native = FakeNative(config)
    closed = []
    monkeypatch.setattr(native, 'close', lambda: closed.append(True), raising=False)
    monkeypatch.setattr(device, 'BasicServerClient', lambda _config: native)
    bundle = DriverBundle(device.build_plugins(config, 'test', ros2))
    state = next(plugin for plugin in bundle.plugins if isinstance(plugin, device.S10StatePlugin))
    yield SimpleNamespace(bundle=bundle, device=device, nodes=state.nodes, closed=closed)
    if not closed:
        bundle.stop_all()



class Node:
    def __init__(self):
        self.callbacks = {}
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, qos):
        self.callbacks[topic] = callback
        return topic

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def test_four_sources_create_subscriptions_and_reject_commands():
    node, events = Node(), []
    reader = SubscriptionReader(node, DEFAULT_SOURCES, lambda name: name, events.append, object())
    reader.apply_control({'active': list(DEFAULT_SOURCES)})
    assert set(node.callbacks) == {'/BATTERY_DATA', '/JOINTS_DATA_10HZ', '/IMU', '/LIDAR/POINTS'}
    old = node.callbacks['/BATTERY_DATA']
    old({'data': [{'battery_level': 80}]})
    assert events[-1]['data']['data'][0]['battery_level'] == 80
    reader.apply_control({'active': ['imu']})
    count = len(events)
    old({'data': []})
    assert len(events) == count  # a canceled callback cannot revive a released stream
    reader.close()
    assert set(node.destroyed) == set(node.callbacks)
    with pytest.raises(ValueError):
        normalize_control({'active': ['motion_info']})
    with pytest.raises(ValueError):
        normalize_control({'active': [], 'action': 'stand_up'})


def test_newest_record_replaces_pending_backlog():
    import io
    writer = LatestEventWriter(io.StringIO(), autostart=False)
    for index in range(100):
        writer.submit({'key': 'imu', 'data': {'sequence': index}})
    assert writer.pending_count == 1
    writer.close()


def test_monitor_card_stop_keeps_the_shared_pro_streams_live(pro_bundle, monkeypatch):
    bundle, nodes = pro_bundle.bundle, pro_bundle.nodes
    monkeypatch.setattr(pro_bundle.device.time, 'time', lambda: 1000.)
    for name in ('battery', 'imu'):
        assert bundle.dispatch(name, {'action': 'start'})['state'] == 'running'
    nodes.robot.subscriptions['/BATTERY_DATA']({'data': [{'battery_level': 80}]})
    nodes.robot.subscriptions['/IMU']({'linear_acceleration': {'z': 9.8}})
    assert nodes.stream_status()['battery']['last_published'] == 1000.

    monkeypatch.setattr(pro_bundle.device.time, 'time', lambda: 1060.)
    assert bundle.dispatch('battery', {'action': 'stop'})['state'] == 'idle'
    nodes.robot.subscriptions['/IMU']({'linear_acceleration': {'z': 9.7}})
    status = nodes.stream_status()
    assert status['battery']['published'] == 1 and status['battery']['last_published'] == 1000.
    assert status['imu']['published'] == 2 and status['imu']['last_published'] == 1060.
    assert not pro_bundle.closed

    bundle.stop_all()
    assert pro_bundle.closed == [True]
    assert not nodes.robot.subscriptions


def test_cloud_preserves_axes_and_limits_valid_points():
    points = [(float(i), -2., 3.) for i in range(40005)] + [(float('nan'), 0., 0.)]
    message = {'header': {'frame_id': 'lidar_link'}, 'width': len(points), 'height': 1,
               'point_step': 12, 'row_step': len(points) * 12, 'is_bigendian': False,
               'fields': [{'name': axis, 'offset': index * 4, 'datatype': 7, 'count': 1}
                          for index, axis in enumerate('xyz')],
               'data': b''.join(struct.pack('<fff', *point) for point in points)}
    binary, info = convert_pointcloud(message)
    assert struct.unpack_from('<II', binary) == (12, 40000)
    assert len(binary) == 8 + 40000 * 12
    assert struct.unpack_from('<fff', binary, 8) == (0., -2., 3.)
    assert info['valid_count'] == 40005 and info['invalid_count'] == 1
    assert info['frame_id'] == 'lidar_link'


def test_pro_entrypoint_and_build_inputs_are_complete(pro_bundle, monkeypatch):
    monkeypatch.setitem(sys.modules, 'device', pro_bundle.device)
    from common import logsafe
    monkeypatch.setattr(logsafe, 'install', lambda: None)
    spec = importlib.util.spec_from_file_location('s10_pro_entrypoint', SOURCE / 'main.py')
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    assert main.build_plugins is pro_bundle.device.build_plugins
    manifest = yaml.safe_load((SOURCE / 'driver.yaml').read_text())
    cards = {c['name']: c['type'] for c in manifest['cards']}
    tools = {t['name']: t['type'] for t in pro_bundle.bundle.get_all_tools()}
    assert tools == {**cards, 'odom': 'sensor'}
    for line in (SOURCE / 'Dockerfile').read_text().splitlines():
        if line.startswith('COPY ') and '--from=' not in line:
            for name in line.split()[1:-1]:
                if '${' not in name and name != 'common/':
                    assert (SOURCE / name).exists(), name
    vendor = SOURCE / 'resource' / 'deep-robotics-msg'
    names = {file.stem for file in (vendor / 'msg').glob('*.msg')}
    for file in (vendor / 'msg').glob('*.msg'):
        for line in file.read_text().splitlines():
            fields = line.split('#', 1)[0].split()
            if fields:
                kind = fields[0].split('[')[0]
                if kind[0].isupper():
                    assert kind in names


def test_http_mcp_lists_pro_tools_and_starts_a_monitor(pro_bundle):
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(lambda: pro_bundle.bundle, 's10-pro', 's10'))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def rpc(method, params):
        request = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/mcp',
            data=json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)
    try:
        tools = rpc('tools/list', {})['result']['tools']
        assert {'battery', 'joints', 'model', 'lidar_cloud', 'imu', 'odom',
                'camera_front_depth', 'loco_servo'} <= {tool['name'] for tool in tools}
        result = rpc('tools/call', {'name': 'battery', 'arguments': {'action': 'start'}})
        state = json.loads(result['result']['content'][0]['text'])
        assert state == {'state': 'running', 'topic_out': [{'topic': '/test/battery', 'format': 'data/json'}]}
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
