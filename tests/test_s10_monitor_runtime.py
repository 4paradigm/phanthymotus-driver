"""Offline end-to-end monitor contracts using fake vendor subscriptions."""
import ast
import importlib.util
import json
import struct
import sys
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'deep_robotics/s10_pro'
sys.path.insert(0, str(SOURCE))
from device import build_plugins
from reader import DEFAULT_SOURCES, SubscriptionReader, LatestEventWriter, normalize_control
from pointcloud import convert_pointcloud
from common.vendor_runtime import DriverBundle, make_handler


class Bridge:
    def __init__(self):
        self.owners = set()
        self.age = 0.01

    def acquire(self, name, sources, settings):
        assert set(sources) <= set(DEFAULT_SOURCES)
        self.owners.add(name)

    def release(self, name):
        self.owners.discard(name)

    def snapshot(self, source):
        return {'data': {'data': [{'battery_level': 80}]}, 'age_s': self.age,
                'fresh': True, 'sequence': 1}

    def status(self):
        return {'reader_running': bool(self.owners)}


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


def test_card_stop_releases_only_its_lease_and_stale_data_is_not_live():
    bridge = Bridge()
    cards = build_plugins({}, 'test', bridge, lambda *args: None)
    try:
        cards[0].start()
        cards[3].start()
        assert bridge.owners == {'battery', 'imu'}
        assert cards[0].status()['fresh']
        bridge.age = 60
        assert not cards[0].status()['fresh']
        assert cards[0].status()['status'] == 'stale'
        cards[0].stop()
        assert bridge.owners == {'imu'}
        assert not cards[0].status()['connected']
    finally:
        for card in cards:
            card.stop()


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


def test_standalone_entrypoint_and_build_inputs_are_complete():
    spec = importlib.util.spec_from_file_location('s10_monitor_entrypoint', SOURCE / 'main.py')
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    assert [p.name for p in main.build_runtime_plugins({}, 'test', Bridge(), lambda *args: None)] == [
        'battery', 'joints', 'lidar_cloud', 'imu']
    manifest = yaml.safe_load((SOURCE / 'driver.yaml').read_text())
    assert {c['name'] for c in manifest['cards']} == {'battery', 'joints', 'model', 'lidar_cloud', 'imu'}
    for file in SOURCE.glob('*.py'):
        tree = ast.parse(file.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or '').startswith(('motion', 'acp', 'operator_authorization', 'loco'))
    assert not list(SOURCE.glob('*motion*'))
    for line in (SOURCE / 'Dockerfile').read_text().splitlines():
        if line.startswith('COPY ') and '--from=' not in line:
            for name in line.split()[1:-1]:
                if '${' not in name and name != 'common/':
                    assert (SOURCE / name).exists(), name
    # Every local vendor field type is included in the minimal interface subset.
    vendor = next((SOURCE / 'vendor').glob('deep-*'))
    names = {file.stem for file in (vendor / 'msg').glob('*.msg')}
    for file in (vendor / 'msg').glob('*.msg'):
        for line in file.read_text().splitlines():
            fields = line.split('#', 1)[0].split()
            if fields:
                kind = fields[0].split('[')[0]
                if kind[0].isupper():
                    assert kind in names


def test_http_mcp_lists_only_monitor_tools_and_starts_sensor():
    bridge = Bridge()
    bundle = DriverBundle(build_plugins({}, 'test', bridge, lambda *args: None))
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(lambda: bundle, 's10-monitors', 's10'))
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
        assert {tool['name'] for tool in tools} == {'battery', 'joints', 'model', 'lidar_cloud', 'imu'}
        result = rpc('tools/call', {'name': 'battery', 'arguments': {}})
        assert 'error' not in result
        assert bridge.owners == {'battery'}
    finally:
        bundle.stop_all()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
