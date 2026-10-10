"""Input decode errors must leave the next valid frame usable; no DDS/device."""
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

import pytest

import teleop_executor
from test_motion_control import chain, packet


@pytest.mark.parametrize('route,value', [
    ('eef', None), ('eef', []), ('eef', 'pose'), ('eef', 4),
    ('arm', None), ('arm', []), ('arm', True),
    (None, {}), ([], {}), ({}, {}), ('unknown', {}),
])
def test_bad_route_payload_is_rejected_and_next_preview_frame_is_accepted(chain, route, value):
    controller, executor = chain.c, chain.e
    lease = controller.dispatch('prepare_preview', {})
    executor._command(SimpleNamespace(data=json.dumps({'_motion_route': route, 'packet': value})))
    assert controller._decision['state'] == 'rejected'
    assert controller._pending is None
    valid = packet(controller, lease)
    executor._command(SimpleNamespace(data=json.dumps({'_motion_route': 'eef', 'packet': valid})))
    assert controller._pending[0]['seq'] == valid['seq']
    assert not chain.p.writes and not chain.commands


@pytest.mark.parametrize('bad', [
    '[' * 1100 + '0' + ']' * 1100,
    '{"seq":1,"seq":2}',
    '{"value":NaN}',
    '{',
], ids=['nested-json', 'duplicate-key', 'non-finite', 'truncated-json'])
def test_bus_decode_error_does_not_prevent_the_next_valid_input(monkeypatch, bad):
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    parent.settimeout(1)
    subscriptions, publications, calls = {}, [], []

    class Node:
        def __init__(self, name): pass
        def create_subscription(self, message, topic, callback, qos):
            subscriptions[topic] = callback
        def create_publisher(self, message, topic, qos):
            return SimpleNamespace(publish=lambda msg: publications.append(topic))
        def destroy_node(self): pass

    def spin(node, timeout_sec):
        calls.append(True)
        for suffix, route in (('control', 'eef'), ('arm', 'arm')):
            callback = subscriptions[f'/test/motion/{suffix}/command']
            callback(SimpleNamespace(data=bad))
            callback(SimpleNamespace(data='{"seq":1}'))
            assert json.loads(parent.recv(4096)) == {'_motion_route': route, 'packet': {'seq': 1}}

    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace(init=lambda **kw: None,
        ok=lambda: not calls, spin_once=spin, try_shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, 'rclpy.node', SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy.executors', SimpleNamespace(ExternalShutdownException=RuntimeError))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(QoSProfile=lambda **kw: kw,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1), HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1)))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(String=SimpleNamespace))
    monkeypatch.setitem(sys.modules, 'common', SimpleNamespace(logsafe=SimpleNamespace(install=lambda **kw: None)))
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', str(Path(teleop_executor.__file__).with_name('dds-local.xml')))
    try:
        teleop_executor.run_local_bus(child.detach(), 'test', True)
    finally:
        parent.close()
        child.close()
    assert len(calls) == 1 and not publications
