"""Real bus routing over anonymous sockets with ROS stand-ins; no DDS or robot."""
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

import teleop_executor


def test_v2_bus_keeps_feedback_and_joint_latest_separately(monkeypatch):
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    parent.settimeout(1)
    subscriptions, publications, domains = {}, [], []
    calls = [0]

    class Node:
        def __init__(self, name):pass
        def create_subscription(self, message, topic, callback, qos):
            assert qos['depth'] == 1
            subscriptions[topic] = callback
        def create_publisher(self, message, topic, qos):
            def publish(msg):
                publications.append((topic, json.loads(msg.data)))
                if topic in subscriptions:subscriptions[topic](msg)
            return SimpleNamespace(publish=publish)
        def destroy_node(self):pass

    def spin(node, timeout_sec):
        calls[0] += 1
        if calls[0] == 1:
            for seq in (1, 2):
                subscriptions['/test/motion/control/command'](SimpleNamespace(data=json.dumps({'seq': seq})))
                assert json.loads(parent.recv(4096)) == {'_motion_route': 'eef', 'packet': {'seq': seq}}
            for seq in (3, 4):
                parent.send(json.dumps({'_motion_route': 'joint_output', 'packet': {'seq': seq}}).encode())
                parent.send(json.dumps({'state': 'ready', 'sequence': seq}).encode())
        else:
            # Locally published joint result returns via the actual arm subscriber.
            assert json.loads(parent.recv(4096)) == {'_motion_route': 'arm', 'packet': {'seq': 4}}

    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace(init=lambda **kw: domains.append(kw['domain_id']),
        ok=lambda: calls[0] < 2, spin_once=spin, try_shutdown=lambda: None))
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
    assert domains == [42]
    assert set(subscriptions) == {'/test/motion/teleop/command', '/test/motion/control/command', '/test/motion/arm/command'}
    assert publications == [('/test/motion/arm/command', {'seq': 4}),
                            ('/test/motion/teleop/feedback', {'state': 'ready', 'sequence': 4})]
