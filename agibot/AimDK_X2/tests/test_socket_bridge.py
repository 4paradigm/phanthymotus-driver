"""Socket-to-ROS integration tests for the isolated X2 bridge."""

from __future__ import annotations

import importlib
import json
import socket
import struct
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

DEVICE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = DEVICE_DIR.parent.parent
for path in (str(REPO_ROOT), str(DEVICE_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)


class FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []

    def publish(self, message):
        self.published.append(message)


class FakeNode:
    def __init__(self, name, context=None):
        self.name = name
        self.context = context
        self.publishers = []

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher(msg_type, topic, qos)
        self.publishers.append(publisher)
        return publisher


class FakeExecutor:
    def __init__(self, context=None):
        self.context = context
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def spin_once(self, timeout_sec=None):
        return None

    def shutdown(self):
        return None


class FakeContext:
    pass


class FakeQoSProfile:
    def __init__(self, **kwargs):
        self.settings = kwargs


def module(name, **attributes):
    result = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(result, key, value)
    sys.modules[name] = result
    return result


def import_socket_bridge():
    import common

    fake_logsafe = types.SimpleNamespace(install=mock.Mock())
    common.logsafe = fake_logsafe
    sys.modules["common.logsafe"] = fake_logsafe

    rclpy = module(
        "rclpy",
        init=mock.Mock(),
        ok=lambda context=None: True,
        shutdown=mock.Mock(),
    )
    rclpy.executors = types.SimpleNamespace(MultiThreadedExecutor=FakeExecutor)
    module("rclpy.context", Context=FakeContext)
    module("rclpy.node", Node=FakeNode)
    module(
        "rclpy.qos",
        DurabilityPolicy=types.SimpleNamespace(
            VOLATILE="volatile", TRANSIENT_LOCAL="transient_local"
        ),
        HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"),
        QoSProfile=FakeQoSProfile,
        ReliabilityPolicy=types.SimpleNamespace(
            RELIABLE="reliable", BEST_EFFORT="best_effort"
        ),
    )
    module(
        "rclpy.serialization",
        deserialize_message=lambda payload, msg_type: (msg_type, payload),
    )
    module("rosidl_runtime_py")
    module(
        "rosidl_runtime_py.utilities",
        get_message=lambda type_name: f"resolved:{type_name}",
    )
    sys.modules.pop("x2_socket_bridge", None)
    imported = importlib.import_module("x2_socket_bridge")
    return imported, fake_logsafe


class SocketBridgeIntegrationTests(unittest.TestCase):
    QOS = {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 5,
    }

    def send_registration(self, server, metadata, payload=None):
        client, accepted = socket.socketpair()
        worker = threading.Thread(target=server.client, args=(accepted,))
        worker.start()
        encoded = json.dumps(metadata).encode()
        client.sendall(struct.pack("<I", len(encoded)) + encoded)
        if payload is not None:
            client.sendall(struct.pack("<I", len(payload)) + payload)
        client.shutdown(socket.SHUT_WR)
        worker.join(timeout=2)
        client.close()
        self.assertFalse(worker.is_alive())

    def test_registration_frame_and_reconnect_publish_to_same_ros_topic(self):
        bridge, fake_logsafe = import_socket_bridge()
        server = bridge.Server()

        def send_client(payload):
            client, accepted = socket.socketpair()
            worker = threading.Thread(target=server.client, args=(accepted,))
            worker.start()
            metadata = json.dumps({
                "topic": "/agibot_x2/state/joints",
                "msg_type": "std_msgs/msg/String",
                "qos": self.QOS,
            }).encode()
            client.sendall(struct.pack("<I", len(metadata)) + metadata)
            client.sendall(struct.pack("<I", len(payload)) + payload)
            client.shutdown(socket.SHUT_WR)
            worker.join(timeout=2)
            client.close()
            self.assertFalse(worker.is_alive())

        send_client(b"first-frame")
        send_client(b"second-frame")

        fake_logsafe.install.assert_called_once_with()
        self.assertEqual(list(server.handlers), ["/agibot_x2/state/joints"])
        handler = server.handlers["/agibot_x2/state/joints"]
        self.assertEqual(handler.msg_class, "resolved:std_msgs/msg/String")
        self.assertEqual(handler.publisher.topic, "/agibot_x2/state/joints")
        self.assertEqual(handler.publisher.qos.settings["reliability"], "reliable")
        self.assertEqual(handler.publisher.qos.settings["depth"], 5)
        self.assertEqual(handler.publisher.published, [
            ("resolved:std_msgs/msg/String", b"first-frame"),
            ("resolved:std_msgs/msg/String", b"second-frame"),
        ])

    def test_concurrent_same_topic_clients_create_one_handler(self):
        bridge, _ = import_socket_bridge()
        server = bridge.Server()
        original_handler = bridge.TopicHandler

        class RacingHandler(original_handler):
            created = 0

            def __init__(self, *args, **kwargs):
                type(self).created += 1
                # Widen the race: without handlers_lock both client threads
                # observe the missing topic and construct a handler.
                time.sleep(0.05)
                super().__init__(*args, **kwargs)

        metadata = json.dumps({
            "topic": "/agibot_x2/state/joints",
            "msg_type": "std_msgs/msg/String",
            "qos": self.QOS,
        }).encode()
        clients = []
        workers = []
        with mock.patch.object(bridge, "TopicHandler", RacingHandler):
            for _ in range(2):
                client, accepted = socket.socketpair()
                worker = threading.Thread(target=server.client, args=(accepted,))
                worker.start()
                clients.append(client)
                workers.append(worker)
            for client in clients:
                client.sendall(struct.pack("<I", len(metadata)) + metadata)
                client.shutdown(socket.SHUT_WR)
            for client, worker in zip(clients, workers):
                worker.join(timeout=3)
                client.close()
                self.assertFalse(worker.is_alive())

        self.assertEqual(RacingHandler.created, 1)
        self.assertEqual(list(server.handlers), ["/agibot_x2/state/joints"])

    def test_conflicting_qos_registration_does_not_replace_handler(self):
        bridge, _ = import_socket_bridge()
        server = bridge.Server()
        metadata = {
            "topic": "/agibot_x2/state/joints",
            "msg_type": "std_msgs/msg/String",
            "qos": self.QOS,
        }
        self.send_registration(server, metadata, b"accepted")
        original = server.handlers[metadata["topic"]]

        conflicting = dict(metadata)
        conflicting["qos"] = {**self.QOS, "reliability": "best_effort"}
        self.send_registration(server, conflicting, b"rejected")

        self.assertIs(server.handlers[metadata["topic"]], original)
        self.assertEqual(original.publisher.published, [
            ("resolved:std_msgs/msg/String", b"accepted"),
        ])

    def test_oversized_metadata_is_rejected_before_allocation(self):
        bridge, _ = import_socket_bridge()
        server = bridge.Server()
        client, accepted = socket.socketpair()
        worker = threading.Thread(target=server.client, args=(accepted,))
        worker.start()
        client.sendall(struct.pack("<I", bridge.MAX_METADATA_BYTES + 1))
        client.shutdown(socket.SHUT_WR)
        worker.join(timeout=2)
        client.close()

        self.assertFalse(worker.is_alive())
        self.assertEqual(server.handlers, {})


if __name__ == "__main__":
    unittest.main()
