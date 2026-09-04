#!/usr/bin/env python3
"""
Two-process joints bridge with proper DDS isolation.

Process 1 (subscriber): domain 0 + vendor profile → subscribe from body controller
Process 2 (publisher): domain 42 + dds-local.xml → publish to agent-core

Communication: Unix domain socket (fast, local, simple)
"""

import os
import json
import signal
import socket
import struct
import threading
import time
from pathlib import Path

import yaml
import rclpy
import rclpy.executors
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import String


SOCKET_PATH = "/tmp/tianyi_joints_bridge.sock"


class JointsSubscriber(Node):
    """Subscribe to joints from domain 0, forward to Unix socket."""

    def __init__(self, namespace: str, sock):
        super().__init__("joints_subscriber")
        self.sock = sock
        self.sub = self.create_subscription(
            String,
            f"/{namespace}/state/joints",
            self.callback,
            10
        )
        print(f"[joints-bridge-sub] subscribed to /{namespace}/state/joints on domain 0")

    def callback(self, msg: String):
        try:
            # Send message length + data
            data = msg.data.encode('utf-8')
            self.sock.sendall(struct.pack('!I', len(data)) + data)
        except Exception as e:
            print(f"[joints-bridge-sub] send error: {e}")


class JointsPublisher(Node):
    """Receive from Unix socket, publish to domain 42."""

    def __init__(self, namespace: str):
        super().__init__("joints_publisher")
        self.pub = self.create_publisher(String, f"/{namespace}/state/joints", 10)
        print(f"[joints-bridge-pub] publishing to /{namespace}/state/joints on domain 42")

    def publish(self, data: str):
        msg = String()
        msg.data = data
        self.pub.publish(msg)


def run_subscriber():
    """Process 1: domain 0 subscriber with vendor profile."""
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    namespace = cfg.get("ros_namespace", "").strip() or socket.gethostname()

    # Use vendor profile for domain 0
    dds_profile = "/work/dds_profile.xml"
    if os.path.exists(dds_profile):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = dds_profile
        print(f"[joints-bridge-sub] using DDS profile {dds_profile}")

    ctx = Context()
    rclpy.init(context=ctx, domain_id=0)

    # Create Unix socket client
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    while True:
        try:
            sock.connect(SOCKET_PATH)
            print(f"[joints-bridge-sub] connected to {SOCKET_PATH}")
            break
        except:
            time.sleep(0.5)

    node = JointsSubscriber(namespace, sock)
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        sock.close()
        executor.shutdown()
        rclpy.shutdown(context=ctx)


def run_publisher():
    """Process 2: domain 42 publisher with dds-local.xml."""
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    namespace = cfg.get("ros_namespace", "").strip() or socket.gethostname()

    # Use dds-local.xml for domain 42
    dds_local = "/opt/phanthy-motus/dds-local.xml"
    if os.path.exists(dds_local):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = dds_local
        print(f"[joints-bridge-pub] using DDS profile {dds_local}")
    else:
        print(f"[joints-bridge-pub] WARNING: {dds_local} not found")

    ctx = Context()
    rclpy.init(context=ctx, domain_id=42)

    node = JointsPublisher(namespace)
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)

    # Create Unix socket server
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    print(f"[joints-bridge-pub] listening on {SOCKET_PATH}")

    conn, _ = server.accept()
    print(f"[joints-bridge-pub] subscriber connected")

    stop_flag = threading.Event()

    def receive_loop():
        """Receive from socket and publish."""
        try:
            while not stop_flag.is_set():
                # Read message length
                length_bytes = b''
                while len(length_bytes) < 4:
                    chunk = conn.recv(4 - len(length_bytes))
                    if not chunk:
                        return
                    length_bytes += chunk

                length = struct.unpack('!I', length_bytes)[0]

                # Read message data
                data = b''
                while len(data) < length:
                    chunk = conn.recv(length - len(data))
                    if not chunk:
                        return
                    data += chunk

                # Publish to domain 42
                node.publish(data.decode('utf-8'))
        except Exception as e:
            if not stop_flag.is_set():
                print(f"[joints-bridge-pub] receive error: {e}")

    def spin_loop():
        """Spin ROS executor."""
        while not stop_flag.is_set() and rclpy.ok(context=ctx):
            executor.spin_once(timeout_sec=0.1)

    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    recv_thread.start()
    spin_thread.start()

    def stop(*_args):
        stop_flag.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    try:
        while not stop_flag.is_set():
            time.sleep(0.1)
    finally:
        conn.close()
        server.close()
        os.unlink(SOCKET_PATH)
        executor.shutdown()
        rclpy.shutdown(context=ctx)


def main():
    """Launch both processes."""
    import sys
    import multiprocessing as mp

    if len(sys.argv) > 1:
        if sys.argv[1] == "subscriber":
            run_subscriber()
        elif sys.argv[1] == "publisher":
            run_publisher()
        else:
            print(f"Usage: {sys.argv[0]} [subscriber|publisher]")
            sys.exit(1)
    else:
        # Launch both as separate processes
        mp.set_start_method('spawn', force=True)

        pub_proc = mp.Process(target=run_publisher, name="joints-bridge-pub")
        sub_proc = mp.Process(target=run_subscriber, name="joints-bridge-sub")

        pub_proc.start()
        time.sleep(1)  # Let publisher start first
        sub_proc.start()

        print("[joints-bridge] both processes started")

        try:
            pub_proc.join()
            sub_proc.join()
        except KeyboardInterrupt:
            pub_proc.terminate()
            sub_proc.terminate()
            pub_proc.join()
            sub_proc.join()


if __name__ == "__main__":
    main()
