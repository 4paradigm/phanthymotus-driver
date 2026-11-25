"""Hardware-free real DDS/JPEG QR smoke test; run in a ROS2 environment."""
from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('u1_qr', ROOT / 'ubtrobot/u1_pro/qr_scan.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def main():
    # Independent explicit contexts prove the plugin does not need the default context.
    contexts = [Context(), Context()]
    for context in contexts:
        rclpy.init(context=context, domain_id=142)
    scanner_node = Node('qr_scanner_check', context=contexts[0])
    peer = Node('qr_camera_check', context=contexts[1])
    executor = SingleThreadedExecutor(context=contexts[0])
    peer_executor = SingleThreadedExecutor(context=contexts[1])
    executor.add_node(scanner_node)
    peer_executor.add_node(peer)
    nodes = SimpleNamespace(namespace='u1_check', core=scanner_node, CompressedImage=CompressedImage)
    scanner = module.QrScanPlugin(nodes, {'scan_hz': 2, 'stale_after_s': 3})
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    received = []
    peer.create_subscription(String, scanner.output_topic, lambda m: received.append(json.loads(m.data)), qos)
    publisher = peer.create_publisher(CompressedImage, scanner.input_topic, qos)
    pixels = cv2.resize(cv2.QRCodeEncoder_create().encode('U1-DDS-001'), (290, 290),
                        interpolation=cv2.INTER_NEAREST)
    ok, data = cv2.imencode('.jpg', pixels)
    assert ok
    message = CompressedImage()
    message.format = 'jpeg'
    message.data = data.tobytes()
    try:
        scanner.start()
        deadline = time.monotonic() + 15
        detected = None
        while time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.03)
            peer_executor.spin_once(timeout_sec=0.03)
            detected = next((r for r in received if r['status'] == 'detected'), None)
            if detected:
                break
        assert detected, f'No QR result over DDS: {received[-3:]}'
        assert detected['codes'][0]['text'] == 'U1-DDS-001'
        assert detected['new_events'][0]['sequence'] == 1
        # Stop sending frames and verify stale is actually delivered over DDS.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(r['status'] == 'stale' for r in received):
            executor.spin_once(timeout_sec=0.03)
            peer_executor.spin_once(timeout_sec=0.03)
        stale = next((r for r in received if r['status'] == 'stale'), None)
        assert stale and stale['codes'] == [] and stale['new_events'] == []
        scanner.stop()
        assert scanner.dispatch('read', {})['state'] == 'idle'
        print(json.dumps({'dds': 'passed', 'decoded': detected['codes'][0]['text'],
                          'stale_clears_codes': True, 'explicit_contexts': True,
                          'opencv': cv2.__version__, 'numpy': np.__version__}))
    finally:
        scanner.stop()
        executor.shutdown()
        peer_executor.shutdown()
        scanner_node.destroy_node()
        peer.destroy_node()
        for context in contexts:
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
