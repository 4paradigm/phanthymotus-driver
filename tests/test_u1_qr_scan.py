"""Real QR images through U1 JPEG conversion and simulated ROS endpoints.

These tests exercise message contracts, not real DDS transport or hardware.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.qr_decoder import QrDecoder, QrTracker


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


qr = load('u1_qr', 'ubtrobot/u1_pro/qr_scan.py')
device = load('u1_device', 'ubtrobot/u1_pro/device.py')


def image(text):
    matrix = cv2.QRCodeEncoder_create().encode(text)
    return cv2.resize(matrix, (290, 290), interpolation=cv2.INTER_NEAREST)


def jpeg(array):
    ok, data = cv2.imencode('.jpg', array)
    assert ok
    return data.tobytes()


@pytest.mark.parametrize('text', ['U1-001', '展品编号：001', 'https://example.com/item/001'])
def test_decode_real_jpeg(text):
    result = QrDecoder().decode(jpeg(image(text)))
    assert [c['text'] for c in result] == [text]
    assert result[0]['image_width'] == 290
    assert len(result[0]['corners_px']) == 4


def test_blank_multi_and_invalid():
    canvas = np.full((390, 740), 255, dtype=np.uint8)
    assert QrDecoder().decode(jpeg(canvas)) == []
    canvas[50:340, 30:320] = image('A')
    canvas[50:340, 420:710] = image('B')
    assert {c['text'] for c in QrDecoder().decode(jpeg(canvas))} == {'A', 'B'}
    for data in (b'', b'broken', b'x' * 5_000_001):
        with pytest.raises(ValueError):
            QrDecoder().decode(data)


def test_dedup_reappearance_and_bound():
    tracker = QrTracker(3)
    assert tracker.update([{'text': 'A'}], 0)
    assert tracker.update([{'text': 'A'}], 1) == []
    assert tracker.update([{'text': 'A'}], 5)
    tracker.update([{'text': str(i)} for i in range(300)], 6)
    assert len(tracker.last_seen) <= 256


class FakeCore:
    def __init__(self):
        self.subscriptions = []
        self.publishers = []
        self.messages = []

    def create_subscription(self, cls, topic, callback, qos):
        subscription = SimpleNamespace(topic=topic, callback=callback, cls=cls, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def create_publisher(self, cls, topic, qos):
        def publish(message):
            self.messages.append((topic, message))
            for subscription in list(self.subscriptions):
                if subscription.topic == topic:
                    subscription.callback(message)
        publisher = SimpleNamespace(publish=publish, topic=topic, cls=cls, qos=qos)
        self.publishers.append(publisher)
        return publisher

    def destroy_subscription(self, obj):
        self.subscriptions.remove(obj)

    def destroy_publisher(self, obj):
        self.publishers.remove(obj)


@pytest.fixture
def nodes(monkeypatch):
    class Message:
        pass
    class QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(
        String=Message, Header=lambda: SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(
        QoSProfile=QoS, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'),
        DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')))
    return SimpleNamespace(namespace='u1_test', core=FakeCore(), CompressedImage=Message)


def wait_status(plugin, status, sequence=None):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        result = plugin.dispatch('read', {})
        if result['status'] == status and (sequence is None or result['frame_sequence'] == sequence):
            return result
        time.sleep(0.01)
    pytest.fail(f'Expected {status}, got {result}')


def test_u1_rgb_camera_to_qr_json_and_stop_restart(nodes):
    scanner = qr.QrScanPlugin(nodes, {'scan_hz': 10, 'stale_after_s': 0.5})
    camera = device.EyeCameraPlugin(nodes, 'left')
    camera._publisher = nodes.core.create_publisher(nodes.CompressedImage, camera.topic, 1)
    scanner.start()
    try:
        wait_status(scanner, 'waiting_for_frame')
        subscription = nodes.core.subscriptions[0]
        assert subscription.topic == camera.topic
        assert subscription.cls is nodes.CompressedImage
        assert subscription.qos.reliability == 'best_effort'
        # Exercise the actual U1 camera conversion/publish method with an RGB frame.
        rgb = cv2.cvtColor(image('U1-LABEL'), cv2.COLOR_GRAY2RGB)
        metadata = {'width': 290, 'height': 290, 'step': 870, 'encoding': 'rgb8'}
        camera._publish_frame(rgb.tobytes(), metadata, 1234567890)
        result = wait_status(scanner, 'detected', 1)
        assert result['codes'][0]['text'] == 'U1-LABEL'
        assert result['new_events'] == [{'sequence': 1, 'text': 'U1-LABEL'}]
        outputs = [json.loads(m.data) for topic, m in nodes.core.messages if topic == scanner.output_topic]
        assert any(r['status'] == 'detected' for r in outputs)
        camera._publish_frame(rgb.tobytes(), metadata, 2234567890)
        assert wait_status(scanner, 'detected', 2)['new_events'] == []
        stale = wait_status(scanner, 'stale', 2)
        assert stale['codes'] == stale['new_events'] == []
        old_session = stale['session_id']
        scanner.stop()
        assert not nodes.core.subscriptions
        assert scanner.dispatch('read', {})['state'] == 'idle'
        scanner.start()
        restarted = wait_status(scanner, 'waiting_for_frame')
        assert restarted['session_id'] != old_session
        assert restarted['frame_sequence'] is None
    finally:
        scanner.stop()
    assert len(nodes.core.publishers) == 1  # Only the camera publisher remains.


@pytest.mark.parametrize('format,data', [('png', b'x'), ('jpeg', b''), ('jpeg', b'broken')])
def test_bad_message_is_decode_error(nodes, format, data):
    scanner = qr.QrScanPlugin(nodes, {'scan_hz': 10})
    scanner.start()
    try:
        nodes.core.subscriptions[0].callback(SimpleNamespace(format=format, data=data))
        result = wait_status(scanner, 'decode_error', 1)
        assert result['codes'] == []
    finally:
        scanner.stop()


def test_topic_validation_and_latest_frame(nodes):
    scanner = qr.QrScanPlugin(nodes)
    assert scanner.dispatch('info', {})['input_topic'] == '/u1_test/camera/left'
    assert scanner.dispatch('start', {'input_topic': '/u1_test/camera/right'})['state'] == 'error'
    assert not nodes.core.subscriptions
    scanner._on_frame(SimpleNamespace(format='jpeg', data=jpeg(image('A'))))
    scanner._on_frame(SimpleNamespace(format='jpeg', data=jpeg(image('B'))))
    scanner.decoder = QrDecoder()
    assert scanner.process_frame(scanner._frame)['codes'][0]['text'] == 'B'


def test_start_idempotent_and_partial_failure_cleanup(nodes):
    scanner = qr.QrScanPlugin(nodes)
    scanner.start()
    scanner.start()
    assert len(nodes.core.subscriptions) == len(nodes.core.publishers) == 1
    scanner.stop()
    scanner.stop()
    assert not nodes.core.subscriptions and not nodes.core.publishers
    def fail(*args):
        raise RuntimeError('subscription failure')
    nodes.core.create_subscription = fail
    with pytest.raises(RuntimeError):
        scanner.start()
    assert not nodes.core.publishers


@pytest.mark.parametrize('cfg', [{'scan_hz': 0}, {'scan_hz': float('nan')}, {'stale_after_s': -1}])
def test_invalid_config(nodes, cfg):
    with pytest.raises(ValueError):
        qr.QrScanPlugin(nodes, cfg)
