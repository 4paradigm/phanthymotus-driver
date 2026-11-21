"""Read existing robot topics only; no service starts, movement or deployment.

Run in the Panther image with the changed modules on PYTHONPATH. Only numeric
summaries are printed; no images or IMU recordings are written to disk.
"""
import json
import threading
import time
import zlib
import os

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from camera import PantherCameraRuntime
from core_bridge import output_topics, encode_bridge_payload
from device import PantherDevice


class Recorder:
    _namespace = "panther_sensor_validation"

    def __init__(self):
        self.lock = threading.Lock()
        self.counts = {}
        self.latest = {}

    def publish(self, card, payload):
        encode_bridge_payload(card, payload)
        with self.lock:
            self.counts[card] = self.counts.get(card, 0) + 1
            self.latest[card] = payload


def main():
    rclpy.init()
    node = Node('panther_sensor_inputs_validation', enable_rosout=False, start_parameter_services=False)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    bridge = Recorder()
    camera_config = {'rgb_hz': 2, 'depth_hz': 2}
    if os.environ.get('PANTHER_VALIDATE_ESTIMATED_MOUNT') == '1':
        camera_config['estimated_body_mount'] = {'enabled': True, 'model_frame': 'link_rgbd',
                                               'source_frame': 'camera_link', 'translation_m': [0., 0., 0.],
                                               'rpy_rad': [0., 0., 0.]}
    runtime = PantherCameraRuntime(node, bridge, camera_config, qos_profile_sensor_data)
    # Arm forwarding only. Never call start()/arm_imu()/_ensure_source(): the
    # existing camera owner and its production service remain untouched.
    runtime._active.update({'rgb', 'depth', 'imu'})
    body = object.__new__(PantherDevice)
    body._lock = threading.RLock()
    body._odom_segment = 'validation'
    body._odom_frames = None
    body._odom_last_stamp = 0
    body._odom_window = []
    body._odom_vendor_ms = None
    body._odom_vendor_meta = {}
    body._bridge = bridge
    body._put = lambda *args: None
    node.create_subscription(Odometry, '/odom', body._on_odom, qos_profile_sensor_data)
    node.create_timer(.1, body._publish_odom_window)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        time.sleep(20)
        with bridge.lock:
            latest, counts = dict(bridge.latest), dict(bridge.counts)
        topics = output_topics(bridge._namespace)
        declarations = [runtime.declaration(kind, topics[card])
                        for kind, card in [('rgb', 'camera_color'), ('depth', 'camera_depth')]]
        report = {'counts': counts, 'camera_info': declarations,
                  'imu_status': runtime.info('imu'), 'mount_status': runtime.mount_declaration()}
        for card in ('camera_color', 'camera_depth'):
            payload = latest.get(card)
            if payload:
                report[card] = {k: v for k, v in payload.items() if k != 'data'}
                report[card]['bytes'] = len(payload['data'])
        if 'camera_color' in latest:
            import cv2
            decoded = cv2.imdecode(np.frombuffer(latest['camera_color']['data'], np.uint8), cv2.IMREAD_COLOR)
            report['rgb_shape'] = list(decoded.shape)
            assert decoded.shape == (480, 640, 3)
        if 'camera_depth' in latest:
            depth = np.frombuffer(zlib.decompress(latest['camera_depth']['data']), '<u2').reshape(480, 640)
            report['depth'] = {'valid_pixels': int(np.count_nonzero(depth)),
                               'min_valid_mm': int(depth[depth > 0].min()) if np.any(depth > 0) else None,
                               'max_mm': int(depth.max())}
        report['camera_imu'] = latest.get('camera_imu')
        report['body_mount'] = latest.get('body_mount')
        report['odom'] = latest.get('odom')
        print(json.dumps(report, allow_nan=False), flush=True)
        assert counts.get('camera_color', 0) > 0 and counts.get('camera_depth', 0) > 0
        assert counts.get('odom', 0) > 0 and latest['odom']['pose'] is not None
        assert declarations[0]['K'] == declarations[1]['K']
        # Optional TF/IMU availability is reported, never fabricated to pass.
    finally:
        runtime.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
