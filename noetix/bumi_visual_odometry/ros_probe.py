"""Actual RTAB-Map RGB-D integration (no robot, no USB, no IMU).

Run inside the sidecar image with ROS_DOMAIN_ID=239. Renders a fixed textured
wall from a camera moving forward, feeds the sidecar's real shared-input topics,
and checks tracking, a synthetic one-metre displacement, input-loss detection and
segment replacement. This validates software integration, not metric accuracy.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import time


def truth(elapsed):
    return .25*max(0., min(4., elapsed-7.))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '239':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=239')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    try:
        from .runtime import VisualRuntime
    except ImportError:
        from runtime import VisualRuntime
    rclpy.init()
    runtime = VisualRuntime({'source_namespace': 'visual_probe'})
    producer = Node('bumi_visual_odometry_rendered_probe')
    root = '/visual_probe/odometry_input/'
    pubs = {name: producer.create_publisher(cls, root+name, qos_profile_sensor_data)
            for name, cls in (('rgb', Image), ('depth', Image),
                              ('info', CameraInfo), ('camera_meta', String))}
    rng = np.random.default_rng(408)
    texture = rng.integers(0, 256, (2048, 2048, 3), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (3, 3), 0)
    u, v = np.meshgrid(np.arange(640, dtype=np.float32), np.arange(480, dtype=np.float32))
    fx = fy = 600.
    cx, cy = 320., 240.
    sequence = 0
    last_image = -1.
    started = time.monotonic()
    rows = []
    result = {}
    runtime.start()
    try:
        while time.monotonic()-started < 14.:
            elapsed = time.monotonic()-started
            p = truth(elapsed)
            now = producer.get_clock().now().nanoseconds
            sequence += 1
            if elapsed-last_image >= .10:
                last_image = elapsed
                depth = 4.-p
                # Optical frame: x right, y down, z forward.
                x_img = (u-cx)/fx*depth
                y_img = (v-cy)/fy*depth
                rgb = cv2.remap(texture, (x_img+4.)/.004, (y_img+4.)/.004,
                                cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
                z = np.full((480, 640), depth, dtype='<f4')
                msg = None
                for name, pixels, encoding, bpp in (('rgb', rgb, 'bgr8', 3),
                                                    ('depth', z, '32FC1', 4)):
                    msg = Image()
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 10**9)
                    msg.header.frame_id = 'visual_probe_rgb_optical'
                    msg.width, msg.height = 640, 480
                    msg.step = 640*bpp
                    msg.encoding = encoding
                    msg.data = pixels.tobytes()
                    pubs[name].publish(msg)
                info = CameraInfo()
                info.header = msg.header
                info.width, info.height = 640, 480
                info.distortion_model = 'plumb_bob'
                info.d = [0.]*5
                info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
                info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
                pubs['info'].publish(info)
                source = dict(source_ms=now/1e6, frame_number=sequence, domain='global_time')
                meta = dict(epoch='synthetic-camera', rgb=source, depth=source,
                            depth_scale=.001, rgb_stamp_ns=now, depth_stamp_ns=now,
                            received_ns=now, clock_mapping='synthetic_shared_clock')
                pubs['camera_meta'].publish(String(data=json.dumps(meta)))
            rclpy.spin_once(runtime.node, timeout_sec=.005)
            sample = runtime.publish()
            if sample['vendor']['status'] == 'lost':
                raise AssertionError(sample['vendor'])
            if sample['pose'] is not None:
                rows.append({'elapsed_s': elapsed, 'truth_m': p,
                             'estimated_position': sample['pose']['position'],
                             'estimate_stamp_ms': sample['stamp_ms'],
                             'segment_id': sample['vendor']['segment_id']})
            time.sleep(.005)
        first = [r for r in rows if 5.5 < r['elapsed_s'] < 6.5]
        last = [r for r in rows if 12.5 < r['elapsed_s'] < 13.5]
        if len(first) < 5 or len(last) < 5:
            raise AssertionError('insufficient tracking coverage')
        moved = float(np.mean([r['estimated_position'][2] for r in last])
                      - np.mean([r['estimated_position'][2] for r in first]))
        if abs(moved-1.) > .10:
            raise AssertionError('synthetic displacement error '+str(moved-1.))
        # Stop the inputs but keep the sidecar running: the state must go lost,
        # not keep predicting a pose.
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            rclpy.spin_once(runtime.node, timeout_sec=.02)
        if runtime.publish()['vendor']['status'] != 'lost':
            raise AssertionError('input loss hidden')
        old = runtime.segment
        runtime.reset()
        if runtime.segment == old:
            raise AssertionError('reset did not replace the coordinate segment')
        result.update(integration_passed=True, estimated_distance_m=moved,
                      truth_distance_m=1., error_m=moved-1.)
    except Exception as exc:
        result.update(integration_passed=False, error=str(exc))
    finally:
        result.update(data_source='synthetic_rendered_rgbd',
                      hardware_accuracy_validated=False, rows=rows)
        Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False))
        runtime.close()
        producer.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}))
    return 0 if result.get('integration_passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
