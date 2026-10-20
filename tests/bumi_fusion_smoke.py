"""ROS-only synthetic integration probe. No camera, SDK, robot or motion calls.

Run with a private ROS_DOMAIN_ID and installed runtime dependencies. This is
deliberately a CLI probe so ordinary laptop pytest does not import ROS.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zlib


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, CameraInfo, CompressedImage
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from std_msgs.msg import String

    if os.environ.get('ROS_DOMAIN_ID') != '242':
        raise SystemExit('Probe requires isolated ROS_DOMAIN_ID=242')
    root = Path(__file__).resolve().parent.parent/'experiments/bumi_odometry/managed'
    if not root.exists():
        raise SystemExit('The legacy fusion probe requires the experiment source checkout')
    sys.path.insert(0, str(root))
    from camera_runtime import CAMERA, stop_process
    processes = [subprocess.Popen([sys.executable, str(root/'camera_runtime.py'), 'bridge', 'smoke'],
                                  start_new_session=True),
                 subprocess.Popen([sys.executable, str(root/'fusion_runtime.py')], start_new_session=True)]
    rclpy.init()
    node = Node('bumi_fusion_synthetic_probe')
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for child in ('bumi_odom_camera_color_optical_frame', 'bumi_odom_camera_imu_optical_frame'):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'bumi_odom_camera_link', child
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
        transforms.append(t)
    tf.sendTransform(transforms)
    pubs = {name: node.create_publisher(cls, CAMERA+'/'+name, qos_profile_sensor_data)
            for name, cls in [('color/image_raw', Image), ('color/camera_info', CameraInfo),
                              ('aligned_depth_to_color/image_raw', Image), ('imu', Imu)]}
    result = dict(color=0, depth=0, tracking=False)
    def color(msg):
        assert cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR).shape == (480, 640, 3)
        result['color'] += 1
    def depth(msg):
        assert len(zlib.decompress(msg.data)) == 640*480*2
        result['depth'] += 1
    def status(msg):
        data = json.loads(msg.data)
        result['status'] = data
        if data['status'] == 'tracking': result['tracking'] = True
    node.create_subscription(CompressedImage, '/smoke/camera/color', color, qos_profile_sensor_data)
    node.create_subscription(CompressedImage, '/smoke/camera/depth', depth, qos_profile_sensor_data)
    node.create_subscription(String, '/bumi_odom/status', status, 10)
    pixels = np.random.default_rng(408).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    depth_pixels = np.full((480, 640), 2000, dtype='<u2')
    last_image = 0
    started = time.monotonic()
    try:
        while time.monotonic()-started < 45:
            stamp = node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'bumi_odom_camera_imu_optical_frame'
            imu.orientation_covariance[0] = -1
            imu.linear_acceleration.y = -9.81
            for i in (0, 4, 8):
                imu.angular_velocity_covariance[i] = 0.01
                imu.linear_acceleration_covariance[i] = 0.01
            pubs['imu'].publish(imu)
            if time.monotonic()-last_image > 0.1:
                last_image = time.monotonic()
                for name, encoding, data, step in [('color/image_raw', 'rgb8', pixels.tobytes(), 1920),
                                                  ('aligned_depth_to_color/image_raw', '16UC1', depth_pixels.tobytes(), 1280)]:
                    msg = Image()
                    msg.header.stamp, msg.header.frame_id = stamp, 'bumi_odom_camera_color_optical_frame'
                    msg.height, msg.width, msg.step, msg.encoding, msg.data = 480, 640, step, encoding, data
                    pubs[name].publish(msg)
                info = CameraInfo()
                info.header = msg.header
                info.height, info.width, info.distortion_model, info.d = 480, 640, 'plumb_bob', [0.0]*5
                info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                pubs['color/camera_info'].publish(info)
            rclpy.spin_once(node, timeout_sec=0.01)
            if result['tracking'] and result['color'] >= 10 and result['depth'] >= 10:
                break
        assert result['tracking'] and result['color'] >= 10 and result['depth'] >= 10, result
        # Stop all sensor input; the quality publisher must detect real loss.
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if result.get('status', {}).get('status') == 'lost': break
        assert result['status']['status'] == 'lost', result
        print('SYNTHETIC_FUSION_OK '+json.dumps(result), flush=True)
    finally:
        for proc in reversed(processes): stop_process(proc)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
