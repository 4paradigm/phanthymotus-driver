"""Read calibrated depth already published by a site's ROS camera driver."""

import math
import os

from agilex.piper.camera_worker import capture_frames


def depth_pixels(message, width, height, frame_id):
    import numpy as np

    if (message.width, message.height, message.encoding, message.header.frame_id) != (
            width, height, '16UC1', frame_id):
        raise ValueError('ROS depth does not match the configured dimensions, encoding, or optical frame')
    if message.step < 2 * width or len(message.data) != height * message.step:
        raise ValueError('Invalid ROS depth row stride or buffer length')
    dtype = '>u2' if message.is_bigendian else '<u2'
    return np.ndarray((height, width), dtype=dtype, buffer=message.data,
                      strides=(message.step, 2)).astype(np.uint16, copy=False)


def capture_ros_depth(settings, mailbox, stop, encode):
    if settings['ros_profile']:
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = settings['ros_profile']
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CameraInfo, Image

    width, height, frame_id = settings['width'], settings['height'], settings['ros_frame_id']
    source = settings['ros_source']
    aligned = settings['ros_color_aligned']
    image_name = 'image_raw' if aligned else 'image_rect_raw'
    image_qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
    latest = calibration = None
    stamp = 0.0

    def on_image(message):
        nonlocal latest
        latest = message

    def on_info(message):
        nonlocal calibration
        if (message.width, message.height, message.header.frame_id) != (width, height, frame_id):
            raise ValueError('ROS CameraInfo does not match the configured depth stream')
        k, d = list(message.k), list(message.d)
        if k[0] <= 0 or k[4] <= 0 or not all(map(math.isfinite, (*k, *d))):
            raise ValueError('Invalid ROS depth intrinsics')
        metadata = dict(K=k, D=d, source='derived-from-K', distortion_model=(
            'pinhole' if message.distortion_model == 'plumb_bob' and not any(d) else 'unknown'),
            pipeline=['align-depth-to-color'] if aligned else [],
            vendor={'ros_frame_id': frame_id})
        if metadata != calibration:
            mailbox.set_metadata(metadata)
            calibration = metadata

    context = Context()
    rclpy.init(context=context, domain_id=settings['ros_domain'], signal_handler_options=SignalHandlerOptions.NO)
    executor = node = None
    try:
        executor = SingleThreadedExecutor(context=context)
        node = rclpy.create_node('piper_ros_depth_input', context=context)
        node.create_subscription(Image, f'{source}/{image_name}', on_image, image_qos)
        node.create_subscription(CameraInfo, source + '/camera_info', on_info, qos_profile_sensor_data)
        executor.add_node(node)

        def read():
            nonlocal latest, stamp
            executor.spin_once(timeout_sec=.2)
            if latest is None or calibration is None:
                return None
            message, latest = latest, None
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            return depth_pixels(message, width, height, frame_id)

        capture_frames(settings, mailbox, stop, read, lambda frame, _: encode(frame, .001),
                       frame_time=lambda: stamp)
    finally:
        if node is not None:
            if executor is not None:
                executor.remove_node(node)
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        rclpy.try_shutdown(context=context)
