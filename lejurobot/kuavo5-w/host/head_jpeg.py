#!/usr/bin/env python3
"""Latest-only JPEG adapter for the installed Orbbec RGB-only ROS1 stream.

Uses host OpenCV/numpy, not additional driver-image dependencies. The installed
ROS1 environment lacks compressed_image_transport; no ROS package is patched.
"""
import threading


def encode(message, cv2, np, geometry=None):
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(message.encoding)
    if (channels is None or not 1 <= message.width <= 4096 or not 1 <= message.height <= 2160
            or not message.width * channels <= message.step <= message.width * channels + 4096
            or len(message.data) != message.step * message.height or len(message.data) > 12_000_000):
        raise ValueError('unsupported or invalid RGB image')
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = image[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if geometry is not None:
        image = cv2.remap(image, geometry['color_x'], geometry['color_y'], cv2.INTER_LINEAR)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise ValueError('JPEG encode failed')
    return jpeg.tobytes()


def main():
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from rgbd_geometry import Geometry, camera_info
    rospy.init_node('motus_head_jpeg')
    lock, latest = threading.Lock(), []
    publisher = rospy.Publisher('/camera_head/color/image_rect/compressed', CompressedImage, queue_size=1)
    info_pub = rospy.Publisher('/camera_head/color/image_rect/camera_info', CameraInfo, queue_size=1)
    geometry = Geometry()
    infos = [rospy.Subscriber('/camera_head/'+kind+'/camera_info', CameraInfo,
             lambda msg, k=kind: geometry.receive(k, msg), queue_size=1) for kind in ('color', 'depth')]

    def receive(message):
        with lock:
            latest[:] = [message]  # Replace pending work rather than queue frames.

    subscriber = rospy.Subscriber('/camera_head/color/image_raw', Image, receive, queue_size=1, buff_size=2_000_000)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with lock:
            source = latest.pop() if latest else None
        if source is not None:
            try:
                message = CompressedImage()
                message.header, message.format = source.header, 'jpeg'
                g = geometry.get(cv2, np)
                message.data = encode(source, cv2, np, g)
                info_pub.publish(camera_info(g, source.header))
                publisher.publish(message)
            except Exception as exc:
                rospy.logwarn_throttle(10, 'head JPEG frame rejected: %s', type(exc).__name__)
        rate.sleep()


if __name__ == '__main__':
    main()
