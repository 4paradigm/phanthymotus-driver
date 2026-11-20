#!/usr/bin/env python3
"""Latest-only, lossless depth relay for the installed Orbbec ROS1 driver.

Orbbec's publishFrame multiplies Y16 by getValueScale before publishing 16UC1
millimetres. Dashboard image/depth-zlib fixes 640x480 little-endian uint16 mm;
zero stays invalid. This is zlib, NOT ROS compressedDepth's PNG/RVL envelope.
"""
import threading
import zlib

WIDTH, HEIGHT = 640, 480
FORMAT = 'image/depth-zlib'


def encode(message, geometry=None, cv2=None, np=None):
    if (message.encoding != '16UC1' or message.width != WIDTH or message.height != HEIGHT
            or not WIDTH * 2 <= message.step <= WIDTH * 2 + 4096
            or len(message.data) != message.step * HEIGHT
            or message.is_bigendian not in (0, 1)):
        raise ValueError('expected 640x480 Orbbec 16UC1 millimetres')
    raw = bytes(message.data)
    if message.step != WIDTH * 2:
        raw = b''.join(raw[y * message.step:y * message.step + WIDTH * 2] for y in range(HEIGHT))
    if message.is_bigendian:
        swapped = bytearray(len(raw))
        swapped[0::2], swapped[1::2] = raw[1::2], raw[0::2]
        raw = bytes(swapped)
    if geometry is not None:
        image = np.frombuffer(raw, dtype='<u2').reshape(HEIGHT, WIDTH)
        raw = cv2.remap(image, geometry['depth_x'], geometry['depth_y'], cv2.INTER_NEAREST).astype('<u2').tobytes()
    return zlib.compress(raw, 1)


def main():
    import rospy
    import cv2
    import numpy as np
    from rgbd_geometry import Geometry, camera_info
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    rospy.init_node('motus_head_depth')
    lock, latest = threading.Lock(), []
    publisher = rospy.Publisher('/camera_head/depth/image_rect/compressed_zlib', CompressedImage, queue_size=1)
    info_pub = rospy.Publisher('/camera_head/depth/image_rect/camera_info', CameraInfo, queue_size=1)
    geometry = Geometry()
    infos = [rospy.Subscriber('/camera_head/'+kind+'/camera_info', CameraInfo,
             lambda msg, k=kind: geometry.receive(k, msg), queue_size=1) for kind in ('color', 'depth')]

    def receive(message):
        with lock:
            latest[:] = [message]

    subscriber = rospy.Subscriber('/camera_head/depth/image_raw', Image, receive,
                                  queue_size=1, buff_size=2_000_000, tcp_nodelay=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with lock:
            source = latest.pop() if latest else None
        if source is not None:
            try:
                message = CompressedImage()
                message.header, message.format = source.header, FORMAT
                g = geometry.get(cv2, np)
                message.data = encode(source, g, cv2, np)
                info_pub.publish(camera_info(g, source.header))
                publisher.publish(message)
            except Exception as exc:
                rospy.logwarn_throttle(10, 'head depth frame rejected: %s', type(exc).__name__)
        rate.sleep()


if __name__ == '__main__':
    main()
