#!/usr/bin/env python3
"""One bounded, capture-paired rectification owner for both public image cards."""
from collections import deque
import threading


def capture_ns(message):
    return message.header.stamp.to_nsec()


class Pairs:
    def __init__(self):
        self.lock = threading.Lock()
        # Registered depth arrives several frames after RGB on the host even
        # when their capture times agree. Retain a bounded second at 15 Hz;
        # a four-frame queue discarded RGB before its depth partner arrived.
        self.frames = {'color': deque(maxlen=16), 'depth': deque(maxlen=16)}
        self.latest = None
        self.last = {'color': 0, 'depth': 0}

    def receive(self, kind, message):
        ns = capture_ns(message)
        if ns <= 0:
            return
        with self.lock:
            if ns <= self.last[kind]:
                return
            self.last[kind] = ns
            self.frames[kind].append(message)
            colors, depths = self.frames['color'], self.frames['depth']
            while colors and depths:
                color, depth = colors[0], depths[0]
                delta = capture_ns(color)-capture_ns(depth)
                if abs(delta) <= 50_000_000:
                    self.latest = colors.popleft(), depths.popleft()
                elif delta < 0:
                    colors.popleft()
                else:
                    depths.popleft()

    def take(self):
        with self.lock:
            pair, self.latest = self.latest, None
            return pair


def main():
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from rgbd_geometry import Geometry, camera_info
    from head_jpeg import encode as jpeg
    from head_depth import encode as depth_zlib
    rospy.init_node('motus_head_rgbd')
    pairs, geometry = Pairs(), Geometry()
    pubs = {kind: rospy.Publisher('/camera_head/'+kind+'/image_rect/'+suffix,
            CompressedImage, queue_size=1) for kind, suffix in
            [('color','compressed'), ('depth','compressed_zlib')]}
    infos = {kind: rospy.Publisher('/camera_head/'+kind+'/image_rect/camera_info',
             CameraInfo, queue_size=1) for kind in ('color','depth')}
    subs = []
    for kind in ('color','depth'):
        subs.append(rospy.Subscriber('/camera_head/'+kind+'/camera_info', CameraInfo,
            lambda msg,k=kind:geometry.receive(k,msg), queue_size=1))
        subs.append(rospy.Subscriber('/camera_head/'+kind+'/image_raw', Image,
            lambda msg,k=kind:pairs.receive(k,msg), queue_size=2,
            buff_size=3_000_000, tcp_nodelay=True))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pair = pairs.take()
        if pair is not None:
            try:
                color, depth = pair
                g = geometry.get(cv2,np)
                if any(m.header.frame_id != g['info'].header.frame_id for m in pair):
                    raise ValueError('image frames do not match calibrated optical rays')
                payloads = {'color':jpeg(color,cv2,np,g), 'depth':depth_zlib(depth,g,cv2,np)}
                # Each retains its original source time, not encoding/receipt time.
                for kind, source in [('color',color), ('depth',depth)]:
                    out = CompressedImage()
                    out.header, out.data = source.header, payloads[kind]
                    out.format = 'jpeg' if kind == 'color' else 'image/depth-zlib'
                    infos[kind].publish(camera_info(g,source.header))
                    pubs[kind].publish(out)
            except Exception as error:
                rospy.logwarn_throttle(10, 'head RGB-D pair rejected: %s: %s',type(error).__name__,str(error)[:180])
        rate.sleep()


if __name__ == '__main__':
    main()
