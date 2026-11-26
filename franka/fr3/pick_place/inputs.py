"""Bounded, stationary RGB-D/VOP snapshots; never opens a camera device."""
from collections import deque
import copy
import json
import math
import threading
import time
import zlib

import numpy as np
from .alignment import align_depth


def resolve_topics(args):
    topics = args.get('input_topics') or []
    if not isinstance(topics, list) or len(topics) != 3:
        raise ValueError('Connect RGB, depth and VOP objects inputs')
    ordered = []
    for suffix in ('/rgb', '/depth', '/objects'):
        found = [t for t in topics if isinstance(t, str) and t.startswith('/') and t.endswith(suffix)]
        if len(found) != 1:
            raise ValueError('Expected one RGB, depth and objects topic')
        ordered.append(found[0])
    if ordered[2] != ordered[0] + '/objects':
        raise ValueError('VOP must consume the selected RGB topic')
    return ordered


def detection(value):
    if not isinstance(value, dict):
        raise ValueError('Invalid VOP payload')
    stamp, latency = value.get('timestamp'), value.get('latency_ms', 0)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (stamp, latency)) or latency < 0:
        raise ValueError('Invalid VOP timestamps')
    objects = value.get('objects')
    if not isinstance(objects, list) or len(objects) > 1000:
        raise ValueError('Invalid VOP objects')
    clean = []
    for obj in objects:
        name, pos, confidence = obj.get('name'), obj.get('position'), obj.get('confidence')
        if (not isinstance(name, str) or not name or len(name.encode()) > 256 or
                not isinstance(pos, list) or len(pos) != 2 or
                any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1 for x in pos) or
                type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError('Invalid VOP name, coordinates or confidence')
        clean.append(dict(name=name, position=list(pos), confidence=confidence))
    return dict(timestamp=stamp, latency_ms=latency, objects=clean)


class ObservationInputs:
    def __init__(self, ros2, camera, serial):
        self.ros2, self.camera, self.serial = ros2, camera, serial
        self.lock = threading.RLock()
        self.topics = []
        self.buffers = {k: deque(maxlen=30) for k in ('rgb', 'depth', 'objects')}
        self.errors = {}
        self.source_ready = False
        self.source_error = None
        self.thread = None
        self.quit = threading.Event()

    def select_serial(self, serial):
        with self.lock:
            self.serial = serial
            self.errors.clear()
            self.source_ready = False
            for buffer in self.buffers.values(): buffer.clear()

    def prepare(self):
        """Keep connected topics stable while selecting the chosen wrist device."""
        if len(self.topics) != 3:
            raise RuntimeError('Start the three observation inputs first')
        with self.camera._lock:
            views = [(i, self.camera._info(i)) for i in self.camera._nodes]
        routes = []
        for topic, channel in zip(self.topics[:2], ('rgb', 'depth')):
            matches = [(i, v) for i, v in views if v['channel'] == channel and
                       any(t['topic'] == topic for t in v['topic_out'])]
            if len(matches) != 1:
                raise RuntimeError('Connect one RGB and one depth camera card')
            routes.append((matches[0][0], matches[0][1], channel))
        for instance, view, channel in routes:
            if view.get('serial_number') != self.serial:
                self.camera.dispatch('start', dict(instance_id=instance,
                    device_path='realsense://'+self.serial, channel=channel))
        deadline = time.monotonic()+8
        while True:
            try:
                self.binding()
                return
            except RuntimeError:
                if time.monotonic() >= deadline: raise
                time.sleep(.1)

    def binding(self):
        with self.camera._lock:
            views = [self.camera._info(i) for i in self.camera._nodes]
        selected = []
        for topic, channel in zip(self.topics[:2], ('rgb', 'depth')):
            matches = [v for v in views if v['channel'] == channel and
                       any(t['topic'] == topic for t in v['topic_out'])]
            if len(matches) != 1 or matches[0].get('serial_number') != self.serial or not matches[0].get('fresh'):
                raise RuntimeError('RGB and depth must be fresh streams from the configured wrist camera')
            selected.append(matches[0])
        if len(selected) != 2:
            raise RuntimeError('Start the three observation inputs first')
        a, b = (v.get('calibration') for v in selected)
        if not a or a != b or a.get('serial_number') != self.serial:
            raise RuntimeError('Camera calibration/session does not match RGB and depth')
        return copy.deepcopy(a)

    def start(self, args):
        topics = resolve_topics(args)
        if self.thread and self.topics == topics:
            self.prepare()
            return
        self.stop()
        self.topics = topics
        self.prepare()
        self.quit = threading.Event()
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        node = Node('fr3_pick_place_inputs', context=self.ros2.ctx_core)
        executor = SingleThreadedExecutor(context=self.ros2.ctx_core)
        try:
            for kind, topic, cls in zip(('rgb', 'depth', 'objects'), topics,
                                        (CompressedImage, CompressedImage, String)):
                node.create_subscription(cls, topic, lambda msg, k=kind: self.receive(k, msg), qos_profile_sensor_data)
            executor.add_node(node)
        except Exception:
            executor.shutdown(); node.destroy_node(); raise
        def run():
            try:
                while not self.quit.is_set():
                    executor.spin_once(timeout_sec=.1)
                    endpoints = [node.get_publishers_info_by_topic(t) for t in topics]
                    valid = (all(len(e) == 1 for e in endpoints) and
                             (endpoints[0][0].node_name, endpoints[0][0].node_namespace) ==
                             (endpoints[1][0].node_name, endpoints[1][0].node_namespace))
                    with self.lock:
                        self.source_ready = valid
                        self.source_error = (None if valid else
                            'Expected one publisher per input and matching RGB/depth source; publisher counts: '+
                            ', '.join(f'{topic}={len(rows)}' for topic,rows in zip(topics,endpoints)))
            finally:
                executor.shutdown(); node.destroy_node()
        self.thread = threading.Thread(target=run, daemon=True, name='fr3-observation-inputs')
        self.thread.start()

    def stop(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise RuntimeError('Observation input reader did not stop')
        self.thread = None
        with self.lock:
            self.topics = []; self.errors.clear()
            self.source_ready = False
            for buf in self.buffers.values(): buf.clear()

    def info(self):
        with self.lock:
            fresh = all(buf and time.time()-buf[-1][0] < 1 for buf in self.buffers.values())
            return dict(state='error' if self.errors else 'running' if fresh and self.source_ready else 'starting' if self.thread else 'idle',
                        fresh=fresh and self.source_ready, errors=dict(self.errors),waiting_for=self.source_error)

    def receive(self, kind, msg):
        serial = self.serial
        try:
            if kind == 'objects':
                if len(msg.data.encode()) > 256 * 1024: raise ValueError('VOP payload too large')
                value = detection(json.loads(msg.data)); stamp = value['timestamp']
            else:
                if len(msg.data) > 5 * 1024 * 1024: raise ValueError('Image payload too large')
                expected = 'jpeg' if kind == 'rgb' else '16UC1; compressedDepth zlib'
                if msg.format != expected: raise ValueError('Unexpected image encoding')
                if not msg.header.frame_id.endswith(f'serial_{serial}_{kind}_optical'):
                    raise ValueError('Image optical frame does not match the configured wrist camera')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                value = bytes(msg.data)
            if not 0 <= time.time() - stamp < 1: return
            with self.lock:
                if self.serial != serial: return
                self.buffers[kind].append((stamp, value)); self.errors.pop(kind, None)
        except Exception as exc:
            with self.lock: self.errors[kind] = str(exc)

    def snapshot(self, after, cancel, check):
        import cv2
        calibration = self.binding()
        baseline = None
        steady_since = time.time()
        deadline = time.monotonic()+30
        while not cancel.wait(.05):
            check()
            if time.monotonic()>=deadline:
                raise TimeoutError('Observation inputs did not synchronize within 30 seconds: '+
                                   (self.source_error or str(self.info())))
            if self.binding() != calibration:
                raise RuntimeError('Camera source or calibration changed during observation')
            with self.lock:
                if self.errors: raise RuntimeError('; '.join(self.errors.values()))
                if not self.source_ready: continue
                rows = {k: list(v) for k, v in self.buffers.items()}
            if not all(rows.values()): continue
            stamp, raw = rows['rgb'][-1]
            if stamp < after or time.time()-stamp > 1: continue
            im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            intr = calibration['rgb_intrinsics']
            if im is None or im.shape[:2] != (intr['height'], intr['width']):
                raise RuntimeError('RGB dimensions differ from calibration')
            thumb = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (64, 36)).astype(float)
            if baseline is None or np.mean(np.abs(thumb-baseline)) > 4:
                baseline, steady_since = thumb, stamp
            if stamp-steady_since < .6: continue
            choices = [(s, d) for s, d in rows['objects'] if s-d['latency_ms']/1000 > max(after, steady_since)]
            if not choices: continue
            ds, objects = choices[-1]; source_stamp = ds-objects['latency_ms']/1000
            rs, jpeg = min(rows['rgb'], key=lambda x: abs(x[0]-source_stamp))
            zs, depth = min(rows['depth'], key=lambda x: abs(x[0]-rs))
            if max(abs(rs-source_stamp), abs(zs-rs)) > .2 or time.time()-min(rs,zs,ds) > 1: continue
            aligned = align_depth(depth, calibration)
            check()
            with self.lock:
                if not self.source_ready or self.errors:
                    raise RuntimeError('Observation input source changed during depth alignment')
            return dict(jpeg=jpeg, depth_zlib=zlib.compress(aligned.tobytes(), 1),
                        captured_at=rs, width=intr['width'], height=intr['height'],
                        objects=objects['objects'], objects_timestamp=ds,
                        calibration=calibration, input_identity=dict(camera_serial=self.serial,
                            session_id=calibration['session_id'], topics=list(self.topics)),
                        synchronization=dict(rgb_depth_skew_ms=round(abs(rs-zs)*1000, 2),
                            detection_rgb_skew_ms=round(abs(source_stamp-rs)*1000, 2),
                            timestamp_source='publication_clock', hardware_capture_timestamp_available=False))
        raise RuntimeError('Observation cancelled')
