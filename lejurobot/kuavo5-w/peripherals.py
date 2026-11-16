"""Verified chassis peripherals, independent of the whole-body controller."""
import base64
from array import array
import copy
import json
import math
import struct
import threading
import time

from std_msgs.msg import String, UInt8MultiArray
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

CARDS = ('battery', 'laser_scan', 'camera_left', 'camera_right', 'camera_head')
CAMERAS = {'camera_left': ('/camera_1/color/image_raw/compressed', '/camera_left/set_camera_enabled'),
           'camera_right': ('/camera_2/color/image_raw/compressed', '/camera_right/set_camera_enabled'),
           'camera_head': ('/camera_head/color/image_raw/compressed', None)}


def latest_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                      depth=1, durability=DurabilityPolicy.VOLATILE)


def number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def image_header(message):
    header = message.get('header', {})
    if not isinstance(header, dict):
        raise ValueError('invalid image header')
    frame = header.get('frame_id', '')
    stamp = header.get('stamp', {})
    if not isinstance(frame, str) or len(frame) > 256 or not isinstance(stamp, dict):
        raise ValueError('invalid image provenance')
    sec, nsec = stamp.get('secs', 0), stamp.get('nsecs', 0)
    if type(sec) is not int or type(nsec) is not int or not 0 <= sec < 2**31 or not 0 <= nsec < 1_000_000_000:
        raise ValueError('invalid image stamp')
    return {'frame_id': frame, 'sec': sec, 'nanosec': nsec}


def battery_sample(message, now_ms):
    result = {}
    mapping = {'/battery': 'main', '/jaten_behavior_manager/out/leju/battery': 'chassis',
               '/jaten_behavior_manager/out/leju/small_battery': 'small'}
    fields = {'percentage': 'soc_percent', 'voltage': 'voltage_v', 'current': 'current_a',
              'temperature': 'temperature_c', 'health': 'health_percent', 'status': 'status_code',
              'time_remaining': 'time_remaining_vendor'}
    result = {f'{prefix}_{label}': None for prefix in mapping.values() for label in fields.values()}
    result.update({f'{prefix}_battery_door_di': None for prefix in mapping.values()})
    for topic, prefix in mapping.items():
        entry = message.get('sources', {}).get(topic)
        if not isinstance(entry, dict) or not number(entry.get('received_ms')):
            continue
        if not 0 <= now_ms - entry['received_ms'] <= 5000:
            continue
        raw = entry.get('values', {})
        if not isinstance(raw, dict):
            continue
        pct = raw.get('percentage')
        if not number(pct) or not 0 <= pct <= 100:
            continue
        for key, label in fields.items():
            value = raw.get(key)
            if not number(value) or (key == 'voltage' and value <= 0):
                value = None
            result[f'{prefix}_{label}'] = value
        if isinstance(raw.get('battery_door_di'), bool):
            result[f'{prefix}_battery_door_di'] = raw['battery_door_di']
    primary = result['main_soc_percent'] is not None
    return {**result, 'valid': primary, 'reason': None if primary else 'main_battery_missing_or_stale'}


class Peripherals:
    def __init__(self, ros1, node, namespace, media_factory=None):
        self.ros1, self.node = ros1, node
        self._lock = threading.RLock()
        self._closed = False
        self._generation = None
        self._running, self._data, self._seen = set(), {}, {}
        self._images, self._last_image_publish = {}, {}
        self._enabling, self._retry_at, self._camera_error = set(), {}, {}
        # Each camera owns a WebSocket receiver and a latest-frame publisher.
        # Image JSON/base64 and DDS work cannot block control/odom replies.
        self._media = {card: media_factory() if media_factory else ros1 for card in CAMERAS}
        self._owns_media = media_factory is not None
        self._media_generations = {}
        self._workers = []
        self._wake = {card: threading.Event() for card in CAMERAS}
        self._stop = threading.Event()
        self._image_stats = {card: {'received': 0, 'published': 0, 'publish_delay_ms': None} for card in CAMERAS}
        self.topics = {card: f'/{namespace}/state/{card}' for card in CARDS}
        self._pubs = {card: node.create_publisher(String, self.topics[card], 10) for card in ('battery', 'laser_scan')}
        self._points_topic = f'/{namespace}/state/laser_scan/points'
        self._points_pub = node.create_publisher(UInt8MultiArray, self._points_topic, latest_qos())
        self._image_type = CompressedImage
        self._image_pubs = {card: node.create_publisher(CompressedImage, self.topics[card], latest_qos()) for card in CAMERAS}
        ros1.subscribe('/motus/kuavo5w/battery', 'std_msgs/String', self._on_battery)
        ros1.subscribe('/scan', 'sensor_msgs/LaserScan', self._on_scan, throttle_ms=200)
        self._timer = node.create_timer(.2, self.tick)

    def start(self):
        if self._workers:
            return
        if self._owns_media:
            for client in self._media.values():
                client.start()
        for card in CAMERAS:
            worker = threading.Thread(target=self._image_loop, args=(card,), daemon=True, name=f'{card}-publish')
            self._workers.append(worker)
            worker.start()

    def _image_loop(self, card):
        while not self._stop.is_set():
            self._wake[card].wait(.5)
            self._wake[card].clear()
            try:
                self._publish_image(card)
            except Exception as exc:
                with self._lock:
                    self._camera_error[card] = type(exc).__name__

    def _session(self):
        connected, generation = self.ros1.connection
        if not connected or generation != self._generation:
            self._data.clear()
            self._seen.clear()
            self._images.clear()
            self._last_image_publish.clear()
            self._retry_at.clear()
            self._generation = generation
        return connected

    def _on_battery(self, message):
        with self._lock:
            if not self._session():
                return
            try:
                raw = message.get('data')
                if not isinstance(raw, str) or len(raw) > 32768:
                    raise ValueError('invalid battery envelope')
                parsed = json.loads(raw)
                if not isinstance(parsed, dict) or not isinstance(parsed.get('sources'), dict):
                    raise ValueError('invalid battery sources')
                self._data['battery'] = parsed
                self._seen['battery'] = time.monotonic()
            except (ValueError, TypeError):
                self._data.pop('battery', None)

    def _on_scan(self, message):
        with self._lock:
            if not self._session():
                return
            try:
                ranges = message['ranges']
                fields = [message[k] for k in ('angle_min', 'angle_increment', 'range_min', 'range_max')]
                if (not isinstance(ranges, list) or not 1 <= len(ranges) <= 20000
                        or not all(map(number, fields)) or fields[1] <= 0 or not 0 <= fields[2] < fields[3]
                        or abs(fields[0]) > 2 * math.pi or fields[1] * (len(ranges) - 1) > 4 * math.pi):
                    raise ValueError('invalid LaserScan')
                low, high = fields[2:]
                clean = [v if number(v) and low <= v <= high else None for v in ranges]
                header = message.get('header', {})
                if not isinstance(header, dict) or not isinstance(header.get('frame_id', ''), str):
                    raise ValueError('invalid laser header')
                self._data['laser_scan'] = {'valid': True, 'frame_id': header.get('frame_id', '')[:256],
                    **dict(zip(('angle_min', 'angle_increment', 'range_min', 'range_max'), fields)),
                    'ranges': clean, 'units': {'range': 'm', 'angle': 'rad'}}
                self._seen['laser_scan'] = time.monotonic()
            except (KeyError, TypeError, ValueError):
                self._data.pop('laser_scan', None)

    def _on_image(self, card, message, generation):
        with self._lock:
            if (not self._media[card].connection[0] or generation != self._media[card].connection[1]
                    or card not in self._running or self._closed):
                return
            try:
                data = message['data']
                if not isinstance(data, str) or len(data) > 1_500_000:
                    raise ValueError('oversized/non-base64 JPEG')
                jpeg = base64.b64decode(data, validate=True)
                if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                    raise ValueError('not a complete JPEG')
                self._images[card] = (jpeg, image_header(message))
                self._data[card] = {'valid': True, 'bytes': len(jpeg), 'encoding': 'jpeg',
                                    'source_topic': CAMERAS[card][0]}
                self._seen[card] = time.monotonic()
                self._image_stats[card]['received'] += 1
                self._wake[card].set()
            except (KeyError, ValueError, TypeError):
                self._data.pop(card, None)
                self._images.pop(card, None)

    def _enable_camera(self, card, generation):
        try:
            result = self.ros1.call_service(CAMERAS[card][1], 'std_srvs/SetBool', {'data': True}, timeout=2)
            if result.get('success') is not True:
                raise RuntimeError('camera service did not accept enable')
            with self._lock:
                if generation == self._media[card].connection[1]:
                    self._camera_error.pop(card, None)
        except Exception as exc:
            with self._lock:
                self._camera_error[card] = type(exc).__name__
        finally:
            with self._lock:
                self._enabling.discard(card)
                self._retry_at[card] = time.monotonic() + 5

    def snapshot(self, card):
        with self._lock:
            return self._snapshot(card)

    def _snapshot(self, card):
        connected = self._media[card].connection[0] if card in CAMERAS else self._session()
        age = time.monotonic() - self._seen.get(card, -1e9)
        valid = (connected and age <= (5 if card == 'battery' else 1) and card in self._data
                 and (card not in CAMERAS or self._media_generations.get(card) == self._media[card].connection))
        if card == 'battery':
            return {**battery_sample(self._data[card] if valid else {'sources': {}}, int(time.time() * 1000)),
                    'connected': connected}
        return {**(copy.deepcopy(self._data.get(card, {})) if valid else {}),
                **(self._image_stats[card] if card in CAMERAS else {}),
                'valid': valid, 'connected': connected,
                'reason': None if valid else self._camera_error.get(card, 'missing_or_stale_measurement')}

    def tools(self):
        descriptions = {'battery': '实测电池 SOC/电压/电流/温度；保留三路来源及冲突，0V 不冒充实测电压',
            'laser_scan': '二维雷达点云图 + 原始测距（laser 坐标系，z=0；不是额外3D雷达）',
            'camera_left': '底盘左路 RGB JPEG，独立接收/发布线程，自动开启',
            'camera_right': '底盘右路 RGB JPEG，独立接收/发布线程，自动开启',
            'camera_head': 'Orbbec Gemini 335L 头部 RGB JPEG，独立接收/发布线程'}
        tools = []
        for card in CARDS:
            outputs = [{'topic': self.topics[card], 'format': 'image/jpeg' if card in CAMERAS else 'data/json'}]
            if card == 'laser_scan':
                outputs.insert(0, {'topic': self._points_topic, 'format': 'sensor/pointcloud',
                                   'desc': '原始 laser 平面点云，z=0m'})
            tools.append({'name': card, 'type': 'sensor', 'multiInstance': False, 'description': descriptions[card],
                          'inputSchema': {'type': 'object', 'properties': {}}, 'topic_out': outputs})
        return tools

    def dispatch(self, card, action):
        with self._lock:
            self._session()
            if action == 'start':
                self._running.add(card)
                if card in CAMERAS:
                    self._subscribe_camera(card)
            elif action == 'stop':
                self._running.discard(card)
                if card in CAMERAS:
                    self._media[card].unsubscribe(CAMERAS[card][0])
                    self._data.pop(card, None)
                    self._images.pop(card, None)
                # Physical streams are shared by the chassis: do not disable them.
            elif action != 'info':
                return {'state': 'error', 'error': 'unsupported sensor lifecycle action'}
            result = {'state': 'running' if card in self._running else 'idle', 'feedback': self.snapshot(card)}
            if card in self._running:
                result['topic_out'] = next(t['topic_out'] for t in self.tools() if t['name'] == card)
            return result

    def tick(self):
        with self._lock:
            if self._closed or not self._session():
                return
            for card in self._running:
                value = self.snapshot(card)
                if card in CAMERAS:
                    client = self._media[card]
                    if self._media_generations.get(card) != client.connection:
                        self._images.pop(card, None)
                        self._data.pop(card, None)
                        self._seen.pop(card, None)
                        self._subscribe_camera(card)
                    if (CAMERAS[card][1] and not value['valid'] and card not in self._enabling
                            and time.monotonic() >= self._retry_at.get(card, 0)):
                        self._enabling.add(card)
                        # Rebind callback to the current WebSocket generation.
                        generation = client.connection[1]
                        self._subscribe_camera(card)
                        threading.Thread(target=self._enable_camera, args=(card, generation), daemon=True).start()
                    if not self._workers:
                        self._publish_image(card)
                else:
                    msg = String()
                    msg.data = json.dumps(value, allow_nan=False, separators=(',', ':'))
                    self._pubs[card].publish(msg)
                    if card == 'laser_scan':
                        points = []
                        if value['valid']:
                            for i in range(len(value['ranges'])):
                                distance = value['ranges'][i]
                                if distance is not None:
                                    angle = value['angle_min'] + i * value['angle_increment']
                                    points.append((distance * math.cos(angle), distance * math.sin(angle), 0., 0.))
                        cloud = UInt8MultiArray()
                        cloud.data = array('B', struct.pack('<II', 16, len(points))
                                           + b''.join(struct.pack('<ffff', *p) for p in points))
                        self._points_pub.publish(cloud)

    def _subscribe_camera(self, card):
        client = self._media[card]
        generation = client.connection[1]
        self._media_generations[card] = client.connection
        client.subscribe(CAMERAS[card][0], 'sensor_msgs/CompressedImage',
            lambda msg, c=card, g=generation: self._on_image(c, msg, g), throttle_ms=100)

    def _publish_image(self, card):
        with self._lock:
            seen = self._seen.get(card)
            if (self._closed or card not in self._running or not self.snapshot(card)['valid']
                    or seen == self._last_image_publish.get(card)):
                return
            jpeg, header = self._images[card]
            self._last_image_publish[card] = seen
        msg = self._image_type()
        # Generated ROS2 uint8 setters validate bytes element-by-element under
        # __debug__. A typed array takes their native fast path (no Python loop).
        msg.format, msg.data = 'jpeg', array('B', jpeg)
        msg.header.frame_id = header['frame_id']
        msg.header.stamp.sec, msg.header.stamp.nanosec = header['sec'], header['nanosec']
        self._image_pubs[card].publish(msg)  # No shared lock held during DDS serialization.
        with self._lock:
            self._image_stats[card]['published'] += 1
            self._image_stats[card]['publish_delay_ms'] = round((time.monotonic() - seen) * 1000, 2)

    def close(self):
        with self._lock:
            self._closed = True
            self._stop.set()
            for card in CAMERAS:
                self._media[card].unsubscribe(CAMERAS[card][0])
                self._wake[card].set()
            self._running.clear()
        if self._owns_media:
            for client in self._media.values():
                client.close()
        for worker in self._workers:
            worker.join(timeout=2)
