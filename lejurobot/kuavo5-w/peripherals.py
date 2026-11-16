"""Verified chassis peripherals, independent of the whole-body controller."""
import base64
import copy
import json
import math
import threading
import time

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

CARDS = ('battery', 'laser_scan', 'camera_rgb', 'camera_rgb_right')
CAMERAS = {'camera_rgb': ('/camera_1/color/image_raw/compressed', '/camera_left/set_camera_enabled'),
           'camera_rgb_right': ('/camera_2/color/image_raw/compressed', '/camera_right/set_camera_enabled')}


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
    sources = {}
    for topic, entry in message.get('sources', {}).items():
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
        values = {key: value if number(value) else None for key, value in raw.items()
                  if key in ('voltage', 'current', 'temperature', 'health', 'status', 'time_remaining')}
        if values.get('voltage') is not None and values['voltage'] <= 0:
            values['voltage'] = None
        safe_raw = {key: value if number(value) or isinstance(value, bool) else None
                    for key, value in raw.items() if key in ('percentage', 'battery_door_di', *values)}
        sources[topic] = {'soc': pct, **values, 'received_ms': entry['received_ms'], 'vendor': safe_raw}
    primary = sources.get('/battery')
    return {'valid': primary is not None, **(primary or {}), 'sources': sources,
            'units': {'soc': '%', 'voltage': 'V', 'current': 'A', 'temperature': 'degC'},
            'reason': None if primary else 'main_battery_missing_or_stale'}


class Peripherals:
    def __init__(self, ros1, node, namespace):
        self.ros1, self.node = ros1, node
        self._lock = threading.RLock()
        self._closed = False
        self._generation = None
        self._running, self._data, self._seen = set(), {}, {}
        self._images, self._last_image_publish = {}, {}
        self._enabling, self._retry_at, self._camera_error = set(), {}, {}
        self.topics = {card: f'/{namespace}/state/{card}' for card in CARDS}
        self._pubs = {card: node.create_publisher(String, self.topics[card], 10) for card in ('battery', 'laser_scan')}
        self._points_topic = f'/{namespace}/state/laser_scan/points'
        self._points_pub = node.create_publisher(String, self._points_topic, 10)
        self._image_type = CompressedImage
        self._image_pubs = {card: node.create_publisher(CompressedImage, self.topics[card], 2) for card in CAMERAS}
        ros1.subscribe('/motus/kuavo5w/battery', 'std_msgs/String', self._on_battery)
        ros1.subscribe('/scan', 'sensor_msgs/LaserScan', self._on_scan, throttle_ms=200)
        self._timer = node.create_timer(.2, self.tick)

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
            if not self._session() or generation != self._generation or card not in self._running:
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
            except (KeyError, ValueError, TypeError):
                self._data.pop(card, None)
                self._images.pop(card, None)

    def _enable_camera(self, card, generation):
        try:
            result = self.ros1.call_service(CAMERAS[card][1], 'std_srvs/SetBool', {'data': True}, timeout=2)
            if result.get('success') is not True:
                raise RuntimeError('camera service did not accept enable')
            with self._lock:
                if generation == self._generation:
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
        connected = self._session()
        age = time.monotonic() - self._seen.get(card, -1e9)
        valid = connected and age <= (5 if card == 'battery' else 1) and card in self._data
        if valid and card == 'battery':
            return {**battery_sample(self._data[card], int(time.time() * 1000)), 'connected': True}
        return {**(copy.deepcopy(self._data.get(card, {})) if valid else {}),
                'valid': valid, 'connected': connected,
                'reason': None if valid else self._camera_error.get(card, 'missing_or_stale_measurement')}

    def tools(self):
        descriptions = {'battery': '实测电池 SOC/电压/电流/温度；保留三路来源及冲突，0V 不冒充实测电压',
            'laser_scan': '二维雷达测距及俯视点集（laser 原始坐标系，不冒充底盘 TF）',
            'camera_rgb': 'camera_1 左路 RGB JPEG，start 自动开启相机，不控制机器人运动',
            'camera_rgb_right': 'camera_2 右路 RGB JPEG，start 自动开启相机'}
        tools = []
        for card in CARDS:
            outputs = [{'topic': self.topics[card], 'format': 'image/jpeg' if card in CAMERAS else 'data/json'}]
            if card == 'laser_scan':
                outputs.append({'topic': self._points_topic, 'format': 'sensor/lidar'})
            tools.append({'name': card, 'type': 'sensor', 'multiInstance': False, 'description': descriptions[card],
                          'inputSchema': {'type': 'object', 'properties': {}}, 'topic_out': outputs})
        return tools

    def dispatch(self, card, action):
        with self._lock:
            self._session()
            if action == 'start':
                self._running.add(card)
                if card in CAMERAS:
                    generation = self._generation
                    self.ros1.subscribe(CAMERAS[card][0], 'sensor_msgs/CompressedImage',
                        lambda msg, c=card, g=generation: self._on_image(c, msg, g), throttle_ms=200)
            elif action == 'stop':
                self._running.discard(card)
                if card in CAMERAS:
                    self.ros1.unsubscribe(CAMERAS[card][0])
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
                    if not value['valid'] and card not in self._enabling and time.monotonic() >= self._retry_at.get(card, 0):
                        self._enabling.add(card)
                        # Rebind callback to the current WebSocket generation.
                        generation = self._generation
                        self.ros1.subscribe(CAMERAS[card][0], 'sensor_msgs/CompressedImage',
                            lambda msg, c=card, g=generation: self._on_image(c, msg, g), throttle_ms=200)
                        threading.Thread(target=self._enable_camera, args=(card, generation), daemon=True).start()
                    seen = self._seen.get(card)
                    if value['valid'] and seen != self._last_image_publish.get(card):
                        jpeg, header = self._images[card]
                        msg = self._image_type()
                        msg.format, msg.data = 'jpeg', jpeg
                        msg.header.frame_id = header.get('frame_id', '')
                        msg.header.stamp.sec = header['sec']
                        msg.header.stamp.nanosec = header['nanosec']
                        self._image_pubs[card].publish(msg)
                        self._last_image_publish[card] = seen
                else:
                    msg = String()
                    msg.data = json.dumps(value, allow_nan=False, separators=(',', ':'))
                    self._pubs[card].publish(msg)
                    if card == 'laser_scan':
                        points = []
                        if value['valid']:
                            step = max(1, (len(value['ranges']) + 1999) // 2000)
                            for i in range(0, len(value['ranges']), step):
                                distance = value['ranges'][i]
                                if distance is not None:
                                    angle = value['angle_min'] + i * value['angle_increment']
                                    points.append({'x': distance * math.cos(angle), 'y': distance * math.sin(angle)})
                        cloud = String()
                        cloud.data = json.dumps(points, allow_nan=False, separators=(',', ':'))
                        self._points_pub.publish(cloud)

    def close(self):
        with self._lock:
            self._closed = True
            for card in CAMERAS:
                self.ros1.unsubscribe(CAMERAS[card][0])
            self._running.clear()
