"""Verified chassis peripherals, independent of the whole-body controller."""
import base64
from array import array
import copy
import json
import math
import struct
import threading
import time
import zlib

from std_msgs.msg import String, UInt8MultiArray
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from camera_specs import INFO_TOPICS, calibration, declare, jpeg_dimensions, registered_pair
from base_servo import footprint_from_polygon, footprint_from_model

CARDS = ('battery', 'laser_scan', 'camera_left', 'camera_right', 'camera_head', 'camera_depth')
CAMERAS = {'camera_left': ('/camera_1/color/image_raw/compressed', '/camera_left/set_camera_enabled'),
           'camera_right': ('/camera_2/color/image_raw/compressed', '/camera_right/set_camera_enabled'),
           'camera_head': ('/camera_head/color/image_rect/compressed', None),
           'camera_depth': ('/camera_head/depth/image_rect/compressed_zlib', None)}
DEPTH_FORMAT = 'image/depth-zlib'
DEPTH_BYTES = 640 * 480 * 2


def validate_depth(payload):
    # Bound decompression even for untrusted rosbridge packets; reject appended
    # streams, trailing bytes and raw/PNG/RVL data with a misleading format tag.
    decoder = zlib.decompressobj()
    raw = decoder.decompress(payload, DEPTH_BYTES + 1)
    if (len(raw) != DEPTH_BYTES or not decoder.eof or decoder.unconsumed_tail or decoder.unused_data):
        raise ValueError('invalid 640x480 uint16 depth stream')


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
    def __init__(self, ros1, node, namespace, media_factory=None, camera_ensure=None):
        self.ros1, self.node = ros1, node
        self._lock = threading.RLock()
        self._closed = False
        self._camera_ensure = camera_ensure
        self._generation = None
        self._running, self._data, self._seen = set(), {}, {}
        self._images, self._last_image_publish = {}, {}
        self._calibrations, self._dimensions = {}, {}
        self._depth_registration, self._footprint = None, None
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
        # Released navi versions identify depth by this suffix before DDS
        # discovery completes; they do not read explicit format declarations.
        # Keep the old wire endpoint as an alias for existing direct readers.
        self._legacy_depth_topic = self.topics['camera_depth']
        self.topics['camera_depth'] = f'/{namespace}/state/camera_head/visual_depth'
        self._pubs = {card: node.create_publisher(String, self.topics[card], 10) for card in ('battery', 'laser_scan')}
        self._points_topic = f'/{namespace}/state/laser_scan/points'
        self._points_pub = node.create_publisher(UInt8MultiArray, self._points_topic, latest_qos())
        self._image_type = CompressedImage
        self._image_pubs = {card: node.create_publisher(CompressedImage, self.topics[card], latest_qos()) for card in CAMERAS}
        self._legacy_depth_pub = node.create_publisher(CompressedImage, self._legacy_depth_topic, latest_qos())
        ros1.subscribe('/motus/kuavo5w/battery', 'std_msgs/String', self._on_battery)
        ros1.subscribe('/scan', 'sensor_msgs/LaserScan', self._on_scan, throttle_ms=200)
        for card, topic in INFO_TOPICS.items():
            ros1.subscribe(topic, 'sensor_msgs/CameraInfo',
                           lambda msg, c=card: self._on_camera_info(c, msg), throttle_ms=1000)
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
        worker = threading.Thread(target=self._metadata_loop, daemon=True, name='camera-geometry')
        self._workers.append(worker)
        worker.start()

    def _on_camera_info(self, card, message):
        with self._lock:
            if self._closed or not self._session():
                return
            try:
                self._calibrations[card] = calibration(message)
            except (ValueError, TypeError, AttributeError):
                self._calibrations.pop(card, None)

    def _metadata_loop(self):
        # Parameter services may be slow. Never perform them in an image,
        # control callback, or under either the motion or peripheral lock.
        while not self._stop.is_set():
            generation = self.ros1.connection
            if not generation[0]:
                self._stop.wait(.2)
                continue
            try:
                registration = self.ros1.get_param('/camera_head/camera/depth_registration')
            except Exception:
                registration = None
            try:
                polygon = self.ros1.get_param('/lqr_path_track/local_costmap/footprint')
                padding = self.ros1.get_param('/lqr_path_track/local_costmap/footprint_padding', 0.)
                footprint = footprint_from_polygon(polygon, padding)
            except Exception:
                footprint = None
            try:
                model_footprint = footprint_from_model(self.ros1.get_param('/humanoid_description'))
                # Never shrink an existing factory planner envelope merely
                # because the chassis-only collision model is smaller.
                if (footprint is None or all(model_footprint[k] >= footprint[k]
                        for k in ('half_width', 'front', 'rear'))):
                    footprint = model_footprint
            except Exception:
                pass  # Existing costmap estimate stays explicit as the fallback.
            with self._lock:
                if generation == self.ros1.connection and not self._closed:
                    self._depth_registration, self._footprint = registration, footprint
            self._stop.wait(5.)

    def footprint(self):
        with self._lock:
            self._session()
            return copy.deepcopy(self._footprint)

    def _depth_aligned(self):
        return registered_pair(self._calibrations.get('camera_head'),
                               self._calibrations.get('camera_depth'), self._depth_registration)

    def _camera_declaration(self, card):
        entries = declare(card, self.topics[card], DEPTH_FORMAT if card == 'camera_depth' else 'image/jpeg',
                       self._calibrations.get(card), self._dimensions.get(card), self._depth_aligned())
        if card == 'camera_depth':
            entries.extend(declare(card, self._legacy_depth_topic, DEPTH_FORMAT,
                           self._calibrations.get(card), self._dimensions.get(card), self._depth_aligned()))
        return entries

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
            self._calibrations.clear()
            self._dimensions.clear()
            self._depth_registration, self._footprint = None, None
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
                    raise ValueError('oversized/non-base64 image')
                payload = base64.b64decode(data, validate=True)
                if card == 'camera_depth':
                    if message.get('format') != DEPTH_FORMAT:
                        raise ValueError('unexpected depth format')
                    validate_depth(payload)
                    if (not self._depth_aligned() or image_header(message)['frame_id']
                            != 'camera_head_color_optical_frame'):
                        raise ValueError('depth is not verified RGB-registered')
                elif not payload.startswith(b'\xff\xd8') or not payload.endswith(b'\xff\xd9'):
                    raise ValueError('not a complete JPEG')
                self._images[card] = (payload, image_header(message))
                dimensions = (640, 480) if card == 'camera_depth' else jpeg_dimensions(payload)
                if dimensions:
                    self._dimensions[card] = dimensions
                self._data[card] = {'valid': True, 'bytes': len(payload),
                                    'encoding': '16UC1-zlib' if card == 'camera_depth' else 'jpeg',
                                    'source_topic': CAMERAS[card][0]}
                if card == 'camera_depth':
                    self._data[card].update(width=640, height=480, unit='mm', invalid_value=0, aligned_to_rgb=True)
                self._seen[card] = time.monotonic()
                self._image_stats[card]['received'] += 1
                self._wake[card].set()
            except (KeyError, ValueError, TypeError, zlib.error):
                self._data.pop(card, None)
                self._images.pop(card, None)

    def _enable_camera(self, card, generation):
        try:
            with self._lock:
                if self._closed or card not in self._running:
                    return
            service = CAMERAS[card][1]
            if service:
                result = self.ros1.call_service(service, 'std_srvs/SetBool', {'data': True}, timeout=2)
                if result.get('success') is not True:
                    raise RuntimeError('camera service did not accept enable')
            elif self._camera_ensure:
                self._camera_ensure(card)
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
            'camera_head': '640×480 RGB；Orbbec Gemini 335L头部相机，运行时CameraInfo内参。接VOP后可配camera_depth用于navi；导航前头部须朝正前，导航时不要转头。',
            'camera_depth': '640×480 uint16 毫米，0无效；Orbbec头部深度经厂商SDK对齐camera_head RGB，zlib无损压缩。用于navi时，VOP也必须来自camera_head，头部保持朝正前；不可与底盘左右相机混用。'}
        tools = []
        for card in CARDS:
            outputs = [{'topic': self.topics[card], 'format': DEPTH_FORMAT if card == 'camera_depth'
                        else 'image/jpeg' if card in CAMERAS else 'data/json'}]
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
                    self._retry_at[card] = 0
                    self._ensure_camera_if_needed(card, self.snapshot(card))
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
            if card in self._running or card in CAMERAS:
                result['topic_out'] = next(t['topic_out'] for t in self.tools() if t['name'] == card)
            if card in CAMERAS:
                result['camera_info'] = self._camera_declaration(card)
            if card in self._running and card in ('camera_head', 'camera_depth'):
                ready = bool(self._calibrations.get(card)) and result['feedback']['valid']
                if card == 'camera_depth':
                    ready = ready and self._depth_aligned()
                if not ready:
                    result.update(state='loading', message='Waiting for camera calibration and verified RGB-aligned frames.')
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
                    self._ensure_camera_if_needed(card, value)
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

    def _ensure_camera_if_needed(self, card, value):
        can_enable = CAMERAS[card][1] or (self._camera_ensure and card in ('camera_head', 'camera_depth'))
        if (can_enable and not value['valid'] and card not in self._enabling
                and time.monotonic() >= self._retry_at.get(card, 0)):
            self._enabling.add(card)
            generation = self._media[card].connection[1]
            threading.Thread(target=self._enable_camera, args=(card, generation), daemon=True).start()

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
        msg.format = DEPTH_FORMAT if card == 'camera_depth' else 'jpeg'
        msg.data = array('B', jpeg)
        msg.header.frame_id = header['frame_id']
        msg.header.stamp.sec, msg.header.stamp.nanosec = header['sec'], header['nanosec']
        self._image_pubs[card].publish(msg)  # No shared lock held during DDS serialization.
        if card == 'camera_depth':
            self._legacy_depth_pub.publish(msg)
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
            for topic in INFO_TOPICS.values():
                self.ros1.unsubscribe(topic)
            self._running.clear()
        if self._owns_media:
            for client in self._media.values():
                client.close()
        for worker in self._workers:
            worker.join(timeout=2)
