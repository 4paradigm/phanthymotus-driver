"""Panther vendor SLAM + Nav2 controlled mapping, semantic tags and navigation.

Body ROS stays on domain 0. All calls are made outside executor callbacks; the
existing bundle executor spins this independent node. No SLAM stack is launched
or controller configuration changed by the card.
"""
from __future__ import annotations

from copy import copy
import json
import math
import os
import re
import shutil
import sqlite3
import threading
import time
import uuid
from contextlib import nullcontext
from pathlib import Path


def safe_name(value):
    value = str(value).strip()
    if not re.fullmatch(r"[\w\-\u4e00-\u9fff]{1,64}", value):
        raise ValueError("name must contain 1–64 letters, numbers, underscores or hyphens")
    return value


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def filter_body_returns(scan, base_from_laser, box):
    """Invalidate endpoints inside the chassis without changing sensor geometry."""
    import numpy as np

    ranges = np.asarray(scan.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    indices = np.flatnonzero(valid)
    angles = scan.angle_min + indices * scan.angle_increment
    points = ranges[indices] * np.stack((np.cos(angles), np.sin(angles)))
    endpoints = base_from_laser[:2, :2] @ points + base_from_laser[:2, 3, None]
    xmin, xmax, ymin, ymax = box
    x, y = endpoints
    inside = (xmin <= x) & (x <= xmax) & (ymin <= y) & (y <= ymax)
    if not inside.any():
        return scan, 0
    filtered = copy(scan)
    values = ranges.copy()
    values[indices[inside]] = np.nan  # invalid ray, not a clear ray through the body
    filtered.ranges = values.tolist()
    return filtered, int(inside.sum())


class SpatialStore:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        with self.conn:
            self.conn.executescript("""
                CREATE TABLE IF NOT EXISTS maps (
                    name TEXT PRIMARY KEY, pcd_path TEXT NOT NULL,
                    created_at REAL NOT NULL);
                CREATE TABLE IF NOT EXISTS poi (
                    name TEXT NOT NULL, description TEXT DEFAULT '',
                    x REAL NOT NULL, y REAL NOT NULL, yaw REAL NOT NULL,
                    map_name TEXT NOT NULL, UNIQUE(name, map_name));
                CREATE TABLE IF NOT EXISTS map_start_poses (
                    map_name TEXT PRIMARY KEY,
                    x REAL NOT NULL, y REAL NOT NULL, yaw REAL NOT NULL);
            """)

    def maps(self):
        with self.lock:
            return [dict(r) for r in self.conn.execute("SELECT * FROM maps ORDER BY created_at DESC")]

    def tags(self, name):
        with self.lock:
            return [dict(r) for r in self.conn.execute(
                "SELECT name,description,x,y,yaw FROM poi WHERE map_name=? ORDER BY name", (name,))]

    def map_start_pose(self, name):
        with self.lock:
            row = self.conn.execute(
                "SELECT x,y,yaw FROM map_start_poses WHERE map_name=?", (name,)).fetchone()
            return dict(row) if row is not None else None

    def save_map(self, name, path, start_pose=None):
        with self.lock, self.conn:
            self.conn.execute("INSERT OR REPLACE INTO maps (name,pcd_path,created_at) VALUES(?,?,?)", (name, str(path), time.time()))
            if start_pose is not None:
                self.conn.execute("INSERT OR REPLACE INTO map_start_poses (map_name,x,y,yaw) VALUES(?,?,?,?)",
                                  (name, start_pose['x'], start_pose['y'], start_pose['yaw']))

    def remember_map_start(self, name, pose):
        with self.lock, self.conn:
            self.conn.execute("INSERT OR IGNORE INTO map_start_poses (map_name,x,y,yaw) VALUES(?,?,?,?)",
                              (name, pose['x'], pose['y'], pose['yaw']))

    def tag(self, name, map_name, pose, description):
        with self.lock, self.conn:
            self.conn.execute("INSERT OR REPLACE INTO poi (name,description,x,y,yaw,map_name) VALUES(?,?,?,?,?,?)",
                              (name, description, pose['x'], pose['y'], pose['yaw'], map_name))

    def untag(self, name, map_name):
        with self.lock, self.conn:
            return self.conn.execute("DELETE FROM poi WHERE name=? AND map_name=?", (name, map_name)).rowcount

    def delete(self, name):
        with self.lock, self.conn:
            self.conn.execute("DELETE FROM poi WHERE map_name=?", (name,))
            self.conn.execute("DELETE FROM map_start_poses WHERE map_name=?", (name,))
            self.conn.execute("DELETE FROM maps WHERE name=?", (name,))


class ControlledSpatialPlugin:
    ACTION_PARAMS = {
        'start': [], 'info': [], 'stop': [],
        'start_mapping': ['map_name'], 'stop_mapping': [],
        'list_maps': [], 'load_map': ['map_name', 'x', 'y', 'yaw'], 'delete_map': ['map_name'],
        'tag_place': ['name', 'description'], 'untag_place': ['name'], 'list_tags': [],
        'relocalize_at_tag': ['tag_name'],
        'navigate_to_tag': ['tag_name'], 'navigate_to_pose': ['x', 'y', 'yaw'],
        'stop_nav': [], 'get_pose': [],
    }

    def __init__(self, config, namespace, executor):
        self.config = config
        self.namespace = namespace
        self.executor = executor
        self.root = Path(config.get('native_slam_pcd_dir', '/opt/phanthy-motus/data/maps'))
        self.root.mkdir(parents=True, exist_ok=True)
        self.store = SpatialStore(config.get('native_slam_db_path', '/opt/phanthy-motus/data/controlled_spatial.db'))
        self.vendor_dir = Path(config.get('vendor_map_dir', '/home/unix_ai/map_dir'))
        self.lock = threading.RLock()
        self.operations = threading.Lock()
        self.node = None
        self.active_map = None
        self.mapping = False
        self.localized_after = float('inf')
        self.pose = None
        self.pose_received = 0.0
        self.pose_source = None
        self.pose_stamp_after = 0.0
        self.pose_stamp = 0.0
        self.localization_verified = False
        self.mapping_start_pose = None
        self.scan_received = 0.0
        self.scan_frames = 0
        self.scan_frame = None
        self.external_scan_received = 0.0
        self.input_scan_frame = None
        self.relayed_scans = 0
        self.filtered_returns = 0
        self.scan_filter_error = ''
        self.self_filter = config.get('self_filter_box_m')
        if self.self_filter is not None:
            if (len(self.self_filter) != 4 or not all(math.isfinite(v) for v in self.self_filter)
                    or self.self_filter[0] >= self.self_filter[1] or self.self_filter[2] >= self.self_filter[3]):
                raise ValueError('self_filter_box_m must be [xmin, xmax, ymin, ymax] in base_frame')
        self.grid = None
        self.grid_received = 0.0
        self.nav = None
        self.last_navigation = None
        self.last_error = ''
        self.stop_generation = 0
        self.base_device = None

    def get_tool(self):
        props = {'action': {'type': 'string', 'enum': list(self.ACTION_PARAMS)},
                 **{key: {'type': 'string'} for key in ('map_name', 'name', 'tag_name', 'description')},
                 **{key: {'type': 'number'} for key in ('x', 'y', 'yaw')}}
        return {'name': 'controlled_spatial', 'type': 'actuator', 'multiInstance': False,
                'description': 'Panther controlled SLAM: map manually and tag places. To load a map, return to its mapping start position and heading. For legacy maps without a saved start, optionally supply the known map-frame x, y, yaw together on the first load. relocalize_at_tag is optional recovery at a known tag. yaw is radians.',
                'inputSchema': {'type': 'object', 'properties': props, 'required': ['action'],
                    'additionalProperties': False, 'x-resource': 'base',
                    'x-completion': {'actions': ['navigate_to_tag', 'navigate_to_pose'],
                                     'timeout': self.config.get('navigation_timeout_s', 180) + 15},
                    'x-hooks': {'on_interrupt_motion': {'action': 'stop_nav'},
                                'on_interrupt_all': {'action': 'stop_nav'}},
                    'x-action-params': {a: {'params': p} for a, p in self.ACTION_PARAMS.items()}}}

    def start(self):
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from nav2_msgs.action import NavigateToPose
        from tf2_ros import Buffer, TransformListener
        from std_srvs.srv import Trigger, SetBool
        from unix_msgs.srv import SaveMap, LoadMap, DeleteMap, GetMapList
        if self.node is not None:
            return
        self.node = Node('panther_controlled_spatial')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        group = ReentrantCallbackGroup()
        self.services = {}
        self.types = {'save_map': SaveMap, 'load_map': LoadMap, 'delete_map': DeleteMap,
                      'get_map_list': GetMapList, 'reset': Trigger, 'set_localization_mode': SetBool}
        for name, service_type in self.types.items():
            self.services[name] = self.node.create_client(service_type, '/unix_slam/' + name,
                                                          callback_group=group)
        self.node.create_subscription(PoseWithCovarianceStamped, '/pose', self._pose, 10, callback_group=group)
        self.initial_pose_type = PoseWithCovarianceStamped
        self.initial_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        from rclpy.qos import qos_profile_sensor_data
        self.node.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data, callback_group=group)
        self.scan_relay = None
        source = self.config.get('single_lidar_topic', '')
        if source:
            if source == '/scan':
                raise ValueError('single_lidar_topic must differ from /scan')
            # The shipped vendor merger waits forever for a second lidar on
            # single-lidar Panthers. Preserve the real sensor frame/timestamps;
            # SLAM and Nav2 already consume /scan and use TF for the extrinsic.
            self.scan_relay = self.node.create_publisher(LaserScan, '/scan', 10)
            self.node.create_subscription(LaserScan, source, self._single_scan,
                                          qos_profile_sensor_data, callback_group=group)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid, '/map', self._grid, qos, callback_group=group)
        self.client = ActionClient(self.node, NavigateToPose, self.config.get('navigation_action', '/navigate_to_pose'), callback_group=group)
        navigator_state_service = self.config.get('navigator_state_service')
        if navigator_state_service:
            from lifecycle_msgs.srv import GetState

            self.types['navigator_state'] = GetState
            self.services['navigator_state'] = self.node.create_client(
                GetState, navigator_state_service, callback_group=group)
        self.node.create_timer(0.5, self._watchdog, callback_group=group)
        self.node.create_timer(0.1, self._tf_pose, callback_group=group)
        self.executor.add_node(self.node)

    def stop(self):
        if self.node is not None:
            self.stop_nav()
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.node = None
        with self.store.lock:
            self.store.conn.close()

    def _pose(self, msg):
        if msg.header.frame_id.lstrip('/') != 'map':
            return
        p = msg.pose.pose
        values = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_of(p.orientation)}
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if not all(math.isfinite(v) for v in values.values()) or age < -1 or age > 3:
            return
        with self.lock:
            if stamp <= self.pose_stamp_after:
                return
            if self._requires_localization():
                return  # use the same base_link TF for mapping, tags and localization
            self.pose = values
            self.pose_source = "/pose"
            self.pose_received = time.monotonic()
            self.pose_stamp = stamp

    def _tf_pose(self):
        from rclpy.time import Time
        try:
            transform = self.tf_buffer.lookup_transform('map', self.config.get('base_frame', 'base_link'), Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
            t = transform.transform.translation
            pose = {'x': t.x, 'y': t.y, 'yaw': yaw_of(transform.transform.rotation)}
            if age < -1 or age > 3 or not all(math.isfinite(v) for v in pose.values()):
                return
            with self.lock:
                source = 'tf:map->' + self.config.get('base_frame', 'base_link')
                if stamp <= self.pose_stamp_after or (self.pose_source == source and stamp <= self.pose_stamp):
                    return
                self.pose = pose
                self.pose_source = source
                self.pose_received = time.monotonic()
                self.pose_stamp = stamp
                if self.mapping and self.mapping_start_pose is None and self.grid is not None:
                    self.mapping_start_pose = dict(pose)
        except Exception:
            # TF is normally absent before SLAM starts or localization converges.
            return

    def _single_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if age < -1 or age > 3 or not msg.ranges:
            return
        removed = 0
        if self.self_filter is not None:
            try:
                from rclpy.time import Time
                from spatial_inputs import transform_matrix

                transform = self.tf_buffer.lookup_transform(self.config.get('base_frame', 'base_link'),
                                                            msg.header.frame_id, Time.from_msg(msg.header.stamp))
                msg, removed = filter_body_returns(msg, transform_matrix(transform), self.self_filter)
            except Exception as exc:
                with self.lock:
                    self.scan_filter_error = str(exc)[:240]
                return
        with self.lock:
            self.input_scan_frame = msg.header.frame_id
            # Yield if the vendor merger starts producing real merged scans.
            if self.external_scan_received and time.monotonic() - self.external_scan_received < 3:
                return
            self.relayed_scans += 1
            self.filtered_returns += removed
            self.scan_filter_error = ''
        self.scan_relay.publish(msg)

    def _scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if age < -1 or age > 3 or not msg.ranges:
            return
        with self.lock:
            self.scan_received = time.monotonic()
            self.scan_frames += 1
            self.scan_frame = msg.header.frame_id
            if self.input_scan_frame and msg.header.frame_id != self.input_scan_frame:
                self.external_scan_received = time.monotonic()

    def _require_scan(self):
        if not self.scan_received or time.monotonic() - self.scan_received > 3:
            raise RuntimeError("No fresh /scan frames; check the configured single_lidar_topic or vendor /scan_sync/diagnostics")

    def _grid(self, msg):
        if msg.header.frame_id.lstrip('/') != 'map' or not msg.info.width or not msg.info.height:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        with self.lock:
            if stamp <= self.pose_stamp_after:
                return
            self.grid = msg
            self.grid_received = time.monotonic()

    @staticmethod
    def _wait(future, timeout=10):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            # Service may still finish remotely. Do not imply it was rolled back.
            raise TimeoutError('ROS response timed out; remote operation outcome is unknown, check info before retrying')
        return future.result()

    def _call(self, name, **kwargs):
        client = self.services[name]
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError(client.srv_name + ' is unavailable')
        request = self.types[name].Request(**kwargs)
        response = self._wait(client.call_async(request), self.config.get('service_timeout_s', 20))
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(response.message or name + ' failed')
        return response

    def current_pose(self):
        with self.lock:
            if (self.pose is None or time.monotonic() - self.pose_received > 3
                    or self.pose_received <= self.localized_after):
                raise RuntimeError('No fresh map-frame pose after the last map transition; wait for localization')
            return dict(self.pose)

    def info(self):
        try:
            pose = self.current_pose()
        except RuntimeError:
            pose = None
        with self.lock:
            state = 'mapping' if self.mapping else 'idle'
            if self.active_map and not self.mapping:
                ready = pose and (self.localization_verified or not self._requires_localization())
                state = 'localized' if ready else 'localizing'
            return {'state': state,
                    'active_map': self.active_map, 'pose': pose, 'pose_source': self.pose_source,
                    'localization_verified': self.localization_verified,
                    'mapping_start_pose': self.mapping_start_pose,
                    'navigation': {k: v for k, v in self.nav.items() if k not in ('handle', 'future')} if self.nav else None,
                    'last_navigation': self.last_navigation, 'error': self.last_error,
                    'scan': {'frames': self.scan_frames, 'frame_id': self.scan_frame,
                             'single_lidar_topic': self.config.get('single_lidar_topic'),
                             'relayed_frames': self.relayed_scans,
                             'filtered_body_returns': self.filtered_returns,
                             'filter_error': self.scan_filter_error,
                             'age_s': time.monotonic()-self.scan_received if self.scan_received else None},
                    'map_available': self.grid is not None,
                    'map_dir': str(self.root), 'vendor_map_dir': str(self.vendor_dir)}

    def _transition(self):
        with self.lock:
            if self.nav:
                raise RuntimeError('Stop navigation before changing maps')
            self.localized_after = time.monotonic()
            self.pose_stamp_after = self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
            self.pose = None
            self.localization_verified = False
            self.mapping_start_pose = None
            self.grid = None
            self.active_map = None

    def _requires_localization(self):
        return self.config.get('require_origin_localization') or self.config.get('require_tag_localization')

    def _localize(self, target):
        if self.mapping or not self.active_map or self.grid is None:
            raise RuntimeError('Wait for the loaded map before setting its pose')
        self._require_scan()
        if not all(math.isfinite(target[key]) for key in ('x', 'y', 'yaw')):
            raise ValueError('Invalid localization seed pose')
        with self.lock:
            if self.nav:
                raise RuntimeError('Stop navigation before setting the pose')
        if not self.initial_pose_pub.get_subscription_count():
            raise RuntimeError('No SLAM subscriber on /initialpose')
        message = self.initial_pose_type()
        stamp = self.node.get_clock().now()
        message.header.frame_id = 'map'
        message.header.stamp = stamp.to_msg()
        message.pose.pose.position.x = target['x']
        message.pose.pose.position.y = target['y']
        message.pose.pose.orientation.z = math.sin(target['yaw'] / 2)
        message.pose.pose.orientation.w = math.cos(target['yaw'] / 2)
        message.pose.covariance[0] = message.pose.covariance[7] = 0.15 ** 2
        message.pose.covariance[35] = math.radians(15) ** 2
        stamp_s = stamp.nanoseconds / 1e9
        with self.lock:
            self.localization_verified = False
            self.pose = None
            self.pose_stamp_after = stamp_s
            self.localized_after = time.monotonic()
        self.initial_pose_pub.publish(message)

        # Check the live base-frame TF after the seed, allowing for SLAM's
        # scan-matching period. A cached /pose message cannot confirm the seed.
        deadline = time.monotonic() + 5
        stable_since = None
        last_stamp = stamp_s
        while time.monotonic() < deadline:
            with self.lock:
                pose = self.pose
                if (pose and self.pose_source == 'tf:map->' + self.config.get('base_frame', 'base_link')
                        and self.pose_stamp > stamp_s and time.monotonic() - self.pose_received < 0.5
                        and math.hypot(pose['x'] - target['x'], pose['y'] - target['y']) <= 0.25
                        and abs(math.remainder(pose['yaw'] - target['yaw'], 2 * math.pi)) <= math.radians(20)):
                    now = time.monotonic()
                    stable_since = now if stable_since is None else stable_since
                    if self.pose_stamp > last_stamp and now - stable_since >= 1.0:
                        self.localization_verified = True
                        self.last_error = ''
                        return dict(pose)
                    last_stamp = self.pose_stamp
                else:
                    stable_since = None
            time.sleep(0.05)
        raise RuntimeError('SLAM pose did not settle within 0.25 m and 20 degrees of the requested pose')

    @staticmethod
    def _wait_until(ready, timeout=8):
        deadline = time.monotonic() + timeout
        while not ready():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True

    def _archive(self, name):
        """Mirror native artifacts into the same persistent map root as G1/Tianyi."""
        target = self.root / safe_name(name)
        target.mkdir(parents=True, exist_ok=True)
        files = [p for p in self.vendor_dir.glob(name + '*')
                 if p.name == name or p.name.startswith(name + '.')]
        if not files:
            raise RuntimeError('Vendor reported save success but no native map files found in ' + str(self.vendor_dir))
        for path in files:
            if path.is_dir():
                shutil.copytree(path, target / path.name, dirs_exist_ok=True)
            else:
                shutil.copy2(path, target / path.name)
        with self.lock:
            grid = self.grid
        if grid is not None:
            from spatial_map import save_grid
            save_grid(target / 'occupancy.npz', grid)
        self.store.save_map(name, target, self.mapping_start_pose)
        return target

    def dispatch(self, action, args):
        try:
            if action in ('stop', 'stop_nav'):
                return self.stop_nav()
            if action == 'start':
                return {'state': 'ready', 'data': self.info()}
            if action == 'info':
                return self.info()
            if action == 'get_pose':
                with self.lock:
                    return {'pose': self.current_pose(), 'localization_verified': self.localization_verified}
            if not self.operations.acquire(blocking=False):
                return {'state': 'busy', 'error': 'Another mapping operation is in progress'}
            try:
                return self._dispatch(action, args)
            finally:
                self.operations.release()
        except Exception as exc:
            self.last_error = str(exc)
            return {'state': 'error', 'error': str(exc)}

    def _dispatch(self, action, args):
        if action == 'list_maps':
            response = self._call('get_map_list')
            return {'maps': self.store.maps(), 'vendor_maps': list(response.map_names)}
        if action == 'start_mapping':
            if self.mapping:
                raise RuntimeError('Mapping already active; save it before starting another map')
            name = safe_name(args.get('map_name', ''))
            names = self._call('get_map_list').map_names
            if name in names or any(m['name'] == name for m in self.store.maps()):
                raise ValueError('Map already exists; use a new map name')
            self._require_scan()
            self._transition()
            self._call('reset')
            self._call('set_localization_mode', data=False)
            with self.lock:
                self.active_map = name
                self.mapping = True
                self.localized_after = time.monotonic()
            if self._requires_localization() and not self._wait_until(lambda: self.mapping_start_pose is not None):
                return {'state': 'mapping', 'map_name': name,
                        'message': 'Keep the robot stationary until info shows mapping_start_pose; then move manually.'}
            return {'state': 'mapping', 'map_name': name, 'mapping_start_pose': self.mapping_start_pose,
                    'message': 'Move manually using the remote control; tag places then stop_mapping to save.'}
        if action == 'stop_mapping':
            if not self.mapping:
                raise RuntimeError('Mapping is not active')
            name = self.active_map
            self.current_pose()
            if self.grid is None:
                raise RuntimeError('No occupancy grid after the map transition; wait for the next SLAM map update')
            if self._requires_localization() and self.mapping_start_pose is None:
                raise RuntimeError('Wait for the mapping start pose before saving')
            self._call('save_map', map_name=name)
            target = self._archive(name)
            self._call('set_localization_mode', data=True)
            with self.lock:
                self.mapping = False
                self.localization_verified = False
            return {'state': 'saved', 'map_name': name, 'path': str(target)}
        if action == 'load_map':
            if self.mapping:
                raise RuntimeError('Save the active mapping session first')
            name = safe_name(args.get('map_name', ''))
            seed = self.store.map_start_pose(name)
            explicit_seed = None
            pose_keys = ('x', 'y', 'yaw')
            if any(key in args for key in pose_keys):
                if not all(key in args for key in pose_keys):
                    raise ValueError('Supply x, y and yaw together for a known mapping start pose')
                explicit_seed = {key: float(args[key]) for key in pose_keys}
                if not all(math.isfinite(value) for value in explicit_seed.values()):
                    raise ValueError('Pose values must be finite')
            if name not in self._call('get_map_list').map_names:
                # Restore only this card's archived artifacts; never overwrite a native map.
                source = self.root / name
                if not source.is_dir():
                    raise ValueError('Unknown map')
                for path in source.iterdir():
                    if path.name != name and not path.name.startswith(name + '.'):
                        continue
                    dest = self.vendor_dir / path.name
                    if not dest.exists():
                        if path.is_dir():
                            shutil.copytree(path, dest)
                        else:
                            shutil.copy2(path, dest)
            self._transition()
            self._call('load_map', map_name=name)
            self._call('set_localization_mode', data=True)
            with self.lock:
                self.active_map = name
                self.localized_after = time.monotonic()
            result = {'state': 'localizing', 'map_name': name}
            if not self._requires_localization():
                return result
            if explicit_seed is None and seed is None:
                return {**result, 'message': 'Legacy map has no saved start pose. At the original mapping start, call load_map with its known x, y and yaw once, or relocalize at a known tag.'}
            result['seed_source'] = 'provided_start_pose' if explicit_seed is not None else 'mapping_start_pose'
            try:
                if not self._wait_until(lambda: self.grid is not None):
                    raise RuntimeError('No fresh occupancy grid after loading the map')
                pose = self._localize(explicit_seed if explicit_seed is not None else seed)
            except RuntimeError as exc:
                self.last_error = str(exc)
                return {**result, 'error': str(exc),
                        'message': 'Return to the original mapping position and heading and retry load_map.'}
            if explicit_seed is not None and seed is None:
                self.store.remember_map_start(name, explicit_seed)
            return {**result, 'state': 'localized', 'pose': pose}
        if action == 'relocalize_at_tag':
            tag = next((p for p in self.store.tags(self.active_map) if p['name'] == args.get('tag_name')), None)
            if tag is None:
                raise ValueError('Unknown tag on the loaded map')
            return {'state': 'localized', 'tag_name': tag['name'], 'pose': self._localize(tag)}
        if action == 'delete_map':
            name = safe_name(args.get('map_name', ''))
            if name == self.active_map:
                raise ValueError('Cannot delete the active map; load another map first')
            if not any(m['name'] == name for m in self.store.maps()):
                raise ValueError('Only maps managed by this card can be deleted')
            self._call('delete_map', map_name=name)
            self.store.delete(name)
            target = self.root / name
            if target.is_dir():
                shutil.rmtree(target)
            return {'state': 'deleted', 'map_name': name}
        if action in ('list_tags', 'tag_place', 'untag_place'):
            if not self.active_map:
                raise ValueError('Load a map or start mapping first')
            if action == 'tag_place':
                if self.mapping and self._requires_localization() and self.mapping_start_pose is None:
                    raise RuntimeError('Wait for the mapping start pose before saving tags')
                if not self.mapping and self._requires_localization() and not self.localization_verified:
                    raise RuntimeError('Confirm localization with load_map at the start pose or relocalize_at_tag before saving tags')
                self.store.tag(safe_name(args.get('name', '')), self.active_map,
                               self.current_pose(), str(args.get('description', ''))[:1024])
            elif action == 'untag_place':
                if not self.store.untag(safe_name(args.get('name', '')), self.active_map):
                    raise ValueError('Unknown tag')
            return {'map_name': self.active_map, 'tags': self.store.tags(self.active_map)}
        if action in ('navigate_to_tag', 'navigate_to_pose'):
            if self.mapping or not self.active_map:
                raise ValueError('Load a saved map before navigation')
            self.current_pose()
            self._require_scan()
            if self._requires_localization() and not self.localization_verified:
                raise RuntimeError('Confirm localization with load_map at the start pose or relocalize_at_tag before navigation')
            if action == 'navigate_to_tag':
                target = next((p for p in self.store.tags(self.active_map) if p['name'] == args.get('tag_name')), None)
                if target is None:
                    raise ValueError('Unknown tag; tag names must match exactly')
            else:
                target = {key: float(args[key]) for key in ('x', 'y', 'yaw')}
            if not all(math.isfinite(target[key]) for key in ('x', 'y', 'yaw')):
                raise ValueError('Pose values must be finite')
            return self._navigate(target)
        raise ValueError('Unknown action: ' + action)

    def _navigate(self, target):
        device = self.base_device
        with device._base_gate if device is not None else nullcontext():
            if device is not None:
                if device._base_servo is not None:
                    device._base_servo.pause_for_explicit_command()
                device.base_check_available("controlled_spatial")
            return self._navigate_owned(target)

    def _navigate_owned(self, target):
        from nav2_msgs.action import NavigateToPose
        with self.lock:
            if self.nav:
                raise RuntimeError('Navigation is already active')
            generation = self.stop_generation
        if self.config.get('navigator_state_service'):
            from lifecycle_msgs.msg import State

            state = self._call('navigator_state').current_state
            if state.id != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(f'Nav2 bt_navigator is {state.label}; check its lifecycle and behavior tree configuration')
        if not self.client.wait_for_server(timeout_sec=2):
            raise RuntimeError('Nav2 navigate_to_pose action server is unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target['x'])
        goal.pose.pose.position.y = float(target['y'])
        goal.pose.pose.orientation.z = math.sin(target['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(target['yaw'] / 2)
        action_id = str(uuid.uuid4())
        nav = {'action_id': action_id, 'target': target, 'state': 'accepting',
               'started': time.monotonic(), 'handle': None}
        with self.lock:
            if generation != self.stop_generation:
                raise RuntimeError('Navigation canceled before sending the goal')
            self.nav = nav
            try:
                future = self.client.send_goal_async(goal)
                nav['future'] = future
            except Exception:
                self.nav = None
                raise
        def accepted(f):
            try:
                handle = f.result()
                with self.lock:
                    nav['handle'] = handle
                    canceled = self.nav is not nav or nav['state'] == 'canceling'
                    if handle.accepted and not canceled:
                        nav['state'] = 'navigating'
                if not handle.accepted:
                    self._finish(nav, 'error', 'Nav2 rejected the goal')
                else:
                    handle.get_result_async().add_done_callback(lambda r: self._result(nav, r))
                    if canceled:
                        handle.cancel_goal_async()
            except Exception as exc:
                self._finish(nav, 'error', str(exc))
        future.add_done_callback(accepted)
        return {'status': 'accepted', 'action_id': action_id, 'target': target}

    def _result(self, nav, future):
        try:
            result = future.result()
            status = {4: 'completed', 5: 'cancelled', 6: 'error'}.get(result.status, 'error')
            self._finish(nav, status, 'Nav2 terminal status ' + str(result.status))
        except Exception as exc:
            self._finish(nav, 'error', str(exc))

    def _finish(self, nav, status, message):
        with self.lock:
            if self.nav is not nav:
                return
            if nav.get('timeout'):
                status, message = 'error', 'Navigation timed out; cancellation requested'
            self.nav = None
            result = {'status': status, 'message': message, 'target': nav['target']}
            self.last_navigation = {'action_id': nav['action_id'], **result}
        threading.Thread(target=self._notify, args=(nav['action_id'], status, result), daemon=True).start()

    @staticmethod
    def _notify(action_id, status, result):
        import urllib.request
        import ssl
        # Fleet core exposes a local HTTPS endpoint with a self-signed certificate.
        context = ssl._create_unverified_context()
        url = os.environ.get('AGENT_CORE_URL', 'https://localhost:15678') + '/api/acp/complete'
        payload = json.dumps({'action_id': action_id, 'status': status, 'result': result,
                              'tool': 'controlled_spatial', 'ts': time.time()}).encode()
        request = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
        for attempt in range(3):
            try:
                with urllib.request.urlopen(request, timeout=3, context=context):
                    return
            except Exception as exc:
                if attempt == 2:
                    print('[controlled_spatial] completion callback failed:', exc, flush=True)
                else:
                    time.sleep(0.5)

    def stop_nav(self):
        with self.lock:
            self.stop_generation += 1
            nav = self.nav
            if not nav:
                return {'state': 'idle'}
            nav['state'] = 'canceling'
            handle = nav.get('handle')
        if handle is not None and handle.accepted:
            response = self._wait(handle.cancel_goal_async(), 5)
            if not response.goals_canceling:
                with self.lock:
                    if self.nav is nav:
                        return {'state': 'error', 'error': 'Nav2 did not acknowledge cancellation; check navigation status'}
        return {'state': 'canceling', 'action_id': nav['action_id']}

    def _watchdog(self):
        with self.lock:
            nav = self.nav
            if nav is None or nav.get('timeout'):
                return
            if time.monotonic() - nav['started'] <= self.config.get('navigation_timeout_s', 180):
                return
            nav.update(timeout=True, state='canceling')
            handle = nav.get('handle')
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
        # Remain busy until Nav2 reports terminal status; never claim the robot stopped.
        self.last_error = 'Navigation timed out; waiting for Nav2 cancellation result'
