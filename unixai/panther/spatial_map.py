"""Occupancy-grid display using the fleet sensor/mapping binary contract."""
from __future__ import annotations

from array import array
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import struct
import threading
import time


MAX_POINTS = 80000  # Matches the Agent Core mapping renderer.
MAX_TAG_POINTS = 20000


def save_grid(path, msg):
    import numpy as np
    q = msg.info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    temp = Path(str(path) + '.tmp')
    with temp.open('wb') as out:
        np.savez_compressed(out, data=np.asarray(msg.data, dtype=np.int8),
                            width=msg.info.width, height=msg.info.height,
                            resolution=msg.info.resolution,
                            origin=[msg.info.origin.position.x, msg.info.origin.position.y, yaw])
    temp.replace(path)


def grid_points(data, width, height, resolution, origin):
    import numpy as np
    values = np.asarray(data, dtype=np.int8)
    if values.size != width * height or resolution <= 0:
        raise ValueError('Invalid occupancy grid')
    occupied = np.flatnonzero(values >= 50)
    occupied = occupied[::max(1, math.ceil(len(occupied) / 78000))]
    x = (occupied % width + 0.5) * resolution
    y = (occupied // width + 0.5) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return np.column_stack((origin[0] + c*x - s*y, origin[1] + s*x + c*y,
                            np.zeros(len(x)))).astype('<f4')


def tag_points(tags):
    """Render each tag as a raised pillar and a heading arrow in map coordinates."""
    import numpy as np

    if not tags:
        return np.empty((0, 3), dtype='<f4')
    angles = np.linspace(0, math.tau, 24, endpoint=False)
    ring = [(0.14 * math.cos(angle), 0.14 * math.sin(angle)) for angle in angles]
    marker = [(0, 0, 0.82)]
    marker.extend((x, y, z) for z in np.linspace(0.04, 0.82, 20) for x, y in ring)
    marker.extend((x, y, 0.14) for x in np.linspace(0, 0.38, 28) for y in (-0.035, 0, 0.035))
    for x in np.linspace(0.30, 0.55, 18):
        half_width = 0.16 * (0.55 - x) / 0.25
        marker.extend((x, y, 0.14) for y in np.linspace(-half_width, half_width, 9))

    tags = tags[:MAX_TAG_POINTS]
    samples = min(len(marker), MAX_TAG_POINTS // len(tags))
    indices = np.linspace(0, len(marker) - 1, samples, dtype=np.intp)
    template = np.asarray(marker)[indices]
    points = []
    for tag in tags:
        c, s = math.cos(tag['yaw']), math.sin(tag['yaw'])
        rotated = template @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        points.append(rotated + [tag['x'], tag['y'], 0])
    return np.concatenate(points).astype('<f4')


def _publisher(messages, topic):
    from common import logsafe
    logsafe.install(check_fd=False)
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('AGENT_CORE_ROS_DOMAIN_ID', '42')
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/opt/phanthy-motus/dds-local.xml'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import UInt8MultiArray
    rclpy.init()
    node = Node('panther_spatial_map_core', enable_rosout=False, start_parameter_services=False)
    publisher = node.create_publisher(UInt8MultiArray, topic, 1)
    try:
        while True:
            try:
                payload = messages.get(timeout=0.1)
            except queue.Empty:
                rclpy.spin_once(node, timeout_sec=0)
                continue
            if payload is None:
                return
            msg = UInt8MultiArray()
            msg.data = array('B', payload)
            publisher.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


class SpatialMapPlugin:
    def __init__(self, spatial, namespace):
        self.spatial = spatial
        self.topic = f'/{namespace}/controlled_spatial/map'
        self.selected = None
        self.event = threading.Event()
        self.lock = threading.Lock()
        self.process = None
        self.thread = None
        self.error = ''

    def get_tool(self):
        actions = {action: {'params': []} for action in ('start', 'stop', 'info', 'list_maps')}
        actions['select_map'] = {'params': ['map_name'], 'required': ['map_name']}
        return {'name': 'spatial_map', 'type': 'actuator', 'multiInstance': False,
                'description': 'Panther saved occupancy map, live pose and semantic tags.',
                'topic_out': [{'topic': self.topic, 'format': 'sensor/mapping'}],
                'inputSchema': {'type': 'object', 'required': ['action'], 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': list(actions)},
                                   'map_name': {'type': 'string'}},
                    'x-action-params': actions}}

    def start(self):
        # Map stream is lazy, like the audio and camera cards.
        pass

    def stop(self):
        self.event.set()
        if self.thread:
            self.thread.join(timeout=3)
            self.thread = None
        if self.process:
            try:
                self.messages.put_nowait(None)
            except queue.Full:
                pass
            self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
            self.process = None

    def dispatch(self, action, args):
        from controlled_spatial import safe_name
        with self.lock:
            if action == 'stop':
                self.stop()
            elif action == 'select_map':
                name = safe_name(args.get('map_name', ''))
                if not (self.spatial.root / name / 'occupancy.npz').is_file():
                    return {'state': 'error', 'error': 'No saved occupancy grid for this map'}
                self.selected = name
            elif action not in ('start', 'info', 'list_maps'):
                return {'state': 'error', 'error': 'Unknown action'}
            if action in ('start', 'select_map') and self.process is None:
                ctx = mp.get_context('spawn')
                self.messages = ctx.Queue(maxsize=1)
                self.process = ctx.Process(target=_publisher, args=(self.messages, self.topic), daemon=True)
                self.process.start()
                self.event.clear()
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
            return {'state': 'running' if self.process and self.process.is_alive() else 'idle',
                    'selected_map': self.selected, 'maps': self.spatial.store.maps(), 'error': self.error,
                    'topic_out': [{'topic': self.topic, 'format': 'sensor/mapping'}]}

    def _run(self):
        while not self.event.is_set():
            try:
                payload = self._payload()
                try:
                    self.messages.put_nowait(payload)
                except queue.Full:
                    try:
                        self.messages.get_nowait()
                    except queue.Empty:
                        pass
                self.error = ''
            except Exception as exc:
                self.error = str(exc)
            self.event.wait(0.5)

    def _payload(self):
        import numpy as np
        with self.spatial.lock:
            grid = self.spatial.grid
            active = self.spatial.active_map
            mapping = self.spatial.mapping
        name = self.selected or active
        path = self.spatial.root / name / 'occupancy.npz' if name else None
        points = np.zeros((0, 3), dtype='<f4')
        if name == active and mapping and grid is not None:
            q = grid.info.origin.orientation
            origin = [grid.info.origin.position.x, grid.info.origin.position.y,
                      math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            points = grid_points(grid.data, grid.info.width, grid.info.height, grid.info.resolution, origin)
        elif path and path.is_file():
            with np.load(path, allow_pickle=False) as saved:
                points = grid_points(saved['data'], int(saved['width']), int(saved['height']),
                                     float(saved['resolution']), saved['origin'])
        try:
            pose = self.spatial.current_pose() if name == active else None
        except RuntimeError:
            pose = None
        tags = self.spatial.store.tags(name) if name else []
        markers = tag_points(tags)
        grid_budget = MAX_POINTS - len(markers)
        if len(points) > grid_budget:
            points = points[np.linspace(0, len(points) - 1, grid_budget, dtype=np.intp)]
        meta = {'version': 3, 'active_map': name, 'maps': self.spatial.store.maps(), 'tags': tags,
                'robot': {**(pose or {'x': 0, 'y': 0, 'yaw': 0}), 'pose_available': pose is not None},
                'grid_points': len(points), 'laser_points': 0, 'trajectory_points': 0,
                'artifacts': {'walls': [], 'tracks': [], 'areas': {}, 'pois': []}}
        robot = meta['robot']
        metadata = json.dumps(meta, ensure_ascii=False, allow_nan=False).encode()
        return (struct.pack('<fffBI', robot['x'], robot['y'], -robot['yaw'], 7, len(points) + len(markers))
                + points.tobytes() + markers.tobytes() + struct.pack('<I', len(metadata)) + metadata)
