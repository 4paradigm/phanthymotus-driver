"""Read-only display of verified JaTen map snapshots and their POIs."""
from array import array
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import threading
import time

import yaml

from map_storage import MapArchive, safe_name


MAX_GRID_BYTES = 32_000_000
MAX_POINTS = 80_000
MAX_TAGS = 256
PGM_TOKEN = re.compile(rb'[ \t\r\n\f\v]+|#[^\r\n]*|[^ \t\r\n\f\v#]+')


def read_grid(image, config):
    """Decode the factory's 8-bit P5 with ROS map_server coordinates."""
    tokens = []
    offset = 0
    while len(tokens) < 4 and offset < min(len(image), 4096):
        token = PGM_TOKEN.match(image, offset)
        offset = token.end()
        value = token.group()
        if not value.isspace() and not value.startswith(b'#'):
            tokens.append(value)
    if (len(tokens) != 4 or tokens[0] != b'P5' or tokens[3] != b'255'
            or not image[offset:offset + 1].isspace()):
        raise ValueError('unsupported_pgm_header')
    width, height = map(int, tokens[1:3])
    offset += 2 if image[offset:offset + 2] == b'\r\n' else 1
    pixels = memoryview(image)[offset:]
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError('invalid_pgm_dimensions')

    metadata = yaml.safe_load(config)
    resolution = float(metadata['resolution'])
    x, y, yaw = map(float, metadata['origin'])
    occupied = float(metadata['occupied_thresh'])
    negate = int(metadata.get('negate', 0))
    if (not all(math.isfinite(v) and abs(v) <= 1e6 for v in (x, y, yaw))
            or not 0 < resolution <= 10 or not 0 < occupied < 1 or negate not in (0, 1)):
        raise ValueError('invalid_grid_metadata')

    cosine, sine = math.cos(yaw), math.sin(yaw)

    def world(dx, dy):
        return x + cosine * dx - sine * dy, y + sine * dx + cosine * dy

    corners = [world(u * width * resolution, v * height * resolution)
               for u, v in ((0, 0), (1, 0), (0, 1), (1, 1))]
    min_x, min_y = map(min, zip(*corners))
    max_x, max_y = map(max, zip(*corners))
    boundary = [min_x, min_y, max_x - min_x, max_y - min_y]
    # Walk occupied pixels in order and sample across the entire map. Sample
    # occupied cells, rather than every nth pixel, to preserve thin walls.
    palette = bytes(1 if (p if negate else 255 - p) / 255 > occupied else 0
                    for p in range(256))
    mask = image[offset:].translate(palette)
    stride = max(1, math.ceil(mask.count(1) / (MAX_POINTS - MAX_TAGS * 68)))
    grid = bytearray()
    for index, match in enumerate(re.finditer(b'\x01', mask)):
        if index % stride == 0:
            row, col = divmod(match.start(), width)
            px, py = world((col + .5) * resolution, (height - row - .5) * resolution)
            grid.extend(struct.pack('<fff', px, py, 0.0))
    return grid, boundary


def read_snapshot(name, directory, version):
    manifest = json.loads((directory / 'manifest.json').read_text())
    if MapArchive.validate(manifest, name) != version:
        raise ValueError('map_archive_manifest_mismatch')
    contents = {}
    for suffix in ('.pgm', '.yaml', '.path'):
        expected = manifest['files'][suffix]
        limit = 65536 if suffix == '.yaml' else MAX_GRID_BYTES
        if expected['size'] > limit:
            raise ValueError('map_archive_file_too_large')
        with (directory / (name + suffix)).open('rb') as stream:
            data = stream.read(expected['size'] + 1)
        if len(data) != expected['size'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
            raise ValueError('map_archive_digest_mismatch')
        contents[suffix] = data
    grid_sha = hashlib.sha256()
    for suffix in ('.pgm', '.yaml'):
        grid_sha.update(suffix.encode())
        grid_sha.update(contents[suffix])
    road_md5 = hashlib.md5(contents['.path'], usedforsecurity=False).hexdigest()
    grid, boundary = read_grid(contents['.pgm'], contents['.yaml'])
    return (name, grid_sha.hexdigest() + ':' + road_md5), grid, boundary


def encode_frame(name, grid, boundary, tags, maps, pose=None):
    points = bytearray(grid)
    for tag in tags:
        x, y, yaw = (tag[key] for key in ('x', 'y', 'yaw'))
        for level in range(8):
            for i in range(8):
                angle = i * math.tau / 8
                points.extend(struct.pack('<fff', x + .09 * math.cos(angle),
                                          y + .09 * math.sin(angle), .05 + level * .07))
        for distance in (.12, .18, .24, .30):
            points.extend(struct.pack('<fff', x + distance * math.cos(yaw),
                                      y + distance * math.sin(yaw), .15))
    x, y, width, height = boundary
    center_x, center_y = x + width / 2, y + height / 2
    robot = dict(pose, pose_available=True) if pose is not None else {'pose_available': False}
    # The legacy renderer ignores metadata and always draws the cone. A NaN
    # yaw hides its geometry while its finite XY keeps the view centered.
    header = (pose['x'], pose['y'], -pose['yaw']) if pose is not None else (center_x, center_y, math.nan)
    metadata = json.dumps({
        'version': 3, 'active_map': name, 'maps': [{'name': item} for item in maps],
        'tags': tags, 'boundary': boundary, 'grid_points': len(grid) // 12,
        'robot': robot,
    }, ensure_ascii=False, allow_nan=False).encode()
    return (struct.pack('<fffBI', *header, 7, len(points) // 12)
            + points + struct.pack('<I', len(metadata)) + metadata)


class SpatialMap:
    def __init__(self, node, namespace, archive, store, pose_updates):
        self.node, self.archive, self.store = node, archive, store
        self.pose_updates = pose_updates
        self.latest_pose = None
        self.pose_thread = None
        self.started = threading.Event()
        self.pose = None
        self.topic = '/' + '/'.join(filter(None, (namespace.strip('/'), 'state', 'spatial_map')))
        self.output = [{'topic': self.topic, 'format': 'sensor/mapping'}]
        self.lock = threading.RLock()
        self.running = False
        self.selected = None
        self.current = None
        self.tags = []
        self.error = None
        self.cached = None
        self.thread = None
        self.publisher = None
        self.closed = threading.Event()
        self.wake = threading.Event()

    def tool(self):
        return {
            'name': 'spatial_map', 'type': 'actuator', 'multiInstance': False,
            'description': '预览底盘地图和点位；自动同步预览所需文件，不载图或改变定位。',
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['list_maps', 'select_map']},
                'map_name': {'type': 'string'},
            }, 'required': ['action'], 'x-resource': 'spatial_map',
                'x-completion': {'actions': []}, 'x-action-params': {
                'list_maps': {'params': [], 'required': [], 'description': 'List chassis and locally available maps.'},
                'select_map': {'params': ['map_name'], 'required': ['map_name'], 'description': 'Sync and preview a map without loading it on the chassis.'},
            }},
            'topic_out': self.output,
        }

    def snapshots(self):
        root = self.archive.root if self.archive else None
        result = {}
        if root is None or not root.is_dir():
            return result
        for item in root.iterdir():
            try:
                name = safe_name(item.name)
                pointer = json.loads((item / 'current.json').read_text())
                version = pointer['version']
                if pointer['map_name'] != name or not re.fullmatch(r'[a-f0-9]{64}', version):
                    continue
                path = item / 'versions' / version
                result[name] = (version, path, path.stat().st_mtime_ns)
            except (OSError, ValueError, TypeError, KeyError):
                continue
        return result

    def dispatch(self, action, args):
        if action in ('list_maps', 'select_map'):
            local = self.snapshots()
            try:
                remote = {safe_name(Path(item['fileName']).stem) for item in self.archive.client.maps()} if self.archive else set()
            except Exception as exc:
                return {'state': 'error', 'error': 'map_list_unavailable', 'error_type': type(exc).__name__}
            maps = sorted(set(local) | remote)
            if action == 'list_maps':
                return {'state': 'ready', 'maps': maps}
            try:
                name = safe_name(args.get('map_name'))
            except ValueError as exc:
                return {'state': 'error', 'error': str(exc)}
            if name not in maps:
                return {'state': 'error', 'error': 'map_not_found'}
            if name in remote:
                try:
                    self.archive.sync(name)
                except Exception as exc:
                    return {'state': 'error', 'error': 'map_preview_sync_failed', 'error_type': type(exc).__name__}
            with self.lock:
                self.selected = name
                self.current = self.error = None
                self.tags = []
                self.pose = None
                self.wake.set()
            return {'state': 'completed', 'selected_map': name}
        with self.lock:
            if action == 'start':
                if self.archive is None:
                    return {'state': 'error', 'error': 'map_archive_not_configured'}
                if self.publisher is None:
                    from std_msgs.msg import UInt8MultiArray
                    self.message_type = UInt8MultiArray
                    self.publisher = self.node.create_publisher(UInt8MultiArray, self.topic, 1)
                    self.thread = threading.Thread(target=self.run, name='spatial-map', daemon=True)
                    self.thread.start()
                    self.pose_thread = threading.Thread(target=self.track_pose, name='spatial-map-pose', daemon=True)
                    self.pose_thread.start()
                self.running = True
                self.started.set()
                self.wake.set()
            elif action == 'stop':
                self.running = False
                self.started.clear()
                self.latest_pose = None
                return {'state': 'idle'}
            elif action != 'info':
                return {'state': 'error', 'error': 'unsupported_action'}
            return {'state': 'ready' if self.running else 'idle', 'topic_out': self.output,
                    'selected_map': self.selected, 'displayed_map': self.current,
                    'tags': self.tags, 'pose': self.pose if self.running else None, 'error': self.error}

    def track_pose(self):
        def active():
            return self.started.is_set() and not self.closed.is_set()

        while not self.closed.is_set():
            if not self.started.wait(.5) or not active():
                continue
            try:
                for sample in self.pose_updates(active):
                    with self.lock:
                        if active():
                            self.latest_pose = sample
            except (OSError, ValueError, RuntimeError):
                pass  # The map stays visible when the pose source is unavailable.
            finally:
                with self.lock:
                    self.latest_pose = None
            self.closed.wait(.2)

    def run(self):
        while not self.closed.is_set():
            self.wake.wait(.2)
            self.wake.clear()
            with self.lock:
                if self.closed.is_set() or not self.running:
                    continue
                selected = self.selected
            name, frame, error, tags, pose = None, None, None, [], None
            try:
                snapshots = self.snapshots()
                with self.lock:
                    sample = self.latest_pose
                active = sample[0][0] if sample else None
                name = selected or (active if active in snapshots else
                                    max(snapshots, key=lambda key: snapshots[key][2], default=None))
                if name is not None:
                    version, directory, _ = snapshots[name]
                    key = name, version
                    if self.cached is None or self.cached[0] != key:
                        self.cached = key, read_snapshot(name, directory, version)
                    _, (identity, grid, boundary) = self.cached
                    tags = self.store.tags(identity)[:MAX_TAGS]
                    if sample and sample[0] == identity and time.monotonic() - sample[2] <= 1.:
                        pose = sample[1]
                    frame = encode_frame(name, grid, boundary, tags, sorted(snapshots), pose)
            except (OSError, ValueError, RuntimeError, KeyError, TypeError, OverflowError, yaml.YAMLError) as exc:
                error = f'map_preview_unavailable: {type(exc).__name__}'
            with self.lock:
                if self.running and not self.closed.is_set() and selected == self.selected:
                    self.current = name if frame is not None else None
                    self.tags = tags if frame is not None else []
                    self.pose = pose if frame is not None else None
                    self.error = error
                    msg = self.message_type()
                    msg.data = array('B', frame if frame is not None
                                     else encode_frame(None, b'', [0, 0, 0, 0], [], []))
                    self.publisher.publish(msg)

    def close(self):
        self.closed.set()
        self.wake.set()
        self.started.set()
        if self.pose_thread is not None:
            self.pose_thread.join()
        if self.thread is not None:
            self.thread.join()
            self.node.destroy_publisher(self.publisher)
