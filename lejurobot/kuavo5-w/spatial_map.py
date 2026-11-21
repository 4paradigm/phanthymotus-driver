"""Read-only display of verified JaTen map snapshots and their POIs."""
from array import array
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import threading

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


def encode_frame(name, grid, boundary, tags, maps):
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
    metadata = json.dumps({
        'version': 3, 'active_map': name, 'maps': [{'name': item} for item in maps],
        'tags': tags, 'boundary': boundary, 'grid_points': len(grid) // 12,
        'robot': {'x': center_x, 'y': center_y, 'yaw': 0.0, 'pose_available': False},
    }, ensure_ascii=False, allow_nan=False).encode()
    return (struct.pack('<fffBI', center_x, center_y, 0.0, 7, len(points) // 12)
            + points + struct.pack('<I', len(metadata)) + metadata)


class SpatialMap:
    def __init__(self, node, namespace, archive, store):
        self.node, self.archive, self.store = node, archive, store
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
            'name': 'spatial_map', 'type': 'sensor', 'multiInstance': False,
            'description': '预览上位机已归档的底盘地图和点位；可选择历史地图。',
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['list_maps', 'select_map']},
                'map_name': {'type': 'string'},
            }, 'x-action-params': {
                'list_maps': {'params': [], 'description': 'List archived maps.'},
                'select_map': {'params': ['map_name'], 'description': 'Select an archived map for display.'},
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
            maps = sorted(self.snapshots())
            if action == 'list_maps':
                return {'state': 'ready', 'maps': maps}
            try:
                name = safe_name(args.get('map_name'))
            except ValueError as exc:
                return {'state': 'error', 'error': str(exc)}
            if name not in maps:
                return {'state': 'error', 'error': 'archived_map_not_found'}
            with self.lock:
                self.selected = name
                self.current = self.error = None
                self.tags = []
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
                self.running = True
                self.wake.set()
            elif action == 'stop':
                self.running = False
                return {'state': 'idle'}
            elif action != 'info':
                return {'state': 'error', 'error': 'unsupported_action'}
            return {'state': 'running' if self.running else 'idle', 'topic_out': self.output,
                    'selected_map': self.selected, 'displayed_map': self.current,
                    'tags': self.tags, 'error': self.error}

    def run(self):
        while not self.closed.is_set():
            self.wake.wait(1)
            self.wake.clear()
            with self.lock:
                if self.closed.is_set() or not self.running:
                    continue
                selected = self.selected
            name, frame, error, tags = None, None, None, []
            try:
                snapshots = self.snapshots()
                name = selected or max(snapshots, key=lambda key: snapshots[key][2], default=None)
                if name is not None:
                    version, directory, _ = snapshots[name]
                    key = name, version
                    if self.cached is None or self.cached[0] != key:
                        self.cached = key, read_snapshot(name, directory, version)
                    _, (identity, grid, boundary) = self.cached
                    tags = self.store.tags(identity)[:MAX_TAGS]
                    frame = encode_frame(name, grid, boundary, tags, sorted(snapshots))
            except (OSError, ValueError, RuntimeError, KeyError, TypeError, OverflowError, yaml.YAMLError) as exc:
                error = f'map_preview_unavailable: {type(exc).__name__}'
            with self.lock:
                if self.running and not self.closed.is_set() and selected == self.selected:
                    self.current = name if frame is not None else None
                    self.tags = tags if frame is not None else []
                    self.error = error
                    msg = self.message_type()
                    msg.data = array('B', frame if frame is not None
                                     else encode_frame(None, b'', [0, 0, 0, 0], [], []))
                    self.publisher.publish(msg)

    def close(self):
        self.closed.set()
        self.wake.set()
        if self.thread is not None:
            self.thread.join()
            self.node.destroy_publisher(self.publisher)
