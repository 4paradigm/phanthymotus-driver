"""Shared scene-graph core for the Bumi `scene_graph` card.

ROS-free helpers used by `scene_graph_card.py`: VOP validation, pixel/depth
projection, scene-graph construction, scene JSON validation and evidence drawing.
No MCP card is registered from this file.

The VOP contract is the one the rest of this repository consumes:
`std_msgs/String` JSON with `timestamp`, `count`, `latency_ms` and
`objects: [{name, position: [x, y], confidence}]`, where `position` is
normalised to [-1, 1] with +x right and +y down.  The Bumi depth image is
registered to the colour grid, so a detection centre can be sampled for a
metric distance; no bounding-box field is invented.
"""

from __future__ import annotations

import json
import math
import re
import time
import uuid
from pathlib import Path

MAX_OBJECTS = 1000

MAX_OBJECT_NAME_BYTES = 256

MAX_PAYLOAD_BYTES = 262144

MAX_LATENCY_MS = 5000

MAX_RESULT_AGE_S = 2.0

MAX_TIMESTAMP_AGE_S = 5.0

DEFAULT_DEPTH_SCALE = 0.001

NEAR_M = 0.45

MIN_RELATION_M = 0.1

MAX_EDGE_M = 2.5

MAX_EVIDENCE_FILES = 200

def _norm(text) -> str:
    """Casefold, normalise separators, drop punctuation, collapse spaces."""
    if not isinstance(text, str):
        return ''
    text = text.strip().casefold()
    text = re.sub('[\\s_\\-]+', ' ', text)
    text = re.sub('[^\\w\\u4e00-\\u9fff ]+', ' ', text)
    return ' '.join(text.split())

def _finite(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)

def decode_vop(payload, now: float) -> dict:
    """Parse one VOP ``String`` payload; raises ``ValueError`` when invalid."""
    if isinstance(payload, bytes):
        if len(payload) > MAX_PAYLOAD_BYTES:
            raise ValueError('VOP payload exceeds 262144 bytes')
        try:
            payload = payload.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ValueError('VOP payload is not UTF-8') from exc
    if not isinstance(payload, str) or len(payload.encode('utf-8')) > MAX_PAYLOAD_BYTES:
        raise ValueError('VOP payload is not a bounded UTF-8 string')
    try:
        value = json.loads(payload)
    except Exception as exc:
        raise ValueError('VOP payload is not JSON') from exc
    if not isinstance(value, dict):
        raise ValueError('VOP payload is not an object')
    timestamp, latency = (value.get('timestamp'), value.get('latency_ms'))
    objects = value.get('objects')
    if not _finite(timestamp) or not -0.1 <= now - timestamp <= MAX_TIMESTAMP_AGE_S:
        raise ValueError('VOP result timestamp is stale or invalid')
    if not _finite(latency) or not 0 <= latency <= MAX_LATENCY_MS:
        raise ValueError('VOP result lacks a valid inference duration')
    if not isinstance(objects, list) or len(objects) > MAX_OBJECTS or value.get('count') != len(objects):
        raise ValueError('VOP object list is invalid')
    kept = []
    for item in objects:
        if not isinstance(item, dict):
            raise ValueError('VOP detection is not an object')
        name, position, confidence = (item.get('name'), item.get('position'), item.get('confidence'))
        if not isinstance(name, str) or not name.strip() or len(name.encode('utf-8')) > MAX_OBJECT_NAME_BYTES:
            raise ValueError('VOP object name is invalid')
        if not isinstance(position, list) or len(position) != 2 or any((not _finite(v) or not -1 <= v <= 1 for v in position)):
            raise ValueError('VOP object position is invalid')
        if not _finite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('VOP object confidence is invalid')
        kept.append({'name': name.strip(), 'position': [float(position[0]), float(position[1])], 'confidence': float(confidence)})
    return {'timestamp': float(timestamp), 'latency_ms': float(latency), 'count': len(kept), 'objects': kept, 'received_at': now}

def pixel_from_position(position, width: int, height: int):
    """Normalised VOP centre -> integer pixel in the RGB frame."""
    x, y = (float(position[0]), float(position[1]))
    u = int(round((x + 1.0) * 0.5 * (width - 1)))
    v = int(round((y + 1.0) * 0.5 * (height - 1)))
    return (min(max(u, 0), width - 1), min(max(v, 0), height - 1))

def intrinsics_for(width: int, height: int):
    """Bumi's colour intrinsics from the card's declared field of view."""
    try:
        from camera_specs import SPECS
        spec = SPECS['camera']
        fx = width / 2.0 / math.tan(spec['half_fov_rad'])
        fy = height / 2.0 / math.tan(spec['half_fov_v_rad'])
        return {'fx': fx, 'fy': fy, 'ppx': width / 2.0, 'ppy': height / 2.0}
    except Exception:
        return None

def decode_depth(blob, width: int, height: int):
    """Decode the camera's zlib-compressed Z16 depth into a uint16 image."""
    try:
        import zlib
        import numpy as np
        raw = zlib.decompress(bytes(blob))
        depth = np.frombuffer(raw, dtype=np.uint16)
        if depth.size != width * height:
            return None
        return depth.reshape(height, width)
    except Exception:
        return None

def depth_at(depth, u: int, v: int, patch: int=2):
    """Median valid depth in raw sensor units around a pixel, or None."""
    if depth is None:
        return None
    height, width = depth.shape[:2]
    y0, y1 = (max(0, v - patch), min(height, v + patch + 1))
    x0, x1 = (max(0, u - patch), min(width, u + patch + 1))
    window = depth[y0:y1, x0:x1]
    try:
        import numpy as np
        valid = window[window > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid))
    except Exception:
        return None

def robot_from_pixel(u: int, v: int, distance_m: float, intrinsics: dict):
    """Colour optical (x right, y down, z forward) -> robot (x fwd, y left, z up)."""
    if not intrinsics or distance_m is None:
        return None
    fx, fy = (intrinsics['fx'], intrinsics['fy'])
    x_cam = (u - intrinsics['ppx']) / fx * distance_m
    y_cam = (v - intrinsics['ppy']) / fy * distance_m
    return {'forward': round(distance_m, 3), 'left': round(-x_cam, 3), 'up': round(-y_cam, 3)}

def _relation(a: dict, b: dict) -> str:
    """Spatial relation of object a with respect to object b."""
    robot_a, robot_b = (a.get('robot'), b.get('robot'))
    if robot_a and robot_b:
        dx = robot_a['left'] - robot_b['left']
        dz = robot_a['forward'] - robot_b['forward']
        dy = robot_a['up'] - robot_b['up']
        if math.hypot(dx, dz) < NEAR_M:
            return 'near'
        if abs(dx) >= abs(dz) and abs(dx) >= MIN_RELATION_M:
            return 'left_of' if dx > 0 else 'right_of'
        if abs(dz) >= MIN_RELATION_M:
            return 'in_front_of' if dz > 0 else 'behind'
        if abs(dy) >= MIN_RELATION_M:
            return 'above' if dy > 0 else 'below'
        return 'near'
    dx = a['position'][0] - b['position'][0]
    dy = a['position'][1] - b['position'][1]
    if math.hypot(dx, dy) < 0.25:
        return 'near'
    if abs(dx) >= abs(dy) and abs(dx) >= 0.12:
        return 'left_of' if dx < 0 else 'right_of'
    if abs(dy) >= 0.12:
        return 'above' if dy < 0 else 'below'
    return 'near'

def _edge_relevant(a: dict, b: dict) -> bool:
    robot_a, robot_b = (a.get('robot'), b.get('robot'))
    if robot_a and robot_b:
        return math.dist((robot_a['forward'], robot_a['left']), (robot_b['forward'], robot_b['left'])) <= MAX_EDGE_M
    return math.dist(a['position'], b['position']) <= 1.5

def build_scene(detections, width: int, height: int, depth, depth_scale: float, intrinsics, captured_at: float, scene_id: str=None) -> dict:
    """Detections -> scene graph with robot-frame points and spatial edges."""
    scene_id = scene_id or uuid.uuid4().hex[:12]
    nodes = []
    for index, detection in enumerate(detections, start=1):
        u, v = pixel_from_position(detection['position'], width, height)
        raw_depth = depth_at(depth, u, v)
        distance = None if raw_depth is None else round(raw_depth * float(depth_scale), 3)
        robot = robot_from_pixel(u, v, distance, intrinsics)
        nodes.append({'id': f'obj{index}', 'label': detection['name'], 'label_key': _norm(detection['name']), 'confidence': round(float(detection['confidence']), 3), 'position': [round(float(detection['position'][0]), 4), round(float(detection['position'][1]), 4)], 'pixel': [u, v], 'distance_m': distance, 'robot': robot})
    edges = []
    for i, first in enumerate(nodes):
        for second in nodes[i + 1:]:
            if _edge_relevant(first, second):
                edges.append({'source': first['id'], 'relation': _relation(first, second), 'target': second['id']})
    return {'id': scene_id, 'captured_at': captured_at, 'captured_at_text': format_time(captured_at), 'width': width, 'height': height, 'depth_available': depth is not None, 'nodes': nodes, 'edges': edges}

def format_time(timestamp: float) -> str:
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))

def annotation_targets(scene: dict, target_key: str=None) -> list:
    """(node, is_target) pairs; drawing itself stays in the plugin."""
    key = _norm(target_key) if target_key else None
    return [(node, bool(key and node.get('label_key') == key)) for node in scene.get('nodes') or []]

def draw_evidence(jpeg, scene, target_key=None):
    """Annotate every node and highlight the target; returns JPEG bytes."""
    try:
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return jpeg
        for node, is_target in annotation_targets(scene, target_key):
            u, v = node['pixel']
            color = (0, 0, 255) if is_target else (0, 200, 0)
            cv2.circle(image, (int(u), int(v)), 14, color, 3 if is_target else 2)
            if is_target:
                cv2.circle(image, (int(u), int(v)), 24, color, 2)
                cv2.line(image, (int(u) - 30, int(v)), (int(u) + 30, int(v)), color, 2)
                cv2.line(image, (int(u), int(v) - 30), (int(u), int(v) + 30), color, 2)
            label = node['label']
            tag = label if label.isascii() else f"#{node['id'][3:]}"
            if node.get('distance_m') is not None:
                tag += f" {node['distance_m']:.2f}m"
            cv2.putText(image, tag, (int(u) + 18, max(18, int(v) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        header = f"scene_graph  {scene.get('captured_at_text', '')}  objects={len(scene['nodes'])}"
        cv2.putText(image, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        return encoded.tobytes() if ok else jpeg
    except Exception:
        return jpeg

def save_evidence(directory, prefix, jpeg):
    """Write a bounded evidence JPEG and return its absolute path."""
    try:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        path = directory / f'{prefix}_{stamp}_{time.time_ns() % 1000000:06d}.jpg'
        path.write_bytes(jpeg)
        files = sorted(directory.glob('*.jpg'), key=lambda item: item.stat().st_mtime)
        for stale in files[:-MAX_EVIDENCE_FILES]:
            stale.unlink(missing_ok=True)
        return str(path)
    except OSError:
        return ''

def decode_scene(payload, now=None):
    """Parse a scene-graph JSON message; raises ValueError when malformed."""
    if isinstance(payload, bytes):
        try:
            payload = payload.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ValueError('scene payload is not UTF-8') from exc
    if not isinstance(payload, str):
        raise ValueError('scene payload is not a string')
    try:
        scene = json.loads(payload)
    except Exception as exc:
        raise ValueError('scene payload is not JSON') from exc
    if not isinstance(scene, dict) or not isinstance(scene.get('id'), str):
        raise ValueError('scene payload lacks an id')
    if not _finite(scene.get('captured_at')):
        raise ValueError('scene payload lacks a timestamp')
    nodes = scene.get('nodes')
    if not isinstance(nodes, list) or len(nodes) > MAX_OBJECTS:
        raise ValueError('scene payload has no valid nodes')
    for node in nodes:
        if not isinstance(node, dict) or not isinstance(node.get('label'), str) or (not isinstance(node.get('pixel'), list)) or (len(node['pixel']) != 2):
            raise ValueError('scene payload contains an invalid node')
    if not isinstance(scene.get('edges'), list):
        raise ValueError('scene payload has no edge list')
    return scene
