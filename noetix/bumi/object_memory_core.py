"""Shared memory core for the Bumi `object_memory` card.

ROS-free helpers used by `object_memory_card.py`: scene JSON validation, the
on-disk memory format, scene merging, open-vocabulary label matching for
`forget`/`get`, and evidence-file storage.  No MCP card is registered from this
file.

The memory file is the same format the full pipeline's `object_locator` reads:
`objects` keyed by normalised label plus a bounded `scenes` map referencing the
scene screenshots.
"""

from __future__ import annotations

import difflib
import json
import math
import re
import time
from pathlib import Path

MAX_OBJECTS = 1000

MAX_SCENES = 40

MAX_EVIDENCE_FILES = 200

MIN_MATCH = 0.62

AMBIGUITY_MARGIN = 0.08

_VERSION = 1

_ALIASES = {'keys': ('钥匙', 'key', 'keys', '门钥匙'), 'wallet': ('钱包', 'wallet', 'purse'), 'phone': ('手机', 'phone', 'cellphone', 'mobile', 'smartphone'), 'glasses': ('眼镜', 'glasses', 'spectacles'), 'cup': ('杯子', '水杯', '马克杯', 'cup', 'mug'), 'bottle': ('瓶子', '水壶', '保温杯', 'bottle', 'flask'), 'laptop': ('电脑', '笔记本', 'laptop', 'computer', 'notebook'), 'remote': ('遥控器', 'remote', 'remote control'), 'headphones': ('耳机', 'headphones', 'earbuds', 'earphone'), 'charger': ('充电器', 'charger', 'adapter'), 'umbrella': ('雨伞', 'umbrella'), 'mask': ('口罩', 'mask'), 'book': ('书', '书本', 'book'), 'pen': ('笔', '钢笔', 'pen'), 'watch': ('手表', 'watch'), 'bag': ('包', '背包', '书包', 'bag', 'backpack'), 'tissue': ('纸巾', '抽纸', 'tissue', 'paper'), 'medicine': ('药', '药品', 'medicine', 'pill'), 'table': ('桌子', '桌', 'table', 'desk'), 'chair': ('椅子', 'chair'), 'sofa': ('沙发', 'sofa', 'couch'), 'bed': ('床', 'bed'), 'door': ('门', 'door'), 'window': ('窗户', '窗', 'window'), 'fridge': ('冰箱', 'fridge', 'refrigerator'), 'microwave': ('微波炉', 'microwave'), 'tv': ('电视', 'tv', 'television'), 'monitor': ('显示器', '屏幕', 'monitor', 'screen'), 'keyboard': ('键盘', 'keyboard'), 'mouse': ('鼠标', 'mouse'), 'snack': ('零食', 'snack'), 'apple': ('苹果', 'apple'), 'banana': ('香蕉', 'banana')}

_INVERSE = {'left_of': 'right_of', 'right_of': 'left_of', 'in_front_of': 'behind', 'behind': 'in_front_of', 'above': 'below', 'below': 'above', 'near': 'near'}

def _norm(text) -> str:
    """Casefold, normalise separators, drop punctuation, collapse spaces."""
    if not isinstance(text, str):
        return ''
    text = text.strip().casefold()
    text = re.sub('[\\s_\\-]+', ' ', text)
    text = re.sub('[^\\w\\u4e00-\\u9fff ]+', ' ', text)
    return ' '.join(text.split())

def _bigrams(text: str) -> set:
    text = text.replace(' ', '')
    if len(text) < 2:
        return {text} if text else set()
    return {text[i:i + 2] for i in range(len(text) - 1)}

_TERM_INDEX = {}

for _canonical, _terms in _ALIASES.items():
    for _term in (_canonical,) + _terms:
        _TERM_INDEX[_norm(_term)] = _canonical

def _terms(text: str) -> set:
    """All normalised forms of a label, including its alias group."""
    norm = _norm(text)
    if not norm:
        return set()
    terms = {norm}
    canonical = _TERM_INDEX.get(norm)
    if canonical:
        terms.add(canonical)
        terms.update((_norm(term) for term in _ALIASES[canonical]))
    return terms

def label_score(query: str, label: str) -> float:
    """Similarity in [0, 1]; exact alias groups score highest."""
    query, label = (_norm(query), _norm(label))
    if not query or not label:
        return 0.0
    if query == label:
        return 1.0
    if _terms(query) & _terms(label):
        return 0.95
    if len(query) >= 2 and (query in label or label in query):
        return 0.85
    query_tokens, label_tokens = (set(query.split()), set(label.split()))
    if query_tokens & label_tokens:
        return 0.8
    ratio = difflib.SequenceMatcher(None, query, label).ratio()
    union = _bigrams(query) | _bigrams(label)
    if union:
        ratio = max(ratio, len(_bigrams(query) & _bigrams(label)) / len(union))
    return min(0.8, ratio)

def match_memory(query: str, memory: dict, threshold: float=MIN_MATCH) -> dict:
    """Best remembered object for a free-form label, with ambiguity candidates."""
    ranked = []
    for key, entry in (memory.get('objects') or {}).items():
        if not isinstance(entry, dict):
            continue
        label = entry.get('label') or key
        score = label_score(query, label)
        if score >= threshold:
            ranked.append({'key': key, 'label': label, 'score': round(score, 3)})
    ranked.sort(key=lambda item: item['score'], reverse=True)
    if not ranked:
        return {'state': 'none', 'candidates': []}
    if len(ranked) > 1 and ranked[0]['key'] != ranked[1]['key'] and (ranked[0]['score'] - ranked[1]['score'] < AMBIGUITY_MARGIN):
        return {'state': 'ambiguous', 'candidates': ranked[:5]}
    return {'state': 'matched', **ranked[0], 'candidates': ranked[:5]}

def _finite(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)

def format_time(timestamp: float) -> str:
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))

def empty_memory() -> dict:
    return {'version': _VERSION, 'updated_at': None, 'objects': {}, 'scenes': {}}

def _valid_entry(entry) -> bool:
    return isinstance(entry, dict) and isinstance(entry.get('label'), str) and isinstance(entry.get('last_seen'), (int, float))

def load_memory(path) -> dict:
    """Load the persisted memory; any corruption degrades to an empty memory."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except Exception:
        return empty_memory()
    if not isinstance(data, dict) or data.get('version') != _VERSION:
        return empty_memory()
    objects = data.get('objects')
    if not isinstance(objects, dict):
        return empty_memory()
    scenes = data.get('scenes') if isinstance(data.get('scenes'), dict) else {}
    return {'version': _VERSION, 'updated_at': data.get('updated_at'), 'objects': {key: entry for key, entry in objects.items() if isinstance(key, str) and _valid_entry(entry)}, 'scenes': {key: scene for key, scene in scenes.items() if isinstance(key, str) and isinstance(scene, dict)}}

def save_memory(path, memory: dict) -> None:
    """Atomic write; a failed save never leaves a half-written memory file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(memory, ensure_ascii=False, indent=1), encoding='utf-8')
    temporary.replace(path)

def merge_memory(memory: dict, scene: dict, evidence_path: str, note: str=None) -> dict:
    """Upsert every remembered label with the newest scene's observation."""
    objects = memory.setdefault('objects', {})
    by_id = {node['id']: node for node in scene['nodes']}
    newest = {}
    for node in scene['nodes']:
        key = node.get('label_key') or _norm(node.get('label', ''))
        if key and (key not in newest or node['confidence'] > newest[key]['confidence']):
            newest[key] = node
    for key, node in newest.items():
        relations = []
        for edge in scene['edges']:
            if edge['source'] == node['id']:
                target = by_id.get(edge['target'])
                if target:
                    relations.append({'relation': edge['relation'], 'target': target['label']})
            elif edge['target'] == node['id']:
                source = by_id.get(edge['source'])
                if source:
                    relations.append({'relation': _INVERSE.get(edge['relation'], edge['relation']), 'target': source['label']})
        previous = objects.get(key) or {}
        entry = {'label': node['label'], 'position': node.get('robot'), 'pixel': node['pixel'], 'scene_width': scene['width'], 'distance_m': node.get('distance_m'), 'confidence': node['confidence'], 'scene_id': scene['id'], 'evidence': evidence_path, 'last_seen': scene['captured_at'], 'first_seen': previous.get('first_seen', scene['captured_at']), 'sightings': int(previous.get('sightings', 0)) + 1, 'relations': relations}
        if note:
            entry['note'] = note
        elif previous.get('note'):
            entry['note'] = previous['note']
        objects[key] = entry
    memory['updated_at'] = scene['captured_at']
    memory.setdefault('scenes', {})[scene['id']] = scene
    if len(memory['scenes']) > MAX_SCENES:
        ordered = sorted(memory['scenes'].items(), key=lambda item: item[1].get('captured_at', 0))
        for stale, _ in ordered[:-MAX_SCENES]:
            memory['scenes'].pop(stale, None)
    return memory

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
