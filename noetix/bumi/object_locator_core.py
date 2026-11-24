"""Shared query core for the Bumi `object_locator` card.

ROS-free helpers used by `object_locator_card.py`: read the on-disk memory,
parse Chinese/English "where is my xxx" questions, match remembered labels,
build the answer with spatial relations, and draw/save the evidence image.
No MCP card is registered from this file.

The memory file is the one written by `object_memory`: `objects` keyed by
normalised label and a bounded `scenes` map with the original scene graphs.
"""

from __future__ import annotations

import difflib
import json
import math
import re
import time
from pathlib import Path

MAX_EVIDENCE_FILES = 200

MIN_MATCH = 0.62

AMBIGUITY_MARGIN = 0.08

MAX_QUERY_CHARS = 80

_VERSION = 1

_ALIASES = {'keys': ('钥匙', 'key', 'keys', '门钥匙'), 'wallet': ('钱包', 'wallet', 'purse'), 'phone': ('手机', 'phone', 'cellphone', 'mobile', 'smartphone'), 'glasses': ('眼镜', 'glasses', 'spectacles'), 'cup': ('杯子', '水杯', '马克杯', 'cup', 'mug'), 'bottle': ('瓶子', '水壶', '保温杯', 'bottle', 'flask'), 'laptop': ('电脑', '笔记本', 'laptop', 'computer', 'notebook'), 'remote': ('遥控器', 'remote', 'remote control'), 'headphones': ('耳机', 'headphones', 'earbuds', 'earphone'), 'charger': ('充电器', 'charger', 'adapter'), 'umbrella': ('雨伞', 'umbrella'), 'mask': ('口罩', 'mask'), 'book': ('书', '书本', 'book'), 'pen': ('笔', '钢笔', 'pen'), 'watch': ('手表', 'watch'), 'bag': ('包', '背包', '书包', 'bag', 'backpack'), 'tissue': ('纸巾', '抽纸', 'tissue', 'paper'), 'medicine': ('药', '药品', 'medicine', 'pill'), 'table': ('桌子', '桌', 'table', 'desk'), 'chair': ('椅子', 'chair'), 'sofa': ('沙发', 'sofa', 'couch'), 'bed': ('床', 'bed'), 'door': ('门', 'door'), 'window': ('窗户', '窗', 'window'), 'fridge': ('冰箱', 'fridge', 'refrigerator'), 'microwave': ('微波炉', 'microwave'), 'tv': ('电视', 'tv', 'television'), 'monitor': ('显示器', '屏幕', 'monitor', 'screen'), 'keyboard': ('键盘', 'keyboard'), 'mouse': ('鼠标', 'mouse'), 'snack': ('零食', 'snack'), 'apple': ('苹果', 'apple'), 'banana': ('香蕉', 'banana')}

_RELATION_ZH = {'left_of': '左边', 'right_of': '右边', 'in_front_of': '前方', 'behind': '后方', 'above': '上方', 'below': '下方', 'near': '附近'}

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

def _display_label(label: str) -> str:
    """Prefer a Chinese alias for a remembered English VOP label."""
    if not isinstance(label, str):
        return str(label)
    if re.search('[\\u4e00-\\u9fff]', label):
        return label
    canonical = _TERM_INDEX.get(_norm(label))
    if canonical:
        for term in _ALIASES.get(canonical, ()):
            if re.search('[\\u4e00-\\u9fff]', term):
                return term
    return label

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

_PATTERNS = (re.compile('我的(?P<label>.+?)(?:在|放|丢|落)(?:哪里|哪儿|哪|什么地方)'), re.compile('我(?P<label>.+?)(?:在|放)(?:哪里|哪儿|哪|什么地方)'), re.compile('我的(?P<label>.+?)(?:在哪里呢|在哪儿呢|呢)\\s*[？?]?$'), re.compile('(?P<label>.+?)(?:在|放)(?:哪里|哪儿|哪|什么地方)'), re.compile('(?:找|看见|看到|见)(?:一下|下)?(?:我的)?(?P<label>.+?)(?:了吗|吗|呢|？|\\?|$)'), re.compile('where\\s+(?:is|are)\\s+(?:my|the)\\s+(?P<label>.+?)(?:\\?|$)', re.I), re.compile('find\\s+(?:my\\s+)?(?P<label>.+?)(?:\\?|$)', re.I))

_LEADING = ('我的', '一下', '请问', '帮我', '找一下', '找找', '看看')

_TRAILING = ('在哪里', '在哪儿', '在哪', '放哪', '丢哪', '呢', '吗', '了', '的', '啊', '呀')

def _clean_label(text: str) -> str:
    text = text.strip(' \t\r\n，,。.？?！!：:；;"\'“”‘’')
    for prefix in _LEADING:
        if text.startswith(prefix):
            text = text[len(prefix):]
    for suffix in _TRAILING:
        if text.endswith(suffix):
            text = text[:-len(suffix)]
    return text.strip(' 的')

def extract_target(question) -> str:
    """The object phrase in a question, or "" when it is not a 'where' question."""
    if not isinstance(question, str):
        return ''
    question = question.strip()[:MAX_QUERY_CHARS]
    if not question:
        return ''
    for pattern in _PATTERNS:
        match = pattern.search(question)
        if match:
            label = _clean_label(match.group('label'))
            if label and len(label) <= 64:
                return label
    return ''

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

def _direction_text(entry: dict) -> str:
    robot = entry.get('position')
    if isinstance(robot, dict):
        forward, left = (float(robot.get('forward', 0)), float(robot.get('left', 0)))
        if forward < 0.2:
            return '机器人身后'
        if abs(left) < 0.1:
            return f'机器人正前方，距机器人约 {forward:.2f} 米'
        side = '左' if left > 0 else '右'
        return f'机器人前方偏{side} {abs(left):.2f} 米，距机器人约 {forward:.2f} 米'
    pixel = entry.get('pixel') or []
    width = int(entry.get('scene_width') or DEFAULT_WIDTH)
    if len(pixel) == 2 and width > 0:
        ratio = float(pixel[0]) / width
        if ratio < 0.4:
            return '画面左侧（没有深度，无法给出距离）'
        if ratio > 0.6:
            return '画面右侧（没有深度，无法给出距离）'
        return '画面中央（没有深度，无法给出距离）'
    return '上次看到的位置'

def _relation_texts(entry: dict) -> list:
    texts = []
    for relation in entry.get('relations') or []:
        if not isinstance(relation, dict):
            continue
        target = relation.get('target')
        kind = relation.get('relation')
        if not isinstance(target, str) or kind not in _RELATION_ZH:
            continue
        if kind == 'near':
            texts.append(f'{_display_label(target)}附近')
        else:
            texts.append(f'{_display_label(target)}的{_RELATION_ZH[kind]}')
        if len(texts) >= 2:
            break
    return texts

def answer_question(memory: dict, question: str) -> dict:
    """Match a question against memory and build a human-readable answer."""
    target = extract_target(question)
    if not target:
        return {'state': 'error', 'code': 'NO_TARGET', 'message': '没听清要找什么，请说“我的xxx在哪”'}
    match = match_memory(target, memory)
    if match['state'] == 'none':
        return {'state': 'not_found', 'target': target, 'message': f'还没记住“{target}”。可以先让我看一眼房间（action=remember）', 'candidates': []}
    if match['state'] == 'ambiguous':
        labels = '、'.join((candidate['label'] for candidate in match['candidates'][:3]))
        return {'state': 'ambiguous', 'target': target, 'message': f'我记住了几个相近的东西：{labels}；请说得更具体一点', 'candidates': match['candidates']}
    entry = (memory.get('objects') or {}).get(match['key']) or {}
    where = _relation_texts(entry)
    place = '，'.join(where) if where else _direction_text(entry)
    answer = f'你的{target}在{place}'
    if entry.get('distance_m') is not None and where:
        answer += f"（距机器人约 {entry['distance_m']:.2f} 米）"
    answer += f"。上次看到：{format_time(entry['last_seen'])}"
    return {'state': 'found', 'target': target, 'label': entry.get('label'), 'answer': answer, 'score': match['score'], 'position': entry.get('position'), 'pixel': entry.get('pixel'), 'distance_m': entry.get('distance_m'), 'relations': entry.get('relations') or [], 'last_seen': entry.get('last_seen'), 'last_seen_text': format_time(entry['last_seen']), 'scene_id': entry.get('scene_id'), 'evidence_path': entry.get('evidence'), 'candidates': match['candidates']}

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
