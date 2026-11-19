"""Operator-provisioned normal states and local report drafts for office patrol."""
from copy import deepcopy
from datetime import datetime, timezone
import json
import re
from threading import Lock
from pathlib import Path


NAME = 'patrol_baseline'
SCHEMA = 'motus.patrol-baseline/1'
EVIDENCE_KINDS = ('visual', 'sensor')
_IDENTIFIER = re.compile(r'[a-z0-9][a-z0-9_-]{0,63}')


def _fields(value, required, label):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ValueError(f'{label} requires exactly: {", ".join(required)}')
    return value


def _id(value, label):
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError(f'{label} must be a lowercase identifier (up to 64 characters)')
    return value


def _text(value, label, limit):
    if (not isinstance(value, str) or not value.strip() or len(value) > limit
            or any(ord(char) < 32 for char in value)):
        raise ValueError(f'{label} must be nonblank single-line text (up to {limit} characters)')
    return value.strip()


def _items(value, label, limit):
    if not isinstance(value, list) or not 1 <= len(value) <= limit:
        raise ValueError(f'{label} must contain 1 to {limit} records')
    return value


def _timestamp(value):
    if not isinstance(value, str) or len(value) > 40 or 'T' not in value:
        raise ValueError('captured_at must be an ISO timestamp with time zone')
    stamp = datetime.fromisoformat(value[:-1] + '+00:00' if value.endswith('Z') else value)
    if stamp.utcoffset() is None:
        raise ValueError('captured_at must include a time zone')
    return stamp.astimezone(timezone.utc)


def tool_spec():
    return {
        'name': NAME, 'title': '巡逻正常基线与报告草稿', 'type': 'processor',
        'description': '读取管理员配置的正常状态、路线与上报分流，按 Agent 的判断生成草稿。'
                       '不控制机器人，不发送消息。',
        'multiInstance': False, 'topic_in': [], 'topic_out': [],
        'inputSchema': {
            'type': 'object', 'required': ['action'],
            'properties': {
                'action': {'type': 'string', 'enum': ['info', 'start', 'stop', 'plan', 'read', 'prepare_report']},
                'instance_id': {'type': 'string'},
                'point_id': {'type': 'string', 'description': 'plan 返回的点位标识'},
                'check_id': {'type': 'string', 'description': 'read 返回的检查项标识'},
                'run_id': {'type': 'string', 'description': '本轮巡逻的稳定标识'},
                'revision': {'type': 'string', 'description': '本轮 plan 返回的基线版本'},
                'decision': {'type': 'string', 'enum': ['deviation', 'needs_review']},
                'priority': {'type': 'string', 'enum': ['routine', 'urgent']},
                'assessment': {'type': 'string', 'description': 'Agent 对偏离及其潜在影响的简短说明'},
                'evidence': {'type': 'object', 'description': '可信观测的原始元数据和可核实描述。手动表单可粘贴 JSON。',
                             'required': ['point_id', 'view_id', 'kind', 'captured_at', 'reference', 'observed'],
                             'properties': {key: {'type': 'string'} for key in
                                            ('point_id', 'view_id', 'kind', 'captured_at', 'reference', 'observed')}},
            },
            'x-action-params': {
                'info': {'params': []}, 'start': {'params': []}, 'stop': {'params': []},
                'plan': {'params': [], 'description': '读取巡逻顺序和当前基线版本'},
                'read': {'params': ['point_id'], 'required': ['point_id'],
                         'description': '读取一个点位的正常状态和观测要求'},
                'prepare_report': {'params': ['run_id', 'revision', 'point_id', 'check_id', 'decision', 'priority',
                                              'assessment', 'evidence'],
                                   'required': ['run_id', 'revision', 'point_id', 'check_id', 'decision'],
                                   'description': '生成结构化草稿，由 Agent 判定偏离并选优先级'},
            },
        },
    }


class PatrolBaseline:
    """A validated snapshot of operator configuration, read once at service startup."""

    def __init__(self, config):
        _fields(config, ('schema', 'site_id', 'revision', 'mode', 'max_age_seconds',
                         'review_route', 'routes', 'points'), 'baseline')
        if config['schema'] != SCHEMA or config['mode'] not in ('rehearsal', 'site'):
            raise ValueError('Unsupported baseline schema or mode')
        self.site_id = _id(config['site_id'], 'site_id')
        self.revision = _id(config['revision'], 'revision')
        self.mode = config['mode']
        self._state = 'idle'
        self._lock = Lock()
        self.max_age = config['max_age_seconds']
        if type(self.max_age) is not int or not 1 <= self.max_age <= 300:
            raise ValueError('max_age_seconds must be an integer from 1 to 300')
        routes = config['routes']
        if not isinstance(routes, dict) or not 1 <= len(routes) <= 32:
            raise ValueError('routes must contain 1 to 32 route labels')
        self.routes = {_id(key, 'route_id'): _text(label, 'route label', 96) for key, label in routes.items()}
        self.review_route = self._route(config['review_route'])
        self.points = {}
        for record in _items(config['points'], 'points', 64):
            point = _fields(record, ('point_id', 'label', 'waypoint_id', 'view_id', 'checks'), 'point')
            key = _id(point['point_id'], 'point_id')
            if key in self.points:
                raise ValueError(f'Duplicate point: {key}')
            checks = {}
            for item in _items(point['checks'], 'checks', 16):
                check = _fields(item, ('check_id', 'normal', 'evidence_kind', 'route_id'), 'check')
                check_id = _id(check['check_id'], 'check_id')
                if check_id in checks or check['evidence_kind'] not in EVIDENCE_KINDS:
                    raise ValueError(f'Duplicate check or unsupported evidence kind: {check_id}')
                checks[check_id] = {'check_id': check_id, 'normal': _text(check['normal'], 'normal', 480),
                                    'evidence_kind': check['evidence_kind'], 'route_id': self._route(check['route_id'])}
            self.points[key] = {
                'point_id': key, 'label': _text(point['label'], 'point label', 96),
                'waypoint_id': _id(point['waypoint_id'], 'waypoint_id'),
                'view_id': _id(point['view_id'], 'view_id'), 'checks': checks,
            }

    def _route(self, value):
        value = _id(value, 'route_id')
        if value not in self.routes:
            raise ValueError(f'Unknown route: {value}')
        return value

    def _meta(self):
        return {'site_id': self.site_id, 'revision': self.revision, 'mode': self.mode,
                'max_age_seconds': self.max_age}

    def call(self, args):
        action = args.get('action')
        if action in ('info', 'start', 'stop'):
            with self._lock:
                if action != 'info':
                    self._state = 'idle' if action == 'stop' else 'ready'
                state = self._state
            return {**self._meta(), 'state': state,
                    'topic_in': [], 'topic_out': [], 'hardware_command': False}
        if action == 'plan':
            return {**self._meta(), 'points': [{k: v for k, v in point.items() if k != 'checks'}
                                              for point in self.points.values()]}
        if action not in ('read', 'prepare_report'):
            raise ValueError('Unknown patrol action')
        point_id = _id(args.get('point_id'), 'point_id')
        if point_id not in self.points:
            raise ValueError(f'Unknown point: {point_id}')
        point = self.points[point_id]
        if action == 'read':
            return {**self._meta(), **point, 'checks': deepcopy(list(point['checks'].values()))}
        check_id = _id(args.get('check_id'), 'check_id')
        if check_id not in point['checks']:
            raise ValueError(f'Unknown check for {point_id}: {check_id}')
        return self._prepare_report(args, point, point['checks'][check_id])

    def _prepare_report(self, args, point, check):
        run_id = _id(args.get('run_id'), 'run_id')
        if args.get('revision') != self.revision:
            raise ValueError('Baseline revision changed or missing; refresh plan before preparing a report')
        decision = args.get('decision')
        priority = args.get('priority', 'routine')
        if decision not in ('deviation', 'needs_review') or priority not in ('routine', 'urgent'):
            raise ValueError('Unsupported decision or priority')
        evidence, reason = self._evidence(args.get('evidence'), point, check)
        confirmed = decision == 'deviation' and evidence is not None
        assessment = _text(args.get('assessment'), 'assessment', 480) if confirmed else None
        route = check['route_id'] if confirmed else self.review_route
        status = 'deviation_report' if confirmed else 'needs_review'
        return {
            **self._meta(), 'run_id': run_id,
            'report_key': '/'.join((self.site_id, self.revision, run_id, point['point_id'],
                                    check['check_id'], status)),
            'point_id': point['point_id'], 'point_label': point['label'], 'check_id': check['check_id'],
            'normal': check['normal'], 'agent_decision': decision, 'status': status,
            'priority': priority if confirmed else 'review',
            'assessment': assessment,
            'route_id': route, 'route_label': self.routes[route],
            'evidence': evidence, 'review_reason': None if confirmed else reason or 'agent_needs_review',
            'delivery': 'rehearsal_only' if self.mode == 'rehearsal' else 'pending', 'sent': False,
        }

    def _evidence(self, value, point, check):
        if value is None:
            return None, 'missing_evidence'
        try:
            # Dashboard's manual canvas form sends pasted objects as JSON text.
            if isinstance(value, str):
                if len(value) > 2048:
                    return None, 'invalid_evidence'
                value = json.loads(value)
            ev = _fields(value, ('point_id', 'view_id', 'kind', 'captured_at', 'reference', 'observed'), 'evidence')
            captured = _timestamp(ev['captured_at'])
            reference = _text(ev['reference'], 'reference', 128)
            observed = _text(ev['observed'], 'observed', 480)
        except (ValueError, OverflowError, RecursionError):
            return None, 'invalid_evidence'
        if (ev['point_id'] != point['point_id'] or ev['view_id'] != point['view_id']
                or ev['kind'] != check['evidence_kind']):
            return None, 'point_view_or_source_mismatch'
        age = (datetime.now(timezone.utc) - captured).total_seconds()
        if not 0 <= age <= self.max_age:
            return None, 'evidence_outside_time_window'
        return {**ev, 'captured_at': captured.isoformat(), 'reference': reference, 'observed': observed}, None


MAX_BASELINE_BYTES = 2 * 1024 * 1024
DEFAULT_BASELINE_PATH = 'examples/patrol-baseline-bumi-fixed.json'


class PatrolBaselinePlugin:
    """Bumi adapter for an operator-owned, read-only baseline snapshot.

    The constructor accepts the standard bundle arguments but creates no ROS
    nodes, hardware clients, network sessions or background workers. Updating a
    baseline requires an operator to edit the configured file and restart the
    driver; tool calls can neither select another file nor rewrite this snapshot.
    """

    def __init__(self, plugin_config, namespace, executor):
        if not isinstance(plugin_config, dict):
            raise ValueError('patrol_baseline plugin configuration must be an object')
        configured_path = plugin_config.get('baseline_path', DEFAULT_BASELINE_PATH)
        if not isinstance(configured_path, str) or not configured_path.strip():
            raise ValueError('patrol_baseline baseline_path must be a nonblank string')
        path = Path(configured_path)
        if not path.is_absolute():
            path = Path(__file__).resolve().parent / path
        try:
            with path.open('rb') as handle:
                raw = handle.read(MAX_BASELINE_BYTES + 1)
            if len(raw) > MAX_BASELINE_BYTES:
                raise ValueError('baseline file exceeds 2 MiB')
            config = json.loads(raw.decode('utf-8'))
            self._baseline = PatrolBaseline(config)
        except (OSError, ValueError, RecursionError) as exc:
            raise ValueError(f'Cannot load patrol baseline {path}: {exc}') from exc

    def get_tool(self):
        return tool_spec()

    def start(self):
        return self._baseline.call({'action': 'start'})

    def stop(self):
        return self._baseline.call({'action': 'stop'})

    def dispatch(self, action, args):
        if not isinstance(args, dict):
            raise ValueError('patrol_baseline arguments must be an object')
        if 'baseline_path' in args:
            raise ValueError('baseline_path is operator configuration, not a tool argument')
        return self._baseline.call({**args, 'action': action})
