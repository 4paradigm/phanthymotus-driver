"""Named measured joint targets; storage never issues SDK commands."""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import unicodedata

from common.vendor_runtime import action_schema, tool
from device import RM75Plugin

WAYPOINT_DIRECTORY = Path('/opt/phanthy-motus/data/waypoints/realman')


def _name(value):
    if (not isinstance(value, str) or not 1 <= len(value) <= 64
            or value != unicodedata.normalize('NFC', value)
            or re.fullmatch(r'[\w\-\u4e00-\u9fff]+', value) is None):
        raise ValueError('点位名称须为 1–64 个字母、数字、中文、下划线或短横线')
    return value


def _numbers(value, count):
    if (not isinstance(value, list) or len(value) != count
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError(f'状态必须包含 {count} 个有限数值')
    return value


def _frame(value):
    if not isinstance(value, dict) or not isinstance(value.get('name'), str) or not value['name']:
        raise ValueError('工具/工作坐标系名称不可读取')
    _numbers(value.get('pose'), 6)
    _numbers([value.get(k) for k in ('payload', 'x', 'y', 'z')], 4)
    return value


def _identity(value):
    if (not isinstance(value, dict) or value.get('arm_dof') != 7
            or value.get('arm_model') != 'RM_75'
            or value.get('force_type') not in {'B', 'ZF', '6F', '6FB', 'B-V', '6FB-V'}):
        raise ValueError('无法确认 RM75 七轴型号和末端类型')
    return {k: value[k] for k in ('arm_model', 'arm_dof', 'force_type')}


def _record(value):
    if not isinstance(value, dict):
        raise ValueError('点位记录格式错误')
    _name(value.get('name'))
    _numbers(value.get('joint_degree'), 7)
    _identity(value.get('robot'))
    _frame(value.get('tool_frame'))
    _frame(value.get('work_frame'))
    if value.get('tcp_pose_m_rad') is not None:
        _numbers(value['tcp_pose_m_rad'], 6)
    datetime.fromisoformat(value['saved_at'])
    return value


class WaypointStore:
    """One versioned JSON document, guarded across threads and processes.

    The constructor's directory is an internal test seam, never a card parameter.
    """
    def __init__(self, directory=WAYPOINT_DIRECTORY):
        self.directory = Path(directory)
        self._lock = threading.Lock()

    @contextmanager
    def _locked(self):
        with self._lock:
            self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
            with (self.directory / '.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                try:
                    yield
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)

    def _read(self):
        path = self.directory / 'waypoints.json'
        try:
            with path.open(encoding='utf-8') as stream:
                data = json.load(stream)
        except FileNotFoundError:
            return {}
        except (ValueError, UnicodeError) as exc:
            raise ValueError('点位文件损坏，拒绝读取或覆盖，请恢复备份') from exc
        try:
            if (not isinstance(data, dict) or data.get('version') != 1
                    or not isinstance(data.get('waypoints'), dict)):
                raise ValueError('未知文件格式')
            for name, record in data['waypoints'].items():
                if _record(record)['name'] != name:
                    raise ValueError('点位索引与名称不一致')
        except (ValueError, TypeError, KeyError) as exc:
            raise ValueError('点位文件损坏，拒绝读取或覆盖，请恢复备份') from exc
        return data['waypoints']

    def _write(self, records):
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.directory,
                                             prefix='.waypoints-', delete=False) as stream:
                temporary = stream.name
                json.dump({'version': 1, 'waypoints': records}, stream,
                          ensure_ascii=False, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.directory / 'waypoints.json')
            directory_fd = os.open(self.directory, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def list(self):
        with self._locked():
            return [v for _, v in sorted(self._read().items())]

    def get(self, name):
        _name(name)
        with self._locked():
            records = self._read()
            if name not in records:
                raise ValueError(f'点位不存在：{name}')
            return records[name]

    def save(self, record, overwrite=False):
        _record(record)
        with self._locked():
            records = self._read()
            if record['name'] in records and overwrite is not True:
                raise ValueError('同名点位已存在；覆盖须显式设置 overwrite=true')
            records[record['name']] = record
            self._write(records)

    def delete(self, name):
        _name(name)
        with self._locked():
            records = self._read()
            if name not in records:
                raise ValueError(f'点位不存在：{name}')
            del records[name]
            self._write(records)


class WaypointPlugin(RM75Plugin):
    """Reuse joint execution directly, including its limits, watchdog and ACP."""
    PREFIX = 'waypoint'

    def __init__(self, client, config, arm_plugin):
        super().__init__(client, config)
        if config.get('arm_dof', 7) != 7:
            raise ValueError('点位卡片仅支持七轴配置')
        self._arm = arm_plugin
        self._motion_state = arm_plugin._motion_state
        self._action_lock = arm_plugin._action_lock
        self._submission_lock = arm_plugin._submission_lock
        self._cancelled = arm_plugin._cancelled
        self.store = WaypointStore()
        self._operation_lock = threading.Lock()

    def get_tools(self):
        schema = action_schema({
            'save': (['name', 'overwrite'], '只读保存当前实测七轴关节角与坐标配置，不移动机械臂'),
            'list': ([], '列出点位、关节角、坐标配置和保存时间，不移动机械臂'),
            'delete': (['name'], '按名称删除点位，不移动机械臂'),
            'move': (['name', 'speed_percent', 'confirm_motion'], '关节空间返回点位；路径不具备自动避障，请先清空路径'),
            'info': ([], '查看本卡片当前动作及最近结果，不移动机械臂'),
            'cancel': ([], '中止本卡片正在执行的运动，不中止其他卡片的动作'),
        }, {
            'name': {'type': 'string', 'minLength': 1, 'maxLength': 64, 'description': '点位名称：中文、字母、数字、下划线或短横线'},
            'overwrite': {'type': 'boolean', 'default': False, 'description': '仅 true 允许覆盖同名点位'},
            'confirm_motion': {'type': 'boolean', 'default': False, 'description': '移动前必须显式设为 true'},
            'speed_percent': {'type': 'integer', 'minimum': 1, 'maximum': self.max_speed_percent,
                              'default': self.default_speed_percent, 'description': '关节运动速度百分比，遵守驱动速度上限'},
        })
        schema['properties']['action']['description'] = '选择保存、查询、删除、返回点位、状态或取消动作'
        schema['x-completion'] = {'actions': ['move'], 'timeout': 305}
        schema['x-hooks'] = {'on_interrupt_motion': {'action': 'cancel'},
                             'on_interrupt_all': {'action': 'cancel'}}
        schema['x-resource'] = 'arm'
        schema['x-is-dangerous'] = True
        definition = tool(self.PREFIX, 'actuator', '点位管理：保存实测七轴关节角并返回单个点位；不提供路径自动避障', schema)
        definition['title'] = '点位管理'
        return [definition]

    def start(self):
        pass

    def stop(self):
        with self._operation_lock:
            self._stop_motion()

    def _configuration(self):
        return {'robot': _identity(self.client.call('rm_get_robot_info')),
                'tool_frame': _frame(self.client.call('rm_get_current_tool_frame')),
                'work_frame': _frame(self.client.call('rm_get_current_work_frame'))}

    def _prepare_target(self, args):
        # Called by the inherited executor AFTER acquiring the shared motion gate.
        record = self.store.get(args['name'])
        current_config = self._configuration()
        for key, value in current_config.items():
            if record[key] != value:
                raise ValueError(f'点位 {key} 与当前配置不兼容，拒绝运动；不会自动切换坐标系')
        motion_args = {'confirm_motion': args.get('confirm_motion'),
                       'speed_percent': args.get('speed_percent', self.default_speed_percent)}
        speed = motion_args['speed_percent']
        if isinstance(speed, bool) or not isinstance(speed, int):
            raise ValueError('speed_percent 必须是整数')
        motion_args.update({f'joint{i}_deg': angle for i, angle in enumerate(record['joint_degree'], 1)})
        return super()._prepare_target(motion_args)

    def _start_motion(self, args):
        # Cartesian cached poses belong to the original joint card.
        self._cartesian_pose_invalidator = self._arm._cartesian_pose_invalidator
        return super()._start_motion(args)

    def _stop_motion(self):
        # Same ordering as joint submission; no stale ID can stop a later owner.
        with self._action_lock:
            action_id = self._active_action_id
            if action_id is None:
                return {'state': 'idle', 'action_id': None}
            with self._submission_lock:
                self.client.command('rm_set_arm_slow_stop')
            self._cancelled.add(action_id)
        return {'state': 'stop_requested', 'action_id': action_id}

    def dispatch(self, action, args):
        if action == 'start':
            return {'state': 'ready'}
        if action in ('stop', 'cancel'):
            # Serialize with validation/submission as well as active monitoring.
            with self._operation_lock:
                result = self._stop_motion()
            return {'state': 'idle'} if action == 'stop' else result
        if action == 'info':
            with self._action_lock:
                return {'state': 'running' if self._active_action_id else 'ready',
                        'active_action_id': self._active_action_id,
                        'last_completion': self._last_completion,
                        'storage_directory': str(WAYPOINT_DIRECTORY)}
        if action == 'list':
            records = self.store.list()
            return {'waypoints': records, 'count': len(records)}
        if action == 'delete':
            self.store.delete(args.get('name'))
            return {'state': 'deleted', 'name': args['name']}
        if action == 'save':
            name = _name(args.get('name'))
            if not self._motion_lock.acquire(blocking=False):
                raise RuntimeError('机械臂运动中，拒绝保存点位')
            try:
                configuration = self._configuration()
                joints = _numbers(self.client.call('rm_get_joint_degree'), 7)
                # TCP and coordinate queries must succeed; no guessed metadata.
                state = self.client.call('rm_get_current_arm_state')
                pose = _numbers(state.get('pose'), 6)
                if self._configuration() != configuration:
                    raise ValueError('采集期间坐标配置变化，请重试')
                record = {'name': name, 'joint_degree': joints, **configuration,
                          'tcp_pose_m_rad': pose, 'saved_at': datetime.now(timezone.utc).isoformat()}
                self.store.save(record, args.get('overwrite', False))
                return {'state': 'saved', 'waypoint': record}
            finally:
                self._motion_lock.release()
        if action == 'move':
            _name(args.get('name'))
            with self._operation_lock:
                return self._start_motion(args)
        return None
