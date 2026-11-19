"""Controlled mapping, map-bound semantic tags and native Kuavo navigation."""
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
from uuid import uuid4

from spatial_client import JatenClient


def text(value, field):
    if (not isinstance(value, str) or not value.strip() or len(value) > 80
            or any(ord(c) < 32 for c in value) or any(c in value for c in '/\\')):
        raise ValueError('invalid_' + field)
    return value


def number(value, field, low=None, high=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(field + '_must_be_finite_number')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(field + '_out_of_range')
    return float(value)


class SpatialStore:
    def __init__(self, path):
        self.path, self._db = Path(path), None
        self.lock = threading.RLock()

    def db(self):
        if self._db is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._db = sqlite3.connect(self.path, check_same_thread=False)
            self._db.row_factory = sqlite3.Row
            self._db.executescript('''
                CREATE TABLE IF NOT EXISTS tags (
                    map_id TEXT, fingerprint TEXT, name TEXT, description TEXT,
                    x REAL, y REAL, yaw REAL, PRIMARY KEY(map_id,fingerprint,name));
                CREATE TABLE IF NOT EXISTS state (key TEXT PRIMARY KEY, value TEXT);
            ''')
            self._db.commit()
        return self._db

    def state(self, key, value=...):
        with self.lock:
            db = self.db()
            if value is ...:
                row = db.execute('SELECT value FROM state WHERE key=?', (key,)).fetchone()
                return json.loads(row[0]) if row else None
            db.execute('INSERT OR REPLACE INTO state VALUES (?,?)', (key, json.dumps(value)))
            db.commit()

    def tags(self, identity):
        with self.lock:
            return [dict(row) for row in self.db().execute(
                'SELECT name,description,x,y,yaw FROM tags WHERE map_id=? AND fingerprint=? ORDER BY name', identity)]

    def tag(self, identity, name, description, pose):
        with self.lock:
            self.db().execute('INSERT OR REPLACE INTO tags VALUES (?,?,?,?,?,?,?)',
                              (*identity, name, description, pose['x'], pose['y'], pose['yaw']))
            self.db().commit()

    def untag(self, identity, name):
        with self.lock:
            cursor = self.db().execute('DELETE FROM tags WHERE map_id=? AND fingerprint=? AND name=?',
                                       (*identity, name))
            self.db().commit()
            return cursor.rowcount > 0

    def bind_mapping(self, previous, identity):
        with self.lock:
            self.db().execute('UPDATE tags SET map_id=?,fingerprint=? WHERE map_id=? AND fingerprint=?',
                              (*identity, *previous))
            self.db().commit()

    def close(self):
        with self.lock:
            if self._db:
                self._db.close()
                self._db = None


class ControlledSpatialPlugin:
    PREFIX, CHANNEL = 'controlled_spatial', 'base'
    NAV_TIMEOUT = 150.
    ACTIONS = {
        'start_mapping': (['map_name'], '开始厂商Cartographer建图；人工控制底盘采集'),
        'stop_mapping': ([], '保存当前建图，重启厂商地图服务并确认地图身份'),
        'tag_place': (['name', 'description'], '记录当前位置和朝向，绑定地图身份'),
        'untag_place': (['name'], '删除当前地图中的语义点'),
        'list_tags': ([], '列出当前地图语义点及相对位置'),
        'list_maps': ([], '列出底盘真实栅格地图'),
        'load_map': (['map_name'], '载入已有地图，确认地图身份；不伪造初始定位'),
        'navigate_to_pose': (['x', 'y', 'yaw', 'speed'], '导航到map系位姿，按任务ID跟踪ACP'),
        'navigate_to_tag': (['tag_name', 'speed'], '导航到当前地图的语义点'),
        'stop_nav': ([], '原生取消当前导航并确认新鲜里程计停稳'),
    }

    def __init__(self, bundle, config=None, client=None):
        self.bundle = bundle
        config = config or {}
        self._motion_music_enabled = config.get('motion_music_enabled', False)
        if type(self._motion_music_enabled) is not bool:
            raise ValueError('motion_music_enabled must be a boolean')
        self.client = client or JatenClient(bundle.startup_checks.provisioner)
        self.store = SpatialStore(config.get('db_path', str(Path(os.environ.get('KUAVO_STATE_DIR', '/config'))
                                                           / 'controlled_spatial.db')))
        self._operations = threading.Lock()
        self._native_lock = threading.RLock()
        self._start_lock = threading.Lock()
        self._startup_guard = threading.Lock()
        self._startup = None
        self._startup_key = None
        self._startup_thread = None
        self._preparation = None
        self._task = None
        self._uncertain = False
        self._restored = False
        self._map_pending = None
        self._workers = set()
        self._map_deadline = float('inf')

    def get_tool(self):
        props = {'action': {'type': 'string', 'enum': list(self.ACTIONS)},
                 **{k: {'type': 'string'} for k in ('map_name', 'name', 'description', 'tag_name')},
                 **{k: {'type': 'number', 'description': 'map坐标系位置，米'} for k in ('x', 'y')},
                 'yaw': {'type': 'number', 'description': 'map坐标系绝对朝向，弧度'},
                 'speed': {'type': 'number', 'description': '导航线速度，米/秒', 'minimum': .05, 'maximum': .3, 'default': .15}}
        required = {'start_mapping': ['map_name'], 'tag_place': ['name'], 'untag_place': ['name'],
                    'load_map': ['map_name'], 'navigate_to_pose': ['x', 'y', 'yaw'],
                    'navigate_to_tag': ['tag_name']}
        return {'name': self.PREFIX, 'type': 'actuator', 'multiInstance': False,
                'description': 'Kuavo 5-W人工建图、地图绑定打点与避障导航；使用厂商原生接口。',
                'inputSchema': {'type': 'object', 'properties': props, 'required': ['action'],
                    'x-resource': 'base', 'x-hooks': {'on_interrupt_motion': {'action': 'stop_nav'}},
                    'x-completion': {'actions': ['navigate_to_pose', 'navigate_to_tag', 'start_mapping', 'stop_mapping', 'load_map'], 'timeout': 180},
                    'x-action-params': {k: {'params': p, 'description': d, 'required': required.get(k, [])}
                                        for k, (p, d) in self.ACTIONS.items()}}}

    def _restore(self):
        if not self._restored:
            if self.store.path.exists():
                pending = self.store.state('pending') or {}
                self._task = pending.get('navigation')
                self._uncertain = bool(pending.get('uncertain'))
                self._map_pending = pending.get('map_transition')
                self._preparation = pending.get('chassis_preparation')
            self._restored = True

    def _persist(self):
        self.store.state('pending', {'navigation': self._task, 'uncertain': self._uncertain,
                                    'map_transition': self._map_pending, 'chassis_preparation': self._preparation})

    def unresolved(self):
        self._restore()
        return (self._uncertain or self._task is not None or self._map_pending is not None
                or self._preparation is not None)

    @staticmethod
    def _check_chassis_faults(observation):
        agv = observation['agvState']
        if any(agv.get(k) is not False for k in ('estopButtonEvent', 'bumperEvent', 'driverError')):
            raise RuntimeError('chassis_hardware_stop_or_fault_is_active')

    def start_chassis(self):
        """Share one background preflight across both base cards and a stop epoch."""
        if not self.bundle.ros1.connection[0]:
            return {'state': 'error', 'error': 'control_connection_is_disconnected'}
        key = (self.bundle._stop_epoch, self.bundle.ros1.connection)
        with self._startup_guard:
            if self._startup_thread and self._startup_thread.is_alive():
                return dict(self._startup)
            if self._startup_key == key and self._startup and self._startup['state'] == 'running':
                return dict(self._startup)
            self._startup_key = key
            self._startup = {'state': 'loading', 'message':
                'Checking chassis status in background, please wait. Control will be ready automatically.'}
            def worker():
                try:
                    # Audio configuration is independent of motor state.
                    try:
                        music = self.client.native('motion_music', enabled=self._motion_music_enabled)
                    except Exception:
                        music = {'state': 'unavailable'}
                    result = self.prepare_chassis(request_epoch=key[0])
                    if key != (self.bundle._stop_epoch, self.bundle.ros1.connection):
                        raise RuntimeError('chassis_start_superseded_by_stop_or_reconnect')
                    status = {'state': 'running', 'chassis_preparation': result, 'motion_music': music}
                except Exception as exc:
                    status = {'state': 'error', 'error': str(exc).encode('unicode_escape').decode('ascii')[:400]}
                with self._startup_guard:
                    self._startup = status
            self._startup_thread = threading.Thread(target=worker, daemon=True, name='chassis-start')
            self._workers.add(self._startup_thread)
            self._startup_thread.start()
            return dict(self._startup)

    def startup_status(self):
        with self._startup_guard:
            if self._startup_key != (self.bundle._stop_epoch, self.bundle.ros1.connection):
                return {'state': 'idle'}
            return dict(self._startup or {'state': 'idle'})

    def prepare_chassis(self, request_epoch=None):
        """Explicit card start: cancel old tasks, release soft stop, confirm rest.

        One shared preparation for both cards. Stops supersede its transition;
        the native lock orders a later soft stop after any resolved release.
        """
        epoch = self.bundle._stop_epoch if request_epoch is None else request_epoch
        if not self._start_lock.acquire(timeout=15):
            raise RuntimeError('chassis_preparation_busy')
        token = None
        try:
            self._restore()
            if self.unresolved():
                raise RuntimeError('previous_chassis_operation_unresolved_stop_first')
            if not self.bundle.ros1.connection[0]:
                raise RuntimeError('control_connection_is_disconnected')
            self._check_chassis_faults(self.client.observe())
            with self.bundle.tracker.lock:
                if epoch != self.bundle._stop_epoch:
                    raise RuntimeError('chassis_start_superseded_by_stop')
                token, _, was_blocked = self.bundle._begin_control('prepare_chassis')
            with self._native_lock:
                with self.bundle.tracker.lock:
                    self.bundle._check_transition(token)
                    self._preparation = {'delivery_uncertain': True}
                    self._persist()
                result = self.client.native('prepare')
                if result.get('success') is False and result.get('delivery_uncertain') is False:
                    self._preparation = {'delivery_uncertain': False}
                    self._persist()
                if (result.get('success') is not True or result.get('stationary') is not True
                        or result.get('software_estop') is not False or result.get('old_task_cancelled') is not True):
                    stage = result.get('stage')
                    known = {'read_fault_state', 'cancel_old_task', 'confirm_cancel_stationary',
                             'release_software_stop', 'confirm_release_state', 'confirm_release_stationary',
                             'validate_runtime'}
                    raise RuntimeError('chassis_preparation_failed_at_' + stage if stage in known
                                       else 'chassis_preparation_not_confirmed')
                self._preparation = {'delivery_uncertain': False}
                self._persist()
            observation = self.client.observe()
            self._check_chassis_faults(observation)
            if observation['agvState'].get('softwareEstop') is not False:
                raise RuntimeError('chassis_software_stop_release_unconfirmed')
            state = observation['state']
            # Native prepare already checked distinct, fresh raw /odom samples.
            # The web velocity cache can remain nonzero after driving=false.
            if state.get('driving') is not False:
                raise RuntimeError('chassis_not_stationary_after_start')
            with self.bundle.tracker.lock:
                self.bundle._check_transition(token)
                self._preparation = None
                self._persist()
                self.bundle.state.motion_blocked = was_blocked
            return {**result, 'completion_basis': 'old_task_cancelled_and_fresh_stationary_release_feedback'}
        except Exception:
            if token is not None:
                self.bundle.pause('chassis_start_failed_or_interrupted', expected_transition=token)
            raise
        finally:
            if token is not None:
                self.bundle._end_control(token)
            self._start_lock.release()

    def _identity(self):
        name, fingerprint = self.client.map_identity()
        if not isinstance(fingerprint, str) or not fingerprint or len(fingerprint) > 160:
            raise RuntimeError('invalid_map_fingerprint')
        return text(name, 'map_id'), fingerprint

    def _observation(self, navigable=False):
        observation = self.client.observe()
        pose = observation['pose2d']
        result = {'x': number(pose.get('x'), 'x'), 'y': number(pose.get('y'), 'y'),
                  'yaw': number(pose.get('theta'), 'yaw')}
        state = observation['state']
        if state.get('agvPosition', {}).get('positionInitialized') is not True:
            raise RuntimeError('map_localization_not_initialized')
        if navigable:
            agv = observation['agvState']
            if agv.get('isMapping') is not False or agv.get('runningMode') != 'AUTO':
                raise RuntimeError('navigation_requires_localized_auto_mode')
            if (agv.get('softwareEstop') is not False or agv.get('estopButtonEvent') is not False
                    or agv.get('bumperEvent') is not False or agv.get('driverError') is not False):
                raise RuntimeError('chassis_stop_or_fault_is_active')
            if observation['localizationStatus'].get('cartoStatusId') != 1:
                raise RuntimeError('cartographer_localization_not_ready')
        if navigable and state['agvPosition'].get('mapId') != self._identity()[0]:
            raise RuntimeError('pose_map_identity_mismatch')
        return result, observation

    def dispatch(self, action, args):
        try:
            self._restore()
            if action == 'start':
                return self.start_chassis()
            if action == 'info':
                mapping = self.store.state('mapping')
                startup = self.startup_status()
                return {'state': startup['state'] if startup['state'] in ('loading', 'error') else 'mapping' if mapping else 'ready', 'mapping': mapping,
                        'remote_task': self._task, 'command_delivery_uncertain': self._uncertain,
                        'map_transition': self._map_pending, 'chassis_preparation': self._preparation,
                        'startup': self.startup_status(),
                        'map': self.client.current_map(), 'feedback': self.client.observe()}
            if action in ('stop_nav', 'stop'):
                result = self.bundle.pause('navigation_stopped')
                return result if action == 'stop_nav' or result.get('error') else {'state': 'idle'}
            if action not in self.ACTIONS:
                raise ValueError('unknown_controlled_spatial_action')
            if not self._operations.acquire(blocking=False):
                raise RuntimeError('spatial_operation_busy')
            try:
                if action in ('navigate_to_pose', 'navigate_to_tag'):
                    return self._navigate(action, args)
                return self._map_action(action, args)
            finally:
                self._operations.release()
        except Exception as exc:
            # Do not echo ROS peer errors, remote URLs or arbitrary request text.
            return {'state': 'error', 'error': str(exc)[:300].encode('unicode_escape').decode('ascii')[:400]}

    def _navigate(self, action, args):
        if self.store.state('mapping'):
            raise RuntimeError('stop_mapping_before_navigation')
        identity = self._identity()
        if action == 'navigate_to_tag':
            name = text(args.get('tag_name'), 'tag_name')
            matches = [p for p in self.store.tags(identity) if p['name'] == name]
            if not matches:
                raise ValueError('tag_not_found_on_current_map')
            target = {k: matches[0][k] for k in ('x', 'y', 'yaw')}
        else:
            target = {k: number(args.get(k), k) for k in ('x', 'y', 'yaw')}
        speed = number(args.get('speed', .15), 'speed', .05, .3)

        def validate():
            if self._uncertain or self._task is not None:
                raise RuntimeError('previous_native_navigation_is_unresolved')

        # All potentially blocking IO is outside the publication/tracker lock.
        epoch = self.bundle._stop_epoch
        self.client.native('probe')
        self._observation(navigable=True)
        if self._identity() != identity:
            raise RuntimeError('map_changed_during_navigation_preflight')
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            'base', self.PREFIX, (2, 3), lambda: None, validate, request_epoch=epoch)
        # The worker owns the external service, including any late response.
        record = {'action_id': aid, 'task_id': None, 'identity': identity, 'target': target}
        with self.bundle.tracker.lock:
            if cancelled.is_set():
                return {'state': 'error', 'error': 'navigation_superseded_by_stop'}
            self._task = record
            self._persist()

        def worker():
            try:
                if self._identity() != identity:
                    raise RuntimeError('map_changed_before_native_submit')
                self._observation(navigable=True)
                with self._native_lock:
                    if cancelled.is_set():
                        return
                    self._uncertain = True
                    self._persist()
                    response = self.client.native('navigate', **target, speed=speed)
                    if response.get('success') is False:
                        self._uncertain = False
                        self._task = None
                        self._persist()
                        raise RuntimeError('native_navigation_goal_rejected')
                    task_id = response.get('task_id')
                    if response.get('success') is not True or not isinstance(task_id, str) or not task_id or len(task_id) > 160:
                        raise RuntimeError('native_navigation_task_id_missing')
                    record['task_id'] = task_id
                    self._uncertain = False
                    self._persist()
                while not cancelled.wait(.2):
                    with self.bundle.tracker.lock:
                        if not self.bundle.tracker.current('base', aid):
                            return
                        self.bundle.state.require_motion((2, 3))
                        if self.bundle.ros1.connection != (True, generation):
                            raise RuntimeError('navigation_control_connection_changed')
                    if time.monotonic() - issued >= self.NAV_TIMEOUT:
                        raise TimeoutError('navigation_timeout')
                    if self._identity() != identity:
                        raise RuntimeError('map_identity_changed_during_navigation')
                    self._observation(navigable=True)
                    if cancelled.is_set():
                        return
                    # Status reads cannot mutate the controller. Do not hold
                    # the submit/cancel ordering lock across a slow status RPC.
                    status = self.client.native('check', task_id=task_id)
                    if status.get('success') is not True:
                        raise RuntimeError('task_status_query_failed')
                    code = status.get('status')
                    if type(code) is not int or code not in (0, 1, 2, 3):
                        raise RuntimeError('unknown_native_navigation_status')
                    if code == 3:
                        raise RuntimeError('native_navigation_failed')
                    if code == 2 and status.get('arrived') is True:
                        pose, observation = self._observation(navigable=True)
                        velocity = observation['state'].get('velocity', {})
                        if any(abs(number(velocity.get(k), k)) > .01 for k in ('vx', 'vy', 'omega')):
                            continue
                        error = math.hypot(pose['x'] - target['x'], pose['y'] - target['y'])
                        yaw_error = abs(math.atan2(math.sin(pose['yaw'] - target['yaw']),
                                                   math.cos(pose['yaw'] - target['yaw'])))
                        if error > .15 or yaw_error > .15:
                            raise RuntimeError('native_arrival_pose_mismatch')
                        with self.bundle.tracker.lock:
                            if not cancelled.is_set() and self.bundle.tracker.current('base', aid):
                                self._task = None
                                self._persist()
                                self.bundle.tracker.finish('base', aid, self.PREFIX, 'completed',
                                    {'task_id': task_id, 'map': identity[0], 'position_error_m': error,
                                     'completion_basis': 'native_task_id_arrival_and_map_pose'})
                        return
            except Exception:
                self.bundle.pause('native_navigation_failed_or_unconfirmed', status='error', expected=('base', aid))
            finally:
                self._workers.discard(threading.current_thread())

        thread = threading.Thread(target=worker, daemon=True, name='kuavo-native-navigation')
        self._workers.add(thread)
        thread.start()
        return {'state': 'executing', 'action_id': aid, 'target': target, 'map': identity[0],
                'control_preparation': preparation, 'completion_basis': 'native_task_id_arrival_and_map_pose'}

    def cancel_remote(self):
        """Called outside tracker.lock, while bundle pause owns preparation.

        Serializes CANCEL after an already-executing SUBMIT response. A timed
        out SUBMIT is unresolved even if a newer cancel is acknowledged.
        """
        self._restore()
        if self._map_pending is not None:
            raise RuntimeError('map_transition_unconfirmed_inspect_info')
        with self._native_lock:
            if self._preparation is not None:
                uncertain = self._preparation['delivery_uncertain']
                response = self.client.native('soft_stop')
                if (response.get('success') is not True or response.get('stationary') is not True
                        or response.get('software_estop') is not True):
                    raise RuntimeError('chassis_soft_stop_unconfirmed')
                if uncertain:
                    raise RuntimeError('older_chassis_release_unresolved')
                self._preparation = None
                self._persist()
            if self._task is None and not self._uncertain:
                return
            uncertain = self._uncertain
            response = self.client.native('cancel')
            if response.get('success') is not True or response.get('stationary') is not True:
                raise RuntimeError('native_navigation_stop_unconfirmed')
            if uncertain:
                raise RuntimeError('older_navigation_submit_unresolved_stop_cannot_be_confirmed')
            self._task = None
            self._persist()

    def _map_action(self, action, args):
        epoch = self.bundle._stop_epoch
        if action == 'list_maps':
            return {'state': 'ready', 'maps': self.client.maps(), 'current_map': self.client.current_map()}
        mapping = self.store.state('mapping')
        if action in ('list_tags', 'tag_place', 'untag_place'):
            identity = (mapping['name'], mapping['token']) if mapping else self._identity()
            if action == 'tag_place':
                if self._map_pending is not None or (mapping and mapping['phase'] != 'mapping'):
                    raise RuntimeError('map_transition_in_progress_or_unconfirmed')
                name = text(args.get('name'), 'name')
                description = args.get('description', '')
                if not isinstance(description, str) or len(description) > 512:
                    raise ValueError('invalid_description')
                pose, _ = self._observation()
                if not mapping and self._identity() != identity:
                    raise RuntimeError('map_changed_while_tagging')
                self.store.tag(identity, name, description, pose)
                return {'state': 'completed', 'name': name, 'map': identity[0], 'pose': pose}
            if action == 'untag_place':
                return {'state': 'completed', 'removed': self.store.untag(identity, text(args.get('name'), 'name'))}
            tags = self.store.tags(identity)
            pose, _ = self._observation()
            for tag in tags:
                tag['distance'] = math.hypot(tag['x'] - pose['x'], tag['y'] - pose['y'])
                tag['bearing'] = math.atan2(tag['y'] - pose['y'], tag['x'] - pose['x']) - pose['yaw']
            return {'state': 'ready', 'map': identity[0], 'tags': tags}

        # Reject invalid requests before reserving/persisting an external mutation.
        if action == 'start_mapping':
            text(args.get('map_name'), 'map_name')
            if mapping:
                raise RuntimeError('mapping_session_already_exists')
        elif action == 'stop_mapping':
            if not mapping or mapping['phase'] != 'mapping':
                raise RuntimeError('no_confirmed_mapping_session_inspect_info')
        elif action == 'load_map':
            text(args.get('map_name'), 'map_name')
            if mapping:
                raise RuntimeError('finish_or_reconcile_mapping_before_loading')
        else:
            raise ValueError('unsupported_map_action')
        if action in ('start_mapping', 'load_map'):
            exists = args['map_name'] in [Path(m.get('fileName', '')).stem for m in self.client.maps()]
            if exists == (action == 'start_mapping'):
                raise ValueError('map_name_already_exists' if exists else 'grid_map_not_found')

        self._require_stationary()

        # Map mutations cannot overlap chassis driving or another preparation.
        with self.bundle.tracker.lock:
            if epoch != self.bundle._stop_epoch:
                raise RuntimeError('map_request_superseded_by_stop')
            self.bundle.state.require_motion_enabled()
            if (self.bundle._transition is not None or 'base' in self.bundle.tracker._active
                    or self.unresolved()):
                raise RuntimeError('base_busy_or_navigation_unresolved')
            aid, cancelled = self.bundle.tracker.begin('base', self.PREFIX)
            self._map_pending = {'operation': action, 'action_id': aid}
            self._persist()
        self._map_deadline = time.monotonic() + 150
        def worker():
            try:
                result = self._mutate_map(action, args, mapping, cancelled)
                if cancelled.is_set():
                    raise RuntimeError('map_operation_interrupted')
                self._map_pending = None
                self._persist()
                self.bundle.tracker.finish('base', aid, self.PREFIX, 'completed', result)
            except Exception:
                self.bundle.tracker.finish('base', aid, self.PREFIX, 'error',
                    {'reason': 'map_transition_unconfirmed_inspect_info', 'map_state': self.store.state('mapping')})
            finally:
                self._workers.discard(threading.current_thread())
        thread = threading.Thread(target=worker, daemon=True, name='kuavo-map-operation')
        self._workers.add(thread)
        thread.start()
        return {'state': 'executing', 'action_id': aid, 'operation': action}

    def _require_stationary(self):
        state = self.client.observe()['state']
        if state.get('driving') is not False or any(
                abs(number(state.get('velocity', {}).get(k), k)) > .01
                for k in ('vx', 'vy', 'omega')):
            raise RuntimeError('stop_chassis_before_changing_map')

    def _mutate_map(self, action, args, mapping, cancelled):
        self._require_stationary()
        if action == 'start_mapping':
            if mapping:
                raise RuntimeError('mapping_session_already_exists')
            name = text(args.get('map_name'), 'map_name')
            if name in [Path(m.get('fileName', '')).stem for m in self.client.maps()]:
                raise ValueError('map_name_already_exists')
            mapping = {'name': name, 'token': 'mapping-' + uuid4().hex, 'phase': 'starting'}
            self.store.state('mapping', mapping)
            self._command('ChangeMode', {'mode': 'MANUAL'}, cancelled)
            self._command('StartCartoMapping', {}, cancelled)
            self._wait(lambda: self.client.observe()['agvState'].get('isMapping') is True, cancelled)
            mapping['phase'] = 'mapping'
            self.store.state('mapping', mapping)
            return {'state': 'mapping', 'map_name': name}
        if action == 'stop_mapping':
            if not mapping or mapping['phase'] != 'mapping':
                raise RuntimeError('no_confirmed_mapping_session_inspect_info')
            mapping['phase'] = 'saving'
            self.store.state('mapping', mapping)
            self._command('ChangeMode', {'mode': 'MANUAL'}, cancelled)
            self._command('SaveCartoMap', {'mapName': mapping['name'], 'savePath': True}, cancelled)
            self._wait(lambda: mapping['name'] in [Path(m.get('fileName', '')).stem
                                                   for m in self.client.maps()], cancelled)
            self._command('AgvSystemRestart', {}, cancelled)
            self._wait(lambda: self.client.observe()['agvState'].get('isMapping') is False, cancelled, 60)
            self._load(mapping['name'], cancelled)
            new_identity = self._identity()
            self.store.bind_mapping((mapping['name'], mapping['token']), new_identity)
            self.store.state('mapping', None)
            return {'state': 'completed', 'map_name': new_identity[0], 'map_fingerprint': new_identity[1]}
        if action == 'load_map':
            if mapping:
                raise RuntimeError('finish_or_reconcile_mapping_before_loading')
            name = text(args.get('map_name'), 'map_name')
            if name not in [Path(m.get('fileName', '')).stem for m in self.client.maps()]:
                raise ValueError('grid_map_not_found')
            self._load(name, cancelled)
            return {'state': 'completed', 'map_name': name,
                    'localization_required': True, 'feedback': self.client.observe()}
        raise ValueError('unsupported_map_action')

    def _command(self, method, params, cancelled):
        if cancelled.is_set():
            raise RuntimeError('map_operation_interrupted')
        remaining = self._map_deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('map_operation_timeout')
        self.client.command(method, params, timeout=min(25, remaining))
        if cancelled.is_set():
            raise RuntimeError('map_operation_interrupted_inspect_map_state')

    def _load(self, name, cancelled):
        self._command('ChangeMode', {'mode': 'MANUAL'}, cancelled)
        self._command('ChangeCartoMap', {'mapName': name}, cancelled)
        self._command('ChangeRoadMap', {'mapName': name}, cancelled)
        self._wait(lambda: self._identity()[0] == name, cancelled)
        self._command('ChangeMode', {'mode': 'AUTO'}, cancelled)
        self._wait(lambda: self.client.observe()['agvState'].get('runningMode') == 'AUTO', cancelled)

    def _wait(self, predicate, cancelled, timeout=30):
        end = min(time.monotonic() + timeout, self._map_deadline)
        while not cancelled.is_set() and time.monotonic() < end:
            try:
                if predicate():
                    return
            except Exception:
                pass
            cancelled.wait(.5)
        raise RuntimeError('map_transition_not_confirmed_inspect_info')

    def close(self):
        for worker in list(self._workers):
            worker.join(timeout=1)
        if not any(worker.is_alive() for worker in self._workers):
            self.store.close()
