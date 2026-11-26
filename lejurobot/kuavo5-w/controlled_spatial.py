"""Controlled mapping, map-bound semantic tags and native Kuavo navigation."""
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
from uuid import UUID, uuid4

from spatial_client import JatenClient, MapCommandRejected, NavigationStateUnavailable
from map_storage import MapArchive, migrate_database, safe_name
from mapping_keyboard import MappingKeyboard


NAV_SPEED_MIN, NAV_SPEED_MAX, NAV_SPEED_DEFAULT = .05, .5, .15


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


def map_failure_code(exc):
    # Only image-owned validation codes; never echo arbitrary peer text.
    code = str(exc)
    return code if code in {'cannot_modify_active_map', 'grid_map_not_found',
        'finish_factory_mapping_before_deleting', 'invalid_map_delete_manifest',
        'invalid_map_delete_source', 'map_changed_during_delete_inspection',
        'chassis_map_reappeared_during_delete', 'map_delete_interrupted',
        'chassis_map_delete_not_confirmed', 'map_delete_not_confirmed'} else None


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

    def claim_boot_remote(self, boot_id):
        """Cross-process one-shot claim, durable before any startup control write."""
        with self.lock, self.db() as db:
            return db.execute('INSERT OR IGNORE INTO state VALUES (?,?)',
                ('boot_remote:' + boot_id, json.dumps({'consumed': True}))).rowcount == 1

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

    def delete_map(self, name):
        with self.lock, self.db() as db:
            db.execute('DELETE FROM tags WHERE map_id=?', (name,))
            prefix = 'mapping_anchor:' + json.dumps([name], ensure_ascii=False)[:-1] + ','
            for key_prefix in (prefix, 'unconfirmed:' + prefix):
                db.execute('DELETE FROM state WHERE substr(key, 1, ?) = ?', (len(key_prefix), key_prefix))
            row = db.execute("SELECT value FROM state WHERE key='loaded_map_identity'").fetchone()
            if row and (json.loads(row[0]) or [None])[0] == name:
                db.execute("DELETE FROM state WHERE key='loaded_map_identity'")

    def bind_mapping(self, previous, identity, start_pose=None):
        with self.lock:
            with self.db() as db:
                db.execute('UPDATE tags SET map_id=?,fingerprint=? WHERE map_id=? AND fingerprint=?',
                           (*identity, *previous))
                if start_pose is not None:
                    db.execute('INSERT OR REPLACE INTO state VALUES (?, ?)',
                               (self.anchor_key(identity), json.dumps(start_pose)))

    def retire_mapping(self, mapping, pending, evidence):
        """Audit and clear only the obsolete session, in one transaction."""
        with self.lock, self.db() as db:
            for key, expected in (('mapping', mapping), ('pending', pending)):
                row = db.execute('SELECT value FROM state WHERE key=?', (key,)).fetchone()
                if (json.loads(row[0]) if row else None) != expected:
                    raise RuntimeError('mapping_record_changed_during_reconciliation')
            updated = dict(pending, map_transition=None)
            for key, value in (('last_mapping_reconciliation', evidence), ('mapping', None), ('pending', updated)):
                db.execute('INSERT OR REPLACE INTO state VALUES (?,?)', (key, json.dumps(value)))

    @staticmethod
    def anchor_key(identity):
        return 'mapping_anchor:' + json.dumps(identity, ensure_ascii=False)

    def close(self):
        with self.lock:
            if self._db:
                self._db.close()
                self._db = None


class ControlledSpatialPlugin:
    PREFIX, CHANNEL = 'controlled_spatial', 'base'
    NAV_TIMEOUT = 150.
    NAV_HANDOFF_GRACE = 5.
    LOAD_LOCALIZATION_WAIT = 8.
    ACTIONS = {
        'start_mapping': (['map_name'], '开始厂商Cartographer建图；人工控制底盘采集'),
        'stop_mapping': ([], '保存当前建图，重启厂商地图服务并确认地图身份'),
        'reconcile_mapping': (['map_name', 'confirm_abandon'], '底盘重启且已退出建图后，确认放弃旧会话记录；保留地图文件和点位，不恢复运动'),
        'tag_place': (['name', 'description'], '记录当前位置和朝向，绑定地图身份'),
        'untag_place': (['name'], '删除当前地图中的语义点'),
        'list_tags': ([], '列出当前地图语义点及相对位置'),
        'list_maps': ([], '列出底盘真实栅格地图'),
        'get_pose': ([], '读取当前地图定位，x/y 为米，yaw 为弧度'),
        'delete_map': (['map_name', 'confirm_delete'], '永久删除指定的非活动地图、上位机副本和关联点位'),
        'load_map': (['map_name'], '在建图起点附近载入地图，以记录的起点辅助定位并核对结果'),
        'navigate_to_pose': (['x', 'y', 'yaw', 'speed'], '导航到map系位姿，按任务ID跟踪ACP'),
        'navigate_to_tag': (['tag_name', 'speed'], '导航到当前地图的语义点'),
        'stop_nav': ([], '原生取消当前导航并确认新鲜里程计停稳'),
        'mapping_console': ([], '≤0.15 m/s、0.3 rad/s；打开建图键盘面板，不使能、不移动'),
    }

    def __init__(self, bundle, config=None, client=None):
        self.bundle = bundle
        config = config or {}
        self._motion_music_enabled = config.get('motion_music_enabled', False)
        if type(self._motion_music_enabled) is not bool:
            raise ValueError('motion_music_enabled must be a boolean')
        self._standby_voice_enabled = config.get('standby_voice_enabled', False)
        if type(self._standby_voice_enabled) is not bool:
            raise ValueError('standby_voice_enabled must be a boolean')
        self.client = client or JatenClient(bundle.startup_checks.provisioner)
        legacy_db = Path(os.environ.get('KUAVO_STATE_DIR', '/config')) / 'controlled_spatial.db'
        data_dir = os.environ.get('KUAVO_DATA_DIR')
        default_db = Path(data_dir) / 'kuavo5w/controlled_spatial.db' if data_dir else legacy_db
        db_path = Path(config.get('db_path', str(default_db)))
        if data_dir and 'db_path' not in config:
            migrate_database(legacy_db, db_path)
        self.store = SpatialStore(db_path)
        self._operations = threading.Lock()
        self._native_lock = threading.RLock()
        self._start_lock = threading.Lock()
        self._startup_guard = threading.Lock()
        self._startup = None
        self._startup_key = None
        self._startup_thread = None
        self._music_status = {'state': 'idle'}
        self._standby_status = {'state': 'idle'}
        self._music_thread = None
        self._closed = threading.Event()
        self.archive = (MapArchive(self.client, config.get('map_archive_dir',
                        str(Path(data_dir) / 'maps/kuavo5w')), self._closed,
                        idle=lambda: not self._map_pending and not self.store.state('mapping'))
                        if data_dir else None)
        self._preparation = None
        self._task = None
        self._uncertain = False
        self._restored = False
        self._map_pending = None
        self._workers = set()
        self._map_deadline = float('inf')
        self._keyboard_pending = False
        self._mapping_cleanup = None
        self._mapping_cleanup_key = None
        self._mapping_cleanup_retry_at = 0.
        self._mapping_console_grant = None
        self._preparation_check_at = 0.
        self.base_preparation_required = False
        self.keyboard = MappingKeyboard(self)

    def get_tool(self):
        keyboard_actions = {
            'keyboard_open': ([], '准备MANUAL建图键盘控制，首帧必须为零速度'),
            'keyboard_command': (['session_id', 'seq', 'values'], '≤0.15 m/s、0.3 rad/s；发送带会话和递增序号的键盘速度心跳'),
            'keyboard_release': (['session_id'], '释放指定键盘会话并确认底盘停止'),
            'keyboard_status': ([], '读取键盘会话、控制权和停止确认状态'),
        }
        actions = {**self.ACTIONS, **keyboard_actions}
        props = {'action': {'type': 'string', 'enum': list(actions)},
                 'session_id': {'type': 'string'}, 'seq': {'type': 'integer', 'minimum': 0},
                 'values': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 2, 'maxItems': 2,
                            'description': '≤0.15 m/s、0.3 rad/s；面板内部使用[vx,vw]，无侧移'},
                 **{k: {'type': 'string'} for k in ('map_name', 'name', 'description', 'tag_name')},
                 **{k: {'type': 'number', 'description': 'map坐标系位置，米'} for k in ('x', 'y')},
                 'confirm_delete': {'type': 'boolean', 'description': '确认永久删除地图及关联点位'},
                 'confirm_abandon': {'type': 'boolean', 'description': '确认放弃指定旧建图会话；不保存、不删除地图文件和点位'},
                 'yaw': {'type': 'number', 'description': 'map坐标系绝对朝向，弧度'},
                 'speed': {'type': 'number',
                           'description': f'{NAV_SPEED_MIN:g}–{NAV_SPEED_MAX:g} m/s；导航线速度上限，默认{NAV_SPEED_DEFAULT:g} m/s，实际速度由厂商规划及安全限速决定',
                           'minimum': NAV_SPEED_MIN, 'maximum': NAV_SPEED_MAX, 'default': NAV_SPEED_DEFAULT}}
        required = {'start_mapping': ['map_name'], 'tag_place': ['name'], 'untag_place': ['name'],
                    'load_map': ['map_name'], 'delete_map': ['map_name', 'confirm_delete'],
                    'reconcile_mapping': ['map_name', 'confirm_abandon'],
                    'keyboard_command': ['session_id', 'seq', 'values'],
                    'keyboard_release': ['session_id'],
                    'navigate_to_pose': ['x', 'y', 'yaw'],
                    'navigate_to_tag': ['tag_name']}
        return {'name': self.PREFIX, 'type': 'actuator', 'multiInstance': False,
                'description': f'{NAV_SPEED_MIN:g}–{NAV_SPEED_MAX:g} m/s；Kuavo 5-W人工建图、打点与厂商底盘原生导航；导航独占底盘速度，保持全身BaseArm，可并行控制手臂/头部/下肢。',
                'inputSchema': {'type': 'object', 'properties': props, 'required': ['action'],
                    'x-workbench': {'protocol': 'motus.workbench/1', 'kind': 'mapping-control',
                        'title': 'Mapping keyboard control', 'entry': 'mapping_console',
                        'actions': ['info', 'stop', *keyboard_actions]},
                    'x-resource': 'base', 'x-hooks': {'on_interrupt_motion': {'action': 'stop_nav'}},
                    'x-completion': {'actions': ['navigate_to_pose', 'navigate_to_tag', 'start_mapping', 'stop_mapping', 'load_map', 'delete_map'], 'timeout': 1830},
                    # Keyboard transport is workbench-only, like G1 teaching
                    # authoring: keep it out of canvas/LLM action forms.
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
                self._keyboard_pending = bool(pending.get('keyboard_stop_unconfirmed'))
            self._restored = True

    def _persist(self):
        self.store.state('pending', {'navigation': self._task, 'uncertain': self._uncertain,
                                    'map_transition': self._map_pending, 'chassis_preparation': self._preparation,
                                    'keyboard_stop_unconfirmed': self._keyboard_pending})

    def unresolved(self):
        self._restore()
        return (self._uncertain or self._task is not None or self._map_pending is not None
                or self._preparation is not None or self._keyboard_pending)

    def mapping_active(self):
        self._restore()
        return self.store.path.exists() and self.store.state('mapping') is not None

    def chassis_start_in_progress(self):
        """Caller holds tracker.lock: a live shared start can be joined, not owned.

        Persisted/orphan preparations and uncertain navigation/map writes still
        block control. A stopped or reconnected startup cannot be joined.
        """
        self._restore()
        token = self.bundle._transition
        with self._startup_guard:
            return bool(self._startup_thread and self._startup_thread.is_alive()
                and self._startup_key == (self.bundle._stop_epoch, self.bundle.ros1.connection)
                and self._startup and self._startup['state'] == 'loading'
                and (token is None or (token['action'] == 'prepare_chassis'
                                      and not token['cancelled'].is_set()))
                and not self._uncertain and self._task is None and self._map_pending is None)

    def motion_block_reason(self):
        if self._keyboard_pending:
            return 'mapping keyboard owns the base or its stop is unconfirmed; close keyboard and confirm stop_nav'
        if self.mapping_active():
            return 'manual mapping owns the chassis; finish mapping before another base controller'
        if not self.unresolved():
            return None
        if self._map_pending is not None:
            return 'map transition is active or unconfirmed; inspect controlled_spatial.info; stop_nav does not finish map saving'
        if self._preparation is not None:
            return 'chassis preparation is unconfirmed; waiting for held stationary chassis feedback'
        return 'native navigation is active or unresolved; stop_nav must be confirmed first'

    def allows_independent_motion(self, channel):
        """A confirmed, tracked base task does not own arms/head/lower body.

        Persisted orphan tasks, uncertain submissions and map/control transitions
        remain blocked. Never reinterpret those as a healthy parallel action.
        Caller holds tracker.lock, so a stop cannot invalidate this check midway.
        """
        mapping = self.store.state('mapping') if self.store.path.exists() else None
        if mapping and mapping.get('phase') == 'mapping':
            return (channel != 'base' and not self._uncertain and self._task is None
                    and self._map_pending is None and self._preparation is None
                    and (not self._keyboard_pending or self.keyboard.session is not None))
        task = self._task
        return (channel != 'base' and task is not None and bool(task.get('task_id'))
                and not self._uncertain and self._map_pending is None and self._preparation is None
                and self.bundle.tracker.current('base', task['action_id']))

    @staticmethod
    def _check_chassis_faults(observation):
        agv = observation['agvState']
        if any(agv.get(k) is not False for k in ('estopButtonEvent', 'bumperEvent', 'driverError')):
            raise RuntimeError('chassis_hardware_stop_or_fault_is_active')

    def preserve_upper_route(self):
        """Lifecycle attachment must not stop an already-ready G12/upper route."""
        state = self.bundle.state
        boot = state.snapshot().get('controller_boot')
        if not boot or boot.get('state') != 'complete' or boot['age_s'] > 3.:
            return False
        try:
            enabled = state.confirmed_enable()
        except RuntimeError:
            return False
        mode, seen = state.read('mpc_mode')
        return (enabled is True and mode == 3 and time.monotonic()-seen <= 2.
                and state.read('upper_velocity_enabled')[0] is True)

    def boot_remote_pending(self):
        remote = getattr(self.bundle, 'boot_remote', None)
        return (remote is not None and remote.enabled and not remote.cancelled
                and remote.status['state'] in ('waiting', 'preparing'))

    def start_chassis(self):
        """Share one background preflight across both base cards and a stop epoch."""
        if not self.bundle.ros1.connection[0]:
            return {'state': 'error', 'error': 'control_connection_is_disconnected'}
        with self.bundle.tracker.lock:
            if self.boot_remote_pending():
                return {'state': 'ready', 'preparation_deferred': True,
                        'message': 'First-boot remote preparation is pending; card startup will not take chassis control.'}
            if self.mapping_active() or self._keyboard_pending:
                return {'state': 'ready', 'preparation_deferred': True,
                        'active_base_controller': 'mapping_keyboard',
                        'message': 'Manual mapping owns the chassis; startup will not change its mode or enable state.'}
            active = self.bundle.tracker._active.get('base')
            if active is not None:
                return {'state': 'running', 'active_base_controller': active[1],
                        'message': 'Base control is already reserved; keeping its current controller.'}
            if not self.unresolved() and self.preserve_upper_route():
                return {'state': 'ready', 'preparation_deferred': True,
                        'active_base_controller': 'existing_upper_control',
                        'message': 'Upper control is already enabled; startup preserves the current remote-control route.'}
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
                    result = self.prepare_chassis(request_epoch=key[0])
                    if key != (self.bundle._stop_epoch, self.bundle.ros1.connection):
                        raise RuntimeError('chassis_start_superseded_by_stop_or_reconnect')
                    status = {'state': 'running', 'chassis_preparation': result}
                except Exception as exc:
                    status = {'state': 'error', 'error': str(exc).encode('unicode_escape').decode('ascii')[:400]}
                with self._startup_guard:
                    self._startup = status
            self._startup_thread = threading.Thread(target=worker, daemon=True, name='chassis-start')
            self._workers.add(self._startup_thread)
            self._startup_thread.start()
            return dict(self._startup)

    def start_background(self):
        """Configure factory movement/standby tracks without starting chassis control."""
        if self.archive is not None:
            self.archive.schedule()
        with self._startup_guard:
            if self._music_thread is not None or self._closed.is_set():
                return
            provisioner = self.bundle.startup_checks.provisioner
            if not provisioner.enabled or provisioner._target('chassis').get('enabled', True) is False:
                self._music_status = {'state': 'skipped', 'reason': 'automatic_chassis_access_disabled'}
                self._standby_status = dict(self._music_status)
                return
            self._music_status = {'state': 'loading'}
            self._standby_status = {'state': 'loading'}
            def worker():
                pending = {'motion_music': (self._motion_music_enabled, '_music_status'),
                           'standby_voice': (self._standby_voice_enabled, '_standby_status')}
                while pending and not self._closed.is_set():
                    for operation, (enabled, attribute) in list(pending.items()):
                        if self._closed.is_set():
                            return
                        try:
                            status = self.client.native(operation, enabled=enabled)
                        except Exception:
                            status = {'state': 'unavailable', 'retry_in_s': 30}
                        with self._startup_guard:
                            setattr(self, attribute, status)
                        if status.get('state') in ('enabled', 'disabled', 'unsupported', 'blocked'):
                            pending.pop(operation)
                    if pending and self._closed.wait(30):
                        return
            self._music_thread = threading.Thread(target=worker, daemon=True, name='motion-music-start')
            self._workers.add(self._music_thread)
            self._music_thread.start()

    def startup_status(self):
        with self._startup_guard:
            status = (dict(self._startup or {'state': 'idle'})
                      if self._startup_key == (self.bundle._stop_epoch, self.bundle.ros1.connection)
                      else {'state': 'idle'})
            status['motion_music'] = dict(self._music_status)
            status['standby_voice'] = dict(self._standby_status)
            return status

    def _startup_observation(self, epoch):
        """Retry reads only; never resubmit prepare/release on a missing frame."""
        for attempt in range(3):
            if self._closed.is_set() or epoch != self.bundle._stop_epoch:
                raise RuntimeError('chassis_start_superseded_by_stop')
            try:
                result = self.client.observe(required=('agvState', 'state'))
                if self._closed.is_set() or epoch != self.bundle._stop_epoch:
                    raise RuntimeError('chassis_start_superseded_by_stop')
                return result
            except NavigationStateUnavailable as exc:
                if attempt == 2:
                    raise
                with self._startup_guard:
                    self._startup = {'state': 'loading',
                        'message': 'Waiting for fresh chassis status, retrying in background. Please wait.',
                        'state_read_attempt': attempt + 1, 'missing_state': list(exc.missing)}
                if self._closed.wait(.5):
                    raise RuntimeError('driver_is_shutting_down')

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
            if self.mapping_active() or self._keyboard_pending:
                raise RuntimeError('manual_mapping_owns_chassis_no_startup_preparation')
            self._settle_held_preparation()
            if self.unresolved():
                raise RuntimeError('previous_chassis_operation_unresolved_stop_first')
            if not self.bundle.ros1.connection[0]:
                raise RuntimeError('control_connection_is_disconnected')
            self._check_chassis_faults(self._startup_observation(epoch))
            with self.bundle.tracker.lock:
                if epoch != self.bundle._stop_epoch:
                    raise RuntimeError('chassis_start_superseded_by_stop')
                token, _, was_blocked = self.bundle._begin_control('prepare_chassis')
                self.bundle._upper_route_key = None
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
                             'hold_software_stop', 'pause_old_tracker',
                             'release_software_stop', 'confirm_release_state', 'confirm_release_stationary',
                             'validate_runtime'}
                    raise RuntimeError('chassis_preparation_failed_at_' + stage if stage in known
                                       else 'chassis_preparation_not_confirmed')
                self._preparation = {'delivery_uncertain': False}
                self._persist()
            observation = self._startup_observation(epoch)
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
                self.base_preparation_required = False
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

    def _observation(self, navigable=False, identity=None):
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
                raise RuntimeError('navigation_requires_localized_auto_mode: isMapping='
                                   + str(agv.get('isMapping')) + ', runningMode=' + str(agv.get('runningMode')))
            if (agv.get('softwareEstop') is not False or agv.get('estopButtonEvent') is not False
                    or agv.get('bumperEvent') is not False or agv.get('driverError') is not False):
                flags = ('softwareEstop', 'estopButtonEvent', 'bumperEvent', 'driverError')
                raise RuntimeError('chassis_stop_or_fault_is_active: ' + ','.join(k for k in flags if agv.get(k) is not False))
            if observation['localizationStatus'].get('cartoStatusId') != 1:
                raise RuntimeError('cartographer_localization_not_ready')
        if navigable and state['agvPosition'].get('mapId') != (identity or self._identity())[0]:
            raise RuntimeError('pose_map_identity_mismatch')
        return result, observation

    def preview_pose_updates(self, active):
        self._restore()
        if self._map_pending is not None:
            return
        identity = self._identity()
        for observation, stamp in self.client.pose_observations(active):
            if self._map_pending is not None:
                return
            position = observation['state'].get('agvPosition', {})
            if (position.get('mapId') != identity[0]
                    or position.get('positionInitialized') is not True
                    or observation['agvState'].get('isMapping') is not False
                    or observation['localizationStatus'].get('cartoStatusId') != 1):
                yield None
                continue
            raw = observation['pose2d']
            try:
                pose = {axis: number(raw.get(key), axis)
                        for axis, key in (('x', 'x'), ('y', 'y'), ('yaw', 'theta'))}
            except ValueError:
                yield None
                continue
            yield identity, pose, stamp

    def get_pose(self):
        self._restore()
        if self._map_pending is not None:
            raise RuntimeError('map_transition_in_progress_or_unconfirmed')
        identity = self._identity()
        pose, observation = self._observation()
        received = time.monotonic()
        localization = observation['localizationStatus']
        if observation['agvState'].get('isMapping') is not False:
            raise RuntimeError('map_localization_unavailable_while_mapping')
        if localization.get('cartoStatusId') != 1:
            raise RuntimeError('cartographer_localization_not_ready')
        position = observation['state']['agvPosition']
        current = self.client.current_map()
        if (position.get('mapId') != identity[0] or current['mapId'] != identity[0]
                or current['mapMd5'] != identity[1].partition(':')[2]):
            raise RuntimeError('pose_map_identity_mismatch')
        if self._map_pending is not None or time.monotonic() - received > 1.:
            raise RuntimeError('map_pose_no_longer_fresh')
        score = localization.get('score')
        return {'state': 'ready', 'map': identity[0], 'map_fingerprint': identity[1],
                'frame': 'map', 'pose': pose, 'units': {'x': 'm', 'y': 'm', 'yaw': 'rad'},
                'localization_score': score if type(score) in (float, int) and math.isfinite(score) else None}

    def dispatch(self, action, args):
        try:
            self._restore()
            if action in ('info', 'start', 'start_mapping', 'load_map', 'delete_map'):
                self._settle_held_preparation()
            if action in ('start', 'info', 'list_maps', 'get_pose', 'start_mapping', 'stop_mapping',
                          'load_map', 'delete_map', 'tag_place', 'untag_place', 'list_tags',
                          'navigate_to_pose', 'navigate_to_tag'):
                cleanup = self._auto_reconcile_mapping()
                if action == 'stop_mapping' and cleanup is not None:
                    return {**cleanup, 'completion_basis': 'obsolete_mapping_session_cleared',
                            'map_saved': False,
                            'message': 'Obsolete mapping session cleared automatically. No new save was requested.'}
            if action == 'mapping_console':
                with self.bundle.tracker.lock:
                    mapping = self.store.state('mapping') or {}
                    self._mapping_console_grant = ((mapping.get('token'), self.bundle._stop_epoch,
                        self.bundle.ros1.connection) if mapping.get('phase') == 'mapping'
                        and not self._operations.locked() else None)
                    pending = self._map_pending or {}
                    if (pending.get('operation') == 'start_mapping' and self._map_worker_active()):
                        self._mapping_console_grant = ('starting', pending['action_id'],
                            self.bundle._stop_epoch, self.bundle.ros1.connection)
                return {'protocol': 'motus.workbench/1',
                        'html': (Path(__file__).parent/'resource/mapping_keyboard.html').read_text()}
            if action == 'keyboard_status':
                return {**self.keyboard.info(), **self.mapping_console_status()}
            if action == 'keyboard_open':
                return self.keyboard.open()
            if action == 'keyboard_command':
                return self.keyboard.command(args)
            if action == 'keyboard_release':
                return self.keyboard.stop(text(args.get('session_id'), 'session_id'))
            if action == 'start':
                return self.start_chassis()
            if action == 'info':
                mapping = self.store.state('mapping')
                startup = self.startup_status()
                result = {'state': startup['state'] if startup['state'] in ('loading', 'error') else 'transition' if self._map_pending else 'mapping' if mapping else 'ready', 'mapping': mapping,
                          'remote_task': self._task, 'command_delivery_uncertain': self._uncertain,
                          'map_transition': self._map_pending, 'chassis_preparation': self._preparation,
                          'startup': startup}
                result['control_ownership'] = self.bundle.control_ownership()
                result['keyboard'] = self.keyboard.info()
                result['busy'] = result['keyboard']['busy'] or self._keyboard_pending
                result['storage'] = {'database': str(self.store.path),
                    'factory_maps': '/home/jt/jmap_config/jmaps',
                    'archive': self.archive.snapshot() if self.archive else {'state': 'disabled'}}
                result['last_map_operation'] = self.store.state('last_map_operation')
                if self._map_pending and self._map_pending.get('stage_started_at'):
                    result['map_stage_elapsed_s'] = max(0., time.time()-self._map_pending['stage_started_at'])
                if mapping and mapping.get('phase') == 'saving':
                    result['state'] = 'saving'
                    result['message'] = 'Optimizing and saving the map in background. Please wait; do not restart mapping.'
                recovery = self._mapping_recovery()
                if recovery:
                    result['state'] = 'loading'
                    result['message'] = 'Checking the previous mapping operation automatically; waiting for confirmed chassis state.'
                if self._mapping_cleanup is not None:
                    result['mapping_cleanup'] = self._mapping_cleanup
                result['last_mapping_reconciliation'] = self.store.state('last_mapping_reconciliation')
                try:
                    result['map'] = self.client.current_map()
                except Exception:
                    result['map'] = None
                    result['map_status'] = {'state': 'unavailable'}
                try:
                    result['feedback'] = self.client.observe()
                    result['feedback_status'] = {'state': 'ready'}
                except NavigationStateUnavailable as exc:
                    result['feedback'] = None
                    result['feedback_status'] = {'state': 'loading', 'missing': list(exc.missing), 'reason': exc.reason}
                    if result['state'] not in ('error', 'saving'):
                        result['state'] = 'loading'
                    if result['state'] == 'loading':
                        result['message'] = 'Waiting for fresh chassis navigation feedback; checking in background, please wait.'
                return result
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
                if action == 'get_pose':
                    return self.get_pose()
                if action == 'reconcile_mapping':
                    return self._reconcile_mapping(args)
                return self._map_action(action, args)
            finally:
                self._operations.release()
        except Exception as exc:
            # Do not echo ROS peer errors, remote URLs or arbitrary request text.
            result = {'state': 'error', 'error': str(exc)[:300].encode('unicode_escape').decode('ascii')[:400]}
            if str(exc) in ('mapping_session_already_exists', 'no_confirmed_mapping_session_inspect_info',
                            'finish_or_reconcile_mapping_before_deleting', 'finish_or_reconcile_mapping_before_loading'):
                recovery = self._mapping_recovery()
                if recovery:
                    result.update(mapping_cleanup=self._mapping_cleanup or {'state': 'waiting'},
                        message='Checking the previous mapping operation automatically; waiting for confirmed chassis state.')
            return result

    def _navigate(self, action, args):
        if self.store.state('mapping'):
            raise RuntimeError('stop_mapping_before_navigation')
        identity = self._identity()
        if self.store.state('unconfirmed:' + self.store.anchor_key(identity)):
            raise RuntimeError('map_relocalization_unconfirmed_load_map_again')
        if action == 'navigate_to_tag':
            name = text(args.get('tag_name'), 'tag_name')
            matches = [p for p in self.store.tags(identity) if p['name'] == name]
            if not matches:
                raise ValueError('tag_not_found_on_current_map')
            target = {k: matches[0][k] for k in ('x', 'y', 'yaw')}
        else:
            target = {k: number(args.get(k), k) for k in ('x', 'y', 'yaw')}
        speed = number(args.get('speed', NAV_SPEED_DEFAULT), 'speed', NAV_SPEED_MIN, NAV_SPEED_MAX)

        def validate():
            if self._uncertain or self._task is not None:
                raise RuntimeError('previous_native_navigation_is_unresolved')

        # All potentially blocking IO is outside the publication/tracker lock.
        epoch = self.bundle._stop_epoch
        self.client.native('probe')
        self._observation(navigable=True, identity=identity)
        if self._identity() != identity:
            raise RuntimeError('map_changed_during_navigation_preflight')
        # Explicit discrete commands have priority over the continuous stream.
        # Latch the stream paused so subsequent packets cannot retake the base.
        self.bundle.base_servo.preempt()
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            'base', self.PREFIX, (3,), lambda: None, validate, request_epoch=epoch)
        # The worker owns the external service, including any late response.
        record = {'action_id': aid, 'task_id': None, 'identity': identity, 'target': target,
                  'submission_started': False}
        with self.bundle.tracker.lock:
            if cancelled.is_set():
                return {'state': 'error', 'error': 'navigation_superseded_by_stop'}
            self._task = record
            self._persist()

        def check_velocity_source(accepted_at):
            # The chassis publishes the selected source as a latched value
            # on changes. RobotState clears it when the ROS bridge reconnects.
            source, _ = self.bundle.state.read('base_velocity_source')
            if (source == 'nav_mpc_move_base'
                    and time.monotonic() - accepted_at >= self.NAV_HANDOFF_GRACE):
                raise RuntimeError('native_navigation_velocity_handoff_unconfirmed: '
                                   'chassis_selected_nav_mpc_move_base')

        def worker():
            submitted = False
            try:
                # A fresh socket can time out even while other snapshots are
                # healthy. Retry read-only preflight once, before any goal RPC.
                for attempt in range(2):
                    if cancelled.is_set():
                        return
                    try:
                        self._observation(navigable=True, identity=identity)
                        break
                    except NavigationStateUnavailable:
                        if attempt:
                            raise
                if self._identity() != identity:
                    raise RuntimeError('map_changed_before_native_submit')
                with self._native_lock:
                    if cancelled.is_set():
                        return
                    with self.bundle.tracker.lock:
                        if not self.bundle.tracker.current('base', aid):
                            return
                        self.bundle.require_base_control(self.PREFIX)
                        if self.bundle.ros1.connection != (True, generation):
                            raise RuntimeError('navigation_control_connection_changed')
                    self._uncertain = True
                    submitted = True
                    record['submission_started'] = True
                    self._persist()
                    response = self.client.native('navigate', **target, speed=speed)
                    if response.get('success') is False:
                        self._uncertain = bool(response.get('delivery_uncertain'))
                        if not self._uncertain and not response.get('goal_submitted'):
                            self._task = None
                        self._persist()
                        if response.get('stage') == 'dispatch_handoff':
                            raise RuntimeError('native_navigation_dispatch_handoff_rejected')
                        stage = response.get('stage')
                        if stage in ('hold_and_cancel', 'pause_old_tracker', 'mux_handoff',
                                     'goal_submit', 'confirm_new_path', 'resume_new_tracker', 'confirm_mux_handoff',
                                     'release_software_stop', 'validate_runtime'):
                            raise RuntimeError('native_navigation_failed_at_' + stage)
                        raise RuntimeError('native_navigation_goal_rejected')
                    task_id = response.get('task_id')
                    if response.get('success') is not True or not isinstance(task_id, str) or not task_id or len(task_id) > 160:
                        raise RuntimeError('native_navigation_task_id_missing')
                    accepted_at = time.monotonic()
                    record['task_id'] = task_id
                    self._uncertain = False
                    self._persist()
                while not cancelled.wait(.2):
                    with self.bundle.tracker.lock:
                        if not self.bundle.tracker.current('base', aid):
                            return
                        self.bundle.state.require_motion((3,))
                        if self.bundle.ros1.connection != (True, generation):
                            raise RuntimeError('navigation_control_connection_changed')
                        upper_velocity, _ = self.bundle.state.read('upper_velocity_enabled')
                        if upper_velocity is not False:
                            raise RuntimeError('upper_mpc_velocity_handoff_lost')
                        check_velocity_source(accepted_at)
                    if time.monotonic() - issued >= self.NAV_TIMEOUT:
                        raise TimeoutError('navigation_timeout')
                    if self._identity() != identity:
                        raise RuntimeError('map_identity_changed_during_navigation')
                    try:
                        self._observation(navigable=True, identity=identity)
                    except NavigationStateUnavailable:
                        check_velocity_source(accepted_at)
                        raise
                    if cancelled.is_set():
                        return
                    # Status reads cannot mutate the controller. Do not hold
                    # the submit/cancel ordering lock across a slow status RPC.
                    status = self.client.native('check', task_id=task_id)
                    if status.get('success') is not True:
                        raise RuntimeError('task_status_query_failed')
                    code = status.get('status')
                    record['native_status'] = code
                    record['native_arrived'] = status.get('arrived')
                    if type(code) is not int or code not in (0, 1, 2, 3):
                        raise RuntimeError('unknown_native_navigation_status')
                    if code == 3:
                        raise RuntimeError('native_navigation_failed')
                    if code == 2 and status.get('arrived') is True:
                        pose, observation = self._observation(navigable=True, identity=identity)
                        if observation['state'].get('driving') is not False:
                            continue
                        error = math.hypot(pose['x'] - target['x'], pose['y'] - target['y'])
                        yaw_error = abs(math.atan2(math.sin(pose['yaw'] - target['yaw']),
                                                   math.cos(pose['yaw'] - target['yaw'])))
                        if error > .15 or yaw_error > .15:
                            raise RuntimeError('native_arrival_pose_mismatch')
                        stationary = self.client.native('stationary')
                        if stationary.get('success') is not True or stationary.get('stationary') is not True:
                            continue
                        with self.bundle.tracker.lock:
                            if not cancelled.is_set() and self.bundle.tracker.current('base', aid):
                                self._task = None
                                self._persist()
                                self.bundle.tracker.finish('base', aid, self.PREFIX, 'completed',
                                    {'task_id': task_id, 'map': identity[0], 'position_error_m': error,
                                     'completion_basis': 'native_task_id_arrival_map_pose_and_fresh_stationary_odometry'})
                        return
            except Exception as exc:
                detail = str(exc)[:160].encode('unicode_escape').decode('ascii')[:200]
                if not submitted:
                    detail = 'preflight_failed_no_goal_sent: ' + detail
                self.bundle.pause('native_navigation_failed_or_unconfirmed: ' + detail,
                                  status='error', expected=('base', aid))
            finally:
                self._workers.discard(threading.current_thread())

        thread = threading.Thread(target=worker, daemon=True, name='kuavo-native-navigation')
        self._workers.add(thread)
        thread.start()
        return {'state': 'executing', 'action_id': aid, 'target': target, 'map': identity[0],
                'linear_limit_m_s': speed, 'control_preparation': preparation,
                'completion_basis': 'native_task_id_arrival_and_map_pose'}

    def cancel_remote(self):
        """Called outside tracker.lock, while bundle pause owns preparation.

        Serializes CANCEL after an already-executing SUBMIT response. A timed
        out SUBMIT is unresolved even if a newer cancel is acknowledged.
        """
        self._restore()
        self.keyboard.stop()
        if self._keyboard_pending:
            # An orphaned SSH helper never resumes after a lost heartbeat. Still
            # require an ordered native stop before clearing its persisted fence.
            response = self.client.native('soft_stop')
            if (response.get('success') is not True or response.get('stationary') is not True
                    or response.get('software_estop') is not True
                    or (self.keyboard.thread and self.keyboard.thread.is_alive())):
                raise RuntimeError('mapping_keyboard_stop_unconfirmed')
            self._keyboard_pending = False
            self._persist()
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
            not_submitted = self._task is not None and self._task.get('submission_started') is False and not uncertain
            response = self.client.native('soft_stop' if not_submitted else 'cancel')
            if response.get('success') is not True or response.get('stationary') is not True:
                raise RuntimeError('native_navigation_stop_unconfirmed')
            if not_submitted and response.get('software_estop') is not True:
                raise RuntimeError('chassis_soft_stop_unconfirmed')
            if uncertain:
                raise RuntimeError('older_navigation_submit_unresolved_stop_cannot_be_confirmed')
            self._task = None
            self._persist()

    def _map_worker_active(self):
        return any(w.is_alive() and w.name == 'kuavo-map-operation' for w in self._workers.copy())

    def mapping_console_status(self):
        with self.bundle.tracker.lock:
            pending = self._map_pending or {}
            mapping = self.store.state('mapping') or {}
            waiting = (pending.get('operation') == 'start_mapping' and self._map_worker_active()
                and self._mapping_console_grant == ('starting', pending.get('action_id'),
                    self.bundle._stop_epoch, self.bundle.ros1.connection))
            authorized = (mapping.get('phase') == 'mapping' and self._map_pending is None
                and self._mapping_console_grant == (mapping.get('token'), self.bundle._stop_epoch,
                    self.bundle.ros1.connection))
            return {'mapping_state': 'starting' if waiting else 'mapping' if authorized else 'unavailable',
                    'console_authorized': bool(waiting or authorized)}

    def _settle_held_preparation(self):
        """Clear only a resolved preparation after passive held/idle evidence."""
        if time.monotonic() < self._preparation_check_at or not self._native_lock.acquire(blocking=False):
            return
        try:
            with self.bundle.tracker.lock:
                preparation = self._preparation
                epoch, connection = self.bundle._stop_epoch, self.bundle.ros1.connection
                def eligible():
                    return (preparation is not None and preparation.get('delivery_uncertain') is False
                        and self._preparation == preparation and epoch == self.bundle._stop_epoch
                        and connection[0] and connection == self.bundle.ros1.connection
                        and not self._closed.is_set() and self.bundle._transition is None
                        and not self.bundle.tracker._active and not self._task and not self._uncertain
                        and self._map_pending is None and not self.store.state('mapping')
                        and not self._keyboard_pending and self.keyboard.session is None
                        and not self.bundle.state.motion_send_uncertain
                        and not self.bundle._resume_unresolved() and not self.bundle._mode_unresolved())
                if not eligible():
                    return
            self._preparation_check_at = time.monotonic()+2.
            try:
                observation = self.client.observe(required=('agvState', 'state'))
                state = observation['state']
                if (observation['agvState'].get('isMapping') is not False or state.get('driving') is not False
                        or any(state.get(key) != [] for key in ('nodeStates', 'edgeStates', 'actionStates'))):
                    return
                result = self.client.native('held_stationary')
                if (result.get('success') is not True or result.get('stationary') is not True
                        or result.get('software_estop') is not True):
                    return
                final = self.client.observe(required=('agvState', 'state'))
                if (final['agvState'].get('isMapping') is not False or final['state'].get('driving') is not False
                        or any(final['state'].get(key) != [] for key in ('nodeStates', 'edgeStates', 'actionStates'))):
                    return
            except Exception:
                return
            with self.bundle.tracker.lock:
                if eligible():
                    self._preparation = None
                    self._persist()
                    self.base_preparation_required = True
                    with self._startup_guard:
                        self._startup_key = None
        finally:
            self._native_lock.release()

    def _mapping_recovery(self):
        """Identify a detached mapping operation without retiring anything."""
        mapping, transition = self.store.state('mapping'), self._map_pending
        if (not mapping or not transition or self._map_worker_active()
                or transition.get('operation') not in ('start_mapping', 'stop_mapping')):
            return None
        return {'action': 'reconcile_mapping', 'map_name': mapping['name'],
                'phase': mapping.get('phase'), 'operation': transition['operation'],
                'stage': transition.get('stage'), 'confirm_abandon_required': True,
                'previous_chassis_operation_must_be_proven_inactive': True,
                'container_restart_is_not_evidence': True}

    def _auto_reconcile_mapping(self):
        """Retire proven obsolete metadata; never retry a factory command."""
        if not self._mapping_recovery() or not self._operations.acquire(blocking=False):
            return None
        try:
            recovery = self._mapping_recovery()
            if not recovery:
                return None
            key = json.dumps([self.store.state('mapping'), self._map_pending,
                              self.bundle._stop_epoch, self.bundle.ros1.connection], sort_keys=True)
            if key == self._mapping_cleanup_key and time.monotonic() < self._mapping_cleanup_retry_at:
                return None
            self._mapping_cleanup_key = key
            self._mapping_cleanup = {'state': 'checking', 'map_name': recovery['map_name']}
            try:
                result = self._reconcile_mapping({'map_name': recovery['map_name'],
                                                   'confirm_abandon': True}, automatic=True)
            except Exception as exc:
                # Unknown/lost feedback is not proof that a save ended. Retry
                # only these reads on a later request, without asking the user
                # to clear records or issuing a stop/restart/mode command.
                self._mapping_cleanup.update(state='waiting', error_type=type(exc).__name__)
                self._mapping_cleanup_retry_at = time.monotonic()+2.
                return None
            self._mapping_cleanup = {'state': 'completed', 'map_name': recovery['map_name'],
                                     'basis': result['basis'], 'files_and_tags_preserved': True,
                                     'motion_command_sent': False}
            self._mapping_cleanup_key = None
            self._mapping_cleanup_retry_at = 0.
            return result
        finally:
            self._operations.release()

    @staticmethod
    def _runtime_identity(runtime):
        return str(UUID(runtime['boot_id']))

    def _reconcile_mapping(self, args, automatic=False):
        name = safe_name(args.get('map_name'))
        if args.get('confirm_abandon') is not True:
            raise ValueError('confirm_abandon_required')
        with self._native_lock:
            with self.bundle.tracker.lock:
                mapping, pending = self.store.state('mapping'), self.store.state('pending')
                transition = json.loads(json.dumps(self._map_pending))
                epoch, connection = self.bundle._stop_epoch, self.bundle.ros1.connection
                def check():
                    if (not connection[0] or self.bundle.ros1.connection != connection
                            or epoch != self.bundle._stop_epoch or self._closed.is_set()
                            or self._map_worker_active() or self.bundle._transition is not None
                            or self.bundle.tracker._active or self.keyboard.session is not None
                            or self._keyboard_pending or self._task is not None or self._uncertain
                            or self._preparation is not None or self.bundle.state.motion_send_uncertain
                            or self.bundle._resume_unresolved() or self.bundle._mode_unresolved()
                            or self.store.state('mapping') != mapping or self._map_pending != transition):
                        raise RuntimeError('mapping_reconciliation_busy_or_state_changed')
                check()
                if (not mapping or mapping.get('name') != name or not transition
                        or transition.get('operation') not in ('start_mapping', 'stop_mapping')):
                    raise RuntimeError('matching_unconfirmed_mapping_session_required')
            # No stop/restart/mode/enable calls. A stopped web state alone does
            # not prove an old synchronous SaveCartoMap has finished.
            runtime = self.client.native('map_runtime_status')
            identity = self._runtime_identity(runtime)
            previous_boot = transition.get('chassis_boot_id')
            evidence_type = 'different_chassis_boot_id'
            if previous_boot is not None:
                if self._runtime_identity({'boot_id': previous_boot}) == identity:
                    raise RuntimeError('old_map_operation_may_still_be_running_chassis_reboot_not_confirmed')
            else:
                # Legacy records predate boot IDs. Require a clock-consistent
                # chassis boot strictly AFTER the persisted command timestamp.
                observed = number(runtime.get('observed_at'), 'observed_at')
                uptime = number(runtime.get('uptime_s'), 'uptime_s', 0)
                started = number(transition.get('stage_started_at'), 'stage_started_at')
                if abs(time.time()-observed) > 5 or observed-uptime <= started+10:
                    raise RuntimeError('legacy_map_operation_requires_confirmed_later_chassis_boot')
                evidence_type = 'legacy_command_predates_chassis_boot'
            current = None
            for _ in range(2):
                observation = self.client.observe(required=('agvState', 'state'))
                if (observation['agvState'].get('isMapping') is not False
                        or observation['state'].get('driving') is not False
                        or observation['state'].get('nodeStates') or observation['state'].get('edgeStates')
                        or observation['state'].get('actionStates')
                        or (automatic and any(observation['state'].get(key) != []
                                              for key in ('nodeStates', 'edgeStates', 'actionStates')))):
                    raise RuntimeError('chassis_mapping_or_task_still_active')
                selected = self.client.current_map()
                if current is not None and selected != current:
                    raise RuntimeError('current_map_changed_during_reconciliation')
                current = selected
            if self._runtime_identity(self.client.native('map_runtime_status')) != identity:
                raise RuntimeError('chassis_rebooted_during_reconciliation')
            evidence = {'state': 'abandoned', 'map_name': name, 'at': time.time(),
                        'previous_mapping': mapping, 'previous_transition': transition,
                        'chassis_boot_id': identity, 'basis': evidence_type,
                        'current_map': current, 'files_and_tags_preserved': True,
                        'motion_command_sent': False, 'automatic': automatic}
            with self.bundle.tracker.lock:
                check()
                self.store.retire_mapping(mapping, pending, evidence)
                self._map_pending = None
                self.base_preparation_required = True
                with self._startup_guard:
                    self._startup_key = None
            return {'state': 'completed', **{k: evidence[k] for k in
                    ('map_name', 'basis', 'current_map', 'files_and_tags_preserved', 'motion_command_sent')},
                    'message': 'Old mapping session retired. Map files and tags were preserved; deletion requires a separate confirmed delete_map request.'}

    def _map_action(self, action, args):
        epoch = self.bundle._stop_epoch
        if action == 'list_maps':
            maps = self.client.maps()
            names = {Path(item.get('fileName', '')).stem for item in maps}
            archived = self.archive.names() if self.archive else []
            maps = maps + [{'fileName': name + '.pgm', 'status': 'archive_only'}
                           for name in archived if name not in names]
            chassis = self.client.current_map()
            result = {'state': 'ready', 'maps': maps,
                      'current_map': chassis,
                      'chassis_map': chassis}
            recovery = self._mapping_recovery()
            if recovery:
                result.update(mapping_cleanup=self._mapping_cleanup or {'state': 'waiting'},
                    message='Maps are available; checking the previous mapping operation automatically.')
            return result
        mapping = self.store.state('mapping')
        recovering_save = (action == 'stop_mapping' and mapping and mapping.get('phase') == 'saving'
            and self._map_pending is not None
            and self._map_pending.get('operation') == 'stop_mapping'
            and self._map_pending.get('stage') == 'SaveCartoMap'
            and not any(w.is_alive() and w.name == 'kuavo-map-operation' for w in self._workers)
            and not self._task and not self._uncertain and self._preparation is None)
        recovering_delete = (action == 'delete_map' and self._map_pending is not None
            and self._map_pending.get('operation') == 'delete_map'
            and self._map_pending.get('stage') == 'delete_local_map'
            and self._map_pending.get('map_name') == args.get('map_name')
            and not any(w.is_alive() and w.name == 'kuavo-map-operation' for w in self._workers))
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
            if not mapping or (mapping['phase'] != 'mapping' and not recovering_save):
                raise RuntimeError('no_confirmed_mapping_session_inspect_info')
        elif action == 'delete_map':
            safe_name(args.get('map_name'))
            if mapping:
                raise RuntimeError('finish_or_reconcile_mapping_before_deleting')
            if args.get('confirm_delete') is not True:
                raise ValueError('confirm_delete_required')
            if self.client.current_map()['mapId'] == args['map_name']:
                raise ValueError('cannot_modify_active_map')
        elif action == 'load_map':
            text(args.get('map_name'), 'map_name')
            if mapping:
                raise RuntimeError('finish_or_reconcile_mapping_before_loading')
        else:
            raise ValueError('unsupported_map_action')
        if action == 'stop_mapping':
            self._mapping_console_grant = None
            if self.keyboard.session is not None or self._keyboard_pending:
                self.keyboard.stop()
                worker = self.keyboard.thread
                if worker is not None and worker.is_alive():
                    worker.join(timeout=8.)
        if self.keyboard.session is not None or self._keyboard_pending:
            raise RuntimeError('mapping_keyboard_stop_unconfirmed' if action == 'stop_mapping'
                               else 'release_mapping_keyboard_and_confirm_stop_before_map_change')
        restore_archive = False
        if action in ('start_mapping', 'load_map', 'delete_map'):
            exists = args['map_name'] in [Path(m.get('fileName', '')).stem for m in self.client.maps()]
            if action == 'delete_map':
                if not (exists or recovering_delete or (self.archive and self.archive.contains(args['map_name']))):
                    raise ValueError('grid_map_not_found')
            elif action == 'load_map' and not exists and self.archive and args['map_name'] in self.archive.names():
                restore_archive = True
            elif exists == (action == 'start_mapping'):
                raise ValueError('map_name_already_exists' if exists else 'grid_map_not_found')

        if recovering_save:
            # The old SaveCartoMap may still be optimizing with map TF paused.
            # Never send it twice. Only wait for its files, then finalize.
            stopped = self.client.native('stationary')
            if stopped.get('success') is not True or stopped.get('stationary') is not True:
                raise RuntimeError('fresh_stationary_odometry_required_before_changing_map')
        else:
            self._require_stationary()

        if action == 'start_mapping':
            self.bundle.prepare_mapping_velocity(epoch)

        # Map mutations cannot overlap chassis driving or another preparation.
        with self.bundle.tracker.lock:
            if epoch != self.bundle._stop_epoch:
                raise RuntimeError('map_request_superseded_by_stop')
            self.bundle.state.require_motion_enabled()
            if (self.bundle._transition is not None or 'base' in self.bundle.tracker._active
                    or (self.unresolved() and not (recovering_save or recovering_delete))):
                raise RuntimeError('base_busy_or_navigation_unresolved')
            aid, cancelled = self.bundle.tracker.begin('base', self.PREFIX)
            # A previous startup result cannot release the stop left by a
            # later manual session. Future upper-MPC actions need fresh prep.
            if action != 'delete_map':
                self.base_preparation_required = True
                with self._startup_guard:
                    self._startup_key = None
            self._map_pending = {'operation': action, 'action_id': aid, 'stage_started_at': time.time()}
            if recovering_save:
                self._map_pending['stage'] = 'SaveCartoMap'
            if recovering_delete:
                self._map_pending.update(stage='delete_local_map', map_name=args['map_name'])
            self._persist()
        self._map_deadline = time.monotonic() + (1800 if restore_archive or action in ('stop_mapping', 'delete_map') else 150)
        started = time.monotonic()
        timing = {'operation': action, 'started_at': time.time(), 'stages': []}
        if action in ('start_mapping', 'load_map', 'delete_map'):
            timing['map_name'] = args['map_name']
        self._map_timing = timing
        def worker():
            try:
                if action == 'delete_map':
                    result = self._delete_map(args['map_name'], cancelled)
                else:
                    if restore_archive:
                        self._restore_archive(args['map_name'], cancelled)
                        for key in ('stage', 'stage_started_at', 'map_name', 'transfer_id'):
                            self._map_pending.pop(key, None)
                        self._persist()
                    result = self._mutate_map(action, args, mapping, cancelled)
                if cancelled.is_set() and action != 'delete_map':
                    raise RuntimeError('map_operation_interrupted')
                timing.update(state='completed', elapsed_s=time.monotonic()-started)
                self.store.state('last_map_operation', timing)
                result['timing'] = dict(timing)
                with self.bundle.tracker.lock:
                    if (action == 'start_mapping' and not cancelled.is_set()
                            and epoch == self.bundle._stop_epoch
                            and self._mapping_console_grant == ('starting', aid, epoch, self.bundle.ros1.connection)):
                        completed_mapping = self.store.state('mapping') or {}
                        if completed_mapping.get('phase') == 'mapping':
                            self._mapping_console_grant = (completed_mapping['token'], epoch, self.bundle.ros1.connection)
                    self._map_pending = None
                    self._persist()
                if self.archive is not None:
                    self.archive.schedule()
                self.bundle.tracker.finish('base', aid, self.PREFIX, 'completed', result)
            except Exception as exc:
                error_code = map_failure_code(exc)
                timing.update(state='error', elapsed_s=time.monotonic()-started,
                              error_type=type(exc).__name__)
                if error_code:
                    timing['error_code'] = error_code
                self.store.state('last_map_operation', timing)
                failed_stage = (self._map_pending or {}).get('stage')
                # 8211 is the verified native precondition rejection, before
                # ChangeCartoMap touches the map. No mutation is still running.
                if ((failed_stage is None and self.store.state('mapping') is None) or
                        (action == 'load_map' and isinstance(exc, MapCommandRejected)
                         and exc.method == 'ChangeCartoMap' and exc.code == 8211)):
                    self._map_pending = None
                    self._persist()
                self.bundle.tracker.finish('base', aid, self.PREFIX, 'error',
                    {'reason': 'map_preflight_failed_no_command_sent' if failed_stage is None
                               else 'factory_map_command_rejected' if isinstance(exc, MapCommandRejected)
                               else 'map_transition_unconfirmed_inspect_info',
                     'failed_stage': failed_stage,
                     'error_type': type(exc).__name__, 'map_state': self.store.state('mapping'),
                     **({'error_code': error_code} if error_code else {})})
            finally:
                self._workers.discard(threading.current_thread())
        thread = threading.Thread(target=worker, daemon=True, name='kuavo-map-operation')
        self._workers.add(thread)
        thread.start()
        result = {'state': 'executing', 'action_id': aid, 'operation': action}
        if action == 'start_mapping':
            result['message'] = ('Starting mapping. Once ready, call mapping_console to drive the robot. '
                                 'Use this console instead of the G12 remote controller.')
        return result

    def _delete_map(self, name, cancelled):
        if self.client.current_map()['mapId'] == name:
            raise ValueError('cannot_modify_active_map')
        if self.client.observe(required=('agvState',))['agvState'].get('isMapping') is not False:
            raise RuntimeError('finish_factory_mapping_before_deleting')
        exists = name in {Path(m.get('fileName', '')).stem for m in self.client.maps()}
        if exists:
            if self._map_pending.get('stage') == 'delete_local_map':
                raise RuntimeError('chassis_map_reappeared_during_delete')
            manifest = self.client.native('map_delete_manifest', map_name=name)
            if (manifest.get('map_name') != name or not isinstance(manifest.get('files'), dict)
                    or not manifest['files'] or not set(manifest['files']) <= {'.pgm', '.yaml', '.pbstream', '.path'}):
                raise RuntimeError('invalid_map_delete_manifest')
            self._require_stationary()
            if self.client.observe(required=('agvState',))['agvState'].get('isMapping') is not False:
                raise RuntimeError('finish_factory_mapping_before_deleting')
            if cancelled.is_set() or time.monotonic() >= self._map_deadline:
                raise RuntimeError('map_delete_interrupted')
            token = uuid4().hex
            self._map_pending.update(stage='map_delete', stage_started_at=time.time(),
                                     map_name=name, transfer_id=token)
            self._persist()
            result = self.client.native('map_delete', map_name=name, token=token, manifest=manifest)
            if result.get('success') is not True:
                raise RuntimeError('chassis_map_delete_not_confirmed')
            self._wait(lambda: name not in {Path(m.get('fileName', '')).stem
                                            for m in self.client.maps()}, threading.Event(), 15)
        elif cancelled.is_set() and self._map_pending.get('stage') != 'delete_local_map':
            raise RuntimeError('map_delete_interrupted')
        self._map_pending.update(stage='delete_local_map', stage_started_at=time.time(), map_name=name)
        self._persist()
        if self.archive:
            self.archive.delete(name)
        self.store.delete_map(name)
        return {'state': 'completed', 'map_name': name, 'deleted': True}

    def _restore_archive(self, name, cancelled):
        if self.client.current_map()['mapId'] == name:
            raise ValueError('cannot_modify_active_map')
        token = uuid4().hex
        transfer = {'map_name': name, 'token': token}
        try:
            manifest = self.archive.stage_upload(name, token, cancelled, self._map_deadline)
            if cancelled.is_set():
                raise RuntimeError('map_transfer_interrupted')
            self._require_stationary()
            if cancelled.is_set() or time.monotonic() >= self._map_deadline:
                raise RuntimeError('map_transfer_interrupted')
            self._map_pending.update(stage='map_upload_commit', stage_started_at=time.time(),
                                     map_name=name, transfer_id=token)
            self._persist()
            response = self.client.native('map_upload_commit', **transfer, manifest=manifest)
            if response.get('success') is not True:
                raise RuntimeError('map_transfer_not_confirmed')
            # Verify the native map catalog, not just the SSH response.
            self._wait(lambda: name in {Path(m.get('fileName', '')).stem for m in self.client.maps()},
                       cancelled, 15)
        finally:
            try:
                self.client.native('map_upload_abort', **transfer)
            except Exception:
                pass

    def _require_stationary(self):
        state = self.client.observe(required=('agvState', 'state'))['state']
        # The factory web bridge can retain the final nonzero command velocity
        # while driving=False. Use fresh raw odometry to establish a stop.
        if state.get('driving') is not False:
            raise RuntimeError('stop_chassis_before_changing_map')
        result = self.client.native('stationary')
        if result.get('success') is not True or result.get('stationary') is not True:
            raise RuntimeError('fresh_stationary_odometry_required_before_changing_map')

    def _mutate_map(self, action, args, mapping, cancelled):
        recovering_save = action == 'stop_mapping' and mapping and mapping['phase'] == 'saving'
        if not recovering_save:
            self._require_stationary()
        hold = self.client.native('soft_stop' if action in ('start_mapping', 'load_map') else 'manual_hold')
        if (hold.get('success') is not True or hold.get('software_estop') is not True
                or hold.get('stationary') is not True):
            raise RuntimeError('map_operation_requires_confirmed_software_stop')
        if cancelled.is_set():
            raise RuntimeError('map_operation_interrupted')
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
            # Factory web RobotMotion is the MANUAL route. Never restore the
            # unrelated upper-board remote/MPC velocity source during mapping.
            def mapping_remote_ready():
                state = self.client.observe()['agvState']
                return state.get('isMapping') is True and state.get('runningMode') == 'MANUAL'
            self._wait(mapping_remote_ready, cancelled)
            try:
                self._require_stationary()
                start_pose, sample = self._observation()
                if sample['agvState'].get('isMapping') is True:
                    mapping['start_pose'] = start_pose
            except (RuntimeError, ValueError, TimeoutError):
                pass
            mapping['phase'] = 'mapping'
            self.store.state('mapping', mapping)
            return {'state': 'mapping', 'map_name': name,
                    'remote_control_route': 'factory_RobotMotion', 'chassis_mode': 'MANUAL',
                    'keyboard_control': 'open_mapping_console', 'software_stop': True,
                    'message': 'Mapping started. Please call mapping_console for remote control. Use this console instead of the G12 remote controller.'}
        if action == 'stop_mapping':
            if not mapping or mapping['phase'] not in ('mapping', 'saving'):
                raise RuntimeError('no_confirmed_mapping_session_inspect_info')
            mapping['phase'] = 'saving'
            self.store.state('mapping', mapping)
            if not recovering_save:
                self._command('ChangeMode', {'mode': 'MANUAL'}, cancelled)
                self._command('SaveCartoMap', {'mapName': mapping['name'], 'savePath': True}, cancelled)
            else:
                # A timed-out socket does not cancel the factory's save. The
                # final .path write fences the synchronous Cartographer call;
                # require stable output snapshots before any restart/load.
                previous = [None, None]
                def saved():
                    status = self.client.native('map_save_status', map_name=mapping['name'])
                    signature = status.get('files') if status.get('complete') is True else None
                    now = time.monotonic()
                    if signature is not None and signature == previous[0]:
                        return now - previous[1] >= 2.
                    previous[:] = [signature, now]
                    return False
                self._wait(saved, cancelled, 1500)
            self._wait(lambda: mapping['name'] in [Path(m.get('fileName', '')).stem
                                                   for m in self.client.maps()], cancelled, 1500)
            self._command('AgvSystemRestart', {}, cancelled)
            self._wait(lambda: self.client.observe()['agvState'].get('isMapping') is False, cancelled, 60)
            hold = self.client.native('manual_hold')
            if hold.get('software_estop') is not True or hold.get('stationary') is not True:
                raise RuntimeError('post_restart_software_stop_unconfirmed')
            self._load(mapping['name'], cancelled)
            new_identity = self._identity()
            self.store.bind_mapping((mapping['name'], mapping['token']), new_identity,
                                    mapping.get('start_pose'))
            self.store.state('mapping', None)
            return {'state': 'completed', 'map_name': new_identity[0], 'map_fingerprint': new_identity[1],
                    'mapping_start_pose': mapping.get('start_pose')}
        if action == 'load_map':
            if mapping:
                raise RuntimeError('finish_or_reconcile_mapping_before_loading')
            name = text(args.get('map_name'), 'map_name')
            if name not in [Path(m.get('fileName', '')).stem for m in self.client.maps()]:
                raise ValueError('grid_map_not_found')
            identity = self._load(name, cancelled)
            anchor = self.store.state(self.store.anchor_key(identity))
            correction = self._relocalize(identity, anchor, cancelled) if anchor is not None else None
            # Loading the files and initializing Cartographer are separate.
            # Give the factory localizer time to initialize without replaying
            # the map change or treating missing feedback as an unresolved write.
            localization, feedback = self._loaded_map_localization(name, cancelled)
            if correction is not None:
                localization['anchor'] = correction
                if (correction.get('state') not in ('corrected', 'already_aligned')
                        or self.store.state('unconfirmed:' + self.store.anchor_key(identity))):
                    localization.update(state='unconfirmed', reason='anchor_relocalization_not_verified')
            return {'state': 'completed', 'map_name': name,
                    'localization_required': localization['state'] != 'initialized',
                    'localization': localization, 'feedback': feedback}
        raise ValueError('unsupported_map_action')

    def _relocalize(self, identity, anchor, cancelled):
        anchor = {key: number(anchor[key], key) for key in ('x', 'y', 'yaw')}
        try:
            self._require_stationary()
        except (RuntimeError, TimeoutError):
            return {'state': 'stationary_pause_required'}
        if cancelled.is_set() or self._identity() != identity:
            raise RuntimeError('map_changed_or_stopped_before_relocalization')
        key = 'unconfirmed:' + self.store.anchor_key(identity)
        previous = self.store.state(key)
        self.store.state(key, True)
        try:
            result = self.client.native('relocalize', anchor=anchor)
        except Exception:
            return {'state': 'relocalization_delivery_unconfirmed'}
        state = result.get('state')
        if state in ('corrected', 'already_aligned') and self._identity() == identity:
            self.store.state(key, None)
        elif state in ('stationary_pause_required', 'invalid_map_or_scan', 'insufficient_scan',
                       'unsupported_localization_receiver', 'relocalization_precondition_lost'):
            # These states occur before any initial pose is published.
            self.store.state(key, previous)
        return result

    def _loaded_map_localization(self, name, cancelled):
        end = min(self._map_deadline, time.monotonic() + self.LOAD_LOCALIZATION_WAIT)
        feedback = None
        while not cancelled.is_set() and time.monotonic() < end:
            try:
                feedback = self.client.observe()
            except NavigationStateUnavailable:
                feedback = None
            if feedback is not None:
                position = feedback['state'].get('agvPosition', {})
                status = feedback['localizationStatus']
                if (position.get('mapId') == name and position.get('positionInitialized') is True
                        and status.get('cartoStatusId') == 1
                        and feedback['agvState'].get('isMapping') is False
                        and feedback['agvState'].get('runningMode') == 'AUTO'):
                    score = status.get('score')
                    return {'state': 'initialized',
                            'score': score if type(score) in (float, int) and math.isfinite(score) else None}, feedback
            cancelled.wait(min(.5, max(0., end - time.monotonic())))
        return {'state': 'unconfirmed', 'reason': 'factory_localization_not_confirmed'}, feedback

    def _command(self, method, params, cancelled):
        if cancelled.is_set():
            raise RuntimeError('map_operation_interrupted')
        remaining = self._map_deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('map_operation_timeout')
        if method == 'ChangeMode':
            try:
                if self.client.observe(required=('agvState',))['agvState'].get('runningMode') == params['mode']:
                    return  # Fresh feedback already proves the requested mode.
            except NavigationStateUnavailable:
                pass
        if self._map_pending is not None:
            runtime = self.client.native('map_runtime_status')
            identity = self._runtime_identity(runtime)
            if self._map_pending.get('chassis_boot_id', identity) != identity:
                raise RuntimeError('chassis_boot_changed_during_map_operation')
            self._map_pending['chassis_boot_id'] = identity
            self._map_pending['stage'] = method
            self._map_pending['stage_started_at'] = time.time()
            self._persist()
        started = time.monotonic()
        try:
            self.client.command(method, params, timeout=min(1500 if method == 'SaveCartoMap' else 25, remaining))
            if cancelled.is_set():
                raise RuntimeError('map_operation_interrupted_inspect_map_state')
            if method == 'ChangeMode':
                self._wait(lambda: self.client.observe(required=('agvState',))['agvState'].get('runningMode')
                           == params['mode'], cancelled)
        finally:
            if hasattr(self, '_map_timing'):
                self._map_timing['stages'].append({'stage': method, 'elapsed_s': time.monotonic()-started})

    def _load(self, name, cancelled):
        try:
            identity = self._identity()
        except Exception:
            identity = None  # A missing old map must not prevent loading a valid replacement.
        if identity and identity[0] == name and self.store.state('loaded_map_identity') == list(identity):
            observation = self.client.observe()
            if (observation['agvState'].get('isMapping') is False
                    and observation['localizationStatus'].get('cartoStatusId') == 1
                    and observation['state'].get('agvPosition', {}).get('mapId') == name
                    and observation['state']['agvPosition'].get('positionInitialized') is True):
                self._command('ChangeMode', {'mode': 'AUTO'}, cancelled)
                return identity
        self._command('ChangeMode', {'mode': 'MANUAL'}, cancelled)
        self._command('ChangeCartoMap', {'mapName': name}, cancelled)
        self._command('ChangeRoadMap', {'mapName': name}, cancelled)
        self._wait(lambda: self.client.current_map()['mapId'] == name, cancelled)
        self._command('ChangeMode', {'mode': 'AUTO'}, cancelled)
        identity = self._identity()
        if identity[0] != name:
            raise RuntimeError('map_changed_during_load_confirmation')
        self.store.state('loaded_map_identity', list(identity))
        return identity

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
        self._closed.set()
        self.keyboard.stop()
        if self.keyboard.thread:
            self._workers.add(self.keyboard.thread)
        if self.archive is not None:
            self.archive.wake.set()
            if self.archive.thread is not None:
                self._workers.add(self.archive.thread)
        for worker in list(self._workers):
            worker.join(timeout=1)
        if not any(worker.is_alive() for worker in self._workers):
            self.store.close()
