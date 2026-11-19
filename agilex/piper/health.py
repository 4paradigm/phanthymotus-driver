"""Read-only health diagnostics from the existing runtime's passive snapshot.

This card owns only a DDS publisher. It never connects to CAN, calls status(),
changes permission, or invokes an arm/gripper command. Readiness is advisory:
the control card still performs its own per-action and preparation checks.
"""
from collections.abc import Mapping
import math
import re
import threading
import time

from common.vendor_runtime import tool
from agilex.piper.monitor import MAX_FEEDBACK_AGE, _RosPublisher


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _text(value):
    return value[:500] if isinstance(value, str) else None


def health_report(snapshot, *, now, error=None):
    """Diagnose direct arm control prerequisites; unknown never means passed."""
    snapshot = snapshot if isinstance(snapshot, Mapping) else {}
    connected = snapshot.get('connected')
    connected = connected if type(connected) is bool else None
    stamp = snapshot.get('feedback_time')
    age = now - stamp if _number(stamp) else None
    angles = snapshot.get('joints_deg')
    valid_angles = (isinstance(angles, (list, tuple)) and len(angles) == 6
                    and all(_number(q) for q in angles))
    fresh = (not error and connected is True and valid_angles
             and snapshot.get('fresh', snapshot.get('feedback_fresh')) is True
             and age is not None and 0 <= age <= MAX_FEEDBACK_AGE)
    faults = snapshot.get('faults')
    faults = faults if fresh and isinstance(faults, Mapping) else {}
    mode = _text(snapshot.get('control_mode')) if fresh else None
    enabled = snapshot.get('enabled') if fresh else None
    if not (isinstance(enabled, (list, tuple)) and len(enabled) == 6
            and all(type(value) is bool for value in enabled)):
        enabled = None
    code = faults.get('error_code')
    code = code if type(code) is int else None
    motor_faults = faults.get('motor_faults')
    if not (isinstance(motor_faults, (list, tuple))
            and all(isinstance(value, str) for value in motor_faults)):
        motor_faults = None
    arm_status = _text(faults.get('arm_status'))
    teach_status = _text(faults.get('teach_status'))
    busy = snapshot.get('busy')
    busy = busy if type(busy) is bool else None
    permission = snapshot.get('motion_enabled')
    permission = permission if type(permission) is bool else None
    started = snapshot.get('control_card_started')
    started = started if type(started) is bool else None
    limits = snapshot.get('joint_limits_deg')
    valid_limits = (isinstance(limits, (list, tuple)) and len(limits) == 6
                    and all(isinstance(pair, (list, tuple)) and len(pair) == 2
                            and all(_number(v) for v in pair) and pair[0] < pair[1]
                            for pair in limits))
    in_limits = (all(lo <= q <= hi for q, (lo, hi) in zip(angles, limits))
                 if fresh and valid_limits else None)
    healthy = (code == 0 and not motor_faults and arm_status.startswith('NORMAL')
               if code is not None and motor_faults is not None and arm_status else None)
    stopped_teaching = (teach_status.startswith(('DISABLED', 'STOP_RECORDING', 'TERMINATE_EXECUTION'))
                        if teach_status else None)
    all_enabled = all(enabled) if enabled is not None else None
    labels = {
        'connection': '共享驱动连接可用', 'feedback': '六轴反馈有效且未超过 0.5 秒',
        'faults': '机械臂状态正常且未报告故障', 'mode': '当前为 CAN 控制模式',
        'teaching': '示教录制或回放已停止', 'enabled': '六轴全部使能',
        'limits': '当前关节位置位于有效限位内', 'permission': '部署运动许可已开启',
        'control_card': 'Arm control 卡片已启动', 'idle': '当前没有驱动任务执行',
    }
    values = {'connection': connected, 'feedback': bool(fresh), 'faults': healthy,
              'mode': mode.startswith('CAN_CTRL') if mode else None,
              'teaching': stopped_teaching, 'enabled': all_enabled, 'limits': in_limits,
              'permission': permission, 'control_card': started,
              'idle': not busy if busy is not None else None}
    checks = [{'id': key, 'passed': passed, 'description': labels[key]}
              for key, passed in values.items()]
    blockers = [{'id': check['id'], 'reason': check['description'] +
                 ('：未满足' if check['passed'] is False else '：反馈不可用')}
                for check in checks if check['passed'] is not True]
    ready = not blockers and not error
    state = ('read_error' if error else 'disconnected' if connected is False else
             'unavailable' if connected is None else 'stale' if not fresh else
             'fault' if healthy is False else 'ready' if ready else 'blocked')
    titles = {'read_error': '读取失败', 'disconnected': '未连接', 'unavailable': '状态未知',
              'stale': '反馈过期或无效', 'fault': '设备报告异常',
              'ready': '通过基础检查', 'blocked': '直接控制条件未满足'}
    enable_label = ('全部使能' if all_enabled else '部分使能' if any(enabled) else '全部失能') if enabled else '不可用'
    display = {
        '设备': _text(snapshot.get('device_name')) or 'PiPER',
        '诊断结果': titles[state], '数据来源': '共享驱动实际反馈',
        '连接': '已连接' if connected is True else '未连接' if connected is False else '未知',
        '六轴反馈': '新鲜' if fresh else '过期或不可用',
        '反馈年龄（秒）': round(age, 3) if age is not None and age >= 0 else None,
        '控制模式': mode or '不可用', '电机使能': enable_label,
        **{f'J{i + 1} 使能': value for i, value in enumerate(enabled or [None] * 6)},
        '设备状态': arm_status or '不可用', '故障码': code,
        '电机故障': ', '.join(motor_faults) if motor_faults else '无' if motor_faults == [] or motor_faults == () else '不可用',
        '示教状态': teach_status or '不可用', '运动许可': permission,
        '任务执行中': busy, '当前动作': _text(snapshot.get('active_action')),
        '阻塞原因': '；'.join(item['reason'] for item in blockers) or '无',
        '说明': '仅检查直接控制前置条件；Arm control 仍会逐次校验并按需准备。未检查路径、障碍物与现场安全。',
    }
    if error:
        display['读取错误'] = error
    return {'schema': 'piper.health/1', 'timestamp_ms': int(now * 1000),
            'status': state, 'connected': connected, 'feedback_fresh': bool(fresh),
            'feedback_time': stamp if _number(stamp) else None,
            'feedback_age_s': age if age is not None and age >= 0 else None,
            'ready_for_direct_control': ready, 'checks': checks, 'blockers': blockers,
            'display': display, 'read_error': error}


class PiperHealthPlugin:
    """A 2 Hz JSON sensor with an independent publishing lifecycle."""
    def __init__(self, read_snapshot, *, namespace, ros2=None, clock=time.time,
                 publisher_factory=None):
        if not callable(read_snapshot):
            raise TypeError('read_snapshot must be callable')
        namespace = namespace.strip('/')
        if not namespace or not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', p) for p in namespace.split('/')):
            raise ValueError('A valid explicit ROS namespace is required')
        self._read_snapshot, self._namespace = read_snapshot, namespace
        self._ros2, self._clock = ros2, clock
        self._publisher_factory = publisher_factory or _RosPublisher
        self._lock = threading.RLock()
        self._transport = None
        self._running = False
        self._generation = 0
        self._publish_error = None

    def _topics(self):
        return [{'topic': f'/{self._namespace}/state/health', 'format': 'data/json'}]

    def get_tools(self):
        definition = tool('piper_health', 'sensor',
            'Read-only health and direct-control prerequisite diagnostics at 2 Hz. '
            'Shares existing feedback; never connects CAN, enables motors, changes mode or sends motion. '
            'Passing these checks does not authorize motion or certify a safe path.',
            {'type': 'object', 'properties': {}, 'additionalProperties': False},
            topic_out=self._topics())
        definition['title'] = 'Health'
        return [definition]

    def read(self):
        error = None
        try:
            snapshot = self._read_snapshot()
            if not isinstance(snapshot, Mapping):
                raise ValueError('Snapshot provider must return a mapping')
        except Exception as exc:
            snapshot = {}
            error = str(exc).replace('\n', ' ').replace('\r', ' ')[:200] or type(exc).__name__
        return health_report(snapshot, now=self._clock(), error=error)

    def info(self):
        report = self.read()
        with self._lock:
            return {'state': 'error' if self._publish_error else 'running' if self._running else 'idle',
                    'publish_hz': 2, 'topic_out': self._topics(),
                    'publishing': self._transport is not None,
                    'publish_error': self._publish_error, 'health': report}

    def start(self):
        with self._lock:
            if not self._running:
                self._generation += 1
                generation = self._generation
                if self._ros2 is not None:
                    # Distinct node name from the existing skeleton publisher.
                    self._transport = self._publisher_factory(
                        self._ros2, self._namespace + '/health', self._topics(), .5,
                        lambda: self._tick(generation))
                self._running = True
        return self.info()

    def stop(self):
        with self._lock:
            self._running = False
            self._generation += 1
            self._publish_error = None
            transport, self._transport = self._transport, None
            if transport is not None:
                transport.close()
        return self.info()

    def _tick(self, generation):
        with self._lock:
            if not self._running or generation != self._generation:
                return
        report = self.read()
        with self._lock:
            if not self._running or generation != self._generation or self._transport is None:
                return
            try:
                # The existing KV renderer displays a compact Chinese panel.
                # info.health retains machine-readable check IDs and booleans.
                self._transport.publish('data/json', report['display'])
                self._publish_error = None
            except Exception as exc:
                self._publish_error = str(exc).replace('\n', ' ')[:200]

    def dispatch(self, action, args):
        allowed = {'action', '_tool_name', 'instance_id'}
        if args.get('input_topic') == '':
            allowed.add('input_topic')
        if set(args) - allowed or args.get('_tool_name', 'piper_health') != 'piper_health':
            raise ValueError('Health only accepts read-only lifecycle operations')
        if action == 'piper_health':
            action = 'info'
        if action not in {'start', 'stop', 'info'}:
            raise ValueError('Health does not support control actions')
        return getattr(self, action)()


def add_health(plugins, ros2):
    from agilex.piper.runtime import PiperRuntime

    runtime = plugins[0]

    def read_snapshot():
        # The base passive contract contains everything needed here. Avoid
        # control-card extensions (TCP/path capabilities and recovery metadata),
        # which assume connected controller feedback in some source versions.
        snapshot = PiperRuntime.read_monitor_snapshot(runtime)
        snapshot['control_card_started'] = not runtime.closed and 'arm_control' not in runtime.stopped_cards
        return snapshot

    return [*plugins, PiperHealthPlugin(read_snapshot,
        namespace=runtime.topic.rsplit('/', 1)[0].strip('/'), ros2=ros2)]
