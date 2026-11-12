"""Calibrated passive gripper-to-table measurements; no hardware commands."""
from copy import deepcopy
import math
import threading
import time
import numpy as np
from common.vendor_runtime import tool
from agilex.piper.user_motion import rotation
from agilex.piper.monitor import _RosPublisher

def finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def vector(v, n):
    if not isinstance(v, (list, tuple)) or len(v) != n or not all(finite(x) for x in v):
        raise ValueError(f'需要 {n} 个有限数值')
    return np.array(v, dtype=float)


def section(config, key, defaults):
    raw = config.get(key, {})
    if not isinstance(raw, dict) or set(raw) - set(defaults):
        raise ValueError(f'{key} 配置包含未知字段或不是对象')
    cfg = deepcopy({**defaults, **raw})
    if type(cfg['enabled']) is not bool:
        raise ValueError(f'{key}.enabled 必须为布尔值')
    return cfg


CLEARANCE_DEFAULTS = dict(enabled=False, calibration_id='', verified=False, can_interface='',
    base_frame='', plane_base_mm=None, body_vertices_mm=[],
    left_finger_vertices_mm=[], right_finger_vertices_mm=[], opening_axis=None,
    opening_range_mm=None, uncertainty_mm=None)


def clearance_settings(config):
    cfg = section(config, 'table_clearance', CLEARANCE_DEFAULTS)
    if type(cfg['verified']) is not bool:
        raise ValueError('table_clearance.verified 必须为布尔值')
    for key in ('calibration_id', 'base_frame', 'can_interface'):
        if not isinstance(cfg[key], str) or len(cfg[key]) > 128:
            raise ValueError(f'{key} 必须为不超过 128 字符的文本')
    if not cfg['verified']:
        return cfg
    if not all(cfg[k].strip() for k in ('calibration_id', 'base_frame', 'can_interface')):
        raise ValueError('验证后的标定必须提供 calibration_id、base_frame 和 can_interface')
    plane = vector(cfg['plane_base_mm'], 4)
    if abs(np.linalg.norm(plane[:3]) - 1) > 1e-6 or plane[2] <= 0:
        raise ValueError('桌面法线必须是朝上的单位向量；plane=[nx,ny,nz,d_mm]')
    axis = vector(cfg['opening_axis'], 3)
    if abs(np.linalg.norm(axis) - 1) > 1e-6:
        raise ValueError('opening_axis 必须为 SDK 末端坐标系下的单位向量')
    limits = vector(cfg['opening_range_mm'], 2)
    if not 0 <= limits[0] < limits[1] <= 1000:
        raise ValueError('opening_range_mm 无效')
    if not finite(cfg['uncertainty_mm']) or not 0 <= cfg['uncertainty_mm'] <= 100:
        raise ValueError('必须填写实测 uncertainty_mm，范围 0–100 mm')
    for key in ('body_vertices_mm', 'left_finger_vertices_mm', 'right_finger_vertices_mm'):
        vertices = cfg[key]
        if not isinstance(vertices, list) or not 4 <= len(vertices) <= 256:
            raise ValueError(f'{key} 必须含 4–256 个凸包顶点')
        points = np.array([vector(v, 3) for v in vertices])
        if np.linalg.matrix_rank(points[1:] - points[0]) != 3:
            raise ValueError(f'{key} 顶点必须描述三维几何')
    return cfg


def table_distance(snapshot, cfg, now):
    """Minimum signed normal distance of the three calibrated convex hulls."""
    bad = dict(state='unavailable', clearance_mm=None, lowest_point_base_mm=None,
               conservative_clearance_mm=None)
    if not cfg['verified']:
        return dict(bad, reason='calibration_unverified')
    if snapshot.get('connected') is not True:
        return dict(bad, reason='disconnected')
    if snapshot.get('can_interface') != cfg['can_interface']:
        return dict(bad, reason='calibration_arm_mismatch')
    pose, grip = snapshot.get('end_pose') or {}, snapshot.get('gripper') or {}
    stamps = [pose.get('feedback_time'), grip.get('feedback_time')]
    if not all(finite(t) and 0 <= now - t <= .5 for t in stamps):
        return dict(bad, reason='missing_or_stale_feedback')
    if abs(stamps[0] - stamps[1]) > .1:
        return dict(bad, reason='pose_gripper_time_skew')
    width = grip.get('width_mm')
    if not finite(width) or not cfg['opening_range_mm'][0] <= width <= cfg['opening_range_mm'][1]:
        return dict(bad, reason='opening_outside_calibrated_range')
    position, rpy = vector(pose.get('position_mm'), 3), vector(pose.get('orientation_deg'), 3)
    axis = np.array(cfg['opening_axis']) * width / 2
    # Finger vertices are measured at ZERO opening. Positive/negative local
    # translation must be physically verified for the installed finger pair.
    local = np.concatenate([np.array(cfg['body_vertices_mm']),
        np.array(cfg['left_finger_vertices_mm']) + axis,
        np.array(cfg['right_finger_vertices_mm']) - axis])
    points = local @ np.array(rotation(rpy)).T + position
    plane = np.array(cfg['plane_base_mm'])
    distances = points @ plane[:3] + plane[3]
    index = int(np.argmin(distances))
    distance = float(distances[index])
    return dict(state='ok', reason=None, clearance_mm=distance,
        conservative_clearance_mm=distance - cfg['uncertainty_mm'],
        lowest_point_base_mm=points[index].tolist(), feedback_time=min(stamps),
        opening_mm=width, unit='mm', quantity='signed_plane_normal_distance',
        calibration_id=cfg['calibration_id'], base_frame=cfg['base_frame'],
        uncertainty_mm=cfg['uncertainty_mm'], model='calibrated_convex_hulls')


class TableClearance:
    def __init__(self, read_snapshot, config, namespace, ros2=None, *, clock=time.time,
                 monotonic=time.monotonic, publisher_factory=_RosPublisher):
        self.cfg = clearance_settings(config)
        self.read_snapshot, self.namespace, self.ros2 = read_snapshot, namespace.strip('/'), ros2
        self.clock, self.monotonic, self.factory = clock, monotonic, publisher_factory
        self.lock, self.lifecycle = threading.RLock(), threading.RLock()
        self.running, self.generation, self.transport = False, 0, None
        self.result = self.received = None

    def topics(self):
        return [{'topic': f'/{self.namespace}/piper/state/table_clearance', 'format': 'data/json'}]

    def get_tools(self):
        definition = tool('piper_table_clearance', 'sensor',
            '持续输出经标定夹爪模型最低点到桌面的有符号法向间隙（毫米）。'
            '复用已有反馈；缺少标定或反馈失效则不可用。仅供观测，不执行控制或避碰。',
            {'type': 'object', 'properties': {}, 'additionalProperties': False}, topic_out=self.topics())
        definition['title'] = '夹爪离桌距离'
        return [definition]

    def unavailable(self, reason):
        return dict(state='unavailable', reason=reason, clearance_mm=None,
                    conservative_clearance_mm=None, lowest_point_base_mm=None)

    def current(self):
        if not self.running:
            return dict(state='idle')
        if self.result is None:
            return self.unavailable('waiting_for_feedback')
        stamp = self.result.get('feedback_time')
        if self.result['state'] == 'ok' and (self.received is None
                or not 0 <= self.monotonic() - self.received <= .5
                or not finite(stamp) or not 0 <= self.clock() - stamp <= .5):
            return self.unavailable('feedback_expired')
        return deepcopy(self.result)

    def info(self):
        with self.lock:
            return dict(running=self.running, publishing=self.transport is not None,
                        result=self.current(), config=deepcopy(self.cfg), topic_out=self.topics())

    def start(self):
        with self.lifecycle, self.lock:
            if self.running:
                return self.info()
            self.generation += 1
            gen = self.generation
            self.result = self.received = None
            if self.ros2 is not None:
                self.transport = self.factory(self.ros2, self.namespace + '/table_clearance',
                    self.topics(), .2, lambda: self.tick(gen))
            self.running = True
            return self.info()

    def stop(self):
        with self.lifecycle:
            with self.lock:
                self.running = False
                self.generation += 1
                transport, self.transport = self.transport, None
                self.result = self.received = None
            if transport is not None:
                transport.close()
            return self.info()

    def tick(self, gen):
        with self.lock:
            if not self.running or gen != self.generation:
                return
            try:
                snapshot = self.read_snapshot() if self.cfg['verified'] else {}
                self.result = table_distance(snapshot, self.cfg, self.clock())
                self.received = self.monotonic()
            except Exception:
                self.result, self.received = self.unavailable('invalid_feedback'), None
            result = dict(self.current(), timestamp=self.clock(), advisory_only=True)
            if self.transport is not None:
                try:
                    self.transport.publish('data/json', result)
                except Exception:
                    self.result, self.received = self.unavailable('publish_failed'), None

    def dispatch(self, action, args):
        allowed = {'_tool_name', 'instance_id'}
        if args.get('input_topic') == '':
            allowed.add('input_topic')
        if set(args) - allowed or action not in {'start', 'stop', 'info', 'piper_table_clearance'}:
            raise ValueError('夹爪离桌距离仅接受 start、stop、info')
        return self.info() if action == 'piper_table_clearance' else getattr(self, action)()


def add_table_clearance(plugins, config, namespace, ros2):
    cfg = clearance_settings(config)
    if not cfg['enabled']:
        return plugins
    return [*plugins, TableClearance(plugins[0].read_monitor_snapshot, config, namespace, ros2)]
