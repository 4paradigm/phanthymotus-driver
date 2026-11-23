"""Calibrated passive gripper-to-table measurements; no hardware commands."""
from copy import deepcopy
import math
import json
import threading
import time
import numpy as np
from common.vendor_runtime import tool
from agilex.piper.user_motion import rotation
from agilex.piper.monitor import _RosPublisher

def finite(v):
    try:
        return type(v) in (int, float) and math.isfinite(v)
    except OverflowError:
        return False


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
    arm_label='',
    mode='calibrated', estimate_profile_path='', table_z_mm=None,
    base_frame='', plane_base_mm=None, body_vertices_mm=[],
    left_finger_vertices_mm=[], right_finger_vertices_mm=[], opening_axis=None,
    opening_range_mm=None, uncertainty_mm=None)


def clearance_settings(config):
    cfg = section(config, 'table_clearance', CLEARANCE_DEFAULTS)
    if cfg['mode'] not in {'calibrated', 'model_estimate'}:
        raise ValueError('mode 必须为 calibrated 或 model_estimate')
    if type(cfg['verified']) is not bool:
        raise ValueError('table_clearance.verified 必须为布尔值')
    for key in ('calibration_id', 'base_frame', 'can_interface', 'arm_label'):
        if not isinstance(cfg[key], str) or len(cfg[key]) > 128:
            raise ValueError(f'{key} 必须为不超过 128 字符的文本')
    if cfg['mode'] == 'model_estimate':
        if cfg['verified'] or not cfg['can_interface'].strip():
            raise ValueError('估算模式必须保留 verified=false，且显式绑定 can_interface')
        if (not isinstance(cfg['estimate_profile_path'], str) or not cfg['estimate_profile_path']
                or not finite(cfg['table_z_mm'])):
            raise ValueError('估算模式需要本地模型 profile 和显式 table_z_mm')
        return cfg
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
        # Scale before subtraction/SVD: finite inputs can otherwise overflow.
        scale = float(np.max(np.abs(points)))
        normalized = points / scale if scale else points
        if np.linalg.matrix_rank(normalized[1:] - normalized[0]) != 3:
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
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            axis = np.array(cfg['opening_axis']) * width / 2
            # Finger vertices are measured at ZERO opening. Positive/negative
            # translation must be physically verified for this finger pair.
            local = np.concatenate([np.array(cfg['body_vertices_mm']),
                np.array(cfg['left_finger_vertices_mm']) + axis,
                np.array(cfg['right_finger_vertices_mm']) - axis])
            points = local @ np.array(rotation(rpy)).T + position
            plane = np.array(cfg['plane_base_mm'])
            distances = points @ plane[:3] + plane[3]
            if not all(np.isfinite(a).all() for a in (axis, local, points, distances)):
                raise FloatingPointError('nonfinite_geometry')
            index = int(np.argmin(distances))
            distance = float(distances[index])
            conservative = distance - cfg['uncertainty_mm']
            if not math.isfinite(conservative):
                raise FloatingPointError('nonfinite_geometry')
    except (FloatingPointError, OverflowError):
        return dict(bad, reason='nonfinite_geometry')
    return dict(state='ok', reason=None, clearance_mm=distance,
        conservative_clearance_mm=conservative,
        lowest_point_base_mm=points[index].tolist(), feedback_time=min(stamps),
        opening_mm=width, unit='mm', quantity='signed_plane_normal_distance',
        calibration_id=cfg['calibration_id'], base_frame=cfg['base_frame'],
        uncertainty_mm=cfg['uncertainty_mm'], model='calibrated_convex_hulls')


class TableClearance:
    def __init__(self, read_snapshot, config, namespace, ros2=None, *, clock=time.time,
                 monotonic=time.monotonic, publisher_factory=_RosPublisher, source_factory=None):
        self.cfg = clearance_settings(config)
        self.estimate_model = None
        if self.cfg['mode'] == 'model_estimate':
            from agilex.piper.clearance_estimate import load_profile
            self.estimate_model = load_profile(self.cfg['estimate_profile_path'])
        self.read_snapshot, self.namespace, self.ros2 = read_snapshot, namespace.strip('/'), ros2
        from agilex.piper.clearance_feedback import PassiveFeedback
        self.runtime_interface = config.get('can_interface','can0')
        self.source_factory = source_factory or PassiveFeedback
        self.source = (self.source_factory(self.cfg['can_interface'])
                       if self.cfg['can_interface'] and self.cfg['can_interface'] != self.runtime_interface else None)
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
            {'type': 'object', 'properties': {'action': {'type':'string',
                'enum':['begin_measurement','pause_measurement'],'default':'begin_measurement',
                'oneOf':[{'const':'begin_measurement','title':'开始测距（仅监测）'},
                         {'const':'pause_measurement','title':'暂停测距'}]}},
             'additionalProperties': False,'x-manual-execution':True,
             'x-action-params':{'begin_measurement':{'params':[],'description':'启动本卡只读距离数据发布'},
                                'pause_measurement':{'params':[],'description':'暂停本卡发布，不操作电机'}}}, topic_out=self.topics())
        if self.estimate_model is not None:
            definition['description'] = '按部署的夹爪模型、关节角度和开度估算离桌高度；未完成实测标定，不执行避碰。'
        definition['title'] = '夹爪离桌距离（估算）' if self.estimate_model is not None else '夹爪离桌距离'
        from agilex.piper.clearance_ui import schema
        definition['configSchema'] = schema(self.cfg)
        return [definition]

    def unavailable(self, reason):
        return dict(state='unavailable', reason=reason, clearance_mm=None,
                    conservative_clearance_mm=None, lowest_point_base_mm=None)

    def current(self):
        if not self.running:
            return dict(state='idle',clearance_mm=None,lowest_point_base_mm=None,
                        conservative_clearance_mm=None,reason='measurement_stopped')
        if self.result is None:
            return self.unavailable('waiting_for_feedback')
        stamp = self.result.get('feedback_time')
        if self.result['state'] in {'ok', 'estimated'} and (self.received is None
                or not 0 <= self.monotonic() - self.received <= .5
                or not finite(stamp) or not 0 <= self.clock() - stamp <= .5):
            return self.unavailable('feedback_expired')
        return deepcopy(self.result)

    def info(self):
        with self.lock:
            from agilex.piper.clearance_display import annotate
            return dict(state='running' if self.running else 'idle',running=self.running, publishing=self.transport is not None,
                        result=annotate(self.current(),self.cfg,self.clock()), config=deepcopy(self.cfg), topic_out=self.topics())

    def start(self):
        with self.lifecycle, self.lock:
            if self.running:
                return self.info()
            self.generation += 1
            gen = self.generation
            self.result = self.received = None
            try:
                if self.source is not None:self.source.start()
                if self.ros2 is not None:
                    self.transport = self.factory(self.ros2, self.namespace + '/table_clearance',
                        self.topics(), .2, lambda: self.tick(gen))
            except BaseException:
                if self.source is not None:self.source.close()
                raise
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
                try:
                    from agilex.piper.clearance_display import annotate
                    transport.publish('data/json',annotate(dict(self.current(),timestamp=self.clock(),
                        advisory_only=True),self.cfg,self.clock()))
                except Exception:
                    pass
                finally:
                    transport.close()
            if self.source is not None:self.source.close()
            return self.info()

    def tick(self, gen):
        with self.lock:
            if not self.running or gen != self.generation:
                return
            try:
                reader = self.source.snapshot if self.source is not None else self.read_snapshot
                snapshot = reader() if self.cfg['verified'] or self.estimate_model else {}
                if self.estimate_model is not None:
                    from agilex.piper.clearance_estimate import estimate
                    self.result = estimate(snapshot, self.cfg, self.clock(), self.estimate_model)
                else:
                    self.result = table_distance(snapshot, self.cfg, self.clock())
                self.received = self.monotonic()
            except Exception:
                self.result, self.received = self.unavailable('invalid_feedback'), None
            result = dict(self.current(), timestamp=self.clock(), advisory_only=True)
            from agilex.piper.clearance_display import annotate
            result = annotate(result,self.cfg,self.clock())
            if self.transport is not None:
                try:
                    self.transport.publish('data/json', result)
                except Exception:
                    self.result, self.received = self.unavailable('publish_failed'), None

    def dispatch(self, action, args):
        action={'begin_measurement':'start','pause_measurement':'stop'}.get(action,action)
        if action == 'config':return self.configure(args)
        if action == 'start' and set(args) & set(CLEARANCE_DEFAULTS):
            self.configure(args)
            return self.start()
        allowed = {'_tool_name', 'instance_id'}
        if args.get('input_topic') == '':
            allowed.add('input_topic')
        if set(args) - allowed or action not in {'start', 'stop', 'info', 'piper_table_clearance'}:
            raise ValueError('夹爪离桌距离仅接受 start、stop、info')
        return self.info() if action == 'piper_table_clearance' else getattr(self, action)()

    def configure(self,args):
        with self.lifecycle:
            allowed=set(CLEARANCE_DEFAULTS)-{'enabled'}
            meta={'_tool_name','instance_id'}
            if args.get('input_topic')=='':meta.add('input_topic')
            if set(args)-allowed-meta:raise ValueError('未知测距配置参数')
            updates={k:v for k,v in args.items() if k in allowed}
            for key in ['plane_base_mm','body_vertices_mm','left_finger_vertices_mm',
                        'right_finger_vertices_mm','opening_axis','opening_range_mm']:
                if isinstance(updates.get(key),str):updates[key]=json.loads(updates[key])
            if updates.get('mode')=='model_estimate':updates['verified']=False
            if updates.get('can_interface',self.cfg['can_interface'])!=self.cfg['can_interface']:
                updates['verified']=False
                updates['calibration_id']=''
                if updates.get('arm_label',self.cfg['arm_label'])==self.cfg['arm_label']:
                    updates['arm_label']='机械臂 '+updates['can_interface']
            cfg=clearance_settings({'table_clearance':{**self.cfg,**updates}})
            if cfg['can_interface'] not in {'can0','can1'}:raise ValueError('请选择 can0 或 can1')
            model=None
            if cfg['mode']=='model_estimate':
                from agilex.piper.clearance_estimate import load_profile
                model=load_profile(cfg['estimate_profile_path'])
            if cfg==self.cfg:return self.info()
            source=(self.source_factory(cfg['can_interface']) if cfg['can_interface']!=self.runtime_interface else None)
            running=self.running
            self.stop()
            with self.lock:self.cfg,self.estimate_model,self.source=cfg,model,source
            return self.start() if running else self.info()


def add_table_clearance(plugins, config, namespace, ros2):
    cfg = clearance_settings(config)
    if not cfg['enabled']:
        return plugins
    return [*plugins, TableClearance(plugins[0].read_monitor_snapshot, config, namespace, ros2)]
