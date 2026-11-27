"""Synthetic point-control fixtures using this checkout's production modules.

No DDS connection or robot device is opened. The real executor, packet envelope,
release state machine, FK and collision model run against deterministic feedback.
The numerical envelope method runs with list-backed array primitives; this does
not claim native Pinocchio integration or physical robot validation.
"""
import ast
import inspect
import json
import math
from functools import lru_cache
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

DRIVER = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DRIVER),str(DRIVER.parents[1])]
import arm_stream
from arm_stream import ArmStreamExecutor, MOTOR_IDS
from directional_backend import DirectionalBackend

TARGET = [.2,-.1,.15,.2,.1]
FIELD_START = [.2926547825,-.2243207246,.0195942074,.9778409004,-.1476456523]
BIAS = (.004,-.004,.006,-.003,.007,.004,.0020732730627059937,-.004074635915458202,-.003,.007)


def calibration(tmp_path):
    profile = json.loads((DRIVER/'tests/fixtures/calibration.json').read_text())
    profile['urdf_path'] = str(DRIVER/'motion/models/body23.urdf')
    profile['locked_joints'] = dict.fromkeys(profile['locked_joints'],0.)
    for side in ('left','right'):
        profile['palm_frames'][side] = {'position':[.2,0.,0.],'orientation':[0.,0.,0.,1.]}
    path = tmp_path/'synthetic-point-calibration.json'
    path.write_text(json.dumps(profile))
    return path

class Channel:
    def __init__(self, sample):
        self.sample = sample
        self.ids = list(MOTOR_IDS)
        self.writes = []
        self.closed = False

    def select_arm_motors(self, ids):
        self.ids = list(ids)

    def open(self):
        self.closed = False

    def publish_servo_position(self, q, *, weight):
        assert self.ids == list(MOTOR_IDS[5:])
        assert len(q) == 5
        self.writes.append((list(q), weight))
        self.sample['q'][5:] = list(q)
        return True

    def close(self):
        self.closed = True

    def forget_target(self):
        pass  # Synthetic position plant has no independently queued target.


def rig(tmp_path, monkeypatch, **config):
    now = [1_000_000_000]
    sample = {'q': [0.]*10, 'dq': [0.]*10, 'arm_ns': now[0]}
    channel = Channel(sample)
    vendor = []
    arm = ArmStreamExecutor({'live_enabled': True, 'servo_position': True}, 'g1',
                            SimpleNamespace(ExecuteAction=lambda a: vendor.append(a) or 0),
                            snapshot=lambda: sample, channel=channel, clock=lambda: now[0])
    path = calibration(tmp_path)
    profile = json.loads(path.read_text())
    profile['palm_frames']['right'] = {'position': [.2, 0., 0.], 'orientation': [0., 0., 0., 1.]}
    path.write_text(json.dumps(profile))
    arm.configure_profile(path)
    admitted = [True]
    fresh = [True]

    def advance(seconds):
        for _ in range(max(1, round(seconds/.004))):
            now[0] += 4_000_000
            if fresh[0]:
                sample['arm_ns'] = now[0]
            arm.tick()

    class Stop(threading.Event):
        def wait(self, timeout=None):
            advance(timeout or .02)
            return self.is_set()

    def dispatch(action, args):
        return arm.dispatch(action, args)

    def joint_envelope(measured, previous, target):
        if any(not lo <= q <= hi for q, (lo, hi) in zip(target, arm.limits)):
            raise ValueError('motion_envelope_joint_limit')
        return {'lower': [min(values)-.002 for values in zip(measured, previous, target)],
                'upper': [max(values)+.002 for values in zip(measured, previous, target)]}

    motion = SimpleNamespace(executor=arm, dispatch=dispatch,
                             solver=SimpleNamespace(joint_envelope=joint_envelope))
    backend = DirectionalBackend(motion, stationary=lambda: {'stationary': admitted[0]},
                                 config={'verified_start_q_rad': [0.]*5, **config})
    monkeypatch.setattr('directional_backend.time.sleep', advance)
    return backend, arm, sample, channel, Stop(), admitted, fresh, vendor, advance

def setup(tmp_path, monkeypatch, *, isolate_geometry=True, **config):
    backend, arm, sample, channel, stop, admitted, fresh, vendor, advance = rig(
        tmp_path, monkeypatch, left_arm_mode='hold_measured', solver_mode='bounded_ik',
        ik_reference_q_rad=[-.268, -.096, .005, .041, 1.50], **config)
    writes = []

    def publish(q, *, weight):
        assert channel.ids == [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]
        assert len(q) == 10
        writes.append((arm.clock(), list(q), weight))
        channel.writes.append((list(q), weight))
        sample['q'][:] = q
        return True

    channel.publish_point_position = publish
    if isolate_geometry:
        # Fault-injection tests isolate the executor; the integration test below
        # uses the real repository model and the real checked path.
        monkeypatch.setattr(backend.kinematics, 'check_path', lambda *a, **kw: .01)
        if callable(getattr(backend.kinematics, 'check_measured_pose', None)):
            monkeypatch.setattr(backend.kinematics, 'check_measured_pose', lambda *a, **kw: .01)
    return backend, arm, sample, channel, stop, admitted, fresh, writes, advance

def assert_released(arm, channel):
    state = arm.status()
    assert not state['ownership_held'] and not state['output_active']
    assert state['release']['physical_confirmed'] and channel.closed


def delayed_bias_plant(arm, sample, channel, monkeypatch, *, lag_ns, bias=BIAS):
    """Feedback follows old commands plus bias, never the latest command exactly.

    Bias persists through takeover, moving, holding and returning. The plant
    returns to its ordinary fixture only when explicit handback begins. This is
    a deterministic error/latency model, not a robot dynamics simulation.
    """
    phase = ['taking_over']
    history = [(arm.clock()-lag_ns-1, list(sample['q']))]
    reads = []
    writes = []
    original_publish = channel.publish_point_position
    original_snapshot = arm._snapshot_provider

    def publish(q, *, weight):
        result = original_publish(q, weight=weight)
        history.append((arm.clock(), list(q)))
        writes.append((arm.clock(), list(q), weight, phase[0]))
        return result

    def snapshot():
        state = dict(original_snapshot())
        state['q'], state['dq'] = list(state['q']), list(state['dq'])
        if not arm.session_id or len(history) == 1 or phase[0] == 'releasing':
            return state
        stamp = state['arm_ns']-lag_ns
        index = max(i for i, (t, _) in enumerate(history) if t <= stamp)
        stamp_at, past = history[index]
        state['q'] = [q+b for q,b in zip(past, bias)]
        state['arm_ns'] = stamp
        if index:
            before_at, before = history[index-1]
            dt = (stamp_at-before_at)/1e9
            state['dq'] = [(q-p)/dt for q,p in zip(past, before)] if dt else [0.]*10
        else:
            state['dq'] = [0.]*10
        reads.append((phase[0], arm.clock()-stamp, list(state['q']),
                      list(arm.last_q), arm._weight))
        return state

    monkeypatch.setattr(channel, 'publish_point_position', publish)
    monkeypatch.setattr(arm, '_snapshot_provider', snapshot)
    return phase, reads, writes

def optional_left(method, left):
    """Exercise old right-only and new dual-arm model APIs with the same tests."""
    return {'left_q5': left} if 'left_q5' in inspect.signature(method).parameters else {}


class Vector(list):
    """List-backed primitives for the repository worker's actual envelope method."""
    def __lt__(self, other):
        return [a < b for a,b in zip(self,other)]

    def __gt__(self, other):
        return [a > b for a,b in zip(self,other)]

    def __sub__(self, scalar):
        return Vector(x-scalar for x in self)

    def __add__(self, scalar):
        return Vector(x+scalar for x in self)

    def tolist(self):
        return list(self)


@lru_cache(maxsize=1)
def repository_worker_envelope():
    # Use the repository method body rather than reimplementing its padding and
    # clipping algorithm. Only array primitives are replaced in this harness;
    # native numerical/Pinocchio integration is deliberately a separate test.
    source = Path(arm_stream.__file__).parent/'motion'/'kinematics.py'
    tree = ast.parse(source.read_text(encoding='utf8'))
    klass = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == 'IK')
    method = next(n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name == 'motion_envelope')

    def finite(values, shape):
        assert shape == (10,)
        if len(values) != 10 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError('invalid_finite_shape')
        return Vector(values)

    namespace = {'finite': finite, 'np': SimpleNamespace(
        minimum=lambda a,b: Vector(min(x,y) for x,y in zip(a,b)),
        maximum=lambda a,b: Vector(max(x,y) for x,y in zip(a,b)), any=any)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['motion_envelope']


def direct_setup(tmp_path, monkeypatch, **kwargs):
    values = setup(tmp_path, monkeypatch, **kwargs)
    backend, arm = values[:2]
    numerical = SimpleNamespace(model=SimpleNamespace(
        lowerPositionLimit=Vector(low for low,_ in arm.limits),
        upperPositionLimit=Vector(high for _,high in arm.limits)))
    method = repository_worker_envelope()
    monkeypatch.setattr(backend.motion.solver, 'joint_envelope',
                        lambda measured,lower,upper: method(numerical,measured,lower,upper))
    return values

class Backend:
    def __init__(self, directory):
        self.config = {'solver_mode': 'bounded_ik', 'receipt_directory': directory}
        self.kinematics = SimpleNamespace(
            solver_mode='bounded_ik', angular_tolerance_deg=3.,
            yaw_limits_deg=(-20, 20), pitch_limits_deg=(-5, 25),
            evaluate=lambda q, direction: {'direction': direction, 'angular_error_deg': 0.})
        self.starts = []
        self.calls = []
        self.error = None

    def plan(self, direction, hold):
        start = [len(self.starts) * .1] * 5
        self.starts.append(start)
        return {'start_q_rad': start, 'target_q_rad': [.3] * 5, 'planned_direction': direction}

    def execute(self, target, hold, stop, observe, *, planned_start_q5):
        self.calls.append(planned_start_q5)
        observe('taking_over', planned_start_q5)
        if self.error is not None:
            raise self.error
        observe('holding', target)
        observe('releasing', None)
        return {'left_hold': {'target_q_rad': [.1] * 5, 'actual_q_rad': [.1004] * 5,
                              'max_error_rad': .0004}}

