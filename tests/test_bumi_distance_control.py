"""Distance feedback uses ROS-shaped messages but never opens hardware."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'noetix' / 'bumi'))
from distance_control import DistanceFeedback, run_distance


def pose(stamp=1_000_000_000, x=0, y=0, yaw=0):
    covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = 0.001
    return NS(header=NS(stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9),
                        frame_id='bumi_odom_odom'), child_frame_id='bumi_odom_base_link',
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0), orientation=NS(
                  x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2))), covariance=covariance))


@pytest.fixture
def feedback():
    clock = NS(ns=10**9, mono=1.0)
    result = DistanceFeedback(None, now_ns=lambda: clock.ns, monotonic=lambda: clock.mono)
    result.update_pose(pose())
    result.update_status({'status': 'tracking', 'segment_id': 'trial', 'stamp_ns': clock.ns})
    return result, clock


def test_feedback_requires_both_streams():
    result = DistanceFeedback(None)
    with pytest.raises(ValueError, match='feedback'):
        result.snapshot()


def test_feedback_accepts_pose_and_quality(feedback):
    result, _ = feedback
    sample = result.snapshot()
    assert sample['position'] == (0, 0, 0)
    assert sample['segment_id'] == 'trial'


@pytest.mark.parametrize('kind', ['source', 'receipt', 'quality'])
def test_stale_feedback_is_rejected(feedback, kind):
    result, clock = feedback
    clock.mono += 0.51
    if kind != 'receipt':
        clock.ns += 510_000_000
    if kind == 'quality':
        result.update_pose(pose(stamp=clock.ns))
    with pytest.raises(ValueError, match='stale'):
        result.snapshot()


@pytest.mark.parametrize('bad', ['frame', 'nan', 'quaternion', 'covariance', 'repeat', 'future', 'jump'])
def test_bad_pose_latches_loss(feedback, bad):
    result, clock = feedback
    clock.ns += 50_000_000
    msg = pose(stamp=clock.ns)
    if bad == 'frame':
        msg.header.frame_id = 'map'
    elif bad == 'nan':
        msg.pose.pose.position.x = float('nan')
    elif bad == 'quaternion':
        msg.pose.pose.orientation.w = 0
    elif bad == 'covariance':
        msg.pose.covariance[0] = -1
    elif bad == 'repeat':
        msg = pose()
    elif bad == 'future':
        msg = pose(stamp=clock.ns+100_000_000)
    else:
        msg.pose.pose.position.x = 1
    result.update_pose(msg)
    with pytest.raises(ValueError):
        result.snapshot()
    clock.ns += 50_000_000
    result.update_pose(pose(stamp=clock.ns))
    with pytest.raises(ValueError):
        result.snapshot()


def test_quality_loss_and_segment_change(feedback):
    result, clock = feedback
    result.update_status({'status': 'lost', 'segment_id': 'trial', 'stamp_ns': clock.ns})
    with pytest.raises(ValueError):
        result.snapshot()
    clock.ns += 50_000_000
    result.update_status({'status': 'tracking', 'segment_id': 'next', 'stamp_ns': clock.ns})
    with pytest.raises(ValueError):
        result.snapshot()  # A new trial cannot reuse the previous pose.
    result.update_pose(pose(stamp=clock.ns))
    assert result.snapshot()['segment_id'] == 'next'


class Plant:
    def __init__(self, monkeypatch, module, speed=0.3, heading=0):
        import threading
        self.clock = 1.0
        self.forward = 0.0
        self.speed = speed
        self.heading = heading
        self.commands = []
        self.command = 0
        self._rotate_stop_event = threading.Event()
        self._read_distance_feedback = self.sample
        self._rotate_mode_error = lambda: None
        self.failure = None
        self.extra_y = 0
        self.segment = 'trial'
        self.freeze_stamp = False
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.clock)
        monkeypatch.setattr(module.time, 'sleep', self.advance)

    def advance(self, duration):
        self.clock += duration
        if self.command:
            self.forward += self.speed * duration

    def sample(self):
        if self.failure and self.forward > 0.1:
            raise ValueError(self.failure)
        return {'position': (self.forward*math.cos(self.heading),
                             self.forward*math.sin(self.heading)+self.extra_y, 0),
                'yaw': self.heading, 'stamp_ns': int((1 if self.freeze_stamp else self.clock)*1e9),
                'segment_id': self.segment}

    def _distance_publish(self, vx):
        if vx and self._rotate_stop_event.is_set():
            return
        self.command = vx
        self.commands.append(vx)


@pytest.mark.parametrize('heading', [0, math.pi/2, -math.pi+0.01])
def test_distance_projects_onto_start_heading_and_settles(monkeypatch, heading):
    import distance_control as module
    plant = Plant(monkeypatch, module, heading=heading)
    result = run_distance(plant, 1.0)
    assert result['state'] == 'done'
    assert abs(result['final_error_m']) <= 0.05
    assert result['moved_m'] == pytest.approx(plant.forward)
    assert plant.commands[-1] == 0
    assert plant.clock >= plant.forward / plant.speed + 1.5


@pytest.mark.parametrize('failure', ['lost', 'segment', 'frozen', 'stuck', 'mode', 'cancel', 'jump', 'lateral', 'heading'])
def test_distance_faults_brake_and_never_complete(monkeypatch, failure):
    import distance_control as module
    plant = Plant(monkeypatch, module, speed=0 if failure == 'stuck' else 0.3)
    original = plant.advance

    def advance(duration):
        original(duration)
        if plant.clock > 1.4:
            if failure == 'lost':
                plant.failure = 'tracking lost'
            elif failure == 'segment':
                plant.segment = 'reset'
            elif failure == 'frozen':
                plant.freeze_stamp = True
            elif failure == 'mode':
                plant._rotate_mode_error = lambda: {'state': 'error', 'error': 'protection'}
            elif failure == 'cancel':
                plant._rotate_stop_event.set()
            elif failure == 'jump':
                plant.forward += 1
            elif failure == 'lateral':
                plant.extra_y = 0.2
            elif failure == 'heading':
                plant.heading = math.pi/3
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = run_distance(plant, 1)
    assert result['state'] == ('stopped' if failure == 'cancel' else 'error')
    assert plant.commands[-1] == 0
    assert plant.clock < 6


def test_overshoot_is_reported_after_braking(monkeypatch):
    import distance_control as module
    plant = Plant(monkeypatch, module)
    original = plant.advance
    def advance(duration):
        original(duration)
        if not plant.command and plant.forward > 0.8:
            plant.forward += 0.01
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = run_distance(plant, 1)
    assert result['state'] == 'error'
    assert plant.commands[-1] == 0


def test_distance_timeout_brakes_and_reports(monkeypatch):
    # 规格"范围与验证"点名超时：唯一兜底，必须刹停并报错而非挂死或误报完成。
    import distance_control as module
    plant = Plant(monkeypatch, module, speed=0)
    monkeypatch.setattr(module, 'distance_timeout', lambda d: 0.05)
    result = run_distance(plant, 1)
    assert result['state'] == 'error'
    assert 'timed out' in result['error']
    assert plant.commands[-1] == 0


def test_distance_timestamp_going_backwards_brakes(monkeypatch):
    # 规格"时间倒退"：源时间回拨（dt<0）必须刹停报错，不得当作正常样本继续。
    import distance_control as module
    plant = Plant(monkeypatch, module)
    sample = plant.sample

    def rewinding():
        data = sample()
        if plant.forward > 0.3:
            data = dict(data, stamp_ns=data['stamp_ns'] - 10**9)
        return data
    plant._read_distance_feedback = rewinding
    result = run_distance(plant, 1)
    assert result['state'] == 'error'
    assert 'reset' in result['error']
    assert plant.commands[-1] == 0


def test_latched_fault_clears_only_on_healthy_revalidation(feedback):
    # 复核 Important#3：单帧坏消息锁存 reason 后，运动中必须持续拒绝（刹停
    # 契约）；仅当同 segment 最新 pose+status 均健康时，preflight 的 revalidate
    # 才清除锁存，让下一次 move 重试可用——无需重启融合进程。
    result, clock = feedback
    clock.ns += 50_000_000
    bad = pose(stamp=clock.ns)
    bad.pose.pose.position.x = float('nan')
    result.update_pose(bad)
    with pytest.raises(ValueError):
        result.snapshot()
    clock.ns += 50_000_000
    result.update_pose(pose(stamp=clock.ns))
    result.update_status({'status': 'tracking', 'segment_id': 'trial', 'stamp_ns': clock.ns})
    with pytest.raises(ValueError):
        result.snapshot()          # 锁存保持：流恢复不自动放行（运动中止契约）
    result.revalidate()
    assert result.snapshot()['segment_id'] == 'trial'


def test_revalidate_keeps_fault_while_stream_unhealthy(feedback):
    result, clock = feedback
    # 单帧坏消息锁存
    clock.ns += 50_000_000
    bad = pose(stamp=clock.ns)
    bad.pose.pose.position.x = float('nan')
    result.update_pose(bad)
    with pytest.raises(ValueError):
        result.snapshot()
    # 流恢复 tracking 但接收时效已过期：revalidate 的健康门不得放行
    clock.ns += 50_000_000
    result.update_pose(pose(stamp=clock.ns))
    result.update_status({'status': 'tracking', 'segment_id': 'trial', 'stamp_ns': clock.ns})
    clock.mono += 1.0
    result.revalidate()
    with pytest.raises(ValueError):
        result.snapshot()          # 锁存仍在：过期样本不算健康
    # 新鲜的 tracking 样本到位后才放行
    clock.ns += 50_000_000
    result.update_pose(pose(stamp=clock.ns))
    result.update_status({'status': 'tracking', 'segment_id': 'trial', 'stamp_ns': clock.ns})
    result.revalidate()
    assert result.snapshot()['segment_id'] == 'trial'
