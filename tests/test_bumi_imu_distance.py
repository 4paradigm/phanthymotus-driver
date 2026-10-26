"""Inertial experiment: analytical trajectories and hardware-free stopping paths."""
import math
from pathlib import Path
import sys
import threading
import json
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'noetix' / 'bumi'))
import imu_distance_control as module


def imu(acc=(0, 0, 9.81), yaw=0, pitch=0, gyro=(0, 0, 0)):
    return NS(ori=(-math.sin(pitch/2)*math.sin(yaw/2),
                   math.sin(pitch/2)*math.cos(yaw/2),
                   math.cos(pitch/2)*math.sin(yaw/2),
                   math.cos(pitch/2)*math.cos(yaw/2)),
              linear_acc=acc, angular_vel=gyro)


def calibrated():
    estimator = module.ImuOdometry()
    for index in range(101):
        estimator.observe(imu(), index*0.02)
    assert estimator.ready
    return estimator


def test_constant_acceleration_integrates_velocity_and_distance_without_auto_zero():
    estimator = calibrated()
    # Linear ramp from 0 to 1 m/s² over the first 20 ms, then constant.
    for index in range(1, 51):
        estimator.observe(imu(acc=(1, 0, 9.81)), 2+index*0.02)
    assert estimator.velocity == pytest.approx((0.99, 0, 0))
    assert estimator.position == pytest.approx((0.4901, 0, 0))
    # Zero acceleration during uniform motion must NOT zero its velocity.
    for index in range(1, 51):
        estimator.observe(imu(), 3+index*0.02)
    assert estimator.velocity == pytest.approx((1, 0, 0))
    assert estimator.position == pytest.approx((1.49, 0, 0))


def test_gravity_rotation_and_initial_bias_do_not_create_translation():
    estimator = module.ImuOdometry()
    pitch = math.radians(10)
    raw = imu(acc=(-9.81*math.sin(pitch), 0, 9.81*math.cos(pitch)), pitch=pitch)
    for index in range(201):
        estimator.observe(raw, index*0.02)
    assert estimator.ready
    assert estimator.position == pytest.approx((0, 0, 0), abs=1e-9)
    assert estimator.velocity == pytest.approx((0, 0, 0), abs=1e-9)


@pytest.mark.parametrize('bad', [imu(acc=(0, 0, 1)), imu(acc=(0, 0, 0)),
                                 imu(acc=(float('nan'), 0, 9.81)),
                                 NS(ori=(0, 0, 0, 0), linear_acc=(0, 0, 9.81), angular_vel=(0, 0, 0))])
def test_invalid_or_unverified_acceleration_units_cannot_calibrate(bad):
    estimator = module.ImuOdometry()
    with pytest.raises(ValueError):
        for index in range(302):
            estimator.observe(bad, index*0.02)
    assert not estimator.ready


@pytest.mark.parametrize('sample,message', [
    (imu(acc=(51, 0, 0)), 'IMU acceleration norm 51.000 exceeds experimental limit 50'),
    (NS(ori=(0, 0, 0, 0), linear_acc=(0, 0, 9.81), angular_vel=(0, 0, 0)),
     'IMU quaternion norm 0.000 outside experimental range'),
])
def test_imu_limit_error_identifies_the_rejected_quantity(sample, message):
    with pytest.raises(ValueError, match=message):
        module.decode_imu(sample)


def test_calibration_requires_a_continuous_quiet_window():
    estimator = module.ImuOdometry()
    for index in range(200):
        estimator.observe(imu(gyro=(0, 0, 0.2 if index % 20 == 0 else 0)), index*0.02)
    assert not estimator.ready


@pytest.mark.parametrize('period', [0.0205, 0.025, 0.05, 0.09])
def test_calibration_uses_elapsed_time_despite_sdk_and_scheduler_overhead(period):
    estimator = module.ImuOdometry()
    for index in range(math.ceil(2.1/period)+1):
        estimator.observe(imu(), index*period)
    assert estimator.ready


@pytest.mark.parametrize('stamp', [1.9, 2.0, 2.2])
def test_bad_sampling_intervals_abort_integration(stamp):
    estimator = calibrated()
    with pytest.raises(ValueError, match='sampling'):
        estimator.observe(imu(), stamp)


class Plant:
    """Independent 1D plant: acceleration/deceleration are bounded at 1 m/s²."""
    def __init__(self, monkeypatch, yaw=0):
        self.clock = 0.0
        self.velocity = self.distance = self.acceleration = 0.0
        self.yaw = yaw
        self.command = 0
        self.commands = []
        self.samples = 0
        self.fault = None
        self._rotate_stop_event = threading.Event()
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.clock)
        monkeypatch.setattr(module.time, 'sleep', self.advance)

    def advance(self, dt):
        self.clock += dt
        target = 0.3 if self.command else 0
        old = self.velocity
        self.velocity += max(-dt, min(dt, target-old))
        self.acceleration = (self.velocity-old)/dt
        self.distance += (old+self.velocity)*dt/2

    def _read_distance_imu(self):
        self.samples += 1
        if self.fault == 'read' and self.distance > 0.1:
            raise RuntimeError('SDK read failed')
        # Tiny changing sensor noise also lets the controller detect a frozen SDK cache.
        noise = 1e-7*math.sin(self.samples)
        acc = self.acceleration if self.fault != 'frozen' else 0
        return imu(acc=(acc, noise if self.fault != 'frozen' else 0, 9.81), yaw=self.yaw)

    def _distance_publish(self, vx, vyaw=0):
        if (vx or vyaw) and self._rotate_stop_event.is_set():
            return
        self.command = vx
        self.commands.append((self.clock, vx, vyaw))

    def _rotate_mode_error(self):
        return {'error': 'protection mode'} if self.fault == 'mode' and self.distance > 0.1 else None


def test_target_walks_and_turns_inside_the_turn_first_band(monkeypatch):
    clock = [0.0]
    commands = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx
            commands.append((vx, vyaw))

    owner = Owner()
    bearing = math.atan2(0.5, 1.0)

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self._last = None
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def observe(self, joints, quaternion, now, frozen=False):
            if frozen:
                self.velocity = [0.0, 0.0, 0.0]
                return
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            step = 0.16*dt if owner.command else 0.0
            self.position = [self.position[0]+step*math.cos(bearing),
                             self.position[1]+step*math.sin(bearing),
                             self.position[2]]
            self.velocity = [step*math.cos(bearing)/dt, step*math.sin(bearing)/dt, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 1.0, 0.5)   # 26.6 deg: inside the turn-first band
    driving = [(vx, vyaw) for vx, vyaw in commands if vx or vyaw]
    assert driving, commands
    # It must walk forward while turning, not stop and turn in place first.
    assert any(vx > 0 for vx, vyaw in driving[:5]), driving[:5]


def test_target_exactly_forty_five_degrees_still_walks(monkeypatch):
    clock = [0.0]
    commands = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx
            commands.append((vx, vyaw))

    owner = Owner()
    bearing = math.atan2(1.0, 1.0)   # exactly 45 deg: inside the walk band

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self._last = None
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def observe(self, joints, quaternion, now, frozen=False):
            if frozen:
                self.velocity = [0.0, 0.0, 0.0]
                return
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            step = 0.16*dt if owner.command else 0.0
            self.position = [self.position[0]+step*math.cos(bearing),
                             self.position[1]+step*math.sin(bearing),
                             self.position[2]]
            self.velocity = [step*math.cos(bearing)/dt, step*math.sin(bearing)/dt, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 1.0, 1.0)
    driving = [(vx, vyaw) for vx, vyaw in commands if vx or vyaw]
    assert driving, commands
    # The boundary belongs to the walk: forward commands must be present.
    assert any(vx > 0 for vx, vyaw in driving[:5]), driving[:5]


def test_target_turns_first_for_a_large_bearing(monkeypatch):
    clock = [0.0]
    commands = []
    frozen_seen = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            commands.append((vx, vyaw))

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        def observe(self, joints, quaternion, now, frozen=False):
            frozen_seen.append(frozen)
            self.velocity = [0.0, 0.0, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.0, 0.5)   # bearing 90 deg left
    driving = [(vx, vyaw) for vx, vyaw in commands if vx or vyaw]
    assert driving, commands
    # The first motion is an in-place left turn, not a forward arc.
    assert driving[0] == (0.0, pytest.approx(0.8))
    assert any(frozen_seen)  # the leg is frozen while turning in place
    assert result['state'] in ('error', 'stopped', 'done')


def test_target_heading_drift_is_corrected_away_from_the_drift(monkeypatch):
    clock = [0.0]
    commands = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            # The robot drifts 10 deg left while walking toward a target ahead.
            yaw = 0.0 if clock[0] < 2.1 else math.radians(10)
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=yaw)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx
            commands.append((vx, vyaw))

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self._last = None
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def observe(self, joints, quaternion, now, frozen=False):
            if frozen:
                self.velocity = [0.0, 0.0, 0.0]
                return
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            step = 0.16*dt if owner.command else 0.0
            self.position = [self.position[0]+step, self.position[1], self.position[2]]
            self.velocity = [step/dt, 0.0, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.5, 0.0)
    assert result['state'] == 'done', result.get('error')
    # Drifting left must command a right correction (negative yaw), not another
    # left turn. The stale sign would reinforce the drift; assert the exact
    # direction: a correction is issued and no left (positive) turn appears.
    driving = [(vx, vyaw) for vx, vyaw in commands if vx or vyaw]
    assert any(vyaw < 0 for vx, vyaw in driving), driving
    assert not any(vyaw > 0 for vx, vyaw in driving), driving


def test_target_endpoint_catches_coasting_past_the_tolerance(monkeypatch):
    clock = [0.0]

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self._last = None
            self._slid = 0.0
            self._moved = False
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
            self._slid = 0.0
            self._moved = False
        def observe(self, joints, quaternion, now, frozen=False):
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            if owner.command:
                self._moved = True
                step = 0.16*dt
            elif self._moved and self._slid < 0.45:
                # Coast 45 cm after the brake: the pre-brake position is within
                # tolerance, the post-brake one is outside the 20 cm endpoint.
                step = 0.16*dt
                self._slid += step
            else:
                step = 0.0
            self.position = [self.position[0]+step, self.position[1], self.position[2]]
            self.velocity = [step/dt, 0.0, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.5, 0.0)
    # The coast takes the robot beyond tolerance; the endpoint must see the
    # post-brake position, not the pre-brake one.
    assert result['state'] == 'error'
    assert 'outside the target point' in result['error']


def test_target_fails_closed_when_pose_feedback_is_frozen(monkeypatch):
    clock = [0.0]

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
        def reset_baseline(self):
            pass
        def observe(self, joints, quaternion, now, frozen=False):
            pass

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.5, 0.0)
    # The pose never moves while forward commands are sent: the independent
    # command model must fail the target closed, not drive on and report done.
    assert result['state'] == 'error'
    assert 'dead-reckoned' in result['error']


def test_target_enforces_the_execution_deadline(monkeypatch):
    clock = [0.0]

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self._last = None
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def observe(self, joints, quaternion, now, frozen=False):
            if frozen:
                self.velocity = [0.0, 0.0, 0.0]
                return
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            # Very slow but steady progress: avoids the 8 s no-progress guard
            # yet must hit min(60, 12+20*0.5) = 22 s.
            step = 0.02*dt if owner.command else 0.0
            self.position = [self.position[0]+step, self.position[1], self.position[2]]
            self.velocity = [step/dt, 0.0, 0.0]

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    # Isolate the deadline: with the normal bound the deliberately slow leg
    # would (correctly) trip the dead-reckoned guard first.
    monkeypatch.setattr(module, 'travel_tolerance', lambda distance: 1e9)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.5, 0.0)
    assert result['state'] == 'error'
    assert 'time limit' in result['error']


@pytest.mark.parametrize('cross,target_error,state', [
    (0.17, 0.192, 'done'),
    (0.20, 0.219, 'error'),
])
def test_target_off_axis_endpoint_acceptance_bound(monkeypatch, cross, target_error, state):
    clock = [0.0]

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            # On the along-track target + creep, but off-axis by `cross`.
            self.position = [0.59, cross, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
        def reset_baseline(self):
            pass
        def observe(self, joints, quaternion, now, frozen=False):
            pass

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.5, 0.0)
    if state == 'done':
        assert result['state'] == 'done', result.get('error')
        assert result['final_position_error_m'] == pytest.approx(target_error, abs=0.01)
    else:
        assert result['state'] == 'error'
        assert 'outside the target point' in result['error']


def test_target_reports_arrival_at_the_requested_point(monkeypatch):
    clock = [0.0]
    commands = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
        def _read_distance_imu(self):
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            commands.append((vx, vyaw))

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            # At the estimated target plus the 5 cm arrival creep (53.1 deg).
            self.position = [0.33, 0.44, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
        def reset_baseline(self):
            self.position = [0.33, 0.44, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        def observe(self, joints, quaternion, now, frozen=False):
            pass

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.3, 0.4)
    assert result['state'] == 'done', result.get('error')
    assert result['action'] == 'target'
    assert result['requested_target_x_m'] == 0.3
    assert result['requested_target_y_m'] == 0.4
    assert result['final_position_error_m'] == pytest.approx(0.05, abs=0.01)
    assert commands and commands[-1] == (0, 0)


def test_target_does_not_turn_when_already_facing_the_point(monkeypatch):
    clock = [0.0]
    commands = []

    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
        def _read_distance_imu(self):
            # Calibration is at yaw 0; by the first motion sample the robot has
            # turned 90 deg to face the target, which is 90 deg left.
            yaw = 0.0 if clock[0] < 2.1 else math.pi/2
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=yaw)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            commands.append((vx, vyaw))

    owner = Owner()

    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        def observe(self, joints, quaternion, now, frozen=False):
            pass

    monkeypatch.setattr(module, 'LegOdometry', Leg)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def advance(dt):
        clock[0] += dt
    monkeypatch.setattr(module.time, 'sleep', advance)
    result = module.run_target(owner, 0.0, 0.5)
    driving = [(vx, vyaw) for vx, vyaw in commands if vx or vyaw]
    assert driving, commands
    # The origin sample may still be at yaw 0 (a transient turn). Once the
    # robot faces the point (yaw 90 = bearing), a correct heading calculation
    # walks forward with no turn. With the old sign the error stays 180 deg
    # and the turn never returns to zero.
    assert any(vx > 0 and abs(vyaw) < 1e-9 for vx, vyaw in driving[1:]), commands


def _slow_calibration_owner(clock, gyro_until):
    class Owner:
        def __init__(self):
            self._rotate_stop_event = threading.Event()
            self.command = 0.0
        def _read_distance_imu(self):
            gyro = (0.2, 0.0, 0.0) if clock[0] < gyro_until else (0.0, 0.0, 0.0)
            return imu(acc=(0.0, 1e-7*math.sin(clock[0]*100), 9.81), yaw=0.0, gyro=gyro)
        def _read_distance_joints(self):
            return [0.0]*21
        def _rotate_mode_error(self):
            return None
        def _distance_publish(self, vx, vyaw=0):
            self.command = vx
    return Owner()


def _commanded_leg(owner):
    class Leg:
        def __init__(self, slip_scale=None):
            self.slip_scale = slip_scale
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.feet = []
            self.foot_gap = 0.0
            self.double_support = True
            self._last = None
        def reset_baseline(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def observe(self, joints, quaternion, now, frozen=False):
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            step = 0.16*dt if owner.command else 0.0
            self.position = [self.position[0]+step, self.position[1], self.position[2]]
            self.velocity = [step/dt, 0.0, 0.0]
    return Leg


def test_travel_scale_uses_the_measured_chassis_calibration():
    # README.md "标定 loco_servo": normalised 1.0 is 0.55 m/s forward and
    # 0.20 m/s sideways on this chassis.
    assert module.travel_scale('forward') == pytest.approx(0.55)
    assert module.travel_scale('backward') == pytest.approx(0.55)
    assert module.travel_scale('left') == pytest.approx(0.20)
    assert module.travel_scale('right', {'right': 0.3}) == pytest.approx(0.3)
    for bad in ({}, {'forward': 0}, {'forward': float('nan')}, {'forward': 'x'}):
        with pytest.raises(ValueError, match='calibrated|positive|finite'):
            module.travel_scale('forward', bad)


def test_travel_speed_respects_the_measured_deadband():
    # The legs do nothing below the normalised deadband; between the deadband
    # and 1.0 the speed rises linearly to the full scale.
    assert module.travel_speed(0.45, 'forward') == pytest.approx(0.0)
    assert module.travel_speed(0.30, 'forward') == pytest.approx(0.0)
    assert module.travel_speed(1.0, 'forward') == pytest.approx(0.55)
    assert module.travel_speed(0.60, 'forward') == pytest.approx(0.15)
    assert module.travel_speed(0.60, 'left') == pytest.approx(0.0)
    assert module.travel_speed(0.65, 'left') == pytest.approx(0.025)


