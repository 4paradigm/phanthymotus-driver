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


@pytest.mark.parametrize('yaw', [0, math.pi/2, -math.pi+0.01])
def test_move_calibrates_then_brakes_and_reports_an_estimate(monkeypatch, yaw):
    plant = Plant(monkeypatch, yaw)
    result = module.run_distance(plant, 1)
    assert result['state'] == 'done', result
    assert result['feedback_source'] == 'imu_integrated_experimental'
    assert result['distance_source'] == 'imu_integrated'
    assert result['accuracy_validated'] is False
    assert result['estimated_distance_m'] == pytest.approx(plant.distance, abs=0.005)
    assert abs(result['estimated_error_m']) <= 0.05
    assert plant.commands[-1][1:] == (0, 0)
    assert min(t for t, vx, _ in plant.commands if vx) >= 2
    assert result['stationary_assumed'] is True


@pytest.mark.parametrize('fault', ['read', 'mode', 'frozen', 'cancel', 'gap', 'heading'])
def test_fault_and_cancel_paths_finish_with_brake(monkeypatch, fault):
    plant = Plant(monkeypatch)
    if fault in ('read', 'mode', 'frozen'):
        plant.fault = fault
    advance = plant.advance
    def steered(dt):
        advance(dt)
        if plant.distance > 0.1:
            if fault == 'cancel':
                plant._rotate_stop_event.set()
            elif fault == 'gap':
                plant.clock += 0.2
            elif fault == 'heading':
                plant.yaw = 0.5
    monkeypatch.setattr(module.time, 'sleep', steered)
    result = module.run_distance(plant, 1)
    assert result['state'] == ('stopped' if fault == 'cancel' else 'error'), result
    assert plant.commands[-1][1:] == (0, 0)
    assert plant.clock < 10


def test_acceleration_limit_during_motion_brakes_and_retains_rejected_sample(monkeypatch):
    plant = Plant(monkeypatch)
    read = plant._read_distance_imu
    def shock():
        return imu(acc=(51, 0, 9.81)) if plant.distance > 0.1 else read()
    monkeypatch.setattr(plant, '_read_distance_imu', shock)
    result = module.run_distance(plant, 0.5)
    assert result['state'] == 'error'
    assert 'IMU acceleration norm' in result['error']
    assert not result['stationary_assumed']
    assert plant.commands[-1][1:] == (0, 0)
    trace = json.loads(Path(result['imu_trace_path']).read_text())
    assert trace['samples'][-1]['linear_acceleration_mps2'] == [51, 0, 9.81]
    assert trace['result']['state'] == 'error'


@pytest.mark.parametrize('direction', ['forward', 'backward', 'left', 'right'])
def test_estimated_reverse_progress_aborts_instead_of_continuing_command(monkeypatch, direction):
    plant = Plant(monkeypatch)
    _, x, y = module.movement_direction(direction)
    def reversed_feedback():
        plant.samples += 1
        acceleration = -1 if plant.command else 0
        return imu(acc=(acceleration*x, acceleration*y, 9.81+1e-7*math.sin(plant.samples)))
    monkeypatch.setattr(plant, '_read_distance_imu', reversed_feedback)
    # This fake checks commands and the estimator only, not physical accuracy.
    def publish(vx, vyaw=0, *, vy=0):
        plant.command = abs(vx)+abs(vy)
        plant.commands.append((plant.clock, vx, vy, vyaw))
    monkeypatch.setattr(plant, '_distance_publish', publish)
    result = module.run_distance(plant, 0.5, direction)
    assert result['state'] == 'error'
    assert 'opposite to requested direction' in result['error']
    assert result['estimated_distance_m'] < -0.05
    assert plant.clock < 3
    assert plant.commands[-1][1:] == (0, 0, 0)


def test_cancel_during_calibration_never_starts_motion(monkeypatch):
    plant = Plant(monkeypatch)
    plant._rotate_stop_event.set()
    result = module.run_distance(plant, 1)
    assert result['state'] == 'stopped'
    assert all(vx == vyaw == 0 for _, vx, vyaw in plant.commands)


def test_quiet_acceleration_alone_cannot_certify_stationary_endpoint(monkeypatch):
    plant = Plant(monkeypatch)
    advance = plant.advance
    def coast(dt):
        if plant.distance > 0.8 and not plant.command:
            plant.clock += dt
            plant.distance += plant.velocity*dt
            plant.acceleration = 0
        else:
            advance(dt)
    monkeypatch.setattr(module.time, 'sleep', coast)
    result = module.run_distance(plant, 1)
    assert result['state'] == 'error', result
    assert plant.commands[-1][1:] == (0, 0)


def test_whole_request_timeout_brakes_even_before_calibration(monkeypatch):
    plant = Plant(monkeypatch)
    monkeypatch.setattr(module, 'distance_timeout', lambda distance: 0.5)
    result = module.run_distance(plant, 1)
    assert result['state'] == 'error'
    assert 'timed out' in result['error']
    assert all(vx == vyaw == 0 for _, vx, vyaw in plant.commands)


class DirectionalPlant(Plant):
    """Two independent body axes with bounded acceleration, no yaw actuation."""
    def __init__(self, monkeypatch, yaw):
        super().__init__(monkeypatch, yaw)
        self.body_velocity = [0., 0.]
        self.body_acceleration = [0., 0.]
        self.body_position = [0., 0.]
        self.command = (0., 0.)

    def advance(self, dt):
        self.clock += dt
        for axis in (0, 1):
            old = self.body_velocity[axis]
            target = math.copysign(.3, self.command[axis]) if self.command[axis] else 0.
            new = old + max(-dt, min(dt, target-old))
            self.body_velocity[axis] = new
            self.body_acceleration[axis] = (new-old)/dt
            self.body_position[axis] += (old+new)*dt/2

    def _read_distance_imu(self):
        self.samples += 1
        ax, ay = self.body_acceleration
        return imu(acc=(ax, ay, 9.81+1e-7*math.sin(self.samples)), yaw=self.yaw)

    def _distance_publish(self, vx, vyaw=0, *, vy=0):
        if (vx or vy or vyaw) and self._rotate_stop_event.is_set():
            return
        self.command = (vx, vy)
        self.commands.append((self.clock, vx, vy, vyaw))


@pytest.mark.parametrize('direction,expected', [
    ('前进', (1, 0)), ('后退', (-1, 0)), ('左移', (0, 1)), ('右移', (0, -1)),
    ('forward', (1, 0)), ('backward', (-1, 0)), ('left', (0, 1)), ('right', (0, -1)),
    ('', (1, 0)),
])
@pytest.mark.parametrize('yaw', [0., 1.2, -2.8])
def test_direction_moves_along_requested_body_axis_and_brakes(monkeypatch, direction, expected, yaw):
    plant = DirectionalPlant(monkeypatch, yaw)
    result = module.run_distance(plant, 1., direction=direction)
    assert result['state'] == 'done', result
    assert result['estimated_distance_m'] == pytest.approx(1., abs=.05)
    assert plant.body_position == pytest.approx([expected[0], expected[1]], abs=.05)
    for _, vx, vy, turn in plant.commands:
        assert vx*expected[0] >= 0 and vy*expected[1] >= 0
        assert (vy if expected[0] else vx) == 0
        assert turn == pytest.approx(0.)
    assert plant.commands[-1][1:] == (0, 0, 0)


def test_distance_uses_leg_odometry_when_the_owner_exposes_joint_states(monkeypatch):
    """Production LocoPlugin exposes get_joint_state(); the displacement must
    then come from the leg estimate, not from the integrated IMU."""
    plant = Plant(monkeypatch)
    class FakeLeg:
        def __init__(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self._last = None
        def reset_position(self):
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        def observe(self, joints, quaternion, now):
            if self._last is None:
                self._last = now
                return
            dt = now-self._last
            self._last = now
            step = 0.16*dt if plant.command else 0.0
            self.position = [self.position[0]+step, self.position[1], self.position[2]]
            self.velocity = [step/dt, 0.0, 0.0]
    monkeypatch.setattr(module, 'LegOdometry', FakeLeg)
    monkeypatch.setattr(plant, '_read_distance_joints', lambda: [0.0]*21, raising=False)
    result = module.run_distance(plant, 1.0)
    assert result['state'] == 'done', result
    assert result['distance_source'] == 'leg_odometry'
    assert result['estimated_distance_m'] == pytest.approx(1.0, abs=0.06)
    assert 'imu_position_m' in result
    assert plant.commands[-1][1:] == (0, 0)


def test_distance_trace_keeps_raw_control_samples_commands_and_terminal_error(monkeypatch):
    plant = Plant(monkeypatch)
    plant.fault = 'frozen'
    result = module.run_distance(plant, 1.)
    assert result['state'] == 'error'
    trace = json.loads(Path(result['imu_trace_path']).read_text())
    assert trace['result']['error'] == result['error']
    assert trace['samples'][0]['orientation_xyzw'] == [0, 0, 0, 1]
    assert trace['samples'][0]['linear_acceleration_mps2'] == [0, 0, 9.81]
    assert trace['samples'][0]['angular_velocity_rad_s'] == [0, 0, 0]
    assert all(b['received_mono_s'] > a['received_mono_s']
               for a, b in zip(trace['samples'], trace['samples'][1:]))
    assert trace['command_requests'][-1]['velocity'] == [0, 0, 0]


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


def test_lateral_limit_scales_with_the_requested_distance():
    assert module.lateral_limit(0.2) == pytest.approx(0.15)
    assert module.lateral_limit(0.8) == pytest.approx(0.40)
    assert module.lateral_limit(3.0) == pytest.approx(1.50)


def test_lagging_imu_stops_with_an_error_instead_of_dead_reckoning(monkeypatch):
    """The IMU cannot see the motion; a command-speed model cannot observe the
    real displacement either, so the worker must brake and report a fault."""
    plant = Plant(monkeypatch)
    def quiet():
        plant.samples += 1
        return imu(acc=(0, 0, 9.81+1e-7*math.sin(plant.samples)))
    monkeypatch.setattr(plant, '_read_distance_imu', quiet)
    result = module.run_distance(plant, 0.5)
    assert result['state'] == 'error', result
    assert 'dead-reckoned travel' in result['error']
    assert result['dead_reckoned_m'] > 0.1
    assert result['estimated_distance_m'] == pytest.approx(0.0, abs=0.01)
    assert result['accuracy_validated'] is False
    assert plant.commands[-1][1:] == (0, 0)
    trace = json.loads(Path(result['imu_trace_path']).read_text())
    assert any('dead_reckoned_m' in sample for sample in trace['samples'])
    assert trace['result']['state'] == 'error'


def test_imu_under_reporting_stops_with_an_error(monkeypatch):
    """An IMU that sees only a fraction of the motion still progresses, so the
    no-progress check cannot catch it; the cross-check must brake and fault."""
    plant = Plant(monkeypatch)
    def weak():
        plant.samples += 1
        return imu(acc=(0.2*plant.acceleration, 0,
                        9.81+1e-7*math.sin(plant.samples)))
    monkeypatch.setattr(plant, '_read_distance_imu', weak)
    result = module.run_distance(plant, 1.)
    assert result['state'] == 'error', result
    assert 'dead-reckoned travel' in result['error']
    assert result['dead_reckoned_m'] > result['estimated_distance_m']
    assert plant.commands[-1][1:] == (0, 0)
