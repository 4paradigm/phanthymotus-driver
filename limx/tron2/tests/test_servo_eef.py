"""Exercise complete absolute EEF frames without a robot or a ROS graph."""
import asyncio
from contextlib import contextmanager
import json
import math
import time
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

from device import Adapter
from eef_kinematics import Kinematics, NoFeasibleStep
from eef_state import CONTROL_TOPIC, FRAME, SCHEMA, STATE_TOPIC
from servo_eef import EefServo
from vendor import MoveError


def message(values, *, seq=None, source="test", priority=50, ttl=1000, stamp=None, obs_stamp=None):
    payload = {"schema": "motus.control/1", "mode": "eef_pose", "dof": 16,
               "source": source, "priority": priority, "ttl_ms": ttl,
               "seq": time.monotonic_ns() if seq is None else seq,
               "stamp_ms": time.time_ns() // 1_000_000 if stamp is None else stamp,
               "values": values}
    if obs_stamp is not None:
        payload["obs_stamp_ms"] = obs_stamp
    return json.dumps(payload)


class Reader:
    configuration = {"mode": "upper_limb"}
    def __init__(self):
        self.ws = object()
        self.q = [0.] * 16
        self.sent = []
        self.first_write = asyncio.Event()
        self.send_gripper = AsyncMock(return_value={"result": "success"})

    def sample(self, name):
        return {"available": True, "data": {"working_mode": "developer_mode", "working_mode_valid": True,
                "motor": "OK", "collision_enable_valid": True, "collision_enable": True,
                "compliant_enable_valid": True, "compliant_enable": False}}

    @contextmanager
    def pose_feedback(self):
        yield

    @contextmanager
    def pose_servo(self, job):
        yield {"job": job}

    async def send_arm_servoj(self, target, token):
        self.sent.append(target)
        self.q = target
        self.first_write.set()


class Bus:
    def subscribe_json(self, topic, callback):
        self.topic, self.callback = topic, callback
        return callback

    def unsubscribe(self, subscription):
        self.callback = None


class SimulatedMotors(Reader):
    """A position servo with encoder, velocity and two jaw readings."""
    configuration = {'mode': 'upper_limb', 'state': 'identified'}
    connected, ws = True, object()
    move_pending = head_pending = _pose_servo = None
    last_round_trip = 0.

    def __init__(self, shoulder_bias=0.):
        super().__init__()
        self.q = [.2, .25, -.3, -.9, .15, .1, -.15,
                  .1, -.2, .3, -.8, .05, -.1, .2, .1, -.1]
        self.dq = [0.] * 16
        self.last_send = time.monotonic()
        self.shoulder_bias = shoulder_bias
        self.applied_bias = 0.

    def sample(self, name):
        if name != 'joint_state':
            return super().sample(name)
        return {'available': True, 'reception_age_seconds': 0., 'data': {
            'robot_timestamp': time.monotonic_ns(), 'q': self.q.copy(),
            'dq': self.dq.copy() if time.monotonic() - self.last_send < .05 else [0.] * 16}}

    async def read_gripper(self):
        return {'result': 'success', 'timestamp': time.monotonic_ns(),
                'left_opening': 40, 'left_speed': 0, 'left_force': 0,
                'right_opening': 56, 'right_speed': 0, 'right_force': 0}

    async def send_arm_servoj(self, target, token):
        now = time.monotonic()
        dt = max(now - self.last_send, .000001)
        # Bias appears gradually as the motor takes load. Report actual encoder
        # feedback while retaining the original wire commands for verification.
        self.applied_bias += (self.shoulder_bias - self.applied_bias) * (1 - math.exp(-dt / .18))
        measured = target.copy()
        measured[0] += self.applied_bias
        self.dq = [(a - b) / dt for a, b in zip(measured, self.q)]
        self.last_send = now
        await super().send_arm_servoj(target, token)
        self.q = measured


class WiredRoundTripTests(unittest.IsolatedAsyncioTestCase):
    async def test_full_frame_left_tcp_lift_and_return(self):
        await self.exercise_left_lift(SimulatedMotors(), return_to_start=True)

    async def test_encoder_reports_residual_after_exact_nominal_command_with_other_axes_held(self):
        # Shoulder pitch had ~+0.0083 rad residual in the stationary field capture.
        motors = SimulatedMotors(shoulder_bias=.0083)
        motors.q[:7] = [-1.4025, .4275, -2.2764, -.9871, 1.1352, .4776, -.5942]
        await self.exercise_left_lift(motors, return_to_start=False)

    async def exercise_left_lift(self, motors, *, return_to_start):
        bus = Bus()
        frames = asyncio.Queue()
        adapter = Adapter(lambda topic, packet: frames.put_nowait((topic, packet)),
                          {'enabled_cards': ['eef_state', 'servo_eef']}, reader=motors)
        adapter.servo_eef.publisher = bus
        sensor = asyncio.create_task(adapter.eef_state.run())

        async def measure(after=-1):
            async def read():
                while True:
                    topic, packet = await frames.get()
                    self.assertEqual(topic, STATE_TOPIC)
                    self.assertEqual((packet['schema'], packet['frame']), (SCHEMA, FRAME))
                    if packet['stamp_ms'] > after:
                        return packet['eef']
            return await asyncio.wait_for(read(), 1.)

        try:
            producer_topic = '/autonomous_task/control/eef'
            await adapter.execute_tool('eef_state', {'action': 'start'})
            await adapter.execute_tool('servo_eef', {'action': 'start', 'input_topics': [producer_topic]})
            self.assertEqual(bus.topic, producer_topic)
            original = await measure()
            self.assertEqual(len(original), 16)
            self.assertTrue(all(math.isfinite(x) for x in original))
            held = motors.q[7:].copy()
            raised = original[:7]
            raised[2] += .003
            targets = (raised, original[:7]) if return_to_start else (raised,)
            for target in targets:
                start = motors.q[:7].copy()
                nominal = adapter.eef_state.kinematics.solve('left', target[:3], target[3:], start)
                count = adapter.servo_eef.accepted + 1
                # A producer may copy feedback, set a TCP, and stream the absolute target.
                frame = await measure()
                command = frame.copy()
                command[:7] = target
                self.assertEqual(command[8:], frame[8:])
                bus.callback(message(command))

                async def wait_for_dispatch():
                    planned = {}
                    while True:
                        info = adapter.servo_eef.info()
                        self.assertEqual(info['state'], 'running', info['last'])
                        if info['accepted'] >= count:
                            planned.update(info['last']['planned_tcp'])
                            if not info['active_arms']:
                                return planned
                        self.assertEqual((await measure())[7:], original[7:])
                        bus.callback(message(command))

                planned = await asyncio.wait_for(wait_for_dispatch(), 4.)
                self.assertEqual(motors.sent[-1][:7], nominal)
                self.assertEqual(planned, {'left': adapter.eef_state.kinematics.forward('left', nominal)})
                sent = len(motors.sent)
                # A repeated observation still describes its original sampling instant.
                actual = await measure(after=time.time_ns() // 1_000_000)
                self.assertEqual(actual[:7], adapter.eef_state.kinematics.forward('left', motors.q[:7]))
                self.assertEqual(len(motors.sent), sent)
                if motors.shoulder_bias:
                    self.assertGreater(math.dist(actual[:3], target[:3]), .001)
                else:
                    self.assertLess(math.dist(actual[:3], target[:3]), .0001)
            self.assertGreater(len(motors.sent), 2)
            self.assertTrue(all(q[7:] == held for q in motors.sent))
            motors.send_gripper.assert_not_awaited()
        finally:
            await adapter.servo_eef.close()
            sensor.cancel()
            await asyncio.gather(sensor, return_exceptions=True)


class ServoTests(unittest.IsolatedAsyncioTestCase):
    POSE = [.02, 0., 0., 0., 0., 0., 1.]

    async def asyncSetUp(self):
        self.reader = Reader()
        self.speeds = [0.] * 16
        origin = [0.] * 6 + [1.]
        self.measured = origin + [.03] + origin + [.04]

        def solve(side, pos, quat, seed):
            if pos[0] > 1.:
                raise ValueError("unreachable")
            return [pos[0]] * 7
        self.observed = {}
        self.state = SimpleNamespace(observation=self.observed.get, values=self.measured.copy, joints=lambda: list(self.reader.q),
            joint_snapshot=lambda: (time.monotonic(), list(self.reader.q), self.speeds),
            kinematics=SimpleNamespace(solve=Mock(side_effect=solve),
                forward=lambda side, q: [q[0], 0., 0., 0., 0., 0., 1.]))
        self.servo = EefServo(SimpleNamespace(reader=self.reader,
            arm_servo=SimpleNamespace(_conflict=lambda token=None: None)), self.state)
        self.servo.publisher = Bus()
        await self.servo.call({"action": "start"})
        self.stamp = time.time_ns() // 1_000_000 - 50

    async def asyncTearDown(self):
        await self.servo.close()

    async def send(self, raw):
        count = self.servo.accepted + self.servo.rejected
        self.servo.publisher.callback(raw)
        for _ in range(400):
            if self.servo.accepted + self.servo.rejected > count or self.servo.paused:
                return
            await asyncio.sleep(.002)
        self.fail(self.servo.info())

    async def feed(self, values, **fields):
        await self.send(message(values, **fields))

    @contextmanager
    def delayed_ik(self):
        started, release = threading.Event(), threading.Event()
        solve = self.state.kinematics.solve.side_effect

        def wait_for_release(*args):
            started.set()
            if not release.wait(5.):
                raise RuntimeError("test solver timeout")
            return solve(*args)

        self.state.kinematics.solve.side_effect = wait_for_release
        try:
            yield started
        finally:
            release.set()
            self.state.kinematics.solve.side_effect = solve

    async def wait_for_end_of_stream(self, target=None, **fields):
        async def wait():
            refresh = time.monotonic() + .05
            while self.servo.info()['active_arms']:
                if target is not None and time.monotonic() >= refresh:
                    await self.feed(target, **fields)
                    refresh = time.monotonic() + .05
                await asyncio.sleep(.005)
        await asyncio.wait_for(wait(), 4.)

    async def echo_measurement(self, frame=None):
        if frame is None:
            frame = self.measured.copy()
        self.stamp = max(self.stamp, time.time_ns() // 1_000_000)
        observed = self.stamp
        self.observed[observed] = (frame.copy(), self.state.joints()[:14])
        await self.feed(frame, obs_stamp=observed)

    async def test_echoing_stationary_measurements_needs_no_motor_commands(self):
        frame = self.measured.copy()
        frame[6] = -1.  # The same measured orientation with the opposite sign.
        await self.feed(frame)
        self.assertEqual(self.servo.accepted, 1)
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()

        for drift in (.0001, -.0002):
            observation = self.measured.copy()
            self.measured[0] += drift
            self.measured[15] += drift
            await self.echo_measurement(observation)
        self.state.kinematics.solve.assert_not_called()
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()

    async def test_claw_target_reaches_driver_despite_later_drift_of_both_tcps(self):
        baseline_stamp = self.stamp - 20
        # Raw feedback can report 100 when the command protocol's nominal maximum is 94.
        self.measured[15] = .085 * 100 / 94
        frame = self.measured.copy()
        self.observed[baseline_stamp] = (frame.copy(), self.state.joints()[:14])
        self.measured[0], self.measured[8] = .0002, -.0001
        frame[7] += .01
        await self.feed(frame, obs_stamp=baseline_stamp)
        self.observed.clear()  # A fixed goal can outlive the sensor history.
        await self.feed(frame, obs_stamp=baseline_stamp)
        self.reader.send_gripper.assert_awaited_once()
        self.assertIn("left_opening", self.reader.send_gripper.await_args.args[0])
        self.state.kinematics.solve.assert_not_called()
        self.assertFalse(self.reader.sent)
        self.assertEqual(self.servo.accepted, 2)

    async def test_tiny_changed_tcp_is_solved_while_copied_peer_holds_after_drift(self):
        baseline_stamp = self.stamp - 20
        frame = self.measured.copy()
        self.observed[baseline_stamp] = (frame.copy(), self.state.joints()[:14])
        self.measured[0], self.measured[8] = -.0002, .0001
        frame[0] += .00005
        await self.feed(frame, obs_stamp=baseline_stamp)
        await self.wait_for_end_of_stream(frame, obs_stamp=baseline_stamp)
        self.state.kinematics.solve.assert_called_once()
        self.assertEqual(self.state.kinematics.solve.call_args.args[0], "left")
        self.assertEqual(self.reader.sent[-1], [.00005] * 7 + [0.] * 9)
        self.reader.send_gripper.assert_not_awaited()

    async def test_submillimetre_target_reaches_the_wire(self):
        self.assertNotIn('pose_tolerance', self.servo.info()['control_interface'])
        frame = self.measured.copy()
        frame[0] = .0005
        await self.feed(frame)
        await self.wait_for_end_of_stream(frame)
        self.assertEqual(self.reader.sent[-1], [.0005] * 7 + [0.] * 9)
        self.assertFalse(self.servo.info()['paused'])

    async def test_new_frame_replaces_active_absolute_targets(self):
        first = self.measured.copy()
        first[:7], first[8:15] = self.POSE, self.POSE
        await self.feed(first)
        self.assertEqual(set(self.servo.info()['active_arms']), {'left', 'right'})
        second = self.measured.copy()
        second[8:15] = self.POSE
        await self.feed(second)
        await self.wait_for_end_of_stream(second)
        self.assertEqual(self.reader.sent[-1], [0.] * 7 + [.02] * 7 + [0.] * 2)
        self.reader.send_gripper.assert_not_awaited()

    async def test_gap_bounds_and_zero_as_a_target(self):
        frame = self.measured.copy()
        frame[7] = .100
        await self.feed(frame)
        self.assertEqual(self.reader.send_gripper.await_args.args[0],
                         {"left_opening": 94., "left_speed": 15, "left_force": 30})
        self.assertEqual(self.servo.last["clamped_opening_m"], {"left": .085})
        frame[7] = 0.
        await self.feed(frame)
        self.assertEqual(self.reader.send_gripper.await_args.args[0],
                         {"left_opening": 0., "left_speed": 15, "left_force": 30})
        self.assertFalse(self.reader.sent)
        frame[7] = -.01
        await self.feed(frame)
        self.assertEqual(self.servo.last["clamped_opening_m"], {"left": 0.})
        self.assertEqual(self.reader.send_gripper.await_count, 2)

    async def test_gripper_can_replace_an_unfinished_goal_with_its_current_opening(self):
        frame = self.measured.copy()
        frame[7] += .01
        await self.feed(frame)
        await self.feed(self.measured.copy())
        self.assertEqual(self.reader.send_gripper.await_count, 2)
        last_command = self.reader.send_gripper.await_args.args[0]
        self.assertAlmostEqual(last_command['left_opening'], .03 / .085 * 94, places=5)
        self.assertFalse(self.reader.sent)

    async def test_gripper_cancel_replaces_inflight_opening_once(self):
        frame = self.measured.copy()
        frame[7] += .02
        await self.feed(frame)
        self.measured[7] += .005  # Only part of the opening has been executed.
        await self.echo_measurement()
        last_command = self.reader.send_gripper.await_args.args[0]
        self.assertAlmostEqual(last_command['left_opening'], self.measured[7] / .085 * 94, places=5)
        for drift in (.0002, -.0001):
            self.measured[7] += drift
            await self.echo_measurement()
        self.assertEqual(self.reader.send_gripper.await_count, 2)
        self.assertFalse(self.reader.sent)

    async def check_cancel_holds_measured_joints(self, *, after_final_write):
        frame = self.measured.copy()
        frame[0] = .0005 if after_final_write else .025
        await self.feed(frame)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        if after_final_write:
            await self.wait_for_end_of_stream(frame)
        else:
            self.assertEqual(self.servo.info()['active_arms'], ['left'])

        # The fake FK depends on the first joint. The remaining joints expose
        # an unnecessary IK solve even if it would yield the same TCP.
        measured_q = self.reader.q.copy()
        measured_q[0] -= .00004
        measured_q[1] += .00005
        self.measured[0] = measured_q[0]
        held_frame = self.measured.copy()
        self.stamp = time.time_ns() // 1_000_000
        observed = self.stamp
        self.observed[observed] = (held_frame.copy(), measured_q[:14])
        # The SDK encoder has advanced by the time this observation is echoed.
        live = measured_q.copy()
        live[0] += .00008
        self.state.joints = live.copy
        self.state.joint_snapshot = lambda: (time.monotonic(), live.copy(), self.speeds)
        self.measured[0] = live[0]
        await self.feed(held_frame, obs_stamp=observed)
        self.assertEqual(self.servo.last['status'], 'accepted', self.servo.last)
        self.assertEqual(self.servo.last['planned_tcp'], {'left': held_frame[:7]})
        await self.wait_for_end_of_stream(held_frame, obs_stamp=observed)
        self.assertEqual(self.reader.sent[-1], measured_q)
        self.state.kinematics.solve.assert_called_once()
        writes = len(self.reader.sent)
        self.observed.clear()
        self.stamp = time.time_ns() // 1_000_000
        await self.feed(held_frame, obs_stamp=observed)
        self.assertEqual(len(self.reader.sent), writes)
        self.measured[0] += .0001
        await self.echo_measurement()
        self.assertEqual(len(self.reader.sent), writes)
        self.state.kinematics.solve.assert_called_once()
        self.reader.send_gripper.assert_not_awaited()

    async def test_cancel_holds_measured_joints_during_interpolation(self):
        await self.check_cancel_holds_measured_joints(after_final_write=False)

    async def test_cancel_holds_measured_joints_after_final_write(self):
        await self.check_cancel_holds_measured_joints(after_final_write=True)

    async def test_field_boundary_holds_left_tcp_and_still_opens_jaw(self):
        # Captured at the failed combined +50 mm XYZ / +1 m opening request.
        self.reader.q[:7] = [-1.5459002, .1120002, -2.6788998, -.8249000,
                             1.3917000, .7498000, -.7510002]
        kinematics = Kinematics()
        self.state.kinematics = Mock(wraps=kinematics)
        self.measured[:7] = kinematics.forward('left', self.reader.q[:7])
        self.measured[7] = .085 * 93 / 94
        target = self.measured.copy()
        target[:3] = [x + .05 for x in target[:3]]
        target[7] += 1.
        for count in (1, 2):
            await self.feed(target)
            info = self.servo.info()
            self.assertEqual(info['accepted'], count, info['last'])
            self.assertEqual(info['last']['held_at_boundary'], ['left'])
            self.assertEqual(info['last']['clamped_opening_m'], {'left': .085})
            self.assertFalse(info['active_arms'])
            self.assertFalse(self.reader.sent)
        self.state.kinematics.solve.assert_called_once()
        self.reader.send_gripper.assert_awaited_once()
        self.assertEqual(self.reader.send_gripper.await_args.args[0]['left_opening'], 94.)

        # The same measured configuration permits a 2 mm lift.
        target[:7] = self.measured[:7]
        target[2] += .002
        await self.feed(target)
        await self.wait_for_end_of_stream(target)
        self.assertEqual(self.servo.info()['last']['held_at_boundary'], [])
        self.assertEqual(self.state.kinematics.solve.call_count, 2)
        actual = kinematics.forward('left', self.reader.sent[-1][:7])
        self.assertLess(math.dist(actual[:3], target[:3]), .00001)
        self.assertTrue(all(q[7:] == [0.] * 9 for q in self.reader.sent))
        self.reader.send_gripper.assert_awaited_once()

    async def test_boundary_replaces_inflight_goal_while_other_arm_can_move(self):
        target = self.measured.copy()
        target[0] = .2
        await self.feed(target)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        self.assertEqual(self.servo.info()['active_arms'], ['left'])

        solve = self.state.kinematics.solve.side_effect
        def blocked_left(side, *args):
            if side == 'left':
                raise NoFeasibleStep('no feasible command')
            return solve(side, *args)

        self.state.kinematics.solve.side_effect = blocked_left
        target[0], target[8] = -.1, .015
        await self.feed(target)
        self.assertEqual(self.servo.info()['last']['held_at_boundary'], ['left'])
        self.assertEqual(self.servo.accepted, 2)
        await self.wait_for_end_of_stream(target)
        self.assertLess(abs(self.reader.sent[-1][0]), .01)
        self.assertEqual(self.reader.sent[-1][7:14], [.015] * 7)
        self.reader.send_gripper.assert_not_awaited()

    async def test_gripper_target_is_submitted_again_on_a_new_robot_connection(self):
        frame = self.measured.copy()
        frame[7] += .01
        await self.feed(frame)
        self.reader.ws = object()
        await self.feed(frame)
        self.assertEqual(self.servo.accepted, 2)
        self.assertEqual(self.reader.send_gripper.await_count, 2)
        first, second = self.reader.send_gripper.await_args_list
        self.assertEqual(first, second)
        self.assertIn('left_opening', second.args[0])

    async def test_feedback_lost_after_first_gripper_ack_prevents_second_write(self):
        def unavailable():
            raise ValueError("Fresh gripper openings are required")

        async def send_first(command):
            self.state.values = unavailable
            return {"result": "success"}

        self.reader.send_gripper.side_effect = send_first
        frame = self.measured.copy()
        frame[7], frame[15] = .02, .06
        await self.feed(frame)
        self.reader.send_gripper.assert_awaited_once()
        self.assertIn('left_opening', self.reader.send_gripper.await_args.args[0])
        self.assertTrue(self.servo.paused)
        self.assertEqual(self.servo.last["status"], "fault")
        self.assertEqual(self.servo.accepted, 0)

    async def test_invalid_frame_rejects_all_new_targets_and_preserves_previous(self):
        previous = self.measured.copy()
        previous[:7] = self.POSE
        await self.feed(previous)
        original = self.servo.arms.copy()
        for right in ([2.] + self.POSE[1:], [0.] * 7):
            frame = self.measured.copy()
            frame[8:15], frame[7] = right, .04
            await self.feed(frame)
        self.assertEqual(self.servo.arms, original)
        self.reader.send_gripper.assert_not_awaited()
        self.assertEqual(self.servo.rejected, 2)
        self.assertFalse(self.servo.paused)

    async def test_complete_feedback_is_required_before_output(self):
        def unavailable():
            raise ValueError("End-effector feedback unavailable")
        self.state.values = unavailable
        frame = self.measured.copy()
        frame[:7], frame[7] = self.POSE, .05
        await self.feed(frame)
        self.assertEqual(self.servo.rejected, 1)
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()

    async def test_repeated_goal_with_opposite_quaternion_sign_reuses_trajectory(self):
        frame = self.measured.copy()
        frame[:7] = self.POSE
        await self.feed(frame)
        self.assertEqual(self.servo.info()['active_arms'], ['left'])
        frame[6] = -1.
        await self.feed(frame)
        self.state.kinematics.solve.assert_called_once()
        self.assertEqual(self.servo.accepted, 2)

    async def test_long_ik_uses_new_stamp_for_same_goal_or_discards_superseded_goal(self):
        for changed in (False, True):
            frame = self.measured.copy()
            frame[0] = .023 if changed else .012
            # The original frame has less time left than the test solve takes.
            with self.delayed_ik() as started:
                self.servo.publisher.callback(message(frame, stamp=time.time_ns() // 1_000_000 - 950))
                self.assertTrue(await asyncio.to_thread(started.wait, 1))
                await asyncio.sleep(.08)
                final = frame.copy()
                if changed:
                    final[0] = .01
                self.servo.publisher.callback(message(final))
                await asyncio.sleep(.005)
            await asyncio.sleep(.05)
            await self.wait_for_end_of_stream(final)
            self.assertEqual(self.servo.rejected, 0)
            self.assertAlmostEqual(self.reader.sent[-1][0], final[0])
            if changed:
                self.assertTrue(all(q[0] < .023 for q in self.reader.sent))
            # A new subscription is a new admission context for the next trial.
            await self.servo.call({"action": "pause"})
            await self.servo.call({"action": "resume"})

    async def test_fresh_observations_keep_one_slow_ik_and_jaw_command(self):
        target = self.measured.copy()
        target[0], target[7] = .0005, .05

        async def stream():
            while True:
                observed = time.time_ns() // 1_000_000
                self.servo.publisher.callback(message(target, obs_stamp=observed))
                await asyncio.sleep(.03)

        publisher = asyncio.create_task(stream())
        try:
            with self.delayed_ik() as started:
                self.assertTrue(await asyncio.to_thread(started.wait, 1.))
                await asyncio.sleep(.25)
            await asyncio.wait_for(self.reader.first_write.wait(), 1.)
            await self.wait_for_end_of_stream()
            self.assertEqual(self.reader.sent[-1], [.0005] * 7 + [0.] * 9)
            self.reader.send_gripper.assert_awaited_once()
            self.state.kinematics.solve.assert_called_once()
            self.assertEqual(self.servo.rejected, 0)
        finally:
            publisher.cancel()
            await asyncio.gather(publisher, return_exceptions=True)

    async def test_echo_supersedes_outstanding_ik_and_active_arm_before_solve_returns(self):
        left = self.measured.copy()
        left[0] = .035
        await self.feed(left)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        both = left.copy()
        both[8] = .02
        self.stamp += 1
        with self.delayed_ik() as started:
            self.servo.publisher.callback(message(both, stamp=self.stamp))
            self.assertTrue(await asyncio.to_thread(started.wait, 1.))
            self.assertEqual(self.servo.info()['active_arms'], ['left'])
            self.measured[0] = self.reader.q[0]
            observed = self.stamp
            self.observed[observed] = (self.measured.copy(), self.state.joints()[:14])
            self.stamp += 1
            self.servo.publisher.callback(message(self.measured, stamp=self.stamp, obs_stamp=observed))
            await self.wait_for_end_of_stream(self.measured, obs_stamp=observed)
            self.assertLess(abs(self.reader.sent[-1][0] - self.measured[0]), .001)
            self.assertFalse(self.servo.paused)
        await self.echo_measurement()
        await self.wait_for_end_of_stream(self.measured, obs_stamp=self.stamp)
        self.assertEqual(self.state.kinematics.solve.call_count, 2)
        self.assertTrue(all(q[7:] == [0.] * 9 for q in self.reader.sent))

    async def test_repeated_absolute_goal_does_not_restart_completed_output(self):
        frame = self.measured.copy()
        frame[:7] = self.POSE
        await self.feed(frame)
        await self.wait_for_end_of_stream(frame)
        sent = len(self.reader.sent)
        await self.feed(frame)
        self.state.kinematics.solve.assert_called_once()
        self.assertEqual(len(self.reader.sent), sent)

    async def test_duplicate_and_older_sequences_do_not_reissue_commands(self):
        frame = self.measured.copy()
        frame[7] += .01
        await self.feed(frame, seq=10)
        frame[7] += .02
        for seq in (10, 9):
            await self.feed(frame, seq=seq)
            self.assertEqual(self.servo.last["status"], "rejected")
        self.assertEqual(self.servo.accepted, 1)
        self.reader.send_gripper.assert_awaited_once()

    async def test_arbitration_uses_sender_and_sequence_per_source(self):
        target = self.measured.copy()
        target[7] = .05
        await self.feed(target, source="task", seq=90, priority=10)
        target[7] = .02
        await self.feed(target, source="operator", seq=1, priority=50)
        target[7] = .06
        await self.feed(target, source="task", seq=91, priority=10)
        self.assertEqual((self.servo.accepted, self.servo.rejected), (2, 1))
        self.assertEqual(self.reader.send_gripper.await_count, 2)
        self.assertAlmostEqual(self.reader.send_gripper.await_args.args[0]['left_opening'],
                               .02 / .085 * 94, places=5)

    async def test_receive_checks_ttl_and_age_of_optional_external_observation(self):
        target = self.measured.copy()
        target[7] = .05
        now = time.time_ns() // 1_000_000
        max_age = self.servo.info()['control_interface']['rate']['max_obs_age_ms']
        await self.feed(target, stamp=now - 100, ttl=10)
        await self.feed(target, obs_stamp=now - max_age - 1)
        self.reader.send_gripper.assert_not_awaited()
        await self.feed(target, obs_stamp=now - 25)  # A fresh external camera frame.
        target[7] = .02
        await self.feed(target)  # An absolute setpoint independent of any observation.
        self.assertFalse(self.observed)
        self.assertEqual((self.servo.accepted, self.servo.rejected), (2, 2))
        self.assertEqual(self.reader.send_gripper.await_count, 2)

    async def test_expired_target_cannot_reach_sdk_after_slow_ik(self):
        target = self.measured.copy()
        target[0], target[7] = .001, .05
        with self.delayed_ik() as started:
            self.servo.publisher.callback(message(target, ttl=40))
            self.assertTrue(await asyncio.to_thread(started.wait, 1.))
            await asyncio.sleep(.06)
        await asyncio.sleep(.03)
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()

    async def test_oversized_gripper_request_still_executes_reachable_arm_target(self):
        target = self.measured.copy()
        target[0], target[7] = .0005, .2
        await self.feed(target)
        self.assertEqual(self.servo.last['clamped_opening_m'], {'left': .085})
        await self.wait_for_end_of_stream(target)
        self.assertEqual(self.reader.sent[-1], [.0005] * 7 + [0.] * 9)
        self.reader.send_gripper.assert_awaited_once()
        self.assertEqual(self.reader.send_gripper.await_args.args[0]['left_opening'], 94.)

    async def test_watchdog_interrupts_active_arm_and_discards_inflight_ik(self):
        target = self.measured.copy()
        target[0] = .2
        await self.feed(target)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        target[8], target[15] = .05, .08
        with self.delayed_ik() as started:
            self.servo.publisher.callback(message(target))
            self.assertTrue(await asyncio.to_thread(started.wait, 1.))
            await asyncio.wait_for(self.wait_for_end_of_stream(), 1.)
            sent = len(self.reader.sent)
            self.assertLess(self.reader.sent[-1][0], .2)
        await asyncio.sleep(.05)
        self.assertEqual(len(self.reader.sent), sent)
        self.assertTrue(all(q[7:] == [0.] * 9 for q in self.reader.sent))
        self.reader.send_gripper.assert_not_awaited()
        target = self.measured.copy()
        target[7] = .05
        await self.feed(target)
        self.reader.send_gripper.assert_awaited_once()

    async def test_slow_replacement_does_not_keep_old_arm_goal_alive(self):
        old = self.measured.copy()
        old[0] = .2
        await self.feed(old, source="task", priority=10, ttl=5000)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        new = old.copy()
        new[0] = -.001
        with self.delayed_ik() as started:
            self.servo.publisher.callback(message(new, source="operator"))
            self.assertTrue(await asyncio.to_thread(started.wait, 1.))
            # Refresh the replacement past the old goal's watchdog interval.
            watchdog = self.servo.info()['control_interface']['rate']['watchdog_ms'] / 1000
            end = time.monotonic() + watchdog + .10
            while time.monotonic() < end:
                self.servo.publisher.callback(message(new, source="operator"))
                await asyncio.sleep(.025)
            self.assertFalse(self.servo.info()['active_arms'])
            self.assertLess(self.reader.sent[-1][0], .2)
            writes = len(self.reader.sent)
            await asyncio.sleep(.03)
            self.assertEqual(len(self.reader.sent), writes)

    async def test_moving_head_blocks_arm_takeover_and_sibling_jaw(self):
        self.speeds[-1] = .2
        frame = self.measured.copy()
        frame[:7], frame[7] = self.POSE, .04
        await self.feed(frame)
        self.assertEqual(self.servo.rejected, 1)
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()

    async def test_lifecycle_ignores_queued_input_from_previous_session(self):
        old = self.servo.publisher.callback
        await self.servo.preempt()
        await self.servo.call({"action": "resume"})
        frame = self.measured.copy()
        frame[7] = .04
        old(message(frame))
        await asyncio.sleep(.01)
        self.assertEqual(self.servo.accepted, 0)
        self.assertEqual(self.servo.topic, CONTROL_TOPIC)

    async def test_lost_joint_feedback_aborts_further_output(self):
        target = self.measured.copy()
        target[:7] = self.POSE
        await self.feed(target)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        self.assertEqual(self.servo.info()['active_arms'], ['left'])

        def unavailable():
            raise ValueError("Fresh DACH joint feedback is required")

        self.state.joints = self.state.joint_snapshot = unavailable
        sent = len(self.reader.sent)
        await self.wait_for_end_of_stream()
        self.assertEqual(self.servo.info()['state'], 'paused')
        self.assertEqual(self.servo.last['status'], 'fault')
        self.assertEqual(len(self.reader.sent), sent)

    async def test_admitted_arm_finishes_sending_if_jaw_feedback_is_lost(self):
        target = self.measured.copy()
        # A short, already-admitted move can finish inside its watchdog interval.
        target[0] = .00003
        await self.feed(target)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        self.state.values = Mock(side_effect=ValueError('Fresh gripper openings are required'))
        await self.wait_for_end_of_stream()
        info = self.servo.info()
        self.assertEqual(info['state'], 'running', info['last'])
        self.assertEqual(info['last']['status'], 'accepted')
        self.assertEqual(self.reader.sent[-1], [.00003] * 7 + [0.] * 9)

    async def test_encoder_divergence_aborts_stream(self):
        target = self.measured.copy()
        target[:7] = self.POSE
        await self.feed(target)
        await asyncio.wait_for(self.reader.first_write.wait(), 1.)
        self.assertEqual(self.servo.info()['active_arms'], ['left'])
        self.reader.q = self.reader.q.copy()
        self.reader.q[0] += .3
        sent = len(self.reader.sent)
        await self.wait_for_end_of_stream()
        self.assertEqual(self.servo.info()['state'], 'paused')
        self.assertEqual(self.servo.last['status'], 'fault')
        self.assertEqual(len(self.reader.sent), sent)

    async def test_stop_reports_uncertain_gripper_and_skips_queued_writes(self):
        entered = asyncio.Event()
        release = asyncio.Event()

        async def pending_ack(command):
            entered.set()
            await release.wait()
            return {"result": "success"}

        self.reader.send_gripper.side_effect = pending_ack
        target = self.measured.copy()
        target[7], target[15] = .01, .02
        self.servo.publisher.callback(message(target, stamp=self.stamp))
        await asyncio.wait_for(entered.wait(), 1.)
        try:
            info = await asyncio.wait_for(self.servo.call({'action': 'stop'}), 1.)
            self.assertEqual(info['state'], 'idle')
            self.assertEqual(info['last']['status'], 'outcome_unknown')
            self.assertEqual(info['last']['stamp_ms'], self.stamp)
        finally:
            release.set()
        await self.servo.call({'action': 'start'})
        await asyncio.sleep(.02)
        self.reader.send_gripper.assert_awaited_once()
        self.assertEqual(self.servo.accepted, 0)

    async def test_unknown_gripper_outcome_faults_without_retransmission(self):
        self.reader.send_gripper.side_effect = MoveError("unknown result", None)
        frame = self.measured.copy()
        frame[7] = .01
        await self.feed(frame)
        self.assertTrue(self.servo.paused)
        self.assertEqual(self.servo.last["status"], "fault")
        self.reader.send_gripper.assert_awaited_once()

    async def test_wire_validation(self):
        valid = self.measured.copy()
        wrong_quat = valid.copy()
        wrong_quat[3:7] = [0.] * 4
        old_frame = {"schema": SCHEMA, "frame": FRAME, "stamp_ms": self.stamp, "eef": valid}
        for raw in (json.dumps(old_frame), message(wrong_quat), message(valid[:-1]), message(valid[:-1] + [True]),
                    message(valid[:-1] + [float("inf")]), message(valid, stamp=1),
                    message(valid, stamp=time.time_ns() // 1_000_000 + 500)):
            with self.subTest(raw=raw):
                await self.send(raw)
                self.assertEqual(self.servo.last["status"], "rejected")
        self.assertFalse(self.reader.sent)
        self.reader.send_gripper.assert_not_awaited()
