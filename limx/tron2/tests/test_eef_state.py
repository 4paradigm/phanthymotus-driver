"""DACH kinematics and complete, fresh end-effector feedback, off hardware."""
import asyncio
from contextlib import contextmanager, nullcontext
import math
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch

import numpy as np

from device import Adapter
from eef_kinematics import Kinematics
from eef_state import EefState, FRAME, STATE_TOPIC
from skeleton import ARM_NAMES, JOINT_NAMES
from state import joint_state


class EefKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = Kinematics()

    def test_reference_and_local_solution_for_each_arm(self):
        for side, sign in [('left', 1), ('right', -1)]:
            with self.subTest(side=side):
                np.testing.assert_allclose(self.k.forward(side, [0.] * 7),
                    [-.00254, sign * .16758, -.75719, 0, 0, 0, 1], atol=1e-10)
                q = [.2, sign * .25, -.3, -.9, .15, .1, -.15]
                pose = self.k.forward(side, q)
                sol = self.k.solve(side, pose[:3], pose[3:], [v + .012 for v in q])
                reconstructed = self.k.forward(side, sol)
                self.assertLessEqual(math.dist(reconstructed[:3], pose[:3]), .001)
                cosine = min(1., abs(sum(a*b for a, b in zip(reconstructed[3:], pose[3:]))))
                self.assertLessEqual(2 * math.acos(cosine), .01)

    def test_small_nonzero_targets_change_the_command(self):
        q = [.2, .25, -.3, -.9, .15, .1, -.15]
        pose = self.k.forward('left', q)
        x, y, z, w = pose[3:]
        for distance, angle in ((.0001, 0.), (0., .0001)):
            with self.subTest(distance=distance, angle=angle):
                target = pose[:3]
                target[0] += distance
                c, s = math.cos(angle / 2), math.sin(angle / 2)
                rot = [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z]
                solved = self.k.solve('left', target, rot, q)
                self.assertNotEqual(solved, q)
                expected = target + rot
                np.testing.assert_allclose(self.k.forward('left', solved), expected, atol=1e-8, rtol=0)

    def test_fixed_tcp_matches_independent_vendor_readback_for_both_arms(self):
        # Concurrent data from the TRON2 SDK joint and move_pose read interfaces.
        # SDK poses use WXYZ; card uses XYZW. The two arms have different poses.
        samples = (
            ('left', [-1.4025, .4275005, -2.2763996, -.9870999, 1.1352, .4776000, -.5942000],
             [.48109232, .20505833, -.38151157, -.08703565, -.08346913, -.51331834, .84968346]),
            ('right', [-.10459997, -.04019988, -.17069983, .21409988, 1.5789001, .03180003, -.5951999],
             [.10649767, -.21099055, -.72738601, -.20187166, -.13689639, .62924510, .73794160]),
        )
        for side, q, expected in samples:
            with self.subTest(side=side):
                np.testing.assert_allclose(self.k.forward(side, q), expected, atol=1e-6)

    def test_encoder_and_target_have_separate_limits(self):
        q = [.2, .25, -.3, -.9, .15, .1, -.7505]
        pose = self.k.forward('left', q)
        seed = q[:6] + [-.76]
        self.assertAlmostEqual(self.k.solve('left', pose[:3], pose[3:], seed)[6], -.7505)
        with self.assertRaisesRegex(ValueError, 'one degree'):
            self.k.solve('left', pose[:3], pose[3:], q[:6] + [-.8])

    def test_far_target_is_shortened_along_requested_direction(self):
        q = [.2, .25, -.3, -.9, .15, .1, -.15]
        pose = self.k.forward('left', q)
        for distance in (1., 1.e90):
            with self.subTest(distance=distance):
                target = [pose[0] + distance, *pose[1:3]]
                solved = self.k.solve('left', target, pose[3:], q)
                actual = self.k.forward('left', solved)
                self.assertGreater(actual[0] - pose[0], .01)
                self.assertLess(actual[0] - pose[0], distance)
                np.testing.assert_allclose(actual[1:], pose[1:], atol=1e-7)

    def test_limited_rotation_advances_on_the_requested_axis_without_translating(self):
        # A captured DACH configuration close to left wrist command limits.
        seed = [-1.59970, .17900, -2.57080, -1.07340, 1.39310, .74810, -.49990]
        initial = self.k.forward('left', seed)
        angle = .17
        x, y, z, w = initial[3:]
        c, s = math.cos(angle/2), math.sin(angle/2)
        goal = [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z]
        solved = self.k.solve('left', initial[:3], goal, seed)
        lower, upper = self.k._limits['left']
        self.assertTrue(np.all(lower <= solved) and np.all(solved <= upper))
        self.assertLessEqual(max(abs(a-b) for a, b in zip(solved, seed)), math.radians(35))
        actual = self.k.forward('left', solved)
        np.testing.assert_allclose(actual[:3], initial[:3], atol=1e-8, rtol=0)
        # Relative orientation must be a *positive* partial Z rotation.
        fraction_angle = 2 * math.acos(min(1., abs(np.dot(actual[3:], initial[3:]))))
        self.assertGreater(fraction_angle, 0.)
        self.assertLess(fraction_angle, angle)
        c, s = math.cos(fraction_angle/2), math.sin(fraction_angle/2)
        np.testing.assert_allclose(actual[3:], [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z],
                                   atol=1e-8, rtol=0)

    def test_rejects_local_request_with_no_feasible_forward_segment(self):
        seed = self.k._limits['left'][0]
        pose = self.k.forward('left', seed)
        target = pose[:3]
        target[2] -= .01
        with self.assertRaisesRegex(ValueError, 'no feasible command'):
            self.k.solve('left', target, pose[3:], seed)

    def test_shortens_a_small_cartesian_move_that_needs_a_large_joint_excursion(self):
        # Nearly straight elbow. The full translation needs ~49 degrees on a joint.
        seed = [0., .2, 0., -.05, 0., 0., 0.]
        pose = self.k.forward('left', seed)
        start = np.array(pose[:3])
        delta = np.array([-.060990, -.027047, .052666])
        solved = self.k.solve('left', start + delta, pose[3:], seed)
        self.assertLessEqual(max(abs(a-b) for a, b in zip(solved, seed)), math.radians(35))
        actual = self.k.forward('left', solved)
        shift = np.array(actual[:3]) - start
        fraction = np.dot(shift, delta) / np.dot(delta, delta)
        self.assertGreater(fraction, 0.)
        self.assertLess(fraction, 1.)
        np.testing.assert_allclose(shift, fraction * delta, atol=1e-8, rtol=0)
        np.testing.assert_allclose(actual[3:], pose[3:], atol=1e-8, rtol=0)


def fake_adapter():
    joint = {'available': True, 'reception_age_seconds': .01,
             'data': {'names': list(JOINT_NAMES), 'q': [0.] * 16, 'robot_timestamp': 1}}
    grippers = {'result': 'success', 'timestamp': 5,
                'left_opening': 100, 'left_speed': 0, 'left_force': 0,
                'right_opening': 47, 'right_speed': 0, 'right_force': 0}
    reader = SimpleNamespace(configuration={'mode': 'upper_limb', 'state': 'identified'},
        connected=True, ws=object(), last_round_trip=0., sample=Mock(return_value=joint),
        read_gripper=AsyncMock(return_value=grippers), pose_feedback=nullcontext)
    return SimpleNamespace(reader=reader, enabled={'eef_state'}, publish=Mock())


class EefStateTests(unittest.TestCase):
    def setUp(self):
        self.adapter = fake_adapter()
        self.state = EefState(self.adapter)
        self.state.call('start')
        self.state.joints()  # Establish the mock reader's connection.
        self.claws = self.adapter.reader.read_gripper.return_value

    def test_dense_packet_in_official_order_and_zero_opening(self):
        data = self.adapter.reader.sample.return_value['data']
        q = [0.] * 16
        q[1], q[8] = .25, -.12
        data.update(names=JOINT_NAMES[::-1], q=q[::-1])
        self.state._update_openings(dict(self.claws, right_opening=0), 0.)
        packet = self.state.packet()
        self.assertEqual(set(packet), {'schema', 'frame', 'stamp_ms', 'eef'})
        self.assertEqual((packet['schema'], packet['frame']), ('tron2.eef/1', FRAME))
        self.assertEqual(packet['eef'][:7], self.state.kinematics.forward('left', q[:7]))
        self.assertEqual(packet['eef'][8:15], self.state.kinematics.forward('right', q[7:14]))
        # A true zero and full raw sensor travel are both legitimate measurements.
        self.assertAlmostEqual(packet['eef'][7], .085 * 100 / 94)
        self.assertEqual(packet['eef'][15], 0.)
        self.assertEqual(self.state.info()['sample'], packet)

    def test_feedback_reference_is_unambiguous_across_repeated_publications(self):
        forward = self.state.kinematics.forward

        def negative_quaternion(arm, joints):
            pose = forward(arm, joints)
            return pose[:3] + [-v for v in pose[3:]]

        self.state.kinematics.forward = negative_quaternion
        clock = 1000.
        with patch('eef_state.time.time', side_effect=lambda: clock):
            self.adapter.reader.ws = object()
            self.state.joints()
            self.state._update_openings(self.claws, 0.)
            self.state._poll_joints()
            first = self.adapter.publish.call_args.args[1]
            data = self.adapter.reader.sample.return_value['data']
            first_q = data['q'][:14]
            data['q'][0] += .0002
            self.state._poll_joints()
            self.assertEqual(self.adapter.publish.call_args.args[1], first)
            clock += .2
            data['robot_timestamp'] += 1
            self.state._update_openings(dict(self.claws, timestamp=6), 0.)
            self.state._poll_joints()
            second = self.adapter.publish.call_args.args[1]
        self.assertGreater(second['stamp_ms'], first['stamp_ms'])
        self.assertNotEqual(first['eef'][:7], second['eef'][:7])
        for packet, joints in ((first, first_q), (second, data['q'][:14])):
            stamp = packet['stamp_ms']
            self.assertEqual(self.state.observation(stamp), (packet['eef'], joints))
            self.assertIsNone(self.state.observation(stamp - 1))

    def test_unnamed_native_vector_and_unavailable_status(self):
        data = self.adapter.reader.sample.return_value['data']
        data['names'] = []
        data['q'][2] = .123
        self.assertEqual(self.state.joints(), data['q'])
        self.assertFalse(self.state.info()['available'])
        self.state._update_openings(self.claws, 0.)
        data['q'].pop()
        self.assertFalse(self.state.info()['available'])

    def test_encoder_feedback_for_only_the_named_arms_is_enough_for_fk(self):
        q = [.2, .25, -.3, -.9, .15, .1, -.15, .1, -.2, .3, -.8, .05, -.1, .2]
        raw = dict(result='success', names=ARM_NAMES[::-1], q=q[::-1], dq=[0.] * 14, timestamp=2)
        sample = self.adapter.reader.sample.return_value
        sample['data'] = joint_state(raw)
        self.state._update_openings(self.claws, 0.)
        values = self.state.packet()['eef']
        self.assertEqual(len(values), 16)
        self.assertEqual(values[:7], self.state.kinematics.forward('left', q[:7]))
        self.assertEqual(values[8:15], self.state.kinematics.forward('right', q[7:]))
        with self.assertRaisesRegex(ValueError, '16 DACH joints'):
            self.state.joint_snapshot()
        sample['data'] = joint_state(dict(raw, names=raw['names'][1:], q=raw['q'][1:], dq=[0.] * 13))
        self.assertFalse(self.state.info()['available'])

    def test_joint_snapshot_requires_measured_velocities(self):
        data = self.adapter.reader.sample.return_value['data']
        with self.assertRaises(ValueError):
            self.state.joint_snapshot()
        data['names'] = list(reversed(JOINT_NAMES))
        data['dq'] = list(reversed([i / 10 for i in range(16)]))
        self.assertEqual(self.state.joint_snapshot(), (1, [0.] * 16, [i / 10 for i in range(16)]))
        data['dq'][0] = math.nan
        with self.assertRaises(ValueError):
            self.state.joint_snapshot()

    def test_joint_snapshot_takes_position_and_velocity_from_one_frame(self):
        reader = self.adapter.reader
        q = [i / 20 for i in range(16)]
        dq = [-x for x in q]
        reader.sample.reset_mock()
        reader.sample.side_effect = [{'available': True, 'reception_age_seconds': .01,
            'data': {'names': list(reversed(JOINT_NAMES)), 'q': q[::-1], 'dq': dq[::-1],
                     'robot_timestamp': 2}}]
        self.assertEqual(self.state.joint_snapshot(), (2, q, dq))
        reader.sample.assert_called_once_with('joint_state')

    def test_joint_freshness_includes_query_time_and_repeated_late_sample(self):
        reader = self.adapter.reader
        sample = reader.sample.return_value
        with patch('eef_state.time') as clock:
            clock.monotonic.return_value = 1000.
            clock.time.return_value = 2000.
            reader.ws = object()
            reader.last_round_trip = .30
            sample['reception_age_seconds'] = .10
            with self.assertRaisesRegex(ValueError, 'Fresh DACH joint feedback'):
                self.state.joints()

            # A faster second query still returns the same old vendor sample.
            reader.last_round_trip = .125
            sample['reception_age_seconds'] = .0625
            with self.assertRaisesRegex(ValueError, 'Fresh DACH joint feedback'):
                self.state.joints()
            sample['data']['robot_timestamp'] += 1
            self.state._update_openings(self.claws, .03)
            self.assertEqual(self.state.packet()['stamp_ms'], int((2000. - .125 - .0625) * 1000))

    def test_stale_frozen_feedback_and_new_connection(self):
        with patch('eef_state.time.monotonic', return_value=10) as clock:
            self.adapter.reader.ws = object()
            self.state.joints()
            self.state._update_openings(self.claws, 0.)
            self.assertTrue(self.state.info()['available'])
            clock.return_value = 12
            self.assertFalse(self.state.info()['available'])
            self.adapter.reader.sample.return_value['data']['robot_timestamp'] += 1
            self.assertFalse(self.state.info()['available'])
            with self.assertRaises(ValueError):
                self.state._update_openings(self.claws, 0.)
            self.state._update_openings(dict(self.claws, timestamp=6), 0.)
            self.assertTrue(self.state.info()['available'])
            self.adapter.reader.sample.return_value['reception_age_seconds'] = .4
            self.assertFalse(self.state.info()['available'])
            self.adapter.reader.connected = False
            self.assertFalse(self.state.info()['available'])
            self.adapter.reader.ws = object()
            self.adapter.reader.connected = True
            self.adapter.reader.sample.return_value['reception_age_seconds'] = .01
            self.adapter.reader.sample.return_value['data']['robot_timestamp'] = 1
            self.assertFalse(self.state.info()['available'])
            self.state._update_openings(self.claws, 0.)
            self.assertTrue(self.state.info()['available'])


class EefStateLoopTests(unittest.IsolatedAsyncioTestCase):
    async def test_late_gripper_reply_and_its_fast_repeat_stay_unavailable(self):
        adapter = fake_adapter()
        state = EefState(adapter)
        reader = adapter.reader
        sample = reader.sample.return_value
        grippers = reader.read_gripper.return_value
        replies = iter(((1., 10), (.03125, 10), (.125, 11)))
        observations = []
        now = 1000.

        def read_joints(_):
            sample['data']['robot_timestamp'] = now
            return sample

        async def read_gripper():
            nonlocal now
            observations.append(state.info())
            if len(observations) == 4:
                raise asyncio.CancelledError()
            delay, stamp = next(replies)
            now += delay
            return dict(grippers, timestamp=stamp)

        reader.sample.side_effect = read_joints
        reader.read_gripper.side_effect = read_gripper
        with patch('eef_state.time') as clock, patch('eef_state.GRIPPER_PERIOD', 0.):
            clock.monotonic.side_effect = lambda: now
            clock.time.side_effect = lambda: 5000. + now
            state.call('start')
            with self.assertRaises(asyncio.CancelledError):
                await state._poll_grippers()
        self.assertEqual([item['available'] for item in observations], [False, False, False, True])
        self.assertEqual(observations[-1]['sample']['stamp_ms'], int(6001.03125 * 1000))

    async def test_standalone_sensor_polls_hardware_only_while_started(self):
        reader = fake_adapter().reader
        released = asyncio.Event()

        @contextmanager
        def feedback():
            try:
                yield
            finally:
                released.set()

        reader.pose_feedback = Mock(side_effect=feedback)
        frames = asyncio.Queue()
        adapter = Adapter(lambda topic, packet: frames.put_nowait(packet),
                          {'enabled_cards': ['eef_state']}, reader=reader)
        task = asyncio.create_task(adapter.eef_state.run())
        try:
            await asyncio.sleep(.25)
            reader.pose_feedback.assert_not_called()
            reader.read_gripper.assert_not_awaited()
            self.assertTrue(frames.empty())

            await adapter.execute_tool('eef_state', {'action': 'start'})
            await asyncio.wait_for(frames.get(), 1.)
            stopped = await adapter.execute_tool('eef_state', {'action': 'stop'})
            self.assertEqual(stopped['state'], 'idle')
            await asyncio.wait_for(released.wait(), 1.)
            polls = reader.read_gripper.await_count
            await asyncio.sleep(.3)
            self.assertEqual(reader.read_gripper.await_count, polls)
            self.assertTrue(frames.empty())

            # Toggle twice without yielding to the background loop.
            reader.sample.return_value['data']['robot_timestamp'] += 1
            reply = reader.read_gripper.return_value
            reply.update(right_opening=0, timestamp=reply['timestamp'] + 1)
            for action in ('start', 'stop', 'start'):
                await adapter.execute_tool('eef_state', {'action': action})

            async def next_closed_gripper():
                while True:
                    packet = await frames.get()
                    if packet['eef'][15] == 0.:
                        return

            await asyncio.wait_for(next_closed_gripper(), 1.)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def test_stop_quiets_topic_while_servo_keeps_fresh_feedback_and_restart_republishes(self):
        reader = fake_adapter().reader
        reader.move_pending = reader.head_pending = reader._pose_servo = None
        sample = reader.sample.return_value
        reply = reader.read_gripper.return_value

        def read_joints(_):
            sample['data']['robot_timestamp'] += 1
            return sample

        async def read_gripper():
            reply['timestamp'] += 1
            return dict(reply)

        reader.sample.side_effect = read_joints
        reader.read_gripper.side_effect = read_gripper
        frames = asyncio.Queue()
        adapter = Adapter(lambda topic, packet: frames.put_nowait((topic, packet)),
                          {'enabled_cards': ['eef_state', 'servo_eef']}, reader=reader)
        adapter.servo_eef.publisher = Mock()

        async def sensor(action):
            return await adapter.execute_tool('eef_state', {'action': action, 'instance_id': 'test-card'})

        task = asyncio.create_task(adapter.eef_state.run())
        try:
            await adapter.execute_tool('servo_eef', {'action': 'start'})
            self.assertEqual((await sensor('start'))['state'], 'running')
            topic, frame = await asyncio.wait_for(frames.get(), 1.)
            self.assertEqual(topic, STATE_TOPIC)
            self.assertEqual(len(frame['eef']), 16)
            self.assertAlmostEqual(frame['eef'][15], .0425)

            for action in ('stop', 'stop', 'info'):
                status = await sensor(action)
                self.assertEqual(status['state'], 'idle')
                self.assertFalse(status['available'])
                self.assertNotIn('sample', status)
            reply['right_opening'] = 0
            with self.assertRaises(asyncio.TimeoutError):
                await asyncio.wait_for(frames.get(), .4)
            self.assertTrue(adapter.servo_eef.running)
            self.assertEqual(adapter.servo_eef.state.values()[15], 0.)

            self.assertEqual((await sensor('start'))['state'], 'running')
            topic, frame = await asyncio.wait_for(frames.get(), 1.)
            self.assertEqual(topic, STATE_TOPIC)
            self.assertEqual(frame['eef'][15], 0.)
        finally:
            await adapter.servo_eef.close()
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def test_publishes_only_complete_measurements_and_resumes_after_recovery(self):
        adapter = Adapter(Mock(), {'enabled_cards': ['eef_state']}, reader=fake_adapter().reader)
        reply = adapter.reader.read_gripper.return_value
        measured = asyncio.Event()
        async def read():
            measured.set()
            return dict(reply)
        adapter.reader.read_gripper.side_effect = read
        opening = reply.pop('right_opening')
        published = asyncio.Event()
        adapter.publish.side_effect = lambda topic, packet: published.set()
        state = adapter.eef_state
        task = asyncio.create_task(state.run())
        try:
            await adapter.execute_tool('eef_state', {'action': 'start'})
            await asyncio.wait_for(measured.wait(), 1.)
            adapter.publish.assert_not_called()
            self.assertFalse(state.info()['available'])
            reply['right_opening'] = opening
            reply['timestamp'] += 1
            await asyncio.wait_for(published.wait(), 1.)
            topic, packet = adapter.publish.call_args.args
            self.assertEqual(topic, STATE_TOPIC)
            self.assertEqual(len(packet['eef']), 16)
            self.assertAlmostEqual(packet['eef'][15], .0425)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
