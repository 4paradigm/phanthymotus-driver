"""Drive the standard control port and observe native DACH MoveP/gripper writes."""
import asyncio
import json
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

import aiohttp

from common.control import ControlSink
from eef_state import CONTROL_TOPIC
from servo_eef import EefServo, WATCHDOG_MS
from vendor import MoveError, RobotReader


LEFT = [.48, .20, -.38, -.5, .5, -.5, .5]
RIGHT = [.41, -.21, -.37, 0., -.6, 0., .8]
MEASURED = LEFT + [.03] + RIGHT + [.04]
READY = {'working_mode': 'developer_mode', 'working_mode_valid': True, 'motor': 'OK',
         'collision_enable_valid': True, 'collision_enable': True,
         'compliant_enable_valid': True, 'compliant_enable': False}


def message(values, **fields):
    now = time.time_ns() // 1_000_000
    msg = {'schema': 'motus.control/1', 'mode': 'eef_pose', 'dof': 16, 'values': values,
           'source': 'test', 'seq': time.monotonic_ns(), 'stamp_ms': now, 'ttl_ms': 200,
           'obs_stamp_ms': now}
    msg.update(fields)
    return json.dumps(msg)


async def wait_until(predicate):
    async def poll():
        while not predicate():
            await asyncio.sleep(.002)
    await asyncio.wait_for(poll(), 1.5)


class Bus:
    def subscribe_json(self, topic, callback):
        self.topic, self.callback = topic, callback
        return callback

    def unsubscribe(self, subscriber):
        self.callback = None


class ServoTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.measured = MEASURED.copy()
        self.reader = RobotReader({'serial': 'DACH_TRON2A_TEST', 'response_timeout_seconds': .2})
        self.reader.ws = Mock(closed=False)
        self.reader.connected = True
        self.reader.configuration = {'mode': 'upper_limb', 'state': 'identified'}
        self.reader.sample = Mock(return_value={'available': True, 'data': READY})
        self.reader.send_move_p = AsyncMock(return_value={'result': 'success', 'status': 'accepted'})
        self.reader.send_gripper = AsyncMock(return_value={'result': 'success'})
        state = SimpleNamespace(values=lambda: self.measured.copy())
        self.servo = EefServo(SimpleNamespace(reader=self.reader,
            arm_servo=SimpleNamespace(_conflict=lambda token=None: None)), state)
        self.servo.publisher = Bus()
        await self.servo.call({'action': 'start'})

    async def asyncTearDown(self):
        await self.servo.close()

    async def feed(self, values, **fields):
        await self.send(message(values, **fields))

    async def send(self, raw):
        before = self.servo.accepted + self.servo.rejected
        self.servo.publisher.callback(raw)
        await wait_until(lambda: self.servo.accepted + self.servo.rejected > before or self.servo.paused)

    def capture_transitions(self):
        announce = self.servo._announce = Mock(wraps=self.servo._announce)
        return lambda: [call.args[0] for call in announce.call_args_list]

    def assert_activity(self, event):
        activity = self.servo.info()['activity']
        self.assertEqual(activity['event'], event)
        self.assertTrue(activity['text'])
        self.assertGreater(activity['stamp_ms'], 0)

    def assert_pose_call(self, left, right, *, index=-1):
        args = self.reader.send_move_p.await_args_list[index].args
        self.assertEqual(list(args[0]), left)
        self.assertEqual(list(args[1]), right)
        self.assertTrue(math.isfinite(args[2]) and args[2] > 0)
        self.assertIsNotNone(args[3])

    async def test_start_and_resume_require_ready_robot_before_subscribing(self):
        for action, end, expected in (('start', 'stop', 'idle'), ('resume', 'pause', 'paused')):
            with self.subTest(action=action):
                await self.servo.call({'action': end})
                self.reader.sample.return_value = {'available': False, 'data': None}
                with self.assertRaisesRegex(ValueError, 'unavailable'):
                    await self.servo.call({'action': action})
                self.assertEqual(self.servo.info()['state'], expected)
                self.assertIsNone(self.servo.publisher.callback)
                self.reader.sample.return_value = {'available': True, 'data': READY}
                result = await self.servo.call({'action': action})
                self.assertEqual(result['state'], 'ready')
                self.assertIsNotNone(self.servo.publisher.callback)
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_robot_losing_readiness_after_start_rejects_motion_frame(self):
        self.reader.sample.return_value = {'available': True, 'data': dict(READY, collision_enable=False)}
        for action in ('start', 'resume'):
            with self.assertRaisesRegex(ValueError, 'unavailable'):
                await self.servo.call({'action': action})
        target = self.measured.copy()
        target[0] += .003
        target[7] += .005
        await self.feed(target)
        self.assertEqual(self.servo.last['status'], 'rejected')
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_feedback_echo_and_gripper_only_do_not_write_movep(self):
        frame = self.measured.copy()
        await self.feed(frame)
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()
        frame[7] += .01
        await self.feed(frame)
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_awaited_once()

    async def test_paired_movep_uses_current_peer_without_rewriting_echoed_opening(self):
        self.measured[15] = .085
        target = self.measured.copy()
        target[0] += .003
        await self.feed(target)
        self.assert_pose_call(target[:7], self.measured[8:15])
        self.reader.send_gripper.assert_not_awaited()

    async def test_changed_target_replaces_active_movep_and_measurement_can_hold_it(self):
        target = self.measured.copy()
        target[0] += .004
        await self.feed(target)
        self.assert_pose_call(target[:7], RIGHT)
        # The native call has acknowledged, but the robot is still in transit.
        self.measured[0] += .001
        target = self.measured.copy()
        target[2] += .003
        await self.feed(target)
        self.assert_pose_call(target[:7], RIGHT)
        self.measured[2] += .001
        hold = self.measured.copy()
        await self.feed(hold)
        self.assert_pose_call(hold[:7], RIGHT)
        await self.feed(hold)
        self.assertEqual(self.reader.send_move_p.await_count, 3)
        self.reader.send_gripper.assert_not_awaited()

    async def test_repeated_target_and_equivalent_quaternion_are_sent_once(self):
        target = self.measured.copy()
        target[8] -= .004
        target[7] = .05
        await self.feed(target)
        self.assert_pose_call(LEFT, target[8:15])
        target[11:15] = [-v for v in target[11:15]]
        for _ in range(3):
            await self.feed(target)
        self.measured[:] = target
        await self.feed(target)
        self.reader.send_move_p.assert_awaited_once()
        self.reader.send_gripper.assert_awaited_once()

    async def test_raw_targets_outside_hard_limits_reject_every_write(self):
        # Establish an in-range baseline: a step limiter alone could otherwise
        # turn the illegal requests below into legal intermediate points.
        await self.feed(self.measured)
        for index, raw in ((0, .733), (10, -.674), (7, .086), (15, -.001)):
            with self.subTest(index=index):
                target = self.measured.copy()
                target[0] += .003
                target[7] += .02
                target[index] = raw
                await self.feed(target)
                self.assertEqual(self.servo.last['status'], 'rejected')
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_target_on_vendor_boundary_is_legal(self):
        self.measured[0] = .730
        target = self.measured.copy()
        target[0], target[1], target[7] = .732, .201, .085
        await self.feed(target)
        self.assert_pose_call(target[:7], RIGHT)
        self.assertEqual(self.reader.send_gripper.await_args.args[0]['left_opening'], 94.)

    async def test_outside_echo_prevents_motion_and_independent_gripper_write(self):
        self.measured[8] = .107  # Actual held right TCP below its native X minimum.
        target = self.measured.copy()
        target[7] = .06
        await self.feed(target)
        self.assertEqual(self.servo.last['status'], 'rejected')
        target[0] -= .003
        await self.feed(target)
        self.assertEqual(self.servo.last['status'], 'rejected')
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_priority_replay_expiry_and_observation_age_gate_native_writes(self):
        target = self.measured.copy()
        target[7] = .05
        await self.feed(target, source='policy', seq=10, priority=20)
        target[7] = .025
        now = time.time_ns() // 1_000_000
        await self.feed(target, source='operator', seq=1, priority=80, stamp_ms=now)
        max_age = self.servo.info()['control_interface']['rate']['max_obs_age_ms']
        target[7] = .02
        for fields in ({'source': 'policy', 'seq': 11, 'priority': 20},
                       {'source': 'operator', 'seq': 1, 'priority': 80},
                       {'source': 'operator', 'seq': 2, 'priority': 80, 'stamp_ms': now - 500},
                       {'source': 'operator', 'seq': 2, 'priority': 80, 'obs_stamp_ms': now - max_age - 1}):
            with self.subTest(fields=fields):
                await self.feed(target, **fields)
                self.assertEqual(self.reader.send_gripper.await_count, 2)
        # A later sequence sent in the same millisecond can replace the prior target.
        await self.feed(target, source='operator', seq=2, priority=80, stamp_ms=now)
        self.assertEqual(self.reader.send_gripper.await_count, 3)
        self.assertEqual((self.servo.accepted, self.servo.rejected), (3, 4))
        self.assertEqual(self.servo.info()['sink']['holder'], 'operator')
        self.reader.send_move_p.assert_not_awaited()

    async def test_malformed_frame_and_missing_feedback_never_write(self):
        target = self.measured.copy()
        target[7] = .06
        bad = target.copy()
        bad[3:7] = [0.] * 4
        for raw in (message(bad), message(target[:-1]), message(target[:-1] + [True]),
                    message(target[:-1] + [math.nan]), message(target, frame='other'),
                    message(target, stamp_ms=True), message(target, stamp_ms=time.time_ns() // 1_000_000 + 900),
                    message(target, schema='motus.eef/1'), message(target, mode='joint_position'),
                    message(target, dof=15), message(target, seq=True), message(target, ttl_ms=0),
                    message(target, source='')):
            await self.send(raw)
        for key in ('schema', 'mode', 'dof', 'source', 'seq', 'ttl_ms', 'stamp_ms', 'values'):
            missing = json.loads(message(target))
            del missing[key]
            await self.send(json.dumps(missing))
        self.servo.state.values = Mock(side_effect=ValueError('Fresh native feedback required'))
        await self.feed(target)
        self.assertEqual(self.servo.accepted, 0)
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_feedback_lost_after_first_gripper_ack_prevents_second_write(self):
        async def lose_feedback(_):
            self.servo.state.values = Mock(side_effect=ValueError('No fresh native feedback'))
            return {'result': 'success'}

        self.reader.send_gripper.side_effect = lose_feedback
        target = self.measured.copy()
        target[7], target[15] = .02, .06
        await self.feed(target)
        self.reader.send_gripper.assert_awaited_once()
        self.assertIn('left_opening', self.reader.send_gripper.await_args.args[0])
        self.assertEqual(self.servo.accepted, 0)

    async def test_no_late_gripper_after_native_ack_crosses_watchdog(self):
        started, acknowledge = asyncio.Event(), asyncio.Event()
        async def delayed(*args):
            started.set()
            await acknowledge.wait()
            return {'result': 'success', 'status': 'accepted'}
        self.reader.send_move_p.side_effect = delayed
        target = self.measured.copy()
        target[0] -= .002
        target[15] = .06
        self.servo.publisher.callback(message(target))
        await asyncio.wait_for(started.wait(), 1.)
        try:
            await wait_until(lambda: self.servo.info()['sink']['stood_down'])
            self.assert_activity('stream_stood_down')
        finally:
            acknowledge.set()
        await asyncio.sleep(.05)
        self.reader.send_gripper.assert_not_awaited()
        self.reader.send_move_p.assert_awaited_once()
        self.assert_activity('stream_stood_down')

    async def test_expired_observation_during_movep_ack_prevents_late_gripper(self):
        started, release = asyncio.Event(), asyncio.Event()
        async def delayed(*args):
            started.set()
            await release.wait()
        self.reader.send_move_p.side_effect = delayed
        target = self.measured.copy()
        target[0] -= .002
        target[7] += .02
        max_age = self.servo.info()['control_interface']['rate']['max_obs_age_ms']
        now = time.time_ns() // 1_000_000
        self.servo.publisher.callback(message(target, obs_stamp_ms=now - max_age + 80))
        try:
            await asyncio.wait_for(started.wait(), 1.)
            await asyncio.sleep(.1)  # Observation expired, but the command TTL and watchdog are still live.
        finally:
            release.set()
        await asyncio.sleep(.03)
        self.reader.send_gripper.assert_not_awaited()
        self.reader.send_move_p.assert_awaited_once()

    async def test_watchdog_notifies_once_per_gap_and_only_valid_input_resumes(self):
        transitions = self.capture_transitions()
        interrupted, resumed = 'stream_interrupted', 'stream_resumed'
        await self.feed(self.measured)
        with self.assertLogs('servo_eef', level='INFO') as interrupted_log:
            await wait_until(lambda: interrupted in transitions())
        self.assert_activity(interrupted)
        self.assertIn(self.servo.info()['activity']['text'], interrupted_log.records[0].getMessage())

        await self.feed(self.measured, stamp_ms=time.time_ns() // 1_000_000 - 500)
        await wait_until(lambda: self.servo.info()['sink']['stood_down'])
        escalated = 'stream_stood_down'
        self.assert_activity(escalated)
        await asyncio.sleep(WATCHDOG_MS / 1000 + .03)
        self.assertEqual(transitions(), [interrupted, escalated])
        with self.assertLogs('servo_eef', level='INFO') as resumed_log:
            await self.feed(self.measured)
            await self.feed(self.measured)
        self.assertEqual(len(resumed_log.records), 1)
        self.assertEqual(transitions(), [interrupted, escalated, resumed])
        self.assert_activity(resumed)
        self.assertIn(self.servo.info()['activity']['text'], resumed_log.records[0].getMessage())

        await wait_until(lambda: transitions().count(interrupted) == 2)
        await self.feed(self.measured)
        self.assertEqual(transitions(), [interrupted, escalated, resumed, interrupted, resumed])
        self.reader.send_move_p.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_explicit_pause_and_stop_do_not_announce_watchdog_recovery(self):
        transitions = self.capture_transitions()
        for ending, restart in (('pause', 'resume'), ('stop', 'start')):
            with self.subTest(ending=ending):
                await self.feed(self.measured)
                before = transitions()
                await self.servo.call({'action': ending})
                await self.servo.call({'action': restart})
                await self.feed(self.measured)
                self.assertEqual(transitions(), before)
                await wait_until(lambda: transitions() == before + ['stream_interrupted'])
                stale = self.servo.publisher.callback
                await self.servo.call({'action': ending})
                await self.servo.call({'action': restart})
                stale(message(self.measured))
                await self.feed(self.measured)
                self.assertEqual(transitions(), before + ['stream_interrupted'])

    async def test_native_rejection_does_not_resubmit_the_same_pose(self):
        self.reader.send_move_p.side_effect = MoveError('vendor refused', True, 'vendor_rejected')
        target = self.measured.copy()
        target[2] += .004
        target[15] = .06
        await self.feed(target)
        self.assertFalse(self.servo.paused)
        self.assertEqual(self.servo.last['status'], 'partial')
        self.reader.send_gripper.assert_awaited_once()
        for _ in range(3):
            await self.feed(target)
        self.reader.send_move_p.assert_awaited_once()
        self.reader.send_gripper.assert_awaited_once()

    async def test_rejected_gripper_pauses_without_retransmitting_motor_fault(self):
        self.reader.send_gripper.side_effect = MoveError('fail_motor', True, 'vendor_rejected')
        target = self.measured.copy()
        target[7] += .01
        await self.feed(target)
        self.assertTrue(self.servo.paused)
        self.assertEqual(self.servo.last['status'], 'rejected')
        self.servo.publisher.callback(message(target))
        await asyncio.sleep(.03)
        self.reader.send_gripper.assert_awaited_once()

    async def test_unconfirmed_native_write_pauses_without_dispatching_sibling(self):
        self.reader.send_move_p.side_effect = MoveError('ack timeout', True, 'outcome_unknown')
        target = self.measured.copy()
        target[2] += .004
        target[15] = .06
        await self.feed(target)
        self.assertTrue(self.servo.paused)
        self.assertEqual(self.servo.last['status'], 'outcome_unknown')
        target[2] += .001
        self.servo.publisher.callback(message(target))
        await asyncio.sleep(.03)
        self.reader.send_move_p.assert_awaited_once()
        self.reader.send_gripper.assert_not_awaited()

    async def test_rotation_uses_bounded_angle_and_preserves_peer(self):
        self.measured[3:7] = [0., 0., 0., 1.]
        target = self.measured.copy()
        target[3:7] = [0., 0., math.sin(.4), math.cos(.4)]
        await self.feed(target)
        left, other, duration, _ = self.reader.send_move_p.await_args.args
        self.assertEqual(list(left[:5]), self.measured[:5])
        self.assertEqual(list(other), RIGHT)
        angle = 2 * math.atan2(left[5], left[6])
        self.assertGreater(angle, 0.)
        self.assertLessEqual(angle, .5 * duration + 1e-9)
        self.assertAlmostEqual(math.hypot(*left[3:]), 1.)

    async def test_long_target_advances_in_bounded_segments_only_with_fresh_input(self):
        target = self.measured.copy()
        target[0], target[1] = .64, .28
        await self.feed(target)
        first, other, duration, _ = self.reader.send_move_p.await_args.args
        watchdog = WATCHDOG_MS / 1000
        self.assertGreater(duration, 0.)
        self.assertLessEqual(duration, watchdog)
        self.assertEqual(list(other), RIGHT)
        self.assertGreater(first[0], LEFT[0])
        self.assertLess(first[0], target[0])
        self.assertAlmostEqual(first[1] - LEFT[1], (first[0] - LEFT[0]) / 2)
        await asyncio.sleep(watchdog + .08)
        self.reader.send_move_p.assert_awaited_once()
        self.measured[:7] = first
        await self.feed(target)
        self.assertEqual(self.reader.send_move_p.await_count, 2)
        second = self.reader.send_move_p.await_args.args[0]
        self.assertGreater(second[0], first[0])
        self.assertLessEqual(second[0], target[0])

    async def test_first_diagonal_and_rotation_reach_vendor_as_one_sink_approved_point(self):
        self.measured[3:7] = [0., 0., 0., 1.]
        target = self.measured.copy()
        target[:3] = [.64, .28, -.42]
        target[3:7] = [0., 0., math.sin(.2), math.cos(.2)]
        approved = []
        submit = ControlSink.submit

        def record_outcome(sink, frame):
            outcome = submit(sink, frame)
            if outcome.applied:
                approved.append(outcome.values)
            return outcome

        with patch.object(ControlSink, 'submit', record_outcome):
            await self.feed(target)
        left, right, duration, _ = self.reader.send_move_p.await_args.args
        self.assertIn((*left, target[7], *right, target[15]), approved)
        self.assertEqual(list(right), RIGHT)
        limits = self.servo.info()['control_interface']['end_effector']
        self.assertAlmostEqual(duration, limits['move_time_s'])
        dx, dy, dz = (end - start for end, start in zip(left[:3], self.measured))
        self.assertGreater(dx, 0.)
        self.assertAlmostEqual(dy, dx / 2)
        self.assertAlmostEqual(dz, -dx / 4)
        self.assertLessEqual(math.hypot(dx, dy, dz), limits['max_translation_per_step_m'] + 1e-9)
        angle = 2 * math.atan2(left[5], left[6])
        self.assertGreater(angle, 0.)
        self.assertLessEqual(angle, limits['max_rotation_per_step_rad'] + 1e-9)

    async def test_recovery_rebases_on_the_new_robot_measurement(self):
        target = self.measured.copy()
        target[0] += .14
        await self.feed(target)
        await wait_until(lambda: self.servo.info()['sink']['holding'])
        self.measured[0] = .36
        await self.feed(target)
        point, right, duration, _ = self.reader.send_move_p.await_args.args
        self.assertEqual(self.reader.send_move_p.await_count, 2)
        self.assertGreater(point[0], self.measured[0])
        self.assertLessEqual(point[0] - self.measured[0], .10 * duration + 1e-9)
        self.assertEqual(list(right), RIGHT)

    async def test_burst_during_pending_ack_cannot_accumulate_unsent_arm_steps(self):
        release = asyncio.Event()

        async def move(*_):
            await release.wait()

        self.reader.send_move_p.side_effect = move
        target = self.measured.copy()
        target[0] += .15
        self.servo.publisher.callback(message(target))
        try:
            await wait_until(lambda: self.reader.send_move_p.await_count == 1)
            for dx, dy in ((.15, 0.), (0., .06), (-.16, 0.)):
                target[0], target[1] = LEFT[0] + dx, LEFT[1] + dy
                for _ in range(10):
                    self.servo.publisher.callback(message(target))
            # Simulate movement of the first segment as its ACK is pending.
            await asyncio.sleep(.04)
            self.assertEqual(self.reader.send_move_p.await_count, 1)
            self.measured[0] += .015
        finally:
            release.set()
        # A host may discard the pending point if feedback changed during the ACK.
        # Only a new, legitimately sequenced input may then drive the next point.
        await asyncio.sleep(.03)
        if self.reader.send_move_p.await_count == 1:
            self.servo.publisher.callback(message(target))
        await wait_until(lambda: self.reader.send_move_p.await_count >= 2)
        left, right, duration, _ = self.reader.send_move_p.await_args.args
        self.assertLess(left[0], self.measured[0])
        self.assertAlmostEqual(left[1], self.measured[1])
        self.assertLessEqual(math.dist(left[:3], self.measured[:3]), .10 * duration + 1e-9)
        self.assertEqual(list(right), RIGHT)
        await asyncio.sleep(.02)
        self.assertEqual(self.reader.send_move_p.await_count, 2)

    async def test_only_latest_target_dispatches_after_pending_native_ack(self):
        started, release = asyncio.Event(), asyncio.Event()
        async def move(*args):
            started.set()
            await release.wait()
            return {'status': 'accepted'}
        self.reader.send_move_p.side_effect = move
        target = self.measured.copy()
        target[0] += .002
        self.servo.publisher.callback(message(target))
        await asyncio.wait_for(started.wait(), 1.)
        try:
            for dx in (-.002, .003):
                target[0] = LEFT[0] + dx
                await asyncio.sleep(.005)
                self.servo.publisher.callback(message(target))
        finally:
            release.set()
        await wait_until(lambda: self.reader.send_move_p.await_count >= 2)
        self.assert_pose_call(target[:7], RIGHT)
        self.assertEqual(self.reader.send_move_p.await_count, 2)

    async def test_stop_and_resume_ignore_callback_from_previous_subscription(self):
        old = self.servo.publisher.callback
        await self.servo.call({'action': 'stop'})
        await self.servo.call({'action': 'start'})
        frame = self.measured.copy()
        frame[0] -= .002
        old(message(frame))
        await asyncio.sleep(.03)
        self.reader.send_move_p.assert_not_awaited()
        self.assertEqual(self.servo.publisher.topic, CONTROL_TOPIC)


class MovePWireTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.reader = RobotReader({'serial': 'DACH_TRON2A_TEST', 'response_timeout_seconds': .08})
        self.reader.configuration = {'state': 'identified', 'mode': 'upper_limb'}
        self.reader.connected = True
        self.reader.sample = Mock(return_value={'available': True, 'data': READY})
        self.socket = self.reader.ws = Mock(closed=False)
        self.writes = asyncio.Queue()
        self.socket.send_json = AsyncMock(side_effect=self.writes.put_nowait)

    async def receive(self, request, data, **fields):
        packet = {'accid': request['accid'], 'guid': request['guid'],
                  'title': request['title'].replace('request_', 'response_'), 'data': data}
        packet.update(fields)
        self.socket.receive = AsyncMock(return_value=SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
            data=json.dumps(packet)))
        await self.reader.receive(self.socket, time.monotonic() + 1.)

    async def test_move_pose_query_matches_native_guid_title_and_accid(self):
        future = asyncio.create_task(self.reader.read_move_pose())
        sent = await asyncio.wait_for(self.writes.get(), 1.)
        self.assertEqual((sent['title'], sent['data']), ('request_get_move_pose', {}))
        reply = {'result': 'success', 'timestamp': 100., 'left_position': LEFT[:3],
                 'left_quat': [LEFT[6], *LEFT[3:6]], 'right_position': RIGHT[:3],
                 'right_quat': [RIGHT[6], *RIGHT[3:6]]}
        for changes in ({'guid':'unrelated'}, {'accid':'other'}, {'title':'response_movep'}):
            await self.receive(sent, reply, **changes)
            self.assertFalse(future.done())
        await self.receive(sent, reply)
        self.assertEqual(await future, reply)

    async def test_motion_writes_sdk_pos_in_wxyz_and_waits_for_its_own_ack(self):
        with self.reader.pose_servo({'motion_tool': 'servo_eef'}) as token:
            future = asyncio.create_task(self.reader.send_move_p(LEFT, RIGHT, 2., token))
            sent = await asyncio.wait_for(self.writes.get(), 1.)
            self.assertEqual(sent['title'], 'request_movep')
            self.assertEqual(sent['data'], {'time': 2., 'pos': [*LEFT[:3], LEFT[6], *LEFT[3:6],
                                                                *RIGHT[:3], RIGHT[6], *RIGHT[3:6]]})
            for changes in ({'guid':'unrelated'}, {'accid':'other'}, {'title':'response_movej'}):
                await self.receive(sent, {'result':'success'}, **changes)
                self.assertFalse(future.done())
            await self.receive(sent, {'result':'success'})
            await future
        self.assertTrue(self.writes.empty())

    async def test_vendor_rejection_and_lost_ack_do_not_retry(self):
        with self.reader.pose_servo({'motion_tool': 'servo_eef'}) as token:
            future = asyncio.create_task(self.reader.send_move_p(LEFT, RIGHT, 1., token))
            sent = await asyncio.wait_for(self.writes.get(), 1.)
            await self.receive(sent, {'result':'fail_invalid_cmd'})
            with self.assertRaises(MoveError) as refused:
                await future
            self.assertEqual(refused.exception.code, 'vendor_rejected')
            with self.assertRaises(MoveError) as timeout:
                await self.reader.send_move_p(LEFT, RIGHT, 1., token)
            self.assertTrue(timeout.exception.motion_sent)
            self.assertNotEqual(timeout.exception.code, 'vendor_rejected')
        self.assertEqual(self.writes.qsize(), 1)
        self.assertEqual(self.socket.send_json.await_count, 2)

    async def test_outside_peer_and_lost_owner_cannot_reach_wire(self):
        with self.reader.pose_servo({'motion_tool': 'servo_eef'}) as token:
            outside = RIGHT.copy()
            outside[0] = .10
            with self.assertRaises(MoveError):
                await self.reader.send_move_p(LEFT, outside, .2, token)
        with self.assertRaises(MoveError):
            await self.reader.send_move_p(LEFT, RIGHT, .2, token)
        self.socket.send_json.assert_not_awaited()
