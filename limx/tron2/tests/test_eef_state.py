"""Native DACH pose and gripper feedback, through the sensor's public lifecycle."""
import asyncio
import time
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import AsyncMock

from eef_state import EefState, FRAME, POSE_MAX_AGE, GRIPPER_MAX_AGE, STATE_TOPIC


class EefStateTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.poses = {'result': 'success', 'left_position': [.48, .20, -.38],
                      'left_quat': [.5, -.5, .5, -.5], 'right_position': [.10, -.21, -.73],
                      'right_quat': [.8, 0., -.6, 0.]}
        self.jaws = {'result': 'success', 'left_opening': 100, 'right_opening': 0,
                     'left_speed': 0, 'left_force': 0, 'right_speed': 0, 'right_force': 0}
        self.reader = SimpleNamespace(connected=True, ws=object(),
            configuration={'mode': 'upper_limb', 'state': 'identified'},
            pose_feedback=nullcontext,
            read_move_pose=AsyncMock(side_effect=lambda: dict(self.poses, timestamp=time.time_ns() // 1_000_000)),
            read_gripper=AsyncMock(side_effect=lambda: dict(self.jaws, timestamp=time.time_ns() // 1_000_000)))
        self.frames = asyncio.Queue()
        self.state = EefState(SimpleNamespace(reader=self.reader, enabled={'eef_state'},
            publish=lambda topic, data: self.frames.put_nowait((topic, data))))
        self.task = asyncio.create_task(self.state.run())

    async def asyncTearDown(self):
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)

    async def next_frame(self):
        topic, frame = await asyncio.wait_for(self.frames.get(), 1.5)
        self.assertEqual(topic, STATE_TOPIC)
        return frame

    async def test_native_pose_owns_feedback_and_quaternion_order(self):
        self.state.call('start')
        frame = await self.next_frame()
        self.assertEqual(set(frame), {'schema', 'frame', 'stamp_ms', 'eef'})
        self.assertEqual((frame['schema'], frame['frame']), ('motus.eef/1', FRAME))
        self.assertEqual(frame['eef'][:7], [.48, .20, -.38, -.5, .5, -.5, .5])
        self.assertAlmostEqual(frame['eef'][7], .085 * 100 / 94)
        # An out-of-workspace measured TCP is valid sensor data.
        self.assertEqual(frame['eef'][8:], [.10, -.21, -.73, 0., -.6, 0., .8, 0.])
        self.assertEqual(self.state.values(), frame['eef'])
        self.assertLessEqual(frame['stamp_ms'], time.time_ns() // 1_000_000)

    async def test_new_tcp_is_published_between_gripper_samples(self):
        self.reader.read_gripper.side_effect = lambda: dict(self.jaws, timestamp=1)
        self.state.call('start')
        first = await self.next_frame()
        self.poses['left_position'][0] += .003
        next_frame = await self.next_frame()
        self.assertAlmostEqual(next_frame['eef'][0] - first['eef'][0], .003)
        self.assertEqual(next_frame['eef'][7], first['eef'][7])

    async def test_standalone_sensor_only_polls_while_running(self):
        await asyncio.sleep(.12)
        self.reader.read_move_pose.assert_not_awaited()
        self.reader.read_gripper.assert_not_awaited()
        self.state.call('start')
        await self.next_frame()
        for _ in range(2):
            self.assertEqual(self.state.call('stop')['state'], 'idle')
        await asyncio.sleep(.25)
        polls = self.reader.read_move_pose.await_count, self.reader.read_gripper.await_count
        await asyncio.sleep(.25)
        self.assertEqual(polls, (self.reader.read_move_pose.await_count, self.reader.read_gripper.await_count))
        self.assertTrue(self.frames.empty())
        self.assertFalse(self.state.info()['available'])
        self.jaws['right_opening'] = 47
        self.state.call('start')
        async def updated_opening():
            while True:
                frame = await self.next_frame()
                if frame['eef'][15] != 0.:
                    return frame['eef'][15]
        self.assertAlmostEqual(await asyncio.wait_for(updated_opening(), 1.5), .0425)

    async def test_sensor_stop_keeps_servo_feedback_current_without_publishing(self):
        self.state.adapter.enabled.add('servo_eef')
        self.state.call('start')
        await self.next_frame()
        self.state.call('stop')
        self.jaws['right_opening'] = 47
        await asyncio.sleep(.35)
        self.assertTrue(self.frames.empty())
        self.assertFalse(self.state.info()['available'])
        self.assertAlmostEqual(self.state.values()[15], .0425)

    async def test_incomplete_or_invalid_response_never_publishes_partial_state(self):
        saved = self.poses.pop('left_position')
        self.state.call('start')
        await asyncio.sleep(.23)
        self.assertFalse(self.frames.qsize())
        self.poses['left_position'] = saved
        self.poses['left_quat'] = [0., 0., 0., 0.]
        await asyncio.sleep(.23)
        self.assertTrue(self.frames.empty())
        self.poses['left_quat'] = [1., 0., 0., 0.]
        self.jaws.pop('left_opening')
        await asyncio.sleep(.85)
        while not self.frames.empty():
            self.frames.get_nowait()
        await asyncio.sleep(.22)
        self.assertTrue(self.frames.empty())
        self.assertFalse(self.state.info()['available'])
        self.jaws['left_opening'] = 0
        self.assertEqual((await self.next_frame())['eef'][7], 0.)

    async def test_frozen_measurement_and_reconnection_discard_old_feedback(self):
        self.reader.read_move_pose.side_effect = lambda: dict(self.poses, timestamp=100.)
        self.state.call('start')
        frame = await self.next_frame()
        await asyncio.sleep(1.)
        self.assertFalse(self.state.info()['available'])
        while not self.frames.empty():
            self.assertEqual(self.frames.get_nowait()[1]['stamp_ms'], frame['stamp_ms'])
        self.reader.ws = object()
        self.reader.read_move_pose.side_effect = lambda: dict(self.poses, timestamp=time.time_ns() // 1_000_000)
        new = await self.next_frame()
        self.assertGreater(new['stamp_ms'], frame['stamp_ms'])
        self.assertTrue(self.state.info()['available'])

    async def test_late_read_and_its_fast_duplicate_are_both_stale(self):
        self.state._connected()
        for update, fields, max_age in ((self.state._update_poses, self.poses, POSE_MAX_AGE),
                                       (self.state._update_openings, self.jaws, GRIPPER_MAX_AGE)):
            with self.subTest(fields=fields):
                data = dict(fields, timestamp=100)
                for round_trip in (max_age + .05, 0.):
                    with self.assertRaises(ValueError):
                        update(data, round_trip)
                update(dict(data, timestamp=101), 0.)
