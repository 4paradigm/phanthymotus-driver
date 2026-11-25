"""Exercise the three public vision actions with a simulated remote controller."""
from contextlib import contextmanager
from threading import Event
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_franka_observe as fixtures
from franka.fr3.pick_place import joint_target


class PickExecutionTests(unittest.TestCase):
    def setUp(self):
        self.card = fixtures.ObserveTests().dual_plugin()
        self.card.enabled = True
        self.decode_depth = self.enterContext(patch(
            'franka.fr3.pick_place.alignment.decode_depth', return_value=np.zeros((1, 1))))
        self.destination = self.enterContext(patch(
            'franka.fr3.pick_place.transfer.destination', return_value=np.zeros(3)))
        self.card.config['force_n'] = 30
        self.card.planning_profile['handeye']['accepted'] = True
        self.card.controller_configuration = Mock()
        self.card.controller_configuration.snapshot.return_value = ('172.16.0.200', self.card.addresses, 'pnp', 'ros')
        self.card.manager = Mock()
        self.card.manager.acquire.side_effect = AssertionError('Unexpected direct arm connection')
        self.card.gripper_manager = Mock()
        self.hand = self.card.gripper_manager.acquire.return_value
        self.hand.snapshot.return_value = dict(connected=True, control_ready=True, data_age_ms=1,
                                                   is_grasped=False, max_width=.08)
        self.hand.motion.side_effect = self.finish_homing
        self.arm = Mock()
        self.card.controller = Mock(side_effect=self.started)

    @contextmanager
    def started(self, _):
        yield self.arm

    def state(self):
        return dict(connected=True, data_age_ms=1, robot_mode=1, control_ready=True,
                    q=np.radians(joint_target(self.card.config['observation_joints_deg'])).tolist(),
                    dq=[0.] * 7, O_T_EE=np.eye(4).flatten(order='F').tolist(),
                    F_T_EE=np.eye(4).flatten(order='F').tolist())

    def finish_homing(self, _):
        self.hand.snapshot.return_value.update(is_grasped=False, sdk_received_ns=time.monotonic_ns())
        return dict(status='succeeded')

    def test_each_confirmed_observe_homes_even_when_hand_is_already_calibrated(self):
        self.arm.snapshot.side_effect = self.state
        for _ in range(2):
            reply = self.card.dispatch('observe', dict(confirm_motion=True))
            self.assertEqual(reply['state'], 'completed', reply)
        self.assertEqual(self.hand.command.call_count, 2)
        self.assertTrue(all(call.args[1] == 'homing' for call in self.hand.command.call_args_list))

    def test_homing_does_not_release_a_possible_load_from_a_failed_transfer(self):
        self.hand.snapshot.return_value['is_grasped'] = True
        self.card.last_result = dict(result=dict(holding_object_possible=True))
        for _ in range(2):
            reply = self.card.dispatch('observe', dict(confirm_motion=True))
            self.assertEqual(reply['state'], 'error', reply)
            self.assertTrue(reply['result']['holding_object_possible'])
        self.hand.command.assert_not_called()

    def test_observe_uses_controller_and_returns_synchronously(self):
        self.card.config['speed_deg_s'] = 8.
        before = self.state()
        before['q'][0] += .1
        arrived = self.state()
        arrived['q'][6] += np.radians(.651)
        self.arm.snapshot.side_effect = iter([before] + [arrived] * 5)
        executor = Mock(goals={})
        executor.fresh.return_value = dict(is_grasped=False)
        with patch('franka.fr3.pick_place.remote_execution.RemoteExecution', return_value=executor) as factory:
            reply = self.card.dispatch('observe', dict(confirm_motion=True))
        self.assertEqual(reply['state'], 'completed', reply)
        self.assertEqual(factory.call_args.args[3]['speed_deg_s'], 8.)
        self.assertEqual(len(reply['result']['objects']), 1)
        self.card.controller.assert_called_once()
        executor.prepare.assert_called_once_with(
            'move_to_observation', self.state()['q'], joint=True)
        executor.move_remote.assert_called_once()
        self.card.manager.acquire.assert_not_called()

    def test_two_transfer_actions_plan_and_execute_with_auto_observe(self):
        from franka.fr3.pick_place.hand_collision import TableClearanceError

        for action, extra in (
            ('grab_to', dict(target_point_x=.3, target_point_y=.4)),
            ('grab_by', dict(delta_x=20, delta_y=0)),
        ):
            with self.subTest(action=action):
                done = Event()
                self.card.observation = None
                self.arm.snapshot.side_effect = lambda: self.state()
                self.card.callback = Mock(side_effect=lambda *args: done.set())
                executor = Mock(goals={})
                executor.fresh.return_value = dict(is_grasped=False, max_width=.08)
                executor.grasp.side_effect = lambda width: executor.fresh.return_value.update(is_grasped=True)
                approaches, descent_starts, planned_stages = [], [], []

                def prepare(stage, *args, **kwargs):
                    planned_stages.append(stage)
                    if stage == 'descend':
                        descent_starts.append((list(executor.q), args[0], kwargs))
                        return stage, {'descent_gap_bound_mm': 1.2}
                    return stage, None

                def check_target(target, stage):
                    if stage == 'descend' and (target >= 10 or (action == 'grab_to' and target < 4)):
                        raise TableClearanceError('Insufficient table clearance')

                def approach_candidates(target):
                    approaches.append(target)
                    return [(2, 'long'), (1, 'short')]

                def validate_approach(stage, target, plan, *args):
                    executor.q = [.25] * 7
                    return stage, plan

                executor.check_descent_target.side_effect = check_target
                executor.approach_candidates.side_effect = approach_candidates
                executor.prepare_trajectory.side_effect = validate_approach
                executor.prepare.side_effect = prepare
                targets = dict(approach=1, descent=2, lift=3, carry=[np.eye(4), np.eye(4)], place_lift=np.eye(4))
                candidates = [{'observation_id': 'test', 'grasp_width_mm': 25, 'yaw_deg': yaw}
                              for yaw in (0, 30)]

                def grasp_targets(*_, extra_retreat_mm=0, only_yaw=None):
                    if only_yaw is None:
                        return candidates
                    return [dict(candidates[only_yaw == 30], retreat=extra_retreat_mm)]

                def moveit_targets(_, pick, *args, **kwargs):
                    return dict(targets, approach=pick['yaw_deg'],
                                descent=targets['descent'] + pick.get('retreat', 0) + pick['yaw_deg'])
                with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25, 'table_normal_base': [0, 0, 1]}) as estimate, \
                     patch('franka.fr3.pick_place.planning.grasp_targets', side_effect=grasp_targets) as geometry, \
                     patch('franka.fr3.pick_place.transfer.moveit_targets', side_effect=moveit_targets), \
                     patch('franka.fr3.pick_place.remote_execution.RemoteExecution', return_value=executor):
                    reply = self.card.dispatch(action, dict(confirm_motion=True, start_point_x=.1, start_point_y=.2, **extra))
                    self.assertEqual(reply['state'], 'running')
                    self.assertTrue(done.wait(3))
                self.assertEqual(self.card.last_result['state'], 'completed', self.card.last_result)
                self.assertIs(estimate.call_args.kwargs['depth'], self.decode_depth.return_value)
                self.assertIs(self.destination.call_args.kwargs['depth'], self.decode_depth.return_value)
                self.assertEqual(self.decode_depth.call_count, self.destination.call_count)
                self.assertEqual(geometry.call_args.args[1], {'diameter_mm': 25, 'table_normal_base': [0, 0, 1]})
                self.assertEqual(approaches, [0])
                executor.prepare_trajectory.assert_called_once()
                self.assertEqual(executor.prepare_trajectory.call_args.args[2], 'short')
                depth = 4 if action == 'grab_to' else 2
                self.assertEqual(descent_starts, [([.25] * 7, [depth], {'cartesian': True})])
                self.assertEqual(self.card.last_result['result']['grasp_retreat_mm'],
                                 2 if action == 'grab_to' else 0)
                self.assertEqual(planned_stages, ['descend',
                                                 'lift', 'move_to_place', 'place_descend', 'place_lift', 'return_to_observation'])
                self.assertEqual([c.args[0][0] for c in executor.move_remote.call_args_list],
                                 ['approach', 'descend', 'lift', 'move_to_place', 'place_descend', 'place_lift', 'return_to_observation'])
                self.card.manager.acquire.assert_not_called()

    def test_remote_motion_failure_skips_grasp(self):
        done = Event()
        self.arm.snapshot.side_effect = lambda: self.state()
        self.card.callback = Mock(side_effect=lambda *args: done.set())
        executor = Mock(goals={})
        executor.approach_candidates.return_value = [(1, None)]
        executor.prepare_trajectory.return_value = ('approach', None)
        executor.fresh.return_value = dict(is_grasped=False, max_width=.08)
        executor.prepare.side_effect = lambda stage, *args, **kwargs: (stage, None)
        executor.move_remote.side_effect = RuntimeError('Remote execution failed')
        with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25, 'table_normal_base': [0, 0, 1]}), \
             patch('franka.fr3.pick_place.planning.grasp_targets', return_value=[{'grasp_width_mm': 25, 'yaw_deg': 0}]), \
             patch('franka.fr3.pick_place.transfer.moveit_targets',
                   return_value=dict(approach=1, descent=2, lift=3, carry=[np.eye(4), np.eye(4)], place_lift=np.eye(4))), \
             patch('franka.fr3.pick_place.remote_execution.RemoteExecution', return_value=executor):
            self.card.dispatch('grab_by', dict(confirm_motion=True, start_point_x=.1,
                                               start_point_y=.2, delta_x=20, delta_y=0))
            self.assertTrue(done.wait(3))
        self.assertEqual(self.card.last_result['state'], 'error')
        executor.grasp.assert_not_called()

    def test_confirmed_miss_opens_returns_and_captures_without_retry(self):
        from franka.fr3.pick_place.remote_execution import GraspMiss

        done = Event()
        self.arm.snapshot.side_effect = self.state
        self.card.callback = Mock(side_effect=lambda *args: done.set())
        executor = Mock(goals={})
        executor.approach_candidates.return_value = [(1, None)]
        executor.prepare_trajectory.return_value = ('approach', None)
        executor.fresh.return_value = dict(is_grasped=False, max_width=.08)
        executor.idle.side_effect = self.state
        executor.prepare.side_effect = lambda stage, *args, **kwargs: (stage, None)
        executor.grasp.side_effect = GraspMiss('Grasp was not confirmed')
        targets = dict(approach=1, descent=2, lift=3, carry=[np.eye(4), np.eye(4)], place_lift=np.eye(4))
        with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25, 'table_normal_base': [0, 0, 1]}), \
             patch('franka.fr3.pick_place.planning.grasp_targets',
                   return_value=[{'grasp_width_mm': 25, 'yaw_deg': 0}]), \
             patch('franka.fr3.pick_place.transfer.moveit_targets', return_value=targets), \
             patch('franka.fr3.pick_place.remote_execution.RemoteExecution', return_value=executor):
            self.card.dispatch('grab_by', dict(confirm_motion=True, start_point_x=.1,
                                               start_point_y=.2, delta_x=20, delta_y=0))
            self.assertTrue(done.wait(3))
        self.assertEqual([c.args[0] for c in executor.open_hand.call_args_list], ['pick_open', 'miss_open'])
        self.assertEqual([c.args[0][0] for c in executor.move_remote.call_args_list],
                         ['approach', 'descend', 'miss_retreat', 'return_to_observation'])
        result = self.card.last_result['result']
        self.assertTrue(result['return_completed'])
        self.assertFalse(result['transfer_completed'])
        self.assertFalse(result['recovery_required'])

        self.assertFalse(result['observe_after_transfer'])
        executor.grasp.assert_called_once()
        self.assertTrue(result['observation']['ok'])
        observation_id = result['observation']['observation_id']
        self.assertNotEqual(result['observation_id'], observation_id)
        self.assertEqual(self.card.observation['result']['observation_id'], observation_id)
        self.assertFalse(self.card.last_result['observation_required'])

    def test_unaccepted_calibration_never_acquires_hardware(self):
        self.card.planning_profile['handeye']['accepted'] = False
        for action in ('grab_to', 'grab_by'):
            reply = self.card.dispatch(action, dict(confirm_motion=True, start_point_x=.1,
                start_point_y=.2, target_point_x=.3, target_point_y=.4, delta_x=20, delta_y=0))
            self.assertEqual(reply['code'], 'CALIBRATION_NOT_ACCEPTED')
        self.card.gripper_manager.acquire.assert_not_called()
        self.card.controller.assert_not_called()


class HandOpeningTests(unittest.TestCase):
    def test_waits_for_post_command_width_without_reissuing_command(self):
        from franka.fr3.pick_place.remote_execution import RemoteExecution

        executor = Mock(goals={})
        executor.active = dict(owner='pick', cancel=Event())
        executor.options = dict(speed_mm_s=20)
        before = dict(width=.02, max_width=.08, is_grasped=True, connected=True,
                      control_ready=True, data_age_ms=1, sdk_received_ns=99)
        after = dict(before, width=.0795, is_grasped=False, sdk_received_ns=101)
        executor.fresh.return_value = before
        executor.hand.snapshot.side_effect = [before, after]
        with patch('franka.fr3.pick_place.remote_execution.time.monotonic_ns', return_value=100):
            self.assertEqual(RemoteExecution.open_hand(executor, 'place_open'), .08)
        executor.hand.command.assert_called_once()
        self.assertEqual(executor.hand.snapshot.call_count, 2)

    def test_partial_opening_still_reports_target_and_feedback(self):
        from franka.fr3.pick_place.remote_execution import RemoteExecution

        executor = Mock(goals={})
        executor.active = dict(owner='pick', cancel=Event())
        executor.options = dict(speed_mm_s=20)
        executor.fresh.return_value = dict(width=.02, max_width=.08, is_grasped=True)
        executor.hand.snapshot.return_value = dict(width=.078, is_grasped=False, connected=True,
                                                   control_ready=True, data_age_ms=1, sdk_received_ns=101)
        with patch('franka.fr3.pick_place.remote_execution.time.monotonic_ns', return_value=100), \
             patch('franka.fr3.pick_place.remote_execution.time.monotonic', side_effect=[1, 2]):
            with self.assertRaisesRegex(RuntimeError, 'target=80.00 mm, feedback=78.00 mm'):
                RemoteExecution.open_hand(executor, 'place_open')


class PlanningCacheTests(unittest.TestCase):
    def test_reuses_planner_but_not_local_arm_and_restarts_after_close(self):
        from franka.fr3.pick_place.controller_session import PlanningCache

        for_planner = Event()
        cache = PlanningCache()
        remote = Mock(local=None, alive=Mock(return_value=True))

        @contextmanager
        def start():
            yield remote
            for_planner.set()

        connection = ('192.0.2.10', 'pnp', 'test-password')
        with patch('franka.fr3.pick_place.controller_session.ControllerSession') as session:
            session.return_value.start.side_effect = start
            self.addCleanup(cache.close)
            warmed = cache.warm(connection, '192.0.2.1')
            cancel = Event()
            for local in (Mock(), Mock()):
                self.assertIs(cache.warm(connection, '192.0.2.1'), warmed)
                with cache.use(warmed, cancel, local) as lease:
                    self.assertIs(lease.local, local)
                    self.assertIs(lease.action_cancel, cancel)
                    self.assertIsNone(remote.local)
            session.assert_called_once()
            cache.close()
            self.assertTrue(for_planner.wait(2))
            restarted = cache.warm(connection, '192.0.2.1')
            self.assertIsNot(restarted, warmed)
            restarted.result(timeout=2)
            self.assertEqual(session.call_count, 2)
