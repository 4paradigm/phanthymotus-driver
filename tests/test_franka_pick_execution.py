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

    def test_first_confirmed_observe_homes_when_hand_reports_grasped(self):
        self.hand.snapshot.return_value['is_grasped'] = True
        self.arm.snapshot.side_effect = self.state

        def finish(_):
            self.hand.snapshot.return_value.update(is_grasped=False, sdk_received_ns=time.monotonic_ns())
            return dict(status='succeeded')

        self.hand.motion.side_effect = finish
        for _ in range(2):
            reply = self.card.dispatch('observe', dict(confirm_motion=True))
            self.assertEqual(reply['state'], 'completed', reply)
        self.hand.command.assert_called_once()
        self.assertEqual(self.hand.command.call_args.args[1], 'homing')

    def test_homing_does_not_release_a_possible_load_from_a_failed_transfer(self):
        self.hand.snapshot.return_value['is_grasped'] = True
        self.card.last_result = dict(result=dict(holding_object_possible=True))
        for _ in range(2):
            reply = self.card.dispatch('observe', dict(confirm_motion=True))
            self.assertEqual(reply['state'], 'error', reply)
            self.assertTrue(reply['result']['holding_object_possible'])
        self.hand.command.assert_not_called()

    def test_healthy_hand_needs_no_homing(self):
        self.arm.snapshot.side_effect = self.state
        reply = self.card.dispatch('observe', dict(confirm_motion=True))
        self.assertEqual(reply['state'], 'completed', reply)
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
                approaches, descent_starts, retreat_starts, planned_stages = [], [], [], []
                def prepare(stage, *args, **kwargs):
                    planned_stages.append(stage)
                    if stage == 'approach':
                        approaches.append(kwargs)
                        if len(approaches) == 1:
                            raise RuntimeError('MoveIt approach IK failed')
                        executor.q = [.25] * 7
                    if stage == 'descend':
                        descent_starts.append((list(executor.q), args[0], kwargs))
                        executor.q = [.5] * 7
                        if action == 'grab_to':
                            raise TableClearanceError('Insufficient table clearance')
                        return stage, {'descent_gap_bound_mm': 1.2}
                    return stage, None

                def shorten(target):
                    retreat_starts.append((list(executor.q), target))
                    executor.q = [.5] * 7
                    if action == 'grab_to' and len(retreat_starts) == 1:
                        raise TableClearanceError('Insufficient table clearance')
                    return 'descend', {'descent_gap_bound_mm': 1.2}

                executor.shorten_descent.side_effect = shorten
                executor.prepare.side_effect = prepare
                targets = dict(approach=1, descent=2, lift=3, carry=[4], place_lift=5)
                candidates = [{'observation_id': 'test', 'grasp_width_mm': 25, 'yaw_deg': yaw}
                              for yaw in (0, 30)]

                def grasp_targets(*_, extra_retreat_mm=0, only_yaw=None):
                    if only_yaw is None:
                        return candidates
                    return [dict(candidates[1], retreat=extra_retreat_mm)]

                def moveit_targets(_, pick, *args):
                    return dict(targets, descent=targets['descent'] + pick.get('retreat', 0))
                with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25}), \
                     patch('franka.fr3.pick_place.planning.grasp_targets', side_effect=grasp_targets) as geometry, \
                     patch('franka.fr3.pick_place.transfer.moveit_targets', side_effect=moveit_targets), \
                     patch('franka.fr3.pick_place.remote_execution.RemoteExecution', return_value=executor):
                    reply = self.card.dispatch(action, dict(confirm_motion=True, start_point_x=.1, start_point_y=.2, **extra))
                    self.assertEqual(reply['state'], 'running')
                    self.assertTrue(done.wait(3))
                self.assertEqual(self.card.last_result['state'], 'completed', self.card.last_result)
                self.assertEqual(geometry.call_args.args[1], {'diameter_mm': 25})
                self.assertEqual(approaches, [{'use_ik': True}, {'use_ik': True}])
                self.assertEqual(descent_starts, [([.25] * 7, [2], {'cartesian': True})])
                expected_retreats = [3, 4] if action == 'grab_to' else []
                self.assertEqual(retreat_starts, [([.25] * 7, goal) for goal in expected_retreats])
                self.assertEqual(self.card.last_result['result']['grasp_retreat_mm'],
                                 2 if action == 'grab_to' else 0)
                self.assertEqual(planned_stages, ['approach', 'approach', 'descend',
                                                 'lift', 'move_to_place', 'place_lift', 'return_to_observation'])
                self.assertEqual([c.args[0][0] for c in executor.move_remote.call_args_list],
                                 ['approach', 'descend', 'lift', 'move_to_place', 'place_lift', 'return_to_observation'])
                self.card.manager.acquire.assert_not_called()

    def test_remote_motion_failure_skips_grasp(self):
        done = Event()
        self.arm.snapshot.side_effect = lambda: self.state()
        self.card.callback = Mock(side_effect=lambda *args: done.set())
        executor = Mock(goals={})
        executor.fresh.return_value = dict(is_grasped=False, max_width=.08)
        executor.move_remote.side_effect = RuntimeError('Remote execution failed')
        with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25}), \
             patch('franka.fr3.pick_place.planning.grasp_targets', return_value=[{'grasp_width_mm': 25, 'yaw_deg': 0}]), \
             patch('franka.fr3.pick_place.transfer.moveit_targets',
                   return_value=dict(approach=1, descent=2, lift=3, carry=[4], place_lift=5)), \
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
        executor.fresh.return_value = dict(is_grasped=False, max_width=.08)
        executor.idle.side_effect = self.state
        executor.prepare.side_effect = lambda stage, *args, **kwargs: (stage, None)
        executor.grasp.side_effect = GraspMiss('Grasp was not confirmed')
        targets = dict(approach=1, descent=2, lift=3, carry=[4], place_lift=5)
        with patch('franka.fr3.pick_place.scene.estimate', return_value={'diameter_mm': 25}), \
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
