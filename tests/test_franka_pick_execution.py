"""Pick card flow with simulated session completion, failures and cancellation."""
import copy
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'franka/fr3')]
from franka.fr3.pick_place import PickPlacePlugin
from franka.fr3.pick_place.execution import PickExecution, parameters, arm_speed
from franka.fr3.pick_place import planning, scene, transfer


class Session:
    def __init__(self, q, hand=False):
        self.q, self.hand = q, hand
        self.commands, self.moves = [], []
        self.records = {}
        self.grasped = False
        self.fail_grasp = False
        self.stopped = []
        self.stale = False
        self.on_move = None
        self.ee = np.eye(4).flatten(order='F').tolist()

    def snapshot(self):
        return dict(connected=True, control_ready=True, data_age_ms=999 if self.stale else 1,
                    robot_mode=1, q=self.q, dq=[0]*7, is_grasped=self.grasped,
                    max_width=.0794, width=.0794, O_T_EE=self.ee)

    def move_path(self, owner, points, **options):
        target = points[-1]
        self.moves.append(list(target))
        self.q = list(target)
        command = 'arm'+str(len(self.moves))
        self.records[command] = dict(status='succeeded')
        if self.on_move:
            self.on_move()
        return command

    def command(self, owner, action, **options):
        self.commands.append((action, options))
        command = 'hand'+str(len(self.commands))
        if action == 'grasp':
            self.grasped = not self.fail_grasp
        self.records[command] = dict(status='failed' if action == 'grasp' and self.fail_grasp else 'succeeded',
                                     error='Gripper command returned false')
        return command

    def motion(self, command):
        return self.records[command]

    def wait(self, command, timeout):
        return dict(status='cancelled')

    def stop_motion(self, owner, command_id):
        self.stopped.append(command_id)
        return dict(status='cancelled')


class Manager:
    def __init__(self, session):
        self.session, self.released = session, 0

    def acquire(self, address, owner):
        return self.session

    def release(self, address, owner):
        self.released += 1


class PickExecutionTests(unittest.TestCase):
    def setUp(self):
        self.q = [0., 0., 0., -1., 0., 1., 0.]
        self.plan = dict(joint_path_rad=[self.q[:] for _ in range(301)],
            phase_end_indices=[100,200,300], observation_id='saved',
            grasp_centre_base_mm=[400,0,100], scene=dict(diameter_mm=65))
        self.arm = Session(self.q[:])
        self.hand = Session(self.q[:], hand=True)
        self.completions = []
        self.plugin = PickPlacePlugin(
            {'motion_arm_ips':{'right':'172.16.0.3'},'vision_pick_and_drop':{'planning':{'handeye':{}},'force_n':30}},
            Manager(self.arm),None,None,gripper_manager=Manager(self.hand),
            callback=lambda *args:self.completions.append(args))
        self.plugin.observation = {'metadata':{'O_T_EE':np.eye(4).flatten(order='F').tolist()}}
        self.args = dict(start_point_x=0,start_point_y=0,speed_deg_s=2,speed_mm_s=20,
                         confirm_motion=True,delta_x=30,delta_y=0)

    def run_card(self, action='grab_by', releases=1, transfer_error=None, observation_required=None):
        with mock.patch.object(scene,'estimate',return_value=self.plan['scene']), \
             mock.patch.object(planning,'preview',return_value=self.plan), \
             mock.patch.object(transfer,'plan_transfer',return_value=self.transfer_plan(),side_effect=transfer_error):
            accepted = self.plugin.dispatch(action,self.args)
            self.assertEqual(accepted['state'],'running')
            deadline = time.monotonic()+3
            while self.plugin.active is not None and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertIsNone(self.plugin.active,'Worker failed to finish')
        self.assertEqual(self.plugin.last_result['observation_required'],releases!=0 if observation_required is None else observation_required)
        self.assertEqual(self.plugin.manager.released,releases)
        self.assertEqual(self.plugin.gripper_manager.released,releases)
        self.assertEqual(self.completions[0][0],accepted['action_id'])
        return self.plugin.last_result

    def test_card_success_opens_grasps_lifts_and_reports_acp(self):
        result = self.run_card()
        self.assertEqual(result['state'],'completed')
        self.assertEqual(len(self.arm.moves),6)
        self.assertEqual([x[0] for x in self.hand.commands],['set_position','grasp','set_position'])
        self.assertTrue(result['result']['return_completed'])
        self.assertEqual(result['callback'],'accepted')
        self.assertTrue(result['result']['transfer_completed'])
        self.assertFalse(result['result']['observe_after_transfer'])
        self.assertEqual(result['result']['observation'],dict(skipped=True,reason='disabled'))
        self.assertNotIn('returned_to_observation',result['result'])

    def test_failed_grasp_prevents_all_lift_targets(self):
        self.hand.fail_grasp = True
        result = self.run_card()
        self.assertEqual(result['state'],'error')
        self.assertEqual(len(self.arm.moves),2)
        self.assertEqual(result['result']['stage'],'pick_close')
        self.assertNotIn('stop_error',result['result'])
        self.assertFalse(self.hand.stopped)

    def test_tool_frame_change_prevents_opening(self):
        self.arm.ee[12] = .01
        result = self.run_card()
        self.assertEqual(result['state'],'error')
        self.assertFalse(self.hand.commands)

    def transfer_plan(self):
        return dict(paths=[[self.q[:] for _ in range(21)] for _ in range(3)],
                    stages=['move_to_place','place_lift','return_to_observation'],
                    result=dict(delta_x=30,delta_y=0,rotation_deg=0,direction_reference='observation_image'))

    def test_grab_by_runs_full_cycle_and_reports_terminal_acp(self):
        self.args.update(delta_x=30,delta_y=0)
        result=self.run_card('grab_by')
        self.assertEqual(result['state'],'completed')
        self.assertEqual(len(self.arm.moves),6)
        self.assertEqual([c[0] for c in self.hand.commands],['set_position','grasp','set_position'])
        self.assertTrue(result['result']['release_completed'])
        self.assertTrue(result['result']['return_completed'])
        self.assertEqual(result['result']['delta_x'],30)
        self.assertEqual(result['callback'],'accepted')

    def test_transfer_planning_failure_does_not_touch_robot_or_gripper(self):
        self.args.update(target_point_x=.2,target_point_y=0)
        result=self.run_card('grab_to',releases=0,transfer_error=ValueError('No place path'))
        self.assertEqual(result['state'],'error')
        self.assertFalse(self.arm.moves);self.assertFalse(self.hand.commands)

    def test_cancel_after_release_does_not_start_return(self):
        self.args.update(delta_x=30,delta_y=0)
        original=self.hand.command
        def cancel_after_open(owner,action,**options):
            command=original(owner,action,**options)
            if len(self.hand.commands)==3:self.plugin.cancel()
            return command
        self.hand.command=cancel_after_open
        result=self.run_card('grab_by')
        self.assertEqual(result['state'],'cancelled')
        self.assertEqual(len(self.arm.moves),4)

    def test_return_failure_reports_transfer_complete_without_claiming_return(self):
        def fail_return():
            if len(self.arm.moves)==6:
                self.arm.records['arm6']=dict(status='failed',error='Return failed')
        self.arm.on_move=fail_return
        terminal=self.run_card()
        result=terminal['result']
        self.assertEqual(terminal['state'],'error')
        self.assertTrue(result['transfer_completed']);self.assertFalse(result['return_completed'])
        self.assertTrue(result['release_completed']);self.assertFalse(result['holding_object_possible'])
        self.assertNotIn('final_pose',result)
        self.assertEqual(result['last_completed_stage'],'place_lift')


if __name__ == '__main__':
    unittest.main()
