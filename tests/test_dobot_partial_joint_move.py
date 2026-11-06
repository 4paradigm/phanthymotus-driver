"""Omitted joints stay at their actual pose for absolute and relative moves."""
import time

import pytest

from common.vendor_runtime import DriverBundle
from dobot.cra.device import DobotSensors
from test_dobot_joint_move import ready

POSE = [20, 30, 40, 50, 60, 70]


@pytest.mark.parametrize('action', ['move_j', 'rel_joint_mov_j'])
@pytest.mark.parametrize('blank', [None, '', '   '])
@pytest.mark.parametrize('index', range(6))
def test_only_supplied_joint_changes_with_other_inputs_blank(action, blank, index):
    feedback, dashboard, control, _ = ready()
    feedback.sample['data']['joint_state']['q_actual'] = list(POSE)
    suffix = '_deg' if action == 'move_j' else '_offset_deg'
    args = {f'joint{i+1}{suffix}': blank for i in range(6)}
    args.update({f'joint{index+1}{suffix}': 5, 'confirm_move': True})
    expected = list(POSE)
    expected[index] = 5 if action == 'move_j' else POSE[index] + 5
    try:
        result = control.execute_motion(action, args)
        assert result['target_joint_angles_deg'] == expected
        if action == 'rel_joint_mov_j':
            assert all(result[f'joint{i+1}_offset_deg'] == (5 if i == index else 0) for i in range(6))
    finally:
        control.close()


@pytest.mark.parametrize('action', ['move_j', 'rel_joint_mov_j'])
@pytest.mark.parametrize('blank', ['missing', None, '', '  '])
def test_no_joint_input_is_rejected_before_any_commands(action, blank):
    _, dashboard, control, calls = ready()
    suffix = '_deg' if action == 'move_j' else '_offset_deg'
    args = {'confirm_move': True}
    if blank != 'missing':
        args.update({f'joint{i+1}{suffix}': blank for i in range(6)})
    with pytest.raises(ValueError, match='at least one'):
        control.execute_motion(action, args)
    assert not calls and not dashboard.commands


def test_omitted_absolute_joints_use_pose_after_preparation_not_initial_pose():
    feedback, dashboard, control, _ = ready()
    feedback.sample['data']['joint_state']['q_actual'] = list(POSE)
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableSafeSkin(0)':
            feedback.sample['data']['joint_state']['q_actual'][1] = 35
        return reply
    dashboard.send_receive_text = send
    try:
        result = control.execute_motion('move_j', {'joint1_deg': 0, 'confirm_move': True})
        assert result['target_joint_angles_deg'] == [0, 35, 40, 50, 60, 70]
        assert dashboard.commands[-1] == 'MovJ(joint={0.000000,35.000000,40.000000,50.000000,60.000000,70.000000},a=10,v=10,cp=0)'
    finally:
        control.close()


def test_partial_absolute_move_completion_uses_resolved_target():
    feedback, dashboard, control, _ = ready()
    feedback.sample['data']['joint_state']['q_actual'] = list(POSE)
    plugin = DobotSensors({}, client=feedback, controller=control)
    events = []
    plugin.completion._notify = lambda *event: events.append(event)
    original_send, original_snapshot = dashboard.send_receive_text, feedback.snapshot
    count = [0]
    def send(command):
        reply = original_send(command)
        if command.startswith('MovJ('):
            feedback.sample['data']['joint_state']['q_actual'] = [0, 30, 40, 50, 60, 70]
            feedback.sample['data']['current_command_id'] = 4
        return reply
    def snapshot():
        count[0] += 1
        return {**original_snapshot(), 'packets_received': count[0]}
    dashboard.send_receive_text, feedback.snapshot = send, snapshot
    try:
        result = DriverBundle([plugin]).dispatch('joint_move', {'action': 'move_j', 'joint1_deg': 0, 'confirm_move': True})
        deadline = time.monotonic() + 1
        while not events and time.monotonic() < deadline:
            time.sleep(0.001)
        assert events[0][0] == result['action_id'] and events[0][1] == 'completed'
        assert control.snapshot()['last_request']['target_joint_angles_deg'] == [0, 30, 40, 50, 60, 70]
    finally:
        plugin.stop()
