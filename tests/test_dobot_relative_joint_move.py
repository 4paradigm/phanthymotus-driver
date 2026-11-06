"""Relative SDK motion, endpoint bounds, preparation and ACP contracts."""
import time

import pytest

from common.vendor_runtime import DriverBundle
from dobot.cra.device import DobotSensors
from test_dobot_joint_move import ready, set_mode

OFFSETS = {f'joint{i}_offset_deg': value for i, value in enumerate((10, 0, -5, 0, 0, 0), 1)}
REL = 'RelJointMovJ(10.000000,0.000000,-5.000000,0.000000,0.000000,0.000000,a=10,v=10,cp=0)'


def relative(control, **changes):
    return control.execute_motion('rel_joint_mov_j', {**OFFSETS, 'confirm_move': True, **changes})


def test_relative_motion_uses_official_sdk_and_existing_preparation_order():
    feedback, dashboard, control, _ = ready()
    feedback.sample['data']['joint_state']['q_actual'] = [20, 10, 30, 0, 0, -10]
    try:
        result = relative(control)
        assert dashboard.commands == ['ClearError()', 'EnableSafeSkin(0)', 'ClearError()', REL]
        assert result['start_joint_angles_deg'] == [20, 10, 30, 0, 0, -10]
        assert result['target_joint_angles_deg'] == [30, 10, 25, 0, 0, -10]
        assert result['detect_collision'] is False
        assert result['velocity'] == 10 and result['acceleration'] == 10
    finally:
        control.close()


@pytest.mark.parametrize('joint', list(OFFSETS))
@pytest.mark.parametrize('bad', [True, False, '10', float('nan'), float('inf'), 10**1000])
def test_invalid_relative_offsets_send_no_commands(joint, bad):
    _, dashboard, control, calls = ready()
    with pytest.raises(ValueError, match=joint):
        relative(control, **{joint: bad})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('joint', list(OFFSETS))
def test_missing_relative_offset_defaults_to_zero(joint):
    _, dashboard, control, _ = ready()
    args = {**OFFSETS, 'confirm_move': True}
    del args[joint]
    try:
        result = control.execute_motion('rel_joint_mov_j', args)
        assert result[joint] == 0
    finally:
        control.close()


@pytest.mark.parametrize('index,limit', [(0, 360), (1, 360), (2, 155), (3, 360), (4, 360), (5, 360)])
@pytest.mark.parametrize('sign', [-1, 1])
def test_actual_angle_plus_offset_is_checked_for_every_joint(index, limit, sign):
    feedback, dashboard, control, calls = ready()
    feedback.sample['data']['joint_state']['q_actual'] = [0] * 6
    feedback.sample['data']['joint_state']['q_actual'][index] = sign * (limit - 1)
    offsets = {name: 0 for name in OFFSETS}
    offsets[f'joint{index+1}_offset_deg'] = sign * 2
    with pytest.raises(ValueError, match=f'relative target for joint{index+1}'):
        relative(control, **offsets)
    assert not calls and not dashboard.commands


def test_relative_endpoint_is_rechecked_after_preparation_changes_pose():
    feedback, dashboard, control, _ = ready()
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableSafeSkin(0)':
            feedback.sample['data']['joint_state']['q_actual'][0] = 355
        return reply
    dashboard.send_receive_text = send
    with pytest.raises(ValueError, match='relative target for joint1'):
        relative(control)
    assert not any(c.startswith('RelJointMovJ(') for c in dashboard.commands)
    assert dashboard.commands == ['ClearError()', 'EnableSafeSkin(0)', 'ClearError()']


def test_relative_completion_target_uses_pose_just_before_submission():
    feedback, dashboard, control, _ = ready()
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableSafeSkin(0)':
            feedback.sample['data']['joint_state']['q_actual'][0] = 20
        return reply
    dashboard.send_receive_text = send
    try:
        result = relative(control)
        assert result['start_joint_angles_deg'][0] == 20
        assert result['target_joint_angles_deg'][0] == 30
    finally:
        control.close()


@pytest.mark.parametrize('confirmation', [None, False, 'true', 1])
def test_relative_confirmation_is_still_exact_manual_boolean(confirmation):
    _, dashboard, control, calls = ready()
    with pytest.raises(ValueError, match='confirm_move'):
        relative(control, confirm_move=confirmation)
    assert not calls and not dashboard.commands


def test_relative_custom_rates_and_skin_low_share_existing_settings():
    _, dashboard, control, _ = ready()
    original = dashboard.send_receive_text
    def send(command):
        if command == 'EnableSafeSkin(1)' or command.startswith('SetSafeSkin('):
            dashboard.commands.append(command)
            return f'0,{{}},{command};'
        return original(command)
    dashboard.send_receive_text = send
    try:
        relative(control, velocity=25, acceleration=15, detect_collision=True)
        assert dashboard.commands == ['ClearError()', 'EnableSafeSkin(1)'] + [f'SetSafeSkin({p},1)' for p in (3,4,5,6)] + ['ClearError()', REL.replace('a=10,v=10', 'a=15,v=25')]
    finally:
        control.close()


def test_relative_action_schema_and_acp_completion():
    feedback, dashboard, control, _ = ready()
    feedback.sample['data']['joint_state']['q_actual'] = [20, 10, 30, 0, 0, -10]
    plugin = DobotSensors({}, client=feedback, controller=control)
    events = []
    plugin.completion._notify = lambda *event: events.append(event)
    schema = next(t for t in plugin.get_tools() if t['name'] == 'joint_move')['inputSchema']
    assert 'rel_joint_mov_j' in schema['x-completion']['actions']
    assert schema['x-action-params']['rel_joint_mov_j']['params'] == [*OFFSETS, 'velocity', 'acceleration', 'detect_collision', 'confirm_move']
    original_send, original_snapshot = dashboard.send_receive_text, feedback.snapshot
    packets = [0]
    def send(command):
        reply = original_send(command)
        if command.startswith('RelJointMovJ('):
            feedback.sample['data']['joint_state']['q_actual'] = [30, 10, 25, 0, 0, -10]
            feedback.sample['data']['current_command_id'] = 4
        return reply
    def snapshot():
        packets[0] += 1
        return {**original_snapshot(), 'packets_received': packets[0]}
    dashboard.send_receive_text, feedback.snapshot = send, snapshot
    try:
        result = DriverBundle([plugin]).dispatch('joint_move', {'action': 'rel_joint_mov_j', **OFFSETS, 'confirm_move': True})
        assert result['state'] == 'running'
        deadline = time.monotonic() + 1
        while not events and time.monotonic() < deadline:
            time.sleep(0.001)
        assert events[0][0] == result['action_id'] and events[0][1] == 'completed'
        assert events[0][2]['reason'] == 'target_reached'
        assert dashboard.commands[-1] == REL
    finally:
        plugin.stop()


@pytest.mark.parametrize('case', ['stale', 'other_model', 'running', 'paused_queue'])
def test_relative_motion_keeps_existing_preparation_guards(case):
    feedback, dashboard, control, calls = ready()
    if case == 'stale': feedback.sample['stale'] = True
    if case == 'other_model': feedback.sample['robot_type_code'] = 130
    if case == 'running': set_mode(feedback, 7)
    if case == 'paused_queue': feedback.sample['data']['pause_cmd_flag'] = 1
    with pytest.raises(RuntimeError):
        relative(control)
    assert not calls and not dashboard.commands
