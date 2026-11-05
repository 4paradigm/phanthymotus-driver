"""Joint-motion contracts with the pinned SDK and fake transports only."""

import threading

import pytest

from common.vendor_runtime import DriverBundle
from dobot.cra.control import CommandRejected
from dobot.cra.device import DobotSensors
import copy
import numpy as np
from dobot.cra.control import RobotController
from dobot.cra.feedback import ROBOT_MODES, decode_packet, endpoint
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC

class Feedback:
    address = "127.0.0.1"
    port = 30004
    model_name = "CR3A"

    def __init__(self):
        record = np.zeros(1, dtype=MyType)
        record['len'] = 1440
        record['TestValue'] = TEST_VALUE_MAGIC
        record['RobotMode'] = 4
        record['CRRobotType'] = 113
        self.sample = dict(connected=True, stale=False, controller_ip=self.address,
                           robot_type_code=113, data=decode_packet(record.tobytes()))
        self.starts = self.stops = 0

    def snapshot(self):
        return copy.deepcopy(self.sample)

    def start(self):
        self.starts += 1

    def stop(self):
        self.stops += 1

    def configure(self, address, port):
        self.address, self.port = endpoint(address, port)
        self.sample.update(controller_ip=address, connected=False, stale=True, data=None)


class Dashboard:
    def __init__(self, reply=None, failure=None):
        self.commands = []
        self.reply = reply
        self.failure = failure
        self.closed = False
        self.connects = 0
        self.on_connect = None

    def set_timeout(self, *args):
        self.timeouts = args

    def connect(self):
        self.connects += 1
        if self.on_connect:
            self.on_connect()

    def disconnect(self):
        self.closed = True

    def send_receive_text(self, command):
        self.commands.append(command)
        if self.failure:
            raise self.failure
        if self.reply is not None:
            return self.reply
        return f"0,{{{len(self.commands)}}},{command};"


def setup_control(dashboard=None):
    feedback = Feedback()
    dashboard = Dashboard() if dashboard is None else dashboard
    factories = []

    def factory(address, port):
        factories.append((address, port))
        return dashboard

    controller = RobotController(feedback, {}, connection_factory=factory)
    return feedback, dashboard, controller, factories


def set_mode(feedback, mode):
    feedback.sample['data'].update(
        robot_mode=mode, robot_mode_name=ROBOT_MODES[mode],
        enable_status=mode in (5, 6, 7, 8, 10), running_status=int(mode in (7, 8)),
        drag_status=int(mode == 6), pause_cmd_flag=int(mode == 10), error_status=mode == 9,
    )


TARGET = dict(joint1_deg=10, joint2_deg=-30, joint3_deg=60,
              joint4_deg=0, joint5_deg=90, joint6_deg=-10)
MOVJ = 'MovJ(joint={10.000000,-30.000000,60.000000,0.000000,90.000000,-10.000000},a=10,v=10,cp=0)'


def ready(dashboard=None):
    feedback, dashboard, control, calls = setup_control(dashboard)
    set_mode(feedback, 5)
    control.enable_timeout = 0.05
    original = dashboard.send_receive_text
    def send(command):
        if command == 'EnableSafeSkin(0)':
            dashboard.commands.append(command)
            return '0,{},EnableSafeSkin(0);'
        if command == 'ClearError()' and (dashboard.reply is not None or dashboard.failure is not None):
            dashboard.commands.append(command)
            return '0,{},ClearError();'
        return original(command)
    dashboard.send_receive_text = send
    return feedback, dashboard, control, calls


def move(control, **changes):
    return control.execute_motion('move_j', {**TARGET, 'confirm_move': True, **changes})


def test_absolute_degrees_use_joint_coordinates_and_explicit_default_rates():
    feedback, dashboard, control, calls = ready()
    bundle = DriverBundle([DobotSensors({}, client=feedback, controller=control)])
    try:
        result = bundle.dispatch('joint_move', {'action': 'move_j', **TARGET, 'confirm_move': True})
        assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ] and len(calls) == 1
        assert result['state'] == 'submitted' and result['result_id'] == 3
        assert result['effect_verified'] is False
        assert {key: result[key] for key in TARGET} == TARGET
        assert result['velocity'] == 10 and result['acceleration'] == 10
        assert control.snapshot('joint_move')['last_request'] == result
    finally:
        control.close()


def test_custom_rates_are_per_command_without_global_speed_or_coordinate_settings():
    _, dashboard, control, calls = ready()
    try:
        result = move(control, velocity=25, acceleration=15)
        assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ.replace('a=10,v=10', 'a=15,v=25')]
        assert result['velocity'] == 25 and result['acceleration'] == 15 and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('joint', list(TARGET))
@pytest.mark.parametrize('bad', [None, True, False, '10', float('nan'), float('inf'), float('-inf'), 10**1000])
def test_invalid_joint_values_never_open_command_connection(joint, bad):
    _, dashboard, control, calls = ready()
    with pytest.raises(ValueError, match=joint):
        move(control, **{joint: bad})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('joint', list(TARGET))
def test_every_joint_is_required_and_has_no_implicit_zero(joint):
    _, dashboard, control, calls = ready()
    args = {**TARGET, 'confirm_move': True}
    del args[joint]
    with pytest.raises(ValueError, match=joint):
        control.execute_motion('move_j', args)
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('joint', list(TARGET))
@pytest.mark.parametrize('sign', [-1, 1])
def test_documented_cr3a_degree_limits_are_enforced(joint, sign):
    _, dashboard, control, calls = ready()
    limit = 155 if joint == 'joint3_deg' else 360
    with pytest.raises(ValueError, match=joint):
        move(control, **{joint: sign * (limit + 0.01)})
    assert not calls and not dashboard.commands
    try:
        move(control, **{joint: sign * limit})
        assert len(calls) == 1 and len(dashboard.commands) == 3
    finally:
        control.close()


@pytest.mark.parametrize('field', ['velocity', 'acceleration'])
@pytest.mark.parametrize('bad', [None, True, False, 0, 101, -1, 10.5, '10'])
def test_invalid_rate_is_rejected_before_connection(field, bad):
    _, dashboard, control, calls = ready()
    with pytest.raises(ValueError, match=field):
        move(control, **{field: bad})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action', ['move_j', 'stop_motion', 'pause', 'resume'])
@pytest.mark.parametrize('confirmation', [None, False, 1, 'true'])
def test_each_motion_action_requires_exact_boolean_confirmation(action, confirmation):
    _, dashboard, control, calls = ready()
    with pytest.raises(ValueError, match='confirm_move'):
        control.execute_motion(action, {**TARGET, 'confirm_move': confirmation})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('changed', [{'stale': True}, {'connected': False}, {'data': None}])
@pytest.mark.parametrize('action,mode', [('move_j', 5), ('pause', 7), ('resume', 10)])
def test_motion_and_resume_require_fresh_feedback(action, mode, changed):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    feedback.sample.update(changed)
    with pytest.raises(RuntimeError, match='fresh feedback'):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,mode', [('move_j', 5), ('resume', 10)])
@pytest.mark.parametrize('changed', [
    {'running_status': 1}, {'jog_status': 1},
    {'drag_status': 1}, {'brake_released': [True] * 6},
])
def test_motion_does_not_bypass_protection(action, mode, changed):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    feedback.sample['data'].update(changed)
    with pytest.raises(RuntimeError):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('mode', [3, 6, 7, 8, 10])
def test_new_absolute_move_requires_idle_enabled_mode(mode):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    with pytest.raises(RuntimeError):
        move(control)
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('changed', [{'pause_cmd_flag': 1}, {'run_queued_cmd': 1}])
def test_new_move_does_not_append_to_paused_or_running_queue(changed):
    feedback, dashboard, control, calls = ready()
    feedback.sample['data'].update(changed)
    with pytest.raises(RuntimeError):
        move(control)
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('robot_type', [None, 0, 130])
def test_unknown_or_other_model_is_not_given_cr3a_motion_limits(robot_type):
    feedback, dashboard, control, calls = ready()
    feedback.sample['robot_type_code'] = robot_type
    with pytest.raises(RuntimeError, match='CR3A'):
        move(control)
    assert not calls and not dashboard.commands


def test_configured_model_must_agree_with_controller_identity():
    feedback, dashboard, control, calls = ready()
    feedback.model_name = 'CR5A'
    with pytest.raises(ValueError, match='disagrees'):
        move(control)
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,mode,command', [('pause', 7, 'Pause()'), ('resume', 10, 'Continue()')])
def test_pause_and_resume_use_sdk_without_chained_actions(action, mode, command):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    try:
        control.execute_motion(action, {'confirm_move': True})
        assert dashboard.commands == (['EnableSafeSkin(0)', command] if action == 'resume' else [command]) and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('action,mode', [('pause', 8), ('pause', 5), ('resume', 5), ('resume', 7)])
def test_pause_cannot_pause_jogging_and_resume_requires_paused_mode(action, mode):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    with pytest.raises(RuntimeError):
        control.execute_motion(action, {'confirm_move': True})
    if action == 'resume' and mode == 5:
        assert dashboard.commands == ['EnableSafeSkin(0)']
        assert len(calls) == 1
    else:
        assert not calls and not dashboard.commands


def test_explicit_stop_remains_possible_when_feedback_is_unavailable():
    feedback, dashboard, control, calls = ready()
    feedback.sample.update(connected=False, stale=True, data=None)
    try:
        control.execute_motion('stop_motion', {'confirm_move': True})
        assert dashboard.commands == ['Stop()'] and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('action', ['move_j', 'stop_motion', 'pause', 'resume'])
def test_unconfigured_endpoint_cannot_send_motion(action):
    feedback, dashboard, control, calls = ready()
    feedback.address = ''
    with pytest.raises(RuntimeError, match='controller IP'):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert not calls and not dashboard.commands


def test_robot_state_is_checked_again_after_connecting():
    feedback, dashboard, control, calls = ready()
    dashboard.on_connect = lambda: feedback.sample['data'].update(running_status=1)
    with pytest.raises(RuntimeError):
        move(control)
    assert not dashboard.commands and len(calls) == 1 and dashboard.closed


@pytest.mark.parametrize('failure', [TimeoutError('lost ack'), ConnectionError('connection lost')])
def test_uncertain_move_is_not_replayed_or_reported_as_arrival(failure):
    _, dashboard, control, calls = ready(Dashboard(failure=failure))
    with pytest.raises(type(failure)):
        move(control)
    request = control.snapshot('joint_move')['last_request']
    assert request['state'] == 'failed_or_outcome_unknown' and request['effect_verified'] is False
    assert len(calls) == 1 and dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ] and dashboard.closed


def test_controller_rejection_is_retained_only_in_motion_history():
    _, dashboard, control, calls = ready(Dashboard(reply=f'-1,{{}},{MOVJ};'))
    with pytest.raises(CommandRejected):
        move(control)
    assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ] and len(calls) == 1
    assert control.snapshot('joint_move')['last_request']['state'] == 'rejected'


def test_empty_success_ack_does_not_claim_move_completed():
    _, dashboard, control, calls = ready(Dashboard(reply=f'0,{{}},{MOVJ};'))
    try:
        result = move(control)
        assert result['state'] == 'accepted_unverified' and result['result_id'] is None
        assert result['effect_verified'] is False and len(calls) == 1
    finally:
        control.close()


def test_motion_schema_has_only_the_four_public_actions_and_no_default_angles():
    feedback, dashboard, control, calls = ready()
    plugin = DobotSensors({}, client=feedback, controller=control)
    tool = plugin.dispatch('info', {'_tool_name': 'joint_move'})
    assert tool['type'] == 'actuator' and 'topic_out' not in tool
    schema = tool['inputSchema']
    assert schema['x-is-dangerous'] and schema['x-resource'] == 'arm'
    assert set(schema['properties']['action']['enum']) - {'info', 'start', 'stop', 'config'} == {
        'move_j', 'stop_motion', 'pause', 'resume',
    }
    for key in TARGET:
        assert 'default' not in schema['properties'][key]
        assert key in schema['x-action-params']['move_j']['params']
    for action in ('get_status', 'power_on', 'request_control', 'enable'):
        with pytest.raises(ValueError, match='unsupported joint motion action'):
            plugin.dispatch(action, {'_tool_name': 'joint_move', 'confirm_move': True})
    for action in ('start', 'config', 'stop', 'info'):
        assert plugin.dispatch(action, {'_tool_name': 'joint_move'})
    assert not calls and not dashboard.commands



def test_each_move_clears_error_even_on_reused_connection():
    _, dashboard, control, calls = ready()
    try:
        move(control)
        move(control)
        assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ, 'EnableSafeSkin(0)', 'ClearError()', MOVJ]
        assert len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('reply', ['-1,{},ClearError();', 'malformed'])
def test_rejected_or_invalid_clear_never_sends_move(reply):
    _, dashboard, control, _ = ready()
    dashboard.reply = reply
    def send(command):
        if command == 'EnableSafeSkin(0)':
            dashboard.commands.append(command)
            return '0,{},EnableSafeSkin(0);'
        return Dashboard.send_receive_text(dashboard, command)
    dashboard.send_receive_text = send
    with pytest.raises((CommandRejected, ValueError)):
        move(control)
    assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()'] and dashboard.closed


def test_clear_timeout_never_sends_move_or_retries():
    _, dashboard, control, calls = ready()
    dashboard.failure = TimeoutError('clear acknowledgement lost')
    def send(command):
        if command == 'EnableSafeSkin(0)':
            dashboard.commands.append(command)
            return '0,{},EnableSafeSkin(0);'
        return Dashboard.send_receive_text(dashboard, command)
    dashboard.send_receive_text = send
    with pytest.raises(TimeoutError):
        move(control)
    assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()'] and len(calls) == 1


def test_protection_state_is_checked_again_after_clear():
    feedback, dashboard, control, _ = ready()
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'ClearError()':
            feedback.sample['data']['collision_state'] = 1
        return reply
    dashboard.send_receive_text = send
    with pytest.raises(RuntimeError):
        move(control)
    assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()'] and dashboard.closed


@pytest.mark.parametrize('action,mode', [('move_j', 5), ('resume', 10)])
@pytest.mark.parametrize('failure', ['rejected', 'malformed', 'timeout'])
def test_skin_disable_failure_blocks_motion_and_resume(action, mode, failure):
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, mode)
    def send(command):
        dashboard.commands.append(command)
        if failure == 'timeout':
            raise TimeoutError('skin disable acknowledgement lost')
        return '-1,{},EnableSafeSkin(0);' if failure == 'rejected' else 'invalid'
    dashboard.send_receive_text = send
    with pytest.raises((CommandRejected, ValueError, TimeoutError)):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert dashboard.commands == ['EnableSafeSkin(0)']
    assert len(calls) == 1 and dashboard.closed


def test_each_resume_disables_skin_on_reused_connection():
    feedback, dashboard, control, calls = ready()
    set_mode(feedback, 10)
    try:
        for _ in range(2):
            result = control.execute_motion('resume', {'confirm_move': True})
            assert result['safe_skin_disable_response'].startswith('0,')
        assert dashboard.commands == ['EnableSafeSkin(0)', 'Continue()'] * 2
        assert len(calls) == 1
    finally:
        control.close()


def test_joint_move_can_start_and_configure_without_starting_joints():
    feedback, dashboard, control, calls = ready()
    plugin = DobotSensors({}, client=feedback, controller=control)
    result = plugin.dispatch('config', {'_tool_name': 'joint_move', 'controller_ip': '127.0.0.2', 'feedback_port': 30006})
    assert result['state'] == 'configured'
    assert feedback.address == '127.0.0.2' and feedback.port == 30006
    assert plugin.dispatch('start', {'_tool_name': 'joint_move'})['state'] == 'ready'
    assert feedback.starts == 1 and not calls and not dashboard.commands
    assert next(t for t in plugin.get_tools() if t['name'] == 'joint_move')['configSchema']['properties']['controller_ip']['scope'] == 'shared'


def test_stop_cannot_interleave_skin_clear_and_move_sequence():
    _, dashboard, control, _ = ready()
    entered, release = threading.Event(), threading.Event()
    original = dashboard.send_receive_text
    errors = []
    def send(command):
        reply = original(command)
        if command == 'EnableSafeSkin(0)':
            entered.set()
            assert release.wait(2)
        return reply
    dashboard.send_receive_text = send
    def run(action):
        try:
            control.execute_motion(action, {**TARGET, 'confirm_move': True})
        except Exception as error:
            errors.append(error)
    first = threading.Thread(target=run, args=('move_j',))
    second = threading.Thread(target=run, args=('stop_motion',))
    first.start()
    try:
        assert entered.wait(1)
        second.start()
        assert dashboard.commands == ['EnableSafeSkin(0)']
    finally:
        release.set()
        first.join(2)
        if second.ident is not None:
            second.join(2)
    try:
        assert not first.is_alive() and not second.is_alive() and not errors
        assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ, 'Stop()']
    finally:
        control.close()


@pytest.mark.parametrize('action,mode', [('move_j', 4), ('resume', 10)])
def test_disabled_robot_is_enabled_once_before_motion(action, mode):
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, mode)
    feedback.sample['data']['enable_status'] = False
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableRobot()':
            set_mode(feedback, 5 if action == 'move_j' else 10)
        return reply
    dashboard.send_receive_text = send
    try:
        result = control.execute_motion(action, {**TARGET, 'confirm_move': True})
        expected = ['EnableSafeSkin(0)'] + (['ClearError()'] if action == 'move_j' else [])
        assert dashboard.commands == ['EnableRobot()'] + expected + [MOVJ if action == 'move_j' else 'Continue()']
        assert result['enable_response'].startswith('0,')
    finally:
        control.close()


@pytest.mark.parametrize('action,mode', [('move_j', 4), ('resume', 10)])
@pytest.mark.parametrize('failure', ['rejected', 'malformed', 'timeout', 'no_feedback', 'protection'])
def test_enable_failure_or_missing_confirmation_blocks_motion(action, mode, failure):
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, mode)
    feedback.sample['data']['enable_status'] = False
    control.enable_timeout = 0.03
    original = dashboard.send_receive_text
    def send(command):
        if command == 'EnableRobot()':
            dashboard.commands.append(command)
            if failure == 'timeout':
                raise TimeoutError('enable acknowledgement lost')
            if failure == 'rejected':
                return '-1,{},EnableRobot();'
            if failure == 'malformed':
                return 'invalid'
            if failure == 'protection':
                feedback.sample['data']['collision_state'] = 1
            return '0,{},EnableRobot();'
        return original(command)
    dashboard.send_receive_text = send
    with pytest.raises((CommandRejected, ValueError, TimeoutError, RuntimeError)):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert dashboard.commands.count('EnableRobot()') == 1
    assert not any(c.startswith('MovJ(') or c == 'Continue()' for c in dashboard.commands)
    assert dashboard.closed


def test_resume_does_not_continue_if_enabling_loses_paused_state():
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, 4)
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableRobot()':
            set_mode(feedback, 5)
        return reply
    dashboard.send_receive_text = send
    with pytest.raises(RuntimeError, match='mode 10'):
        control.execute_motion('resume', {'confirm_move': True})
    assert dashboard.commands == ['EnableRobot()', 'EnableSafeSkin(0)']


def test_enable_ack_waits_for_later_feedback_before_sending_move():
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, 4)
    original_send = dashboard.send_receive_text
    original_snapshot = feedback.snapshot
    polls = []
    def send(command):
        if command.startswith('MovJ('):
            assert len(polls) >= 3 and feedback.sample['data']['enable_status']
        return original_send(command)
    def snapshot():
        if 'EnableRobot()' in dashboard.commands:
            polls.append(True)
            if len(polls) >= 3:
                set_mode(feedback, 5)
        return original_snapshot()
    dashboard.send_receive_text = send
    feedback.snapshot = snapshot
    try:
        move(control)
        assert dashboard.commands.count('EnableRobot()') == 1
        assert dashboard.commands[-1] == MOVJ
    finally:
        control.close()


@pytest.mark.parametrize('action,mode', [('move_j', 5), ('resume', 10)])
def test_enable_lost_after_disabling_skin_blocks_final_motion(action, mode):
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, mode)
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableSafeSkin(0)':
            feedback.sample['data']['enable_status'] = False
        return reply
    dashboard.send_receive_text = send
    with pytest.raises(RuntimeError, match='enabled robot'):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert not any(c.startswith('MovJ(') or c == 'Continue()' for c in dashboard.commands)


@pytest.mark.parametrize('mode', [9, 11])
def test_alarm_and_collision_can_reach_clear_before_final_motion_check(mode):
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, mode)
    feedback.sample['data'].update(enable_status=True, collision_state=int(mode == 11))
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'ClearError()':
            set_mode(feedback, 5)
            feedback.sample['data']['collision_state'] = 0
        return reply
    dashboard.send_receive_text = send
    try:
        move(control)
        assert dashboard.commands == ['EnableSafeSkin(0)', 'ClearError()', MOVJ]
    finally:
        control.close()


@pytest.mark.parametrize('action,mode', [('move_j', 5), ('resume', 10)])
@pytest.mark.parametrize('flag', ['error_status', 'collision_state', 'arm_approach_state',
                                'j4_approach_state', 'j5_approach_state', 'j6_approach_state'])
def test_persistent_protection_blocks_only_final_motion_after_preparation(action, mode, flag):
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, mode)
    feedback.sample['data'][flag] = 1
    with pytest.raises(RuntimeError, match='blocked by alarm'):
        control.execute_motion(action, {**TARGET, 'confirm_move': True})
    assert dashboard.commands == ['EnableSafeSkin(0)'] + (['ClearError()'] if action == 'move_j' else [])
    assert dashboard.closed


def test_clear_ack_waits_for_feedback_to_leave_collision_mode():
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, 11)
    feedback.sample['data']['enable_status'] = True
    original = feedback.snapshot
    polls = []
    def snapshot():
        if 'ClearError()' in dashboard.commands:
            polls.append(True)
            if len(polls) >= 3:
                set_mode(feedback, 5)
        return original()
    feedback.snapshot = snapshot
    try:
        move(control)
        assert len(polls) >= 3 and dashboard.commands[-1] == MOVJ
    finally:
        control.close()


def test_disabled_collision_state_prepares_in_enable_skin_clear_move_order():
    feedback, dashboard, control, _ = ready()
    set_mode(feedback, 11)
    feedback.sample['data'].update(enable_status=False, collision_state=1)
    original = dashboard.send_receive_text
    def send(command):
        reply = original(command)
        if command == 'EnableRobot()':
            feedback.sample['data']['enable_status'] = True
        if command == 'ClearError()':
            set_mode(feedback, 5)
            feedback.sample['data']['collision_state'] = 0
        return reply
    dashboard.send_receive_text = send
    try:
        move(control)
        assert dashboard.commands == ['EnableRobot()', 'EnableSafeSkin(0)', 'ClearError()', MOVJ]
    finally:
        control.close()
