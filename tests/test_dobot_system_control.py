"""System control contracts using the pinned SDK and simulated transports only."""

import json
import threading

import pytest

from common.vendor_runtime import DriverBundle
from dobot.cra.device import DobotSensors
from dobot.cra.feedback import ROBOT_MODES
from test_dobot_safe_skin import Dashboard, setup_control


def set_mode(feedback, mode):
    feedback.sample['data'].update(
        robot_mode=mode, robot_mode_name=ROBOT_MODES[mode],
        enable_status=mode in (5, 6, 7, 8, 10), running_status=int(mode in (7, 8)),
        drag_status=int(mode == 6), pause_cmd_flag=int(mode == 10), error_status=mode == 9,
    )


WRITES = [
    ('request_control', 4, {}, 'RequestControl()'),
    ('power_on', 3, {}, 'PowerOn()'),
    ('enable', 4, {}, 'EnableRobot()'),
    ('disable', 7, {}, 'DisableRobot()'),
    ('clear_error', 9, {}, 'ClearError()'),
    ('stop_motion', 8, {}, 'Stop()'),
    ('pause', 7, {}, 'Pause()'),
    ('resume', 10, {}, 'Continue()'),
    ('set_speed_factor', 5, {'ratio': 20}, 'SpeedFactor(20)'),
    ('start_drag', 5, {}, 'StartDrag()'),
    ('stop_drag', 6, {}, 'StopDrag()'),
]


@pytest.mark.parametrize('action,mode,args,command', WRITES)
def test_system_actions_use_exact_sdk_commands_without_chained_effects(action, mode, args, command):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    bundle = DriverBundle([DobotSensors({}, client=feedback, controller=control)])
    try:
        result = bundle.dispatch('systemcontrol', {'action': action, 'confirm_change': True, **args})
        assert dashboard.commands == [command]
        assert calls == [('127.0.0.1', 29999)]
        assert result['action'] == action and result['effect_verified'] is False
        assert control.snapshot('systemcontrol')['last_request'] == result
        assert control.snapshot()['last_request'] is None
    finally:
        control.close()


@pytest.mark.parametrize('action,mode,args,command', WRITES)
@pytest.mark.parametrize('confirmation', [None, False, 1, 'true'])
def test_every_system_write_requires_explicit_boolean_confirmation(action, mode, args, command, confirmation):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    with pytest.raises(ValueError, match='confirm_change'):
        control.execute_system(action, {**args, 'confirm_change': confirmation})
    assert not calls and not dashboard.commands


def test_system_lifecycle_and_info_do_not_send_commands_and_stop_does_not_stop_robot():
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, 7)
    plugin = DobotSensors({}, client=feedback, controller=control)
    for action, expected in [('start', 'ready'), ('config', 'configured'), ('stop', 'idle')]:
        assert plugin.dispatch(action, {'_tool_name': 'systemcontrol'})['state'] == expected
    info = plugin.dispatch('info', {'_tool_name': 'systemcontrol'})
    assert info['available'] and info['robot_status']['robot_mode'] == 7
    assert not calls and not dashboard.commands
    control.execute_system('pause', {'confirm_change': True})
    plugin.dispatch('stop', {'_tool_name': 'systemcontrol'})
    assert dashboard.closed and dashboard.commands == ['Pause()']
    feedback.sample.update(connected=False, stale=True, data=None)
    assert plugin.dispatch('info', {'_tool_name': 'systemcontrol'})['robot_status'] is None


@pytest.mark.parametrize('changed', [{'stale': True}, {'connected': False}, {'data': None}])
@pytest.mark.parametrize('action,mode', [('power_on', 3), ('enable', 4), ('clear_error', 9),
                                      ('request_control', 4), ('pause', 7), ('resume', 10),
                                      ('set_speed_factor', 5), ('start_drag', 5)])
def test_actions_that_change_operating_state_require_fresh_feedback(changed, action, mode):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    feedback.sample.update(changed)
    with pytest.raises(RuntimeError, match='fresh feedback'):
        control.execute_system(action, {'confirm_change': True, 'ratio': 20})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,command', [('stop_motion', 'Stop()'), ('disable', 'DisableRobot()'),
                                         ('stop_drag', 'StopDrag()')])
def test_stopping_actions_remain_available_when_feedback_is_lost(action, command):
    feedback, dashboard, control, calls = setup_control()
    feedback.sample.update(connected=False, stale=True, data=None)
    try:
        control.execute_system(action, {'confirm_change': True})
        assert dashboard.commands == [command] and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('action', ['stop_motion', 'disable', 'stop_drag', 'get_errors'])
def test_no_command_endpoint_means_no_dashboard_connection(action):
    feedback, dashboard, control, calls = setup_control()
    feedback.address = ''
    with pytest.raises(RuntimeError, match='controller IP'):
        control.execute_system(action, {'confirm_change': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('errors', ['[]', '[134]', '[134,1538]'])
def test_error_query_parses_documented_flat_array_even_without_feedback(errors):
    feedback, dashboard, control, calls = setup_control(Dashboard(reply=f'0,{{{errors}}},GetErrorID();'))
    feedback.sample.update(connected=False, stale=True, data=None)
    try:
        result = control.execute_system('get_errors', {})
        assert result['state'] == 'queried' and result['read_only'] is True
        assert result['error_ids'] == json.loads(errors)
        assert dashboard.commands == ['GetErrorID()'] and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('payload', ['1', 'null', '[true]', '["134"]', '[[134]]', '[134,]'])
def test_malformed_alarm_arrays_fail_without_replay(payload):
    _, dashboard, control, calls = setup_control(Dashboard(reply=f'0,{{{payload}}},GetErrorID();'))
    with pytest.raises(ValueError):
        control.execute_system('get_errors', {})
    assert dashboard.closed and dashboard.commands == ['GetErrorID()'] and len(calls) == 1


@pytest.mark.parametrize('ratio', [None, True, 0, 101, 1.5, '20'])
def test_speed_ratio_is_validated_before_connecting(ratio):
    _, dashboard, control, calls = setup_control()
    with pytest.raises(ValueError, match='ratio'):
        control.execute_system('set_speed_factor', {'ratio': ratio, 'confirm_change': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,mode', [('power_on', 4), ('enable', 3), ('enable', 9), ('enable', 10),
                                      ('resume', 5), ('pause', 8), ('start_drag', 7), ('clear_error', 7)])
def test_wrong_operating_modes_are_rejected_without_automatic_recovery(action, mode):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    with pytest.raises(RuntimeError):
        control.execute_system(action, {'confirm_change': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('change', [{'error_status': True}, {'collision_state': 1}, {'arm_approach_state': 1},
                                  {'brake_released': [True] * 6}, {'drag_status': 1}, {'enable_status': False}])
def test_continue_is_blocked_by_active_protection_or_disabled_state(change):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, 10)
    feedback.sample['data'].update(change)
    with pytest.raises(RuntimeError):
        control.execute_system('resume', {'confirm_change': True})
    assert not calls and not dashboard.commands


def test_system_state_is_rechecked_after_connecting():
    feedback, dashboard, control, calls = setup_control()
    dashboard.on_connect = lambda: feedback.sample['data'].update(collision_state=1)
    with pytest.raises(RuntimeError):
        control.execute_system('enable', {'confirm_change': True})
    assert not dashboard.commands and dashboard.closed and len(calls) == 1


@pytest.mark.parametrize('failure', [TimeoutError('lost response'), ConnectionError('network lost')])
def test_uncertain_system_writes_are_not_replayed(failure):
    _, dashboard, control, calls = setup_control(Dashboard(failure=failure))
    with pytest.raises(type(failure)):
        control.execute_system('enable', {'confirm_change': True})
    status = control.snapshot('systemcontrol')
    assert status['last_request']['state'] == 'failed_or_outcome_unknown'
    assert status['last_request']['effect_verified'] is False
    assert len(calls) == 1 and dashboard.commands == ['EnableRobot()'] and dashboard.closed


def test_both_cards_share_one_serial_transport_and_keep_separate_request_history():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    entered, release = threading.Event(), threading.Event()
    original = dashboard.send_receive_text
    failures = []

    def blocked(command):
        if command.startswith('SetSafeSkin'):
            entered.set()
            assert release.wait(2)
        return original(command)

    dashboard.send_receive_text = blocked

    def execute(name, action, args):
        try:
            plugin.dispatch(action, {'_tool_name': name, 'confirm_change': True, **args})
        except Exception as exc:
            failures.append(exc)

    first = threading.Thread(target=execute, args=('safe_skin_control', 'set_sensitivity', {'part': 4, 'level': 2}))
    second = threading.Thread(target=execute, args=('systemcontrol', 'set_speed_factor', {'ratio': 20}))
    first.start()
    assert entered.wait(1)
    second.start()
    try:
        assert plugin.dispatch('info', {'_tool_name': 'systemcontrol'})['available']
        assert plugin.payload('safe_skin')['available']
    finally:
        release.set()
        first.join(2)
        second.join(2)
        control.close()
    assert not first.is_alive() and not second.is_alive() and not failures
    assert dashboard.commands == ['SetSafeSkin(4,2)', 'SpeedFactor(20)']
    assert len(calls) == 1
    assert control.snapshot()['last_request']['action'] == 'set_sensitivity'
    assert control.snapshot('systemcontrol')['last_request']['action'] == 'set_speed_factor'
    plugin.dispatch('config', {'_tool_name': 'joints', 'controller_ip': '127.0.0.2'})
    assert control.snapshot()['last_request'] is None
    assert control.snapshot('systemcontrol')['last_request'] is None


def test_system_card_metadata_covers_actions_and_keeps_motion_stop_distinct():
    tool = next(tool for tool in DobotSensors({}).get_tools() if tool['name'] == 'systemcontrol')
    assert tool['type'] == 'actuator' and tool['inputSchema']['x-is-dangerous'] is True
    assert tool['inputSchema']['x-resource'] == 'arm'
    assert set(tool['inputSchema']['properties']['action']['enum']) == {
        'info', 'get_errors', 'request_control', 'power_on', 'enable', 'disable', 'clear_error',
        'stop_motion', 'pause', 'resume', 'set_speed_factor', 'start_drag', 'stop_drag', 'start', 'stop', 'config',
    }
