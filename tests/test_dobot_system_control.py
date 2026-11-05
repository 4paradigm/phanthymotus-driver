"""System control contracts using the pinned SDK and simulated transports only."""

import copy
import time
import threading

import numpy as np
import pytest

from common.vendor_runtime import DriverBundle
from dobot.cra.device import DobotSensors
from dobot.cra.feedback import ROBOT_MODES, decode_packet, endpoint
from dobot.cra.control import RobotController, CommandRejected, DashboardConnection, parse_reply
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


WRITES = [
    ('enable', 4, {}, 'EnableRobot()'),
    ('disable', 7, {}, 'DisableRobot()'),
    ('clear_error', 9, {}, 'ClearError()'),
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
        assert control.snapshot()['last_request'] == result
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
    info = plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})
    assert info['available'] and info['robot_status']['robot_mode'] == 7
    assert not calls and not dashboard.commands
    control.execute_system('disable', {'confirm_change': True})
    plugin.dispatch('stop', {'_tool_name': 'systemcontrol'})
    assert dashboard.closed and dashboard.commands == ['DisableRobot()']
    feedback.sample.update(connected=False, stale=True, data=None)
    assert plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})['robot_status'] is None


@pytest.mark.parametrize('changed', [{'stale': True}, {'connected': False}, {'data': None}])
@pytest.mark.parametrize('action,mode', [('enable', 4), ('clear_error', 9), ('start_drag', 5)])
def test_actions_that_change_operating_state_require_fresh_feedback(changed, action, mode):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    feedback.sample.update(changed)
    with pytest.raises(RuntimeError, match='fresh feedback'):
        control.execute_system(action, {'confirm_change': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,command', [('disable', 'DisableRobot()'), ('stop_drag', 'StopDrag()')])
def test_stopping_actions_remain_available_when_feedback_is_lost(action, command):
    feedback, dashboard, control, calls = setup_control()
    feedback.sample.update(connected=False, stale=True, data=None)
    try:
        control.execute_system(action, {'confirm_change': True})
        assert dashboard.commands == [command] and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('action', ['disable', 'stop_drag'])
def test_no_command_endpoint_means_no_dashboard_connection(action):
    feedback, dashboard, control, calls = setup_control()
    feedback.address = ''
    with pytest.raises(RuntimeError, match='controller IP'):
        control.execute_system(action, {'confirm_change': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,mode', [('enable', 3), ('enable', 9), ('enable', 10),
                                      ('start_drag', 7), ('clear_error', 7)])
def test_wrong_operating_modes_are_rejected_without_automatic_recovery(action, mode):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    with pytest.raises(RuntimeError):
        control.execute_system(action, {'confirm_change': True})
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
    status = control.snapshot()
    assert status['last_request']['state'] == 'failed_or_outcome_unknown'
    assert status['last_request']['effect_verified'] is False
    assert len(calls) == 1 and dashboard.commands == ['EnableRobot()'] and dashboard.closed


@pytest.mark.parametrize('code', [-1, -2, -3, -9])
def test_rejected_system_command_records_failure_without_replaying(code):
    _, dashboard, control, calls = setup_control(Dashboard(reply=f'{code},{{}},EnableRobot();'))
    with pytest.raises(CommandRejected):
        control.execute_system('enable', {'confirm_change': True})
    assert dashboard.commands == ['EnableRobot()'] and len(calls) == 1
    assert control.snapshot()['last_request']['state'] == 'rejected'
    assert control.snapshot()['last_error'] and dashboard.closed


def test_empty_success_acknowledgement_does_not_claim_actual_enable():
    feedback, dashboard, control, calls = setup_control(Dashboard(reply='0,{},EnableRobot();'))
    try:
        result = control.execute_system('enable', {'confirm_change': True})
        assert result['state'] == 'accepted_unverified' and result['result_id'] is None
        assert result['effect_verified'] is False
        assert control.system_status()['enabled'] is False
        assert dashboard.commands == ['EnableRobot()'] and len(calls) == 1
    finally:
        control.close()


@pytest.mark.parametrize('reply', ['Control Mode Is Not Tcp', '0,{1},Other();',
                                 '0,{oops},EnableRobot();', '0,{1},EnableRobot();junk'])
def test_malformed_command_acknowledgements_are_rejected(reply):
    with pytest.raises(ValueError):
        parse_reply(reply, 'EnableRobot')


def test_system_card_exposes_only_status_enable_clear_error_and_drag():
    tool = next(tool for tool in DobotSensors({}).get_tools() if tool['name'] == 'systemcontrol')
    assert tool['type'] == 'actuator' and tool['inputSchema']['x-is-dangerous'] is True
    assert tool['inputSchema']['x-resource'] == 'arm'
    assert 'ratio' not in tool['inputSchema']['properties']
    assert set(tool['inputSchema']['properties']['action']['enum']) == {
        'info', 'get_status', 'enable', 'disable', 'clear_error', 'start_drag', 'stop_drag', 'start', 'stop', 'config',
    }


@pytest.mark.parametrize('action', ['get_errors', 'request_control', 'power_on', 'stop_motion',
                                  'pause', 'resume', 'set_speed_factor', 'set_sensitivity'])
def test_removed_system_actions_cannot_send_commands_even_when_confirmed(action):
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    with pytest.raises(ValueError, match='unsupported system control action'):
        plugin.dispatch(action, {'_tool_name': 'systemcontrol', 'confirm_change': True, 'ratio': 20})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('action,mode', [('enable', 4), ('start_drag', 5)])
@pytest.mark.parametrize('change', [{'error_status': True}, {'collision_state': 1}, {'arm_approach_state': 1},
                                  {'j4_approach_state': 1}, {'j5_approach_state': 1}, {'j6_approach_state': 1},
                                  {'brake_released': [True] * 6}, {'drag_status': 1},
                                  {'running_status': 1}, {'jog_status': 1}, {'pause_cmd_flag': 1}])
def test_enable_and_drag_are_blocked_by_active_protection_or_movement(action, mode, change):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    feedback.sample['data'].update(change)
    with pytest.raises(RuntimeError):
        control.execute_system(action, {'confirm_change': True})
    assert not calls and not dashboard.commands


def test_system_writes_are_serialized_while_status_and_joints_remain_readable():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    entered, release = threading.Event(), threading.Event()
    original = dashboard.send_receive_text
    failures = []

    def blocked(command):
        if command == 'DisableRobot()':
            entered.set()
            assert release.wait(2)
        return original(command)

    dashboard.send_receive_text = blocked

    def execute(action):
        try:
            plugin.dispatch(action, {'_tool_name': 'systemcontrol', 'confirm_change': True})
        except Exception as exc:
            failures.append(exc)

    first = threading.Thread(target=execute, args=('disable',))
    second = threading.Thread(target=execute, args=('stop_drag',))
    first.start()
    assert entered.wait(1)
    second.start()
    try:
        assert plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})['available']
        assert plugin.payload('joints')['available']
        assert not dashboard.commands
    finally:
        release.set()
        first.join(2)
        second.join(2)
    try:
        assert not first.is_alive() and not second.is_alive() and not failures
        assert dashboard.commands == ['DisableRobot()', 'StopDrag()'] and len(calls) == 1
        plugin.dispatch('config', {'_tool_name': 'joints', 'controller_ip': '127.0.0.2'})
        assert dashboard.closed and control.snapshot()['last_request'] is None
        assert plugin.payload('joint_angles')['joint1_deg'] is None
    finally:
        control.close()


@pytest.mark.parametrize('name', ['joints', 'systemcontrol'])
def test_info_is_static_metadata_and_status_is_a_visible_action(name, monkeypatch):
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)

    def forbidden(*args, **kwargs):
        raise AssertionError('info must not read live feedback or request history')

    monkeypatch.setattr(feedback, 'snapshot', forbidden)
    monkeypatch.setattr(control, 'snapshot', forbidden)
    info = plugin.dispatch('info', {'_tool_name': name})
    assert info['name'] == name and info['description']
    assert not {'robot_status', 'enabled', 'feedback', 'last_request', 'available'} & info.keys()
    if name == 'joints':
        assert 'action' not in info['inputSchema']['properties']
    else:
        actions = info['inputSchema']['properties']['action']['enum']
        # Core hides these reserved lifecycle names from the action selector.
        visible = set(actions) - {'start', 'stop', 'info', 'config'}
        assert 'get_status' in visible
        assert info['inputSchema']['x-action-params']['get_status']['params'] == []
    if name != 'systemcontrol':
        assert info['topic_out']
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('mode,expected', [
    (3, {'powered_on': False, 'enabled': False}),
    (4, {'powered_on': True, 'enabled': False}),
    (5, {'powered_on': True, 'enabled': True, 'running': False}),
    (6, {'dragging': True}),
    (7, {'enabled': True, 'running': True, 'paused': False}),
    (8, {'running': True}),
    (9, {'powered_on': None, 'has_error': True}),
    (10, {'enabled': True, 'paused': True}),
    (11, {'collision': True}),
])
def test_get_status_reports_current_effects_from_feedback(mode, expected):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, mode)
    feedback.sample['data']['speed_scaling'] = 0.25
    plugin = DobotSensors({}, client=feedback, controller=control)
    result = plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})
    assert result['available'] and result['read_only']
    for key, value in expected.items():
        assert result[key] is value
    assert result['speed_scaling'] == 0.25
    assert result['robot_status']['robot_mode'] == mode
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('changed', [{'stale': True}, {'connected': False}, {'data': None}])
def test_get_status_withholds_all_live_effects_when_feedback_is_unavailable(changed):
    feedback, dashboard, control, calls = setup_control()
    set_mode(feedback, 5)
    feedback.sample.update(changed)
    result = DobotSensors({}, client=feedback, controller=control).dispatch('get_status', {'_tool_name': 'systemcontrol'})
    assert result['available'] is False
    for key in ('powered_on', 'enabled', 'running', 'paused', 'dragging', 'collision', 'has_error', 'speed_scaling', 'robot_status'):
        assert result[key] is None
    assert not calls and not dashboard.commands


def test_enable_acknowledgement_does_not_replace_actual_enable_feedback():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    set_mode(feedback, 4)
    try:
        plugin.dispatch('enable', {'_tool_name': 'systemcontrol', 'confirm_change': True})
        assert plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})['enabled'] is False
        set_mode(feedback, 5)
        assert plugin.dispatch('get_status', {'_tool_name': 'systemcontrol'})['enabled'] is True
        assert dashboard.commands == ['EnableRobot()'] and len(calls) == 1
    finally:
        control.close()


def test_transport_reassembles_fragmented_acknowledgement():
    import socket
    reader, writer = socket.socketpair()
    connection = DashboardConnection('127.0.0.1', 29999)
    connection._socket = reader
    connection._connected = True
    reader.settimeout(1)
    received = []

    def server():
        received.append(writer.recv(4096))
        writer.sendall(b'0,{12},EnableRo')
        time.sleep(0.02)
        writer.sendall(b'bot();')

    thread = threading.Thread(target=server)
    thread.start()
    try:
        result = connection.send_receive_text('EnableRobot()')
        assert parse_reply(result, 'EnableRobot') == 12
        assert received == [b'EnableRobot()\n']
    finally:
        connection.disconnect()
        thread.join(1)
        writer.close()
