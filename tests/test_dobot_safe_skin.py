"""SDK-backed skin controls against fake connections, never a physical robot."""
import copy
import threading
import time

import numpy as np
import pytest

from dobot.cra.control import SafeSkinController, CommandRejected, DashboardConnection, parse_reply
from dobot.cra.device import DobotSensors
from dobot.cra.feedback import decode_packet, endpoint
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
        return f"0,{{{'' if command == 'RequestControl()' else len(self.commands)}}},{command};"


def setup_control(dashboard=None):
    feedback = Feedback()
    dashboard = Dashboard() if dashboard is None else dashboard
    factories = []

    def factory(address, port):
        factories.append((address, port))
        return dashboard

    controller = SafeSkinController(feedback, {}, connection_factory=factory)
    return feedback, dashboard, controller, factories


def test_start_info_stop_and_sensors_do_not_claim_control_or_send_commands():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    for name in ('joints', 'safe_skin_control'):
        assert plugin.dispatch('start', {'_tool_name': name})['state'] in ('running', 'ready')
        assert plugin.dispatch('info', {'_tool_name': name})
        assert plugin.dispatch('stop', {'_tool_name': name})['state'] == 'idle'
    assert calls == [] and dashboard.commands == []
    plugin.stop()
    assert feedback.stops == 1


def test_merged_skin_card_advertises_stream_for_every_lifecycle_action():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, namespace='custom_arm', client=feedback, controller=control)
    output = [{'topic': '/custom_arm/state/safe_skin', 'format': 'data/json'}]
    for action in ('config', 'start', 'info', 'get_status', 'stop'):
        result = plugin.dispatch(action, {'_tool_name': 'safe_skin_control'})
        assert result['topic_out'] == output
        if action == 'get_status':
            assert result['feedback']['regions'] == {'arm': False, 'j4': False, 'j5': False, 'j6': False}
    assert not calls and not dashboard.commands
    assert plugin.dispatch('info', {'_tool_name': 'safe_skin'}) is None
    tool = next(tool for tool in plugin.get_tools() if tool['name'] == 'safe_skin_control')
    assert tool['topic_out'] == output
    assert {'start', 'stop', 'config', 'info', 'set_enabled', 'set_sensitivity'} <= set(tool['inputSchema']['properties']['action']['enum'])


def test_joints_config_closes_dashboard_and_clears_old_state_without_sending_commands():
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    control.execute('set_sensitivity', dict(part=4, level=2, confirm_change=True))
    assert control.snapshot()['last_request'] is not None
    result = plugin.dispatch('config', {
        '_tool_name': 'joints', 'controller_ip': '127.0.0.2', 'feedback_port': 30006,
    })
    assert result['state'] == 'configured'
    assert feedback.address == '127.0.0.2' and feedback.port == 30006
    assert dashboard.closed
    assert dashboard.commands == ['SetSafeSkin(4,2)'] and len(calls) == 1
    assert control.snapshot()['last_request'] is None
    assert not control.snapshot()['dashboard_connected']
    assert plugin.payload('joints')['joints'] == []
    assert plugin.payload('joint_angles')['angles_deg'] == []
    assert not plugin.payload('safe_skin_control')['available']


@pytest.mark.parametrize('changed', [{'controller_ip': 'invalid'}, {'feedback_port': 29999}])
def test_invalid_joints_config_preserves_current_endpoint_and_dashboard(changed):
    feedback, dashboard, control, calls = setup_control()
    plugin = DobotSensors({}, client=feedback, controller=control)
    control.execute('set_sensitivity', dict(part=4, level=2, confirm_change=True))
    previous = control.snapshot()
    with pytest.raises(ValueError):
        plugin.dispatch('config', {'_tool_name': 'joints', **changed})
    assert (feedback.address, feedback.port) == ('127.0.0.1', 30004)
    assert not dashboard.closed and control.snapshot() == previous
    assert len(calls) == 1 and dashboard.commands == ['SetSafeSkin(4,2)']
    control.close()


@pytest.mark.parametrize('enabled', [True, False])
def test_sdk_global_switch_submission_and_unknown_readback(enabled):
    feedback, dashboard, control, calls = setup_control()
    result = control.execute('set_enabled', dict(enabled=enabled, confirm_change=True))
    assert dashboard.commands == [f'EnableSafeSkin({int(enabled)})']
    assert calls == [('127.0.0.1', 29999)]
    assert result['result_id'] == 1 and result['state'] == 'submitted'
    assert result['configuration_verified'] is False
    sensor = DobotSensors({}, client=feedback, controller=control).payload('safe_skin_control')
    assert 'enabled' not in sensor and 'sensitivity' not in sensor and 'control' not in sensor
    info = DobotSensors({}, client=feedback, controller=control).dispatch('get_status', {'_tool_name': 'safe_skin_control'})
    assert info['last_request']['enabled'] is enabled
    assert info['last_request']['configuration_verified'] is False
    control.close()


@pytest.mark.parametrize('part', [3, 4, 5, 6])
@pytest.mark.parametrize('level', [0, 1, 2, 3])
def test_sdk_each_region_and_sensitivity(part, level):
    _, dashboard, control, _ = setup_control()
    result = control.execute('set_sensitivity', dict(part=part, level=level, confirm_change=True))
    assert dashboard.commands == [f'SetSafeSkin({part},{level})']
    assert result['state'] == 'submitted'
    control.close()


def test_request_control_is_explicit_and_never_enables_robot():
    _, dashboard, control, _ = setup_control()
    assert control.execute('request_control', {'confirm_change': True})['state'] == 'tcp_control_requested'
    assert dashboard.commands == ['RequestControl()']
    control.execute('set_enabled', dict(enabled=True, confirm_change=True))
    assert dashboard.commands == ['RequestControl()', 'EnableSafeSkin(1)']
    assert control.snapshot()['tcp_control_requested'] is True
    control.close()


@pytest.mark.parametrize('action,args', [
    ('set_enabled', {'enabled': True}),
    ('set_enabled', {'enabled': 1, 'confirm_change': True}),
    ('set_enabled', {'enabled': 'false', 'confirm_change': True}),
    ('set_sensitivity', {'part': True, 'level': 1, 'confirm_change': True}),
    ('set_sensitivity', {'part': 2, 'level': 1, 'confirm_change': True}),
    ('set_sensitivity', {'part': 4, 'level': 4, 'confirm_change': True}),
    ('set_sensitivity', {'part': 4, 'level': False, 'confirm_change': True}),
    ('reset', {'confirm_change': True}),
    ('EnableRobot', {'confirm_change': True}),
])
def test_invalid_or_unsupported_actions_never_open_connection(action, args):
    _, dashboard, control, calls = setup_control()
    with pytest.raises(ValueError):
        control.execute(action, args)
    assert calls == [] and dashboard.commands == []


@pytest.mark.parametrize('key,value', [
    ('robot_mode', 7), ('robot_mode', 9), ('robot_mode', 11), ('running_status', 1),
    ('error_status', True), ('collision_state', 1), ('drag_status', 1), ('pause_cmd_flag', 1),
    ('brake_released', [False, False, True, False, False, False]),
    ('arm_approach_state', 1), ('j4_approach_state', 1), ('j5_approach_state', 1), ('j6_approach_state', 1),
])
def test_settings_blocked_while_moving_triggered_or_unsafe(key, value):
    feedback, dashboard, control, calls = setup_control()
    feedback.sample['data'][key] = value
    with pytest.raises(RuntimeError):
        control.execute('set_enabled', {'enabled': False, 'confirm_change': True})
    assert calls == [] and dashboard.commands == []


@pytest.mark.parametrize('change', ['stale', 'disconnected', 'no_data'])
def test_settings_require_fresh_feedback(change):
    feedback, dashboard, control, calls = setup_control()
    if change == 'stale':
        feedback.sample['stale'] = True
    elif change == 'disconnected':
        feedback.sample['connected'] = False
    else:
        feedback.sample['data'] = None
    with pytest.raises(RuntimeError, match='fresh feedback'):
        control.execute('set_enabled', {'enabled': True, 'confirm_change': True})
    assert calls == []


@pytest.mark.parametrize('key,value', [('robot_mode', 5), ('auto_manual_mode', 1)])
def test_takeover_has_its_own_disabled_and_mode_requirements(key, value):
    feedback, _, control, calls = setup_control()
    feedback.sample['data'][key] = value
    with pytest.raises(RuntimeError, match='TCP takeover'):
        control.execute('request_control', {'confirm_change': True})
    assert calls == []


def test_state_is_rechecked_after_slow_connect():
    feedback, dashboard, control, _ = setup_control()
    dashboard.on_connect = lambda: feedback.sample['data'].update(running_status=1)
    with pytest.raises(RuntimeError):
        control.execute('set_enabled', dict(enabled=True, confirm_change=True))
    assert dashboard.commands == [] and dashboard.closed


@pytest.mark.parametrize('code', [-1, -2, -3, -9])
def test_rejected_setting_does_not_update_as_success_or_retry(code):
    _, dashboard, control, _ = setup_control(Dashboard(reply=f'{code},{{}},EnableSafeSkin(1);'))
    with pytest.raises(CommandRejected):
        control.execute('set_enabled', dict(enabled=True, confirm_change=True))
    assert len(dashboard.commands) == 1
    assert control.snapshot()['last_request']['state'] == 'rejected'
    assert dashboard.closed


def test_timeout_is_not_replayed_and_outcome_is_explicitly_unknown():
    _, dashboard, control, _ = setup_control(Dashboard(failure=TimeoutError('lost reply')))
    with pytest.raises(TimeoutError):
        control.execute('set_enabled', dict(enabled=True, confirm_change=True))
    control.close()
    assert dashboard.commands == ['EnableSafeSkin(1)']
    assert control.snapshot()['last_request']['state'] == 'failed_or_outcome_unknown'


def test_old_ack_without_queue_id_is_not_claimed_executed():
    _, _, control, _ = setup_control(Dashboard(reply='0,{},EnableSafeSkin(1);'))
    result = control.execute('set_enabled', dict(enabled=True, confirm_change=True))
    assert result['state'] == 'accepted_unverified' and result['result_id'] is None
    control.close()


@pytest.mark.parametrize('reply', ['Control Mode Is Not Tcp', '0,{1},Other(1);', '0,{oops},EnableSafeSkin(1);', '0,{1},EnableSafeSkin(1);junk'])
def test_malformed_acknowledgements_are_rejected(reply):
    with pytest.raises(ValueError):
        parse_reply(reply, 'EnableSafeSkin')


def test_skin_sensor_regions_do_not_misreport_collision_as_proximity():
    feedback, _, control, _ = setup_control()
    feedback.sample['data'].update(j4_approach_state=1, j6_approach_state=1, collision_state=0)
    plugin = DobotSensors({}, client=feedback, controller=control)
    result = plugin.payload('safe_skin_control')
    assert set(result) == {'timestamp_ms', 'available', 'any_approach_paused', 'regions'}
    assert isinstance(result['timestamp_ms'], int) and result['available'] is True
    assert result['any_approach_paused']
    assert result['regions'] == {'arm': False, 'j4': True, 'j5': False, 'j6': True}
    feedback.sample['data'].update(j4_approach_state=0, j6_approach_state=0, collision_state=1)
    assert plugin.payload('safe_skin_control')['any_approach_paused'] is False
    feedback.sample.update(connected=False, stale=True, data=None)
    result = plugin.payload('safe_skin_control')
    assert result['any_approach_paused'] is None
    assert result['available'] is False
    assert result['regions'] == dict.fromkeys(('arm', 'j4', 'j5', 'j6'))


def test_control_requests_are_serialized_but_sensor_snapshot_does_not_block():
    feedback, dashboard, control, _ = setup_control()
    entered = threading.Event()
    release = threading.Event()
    original = dashboard.send_receive_text
    active = peak = 0
    errors = []

    def blocked(command):
        nonlocal active, peak
        active += 1
        peak = max(peak, active)
        entered.set()
        assert release.wait(2)
        try:
            return original(command)
        finally:
            active -= 1

    dashboard.send_receive_text = blocked

    def run(part):
        try:
            control.execute('set_sensitivity', dict(part=part, level=2, confirm_change=True))
        except Exception as exc:
            errors.append(exc)

    workers = [threading.Thread(target=run, args=(part,)) for part in (3, 4)]
    workers[0].start()
    assert entered.wait(1)
    workers[1].start()
    snapshot_done = threading.Event()
    reader = threading.Thread(target=lambda: (DobotSensors({}, client=feedback, controller=control).payload('safe_skin_control'), snapshot_done.set()))
    reader.start()
    try:
        assert snapshot_done.wait(0.5)
    finally:
        release.set()
        for worker in workers:
            worker.join(2)
        reader.join(2)
        control.close()
    assert not errors and peak == 1
    assert dashboard.commands == ['SetSafeSkin(3,2)', 'SetSafeSkin(4,2)']


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
        writer.sendall(b'0,{12},EnableSafe')
        time.sleep(0.02)
        writer.sendall(b'Skin(1);')

    thread = threading.Thread(target=server)
    thread.start()
    try:
        result = connection.send_receive_text('EnableSafeSkin(1)')
        assert parse_reply(result, 'EnableSafeSkin') == 12
        assert received == [b'EnableSafeSkin(1)\n']
    finally:
        connection.disconnect()
        thread.join(1)
        writer.close()
