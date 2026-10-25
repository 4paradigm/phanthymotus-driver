"""Position-only grasp regressions with simulated hardware and real CAN codec."""
import threading
import time

import pytest
from test_linkerhand_l20 import (
    GraspPlugin, GesturePlugin, HandPlugin, L20CanClient, MockCanBus,
    MockCanMessage, _SingleHandController,
)
from can_client import vector20_to_named16
from grasp_device import DEFAULT_GRASP_PREP_POSE, DEFAULT_TOP_GRASP_PREP_POSE

class ScalarBus(MockCanBus):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.scalar_online = True
        self.missing_scalar = None
        self.matrix_only = False
        self.sim_touch = [35] * 5

    def send(self, msg):
        if 0xB1 <= msg.data[0] <= 0xB5 and len(msg.data) == 1:
            if not self.scalar_online or msg.data[0] == self.missing_scalar:
                self.sent_messages.append(msg)
                return
            if self.matrix_only:
                self.sent_messages.append(msg)
                self._inbox.put(MockCanMessage(self.can_id, [msg.data[0], 0, 1, 2, 3, 4, 5, 6]))
                return
        super().send(msg)


@pytest.fixture
def connected():
    bus = ScalarBus(can_id=0x28)
    client = L20CanClient(side='left', can_id=0x28, recv_timeout_s=0.05, bus_factory=lambda *_: bus)
    ctrl = _SingleHandController({}, 'left', 'can0', threading.RLock(), can_id=0x28)
    ctrl._client = client
    hand = HandPlugin({'left_can_id': 0x28, 'right_can_id': 0x27}, {})
    hand.get_controller_for_side = lambda *_: ctrl
    p = GraspPlugin(hand, {'poll_interval_s': 0.005, 'pose_timeout_s': 0.1})
    events = []
    done = threading.Event()
    def completed(aid, status, result):
        events.append((aid, status, result))
        done.set()
    p._acp_callback = completed
    p._test_completions = events
    # Ignore client protocol setup frames; each case observes only its action.
    bus.sent_messages.clear()
    try:
        yield p, client, bus, ctrl, hand, events, done
    finally:
        active = list(p._active.values())
        p.stop()
        for task in active:
            assert task['done_event'].wait(3), 'grasp worker did not stop'
        client.close()


def request(p, action='side_grasp', **kwargs):
    reply = p.dispatch(action, {'instance_id': 'grasp-card', 'bound_side': 'left', 'confirm_motion': True, **kwargs})
    if kwargs.get('wait_for_result') is False or reply is None:
        return reply
    # Mechanical regressions inspect the final ACP payload. Waiting belongs to
    # this test client, never to the driver's MCP request handler.
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        for aid, _, result in p._test_completions:
            if aid == reply['action_id']:
                return result
        time.sleep(0.001)
    pytest.fail('No ACP completion received for ' + reply['action_id'])


def writes(bus):
    return [m for m in bus.sent_messages if 0x41 <= m.data[0] <= 0x45 and len(m.data) > 1]


def no_touch(*args, **kwargs):
    raise AssertionError('position-only grasp must never read tactile data')


@pytest.mark.parametrize('action,mode', [('side_grasp', 'side'), ('top_grasp', 'top')])
@pytest.mark.parametrize('ratio,speed', [('1.0x', 85), ('1.5x', 128), ('2.0x', 170), ('2.5x', 212), ('3.0x', 255)])
def test_position_targets_speed_and_persistent_motor_holding(connected, monkeypatch, action, mode, ratio, speed):
    p, client, bus, ctrl, _, events, _ = connected
    # Missing/unverified tactile feedback must not block even preparation/release.
    for name in ('get_fresh_tactile', 'get_tactile_snapshot', 'get_fresh_matrix_touch'):
        monkeypatch.setattr(client, name, no_touch)
    bus.scalar_online = False
    positions = []
    send = client.set_joint_positions
    def capture(target):
        positions.append(list(target))
        send(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    result = request(p, action, speed_ratio=ratio)
    prep = DEFAULT_GRASP_PREP_POSE if mode == 'side' else DEFAULT_TOP_GRASP_PREP_POSE
    assert positions[-2] == prep
    assert positions[-1][0:5] == [0] * 5
    assert positions[-1][16:20] == [0] * 4
    assert positions[-1][15] == (255 if mode == 'top' else 0)
    assert positions[-1][5:11] == prep[5:11]
    assert vector20_to_named16(positions[-1]) == result['target_positions']
    assert result['success'] and result['command_sent'] and result['holding']
    assert result['approach_speed'] == speed and result['speed_ratio'] == ratio
    assert '指令已下发' in result['message']
    assert not {'grasp_confirmed', 'target_force', 'contact_forces', 'action_id'}.intersection(result)
    assert not p._active and p._sessions[('left', 'can0')]['grasp_mode'] == mode
    assert vars(ctrl)['_grasp_owner'] == (id(p), ('left', 'can0'))
    assert client._currents == [p.motion_current] * 5
    assert client._speeds == [speed] * 5
    assert p.dispatch('info', {'bound_side': 'left'})['state'] == 'holding'
    assert len(events) == 1 and events[0][1] == 'completed'
    assert events[0][2] == result
    count = len(bus.sent_messages)
    time.sleep(0.025)
    assert len(bus.sent_messages) == count, 'no hidden polling/hold worker'
    positions.clear()
    released = request(p, 'release')
    assert positions[0] == prep
    assert vector20_to_named16(positions[-1]) == vector20_to_named16(ctrl._open_pose)
    assert released['hand_opened'] and released['torque_released'] and not released['holding']
    assert client._currents == [0] * 5
    assert not p._sessions and not vars(ctrl).get('_grasp_owner')
    assert not any(0xB1 <= m.data[0] <= 0xB5 for m in bus.sent_messages)


@pytest.mark.parametrize('action', ['side_grasp', 'top_grasp'])
def test_loaded_closing_joints_need_not_reach_zero_or_be_sampled(connected, monkeypatch, action):
    p, client, _, _, *_ = connected
    move = client.set_joint_positions
    read = client.get_positions_vector
    closed = threading.Event()
    def blocked(target):
        if target[0:5] == [0] * 5:
            closed.set()  # Object blocks movement: actual positions remain open.
        else:
            move(target)
    def no_read_after_close():
        assert not closed.is_set(), 'closing must not wait for actual zero position'
        return read()
    monkeypatch.setattr(client, 'set_joint_positions', blocked)
    monkeypatch.setattr(client, 'get_positions_vector', no_read_after_close)
    result = request(p, action)
    assert result['success'] and result['command_sent'] and closed.is_set()
    assert read()[1] == 255


def test_tool_exposes_only_three_actions_and_no_pressure_fields(connected):
    p, *_ = connected
    definition = p.get_tool()
    schema = definition['inputSchema']
    assert set(schema['properties']['action']['enum']) == {'side_grasp', 'top_grasp', 'release'}
    assert set(schema['x-action-params']) == {'side_grasp', 'top_grasp', 'release'}
    assert 'configSchema' not in definition
    assert all('bound_side' in fields['params'] for fields in schema['x-action-params'].values())
    assert schema['properties']['bound_side']['default'] == ''
    assert set(schema['x-completion']['actions']) == {'side_grasp', 'top_grasp', 'release'}
    assert schema['properties']['wait_for_result']['default'] is False
    assert schema['properties']['wait_for_result']['const'] is False
    assert not schema['properties']['instance_id'].get('default')
    assert not {'target_force', 'max_force', 'hold_duration_s', 'active_fingers', 'approach_speed'}.intersection(schema['properties'])


@pytest.mark.parametrize('action', ['grasp', 'hold', 'get_tactile', 'get_matrix_touch'])
def test_removed_actions_never_acquire_or_command_hardware(connected, monkeypatch, action):
    p, _, bus, *_ = connected
    monkeypatch.setattr(p, '_get_client_and_ctrl', no_touch)
    assert request(p, action) is None
    assert not bus.sent_messages


@pytest.mark.parametrize('argument', ['target_force', 'max_force', 'hold_duration_s', 'slip_threshold', 'active_fingers', 'approach_speed'])
def test_removed_arguments_are_rejected_before_hardware(connected, argument):
    p, _, bus, *_ = connected
    with pytest.raises(ValueError, match='已移除参数'):
        request(p, **{argument: 30})
    assert not bus.sent_messages


@pytest.mark.parametrize('ratio', [True, 2, 1.5, None, '0.5x', '4.0x', '1.0'])
def test_invalid_speed_rejected_without_hardware(connected, ratio):
    p, _, bus, *_ = connected
    with pytest.raises(ValueError, match='speed_ratio'):
        request(p, speed_ratio=ratio)
    assert not bus.sent_messages


def test_old_pressure_and_binding_configuration_are_unused(connected):
    p, client, _, ctrl, hand, *_ = connected
    old = GraspPlugin(hand, {'default_target_force': 'obsolete', 'default_max_force': 0,
                           'pressure_fast_step': -5, 'approach_speed': 3, 'auto_hold_s': -1})
    assert old._options('', 'side_grasp', {})['approach_speed'] == 85
    request(p)
    before = list(client._currents)
    p.dispatch('config', {'instance_id': 'grasp-card', 'bound_side': 'left', 'max_force': 200})
    assert client._currents == before and vars(ctrl).get('_grasp_owner')
    p.dispatch('config', {'instance_id': 'grasp-card', 'bound_side': 'right'})
    assert client._currents == before
    assert not hasattr(p, '_bindings')
    assert p.dispatch('start', {})['selection_required']


def test_repeated_grasp_cannot_unexpectedly_open_held_object(connected):
    p, _, bus, *_ = connected
    request(p)
    bus.sent_messages.clear()
    for action in ('side_grasp', 'top_grasp'):
        with pytest.raises(RuntimeError, match='release'):
            request(p, action)
    assert not bus.sent_messages


@pytest.mark.parametrize('lifecycle', ['stop', 'shutdown'])
def test_stop_unloads_holding_without_a_worker(connected, lifecycle):
    p, client, _, ctrl, *_ = connected
    request(p)
    assert not p._active
    if lifecycle == 'stop':
        p.dispatch('stop', {'instance_id': 'grasp-card'})
        assert p.dispatch('start', {'instance_id': 'grasp-card'})['state'] == 'ready'
    else:
        p.stop()
    assert client._currents == [0] * 5
    assert not vars(ctrl).get('_grasp_owner') and not p._sessions


def test_stopping_cached_nonowner_cannot_unpower_another_card(connected):
    p, client, _, ctrl, *_ = connected
    p._get_client_and_ctrl('left')
    ctrl._grasp_owner = (12345, 'other-card')
    client.set_currents([200] * 5)
    p.dispatch('stop', {'instance_id': 'grasp-card'})
    assert client._currents == [200] * 5
    assert vars(ctrl)['_grasp_owner'] == (12345, 'other-card')
    del ctrl._grasp_owner


def test_other_cards_cannot_move_or_unpower_held_grasp(connected):
    p, client, _, ctrl, hand, *_ = connected
    request(p)
    gesture = GesturePlugin(hand, {})
    gesture.dispatch('config', {'instance_id': 'gesture-card', 'bound_side': 'left'})
    hand.dispatch('config', {'instance_id': 'basic-card', 'bound_side': 'left'})
    hand._controllers[('left', 'can0')] = ctrl
    with pytest.raises(RuntimeError, match='请先.*release'):
        gesture.dispatch('number', {'instance_id': 'gesture-card', 'bound_side': 'left', 'number': 0, 'confirm_motion': True})
    with pytest.raises(RuntimeError, match='请先.*release'):
        hand.dispatch('open', {'instance_id': 'basic-card', 'bound_side': 'left', 'confirm_motion': True})
    gesture.dispatch('stop', {'instance_id': 'gesture-card'})
    assert not hand.dispatch('stop', {'instance_id': 'basic-card'})['torque_released']
    assert client._currents == [p.motion_current] * 5
    request(p, 'release')
    hand.dispatch('config', {'instance_id': 'basic-card', 'bound_side': 'left'})
    assert hand.dispatch('open', {'instance_id': 'basic-card', 'bound_side': 'left', 'confirm_motion': True})['state'] == 'running'


def test_already_released_action_does_not_repeat_detour(connected):
    p, client, bus, ctrl, *_ = connected
    client.set_joint_positions(ctrl._open_pose)
    bus.sent_messages.clear()
    result = request(p, 'release')
    assert result['already_released'] and result['torque_released']
    assert not writes(bus)
    assert not any(m.data[0] == 0x81 for m in bus.sent_messages)
    assert client._currents == [0] * 5


@pytest.mark.parametrize('blocked_joint', list(range(11)) + list(range(15, 20)))
def test_release_waits_for_full_preparation_before_thumb_reorientation(connected, monkeypatch, blocked_joint):
    p, client, _, ctrl, *_ = connected
    request(p)
    commands = []
    move = client.set_joint_positions
    def block(target):
        commands.append(list(target))
        measured = list(target)
        measured[blocked_joint] = 0 if target[blocked_joint] > 127 else 255
        move(measured)
    monkeypatch.setattr(client, 'set_joint_positions', block)
    result = request(p, 'release')
    assert not result['success'] and result['error_code'] == 'pose_timeout'
    assert not result['hand_opened'] and result['torque_released']
    assert commands == [DEFAULT_GRASP_PREP_POSE]
    assert client._currents == [0] * 5 and not vars(ctrl).get('_grasp_owner')


def test_unreached_preparation_never_sends_closing_target(connected, monkeypatch):
    p, client, _, ctrl, *_ = connected
    move = client.set_joint_positions
    commands = []
    def block_prep(target):
        commands.append(list(target))
        if target == DEFAULT_GRASP_PREP_POSE:
            changed = list(target)
            changed[10] = 255
            move(changed)
        else:
            move(target)
    monkeypatch.setattr(client, 'set_joint_positions', block_prep)
    result = request(p)
    assert result['error_code'] == 'pose_timeout' and not result['success']
    assert not any(v[:5] == [0] * 5 for v in commands)
    assert client._currents == [0] * 5 and not vars(ctrl).get('_grasp_owner')


def test_release_preempts_preparation_and_inherits_top_mode(connected, monkeypatch):
    p, client, _, ctrl, _, events, _ = connected
    entered = threading.Event()
    def preparation(*args):
        entered.set()
        while True:
            p._wait(0.01)
    monkeypatch.setattr(p, '_do_grasp', preparation)
    preparing = request(p, 'top_grasp', wait_for_result=False)
    assert entered.wait(2)
    # Put the hand into a top-grasp-shaped closed pose for the release path.
    client.set_joint_positions(p._grasp_poses(ctrl, 'top')[1])
    commands = []
    move = client.set_joint_positions
    def capture(target):
        commands.append(list(target))
        move(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    released = request(p, 'release')
    assert released['hand_opened'] and released['torque_released']
    assert commands[0] == DEFAULT_TOP_GRASP_PREP_POSE
    cancelled = [event for event in events if event[0] == preparing['action_id']]
    assert len(cancelled) == 1 and cancelled[0][1] == 'cancelled'
    assert not p._active and not p._sessions


def test_async_completion_then_release_does_not_wait_for_blocked_callback(connected):
    p, client, _, _, _, _, _ = connected
    entered, finish = threading.Event(), threading.Event()
    record = p._acp_callback
    def callback(*args):
        record(*args)
        entered.set()
        assert finish.wait(3)
    p._acp_callback = callback
    try:
        reply = request(p, wait_for_result=False)
        assert reply['state'] == 'running' and entered.wait(2)
        assert not p._active
        result = request(p, 'release')
        assert result['success'] and client._currents == [0] * 5
    finally:
        finish.set()


def test_failed_callback_preserves_dispatched_motor_command(connected):
    p, client, _, ctrl, *_ = connected
    p._acp_callback = lambda *args: (_ for _ in ()).throw(RuntimeError('offline'))
    request(p, wait_for_result=False)
    deadline = time.monotonic() + 2
    while not p._delivery_failed and time.monotonic() < deadline:
        time.sleep(0.005)
    assert p._delivery_failed and not p._active
    assert client._currents == [p.motion_current] * 5 and vars(ctrl).get('_grasp_owner')


def test_thread_start_failure_unloads_and_releases_ownership(connected, monkeypatch):
    p, client, bus, ctrl, *_ = connected
    def fail(*args):
        raise RuntimeError('thread unavailable')
    monkeypatch.setattr(threading.Thread, 'start', fail)
    with pytest.raises(RuntimeError, match='thread unavailable'):
        request(p)
    assert not p._active and not p._sessions and not vars(ctrl).get('_grasp_owner')
    assert client._currents == [0] * 5 and not writes(bus)


def test_duplicate_release_reuses_same_worker(connected, monkeypatch):
    p, _, _, _, _, events, _ = connected
    queued = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: queued.append(worker))
    first = request(p, 'release', wait_for_result=False)
    second = p.dispatch('release', {'bound_side': 'left', 'confirm_motion': True})
    assert first['action_id'] == second['action_id'] and len(queued) == 1
    queued[0].run()
    assert not p._active
    assert len(events) == 1 and events[0][0] == first['action_id']
    assert events[0][1] == 'completed'
    third = p.dispatch('release', {'bound_side': 'left', 'confirm_motion': True})
    assert third['action_id'] != first['action_id'] and len(queued) == 2
    queued[1].run()
    assert len(events) == 2 and events[1][0] == third['action_id']


def test_info_is_per_device_and_does_not_read_hardware(connected):
    p, _, bus, *_ = connected
    request(p)
    bus.sent_messages.clear()
    assert p.dispatch('info', {'instance_id': 'other-card'})['last_result'] is None
    assert p.dispatch('info', {'instance_id': 'other-card'})['selection_required']
    assert p.dispatch('info', {'bound_side': 'right'})['last_result'] is None
    assert p.dispatch('info', {'bound_side': 'left', 'instance_id': 'other-card'})['state'] == 'holding'
    assert p.dispatch('info', {'bound_side': 'left'})['last_result']['result']['command_sent']
    assert not bus.sent_messages


@pytest.mark.parametrize('side', [None, '', 'LEFT', 'both', 1, True])
@pytest.mark.parametrize('action', ['side_grasp', 'top_grasp', 'release'])
def test_each_motion_requires_explicit_side_without_binding_fallback(connected, side, action):
    p, _, bus, *_ = connected
    p.dispatch('config', {'instance_id': 'grasp-card', 'bound_side': 'left'})
    with pytest.raises(ValueError, match='bound_side'):
        request(p, action, bound_side=side)
    assert not bus.sent_messages


@pytest.mark.parametrize('mode,action', [('side', 'side_grasp'), ('top', 'top_grasp')])
@pytest.mark.parametrize('release_source', ['', 'another-card'])
def test_anonymous_grasp_can_be_released_from_another_card_and_retains_geometry(connected, monkeypatch, mode, action, release_source):
    p, client, _, ctrl, *_ = connected
    result = request(p, action, instance_id='')
    assert result['success'] and result['bound_side'] == 'left'
    assert p._sessions[('left', 'can0')]['source_instance_id'] == ''
    positions = []
    move = client.set_joint_positions
    def capture(target):
        positions.append(list(target))
        move(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    released = request(p, 'release', instance_id=release_source)
    assert released['success'] and released['torque_released']
    assert positions[0] == p._grasp_prep_pose(ctrl, mode)
    assert not p._sessions and not p._active


def test_id_only_stop_cannot_infer_anonymous_grasp_or_use_legacy_configuration(connected):
    p, client, bus, ctrl, *_ = connected
    request(p, instance_id='')
    bus.sent_messages.clear()
    p.dispatch('config', {'instance_id': 'guessed-card', 'bound_side': 'left'})
    for source in ({}, {'instance_id': 'guessed-card'}):
        stopped = p.dispatch('stop', source)
        assert not stopped['target_resolved'] and not stopped['torque_released']
    assert not bus.sent_messages
    assert client._currents == [p.motion_current] * 5
    assert vars(ctrl).get('_grasp_owner') and p._sessions
    assert request(p, 'release', instance_id='new-card')['success']


def test_explicit_stop_can_target_anonymous_grasp(connected):
    p, client, _, ctrl, *_ = connected
    request(p, instance_id='')
    stopped = p.dispatch('stop', {'bound_side': 'left'})
    assert stopped['target_resolved'] and stopped['torque_released']
    assert client._currents == [0] * 5 and not vars(ctrl).get('_grasp_owner')


def test_release_clears_prior_tool_hold_marker_after_grasp_takeover(connected):
    p, client, _, ctrl, *_ = connected
    ctrl._holding_torque = 80
    client.set_currents([80] * 5)
    request(p, instance_id='')
    assert request(p, 'release', instance_id='another-card')['torque_released']
    assert client._currents == [0] * 5 and ctrl._holding_torque == 0


def test_failed_connection_check_preserves_previous_tool_hold_ownership(connected, monkeypatch):
    p, client, _, ctrl, *_ = connected
    previous_owner = (12345, 'existing-held-action')
    ctrl._last_command_owner = previous_owner
    ctrl._holding_torque = 80
    client.set_currents([80] * 5)
    def offline(*args):
        raise RuntimeError('offline')
    monkeypatch.setattr('grasp_device.verify_hand_connection', offline)
    with pytest.raises(RuntimeError, match='offline'):
        request(p)
    assert vars(ctrl).get('_last_command_owner') == previous_owner
    assert not vars(ctrl).get('_command_owner') and not vars(ctrl).get('_grasp_owner')
    assert client._currents == [80] * 5 and ctrl._holding_torque == 80


def test_old_source_cannot_cancel_release_started_by_another_card(connected, monkeypatch):
    p, _, _, _, *_ = connected
    request(p)
    workers = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: workers.append(worker))
    request(p, 'release', instance_id='new-card', wait_for_result=False)
    active = p._active[('left', 'can0')]
    stopped = p.dispatch('stop', {'instance_id': 'grasp-card'})
    assert not stopped['target_resolved'] and not active['stop_event'].is_set()
    workers[0].run()
    assert active['terminal']['status'] == 'completed'


def test_two_hands_grasp_independently_and_stop_only_matching_source(connected, monkeypatch):
    p, left_client, _, left_ctrl, hand, *_ = connected
    right_bus = ScalarBus(can_id=0x27)
    right_client = L20CanClient(side='right', can_id=0x27, recv_timeout_s=0.05, bus_factory=lambda *_: right_bus)
    right_ctrl = _SingleHandController({}, 'right', 'can0', threading.RLock(), can_id=0x27)
    right_ctrl._client = right_client
    hand.get_controller_for_side = lambda side, *_: left_ctrl if side == 'left' else right_ctrl
    try:
        request(p, 'top_grasp', bound_side='left', instance_id='left-card')
        request(p, 'side_grasp', bound_side='right', instance_id='right-card')
        assert set(p._sessions) == {('left', 'can0'), ('right', 'can0')}
        with pytest.raises(RuntimeError, match='release'):
            request(p, bound_side='left', instance_id='third-card')
        stopped = p.dispatch('stop', {'instance_id': 'left-card'})
        assert stopped['torque_released'] and left_client._currents == [0] * 5
        assert right_client._currents == [p.motion_current] * 5
        assert set(p._sessions) == {('right', 'can0')}
        assert request(p, 'release', bound_side='right', instance_id='third-card')['success']
    finally:
        p.stop()
        right_client.close()


def test_two_hands_can_prepare_concurrently_without_one_active_slot(connected, monkeypatch):
    p, _, _, left_ctrl, hand, *_ = connected
    right_bus = ScalarBus(can_id=0x27)
    right_client = L20CanClient(side='right', can_id=0x27, recv_timeout_s=0.05, bus_factory=lambda *_: right_bus)
    right_ctrl = _SingleHandController({}, 'right', 'can0', threading.RLock(), can_id=0x27)
    right_ctrl._client = right_client
    hand.get_controller_for_side = lambda side, *_: left_ctrl if side == 'left' else right_ctrl
    entered = {side: threading.Event() for side in ('left', 'right')}
    finish = threading.Event()
    run_grasp = p._do_grasp
    def preparation(client, ctrl, options):
        entered[ctrl.side].set()
        while not finish.is_set():
            p._wait(0.005)
        return run_grasp(client, ctrl, options)
    monkeypatch.setattr(p, '_do_grasp', preparation)
    tasks = []
    try:
        request(p, 'top_grasp', instance_id='', wait_for_result=False)
        assert entered['left'].wait(2)
        request(p, 'side_grasp', bound_side='right', instance_id='', wait_for_result=False)
        assert entered['right'].wait(2)
        tasks = list(p._active.values())
        assert len(tasks) == 2
        finish.set()
        assert all(task['done_event'].wait(3) for task in tasks)
        assert p._sessions[('left', 'can0')]['grasp_mode'] == 'top'
        assert p._sessions[('right', 'can0')]['grasp_mode'] == 'side'
    finally:
        finish.set()
        p.stop()
        for task in tasks:
            assert task['done_event'].wait(3)
        right_client.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_side_specific_preparation_calibration_is_preserved(connected, side):
    p, _, _, ctrl, *_ = connected
    calibrated = list(DEFAULT_GRASP_PREP_POSE)
    calibrated[5] = 120
    p.config[f'{side}_grasp_prep_pose'] = calibrated
    ctrl.side = side
    prep, target = p._grasp_poses(ctrl)
    assert prep == calibrated and target[5] == 120
    assert target[:5] == [0] * 5 and target[15:] == [0] * 5

def test_scalar_query_rejects_cached_partial_and_matrix_replies(connected):
    _, client, bus, *_ = connected
    snap = client.get_fresh_tactile()
    assert set(snap['touch_scalar'].values()) == {35}
    assert all(age is not None for age in snap['scalar_age_s'].values())
    bus.scalar_online = False
    with pytest.raises(TimeoutError, match='本轮完整'):
        client.get_fresh_tactile(timeout_s=0.02)
    bus.scalar_online = True
    bus.missing_scalar = 0xB5
    with pytest.raises(TimeoutError, match='pinky'):
        client.get_fresh_tactile(timeout_s=0.02)
    bus.missing_scalar = None
    bus.matrix_only = True
    with pytest.raises(TimeoutError, match='本轮完整'):
        client.get_fresh_tactile(timeout_s=0.02)


def test_scalar_query_can_be_cancelled(connected):
    _, client, bus, *_ = connected
    bus.scalar_online = False
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(InterruptedError):
        client.get_fresh_tactile(cancel=cancel)


def test_optional_matrix_requires_all_new_rows(connected, monkeypatch):
    _, client, bus, *_ = connected
    original = bus.send
    enabled = [True]

    def matrices(msg):
        if 0xB1 <= msg.data[0] <= 0xB5 and list(msg.data[1:]) == [0xC6]:
            bus.sent_messages.append(msg)
            if enabled[0]:
                for row in range(12):
                    bus._inbox.put(MockCanMessage(bus.can_id, [msg.data[0], row * 16] + [row] * 6))
        else:
            original(msg)

    monkeypatch.setattr(bus, 'send', matrices)
    result = client.get_fresh_matrix_touch(timeout_s=0.05)
    assert len(result) == 5 and result['thumb'][11] == [11] * 6
    enabled[0] = False
    with pytest.raises(TimeoutError, match='本轮完整'):
        client.get_fresh_matrix_touch(timeout_s=0.02)


def test_real_shared_controllers_have_independent_motion_locks():
    from hand_device import get_shared_controller, GLOBAL_CONTROLLERS
    channel = 'grasp-v1-test-isolation'
    registry = threading.RLock()
    try:
        left = get_shared_controller({}, 'left', channel, bus_lock=registry)
        right = get_shared_controller({}, 'right', channel, bus_lock=registry)
        assert left._motion_lock is not registry
        assert right._motion_lock is not left._motion_lock
    finally:
        GLOBAL_CONTROLLERS.pop(('left', channel), None)
        GLOBAL_CONTROLLERS.pop(('right', channel), None)


def test_standalone_grasp_keeps_manufacturer_pose_config(connected, monkeypatch):
    _, _, _, ctrl, *_ = connected
    captured = {}

    def controller(config, side, channel):
        captured.update(config)
        return ctrl

    monkeypatch.setattr('hand_device.get_shared_controller', controller)
    calibration = list(ctrl._fist_pose)
    p = GraspPlugin(None, {}, global_config={'plugins': {'hand': {'enabled': False, 'fist_pose': calibration}}})
    p._get_client_and_ctrl('left')
    assert captured['fist_pose'] == calibration


def test_joint_torque_codec_controls_tips_without_overwriting_roots_or_orientation(connected):
    p, client, bus, *_ = connected
    limits = [230] * 11 + [0] * 4 + [230] * 5
    limits[15:20] = [230, 242, 251, 233, 245]
    bus.sent_messages.clear()
    assert client.set_joint_torque_limits(limits) == limits
    frames = [list(m.data) for m in bus.sent_messages]
    assert frames == [
        [0x51, 230, 230, 230, 0, 0, 230],
        [0x52, 230, 0, 230, 0, 0, 242],
        [0x53, 230, 0, 230, 0, 0, 251],
        [0x54, 230, 0, 230, 0, 0, 233],
        [0x55, 230, 0, 230, 0, 0, 245],
    ]


@pytest.mark.parametrize('action', ['side_grasp', 'top_grasp', 'release'])
@pytest.mark.parametrize('options', [{}, {'wait_for_result': False}])
def test_completion_actions_return_correlatable_id_without_waiting(connected, monkeypatch, action, options):
    p, _, _, _, _, events, _ = connected
    queued = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: queued.append(worker))
    reply = p.dispatch(action, {'bound_side': 'left', 'confirm_motion': True, **options})
    assert reply['state'] == 'running' and isinstance(reply['action_id'], str)
    assert len(queued) == 1 and not events
    assert 'completed_action_id' not in reply and 'success' not in reply
    assert not p._results
    queued[0].run()
    assert len(events) == 1
    assert events[0][0] == reply['action_id'] and events[0][1] == 'completed'
    assert events[0][2]['success']


@pytest.mark.parametrize('action', ['side_grasp', 'top_grasp', 'release'])
@pytest.mark.parametrize('value', [True, 'false', 0, 1, None, [], {}])
def test_sync_or_malformed_wait_is_rejected_before_hardware(connected, monkeypatch, action, value):
    p, _, bus, *_ = connected
    monkeypatch.setattr(p, '_get_client_and_ctrl', no_touch)
    with pytest.raises(ValueError, match='wait_for_result'):
        p.dispatch(action, {'bound_side': 'left', 'confirm_motion': True, 'wait_for_result': value})
    assert not bus.sent_messages and not p._active


@pytest.mark.parametrize('status', ['completed', 'error', 'cancelled'])
def test_each_terminal_outcome_reports_one_matching_completion(connected, monkeypatch, status):
    p, _, _, ctrl, _, events, _ = connected
    queued = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: queued.append(worker))
    if status == 'error':
        def fail(*args):
            raise RuntimeError('simulated grasp failure')
        monkeypatch.setattr(p, '_do_grasp', fail)
    reply = p.dispatch('side_grasp', {'instance_id': 'owner', 'bound_side': 'left', 'confirm_motion': True})
    if status == 'cancelled':
        assert p.dispatch('stop', {'instance_id': 'owner'})['state'] == 'idle'
    queued[0].run()
    assert len(events) == 1 and events[0][0] == reply['action_id']
    assert events[0][1] == status and events[0][2]['success'] == (status == 'completed')
    assert not p._active and not vars(ctrl).get('_command_owner')
    assert bool(vars(ctrl).get('_grasp_owner')) == (status == 'completed')
    p.dispatch('stop', {'instance_id': 'owner'})
    assert len(events) == 1, 'lifecycle cleanup must not duplicate a terminal notification'


@pytest.mark.parametrize('with_hand', [True, False])
def test_mcp_response_and_acp_transport_share_task_identity(connected, monkeypatch, with_hand):
    import io
    import json
    import urllib.request
    from common.vendor_runtime import DriverBundle, make_handler

    p, client, _, ctrl, *_ = connected
    if not with_hand:
        p.hand = None
        monkeypatch.setattr(p, '_get_client_and_ctrl', lambda side, channel: (client, side, ctrl))
    # Exercise the actual completion transport, with the HTTP boundary mocked.
    monkeypatch.delattr(p, '_acp_callback')
    monkeypatch.setenv('AGENT_CORE_URL', 'http://127.0.0.1:15678')
    monkeypatch.delenv('AGENT_CORE_CA_CERT', raising=False)
    callbacks = []

    def complete(req, **kwargs):
        payload = json.loads(req.data)
        callbacks.append((req.full_url, payload))
        class Response:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def read(self):
                return json.dumps({'ok': True, 'action_id': payload['action_id']}).encode()
        return Response()

    monkeypatch.setattr(urllib.request, 'urlopen', complete)
    queued = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: queued.append(worker))
    bundle = DriverBundle([p])
    handler_type = make_handler(lambda: bundle, 'l20-test', 'l20-test-driver')
    handler = object.__new__(handler_type)
    body = json.dumps({'jsonrpc': '2.0', 'id': 42, 'method': 'tools/call', 'params': {
        'name': 'hand_grasp', 'arguments': {'action': 'side_grasp', 'bound_side': 'left', 'confirm_motion': True},
    }}).encode()
    handler.path = '/mcp'
    handler.headers = {'Content-Length': str(len(body))}
    handler.rfile = io.BytesIO(body)
    responses = []
    handler.send_json = lambda code, data: responses.append((code, data))
    handler.do_POST()
    assert responses[0][0] == 200
    accepted = json.loads(responses[0][1]['result']['content'][0]['text'])
    assert accepted['state'] == 'running' and accepted['action_id']
    assert not callbacks
    queued[0].run()
    assert len(callbacks) == 1
    url, payload = callbacks[0]
    assert url == 'http://127.0.0.1:15678/api/acp/complete'
    assert payload['action_id'] == accepted['action_id']
    assert payload['status'] == 'completed' and payload['tool'] == 'hand_grasp'
    assert payload['result']['command_sent'] and payload['result']['holding']
    assert payload['result']['success']
