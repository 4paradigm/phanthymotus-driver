"""Explicit target selection, task ownership and powered CAN hand hot-swaps."""

import threading

import pytest

from test_linkerhand_l20 import (
    GesturePlugin, GraspPlugin, HandPlugin, L20CanClient, MockCanBus,
    _SingleHandController,
)


class SwitchableCanBus(MockCanBus):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.online = True
        self.missing_frame = None

    def send(self, msg):
        if not self.online or msg.data[0] == self.missing_frame:
            self.sent_messages.append(msg)
            return
        super().send(msg)


@pytest.fixture
def connected_hand():
    bus = SwitchableCanBus(can_id=0x28)
    client = L20CanClient(
        side="left", can_id=0x28, recv_timeout_s=0.05,
        bus_factory=lambda *_: bus,
    )
    ctrl = _SingleHandController({}, "left", "can0", threading.RLock(), can_id=0x28)
    ctrl._client = client
    hand = HandPlugin({"left_can_id": 0x28, "right_can_id": 0x27}, {})
    def controller_for_side(side, channel=None):
        hand._controllers[(side, channel or hand.config.get('can_channel', 'can0'))] = ctrl
        return ctrl
    hand.get_controller_for_side = controller_for_side
    try:
        yield hand, client, bus, ctrl
    finally:
        client.close()


@pytest.fixture(params=[HandPlugin, GesturePlugin, GraspPlugin])
def card(request, connected_hand):
    hand, *_ = connected_hand
    return hand if request.param is HandPlugin else request.param(hand, {})


def motion_for(card):
    return {HandPlugin: "open", GesturePlugin: "ok_gesture", GraspPlugin: "side_grasp"}[type(card)]


def test_probe_requires_a_new_complete_reply(connected_hand):
    _, client, bus, _ = connected_hand
    assert client.probe_positions(timeout_s=0.05)
    bus.online = False
    assert not client.probe_positions(timeout_s=0.05), "old positions must not prove a hand is online"
    with pytest.raises(TimeoutError, match="左手.*0x28"):
        client.get_positions_vector()
    bus.online = True
    assert client.probe_positions(timeout_s=0.05)
    bus.missing_frame = 0x45
    assert not client.probe_positions(timeout_s=0.05), "every finger must reply to this query"


def test_reserved_zeros_do_not_prove_presence():
    bus = SwitchableCanBus(can_id=0x28)
    bus.online = False
    client = L20CanClient(side="left", bus_factory=lambda *_: bus, recv_timeout_s=0.05)
    try:
        assert not client.probe_positions(timeout_s=0.05)
    finally:
        client.close()


@pytest.mark.parametrize("side", [None, "", "LEFT", "left ", True, 0, [], {}])
def test_each_execution_requires_a_valid_explicit_side(card, connected_hand, side):
    _, _, bus, _ = connected_hand
    # Legacy config must not establish a default for later calls.
    card.dispatch("config", {"instance_id": "owner", "bound_side": "left"})
    bus.sent_messages.clear()
    args = {"confirm_motion": True, "wait_for_result": False}
    if side is not None:
        args["bound_side"] = side
    with pytest.raises(ValueError, match="bound_side"):
        card.dispatch(motion_for(card), args)
    assert not bus.sent_messages
    assert not card._active


def test_lifecycle_start_and_config_do_not_require_a_target(card, connected_hand):
    _, _, bus, _ = connected_hand
    bus.sent_messages.clear()
    assert card.dispatch("start", {"instance_id": "card"})["state"] == "ready"
    assert card.dispatch("config", {"instance_id": "card"})["state"] == "configured"
    assert not bus.sent_messages
    assert not hasattr(card, "_bindings")


def test_anonymous_motion_uses_only_the_explicit_target(card, connected_hand, monkeypatch):
    _, _, bus, _ = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, "start", lambda worker: workers.append(worker))
    card._acp_callback = lambda *args: None
    reply = card.dispatch(motion_for(card), {
        "bound_side": "left", "confirm_motion": True, "wait_for_result": False,
    })
    assert reply["state"] == "running"
    assert ("left", "can0") in card._active
    # An unrelated deletion cannot identify this anonymous request.
    card.dispatch("stop", {"instance_id": "deleted-card"})
    active = card._active[("left", "can0")]
    event = active.get("stop_event", active["controller"]._stop_evt)
    assert not event.is_set()
    card.dispatch("stop", {"bound_side": "left"})
    assert event.is_set()
    bus.sent_messages.clear()
    workers[0].run()
    assert not card._active
    assert not any(0x41 <= msg.data[0] <= 0x45 and len(msg.data) > 1 for msg in bus.sent_messages)


def test_hot_swap_is_rejected_before_accepting_motion(card, connected_hand):
    _, client, bus, _ = connected_hand
    assert client.probe_positions(timeout_s=0.05)
    # The USB adapter remains, but the left CAN ID no longer replies.
    bus.online = False
    bus.sent_messages.clear()
    with pytest.raises(RuntimeError, match="左手.*0x28"):
        card.dispatch(motion_for(card), {"bound_side": "left", "wait_for_result": False, "instance_id": "owner", "confirm_motion": True})
    assert all(len(msg.data) == 1 for msg in bus.sent_messages), "only read-only probes may be sent"
    active = card._active
    assert not active, "an offline hand must not leave a pending action"


@pytest.mark.parametrize("action", ["get_state"])
def test_read_actions_report_disconnected_hand(action, connected_hand):
    hand, client, bus, _ = connected_hand
    card = hand
    assert client.probe_positions(timeout_s=0.05)
    bus.online = False
    with pytest.raises((RuntimeError, TimeoutError), match="左手.*0x28"):
        card.dispatch(action, {"bound_side": "left", "instance_id": "owner"})


def test_stopping_other_instance_does_not_cancel_active_card(card, monkeypatch):
    workers = []
    monkeypatch.setattr(threading.Thread, "start", lambda worker: workers.append(worker))
    card._acp_callback = lambda *args: None
    card.dispatch(motion_for(card), {"bound_side": "left", "instance_id": "left-card",
                                   "confirm_motion": True, "wait_for_result": False})
    active = card._active[("left", "can0")]
    card.dispatch("stop", {"instance_id": "right-card"})
    event = active.get("stop_event", active["controller"]._stop_evt)
    assert not event.is_set()
    card.dispatch("stop", {"instance_id": "left-card"})
    assert event.is_set()
    workers[0].run()


def test_stop_before_worker_runs_cancels_without_motion(card, connected_hand, monkeypatch):
    _, _, bus, _ = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, "start", lambda worker: workers.append(worker))
    completed = []
    card._acp_callback = lambda aid, status, result: completed.append(status)
    card.dispatch(motion_for(card), {"bound_side": "left", "wait_for_result": False, "instance_id": "owner", "confirm_motion": True})
    card.dispatch("stop", {"instance_id": "owner"})
    bus.sent_messages.clear()
    workers[0].run()
    assert completed == ["cancelled"]
    # Error cleanup may send zero torque, but must never command a position.
    assert not any(0x41 <= msg.data[0] <= 0x45 and len(msg.data) > 1 for msg in bus.sent_messages)
    assert not card._active


def test_disconnection_between_accept_and_worker_reports_acp_error(card, connected_hand, monkeypatch):
    _, _, bus, _ = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, "start", lambda worker: workers.append(worker))
    completed = []
    card._acp_callback = lambda aid, status, result: completed.append((status, result))
    card.dispatch(motion_for(card), {"bound_side": "left", "wait_for_result": False, "instance_id": "owner", "confirm_motion": True})
    bus.online = False
    bus.sent_messages.clear()
    workers[0].run()
    assert completed[0][0] == "error"
    assert "左手" in completed[0][1]["message"]
    assert "0x28" in completed[0][1]["message"]
    assert not any(0x41 <= msg.data[0] <= 0x45 and len(msg.data) > 1 for msg in bus.sent_messages)
    assert not card._active


@pytest.mark.parametrize('first_type', [HandPlugin, GesturePlugin, GraspPlugin])
@pytest.mark.parametrize('second_type', [HandPlugin, GesturePlugin, GraspPlugin])
def test_actuator_tools_reject_competing_commands_on_same_hand(first_type, second_type, connected_hand, monkeypatch):
    hand, _, _, ctrl = connected_hand
    first = hand if first_type is HandPlugin else first_type(hand, {})
    second = hand if second_type is HandPlugin else second_type(hand, {})
    workers = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: workers.append(worker))
    first._acp_callback = lambda *args: None
    args = {'bound_side': 'left', 'confirm_motion': True, 'wait_for_result': False}
    first.dispatch(motion_for(first), args)
    with pytest.raises(RuntimeError):
        second.dispatch(motion_for(second), args)
    first.dispatch('stop', {'bound_side': 'left'})
    workers[0].run()
    assert not vars(ctrl).get('_command_owner')


@pytest.mark.parametrize('failure', ['probe', 'thread'])
def test_hand_failed_replacement_keeps_previous_hold_stoppable(connected_hand, monkeypatch, failure):
    import hand_device
    hand, _, _, ctrl = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: workers.append(worker))
    hand._acp_callback = lambda *args: None
    hand.dispatch('open', {'bound_side': 'left', 'instance_id': 'old',
                          'hold_torque': 80, 'confirm_motion': True})
    workers.pop().run()
    record = hand._held[('left', 'can0')]
    owner = ctrl._last_command_owner
    if failure == 'probe':
        def fail(_):
            raise RuntimeError('probe failed')
        monkeypatch.setattr(hand_device, 'verify_hand_connection', fail)
    else:
        def fail(_):
            raise RuntimeError('thread failed')
        monkeypatch.setattr(threading.Thread, 'start', fail)
    with pytest.raises(RuntimeError, match='failed'):
        hand.dispatch('close', {'bound_side': 'left', 'instance_id': 'new', 'confirm_motion': True})
    assert hand._held[('left', 'can0')] is record
    assert ctrl._last_command_owner == owner
    assert ctrl._holding_torque == 80
    assert not vars(ctrl).get('_command_owner')
    hand.dispatch('stop', {'instance_id': 'old'})
    assert ctrl._holding_torque == 0


def test_hand_failed_worker_actually_unloads_requested_nonzero_hold(connected_hand, monkeypatch):
    hand, client, _, ctrl = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: workers.append(worker))
    hand._acp_callback = lambda *args: None
    writes = []
    monkeypatch.setattr(client, 'set_currents', lambda value: writes.append(value))
    def timed_out(*args, **kwargs):
        client.set_currents([80] * 5)
        return {'state': 'released_timeout'}
    monkeypatch.setattr(ctrl, 'move', timed_out)
    hand.dispatch('open', {'bound_side': 'left', 'hold_torque': 80, 'confirm_motion': True})
    workers[0].run()
    assert writes[-1] == [0] * 5
    assert ctrl._holding_torque == 0
    assert not hand._held


def test_shared_can_change_blocked_until_held_torque_is_released(connected_hand, monkeypatch):
    hand, _, _, _ = connected_hand
    workers = []
    monkeypatch.setattr(threading.Thread, 'start', lambda worker: workers.append(worker))
    hand._acp_callback = lambda *args: None
    hand.dispatch('open', {'bound_side': 'left', 'hold_torque': 80, 'confirm_motion': True})
    with pytest.raises(RuntimeError, match='CAN'):
        hand.dispatch('config', {'can_channel': 'can1'})
    workers.pop().run()
    with pytest.raises(RuntimeError, match='CAN'):
        hand.dispatch('config', {'can_channel': 'can1'})
    hand.dispatch('stop', {'bound_side': 'left'})
    hand.dispatch('config', {'can_channel': 'can1'})
    assert hand.config['can_channel'] == 'can1'


def test_stale_channel_request_is_rejected_before_any_probe(connected_hand, monkeypatch):
    hand, _, bus, ctrl = connected_hand
    def switched(side, channel=None):
        hand.config['can_channel'] = 'can1'
        return ctrl
    monkeypatch.setattr(hand, '_get_controller', switched)
    bus.sent_messages.clear()
    with pytest.raises(RuntimeError, match='CAN'):
        hand.dispatch('open', {'bound_side': 'left', 'confirm_motion': True})
    assert not bus.sent_messages
    assert not hand._active
    assert not vars(ctrl).get('_command_owner')
