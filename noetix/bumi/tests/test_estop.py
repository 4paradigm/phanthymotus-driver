"""Contract tests for the Bumi software emergency stop.

The latch has to be visible to every motion path, not just the card, so the
tests drive `LocoPlugin` directly: a rule that only exists in the card's
`dispatch` would let `loco_servo` keep streaming.
"""

import importlib.util
import sys
import threading
import time
import types
from pathlib import Path

import pytest

from test_mic_card import _load_device


@pytest.fixture
def estop(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'estop.py'
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location('bumi_estop_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeHighCtrl:
    def __init__(self, mode=2):
        self.mode = mode
        self.commands = []
        self.action_commands = []

    def get_mode(self):
        return self.mode

    def publish_cmd(self, x, y, z, action_cmd, index=0):
        self.commands.append((x, y, z))
        self.action_commands.append(action_cmd)


@pytest.fixture
def device(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, 'highcontrol_py', types.SimpleNamespace(
        ControlCmd=types.SimpleNamespace(DEFAULT='DEFAULT')))
    return module


def test_latch_is_one_way_until_reset(estop):
    latch = estop.EmergencyStopLatch()

    assert latch.latched is False
    assert latch.reset() is False
    assert latch.trigger('test', 'because') is True
    assert latch.trigger('again') is False
    assert latch.latched is True
    snap = latch.snapshot()
    assert snap['source'] == 'test'
    assert snap['reason'] == 'because'
    assert snap['since_ms'] is not None
    assert latch.reset() is True
    assert latch.latched is False


def test_refuses_only_non_zero_velocity(estop):
    latch = estop.EmergencyStopLatch()

    assert latch.refuses(0.2, 0.0, 0.0) is False   # idle: nothing to refuse
    latch.trigger('test')
    assert latch.refuses(0.2, 0.0, 0.0) is True
    assert latch.refuses(0.0, 0.0, 0.0) is False   # a stop is always allowed
    assert latch.snapshot()['blocked_commands'] == 1


def test_latch_survives_a_restart(estop, tmp_path):
    path = tmp_path / 'estop_state.json'
    latch = estop.EmergencyStopLatch(str(path))
    assert latch.latched is False
    latch.trigger('estop_card', 'because')

    restored = estop.EmergencyStopLatch(str(path))
    assert restored.latched is True
    assert restored.snapshot()['source'] == 'estop_card'
    assert restored.snapshot()['reason'] == 'because'

    assert restored.reset() is True
    assert estop.EmergencyStopLatch(str(path)).latched is False


def test_unreadable_state_fails_closed(estop, tmp_path):
    path = tmp_path / 'estop_state.json'
    path.write_text('{ this is not json')
    latch = estop.EmergencyStopLatch(str(path))
    assert latch.latched is True
    assert latch.snapshot()['source'] == 'estop_state_unreadable'


def test_latched_loco_refuses_event_commands_inside_the_gate(device, estop):
    """`FALLTOSTAND` has zero velocity but still starts physical motion."""
    latch = estop.EmergencyStopLatch()
    high = _FakeHighCtrl()
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)
    latch.trigger('estop_card')

    with pytest.raises(device.EmergencyStopActive):
        loco._send_edge_and_wait('FALLTOSTAND', {27})
    assert 'FALLTOSTAND' not in high.action_commands

    # The documented WALK exit stays callable while latched.
    loco._send_edge_and_wait('WALK', {2}, allow_when_latched=True)
    assert 'WALK' in high.action_commands


def test_on_latch_callback_runs_once_per_trigger(estop):
    latch = estop.EmergencyStopLatch()
    calls = []
    latch.add_on_latch(lambda source, reason: calls.append((source, reason)))

    latch.trigger('card', 'operator')
    latch.trigger('card', 'operator')              # already latched: no repeat
    assert calls == [('card', 'operator')]


def test_chassis_commands_are_dropped_at_the_shared_choke_point(device, estop):
    latch = estop.EmergencyStopLatch()
    high = _FakeHighCtrl()
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)

    assert loco._publish_cmd(0.2, 0.0, 0.0, 'DEFAULT') is True
    assert high.commands == [(0.2, 0.0, 0.0)]

    latch.trigger('estop_card', 'test')
    # on-latch callback zeroes the chassis before trigger returns to the caller
    assert high.commands[-1] == (0.0, 0.0, 0.0)

    published = len(high.commands)
    assert loco._publish_cmd(0.2, 0.0, 0.0, 'DEFAULT') is False
    assert len(high.commands) == published
    assert loco.estop_latched() is True

    assert loco._publish_cmd(0.0, 0.0, 0.0, 'DEFAULT') is True
    assert len(high.commands) == published + 1


def test_latched_loco_refuses_motion_but_allows_stop(device, estop):
    latch = estop.EmergencyStopLatch()
    loco = device.LocoPlugin({}, 'robot', None, _FakeHighCtrl())
    loco.attach_estop(latch)
    latch.trigger('estop_card')

    refused = loco.dispatch('move', {'vx': 0.2, '_tool_name': 'loco'})
    assert refused['state'] == 'error'
    assert refused['code'] == 'ESTOP_ACTIVE'
    assert refused['requested'] == 'loco.move'

    posture = loco.dispatch('stand_up', {'_tool_name': 'stand_up_lie_prone'})
    assert posture['code'] == 'ESTOP_ACTIVE'

    stopped = loco.dispatch('stop_move', {'_tool_name': 'loco'})
    assert stopped == {'state': 'stopped'}


def test_card_trigger_reset_and_confirm(estop):
    latch = estop.EmergencyStopLatch()
    card = estop.EstopPlugin({}, 'robot', None, latch)

    assert card.get_tool()['name'] == 'estop'
    assert card.dispatch('info', {})['estop']['latched'] is False

    latched = card.dispatch('trigger', {'reason': 'operator'})
    assert latched['state'] == 'latched'
    assert latched['newly_latched'] is True
    assert latch.latched is True

    refused = card.dispatch('reset', {})
    assert refused['code'] == 'CONFIRM_REQUIRED'
    assert latch.latched is True

    released = card.dispatch('reset', {'confirm': True})
    assert released['state'] == 'released'
    assert released['released'] is True
    assert latch.latched is False


def test_stopping_the_card_does_not_release_the_latch(estop):
    latch = estop.EmergencyStopLatch()
    card = estop.EstopPlugin({}, 'robot', None, latch)

    card.dispatch('trigger', {})
    assert card.dispatch('stop', {}) == {'state': 'idle'}
    assert latch.latched is True


class _BlockingHighCtrl(_FakeHighCtrl):
    """Parks one non-zero publish inside the SDK call, like a real write."""

    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def publish_cmd(self, x, y, z, action_cmd, index=0):
        if x or y or z:
            self.entered.set()
            if not self.release.wait(2.0):
                raise AssertionError('test never released the in-flight publish')
        self.commands.append((x, y, z))


def test_latch_waits_for_the_in_flight_publish(device, estop):
    """The check and the SDK write are one critical section.

    A publisher that had already passed `refuses()` must finish before the
    latch transition is visible; otherwise one non-zero command is sent after
    the E-stop. Here the publish is parked inside the SDK call and `trigger`
    must not complete until it is released.
    """
    latch = estop.EmergencyStopLatch()
    high = _BlockingHighCtrl()
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)

    publisher = threading.Thread(
        target=lambda: loco._publish_cmd(0.2, 0.0, 0.0, 'DEFAULT'))
    publisher.start()
    assert high.entered.wait(2.0), 'publisher never reached the SDK call'

    trigger = threading.Thread(target=lambda: latch.trigger('estop_card', 'race'))
    trigger.start()
    time.sleep(0.05)
    assert trigger.is_alive(), 'trigger overtook an in-flight publish'

    high.release.set()
    publisher.join(2.0)
    trigger.join(2.0)
    assert not publisher.is_alive()
    assert not trigger.is_alive()

    # The pre-latch command completed, then the latch's own zero; nothing after.
    assert high.commands == [(0.2, 0.0, 0.0), (0.0, 0.0, 0.0)]
    assert latch.latched is True
    assert loco._publish_cmd(0.2, 0.0, 0.0, 'DEFAULT') is False


def test_actuator_start_reports_ready(estop):
    latch = estop.EmergencyStopLatch()
    card = estop.EstopPlugin({}, 'robot', None, latch)

    assert card.get_tool()['type'] == 'actuator'
    assert card.dispatch('start', {}) == {'state': 'ready'}


def test_servo_only_configuration_pauses_and_zeroes_on_latch(monkeypatch, estop):
    """Without `loco` there is no other listener: dropping commands is not enough."""
    repo_root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(repo_root))
    monkeypatch.setitem(sys.modules, 'highcontrol_py', types.SimpleNamespace(
        ControlCmd=types.SimpleNamespace(DEFAULT='DEFAULT')))
    sys.modules.pop('loco_servo', None)
    from noetix.bumi import loco_servo

    high = _FakeHighCtrl()
    latch = estop.EmergencyStopLatch()
    servo = loco_servo.LocoServoPlugin(
        {'dry_run': False}, 'robot', None, high,
        loco_plugin=None, estop=latch)
    servo._running = True

    latch.trigger('estop_card', 'test')

    assert servo._paused is True
    assert high.commands[-1] == (0.0, 0.0, 0.0)
    assert servo._halt(False)['code'] == 'ESTOP_ACTIVE'


def test_bundle_wires_one_latch_into_loco_and_the_card(monkeypatch, device, estop):
    """The card alone is not the safety property: `loco` must share its latch."""
    monkeypatch.setitem(sys.modules, 'device', device)
    monkeypatch.setitem(sys.modules, 'estop', estop)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', types.ModuleType('rclpy.executors'))
    path = Path(__file__).resolve().parents[1] / 'main.py'
    spec = importlib.util.spec_from_file_location('bumi_estop_bundle_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    high = _FakeHighCtrl()
    bundle = module.BumiDeviceBundle(
        {'plugins': {'loco': {'enabled': True}, 'estop': {'enabled': True}}},
        'robot', types.SimpleNamespace(add_node=lambda node: None), high, None)

    names = {tool['name'] for tool in bundle.get_all_tools()}
    assert {'loco', 'estop'} <= names

    latched = bundle.dispatch('estop', {'action': 'trigger'})
    assert latched['state'] == 'latched'
    assert high.commands[-1] == (0.0, 0.0, 0.0)

    refused = bundle.dispatch('loco', {'action': 'move', 'vx': 0.2})
    assert refused['code'] == 'ESTOP_ACTIVE'

    bundle.dispatch('estop', {'action': 'reset', 'confirm': True})
    assert bundle.dispatch('loco', {'action': 'stop_move'}) == {'state': 'stopped'}
