"""Contract tests for the Bumi software emergency stop.

The latch has to be visible to every motion path, not just the card, so the
tests drive `LocoPlugin` directly: a rule that only exists in the card's
`dispatch` would let `loco_servo` keep streaming.
"""

import importlib.util
import os
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


class _SemanticHighCtrl(_FakeHighCtrl):
    """WALK leaves the semantic mode, the way the vendor controller does."""

    def publish_cmd(self, x, y, z, action_cmd, index=0):
        super().publish_cmd(x, y, z, action_cmd, index)
        if action_cmd == 'WALK':
            self.mode = 2


@pytest.fixture
def device(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, 'highcontrol_py', types.SimpleNamespace(
        ControlCmd=types.SimpleNamespace(DEFAULT='DEFAULT', WALK='WALK')))
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


@pytest.mark.parametrize('record', [
    '{}',
    'null',
    '[]',
    '{"latched": 0}',
    '{"latched": true}',                                      # missing metadata
    '{"latched": true, "source": 1, "reason": "", "since_ms": null}',
    '{"latched": false, "source": "", "reason": "", "since_ms": null}',
])
def test_schema_invalid_state_fails_closed(estop, tmp_path, record):
    """A file that exists must never be read as permission to move."""
    path = tmp_path / 'estop_state.json'
    path.write_text(record)
    latch = estop.EmergencyStopLatch(str(path))
    assert latch.latched is True
    assert latch.snapshot()['source'] == 'estop_state_invalid'


def test_failed_clear_keeps_the_next_start_latched(estop, tmp_path, monkeypatch):
    path = tmp_path / 'estop_state.json'
    latch = estop.EmergencyStopLatch(str(path))
    latch.trigger('estop_card')

    def refuse_unlink(self, missing_ok=False):
        raise OSError('read-only filesystem')

    monkeypatch.setattr(estop.Path, 'unlink', refuse_unlink)
    # A release the next restart would undo is not a release.
    assert latch.reset() is False
    assert latch.latched is True
    monkeypatch.undo()

    # The old record is still there, so a restart must not come back unlatched.
    assert estop.EmergencyStopLatch(str(path)).latched is True


def test_trigger_reports_when_the_marker_cannot_be_written(estop, tmp_path, monkeypatch):
    latch = estop.EmergencyStopLatch(str(tmp_path / 'estop_state.json'))

    def fail_mkdir(self, parents=False, exist_ok=False):
        raise OSError('read-only filesystem')

    monkeypatch.setattr(estop.Path, 'mkdir', fail_mkdir)
    assert latch.trigger('estop_card') is True
    monkeypatch.undo()

    assert latch.latched is True
    assert latch.durable is False


def test_startup_with_unwritable_state_starts_latched(estop, tmp_path, monkeypatch):
    def fail_mkdir(self, parents=False, exist_ok=False):
        raise OSError('read-only filesystem')

    monkeypatch.setattr(estop.Path, 'mkdir', fail_mkdir)
    latch = estop.EmergencyStopLatch(str(tmp_path / 'estop_state.json'))
    monkeypatch.undo()

    assert latch.latched is True
    assert latch.snapshot()['source'] == 'estop_state_unwritable'


def test_card_reset_failure_keeps_the_latch_and_reports_it(estop, tmp_path, monkeypatch):
    latch = estop.EmergencyStopLatch(str(tmp_path / 'estop_state.json'))
    card = estop.EstopPlugin({}, 'robot', None, latch)
    card.dispatch('trigger', {})

    def refuse_unlink(self, missing_ok=False):
        raise OSError('read-only filesystem')

    monkeypatch.setattr(estop.Path, 'unlink', refuse_unlink)
    result = card.dispatch('reset', {'confirm': True})
    monkeypatch.undo()

    assert result['code'] == 'ESTOP_CLEAR_FAILED'
    assert latch.latched is True
    assert result['estop']['latched'] is True


def test_failed_write_still_restores_latched(estop, tmp_path, monkeypatch):
    path = tmp_path / 'estop_state.json'
    latch = estop.EmergencyStopLatch(str(path))

    def fail_fsync(fd):
        raise OSError('disk full')

    monkeypatch.setattr(estop.os, 'fsync', fail_fsync)
    latch.trigger('estop_card')
    monkeypatch.undo()

    # The file was created before the failed write; existence alone means latched.
    assert path.exists()
    assert estop.EmergencyStopLatch(str(path)).latched is True


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

    tool = card.get_tool()
    assert tool['name'] == 'estop'
    assert tool['inputSchema']['x-action-params']['reset']['x-is-dangerous'] is True
    assert 'x-is-dangerous' not in tool['inputSchema']['x-action-params']['trigger']
    assert card.dispatch('info', {})['estop']['latched'] is False

    latched = card.dispatch('trigger', {'reason': 'operator'})
    assert latched['state'] == 'latched'
    assert latched['newly_latched'] is True
    assert latched['estop']['durable'] is True
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


def test_estop_exits_a_running_semantic_action(device, estop):
    """A dance/wave must not keep running behind a latched E-stop."""
    latch = estop.EmergencyStopLatch()
    high = _SemanticHighCtrl(mode=5)
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)

    latch.trigger('estop_card')

    assert 'WALK' in high.action_commands
    assert high.mode == 2
    assert latch.latched is True


def test_estop_exits_teaching_playback(device, estop):
    latch = estop.EmergencyStopLatch()
    high = _SemanticHighCtrl(mode=23)
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)

    latch.trigger('estop_card')

    assert 'WALK' in high.action_commands
    assert high.mode == 2


def test_estop_does_not_abort_stand_up(device, estop):
    """There is no documented abort for the get-up trajectory; do not invent one."""
    latch = estop.EmergencyStopLatch()
    high = _FakeHighCtrl(mode=27)
    loco = device.LocoPlugin({}, 'robot', None, high)
    loco.attach_estop(latch)

    latch.trigger('estop_card')

    assert 'WALK' not in high.action_commands
    assert latch.latched is True


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
        ControlCmd=types.SimpleNamespace(DEFAULT='DEFAULT', WALK='WALK')))
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


def test_trigger_is_not_durable_when_directory_sync_fails(estop, tmp_path, monkeypatch):
    path = tmp_path / 'estop_state.json'
    latch = estop.EmergencyStopLatch(str(path))
    real_open = estop.os.open

    def fail_open(target, flags, *args, **kwargs):
        if os.fspath(target) == str(tmp_path):
            raise OSError('directory cannot be opened for sync')
        return real_open(target, flags, *args, **kwargs)

    monkeypatch.setattr(estop.os, 'open', fail_open)
    assert latch.trigger('estop_card') is True
    monkeypatch.undo()

    assert latch.durable is False
    assert path.exists()


def test_reset_fails_when_directory_sync_cannot_be_proven(estop, tmp_path, monkeypatch):
    path = tmp_path / 'estop_state.json'
    latch = estop.EmergencyStopLatch(str(path))
    latch.trigger('estop_card')
    real_open = estop.os.open

    def fail_open(target, flags, *args, **kwargs):
        if os.fspath(target) == str(tmp_path):
            raise OSError('directory cannot be opened for sync')
        return real_open(target, flags, *args, **kwargs)

    monkeypatch.setattr(estop.os, 'open', fail_open)
    assert latch.reset() is False
    monkeypatch.undo()

    assert latch.latched is True
    assert path.exists()


class _ModeHighCtrl:
    def __init__(self, mode):
        self.mode = mode

    def get_mode(self):
        return self.mode


def test_auto_trigger_on_protection_latches(estop):
    latch = estop.EmergencyStopLatch()
    card = estop.EstopPlugin(
        {'auto_trigger_on_protection': True}, 'robot', None, latch,
        _ModeHighCtrl(26))
    card._last_protection_check = 0.0
    card._check_protection_mode()

    assert latch.latched is True
    assert latch.snapshot()['source'] == 'protection_mode'


def test_walking_mode_does_not_auto_latch(estop):
    latch = estop.EmergencyStopLatch()
    card = estop.EstopPlugin(
        {'auto_trigger_on_protection': True}, 'robot', None, latch,
        _ModeHighCtrl(2))
    card._last_protection_check = 0.0
    card._check_protection_mode()

    assert latch.latched is False


def test_bundle_wires_one_latch_into_loco_and_the_card(monkeypatch, tmp_path, device, estop):
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
        {'plugins': {'loco': {'enabled': True},
                     'estop': {'enabled': True,
                               'state_path': str(tmp_path / 'estop_state.json')}}},
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
