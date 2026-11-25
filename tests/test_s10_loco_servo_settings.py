"""S10 servo configuration and motion transitions, using a fake native backend."""
import importlib.util
import json
import time
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def servo(monkeypatch):
    root = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro'
    monkeypatch.syspath_prepend(str(root))
    spec = importlib.util.spec_from_file_location('s10_servo_settings', root / 'loco_servo.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    class Native:
        def __init__(self):
            self.calls = []
            self.transport_checks = 0
        def ensure_plaintext_transport(self):
            self.transport_checks += 1
        def send(self, *args):
            self.calls.append(args)
            return {}
    class Subscriber:
        _process = SimpleNamespace(is_alive=lambda: True)
        def __init__(self, *args): pass
        def latest(self): return None
        def stop(self): pass
    monkeypatch.setattr(module, 'CoreSubscriber', Subscriber)
    monkeypatch.setenv('S10_MOTION_ENABLED', '1')
    native = Native()
    plugin = module.S10LocoServoPlugin(native, ros2=object())
    yield SimpleNamespace(plugin=plugin, native=native, module=module)
    plugin.stop()


def command(plugin, values, seq=1, stamp=None):
    return {'schema': 'motus.control/1', 'mode': 'twist', 'dof': 6,
            'source': 'test-policy', 'seq': seq, 'priority': 1,
            'stamp_ms': getattr(plugin, '_barrier_ms', int(time.time() * 1000)) + 1 if stamp is None else stamp,
            'ttl_ms': 500, 'values': list(values)}


def submit(plugin, payload):
    plugin._on_message(SimpleNamespace(data=json.dumps(payload)))


def test_default_start_and_restart_preserve_dry_run_without_native_calls(servo, monkeypatch):
    monkeypatch.setenv('S10_MOTION_ENABLED', '0')
    for _ in range(2):
        assert servo.plugin.dispatch('start', {'input_topic': '/policy/cmd'})['state'] == 'ready'
        assert servo.plugin.dispatch('info', {})['dry_run'] is True
        servo.plugin.stop()
    assert servo.native.calls == []
    assert servo.native.transport_checks == 0


@pytest.mark.parametrize('confirm', [False, 'true', 1])
@pytest.mark.parametrize('entry', ['start', 'config', 'streaming-config', 'saved-live-start'])
def test_explicit_cancel_or_invalid_confirmation_never_selects_live_control(servo, confirm, entry):
    p = servo.plugin
    if entry == 'saved-live-start':
        p = servo.module.S10LocoServoPlugin(servo.native, ros2=object(), config={'dry_run': False})
    if entry == 'streaming-config':
        assert p.dispatch('start', {'input_topic': '/policy/cmd'})['state'] == 'ready'
    before = p.dispatch('info', {})
    args = {'confirm': confirm}
    try:
        if entry in {'start', 'saved-live-start'}:
            args['input_topic'] = '/policy/cmd'
            if entry == 'start':
                args['dry_run'] = False
            result = p.dispatch('start', args)
        else:
            result = p.dispatch('config', {**args, 'config': {'dry_run': False}})
        assert result.get('code') == 'CONFIRM_REQUIRED'
        after = p.dispatch('info', {})
        assert after['state'] == before['state']
        assert after['dry_run'] == before['dry_run']
        assert servo.native.calls == []
        assert servo.native.transport_checks == 0
    finally:
        p.stop()


def test_speed_presets_change_forward_limit_without_scaling_yaw_or_raising_floor(servo):
    for name, limit in [('slow', .3), ('normal', .6), ('fast', 1.)]:
        assert servo.plugin.dispatch('config', {'speed_preset': name})['ok']
        info = servo.plugin.dispatch('info', {})
        assert info['speed_preset'] == name and info['vx_limit'] == limit
        assert info['control_interface']['limits']['upper'][0] == limit
        assert info['control_interface']['limits']['upper'][5] == 1.0
        assert info['min_vx'] == 0


def test_servo_reports_static_standing_footprint_as_an_estimate(servo):
    descriptor = servo.plugin.dispatch('info', {})['control_interface']
    assert descriptor['frame'] == 'base_link'
    assert descriptor['footprint'] == {'half_width': 0.20, 'half_length': 0.23, 'source': 'estimate'}


def test_servo_watchdog_fires_at_1000_ms_without_admitting_stale_commands(servo, monkeypatch):
    p = servo.plugin
    monkeypatch.setattr(p, '_read_loop', lambda: None)
    started = p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False})
    assert started['state'] == 'ready'
    stamp = p._barrier_ms + 1
    elapsed_ms = [0]
    monkeypatch.setattr(p._sink, '_clock', lambda: 1000 + elapsed_ms[0])
    monkeypatch.setattr(p._sink, '_wall_clock', lambda: stamp + elapsed_ms[0])

    values = [.2, 0, 0, 0, 0, 0]
    submit(p, {**command(p, values, stamp=stamp), 'ttl_ms': 600})
    assert servo.native.calls[-1][2]['X'] == pytest.approx(.03)
    commands_sent = len(servo.native.calls)

    elapsed_ms[0] = 301
    submit(p, {**command(p, values, seq=2, stamp=stamp), 'ttl_ms': 300, 'obs_stamp_ms': stamp})
    assert p.dispatch('info', {})['last_command']['reason'] == 'expired: 301 ms old, ttl 300 ms'

    elapsed_ms[0] = 500
    p._tick()
    assert len(servo.native.calls) == commands_sent

    elapsed_ms[0] = 501
    submit(p, {**command(p, values, seq=3, stamp=stamp + 501), 'obs_stamp_ms': stamp - 500})
    assert p.dispatch('info', {})['last_command']['reason'] == 'observation 1001 ms old, limit 1000 ms'

    elapsed_ms[0] = 599
    p._tick()
    assert len(servo.native.calls) == commands_sent

    elapsed_ms[0] = 600
    p._tick()
    assert len(servo.native.calls) == commands_sent + 1
    assert servo.native.calls[-1] == (0x00100001, 0x00110002, dict.fromkeys(
        ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'), 0.0))
    assert p._sink.stats()['watchdog_strikes'] == 0

    elapsed_ms[0] = 999
    p._tick()
    assert len(servo.native.calls) == commands_sent + 1
    assert p._sink.stats()['watchdog_strikes'] == 0

    elapsed_ms[0] = 1000
    p._tick()
    assert len(servo.native.calls) == commands_sent + 2
    assert servo.native.calls[-1] == servo.native.calls[-2]
    assert p._sink.stats()['watchdog_strikes'] == 1
    rate = started['control_interface']['rate']
    assert rate['watchdog_ms'] == 1000
    assert rate['max_command_ttl_ms'] == 1000
    assert rate['max_obs_age_ms'] == 1000
    assert p.dispatch('info', {})['control_interface']['rate'] == rate


@pytest.mark.parametrize(('obs_age_ms', 'verdict'), [
    (800, 'APPLIED'), (999, 'APPLIED'), (1000, 'DROPPED'), (1001, 'DROPPED'),
])
def test_s10_admits_delayed_observations_only_with_time_left(servo, monkeypatch, obs_age_ms, verdict):
    p = servo.plugin
    monkeypatch.setattr(p, '_read_loop', lambda: None)
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False})['state'] == 'ready'
    stamp = p._barrier_ms + 1
    monkeypatch.setattr(p._sink, '_wall_clock', lambda: stamp)
    sent = len(servo.native.calls)

    submit(p, {**command(p, [.02, 0, 0, 0, 0, 0], stamp=stamp),
               'obs_stamp_ms': stamp - obs_age_ms})

    assert p.dispatch('info', {})['last_command']['verdict'] == verdict
    assert len(servo.native.calls) == sent + (verdict == 'APPLIED')


@pytest.mark.parametrize(('age_ms', 'verdict'), [(999, 'APPLIED'), (1000, 'DROPPED'), (1001, 'DROPPED')])
def test_servo_caps_command_ttl_at_1000_ms(servo, monkeypatch, age_ms, verdict):
    p = servo.plugin
    monkeypatch.setattr(p, '_read_loop', lambda: None)
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False})['state'] == 'ready'
    stamp = p._barrier_ms + 1
    monkeypatch.setattr(p._sink, '_wall_clock', lambda: stamp + age_ms)

    submit(p, {**command(p, [.02, 0, 0, 0, 0, 0], stamp=stamp), 'ttl_ms': 1500})

    last = p.dispatch('info', {})['last_command']
    assert last['verdict'] == verdict
    if verdict == 'DROPPED':
        assert 'expired' in last['reason']


@pytest.mark.parametrize(('ttl_ms', 'obs_age_ms', 'remaining_ms'), [
    (250, None, 180),  # 70 ms already elapsed in transit
    (1500, None, 930),  # descriptor caps TTL before subtracting transit
    (1000, 600, 400),  # observation expires before the command
])
def test_s10_sends_zero_at_active_command_expiry_before_watchdog(
        servo, monkeypatch, ttl_ms, obs_age_ms, remaining_ms):
    p = servo.plugin
    monkeypatch.setattr(p, '_read_loop', lambda: None)
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False})['state'] == 'ready'
    stamp = p._barrier_ms + 1
    elapsed = [0]
    monkeypatch.setattr(p._sink, '_clock', lambda: 1000 + elapsed[0])
    monkeypatch.setattr(p._sink, '_wall_clock', lambda: stamp + 70)
    payload = {**command(p, [.2, 0, 0, 0, 0, 0], stamp=stamp), 'ttl_ms': ttl_ms}
    if obs_age_ms is not None:
        payload['obs_stamp_ms'] = stamp + 70 - obs_age_ms

    submit(p, payload)
    assert servo.native.calls[-1][2]['X'] > 0.
    sent = len(servo.native.calls)
    elapsed[0] = remaining_ms - 1
    p._tick()
    assert len(servo.native.calls) == sent

    elapsed[0] += 1
    p._tick()
    assert len(servo.native.calls) == sent + 1
    assert servo.native.calls[-1][2] == dict.fromkeys(('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'), 0.)
    assert p._sink.stats()['watchdog_strikes'] == 0
    p._tick()
    assert len(servo.native.calls) == sent + 1


@pytest.mark.parametrize('settings', [
    {'speed_preset': 'turbo'}, {'linear_acceleration': 0},
    {'linear_acceleration': float('nan')}, {'angular_acceleration': True},
    {'wz_limit': 1.01}, {'wz_limit': 0}, {'min_vx': .61},
    {'rotate_only': 'false'}, {'unexpected': 1},
])
def test_bad_settings_are_atomic_and_issue_no_commands(servo, settings):
    before = servo.plugin.dispatch('info', {})
    result = servo.plugin.dispatch('config', settings)
    assert result['ok'] is False
    after = servo.plugin.dispatch('info', {})
    assert after['control_interface'] == before['control_interface']
    assert after['dry_run'] == before['dry_run']
    assert servo.native.calls == []


def test_config_schema_exposes_controls_without_start_defaults(servo):
    tool = servo.plugin.get_tool()
    props = tool['configSchema']['properties']
    for action in ['start', 'config']:
        assert 'confirm' in tool['inputSchema']['x-action-params'][action]['params']
    for key in ['speed_preset', 'dry_run', 'rotate_only', 'min_vx',
                'linear_acceleration', 'angular_acceleration', 'wz_limit']:
        assert key in props
        assert 'default' not in tool['inputSchema']['properties'][key]


def test_resume_has_no_canvas_parameters(servo):
    actions = servo.plugin.get_tool()['inputSchema']['x-action-params']
    assert actions['resume']['params'] == []


def test_canvas_resume_reuses_paused_stream_and_saved_settings(servo):
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False,
                                'speed_preset': 'slow'})['state'] == 'ready'
    assert p.dispatch('pause', {})['state'] == 'paused'
    result = p.dispatch('resume', {'input_topic': '/policy/cmd',
                                   'speed_preset': 'fast', 'dry_run': True})
    assert result['state'] == 'running'
    info = p.dispatch('info', {})
    assert info['input'] == '/policy/cmd'
    assert info['speed_preset'] == 'slow'
    assert info['dry_run'] is False


def test_dry_run_counts_valid_commands_without_any_native_setup_or_velocity(servo):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd'})
    submit(p, command(p, [.2, .1, 0, 0, 0, .8]))
    info = p.dispatch('info', {})
    assert info['dry_run_commands'] == 1
    assert servo.native.calls == []


@pytest.mark.parametrize('confirm', [None, True])
def test_selecting_live_mode_in_a_dry_run_stream_waits_for_explicit_resume(servo, confirm):
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd'})['state'] == 'ready'
    args = {'dry_run': False}
    if confirm is not None:
        args['confirm'] = confirm
    assert p.dispatch('config', args)['ok']
    info = p.dispatch('info', {})
    assert info['state'] == 'paused' and info['dry_run'] is False
    submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
    assert servo.native.calls == []
    assert servo.native.transport_checks == 0
    assert p.dispatch('resume', {})['state'] == 'running'
    assert not any(servo.native.calls[0][2].values())
    assert [call[2] for call in servo.native.calls[1:]] == [{'Mode': 1}, {'GaitParam': 0x3002}]
    submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
    assert servo.native.calls[-1][2]['X'] == pytest.approx(.03)
    assert p.dispatch('info', {})['dry_run'] is False


@pytest.mark.parametrize('nested', [False, True])
def test_canvas_saved_no_then_start_uses_live_mode_without_hidden_confirm(servo, nested):
    p = servo.plugin
    # Core restores instance config before start, without a confirm argument.
    saved = {'dry_run': False, 'rotate_only': False, 'speed_preset': 'slow',
             'min_vx': 0, 'linear_acceleration': .3, 'angular_acceleration': 2,
             'wz_limit': 1}
    body = {'config': saved} if nested else saved
    result = p.dispatch('config', {**body, 'instance_id': 'card-test'})
    assert result['ok'] and result['dry_run'] is False
    assert servo.native.calls == []
    assert p.dispatch('info', {})['state'] == 'idle'
    for _ in range(2):
        assert p.dispatch('start', {'input_topic': '/policy/cmd'})['state'] == 'ready'
        assert p.dispatch('info', {})['vx_limit'] == .3
        assert p.dispatch('info', {})['dry_run'] is False
        submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
        assert servo.native.calls[-1][2]['X'] == pytest.approx(.03)
        p.stop()


def test_static_live_default_or_instance_id_alone_does_not_confirm_motion(servo):
    p = servo.module.S10LocoServoPlugin(servo.native, ros2=object(), config={'dry_run': False})
    try:
        result = p.dispatch('start', {'input_topic': '/policy/cmd', 'instance_id': 'card-test'})
        assert result['code'] == 'CONFIRM_REQUIRED'
        assert servo.native.calls == []
    finally:
        p.stop()


def test_explicit_live_start_uses_the_same_selection_as_canvas_no(servo):
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False})['state'] == 'ready'
    assert [call[2] for call in servo.native.calls] == [{'Mode': 1}, {'GaitParam': 0x3002}]
    submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
    assert servo.native.calls[-1][2]['X'] == pytest.approx(.03)
    assert p.dispatch('info', {})['dry_run'] is False


@pytest.mark.parametrize('args,permission,code', [
    ({'confirm': False}, '1', 'CONFIRM_REQUIRED'),
    ({}, '0', 'MOTION_DISABLED'), ({}, 'true', 'INVALID_MOTION_PERMISSION'),
])
def test_pending_live_resume_rechecks_selection_and_deployment_permission(
        servo, monkeypatch, args, permission, code):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd'})
    assert p.dispatch('config', {'dry_run': False})['ok']
    monkeypatch.setenv('S10_MOTION_ENABLED', permission)
    result = p.dispatch('resume', args)
    assert result['code'] == code and result['state'] == 'error'
    assert p.dispatch('info', {})['state'] == 'paused'
    before = len(servo.native.calls)
    submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
    assert len(servo.native.calls) == before
    assert all(call[1] == 0x00110002 and not any(call[2].values())
               for call in servo.native.calls)
    assert servo.native.transport_checks == 0


def test_failed_live_resume_stays_paused_and_does_not_admit_velocity(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd'})
    assert p.dispatch('config', {'dry_run': False})['ok']
    send = servo.native.send
    monkeypatch.setattr(servo.native, 'send', lambda *args: {'ErrorCode': 7})
    try:
        assert p.dispatch('resume', {})['state'] == 'error'
        assert p.dispatch('info', {})['state'] == 'paused'
        submit(p, command(p, [.1, 0, 0, 0, 0, 0]))
        assert p.dispatch('info', {})['applied'] == 0
    finally:
        monkeypatch.setattr(servo.native, 'send', send)


def test_resume_requires_acknowledged_zero_before_native_mode_setup(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd'})
    assert p.dispatch('config', {'dry_run': False})['ok']
    send, attempts = servo.native.send, []
    def rejected(*args):
        attempts.append(args)
        return {'ErrorCode': 7}
    monkeypatch.setattr(servo.native, 'send', rejected)
    try:
        assert p.dispatch('resume', {})['state'] == 'error'
        info = p.dispatch('info', {})
        assert info['state'] == 'paused' and info['stop_pending']
        assert servo.native.transport_checks == 0
        assert attempts and all(call[1] == 0x00110002 and not any(call[2].values())
                                for call in attempts)
    finally:
        monkeypatch.setattr(servo.native, 'send', send)


@pytest.mark.parametrize('preset,limit', [('slow', .3), ('normal', .6), ('fast', 1.)])
@pytest.mark.parametrize('direction', [-1, 1])
def test_s10_speed_tiers_enforce_limits_without_amplifying_small_commands(servo, preset, limit, direction):
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False,
                               'speed_preset': preset, 'linear_acceleration': 20})['state'] == 'ready'
    submit(p, command(p, [direction * .1, 0, 0, 0, 0, 0]))
    assert servo.native.calls[-1][2]['X'] == pytest.approx(direction * .1)
    submit(p, command(p, [direction * limit, 0, 0, 0, 0, 0], seq=2))
    assert servo.native.calls[-1][2]['X'] == pytest.approx(direction * limit)
    before = len(servo.native.calls)
    submit(p, command(p, [direction * (limit + .01), 0, 0, 0, 0, 0], seq=3))
    assert p.dispatch('info', {})['last_command']['verdict'] == 'REJECTED'
    assert len(servo.native.calls) == before


@pytest.mark.parametrize('source,priority,seq,reason', [
    ('lower-priority', 1, 1, 'outranked'),
    ('controller', 50, 9, 'seq'),
    ('controller', 50, 10, 'seq'),
])
def test_live_config_preserves_source_arbitration_and_sequence_history(
        servo, monkeypatch, source, priority, seq, reason):
    monkeypatch.setattr(servo.module.time, 'monotonic', lambda: 100.)
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False,
                                'confirm': True})['state'] == 'ready'
    first = {**command(p, [.1, 0, 0, 0, 0, 0], seq=10),
             'source': 'controller', 'priority': 50}
    submit(p, first)
    counters = p._sink.stats()['counters']
    assert p.dispatch('config', {'speed_preset': 'slow', 'confirm': True})['ok']
    configured_counters = p._sink.stats()['counters']
    assert all(value == 0 for value in servo.native.calls[-1][2].values())
    count = len(servo.native.calls)
    submit(p, {**command(p, [.1, 0, 0, 0, 0, 0], seq=seq),
               'source': source, 'priority': priority})
    last = p.dispatch('info', {})['last_command']
    assert last['verdict'] == 'DROPPED' and reason in last['reason']
    assert configured_counters == counters
    assert len(servo.native.calls) == count
    submit(p, {**command(p, [.1, 0, 0, 0, 0, 0], seq=11),
               'source': 'controller', 'priority': 50})
    assert servo.native.calls[-1][2]['X'] == pytest.approx(.03)


def test_acceleration_begins_at_zero_and_zero_velocity_stops_immediately(servo):
    p = servo.plugin
    assert p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})['state'] == 'ready'
    servo.native.calls.clear()
    submit(p, command(p, [.3, .2, 0, 0, 0, .9]))
    values = servo.native.calls[-1][2]
    assert values['X'] == pytest.approx(.03)
    assert values['Y'] == pytest.approx(.03)
    assert values['Yaw'] == pytest.approx(.2)
    submit(p, command(p, [0] * 6, seq=2))
    assert all(value == 0 for value in servo.native.calls[-1][2].values())


def test_live_to_dry_run_stops_then_discards_preconfiguration_commands(servo):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    previous = command(p, [.2, 0, 0, 0, 0, .2])
    submit(p, previous)
    result = p.dispatch('config', {'config': {'dry_run': True}})
    assert result['ok'] and result['dry_run']
    assert all(value == 0 for value in servo.native.calls[-1][2].values())
    count = len(servo.native.calls)
    submit(p, command(p, [.2, 0, 0, 0, 0, .2], seq=2, stamp=p._barrier_ms))
    assert p.dispatch('info', {})['last_command']['verdict'] == 'DROPPED'
    submit(p, command(p, [.2, 0, 0, 0, 0, .2], seq=3))
    assert len(servo.native.calls) == count
    assert p.dispatch('info', {})['dry_run_commands'] == 1


@pytest.mark.parametrize('yaw', [-.1, .1])
def test_rotate_only_and_yaw_cap_match_the_advertised_limits(servo, yaw):
    p = servo.plugin
    assert p.dispatch('config', {'rotate_only': True, 'wz_limit': .1})['ok']
    limits = p.dispatch('info', {})['control_interface']['limits']
    assert limits['lower'][5] == -.1
    assert limits['upper'][5] == .1
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    submit(p, command(p, [.2, .2, 0, 0, 0, yaw]))
    assert servo.native.calls[-1][2] == {'X': 0., 'Y': 0., 'Z': 0.,
                                        'Roll': 0., 'Pitch': 0., 'Yaw': yaw}
    count = len(servo.native.calls)
    submit(p, command(p, [0, 0, 0, 0, 0, 9 * yaw], seq=2))
    assert p.dispatch('info', {})['last_command']['verdict'] == 'REJECTED'
    assert len(servo.native.calls) == count


def test_revoked_motion_permission_stops_and_disarms_an_active_stream(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    submit(p, command(p, [.2, 0, 0, 0, 0, 0]))
    monkeypatch.setenv('S10_MOTION_ENABLED', '0')
    submit(p, command(p, [.2, 0, 0, 0, 0, 0], seq=2))
    assert all(value == 0 for value in servo.native.calls[-1][2].values())
    assert not p._running and p._paused


def test_enabling_dry_run_does_not_hide_a_failed_native_stop(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    submit(p, command(p, [.2, 0, 0, 0, 0, 0]))
    send = servo.native.send
    monkeypatch.setattr(servo.native, 'send', lambda *args: {'ErrorCode': 7})
    result = p.dispatch('config', {'dry_run': True})
    try:
        assert result['ok'] is False
        assert p.dispatch('info', {})['dry_run'] is False
        assert p._paused
    finally:
        monkeypatch.setattr(servo.native, 'send', send)


def test_failed_stop_survives_disarm_and_blocks_dry_run_until_zero_is_acknowledged(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    submit(p, command(p, [.2, 0, 0, 0, 0, 0]))
    send, attempts = servo.native.send, []
    def reject_stop(*args):
        attempts.append(args)
        return {'ErrorCode': 7}
    monkeypatch.setattr(servo.native, 'send', reject_stop)
    monkeypatch.setenv('S10_MOTION_ENABLED', '0')
    submit(p, command(p, [.2, 0, 0, 0, 0, 0], seq=2))
    try:
        assert p._stop_pending and not p._running
        assert not p.dispatch('config', {'dry_run': True})['ok']
        assert p.dispatch('info', {})['dry_run'] is False
        before = len(attempts)
        p._tick()
        assert len(attempts) > before
        assert p._stop_pending
    finally:
        monkeypatch.setattr(servo.native, 'send', send)
    p._tick()
    assert not p._stop_pending
    assert all(value == 0 for value in servo.native.calls[-1][2].values())


def test_reader_keeps_retrying_zero_after_revocation_until_native_ack(servo, monkeypatch):
    p = servo.plugin
    p.dispatch('start', {'input_topic': '/policy/cmd', 'dry_run': False, 'confirm': True})
    submit(p, command(p, [.2, 0, 0, 0, 0, 0]))
    send, attempts = servo.native.send, []
    retried, allow_ack, acknowledged = threading.Event(), threading.Event(), threading.Event()
    def stop_reply(*args):
        attempts.append(args)
        assert all(value == 0 for value in args[2].values())
        if len(attempts) >= 2:
            retried.set()
        if allow_ack.is_set():
            reply = send(*args)
            acknowledged.set()
            return reply
        return {'ErrorCode': 7}
    monkeypatch.setattr(servo.native, 'send', stop_reply)
    monkeypatch.setenv('S10_MOTION_ENABLED', '0')
    try:
        submit(p, command(p, [.2, 0, 0, 0, 0, 0], seq=2))
        assert retried.wait(1), 'reader exited before retrying the failed stop'
        assert p.dispatch('info', {})['stop_pending']
        allow_ack.set()
        assert acknowledged.wait(1)
        assert not p.dispatch('info', {})['stop_pending']
        assert not p._running and p._paused
    finally:
        monkeypatch.setattr(servo.native, 'send', send)
