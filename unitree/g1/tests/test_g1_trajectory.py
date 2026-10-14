"""Real recorder, executor and adapter; SDK and plant are explicit substitutes."""
import copy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
from test_teleop_servo_execution import setup_servo
from test_arm_stream import packet
from trajectory import Recorder, read_recording
from trajectory_control import TrajectoryControl
from common.motion.envelope import IDENTITY


def recorded(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {'test': True}, list(arm.profile['arm_joint_names']), arm.limits, clock=arm.clock)
    arm.record_sink = recorder
    for seq in range(1, 4):
        advance(50)
        arm.accept_control(packet(arm, seq, target=.03))
        arm.tick()
    advance(20)
    arm.hold('operator_pause')
    advance(100)
    result = recorder.stop()
    assert result['complete'], result
    value = read_recording(recorder.directory, result['filename'], arm.limits)
    return arm, sample, channel, advance, recorder, value


def test_record_only_successful_smoothed_sdk_references_and_pause(tmp_path):
    arm, sample, channel, advance, recorder, value = recorded(tmp_path)
    targets = [f['target_q'] for f in value['frames'] if f['kind'] == 'target']
    assert targets == [q for q, _ in channel.positions]
    assert targets[0][0] != .03
    assert value['frames'][-1]['kind'] == 'hold'
    assert value['duration_ns']-value['frames'][-1]['t_ns'] == 100_000_000
    assert value['sha256'] == hashlib.sha256((recorder.directory/recorder.filename).read_bytes()).hexdigest()
    assert all(f['feedback_q'] == sample['q'] and f['feedback_dq'] == sample['dq'] for f in value['frames'])


@pytest.mark.parametrize('failure', ['sdk', 'queue', 'disk'])
def test_failed_recording_never_complete_or_blocks_motion(tmp_path, monkeypatch, failure):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {}, [], arm.limits, clock=arm.clock)
    arm.record_sink = recorder
    if failure == 'sdk': channel.publish_servo_position = lambda *a, **k: False
    if failure == 'queue':
        import queue
        monkeypatch.setattr(recorder.queue, 'put_nowait', lambda _: (_ for _ in ()).throw(queue.Full()))
    if failure == 'disk': monkeypatch.setattr('trajectory.os.fsync', lambda _: (_ for _ in ()).throw(OSError('disk_full')))
    for seq in (1, 2):
        advance(50)
        if arm.state != 'fault':
            arm.accept_control(packet(arm, seq, target=.03)); arm.tick()
    state = recorder.stop()
    assert not state['complete'] and state['error']
    if failure != 'sdk': assert len(channel.positions) == 2 and arm.state != 'fault'
    else: assert not channel.positions and arm.applied_seq == -1
    with pytest.raises(ValueError): read_recording(recorder.directory, state['filename'], arm.limits)


@pytest.mark.parametrize('change', ['truncated', 'target', 'duration', 'duplicate', 'symlink', 'path'])
def test_corrupt_or_unsafe_file_rejected(tmp_path, change):
    arm, _, _, _, recorder, _ = recorded(tmp_path)
    path = recorder.directory/recorder.filename
    raw = path.read_bytes()
    filename = recorder.filename
    if change == 'truncated': path.write_bytes(raw.rsplit(b'\n', 2)[0]+b'\n')
    if change in ('target', 'duration'):
        lines = raw.splitlines()
        idx, key = (1, 'target_q') if change == 'target' else (-1, 'duration_ns')
        value = json.loads(lines[idx]); value[key] = [.1]*10 if change == 'target' else value[key]+1
        lines[idx] = json.dumps(value).encode(); path.write_bytes(b'\n'.join(lines)+b'\n')
    if change == 'duplicate': path.write_bytes(raw.replace(b'"t_ns":', b'"t_ns":0,"t_ns":', 1))
    if change == 'symlink':
        link = recorder.directory/'link.jsonl'; link.symlink_to(path); filename = link.name
    if change == 'path': filename = '../'+filename
    with pytest.raises((ValueError, OSError)): read_recording(recorder.directory, filename, arm.limits)


def test_exact_replay_reference_bypasses_filter_but_not_execution_gate(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    command = packet(arm, target=.02)
    proof = dict(zip(IDENTITY, arm._proofs[1].identity),
                 schema='motus.motion.envelope/1', lower=[-1.]*10, upper=[1.]*10, measured_ns=arm.clock())
    advance(50); arm.replay_deadline_ns = arm.clock()+100_000_000
    arm.write_replay(command, proof)
    assert channel.positions[-1][0] == [.02]*10
    advance(101); arm.tick()
    assert arm.state == 'hold' and arm.reason == 'replay_watchdog'
    before = len(channel.positions)
    with pytest.raises(ValueError): arm.write_replay(command, proof)
    assert len(channel.positions) == before


def test_replay_rejects_fast_target_and_stale_feedback(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    command = packet(arm, target=.1)
    proof = {'schema': 'motus.motion.envelope/1', **{k: command[k] for k in IDENTITY},
             'lower': [-1.]*10, 'upper': [1.]*10, 'measured_ns': arm.clock()}
    advance(10); arm.replay_deadline_ns = arm.clock()+200_000_000
    with pytest.raises(ValueError, match='speed'): arm.write_replay(command, proof)
    command = packet(arm, seq=2, target=.1)
    proof.update({k: command[k] for k in IDENTITY})
    advance(101, fresh=False)
    with pytest.raises(ValueError, match='feedback'): arm.write_replay(command, proof)
    assert not channel.positions


def api_rig(tmp_path):
    import threading
    from motion_control import MotionControl
    from teleop_control import TeleopControl
    from test_teleop_settings import input_card
    arm, sample, channel, _, _, _ = setup_servo(tmp_path)
    arm.clock = time.monotonic_ns
    arm.last_emit = arm._takeover_ns = arm.clock()-1_000_000_000
    channel.open = lambda: None
    sample['locked_joints'] = copy.deepcopy(arm.profile['locked_joints'])
    def snapshot():
        sample['arm_ns'] = arm.clock()
        if channel.positions: sample['q'] = list(channel.positions[-1][0])
        return copy.deepcopy(sample)
    arm._snapshot_provider = snapshot
    motion = MotionControl({}, arm)
    arm.motion_control = motion
    motion.start = lambda: None  # No DDS or SDK setup in this synthetic plant.
    motion._live_session = arm.session_id
    motion.solver = SimpleNamespace(profile=arm.profile, profile_sha256=arm.profile_sha256,
        joint_envelope=lambda measured, previous, target: {'lower': [-1.]*10, 'upper': [1.]*10})
    receipts = []
    card = TeleopControl({'mode': 'live', 'data_dir': str(tmp_path), 'trajectory_device_id': 'synthetic-g1'},
                        motion, notify=lambda *args: receipts.append(args))
    _, frame = input_card(tmp_path)
    card.clock_id = 'offline'; card._operator = 'test-session'; card._state = 'active'
    card.binding = {'command_topic': '/teleop/command', 'feedback_topic': '/teleop/state'}
    card.mapping.calibrate(frame, [[0, 0, 0, 0, 0, 0, 1]]*2)
    frame['received_monotonic_ns'] = card.clock(); card.receive(frame)
    running = threading.Event()
    def tick():
        while not running.wait(.002): arm.tick()
    ticker = threading.Thread(target=tick); ticker.start()
    return card, arm, sample, channel, receipts, running, ticker


def test_adapter_record_inspect_preview_replay_real_termination_and_acp(tmp_path):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        assert card.dispatch('record_start', {})['recording']
        for seq in (1, 2, 3):
            arm.accept_control(packet(arm, seq, target=.01*seq))
            time.sleep(.06)
        recorded = card.dispatch('record_stop', {})
        assert recorded['complete'], recorded
        name = recorded['filename']
        asset = card.dispatch('inspect', {'filename': name})
        assert asset['complete'] and asset['frames'] == 3
        assert card.dispatch('play', {'filename': name})['code'] == 'stop_teleop_before_replay'
        assert card.stop()['authority_released']
        # The operator, represented by this plant substitute, restores the start.
        sample['q'] = list(asset['preview'][0]['target_q']); channel.positions.clear()
        sample['q'] = [q+.1 for q in sample['q']]
        assert card.dispatch('play', {'filename': name})['code'] == 'recording_start_mismatch'
        sample['q'] = list(asset['preview'][0]['target_q'])
        preview = card.dispatch('play', {'filename': name})
        assert preview['execute'] is False and not arm.session_id
        count = len(channel.positions)
        result = card.dispatch('play', {'filename': name, 'execute': True, 'preview_id': preview['preview_id'],
                                        'workspace_clear': True, 'take_control': True})
        assert result['status'] == 'running', result
        assert card.dispatch('start', {'input_topic': '/teleop/command'})['code'] == 'replay_active'
        card.trajectory.thread.join(6)
        outcome = card.dispatch('action_status', {'action_id': result['action_id']})
        assert outcome['status'] == 'completed', outcome
        assert outcome['result']['stop']['authority_released'] and not arm.session_id
        assert receipts == [(result['action_id'], outcome['status'], outcome['result'])]
        assert [q for q, _ in channel.positions[count:count+3]] == [f['target_q'] for f in asset['preview']]
        assert not card.trajectory.recorder.status()['error']
        arm.cfg['live_enabled'] = False
        rejected = card.dispatch('play', {'filename': name, 'execute': True, 'preview_id': preview['preview_id'],
                                         'workspace_clear': True, 'take_control': True})
        assert rejected['state'] == 'error'  # A consumed confirmation cannot run again.
        card.cfg['trajectory_device_id'] = 'different-robot'
        assert card.dispatch('inspect', {'filename': name})['code'] == 'recording_binding_mismatch'
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


def test_replay_stop_is_not_blocked_by_file_verification(tmp_path, monkeypatch):
    import threading
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    entered, release = threading.Event(), threading.Event()
    try:
        card.stop()
        original = __import__('trajectory_control').read_recording
        def read(*args):
            entered.set(); release.wait(2)
            return original(*args)
        monkeypatch.setattr('trajectory_control.read_recording', read)
        caller = threading.Thread(target=lambda: card.dispatch('play', {'filename': 'missing.jsonl'}))
        caller.start(); assert entered.wait(1)
        started = time.monotonic(); assert card.stop()['authority_released']
        assert time.monotonic()-started < .1
        release.set(); caller.join(2)
        assert not arm.session_id
    finally:
        release.set(); done.set(); ticker.join(1)


@pytest.mark.parametrize('failure', ['cancel', 'stop_unconfirmed', 'body_changed'])
def test_replay_failure_never_reports_completed(tmp_path, failure):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        assert card.dispatch('record_start', {})['recording']
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        asset = card.dispatch('inspect', {'filename': name})
        card.stop(); channel.positions.clear(); sample['q'] = list(asset['preview'][0]['target_q'])
        preview = card.dispatch('play', {'filename': name})
        if failure == 'body_changed':
            key = next(iter(sample['locked_joints'])); sample['locked_joints'][key] += .1
            result = card.dispatch('play', {'filename': name, 'execute': True,
                'preview_id': preview['preview_id'], 'workspace_clear': True, 'take_control': True})
            assert result['code'] == 'recording_body_baseline_mismatch' and not arm.session_id
            return
        original_wait = card.trajectory._wait_until
        def wait(deadline):
            if failure == 'cancel': card.trajectory.cancel.set()
            return original_wait(deadline)
        card.trajectory._wait_until = wait
        if failure == 'stop_unconfirmed':
            card._stop_motion = lambda: (_ for _ in ()).throw(ValueError('stop_unconfirmed'))
        result = card.dispatch('play', {'filename': name, 'execute': True,
            'preview_id': preview['preview_id'], 'workspace_clear': True, 'take_control': True})
        card.trajectory.thread.join(6)
        outcome = card.dispatch('action_status', {'action_id': result['action_id']})
        assert outcome['status'] == ('cancelled' if failure == 'cancel' else 'error'), outcome
        assert receipts[-1][0] == result['action_id'] and receipts[-1][1] != 'completed'
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)



def test_command_timeout_preserves_recording_and_resumes_same_file(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {}, [], arm.limits, clock=arm.clock)
    arm.record_sink = recorder
    advance(50); arm.accept_control(packet(arm, 1, target=.03)); arm.tick()
    first_feedback_ns = sample['arm_ns']
    advance(301); arm.tick()
    advance(10); arm.tick()
    assert arm.status()['continuation_ready'] and recorder.status()['recording']
    assert recorder.status()['error'] is None
    for seq in range(2, 12):
        advance(50); arm.accept_control(packet(arm, seq, target=.03)); arm.tick()
    result = recorder.stop()
    assert result['complete'], result
    frames = read_recording(recorder.directory, result['filename'], arm.limits)['frames']
    assert len(channel.positions) == 11 and len(frames) == 12
    assert [f['target_q'] for f in frames if f['kind'] == 'target'] == [q for q, _ in channel.positions]
    assert frames[1]['kind'] == 'hold' and frames[1]['feedback_ns'] == first_feedback_ns
    assert frames[2]['t_ns']-frames[1]['t_ns'] == 60_000_000


@pytest.mark.parametrize('failure', ['sdk', 'feedback'])
def test_execution_failure_keeps_recording_valid_prefix(tmp_path, failure):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {}, [], arm.limits, clock=arm.clock)
    arm.record_sink = recorder
    advance(50); arm.accept_control(packet(arm, 1, target=.03)); arm.tick()
    if failure == 'sdk':
        channel.publish_servo_position = lambda *a, **k: False
        advance(50); arm.accept_control(packet(arm, 2, target=.04)); arm.tick()
        assert arm.state == 'fault'
    else:
        advance(101, fresh=False); arm.tick()
        assert arm.state == 'hold'
    for _ in range(3): advance(10, fresh=False); arm.tick()
    assert recorder.status()['recording'] and recorder.error is None
    result = recorder.stop()
    assert result['complete'], result
    frames = read_recording(recorder.directory, result['filename'], arm.limits)['frames']
    assert [f['kind'] for f in frames] == ['target', 'hold']
    assert all(f['target_q'] == channel.positions[0][0] and f['sdk_sequence'] == 1 for f in frames)


@pytest.mark.parametrize('reason', ['input_stale', 'tracking_lost', 'ik_worker_timeout'])
def test_teleop_hold_does_not_terminate_recording(tmp_path, reason):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        assert card.dispatch('record_start', {})['recording']
        arm.accept_control(packet(arm, 1, target=.01))
        time.sleep(.06)
        card._hold(reason, grip=True)
        card._hold(reason, grip=True)
        assert card.trajectory.recorder.status()['recording']
        time.sleep(.03)
        resumed = arm.dispatch('resume', card.motion._lease())
        assert not resumed.get('error'), resumed
        arm.accept_control(packet(arm, 2, target=.02))
        time.sleep(.06)
        result = card.dispatch('record_stop', {})
        assert result['complete'], result
        frames = read_recording(card.trajectory.directory, result['filename'], arm.limits)['frames']
        assert [f['kind'] for f in frames] == ['target', 'hold', 'target']
    finally:
        done.set(); ticker.join(1)
        if card.trajectory.recorder: card.trajectory.recorder.stop()


def test_pause_without_successful_output_does_not_fabricate_frame(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {}, [], arm.limits, clock=arm.clock)
    for _ in range(3): advance(100); recorder.pause()
    result = recorder.stop()
    assert result['frames'] == 0 and not result['complete']
    assert result['error'] == 'recording_duration_or_count'
