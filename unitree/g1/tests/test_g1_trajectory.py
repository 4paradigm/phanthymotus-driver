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
    advance(50); arm.replay_deadline_ns = arm.clock()+100_000_000
    arm.write_replay(command)
    assert channel.positions[-1][0] == [.02]*10
    advance(101); arm.tick()
    assert arm.state == 'hold' and arm.reason == 'replay_watchdog'
    before = len(channel.positions)
    with pytest.raises(ValueError): arm.write_replay(command)
    assert len(channel.positions) == before


def test_replay_sends_saved_target_and_rejects_stale_feedback(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    command = packet(arm, target=.1)
    advance(10); arm.replay_deadline_ns = arm.clock()+200_000_000
    arm.write_replay(command)
    assert channel.positions[-1][0] == pytest.approx([.1]*10)
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    arm.replay_deadline_ns = arm.clock()+200_000_000
    command = packet(arm, seq=2, target=.1)
    advance(101, fresh=False)
    with pytest.raises(ValueError, match='feedback'): arm.write_replay(command)
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
    motion.start = arm.start = lambda: None  # No DDS or SDK setup in this synthetic plant.
    motion._live_session = arm.session_id
    motion.solver = SimpleNamespace(profile=arm.profile, profile_sha256=arm.profile_sha256,
        interrupt=lambda: None, close=lambda: None,
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
        offset_preview = card.dispatch('start_difference', {'filename': name})
        assert offset_preview['ready']
        sample['q'] = list(asset['preview'][0]['target_q'])
        preview = card.dispatch('start_difference', {'filename': name})
        assert preview['ready'] and not arm.session_id
        count = len(channel.positions)
        result = card.dispatch('play', {'filename': name})
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
        rejected = card.dispatch('play', {'filename': name})
        assert rejected['status'] == 'running'
        card.trajectory.thread.join(6)
        assert card.trajectory.last_action['status'] == 'error'  # Hardware gates still apply.
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


@pytest.mark.parametrize('failure', ['cancel', 'stop_unconfirmed'])
def test_replay_failure_never_reports_completed(tmp_path, failure):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        assert card.dispatch('record_start', {})['recording']
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        asset = card.dispatch('inspect', {'filename': name})
        card.stop(); channel.positions.clear(); sample['q'] = list(asset['preview'][0]['target_q'])
        preview = card.dispatch('start_difference', {'filename': name})
        original_wait = card.trajectory._wait_until
        def wait(deadline):
            if failure == 'cancel': card.trajectory.cancel.set()
            return original_wait(deadline)
        card.trajectory._wait_until = wait
        if failure == 'stop_unconfirmed':
            card._stop_motion = lambda: (_ for _ in ()).throw(ValueError('stop_unconfirmed'))
        result = card.dispatch('play', {'filename': name})
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


@pytest.mark.parametrize('degrees', [3., 5., 20.])
def test_start_difference_reports_offset_without_position_gate(tmp_path, degrees):
    import math
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    control = SimpleNamespace(cfg={}, motion=SimpleNamespace(executor=arm), executor=arm)
    trajectory = TrajectoryControl(control)
    sample['q'] = [math.radians(degrees)]*10
    difference = trajectory._difference({'frames': [{'target_q': [0.]*10}]})
    assert difference['ready']
    assert difference['joint_delta_rad'] == [math.radians(degrees)]*10
    assert 'joint_tolerance_rad' not in difference
    sample['dq'][0] = .03
    assert not trajectory._difference({'frames': [{'target_q': [0.]*10}]})['ready']


def test_replay_moves_to_start_at_point_one_then_preserves_recorded_timing(tmp_path):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    writes = []
    try:
        card.dispatch('record_start', {})
        for seq in (1, 2, 3):
            arm.accept_control(packet(arm, seq, target=.01*seq)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        recording = read_recording(card.trajectory.directory, name, arm.limits)
        card.stop(); channel.positions.clear()
        sample['q'] = [q+.12 for q in recording['frames'][0]['target_q']]
        preview = card.dispatch('start_difference', {'filename': name})
        assert preview['ready'] and not channel.positions
        original = arm.write_replay
        def write(command):
            before, stamp = list(arm.last_q), arm.last_emit
            original(command)
            writes.append((card.trajectory.last_action['phase'], before, stamp,
                           list(command['values']), arm.last_emit, command['seq']))
        arm.write_replay = write
        result = card.dispatch('play', {'filename': name})
        card.trajectory.thread.join(7)
        outcome = card.dispatch('action_status', {'action_id': result['action_id']})
        assert outcome['status'] == 'completed', outcome
        lead = [w for w in writes if w[0] == 'moving_to_start']
        replay = [w for w in writes if w[0] == 'replaying']
        assert len(lead) > 1 and lead[-1][3] == recording['frames'][0]['target_q']
        for _, before, start, after, end, _ in lead:
            assert max(abs(a-b) for a,b in zip(after, before)) <= .1*(end-start)/1e9+1e-9
        assert [w[3] for w in replay] == [f['target_q'] for f in recording['frames']]
        assert [w[5] for w in writes] == list(range(len(writes)))
        for previous, current, fp, fc in zip(replay, replay[1:], recording['frames'], recording['frames'][1:]):
            assert (current[4]-previous[4])/1e9 == pytest.approx((fc['t_ns']-fp['t_ns'])/1e9, abs=.02)
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('failure', ['cancel', 'stale_feedback'])
def test_start_transition_errors_stop_and_position_offset_does_not_gate(tmp_path, failure):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    writes = []
    try:
        card.dispatch('record_start', {})
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        target = read_recording(card.trajectory.directory, name, arm.limits)['frames'][0]['target_q']
        card.stop(); channel.positions.clear(); sample['q'] = [q+.12 for q in target]
        preview = card.dispatch('start_difference', {'filename': name})
        original = arm.write_replay
        def write(command):
            original(command)
            writes.append(card.trajectory.last_action['phase'])
            if failure == 'cancel': card.trajectory.stop()
            elif failure == 'stale_feedback':
                snapshot = arm._snapshot_provider()
                snapshot['arm_ns'] = arm.clock()-1_000_000_000
                arm._snapshot_provider = lambda: copy.deepcopy(snapshot)
        arm.write_replay = write
        result = card.dispatch('play', {'filename': name})
        card.trajectory.thread.join(9)
        outcome = card.dispatch('action_status', {'action_id': result['action_id']})
        assert outcome['status'] in ('cancelled', 'error'), outcome
        assert 'replaying' not in writes
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('waiting_binding', [False, True])
def test_filename_only_replay_without_teleop_start_or_manual_stop(tmp_path, waiting_binding):
    from teleop_control import TeleopControl
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.dispatch('record_start', {})
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        card.stop(); channel.positions.clear()
        card = TeleopControl(card.cfg, card.motion)
        if waiting_binding:
            card.binding = {'command_topic': '/teleop/command', 'feedback_topic': '/teleop/state'}
        schema = card.get_tool()['inputSchema']
        assert schema['x-action-params']['play']['params'] == ['filename']
        assert not {'execute', 'preview_id', 'workspace_clear', 'take_control'} & schema['properties'].keys()
        assert card.dispatch('play', {'filename': name, 'execute': False})['code'] == 'use_inspect_before_replay'
        assert not channel.positions
        result = card.dispatch('play', {'filename': name})
        assert result['status'] == 'running', result
        assert card._closed.is_set()
        card.trajectory.thread.join(6)
        assert card.trajectory.last_action['status'] == 'completed', card.trajectory.last_action
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('change', ['stop', 'operator', 'begin', 'reset'])
def test_replay_does_not_race_teleop_during_file_read(tmp_path, monkeypatch, change):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.dispatch('record_start', {})
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        recorded = card.dispatch('record_stop', {})
        assert recorded['complete'], recorded
        name = recorded['filename']
        card.stop(); channel.positions.clear()
        original = __import__('trajectory_control').read_recording
        def read(*args):
            recording = original(*args)
            if change == 'stop': card.stop()
            elif change == 'operator': card._operator = 'concurrent-session'
            elif change == 'begin': card._begin_lock.acquire()
            else: card._reset_operation = object()
            return recording
        monkeypatch.setattr('trajectory_control.read_recording', read)
        result = card.dispatch('play', {'filename': name})
        assert result['code'] == {'stop': 'replay_cancelled', 'operator': 'stop_teleop_before_replay',
                                  'begin': 'motion_busy', 'reset': 'motion_busy'}[change]
        assert not channel.positions and not card.trajectory.busy
    finally:
        if card._begin_lock.locked(): card._begin_lock.release()
        card._reset_operation = None
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('delay', [.025, .075])
def test_close_pause_frame_tolerates_scheduler_jitter(tmp_path, delay):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.stop(); channel.positions.clear()
        target = list(sample['q'])
        frame = {'kind': 'target', 'target_q': target, 'source_sequence': 1, 'mapping_epoch': 1}
        recording = {'filename': 'timing.jsonl', 'sha256': 'test',
            'binding': card.trajectory.binding(), 'joint_names': arm.profile['arm_joint_names'],
            'duration_ns': 200_000_000,
            'frames': [dict(frame, t_ns=50_000_000),
                       dict(frame, kind='hold', t_ns=64_000_000),
                       dict(frame, t_ns=150_000_000, source_sequence=2)]}
        original = card.trajectory._send_frame
        def send(frame, seq, due):
            original(frame, seq, due)
            if seq == 0: time.sleep(delay)
        card.trajectory._send_frame = send
        card.trajectory.cancel.clear()
        card.trajectory.last_action = {'phase': 'moving_to_start'}
        card.trajectory._run('close-pause', recording)
        assert card.trajectory.last_action['status'] == 'completed', card.trajectory.last_action
        assert receipts[-1][1] == 'completed'
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('replay', [False, True])
@pytest.mark.parametrize('failure', ['sdk', 'envelope', 'expired'])
def test_live_and_replay_use_same_execution_failure_handling(tmp_path, replay, failure):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    advance(50)
    command = packet(arm, target=.01, life=1 if failure == 'expired' else 300)
    proof = {'schema': 'motus.motion.envelope/1', **{k: command[k] for k in IDENTITY},
             'lower': [-1.]*10, 'upper': [1.]*10, 'measured_ns': arm.clock()}
    if failure == 'sdk':
        channel.publish_servo_position = lambda *a, **k: False
    if failure == 'envelope':
        proof['lower'] = [.5]*10
    arm.install_motion_envelope(proof)
    arm.accept_control(command)
    if replay:
        arm.replay_deadline_ns = arm.clock()+100_000_000
    advance(2)
    arm.tick()
    if replay and failure == 'envelope':
        assert channel.positions and arm.applied_seq == command['seq']
        return
    assert not channel.positions and arm.applied_seq == -1
    assert arm.reason == {'sdk': 'arm_sdk_write_failed',
                          'envelope': 'motion_envelope_exceeded',
                          'expired': 'command_timeout'}[failure]
    assert arm.state == ('fault' if failure == 'sdk' else 'hold')


def test_delayed_replay_skips_obsolete_targets_but_keeps_pause(tmp_path):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.stop()
        target = list(sample['q'])
        frame = {'kind': 'target', 'target_q': target, 'mapping_epoch': 1}
        frames = [dict(frame, t_ns=t*1_000_000, source_sequence=i+1,
                       kind='hold' if i == 4 else 'target')
                  for i, t in enumerate((10, 20, 30, 40, 45, 100))]
        recording = {'filename': 'latest.jsonl', 'sha256': 'test',
            'binding': card.trajectory.binding(), 'joint_names': arm.profile['arm_joint_names'],
            'duration_ns': 150_000_000, 'frames': frames}
        sent = []; holds = []
        send = card.trajectory._send_frame
        hold = arm.hold
        def delayed(frame, seq, due):
            send(frame, seq, due); sent.append(frame['source_sequence'])
            if len(sent) == 1: time.sleep(.055)
        def pause(reason, **kwargs):
            holds.append(reason); return hold(reason, **kwargs)
        card.trajectory._send_frame = delayed; arm.hold = pause
        card.trajectory.cancel.clear()
        card.trajectory.last_action = {'phase': 'moving_to_start'}
        card.trajectory._run('latest', recording)
        assert card.trajectory.last_action['status'] == 'completed', card.trajectory.last_action
        assert sent == [1, 4, 6]
        assert 'replay_pause' in holds
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


def test_final_reference_is_sent_without_extra_convergence_loop(tmp_path):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.stop(); channel.positions.clear()
        sent = []
        original = arm.write_replay
        def write(command):
            sent.append(command['values'])
            original(command)
        arm.write_replay = write
        start = list(sample['q']); target = [q+.08 for q in start]
        frame = {'kind': 'target', 'mapping_epoch': 1}
        recording = {'filename': 'final.jsonl', 'sha256': 'test',
            'binding': card.trajectory.binding(), 'joint_names': arm.profile['arm_joint_names'],
            'duration_ns': 60_000_000,
            'frames': [dict(frame, t_ns=10_000_000, target_q=start, source_sequence=1),
                       dict(frame, t_ns=40_000_000, target_q=target, source_sequence=2)]}
        card.trajectory.cancel.clear(); card.trajectory.last_action = {'phase': 'moving_to_start'}
        card.trajectory._run('final', recording)
        assert card.trajectory.last_action['status'] == 'completed', card.trajectory.last_action
        assert channel.positions[-1][0] == pytest.approx(target)
        assert sent == [start, target]
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


@pytest.mark.parametrize('cold_profile', [False, True])
def test_replay_uses_recorded_joints_without_numerical_worker_or_body_baseline(tmp_path, cold_profile):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        card.dispatch('record_start', {})
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        name = card.dispatch('record_stop', {})['filename']
        card.stop(); channel.positions.clear()
        sample['locked_joints'][next(iter(sample['locked_joints']))] += .1
        if cold_profile:
            arm.profile = None
            card.motion.cfg['calibration_path'] = str(tmp_path/'synthetic.json')
        card.motion.calibrate = lambda *a: pytest.fail('replay must not calibrate IK')
        card.motion.solver.joint_envelope = lambda *a: pytest.fail('replay must not calculate envelopes')
        result = card.dispatch('play', {'filename': name})
        assert result['status'] == 'running', result
        card.trajectory.thread.join(6)
        assert card.trajectory.last_action['status'] == 'completed', card.trajectory.last_action
        assert card.motion.solver is None and channel.positions
        assert receipts[-1][1] == 'completed' and not arm.session_id
    finally:
        card.trajectory.stop(); done.set(); ticker.join(1)


def test_recording_keeps_successful_targets_despite_record_timestamp_jitter(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    recorder = Recorder(tmp_path/'recordings', {}, [], arm.limits, clock=arm.clock)
    arm.record_sink = recorder
    original_record = arm._record
    def delayed_record(kind, sample, command):
        if command['seq'] == 1:
            advance(10)
        original_record(kind, sample, command)
    arm._record = delayed_record
    for seq in (1, 2):
        advance(20)
        arm.accept_control(packet(arm, seq, target=.8)); arm.tick()
    result = recorder.stop()
    assert result['complete'], result
    frames = read_recording(recorder.directory, result['filename'], arm.limits)['frames']
    assert [f['target_q'] for f in frames] == [q for q, _ in channel.positions]
    elapsed = (frames[1]['t_ns']-frames[0]['t_ns'])/1e9
    assert max(abs(a-b) for a,b in zip(frames[1]['target_q'], frames[0]['target_q'])) > elapsed


@pytest.mark.parametrize('ending', ['reset', 'stop', 'release'])
def test_operator_end_preserves_recording_and_later_record_stop(tmp_path, ending):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    try:
        assert card.dispatch('record_start', {})['recording']
        for seq in (1, 2):
            arm.accept_control(packet(arm, seq, target=.01)); time.sleep(.06)
        recorder = card.trajectory.recorder
        if ending == 'reset':
            card._start_reset()
        elif ending == 'release':
            card.cancel_for_arm_release()
        else:
            card.stop()
        result = card.dispatch('record_stop', {})
        assert result['complete'] and result['error'] is None, result
        assert result == recorder.status()
        assert arm.record_sink is None
        frames = read_recording(recorder.directory, result['filename'], arm.limits)['frames']
        assert len([f for f in frames if f['kind'] == 'target']) == 2
        assert card.dispatch('record_stop', {}) == result
    finally:
        done.set(); ticker.join(1)
        if card.trajectory.recorder: card.trajectory.recorder.stop()
