"""Real numerical controller + MotionGate, synthetic plant; no ROS/network/device.

The plant ticks independently with finite speed. Publishing a target does not
overwrite measured positions; physical receipts require later plant samples.
"""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
from motion_control import MotionControl
from motion_stream import sign
from teleop_executor import TeleopExecutor
from tianyi_motion.kinematics import ARM_NAMES, TianyiIK
from tianyi_motion.protocol import envelope


def profile(tmp_path):
    xml = ['<robot name="offline"><link name="torso_link"/>']
    axes = ['0 1 0', '1 0 0', '0 0 1', '0 1 0', '0 0 1', '0 1 0', '1 0 0']
    for side, direction in [('left', 1), ('right', -1)]:
        parent = 'torso_link'
        for i, name in enumerate(n for n in ARM_NAMES if n.startswith(side)):
            child = name.replace('_joint', '_link')
            xyz = f'0 {direction*.5} 0' if i == 0 else '.08 0 0'
            xml.append(f'<link name="{child}"/><joint name="{name}" type="revolute">'
                f'<parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}"/>'
                f'<axis xyz="{axes[i]}"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>')
            parent = child
    xml.append('</robot>')
    urdf = tmp_path/'synthetic.urdf'
    urdf.write_text(''.join(xml))
    p = dict(schema='motus.tianyi-calibration.v1', version='synthetic-not-accepted',
        arm_joint_names=list(ARM_NAMES), torso_frame='torso_link', urdf_path=str(urdf),
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(), hands_enabled=False,
        joint_velocity_rad_s=1.,
        palm_frames={s: dict(position=[.05, 0., 0.], orientation=[0., 0., 0., 1.]) for s in ('left', 'right')},
        workspace={'torso_box': [[-.2, -.1, -.2], [.1, .1, .2]],
            'left': [[0., .3, -1.], [1., 1., 1.]], 'right': [[0., -1., -1.], [1., -.3, 1.]],
            'capsules': [{'from': f'{s}_elbow_pitch_link', 'to': f'{s}_wrist_roll_link',
                         'radius_m': .02, 'group': s} for s in ('left', 'right')]},
        acceptance={**{k: True for k in ('model_verified', 'workspace_verified', 'pico_verified',
            'external_control_excluded', 'stop_verified', 'driver_crash_verified')},
            'operator': 'synthetic-test', 'date': 'offline', 'evidence_sha256': '0'*64})
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(p))
    return path


class Plant:
    def __init__(self):
        self.lock = threading.RLock()
        self.q = np.zeros(14)
        self.dq = np.zeros(14)
        self.target = np.zeros(14)
        self.frozen = False
        self.timestamp = time.monotonic_ns()
        self.writes = []
        self.closed = threading.Event()
        self.thread = None

    def snapshot(self):
        with self.lock:
            result = {k: self.timestamp for k in ('arm_ns', 'power_ns', 'fixed_ns', 'hand_ns')}
            return {**result, 'q': self.q.tolist(), 'dq': self.dq.tolist(),
                    'power_on': True, 'estop': False, 'fault': False, 'fixed_body': True}

    def send(self, poses, speed):
        with self.lock:
            self.target = np.deg2rad(poses['left']+poses['right'])
            self.writes.append(self.target.copy())
        return {'state': 'sent'}

    def tick(self):
        with self.lock:
            if self.frozen:return
            desired = np.clip((self.target-self.q)/.02, -1., 1.)
            # Acceleration bound exposes follow/hold settling rather than teleportation.
            velocity = self.dq+np.clip(desired-self.dq, -.4, .4)
            step = np.clip(velocity*.02, -np.abs(self.target-self.q), np.abs(self.target-self.q))
            self.q += step
            self.dq = step/.02
            self.timestamp = time.monotonic_ns()

    def run(self, gate):
        def loop():
            while not self.closed.wait(.02):
                self.tick()
                gate.tick()
        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        if self.thread:self.thread.join(1)


@pytest.fixture
def chain(tmp_path, monkeypatch):
    plant = Plant()
    path = profile(tmp_path)
    arm = SimpleNamespace(_pos_publisher=True, _send_pos=plant.send)
    executor = TeleopExecutor({'calibration_path': str(path), 'live_enabled': True,
        'operator_session_enabled': True}, 'offline', None, arm, None, [])
    monkeypatch.setattr(executor, 'start', lambda: None)
    monkeypatch.setattr(executor, 'foreign_publishers', lambda: [])
    monkeypatch.setattr(executor, 'legacy_busy', lambda: False)
    monkeypatch.setattr(executor, '_capture_fixed_baseline', lambda: None)
    executor.gate.snapshot = plant.snapshot
    controller = MotionControl({}, executor)
    executor.motion_control = controller
    # Exercise the actual arm receiver method, not an always-success mock.
    cls = next(n for n in ast.parse((DRIVER/'device.py').read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'ArmPlugin')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'accept_control')
    scope = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'device.py', 'exec'), scope)
    arm._motion_control = controller
    arm.accept_control = lambda packet: scope['accept_control'](arm, packet)
    commands = []
    def publish(packet):
        commands.append(packet)
        # Only local DDS transport is replaced. Signature/gate/arm publish/plant remain real.
        return arm.accept_control(packet)
    monkeypatch.setattr(executor, 'publish_joint_command', publish)
    monkeypatch.setattr(controller, 'start', lambda: None)
    # Numerical library import is cold on the first case; sensors continue
    # independently in production. Refresh a real sample after the cold load.
    controller.solver_factory = lambda path: (lambda solver: (plant.tick(), solver)[1])(TianyiIK(path))
    controller.calibrate()
    yield SimpleNamespace(c=controller, e=executor, p=plant, commands=commands, path=path)
    controller.stop()
    plant.close()


def packet(c, lease, *, seq=0, q=None, mode='eef_pose', epoch=0, **changes):
    if q is None:q=np.zeros(14)
    from scipy.spatial.transform import Rotation
    values = list(q) if mode == 'joint_position' else [v for t in c.solver.palms(q)
        for v in t[:3, 3].tolist()+Rotation.from_matrix(t[:3, :3]).as_quat().tolist()]
    now = time.monotonic_ns()
    body = envelope(lease, seq=seq, source_seq=seq+10, mapping_epoch=epoch,
        generated_ns=now, valid_until_ns=now+(300_000_000 if mode == 'eef_pose' else 100_000_000),
        mode=mode, values=values, **c.versions)
    body.pop('mac')
    body.update(changes)
    return {**body, 'mac': sign(body, lease['secret'])}


def wait_for(predicate, timeout=2):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if predicate():return
        time.sleep(.01)
    assert predicate()


def test_metadata_topics_and_calibration_use_driver_model(chain):
    c = chain.c
    info = c.info()
    tool = c.get_tool()
    assert info['x-teleop-target']['protocol_version'] == 2
    assert tool['topic_out'][0]['topic'] == c.arm_metadata()['topic_in'][0]['topic']
    assert tool['topic_out'][1]['topic'] == chain.e.topic+'/feedback'
    assert len(info['eef_snapshot']['poses']) == 2
    assert all(len(p) == 7 for p in info['eef_snapshot']['poses'])
    assert info['control_interface']['frame'] == 'torso_link'
    assert not chain.p.writes and not chain.e.gate.session_id


def test_preview_solves_and_retains_visualization_without_claim_or_output(chain):
    c, e = chain.c, chain.e
    lease = c.dispatch('prepare_preview', {})
    assert lease['preview'] and lease['secret']
    goal = np.zeros(14);goal[3] = .03
    c.receive_eef(packet(c, lease, q=goal))
    assert c.process_latest()
    info = c.info()
    assert info['visualization']['available'] and info['visualization']['ik']
    assert info['eef_snapshot']['monotonic_ns'] == chain.p.timestamp
    assert not info['ownership_held'] and not info['output_active']
    assert not e.gate.session_id and not chain.commands and not chain.p.writes
    with pytest.raises(ValueError, match='preview_cannot_execute'):
        c.receive_joint(packet(c, lease, mode='joint_position'))
    assert c.dispatch('pause', lease)['hold_confirmed']
    again = c.dispatch('resume', lease)
    assert again['session_id'] != lease['session_id']
    with pytest.raises(ValueError, match='stale_session'):
        c.receive_eef(packet(c, lease, seq=2))
    assert c.dispatch('finish', again)['authority_released']


@pytest.mark.parametrize('change,reason', [
    ({'dof': True}, 'invalid_control_interface'), ({'values': [0]*13}, 'values'),
    ({'source_seq': True}, 'invalid_source_seq'), ({'mapping_epoch': -1}, 'invalid_mapping_epoch'),
    ({'model_version': 'other'}, 'control_calibration_mismatch'),
    ({'frame': 'world'}, 'control_calibration_mismatch'),
    ({'values': [0.]*14}, 'quaternion_not_unit'),
    ({'extra': 1}, 'invalid_command_fields'), ({'generated_ns': 0, 'valid_until_ns': 1}, 'command_expired'),
])
def test_v2_rejects_invalid_envelope_without_output(chain, change, reason):
    c = chain.c
    lease = c.dispatch('prepare_preview', {})
    with pytest.raises(ValueError, match=reason):c.receive_eef(packet(c, lease, **change))
    assert not chain.p.writes and not chain.commands


def test_latest_input_epoch_and_deadline_never_replay_superseded_result(chain):
    c = chain.c;lease = c.dispatch('claim', {})
    first = packet(c, lease, seq=1, epoch=2)
    second = packet(c, lease, seq=2, epoch=2)
    c.receive_eef(first);c.receive_eef(second)
    with pytest.raises(ValueError, match='stale_sequence'):c.receive_eef(first)
    with pytest.raises(ValueError, match='stale_mapping_epoch'):c.receive_eef(packet(c, lease, seq=3, epoch=1))
    assert c.process_latest()
    sent = chain.commands[-1]
    assert sent['source_seq'] == second['source_seq'] and sent['mapping_epoch'] == 2
    assert sent['valid_until_ns'] <= second['valid_until_ns']
    assert 0 < sent['valid_until_ns']-sent['generated_ns'] <= 300_000_000
    assert not c.process_latest()


def test_live_eef_to_arm_gate_plant_and_stop_require_later_feedback(chain):
    c, e, p = chain.c, chain.e, chain.p
    lease = c.dispatch('claim', {})
    assert e.gate.session_id and not p.writes
    with pytest.raises(ValueError, match='owned'):
        with e.gate.legacy():pass
    target = np.zeros(14);target[3] = .1
    c.receive_eef(packet(c, lease, q=target));assert c.process_latest()
    assert not p.writes and np.all(p.q == 0)
    e.gate.tick()
    assert p.writes and np.all(p.q == 0)  # publication is not measured motion
    p.tick();assert np.max(np.abs(p.q)) > 0
    p.run(e.gate)
    assert not c.dispatch('release', lease).get('error')
    assert e.gate.session_id
    wait_for(lambda: not e.gate.session_id)
    assert e.gate.status()['stop_confirmed']


def test_cancel_during_solver_never_publishes_result(chain, monkeypatch):
    c = chain.c;lease = c.dispatch('claim', {})
    original = c.solver.solve
    def solve(*args, **kwargs):
        result = original(*args, **kwargs)
        assert not c.dispatch('pause', lease).get('error')
        return result
    monkeypatch.setattr(c.solver, 'solve', solve)
    c.receive_eef(packet(c, lease))
    assert not c.process_latest()
    assert not chain.commands


def test_failed_ik_hold_recovers_same_session_with_fresh_frame(chain, monkeypatch):
    c, e, p = chain.c, chain.e, chain.p
    lease = c.dispatch('claim', {})
    p.run(e.gate)
    original = c.solver.solve
    monkeypatch.setattr(c.solver, 'solve', lambda *a, **kw: (_ for _ in ()).throw(ValueError('ik_target_unreachable')))
    c.receive_eef(packet(c, lease));assert not c.process_latest()
    assert not chain.commands
    wait_for(lambda: e.gate.status()['hold_confirmed'])
    monkeypatch.setattr(c.solver, 'solve', original)
    c.receive_eef(packet(c, lease, seq=1));assert c.process_latest()
    wait_for(lambda: e.gate.state == 'active')
    assert e.gate.session_id == lease['session_id']


def test_finish_without_started_session_does_not_move(chain):
    result = chain.c.dispatch('finish', {})
    assert result['return_completed'] and result['authority_released']
    assert result['motion_requested'] is False
    assert not chain.p.writes and not chain.e.gate.session_id


def test_finish_returns_neutral_and_releases_without_headset_dependency(chain):
    c, e, p = chain.c, chain.e, chain.p
    p.q[3] = p.target[3] = .08
    lease = c.dispatch('claim', {})
    p.run(e.gate)
    result = c.dispatch('finish', lease)
    assert result['state'] == 'returning'
    again = c.dispatch('finish', lease)
    assert again['operation_id'] == result['operation_id']
    wait_for(lambda: c.dispatch('finish_status', {})['state'] != 'returning', 4)
    final = c.dispatch('finish_status', {})
    assert final['return_completed'] and final['authority_released'], final
    assert not e.gate.session_id and np.max(np.abs(p.q)) <= .02
    assert len(chain.commands) > 2 and len(p.writes) > 2


def test_return_failure_never_reports_completion_and_retry_can_finish(chain):
    c, e, p = chain.c, chain.e, chain.p
    p.q[3] = p.target[3] = .08
    lease = c.dispatch('claim', {})
    p.frozen = True
    p.run(e.gate)
    c.dispatch('finish', lease)
    wait_for(lambda: c.dispatch('finish_status', {})['state'] == 'error', 2)
    failed = c.dispatch('finish_status', {})
    assert not failed['return_completed'] and not failed['authority_released']
    assert e.gate.session_id
    p.frozen = False
    # A prolonged stale-feedback fault remains visible; fresh physical stop
    # is the receipt needed by an explicit return retry, not a fabricated HOLD.
    wait_for(lambda: e.gate.status()['stop_confirmed'])
    c.dispatch('finish', lease)
    wait_for(lambda: c.dispatch('finish_status', {})['state'] != 'returning', 4)
    final = c.dispatch('finish_status', {})
    assert final['return_completed'] and final['authority_released'], final


def test_model_bytes_and_license_preserved():
    model = DRIVER/'tianyi_motion/models/tianyi2-official.urdf'
    assert hashlib.sha256(model.read_bytes()).hexdigest() == 'a7e742ad600c7f1e9eeecdd04046dea4cd5f81cccb6dde73f290968b345e4b20'
    assert 'OpenAtom Open Hardware License' in model.with_name('LICENSE').read_text()


def test_framework_stop_can_clear_preview_without_live_credentials(chain):
    c = chain.c
    c.dispatch('prepare_preview', {})
    assert c.dispatch('stop', {})['state'] == 'idle'
    assert not c.info()['preview'] and not chain.p.writes
    lease = c.dispatch('claim', {})
    assert c.dispatch('stop', {})['code'] == 'invalid_lease'
    assert chain.e.gate.session_id == lease['session_id']


def test_legacy_claim_revokes_preview_without_hiding_ownership(chain):
    c, e = chain.c, chain.e
    preview = c.dispatch('prepare_preview', {})
    lease = e.dispatch('claim', {})
    info = c.info()
    assert info['ownership_held'] and info['session_id'] == lease['session_id']
    assert not info['preview']
    with pytest.raises(ValueError, match='invalid_lease'):
        c.receive_eef(packet(c, preview))
    assert not chain.p.writes


def test_claim_reply_retry_keeps_pending_input_and_rejects_replay(chain):
    import secrets
    c = chain.c
    args = {'request_id': secrets.token_hex(16), 'request_valid_until_ns': time.monotonic_ns()+300_000_000}
    lease = c.dispatch('claim', args)
    value = packet(c, lease, seq=10)
    c.receive_eef(value)
    assert c.dispatch('claim', args) == lease
    assert c.process_latest() and chain.commands[-1]['source_seq'] == 20
    with pytest.raises(ValueError, match='stale_sequence'):c.receive_eef(value)


def test_descriptors_and_binding_checked_before_start_without_claim(chain):
    c = chain.c
    binding = {**c.arm_metadata()['x-control-target'], 'tool': 'arm', 'url': 'http://127.0.0.1:15707/mcp', 'mcp_id': 'test'}
    args = {'execution_binding': binding, 'control_interface': c.control_interface('joint_position'),
            'input_topic': c.topic+'/command'}
    assert not c.dispatch('start', args).get('error')
    assert not chain.e.gate.session_id and not chain.p.writes
    assert c.dispatch('start', {**args, 'execution_binding': {**binding, 'namespace': 'other'}})['code'] == 'motion_control_execution_binding_mismatch'
    assert c.dispatch('start', {**args, 'control_interface': c.control_interface()})['code'] == 'motion_control_execution_binding_mismatch'


def test_retired_ik_result_cannot_cross_release_reclaim(chain, monkeypatch):
    c, e, p = chain.c, chain.e, chain.p
    lease = c.dispatch('claim', {})
    p.run(e.gate)
    original = c.solver.solve
    def solve(*args, **kwargs):
        result = original(*args, **kwargs)
        assert not c.dispatch('release', lease).get('error')
        wait_for(lambda: not e.gate.session_id)
        new = c.dispatch('claim', {})
        assert new['session_id'] != lease['session_id']
        return result
    monkeypatch.setattr(c.solver, 'solve', solve)
    c.receive_eef(packet(c, lease))
    assert not c.process_latest() and not chain.commands


def test_old_source_deadline_caps_joint_lifetime(chain):
    c = chain.c;lease = c.dispatch('claim', {})
    raw = packet(c, lease)
    raw.pop('mac')
    raw['valid_until_ns'] = time.monotonic_ns()+80_000_000
    raw['mac'] = sign(raw, lease['secret'])
    c.receive_eef(raw)
    assert c.process_latest()
    assert chain.commands[-1]['valid_until_ns'] <= raw['valid_until_ns']


def test_live_speed_is_not_multiplied_by_two_rate_limits(chain):
    c, e, p = chain.c, chain.e, chain.p
    lease = c.dispatch('claim', {})
    target = np.zeros(14);target[3] = .4
    c.receive_eef(packet(c, lease, q=target))
    assert c.process_latest()
    commanded = np.asarray(chain.commands[-1]['values'])
    assert np.max(np.abs(commanded)) > .3  # Full IK reference, not one frame's speed quota.
    e.gate.last_emit = e.gate.clock()-20_000_000
    previous_emit = e.gate.last_emit
    e.gate.tick()
    actual_budget = min(.1, (e.gate.last_emit-previous_emit)/1e9)*e.gate.velocity
    assert .015 < np.max(np.abs(p.writes[-1])) <= actual_budget+1e-9
    # Without a second input frame, the arm continues on its own clock.
    p.tick()
    previous_target = p.writes[-1][3]
    e.gate.last_emit = e.gate.clock()-20_000_000
    previous_emit = e.gate.last_emit
    e.gate.tick()
    actual_budget = min(.1, (e.gate.last_emit-previous_emit)/1e9)*e.gate.velocity
    assert .035 < p.writes[-1][3] <= previous_target+actual_budget+1e-9


def test_card_config_validates_atomically_and_requires_recalibration(chain):
    c, e = chain.c, chain.e
    before = c.info()['config']
    assert c.get_tool()['configSchema']['properties']['calibration_path']['x-sensitive']
    assert c.dispatch('config', {'calibration_path': '/does/not/exist'})['state'] == 'error'
    assert c.info()['config'] == before
    assert c.dispatch('config', {'joint_velocity_rad_s': 1.2})['code'] == 'joint_velocity_limit'  # model max is 1
    assert c.info()['config'] == before and c.solver is not None
    result = c.dispatch('config', {'joint_velocity_rad_s': .6})
    assert result['state'] == 'configured' and result['calibrated'] is False
    assert c.solver is None and e.gate.velocity == .6
    assert c.info()['config']['joint_velocity_rad_s'] == .6
    assert c.dispatch('calibrate', {})['calibrated'] and c.solver.velocity == .6
    assert not chain.p.writes
    preview = c.dispatch('prepare_preview', {})
    assert c.dispatch('config', {'joint_velocity_rad_s': .5})['code'] == 'configuration_requires_idle'
    c.dispatch('release', preview)
    lease = c.dispatch('claim', {})
    assert c.dispatch('config', {'joint_velocity_rad_s': .5})['code'] == 'motion_owned_by_teleop'
    assert e.gate.session_id == lease['session_id'] and e.gate.velocity == .6


def test_finish_from_pause_retry_and_stop_use_original_operation_credentials(chain):
    c, e, p = chain.c, chain.e, chain.p
    p.q[3] = p.target[3] = .3
    lease = c.dispatch('claim', {})
    p.run(e.gate)
    c.dispatch('pause', lease)
    wait_for(lambda: e.gate.status()['hold_confirmed'])
    first = c.dispatch('finish', lease)
    wait_for(lambda: e.gate.session_id != lease['session_id'])
    assert c.dispatch('finish', lease)['operation_id'] == first['operation_id']
    assert not c.dispatch('stop', lease).get('error')
    wait_for(lambda: c.dispatch('finish_status', {})['state'] == 'error')
    wait_for(lambda: not e.gate.session_id)
    assert not c.dispatch('finish_status', {})['return_completed']
    assert e.gate.status()['stop_confirmed']


def test_input_queued_before_hold_receipt_cannot_resume_afterward(chain):
    c, e, p = chain.c, chain.e, chain.p
    lease = c.dispatch('claim', {})
    p.run(e.gate)
    c.dispatch('recoverable_hold', lease)
    c.receive_eef(packet(c, lease))
    wait_for(lambda: e.gate.status()['hold_confirmed'])
    assert not c.process_latest() and not chain.commands
    assert c.info()['control_decision']['reason'] == 'waiting_fresh_input_after_hold'
    c.receive_eef(packet(c, lease, seq=1))
    assert c.process_latest()


def test_new_mapping_epoch_fences_inflight_solver_and_display(chain, monkeypatch):
    c = chain.c;lease = c.dispatch('claim', {})
    first = packet(c, lease, seq=1, epoch=1)
    second = packet(c, lease, seq=2, epoch=2)
    original = c.solver.solve
    def solve(*args, **kwargs):
        result = original(*args, **kwargs)
        c.receive_eef(second)  # equivalent to input arriving while solve is active
        return result
    monkeypatch.setattr(c.solver, 'solve', solve)
    c.receive_eef(first)
    assert not c.process_latest() and not chain.commands
    assert c.solver.visualization_sample is None and c.solver.last_valid_visualization is None
    assert chain.e.gate.state == 'ready'
    monkeypatch.setattr(c.solver, 'solve', original)
    assert c.process_latest() and chain.commands[-1]['mapping_epoch'] == 2
