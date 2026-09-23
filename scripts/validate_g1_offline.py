"""Read-only candidate audit with a finite-velocity plant; NEVER opens a robot.

Requires the existing CasADi-enabled numerical environment. Candidate and
mapping directories are explicit so an uncommitted worktree is not copied.
This probes the old G1 execution components; it is not a product wire adapter.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--pico', type=Path, required=True)
    parser.add_argument('--mapping', type=Path, required=True)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trajectory', choices=('recording', 'reachable'), default='recording')
    parser.add_argument('--seed', choices=('zero', 'recorded'), default='zero')
    parser.add_argument('--inject-ik-failure', action='store_true')
    args = parser.parse_args()
    driver = args.candidate.resolve() / 'unitree/g1'
    sys.path[:0] = [str(driver), str(args.candidate.resolve())]
    from arm_stream import ArmStreamExecutor
    from motion_control import MotionControl
    from common.motion.protocol import envelope
    numeric_test = load('numeric_fixture', driver / 'tests/test_motion_control_numeric.py')
    mapper_class = load('recorded_mapping', args.mapping).G1ClutchRelativeMapper
    contract = load('pico_contract', args.pico / 'common/teleop_contract.py')
    from g1_motion.kinematics import G1IK

    rows = json.loads(args.recording.read_text())
    report = {'hardware_output': False, 'product_two_card_pass': False,
              'substitutes': ['SDK channel', 'finite-speed feedback plant',
                              'vendor action 99', 'internal DDS JSON round trip'],
              'synthetic_profile': True, 'frozen_comparison_pass': False,
              'frozen_gap': 'Saturday image/source/calibration identities not established',
              'frames': len(rows), 'trajectory': args.trajectory,
              'mapping_in_path': args.trajectory == 'recording',
              'initial_state': args.seed, 'resampled_input_hz': 20}
    report['injected_unreachable_source_seq'] = 25 if args.inject_ik_failure else None
    files = [args.recording, args.mapping, driver / 'arm_stream.py',
             driver / 'motion_control.py', driver / 'g1_motion/g1_ik.py',
             driver / 'g1_motion/kinematics.py', args.pico / 'common/teleop_contract.py']
    report['sources'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    lock = threading.Lock()
    initial_q = np.asarray(rows[0]['q'] if args.seed == 'recorded' else [0.]*10)
    plant = initial_q.copy()
    target = initial_q.copy()
    writes = []
    publications = []
    stop = threading.Event()
    sample = {'q': plant.tolist(), 'dq': [0.]*10, 'arm_ns': time.monotonic_ns(),
              'fault': False, 'model_verified': True}
    sdk = []

    class Channel:
        def publish_stream(self, q, tau, *, weight):
            with lock:
                target[:] = q
                writes.append((time.monotonic_ns(), list(q), weight))
            return True

        def close(self):
            pass

    def snapshot():
        with lock:
            return dict(sample, q=sample['q'][:], dq=sample['dq'][:])

    def plant_loop():
        last = time.monotonic()
        while not stop.wait(.004):
            now = time.monotonic()
            dt = now-last
            last = now
            with lock:
                step = np.clip(target-plant, -.6*dt, .6*dt)
                plant[:] += step
                sample.update(q=plant.tolist(), dq=(step/dt).tolist(), arm_ns=time.monotonic_ns())

    thread = threading.Thread(target=plant_loop, daemon=True)
    thread.start()
    motion = None
    arm = None
    trace = []
    try:
        with tempfile.TemporaryDirectory(prefix='g1-offline-profile-') as tmp:
            path = numeric_test.profile(Path(tmp))
            # Only isolated plant execution: do not populate real acceptance fields.
            arm = ArmStreamExecutor({'live_enabled': True}, 'g1',
                SimpleNamespace(ExecuteAction=lambda value: sdk.append(value) or 0),
                snapshot=snapshot, channel=Channel())
            motion = MotionControl({'calibration_path': str(path), 'joint_velocity_rad_s': 1.}, arm)
            motion.calibrate()
            fk = G1IK(path)
            mapper = mapper_class()
            mapper.reset(rows[0]['frame'], fk.palms(initial_q))
            # Explicit test-only arming, no safety records or real SDK are created.
            arm._prepared = True
            claim = motion.dispatch('claim', {})
            if claim.get('error'):
                raise RuntimeError(claim)
            def publish(packet):
                result = arm.accept_control(json.loads(json.dumps(packet)))
                publications.append({'source_seq': packet['source_seq'],
                                     'mapping_epoch': packet['mapping_epoch'],
                                     'session_id': packet['session_id']})
                return result
            arm.set_joint_publisher(publish)
            ports = motion.get_tool()
            report['current_ports'] = {k: ports[k] for k in ('name', 'topic_in', 'topic_out')}
            pose = {'tracked': True, 'position': [0., 0., 1.], 'orientation_xyzw': [0., 0., 0., 1.]}
            now = time.monotonic_ns()
            wire = {'schema': contract.COMMAND_SCHEMA, 'kind': 'input', 'instance_id': 'pico_1',
                    'device_id': 'offline', 'connection_epoch': 1, 'space_epoch': 1,
                    'sequence': 1, 'source_monotonic_ns': 1, 'received_monotonic_ns': now,
                    'clock_id': 'offline', 'tracking_frame': contract.TRACKING_FRAME,
                    'head_reference': pose, 'left': dict(pose, grip=1., trigger=0.),
                    'right': dict(pose, grip=1., trigger=0.)}
            contract.validate_input(wire, instance_id='pico_1', clock_id='offline', now_ns=now)
            try:
                motion.receive_eef(wire)
                report['wire_admission'] = 'unexpected_acceptance'
            except ValueError as exc:
                report['wire_admission'] = str(exc)
            session = arm.session_id
            for seq, row in enumerate(rows):
                started = time.monotonic()
                if seq == len(rows)//2:
                    # Deliberate lost-input interval; the next packet remains in the same mapping.
                    time.sleep(.4)
                    report['dropout_state'] = arm.status()['state']
                    report['dropout_reason'] = arm.status()['reason']
                if args.trajectory == 'reachable' and seq == 15:
                    # Exercise the real pause/resume management path. The upper
                    # layer's physical grip admission is still missing.
                    paused = motion.dispatch('pause', motion._lease())
                    deadline = time.monotonic()+1.
                    while not arm.status()['hold_confirmed'] and time.monotonic() < deadline:
                        time.sleep(.004)
                    resumed = motion.dispatch('resume', motion._lease())
                    report['pause_resume'] = {'pause_error': paused.get('error'),
                                               'resume_error': resumed.get('error'),
                                               'old_session_replaced': session != arm.session_id}
                    session = arm.session_id
                now = time.monotonic_ns()
                if args.trajectory == 'reachable':
                    desired = np.zeros(10)
                    desired[[3, 8]] = .08*(1-np.cos(seq*.08))
                    desired[[0, 5]] = -.03*np.sin(seq*.08)
                    targets = fk.palms(desired)
                else:
                    targets = mapper.map_frame(row['frame'])
                if args.inject_ik_failure and seq == 25:
                    targets[0][:3, 3] = [10., 10., 10.]
                values = [v for t in targets for v in
                          t[:3, 3].tolist()+Rotation.from_matrix(t[:3, :3]).as_quat().tolist()]
                packet = envelope(motion._lease(), seq=seq, source_seq=seq, mapping_epoch=1,
                                  generated_ns=now, valid_until_ns=now+300_000_000,
                                  mode='eef_pose', values=values, **motion.versions)
                motion.receive_eef(packet)
                time.sleep(max(0., .05-(time.monotonic()-started)))
                state = arm.status()
                trace.append({'source_seq': seq, 'state': state['state'], 'reason': state['reason'],
                              'applied_sequence': state['applied_sequence'],
                              'decision': dict(motion._decision), 'q': state['feedback']['q']})
            time.sleep(.1)
            report['same_session'] = session == arm.session_id
            report['decisions'] = dict(Counter(x['decision'].get('reason') or x['decision'].get('state') for x in trace))
            report['output_writes'] = len(writes)
            report['accepted_joint_publications'] = len(publications)
            report['publication_source_sequences'] = [p['source_seq'] for p in publications]
            report['mapping_epochs_in_published_targets'] = sorted(set(p['mapping_epoch'] for p in publications))
            report['max_measured_excursion_rad'] = float(np.max(np.abs(np.asarray([x['q'] for x in trace])-initial_q)))
            report['target_publications_after_dropout'] = sum(
                p['source_seq'] >= len(rows)//2 for p in publications)
            report['trace'] = trace
            result = motion.dispatch('finish', {'session_id': arm.session_id, 'secret': arm.secret,
                                                 'operation_id': 'offline-release'})
            deadline = time.monotonic()+5.
            while time.monotonic() < deadline and arm.session_id:
                time.sleep(.02)
            report['release'] = {'initial': result, 'final': arm.status()['release'],
                                 'sdk_actions': sdk, 'ownership_held': arm.status()['ownership_held']}
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    finally:
        if motion:
            motion.stop()
        if arm:
            # Only the test's own plant threads; no SDK was loaded or opened.
            arm._closed.set()
            if arm._thread:
                arm._thread.join(1.)
        stop.set()
        thread.join(1.)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('sources', 'trace')}, indent=2))
    # A failed product integration audit must not print a green acceptance result.
    return 0 if report['product_two_card_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
