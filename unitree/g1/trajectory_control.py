"""G1 recording/replay operations for downstream teaching adapters; no UI."""
import copy
import hashlib
import json
import math
from pathlib import Path
import secrets
import threading
import time

from common.motion.protocol import envelope
from common.motion.envelope import IDENTITY
from trajectory import Recorder, list_recordings, read_recording


class TrajectoryControl:
    START_SPEED = .1

    def __init__(self, control, notify=None):
        self.control, self.motion, self.arm = control, control.motion, control.executor
        self.directory = Path(control.cfg.get('data_dir', '/opt/phanthy-motus/data/g1'))/'recordings'
        self.notify = notify or (lambda *args: None)
        self.lock = threading.RLock()
        self.recorder = self.thread = None
        self.cancel = threading.Event()
        self.last_action = None

    @property
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def binding(self):
        machine = self.control.cfg.get('trajectory_device_id')
        if not machine:
            path = Path('/run/g1-machine-id')
            if not path.is_file(): raise ValueError('trajectory_device_id_missing')
            machine = path.read_text().strip()
        if not isinstance(machine, str) or not machine or len(machine) > 256:
            raise ValueError('trajectory_device_id_invalid')
        profile = copy.deepcopy(self.arm.profile)
        if not profile: raise ValueError('calibration_missing')
        locked = profile.pop('locked_joints', {})
        for key in ('baseline_sample_ns', 'urdf_path'): profile.pop(key, None)
        return {'device_id': hashlib.sha256(machine.encode()).hexdigest(),
                'model_sha256': profile['urdf_sha256'],
                'geometry_sha256': hashlib.sha256(json.dumps(profile, sort_keys=True, allow_nan=False).encode()).hexdigest(),
                'locked_joints': locked}

    def _check_binding(self, recording):
        current, stored = self.binding(), recording['binding']
        if any(current[k] != stored.get(k) for k in ('device_id', 'model_sha256', 'geometry_sha256')):
            raise ValueError('recording_binding_mismatch')
        if recording['joint_names'] != self.arm.profile['arm_joint_names']:
            raise ValueError('recording_joint_order')
        baseline = stored.get('locked_joints')
        if not isinstance(baseline, dict) or not baseline: raise ValueError('recording_body_baseline_missing')
        sample = self.arm._fresh()[0]
        measured = sample.get('locked_joints', {})
        model_baseline = current['locked_joints']
        if set(measured) != set(baseline) or set(model_baseline) != set(baseline) or any(type(v) not in (int, float) for v in baseline.values()):
            raise ValueError('recording_body_baseline_mismatch')
        if any(not math.isfinite(baseline[n]) or not math.isfinite(measured[n]) or abs(measured[n]-baseline[n]) > .02
               or abs(model_baseline[n]-baseline[n]) > .02
               for n in baseline): raise ValueError('recording_body_baseline_mismatch')

    def info(self):
        return {'recording': self.recorder.status() if self.recorder else None,
                'replay_active': self.busy, 'last_action': copy.deepcopy(self.last_action)}

    def record_start(self):
        with self.lock:
            if self.busy: raise ValueError('replay_active')
            if self.recorder and self.recorder.thread.is_alive(): raise ValueError('recording_busy')
            if not self.control.can_record(): raise ValueError('live_mapped_teleop_required')
            self.arm._fresh()
            binding = self.binding()
            if not binding['locked_joints']: raise ValueError('recording_body_baseline_missing')
            recorder = Recorder(self.directory, binding, self.arm.profile['arm_joint_names'], self.arm.limits,
                                clock=self.control.clock)
            with self.arm.lock:
                if not self.control.can_record():
                    recorder.fail('teleop_changed'); recorder.stop()
                    raise ValueError('live_mapped_teleop_required')
                self.recorder = recorder
                self.arm.record_sink = recorder
            return recorder.status()

    def record_stop(self, reason=None):
        # Stop must not wait for play's file validation / model preparation lock.
        recorder = self.recorder
        if not recorder: return {'recording': False, 'complete': False, 'reason': 'no_recording'}
        with self.arm.lock:
            if self.arm.record_sink is recorder: self.arm.record_sink = None
        if reason and recorder.thread.is_alive(): recorder.fail(reason)
        return recorder.stop()

    def inspect(self, filename):
        recording = read_recording(self.directory, filename, self.arm.limits)
        self._check_binding(recording)
        frames = recording['frames']
        step = max(1, (len(frames)+179)//180)
        sampled = frames[::step]
        if sampled[-1] is not frames[-1]: sampled.append(frames[-1])
        return {**{k: recording[k] for k in ('filename', 'sha256', 'binding', 'joint_names', 'duration_seconds')},
                'frames': len(frames), 'complete': True,
                'preview': [{'t': f['t_ns']/1e9, 'kind': f['kind'], 'target_q': f['target_q'],
                             'feedback_q': f['feedback_q']} for f in sampled]}

    def start_difference(self, filename):
        recording = read_recording(self.directory, filename, self.arm.limits)
        self._check_binding(recording)
        return self._difference(recording)

    def _difference(self, recording):
        target = recording['frames'][0]['target_q']
        _, q, dq = self.arm._fresh()
        delta = [a-b for a,b in zip(q, target)]
        return {'ready': max(map(abs, dq)) <= .02,
                'target_q': target, 'current_q': q, 'joint_delta_rad': delta}

    def _idle(self):
        if self.control._operator: raise ValueError('stop_teleop_before_replay')
        if self.busy: raise ValueError('replay_active')
        if self.recorder and self.recorder.thread.is_alive(): raise ValueError('recording_busy')
        state = self.arm.status()
        if (state['ownership_held'] or self.arm.opening or self.control._begin_lock.locked()
                or self.control._reset_operation): raise ValueError('motion_busy')
        if state['state'] == 'fault': raise ValueError('driver_fault')

    def play(self, args):
        # Never turn an older client's explicit dry run into motion.
        if args.get('execute') is False: raise ValueError('use_inspect_before_replay')
        with self.lock:
            self._idle()
            generation = self.control._generation
            self.motion.start()
            if self.motion.solver is None: self.motion.calibrate()
            recording = read_recording(self.directory, args.get('filename'), self.arm.limits)
            self._check_binding(recording)
            if recording['frames'][0]['kind'] != 'target': raise ValueError('recording_initial_target_missing')
            ident = secrets.token_hex(16)
            with self.control._lock:
                if generation != self.control._generation: raise ValueError('replay_cancelled')
                self._idle()
                self.control._closed.set()
                self.control._generation += 1
                self.cancel.clear()
                self.last_action = {'action_id': ident, 'status': 'running', 'phase': 'moving_to_start', 'result': {}}
                self.thread = threading.Thread(target=self._run, args=(ident, recording), name='g1-replay', daemon=True)
                self.thread.start()
            return {'action_id': ident, 'status': 'running'}

    def _wait_until(self, deadline):
        while True:
            if self.cancel.is_set(): raise ValueError('replay_cancelled')
            now = self.control.clock()
            with self.arm.lock:
                if (not self.arm.session_id or self.arm.state == 'fault' or self.arm.release_requested or
                        self.arm.state == 'hold' and self.arm.reason != 'replay_pause'):
                    raise ValueError('replay_interrupted')
                self.arm._fresh()
                self.arm.replay_deadline_ns = now+100_000_000
            remaining = (deadline-now)/1e9
            if remaining <= 0:
                if remaining < -.05: raise ValueError('replay_schedule_missed')
                return
            self.cancel.wait(min(.02, remaining))

    def _send_frame(self, frame, seq, due):
        state, q = self.motion._fresh()
        proof = self.motion.solver.joint_envelope(q, state['commanded_q'], frame['target_q'])
        self._wait_until(due)
        now = self.control.clock()
        command = envelope(self.motion._lease(), seq=seq, source_seq=frame['source_sequence'],
            mapping_epoch=frame['mapping_epoch'], generated_ns=now, valid_until_ns=now+100_000_000,
            mode='joint_position', dof=10, values=frame['target_q'], **self.motion.versions)
        record = {**{k: command[k] for k in IDENTITY}, 'schema': 'motus.motion.envelope/1',
                  **proof, 'measured_ns': state['feedback']['arm_ns']}
        if self.cancel.is_set(): raise ValueError('replay_cancelled')
        self.arm.write_replay(command, record)

    def _move_to_start(self, recording):
        first, seq = recording['frames'][0], 0
        target = first['target_q']
        while self.arm.last_q != target:
            self._wait_until(self.control.clock()+20_000_000)
            self._check_binding(recording)
            # Bound each reference step by elapsed time; never catch up after a delay.
            dt = max(0., min((self.control.clock()-self.arm.last_emit)/1e9, .05))
            step = self.START_SPEED*dt
            values = [t if abs(t-p) <= step else p+math.copysign(step, t-p)
                      for p,t in zip(self.arm.last_q, target)]
            self._send_frame(dict(first, target_q=values), seq, self.control.clock())
            seq += 1
        return seq

    def _run(self, ident, recording):
        status, result = 'error', {}
        try:
            if self.cancel.is_set(): raise ValueError('replay_cancelled')
            self._check_binding(recording)
            prepared = self.motion.dispatch('prepare_operator_session', {})
            if prepared.get('error'): raise ValueError(prepared['error'])
            if self.cancel.is_set(): raise ValueError('replay_cancelled')
            claimed = self.motion.dispatch('claim', {})
            if claimed.get('error'): raise ValueError(claimed['error'])
            next_seq = self._move_to_start(recording)
            self.last_action['phase'] = 'replaying'
            start = self.control.clock()+50_000_000
            for seq, frame in enumerate(recording['frames'], start=next_seq):
                self._check_binding(recording)
                due = start+frame['t_ns']
                self._wait_until(max(start, due-60_000_000, self.control.clock()))
                if frame['kind'] == 'hold':
                    self._wait_until(due)
                    self.arm.hold('replay_pause')
                    continue
                if self.arm.state == 'hold':
                    resumed = self.motion.dispatch('resume', self.motion._lease())
                    if resumed.get('error'): raise ValueError(resumed['error'])
                self._send_frame(frame, seq, due)
            self._wait_until(start+recording['duration_ns'])
            # Observe a fresh final sample; time elapsed alone is not completion.
            end = self.control.clock()+2_000_000_000
            last_write = self.arm.last_emit
            while True:
                self._wait_until(self.control.clock()+20_000_000)
                sample, q, dq = self.arm._fresh()
                if sample['arm_ns'] > last_write and max(abs(a-b) for a,b in zip(q, recording['frames'][-1]['target_q'])) <= .02 and max(map(abs, dq)) <= .02:
                    break
                if self.control.clock() >= end: raise ValueError('replay_final_feedback_unconfirmed')
            status = 'completed'
            result = {'filename': recording['filename'], 'sha256': recording['sha256'], 'frames': len(recording['frames'])}
        except Exception as exc:
            status = 'cancelled' if self.cancel.is_set() else 'error'
            result = {'reason': str(exc)}
        finally:
            try:
                stopped = self.control._stop_motion()
                result['stop'] = stopped
            except Exception as exc:
                status = 'error'; result['stop_error'] = str(exc)
            with self.arm.lock: self.arm.replay_deadline_ns = None
            self.last_action = {'action_id': ident, 'status': status, 'result': result}
            self.notify(ident, status, result)

    def stop(self):
        self.cancel.set()

    def dispatch(self, action, args):
        if action == 'record_start': return self.record_start()
        if action == 'record_stop': return self.record_stop()
        if action == 'recordings': return {'recordings': list_recordings(self.directory)}
        if action == 'inspect': return self.inspect(args.get('filename'))
        if action == 'start_difference': return self.start_difference(args.get('filename'))
        if action == 'play': return self.play(args)
        if action == 'action_status':
            if not self.last_action or args.get('action_id') != self.last_action['action_id']:
                raise ValueError('unknown_action_id')
            return copy.deepcopy(self.last_action)
        raise ValueError('unknown_action')
