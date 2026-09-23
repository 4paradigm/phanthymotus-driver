"""Offline mapping/execution A/B against an explicit Git baseline; no SDK/ROS.

Recorded OpenXR controller poses are converted to the current wire axes. Recorded
joint samples separately exercise execution interpolation, not an IK replay or
physical feedback plant. Numerical-model coverage is in test_motion_control_numeric.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT/'unitree/g1'
sys.path[:0] = [str(ROOT), str(DRIVER)]
from common.motion.protocol import envelope


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def converted(frame):
    result = {}
    for target, source in [('head_reference', 'head'), ('left', 'left_controller'), ('right', 'right_controller')]:
        pose = frame[source]; p, q = pose['position'], pose['orientation']
        result[target] = {'position': [-p[2], -p[0], p[1]],
                          'orientation_xyzw': [-q[2], -q[0], q[1], q[3]]}
    return result


def execute(module, profile, rows):
    now = [1_000_000_000]; output = []
    feedback = {'q': list(rows[0]['q']), 'dq': [0.]*10, 'arm_ns': now[0]}
    channel = SimpleNamespace(open=lambda: None, close=lambda: None,
        publish_servo_position=lambda q, weight: output.append(list(q)) or True)
    arm = module.ArmStreamExecutor({'servo_position': True, 'live_enabled': True}, 'offline',
        SimpleNamespace(), snapshot=lambda: feedback, channel=channel, clock=lambda: now[0])
    arm.configure_profile(str(profile)); arm._prepared = True
    assert arm.dispatch('claim', {})['state'] == 'ready'
    for seq, row in enumerate(rows):
        now[0] += 20_000_000; feedback['arm_ns'] = now[0]
        body = envelope({'boot_id': arm.boot_id, 'session_id': arm.session_id, 'secret': arm.secret},
            seq=seq, source_seq=seq, mapping_epoch=1, generated_ns=now[0], valid_until_ns=now[0]+300_000_000,
            mode='joint_position', dof=10, values=row['q'], **arm.versions)
        proof = {k: body[k] for k in ('boot_id','session_id','seq','source_seq','mapping_epoch',
                 'model_version','calibration_version','frame','valid_until_ns')}
        proof.update(schema='motus.motion.envelope/1', measured_ns=now[0],
                     lower=[v[0] for v in arm.limits], upper=[v[1] for v in arm.limits])
        arm.install_motion_envelope(proof); assert arm.accept_control(body)
        arm.tick()
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--recording', required=True, type=Path)
    parser.add_argument('--profile', required=True, type=Path)
    args = parser.parse_args()
    baseline = subprocess.check_output(['git','rev-parse',args.baseline], cwd=ROOT, text=True).strip()
    rows = json.loads(args.recording.read_text())
    with tempfile.TemporaryDirectory(prefix='g1-teleop-ab-') as temp:
        temp = Path(temp); old = {}
        for name in ('teleop_control', 'arm_stream'):
            path = temp/(name+'.py')
            path.write_bytes(subprocess.check_output(['git','show',baseline+':unitree/g1/'+name+'.py'], cwd=ROOT))
            old[name] = load('baseline_'+name, path)
        current = {name: load('candidate_'+name, DRIVER/(name+'.py')) for name in old}
        profile = json.loads(args.profile.read_text())
        mappings = [version['teleop_control'].RelativeMapping(1.) for version in (old,current)]
        origin = [[.1,.2,.3,0.,0.,0.,1.], [.1,-.2,.3,0.,0.,0.,1.]]
        for mapping in mappings:
            mapping.set_controller_offsets(profile['controller_to_palm'])
            mapping.calibrate(converted(rows[0]['frame']), origin)
        mapping_delta = max(abs(a-b) for row in rows for a,b in zip(
            mappings[0].targets(converted(row['frame'])), mappings[1].targets(converted(row['frame']))))
        outputs = [execute(version['arm_stream'], args.profile, rows) for version in (old,current)]
        assert len(outputs[0]) == len(outputs[1]) == len(rows)
        execution_delta = max(abs(a-b) for x,y in zip(*outputs) for a,b in zip(x,y))
        assert mapping_delta == execution_delta == 0.
        print(json.dumps({'baseline':baseline,'recording_sha256':hashlib.sha256(args.recording.read_bytes()).hexdigest(),
            'profile_sha256':hashlib.sha256(args.profile.read_bytes()).hexdigest(), 'frames':len(rows),
            'mapping_max_abs_difference':mapping_delta, 'execution_max_abs_difference':execution_delta,
            'hardware_output':False, 'ik_replay':False, 'fixed_dt_ms':20}))


if __name__ == '__main__': main()
