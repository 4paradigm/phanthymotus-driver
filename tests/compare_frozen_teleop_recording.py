"""Offline numerical A/B from a private recording, with no ROS or hardware.

Use extracted immutable frozen sources. The recorded measured joint state is
the common seed for both solvers, not a simulated instantaneous response.
This isolates mapping/IK and cannot establish execution or physical tracking.
"""
import argparse
from collections import Counter
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time
from types import ModuleType

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frozen', type=Path, required=True)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'summary.json').exists():
        raise ValueError('use a new evidence directory')
    package = ModuleType('frozen_numerics')
    package.__path__ = [str(args.frozen.resolve())]
    sys.modules[package.__name__] = package
    frozen = importlib.import_module('frozen_numerics.kinematics')
    driver = Path(__file__).resolve().parents[1] / 'x-humanoid/tianyi2.0'
    sys.path[:0] = [str(driver), str(driver.parents[1])]
    from teleop_control import RelativeMapping
    from tianyi_motion.kinematics import TianyiIK

    profile = json.loads(args.profile.read_text())
    assert profile['urdf_sha256'] == sha(args.urdf)
    profile['urdf_path'] = str(args.urdf.resolve())
    profile['joint_velocity_rad_s'] = 1.
    local_profile = args.output / 'offline-profile.json'
    local_profile.write_text(json.dumps(profile))
    old, new = frozen.TianyiIK(local_profile), TianyiIK(local_profile)
    rows = [json.loads(line) for line in args.record.read_text().splitlines()]
    valid = [row for row in rows if all(row['frame']['tracking'].values())]
    assert valid, 'record has no tracked frames'
    first = valid[0]
    old_map, new_map = frozen.RelativeMapping(.5), RelativeMapping(.5)
    offsets = profile.get('controller_to_palm', {})
    old_map.controller_offsets = {s: frozen.transform(offsets.get(s, {
        'position': [0., 0., 0.], 'orientation': [0., 0., 0., 1.]}))
        for s in ('left', 'right')}
    new_map.set_controller_offsets(offsets)
    basis = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])

    def normalize(frame):
        def pose(value):
            q = np.array(value['orientation'])
            q = q / np.linalg.norm(q)
            return {'position': (basis @ value['position']).tolist(),
                    'orientation_xyzw': [*(basis @ q[:3]), q[3]]}
        return {'head_reference': pose(frame['head']),
                **{s: pose(frame[s + '_controller']) for s in ('left', 'right')}}

    palms = old.palms(first['driver']['feedback']['q'])
    old_map.reset(first['frame'], palms)
    new_map.calibrate(normalize(first['frame']), [
        [*p[:3, 3], *Rotation.from_matrix(p[:3, :3]).as_quat()] for p in palms])
    counts = {'frozen': Counter(), 'candidate': Counter()}
    times = {'frozen': [], 'candidate': []}
    mapping_errors, ik_errors, paired = [], [], 0
    with (args.output / 'frames.jsonl').open('w') as stream:
        for row in valid:
            frame = row['frame']
            before = old_map.targets(frame)
            flat = new_map.targets(normalize(frame))
            after = [frozen.transform({'position': flat[i:i+3],
                      'orientation': flat[i+3:i+7]}) for i in (0, 7)]
            mapping_error = float(max(np.max(np.abs(a-b)) for a, b in zip(before, after)))
            mapping_errors.append(mapping_error)
            sample = {'input_sequence': frame['sequence'], 'mapping_matrix_max_abs': mapping_error}
            results = []
            for name, solver, target in [('frozen', old, before), ('candidate', new, after)]:
                # Identical cold numerical seed avoids letting a different
                # wall-clock/cache age masquerade as an algorithm difference.
                solver.last_valid_visualization = None
                begin = time.monotonic()
                try:
                    solver.solve(target, row['driver']['feedback']['q'])
                    result = np.asarray(solver.visualization_sample['ik_q'])
                    outcome = 'solved'
                except ValueError as exc:
                    result, outcome = None, str(exc)
                elapsed = (time.monotonic()-begin)*1000
                counts[name][outcome] += 1
                times[name].append(elapsed)
                sample[name] = {'outcome': outcome, 'elapsed_ms': elapsed}
                results.append(result)
            if all(result is not None for result in results):
                paired += 1
                error = float(np.max(np.abs(results[0]-results[1])))
                ik_errors.append(error)
                sample['ik_max_abs_difference_rad'] = error
            stream.write(json.dumps(sample, allow_nan=False)+'\n')
    summary = {'record_sha256': sha(args.record), 'original_profile_sha256': sha(args.profile),
        'urdf_sha256': sha(args.urdf), 'frozen_kinematics_sha256': sha(args.frozen/'kinematics.py'),
        'candidate_kinematics_sha256': sha(driver/'tianyi_motion/kinematics.py'),
        'recorded_frames': len(rows), 'tracked_frames': len(valid), 'paired_solved_frames': paired,
        'mapping_max_abs': max(mapping_errors), 'ik_max_abs_difference_rad': max(ik_errors, default=None),
        'outcomes': {k: dict(v) for k, v in counts.items()},
        'solve_ms': {k: {'p50': float(np.percentile(v, 50)), 'p95': float(np.percentile(v, 95)),
                         'max': max(v)} for k, v in times.items()},
        'limits': 'Recorded measured seed; numerical A/B only, no physical execution or follow-error acceptance.'}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
