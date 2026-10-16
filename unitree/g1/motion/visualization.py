"""Read-only kinematic telemetry. Never claims authority or sends motion commands."""
import math
import time
import numpy as np


def chains(solver, q):
    q = np.asarray(q, dtype=float)
    if q.shape != (10,) or not np.isfinite(q).all():
        raise ValueError('visualization_invalid_q')
    data = solver.model.createData()
    solver.pin.framesForwardKinematics(solver.model, data, q)
    return [[data.oMf[solver.model.getFrameId(name)].translation.tolist() for name in
             [side+'_'+joint+'_joint' for joint in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll')] + [ee]]
            for side,ee in (('left','L_ee'),('right','R_ee'))]


def snapshot(adapter):
    # Published sample/output objects are replaced atomically, never edited.
    # Display must remain available while the execution owner holds its lock.
    # FK uses immutable model geometry and its own Data, not solver.data.
    try:
        solver = adapter.solver
        output = adapter.output
        if solver is None: return {'schema':'motus.g1-visualization.v1','available':False,'reason':'calibration_missing'}
        state = adapter.link.feedback()
        feedback = state.get('feedback', {})
        now = time.monotonic_ns()
        age = (now-feedback.get('arm_ns',0))/1e6
        fresh = 0 <= age <= getattr(adapter.link,'feedback_max_age_ns',100_000_000)/1e6
        preview = solver.visualization_sample
        intent_age = (now-preview['monotonic_ns'])/1e6 if preview else None
        alignment = {'state': 'unavailable', 'ready': False}
        valid = preview and preview.get('ik_q') is not None and intent_age is not None and 0 <= intent_age < 300
        if fresh and preview and intent_age is not None and 0 <= intent_age < 300:
            measured = solver.palms(feedback['q'])
            target = solver.palms(preview['ik_q']) if valid else [np.asarray(t) for t in preview['targets']]
            errors = {}
            for side, a, b in zip(('left', 'right'), measured, target):
                errors[side] = {'position_error_m': float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                    'orientation_error_rad': float(np.linalg.norm(solver.pin.log3(a[:3, :3].T@b[:3, :3]))),
                    'position_tolerance_m': .03, 'orientation_tolerance_rad': math.radians(15)}
            ready = all(e['position_error_m'] <= .03 and e['orientation_error_rad'] <= math.radians(15)
                        for e in errors.values())
            alignment = {'state': ('ready' if ready else 'unaligned') if valid else 'unreachable',
                         'ready': bool(valid and ready), **errors,
                         'error_reference': 'ik' if valid else 'requested_target'}
        targets = [np.asarray(t)[:3,3].tolist() for t in preview['targets']] if preview else []
        # Model-based orientation reference, not a body mesh or collision proof.
        neutral = chains(solver, np.zeros(10))
        left, right = neutral[0][0], neutral[1][0]
        height = (left[2]+right[2])/2
        torso = [[left, [0, left[1], 0], [0, right[1], 0], right, left],
                 [[0, 0, 0], [0, 0, height+.06]],
                 [[0, -.06, height+.06], [0, .06, height+.06],
                  [0, .06, height+.20], [0, -.06, height+.20], [0, -.06, height+.06]]]
        return {'schema':'motus.g1-visualization.v1','available':True,
                'mode':'live' if adapter.hardware_output else 'shadow',
                'state':output.get('state','idle'),
                'reason':output.get('code') or state.get('reason') or '',
                'feedback_fresh':fresh,'feedback_age_ms':max(0.,age),
                'feedback_source':state.get('feedback_source','teleop_executor'),
                'source_timestamp_available':state.get('source_timestamp_available',True),
                'intent_age_ms':intent_age if intent_age is not None else 1e12,'solve_ms':solver.last_ms,
                'alignment': alignment, 'target_residual': solver.residual,
                'measured':chains(solver,feedback['q']) if fresh else [],
                'ik':chains(solver,preview['ik_q']) if valid else [],
                'command':chains(solver,output['target_q']) if output.get('target_q') is not None else [],
                'targets':targets,
                'torso':torso,'shoulder_height':height,
                'collision_reason':(solver.last_collision_rejection or {}).get('reason','')}
    except (ValueError,KeyError,TypeError):
        return {'schema':'motus.g1-visualization.v1','available':False,'reason':'feedback_unavailable'}
