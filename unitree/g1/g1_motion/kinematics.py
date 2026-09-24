"""Calibrated G1_23 dual-arm solver migrated from the frozen ActuCore path.

Numerical subprocess only: preserve the PR152 five-DOF objective and conservative
geometry, output complete position reference; execution interpolation and final-q
RNEA belong to arm. See NOTICE.md for exact source provenance.
"""
import copy
import hashlib
from itertools import product
import json
from pathlib import Path
import time
import numpy as np
from .g1_ik import G123PinocchioIk, ARM_JOINT_NAMES
from .g1_collision import G1Collision
from .workspace import ArmWorkspace

PROFILE_ID = 'unitree_g1_23_dual_arm_relative_v1'


def finite(value, shape):
    raw = np.asarray(value)
    if raw.dtype.kind not in 'iuf': raise ValueError('invalid_numeric_type')
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all(): raise ValueError('invalid_finite_shape')
    return result


def transform(pose):
    from scipy.spatial.transform import Rotation
    result = np.eye(4)
    result[:3,3] = finite(pose['position'], (3,))
    quaternion = finite(pose['orientation'], (4,))
    if abs(float(quaternion@quaternion)-1) > .002: raise ValueError('quaternion_not_unit')
    result[:3,:3] = Rotation.from_quat(quaternion).as_matrix()
    return result


class G1IK(ArmWorkspace):
    def __init__(self, path, *, pr152_objective=True, collision_checks=False):
        self.collision_checks_enabled = bool(collision_checks)
        raw = Path(path).read_bytes()
        self.profile = json.loads(raw)
        self.hands_enabled = False
        self.profile_sha256 = hashlib.sha256(raw).hexdigest()
        p = self.profile
        if p.get('schema') != 'motus.g1-calibration.v1' or p.get('profile_id') != PROFILE_ID:
            raise ValueError('g1_calibration_schema')
        if p.get('arm_joint_names') != list(ARM_JOINT_NAMES):
            raise ValueError('g1_joint_order')
        model_path = Path(p['urdf_path'])
        if not model_path.is_absolute(): model_path = Path(path).parent / model_path
        if hashlib.sha256(model_path.read_bytes()).hexdigest() != p['urdf_sha256']:
            raise ValueError('calibration_model_changed')
        self.ik = G123PinocchioIk(model_path, palm_frames=p['palm_frames'], locked_joints=p['locked_joints'], pr152_objective=pr152_objective)
        self.pin, self.model = self.ik._pin, self.ik._model
        self.data = self.model.createData()
        self.indices = np.arange(10)
        self.frames = [self.model.getFrameId(n) for n in ('L_ee','R_ee')]
        self.torso = self.model.getFrameId(p['torso_frame'])
        if self.torso >= self.model.nframes: raise ValueError('torso_frame_missing')
        self.velocity = p['joint_velocity_rad_s']
        if type(self.velocity) not in (int,float) or not 0 < self.velocity <= 5.:
            raise ValueError('joint_velocity_limit')
        if self.collision_checks_enabled:
            if not p['workspace'].get('capsules') or not p['workspace'].get('torso_box'):
                raise ValueError('collision_calibration_missing')
            # Official G1_23 rubber-hand mesh bounds, rounded outwards (see VALIDATION.md).
            # Bind these dimensions to the verified model, not an arbitrary replacement URDF.
            if p['urdf_sha256'] not in (
                'b1af86fb023c0b6f8e52723d224be6cad70916eaff2778e7dbe09e6f91faa9b9',
                '2d61264aaae95eac2545497a34ef316f270b0cd3d856bf9a4941ebff2592aca2',
            ):
                raise ValueError('g1_hand_geometry_unverified')
            for side, ee in (('left','L_ee'), ('right','R_ee')):
                y = (-.042408,.029961) if side == 'left' else (-.029961,.042408)
                corners = np.array(list(product((.000097,.253316),y,(-.041527,.064957))))
                tip = finite(p['palm_frames'][side]['position'],(3,))
                length2 = float(tip @ tip)
                projection = np.zeros(8) if length2 == 0 else np.clip(corners @ tip / length2,0,1)
                radius = float(np.max(np.linalg.norm(corners-projection[:,None]*tip,axis=1)))
                # A capsule is convex: enclosing all box corners encloses the complete mesh.
                p['workspace']['capsules'].append({'from':side+'_wrist_roll_joint',
                    'to':ee,'radius_m':radius,'group':side})
            self.transition_refinement_depth = 10
            self.configure_workspace()
            self.body_collision = G1Collision(self.pin,self.model,model_path)
        self.last_collision_rejection = None
        self._collision_context = None
        self.last_ms = None
        self.residual = None
        self.last_solve_at = None
        self.visualization_sample = None
        self.last_valid_visualization = None
        self.last_envelope = None

    def _safe_configuration(self,q,excursion=None):
        q = finite(q, (10,))
        if np.any(q < self.model.lowerPositionLimit) or np.any(q > self.model.upperPositionLimit):
            raise ValueError('motion_envelope_joint_limit')
        if not self.collision_checks_enabled: return
        try:
            super()._safe_configuration(q,excursion)
            self.body_collision.check(self.data,excursion)
        except ValueError as exc:
            self.last_collision_rejection = {
                'reason':str(exc), 'checked_q':np.asarray(q).tolist(),
                'excursion_rad':None if excursion is None else np.asarray(excursion).tolist(),
                'context':copy.deepcopy(self._collision_context),
                'body':copy.deepcopy(self.body_collision.last_rejection) if str(exc).startswith('g1_body_collision:') else None}
            raise

    def palms(self, q):
        return self.ik.current_targets(finite(q,(10,)))

    def self_test(self, q):
        q = finite(q,(10,))
        self._safe_configuration(q)
        result = self.ik.warm_up(q, np.zeros(10))
        self.ik.reset(q)
        return {'state':'ready','hardware_output':False, **result, 'profile_id':PROFILE_ID,
                'calibration_sha256':self.profile_sha256}

    def motion_envelope(self, measured, previous, target, budget=lambda: None):
        measured, previous, target = (finite(v, (10,)) for v in (measured, previous, target))
        # Authorize the actual endpoint, not a speed-clipped partial target.
        # The servo-position executor sends this endpoint unchanged.
        near = target
        lo, hi = np.minimum(np.minimum(measured, previous), near), np.maximum(np.maximum(measured, previous), near)
        if np.any(lo < self.model.lowerPositionLimit) or np.any(hi > self.model.upperPositionLimit):
            raise ValueError('motion_envelope_joint_limit')
        lo = np.maximum(self.model.lowerPositionLimit, lo-.002)
        hi = np.minimum(self.model.upperPositionLimit, hi+.002)
        self._collision_context = {'measured_q': measured.tolist(), 'reference_q': previous.tolist(),
                                   'target_q': target.tolist()}
        if self.collision_checks_enabled:
            self._safe_transition(lo, hi, budget)
        budget()
        return {'lower': lo.tolist(), 'upper': hi.tolist()}

    def solve(self, targets, measured, commanded=None, *, deadline_monotonic=None):
        begin = time.monotonic()
        end = begin+.3 if deadline_monotonic is None else deadline_monotonic
        def budget():
            if time.monotonic() >= end: raise ValueError('ik_timeout')
        budget()
        q0 = finite(measured, (10,))
        try:
            # PR152's weighted five-DOF pose objective is deliberately retained.
            # No new arbitrary six-DOF residual threshold is introduced.
            q, _ = self.ik.solve(*targets, q0, np.zeros(10))
            budget()
            actual = self.palms(q)
            self.residual = [{'position_m': float(np.linalg.norm(a[:3,3]-b[:3,3])),
                'orientation_rad': float(np.linalg.norm(self.pin.log3(b[:3,:3].T@a[:3,:3])))}
                for a,b in zip(actual, targets)]
            previous = q0 if commanded is None else finite(commanded, (10,))
            self.last_envelope = self.motion_envelope(q0, previous, q, budget)
            self.last_ms = (time.monotonic()-begin)*1000
            sample = {'monotonic_ns': time.monotonic_ns(),
                      'targets': [np.asarray(t).tolist() for t in targets], 'ik_q': q.tolist()}
            self.visualization_sample = self.last_valid_visualization = sample
            return q.tolist()
        except Exception as exc:
            # The numerical filter was updated before envelope validation.
            # Rejected targets must never influence the next valid solution.
            self.ik.reset(q0)
            self.visualization_sample = {'monotonic_ns': time.monotonic_ns(),
                'targets': [np.asarray(t).tolist() for t in targets], 'ik_q': None, 'error': str(exc)}
            raise
