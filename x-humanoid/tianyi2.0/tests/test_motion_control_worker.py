"""Offline real Pinocchio solver and OS processes; never opens ROS or hardware."""
import os
import signal
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_motion_control import profile
from common.motion.envelope import MotionEnvelope
from common.motion import worker as ipc
from tianyi_motion.kinematics import TianyiIK
from tianyi_motion.worker import NumericalWorker
from tianyi_motion.workspace import ArmWorkspace


def record(dof=14):
    return {'schema':'motus.motion.envelope/1','boot_id':'b','session_id':'s','seq':2,
        'source_seq':8,'mapping_epoch':1,'model_version':'m','calibration_version':'c',
        'frame':'torso','valid_until_ns':300,'measured_ns':100,
        'lower':[-.2]*dof,'upper':[.2]*dof}


def test_execution_proof_is_immutable_and_fenced_by_identity_and_expiry():
    raw = record(); proof = MotionEnvelope.from_record(raw,dof=14)
    raw['lower'][0] = -100
    cmd = {k:v for k,v in raw.items() if k not in ('schema','lower','upper','measured_ns')}
    assert proof.allows(cmd,[0.]*14,[0.]*14,[.1]*14,now_ns=200)
    assert not proof.allows(cmd,[.3]*14,[0.]*14,[0.]*14,now_ns=200)
    assert not proof.allows(cmd,[0.]*14,[0.]*14,[0.]*14,now_ns=300)
    assert not proof.matches({**cmd,'seq':3},now_ns=200)
    assert proof.lower[0] == -.2


class MixedCornerCollision(ArmWorkspace):
    """A counterexample whose synchronized diagonal is safe, mixed corner is not."""
    def __init__(self):
        self.indices = np.arange(2)
        self.transition_refinement_depth = 8

    def _safe_configuration(self, q, excursion=None):
        radius = np.zeros(2) if excursion is None else excursion
        if q[0]+radius[0] >= .8 and q[1]-radius[1] <= .2:
            raise ValueError('arm_collision')


def test_joint_box_proof_rejects_collision_which_diagonal_samples_miss():
    checker = MixedCornerCollision()
    for t in np.linspace(0,1,101): checker._safe_configuration(np.array([t,t]))
    with pytest.raises(ValueError,match='arm_collision'):
        checker._safe_transition([0.,0.],[1.,1.])


def test_solver_returns_complete_reference_not_one_twenty_ms_step(tmp_path):
    solver = TianyiIK(profile(tmp_path))
    actual = np.zeros(14); desired = np.zeros(14); desired[3] = .4
    q = np.asarray(solver.solve(solver.palms(desired),actual,actual))
    assert np.max(np.abs(q-actual)) > .02*solver.velocity*2
    proof = solver.last_envelope
    assert np.all(np.asarray(proof['lower']) <= actual)
    assert np.all(np.asarray(proof['upper']) >= actual)
    assert np.max(np.asarray(proof['upper'])-np.asarray(proof['lower'])) <= .2*solver.velocity+.004+1e-12


@pytest.fixture
def numerical(tmp_path):
    worker = NumericalWorker(profile(tmp_path))
    yield worker
    worker.close()


def frame(worker, seq=1, epoch=0):
    reference = TianyiIK(worker.path)
    q = [0.]*14
    values = [x for t in reference.palms(q) for x in
        t[:3,3].tolist()+Rotation.from_matrix(t[:3,:3]).as_quat().tolist()]
    now = time.monotonic_ns()
    body = {'boot_id':'boot','session_id':'s','seq':seq,'source_seq':seq,'mapping_epoch':epoch,
        'model_version':worker.profile['urdf_sha256'],'calibration_version':worker.profile_sha256,
        'frame':worker.profile['torso_frame'],'valid_until_ns':now+300_000_000,'values':values}
    state = {'state':'ready','feedback':{'q':q,'dq':q,'arm_ns':now},'commanded_q':q}
    return body,state


def test_real_child_solves_renders_and_produces_verified_motion_box(numerical):
    assert numerical._process.pid != os.getpid()
    body,state = frame(numerical)
    q = numerical.solve_frame(body,state,1)
    np.testing.assert_allclose(q,[0.]*14,atol=1e-8)
    assert numerical.last_envelope['measured_ns'] == state['feedback']['arm_ns']
    state['feedback']['arm_ns'] = time.monotonic_ns()
    rendered = numerical.render(state,{'state':'active'},False,1)
    assert len(rendered['poses']) == 2 and rendered['visualization']['ik']
    display = rendered['visualization']
    assert 'body' not in display and len(display['torso']) == 3
    assert all(len(point) == 3 for line in display['torso'] for point in line)
    np.testing.assert_allclose(display['torso'][0][0], display['measured'][0][0])
    np.testing.assert_allclose(display['torso'][0][3], display['measured'][1][0])
    rendered = numerical.render(state,{'state':'hold'},False,2)
    assert rendered['visualization']['ik'] == []


def test_hung_child_is_bounded_restarted_and_never_replays_trigger(numerical):
    body,state = frame(numerical)
    old = numerical._process
    os.kill(old.pid,signal.SIGSTOP)
    started = time.monotonic()
    with pytest.raises(ValueError,match='ik_worker_unavailable'):
        numerical.solve_frame(body,state,1)
    assert time.monotonic()-started < 1.
    assert old.poll() is not None
    with pytest.raises(ValueError,match='fresh_input_required'):
        numerical.solve_frame(body,state,1)
    body,state = frame(numerical,seq=2)
    assert len(numerical.solve_frame(body,state,2)) == 14


def test_result_identity_mismatch_discards_child(numerical,monkeypatch):
    body,state = frame(numerical)
    original = ipc.receive
    def altered(sock):
        result = original(sock); result['identity']['mapping_epoch'] += 1; return result
    monkeypatch.setattr(ipc,'receive',altered)
    with pytest.raises(ValueError,match='identity_mismatch'): numerical.solve_frame(body,state,1)
    assert numerical._process is None


def test_stop_interrupts_child_without_waiting_for_ik_deadline(numerical):
    body,state = frame(numerical)
    os.kill(numerical._process.pid,signal.SIGSTOP)
    errors = []
    def call():
        try: numerical.solve_frame(body,state,1)
        except ValueError as exc: errors.append(str(exc))
    thread = threading.Thread(target=call); thread.start(); time.sleep(.02)
    started = time.monotonic(); numerical.interrupt(); thread.join(.5)
    assert not thread.is_alive() and errors and time.monotonic()-started < .5


def test_safe_short_advance_survives_collision_in_larger_full_reference_box(tmp_path,monkeypatch):
    from tianyi_motion.workspace import WorkspaceViolation
    solver=TianyiIK(profile(tmp_path));checked=[]
    original=solver._safe_transition
    def boundary(a,b,budget=lambda:None):
        checked.append(float(max(np.max(a),np.max(b))))
        if max(np.max(a),np.max(b))>.03:raise WorkspaceViolation('torso_collision',.01)
        return original(a,b,budget)
    monkeypatch.setattr(solver,'_safe_transition',boundary)
    proof=solver.motion_envelope(np.zeros(14),np.zeros(14),np.ones(14)*.2)
    assert any(x>.03 for x in checked)
    assert .02<=max(proof['upper'])<=.03
    assert all(lo<=0<=hi for lo,hi in zip(proof['lower'],proof['upper']))


def test_unsafe_short_segment_is_not_accepted_when_all_backoffs_collide(tmp_path,monkeypatch):
    from tianyi_motion.workspace import WorkspaceViolation
    solver=TianyiIK(profile(tmp_path))
    def collision(*args,**kwargs):raise WorkspaceViolation('torso_collision',.01)
    monkeypatch.setattr(solver,'_safe_transition',collision)
    with pytest.raises(ValueError,match='torso_collision'):
        solver.motion_envelope(np.zeros(14),np.zeros(14),np.ones(14)*.2)
