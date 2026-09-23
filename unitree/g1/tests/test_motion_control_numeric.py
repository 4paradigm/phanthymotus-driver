"""Real G1_23 model/solver tests. Needs the pinned CasADi-capable ABI and meshes.

Calibration is deliberately synthetic and cannot be used for physical acceptance.
"""
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
import pytest

pytest.importorskip('pinocchio.casadi', reason='G1 requires CasADi-enabled Pinocchio 3.1 ABI')
DRIVER=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(DRIVER))
from g1_motion.kinematics import G1IK
from g1_motion.worker import NumericalWorker


def profile(tmp_path):
    root=DRIVER/'g1_motion'
    p=json.loads((root/'calibration.example.json').read_text())
    p['urdf_path']=str(root/'models/g1_body23.urdf')
    assert hashlib.sha256(Path(p['urdf_path']).read_bytes()).hexdigest()==p['urdf_sha256']
    p['version']='SYNTHETIC-NOT-ACCEPTED'
    p['locked_joints']=dict.fromkeys(p['locked_joints'],0.)
    for side in ('left','right'):
        p['palm_frames'][side]={'position':[.2,0.,0.],'orientation':[0.,0.,0.,1.]}
    p['workspace']={'torso_box':[[-.05,-.05,-.05],[.05,.05,.05]],
        'left':[[-1.,-1.,-1.],[1.,1.,1.]],'right':[[-1.,-1.,-1.],[1.,1.,1.]],
        'capsules':[{'from':s+'_elbow_joint','to':s+'_wrist_roll_joint','radius_m':.025,'group':s}
                    for s in ('left','right')]}
    path=tmp_path/'g1.json';path.write_text(json.dumps(p));return path


def test_real_solver_retains_five_dof_objective_and_complete_q(tmp_path):
    solver=G1IK(profile(tmp_path))
    assert solver.model.nq==10 and not solver.ik._strict_position
    q=np.zeros(10);solver.self_test(q)
    desired=q.copy();desired[3]=.2
    result=np.asarray(solver.solve(solver.palms(desired),q,q,deadline_monotonic=time.monotonic()+.3))
    assert result.shape==(10,) and np.max(np.abs(result))>.02
    assert len(solver.last_envelope['lower'])==10 and len(solver.residual)==2


def test_real_g1_process_has_only_ten_joint_result_and_two_eef(tmp_path):
    from scipy.spatial.transform import Rotation
    path=profile(tmp_path);reference=G1IK(path)
    worker=NumericalWorker(path,1.)
    try:
        q=[0.]*10;worker.self_test(q)
        now=time.monotonic_ns()
        body={'boot_id':'test','session_id':'test','seq':1,'source_seq':1,'mapping_epoch':1,
            'model_version':worker.profile['urdf_sha256'],'calibration_version':worker.profile_sha256,
            'frame':worker.profile['torso_frame'],'valid_until_ns':now+300_000_000,
            'values':[v for t in reference.palms(q) for v in t[:3,3].tolist()+Rotation.from_matrix(t[:3,:3]).as_quat().tolist()]}
        state={'state':'ready','feedback':{'q':q,'dq':q,'arm_ns':now},'commanded_q':q}
        assert len(worker.solve_frame(body,state,1))==10
        state['feedback']['arm_ns']=time.monotonic_ns()
        rendered = worker.render(state,{'state':'active'},False,1)
        assert len(rendered['poses'])==2
        display = rendered['visualization']
        assert 'body' not in display and len(display['torso']) == 3
        assert all(len(point) == 3 for line in display['torso'] for point in line)
        np.testing.assert_allclose(display['torso'][0][0], display['measured'][0][0])
        np.testing.assert_allclose(display['torso'][0][3], display['measured'][1][0])
    finally:worker.close()


def test_card_configures_empty_arm_and_keeps_previous_on_invalid_candidate(tmp_path):
    from arm_stream import ArmStreamExecutor
    from motion_control import MotionControl
    path = profile(tmp_path)
    # Synthetic feedback; no channel or SDK is constructed and no start is called.
    executor = ArmStreamExecutor({}, 'g1', None, snapshot=lambda: {
        'q': [0.]*10, 'dq': [0.]*10, 'arm_ns': time.monotonic_ns(),
        'fault': False, 'model_verified': True})
    card = MotionControl({}, executor)
    try:
        result = card.dispatch('config', {'calibration_path': str(path), 'joint_velocity_rad_s': 1.})
        assert result.get('state') == 'configured', result
        assert result['config']['calibration_path'] == str(path)
        assert result['effector_ids'] == ['left', 'right']
        assert executor.profile_sha256 == card.solver.profile_sha256
        previous, digest = card.solver, executor.profile_sha256
        invalid = tmp_path/'invalid.json'
        data = json.loads(path.read_text()); data['urdf_sha256'] = 'invalid'
        invalid.write_text(json.dumps(data))
        rejected = card.dispatch('config', {'calibration_path': str(invalid)})
        assert rejected['state'] == 'error'
        assert executor.profile_sha256 == digest and card.solver is previous
        assert card.cfg['calibration_path'] == str(path)
        assert executor.session_id is None and executor._thread is None
    finally:
        card.stop()


def test_rejected_collision_cannot_contaminate_return_to_valid_target(tmp_path):
    solver=G1IK(profile(tmp_path), collision_checks=True)
    q=np.array([.2037558,.28927523,.05398893,1.03511345,-.07312774,
                .22553113,-.3246167,.02247042,.80643052,.08557935])
    good=solver.palms(q)
    bad=[t.copy() for t in good]
    bad[0][1,3]-=.3;bad[1][1,3]+=.3
    for _ in range(4):
        with pytest.raises(ValueError,match='collision'):
            solver.solve(bad,q,q,deadline_monotonic=time.monotonic()+1.)
        assert solver.ik.snapshot()['history_depth']==0
    result=solver.solve(good,q,q,deadline_monotonic=time.monotonic()+1.)
    assert len(result)==10
    assert solver.ik.snapshot()['history_depth']==1


def test_default_collision_policy_does_not_block_inward_targets(tmp_path):
    solver=G1IK(profile(tmp_path))
    assert solver.collision_checks_enabled is False
    q=np.array([.2037558,.28927523,.05398893,1.03511345,-.07312774,
                .22553113,-.3246167,.02247042,.80643052,.08557935])
    targets=[t.copy() for t in solver.palms(q)]
    targets[0][1,3]-=.3;targets[1][1,3]+=.3
    assert len(solver.solve(targets,q,q,deadline_monotonic=time.monotonic()+1))==10
    invalid=q.copy();invalid[0]=100.
    with pytest.raises(ValueError,match='joint_limit'):
        solver.motion_envelope(q,q,invalid)
