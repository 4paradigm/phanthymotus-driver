import _point_test_helpers
"""Cross-body geometry with captured left feedback, no robot transport."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import directional_kinematics as g

RIGHT = [.2926547825,-.2243207246,.0195942074,.9778409004,-.1476456523]
LEFT = [.28758546710014343,.23094800114631653,-.018096180632710457,.9727954864501953,.09320131689310074]
ROOT = Path(__file__).resolve().parent

@pytest.fixture
def model():
    return g.DirectionalKinematics(dict(solver_mode='bounded_ik',verified_start_q_rad=RIGHT,
        ik_reference_q_rad=[-.268,-.096,.005,.041,1.5],angular_tolerance_deg=5))

def direction(yaw,pitch):
    y,p = map(math.radians,(yaw,pitch))
    return [math.cos(y)*math.cos(p),math.sin(y)*math.cos(p),math.sin(p)]

def mirror_joint_fixture(q):
    # Synthetic opposite-arm posture only. The model still uses independently
    # parsed left origins/limits and independently derived left mesh boxes.
    return [q[0],-q[1],-q[2],q[3],-q[4]]

@pytest.mark.parametrize('method',('solve','check_path','check_measured_pose'))
def test_missing_left_feedback_is_never_an_empty_obstacle_set(model,method):
    with pytest.raises(ValueError,match='directional_left_feedback_required'):
        if method == 'solve': model.solve([0,1,0])
        elif method == 'check_path': model.check_path(RIGHT,RIGHT)
        else: model.check_measured_pose(RIGHT)

@pytest.mark.parametrize('bad',([0.]*4,[0.]*6,[0.]*4+[float('nan')],[False]*5,'zero'))
def test_malformed_left_feedback_fails_before_geometric_use(model,bad):
    with pytest.raises(ValueError):
        model.check_measured_pose(RIGHT,left_q5=bad)

@pytest.mark.parametrize('index',range(5))
@pytest.mark.parametrize('side',('left','right'))
def test_both_arms_use_original_hard_limits(model,index,side):
    right,left = RIGHT[:],LEFT[:]
    if side == 'left': left[index] = model._left_chain[index].upper+.000001
    else: right[index] = model._chain[index].upper+.000001
    with pytest.raises(ValueError,match='directional_joint_limit'):
        model.check_measured_pose(right,left_q5=left)

@pytest.mark.parametrize('yaw,pitch',((90,0),(20,20),(-90,0),(180,0),(0,90)))
def test_full_direction_ik_and_fixed_left_nominal_path(model,yaw,pitch):
    wanted = direction(yaw,pitch)
    q = model.solve(wanted,start_q5=RIGHT,left_q5=LEFT)
    assert q == model.solve(wanted,start_q5=RIGHT,left_q5=LEFT)
    assert q[4] == RIGHT[4]
    assert g.angular_error_deg(model.direction(q),wanted) < .01
    assert all(j.lower <= v <= j.upper for j,v in zip(model._chain,q))
    assert model.check_measured_pose(q,left_q5=LEFT) >= .003
    assert model.check_path(RIGHT,q,left_q5=LEFT) >= .003
    assert model.check_path(q,RIGHT,left_q5=LEFT) >= .003


def test_positive90_crosses_old_halfspace_but_has_full_arm_body_certificate(model):
    q = model.solve([0,1,0],left_q5=LEFT)
    assert max(box[4] for box in model._collision_bounds(q)) > .1
    assert model.check_path(RIGHT,q,left_q5=LEFT) >= .003
    # Sample the proven path independently against actual left boxes.
    left_bounds = model._left_obstacles(LEFT)
    for step in range(101):
        actual = [a+(b-a)*step/100 for a,b in zip(RIGHT,q)]
        assert model._clearance(actual,left_bounds=left_bounds) >= .003


def test_actual_left_arm_collision_is_rejected_without_side_assumptions(model):
    q = model.solve([0,1,0],left_q5=LEFT)
    changed_left = mirror_joint_fixture(q)
    assert model._left_angles(changed_left)
    with pytest.raises(ValueError,match='directional_measured_body_clearance'):
        model.check_measured_pose(q,left_q5=changed_left)


def test_left_change_can_invalidate_path_even_when_both_endpoints_are_clear(model):
    q = model.solve([0,1,0],left_q5=LEFT)
    changed_left = mirror_joint_fixture([(a+b)/2 for a,b in zip(RIGHT,q)])
    assert model.check_measured_pose(RIGHT,left_q5=changed_left) >= .003
    assert model.check_measured_pose(q,left_q5=changed_left) >= .003
    with pytest.raises(ValueError,match='directional_body_clearance'):
        model.check_path(RIGHT,q,left_q5=changed_left)


def test_measured_pose_uses_actual_left_without_ik_or_path_search(model,monkeypatch):
    before = dict(model.__dict__)
    monkeypatch.setattr(model,'solve',lambda *a,**k:pytest.fail('measured check called IK'))
    monkeypatch.setattr(model,'check_path',lambda *a,**k:pytest.fail('measured check called path search'))
    right,left = RIGHT[:],LEFT[:]
    assert model.check_measured_pose(right,left_q5=left) >= .003
    assert right == RIGHT and left == LEFT
    assert all(model.__dict__[key] == value for key,value in before.items())


def test_solve_never_ignores_total_certificate_budget(model,monkeypatch):
    monkeypatch.setattr(g,'_MAX_IK_TOTAL_CERTIFICATE_NODES',0)
    with pytest.raises(ValueError,match='directional_no_feasible_path'):
        model.solve([0,1,0],left_q5=LEFT)


def test_nominal_path_node_budget_and_three_mm_margin_remain(model,monkeypatch):
    q = model.solve([0,1,0],left_q5=LEFT)
    monkeypatch.setattr(g,'_MAX_CERTIFICATE_NODES',0)
    with pytest.raises(ValueError,match='directional_geometry_budget_exceeded'):
        model.check_path(RIGHT,q,left_q5=LEFT)


def test_original_torso_keepout_still_rejects(model):
    witness = [.270413737,-.204308744,.032104068,.958488028,-.167813439]
    assert model._clearance(witness,left_bounds=model._left_obstacles(LEFT)) < .003
    with pytest.raises(ValueError,match='directional_measured_body_clearance'):
        model.check_measured_pose(witness,left_q5=LEFT)


def test_left_origins_limits_and_both_pitch_collision_parts_are_not_mirror_guesses(model):
    assert model._left_chain[0].origin[1] == .10022
    assert model._chain[0].origin[1] == -.10021
    assert (model._left_chain[1].lower,model._left_chain[1].upper) == (-1.5882,2.2515)
    assert (model._chain[1].lower,model._chain[1].upper) == (-2.2515,1.5882)
    assert len(g._ARM_BOXES[0]) == len(g._LEFT_ARM_BOXES[0]) == 1
    assert len(model._left_obstacles(LEFT)) == sum(map(len,g._LEFT_ARM_BOXES))
    # The source mesh slice has x=45 mm, versus right mesh's distinct 46 mm.
    assert g._LEFT_ARM_BOXES_MM[2][1][3] == 45
    assert g._ARM_BOXES_MM[2][1][3] == 46


def test_left_mesh_provenance_and_embedded_bounds_match_audited_cover():
    # The optional provenance tool verifies every original STL triangle. Unit
    # tests need no vendor mesh downloads and bind that audited report here.
    metadata = json.loads((ROOT.parent/'directional_geometry/point-left-collision-cover.json').read_text())
    assert metadata['source_commit'] == '157ff950351e5f535e03c5d85a16b7d594b3dbcc'
    for link,entry in enumerate(metadata['meshes'],2):
        assert entry['sha256'] == g._LEFT_MESH_SHA256[entry['filename']]
        assert tuple(map(tuple,entry['boxes_mm'])) == g._LEFT_ARM_BOXES_MM[link]
        assert entry['triangles'] > 0


def test_both_shoulder_primitive_covers_contain_the_original_urdf_cylinders():
    root = ET.fromstring((g._MOTION/'models/body23.urdf').read_text())
    links = {item.get('name'):item for item in root.findall('link')}
    for side,boxes in (('right',g._ARM_BOXES_MM),('left',g._LEFT_ARM_BOXES_MM)):
        for index,joint in enumerate(('pitch','roll')):
            collision = links[side+'_shoulder_'+joint+'_link'].find('collision')
            origin = collision.find('origin')
            center = tuple(map(float,origin.get('xyz').split()))
            rotation = g._rpy_rotation(tuple(map(float,origin.get('rpy').split())))
            cylinder = collision.find('geometry/cylinder')
            radius,half = float(cylinder.get('radius')),float(cylinder.get('length'))/2
            # Exact support function of a cylinder on each transformed axis.
            support = [radius*math.hypot(row[0],row[1])+half*abs(row[2]) for row in rotation]
            bounds = boxes[index][0]
            assert all(bounds[i]/1000 <= center[i]-support[i]+1e-12
                       and center[i]+support[i] <= bounds[i+3]/1000+1e-12 for i in range(3))
