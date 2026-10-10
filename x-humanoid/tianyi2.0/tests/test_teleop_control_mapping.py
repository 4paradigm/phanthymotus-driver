"""Calibrated mapping and optional direct comparison to extracted frozen r4.

Set TIANYI_FROZEN_TELEOP_DIR to the extracted plugins/teleop directory. The
comparison executes the actual frozen class AST, without loading its SDK/ROS.
"""
import ast
import copy
import os
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop_control import RelativeMapping


BASIS = np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])


def normalize(frame):
    def pose(value):
        q=value['orientation']
        return {'position':(BASIS@value['position']).tolist(),
                'orientation_xyzw':[*list(BASIS@q[:3]),q[3]]}
    return {'head_reference':pose(frame['head']),
            **{side:pose(frame[side+'_controller']) for side in ('left','right')}}


def test_offset_rotation_moves_palm_lever_in_normalized_axes():
    start={'head':{'position':[0.,0.,1.], 'orientation':[0.,0.,0.,1.]},
           **{s+'_controller':{'position':[0.,0.,0.], 'orientation':[0.,0.,0.,1.]}
              for s in ('left','right')}}
    current=copy.deepcopy(start)
    for side in ('left','right'):
        current[side+'_controller']['orientation']=Rotation.from_euler('y',90,degrees=True).as_quat().tolist()
    mapping=RelativeMapping(.5)
    mapping.set_controller_offsets({s:{'position':[.1,.2,.3],
        'orientation':Rotation.from_euler('xyz',[.1,.2,.3]).as_quat().tolist()} for s in ('left','right')})
    palms=[[.4,.3,1.,0.,0.,0.,1.],[.4,-.3,1.,0.,0.,0.,1.]]
    mapping.calibrate(normalize(start),palms)
    targets=mapping.targets(normalize(current))
    # Raw offset .1,.2,.3 rotates to .3,.2,-.1: delta (.2,0,-.4),
    # normalized delta (.4,-.2,0), then the configured .5 position scale.
    assert targets[:3]==pytest.approx([.6,.2,1.])
    assert targets[7:10]==pytest.approx([.6,-.4,1.])


def test_nonidentity_mapping_matches_actual_frozen_r4():
    path=os.environ.get('TIANYI_FROZEN_TELEOP_DIR')
    if not path:pytest.skip('explicit extracted frozen r4 directory not provided')
    source=Path(path,'kinematics.py').read_text()
    tree=ast.parse(source)
    wanted={'finite','transform','RelativeMapping'}
    selected=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name in wanted]
    assert len(selected)==3
    scope={'np':np}
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(Path(path,'kinematics.py')),'exec'),scope)
    frozen=scope['RelativeMapping'](.5);current=RelativeMapping(.5)
    random=np.random.default_rng(321)
    def pose():
        return {'position':random.uniform(-.5,.5,3).tolist(),
                'orientation':Rotation.random(random_state=random).as_quat().tolist()}
    def frame():return {k:pose() for k in ('head','left_controller','right_controller')}
    offsets={s:pose() for s in ('left','right')}
    current.set_controller_offsets(offsets)
    frozen.controller_offsets={s:scope['transform'](offsets[s]) for s in offsets}
    origin=frame();palms=[scope['transform'](pose()) for _ in range(2)]
    frozen.reset(origin,palms)
    current.calibrate(normalize(origin),[m[:3,3].tolist()+Rotation.from_matrix(m[:3,:3]).as_quat().tolist() for m in palms])
    for _ in range(64):
        sample=frame();expected=frozen.targets(sample);actual=current.targets(normalize(sample))
        for i,target in enumerate(expected):
            assert actual[i*7:i*7+3]==pytest.approx(target[:3,3],abs=1e-12)
            assert Rotation.from_quat(actual[i*7+3:i*7+7]).as_matrix()==pytest.approx(target[:3,:3],abs=1e-12)
