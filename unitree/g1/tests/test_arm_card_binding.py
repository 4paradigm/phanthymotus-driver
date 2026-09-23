"""Actual arm card dispatch/metadata, with no SDK or ROS imports."""
import ast
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest


def plugin():
    source = Path(__file__).parents[1]/'device.py'
    tree = ast.parse(source.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ArmActionPlugin')
    ns = {'_ARM_ACTION_MAP': {'release arm':99,'wave':1}, '_ARM_ID_MAP': {99:'release arm',1:'wave'}}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
    calls=[]
    client=SimpleNamespace(ExecuteAction=lambda action:calls.append(('sdk',action)) or 0)
    return ns['ArmActionPlugin']({},'robot',None,client), calls


def test_existing_arm_gestures_and_release_keep_legacy_behavior():
    arm,calls=plugin()
    assert arm.dispatch('execute',{'gesture':'wave'})['ret']==0
    assert arm.dispatch('release',{})['action_id']==99
    assert calls==[('sdk',1),('sdk',99)]


@pytest.mark.parametrize('action,args', [('release',{}),('execute',{'action_id':99}),('execute',{'gesture':'release arm'})])
def test_all_existing_release_aliases_share_stream_handback(action,args):
    arm,calls=plugin()
    arm._stream=SimpleNamespace(dispatch=lambda verb,body:calls.append((verb,body)) or {'state':'accepted'})
    assert arm.dispatch(action,args)=={'state':'accepted'}
    assert calls==[('release',args)]


def test_legacy_motion_cannot_bypass_stream_owner():
    arm,calls=plugin()
    @contextmanager
    def occupied():
        raise ValueError('arm_owned')
        yield
    arm._stream=SimpleNamespace(legacy=occupied)
    with pytest.raises(ValueError,match='arm_owned'):arm.dispatch('execute',{'action_id':1})
    assert calls==[]


def test_current_metadata_is_exposed_by_discovery_and_info():
    arm,_=plugin()
    metadata={'x-control-target':{'resources':['arm_l','arm_r']},'topic_in':[{'topic':'/robot/motion/arm/command'}]}
    descriptor={'mode':'joint_position','dof':10}
    arm._stream=SimpleNamespace(motion_control=SimpleNamespace(
        arm_metadata=lambda:metadata,control_interface=lambda mode:descriptor),info=lambda:{'state':'idle'})
    tool=arm.get_tool();info=arm.dispatch('info',{})
    assert tool['control_interface']==info['control_interface']==descriptor
    assert tool['topic_in']==info['topic_in']==metadata['topic_in']
    assert {'release','finish','finish_status','info','start','stop'} <= set(tool['inputSchema']['properties']['action']['enum'])


@pytest.mark.parametrize('action,args', [('release',{}),('execute',{'action_id':99}),('execute',{'gesture':'release arm'})])
def test_release_cancels_teleop_before_stream_handback(action,args):
    arm,calls=plugin()
    arm._teleop_control=SimpleNamespace(cancel_for_arm_release=lambda:calls.append(('cancel',)))
    arm._stream=SimpleNamespace(dispatch=lambda verb,body:calls.append((verb,body)) or {'state':'accepted'})
    assert arm.dispatch(action,args)=={'state':'accepted'}
    assert calls==[('cancel',),('release',args)]
