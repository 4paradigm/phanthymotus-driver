"""Two-Driver state machine with actual FK/IK/gate and a finite-speed plant.

No ROS or robot. DDS is exercised separately; the underlying chain fixture
retains real solver/arm receiver and does not overwrite measurement on publish.
"""
import copy
import json
import threading
import time

import numpy as np
import pytest

from common.teleop_contract import COMMAND_SCHEMA, TRACKING_FRAME, validate_feedback
from test_motion_control import chain, wait_for  # noqa: F401
from teleop_control import TeleopControl
from test_motion_control_bundle import bundle_class  # noqa: F401
from test_servo import ros_stubs  # noqa: F401


@pytest.fixture
def control(chain):
    c = TeleopControl({'mode': 'shadow'}, chain.c)
    c.clock_id = 'offline-boot'
    c.binding = {'namespace': 'offline', 'instance_id': 'pico_1',
                 'command_topic': '/teleop/command',
                 'feedback_topic': '/teleop/state'}
    c._state = 'ready'
    yield c
    c._closed.set()
    c._generation += 1


def frame(c, sequence, *, grip=0, x=0., space=0, conn=0, received=None):
    stamp = c.clock() if received is None else received
    pose = dict(tracked=True, position=[0., 0., 1.6], orientation_xyzw=[0., 0., 0., 1.])
    return dict(schema=COMMAND_SCHEMA, kind='input', instance_id='pico_1', device_id='pico',
        connection_epoch=conn, space_epoch=space, sequence=sequence, clock_id=c.clock_id,
        received_monotonic_ns=stamp, source_monotonic_ns=stamp, tracking_frame=TRACKING_FRAME,
        head_reference=pose,
        **{s:dict(pose, position=[x,sign*.3,1.], grip=grip, trigger=0.)
           for s,sign in [('left',1),('right',-1)]})


def feed(c, sequence, **kwargs):
    c.receive(frame(c, sequence, **kwargs)); c.step()


def op(c, action, seq=0, request=None):
    current = c._latest
    now = c.clock()
    return dict(schema=COMMAND_SCHEMA, kind='operation', instance_id='pico_1', device_id='pico',
        connection_epoch=current['connection_epoch'], space_epoch=current['space_epoch'],
        sequence=seq, clock_id=c.clock_id, received_monotonic_ns=now,
        expires_monotonic_ns=now+5_000_000_000, request_id=request or f'op-{seq}', action=action)


def begin(c, continuous=False):
    c.receive(frame(c,0))
    if not continuous:
        c._begin(c._generation,'begin');return 0
    closed=threading.Event()
    def sample():
        seq=0
        while not closed.wait(.03):
            seq+=1;c.receive(frame(c,seq))
    producer=threading.Thread(target=sample,daemon=True);producer.start()
    try:c._begin(c._generation,'begin')
    finally:closed.set();producer.join(1)
    return c._input_sequence


def test_begin_anchors_after_cold_calibration_latest_input(control, monkeypatch):
    c=control
    c.receive(frame(c,0,x=0.))
    original=c.motion.dispatch
    def delayed(action,args):
        result=original(action,args)
        if action=='calibrate':c.receive(frame(c,1,x=.4))
        return result
    monkeypatch.setattr(c.motion,'dispatch',delayed)
    c._begin(c._generation,'begin')
    assert c.mapping.anchor[0]['sequence']==1
    assert c.mapping.anchor[0]['left']['position'][0]==.4
    assert c.mapping.epoch==1


def test_regrip_preserves_mapping_despite_actual_lag_and_hand_mismatch(control, chain):
    c=control;begin(c)
    original=copy.deepcopy(c.mapping.anchor)
    base_x=original[1][0][0]
    feed(c,1,grip=1,x=.04)
    assert c.motion._pending[0]['values'][0]==pytest.approx(base_x+.02)
    # This mapping-only case deliberately asks beyond the straight-arm reach.
    # The robot has NOT instantaneously reached the solver result.
    assert np.max(np.abs(chain.p.q))==0
    for index in range(10):
        x=.04+.002*index
        feed(c,2+2*index,grip=0,x=x)
        feed(c,3+2*index,grip=1,x=x+.01)
        body=c.motion._pending[0]
        assert body['source_seq']==3+2*index
        assert body['values'][0]==pytest.approx(base_x+.5*(x+.01))
        assert body['mapping_epoch']==1
        assert c.mapping.anchor==original
    assert c.mapping.epoch==1


def test_space_reset_needs_explicit_calibration_not_grip(control):
    c=control;begin(c)
    feed(c,1,grip=1,space=1)
    assert c._reason=='needs_calibration' and c.motion._pending is None
    feed(c,2,grip=0,space=1);feed(c,3,grip=1,space=1)
    assert c.mapping.epoch==1 and c.motion._pending is None
    # An explicit operation, rather than grip, is the new reference authority.
    c._begin(c._generation,'calibrate')
    assert c.mapping.epoch==2


def test_canvas_stop_only_holds_and_does_not_call_finish(control, monkeypatch):
    c=control;begin(c)
    actions=[];real=c.motion.dispatch
    def track(action,args):actions.append(action);return real(action,args)
    monkeypatch.setattr(c.motion,'dispatch',track)
    result=c.stop()
    assert 'finish' not in actions and result['return_required'] is False
    assert result['authority_released'] and not c._operator


def feed_reachable(c, sequence, amount, grip=1):
    from scipy.spatial.transform import Rotation
    from teleop_control import _mul, _inverse
    q=np.zeros(14);q[0]=q[7]=amount
    poses=getattr(c,'_test_fk',c.motion.solver).palms(q)
    value=frame(c,sequence,grip=grip)
    initial,robot,_=c.mapping.anchor
    for i,side in enumerate(('left','right')):
        value[side]['position']=[float(a+2*(b-d)) for a,b,d in zip(initial[side]['position'],poses[i][:3,3],robot[i][:3])]
        value[side]['orientation_xyzw']=_mul(Rotation.from_matrix(poses[i][:3,:3]).as_quat().tolist(),_inverse(robot[i][3:]))
    c.receive(value);c.step()


def test_live_regrip_rotation_preserves_anchor_with_real_plant(control, chain):
    from tianyi_motion.worker import NumericalWorker
    c=control;c.cfg['mode']='live';chain.e.gate.live_enabled=True
    c._test_fk=c.motion.solver;c.motion.solver_factory=NumericalWorker
    chain.e.cfg['operator_session_enabled']=True
    chain.p.run(chain.e.gate)
    base=begin(c,continuous=True)
    anchor=copy.deepcopy(c.mapping.anchor)
    feed_reachable(c,base+1,.06)
    assert c.motion.process_latest(), c.motion._decision
    wait_for(lambda:len(chain.p.writes)>1)
    first_session=chain.e.gate.session_id
    feed_reachable(c,base+2,.09,grip=0)
    try:wait_for(lambda:chain.e.gate.status()['hold_confirmed'])
    except AssertionError:raise AssertionError(c.feedback()) from None
    feed_reachable(c,base+3,.1)
    assert chain.e.gate.session_id!=first_session
    assert c.mapping.anchor==anchor and c.mapping.epoch==1
    assert c.motion._pending[0]['mapping_epoch']==1
    assert c.motion.process_latest(), c.motion._decision
    # Follow finite-speed measurements while refreshing identical new targets.
    for seq in range(4,14):
        feed_reachable(c,base+seq,.1);c.motion.process_latest();time.sleep(.04)
    assert np.max(np.abs(chain.p.q))>.005,c.feedback()
    assert np.max(np.abs(chain.p.dq))<=1.01
    result=c.stop()
    assert result['authority_released'] and not chain.e.gate.session_id


def test_model_config_save_failure_reports_error_with_previous_effective_values(control, chain, tmp_path, monkeypatch):
    import teleop_control
    c=control;c._config_path=tmp_path/'state.json'
    monkeypatch.setattr(teleop_control.os,'replace',lambda *args:(_ for _ in ()).throw(OSError('disk_failed')))
    result=c.dispatch('config',{'position_scale':.8,'calibration_path':str(chain.path)})
    assert result['error']=='disk_failed'
    assert c.mapping.scale==.5 and not c._config_path.exists()


def test_new_card_bundle_constructs_one_control_no_external_tokens(bundle_class, tmp_path):
    from test_motion_control_bundle import calibration
    from test_servo import FakeROS2
    path=calibration(tmp_path)
    cfg={'plugins':{'arm':{'enabled':True}},'teleop_control':{
        'enabled':True,'mode':'shadow','calibration_path':str(path), 'state_path':str(tmp_path/'cfg.json')}}
    bundle=bundle_class(cfg,'offline',FakeROS2(),None)
    tools=bundle.get_all_tools();names=[t['name'] for t in tools]
    assert names.count('teleop_control')==1
    assert 'motion_control' not in names and 'teleop_executor' not in names
    assert not bundle._teleop.gate.session_id and bundle._teleop.node is None
    tool=next(t for t in tools if t['name']=='teleop_control')
    assert all(p['scope']=='instance' for p in tool['configSchema']['properties'].values())
    assert not any('secret' in p or 'binding' in p for p in tool['inputSchema']['properties'])


@pytest.mark.parametrize('phase',['calibrate','prepare_preview'])
def test_connection_change_during_begin_cannot_inherit_old_request(control,monkeypatch,phase):
    c=control;c.receive(frame(c,0))
    original=c.motion.dispatch
    def changed(action,args):
        result=original(action,args)
        if action==phase:c.receive(frame(c,1,conn=1))
        return result
    monkeypatch.setattr(c.motion,'dispatch',changed)
    with pytest.raises(ValueError,match='operation_input_generation_changed'):
        c._begin(c._generation,'begin')
    assert c.mapping.epoch==0 and not c._operator and c.motion._preview is None


def test_anchor_refreshes_input_and_measured_fk_after_prepare(control,chain,monkeypatch):
    c=control;c.receive(frame(c,0))
    original=c.motion.dispatch
    def changed(action,args):
        result=original(action,args)
        if action=='prepare_preview':
            c.receive(frame(c,1,x=.3))
            with chain.p.lock:
                chain.p.q[0]=.05;chain.p.timestamp=time.monotonic_ns()
        return result
    monkeypatch.setattr(c.motion,'dispatch',changed)
    c._begin(c._generation,'begin')
    from scipy.spatial.transform import Rotation
    actual=c.motion.solver.palms(chain.p.q)
    assert c.mapping.anchor[0]['sequence']==1
    assert c.mapping.anchor[1][0][:3]==pytest.approx(actual[0][:3,3])
    assert c.mapping.anchor[1][0][3:]==pytest.approx(Rotation.from_matrix(actual[0][:3,:3]).as_quat())


def test_canvas_auto_prepares_and_rejects_old_headset_operations(control):
    c=control
    feed(c,0,grip=1)
    assert c.mapping.anchor is None
    feed(c,1)
    assert c.mapping.epoch == 1 and c._operator
    with pytest.raises(ValueError, match='device_input_only'):
        c.receive(op(c,'begin'))
    assert validate_feedback(c.feedback(), instance_id='pico_1',clock_id=c.clock_id,now_ns=c.clock())


def test_legacy_ui_config_cannot_override_robot_preset(chain,tmp_path):
    path=tmp_path/'state.json';path.write_text('{"mode":"shadow","position_scale":-1}')
    c=TeleopControl({'state_path':str(path),'position_scale':.5},chain.c)
    assert c.live and c.mapping.scale == .5
    assert set(c.get_tool()['configSchema']['properties']) == {'usage_guide'}
    assert 'instance_id' not in c.get_tool()['inputSchema']['properties']


@pytest.mark.parametrize('phase',['calibrate','prepare_preview'])
def test_canvas_stop_fences_blocked_preparation(control,monkeypatch,phase):
    c=control;c.receive(frame(c,0))
    entered,release=threading.Event(),threading.Event()
    real=c.motion.dispatch
    def delayed(action,args):
        if action==phase:entered.set();release.wait(2)
        return real(action,args)
    monkeypatch.setattr(c.motion,'dispatch',delayed)
    worker=threading.Thread(target=c.step);worker.start()
    assert entered.wait(1)
    try:
        before=time.monotonic();assert c.stop()['authority_released']
        assert time.monotonic()-before < .5
    finally:release.set();worker.join(2)
    assert not c._operator and not c.motion.gate.session_id
    assert c.motion._preview is None
