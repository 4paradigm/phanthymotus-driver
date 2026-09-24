"""G1 behavior port with Tianyi position execution; no robot/network."""
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from types import SimpleNamespace
import pytest
from test_motion_stream import rig
from test_motion_control_bundle import bundle_class, ros_stubs
from test_servo import FakeROS2
from teleop_executor import TeleopExecutor, load_profile
from tianyi_motion.kinematics import TianyiIK


def test_builtin_model_opens_in_numeric_worker_without_site_paths():
    path=Path(__file__).resolve().parents[1]/'tianyi_motion/tianyi2_dual_arm.json'
    profile,limits,_=load_profile(path)
    assert len(limits)==14 and profile['hands_enabled'] is False
    assert 'first_acceptance' not in profile and 'acceptance' not in profile
    model=TianyiIK(path)
    assert model.model.nq==14 and len(model.palms([0.]*14))==2


def test_default_bundle_is_live_but_registration_has_no_output(bundle_class):
    bundle=bundle_class({'plugins':{'arm':{'enabled':True}}},'offline',FakeROS2(),None)
    executor=bundle._teleop
    assert executor.profile_error is None and executor.profile['hands_enabled'] is False
    assert executor.teleop_control.live
    assert executor.gate.session_id is None and not executor._output_ready
    assert executor._bus_process is None
    assert [t['name'] for t in bundle.get_all_tools()].count('teleop_control')==1


def test_ordinary_hold_resumes_with_fresh_feedback_without_stationary_threshold():
    gate,status,writes,advance,packet=rig()
    gate.resume_without_settle=True
    gate.accept(packet());gate.hold('operator_pause')
    advance(20);gate.tick()
    status['q']=[.05]*14;status['dq']=[.1]*14
    advance(20);gate.tick()
    assert gate.status()['hold_confirmed']
    gate.resume()
    assert gate.state=='ready' and gate.latest is None


def test_release_still_requires_actual_stop_receipt():
    gate,status,writes,advance,packet=rig();gate.resume_without_settle=True
    gate.hold(release=True);advance(20);gate.tick()
    status['dq']=[.1]*14;advance(20);gate.tick()
    assert gate.session_id and not gate.status()['stop_confirmed']


@pytest.mark.parametrize('gap',[20,50,150])
def test_smoothing_is_from_sent_reference_and_never_accumulates_gap(gap):
    gate,status,writes,advance,packet=rig()
    gate.velocity=1.;gate.smoothing_seconds=.12
    gate.accept(packet(q=[.1]*14))
    # Isolate the reference shaper from the independent model-proof protocol.
    gate._continuous_v2=True;gate.lease_deadline+=1_000_000_000
    advance(gap);gate.tick()
    dt=min(gap/1000.,.05)
    expected=min(dt,.1*(1-math.exp(-dt/.12)))
    assert writes[-1][0]==pytest.approx([expected]*14)
    assert gate.last_q==writes[-1][0]


def test_failed_publish_does_not_advance_reference():
    gate,status,writes,advance,packet=rig();gate.smoothing_seconds=.12
    gate.accept(packet());before=list(gate.last_q)
    def fail(*a):raise RuntimeError('transport_failed')
    gate.emit=fail;advance(20);gate.tick()
    assert gate.last_q==before and gate.applied_seq==-1


def test_transport_recovery_preserves_operator_pause_reason():
    item=TeleopExecutor({},'test',None,None,None,[])
    gate,status,writes,advance,packet=rig();item.gate=gate
    gate.hold('operator_pause');item._communication_hold('local_dds_process_exited')
    assert gate.reason=='operator_pause'


def test_reset_stops_teleop_before_using_existing_arm_gesture(bundle_class):
    events=[]
    class Gate:
        def legacy(self):
            from contextlib import nullcontext
            events.append('legacy');return nullcontext()
    control=SimpleNamespace(dispatch=lambda *a:events.append('stop') or {})
    bundle=bundle_class.__new__(bundle_class)
    bundle._teleop=SimpleNamespace(gate=Gate(),teleop_control=control)
    bundle._dispatch=lambda *a:events.append('reset') or {'state':'completed'}
    assert bundle.dispatch('arm_gesture',{'action':'reset','side':'both'})['state']=='completed'
    assert events==['stop','legacy','reset']
