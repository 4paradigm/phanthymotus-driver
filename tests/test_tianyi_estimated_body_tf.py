"""Estimated kinematics never imply measured calibration or relaxed timing."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]/'x-humanoid/tianyi2.0'
spec = importlib.util.spec_from_file_location('estimated_body_tf', ROOT/'estimated_body_tf.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

@pytest.fixture
def estimator():
    config = json.loads((ROOT/'resource/body_tf_estimate.json').read_text())
    clock = [10_000_000_000, 1_000_000_000]
    e = m.Estimator(config, wall_ns=lambda: clock[0], mono_ns=lambda: clock[1])
    e.calibrate(clock[0]-1_000_000, clock[0]+69_000_000_000, clock[0]+1_000_000, 2_000_000)
    return e, clock

def feed(e, gap=10_000_000):
    for name, values in [('head',{1:0.,2:0.,3:0.}),('waist',{31:0.,32:.6}),('leg',{51:-.6,52:.25})]:
        for delta in (-gap//2,gap//2):
            e.observe(name,79_000_000_000+delta, values)

def test_real_leg_coupling_and_estimation_provenance(estimator):
    e, _ = estimator
    feed(e)
    matrix, budget = e.at(10_000_000_000)
    assert np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3))
    assert budget <= 50_000_000
    assert e.config['provenance']['calibration_kind'] == 'estimate'
    assert e.config['provenance']['accuracy_validated'] is False
    q = m.joints({1:0.,2:0.,3:0.,31:0.,32:.6,51:-.6,52:.25})
    assert q['first_leg_pitch_joint'] == .25
    assert q['second_leg_pitch_joint'] == -.85
    assert abs(q['first_leg_pitch_joint']+q['second_leg_pitch_joint']+q['waist_pitch_joint']) < 1e-12

@pytest.mark.parametrize('gap',[50_000_000,100_000_000])
def test_mapping_and_joint_gap_share_budget(estimator,gap):
    e,_=estimator; feed(e,gap)
    with pytest.raises(ValueError,match='budget'): e.at(10_000_000_000)

def test_clock_expiry_and_wall_jump_fail_closed(estimator):
    e,clock=estimator; feed(e)
    clock[1] += 2_000_000_001; clock[0] += 2_000_000_001
    with pytest.raises(ValueError,match='expired'): e.at(clock[0])
    clock[1] -= 2_000_000_001; clock[0] += 1_000_000_000
    with pytest.raises(ValueError,match='clock jumped'): e.at(clock[0])

def test_bad_clock_sample_does_not_extend_expiry(estimator):
    e,clock=estimator; feed(e)
    with pytest.raises(ValueError,match='RTT'):
        e.calibrate(clock[0],clock[0]+69_000_000_000,clock[0]+20_000_000,20_000_000)
    clock[1] += 2_100_000_000;clock[0] += 2_100_000_000
    with pytest.raises(ValueError,match='expired'): e.at(clock[0])

def test_missing_samples_never_extrapolate(estimator):
    e,_=estimator
    with pytest.raises(ValueError,match='bracket'):e.at(10_000_000_000)
    feed(e)
    with pytest.raises(ValueError,match='bracket'):e.at(10_020_000_000)

@pytest.mark.parametrize('values',[{1:0.,2:0.},{1:0.,2:float('nan'),3:0.}])
def test_invalid_feedback_invalidates_history(estimator,values):
    e,_=estimator;feed(e)
    with pytest.raises(ValueError):e.observe('head',79_010_000_000,values)
    with pytest.raises(ValueError,match='bracket'):e.at(10_000_000_000)

def test_clock_reset_discards_prior_joint_history(estimator):
    e,clock=estimator;feed(e)
    e.calibrate(clock[0]-1_000_000,clock[0]+70_000_000_000,clock[0]+1_000_000,2_000_000)
    with pytest.raises(ValueError,match='bracket'):e.at(10_000_000_000)

def test_stale_capture_and_nonmonotonic_feedback_rejected(estimator):
    e,_=estimator;feed(e)
    with pytest.raises(ValueError,match='stale'):e.at(9_000_000_000)
    with pytest.raises(ValueError,match='order'):e.observe('head',78_000_000_000,{1:0.,2:0.,3:0.})

@pytest.mark.parametrize('angle',[0.,.1,1.,np.pi])
def test_tf_matrix_quaternion_round_trip(angle):
    r = m.rotation([1,2,3],angle)
    assert np.allclose(m.quaternion_matrix(m.matrix_quaternion(r)),r,atol=1e-10)

def test_native_optical_chain_composes_without_second_parent(estimator):
    e,_=estimator;feed(e)
    body_from_optical,_=e.at(10_000_000_000)
    native=np.eye(4);native[:3,:3]=m.rotation([1,0,0],.4);native[:3,3]=[.01,.02,.03]
    body_from_root=body_from_optical@np.linalg.inv(native)
    assert np.allclose(body_from_root@native,body_from_optical)

def test_clock_exchange_detects_wall_step(estimator):
    e,_=estimator
    with pytest.raises(ValueError,match='jumped'):
        e.calibrate(10_000_000_000,79_000_000_000,10_100_000_000,1_000_000)

def test_joint_clock_rollback_invalidates_all_groups_and_mapping(estimator):
    e,_=estimator;feed(e)
    with pytest.raises(ValueError,match='order'):
        e.observe('head',78_900_000_000,{1:0.,2:0.,3:0.})
    assert e.clock is None
    assert all(not rows for rows in e.samples.values())

def test_clock_transport_sends_complete_lines(monkeypatch,estimator):
    import subprocess,sys,time
    e,_=estimator
    e.clock=None
    popen=subprocess.Popen
    def local_server(command,**kwargs):
        return popen([sys.executable,'-u','-c',
            'import sys,time; [(print(time.time_ns(),flush=True)) for line in sys.stdin]'],**kwargs)
    monkeypatch.setattr(m.subprocess,'Popen',local_server)
    clock=m.MotionClock(e);clock.worker.start()
    try:
        end=time.monotonic()+1.5
        while e.clock is None and time.monotonic()<end:time.sleep(.01)
        assert e.clock is not None, clock.reason
    finally:clock.close()


def test_clock_close_reaps_child_that_returns_a_partial_line(monkeypatch,estimator):
    import select,subprocess,sys,threading
    e,_=estimator
    popen,wait_readable=subprocess.Popen,select.select
    children=[];readable=threading.Event()
    def local_server(command,**kwargs):
        child=popen([sys.executable,'-u','-c',
            "import sys; sys.stdin.readline(); sys.stdout.write('123'); sys.stdout.flush(); sys.stdin.read()"],**kwargs)
        children.append(child)
        return child
    def observe_readable(*args):
        result=wait_readable(*args)
        if result[0]:readable.set()
        return result
    monkeypatch.setattr(m.subprocess,'Popen',local_server)
    monkeypatch.setattr(m.select,'select',observe_readable)
    clock=m.MotionClock(e);clock.worker.start()
    try:
        assert readable.wait(2), 'local clock child did not return its partial line'
        clock.close()
        assert not clock.worker.is_alive()
        assert children[0].poll() is not None
    finally:
        clock.stop_event.set()
        for child in children:
            if child.poll() is None:child.kill()
            child.wait(timeout=2)
        clock.worker.join(timeout=2)


def test_no_certificate_never_uses_interpolated_tf(estimator):
    e,_=estimator
    evidence=m.CaptureEvidence(e.config,wall_ns=lambda:10_000_000_000,mono_ns=lambda:0)
    with pytest.raises(ValueError,match='exact'):
        evidence.lookup(10_000_000_000,'base_link','ob_camera_head_color_optical_frame','orbbec_head_global_ros_header')

def test_certificate_budget_identity_and_exact_stamp(estimator):
    e,_=estimator;feed(e)
    evidence=m.CaptureEvidence(e.config,wall_ns=lambda:10_000_000_000,mono_ns=lambda:0)
    matrix,bound=e.at(10_000_000_000)
    packet=dict(schema='motus.body_tf_estimate/1',epoch='a',stamp_ns=10_000_000_000,
                parent_frame='base_link',child_frame='ob_camera_head_color_optical_frame',
                clock_id='orbbec_head_global_ros_header',matrix=matrix.tolist(),
                synchronization_bound_ns=bound,provenance=e.config['provenance'])
    evidence.observe(packet)
    assert np.allclose(evidence.lookup(packet['stamp_ns'],packet['parent_frame'],
        packet['child_frame'],packet['clock_id']),matrix)
    with pytest.raises(ValueError,match='exact'):
        evidence.lookup(packet['stamp_ns']+1,packet['parent_frame'],packet['child_frame'],packet['clock_id'])
    packet['synchronization_bound_ns']=50_000_001
    evidence.observe(packet)
    with pytest.raises(ValueError,match='budget'):
        evidence.lookup(packet['stamp_ns'],packet['parent_frame'],packet['child_frame'],packet['clock_id'])

def test_certificate_cache_does_not_revive_after_clock_rollback(estimator):
    e,_=estimator;feed(e)
    clocks=[10_000_000_000,0]
    evidence=m.CaptureEvidence(e.config,wall_ns=lambda:clocks[0],mono_ns=lambda:clocks[1])
    evidence.observe({'epoch':'a','stamp_ns':clocks[0]})
    clocks[0]+=1_000_000_000;clocks[1]+=1_000_000_000
    # Normal passage of time does not refresh receipt age.
    with pytest.raises(ValueError,match='expired'):evidence.lookup(10_000_000_000,'','','')
    clocks[0]-=1_000_000_000
    with pytest.raises(ValueError,match='jumped'):evidence.lookup(10_000_000_000,'','','')
    assert not evidence.packets

def test_exact_certificate_can_arrive_during_bounded_wait(estimator):
    import threading,time
    e,_=estimator;feed(e)
    evidence=m.CaptureEvidence(e.config,wall_ns=lambda:10_000_000_000,mono_ns=lambda:0)
    matrix,bound=e.at(10_000_000_000)
    packet=dict(schema='motus.body_tf_estimate/1',epoch='a',stamp_ns=10_000_000_000,
                parent_frame='base_link',child_frame='ob_camera_head_color_optical_frame',
                clock_id='orbbec_head_global_ros_header',matrix=matrix.tolist(),
                synchronization_bound_ns=bound,provenance=e.config['provenance'])
    t=threading.Timer(.005,lambda:evidence.observe(packet));t.start()
    try:
        assert np.allclose(evidence.lookup(packet['stamp_ns'],packet['parent_frame'],
            packet['child_frame'],packet['clock_id']),matrix)
    finally:t.join()

def test_delayed_old_epoch_cannot_repopulate_cache_after_clock_jump(estimator):
    e,_=estimator
    clocks=[10_000_000_000,0]
    evidence=m.CaptureEvidence(e.config,wall_ns=lambda:clocks[0],mono_ns=lambda:clocks[1])
    p={'epoch':'old','stamp_ns':10_000_000_000}
    evidence.observe(p)
    clocks[:]=[10_100_000_000,1_000_000_000]  # clock is 900 ms behind elapsed time
    evidence.observe(p)
    assert not evidence.packets
    with pytest.raises(ValueError,match='exact'):evidence.lookup(p['stamp_ns'],'','','')
    evidence.observe({'epoch':'new','stamp_ns':10_100_000_000})
    assert list(evidence.packets)==[10_100_000_000]

def test_color_capture_arriving_after_newer_depth_is_not_discarded():
    pending, seen = set(), set()
    m.queue_capture(10_000_150_000, pending, seen)
    pending.clear()  # newer depth was already published
    m.queue_capture(10_000_000_000, pending, seen)
    assert pending == {10_000_000_000}
    pending.clear()
    m.queue_capture(10_000_000_000, pending, seen)
    assert not pending  # duplicate capture still suppressed


def test_scheduler_pause_between_clock_reads_is_not_a_clock_reset(estimator):
    e, _ = estimator
    clocks = [10_000_000_000, 0]
    pause = [False]
    def wall():
        result = clocks[0]
        if pause[0]:
            clocks[0] += 10_000_000
            clocks[1] += 10_000_000
            pause[0] = False
        return result
    evidence = m.CaptureEvidence(e.config, wall_ns=wall, mono_ns=lambda: clocks[1])
    evidence.observe({'epoch': 'a', 'stamp_ns': 10_000_000_000})
    pause[0] = True
    evidence.observe({'epoch': 'a', 'stamp_ns': 10_001_000_000})
    assert evidence.retired_epoch is None
    assert 10_001_000_000 in evidence.packets
