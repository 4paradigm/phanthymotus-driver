import pytest
from noetix.bumi.odometry_feedback import DistanceFeedback


def binding(revision='one',topic='',issued=10_000_000_000):
    return dict(version=1,revision=revision,input_topic=topic,resolved=True,
                issued_ns=issued,expires_ns=issued+1_000_000_000)


def test_disconnected_binding_is_explicit_and_lease_is_required():
    clock=[10.]
    f=DistanceFeedback(None,now_ns=lambda:int(clock[0]*1e9),monotonic=lambda:clock[0])
    assert f.freeze()['source']=='imu_integrated_experimental' # preserved direct experiment
    f.apply(binding())
    context=f.freeze()
    assert context['revision']=='one'
    clock[0]=11.1
    with pytest.raises(ValueError,match='expired'): f.validate(context)


def test_connected_not_ready_never_falls_back_to_imu(monkeypatch):
    f=DistanceFeedback(None,now_ns=lambda:10_000_000_000,monotonic=lambda:10.)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/sensor/odom'))
    with pytest.raises(ValueError,match='unavailable'): f.freeze()


def test_same_topic_changed_connection_revision_invalidates_action(monkeypatch):
    f=DistanceFeedback(None,now_ns=lambda:10_000_000_000,monotonic=lambda:10.)
    f.apply(binding())
    context=f.freeze()
    f.apply(binding('different-edge',issued=10_000_000_001))
    with pytest.raises(ValueError,match='changed'): f.validate(context)
    with pytest.raises(ValueError,match='out of order'): f.apply(binding())


def sample(t=10_000_000_000, segment='s', status='tracking'):
    return dict(schema='motus.odom/1',stamp_ms=t//1_000_000,frame='body',
                units={'linear':'m/s','angular':'rad/s','length':'m'},twist=[0.]*6,
                pose={'position':[0,0,0],'quaternion':[0,0,0,1], 'frame_id':'odom','child_frame_id':'base_link','covariance':[.01 if i%7==0 else 0 for i in range(36)]},
                vendor=dict(status=status,segment_id=segment,calibration_id='c',feedback_source='rgbd_imu_ekf',
                            published_ns=t,visual_stamp_ns=t,accuracy_validated=False))


def test_atomic_pose_quality_and_reset_latch(monkeypatch):
    now=[10_000_000_000]; faults=[]
    f=DistanceFeedback(None,now_ns=lambda:now[0],monotonic=lambda:now[0]/1e9,on_fault=faults.append)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/sensor/odom'))
    f.update(sample()); context=f.freeze()
    now[0]+=50_000_000
    f.update(sample(now[0],status='lost'))
    f.update(sample(now[0]+1,segment='replacement'))
    with pytest.raises(ValueError): f.validate(context)
    assert faults


def test_normal_sensor_startup_can_become_ready_without_a_previous_pose(monkeypatch):
    now=[10_000_000_000]
    f=DistanceFeedback(None,now_ns=lambda:now[0],monotonic=lambda:now[0]/1e9)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/sensor/odom'))
    initializing=sample(status='initializing');initializing['pose']=None
    f.update(initializing)
    with pytest.raises(ValueError,match='unavailable'): f.freeze()
    now[0]+=50_000_000; f.update(sample(now[0]))
    assert f.freeze()['source']=='rgbd_imu_ekf'


def test_binding_error_can_recover_without_clearing_data_faults(monkeypatch):
    now=[10_000_000_000]
    f=DistanceFeedback(None,now_ns=lambda:now[0],monotonic=lambda:now[0]/1e9)
    f.apply(binding())
    bad=binding(issued=now[0]+1);bad['resolved']=False
    with pytest.raises(ValueError): f.apply(bad)
    now[0]+=100
    f.apply(binding(issued=now[0]))
    assert f.freeze()['source']=='imu_integrated_experimental'


def test_old_subscription_callback_cannot_populate_replacement_binding(monkeypatch):
    import json
    from types import SimpleNamespace as NS
    f=DistanceFeedback(None,now_ns=lambda:10_000_000_000,monotonic=lambda:10.)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/old'))
    old_generation=f.generation
    f.apply(binding('new-edge',topic='/new',issued=10_000_000_001))
    f._message(NS(data=json.dumps(sample())),old_generation)
    with pytest.raises(ValueError,match='unavailable'): f.freeze()
    f._message(NS(data=json.dumps(sample())),f.generation)
    assert f.freeze()['revision']=='new-edge'


def test_acceptance_freezes_relative_origin_not_a_later_pose(monkeypatch):
    f=DistanceFeedback(None,now_ns=lambda:10_000_000_000,monotonic=lambda:10.)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/sensor/odom')); f.update(sample())
    context=f.freeze()
    newer=sample(10_001_000_000); newer['pose']['position'][0]=.02
    f.now_ns=lambda:10_001_000_000; f.update(newer)
    assert context['origin']['pose']['position']==[0,0,0]
    assert f.validate(context)['pose']['position']==[.02,0,0]


def test_expired_lease_renewal_faults_old_action_even_before_watchdog_runs():
    clock=[10.]; faults=[]
    f=DistanceFeedback(None,now_ns=lambda:int(clock[0]*1e9),monotonic=lambda:clock[0],on_fault=faults.append)
    f.apply(binding()); f.freeze()
    clock[0]=11.01
    f.apply(binding(issued=11_010_000_000))
    assert faults==['feedback binding lease expired']
    assert f.freeze()['source']=='imu_integrated_experimental'  # fresh action may recover


def test_cancel_before_first_control_iteration_keeps_healthy_quality(monkeypatch):
    import threading
    from types import SimpleNamespace as NS
    from noetix.bumi.odometry_feedback import run_fused_distance
    f=DistanceFeedback(None,now_ns=lambda:10_000_000_000,monotonic=lambda:10.)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/sensor/odom')); f.update(sample())
    commands=[]; stopped=threading.Event(); stopped.set()
    owner=NS(_distance_feedback=f,_distance_fault=None,_rotate_stop_event=stopped,
             _distance_publish=lambda *args:commands.append(args),_rotate_mode_error=lambda:None)
    result=run_fused_distance(owner,1.,f.freeze())
    assert result['state']=='stopped'
    assert 'error' not in result
    assert result['estimated_quality']['status']=='tracking'
    assert commands[-1]==(0,)


def test_old_watchdog_validation_cannot_fault_a_replacement_action(monkeypatch):
    import threading
    f=DistanceFeedback(None)
    old={'revision':'old'}; current=[old]; faults=[]
    validating=threading.Event(); resume=threading.Event()
    def validate(context):
        if context is old:
            validating.set(); resume.wait(1)
            raise ValueError('old binding changed')
    monkeypatch.setattr(f,'validate',validate)
    f.on_fault=faults.append
    f.start_watchdog(lambda:current[0])
    try:
        assert validating.wait(1)
        current[0]={'revision':'new'}
        resume.set()
        f.stopped.set(); f.thread.join(1)
        assert not faults
    finally:
        resume.set(); f.close()
