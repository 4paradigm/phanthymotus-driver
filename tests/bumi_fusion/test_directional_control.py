"""Control model truth, estimated odometry and requested target stay separate."""
import math
import threading
from types import SimpleNamespace as NS

import pytest
from noetix.bumi import odometry_feedback as module
from test_feedback import binding, sample


def trial(monkeypatch, direction, axis, *, scale=1., fault=None):
    now=[10.]; truth=[0.,0.]; velocity=[0.,0.]; command=[0.,0.,0.]; commands=[]; rows=[]
    f=module.DistanceFeedback(None,now_ns=lambda:int(now[0]*1e9),monotonic=lambda:now[0])
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    owner=NS(_distance_feedback=f,_distance_fault=None,_rotate_stop_event=threading.Event(),_rotate_mode_error=lambda:None)
    def abort(reason):
        owner._distance_fault=owner._distance_fault or reason
        owner._rotate_stop_event.set()
        publish(0)
    f.on_fault=abort
    def publish(vx, vyaw=0, *, vy=0):
        command[:]=[vx,vy,vyaw]; commands.append(tuple(command))
    owner._distance_publish=publish
    def observe():
        f.apply(binding(topic='/sensor/odom',issued=int(now[0]*1e9)))
        row=sample(int(now[0]*1e9))
        # The robot initially faces world +Y, and starts far from world zero.
        row['pose']['position']=[2-scale*truth[1],3+scale*truth[0],0]
        row['pose']['quaternion']=[0,0,math.sqrt(.5),math.sqrt(.5)]
        row['twist'][:2]=[scale*v for v in velocity]
        f.update(row)
        rows.append(dict(truth_m=truth[0]*axis[0]+truth[1]*axis[1],
                         estimated_m=scale*(truth[0]*axis[0]+truth[1]*axis[1]),target_m=1.))
    observe(); context=f.freeze()
    def sleep(dt):
        for i in range(2):
            velocity[i]+=(.55*command[i]-velocity[i])*min(1,dt/.12)
            truth[i]+=velocity[i]*dt
        now[0]+=dt
        if fault and now[0]>11:
            if fault=='stale': return
            if fault=='cancel': owner._rotate_stop_event.set(); return
            row=sample(int(now[0]*1e9),status='lost' if fault=='lost' else 'tracking',
                       segment='replacement' if fault=='reset' else 's')
            if fault=='connection':
                f.apply(binding('new-edge',topic='/sensor/odom',issued=int(now[0]*1e9)))
            else: f.update(row)
            return
        observe()
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(module.time,'sleep',sleep)
    result=module.run_fused_distance(owner,1.,context,direction=direction)
    return result,commands,rows


@pytest.mark.parametrize('direction,axis', [('forward',(1,0)),('backward',(-1,0)),('left',(0,1)),('right',(0,-1))])
def test_fused_distance_preserves_all_starting_body_directions(monkeypatch,direction,axis):
    result,commands,rows=trial(monkeypatch,direction,axis)
    assert result['state']=='done',result
    assert commands[-1]==(0,0,0)
    moving=next(c for c in commands if c[0] or c[1])
    assert moving[0]*axis[0]+moving[1]*axis[1]>0
    assert abs(moving[0]*axis[1]-moving[1]*axis[0])<1e-8
    assert result['requested_direction']==direction
    assert abs(rows[-1]['truth_m']-1.)<.05
    assert result['accuracy_validated'] is False


def test_scale_error_can_finish_without_meeting_physical_accuracy(monkeypatch):
    result,_,rows=trial(monkeypatch,'left',(0,1),scale=1.15)
    assert result['state']=='done'
    assert abs(rows[-1]['estimated_m']-1.)<.05
    assert abs(rows[-1]['truth_m']-1.)>.1
    assert result['accuracy_validated'] is False


@pytest.mark.parametrize('fault',['lost','stale','reset','connection','cancel'])
def test_lateral_fault_or_cancel_brakes_without_imu_fallback(monkeypatch,fault):
    result,commands,_=trial(monkeypatch,'left',(0,1),fault=fault)
    assert result['state']==('stopped' if fault=='cancel' else 'error')
    assert commands[-1]==(0,0,0)
    assert result['feedback_source']=='rgbd_imu_ekf'
