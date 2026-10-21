"""Deterministic control-contract simulation; NOT a RTAB-Map accuracy test.

Model truth, delayed/noisy odometry estimate and target are independent. A scale
error can yield done while the physical model missed the target: report both.
"""
from __future__ import annotations
import argparse
from contextlib import ExitStack
import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import patch
from noetix.bumi.odometry_feedback import DistanceFeedback,run_fused_distance


def trial(*,scale=1.,bias=0.,noise=.001,fault=None):
    t=10.; truth=0.; speed=0.; command=0.; last_observation=0.; last_lease=0.
    rows=[]; commands=[]
    owner=SimpleNamespace(_distance_fault=None,_rotate_stop_event=threading.Event(),_rotate_mode_error=lambda:None)
    def abort(reason): owner._distance_fault=owner._distance_fault or reason
    feedback=DistanceFeedback(None,on_fault=abort,now_ns=lambda:int(t*1e9),monotonic=lambda:t)
    feedback._subscribe=lambda topic:None  # replace ROS transport only
    owner._distance_feedback=feedback
    def lease(revision='fixture'):
        stamp=int(t*1e9)
        feedback.apply(dict(version=1,revision=revision,input_topic='/fixture/odom',resolved=True,
                            issued_ns=stamp,expires_ns=stamp+1_000_000_000))
    def observe(segment='fixture',state='tracking'):
        estimate=scale*truth+bias*(t-10)+noise*math.sin((t-10)*7)
        stamp=int(t*1e9)
        feedback.update(dict(schema='motus.odom/1',stamp_ms=stamp//10**6,frame='body',
            units={'linear':'m/s','angular':'rad/s','length':'m'},
            twist=[scale*speed+bias,0,0,0,0,0],
            pose=dict(position=[estimate,0,0],quaternion=[0,0,0,1],frame_id='odom',child_frame_id='base_link',
                      covariance=[.001 if i%7==0 else 0 for i in range(36)]),
            vendor=dict(status=state,segment_id=segment,calibration_id='fixture',feedback_source='rgbd_imu_ekf',
                        published_ns=stamp,visual_stamp_ns=stamp,accuracy_validated=False)))
        rows.append(dict(time_s=t-10,truth_m=truth,estimated_m=estimate,target_m=1.))
    def publish(value,vyaw=0):
        nonlocal command
        if value: feedback.validate(context)
        command=value;commands.append((t-10,value))
    owner._distance_publish=publish
    lease();observe();context=feedback.freeze()
    def sleep(dt):
        nonlocal t,truth,speed,last_observation,last_lease
        # First order gait response and coast, not instantaneous target motion.
        speed+=(.55*command-speed)*min(1.,dt/.12)
        truth+=speed*dt;t+=dt
        elapsed=t-10
        if elapsed-last_lease>=.2:
            lease('changed' if fault=='connection' and elapsed>1 else 'fixture');last_lease=elapsed
        if elapsed-last_observation>=.05:
            if not (fault=='stale' and elapsed>1):
                observe('new' if fault=='reset' and elapsed>1 else 'fixture',
                        'lost' if fault=='lost' and elapsed>1 else 'tracking')
            last_observation=elapsed
        if fault=='cancel' and elapsed>1: owner._rotate_stop_event.set()
    import noetix.bumi.odometry_feedback as module
    with ExitStack() as stack:
        stack.enter_context(patch.object(module.time,'monotonic',lambda:t))
        stack.enter_context(patch.object(module.time,'sleep',sleep))
        result=run_fused_distance(owner,1.,context)
    return dict(data_source='synthetic_control_model',fusion_algorithm_executed=False,
                target_m=1.,truth_m=truth,estimated_m=result['estimated_distance_m'],
                estimation_error_m=result['estimated_distance_m']-truth,
                control_error_m=truth-1.,result=result,rows=rows,final_command=command)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args()
    report={'nominal':trial(),'scale_error':trial(scale=1.15),
            'faults':{name:trial(fault=name) for name in ('lost','stale','reset','connection','cancel')}}
    from pathlib import Path
    Path(args.output).write_text(json.dumps(report,indent=2,allow_nan=False))
    for name,data in [(k,v) for k,v in report.items() if k!='faults']:
        print(name,json.dumps({k:data[k] for k in ('truth_m','estimated_m','estimation_error_m','control_error_m')}))


if __name__=='__main__': main()
