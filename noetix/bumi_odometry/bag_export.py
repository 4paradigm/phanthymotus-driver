"""Export body poses and independent visual/IMU rates from a ROS2 sqlite3 bag.

Run in the validation image. Refuses overwrite. Pose CSV is compatible with
experiments/bumi_odometry/evaluate.py; synthetic bags remain synthetic evidence.
"""
import argparse
import csv
import json
import math
from contextlib import ExitStack
from pathlib import Path
from common.fusion_odometry import quat_matrix,rotate


def angular_rate(previous,current,dt):
    quat_matrix(previous);quat_matrix(current)
    if not 0<dt<=.5: raise ValueError('invalid visual differentiation interval')
    x,y,z,w=previous;a,b,c,d=current
    q=[w*a-x*d-y*c+z*b,w*b+x*c-y*d-z*a,w*c-x*b+y*a-z*d,w*d+x*a+y*b+z*c]
    if q[3]<0: q=[-v for v in q]
    norm=math.hypot(*q[:3])
    if norm<1e-12: return [0.,0.,0.]
    angle=2*math.atan2(norm,q[3])
    return [v/norm*angle/dt for v in q[:3]]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag');parser.add_argument('--poses');parser.add_argument('--visual-rates');parser.add_argument('--imu-rates')
    parser.add_argument('--calibration',help='Measured IMU-to-body rotation and gyro bias for rate export')
    args=parser.parse_args()
    if not any((args.poses,args.visual_rates,args.imu_rates)): parser.error('choose an output')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    mount=json.loads(Path(args.calibration).read_text()) if args.calibration else None
    if args.imu_rates and mount is None: parser.error('IMU body-rate export requires measured calibration')
    previous={}
    with ExitStack() as stack:
        outputs={}
        for name,path in (('poses',args.poses),('visual',args.visual_rates),('imu',args.imu_rates)):
            if path:
                writer=csv.writer(stack.enter_context(Path(path).open('x')));outputs[name]=writer
                writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','status','segment_id','fault_stamp_ns'] if name=='poses'
                                else ['stamp_ns','wx','wy','wz'])
        while reader.has_next():
            topic,raw,receipt=reader.read_next();msg=deserialize_message(raw,types[topic])
            if topic=='/bumi_rgbd_imu_odometry/state/odom' and 'poses' in outputs:
                data=json.loads(msg.data);v=data['vendor'];pose=data.get('pose')
                values=pose['position']+pose['quaternion'] if pose else ['']*7
                outputs['poses'].writerow([data['stamp_ms']*10**6]+values+[v['status'],v['segment_id'],
                                         v.get('fault_stamp_ns',v['published_ns']) if v['status']=='lost' else ''])
            elif topic.endswith('/odometry_input/imu') and 'imu' in outputs:
                data=json.loads(msg.data)
                gyro=[v-b for v,b in zip(data['gyro'],mount['gyro_bias'])]
                gyro=rotate(quat_matrix(mount['imu_to_body']['quaternion']),gyro)
                outputs['imu'].writerow([(data['read_begin_ns']+data['read_end_ns'])//2]+gyro)
            elif topic.endswith('/visual') and 'visual' in outputs:
                stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                q=msg.pose.pose.orientation;current=[q.x,q.y,q.z,q.w]
                if topic in previous:
                    pt,pq=previous[topic]
                    outputs['visual'].writerow([(stamp+pt)//2]+angular_rate(pq,current,(stamp-pt)/1e9))
                previous[topic]=stamp,current


if __name__=='__main__': main()
