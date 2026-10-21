"""Actual RTAB-Map/EKF synthetic RGB-D/IMU integration (no robot or USB).

Run inside the sidecar image with ROS_DOMAIN_ID=239. Renders a fixed textured
wall from a camera attached to an independently moving body. Never commands a
chassis. This validates software integration, not real-world metric accuracy.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import time


def truth(elapsed): return .25*max(0.,min(4.,elapsed-7.))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='239': raise SystemExit('Use isolated ROS_DOMAIN_ID=239')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    try: from .runtime import FusionRuntime
    except ImportError: from runtime import FusionRuntime
    calibration=dict(calibration_id='SYNTHETIC_ONLY',fixed_mount=True,
        camera_to_body={'translation':[.1,0,.3],'quaternion':[-.5,.5,-.5,.5]},
        imu_to_body={'translation':[0,0,0],'quaternion':[0,0,0,1]},
        imu_units='m/s2,rad/s',imu_quaternion='xyzw_sensor_to_enu',gravity_included=True,
        gyro_bias=[0,0,0],gyro_variance=[.001]*3,
        timing={'mode':'host_read_qualified','offset_s':0.,'uncertainty_s':0.,
                'qualification_id':'SYNTHETIC_ONLY','max_age_s':.25})
    rclpy.init(); runtime=FusionRuntime({'calibration':calibration,'source_namespace':'fusion_probe'})
    producer=Node('bumi_fusion_rendered_probe'); root='/fusion_probe/odometry_input/'
    pubs={name:producer.create_publisher(cls,root+name,qos_profile_sensor_data) for name,cls in
          (('rgb',Image),('depth',Image),('info',CameraInfo),('camera_meta',String),('imu',String))}
    rng=np.random.default_rng(408)
    texture=rng.integers(0,256,(2048,2048,3),dtype=np.uint8)
    texture=cv2.GaussianBlur(texture,(3,3),0)
    u,v=np.meshgrid(np.arange(640,dtype=np.float32),np.arange(480,dtype=np.float32))
    sequence=0; last_image=-1.; started=time.monotonic(); rows=[]; result={}
    runtime.start()
    try:
        while time.monotonic()-started<14.:
            elapsed=time.monotonic()-started; p=truth(elapsed)
            now=producer.get_clock().now().nanoseconds; sequence+=1
            imu={'epoch':'synthetic-imu','sequence':sequence,'source_stamp_ns':None,'stamp_source':'host_read',
                 'read_begin_ns':now,'read_end_ns':now,'quaternion':[0,0,0,1],'gyro':[0,0,0],
                 'acceleration':[0,0,9.81]}
            pubs['imu'].publish(String(data=json.dumps(imu)))
            if elapsed-last_image>=.10:
                last_image=elapsed; depth=4.-(p+.1)
                # B->optical: right=-body Y, down=-body Z, forward=body X.
                wall_y=-(u-320.)/600.*depth; wall_z=.3-(v-240.)/600.*depth
                rgb=cv2.remap(texture,(wall_y+4.)/.004,(wall_z+4.)/.004,
                              cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
                z=np.full((480,640),depth,dtype='<f4')
                for name,pixels,encoding,bpp in (('rgb',rgb,'bgr8',3),('depth',z,'32FC1',4)):
                    msg=Image();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now,10**9)
                    msg.header.frame_id='fusion_probe_rgb_optical';msg.width=640;msg.height=480
                    msg.step=640*bpp;msg.encoding=encoding;msg.data=pixels.tobytes();pubs[name].publish(msg)
                info=CameraInfo();info.header=msg.header;info.width=640;info.height=480
                info.distortion_model='plumb_bob';info.d=[0.]*5
                info.k=[600.,0.,320.,0.,600.,240.,0.,0.,1.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                info.p=[600.,0.,320.,0.,0.,600.,240.,0.,0.,0.,1.,0.];pubs['info'].publish(info)
                source={'source_ms':now/1e6,'frame_number':sequence,'domain':'global_time'}
                meta={'epoch':'synthetic-camera','rgb':source,'depth':source,'depth_scale':.001,
                      'rgb_stamp_ns':now,'depth_stamp_ns':now,'received_ns':now,
                      'clock_mapping':'synthetic_shared_clock'}
                pubs['camera_meta'].publish(String(data=json.dumps(meta)))
            rclpy.spin_once(runtime.node,timeout_sec=.005)
            sample=runtime.publish()
            if sample['vendor']['status']=='lost': raise AssertionError(sample['vendor'])
            if sample['pose'] is not None:
                rows.append({'elapsed_s':elapsed,'truth_m':p,'estimated_position':sample['pose']['position'],
                             'estimate_stamp_ms':sample['stamp_ms'],'segment_id':sample['vendor']['segment_id']})
            time.sleep(.005)
        first=[r for r in rows if 5.5<r['elapsed_s']<6.5]
        last=[r for r in rows if 12.5<r['elapsed_s']<13.5]
        if len(first)<5 or len(last)<5: raise AssertionError('insufficient tracking coverage')
        moved=float(np.mean([r['estimated_position'][0] for r in last])-np.mean([r['estimated_position'][0] for r in first]))
        if abs(moved-1.)>.08: raise AssertionError('synthetic displacement error '+str(moved-1.))
        # Stop inputs but keep the sidecar running: EKF predictions must not hide loss.
        deadline=time.monotonic()+1.
        while time.monotonic()<deadline: rclpy.spin_once(runtime.node,timeout_sec=.02)
        if runtime.publish()['vendor']['status']!='lost': raise AssertionError('input loss hidden')
        old=runtime.segment;runtime.reset()
        if runtime.segment==old: raise AssertionError('reset did not replace coordinate segment')
        result.update(integration_passed=True,estimated_distance_m=moved,truth_distance_m=1.,error_m=moved-1.)
    except Exception as exc:
        result.update(integration_passed=False,error=str(exc))
    finally:
        result.update(data_source='synthetic_rendered_rgbd',hardware_accuracy_validated=False,rows=rows)
        Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False))
        runtime.close();producer.destroy_node();rclpy.shutdown()
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    return 0 if result.get('integration_passed') else 1


if __name__=='__main__': raise SystemExit(main())
