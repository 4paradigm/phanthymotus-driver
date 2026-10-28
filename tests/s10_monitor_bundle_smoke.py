"""Synthetic full monitoring bundle; Docker --network none only, never hardware."""
from __future__ import annotations
import argparse,json,os,signal,struct,subprocess,sys,tempfile,time
from pathlib import Path
sys.path[:0]=['/work','/work/deep_robotics/s10_pro','/work/tests']


def main():
    from s10_network_isolation import require_loopback_network
    require_loopback_network()
    from bridge import ReaderBridge,reader_environment
    from main import PlatformBus,build_runtime_plugins,validate_platform_profile
    import yaml
    seconds=float(os.environ.get('S10_SYNTHETIC_SECONDS','30'))
    assert 10<=seconds<=900
    profile=os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'];validate_platform_profile(profile)
    code=r'''import array,struct,time,rclpy
from drdds.msg import BatteryData,BatteryDataValue,MotionInfo,JointsData,StdMsgInt32,FaultEventArray,FaultEvent,Exception,CPUStatus,MemoryStatus,FileSystemStatus,NetworkStatus
from sensor_msgs.msg import Imu,PointCloud2,PointField
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions
rclpy.init(domain_id=0,signal_handler_options=SignalHandlerOptions.NO)
n=rclpy.create_node('synthetic_s10_monitor_fixture',enable_rosout=False,start_parameter_services=False)
specs=[('/BATTERY_DATA',BatteryData),('/MOTION_INFO',MotionInfo),('/JOINTS_DATA_10HZ',JointsData),('/HES_STATUS',StdMsgInt32),('/fault_aggregator',FaultEventArray),('/fault_event',FaultEvent),('/EXCEPTION_NOTIFICATION',Exception),('/cpu_status',CPUStatus),('/memory_status',MemoryStatus),('/filesystem_status',FileSystemStatus),('/network_status',NetworkStatus),('/IMU',Imu),('/ODOM',Odometry),('/LIDAR/POINTS',PointCloud2)]
pubs={t:n.create_publisher(c,t,1) for t,c in specs}
messages={t:c() for t,c in specs}
b=messages['/BATTERY_DATA'];v=BatteryDataValue();v.battery_level=59;v.voltage=480;v.battery_temperature=[25.];b.data=[v]
m=messages['/MOTION_INFO'];m.data.motion_state.state=0;m.data.gait_state.gait=0
j=messages['/JOINTS_DATA_10HZ']
for item in j.data.joints_data:item.motion_temp=30.;item.driver_temp=31.
messages['/HES_STATUS'].value=0
messages['/EXCEPTION_NOTIFICATION'].source='synthetic_fixture';messages['/EXCEPTION_NOTIFICATION'].description='TEST ONLY';messages['/EXCEPTION_NOTIFICATION'].is_abnormal=False
for key in ['/cpu_status','/memory_status','/filesystem_status','/network_status']:messages[key].soc_id='SYNTHETIC_CPU'
messages['/cpu_status'].package_temp=30;messages['/cpu_status'].avg_util=12
messages['/memory_status'].mem_total_kb=1000;messages['/memory_status'].mem_available_kb=900
messages['/filesystem_status'].mount_point='/synthetic';messages['/filesystem_status'].total_bytes=1000;messages['/filesystem_status'].available_bytes=900
messages['/network_status'].iface='lo';messages['/network_status'].is_up=True
messages['/IMU'].header.frame_id='SYNTHETIC_TEST';messages['/IMU'].orientation.w=1.
messages['/ODOM'].header.frame_id='SYNTHETIC_TEST';messages['/ODOM'].child_frame_id='SYNTHETIC_BODY'
c=messages['/LIDAR/POINTS'];c.height=1;c.width=12000;c.point_step=16;c.row_step=c.width*16;c.is_dense=True;c.header.frame_id='SYNTHETIC_TEST'
c.fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(['x','y','z','intensity'])]
c.data=array.array('B',b''.join(struct.pack('<4f',(i%100)/10.,(i//100)/10.,(i%13)/10.,float(i%255)) for i in range(c.width)))
lastslow=0.
while rclpy.ok():
 now=time.monotonic();stamp=n.get_clock().now().to_msg()
 for topic,msg in messages.items():
  if topic in ['/MOTION_INFO','/HES_STATUS'] or now-lastslow>=.5:
   if hasattr(msg,'header') and hasattr(msg.header,'stamp'):msg.header.stamp=stamp
   if hasattr(msg,'timestamp'):msg.timestamp=stamp
   pubs[topic].publish(msg)
 if now-lastslow>=.5:lastslow=now
 rclpy.spin_once(n,timeout_sec=.04)
'''
    result={'synthetic_isolated_test':True,'hardware_test':False,'duration_s':seconds,'passed':False}
    with tempfile.TemporaryDirectory(prefix='s10-monitor-smoke-') as temp:
        config={'register':False,'reader':{'domain_id':0,'vendor_profile':profile},'motion':{'enabled':True}}
        path=Path(temp)/'config.yaml';path.write_text(yaml.safe_dump(config))
        fixture=subprocess.Popen([sys.executable,'-u','-c',code],env=reader_environment(config),stdout=subprocess.DEVNULL)
        bus=PlatformBus(42);sensors=ReaderBridge(config,path);motion=ReaderBridge(config,path)
        counts={};last={}
        def publish(topic,fmt,payload):
            counts[topic]=counts.get(topic,0)+1;last[topic]=(fmt,payload)
            bus.publish(topic,fmt,payload)
        plugins=build_runtime_plugins(config,'synthetic',sensors,publish,motion_bridge=motion)
        cards={p.name:p for p in plugins}
        def until(predicate,limit=15):
            deadline=time.monotonic()+limit
            while time.monotonic()<deadline:
                if predicate():return
                time.sleep(.05)
            raise AssertionError('synthetic monitoring condition timed out')
        try:
            for p in plugins:p.start()
            robot_cards=['state','joints','imu','odometry','lidar','motion_info','device_status','hard_estop','ros_graph']
            until(lambda:all(cards[n].status()['fresh'] for n in robot_cards))
            until(lambda:cards['errors'].status()['summary']['current_fault_state']=='no_active_faults_reported')
            cycles=[]
            for cycle in range(3):
                for name in ('imu','lidar','hard_estop','motion_events','capabilities','device','ros_graph'):
                    cards[name].stop();before=counts.get(cards[name].topic,0);time.sleep(.06)
                    assert counts.get(cards[name].topic,0)==before
                    assert cards['motion'].status()['readiness']['feedback']['fresh']
                    cards[name].start()
                until(lambda:all(cards[n].status()['fresh'] for n in robot_cards))
                cycles.append({'round':cycle+1,'independent_stop_start':True})
            samples=0;max_age=0.;started=time.monotonic();next_progress=started+30
            while time.monotonic()-started<seconds:
                info=cards['motion'].status();age=info['readiness']['feedback']['age_s']
                assert info['readiness']['feedback']['fresh'] and age<=.5
                assert not info['execution_authorized'] and info['active_request'] is None
                assert info['availability_detail']['command_worker_running'] is False
                assert sensors.snapshot('lidar')['fresh']
                samples+=1;max_age=max(max_age,age)
                if time.monotonic()>=next_progress:
                    print(json.dumps({'synthetic_progress_s':round(time.monotonic()-started),'samples':samples,'max_motion_age_s':max_age}),flush=True)
                    next_progress=time.monotonic()+30
                time.sleep(.1)
            resource=cards['device_status'].status()['summary']['compute_resources']
            assert all(resource[k]['items'] and resource[k]['fresh'] for k in ('cpu_status','memory_status','filesystem_status','network_status'))
            for p in plugins:p.stop()
            until(lambda:not sensors.status()['reader_running'] and not motion.status()['reader_running'])
            result.update(passed=True,card_count=len(cards),sample_count=samples,max_motion_age_s=max_age,
                          three_independent_rounds=cycles,reader_exit=True,command_worker_running=False,
                          available_card_names=list(cards),publication_counts=counts,
                          note='Synthetic monitoring only. This does not validate robot ownership, cancellation, real point-cloud rendering, or the physical canvas.')
        finally:
            for p in plugins:
                p.stop()
                if hasattr(p,'close'):p.close()
            sensors.close();motion.close();bus.close();fixture.terminate()
            try:fixture.wait(timeout=3)
            except subprocess.TimeoutExpired:fixture.kill();fixture.wait()
            print(json.dumps(result,sort_keys=True),flush=True)
            if not result['passed'] and sys.exc_info()[0] is None:raise SystemExit(2)


if __name__=='__main__':main()
