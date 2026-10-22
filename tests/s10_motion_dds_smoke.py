"""Real DDS pipeline against a synthetic robot, guarded by network isolation.

Run only in Docker --network none. It is not a physical motion test. A fictional
lease/mode/deadman contract is explicitly injected solely inside this container.
"""
from __future__ import annotations
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path('/work')
sys.path.insert(0, str(ROOT))


def main():
    if {name for _, name in socket.if_nameindex()} != {'lo'}:
        raise RuntimeError('synthetic motion test requires Docker --network none')
    from deep_robotics.s10_pro.bridge import ReaderBridge, reader_environment
    from deep_robotics.s10_pro.motion import MotionPlugin
    from deep_robotics.s10_pro.motion_backend import DDSMotionBackend
    from deep_robotics.s10_pro.motion_process import DDSProcessTransport
    import yaml
    config = {'reader': {'domain_id': 0, 'vendor_profile': '/opt/phanthy-motus/dds-local.xml'}, 'register': False}
    temp = tempfile.TemporaryDirectory(prefix='s10-fictional-motion-')
    base = Path(temp.name)
    config_path = base / 'config.yaml'; config_path.write_text(yaml.safe_dump(config))
    log_path = base / 'fake-robot-commands.json'
    code = '''import json,os,socket,time,rclpy
from pathlib import Path
from drdds.msg import MotionInfo,MotionState,NavCmd
from rclpy.signals import SignalHandlerOptions
if {n for _,n in socket.if_nameindex()} != {'lo'}: raise RuntimeError('isolated fake robot only')
rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
n=rclpy.create_node('synthetic_motion_robot',enable_rosout=False,start_parameter_services=False)
pub=n.create_publisher(MotionInfo,'/MOTION_INFO',10)
state=0; velocity=[0.,0.,0.]; commands=[]
def save():Path(os.environ['S10_FAKE_COMMAND_LOG']).write_text(json.dumps(commands))
def posture(m):
 global state,velocity
 state=m.data.state;velocity=[0.,0.,0.];commands.append({'state':state,'received':time.monotonic()});save()
def speed(m):
 global velocity
 velocity=[m.data.x_vel,m.data.y_vel,m.data.yaw_vel];commands.append({'velocity':velocity,'received':time.monotonic()});save()
a=n.create_subscription(MotionState,'/MOTION_STATE',posture,10)
b=n.create_subscription(NavCmd,'/NAV_CMD',speed,10)
while rclpy.ok():
 m=MotionInfo();m.header.stamp=n.get_clock().now().to_msg();m.data.motion_state.state=state;m.data.gait_state.gait=0x3002
 m.data.vel_x,m.data.vel_y,m.data.vel_yaw=velocity;pub.publish(m);rclpy.spin_once(n,timeout_sec=.04)
'''
    fake = subprocess.Popen([sys.executable, '-u', '-c', code], env={**reader_environment(config), 'S10_FAKE_COMMAND_LOG': str(log_path)}, stdout=subprocess.DEVNULL)
    bridge = ReaderBridge(config, config_path)
    transport = DDSProcessTransport(config, execution_enabled=True)
    policy = {'verified': True, 'evidence': {key: 'FICTITIOUS: isolated synthetic robot only' for key in
        ['identity','firmware','authority','feedback','stop','watchdog']}, 'robot_identity_matched': True,
        'firmware_matched': True, 'robot_deadman_verified': True, 'robot_deadman_s': .8,
        'send_timeout_s': .2, 'actions': ['stand','lie','velocity','stopmotion'],
        'velocity_units': {'vx':'m/s','vy':'m/s','yaw_rate':'rad/s'},
        'stop_semantics':'zero_velocity_until_feedback','stop_on_authority_loss':False}
    def fictional_lease():
        return {'owned':True,'fresh':True,'age_s':0.,'received_monotonic':time.monotonic(),'navigation_mode_confirmed':True}
    backend = DDSMotionBackend(bridge, transport, contract=policy, authority_provider=fictional_lease, execution_authorized=True)
    plugin = MotionPlugin({'enabled':True,'action_timeout_s':5}, 'isolated', backend,
        feedback=backend.feedback, execution_authorized=True)
    result = {'synthetic_isolated_test':True,'physical_robot':False,'transport':'ros2_dds','passed':False}
    def until(predicate, seconds=10):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.03)
        raise AssertionError('isolated DDS condition timed out')
    try:
        plugin.start();transport.start()
        until(lambda: backend.feedback.observation()['fresh'])
        until(lambda: all(v['subscriber_count']>0 for v in transport.graph_snapshot().values()))
        result['stand']=plugin.dispatch('stand',{})
        assert result['stand']['completed'] and result['stand']['accepted'] is None
        result['velocity']=plugin.dispatch('velocity',{'vx':.1,'duration':.8})
        assert result['velocity']['completed'] and result['velocity']['stop_confirmed'] and result['velocity']['accepted'] is None
        result['lie']=plugin.dispatch('lie',{})
        assert result['lie']['completed']
        commands=json.loads(log_path.read_text())
        states=[v['state'] for v in commands if 'state' in v]
        assert states == [1,17,4]
        speeds=[v for v in commands if 'velocity' in v]
        assert len([v for v in speeds if v['velocity'][0]>.09])>=5
        assert all(abs(v)<1e-6 for v in speeds[-1]['velocity'])
        moving=[v['received'] for v in speeds if v['velocity'][0]>.09]
        result['moving_frame_max_gap_s']=max(b-a for a,b in zip(moving,moving[1:]))
        assert result['moving_frame_max_gap_s']<.2
        result['state_sequence']=states
        result['velocity_frames']=len(speeds)
        result['command_worker_separate_process']=transport.status()['worker_running']
        plugin.stop();backend.close();bridge.close()
        result['worker_closed']=not transport.status()['worker_running']
        result['passed']=True
    finally:
        plugin.close();bridge.close()
        fake.terminate()
        try:fake.wait(timeout=3)
        except subprocess.TimeoutExpired:fake.kill();fake.wait(timeout=2)
        print(json.dumps(result,ensure_ascii=False,allow_nan=False))
        temp.cleanup()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
