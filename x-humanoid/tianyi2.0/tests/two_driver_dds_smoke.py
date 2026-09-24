"""Real loopback DDS + isolated IK + finite-speed plant, never robot hardware.

Run only in a network-none container, with read-only source mounts and /tmp
writable. This harness imports each Driver's own common package in a separate
process. It replaces vendor feedback/publishers with the existing offline
Plant, while retaining real DeviceRuntime, ROS transport,
TeleopControl, NumericalWorker, arm receiver and MotionGate.
"""
import argparse
import ast
import asyncio
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


def guard():
    if os.environ.get('TIANYI_ISOLATED_DDS') != '1':
        raise RuntimeError('set TIANYI_ISOLATED_DDS=1 only in network-none/no-devices container')
    if sys.platform != 'linux':
        raise RuntimeError('Linux network-none container required')
    # Docker Desktop may expose dormant kernel tunnel devices even with
    # --network none. Reject every non-loopback UP interface and IPv4 route.
    interfaces=[p for p in Path('/sys/class/net').iterdir() if (p/'flags').is_file()]
    if any(p.name!='lo' and int((p/'flags').read_text(),16)&1 for p in interfaces):
        raise RuntimeError('non-loopback interface is up')
    if any(line.split()[0]!='lo' for line in Path('/proc/net/route').read_text().splitlines()[1:]):
        raise RuntimeError('external route present')
    if list(Path('/dev').glob('ttyUSB*')) or list(Path('/dev').glob('ttyACM*')):
        raise RuntimeError('physical serial devices are not permitted')
    os.environ.update(ROS_DOMAIN_ID='42', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      FASTRTPS_DEFAULT_PROFILES_FILE='/opt/phanthy-motus/dds-local.xml')


def event(stream, kind, **values):
    stream.write(json.dumps({'event':kind,'monotonic_ns':time.monotonic_ns(),**values},allow_nan=False)+'\n')
    stream.flush()


def load_fixture(driver):
    """Reuse fixture source without pytest or importing hardware modules."""
    import numpy as np
    from tianyi_motion.kinematics import ARM_NAMES
    tree=ast.parse((driver/'tests/test_motion_control.py').read_text())
    nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ('profile','Plant')]
    scope={'np':np,'ARM_NAMES':ARM_NAMES,'json':json,'hashlib':hashlib,'threading':threading,'time':time}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'offline-fixtures','exec'),scope)
    return scope['profile'],scope['Plant']


def control(args):
    import numpy as np
    root=Path(args.control_root);driver=root/'x-humanoid/tianyi2.0'
    sys.path[:0]=[str(root),str(driver)]
    from motion_control import MotionControl
    from teleop_executor import TeleopExecutor
    from teleop_control import TeleopControl
    from tianyi_motion.worker import NumericalWorker
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    make_profile,Plant=load_fixture(driver)
    profile=make_profile(output);plant=Plant()
    arm=SimpleNamespace(_pos_publisher=True,_send_pos=plant.send)
    executor=TeleopExecutor({'calibration_path':str(profile),'live_enabled':True,
        'operator_session_enabled':True},'offline',None,arm,None,[])
    # Only vendor I/O and competing-hardware lookup are replaced. Real DDS
    # helper, socket IPC, status/watchdog threads and solver worker remain.
    executor.subscribe_feedback=lambda:None
    executor.foreign_publishers=lambda:[]
    executor.legacy_busy=lambda:False
    executor._capture_fixed_baseline=lambda:None
    executor.gate.snapshot=plant.snapshot
    motion=MotionControl({'calibration_path':str(profile),'joint_velocity_rad_s':1.},executor)
    executor.motion_control=motion;arm._motion_control=motion
    cls=next(n for n in ast.parse((driver/'device.py').read_text()).body if isinstance(n,ast.ClassDef) and n.name=='ArmPlugin')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='accept_control')
    scope={};exec(compile(ast.Module(body=[method],type_ignores=[]),'arm-receiver','exec'),scope)
    arm.accept_control=lambda packet:scope['accept_control'](arm,packet)
    card=TeleopControl({'mode':'live','calibration_path':str(profile),
        'state_path':str(output/'config.json')},motion)
    executor.teleop_control=card
    # Independent physical lag; watchdog itself is executor's real thread.
    def plant_loop():
        while not plant.closed.wait(.02):plant.tick()
    plant.thread=threading.Thread(target=plant_loop,daemon=True);plant.thread.start()
    stream=(output/'control.jsonl').open('w')
    try:
        card.start({'input_topic':'/teleop/command','instance_id':'control_1'})
        result=motion.dispatch('calibrate',{});assert not result.get('error'),result
        assert isinstance(motion.solver,NumericalWorker)
        poses=[]
        for amount in (0.,.1,.16):
            state=executor.gate.status();state['feedback']['q']=[amount if i in (0,7) else 0. for i in range(14)]
            state['feedback']['arm_ns']=time.monotonic_ns()
            poses.append(motion.solver.render(state,{'state':'ready'},False,0)['poses'])
        (output/'ready.json').write_text(json.dumps({'poses':poses,'ik_pid':motion.solver._process.pid,
            'control_pid':os.getpid(),'bus_pid':executor._bus_process.pid}))
        deadline=time.monotonic()+120
        while not (output/'producer.done').exists() and time.monotonic()<deadline:
            event(stream,'state',feedback=card.feedback(),command=executor._last_vendor_command,
                  plant=plant.snapshot(),writes=len(plant.writes))
            time.sleep(.05)
        if not (output/'producer.done').exists():raise RuntimeError('producer_timeout')
        result=card.stop()
        event(stream,'stopped',result=result,plant=plant.snapshot(),writes=len(plant.writes))
        assert result['authority_released'] and plant.writes
        (output/'control.result.json').write_text(json.dumps({'pass':True,'writes':len(plant.writes),
            'final_q':plant.q.tolist(),'max_command_abs_rad':float(np.max(np.abs(plant.writes)))},indent=2))
    finally:
        try:card.stop()
        finally:executor.stop();plant.close();stream.close()


async def producer(args):
    root=Path(args.pico_root);sys.path[:0]=[str(root/'pico/4ultra'),str(root)]
    import numpy as np
    from scipy.spatial.transform import Rotation
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from ext_vr.runtime import DeviceRuntime
    from ext_vr.transport import RosTransport
    from common.teleop_contract import validate_feedback
    output=Path(args.output);poses=json.loads((output/'ready.json').read_text())['poses']
    stream=(output/'producer.jsonl').open('w')
    runtime=DeviceRuntime('pico_1');runtime.start()
    authority,generation=runtime.bind_capture('offline-pico')
    connection=SimpleNamespace(connection_id='offline-connection',capture_id='offline-pico',events=asyncio.Queue(maxsize=32))
    manager=SimpleNamespace(_connection=connection,presence_expired=lambda c:False)
    rclpy.init();ros_executor=SingleThreadedExecutor()
    transport=None
    def publish(value):
        transport.publish(value)
        event(stream,'command',packet=value)
    latest={};errors=[]
    def feedback(value):
        try:
            validate_feedback(value,instance_id='pico_1',clock_id=runtime.clock_id,now_ns=runtime.clock_ns())
            latest.clear();latest.update(value)
            event(stream,'feedback',packet=value)
        except Exception as exc:errors.append(str(exc))
    transport=RosTransport(ros_executor,'offline','pico_1',None)
    # Independent Canvas-monitor subscriber, not a device feedback dependency.
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from std_msgs.msg import String
    monitor=Node('offline_canvas_monitor')
    monitor.create_subscription(String,'/teleop/state',lambda msg:feedback(json.loads(msg.data)),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE))
    ros_executor.add_node(monitor)
    basis=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
    def raw_pose(position,quaternion):
        return {'position':(basis.T@position).tolist(),
                'orientation':[*((basis.T@quaternion[:3]).tolist()),float(quaternion[3])]}
    templates=[]
    for target in poses:
        template={'head':raw_pose([0.,0.,1.6],[0.,0.,0.,1.])}
        for i,side in enumerate(('left','right')):
            origin=[0.,.3 if i==0 else -.3,1.]
            position=np.array(origin)+(np.array(target[i][:3])-poses[0][i][:3])/.5
            rotation=Rotation.from_quat(target[i][3:])*Rotation.from_quat(poses[0][i][3:]).inv()
            template[side+'_controller']=raw_pose(position,rotation.as_quat().tolist())
        templates.append(template)
    settings={'enabled':True,'grip':0.,'pose':0};sequence=0;next_input=0.
    async def pump(duration=0.,until=None,timeout=12.):
        nonlocal sequence,next_input
        deadline=time.monotonic()+(timeout if until else duration)
        while time.monotonic()<deadline:
            ros_executor.spin_once(timeout_sec=0.)
            if errors:raise RuntimeError('feedback_validation: '+str(errors))
            now=time.monotonic()
            if settings['enabled'] and now>=next_input:
                sequence+=1;grip=settings['grip']
                wire={'schema_version':1,'sequence':sequence,'client_monotonic_ns':runtime.clock_ns(),
                    'mode':'shadow','deadman':bool(grip),'clutch_sequence':0,
                    'tracking':{'head':True,'left_controller':True,'right_controller':True},
                    **templates[settings['pose']],
                    'controllers':{s:{'axes':[],'buttons':[0.,grip]} for s in ('left','right')}}
                publish(runtime.submit_rtc_frame(wire,authority=authority,rtc_generation=generation))
                next_input=now+.05
            if until and until():return
            await asyncio.sleep(.005)
        if until:raise AssertionError({'timeout':timeout,'last_feedback':latest})
    try:
        await pump(until=lambda:bool(latest.get('execution',{}).get('armed')))
        await pump(until=lambda:bool(latest.get('operator_session_id')))
        epoch=latest['mapping_epoch'];operator=latest['operator_session_id']
        assert epoch==1 and operator
        settings.update(grip=1.,pose=1)
        await pump(until=lambda: max(map(abs,latest.get('execution',{}).get('feedback',{}).get('q',[0.])))>.005)
        await pump(.4)
        for cycle in range(10):
            settings.update(grip=0.,pose=2 if cycle%2==0 else 1)
            await pump(until=lambda:latest.get('execution',{}).get('hold_confirmed'))
            before=np.asarray(latest['execution']['feedback']['q'])
            resumed_after=sequence;settings['grip']=1.
            await pump(until=lambda:latest.get('execution',{}).get('output_active')
                       and latest['source_sequence']>resumed_after)
            await pump(.25)
            delta=float(np.max(np.abs(np.asarray(latest['execution']['feedback']['q'])-before)))
            assert delta>1e-5,{'cycle':cycle,'plant_delta_rad':delta,'feedback':latest}
            assert latest['mapping_epoch']==epoch and latest['operator_session_id']==operator
            event(stream,'regrip_verified',cycle=cycle+1,mapping_epoch=epoch,
                  operator_session_id=operator,plant_delta_rad=delta)
        settings['enabled']=False
        await pump(until=lambda:latest.get('state')=='hold')
        await pump(.1)
        settings.update(enabled=True,pose=1)
        await pump(until=lambda:latest.get('execution',{}).get('output_active'))
        assert latest['mapping_epoch']==epoch and latest['operator_session_id']==operator
        await pump(.3)
        settings['enabled']=False
        summary={'pass':True,'input_frames':sequence,'mapping_epoch':epoch,
            'transport':'RosTransport/BoundedWriter','writer_error':transport.writer.last_error,
            'regrip_cycles':10,'operator_session_id':operator,'final_feedback':latest}
        (output/'producer.result.json').write_text(json.dumps(summary,indent=2))
    finally:
        runtime.stop();transport.close();ros_executor.remove_node(monitor);monitor.destroy_node();ros_executor.shutdown();rclpy.shutdown();stream.close()
        (output/'producer.done').write_text('complete\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--pico-root',default='/work/pico-src')
    parser.add_argument('--control-root',default='/work/control-src')
    parser.add_argument('--output',default='/tmp/two-driver-dds')
    parser.add_argument('--role',choices=('all','control','producer'),default='all')
    args=parser.parse_args();guard()
    if args.role=='control':return control(args)
    if args.role=='producer':return asyncio.run(producer(args))
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    if (output/'ready.json').exists():raise RuntimeError('use a fresh evidence output directory')
    invocation=[sys.executable,__file__,'--pico-root',args.pico_root,'--control-root',args.control_root,'--output',args.output]
    with (output/'control.log').open('w') as log:
        process=subprocess.Popen([*invocation,'--role','control'],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+30
            while not (output/'ready.json').exists():
                if process.poll() is not None:raise RuntimeError('control exited; inspect control.log')
                if time.monotonic()>deadline:raise RuntimeError('control startup timeout')
                time.sleep(.05)
            with (output/'producer.log').open('w') as producer_log:
                result=subprocess.run([*invocation,'--role','producer'],stdout=producer_log,
                    stderr=subprocess.STDOUT,timeout=100)
            if result.returncode:raise RuntimeError('producer failed; inspect producer.log and control.jsonl')
            if process.wait(timeout=10):raise RuntimeError('control failed; inspect control.log')
            print('ISOLATED DDS PASS: real producer, ROS DDS, IK process and finite-speed plant; no hardware')
        finally:
            (output/'producer.done').touch()
            if process.poll() is None:
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=2)


if __name__=='__main__':main()
