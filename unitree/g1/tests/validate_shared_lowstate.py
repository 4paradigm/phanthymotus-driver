"""Synthetic DDS/ROS integration; run with --network none and no device mounts."""
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import socket

if (os.environ.get('G1_TELEOP_ISOLATED_TEST') != '1' or any(
        name != 'lo' and int(Path('/sys/class/net', name, 'flags').read_text(), 16) & 1
        for _, name in socket.if_nameindex())):
    raise SystemExit('Use a network-isolated container and G1_TELEOP_ISOLATED_TEST=1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_dds_receive_threads import initialize


def writer():
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    sdk = initialize()
    publisher = sdk.ChannelPublisher('rt/lowstate', LowState_); publisher.Init()
    time.sleep(1)
    msg = unitree_hg_msg_dds__LowState_()
    for motor in msg.motor_state: motor.mode = 1
    start = time.monotonic(); count = 0; first = last = None
    while time.monotonic()-start < 3:
        before = time.monotonic_ns()
        msg.tick = (before//1_000_000) & 0xffffffff
        assert publisher.Write(msg)
        first = before if first is None else first
        last = time.monotonic_ns(); count += 1
        time.sleep(max(0., start+count*.001-time.monotonic()))
    time.sleep(.5); publisher.Close()
    print(json.dumps({'count':count, 'first_send_ns':first, 'last_send_ns':last}))


def check():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from cyclonedds.sub import DataReader
    from arm_stream import ArmStreamExecutor, MOTOR_IDS
    from arm_sdk import ArmSdkChannel
    from device import _LowStateNode, _LOW_LAT_QOS
    from low_state import LowStateSource

    from sdk_adapter import FeedbackSubscriber
    initialize(); opened = []; decoded = [0]
    init = FeedbackSubscriber.__init__
    def track(self, topic, dtype):
        init(self, topic, dtype); opened.append((topic, self))
    FeedbackSubscriber.__init__ = track
    take = DataReader.take
    def count_take(self, *args, **kwargs):
        samples = take(self, *args, **kwargs)
        decoded[0] += sum(hasattr(sample, 'tick') for sample in samples)
        return samples
    DataReader.take = count_take
    rclpy.init(); executor = SingleThreadedExecutor()
    observer = Node('shared_lowstate_observer'); executor.add_node(observer)
    source = LowStateSource(); arm = ArmStreamExecutor({}, 'test', None, low_state=source)
    child = None
    try:
        for iteration in range(2):
            source.start(); arm.start(); before_decode = decoded[0]
            state = _LowStateNode('/probe/imu','/probe/battery','/probe/joints','/probe/mainboard', low_state=source)
            executor.add_node(state)
            counts = {'imu':[], 'joints':[]}; ages = []; initialized = False
            subs = [observer.create_subscription(String, '/probe/'+name,
                lambda msg, name=name:counts[name].append(time.monotonic()), _LOW_LAT_QOS) for name in counts]
            active_lowstate = [sub for topic,sub in opened if topic == 'rt/lowstate'
                and sub._channel is not None]
            assert len(active_lowstate) == 1, 'duplicate LowState reader'
            child = subprocess.Popen([sys.executable, __file__, 'writer'], stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic()+7
            while child.poll() is None and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.005)
                feedback = arm._snapshot(); now = time.monotonic_ns()
                if feedback.get('source_tick') is not None:
                    if not initialized and 0 <= now-feedback['arm_ns'] < 50_000_000:
                        channel = ArmSdkChannel(grippers=False, driven_arm_ids=MOTOR_IDS, low_state=source)
                        assert len(channel._read_measured_arms()) == 10
                        assert sum(topic == 'rt/lowstate' for topic, _ in opened) == iteration+1
                        initialized = True
                    ages.append((now, ((now//1_000_000)-feedback['source_tick']) & 0xffffffff,
                                 (now-feedback['arm_ns'])/1e6))
            out, err = child.communicate(timeout=2)
            assert child.returncode == 0, err
            publication = json.loads(out)
            active = [row for row in ages if publication['first_send_ns'] <= row[0] <= publication['last_send_ns']]
            rates = {name:(len(stamps)-1)/(stamps[-1]-stamps[0]) for name,stamps in counts.items()}
            summary = {'iteration':iteration, 'published':publication['count'], 'decoded':decoded[0]-before_decode,
                'readers':len(active_lowstate), 'rates_hz':rates,
                'source_age_ms_median':statistics.median(row[1] for row in active),
                'source_age_ms_max':max(row[1] for row in active),
                'reported_age_ms_max':max(row[2] for row in active)}
            print(json.dumps(summary), flush=True)
            assert publication['count'] > 2000 and len(active) > 100 and initialized
            assert 19 < rates['imu'] < 21 and 9 < rates['joints'] < 11, summary
            assert max(row[1] for row in active) < 100 and max(row[2] for row in active) < 100, summary
            assert state.get_posture()
            frozen = source.snapshot(); feedback = arm._snapshot(); count = len(counts['imu'])
            until = time.monotonic()+.2
            while time.monotonic()<until: executor.spin_once(timeout_sec=.005)
            assert source.snapshot() is frozen and arm._snapshot()['arm_ns'] == feedback['arm_ns']
            assert len(counts['imu']) == count, 'old sample republished'
            assert arm.stop()['authority_released'] and source.snapshot() is frozen
            executor.remove_node(state); state.destroy_node(); state.destroy_node()
            for sub in subs: observer.destroy_subscription(sub)
            source.stop(); source.stop()
            assert source.snapshot() is None and arm._snapshot()['arm_ns'] == 0
            for _, sub in opened: sub.Close()
        print('SHARED_LOWSTATE_PASS hardware_output=false')
    finally:
        if child and child.poll() is None: child.kill(); child.wait()
        arm.stop(); source.stop()
        for _, sub in opened: sub.Close()
        executor.remove_node(observer); observer.destroy_node(); executor.shutdown(); rclpy.shutdown()
        DataReader.take = take; FeedbackSubscriber.__init__ = init


if __name__ == '__main__':
    writer() if len(sys.argv)>1 and sys.argv[1]=='writer' else check()
