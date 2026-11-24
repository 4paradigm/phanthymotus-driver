"""Real DDS ingress/output on private topics; never publishes robot commands.

Use the existing Panther image with modified modules on PYTHONPATH. The device
boundary is a private geometry_msgs/Twist recorder, not /cmd_vel or its aliases.
"""
import json
import multiprocessing as mp
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from base_servo import BaseServoPlugin, ZERO

INPUT = '/panther_validation/base_servo/input'
OUTPUT = '/panther_validation/base_servo/output'


def producer():
    os.environ['ROS_DOMAIN_ID'] = '42'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    from std_msgs.msg import String
    rclpy.init(domain_id=42)
    node = Node('panther_servo_validation_producer', enable_rosout=False, start_parameter_services=False)
    pub = node.create_publisher(String, INPUT, qos_profile_sensor_data)
    deadline = time.monotonic()+5
    while not pub.get_subscription_count() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert pub.get_subscription_count() > 0, 'private control receiver not discovered'
    time.sleep(.5)
    for seq in range(10):
        now = int(time.time()*1000)
        message = dict(schema='motus.control/1', mode='twist', dof=6, source='navi_validation',
                       seq=seq, stamp_ms=now, obs_stamp_ms=now, ttl_ms=150,
                       values=[.1, -.05, 0., 0., 0., .2], priority=50)
        pub.publish(String(data=json.dumps(message)))
        time.sleep(.1)
    time.sleep(.2)
    node.destroy_node(); rclpy.shutdown()


class PrivateDevice:
    def __init__(self, node):
        self._base_gate = threading.RLock()
        self._cfg = {'plugins': {'base_drive': {'max_linear_mps': .5, 'max_angular_deg_s': 45.}}}
        self.publisher = node.create_publisher(Twist, OUTPUT, 10)
        self.attempts = []

    def base_check_available(self, owner):
        pass

    def servo_prepare(self, dry_run):
        assert not dry_run
        deadline = time.monotonic()+3
        while not self.publisher.get_subscription_count() and time.monotonic() < deadline:
            time.sleep(.05)
        assert self.publisher.get_subscription_count() > 0

    def servo_publish(self, values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = values[0], values[1], values[5]
        self.publisher.publish(msg)
        self.attempts.append((time.monotonic(), tuple(values)))
        return self.publisher.get_subscription_count() > 0


def main():
    rclpy.init(domain_id=0)
    node = Node('panther_servo_validation_output', enable_rosout=False, start_parameter_services=False)
    received = []
    node.create_subscription(Twist, OUTPUT, lambda m: received.append(
        (time.monotonic(), [m.linear.x, m.linear.y, m.angular.z])), 10)
    done = threading.Event()
    def spin():
        while not done.is_set():
            rclpy.spin_once(node, timeout_sec=.02)
    thread = threading.Thread(target=spin); thread.start()
    device = PrivateDevice(node)
    servo = BaseServoPlugin(device, {})
    source = None
    try:
        started = servo.dispatch('start', {'input_topic': INPUT})
        assert started['state'] == 'running', started
        source = mp.get_context('spawn').Process(target=producer)
        source.start(); source.join(timeout=10)
        assert not source.is_alive() and source.exitcode == 0
        time.sleep(.6)
        moving = [(t, v) for t, v in received if any(v)]
        assert moving, {'status': servo.info(), 'received': received}
        last_motion = moving[-1][0]
        first_zero = next(t for t, v in received if t > last_motion and not any(v))
        assert first_zero-last_motion < .35, first_zero-last_motion
        assert not any(received[-1][1])
        paused = servo.dispatch('pause', {})
        assert paused['stop_confirmed'] and not paused['reserved'], paused
        stopped = servo.dispatch('stop', {})
        assert stopped['state'] == 'idle' and stopped['stop_confirmed'], stopped
        assert not servo._thread.is_alive()
        print(json.dumps(dict(private_topics=[INPUT, OUTPUT], received=len(received),
                  moving_frames=len(moving), output_zero_delay_ms=round((first_zero-last_motion)*1000, 1),
                  pause=paused, stop=stopped)), flush=True)
    finally:
        servo.stop()
        if source is not None and source.is_alive():
            source.terminate(); source.join(timeout=2)
        done.set(); thread.join(timeout=2)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
