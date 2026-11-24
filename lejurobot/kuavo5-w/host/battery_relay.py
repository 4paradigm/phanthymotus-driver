#!/usr/bin/env python3
"""Read known battery wire schemas without installing the chassis SDK.

Use the exact observed type+MD5 pair, bounded TCPROS definitions and frames.
Only publishes a standard String telemetry envelope, never actuator messages.
"""
import json
import math
import threading
import time

SCHEMAS = {
    '/battery': ('leju_mobile_base_msgs/Battery', '194127313f3eab30becb94a79e4c5562'),
    '/jaten_behavior_manager/out/leju/battery': ('jaten_msgs/leju_battery', '29a3d6075735cfbefd0218e8a3a12e0d'),
    '/jaten_behavior_manager/out/leju/small_battery': ('jaten_msgs/leju_battery', '29a3d6075735cfbefd0218e8a3a12e0d'),
}
FIELDS = ('voltage', 'current', 'percentage', 'temperature', 'status', 'time_remaining', 'health', 'battery_door_di')


def main():
    import genpy.dynamic
    import rospy
    from std_msgs.msg import String
    rospy.init_node('motus_kuavo5w_battery_relay', disable_signals=False)
    lock, latest, classes = threading.RLock(), {}, {}
    publisher = rospy.Publisher('/motus/kuavo5w/battery', String, queue_size=1)

    def receive(topic, message):
        try:
            header = message._connection_header
            expected_type, expected_md5 = SCHEMAS[topic]
            if (header.get('type'), header.get('md5sum')) != (expected_type, expected_md5):
                raise ValueError('unverified battery wire schema')
            if len(message._buff) > 65536 or len(header.get('message_definition', '')) > 262144:
                raise ValueError('oversized battery message/definition')
            with lock:
                if topic not in classes:
                    cls = genpy.dynamic.generate_dynamic(expected_type, header['message_definition'])[expected_type]
                    if cls._md5sum != expected_md5:
                        raise ValueError('battery definition does not match pinned schema')
                    classes[topic] = cls
                decoded = classes[topic]().deserialize(message._buff)
                values = {name: getattr(decoded, name) for name in FIELDS if hasattr(decoded, name)}
                values = {name: None if isinstance(value, float) and not math.isfinite(value) else value
                          for name, value in values.items()}
                latest[topic] = {'received_ms': int(time.time() * 1000),
                    'message_type': expected_type, 'md5': expected_md5,
                    'values': values}
        except Exception as exc:
            rospy.logwarn_throttle(10, 'battery relay rejected %s: %s', topic, type(exc).__name__)

    subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, lambda msg, t=topic: receive(t, msg), queue_size=1)
                   for topic in SCHEMAS]
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        with lock:
            now = int(time.time() * 1000)
            sources = {topic: value for topic, value in latest.items() if 0 <= now - value['received_ms'] <= 5000}
        publisher.publish(String(data=json.dumps({'sources': sources}, allow_nan=False)))
        rate.sleep()


if __name__ == '__main__':
    main()
