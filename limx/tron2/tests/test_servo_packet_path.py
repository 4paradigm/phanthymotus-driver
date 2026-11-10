"""Synthetic commands -> real card/sink/interpolator/transport -> captured WS.

No policy, no robot socket. The optional DDS test uses a private ROS context
inside a network-isolated container, never the live robot namespace.
"""
import asyncio
import json
import math
import os
import time
from types import SimpleNamespace
import unittest

from arm_servo import ArmServoPlugin, SERVO_LOWER, SERVO_UPPER
from state import configuration
from vendor import RobotReader
from test_arm_servo import Reader, Bus, q0, message


class CapturedSocket:
    closed = False

    def __init__(self, feedback):
        self.feedback = feedback
        self.packets = []

    async def send_json(self, packet):
        # Deep-copy the actual encoded request and simulate matching encoder feedback.
        saved = json.loads(json.dumps(packet, allow_nan=False))
        self.packets.append(saved)
        self.feedback.q = list(saved['data']['q'])


def real_transport_plugin():
    feedback = Reader()
    reader = RobotReader({'serial': '', 'poll_interval_seconds': .5,
        'stale_after_seconds': 5, 'response_timeout_seconds': 5})
    reader.connected = True
    reader.serial = 'DACH_TRON2A_TEST'
    reader.configuration = configuration(reader.serial)
    reader.ws = CapturedSocket(feedback)
    reader.sample = feedback.sample
    adapter = SimpleNamespace(reader=reader, motion=SimpleNamespace(active=None),
        head=SimpleNamespace(active=None), follow_runtime=None)
    card = ArmServoPlugin(adapter)
    card.publisher = Bus()
    return card, reader.ws


def target(seq=1):
    m = message(seq=seq, ttl=180)
    m['values'] = q0()
    for index, delta in ((0, .01), (7, -.01), (14, .01), (15, -.01)):
        m['values'][index] += delta
    return m


class PacketPathTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.card, self.socket = real_transport_plugin()
        await self.card.call({'action': 'start', 'input_topic': '/test/servo_packets'})

    async def asyncTearDown(self):
        await self.card.close()

    async def publish(self, value):
        self.card.publisher.callback(json.dumps(value))
        for _ in range(20):
            await asyncio.sleep(.005)
            if self.socket.packets:
                return

    async def test_receive_to_real_vendor_packet_and_joint_order(self):
        await self.publish(target())
        self.assertGreater(len(self.socket.packets), 0)
        await asyncio.sleep(.015)
        # Let an in-flight send finish before comparing submission counters.
        self.card._clear()
        await asyncio.wait_for(asyncio.shield(self.card.output_task), 1)
        for packet in self.socket.packets:
            self.assertEqual(set(packet), {'title', 'accid', 'guid', 'timestamp', 'data'})
            self.assertEqual(packet['title'], 'request_servoj')
            self.assertEqual(packet['accid'], 'DACH_TRON2A_TEST')
            self.assertIsInstance(packet['timestamp'], int)
            self.assertTrue(packet['guid'])
            self.assertEqual(set(packet['data']), {'q', 'filter_ratio'})
            self.assertEqual(packet['data']['filter_ratio'], 1.0)
            q = packet['data']['q']
            self.assertEqual(len(q), 16)
            self.assertTrue(all(math.isfinite(v) and lo <= v <= hi
                for v, lo, hi in zip(q, SERVO_LOWER, SERVO_UPPER)))
            self.assertGreater(q[0], q0()[0])
            self.assertLess(q[7], q0()[7])
            self.assertGreater(q[14], q0()[14])
            self.assertLess(q[15], q0()[15])
            for i in set(range(16)) - {0, 7, 14, 15}:
                self.assertAlmostEqual(q[i], q0()[i], places=12)
        self.assertEqual(self.card.info()['commands_submitted'], len(self.socket.packets))
        self.assertFalse(self.card.info()['vendor_acknowledged'])
        self.assertFalse(self.card.info()['arrival_verified'])

    async def test_rejected_input_cannot_reach_real_sender(self):
        for patch in ({'mode': 'twist'}, {'dof': 14}, {'stamp_ms': 1},
                      {'values': [99.] * 16}, {'values': [float('nan')] * 16}):
            m = target();m.update(patch)
            self.card.publisher.callback(json.dumps(m))
            await asyncio.sleep(.01)
            self.assertEqual(self.socket.packets, [])
        self.assertEqual(self.card.commands, 0)

    async def test_expiry_and_pause_cancel_actual_packet_output(self):
        await self.publish(target())
        self.assertGreater(len(self.socket.packets), 0)
        await asyncio.sleep(.25)
        count = len(self.socket.packets)
        await asyncio.sleep(.025)
        self.assertEqual(len(self.socket.packets), count)
        self.assertIsNone(self.card.reader._pose_servo)
        await self.card.call({'action': 'pause'})
        self.card.publisher.callback(json.dumps(target(2)))
        await asyncio.sleep(.02)
        self.assertEqual(len(self.socket.packets), count)


@unittest.skipUnless(os.environ.get('TRON2_TEST_PRIVATE_DDS') == '1',
                     'requires explicitly isolated ROS/DDS test container')
class PrivateDdsPacketTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_dds_to_captured_vendor_output(self):
        from ros2 import JsonPublisher
        # Separate network and IPC namespaces are mandatory in the runner.
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '213')
        topic = '/tron2_private_test/servo_command'
        card, socket = real_transport_plugin()
        bus = JsonPublisher([topic])
        card.publisher = bus
        try:
            await card.call({'action': 'start', 'input_topic': topic})
            deadline = time.monotonic() + 4
            while bus.publishers[topic].get_subscription_count() < 1 and time.monotonic() < deadline:
                await asyncio.sleep(.02)
            self.assertGreater(bus.publishers[topic].get_subscription_count(), 0)
            for seq in range(1, 9):
                bus.publish(topic, target(seq))
                await asyncio.sleep(.04)
            self.assertGreater(card.applied, 0)
            self.assertGreater(len(socket.packets), 0)
            q = socket.packets[-1]['data']['q']
            self.assertEqual(len(q), 16)
            self.assertGreater(q[0], q0()[0]);self.assertLess(q[7], q0()[7])
            self.assertGreater(q[14], q0()[14]);self.assertLess(q[15], q0()[15])
            await card.call({'action': 'pause'})
            count = len(socket.packets)
            bus.publish(topic, target(9))
            await asyncio.sleep(.04)
            self.assertEqual(len(socket.packets), count)
            print('DDS_PACKET_PROOF '+json.dumps({'received_targets': card.applied,
                'captured_servoj_packets': count, 'dof': len(q),
                'joint_directions_verified': ['left_arm', 'right_arm', 'head_pitch', 'head_yaw'],
                'robot_network_opened': False, 'pause_prevents_output': True}))
        finally:
            await card.close()
            bus.close()


if __name__ == '__main__':
    unittest.main()
