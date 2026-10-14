"""Run in the ARM64 image with --network none; requires pytest for fixtures."""
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import time

import aiohttp
from aiohttp import web
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from common.teleop_contract import COMMAND_TOPIC, STATE_TOPIC, validate_feedback
from test_tron2_teleop import VendorServer, Q, frame, server


async def check():
    # A loopback-only transport plus Docker --network none excludes real hardware.
    profile = Path('/tmp/tron2-test-dds.xml')
    profile.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor><transport_id>loopback</transport_id>
    <type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="loopback_only" is_default_profile="true"><rtps>
    <userTransports><transport_id>loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
    <builtin><initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></initialPeersList></builtin>
  </rtps></participant>
</profiles>''')
    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(profile), ROS_DOMAIN_ID='42',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', TRON2_ROBOT_URL='ws://127.0.0.1:5000',
                      AGENT_CORE_URL='http://127.0.0.1:15678')
    os.environ.pop('AGENT_CORE_CA_CERT', None)
    clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    vendor, monitors, registrations = VendorServer(), [], []
    vendor_app, core_app = web.Application(), web.Application()
    vendor_app.router.add_get('/', vendor.endpoint)

    async def register(request):
        registrations.append(await request.json())
        return web.json_response({'code': 200, 'data': {'id': 'loopback-only'}})

    core_app.router.add_post('/api/mcp', register)
    rclpy.init()
    node = rclpy.create_node('tron2_teleop_validation')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(String, COMMAND_TOPIC, qos)
    node.create_subscription(String, STATE_TOPIC, lambda msg: monitors.append(json.loads(msg.data)), qos)
    process = None

    async def spin_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(.005)

    sequence = 0

    async def feed(grip, seconds=.3, offset=0.):
        nonlocal sequence
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            sequence += 1
            publisher.publish(String(data=json.dumps(frame(sequence, grip, clock_id=clock_id, offset=offset))))
            await spin_for(.02)

    try:
        async with server(vendor_app, 5000), server(core_app, 15678), aiohttp.ClientSession() as client:
            process = await asyncio.create_subprocess_exec(sys.executable, '-m', 'limx.tron2_teleop.main')
            async def health():
                async with client.get('http://127.0.0.1:15794/health') as response:
                    return await response.json()

            async def rpc(action):
                args = {'action': action}
                if action == 'start':
                    args['input_topic'] = COMMAND_TOPIC
                async with client.post('http://127.0.0.1:15794/mcp', json={
                    'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                    'params': {'name': 'teleop_control', 'arguments': args}}) as response:
                    value = await response.json()
                assert 'error' not in value, value
                assert not value['result']['isError'], value
                return json.loads(value['result']['content'][0]['text'])

            deadline = time.monotonic()+15
            while True:
                assert process.returncode is None, 'service exited'
                try:
                    if (await health())['robot']['ready'] and publisher.get_subscription_count():
                        break
                except aiohttp.ClientError:
                    pass
                assert time.monotonic() < deadline, 'startup timeout'
                await spin_for(.1)
            await spin_for(.3)
            assert registrations and not vendor.commands
            assert not (await health())['active']
            await rpc('start')
            await spin_for(.3)
            assert not vendor.commands, 'start without input must not move'
            await feed(1.)
            assert not vendor.commands, 'held grips must not calibrate'
            await feed(0.)
            assert (await health())['mapping_epoch'] == 1
            await feed(1., seconds=.6, offset=.01)
            assert len(vendor.commands) > 10, await health()
            assert all(len(body['data']['q']) == 16 and body['data']['q'][14:] == Q[14:]
                       for _, body in vendor.commands)
            await feed(0.)
            count = len(vendor.commands)
            await feed(0.)
            assert len(vendor.commands) == count
            await feed(1., offset=.01)
            assert len(vendor.commands) > count
            # Missing DDS inputs expire independently of the publisher's liveness.
            await spin_for(.4)
            count = len(vendor.commands)
            await spin_for(.15)
            assert len(vendor.commands) == count and not (await health())['output_active']
            await feed(1.)
            assert len(vendor.commands) == count, 'stale input requires released-grip calibration'
            await feed(0.)
            await feed(1., offset=.005)
            assert len(vendor.commands) > count
            assert not (await rpc('stop'))['active']
            await spin_for(.03)
            count = len(vendor.commands)
            await feed(1.)
            assert len(vendor.commands) == count, 'MCP stop must fence DDS input'
            assert monitors and any(m['state'] == 'following' for m in monitors)
            latest = monitors[-1]
            validate_feedback(latest, instance_id='pico-1', clock_id=clock_id, now_ns=time.monotonic_ns())
            assert latest['execution']['stop_confirmed'] is False
            assert set(vendor.requests) <= {'request_get_joint_state', 'request_servoj'}
            process.send_signal(signal.SIGTERM)
            await asyncio.wait_for(process.wait(), 10)
            assert process.returncode == 0
            print(json.dumps({'result': 'PASS', 'transport': 'real FastDDS loopback',
                'servo_commands': count, 'monitor_frames': len(monitors),
                'core_registrations': len(registrations), 'service_exit': process.returncode,
                'hardware_tested': False}))
    finally:
        if process and process.returncode is None:
            process.kill()
            await process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(check())
