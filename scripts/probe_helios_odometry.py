#!/usr/bin/env python3
"""Passively record chassis feedback for unit/frame verification; send no commands."""
import argparse
import asyncio
import json
import math
import time
from pathlib import Path

import aiohttp
from rokae.helios.chassis_pose_pb2 import UiDatagram


async def record(args):
    deadline = time.monotonic() + args.seconds
    count = 0
    with Path(args.output).open('w', encoding='utf-8') as output:
        async with aiohttp.ClientSession(trust_env=False) as session:
            async with session.ws_connect(args.url, max_msg_size=8 * 1024 * 1024) as socket:
                print('READY: passive chassis recording; no application messages sent', flush=True)
                while time.monotonic() < deadline:
                    try:
                        msg = await socket.receive(timeout=min(1, max(.01, deadline-time.monotonic())))
                    except asyncio.TimeoutError:
                        continue
                    if msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.CLOSE,
                                    aiohttp.WSMsgType.ERROR):
                        raise RuntimeError('Feedback socket closed')
                    if msg.type != aiohttp.WSMsgType.BINARY:
                        continue
                    packet = UiDatagram()
                    packet.ParseFromString(msg.data)
                    if not packet.HasField('current_state'):
                        continue
                    state = packet.current_state
                    pose = [state.pose.x, state.pose.y, state.pose.theta] if state.HasField('pose') else None
                    speed = [state.speed.vx, state.speed.vy, state.speed.w] if state.HasField('speed') else None
                    if not all(math.isfinite(x) for row in (pose, speed) if row for x in row):
                        continue
                    row = {'received_monotonic_ns': time.monotonic_ns(),
                           'received_unix_ms': time.time_ns()//1_000_000,
                           'source_stamp_ms': state.timeStamp, 'pose': pose, 'speed': speed,
                           'speed_stamp_ms': state.speed.timeStamp if speed is not None else None}
                    output.write(json.dumps(row, allow_nan=False)+'\n')
                    output.flush()
                    count += 1
                    if count % 50 == 0:
                        print('samples', count, 'pose', pose, 'speed', speed, flush=True)
    print('DONE', count, 'samples', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='ws://192.168.0.100/ws/AGVInfo')
    parser.add_argument('--seconds', type=float, default=240)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 900:
        parser.error('--seconds must be finite and between 0 and 900')
    asyncio.run(record(args))
