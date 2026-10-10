#!/usr/bin/env python3
"""Record three stationary samples. Contains no robot motion request."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import re
from urllib.parse import urlparse

import aiohttp


async def capture(serial, url, destination):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', serial):
        raise ValueError('Configure the actual robot serial')
    endpoint = urlparse(url)
    if endpoint.scheme not in ('ws', 'wss') or not endpoint.hostname \
            or endpoint.port != 5000 or endpoint.username or endpoint.password \
            or endpoint.query or endpoint.fragment or endpoint.path not in ('', '/'):
        raise ValueError('Use the vendor WebSocket endpoint on port 5000')
    # Bind identity before importing modules, exactly as the MCP entry point does.
    os.environ['TRON2_SERIAL'] = serial
    os.environ['TRON2_ROBOT_URL'] = url
    from motion import RobotClient
    import pose
    async with aiohttp.ClientSession(trust_env=False,
                                    timeout=aiohttp.ClientTimeout(total=8)) as session:
        async with RobotClient(session) as robot:
            current, _teach = await robot.status()
            pose.validate_snapshot(current)
            samples = [current]
            for _ in range(2):
                await asyncio.sleep(2.2)
                joints = await robot.joints()
                samples.append(robot.snapshot(joints))
    home = pose.make_pose(samples)
    pose.save_pose(destination, home)  # Exclusive creation, never overwrite.
    print(json.dumps({'recorded': str(destination.resolve()), 'motion_sent': False,
                      'accid': home['accid']}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--url', default='ws://10.192.1.2:5000')
    parser.add_argument('--pose', type=Path, required=True)
    args = parser.parse_args()
    if args.pose.exists():
        parser.error('Destination exists; the recorded target will not be overwritten')
    asyncio.run(capture(args.serial, args.url, args.pose))
