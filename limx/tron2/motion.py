"""Fixed-target MoveJ, with correlated replies and feedback verification.

No initial pose, mode switch, teach exit, head, gripper or emergency-stop command.
"""
import asyncio
import json
import math
import os
import time
import uuid

import aiohttp
import pose
from teach import inactive_value

ROBOT_URL = os.environ.get('TRON2_ROBOT_URL', 'ws://10.192.1.2:5000')
MIN_DURATION = 8
# Timing budget, not a vendor-certified peak speed or collision guarantee.
PLANNED_AVERAGE_DEG_S = 2.5


class MotionFailure(Exception):
    def __init__(self, message, attempted=False):
        super().__init__(message)
        self.attempted = attempted


def plan_preview(home, snapshot):
    result = pose.preview(home, snapshot)
    duration = max(MIN_DURATION, math.ceil(
        result['max_joint_change_deg'] / PLANNED_AVERAGE_DEG_S))
    result.update({'duration_seconds': duration,
                   'timing_policy': 'max_joint_delta_over_2_5_deg_s_min_8_seconds',
                   'automatic_collision_check': False})
    return result


def preflight(home, snapshot, teach, config):
    result = plan_preview(home, snapshot)
    robot = snapshot['robot']
    if robot.get('working_mode_valid') is not True or \
            robot.get('working_mode') != config.get('expected_working_mode'):
        raise ValueError('Working mode changed or is not confirmed')
    inactive_value(teach, config)
    return result


class RobotClient:
    def __init__(self, session):
        self.session = session
        self.ws = None
        self.robot = None
        self.info_at = None

    async def __aenter__(self):
        self.ws = await self.session.ws_connect(ROBOT_URL, autoping=True,
                                                max_msg_size=256 * 1024)
        try:
            deadline = time.monotonic() + 5
            while self.robot is None:
                await self.receive(deadline)
            return self
        except BaseException:
            await self.ws.close()
            raise

    async def __aexit__(self, *args):
        await self.ws.close()

    async def receive(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('Robot reply timeout')
        msg = await asyncio.wait_for(self.ws.receive(), remaining)
        if msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
                        aiohttp.WSMsgType.ERROR):
            raise ConnectionError('Robot WebSocket disconnected')
        if msg.type != aiohttp.WSMsgType.TEXT:
            return {}
        body = json.loads(msg.data)
        if not isinstance(body, dict):
            return {}
        if body.get('title') == 'notify_robot_info':
            data = body.get('data')
            if not isinstance(data, dict) or \
                    (body.get('accid') or data.get('accid')) != pose.SERIAL:
                raise ValueError('Robot serial mismatch')
            self.robot, self.info_at = data, time.monotonic()
        return body

    async def request(self, title, data, timeout=4):
        if title not in ('request_get_joint_state', 'request_drag_teach_manage', 'request_movej'):
            raise ValueError('Unsupported robot command')
        if title == 'request_drag_teach_manage' and data != {'cmd': 'status', 'arg': ''}:
            raise ValueError('Only read-only teach status is supported')
        guid = str(uuid.uuid4())
        await self.ws.send_json({'accid': pose.SERIAL, 'title': title,
            'guid': guid, 'timestamp': int(time.time() * 1000), 'data': data})
        deadline = time.monotonic() + timeout
        while True:
            body = await self.receive(deadline)
            if body.get('title') == 'notify_invalid_request' and body.get('guid') == guid:
                raise ValueError('Robot rejected the request')
            if body.get('title') != title.replace('request_', 'response_', 1) \
                    or body.get('guid') != guid:
                continue
            if body.get('accid') != pose.SERIAL:
                raise ValueError('Response serial mismatch')
            response = body.get('data')
            if not isinstance(response, dict) or response.get('result') != 'success':
                raise ValueError('Robot rejected request: ' + str(response)[:200])
            if title == 'request_drag_teach_manage' and response.get('success') is False:
                raise ValueError('Robot rejected teach-status query')
            return response

    async def joints(self):
        data = await self.request('request_get_joint_state', {})
        pose.vector(data.get('q'), 16, 'q')
        pose.vector(data.get('dq'), 16, 'dq')
        stamp = data.get('timestamp')
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) \
                or not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('Invalid joint-state timestamp')
        return {**data, 'robot_timestamp': stamp}

    def snapshot(self, joints):
        age = time.monotonic() - self.info_at
        return {'accid': pose.SERIAL, 'connected': True, 'fresh': age <= 3,
                'info_age_seconds': age, 'joint_age_seconds': 0,
                'robot': self.robot, 'joint_state': joints}

    async def status(self):
        teach = await self.request('request_drag_teach_manage', {'cmd': 'status', 'arg': ''})
        joints = await self.joints()
        return self.snapshot(joints), teach

    async def ready_state(self):
        current, teach = await self.status()
        samples = [pose.validate_snapshot(current)]
        last_stamp = current['joint_state']['robot_timestamp']
        for _ in range(2):
            await asyncio.sleep(.2)
            joints = await self.joints()
            current = self.snapshot(joints)
            samples.append(pose.validate_snapshot(current))
            if joints['robot_timestamp'] <= last_stamp:
                raise ValueError('Stationarity check received non-advancing feedback')
            last_stamp = joints['robot_timestamp']
        if max(max(q[i] for q in samples) - min(q[i] for q in samples)
               for i in range(16)) > .005:
            raise ValueError('Robot pose changed during the stationarity check')
        return current, teach

    async def move_and_verify(self, home, config, before_send):
        """One send attempt; uncertain outcome is never retried."""
        attempted = False
        try:
            current, teach = await self.ready_state()
            plan = preflight(home, current, teach, config)
            duration = plan['duration_seconds']
            await before_send(current)
            attempted = True  # Bytes may reach the controller even if await raises.
            started = time.monotonic()
            acknowledgement = await self.request('request_movej',
                {'time': duration, 'joint': pose.validate_pose(home)}, timeout=duration + 4)
            deadline = started + duration + 10
            last_stamp = current['joint_state']['robot_timestamp']
            stable = 0
            while time.monotonic() < deadline:
                joints = await self.joints()
                robot = self.robot
                if time.monotonic() - self.info_at > 3 or robot.get('motor') != 'OK' \
                        or robot.get('imu') != 'OK' or robot.get('working_mode_valid') is not True \
                        or robot.get('working_mode') != config['expected_working_mode'] \
                        or robot.get('sw_version') != home['firmware']:
                    raise ValueError('Robot health, firmware or working mode changed')
                stamp = joints['robot_timestamp']
                if stamp <= last_stamp:
                    raise ValueError('Joint feedback timestamp did not advance')
                last_stamp = stamp
                pose.arms(joints['q'][:14])
                error = max(abs(math.degrees(a-b))
                            for a,b in zip(home['target_q'], joints['q'][:14]))
                stationary = max(abs(v) for v in joints['dq'][:14]) <= .01
                if time.monotonic() - started >= duration and error <= 1 and stationary:
                    stable += 1
                else:
                    stable = 0
                if stable >= 3:
                    # Mode and teach state are checked again before claiming completion.
                    teach = await self.request('request_drag_teach_manage',
                                                {'cmd': 'status', 'arg': ''})
                    inactive_value(teach, config)
                    return {'status': 'arrived', 'motion_sent': True,
                            'at_home': True, 'max_error_deg': error,
                            'tolerance_deg': 1, 'stable_samples': stable,
                            'duration_seconds': duration, 'acknowledgement': acknowledgement,
                            'head_policy': 'no_head_command', 'gripper_policy': 'no_gripper_command',
                            'path_validated': False, 'automatic_collision_check': False}
                await asyncio.sleep(.2)
            raise TimeoutError('MoveJ was acknowledged but arrival was not verified')
        except (OSError, ValueError, TypeError, KeyError, aiohttp.ClientError, TimeoutError) as exc:
            raise MotionFailure(str(exc), attempted) from exc
