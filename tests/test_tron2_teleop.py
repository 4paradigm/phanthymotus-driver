"""Software-only admission, IK and real loopback MCP/WebSocket checks."""
import asyncio
from contextlib import asynccontextmanager
import copy
import json
import math
import threading
import time

import aiohttp
from aiohttp import web
import numpy as np
import pytest

from common.teleop_contract import COMMAND_TOPIC, TRACKING_FRAME, validate_feedback
from limx.tron2_teleop.control import TeleopControl, make_worker
from limx.tron2_teleop.kinematics import Tron2IK, pose_values, quaternion, transform
from limx.tron2_teleop.main import Adapter, application, load_settings
from limx.tron2_teleop.mapping import RelativeMapping
from limx.tron2_teleop.model import MODEL, JOINT_NAMES, feedback_q, limits, validate_q
from limx.tron2_teleop.vendor import Robot, decode

Q = [0., .4, 0., -1., 0., .1, 0., 0., -.4, 0., -1., 0., .1, 0., .1, -.2]
INFO = {'accid': 'DACH_TRON2A_test', 'sw_version': 'test-fixture', 'motor': 'OK',
        'working_mode': 'developer_mode', 'working_mode_valid': True,
        'collision_enable_valid': True, 'collision_enable': True,
        'compliant_enable_valid': True, 'compliant_enable': False}


def frame(sequence, grip=0., *, clock_id='test-boot', offset=0.):
    pose = {'tracked': True, 'position': [0., 0., 1.], 'orientation_xyzw': [0., 0., 0., 1.]}
    now = time.monotonic_ns()
    value = {'schema': 'motus.teleop.command/1', 'kind': 'input', 'instance_id': 'pico-1',
             'device_id': 'headset', 'clock_id': clock_id, 'tracking_frame': TRACKING_FRAME,
             'sequence': sequence, 'connection_epoch': 1, 'space_epoch': 1,
             'source_monotonic_ns': now, 'received_monotonic_ns': now,
             'head_reference': copy.deepcopy(pose),
             'left': dict(copy.deepcopy(pose), grip=grip, trigger=0.),
             'right': dict(copy.deepcopy(pose), grip=grip, trigger=0.)}
    value['left']['position'][0] += offset
    value['right']['position'][0] += offset
    return value


async def until(predicate, timeout=3):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('condition timed out')
        await asyncio.sleep(.005)


class VendorServer:
    """Only a software model; records exact traffic and never reaches hardware."""
    def __init__(self):
        self.q = Q.copy()
        self.commands, self.requests, self.sockets = [], [], []
        self.reply = True
        self.reject = False
        self.info = INFO.copy()

    async def endpoint(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.sockets.append(ws)
        async def notify():
            while not ws.closed:
                await ws.send_json({'accid': INFO['accid'], 'title': 'notify_robot_info', 'data': self.info})
                await asyncio.sleep(.1)
        task = asyncio.create_task(notify())
        try:
            async for message in ws:
                value = json.loads(message.data)
                self.requests.append(value['title'])
                if value['title'] == 'request_get_joint_state' and self.reply:
                    await ws.send_json({**value, 'title': 'response_get_joint_state',
                                       'data': {'q': self.q, 'names': JOINT_NAMES, 'result': 'success'}})
                elif value['title'] == 'request_servoj':
                    self.commands.append((time.monotonic(), value))
                    if self.reject:
                        await ws.send_json({**value, 'title': 'notify_servoJ', 'data': {'result': 'fail_motor'}})
                    else:
                        self.q = value['data']['q']
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        return ws


@asynccontextmanager
async def server(app, port=0):
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '127.0.0.1', port)
    await site.start()
    try:
        yield site._server.sockets[0].getsockname()[1]
    finally:
        await runner.cleanup()


@asynccontextmanager
async def connected_control():
    vendor = VendorServer()
    app = web.Application()
    app.router.add_get('/', vendor.endpoint)
    async with server(app) as port, aiohttp.ClientSession() as session:
        robot = Robot(f'ws://127.0.0.1:{port}')
        worker = await asyncio.to_thread(make_worker)
        control = TeleopControl(robot, worker, clock_id='test-boot')
        tasks = [asyncio.create_task(robot.run(session)), asyncio.create_task(control.solve_loop()),
                 asyncio.create_task(control.send_loop())]
        try:
            await until(lambda: robot.ready() is None)
            yield control, vendor
        finally:
            control.closed = True
            await control.stop()
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await asyncio.to_thread(worker.close)


async def prepare(control):
    control.start(COMMAND_TOPIC)
    control.receive(frame(1))
    await until(lambda: control.mapping is not None)


async def feed(control, start, count=10, grip=1., offset=.01):
    for sequence in range(start, start+count):
        control.receive(frame(sequence, grip, offset=offset))
        await asyncio.sleep(.02)


def test_named_joint_order_and_vendor_urdf_limit_intersection():
    assert feedback_q({'result': 'success', 'names': JOINT_NAMES[::-1], 'q': Q[::-1]}) == Q
    assert feedback_q({'result': 'success', 'names': [], 'q': Q}) == Q
    assert limits()[1][3] == .2618
    for bad in (Q[:14], [math.nan]*16, [True]*16, [10.]*16):
        with pytest.raises(ValueError):
            validate_q(bad)
    with pytest.raises(ValueError):
        feedback_q({'result': 'success', 'names': [JOINT_NAMES[0]]*16, 'q': Q})


def test_fk_jacobian_and_seven_joint_ik_roundtrip():
    solver = Tron2IK(MODEL)
    q = np.array(Q[:14])
    for index, chain in enumerate(solver.chains):
        local = q[index*7:index*7+7]
        pose, jacobian = chain.fk(local, jacobian=True)
        for joint in range(7):
            changed = local.copy()
            changed[joint] += 1e-6
            derivative = (chain.fk(changed)[:3, 3]-pose[:3, 3])/1e-6
            assert np.allclose(derivative, jacobian[:3, joint], atol=1e-6)
    target_q = q + np.array([.03, -.02, .02, .04, .01, .02, -.01]*2)
    targets = solver.palms(target_q)
    result = solver.solve(targets, q, None, time.monotonic()+1)
    for actual, target in zip(solver.palms(result), targets):
        assert np.linalg.norm(actual[:3, 3]-target[:3, 3]) <= .002
        assert np.linalg.norm(actual[:3, :3]-target[:3, :3]) < .03
    targets[0][:3, 3] = [10, 10, 10]
    with pytest.raises(ValueError):
        solver.solve(targets, q, None, time.monotonic()+.5)


def test_quaternion_and_relative_mapping_preserve_base_and_head_yaw():
    pose = transform({'position': [1, 2, 3], 'orientation': [1., 0., 0., 0.]})
    assert np.allclose(transform({'position': [1, 2, 3], 'orientation': quaternion(pose).tolist()}), pose)
    origin = frame(1)
    origin['head_reference']['orientation_xyzw'] = [0, 0, math.sqrt(.5), math.sqrt(.5)]
    poses = [pose_values(p) for p in Tron2IK(MODEL).palms(Q[:14])]
    mapping = RelativeMapping(origin, poses, .5)
    moved = copy.deepcopy(origin)
    moved['left']['position'][1] += .1
    values = mapping.targets(moved)
    assert values[0] == pytest.approx(poses[0][0]+.05)
    assert values[1:7] == pytest.approx(poses[0][1:])
    assert values[7:] == pytest.approx(poses[1])


def test_real_numerical_process_and_closed_worker():
    worker = make_worker()
    try:
        poses = worker.call('forward', {'q': Q[:14]})
        result = worker.call('solve', {'values': sum(poses, []), 'q': Q[:14],
            'commanded': None, 'measured_ns': 1}, identity={'boot_id': 'test', 'session_id': 'test',
            'mapping_epoch': 1, 'generation': 1})
        assert result['q'] == pytest.approx(Q[:14])
    finally:
        worker.close()
    with pytest.raises(ValueError, match='closed'):
        worker.call('forward', {'q': Q[:14]})


def test_loopback_mcp_and_vendor_lifecycle():
    async def scenario():
        async with connected_control() as (control, vendor):
            async with server(application(Adapter(control))) as port, aiohttp.ClientSession() as session:
                async def rpc(method, params=None):
                    async with session.post(f'http://127.0.0.1:{port}/mcp', json={
                            'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params or {}}) as reply:
                        return await reply.json()
                tools = (await rpc('tools/list'))['result']['tools']
                assert len(tools) == 1 and tools[0]['type'] == 'actuator'
                assert tools[0]['topic_in'][0]['topic'] == COMMAND_TOPIC
                args = {'name': 'teleop_control', 'arguments': {'action': 'start', 'input_topic': COMMAND_TOPIC}}
                assert not (await rpc('tools/call', args))['result']['isError']
                await asyncio.sleep(.05)
                assert vendor.commands == []
                await feed(control, 1, count=3, grip=1.)
                assert control.mapping is None and vendor.commands == []
                await feed(control, 4, count=3, grip=0., offset=0.)
                await until(lambda: control.mapping is not None)
                anchor = control.mapping
                await feed(control, 7)
                assert len(vendor.commands) > 5
                assert set(vendor.requests) <= {'request_get_joint_state', 'request_servoj'}
                assert all(len(body['data']['q']) == 16 and body['data']['q'][14:] == Q[14:]
                           for _, body in vendor.commands)
                for (ta, a), (tb, b) in zip(vendor.commands, vendor.commands[1:]):
                    assert max(abs(x-y) for x, y in zip(a['data']['q'][:14], b['data']['q'][:14])) <= .35/300+1e-9
                control.receive(frame(17, 0.))
                await asyncio.sleep(.03)
                count = len(vendor.commands)
                await asyncio.sleep(.04)
                assert len(vendor.commands) == count and control.mapping is anchor
                await feed(control, 18, count=5)
                assert len(vendor.commands) > count and control.mapping is anchor
                args['arguments'] = {'action': 'stop'}
                assert not (await rpc('tools/call', args))['result']['isError']
                await asyncio.sleep(.02)
                count = len(vendor.commands)
                await feed(control, 30, count=3)
                assert len(vendor.commands) == count
                info = control.feedback()
                validate_feedback(info, instance_id='pico-1', clock_id='test-boot', now_ns=time.monotonic_ns())
                assert info['execution']['stop_confirmed'] is False
    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['old_sequence', 'stale', 'untracked', 'epoch', 'old_epoch', 'robot_disconnect', 'vendor_fault', 'feedback_stale'])
def test_failure_discards_target_and_does_not_replay(failure):
    async def scenario():
        async with connected_control() as (control, vendor):
            await prepare(control)
            await feed(control, 2, count=6)
            assert vendor.commands
            value = frame(8, 1.)
            if failure == 'old_sequence':
                value['sequence'] = 1
            elif failure == 'stale':
                value['received_monotonic_ns'] -= 1_000_000_000
            elif failure == 'untracked':
                value['left'].update(tracked=False, position=None, orientation_xyzw=None)
            elif failure == 'epoch':
                value['space_epoch'] += 1
            elif failure == 'old_epoch':
                value['space_epoch'] = 0
            elif failure == 'robot_disconnect':
                await vendor.sockets[-1].close()
            elif failure == 'vendor_fault':
                vendor.reject = True
            else:
                vendor.reply = False
            control.receive(value)
            await asyncio.sleep(.35)
            count = len(vendor.commands)
            await asyncio.sleep(.1)
            assert len(vendor.commands) == count
            assert not control.output and control.target is None
            assert control.mapping is None
            if failure == 'robot_disconnect':
                await until(lambda: control.robot.ready() is None)
                control.receive(frame(9, 1.))
                await asyncio.sleep(.05)
                assert len(vendor.commands) == count
    asyncio.run(scenario())


def test_no_fresh_input_watchdog_stops_without_dds_pump():
    async def scenario():
        async with connected_control() as (control, vendor):
            await prepare(control)
            await feed(control, 2, count=5)
            await asyncio.sleep(.35)
            count = len(vendor.commands)
            await asyncio.sleep(.05)
            assert len(vendor.commands) == count and control.reason == 'input_stale'
    asyncio.run(scenario())


def test_stop_ignores_late_numerical_result_and_cancels_pending_send():
    async def scenario():
        async with connected_control() as (control, vendor):
            await prepare(control)
            original = control.worker.call
            entered, release = threading.Event(), threading.Event()
            def delayed(*args, **kwargs):
                entered.set()
                release.wait(2)
                return {'q': Q[:14], 'last_ms': 1.}
            control.worker.call = delayed
            try:
                control.receive(frame(2, 1.))
                await until(entered.is_set)
                await control.stop()
                release.set()
                await asyncio.sleep(.04)
                assert control.target is None and vendor.commands == []
            finally:
                release.set()
                control.worker.call = original
            await prepare(control)
            sent = asyncio.Event()
            async def delayed_send(q):
                sent.set()
                await asyncio.sleep(10)
                raise AssertionError('send must have been cancelled')
            control.robot.send = delayed_send
            control.receive(frame(2, 1.))
            await until(sent.is_set)
            await control.stop()
            await asyncio.sleep(.01)
            assert not control.output and vendor.commands == []
    asyncio.run(scenario())


def test_settings_and_json_trust_boundary(monkeypatch, tmp_path):
    for variable in ('CONFIG_PATH', 'TRON2_ROBOT_URL', 'TRON2_SERIAL'):
        monkeypatch.delenv(variable, raising=False)
    assert load_settings()[2] == .5
    for endpoint in ('http://10.192.1.2:5000', 'ws://host:80', 'ws://user:pass@host:5000', 'ws://host:5000/move'):
        monkeypatch.setenv('TRON2_ROBOT_URL', endpoint)
        with pytest.raises(ValueError):
            load_settings()
    for text in ('{"x":1,"x":2}', '{"x":NaN}', '[]'):
        with pytest.raises(ValueError):
            decode(text)


@pytest.mark.parametrize('key,value', [('motor', 'ERROR'), ('working_mode', 'controller_mode'),
    ('collision_enable', False), ('collision_enable_valid', 1), ('compliant_enable', True)])
def test_motion_readiness_requires_explicit_valid_vendor_state(key, value):
    robot = Robot('ws://127.0.0.1:5000')
    robot.ws = type('Socket', (), {'closed': False})()
    robot.info = {**INFO, key: value}
    robot.q, robot.info_at, robot.q_at = Q.copy(), time.monotonic(), time.monotonic()
    assert robot.ready() is not None


def test_vendor_identity_and_send_deadline():
    async def scenario():
        async with connected_control() as (control, vendor):
            robot = control.robot
            for serial in ('WF_TRON2_test', 'DACH_TRON2A_other'):
                with pytest.raises(ValueError, match='identity_mismatch'):
                    robot.accept({'title': 'notify_robot_info', 'accid': serial, 'data': INFO})
            original = robot.ws
            cancelled = asyncio.Event()
            class SlowSocket:
                closed = False
                async def send_json(self, packet):
                    try:
                        await asyncio.sleep(10)
                    finally:
                        cancelled.set()
            robot.ws = SlowSocket()
            try:
                with pytest.raises(asyncio.TimeoutError):
                    await robot.send(Q)
                assert cancelled.is_set() and robot.sent_count == 0
            finally:
                robot.ws = original
            assert not vendor.commands
    asyncio.run(scenario())


def test_concurrent_mcp_lifecycle_and_final_stop():
    async def scenario():
        async with connected_control() as (control, vendor):
            async with server(application(Adapter(control))) as port, aiohttp.ClientSession() as session:
                async def call(action):
                    async with session.post(f'http://127.0.0.1:{port}/mcp', json={
                        'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call', 'params': {
                            'name': 'teleop_control', 'arguments': {'action': action, 'input_topic': COMMAND_TOPIC}}}) as response:
                        result = await response.json()
                    assert 'error' not in result
                await asyncio.gather(*(call(action) for action in ['start', 'stop']*5))
                await call('stop')
                await feed(control, 1, grip=0.)
                await feed(control, 20, grip=1.)
                assert not control.active and not control.target and not vendor.commands
    asyncio.run(scenario())
