"""Current motion/ACP contract tests; synthetic bridge/transport, no robot I/O."""
import copy
import json
import math
import ssl
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import jsonschema
import pytest

from deep_robotics.s10_pro import acp
from deep_robotics.s10_pro.loco_move import LocoMovePlugin
from deep_robotics.s10_pro.motion_worker import dispatch_request


class Bridge:
    def __init__(self):
        self.state = 17
        self.speed = (0., 0., 0.)
        self.x = self.y = self.yaw = 0.
        self.last = time.monotonic()
        self.sequence = 0
        self.faults = []
        self.hes = 0
        self.stale = False
        self.owners = {}

    def acquire(self, owner, sources, options):
        self.owners[owner] = sources

    def release(self, owner):
        self.owners.pop(owner, None)

    def snapshot(self, source):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        self.x += self.speed[0] * dt
        self.y += self.speed[1] * dt
        self.yaw += self.speed[2] * dt
        self.sequence += 1
        record = {'fresh': not self.stale, 'error': None, 'age_s': 0.,
                  'received_monotonic': now, 'sequence': self.sequence}
        if source == 'hard_estop':
            data = {'value': self.hes}
        elif source == 'fault_aggregator':
            data = {'active_faults': self.faults}
        elif source == 'joints':
            data = {'data': {'joints_data': [{'name': str(i), 'data_id': self.sequence,
                                            'status_word': 1} for i in range(16)]}}
        elif source == 'odometry':
            data = {'header': {'stamp': {'sec': int(now), 'nanosec': int(now % 1 * 1e9)}},
                'pose': {'pose': {'position': {'x': self.x, 'y': self.y, 'z': 0.},
                    'orientation': {'x': 0., 'y': 0., 'z': math.sin(self.yaw / 2), 'w': math.cos(self.yaw / 2)}}},
                'twist': {'twist': {'linear': {'x': self.speed[0], 'y': self.speed[1], 'z': 0.},
                                   'angular': {'z': self.speed[2]}}}}
        else:
            data = {'data': {'motion_state': {'state': self.state}, 'gait_state': {'gait': 0},
                            'vel_x': self.speed[0], 'vel_y': self.speed[1], 'vel_yaw': self.speed[2]}}
        return {**record, 'data': data}


class Transport:
    def __init__(self, bridge):
        self.bridge = bridge
        self.closed = self.running = False
        self.states = []
        self.velocities = []
        self.respond = True
        self.matches = 1

    def status(self):
        return {'closed': self.closed, 'worker_running': self.running}

    def start(self, **kwargs):
        self.running = True

    def close(self):
        self.closed, self.running = True, False

    def graph_snapshot(self):
        return {k: {'publisher_prepared': True, 'matched_subscribers': self.matches} for k in ('state', 'velocity')}

    def publish_state(self, state, *, cancel_event):
        assert not cancel_event.is_set()
        self.states.append(state)
        if self.respond:
            self.bridge.state = 0 if state == 4 else state
        return {'sent': True}

    def publish_velocity(self, *velocity, cancel_event):
        assert not cancel_event.is_set()
        self.velocities.append(velocity)
        self.bridge.snapshot('motion_info')
        if self.respond:
            self.bridge.speed = velocity
        return {'sent': True}


@pytest.fixture
def rig():
    bridge = Bridge()
    transport = Transport(bridge)
    callbacks = []
    p = LocoMovePlugin({'enabled': True, 'action_timeout_s': 8.}, bridge,
                      lambda: transport, callback=lambda *args: callbacks.append(args))
    assert p.start() == {'state': 'ready'}
    yield p, bridge, transport, callbacks
    p.close()


def wait(p):
    p._worker.join(12)
    assert not p._worker.is_alive()
    return p._last


def test_schema_and_lifecycle(rig):
    p, _, transport, _ = rig
    tools = p.get_tools()
    assert [t['name'] for t in tools] == ['loco_move']
    for t in tools:
        jsonschema.Draft202012Validator.check_schema(t['inputSchema'])
        assert t['inputSchema']['x-resource'] == 'base'
    assert p.dispatch('stop', {}) == {'state': 'idle'}
    assert p.dispatch('start', {}) == {'state': 'ready'}
    assert not transport.states and not transport.velocities


@pytest.mark.parametrize('cause', ['fault', 'estop', 'stale', 'disabled'])
def test_gates_send_nothing(rig, cause):
    p, b, t, callbacks = rig
    if cause == 'fault': b.faults = [{'code': 1}]
    if cause == 'estop': b.hes = 1
    if cause == 'stale': b.stale = True
    if cause == 'disabled': p._host_enabled = False
    r = p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    assert not r['accepted'] and r['blocked_by']
    assert not t.states and not t.velocities and not callbacks


@pytest.mark.parametrize('state,expected', [(0, [1, 17]), (1, [17]), (17, [])])
def test_stand_sequence(rig, state, expected):
    p, b, t, callbacks = rig
    b.state = state
    r = p.execute('stand_up', {})
    assert r['action_id']
    assert wait(p)['completed']
    assert t.states == expected and not t.velocities
    assert len(callbacks) == 1 and callbacks[0][0] == r['action_id']


@pytest.mark.parametrize('alias', ['sit_down', 'sitdown', 'stand_down'])
def test_sit_aliases(rig, alias):
    p, b, t, callbacks = rig
    p.dispatch(alias, {'direction': 'forward', 'speed': .1, 'duration': 1.})
    assert wait(p)['completed'] and b.state == 0
    assert t.states == [4] and len(callbacks) == 1


@pytest.mark.parametrize('direction', ['forward', 'backward', 'turn_left', 'turn_right'])
def test_finite_move_stops_and_posts_once(rig, direction):
    p, b, t, callbacks = rig
    reply = p.execute('move', {'direction': direction, 'speed': .1, 'duration': .5, 'request_id': 'one'})
    done = wait(p)
    assert done['completed'] and done['stop_confirmed']
    assert all(v == (0., 0., 0.) for v in t.velocities[-2:])
    assert len(callbacks) == 1 and callbacks[0][0] == reply['action_id']
    before = len(t.velocities)
    replay = p.execute('move', {'direction': direction, 'speed': .1, 'duration': .5, 'request_id': 'one'})
    assert 'action_id' not in replay and replay['completed']
    assert len(t.velocities) == before and len(callbacks) == 1


def test_no_response_is_error_but_not_a_persistent_lock(rig):
    p, b, t, callbacks = rig
    t.respond = False
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    assert wait(p)['reason'] == 'motion_response_not_observed'
    assert p._last['stop_confirmed']
    t.respond = True
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    assert wait(p)['completed'] and len(callbacks) == 2


def test_cancel_and_shared_resource(rig):
    p, b, t, callbacks = rig
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': 4.})
    assert p.execute('sit_down', {})['state'] == 'busy'
    deadline = time.monotonic() + 3
    while not t.velocities and time.monotonic() < deadline: time.sleep(.01)
    assert p.cancel()['accepted']
    result = wait(p)
    assert result['status'] == 'cancelled' and result['stop_confirmed']
    assert not result['completed'] and len(callbacks) == 1


def test_posture_timeout(rig):
    p, b, t, callbacks = rig
    b.state = 1
    t.respond = False
    p._settings['action_timeout_s'] = 1.2
    p.execute('sit_down', {})
    assert wait(p)['reason'] == 'posture_feedback_timeout'
    assert callbacks[0][1] == 'error'


def test_removed_relative_motion_cannot_reach_transport(rig):
    p, _, transport, callbacks = rig
    args = {'direction': 'forward', 'distance': .3, 'speed': .12}
    with pytest.raises(ValueError):
        p.dispatch('move', {**args, '_tool_name': 'relative_move'})
    with pytest.raises(ValueError):
        p.dispatch('relative_move', args)
    with pytest.raises(ValueError):
        p.execute('relative_move', args)
    assert not transport.states and not transport.velocities and not callbacks


def test_internal_feedback_survives_monitor_removal(rig):
    from deep_robotics.s10_pro.reader import SubscriptionReader, source_specs, normalize_control
    _, bridge, _, _ = rig
    sources = bridge.owners['s10_motion']
    assert sources == set(source_specs({})) == {
        'motion_info', 'joints', 'odometry', 'hard_estop', 'fault_aggregator'}
    callbacks, events = {}, []
    class Node:
        def create_subscription(self, cls, topic, callback, qos):
            callbacks[topic] = callback
            return topic
        def destroy_subscription(self, subscription):
            callbacks.pop(subscription)
    reader = SubscriptionReader(Node(), source_specs({}), lambda kind: dict,
                                events.append, None)
    try:
        reader.apply_control({'active': sorted(sources), 'rates_hz': {s: 10. for s in sources}})
        for source, spec in source_specs({}).items():
            callbacks[spec['topic']](bridge.snapshot(source)['data'])
        assert {event['key'] for event in events} == sources
        assert all(isinstance(event['data'], dict) for event in events)
        old_callback = callbacks['/MOTION_INFO']
        reader.apply_control({'active': []})
        old_callback({'data': {}})
        assert not callbacks and len(events) == len(sources)
        for removed in ('battery', 'imu', 'lidar', 'ros_graph'):
            with pytest.raises(ValueError):
                normalize_control({'active': [removed]})
    finally:
        reader.close()


def test_worker_rejects_expired_command():
    class Adapter:
        def publish_state(self, *args, **kwargs): raise AssertionError('must not send')
    with pytest.raises(TimeoutError):
        dispatch_request(Adapter(), {'id': 1, 'op': 'state', 'payload': {'state': 1},
                                     'deadline_monotonic': 1.}, clock=lambda: 2.)


def test_acp_self_signed_local_core_posts_exactly_once(tmp_path, monkeypatch):
    key, cert = tmp_path / 'key.pem', tmp_path / 'cert.pem'
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1',
                    '-subj', '/CN=localhost', '-keyout', str(key), '-out', str(cert)],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    received = []
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            received.append((self.path, json.loads(self.rfile.read(int(self.headers['Content-Length'])))))
            self.send_response(200); self.end_headers(); self.wfile.write(b'{"ok":true}')
        def log_message(self, *args): pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER); context.load_cert_chain(cert, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    monkeypatch.setenv('AGENT_CORE_URL', 'https://localhost:' + str(server.server_port))
    try:
        acp.complete('unique', 'completed', {'stopped': True}, 'loco_move')
        assert len(received) == 1
        path, payload = received[0]
        assert path == '/api/acp/complete'
        assert payload['action_id'] == 'unique' and payload['status'] == 'completed'
        assert payload['tool'] == 'loco_move' and payload['result'] == {'stopped': True}
        assert isinstance(payload['ts'], float)
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_stop_failure_never_reports_success(rig):
    p, b, t, callbacks = rig
    original = t.publish_velocity
    def send(*velocity, cancel_event):
        if velocity == (0., 0., 0.):
            raise TimeoutError('zero failed')
        return original(*velocity, cancel_event=cancel_event)
    t.publish_velocity = send
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    result = wait(p)
    assert not result['completed'] and not result['stop_confirmed']
    assert result['reason'] == 'reported_stop_not_confirmed' and callbacks[0][1] == 'error'


def test_fault_during_move_stops_output(rig):
    p, b, t, callbacks = rig
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': 3.})
    deadline = time.monotonic() + 3
    while not t.velocities and time.monotonic() < deadline: time.sleep(.01)
    b.faults = [{'code': 1}]
    result = wait(p)
    assert not result['completed'] and result['stop_confirmed']
    assert 'fault' in result['reason'] and t.velocities[-1] == (0., 0., 0.)


@pytest.mark.parametrize('removed_option', [{}, {'auto_navigation': False}])
def test_legacy_execution_environment_only_disables_motion(monkeypatch, removed_option):
    import importlib.util
    import sys
    path = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro/main.py'
    monkeypatch.syspath_prepend(str(path.parent))
    monkeypatch.setenv('S10_EXECUTION_AUTHORIZATION_FILE', '/old-policy')
    spec = importlib.util.spec_from_file_location('s10_main_degraded', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    bridge = Bridge()
    plugins = module.build_runtime_plugins({'motion': {'enabled': True, **removed_option}}, bridge)
    try:
        assert len(plugins) == 1 and not plugins[-1]._host_enabled
        assert not bridge.owners
    finally:
        for plugin in plugins:
            plugin.close() if hasattr(plugin, 'close') else plugin.stop()


def test_worker_codec_and_dispatch_match_vendor_messages():
    from deep_robotics.s10_pro.motion_dds import MessageCodec, EXPECTED_FIELDS, DDSCommandAdapter
    classes = {}
    for name, fields in EXPECTED_FIELDS.items():
        def init(self, fields=fields):
            for key, kind in fields.items():
                setattr(self, key, classes[kind]() if kind in classes else 0)
        classes[name] = type(name.split('/')[-1], (), {'__init__': init,
            'get_fields_and_field_types': staticmethod(lambda fields=fields: fields)})
    codec = MessageCodec(classes.__getitem__, clock_ns=lambda: 1234567890)
    codec.prepare()
    states, velocities = [], []
    class Publisher:
        def __init__(self, topic): self.topic = topic
        def get_subscription_count(self): return 1
        def publish(self, msg):
            (states if self.topic == '/MOTION_STATE' else velocities).append(msg)
    class Node:
        def create_publisher(self, cls, topic, qos): return Publisher(topic)
        def destroy_publisher(self, publisher): pass
    adapter = DDSCommandAdapter(Node(), execution_enabled=True, codec=codec)
    adapter.prepare_state_channel()
    request = {'id': 1, 'op': 'state', 'payload': {'state': 17}, 'deadline_monotonic': time.monotonic() + .19}
    assert dispatch_request(adapter, request)['sent']
    assert states[0].data.state == 17 and states[0].header.stamp.sec == 1
    assert adapter.publish_velocity(.1, 0., -.2)['sent']
    assert velocities[0].data.x_vel == .1 and velocities[0].data.yaw_vel == -.2
    with pytest.raises(ValueError): adapter.publish_state(2)
    with pytest.raises(ValueError): adapter.publish_velocity(float('nan'))
    adapter.close()


def test_remote_acp_keeps_certificate_verification(monkeypatch):
    seen = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
    def open_request(request, *, timeout, context):
        seen.append(context)
        return Response()
    monkeypatch.setenv('AGENT_CORE_URL', 'https://core.example.test:15678')
    monkeypatch.setattr(acp.urllib.request, 'urlopen', open_request)
    acp.complete('id', 'error', {}, 'loco_move')
    assert len(seen) == 1 and seen[0].verify_mode == ssl.CERT_REQUIRED and seen[0].check_hostname


@pytest.mark.parametrize('source', ['motion_info', 'hard_estop', 'fault_aggregator', 'joints'])
def test_malformed_telemetry_blocks_without_send(rig, source):
    p, b, t, callbacks = rig
    original = b.snapshot
    def snapshot(name):
        record = original(name)
        if name == source:
            record['data'] = []
        return record
    b.snapshot = snapshot
    reply = p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    assert reply['state'] == 'unavailable' and not t.velocities


def test_acp_failure_does_not_retry_or_change_motion_result(rig):
    p, b, t, callbacks = rig
    attempts = []
    def fail(*args):
        attempts.append(args)
        raise ConnectionError('local Core unavailable')
    p._callback = fail
    p.execute('move', {'direction': 'forward', 'speed': .1, 'duration': .5})
    assert wait(p)['completed'] and len(attempts) == 1
    assert p._active is None
