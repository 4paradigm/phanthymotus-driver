"""Private DDS worker framing/lifecycle, without ROS or robot connections."""
import json
import os
import subprocess
import sys
import threading
import time

import pytest

from deep_robotics.s10_pro.motion_process import (
    DDSProcessTransport, MotionProcessClosed, MotionProcessDisabled,
    MotionProcessNotStarted, MotionProcessCancelled, MotionWorkerRejected,
)
from deep_robotics.s10_pro.motion_worker import dispatch_request, strict_json, serve


class Adapter:
    def __init__(self):
        self.calls = []

    def graph_snapshot(self):
        return {'authority_proven': False}

    def publish_state(self, state, **kwargs):
        if kwargs['cancel_event'].is_set():
            raise InterruptedError('expired')
        self.calls.append(('state', state, kwargs['authority']))
        return {'sent': True, 'accepted': None}

    def publish_velocity(self, **kwargs):
        if kwargs['cancel_event'].is_set():
            raise InterruptedError('expired')
        self.calls.append(('velocity', kwargs))
        return {'sent': True, 'accepted': None}


def request(op='graph', payload=None, authority=None, deadline=10.1):
    return {'id': 1, 'op': op, 'payload': {} if payload is None else payload,
            'authority': authority, 'deadline_monotonic': deadline}


@pytest.mark.parametrize('op,payload', [
    ('state', {'state': 1}), ('state', {'state': 4}),
    ('state', {'state': 17}), ('velocity', {'vx': .1, 'vy': 0., 'yaw_rate': 0.}),
])
def test_worker_dispatches_only_documented_operations(op, payload):
    adapter = Adapter()
    result = dispatch_request(adapter, request(op, payload, {'offline': True}), clock=lambda: 10.)
    assert result['sent'] and result['accepted'] is None
    assert len(adapter.calls) == 1


def test_graph_is_readonly():
    adapter = Adapter()
    assert dispatch_request(adapter, request(), clock=lambda: 10.) == {'authority_proven': False}
    assert not adapter.calls


@pytest.mark.parametrize('deadline', [9., 10., 10.201, True, float('nan')])
def test_worker_rejects_expired_or_unbounded_request_before_adapter(deadline):
    adapter = Adapter()
    with pytest.raises(TimeoutError):
        dispatch_request(adapter, request('state', {'state': 1}, {}, deadline), clock=lambda: 10.)
    assert not adapter.calls


@pytest.mark.parametrize('op,payload,authority', [
    ('sdk_mode', {}, {}), ('gait', {}, {}), ('joints', {}, {}),
    ('state', {'state': 1, 'extra': 1}, {}), ('velocity', {'vx': .1}, {}),
    ('graph', {'state': 1}, None), ('graph', {}, {}),
])
def test_worker_rejects_unsupported_fields_and_actions(op, payload, authority):
    adapter = Adapter()
    with pytest.raises(ValueError):
        dispatch_request(adapter, request(op, payload, authority), clock=lambda: 10.)
    assert not adapter.calls


@pytest.mark.parametrize('raw', [b'{"id":1,"id":2}', b'{"x":NaN}', b'{bad'])
def test_private_json_is_strict(raw):
    with pytest.raises(ValueError):
        strict_json(raw)


def test_deadline_is_rechecked_by_adapter_after_type_loading():
    ticks = iter([10., 10.2])
    adapter = Adapter()
    with pytest.raises(InterruptedError):
        dispatch_request(adapter, request('state', {'state': 1}, {}), clock=lambda: next(ticks))
    assert not adapter.calls


FAKE_WORKER = r'''
import json,os,sys,time
mode=sys.argv[1]
if mode=='badready':
 print(json.dumps({'ready':False}),flush=True)
else:
 print(json.dumps({'ready':True}),flush=True)
for raw in sys.stdin:
 d=json.loads(raw)
 if mode=='silent':
  time.sleep(5)
 elif mode=='oversize':
  print('x'*70000,flush=True)
 elif mode=='mismatch':
  print(json.dumps({'id':d['id']+1,'ok':True,'result':{}}),flush=True)
 elif mode=='reject':
  print(json.dumps({'id':d['id'],'ok':False,'error_type':'ValueError'}),flush=True)
 else:
  print(json.dumps({'id':d['id'],'ok':True,'result':{'sent':d['op']!='graph','accepted':None,'operation':d['op']}}),flush=True)
'''


class Factory:
    def __init__(self, mode='okay'):
        self.mode = mode
        self.calls = []
        self.children = []

    def __call__(self, command, **kwargs):
        self.calls.append((command, kwargs.copy()))
        child = subprocess.Popen([sys.executable, '-u', '-c', FAKE_WORKER, self.mode], **kwargs)
        self.children.append(child)
        return child


def transport(factory=None, **kwargs):
    factory = factory or Factory()
    return DDSProcessTransport({'reader': {'domain_id': 0}}, process_factory=factory, **kwargs), factory


def test_default_constructor_is_dormant_and_cannot_start_motion():
    client, factory = transport()
    try:
        assert not factory.calls
        with pytest.raises(MotionProcessDisabled):
            client.start()
        with pytest.raises(MotionProcessDisabled):
            client.publish_state(1, authority={})
        assert not factory.calls
    finally:
        client.close()


def test_explicit_start_and_environment_separation(monkeypatch, tmp_path):
    profile = tmp_path/'vendor.xml'; profile.write_text('<test/>')
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/platform-loopback.xml')
    monkeypatch.setenv('FASTDDS_DEFAULT_PROFILES_FILE', '/platform.xml')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    factory = Factory()
    client = DDSProcessTransport({'reader': {'domain_id': 0, 'vendor_profile': str(profile)}},
        execution_enabled=True, process_factory=factory)
    try:
        assert client.start()['started']
        command, options = factory.calls[0]
        assert '--allow-execution' in command
        assert options['env']['ROS_DOMAIN_ID'] == '0'
        assert options['env']['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(profile)
        assert 'FASTDDS_DEFAULT_PROFILES_FILE' not in options['env']
        assert 'ROS_LOCALHOST_ONLY' not in options['env']
        result = client.publish_state(1, authority={'offline_fixture': True})
        assert result['sent'] and result['accepted'] is None
    finally:
        client.close()
    assert factory.children[0].poll() is not None


def test_enabled_constructor_still_requires_explicit_start():
    client, factory = transport(execution_enabled=True)
    try:
        with pytest.raises(MotionProcessNotStarted):
            client.publish_state(1, authority={})
        assert not factory.calls
    finally:
        client.close()


def test_readonly_probe_mode_has_no_execution_flag_or_commands():
    client, factory = transport()
    try:
        assert client.start(probe_only=True)['execution_enabled'] is False
        assert '--allow-execution' not in factory.calls[0][0]
        assert client.graph_snapshot()['operation'] == 'graph'
        with pytest.raises(MotionProcessDisabled):
            client.publish_velocity(authority={})
    finally:
        client.close()


@pytest.mark.parametrize('mode,error', [('silent', TimeoutError), ('oversize', ValueError),
    ('mismatch', ValueError), ('reject', MotionWorkerRejected)])
def test_timeout_malformed_or_rejected_session_is_not_reused(mode, error):
    client, factory = transport(Factory(mode), execution_enabled=True)
    client.start()
    begin = time.monotonic()
    with pytest.raises(error):
        client.publish_state(1, authority={})
    assert time.monotonic() - begin < 1
    assert client.status()['closed']
    assert factory.children[0].poll() is not None
    with pytest.raises(MotionProcessClosed):
        client.start()
    assert len(factory.calls) == 1


def test_bad_startup_ack_kills_child_without_command():
    client, factory = transport(Factory('badready'), execution_enabled=True)
    with pytest.raises(ValueError):
        client.start()
    assert factory.children[0].poll() is not None and client.status()['closed']


def test_cancellation_kills_inflight_worker_without_retry():
    client, factory = transport(Factory('silent'), execution_enabled=True)
    client.start()
    cancel = threading.Event()
    result = []
    def call():
        try:
            client.publish_velocity(.1, authority={}, cancel_event=cancel)
        except Exception as exc:
            result.append(type(exc))
    thread = threading.Thread(target=call)
    thread.start(); time.sleep(.03); cancel.set(); thread.join(1)
    assert not thread.is_alive() and result == [MotionProcessCancelled]
    assert factory.children[0].poll() is not None
    assert client.status()['closed'] and len(factory.calls) == 1


def test_cancel_before_send_does_not_start_or_close_dormant_worker():
    client, factory = transport(execution_enabled=True)
    cancel = threading.Event(); cancel.set()
    with pytest.raises(MotionProcessCancelled):
        client.publish_state(1, authority={}, cancel_event=cancel)
    assert not factory.calls
    client.close()


def test_closed_transport_cannot_restart():
    client, factory = transport(execution_enabled=True)
    client.close()
    with pytest.raises(MotionProcessClosed):
        client.start()
    assert not factory.calls


def test_parent_requests_and_responses_are_bounded():
    client, factory = transport(execution_enabled=True)
    client.start()
    with pytest.raises(ValueError):
        client.publish_state(1, authority={'x': 'y'*9000})
    assert client.status()['closed'] and factory.children[0].poll() is not None


def test_worker_serve_rejects_oversized_input_without_dispatch():
    read_fd, write_fd = os.pipe()
    adapter = Adapter()
    import io
    output = io.BytesIO()
    error = []
    def worker():
        try:
            serve(adapter, lambda: None, stdin=read_fd, stdout=output)
        except Exception as exc:
            error.append(type(exc))
    thread = threading.Thread(target=worker); thread.start()
    os.write(write_fd, b'x'*8192)
    os.write(write_fd, b'x\n')
    thread.join(1); os.close(write_fd); os.close(read_fd)
    assert error == [ValueError] and not adapter.calls
    assert json.loads(output.getvalue()) == {'ready': True}
