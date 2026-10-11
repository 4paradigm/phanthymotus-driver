#!/usr/bin/env python3
"""Read-only MCP/HTTP readiness check; does not open a robot command socket."""
import json
import time
import urllib.error
import urllib.request

BASE = 'http://127.0.0.1:15791'
VERSION = '0.4.0'
CARDS = {'joint_state', 'robot_status', 'battery_state'}


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())


def rpc(method, params=None):
    req = urllib.request.Request(BASE + '/mcp', data=json.dumps({
        'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params or {}}).encode(),
        headers={'Content-Type': 'application/json'})
    with OPENER.open(req, timeout=5) as response:
        body = json.load(response)
    if 'error' in body:
        raise ValueError('MCP request rejected')
    return body['result']


def read_samples():
    result = rpc('tools/call', {'name': 'read_state', 'arguments': {'action': 'read'}})
    if result.get('isError'):
        raise ValueError('Read-state request failed')
    return json.loads(result['content'][0]['text'])['samples']


def run_check(timeout=20):
    deadline = time.monotonic() + timeout
    reason = 'Service is not ready'
    while time.monotonic() < deadline:
        try:
            with OPENER.open(BASE + '/health', timeout=min(5, max(.1, deadline-time.monotonic()))) as response:
                health = json.load(response)
            if health.get('version') != VERSION or health.get('read_only') is not True:
                raise ValueError('Wrong driver version or a non-sensor service occupies port 15791')
            if not health.get('registered_mcp_id'):
                raise ValueError('Core registration is pending; check certificate and hostname')
            break
        except (OSError, ValueError, KeyError, TypeError) as exc:
            reason = str(exc)[:200]
            time.sleep(min(.5, max(0, deadline-time.monotonic())))
    else:
        raise ValueError(reason)
    tools = rpc('tools/list')['tools']
    if {tool['name'] for tool in tools} != CARDS | {'read_state'} or any(
            tool['type'] not in ('sensor', 'resource') for tool in tools):
        raise ValueError('Unexpected card set; no actuators may be exposed')
    for name in CARDS:
        for action in ('start', 'info', 'stop'):
            result = rpc('tools/call', {'name': name,
                'arguments': {'action': action, 'instance_id': 'readonly-check'}})
            if result.get('isError'):
                raise ValueError('Sensor lifecycle failed: ' + name)
    before = read_samples()
    time.sleep(1)
    after = read_samples()
    for name in CARDS:
        if not before[name]['available'] or not after[name]['available']:
            raise ValueError('Sensor unavailable: ' + name)
    joint = after['joint_state']
    if joint['received_at'] <= before['joint_state']['received_at']:
        raise ValueError('Joint response receipt did not update')
    values = joint['data']
    print('PASS: only sensor/resource cards; MCP lifecycle; Core registration; periodic replies')
    print('Joint count:', values['joint_count'])
    print('Joint labels:', values['name_source'])
    print('First joint positions (degrees):', values['positions_deg'][:3])
    print('Battery (%):', after['battery_state']['data']['battery_percent'])
    print('Read-only. Measurement timestamp clock is unverified; receipt age is reported separately.')


if __name__ == '__main__':
    try:
        run_check()
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit('CHECK FAILED: ' + str(exc))
