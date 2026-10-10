#!/usr/bin/env python3
"""Actual Jetson HTTP check; cannot send robot motion commands."""
import json
import sys
import time
import urllib.request

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def wait_ready(timeout=20):
    deadline = time.monotonic() + timeout
    last_error = 'Service has not started'
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError('Service was not ready within ' + str(timeout)
                               + ' seconds: ' + last_error
                               + '; inspect the limx-tron2 service logs')
        try:
            with opener.open('http://127.0.0.1:15791/health',
                             timeout=min(8, remaining)) as resp:
                health = json.load(resp)
            if not isinstance(health, dict) or health.get('pose_valid') is not True:
                raise ValueError('Pose validation is not ready')
            if not health.get('registered_mcp_id'):
                raise ValueError('Core registration is pending')
            return health
        except (OSError, ValueError, KeyError, TypeError) as exc:
            last_error = str(exc)
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(1, remaining))


def rpc(action, **metadata):
    payload = {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
               'params': {'name': 'reset_pose', 'arguments': {'action': action, **metadata}}}
    req = urllib.request.Request('http://127.0.0.1:15791/mcp',
          data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json'})
    with opener.open(req, timeout=12) as resp:
        data = json.load(resp)
    if 'error' in data or data.get('result', {}).get('isError'):
        raise ValueError(str(data))
    return json.loads(data['result']['content'][0]['text'])


if __name__ == '__main__':
    try:
        health = wait_ready()
        if health.get('operation_in_progress') or health.get('platform_active') or health.get('armed') or health.get('unconfirmed_operation'):
            raise ValueError('Check requires platform control off and no active or unconfirmed motion')
        lifecycle = rpc('start', instance_id='compat-check-card')
        if lifecycle.get('state') != 'ready' or lifecycle.get('motion_sent') is not False:
            raise ValueError('Canvas lifecycle check failed')
        rpc('info', instance_id='compat-check-card')
        result = rpc('preview', instance_id='compat-check-card')
        rpc('stop', instance_id='compat-check-card')
        if result.get('motion_enabled') is not False or result.get('motion_sent') is not False:
            raise ValueError('Unexpected motion capability')
        print('PASS: pose, fresh state, Core registration and preview')
        print('Max joint change (degrees):', result['max_joint_change_deg'])
        print('Planned duration (seconds):', result['duration_seconds'])
        print('Reported working mode:', result.get('working_mode'))
        if health.get('version') != '0.3.0' or health.get('motion_capability') is not True:
            raise ValueError('The expected control version is not running')
        print('Control implementation is present; no motion sent by this check.')
        print('PASS: canvas instance_id with start, info, preview and stop')
        print('Platform authorization: start/stop lifecycle; no local arm required')
        print('Unconfirmed operation:', health.get('unconfirmed_operation'))
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        print('CHECK FAILED:', str(exc), file=sys.stderr)
        sys.exit(1)
