#!/usr/bin/env python3
"""Read-only MCP/HTTP readiness check; does not open a robot command socket."""
import json
import time
import urllib.error
import urllib.request

BASE = 'http://127.0.0.1:15791'
VERSION = '0.5.3'
CARDS = {'joint_state', 'joint_angles', 'battery_state'}


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
    samples = {}
    for name in CARDS:
        result = rpc('tools/call', {'name': name, 'arguments': {
            'action': 'info', 'instance_id': 'readonly-check', '_trace_id': 'readonly-check-trace'}})
        if result.get('isError'):
            raise ValueError('Sensor info request failed: ' + name)
        samples[name] = json.loads(result['content'][0]['text'])['sample']
    return samples


def run_check(timeout=20):
    deadline = time.monotonic() + timeout
    reason = 'Service is not ready'
    while time.monotonic() < deadline:
        try:
            with OPENER.open(BASE + '/health', timeout=min(5, max(.1, deadline-time.monotonic()))) as response:
                health = json.load(response)
            if health.get('version') != VERSION:
                raise ValueError('Wrong driver version on port 15791')
            if not health.get('registered_mcp_id'):
                raise ValueError('Core registration is pending; check certificate and hostname')
            break
        except (OSError, ValueError, KeyError, TypeError) as exc:
            reason = str(exc)[:200]
            time.sleep(min(.5, max(0, deadline-time.monotonic())))
    else:
        raise ValueError(reason)
    tools = rpc('tools/list')['tools']
    names = {tool['name'] for tool in tools}
    expected = {**dict.fromkeys(CARDS, 'sensor'), 'arm_control': 'actuator', 'model': 'resource'}
    if not CARDS <= names or names - set(expected) or any(tool['type'] != expected[tool['name']] for tool in tools):
        raise ValueError('Expected three sensors, optional arm_control and model resource')
    if 'model' not in names:
        raise ValueError('Skeleton model resource is missing')
    model_reply = rpc('tools/call', {'name': 'model', 'arguments': {}})
    model_data = json.loads(model_reply['content'][0]['text'])
    from skeleton import JOINT_NAMES
    import xml.etree.ElementTree as ET
    model_names = {joint.get('name') for joint in ET.fromstring(model_data['urdf']).findall('joint')}
    if model_reply.get('isError') or not set(JOINT_NAMES) <= model_names:
        raise ValueError('Skeleton joint names do not match model')
    joint_tool = next(tool for tool in tools if tool['name'] == 'joint_state')
    if joint_tool['topic_out'][0]['format'] != 'sensor/skeleton':
        raise ValueError('Joint card must use skeleton format')
    for name in CARDS:
        for action in ('start', 'info', 'stop'):
            result = rpc('tools/call', {'name': name,
                'arguments': {'action': action, 'instance_id': 'readonly-check',
                              '_trace_id': 'readonly-check-trace'}})
            if result.get('isError'):
                raise ValueError('Sensor lifecycle failed: ' + name)
    for name in names & {'arm_control'}:
        for action in ('info', 'start', 'stop'):
            reply = rpc('tools/call', {'name': name, 'arguments': {'action': action,
                '_trace_id': 'readonly-check-trace', 'instance_id': 'readonly-check'}})
            if reply.get('isError'):
                raise ValueError('Actuator lifecycle failed: ' + name)
            data = json.loads(reply['content'][0]['text'])
            if data.get('motion_sent') is not False:
                raise ValueError('Lifecycle must not send motion')
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
    from skeleton import skeleton_stream
    skeleton_data = skeleton_stream(joint)
    if not skeleton_data['status']['available'] or len(skeleton_data['joints']) != 16:
        raise ValueError('Unsupported skeleton joint layout or names')
    from joint_angles import angle_sample, ANGLE_FIELDS
    angles = after['joint_angles']
    if set(angles['data']) != set(ANGLE_FIELDS) or len(angles['data']) != 16:
        raise ValueError('Expected sixteen joint-angle fields')
    # Separate info calls may straddle an incoming response. Compare only when
    # both samples refer to the same received frame; never assume static arms.
    if angles['received_at'] == joint['received_at'] and angles['data'] != angle_sample(joint)['data']:
        raise ValueError('Joint-angle values do not match the skeleton feedback')
    if angles['received_at'] <= before['joint_angles']['received_at']:
        raise ValueError('Joint-angle feedback receipt did not update')
    battery = after['battery_state']['data']
    if set(battery) != {'schema_version', 'soc', 'voltage', 'current', 'power',
                       'temperature', 'cycle', 'units', 'validity',
                       'power_status', 'current_updated_at_raw', 'temperature_channels_raw',
                       'temperature_policy'}:
        raise ValueError('Unexpected battery fields')
    if battery['schema_version'] != 3 or len(battery['temperature_channels_raw']) != 5 \
            or battery['power_status'] != 'estimated_assuming_mV_mA':
        raise ValueError('Unexpected battery schema, channels or power interpretation')
    print('PASS: three sensors including 16 joint-angle tiles, skeleton/model and arm actions; motion not tested; Core trace metadata; MCP lifecycle; Core registration; periodic replies')
    print('Joint count:', values['joint_count'])
    print('Joint labels:', values['name_source'])
    print('First joint positions (degrees):', values['positions_deg'][:3])
    print('Left/right elbow feedback (degrees):', values['positions_deg'][3], values['positions_deg'][10])
    print('Battery (%):', battery['soc'])
    print('Voltage raw/current magnitude (vendor units unverified):', battery['voltage'], battery['current'])
    print('Estimated power (W, assuming mV/mA):', battery['power'])
    print('Temperature (maximum valid vendor raw channel, not Celsius):', battery['temperature'])
    print('Charge cycles:', battery['cycle'])
    print('Read-only. Measurement timestamp clock is unverified; receipt age is reported separately.')


if __name__ == '__main__':
    try:
        run_check()
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit('CHECK FAILED: ' + str(exc))
