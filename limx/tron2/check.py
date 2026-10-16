#!/usr/bin/env python3
"""Read-only MCP/HTTP readiness check; does not open a robot command socket."""
import json
import time
import urllib.error
import urllib.request

BASE = 'http://127.0.0.1:15791'
VERSION = '0.7.7'
CARDS = {'joint_state'}
from camera import CAMERA_SOURCES


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())


def rpc(method, params=None):
    req = urllib.request.Request(BASE + '/mcp', data=json.dumps({
        'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params or {}}).encode(),
        headers={'Content-Type': 'application/json'})
    with OPENER.open(req, timeout=12) as response:
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


def check_camera_bus(topics):
    """Optional actual DDS subscriber proof, no frame files or motion calls."""
    try:
        import rclpy
    except ImportError as exc:
        raise ValueError('Camera DDS check requires the ROS environment; source /opt/ros/humble/setup.bash first') from exc
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from camera import jpeg_size
    # Match the driver's authoritative mounted FastDDS profile.
    import os
    os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
    rclpy.init()
    node = None
    frames = {topic: [] for topic in topics}
    try:
        node = rclpy.create_node('tron2_camera_readonly_check')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        def receive(msg, topic):
            if len(frames[topic]) < 2:
                frames[topic].append(msg)
        for topic in topics:
            node.create_subscription(CompressedImage, topic, lambda msg, topic=topic: receive(msg, topic), qos)
        deadline = time.monotonic() + 5
        while any(len(messages) < 2 for messages in frames.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        missing = [topic for topic, messages in frames.items() if len(messages) < 2]
        if missing:
            raise ValueError('No continuous JPEG frames reached local DDS for: ' + ', '.join(missing))
        if any(msg.format != 'jpeg' for messages in frames.values() for msg in messages):
            raise ValueError('Unexpected ROS image encoding')
        return {topic: jpeg_size(bytes(messages[-1].data)) for topic, messages in frames.items()}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def run_check(timeout=20, camera=False):
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
    expected = {**dict.fromkeys(CARDS | set(CAMERA_SOURCES), 'sensor'), 'arm_control': 'actuator', 'move_head': 'actuator', 'gripper': 'actuator', 'model': 'resource'}
    if not CARDS <= names or names - set(expected) or any(tool['type'] != expected[tool['name']] for tool in tools):
        raise ValueError('Expected skeleton sensor, optional camera/arm_control and model resource')
    camera_tools = {tool['name']: tool for tool in tools if tool['name'] in CAMERA_SOURCES}
    for name, camera_tool in camera_tools.items():
        if camera_tool['topic_out'] != [{'topic': CAMERA_SOURCES[name][0], 'format': 'image/jpeg'}] \
                or camera_tool['inputSchema']['properties']['action']['enum'] != ['info', 'start', 'stop']:
            raise ValueError('Unexpected camera topic, format or lifecycle')
        camera_info = rpc('tools/call', {'name': name, 'arguments': {'action': 'info'}})
        if camera_info.get('isError') or json.loads(camera_info['content'][0]['text']).get('read_only') is not True:
            raise ValueError('Camera information request failed')
    if 'model' not in names:
        raise ValueError('Skeleton model resource is missing')
    model_reply = rpc('tools/call', {'name': 'model', 'arguments': {}})
    model_data = json.loads(model_reply['content'][0]['text'])
    from skeleton import JOINT_NAMES
    import xml.etree.ElementTree as ET
    model_names = {joint.get('name') for joint in ET.fromstring(model_data['urdf']).findall('joint')}
    if model_reply.get('isError') or not set(JOINT_NAMES) <= model_names:
        raise ValueError('Skeleton joint names do not match model')
    display_joints = [joint for joint in ET.fromstring(model_data['urdf']).findall('joint')
                      if joint.get('name', '').startswith('display_hand_gripper_')]
    if len(display_joints) != 8 or any(joint.get('type') != 'fixed' for joint in display_joints):
        raise ValueError('Fixed left/right gripper display outlines are missing')
    joint_tool = next(tool for tool in tools if tool['name'] == 'joint_state')
    if joint_tool['topic_out'][0]['format'] != 'sensor/skeleton':
        raise ValueError('Joint card must use skeleton format')
    arm_tool = next((tool for tool in tools if tool['name'] == 'arm_control'), None)
    if arm_tool and (arm_tool['inputSchema'].get('x-completion') != {
            'actions': ['move', 'reset'], 'timeout': 120}
            or arm_tool['inputSchema'].get('x-resource') != ['arm_l', 'arm_r']
            or arm_tool['inputSchema']['properties']['action']['enum'] != ['move', 'reset']
            or set(arm_tool['inputSchema']['x-action-params']) != {'move', 'reset'}):
        raise ValueError('Arm completion tracking or physical resources are missing')
    head_tool = next((tool for tool in tools if tool['name'] == 'move_head'), None)
    if head_tool:
        schema = head_tool['inputSchema']; props = schema['properties']
        if set(props) != {'pitch_deg','yaw_deg','time'} or schema.get('x-completion') != {'timeout':120} or \
                schema.get('x-resource') != ['head','arm_l','arm_r'] or \
                (props['pitch_deg'].get('minimum'),props['pitch_deg'].get('maximum')) != (-60,45) or \
                (props['yaw_deg'].get('minimum'),props['yaw_deg'].get('maximum')) != (-88,88):
            raise ValueError('Head limits, schema or ACP tracking differ from the reviewed card')
    for name in CARDS:
        for action in ('start', 'info', 'stop'):
            result = rpc('tools/call', {'name': name,
                'arguments': {'action': action, 'instance_id': 'readonly-check',
                              '_trace_id': 'readonly-check-trace'}})
            if result.get('isError'):
                raise ValueError('Sensor lifecycle failed: ' + name)
    motion_readiness = None
    for name in names & {'arm_control', 'move_head', 'gripper'}:
        for action in ('info', 'start', 'stop'):
            reply = rpc('tools/call', {'name': name, 'arguments': {'action': action,
                '_trace_id': 'readonly-check-trace', 'instance_id': 'readonly-check'}})
            if reply.get('isError'):
                raise ValueError('Actuator lifecycle failed: ' + name)
            data = json.loads(reply['content'][0]['text'])
            if name == 'arm_control' and action == 'info':
                motion_readiness = data.get('motion_readiness')
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
    print('PASS: skeleton/model with fixed gripper outlines and move/reset with ACP completion; motion not tested; Core trace metadata; MCP lifecycle; Core registration; periodic replies')
    print('Arm motion readiness:', motion_readiness)
    print('Joint count:', values['joint_count'])
    print('Joint labels:', values['name_source'])
    print('First joint positions (degrees):', values['positions_deg'][:3])
    print('Left/right elbow feedback (degrees):', values['positions_deg'][3], values['positions_deg'][10])
    if camera:
        if not camera_tools:
            raise ValueError('Camera card is not enabled')
        first = {}
        for name in camera_tools:
            reply = rpc('tools/call', {'name': name, 'arguments': {'action': 'start'}})
            if reply.get('isError'):
                raise ValueError('Camera start failed: ' + name + ': ' + reply['content'][0]['text'])
            first[name] = json.loads(reply['content'][0]['text'])
        time.sleep(1)
        latest = {name: json.loads(rpc('tools/call', {'name': name, 'arguments': {'action': 'info'}})['content'][0]['text'])
                  for name in camera_tools}
        for name, info in latest.items():
            if not info['available'] or info['frames_published'] <= first[name]['frames_published']:
                raise ValueError('Camera frames did not continue publishing: ' + name)
        dimensions = check_camera_bus([CAMERA_SOURCES[name][0] for name in camera_tools])
        for name, info in latest.items():
            width, height = dimensions[CAMERA_SOURCES[name][0]]
            if (width, height) != (info['width'], info['height']):
                raise ValueError('Camera dimensions changed between metadata and DDS check; repeat the probe')
            print('Camera PASS:', name, 'continuous JPEG frames received through local DDS; resolution:', width, height)
        print('Cameras remain active for platform viewing; stop each camera lifecycle when finished.')
    elif camera_tools:
        print('Camera cards ready; live video not tested. Use --camera to start enabled sources and verify publication.')
    print('Read-only. Measurement timestamp clock is unverified; receipt age is reported separately.')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', action='store_true', help='Start enabled read-only RGB streams, verify each DDS topic, and leave them active')
    args = parser.parse_args()
    try:
        run_check(camera=args.camera)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit('CHECK FAILED: ' + str(exc))
