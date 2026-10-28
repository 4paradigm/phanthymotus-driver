"""Read-only high-level DDS preflight: message metadata and graph endpoints.

No command publisher, control SDK, service call, or mode change is created.
Graph discovery alone does not establish ownership, operating mode or deadman.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

try:
    from .preflight import collect_contract, compare_contracts, _message_getter, _load_reference
except ImportError:
    from preflight import collect_contract, compare_contracts, _message_getter, _load_reference

SPECS = {
    'state_command': {'topic': '/MOTION_STATE', 'type': 'drdds/msg/MotionState'},
    'velocity_command': {'topic': '/NAV_CMD', 'type': 'drdds/msg/NavCmd'},
    'feedback': {'topic': '/MOTION_INFO', 'type': 'drdds/msg/MotionInfo'},
}
SOURCE = 'https://alidocs.dingtalk.com/i/p/OlnXRnDldg3J7XLp/docs/dQPGYqjpJYgQaZdQtBNLa6a6Wakx1Z5N?dontjump=true'


def observe_graph(node, *, seconds=3., spin_once=None, clock=time.monotonic):
    if not isinstance(seconds, (int, float)) or isinstance(seconds, bool) or not math.isfinite(seconds) or not .1 <= seconds <= 10:
        raise ValueError('bounded discovery duration must be .1..10 seconds')
    end = clock() + seconds
    while clock() < end:
        spin_once(node, timeout_sec=min(.1, max(0, end - clock())))
    rows = {}
    for key, spec in SPECS.items():
        def endpoints(method):
            return [{'node': endpoint.node_name, 'namespace': endpoint.node_namespace,
                     'type': endpoint.topic_type} for endpoint in method(spec['topic'])][:64]
        publishers = endpoints(node.get_publishers_info_by_topic)
        subscribers = endpoints(node.get_subscriptions_info_by_topic)
        rows[key] = {**spec, 'publishers': publishers, 'subscribers': subscribers,
            'expected_type_present': any(e['type'] == spec['type'] for e in publishers + subscribers)}
    return rows


def report(contract, graph=None):
    return {'schema': 's10_pro.motion_preflight/1', 'read_only': True,
        'transport': 'ros2_dds', 'tls_required': False, 'source': SOURCE,
        'command_publishers_created': 0, 'robot_commands_sent': 0,
        'contract': contract, 'graph': graph,
        'execution_available': False, 'operating_mode': 'unknown',
        'control_ownership': 'unknown', 'robot_deadman': 'unknown',
        'notes': ['Graph discovery is not proof of ownership or permission.',
                  'DDS stand needs state 1 feedback before state 17; no automatic mode switch.',
                  'NAV_CMD is navigation-mode only and documented at 10 Hz.',
                  'Firmware/handheld version compatibility and loss-of-command stopping remain unverified.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph', action='store_true', help='join current ROS environment for read-only graph observation')
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--public-contract', action='store_true')
    parser.add_argument('--compare-contract', type=Path)
    args = parser.parse_args(argv)
    try:
        contract = collect_contract(SPECS, _message_getter())
        graph = None
        if args.graph:
            if os.environ.get('ROS_DOMAIN_ID', '0') != '0':
                raise ValueError('motion graph preflight requires the explicit vendor domain 0')
            import rclpy
            rclpy.init(args=[])
            node = rclpy.create_node('s10_motion_readonly_preflight', enable_rosout=False, start_parameter_services=False)
            try:
                graph = observe_graph(node, seconds=args.seconds, spin_once=rclpy.spin_once)
            finally:
                node.destroy_node()
                rclpy.shutdown()
        result = contract if args.public_contract else report(contract, graph)
        code = 0
        if args.compare_contract:
            result['comparison'] = compare_contracts(contract, _load_reference(args.compare_contract))
            code = 0 if result['comparison']['compatible'] else 2
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return code
    except Exception as exc:
        print(json.dumps({'schema': 's10_pro.motion_preflight/1', 'status': 'error', 'error_type': type(exc).__name__}))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
