"""Private bounded command worker for one independent vendor DDS process.

Construction/import does not initialize ROS. The CLI creates a domain-0 node;
without an execution flag it can only inspect the graph. Enabled startup
prepares publishers for discovery without sending commands.
EOF/close only destroys this worker's publishers, and is not a robot stop.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import time

try:
    from .motion_dds import DDSCommandAdapter, MessageCodec
except ImportError:
    from motion_dds import DDSCommandAdapter, MessageCodec

MAX_REQUEST_BYTES = 8192
MAX_RESPONSE_BYTES = 65536
RPC_BUDGET_S = .2


class _DeadlineCancellation:
    def __init__(self, deadline, clock):
        self.deadline, self.clock = deadline, clock

    def is_set(self):
        return self.clock() >= self.deadline


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate request field')
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))


def dispatch_request(adapter, request, *, clock=time.monotonic):
    """Pure dispatcher, injectable for offline tests; never initializes ROS."""
    if not isinstance(request, dict) or set(request) != {'id', 'op', 'payload', 'deadline_monotonic'}:
        raise ValueError('invalid private motion request shape')
    request_id = request['id']
    if type(request_id) is not int or not 1 <= request_id <= 2**53 - 1:
        raise ValueError('invalid private request id')
    deadline, now = request['deadline_monotonic'], clock()
    if isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or not math.isfinite(deadline) or not now < deadline <= now + RPC_BUDGET_S:
        raise TimeoutError('private motion request expired')
    operation, payload = request['op'], request['payload']
    if not isinstance(payload, dict):
        raise ValueError('private motion payload must be an object')
    if operation == 'graph':
        if payload:
            raise ValueError('graph query does not accept motion arguments')
        return adapter.graph_snapshot()
    if operation == 'state':
        if set(payload) != {'state'}:
            raise ValueError('state command requires only state')
        # The private adapter checks execution and deadline before publishing.
        return adapter.publish_state(payload['state'], cancel_event=_DeadlineCancellation(deadline, clock))
    if operation == 'velocity':
        if set(payload) != {'vx', 'vy', 'yaw_rate'}:
            raise ValueError('velocity command requires three physical axes')
        return adapter.publish_velocity(**payload, cancel_event=_DeadlineCancellation(deadline, clock))
    raise ValueError('unknown private motion operation')


def _write(record, stream):
    raw = json.dumps(record, allow_nan=False, separators=(',', ':')).encode() + b'\n'
    if len(raw) > MAX_RESPONSE_BYTES:
        raise ValueError('private worker response exceeds bound')
    stream.write(raw)
    stream.flush()


def serve(adapter, spin_once, *, stdin, stdout, clock=time.monotonic):
    """Bounded byte-stream server. stdin is a private pipe, never a network port."""
    pending = bytearray()
    _write({'ready': True}, stdout)
    while True:
        spin_once()
        readable, _, _ = select.select([stdin], [], [], .02)
        if not readable:
            continue
        chunk = os.read(stdin, 4096)
        if not chunk:
            return
        pending.extend(chunk)
        while b'\n' in pending:
            raw, _, rest = pending.partition(b'\n')
            pending = bytearray(rest)
            if len(raw) > MAX_REQUEST_BYTES or not raw:
                raise ValueError('private worker request exceeds bound')
            request = strict_json(raw)
            wire_id = request.get('id') if isinstance(request, dict) else None
            if type(wire_id) is not int or not 1 <= wire_id <= 2**53 - 1:
                raise ValueError('invalid private worker request id')
            try:
                result = dispatch_request(adapter, request, clock=clock)
                reply = {'id': wire_id, 'ok': True, 'result': result}
            except Exception as exc:
                # No arbitrary exception text, environment or private paths.
                reply = {'id': wire_id, 'ok': False, 'error_type': type(exc).__name__}
            _write(reply, stdout)
        if len(pending) > MAX_REQUEST_BYTES:
            raise ValueError('unterminated private request exceeds bound')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--allow-execution', action='store_true')
    args = parser.parse_args(argv)
    # The parent uses reader_environment before spawning. A second check keeps
    # accidental direct invocation from joining the platform's DDS domain.
    if os.environ.get('ROS_DOMAIN_ID') != '0':
        raise RuntimeError('motion worker requires explicitly isolated vendor domain 0')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('s10_pro_motion_commands', enable_rosout=False, start_parameter_services=False)
    codec = MessageCodec()
    if args.allow_execution:
        codec.prepare()
    adapter = DDSCommandAdapter(node, execution_enabled=args.allow_execution,
        codec=codec)
    try:
        if args.allow_execution:
            adapter.prepare_state_channel()
        serve(adapter, lambda: rclpy.spin_once(node, timeout_sec=0), stdin=sys.stdin.buffer.fileno(), stdout=sys.stdout.buffer)
    finally:
        adapter.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
