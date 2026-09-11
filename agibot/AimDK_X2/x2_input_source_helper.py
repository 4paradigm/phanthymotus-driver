#!/usr/bin/env python3
"""Isolated X2 SetMcInputSource client.

Run as a child of the driver so vendor/FastDDS service bugs cannot crash or
starve the long-running MCP process.
"""

from __future__ import annotations

import argparse
import json
import os
import time

try:
    from common import logsafe
    logsafe.install()
except Exception as exc:  # pragma: no cover
    import sys
    sys.stderr.write(f"[x2-input-helper] logsafe unavailable ({exc})\n")

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import SetMcInputSource

from common.vendor_runtime import jsonable


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action", type=int, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--priority", type=int, required=True)
    parser.add_argument("--timeout-ms", type=int, required=True)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node(f"agibot_x2_input_source_{args.action}")
    client = node.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
    try:
        if not client.wait_for_service(timeout_sec=min(3.0, args.timeout_sec)):
            raise TimeoutError("service unavailable")
        request = SetMcInputSource.Request()
        request.request.header.stamp = node.get_clock().now().to_msg()
        request.action.value = args.action
        request.input_source.name = args.name
        request.input_source.priority = args.priority
        request.input_source.timeout = args.timeout_ms
        future = client.call_async(request)
        deadline = time.monotonic() + args.timeout_sec
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            # Some vendor FastDDS builds segfault while tearing down a client
            # after an unanswered request.  Let the kernel reap this tiny
            # helper; the parent will treat 124 as a retryable timeout.
            import sys
            print(f"service response timed out after {args.timeout_sec:.1f}s", file=sys.stderr, flush=True)
            os._exit(124)
        result = future.result()
        print(json.dumps(jsonable(result.response), ensure_ascii=False), flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
